/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

// Kvaser CAN driver module for Linux
// Hardware independent parts
//

#if LINUX

//--------------------------------------------------
// NOTE! module_versioning HAVE to be included first
#include "module_versioning.h"
//--------------------------------------------------

#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/timer.h>
#   if LINUX_2_6
#       include <linux/workqueue.h>
#   else
#       include <linux/tqueue.h>
#   endif
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>


#else // win32
#   include "linuxErrors.h"
// It's not really good to require this, I think, but...
#   include "driver.h"
#endif // end win32

// Kvaser definitions
#include "vcanevt.h"
#include "vcan_ioctl.h"
#include "kcan_ioctl.h"
#if LINUX
#   include "hwnames.h"
#endif
#include "osif_functions_kernel.h"
#if !LINUX
#   include "osif_user.h"
#endif
#include "queue.h"
#include "VCanOsIf.h"
#include "debug.h"


#if LINUX
    MODULE_LICENSE("GPL");
    MODULE_AUTHOR("KVASER");
#endif

#if LINUX && !LINUX_2_6
    EXPORT_NO_SYMBOLS;
#endif

#if LINUX
#  if defined PCMCIA_DEBUG || defined PCICAN_DEBUG || defined USBCAN_DEBUG || defined LEAF_DEBUG
#     if (PCMCIA_DEBUG > PCICAN_DEBUG) && (PCMCIA_DEBUG > USBCAN_DEBUG) && (PCMCIA_DEBUG > LEAF_DEBUG)
           static int debug_levell = PCMCIA_DEBUG;
#       elif (PCICAN_DEBUG > USBCAN_DEBUG) && (PCMCIA_DEBUG > LEAF_DEBUG)
           static int debug_levell = PCICAN_DEBUG;
#       elif (USBCAN_DEBUG > LEAF_DEBUG)
           static int debug_levell = USBCAN_DEBUG;
#       else
           static int debug_levell = LEAF_DEBUG;
#     endif
#     if !LINUX_2_6
         MODULE_PARM(debug_levell, "i");
#     else
         MODULE_PARM_DESC(debug_levell, "Common debug level");
         module_param(debug_levell, int, 0644);
#     endif
#     define DEBUGPRINT(n, arg)    if (debug_levell >= (n)) { DEBUGOUT(n, arg); }
#  else
#     define DEBUGPRINT(n, args...)
#  endif
#else // !LINUX
#  if defined PCMCIA_DEBUG || defined USBCAN_DEBUG || defined LEAF_DEBUG
#     pragma message("DEBUG")
#     if (PCMCIA_DEBUG > USBCAN_DEBUG) && (PCMCIA_DEBUG > LEAF_DEBUG)
           static int debug_levell = PCMCIA_DEBUG;
#       elif (USBCAN_DEBUG > LEAF_DEBUG)
           static int debug_levell = USBCAN_DEBUG;
#       else
           static int debug_levell = LEAF_DEBUG;
#     endif
#     define __FUNCTION__          TEXT("")
#     define DEBUGPRINT(n, arg)    DEBUGOUT(debug_levell >= (n), arg)
#  else
#     define DEBUGPRINT(n, arg)
#  endif
#endif // LINUX

VCanDriverData driverData;
VCanCardData   *canCards = NULL;
OS_IF_LOCK     canCardsLock;

#if !LINUX
WaitNode waitNodes[32];
WaitNode *freeWaitNode;
#define EVENT_HANDLE_MASK 0x12345678
#endif

//======================================================================
// File operations...
//======================================================================
#if LINUX

static int          vCanClose(struct inode *inode, struct file *filp);
#if !defined(HAVE_UNLOCKED_IOCTL)
static int          vCanIOCtl(struct inode *inode, struct file *filp,
                              unsigned int cmd, unsigned long arg);
#endif
static long         vCanIOCtl_unlocked(struct file *filp,
                                       unsigned int cmd, unsigned long arg);
static int          vCanOpen(struct inode *inode, struct file *filp);
static unsigned int vCanPoll(struct file *filp, poll_table *wait);

struct file_operations fops = {
  .poll    = vCanPoll,
  .open    = vCanOpen,
  .release = vCanClose,
#if defined(HAVE_UNLOCKED_IOCTL)
  .unlocked_ioctl = vCanIOCtl_unlocked,
#else
  .ioctl   = vCanIOCtl,
#endif
#if defined(HAVE_COMPAT_IOCTL)
  .compat_ioctl   = vCanIOCtl_unlocked,
#endif
             // Fills rest with NULL entries
};
#else
#           pragma message("qqq")
#endif


//======================================================================
// Time
//======================================================================

unsigned long vCanTime (VCanCardData *vCard)
{
  struct timeval tv;
  os_if_do_get_time_of_day(&tv);

  tv.tv_usec -= driverData.startTime.tv_usec;
  tv.tv_sec  -= driverData.startTime.tv_sec;

  return tv.tv_usec / (long)vCard->usPerTick +
         (1000000 / vCard->usPerTick) * tv.tv_sec;
}

//======================================================================
//  Calculate queue length
//======================================================================

unsigned long getQLen (unsigned long head, unsigned long tail,
                       unsigned long size)
{
  return (head < tail ? head - tail + size : head - tail);
}

//======================================================================
//  Discard send queue
//======================================================================

int vCanFlushSendBuffer (VCanChanData *chd)
{
  queue_reinit(&chd->txChanQueue);

  return VCAN_STAT_OK;
}

//======================================================================
//  Discard recieve queue
//======================================================================

int vCanFlushReceiveBuffer (VCanOpenFileNode *fileNodePtr)
{
  fileNodePtr->rcv.bufTail = 0;
  fileNodePtr->rcv.bufHead = 0;

  return VCAN_STAT_OK;
}

//======================================================================
//  Pop rx queue
//======================================================================
int vCanPopReceiveBuffer (VCanReceiveData *rcv)
{
  #if !LINUX
  int isEmpty;
  os_if_spin_lock(&rcv->lock);
  #endif

  if (rcv->bufTail >= rcv->size - 1) {
    rcv->bufTail = 0;
  }
  else {
    rcv->bufTail++;
  }

  if ((rcv->bufTail % 10) == 0) {
    DEBUGPRINT(4, (TXT("RXpop(%d)\n"), rcv->bufTail));
  }

  #if !LINUX
  isEmpty = rcv->bufTail == rcv->bufHead;

  if (isEmpty) {
    #if DEBUG
    rcv->lastEmpty = rcv->bufTail;
    #endif
    os_if_clear_event(&rcv->rxWaitQ);
  }
  os_if_spin_unlock(&rcv->lock);
  #endif

  return VCAN_STAT_OK;
}

//======================================================================
//  Push rx queue
//======================================================================
int vCanPushReceiveBuffer (VCanReceiveData *rcv)
{
  int wasEmpty;
  #if !LINUX
  os_if_spin_lock(&rcv->lock);
  #endif

  wasEmpty = rcv->bufTail == rcv->bufHead;
  if (rcv->bufHead >= rcv->size - 1) {
    rcv->bufHead = 0;
  }
  else {
    rcv->bufHead++;
  }

  if ((rcv->bufHead % 10) == 0) {
    DEBUGPRINT(4, (TXT("RXpush(%d)\n"), rcv->bufHead));
  }

  // Wake up if the queue was empty BEFORE
  if (wasEmpty) {
    #if LINUX
    os_if_wake_up_interruptible(&rcv->rxWaitQ);
    #else
    os_if_mark_event(&rcv->rxWaitQ);
    #if DEBUG
    rcv->lastNotEmpty = rcv->bufHead;
    #endif
    #endif
  }
  #if !LINUX
  os_if_spin_unlock(&rcv->lock);
  #endif

  return VCAN_STAT_OK;
}

//======================================================================
//  Deliver to receive queue
//======================================================================

int vCanDispatchEvent (VCanChanData *chd, VCAN_EVENT *e)
{
  VCanOpenFileNode *fileNodePtr;
  int rcvQLen;
  unsigned long irqFlags;
  long objbuf_mask;

  // Update and notify readers
  // Needs to be _irqsave since some drivers call from ISR:s.
  os_if_spin_lock_irqsave(&chd->openLock, &irqFlags);
  for (fileNodePtr = chd->openFileList; fileNodePtr != NULL;
       fileNodePtr = fileNodePtr->next) {
    // Event filter
    if (!(e->tag & fileNodePtr->filter.eventMask)) {
      continue;
    }
    if (e->tag == V_RECEIVE_MSG && e->tagData.msg.flags & VCAN_MSG_FLAG_TXACK) {
      // Skip if we sent it ourselves and we don't want the ack
      if (e->transId == fileNodePtr->transId && !fileNodePtr->modeTx) {
        DEBUGPRINT(2, (TXT("TXACK Skipped since we sent it ourselves and we don't want the ack!\n")));
        continue;
      }
      if (e->transId != fileNodePtr->transId) {
        // Other receivers (virtual bus extension) should not see the TXACK.
        e->tagData.msg.flags &= ~VCAN_MSG_FLAG_TXACK;
      }
    }
    if (e->tag == V_RECEIVE_MSG && e->tagData.msg.flags & VCAN_MSG_FLAG_TXRQ) {
      // Receive only if we sent it and we want the tx request
      if (e->transId != fileNodePtr->transId || !fileNodePtr->modeTxRq) {
        continue;
      }
    }
    // CAN filter
    if (e->tag == V_RECEIVE_MSG || e->tag == V_TRANSMIT_MSG) {
      unsigned int id = e->tagData.msg.id & ~VCAN_EXT_MSG_ID;
      if ((e->tagData.msg.id & VCAN_EXT_MSG_ID) == 0) {

        // Standard message
        if ((fileNodePtr->filter.stdId ^ id) & fileNodePtr->filter.stdMask) {
          continue;
        }
      }

      else {
        // Extended message
        if ((fileNodePtr->filter.extId ^ id) & fileNodePtr->filter.extMask) {
          continue;
        }
      }
      // Filter on message flags ; note inverting
      //if (!((fileNodePtr->filter.msgFlags ^ e->tagData.msg.flags) &
      //      fileNodePtr->filter.flagsMask))
      //  continue;

      if (e->tagData.msg.flags & VCAN_MSG_FLAG_OVERRUN) {
        fileNodePtr->overruns++;
      }

      //
      // Check against the object buffers, if any.
      //
      if (!(e->tagData.msg.flags & (VCAN_MSG_FLAG_TXRQ | VCAN_MSG_FLAG_TXACK)) &&
          ((objbuf_mask = objbuf_filter_match(fileNodePtr->objbuf,
                                              e->tagData.msg.id,
                                              e->tagData.msg.flags)) != 0)) {
        // This is something that matched the code/mask for at least one buffer,
        // and it's *not* a TXRQ or a TXACK.
        atomic_set_mask(objbuf_mask, &fileNodePtr->objbufActive);
        os_if_queue_task_not_default_queue(fileNodePtr->objbufTaskQ,
                                           &fileNodePtr->objbufWork);
      }
    }

    rcvQLen = getQLen(fileNodePtr->rcv.bufHead, fileNodePtr->rcv.bufTail,
                      fileNodePtr->rcv.size);
    if (rcvQLen >= fileNodePtr->rcv.size - 1) {
      // The buffer is full, ignore new message
      fileNodePtr->overruns++;
      DEBUGPRINT(2, (TXT("File node overrun\n")));
      // Mark message that arrived before this one
      {
        int i;
        int head = fileNodePtr->rcv.bufHead;
        for (i = fileNodePtr->rcv.size - 1; i; --i){
          head = (head == 0 ? fileNodePtr->rcv.size - 1 : head - 1);
          if (fileNodePtr->rcv.fileRcvBuffer[head].tag == V_RECEIVE_MSG)
            break;
        }
        if (i) {
          fileNodePtr->rcv.fileRcvBuffer[head].tagData.msg.flags |= VCAN_MSG_FLAG_OVERRUN;
        }
      }

    }
    // Insert into buffer
    else {
      memcpy(&(fileNodePtr->rcv.fileRcvBuffer[fileNodePtr->rcv.bufHead]),
             e, sizeof(VCAN_EVENT));
      vCanPushReceiveBuffer(&fileNodePtr->rcv);
#if 0
      if ((rcvQLen % 10) == 0) {
#else
      {
#endif
        DEBUGPRINT(3, (TXT("Number of packets in receive queue: %ld\n"),
                       getQLen(fileNodePtr->rcv.bufHead,
                               fileNodePtr->rcv.bufTail,
                               fileNodePtr->rcv.size)));
      }
    }
  }
  os_if_spin_unlock_irqrestore(&chd->openLock, irqFlags);

  return 0;
}


//======================================================================
//    Open - File operation
//======================================================================

#if LINUX
int vCanOpen (struct inode *inode, struct file *filp)
#else
VCanOpenFileNode* vCanOpen(USBCAN_CONTEXT *usbcan_context)
#endif
{
  VCanOpenFileNode *openFileNodePtr;
  VCanCardData *cardData = NULL;
  VCanChanData *chanData = NULL;

#if LINUX
  unsigned long irqFlags;
  int minorNr;
  int channelNr;

  minorNr = MINOR(inode->i_rdev);

  DEBUGPRINT(2, (TXT("VCanOpen minor %d major (%d)\n"),
                 minorNr, MAJOR(inode->i_rdev)));

  // Make sure seeks do not work on the file
#if defined(FMODE_LSEEK)
  filp->f_mode &= ~FMODE_LSEEK;
#endif
#if defined(FMODE_PREAD)
  filp->f_mode &= ~(FMODE_PREAD | FMODE_PWRITE);
#endif

  os_if_spin_lock(&canCardsLock);
  if (canCards == NULL) {
    os_if_spin_unlock(&canCardsLock);
    DEBUGPRINT(1, (TXT("NO CAN cards available at this point. \n")));
    return -ENODEV;
  }

  // Find the right minor inode number
  for (cardData = canCards; cardData != NULL; cardData = cardData->next) {
    for (channelNr = 0; channelNr < cardData->nrChannels; channelNr++) {
      if (minorNr == cardData->chanData[channelNr]->minorNr) {
        chanData = cardData->chanData[channelNr];
        break;
      }
    }
  }
  os_if_spin_unlock(&canCardsLock);
  if (chanData == NULL) {
    DEBUGPRINT(1, (TXT("FAILURE: Unable to open minor %d major (%d)\n"),
                   minorNr, MAJOR(inode->i_rdev)));
    return -ENODEV;
  }

  // Allocate memory and zero the whole struct
  openFileNodePtr = os_if_kernel_malloc(sizeof(VCanOpenFileNode));
  if (openFileNodePtr == NULL) {
    return -ENOMEM;
  }
  memset(openFileNodePtr, 0, sizeof(VCanOpenFileNode));
#else
  os_if_spin_lock(&canCardsLock);
  if (canCards == NULL) {
    SetLastError(ENODEV);
    os_if_spin_unlock(&canCardsLock);
    return 0;
  }
  os_if_spin_unlock(&canCardsLock);

  // This is a dummy that will never be used for anything
#if 0
  // except finding the corresponding vCard structure.
#endif
  // Updated when an actual channel open is done via
  // ioctl and openFileNodePtr->channelOpen is set.
  chanData = 0;    /* usbcan_context->vCard->chanData[0]; */

  // Allocate memory and zero the whole struct
  openFileNodePtr = os_if_kernel_malloc(sizeof(VCanOpenFileNode));
  if (openFileNodePtr == NULL) {
    SetLastError(ENOMEM);
    return 0;
  }

  memset(openFileNodePtr, 0, sizeof(VCanOpenFileNode));

  os_if_init_cond(&openFileNodePtr->inactive);
  os_if_set_cond(&openFileNodePtr->inactive);

  os_if_spin_lock_init(&openFileNodePtr->rcv.lock);
#endif

  os_if_init_sema(&openFileNodePtr->ioctl_mutex);
  os_if_up_sema(&openFileNodePtr->ioctl_mutex);

  openFileNodePtr->rcv.size = sizeof(openFileNodePtr->rcv.fileRcvBuffer) / sizeof(openFileNodePtr->rcv.fileRcvBuffer[0]);

#if LINUX
    // Init wait queue
  os_if_init_waitqueue_head(&(openFileNodePtr->rcv.rxWaitQ));
#else
    {
      // Just to get a unique id for the fileNodePtr to be able to create a shared event between driver and canlib
      DWORD unique_identifier = (DWORD)(&(openFileNodePtr->rcv.rxWaitQ)) ^ EVENT_HANDLE_MASK;
      char eventName[DEVICE_NAME_LEN];

      snprintf(eventName, DEVICE_NAME_LEN, "CAN_Event_%x", unique_identifier);
      DEBUGOUT(ZONE_CAN_IOCTL, (TXT("CreateEvent(%S)\n"), eventName));

      // Init named wait queue
      os_if_init_named_waitqueue_head(&(openFileNodePtr->rcv.rxWaitQ), eventName);
      if (NULL == openFileNodePtr->rcv.rxWaitQ) {
        DEBUGOUT(ZONE_CAN_IOCTL, (TXT("os_if_init_named_waitqueue_head failed(%S)\n"), eventName));
        SetLastError(ENOMEM);
        return 0;
      }
    }
#endif


  vCanFlushReceiveBuffer(openFileNodePtr);
#if LINUX
  openFileNodePtr->filp               = filp;
#else
  openFileNodePtr->usbcan_context     = usbcan_context;
#endif
  openFileNodePtr->chanData           = chanData;
  openFileNodePtr->writeIsBlock       = 1;
  openFileNodePtr->readIsBlock        = 1;
  openFileNodePtr->modeTx             = 0;
  openFileNodePtr->modeTxRq           = 0;
  openFileNodePtr->writeTimeout       = -1;
  openFileNodePtr->readTimeout        = -1;
#if LINUX    // WinCE does this later   qqq transId should be larger to avoid possible repetition!!!!
  openFileNodePtr->transId             = atomic_read(&chanData->chanId);
  atomic_add(1, &chanData->chanId);
#endif
  openFileNodePtr->filter.eventMask   = ~0;
  openFileNodePtr->overruns           = 0;

#if LINUX
  // Insert this node first in list of "opens"
  os_if_spin_lock_irqsave(&chanData->openLock, &irqFlags);
  openFileNodePtr->chanNr             = -1;
  openFileNodePtr->channelLocked      = 0;
  openFileNodePtr->channelOpen        = 0;
  atomic_inc(&chanData->fileOpenCount);
  openFileNodePtr->next  = chanData->openFileList;
  chanData->openFileList = openFileNodePtr;
  os_if_spin_unlock_irqrestore(&chanData->openLock, irqFlags);

  // We want a pointer to the node as private_data
  filp->private_data = openFileNodePtr;

  // Dummy for 2.6
  OS_IF_MOD_INC_USE_COUNT;

# if !LINUX_2_6
  // qqq should this be called?
  /*
  if (!try_module_get(THIS_MODULE)) {
    DEBUGPRINT(1, (TXT("try_module_get failed...")));
    return NULL;
  }*/
# endif

  return 0;
#else
  openFileNodePtr->chanNr             = -1;
  openFileNodePtr->channelLocked      = 0;
  openFileNodePtr->channelOpen        = 0;

  return openFileNodePtr;
#endif
}


/*======================================================================*/
/*    Release - File operation                                          */
/*======================================================================*/

#if LINUX
int vCanClose (struct inode *inode, struct file *filp)
#else
int vCanClose (VCanOpenFileNode *fileNodePtr)
#endif
{
  VCanOpenFileNode **openPtrPtr;
#if LINUX
  VCanOpenFileNode *fileNodePtr;
#endif
  VCanChanData *chanData;
  unsigned long irqFlags;

#if LINUX
  fileNodePtr = filp->private_data;
#endif
  chanData = fileNodePtr->chanData;

#if !LINUX
  // chanData is not assigned until a successul ioctl open is done,
  // so there's not much to do if it is NULL.
  if (chanData) {
#endif

    os_if_spin_lock_irqsave(&chanData->openLock, &irqFlags);
    // Find the open file node
    openPtrPtr = &chanData->openFileList;
    for(; *openPtrPtr != NULL; openPtrPtr = &((*openPtrPtr)->next)) {
#if LINUX
      if ((*openPtrPtr)->filp == filp) {
#else
      if ((*openPtrPtr)->usbcan_context == fileNodePtr->usbcan_context) {
#endif
        break;
      }
    }
    // We did not find anything?
    if (*openPtrPtr == NULL) {
      os_if_spin_unlock_irqrestore(&chanData->openLock, irqFlags);
#if LINUX
      return -EBADF;
#else
      return FALSE;
#endif
    }
    // openPtrPtr now points to the next-pointer that points to the correct node
#if 1
    if (fileNodePtr != *openPtrPtr) {
      DEBUGPRINT(1, (TXT("VCanClose - not same fileNodePtr: %p vs %p\n"),
                     fileNodePtr, *openPtrPtr));
    }
#endif
    fileNodePtr = *openPtrPtr;

#if 0
    if (fileNodePtr->chanNr != -1 ) {
#endif
#if LINUX
    {
      atomic_dec(&chanData->fileOpenCount);
#else
    if (fileNodePtr->chanNr != -1 ) {
      if (atomic_dec_and_test(&chanData->fileOpenCount) == 0) {
        hwIf.busOff(chanData);
        if (fileNodePtr->objbuf) {
          // Driver-implemented auto response buffers.
          objbuf_shutdown(fileNodePtr);
          os_if_kernel_free(fileNodePtr->objbuf);
          DEBUGPRINT(2, (TXT("Driver objbuf handling shut down.\n")));
        }
        if (hwIf.objbufFree) {
          if (chanData->vCard->card_flags & DEVHND_CARD_AUTO_RESP_OBJBUFS) {
            // Firmware-implemented auto response buffers
            hwIf.objbufFree(chanData, OBJBUF_TYPE_AUTO_RESPONSE, -1);
          }
          if (chanData->vCard->card_flags & DEVHND_CARD_AUTO_TX_OBJBUFS) {
            // Firmware-implemented periodic transmit buffers
            hwIf.objbufFree(chanData, OBJBUF_TYPE_PERIODIC_TX, -1);
          }
        }
      }
#endif
      fileNodePtr->chanNr = -1;
      fileNodePtr->channelLocked = 0;
      fileNodePtr->channelOpen   = 0;
    }

    // Remove node
    *openPtrPtr = (*openPtrPtr)->next;

    os_if_spin_unlock_irqrestore(&chanData->openLock, irqFlags);

#if !LINUX
  }  // chanData
#endif

#if !LINUX
  os_if_delete_waitqueue_head(&fileNodePtr->rcv.rxWaitQ);
  os_if_delete_cond(&fileNodePtr->inactive);
  os_if_spin_lock_remove(&fileNodePtr->rcv.lock);
#endif

  os_if_delete_sema(&fileNodePtr->ioctl_mutex);
  
  // Should this be here or up?
#if LINUX
  if (!atomic_read(&chanData->fileOpenCount)) {
    hwIf.busOff(chanData);
    if (fileNodePtr->objbuf) {
      // Driver-implemented auto response buffers.
      objbuf_shutdown(fileNodePtr);
      os_if_kernel_free(fileNodePtr->objbuf);
      DEBUGPRINT(2, (TXT("Driver objbuf handling shut down.\n")));
    }
    if (hwIf.objbufFree) {
      if (chanData->vCard->card_flags & DEVHND_CARD_AUTO_RESP_OBJBUFS) {
        // Firmware-implemented auto response buffers
        hwIf.objbufFree(chanData, OBJBUF_TYPE_AUTO_RESPONSE, -1);
      }
      if (chanData->vCard->card_flags & DEVHND_CARD_AUTO_TX_OBJBUFS) {
        // Firmware-implemented periodic transmit buffers
        hwIf.objbufFree(chanData, OBJBUF_TYPE_PERIODIC_TX, -1);
      }
    }
  }
#endif

  if (fileNodePtr != NULL) {
    os_if_kernel_free(fileNodePtr);
    fileNodePtr = NULL;
  }

#if LINUX
  // Dummy for 2.6
  OS_IF_MOD_DEC_USE_COUNT;

# if !LINUX_2_6
  // qqq Should this be called?
  //module_put(THIS_MODULE);
# endif

  DEBUGPRINT(2, (TXT("VCanClose minor %d major (%d)\n"),
                 chanData->minorNr, MAJOR(inode->i_rdev)));
#endif

#if LINUX
  return 0;
#else
  return TRUE;
#endif
}

//======================================================================
// Returns whether the transmit queue on a specific channel is full
//======================================================================

int txQFull (VCanChanData *chd)
{
  return queue_full(&chd->txChanQueue);
}


//======================================================================
// Returns whether the transmit queue on a specific channel is empty
//======================================================================

int txQEmpty (VCanChanData *chd)
{
  return queue_empty(&chd->txChanQueue);
}

//======================================================================
// Returns whether the receive queue on a specific channel is empty
//======================================================================

int rxQEmpty (VCanOpenFileNode *fileNodePtr)
{
  return (getQLen(fileNodePtr->rcv.bufHead, fileNodePtr->rcv.bufTail,
                  fileNodePtr->rcv.size) == 0);
}


//======================================================================
//  IOCtl - File operation
//======================================================================

#if LINUX
static int ioctl(VCanOpenFileNode *fileNodePtr,
                 unsigned int ioctl_cmd, unsigned long arg);

#if !defined(HAVE_UNLOCKED_IOCTL)
int vCanIOCtl (struct inode *inode, struct file *filp,
               unsigned int ioctl_cmd, unsigned long arg)
{
  VCanOpenFileNode  *fileNodePtr = filp->private_data;

#if defined(HAVE_UNLOCKED_IOCTL)
  DEBUGPRINT(2, (TXT("Why am I using standard ioctl()?\n")));
#endif

  return ioctl(fileNodePtr, ioctl_cmd, arg);
}
#else

long vCanIOCtl_unlocked (struct file *filp,
                         unsigned int ioctl_cmd, unsigned long arg)
{
  VCanOpenFileNode  *fileNodePtr = filp->private_data;
  int               ret;
#endif

#else
static int ioctl(VCanOpenFileNode *fileNodePtr, unsigned int ioctl_cmd,
                 PBYTE pBufIn, DWORD dwLenIn,
                 PBYTE pBufOut, DWORD dwLenOut, PDWORD pdwActualOut);

int empty_eq(void *par)
{
  VCanChanData *chd = (VCanChanData *)par;

  return txQEmpty(chd) && hwIf.txAvailable(chd);
}

int vCanIOCtl (VCanOpenFileNode *fileNodePtr,
               DWORD ioctl_cmd, PBYTE pBufIn, DWORD dwLenIn,
               PBYTE pBufOut, DWORD dwLenOut, PDWORD pdwActualOut)
{
  int               ret;

  if (!pdwActualOut) {
    return -EINVAL;
  } else {
    *pdwActualOut = 0;   // Assume no output
  }

  // Only allow open channel and a couple of information request
  // functions when channel is not already opened (via ioctl).
  if (!fileNodePtr->channelOpen &&
      (ioctl_cmd != VCAN_IOC_OPEN_TRANSP)      &&
      (ioctl_cmd != VCAN_IOC_OPEN_CHAN)        &&
      (ioctl_cmd != VCAN_IOC_OPEN_EXCL)        &&
      (ioctl_cmd != VCAN_IOC_GET_NRCHANNELS)   &&
      (ioctl_cmd != VCAN_IOC_GET_SERIAL)       &&
      (ioctl_cmd != VCAN_IOC_GET_FIRMWARE_REV) &&
      (ioctl_cmd != VCAN_IOC_GET_EAN)          &&
      (ioctl_cmd != VCAN_IOC_GET_CARD_TYPE)
     ) {
    return -EINVAL;
  }
#endif

  // Use semaphore to enforce mutual exclusion
  // for a specific file descriptor.
  os_if_down_sema(&fileNodePtr->ioctl_mutex);
  ret = ioctl(fileNodePtr, ioctl_cmd,
#if LINUX
              arg);
#else
              pBufIn, dwLenIn, pBufOut, dwLenOut, pdwActualOut);
#endif
  os_if_up_sema(&fileNodePtr->ioctl_mutex);

  return ret;
}


static int ioctl (VCanOpenFileNode *fileNodePtr,
#if LINUX
                  unsigned int ioctl_cmd, unsigned long arg)
{
#else
                  DWORD ioctl_cmd, PBYTE pBufIn, DWORD dwLenIn,
                  PBYTE pBufOut, DWORD dwLenOut, PDWORD pdwActualOut)
{
  DWORD             arg;
#endif
  VCanChanData      *chd;
  unsigned long     timeLeft;
  unsigned long     timeout;
  OS_IF_WAITQUEUE   wait;
  int               ret;
  int               vStat = VCAN_STAT_OK;
  int               chanNr;
  unsigned long     irqFlags;
  int               tmp;

  chd = fileNodePtr->chanData;

  switch (ioctl_cmd) {
  //------------------------------------------------------------------
    case VCAN_IOC_SENDMSG:
      ArgPtrIn(sizeof(CAN_MSG));
      if (!fileNodePtr->writeIsBlock && txQFull(chd)) {
        DEBUGPRINT(2, (TXT("VCAN_IOC_SENDMSG - returning -EAGAIN\n")));
        return -EAGAIN;
      }

      os_if_init_waitqueue_entry(&wait);
      queue_add_wait_for_space(&chd->txChanQueue, &wait);

      while (1) {
        os_if_set_task_interruptible();

        if (txQFull(chd)) {
          if (fileNodePtr->writeTimeout != -1) {
            if (os_if_wait_for_event_timeout(fileNodePtr->writeTimeout,
                                             &wait) == 0) {
              // Transmit timed out
              queue_remove_wait_for_space(&chd->txChanQueue, &wait);
              DEBUGPRINT(2, (TXT("VCAN_IOC_SENDMSG - returning -EAGAIN 2\n")));
              return -EAGAIN;
            }

          } else {
#if LINUX
            // This is not the right thing to do!
            // (Function only does a schedule().)
            os_if_wait_for_event(queue_space_event(&chd->txChanQueue));
#else
            os_if_wait_for_event_timeout(INFINITE, &wait);
#endif
          }
#if LINUX
          if (signal_pending(current)) {
            // Sleep was interrupted by signal
            queue_remove_wait_for_space(&chd->txChanQueue, &wait);
            return -ERESTARTSYS;
          }
#else
          // Is the file being closed for some reason?
          if (!fileNodePtr->channelOpen) {
            queue_remove_wait_for_space(&chd->txChanQueue, &wait);
            DEBUGPRINT(2, (TXT("VCAN_IOC_SENDMSG - returning -ERESTARTSYS\n")));
            return -ERESTARTSYS;
          }
#endif
        }
        else {
          CAN_MSG *bufMsgPtr;
          CAN_MSG message;
          int queuePos;

          // The copy from user memory can sleep, so it must
          // not be done while holding the queue lock.
          // This means an extra memcpy() from a stack buffer,
          // but that can only be avoided by changing the queue
          // buffer "allocation" method (queue_back/push).
          if (os_if_set_user_data(&message, (CAN_MSG *)arg, sizeof(CAN_MSG))) {
            DEBUGPRINT(2, (TXT("VCAN_IOC_SENDMSG - returning -EFAULT\n")));
            return -EFAULT;
          }

          queuePos = queue_back(&chd->txChanQueue);
          if (queuePos < 0) {
            queue_release(&chd->txChanQueue);
            continue;
          }
          bufMsgPtr = &chd->txChanBuffer[queuePos];

          os_if_set_task_running();
          queue_remove_wait_for_space(&chd->txChanQueue, &wait);

          memcpy(bufMsgPtr, &message, sizeof(CAN_MSG));

          // This is for keeping track of the originating fileNode
          bufMsgPtr->user_data = fileNodePtr->transId;
          bufMsgPtr->flags &= ~(VCAN_MSG_FLAG_TX_NOTIFY | VCAN_MSG_FLAG_TX_START);
          if (fileNodePtr->modeTx || (atomic_read(&chd->fileOpenCount) > 1)) {
            bufMsgPtr->flags |= VCAN_MSG_FLAG_TX_NOTIFY;
          }
          if (fileNodePtr->modeTxRq) {
            bufMsgPtr->flags |= VCAN_MSG_FLAG_TX_START;
          }

          queue_push(&chd->txChanQueue);

          // Exit loop
          break;
        }
      }
      hwIf.requestSend(chd->vCard, chd); // Ok to fail ;-)
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_RECVMSG:
     ArgPtrOut(sizeof(VCAN_EVENT));
     if (!fileNodePtr->readIsBlock && rxQEmpty(fileNodePtr)) {
        DEBUGPRINT(3, (TXT("VCAN_IOC_RECVMSG - returning -EAGAIN, line = %d\n"),
                       __LINE__));
        DEBUGPRINT(3, (TXT("head = %d, tail = %d, size = %d, ")
                       TXT2("readIsBlock = %d\n"),
                       fileNodePtr->rcv.bufHead, fileNodePtr->rcv.bufTail,
                       fileNodePtr->rcv.size, fileNodePtr->readIsBlock));
        return -EAGAIN;
     }

      os_if_init_waitqueue_entry(&wait);
      os_if_add_wait_queue(&fileNodePtr->rcv.rxWaitQ, &wait);
#if LINUX
      while(1) {
        os_if_set_task_interruptible();

        if (rxQEmpty(fileNodePtr)) {

          if (fileNodePtr->readTimeout != -1) {
            if (os_if_wait_for_event_timeout(fileNodePtr->readTimeout,
                                             &wait) == 0) {
              // Receive timed out
              os_if_remove_wait_queue(&fileNodePtr->rcv.rxWaitQ, &wait);
              // Reset when finished
              return -EAGAIN;
            }
          }
          else {
#if LINUX
            os_if_wait_for_event(&fileNodePtr->rcv.rxWaitQ);
#else
            os_if_wait_for_event_timeout(INFINITE, &wait);
#endif
          }
#if LINUX
          if (signal_pending(current)) {
            // Sleep was interrupted by signal
            os_if_remove_wait_queue(&fileNodePtr->rcv.rxWaitQ, &wait);
            DEBUGPRINT(4, (TXT("VCAN_IOC_RECVMSG - returning -ERESTARTSYS, "
                               "line = %d\n"), __LINE__));
            return -ERESTARTSYS;
          }
#else
          // Is the file being closed for some reason?
          if (!fileNodePtr->channelOpen) {
            os_if_remove_wait_queue(&fileNodePtr->rcv.rxWaitQ, &wait);
            DEBUGPRINT(4, (TXT("VCAN_IOC_RECVMSG - returning -ERESTARTSYS, "
                               "line = %d\n"), __LINE__));
            return -ERESTARTSYS;
          }
#endif
        }
        // We have events in Q
        else {
          os_if_set_task_running();
          os_if_remove_wait_queue(&fileNodePtr->rcv.rxWaitQ, &wait);
          copy_to_user_ret((VCAN_EVENT *)arg,
                           &(fileNodePtr->rcv.fileRcvBuffer[fileNodePtr->rcv.bufTail]),
                           sizeof(VCAN_EVENT), -EFAULT);
          vCanPopReceiveBuffer(&fileNodePtr->rcv);
          // Exit loop
          break;
        }
      }
#else // CE

      if (fileNodePtr->readTimeout != -1) {
        if (os_if_wait_for_event_timeout(fileNodePtr->readTimeout, &wait) == 0) {
          // Receive timed out

          //DEBUGPRINT(4, (TXT("VCAN_IOC_RECVMSG - returning -EAGAIN, "
          //                   "line = %d\n"), __LINE__));
          //DEBUGPRINT(0, (TXT("VCAN_IOC_RECVMSG: head = %d, tail = %d, "
          //                   "empty = %d, notEmpty = %d\n"),
          //               fileNodePtr->rcv.bufHead, fileNodePtr->rcv.bufTail,
          //               fileNodePtr->rcv.lastEmpty,
          //               fileNodePtr->rcv.lastNotEmpty));
          if (fileNodePtr->rcv.bufHead == fileNodePtr->rcv.bufTail) {
            // Reset when finished
            os_if_remove_wait_queue(&fileNodePtr->rcv.rxWaitQ, &wait);
            return -EAGAIN;
          }
          //DEBUGPRINT(0, (TXT("!!!!!!!!!!!!!!!!: head = %d, tail = %d, "
          //                   "empty = %d, notEmpty = %d\n"),
          //               fileNodePtr->rcv.bufHead, fileNodePtr->rcv.bufTail,
          //               fileNodePtr->rcv.lastEmpty,
          //               fileNodePtr->rcv.lastNotEmpty));
        }
      }
      else {
        os_if_wait_for_event_timeout(INFINITE, &wait);
      }

      // Is the file being closed for some reason?
      if (!fileNodePtr->channelOpen) {
        os_if_remove_wait_queue(&fileNodePtr->rcv.rxWaitQ, &wait);
        DEBUGPRINT(4, (TXT("VCAN_IOC_RECVMSG - returning -ERESTARTSYS, ")
                       TXT2("line = %d\n"),
                       __LINE__));
        return -ERESTARTSYS;
      }

      // We have events in Q
      os_if_remove_wait_queue(&fileNodePtr->rcv.rxWaitQ, &wait);
      copy_to_user_ret((VCAN_EVENT *)arg,
                       &(fileNodePtr->rcv.fileRcvBuffer[fileNodePtr->rcv.bufTail]),
                       sizeof(VCAN_EVENT), -EFAULT);
      vCanPopReceiveBuffer(&fileNodePtr->rcv);

#endif
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_BUS_ON:
      DEBUGPRINT(3, (TXT("VCAN_IOC_BUS_ON\n")));
      vCanFlushReceiveBuffer(fileNodePtr);
      vStat = hwIf.flushSendBuffer(chd);
      if (vStat == VCAN_STAT_OK) {
        vStat = hwIf.busOn(chd);
      }
      // Make synchronous? qqq
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_BUS_OFF:
      DEBUGPRINT(3, (TXT("VCAN_IOC_BUS_OFF\n")));
      vStat = hwIf.busOff(chd);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_SET_BITRATE:
      {
        VCanBusParams busParams;
        ArgPtrOut(sizeof(VCanBusParams));   // Check first, to make sure
        ArgPtrIn(sizeof(VCanBusParams));
        copy_from_user_ret(&busParams, (VCanBusParams *)arg,
                           sizeof(VCanBusParams), -EFAULT);
        vStat = hwIf.setBusParams(chd, &busParams);
        if (vStat != VCAN_STAT_OK) {
          DEBUGPRINT(4, (TXT("hwIf.setBusParams(...) = %d\n"), vStat));
        } else {
          VCanBusParams busParamsSet;
          vStat = hwIf.getBusParams(chd, &busParamsSet);
          // Indicate that something went wrong by setting freq to 0
          if (vStat == VCAN_STAT_BAD_PARAMETER) {
            DEBUGPRINT(4, (TXT("Some bus parameter bad\n")));
            busParams.freq = 0;
          } else if (vStat != VCAN_STAT_OK) {
            DEBUGPRINT(4, (TXT("hwIf.getBusParams(...) = %d\n"), vStat));
          } else {
            if (busParamsSet.freq != busParams.freq) {
              DEBUGPRINT(2, (TXT("Bad freq: %ld vs %ld\n"),
                             busParamsSet.freq, busParams.freq));
              busParams.freq = 0;
            }
            if (busParams.freq > 1000000) {
              DEBUGPRINT(2, (TXT("Too high freq: %ld\n"), busParams.freq));
            }
            if (busParamsSet.sjw != busParams.sjw) {
              DEBUGPRINT(2, (TXT("Bad sjw: %d vs %d\n"),
                             busParamsSet.sjw, busParams.sjw));
              busParams.freq = 0;
            }
            if (busParamsSet.tseg1 != busParams.tseg1) {
              DEBUGPRINT(2, (TXT("Bad tseg1: %d vs %d\n"),
                             busParamsSet.tseg1, busParams.tseg1));
              busParams.freq = 0;
            }
            if (busParamsSet.tseg2 != busParams.tseg2) {
              DEBUGPRINT(2, (TXT("Bad tseg2: %d vs %d\n"),
                             busParamsSet.tseg2, busParams.tseg2));
              busParams.freq = 0;
            }
#if 0
            if (!!busParamsSet.samp3 != !!busParams.samp3) {
              DEBUGPRINT(2, (TXT("Bad samp3: %d vs %d\n"),
                             busParamsSet.samp3, busParams.samp3));
              busParams.freq = 0;
            }
#endif
          }
        }
        ArgPtrOut(sizeof(VCanBusParams));
        copy_to_user_ret((VCanBusParams *)arg, &busParams,
                         sizeof(VCanBusParams), -EFAULT);
      }
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_BITRATE:
      ArgPtrOut(sizeof(VCanBusParams));
      {
        VCanBusParams busParams;
        vStat = hwIf.getBusParams(chd, &busParams);
        copy_to_user_ret((VCanBusParams *)arg, &busParams,
                         sizeof(VCanBusParams), -EFAULT);
      }
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_SET_OUTPUT_MODE:
      ArgIntIn;
      vStat = hwIf.setOutputMode(chd, arg);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_SET_MSG_FILTER:
      ArgPtrIn(sizeof(VCanMsgFilter));
      copy_from_user_ret(&(fileNodePtr->filter), (VCanMsgFilter *)arg,
                         sizeof(VCanMsgFilter), -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_MSG_FILTER:
      ArgPtrOut(sizeof(VCanMsgFilter));
      copy_to_user_ret((VCanMsgFilter *)arg, &(fileNodePtr->filter),
                       sizeof(VCanMsgFilter), -EFAULT);
      break;
    //------------------------------------------------------------------
#if LINUX
    case VCAN_IOC_OPEN_TRANSP:
      // This is really a NOP. Implemented to simplify CANlib.
      os_if_get_int(&chanNr, (int *)arg);
      ret = os_if_set_int(chanNr, (int *)arg);
      if (ret) {
        DEBUGPRINT(4, (TXT("VCAN_IOC_OPEN_TRANSP - returning -EFAULT\n")));
        return -EFAULT;
      }
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_OPEN_CHAN:
      os_if_get_int(&chanNr, (int *)arg);
      os_if_spin_lock_irqsave(&chd->openLock, &irqFlags);
      {
        VCanOpenFileNode *tmpFnp;
        for (tmpFnp = chd->openFileList; tmpFnp && !tmpFnp->channelLocked;
             tmpFnp = tmpFnp->next) {
          /* */
        }
        // Was channel not locked (i.e. not opened exclusive) before?
        if (!tmpFnp) {
          fileNodePtr->channelOpen = 1;
          fileNodePtr->chanNr = chanNr;
        }
        os_if_spin_unlock_irqrestore(&chd->openLock, irqFlags);
        if (tmpFnp) {
          ret = os_if_set_int(-1, (int *)arg);
        } else {
          ret = os_if_set_int(chanNr, (int *)arg);
        }
      }
      if (ret) {
        DEBUGPRINT(4, (TXT("VCAN_IOC_OPEN_CHAN - returning -EFAULT\n")));
        return -EFAULT;
      }
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_OPEN_EXCL:
      os_if_get_int(&chanNr, (int *)arg);
      os_if_spin_lock_irqsave(&chd->openLock, &irqFlags);
      {
        VCanOpenFileNode *tmpFnp;
        // Return -1 if channel is opened by someone else
        for (tmpFnp = chd->openFileList; tmpFnp && !tmpFnp->channelOpen;
             tmpFnp = tmpFnp->next) {
          /* */
        }
        // Was channel not opened before?
        if (!tmpFnp) {
          fileNodePtr->channelOpen = 1;
          fileNodePtr->channelLocked = 1;
          fileNodePtr->chanNr = chanNr;
        }
        os_if_spin_unlock_irqrestore(&chd->openLock, irqFlags);
        if (tmpFnp) {
          ret = os_if_set_int(-1, (int *)arg);
        } else {
          ret = os_if_set_int(chanNr, (int *)arg);
        }
      }
      if (ret) {
        DEBUGPRINT(4, (TXT("VCAN_IOC_OPEN_EXCL - returning -EFAULT\n")));
        return -EFAULT;
      }
      break;

#else
    case VCAN_IOC_OPEN_TRANSP:
    case VCAN_IOC_OPEN_CHAN:
    case VCAN_IOC_OPEN_EXCL:
      ArgPtrOut(sizeof(int));    // Check first to make sure
      ArgPtrIn(sizeof(int));
      get_user_int_ret(&chanNr, (int *)arg, -EFAULT);

      if (fileNodePtr->channelOpen || (chanNr < 0) ||
          ((unsigned int)chanNr >=
           fileNodePtr->usbcan_context->vCard->nrChannels)) {
        return -EINVAL;
      }
      chd = fileNodePtr->usbcan_context->vCard->chanData[chanNr];

      os_if_spin_lock_irqsave(&chd->openLock, &irqFlags);
      {
        VCanOpenFileNode *tmpFnp;
        for (tmpFnp = chd->openFileList; tmpFnp; tmpFnp = tmpFnp->next) {
          if (((ioctl_cmd == VCAN_IOC_OPEN_CHAN) && tmpFnp->channelLocked) ||
              ((ioctl_cmd == VCAN_IOC_OPEN_EXCL) && (tmpFnp->channelOpen == 1))) {
            break;
          }
        }
        ArgPtrOut(sizeof(int));
        // Was channel OK regarding exclusivity before?
        if (!tmpFnp) {
          fileNodePtr->channelOpen   = (ioctl_cmd != VCAN_IOC_OPEN_TRANSP) ? 1 : 2;
          fileNodePtr->channelLocked = (ioctl_cmd == VCAN_IOC_OPEN_EXCL);
          fileNodePtr->chanNr = chanNr;

          // Linux does this earlier
          // qqq transId should be larger to avoid possible repetition!
          fileNodePtr->transId = atomic_read(&chd->chanId);
          atomic_inc(&chd->chanId);
          fileNodePtr->chanData = chd;
          // Insert this node first in list of "opens"
          atomic_inc(&chd->fileOpenCount);
          fileNodePtr->next = chd->openFileList;
          chd->openFileList = fileNodePtr;

          os_if_spin_unlock_irqrestore(&chd->openLock, irqFlags);
          if (tmpFnp) {
            ret = os_if_set_int(-1, (int *)arg);
          } else {
            ret = os_if_set_int(chanNr, (int *)arg);
          }
        }
        if (ret) {
          DEBUGPRINT(4, (TXT("VCAN_IOC_OPEN_x - returning -EFAULT\n")));
          return -EFAULT;
        }
      }
      break;
#endif
    //------------------------------------------------------------------
    case VCAN_IOC_WAIT_EMPTY:
      ArgPtrIn(sizeof(unsigned long));
      get_user_long_ret(&timeout, (unsigned long *)arg, -EFAULT);
      timeLeft = -1;
      set_bit(0, &chd->waitEmpty);   // qqq Think some more about this wait!

#if LINUX
      timeLeft = os_if_wait_event_interruptible_timeout(chd->flushQ,
          txQEmpty(chd) && hwIf.txAvailable(chd), timeout);
#else
      timeLeft = os_if_wait_event_interruptible_timeout(chd->flushQ, empty_eq,
                                                        chd, timeout);
#endif
      clear_bit(0, &chd->waitEmpty);   // qqq Think some more about this wait!

      if (timeLeft == OS_IF_TIMEOUT) {
        /*
        DEBUGPRINT(2, (TXT("VCAN_IOC_WAIT_EMPTY- EAGAIN (TxQLen = %ld, TxAvail = %ld\n"),
                       queue_length(&chd->txChanQueue),
                       hwIf.txQLen(chd)));
                       */
        return -EAGAIN;
      }
      break;
    //------------------------------------------------------------------
#if !LINUX
# define VCARD fileNodePtr->usbcan_context->vCard
#else
# define VCARD chd->vCard
#endif
    case VCAN_IOC_GET_NRCHANNELS:
      ArgPtrOut(sizeof(int));
      put_user_ret(VCARD->nrChannels, (int *)arg, -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_SERIAL:
      ArgPtrOut(8);
      put_user_ret(VCARD->serialNumber, (int *)arg, -EFAULT);
      put_user_ret(0, ((int *)arg) + 1, -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_FIRMWARE_REV:
      ArgPtrOut(8);
      tmp = (VCARD->firmwareVersionMajor << 16) |
             VCARD->firmwareVersionMinor;
      put_user_ret(tmp, ((int *)arg) + 1, -EFAULT);
      put_user_ret(VCARD->firmwareVersionBuild, ((int *)arg), -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_EAN:
      ArgPtrOut(8);
      put_user_ret(((int *)VCARD->ean)[0], (int *)arg, -EFAULT);
      put_user_ret(((int *)VCARD->ean)[1], ((int *)arg) + 1, -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_CARD_TYPE:
      ArgPtrOut(sizeof(int));
      put_user_ret(VCARD->hw_type, (int *)arg, -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_CHAN_CAP:   // qqq This should be per channel!
      ArgPtrOut(sizeof(int));
      put_user_ret(VCARD->capabilities, (int *)arg, -EFAULT);
      break;
#undef VCARD
    //------------------------------------------------------------------
    case VCAN_IOC_FLUSH_RCVBUFFER:
      vCanFlushReceiveBuffer(fileNodePtr);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_FLUSH_SENDBUFFER:
      vStat = hwIf.flushSendBuffer(chd);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_SET_WRITE_BLOCK:
      ArgIntIn;
      fileNodePtr->writeIsBlock = (unsigned char)arg;
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_SET_READ_BLOCK:
      ArgIntIn;
      fileNodePtr->readIsBlock = (unsigned char)arg;
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_SET_WRITE_TIMEOUT:
      ArgIntIn;
      fileNodePtr->writeTimeout = arg;
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_SET_READ_TIMEOUT:
      ArgIntIn;
      fileNodePtr->readTimeout = arg;
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_READ_TIMER:
      ArgPtrOut(sizeof(int));
      // qqq Unable to distinguish error from time!
      put_user_ret(hwIf.getTime(chd->vCard), (int *)arg, -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_TX_ERR:
      ArgPtrOut(sizeof(int));
      put_user_ret(hwIf.getTxErr(chd), (int *)arg, -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_RX_ERR:
      ArgPtrOut(sizeof(int));
      put_user_ret(hwIf.getRxErr(chd), (int *)arg, -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_OVER_ERR:
      ArgPtrOut(sizeof(int));
      put_user_ret(fileNodePtr->overruns, (int *)arg, -EFAULT);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_RX_QUEUE_LEVEL:
      ArgPtrOut(sizeof(int));
      {
        int ql = getQLen(fileNodePtr->rcv.bufHead, fileNodePtr->rcv.bufTail,
                         fileNodePtr->rcv.size) + hwIf.rxQLen(chd);
        put_user_ret(ql, (int *)arg, -EFAULT);
      }
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_TX_QUEUE_LEVEL:
      ArgPtrOut(sizeof(int));
      {
        int ql = queue_length(&chd->txChanQueue) + hwIf.txQLen(chd);
        put_user_ret(ql, (int *)arg, -EFAULT);
      }
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_CHIP_STATE:
      ArgPtrOut(sizeof(int));
      {
        hwIf.requestChipState(chd);
        put_user_ret(chd->chipState.state, (int *)arg, -EFAULT);
      }
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_GET_TXACK:
      ArgPtrOut(sizeof(int));
      {
        DEBUGPRINT(3, (TXT("KVASER VCAN_IOC_GET_TXACK, was %d\n"), fileNodePtr->modeTx));
        put_user_ret(fileNodePtr->modeTx, (int *)arg, -EFAULT);
        break;
      }
    case VCAN_IOC_SET_TXACK:
      ArgPtrIn(sizeof(int));
      {
        DEBUGPRINT(3, (TXT("KVASER Try to set VCAN_IOC_SET_TXACK to %d, was %d\n"),
                       *(int *)arg, fileNodePtr->modeTx));
        if (*(int *)arg >= 0 && *(int *)arg <= 2) {
          fileNodePtr->modeTx = *(int *)arg;
          DEBUGPRINT(3, (TXT("KVASER Managed to set VCAN_IOC_SET_TXACK to %d\n"),
                         fileNodePtr->modeTx));
        }
        else {
          return -EFAULT;
        }
      }
      break;
    case VCAN_IOC_SET_TXRQ:
      ArgPtrIn(sizeof(int));
      {
        DEBUGPRINT(3, (TXT("KVASER Try to set VCAN_IOC_SET_TXRQ to %d, was %d\n"),
                       *(int *)arg, fileNodePtr->modeTx));
        if (*(int *)arg >= 0 && *(int *)arg <= 2) {
          fileNodePtr->modeTxRq = *(int *)arg;
          DEBUGPRINT(3, (TXT("KVASER Managed to set VCAN_IOC_SET_TXRQ to %d\n"),
                         fileNodePtr->modeTxRq));
        }
        else {
          return -EFAULT;
        }
      }
      break;
#if !LINUX
    case VCAN_IOC_GET_EVENTHANDLE:
      ArgPtrOut(sizeof(DWORD));
      {
        // Just to get a unique id for the fileNodePtr to be able to create a shared event between driver and canlib
        DWORD unique_identifier = (DWORD)(&(fileNodePtr->rcv.rxWaitQ)) ^ EVENT_HANDLE_MASK;

        put_user_ret(unique_identifier, (DWORD *)arg, -EFAULT);
      }
      break;
#endif
#if LINUX
    // WARNING! IT IS NOT RECOMMENDED TO USE THIS IOCTL
    // (TEMP_IOCHARDRESET).
    // IT IS A SPECIAL SOLUTION FOR A CUSTOMER AND WE TAKE NO
    // RESPONSIBILITY FOR THE FUNCTIONALITY.

#   if !LINUX_2_6
    case TEMP_IOCHARDRESET:
      {
        while (MOD_IN_USE) {
          MOD_DEC_USE_COUNT;
        }

        MOD_INC_USE_COUNT; // this is because this is open
      }
      break;
#   endif
#endif

    case KCAN_IOCTL_OBJBUF_FREE_ALL:
      if (fileNodePtr->objbuf) {
        // Driver-implemented auto response buffers.
        int i;
        for (i = 0; i < MAX_OBJECT_BUFFERS; i++) {
          fileNodePtr->objbuf[i].in_use = 0;
        }
      }
      if (hwIf.objbufFree) {
        if (chd->vCard->card_flags & DEVHND_CARD_AUTO_RESP_OBJBUFS) {
          // Firmware-implemented auto response buffers
          vStat = hwIf.objbufFree(chd, OBJBUF_TYPE_AUTO_RESPONSE, -1);
        }
        if (chd->vCard->card_flags & DEVHND_CARD_AUTO_TX_OBJBUFS) {
          // Firmware-implemented periodic transmit buffers
          vStat = hwIf.objbufFree(chd, OBJBUF_TYPE_PERIODIC_TX, -1);
        }
      }
      break;

    case KCAN_IOCTL_OBJBUF_ALLOCATE:
      {
        KCanObjbufAdminData io;
        ArgPtrOut(sizeof(KCanObjbufAdminData));   // Check first, to make sure
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);

        DEBUGPRINT(2, (TXT("ObjBuf Allocate type=%d card_flags=0x%x\n"),
                       io.type, chd->vCard->card_flags));
        
        vStat = VCAN_STAT_NO_RESOURCES;
        if (io.type == OBJBUF_TYPE_AUTO_RESPONSE) {
          if (chd->vCard->card_flags & DEVHND_CARD_AUTO_RESP_OBJBUFS) {
            // Firmware-implemented auto response buffers
            if (hwIf.objbufAlloc) {
              vStat = hwIf.objbufAlloc(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                       &io.buffer_number);
            }
          } else {
            // Driver-implemented auto response buffers
            int i;
            if (!fileNodePtr->objbuf) {
              fileNodePtr->objbuf = os_if_kernel_malloc(sizeof(OBJECT_BUFFER) *
                                                        MAX_OBJECT_BUFFERS);
              if (!fileNodePtr->objbuf) {
                vStat = VCAN_STAT_NO_MEMORY;
              } else {
                memset(fileNodePtr->objbuf, 0,
                       sizeof(OBJECT_BUFFER) * MAX_OBJECT_BUFFERS);
                objbuf_init(fileNodePtr);
              }
            }
            if (fileNodePtr->objbuf) {
              vStat = VCAN_STAT_NO_MEMORY;
              for (i = 0; i < MAX_OBJECT_BUFFERS; i++) {
                if (!fileNodePtr->objbuf[i].in_use) {
                  io.buffer_number              = i | OBJBUF_DRIVER_MARKER;
                  fileNodePtr->objbuf[i].in_use = 1;
                  vStat                         = VCAN_STAT_OK;
                  break;
                }
              }
            }
          }
        }
        else if (io.type == OBJBUF_TYPE_PERIODIC_TX) {
          if (chd->vCard->card_flags & DEVHND_CARD_AUTO_TX_OBJBUFS) {
            // Firmware-implemented periodic transmit buffers
            if (hwIf.objbufAlloc) {
              vStat = hwIf.objbufAlloc(chd, OBJBUF_TYPE_PERIODIC_TX,
                                       &io.buffer_number);
            }
          }
        }

        DEBUGPRINT(2, (TXT("ObjBuf Allocate got nr=%x stat=0x%x\n"),
                       io.buffer_number, vStat));

        ArgPtrOut(sizeof(KCanObjbufAdminData));
        copy_to_user_ret((KCanObjbufAdminData *)arg, &io,
                         sizeof(KCanObjbufAdminData), -EFAULT);
      }
      break;

    case KCAN_IOCTL_OBJBUF_FREE:
      {
        KCanObjbufAdminData io;
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);

        DEBUGPRINT(2, (TXT("ObjBuf Free nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));
        
        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS)) {
            fileNodePtr->objbuf[buffer_number].in_use = 0;
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufFree) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                io.buffer_number)) {
            vStat = hwIf.objbufFree(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                    io.buffer_number);
          }
          else if (hwIf.objbufExists(chd, OBJBUF_TYPE_PERIODIC_TX,
                                     io.buffer_number)) {
            vStat = hwIf.objbufFree(chd, OBJBUF_TYPE_PERIODIC_TX,
                                    io.buffer_number);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;
        }
      }
      break;

    case KCAN_IOCTL_OBJBUF_WRITE:
      {
        KCanObjbufBufferData io;
        ArgPtrIn(sizeof(KCanObjbufBufferData));
        copy_from_user_ret(&io, (KCanObjbufBufferData *)arg,
                           sizeof(KCanObjbufBufferData), -EFAULT);

        DEBUGPRINT(2, (TXT("ObjBuf Write nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));
        
        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS) &&
              (fileNodePtr->objbuf[buffer_number].in_use)) {
            fileNodePtr->objbuf[buffer_number].msg.tag    = V_TRANSMIT_MSG;
            fileNodePtr->objbuf[buffer_number].msg.channel_index =
              (unsigned char)fileNodePtr->chanNr;   // qqq What is this for?
            fileNodePtr->objbuf[buffer_number].msg.id     = io.id;
            fileNodePtr->objbuf[buffer_number].msg.flags  =
              (unsigned char)io.flags;
            fileNodePtr->objbuf[buffer_number].msg.length =
              (unsigned char)io.dlc;
            memcpy(fileNodePtr->objbuf[buffer_number].msg.data, io.data, 8);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufWrite) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                io.buffer_number)) {
            vStat = hwIf.objbufWrite(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                     io.buffer_number,
                                     io.id, io.flags, io.dlc, io.data);
          }
          else if (hwIf.objbufExists(chd, OBJBUF_TYPE_PERIODIC_TX,
                                     io.buffer_number)) {
            vStat = hwIf.objbufWrite(chd, OBJBUF_TYPE_PERIODIC_TX,
                                     io.buffer_number,
                                     io.id, io.flags, io.dlc, io.data);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;
        }
      }
      break;

      
    case KCAN_IOCTL_OBJBUF_SET_FILTER:
      {
        KCanObjbufAdminData io;
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);

        DEBUGPRINT(2, (TXT("ObjBuf SetFilter nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));

        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS) &&
              (fileNodePtr->objbuf[buffer_number].in_use))
          {
            fileNodePtr->objbuf[buffer_number].acc_code = io.acc_code;
            fileNodePtr->objbuf[buffer_number].acc_mask = io.acc_mask;
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufSetFilter) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                io.buffer_number)) {
            vStat = hwIf.objbufSetFilter(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                         io.buffer_number,
                                         io.acc_code, io.acc_mask);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;
        }
      }
      break;
      
    case KCAN_IOCTL_OBJBUF_SET_FLAGS:
      {
        KCanObjbufAdminData io;
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);
        
        DEBUGPRINT(2, (TXT("ObjBuf SetFlags nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));

        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS) &&
              (fileNodePtr->objbuf[buffer_number].in_use))
          {
            fileNodePtr->objbuf[buffer_number].flags = io.flags;
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufSetFlags) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                io.buffer_number)) {
            vStat = hwIf.objbufSetFlags(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                        io.buffer_number,
                                        io.flags);
          } else if (hwIf.objbufExists(chd, OBJBUF_TYPE_PERIODIC_TX,
                                       io.buffer_number)) {
            vStat = hwIf.objbufSetFlags(chd, OBJBUF_TYPE_PERIODIC_TX,
                                        io.buffer_number,
                                        io.flags);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;
        }
      }
      break;
      
    case KCAN_IOCTL_OBJBUF_ENABLE:
      {
        KCanObjbufAdminData io;
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);

        DEBUGPRINT(2, (TXT("ObjBuf Enable nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));
        
        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS) &&
              (fileNodePtr->objbuf[buffer_number].in_use))
          {
            fileNodePtr->objbuf[buffer_number].active = 1;
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufEnable) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                io.buffer_number)) {
            vStat = hwIf.objbufEnable(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                      io.buffer_number, 1);
          } else if (hwIf.objbufExists(chd, OBJBUF_TYPE_PERIODIC_TX,
                                       io.buffer_number)) {
            vStat = hwIf.objbufEnable(chd, OBJBUF_TYPE_PERIODIC_TX,
                                      io.buffer_number, 1);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;
        }
      }
      break;

    case KCAN_IOCTL_OBJBUF_DISABLE:
      {
        KCanObjbufAdminData io;
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);

        DEBUGPRINT(2, (TXT("ObjBuf Disable nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));

        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS) &&
              (fileNodePtr->objbuf[buffer_number].in_use))
          {
            fileNodePtr->objbuf[buffer_number].active = 0;
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufEnable) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                io.buffer_number)) {
            vStat = hwIf.objbufEnable(chd, OBJBUF_TYPE_AUTO_RESPONSE,
                                      io.buffer_number, 0);
          } else if (hwIf.objbufExists(chd, OBJBUF_TYPE_PERIODIC_TX,
                                       io.buffer_number)) {
            vStat = hwIf.objbufEnable(chd, OBJBUF_TYPE_PERIODIC_TX,
                                      io.buffer_number, 0);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;
        }
      }
      break;

    case KCAN_IOCTL_OBJBUF_SET_PERIOD:
      {
        KCanObjbufAdminData io;
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);

        DEBUGPRINT(2, (TXT("ObjBuf SetPeriod nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));

        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS) &&
              (fileNodePtr->objbuf[buffer_number].in_use))
          {
            // Driver-implemented auto tx buffers are not implemented.
            // fileNodePtr->objbuf[buffer_number].period = io.period;
            vStat = VCAN_STAT_NO_RESOURCES;   // qqq Not implemented
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufSetPeriod) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_PERIODIC_TX,
                                io.buffer_number)) {
            vStat = hwIf.objbufSetPeriod(chd, OBJBUF_TYPE_PERIODIC_TX,
                                         io.buffer_number,
                                         io.period);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;   // qqq Not implemented
        }
      }
      break;

    case KCAN_IOCTL_OBJBUF_SET_MSG_COUNT:
      {
        KCanObjbufAdminData io;
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);

        DEBUGPRINT(2, (TXT("ObjBuf SetMsgCount nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));

        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS) &&
              (fileNodePtr->objbuf[buffer_number].in_use))
          {
            // Driver-implemented auto tx buffers are not implemented.
            // fileNodePtr->objbuf[buffer_number].period = io.period;
            vStat = VCAN_STAT_NO_RESOURCES;   // qqq Not implemented
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufSetMsgCount) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_PERIODIC_TX,
                                io.buffer_number)) {
            vStat = hwIf.objbufSetMsgCount(chd, OBJBUF_TYPE_PERIODIC_TX,
                                           io.buffer_number,
                                           io.period);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;   // qqq Not implemented
        }
      }
      break;

    case KCAN_IOCTL_OBJBUF_SEND_BURST:
      {
        KCanObjbufAdminData io;
        ArgPtrIn(sizeof(KCanObjbufAdminData));
        copy_from_user_ret(&io, (KCanObjbufAdminData *)arg,
                           sizeof(KCanObjbufAdminData), -EFAULT);
#if 0
        NTSTATUS stat = STATUS_NOT_IMPLEMENTED; 
#endif

        DEBUGPRINT(2, (TXT("ObjBuf SendBurst nr=%x card_flags=0x%x\n"),
                       io.buffer_number, chd->vCard->card_flags));

        if (io.buffer_number & OBJBUF_DRIVER_MARKER) {
          int buffer_number = io.buffer_number & OBJBUF_DRIVER_MASK;
          if (fileNodePtr->objbuf && (buffer_number < MAX_OBJECT_BUFFERS) &&
              (fileNodePtr->objbuf[buffer_number].in_use))
          {
            // Driver-implemented auto tx buffers are not implemented.
            vStat = VCAN_STAT_NO_RESOURCES;   // qqq Not implemented
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else if (hwIf.objbufExists && hwIf.objbufSendBurst) {
          if (hwIf.objbufExists(chd, OBJBUF_TYPE_PERIODIC_TX,
                                io.buffer_number)) {
            vStat = hwIf.objbufSendBurst(chd, OBJBUF_TYPE_PERIODIC_TX,
                                         io.buffer_number,
                                         io.period);
          } else {
            vStat = VCAN_STAT_BAD_PARAMETER;
          }
        } else {
          vStat = VCAN_STAT_NO_RESOURCES;   // qqq Not implemented
        }
      }
      break;

    default:
      DEBUGPRINT(1, (TXT("vCanIOCtrl UNKNOWN VCAN_IOC!!!: %d\n"), ioctl_cmd));
      return -EINVAL;
  }
  //------------------------------------------------------------------

  switch (vStat) {
  case VCAN_STAT_OK:
    return 0;
  case VCAN_STAT_FAIL:
    return -EIO;
  case VCAN_STAT_TIMEOUT:
    return -EAGAIN;
  case VCAN_STAT_NO_DEVICE:
    return -ENODEV;
  case VCAN_STAT_NO_RESOURCES:
    return -EAGAIN;
  case VCAN_STAT_NO_MEMORY:
    return -ENOMEM;
  case VCAN_STAT_SIGNALED:
    return -ERESTARTSYS;
  case VCAN_STAT_BAD_PARAMETER:
    return -EINVAL;
  default:
    return -EIO;
  }
}


//======================================================================
//  Poll - File operation
//======================================================================
#if LINUX

unsigned int vCanPoll (struct file *filp, poll_table *wait)
{
  VCanOpenFileNode  *fileNodePtr = filp->private_data;
  VCanChanData      *chd;
  int full = 0;
  unsigned int mask = 0;

  // Use semaphore to enforce mutual exclusion
  // for a specific file descriptor.
  os_if_down_sema(&fileNodePtr->ioctl_mutex);

  chd = fileNodePtr->chanData;

  full = txQFull(chd);

  // Add the channel wait queues to the poll
  poll_wait(filp, queue_space_event(&chd->txChanQueue), wait);
  poll_wait(filp, &fileNodePtr->rcv.rxWaitQ, wait);

  if (!rxQEmpty(fileNodePtr)) {
    // Readable
    mask |= POLLIN | POLLRDNORM;
    DEBUGPRINT(4, (TXT("vCanPoll: Channel %d readable\n"), fileNodePtr->chanNr));
  }

  if (!full) {
    // Writable
    mask |= POLLOUT | POLLWRNORM;
    DEBUGPRINT(4, (TXT("vCanPoll: Channel %d writable\n"), fileNodePtr->chanNr));
  }

  os_if_up_sema(&fileNodePtr->ioctl_mutex);

  return mask;
}
#endif

//======================================================================
// Init common data structures for one card
//======================================================================
int vCanInitData (VCanCardData *vCard)
{
  unsigned int  chNr;
  int           minorsUsed = 0;
  int           minor;
  VCanCardData *cardData   = canCards;
  VCanChanData *chanData;

  /* Build bitmap for used minor numbers */
  while (NULL != cardData) {
    /* Only interested in other cards */
    if (cardData != vCard) {
      for (chNr = 0; chNr < cardData->nrChannels; chNr++) {
        chanData = cardData->chanData[chNr];
        minorsUsed |= 1 << chanData->minorNr;
      }
    }
    cardData = cardData->next;
  }
  DEBUGPRINT(4, (TXT("vCanInitCardData: minorsUsed 0x%x \n"), minorsUsed));

  vCard->usPerTick = 10; // Currently, a tick is 10us long

  for (chNr = 0; chNr < vCard->nrChannels; chNr++) {
    VCanChanData *vChd = vCard->chanData[chNr];

    driverData.noOfDevices++;
    DEBUGPRINT(4, (TXT("vCanInitCardData: noOfDevices %d\n"), driverData.noOfDevices));

    // qqq This is easy to modify to only look for minors if they
    //     do not already exist (vChd->minorNr == -1 if not set).
    for (minor = 0; minor < driverData.noOfDevices; minor++) {
      DEBUGPRINT(4, (TXT("vCanInitCardData: mask 0x%x, minor %d\n"),
                     1 << minor, minor));
      if (!(minorsUsed & (1 << minor))) {
        /* Found a free minor number */
        vChd->minorNr = minor;
        minorsUsed |= 1 << minor;
        break;
      }
    }

    // Init waitqueues
    os_if_init_waitqueue_head(&(vChd->flushQ));
    queue_init(&vChd->txChanQueue, TX_CHAN_BUF_SIZE);

    os_if_spin_lock_init(&(vChd->openLock));
    os_if_init_atomic_bit(&(vChd->waitEmpty));

    atomic_set(&vChd->chanId, 1);

    // vCard points back to card
    vChd->vCard = vCard;
  }

  return 0;
}

//======================================================================
// Module init
//======================================================================

// Major device number qqq

INIT int init_module (void)
{
#if LINUX
  int result;
#endif

#if 0
  DEBUGPRINT(1, (TXT("init %d:%d@%d\n"),
               driverData.noOfDevices, driverData.majorDevNr, (int)&driverData));
#endif

  // Initialise card and data structures
  memset(&driverData, 0, sizeof(VCanDriverData));

  os_if_spin_lock_init(&canCardsLock);

#if LINUX
  result = hwIf.initAllDevices();
  if (result == -ENODEV) {
    DEBUGPRINT(1, (TXT("No Kvaser %s cards found!\n"), driverData.deviceName));
    return -1;
  } else if (result != 0) {
    DEBUGPRINT(1, (TXT("Error (%d) initializing Kvaser %s driver!\n"), result,
                   driverData.deviceName));
    return -1;
  }


  if (!create_proc_read_entry(driverData.deviceName,
                              0,             // default mode
                              NULL,          // parent dir
                              hwIf.procRead,
                              NULL           // client data
                              )) {
    DEBUGPRINT(1, (TXT("Error creating proc read entry!\n")));
    hwIf.closeAllDevices();
    return -1;
  }

  // Register driver for device
  // qqq update this to alloc_chrdev_region further on...
  //DEBUGPRINT(1, (TXT("Register = %d|%s\n"), driverData.majorDevNr,
  //               driverData.deviceName));
  result = register_chrdev(driverData.majorDevNr, driverData.deviceName, &fops);
  if (result < 0) {
    DEBUGPRINT(1, (TXT("register_chrdev(%d, %s, %x) failed, error = %d\n"),
                   driverData.majorDevNr, driverData.deviceName,
                   (int)((int64_t)&fops), result));
    hwIf.closeAllDevices();
    return -1;
  }
  DEBUGPRINT(1, (TXT("Registered = %d|%s => %d\n"), driverData.majorDevNr,
                     driverData.deviceName, result));
  if (driverData.majorDevNr == 0)
    driverData.majorDevNr = result;

  DEBUGPRINT(2, (TXT("REGISTER CHRDEV (%s) majordevnr = %d\n"),
                 driverData.deviceName, driverData.majorDevNr));
#else
  waitNodes[0].list.next = NULL;
  waitNodes[0].replyPtr = waitNodes[0].data;

  freeWaitNode = &waitNodes[0];
  DEBUGPRINT(2, (TXT("(&waitNodes[0] = %p)\n"), &waitNodes[0]));
  {
    int i;
    for(i = 1; i < sizeof(waitNodes) / sizeof(*waitNodes); i++) {
      waitNodes[i].replyPtr = waitNodes[i].data;
      waitNodes[i].list.next = freeWaitNode;
      freeWaitNode = &waitNodes[i];
      DEBUGPRINT(2, (TXT("(&waitNodes[%d] = %p)\n"),i, &waitNodes[i]));
    }
  }
  DEBUGPRINT(2, (TXT("(freeWaitNode = %p)\n"),freeWaitNode));
#endif

  os_if_do_get_time_of_day(&driverData.startTime);

  return 0;
}


//======================================================================
// Module shutdown
//======================================================================
EXIT void cleanup_module (void)
{
#if LINUX
  if (driverData.majorDevNr > 0) {
    DEBUGPRINT(2, (TXT("UNREGISTER CHRDEV (%s) majordevnr = %d\n"),
                   driverData.deviceName, driverData.majorDevNr));
    unregister_chrdev(driverData.majorDevNr, driverData.deviceName);
  }
  remove_proc_entry(driverData.deviceName, NULL /* parent dir */);
  hwIf.closeAllDevices();
#else
#endif
  os_if_spin_lock_remove(&canCardsLock);
}
//======================================================================
