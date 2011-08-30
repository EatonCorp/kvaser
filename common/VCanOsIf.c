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
#if LINUX
#include "hwnames.h"
#endif
#include "osif_functions_kernel.h"
#include "queue.h"
#include "VCanOsIf.h"
#include "debug.h"


#if LINUX
#   if LINUX_2_6
        MODULE_LICENSE("GPL");
#   else
        MODULE_LICENSE("GPL");
        EXPORT_NO_SYMBOLS;
#   endif
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

//======================================================================
// File operations...
//======================================================================
#if LINUX

static int          vCanClose(struct inode *inode, struct file *filp);
static int          vCanIOCtl(struct inode *inode, struct file *filp,
                              unsigned int cmd, unsigned long arg);
static int          vCanOpen(struct inode *inode, struct file *filp);
static unsigned int vCanPoll(struct file *filp, poll_table *wait);

struct file_operations fops = {
  .llseek         = NULL,
  .read           = NULL,
  .write          = NULL,
  .readdir        = NULL,
  .poll           = vCanPoll,
  .unlocked_ioctl = vCanIOCtl,
  .mmap           = NULL,
  .open           = vCanOpen,
  .flush          = NULL,
  .release        = vCanClose,
  .fsync          = NULL
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
//  Deliver to receive queue
//======================================================================

int vCanDispatchEvent (VCanChanData *chd, VCAN_EVENT *e)
{
  VCanOpenFileNode *fileNodePtr;
  int rcvQLen;
  unsigned long irqFlags;

  // Update and notify readers
  // Needs to be _irqsave since some drivers call from ISR:s.
  os_if_spin_lock_irqsave(&chd->openLock, &irqFlags);
  for (fileNodePtr = chd->openFileList; fileNodePtr != NULL;
       fileNodePtr = fileNodePtr->next) {
    // Event filter
    if (!(e->tag & fileNodePtr->filter.eventMask))
      continue;
    if (e->tag == V_RECEIVE_MSG && ((CAN_MSG *)e)->flags & VCAN_MSG_FLAG_TXACK) {
      // Skip if we sent it ourselves and we don't want the ack
#if LINUX
      if (e->transId == fileNodePtr->transId && !fileNodePtr->modeTx) {
#else
      if (e->chanId == fileNodePtr->chanId && !fileNodePtr->modeTx) {
#endif
        DEBUGPRINT(2, (TXT("TXACK Skipped since we sent it ourselves and we don't want the ack!\n")));
        continue;
      }
    }
    if (e->tag == V_TRANSMIT_MSG && ((CAN_MSG *)e)->flags & VCAN_MSG_FLAG_TXRQ) {
      // Receive only if we sent it and we want the tx request
#if LINUX
      if (e->transId != fileNodePtr->transId || !fileNodePtr->modeTxRq)
#else
      if (e->chanId != fileNodePtr->chanId || !fileNodePtr->modeTxRq)
#endif
        continue;
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

      if (e->tagData.msg.flags & VCAN_MSG_FLAG_OVERRUN)
        fileNodePtr->overruns++;
    }
    rcvQLen = getQLen(fileNodePtr->rcvBufHead, fileNodePtr->rcvBufTail,
                      FILE_RCV_BUF_SIZE);
    if (rcvQLen >= FILE_RCV_BUF_SIZE - 1) {
      // The buffer is full, ignore new message
      fileNodePtr->overruns++;
      DEBUGPRINT(2, (TXT("File node overrun\n")));
      // Mark message that arrived before this one
      {
        //int i = FILE_RCV_BUF_SIZE - 1, head = fileNodePtr->rcvBufHead;
        int i;
        int head = fileNodePtr->rcvBufHead;
        for (i = FILE_RCV_BUF_SIZE - 1; i; --i){
          head = (head == 0 ? FILE_RCV_BUF_SIZE - 1 : head - 1);
          if (fileNodePtr->fileRcvBuffer[head].tag == V_RECEIVE_MSG)
            break;
        }
        if (i) {
          fileNodePtr->fileRcvBuffer[head].tagData.msg.flags |= VCAN_MSG_FLAG_OVERRUN;
        }
      }

    }
    // Insert into buffer
    else {
      memcpy(&(fileNodePtr->fileRcvBuffer[fileNodePtr->rcvBufHead]),
             e, sizeof(VCAN_EVENT));
      if (++fileNodePtr->rcvBufHead >= FILE_RCV_BUF_SIZE)
        fileNodePtr->rcvBufHead = 0;
      DEBUGPRINT(3, (TXT("Number of packets in receive queue: %d\n"), rcvQLen));
      // Wake up if the queue was empty BEFORE
      if (rcvQLen == 0) {
        os_if_wake_up_interruptible(&fileNodePtr->rxWaitQ);
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
  unsigned long irqFlags;

#if LINUX
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
  if (openFileNodePtr == NULL)
    return -ENOMEM;
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
#endif

  // Init wait queue
  os_if_init_waitqueue_head(&(openFileNodePtr->rxWaitQ));


  openFileNodePtr->rcvBufTail         = 0;
  openFileNodePtr->rcvBufHead         = 0;
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
#if LINUX    // qqq Only LINUX?
  openFileNodePtr->transId            = atomic_read(&chanData->transId);
  atomic_add(1, &chanData->transId);
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
  OS_IF_MODE_INC_USE_COUNT;

  // qqq should this be called?
  /*
  if (!try_module_get(THIS_MODULE)) {
    DEBUGPRINT(1, (TXT("try_module_get failed...")));
    return NULL;
  }*/

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
      if ((*openPtrPtr)->filp == filp)
        break;
#else
      if ((*openPtrPtr)->usbcan_context == fileNodePtr->usbcan_context)
        break;
#endif
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
  os_if_delete_waitqueue_head(&fileNodePtr->rxWaitQ);
  os_if_delete_cond(&fileNodePtr->inactive);
#endif

  if (fileNodePtr != NULL) {
    os_if_kernel_free(fileNodePtr);
    fileNodePtr = NULL;
  }

  // Should this be here or up?
#if LINUX
  if (!atomic_read(&chanData->fileOpenCount)) {
    hwIf.busOff(chanData);
  }
#endif

#if LINUX
  // Dummy for 2.6
  OS_IF_MODE_DEC_USE_COUNT;

  // qqq should this be called?
  //module_put(THIS_MODULE);
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
  return (getQLen(fileNodePtr->rcvBufHead, fileNodePtr->rcvBufTail,
                  FILE_RCV_BUF_SIZE) == 0);
}


//======================================================================
//  IOCtl - File operation
//  This function is not reentrant with the same file descriptor!
//======================================================================

#if LINUX
int vCanIOCtl (struct inode *inode, struct file *filp,
               unsigned int ioctl_cmd, unsigned long arg)
#else
int empty_eq(void *par)
{
  VCanChanData *chd = (VCanChanData *)par;

  return txQEmpty(chd) && hwIf.txAvailable(chd);
}

int vCanIOCtl (VCanOpenFileNode *fileNodePtr,
               DWORD ioctl_cmd, PBYTE pBufIn, DWORD dwLenIn,
               PBYTE pBufOut, DWORD dwLenOut, PDWORD pdwActualOut)
#endif
{
#if LINUX
  VCanOpenFileNode  *fileNodePtr;
#else
  DWORD             arg;
#endif
  VCanChanData      *chd;
  unsigned long     timeLeft;
  unsigned long     timeout;
  OS_IF_WAITQUEUE   wait;
  int               ret;
  int               vStat;
  int               chanNr;
  unsigned long     irqFlags;

#if LINUX
  fileNodePtr = filp->private_data;
#else
  if (!pdwActualOut) {
    return -EINVAL;
  } else {
    *pdwActualOut = 0;   // Assume no output
  }
#endif

  chd = fileNodePtr->chanData;

#if !LINUX
  // Only allow open channel and get number of channels
  // when channel is not already opened (via ioctl)
  if (!fileNodePtr->channelOpen &&
      (ioctl_cmd != VCAN_IOC_OPEN_TRANSP) &&
      (ioctl_cmd != VCAN_IOC_OPEN_CHAN)   &&
      (ioctl_cmd != VCAN_IOC_OPEN_EXCL)   &&
      (ioctl_cmd != VCAN_IOC_GET_NRCHANNELS)) {
    return -EINVAL;
  }
#endif


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

      while(1) {
        os_if_set_task_interruptible();
        //os_if_spin_lock(&chd->sendQLock);  qqq Should this be used?

        if (txQFull(chd)) {
          //os_if_spin_unlock(&chd->sendQLock);
          if (fileNodePtr->writeTimeout != -1) {
            if (os_if_wait_for_event_timeout(1 + fileNodePtr->writeTimeout * HZ / 1000,
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
#if LINUX
          bufMsgPtr->user_data = fileNodePtr->transId;
#else
          bufMsgPtr->user_data = fileNodePtr->chanId;
#endif
          bufMsgPtr->flags &= ~(VCAN_MSG_FLAG_TX_NOTIFY | VCAN_MSG_FLAG_TX_START);
          if (fileNodePtr->modeTx || (atomic_read(&chd->fileOpenCount) > 1))
            bufMsgPtr->flags |= VCAN_MSG_FLAG_TX_NOTIFY;
          if (fileNodePtr->modeTxRq)
            bufMsgPtr->flags |= VCAN_MSG_FLAG_TX_START;

          queue_push(&chd->txChanQueue);

          //os_if_spin_unlock(&chd->sendQLock);   qqq Should this be here?
          // Exit loop
          break;
        }
      }
      hwIf.requestSend(chd->vCard, chd);
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_RECVMSG:
     ArgPtrOut(sizeof(VCAN_EVENT));
     if (!fileNodePtr->readIsBlock && rxQEmpty(fileNodePtr))
        return -EAGAIN;

      os_if_init_waitqueue_entry(&wait);
      os_if_add_wait_queue(&fileNodePtr->rxWaitQ, &wait);
      while(1) {
        os_if_set_task_interruptible();

        if (rxQEmpty(fileNodePtr)) {

          if (fileNodePtr->readTimeout != -1) {
            if (os_if_wait_for_event_timeout(1 + fileNodePtr->readTimeout * HZ / 1000,
                                             &wait) == 0) {
              // Receive timed out
              os_if_remove_wait_queue(&fileNodePtr->rxWaitQ, &wait);
              // Reset when finished
              return -EAGAIN;
            }
          }
          else {
#if LINUX
            os_if_wait_for_event(&fileNodePtr->rxWaitQ);
#else
            os_if_wait_for_event_timeout(INFINITE, &wait);
#endif
          }
#if LINUX
          if (signal_pending(current)) {
            // Sleep was interrupted by signal
            os_if_remove_wait_queue(&fileNodePtr->rxWaitQ, &wait);
            return -ERESTARTSYS;
          }
#else
          // Is the file being closed for some reason?
          if (!fileNodePtr->channelOpen) {
            os_if_remove_wait_queue(&fileNodePtr->rxWaitQ, &wait);
            return -ERESTARTSYS;
          }
#endif
        }
        // We have events in Q
        else {
          os_if_set_task_running();
          os_if_remove_wait_queue(&fileNodePtr->rxWaitQ, &wait);
          copy_to_user_ret((VCAN_EVENT *)arg,
                           &(fileNodePtr->fileRcvBuffer[fileNodePtr->rcvBufTail]),
                           sizeof(VCAN_EVENT), -EFAULT);
          if (++(fileNodePtr->rcvBufTail) >= FILE_RCV_BUF_SIZE) {
            fileNodePtr->rcvBufTail = 0;
          }
          // Exit loop
          break;
        }
      }
      break;
    //------------------------------------------------------------------
    case VCAN_IOC_BUS_ON:
      DEBUGPRINT(3, (TXT("VCAN_IOC_BUS_ON\n")));
      fileNodePtr->rcvBufHead = 0;
      fileNodePtr->rcvBufTail = 0;
      vStat = hwIf.flushSendBuffer(chd);
      vStat = hwIf.busOn(chd);
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
        if (hwIf.setBusParams(chd, &busParams)) {

          // Indicate that something went wrong by setting freq to 0
          busParams.freq = 0;
        } else {
          vStat = hwIf.getBusParams(chd, &busParams);
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
    case VCAN_IOC_OPEN_CHAN:
      os_if_get_int(&chanNr, (int *)arg);
      os_if_spin_lock_irqsave(&chd->openLock, &irqFlags);
      {
        VCanOpenFileNode *tmpFnp;
        for (tmpFnp = chd->openFileList; tmpFnp && !tmpFnp->channelLocked;
             tmpFnp = tmpFnp->next) {
          /* */
        }
        // This channel is locked (i.e opened exclusive)
        if (tmpFnp) {
          ret = os_if_set_int(-1, (int *)arg);
#if 0
          atomic_dec(&chd->fileOpenCount);
#endif
        } else {
          ret = os_if_set_int(chanNr, (int *)arg);
          fileNodePtr->channelOpen = 1;
          fileNodePtr->chanNr = chanNr;
        }
      }
      os_if_spin_unlock_irqrestore(&chd->openLock, irqFlags);
      if (ret)
        return -EFAULT;
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
        // This channel is already opened
        if (tmpFnp) {
          ret = os_if_set_int(-1, (int *)arg);
#if 0
          atomic_dec(&chd->fileOpenCount);
#endif
        } else {
          ret = os_if_set_int(chanNr, (int *)arg);
          fileNodePtr->channelOpen = 1;
          fileNodePtr->channelLocked = 1;
          fileNodePtr->chanNr = chanNr;
        }
      }
      os_if_spin_unlock_irqrestore(&chd->openLock, irqFlags);
      if (ret)
        return -EFAULT;
      break;
#else
#if !LINUX
    case VCAN_IOC_GET_NRCHANNELS:
      ArgPtrOut(sizeof(int));
      put_user_ret(fileNodePtr->usbcan_context->vCard->nrChannels,
                   (int *)arg, -EFAULT);
      break;
    case VCAN_IOC_OPEN_TRANSP:
#endif
    case VCAN_IOC_OPEN_CHAN:
    case VCAN_IOC_OPEN_EXCL:
      ArgPtrOut(sizeof(int));    // Check first to make sure
      ArgPtrIn(sizeof(int));
      get_user_int_ret(&chanNr, (int *)arg, -EFAULT);

#if !LINUX
#if 1
      if (fileNodePtr->channelOpen || (chanNr < 0) ||
          ((unsigned int)chanNr >= fileNodePtr->usbcan_context->vCard->nrChannels)) {
        return -EINVAL;
      }
      chd = fileNodePtr->usbcan_context->vCard->chanData[chanNr];
#else
      if (fileNodePtr->channelOpen ||
          (chanNr < 0) || (chanNr >= fileNodePtr->chanData->vCard->nrChannels)) {
        return -EINVAL;
      }
      chd = fileNodePtr->chanData->vCard->chanData[chanNr];
#endif
#else
      if (fileNodePtr->channelOpen || (chanNr < 0) ||
          ((unsigned int)chanNr >= chd->vCard->nrChannels)) {
        return -EINVAL;
      }
#endif

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
        if (tmpFnp) {   // Channel can't be (or already is) locked (opened exclusive)
          ret = os_if_set_int(-1, (int *)arg);
#if LINUX
 #if 0
          atomic_dec(&chd->fileOpenCount);
 #endif
#endif
        } else {
          ret = os_if_set_int(chanNr, (int *)arg);
          fileNodePtr->channelOpen   = (ioctl_cmd != VCAN_IOC_OPEN_TRANSP) ? 1 : 2;
          fileNodePtr->channelLocked = (ioctl_cmd == VCAN_IOC_OPEN_EXCL);
          fileNodePtr->chanNr = chanNr;
#if !LINUX
          fileNodePtr->chanId   = chd->chanId++;
          fileNodePtr->chanData = chd;
          // Insert this node first in list of "opens"
          atomic_inc(&chd->fileOpenCount);
          fileNodePtr->next = chd->openFileList;
          chd->openFileList = fileNodePtr;
#endif
        }
      }
      os_if_spin_unlock_irqrestore(&chd->openLock, irqFlags);
      if (ret)
        return -EFAULT;
      break;
#endif
    //------------------------------------------------------------------
    case VCAN_IOC_WAIT_EMPTY:
      ArgPtrIn(sizeof(unsigned long));
      get_user_long_ret(&timeout, (unsigned long *)arg, -EFAULT);
      timeLeft = -1;
      atomic_set(&chd->waitEmpty, 1);

#if LINUX
#   if LINUX_2_6
      timeLeft = wait_event_interruptible_timeout(chd->flushQ,
          txQEmpty(chd) && hwIf.txAvailable(chd),
          1 + timeout * HZ / 1000);
#    else
      timeLeft = os_if_wait_event_interruptible_timeout(chd->flushQ,
          txQEmpty(chd) && hwIf.txAvailable(chd),
          1 + timeout * HZ / 1000);
#    endif
#else
      timeLeft = os_if_wait_event_interruptible_timeout(chd->flushQ, empty_eq, chd,
                                                        1 + timeout);
#endif
      if (atomic_read(&chd->waitEmpty)) {
        atomic_set(&chd->waitEmpty, 0);
      }

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
    case VCAN_IOC_FLUSH_RCVBUFFER:
      fileNodePtr->rcvBufHead = 0;
      fileNodePtr->rcvBufTail = 0;
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
        int ql = getQLen(fileNodePtr->rcvBufHead, fileNodePtr->rcvBufTail,
                         FILE_RCV_BUF_SIZE) + hwIf.rxQLen(chd);
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

    default:
      DEBUGPRINT(1, (TXT("vCanIOCtrl UNKNOWN VCAN_IOC!!!\n")));
      return -EINVAL;
  }
  //------------------------------------------------------------------

  return 0;
}


//======================================================================
//  Poll - File operation
//  This function is not reentrant with the same file descriptor!
//======================================================================
#if LINUX

unsigned int vCanPoll (struct file *filp, poll_table *wait)
{
  VCanOpenFileNode  *fileNodePtr;
  VCanChanData      *chd;
  int full = 0;
  unsigned int mask = 0;

  fileNodePtr = filp->private_data;
  chd = fileNodePtr->chanData;

  full = txQFull(chd);

  // Add the channel wait queues to the poll
  poll_wait(filp, queue_space_event(&chd->txChanQueue), wait);
  poll_wait(filp, &fileNodePtr->rxWaitQ, wait);

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

  vCard->usPerTick = 1000;

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

    // vCard points back to card
    vChd->vCard = vCard;
  }

  return 0;
}

//======================================================================
// Module init
//======================================================================

// Major device number qqq

int init_module (void)
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

#if LINUX
  os_if_spin_lock_init(&canCardsLock);

  result = hwIf.initAllDevices();
  if (result == -ENODEV) {
    DEBUGPRINT(1, (TXT("No Kvaser %s cards found!"), driverData.deviceName));
    return -1;
  } else if (result != 0) {
    DEBUGPRINT(1, (TXT("Error (%d) initializing Kvaser %s driver!"), result,
                   driverData.deviceName));
    return -1;
  }


  if (!create_proc_read_entry(driverData.deviceName,
                              0,             // default mode
                              NULL,          // parent dir
                              hwIf.procRead,
                              NULL           // client data
                              )) {
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
                   (int)&fops, result));
    hwIf.closeAllDevices();
    return -1;
  }
  DEBUGPRINT(1, (TXT("Registered = %d|%s => %d\n"), driverData.majorDevNr,
                     driverData.deviceName, result));
  if (driverData.majorDevNr == 0)
    driverData.majorDevNr = result;

  DEBUGPRINT(2, (TXT("REGISTER CHRDEV (%s) majordevnr = %d\n"),
                 driverData.deviceName, driverData.majorDevNr));
#endif

  os_if_do_get_time_of_day(&driverData.startTime);

  return 0;
}


//======================================================================
// Module shutdown
//======================================================================
void cleanup_module (void)
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
}
//======================================================================
