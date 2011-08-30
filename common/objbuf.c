#if WIN32
#  include <winbase.h>
#endif

#if LINUX
#  include <linux/version.h>
#endif

#include "debug.h"
#include "vcanevt.h"
#include "VCanOsIf.h"
#include "queue.h"
#include "objbuf.h"

// Returns bitmap with matching buffers, or 0 if no match.
// "id" is assumed to have bit 31 set if it's extended.
// 
unsigned int objbuf_filter_match (OBJECT_BUFFER *buf, unsigned int id,
                                  unsigned int flags)
{
  unsigned int result = 0;
  int i;
  
  if (!buf) {
    return 0;
  }

  for (i = 0; i < MAX_OBJECT_BUFFERS; i++) {
    if (buf[i].in_use && buf[i].active) {
      if (buf[i].flags & OBJBUF_AUTO_RESPONSE_RTR_ONLY) {
        if ((flags & VCAN_MSG_FLAG_REMOTE_FRAME) == 0) {
          continue;
        }
      }
  
      if ((id & buf[i].acc_mask) == buf[i].acc_code) {
        result |= 1 << i;
      }
    }
  }

  DEBUGOUT(2, (TXT("objbuf_filter_match = %04x\n"), result));
  
  return result;
}


#if LINUX
# if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
#  define USE_CONTEXT 1
# else
#  define USE_CONTEXT 0
# endif
#else
# define USE_CONTEXT 1
#endif

#if USE_CONTEXT
static void objbuf_write_all (void *context)
#else
static void objbuf_write_all (OS_IF_TASK_QUEUE_HANDLE *work)
#endif
{
  unsigned int     i;
  uint32_t         active_mask, done_mask;
#if USE_CONTEXT
  VCanOpenFileNode *fileNodePtr = (VCanOpenFileNode *)context;
#else
  VCanOpenFileNode *fileNodePtr = container_of(work, VCanOpenFileNode, objbufWork);
#endif
  VCanChanData     *vChan       = fileNodePtr->chanData;
  VCanCardData     *vCard       = vChan->vCard;
  CAN_MSG          *bufMsgPtr;
  int              queuePos;

#if 1
  DEBUGOUT(2, (TXT("objbuf_write_all: dev = 0x%p  vCard = 0x%p\n"),
               vChan, vCard));
#endif

  // Use semaphore to enforce mutual exclusion
  // for a specific file descriptor.
  os_if_down_sema(&fileNodePtr->ioctl_mutex);

  // qqq Would be good to check "present" here, but that is device specific!
  // qqq The same for bus_on, perhaps.
  if (!fileNodePtr->objbuf) {
    // The device was unplugged before the file was released
    // We cannot deallocate here, it is too early and handled elsewhere
    os_if_up_sema(&fileNodePtr->ioctl_mutex);
    return;
  }

  active_mask = fileNodePtr->objbufActive;
  if (!active_mask) {
    os_if_up_sema(&fileNodePtr->ioctl_mutex);
    return;
  }

  done_mask = 0;
  for (i = 0; i < MAX_OBJECT_BUFFERS; i++) {
    if ((active_mask & (1 << i)) == 0) {
      continue;
    }
    if (!fileNodePtr->objbuf[i].in_use || !fileNodePtr->objbuf[i].active) {
      continue;
    }

    // qqq Would be good to check "present" here, but that is device specific!

    queuePos = queue_back(&vChan->txChanQueue);
    if (queuePos < 0) {
      queue_release(&vChan->txChanQueue);
      break;
    }
    bufMsgPtr = &vChan->txChanBuffer[queuePos];

    memcpy(bufMsgPtr, &fileNodePtr->objbuf[i].msg, sizeof(CAN_MSG));

    // This is for keeping track of the originating fileNode
    bufMsgPtr->user_data = fileNodePtr->transId;
    bufMsgPtr->flags &= ~(VCAN_MSG_FLAG_TX_NOTIFY | VCAN_MSG_FLAG_TX_START);
    if (fileNodePtr->modeTx || (atomic_read(&vChan->fileOpenCount) > 1)) {
      bufMsgPtr->flags |= VCAN_MSG_FLAG_TX_NOTIFY;
    }
    if (fileNodePtr->modeTxRq) {
      bufMsgPtr->flags |= VCAN_MSG_FLAG_TX_START;
    }

    queue_push(&vChan->txChanQueue);

    done_mask |= (1 << i);
  }
  atomic_clear_mask(done_mask, &fileNodePtr->objbufActive);

  if (active_mask != done_mask) {
    // Give ourselves a little extra work in case all the sends could not
    // be handled this time.
    // qqq Should probably be delayed (or waiting until space)
    os_if_queue_task_not_default_queue(fileNodePtr->objbufTaskQ,
                                       &fileNodePtr->objbufWork);
  }

  if (done_mask) {
    hwIf.requestSend(vCard, vChan); // Ok to fail ;-)
  }

  os_if_up_sema(&fileNodePtr->ioctl_mutex);

  DEBUGOUT(2, (TXT("objbuf_write_all: %04x/%04x\n"), done_mask, active_mask));
}


void objbuf_init (VCanOpenFileNode *fileNodePtr)
{
  os_if_init_task(&fileNodePtr->objbufWork, objbuf_write_all, fileNodePtr);
  fileNodePtr->objbufTaskQ = os_if_declare_task("objbuf",
                                                &fileNodePtr->objbufWork);
}


void objbuf_shutdown (VCanOpenFileNode *fileNodePtr)
{
  os_if_destroy_task(fileNodePtr->objbufTaskQ);
}
