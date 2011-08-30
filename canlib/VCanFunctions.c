/*
** Copyright 2002-2009 KVASER AB, Sweden.  All rights reserved.
*/

/*  Kvaser Linux Canlib VCan layer functions */

#include "vcan_ioctl.h"
#include "kcan_ioctl.h"
#include "canIfData.h"
#include "canlib_data.h"
#include "vcanevt.h"

#if LINUX
#   include <canlib.h>
#   include <stdio.h>
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <fcntl.h>
#   include <sys/ioctl.h>
#   include <unistd.h>
#   include <stdio.h>
#   include <errno.h>
#   include <signal.h>
#   include <pthread.h>
#   include <string.h>
#   include <sys/stat.h>
#else
#   include <windows.h>
#   include "linuxErrors.h"
#include "osif_functions_kernel.h"
#endif


#include "osif_functions_user.h"
#include "VCanFunctions.h"
#include "debug.h"


#if LINUX
#   if DEBUG
#      define DEBUGPRINT(args) printf args
#   else
#      define DEBUGPRINT(args)
#   endif
#else
#   if DEBUG
#      define DEBUGPRINT(args)  DEBUGOUT(1, args)
#   else
#      define DEBUGPRINT(args)
#   endif
#endif

// Standard resolution for the time stamps and canReadTimer is
// 1 ms, i.e. 100 VCAND ticks.
#define DEFAULT_TIMER_FACTOR 100


static uint32_t capabilities_table[][2] = {
  {VCAN_CHANNEL_CAP_EXTENDED_CAN,        canCHANNEL_CAP_EXTENDED_CAN},
  {VCAN_CHANNEL_CAP_SEND_ERROR_FRAMES,   canCHANNEL_CAP_GENERATE_ERROR},
  {VCAN_CHANNEL_CAP_BUSLOAD_CALCULATION, canCHANNEL_CAP_BUS_STATISTICS},
  {VCAN_CHANNEL_CAP_ERROR_COUNTERS,      canCHANNEL_CAP_ERROR_COUNTERS},
  {VCAN_CHANNEL_CAP_CAN_DIAGNOSTICS,     canCHANNEL_CAP_CAN_DIAGNOSTICS},
  {VCAN_CHANNEL_CAP_TXREQUEST,           canCHANNEL_CAP_TXREQUEST},
  {VCAN_CHANNEL_CAP_TXACKNOWLEDGE,       canCHANNEL_CAP_TXACKNOWLEDGE},
  {VCAN_CHANNEL_CAP_VIRTUAL,             canCHANNEL_CAP_VIRTUAL},
  {VCAN_CHANNEL_CAP_SIMULATED,           canCHANNEL_CAP_SIMULATED},
  {VCAN_CHANNEL_CAP_REMOTE,              canCHANNEL_CAP_REMOTE}
};


// If there are more handles than this, the rest will be
// handled by a linked list.
#define MAX_ARRAY_HANDLES 64

static HandleData  *handleArray[MAX_ARRAY_HANDLES];
static HandleList  *handleList;
static CanHandle   handleMax   = MAX_ARRAY_HANDLES;
static OS_IF_MUTEX handleMutex = OS_IF_MUTEX_INITIALIZER;


//******************************************************
// Compare handles
//******************************************************
static int hndCmp (const void *hData1, const void *hData2)
{
  return ((HandleData *)(hData1))->handle ==
         ((HandleData *)(hData2))->handle;
}


//******************************************************
// Find handle in list
//******************************************************
HandleData * findHandle (CanHandle hnd)
{
  HandleData dummyHandleData, *found;
  dummyHandleData.handle = hnd;

  os_if_mutex_lock(&handleMutex);
  if (hnd < MAX_ARRAY_HANDLES) {
    found = handleArray[hnd];
  } else {
    found = listFind(&handleList, &dummyHandleData, &hndCmp);
  }
  os_if_mutex_unlock(&handleMutex);

  return found;
}


//******************************************************
// Remove handle from list
//******************************************************
HandleData * removeHandle (CanHandle hnd)
{
  HandleData dummyHandleData, *found;
  dummyHandleData.handle = hnd;

  os_if_mutex_lock(&handleMutex);
  if (hnd < MAX_ARRAY_HANDLES) {
    found = handleArray[hnd];
    handleArray[hnd] = NULL;
  } else {
    found = listRemove(&handleList, &dummyHandleData, &hndCmp);
  }
  os_if_mutex_unlock(&handleMutex);

  return found;
}

//******************************************************
// Insert handle in list
//******************************************************
CanHandle insertHandle (HandleData *hData)
{
  CanHandle hnd = -1;
  int i;

  os_if_mutex_lock(&handleMutex);

  for(i = 0; i < MAX_ARRAY_HANDLES; i++) {
    if (!handleArray[i]) {
      hData->handle = hnd = (CanHandle)i;
      handleArray[i] = hData;
      break;
    }
  }

  if (i == MAX_ARRAY_HANDLES) {
    if (listInsertFirst(&handleList, hData) == 0) {
      hData->handle = hnd = handleMax++;
    }
  }

  os_if_mutex_unlock(&handleMutex);

  return hnd;
}

static canStatus toStatus (int error)
{
  switch (error) {
  case 0:
    return canOK;
  case EINVAL:
    return canERR_PARAM;
  case ENOMEM:
    return canERR_NOMEM;
  case EAGAIN:
    return canERR_NOMSG;      // Sometimes overridden
  case EIO:
    return canERR_NOTFOUND;   // Not so good
  case ENODEV:
    return canERR_NOTFOUND;
  case EINTR:
    return canERR_INTERRUPTED;
  case EBADMSG:
    return canERR_PARAM;      // Used?
  default:
    return canERR_INTERNAL;   // Not so good
  }
}


#if LINUX
static void test_cancel (HandleData *hData)
{
  pthread_testcancel();
}


static void notify (HandleData *hData, VCAN_EVENT *msg)
{
  canNotifyData *notifyData = &hData->notifyData;

  if (hData->callback) {
    hData->callback(notifyData);
  } else if (hData->callback2) {
    hData->callback2(hData->handle, notifyData->tag, notifyData->eventType);
  }
}
#else
extern void test_cancel(HandleData *hData);
extern void notify(HandleData *hData, VCAN_EVENT *msg);
#endif

#if !LINUX
void test_cancel (HandleData *hData)
{
  // Make sure only one message is posted per read,
  // to avoid getting the actual read channel getting
  // out of synch with the one used by the notify thread.
  // qqq Really needs a kernel call for notification!
  os_if_wait_for_cond(&hData->notify_ack);

  // Should we exit the thread?
  if (WaitForSingleObject(hData->notify_die, 0) != WAIT_OBJECT_0) {
    SetEvent(hData->notify_die);
    ExitThread(0);
  }
}


void notify (HandleData *hData, VCAN_EVENT *msg)
{
  canNotifyData *notifyData = &hData->notifyData;
  
  if (msg->tag == V_CHIP_STATE) {
    // This message would not be received by an attempt to read,
    // so there is no point in storing it.
  } else {
    hData->notify_msg = *msg;
    hData->notified   = 1;
    os_if_clear_cond(&hData->notify_ack);
  }
  PostMessage(hData->notify_hwnd, WM__CANLIB,
              (WPARAM)(DWORD)hData->handle, (LPARAM)notifyData->eventType);
}
#endif


//======================================================================
// Notification thread
//======================================================================
#if LINUX
static void *vCanNotifyThread (void *arg)
#else
static DWORD WINAPI vCanNotifyThread(PVOID arg)
#endif
{
  VCAN_EVENT msg;

  HandleData    *hData      = (HandleData *)arg;
  canNotifyData *notifyData = &hData->notifyData;
  int           ret;

  // Get time to start with
  while (1) {
    test_cancel(hData);    // Allow cancellation here

    ret = os_if_ioctl_read(hData->notifyFd, VCAN_IOC_RECVMSG,
                           &msg, sizeof(VCAN_EVENT));
    if (ret != 0 && errno == EAGAIN) {
      continue;            // Retry when no message received
    }

    // When this thread is cancelled, ioctl will be interrupted by a signal.
    if (ret != 0) {
      OS_IF_EXIT_THREAD(0);
    }

    if (msg.tag == V_CHIP_STATE) {
      struct s_vcan_chip_state *chipState    = &msg.tagData.chipState;
      notifyData->eventType                  = canEVENT_STATUS;
      notifyData->info.status.busStatus      = chipState->busStatus;
      notifyData->info.status.txErrorCounter = chipState->txErrorCounter;
      notifyData->info.status.rxErrorCounter = chipState->rxErrorCounter;
      notifyData->info.status.time           = msg.timeStamp * hData->timerScale;
      notify(hData, &msg);
    } else if (msg.tag == V_RECEIVE_MSG) {
      if (msg.tagData.msg.flags & VCAN_MSG_FLAG_ERROR_FRAME) {
        if (hData->notifyFlags & canNOTIFY_ERROR) {
          notifyData->eventType = canEVENT_ERROR;
          notifyData->info.busErr.time = msg.timeStamp * hData->timerScale;
          notify(hData, &msg);
        }
      } else if (msg.tagData.msg.flags & VCAN_MSG_FLAG_TXACK) {
        if (hData->notifyFlags & canNOTIFY_TX) {
          notifyData->eventType    = canEVENT_TX;
          notifyData->info.tx.id   = msg.tagData.msg.id;
          notifyData->info.tx.time = msg.timeStamp * hData->timerScale;
          notify(hData, &msg);
        }
      } else {
        if (hData->notifyFlags & canNOTIFY_RX) {
          notifyData->eventType    = canEVENT_RX;
          notifyData->info.rx.id   = msg.tagData.msg.id;
          notifyData->info.tx.time = msg.timeStamp * hData->timerScale;
          notify(hData, &msg);
        }
      }
    }
  }
}


//======================================================================
// vCanSetNotify
//======================================================================
static canStatus vCanSetNotify (HandleData *hData,
                                //void (*callback) (canNotifyData *)
                                OS_IF_SET_NOTIFY_PARAM, // qqq ugly
                                unsigned int notifyFlags)
{
  int one = 1;
  int ret;
  unsigned char newThread = 0;
  VCanMsgFilter filter;
  int timeout = 100; // Timeout in ms

  if (hData->notifyFd == OS_IF_INVALID_HANDLE) {
    newThread = 1;
    // Open an fd to read events from
    hData->notifyFd = os_if_open(hData->deviceName);
    if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
      int ch = hData->channelNr;
      if (os_if_ioctl_write(hData->notifyFd, VCAN_IOC_OPEN_TRANSP, 
                            &ch, sizeof(ch))) {
        OS_IF_CLOSE_HANDLE(hData->notifyFd);
        hData->notifyFd = OS_IF_INVALID_HANDLE;
      }
    }
    if (hData->notifyFd == OS_IF_INVALID_HANDLE) {
      goto error_open;
    }

    // Set blocking fileop
    os_if_ioctl_write(hData->notifyFd, VCAN_IOC_SET_READ_BLOCK,
                      &one, sizeof(one));

    // Set timeout in milliseconds
    os_if_ioctl_write(hData->notifyFd, VCAN_IOC_SET_READ_TIMEOUT,
                      &timeout, sizeof(timeout));

  }
  
  hData->notifyFlags = notifyFlags;

  // Set filters
  memset(&filter, 0, sizeof(VCanMsgFilter));
  filter.eventMask = 0;

  if ((notifyFlags & canNOTIFY_RX) ||
      (notifyFlags & canNOTIFY_TX) ||
      (notifyFlags & canNOTIFY_ERROR)) {
    filter.eventMask |= V_RECEIVE_MSG;
  }

  if (notifyFlags & canNOTIFY_STATUS) {
    filter.eventMask |= V_CHIP_STATE;
  }

  ret = os_if_ioctl_write(hData->notifyFd, VCAN_IOC_SET_MSG_FILTER,
                          &filter, sizeof(VCanMsgFilter));
  if (ret != 0) {
    goto error_ioc;
  }

  if (notifyFlags & canNOTIFY_TX) {
    int par = 1;
    ret = os_if_ioctl_write(hData->notifyFd, VCAN_IOC_SET_TXACK,
                            &par, sizeof(par));

    if (ret != 0) {
      goto error_ioc;
    }
  }

  if (newThread) {
    ret = os_if_ioctl_write(hData->fd, VCAN_IOC_FLUSH_RCVBUFFER, NULL, 0);
    if (ret != 0) {
      goto error_ioc;
    }

#if LINUX
    ret = pthread_create(&hData->notifyThread, NULL, vCanNotifyThread, hData);
    if (ret != 0) {
      goto error_thread;
    }
#else
    if ((hData->notify_die = CreateEvent(NULL, TRUE, TRUE, NULL)) == NULL) {
      goto error_thread;
    }
    if (CreateThread(NULL, 32768, vCanNotifyThread, hData,
                     STACK_SIZE_PARAM_IS_A_RESERVATION, NULL) == NULL) {
      CloseHandle(hData->notify_die);
      goto error_thread;
    }
#endif
  }

#if LINUX
  hData->callback  = callback;
  hData->callback2 = callback2;
#else
  hData->notify_hwnd = hwnd;
#endif
  return canOK;

error_thread:
error_ioc:
  OS_IF_CLOSE_HANDLE(hData->notifyFd);
  hData->notifyFd = OS_IF_INVALID_HANDLE;
error_open:
  return canERR_NOTFOUND;
}


//======================================================================
// vCanOpenChannel
//======================================================================
static canStatus vCanOpenChannel (HandleData *hData)
{
  int ret;
  VCanMsgFilter filter;

  hData->fd = os_if_open(hData->deviceName);
  if (hData->fd == OS_IF_INVALID_HANDLE) {
    return canERR_NOTFOUND;
  }

  if (hData->wantExclusive) {
    ret = os_if_ioctl_write(hData->fd, VCAN_IOC_OPEN_EXCL,
                            &hData->channelNr, sizeof(hData->channelNr));
  }
  else {
    ret = os_if_ioctl_write(hData->fd, VCAN_IOC_OPEN_CHAN,
                            &hData->channelNr, sizeof(hData->channelNr));
  }

  if (ret) {
    OS_IF_CLOSE_HANDLE(hData->fd);
    return canERR_NOTFOUND;
  }

  // VCAN_IOC_OPEN_CHAN sets channelNr to -1 if it fails
  if (hData->channelNr < 0) {
    OS_IF_CLOSE_HANDLE(hData->fd);
    return canERR_NOCHANNELS;
  }

  if (!hData->acceptVirtual) {
    uint32_t capability;
    ret = os_if_ioctl_read(hData->fd, VCAN_IOC_GET_CHAN_CAP,
                           &capability, sizeof(capability));

    if (ret) {
      OS_IF_CLOSE_HANDLE(hData->fd);
      return canERR_NOTFOUND;   // qqq Other error code? Should never happen.
    }

    if (capability & VCAN_CHANNEL_CAP_VIRTUAL) {
      OS_IF_CLOSE_HANDLE(hData->fd);
      return canERR_NOTFOUND;
    }
  }

#if !LINUX
  hData->notified = 0;
  os_if_init_cond(&hData->notify_ack);
  os_if_set_cond(&hData->notify_ack);

  hData->notification_event = NULL;
#endif

  memset(&filter, 0, sizeof(VCanMsgFilter));
  // Read only CAN messages
  filter.eventMask = V_RECEIVE_MSG | V_TRANSMIT_MSG;
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_SET_MSG_FILTER,
                          &filter, sizeof(VCanMsgFilter));

  hData->timerScale = 1.0 / DEFAULT_TIMER_FACTOR;
  hData->timerResolution = (unsigned int)(10.0 / hData->timerScale);

  return canOK;
}

//======================================================================
// vCanBusOn
//======================================================================
static canStatus vCanBusOn (HandleData *hData)
{
  int ret;
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_BUS_ON, NULL, 0);
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


//======================================================================
// vCanBusOff
//======================================================================
static canStatus vCanBusOff (HandleData *hData)
{
  int ret;

  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_BUS_OFF, NULL, 0);
  if (ret != 0) {
    return canERR_INVHANDLE;
  }

  return canOK;
}


//======================================================================
// vCanSetBusparams
//======================================================================
static canStatus vCanSetBusParams (HandleData *hData, long freq,
                                   unsigned int tseg1, unsigned int tseg2,
                                   unsigned int sjw,
                                   unsigned int noSamp, unsigned int syncmode)
{
  VCanBusParams busParams;
  int ret;

  busParams.freq    = (signed long)freq;
  busParams.sjw     = (unsigned char)sjw;
  busParams.tseg1   = (unsigned char)tseg1;
  busParams.tseg2   = (unsigned char)tseg2;
  busParams.samp3   = noSamp;   // This variable is # of samples inspite of name!

  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_SET_BITRATE,
                          &busParams, sizeof(VCanBusParams));
  if (busParams.freq == 0) {
    return canERR_PARAM;
  }
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


//======================================================================
// vCanGetBusParams
//======================================================================
static canStatus vCanGetBusParams(HandleData *hData, long *freq,
                                  unsigned int *tseg1, unsigned int *tseg2,
                                  unsigned int *sjw,
                                  unsigned int *noSamp, unsigned int *syncmode)
{
  VCanBusParams busParams;
  int ret;

  ret = os_if_ioctl_read(hData->fd, VCAN_IOC_GET_BITRATE,
                         &busParams, sizeof(VCanBusParams));

  if (ret != 0) {
    return canERR_PARAM;
  }

  if (freq)     *freq     = busParams.freq;
  if (sjw)      *sjw      = busParams.sjw;
  if (tseg1)    *tseg1    = busParams.tseg1;
  if (tseg2)    *tseg2    = busParams.tseg2;
  if (noSamp)   *noSamp   = busParams.samp3;
  if (syncmode) *syncmode = 0;

  return canOK;
}


//======================================================================
// vCanReadInternal
//======================================================================
static canStatus vCanReadInternal (HandleData *hData, long *id,
                                   void *msgPtr, unsigned int *dlc,
                                   unsigned int *flag, unsigned long *time)
{
  int i;
  int ret;
  VCAN_EVENT msg;

  while (1) {
#if !LINUX
    if (hData->notified) {
      msg = hData->notify_msg;
      hData->notified = 0;
      os_if_set_cond(&hData->notify_ack);
      ret = 0;
    } else
#endif
    ret = os_if_ioctl_read(hData->fd, VCAN_IOC_RECVMSG, &msg, sizeof(VCAN_EVENT));
    if (ret != 0) {
      return toStatus(errno);
    }
    // Receive CAN message
    if (msg.tag == V_RECEIVE_MSG) {
      unsigned int flags;

      if (msg.tagData.msg.id & EXT_MSG) {
        flags = canMSG_EXT;
      } else {
        flags = canMSG_STD;
      }
      if (msg.tagData.msg.flags & VCAN_MSG_FLAG_OVERRUN)
        flags |= canMSGERR_HW_OVERRUN | canMSGERR_SW_OVERRUN;;
      if (msg.tagData.msg.flags & VCAN_MSG_FLAG_REMOTE_FRAME)
        flags |= canMSG_RTR;
      if (msg.tagData.msg.flags & VCAN_MSG_FLAG_ERROR_FRAME)
        flags |= canMSG_ERROR_FRAME;
      if (msg.tagData.msg.flags & VCAN_MSG_FLAG_TXACK)
        flags |= canMSG_TXACK;
      if (msg.tagData.msg.flags & VCAN_MSG_FLAG_TX_START)
        flags |= canMSG_TXRQ;

      // MSb is extended flag
      if (id)   *id   = msg.tagData.msg.id & ~EXT_MSG;
      if (dlc)  *dlc  = msg.tagData.msg.dlc;
      if (time) *time = msg.timeStamp * hData->timerScale;
      if (flag) *flag = flags;

      // Copy data
      if (msgPtr != NULL) {
        int count = msg.tagData.msg.dlc;
        if (count > 8) {
          count = 8;
        }
        for (i = 0; i < count; i++)
          ((unsigned char *)msgPtr)[i] = msg.tagData.msg.data[i];
      }
      break;
    }
  }

  return canOK;
}


//======================================================================
// vCanRead
//======================================================================
static canStatus vCanRead (HandleData    *hData,
                           long          *id,
                           void          *msgPtr,
                           unsigned int  *dlc,
                           unsigned int  *flag,
                           unsigned long *time)
{
  int zero = 0;

  // Set non blocking fileop
  if (hData->readIsBlock) {
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_READ_BLOCK, &zero, sizeof(zero));
    hData->readIsBlock = 0;
  }

  return vCanReadInternal(hData, id, msgPtr, dlc, flag, time);
}


//======================================================================
// vCanReadWait
//======================================================================
static canStatus vCanReadWait (HandleData    *hData,
                               long          *id,
                               void          *msgPtr,
                               unsigned int  *dlc,
                               unsigned int  *flag,
                               unsigned long *time,
                               long          timeout)
{
  int one = 1;
  int timeout_int = timeout;
  
  if (timeout_int == 0) {
    return vCanRead(hData, id, msgPtr, dlc, flag, time);
  }

  // Set blocking fileop
  if (!hData->readIsBlock) {
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_READ_BLOCK, &one, sizeof(one));
    hData->readIsBlock = 1;
  }

  // Set timeout in milliseconds
  if (hData->readTimeout != timeout_int) {
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_READ_TIMEOUT,
                      &timeout_int, sizeof(timeout_int));
    hData->readTimeout = timeout_int;
  }

  return vCanReadInternal(hData, id, msgPtr, dlc, flag, time);
}


//======================================================================
// vCanSetBusOutputControl
//======================================================================
static canStatus vCanSetBusOutputControl (HandleData *hData,
                                          unsigned int drivertype)
{
  int silent;
  int ret;

  switch (drivertype) {
  case canDRIVER_NORMAL:
    silent = 0;
    break;
  case canDRIVER_SILENT:
    silent = 1;
    break;
  default:
    return canERR_PARAM;
  }
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_SET_OUTPUT_MODE,
                          &silent, sizeof(silent));

  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


//======================================================================
// vCanGetBusOutputControl
//======================================================================
static canStatus vCanGetBusOutputControl (HandleData *hData,
                                          unsigned int *drivertype)
{
  int silent;
  int ret;

  ret = os_if_ioctl_read(hData->fd, VCAN_IOC_GET_OUTPUT_MODE,
                         &silent, sizeof(int));

  if (ret != 0) {
    return toStatus(errno);
  }

  switch (silent) {
  case 0:
    *drivertype = canDRIVER_NORMAL;
    break;
  case 1:
    *drivertype = canDRIVER_SILENT;
    break;
  default:
    break;
  }

  return canOK;
}


//======================================================================
// vCanAccept
//======================================================================
static canStatus vCanAccept(HandleData *hData,
                            const long envelope,
                            const unsigned int flag)
{
  VCanMsgFilter filter;
  int ret;

  // ret = ioctl(hData->fd, VCAN_IOC_GET_MSG_FILTER, &filter);
  ret = os_if_ioctl_read(hData->fd, VCAN_IOC_GET_MSG_FILTER,
                         &filter, sizeof(VCanMsgFilter));
  if (ret != 0) {
    return toStatus(errno);
  }

  switch (flag) {
  case canFILTER_SET_CODE_STD:
    filter.stdId   = envelope & ((1 << 11) - 1);
    break;
  case canFILTER_SET_MASK_STD:
    filter.stdMask = envelope & ((1 << 11) - 1);
    break;
  case canFILTER_SET_CODE_EXT:
    filter.extId   = envelope & ((1 << 29) - 1);
    break;
  case canFILTER_SET_MASK_EXT:
    filter.extMask = envelope & ((1 << 29) - 1);
    break;
  default:
    return canERR_PARAM;
  }
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_SET_MSG_FILTER,
                          &filter, sizeof(VCanMsgFilter));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


//======================================================================
// vCanWriteInternal
//======================================================================
static canStatus vCanWriteInternal(HandleData *hData, long id, void *msgPtr,
                                   unsigned int dlc, unsigned int flag)
{
  CAN_MSG msg;

  int ret;
  unsigned char sendExtended;

  if      (flag & canMSG_STD) sendExtended = 0;
  else if (flag & canMSG_EXT) sendExtended = 1;
  else                        sendExtended = hData->isExtended;

  if  (( sendExtended && (id >= (1 << 29))) ||
       (!sendExtended && (id >= (1 << 11))) ||
       (dlc > 15)) {
    DEBUGPRINT((TXT("canERR_PARAM on line %d\n"), __LINE__));  // Was 3,
    return canERR_PARAM;
  }

  if (sendExtended) {
    id |= EXT_MSG;
  }
  msg.id     = id;
  msg.length = dlc;
  msg.flags  = 0;
  if (flag & canMSG_ERROR_FRAME) msg.flags |= VCAN_MSG_FLAG_ERROR_FRAME;
  if (flag & canMSG_RTR)         msg.flags |= VCAN_MSG_FLAG_REMOTE_FRAME;
  if (msgPtr) {
    memcpy(msg.data, msgPtr, dlc > 8 ? 8 : dlc);
  }

  // ret = ioctl(hData->fd, VCAN_IOC_SENDMSG, &msg);
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_SENDMSG, &msg, sizeof(CAN_MSG));

#if DEBUG
  if (ret == 0) {
    ;
  } else if (errno == EAGAIN) {
    DEBUGPRINT((TXT("VCAN_IOC_SENDMSG canERR_TXBUFOFL\n")));
  } else if (errno == EBADMSG) {
    DEBUGPRINT((TXT("VCAN_IOC_SENDMSG canERR_PARAM\n")));
  } else if (errno == EINTR) {
    DEBUGPRINT((TXT("VCAN_IOC_SENDMSG canERR_INTERRUPTED\n")));
  } else {
    DEBUGPRINT((TXT("VCAN_IOC_SENDMSG ERROR: %d\n"), errno));
  }
#endif

  if      (ret   == 0)       return canOK;
  else if (errno == EAGAIN)  return canERR_TXBUFOFL;
  else                       return toStatus(errno);
}


//======================================================================
// vCanWrite
//======================================================================
static canStatus vCanWrite (HandleData *hData, long id, void *msgPtr,
                            unsigned int dlc, unsigned int flag)
{
  int zero = 0;

  // Set non blocking fileop
  if (hData->writeIsBlock) {
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_WRITE_BLOCK, &zero, sizeof(zero));
  }
  hData->writeIsBlock = 0;

  return vCanWriteInternal(hData, id, msgPtr, dlc, flag);
}


//======================================================================
// vCanWriteWait
//======================================================================
static canStatus vCanWriteWait (HandleData *hData, long id, void *msgPtr,
                                unsigned int dlc, unsigned int flag,
                                long timeout)
{
  int one = 1;
  int timeout_int = timeout;
  
  if (timeout == 0) {
    return vCanWrite(hData, id, msgPtr, dlc, flag);
  }

  // Set blocking fileop
  if (!hData->writeIsBlock) {
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_WRITE_BLOCK, &one, sizeof(one));
  }
  hData->writeIsBlock = 1;

  // Set timeout in milliseconds
  if (hData->writeTimeout != timeout_int) {
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_WRITE_TIMEOUT,
                      &timeout_int, sizeof(timeout_int));
  }
  hData->writeTimeout = timeout_int;

  return vCanWriteInternal(hData, id, msgPtr, dlc, flag);
}


//======================================================================
// vCanWriteSync
//======================================================================
static canStatus vCanWriteSync (HandleData *hData, unsigned long timeout)
{
  int ret;
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_WAIT_EMPTY,
                          &timeout, sizeof(unsigned long));

  if      (ret   == 0)       return canOK;
  else if (errno == EAGAIN)  return canERR_TIMEOUT;
  else                       return toStatus(errno);
}


//======================================================================
// vCanReadTimer
//======================================================================
static canStatus vCanReadTimer (HandleData *hData, unsigned long *time)
{
  unsigned long tmpTime;

  if (!time) {
    return canERR_PARAM;
  }

  if (os_if_ioctl_read(hData->fd, VCAN_IOC_READ_TIMER,
                       &tmpTime, sizeof(unsigned long))) {
    return toStatus(errno);
  }
  *time = tmpTime * hData->timerScale;

  return canOK;
}


//======================================================================
// vCanReadErrorCounters
//======================================================================
static canStatus vCanReadErrorCounters (HandleData *hData, unsigned int *txErr,
                                        unsigned int *rxErr, unsigned int *ovErr)
{
  if (txErr != NULL) {
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TX_ERR,
                         txErr, sizeof(unsigned int))) {
      goto ioc_error;
    }
  }
  if (rxErr != NULL) {
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_RX_ERR,
                         rxErr, sizeof(unsigned int))) {
      goto ioc_error;
    }
  }
  if (ovErr != NULL) {
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_OVER_ERR,
                         ovErr, sizeof(unsigned int))) {
      goto ioc_error;
    }
  }

  return canOK;

ioc_error:
  return toStatus(errno);
}


//======================================================================
// vCanReadStatus
//======================================================================
static canStatus vCanReadStatus (HandleData *hData, unsigned long *flags)
{
  int reply;

  if (flags == NULL) {
    return canERR_PARAM;
  }

  *flags = 0;

  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_CHIP_STATE, &reply, sizeof(int))) {
    goto ioctl_error;
  }
  if (reply & CHIPSTAT_BUSOFF)        *flags  |= canSTAT_BUS_OFF;
  if (reply & CHIPSTAT_ERROR_PASSIVE) *flags  |= canSTAT_ERROR_PASSIVE;
  if (reply & CHIPSTAT_ERROR_WARNING) *flags  |= canSTAT_ERROR_WARNING;
  if (reply & CHIPSTAT_ERROR_ACTIVE)  *flags  |= canSTAT_ERROR_ACTIVE;

  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TX_ERR, &reply, sizeof(int))) {
    goto ioctl_error;
  }
  if (reply) {
    *flags |= canSTAT_TXERR;
  }
  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_RX_ERR, &reply, sizeof(int))) {
    goto ioctl_error;
  }
  if (reply) {
    *flags |= canSTAT_RXERR;
  }
  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_OVER_ERR, &reply, sizeof(int))) {
    goto ioctl_error;
  }
  if (reply) {
    *flags |= canSTAT_SW_OVERRUN | canSTAT_HW_OVERRUN;
  }
  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_RX_QUEUE_LEVEL, &reply, sizeof(int))) {
    goto ioctl_error;
  }
  if (reply) {
    *flags |= canSTAT_RX_PENDING;
  }
  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TX_QUEUE_LEVEL, &reply, sizeof(int))) {
    goto ioctl_error;
  }
  if (reply) {
    *flags |= canSTAT_TX_PENDING;
  }

  return canOK;

ioctl_error:
  return toStatus(errno);
}


//======================================================================
// vCanGetChannelData
//======================================================================
static canStatus vCanGetChannelData (char *deviceName, int item,
                                     void *buffer, size_t bufsize)
{
  OS_IF_FILE_HANDLE fd;
  int err = 1;

  fd = os_if_open(deviceName);
  if (fd == OS_IF_INVALID_HANDLE) {
    DEBUGPRINT((TXT("Unable to open %s\n"), deviceName));
    return canERR_NOTFOUND;
  }
  
  switch (item) {
  case canCHANNELDATA_CARD_SERIAL_NO:
    err = os_if_ioctl_read(fd, VCAN_IOC_GET_SERIAL, buffer, bufsize);
    break;

  case canCHANNELDATA_CARD_UPC_NO:
    err = os_if_ioctl_read(fd, VCAN_IOC_GET_EAN, buffer, bufsize);
    break;

  case canCHANNELDATA_CARD_FIRMWARE_REV:
    err = os_if_ioctl_read(fd, VCAN_IOC_GET_FIRMWARE_REV, buffer, bufsize);
    break;

  case canCHANNELDATA_CHANNEL_CAP:
    err = os_if_ioctl_read(fd, VCAN_IOC_GET_CHAN_CAP, buffer, bufsize);
    if (!err) {
      int i;
      uint32_t capabilities = 0;
      for(i = 0;
          i < sizeof(capabilities_table) / sizeof(capabilities_table[0]);
          i++) {
        if (*(uint32_t *)buffer & capabilities_table[i][0]) {
          capabilities |= capabilities_table[i][1];
        }
      }
      *(uint32_t *)buffer = capabilities;
    }
    break;

  case canCHANNELDATA_CARD_TYPE:
    err = os_if_ioctl_read(fd, VCAN_IOC_GET_CARD_TYPE, buffer, bufsize);
    break;

  default:
    OS_IF_CLOSE_HANDLE(fd);
    return canERR_PARAM;
  }

  OS_IF_CLOSE_HANDLE(fd);

  if (err) {
    DEBUGPRINT((TXT("Error on ioctl: %d / %d\n"), err, errno));
    return toStatus(errno);
  }

  return canOK;
}


//======================================================================
// vCanIoCtl
//======================================================================
static canStatus vCanIoCtl(HandleData *hData, unsigned int func,
                           void *buf, size_t buflen)
{
  switch(func) {
  case canIOCTL_GET_RX_BUFFER_LEVEL:
    // buf points at a DWORD which receives the current RX queue level.
    if (buf == NULL) {
      return canERR_PARAM;
    }
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_RX_QUEUE_LEVEL, buf, buflen)) {
      return toStatus(errno);
    }
    break;
  case canIOCTL_GET_TX_BUFFER_LEVEL:
    // buf points at a DWORD which receives the current TX queue level.
    if (buf == NULL) {
      return canERR_PARAM;
    }
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TX_QUEUE_LEVEL, buf, buflen)) {
      return toStatus(errno);
    }
    break;
  case canIOCTL_FLUSH_RX_BUFFER:
    // Discard the current contents of the RX queue.
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_FLUSH_RCVBUFFER, buf, buflen)) {
      return toStatus(errno);
    }
    break;
  case canIOCTL_FLUSH_TX_BUFFER:
    //  Discard the current contents of the TX queue.
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_FLUSH_SENDBUFFER, buf, buflen)) {
      return toStatus(errno);
    }
    break;
  case canIOCTL_SET_TXACK:
    // buf points at a DWORD which contains 0/1 to turn TXACKs on/ff
    if (os_if_ioctl_write(hData->fd, VCAN_IOC_SET_TXACK, buf, buflen)) {
      return toStatus(errno);
    }
    break;
  case canIOCTL_GET_TXACK:
    // buf points at a DWORD which receives current TXACKs setting
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TXACK, buf, buflen)) {
      return toStatus(errno);
    }
    break;
  case canIOCTL_SET_TXRQ:
    // buf points at a DWORD which contains 0/1 to turn TXRQs on/ff
    if (os_if_ioctl_write(hData->fd, VCAN_IOC_SET_TXRQ, buf, buflen)) {
      return toStatus(errno);
    }
    break;
  case canIOCTL_SET_TIMER_SCALE:
    {
      DWORD t;
      //
      // t is the desired resolution in microseconds.
      // VCAN uses 10 us ticks, so we scale by 10 here.
      //
      if (!buf) {
        return canERR_PARAM;
      }
      if (buflen != sizeof(DWORD)) {
        return canERR_PARAM;
      }
      t = *(DWORD *)buf;
      if (t == 0) {
        t = DEFAULT_TIMER_FACTOR * 10;
      }
      hData->timerScale = 10.0 / t;
      hData->timerResolution = t;
      break;
    }
  case canIOCTL_GET_TIMER_SCALE:
    //
    // Report the used resolution in microseconds.
    //
    if (!buf) {
      return canERR_PARAM;
    }
    if (buflen != sizeof(DWORD)) {
      return canERR_PARAM;
    }
    *(unsigned int *)buf = hData->timerResolution;
    break;
#if !LINUX
  case canIOCTL_GET_EVENTHANDLE:
    if (!buf) {
      return canERR_PARAM;
    }
    if (buflen != sizeof(HANDLE)) {
      return canERR_PARAM;
    }
    
    if (hData->notification_event == NULL) {
      DWORD unique_identifier;
      WCHAR eventName[DEVICE_NAME_LEN];

      if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_EVENTHANDLE,
                           &unique_identifier, sizeof(unique_identifier))) {
        return toStatus(errno);
      }

      wsprintf(eventName, /*DEVICE_NAME_LEN, */ L"CAN_Event_%x", unique_identifier);
      hData->notification_event = OpenEvent(EVENT_ALL_ACCESS, FALSE, eventName);
      DEBUGOUT(ZONE_CAN_IOCTL, (TXT("OpenEvent(%S)\n"), eventName));
      if (!hData->notification_event) {
        DEBUGOUT(ZONE_ERR, (TXT("Failed OpenEvent %S, (%d)!\n"),
                            eventName, GetLastError()));
        return canERR_NOHANDLES;
      }
    }
    
    *(HANDLE *)buf = hData->notification_event;
    break;
#endif
  default:
    return canERR_PARAM;
  }

  return canOK;
}


static canStatus kCanObjbufFreeAll (HandleData *hData)
{
  int ret;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_FREE_ALL, NULL, 0);
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}

static canStatus kCanObjbufAllocate (HandleData *hData, int type, int *number)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.type = type;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_ALLOCATE,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  *number = ioc.buffer_number;

  return canOK;
}


static canStatus kCanObjbufFree (HandleData *hData, int idx)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.buffer_number = idx;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_FREE,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


static canStatus kCanObjbufWrite (HandleData *hData, int idx, int id, void* msg,
                                  unsigned int dlc, unsigned int flags)
{
  int ret;
  KCanObjbufBufferData ioc;

  ioc.buffer_number = idx;
  ioc.id            = id;
  ioc.dlc           = dlc;
  memcpy(ioc.data, msg, sizeof(ioc.data));
  ioc.flags         = flags;
  
  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_WRITE,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


static canStatus kCanObjbufSetFilter (HandleData *hData, int idx,
                                      unsigned int code, unsigned int mask)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.buffer_number = idx;
  ioc.acc_code      = code;
  ioc.acc_mask      = mask;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_SET_FILTER,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


static canStatus kCanObjbufSetFlags (HandleData *hData, int idx, unsigned int flags)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.buffer_number = idx;
  ioc.flags         = flags;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_SET_FLAGS,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


static canStatus kCanObjbufSetPeriod (HandleData *hData, int idx, unsigned int period)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.buffer_number = idx;
  ioc.period        = period;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_SET_PERIOD,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


static canStatus kCanObjbufSetMsgCount (HandleData *hData, int idx, unsigned int count)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.buffer_number = idx;
  ioc.period        = count;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_SET_MSG_COUNT,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


static canStatus kCanObjbufSendBurst (HandleData *hData, int idx, unsigned int burstLen)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.buffer_number = idx;
  ioc.period        = burstLen;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_SEND_BURST,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


static canStatus kCanObjbufEnable (HandleData *hData, int idx)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.buffer_number = idx;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_ENABLE,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}


static canStatus kCanObjbufDisable (HandleData *hData, int idx)
{
  int ret;
  KCanObjbufAdminData ioc;

  ioc.buffer_number = idx;

  ret = os_if_ioctl_write(hData->fd, KCAN_IOCTL_OBJBUF_DISABLE,
                          &ioc, sizeof(ioc));
  if (ret != 0) {
    return toStatus(errno);
  }

  return canOK;
}

#if LINUX
CANOps vCanOps = {
  // VCan Functions
  .setNotify           = vCanSetNotify,
  .openChannel         = vCanOpenChannel,
  .busOn               = vCanBusOn,
  .busOff              = vCanBusOff,
  .setBusParams        = vCanSetBusParams,
  .getBusParams        = vCanGetBusParams,
  .read                = vCanRead,
  .readWait            = vCanReadWait,
  .setBusOutputControl = vCanSetBusOutputControl,
  .getBusOutputControl = vCanGetBusOutputControl,
  .accept              = vCanAccept,
  .write               = vCanWrite,
  .writeWait           = vCanWriteWait,
  .writeSync           = vCanWriteSync,
  .readTimer           = vCanReadTimer,
  .readErrorCounters   = vCanReadErrorCounters,
  .readStatus          = vCanReadStatus,
  .getChannelData      = vCanGetChannelData,
  .ioCtl               = vCanIoCtl,
  .objbufFreeAll       = kCanObjbufFreeAll,
  .objbufAllocate      = kCanObjbufAllocate,
  .objbufFree          = kCanObjbufFree,
  .objbufWrite         = kCanObjbufWrite,
  .objbufSetFilter     = kCanObjbufSetFilter,
  .objbufSetFlags      = kCanObjbufSetFlags,
  .objbufSetPeriod     = kCanObjbufSetPeriod,
  .objbufSetMsgCount   = kCanObjbufSetMsgCount,
  .objbufSendBurst     = kCanObjbufSendBurst,
  .objbufEnable        = kCanObjbufEnable,
  .objbufDisable       = kCanObjbufDisable
};
#else
CANOps vCanOps = {
  /* VCan Functions */
  /*openChannel:*/      vCanOpenChannel,
  /*setNotify:*/        vCanSetNotify,
  /*busOn:*/            vCanBusOn,
  /*busOff:*/           vCanBusOff,
  /*setBusParams:*/     vCanSetBusParams,
  /*getBusParams:*/     vCanGetBusParams,
  /*read:*/             vCanRead,
  /*readWait:*/         vCanReadWait,
  /*setBusOutputControl:*/ vCanSetBusOutputControl,
  /*getBusOutputControl:*/ vCanGetBusOutputControl,
  /*accept:*/           vCanAccept,
  /*write:*/            vCanWrite,
  /*writeWait:*/        vCanWriteWait,
  /*writeSync:*/        vCanWriteSync,
                        0,
  /*readTimer:*/        vCanReadTimer,
  /*readErrorCounters:*/ vCanReadErrorCounters,
  /*readStatus:*/       vCanReadStatus,
  /*getChannelData:*/   vCanGetChannelData,
  /*ioCtl:*/            vCanIoCtl,
  kCanObjbufFreeAll,
  kCanObjbufAllocate,
  kCanObjbufFree,
  kCanObjbufWrite,
  kCanObjbufSetFilter,
  kCanObjbufSetFlags,
  kCanObjbufSetPeriod,
  kCanObjbufSetMsgCount,
  kCanObjbufSendBurst,
  kCanObjbufEnable,
  kCanObjbufDisable
};

#endif
