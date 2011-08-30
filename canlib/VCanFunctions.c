/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*  Kvaser Linux Canlib VCan layer functions */

#include "vcan_ioctl.h"
#include "canIfData.h"
#include "canlib_data.h"
#include "vcanevt.h"

#include <canlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include <sys/stat.h>


#include "osif_functions_user.h"
#include "VCanFunctions.h"
#include "debug.h"


#if DEBUG
#   define DEBUGPRINT(args) printf args
#else
#   define DEBUGPRINT(args)
#endif


//======================================================================
// Notification thread
//======================================================================
void *vCanNotifyThread (void *arg)
{
  VCAN_EVENT msg;

  HandleData    *hData;
  canNotifyData *notifyDataPtr;
  int           ret;

  hData = (HandleData *)arg;
  // Get time to start with
  while (1) {
    // Allow cancellation here

    pthread_testcancel();
    ret = os_if_ioctl_read(hData->notifyFd, VCAN_IOC_RECVMSG,
                           &msg, sizeof(VCAN_EVENT));

    // When this thread is cancelled ioctl will be interrupted by a signal
    if (ret != 0)
      OS_IF_EXIT_THREAD(0);
    notifyDataPtr = &(hData->notifyData);

    if (msg.tag == V_CHIP_STATE) {
      notifyDataPtr->eventType                  = canEVENT_STATUS;
      notifyDataPtr->info.status.busStatus      = msg.tagData.chipState.busStatus;
      notifyDataPtr->info.status.txErrorCounter = msg.tagData.chipState.txErrorCounter;
      notifyDataPtr->info.status.rxErrorCounter = msg.tagData.chipState.rxErrorCounter;
      notifyDataPtr->info.status.time           = msg.timeStamp;
      hData->callback(notifyDataPtr);
    }

    if (msg.tag == V_RECEIVE_MSG) {

      if (msg.tagData.msg.flags & VCAN_MSG_FLAG_ERROR_FRAME){
        if (hData->notifyFlags & canNOTIFY_ERROR) {
          notifyDataPtr->eventType = canEVENT_ERROR;
          notifyDataPtr->info.busErr.time = msg.timeStamp;
          hData->callback(notifyDataPtr);
        }
      } else if (msg.tagData.msg.flags & VCAN_MSG_FLAG_TXACK) {
        if (hData->notifyFlags & canNOTIFY_TX) {
          notifyDataPtr->eventType    = canEVENT_TX;
          notifyDataPtr->info.tx.id   = msg.tagData.msg.id;
          notifyDataPtr->info.tx.time = msg.timeStamp;
          hData->callback(notifyDataPtr);
        }
      } else {
        if (hData->notifyFlags & canNOTIFY_RX) {
          notifyDataPtr->eventType    = canEVENT_RX;
          notifyDataPtr->info.rx.id   = msg.tagData.msg.id;
          notifyDataPtr->info.tx.time = msg.timeStamp;
          hData->callback(notifyDataPtr);
        }
      }
    }
  }
}


//======================================================================
// vCanSetNotify
//======================================================================
canStatus vCanSetNotify (HandleData *hData,
                         //void (*callback) (canNotifyData *)
                         OS_IF_SET_NOTIFY_PARAM, // qqq ugly
                         unsigned int notifyFlags)
{
  int ret;
  unsigned char newThread = 0;
  VCanMsgFilter filter;

  if (hData->notifyFd == 0) {
    newThread = 1;
    // Open an fd to read events from
    hData->notifyFd = open(hData->deviceName, O_RDONLY);
    if (hData->notifyFd == OS_IF_INVALID_HANDLE)
      goto error_open;
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
  if (ret != 0)
    goto error_ioc;

  if (notifyFlags & canNOTIFY_TX) {
    int par = 1;
    ret = os_if_ioctl_write(hData->notifyFd, VCAN_IOC_SET_TXACK,
                            &par, sizeof(par));

    if (ret != 0)
      goto error_ioc;
  }

  if (newThread) {
    ret = os_if_ioctl_write(hData->fd, VCAN_IOC_FLUSH_RCVBUFFER, NULL, 0);
    if (ret != 0)
      goto error_ioc;
    ret = pthread_create((pthread_t *)&(hData->notifyThread), NULL,
                         vCanNotifyThread, hData);
    if (ret != 0)
      goto error_thread;
  }

  hData->callback = callback;
  return canOK;

error_thread:
error_ioc:
  OS_IF_CLOSE_HANDLE(hData->notifyFd);
error_open:
  return canERR_NOTFOUND;
}


//======================================================================
// vCanOpenChannel
//======================================================================
canStatus vCanOpenChannel (HandleData *hData)
{
  int ret;
  VCanMsgFilter filter;

  hData->fd = open(hData->deviceName, O_RDONLY);

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

  memset(&filter, 0, sizeof(VCanMsgFilter));
  // Read only CAN messages
  filter.eventMask = V_RECEIVE_MSG | V_TRANSMIT_MSG;
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_SET_MSG_FILTER,
                          &filter, sizeof(VCanMsgFilter));

  return canOK;
}

//======================================================================
// vCanBusOn
//======================================================================
canStatus vCanBusOn (HandleData *hData)
{
  int ret;
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_BUS_ON, NULL, 0);
  if (ret != 0) {
    return canERR_INVHANDLE;
  }
  else {
    return canOK;
  }
}


//======================================================================
// vCanBusOff
//======================================================================
canStatus vCanBusOff (HandleData *hData)
{
  int ret;

  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_BUS_OFF, NULL, 0);
  if (ret != 0) {
    return canERR_INVHANDLE;
  }
  else {
    return canOK;
  }
}


//======================================================================
// vCanSetBusparams
//======================================================================
canStatus vCanSetBusParams (HandleData *hData, long freq, unsigned int tseg1,
                            unsigned int tseg2, unsigned int sjw,
                            unsigned int noSamp, unsigned int syncmode)
{
  VCanBusParams busParams;
  int ret;

  busParams.freq    = (signed long)freq;
  busParams.sjw     = (unsigned char)sjw;
  busParams.tseg1   = (unsigned char)tseg1;
  busParams.tseg2   = (unsigned char)tseg2;
  busParams.samp3   = (unsigned char)noSamp;

  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_SET_BITRATE,
                          &busParams, sizeof(VCanBusParams));
  if (busParams.freq == 0)
    return canERR_PARAM;
  if (ret != 0)
    return canERR_INVHANDLE;

  return canOK;
}


//======================================================================
// vCanGetBusParams
//======================================================================
canStatus vCanGetBusParams(HandleData *hData, long *freq, unsigned int *tseg1,
                           unsigned int *tseg2, unsigned int *sjw,
                           unsigned int *noSamp, unsigned int *syncmode)
{
  VCanBusParams busParams;
  int ret;

  ret = os_if_ioctl_read(hData->fd, VCAN_IOC_GET_BITRATE,
                         &busParams, sizeof(VCanBusParams));

  if (ret != 0)
    return canERR_PARAM;
  if (freq)
    *freq   = busParams.freq;
  if (sjw)
    *sjw    = busParams.sjw;
  if (tseg1)
    *tseg1  = busParams.tseg1;
  if (tseg2)
    *tseg2  = busParams.tseg2;
  if (noSamp)
    *noSamp = busParams.samp3;
  if (noSamp)
    *syncmode = 0;

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
    ret = os_if_ioctl_read(hData->fd, VCAN_IOC_RECVMSG, &msg, sizeof(VCAN_EVENT));
    if (ret != 0){
      if (errno == EAGAIN) {
        return canERR_NOMSG;
      }
      else if (errno == EINTR) {
        return canERR_INTERRUPTED;
      }
      else {
        return canERR_INVHANDLE;
      }
    }
    // Receive CAN message
    if (msg.tag == V_RECEIVE_MSG){

      // MSb is extended flag
      if (id   != NULL)
        *id   = msg.tagData.msg.id & ~EXT_MSG;
      if (dlc  != NULL)
        *dlc  = msg.tagData.msg.dlc;
      if (time != NULL)
        *time = msg.timeStamp;
      if (flag != NULL) {
        if (msg.tagData.msg.id & EXT_MSG) {
          *flag = canMSG_EXT;
        } else {
          *flag = canMSG_STD;
        }
        if (msg.tagData.msg.flags & VCAN_MSG_FLAG_OVERRUN)
          *flag |=canMSGERR_HW_OVERRUN | canMSGERR_SW_OVERRUN;;
        if (msg.tagData.msg.flags & VCAN_MSG_FLAG_REMOTE_FRAME)
          *flag |= canMSG_RTR;
        if (msg.tagData.msg.flags & VCAN_MSG_FLAG_ERROR_FRAME)
          *flag |= canMSG_ERROR_FRAME;
        if (msg.tagData.msg.flags & VCAN_MSG_FLAG_TXACK)
          *flag |= canMSG_TXACK;
        if (msg.tagData.msg.flags & VCAN_MSG_FLAG_TX_START)
          *flag |= canMSG_TXRQ;
      }

      // Copy data
      if (msgPtr != NULL) {
        for (i = 0; i < msg.tagData.msg.dlc; i++)
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
canStatus vCanRead (HandleData    *hData,
                    long          *id,
                    void          *msgPtr,
                    unsigned int  *dlc,
                    unsigned int  *flag,
                    unsigned long *time)
{
  // Set non blocking fileop
  if (hData->readIsBlock) {
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_READ_BLOCK, NULL, 0);
    hData->readIsBlock = 0;
  }

  return vCanReadInternal(hData, id, msgPtr, dlc, flag, time);
}


//======================================================================
// vCanReadWait
//======================================================================
canStatus vCanReadWait (HandleData    *hData,
                        long          *id,
                        void          *msgPtr,
                        unsigned int  *dlc,
                        unsigned int  *flag,
                        unsigned long *time,
                        long          timeout)
{
  if (timeout == 0)
    return vCanRead(hData, id, msgPtr, dlc, flag, time);

  // Set blocking fileop
  if (!hData->readIsBlock) {
    // qqq at this point i dont know why linux wants the '1' but it
    // should be here... Hopefully this won't mess anything up for
    // the wince driver since buflen is 0 /MH
    // The Linux version of os_if_ioctl_* does not use the last
    // parameter at all...   /Johan
    os_if_ioctl_read(hData->fd, VCAN_IOC_SET_READ_BLOCK, (void *)1, 1);
    hData->readIsBlock = 1;
  }

  // Set timeout in milliseconds
  if (hData->readTimeout != timeout) {
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_READ_TIMEOUT, (void *)timeout, 0);
    hData->readTimeout = timeout;
  }

  return vCanReadInternal(hData, id, msgPtr, dlc, flag, time);
}


//======================================================================
// vCanSetBusOutputControl
//======================================================================
canStatus vCanSetBusOutputControl (HandleData *hData, unsigned int drivertype)
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
                          (int *)silent, sizeof(int));

  if (ret != 0)
    return canERR_INVHANDLE;

  return canOK;
}


//======================================================================
// vCanGetBusOutputControl
//======================================================================
canStatus vCanGetBusOutputControl (HandleData *hData, unsigned int *drivertype)
{
  int silent;
  int ret;

  ret = os_if_ioctl_read(hData->fd, VCAN_IOC_GET_OUTPUT_MODE,
                         &silent, sizeof(int));

  if (ret != 0)
    return canERR_INVHANDLE;

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
canStatus vCanAccept(HandleData *hData,
                     const long envelope,
                     const unsigned int flag)
{
  VCanMsgFilter filter;
  int ret;

  // ret = ioctl(hData->fd, VCAN_IOC_GET_MSG_FILTER, &filter);
  ret = os_if_ioctl_read(hData->fd, VCAN_IOC_GET_MSG_FILTER,
                         &filter, sizeof(VCanMsgFilter));
  if (ret != 0)
    return canERR_INVHANDLE;

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
  if (ret != 0)
    return canERR_INVHANDLE;

  return canOK;
}


//======================================================================
// vCanWriteInternal
//======================================================================
canStatus vCanWriteInternal(HandleData *hData, long id, void *msgPtr,
                            unsigned int dlc, unsigned int flag)
{
  CAN_MSG msg;

  int ret;
  unsigned char sendExtended;

  if (flag & canMSG_STD)
    sendExtended = 0;
  else if (flag & canMSG_EXT)
    sendExtended = 1;
  else
    sendExtended = hData->isExtended;

  if  ((sendExtended && (id >= (1 << 29))) ||
       (!sendExtended && (id >= (1 << 11))) ||
       (dlc > 15))
    return canERR_PARAM;

  if (sendExtended) {
    id |= EXT_MSG;
  }
  msg.id = id;
  msg.length = dlc;
  msg.flags = 0;
  if (flag & canMSG_ERROR_FRAME)
    msg.flags |= VCAN_MSG_FLAG_ERROR_FRAME;
  if (flag & canMSG_RTR)
    msg.flags |= VCAN_MSG_FLAG_REMOTE_FRAME;
  if (msgPtr)
    memcpy(msg.data, msgPtr, dlc > 8 ? 8 : dlc);

  // ret = ioctl(hData->fd, VCAN_IOC_SENDMSG, &msg);
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_SENDMSG, &msg, sizeof(CAN_MSG));

#if DEBUG
  if (ret == 0)
    ;
  else if (errno == EAGAIN) {
    DEBUGPRINT((TXT("VCAN_IOC_SENDMSG canERR_TXBUFOFL\n")));
  } else if (errno == EBADMSG) {
    DEBUGPRINT((TXT("VCAN_IOC_SENDMSG canERR_PARAM\n")));
  } else if (errno == EINTR) {
    DEBUGPRINT((TXT("VCAN_IOC_SENDMSG canERR_INTERRUPTED\n")));
  } else {
    DEBUGPRINT((TXT("VCAN_IOC_SENDMSG ERROR: %d\n"), errno));
  }
#endif

  if (ret == 0)
    return canOK;
  else if (errno == EAGAIN)
    return canERR_TXBUFOFL;
  else if (errno == EBADMSG)
    return canERR_PARAM;
  // Interrupted by signal
  else if (errno == EINTR)
    return canERR_INTERRUPTED;
  else
    return canERR_PARAM;
}


//======================================================================
// vCanWrite
//======================================================================
canStatus vCanWrite (HandleData *hData, long id, void *msgPtr,
                     unsigned int dlc, unsigned int flag)
{
  // Set non blocking fileop
  if (hData->writeIsBlock)
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_WRITE_BLOCK, NULL, 0);
  hData->writeIsBlock = 0;

  return vCanWriteInternal(hData, id, msgPtr, dlc, flag);
}


//======================================================================
// vCanWriteWait
//======================================================================
canStatus vCanWriteWait (HandleData *hData, long id, void *msgPtr,
                         unsigned int dlc, unsigned int flag, long timeout)
{
  if (timeout == 0)
    return vCanWrite(hData, id, msgPtr, dlc, flag);

  // Set blocking fileop
  if (!hData->writeIsBlock)
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_WRITE_BLOCK, (void *)1, 0);
  hData->writeIsBlock = 1;

  // Set timeout in milliseconds
  if (hData->writeTimeout != timeout)
    os_if_ioctl_write(hData->fd, VCAN_IOC_SET_WRITE_TIMEOUT, (void *)timeout, 0);
  hData->writeTimeout = timeout;

  return vCanWriteInternal(hData, id, msgPtr, dlc, flag);
}


//======================================================================
// vCanWriteSync
//======================================================================
canStatus vCanWriteSync (HandleData *hData, unsigned long timeout)
{
  int ret;
  ret = os_if_ioctl_write(hData->fd, VCAN_IOC_WAIT_EMPTY,
                          &timeout, sizeof(unsigned long));

  if (ret == 0)
    return canOK;
  switch(errno){
  case EAGAIN:
    return canERR_TIMEOUT;
  case EINTR:
    return canERR_INTERRUPTED;
  default:
    return canERR_NOTFOUND;
  }
}


//======================================================================
// vCanReadTimer
//======================================================================
canStatus vCanReadTimer (HandleData *hData, unsigned long *time)
{
  unsigned long tmpTime;

  if (!time)
    return canERR_PARAM;

  if (os_if_ioctl_read(hData->fd, VCAN_IOC_READ_TIMER,
                       &tmpTime, sizeof(unsigned long))) {
    return canERR_INVHANDLE;
  }
  *time = tmpTime;

  return canOK;
}


//======================================================================
// vCanReadErrorCounters
//======================================================================
canStatus vCanReadErrorCounters (HandleData *hData, unsigned int *txErr,
                                 unsigned int *rxErr, unsigned int *ovErr)
{
  if (txErr != NULL) {
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TX_ERR,
                         txErr, sizeof(unsigned int)))
      goto ioc_error;
  }
  if (rxErr != NULL) {
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_RX_ERR,
                         rxErr, sizeof(unsigned int)))
      goto ioc_error;
  }
  if (ovErr != NULL) {
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_OVER_ERR,
                         ovErr, sizeof(unsigned int)))
      goto ioc_error;
  }

  return canOK;

ioc_error:
  return canERR_INVHANDLE;
}


//======================================================================
// vCanReadStatus
//======================================================================
canStatus vCanReadStatus (HandleData *hData, unsigned long *flags)
{
  int reply;
  if (flags == NULL)
    return canERR_PARAM;

  *flags = 0;

  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_CHIP_STATE, &reply, sizeof(int)))
    goto ioctl_error;
  if (reply & CHIPSTAT_BUSOFF)
    *flags  |= canSTAT_BUS_OFF;
  if (reply & CHIPSTAT_ERROR_PASSIVE)
    *flags  |= canSTAT_ERROR_PASSIVE;
  if (reply & CHIPSTAT_ERROR_WARNING)
    *flags  |= canSTAT_ERROR_WARNING;
  if (reply & CHIPSTAT_ERROR_ACTIVE)
    *flags  |= canSTAT_ERROR_ACTIVE;

  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TX_ERR, &reply, sizeof(int)))
    goto ioctl_error;
  if (reply)
    *flags |= canSTAT_TXERR;
  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_RX_ERR, &reply, sizeof(int)))
    goto ioctl_error;
  if (reply)
    *flags |= canSTAT_RXERR;
  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_OVER_ERR, &reply, sizeof(int)))
    goto ioctl_error;
  if (reply)
    *flags |= canSTAT_SW_OVERRUN | canSTAT_HW_OVERRUN;
  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_RX_QUEUE_LEVEL, &reply, sizeof(int)))
    goto ioctl_error;
  if (reply)
    *flags |= canSTAT_RX_PENDING;
  if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TX_QUEUE_LEVEL, &reply, sizeof(int)))
    goto ioctl_error;
  if (reply)
    *flags |= canSTAT_TX_PENDING;

  return canOK;

ioctl_error:
  return canERR_INVHANDLE;
}


//======================================================================
// vCanGetChannelData
//======================================================================
canStatus vCanGetChannelData (int channel, int item, void *buffer, size_t bufsize)
{
  switch (item) {
  default:
    return canERR_PARAM;
  }
}


//======================================================================
// vCanIoCtl
//======================================================================
canStatus vCanIoCtl(HandleData *hData, unsigned int func,
                    void *buf, unsigned int buflen)
{
  switch(func) {
  case canIOCTL_GET_RX_BUFFER_LEVEL:
    // buf points at a DWORD which receives the current RX queue level.
    if (buf == NULL)
      return canERR_PARAM;
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_RX_QUEUE_LEVEL, buf, buflen))
      return canERR_INVHANDLE;
    break;
  case canIOCTL_GET_TX_BUFFER_LEVEL:
    // buf points at a DWORD which receives the current TX queue level.
    if (buf == NULL)
      return canERR_PARAM;
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TX_QUEUE_LEVEL, buf, buflen))
      return canERR_INVHANDLE;
    break;
  case canIOCTL_FLUSH_RX_BUFFER:
    // Discard the current contents of the RX queue.
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_FLUSH_RCVBUFFER, buf, buflen))
      return canERR_INVHANDLE;
    break;
  case canIOCTL_FLUSH_TX_BUFFER:
    //  Discard the current contents of the TX queue.
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_FLUSH_SENDBUFFER, buf, buflen))
      return canERR_INVHANDLE;
    break;
  case canIOCTL_SET_TXACK:
    // buf points at a DWORD which contains 0/1 to turn TXACKs on/ff
    if (os_if_ioctl_write(hData->fd, VCAN_IOC_SET_TXACK, buf, buflen))
      return canERR_INVHANDLE;
    break;
  case canIOCTL_GET_TXACK:
    // buf points at a DWORD which receives current TXACKs setting
    if (os_if_ioctl_read(hData->fd, VCAN_IOC_GET_TXACK, buf, buflen))
      return canERR_INVHANDLE;
    break;
  default:
    return canERR_PARAM;
  }

  return canOK;
}

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
  .ioCtl               = vCanIoCtl
};

