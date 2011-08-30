/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/* Kvaser Linux Canlib */

#include "canlib.h"

#include "lapcan_data.h"
#include "lapcan_ioctl.h"
#include "lapcmds.h"

#include "canlib_data.h"
#include "canlib_util.h"

#include "osif_common.h"
#include "osif_user.h"
#include "osif_functions_user.h"

#include "VCanFunctions.h"
#include "debug.h"

#include <stdio.h>
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <fcntl.h>
#   include <sys/ioctl.h>
#   include <unistd.h>
#   include <errno.h>
#   include <signal.h>
#   include <pthread.h>
#   include <string.h>


#   if DEBUG
#      define DEBUGPRINT(args) printf args
#   else
#      define DEBUGPRINT(args)
#   endif



static HandleList *handleList;

static const char *errorStrings [] = {

  [-canOK]                    = "No error",
  [-canERR_PARAM]             = "Error in parameter",
  [-canERR_NOMSG]             = "No messages available",
  [-canERR_NOTFOUND]          = "Specified device not found",
  [-canERR_NOMEM]             = "Out of memory",
  [-canERR_NOCHANNELS]        = "No channels avaliable",
  [-canERR_INTERRUPTED]       = "Interrupted by signal",
  [-canERR_TIMEOUT]           = "Timeout occurred",
  [-canERR_NOTINITIALIZED]    = "Library not initialized",
  [-canERR_NOHANDLES]         = "No more handles",
  [-canERR_INVHANDLE]         = "Handle is invalid",
  [-canERR_DRIVER]            = "CAN driver type not supported",
  [-canERR_TXBUFOFL]          = "Transmit buffer overflow",
  [-canERR_HARDWARE]          = "A hardware error was detected",
  [-canERR_DRIVERLOAD]        = "Can not load or open the device driver",
  [-canERR_NOCARD]            = "Card not found"
};

//******************************************************
// Find out channel specific data
//******************************************************
canStatus getDevParams (int channel, char devName[], int *devChannel,
                        CANOps **canOps, char officialName[])
{
  //
  // qqq This function (and canGetNumberOfChannels) has to be modified if we
  // have other cards than LAPcan and pcican, pcicanII, usbcanII and leaf.
  // For now, we just count the number of /dev/lapcan%d, /dev/pcican%d,
  // /dev/pcicanII%d, /dev/usbcanII%d, /dev/leaf%d files,
  // where %d is numbers between 0 and 255
  //

  int         chanCounter = 0;
  int         devCounter  = 0;
  struct stat stbuf;

  // qqq Add here when new driver maybe we can read the dev names
  // from some struct? THIS IS SLOW
  static char *dev_name[] = {"pcican",   "pcicanII", "lapcan",
                             "usbcanII", "leaf", "kvvirtualcan"};
  static char *off_name[] = {"PCIcan",   "PCIcanII", "LAPcan",
                             "USBcanII", "Leaf", "VIRTUALcan"};
  int n = 0;

  for(n = 0; n < sizeof(dev_name) / sizeof(*dev_name); n++) {
    // There are 256 minor inode numbers
    for(devCounter = 0; devCounter <= 255; devCounter++) {
      snprintf(devName, DEVICE_NAME_LEN, "/dev/%s%d", dev_name[n], devCounter);
      //DEBUGPRINT((TXT("looking for '%s' dev: %d| ch: %d\n"), devName,
      //            devCounter, chanCounter));
      if (stat(devName, &stbuf) != -1) {  // Check for existance
        if (chanCounter++ == channel) {
          *devChannel = channel;
          *canOps = &vCanOps;
          sprintf(officialName, "KVASER %s channel %d", off_name[n], devCounter);
          //DEBUGPRINT((TXT("Found %s\n"), officialName));

          return canOK;
        }
      }
      else {
        // This implies that there can be no gaps in the numbering.
        // qqq, What happens if there are several Xs connected and the
        // user disconnect one with low numbers. Does hotplug reenumerate the
        // usbcanIIs and give them new numbers?
        break;
      }
    }
  }

  DEBUGPRINT((TXT("return canERR_NOTFOUND\n")));
  devName[0]  = 0;
  *devChannel = -1;
  *canOps     = NULL;

  return canERR_NOTFOUND;
}


//******************************************************
// Compare handles
//******************************************************
OS_IF_INLINE static int hndCmp (const void *hData1, const void *hData2)
{
  return ((HandleData *)(hData1))->handle ==
         ((HandleData *)(hData2))->handle;
}


//******************************************************
// Find handle in list
//******************************************************
OS_IF_INLINE static HandleData * findHandle (HandleList **handleList, CanHandle hnd)
{
  HandleData dummyHandleData;
  dummyHandleData.handle = hnd;
  return listFind(handleList, &dummyHandleData, &hndCmp);
}


//******************************************************
// Remove handle from list
//******************************************************
OS_IF_INLINE static HandleData * removeHandle (HandleList **handleList, CanHandle hnd)
{
  HandleData dummyHandleData;
  dummyHandleData.handle = hnd;
  return listRemove(handleList, &dummyHandleData, &hndCmp);
}




//
// API FUNCTIONS
//

//******************************************************
// Open a can channel
//******************************************************
CANLIB_API CanHandle __stdcall canOpenChannel (int channel, int flags)
{
  canStatus          status;
  static CanHandle   hnd = 0;
  HandleData         *hData;

  hData = (HandleData *)OS_IF_ALLOC_MEM(sizeof(HandleData));
  if (hData == NULL) {
    DEBUGPRINT((TXT("ERROR: cannot allocate memory:\n")));
    return canERR_NOMEM;
  }

  memset(hData, 0, sizeof(HandleData));

  hData->readIsBlock      = 1;
  hData->writeIsBlock     = 1;
  hData->handle           = hnd;
  hData->isExtended       = flags & canWANT_EXTENDED;
  hData->wantExclusive    = flags & canWANT_EXCLUSIVE;

  status = getDevParams(channel,
                        hData->deviceName,
                        &hData->channelNr,
                        &hData->canOps,
                        hData->deviceOfficialName);

  if (status < 0) {
    DEBUGPRINT((TXT("getDevParams ret %d\n"), status));
    OS_IF_FREE_MEM(hData);
    return status;
  }

  status = hData->canOps->openChannel(hData);
  if (status < 0) {
    DEBUGPRINT((TXT("openChannel ret %d\n"), status));
    OS_IF_FREE_MEM(hData);
    return status;
  }
  listInsertFirst(&handleList, hData);

  return hnd++;
}


//******************************************************
// Close can channel
//******************************************************
CANLIB_API int __stdcall canClose (const CanHandle hnd)
{
  HandleData *hData;
  hData = removeHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;
  canSetNotify(hnd, NULL, 0, NULL);
  if (OS_IF_IS_CLOSE_ERROR(OS_IF_CLOSE_HANDLE(hData->fd)))
    return canERR_INVHANDLE;
  OS_IF_FREE_MEM(hData);

  return canOK;
}


//******************************************************
// Get raw handle/file descriptor to use in system calls
//******************************************************
CANLIB_API canStatus __stdcall canGetRawHandle (const CanHandle hnd, void *pvFd)
{
  HandleData *hData;
  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;
  *(OS_IF_FILE_HANDLE *)pvFd = hData->fd;

  return canOK;
}


//******************************************************
// Go on bus
//******************************************************
CANLIB_API canStatus __stdcall canBusOn (const CanHandle hnd)
{
  HandleData *hData;
  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->busOn(hData);
}


//******************************************************
// Go bus off
//******************************************************
CANLIB_API canStatus __stdcall canBusOff (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;
  return hData->canOps->busOff(hData);
}


//******************************************************
// Try to "reset" the CAN bus.
//******************************************************
CANLIB_API canStatus __stdcall canResetBus(const CanHandle hnd)
{
  canStatus stat;
  unsigned long handle_status;
  
  stat = canReadStatus(hnd, &handle_status);
  if (stat < 0) return stat;
  stat = canBusOff(hnd);
  if (stat < 0) return stat;
  if ((handle_status & canSTAT_BUS_OFF) == 0) {
    stat = canBusOn(hnd);
  }
  return stat;
}



//******************************************************
// Set bus parameters
//******************************************************
CANLIB_API canStatus __stdcall
canSetBusParams (const CanHandle hnd, long freq, unsigned int tseg1,
                 unsigned int tseg2, unsigned int sjw,
                 unsigned int noSamp, unsigned int syncmode)
{
  canStatus ret;
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;
  if (freq < 0) {
    ret = canTranslateBaud(&freq, &tseg1, &tseg2, &sjw, &noSamp, &syncmode);
    if (ret != canOK)
      return ret;
  }

  return hData->canOps->setBusParams(hData, freq, tseg1, tseg2, sjw, noSamp, syncmode);
}

CANLIB_API canStatus __stdcall
canSetBusParamsC200(const CanHandle hnd, unsigned char btr0, unsigned char btr1)
{
  long bitrate;
  unsigned int tseg1;
  unsigned int tseg2;
  unsigned int sjw;
  unsigned int noSamp;
  unsigned int syncmode = 0;

  sjw     = ((btr0 & 0xc0) >> 6) + 1;
  tseg1   = ((btr1 & 0x0f) + 1);
  tseg2   = ((btr1 & 0x70) >> 4) + 1;
  noSamp = ((btr1 & 0x80) >> 7) ? 3:1;
  bitrate = 8000000L * 64 / (((btr0 & 0x3f) +1) << 6) /
                     (tseg1 + tseg2 + 1);

  return canSetBusParams(hnd, bitrate, tseg1, tseg2, sjw, noSamp, syncmode);
}


//******************************************************
// Get bus parameters
//******************************************************
CANLIB_API canStatus __stdcall
canGetBusParams (const CanHandle hnd, long *freq, unsigned int *tseg1,
                 unsigned int *tseg2, unsigned int *sjw,
                 unsigned int *noSamp, unsigned int *syncmode)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->getBusParams(hData, freq, tseg1, tseg2, sjw, noSamp, syncmode);
}


//******************************************************
// Set bus output control (silent/normal)
//******************************************************
CANLIB_API canStatus __stdcall
canSetBusOutputControl (const CanHandle hnd, const unsigned int drivertype)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;
  if (drivertype != canDRIVER_NORMAL && drivertype != canDRIVER_OFF &&
      drivertype != canDRIVER_SILENT && drivertype != canDRIVER_SELFRECEPTION) {
    return canERR_PARAM;
  }

  return hData->canOps->setBusOutputControl(hData, drivertype);
}


//******************************************************
// Get bus output control (silent/normal)
//******************************************************
CANLIB_API canStatus __stdcall
canGetBusOutputControl (const CanHandle hnd, unsigned int * drivertype)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->getBusOutputControl(hData, drivertype);
}


//******************************************************
// Set filters
//******************************************************
CANLIB_API canStatus __stdcall canAccept (const CanHandle hnd,
                                          const long envelope,
                                          const unsigned int flag)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->accept(hData, envelope, flag);
}


//******************************************************
// Read bus status
//******************************************************
CANLIB_API canStatus __stdcall canReadStatus (const CanHandle hnd,
                                              unsigned long *const flags)
{
  HandleData *hData;
  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->readStatus(hData, flags);
}


//******************************************************
// Read the error counters
//******************************************************
CANLIB_API canStatus __stdcall canReadErrorCounters (const CanHandle hnd,
                                                     unsigned int *txErr,
                                                     unsigned int *rxErr,
                                                     unsigned int *ovErr)
{
  HandleData *hData;
  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->readErrorCounters(hData, txErr, rxErr, ovErr);
}


//******************************************************
// Write can message
//******************************************************
CANLIB_API canStatus __stdcall
canWrite (const CanHandle hnd, long id, void *msgPtr,
          unsigned int dlc, unsigned int flag)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;
  return hData->canOps->write(hData, id, msgPtr, dlc, flag);
}


//******************************************************
// Write can message and wait
//******************************************************
CANLIB_API canStatus __stdcall
canWriteWait (CanHandle hnd, long id, void *msgPtr,
              unsigned int dlc, unsigned int flag, long timeout)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->writeWait(hData, id, msgPtr, dlc, flag, timeout);
}


//******************************************************
// Read can message
//******************************************************
CANLIB_API canStatus __stdcall
canRead (const CanHandle hnd, long *id, void *msgPtr, unsigned int *dlc,
         unsigned int *flag, unsigned long *time)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->read(hData, id, msgPtr, dlc, flag, time);
}


//*********************************************************
// Read can message or wait until one appears or timeout
//*********************************************************
CANLIB_API canStatus __stdcall
canReadWait (const CanHandle hnd, long *id, void *msgPtr, unsigned int *dlc,
             unsigned int *flag, unsigned long *time, long timeout)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->readWait(hData, id, msgPtr, dlc, flag, time, timeout);
}


//****************************************************************
// Wait until all can messages on a circuit are sent or timeout
//****************************************************************
CANLIB_API canStatus __stdcall
canWriteSync (const CanHandle hnd, unsigned long timeout)
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->writeSync(hData, timeout);
}


//******************************************************
// IOCTL
//******************************************************
CANLIB_API canStatus __stdcall
canIoCtl (const CanHandle hnd, unsigned int func,
          void *buf, unsigned int buflen)
{
  HandleData *hData;
  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->ioCtl(hData, func, buf, buflen);
}


//******************************************************
// Read the time from hw
//******************************************************
CANLIB_API canStatus __stdcall canReadTimer (CanHandle hnd, unsigned long *time)
{
  HandleData *hData;
  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;

  return hData->canOps->readTimer(hData, time);
}


//******************************************************
// Translate from baud macro to bus params
//******************************************************
CANLIB_API canStatus __stdcall canTranslateBaud (long *const freq,
                                                 unsigned int *const tseg1,
                                                 unsigned int *const tseg2,
                                                 unsigned int *const sjw,
                                                 unsigned int *const nosamp,
                                                 unsigned int *const syncMode)
{
  switch (*freq) {
  case BAUD_1M:
    *freq     = 1000000L;
    *tseg1    = 4;
    *tseg2    = 3;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_500K:
    *freq     = 500000L;
    *tseg1    = 4;
    *tseg2    = 3;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_250K:
    *freq     = 250000L;
    *tseg1    = 4;
    *tseg2    = 3;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_125K:
    *freq     = 125000L;
    *tseg1    = 10;
    *tseg2    = 5;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_100K:
    *freq     = 100000L;
    *tseg1    = 10;
    *tseg2    = 5;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_62K:
    *freq     = 62500L;
    *tseg1    = 10;
    *tseg2    = 5;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  case BAUD_50K:
    *freq     = 50000L;
    *tseg1    = 10;
    *tseg2    = 5;
    *sjw      = 1;
    *nosamp   = 1;
    *syncMode = 0;
    break;

  default:
    return canERR_PARAM;
  }

  return canOK;
}


//******************************************************
// Get error text
//******************************************************
CANLIB_API canStatus __stdcall
canGetErrorText (canStatus err, char *buf, unsigned int bufsiz)
{
  signed char code;

  code = (signed char)(err & 0xFF);

  if (!buf || bufsiz == 0)
    return canOK;
  if ((code <= 0) && (-code < sizeof(errorStrings) / sizeof(char *))) {
    if (errorStrings [-code] == NULL) {
      snprintf(buf, bufsiz, "Unknown error (%d)", (int)code);
    } else {
      strncpy(buf, errorStrings[-code], bufsiz);
    }
  } else {
    strncpy(buf, "This is not an error code", bufsiz);
  }
  buf[bufsiz - 1] = '\0';

  return canOK;
}


//******************************************************
// Get library version
//******************************************************
CANLIB_API unsigned short __stdcall canGetVersion (void)
{
  return (CANLIB_MAJOR_VERSION << 8) + CANLIB_MINOR_VERSION;
}

/*
CANLIB_API unsigned int __stdcall canGetVersionEx (unsigned int itemCode)
{
  unsigned int version = 0;

  switch (itemCode) {
  // build number
  case 0:
    //version = BUILD_NUMBER;
    break;

  default:

    break;
  }

  return version;
}
*/


//******************************************************
// Get the total number of channels
//******************************************************
CANLIB_API canStatus __stdcall canGetNumberOfChannels (int *channelCount)
{
  //
  // qqq This function (and getDevParams) has to be modified if we
  // have other cards than LAPcan and pcican, pcicanII, usbcanII and leaf.
  // For now, we just count the number of /dev/lapcan%d, /dev/pcican%d,
  // /dev/pcicanII%d, /dev/usbcanII%d, /dev/leaf%d files,
  // where %d is numbers between 0 and 255
  //

  int tmpCount = 0;
  int cardNr;
  char filename[DEVICE_NAME_LEN + 1];
  int n = 0;
  // qqq Add here when new driver maybe we can read the dev names
  // from some struct? THIS IS SLOW
  static char *dev_name[] = {"lapcan",   "pcican", "pcicanII",
                             "usbcanII", "leaf", "kvvirtualcan"};

  for(n = 0; n < sizeof(dev_name) / sizeof(*dev_name); n++) {
    // There are 256 minor inode numbers
    for(cardNr = 0; cardNr <= 255; cardNr++) {
      snprintf(filename,  DEVICE_NAME_LEN, "/dev/%s%d", dev_name[n], cardNr);
      if (os_if_access(filename, F_OK) == 0) {  // Check for existance
        tmpCount++;
      }
      else {
        // This implies that there can be no gaps in the numbering.
        // qqq, What happens if there are several Xs connected and the
        // user disconnect one with low numbers. Does hotplug reenumerate the
        // usbcanIIs and give them new numbers?
        break;
      }
    }
  }

  *channelCount = tmpCount;

  return canOK;
}


//******************************************************
// Find out channel specific data
//******************************************************
CANLIB_API canStatus __stdcall
canGetChannelData (int channel, int item, void *buffer, size_t bufsize)
{
  canStatus status;
  HandleData hData;
  status = getDevParams(channel, hData.deviceName, &hData.channelNr,
                        &hData.canOps, hData.deviceOfficialName);

  if (status < 0)
    return status;

  switch(item) {
  case canCHANNELDATA_CHANNEL_NAME:
    strcpy(buffer, hData.deviceOfficialName);
    bufsize = strlen(hData.deviceOfficialName);
    return canOK;

  default:
    return hData.canOps->getChannelData(channel, item, buffer, bufsize);
  }
}


//******************************************************
// Set notification callback
//******************************************************
CANLIB_API canStatus __stdcall
canSetNotify (const CanHandle hnd, void (*callback)(canNotifyData *),
              unsigned int notifyFlags, void *tag)
//
// Notification is done by filtering out interesting messages and
// doing a blocked read from a thread
//
{
  HandleData *hData;

  hData = findHandle(&handleList, hnd);
  if (hData == NULL)
    return canERR_INVHANDLE;
  if (notifyFlags == 0 || callback == NULL) {
    // We want to shut off notification, close file and clear callback
    pthread_cancel((pthread_t)hData->notifyThread);

    // Wait for thread to finish
    pthread_join((pthread_t)hData->notifyThread, NULL);
    if (hData -> notifyFd != 0) {
      OS_IF_CLOSE_HANDLE(hData->notifyFd);
    }
    hData->notifyFd = 0;
    callback = NULL;
    return canOK;
  }
  hData->notifyData.tag = tag;

  return hData->canOps->setNotify(hData, callback, notifyFlags);
}


//******************************************************
// Initialize library
//******************************************************
CANLIB_API void __stdcall canInitializeLibrary (void)
{
  return;
}
