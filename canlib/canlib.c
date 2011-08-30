/*
** Copyright 2002-2009 KVASER AB, Sweden.  All rights reserved.
*/

/* Kvaser Linux Canlib */

#include "canlib.h"

#include "canlib_data.h"

#include "osif_user.h"
#include "osif_functions_user.h"

#include "vcan_ioctl.h"    // Need this for IOCtl to check # channels
#include "vcanevt.h"

#include "VCanFunctions.h"
#include "debug.h"

#include <stdio.h>
#if LINUX
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <fcntl.h>
#   include <unistd.h>
#   include <errno.h>
#   include <signal.h>
#   include <pthread.h>
#endif

#include <string.h>


#if LINUX
#   if DEBUG
#      define DEBUGPRINT(args) printf args
#   else
#      define DEBUGPRINT(args)
#   endif
#else
#   if DEBUG
#      define DEBUGPRINT(args) DEBUGOUT(1, args)
#   else
#      define DEBUGPRINT(args)
#   endif
#endif



static const char *errorStrings[] = {
  "No error",                        // canOK
  "Error in parameter",              // canERR_PARAM
  "No messages available",           // canERR_NOMSG
  "Specified device not found",      // canERR_NOTFOUND
  "Out of memory",                   // canERR_NOMEM
  "No channels available",           // canERR_NOCHANNELS
  "Interrupted by signal",           // canERR_INTERRUPTED
  "Timeout occurred",                // canERR_TIMEOUT
  "Library not initialized",         // canERR_NOTINITIALIZED
  "No more handles",                 // canERR_NOHANDLES
  "Handle is invalid",               // canERR_INVHANDLE
  "Unknown error (-11)",             // canERR_INIFILE
  "CAN driver type not supported",   // canERR_DRIVER
  "Transmit buffer overflow",        // canERR_TXBUFOFL
  "Unknown error (-14)",             // canERR_RESERVED_1
  "A hardware error was detected",   // canERR_HARDWARE
  "Can not find requested DLL",      // canERR_DYNALOAD
  "DLL seems to be wrong version",   // canERR_DYNALIB
  "Error initializing DLL or driver", // canERR_DYNAINIT
  "Operation not supported by hardware or firmware", // canERR_NOT_SUPPORTED
  "Unknown error (-20)",             // canERR_RESERVED_5
  "Unknown error (-21)",             // canERR_RESERVED_6
  "Unknown error (-22)",             // canERR_RESERVED_2
  "Can not load or open the device driver", // canERR_DRIVERLOAD
  "The I/O request failed, probably due to resource shortage", //canERR_DRIVERFAILED
  "Unknown error (-25)",             // canERR_NOCONFIGMGR
  "Card not found"                   // canERR_NOCARD
  "Unknown error (-27)",             // canERR_RESERVED_7
  "Config not found",                // canERR_REGISTRY
  "The license is not valid",        // canERR_LICENSE
  "Internal error in the driver",    // canERR_INTERNAL
  "Access denied",                   // canERR_NO_ACCESS
  "Not implemented"                  // canERR_NOT_IMPLEMENTED
};

// qqq This has to be modified if we add/remove drivers.
static const char *dev_name[] = {"lapcan",   "pcican",   "pcicanII",
                                 "usbcanII", "leaf",     "kvvirtualcan"};
static const char *off_name[] = {"LAPcan",   "PCIcan",   "PCIcanII",
                                 "USBcanII", "Leaf",     "VIRTUALcan"};


#if LINUX
//******************************************************
// Find out channel specific data
//******************************************************
static
canStatus getDevParams (int channel, char devName[], int *devChannel,
                        CANOps **canOps, char officialName[])
{
  // For now, we just count the number of /dev/%s%d files (see dev_name),
  // where %d is numbers between 0 and 255.
  // This is slow!
  // qqq Maybe we can read the dev names from some struct?

  int         chanCounter = 0;
  int         devCounter  = 0;
  struct stat stbuf;

  int n = 0;

  int CardNo          = -1;
  int ChannelNoOnCard = 0;
  int ChannelsOnCard  = 0;
  int err;
  OS_IF_FILE_HANDLE fd;
  
  for(n = 0; n < sizeof(dev_name) / sizeof(*dev_name); n++) {
    CardNo = -1;
    ChannelNoOnCard = 0;
    ChannelsOnCard = 0;

    // There are 256 minor inode numbers
    for(devCounter = 0; devCounter <= 255; devCounter++) {
      snprintf(devName, DEVICE_NAME_LEN, "/dev/%s%d", dev_name[n], devCounter);
#if 0
      DEBUGPRINT((TXT("looking for '%s' dev: %d| ch: %d\n"), devName,
                  devCounter, chanCounter));
#endif
      if (stat(devName, &stbuf) != -1) {  // Check for existance

        if (!ChannelsOnCard) {
          err = 1;
          fd = open(devName, O_RDONLY);
          if (fd != -1) {
            err = os_if_ioctl_read(fd, VCAN_IOC_GET_NRCHANNELS,
                                   &ChannelsOnCard, sizeof(ChannelsOnCard));
            close(fd);
          }
          if (err) {
            ChannelsOnCard = 1;
          } else {
            ChannelNoOnCard = 0;
             // qqq, CardNo for a given card will decrease if
             //      a card with lower No is removed.
            CardNo++;
          }
        } else {
          ChannelNoOnCard++;
        }
        ChannelsOnCard--;

        if (chanCounter++ == channel) {
          *devChannel = channel;
          *canOps = &vCanOps;
          sprintf(officialName, "KVASER %s channel %d", off_name[n], devCounter);
          *devChannel = ChannelNoOnCard;
          
          return canOK;
        }
      }
      else {
        // This implies that there can be no gaps in the numbering.
        // qqq What happens if there are several Xs connected and the
        //     user disconnects one with low number. Does hotplug reenumerate
        //     the usbcanIIs and give them new numbers?
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
#else
extern canStatus getDevParams (int channel, char devName[], int *devChannel,
                        CANOps **canOps, char officialName[]);
#endif




//
// API FUNCTIONS
//

//******************************************************
// Open a can channel
//******************************************************
CanHandle CANLIBAPI canOpenChannel (int channel, int flags)
{
  canStatus          status;
  HandleData         *hData;
  CanHandle          hnd;

  hData = (HandleData *)OS_IF_ALLOC_MEM(sizeof(HandleData));
  if (hData == NULL) {
    DEBUGPRINT((TXT("ERROR: cannot allocate memory (%d)\n"),
                (int)sizeof(HandleData)));
    return canERR_NOMEM;
  }

  memset(hData, 0, sizeof(HandleData));

  hData->readIsBlock      = 1;
  hData->writeIsBlock     = 1;
  hData->isExtended       = flags & canOPEN_REQUIRE_EXTENDED;
  hData->wantExclusive    = flags & canOPEN_EXCLUSIVE;
  hData->acceptVirtual    = flags & canOPEN_ACCEPT_VIRTUAL;
  hData->notifyFd         = OS_IF_INVALID_HANDLE;

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

  hnd = insertHandle(hData);

  if (hnd < 0) {
    DEBUGPRINT((TXT("insertHandle ret %d\n"), hnd));
    OS_IF_CLOSE_HANDLE(hData->fd);
    OS_IF_FREE_MEM(hData);
    return canERR_NOMEM;
  }

  return hnd;
}


#if LINUX
//******************************************************
// Close can channel
//******************************************************
int CANLIBAPI canClose (const CanHandle hnd)
{
  HandleData *hData;

  // Try to go Bus Off before closing
  canBusOff(hnd);

  hData = removeHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  canSetNotify(hnd, NULL, 0, NULL);
  if (OS_IF_IS_CLOSE_ERROR(OS_IF_CLOSE_HANDLE(hData->fd))) {
    return canERR_INVHANDLE;
  }

  OS_IF_FREE_MEM(hData);

  return canOK;
}
#endif


//******************************************************
// Get raw handle/file descriptor to use in system calls
//******************************************************
canStatus CANLIBAPI canGetRawHandle (const CanHandle hnd, void *pvFd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  *(OS_IF_FILE_HANDLE *)pvFd = hData->fd;

  return canOK;
}


//******************************************************
// Go on bus
//******************************************************
canStatus CANLIBAPI canBusOn (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->busOn(hData);
}


//******************************************************
// Go bus off
//******************************************************
canStatus CANLIBAPI canBusOff (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->busOff(hData);
}


//******************************************************
// Try to "reset" the CAN bus.
//******************************************************
canStatus CANLIBAPI canResetBus (const CanHandle hnd)
{
  canStatus stat;
  unsigned long handle_status;

  stat = canReadStatus(hnd, &handle_status);
  if (stat < 0) {
    return stat;
  }
  stat = canBusOff(hnd);
  if (stat < 0) {
    return stat;
  }
  if ((handle_status & canSTAT_BUS_OFF) == 0) {
    stat = canBusOn(hnd);
  }

  return stat;
}



//******************************************************
// Set bus parameters
//******************************************************
canStatus CANLIBAPI
canSetBusParams (const CanHandle hnd, long freq, unsigned int tseg1,
                 unsigned int tseg2, unsigned int sjw,
                 unsigned int noSamp, unsigned int syncmode)
{
  canStatus ret;
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (freq < 0) {
    ret = canTranslateBaud(&freq, &tseg1, &tseg2, &sjw, &noSamp, &syncmode);
    if (ret != canOK) {
      return ret;
    }
  }

  return hData->canOps->setBusParams(hData, freq, tseg1, tseg2,
                                     sjw, noSamp, syncmode);
}

canStatus CANLIBAPI
canSetBusParamsC200 (const CanHandle hnd, unsigned char btr0, unsigned char btr1)
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
  noSamp  = ((btr1 & 0x80) >> 7) ? 3 : 1;
  bitrate = 8000000L * 64 / (((btr0 & 0x3f) + 1) << 6) /
            (tseg1 + tseg2 + 1);

  return canSetBusParams(hnd, bitrate, tseg1, tseg2, sjw, noSamp, syncmode);
}


//******************************************************
// Get bus parameters
//******************************************************
canStatus CANLIBAPI
canGetBusParams (const CanHandle hnd, long *freq, unsigned int *tseg1,
                 unsigned int *tseg2, unsigned int *sjw,
                 unsigned int *noSamp, unsigned int *syncmode)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->getBusParams(hData, freq, tseg1, tseg2,
                                     sjw, noSamp, syncmode);
}


//******************************************************
// Set bus output control (silent/normal)
//******************************************************
canStatus CANLIBAPI
canSetBusOutputControl (const CanHandle hnd, const unsigned int drivertype)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (drivertype != canDRIVER_NORMAL && drivertype != canDRIVER_OFF &&
      drivertype != canDRIVER_SILENT && drivertype != canDRIVER_SELFRECEPTION) {
    return canERR_PARAM;
  }

  return hData->canOps->setBusOutputControl(hData, drivertype);
}


//******************************************************
// Get bus output control (silent/normal)
//******************************************************
canStatus CANLIBAPI
canGetBusOutputControl (const CanHandle hnd, unsigned int * drivertype)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->getBusOutputControl(hData, drivertype);
}


//******************************************************
// Set filters
//******************************************************
canStatus CANLIBAPI canAccept (const CanHandle hnd,
                               const long envelope,
                               const unsigned int flag)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->accept(hData, envelope, flag);
}


//******************************************************
// Read bus status
//******************************************************
canStatus CANLIBAPI canReadStatus (const CanHandle hnd,
                                   unsigned long *const flags)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->readStatus(hData, flags);
}


//******************************************************
// Read the error counters
//******************************************************
canStatus CANLIBAPI canReadErrorCounters (const CanHandle hnd,
                                          unsigned int *txErr,
                                          unsigned int *rxErr,
                                          unsigned int *ovErr)
{
  HandleData *hData;
  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->readErrorCounters(hData, txErr, rxErr, ovErr);
}


//******************************************************
// Write can message
//******************************************************
canStatus CANLIBAPI
canWrite (const CanHandle hnd, long id, void *msgPtr,
          unsigned int dlc, unsigned int flag)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->write(hData, id, msgPtr, dlc, flag);
}


//******************************************************
// Write can message and wait
//******************************************************
canStatus CANLIBAPI
canWriteWait (const CanHandle hnd, long id, void *msgPtr,
              unsigned int dlc, unsigned int flag, unsigned long timeout)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->writeWait(hData, id, msgPtr, dlc, flag, timeout);
}


//******************************************************
// Read can message
//******************************************************
canStatus CANLIBAPI
canRead (const CanHandle hnd, long *id, void *msgPtr, unsigned int *dlc,
         unsigned int *flag, unsigned long *time)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->read(hData, id, msgPtr, dlc, flag, time);
}


//*********************************************************
// Read can message or wait until one appears or timeout
//*********************************************************
canStatus CANLIBAPI
canReadWait (const CanHandle hnd, long *id, void *msgPtr, unsigned int *dlc,
             unsigned int *flag, unsigned long *time, unsigned long timeout)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->readWait(hData, id, msgPtr, dlc, flag, time, timeout);
}


//****************************************************************
// Wait until all can messages on a circuit are sent or timeout
//****************************************************************
canStatus CANLIBAPI
canWriteSync (const CanHandle hnd, unsigned long timeout)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->writeSync(hData, timeout);
}


//******************************************************
// IOCTL
//******************************************************
canStatus CANLIBAPI
canIoCtl (const CanHandle hnd, unsigned int func,
          void *buf, unsigned int buflen)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->ioCtl(hData, func, buf, buflen);
}


//******************************************************
// Read the time from hw
//******************************************************
canStatus CANLIBAPI canReadTimer (const CanHandle hnd, unsigned long *time)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->readTimer(hData, time);
}


//******************************************************
// Translate from baud macro to bus params
//******************************************************
canStatus CANLIBAPI canTranslateBaud (long *const freq,
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

  case canBITRATE_83K:
    *freq     = 83333L;
    *tseg1    = 5;
    *tseg2    = 2;
    *sjw      = 2;
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

  case canBITRATE_10K:
    *freq     = 10000L;
    *tseg1    = 11;
    *tseg2    = 4;
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
canStatus CANLIBAPI
canGetErrorText (canStatus err, char *buf, unsigned int bufsiz)
{
  signed char code;

  code = (signed char)(err & 0xFF);

  if (!buf || bufsiz == 0) {
    return canOK;
  }
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
unsigned short CANLIBAPI canGetVersion (void)
{
  return (CANLIB_MAJOR_VERSION << 8) + CANLIB_MINOR_VERSION;
}

/*
unsigned int CANLIBAPI canGetVersionEx (unsigned int itemCode)
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


#if LINUX
//******************************************************
// Get the total number of channels
//******************************************************
canStatus CANLIBAPI canGetNumberOfChannels (int *channelCount)
{
  // For now, we just count the number of /dev/%s%d files (see dev_name),
  // where %d is numbers between 0 and 255.
  // This is slow!

  int tmpCount = 0;
  int cardNr;
  char filename[DEVICE_NAME_LEN];
  int n = 0;

  for(n = 0; n < sizeof(dev_name) / sizeof(*dev_name); n++) {
    // There are 256 minor inode numbers
    for(cardNr = 0; cardNr <= 255; cardNr++) {
      snprintf(filename,  DEVICE_NAME_LEN, "/dev/%s%d", dev_name[n], cardNr);
      if (os_if_access(filename, F_OK) == 0) {  // Check for existance
        tmpCount++;
      }
      else {
        // This implies that there can be no gaps in the numbering.
        // qqq What happens if there are several Xs connected and the
        //     user disconnects one with low number. Does hotplug reenumerate
        //     the usbcanIIs and give them new numbers?
        break;
      }
    }
  }

  *channelCount = tmpCount;

  return canOK;
}
#endif


//******************************************************
// Find out channel specific data
//******************************************************
canStatus CANLIBAPI
canGetChannelData (int channel, int item, void *buffer, size_t bufsize)
{
  canStatus status;
  HandleData hData;

  status = getDevParams(channel, hData.deviceName, &hData.channelNr,
                        &hData.canOps, hData.deviceOfficialName);

  if (status < 0) {
    return status;
  }
  
  switch(item) {
  case canCHANNELDATA_CHANNEL_NAME:
    strcpy(buffer, hData.deviceOfficialName);
    bufsize = strlen(hData.deviceOfficialName);
    return canOK;

  case canCHANNELDATA_CHAN_NO_ON_CARD:
    bufsize = sizeof(hData.channelNr);
    memcpy(buffer, &hData.channelNr, bufsize);
    return canOK;

  default:
    return hData.canOps->getChannelData(hData.deviceName, item, buffer, bufsize);
  }
}


//===========================================================================
canStatus CANLIBAPI canObjBufFreeAll (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufFreeAll(hData);
}


//===========================================================================
canStatus CANLIBAPI canObjBufAllocate (const CanHandle hnd, int type)
{
  HandleData *hData;
  int number;
  canStatus status;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  status = hData->canOps->objbufAllocate(hData, type, &number);

  return (status == canOK) ? number : status;
}


//===========================================================================
canStatus CANLIBAPI canObjBufFree (const CanHandle hnd, int idx)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufFree(hData, idx);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufWrite (const CanHandle hnd, int idx, int id, void *msg,
                unsigned int dlc, unsigned int flags)
{
  HandleData *hData;
  unsigned int canio_flags = 0;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

#if 0   // qqq Add support for HOPTION_ACCEPT_LARGE_DLC!
  if (!(hData->hndOptions & HOPTION_ACCEPT_LARGE_DLC) && (dlc > 8)) {
    return canERR_PARAM;
  }
#endif

  if (flags & canMSG_EXT) {
    id |= EXT_MSG;
  }
  if (flags & canMSG_RTR) {
    canio_flags |= VCAN_MSG_FLAG_REMOTE_FRAME;
  }

  return hData->canOps->objbufWrite(hData, idx, id, msg, dlc, canio_flags);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSetFilter (const CanHandle hnd, int idx,
                    unsigned int code, unsigned int mask)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSetFilter(hData, idx, code, mask);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSetFlags (const CanHandle hnd, int idx, unsigned int flags)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSetFlags(hData, idx, flags);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSetPeriod (const CanHandle hnd, int idx, unsigned int period)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSetPeriod(hData, idx, period);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSetMsgCount (const CanHandle hnd, int idx, unsigned int count)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSetMsgCount(hData, idx, count);
}


//===========================================================================
canStatus CANLIBAPI
canObjBufSendBurst (const CanHandle hnd, int idx, unsigned int burstLen)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufSendBurst(hData, idx, burstLen);
}


//===========================================================================
canStatus CANLIBAPI canObjBufEnable (const CanHandle hnd, int idx)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufEnable(hData, idx);
}


//===========================================================================
canStatus CANLIBAPI canObjBufDisable (const CanHandle hnd, int idx)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->objbufDisable(hData, idx);
}


//******************************************************
// Flush receive queue
//******************************************************
canStatus CANLIBAPI
canFlushReceiveQueue (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->ioCtl(hData, canIOCTL_FLUSH_RX_BUFFER, NULL, 0);
}


//******************************************************
// Flush transmit queue
//******************************************************
canStatus CANLIBAPI
canFlushTransmitQueue (const CanHandle hnd)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }

  return hData->canOps->ioCtl(hData, canIOCTL_FLUSH_TX_BUFFER, NULL, 0);
}


#if LINUX
//******************************************************
// Set notification callback
//******************************************************
canStatus CANLIBAPI
canSetNotify (const CanHandle hnd, void (*callback)(canNotifyData *),
              unsigned int notifyFlags, void *tag)
//
// Notification is done by filtering out interesting messages and
// doing a blocked read from a thread.
//
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (notifyFlags == 0 || callback == NULL) {
    if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
      // We want to shut off notification, close file and clear callback

      pthread_cancel(hData->notifyThread);

      // Wait for thread to finish
      pthread_join(hData->notifyThread, NULL);

      if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
        OS_IF_CLOSE_HANDLE(hData->notifyFd);
      }
      hData->notifyFd = OS_IF_INVALID_HANDLE;
    }

    return canOK;
  }

  hData->notifyData.tag = tag;

  return hData->canOps->setNotify(hData, callback, NULL, notifyFlags);
}

kvStatus CANLIBAPI kvSetNotifyCallback(const CanHandle hnd,
                                       kvCallback_t callback, void* context,
                                       unsigned int notifyFlags)
{
  HandleData *hData;

  hData = findHandle(hnd);
  if (hData == NULL) {
    return canERR_INVHANDLE;
  }
  if (notifyFlags == 0 || callback == NULL) {
    if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
      // We want to shut off notification, close file and clear callback

      pthread_cancel(hData->notifyThread);

      // Wait for thread to finish
      pthread_join(hData->notifyThread, NULL);

      if (hData->notifyFd != OS_IF_INVALID_HANDLE) {
        OS_IF_CLOSE_HANDLE(hData->notifyFd);
      }
      hData->notifyFd = OS_IF_INVALID_HANDLE;
    }

    return canOK;
  }

  hData->notifyData.tag = context;

  return hData->canOps->setNotify(hData, NULL, callback, notifyFlags);
}
#endif


//******************************************************
// Initialize library
//******************************************************
void CANLIBAPI canInitializeLibrary (void)
{
  return;
}
