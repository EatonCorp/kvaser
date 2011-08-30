/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/* Kvaser Linux Canlib */

#ifndef _CANLIB_DATA_H_
#define _CANLIB_DATA_H_


#include "linkedlist.h"

#include "osif_user.h"

#if !LINUX
#include "osif_kernel.h"
#include "vcanevt.h"
#endif

#define N_O_CHANNELS 2

typedef LinkedList HandleList;

#include <canlib.h>
#include <canlib_version.h>

#define LAPCAN_TICKS_PER_MS 125

struct CANops;

// This struct is associated with each handle
// returned by canOpenChannel
typedef struct HandleData
{
  OS_IF_FILE_HANDLE fd;
  char               deviceName[DEVICE_NAME_LEN];
  char               deviceOfficialName[150];
  int                channelNr; // Absolute ch nr i.e. it can be >2 for lapcan
#if 0
  unsigned int       serialNo;
#endif
  canHandle          handle;
  unsigned char      isExtended;
  unsigned char      wantExclusive;
  unsigned char      acceptVirtual;
  unsigned char      readIsBlock;
  unsigned char      writeIsBlock;
  long               readTimeout;
  long               writeTimeout;
  unsigned long      currentTime;
  DWORD              timerResolution;
  double             timerScale;
#if LINUX
  void               (*callback)(canNotifyData *);
  void               (*callback2)(CanHandle hnd, void* ctx, unsigned int event);
#else
  HWND               notify_hwnd;
  HANDLE             notify_die;
  int                notified;
  OS_IF_EVENT        notify_ack;
  VCAN_EVENT         notify_msg;

  HANDLE             notification_event;
#endif
  canNotifyData      notifyData;
  OS_IF_FILE_HANDLE  notifyFd;
#if LINUX
  pthread_t          notifyThread;
#endif
  unsigned int       notifyFlags;
  struct CANOps      *canOps;
} HandleData;


/* Hardware dependent functions that do the actual work with the card
 * The functions are given a HandleData struct */
//typedef struct HWOps
typedef struct CANOps
{
  /* Read a channel and flags e.g canWANT_EXCLUSIVE and return a file descriptor
   * to read messages from.
   */
  canStatus (*openChannel)(HandleData *);
  /* Read a callback function and flags that defines which events triggers it */
  canStatus (*setNotify)(HandleData *,
             /*void (*callback)(canNotifyData *)*/
             OS_IF_SET_NOTIFY_PARAM, unsigned int);
  canStatus (*busOn)(HandleData *);
  canStatus (*busOff)(HandleData *);

  canStatus (*setBusParams)(HandleData *, long, unsigned int, unsigned int,
                            unsigned int, unsigned int, unsigned int);
  canStatus (*getBusParams)(HandleData *, long *, unsigned int *, unsigned int *,
                            unsigned int *, unsigned int *, unsigned int *);
  canStatus (*read)(HandleData *, long *, void *, unsigned int *,
                    unsigned int *, unsigned long *);
  canStatus (*readWait)(HandleData *, long *, void *, unsigned int *,
                        unsigned int *, unsigned long *, long);
  canStatus (*setBusOutputControl)(HandleData *, unsigned int);
  canStatus (*getBusOutputControl)(HandleData *, unsigned int *);
  canStatus (*accept)(HandleData *, const long, const unsigned int);
  canStatus (*write)(HandleData *, long, void *, unsigned int, unsigned int);
  canStatus (*writeWait)(HandleData *, long, void *,
                         unsigned int, unsigned int, long);
  canStatus (*writeSync)(HandleData *, unsigned long);
  canStatus (*getNumberOfChannels)(HandleData *, int *);
  canStatus (*readTimer)(HandleData *, unsigned long *);
  canStatus (*readErrorCounters)(HandleData *, unsigned int *,
                                 unsigned int *, unsigned int *);
  canStatus (*readStatus)(HandleData *, unsigned long *);
  canStatus (*getChannelData)(char *, int, void *, size_t);
  canStatus (*ioCtl)(HandleData * , unsigned int, void *, size_t);
  canStatus (*objbufFreeAll)(HandleData *hData);
  canStatus (*objbufAllocate)(HandleData *hData, int type, int *number);
  canStatus (*objbufFree)(HandleData *hData, int idx);
  canStatus (*objbufWrite)(HandleData *hData, int idx, int id, void* msg,
                           unsigned int dlc, unsigned int flags);
  canStatus (*objbufSetFilter)(HandleData *hData, int idx,
                               unsigned int code, unsigned int mask);
  canStatus (*objbufSetFlags)(HandleData *hData, int idx, unsigned int flags);
  canStatus (*objbufSetPeriod)(HandleData *hData, int idx, unsigned int period);
  canStatus (*objbufSetMsgCount)(HandleData *hData, int idx, unsigned int count);
  canStatus (*objbufSendBurst)(HandleData *hData, int idx, unsigned int burstLen);
  canStatus (*objbufEnable)(HandleData *hData, int idx);
  canStatus (*objbufDisable)(HandleData *hData, int idx);
//} HWOps;
} CANOps;

//typedef struct CanBusTiming

#endif
