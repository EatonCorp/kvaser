/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*
** File:
**   vcan.h
** Project:
**   linuxcan
** Description:
**   common driver data structures.
*/

#ifndef _VCAN_OS_IF_H_
#define _VCAN_OS_IF_H_


#if LINUX
#   include <linux/poll.h>
#   include <asm/atomic.h>
#   include <linux/types.h>
#else // WIN32
#   include "windows.h"

typedef struct _USBCAN_CONTEXT USBCAN_CONTEXT;
typedef unsigned long uint32_t;
#endif

#include "canIfData.h"
#include "vcanevt.h"
#include "objbuf.h"

#include "osif_common.h"
#include "osif_kernel.h"
#include "queue.h"


/*****************************************************************************/
/*  Defines                                                                  */
/*****************************************************************************/

#define MAIN_RCV_BUF_SIZE  16
#define FILE_RCV_BUF_SIZE 500
#define TX_CHAN_BUF_SIZE  500

/*****************************************************************************/
/*  From canlib30/Src/CANLIB32/2K/include/vcanio.h                           */
/*****************************************************************************/

#define CAN_EXT_MSG_ID                  0x80000000

#define CAN_BUSSTAT_BUSOFF              0x01
#define CAN_BUSSTAT_ERROR_PASSIVE       0x02
#define CAN_BUSSTAT_ERROR_WARNING       0x04
#define CAN_BUSSTAT_ERROR_ACTIVE        0x08
#define CAN_BUSSTAT_BUSOFF_RECOVERY     0x10
#define CAN_BUSSTAT_IGNORING_ERRORS     0x20

#define CAN_CHIP_TYPE_UNKNOWN           0
#define CAN_CHIP_TYPE_VIRTUAL           1
#define CAN_CHIP_TYPE_SJA1000           2
#define CAN_CHIP_TYPE_527               3
#define CAN_CHIP_TYPE_C200              4


/*****************************************************************************/
/*  From below                                                               */
/*****************************************************************************/

#define put_user_ret(x,ptr,ret)              \
  { if (os_if_set_int(x,ptr)) return ret; }
#define get_user_int_ret(x,ptr,ret)          \
  { if (os_if_get_int(x,ptr)) return ret; }
#define get_user_long_ret(x,ptr,ret)         \
  { if (os_if_get_long(x,ptr)) return ret; }
#define copy_to_user_ret(to,from,n,retval)   \
  { if (os_if_get_user_data(to,from,n)) return retval; }
#define copy_from_user_ret(to,from,n,retval) \
  { if (os_if_set_user_data(to,from,n)) return retval; }

#if LINUX
#define ArgPtrIn(s)
#define ArgPtrOut(s)
#define ArgIntIn     do {                                   \
                       int argh;                            \
                       get_user_int_ret(&argh, (int *)arg,  \
                                        -EFAULT);           \
                       arg = argh;                          \
                     } while (0)

#else
#define ArgPtrIn(s)  do { if (pBufIn && (dwLenIn >= s)) {   \
                            arg = (long)pBufIn;             \
                          } else {                          \
                            return -EINVAL;                 \
                          }                                 \
                        } while(0)
#define ArgPtrOut(s) do { if (pBufOut && (dwLenOut >= s)) { \
                            arg = (long)pBufOut;            \
                            *pdwActualOut = s;              \
                          } else {                          \
                            return -EINVAL;                 \
                          }                                 \
                        } while(0)
#if 1
#define ArgIntIn     do { if (pBufIn && (dwLenIn == sizeof(int))) {   \
                            get_user_int_ret(&arg, (int *)pBufIn, -EFAULT); \
                          } else {                          \
                            return -EFAULT;                 \
                          }                                 \
                        } while(0)
#else
#define ArgIntIn     do { arg = (int)pBufIn; } while(0)
#endif
#endif


#define VCAN_STAT_OK                 0
#define VCAN_STAT_FAIL              -1    // -EIO
#define VCAN_STAT_TIMEOUT           -2    // -EAGAIN (TIMEDOUT)?
#define VCAN_STAT_NO_DEVICE         -3    // -ENODEV
#define VCAN_STAT_NO_RESOURCES      -4    // -EAGAIN
#define VCAN_STAT_NO_MEMORY         -5    // -ENOMEM
#define VCAN_STAT_SIGNALED          -6    // -ERESTARTSYS
#define VCAN_STAT_BAD_PARAMETER     -7    // -EINVAL

/*****************************************************************************/
/*  Data structures                                                          */
/*****************************************************************************/

typedef union {
    uint32_t L;
    struct { unsigned short w0, w1; } W;
    struct { unsigned char b0, b1, b2, b3; } B;
} WL;

typedef struct CanChipState {
    int state;  /* buson / busoff / error passive / warning */
    int txerr;  /* tx error counter */
    int rxerr;  /* rx error counter */
} CanChipState;


/* Channel specific data */
typedef struct VCanChanData
{
    int                      minorNr;
    unsigned char            channel;
    unsigned char            chipType;
    unsigned char            ean[8];
    uint32_t                 serialHigh;
    uint32_t                 serialLow;

    /* Status */
    unsigned char            isOnBus;
    unsigned char            transType;   // TRANSCEIVER_TYPE_xxx
    unsigned char            lineMode;    // TRANSCEIVER_LINEMODE_xxx
    unsigned char            resNet;      // TRANSCEIVER_RESNET_xxx
    atomic_t                 transId;
    atomic_t                 chanId;
    unsigned int             overrun;
    CanChipState             chipState;
    unsigned int             errorCount;
    uint32_t                 errorTime;
    unsigned char            rxErrorCounter;
    unsigned char            txErrorCounter;

    /* Transmit buffer */
    CAN_MSG                  txChanBuffer[TX_CHAN_BUF_SIZE];
    CAN_MSG                 *currentTxMsg;
    Queue                    txChanQueue;

    /* Processes waiting for all messages to be sent */
    OS_IF_WAITQUEUE_HEAD     flushQ;

    atomic_t                 fileOpenCount;
    struct VCanOpenFileNode *openFileList;


    OS_IF_LOCK              openLock;
    void                   *hwChanData;
#if 0
    atomic_t                waitEmpty;
#else
    OS_IF_ATOMIC_BIT        waitEmpty;
#endif

    struct VCanCardData    *vCard;
} VCanChanData;


// For VCanCardData->card_flags
#define DEVHND_CARD_FIRMWARE_BETA       0x01  // Firmware is beta
#define DEVHND_CARD_FIRMWARE_RC         0x02  // Firmware is release candidate
#define DEVHND_CARD_AUTO_RESP_OBJBUFS   0x04  // Firmware supports auto-response object buffers
#define DEVHND_CARD_REFUSE_TO_RUN       0x08  // Major problem detected
#define DEVHND_CARD_REFUSE_TO_USE_CAN   0x10  // Major problem detected
#define DEVHND_CARD_AUTO_TX_OBJBUFS     0x20  // Firmware supports periodic transmit object buffers

/*  Cards specific data */
typedef struct VCanCardData
{
    uint32_t                hw_type;
    uint32_t                card_flags;
    uint32_t                capabilities;   // qqq This should be per channel!
    unsigned int            nrChannels;
    uint32_t                serialNumber;
    unsigned char           ean[8];
    unsigned int            firmwareVersionMajor;
    unsigned int            firmwareVersionMinor;
    unsigned int            firmwareVersionBuild;

    uint32_t                timeHi;
    uint32_t                timeOrigin;
    uint32_t                usPerTick;

    /* Ports and addresses */
    unsigned char           cardPresent;
    VCanChanData          **chanData;
    void                   *hwCardData;

    OS_IF_SEMAPHORE         open;

    struct VCanCardData    *next;
} VCanCardData;


typedef struct VCanDriverData
{
    int                     noOfDevices;
    int                     majorDevNr;
    struct timeval          startTime;
    char                   *deviceName;
} VCanDriverData;

typedef struct
{
    int                     bufHead;
    int                     bufTail;
#if DEBUG
    int                     lastEmpty;
    int                     lastNotEmpty;
#endif
    VCAN_EVENT              fileRcvBuffer[FILE_RCV_BUF_SIZE];
    int                     size;
    OS_IF_WAITQUEUE_HEAD    rxWaitQ;
#if !LINUX
    OS_IF_LOCK              lock;
#endif
} VCanReceiveData;

/* File pointer specific data */
typedef struct VCanOpenFileNode {
    OS_IF_SEMAPHORE         ioctl_mutex;
    VCanReceiveData         rcv;
    unsigned char           transId;
#if LINUX
    struct file            *filp;
#else
    USBCAN_CONTEXT         *usbcan_context;
    OS_IF_EVENT             inactive;
#endif
    struct VCanChanData    *chanData;
    int                     chanNr;
    unsigned char           writeIsBlock;
    unsigned char           readIsBlock;
    unsigned char           modeTx;
    unsigned char           modeTxRq;
    unsigned char           channelOpen;
    unsigned char           channelLocked;
    long                    writeTimeout;
    long                    readTimeout;
    VCanMsgFilter           filter;
    OS_IF_TASK_QUEUE_HANDLE objbufWork;
    OS_IF_WQUEUE            *objbufTaskQ;
    OBJECT_BUFFER           *objbuf;
    uint32_t                objbufActive;
    uint32_t                overruns;
    struct VCanOpenFileNode *next;
} VCanOpenFileNode;


/* Dispatch call structure */
typedef struct VCanHWInterface {
    int (*initAllDevices)       (void);
    int (*setBusParams)         (VCanChanData *chd, VCanBusParams *par);
    int (*getBusParams)         (VCanChanData *chd, VCanBusParams *par);
    int (*setOutputMode)        (VCanChanData *chd, int silent);
    int (*setTranceiverMode)    (VCanChanData *chd, int linemode, int resnet);
    int (*busOn)                (VCanChanData *chd);
    int (*busOff)               (VCanChanData *chd);
    int (*txAvailable)          (VCanChanData *chd);
#if 0
    int (*transmitMessage)      (VCanChanData *chd, CAN_MSG *m);
#endif
    int (*procRead)             (char *buf, char **start, OS_IF_OFFSET offset,
                                 int count, int *eof, void *data);
    int (*closeAllDevices)      (void);
    unsigned long (*getTime)    (VCanCardData*);
    int (*flushSendBuffer)      (VCanChanData*);
    int (*getRxErr)             (VCanChanData*);
    int (*getTxErr)             (VCanChanData*);
    unsigned long (*rxQLen)     (VCanChanData*);
    unsigned long (*txQLen)     (VCanChanData*);
    int (*requestChipState)     (VCanChanData*);
    int (*requestSend)          (VCanCardData*, VCanChanData*);
    unsigned int (*getVersion)  (int);
    int (*objbufExists)         (VCanChanData *chd, int bufType, int bufNo);
    int (*objbufFree)           (VCanChanData *chd, int bufType, int bufNo);
    int (*objbufAlloc)          (VCanChanData *chd, int bufType, int *bufNo);
    int (*objbufWrite)          (VCanChanData *chd, int bufType, int bufNo,
                                 int id, int flags, int dlc, unsigned char *data);
    int (*objbufEnable)         (VCanChanData *chd, int bufType, int bufNo,
                                 int enable);
    int (*objbufSetFilter)      (VCanChanData *chd, int bufType, int bufNo,
                                 int code, int mask);
    int (*objbufSetFlags)       (VCanChanData *chd, int bufType, int bufNo,
                                 int flags);
    int (*objbufSetPeriod)      (VCanChanData *chd, int bufType, int bufNo,
                                 int period);
    int (*objbufSetMsgCount)    (VCanChanData *chd, int bufType, int bufNo,
                                 int count);
    int (*objbufSendBurst)      (VCanChanData *chd, int bufType, int bufNo,
                                 int burstLen);
} VCanHWInterface;


#define WAITNODE_DATA_SIZE 32 // 32 == MAX(MAX_CMD_LEN in filo_cmds.h and helios_cmds.h)
typedef struct WaitNode {
  struct list_head list;
  OS_IF_SEMAPHORE  waitSemaphore;
  void             *replyPtr;
  unsigned char    cmdNr;
  unsigned char    transId;
  unsigned char    timedOut;
#if !LINUX
  char             data[WAITNODE_DATA_SIZE]; 
#endif
} WaitNode;




/*****************************************************************************/
/*  Shared data structures                                                   */
/*****************************************************************************/

extern VCanDriverData         driverData;
extern VCanCardData          *canCards;
extern VCanHWInterface        hwIf;
extern OS_IF_LOCK             canCardsLock;
extern struct file_operations fops;

#if !LINUX
extern WaitNode waitNodes[];
extern WaitNode *freeWaitNode;
#endif



/*****************************************************************************/
/*  Function definitions                                                     */
/*****************************************************************************/

#if LINUX
#if 0
int  vCanOpen(struct inode *inode, struct file *filp);
int vCanClose(struct inode *inode, struct file *filp);
int vCanIOCtl(struct inode *inode, struct file *filp,
              unsigned int cmd, unsigned long arg);
unsigned int vCanPoll(struct file *filp, poll_table *wait);
#endif
#else
VCanOpenFileNode* vCanOpen(USBCAN_CONTEXT *usbcan_context);
int vCanClose(VCanOpenFileNode *context);
int vCanIOCtl(VCanOpenFileNode *fileNodePtr,
              DWORD ioctl_cmd, PBYTE pBufIn, DWORD dwLenIn,
              PBYTE pBufOut, DWORD dwLenOut, PDWORD pdwActualOut);
int init_module(void);
void cleanup_module(void);
#endif


/* Functions */
int             vCanInitData(VCanCardData *chd);
unsigned long   vCanTime(VCanCardData *vCard);
int             vCanDispatchEvent(VCanChanData *chd, VCAN_EVENT *e);
int             vCanFlushSendBuffer(VCanChanData *chd);
unsigned long   getQLen(unsigned long head, unsigned long tail, unsigned long size);


#endif /* _VCAN_OS_IF_H_ */
