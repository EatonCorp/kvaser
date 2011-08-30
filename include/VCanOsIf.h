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
#else // WIN32
#   include "windows.h"

typedef struct _USBCAN_CONTEXT USBCAN_CONTEXT;
#endif

#include "canIfData.h"
#include "vcanevt.h"

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
#define ArgIntIn
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
#if 0
#define ArgIntIn     do { if (pBufIn && (dwLenIn == 4)) {   \
                            get_user_long_ret(&arg, (long *)pBufIn, -EFAULT); \
                          } else {                          \
                            return -EFAULT;                 \
                          }                                 \
                        } while(0)
#else
#define ArgIntIn     do { arg = (int)pBufIn; } while(0)
#endif
#endif


#define VCAN_STAT_OK                 0
#define VCAN_STAT_FAIL              -1
#define VCAN_STAT_TIMEOUT           -2


/*****************************************************************************/
/*  Data structures                                                          */
/*****************************************************************************/

typedef union {
    unsigned long L;
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
    unsigned char            ean[6];      // QQQ GB: I believe this should be 8 chars or two 32-bit int
    unsigned long            serialHigh;
    unsigned long            serialLow;

    /* Status */
    unsigned char            isOnBus;
    unsigned char            transType;   // TRANSCEIVER_TYPE_xxx
    unsigned char            lineMode;    // TRANSCEIVER_LINEMODE_xxx
    unsigned char            resNet;      // TRANSCEIVER_RESNET_xxx
    atomic_t                 transId;
#if WIN32
    unsigned char            chanId;
#endif
    unsigned int             overrun;
    CanChipState             chipState;
    unsigned int             errorCount;
    unsigned long            errorTime;
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
    atomic_t                waitEmpty;

    struct VCanCardData    *vCard;
} VCanChanData;


/*  Cards specific data */
typedef struct VCanCardData
{
    unsigned int            nrChannels;
    unsigned long           serialNumber;
    unsigned char           ean[6];            // QQQ GB: I believe this should be 8 chars or two 32-bit int
    unsigned int            firmwareVersionMajor;
    unsigned int            firmwareVersionMinor;
    unsigned int            firmwareVersionBuild;

    unsigned long           timeHi;
    unsigned long           timeOrigin;
    unsigned long           usPerTick;

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

/* File pointer specific data */
typedef struct VCanOpenFileNode {
    int                     rcvBufHead;
    int                     rcvBufTail;
    VCAN_EVENT              fileRcvBuffer[FILE_RCV_BUF_SIZE];
#if LINUX
    unsigned char           transId;
#else
    unsigned char           chanId;
#endif
    OS_IF_WAITQUEUE_HEAD    rxWaitQ;
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
    unsigned long           overruns;
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
    int (*transmitMessage)      (VCanChanData *chd, CAN_MSG *m);
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
} VCanHWInterface;



/*****************************************************************************/
/*  Shared data structures                                                   */
/*****************************************************************************/

extern VCanDriverData         driverData;
extern VCanCardData          *canCards;
extern VCanHWInterface        hwIf;
extern OS_IF_LOCK             canCardsLock;
extern struct file_operations fops;



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
