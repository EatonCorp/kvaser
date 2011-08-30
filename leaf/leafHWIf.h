/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

//
// Linux/WinCE Leaf driver
//

#ifndef _LEAF_HW_IF_H_
#define _LEAF_HW_IF_H_


#if WIN32
#include "vcanevt.h"
#include "VCanOSif.h"
#endif

#include "osif_kernel.h"
#include "filo_cmds.h"
#include "objbuf.h"

/*****************************************************************************/
/* defines */
/*****************************************************************************/

#if LINUX
#define DEVICE_NAME_STRING                    "leaf"
#endif
#define MAX_CHANNELS                               3
#define KV_LEAF_MAIN_RCV_BUF_SIZE                 16
#define KV_LEAF_TX_CMD_BUF_SIZE                   16
#define LEAF_CMD_RESP_WAIT_TIME                 5000
#define DEMETER_MAX_OUTSTANDING_TX               128


// Bits in the CxSTRH register in the M16C.
#define M16C_BUS_RESET    0x01    // Chip is in Reset state
#define M16C_BUS_ERROR    0x10    // Chip has seen a bus error
#define M16C_BUS_PASSIVE  0x20    // Chip is error passive
#define M16C_BUS_OFF      0x40    // Chip is bus off



#if DEBUG
#   define LEAF_Q_CMD_WAIT_TIME                 800
#else
#   define LEAF_Q_CMD_WAIT_TIME                 200
#endif


/* Channel specific data */
typedef struct LeafChanData
{
#if 0
  unsigned int transId;
#endif
  /* These are the number of outgoing packets residing in the device */
  unsigned int outstanding_tx;
  OS_IF_LOCK   outTxLock;

#if 0
  // Card specific data (from Windows version)
  unsigned int           max_outstanding_tx;

  unsigned int           tx_highwater;
#endif

  unsigned int           timestamp_correction_value;

  OBJECT_BUFFER          *objbufs;

  // qqq Max_outstanding_tx is received from the card
  CAN_MSG          current_tx_message[DEMETER_MAX_OUTSTANDING_TX];

  } LeafChanData;



/*  Cards specific data */
typedef struct LeafCardData {

  unsigned int     max_outstanding_tx;
  int              autoTxBufferCount;
  int              autoTxBufferResolution;

  OS_IF_LOCK       replyWaitListLock;
  struct list_head replyWaitList;

  /* Structure to hold all of our device specific stuff */

  OS_IF_WQUEUE                *txTaskQ;
  OS_IF_TASK_QUEUE_HANDLE     txWork;

  filoCmd            txCmdBuffer[KV_LEAF_TX_CMD_BUF_SIZE]; /* Control messages */
#if 0
  unsigned int       txCmdBufHead;   /* Where we write outgoing control messages */
  unsigned int       txCmdBufTail;   /* The messages are sent from this end */

  OS_IF_WAITQUEUE_HEAD  txCmdWaitQ;    /* WaitQ for sending commands */
#else
  Queue              txCmdQueue;
#endif

  // busparams
  unsigned long freq;
  unsigned char sjw;
  unsigned char tseg1;
  unsigned char tseg2;
  unsigned char samples;

#if LINUX
  struct usb_device       *udev;               // save off the usb device pointer
  struct usb_interface    *interface;          // the interface for this device
#else
  LPCUSB_FUNCS            usb_funcs;
#endif

  unsigned char *         bulk_in_buffer;      // the buffer to receive data
  size_t                  bulk_in_size;        // the size of the receive buffer
  __u8                    bulk_in_endpointAddr;// the address of the bulk in endpoint
// qqq, why is bulk_in_MaxPacketSize unsigned in Linux and signed otherwise?
#if LINUX
  unsigned int            bulk_in_MaxPacketSize;
#else
  int                     bulk_in_MaxPacketSize;
#endif

#if LINUX
  unsigned char *         bulk_out_buffer;     // the buffer to send data
#endif
  size_t                  bulk_out_size;       // the size of the send buffer

// qqq, why is bulk_out_MaxPacketSize unsigned in Linux and signed otherwise?
#if LINUX
  unsigned int            bulk_out_MaxPacketSize;
#else
  int                     bulk_out_MaxPacketSize;
#endif

  struct urb *            write_urb;           // the urb used to send data
  struct urb *            read_urb;            // the urb used to receive data
  __u8                    bulk_out_endpointAddr;//the address of the bulk out endpoint
  OS_IF_SEMAPHORE         write_finished;       // wait for the write to finish

  volatile int            present;              // if the device is not disconnected
  OS_IF_SEMAPHORE         sem;                  // locks this structure

  VCanCardData           *vCard;

  // General data (from Windows version)
  // Time stamping timer frequency in MHz
  unsigned int  hires_timer_fq;
  unsigned int  time_offset_valid;

} LeafCardData;


#if !LINUX
int leaf_plugin(USBCAN_CONTEXT *usbcan_context);
void leaf_remove(USBCAN_CONTEXT *usbcan_context);
#endif

#endif  /* _LEAF_HW_IF_H_ */
