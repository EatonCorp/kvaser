/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/* vcanevt.h: Defines for CAN driver event
 * (CAN messages, timer events, statistic, ...)
*/

#ifndef VCANEVT_H
#define VCANEVT_H

#if LINUX
#   if !defined(__KERNEL__)
#      include <stdint.h>
#   else
#      include <linux/types.h>
#   endif
#else
typedef unsigned long uint32_t;
#endif

/***************************************************************************/

#include <pshpack1.h>

/***************************************************************************/

enum e_vevent_type {
       V_NO_COMMAND            = 1,
       V_RECEIVE_MSG           = 2,
       V_STATISTIC_STD         = 4,
       V_STATISTIC_EXT         = 8,
       V_CHIP_STATE            = 16,
       V_TRANSCEIVER           = 32,
       V_TIMER                 = 64,
       V_TRANSMIT_MSG          = 128,
     };

typedef unsigned char VeventTag;


/* Structure for V_RECEIVE_MSG, V_TRANSMIT_MSG */

/* Message flags */
#define MAX_MSG_LEN 8
#define EXT_MSG                     0x80000000 // signs an extended identifier
#define VCAN_EXT_MSG_ID             EXT_MSG

#define VCAN_MSG_FLAG_ERROR_FRAME   0x01
#define VCAN_MSG_FLAG_OVERRUN       0x02  /* Overrun in Driver or CAN Controller */
                                          /* special case: OVERRUN combined with TIMER
                                           * means the 32 bit timer has overrun
                                           */
#define VCAN_MSG_FLAG_NERR          0x04  /* Line Error on Lowspeed */
#define VCAN_MSG_FLAG_WAKEUP        0x08  /* High Voltage Message on Single Wire CAN */
#define VCAN_MSG_FLAG_REMOTE_FRAME  0x10
#define VCAN_MSG_FLAG_RESERVED_1    0x20
#define VCAN_MSG_FLAG_TX_NOTIFY     0x40  /* Message Transmitted */
#define VCAN_MSG_FLAG_TXACK         0x40  /* Message Transmitted */
#define VCAN_MSG_FLAG_TX_START      0x80  /* Transmit Message stored into Controller  */
#define VCAN_MSG_FLAG_TXRQ          0x80  /* Transmit Message stored into Controller  */

struct s_vcan_msg {  /* 14 Bytes */
         uint32_t      id;
         unsigned char flags;
         unsigned char dlc;
         unsigned char data[MAX_MSG_LEN];
       };


/* Structure for V_CHIP_STATE */

#define CHIPSTAT_BUSOFF              0x01
#define CHIPSTAT_ERROR_PASSIVE       0x02
#define CHIPSTAT_ERROR_WARNING       0x04
#define CHIPSTAT_ERROR_ACTIVE        0x08

struct s_vcan_chip_state {
         unsigned char busStatus;
         unsigned char txErrorCounter;
         unsigned char rxErrorCounter;
       };


/* Structure for V_STATISTIC_STD */
struct s_vcan_statistic_std {
         uint32_t       stdData;
         uint32_t       stdRemote;
         uint32_t       errFrame;
         unsigned short busLoad; // 0.00-100.00%
       };


/* Structure for V_STATISTIC_EXT */
struct s_vcan_statistic_ext {
         uint32_t extData;
         uint32_t extRemote;
         uint32_t ovrFrame;
       };


/* Structure for V_ERROR */
struct s_vcan_error {
         unsigned char code;
       };


/* Structure for SET_OUTPUT_MODE */
#define OUTPUT_MODE_SILENT 0
#define OUTPUT_MODE_NORMAL 1


/* Transceiver modes */
#define TRANSCEIVER_EVENT_ERROR   1
#define TRANSCEIVER_EVENT_CHANGED 2



/* Transceiver modes */
#define VCAN_TRANSCEIVER_LINEMODE_NA            0 // Not Affected/Not available.
#define VCAN_TRANSCEIVER_LINEMODE_TWO_LINE      1 // W210 two-line.
#define VCAN_TRANSCEIVER_LINEMODE_CAN_H         2 // W210 single-line CAN_H
#define VCAN_TRANSCEIVER_LINEMODE_CAN_L         3 // W210 single-line CAN_L
#define VCAN_TRANSCEIVER_LINEMODE_SWC_SLEEP     4 // SWC Sleep Mode.
#define VCAN_TRANSCEIVER_LINEMODE_SWC_NORMAL    5 // SWC Normal Mode.
#define VCAN_TRANSCEIVER_LINEMODE_SWC_FAST      6 // SWC High-Speed Mode.
#define VCAN_TRANSCEIVER_LINEMODE_SWC_WAKEUP    7 // SWC Wakeup Mode.
#define VCAN_TRANSCEIVER_LINEMODE_SLEEP         8 // Sleep mode for those supporting it.
#define VCAN_TRANSCEIVER_LINEMODE_NORMAL        9 // Normal mode (the inverse of sleep mode) for those supporting it.
#define VCAN_TRANSCEIVER_LINEMODE_STDBY        10 // Standby for those who support it
#define VCAN_TRANSCEIVER_LINEMODE_TT_CAN_H     11 // Truck & Trailer: operating mode single wire using CAN high
#define VCAN_TRANSCEIVER_LINEMODE_TT_CAN_L     12 // Truck & Trailer: operating mode single wire using CAN low
#define VCAN_TRANSCEIVER_LINEMODE_OEM1         13 // Reserved for OEM apps
#define VCAN_TRANSCEIVER_LINEMODE_OEM2         14 // Reserved for OEM apps
#define VCAN_TRANSCEIVER_LINEMODE_OEM3         15 // Reserved for OEM apps
#define VCAN_TRANSCEIVER_LINEMODE_OEM4         16 // Reserved for OEM apps


#define VCAN_TRANSCEIVER_RESNET_NA              0
#define VCAN_TRANSCEIVER_RESNET_MASTER          1
#define VCAN_TRANSCEIVER_RESNET_MASTER_STBY     2
#define VCAN_TRANSCEIVER_RESNET_SLAVE           3


/* VCAN_EVENT structure */
union s_vcan_tag_data {
        struct s_vcan_msg                  msg;
        struct s_vcan_chip_state           chipState;
        struct s_vcan_statistic_std        statisticStd;
        struct s_vcan_statistic_ext        statisticExt;
        struct s_vcan_error                error;
      };


/* Event type definition */
struct s_vcan_event {
         VeventTag     tag;             // 1
         unsigned char chanIndex;       // 1
         unsigned char transId;         // 1
         unsigned char unused_1;        // 1 internal use only !!!!
         uint32_t      timeStamp;            // 4
         union s_vcan_tag_data
                       tagData;         // 14 Bytes (_VMessage)
       };
                                        // --------
                                        // 22 Bytes

typedef struct s_vcan_event VCAN_EVENT, Vevent, *PVevent;


typedef struct s_can_msg {
          VeventTag     tag;
          unsigned char channel_index;
          unsigned char user_data;
          unsigned char unused_1;
          uint32_t      timestamp;
          uint32_t      id;
          unsigned char flags;
          unsigned char length;
          unsigned char data [8];
        } CAN_MSG;


/*****************************************************************************/

#include <poppack.h>

/*****************************************************************************/

#endif /* VCANEVT_H */

