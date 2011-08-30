/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*
** This file defines the interface to LAPcan and ISAcan.
**
** Rules:
**  1) Don't remove anything, just add.
**  2) Don't use enums, use explicit types whose size is known (for ALL compilers)
**  3) Mind the alignment, please.
*/
#ifndef _LAPCMDS_H_
#define _LAPCMDS_H_

//----------------------------------------------------------------------
// Version numbers  IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT
// ---------------------------------------------------------------------
// Increase the major rev number when *incompatible* changes are made.
// Increase the minor number otherwise.
#define LAPCMDS_MAJOR_REVISION 1
#define LAPCMDS_MINOR_REVISION 1
//
// Please increment this if ANY change is made to the layout of the frequent
// messages. This is because their layout may be defined in parallel in
// some other header files, for performance reasons.
//
#define SHORT_MESSAGE_LAYOUT_VERSION 2
//
//----------------------------------------------------------------------


#define CMD_RX_STD_MESSAGE          0
#define CMD_RX_EXT_MESSAGE          1
#define CMD_TX_STD_MESSAGE          0
#define CMD_TX_EXT_MESSAGE          1


/* 32c: !------------------------------!  */
#define CMD__LOW                                 10
#define CMD_AUTOBAUD_REQ                         10
#define CMD_AUTOBAUD_RESP                        11
#define CMD_CANCEL_SCHDL_MSGS_REQ                12
#define CMD_CANCEL_SCHDL_MSGS_RESP               13
#define CMD____UNUSED7                           14
#define CMD____UNUSED8                           15
#define CMD_CHIP_STATE_EVENT                     16
#define CMD_GET_SW_FILTER_REQ                    17
#define CMD_GET_SW_FILTER_RESP                   18
#define CMD_ERROR_EVENT                          19
#define CMD_GET_BUSPARAMS_REQ                    20
#define CMD_GET_BUSPARAMS_RESP                   21
#define CMD_GET_CARD_INFO_REQ                    22
#define CMD_GET_CARD_INFO_RESP                   23
#define CMD_SET_SW_FILTER_REQ                    24
#define CMD____UNUSED2                           25
#define CMD_GET_CHIP_STATE_REQ                   26
#define CMD_GET_CHIP_STATE_RESP                  27
#define CMD_GET_DRIVERMODE_REQ                   28
#define CMD_GET_DRIVERMODE_RESP                  29
#define CMD_GET_HW_FILTER_REQ                    30
#define CMD_GET_HW_FILTER_RESP                   31
#define CMD_GET_INTERFACE_INFO_REQ               32
#define CMD_GET_INTERFACE_INFO_RESP              33
#define CMD_GET_IO_PIN_STATE_REQ                 34
#define CMD_GET_IO_PIN_STATE_RESP                35
#define CMD_GET_IO_PIN_TRIGGER_REQ               36
#define CMD_GET_IO_PIN_TRIGGER_RESP              37
#define CMD_GET_GLOBAL_OPTIONS_REQ               38
#define CMD_GET_GLOBAL_OPTIONS_RESP              39
#define CMD_GET_SOFTWARE_INFO_REQ                40
#define CMD_GET_SOFTWARE_INFO_RESP               41
#define CMD_GET_STATISTICS_REQ                   42
#define CMD_GET_STATISTICS_RESP                  43
#define CMD_GET_TIMER_REQ                        44
#define CMD_GET_TIMER_RESP                       45
#define CMD_NO_COMMAND                           46
#define CMD_RESET_CHIP_REQ                       47
#define CMD_RESET_CARD_REQ                       48
#define CMD_RESET_STATISTICS_REQ                 49
#define CMD_SET_BUSPARAMS_REQ                    50
#define CMD____UNUSED5                           51
#define CMD_SET_DRIVERMODE_REQ                   52
#define CMD_SET_HW_FILTER_REQ                    53
#define CMD_SET_IO_PIN_STATE_REQ                 54
#define CMD_SET_IO_PIN_TRIGGER_REQ               55
#define CMD_SET_GLOBAL_OPTIONS_REQ               56
#define CMD_SET_TIMER_REQ                        57
#define CMD_START_CHIP_REQ                       58
#define CMD_START_CHIP_RESP                      59
#define CMD_STOP_CHIP_REQ                        60
#define CMD_STOP_CHIP_RESP                       61
#define CMD_TIMER_EVENT                          62
#define CMD_CLOCK_OVERFLOW_EVENT                 63
#define CMD_TRIGGER_EVENT                        64
#define CMD_START_APPLICATION_REQ                67
#define CMD_READ_CLOCK_REQ                       68
#define CMD_READ_CLOCK_RESP                      69
#define CMD_RESET_CLOCK                          75
#define CMD_SET_HEARTBEAT_REQ                    76
#define CMD_SET_TRANSCEIVER_MODE_REQ             77
#define CMD_SET_TRANSCEIVER_MODE_RESP            78
#define CMD_TRANSCEIVER_EVENT                    79
#define CMD_GET_TRANSCEIVER_INFO_REQ             80
#define CMD_GET_TRANSCEIVER_INFO_RESP            81
#define CMD_READ_PARAMETER_REQ                   84
#define CMD_READ_PARAMETER_RESP                  85 // Yes, 85.
#define CMD_DETECT_TRANSCEIVERS                  85 // Yes, 85.
#define CMD__UNUSED6                             86
#define CMD_READ_CLOCK_NOW_REQ                   87
#define CMD_READ_CLOCK_NOW_RESP                  88
#define CMD_FILTER_MESSAGE                       89
#define CMD_FLUSH_QUEUE                          90
#define CMD_GET_CHIP_STATE2_REQ                  91
#define CMD_GET_CHIP_STATE2_RESP                 92

#define CMD__HIGH                                92
// NOTE: CMD__HIGH must not exceed 127!! (CMD__LOW + 117)

/*
** This transaction ID is used by the card for unsolicited
** messages (like error messages etc).
*/
#define SPECIAL_XID                 0xff

/*
** CAN message flags
*/

#define MSGFLAG_ERROR_FRAME         0x01        // Msg is a bus error
#define MSGFLAG_OVERRUN             0x02        // Msgs following this has been lost
#define MSGFLAG_NERR                0x04        // NERR active during this msg
#define MSGFLAG_WAKEUP              0x08        // Msg rcv'd in wakeup mode
#define MSGFLAG_REMOTE_FRAME        0x10        // Msg is a remote frame
#define MSGFLAG_RESERVED_1          0x20        // Reserved for future usage
#define MSGFLAG_TX                  0x40        // TX acknowledge
#define MSGFLAG_TXRQ                0x80        // TX request


/*
** Driver modes
*/
#define DRIVERMODE_NORMAL           0x01
#define DRIVERMODE_SILENT           0x02
#define DRIVERMODE_SELFRECEPTION    0x03
#define DRIVERMODE_OFF              0x04

/*
** Chip status flags
*/
#define BUSSTAT_BUSOFF              0x01
#define BUSSTAT_ERROR_PASSIVE       0x02
#define BUSSTAT_ERROR_WARNING       0x04
#define BUSSTAT_ERROR_ACTIVE        0x08
#define BUSSTAT_BUSOFF_RECOVERY     0x10
#define BUSSTAT_IGNORING_ERRORS     0x20

/*
** Applications.
*/
#define APP_EEPROM                  1
#define APP_SHUFFLE                 2
#define APP_ADDRESS                 3
#define APP_KVASER                  4
#define APP_VECTOR                  5
#define APP_DIAGNOSE                6
#define APP_BOOTSTRAP               7

/*
** Error codes.
** Used in CMD_ERROR_EVENT.
*/
#define LAPERR_OK                   0  // No error.
#define LAPERR_CAN                  1  // CAN error, addInfo1 contains error code.
#define LAPERR_NVRAM_ERROR          2  // Flash error
#define LAPERR_NOPRIV               3  // No privilege for attempted operation
#define LAPERR_ILLEGAL_ADDRESS      4  // Illegal RAM/ROM address specified
#define LAPERR_UNKNOWN_CMD          5  // Unknown command or subcommand
#define LAPERR_FATAL                6  // A severe error. addInfo1 contains error code.
#define LAPERR_CHECKSUM_ERROR       7  // Downloaded code checksum mismatch
#define LAPERR_QUEUE_LEVEL          8  // Not really an error: Tx queue levels.

/*
** Subcodes for fatal and internal errors.
** Used in conjunction with LAPERR_FATAL.
*/
#define LAPERR_FATAL_NOEVENTS       1   // Event queue exhausted.
#define LAPERR_FATAL_FIFOSYNC       2   // FIFO out of synch
#define LAPERR_FATAL_HW_TOO_OLD     3   // Hardware is too old for this firmware's taste.

/*
** Subcodes for transmit queue level reporting.
** Used in conjunction with LAPERR_QUEUE_LEVEL.
*/
#define TRANSMIT_QUEUE_LOW          1
#define TRANSMIT_QUEUE_HIGH         2

/*
** Card global options for SET_GLOBAL_OPTIONS_REQ et al.
*/
#define GLOPT_USE_DCD_PROTOCOL       0x01  // DCD protocol shall be used
#define GLOPT__RESERVED2             0x02  // Was: enable DSR toggling

/*
** Channel options. Currently unused.
*/
#define CHOPT_ENABLE_TX_ACK          0x01  // Enable TX receipts
#define CHOPT_ENABLE_TXRQ            0x02  // Enable TXRQ receipts


/*
** Flags for setting hardware message filters.
*/
#define FILTER_EXTENDED_ID          0x01
#define FILTER_MASK_IS_VALID        0x02
#define FILTER_CODE_IS_VALID        0x04
#define FILTER_REMOVE_ALL           0x08
#define FILTER_SJA1000_SINGLE       0x40   // Don't like it, but..
#define FILTER_SJA1000_DOUBLE       0x80   // Still don't like it.

/*
** Flags for setting software message filters.
*/
#define FILTER_SW_EXTENDED_ID       0x01
#define FILTER_SW_MASK_IS_VALID     0x02
#define FILTER_SW_CODE_IS_VALID     0x04
#define FILTER_SW_REMOVE_ALL        0x08


/*
** CAN chip types.
*/
#define CANCHIP_NO_CHIP             0x0000
#define CANCHIP_82C200              0x0001
#define CANCHIP_SJA1000             0x0002

/*
** CAN chip subtypes.
*/
#define CANCHIP_SUBTYPE_UNDEFINED   0x0000
#define CANCHIP_SUBTYPE_SJA1000_N1A 0x0100
#define CANCHIP_SUBTYPE_SJA1000_N1B 0x0200
#define CANCHIP_SUBTYPE_SJA1000_N1C 0x0300

/*
** CAN channel capabilities.
*/
#define CHANNEL_CAP_EXTENDED_CAN         0x00000001L
#define CHANNEL_CAP_BUS_STATISTICS       0x00000002L
#define CHANNEL_CAP_ERROR_COUNTERS       0x00000004L
#define CHANNEL_CAP_CAN_DIAGNOSTICS      0x00000008L
#define CHANNEL_CAP_GENERATE_ERROR       0x00000010L
#define CHANNEL_CAP_GENERATE_OVERLOAD    0x00000020L
#define CHANNEL_CAP_TXREQUEST            0x00000040L
#define CHANNEL_CAP_TXACKNOWLEDGE        0x00000080L

/*
** Transceiver status flags.
*/
#define TRANSCEIVER_STATUS_PRESENT       0x01
#define TRANSCEIVER_STATUS_POWER         0x02
#define TRANSCEIVER_STATUS_MEMBLANK      0x04
#define TRANSCEIVER_STATUS_MEMCORRUPT    0x08
#define TRANSCEIVER_STATUS_POWER_GOOD    0x10
#define TRANSCEIVER_STATUS_EXTPWR_GOOD   0x20

/*
** Transceiver capabilities.
*/
#define TRANSCEIVER_CAP_HIGHSPEED        0x00000001L

/*
** lapcan timestamp clock
*/
#define LAPCAN_US_PER_TICK               8

/*
** Other transceiver flags.
*/
#define TRANSCEIVER_LINEMODE_NA          0  // Not Affected/Not available.
#define TRANSCEIVER_LINEMODE_TWO_LINE    1  // W210 two-line.
#define TRANSCEIVER_LINEMODE_CAN_H       2  // W210 single-line CAN_H
#define TRANSCEIVER_LINEMODE_CAN_L       3  // W210 single-line CAN_L
#define TRANSCEIVER_LINEMODE_SWC_SLEEP   4  // SWC Sleep Mode.
#define TRANSCEIVER_LINEMODE_SWC_NORMAL  5  // SWC Normal Mode.
#define TRANSCEIVER_LINEMODE_SWC_FAST    6  // SWC High-Speed Mode.
#define TRANSCEIVER_LINEMODE_SWC_WAKEUP  7  // SWC Wakeup Mode.
#define TRANSCEIVER_LINEMODE_SLEEP       8  // Sleep mode for those supporting it.
#define TRANSCEIVER_LINEMODE_NORMAL      9  // Normal mode (the inverse of sleep mode) for those supporting it.
#define TRANSCEIVER_LINEMODE_STDBY      10  // Standby for those who support it
#define TRANSCEIVER_LINEMODE_TT_CAN_H   11  // Truck & Trailer: operating mode single wire using CAN high
#define TRANSCEIVER_LINEMODE_TT_CAN_L   12  // Truck & Trailer: operating mode single wire using CAN low
#define TRANSCEIVER_LINEMODE_OEM1       13  // Reserved for OEM apps
#define TRANSCEIVER_LINEMODE_OEM2       14  // Reserved for OEM apps
#define TRANSCEIVER_LINEMODE_OEM3       15  // Reserved for OEM apps
#define TRANSCEIVER_LINEMODE_OEM4       16  // Reserved for OEM apps

#define TRANSCEIVER_RESNET_NA            0
#define TRANSCEIVER_RESNET_MASTER        1
#define TRANSCEIVER_RESNET_MASTER_STBY   2
#define TRANSCEIVER_RESNET_SLAVE         3

#define TRANSCEIVER_RESNET_EVA_00        0
#define TRANSCEIVER_RESNET_EVA_01        1
#define TRANSCEIVER_RESNET_EVA_10        2
#define TRANSCEIVER_RESNET_EVA_11        3


/*
** Transceiver events.
*/
#define TRANSCEIVER_EVENT_NONE           0
#define TRANSCEIVER_EVENT_NERR           1
#define TRANSCEIVER_EVENT_REMOVED        2
#define TRANSCEIVER_EVENT_DETECTED       3

/*
** Software options/flags for cmdGetSoftwareInfo
*/
#define SWOPT_CAN_TOGGLE_DCD            0x01    // S/W can handle the DCD protocol

/*
** Flags for the cmdGetCardInfo.hwInfo field.
*/
#define CARDINFO_RESERVED_1             0x10    // Future usage
#define CARDINFO_RESERVED_2             0x20    // Future usage
#define CARDINFO_BIST_OK                0x40    // Card passed Built-In Self Test
#define CARDINFO_HW_UPTODATE            0x80    // Firmware accepts hw revision
#define CARDINFO_FLAG_MASK              0xF0
#define CARDINFO_REVISION_MASK          0x0F    // Lower 4 bits are the hw rev.

/*
** For CMD_FLUSH_QUEUE
*/
#define FLUSH_TX_QUEUE                  1
// #define FLUSH_RX_QUEUE               2

/*
** For CMD_FILTER_MESSAGE
*/
#define FILTER_CLEAR_ALL                1
#define FILTER_ACCEPT_MESSAGE           2
#define FILTER_REJECT_MESSAGE           3
#define FILTER_USE_BITMAP               4

/*
** ====================================================================
** Declarations of structures begin here. First, let's ensure they are
** packed on byte boundaries.
*/
#include "pshpack1.h"

/*
** Structures for arbitrary messages.
*/
typedef struct {
  unsigned char cmdAndLen; // len | (cmd << 5)
  unsigned char b[1];
} cmdShort;

typedef struct {
    unsigned char cmdLen; // (length not including cmdLen) | 0x80
    unsigned char cmdNo;
    unsigned char b[1];
} cmdLong;

// If the union contains an "expanded message" which always has the length and
// the number separated, they can be accessed via head (bit 7 should not
// be set in cmdLen).
typedef struct {
    unsigned char cmdLen;  // The length, not counting cmdLen.
    unsigned char cmdNo;
} cmdHead;


/*
** The short (frequent) messages.
*/


#define CMDLEN_TX_STD_MESSAGE_CMDANDLEN(dlc) (unsigned char)((CMD_TX_STD_MESSAGE << 5) | ((dlc)+5))
#define CMDLEN_TX_EXT_MESSAGE_CMDANDLEN(dlc) (unsigned char)((CMD_TX_EXT_MESSAGE << 5) | ((dlc)+7))
#define CMDLEN_TX_STD_MESSAGE(dlc) (unsigned char)((dlc)+6) // Number of bytes to transfer to the FIFO
#define CMDLEN_TX_EXT_MESSAGE(dlc) (unsigned char)((dlc)+8)

#define CMDLEN_RX_STD_MESSAGE_CMDANDLEN(dlc) (unsigned char)((CMD_RX_STD_MESSAGE << 5) | (dlc)+7)
#define CMDLEN_RX_EXT_MESSAGE_CMDANDLEN(dlc) (unsigned char)((CMD_RX_EXT_MESSAGE << 5) | (dlc)+9)
#define CMDLEN_RX_STD_MESSAGE(dlc) (unsigned char)((dlc)+8) // Number of bytes to transfer to the FIFO
#define CMDLEN_RX_EXT_MESSAGE(dlc) (unsigned char)((dlc)+10)


typedef struct {
    unsigned char cmdAndLen;
    unsigned char channel_and_dlc;  // Bit 0..3 is dlc, bit 4..7 is the channel.
    unsigned char flags;
    unsigned char transId;
    unsigned short id;
    unsigned char data[8];
} cmdTxStdMessage;

typedef struct {
    unsigned char cmdAndLen;
    unsigned char channel_and_dlc;  // Bit 0..3 is dlc, bit 4..7 is the channel.
    unsigned char flags;
    unsigned char transId;
    unsigned long id;
    unsigned char data[8];
} cmdTxExtMessage;

typedef struct {
    unsigned char cmdAndLen;
    unsigned char channel_and_dlc;  // Bit 0..3 is dlc, bit 4..7 is the channel.
    unsigned char flags;
    unsigned char transId;
    unsigned short time;
    unsigned short id;
    unsigned char data[8];
} cmdRxStdMessage;

typedef struct {
    unsigned char cmdAndLen;
    unsigned char channel_and_dlc;  // Bit 0..3 is dlc, bit 4..7 is the channel.
    unsigned char flags;
    unsigned char transId;
    unsigned short time;
    unsigned long id;
    unsigned char data[8];
} cmdRxExtMessage;



// A long form of the short messages. Not as received via the FIFO,
// but can be used if the FIFO-extractor expands the short message.
// The layout is the same after head. These messages are one byte longer
// than the corresponding FIFO-messages.
//
// qqq These are not properly aligned..!
//
typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char channel_and_dlc; // Bit 0..3 is dlc, bit 4..7 is the channel.
  unsigned char flags;
  unsigned char transId;
  unsigned short time;
  unsigned short id;
  unsigned char data[8];
} cmdRxStdMessageX;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char channel_and_dlc;
  unsigned char flags;
  unsigned char transId;
  unsigned short time;
  unsigned long id;
  unsigned char data[8];
} cmdRxExtMessageX;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char channel_and_dlc;
  unsigned char flags;
  unsigned char transId;
  unsigned short id;
  unsigned char data[8];
} cmdTxStdMessageX;

typedef struct {
  unsigned char cmdLen;
  unsigned char cmdNo;
  unsigned char channel_and_dlc;
  unsigned char flags;
  unsigned char transId;
  unsigned long id;
  unsigned char data[8];
} cmdTxExtMessageX;


/*
** The long (not-so-frequent) messages.
*/
#define CMDLEN(msg)  (0x80 | (sizeof(msg)-1))  // Length to store at xx.cmdLen
#define CMDLEN_MESSAGE(msg)  (sizeof(msg))  // Number of bytes to transfer to the FIFO

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdAutobaudReq;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdAutobaudResp;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdCancelSchdlMsgsReq;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdCancelSchdlMsgsResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char channel;
    unsigned char busStatus;
    unsigned char txErrorCounter;
    unsigned char rxErrorCounter;
    unsigned short time;
} cmdChipStateEvent;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char errorCode;
    unsigned short addInfo1;
    unsigned short addInfo2;
    unsigned short time;
} cmdErrorEvent;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetBusparamsReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long bitRate;
    unsigned char tseg1;
    unsigned char tseg2;
    unsigned char sjw;
    unsigned char noSamp;
} cmdGetBusparamsResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
      signed char dataLevel;
} cmdGetCardInfoReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channelCount;
    unsigned long serialNumberLow;
    unsigned long serialNumberHigh;
    unsigned long clockResolution;
    unsigned char EAN[6];       // LSB..MSB, then the check digit.
      signed char NVRAMStatus;
    unsigned char dataLevel;
    unsigned char hwRevision;
    unsigned char hwInfo;
} cmdGetCardInfoResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetChipStateReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char busStatus;
    unsigned char txErrorCounter;
    unsigned char rxErrorCounter;
} cmdGetChipStateResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetChipState2Req;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned short time;
    unsigned char busStatus;
    unsigned char _pad1;
    unsigned char txErrorCounter;
    unsigned char rxErrorCounter;
} cmdGetChipState2Resp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetDrivermodeReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char driverMode;
} cmdGetDrivermodeResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetHwFilterReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long acceptanceMask;
    unsigned long acceptanceCode;
    unsigned char flags;
} cmdGetHwFilterResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char flags;
} cmdGetSwFilterReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long acceptanceMask;
    unsigned long acceptanceCode;
    unsigned char flags;
} cmdGetSwFilterResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetInterfaceInfoReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long channelCapabilities;
    unsigned char canChipType;
    unsigned char canChipSubType;
} cmdGetInterfaceInfoResp;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdGetIoPinStateReq;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdGetIoPinStateResp;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdGetIoPinTriggerReq;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdGetIoPinTriggerResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdGetGlobalOptionsReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned long currentOptions;
    unsigned long availableOptions;
} cmdGetGlobalOptionsResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdGetSoftwareInfoReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char applicationName[13];
    unsigned char applicationVersion[6];
    unsigned short checkSum;
    unsigned short swOptions;
} cmdGetSoftwareInfoResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdGetStatisticsReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long stdData;
    unsigned long extData;
    unsigned long stdRemote;
    unsigned long extRemote;
    unsigned long errorFrames;
    unsigned long overloadFrames;
    unsigned long numberOfBits;
    unsigned long numberOfTicks;
    unsigned long bitRate;
    unsigned short time;
} cmdGetStatisticsResp;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdGetTimerReq;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdGetTimerResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdNoCommand;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdReadClockReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char _pad0;
    unsigned long currentTime;
} cmdReadClockResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdResetCardReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdResetChipReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdResetStatisticsReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdResetClock;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long bitRate;
    unsigned char tseg1;
    unsigned char tseg2;
    unsigned char sjw;
    unsigned char noSamp;
} cmdSetBusparamsReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char driverMode;
} cmdSetDrivermodeReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char enable;
    unsigned short interval;
} cmdSetHeartbeatReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long acceptanceCode;
    unsigned long acceptanceMask;
    unsigned char flags;
} cmdSetHwFilterReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long acceptanceCode;
    unsigned long acceptanceMask;
    unsigned char flags;
} cmdSetSwFilterReq;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char line;
    unsigned char flags;
} cmdSetIoPinStateReq;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdSetIoPinTriggerReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned long optionsMask;
    unsigned long optionsCode;
} cmdSetGlobalOptionsReq;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdSetTimerReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char application;
    unsigned long startAddress;
    unsigned short p1;
    unsigned short p2;
} cmdStartApplicationReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdStartChipReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdStartChipResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdStopChipReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
} cmdStopChipResp;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdTimerEvent;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned long currentTime;
} cmdClockOverflowEvent;

// Not implemented for now
typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char qqq;
    unsigned short time;
} cmdTriggerEvent;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char password[8];
} cmdValidatePasswordReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char ok;
} cmdValidatePasswordResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char lineMode;
    unsigned char resistorNet;
} cmdSetTransceiverModeReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char lineMode;
    unsigned char resistorNet;
} cmdSetTransceiverModeResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char channel;
    unsigned char eventCode;
    unsigned short time;
} cmdTransceiverEvent;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
      signed char dataLevel;
} cmdGetTransceiverInfoReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned long serialNumberLow;
    unsigned long serialNumberHigh;
    unsigned long transceiverCapabilities;
    unsigned char EAN[5];
    unsigned char reservedWOBytes[3];
    unsigned char transceiverStatus;
    unsigned char dataLevel;
    unsigned char transceiverType;
    // unsigned char __align;
} cmdGetTransceiverInfoResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
      signed char dataLevel;
    unsigned short paramNo;
    unsigned short offset;
} cmdReadParameterReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char _pad0;
    unsigned char data[16];
} cmdReadParameterResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
      signed char channel;
} cmdDetectTransceivers;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
} cmdReadClockNowReq;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char _pad0;
    unsigned long currentTime;
} cmdReadClockNowResp;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char action;
    unsigned char _pad1;
    unsigned short idL;
    unsigned short idH; // Not currently used; set to idL.
} cmdFilterMessage;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char action;
    unsigned char _pad1;
    unsigned short start;
    unsigned short len;
    unsigned char bitmap[16];
} cmdFilterMessage2;

typedef struct {
    unsigned char cmdLen;
    unsigned char cmdNo;
    unsigned char transId;
    unsigned char channel;
    unsigned char flags;
} cmdFlushQueue;


/*
** A union for all messages.
*/
typedef union {
  unsigned char                b[1]; // Access to the raw data bytes.
  cmdHead                      head;
  cmdShort                     s;
  cmdLong                      l;

  cmdTxStdMessage              txStdMessage;
  cmdTxExtMessage              txExtMessage;
  cmdRxStdMessage              rxStdMessage;
  cmdRxExtMessage              rxExtMessage;

#ifndef __C166__
  cmdTxStdMessageX             txStdMessageX;
  cmdTxExtMessageX             txExtMessageX;
  cmdRxStdMessageX             rxStdMessageX;
  cmdRxExtMessageX             rxExtMessageX;
#endif

  cmdAutobaudReq               autobaudReq;
  cmdAutobaudResp              autobaudResp;
  cmdCancelSchdlMsgsReq        cancelSchdlMsgsReq;
  cmdCancelSchdlMsgsResp       cancelSchdlMsgsResp;
  cmdChipStateEvent            chipStateEvent;
  cmdClockOverflowEvent        clockOverflowEvent;
  cmdErrorEvent                errorEvent;
  cmdGetBusparamsReq           getBusparamsReq;
  cmdGetBusparamsResp          getBusparamsResp;
  cmdGetCardInfoReq            getCardInfoReq;
  cmdGetCardInfoResp           getCardInfoResp;
  cmdGetChipStateReq           getChipStateReq;
  cmdGetChipStateResp          getChipStateResp;
  cmdGetChipState2Req          getChipState2Req;
  cmdGetChipState2Resp         getChipState2Resp;
  cmdGetDrivermodeReq          getDrivermodeReq;
  cmdGetDrivermodeResp         getDrivermodeResp;
  cmdGetHwFilterReq            getHwFilterReq;
  cmdGetHwFilterResp           getHwFilterResp;
  cmdGetSwFilterReq            getSwFilterReq;
  cmdGetSwFilterResp           getSwFilterResp;
  cmdGetInterfaceInfoReq       getInterfaceInfoReq;
  cmdGetInterfaceInfoResp      getInterfaceInfoResp;
  cmdGetIoPinStateReq          getIoPinStateReq;
  cmdGetIoPinStateResp         getIoPinStateResp;
  cmdGetIoPinTriggerReq        getIoPinTriggerReq;
  cmdGetIoPinTriggerResp       getIoPinTriggerResp;
  cmdGetGlobalOptionsReq       getGlobalOptionsReq;
  cmdGetGlobalOptionsResp      getGlobalOptionsResp;
  cmdGetSoftwareInfoReq        getSoftwareInfoReq;
  cmdGetSoftwareInfoResp       getSoftwareInfoResp;
  cmdGetStatisticsReq          getStatisticsReq;
  cmdGetStatisticsResp         getStatisticsResp;
  cmdGetTimerReq               getTimerReq;
  cmdGetTimerResp              getTimerResp;
  cmdNoCommand                 noCommand;
  cmdReadClockReq              readClockReq;
  cmdReadClockResp             readClockResp;
  cmdResetChipReq              resetChipReq;
  cmdResetCardReq              resetCardReq;
  cmdResetStatisticsReq        resetStatisticsReq;
  cmdSetBusparamsReq           setBusparamsReq;
  cmdSetDrivermodeReq          setDrivermodeReq;
  cmdSetHwFilterReq            setHwFilterReq;
  cmdSetSwFilterReq            setSwFilterReq;
  cmdSetIoPinStateReq          setIoPinStateReq;
  cmdSetIoPinTriggerReq        setIoPinTriggerReq;
  cmdSetGlobalOptionsReq       setGlobalOptionsReq;
  cmdSetTimerReq               setTimerReq;
  cmdStartApplicationReq       startApplicationReq;
  cmdStartChipReq              startChipReq;
  cmdStartChipResp             startChipResp;
  cmdStopChipReq               stopChipReq;
  cmdStopChipResp              stopChipResp;
  cmdTimerEvent                timerEvent;
  cmdTriggerEvent              triggerEvent;
  cmdResetClock                resetClock;
  cmdSetHeartbeatReq           setHeartbeatReq;
  cmdSetTransceiverModeReq     setTransceiverModeReq;
  cmdSetTransceiverModeResp    setTransceiverModeResp;
  cmdTransceiverEvent          transceiverEvent;
  cmdGetTransceiverInfoReq     getTransceiverInfoReq;
  cmdGetTransceiverInfoResp    getTransceiverInfoResp;
  cmdReadParameterReq          readParameterReq;
  cmdReadParameterResp         readParameterResp;
  cmdDetectTransceivers        detectTransceivers;
  cmdReadClockNowReq           readClockNowReq;
  cmdReadClockNowResp          readClockNowResp;
  cmdFilterMessage             filterMessage;
  cmdFlushQueue                flushQueue;
} lpcCmd;


#ifndef __C166__
typedef union {
  cmdHead head;
  unsigned char b[1];
  cmdShort s;
  cmdLong l;

  cmdRxStdMessage              rxStdMessage;
  cmdRxExtMessage              rxExtMessage;
  cmdRxStdMessageX             rxStdMessageX;
  cmdRxExtMessageX             rxExtMessageX;
} lpcResp;
#endif

#include "poppack.h"

#endif
