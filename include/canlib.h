/*
**                   Copyright 1994-2009 by KVASER AB, SWEDEN      
**                        WWW: http://www.kvaser.com
**
** This software is furnished under a license and may be used and copied
** only in accordance with the terms of such license.
**
** Description:
**   Definitions for the CANLIB API.
**   Do not use this file in a 16-bit environment.
**   
**   MATLAB users on Windows: if you define WIN32_LEAN_AND_MEAN before 
**   including this file, you will see a lot less warnings.
** ---------------------------------------------------------------------------
*/
#ifndef _CANLIB_H_
#define _CANLIB_H_

#include <stdlib.h>

#if defined(_WIN32_WCE)
# include <windows.h>
# if defined(DEBUG) && DEBUG
#   define CANLIB_DECLARE_ALL
# endif
#elif defined(_WIN32)
# include <windows.h>
# include "predef.h"
# include "canevt.h"
# define CANLIB_DECLARE_ALL
#else
# if defined(DEBUG) && DEBUG
#   define CANLIB_DECLARE_ALL
# endif
typedef unsigned char BYTE;
typedef unsigned int DWORD;
typedef unsigned int HANDLE;   // Only for a non-implemented function.
typedef unsigned int BOOL;     // Only for a non-implemented function.
#endif
#include "canstat.h"

typedef int canHandle;
#if defined(_WIN32) && !defined(_WIN32_WCE)
# define CanHandle int
# define canINVALID_HANDLE      (-1)
#else
   typedef canHandle CanHandle;

/* Note the difference from the windows version */
// #define canINVALID_HANDLE      0

typedef struct canNotifyData {
  void *tag;
  int eventType;
  union {
    struct {
      unsigned long time;
    } busErr;
    struct {
      long id;
      unsigned long time;
    } rx;
    struct {
      long id;
      unsigned long time;
    } tx;
    struct {
      unsigned char busStatus;
      unsigned char txErrorCounter;
      unsigned char rxErrorCounter;
      unsigned long time;
    } status;
  } info;
} canNotifyData;

#endif

/* Notify message sent to the application window */
#if defined(_WIN32) && !defined(_WIN32_WCE)
# define WM__CANLIB              (WM_USER+16354)
#else
# define WM__CANLIB 648
#endif



// Flags for canOpenChannel
// 0x01, 0x02, 0x04 are obsolete and reserved.
// The canWANT_xxx names are also obsolete, use canOPEN_xxx instead for new developments.
#define canWANT_EXCLUSIVE               0x0008
#define canWANT_EXTENDED                0x0010
#define canWANT_VIRTUAL                 0x0020
#define canOPEN_EXCLUSIVE               canWANT_EXCLUSIVE
#define canOPEN_REQUIRE_EXTENDED        canWANT_EXTENDED
#define canOPEN_ACCEPT_VIRTUAL          canWANT_VIRTUAL
#if defined(CANLIB_DECLARE_ALL)
# define canOPEN_OVERRIDE_EXCLUSIVE     0x0040
# define canOPEN_REQUIRE_INIT_ACCESS    0x0080
# define canOPEN_NO_INIT_ACCESS         0x0100
# define canOPEN_ACCEPT_LARGE_DLC       0x0200  // DLC can be greater than 8
#endif

// Flags for canAccept
#define canFILTER_ACCEPT        1
#define canFILTER_REJECT        2
#define canFILTER_SET_CODE_STD  3
#define canFILTER_SET_MASK_STD  4
#define canFILTER_SET_CODE_EXT  5
#define canFILTER_SET_MASK_EXT  6

#define canFILTER_NULL_MASK     0L


//
// CAN driver types - not all are supported on all cards.
//
#define canDRIVER_NORMAL           4
#define canDRIVER_SILENT           1
#define canDRIVER_SELFRECEPTION    8
#define canDRIVER_OFF              0
// 2,3,5,6,7 are reserved values for compatibility reasons.

/*
** Common bus speeds. Used in canSetBusParams.
** The values are translated in canlib, canTranslateBaud().
*/
#define canBITRATE_1M        (-1)
#define canBITRATE_500K      (-2)
#define canBITRATE_250K      (-3)
#define canBITRATE_125K      (-4)
#define canBITRATE_100K      (-5)
#define canBITRATE_62K       (-6)
#define canBITRATE_50K       (-7)
#define canBITRATE_83K       (-8)
#define canBITRATE_10K       (-9)

// The BAUD_xxx names are retained for compability.
#define BAUD_1M        (-1)
#define BAUD_500K      (-2)
#define BAUD_250K      (-3)
#define BAUD_125K      (-4)
#define BAUD_100K      (-5)
#define BAUD_62K       (-6)
#define BAUD_50K       (-7)
#define BAUD_83K       (-8)




/*
** IOCTL types
*/
#define canIOCTL_PREFER_EXT             1
#define canIOCTL_PREFER_STD             2
// 3,4 reserved.
#define canIOCTL_CLEAR_ERROR_COUNTERS   5
#define canIOCTL_SET_TIMER_SCALE        6
#define canIOCTL_SET_TXACK              7


#define CANID_METAMSG  (-1L)        // Like msgs containing bus status changes
#define CANID_WILDCARD (-2L)        // We don't care or don't know

//
// Define CANLIBAPI unless it's done already.
// (canlib.c provides its own definitions of CANLIBAPI, DLLIMPORT
// and DLLEXPORT before including this file.)
//
#ifndef CANLIBAPI
# ifdef _WIN32
#   define CANLIBAPI __stdcall
#   define DLLIMPORT __declspec(dllimport)
#   define DLLEXPORT __declspec(dllexport)
# else
#   define CANLIBAPI 
#   define __stdcall
# endif
#endif


#ifdef __cplusplus
extern "C" {
#endif

void CANLIBAPI canInitializeLibrary(void);

canStatus CANLIBAPI canClose(const CanHandle hnd);

canStatus CANLIBAPI canBusOn(const CanHandle hnd);

canStatus CANLIBAPI canBusOff(const CanHandle hnd);

canStatus CANLIBAPI canSetBusParams(const CanHandle hnd,
                                    long freq,
                                    unsigned int tseg1,
                                    unsigned int tseg2,
                                    unsigned int sjw,
                                    unsigned int noSamp,
                                    unsigned int syncmode);

canStatus CANLIBAPI canGetBusParams(const CanHandle hnd,
                                    long  *freq,
                                    unsigned int *tseg1,
                                    unsigned int *tseg2,
                                    unsigned int *sjw,
                                    unsigned int *noSamp,
                                    unsigned int *syncmode);

canStatus CANLIBAPI canSetBusOutputControl(const CanHandle hnd,
                                           const unsigned int drivertype);

canStatus CANLIBAPI canGetBusOutputControl(const CanHandle hnd,
                                           unsigned int *drivertype);

canStatus CANLIBAPI canAccept(const CanHandle hnd,
                              const long envelope,
                              const unsigned int flag);

canStatus CANLIBAPI canReadStatus(const CanHandle hnd,
                                  unsigned long * const flags);

canStatus CANLIBAPI canReadErrorCounters(const CanHandle hnd,
                                         unsigned int *txErr,
                                         unsigned int *rxErr,
                                         unsigned int *ovErr);

canStatus CANLIBAPI canWrite(const CanHandle hnd, long id, void *msg,
                             unsigned int dlc, unsigned int flag);
  /*  */
canStatus CANLIBAPI canWriteSync(const CanHandle hnd, unsigned long timeout);

canStatus CANLIBAPI canRead(const CanHandle hnd,
                            long *id,
                            void *msg,
                            unsigned int *dlc,
                            unsigned int *flag,
                            unsigned long *time);

canStatus CANLIBAPI canReadWait(const CanHandle hnd,
                                long *id,
                                void *msg,
                                unsigned int  *dlc,
                                unsigned int  *flag,
                                unsigned long *time,
                                unsigned long timeout);

#if defined(CANLIB_DECLARE_ALL)
canStatus CANLIBAPI canReadSpecific(const CanHandle hnd, long id, void * msg,
                                    unsigned int * dlc, unsigned int * flag,
                                    unsigned long * time);

canStatus CANLIBAPI canReadSync(const CanHandle hnd, unsigned long timeout);

canStatus CANLIBAPI canReadSyncSpecific(const CanHandle hnd, long id, unsigned long timeout);

canStatus CANLIBAPI canReadSpecificSkip(const CanHandle hnd,
                                        long id,
                                        void * msg,
                                        unsigned int * dlc,
                                        unsigned int * flag,
                                        unsigned long * time);
#endif

#if defined(_WIN32)
canStatus CANLIBAPI canSetNotify(const CanHandle hnd, 
                                 HWND aHWnd,
                                 unsigned int aNotifyFlags);
#else
canStatus CANLIBAPI canSetNotify(const CanHandle hnd,
                                 void (*callback)(canNotifyData *),
                                 unsigned int notifyFlags, void *tag);
#endif

#if defined(_WIN32_WCE) || !defined(_WIN32)
canStatus CANLIBAPI canGetRawHandle(const CanHandle hnd, void *pvFd);
#endif

canStatus CANLIBAPI canTranslateBaud(long *const freq,
                                     unsigned int *const tseg1,
                                     unsigned int *const tseg2,
                                     unsigned int *const sjw,
                                     unsigned int *const nosamp,
                                     unsigned int *const syncMode);

canStatus CANLIBAPI canGetErrorText(canStatus err, char *buf, unsigned int bufsiz);

unsigned short CANLIBAPI canGetVersion(void);

canStatus CANLIBAPI canIoCtl(const CanHandle hnd, unsigned int func,
                             void *buf, unsigned int buflen);

/* Note the difference from the windows version */
#if defined(_WIN32) && !defined(_WIN32_WCE)
unsigned long CANLIBAPI canReadTimer(const CanHandle hnd);
#else
canStatus CANLIBAPI canReadTimer(const CanHandle hnd, unsigned long *time);
#endif

CanHandle CANLIBAPI canOpenChannel(int channel, int flags);

canStatus CANLIBAPI canGetNumberOfChannels(int *channelCount);

canStatus CANLIBAPI canGetChannelData(int channel, int item, void *buffer, size_t bufsize);

#define canCHANNELDATA_CHANNEL_CAP                1
#define canCHANNELDATA_TRANS_CAP                  2
#define canCHANNELDATA_CHANNEL_FLAGS              3   // available, etc
#define canCHANNELDATA_CARD_TYPE                  4   // canHWTYPE_xxx
#define canCHANNELDATA_CARD_NUMBER                5   // Number in machine, 0,1,...
#define canCHANNELDATA_CHAN_NO_ON_CARD            6
#define canCHANNELDATA_CARD_SERIAL_NO             7
#define canCHANNELDATA_TRANS_SERIAL_NO            8
#define canCHANNELDATA_CARD_FIRMWARE_REV          9
#define canCHANNELDATA_CARD_HARDWARE_REV          10
#define canCHANNELDATA_CARD_UPC_NO                11
#define canCHANNELDATA_TRANS_UPC_NO               12
#define canCHANNELDATA_CHANNEL_NAME               13
#if defined(CANLIB_DECLARE_ALL)
# define canCHANNELDATA_DLL_FILE_VERSION          14
# define canCHANNELDATA_DLL_PRODUCT_VERSION       15
# define canCHANNELDATA_DLL_FILETYPE              16
# define canCHANNELDATA_TRANS_TYPE                17
# define canCHANNELDATA_DEVICE_PHYSICAL_POSITION  18
# define canCHANNELDATA_UI_NUMBER                 19
# define canCHANNELDATA_TIMESYNC_ENABLED          20
# define canCHANNELDATA_DRIVER_FILE_VERSION       21
# define canCHANNELDATA_DRIVER_PRODUCT_VERSION    22
# define canCHANNELDATA_MFGNAME_UNICODE           23
# define canCHANNELDATA_MFGNAME_ASCII             24
# define canCHANNELDATA_DEVDESCR_UNICODE          25
# define canCHANNELDATA_DEVDESCR_ASCII            26
# define canCHANNELDATA_DRIVER_NAME               27
#endif


// channelFlags in canChannelData
#define canCHANNEL_IS_EXCLUSIVE         0x0001
#define canCHANNEL_IS_OPEN              0x0002


// Hardware types.
#define canHWTYPE_NONE         0        // Unknown
#define canHWTYPE_VIRTUAL      1        // Virtual channel.
#define canHWTYPE_LAPCAN       2        // LAPcan Family
#define canHWTYPE_CANPARI      3        // CANpari (obsolete)
#define canHWTYPE_PCCAN        8        // PCcan Family
#define canHWTYPE_PCICAN       9        // PCIcan Family
#define canHWTYPE_USBCAN      11        // USBcan (obsolete)
#define canHWTYPE_PCICAN_II   40        // PCIcan II family
#define canHWTYPE_USBCAN_II   42        // USBcan II, Memorator et al
#define canHWTYPE_SIMULATED   44        // (obsolete)
#define canHWTYPE_ACQUISITOR  46        // Acquisitor (obsolete)
#define canHWTYPE_LEAF        48        // Kvaser Leaf Family
#define canHWTYPE_PC104_PLUS     50     // PC104+
#define canHWTYPE_PCICANX_II     52     // PCIcanx II
#define canHWTYPE_MEMORATOR_II   54     // Memorator Professional
#define canHWTYPE_MEMORATOR_PRO  54     // Memorator Professional
#define canHWTYPE_USBCAN_PRO  56        // USBcan Professional
#define canHWTYPE_IRIS        58        // Iris
#define canHWTYPE_BLACKBIRD   58        // BlackBird
#define canHWTYPE_MEMORATOR_LIGHT 60    // Memorator Light
#define canHWTYPE_MINIHYDRA   62        // Eagle
#define canHWTYPE_EAGLE       62        // Kvaser Eagle family 

// Channel capabilities.
#define canCHANNEL_CAP_EXTENDED_CAN         0x00000001L
#define canCHANNEL_CAP_BUS_STATISTICS       0x00000002L
#define canCHANNEL_CAP_ERROR_COUNTERS       0x00000004L
#define canCHANNEL_CAP_CAN_DIAGNOSTICS      0x00000008L
#define canCHANNEL_CAP_GENERATE_ERROR       0x00000010L
#define canCHANNEL_CAP_GENERATE_OVERLOAD    0x00000020L
#define canCHANNEL_CAP_TXREQUEST            0x00000040L
#define canCHANNEL_CAP_TXACKNOWLEDGE        0x00000080L
#define canCHANNEL_CAP_VIRTUAL              0x00010000L
#define canCHANNEL_CAP_SIMULATED            0x00020000L
#define canCHANNEL_CAP_REMOTE               0x00040000L // Remote device, like BlackBird

// Driver (transceiver) capabilities
#define canDRIVER_CAP_HIGHSPEED             0x00000001L

#define canIOCTL_GET_RX_BUFFER_LEVEL              8
#define canIOCTL_GET_TX_BUFFER_LEVEL              9
#define canIOCTL_FLUSH_RX_BUFFER                  10
#define canIOCTL_FLUSH_TX_BUFFER                  11
#define canIOCTL_GET_TIMER_SCALE                  12
#define canIOCTL_SET_TXRQ                         13
#define canIOCTL_GET_EVENTHANDLE                  14
#define canIOCTL_SET_BYPASS_MODE                  15
#define canIOCTL_SET_WAKEUP                      16
#if defined(CANLIB_DECLARE_ALL)
# define canIOCTL_GET_DRIVERHANDLE                17
# define canIOCTL_MAP_RXQUEUE                     18
# define canIOCTL_GET_WAKEUP                      19
# define canIOCTL_SET_REPORT_ACCESS_ERRORS        20
# define canIOCTL_GET_REPORT_ACCESS_ERRORS        21
# define canIOCTL_CONNECT_TO_VIRTUAL_BUS          22
# define canIOCTL_DISCONNECT_FROM_VIRTUAL_BUS     23
# define canIOCTL_SET_USER_IOPORT                 24
# define canIOCTL_GET_USER_IOPORT                 25
# define canIOCTL_SET_BUFFER_WRAPAROUND_MODE      26
# define canIOCTL_SET_RX_QUEUE_SIZE               27
# define canIOCTL_SET_USB_THROTTLE                28
# define canIOCTL_GET_USB_THROTTLE                29
# define canIOCTL_SET_BUSON_TIME_AUTO_RESET       30
#endif
#define canIOCTL_GET_TXACK                        31  // used to be 17

#if defined(CANLIB_DECLARE_ALL)
// For canIOCTL_xxx_USER_IOPORT
typedef struct {
  unsigned int portNo;
  unsigned int portValue;
} canUserIoPortData;

#endif

#if defined(CANLIB_DECLARE_ALL)
canStatus CANLIBAPI canWaitForEvent(const CanHandle hnd, DWORD timeout);
#endif

canStatus CANLIBAPI canSetBusParamsC200(const CanHandle hnd, BYTE btr0, BYTE btr1);

#if defined(CANLIB_DECLARE_ALL)
canStatus CANLIBAPI canSetDriverMode(const CanHandle hnd, int lineMode, int resNet);
canStatus CANLIBAPI canGetDriverMode(const CanHandle hnd, int *lineMode, int *resNet);
#endif

// Item codes for canGetVersionEx()
#define canVERSION_CANLIB32_VERSION     0
#define canVERSION_CANLIB32_PRODVER     1
#define canVERSION_CANLIB32_PRODVER32   2
#define canVERSION_CANLIB32_BETA        3

#if defined(CANLIB_DECLARE_ALL)
unsigned int CANLIBAPI canGetVersionEx(unsigned int itemCode);

canStatus CANLIBAPI canParamGetCount (void);

canStatus CANLIBAPI canParamCommitChanges (void);

canStatus CANLIBAPI canParamDeleteEntry (int index);

canStatus CANLIBAPI canParamCreateNewEntry (void);

canStatus CANLIBAPI canParamSwapEntries (int index1, int index2);

canStatus CANLIBAPI canParamGetName (int index, char *buffer, int maxlen);

canStatus CANLIBAPI canParamGetChannelNumber (int index);

canStatus CANLIBAPI canParamGetBusParams (int index,
                                          long* bitrate,
                                          unsigned int *tseg1,
                                          unsigned int *tseg2,
                                          unsigned int *sjw,
                                          unsigned int *noSamp);

canStatus CANLIBAPI canParamSetName (int index, const char *buffer);

canStatus CANLIBAPI canParamSetChannelNumber (int index, int channel);

canStatus CANLIBAPI canParamSetBusParams (int index,
                                          long bitrate,
                                          unsigned int tseg1,
                                          unsigned int tseg2,
                                          unsigned int sjw,
                                          unsigned int noSamp);

canStatus CANLIBAPI canParamFindByName (const char *name);

#endif


// Frees all object buffers associated with the specified handle.
canStatus CANLIBAPI canObjBufFreeAll(const CanHandle hnd);

// Allocates an object buffer of the specified type.
canStatus CANLIBAPI canObjBufAllocate(const CanHandle hnd, int type);
#define canOBJBUF_TYPE_AUTO_RESPONSE            0x01
#define canOBJBUF_TYPE_PERIODIC_TX              0x02

// Deallocates the object buffer with the specified index.
canStatus CANLIBAPI canObjBufFree(const CanHandle hnd, int idx);

// Writes CAN data to the object buffer with the specified index.
canStatus CANLIBAPI canObjBufWrite(const CanHandle hnd, int idx, int id, void* msg,
                                   unsigned int dlc, unsigned int flags);

// For an AUTO_RESPONSE buffer, set the code and mask that together define
// the identifier(s) that trigger(s) the automatic response.
canStatus CANLIBAPI canObjBufSetFilter(const CanHandle hnd, int idx,
                                       unsigned int code, unsigned int mask);

// Sets buffer-speficic flags.
canStatus CANLIBAPI canObjBufSetFlags(const CanHandle hnd, int idx, unsigned int flags);
// The buffer responds to RTRs only, not regular messages.
// AUTO_RESPONSE buffers only
#define canOBJBUF_AUTO_RESPONSE_RTR_ONLY        0x01

// Sets transmission period for auto tx buffers.
canStatus CANLIBAPI canObjBufSetPeriod(const CanHandle hnd, int idx, unsigned int period);

// Sets message count for auto tx buffers.
canStatus CANLIBAPI canObjBufSetMsgCount(const CanHandle hnd, int idx, unsigned int count);

// Enable object buffer with index idx.
canStatus CANLIBAPI canObjBufEnable(const CanHandle hnd, int idx);

// Disable object buffer with index idx.
canStatus CANLIBAPI canObjBufDisable(const CanHandle hnd, int idx);

// For certain diagnostics.
canStatus CANLIBAPI canObjBufSendBurst(const CanHandle hnd, int idx, unsigned int burstlen);


#if defined(CANLIB_DECLARE_ALL)

// Check for specific version(s) of CANLIB.
#define canVERSION_DONT_ACCEPT_LATER      0x01
#define canVERSION_DONT_ACCEPT_BETAS      0x02
BOOL CANLIBAPI canProbeVersion(const CanHandle hnd, int major, int minor, int oem_id, unsigned int flags);

#endif


// Try to "reset" the CAN bus.
canStatus CANLIBAPI canResetBus(const CanHandle hnd);

// Convenience function that combines canWrite and canWriteSync.
canStatus CANLIBAPI canWriteWait(const CanHandle hnd, long id, void *msg,
                                 unsigned int dlc, unsigned int flag,
                                 unsigned long timeout);


#if defined(CANLIB_DECLARE_ALL)

// Tell canlib32.dll to unload its DLLs.
canStatus CANLIBAPI canUnloadLibrary(void);

canStatus CANLIBAPI canSetAcceptanceFilter(const CanHandle hnd, unsigned int code,
                                           unsigned int mask, int is_extended);
#endif

canStatus CANLIBAPI canFlushReceiveQueue(const CanHandle hnd);
canStatus CANLIBAPI canFlushTransmitQueue(const CanHandle hnd);


#if defined(CANLIB_DECLARE_ALL)
canStatus CANLIBAPI kvGetApplicationMapping(int busType,
                                  char *appName,
                                  int appChannel,
                                  int *resultingChannel);

canStatus CANLIBAPI kvBeep(const CanHandle hnd, int freq, unsigned int duration);

canStatus CANLIBAPI kvSelfTest(const CanHandle hnd, unsigned long *presults);


#define kvLED_ACTION_ALL_LEDS_ON    0
#define kvLED_ACTION_ALL_LEDS_OFF   1  
#define kvLED_ACTION_LED_0_ON       2
#define kvLED_ACTION_LED_0_OFF      3
#define kvLED_ACTION_LED_1_ON       4
#define kvLED_ACTION_LED_1_OFF      5
#define kvLED_ACTION_LED_2_ON       6
#define kvLED_ACTION_LED_2_OFF      7
#define kvLED_ACTION_LED_3_ON       8
#define kvLED_ACTION_LED_3_OFF      9

canStatus CANLIBAPI kvFlashLeds(const CanHandle hnd, int action, int timeout);

canStatus CANLIBAPI canRequestChipStatus(const CanHandle hnd);

canStatus CANLIBAPI canRequestBusStatistics(const CanHandle hnd);

typedef struct canBusStatistics_s {
  unsigned long  stdData;
  unsigned long  stdRemote;
  unsigned long  extData;
  unsigned long  extRemote;
  unsigned long  errFrame;      // Error frames
  unsigned long  busLoad;       // 0 .. 10000 meaning 0.00-100.00%
  unsigned long  overruns;
} canBusStatistics;

canStatus CANLIBAPI canGetBusStatistics(const CanHandle hnd, canBusStatistics *stat, size_t bufsiz);

canStatus CANLIBAPI canSetBitrate(const CanHandle hnd, int bitrate);

canStatus CANLIBAPI kvAnnounceIdentity(const CanHandle hnd, void *buf, size_t bufsiz);

canStatus CANLIBAPI canGetHandleData(const CanHandle hnd, int item, void *buffer, size_t bufsize);


typedef void *kvTimeDomain;
#endif
typedef canStatus kvStatus;

#if defined(CANLIB_DECLARE_ALL)
typedef struct kvTimeDomainData_s {
  int nMagiSyncGroups;
  int nMagiSyncedMembers;
  int nNonMagiSyncCards;
  int nNonMagiSyncedMembers;
} kvTimeDomainData;

kvStatus CANLIBAPI kvTimeDomainCreate(kvTimeDomain *domain);
kvStatus CANLIBAPI kvTimeDomainDelete(kvTimeDomain domain);

kvStatus CANLIBAPI kvTimeDomainResetTime(kvTimeDomain domain);
kvStatus CANLIBAPI kvTimeDomainGetData(kvTimeDomain domain, kvTimeDomainData *data, size_t bufsiz);

kvStatus CANLIBAPI kvTimeDomainAddHandle(kvTimeDomain domain, const CanHandle hnd);
kvStatus CANLIBAPI kvTimeDomainRemoveHandle(kvTimeDomain domain, const CanHandle hnd);
#endif


typedef void (CANLIBAPI *kvCallback_t)(CanHandle hnd, void* context, unsigned int notifyEvent);
kvStatus CANLIBAPI kvSetNotifyCallback(const CanHandle hnd, kvCallback_t callback, void* context, unsigned int notifyFlags);


#if defined(CANLIB_DECLARE_ALL)
#define kvBUSTYPE_NONE          0
#define kvBUSTYPE_PCI           1
#define kvBUSTYPE_PCMCIA        2
#define kvBUSTYPE_USB           3
#define kvBUSTYPE_WLAN          4
#define kvBUSTYPE_PCI_EXPRESS   5
#define kvBUSTYPE_ISA           6
#define kvBUSTYPE_VIRTUAL       7
#define kvBUSTYPE_PC104_PLUS    8

kvStatus CANLIBAPI kvGetSupportedInterfaceInfo(int index, char *hwName, size_t nameLen, int *hwType, int *hwBusType);


kvStatus CANLIBAPI kvReadTimer(const CanHandle hnd, unsigned int *time);
#if defined(KVINT64)
kvStatus CANLIBAPI kvReadTimer64(const CanHandle hnd, KVINT64 *time);
#endif

kvStatus CANLIBAPI kvReadDeviceCustomerData(const CanHandle hnd, int userNumber, int itemNumber, void *data, size_t bufsiz);

//いいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいい
//
// APIs for t-script
// 
#define ENVVAR_TYPE_INT       1
#define ENVVAR_TYPE_FLOAT     2
#define ENVVAR_TYPE_STRING    3

#if defined(_WIN32_WCE) || defined(_WIN32)
typedef __int64 kvEnvHandle;
#else
typedef long long kvEnvHandle;
#endif

kvStatus CANLIBAPI kvScriptStart(const CanHandle hnd, int scriptNo); 

kvStatus CANLIBAPI kvScriptStop(const CanHandle hnd, int scriptNo); 

kvStatus CANLIBAPI kvScriptForceStop(const CanHandle hnd, int scriptNo);

kvStatus CANLIBAPI kvScriptSendEvent(const CanHandle hnd, int scriptNo, int eventNo, int data);

kvEnvHandle CANLIBAPI kvScriptEnvvarOpen(const CanHandle hnd, char* envvarName, int scriptNo, int *envvarType, int *envvarSize); // returns scriptHandle

kvStatus     CANLIBAPI kvScriptEnvvarClose(kvEnvHandle eHnd);

kvStatus     CANLIBAPI kvScriptEnvvarSetInt(kvEnvHandle eHnd, int val);

kvStatus     CANLIBAPI kvScriptEnvvarGetInt(kvEnvHandle eHnd, int *val);

kvStatus     CANLIBAPI kvScriptEnvvarSetData(kvEnvHandle eHnd, unsigned char *buf, int start_index, int data_len);

kvStatus     CANLIBAPI kvScriptEnvvarGetData(kvEnvHandle eHnd, unsigned char *buf, int start_index, int data_len);

kvStatus     CANLIBAPI kvScriptGetMaxEnvvarSize(const CanHandle hnd, int *envvarSize);

kvStatus CANLIBAPI kvScriptLoadFileOnDevice(const CanHandle hnd, int scriptNo, char *localFile);

kvStatus CANLIBAPI kvScriptLoadFile(const CanHandle hnd, int scriptNo, char *filePathOnPC);

kvStatus CANLIBAPI kvFileCopyToDevice(const CanHandle hnd, char *hostFileName, char *deviceFileName);

kvStatus CANLIBAPI kvFileCopyFromDevice(const CanHandle hnd, char *deviceFileName, char *hostFileName);

kvStatus CANLIBAPI kvFileDelete(const CanHandle hnd, char *deviceFileName);

kvStatus CANLIBAPI kvFileGetFileData(const CanHandle hnd, int fileNo, ... );                 // return list with names, sizes, optional checksums

kvStatus CANLIBAPI kvGetFileCount(const CanHandle hnd, int *count);                          // return number of files

kvStatus CANLIBAPI kvFileGetSystemData(const CanHandle hnd, int itemCode, int *result);

//いいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいいい


//
// The following functions are not yet implemented. Do not use them.
//
#if defined(_CANEVT_H_)
canStatus CANLIBAPI canReadEvent(const CanHandle hnd, CanEvent *event);
#endif
void CANLIBAPI canSetDebug(int d);
canStatus CANLIBAPI canSetNotifyEx(const CanHandle hnd, HANDLE event, unsigned int flags);
canStatus CANLIBAPI canSetTimer(const CanHandle hnd, DWORD interval, DWORD flags);
#define canTIMER_CYCLIC             0x01
#define canTIMER_EXPENSIVE          0x02
int CANLIBAPI canSplitHandle(CanHandle hnd, int channel);
int CANLIBAPI canOpenMultiple(DWORD bitmask, int flags);
#endif

#ifdef __cplusplus
}
#endif

#if defined(_WIN32) && !defined(_WIN32_WCE)
# include "obsolete.h"
#endif

#endif
