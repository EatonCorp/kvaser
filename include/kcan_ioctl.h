/* kcan_ioctl.h: ioctls()'s specific for Kvasers CAN drivers */

#ifndef _KCAN_IOCTL_H
#define _KCAN_IOCTL_H

#if LINUX
//#   include <linux/ioctl.h>
#   include <asm/ioctl.h>

#   define KCAN_IOC_MAGIC 'k'
#else
#   define KCAN_IOC_MAGIC           842 // Just made it up /JK
#   define _IO(x,y) x+y
#   define _IOW(x,y,z) x+y
#   define _IOR(x,y,z) x+y
#endif

// For compatibility with Windows #define:s below.
#define VCAN_DEVICE      0     // dummy
#define KCAN_IOCTL_START 0
#define METHOD_BUFFERED  0     // dummy
#define FILE_ANY_ACCESS  0

// qqq This likely is not a good idea!
#if !LINUX
#  undef CTL_CODE
#endif

#define CTL_CODE(x,i,y,z) _IO(KCAN_IOC_MAGIC, (i))


#define KCAN_IOCTL_OBJBUF_FREE_ALL              CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 6, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_ALLOCATE              CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 7, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_FREE                  CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 8, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_WRITE                 CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 9, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SET_FILTER            CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 10, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SET_FLAGS             CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 11, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_ENABLE                CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 12, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_DISABLE               CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 13, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_OBJBUF_SET_PERIOD            CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 22, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define KCAN_IOCTL_OBJBUF_SEND_BURST            CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 23, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define KCAN_IOCTL_OBJBUF_SET_MSG_COUNT         CTL_CODE (VCAN_DEVICE, KCAN_IOCTL_START + 34, METHOD_BUFFERED, FILE_ANY_ACCESS)


#define KCAN_CARDFLAG_FIRMWARE_BETA       0x01  // Firmware is beta
#define KCAN_CARDFLAG_FIRMWARE_RC         0x02  // Firmware is release candidate
#define KCAN_CARDFLAG_AUTO_RESP_OBJBUFS   0x04  // Firmware supports auto-response object buffers
#define KCAN_CARDFLAG_REFUSE_TO_RUN       0x08  // Major problem detected
#define KCAN_CARDFLAG_REFUSE_TO_USE_CAN   0x10  // Major problem detected
#define KCAN_CARDFLAG_AUTO_TX_OBJBUFS     0x20  // Firmware supports periodic transmit object buffers

#if 0 // qqq Perhaps do this?
#if defined(DEVHND_CARD_FIRMWARE_BETA)
CompilerAssert(KCAN_CARDFLAG_FIRMWARE_BETA    == DEVHND_CARD_FIRMWARE_BETA    );
CompilerAssert(KCAN_CARDFLAG_FIRMWARE_RC      == DEVHND_CARD_FIRMWARE_RC      );
CompilerAssert(KCAN_CARDFLAG_AUTO_RESP_OBJBUFS== DEVHND_CARD_AUTO_RESP_OBJBUFS);
CompilerAssert(KCAN_CARDFLAG_REFUSE_TO_RUN    == DEVHND_CARD_REFUSE_TO_RUN    );
CompilerAssert(KCAN_CARDFLAG_REFUSE_TO_USE_CAN== DEVHND_CARD_REFUSE_TO_USE_CAN);
CompilerAssert(KCAN_CARDFLAG_AUTO_TX_OBJBUFS  == DEVHND_CARD_AUTO_TX_OBJBUFS  );
#endif
#endif

#define KCAN_DRVFLAG_BETA                 0x01    // Driver is beta

#if defined(DEVHND_DRIVER_IS_BETA)
CompilerAssert(KCAN_DRVFLAG_BETA == DEVHND_DRIVER_IS_BETA);
#endif

#endif /* KCANIO_H */

