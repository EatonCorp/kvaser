#if !defined(DEBUG_H)
#define DEBUG_H

//
// DEBUG
//
#if DEBUG

#  if 0
#    define ZONE_ERR            DEBUGZONE(0)
#    define ZONE_WARN           DEBUGZONE(1)
#    define ZONE_INIT           DEBUGZONE(2)
#    define ZONE_TRACE          DEBUGZONE(3)

#    define ZONE_CAN_INIT       DEBUGZONE(4)
#    define ZONE_CAN_READ       DEBUGZONE(5)
#    define ZONE_CAN_WRITE      DEBUGZONE(6)
#    define ZONE_CAN_IOCTL      DEBUGZONE(7)

#    define ZONE_USB_PARSE      DEBUGZONE(8)
#    define ZONE_USB_INIT       DEBUGZONE(9)
#    define ZONE_USB_CONTROL    DEBUGZONE(10)
#    define ZONE_USB_BULK       DEBUGZONE(11)

//#define ZONE_USBCLIENT      DEBUGZONE(15)
#  else
#    if 0
#      define ZONE_ERR            1
#      define ZONE_WARN           1
#      define ZONE_INIT           0
#      define ZONE_TRACE          0

#      define ZONE_CAN_INIT       0
#      define ZONE_CAN_READ       0
#      define ZONE_CAN_WRITE      0
#      define ZONE_CAN_IOCTL      0

#      define ZONE_USB_PARSE      0
#      define ZONE_USB_INIT       0
#      define ZONE_USB_CONTROL    0
#      define ZONE_USB_BULK       0

#      define ZONE_J1587          0
#    else
#      define ZONE_ERR            1
#      define ZONE_WARN           1
#      define ZONE_INIT           1
#      define ZONE_TRACE          1

#      define ZONE_CAN_INIT       1
#      define ZONE_CAN_READ       1
#      define ZONE_CAN_WRITE      1
#      define ZONE_CAN_IOCTL      1

#      define ZONE_USB_PARSE      1
#      define ZONE_USB_INIT       1
#      define ZONE_USB_CONTROL    1
#      define ZONE_USB_BULK       1

#      define ZONE_J1587          1
#    endif
//#define ZONE_USBCLIENT      0
#  endif

#  else
#    define ZONE_ERR            0
#    define ZONE_WARN           0
#    define ZONE_INIT           0
#    define ZONE_TRACE          0

#    define ZONE_CAN_INIT       0
#    define ZONE_CAN_READ       0
#    define ZONE_CAN_WRITE      0
#    define ZONE_CAN_IOCTL      0

#    define ZONE_USB_PARSE      0
#    define ZONE_USB_INIT       0
#    define ZONE_USB_CONTROL    0
#    define ZONE_USB_BULK       0

#    define ZONE_J1587          0
#  endif

#  if LINUX
#    define TXT(t)                    t
#    define TXT2(t)                   t
#    define DEBUGPRINTHELP(args...)   args
#    define DEBUGOUT(c, arg)          if (c)                                 \
                                        printk("<" #c ">" DEBUGPRINTHELP arg)

#    if DEBUG
#      define CompilerAssert(e) extern char _kvaser_compiler_assert_[(e)?1:-1]
#    endif

//----------------------------------------------------------------------------
#  else /* !LINUX */
/*#    if !defined(ARM)
#      if DEBUG
#        define TXT(quote)               TEXT(quote)
#        define TXT2(quote)              TEXT(quote)
#        define DEBUGOUT(c, arg)         DEBUGMSG(c, arg)
#      else
#        define TXT(quote)               TEXT(quote)
#        define TXT2(quote)              TEXT(quote)
#        define DEBUGOUT(c, arg)
#      endif
#    else*/
#      define TXT(quote)               outfile, TEXT(quote)
#      define TXT2(quote)              TEXT(quote)
#      if DEBUG && _WIN32_WCE >= 0x600
#        include <winnt.h>
#        include <stdio.h>
#        define DEBUGOUT(c, arg)       if (c) {                                \
                                         FILE *outfile;                        \
                                         if (fopen_s(&outfile, "\\debug.txt", "a+")) { \
                                           fwprintf arg;                       \
                                           fclose(outfile);                    \
                                         } \
                                       }
#      elif DEBUG
#        include <winnt.h>
#        include <stdio.h>
#        define DEBUGOUT(c, arg)       if (c) {                                \
                                         FILE *outfile;                        \
                                         outfile = fopen("\\debug.txt", "a+"); \
                                         fwprintf arg;                       \
                                         fclose(outfile);                    \
                                       }
#      else
#        define DEBUGOUT(c, arg)
#      endif
#    endif
//#  endif
#endif

