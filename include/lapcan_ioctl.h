/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*
** Description:
** LAPcan driver ioctl commands for Linux 2.2
**
*/

#ifndef _LAPCAN_IOCTL_H
#define _LAPCAN_IOCTL_H

#if LINUX
#   include <linux/ioctl.h>
#   define LAPCAN_IOC_MAGIC 'l'

#else // WIN32

    // some junk to make wince work with linux macros
#   define LAPCAN_IOC_MAGIC 769 // from old wince driver
#   define _IO(x,y) x+y
#   define _IOW(x,y,z) x+y
#   define _IOR(x,y,z) x+y

#endif




#define LAPCAN_IOC_SENDCMD           _IOW(LAPCAN_IOC_MAGIC, 1, 0)
#define LAPCAN_IOC_RECVCMD           _IOR(LAPCAN_IOC_MAGIC, 2, 0)
#define LAPCAN_IOC_TEST              _IO(LAPCAN_IOC_MAGIC,4)
#define LAPCAN_IOC_SET_EXCL          _IO(LAPCAN_IOC_MAGIC,5)
#define LAPCAN_IOC_CLR_EXCL          _IO(LAPCAN_IOC_MAGIC,6)
#define LAPCAN_IOC_OPEN_CHAN         _IO(LAPCAN_IOC_MAGIC,7)
#define LAPCAN_IOC_CLOSE_CHAN        _IO(LAPCAN_IOC_MAGIC,8)
#define LAPCAN_IOC_WAIT_EMPTY        _IO(LAPCAN_IOC_MAGIC,9)
#define LAPCAN_IOC_ADD_FILTER        _IO(LAPCAN_IOC_MAGIC,10)
#define LAPCAN_IOC_CLEAR_FILTERS     _IO(LAPCAN_IOC_MAGIC,11)
#define LAPCAN_IOC_FLUSH_RCVBUFFER   _IO(LAPCAN_IOC_MAGIC,12)
#define LAPCAN_IOC_GET_STAT          _IO(LAPCAN_IOC_MAGIC,13)
#define LAPCAN_IOC_SET_WRITE_BLOCK   _IO(LAPCAN_IOC_MAGIC,14)
#define LAPCAN_IOC_SET_READ_BLOCK    _IO(LAPCAN_IOC_MAGIC,15)
#define LAPCAN_IOC_SET_WRITE_TIMEOUT _IO(LAPCAN_IOC_MAGIC,16)
#define LAPCAN_IOC_SET_READ_TIMEOUT  _IO(LAPCAN_IOC_MAGIC,17)
#define LAPCAN_IOC_FLUSH_SENDBUFFER  _IO(LAPCAN_IOC_MAGIC,18)

#endif /* _LAPCAN_IOCTL_H */









