#ifndef OSIF_COMMON_H_
#define OSIF_COMMON_H_

/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

#if LINUX
//###############################################################
#   include <linux/types.h>
#   include <linux/irqreturn.h>

// DEFINES
#   define OS_IF_INLINE inline

#if LINUX_2_6
#   define OS_IF_INTR_HANDLER irqreturn_t
#else
#   define OS_IF_INTR_HANDLER void
#endif

#else // end LINUX

//###############################################################
// win32 defines

// INCLUDES
#   include <windows.h>

// DEFINES
#   define OS_IF_INLINE __inline


#endif // win32


#endif //OSIF_COMMON_H_
