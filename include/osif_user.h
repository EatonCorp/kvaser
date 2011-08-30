#ifndef _OSIF_USER_H_
#define _OSIF_USER_H_

/*
** Copyright 2002-2009 KVASER AB, Sweden.  All rights reserved.
*/

#if LINUX
#  include <pthread.h>
#endif

//#################################################
#if LINUX
    typedef int OS_IF_FILE_HANDLE;
#   define DEVICE_NAME_LEN 32

#   define OS_IF_INVALID_HANDLE -1

#   define OS_IF_SET_NOTIFY_PARAM  void (*callback) (canNotifyData *), \
    __stdcall void (*callback2)(int handle, void *context, unsigned int notifyEvent)

#   define OS_IF_IS_CLOSE_ERROR(x) (1 == x)
#   define OS_IF_CLOSE_HANDLE close
#   define OS_IF_ALLOC_MEM(s) malloc(s)
#   define OS_IF_FREE_MEM(a)  free(a)

#   define OS_IF_EXIT_THREAD(x) pthread_exit(NULL)

    typedef pthread_mutex_t OS_IF_MUTEX;
#   define OS_IF_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER


//#################################################
#else //WIN32

#   include "windows.h"

    typedef HANDLE OS_IF_FILE_HANDLE;
#   define DEVICE_NAME_LEN 32


#   define OS_IF_INVALID_HANDLE INVALID_HANDLE_VALUE

#   define OS_IF_SET_NOTIFY_PARAM  HWND hwnd

#   define OS_IF_IS_CLOSE_ERROR(x) (0 == x)
#   define OS_IF_CLOSE_HANDLE CloseHandle
#   define OS_IF_ALLOC_MEM(s) malloc(s)
#   define OS_IF_FREE_MEM(a)  free(a)

#   define OS_IF_EXIT_THREAD(x) ExitThread(x)
#   define errno GetLastError()

#   define snprintf _snprintf


    // qqq Add lock/unlock capability later.
    typedef int OS_IF_MUTEX;
#   define OS_IF_MUTEX_INITIALIZER 0


#endif
//#################################################


#endif //_OSIF_USER_H_
