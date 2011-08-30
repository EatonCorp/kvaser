#ifndef OSIF_FUNCTIONS_USER_H_
#define OSIF_FUNCTIONS_USER_H_

/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

#include <string.h>      // For size_t only
#include "osif_user.h"


///////////////////////////////////////////////////////////
//
int os_if_ioctl_read(OS_IF_FILE_HANDLE  fd,
                     unsigned int       ioctl_code,
                     void               *in_buffer,
                     size_t             in_bufsize);

///////////////////////////////////////////////////////////
//
int os_if_ioctl_write(OS_IF_FILE_HANDLE  fd,
                      unsigned int       ioctl_code,
                      void               *out_buffer,
                      size_t             out_bufsize);


void os_if_mutex_lock(OS_IF_MUTEX *mutex);
void os_if_mutex_unlock(OS_IF_MUTEX *mutex);

#if LINUX
#else
#   define F_OK 99
#endif
OS_IF_FILE_HANDLE os_if_open(char *fileName);
int os_if_access(char *fileName, int code);





#endif //OSIF_FUNCTIONS_USER_H_
