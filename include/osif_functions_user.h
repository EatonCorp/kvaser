#ifndef OSIF_FUNCTIONS_USER_H_
#define OSIF_FUNCTIONS_USER_H_

/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

#include "osif_user.h"


///////////////////////////////////////////////////////////
//
int os_if_ioctl_read(OS_IF_FILE_HANDLE  fd,
                     unsigned int       ioctl_code,
                     void               *in_buffer,
                     unsigned int       in_bufsize);

///////////////////////////////////////////////////////////
//
int os_if_ioctl_write(OS_IF_FILE_HANDLE  fd,
                      unsigned int       ioctl_code,
                      void               *out_buffer,
                      unsigned int       out_bufsize);


#if LINUX
#else
#   define F_OK 99
#endif
int os_if_access(char *fileName, int code);





#endif //OSIF_FUNCTIONS_USER_H_
