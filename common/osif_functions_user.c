/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

#include "osif_functions_user.h"

#if LINUX
#   include <sys/ioctl.h>
#   include <unistd.h>
#   include <sys/io.h>
#else
#   include <windows.h>

// To deal with os_if_ioctl_write that wants to return data, a temporary
// buffer is used. This is allocated on the stack for speed.
// If the size below is not enough, memory will be allocated and freed.
// The size should be enough for all ioctl calls.
#define IOCTL_TEMPBUF_SIZE 32
#endif


// Common
#include <stdio.h>





//////////////////////////////////////////////////////////////////////
// os_if_ioctl_read
// Let subsystem fill out the buffer
// IOCTL
//////////////////////////////////////////////////////////////////////
int os_if_ioctl_read (OS_IF_FILE_HANDLE fd,
                      unsigned int      ioctl_code,
                      void              *out_buffer,
                      unsigned int      out_bufsize)
{
#if LINUX
  return ioctl(fd, ioctl_code, out_buffer);
#else
  BOOL ret;
  unsigned int ret_bytes = 0;

  ret = DeviceIoControl(fd, ioctl_code, NULL, 0, out_buffer, out_bufsize,
                        &ret_bytes, NULL);
  if (ret == FALSE)
    return -1;
  else
    return 0;
#endif
}


//////////////////////////////////////////////////////////////////////
// os_if_ioctl_write
// Send something to the subsystem
// IOCTL
//////////////////////////////////////////////////////////////////////
int os_if_ioctl_write (OS_IF_FILE_HANDLE fd,
                       unsigned int      ioctl_code,
                       void              *in_buffer,
                       unsigned int      in_bufsize)
{
#if LINUX
  return ioctl(fd, ioctl_code, in_buffer);
#else
  BOOL ret;
  unsigned int ret_bytes = 0;
  char buffer[IOCTL_TEMPBUF_SIZE], *out_buffer;

  if (in_bufsize <= sizeof(buffer)) {
    out_buffer = buffer;
  } else {
    out_buffer = LocalAlloc(0, in_bufsize);
  }

  // While it might have been possible to specify out_buffer as the
  // same as in_buffer, that sort of goes against the idea of having
  // separate buffers for the call.
  ret = DeviceIoControl(fd, ioctl_code, in_buffer, in_bufsize,
                        out_buffer, in_bufsize,
                        &ret_bytes, NULL);
  if (ret == TRUE) {
    if (ret_bytes) {
      memcpy(in_buffer, out_buffer,
             (ret_bytes < in_bufsize) ? ret_bytes : in_bufsize);
    }

    if (out_buffer != buffer) {
      LocalFree(out_buffer);
    }
    return 0;
  }

  return -1;
#endif
}


//////////////////////////////////////////////////////////////////////
// os_if_access
//
//////////////////////////////////////////////////////////////////////

int os_if_access(char *fileName, int code)
{
  if (code != F_OK)
    return -1;
  else {
#if LINUX
    return access(fileName, code);
#else
    HANDLE fd;
    WCHAR tmpFileName[MAX_PATH];

    // qqq will this work?
    wsprintf(tmpFileName, L"%S", fileName);
    fd = CreateFile(tmpFileName, GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (fd != INVALID_HANDLE_VALUE) {
      CloseHandle(fd);
      return 0;
    }
    else
      return -2;
#endif
  }
}




















#if LINUX


/*
// In linux the outbuffer is ignored
int OS_IF_IOCTL(OS_IF_FILE_HANDLE fd, unsigned int ioctl_code, void *in_buffer,
                unsigned int in_bufsize, void *out_buffer, unsigned int out_bufsize)
{
  return ioctl(fd, ioctl_code, in_buffer);
}


#else

int OS_IF_IOCTL(OS_IF_FILE_HANDLE fd, unsigned int ioctl_code, void *in_buffer,
                unsigned int in_bufsize, void *out_buffer, unsigned int out_bufsize)
{
  BOOL ret;
  unsigned int ret_bytes;

  ret = DeviceIoControl(fd, ioctl_code, in_buffer, in_bufsize,
                        out_buffer, out_bufsize, &ret_bytes, NULL);
  if (ret == FALSE)
    return -1;
  else
    return ret_bytes;
}
*/

#endif
