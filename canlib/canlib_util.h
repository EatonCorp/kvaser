/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*  Kvaser Linux Canlib LAPcan helper functions */

#ifndef _CANLIB_UTIL_H_
#define _CANLIB_UTIL_H_


#include <lapcmds.h>
#include <lapcan_ioctl.h>
#include <lapcan_data.h>
#include <canlib_data.h>

#include <osif_common.h>

#if LINUX
#   include <sys/ioctl.h>
#else //WIN32

#endif

int addFilter (OS_IF_FILE_HANDLE fd,
           unsigned int cmdNr,  int cmdNrMask,
           unsigned char chanNr, char chanNrMask,
           unsigned char flags,  char flagsMask,
           int isPass);
int clearFilters (OS_IF_FILE_HANDLE fd);

OS_IF_INLINE int getCmdNr (lpcCmd *cmd);
OS_IF_INLINE int getCmdLen (lpcCmd *cmd);
OS_IF_INLINE int getCmdChan (lpcCmd *cmd);
OS_IF_INLINE unsigned char getFlags (lpcCmd *cmd);
OS_IF_INLINE void setFlags (lpcCmd *cmd, unsigned char flags);
OS_IF_INLINE void copyLapcanCmd (lpcCmd *cmd_to, lpcCmd *cmd_from);

#endif /* CANLIB_UTIL_H */
