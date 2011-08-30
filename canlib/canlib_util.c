/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*  Kvaser Linux Canlib LAPcan helper functions */

#include <canlib_util.h>







//====================================================================
// addFilter - Add a filter to a specific handle, type=pass
//====================================================================
int addFilter (OS_IF_FILE_HANDLE fd,
          unsigned int cmdNr,  int cmdNrMask,
          unsigned char chanNr, char chanNrMask,
          unsigned char flags,  char flagsMask,
          int isPass)
{
  FilterData filterData;

  filterData.cmdNr = cmdNr;
  filterData.cmdNrMask = cmdNrMask;
  filterData.chanNr = chanNr;
  filterData.chanNrMask = chanNrMask;
  filterData.flags = flags;
  filterData.flagsMask = flagsMask;
  filterData.isPass = isPass;

  return os_if_ioctl_write(fd, LAPCAN_IOC_ADD_FILTER, &filterData, sizeof(FilterData));
}


//====================================================================
// clearFilters - Remove all filters on a handle
//====================================================================
int clearFilters (OS_IF_FILE_HANDLE fd)
{
  return os_if_ioctl_write(fd, LAPCAN_IOC_CLEAR_FILTERS, NULL, 0);
}


//====================================================================
unsigned char chanMap [CMD__HIGH + 1] = \
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
 0, 0, 0, 0, 0, 0, 2, 3, 3, 5, \
 3, 3, 0, 0, 3, 0, 3, 3, 3, 3, \
 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, \
 0, 0, 3, 3, 0, 0, 0, 3, 0, 3, \
 3, 0, 3, 3, 3, 0, 0, 0, 3, 3, \
 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, \
 0, 0, 0, 0, 0, 0, 0, 3, 3, 2, \
 3, 3, 0, 0, 0, 3, 0, 0, 0, 3, \
 3, 3, 3};

unsigned char flagMap [CMD__HIGH + 1] = \
{2, 2, 0, 0, 0, 0, 0, 0, 0, 0, \
 0, 0, 0, 0, 0, 0, 0, 4, 6, 0, \
 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, \
 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, \
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \

 0, 0, 0, 6, 5, 0, 0, 0, 0, 0, \
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, \
 4, 0, 0};


//====================================================================
// getCmdNr
//====================================================================
OS_IF_INLINE int getCmdNr (lpcCmd *cmd)
{
  unsigned char *b;

  b = (unsigned char*) cmd;

  // Infrequent command:bit 7 is set
  if (b [0] & 0x80) {
    return b [1] & 0x7f;
  }
  else {
    return b [0] >> 5;
  }
}


//====================================================================
// getCmdLen
//====================================================================
OS_IF_INLINE int getCmdLen (lpcCmd *cmd)
{
  unsigned char *b;

  b = (unsigned char*) cmd;
  if (b [0] & 0x80) { // Infrequent command:bit 7 is set
    return 1 + (b [0] & 0x7f); // Length is lower 7 bits
  }
  else {
    return 1 + (b [0] & 0x1f); // Length is lower 5 bits
  }
}


//====================================================================
// getCmdChannel
//====================================================================
OS_IF_INLINE int getCmdChan (lpcCmd *cmd)
{
  unsigned char *b;
  b = (unsigned char*) cmd;
  if (b [0] & 0x80) { // Infrequent command:bit 7 is set
    return b [chanMap [getCmdNr(cmd)] ];
  }
  else {
    return b [1] >> 4; // channel is upper 4 bits
  }
}


//====================================================================
// getFlags
//====================================================================
OS_IF_INLINE unsigned char getFlags (lpcCmd *cmd)
{
  // flagMap[getCmdNr(cmd)] is the position of the flags in the command
  return ((unsigned char*) cmd) [flagMap [getCmdNr(cmd)] ];
  // OLD return ((unsigned char*) cmd) [2];
}


//====================================================================
// setFlags
//====================================================================
OS_IF_INLINE void setFlags (lpcCmd *cmd, unsigned char flags)
{
  ((unsigned char*) cmd) [flagMap [getCmdNr(cmd)] ] = flags;
  // OLD ((unsigned char*) cmd) [2] = flags;
  return;
}


//====================================================================
// copyLapcanCmd
//====================================================================
OS_IF_INLINE void copylpcCmd (lpcCmd *cmd_to, lpcCmd *cmd_from)
{
  int cmdLen;
  unsigned char *from, *to;

  from = (unsigned char*) cmd_from;
  to = (unsigned char*) cmd_to;

  for (cmdLen = getCmdLen(cmd_from); cmdLen > 0; cmdLen--) {
    *to++ = *from++;
  }
  return;
}

