/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*
** File:
**   lapcan_data.h
** Project:
**   Lapcan Linux
** Desciption:
**   Some lapcan datastructures that need to be shared with canlib
*/

#ifndef _LAPCAN_DATA_H_
#define _LAPCAN_DATA_H_

#include <lapcmds.h>

/* Used as interface datatype */
typedef struct FilterData
{
  unsigned int cmdNr;
  int cmdNrMask;
  unsigned char chanNr;
  char chanNrMask;
  unsigned char flags;
  char flagsMask;
  unsigned char isPass;
} FilterData;

typedef struct LapcanStat
{
  unsigned int overruns;
  unsigned short statSize;
  unsigned short sendQL;
  unsigned short rcvQL;
} LapcanStat;


#endif








