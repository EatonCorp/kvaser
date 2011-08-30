/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

#ifndef _CANIF_DATA_H_
#define _CANIF_DATA_H_

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

typedef struct CanIfStat
{
    unsigned int   overruns;
    unsigned short statSize;
    unsigned short sendQL;
    unsigned short rcvQL;
} CanIfStat;


typedef struct {
    signed long freq;
    unsigned char sjw;
    unsigned char tseg1;
    unsigned char tseg2;
    unsigned char samp3;
} VCanBusParams;

typedef struct {
    unsigned char eventMask;
  //unsigned char msgFlags;
  //unsigned char flagsMask;
    unsigned long stdId;
    unsigned long stdMask;
    unsigned long extId;
    unsigned long extMask;
    unsigned char typeBlock;
} VCanMsgFilter;

#endif /* _CANIF_DATA_H_ */
