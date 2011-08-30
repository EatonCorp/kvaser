/*
** Copyright 2002-2009 KVASER AB, Sweden.  All rights reserved.
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

typedef struct {
  unsigned int buffer_number;
  unsigned int acc_code;
  unsigned int acc_mask;
  unsigned int flags;
  unsigned int type;
  unsigned int period;
//  unsigned int reserved[16];      // For future usage
} KCanObjbufAdminData;


typedef struct {
  unsigned int buffer_number;
  unsigned int id;
  unsigned int dlc;
  unsigned char data[8];
  unsigned int flags;
} KCanObjbufBufferData;

#endif /* _CANIF_DATA_H_ */
