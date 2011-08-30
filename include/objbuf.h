#ifndef OBJBUF_H
#define OBJBUF_H

#include "vcanevt.h"

#define MAX_OBJECT_BUFFERS 8

// Object buffer types.
#define OBJBUF_TYPE_AUTO_RESPONSE       1
#define OBJBUF_TYPE_PERIODIC_TX         2

#define OBJBUF_AUTO_RESPONSE_RTR_ONLY   0x01    // Flag: respond to RTR's only

#define OBJBUF_DRIVER_MARKER 0x40000000  // To make handles different
#define OBJBUF_DRIVER_MASK   0x1f        // Support up to 32 object buffers

typedef struct {
  unsigned int acc_code;    // For autoresponse bufs; filter code
  unsigned int acc_mask;    // For autoresponse bufs; filter mask
  unsigned int period;      // For auto tx buffers; interval in microseconds
  CAN_MSG msg;
  unsigned char in_use;
  unsigned char active;
  unsigned char type;
  unsigned char flags;
} OBJECT_BUFFER;

struct VCanOpenFileNode;

unsigned int objbuf_filter_match(OBJECT_BUFFER *buf, unsigned int id,
                                 unsigned int flags);
void objbuf_init(struct VCanOpenFileNode *fileNodePtr);
void objbuf_shutdown(struct VCanOpenFileNode *fileNodePtr);

#endif
