/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/*  Kvaser Linux Canlib LAPcan helper functions */

#include <malloc.h>

#ifndef _LINKEDLIST_H_
#define _LINKEDLIST_H_

typedef struct ListNode {
  void *elem;
  struct ListNode *next;
} LinkedList;

int listInsertFirst(LinkedList **listPtrPtr, void *elem);
void *listRemove(LinkedList **listPtrPtr, void *elem,
                 int (*compare)(const void *, const void *));
int listSize(LinkedList **listPtrPtr);
void *listFind(LinkedList **listPtrPtr, void *elem,
               int (*compare)(const void *, const void *));

#endif /*_LINKEDLIST_H_ */
