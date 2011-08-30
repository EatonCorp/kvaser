/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

/* Kvaser Linux Canlib */

//********************************************
//  Simple linked list
//********************************************
#include "linkedlist.h"
#include "osif_common.h"


//======================================================================
// listInsertFirst
//======================================================================
int listInsertFirst (LinkedList **listPtrPtr, void *elem)
{
  LinkedList *newNode;

  newNode = malloc(sizeof(LinkedList));
  if (newNode == NULL)
    return -1;
  newNode->elem = elem;
  newNode->elem = elem;
  newNode->next = *listPtrPtr;
  *listPtrPtr = newNode;

  return 0;
}


//======================================================================
// listRemove
//======================================================================
void *listRemove (LinkedList **listPtrPtr, void *elem,
                  int (*compare)(const void *, const void *))
{
  void *removedNode;
  void *removedElem;

  while (*listPtrPtr != NULL) {
    if (compare((*listPtrPtr)->elem, elem)) {
      removedNode = *listPtrPtr;
      removedElem = (*listPtrPtr)->elem;
      *listPtrPtr = (*listPtrPtr)->next;
      free(removedNode);
      return removedElem;
    }
    listPtrPtr = &((*listPtrPtr)->next);
  }

  return NULL;
}



//======================================================================
// listFind
//======================================================================
void *listFind (LinkedList **listPtrPtr, void *elem,
                int (*compare)(const void *, const void *))
{
  while (*listPtrPtr != NULL){
    if (compare((*listPtrPtr)->elem, elem)) {
      return (*listPtrPtr)->elem;
    }
    listPtrPtr = &((*listPtrPtr)->next);
  }

  return NULL;
}


//======================================================================
// listSize
//======================================================================
OS_IF_INLINE int listSize (LinkedList **listPtrPtr)
{
  int n;
  for (n = 0; *listPtrPtr != NULL; listPtrPtr = &((*listPtrPtr)->next)) {
    n++;
  }

  return n;
}
