/*
** Copyright 2006 KVASER AB, Sweden.  All rights reserved.
*/

// FIFO
// This abstraction currently only keeps track of indices into a queue,
// the actual data needs to be kept elsewhere.
//
// head points to next place to put new data (back/push)
// tail points to the oldest data (front/pop)
// If they are equal, the queue is empty, so
// only size-1 elements fit.
//
// Without USE_LOCKS defined, the behaviour is
// exactly the same as the old code.

#ifndef QUEUE_H
#define QUEUE_H

#include "osif_functions_kernel.h"
#include "osif_kernel.h"


typedef enum {Normal_lock, Softirq_lock, Irq_lock} Lock_type;
typedef struct {
  int size;
  int head;
  int tail;
  atomic_t length;      // For length queries without locking
  unsigned int flags;   // Only used when holding the lock (Sparc incompatible)!
  OS_IF_WAITQUEUE_HEAD space_event;
  Lock_type lock_type;
  OS_IF_LOCK lock;
  int locked;           // For debugging
  int line;             // For debugging
} Queue;


extern void queue_reinit(Queue *queue);
extern void queue_init(Queue *queue, int size);
extern void queue_irq_lock(Queue *queue);
extern int  queue_length(Queue *queue);
extern int  queue_full(Queue *queue);
extern int  queue_empty(Queue *queue);

// queue_back/front _must_ always be paired with queue_push/pop or _release.
// The first two grab the Queue lock and the last three release it again.
// Make _sure_ not to sleep inbetween and do as little work as possible
// (the interrupts are disabled while holding the lock).
extern int  queue_back(Queue *queue);
extern void queue_push(Queue *queue);
extern int  queue_front(Queue *queue);
extern void queue_pop(Queue *queue);
extern void queue_release(Queue *queue);

extern void queue_add_wait_for_space(Queue *queue, OS_IF_WAITQUEUE *waiter);
extern void queue_remove_wait_for_space(Queue *queue, OS_IF_WAITQUEUE *waiter);
extern void queue_add_wait_for_data(Queue *queue, OS_IF_WAITQUEUE *waiter);
extern void queue_wakeup_on_space(Queue *queue);
extern void queue_wakeup_on_data(Queue *queue);
extern OS_IF_WAITQUEUE_HEAD *queue_space_event(Queue *queue);

#endif
