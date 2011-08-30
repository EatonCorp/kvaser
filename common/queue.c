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

//--------------------------------------------------
// NOTE! module_versioning HAVE to be included first
#include "module_versioning.h"
//--------------------------------------------------

#include "osif_functions_kernel.h"
#include "osif_kernel.h"
#include "VCanOsIf.h"
#include "queue.h"


// Without this defined, all length queries will use locking
#define ATOMIC_LENGTH

// Lock type to use for queue
#define LOCK_TYPE Softirq_lock


// Some lock debug macros

#define PRINT(t)
//#define PRINT(t) DEBUGOUT(1,t)
//#define PRINTX(t)
#define PRINTX(t) DEBUGOUT(1,t)

#if 0
#define QUEUE_DEBUG				       \
  if (!queue->size) {                                  \
    PRINT((TXT("Using unitialized queue\n")));         \
    return;                                            \
  }
#endif
#ifdef QUEUE_DEBUG
 #include "debug.h"
 #define QUEUE_DEBUG_RET(ret)                          \
   if (!queue->size) {                                 \
     PRINT((TXT("Using unitialized queue\n")));        \
     return ret;                                       \
   }

 #define QUEUE_DEBUG_LOCK                              \
   if (queue->locked) {                                \
     PRINTX((TXT("Queue already locked (%d/%d)\n"),    \
             __LINE__, queue->line));                  \
     return;                                           \
   } else {                                            \
     queue->line = __LINE__;                           \
   }
 #define QUEUE_DEBUG_LOCK_RET(ret)                     \
   if (queue->locked) {                                \
     PRINTX((TXT("Queue already locked (%d/%d)\n"),    \
             __LINE__, queue->line));                  \
     return ret;                                       \
   } else {                                            \
     queue->line = __LINE__;                           \
   }

 #define QUEUE_DEBUG_UNLOCK                            \
   if (!queue->locked) {                               \
     PRINTX((TXT("Queue already unlocked (%d)\n"),     \
             __LINE__));                               \
     return;                                           \
   }
 #define QUEUE_DEBUG_UNLOCK_RET(ret)                   \
   if (!queue->locked) {                               \
     PRINTX((TXT("Queue already unlocked (%d)\n"),     \
             __LINE__));                               \
     return ret;                                       \
   }
#else
 #define QUEUE_DEBUG
 #define QUEUE_DEBUG_RET(ret)
 #define QUEUE_DEBUG_LOCK
 #define QUEUE_DEBUG_LOCK_RET(ret)
 #define QUEUE_DEBUG_UNLOCK
 #define QUEUE_DEBUG_UNLOCK_RET(ret)
#endif


#define LOCKQ(queue, flags_ptr)                         \
  PRINT((TXT("Locking (%d) at %d\n"),                   \
         queue->lock_type, __LINE__));                  \
  switch (queue->lock_type) {                           \
  case Irq_lock:                                        \
    os_if_spin_lock_irqsave(&queue->lock, flags_ptr);   \
    break;                                              \
  case Softirq_lock:                                    \
    os_if_spin_lock_softirq(&queue->lock);              \
    break;                                              \
  default:                                              \
    os_if_spin_lock(&queue->lock);                      \
    break;                                              \
  }                                                     \
  queue->locked = 1;   /* After acquiring */

#define UNLOCKQ(queue, flags)                           \
  PRINT((TXT("Unlocking (%d) at %d\n"),                 \
         queue->lock_type, __LINE__));                  \
  queue->locked = 0;   /* Before releasing */           \
  switch (queue->lock_type) {                           \
  case Irq_lock:                                        \
    os_if_spin_unlock_irqrestore(&queue->lock, flags);  \
    break;                                              \
  case Softirq_lock:                                    \
    os_if_spin_unlock_softirq(&queue->lock);            \
    break;                                              \
  default:                                              \
    os_if_spin_lock(&queue->lock);                      \
    break;                                              \
  }


void queue_reinit(Queue *queue)
{
  unsigned long flags;

  QUEUE_DEBUG;
  QUEUE_DEBUG_LOCK;
  LOCKQ(queue, &flags);

  queue->head = 0;
  queue->tail = 0;

  atomic_set(&queue->length, 0);

  QUEUE_DEBUG_UNLOCK;
  UNLOCKQ(queue, flags);
}

void queue_init(Queue *queue, int size)
{
  queue->lock_type = LOCK_TYPE;
  os_if_spin_lock_init(&queue->lock);
  queue->size = size;
  os_if_init_waitqueue_head(&queue->space_event);
  queue->locked = 0;
  queue_reinit(queue);
}

int queue_length(Queue *queue)
{
  int length;
#ifndef ATOMIC_LENGTH
  unsigned long flags;

  QUEUE_DEBUG_RET(0);
  QUEUE_DEBUG_LOCK_RET(0);
  LOCKQ(queue, &flags);

  length = queue->head - queue->tail;
  if (length < 0)
    length += queue->size;

  QUEUE_DEBUG_UNLOCK_RET(0);
  UNLOCKQ(queue, flags);
#else
  length = atomic_read(&queue->length);
#endif

  return length;
}

int queue_full(Queue *queue)
{
  QUEUE_DEBUG_RET(0);

  return queue_length(queue) >= queue->size - 1;
}

int queue_empty(Queue *queue)
{
  QUEUE_DEBUG_RET(1);

  return queue_length(queue) == 0;
}

// Lock will be held when this returns.
// Must be released with a call to queue_push/release()
// as soon as possible. Make _sure_ not to sleep inbetween!
// (Storing irq flags like this is supposedly incompatible
//  with Sparc CPU:s. But that only applies for Irq_lock queues.)
int queue_back(Queue *queue)
{
  int back;
  unsigned long flags;

  QUEUE_DEBUG_RET(0);
  QUEUE_DEBUG_LOCK_RET(0);
  LOCKQ(queue, &flags);

  back = queue->head;
  // Is there actually any space in the queue?
#ifndef ATOMIC_LENGTH
  // (Holding lock, so can't use queue_full.)
  {
    int length = back - queue->tail;
    if (length < 0)
      length += queue->size;
    if (length >= queue->size - 1) {
      back = -1;
    }
  }
#else
  if (queue_full(queue)) {
    back = -1;
  }
#endif

  queue->flags = flags;

  return back;
}

// Lock must be held from a previous queue_back().
void queue_push(Queue *queue)
{
  QUEUE_DEBUG;

  queue->head++;
  if (queue->head >= queue->size)
    queue->head = 0;

  atomic_inc(&queue->length);

  QUEUE_DEBUG_UNLOCK;
  UNLOCKQ(queue, queue->flags);
}

// Lock will be held when this returns.
// Must be released with a call to queue_pop/release()
// as soon as possible. Make _sure_ not to sleep inbetween!
int queue_front(Queue *queue)
{
  int front;
  unsigned long flags;

  QUEUE_DEBUG_RET(0);
  QUEUE_DEBUG_LOCK_RET(0);
  LOCKQ(queue, &flags);

  front = queue->tail;
  // Is there actually anything in the queue?
#ifndef ATOMIC_LENGTH
  // (Holding lock, so can't use queue_empty.)
  if (queue->head == front) {
    front = -1;
  }
#else
  if (queue_empty(queue)) {
    front = -1;
  }
#endif

  queue->flags = flags;

  return front;
}

// Lock must be held from a previous queue_front().
void queue_pop(Queue *queue)
{
  QUEUE_DEBUG;

  queue->tail++;
  if (queue->tail >= queue->size)
    queue->tail = 0;  

  atomic_dec(&queue->length);

  QUEUE_DEBUG_UNLOCK;
  UNLOCKQ(queue, queue->flags);
}

// Lock must be held from a previous queue_front/back().
void queue_release(Queue *queue)
{
  QUEUE_DEBUG;

  QUEUE_DEBUG_UNLOCK;
  UNLOCKQ(queue, queue->flags);
}


void queue_add_wait_for_space(Queue *queue, OS_IF_WAITQUEUE *waiter)
{
  QUEUE_DEBUG;

  os_if_add_wait_queue(&queue->space_event, waiter);
}

void queue_remove_wait_for_space(Queue *queue, OS_IF_WAITQUEUE *waiter)
{
  QUEUE_DEBUG;

  os_if_remove_wait_queue(&queue->space_event, waiter);
}

#if 0
void queue_add_wait_for_data(Queue *queue, OS_IF_WAITQUEUE *waiter)
{
  QUEUE_DEBUG;

  os_if_add_wait_queue(&queue->wait_data, waiter);
}
#endif

void queue_wakeup_on_space(Queue *queue)
{
  QUEUE_DEBUG;

  os_if_wake_up_interruptible(&queue->space_event);
}

OS_IF_WAITQUEUE_HEAD *queue_space_event(Queue *queue)
{
  QUEUE_DEBUG_RET(0);

  return &queue->space_event;
}
