#ifndef OSIF_KERNEL_H_
#define OSIF_KERNEL_H_

/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

#if LINUX
//###############################################################

#   include <linux/types.h>
#   include <linux/tty.h>
#   include <linux/module.h>
#   include <linux/spinlock.h>
#   if LINUX_2_6
#       include <linux/workqueue.h>
#   else
#       include <linux/tqueue.h>
#       include <pcmcia/driver_ops.h>
#       include <pcmcia/bus_ops.h>
#   endif

// DEFINES
#   define OS_IF_TIMEOUT 0
#   define OS_IF_TICK_COUNT jiffies
#if LINUX_2_6
#   define OS_IF_MODE_INC_USE_COUNT ;
#   define OS_IF_MODE_DEC_USE_COUNT ;
#else
#   define OS_IF_MODE_INC_USE_COUNT MOD_INC_USE_COUNT;
#   define OS_IF_MODE_DEC_USE_COUNT MOD_DEC_USE_COUNT;
#   define IRQ_NONE
#   define IRQ_HANDLED
#   define IRQ_RETVAL(x)

#endif

    // TYPEDEFS
    typedef struct timeval        OS_IF_TIME_VAL;
    typedef off_t                 OS_IF_OFFSET;
    typedef unsigned long         OS_IF_SIZE;
#if LINUX_2_6
    typedef struct task_struct*   OS_IF_THREAD;
#else
    typedef int                   OS_IF_THREAD;
#endif
    typedef wait_queue_head_t     OS_IF_WAITQUEUE_HEAD;
    typedef wait_queue_t          OS_IF_WAITQUEUE;
    typedef spinlock_t            OS_IF_LOCK;

    #if LINUX_2_6
       typedef struct work_struct      OS_IF_TASK_QUEUE_HANDLE;
       typedef struct workqueue_struct OS_IF_WQUEUE;
       typedef struct completion       OS_IF_SEMAPHORE;
    #else
       typedef struct tq_struct        OS_IF_TASK_QUEUE_HANDLE;
       typedef int                     OS_IF_WQUEUE;
       typedef struct semaphore        OS_IF_SEMAPHORE;
    #endif




#else // end LINUX

//###############################################################
// win32 defines

    // INCLUDES
#   include <windows.h>
#   include <usbtypes.h>

#if 0
#   include "transfer.h"
#else
#   define USB_ERROR DWORD
#   define PUSB_ERROR LPDWORD
#endif

    // DEFINES
#   define OS_IF_TIMEOUT WAIT_TIMEOUT
#   define HZ 1000
#   define OS_IF_TICK_COUNT GetTickCount()
#   define SPIN_LOCK_UNLOCKED {0}    // Just a dummy value

    // TYPEDEFS
    typedef CRITICAL_SECTION    OS_IF_LOCK;

    typedef struct timeval {
        long tv_sec; /* seconds */
        long tv_usec; /* microseconds */
    };
    typedef struct timeval      OS_IF_TIME_VAL;
    typedef long off_t;
    typedef off_t               OS_IF_OFFSET;
    typedef char*               OS_IF_DEVICE_CONTEXT_NODE;
    typedef HANDLE              OS_IF_SEMAPHORE;
    typedef HANDLE              OS_IF_COUNTSEM;
    typedef size_t              OS_IF_SIZE;
    typedef HANDLE              OS_IF_THREAD;
    typedef HANDLE              OS_IF_EVENT;
    typedef struct {
      OS_IF_THREAD thread;
      struct task_info {
        void (*routine)(void *context);
        void *data;
        OS_IF_COUNTSEM work;
        OS_IF_EVENT finished;
        int shutdown;
      } info;
    }                           OS_IF_TASK_QUEUE_HANDLE;
    typedef OS_IF_TASK_QUEUE_HANDLE     OS_IF_WQUEUE;
    typedef HANDLE              OS_IF_WAITQUEUE_HEAD;
    typedef HANDLE              OS_IF_WAITQUEUE;
    typedef CRITICAL_SECTION    rwlock_t;
    typedef unsigned char       __u8;
    struct urb {
      void *transfer_buffer;
      int transfer_buffer_length;
      void *context;
      USB_ERROR status;
      USB_PIPE pipe;
      USB_TRANSFER transfer_handle;
      DWORD transfer_flags;
    };

#define inline __inline
    // These are borrowed from Linux
    typedef struct { long counter; } atomic_t;
#define atomic_set(v,i)         (((v)->counter) = (i))
#define atomic_read(v)          ((v)->counter)
#define atomic_inc(v)           (void)InterlockedIncrement(&((v)->counter))
#define atomic_dec(v)           (void)InterlockedDecrement(&((v)->counter))
#define atomic_dec_and_test(v)  (InterlockedDecrement(&((v)->counter)) == 0)
struct list_head {
        struct list_head *next, *prev;
};
#define INIT_LIST_HEAD(ptr) do {             \
  (ptr)->next = (ptr); (ptr)->prev = (ptr);  \
} while (0)
static inline void __list_add(struct list_head *new,
                              struct list_head *prev,
                              struct list_head *next)
{
  next->prev = new;
  new->next  = next;
  new->prev  = prev;
  prev->next = new;
}
static inline void list_add(struct list_head *new, struct list_head *head)
{
  __list_add(new, head, head->next);
}
static inline void __list_del(struct list_head *prev, struct list_head *next)
{
  next->prev = prev;
  prev->next = next;
}
static inline void list_del(struct list_head *entry)
{
  __list_del(entry->prev, entry->next);
  entry->next = 0;
  entry->prev = 0;
}

//    typedef CARD_EVENT          OS_IF_EVENT;

#if defined(BREAKPOINTS)
extern short debug_break;
#include <Kfuncs.h>
#  define BREAK(v,t)  do {                                           \
                        if ((debug_break & (v)) || ((v) == 0xffff))  \
                          DebugBreak();                              \
                      } while(0)
#else
#  define BREAK(v,t)  do { } while(0)
#endif

#endif // win32


#endif //OSIF_KERNEL_H_
