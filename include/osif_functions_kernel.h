#ifndef OSIF_FUNCTIONS_KERNEL_H_
#define OSIF_FUNCTIONS_KERNEL_H_

/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

#include "osif_kernel.h"


//////////////////////////////////////////////////////////////////////
//
#if LINUX_2_6
#define os_if_wait_event_interruptible_timeout(wq, condition, timeout)  \
  wait_event_interruptible_timeout(wq, condition, msecs_to_jiffies(timeout) + 1)
#elif LINUX
#define __os_if_wait_event_interruptible_timeout(wq, condition, ret)    \
do {                                                                    \
  wait_queue_t __wait;                                                  \
  init_waitqueue_entry(&__wait, current);                               \
                                                                        \
  add_wait_queue(&wq, &__wait);                                         \
  for (;;) {                                                            \
    set_current_state(TASK_INTERRUPTIBLE);                              \
    if (condition) {                                                    \
      break;                                                            \
    }                                                                   \
    if (!signal_pending(current)) {                                     \
      ret = schedule_timeout(ret);                                      \
      if (!ret) {                                                       \
        break;                                                          \
      }                                                                 \
      continue;                                                         \
    }                                                                   \
    ret = -ERESTARTSYS;                                                 \
    break;                                                              \
  }                                                                     \
  current->state = TASK_RUNNING;                                        \
  remove_wait_queue(&wq, &__wait);                                      \
} while (0)


#define os_if_wait_event_interruptible_timeout(wq, condition, timeout)  \
({                                                                      \
  long __ret = msecs_to_jiffies(timeout) + 1;                           \
  if (!(condition)) {                                                   \
    __os_if_wait_event_interruptible_timeout(wq, condition, __ret);     \
  }                                                                     \
  __ret;                                                                \
})
#else
int os_if_wait_event_interruptible_timeout(OS_IF_WAITQUEUE_HEAD *wq,
                                           int (*eq)(void *),
                                           void *par, int timeout);
#endif

//////////////////////////////////////////////////////////////////////
//
void os_if_write_port(unsigned regist, unsigned portAddr);

//////////////////////////////////////////////////////////////////////
//
unsigned int os_if_read_port(unsigned portAddr);

//////////////////////////////////////////////////////////////////////
//
int os_if_queue_task(OS_IF_TASK_QUEUE_HANDLE *hnd);

//////////////////////////////////////////////////////////////////////
//
int os_if_queue_task_not_default_queue(OS_IF_WQUEUE *wq,
                                       OS_IF_TASK_QUEUE_HANDLE *hnd);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_waitqueue_head(OS_IF_WAITQUEUE_HEAD *handle);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_named_waitqueue_head(OS_IF_WAITQUEUE_HEAD *handle, char *name);

//////////////////////////////////////////////////////////////////////
//
void os_if_delete_waitqueue_head(OS_IF_WAITQUEUE_HEAD *handle);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_waitqueue_entry(OS_IF_WAITQUEUE *wait);

//////////////////////////////////////////////////////////////////////
//
void os_if_add_wait_queue(OS_IF_WAITQUEUE_HEAD *waitQ,
                          OS_IF_WAITQUEUE *wait);

//////////////////////////////////////////////////////////////////////
//
void os_if_remove_wait_queue(OS_IF_WAITQUEUE_HEAD *waitQ,
                             OS_IF_WAITQUEUE *wait);

//////////////////////////////////////////////////////////////////////
//
signed long os_if_wait_for_event_timeout(signed long timeout,
                                         OS_IF_WAITQUEUE *handle);

//////////////////////////////////////////////////////////////////////
//
signed long os_if_wait_for_event_timeout_simple(signed long timeout);

//////////////////////////////////////////////////////////////////////
//
void os_if_wait_for_event(OS_IF_WAITQUEUE_HEAD *handle);

//////////////////////////////////////////////////////////////////////
//
//long os_if_wait_event_interruptible_timeout(OS_IF_WAITQUEUE_HEAD handle,
//                                            unsigned long *cond, long time);

//////////////////////////////////////////////////////////////////////
//
void os_if_wake_up_interruptible(OS_IF_WAITQUEUE_HEAD *handle);

//////////////////////////////////////////////////////////////////////
//
#if !LINUX
void os_if_mark_event (OS_IF_WAITQUEUE_HEAD *handle);
#endif

//////////////////////////////////////////////////////////////////////
//
#if !LINUX
void os_if_clear_event (OS_IF_WAITQUEUE_HEAD *handle);
#endif


//////////////////////////////////////////////////////////////////////
//
void os_if_up_sema(OS_IF_SEMAPHORE *var);

//////////////////////////////////////////////////////////////////////
//
void os_if_down_sema(OS_IF_SEMAPHORE *var);

//////////////////////////////////////////////////////////////////////
//
int  os_if_down_sema_time(OS_IF_SEMAPHORE *var, int timeout);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_sema(OS_IF_SEMAPHORE *var);

//////////////////////////////////////////////////////////////////////
//
void os_if_delete_sema(OS_IF_SEMAPHORE *var);

#if !LINUX
//////////////////////////////////////////////////////////////////////
//
void os_if_up_countsem(OS_IF_COUNTSEM *sem, int count);

//////////////////////////////////////////////////////////////////////
//
void os_if_down_countsem(OS_IF_COUNTSEM *sem);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_countsem(OS_IF_COUNTSEM *sem, int count, int max);

//////////////////////////////////////////////////////////////////////
//
void os_if_delete_countsem(OS_IF_COUNTSEM *sem);
#endif

//////////////////////////////////////////////////////////////////////
//
void os_if_set_task_interruptible(void);

//////////////////////////////////////////////////////////////////////
//
void os_if_set_task_uninterruptible(void);

//////////////////////////////////////////////////////////////////////
//
void os_if_set_task_running(void);

//////////////////////////////////////////////////////////////////////
//
unsigned long os_if_get_timeout_time(void);

//////////////////////////////////////////////////////////////////////
//
void os_if_do_get_time_of_day(OS_IF_TIME_VAL *tv);

//////////////////////////////////////////////////////////////////////
//
int os_if_is_rec_busy(int nr, volatile unsigned long *addr);

//////////////////////////////////////////////////////////////////////
//
void os_if_rec_not_busy(int nr, volatile unsigned long *addr);

//////////////////////////////////////////////////////////////////////
//
#if !LINUX
void os_if_spin_lock_init(OS_IF_LOCK *lock);
#else
# define os_if_spin_lock_init(lock) spin_lock_init(lock)
#endif

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_lock(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_unlock(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_lock_remove(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_irq_disable(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_irq_enable(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_irq_save(OS_IF_LOCK *lock, unsigned long *flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_irq_restore(OS_IF_LOCK *lock, unsigned long flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_lock_irqsave(OS_IF_LOCK *lock, unsigned long *flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_unlock_irqrestore(OS_IF_LOCK *lock, unsigned long flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_lock_softirq(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_spin_unlock_softirq(OS_IF_LOCK *lock);

//////////////////////////////////////////////////////////////////////
//
int os_if_get_user_data(void *to, const void *from, OS_IF_SIZE n);

//////////////////////////////////////////////////////////////////////
//
int os_if_set_user_data(void *to, const void *from, OS_IF_SIZE n);

//////////////////////////////////////////////////////////////////////
//
int os_if_set_int(int val, int *dest);

//////////////////////////////////////////////////////////////////////
//
int os_if_get_int(int *val, int *src);

//////////////////////////////////////////////////////////////////////
//
int os_if_get_long(long *val, long *src);

//////////////////////////////////////////////////////////////////////
//
OS_IF_WQUEUE* os_if_declare_task(char *name, OS_IF_TASK_QUEUE_HANDLE *taskQ);

//////////////////////////////////////////////////////////////////////
//
OS_IF_WQUEUE* os_if_declare_rt_task(char *name, OS_IF_TASK_QUEUE_HANDLE *taskQ);

//////////////////////////////////////////////////////////////////////
//
void os_if_destroy_task(OS_IF_WQUEUE *wQueue);

//////////////////////////////////////////////////////////////////////
//
void os_if_init_task(OS_IF_TASK_QUEUE_HANDLE *taskQ, void *function, void *data);

//////////////////////////////////////////////////////////////////////
//
void os_if_rwlock_init(rwlock_t *lock);

//////////////////////////////////////////////////////////////////////
//
void os_if_read_lock_irqsave(rwlock_t *rw_lock, unsigned long *flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_read_unlock_irqrestore(rwlock_t *rw_lock, unsigned long flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_write_lock_irqsave(rwlock_t *rw_lock, unsigned long *flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_write_unlock_irqrestore(rwlock_t *rw_lock, unsigned long flags);

//////////////////////////////////////////////////////////////////////
//
void os_if_rwlock_remove(rwlock_t *lock);

//////////////////////////////////////////////////////////////////////
//
int os_if_signal_pending(void);

//////////////////////////////////////////////////////////////////////
//
void* os_if_kernel_malloc(size_t buffer_size);

//////////////////////////////////////////////////////////////////////
//
void os_if_kernel_free(void *mem_ptr);

//////////////////////////////////////////////////////////////////////
//
#if LINUX
#  if LINUX_2_6
OS_IF_THREAD os_if_kernel_thread(int (*thread)(void *context), void *context);
#  else
// Only used in usbcanII driver (kernel 2.6)
#  endif
#else
OS_IF_THREAD os_if_kernel_thread(DWORD (*thread)(LPVOID context),
                                 LPVOID context);
#endif

//////////////////////////////////////////////////////////////////////
//
void os_if_exit_thread(int result);

#if !LINUX
//////////////////////////////////////////////////////////////////////
//
void os_if_remove_thread(HANDLE thread);

void os_if_set_cond(OS_IF_EVENT *event);
void os_if_clear_cond(OS_IF_EVENT *event);
void os_if_wait_for_cond(OS_IF_EVENT *event);
void os_if_init_cond(OS_IF_EVENT *sem);
void os_if_delete_cond(OS_IF_EVENT *event);
#endif

#if LINUX
  typedef unsigned long AtomicBit;
  typedef AtomicBit OS_IF_ATOMIC_BIT;
#else
  typedef struct
  {
    OS_IF_LOCK critsect;
    unsigned long value;
  } OS_IF_ATOMIC_BIT;
  
  int test_and_clear_bit(int nr, OS_IF_ATOMIC_BIT *ab);
  int constant_test_bit(int nr, OS_IF_ATOMIC_BIT *ab);
  void clear_bit(int nr, OS_IF_ATOMIC_BIT *ab);
  void set_bit(int nr, OS_IF_ATOMIC_BIT *ab);
  void atomic_set_mask(long mask, long *addr);
  void atomic_clear_mask(long mask, long *addr);
#endif
  
  void os_if_init_atomic_bit(OS_IF_ATOMIC_BIT *ab);
  void os_if_remove_atomic_bit(OS_IF_ATOMIC_BIT *ab);

#endif //OSIF_FUNCTIONS_KERNEL_H_
