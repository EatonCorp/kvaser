/*
** Copyright 2002-2006 KVASER AB, Sweden.  All rights reserved.
*/

#if LINUX
//--------------------------------------------------
// NOTE! module_versioning HAVE to be included first
#include "module_versioning.h"
//--------------------------------------------------

#   include <linux/version.h>
#   include <linux/slab.h>
#   include <linux/sched.h>
#   include <linux/interrupt.h>
#   include <asm/io.h>
#   include <asm/system.h>
#   include <asm/bitops.h>
#   include <asm/uaccess.h>
#   if LINUX_2_6
#       include <linux/workqueue.h>
#       include <linux/wait.h>
#       include <linux/completion.h>
#       include <linux/kthread.h>
#       include <linux/module.h>
#   else
#       include <linux/tqueue.h>
#   endif
#else
#   include <windows.h>
#   include <ceddk.h>
#   include <Winbase.h>
#   include <excpt.h>
#endif

// Common
#include "osif_kernel.h"
#include "osif_functions_kernel.h"
#include "debug.h"

#ifdef BREAKPOINTS
short debug_break = 0x0000;
#endif

#if !LINUX
/* This is more or less copied from the Linux version,
 * only as a function and mostly OS independent.
 */
int os_if_wait_event_interruptible_timeout (OS_IF_WAITQUEUE_HEAD *wq,
                                            int (*eq)(void *),
                                            void *par, int timeout)
{
#if LINUX
  long ret = timeout;    // Not correct since timeout is supposed to be ms
#else
  long ret = timeout;
#endif

  if (!eq(par)) {
    OS_IF_WAITQUEUE __wait;
    unsigned long int timeOut = GetTickCount();
    long int doBreak = 0;
    os_if_init_waitqueue_entry(&__wait);

#if 1
#if 0     // This caused crash due to unavailable memory address!
          // Needs to be fixed if it is to be used (not currently)
    os_if_add_wait_queue(wq, &__wait);
#endif
    while (!doBreak) {
      if (eq(par)) {
        break;
      }
      while (1) {
        os_if_wait_for_event_timeout_simple(1);
        if (eq(par)) {
          doBreak = 1;
          break;
        }
        if ((GetTickCount() - timeOut) >= (unsigned long)ret) {
          ret = WAIT_TIMEOUT;
          doBreak = 1;
          break;
        }
      }
    }
#else
    os_if_add_wait_queue(&wq, &__wait);
    for (;;) {
      if (eq(par)) {
        break;
      }
      ret = os_if_wait_for_event_timeout(ret, &__wait);
      if (!ret) {
        break;
      }
    }
#endif
    os_if_remove_wait_queue(wq, &__wait);
  }

  return ret;
}
#endif

#if LINUX
//////////////////////////////////////////////////////////////////////
// os_if_write_port
// write to port
//////////////////////////////////////////////////////////////////////
void os_if_write_port (unsigned regist, unsigned portAddr)
{
#if LINUX
  outb(regist, portAddr);
#else
  WRITE_PORT_UCHAR((PUCHAR)portAddr, (UCHAR)regist);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_read_port
// read from port
//////////////////////////////////////////////////////////////////////
unsigned int os_if_read_port (unsigned portAddr)
{
#if LINUX
  return inb(portAddr);
#else
  return READ_PORT_UCHAR((PUCHAR)portAddr);
#endif
}
#endif

//////////////////////////////////////////////////////////////////////
// os_if_queue_task
// put task to queue/set event
//////////////////////////////////////////////////////////////////////
int os_if_queue_task (OS_IF_TASK_QUEUE_HANDLE *hnd)
{
#if LINUX
    #if LINUX_2_6
      return schedule_work(hnd); // ret nonzero if ok
    #else
      int ret = queue_task(hnd, &tq_immediate);
      mark_bh(IMMEDIATE_BH);
      return ret;
    #endif
#else
  // This is not used, so don't bother with implementation for now.
  UNREFERENCED_PARAMETER(hnd);
  DEBUGOUT(ZONE_ERR, (TXT("os_if_queue_task not implemented!")));
  return 1;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_queue_task_not_default_queue
// put task to queue/set event
//////////////////////////////////////////////////////////////////////
int os_if_queue_task_not_default_queue (OS_IF_WQUEUE *wq,
                                        OS_IF_TASK_QUEUE_HANDLE *hnd)
{
#if LINUX
    #if LINUX_2_6
      return queue_work(wq, hnd); // ret nonzero if ok
    #else
      int ret = queue_task(hnd, &tq_immediate);
      mark_bh(IMMEDIATE_BH);
      return ret;
    #endif
#else
  UNREFERENCED_PARAMETER(hnd);

  os_if_up_countsem(&wq->info.work, 1);

  return 1;
#endif
}



//////////////////////////////////////////////////////////////////////
// os_if_init_waitqueue_head
//
//////////////////////////////////////////////////////////////////////
void os_if_init_waitqueue_head (OS_IF_WAITQUEUE_HEAD *handle)
{
#if LINUX
  init_waitqueue_head(handle);
#else
  // Instead of on a queue, everyone will be waiting on this event.
  // The event will later be pulsed, releasing all currently waiting.
  // See os_if_wake_up_interruptible().
  *handle = CreateEvent(NULL, TRUE, FALSE, NULL);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_delete_waitqueue_head
//
//////////////////////////////////////////////////////////////////////
void os_if_delete_waitqueue_head (OS_IF_WAITQUEUE_HEAD *handle)
{
#if LINUX
#else
  if (!CloseHandle(*handle)) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to delete waitqueue head (%d)!\n"), GetLastError()));
  }

  *handle = 0;
#endif
}


//////////////////////////////////////////////////////////////////////
// os_if_init_waitqueue_entry
//
//////////////////////////////////////////////////////////////////////
void os_if_init_waitqueue_entry (OS_IF_WAITQUEUE *wait)
{
#if LINUX
    #if LINUX_2_6
      init_waitqueue_entry(wait, current);
    #else
      init_waitqueue_entry(wait, current);
    #endif
#else
  // Wait queues are implemented using events, so nothing is needed here.
  UNREFERENCED_PARAMETER(wait);
#endif
}


//////////////////////////////////////////////////////////////////////
// os_if_add_wait_queue
//
//////////////////////////////////////////////////////////////////////
void os_if_add_wait_queue (OS_IF_WAITQUEUE_HEAD *waitQ, OS_IF_WAITQUEUE *wait)
{
#if LINUX
    #if LINUX_2_6
      add_wait_queue(waitQ, wait);
    #else
      add_wait_queue(waitQ, wait);
    #endif
#else
  // There is no queue, so just point out the event.
  *wait = *waitQ;
#endif
}


//////////////////////////////////////////////////////////////////////
// os_if_remove_wait_queue
//
//////////////////////////////////////////////////////////////////////
void os_if_remove_wait_queue (OS_IF_WAITQUEUE_HEAD *waitQ, OS_IF_WAITQUEUE *wait)
{
#if LINUX
    #if LINUX_2_6
      remove_wait_queue(waitQ, wait);
    #else
      remove_wait_queue(waitQ, wait);
    #endif
#else
  UNREFERENCED_PARAMETER(waitQ);
  UNREFERENCED_PARAMETER(wait);
#endif
}


//////////////////////////////////////////////////////////////////////
// os_if_wait_for_event_timeout
//
//////////////////////////////////////////////////////////////////////
signed long os_if_wait_for_event_timeout (signed long timeout,
                                          OS_IF_WAITQUEUE *handle)
{
#if LINUX
    #if LINUX_2_6
      //return wait_event_interruptible_timeout(handle, condition, timeout);
      return schedule_timeout(timeout);
    #else
      return schedule_timeout(timeout);
    #endif
#else
  if (!handle) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Bad event handle for wait_for_event_timeout!\n")));
    return ERROR_INVALID_HANDLE;
  }
  switch (WaitForSingleObject(*handle, timeout)) {
  case WAIT_OBJECT_0:
    return 1;
  case WAIT_TIMEOUT:
    return 0;
  }

  DEBUGOUT(ZONE_ERR, (TXT("Failed to wait on event(%d)!\n"), GetLastError()));

  return GetLastError();
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_schedule_timeout_simple
//
//////////////////////////////////////////////////////////////////////
signed long os_if_wait_for_event_timeout_simple (signed long timeout)
{
#if LINUX
    #if LINUX_2_6
      return schedule_timeout(timeout);
    #else
      return schedule_timeout(timeout);
    #endif
#else
  Sleep(timeout);

  return 0;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_schedule
//
//////////////////////////////////////////////////////////////////////
void os_if_wait_for_event (OS_IF_WAITQUEUE_HEAD *handle)
{
#if LINUX
  schedule();
#else
  // No longer used (in favour of _timeout, which uses a "queue object"),
  // so don't worry about the fact that this can't know about a channel
  // being closed, for now.
  if (WAIT_OBJECT_0 == WaitForSingleObject(*handle, INFINITE))
    return;

  DEBUGOUT(ZONE_ERR,
           (TXT("Failed to wait on semaphore(%d)!\n"), GetLastError()));
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_wake_up_interruptible
//
//////////////////////////////////////////////////////////////////////
void os_if_wake_up_interruptible (OS_IF_WAITQUEUE_HEAD *handle)
{
#if LINUX
  wake_up_interruptible(handle);
#else
  // Wake up everyone waiting on the "queue" at this moment.
  // (The event automatically goes back to non-signalled again.)
  PulseEvent(*handle);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_up_sema
//
//////////////////////////////////////////////////////////////////////
void os_if_up_sema (OS_IF_SEMAPHORE *var)
{
#if LINUX
#   if LINUX_2_6
      complete(var);
      //os_if_wake_up_interruptible(var);
#   else
      up(var);
#   endif
#else
#if 1
  if (!SetEvent(*var)) {
#else
  if (!ReleaseSemaphore(*var, 1, 0)) {
#endif
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to signal semaphore(%d)!\n"), GetLastError()));
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_down_sema
//
//////////////////////////////////////////////////////////////////////
void os_if_down_sema (OS_IF_SEMAPHORE *var)
{
#if LINUX
#   if LINUX_2_6
      wait_for_completion(var);
      // interruptible_sleep_on(var);
#   else
      down(var);
#   endif
#else
  if (WAIT_OBJECT_0 != WaitForSingleObject(*var, INFINITE)) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to wait on semaphore(%d)!\n"), GetLastError()));
  }
#endif
}

#if !LINUX
//////////////////////////////////////////////////////////////////////
// os_if_down_sema_time
//
//////////////////////////////////////////////////////////////////////
int os_if_down_sema_time (OS_IF_SEMAPHORE *var, int timeout)
{
#if LINUX
#else
  switch (WaitForSingleObject(*var, timeout)) {
  case WAIT_OBJECT_0:
    return 1;
  case WAIT_TIMEOUT:
    return 0;
  }

  DEBUGOUT(ZONE_ERR,
           (TXT("Failed to wait on semaphore(%d)!\n"), GetLastError()));

  return GetLastError();
#endif
}
#endif


//////////////////////////////////////////////////////////////////////
// os_if_init_sema
//
//////////////////////////////////////////////////////////////////////
void os_if_init_sema (OS_IF_SEMAPHORE *var)
{
#if LINUX
#   if LINUX_2_6
      init_completion(var);
      //os_if_init_waitqueue_head(var);
#   else
      sema_init(var, 0);
#   endif
#else
#if 1
  // Auto reset event (single wait possible per set)
  *var = CreateEvent(0, FALSE, FALSE, 0);
#else
  *var = CreateSemaphore(0, 0, 1, 0);    // Not available at the beginning
#endif
  if (!*var) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to create semaphore(%d)!\n"), GetLastError()));
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_delete_sema
//
//////////////////////////////////////////////////////////////////////
void os_if_delete_sema (OS_IF_SEMAPHORE *var)
{
#if LINUX
#else
  if (!CloseHandle(*var)) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to delete semaphore (%d)!\n"), GetLastError()));
  }

  *var = 0;
#endif
}

#if !LINUX
//////////////////////////////////////////////////////////////////////
// os_if_up_countsem
//
//////////////////////////////////////////////////////////////////////
void os_if_up_countsem (OS_IF_COUNTSEM *sem, int count)
{
#if LINUX
#else
  if (!ReleaseSemaphore(*sem, count, 0)) {
#   if 0   // Error text removed since it is displayed when semaphore is maxed
      DEBUGOUT(ZONE_TRACE, (TXT("Failed to signal counting semaphore(%d)!\n"),
                            GetLastError()));
#   endif
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_down_countsem
//
//////////////////////////////////////////////////////////////////////
void os_if_down_countsem (OS_IF_COUNTSEM *sem)
{
#if LINUX
#else
  if (WAIT_OBJECT_0 != WaitForSingleObject(*sem, INFINITE)) {
      DEBUGOUT(ZONE_ERR, (TXT("Failed to wait on counting semaphore(%d)!\n"),
                          GetLastError()));
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_init_countsem
//
//////////////////////////////////////////////////////////////////////
void os_if_init_countsem (OS_IF_COUNTSEM *sem, int count, int max)
{
#if LINUX
#else
  *sem = CreateSemaphore(0, count, max, 0);
  if (!*sem) {
    DEBUGOUT(ZONE_ERR, (TXT("Failed to create counting semaphore(%d)!\n"),
                        GetLastError()));
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_delete_countsem
//
//////////////////////////////////////////////////////////////////////
void os_if_delete_countsem (OS_IF_COUNTSEM *sem)
{
#if LINUX
#else
  if (!CloseHandle(*sem)) {
    DEBUGOUT(ZONE_ERR, (TXT("Failed to delete counting semaphore (%d)!\n"),
                        GetLastError()));
  }

  *sem = 0;
#endif
}


//////////////////////////////////////////////////////////////////////
// os_if_set_cond
//
//////////////////////////////////////////////////////////////////////
void os_if_set_cond (OS_IF_EVENT *event)
{
#if LINUX
#else
  if (!SetEvent(*event)) {
    DEBUGOUT(ZONE_ERR, (TXT("Failed to set event(%d)!\n"), GetLastError()));
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_clear_cond
//
//////////////////////////////////////////////////////////////////////
void os_if_clear_cond (OS_IF_EVENT *event)
{
#if LINUX
#else
  if (!ResetEvent(*event)) {
    DEBUGOUT(ZONE_ERR, (TXT("Failed to clear event(%d)!\n"), GetLastError()));
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_wait_for_cond
//
//////////////////////////////////////////////////////////////////////
void os_if_wait_for_cond (OS_IF_EVENT *event)
{
#if LINUX
#else
  if (WAIT_OBJECT_0 != WaitForSingleObject(*event, INFINITE)) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to wait on event(%d)!\n"), GetLastError()));
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_init_cond
//
//////////////////////////////////////////////////////////////////////
void os_if_init_cond (OS_IF_EVENT *sem)
{
#if LINUX
#else
  *sem = CreateEvent(0, TRUE, FALSE, 0);    // Manual reset
  if (!*sem) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to create event(%d)!\n"), GetLastError()));
  }
#endif
}


//////////////////////////////////////////////////////////////////////
// os_if_delete_cond
//
//////////////////////////////////////////////////////////////////////
void os_if_delete_cond (OS_IF_EVENT *event)
{
#if LINUX
#else
  if (!CloseHandle(*event)) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to delete event (%d)!\n"), GetLastError()));
  }

  *event = 0;
#endif
}
#endif


//////////////////////////////////////////////////////////////////////
// Time management functions
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_get_timeout_time
//
//////////////////////////////////////////////////////////////////////
unsigned long os_if_get_timeout_time (void)
{
#if LINUX
  return jiffies + 1 * HZ;
#else
  return GetTickCount() + 10000;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_do_get_time_of_day
//
//////////////////////////////////////////////////////////////////////
void os_if_do_get_time_of_day (OS_IF_TIME_VAL *tv)
{
#if LINUX
  do_gettimeofday(tv);
#else
  long tmpTime = GetTickCount();
 #if 1
  DEBUGOUT(2, (TXT("GetTickCount() returned %ld"), tmpTime));
 #endif
  tv->tv_sec  = tmpTime / 1000;
  tv->tv_usec = (tmpTime % 1000) * 1000;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_is_rec_busy
//
//////////////////////////////////////////////////////////////////////
int os_if_is_rec_busy (int nr, volatile unsigned long *addr)
{
#if LINUX
  return test_and_set_bit(nr, addr);
#else
  UNREFERENCED_PARAMETER(nr);
  UNREFERENCED_PARAMETER(addr);
  // qqq

  return 1;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_rec_not_busy
//
//////////////////////////////////////////////////////////////////////
void os_if_rec_not_busy (int nr, volatile unsigned long *addr)
{
#if LINUX
  return clear_bit(nr, addr);
#else
  // qqq
  UNREFERENCED_PARAMETER(nr);
  UNREFERENCED_PARAMETER(addr);
#endif
}

//////////////////////////////////////////////////////////////////////
// Spin locks
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_spin_lock_init
//
//////////////////////////////////////////////////////////////////////
void os_if_spin_lock_init (OS_IF_LOCK *lock)
{
#if LINUX
  spin_lock_init(lock);
#else
  InitializeCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_spin_lock
//
//////////////////////////////////////////////////////////////////////
void os_if_spin_lock (OS_IF_LOCK *lock)
{
#if LINUX
  spin_lock(lock);
#else
  EnterCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_spin_unlock
//
//////////////////////////////////////////////////////////////////////
void os_if_spin_unlock (OS_IF_LOCK *lock)
{
#if LINUX
  spin_unlock(lock);
#else
  LeaveCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_spin_lock_remove
//
//////////////////////////////////////////////////////////////////////
void os_if_spin_lock_remove (OS_IF_LOCK *lock)
{
#if LINUX
#else
  DeleteCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// Interrupts
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_irq_disable
//
//////////////////////////////////////////////////////////////////////
void os_if_irq_disable (OS_IF_LOCK *lock)
{
// qqq should work for 2_4 too!
#if LINUX
    #if LINUX_2_6
      spin_lock_irq(lock);
    #else
      cli();
    #endif
#else
  EnterCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_irq_enable
//
//////////////////////////////////////////////////////////////////////
void os_if_irq_enable (OS_IF_LOCK *lock)
{
#if LINUX
  // qqq should work for 2_4 too!
    #if LINUX_2_6
      spin_unlock_irq(lock);
    #else
      sti();
    #endif
#else
  LeaveCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_irq_save
//
//////////////////////////////////////////////////////////////////////
void os_if_irq_save (OS_IF_LOCK *lock, unsigned long *flags)
{
#if LINUX
  // not needed in 2_4
    #if LINUX_2_6
      spin_lock_irqsave(lock, *flags);
    #endif
#else
  UNREFERENCED_PARAMETER(flags);
  EnterCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_irq_restore
//
//////////////////////////////////////////////////////////////////////
void os_if_irq_restore (OS_IF_LOCK *lock, unsigned long flags)
{
#if LINUX
  // not needed in 2_4
    #if LINUX_2_6
      spin_unlock_irqrestore(lock, flags);
    #endif
#else
  UNREFERENCED_PARAMETER(flags);
  LeaveCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_spin_lock_irqsave
//
//////////////////////////////////////////////////////////////////////
void os_if_spin_lock_irqsave (OS_IF_LOCK *lock, unsigned long *flags)
{
#if LINUX
  spin_lock_irqsave(lock, *flags);
#else
  UNREFERENCED_PARAMETER(flags);
  EnterCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_spin_unlock_irqrestore
//
//////////////////////////////////////////////////////////////////////
void os_if_spin_unlock_irqrestore (OS_IF_LOCK *lock, unsigned long flags)
{
#if LINUX
  spin_unlock_irqrestore(lock, flags);
#else
  UNREFERENCED_PARAMETER(flags);
  LeaveCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_spin_lock_softirq
//
//////////////////////////////////////////////////////////////////////
void os_if_spin_lock_softirq (OS_IF_LOCK *lock)
{
#if LINUX
  spin_lock_bh(lock);
#else
  EnterCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_spin_unlock_softirq
//
//////////////////////////////////////////////////////////////////////
void os_if_spin_unlock_softirq (OS_IF_LOCK *lock)
{
#if LINUX
  spin_unlock_bh(lock);
#else
  LeaveCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// Data transfer between user and kernel space
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_get_user_data
//
//////////////////////////////////////////////////////////////////////
int os_if_get_user_data (void *to, const void *from, OS_IF_SIZE n)
{
#if LINUX
  return copy_to_user(to, from, n);
#else
#if USE_TRY_EXCEPT
  __try {
#endif
    memcpy(to, from, n);
#if USE_TRY_EXCEPT
  }
  __except(EXCEPTION_EXECUTE_HANDLER) {
    return 1;
  }
#endif

  return 0;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_set_user_data
//
//////////////////////////////////////////////////////////////////////
int os_if_set_user_data (void *to, const void *from, OS_IF_SIZE n)
{
#if LINUX
  return copy_from_user(to, from, n);
#else
#if USE_TRY_EXCEPT
  __try {
#endif
    memcpy(to, from, n);
#if USE_TRY_EXCEPT
  }
  __except(EXCEPTION_EXECUTE_HANDLER) {
    return 1;
  }
#endif

  return 0;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_set_int
//
//////////////////////////////////////////////////////////////////////
int os_if_set_int (int val, int *dest)
{
#if LINUX
  return put_user(val, dest);
#else
#if USE_TRY_EXCEPT
  __try {
#endif
    *dest = val;
#if USE_TRY_EXCEPT
  }
  __except(EXCEPTION_EXECUTE_HANDLER) {
    return 1;
  }
#endif

  return 0;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_get_int
//
//////////////////////////////////////////////////////////////////////
int os_if_get_int (int *val, int *src)
{
#if LINUX
  return get_user(*val, src);
#else
#if USE_TRY_EXCEPT
  __try {
#endif
    *val = *src;
#if USE_TRY_EXCEPT
  }
  __except(EXCEPTION_EXECUTE_HANDLER) {
    return 1;
  }
#endif

  return 0;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_get_long
//
//////////////////////////////////////////////////////////////////////
int os_if_get_long (long *val, long *src)
{
#if LINUX
  return get_user(*val, src);
#else
#if USE_TRY_EXCEPT
  __try {
#endif
    *val = *src;
#if USE_TRY_EXCEPT
  }
  __except(EXCEPTION_EXECUTE_HANDLER) {
    return 1;
  }
#endif

  return 0;
#endif
}



//////////////////////////////////////////////////////////////////////
// Task management functions
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_declare_task
//
//////////////////////////////////////////////////////////////////////
OS_IF_WQUEUE* os_if_declare_task (char *name, OS_IF_TASK_QUEUE_HANDLE *taskQ)
{
#if LINUX
#   if LINUX_2_6
      return create_workqueue(name);
#   else
      return 0;
#   endif
#else
  UNREFERENCED_PARAMETER(name);

  return taskQ;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_destroy_task
//
//////////////////////////////////////////////////////////////////////
void os_if_destroy_task (OS_IF_WQUEUE *wQueue)
{
#if LINUX
#   if LINUX_2_6
      destroy_workqueue(wQueue);
#   else
#   endif
#else
  wQueue->info.shutdown = 1;

  // Tell task that there is some work to do.
  // There is not, really, but it wakes the task up
  // so that it can see the shutdown flag.
  os_if_up_countsem(&wQueue->info.work, 1);

  // Wait for the task to end.
  os_if_down_sema(&wQueue->info.finished);

  os_if_delete_countsem(&wQueue->info.work);
  os_if_delete_sema(&wQueue->info.finished);
  os_if_remove_thread(wQueue->thread);
  // Wait for destruction?
#endif
}

#if !LINUX
static DWORD task (LPVOID context)
{
  struct task_info *info = (struct task_info *)context;

  // Loop and do work while not asked to shut down.
  while (!info->shutdown) {
    os_if_down_countsem(&info->work);

    BREAK(0x8000, "Task1");

    if (!info->shutdown) {
      info->routine(info->data);
    }
  }

  // Task is ending now.
  os_if_up_sema(&info->finished);

  BREAK(0x8000, "Task2");

  return 0;
}
#endif

//////////////////////////////////////////////////////////////////////
// os_if_init_task
//
//////////////////////////////////////////////////////////////////////
void os_if_init_task (OS_IF_TASK_QUEUE_HANDLE *taskQ, void *function, void *data)
{
#if LINUX
    #if LINUX_2_6
        // Work queue functions get the taskQ pointer in 2.6.20+.
        #if (LINUX_VERSION_CODE < 0x020614)
          INIT_WORK(taskQ, function, data);
        #else
          INIT_WORK(taskQ, function);
        #endif
    #else
      taskQ->routine  = function;
      taskQ->data     = data;
    #endif
#else
  taskQ->info.data = data;
  taskQ->info.routine = function;
  taskQ->info.shutdown = 0;
  // Only count to 1 item of work
  // In the WinCE lapcan driver, the amount of work given does not relate
  // to the number of times our task needs to run, anyway. And the task
  // can give itself some extra work to make sure all queues are emptied.
  os_if_init_countsem(&taskQ->info.work, 0, 1);
  os_if_init_sema(&taskQ->info.finished);
  taskQ->thread = os_if_kernel_thread(task, &taskQ->info);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_set_task_interruptible
//
//////////////////////////////////////////////////////////////////////
void os_if_set_task_interruptible (void)
{
#if LINUX
  set_current_state(TASK_INTERRUPTIBLE);
#else
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_set_task_uninterruptible
//
//////////////////////////////////////////////////////////////////////
void os_if_set_task_uninterruptible (void)
{
#if LINUX
  set_current_state(TASK_UNINTERRUPTIBLE);
#else
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_set_task_running
//
//////////////////////////////////////////////////////////////////////
void os_if_set_task_running (void)
{
#if LINUX
  set_current_state(TASK_RUNNING);
#else
#endif
}

//////////////////////////////////////////////////////////////////////
// Read and write locks
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_rwlock_init
//
//////////////////////////////////////////////////////////////////////
void os_if_rwlock_init (rwlock_t *lock)
{
#if LINUX
  // Do nothing
#if LINUX_2_6
  rwlock_init(lock);
#endif
#else
  InitializeCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_read_lock_irqsave
//
//////////////////////////////////////////////////////////////////////
void os_if_read_lock_irqsave (rwlock_t *rw_lock, unsigned long *flags)
{
#if LINUX
  // Do nothing
#if LINUX_2_6
  read_lock_irqsave(rw_lock, *flags);
#else
#endif
#else
  UNREFERENCED_PARAMETER(flags);
  EnterCriticalSection(rw_lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_read_unlock_irqrestore
//
//////////////////////////////////////////////////////////////////////
void os_if_read_unlock_irqrestore (rwlock_t *rw_lock, unsigned long flags)
{
#if LINUX
  // Do nothing
#if LINUX_2_6
  read_unlock_irqrestore(rw_lock, flags);
#else
#endif
#else
  UNREFERENCED_PARAMETER(flags);
  LeaveCriticalSection(rw_lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_write_lock_irqsave
//
//////////////////////////////////////////////////////////////////////
void os_if_write_lock_irqsave (rwlock_t *rw_lock, unsigned long *flags)
{
#if LINUX
  // do nothing
    #if LINUX_2_6
      write_lock_irqsave(rw_lock, *flags);
    #endif
#else
  UNREFERENCED_PARAMETER(flags);
  EnterCriticalSection(rw_lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_write_unlock_irqrestore
//
//////////////////////////////////////////////////////////////////////
void os_if_write_unlock_irqrestore (rwlock_t *rw_lock, unsigned long flags)
{
#if LINUX
  // do nothing
    #if LINUX_2_6
      write_unlock_irqrestore(rw_lock, flags);
    #endif
#else
  UNREFERENCED_PARAMETER(flags);
  LeaveCriticalSection(rw_lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_rwlock_remove
//
//////////////////////////////////////////////////////////////////////
void os_if_rwlock_remove (rwlock_t *lock)
{
#if LINUX
#else
  DeleteCriticalSection(lock);
#endif
}

//////////////////////////////////////////////////////////////////////
// System signals
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_signal_pending
//
//////////////////////////////////////////////////////////////////////
int os_if_signal_pending(void)
{
#if LINUX
  return signal_pending(current);
#else
  return 0;
#endif
}


//////////////////////////////////////////////////////////////////////
// Memory allocation functions
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_kernel_malloc
//
//////////////////////////////////////////////////////////////////////
void *os_if_kernel_malloc (size_t buffer_size)
{
#if LINUX
  return kmalloc(buffer_size, GFP_KERNEL);
#else
  void *addr;

  addr = LocalAlloc(LMEM_FIXED, buffer_size);
  if (!addr) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to allocate memory(%d)!\n"), GetLastError()));
  }
 #if 0
  else {
    DEBUGOUT(ZONE_TRACE, (TXT("Allocated memory at %x\n"), addr));
  }
 #endif

  return addr;
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_kernel_free
//
//////////////////////////////////////////////////////////////////////
void os_if_kernel_free (void *mem_ptr)
{
#if LINUX
  kfree(mem_ptr);
#else
  if (LocalFree(mem_ptr)) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to free memory(%d)!\n"), GetLastError()));
  }
 #if 0
  else {
    DEBUGOUT(ZONE_TRACE, (TXT("Freed memory at %x\n"), mem_ptr));
  }
 #endif
#endif
}

//////////////////////////////////////////////////////////////////////
// Thread management functions
//
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// os_if_kernel_thread
//
//////////////////////////////////////////////////////////////////////
#if LINUX
#  if LINUX_2_6
OS_IF_THREAD os_if_kernel_thread (int (*thread)(void *context), void *context)
{
#  else
// 2.4
#  endif
#else
OS_IF_THREAD os_if_kernel_thread (DWORD (*thread)(LPVOID context), LPVOID context)
{
#endif

#if LINUX
#   if LINUX_2_6
 #if 0
      return kernel_thread(thread, context, CLONE_KERNEL);
 #else
      return kthread_run(thread, context, "Kvaser kernel thread");
 #endif
#   else
      // 2.4
#   endif
#else
  OS_IF_THREAD hnd;

  // Set a smallish stack for the kernel thread.
  hnd = CreateThread(0, 32768, thread, context,
                     STACK_SIZE_PARAM_IS_A_RESERVATION, 0);
  if (!hnd) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to create thread(%d)!\n"), GetLastError()));
  }

  // The CE documentation mentioned USB Function at priority 100, so why not?
  // Might be a good idea to have this modifiable via the registry.
  if (!CeSetThreadPriority(hnd, 100)) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Could not change thread priority(%d)!\n"), GetLastError()));
  }

  return hnd;
#endif
#if LINUX
#  if LINUX_2_6
}
#  else
// 2.4
#  endif
#else
}
#endif

//////////////////////////////////////////////////////////////////////
// os_if_remove_thread
//
//////////////////////////////////////////////////////////////////////
void os_if_remove_thread (OS_IF_THREAD thread)
{
#if LINUX
#else
  if (!CloseHandle(thread)) {
    DEBUGOUT(ZONE_ERR,
             (TXT("Failed to remove thread(%d)!\n"), GetLastError()));
  }
#endif
}

//////////////////////////////////////////////////////////////////////
// os_if_exit_thread
//
//////////////////////////////////////////////////////////////////////
void os_if_exit_thread (int result)
{
#if LINUX
#   if LINUX_2_6
      module_put_and_exit(result);
#   else
      // 2.4
#   endif
#else
#endif
}
