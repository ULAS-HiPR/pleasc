/*
 * FreeRTOS Kernel POSIX port — wait_for_event utility
 * A lightweight semaphore-like event built on pthread condvar.
 * MIT License — see FreeRTOS/LICENSE for terms.
 */
#pragma once
#ifdef LINUX

#include <stdbool.h>
#include <stdint.h>

typedef struct event event_t;

event_t * event_create(void);
void      event_delete(event_t * e);

/* Block until the event is signaled. Always returns true. */
bool event_wait(event_t * e);

/* Block until signaled or timeout_ms elapses. Returns true if signaled. */
bool event_wait_timed(event_t * e, uint32_t timeout_ms);

/* Signal the event, unblocking one waiter. */
void event_signal(event_t * e);

#endif /* LINUX */
