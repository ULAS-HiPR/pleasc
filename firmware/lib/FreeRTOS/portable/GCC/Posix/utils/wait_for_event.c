/*
 * FreeRTOS Kernel POSIX port — wait_for_event utility
 * MIT License — see FreeRTOS/LICENSE for terms.
 */
#ifdef LINUX

#include "utils/wait_for_event.h"

#include <pthread.h>
#include <stdlib.h>
#include <time.h>
#include <errno.h>

struct event {
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    volatile bool   set;
};

event_t * event_create(void) {
    event_t * e = malloc(sizeof(event_t));
    pthread_mutex_init(&e->mutex, NULL);
    pthread_cond_init(&e->cond, NULL);
    e->set = false;
    return e;
}

void event_delete(event_t * e) {
    pthread_mutex_destroy(&e->mutex);
    pthread_cond_destroy(&e->cond);
    free(e);
}

bool event_wait(event_t * e) {
    pthread_mutex_lock(&e->mutex);
    while (!e->set) {
        pthread_cond_wait(&e->cond, &e->mutex);
    }
    e->set = false;
    pthread_mutex_unlock(&e->mutex);
    return true;
}

bool event_wait_timed(event_t * e, uint32_t timeout_ms) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }

    pthread_mutex_lock(&e->mutex);
    while (!e->set) {
        int rc = pthread_cond_timedwait(&e->cond, &e->mutex, &ts);
        if (rc == ETIMEDOUT) {
            pthread_mutex_unlock(&e->mutex);
            return false;
        }
    }
    e->set = false;
    pthread_mutex_unlock(&e->mutex);
    return true;
}

void event_signal(event_t * e) {
    pthread_mutex_lock(&e->mutex);
    e->set = true;
    pthread_cond_signal(&e->cond);
    pthread_mutex_unlock(&e->mutex);
}

#endif /* LINUX */
