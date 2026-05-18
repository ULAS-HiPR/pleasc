/*
 * FreeRTOS Kernel POSIX/Linux simulator port — port.c
 * MIT License — see FreeRTOS/LICENSE for terms.
 *
 * Design:
 *  - Each FreeRTOS task is a pthread. Only one runs at a time (guarded by
 *    xRunMutex). When a task is not scheduled it blocks on its own event_t.
 *  - The currently-running task's Thread_t* is stored in thread-local storage
 *    slot 0 of the FreeRTOS TCB (configNUM_THREAD_LOCAL_STORAGE_POINTERS >= 1).
 *  - A dedicated timer pthread fires the FreeRTOS tick at configTICK_RATE_HZ.
 *  - vPortYield() calls vTaskSwitchContext() then resumes the new current task.
 */
#ifdef LINUX

#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "utils/wait_for_event.h"

/*-----------------------------------------------------------*/

/* Per-task thread state. Stored in TCB thread-local storage slot 0. */
typedef struct {
    pthread_t       thread;
    event_t       * resume_event;   /* signaled to wake this task */
    TaskFunction_t  task_fn;
    void          * task_param;
    volatile int    started;        /* set to 1 when pthread is running */
} Thread_t;

/* Single mutex: only the scheduled task may hold this. */
static pthread_mutex_t xRunMutex = PTHREAD_MUTEX_INITIALIZER;

/* Critical section (recursive). */
static pthread_mutex_t xCritMutex;
static volatile UBaseType_t uxCritNesting = 0;

/* Set to 1 once the scheduler starts. */
static volatile BaseType_t xSchedulerRunning = pdFALSE;

/* Tick event: timer thread signals this to request a tick. */
static event_t * pxTickEvent = NULL;

/*-----------------------------------------------------------
 * Internal helpers
 *----------------------------------------------------------*/

/* Get the Thread_t for the task that is currently running in FreeRTOS. */
static Thread_t * prvGetCurrentThread( void )
{
    return ( Thread_t * ) pvTaskGetThreadLocalStoragePointer( NULL, 0 );
}

/* Suspend the calling task until it is re-scheduled. Called while holding xRunMutex. */
static void prvSuspendSelf( Thread_t * pxThread )
{
    pthread_mutex_unlock( &xRunMutex );
    event_wait( pxThread->resume_event );
    pthread_mutex_lock( &xRunMutex );
}

/* Resume a specific task (signal its event). Called while holding xRunMutex. */
static void prvResumeThread( Thread_t * pxThread )
{
    event_signal( pxThread->resume_event );
}

/*-----------------------------------------------------------
 * Task entry wrapper
 *----------------------------------------------------------*/

static void * prvTaskWrapper( void * pvArg )
{
    Thread_t * pxThread = ( Thread_t * ) pvArg;

    /* Wait until the scheduler kicks us off for the first time. */
    pthread_mutex_lock( &xRunMutex );

    /* Register our Thread_t with the FreeRTOS TCB. */
    vTaskSetThreadLocalStoragePointer( NULL, 0, pxThread );
    pxThread->started = 1;

    /* Unlock — the scheduler holds xRunMutex and will signal us when ready. */
    pthread_mutex_unlock( &xRunMutex );
    event_wait( pxThread->resume_event );
    pthread_mutex_lock( &xRunMutex );

    /* Run the actual FreeRTOS task. */
    pxThread->task_fn( pxThread->task_param );

    /* Task returned — should not happen in normal FreeRTOS operation. */
    vTaskDelete( NULL );
    pthread_mutex_unlock( &xRunMutex );
    return NULL;
}

/*-----------------------------------------------------------
 * Tick timer thread
 *----------------------------------------------------------*/

static void * prvTimerThread( void * pvArg )
{
    ( void ) pvArg;

    const long tick_ns = 1000000000L / configTICK_RATE_HZ;
    struct timespec ts  = { 0, tick_ns };

    while ( xSchedulerRunning )
    {
        nanosleep( &ts, NULL );
        event_signal( pxTickEvent );
    }
    return NULL;
}

/*-----------------------------------------------------------
 * Tick processor (runs in the idle task context)
 *----------------------------------------------------------*/

/* Called from vPortYield and from the idle task to process pending ticks. */
static void prvProcessTick( void )
{
    if( xTaskIncrementTick() != pdFALSE )
    {
        vTaskSwitchContext();
    }
}

/*-----------------------------------------------------------
 * Port API
 *----------------------------------------------------------*/

StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters )
{
    Thread_t * pxThread = ( Thread_t * ) malloc( sizeof( Thread_t ) );
    configASSERT( pxThread );
    memset( pxThread, 0, sizeof( Thread_t ) );

    pxThread->task_fn     = pxCode;
    pxThread->task_param  = pvParameters;
    pxThread->resume_event = event_create();
    pxThread->started     = 0;

    pthread_attr_t attr;
    pthread_attr_init( &attr );
    pthread_attr_setdetachstate( &attr, PTHREAD_CREATE_JOINABLE );
    int rc = pthread_create( &pxThread->thread, &attr, prvTaskWrapper, pxThread );
    pthread_attr_destroy( &attr );
    configASSERT( rc == 0 );

    /* Store Thread_t pointer at the top of the "stack" word.
     * FreeRTOS saves pxTopOfStack as the first TCB member and the port uses
     * pvTaskGetThreadLocalStoragePointer once the task starts — the stack word
     * is only used here to pass pxThread to prvTaskWrapper via the event path. */
    *pxTopOfStack = ( StackType_t ) pxThread;
    pxTopOfStack--;

    return pxTopOfStack;
}

BaseType_t xPortStartScheduler( void )
{
    /* Set up critical-section mutex as recursive. */
    pthread_mutexattr_t attr;
    pthread_mutexattr_init( &attr );
    pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_RECURSIVE );
    pthread_mutex_init( &xCritMutex, &attr );
    pthread_mutexattr_destroy( &attr );

    pxTickEvent = event_create();

    xSchedulerRunning = pdTRUE;

    /* Start the tick timer thread. */
    pthread_t tickThread;
    pthread_create( &tickThread, NULL, prvTimerThread, NULL );
    pthread_detach( tickThread );

    /* Grab the run mutex — we'll hand it to the first task. */
    pthread_mutex_lock( &xRunMutex );

    /* Let FreeRTOS pick the first task. */
    vTaskSwitchContext();

    Thread_t * pxFirst = ( Thread_t * ) pvTaskGetThreadLocalStoragePointer(
                             xTaskGetCurrentTaskHandle(), 0 );

    /* The first task's pthread may not have called vTaskSetThreadLocalStoragePointer
     * yet (it blocks before doing so). Wait briefly for it to start. */
    if( pxFirst == NULL )
    {
        /* Fall back: read Thread_t* from the stack (stored in pxPortInitialiseStack).
         * pxCurrentTCB first member is pxTopOfStack; *(pxTopOfStack+1) is our pointer. */
        TaskHandle_t h = xTaskGetCurrentTaskHandle();
        StackType_t * pxStack = *( StackType_t ** ) h;  /* TCB.pxTopOfStack */
        pxFirst = ( Thread_t * ) *( pxStack + 1 );
    }

    /* Signal the first task to run and release the mutex. */
    pthread_mutex_unlock( &xRunMutex );
    event_signal( pxFirst->resume_event );

    /* The "main" thread now acts as the tick processor.
     * It wakes up every tick, processes it, then goes back to sleep. */
    while( xSchedulerRunning )
    {
        event_wait( pxTickEvent );

        pthread_mutex_lock( &xRunMutex );
        prvProcessTick();

        /* Resume the newly selected task. */
        Thread_t * pxNext = prvGetCurrentThread();
        if( pxNext != NULL )
        {
            prvResumeThread( pxNext );
            /* Main thread gives up the lock; task picks it up via prvSuspendSelf. */
            pthread_mutex_unlock( &xRunMutex );
        }
        else
        {
            pthread_mutex_unlock( &xRunMutex );
        }
    }

    return pdTRUE; /* Should never reach here. */
}

void vPortEndScheduler( void )
{
    xSchedulerRunning = pdFALSE;
}

void vPortYield( void )
{
    Thread_t * pxCurrent = prvGetCurrentThread();

    pthread_mutex_lock( &xRunMutex );
    vTaskSwitchContext();
    Thread_t * pxNext = prvGetCurrentThread();

    if( pxNext != pxCurrent )
    {
        prvResumeThread( pxNext );
        prvSuspendSelf( pxCurrent );
    }
    pthread_mutex_unlock( &xRunMutex );
}

void vPortEnterCritical( void )
{
    pthread_mutex_lock( &xCritMutex );
    uxCritNesting++;
}

void vPortExitCritical( void )
{
    configASSERT( uxCritNesting > 0 );
    uxCritNesting--;
    pthread_mutex_unlock( &xCritMutex );
}

/* FreeRTOS calls this when it wants to delay a task. On POSIX we just yield;
 * the tick timer will wake the task when its delay expires via vTaskDelay. */
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
    ( void ) xExpectedIdleTime;
    /* Sleep until the next tick event rather than busy-waiting. */
    event_wait_timed( pxTickEvent, ( uint32_t )( xExpectedIdleTime * portTICK_PERIOD_MS ) );
}

#endif /* LINUX */
