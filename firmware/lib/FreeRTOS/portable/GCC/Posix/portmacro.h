/*
 * FreeRTOS Kernel POSIX/Linux simulator port — portmacro.h
 * MIT License — see FreeRTOS/LICENSE for terms.
 *
 * Each FreeRTOS task runs as a pthread. Context switches are cooperative
 * (at osDelay / queue block points). A timer thread drives the tick.
 */
#ifdef LINUX
#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

/*-----------------------------------------------------------
 * Port specific definitions.
 *----------------------------------------------------------*/

/* Type definitions. */
#define portSTACK_TYPE      size_t
#define portBASE_TYPE       long

typedef portSTACK_TYPE  StackType_t;
typedef long            BaseType_t;
typedef unsigned long   UBaseType_t;
typedef uint32_t        TickType_t;

#define portMAX_DELAY               ( ( TickType_t ) 0xffffffffUL )
#define portTICK_TYPE_IS_ATOMIC     1

/* Hardware specifics — POSIX doesn't have a real stack direction,
 * but we pick downward to match ARM convention. */
#define portSTACK_GROWTH            ( -1 )
#define portTICK_PERIOD_MS          ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#define portBYTE_ALIGNMENT          8

/* Task function macros. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) \
    void vFunction( void * pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) \
    void vFunction( void * pvParameters )

/* Yield. */
extern void vPortYield( void );
#define portYIELD()                     vPortYield()
#define portEND_SWITCHING_ISR( x )      do { if( x ) vPortYield(); } while(0)
#define portYIELD_FROM_ISR( x )         portEND_SWITCHING_ISR( x )

/* Critical sections — implemented as a recursive mutex in port.c. */
extern void vPortEnterCritical( void );
extern void vPortExitCritical( void );

#define portCRITICAL_NESTING_IN_TCB         1
#define portENTER_CRITICAL()                vPortEnterCritical()
#define portEXIT_CRITICAL()                 vPortExitCritical()
#define portDISABLE_INTERRUPTS()            vPortEnterCritical()
#define portENABLE_INTERRUPTS()             vPortExitCritical()
#define portSET_INTERRUPT_MASK_FROM_ISR()   ( 0 )
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x) ( ( void ) ( x ) )

/* No-ops on Linux. */
#define portNOP()
#define portMEMORY_BARRIER()    __asm volatile( "" ::: "memory" )

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
#endif /* LINUX */
