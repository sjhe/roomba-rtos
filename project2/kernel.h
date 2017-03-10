/**
 * @file   kernel.h
 *
 * @brief kernel data structures used in os.c.
 *
 * CSC 460/560 Real Time Operating Systems - Mantis Cheng
 *
 * @author Scott Craig
 * @author Justin Tanner
 */
#ifndef __KERNEL_H__
#define __KERNEL_H__

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000

#include <util/delay.h>
#include "os.h"

/** Disable default prescaler to make processor speed 8 MHz. */
// #define CLOCK8MHZ()    CLKPR = (1<<CLKPCE); CLKPR = 0x00;

#define Disable_Interrupt()    asm volatile ("cli"::)
#define Enable_Interrupt()     asm volatile ("sei"::)

/** The maximum number of names. Currently the same as the number of tasks. */
#define MAXNAME		MAXTHREAD

/** The number of clock cycles in one "tick" or 5 ms */
#define TICK_CYCLES     (F_CPU / 1000 * TICK)

/** LEDs for OS_Abort() */
// #define LED_RED_MASK    

/** LEDs for OS_Abort() */
// #define LED_GREEN_MASK   


/* Typedefs and data structures. */

typedef void (*voidfuncptr) (void);      /* pointer to void f(void) */ 

/**
 *  This is the set of states that a task can be in at any given time.
 */
typedef enum process_states
{
	DEAD = 0,
	RUNNING,
	READY,
	WAITING  // Waiting state
} PROCESS_STATES;

/**
 * This is the set of kernel requests, i.e., a request code for each system call.
 */
typedef enum kernel_request_type
{
	NONE = 0,
	TIMER_EXPIRED,
	CREATE,
	TERMINATE,
	NEXT,
	TASK_GET_ARG,
	EVENT_INIT,
	EVENT_WAIT,
	EVENT_SIGNAL,
	EVENT_BROADCAST,
	EVENT_SIGNAL_AND_NEXT,
	EVENT_BROADCAST_AND_NEXT
} KERNEL_REQUEST_TYPE;

/**
 * Each task is represented by a process descriptor, which contains all
 * relevant information about this task. For convenience, we also store
 * the task's stack, i.e., its workspace, in here.
 */
typedef struct ProcessDescriptor 
{
	volatile unsigned char *sp;   /* stack pointer into the "workSpace" */
	unsigned char workSpace[WORKSPACE]; 
	PROCESS_STATES state;
	voidfuncptr  code;   /* function to be executed as a task */
	KERNEL_REQUEST_TYPE request;
} PD;

/**
 * This table contains ALL process descriptors. It doesn't matter what
 * state a task is in.
 */
static PD Process[MAXTHREAD];

/**
 * The process descriptor of the currently RUNNING task.
 */
volatile static PD* Cp; 

/** 
 * Since this is a "full-served" model, the kernel is executing using its own
 * stack. We can allocate a new workspace for this kernel stack, or we can
 * use the stack of the "main()" function, i.e., the initial C runtime stack.
 * (Note: This and the following stack pointers are used primarily by the
 *   context switching code, i.e., CSwitch(), which is written in assembly
 *   language.)
 */         
volatile unsigned char *KernelSp;

/**
 * This is a "shadow" copy of the stack pointer of "Cp", the currently
 * running task. During context switching, we need to save and restore
 * it into the appropriate process descriptor.
 */
volatile unsigned char *CurrentSp;

/** index to next task to run */
volatile static unsigned int NextP;  

/** 1 if kernel has been started; 0 otherwise. */
volatile static unsigned int KernelActive;  

/** number of tasks created so far */
volatile static unsigned int Tasks;  


typedef struct td_struct task_descriptor_t;
/**
 * @brief All the data needed to describe the task, including its context.
 */
struct td_struct
{
	/** The stack used by the task. SP points in here when task is RUNNING. */
	uint8_t                         stack[WORKSPACE];
	/** A variable to save the hardware SP into when the task is suspended. */
	uint8_t*						volatile sp;   /* stack pointer into the "workSpace" */
	/** PERIODIC tasks need a name in the PPP array. */
	uint8_t                         name;
	/** The state of the task in this descriptor. */
	PROCESS_STATES                  state;
	/** The argument passed to Task_Create for this task. */
	int                             arg;
	/** The priority (type) of this task. */
	uint8_t                         level;
	/** A link to the next task descriptor in the queue holding this task. */
	task_descriptor_t*              next;
};


/**
 * @brief Contains pointers to head and tail of a linked list.
 */
typedef struct
{
	/** The first item in the queue. NULL if the queue is empty. */
	task_descriptor_t*  head;
	/** The last item in the queue. Undefined if the queue is empty. */
	task_descriptor_t*  tail;
} queue_t;

#endif
