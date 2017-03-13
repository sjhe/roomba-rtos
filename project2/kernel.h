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


#define DEBUG 1
 
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
/**
 * It is important to keep the order of context saving and restoring exactly
 * in reverse. Also, when a new task is created, it is important to
 * initialize its "initial" context in the same order as a saved context.
 *
 * Save r31 and SREG on stack, disable interrupts, then save
 * the rest of the registers on the stack. In the locations this macro
 * is used, the interrupts need to be disabled, or they already are disabled.
 */



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

typedef enum system_levels
{
	RR = 1,
	PERIODIC,
	SYSTEM,
	IDLE
} SYSTEM_LEVELS;

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
	TIMER_TICK
} KERNEL_REQUEST_TYPE;



typedef struct process_struct PD;
/**
 * @brief All the data needed to describe the task, including its context.
 */
struct process_struct
{
	/** The stack used by the task. SP points in here when task is RUNNING. */
	uint8_t                 workSpace[WORKSPACE];
	/** A variable to save the hardware SP into when the task is suspended. */
	volatile uint8_t*		sp;   /* stack pointer into the "workSpace" */
	/** PERIODIC tasks need a name in the PPP array. */
	uint8_t                 name;
	/** The state of the task in this descriptor. */
	PROCESS_STATES          state;
	/** The argument passed to Task_Create for this task. */
	int                     arg;
	/** The priority (type) of this task. */
	uint8_t                 level;
	/** A link to the next task descriptor in the queue holding this task. */
	voidfuncptr  						code;   /* function to be executed as a task */
	// The tick number for when the next
	uint32_t							  next_start;

	TICK 										wcet;

	TICK 										ticks_remaining;

	TICK 										period;

	KERNEL_REQUEST_TYPE			request;

	PD*											next;
};
/**
 * Each task is represented by a process descriptor, which contains all
 * relevant information about this task. For convenience, we also store
 * the task's stack, i.e., its workspace, in here.
 */
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

/**
 * @brief Contains pointers to head and tail of a linked list.
 */
typedef struct
{
	/** The first item in the queue. NULL if the queue is empty. */
	PD*  head;
	/** The last item in the queue. Undefined if the queue is empty. */
	PD*  tail;
} queue_t;


/*
* Queue manipulation.
*/

/**
* @brief Add a task the head of the queue
*
* @param queue_ptr the queue to insert in
* @param task_to_add the task descriptor to add
*/
static void Enqueue(queue_t* queue_ptr, PD* p)
{
	p->next = NULL;

	if (queue_ptr->head == NULL)
	{
		/* empty queue */
		queue_ptr->head = p;
		queue_ptr->tail = p;
	}
	else
	{
		/* put task at the back of the queue */
		queue_ptr->tail->next = p;
		queue_ptr->tail = p;
	}
}

/**
* @brief Add a task based on its next start time
*
* @param queue_ptr the queue to insert in
* @param task_to_add the task descriptor to add
*/
static void EnqueuePeriodic(queue_t* queue_ptr, PD* p)
{
	if (queue_ptr->head == NULL)
	{
		queue_ptr->head = queue_ptr->tail = p;
	}
	else
	{
		PD* cp_curr = queue_ptr->head;
		PD* cp_prev = NULL;

		while (cp_curr != NULL && cp_curr->next_start < p->next_start)
		{
			cp_prev = cp_curr;
			cp_curr = cp_curr->next;
		}

		// insert at head of queue
		if (cp_prev == NULL)
		{
			queue_ptr->head = p;
			p->next = cp_curr;
		}
		else if (cp_curr == NULL)
		{
			queue_ptr->tail = p;
			cp_prev->next = p;
			p->next = NULL;
		}
		else
		{
			cp_prev->next = p;
			p->next = cp_curr;
		}
	}
}


/**
* @brief Pops head of queue and returns it.
*
* @param queue_ptr the queue to pop
* @return the popped task descriptor
*/
static PD* Dequeue(queue_t* queue_ptr)
{
	PD* p = queue_ptr->head;

	if (queue_ptr->head != NULL)
	{
		queue_ptr->head = queue_ptr->head->next;
		p->next = NULL;
	}

	return p;
}


#endif
