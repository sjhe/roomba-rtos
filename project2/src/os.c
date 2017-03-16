#include "os.h"
#include "kernel.h"

#include "led_test.h"

#define OCR_MAX_VAL   625

#define Disable_Interrupt()   asm volatile ("cli"::)
#define Enable_Interrupt()    asm volatile ("sei"::)

#include <string.h>
#include <stdio.h>

extern void Idle();
/** @brief a_main function provided by user application. The first task to run. */
extern void a_main();

/* Kernel functions */
static void Kernel_Handle_Request(void);
static void Kernel_Main_Loop(void);
static void Kernel_Dispatch(void);

static CHAN Kernel_Chan_Init();
static void Kernel_Send();
static void Kernel_Recv();
static void Kernel_Write();

/* Context Switching*/
extern void Exit_Kernel();    /* this is the same as CSwitch() */
extern void CSwitch();
extern void Enter_Kernel();

/* Task management  */
static void Kernel_Create_Task(void);
void Task_Terminate();
void Task_Create(voidfuncptr f, int arg, uint8_t level);
static void EnqueueTaskToStateQueue(PD* process);

/*ISR Management*/
static void init_tick_timer();

PD* idle_process;

static volatile PD new_task_args;
static volatile CH channel_buffer;

static queue_t system_queue;
static queue_t rr_queue;
static queue_t periodic_queue;

// kernel request values
static volatile CHAN kernel_channel_retval;

// The number of elapsed Ticks since OS_Init()
volatile unsigned int num_ticks = 0;

/*
 * FUNCTIONS
 */
 /**
  *  @brief The Idle task does nothing but busy loop.
  */

/**
 * When creating a new task, it is important to initialize its stack just like
 * it has called "Enter_Kernel()"; so that when we switch to it later, we
 * can just restore its execution context on its stack.
 * (See file "cswitch.S" for details.)
 */
void Kernel_Create_Task_At(PD* p)
{
	unsigned char* sp;
	//Changed -2 to -1 to fix off by one error.
	sp = (unsigned char*)&(p->workSpace[WORKSPACE - 1]);

	/*----BEGIN of NEW CODE----*/
	//Initialize the workspace (i.e., stack) and PD here!
	//Clear the contents of the workspace
	memset(&(p->workSpace), 0, WORKSPACE);

	//Notice that we are placing the address (16-bit) of the functions
	//onto the stack in reverse byte order (least significant first, followed
	//by most significant).  This is because the "return" assembly instructions 
	//(rtn and rti) pop addresses off in BIG ENDIAN (most sig. first, least sig. 
	//second), even though the AT90 is LITTLE ENDIAN machine.

	//Store terminate at the bottom of stack to protect against stack underrun.
	*(unsigned char *)sp-- = ((unsigned int)Task_Terminate) & 0xff;
	*(unsigned char *)sp-- = (((unsigned int)Task_Terminate) >> 8) & 0xff;
	*(unsigned char *)sp-- = 0;
	//Place return address of function at bottom of stack
	*(unsigned char *)sp-- = ((unsigned int)new_task_args.code) & 0xff;
	*(unsigned char *)sp-- = (((unsigned int)new_task_args.code) >> 8) & 0xff;

	*(unsigned char *)sp-- = 0;
	//Place stack pointer at top of stack
	sp = sp - 34;

	p->sp = sp;    /* stack pointer into the "workSpace" */
	p->code = new_task_args.code;		/* function to be executed as a task */
	p->arg  = new_task_args.arg;		/* function to be executed as a task */
	p->request = NONE;
	p->state = READY;
	p->level = new_task_args.level;

	/* ---- Need to add switch statement for handling ----
	 * ---- PERIODIC | SYSTEM | RR                    ----
	 */
	switch (new_task_args.level) 
	{
	case SYSTEM:
		Enqueue(&system_queue, p);
		break;
	case RR:
		Enqueue(&rr_queue, p);
		break;
	case PERIODIC:
		p->period 				 =  new_task_args.period;
		p->wcet						 =  new_task_args.wcet;
		p->next_start			 =  new_task_args.next_start;
		p->ticks_remaining =  new_task_args.ticks_remaining ;
		EnqueuePeriodic(&periodic_queue , p);
		break;
	case IDLE:
		idle_process = p;
		break;
	default:
		break;
	}
}


/**
  *  Create a new task
  */
static void Kernel_Create_Task()
{
	int x;
	if (Tasks == MAXTHREAD) 
	{
		OS_Abort(MAX_THREADS_REACHED);
		return;  /* Too many task! */
	}
	 /* find a DEAD PD that we can use  */

	for (x = 0; x < MAXTHREAD; x++) 
	{
		if (Process[x].state == DEAD) break;
	}

	++Tasks;
	Kernel_Create_Task_At(&(Process[x]));
}

/**
  * This internal Kernel function is a part of the "scheduler". It chooses the
  * next task to run, i.e., Cp.
  */
static void Kernel_Dispatch()
{
	// find the next READY task 
	if (Cp->state != RUNNING || Cp == idle_process)
	{
		if (system_queue.head != NULL)
		{
			Cp = Dequeue(&system_queue);
		}
		else if(periodic_queue.head != NULL && (int)(num_ticks - periodic_queue.head->next_start) >= 0)
		{
			// Timing conflict with another periodic task
			if (periodic_queue.head->next != NULL && (int)(num_ticks - periodic_queue.head->next->next_start) >= 0)
			{
				OS_Abort(PERIODIC_TASK_TIMING_CONFLICT);
			}	
			Cp = periodic_queue.head;						
		}
		else if (rr_queue.head != NULL)
		{
			Cp = Dequeue(&rr_queue);
		}
		else 
		{
			Cp = idle_process;
		}
		CurrentSp = Cp->sp;
		Cp->state = RUNNING;
	}
}

/**
  * This internal Kernel function is the "main" driving loop of this full-served
  * model architecture. Basically, on OS_Start(), the Kernel repeatedly
  * requests the next user task's next system call and then invokes the
  * corresponding Kernel function on its behalf.
  *
  * This is the main loop of our Kernel, called by OS_Start().
  */
static void Kernel_Main_Loop()
{
	Kernel_Dispatch();  /* select a new task to run */

	while (1) 
	{
		Cp->request = NONE; /* clear its request */

		 /* activate this newly selected task */
		CurrentSp = Cp->sp;

		Exit_Kernel();    /* or CSwitch() */

		/* if this task makes a system call, it will return to here! */

		/* save the Cp's stack pointer */
		Cp->sp = CurrentSp;

		Kernel_Handle_Request();
	}
}

/** el*
 * Also the first part of the scheduler
 * Determines whether current process should be
 * in the ready or waiting queue
 */
static void Kernel_Handle_Request(void)
{
	switch (Cp->request) 
	{
	case CREATE:
		Kernel_Create_Task();
		break;
	case TIMER_TICK:
		switch (Cp->level) {
			case SYSTEM: // drop down
				Cp->state = RUNNING;
				break;
			case IDLE:
				break;
			case PERIODIC: // drop down
				Cp->state = READY;
				Cp->ticks_remaining--;
				if (Cp->ticks_remaining <= 0) {
					//errno = ERRNO_PERIODIC_TASK_EXCEEDS_WCET;
					OS_Abort(PERIODIC_TASK_EXCEEDS_WCET);
				}
				break;
			case RR:
				Cp->state = READY;
				Enqueue(&rr_queue, Cp);
				break;
			// case RR:
			// 	Cp->ticks_remaining--;
			// 	if (Cp->ticks_remaining == 0) {
			// 		// Reset ticks and move to back
			// 		Cp->ticks_remaining = RR_TICK;
			// 		proc_list_append(&rr_procs, proc_list_pop(&rr_procs));
			// 	}
			// 	break;
		}
		Kernel_Dispatch();
		break;
	case NEXT:
	case NONE:
		/* NONE could be caused by a timer interrupt */
		// if the current process was a system level task, then set state back to ready and put task back into system queue
		if (Cp->level == SYSTEM)
		{
			Cp->state = READY;
			Enqueue(&system_queue, Cp);
		}else if(Cp->level == PERIODIC)
		{

			Dequeue(&periodic_queue);
			Cp->state = READY;
			Cp->next_start = Cp->next_start + Cp->period;

			Cp->ticks_remaining  = Cp->wcet ;
			EnqueuePeriodic(&periodic_queue, Cp);
			// Enqueue(&periodic_queue, Cp);
		}
		else if (Cp->level == RR)
		{
			Cp->state = READY;
			Enqueue(&rr_queue, Cp);
		}
		Kernel_Dispatch();
		break;
	case TERMINATE:
		/* deallocate all resources used by this task */
		Cp->state = DEAD;
		// Cp = Dequeue(&system_queue);
		Kernel_Dispatch();
		break;
	case CREATE_CHANNEL:
		kernel_channel_retval = Kernel_Chan_Init();
		break;
	case SEND:
		Kernel_Send();
		break;
	case RECV:
		Kernel_Recv();
		break;
	case WRITE:
		Kernel_Write();
		break;
	default:
		/* Houston! we have a problem here! */
		break;
	}
}

/*================
  * RTOS  API  and Stubs
  *================
  */

  /**
	* This function initializes the RTOS and must be called before any other
	* system calls.
	*/
void OS_Init()
{
	int x;
	Tasks = 0;
	KernelActive = 0;
	//Reminder: Clear the memory for the task on creation.
	//Init Kernel data structures
	for (x = 0; x < MAXTHREAD; x++) 
	{
		memset(&(Process[x]), 0, sizeof(PD));
		Process[x].state = DEAD;
	}

	// Init channel data structure
	for (x = 0; x < MAXCHAN; x++) 
	{
		memset(&(Channels[x]), 0, sizeof(CH));
		Channels[x].id = 0;
		Channels[x].sender = NULL;
		Channels[x].val = 0;
	}
	
	// initialize starting state to ready so that dispatch can be run..
	Cp->state = READY;
	// create idle process
	Task_Create_Idle(Idle, 2);
	Task_Create_System(a_main, 1);
}


/**
  * This function starts the RTOS after creating a few tasks.
  */
void OS_Start()
{
	if ((!KernelActive) && (Tasks > 0)) {
		Disable_Interrupt();
		/* we may have to initialize the interrupt vector for Enter_Kernel() here. */
		/* here we go...  */
		KernelActive = 1;
		init_tick_timer();

		Kernel_Main_Loop();   /* main loop of the Kernel*/
	}
}

void OS_Abort(unsigned int error)
{
	int i = 0;
	switch (error)
	{
		case PERIODIC_TASK_EXCEEDS_WCET:
			// blink 5 times really fast
			for (i = 0; i < 6; i++)
			{
				led_toggle(LED_ON_BOARD);
				_delay_ms(50);
			}
			break;
		case PERIODIC_TASK_TIMING_CONFLICT:
			// blink on and off 3 times with a period of 1 second
			for (i = 0; i <= 6; i++)
			{
				led_toggle(LED_ON_BOARD);
				_delay_ms(1000);
			}
			break;
		case SENDER_CONFLICT:
			// blink 5 times with a period of ~500 ms
			for (i = 0; i < 6; i++)
			{
				led_toggle(LED_ON_BOARD);
				_delay_ms(10);
				led_toggle(LED_ON_BOARD);
				_delay_ms(500);
			}
			break;
		case MAX_THREADS_REACHED:
			// blink on->off->on in short bursts 5 times
			for (i = 0; i < 6; i++)
			{
				led_toggle(LED_ON_BOARD);
				_delay_ms(50);
				led_toggle(LED_ON_BOARD);
				_delay_ms(50);
				led_toggle(LED_ON_BOARD);
				_delay_ms(200);
			}
			break;
		case MAX_CHANNELS_CREATED:
			// blink on and off 5 times with a period of 1 second
			for (i = 0; i < 6; i++)
			{
				led_toggle(LED_ON_BOARD);
				_delay_ms(50);
				led_toggle(LED_ON_BOARD);
				_delay_ms(1000);
			}

			break;
		default:
			break;
	}
}

/*
 * Task management.
 */
PID Task_Create_Idle(void(*f)(void), int arg) {
	Task_Create(f, arg, IDLE);
};

PID Task_Create_System(void(*f)(void), int arg) {
	Task_Create(f, arg, SYSTEM);
};

PID Task_Create_RR(void(*f)(void), int arg) {
	Task_Create(f, arg, RR);
};

PID Task_Create_Period(void(*f)(void), int arg, TICK period, TICK wcet, TICK offset) {
	new_task_args.code = f;
	new_task_args.arg = arg;
	new_task_args.level = PERIODIC;
	new_task_args.period = period;
	new_task_args.wcet = wcet;
	new_task_args.ticks_remaining = wcet;
	new_task_args.next_start = offset;

	if (KernelActive) 
	{
		Disable_Interrupt();
		Cp->request = CREATE;
		Enter_Kernel();
	}
	else 
	{
		Cp->request = NONE;
		Kernel_Create_Task();
	}
};

int Task_GetArg(void)
{
	return Cp->arg;
}

/**
  * For this example, we only support cooperatively multitasking, i.e.,
  * each task gives up its share of the processor voluntarily by calling
  * Task_Next().
  */
void Task_Create(voidfuncptr f, int arg, uint8_t level)
{
	if (KernelActive) 
	{
		Disable_Interrupt();
		new_task_args.code = f;
		new_task_args.arg = arg;
		new_task_args.level = (uint8_t)level;

		Cp->request = CREATE;
		Enter_Kernel();
	}
	else 
	{
		new_task_args.code = f;
		new_task_args.arg = arg;
		new_task_args.level = (uint8_t)level;
		Cp->request = NONE;

		Kernel_Create_Task();
	}
}

/**
  * The calling task gives up its share of the processor voluntarily.
  */
void Task_Next()
{
	if (KernelActive) 
	{
		Disable_Interrupt();
		Cp->request = NEXT;
		Enter_Kernel();
	}
}

/**
  * The calling task terminates itself.
  */
void Task_Terminate()
{
	if (KernelActive) 
	{
		Disable_Interrupt();
		Tasks--;
		Cp->request = TERMINATE;
		Enter_Kernel();

		/* never returns here! */
	}
}

void init_tick_timer() {
  //Clear timer config.
  TCCR3A = 0;      // Timer 3 A
  TCCR3B = 0;      // Timer 3 B
  //Set to CTC (mode 4)
  TCCR3B |= (1<<WGM32);
  //Set prescaller to 256
  TCCR3B |= (1<<CS32);
  //Set TOP value (1 milisecond)
  OCR3A = OCR_MAX_VAL; // or (TICK * (F_CPU / 1024 ) / 1000)
  //Enable interupt A for timer 3.
  TIMSK3 |= (1<<OCIE3A);
  //Set timer to 0 (optional here).
  TCNT3 = 0;
}


/**
  * Interrupt service routine
  */
ISR(TIMER3_COMPA_vect) {
	Disable_Interrupt();
	num_ticks++;
	Cp->request = TIMER_TICK;
	Enter_Kernel();
}

/**
  * CHANNEL stuff
  */
static CHAN Kernel_Chan_Init()
{
	int x;
	// find empty channel
	for (x = 0; x < MAXCHAN; x++) 
	{
		if (Channels[x].id == 0)
		{
			// initialize channel
			Channels[x].id = x + 1;
			Channels[x].sender = NULL;
			Channels[x].receivers.head = NULL;
			Channels[x].receivers.tail = NULL;
			Channels[x].val = 0;
			break;
		}
	}

	// No empty channels..
	if (x == MAXCHAN) 
	{
		OS_Abort(MAX_CHANNELS_CREATED);	
		return NULL;
	}
	return Channels[x].id;
}

static void EnqueueTaskToStateQueue(PD* process)
{
	switch (process->level)
	{
	case SYSTEM:
		process->state = READY;
		Enqueue(&system_queue, process);
		break;
	case RR:
		process->state = READY;
		Enqueue(&rr_queue, process);
		break;
	default:
		break;
	}
}

CHAN Chan_Init()
{
	if (KernelActive)
	{
		Disable_Interrupt();
		Cp->request = CREATE_CHANNEL;
		Enter_Kernel();
	}
	else 
	{
		kernel_channel_retval = Kernel_Chan_Init();
	}

	return kernel_channel_retval;
}

static void Kernel_Send()
{
	CH* channel_ptr = &(Channels[channel_buffer.id]);
	// if no receivers waiting then add Cp as sender and block
	if (channel_ptr->receivers.head == NULL)
	{
		// if sender already exists.. then error
		if (channel_ptr->sender != NULL)
		{
			// ABORT!!
			OS_Abort(SENDER_CONFLICT);
		}
		channel_ptr->sender = Cp;
		Cp->state = WAITING;
		channel_ptr->val = channel_buffer.val;
		Kernel_Dispatch();
	}
	else 
	{
		// if there are receivers then give all the receivers their value
		PD* recv_process = Dequeue(&(channel_ptr->receivers));
		while (recv_process != NULL)
		{
			recv_process->state = READY;
			recv_process->retval = channel_buffer.val;

			// enqueue revc process back into its respective queue
			EnqueueTaskToStateQueue(recv_process);

			recv_process = Dequeue(&(channel_ptr->receivers));
		}
		channel_ptr->receivers.head = NULL;
		channel_ptr->sender = NULL;
	}

}

void Send(CHAN ch, int v)
{
	if (KernelActive)
	{
		Disable_Interrupt();
		Cp->request = SEND;
		channel_buffer.id = ch;
		channel_buffer.val = v;
		Enter_Kernel();
	}
}

static void Kernel_Recv()
{
	CH* channel_ptr = &(Channels[channel_buffer.id]);
	// if there is no sender then add to wait queue and block
	if (channel_ptr->sender == NULL)
	{
		Cp->state = WAITING;
		Cp->retval = NULL;
		Enqueue(&(channel_ptr->receivers), Cp);
		Kernel_Dispatch();
	}
	else 
	{
		// there is a sender then grab the value and set sender state back to ready
		Cp->retval = channel_ptr->val;
		channel_ptr->sender->state = READY;
	
		// enqueue sender back into its task queue
		EnqueueTaskToStateQueue(channel_ptr->sender);

		channel_ptr->receivers.head = NULL;
		channel_ptr->sender = NULL;
	}
}

int Recv(CHAN ch)
{
	if (KernelActive)
	{
		Disable_Interrupt();
		Cp->request = RECV;
		channel_buffer.id = ch;
		channel_buffer.val = 0;
		Enter_Kernel();
	}

	return Cp->retval;
}

static void Kernel_Write()
{
	CH* channel_ptr = &(Channels[channel_buffer.id]);
	// if there are receivers then give all the receivers their value
	if (channel_ptr->receivers.head != NULL)
	{
		PD* recv_process = Dequeue(&(channel_ptr->receivers));
		while (recv_process != NULL)
		{
			recv_process->state = READY;
			recv_process->retval = channel_buffer.val;

			// enqueue revc process back into its respective queue
			EnqueueTaskToStateQueue(recv_process);

			recv_process = Dequeue(&(channel_ptr->receivers));
		}
		channel_ptr->receivers.head = NULL;
		channel_ptr->sender = NULL;
	}
}

void Write(CHAN ch, int v)
{
	if (KernelActive)
	{
		Disable_Interrupt();
		Cp->request = WRITE;
		channel_buffer.id = ch;
		channel_buffer.val = v;
		Enter_Kernel();
	}
}

unsigned int Now()
{

}

  /**
	* This function creates two cooperative tasks, "Ping" and "Pong". Both
	* will run forever.
	*/
void main()
{
	OS_Init();
	OS_Start();
}

