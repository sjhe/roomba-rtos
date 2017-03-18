/* Test Case: This is a simple test that checks the system abort functionality when the periodic task exceeds the wcet. 
*  Code: test13.c
*  Description: In this test case there is one periodic task with a wcet of 10 ms. 
*  The task itself is a simple function that calls _delay_ms(20)
*  which will delay the function by 20 ms before calling Task_Next().
*  Running this test causing the PERIODIC_TASK_EXCEEDS_WCET error code to be generated.
*/
#include "../trace/trace.h"

#include <util/delay.h>
#include <avr/io.h>

#include <string.h>

CHAN * print_channel;
CHAN * test_channel;

void test_signal() {
	int arg = Task_GetArg();
	for (;;) {
		add_trace(arg, ENTER);
		Send(test_channel,1 );
		add_trace(arg, EXIT);
		Task_Next();
	}
}

void test_waiting() {
	int arg = Task_GetArg();
	int value = NULL;
	for (;;) {
		add_trace(arg, ENTER);
		value = Recv(test_channel);
		add_trace(arg, EXIT);

		Send(print_channel,value);
		Task_Next();
	}
}


void test_results() {
	int value = Recv(print_channel);
	if(value > 0){
		char * trace = get_trace();
		char * correct_trace = "(1,(2,1),(1,2),";
		UART_print("Trace: %s\n", trace);
		if (strcmp(correct_trace, trace) == 0) {
			UART_print("pass");
		} else {
			UART_print("fail");
		}
	}
}

void a_main() {
	UART_Init0(38400);

	print_channel = Chan_Init();
	test_channel  = Chan_Init();
	Task_Create_System(test_results, 0);

	UART_print("\ntest begin\n");
	Task_Create_RR(test_waiting, 2);
	Task_Create_System(test_signal, 1);
}
