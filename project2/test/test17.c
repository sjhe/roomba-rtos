/*
 * 
 *
 * 
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
		Send(test_channel, 1 );
		add_trace(arg, EXIT);
		Send(print_channel,1 );
		Task_Next();
	}
}

void test_waiting() {
	int arg = Task_GetArg();
	int value = NULL;
	for (;;) {
		add_trace(arg, ENTER);
		value = Recv(test_channel);
		if(value > 0){
			add_trace(arg, EXIT);
			Task_Next();
		}

	}
}

void test_results() {
	int value = Recv(print_channel);
	if(value > 0){
		char * trace = get_trace();
		char * correct_trace = "(0,(1,1),0),(1,";
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
	// set_error_handler(err_handler);
	print_channel = Chan_Init();
	test_channel  = Chan_Init();
	Task_Create_System(test_results, 0);

	UART_print("\ntest begin\n");

	Task_Create_System(test_signal, 0);
	Task_Create_System(test_waiting, 1);
}
