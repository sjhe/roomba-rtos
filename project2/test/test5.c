/**
 * @file   test5.c
 * @author 
 * @date   
 *
 * @brief  Test the correctness and order of the periodic tasks
 */


#include "../trace/trace.h"

#include <util/delay.h>
#include <avr/io.h>

#include <string.h>

CHAN* print_channel;
CHAN* test_channel;

void test_signal() {
	int arg = Task_GetArg();
	for (;;) {
		add_trace(arg, ENTER);
		add_trace(arg, EXIT);
		if (arg == 1)
		{
			Send(print_channel, 1);
		}

		Task_Next();
	}
}

void err_handler() {
	UART_print("fail");
}

void test_results() {
	int value = Recv(print_channel);
	if(value > 0){
		char * trace = get_trace();
		char * correct_trace = "(0,0),(4,4),(4,4),(0,0),(2,2),(4,4),(4,4),(0,0),(1,1),";
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

	Task_Create_Period(test_signal, 0, 10, 1, 0);
	Task_Create_Period(test_signal, 1, 20, 1, 22);
	Task_Create_Period(test_signal, 2, 30, 1, 12);
	// Task_Create_Period(test_signal, 3, 40, 1, 1);
	Task_Create_Period(test_signal, 4, 5 , 1, 3);
}
