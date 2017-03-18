/*
 * Test for multiple receivers that are receiving from the same channel
 *
 * 
 */

#include "../trace/trace.h"

#include <util/delay.h>
#include <avr/io.h>

#include <string.h>

CHAN * print_channel;
CHAN * test_channel;

void test_Send() {
	int arg = Task_GetArg();
	for (;;) {
		add_trace(arg, ENTER);
		Send(test_channel,1 );
		add_trace(arg, EXIT);
		Task_Next();
	}
}

void test_Recv() {
	int arg = Task_GetArg();
	int value = NULL;
	for (;;) {
		add_trace(arg, ENTER);
		value = Recv(test_channel);
		add_trace(arg, EXIT);
		if(arg == 3 ){
			Send(print_channel,value);	
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
		char * correct_trace = "(1,(2,(3,(4,4),1),2),3),";
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

	Task_Create_RR(test_Recv, 1);
	Task_Create_RR(test_Recv, 2);
	Task_Create_RR(test_Recv, 3);

	Task_Create_RR(test_Send, 4);

}
