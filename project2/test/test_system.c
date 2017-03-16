/*
 * Tests that the RTOS correctly schedules system tasks.
 */

#include "../trace/trace.h"

#include <util/delay.h>
#include <avr/interrupt.h>

#include <string.h>

int count = 0;

void task(void) {
	int arg = 0;
	arg = Task_GetArg();

	for (;;) {
		add_trace(arg, ENTER);
		add_trace(arg, EXIT);
		Task_Next();
	}
}

void err_handler() {
	UART_print("fail");
}

void test_results() {
	char * trace = get_trace();
	char * correct_trace = "(0,0),(1,1),(2,2),(3,3),(4,4),(5,5),(6,6),(7,7),";
	UART_print("Trace: %s\n", trace);
	if (strcmp(correct_trace, trace) == 0) {
		UART_print("pass");
	} else {
		UART_print("fail");
	}
}

void a_main() {
	UART_Init0(38400);

	UART_print("\ntest begin\n");

	Task_Create_System(task, 0);
	Task_Create_System(task, 1);
	Task_Create_System(task, 2);
	Task_Create_System(task, 3);
	Task_Create_System(task, 4);
	Task_Create_System(task, 5);
	Task_Create_System(task, 6);
	Task_Create_System(task, 7);
	Task_Create_System(test_results, 0);
}
