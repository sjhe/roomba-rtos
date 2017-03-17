/*
 * 	Test Case : Test system failure when periodic tasks timing conflicts 
 * 
 */

#include "../trace/trace.h"

#include <util/delay.h>
#include <avr/io.h>

#include <string.h>

void test_periodic() {
	for (;;)
	{
		// delay greater than wcet of 10ms..,
		_delay_ms(5);
		Task_Next();
	}
}

void a_main() {
	// periodic task with 100ms period and 10ms wcet
	Task_Create_Period(test_periodic, 0, 10, 1, 0);
	Task_Create_Period(test_periodic, 0, 20, 1, 10);
}
