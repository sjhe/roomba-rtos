/*
 * 	Test Case : Test failure when create periodic task with a wcet greater than period 
 * 
 */

#include "../trace/trace.h"

#include <util/delay.h>
#include <avr/io.h>

#include <string.h>

void test_periodic() {
	for (;;)
	{
		Task_Next();
	}
}

void a_main() {
	// wcet greater than period..
	Task_Create_Period(test_periodic, 0, 10, 11, 0);
}
