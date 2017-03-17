/*
 * 	Test Case : Test system failure when periodic task exceeds wcet 
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
		_delay_ms(20);
		Task_Next();
	}
}

void a_main() {
	init_LED_ON_BOARD();

	// periodic task with 100ms period and 10ms wcet
	Task_Create_Period(test_periodic, 0, 10, 1, 0);
}
