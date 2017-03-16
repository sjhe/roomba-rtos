/*
 * Tests periodic tasks
 *
 * PERIODIC task should NOT be preempted by the waking RR task.
 */
#include <util/delay.h>
#include <avr/io.h>

#include <string.h>
#include "../src/os.h"
#include "../src/led_test.h"

void setup () {
	init_LED_ON_BOARD();
	init_LED_PING();
	init_LED_ISR();
	init_pin(PIN10);
}

void test_signal() {
	int arg = Task_GetArg();
	for (;;) {
		if (arg == 0)
		{
			led_toggle(LED_ON_BOARD);
			led_toggle(LED_ON_BOARD);
		}
		else if (arg == 1)
		{
			led_toggle(LED_PING);
			led_toggle(LED_PING);
		}
		else if (arg == 2)
		{
			led_toggle(LED_ISR);
			led_toggle(LED_ISR);
		}
		else if (arg == 3)
		{
			led_toggle(PIN10);
			led_toggle(PIN10);
		}

		Task_Next();
	}
}

void a_main() {
	setup();

	// set_error_handler(err_handler);
	Task_Create_Period(test_signal, 0, 10, 5, 0);
	Task_Create_Period(test_signal, 1, 20, 5, 23);
	Task_Create_Period(test_signal, 2, 30, 5, 17);
	//Task_Create_Period(test_signal, 3, 40, 5, 1);
	Task_Create_RR(test_signal, 3);
}
