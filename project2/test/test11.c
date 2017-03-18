/*Test Case: This is a simple test that checks the system abort functionality when the number of tasks in the system is the same as MAXTHREAD defined in the API document provided for the class. 
filename: test11.c
Comment: The arduino ATmega2560 has an built in on board led.
When the system encounters an error code it will flash its 
led according to its error code. 
In this test, the system will blink on and off for 5 times*/

#include "../trace/trace.h"

#include <util/delay.h>
#include <avr/io.h>

#include <string.h>

CHAN * print_channel;
CHAN * test_channel;

void test_signal() {
	for(;;);
}

void a_main() {
	int i;
	init_LED_ON_BOARD();

	for (i = 0 ; i< 14; i++) {
		Task_Create_System(test_signal, i);
	}

}
