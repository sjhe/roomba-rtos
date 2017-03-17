/*
 * 	Test Case : This test should casue an OS_Abort() if the number of 
 * 	tasks reaches MAXTHREAD 
 * 
 */

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
		Task_Create_System(test_signal, 1);
	}

}
