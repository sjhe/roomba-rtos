/*
 * 	Test Case : Test failure when two senders send on same channel
 * 
 */

#include "../trace/trace.h"

#include <util/delay.h>
#include <avr/io.h>

#include <string.h>

CHAN ch = 0;

void test_sender() {
	Send(ch, 1);
}

void a_main() {
	init_LED_ON_BOARD();
	ch = Chan_Init();
	Task_Create_System(test_sender, 0);
	//Task_Create_System(test_sender, 1);
}
