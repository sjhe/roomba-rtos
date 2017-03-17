/*
 * 	Test Case : This test should casue an OS_Abort() if the number of 
 * 	channels reaches MAXCHAN 
 * 
 */

#include "../src/os.h"
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

	for (i = 0 ; i<= MAXCHAN; i++) {
		Chan_Init();// it should break after 16
	}

}
