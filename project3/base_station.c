#include "src/os.h"
#include "src/led_test.h"
#include <avr/io.h>


void test_periodic(){
	for(;;){
		led_toggle(LED_ON_BOARD);
		Task_Next();
	}
}

void test_ping(){
	for(;;){
		led_toggle(LED_PING);
		Task_Next();
	}
}

void a_main()
{
	init_LED_ON_BOARD();

	init_LED_PING();

	Task_Create_Period(test_periodic, 0, 10, 1, 0);
	Task_Create_Period(test_ping, 0, 10, 1, 5);

}
