/**
 * @file   test_2.c
 * @author 
 * @date   
 * 
 * @brief  Test 002 - Send /Recv
 * 
 */
#include "kernel.h"
#include "os.h"
#include "led_test.h"


volatile CHAN channel_id = 0;

void setup () {
	init_LED_ON_BOARD();
	init_LED_PING();
	init_LED_ISR();
}
/*============
 * A Simple Test 
 * ============
 */

void Ping() 
{
	int index = 0;
	for(;;){
		index++;
		index = index % 1000;

		Send(channel_id, index);
		Task_Next();
	}
}
// Pong
void Pong() 
{
	int index = 0;
	// disable_LEDs();
	for(;;){
		index = Recv(channel_id);
		if(index == 0){
			led_toggle(LED_PING);
		} 

		Task_Next();
	}
}

void Pang() 
{
	int index = 0;
	// disable_LEDs();
	for(;;){
		index = Recv(channel_id);
		if(index == 0){
			led_toggle(LED_ON_BOARD);
		}

		Task_Next();
	}
}

void a_main(void)
{
	setup();
	channel_id = Chan_Init();

	Task_Create_System( Pang, 1 );
	Task_Create_System( Pong, 1 );
	Task_Create_System( Ping, 2 );
}
