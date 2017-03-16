/**
 * @file   test_2.c
 * @author 
 * @date   
 * 
 * @brief  Test 002 - Send /Recv
 * 
 */
#include "../src/kernel.h"
#include "../src/os.h"
#include "../src/led_test.h"


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
		Send(channel_id, index);
		index = Recv(channel_id);
		if(index == 1){
			led_toggle(LED_ON_BOARD);
		}
		else {
		}

	//	Task_Next();
	}
}

void Pang() 
{
	int index = 0;
	// disable_LEDs();
	for(;;){
		index = Recv(channel_id);
		if(index == 1){
			led_toggle(LED_PING);
		}

	//	Task_Next();
	}
}

void Peng()
{
	for (;;)
	{
		led_toggle(LED_PING);

		Task_Next();
	}
}

void a_main(void)
{
	setup();
	channel_id = Chan_Init();

//	Task_Create_System( Ping, 2 );
//	Task_Create_System( Pong, 1 );
//	Task_Create_System( Pang, 1 );

	Task_Create_Period( Peng, 2 , 10, 1, 0);
	Task_Create_Period( Peng, 2 , 10, 1, 1);

}
