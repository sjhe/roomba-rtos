#include "../src/os.h"
#include <avr/io.h>

// EXPECTED RUNNING ORDER: RR1,RR2,RR3,RR4...RR1,RR2,RR3
// 
// RR1 block on recv		  Recv
// RR2 		block on recv	  Recv
// RR3 			delay...................block on recv
// RR4			 	delay.....Send
CHAN c1=0;

void DEBUG_INIT() {
	DDRB |= (1<<PB1);	//pin 52
	DDRB |= (1<<PB2);	//pin 51	
	DDRB |= (1<<PB3);	//pin 50
}

void DEBUG_ON(int num) {
	switch (num) {
		case 1:
			PORTB |= (1<<PB1);
			break;
		case 2:
			PORTB |= (1<<PB2);
			break;
		case 3:
			PORTB |= (1<<PB3);
			break;
		default:
			break;
	}
}

void DEBUG_OFF(int num) {
	switch (num) {
		case 1:
			PORTB &= ~(1<<PB1);
			break;
		case 2:
			PORTB &= ~(1<<PB2);
			break;
		case 3:
			PORTB &= ~(1<<PB3);
			break;
		default:
			break;
	}
}

void Task_RR1()
{
	Recv(c1);
	DEBUG_ON(1);
	for(;;);
}
void Task_RR2()
{
	Recv(c1);
	DEBUG_ON(2);
	for(;;);
}
void Task_RR3()
{
	while(Now() < 100) {
		Task_Next();
	}
	Recv(c1);
	//Should never run.
	DEBUG_OFF(1);
	DEBUG_OFF(2);
	DEBUG_OFF(3);
	for(;;);
}
void Task_RR4()
{
	while(Now() < 50) {
		Task_Next();
	}
	Send(c1, 0);
	DEBUG_ON(3);
	for(;;);
}

void a_main()
{
	DEBUG_INIT();
	Task_Create_RR(Task_RR1,0);
	Task_Create_RR(Task_RR2,0);
	Task_Create_RR(Task_RR3,0);
	Task_Create_RR(Task_RR4,0);	
	
	c1 = Chan_Init();
}
