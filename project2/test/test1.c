#include "../src/os.h"
#include <avr/io.h>

// EXPECTED RUNNING ORDER: S1,S2,S3,P1,S1,S2,S3
// 
// S1 block on recv				Recv
// S2      block on recv		Recv
// S3           block on recv	Recv
// A_MAIN						Send
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


void Task_S1()
{
	int val = Recv(c1);
	if(val == 0) {
		DEBUG_ON(1);
		Task_Next();
		for(;;);
	}
}

void Task_S2()
{
	int val = Recv(c1);
	if(val == 0) {
		DEBUG_ON(2);
		Task_Next();
		for(;;);
	}	
}

void Task_S3()
{
	int val = Recv(c1);
	if(val == 0) {
		DEBUG_ON(3);
		for(;;);
	}	
}

void Task_RR1()
{
	Send(c1, 1);
	//Should never run.
	DEBUG_OFF(1);
	DEBUG_OFF(2);
	DEBUG_OFF(3);
	for(;;);
}

void a_main()
{
	DEBUG_INIT();
	Task_Create_System(Task_S1, 0);
	Task_Create_System(Task_S2, 0);
	Task_Create_System(Task_S3, 0);
	Task_Create_RR(Task_RR1,0);	
	c1 = Chan_Init();
	Task_Next();
	Write(c1, 0);
	// Send(c1,0);
}