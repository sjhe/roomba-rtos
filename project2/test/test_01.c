#include "../src/os.h"
#include "../src/kernel.h"
#include "../src/led_test.h"


//
// LAB - TEST 1
//	Noah Spriggs, Murray Dunne
//
//
// EXPECTED RUNNING ORDER: P1,P2,P3,P1,P2,P3,P1
//
// P1 sleep              lock(attept)            lock
// P2      sleep                     signal
// P3           lock wait                  unlock

CHAN  chan;

void Task_P1(int parameter)
{
	PORTB |= (1<<PB3);	
	_delay_ms(100); // sleep 100ms
	PORTB &= ~(1<<PB3);	//pin 50 off
	Task_Next();

	PORTB |= (1<<PB3);	
	Recv(chan);
	PORTB &= ~(1<<PB3);	//pin 50 off

	// Mutex_Lock(mut);
    for(;;);
}

void Task_P2(int parameter)
{
	PORTB |= (1<<PB2);	//pin 51 on
	_delay_ms(200); // sleep 200ms
	PORTB &= ~(1<<PB2);	//pin 51 off

	Task_Next();
	PORTB |= (1<<PB2);	//pin 51 on
	Send(chan, 1);
	PORTB &= ~(1<<PB2);	//pin 51 off

  for(;;);
}

void Task_P3(int parameter)
{
	PORTB |= (1<<PB1);	//pin 52 on
	PORTB &= ~(1<<PB1);	//pin 52 off

	Recv(chan);

	PORTB |= (1<<PB1);	//pin 52 on
	PORTB &= ~(1<<PB1);	//pin 52 off

  for(;;);
}

void a_main(int parameter)
{
	chan = Chan_Init();



	//Place these as necessary to display output if not already doing so inside the RTOS
	//initialize pins
	DDRB |= (1<<PB1);	//pin 52
	DDRB |= (1<<PB2);	//pin 51	
	DDRB |= (1<<PB3);	//pin 50
	
	
	// PORTB |= (1<<PB1);	//pin 52 on
	// PORTB |= (1<<PB2);	//pin 51 on
	// PORTB |= (1<<PB3);	//pin 50 on


	// PORTB &= ~(1<<PB1);	//pin 52 off
	// PORTB &= ~(1<<PB2);	//pin 51 off
	// PORTB &= ~(1<<PB3);	//pin 50 off



	Task_Create_System(Task_P1, 1);
	Task_Create_System(Task_P2, 2);
	Task_Create_System(Task_P3, 3);
}