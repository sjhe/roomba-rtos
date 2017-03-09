#include <avr/io.h>
#include "LED_Test.h"
/**
 * \file LED_Test.c
 * \brief Small set of test functions for controlling LEDs on a AT90USBKey
 * 
 * \mainpage Simple set of functions to control the state of the onboard
 *  LEDs on the AT90USBKey. 
 *
 * \author Alexander M. Hoole
 * \date October 2006
 */

void init_LED_ON_BOARD(void)
{
	DDRB |= _BV(LED_ON_BOARD);		//Set LED to output (pins 4 and 5)
	PORTB = 0x00;		//Initialize port to LOW (turn off LEDs)
}

void init_LED_PING(void)
{
	DDRB |= _BV(LED_PING);		//Set LED to output (pins 4 and 5)
	PORTB = 0x00;		//Initialize port to LOW (turn off LEDs)
}

void init_LED_ISR(void)
{
	DDRB |= _BV(LED_ISR);		//Set LED to output (pins 4 and 5)
	PORTB = 0x00;		//Initialize port to LOW (turn off LEDs)
}


void enable_LED(unsigned int mask)
{
	PORTB = _BV(mask);		//Initialize port to high
}

void disable_LEDs(void)
{
	PORTB = 0x00;	//Initialize port to high
}
