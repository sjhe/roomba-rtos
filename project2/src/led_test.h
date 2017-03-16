/**
 * \file LED_Test.h
 * \brief Constants and functions for controlling LEDs on a AT90USBKey
 * 
 * \mainpage Constants and functions for controlling the state of the onboard
 *  LEDs on the AT90USBKey. 
 *
 * \author Alexander M. Hoole
 * \date October 2006
 */

#define LED_ON_BOARD	PB7
#define LED_PING 	    PB6
#define LED_ISR 	    PB5


void init_LED_ON_BOARD();
void init_LED_PING();
void init_LED_ISR();
void toggle_LED_ON_BOARD();
void enable_LED(unsigned int mask);
void disable_LEDs(void);
void led_toggle(unsigned int mask);
