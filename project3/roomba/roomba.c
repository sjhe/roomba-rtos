#define F_CPU 16000000

#include "roomba.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include "../uart/uart.h" 

void Roomba_Init(){
   
    // Wake Roomba from sleep
    // Pin 25
    DDRA  |= (1<<PA3);
    PORTA |= (1<<PA3);
    
    PORTA &= ~(1<<PA3);
    
    PORTA |= (1<<PA3);
    
    // start the OI
    Roomba_Send_Byte(START);

    _delay_ms(2100);

    PORTA |= (1<<PA3);
    PORTA &= ~(1<<PA3);
    _delay_ms(100);

    PORTA |= (1<<PA3);
    PORTA &= ~(1<<PA3);
    _delay_ms(100);

    PORTA |= (1<<PA3);
    PORTA &= ~(1<<PA3);
    _delay_ms(100);

    // set Roomba to safe mode
    Roomba_Send_Byte(SAFE_MODE); 

    Roomba_Song(0); // Initialize song 0 
}

void Roomba_Drive(int16_t velocity, int16_t radius) {    
    Roomba_Send_Byte(DRIVE);
    Roomba_Send_Byte(velocity>>8);
    Roomba_Send_Byte(velocity);
    Roomba_Send_Byte(radius>>8);
    Roomba_Send_Byte(radius);   
}

void Roomba_Play(uint8_t song) {
	Roomba_Send_Byte(PLAY);
	Roomba_Send_Byte(song);
}

void Roomba_Sensors(uint8_t packet_id) {
	Roomba_Send_Byte(SENSORS);
	Roomba_Send_Byte(packet_id);
}

void Roomba_QueryList(uint8_t packet1, uint8_t packet2) {

    Roomba_Send_Byte(QUERYLIST);
    Roomba_Send_Byte(2);
    Roomba_Send_Byte(packet1);
    Roomba_Send_Byte(packet2);
}

void Roomba_Song(uint8_t n) {
	Roomba_Send_Byte(SONG);
	Roomba_Send_Byte(n);  // Song 0
	Roomba_Send_Byte(6);  // 6 Notes
	
	Roomba_Send_Byte(72); // C
	Roomba_Send_Byte(16);
	Roomba_Send_Byte(69); // A
	Roomba_Send_Byte(16);
	Roomba_Send_Byte(67); // G
	Roomba_Send_Byte(16);
	Roomba_Send_Byte(64); // E
	Roomba_Send_Byte(16);
	Roomba_Send_Byte(62); // D
	Roomba_Send_Byte(16);
	Roomba_Send_Byte(60); // C
	Roomba_Send_Byte(16);
}
