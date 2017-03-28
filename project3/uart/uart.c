#define F_CPU 16000000
#include <util/delay.h>

#include "uart.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <string.h>
#include <stdio.h>

void UART_Init0(uint32_t baud_rate) {
    // Set baud rate
    UBRR0 = MYBRR(baud_rate);
    // Enable receiver and transmitter
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    // Default frame format: 8 data, 1 stop bit , no parity
}

void UART_Transmit0(unsigned char data) {
    // Busy wait for empty transmit buffer
    while (!(UCSR0A & _BV(UDRE0)))
        ;
    // Put data into buffer, sends the data
    UDR0 = data;
}

unsigned char UART_Receive0() {
    // Busy wait for data to be received
    while (!(UCSR0A & _BV(RXC0)))
        ;
    // Get and return received data from buffer
    return UDR0 ;
}

void UART_print(const char* fmt, ...) {
    uint16_t sreg = SREG;
    cli();
    char buffer[TX_BUFFER_SIZE];
    va_list args;
    size_t size;

    va_start(args, fmt);
    size = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    // Error case: do not output to UART
    if (size < 0) {
        return;
    }

    uint8_t i = 0;
    while (i < size) {
        UART_Transmit0(buffer[i++]);
    }
    SREG = sreg;
}

void UART_send_raw_bytes(const uint8_t num_bytes, const uint8_t* data) {
    uint8_t i;
    for (i = 0; i < num_bytes; i++) {
        UART_Transmit0(data[i]);
    }
}

// Project 3 


void Roomba_UART_Init(){   
    // Set baud rate to 19.2k
    UBRR3 = 0x33;
    
    // Enable receiver, transmitter
    UCSR3B = (1<<RXEN3) | (1<<TXEN3);

    // 8-bit data
    UCSR3C = ((1<<UCSZ31)|(1<<UCSZ30));

    // disable 2x speed
    UCSR3A &= ~(1<<U2X3);
}

void Roomba_Send_Byte(uint8_t data_out){      
    // Wait for empty transmit buffer
    while(!( UCSR3A & (1<<UDRE3)));
    // Put data into buffer
    UDR3 = data_out;
}

unsigned char Roomba_Receive_Byte(){      
    // Wait for data to be received
    while(!( UCSR3A & (1<<RXC3)));
    // Get and return data from buffer
    return UDR3;
}

void Roomba_Send_String(char *string_out){
    for(; *string_out; string_out++){
        Roomba_Send_Byte(*string_out);
    }
}

void Bluetooth_UART_Init(){   
    // Set baud rate to 19.2k
    UBRR1 = 103;
    
    // Enable receiver, transmitter
    UCSR1B = (1<<RXEN1) | (1<<TXEN1);

    // 8-bit data
    UCSR1C = ((1<<UCSZ11)|(1<<UCSZ10));

    // disable 2x speed
    UCSR1A &= ~(1<<U2X1);
}

void Bluetooth_Send_Byte(uint8_t data_out){      
    // Wait for empty transmit buffer
    while(!( UCSR1A & (1<<UDRE1)));
    // Put data into buffer
    UDR1 = data_out;
}

unsigned char Bluetooth_Receive_Byte(){      
    // Wait for data to be received
    while(!( UCSR1A & (1<<RXC1)));
    // Get and return data from buffer
    return UDR1;
}

void Bluetooth_Send_String(char *string_out){
    for(; *string_out; string_out++){
        _delay_ms(10);
        Bluetooth_Send_Byte(*string_out);
    }
}

