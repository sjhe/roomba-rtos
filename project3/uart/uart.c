#define F_CPU 16000000
#include <util/delay.h>

#include "uart.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#include <string.h>
#include <stdio.h>

#include "../src/led_test.h"

#define UART_BUFFER_SIZE    32
#define TX_BUFFER_SIZE      64


static volatile uint8_t uart_buffer[UART_BUFFER_SIZE];
static volatile uint8_t uart_buffer_index;

// ATMega2560 has 4 USART interfaces
// UART0 is the line that is used by the USB plug??
// UART1 is the interface that we are using for this one...


void UART_Init0(uint32_t baud_rate) {
    // Set baud rate
    UBRR0 = BAUD_PRESCALE;
    // Enable receiver and transmitter
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    // Default frame format: 8 data, 1 stop bit , no parity
}

void UART_Transmit0(unsigned char data) {
    // Busy wait for empty transmit buffer
    while (!(UCSR0A & _BV(UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = data;
}

unsigned char UART_Receive0() {
    // Busy wait for data to be received
    while (!(UCSR0A & _BV(RXC0)));
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

    UCSR1B = (1 << RXEN1 ) | (1 << TXEN1 ) | (1 << RXCIE1); // Turn on the transmission and reception circuitry
    
    UCSR1C =  (1 << UCSZ10 ) | (1 << UCSZ11 ); // Use 8- bit character sizes    
    
    UBRR1H = ( BAUD_PRESCALE >> 8) ; // Load upper 8- bits of the baud rate value into the high byte of the UBRR register
    
    UBRR1L = BAUD_PRESCALE ; // Load lower 8 - bits of the baud rate value into the low byte of the UBRR register

	UCSR1A &= ~(1<<U2X1);

    // sei();  
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
    //    _delay_ms(4);
        Bluetooth_Send_Byte(*string_out);
    }
}


uint8_t uart_bytes_received(void)
{
    return uart_buffer_index;
}

void uart_reset_receive(void)
{
    uart_buffer_index = 0;
}

// /**
//  * UART receive byte ISR
//  */


ISR(USART1_RX_vect)
{
    // FEn - frame error
    // DORn - data overrun
    // UPEn - uart pairty error
    // char ReceiveByte;
    // ReceiveByte = UDR1;
    // if(ReceiveByte == '*'){
    //     UDR0 = '\n'; // print byte to serial 0
    // }else{
    //     UDR0 = ReceiveByte; // print byte to serial 0    
    // }
    
    uart_buffer[uart_buffer_index] = UDR1;
    uart_buffer_index = (uart_buffer_index + 1) % UART_BUFFER_SIZE;
}

uint8_t uart_get_byte(int index)
{
    if (index < UART_BUFFER_SIZE)
    {
        return uart_buffer[index];
    }
    return 0;
}




// UDRn - uart io data register n
//  shared by both the tranmission and receiving

// UCSRnA
//  [RXCn,TXCn,UDREn,FEn,DORn,UPEn, U2Xn, MPCMn]

//  RXCn - receive complete flag
//  TXCn - transmission complete flag. check that all trans are complete
//  UDREn - uart data register empty flag
//  FEn - frame erro flag
//  DORn - data overrun flag
//  UPEn - uart parity error
//  U2Xn - flag to set the transmission mode
//          0 - normal async
//          1 - double time async

// UCSRnB
//  RXCIEn - receive complete interrupt enable flag
//  TXCIEn - TX complete interrupt enable flag
//  UDRIEn - uart data register empty interrupt enable flag
//  RXEn - enable ( set high)  the Uart to do receiving
//  TXEn - enables (set high) the uart to do transmit
//  UCSZn2 - character size n
//      uses this with UCSZn1:0 to set the number of bits in a frame
//  RXB8n - Receive data bit 8
//  TXB8n - transmit data bit 8

// UCSRnC
//  UMSELn1,UMSELn0  - Select eh uart mode of operation
//  00 - async uart
//  01 - sync uart
//  10 - (reserved)
//  11 - master spi

//  UPMn1,UPMn0 - uart parity mode
//  00 - disabled
//  01 - reserved
//  10 - enabled, even parity
//  11 - enabled, odd parity

//  USBSn - uart stop bit select
//  0 - 1 stop bit
//  1 - 2 stop bits

//  UCSZn1 - character size n
//      use this with the UCZn2 in UCSRnB register to determine the number
//          of bits in the frame

//     UCPOLn - uart clock parity

// UCSZn2,UCSZn1,UCSZn0
// 000 - 5 bits
// 001 - 6 bits
// 010 - 7 bits
// 011 - 8 bits
// 100
// 101
// 110
// 111 - 9 bits

// TXEN1 -- enable the transmission flag
// RXEN1 -- enable the receive flag
// RXCIEn -- receive control interrupt flag
// TXCIEn -- allow tranmit interrupt
//UCSR1B = (1<<TXEN1) | ( 1<< RXEN1) | (1 << RXCIE1)

// UDRIEn -- UART Data Register Empty Interrupt Enable into UCSRnB
//  This will allow the data register empty interrupt to run...

// need to set the USART tranmission mode ( i.e async normal mode)
// baud rate
// frame format
// enabling the tranmsission and receiver

// UCSR<n><ABC> -- USART Control and Status Register
// UCSR<n>A
// UCSR<n>B
// UCSR<n>C

// RXEN<n> = enable receiver
// TXEN<n> = enable transmitter
// TXCn - transmit complete ( set one when)
// RXCn - receiver register C

// set the UCSR1A - trans mode, clear the tran register
// set the UCSR1B - enable transmite and receive
// set the UCSR1C - data rate

