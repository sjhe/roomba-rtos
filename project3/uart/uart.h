/*
 * BlockingUART.h
 *
 *  Created on: Jul 9, 2013
 *      Author: andpol
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <avr/common.h>

#define TX_BUFFER_SIZE 64

#define USART_BAUDRATE 9600

#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2) ) / ( USART_BAUDRATE )) - 1)

void UART_Init0(uint32_t baud_rate);
void UART_Transmit0(unsigned char data);
unsigned char UART_Receive0();
void UART_print(const char* fmt, ...);
void UART_send_raw_bytes(const uint8_t num_bytes, const uint8_t* data);



void Roomba_UART_Init(void);
void Roomba_Send_Byte(uint8_t);
unsigned char Roomba_Receive_Byte(void);
void Roomba_Send_String(char*);

void Bluetooth_UART_Init(void);
void Bluetooth_Send_Byte(uint8_t);
unsigned char Bluetooth_Receive_Byte(void);
void Bluetooth_Send_String(char*);

#endif /* UART_H_ */