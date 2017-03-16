/*
 * BlockingUART.h
 *
 *  Created on: Jul 9, 2013
 *      Author: andpol
 */

#ifndef BLOCKINGUART_H_
#define BLOCKINGUART_H_

#include <avr/common.h>

#define TX_BUFFER_SIZE 64

#define MYBRR(baud_rate) (F_CPU / 16 / (baud_rate) - 1)

void UART_Init0(uint32_t baud_rate);
void UART_Transmit0(unsigned char data);
unsigned char UART_Receive0();
void UART_print(const char* fmt, ...);
void UART_send_raw_bytes(const uint8_t num_bytes, const uint8_t* data);

#endif /* BLOCKINGUART_H_ */
