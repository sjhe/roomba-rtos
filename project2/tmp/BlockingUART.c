#include "BlockingUART.h"

#include <avr/io.h>
#include <avr/interrupt.h>

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
