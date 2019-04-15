/*
 * File: UART.c
 * Author: Justin Garcia
 * ---------------------
 * Provides low-level UART functionality
 * for the AVR ATMega328p MCU.
 */

#include "UART.h"
#include "config.h"
#include <stdio.h>
#include <avr/io.h>
#include <util/setbaud.h>

FILE UART_output = {0};
FILE UART_input = {0};

// init: Setup UBRRn and UCSRn registers
void UART_init(void)
{
	// Baud Rate:
	UBRR0H = UBRRH_VALUE;	// High byte
	UBRR0L = UBRRL_VALUE;	// Low byte

	// Control and Status:
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);	// 8-bit data
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);	// Enable RX and TX

	// Re-rout stdout and stdin
	fdev_setup_stream(&UART_output, UART_write, NULL, _FDEV_SETUP_WRITE);
	fdev_setup_stream(&UART_input, NULL, UART_read, _FDEV_SETUP_READ);
	
	stdout = &UART_output;
	stdin = &UART_input;
}

// read: Receive UART char input
int  UART_read(FILE *stream)
{
	loop_until_bit_is_set(UCSR0A, RXC0);	// Wait for complete data acquisition

	if (UDR0 == '\r') return '\n';			// Replace carriage return with newline
	else return UDR0;						// Get and return received data from buffer
}

// write: Write char to UART output stream
int UART_write(char c, FILE *stream)
{
	if (c == '\n') UART_write('\r', stream);	// Add carriage return to newline

	loop_until_bit_is_set(UCSR0A, UDRE0);		// Wait for empty transmit buffer
	UDR0 = c;									// Put character into buffer and send

	return 0;
}
