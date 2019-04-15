/*
 * File: UART.h
 * Author: Justin Garcia
 * ---------------------
 * Provides low-level UART functionality
 * for the AVR ATMega328p MCU.
 */

#ifndef UART_H_
#define UART_H_

#include <stdio.h>

/* Prototypes */

	// init: Setup UBRRn and UCSRn registers
	void UART_init(void);

	// read: Receive UART char input
	int UART_read(FILE *);

	// write: Write char to UART output stream
	int UART_write(char, FILE *);

	// Input and output streams
	extern FILE UART_input;
	extern FILE UART_output;

#endif
