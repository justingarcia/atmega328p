/*
 * File: basicout.c
 * Author: Jusitn Garcia
 * ---------------------
 * Demonstrates simple serial communication with an ATmega328p microcontroller
 * and a MAX232 chip.
 * Modified from a Spark Fun tutorial by Nathan Seidle.
 */

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#define sbi(var, mask) ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask) ((var) &= (uint8_t)~(1 << mask))

#define STATUS_LED 0

/* Prototypes */

void init(void);
static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

/* Functions */

// main: Main program
int main(void)
{
	init();

	uint8_t x = 0;
	
	// Main loop
	while(1)
	{
		printf("Now sending the number: %d \n", x);

		sbi(PORTB, STATUS_LED);
		_delay_ms(500);

		cbi(PORTB, STATUS_LED);
		_delay_ms(500);

		x++;
	}

	return(0);
}

// init: Setup registers and defaults
void init(void)
{
	/* Port Registers */

	// IO: 1 = output, 0 = input
	DDRB = 0b11101111; // PORTB: PB4=MISO
	DDRC = 0b11111111; // PORTC: ALL OUT
	DDRD = 0b11111110; // PORTD: PD0=RX IN

	/* UART Registers */

	// Baud Rate:
	UBRR0H = (uint8_t)(MYUBRR >> 8);
	UBRR0L = (uint8_t)(MYUBRR);

	// Control and Status:
	UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter

	/* STDOUT */

	stdout = &mystdout; // Re-rout stdout; enables printf
}

// uart_putchar: Write UART data to output stream
static int uart_putchar(char c, FILE *stream)
{
	if (c == '\n') uart_putchar('\r', stream);	// Replace newline escape character with carriage return

	while( !(UCSR0A & (1 << UDRE0)) );			// Wait for empty transmit buffer
	UDR0 = c;									// Put character into buffer, send character

	return 0;
}

// uart_getchar: Receive UART data
uint8_t uart_getchar(void)
{
	while( !(UCSR0A & (1 << RXC0)) );	// Wait for data to be received
	return(UDR0);						// Get and return received data from buffer
}
