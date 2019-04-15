/*
 * File: basicio.c
 * Author: Justin Garcia
 * ---------------------
 * Implements input and output streams to allow for communication
 * with an ATmega328p microcontroller using printf and fgets.
 */

#define F_CPU 16000000
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
void init_port(void);
void init_uart(unsigned int ubrr);

static int uart_putchar(char c, FILE *stream);
static int uart_getchar(FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE mystdin = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

/* Functions */

// main: Main program
int main(void)
{
	// Initialize
	init();
	char buff[1024];

	printf("Waiting for input:\n");

	// Main Loop
	while(1)
	{
		if ( fgets(buff, 1024, stdin) != NULL )
		{
			sbi(PORTB, STATUS_LED);
			_delay_ms(250);
			cbi(PORTB, STATUS_LED);

			//printf("Transmission received: ");
			//printf(buff);
			//printf("\n");			
		}
	}
}

// init: Setup registers and defaults
void init(void)
{
	init_port();
	init_uart(MYUBRR);

	stdout = &mystdout;
	stdin = &mystdin;
}

// init_port: Setup DDR and PORT registers
void init_port(void)
{
	// IO: 1=OUT, 0=IN
	DDRB = 0b11101111; // PORTB: PB4=MISO
	DDRC = 0b11111111; // PORTC: ALL OUT
	DDRD = 0b11111110; // PORTD: PD0=RXD
}

// init_uart: Setup UBRRn and UCSRn registers
void init_uart(unsigned int ubrr)
{
	// Baud Rate:
	UBRR0H = (uint8_t)(ubrr >> 8); // High byte
	UBRR0L = (uint8_t)(ubrr >> 0); // Low byte

	// Control and Status:
	UCSR0B = (1 << TXEN0) | (1 << RXEN0); // Enable transmitter and receiver
}

// uart_putchar: Write char to UART output stream
static int uart_putchar(char c, FILE *stream)
{
	if (c == '\n') uart_putchar('\r', stream);	// Replace newline with carriage return

	while( !(UCSR0A & (1 << UDRE0)) );			// Wait for empty transmit buffer
	UDR0 = c;									// Put character into buffer, send character

	return 0;
}

// uart_getchar: Receive UART char data input
uint8_t uart_getchar(FILE *stream)
{
	while( !(UCSR0A & (1 << RXC0)) );			// Wait for data to be completely received

	if (UDR0 == '\r') UDR0 = (uint8_t)('\n');	// Replace carriage return with newline
	return(UDR0);								// Get and return received data from buffer
}
