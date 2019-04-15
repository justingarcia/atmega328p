/*
 * File: simpleio.c
 * Author: Justin Garcia
 * ---------------------
 * Implements UART input and output streams to allow for communication
 * with an ATmega328p microcontroller using stdin and stdout.
 */

#define F_CPU 16000000UL
#define BAUD 9600

#include <stdio.h>
#include <avr/io.h>
#include <util/setbaud.h>
#include <util/delay.h>

#define sbi(var, mask) ( (var) |= _BV(mask) )
#define cbi(var, mask) ( (var) &= ~(_BV(mask)) )

#define STATUS_LED 0

/* Prototypes */

static void init(void);
static void init_port(void);
static void init_uart(void);

static int uart_putchar(char c, FILE *stream);
static char uart_getchar(FILE *stream);

static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

/* Functions */

// main: Main program
int main(void)
{
	// Initialize
	init();
	char buff[256];

	printf("Waiting for input:\n");

	// Main Loop
	while(1)
	{
		if ( fgets(buff, 255, stdin) != NULL )
		{
			printf("Transmission received: %s", buff);

			sbi(PORTB, STATUS_LED);
			_delay_ms(100);
			cbi(PORTB, STATUS_LED);
		}
	}

	return 0;
}

// init: Setup registers and defaults
static void init(void)
{
	init_port();
	init_uart();

	stdout = &uart_output;
	stdin = &uart_input;
}

// init_port: Setup DDR and PORT registers
static void init_port(void)
{
	// IO: 1=OUT, 0=IN
	DDRB = 0b11101111; // PORTB: PB4=MISO
	DDRC = 0b11111111; // PORTC: ALL OUT
	DDRD = 0b11111110; // PORTD: PD0=RXD
}

// init_uart: Setup UBRRn and UCSRn registers
static void init_uart(void)
{
	// Baud Rate:
	UBRR0H = UBRRH_VALUE; // High byte
	UBRR0L = UBRRL_VALUE; // Low byte

	// Control and Status:
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);	// 8-bit data
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);	// Enable RX and TX
}

// uart_putchar: Write char to UART output stream
static int uart_putchar(char c, FILE *stream)
{
	if (c == '\n') uart_putchar('\r', stream);	// Add carriage return to newline

	loop_until_bit_is_set(UCSR0A, UDRE0);		// Wait for empty transmit buffer
	UDR0 = c;									// Put character into buffer and send

	return 0;
}

// uart_getchar: Receive UART char input
static char uart_getchar(FILE *stream)
{
	loop_until_bit_is_set(UCSR0A, RXC0);	// Wait for complete data acquisition
	
	if (UDR0 == '\r') return '\n';			// Replace carriage return with newline
	else return UDR0;						// Get and return received data from buffer
}
