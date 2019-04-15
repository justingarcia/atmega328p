/*
 * File: basicsd.c
 * Author: Justin Garcia
 * ---------------------
 * Demonstrates basic SD card reading capabilities, using
 * an AVR ATMega328p, an MicroSD breakout board, and
 * the Petit FatFs library.
 */

#define F_CPU 16000000UL
#define BAUD 9600

#include <stdio.h>
#include <avr/io.h>
#include <util/setbaud.h>
#include <util/delay.h>
#include "src/pff.h"

#define MOSI	PB3
#define MISO	PB4
#define SCK		PB5

#define LED		PC5

#define RXD		PD0
#define TXD		PD1
#define SDCS	PD4

/* Prototypes */

static void init(void);
static void init_IO(void);
static void init_UART(void);

int uart_putchar(char c, FILE *stream);
char uart_getchar(FILE *stream);
static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

static void test(void);

/* Global Variables */

FATFS fs;
uint8_t buff[32];
uint16_t br;
FRESULT res;

/* Functions */

// main: Main Program
int main(void)
{
	init();

	printf("Hello!\n");

	printf("FR_OK = %d\n", FR_OK);
	printf("FR_NOT_READY = %d\n", FR_NOT_READY);
	printf("FR_DISK_ERR = %d\n", FR_DISK_ERR);
	printf("FR_NO_FILESYSTEM = %d\n", FR_NO_FILESYSTEM);

	
	printf("FR_NO_FILE = %d\n", FR_NO_FILE);
	printf("FR_NO_PATH = %d\n", FR_NO_PATH);
	printf("FR_NOT_ENABLED = %d\n", FR_NOT_ENABLED);

	
	printf("FR_NOT_OPENED = %d\n", FR_NOT_OPENED);

	res = pf_mount(&fs);
	printf("Mounting... %d\n", res);

	res = pf_open("test.txt");
	printf("Opening... %d\n", res);

	res = pf_read(buff, 32, &br);
	printf("Reading... %d\n", res);

	printf(buff);

	pf_mount(NULL);

	while(1)
	{

	}

	return 0;
}

// init: Initialize system
static void init(void)
{
	init_IO();
	init_UART();

	stdout = &uart_output;
	stdin = &uart_input;
}

// init_IO: Setup DDR and PORT registers
static void init_IO(void)
{
	// Data Direction Registers: 1=OUT, 0=IN
	DDRB = _BV(MOSI) | _BV(SCK); // MISO=IN
	DDRC = _BV(LED);
	DDRD = _BV(TXD) | _BV(SDCS); // RXD=IN

	// Data Registers: 1=HIGH, 0=LOW
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = _BV(SDCS);
}

// init_uart: Setup UBRRn and UCSRn registers
static void init_UART(void)
{
	// Baud Rate:
	UBRR0H = UBRRH_VALUE; // High byte
	UBRR0L = UBRRL_VALUE; // Low byte

	// Control and Status:
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);	// 8-bit data
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);	// Enable RX and TX
}

// uart_putchar: Write char to UART output stream
int uart_putchar(char c, FILE *stream)
{
	if (c == '\n') uart_putchar('\r', stream);	// Add carriage return to newline

	loop_until_bit_is_set(UCSR0A, UDRE0);		// Wait for empty transmit buffer
	UDR0 = c;									// Put character into buffer and send

	return 0;
}

// uart_getchar: Receive UART char input
char uart_getchar(FILE *stream)
{
	loop_until_bit_is_set(UCSR0A, RXC0);	// Wait for complete data acquisition

	if (UDR0 == '\r') return '\n';			// Replace carriage return with newline
	else return UDR0;						// Get and return received data from buffer
}

static void test(void)
{
	PORTC |= _BV(LED);
	_delay_ms(5000);
	PORTC &= ~_BV(LED);
	_delay_ms(5000);
}
