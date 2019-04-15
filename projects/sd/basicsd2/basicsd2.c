/*
 * File: basicsd2.c
 * Author: Justin Garcia
 * ---------------------
 * Demonstrates basic SD card reading.
 * Utilizes the AVR ATMega328p MCU.
 * Utilizes the Petit FatFs library.
 */

#include "config.h"
#include "UART.h"
#include "SPI.h"
#include "pff.h"
#include <avr/io.h>
#include <util/delay.h>

/* Global Variables */

FATFS fs;
FRESULT res;
uint8_t buff[256];
unsigned int br;

/* Prototypes */

void init(void);
void initIO(void);
void initSPI(void);

/* Functions */

int main(void)
{
	init();

	printf("Hello, world!\n");
	res = pf_mount(&fs);
	printf("Mounting... Error: %d\n", res);

	res = pf_open("test.txt");
	printf("Opening... Error: %d\n", res);

	res = pf_read(buff, 256, &br);
	printf("Reading... Error: %d\n", res);
	for (uint16_t i = 0; i < 256; i++)
	{
		printf("%c", buff[i]);
	}
	printf("\n");

	while(1)
	{

	}

	return 0;
}

void init(void)
{
	_delay_ms(1000);

	initIO();
	initSPI();
	UART_init();
}

void initIO(void)
{
	// Data Direction Registers: 1=OUT, 0=IN
	DDRB = _BV(SS) | _BV(MOSI) | _BV(SCK);				// MISO=IN
	DDRC = _BV(LED);									// ---
	DDRD = _BV(TXD) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);	// RXD=IN

	// Data Registers: 1=HIGH, 0=LOW
	PORTB = _BV(SRST) | _BV(SS) | _BV(MOSI);
	PORTC = 0x00;
	PORTD = _BV(SDCS) | _BV(XDCS) | _BV(XCS);
}

void initSPI(void)
{
	SPI_enable();
	SPI_setBitOrder(SPI_MSB_ORDER);
	SPI_setDataMode(SPI_MODE0);
	SPI_setClockDiv(SPI_CLOCK128);
}
