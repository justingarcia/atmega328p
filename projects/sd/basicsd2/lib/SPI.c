/*
 * File: SPI.c
 * Author: Justin Garcia
 * ---------------------
 * Provides low-level SPI functionality
 * for the AVR ATMega328p MCU.
 */

#include "SPI.h"
#include <avr/io.h>

// enable: Enable SPI and set AVR to master
void SPI_enable(void)
{
	SPCR |= _BV(SPE) | _BV(MSTR);
}

// setBitOrder: Set order of data transmitted
// 0 = MSB first
// 1 = LSB first
void SPI_setBitOrder(uint8_t order)
{
	if (order == SPI_LSB_ORDER)
	{
		SPCR |= _BV(DORD);
	}
	else
	{
		SPCR &= ~_BV(DORD);
	}
}

// setDataMode: Configure clock generation
// SPI_MODE0 = Leading, rising edge
// SPI_MODE1 = Trailing, falling edge
// SPI_MODE2 = Leading, falling edge
// SPI_MODE3 = Trailing, rising edge
void SPI_setDataMode(uint8_t mode)
{
	SPCR = (SPCR & ~SPI_MODE_MASK) | mode;
}

// setClockDiv: Configure clock rate
// SPI_CLOCK4 = F_CPU / 4
// SPI_CLOCK16 = F_CPU / 16
// SPI_CLOCK64 = F_CPU / 64
// SPI_CLOCK128 = F_CPU / 128
void SPI_setClockDiv(uint8_t div)
{
	SPCR = (SPCR & ~SPI_CLOCK_MASK) | div;
}

// transfer: Performs an SPI full duplex 8-bit data transfer
uint8_t SPI_transfer(uint8_t data)
{
	SPDR = data;						// Start transfer
	loop_until_bit_is_set(SPSR, SPIF);	// Wait for transfer complete
	return SPDR;						// Return received value
}
