/*
 * File: SPI.c
 * Author: Justin Garcia
 * ---------------------
 * Provides low-level SPI functionality
 * for the AVR ATMega328p MCU.
 */

#include "SPI.h"
#include <avr/io.h>

/* Data Structures */

typedef struct
{
	uint8_t spcr;
	uint8_t spsr;

} spiData;

/* Global Variables */

static spiData spi_config[SPI_NCONFIGS];
static uint8_t spi_stack[SPI_NCONFIGS];
static uint8_t spi_stack_index = 0;
static uint16_t spi_stack_req = 0;

/* Functions */

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
	SPCR = (SPCR & ~SPI_CLOCK_MASK) | ((div >> 0) & SPI_CLOCK_MASK);
	SPSR = (SPSR & ~SPI_2XCLK_MASK) | ((div >> 2) & SPI_2XCLK_MASK);
}

// transfer: Performs an SPI full duplex 8-bit data transfer
uint8_t SPI_transfer(uint8_t data)
{
	SPDR = data;						// Start transfer
	loop_until_bit_is_set(SPSR, SPIF);	// Wait for transfer complete
	return SPDR;						// Return received value
}

// configStore: Save SPCR and SPSR configuration settings
//   4 different configuration settings can be saved
void SPI_configStore(uint8_t index)
{
	spi_config[index].spcr = SPCR;
	spi_config[index].spsr = SPSR & _BV(SPI2X);
}

// configSet: Switch to a different SPI configuration
void SPI_configSet(uint8_t index)
{
	SPCR = spi_config[index].spcr;
	if (spi_config[index].spsr)	SPSR |= _BV(SPI2X);
	else						SPSR &= ~_BV(SPI2X);
}

// configPush: Temporarily switch to a different SPI configuration
void SPI_configPush(uint8_t config_index)
{
	spi_stack_req++;

	if (config_index == spi_stack[spi_stack_index])	return;
	if (spi_stack_index >= SPI_NCONFIGS)			return;

	spi_stack[spi_stack_index] = config_index;
	spi_stack_index++;

	SPCR = spi_config[config_index].spcr;
	if (spi_config[config_index].spsr)	SPSR |= _BV(SPI2X);
	else								SPSR &= ~_BV(SPI2X);
}

// configPop: Switch back from the SPI configuration
void SPI_configPop(void)
{
	if (spi_stack_req <= 0)					return;
	if (spi_stack_req-- != spi_stack_index) return;

	spi_stack_index--;
	uint8_t config_index = spi_stack[spi_stack_index];

	SPCR = spi_config[config_index].spcr;
	if (spi_config[config_index].spsr)	SPSR |= _BV(SPI2X);
	else								SPSR &= ~_BV(SPI2X);
}
