/*
 * File: SPI.h
 * Author: Justin Garcia
 * ---------------------
 * Provides low-level SPI functionality
 * for the AVR ATMega328p MCU.
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdio.h>

#define SPI_MSB_ORDER	0	
#define SPI_LSB_ORDER	1

#define SPI_MODE0		0x00
#define SPI_MODE1		0x04
#define SPI_MODE2		0x08
#define SPI_MODE3		0x0C

#define SPI_CLOCK4		0x00
#define SPI_CLOCK16		0x01
#define SPI_CLOCK64		0x02
#define SPI_CLOCK128	0x03

#define SPI_MODE_MASK	0x0C
#define SPI_CLOCK_MASK	0x03

/* Prototypes */

	// enable: Enable SPI and set AVR to master
	void SPI_enable(void);

	// setBitOrder: Set order of data transmitted
	void SPI_setBitOrder(uint8_t);

	// setDataMode: Configure clock generation
	void SPI_setDataMode(uint8_t);

	// setClockDiv: Configure clock mode
	void SPI_setClockDiv(uint8_t);

	// transfer: Performs an SPI full duplex 8-bit data transfer
	uint8_t SPI_transfer(uint8_t);

#endif
