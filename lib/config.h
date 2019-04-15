/*
 * File: config.h
 * Author: Justin Garcia
 * ---------------------
 * A list of define MACROS related to the
 * AVR ATMega328p MCU system configuration.
 */

#ifndef AVR_CONFIG_H_
#define AVR_CONFIG_H_

// MCU SETTINGS ---------------------------------------------------------------

#define F_CPU	16000000UL
#define BAUD	9600

// INCLUDE FILES --------------------------------------------------------------

#include <avr/io.h>
#include "SPI.h"

// GPIO SETTINGS --------------------------------------------------------------

#define XRST	PB0
#define WIEN	PB1
#define SS		PB2
#define MOSI	PB3
#define MISO	PB4
#define SCK		PB5

#define ADC1	PC0
#define ADC2	PC1
#define ADC3	PC2
#define ADC4	PC3
#define LED		PC4
#define PWR		PC5

#define RXD		PD0
#define TXD		PD1
#define WIRQ	PD2
#define XDRQ	PD3
#define WICS	PD4
#define SDCS	PD5
#define XDCS	PD6
#define XCS		PD7

// SPI CONFIGS ----------------------------------------------------------------

#define SPI_DEFAULT	SPI_CONFIG0
#define SPI_CC3000	SPI_CONFIG1
#define SPI_VS1053	SPI_CONFIG2
#define SPI_SDCARD	SPI_CONFIG3

#endif
