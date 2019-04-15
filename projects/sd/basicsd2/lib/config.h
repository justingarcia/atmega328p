/*
 * File: config.h
 * Author: Justin Garcia
 * ---------------------
 * A list of define MACROS related to the
 * AVR ATMega328p MMU system configuration.
 */

#ifndef AVR_CONFIG_H_
#define AVR_CONFIG_H_

#define F_CPU	16000000UL
#define BAUD	9600

#include <avr/io.h>

#define SRST	PB1
#define SS		PB2
#define MOSI	PB3
#define MISO	PB4
#define SCK		PB5

#define LED		PC5

#define RXD		PD0
#define TXD		PD1
#define WIRQ	PD2
#define DREQ	PD3
#define WICS	PD4
#define SDCS	PD5
#define XDCS	PD6
#define XCS		PD7

#endif
