/*
 * ----------------------------------------------------------------------------
 * FILE: vsspi.h
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Provides low-level Serial Command Interface and Serial Data Interface
 * communication for the VS1053 mp3 decoder.
 * Header file.
 * ----------------------------------------------------------------------------
 */

#ifndef VSSPI_H_
#define VSSPI_H_

// INCLUDE FILES --------------------------------------------------------------

#include <stdio.h>

// DRIVER SETTINGS ------------------------------------------------------------

#include "lib/config.h"
#include <util/delay.h>

// DRIVER FUNCTIONS -------------------------------------------------------

#define VS1053_CS_HIGH		PORTD |= _BV(XCS)
#define VS1053_DCS_HIGH		PORTD |= _BV(XDCS)
#define VS1053_RST_LOW		PORTB &= ~_BV(XRST)
#define VS1053_RST_HIGH		PORTB |= _BV(XRST)

#define readyDREQ			(PIND & _BV(XDRQ))
#define waitForDREQ()		loop_until_bit_is_set(PIND, XDRQ)
#define VS1053_delay_ms(t)	_delay_ms(t)

// DRIVER FUNCTION PROTOTYPES ---------------------------------------------

// initSPI: Intialize SPI control registers on host
void VS1053_initSPI(void);

// termSPI: Terminate SPI connection
void VS1053_termSPI(void);

// interruptEnable
void VS1053_interruptEnable(void);

// interruptDisable
void VS1053_interruptDisable(void);

// PUBLIC GLOBAL VARIABLES ----------------------------------------------------

typedef void (*tDreqCB)(void);
tDreqCB DreqCB;

// PUBLIC FUNCTION PROTOTYPES -------------------------------------------------

// sciRead: Performs VS1053 Serial Command Interface acquisition
uint16_t sciRead(uint8_t addr);

// sciWrite: Performs VS1053 Serial Command Interface transmission
void sciWrite(uint8_t addr, uint16_t data);

// sdiWrite: Performs VS1053 Serial Data Interface transmission
void sdiWrite(uint8_t *buff, uint16_t buffsize);

#endif
