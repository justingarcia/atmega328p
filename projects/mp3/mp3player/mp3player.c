/*
 * ----------------------------------------------------------------------------
 * FILE: mp3player.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * A basic mp3 player, using the VS1053 mp3 decoder module and firmware
 * written by me. Intended for use with the AVR ATMega328p MCU
 * ----------------------------------------------------------------------------
 */

// INCLUDE FILES --------------------------------------------------------------

#include "lib/config.h"
#include "lib/UART.h"

#include "vs1053/player.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// FUNCTION PROTOTYPES --------------------------------------------------------

void init(void);
void initIO(void);
void initWDT(void);

// FUNCTIONS ------------------------------------------------------------------

// main: Main program
int main(void)
{
	// INITIALIZE ---------------------------------------------------------

	init();
	printf("\n");

	// MAIN LOOP ----------------------------------------------------------

	VS1053_play("test.mp3");

	while (1);

	// TERMINATE ----------------------------------------------------------

	return 0;
}

// init: System initialization
void init(void)
{
	_delay_ms(100);

	cli();

	initIO();
	initWDT();
	UART_init();

	sei();

	VS1053_init();
}

// initIO: Setup IO direction and internal pull-up registers
void initIO(void)
{
	// DATA DIRECTION: 1=OUT, 0=IN ----------------------------------------

	DDRB = _BV(WIEN) | _BV(XRST) | _BV(SS) | _BV(MOSI) | _BV(SCK);
	DDRC = _BV(LED);
	DDRD = _BV(TXD) | _BV(WICS) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);

	// DATA PULLUP: 1=HIGH, 0=LOW -----------------------------------------

	PORTB = _BV(XRST) | _BV(SS);
	PORTC = 0x00;
	PORTD = _BV(WICS) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);
}

// initWDT: Clear watchdog timer control bits
void initWDT(void)
{
	wdt_reset();

	MCUSR &= ~_BV(WDRF);
	WDTCSR &= ~_BV(WDE);
}
