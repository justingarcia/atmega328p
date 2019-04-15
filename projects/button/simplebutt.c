/*
 * File: simplebutt.c
 * Author: Justin Garcia
 * ---------------------
 * Lights up an LED when a button is pushed.
 * Uses the ATmega328 microcontroller.
 * Adapted from a tutorial by newbiehack.com
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define button_pushed(sfr, bit) bit_is_clear(sfr, bit)

#define BUTTON PB0
#define STATUS_LED PB1

/* Prototypes */

static void init(void);
static void init_port(void);

/* Functions */

// main: Main program
int main(void)
{
	// Initialize
	init();

	// Main Loop
	while(1)
	{
		if (button_pushed(PINB, BUTTON))
		{
			PORTB |= _BV(STATUS_LED);
		}
		else
		{
			PORTB &= ~(_BV(STATUS_LED));	
		}
	}

	return 0;
}

// init: Setup registers and defaults
static void init(void)
{
	init_port();
}

// init_port: Setup DDR and PORT registers
static void init_port(void)
{
	// IO: 1=OUT, 0=IN
	DDRB = 0b11101110; // PORTB: PB0=BUTTON, PB4=MISO
	DDRC = 0b11111111; // PORTC: ALL OUT
	DDRD = 0b11111110; // PORTD: PD0=RXD

	// PORT: 1=HIGH, 0=LOW
	PORTB |= _BV(BUTTON); // PORTB: PB0=BUTTON
}
