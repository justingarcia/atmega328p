/*
 * File: blink.c
 * Author: Justin Garcia
 * ---------------------
 * A simple LED blinking program for an ATmega328p microcontroller.
 * Modified from a Spark Fun tutorial by Nathan Seidle.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

/* Prototypes */

void ioinit(void);

/* Functions */

int main(void)
{
	ioinit();

	while(1)
	{
		PORTB |= (1 << 0);
		_delay_ms(1000);
		PORTB &= ~(1 << 0);
		_delay_ms(1000);
	}

	return(0);
}

// ioinit: Setup IO pins and defaults
void ioinit(void)
{
	// 1 = output, 0 = input
	DDRB = 0b11111111; // Port B - all outputs
	DDRC = 0b11111111; // Port C - all outputs
	DDRD = 0b11111110; // Port D - RX input on PD0

	// 1 = high, 0 = low
	PORTB = 0x00; // Port B - all low
	PORTC = 0x00; // Port C - all low
	PORTD = 0x00; // Port D - all low
}
