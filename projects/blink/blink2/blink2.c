/*
 * File: blink2.c
 * Author: Justin Garcia
 * ---------------------
 * Blinks an LED on an AVR ATMega328 MCU.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define LED PC5

/* Prototypes */

void init(void);

/* Functions */

// main: Main program
int main(void)
{
	init();

	while(1)
	{
		PORTC |= _BV(LED);
		_delay_ms(1000);
		PORTC &= ~_BV(LED);
		_delay_ms(1000);
	}

	return 0;
}

// init: Setup IO direction and pull-up registers
void init(void)
{
	// Data Direction Registers: 1=OUT, 0=IN
	DDRB = 0x00;
	DDRC = _BV(LED);
	DDRD = 0x00;

	// Data Registers: 1=HIGH, 0=LOW
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
}
