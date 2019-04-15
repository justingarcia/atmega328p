/*
 * ----------------------------------------------------------------------------
 * FILE: ADC.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Module encapsulating analog-to-digital conversion for the
 * AVR ATMega328p MCU.
 * Source file.
 * ----------------------------------------------------------------------------
 */

// INCLUDE FILES --------------------------------------------------------------

#include "ADC.h"

#include <avr/io.h>

// PUBLIC FUNCTIONS -----------------------------------------------------------

// init: Initialize ADC registers
void ADC_init(void)
{
	// ADC Prescaler Settings: F_ADC = F_CPU/16
	ADCSRA = _BV(ADPS2);// | _BV(ADPS1) | _BV(ADPS0);

	// Enable ADC
	ADCSRA |= _BV(ADEN);
}

// read: Return the value of an ADC channel
int16_t ADC_read(uint8_t ch)
{
	// Select channel
	ch &= 0x07;
	ADMUX = (ADMUX & 0xF8) | ch;

	// Start single conversion
	ADCSRA |= _BV(ADSC);

	// Wait for conversion completion
	loop_until_bit_is_clear(ADCSRA, ADSC);

	// Return 10-bit converted value
	return ADC;
}

// term: Terminate ADC function
void ADC_term(void)
{
	// Disable ADC
	ADCSRA &= ~_BV(ADEN);
}
