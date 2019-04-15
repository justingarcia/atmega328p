/*
 * ----------------------------------------------------------------------------
 * FILE: ADC.h
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Module encapsulating analog-to-digital conversion for the
 * AVR ATMega328p MCU.
 * Header file.
 * ----------------------------------------------------------------------------
 */

#ifndef AVR_ADC_H_
#define AVR_ADC_H_

// INCLUDE FILES --------------------------------------------------------------

#include <stdio.h>

// PUBLIC FUNCTION PROTOTYPES -------------------------------------------------

// init: Initialize ADC registers
void ADC_init(void);

// read: Return the value of an ADC channel
int16_t ADC_read(uint8_t ch);

// term: Terminate ADC function
void ADC_term(void);

#endif
