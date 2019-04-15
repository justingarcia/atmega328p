/*
 * ----------------------------------------------------------------------------
 * FILE: basicclient.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Demonstrates basic WI-FI capabilities, using the AVR ATMega328p MCU
 * and the TI CC3000_client WI-FI module.
 * ----------------------------------------------------------------------------
 */

// INCLUDE FILES --------------------------------------------------------------

#include "lib/config.h"
#include "lib/SPI.h"
#include "lib/UART.h"

#include "cc3000/wlan_client.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// CONSTANTS ------------------------------------------------------------------

#define WLAN_SECURITY			WLAN_SEC_WPA2
const char *WLAN_SSID			= "";
const unsigned char *WLAN_KEY	= "";

// FUNCTION PROTOTYPES --------------------------------------------------------

void init(void);
void initIO(void);
void initWDT(void);
void initSPI(void);

// FUNCTIONS ------------------------------------------------------------------

// main: Main program
int main(void)
{
	init();

	printf("\n");
	printf("MCU initialized\n");

	CC3000_initWlanClient();

	if(CC3000_connectAP(WLAN_SECURITY, WLAN_SSID, WLAN_KEY))
	{
		printf("Connection successful!\n");
	}
	else
	{
		printf("Connection FAILURE\n");
	}

	CC3000_disconnectAP();

	CC3000_termWlanClient();

	while(1);
	return 0;
}

// init: System initialization
void init(void)
{
	_delay_ms(100);

	cli();

	initIO();
	initWDT();
	initSPI();
	UART_init();

	sei();
}

// initIO: Setup IO direction and internal pull-up registers
void initIO(void)
{
	// Data Direction Registers: 1=OUT, 0=IN
	DDRB = _BV(WIEN) | _BV(XRST) | _BV(SS) | _BV(MOSI) | _BV(SCK);
	DDRC = _BV(LED);
	DDRD = _BV(TXD) | _BV(WICS) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);

	// Data Pullup Registers: 1=HIGH, 0=LOW
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

// initSPI: Setup SPI control register
void initSPI(void)
{
	SPI_enable();					// Enable SPI and set AVR to master
	SPI_setBitOrder(SPI_MSB_ORDER);	// Data Order: 0 - Send MSB first

	SPI_setClockDiv(SPI_CLOCK2);	// SCK Clock Rate: F_CPU/2
	SPI_setDataMode(SPI_MODE1);		// SPI Data Mode: 0b01 - Trailing, falling edge
	SPI_configStore(SPI_CC3000);	// Save current config as SPI_CC3000

	SPI_setClockDiv(SPI_CLOCK128);	// SCK Clock Rate: F_CPU/128
	SPI_setDataMode(SPI_MODE0);		// SPI Data Mode: 0b00 - Leading, rising edge
	SPI_configStore(SPI_DEFAULT);	// Save current config as SPI_DEFAULT
}
