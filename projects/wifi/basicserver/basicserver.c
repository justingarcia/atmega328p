/*
 * ----------------------------------------------------------------------------
 * FILE: basicserver.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Simple program implementing both WLAN client connection
 * and TCP server creation.
 * Uses the TI CC3000 WI-FI module and the AVR ATMega328p MCU.
 * ----------------------------------------------------------------------------
 */

// INCLUDE FILES --------------------------------------------------------------

#include "lib/config.h"
#include "lib/SPI.h"
#include "lib/UART.h"

#include "cc3000/wlan_client.h"
#include "cc3000/tcp_server.h"
#include "cc3000/wlan.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

// CONSTANTS ------------------------------------------------------------------

#define WLAN_SECURITY			WLAN_SEC_WPA2
#define PORT_TCP				23

const char *WLAN_SSID			= "";
const unsigned char *WLAN_KEY	= "";

// FUNCTION PROTOTYPES --------------------------------------------------------

void sClientReqCB(unsigned char *req, int len);
void init(void);
void initIO(void);
void initWDT(void);
void initSPI(void);

// FUNCTIONS ------------------------------------------------------------------

// main: Main program
int main(void)
{
	// INITIALIZE ---------------------------------------------------------

	init();
	
	printf("\n");

	CC3000_initWlanClient();

	printf("Connecting to AP.............................");
	long conn_res = CC3000_connectAP(WLAN_SECURITY, WLAN_SSID, WLAN_KEY);
	if (conn_res)	printf("Success!\n");
	else			printf("FAILED\n");

	printf("Initializing server..........................");
	unsigned char serv_res = CC3000_initTcpServer(PORT_TCP, sClientReqCB);
	if (serv_res)	printf("Success!\n");
	else			printf("FAILED\n");

	// MAIN LOOP ----------------------------------------------------------

	unsigned char new_conn;

	while (1)
	{
		new_conn = CC3000_pollConnectReq();
		if (new_conn)	printf("New socket connected.\n");

		CC3000_pollClientReq();
	}

	// TERMINATE ----------------------------------------------------------

	printf("Terminating!\n");
	CC3000_disconnectAP();
	CC3000_termWlanClient();

	return 0;
}

// sClientReqCB
void sClientReqCB(unsigned char *req, int len)
{
	CC3000_sendData(req, len);
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

// initSPI: Setup SPI control register and store config profiles
void initSPI(void)
{
	// GENERAL SETTINGS ---------------------------------------------------

	SPI_enable();					// Enable SPI and set AVR to master
	SPI_setBitOrder(SPI_MSB_ORDER);	// Data Order: Send MSB first

	// CC3000 CONFIG ------------------------------------------------------

	SPI_setClockDiv(SPI_CLOCK2);	// SCK Clock Rate: F_CPU/2
	SPI_setDataMode(SPI_MODE1);		// SPI Data Mode: Trailing, falling edge
	SPI_configStore(SPI_CC3000);	// Save current config as SPI_CC3000

	// DEFAULT CONFIG -----------------------------------------------------

	SPI_setClockDiv(SPI_CLOCK128);	// SCK Clock Rate: F_CPU/128
	SPI_setDataMode(SPI_MODE0);		// SPI Data Mode: Leading, rising edge
	SPI_configStore(SPI_DEFAULT);	// Save current config as SPI_DEFAULT
}
