/*
 * ----------------------------------------------------------------------------
 * FILE: chatmusic.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Simple program implementing WLAN client creation, Telnet TCP server
 * creation, and backround mp3 music control. Utilizes:
 *		- TI CC3000 WI-FI module
 *		- VI VS1053B MP3 decoder module
 *		- AVR ATMega328p MCU
 * ----------------------------------------------------------------------------
 */

// INCLUDE FILES --------------------------------------------------------------

#include "lib/config.h"
#include "lib/UART.h"

#include "cc3000/wlan_client.h"
#include "cc3000/tcp_server.h"
#include "cc3000/wlan.h"

#include "vs1053/player.h"

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

void init(void);
void initIO(void);
void initWDT(void);

void sClientReqCB(unsigned char *req, int len);

// FUNCTIONS ------------------------------------------------------------------

// main: Main program
int main(void)
{
	// INITIALIZE ---------------------------------------------------------

	init();
	printf("\nMCU Initialized.\n");

	printf("Initializing MP3 decoder.....................");
	unsigned char dec_res = VS1053_init();
	if (!dec_res)	printf("Success!\n");
	else			printf("FAILED\n");

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
/*
	printf("Playing MP3..................................");
	unsigned char play_res = VS1053_play("test.mp3");
	if (!play_res)	printf("Success!\n");
	else			printf("FAILED\n");
*/
	unsigned char new_conn;

	while (1)
	{
		new_conn = CC3000_pollConnectReq();
		if (new_conn) printf("New socket connected.\n");

		CC3000_pollClientReq();
	}

	// TERMINATE ----------------------------------------------------------

	printf("Terminating!\n");
	CC3000_disconnectAP();
	CC3000_termWlanClient();

	return 0;
}

// sClientReqCB: TCP request handler callback
void sClientReqCB(unsigned char *req, int len)
{
	switch (req[0])
	{
		case 'p':	VS1053_play("test.mp3");	return;
		case ' ':	VS1053_pause();				return;
		case 'S':	VS1053_stop();				return;
		default:	CC3000_sendData(req, len);	return;
	}
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
