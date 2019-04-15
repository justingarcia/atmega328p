/*
 * ----------------------------------------------------------------------------
 * FILE: basicpower.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Basic program demonstrating a digital power-off button for the
 * AVR ATMega328p MCU. Includes WLAN WI-FI connection, TCP server creation,
 * and MP3 music control. Utilizes:
 *			- TI CC3000 WI-FI module
 *			- VI VS1053b MP3 decoder module
 *			- AVR ATMega328p MCU
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
#include <avr/sleep.h>
#include <util/delay.h>

// CONSTANTS ------------------------------------------------------------------

#define WLAN_SECURITY			WLAN_SEC_WPA2
#define TCP_PORT				23

const char *WLAN_SSID			= "";
const unsigned char *WLAN_KEY	= "";

// GLOBAL VARIABLES -----------------------------------------------------------

static volatile unsigned char powerOn;

// FUNCTION MACROS ------------------------------------------------------------

#define enablePowerIRQ() {		\
	PCICR |= _BV(PCIE1);		\
	PCMSK1 |= _BV(PCINT13); }

#define disablePowerIRQ() {		\
	PCICR &= ~_BV(PCIE1);		\
	PCMSK1 &= ~_BV(PCINT13); }

// FUNCTION PROTOTYPES --------------------------------------------------------

static void start(void);
static void end(void);
static void shutdown(void);

static void init(void);
static void initIO(void);
static void initWDT(void);
static void initVS1053(void);
static void initCC3000(void);

void sClientReqCB(unsigned char *req, int len);

// FUNCTIONS ------------------------------------------------------------------

// Main Program -----------------------------------------------------------

int main(void)
{
	start();

	unsigned char new_conn;

	while (1)
	{
//		printf("Loop\n");
		new_conn = CC3000_pollConnectReq();
		if (new_conn) printf("New socket connected.\n");

		CC3000_pollClientReq();
	}

	return 0;
}

// Handlers ---------------------------------------------------------------

// sClientReqCB: TCP request handler callback
void sClientReqCB(unsigned char *req, int len)
{
	if (req[0] == '>')
	{
		switch (req[1])
		{
			case 'P':	VS1053_play("test.mp3");	return;
			case 'S':	VS1053_stop();				return;
			case 'p':	VS1053_pause();				return;
		}
	}

	else CC3000_sendData(req, len);
}

// Initialization ---------------------------------------------------------

// init: System intialization
static void init(void)
{
	cli();				// Disable global interrupts

	loop_until_bit_is_set(PINC, PWR);

	powerOn = 1;		// Set power state variable
	enablePowerIRQ();	// Enable power-down interrupt

	initIO();			// Init IO registers
	initWDT();			// Init watchdog timer
	UART_init();		// Init UART

	sei();				// Enable global interrupts

	printf("\nMCU Initialized.\n");

	initVS1053();		// Init VS1053
	initCC3000();		// Init CC3000
}

// initIO: Setup IO direction and internal pull-up register
static void initIO(void)
{
	// DATA DIRECTION: 1=OUT, 0=IN ------------------------------------

	DDRB = _BV(WIEN) | _BV(XRST) | _BV(SS) | _BV(MOSI) | _BV(SCK);
	DDRC = _BV(LED);
	DDRD = _BV(TXD) | _BV(WICS) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);

	// DATA PULLUP: 1=HIGH, 0=LOW -------------------------------------

	PORTB = _BV(XRST) | _BV(SS);
	PORTD = _BV(WICS) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);
}

// initWDT: Clear watchdog timer control bits
static void initWDT(void)
{
	wdt_reset();

	MCUSR &= ~_BV(WDRF);
	WDTCSR &= ~_BV(WDE);
}

// initVS1053: VS1053b initialization sequence
static void initVS1053(void)
{
	printf("Initializing MP3 decoder.......................");
	unsigned char vs_res = VS1053_init();
	if (!vs_res)	printf("Success!\n");
	else			printf("FAILED\n");
}

// initCC3000: WLAN client / TCP server initialization
static void initCC3000(void)
{
	CC3000_initWlanClient();

	printf("Connecting to AP...............................");
	long conn_res = CC3000_connectAP(WLAN_SECURITY, WLAN_SSID, WLAN_KEY);
	if (conn_res)	printf("Success!\n");
	else			printf("FAILED\n");

	printf("Initializing TCP server........................");
	unsigned char serv_res = CC3000_initTcpServer(TCP_PORT, sClientReqCB);
	if (serv_res)	printf("Success!\n");
	else			printf("FAILED\n");
}

// Power Management -------------------------------------------------------

// start: Wait for power-on interrupt
static void start(void)
{
	shutdown();
}

static void end(void)
{
	loop_until_bit_is_set(PINC, PWR);

	// Terminate VS1053 processes
	printf("Terminating VS1053 processes.\n");
	VS1053_term();

	// Terminate CC3000 WLAN processes
	printf("Terminating CC3000 processes.\n");
	CC3000_termTcpServer();
	CC3000_disconnectAP();
	CC3000_termWlanClient();

	printf("Shutting down!\n");
	shutdown();
}

// shutdown: Go to power-down sleep
static void shutdown(void)
{
	cli();

	// Set wake-up interrupt
	enablePowerIRQ();

	// Set power state variable
	powerOn = 0;

	// Set sleep properties
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_bod_disable();

	// Enter sleep mode
	sei();
	sleep_cpu();

	// Resume here upon waking
	sleep_disable();
	init();
}

// INTERRUPT: Power On/Off
ISR(PCINT1_vect)
{
	disablePowerIRQ();
	if (powerOn) end();
}
