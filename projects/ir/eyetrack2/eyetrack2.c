/*
 * ----------------------------------------------------------------------------
 * FILE: eyetrack2.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Basic program demonstrating IR eye tracking on the AVR ATMega328p MCU.
 * Features digital calibration and 2-axis tracking.
 * Includes analog-to-digital conversion, digital power management,
 * WLAN WI-FI connection, TCP server creation, and MP3 file control.
 * Utilizes:
 *			- TI CC3000 WI-FI module
 *			- VI VS1053b MP3 decoder module
 *			- AVR ATMega328p MCU
 * ----------------------------------------------------------------------------
 */

// INCLUDE FILES --------------------------------------------------------------

#include "lib/config.h"
#include "lib/ADC.h"
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
#include <string.h>

// DATA STRUCTURES ------------------------------------------------------------

typedef struct
{
	uint8_t	pin;
	int16_t	lo, hi;
	int16_t	midLo, midHi;	
	int16_t	adcAvg;

} irdet_t;

// CONSTANTS ------------------------------------------------------------------

// WLAN Constants ---------------------------------------------------------

#define WLAN_SECURITY			WLAN_SEC_WPA2
#define TCP_PORT				23

const char *WLAN_SSID			= "";
const unsigned char *WLAN_KEY	= "";

// Eye Detection Constants ------------------------------------------------

#define DIR_ST				0

#define DIR_UP				1
#define DIR_RT				2
#define DIR_DN				3
#define DIR_LT				4

#define DIR_UR				5
#define DIR_DR				6
#define DIR_DL				7
#define DIR_UL				8

#define ADC_SAMPLE_SIZE		16
#define PADDING_SIZE		4

// GLOBAL VARIABLES -----------------------------------------------------------

static volatile unsigned char powerOn;

static uint8_t direction;
static irdet_t ir_RO, ir_LO, ir_LI;

// FUNCTION MACROS ------------------------------------------------------------

#define enablePowerIRQ() {			\
	PCICR |= _BV(PCIE1);			\
	PCMSK1 |= _BV(PCINT13); }

#define disablePowerIRQ() {			\
	PCICR &= ~_BV(PCIE1);			\
	PCMSK1 &= ~_BV(PCINT13); }

#define movAvg(avg, val, n)			avg += (val - avg) / n

// FUNCTION PROTOTYPES --------------------------------------------------------

static void start(void);
static void end(void);
static void shutdown(void);

static void init(void);
static void initIO(void);
static void initWDT(void);
static void initIR(void);
static void initVS1053(void);
static void initCC3000(void);

static void calibrateDetectors(void);
static void calibrate(irdet_t *d);

static void pollDetectors(void);
static void updateDirection(void);
void handleTcpReq(unsigned char *req, int len);

// FUNCTIONS ------------------------------------------------------------------

// Main Program -----------------------------------------------------------

int main(void)
{
	// Initialization -------------------------------------------------

	start();
	calibrateDetectors();

	// Main Loop ------------------------------------------------------

	unsigned char new_conn;

	while (1)
	{
//		new_conn = CC3000_pollConnectReq();
//		if (new_conn) printf("New socket connected.\n");

//		CC3000_pollClientReq();

		pollDetectors();
		updateDirection();
	}

	// Termination ----------------------------------------------------

	return 0;
}

// Handlers ---------------------------------------------------------------

// pollDetectors: IR detector handler
static void pollDetectors(void)
{
	movAvg(ir_RO.adcAvg,	ADC_read(ir_RO.pin),	ADC_SAMPLE_SIZE);
	movAvg(ir_LO.adcAvg,	ADC_read(ir_LO.pin),	ADC_SAMPLE_SIZE);
	movAvg(ir_LI.adcAvg,	ADC_read(ir_LI.pin),	ADC_SAMPLE_SIZE);

//	printf("LO: %d LI: %d RO: %d\n", ir_LO.adcAvg, ir_LI.adcAvg, ir_RO.adcAvg);
}

// updateDirection: Update eye direction state
static void updateDirection(void)
{
	switch (direction)
	{
		case DIR_ST:
			if (
				ir_LO.adcAvg >= ir_LO.hi
				&& ir_LI.adcAvg <= ir_LI.lo
				&& ir_RO.adcAvg <= ir_RO.lo
			) 
			{
				direction = DIR_RT;
				printf("RIGHT: %d\n", ir_RO.adcAvg);
				VS1053_playFull("d_rt.mp3");
			}
			else if (
				ir_LO.adcAvg <= ir_LO.lo
				&& ir_LI.adcAvg >= ir_LI.hi
				&& ir_RO.adcAvg >= ir_RO.hi
			)
			{
				direction = DIR_LT;
				printf("LEFT: %d\n", ir_LO.adcAvg);
				VS1053_playFull("d_lt.mp3");
			}
			else if (
				ir_LO.adcAvg >= ir_LO.midHi
				&& ir_LO.adcAvg <= ir_LO.hi
				&& ir_LI.adcAvg >= ir_LI.midLo
				&& ir_LI.adcAvg <= ir_LI.midHi
				&& ir_RO.adcAvg >= ir_RO.hi
			)
			{
				direction = DIR_UP;
				printf("UP: %d\n", ir_LO.adcAvg);
				VS1053_playFull("d_up.mp3");
			}
			else if (
				ir_LO.adcAvg >= ir_LO.midHi
				&& ir_LO.adcAvg <= ir_LO.hi
				&& ir_LI.adcAvg >= ir_LI.lo
				&& ir_LI.adcAvg <= ir_LI.midLo
				&& ir_RO.adcAvg >= ir_RO.hi
			)
			{
				direction = DIR_DN;
				printf("DOWN: ");
				printf("%d\n", ir_RO.adcAvg);
				VS1053_playFull("d_dn.mp3");
			}
/*			else if (
				
			)
			{

			}*/
			else if (
				ir_LO.adcAvg >= ir_LO.hi
				&& ir_LI.adcAvg >= ir_LI.midLo
				&& ir_LI.adcAvg <= ir_LI.midHi
				&& ir_RO.adcAvg >= ir_RO.hi
			)
			{
				direction = DIR_UL;
				printf("UP-LEFT\n");
				VS1053_playFull("6.mp3");
			}
			else if (
				ir_LO.adcAvg >= ir_LO.hi
				&& ir_LI.adcAvg <= ir_LI.lo
				&& ir_RO.adcAvg >= ir_RO.hi
			)
			{
				direction = DIR_DR;
				printf("DOWN-RIGHT\n");
				VS1053_playFull("7.mp3");
			}
/*			else if (
			)
			{

			}*/

			return;

		case DIR_RT:
			if (
				ir_LO.adcAvg <= ir_LO.midHi
				&& ir_LI.adcAvg >= ir_LI.midLo 
				&& ir_RO.adcAvg >= ir_RO.midLo
			)
			{
				direction = DIR_ST;
				printf("CENTER: %d\n", ir_RO.adcAvg);
				VS1053_playFull("d_st.mp3");
			}
			return;

		case DIR_LT:
			if (
				ir_LO.adcAvg >= ir_LO.midLo
				&& ir_LI.adcAvg <= ir_LI.midHi
				&& ir_RO.adcAvg <= ir_RO.midHi
			)
			{
				direction = DIR_ST;
				printf("CENTER: %d\n", ir_LO.adcAvg);
				VS1053_playFull("d_st.mp3");
			}
			return;

		case DIR_UP:
			if (
				ir_LO.adcAvg <= ir_LO.midHi
				&& ir_RO.adcAvg <= ir_RO.hi
			)
			{
				direction = DIR_ST;
				printf("CENTER: %d\n", ir_LO.adcAvg);
				VS1053_playFull("d_st.mp3");
			}
			return;

		case DIR_DN:
			if (
				ir_LO.adcAvg <= ir_LO.midHi
				&& ir_LI.adcAvg >= ir_LI.midLo
				&& ir_RO.adcAvg <= ir_RO.hi
			)
			{
				direction = DIR_ST;
				printf("CENTER: %d\n", ir_RO.adcAvg);
				VS1053_playFull("d_st.mp3");
			}
			return;

		case DIR_UL:
			if (
				ir_LO.adcAvg <= ir_LO.midHi
				&& ir_RO.adcAvg <= ir_RO.midHi
			)
			{
				direction = DIR_ST;
				printf("CENTER\n");
				VS1053_playFull("d_st.mp3");
			}
			return;

		case DIR_DR:
			if (
				ir_LO.adcAvg <= ir_LO.midHi
				&& ir_LI.adcAvg >= ir_LI.midLo
				&& ir_RO.adcAvg <= ir_RO.midHi
			)
			{
				direction = DIR_ST;
				printf("CENTER\n");
				VS1053_playFull("d_st.mp3");
			}
			return;
	}
}

/*
// handleTcpReq: TCP request handler callback
void handleTcpReq(unsigned char *req, int len)
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
*/

// Initialization ---------------------------------------------------------

// calibrateDetectors: Calibrate IR detector thresholds
static void calibrateDetectors(void)
{
	int16_t midLO, midLI, midRO;
	int16_t pad = 0;

	// Initial ADC read to stabilize values
	for (uint16_t i = 0; i < 500; i++)
	{
		movAvg(ir_RO.adcAvg, ADC_read(ir_RO.pin), ADC_SAMPLE_SIZE);
		movAvg(ir_LO.adcAvg, ADC_read(ir_LO.pin), ADC_SAMPLE_SIZE);
		movAvg(ir_LI.adcAvg, ADC_read(ir_LI.pin), ADC_SAMPLE_SIZE);

		_delay_ms(10);
	}

	VS1053_playFull("d_c_beg.mp3");

	// CENTER ---------------------------------------------------------

	// Instruction message
	VS1053_playFull("d_c_st.mp3");
	_delay_ms(1000);

	// Poll ADC
	for (uint16_t i = 0; i < 400; i++)
	{
		movAvg(ir_LO.adcAvg, ADC_read(ir_LO.pin), ADC_SAMPLE_SIZE);
		movAvg(ir_RO.adcAvg, ADC_read(ir_RO.pin), ADC_SAMPLE_SIZE);
		movAvg(ir_LI.adcAvg, ADC_read(ir_LI.pin), ADC_SAMPLE_SIZE);
		_delay_ms(10);
	}

	// Set calibrated values
	midLO = ir_LO.adcAvg;
	midLI = ir_LI.adcAvg;
	midRO = ir_RO.adcAvg;

	VS1053_playFull("ui_good.mp3");


	// RIGHT ----------------------------------------------------------

	// Instruction message
	VS1053_playFull("d_c_look.mp3");
	VS1053_playFull("d_rt.mp3");
	_delay_ms(1000);

	// Poll ADC
	for (uint16_t i = 0; i < 400; i++)
	{
		movAvg(ir_LO.adcAvg, ADC_read(ir_LO.pin), ADC_SAMPLE_SIZE);
		movAvg(ir_RO.adcAvg, ADC_read(ir_RO.pin), ADC_SAMPLE_SIZE);
		movAvg(ir_LI.adcAvg, ADC_read(ir_LI.pin), ADC_SAMPLE_SIZE);
		_delay_ms(10);
	}

	// Set calibrated values
	pad = (ir_LO.adcAvg - midLO) >> PADDING_SIZE;
	ir_LO.hi = ir_LO.adcAvg - pad;
	ir_LO.midHi = midLO + pad;

	pad = (midRO - ir_RO.adcAvg) >> PADDING_SIZE;
	ir_RO.lo = ir_RO.adcAvg + pad;
	ir_RO.midLo = midRO - pad;

	pad = (midLI - ir_LI.adcAvg) >> PADDING_SIZE;
	ir_LI.lo = ir_LI.adcAvg + pad;
	ir_LI.midLo = midLI - pad;

	VS1053_playFull("ui_good.mp3");

	// LEFT -----------------------------------------------------------

	// Instruction message
	VS1053_playFull("d_c_look.mp3");
	VS1053_playFull("d_lt.mp3");
	_delay_ms(1000);

	// Poll ADC
	for (uint16_t i = 0; i < 400; i++)
	{
		movAvg(ir_LO.adcAvg, ADC_read(ir_LO.pin), ADC_SAMPLE_SIZE);
		movAvg(ir_LI.adcAvg, ADC_read(ir_LI.pin), ADC_SAMPLE_SIZE);
		movAvg(ir_RO.adcAvg, ADC_read(ir_RO.pin), ADC_SAMPLE_SIZE);
		_delay_ms(10);
	}
/*
	// Poll ADC for minimum
	for (uint16_t i = 0; i < 400; i++)
	{
//		d->adcAvg = ADC_read(d->pin);
//		if (d->adcAvg < min) min = d->adcAvg;
		movAvg(d->adcAvg, ADC_read(d->pin), ADC_SAMPLE_SIZE);
		_delay_ms(10);
	}
	int16_t lo = d->adcAvg;
*/

	// Set calibrated values
	pad = (midLO - ir_LO.adcAvg) >> PADDING_SIZE;
	ir_LO.lo = ir_LO.adcAvg + pad;
	ir_LO.midLo = midLO - pad;

	pad = (ir_LI.adcAvg - midLO) >> PADDING_SIZE;
	ir_LI.hi = ir_LI.adcAvg - pad;
	ir_LI.midHi = midLI + pad;

	pad = (ir_RO.adcAvg - midRO) >> PADDING_SIZE;
	ir_RO.hi = ir_RO.adcAvg - pad;
	ir_RO.midHi = midRO + pad;

	VS1053_playFull("ui_splen.mp3");
	VS1053_playFull("d_c_end.mp3");

	printf("LO SETTINGS: LO %d MIDLO %d MIDHI %d HI %d\n", ir_LO.lo, ir_LO.midLo, ir_LO.midHi, ir_LO.hi);
	printf("LI SETTINGS: HI %d MIDHI %d MIDLO %d LO %d\n", ir_LI.hi, ir_LI.midHi, ir_LI.midLo, ir_LI.lo);
	printf("RO SETTINGS: HI %d MIDHI %d MIDLO %d LO %d\n", ir_RO.hi, ir_RO.midHi, ir_RO.midLo, ir_RO.lo);
}

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
	initIR();			// Init IR detection

	sei();				// Enable global interrupts

	printf("\nMCU Initialized.\n");

	initVS1053();		// Init VS1053

	VS1053_playFull("ui_start.mp3");
	VS1053_playFull("ui_activ.mp3");

//	initCC3000();		// Init CC3000
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

// initIR: Initialize ADC registers and IR detection state variables
static void initIR(void)
{
	// Init analog-to-digital conversion functionality
	ADC_init();

	// Init state variables
	ir_RO.pin = ADC1;
	ir_RO.adcAvg = 0;

	ir_LO.pin = ADC2;
	ir_LO.adcAvg = 0;

	ir_LI.pin = ADC3;
	ir_LI.adcAvg = 0;

	direction = DIR_ST;
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
	unsigned char serv_res = CC3000_initTcpServer(TCP_PORT, handleTcpReq);
	if (serv_res)	printf("Success!\n");
	else			printf("FAILED\n");
}

// Power Management -------------------------------------------------------

// start: Wait for power-on interrupt
static void start(void)
{
	shutdown();
}

// end: Terminate all processes, prepare for power-down
static void end(void)
{
	loop_until_bit_is_set(PINC, PWR);

	VS1053_playFull("ui_end.mp3");
	printf("\n\n");

	// Terminate MCU processes
	ADC_term();

	// Terminate VS1053 processes
	printf("Terminating VS1053 processes.\n");
	VS1053_term();

	// Terminate CC3000 WLAN processes
	printf("Terminating CC3000 processes.\n");
//	CC3000_termTcpServer();
//	CC3000_disconnectAP();
//	CC3000_termWlanClient();

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
