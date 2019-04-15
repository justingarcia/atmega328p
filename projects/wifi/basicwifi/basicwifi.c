/*
 * ----------------------------------------------------------------------------
 * FILE: basicwifi.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Demonstrates basic WI-FI capabilities, using the AVR ATMega328p MCU
 * and the TI CC3000 WI-FI module.
 * ----------------------------------------------------------------------------
 */

#include "lib/config.h"
#include "lib/SPI.h"
#include "lib/UART.h"

//#include "fatfs/pff.h"

#include "cc3000/ccspi.h"
#include "cc3000/hci.h"
#include "cc3000/wlan.h"
#include "cc3000/socket.h"
#include "cc3000/netapp.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>

// CONSTANTS --------------------------------------------------------------

#define WLAN_SECURITY	WLAN_SEC_WPA2

const char *WLAN_SSID	= "";
const char *WLAN_PASS	= "";

// PROTOTYPES -------------------------------------------------------------

void init(void);
void initIO(void);
void initWDT(void);
void initSPI(void);
void sWlanCB(long lEventType, char *data, unsigned char length);
void printIP(uint32_t ip);
void printIPrev(uint32_t ip);

void blink(int n)
{
	do
	{
		PORTC |= _BV(LED);
		_delay_ms(100);
		PORTC &= ~_BV(LED);
		_delay_ms(100);
	}
	while(--n);
}

// GLOBAL VARIABLES -------------------------------------------------------

volatile unsigned char dhcp_connected = 0;

// FUNCTIONS --------------------------------------------------------------

// main: Main program
int main(void)
{
	init();
	printf("\n");
	printf("MCU initialized.\n");

	// Initialize WLAN driver
	printf("Initializing WLAN driver..............");
	wlan_init
	(
		sWlanCB,					// Callback: Event handler
		sendWLFWPatch,				// Returns NULL: No FW patches
		sendDriverPatch,			// Returns NULL: No driver patches
		sendBootLoaderPatch,		// Returns NULL: No bootloader patches
		sReadWlanInterruptPin,		// Callback: Read WLAN IRQ
		sWlanInterruptEnable,		// Callback: Enable WLAN IRQ
		sWlanInterruptDisable,		// Callback: Disable WLAN IRQ
		sWriteWlanPin				// Callback: Write WLAN-EN
	);
	printf("Done!\n");

	// Assert the enable pin of the WLAN device
	printf("Starting WLAN device..................");
	wlan_start(0);					// No host patches, normal operation
	printf("Done!\n");

	// Set manual connection policy
	wlan_ioctl_set_connection_policy(0, 0, 0);

	// Delete previous profiles from memory
	wlan_ioctl_del_profile(255);

	// Connect to WI-FI access point
	printf("Connecting to WI-FI access point......");
	if (wlan_connect
	(
		WLAN_SECURITY,					// Type of WI-FI security
		(char *)WLAN_SSID,				// SSID of the AP
		strlen(WLAN_SSID),				// Length of the SSID string
		NULL,							// BSSID
		(unsigned char *)WLAN_PASS,		// AP security key
		strlen(WLAN_PASS)				// Length of the AP security key
	) == 0)
	{
		printf("Done!\n");
	}
	else
	{
		printf("Error\n");
	}

	// Wait for DHCP to complete
	printf("Requesting DHCP.......................");
	while(!dhcp_connected)
	{
		_delay_ms(100);
	}
	printf("Done!\n");

	printf("\nConnection established!\n");

	// Display connection details
	printf("\nConnection details:\n");
	tNetappIpconfigRetArgs ipconfig;
	netapp_ipconfig(&ipconfig);

	printf("SSID: ");		for (uint8_t i = 0; i < 4; i++) printf("%c", ipconfig.uaSSID[i]);	printf("...\n");
	printf("IP: ");			printIPrev(*(uint32_t *)ipconfig.aucIP);
	printf("SUBNET: ");		printIPrev(*(uint32_t *)ipconfig.aucSubnetMask);
	printf("GATEWAY: ");	printIPrev(*(uint32_t *)ipconfig.aucDefaultGateway);
	printf("DHCP: ");		printIPrev(*(uint32_t *)ipconfig.aucDHCPServer);
	printf("DNS: ");		printIPrev(*(uint32_t *)ipconfig.aucDNSServer);
	printf("MAC: ");		printIPrev(*(uint32_t *)ipconfig.uaMacAddr);

	// Perform a test ping
	printf("\nPinging www.google.com:\n");

	char *hostname = "www.google.com";
	uint32_t ip = 0;
	uint16_t i = 10;
	do
	{
		_delay_ms(500);
		gethostbyname(hostname, strlen(hostname), &ip);
	}
	while (ip == 0 && --i);

	if (ip != 0)
	{
		printf("IP: ");		printIPrev(ip);

		uint32_t revIP = (ip << 24) | ((ip & 0xFF00) << 8) | ((ip >> 8) & 0xFF00) | (ip >> 24);

		if (netapp_ping_send(&revIP, 3, 500, 32) == 0)
		{
			printf("Ping successful!\n");
		}
		else
		{
			printf("Ping unsuccessful...\n");
		}
	}
	

	// Disconnect WLAN and deactivate device
	printf("\nDisconnecting.........................");
	wlan_disconnect();
	wlan_stop();
	printf("Done!\n");

	while(1)
	{

	}

	return 0;
}

void sWlanCB(long lEventType, char *data, unsigned char length)
{
	switch(lEventType)
	{
		case HCI_EVNT_WLAN_UNSOL_CONNECT:
			return;

		case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
			return;

		case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:
			return;

		case HCI_EVNT_WLAN_UNSOL_DHCP:
			dhcp_connected = 1;
			return;

		case HCI_EVNT_WLAN_ASYNC_PING_REPORT:
			return;

		case HCI_EVNT_WLAN_KEEPALIVE:
			return;

		default: return;
	}
}

void printIP(uint32_t ip)
{
	printf("%d", (uint8_t)(ip >> 0));
	printf(".");
	printf("%d", (uint8_t)(ip >> 8));
	printf(".");
	printf("%d", (uint8_t)(ip >> 16));
	printf(".");
	printf("%d", (uint8_t)(ip >> 24));
	printf("\n");
}

void printIPrev(uint32_t ip)
{
	printf("%d", (uint8_t)(ip >> 24));
	printf(".");
	printf("%d", (uint8_t)(ip >> 16));
	printf(".");
	printf("%d", (uint8_t)(ip >> 8));
	printf(".");
	printf("%d", (uint8_t)(ip >> 0));
	printf("\n");

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
	DDRB = _BV(WIEN) | _BV(XRST) | _BV(SS) | _BV(MOSI) | _BV(SCK);	// MISO=IN
	DDRC = _BV(LED);												// NULL IN
	DDRD = _BV(TXD) | _BV(WICS) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);	// RXD,XDRQ,WIRQ=IN

	// Data Pullup Registers: 1=HIGH, 0=LOW
	PORTB = _BV(XRST) | _BV(SS);							// WIEN=LOW(INACTIVE)
	PORTC = 0x00;											// ALL LOW
	PORTD = _BV(WICS) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);	// ---
}

// initWDT: Clear Watchdog Timer control bits
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
