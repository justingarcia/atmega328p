/*
 * File: basicmp3.c
 * Author: Justin Garcia
 * ---------------------
 * A basic mp3 player, using the ATmega328p microcontroller
 * and the VS1053b mp3 codec.
 */

#include "config.h"
#include "SPI.h"
#include "UART.h"
#include "pff.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define VS1053_OP_SCI_READ		0x03
#define VS1053_OP_SCI_WRITE		0x02

#define VS1053_SCI_MODE			0x00
#define VS1053_SCI_STATUS		0x01
#define VS1053_SCI_BASS			0x02
#define VS1053_SCI_CLOCKF		0x03
#define VS1053_SCI_DECODE_TIME	0x04
#define VS1053_SCI_AUDATA		0x05
#define VS1053_SCI_WRAM			0x06
#define VS1053_SCI_WRAMADDR		0x07
#define VS1053_SCI_HDAT0		0x08
#define VS1053_SCI_HDAT1		0x09
#define VS1053_SCI_AIADDR		0x0A
#define VS1053_SCI_VOL			0x0B
#define VS1053_SCI_AICTRL0		0x0C
#define VS1053_SCI_AICTRL1		0x0D
#define VS1053_SCI_AICTRL2		0x0E
#define VS1053_SCI_AICTRL3		0x0F

#define VS1053_SM_DIFF			0x0001
#define VS1053_SM_LAYER12		0x0002
#define VS1053_SM_RESET			0x0004
#define VS1053_SM_CANCEL		0x0008
#define VS1053_SM_EARSPEAKER_LO	0x0010
#define VS1053_SM_TESTS			0x0020
#define VS1053_SM_STREAM		0x0040
#define VS1053_SM_EARSPEAKER_HI	0x0080
#define VS1053_SM_DACT			0x0100
#define VS1053_SM_SDIORD		0x0200
#define VS1053_SM_SDISHARE		0x0400
#define VS1053_SM_SDINEW		0x0800
#define VS1053_SM_ADPCM			0x1000
#define VS1053_SM_LINE1			0x4000
#define VS1053_SM_CLK_RANGE		0x8000

#define VS1053_BUFFER				0x80
#define VS1053_SDI_MAX				0x20
#define VS1053_CANCEL_MAX			0x40
#define VS1053_END_FILL_BYTE_ADDR	0x1E06
#define VS1053_END_FILL_BYTE_BLOCK	0x41
#define VS1053_DEF_CLOCK			0x8000
#define VS1053_DEF_VOL				0x10

/* System Dependencies */

#define selectXCS()		PORTD &= ~_BV(XCS)
#define deselectXCS()	PORTD |= _BV(XCS)

#define selectXDCS()	PORTD &= ~_BV(XDCS)
#define deselectXDCS()	PORTD |= _BV(XDCS)

#define selectXRST()	PORTB &= ~_BV(XRST)
#define deselectXRST()	PORTB |= _BV(XRST)

#define waitForDREQ()	loop_until_bit_is_set(PIND, XDRQ)

/* Global Variables */

static FATFS pf_fs;
static FRESULT pf_res;
static uint8_t buff[VS1053_BUFFER];

static uint8_t VS1053_is_paused;
static uint8_t VS1053_is_cancelled;

/* Prototypes */

void init(void);
void initIO(void);
void initSPI(void);

uint8_t VS1053_play(char *track);
void VS1053_pause(void);
void VS1053_stop(void);
uint8_t VS1053_SDI_end(void);
void VS1053_reset_hard(void);
void VS1053_reset_soft(void);
void VS1053_set_volume(uint8_t left, uint8_t right);
uint16_t VS1053_SCI_read(uint8_t addr);
void VS1053_SCI_write(uint8_t addr, uint16_t data);
void VS1053_SDI_write(uint8_t *buffer, uint16_t buffsize);

void test(void)
{
	PORTC |= _BV(LED);
	_delay_ms(3000);
	PORTC &= ~_BV(LED);
	_delay_ms(3000);
}


/* Functions */

// main: Main program
int main(void)
{
	init();

	VS1053_SCI_write(VS1053_SCI_MODE, VS1053_SM_SDINEW | VS1053_SM_TESTS);
	
	uint8_t sine_start[] = {0x53, 0xEF, 0x6E, 170, 0, 0, 0, 0};
	uint8_t sine_end[] = {0x45, 0x78, 0x69, 0x74, 0, 0, 0, 0};

	VS1053_SDI_write(sine_start, 8);
	_delay_ms(3000);
	VS1053_SDI_write(sine_end, 8);

	VS1053_play("test.mp3");

	while(1)
	{

	}

	return 0;
}

// init: System initialization
void init(void)
{
	_delay_ms(100);

	initIO();
	initSPI();
	UART_init();
	
	pf_res = pf_mount(&pf_fs);
	printf("Mounting... Error: %d\n", pf_res);

	VS1053_reset_hard();

	SPI_setClockDiv(SPI_CLOCK16);
	SPSR |= _BV(SPI2X);
}

// initIO: Setup IO direction and pull-up registers
void initIO(void)
{
	// Data Direction Registers: 1=OUT, 0=IN
	DDRB = _BV(XRST) | _BV(SS) | _BV(MOSI) | _BV(SCK);	// MISO=IN
	DDRC = _BV(LED);									// ---
	DDRD = _BV(TXD) | _BV(SDCS) | _BV(XDCS) | _BV(XCS);	// RXD=IN

	// Data Registers: 1=HIGH, 0=LOW
	PORTB = _BV(XRST) | _BV(SS);
	PORTC = 0x00;
	PORTD = _BV(SDCS) | _BV(XDCS) | _BV(XCS);
}

// initSPI: Setup SPI control register
void initSPI(void)
{
	SPI_enable();					// Enable SPI and set AVR to master
	SPI_setBitOrder(SPI_MSB_ORDER);	// Data Order: 0 - Send MSB first
	SPI_setClockDiv(SPI_CLOCK128);	// SCK Clock Rate: 0b11 -  Set to F_CPU/128
	SPI_setDataMode(SPI_MODE0);		// SPI Data Mode: 0b00 - Leading, rising edge
}

uint8_t VS1053_play(char *track)
{
	uint16_t bytes_read = 0;
	pf_res = pf_open(track);
	printf("Opening... Error: %d\n", pf_res);
	if (pf_res) return 1;

	VS1053_is_paused = 0;
	VS1053_is_cancelled = 0;
	uint8_t cancel_counter = 0;

	while(1)
	{
		while(VS1053_is_paused);

		if(VS1053_is_cancelled)
		{
			if( !(VS1053_SCI_read(VS1053_SCI_MODE) & _BV(VS1053_SM_CANCEL)) )
			{
				VS1053_SDI_end();
				return 0;
			}
			else cancel_counter++;

			if (cancel_counter >= VS1053_CANCEL_MAX)
			{
				VS1053_reset_soft();
				return 1;
			}
		}

		bytes_read = 0;
		pf_res = pf_read(buff, VS1053_BUFFER, &bytes_read);
			
		if (pf_res || bytes_read != VS1053_BUFFER)
		{
			VS1053_SDI_end();
			if (pf_res) return 1;
			if (bytes_read != VS1053_BUFFER) return 0;
		}

		VS1053_SDI_write(buff, VS1053_BUFFER);
	}
}

void VS1053_pause(void)
{
	VS1053_is_paused ^= 1;
}

void VS1053_stop(void)
{
	VS1053_SCI_write(VS1053_SCI_MODE, VS1053_SM_SDINEW | VS1053_SM_CANCEL);
	VS1053_is_paused = 0;
	VS1053_is_cancelled = 1;
}

uint8_t VS1053_SDI_end(void)
{
	VS1053_SCI_write(VS1053_SCI_WRAMADDR, VS1053_END_FILL_BYTE_ADDR);
	uint8_t end_fill_byte = VS1053_SCI_read(VS1053_SCI_WRAM);

	uint8_t efb_block[VS1053_SDI_MAX];
	for (uint8_t i = 0; i < VS1053_SDI_MAX; i++)
	{
		efb_block[i] = end_fill_byte;
	}

	uint8_t j = 0;
	while( VS1053_SCI_read(VS1053_SCI_HDAT0) || VS1053_SCI_read(VS1053_SCI_HDAT1) )
	{
		VS1053_SDI_write(efb_block, VS1053_SDI_MAX);
		
		if (j > VS1053_END_FILL_BYTE_BLOCK)
		{
			VS1053_reset_soft();
			return 1;
		}
		j++;
	}

	printf("Song finished.\n");
	return 0;
}

// VS1053_reset_hard: Hardware reset
void VS1053_reset_hard(void)
{
	selectXRST();											// Pull RESET line LOW to trigger hardware reset

	_delay_ms(100);

	deselectXCS();											// Pull SCI and SDI slave selects to default HIGH
	deselectXDCS();

	deselectXRST();											// Pull RESET line back to default HIGH

	VS1053_SCI_write(VS1053_SCI_MODE, VS1053_SM_SDINEW);	// Set default SCI_MODE register configuration
	VS1053_SCI_write(VS1053_SCI_BASS, 0x0000);				// Set default BASS register configuration (no bass enhacement)
	VS1053_SCI_write(VS1053_SCI_CLOCKF, VS1053_DEF_CLOCK);	// Set default CLOCK value (internal clock x3)
	VS1053_set_volume(VS1053_DEF_VOL, VS1053_DEF_VOL);		// Set default VOLUME
}

// VS1053_reset_soft: Software reset
void VS1053_reset_soft(void)
{
	VS1053_SCI_write(VS1053_SCI_MODE, VS1053_SM_SDINEW | VS1053_SM_RESET);
	waitForDREQ();
}

void VS1053_set_volume(uint8_t left, uint8_t right)
{
	uint16_t volume;
	volume = left;
	volume <<= 8;
	volume |= right;

	cli();
	VS1053_SCI_write(VS1053_SCI_VOL, volume);
	sei();
}

// VS1053_SCI_read: Performs VS1053 Serial Command Interface acquisition
uint16_t VS1053_SCI_read(uint8_t addr)
{
	uint16_t data;

	waitForDREQ();						// Wait until VS1053 is capable of receiving 

	selectXCS();						// Pull SCI slave select LOW to select device

	SPI_transfer(VS1053_OP_SCI_READ);	// Transmit the READ opcode
	SPI_transfer(addr);					// Transmit the 8-bit address
	data = SPI_transfer(0x00);			// Receive the MSB of the 16-bit data
	data <<= 8;
	data |= SPI_transfer(0x00);			// Receive the LSB of the 16-bit data

	deselectXCS();						// Pull SCI slave select HIGH to end the READ sequence

	return data;
}

// VS1053_SCI_write: Performs VS1053 Serial Command Interface transmission
void VS1053_SCI_write(uint8_t addr, uint16_t data)
{
	waitForDREQ();						// Wait until VS1053 is capable of receiving

	selectXCS();						// Pull SCI slave select LOW to select device

	SPI_transfer(VS1053_OP_SCI_WRITE);	// Transmit the WRITE opcode
	SPI_transfer(addr);					// Transmit the 8-bit address
	SPI_transfer(data >> 8);			// Transmit the MSB of the 16-bit data
	SPI_transfer(data >> 0);			// Transmit the LSB of the 16-bit data

	deselectXCS();						// Pull SCI slave select HIGH to end the WRITE sequence
}

// VS1053_SDI_write: Performs VS1053 Serial Data Interface transmission
void VS1053_SDI_write(uint8_t *buffer, uint16_t buffsize)
{
	waitForDREQ();								// Wait until VS1053 is capable of receiving

	selectXDCS();								// Pull SDI slave select LOW to select device
	
	uint8_t dreq_check_count = 0;
	for(uint16_t i = 0; i < buffsize; i++)
	{
		SPI_transfer(buffer[i]);				// Transmit the contents of the buffer byte by byte

		dreq_check_count++;
		if (dreq_check_count >= VS1053_SDI_MAX)	// if VS1053_SDI_MAX = 32 bytes have been sent
		{
			deselectXDCS();						// Pull SDI slave select HIGH (XDCS should be refreshed periodically)
			waitForDREQ();						// Check DREQ every 32 bytes
			selectXDCS();						// Pull SDI slave select LOW to re-enable device

			dreq_check_count = 0;				// Reset the 32-byte counter
		}

	}

	deselectXDCS();								// Pull SDI slave select HIGH to end the WRITE sequence
}
