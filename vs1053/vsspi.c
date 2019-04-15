/*
 * ----------------------------------------------------------------------------
 * FILE: vsspi.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Provides low-level Serial Command Interface and Serial Data Interface
 * communication for the VS1053 mp3 decoder.
 * Source file.
 * ----------------------------------------------------------------------------
 */

// INCLUDE FILES --------------------------------------------------------------

#include "vsspi.h"

// DRIVER SETTINGS ------------------------------------------------------------

#include "lib/SPI.h"
#include <avr/interrupt.h>

// DRIVER FUNCTIONS -------------------------------------------------------

#define VS1053_ASSERT_CS {							\
	SPI_configPush(SPI_VS1053);						\
	PORTD &= ~_BV(XCS); }

#define VS1053_DEASSERT_CS {						\
	SPI_configPop();								\
	PORTD |= _BV(XCS); }

#define VS1053_ASSERT_DCS {							\
	SPI_configPush(SPI_VS1053);						\
	PORTD &= ~_BV(XDCS); }

#define VS1053_DEASSERT_DCS {						\
	SPI_configPop();								\
	PORTD |= _BV(XDCS); }

#define spiTransfer(b)			SPI_transfer(b)

void VS1053_initSPI(void)
{
	SPI_enable();
	SPI_setBitOrder(SPI_MSB_ORDER);
	SPI_setClockDiv(SPI_CLOCK8);
	SPI_setDataMode(SPI_MODE0);
	SPI_configStore(SPI_VS1053);
}

void VS1053_termSPI(void)
{
	VS1053_interruptDisable();
	VS1053_RST_LOW;
}

// DRIVER INTERRUPT SETTINGS ----------------------------------------------

void VS1053_interruptEnable(void)
{
	EIMSK |= _BV(INT1);		// Interrupt on vector INT1
	EICRA |= _BV(ISC10);	// Interrupt on any logical change
	sei();					// Set global interrupts
}

void VS1053_interruptDisable(void)
{
	EIMSK &= ~_BV(INT1);				// Disable interrupt on vector INT1
}

ISR(INT1_vect)
{
	DreqCB();
}

// PRIVATE CONSTANTS ----------------------------------------------------------

#define VS1053_OP_SCI_WRITE 0x02
#define VS1053_OP_SCI_READ	0x03

// PUBLIC FUNCTIONS -----------------------------------------------------------

//sciRead: Performs VS1053 Serial Command Interface acquisition
uint16_t sciRead(uint8_t addr)
{
	uint16_t data;

	waitForDREQ();						// Wait until VS1053 can receive

	VS1053_ASSERT_CS;					// Pull SCI slave select LOW (active)

	spiTransfer(VS1053_OP_SCI_READ);	// Transmit the READ opcode
	spiTransfer(addr);					// Transmit the 8-bit address
	data = spiTransfer(0x00);			// Receive the MSB of the 16-bit data
	data <<= 8;
	data |= spiTransfer(0x00);			// Receive the LSB of the 16-bit data

	VS1053_DEASSERT_CS;					// Pull SCI select HIGH (inactive)

	return data;
}

// sciWrite: Performs VS1053 Serial Command Interface transmission
void sciWrite(uint8_t addr, uint16_t data)
{
	waitForDREQ();						// Wait until VS1053 can receive

	VS1053_ASSERT_CS;					// Pull SCI slave select LOW (active)

	spiTransfer(VS1053_OP_SCI_WRITE);	// Transmit the WRTIE opcode
	spiTransfer(addr);					// Transmit the 8-bit address
	spiTransfer(data >> 8);				// Transmit the MSB of the 16-bit data
	spiTransfer(data >> 0);				// Transmit the LSB of the 16-bit data

	VS1053_DEASSERT_CS;					// Pull SCI select HIGH (inactive)
}

// sdiWrite: Performs VS1053 Serial Data Interface transmission
void sdiWrite(uint8_t *sdi_buff, uint16_t sdi_buffsize)
{
	VS1053_ASSERT_DCS;							// Pull SDI SS LOW (active)

	for (uint16_t i = 0; i < sdi_buffsize; i++)
	{
		spiTransfer(sdi_buff[i]);				// Send buffer data
	}

	VS1053_DEASSERT_DCS;						// Pull SDI SS HIGH (inactive)
}
