/*
 * ----------------------------------------------------------------------------
 * FILE: player.c
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Implements a basic mp3 player using VS1053b mp3 decoder module.
 * Source file.
 * ----------------------------------------------------------------------------
 */

// INCLUDE FILES --------------------------------------------------------------

#include "player.h"

#include "vsspi.h"
#include "fatfs/pff.h"

#include <avr/io.h>
#include <string.h>

// PRIVATE CONSTANTS ----------------------------------------------------------

#define VS1053_PMODE_START		0
#define VS1053_PMODE_FULL		1

#define VS1053_SCI_MODE			0x00
#define VS1053_SCI_BASS			0x02
#define VS1053_SCI_CLOCKF		0x03
#define VS1053_SCI_WRAM			0x06
#define VS1053_SCI_WRAMADDR		0x07
#define VS1053_SCI_HDAT0		0x08
#define VS1053_SCI_HDAT1		0x09
#define VS1053_SCI_VOL			0x0B

#define VS1053_SM_RESET			0x0004
#define VS1053_SM_CANCEL		0x0008
#define VS1053_SM_SDINEW		0x0800

#define VS1053_SDI_MAX			32
#define VS1053_BUFFER_SIZE		(VS1053_SDI_MAX * 4)
#define VS1053_CANCEL_MAX		64

#define VS1053_EFB_BUFF_SIZE	32
#define VS1053_EFB_SDI_BLOCK	64+1
#define VS1053_EFB_ADDR			0x1E06

#define VS1053_DEF_CLOCK		0x9800
#define VS1053_DEF_VOL			0x10

// FUNCTION MACROS ------------------------------------------------------------

#define sciMode(m)		sciWrite(VS1053_SCI_MODE, VS1053_SM_SDINEW | m)

// PRIVATE GLOBAL VARIABLES ---------------------------------------------------

static FATFS pf_fs;
static FRESULT pf_res;

static uint8_t buff[VS1053_BUFFER_SIZE];

static volatile uint8_t isPlaying;
static volatile uint8_t isPaused;
static volatile uint8_t isCancelled;

static volatile uint16_t sdi_index;
static volatile uint16_t bytesRead;
static volatile uint8_t cancelCount;

// PRIVATE FUNCTION PROTOTYPES ------------------------------------------------

static uint8_t startTrack(char *track, uint8_t mode);
static uint8_t endTrack(void);
void feedBuffer(void);

// PUBLIC FUNCTIONS -----------------------------------------------------------

uint8_t VS1053_init(void)
{
	DreqCB = feedBuffer;
	pf_res = pf_mount(&pf_fs);
	VS1053_initSPI();

	VS1053_resetHard();

	isPlaying = 0;
	isPaused = 0;
	isCancelled = 0;

	VS1053_interruptEnable();

	return pf_res;
}

uint8_t VS1053_play(char *track)
{
	return startTrack(track, VS1053_PMODE_START);
}

uint8_t VS1053_playFull(char *track)
{
	return startTrack(track, VS1053_PMODE_FULL);
}

void VS1053_pause(void)
{
	if (!isPlaying) return;
	if (isCancelled) return;

	isPaused ^= 1;
	feedBuffer();
}

void VS1053_stop(void)
{
	if (!isPlaying) return;
	if (isCancelled) return;

	sciMode(VS1053_SM_CANCEL);
	isCancelled = 1;
}

void VS1053_resetHard(void)
{
	VS1053_RST_LOW;
	VS1053_delay_ms(100);
	VS1053_CS_HIGH;
	VS1053_DCS_HIGH;
	VS1053_RST_HIGH;

	sciMode(0);
	sciWrite(VS1053_SCI_BASS, 0x0000);
	sciWrite(VS1053_SCI_CLOCKF, VS1053_DEF_CLOCK);

	VS1053_setVolume(VS1053_DEF_VOL, VS1053_DEF_VOL);
}

void VS1053_resetSoft(void)
{
	sciMode(VS1053_SM_RESET);
	waitForDREQ();
}

void VS1053_setVolume(uint8_t left, uint8_t right)
{
	uint16_t vol = (left << 8) | (right << 0);
	sciWrite(VS1053_SCI_VOL, vol);
}

void VS1053_term(void)
{
	if (isPlaying) VS1053_stop();
	VS1053_termSPI();
}

// PRIVATE FUNCTIONS ----------------------------------------------------------

static uint8_t startTrack(char *track, uint8_t mode)
{
	// Check for currently playing
	if (isPlaying) VS1053_stop();

	// Load track from SD card
	bytesRead = 0;
	sdi_index = 0;
	pf_res = pf_open(track);
	if (pf_res) return 1;

	// Set state variables
	isPlaying = 1;
	isPaused = 0;
	isCancelled = 0;
	cancelCount = 0;
	
	// Start reading data
	if (mode == VS1053_PMODE_FULL)	while (isPlaying) feedBuffer();
	else							feedBuffer();

	return 0;
}

static uint8_t endTrack(void)
{
	// Create a data block of end fill byte
	sciWrite(VS1053_SCI_WRAMADDR, VS1053_EFB_ADDR);
	uint8_t efb = sciRead(VS1053_SCI_WRAM);
	uint8_t efb_block[VS1053_EFB_BUFF_SIZE];
	memset(efb_block, efb, sizeof(efb_block));

	// SDI write end fill byte block until play end is acknowledged
	uint8_t j = 0;
	while ( sciRead(VS1053_SCI_HDAT0) || sciRead(VS1053_SCI_HDAT1) )
	{
		sdiWrite(efb_block, VS1053_EFB_BUFF_SIZE);

		if (j < VS1053_EFB_SDI_BLOCK)
		{
			VS1053_resetSoft();
			return 1;
		}
		j++;
	}

	isPlaying = 0;
	return 0;
}

// INTERRUPT CALLBACK ---------------------------------------------------------

void feedBuffer(void)
{
	// CHECK STATE --------------------------------------------------------

	if (!isPlaying)	return;
	if (isPaused)	return;
	if (!readyDREQ)	return;

	// CANCEL REQUEST PROCEDURE -------------------------------------------
	
	if (isCancelled)
	{
		if ( !(sciRead(VS1053_SCI_MODE) & _BV(VS1053_SM_CANCEL)) )
		{
			endTrack();
			return;
		}
		else cancelCount++;

		if (cancelCount >= VS1053_CANCEL_MAX)
		{
			VS1053_resetSoft();
			return;
		}
	}

	// FILL BUFFER & WRITE DATA TO CODEC ----------------------------------

	while (readyDREQ)
	{
		// Fill buffer with data from SD card
		if (sdi_index >= bytesRead)
		{
			sdi_index = 0;
			bytesRead = 0;
			pf_res = pf_read(buff, VS1053_BUFFER_SIZE, (WORD *)&bytesRead);
			if (pf_res || bytesRead != VS1053_BUFFER_SIZE)
			{
				endTrack();
				return;
			}
		}

		// Write buffer data to SDI
		while (readyDREQ && (sdi_index < bytesRead))
		{
			sdiWrite(&buff[sdi_index], VS1053_SDI_MAX);
			sdi_index += VS1053_SDI_MAX;
		}
	}
}
