/*
 * ----------------------------------------------------------------------------
 * FILE: player.h
 * AUTHOR: Justin Garcia
 * ----------------------------------------------------------------------------
 * Implements a basic mp3 player using the VS1053b mp3 decoder module.
 * Header file
 * ----------------------------------------------------------------------------
 */

#ifndef VS1053_PLAYER_H_
#define VS1053_PLAYER_H_

// INCLUDE FILES --------------------------------------------------------------

#include <stdio.h>

// PUBLIC FUNCTION PROTOTYPES -------------------------------------------------

// init: Initialize VS1053 module
uint8_t VS1053_init(void);

// term: Terminate VS1053 module
void VS1053_term(void);

// play: Play audio file. Non blocking (requires interrupts).
uint8_t VS1053_play(char *track);

// playFull: Play audio file completely. Blocking.
uint8_t VS1053_playFull(char *track);

// pause: Pause audio playback
void VS1053_pause(void);

// stop: Stop audio playback
void VS1053_stop(void);

// resetHard: Hardware reset
void VS1053_resetHard(void);

// resetSoft: Software reset
void VS1053_resetSoft(void);

// setVolume: Set volume. [SCALE INFORMATION]
void VS1053_setVolume(uint8_t left, uint8_t right);

#endif
