/*=============================================================================
File: Pause.h

Description: Provides a Pause() function to aid in debugging.  You weren't 
  thinking about actually leaving this in your code were you?

Notes:
  - assumes a 20MHz external oscillator
  - entirely blocking code
=============================================================================*/
#ifndef PAUSE_H
#define PAUSE_H

#include "./ConfigurationBits.h"

#define OPS_PER_MS  3200  // operations per millisecond

static inline void Pause(unsigned int milliseconds) {
  unsigned int i,j;
	for(i = 0; i < OPS_PER_MS; ++i) {
    for(j = 0; j < milliseconds; ++j);
  }
}

#endif
