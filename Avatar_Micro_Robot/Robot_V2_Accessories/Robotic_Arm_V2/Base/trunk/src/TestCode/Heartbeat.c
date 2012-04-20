/*=============================================================================
File: Heartbeat.c

Description: This module breathes life into a microcontroller through a
  'Hello World' equivalent of embedded circuits.  It toggles pin to confirm a 
  (at least a partially) functional chip, confirm configuration of the 
  integrated development environment and to confirm the programming header 
  and basic circuit.  Among other things, it serves as a quick, first step
  toward developing your application on a PIC24FJ256GA106.
 
Author: Stellios Leventis (sleventis@robotex.com)
=============================================================================*/
#define TestHeartbeat
/*---------------------------Dependencies------------------------------------*/
#include <p24FJ256GB106.h>

/*---------------------------Macros and Type Definitions---------------------*/
#define OPS_PER_MS  3200  // operations per millisecond

/*---------------------------Helper Function Prototypes----------------------*/
static void Pause(unsigned int milliseconds);
static void ConfigurePins(void);

/*---------------------------Module Variables--------------------------------*/

/*---------------------------Test Harness------------------------------------*/
#ifdef TestHeartbeat
#include "../ConfigurationBits.h"

int main() {
  ConfigurePins();

	while (1) {
    Pause(1000);
	  PORTEbits.RE5 ^= 1; // toggle a pin to indicate normal operation
	}

  return 0;
}

#endif
/*---------------------------End Test Harness--------------------------------*/
/*---------------------------Helper Function Definitions---------------------*/
static inline void ConfigurePins(void) {
	 TRISEbits.TRISE5 = 0;
	 PORTEbits.RE5 = 0;
}

static inline void Pause(unsigned int milliseconds) {
  unsigned int i,j;
	for(i = 0; i < OPS_PER_MS; ++i) {
    for(j = 0; j < milliseconds; ++j);
  }
}
