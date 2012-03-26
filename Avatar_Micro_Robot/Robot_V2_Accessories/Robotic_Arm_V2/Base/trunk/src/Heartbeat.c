/*=============================================================================

 Heartbeat.c

 23 Mar 2011
 Stellios Leventis

=============================================================================*/
#define TestHeartbeat

/*---------------------------Dependencies------------------------------------*/
#include "Heartbeat.h"
#include <p24FJ256GB106.h>

/*---------------------------Macros and Definitions--------------------------*/
#define OPS_PER_MS  3200  // operations per millisecond

/*---------------------------Helper Function Prototypes----------------------*/
static void Pause(unsigned int milliseconds);
static void ConfigurePins(void);

/*---------------------------Module Variables--------------------------------*/

/*---------------------------Test Harness------------------------------------*/
#ifdef TestHeartbeat
// configuration bits
_CONFIG1(JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2 & 
         WDTPS_PS256) 
_CONFIG2(IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS & FNOSC_PRIPLL & 
         PLLDIV_DIV5 & IOL1WAY_ON)

int main() {
  ConfigurePins();

	while (1) {
    Pause(1000);
	  PORTEbits.RE5 ^= 1; // toggle a pin to indicate normal operation
	}

  return 0;
}

static void ConfigurePins(void) {
	// configure and initialize I/O pin(s)
  //---ensure none of these pins are configured as analog (N/A for this pin)
  //---configure debugging pin(s) as digital output(s)
  TRISEbits.TRISE5 = 0;
	// initialize all I/O pins to begin in a known state
	PORTEbits.RE5 = 0;
}

static void Pause(unsigned int milliseconds) {
  unsigned int i,j;

	for(i = 0; i < OPS_PER_MS; ++i) {
    for(j = 0; j < milliseconds; ++j);
  }
}

#endif
/*---------------------------End Test Harness--------------------------------*/
