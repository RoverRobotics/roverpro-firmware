/*=============================================================================
File: PWM.c

Notes:
	- see p.163 of datasheet
	- T_PWM = (PRx + 1) * T_CY * timerx_prescaler
	        = (PRx + 1) * (32MHz/2)^-1 * (64)
	- duty_cycle = (OCxRS - OCxR) / PRx
	- OCxR determines number of high ticks (high time)
	- OCxRS determines period (number of total ticks)
=============================================================================*/
//#define TEST_PWM
/*---------------------------Dependencies------------------------------------*/
#include "./PWM.h"
#include "./PPS.h"
#include <p24FJ256GB106.h>


/*---------------------------Macros and Type Definitions---------------------*/
#define TICKS_PER_MS				63	// Timer2 ticks per millisecond, 62.5 actually

/*---------------------------Helper Function Prototypes----------------------*/
static void ConfigureOC2(unsigned int ms);
static void InitTimer2(void);

/*---------------------------Module Variables--------------------------------*/
static unsigned int period = 0;	// units of Timer2Ticks

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_PWM
#include "./TestCode/Pause.h"
#include "./ConfigurationBits.h"

int main(void) {
	unsigned char duty_cycle = 0;

	InitPWM(2, 10);

	while (1) {
		Pause(1000);
		if (++duty_cycle > 100) duty_cycle = 0;
		UpdateDutyCycle(duty_cycle);
	}

	return 0;
}

#endif
/*---------------------------End Test Harness--------------------------------*/
/*---------------------------Public Function Definitions---------------------*/
void InitPWM(unsigned char pin, unsigned int ms) {
  MapPeripheral(pin, OUTPUT, FN_OC2);
	ConfigureOC2(ms);
}

void UpdateDutyCycle(double duty_cycle) {
	OC2R = (unsigned int)(duty_cycle * (double)period);
}


/*---------------------------Helper Function Definitions---------------------*/
static void ConfigureOC2(unsigned int ms) {
	period = ms * TICKS_PER_MS;   // only because OC2RS appears to not be readable
	OC2RS = period;
	UpdateDutyCycle(0);
	OC2CON2bits.SYNCSEL = 0x1f;		// this OC module
	OC2CON2bits.OCTRIG = 0;				// synchronize OCx with source designated by SYNCSEL bits
	
	OC2CON1bits.OCM = 0b110;			// select Edge-aligned PWM Mode on OC2
	OC2CON1bits.OCTSEL  = 0b000;	// select Timer2 as the time base source
	InitTimer2();
}

static void InitTimer2(void) {
	// using as 16-bit timer
	T2CONbits.TCKPS = 0b11;				// configure the prescaler to divide-by-256 (see p.166)
	PR2 = 0xffff;         				// configure the timer period
	T2CONbits.TON = 1;						// turn on the timer
}


/*---------------------------End of File-------------------------------------*/
