/*=============================================================================
File: PWM.c

Notes:
	- see p.163 of datasheet
	- T_PWM = (PRx + 1) * T_CY * timerx_prescaler
	        = (PRx + 1) * (32MHz/2)^-1 * (64)
	- duty_cycle = (OCxRS - OCxR) / PRx
	- OCxR determines number of high ticks (high time)
	- OCxRS determines period (number of total tocks)
=============================================================================*/
#define TEST_PWM
/*---------------------------Dependencies------------------------------------*/
#include "./PWM.h"
#include "./PPS.h"
#include <p24FJ256GB106.h>

/*---------------------------Macros and Type Definitions---------------------*/
#define TICKS_PER_US				2		// Timer2 ticks per microsecond

/*---------------------------Helper Function Prototypes----------------------*/
static void ConfigureOC2(unsigned int us);
static void InitTimer2(void);

/*---------------------------Module Variables--------------------------------*/
static unsigned int period = 0;	// units of Timer2Ticks

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_PWM
#include "./ConfigurationBits.h"
#include "./Pause.h"

int main(void) {
	unsigned char duty_cycle = 0;

	InitPWM(21, 10000);

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
void InitPWM(unsigned char pin, unsigned int us) {
  MapPeripheral(pin, OUTPUT, FN_OC2);
	ConfigureOC2(us);
}

void UpdateDutyCycle(unsigned char duty_cycle) {
	OC2R = (unsigned int) ((float)duty_cycle / 100 * (float)period);
}


/*---------------------------Helper Function Definitions---------------------*/
static void ConfigureOC2(unsigned int us) {
	period = us * TICKS_PER_US;
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
	T2CONbits.TCKPS = 0b01;				// configure the prescaler to divide-by-8 (see p.152)
	PR2 = 0xffff;         				// configure the timer period
	T2CONbits.TON = 1;						// turn on the timer
}


/*---------------------------End of File-------------------------------------*/
