/*==============================================================================
File: PWM.c

Notes:
	- see p.163 of datasheet
	- T_PWM = (PRx + 1) * T_CY * timerx_prescaler
	        = (PRx + 1) * (32MHz/2)^-1 * (64)
	- duty_cycle = (OCxRS - OCxR) / PRx
	- OCxR determines number of high ticks (high time)
	- OCxRS determines period (number of total ticks)
==============================================================================*/
//#define TEST_PWM
/*---------------------------Dependencies-------------------------------------*/
#include "./PWM.h"
#include "./PPS.h"
#include "./StandardHeader.h"

/*---------------------------Macros-------------------------------------------*/
#define TICKS_PER_MS          2000  // for divide-by-8 prescaler

/*---------------------------Helper Function Prototypes-----------------------*/
static void ConfigureOC1(uint16_t ms);
static void ConfigureOC2(uint16_t ms);
static void InitTimer2(void);

/*---------------------------Module Variables---------------------------------*/
static uint8_t OC1_RPn, OC2_RPn = 0; 			  // remappable pin number
static uint16_t OC1_period, OC2_period = 0;	// units of Timer2Ticks

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_PWM
#include "./ConfigurationBits.h"

int main(void) {
	unsigned char dutyCycle = 0;

	PWM_Init(1, 2, 10);
	
	while (1) {
		Delay(1000);
		if (100 < ++dutyCycle) dutyCycle = 0;
		PWM_UpdateDutyCycle(dutyCycle);
	}

	PWM_Deinit();
	return 0;
}

#endif
/*---------------------------Public Function Definitions----------------------*/
void PWM_Init(uint8_t pin1, uint8_t pin2, uint16_t ms) {
  OC1_RPn = pin1;
  OC2_RPn = pin2;
  PPS_MapPeripheral(OC1_RPn, OUTPUT, FN_OC1);
	PPS_MapPeripheral(OC2_RPn, OUTPUT, FN_OC2);
	InitTimer2();
	ConfigureOC1(ms);
	ConfigureOC2(ms);
}


void inline PWM_UpdateDutyCycle(uint8_t pin, float dutyCycle) {
	if (pin == OC1_RPn) OC1R = (uint16_t)(dutyCycle * (float)OC1_period);
	else if (pin == OC2_RPn) OC2R = (uint16_t)(dutyCycle * (float)OC2_period);
}


void PWM_Deinit(void) {
	// turn off the output compare modules and restore OCx settings to default 
	OC1CON1 = 0x00; OC1CON2 = 0x0C; OC1RS = 0; OC1R = 0;
	OC2CON1 = 0x00; OC2CON2 = 0x0C; OC2RS = 0; OC2R = 0;
	
	// turn off and restore Timer2 settings to default
	T2CON = 0x00; PR2 = 0xffff;
	
	// restore any consumed pins
	PPS_MapPeripheral(OC1_RPn, OUTPUT, FN_NULL);
	PPS_MapPeripheral(OC2_RPn, OUTPUT, FN_NULL);
	
	// clear any module variables
	OC1_RPn = 0; OC1_period = 0;
	OC2_RPn = 0; OC2_period = 0;
}

/*---------------------------Helper Function Definitions----------------------*/
static void ConfigureOC1(uint16_t ms) {
	OC1_period = ms * TICKS_PER_MS;
	OC1RS = OC1_period;
	PWM_UpdateDutyCycle(OC1_RPn, 0);
	OC1CON2bits.SYNCSEL = 0x1f;		// this OC module
	OC1CON2bits.OCTRIG = 0;				// synchronize OCx with source from SYNCSEL bits
	
	OC1CON1bits.OCM = 0b110;			// select Edge-aligned PWM Mode on OC2
	OC1CON1bits.OCTSEL  = 0b000;	// select Timer2 as the time base source
}


static void ConfigureOC2(uint16_t ms) {
	OC2_period = ms * TICKS_PER_MS;
	OC2RS = OC2_period;
	PWM_UpdateDutyCycle(OC2_RPn, 0);
	OC2CON2bits.SYNCSEL = 0x1f;		// this OC module
	OC2CON2bits.OCTRIG = 0;				// synchronize OCx with source from SYNCSEL bits
	
	OC2CON1bits.OCM = 0b110;			// select Edge-aligned PWM Mode on OC2
	OC2CON1bits.OCTSEL  = 0b000;	// select Timer2 as the time base source
}


static void InitTimer2(void) {
	// using as 16-bit timer
	T2CONbits.TCKPS = 0b01;				// configure prescaler to divide-by-8 (p.166)
	PR2 = 0xffff;         				// configure the timer period
	T2CONbits.TON = 1;						// turn on the timer
}
