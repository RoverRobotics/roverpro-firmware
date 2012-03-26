/*=============================================================================

 Base.c

 23 Mar 2011
 Stellios Leventis

=============================================================================*/
#define TEST_BASE

/*---------------------------Dependencies------------------------------------*/
#include "Base.h"
#include <p24FJ256GB106.h>
#include "ConfigurationBits.h"
#include "Timers.h"

/*---------------------------Macros and Definitions--------------------------*/
typedef enum {
	INITIALIZING_BASE = 0,
	COMMUNICATING,	
	// TODO; layout states for all state machines in separate file
} state_t;

typedef enum {
	EV_NO_EVENT = 0,
	EV_ENTRY,
	EV_EXIT
// TODO: layout events for all state machines in separate file
} event_t;


// Timers
#define _500ms            500
#define HEARTBEAT_TIMER   1
#define HEARTBEAT_TIME    _500ms

/*---------------------------Helper Function Prototypes----------------------*/
static void ConfigurePins(void);

/*---------------------------Module Variables--------------------------------*/

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_BASE

int main() {
	InitBase();

	while (1) {
		if (IsTimerExpired(HEARTBEAT_TIMER)) {
			StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
			PORTEbits.RE5 ^= 1; // toggle a pin to indicate normal operation
		}	
	}
}

#endif
/*---------------------------End Test Harness--------------------------------*/

/*---------------------------Public Function Definitions---------------------*/
void InitBase(void) {
	ConfigurePins();

	// initialize any dependent modules
	InitTimers();

	// prime any timers that require it
	StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
}

/*---------------------------Helper Function Definitions---------------------*/
void InitPWM(void) {
	//setup LED PWM channels

	T2CONbits.TCKPS = 0;

	//AP8803: dimming frequency must be below 500Hz
	//Choose 400Hz (2.5ms period)
	//2.5ms = [PR2 + 1]*62.5ns*1
	PR2 = 40000;	
	OC1RS = 39999;
	OC2RS = 39999;

	OC1CON2bits.SYNCSEL = 0x1f;	
	OC2CON2bits.SYNCSEL = 0x1f;	
	
	//use timer 2
	OC1CON1bits.OCTSEL2 = 0;
	OC2CON1bits.OCTSEL2 = 0;
	
	//edge-aligned pwm mode
	OC1CON1bits.OCM = 6;
	OC2CON1bits.OCM = 6;
	
	//turn on timer 2
	T2CONbits.TON = 1;
}



/****************************************************************************
Function: InitBase()
Description: Initializes any required I/O pins
*****************************************************************************/
static void ConfigurePins(void) {
	// configure and initialize I/O pin(s)
  //---ensure none of these pins are configured as analog (N/A for this pin)
  //---configure debugging pin(s) as digital output(s)
  TRISEbits.TRISE5 = 0;
	// initialize all I/O pins to begin in a known state
	PORTEbits.RE5 = 0;

	// PWM pin

	// Motor Controller
	// DIRO(1);
	// BREAKE(OFF);
	// MODE(SOME_MODE);
	// COAST(NONE);
	// DIR(CCW);
	// PWR_BUS_EN(1);

	/*
	AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;

	AD1PCFGL = 0xffff;
	TRISB = 0xffff;
	TRISC = 0xffff;
	TRISD = 0xffff;
	TRISE = 0xffff;
	TRISF = 0xffff;
	TRISG = 0xffff;

	VBAT_DIGI_ON(0);
	V3V3_ON(0);
	V5_ON(0);
	V12_ON(0);
	AMP_PWR_ON(0);
	CODEC_PWR_ON(0);
	COM_EXPRESS_PGOOD_ON(0);
	REAR_PL_PWR_ON(0);

	VBAT_DIGI_EN(1);
	V3V3_EN(1);
	V5_EN(1);
	V12_EN(1);
	AMP_PWR_EN(1);
	MIC_PWR_EN(1);
	CODEC_PWR_EN(1);
	COM_EXPRESS_PGOOD_EN(1);
	HUMIDITY_SENSOR_EN(1);
	REAR_PL_PWR_EN(1);
*/
}

/*---------------------------End of File-------------------------------------*/
