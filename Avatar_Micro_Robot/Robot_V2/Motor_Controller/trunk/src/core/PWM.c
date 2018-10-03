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

/*---------------------------Macros-------------------------------------------*/
#define TICKS_PER_US          16    // for divide-by-1 prescaler, PR2 = 1023
#define N_PWM_MODULES         9     // the maximum number of I2C modules
                                    // available on this microcontroller

/*---------------------------Helper Function Prototypes-----------------------*/
static void ConfigureOC1(uint16_t us);
static void ConfigureOC2(uint16_t us);
static void ConfigureOC3(uint16_t us);
static void ConfigureOC4(uint16_t us);
static void ConfigureOC5(uint16_t us);
static void ConfigureOC6(uint16_t us);
static void ConfigureOC7(uint16_t us);
static void ConfigureOC8(uint16_t us);
static void ConfigureOC9(uint16_t us);
static void InitTimer2(void);

/*---------------------------Module Variables---------------------------------*/
static uint8_t RPns[N_PWM_MODULES] = {0}; 		// remappable pin numbers
static uint16_t periods[N_PWM_MODULES] = {0};	// units of Timer2Ticks

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_PWM
#include "./ConfigurationBits.h"

int main(void) {
	unsigned char dutyCycle = 0;

	PWM_Init(kPWM01, 17, 33);
	PWM_UpdateDutyCycle(kPWM01, 0.5);
	while (1) {
	//	Delay(1000);
 	//	static float duty_cycle = 0;
	//	duty_cycle += 0.1;
 	//	if (1.0 < duty_cycle) dutyCycle = 0;
	//	PWM_UpdateDutyCycle(kPWM01, duty_cycle);
	}

	PWM_Deinit(kPWM01);
	return 0;
}

#endif
/*---------------------------Public Function Definitions----------------------*/
void PWM_Init(const kPWMModule module, 
              const uint8_t RPn, 
              const uint16_t period) {
  RPns[module] = RPn;               // store the pin number to un-map later
    
  if (!T2CONbits.TON) InitTimer2(); // assumes that timer2 is OFF on restart
  
  switch (module) {
    case kPWM01: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC1); 
      ConfigureOC1(period);
      break;
    case kPWM02: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC2);
      ConfigureOC2(period);
      break;
    case kPWM03: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC3);
      ConfigureOC3(period);
      break;
    case kPWM04: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC4);
      ConfigureOC4(period);
      break;
    case kPWM05: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC5);
      ConfigureOC5(period);
      break;
    case kPWM06: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC6);
      ConfigureOC6(period);
      break;
    case kPWM07: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC7);
      ConfigureOC7(period);
      break;
    case kPWM08: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC8);
      ConfigureOC8(period);
      break;
    case kPWM09: 
      PPS_MapPeripheral(RPn, OUTPUT, FN_OC9);
      ConfigureOC9(period);
      break;
  }
}


void inline PWM_UpdateDutyCycle(const kPWMModule module,
                                const float duty_cycle) {
  switch (module) {
    case kPWM01: OC1R = (uint16_t)(duty_cycle * (float)periods[module]); break;
    case kPWM02: OC2R = (uint16_t)(duty_cycle * (float)periods[module]); break;
    case kPWM03: OC3R = (uint16_t)(duty_cycle * (float)periods[module]); break;
    case kPWM04: OC4R = (uint16_t)(duty_cycle * (float)periods[module]); break;
    case kPWM05: OC5R = (uint16_t)(duty_cycle * (float)periods[module]); break;
    case kPWM06: OC6R = (uint16_t)(duty_cycle * (float)periods[module]); break;
    case kPWM07: OC7R = (uint16_t)(duty_cycle * (float)periods[module]); break;
    case kPWM08: OC8R = (uint16_t)(duty_cycle * (float)periods[module]); break;
    case kPWM09: OC9R = (uint16_t)(duty_cycle * (float)periods[module]); break;
  }
}


void PWM_Deinit(const kPWMModule module) {
	// turn off the output compare modules and restore OCx settings to default
	switch (module) {
    case kPWM01: OC1CON1 = 0x00; OC1CON2 = 0x0C; OC1RS = 0; OC1R = 0; break;
    case kPWM02: OC2CON1 = 0x00; OC2CON2 = 0x0C; OC2RS = 0; OC2R = 0; break;
    case kPWM03: OC3CON1 = 0x00; OC3CON2 = 0x0C; OC3RS = 0; OC3R = 0; break;
    case kPWM04: OC4CON1 = 0x00; OC4CON2 = 0x0C; OC4RS = 0; OC4R = 0; break;
    case kPWM05: OC5CON1 = 0x00; OC5CON2 = 0x0C; OC5RS = 0; OC5R = 0; break;
    case kPWM06: OC6CON1 = 0x00; OC6CON2 = 0x0C; OC6RS = 0; OC6R = 0; break;
    case kPWM07: OC7CON1 = 0x00; OC7CON2 = 0x0C; OC7RS = 0; OC7R = 0; break;
    case kPWM08: OC8CON1 = 0x00; OC8CON2 = 0x0C; OC8RS = 0; OC8R = 0; break;
    case kPWM09: OC9CON1 = 0x00; OC9CON2 = 0x0C; OC9RS = 0; OC9R = 0; break;
  }
	
	// restore the consumed pin
	PPS_MapPeripheral(RPns[module], OUTPUT, FN_NULL);
	
	// clear any module variables
	RPns[module] = 0;
	periods[module] = 0;
	
	// if this is the last running PWM module
	// turn off and restore Timer2 settings to default
	uint8_t i;
	uint16_t sum = 0;
	for (i = 0; i < N_PWM_MODULES; i++) sum += RPns[i]; 
	
	if (sum == 0) {
  	T2CON = 0x00;
  	PR2 = 0xffff;
	}
}

/*---------------------------Helper Function Definitions----------------------*/
static void ConfigureOC1(const uint16_t us) {
	periods[kPWM01] = us * TICKS_PER_US;
	OC1RS = periods[kPWM01];
	PWM_UpdateDutyCycle(kPWM01, 0);
	OC1CON2bits.SYNCSEL = 0x1f;		// this OC module
	OC1CON2bits.OCTRIG = 0;				// synchronize OCx with source from SYNCSEL bits
	
	OC1CON1bits.OCM = 0b110;			// select edge-aligned PWM Mode on OCx
	OC1CON1bits.OCTSEL  = 0b000;	// select Timer2 as the time base source
}


static void ConfigureOC2(uint16_t us) {
	periods[kPWM02] = us * TICKS_PER_US;
	OC2RS = periods[kPWM02];
	PWM_UpdateDutyCycle(kPWM02, 0);
	OC2CON2bits.SYNCSEL = 0x1f;
	OC2CON2bits.OCTRIG = 0;
	OC2CON1bits.OCM = 0b110;
	OC2CON1bits.OCTSEL  = 0b000;
}


static void ConfigureOC3(uint16_t us) {
	periods[kPWM03] = us * TICKS_PER_US;
	OC3RS = periods[kPWM03];
	PWM_UpdateDutyCycle(kPWM03, 0);
	OC3CON2bits.SYNCSEL = 0x1f;
	OC3CON2bits.OCTRIG = 0;
	OC3CON1bits.OCM = 0b110;
	OC3CON1bits.OCTSEL  = 0b000;
}


static void ConfigureOC4(uint16_t us) {
	periods[kPWM04] = us * TICKS_PER_US;
	OC4RS = periods[kPWM04];
	PWM_UpdateDutyCycle(kPWM04, 0);
	OC4CON2bits.SYNCSEL = 0x1f;
	OC4CON2bits.OCTRIG = 0;
	OC4CON1bits.OCM = 0b110;
	OC4CON1bits.OCTSEL  = 0b000;
}

static void ConfigureOC5(uint16_t us) {
	periods[kPWM05] = us * TICKS_PER_US;
	OC5RS = periods[kPWM05];
	PWM_UpdateDutyCycle(kPWM05, 0);
	OC5CON2bits.SYNCSEL = 0x1f;
	OC5CON2bits.OCTRIG = 0;
	OC5CON1bits.OCM = 0b110;
	OC5CON1bits.OCTSEL  = 0b000;
}


static void ConfigureOC6(uint16_t us) {
	periods[kPWM06] = us * TICKS_PER_US;
	OC6RS = periods[kPWM06];
	PWM_UpdateDutyCycle(kPWM06, 0);
	OC6CON2bits.SYNCSEL = 0x1f;
	OC6CON2bits.OCTRIG = 0;
	OC6CON1bits.OCM = 0b110;
	OC6CON1bits.OCTSEL  = 0b000;
}


static void ConfigureOC7(uint16_t us) {
	periods[kPWM07] = us * TICKS_PER_US;
	OC7RS = periods[kPWM07];
	PWM_UpdateDutyCycle(kPWM07, 0);
	OC7CON2bits.SYNCSEL = 0x1f;
	OC7CON2bits.OCTRIG = 0;
	OC7CON1bits.OCM = 0b110;
	OC7CON1bits.OCTSEL  = 0b000;
}


static void ConfigureOC8(uint16_t us) {
	periods[kPWM08] = us * TICKS_PER_US;
	OC8RS = periods[kPWM08];
	PWM_UpdateDutyCycle(kPWM08, 0);
	OC8CON2bits.SYNCSEL = 0x1f;
	OC8CON2bits.OCTRIG = 0;
	OC8CON1bits.OCM = 0b110;
	OC8CON1bits.OCTSEL  = 0b000;
}


static void ConfigureOC9(uint16_t us) {
	periods[kPWM09] = us * TICKS_PER_US;
	OC9RS = periods[kPWM09];
	PWM_UpdateDutyCycle(kPWM09, 0);
	OC9CON2bits.SYNCSEL = 0x1f;
	OC9CON2bits.OCTRIG = 0;
	OC9CON1bits.OCM = 0b110;
	OC9CON1bits.OCTSEL  = 0b000;
}


static void InitTimer2(void) {
	// using as 16-bit timer
	// NB: all PWM modules are based on this timer
	T2CONbits.TCKPS = 0b00;		// configure prescaler to divide-by-1 (p.166)
	PR2 = 1023;           	  // configure the timer period
	T2CONbits.TON = 1;				// turn on the timer
}
