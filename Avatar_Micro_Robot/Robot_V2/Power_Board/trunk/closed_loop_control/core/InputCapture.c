/*==============================================================================
File: InputCapture.c 
==============================================================================*/
//#define TEST_INPUT_CAPTURE
/*---------------------------Dependencies-------------------------------------*/
#include "./InputCapture.h"
#include "./PPS.h"
#include <limits.h>           // for UINT_MAX macro
#include <stdbool.h>      // for 'bool' boolean data type
#include "stdhdr.h"
#include "p24FJ256GB106.h"

#define NO                    0
#define YES                   (!NO)

#define FN_IC1						4
#define FN_IC2						5
#define FN_IC3						6


/*---------------------------Macros-------------------------------------------*/
#define MAX_NUM_IC_PINS           9
#define US_PER_TICK               16.0      // microseconds per timer tick

// time_per_tick = ((f_osc/2)/prescaler)^-1 => ((32MHz/2)/256)^-1 => 16us
//#define TICKS_PER_MS              63        // 62.5 actually

#define T4_TICKS_PER_MS           4        // milliseconds per timer4 tick 
                                           // TODO: IS THIS RIGHT????

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitTimer4(void);
static void InitIC1(const uint8_t RPn);
static void InitIC2(const uint8_t RPn);
static void InitIC3(const uint8_t RPn);
static void InitIC4(const uint8_t RPn);
static void InitIC5(const uint8_t RPn);
static void InitIC6(const uint8_t RPn);
static void InitIC7(const uint8_t RPn);
static void InitIC8(const uint8_t RPn);
static void InitIC9(const uint8_t RPn);

void IC1_ISR(void);
void IC2_ISR(void);

/*---------------------------Module Variables---------------------------------*/
static volatile bool is_timer3_running = NO;
static volatile uint8_t RPns[MAX_NUM_IC_PINS] = {0};
static volatile uint32_t timeouts[MAX_NUM_IC_PINS] = {0}; // in units of [ms]
static volatile uint32_t elapsed_times[MAX_NUM_IC_PINS] = {0};
static volatile float periods[MAX_NUM_IC_PINS] = {0};
static volatile uint32_t time = 0;  // running number of timer3 ticks

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_INPUT_CAPTURE
#include "./ConfigurationBits.h"

int main(void) {
  uint8_t RPn = 27;
  uint8_t timeout = 10; // [ms]
  IC_Init(kIC01, RPn, timeout);
  
  float my_square_wave_period;
	while (1) {
	  Delay(1000);
	  IC_UpdatePeriods();
	  my_square_wave_period = IC_period(module);
	}

	return 0;
}
#endif

/*---------------------------Interrupt Service Routines (ISRs)----------------*/
void T4_ISR(void) {
  _T4IF = 0;  // clear the source of the interrupt
  time++;
}


void IC1_ISR(void) {
  _IC1IF = 0;                      // clear the source of the interrupt
  elapsed_times[0] = 0;
  
  static uint16_t last_value = 0;
  uint16_t current_value = IC1BUF; // current running Timer3 tick value
                                   // (you must subtract off last value)
  
	// handle rollover, remove old offset
  if (last_value < current_value) periods[0] = current_value - last_value;
  else periods[0] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}


void IC2_ISR(void) {
  _IC2IF = 0;
  elapsed_times[1] = 0;
  
  static uint16_t last_value = 0;
  uint16_t current_value = IC2BUF;
  
  if (last_value < current_value) periods[1] = (current_value - last_value);
  else periods[1] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}


/*void __attribute__((__interrupt__, auto_psv)) _IC3Interrupt(void) {
  _IC3IF = 0;
  elapsed_times[2] = 0;

  static uint16_t last_value = 0;
  uint16_t current_value = IC3BUF;
  
  if (last_value < current_value) periods[2] = current_value - last_value;
  else periods[2] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}


void __attribute__((__interrupt__, auto_psv)) _IC4Interrupt(void) {
  _IC4IF = 0;
  elapsed_times[3] = 0;
  
  static uint16_t last_value = 0;
  uint16_t current_value = IC4BUF;
  
  if (last_value < current_value) periods[3] = current_value - last_value;
  else periods[3] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}


void __attribute__((__interrupt__, auto_psv)) _IC5Interrupt(void) {
  _IC5IF = 0;
  elapsed_times[4] = 0;
  
  static uint16_t last_value = 0;
  uint16_t current_value = IC5BUF;
  
  if (last_value < current_value) periods[4] = current_value - last_value;
  else periods[4] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}


void __attribute__((__interrupt__, auto_psv)) _IC6Interrupt(void) {
  _IC6IF = 0;
  elapsed_times[5] = 0;

  static uint16_t last_value = 0;
  uint16_t current_value = IC6BUF;
  
  if (last_value < current_value) periods[5] = current_value - last_value;
  else periods[5] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}


void __attribute__((__interrupt__, auto_psv)) _IC7Interrupt(void) {
  _IC7IF = 0;
  elapsed_times[6] = 0;
  
  static uint16_t last_value = 0;
  uint16_t current_value = IC7BUF;
  
  if (last_value < current_value) periods[6] = current_value - last_value;
  else periods[6] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}


void __attribute__((__interrupt__, auto_psv)) _IC8Interrupt(void) {
  _IC8IF = 0;
  elapsed_times[7] = 0;

  static uint16_t last_value = 0;
  uint16_t current_value = IC8BUF;
  
  if (last_value < current_value) periods[7] = current_value - last_value;
  else periods[7] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}


void __attribute__((__interrupt__, auto_psv)) _IC9Interrupt(void) {
  _IC9IF = 0;
  elapsed_times[8] = 0;

  static uint16_t last_value = 0;
  uint16_t current_value = IC9BUF;
  
  if (last_value < current_value) periods[8] = current_value - last_value;
  else periods[8] = (UINT_MAX - last_value) + current_value;
  last_value = current_value;
}*/

/*---------------------------Public Function Definitions----------------------*/
void IC_Init(const kICModule module,
             const uint8_t RPn,
             const uint8_t timeout) {
  timeouts[module] = timeout;
  RPns[module] = RPn;
  
  // turn on the timing base if it not on already
  /*if (!is_timer3_running) {
    T3CONbits.TCKPS = 0b11;   // configure prescaler to divide-by-256
    _T3IF = 0;
    //_T3IE = 1;
    T3CONbits.TON = 1;        // turn on the timer
    is_timer3_running = YES;
    
    InitTimer4();
  }*/

  // Switched timer3 to timer2 (timer 2 not used in regular power board firmware)
  //Timer 4 used in regular power board firmware, but only in IC interrupts.
  if (!is_timer3_running) {
    T5CONbits.TCKPS = 0b11;   // configure prescaler to divide-by-256
    _T5IF = 0;
    //_T3IE = 1;
    T5CONbits.TON = 1;        // turn on the timer
    is_timer3_running = YES;
    
    InitTimer4();
  }


  // initialize the given input capture hardware module
  switch (module) {
    case kIC01: InitIC1(RPn); break;
    case kIC02: InitIC2(RPn); break;
    case kIC03: InitIC3(RPn); break;
    case kIC04: InitIC4(RPn); break;
    case kIC05: InitIC5(RPn); break;
    case kIC06: InitIC6(RPn); break;
    case kIC07: InitIC7(RPn); break;
    case kIC08: InitIC8(RPn); break;
    case kIC09: InitIC9(RPn); break;
  }
}


float IC_period(const kICModule module) {
  return periods[module];
}


void IC_UpdatePeriods(void) {
  // reset any periods if it has been too long
  static uint32_t last_time = 0;
  uint32_t current_time = time;
  uint8_t i;
  int32_t delta_time;
  for (i = 0; i < MAX_NUM_IC_PINS; i++) {
    delta_time = (current_time - last_time); // OK ON ROLLOVER?
    elapsed_times[i] += delta_time;
    // NB: be consistent in units of timer4 ticks
    if ((timeouts[i] * T4_TICKS_PER_MS) < elapsed_times[i]) {
      periods[i] = 0;
      elapsed_times[i] = 0;
      if (i == 0) {
        Nop();
      }
    }
  }
  last_time = current_time;
}


void IC_Deinit(void) {
 /* // TODO: test this
  uint8_t i;
  for (i = 0; i < MAX_NUM_IC_PINS; i++) {
    // un-map mapped pins
    if (RPns[i]) PPS_MapPeripheral(RPns[i], OUTPUT, FN_NULL);
    
    // clear module-level arrays
    timeouts[i] = 0;
    RPns[i] = 0;
    elapsed_times[i] = 0;
    periods[i] = 0;
  }
  
  T3CONbits.TON = 0;  // turn OFF Timer3
  is_timer3_running = NO;*/
}


/*---------------------------Private Function Definitions---------------------*/
static void InitTimer4(void) {
  T4InterruptUserFunction=T4_ISR;
  T4CONbits.TON = 0;        // turn off the timer while we configure it
  T4CONbits.TCS = 0;        // use the internal, system clock
  PR4 = 0x0fd0;
  T4CONbits.TCKPS = 0b00;   // configure prescaler to divide-by-1
  _T4IF = 0;                // begin with the interrupt flag cleared
  _T4IE = 1;                // enable the interrupt
  T4CONbits.TON = 1;        // turn on the timer
}

  
static void InitIC1(const uint8_t RPn) {
  // clear the input capture FIFO buffer
  uint16_t temp;
	while (IC1CON1bits.ICBNE) {temp = IC1BUF;};
  
  // configure the input capture
  //IC1CON1bits.ICTSEL = 0;   // use Timer3 as the time base
  IC1CON1bits.ICTSEL = 0b011;   // use Timer5 as the time base
  IC1CON1bits.ICI = 0b00;   // fire the interrupt every capture event
  IC1CON1bits.ICM = 0b011;  // capture event on every rising edge
  
  //PPS_MapPeripheral(RPn, INPUT, FN_IC1);
  _IC1R = RPn;

  IC1InterruptUserFunction=IC1_ISR;

  _IC1IF = 0;               // begin with the interrupt flag cleared
  _IC1IE = 1;               // enable this interrupt
}


static void InitIC2(const uint8_t RPn) {
  uint16_t temp;
	while (IC2CON1bits.ICBNE) {temp = IC2BUF;};

//  IC2CON1bits.ICTSEL = 0;
  IC2CON1bits.ICTSEL = 0b011;   // use Timer5 as the time base
  IC2CON1bits.ICI = 0b00;
  IC2CON1bits.ICM = 0b011;

  IC2InterruptUserFunction=IC2_ISR;
  
  //PPS_MapPeripheral(RPn, INPUT, FN_IC2);
  _IC2R = RPn; 
  _IC2IF = 0;
  _IC2IE = 1;
}


static void InitIC3(const uint8_t RPn) {
/*  uint16_t temp;
	while (IC3CON1bits.ICBNE) {temp = IC3BUF;};

  IC3CON1bits.ICTSEL = 0;
  IC3CON1bits.ICI = 0b00;
  IC3CON1bits.ICM = 0b011;
  
//  PPS_MapPeripheral(RPn, INPUT, FN_IC3);
  _IC3R = RPn;
  
  _IC3IF = 0;
  _IC3IE = 1;*/
}


static void InitIC4(const uint8_t RPn) {
/*  uint16_t temp;
	while (IC4CON1bits.ICBNE) {temp = IC4BUF;};

  IC4CON1bits.ICTSEL = 0;
  IC4CON1bits.ICI = 0b00;
  IC4CON1bits.ICM = 0b011;
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC4);
  
  _IC4IF = 0;
  _IC4IE = 1;*/
}


static void InitIC5(const uint8_t RPn) {
/*  uint16_t temp;
	while (IC5CON1bits.ICBNE) {temp = IC5BUF;};

  IC5CON1bits.ICTSEL = 0;
  IC5CON1bits.ICI = 0b00;
  IC5CON1bits.ICM = 0b011;
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC5);
  
  _IC5IF = 0;
  _IC5IE = 1;*/
}


static void InitIC6(const uint8_t RPn) {
/*  uint16_t temp;
	while (IC6CON1bits.ICBNE) {temp = IC6BUF;};

  IC6CON1bits.ICTSEL = 0;
  IC6CON1bits.ICI = 0b00;
  IC6CON1bits.ICM = 0b011;
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC6);
  
  _IC6IF = 0;
  _IC6IE = 1;*/
}


static void InitIC7(const uint8_t RPn) {
/*  uint16_t temp;
	while (IC7CON1bits.ICBNE) {temp = IC7BUF;};

  IC7CON1bits.ICTSEL = 0;
  IC7CON1bits.ICI = 0b00;
  IC7CON1bits.ICM = 0b011;
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC7);
  
  _IC7IF = 0;
  _IC7IE = 1;*/
}


static void InitIC8(const uint8_t RPn) {
 /* uint16_t temp;
	while (IC8CON1bits.ICBNE) {temp = IC8BUF;};

  IC8CON1bits.ICTSEL = 0;
  IC8CON1bits.ICI = 0b00;
  IC8CON1bits.ICM = 0b011;
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC8);
  
  _IC8IF = 0;
  _IC8IE = 1;*/
}


static void InitIC9(const uint8_t RPn) {
 /* uint16_t temp;
	while (IC9CON1bits.ICBNE) {temp = IC9BUF;};

  IC9CON1bits.ICTSEL = 0;
  IC9CON1bits.ICI = 0b00;
  IC9CON1bits.ICM = 0b011;
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC9);
  
  _IC9IF = 0;
  _IC9IE = 1;*/
}
