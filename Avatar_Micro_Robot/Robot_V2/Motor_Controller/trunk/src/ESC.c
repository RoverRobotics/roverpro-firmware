/*==============================================================================
File:ESC.c

Notes:
  - if noise becomes an issue, we can only switch the high-side transistor (and
    pulse its low-side counterpart) and leave the low-side transistor on
==============================================================================*/
#define TEST_ESC
//---------------------------Dependencies---------------------------------------
#include "./core/StandardHeader.h"
#include "./ESC.h"
//#include "./core/ADC.h"       // for A/D conversions
#include "./core/StandardHeader.h"  // for CCW, CW macros
#include "./core/PPS.h"             // for OC function name macros
#include <stdlib.h>             // for abs()

//---------------------------Macros---------------------------------------------
// analog sensing pins (ANx)
#define POT2_PIN            12  // TODO: remove after debugging board

// user interface pins
#define DIRI_EN(a)          (_TRISB2 = (a)); AD1PCFGL |= (1 << 2)
#define DIRI                (_RB2)          // pin 1 of P9
#define DIRO_EN(a)          (_TRISB4 = !(a)); AD1PCFGL |= (1 << 4)
#define DIRO                (_RB4)          // pin 2 of P9
#define TACHI_RPN_PIN       (_RP8)
#define TACHO_EN(a)         (_TRISB5 = !(a)); AD1PCFGL |= (1 << 5)
#define TACHO               (_RB5)          // pin 3 of P9

// digital inputs
#define HALL_A_EN(a)        (_TRISE5 = (a))
#define HALL_A              (_RE5)
#define HALL_B_EN(a)        (_TRISE6 = (a))
#define HALL_B              (_RE6)
#define HALL_C_EN(a)        (_TRISE7 = (a))
#define HALL_C              (_RE7)
#define HALL_STATE          ((HALL_C << 2) | (HALL_B << 1) | (HALL_A))

// debugging
//#define HEARTBEAT_EN(a)     (_TRISB9 = !(a))
//#define HEARTBEAT_PIN       (_RB9)  // pin 8 of connector P8

// TODO: remove after debugging
#define TEST1_RPN_PIN       (_RP21R)
#define TEST2_RPN_PIN       (_RP19R)

// input-capture inputs
#define A_HI_RPN_PIN        (_RP11R)
#define A_LO_RPN_PIN        (_RP24R)
#define B_HI_RPN_PIN        (_RP23R)//TEST1_RPN_PIN
#define B_LO_RPN_PIN        (_RP22R)//TEST2_RPN_PIN
#define C_HI_RPN_PIN        (_RP25R)
#define C_LO_RPN_PIN        (_RP20R)

// TODO: ensure these do NOT change
#define DEFAULT_DC              500//956   // [ticks]
#define PWM_PERIOD              1000  // [ticks], 1000 => ~16kHz

// The time before the end of the period to schedule the falling edge of the
// recharge pulse.
#define CROSSOVER_TOLERANCE     4     // [ticks], 28 leaves ~200ns before drive pulse begins

// Time consumed in the interrupt to reinitialize the single-shot pulse.
#define REPULSE_INTERRUPT_DELAY 24    // [ticks]

// The minimum recharge pulse duration in units of timer4 ticks (62.5ns/tick).
#define RECHARGE_DURATION       16    // [ticks], 16 => ~1us recharge pulse width

// The maximum duty cycle must be capped to ensure there is space during a period
// for the recharge pulse and the associated, manually-tuned delays.
#define MAX_DC  (PWM_PERIOD - CROSSOVER_TOLERANCE - REPULSE_INTERRUPT_DELAY - RECHARGE_DURATION) // should compute to 956
// NB: THE MAXIMUM DC FOR REVERSE IS LESS.  THERE IS A LARGE INCREASE IN CURRENT
// FOR DUTY CYCLES GREATER THAN ~860.  THE COMMUTATION BEGINS TO FIGHT ITSELF

// how long to delay after catching a stall condition
#define DELAY_MS    2000  // [ms]

#define MIN_N_EXPECTED_EVENTS_PER_MS  1000

//---------------------------Constants------------------------------------------
// [au], maximum allowable current draw from an individual side
// 0.01S*(17A*0.001Ohm)*11k ~= 1.87V, (1.87 / 3.30) * 1023=
//static const kADCReading kMaxCurrent = 520;

//---------------------------Helper Function Prototypes-------------------------
static void InitPins(void);
static void InitCNs(void);
static void Coast(void);
static bool inline IsMotorStalled(void);

// OC-module related
static void Energize(const uint8_t hall_state);
static void InitOCs(void);
static void UpdateDutyCycle(const uint16_t duty_cycle);
static void InitHighSidePulser(void);
static void InitLowSidePulser(void);
static void InitRechargePulser(void);

//---------------------------Module Variables-----------------------------------
static volatile uint16_t current_speed = 0;

//---------------------------Test Harness---------------------------------------
#ifdef TEST_ESC
#include "./core/ConfigurationBits.h"
int main(void) {
  ESC_Init();
  ESC_StartMotor(400); Delay(3000);
  ESC_set_speed(860);
  //Energize(3);
  //Energize(1);
  //Energize(1);
  //Energize(1);
  //Energize(1);
  //Energize(1);
  
  while (1) {
    /*
    if (IsMotorStalled()) {
      Coast();
      Delay(DELAY_MS);
      asm("reset");
    }
    */
  }
  
  return 0;
}
#endif

//---------------------------Interrupt Service Routines (ISRs)------------------
// This interrupt is thrown when the running timer4 counter counts up to OC1R
void __attribute__((__interrupt__, auto_psv)) _OC1Interrupt(void) {
  // restart the single-shot pulse
  OC3CON1bits.OCM = 0b000;
  OC3CON1bits.OCM = 0b100;
  _OC1IF = 0;
}

void __attribute__((__interrupt__, auto_psv)) _CNInterrupt(void) {
  Energize(HALL_STATE);   // energize the appropriate coils to move to the next position
  TACHO ^= 1;             // output that we commutated
  _CNIF = 0;              // clear the source of the interrupt
}

//---------------------------Public Function Definitions------------------------
void ESC_Init(void) {
  InitPins();
  InitCNs();
  InitOCs();
  //ADC_Init((1 << POT1_PIN) | (1 << POT2_PIN) | (1 << POT3_PIN));
  //HEARTBEAT_EN(1); HEARTBEAT_PIN = 0;
}

void ESC_StartMotor(const int16_t speed) {
  ESC_set_speed(speed);
  Energize(HALL_STATE);
}

void ESC_set_speed(const int16_t speed) {
  // update the magnitude
  UpdateDutyCycle(abs(speed));
}

int16_t ESC_speed(void) {
 return current_speed;
}

//---------------------------Helper Function Definitions------------------------
static void Energize(const uint8_t hall_state) {
  //T4CONbits.TON = 0;
  //TMR4 = PR4 - 2;
  
  // map the desired pins to the existing output compare modules
  if (0) {//DIRI == CCW) {
    switch (hall_state) {
      case 1:
        // disable any previous output
        LATD = 0;
        A_HI_RPN_PIN = FN_NULL; A_LO_RPN_PIN = FN_NULL; B_HI_RPN_PIN = FN_NULL;
        C_HI_RPN_PIN = FN_OC1; C_LO_RPN_PIN = FN_OC3; B_LO_RPN_PIN = FN_OC2; break;
      case 2:
        LATD = 0;
        B_HI_RPN_PIN = FN_NULL; B_LO_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL;
        A_HI_RPN_PIN = FN_OC1; A_LO_RPN_PIN = FN_OC3; C_LO_RPN_PIN = FN_OC2; break;
      case 3:
        LATD = 0;
        B_HI_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL; C_LO_RPN_PIN = FN_NULL;
        A_HI_RPN_PIN = FN_OC1; A_LO_RPN_PIN = FN_OC3; B_LO_RPN_PIN = FN_OC2; break;
      case 4:
        LATD = 0;
        A_HI_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL; C_LO_RPN_PIN = FN_NULL;
        B_HI_RPN_PIN = FN_OC1; B_LO_RPN_PIN = FN_OC3; A_LO_RPN_PIN = FN_OC2; break;
      case 5:
        LATD = 0;
        A_HI_RPN_PIN = FN_NULL; B_HI_RPN_PIN = FN_NULL; B_LO_RPN_PIN = FN_NULL;
        C_HI_RPN_PIN = FN_OC1; C_LO_RPN_PIN = FN_OC3; A_LO_RPN_PIN = FN_OC2; break;
      case 6:
        LATD = 0;
        A_HI_RPN_PIN = FN_NULL; A_LO_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL;
        B_HI_RPN_PIN = FN_OC1; B_LO_RPN_PIN = FN_OC3; C_LO_RPN_PIN = FN_OC2; break;
    }
  } else {
  // TODO: is there an error with this table, draws 10A when tries to do this at 90%dc
    switch (hall_state) {
      case 1:
        LATD = 0;
        A_HI_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL; C_LO_RPN_PIN = FN_NULL;
        B_HI_RPN_PIN = FN_OC1; B_LO_RPN_PIN = FN_OC3; A_LO_RPN_PIN = FN_OC2; break;
      case 2:
        LATD = 0;
        A_HI_RPN_PIN = FN_NULL; A_LO_RPN_PIN = FN_NULL; B_HI_RPN_PIN = FN_NULL;
        C_HI_RPN_PIN = FN_OC1; C_LO_RPN_PIN = FN_OC3; B_LO_RPN_PIN = FN_OC2; break;
      case 3:
        LATD = 0;
        A_HI_RPN_PIN = FN_NULL; B_HI_RPN_PIN = FN_NULL; B_LO_RPN_PIN = FN_NULL;
        C_HI_RPN_PIN = FN_OC1; C_LO_RPN_PIN = FN_OC3; A_LO_RPN_PIN = FN_OC2; break;
      case 4:
        LATD = 0;
        B_HI_RPN_PIN = FN_NULL; B_LO_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL;
        A_HI_RPN_PIN = FN_OC1; A_LO_RPN_PIN = FN_OC3; C_LO_RPN_PIN = FN_OC2; break;
      case 5:
        LATD = 0;
        A_HI_RPN_PIN = FN_NULL; A_LO_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL;
        B_HI_RPN_PIN = FN_OC1; B_LO_RPN_PIN = FN_OC3; C_LO_RPN_PIN = FN_OC2; break;
      case 6:
        LATD = 0;
        B_HI_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL; C_LO_RPN_PIN = FN_NULL;
        A_HI_RPN_PIN = FN_OC1; A_LO_RPN_PIN = FN_OC3; B_LO_RPN_PIN = FN_OC2; break;
    }
  }
  //DIRO = DIRI;  // output the current direction
  //T4CONbits.TON = 1;
}


static void UpdateDutyCycle(const uint16_t duty_cycle) {
  if (MAX_DC < duty_cycle) OC1R = MAX_DC;
  else OC1R = duty_cycle;
  
  // shadow the other variable
  OC2R = OC1R;
  OC3R = OC1R;
}

static void InitPins(void) {
  Coast();
  HALL_A_EN(1);
  HALL_B_EN(1);
  HALL_C_EN(1);
  DIRI_EN(1);
  DIRO_EN(1);
  TACHO_EN(1);
}

// NB: assumes that the associated pins are already configured as digital inputs
static void InitCNs(void) {
  _CNIE = 0;      // ensure all the CN interrupts are disabled as we configure
  
  // register the desired pins to throw a CN interrupt
  _CN63IE = 1;
  _CN64IE = 1;
  _CN65IE = 1;
  
  // initialize the change notification interrupt only once
  _CNIP = 0b110;  // the 2nd-highest priority
  _CNIF = 0;
  _CNIE = 1;
}

static void InitOCs(void) {
  // initialize the output compare timebase
  T4CONbits.TON = 0;        // ensure the timer is off while we configure it
  T4CONbits.TCKPS = 0b00;   // configure prescaler to divide-by-1
  PR4 = PWM_PERIOD;
  TMR4 = PR4;               // NB: important for initialization, so we don't have to wait a whole TMR4 period to get started?
  
  TRISD = 0; LATD = 0;
  InitHighSidePulser();
  InitLowSidePulser();
  InitRechargePulser();
  
  // turn on the timebase timer
  T4CONbits.TON = 1;
}

static void InitHighSidePulser(void) {
  OC1CON1bits.OCTSEL = 0b010;   // select timer4 as the time base
  OC1CON2bits.SYNCSEL = 0b01110;// triggered by timer4
  OC1CON2bits.OCINV = 1;        // invert the output for more intuitive 
                                // duty-cycle writes
  OC1R = DEFAULT_DC;            // initialize the duty cycle (NB: we cannot get 0%?)
  OC1RS = PWM_PERIOD;
  _OC1IP = 0b111;               // the highest priority
  _OC1IF = 0;                   // enable interrupts to be thrown when
  _OC1IE = 1;                   // timer4 counter is equal to OC1R
  
 
  OC1CON1bits.OCM = 0b101;      // Double Compare Continuous Pulse mode, turn on 
}

static void InitLowSidePulser(void) {
  OC2CON1bits.OCTSEL = OC1CON1bits.OCTSEL;
  OC2CON2bits.SYNCSEL = OC1CON2bits.SYNCSEL;
  OC2CON2bits.OCINV = OC1CON2bits.OCINV;
  OC2R = OC1R;
  OC2RS =  OC1RS;
  OC2CON1bits.OCM = OC1CON1bits.OCM;
}

static void InitRechargePulser(void) {
  OC3CON1bits.OCTSEL = OC1CON1bits.OCTSEL;
  OC3CON2bits.SYNCSEL = OC1CON2bits.SYNCSEL;
  // consume the remainder of the period with the recharge pulse
  OC3R = OC1R;  // NB: start as close as we can to when the interrupt fires
  OC3RS = PWM_PERIOD - CROSSOVER_TOLERANCE - REPULSE_INTERRUPT_DELAY;
}

static void Coast(void) {
  // turn everything off
  LATD = 0;
  A_HI_RPN_PIN = FN_NULL; A_LO_RPN_PIN = FN_NULL;
  B_HI_RPN_PIN = FN_NULL; B_LO_RPN_PIN = FN_NULL;
  C_HI_RPN_PIN = FN_NULL; C_LO_RPN_PIN = FN_NULL;
  LATD = 0; // to really ensure everything is off
}

static bool inline IsMotorStalled(void) {
  static uint16_t events_per_ms = 0;
  // NB: assuming our slowest speed
  // if we have not seen any commutation events
  // for a relatively long amount of time
  return (events_per_ms < MIN_N_EXPECTED_EVENTS_PER_MS);
}
