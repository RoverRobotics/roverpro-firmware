/*==============================================================================
File:ESC.c

Notes:
  - uses output compare with a dedicated timer since PWM won't latch until a 
    period is complete.
==============================================================================*/
#define TEST_ESC
//---------------------------Dependencies---------------------------------------
#include "./core/StandardHeader.h"
#include "./ESC.h"
#include "./core/Timers.h"    // for timers
#include "./core/ADC.h"       // for A/D conversions
#include "./core/PPS.h"
#include "./OC.h"

//---------------------------Macros---------------------------------------------
// analog sensing pins (ANx)
#define M1_T_PIN            2           // motor1 temperature sensing
#define M1_I_PIN            3           // motor1 current sensing
#define POT1_PIN            11
#define POT2_PIN            12
#define POT3_PIN            13

// digital inputs
#define HALL_A_EN(a)        (_TRISE5 = (a))
#define HALL_A              (_RE5)
#define HALL_B_EN(a)        (_TRISE6 = (a))
#define HALL_B              (_RE6)
#define HALL_C_EN(a)        (_TRISE7 = (a))
#define HALL_C              (_RE7)

// NB: the hall effect sensor inputs go to both input capture pins as well as digital inputs
// input-capture inputs
#define HALL_A_RPN          21
#define HALL_B_RPN          19
#define HALL_C_RPN          37  // input-only RPn

#define N_PHASES            6

//---------------------------Constants------------------------------------------
/*
Description: This lookup table is used to write the commutation state to 
  the control pins.  It assumes exclusive use of a port and that the high-side
  and low-side commutation pins are grouped as follows: 
              control pin: C_LO C_HI B_LO B_HI A_LO A_HI
              port bit:       5    4    3    2    1    0
Notes:
  - the high-side and low-side transistor sequences
    must be written in a manner that protects against "shoot-through."
    As we step through the states, this should NEVER occurr since there are 
    never two contiguous states where both the high and low-side of the 
    same transistor pair are on.
  - this starting point of this table must also be synced with the hall-effect
    sensor configuration
*/
//static const uint8_t kDriveTable[N_PHASES] = { 
//  0x18, 0x12, 0x06, 0x24, 0x21, 0x09
//};

//---------------------------Type Definitions-----------------------------------
// possible states
typedef enum {
  kWaiting = 0,
  kRunning,
} kControllerState;

//---------------------------Helper Function Prototypes-------------------------
static void InitPins(void);
static void InitInputCapture(void);
static void InitInputCaptureTimebase(void);
static void InitIC1(const uint8_t RPn);
static void InitIC2(const uint8_t RPn);
static void InitIC3(const uint8_t RPn);
static uint8_t inline hall_state(void);

//---------------------------Module Variables-----------------------------------

//---------------------------Test Harness---------------------------------------
#ifdef TEST_ESC
#include "./core/ConfigurationBits.h"
int main(void) {
  ESC_Init();
  ESC_StartMotor();
  while (1) {
  }
  
  return 0;
}
#endif

//---------------------------Interrupt Service Routines (ISRs)------------------
void __attribute__((__interrupt__, auto_psv)) _IC1Interrupt(void) {
  _IC1IF = 0;                           // clear the source of the interrupt
  Energize(hall_state()); // energizes the appropriate coils to move to the next position
}

void __attribute__((__interrupt__, auto_psv)) _IC2Interrupt(void) {
  _IC2IF = 0;
  Energize(hall_state());
}

void __attribute__((__interrupt__, auto_psv)) _IC3Interrupt(void) {
  _IC3IF = 0;
  Energize(hall_state());
}

//---------------------------Public Function Definitions------------------------
void ESC_Init(void) {
  InitPins();
  
  // initialize any dependent module(s)
  /*
  ADC_Init((1 << M1_T_PIN) | 
           (1 << M1_I_PIN) |
           (1 << POT1_PIN) |
           (1 << POT2_PIN) |
           (1 << POT3_PIN));
  */
  InitInputCapture();
  OC_Init();
  TMRS_Init();
}


void ESC_StartMotor(void) {
  Energize(hall_state());
}

void ESC_set_speed(const float speed) {
  // TODO: incomplete implementation
}

float ESC_speed(void) {
  // TODO: incomplete implementation
  return 0;
}

//---------------------------Helper Function Definitions------------------------
static uint8_t inline hall_state(void) {
  uint8_t tempA = HALL_A; Nop(); Nop();
  uint8_t tempB = HALL_B; Nop(); Nop();
  uint8_t tempC = HALL_C; Nop(); Nop();
  // TODO: handle invalid hall_state of 0 or 7
  return ((tempC << 2) | (tempB << 1) | (tempA << 0));
}

static void InitPins(void) {
  // initialize any digital I/O pin(s)
  HALL_A_EN(1);
  HALL_B_EN(1);
  HALL_C_EN(1);
}

static void InitInputCapture(void) {
  InitInputCaptureTimebase();
  InitIC1(HALL_A_RPN);
  InitIC2(HALL_B_RPN);
  InitIC3(HALL_C_RPN);
}

static void InitInputCaptureTimebase(void) {
  T3CONbits.TCKPS = 0b00;   // configure prescaler to divide-by-1
  _T3IF = 0; _T3IE = 0;     // disable the associated interrupt
  T3CONbits.TON = 1;        // turn on the timer
}

static void InitIC1(const uint8_t RPn) {
  // clear the input capture FIFO buffer
  uint16_t temp;
	while (IC1CON1bits.ICBNE) {temp = IC1BUF;};
  
  // configure the input capture
  IC1CON1bits.ICTSEL = 0;   // use Timer3 as the time base
  IC1CON1bits.ICI = 0b00;   // fire the interrupt every capture event
  IC1CON1bits.ICM = 0b001;  // capture event on every edge
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC1);

  _IC1IF = 0;               // begin with the interrupt flag cleared
  _IC1IE = 1;               // enable this interrupt
}

static void InitIC2(const uint8_t RPn) {
  uint16_t temp;
	while (IC2CON1bits.ICBNE) {temp = IC2BUF;};

  IC2CON1bits.ICTSEL = 0;
  IC2CON1bits.ICI = 0b00;
  IC2CON1bits.ICM = 0b001;
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC2);
  
  _IC2IF = 0;
  _IC2IE = 1;
}

static void InitIC3(const uint8_t RPn) {
  uint16_t temp;
	while (IC3CON1bits.ICBNE) {temp = IC3BUF;};

  IC3CON1bits.ICTSEL = 0;
  IC3CON1bits.ICI = 0b00;
  IC3CON1bits.ICM = 0b001;
  
  PPS_MapPeripheral(RPn, INPUT, FN_IC3);
  
  _IC3IF = 0;
  _IC3IE = 1;
}
