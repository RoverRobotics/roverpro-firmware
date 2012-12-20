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
static void InitCNs(void);
static uint8_t inline hall_state(void);

//---------------------------Module Variables-----------------------------------

//---------------------------Test Harness---------------------------------------
#ifdef TEST_ESC
#include "./core/ConfigurationBits.h"
int main(void) {
  ESC_Init();
  ESC_StartMotor();
  //Energize(3);
  while (1) {
    /*
    // test full CW sequence
    Energize(1); Delay(1000);
    Energize(5); Delay(1000);
    Energize(4); Delay(1000);
    Energize(6); Delay(1000);
    Energize(2); Delay(1000);
    Energize(3); Delay(1000);
    */
  }
  
  return 0;
}
#endif

//---------------------------Interrupt Service Routines (ISRs)------------------
void __attribute__((__interrupt__, auto_psv)) _CNInterrupt(void) {
//  uint8_t temp = hall_state();
//  Nop();
//  Nop();
  Energize(hall_state()); // energizes the appropriate coils to move to the next position
  _CNIF = 0;              // clear the source of the interrupt
}

//---------------------------Public Function Definitions------------------------
void ESC_Init(void) {
  InitPins();
  InitCNs();
  OC_Init();
}

void ESC_StartMotor(void) {
  Energize(hall_state());
}

//---------------------------Helper Function Definitions------------------------
static uint8_t inline hall_state(void) {
  static uint8_t tempA, tempB, tempC = 0; // make lookup a little faster
  // BUG ALERT: must have Nop() to guarantee sufficient time to read
  tempA = HALL_A; Nop();
  tempB = HALL_B; Nop();
  tempC = HALL_C; Nop();
  return ((tempC << 2) | (tempB << 1) | (tempA << 0));
}

static void InitPins(void) {
  HALL_A_EN(1);
  HALL_B_EN(1);
  HALL_C_EN(1);
}

// NB: assumes that the associated pins are already configured as digital inputs
static void InitCNs(void) {
  _CNIE = 0;  // ensure all the CN interrupts are disabled as we configure
  
  // register the desired pins to throw a CN interrupt
  _CN63IE = 1;
  _CN64IE = 1;
  _CN65IE = 1;
  
  // initialize the change notification interrupt only once
  _CNIF = 0;
  _CNIE = 1;
}
