/*==============================================================================
File:ESC.c

Notes:
  - if noise becomes an issue, we can only switch the high-side transistor (and
    pulse its low-side counterpart) and leave the low-side transistor on
==============================================================================*/
//#define TEST_ESC
//---------------------------Dependencies---------------------------------------
#include "./core/StandardHeader.h"
#include "./motor_control.h"
//#include "./core/ADC.h"       // for A/D conversions
#include "./core/StandardHeader.h"  // for CCW, CW macros
#include "./core/PPS.h"             // for OC function name macros
#include <stdlib.h>             // for abs()
#include "home_office.h"
#include "uart.h"

//---------------------------Macros---------------------------------------------
// analog sensing pins (ANx)


// debugging
//#define HEARTBEAT_EN(a)     (_TRISB9 = !(a))
//#define HEARTBEAT_PIN       (_RB9)  // pin 8 of connector P8



static void UpdateLDutyCycle(const uint16_t duty_cycle);
static void UpdateRDutyCycle(const uint16_t duty_cycle);


// TODO: ensure these do NOT change
//#define DEFAULT_DC              500//956   // [ticks]
//#define PWM_PERIOD              1000  // [ticks], 1000 => ~16kHz

#define DEFAULT_DC                50
#define PWM_PERIOD                1000

// The time before the end of the period to schedule the falling edge of the
// recharge pulse.
//#define CROSSOVER_TOLERANCE     4     // [ticks], 28 leaves ~200ns before drive pulse begins
//#define CROSSOVER_TOLERANCE     8
#define CROSSOVER_TOLERANCE     24

// Time consumed in the interrupt to reinitialize the single-shot pulse.
#define REPULSE_INTERRUPT_DELAY 24    // [ticks]

// The minimum recharge pulse duration in units of timer4 ticks (62.5ns/tick).
//#define RECHARGE_DURATION       16    // [ticks], 16 => ~1us recharge pulse width
#define RECHARGE_DURATION       32

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


static void test_MOSFETs(void);

// OC-module related
static void InitOCs(void);

static void InitHighSidePulser(void);
static void InitLowSidePulser(void);
static void InitRechargePulser(void);


static void Coast_L(void);
static void Coast_R(void);



//---------------------------Module Variables-----------------------------------
static volatile uint16_t current_speed = 0;

static void Brake();

//---------------------------Test Harness---------------------------------------
#ifdef TEST_ESC
#include "./core/ConfigurationBits.h"
int main(void) {

  unsigned char hall_state_a = 0;
  unsigned char hall_state_b = 0;
  static unsigned int last_millisecond_counter = 0;

  

  ESC_Init();
  //allow power board to come online
  //test_commutation_position();


  /*_CN63PDE = 1;
  _CN64PDE = 1;
  _CN65PDE = 1;*/

  //ESC_StartMotor(400); Delay(500);


  //Energize(3);
  //Energize(1);
  //Energize(1);
  //Energize(1);
  //Energize(1);
  //Energize(1);
  
  _CNIE = 0;
  Brake();
  test_MOSFETs();
  while(1)
  {
    //Coast();

    ClrWdt();
  }

  while (1) {

 }
  
  return 0;
}
#endif

//---------------------------Interrupt Service Routines (ISRs)------------------



// This interrupt is thrown when the running timer4 counter counts up to OC1R
void __attribute__((__interrupt__, auto_psv)) _OC1Interrupt(void) {
  // restart the single-shot pulse

  /*unsigned int i;
  for(i=0;i<100;i++)
    Nop();*/

  OC3CON1bits.OCM = 0b000;
  OC6CON1bits.OCM = 0b000;


  OC3CON1bits.OCM = 0b100;
  OC6CON1bits.OCM = 0b100;

  _OC1IF = 0;
}

void __attribute__((__interrupt__, auto_psv)) _OC4Interrupt(void) {
  // restart the single-shot pulse
  //OC6CON1bits.OCM = 0b000;
  //OC6CON1bits.OCM = 0b100;
  //_OC4IF = 0;
}



//---------------------------Public Function Definitions------------------------
void ESC_Init(void) {
  InitOCs();
  //ADC_Init((1 << POT1_PIN) | (1 << POT2_PIN) | (1 << POT3_PIN));
  //HEARTBEAT_EN(1); HEARTBEAT_PIN = 0;
}

int16_t ESC_speed(void) {
 return current_speed;
}

static void set_motor_pwm(int left_speed, int right_speed)
{
  
  UpdateLDutyCycle(abs(left_speed)*8);
  UpdateRDutyCycle(abs(right_speed)*8);

  if(left_speed == 0)
  {
    Coast_L();
  }
  else if(left_speed > 0)
  {
    A_HI_L_RPN_PIN = FN_OC1; A_LO_L_RPN_PIN = FN_OC3; 
    B_HI_L_RPN_PIN = FN_NULL; B_LO_L_RPN_PIN = FN_OC2;
    //A_LO_L_RPN_PIN = FN_OC1; B_LO_L_RPN_PIN = FN_OC3;
  }
  else
  {
    A_HI_L_RPN_PIN = FN_NULL; A_LO_L_RPN_PIN = FN_OC2; 
    B_HI_L_RPN_PIN = FN_OC1; B_LO_L_RPN_PIN = FN_OC3;
  }

  if(right_speed == 0)
  {
    Coast_R();
  }
  else if(right_speed > 0)
  {
    A_HI_R_RPN_PIN = FN_OC4; A_LO_R_RPN_PIN = FN_OC6; 
    B_HI_R_RPN_PIN = FN_NULL; B_LO_R_RPN_PIN = FN_OC5;
    //A_LO_R_RPN_PIN = FN_OC4; 
    //B_LO_R_RPN_PIN = FN_OC6; 
  }
  else
  {
    A_HI_R_RPN_PIN = FN_NULL; A_LO_R_RPN_PIN = FN_OC5; 
    B_HI_R_RPN_PIN = FN_OC4; B_LO_R_RPN_PIN = FN_OC6;
  }


}

//---------------------------Helper Function Definitions------------------------



static void UpdateLDutyCycle(const uint16_t duty_cycle) {
  if (MAX_DC < duty_cycle) OC1R = MAX_DC;
  else OC1R = duty_cycle;
  
  // shadow the other variable
  OC2R = OC1R;
  OC3R = OC1R;
}


static void UpdateRDutyCycle(const uint16_t duty_cycle) {
  if (MAX_DC < duty_cycle) OC4R = MAX_DC;
  else OC4R = duty_cycle;
  
  // shadow the other variable
  OC5R = OC4R;
  OC6R = OC4R;


}

static void InitOCs(void) {
  // initialize the output compare timebase
  T4CONbits.TON = 0;        // ensure the timer is off while we configure it
  T4CONbits.TCKPS = 0b00;   // configure prescaler to divide-by-1
  PR4 = PWM_PERIOD;
  TMR4 = PR4;               // NB: important for initialization, so we don't have to wait a whole TMR4 period to get started?
  
  /*A_HI_L_EN(1);
  A_LO_L_EN(1);
  B_HI_L_EN(1);
  B_LO_L_EN(1);

  A_HI_R_EN(1);
  A_LO_R_EN(1);
  B_HI_R_EN(1);
  B_LO_R_EN(1);*/


  //TRISD = 0; LATD = 0;
  InitHighSidePulser();
  InitLowSidePulser();
  InitRechargePulser();
//  InitTimer();
  
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


  OC4CON1bits.OCTSEL = 0b010;   // select timer4 as the time base
  OC4CON2bits.SYNCSEL = 0b01110;// triggered by timer4
  OC4CON2bits.OCINV = 1;        // invert the output for more intuitive 
                                // duty-cycle writes
  OC4R = DEFAULT_DC;            // initialize the duty cycle (NB: we cannot get 0%?)
  OC4RS = PWM_PERIOD;
  //_OC4IP = 0b111;               // the highest priority
  //_OC4IF = 0;                   // enable interrupts to be thrown when
  //_OC4IE = 1;                   // timer4 counter is equal to OC1R
  
 
  OC4CON1bits.OCM = 0b101;      // Double Compare Continuous Pulse mode, turn on 


}

static void InitLowSidePulser(void) {
  OC2CON1bits.OCTSEL = OC1CON1bits.OCTSEL;
  OC2CON2bits.SYNCSEL = OC1CON2bits.SYNCSEL;
  OC2CON2bits.OCINV = OC1CON2bits.OCINV;
  OC2R = OC1R;
  OC2RS =  OC1RS;
  OC2CON1bits.OCM = OC1CON1bits.OCM;


  OC5CON1bits.OCTSEL = OC4CON1bits.OCTSEL;
  OC5CON2bits.SYNCSEL = OC4CON2bits.SYNCSEL;
  OC5CON2bits.OCINV = OC4CON2bits.OCINV;
  OC5R = OC4R;
  OC5RS =  OC4RS;
  OC5CON1bits.OCM = OC4CON1bits.OCM;
}

static void InitRechargePulser(void) {
  OC3CON1bits.OCTSEL = OC1CON1bits.OCTSEL;
  OC3CON2bits.SYNCSEL = OC1CON2bits.SYNCSEL;
  // consume the remainder of the period with the recharge pulse
  OC3R = OC1R;  // NB: start as close as we can to when the interrupt fires
  OC3RS = PWM_PERIOD - CROSSOVER_TOLERANCE - REPULSE_INTERRUPT_DELAY;


  OC6CON1bits.OCTSEL = OC4CON1bits.OCTSEL;
  OC6CON2bits.SYNCSEL = OC4CON2bits.SYNCSEL;
  // consume the remainder of the period with the recharge pulse
  OC6R = OC4R;  // NB: start as close as we can to when the interrupt fires
  OC6RS = PWM_PERIOD - CROSSOVER_TOLERANCE - REPULSE_INTERRUPT_DELAY;

}



static void Coast_L(void) {
  // turn everything off
  LATD = 0;
  A_HI_L_RPN_PIN = FN_NULL; A_LO_L_RPN_PIN = FN_NULL;
  B_HI_L_RPN_PIN = FN_NULL; B_LO_L_RPN_PIN = FN_NULL;
  LATD = 0; // to really ensure everything is off
}

static void Coast_R(void) {
  // turn everything off
  LATD = 0;
  A_HI_R_RPN_PIN = FN_NULL; A_LO_R_RPN_PIN = FN_NULL;
  B_HI_R_RPN_PIN = FN_NULL; B_LO_R_RPN_PIN = FN_NULL;
  LATD = 0; // to really ensure everything is off

}





//brake isn't working like I think it should -- need to investigate later
static void Brake(void)
{

}

static void test_MOSFETs(void)
{
  unsigned int i;
    Delay(5000);
  while(1)
  {

    //short in right side PWM code
    set_motor_pwm(0,0);
    Delay(2000);
    set_motor_pwm(-800, -800);
    Delay(2000);
    set_motor_pwm(0,0);
    Delay(2000);
    set_motor_pwm(800,800);
    Delay(2000);
  }

  


}

void motor_control_test_function(void)
{


  test_MOSFETs();
}

void motor_control_FSM(void)
{
  static unsigned int overcurrent_counter = 0;
  static unsigned int overcurrent_wait_counter = 0;
  static unsigned int communication_timeout_counter = 0;

  typedef enum {
    sRunning = 0,
    sWaitingAfterOvercurrent,
    sCommunicationTimeout,
  } sOvercurrentState;

  static sOvercurrentState state = 0;

  switch(state)
  {
    case sRunning:

      if(ADC1BUF3 > OVERCURRENT_ADC)
      {
        overcurrent_counter++;
      }
      else
      {
        overcurrent_counter = 0;
      }
      
      if(overcurrent_counter > 10)
      {
        set_motor_pwm(0,0);
        state = sWaitingAfterOvercurrent;
      }
      else
      {
        
      }

      if(new_motor_control_message_flag)
      {
        new_motor_control_message_flag = 0;
        communication_timeout_counter = 0;
        set_motor_pwm(last_motor_commands[0],last_motor_commands[1]);
      }
      else
      {
        communication_timeout_counter++;
      }

      if(communication_timeout_counter >= 50)
      {
        state = sCommunicationTimeout;
      }


    break;
    case sWaitingAfterOvercurrent:

      set_motor_pwm(0,0);
      overcurrent_wait_counter++;
      if(overcurrent_wait_counter > 200)
      {
        state = sRunning;
        overcurrent_wait_counter = 0;
      }

    break;
    case sCommunicationTimeout:
      set_motor_pwm(0,0);
      if(new_motor_control_message_flag)
      {
        state = sRunning;
      }
    break;
  }

}