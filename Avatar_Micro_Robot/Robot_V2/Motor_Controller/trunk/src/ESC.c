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
#include "Testing.h"


//---------------------------Macros---------------------------------------------
// analog sensing pins (ANx)
#define POT2_PIN            12  // TODO: remove after debugging board

// user interface pins
#define DIRI_EN(a)          (_TRISD7 = (a)); //AD1PCFGL |= (1 << 2)
#define DIRI                (_RD7)          
#define DIRO_EN(a)          (_TRISC13 = !(a)); //AD1PCFGL |= (1 << 4)
#define DIRO                (_RC13)          // pin 2 of P9
#define TACHI_RPN_PIN       (_RP8)
#define TACHI_RP_NUM        8
#define TACHO_EN(a)         (_TRISB5 = !(a)); //AD1PCFGL |= (1 << 5)
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

#define A_HI_EN(a)          (_TRISD0 = !(a))
#define A_LO_EN(a)          (_TRISD1 = !(a))
#define B_HI_EN(a)          (_TRISD2 = !(a))
#define B_LO_EN(a)          (_TRISD3 = !(a))
#define C_HI_EN(a)          (_TRISD4 = !(a))
#define C_LO_EN(a)          (_TRISD5 = !(a))

#define A_LO                (_LATD1)
#define A_LO_MASK           0b10
#define B_LO                (_LATD3)
#define B_LO_MASK           0b1000
#define C_LO                (_LATD5)
#define C_LO_MASK           0b100000


// TODO: ensure these do NOT change
//#define DEFAULT_DC              500//956   // [ticks]
//#define PWM_PERIOD              1000  // [ticks], 1000 => ~16kHz

#define DEFAULT_DC                250
#define PWM_PERIOD                500
//#define PWM_PERIOD                1000

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
static void InitPins(void);
static void InitCNs(void);
static void Coast(void);
static bool inline IsMotorStalled(void);

// OC-module related
static void Energize(const uint8_t hall_state);
static void InitOCs(void);
static void InitIC(void);
static void UpdateDutyCycle(const uint16_t duty_cycle);
static void InitHighSidePulser(void);
static void InitLowSidePulser(void);
static void InitRechargePulser(void);

static void InitTimer(void);
static unsigned int millisecond_counter = 0;
static unsigned int ten_millisecond_counter = 0;
static unsigned int hundred_millisecond_counter = 0;
static unsigned int second_counter = 0;
static void test_commutation_position(void);
static void handle_stall(void);

static unsigned char energized_hall_state;

static unsigned char currently_stalled = 0;

static void handle_tachi_timeout(void);
static unsigned int last_IC_period = 0;
static unsigned char IC_period_updated = 0;
static unsigned int return_speed_command(void);

static unsigned char commutation_flag = 0;

static unsigned int temp_speed_command = 0;
static unsigned int temp_uncorrected_speed = 0;
static unsigned int max_temp_speed_command = 0;
//---------------------------Module Variables-----------------------------------
static volatile uint16_t current_speed = 0;

static unsigned char filtered_direction = 0;

static void handle_filtering();

static void Brake();

static void test_MOSFETs(void);

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
  
  //_CNIE = 0;
  Brake();
  //test_MOSFETs();
  /*while(1)
  {
    Coast();
    Delay(1000);
    Brake();
    Delay(1000);

    ClrWdt();
  }*/

  while(1)
  {
    if(ten_millisecond_counter)
    {
      ten_millisecond_counter = 0;
      motor_temperature_test();
    }
  }

  while (1) {

    if(millisecond_counter != last_millisecond_counter)
    {
      handle_tachi_timeout();

      //TODO: reimplement stall check and timeout
      if(currently_stalled == 0)
      {
        temp_speed_command = return_speed_command();
        ESC_set_speed(temp_speed_command);
        if(temp_speed_command > max_temp_speed_command)
          max_temp_speed_command = temp_speed_command;
      }
      last_millisecond_counter = millisecond_counter;
    }

    if(ten_millisecond_counter)
    {
      ten_millisecond_counter = 0;
      handle_stall();
    }

    

    /*
    if (IsMotorStalled()) {
      Coast();
      Delay(DELAY_MS);
      asm("reset");
    }
    */
   /* hall_state_a = HALL_STATE;
    Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); 
    hall_state_b = HALL_STATE;
    if(hall_state_a == hall_state_b)
    {
      Nop();
      Nop();
      Energize(hall_state_a);
    }
    Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); Nop(); */

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
  //unsigned char temp_hall_state = HALL_STATE;
  //Nop();
  Energize(HALL_STATE);   // energize the appropriate coils to move to the next position
  TACHO ^= 1;             // output that we commutated
  _CNIF = 0;              // clear the source of the interrupt
}

//---------------------------Public Function Definitions------------------------
void ESC_Init(void) {
  InitPins();
  InitCNs();
  InitOCs();
  InitIC();
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
  

  /*if(hall_state != HALL_STATE) 
  {
    Nop();
    return;
  }*/

  // map the desired pins to the existing output compare modules
  //if (DIRI) {//DIRI == CCW)
    if(DIRI==0) {
    switch (hall_state) {
      case 0:
        Coast(); break;
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
      case 7:
        Coast(); break;
    }
  } else {
  // TODO: is there an error with this table, draws 10A when tries to do this at 90%dc
    switch (hall_state) {
      case 0:
        Coast(); break;
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
      case 7:
        Coast(); break;
    }
  }

  energized_hall_state = hall_state;
  commutation_flag = 1;
  //DIRO = DIRI;  // output the current direction
  //T4CONbits.TON = 1;
}


static void UpdateDutyCycle(const uint16_t duty_cycle) {
  if (MAX_DC < duty_cycle) OC1R = MAX_DC;
  else OC1R = duty_cycle;
  
  // shadow the other variable
  OC2R = OC1R;
  OC3R = OC1R;
  OC3RS = OC1R+RECHARGE_DURATION; 
}

static void InitPins(void) {
  AD1PCFGL = 0xffff;
  AD1PCFGH = 0xffff;
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
  
  A_HI_EN(1);
  A_LO_EN(1);
  B_HI_EN(1);
  B_LO_EN(1);
  C_HI_EN(1);
  C_LO_EN(1);
  //TRISD = 0; LATD = 0;
  InitHighSidePulser();
  InitLowSidePulser();
  InitRechargePulser();
  InitTimer();
  
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
  //OC3RS = PWM_PERIOD - CROSSOVER_TOLERANCE - REPULSE_INTERRUPT_DELAY;
  OC3RS = OC1R+RECHARGE_DURATION; //try to get minimum recharge pulse
}

static void InitTimer(void)
{

  //T2 frequency is Fosc/2 = 16 MHz
  //Max timer interrupt at 2^16/16MHz = 4.096ms
  //Let's set PR2 to 16000, to get interrupts every 1ms:
  PR2 = 16000;
  _T2IE = 1;
  T2CONbits.TON = 1;

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

static void test_commutation_position(void)
{

  unsigned int i;
  unsigned int state_order[8] = {0, 1, 5, 4, 6, 2, 3, 7};

  //disable CN interrupts
  //_CNIE = 0;

  Coast();
  while(1);

  for(i=0;i<8;i++)
  {
    //Energize(i);
    Energize(state_order[i]);
    Delay(500);
    Coast();
    Delay(3000);
  }


  
  while(1);


}


void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
  _T2IF = 0;
  millisecond_counter++;
  
  //Make sure rollover is at a multiple of 10
  if(millisecond_counter == 10000)
    millisecond_counter = 0;


  if(millisecond_counter%10 == 0)
  {
    ten_millisecond_counter++;
    if(millisecond_counter%100 == 0)
    {
      hundred_millisecond_counter++;
      if(millisecond_counter%1000 == 0)
      {
        second_counter++;
      }
     
    }
  }
}

static void handle_stall(void)
{

  typedef enum {
    sStalled = 0,
    sStallRestart,
    sNoStall,
  } sStallState;

  static unsigned char current_hall_state = 0;
  static unsigned char last_hall_state = 0;
  static unsigned int stall_counter = 0;
  static unsigned char stall_retry_counter = 0;
  static unsigned char currently_stalled_flag = 0;
  static unsigned char restart_hall_state = 0;
  static sStallState state = sNoStall;

  current_hall_state = HALL_STATE;

switch(state)
{
  case sNoStall:

    if(commutation_flag == 0)
    {
      stall_counter++;
    }
    else
    {
      stall_counter = 0;
    }

    commutation_flag = 0;

    if(stall_counter >= 3)
    {
      if(current_hall_state != energized_hall_state)
        Energize(current_hall_state);

      stall_counter = 0;
      /*_CNIE = 0;
      state = sStallRestart;
      restart_hall_state = current_hall_state;*/
    }
    break;
    if(stall_counter >= 20)
    {
      Coast();
      state = sStalled;
      energized_hall_state = 0;
      restart_hall_state = current_hall_state;
      currently_stalled = 1;
    }

  break;

  case sStalled:
    ESC_set_speed(0);
    stall_retry_counter++;
    if(stall_retry_counter < 200)
      break;

    stall_retry_counter = 0;
    stall_counter = 0;
    ESC_set_speed(300);
    state = sStallRestart;
  break;

  case sStallRestart:
    Energize(current_hall_state);

    if(restart_hall_state != current_hall_state)
    {
      _CNIE = 1;
      currently_stalled = 0;
      //ESC_set_speed(200);
      state = sNoStall;
    }

  break;


}




     


}



void __attribute__((__interrupt__, auto_psv)) _IC1Interrupt(void) {
  static unsigned char rising_edge_detected_last = 1;
  _IC1IF = 0;                      // clear the source of the interrupt
  
  static uint16_t last_value = 0;
  uint16_t current_value = IC1BUF; // current running Timer3 tick value
                                   // (you must subtract off last value)

  //rising edge
  if(rising_edge_detected_last == 0)
  {
    last_value = current_value;
    rising_edge_detected_last = 1;
    IC1CON1bits.ICM = 0b010;  //trigger on falling edge

  }
  //falling edge, so calculate high time
  else
  {

    //If the signal was continuously high, there would have been an overflow
    //TODO: verify that this actually gets caught
    if(IC1CON1bits.ICOV)
    {
      rising_edge_detected_last = 0;
      IC1CON1bits.ICM = 0b011;  //trigger on rising edge
      IC_period_updated = 1;
      return;
    }
  
	  // handle rollover, remove old offset
    if (last_value < current_value) last_IC_period = current_value - last_value;
    else last_IC_period = (0xffff - last_value) + current_value;
    rising_edge_detected_last = 0;
    IC1CON1bits.ICM = 0b011;  //trigger on rising edge
    IC_period_updated = 1;

    //If the pulse is greater than 2000, then we likely missed the last falling edge
    //TODO: Upper limit in case the pulse was actually high for an extended amount of time ?
    if(last_IC_period > 2000)
    {
      last_IC_period = 0;
    }
  }
}

static void InitIC(void)
{

  // clear the input capture FIFO buffer
  uint16_t temp;
	while (IC1CON1bits.ICBNE) {temp = IC1BUF;};

  // configure the input capture
  IC1CON1bits.ICTSEL = 0b100;   // use Timer1 as the time base
  IC1CON1bits.ICI = 0b00;   // fire the interrupt every capture event
  IC1CON1bits.ICM = 0b011;  // capture event on every rising edge

  IC1CON2bits.SYNCSEL = 0b01011; //Timer 1
  //IC1CON2bits.SYNCSEL = 0b10110;
  
  _IC1R = TACHI_RP_NUM; 


  _IC1IF = 0;               // begin with the interrupt flag cleared
  _IC1IE = 1;               // enable this interrupt

  T1CONbits.TON = 1;


}

static void handle_tachi_timeout(void)
{
  
  if((IC_period_updated == 0) && (_RB8==0))
  {
    last_IC_period = 0;
  }
  else if((IC_period_updated == 0))
  {
    //last_IC_period = 2000;
  }
  else
  {
    IC_period_updated = 0;
  }

}
static unsigned int return_speed_command(void)
{
  unsigned int IC_period = last_IC_period;
  unsigned int uncorrected_speed = 0;

  if(IC_period == 0)
    {
      Coast();
      energized_hall_state = 0;
    return 0;
    }
  else
  {
    
    //uncorrected_speed = IC_period/4;
    //uncorrected_speed = 100+IC_period/5;
    uncorrected_speed = 200+IC_period/6.7;
    
    //uncorrected_speed = 300;
    temp_uncorrected_speed = uncorrected_speed;
    //return 0;
    if(uncorrected_speed > 400) 
    {
      Nop();
      return 400;
    }
    //else if (uncorrected_speed < 100)
   //   return 0;

    return uncorrected_speed;

  }


}

static void Brake(void)
{
  // turn everything off, except low side
  LATD = 0 | A_LO_MASK | B_LO_MASK | C_LO_MASK;
  B_HI_RPN_PIN = FN_NULL; C_HI_RPN_PIN = FN_NULL; C_LO_RPN_PIN = FN_NULL;
  A_HI_RPN_PIN = FN_NULL; A_LO_RPN_PIN = FN_NULL; B_LO_RPN_PIN = FN_NULL;
  LATD = 0 | A_LO_MASK | B_LO_MASK | C_LO_MASK; // to really ensure everything is off

}

static void test_MOSFETs(void)
{
  unsigned int i;
  Brake();
  UpdateDutyCycle(400);
  while(1)
  {
  for(i=1;i<7;i++)
  {
    Energize(i);
    Delay(2000);
  }
  }
  


}

