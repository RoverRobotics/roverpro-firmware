/*=============================================================================
File: Hitch.c
=============================================================================*/
//#define TEST_HITCH
/*---------------------------Dependencies------------------------------------*/
#include <p24FJ256GB106.h>
#include "./Timers.h"
#include "./ADC.h"
#include "./PWM.h"
#include "./stdhdr.h"         // to get access to our custom registers

/*---------------------------Wiring Macros-----------------------------------*/
// General Definitions
#define OUTPUT                0
#define INPUT                 1  
#define OFF                   0
#define ON                    1
#define CW                    0
#define CCW                   1

// Latch Motoracce
/*
//---A_SCALED, B_SCALED, C_SCALED
#define CONFIG_A_SCALED(a)    (_TRISD0 = (a)) 
#define TURN_A_SCALED(a)      (_LATD0 = (a))
#define CONFIG_B_SCALED(a)    (_TRISD1 = (a)) 
#define TURN_B_SCALED(a)      (_LATD1 = (a))
#define CONFIG_C_SCALED(a)    (_TRISD2 = (a)) 
#define TURN_C_SCALED(a)      (_LATD2 = (a))
//---V_STAR
#define CONFIG_V_STAR(a)      (_TRISB3 = (a)) 
#define TURN_V_STAR(a)        (_LATB3 = (a))
//---A_HI, B_HI, C_HI, A_LO, B_LO, C_LO
#define CONFIG_A_HI(a)        (_TRISD0 = (a)) 
#define TURN_A_HI(a)          (_LATD0 = (a))
#define CONFIG_B_HI(a)        (_TRISD1 = (a)) 
#define TURN_B_HI(a)          (_LATD1 = (a))
#define CONFIG_C_HI(a)        (_TRISD2 = (a)) 
#define TURN_C_HI(a)          (_LATD2 = (a))
#define CONFIG_A_LO(a)        (_TRISD3 = (a))
#define TURN_A_LO(a)          (_LATD3 = (a))
#define CONFIG_B_LO(a)        (_TRISD4 = (a))
#define TURN_B_LO(a)          (_LATD4 = (a))
#define CONFIG_C_LO(a)        (_TRISD5 = (a))
#define TURN_C_LO(a)          (_LATD5 = (a))
*/
//---PWM Pin
#define LATCH_PWM_PIN         2    // RP2
#define T_PWM                 10   // [us], period of the PWM signal
#define MAX_DC                20   // T = 2ms
#define MAX_ALLOWABLE_DC      16   // T = 1.6
#define NUETRAL_DC            15   // T = 1.5
#define MIN_ALLOWABLE_DC      14   // T = 1.4
#define MIN_DC                10   // T = 1ms
//---Power Bus
#define CONFIG_POWER_BUS(a)   (_TRISD9 = (a))
#define TURN_POWER_BUS(a)     (_LATD9 = (a))

// Actuator Position Sensing
#define ACTUATOR_ANALOG_PIN   4
#define V_UNLATCHED           976 // (3.15 / 3.30) * 1023=
#define V_LATCHED             124 // (0.40 / 3.30) * 1023=

// Temperature Sensing
#define TEMP_ANALOG_PIN       8

// Power Bus Current Sensing
#define CURRENT_ANALOG_PIN    15

// Heartbeat Indicator
#define CONFIGURE_HEARTBEAT_PIN(a)   (_TRISE5 = (a))
#define HEARTBEAT_PIN         (_RE5)

// Times and Timers
#define _100ms                100
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        (10*_100ms)
#define ESC_DELAY_TIMER       1
#define ESC_DELAY_TIME        (5*_100ms)
#define LATCH_ACCEL_TIMER     2
#define UPDATE_TIMER          3
#define UPDATE_TIME           _100ms  // update software-register interfaces every ~10Hz

#define TEST_TIMER            4
#define TEST_TIME             (10*_100ms)

#define DELTA_DC              1
#define DELTA_LATCH_TIME      (_100ms)

/*---------------------------Type Definitions--------------------------------*/
typedef enum {
  WAITING = 0,
  LATCHING
} state_t;  

/*---------------------------Helper Function Prototypes----------------------*/
static void CalibrateHobbyMotorController(void);
static void Latch(unsigned char direction);
static unsigned char inline IsLatched(void);
static unsigned char inline IsUnlatched(void);
static unsigned int Map(int value, int from_low, int from_high, 
                        int to_low, int to_high);
                 
/*---------------------------Module Variables--------------------------------*/
static state_t state = WAITING;

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_HITCH
#include "./ConfigurationBits.h"

int main(void) {
	unsigned char alternator = 0;
	
	InitHitch();

  while (1) {
  	// toggle a pin to indicate normal operation
		if (IsTimerExpired(HEARTBEAT_TIMER)) {
			HEARTBEAT_PIN ^= 1;
      StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
		}
		
  	switch (state) {
      case WAITING:
    	  if (IsTimerExpired(TEST_TIMER)) {
      	  state = LATCHING;
      	  alternator++;
      	  StartTimer(LATCH_ACCEL_TIMER, DELTA_LATCH_TIME);
      	  StartTimer(TEST_TIMER, TEST_TIME); 
      	}  
    	  break;
    	case LATCHING:
    	  if (alternator % 2) {
      	  if (IsLatched()) {
            UpdateDutyCycle(NUETRAL_DC);
            state = WAITING;
            break;
          }
      	  Latch(ON);
        } else {
          if (IsUnlatched()) {
            UpdateDutyCycle(NUETRAL_DC);
            state = WAITING;
            break;
          }
          Latch(OFF);
    	  }
    	  break;
    	default:
    	  StartTimer(HEARTBEAT_TIMER, 65000); // indicate an error
    		break;
    }
	}
	
	return 0;
}

#endif
/*---------------------------End Test Harness--------------------------------*/
/*---------------------------Public Function Definitions---------------------*/
void InitHitch(void) {
  CONFIGURE_HEARTBEAT_PIN(OUTPUT); HEARTBEAT_PIN = 0;
  // configure any associated external IC's
  // assign any application-dependent ISR's
	// initialize any dependent moduels
	InitTimers();
	unsigned int analog_bit_mask = ( (1 << TEMP_ANALOG_PIN)     |
                                   (1 << ACTUATOR_ANALOG_PIN) |
                                   (1 << CURRENT_ANALOG_PIN));
  InitADC(analog_bit_mask);
  InitPWM(LATCH_PWM_PIN, T_PWM);
  
  // initialize the latch motor controller
  CONFIG_POWER_BUS(OUTPUT); TURN_POWER_BUS(ON); Nop(); Nop();
  CalibrateHobbyMotorController();
  
	// prime any timers that require it
	StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
	StartTimer(TEST_TIMER, TEST_TIME);
}

void ProcessHitchIO(void) {
 	// toggle a pin to indicate normal operation
	if (IsTimerExpired(HEARTBEAT_TIMER)) {
		HEARTBEAT_PIN ^= 1;
		StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
	}

  // only update feedback to software as often as needed
  if (IsTimerExpired(UPDATE_TIMER)) {
    REG_HITCH_POSITION = Map(GetADC(ACTUATOR_ANALOG_PIN), V_UNLATCHED, V_LATCHED, 0, 100);
    StartTimer(UPDATE_TIMER, UPDATE_TIME);
  }  

  switch (state) {
    case WAITING:
      if (IsLatched() && REG_HITCH_OPEN) {
        state = LATCHING;
    	  StartTimer(LATCH_ACCEL_TIMER, DELTA_LATCH_TIME);
    	  Latch(OFF);
    	} else if (IsUnlatched() && !REG_HITCH_OPEN) {
    	  state = LATCHING;
    	  StartTimer(LATCH_ACCEL_TIMER, DELTA_LATCH_TIME);
    	  Latch(ON);
    	}
  	  break;
  	case LATCHING:
  	  if (IsUnlatched() && REG_HITCH_OPEN) {
        UpdateDutyCycle(NUETRAL_DC);
        state = WAITING;
      } else if (IsLatched() && !REG_HITCH_OPEN) {
    	  UpdateDutyCycle(NUETRAL_DC);
        state = WAITING;
      }
      break;
  	default:
  	  StartTimer(HEARTBEAT_TIMER, 65000); // indicate an error
  		break;
  }
}

/*---------------------------Helper Function Definitions---------------------*/
/*****************************************************************************
Function: CalibrateHobbyMotorController()
Description: Performs the calibration sequence as defined by the Losi8750KV 
 Hobby sensorless BLDC Motor and controller.
Notes:
  - http://losi.com/Products/Default.aspx?ProdID=LOSB9594
  - WARNING: must pause at nuetral for some period of time before changing 
    direction for hobby controller to recognize the change
******************************************************************************/
static void CalibrateHobbyMotorController(void) {
  // wait in nuetral long enough for the controller (beeps when ready)
  UpdateDutyCycle(NUETRAL_DC);
  //Pause(5000);
}
  
/*****************************************************************************
Function: Latch()
Description: 
Notes:
  - accelerates up to the latching speed to avoid jamming the ball screw
  - use the linear potentiometer to realize when we've reached our destination
  - avoid changing direction faster than the Hobby controller can handle 
    (must pause for ~1s before can change direction) HANDLED IN SOFTWARE
******************************************************************************/
static void Latch(unsigned char direction) {
  //static unsigned char latching_dc = NUETRAL_DC;

  if (direction == ON) {
    UpdateDutyCycle(MAX_ALLOWABLE_DC);
    /*
    if (IsTimerExpired(LATCH_ACCEL_TIMER)) {
      latching_dc += DELTA_DC;
      if (latching_dc > MAX_ALLOWABLE_DC) latching_dc = MAX_ALLOWABLE_DC;
      UpdateDutyCycle(latching_dc);
      StartTimer(LATCH_ACCEL_TIMER, DELTA_LATCH_TIME);
    }
    */
  } else if (direction == OFF) {
    UpdateDutyCycle(MIN_ALLOWABLE_DC);
    /*
    if (IsTimerExpired(LATCH_ACCEL_TIMER)) {
      latching_dc -= DELTA_DC;
      if (latching_dc < MIN_ALLOWABLE_DC) latching_dc = MIN_ALLOWABLE_DC;
      UpdateDutyCycle(latching_dc);
      StartTimer(LATCH_ACCEL_TIMER, DELTA_LATCH_TIME);
    }
    */
  }
  
}

static unsigned char inline IsLatched(void) {
  return (GetADC(ACTUATOR_ANALOG_PIN) < V_LATCHED);
}

static unsigned char inline IsUnlatched(void) {
  return (V_UNLATCHED < GetADC(ACTUATOR_ANALOG_PIN));
}

/*
Function: Map()
Returns:
  unsigned int, the given value mapped to the new range
Parameters:
  int value,      the number to map
  int from_low,   the lower bound of the value's current range
  int from_high,  the upper bound of the value's current range
  int to_low,     the lower bound of the value's target range
  int to_high,    the upper bound of the value's target range
Description: Re-maps a number from one range to another.
Notes:
  - constrains values to within the range
  - be wary of overflow errors producing unexpected results
  - promotes data types and forces division before multiplication 
    to help mitigate unexpected overflow issues
*/
static unsigned int Map(int value, int from_low, int from_high, 
                        int to_low, int to_high) {
  
  // compute the linear interpolation
  unsigned int result = ((double)(value - from_low) / (double)(from_high - from_low))
                        * (double)(to_high - to_low) + to_low;
  
  // constrain the result to within a valid output range
  if (to_high < result) result = to_high;
  else if (result < to_low) result = to_low;
  
  return result;
}

  

/*---------------------------End of File-------------------------------------*/
