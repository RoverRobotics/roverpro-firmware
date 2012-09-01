/*=============================================================================
File: Hitch.c
=============================================================================*/
//#define TEST_HITCH
/*---------------------------Dependencies------------------------------------*/
#include "./Hitch.h"
#include "./Timers.h"
#include "./ADC.h"
#include "./PWM.h"
#include "./stdhdr.h"         // to get access to our custom registers
#include "./StandardHeader.h" // for pic header file, Map() function, etc
#include <stdbool.h>          // for bool data type definition

/*---------------------------Wiring Macros-----------------------------------*/
// PWM Pin
#define LATCH_PWM_PIN         2       // RP2
#define T_PWM                 10      // [ms], period of the PWM signal
#define MAX_DC                0.20    // t_HI = 2ms
#define MAX_ALLOWABLE_DC      0.158
#define NEUTRAL_DC            0.15    // t_HI = 1.5ms
#define MIN_ALLOWABLE_DC      0.142
#define MIN_DC                0.10    // t_HI = 1ms

// Power Bus
#define CONFIG_POWER_BUS(a)   (_TRISD9 = (a))
#define TURN_POWER_BUS(a)     (_LATD9 = (a)); Nop(); Nop()

// Actuator Position Sensing
#define ACTUATOR_ANALOG_PIN   4
#define V_UNLATCHED           800     // [au], (au = 'arbitrary units')
#define V_LATCHED             400     // [au]
#define ACTUATOR_HYSTERESIS   50      // [au], (0.10 / 3.30) * 1023 ~= 31
#define EXPECTED_DELTA_POT    10      // [au], the amount by which we expect to AT LEAST move each cycle

// Temperature Sensing
#define TEMP_ANALOG_PIN       8

// Power Bus Current Sensing
#define CURRENT_ANALOG_PIN    15
#define UNSAFE_CURRENT_LEVEL  310     // 0.01S*(2A*0.05Ohm)*1k ~= 0.75V, (0.75 / 3.30) * 1023=
//230     // 0.01S*(1.5A*0.05Ohm)*1k ~= 0.75V, (0.75 / 3.30) * 1023=

// Heartbeat Indicator
#define CONFIGURE_HEARTBEAT_PIN(a)   (_TRISE5 = (a))
#define HEARTBEAT_PIN         (_RE5)

// Times and Timers
#define _10ms                 10
#define _100ms                100
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        (10*_100ms)
#define UPDATE_TIMER          1
#define UPDATE_TIME           _100ms  // update software-register interfaces every ~10Hz
#define TRANSITION_TIMER      2
#define TRANSITION_TIME       (10*_100ms)
#define JAM_TIMER             3
#define JAM_CHECK_PERIOD      (5*_100ms)
#define UNSAFE_CURRENT_TIMER  4
#define UNSAFE_CURRENT_TIME   (5*_10ms)

/*---------------------------Type Definitions--------------------------------*/
typedef enum {
  WAITING = 0,
  LATCHING,
  UNLATCHING
} state_t;

/*---------------------------Helper Function Prototypes----------------------*/
static void CalibrateHobbyMotorController(void);
static void Latch(unsigned char direction);
static bool IsLatched(void);
static bool IsUnlatched(void);
static bool IsLatching(void);
static bool IsJammed(void);
static bool IsCurrentLevelUnsafe(void);

/*---------------------------Module Variables--------------------------------*/
static state_t state = WAITING;

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_HITCH
#include "./ConfigurationBits.h"

int main(void) {
	InitHitch();

  _SWDTEN = 1; // enable the watchdog timer
  while (1) {
    ProcessHitchIO();
    ClrWdt(); // clear the watchdog timer 
	}
	
	return 0;
}

#endif
/*---------------------------End Test Harness--------------------------------*/
/*---------------------------Public Function Definitions---------------------*/
void InitHitch(void) {
  CONFIGURE_HEARTBEAT_PIN(OUTPUT); HEARTBEAT_PIN = 0;

	// initialize any dependent moduels
	InitTimers();
	unsigned int analog_bit_mask = ( (1 << TEMP_ANALOG_PIN)     |
                                   (1 << ACTUATOR_ANALOG_PIN) |
                                   (1 << CURRENT_ANALOG_PIN));
  InitADC(analog_bit_mask);
  InitPWM(LATCH_PWM_PIN, T_PWM);
  
  // initialize the latch motor controller
  CONFIG_POWER_BUS(OUTPUT); TURN_POWER_BUS(ON);
  CalibrateHobbyMotorController();
  
	// prime any timers that require it
	StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
	StartTimer(TRANSITION_TIMER, 5000);
}

void ProcessHitchIO(void) {
 	// toggle a pin to indicate normal operation
	if (IsTimerExpired(HEARTBEAT_TIMER)) {
		HEARTBEAT_PIN ^= 1;
		StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
	}
	
	/*
	// turn off the power bus if we are drawing too much current (over-current protection)
  if (IsCurrentLevelUnsafe()) {
  	// ensure that if this problem persists, the duty cycle is very low
  	// BUG ALERT: ensure the power bus is turned off long enough 
  	// to be able to reset the OTS controller
  	TURN_POWER_BUS(OFF);
  	Delay(_100ms);
  	Reset();
  }
  */
  
  /*
  // only update feedback to software as often as needed
  if (IsTimerExpired(UPDATE_TIMER)) {
    REG_HITCH_POSITION = Map(GetADC(ACTUATOR_ANALOG_PIN), 
                             V_UNLATCHED - ACTUATOR_HYSTERESIS,
                             V_LATCHED + ACTUATOR_HYSTERESIS, 0, 100);
    StartTimer(UPDATE_TIMER, UPDATE_TIME);
  }
  
  // latch as required
  switch (state) {
    case WAITING:
      if (IsTimerExpired(TRANSITION_TIMER)) {
        if (REG_HITCH_OPEN && IsLatched()) {
      	  Latch(OFF);
      	  state = UNLATCHING;
      	} else if (!(REG_HITCH_OPEN) && IsUnlatched()) {
      	  Latch(ON);
      	  state = LATCHING;
      	} else if (IsLatching()) {
      	  // latch as software desires
      	  if (REG_HITCH_OPEN) {
        	  Latch(OFF);
      	    state = UNLATCHING;
          } else {
        	  Latch(ON);
      	    state = LATCHING;
      	  }
        }
      }
  	  break;
  	case LATCHING:
  	  if (IsLatched() || IsJammed()) {
        UpdateDutyCycle(NEUTRAL_DC);
        StartTimer(TRANSITION_TIMER, TRANSITION_TIME);
        state = WAITING;
      }
      break;
    case UNLATCHING:
      if (IsUnlatched() || IsJammed()) {
        UpdateDutyCycle(NEUTRAL_DC);
        StartTimer(TRANSITION_TIMER, TRANSITION_TIME);
        state = WAITING;
      }
      break;
  	default:
  	  StartTimer(HEARTBEAT_TIMER, 65000); // indicate an error
  		break;
  }
  */


  // basic test
  unsigned int dummy = GetADC(ACTUATOR_ANALOG_PIN);
  if (IsTimerExpired(TRANSITION_TIMER)) {
		HEARTBEAT_PIN ^= 1;
		
		static unsigned char alternator = 0;
		switch (++alternator % 4) {
      case 0:
        UpdateDutyCycle(MAX_ALLOWABLE_DC);
        break;
      case 1:
        UpdateDutyCycle(NEUTRAL_DC);
		    break;
		  case 2:
		    UpdateDutyCycle(MIN_ALLOWABLE_DC);
		    break;
		  case 3:
		    UpdateDutyCycle(NEUTRAL_DC);
		    break;
    }
		
		StartTimer(TRANSITION_TIMER, 2000);
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
  UpdateDutyCycle(NEUTRAL_DC);
}
  
/*****************************************************************************
Function: Latch()
Notes:
  - use the linear potentiometer to realize when we've reached our destination
  - catch when the mechanism has jammed and do not allow it to continue to
    move in that direction until it first moves in the opposite direction
  - avoid changing direction faster than the Hobby controller can handle 
    (must pause for ~1s before can change direction)
******************************************************************************/
static void Latch(unsigned char direction) {
  StartTimer(TRANSITION_TIMER, TRANSITION_TIME);
  StartTimer(JAM_TIMER, JAM_CHECK_PERIOD);
  
  if (direction == ON) UpdateDutyCycle(MAX_ALLOWABLE_DC);
  else if (direction == OFF) UpdateDutyCycle(MIN_ALLOWABLE_DC);
}


static bool IsJammed(void) {
  // check whether the potentiometer is NOT changing as quickly as it should
  if (IsTimerExpired(JAM_TIMER)) {
    StartTimer(JAM_TIMER, JAM_CHECK_PERIOD);
    static unsigned int last_position = 0;  // OK because we never reach 0 on the pot
    unsigned int current_position = GetADC(ACTUATOR_ANALOG_PIN);
  
    unsigned int delta_position = abs((signed int)current_position - (signed int)last_position);
    last_position = current_position;
  
    return (delta_position < EXPECTED_DELTA_POT);
  }
  
  return 0;  
}

/*
Description: Determines the state of the latch from linear potentiometer 
  feedback.  This function additionally adds hysteresis to the pre-defined
  limits to take into tolerance in the locking mechanism sliding back.
*/
static bool IsLatched(void) {
  return (GetADC(ACTUATOR_ANALOG_PIN) <= V_LATCHED);
}

static bool IsUnlatched(void) {
  return (V_UNLATCHED <= GetADC(ACTUATOR_ANALOG_PIN));
}

static bool IsLatching(void) {
  unsigned int position = GetADC(ACTUATOR_ANALOG_PIN);
  return ((V_LATCHED < position) && (position < V_UNLATCHED));
}

/*
Description: Returns whether the current is at an unsafe level, rejecting any
  spurious spikes by ensuring we see a consistent unsafe current level
  for a nominal duration of time.  This implementation is similar to 
  the concept of a watchdog timer.
*/
static bool IsCurrentLevelUnsafe(void) {
  if (UNSAFE_CURRENT_LEVEL < GetADC(CURRENT_ANALOG_PIN)) {
    StartTimer(UNSAFE_CURRENT_TIMER, UNSAFE_CURRENT_TIME);
  }
  
  return (IsTimerExpired(UNSAFE_CURRENT_TIMER));
}
  

/*---------------------------End of File-------------------------------------*/
