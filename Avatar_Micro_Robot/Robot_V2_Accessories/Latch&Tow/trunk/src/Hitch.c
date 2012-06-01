/*=============================================================================
File: Hitch.c
=============================================================================*/
//#define TEST_HITCH
/*---------------------------Dependencies------------------------------------*/
#include "./Hitch.h"
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

//---PWM Pin
#define LATCH_PWM_PIN         2       // RP2
#define T_PWM                 10      // [ms], period of the PWM signal
#define MAX_DC                20      // T = 2ms
#define MAX_ALLOWABLE_DC      16      // T = 1.6
#define NEUTRAL_DC            15      // T = 1.5
#define MIN_ALLOWABLE_DC      14      // T = 1.4
#define MIN_DC                10      // T = 1ms
//---Power Bus
#define CONFIG_POWER_BUS(a)   (_TRISD9 = (a))
#define TURN_POWER_BUS(a)     (_LATD9 = (a))

// Actuator Position Sensing
#define ACTUATOR_ANALOG_PIN   4
#define V_UNLATCHED           976     // [au], (3.15 / 3.30) * 1023=
#define V_LATCHED             124     // [au], (0.40 / 3.30) * 1023=
#define ACTUATOR_HYSTERESIS   50      // [au], (0.10 / 3.30) * 1023 ~= 31

// Temperature Sensing
#define TEMP_ANALOG_PIN       8

// Power Bus Current Sensing
#define CURRENT_ANALOG_PIN    15
#define UNSAFE_CURRENT_LEVEL  700     // 0.01S*(5A*0.05Ohm)*1k ~= 2.5V, (2.5 / 3.30) * 1023=

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
//#define TEST_TIMER            4
//#define TEST_TIME             (10*_100ms)
#define OVERCURRENT_TIMER     5
#define OVERCURRENT_TIME      (50*_100ms)
#define LATCH_TIMER           6
#define TYPICAL_LATCH_TIME    (10*_100ms)
#define TRANSITION_TIMER      7
#define TRANSITION_TIME       (12*_100ms) 
//#define DELTA_DC              1
//#define DELTA_LATCH_TIME      (_100ms)

/*---------------------------Type Definitions--------------------------------*/
typedef enum {
  WAITING = 0,
  LATCHING,
  COOLING_DOWN
} state_t;  

/*---------------------------Helper Function Prototypes----------------------*/
static void CalibrateHobbyMotorController(void);
static void Latch(unsigned char direction);
static unsigned char IsLatched(void);
static unsigned char IsUnlatched(void);
static unsigned char IsLatching(void);
static unsigned int Map(int value, int from_low, int from_high, 
                        int to_low, int to_high);
static unsigned char IsJammed(void);
static unsigned char FinishedLatching(void);
static unsigned char FinishedUnlatching(void);

/*---------------------------Module Variables--------------------------------*/
static int V_unlatched = V_UNLATCHED;           
static int V_latched = 124;// V_LATCHED;
static state_t state = WAITING;

static unsigned char open_hitch = 0;  // TODO: delete after testing

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_HITCH
#include "./ConfigurationBits.h"

int main(void) {
	InitHitch();

  while (1) {
    ProcessHitchIO();
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
  CONFIG_POWER_BUS(OUTPUT); TURN_POWER_BUS(ON); Nop(); Nop();
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
	
	// turn off the power bus if we are drawing too much current (over-current protection)
	if (UNSAFE_CURRENT_LEVEL < GetADC(CURRENT_ANALOG_PIN)) {
    TURN_POWER_BUS(OFF);
    StartTimer(OVERCURRENT_TIMER, OVERCURRENT_TIME);
    state = COOLING_DOWN;
  }
  
  // only update feedback to software as often as needed
  if (IsTimerExpired(UPDATE_TIMER)) {
    REG_HITCH_POSITION = Map(GetADC(ACTUATOR_ANALOG_PIN), V_UNLATCHED, V_LATCHED, 0, 100);
    StartTimer(UPDATE_TIMER, UPDATE_TIME);
  }
  
  switch (state) {
    case WAITING:
      if (IsLatched() && open_hitch && IsTimerExpired(TRANSITION_TIMER)) {
    	  Latch(OFF);
    	  state = LATCHING;
    	} else if (IsUnlatched() && (!open_hitch) && IsTimerExpired(TRANSITION_TIMER)) {	  
    	  Latch(ON);
    	  state = LATCHING;
    	} else if (IsLatching() && IsTimerExpired(TRANSITION_TIMER)) {
    	  Latch(open_hitch); // latch as software desires it
    	  state = LATCHING;
      }
  	  break;
  	case LATCHING:
  	  if (FinishedUnlatching()) {
        UpdateDutyCycle(NEUTRAL_DC);
        state = WAITING;
      } else if (FinishedLatching()) {
    	  UpdateDutyCycle(NEUTRAL_DC);
        state = WAITING;
      } else if (IsJammed()) {
        UpdateDutyCycle(NEUTRAL_DC);
        state = WAITING;
      }
      break;
    case COOLING_DOWN:
      if (IsTimerExpired(OVERCURRENT_TIMER)) {
        TURN_POWER_BUS(ON);
        state = WAITING;
      }
      break;
  	default:
  	  StartTimer(HEARTBEAT_TIMER, 65000); // indicate an error
  		break;
  }
  
  /*
  // basic test
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
	*/
  
  
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
  //Pause(5000);
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
  StartTimer(LATCH_TIMER, TYPICAL_LATCH_TIME);
  
  if (direction == ON) UpdateDutyCycle(MAX_ALLOWABLE_DC);
  else if (direction == OFF) UpdateDutyCycle(MIN_ALLOWABLE_DC);
}

static unsigned char IsJammed(void) {
  return (IsLatching() && IsTimerExpired(LATCH_TIMER));
}

/*
Description: Determines the state of the latch from linear potentiometer 
  feedback.  This function additionally adds hysteresis to the pre-defined
  limits to take into tolerance in the locking mechanism sliding back.
*/
static unsigned char IsLatched(void) {
  if (GetADC(ACTUATOR_ANALOG_PIN) < V_latched) {
    V_latched += ACTUATOR_HYSTERESIS;
    V_unlatched = V_UNLATCHED; // make sure to restore the counterpart limit!
    return 1;
  }
  
  return 0;
}

static unsigned char IsUnlatched(void) {
  if (V_unlatched < GetADC(ACTUATOR_ANALOG_PIN)) {
    V_unlatched -= ACTUATOR_HYSTERESIS;
    V_latched = V_LATCHED;
    return 1;
  }
  
  return 0;
}

static unsigned char IsLatching(void) {
  unsigned int position = GetADC(ACTUATOR_ANALOG_PIN);
  return ((V_latched < position) && (position < V_unlatched));
}

static unsigned char FinishedLatching(void) {
  return (IsLatched() && !open_hitch);
}

static unsigned char FinishedUnlatching(void) {
  return (IsUnlatched() && open_hitch);
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
TODO: put this function in StandardHeader.h/.c
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
