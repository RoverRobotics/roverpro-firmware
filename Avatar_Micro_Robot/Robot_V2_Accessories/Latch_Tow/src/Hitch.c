/*==============================================================================
File: Hitch.c

Description: This is the overarching file that encapsulates the 
  application-level firmware for the hitch.  Most of the effort lies in
  making the latching robust.
  
Notes:
  - The brown wire for the ESC is GND
  - The ESC must pause for at least ~2seconds after braking before it can 
    change direction
  - The thermistor is currently unused
  - UNLATCHED == MIN_DC == ~800au == (REG_HITCH_OPEN = 1)
  - LATCHED == MAX_DC == ~400au == (REG_HITCH_OPEN = 0)
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#define TEST_HITCH
/*---------------------------Dependencies-------------------------------------*/
#include "./core/StandardHeader.h"
#include "./core/Timers.h"
#include "./core/ADC.h"
#include "./core/PWM.h"
#include "./USB/RXUSBDevice.h"        // for USB firmware/software-shared 
                                      // registers
#include <stdlib.h>                   // for abs() function

/*---------------------------Macros-------------------------------------------*/
// USB
#define HITCH_PRODUCT_ID      0x000F  // product ID for USB

// Filtering
#define ACTUATOR_FILTER_INDEX 0
#define A_ACTUATOR            0.2     // "how hard to filter" sensing input
// Note: current sense resistor filter implemented differently

// PWM Pin
#define LATCH_PWM_PIN         2       // RP2
#define T_PWM                 10      // [ms], period of the PWM signal
#define MAX_DC                0.20    // t_HI = 2ms
#define MAX_ALLOWABLE_DC      0.165
#define NEUTRAL_DC            0.15    // t_HI = 1.5ms
#define MIN_ALLOWABLE_DC      0.12    //0.142
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
#define TRANSITION_TIME       (30*_100ms) // ESC needs ~2a to transition from positive to negative
#define JAM_TIMER             3
#define JAM_CHECK_PERIOD      (20*_100ms)//(5*_100ms)  // can be as high as transition time, needs to be at least as much as it takes to begin moving from ESC
#define UNSAFE_CURRENT_TIMER  4
#define UNSAFE_CURRENT_TIME   (5*_10ms)   // how long it must be high to avoid being counted as a false positive

//#define DUMMY_TIMER           10
//#define DUMMY_TIME            (40*_100ms)

/*---------------------------Type Definitions---------------------------------*/
typedef enum {
  WAITING = 0,
  LATCHING,
  UNLATCHING
} state_t;

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitHitch(void);
static void ProcessHitchIO(void);
static void CalibrateHobbyMotorController(void);
static void Latch(const uint8_t direction);
static bool IsLatched(void);
static bool IsUnlatched(void);
static bool IsLatching(void);
static bool IsJammed(void);
static bool IsCurrentLevelUnsafe(void);
static float IIR(const uint8_t signalIndex, const float alpha, const float x);

/*---------------------------Module Variables---------------------------------*/
static state_t state = WAITING;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_HITCH
#include "./core/ConfigurationBits.h"

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
/*---------------------------Helper Function Definitions----------------------*/
static void InitHitch(void) {
  CONFIGURE_HEARTBEAT_PIN(OUTPUT); HEARTBEAT_PIN = 0;
  
  RXUSBDevice_Init(HITCH_PRODUCT_ID);
  
	// initialize any dependent moduels
	TMRS_Init();
	uint16_t analog_bit_mask = ((1 << TEMP_ANALOG_PIN) |
                              (1 << ACTUATOR_ANALOG_PIN) |
                              (1 << CURRENT_ANALOG_PIN));
  ADC_Init(analog_bit_mask);
  PWM_Init(LATCH_PWM_PIN, 0, T_PWM);
  
  // initialize the latch motor controller
  CONFIG_POWER_BUS(OUTPUT); TURN_POWER_BUS(ON);
  CalibrateHobbyMotorController();
  
	// prime any timers that require it
	TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
	TMRS_StartTimer(TRANSITION_TIMER, 5000);
	//TMRS_StartTimer(DUMMY_TIMER, DUMMY_TIME);
}

static void ProcessHitchIO(void) {
 	// toggle a pin to indicate normal operation
	if (TMRS_IsTimerExpired(HEARTBEAT_TIMER)) {
		HEARTBEAT_PIN ^= 1;
		TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
	}
	
	// turn off the power bus if we are drawing too much current (over-current protection)
	// NB: this must be called outside the state machine
  if (IsCurrentLevelUnsafe()) {
  	// ensure that if this problem persists, the duty cycle is very low
  	// BUG ALERT: ensure the power bus is turned off long enough 
  	// to be able to reset the OTS controller
  	TURN_POWER_BUS(OFF);
  	Delay(5000);
  	asm("reset");
  }

  // only update feedback to software as often as needed
  if (TMRS_IsTimerExpired(UPDATE_TIMER)) {
    REG_HITCH_POSITION = Map(ADC_GetConversion(ACTUATOR_ANALOG_PIN), 
                             V_UNLATCHED - ACTUATOR_HYSTERESIS,
                             V_LATCHED + ACTUATOR_HYSTERESIS, 
                             0, 
                             100);
    TMRS_StartTimer(UPDATE_TIMER, UPDATE_TIME);
  }

  // latch as required
  switch (state) {
    case WAITING:
      if (TMRS_IsTimerExpired(TRANSITION_TIMER)) {
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
        PWM_UpdateDutyCycle(LATCH_PWM_PIN, NEUTRAL_DC);
        TMRS_StartTimer(TRANSITION_TIMER, TRANSITION_TIME);
        state = WAITING;
      }
      break;
    case UNLATCHING:
      if (IsUnlatched() || IsJammed()) {
        PWM_UpdateDutyCycle(LATCH_PWM_PIN, NEUTRAL_DC);
        TMRS_StartTimer(TRANSITION_TIMER, TRANSITION_TIME);
        state = WAITING;
      }
      break;
  	default:
  	  TMRS_StartTimer(HEARTBEAT_TIMER, 65000); // indicate an error
  		break;
  }
  
  /*
  //
  // BASIC TEST
  //
  unsigned int dummy = ADC_GetConversion(ACTUATOR_ANALOG_PIN);
  if (TMRS_IsTimerExpired(TRANSITION_TIMER)) {
		HEARTBEAT_PIN ^= 1;
		
		static unsigned char alternator = 0;
		switch (++alternator % 4) {
      case 0:
        PWM_UpdateDutyCycle(LATCH_PWM_PIN, MAX_ALLOWABLE_DC);
        break;
      case 1:
        PWM_UpdateDutyCycle(LATCH_PWM_PIN, NEUTRAL_DC);
		    break;
		  case 2:
		    PWM_UpdateDutyCycle(LATCH_PWM_PIN, MIN_ALLOWABLE_DC);
		    break;
		  case 3:
		    PWM_UpdateDutyCycle(LATCH_PWM_PIN, NEUTRAL_DC);
		    break;
    }
		
		TMRS_StartTimer(TRANSITION_TIMER, 500);
	}
  */
  
  /*
  //
  // FUNCTIONAL-WITHOUT-USB TEST
  //
  static uint8_t dummyDesiredPosition = 0;
  if (TMRS_IsTimerExpired(DUMMY_TIMER)) {
    TMRS_StartTimer(DUMMY_TIMER, DUMMY_TIME);
    dummyDesiredPosition ^= 1;
  }
  
  // latch as required
  switch (state) {
    case WAITING:
      if (TMRS_IsTimerExpired(TRANSITION_TIMER)) {
        if (dummyDesiredPosition && IsLatched()) {
      	  Latch(OFF);
      	  state = UNLATCHING;
      	} else if (!(dummyDesiredPosition) && IsUnlatched()) {
      	  Latch(ON);
      	  state = LATCHING;
      	} else if (IsLatching()) {
      	  // latch as software desires
      	  if (dummyDesiredPosition) {
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
        PWM_UpdateDutyCycle(LATCH_PWM_PIN, NEUTRAL_DC);
        TMRS_StartTimer(TRANSITION_TIMER, TRANSITION_TIME);
        state = WAITING;
      }
      break;
    case UNLATCHING:
      if (IsUnlatched() || IsJammed()) {
        PWM_UpdateDutyCycle(LATCH_PWM_PIN, NEUTRAL_DC);
        TMRS_StartTimer(TRANSITION_TIMER, TRANSITION_TIME);
        state = WAITING;
      }
      break;
  	default:
  	  TMRS_StartTimer(HEARTBEAT_TIMER, 65000); // indicate an error
  		break;
  }
  */
  
  
  RXUSBDevice_ProcessMessage();
}


/******************************************************************************
Function: CalibrateHobbyMotorController()
Description: Performs the calibration sequence as defined by the Losi8750KV 
 Hobby sensorless BLDC Motor and controller.
Notes:
  - http://losi.com/Products/Default.aspx?ProdID=LOSB9594
  - WARNING: must pause at nuetral for some period of time before changing 
    direction for hobby controller to recognize the change
*******************************************************************************/
static void CalibrateHobbyMotorController(void) {
  // wait in nuetral long enough for the controller (beeps when ready)
  PWM_UpdateDutyCycle(LATCH_PWM_PIN, NEUTRAL_DC);
}
  
/*******************************************************************************
Function: Latch()
Notes:
  - use the linear potentiometer to realize when we've reached our destination
  - catch when the mechanism has jammed and do not allow it to continue to
    move in that direction until it first moves in the opposite direction
  - avoid changing direction faster than the Hobby controller can handle 
    (must pause for ~1s before can change direction)
*******************************************************************************/
static void Latch(const uint8_t direction) {
  TMRS_StartTimer(TRANSITION_TIMER, TRANSITION_TIME);
  TMRS_StartTimer(JAM_TIMER, JAM_CHECK_PERIOD);
  
  if (direction == ON) {
    PWM_UpdateDutyCycle(LATCH_PWM_PIN, MAX_ALLOWABLE_DC);
  } else if (direction == OFF) {
    PWM_UpdateDutyCycle(LATCH_PWM_PIN, MIN_ALLOWABLE_DC);
  }
}


static bool IsJammed(void) {
  // check whether the potentiometer is NOT changing as quickly as it should
  if (TMRS_IsTimerExpired(JAM_TIMER)) {
    TMRS_StartTimer(JAM_TIMER, JAM_CHECK_PERIOD);
    static uint16_t last_position = 0;  // OK because we never reach 0 on the pot
    uint16_t current_position = ADC_GetConversion(ACTUATOR_ANALOG_PIN);
  
    uint16_t delta_position = abs((signed int)current_position - (signed int)last_position);
    last_position = current_position;
    return (delta_position < EXPECTED_DELTA_POT);
  }
  
  return 0;
}

/*******************************************************************************
Description: Determines the state of the latch from linear potentiometer 
  feedback.  This function additionally adds hysteresis to the pre-defined
  limits to take into tolerance in the locking mechanism sliding back.
*******************************************************************************/
static bool IsLatched(void) {
  return (IIR(ACTUATOR_FILTER_INDEX, A_ACTUATOR, ADC_GetConversion(ACTUATOR_ANALOG_PIN)) <= V_LATCHED);
}

static bool IsUnlatched(void) {
  return (V_UNLATCHED <= IIR(ACTUATOR_FILTER_INDEX, A_ACTUATOR, ADC_GetConversion(ACTUATOR_ANALOG_PIN)));
}

static bool IsLatching(void) {
  uint16_t position = IIR(ACTUATOR_FILTER_INDEX, A_ACTUATOR, ADC_GetConversion(ACTUATOR_ANALOG_PIN));
  return ((V_LATCHED < position) && (position < V_UNLATCHED));
}


/*******************************************************************************
Description: Returns whether the current is at an unsafe level, rejecting any
  spurious spikes by ensuring we see a consistent unsafe current level
  for a nominal duration of time.  This implementation is similar to 
  the concept of a watchdog timer and MUST BE CALLED IN THE MAIN LOOP
*******************************************************************************/
static bool IsCurrentLevelUnsafe(void) {
  if (ADC_GetConversion(CURRENT_ANALOG_PIN) < UNSAFE_CURRENT_LEVEL) {
    TMRS_StartTimer(UNSAFE_CURRENT_TIMER, UNSAFE_CURRENT_TIME);
  }
  
  return (TMRS_IsTimerExpired(UNSAFE_CURRENT_TIMER));
}


/*******************************************************************************
Function: IIR
Description: Passes the input through an Infinite Impulse Response Filter
  characterized by the parameter alpha.
Paramters:
	unsigned char signalIndex,the index of the signal
	float x,                  the current sample to be filtered
	float alpha,              knob on how much to filter, [0,1)
*******************************************************************************/
static float IIR(const uint8_t signalIndex, const float alpha, const float x) {
  // first-order infinite impulse response (IIR) filter
	// (aka a 'leaky integrator')
  // http://dsp.stackexchange.com/questions/1004/low-pass-filter-in-non-ee-software-api-contexts
  static float yLasts[sizeof(uint8_t)] = {0};
  float y = alpha * yLasts[signalIndex] + (1.0 - alpha) * x;
  yLasts[signalIndex] = y;
  
  return y;
}  
