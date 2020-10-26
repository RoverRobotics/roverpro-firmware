/*==============================================================================
File: drivetrain.c

Notes:
  - another option to consider is using the _PORTDN macros instead of _LATDN
    so that we can read back their state (can get rid of a little if-else logic)
  - return a pointer to a "motor object" and internally keep track of speeds?
==============================================================================*/
//#define TEST_DRIVETRAIN
//---------------------------Dependencies---------------------------------------
#include "./drivetrain.h"
#include "./core/InputCapture.h"// for motor shaft speed feedback
#include "./core/PWM.h"         // for PWM interface
#include <math.h>               // for fabsf()

//---------------------------Macros---------------------------------------------
// motor PWM pin assignments
#define T_PWM               40//33  // [us], 1/33us => 30kHz
#define M1_PWM_PIN    	    24  //_RP24R
#define M2_PWM_PIN 	        2   //_RP2R
#define M3_PWM_PIN          25  //_RP25R

#define SLOW_DECAY_MODE     1   // lower ripple current, but
                                // slower dynamic response
                                // (see p.10 of A3931 datasheet)
// motor driver inputs
#define M1_DIR_EN(a)		    (_TRISD6 = !(a))
#define M1_BRAKE_EN(a)		  (_TRISD7 = !(a))
#define M1_MODE_EN(a)	  	  (_TRISD9 = !(a))
#define M1_COAST_EN(a)		  (_TRISD10 = !(a))
#define M1_DIR 		          _LATD6
#define M1_TURN_BRAKE(a) 	  (_LATD7 = !(a))   // active-low
#define M1_SET_MODE_TO(a) 	(_LATD9 = (a))
#define M1_TURN_COAST(a) 	  (_LATD10 = !(a))  // active-low
#define M1_TACHO_RPN        12                // RP12         

#define M2_DIR_EN(a)		    (_TRISB4 = !(a))
#define M2_BRAKE_EN(a)		  (_TRISB5 = !(a))
#define M2_MODE_EN(a)		    (_TRISG9 = !(a))
#define M2_COAST_EN(a)		  (_TRISG8 = !(a))
#define M2_DIR 		          _LATB4
#define M2_TURN_BRAKE(a)    (_LATB5 = !(a))
#define M2_SET_MODE_TO(a)   (_LATG9 = (a))
#define M2_TURN_COAST(a)    (_LATG8 = !(a))
#define M2_TACHO_RPN        16         

#define M3_DIR_EN(a)		    (_TRISE3 = !(a))
#define M3_BRAKE_EN(a)		  (_TRISF0 = !(a))
#define M3_MODE_EN(a)		    (_TRISE4 = !(a))
#define M3_COAST_EN(a)		  (_TRISF1 = !(a))
#define M3_DIR 		          _LATE3
#define M3_TURN_BRAKE(a) 	  (_LATF0 = (!a))
#define M3_SET_MODE_TO(a)   (_LATE4 = (a))
#define M3_TURN_COAST(a) 	  (_LATF1 = (!a))
#define M3_TACHO_RPN        20

// motor driver outputs
#define M1_DIRO		          _RD0  // whether the motor is moving 'forward'
#define M1_FF1 		          _RC14
#define M1_FF2  	          _RC13

#define M2_DIRO		          _RE5
#define M2_FF1 		          _RG6
#define M2_FF2  	          _RG7

#define M3_DIRO             _RE0
#define M3_FF1              _RE2
#define M3_FF2              _RE1

// GR = GR_motor_gearhead * GR_spurgear_reduction
//    = (1/17) * (1/4) => 0.01470588235 for high-speed version
//    = (1/24) * (1/4) => 0.01041666666 for high-torque version

// #define D_TRACK_WHEEL ~150mm
//#define MPS_PER_EVENT       0.64// (meters/second)/event, for high-torque version
// (1 events/ms)*(150mm)*1/24*1/4) ~=> 1.5m/s, (150*1/24*1/4)^-1=>0.64

//---------------------------Helper Function Prototypes-------------------------
static void InitPins(kMotor motor);

//---------------------------Module Variables-----------------------------------

//---------------------------Test Harness---------------------------------------
#ifdef TEST_DRIVETRAIN
#include "./core/ConfigurationBits.h"
#include "./core/Timers.h"

// power management pin(s) and threshold(s)
#define SIDEA_CONNECT_EN(a)	(_TRISD3 = !(a))
#define SIDEB_CONNECT_EN(a)	(_TRISD2 = !(a))
#define SIDEA_CONNECT 	    _LATD3
#define SIDEB_CONNECT 	    _LATD2

#define TEST_TIMER  0
#define TEST_TIME   1000  // [ms]

// debugging variables to watch
extern float left_speed = 0;
extern float right_speed = 0;
extern float flipper_speed = 0;

static void RunHybridPowerup(void);

int main(void) {
  _SWDTEN = 1; // enable the watchdog timer
  SIDEA_CONNECT_EN(1); SIDEB_CONNECT_EN(1);
  RunHybridPowerup();
  DT_Init(kMotorLeft);
  DT_Init(kMotorRight);
  //DT_Init(kMotorFlipper); // WATCH OUT FOR JAMMING BATTERY DOOR!
  TMRS_Init(); TMRS_StartTimer(TEST_TIMER, TEST_TIME);
  while (1) {
    ClrWdt();
    DT_Run();
    
    // cycle speeds from moderately-fast reverse to moderately-fast forward
    if (TMRS_IsTimerExpired(TEST_TIMER)) {
      TMRS_StartTimer(TEST_TIMER, TEST_TIME);
      static float j = 0.0;
      j += 0.1;
      if (0.5 < j) j = -0.5;
      DT_set_speed(kMotorLeft, j);
      DT_set_speed(kMotorRight, j);
      left_speed = DT_speed(kMotorLeft);
      right_speed = DT_speed(kMotorRight);
      flipper_speed = DT_speed(kMotorFlipper);
    }
  }
  
  return 0;
}


// Description: Unfortunate hack to solve issues with the initial 
//   current spike over-currenting the battery. 
//   (taken from powerboard.c to complete the unit test)
static void RunHybridPowerup(void) {
  uint16_t i, j, k, k0, l;
  k = 2000;
  k0 = 2000;
  l = 0;
	for (i = 0; i < 200; i++) {
    for (l = 0; l < 3; l++) {
    	SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
      for (j = 0; j < k; j++) Nop();
    	SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
      break;
      if (20 < i) break;
      ClrWdt(); Delay(10);
    }
    ClrWdt(); Delay(40); ClrWdt();
    k = k0 + i * i / 4;
    if (20000 < k) k = 20000;
	}
	SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
}
#endif

//---------------------------Public Function Implementations--------------------
void DT_Init(const kMotor motor) {
  InitPins(motor);
  switch (motor) {
    case kMotorLeft:
      M1_DIR = CCW;
      M1_TURN_COAST(OFF);
      IC_Init(kIC01, M1_TACHO_RPN, 5);
      PWM_Init(kPWM01, M1_PWM_PIN, T_PWM);
      break;
    case kMotorRight:
      M2_DIR = CW;
      M2_TURN_COAST(OFF);
      IC_Init(kIC02, M2_TACHO_RPN, 5);
      PWM_Init(kPWM02, M2_PWM_PIN, T_PWM);
      break;
    case kMotorFlipper:
      M3_DIR = CW;
      M3_TURN_COAST(OFF);
      PWM_Init(kPWM03, M3_PWM_PIN, T_PWM);
      break;
  }
}


void DT_Run(void) {
  IC_UpdatePeriods();
}


void DT_set_speed(const kMotor motor, const float speed) {
  uint8_t current_direction;
  
  switch (motor) {
    case kMotorLeft:
    {
      // change direction if needed
      static uint8_t last_direction = CCW;
      if (0 < speed) current_direction = CCW;
      else current_direction = CW;
      if (current_direction != last_direction) {
        M1_DIR = current_direction;
        last_direction = current_direction;
      }
      // update the magnitude
      PWM_UpdateDutyCycle(kPWM01, fabs(speed));
      break;
    } 
    case kMotorRight:
    {
      static uint8_t last_direction = CW;
      if (0 < speed) current_direction = CW;
      else current_direction = CCW;
      if (current_direction != last_direction) {
        M2_DIR = current_direction;
        last_direction = current_direction;
      }
      PWM_UpdateDutyCycle(kPWM02, fabsf(speed));
      break;
    } 
    case kMotorFlipper:
    {
      static uint8_t last_direction = CW;
      if (0 < speed) current_direction = CCW;
      else current_direction = CW;
      if (current_direction != last_direction) {
        M3_DIR = current_direction;
        last_direction = current_direction;
      }
      PWM_UpdateDutyCycle(kPWM03, fabsf(speed));
      break;
    }
  }
}


float DT_speed(const kMotor motor) {
  #define HZ_16US 100000.0
  
  float period = 0;
  switch (motor) {
    case kMotorLeft: {
      period = IC_period(kIC01);
      if (period != 0) {
        if (M1_DIRO) return -(HZ_16US / period);
        else return (HZ_16US / period);
      }
      break;
    }
    case kMotorRight: {
      period = IC_period(kIC02);
      if (period != 0) {
        if (M2_DIRO) return (HZ_16US / period);
        else return -(HZ_16US / period);
      }
      break;
    }
    case kMotorFlipper: {
      period = IC_period(kIC03);
      if (period != 0) {
        if (M3_DIRO) return (HZ_16US / period);
        else return -(HZ_16US / period);
      }
      break;
    }
  }
  return 0;
}

//---------------------------Helper Function Implementations--------------------
// Function: InitPins
// Description: Configures and initializes the pins associated with the 
//   respective motor driver.
static void InitPins(const kMotor motor) {
  switch (motor) {
    case kMotorLeft:
      M1_DIR_EN(1);
    	M1_BRAKE_EN(1);
    	M1_MODE_EN(1);
    	M1_COAST_EN(1);
    	M1_DIR = CCW;
    	M1_TURN_BRAKE(OFF);
    	M1_TURN_COAST(ON);
    	M1_SET_MODE_TO(SLOW_DECAY_MODE);
      break;
    case kMotorRight:
    	M2_DIR_EN(1);
    	M2_BRAKE_EN(1);
    	M2_MODE_EN(1);
    	M2_COAST_EN(1);
      M2_DIR = CCW;
      M2_TURN_BRAKE(OFF);
      M2_TURN_COAST(ON);
    	M2_SET_MODE_TO(SLOW_DECAY_MODE);
    case kMotorFlipper:
    	M3_DIR_EN(1);
    	M3_BRAKE_EN(1);
    	M3_MODE_EN(1);
    	M3_COAST_EN(1);
    	M3_DIR = CCW;
    	M3_TURN_BRAKE(OFF);
    	M3_TURN_COAST(ON);
      M3_SET_MODE_TO(SLOW_DECAY_MODE);
      break;
  }
}