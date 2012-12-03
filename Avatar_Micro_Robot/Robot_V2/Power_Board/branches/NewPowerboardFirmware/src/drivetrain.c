/*==============================================================================
File: drivetrain.c
==============================================================================*/
#define TEST_DRIVETRAIN
/*---------------------------Dependencies-------------------------------------*/
#include "./drivetrain.h"
#include "./core/InputCapture.h"// for motor shaft speed feedback
#include "./core/PWM.h"         // for PWM generators
#include <stdlib.h>             // for abs()

/*---------------------------Macros-------------------------------------------*/
// motor PWM pin assignments
#define T_PWM               1   // [ms], 1/1ms => 1kHz
#define M1_PWM_PIN    	    24  //_RP24R
#define M2_PWM_PIN 	        2   //_RP2R
#define M3_PWM_PIN          25  //_RP25R

#define SLOW_DECAY_MODE     1   // lower ripple current, but
                                // slower dynamic response
                                // (see p.10 of A3931 datasheet)

//---motor driver inputs
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

//---motor driver outputs
#define M1_DIRO		          _RD0
#define M1_FF1 		          _RC14
#define M1_FF2  	          _RC13

#define M2_DIRO		          _RE5  // whether the motor is moving clockwise or counter-clockwise
#define M2_FF1 		          _RG6
#define M2_FF2  	          _RG7

#define M3_DIRO             _RE0
#define M3_FF1              _RE2
#define M3_FF2              _RE1

/*---------------------------Helper Function Prototypes-----------------------*/
void InitPins(kMotor motor);

/*---------------------------Module Variables---------------------------------*/

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_DRIVETRAIN
#include "./core/ConfigurationBits.h"
#include "./core/Timers.h"

#define TEST_TIMER  0
#define TEST_TIME   1000  // [ms]

// debugging variables to watch
extern float left_speed = 0;
extern float right_speed = 0;
extern float flipper_speed = 0;

int main(void) {
  //_SWDTEN = 1; // enable the watchdog timer
  DT_Init(kMotorLeft);
  DT_Init(kMotorRight);
  DT_Init(kMotorFlipper);
  TMRS_Init(); TMRS_StartTimer(TEST_TIMER, TEST_TIME);
  while (1) {
    //ClrWdt();
    DT_Run();
    
    // cycle speeds from moderately-fast reverse to moderately-fast forward
    if (TMRS_IsTimerExpired(TEST_TIMER)) {
      static float j = 0.0;
      j += 0.1;
      if (0.5 < j) j = -0.5;
      DT_set_speed(kMotorLeft, j);
      DT_set_speed(kMotorRight, j);
    }
  }
  
  return 0;
}
#endif

/*---------------------------Public Function Implementations------------------*/
void DT_Init(const kMotor motor) {
  InitPins(motor);
  switch (motor) {
    case kMotorLeft:
      M1_DIR = CCW; Nop(); Nop();
      M1_TURN_COAST(OFF); Nop(); Nop();
      IC_Init(kIC01, M1_TACHO_RPN, 10);
      PWM_Init(kPWM01, M1_PWM_PIN, T_PWM);
      break;
    case kMotorRight:
      M2_DIR = CW; Nop();
      M2_TURN_COAST(OFF); Nop();
      IC_Init(kIC02, M2_TACHO_RPN, 10);
      PWM_Init(kPWM02, M2_PWM_PIN, T_PWM);
      break;
    case kMotorFlipper:
      M3_DIR = CCW; Nop();
      M3_TURN_COAST(OFF); Nop();
      PWM_Init(kPWM03, M3_PWM_PIN, T_PWM);
      break;
  }
}


void DT_Run(void) {
  IC_UpdatePeriods();
}


void DT_set_speed(const kMotor motor, float speed) {
  // this would be better managed as an object but too much right now
  switch (motor) {
    case kMotorLeft: {
      // change direction if needed
      static uint8_t last_direction = CCW;
      uint8_t current_direction;
      if (0 < speed) current_direction = CCW;
      else current_direction = CW;
      if (current_direction != last_direction) M1_DIR = current_direction;
      
      // update the magnitude
      PWM_UpdateDutyCycle(kPWM01, abs(speed));
      break;
    } 
    case kMotorRight: {
      static uint8_t last_direction = CW;
      uint8_t current_direction;
      if (0 < speed) current_direction = CW;
      else current_direction = CCW;
      if (current_direction != last_direction) M2_DIR = current_direction;
      PWM_UpdateDutyCycle(kPWM02, abs(speed));
      break;
    } 
    case kMotorFlipper: {
      static uint8_t last_direction = CCW;
      uint8_t current_direction;
      if (0 < speed) current_direction = CCW;
      else current_direction = CW;
      if (current_direction != last_direction) M3_DIR = current_direction;
      PWM_UpdateDutyCycle(kPWM03, abs(speed));
      break;
    }
  }
}


float DT_speed(const kMotor motor) {
  #define HZ_16US 100000.0
  switch (motor) {
    case kMotorLeft: 
      if (IC_period(kIC01)) return (HZ_16US / IC_period(kIC01));
    case kMotorRight: 
      if (IC_period(kIC02)) return (HZ_16US / IC_period(kIC02));
    case kMotorFlipper:
      if (IC_period(kIC03)) return (HZ_16US / IC_period(kIC03));
  }
  return 0;
}

/*---------------------------Helper Function Implementations------------------*/
// Function: InitPins
// Description: Configures and initializes the pins associated with the 
//   respective motor driver.
void InitPins(kMotor motor) {
  switch (motor) {
    case kMotorLeft:
      M1_DIR_EN(1); Nop();
    	M1_BRAKE_EN(1); Nop();
    	M1_MODE_EN(1); Nop();
    	M1_COAST_EN(1); Nop();
    	M1_TURN_BRAKE(OFF); Nop();
    	M1_TURN_COAST(ON); Nop();
    	M1_SET_MODE_TO(SLOW_DECAY_MODE); Nop();
      break;
    case kMotorRight:
    	M2_DIR_EN(1); Nop();
    	M2_BRAKE_EN(1); Nop();
    	M2_MODE_EN(1); Nop();
    	M2_COAST_EN(1); Nop();
      M2_TURN_BRAKE(OFF); Nop();
      M2_TURN_COAST(ON); Nop();
    	M2_SET_MODE_TO(SLOW_DECAY_MODE); Nop();
    case kMotorFlipper:
    	M3_DIR_EN(1); Nop();
    	M3_BRAKE_EN(1); Nop();
    	M3_MODE_EN(1); Nop();
    	M3_COAST_EN(1); Nop();
    	M3_TURN_BRAKE(OFF); Nop();
    	M3_TURN_COAST(ON); Nop();
      M3_SET_MODE_TO(SLOW_DECAY_MODE); Nop();
      break;
  }
}
