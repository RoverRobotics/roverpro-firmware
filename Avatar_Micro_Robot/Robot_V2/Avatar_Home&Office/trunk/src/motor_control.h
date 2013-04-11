/*==============================================================================
File: ESC.h

Description: This module defines an Electronic Speed Controller (ESC) to drive
  a Brushless DC (BLDC) motor.

Notes:
  - consumes timer4 as the time base for output compare
  - consumes output compare modules OC1, OC2 and OC3
  - employs Bipolar PWM switching -— both the high-side and low-side ON
    transistors are modulated (better for sensorless design)
  - requires a motor with sensors
  - most BLDC motors have a three-phase winding topology with a star connection
  - most timing offsets (mechanical offset of the position sensors along the 
    motor shaft) are 15-45deg

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef ESC_H
#define ESC_H
//---------------------------Dependencies---------------------------------------
#include "./core/StandardHeader.h"  // uintN_t data types

//---------------------------Public Functions-----------------------------------
// Function: ESC_Init
// Description: Initializes the electronic speed controller module.
void ESC_Init(void);

// Function: ESC_StartMotor
// Description: Begins commutating the motor at the current duty cycle.
void ESC_StartMotor(const int16_t speed);

// Function: ESC_set_speed
// Parameters:
//   const int16_t speed,  the speed effort to apply (0.0-to-1.0)
//                         negative speed is interpreted as reverse
void ESC_set_speed(const int16_t speed);

// Function: ESC_speed
// Description: Getter to retreive the last-updated actual speed of the given
//   motor.  A negative speed means the motor track is in reverse.
int16_t ESC_speed(void);

void motor_control_test_function(void);

void motor_control_FSM(void);

unsigned char are_motors_stopped(void);

//ADC counts = .01*i*.001*11k*1023/3.3
//ADC counts = i*34.1
//set overcurrent to be 10A
#define OVERCURRENT_ADC 341

#endif
