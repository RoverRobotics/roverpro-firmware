/*==============================================================================
File: ESC.h

Description: This file encapsulates the control of the brushless DC (BLDC) 
  motors on the robot.  Electronic Speed Controller (ESC).
  
Notes:
  - consumes timer4 as the time base for output compare
  - consumes output compare modules OC1, OC2 and OC3
  - most BLDC motors have three-phase winding topology with star connection

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef ESC_H
#define ESC_H

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

#endif
