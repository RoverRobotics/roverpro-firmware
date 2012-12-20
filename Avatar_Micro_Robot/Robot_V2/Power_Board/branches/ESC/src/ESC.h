/*==============================================================================
File: ESC.h

Description: This file encapsulates the control of the brushless DC (BLDC) 
  motors on the robot.  Electronic Speed Controller (ESC).
  
Notes:
  - consumes timer3 as the time base for input capture
  - consumes input capture module 1, 2 and 3
  - consumes output compare module 1, 2, 3, 4, 5, 6???
  - most BLDC motors have three-phase winding topology with star connection
 
Goals:
  - be lightweight, fast enough to NOT miss commutation event

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef ESC_H
#define ESC_H
//---------------------------Dependencies---------------------------------------

//---------------------------Public Functions-----------------------------------


// Function: ESC_Init
// Description: Initializes the electronic speed controller module.
void ESC_Init(void);

void ESC_StartMotor(void);

/*
// Function: ESC_set_speed
// Parameters:
//   const float speed,   the speed effort to apply (0.0-to-1.0)
//                        negative speed is interpreted as reverse
void ESC_set_speed(const float speed);

// Function: ESC_speed
// Description: Getter to retreive the last-updated actual speed of the given
//   motor.  A negative speed means the motor track is in reverse.
float ESC_speed(void);
*/

#endif
