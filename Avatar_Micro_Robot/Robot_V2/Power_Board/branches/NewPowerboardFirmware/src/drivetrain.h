/*==============================================================================
File: drivetrain.h
 
Description: This module provides an interface to the robot drivetrain.

TODO: ideally a motor would be a struct that owns the following data
typedef struct {
  float desired_speed;
  float actual_speed;
  Direction direction;    // TODO: typedef Direction kForward, kReverse
  uint16_t temperature;
} Motor;

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "./core/StandardHeader.h"

// supported motor options
typedef enum {
  kMotorLeft = 0,
  kMotorRight,
  kMotorFlipper,
} kMotor;

/*---------------------------Public Functions---------------------------------*/
// Function: DT_Init
// Description: Initializes the given motor.  Note that the powerbus must be
//   turned on as well.
void DT_Init(const kMotor motor);

// Function: DT_Run
// Description: Keeps any internal operations running, most notably timing-out
//   the feedback speed.  Place this in the main loop.
void DT_Run(void);

// Function: DR_set_speed
// Parameters:
//   const kMotor motor,  the motor on which to operate
//   const float speed,   the speed effort to apply (0.0-to-1.0)
//                        negative speed is interpreted as reverse
void DT_set_speed(const kMotor motor, const float speed);

// Function: DT_speed
// Description: Getter to retreive the last-updated actual speed of the given
//   motor.  A negative speed means the motor track is in reverse.
float DT_speed(const kMotor motor);

#endif
