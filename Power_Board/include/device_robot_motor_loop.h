/// @file Closed loop motor control

#ifndef DEVICE_ROBOT_MOTOR_LOOP_H
#define DEVICE_ROBOT_MOTOR_LOOP_H

#include "motor.h"

/// Set up the closed loop speed controller
void closed_loop_control_init(void);

/// Should be called at a constant time interval (e.g. every 10 ms)
/// Drives the closed loop speed controller, attempting to set motor speeds closer to the target
/// velocities.
void pid_tick(bool OverCurrent);

/// Get the output effort
/// @param motor Which motor channel the output is intended for
/// @return the speed in the range [-1000, 1000]
int return_closed_loop_control_effort(MotorChannel motor);

/// Sets the input of the closed loop speed controller to the given velocities
/// @param left Desired speed for the left motor in the range [-1000, 1000]
/// @param right Desired speed for the right motor in the range [-1000, 1000]
/// @param flipper Desired speed for the flipper motor in the range [-1000, 1000]
void pid_set_desired_velocities(int left, int right, int flipper);

#endif