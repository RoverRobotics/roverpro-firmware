/// @file
/// Low-level motor details for speed control and tachometry
/// Uses timer4 + timer5, input capture 0, input capture 1, input capture 2, output compare 1,
/// output compare 2, output compare 3
#ifndef MOTOR_H
#define MOTOR_H

#include "stdhdr.h"
#include "motor.h"

/// Reference to individually addressable motors
typedef enum MotorChannel {
    /// Motor controlling the left wheel
    MOTOR_LEFT = 0,
    /// Motor controlling the right wheel
    MOTOR_RIGHT,
    /// Motor controlling the flipper. Not all rovers have this motor.
    MOTOR_FLIPPER,
} MotorChannel;
/// The number of values of MotorChannel
#define MOTOR_CHANNEL_COUNT 3
/// Helper macro for iterating all motors and storing the result in variable i.
/// e.g. @code{.c}
/// int k;
/// for (EACH_MOTOR_CHANNEL(k))`
///     // ...
/// @endcode
// clang-format off
#define EACH_MOTOR_CHANNEL(i) i = 0; i < MOTOR_CHANNEL_COUNT; i++
// clang-format on

/// Initialize motor feedback (tachometry)
void motor_tach_init();

/// Get the last-measured period of a motor in units of 16-microseconds
/// @param channel The motor channel to use
/// @return Period of the motor in units of 16 microseconds. If the motor period is long, returns 0.
float motor_tach_get_period(MotorChannel channel);

/// Initialize motor control
void MotorsInit();

/// Tell motor controller to coast motor
void Coasting(MotorChannel channel);

/// Tell motor controller to brake motor
void Braking(MotorChannel channel);

/// Communicate new motor speeds/direction to the motor controller.
/// effort = signed effort to apply (-1000 : +1000)
void UpdateSpeed(MotorChannel channel, int16_t effort);

/// Initialize PWM channel 1 (left motor)
void PWM1Ini();

/// Initialize PWM channel 2 (right motor)
void PWM2Ini();

/// Initialize PWM channel 3 (flipper motor)
void PWM3Ini();

#endif
