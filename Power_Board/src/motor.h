/// @file
/// Low-level motor details for speed control and tachometry
/// Uses timer4 + timer5, input capture 0, input capture 1, input capture 2, output compare 1,
/// output compare 2, output compare 3
#ifndef MOTOR_H
#define MOTOR_H

#include "stdhdr.h"

/// Motor controllor flags, both input and output. These are bitflags that should be OR-ed together.
/// These correspond to IO flags for Allegro A3930 BLDC Motor Controller (but negated, since the
/// pins are active-low) These cooperate with the PWM signal to determine motor behavior.
typedef enum MotorStatusFlag {
    /// No flags at all.
    MOTOR_FLAG_NONE = 0,

    /// Feedback flag: Does motor experience high current? 1 indicates some sort of long-circuit
    /// condition
    MOTOR_FLAG_FAULT1 = 1 << 0,

    /// Feedback flag: Does motor experience low current? 1 indicates some sort of short-circuit
    /// condition
    MOTOR_FLAG_FAULT2 = 1 << 1,

    /// Control flag: Should use fast current decay?
    /// Fast mode has higher dynamic response but worse for maintaining speed.
    /// Ignored when coasting or braking.
    MOTOR_FLAG_DECAY_MODE = 1 << 2,

    /// Control flag: Should drive motor in reverse direction? (this is the motor direction
    /// clockwise or counterclockwise,
    /// NOT forward or backwards w.r.t. the rover's heading)
    /// Ignored when coasting or braking.
    MOTOR_FLAG_REVERSE = 1 << 3,

    /// Control flag: Should brake motor? If enabled, PWM value is ignored.
    /// Ignored when coasting.
    MOTOR_FLAG_BRAKE = 1 << 4,

    /// Control flag: Should coast motor? If enabled, PWM value is ignored.
    MOTOR_FLAG_COAST = 1 << 5,
} MotorStatusFlag;

/// Bit mask corresponding to all values *read from* the motor, indicating whether the motor is
/// malfunctioning
static const MotorStatusFlag MOTOR_FLAG_MASK_FEEDBACK = (MOTOR_FLAG_FAULT1 | MOTOR_FLAG_FAULT2);

/// Bit mask corresponding corresponding to all values that are *written to* the motor, controlling
/// its behavior
static const MotorStatusFlag MOTOR_FLAG_MASK_CONTROL =
    (MOTOR_FLAG_BRAKE | MOTOR_FLAG_REVERSE | MOTOR_FLAG_COAST | MOTOR_FLAG_DECAY_MODE);

/// Index of each individual motor in the robot
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
#define EACH_MOTOR_CHANNEL(i) i = 0; (i) < MOTOR_CHANNEL_COUNT; (i)++
// clang-format on

/// Initialize driving control for the motor
void motor_init(MotorChannel channel);

/// Initialize motor feedback (tachometry)
void motor_tach_init();

/// Get the last-measured period of a motor in units of clock ticks
/// @param channel which motor?
/// @return Period of the motor in units of clock ticks.
int64_t motor_tach_get_period(MotorChannel channel);

/// Tell the motor controller what to do with the motor
/// @param channel Which motor to update?
/// @param status  What the new status flags should be. Only control flags will be used - fault
/// flags will be ignored
/// @param duty    The duty cycle of the motor; from 0 to 1000
/// @return The new motor status, with any new fault flags
MotorStatusFlag motor_update(MotorChannel channel, MotorStatusFlag status, uint16_t duty);

#endif
