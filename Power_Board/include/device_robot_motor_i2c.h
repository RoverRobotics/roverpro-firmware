/// @file
/// Communication between the robot and various devices over I2C.

#ifndef DEVICE_ROBOT_MOTOR_I2C
#define DEVICE_ROBOT_MOTOR_I2C

#include <stdbool.h>

/// Restart I2C bus 2
void re_init_i2c2(void);

/// Restart I2C bus 3
void re_init_i2c3(void);

/// Asynchronously query fan controller and battery A, which are on I2C bus 2
void i2c2_tick(bool should_reset, bool *did_finish);

/// Asynchronously query charger and battery B, which are on I2C bus 3
void i2c3_tick(bool should_reset, bool *out_did_finish);

#endif