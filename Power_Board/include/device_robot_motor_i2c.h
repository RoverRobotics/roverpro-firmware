/// @file
/// Communication between the robot and various devices over I2C.

#ifndef DEVICE_ROBOT_MOTOR_I2C
#define DEVICE_ROBOT_MOTOR_I2C

/// Asynchronously query and send commands to devices on I2C buses
/// fan controller, both batteries, and charger.
void i2c_tick_all();

#endif