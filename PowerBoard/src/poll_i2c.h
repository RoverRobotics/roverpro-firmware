/// @file
/// Internal digital communication between the MCU and various devices over I2C.

#pragma once
/// Asynchronously query and send commands to devices on I2C buses
/// fan controller, both batteries, and charger.
void i2c_tick_all();
