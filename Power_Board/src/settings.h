/// @file
/// Settings are persistent values that do not change in regular operation of the rover. They
/// are reloaded from non-volatile memory when the robot boots up.

#ifndef SETTINGS_H
#define SETTINGS_H

#include "stdbool.h"
#include "stdint.h"

/// Persistent values that do not change in regular operation of the rover. @ref g_settings =
/// current settings Settings is grouped according to the functional area affected.
typedef struct Settings {
    struct {
        /// Date when the firmware was compiled
        char build_date[12];
        /// Time when the firmware was compiled
        char build_time[12];
        uint16_t release_version_flat;
    } firmware;
    struct {
        /// How often to update the motor pwm / direction protection
        uint16_t drive_poll_ms;
        /// How often to update the electrical protection logic
        uint16_t power_poll_ms;
        /// How often to update internal serial communication.
        /// this is responsible for communicating the with the fan and measuring some SmartBattery
        /// info
        uint16_t i2c_poll_ms;
        /// How often to check for and process inbound UART commands.
        uint16_t communication_poll_ms;
        /// How often to read out internal analog outputs.
        uint16_t analog_poll_ms;
        /// How often to update the flipper positional feedback and command
        uint16_t flipper_poll_ms;
    } main;
    struct {
        uint16_t drive_command_timeout_ms;
        uint16_t fan_command_timeout_ms;
        uint16_t rx_bufsize_bytes;
        uint16_t tx_bufsize_bytes;
        uint32_t baud_rate;
    } communication;
    struct {
        uint16_t overcurrent_trigger_threshold_ma;
        uint16_t overcurrent_trigger_duration_ms;
        uint16_t overcurrent_reset_threshold_ma;
        uint16_t overcurrent_reset_duration_ms;
        uint16_t charging_battery_switch_ms;
    } power;
    struct {
        /// Whether or not the calibration routine has been run and angle_offset can be trusted
        bool is_calibrated;
        /// Value needed to compute the actual flippera angle from onboard sensors. Computed by the
        /// flipper calibration routine
        int16_t angle_offset;
    } flipper;
    struct {
        uint16_t step_timeout_ms;
    } i2c;
    struct {
        /// Frequency of PWM signal to motor controllers
        uint16_t motor_pwm_frequency_khz;
        /// Amount of time to coast the motors in between direction changes
        uint16_t motor_protect_direction_delay_ms;
    } drive;
} Settings;

/// Load and return settings from non-volatile memory
Settings settings_load();

/// Save the specified settings to non-volatile memory
void settings_save(const Settings *settings);

#endif