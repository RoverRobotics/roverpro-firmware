/// @file
/// Settings are persistent values that do not change in regular operation of the rover. They
/// are reloaded from non-volatile memory when the robot boots up.

#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
        /// How long to keep moving after receiving the last motor command
        uint16_t drive_command_timeout_ms;
        /// How many incoming data bytes to hold before discarding data
        uint16_t rx_bufsize_bytes;
        /// How many outgoing data bytes to hold before discarding data
        uint16_t tx_bufsize_bytes;
        /// The number of bits per second for communication (both sending and receiving)
        uint32_t baud_rate;
    } communication;
    struct {
        /// Below this temperature, no need to run the fan, in degrees Celsius.
        float fan_lo_temperature;
        /// The temperature at which to run the fan full blast, in degrees Celsius.
        float fan_hi_temperature;
        /// When the fan kicks in, it does so at this fraction of full effort. 0.0 = fan off; 1.0 =
        /// fan at maximum capacity.
        float fan_lo_duty_factor;
        /// When the fan is trying its hardest, it does so at this fraction of full effort. 0.0 =
        /// fan off; 1.0 = fan at maximum capacity.
        float fan_hi_duty_factor;
    } cooling;
    struct {
        /// The level of current (for either battery) at which we should start worrying
        uint16_t overcurrent_trigger_threshold_ma;
        /// How long the current should be at the trigger threshold for us to panic
        uint16_t overcurrent_trigger_duration_ms;
        /// How low the current must be (for both batteries) to stop panicking
        uint16_t overcurrent_reset_threshold_ma;
        /// If the current doesn't go down to the reset threshold, how long to stop panicking
        uint16_t overcurrent_reset_duration_ms;
        /// When charging, how often to alternate the power bus
        uint16_t charging_battery_switch_ms;
    } power;
    struct {
        /// Whether or not the calibration routine has been run and angle_offset can be trusted
        bool is_calibrated;
        /// Value needed to compute the actual flipper angle from onboard sensors. Computed by the
        /// flipper calibration routine
        int16_t angle_offset;
    } flipper;
    struct {
        /// How long to wait for a I2C communication step to time out
        // TODO: this is waaay longer than needed. it should probably be in microseconds (also I2C
        // should probably be interrupt-driven)
        //       profile the UART stuff and find out.
        uint16_t step_timeout_ms;
    } i2c;
    struct {
        /// Frequency of PWM signal to motor controllers
        float motor_pwm_frequency_hz;
        /// How much we should limit acceleration. If at a full stop, and commanded to go full speed
        /// ahead, this will be how long to ramp up motor commands to that speed.
        float time_to_full_speed;
        /// Should we use slow current decay mode in the motor controller?
        bool motor_slow_decay_mode;

        /// If encoder is above this frequency for a time, we are going too fast.
        float hi_speed_encoder_hz;
        /// If encoder is too fast, for this long trigger an overspeed fault
        float overspeed_fault_trigger_s;

        /// When we are in a fault condition, should we brake? (otherwise coast)
        bool brake_on_fault;
        /// When we stop receiving a motor command, should we brake? (otherwise coast)
        bool brake_on_zero_speed_command;
        /// When we stop receiving a motor command, should we brake? (otherwise coast)
        bool brake_on_drive_timeout;
    } drive;
} Settings;

/// Load and return settings from non-volatile memory
Settings settings_load();

/// Save the specified settings to non-volatile memory
void settings_save(const Settings *settings);

#endif