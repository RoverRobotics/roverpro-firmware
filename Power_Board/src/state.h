/// @file
/// State of the rover.

#ifndef STATE_H
#define STATE_H

#include "battery.h"
#include "bytequeue.h"
#include "motor.h"
#include <stdbool.h>
#include <stdint.h>

/// Transient state of the rover at any point in time.
/// State is grouped according to the functional area that *sets* the value.
/// Values may be used/affect others e.g. "overcurrent" affects the "drive" functional area and is
/// informed by data from the "analog" functional area, but lives in the "power" functional area
/// because it is only set and cleared in power.h
typedef struct State {
    /// State of the drive subsystem
    struct DriveState {
        uint64_t last_encoder_timestamp[MOTOR_CHANNEL_COUNT];
        /// Motor encoder values. Increments when moving forward, decrements backward.
        uint16_t motor_encoder_count[MOTOR_CHANNEL_COUNT];
        /// Motor encoder period - proportional to the inverse of the speed
        uint16_t motor_encoder_period[MOTOR_CHANNEL_COUNT];
        uint16_t flipper_angle;
        MotorStatusFlag motor_status[MOTOR_CHANNEL_COUNT];
        uint64_t last_update_time;
        /// The last effort sent to the motors
        float last_motor_effort[MOTOR_CHANNEL_COUNT];
    } drive;
    /// State of the analog monitoring subsystem
    struct AnalogState {
        /// Voltage from each battery, according to an analog sensor
        uint16_t battery_voltage[BATTERY_COUNT];
        /// Reading of each flipper position potentiometer
        uint16_t flipper_sensors[2];
        /// Current from each battery, according to an analog sensor
        uint16_t battery_current[BATTERY_COUNT];
        /// Current to each motor, according to an analog sensor
        uint16_t motor_current[MOTOR_CHANNEL_COUNT];
    } analog;
    /// State of the overcurrent protection subsystem
    struct PowerState {
        /// If true, we are drawing too much battery power and are at risk of triggering a hardware
        /// overcurrent condition if we continue at current consumption.
        bool overcurrent;
    } power;
    /// State of the digital monitoring subsystem
    struct I2CState {
        /// Digital temperature values, as reported by fan controller
        uint16_t temperature_sensor[2];
        /// Whether @ref temperature_sensor values were successfully polled
        bool temperature_sensor_valid[2];
        /// Whether an external voltage supply is present. (0xdada if true, 0 otherwise)
        uint16_t charger_state;
        /// Last reported SmartBattery state of charge (0-100)
        uint16_t smartbattery_soc[BATTERY_COUNT];
        /// Last reported SmartBattery status bitflags
        uint16_t smartbattery_status[BATTERY_COUNT];
        /// Last reported SmartBattery mode bitflags
        uint16_t smartbattery_mode[BATTERY_COUNT];
        /// Last reported SmartBattery temperature
        uint16_t smartbattery_temperature[BATTERY_COUNT];
        /// Last reported SmartBattery current
        uint16_t smartbattery_current[BATTERY_COUNT];
        /// Last reported smartbattery voltage
        uint16_t smartbattery_voltage[BATTERY_COUNT];
    } i2c;
    /// State of the communication subsystem
    struct CommunicationState {
        ByteQueue rx_q;
        ByteQueue tx_q;
        uint64_t fan_command_timestamp;
        /// Last requested fan speed. Value ranges from 0 (off) to 240 (100%)
        uint8_t fan_speed;
        uint64_t drive_command_timestamp;
        /// Last requested motor effort. Values from -1.0 to 1.0
        float motor_effort[MOTOR_CHANNEL_COUNT];
        /// Whether a 0 motor speed should be interpreted as a brake. Otherwise, interpret it as a
        /// coast.
        bool brake_when_stopped[MOTOR_CHANNEL_COUNT];

        /// A history of times we have faulted. When this fills up and we fault, we enter a runaway
        /// condition.
        uint64_t *overspeed_fault_times;
        /// The time we went over the speed limit. This does not necessarily mean we have faulted
        /// yet.
        uint64_t overspeed_time;
        /// The time we faulted.
        uint64_t overspeed_fault_time;
        /// The last time a runaway condition has been hit
        uint64_t overspeed_runaway_time;
        /// The last time we have gone below the runaway_reset_effort threshold
        /// Any faults before this time should be forgiven.
        uint64_t overspeed_runaway_reset_time;
        /// The last time we have hit a runaway condition.
        bool overspeed_runaway;
    } communication;
} State;

#endif