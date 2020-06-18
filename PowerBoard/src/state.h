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

        int64_t last_lo_speed_timestamp;
        int64_t last_overspeed_fault_timestamp;
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
        /// If nonzero, we are drawing too much battery power and are at risk of triggering a
        /// hardware overcurrent condition if we continue at current consumption.
        int64_t last_overcurrent_fault_timestamp;
    } power;
    /// State of the digital monitoring subsystem
    struct I2CState {
        /// Digital temperature values, as reported by fan controller
        uint16_t temperature_sensor[2];
        /// Whether @ref temperature_sensor values were successfully polled
        bool temperature_sensor_valid[2];
        /// The fan duty (0-240) that the fan controller is currently aiming for
        uint8_t fan_duty[2];
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
        int16_t smartbattery_current[BATTERY_COUNT];
        /// Last reported SmartBattery voltage
        uint16_t smartbattery_voltage[BATTERY_COUNT];

        uint64_t last_manual_fan_speed_request_timestamp;
        uint8_t manual_fan_speed;
        uint64_t last_manual_fan_speed_done_timestamp;
    } i2c;
    /// State of the communication subsystem
    struct CommunicationState {
        ByteQueue rx_q;
        ByteQueue tx_q;
        uint64_t drive_command_timestamp;
        /// Last requested motor effort. Values from -1.0 to 1.0
        float motor_effort[MOTOR_CHANNEL_COUNT];
    } communication;
} State;

typedef enum Fault {
    FAULT_NONE = 0,
    FAULT_OVERSPEED = 1U << 0U,
    FAULT_OVERCURRENT = 1U << 1U,
} SystemFaultFlag;

#endif