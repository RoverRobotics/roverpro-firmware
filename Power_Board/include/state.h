/// @file State of the rover.

#ifndef STATE_H
#define STATE_H

#include "motor.h"
#include "battery.h"
#include "stdbool.h"
#include "stdint.h"

/// The state of the rover at any point in time.
/// The functional area of a particular state variable is the part that *sets* the value. All values
/// should be written from one functional area but may affect others e.g. "overcurrent" affects the
/// "drive" functional area and is informed by data from the "analog" functional area, but liwes in
/// the "power" functional area because it is only set and cleared in power.h
typedef struct State {
    struct DriveState {
        uint16_t motor_current[MOTOR_CHANNEL_COUNT];
        uint32_t motor_encoder_count[2];
        uint8_t motor_fault_flags[2];
        uint16_t flipper_angle;
        uint16_t motor_encoder_period[2];
    } drive;
    struct AnalogState {
        uint16_t battery_voltage[BATTERY_COUNT];
        uint16_t flipper_sensors[2];
        uint16_t battery_current[BATTERY_COUNT];
        uint16_t motor_current[MOTOR_CHANNEL_COUNT];
    } analog;
    struct Power {
        bool overcurrent;
    } power;
    struct I2CState {
        uint16_t temperature_sensor[2];
        bool temperature_sensor_valid[2];
        uint16_t charger_state;
        uint16_t smartbattery_soc[BATTERY_COUNT];
        uint16_t smartbattery_status[BATTERY_COUNT];
        uint16_t smartbattery_mode[BATTERY_COUNT];
        uint16_t smartbattery_temperature[BATTERY_COUNT];
        uint16_t smartbattery_current[BATTERY_COUNT];
        uint16_t smartbattery_voltage[BATTERY_COUNT];
    } i2c;
    struct CommunicationState {
        bool use_manual_fan_speed;
        // value ranges from 0 (off) to 240 (100%)
        uint8_t manual_fan_speed;
        // values from -1000 to 1000
        int16_t motor_effort[MOTOR_CHANNEL_COUNT];
    } communication;
} State;

#endif