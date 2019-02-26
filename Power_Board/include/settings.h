#ifndef SETTINGS_H
#define SETTINGS_H

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

typedef struct Settings {
    struct {
        uint16_t motor_poll_ms;
        uint16_t electrical_poll_ms;
        uint16_t i2c_poll_ms;
        uint16_t communication_poll_ms;
        uint16_t analog_readouts_poll_ms;
        uint16_t motor_controller_poll_ms;
    } main;
    struct {
        uint16_t motor_command_timeout_ms;
        uint16_t fan_command_timeout_ms;
        uint16_t baud_rate;
    } communication;
    struct {
        uint16_t overcurrent_trigger_threshold_ma;
        uint16_t overcurrent_trigger_duration_ms;
        uint16_t overcurrent_reset_threshold_ma;
        uint16_t overcurrent_reset_duration_ms;
    } electrical;
    struct {
        bool is_calibrated;
        uint16_t pot1;
        uint16_t pot2;
    } flipper;
    struct {
        uint16_t step_timeout_ms;
    } i2c;
    struct {
        float pid_p_weight;
        float pid_i_weight;
        float pid_d_weight;
        float min_effort;
        float max_effort;
        float iir_alpha;
    } motor_controller;
    struct {
        uint16_t pwm_hz;
        uint16_t motor_dead_time_between_directions_ms;
    } motor;
} Settings;

Settings settings_load();
void settings_save(const Settings *settings);

#endif