#include "cooling.h"
#include "clock.h"
#include "hardware_definitions.h"
#include "i2clib.h"
#include "main.h"
#include <math.h>

const uint8_t FAN_DUTY_RATE_OF_CHANGE_INSTANTANEOUS = 0x00;
const uint8_t FAN_DUTY_RATE_OF_CHANGE_DEFAULT = 0b01001000;
const uint8_t FAN_DUTY_MAX = 240;

void cooling_blast_fan() {
    g_state.i2c.manual_fan_speed = FAN_DUTY_MAX;
    g_state.i2c.last_manual_fan_speed_done_timestamp = 0;
    g_state.i2c.last_manual_fan_speed_request_timestamp = clock_now();
}

void cooling_init() {
    uint8_t fan_lo_duty_cycle = (uint8_t)(FAN_DUTY_MAX * g_settings.cooling.fan_lo_duty_factor);
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x07, &fan_lo_duty_cycle));
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x08, &fan_lo_duty_cycle));

    uint8_t fan_hi_duty_cycle = (uint8_t)(FAN_DUTY_MAX * g_settings.cooling.fan_hi_duty_factor);
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x09, &fan_hi_duty_cycle));
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0a, &fan_hi_duty_cycle));

    uint8_t fan_start_temp = (uint8_t)g_settings.cooling.fan_lo_temperature;
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0f, &fan_start_temp));
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x10, &fan_start_temp));

    // set fan update rate
    i2c_synchronously_await(
        I2C_BUS2,
        i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x12, &FAN_DUTY_RATE_OF_CHANGE_DEFAULT));

    // configure which temperature sensors to use
    uint8_t fan_config =
        0b00111100; // set both temperature inputs - it will use whichever is higher
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x11, &fan_config));

    float duty_cycle_step =
        FAN_DUTY_MAX *
        (g_settings.cooling.fan_hi_duty_factor - g_settings.cooling.fan_lo_duty_factor) /
        (g_settings.cooling.fan_hi_temperature - g_settings.cooling.fan_lo_temperature) / 2;
    uint8_t duty_cycle_step_uint4 = clamp((uint8_t)ceilf(duty_cycle_step), 0, 0xff);
    uint8_t duty_cycle_step_sizes = duty_cycle_step_uint4 << 4 | duty_cycle_step_uint4;
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x13, &duty_cycle_step_sizes));
}
