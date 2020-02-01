#include "cooling.h"
#include "hardware_definitions.h"
#include "i2clib.h"
#include "main.h"
#include <math.h>

const uint8_t FAN_MAX_DUTY = 240;
void cooling_blast_fan() {

    // set rate of change to instantaneous so that the fan ramps up now
    uint8_t fan_duty_rate_of_change_instant = 0x00;
    i2c_synchronously_await(
        I2C_BUS2,
        i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x12, &fan_duty_rate_of_change_instant));

    // blast fan up to max duty cycle
    uint8_t fan_max_duty = FAN_MAX_DUTY;
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &fan_max_duty));
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0c, &fan_max_duty));

    // reset rate of change so the fan
    uint8_t fan_duty_rate_of_change = 0b01001000;
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x12, &fan_duty_rate_of_change));
}

void cooling_init() {
    uint8_t fan_lo_duty_cycle = (uint8_t)(FAN_MAX_DUTY * g_settings.cooling.fan_lo_duty_factor);
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x07, &fan_lo_duty_cycle));
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x08, &fan_lo_duty_cycle));

    uint8_t fan_hi_duty_cycle = (uint8_t)(FAN_MAX_DUTY * g_settings.cooling.fan_hi_duty_factor);
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
    uint8_t fan_duty_rate_of_change = 0b01001000;
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x12, &fan_duty_rate_of_change));

    // configure which temperature sensors to use
    uint8_t fan_config =
        0b00111100; // set both temperature inputs - it will use whichever is higher
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x11, &fan_config));

    float duty_cycle_step =
        FAN_MAX_DUTY *
        (g_settings.cooling.fan_hi_duty_factor - g_settings.cooling.fan_lo_duty_factor) /
        (g_settings.cooling.fan_hi_temperature - g_settings.cooling.fan_lo_temperature) / 2;
    uint8_t duty_cycle_step_uint4 = clamp((uint8_t)ceilf(duty_cycle_step), 0, 0xff);
    uint8_t duty_cycle_step_sizes = duty_cycle_step_uint4 << 4 | duty_cycle_step_uint4;
    i2c_synchronously_await(
        I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x13, &duty_cycle_step_sizes));
}
