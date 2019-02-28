/// @file high-level motor control and main event loop.

#include "main.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "device_robot_motor_i2c.h"
#include "i2clib.h"
#include "motor.h"
#include "uart_control.h"
#include "counter.h"
#include "device_power_bus.h"
#include "math.h"
#include "flipper.h"
#include "drive.h"
#include "analog.h"

//****************************************************
Counter speed_update_timer[MOTOR_CHANNEL_COUNT] = {
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
};

/// count of how many millis since last inbound motor command. When this expires, we should stop the
/// motors
Counter motor_speed_timeout = {
    .max = INTERVAL_MS_USB_TIMEOUT, .pause_on_expired = true, .is_paused = true};
/// count of how many millis since last motor direction update.
Counter motor_direction_state_machine = {.max = INTERVAL_MS_MOTOR_DIRECTION_STATE_MACHINE};
/// count of how many millis since last time we measured the current to the motors
Counter current_fb = {.max = INTERVAL_MS_CURRENT_FEEDBACK};
Counter uart_fan_speed_timeout = {
    .max = INTERVAL_MS_UART_FAN_SPEED_TIMEOUT, .pause_on_expired = true, .is_paused = true};

void set_firmware_build_time(void);

//*********************************************//

/// Increment `value` until it hits limit,
/// at which point reset it to zero.
bool tick_counter(uint16_t *value, uint16_t limit) {
    if (++(*value) >= limit) {
        *value = 0;
        return true;
    } else {
        return false;
    }
}

void DeviceRobotMotorInit() {
    // make sure we start off in a default state
    AD1PCFGL = 0xffff;
    AD1PCFGH = 0x0003;

    TRISB = 0xffff;
    TRISC = 0xffff;
    TRISD = 0xffff;
    TRISE = 0xffff;
    TRISF = 0xffff;
    TRISG = 0xffff;

    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);
    power_bus_init();
    uart_init();

    analog_init();

    // initialize motor control
    MotorsInit();
    // initialize motor tachometer feedback
    motor_tach_init();

    set_firmware_build_time();

    FANCtrlIni();
}

void Device_MotorController_Process() {
    static struct {
        uint16_t motor;
        uint16_t electrical;
        uint16_t i2c;
        uint16_t communication;
        uint16_t analog;
        uint16_t flipper;
        uint16_t drive;
    } counters = {0};

    UArtTickResult uart_tick_result;
    // this should run every 1ms
    power_bus_tick();

    if (tick_counter(&counters.drive, g_settings.main.drive_poll_ms)) {
        drive_tick();
    }

    if (tick_counter(&counters.analog, g_settings.main.analog_poll_ms)) {
        // update current for all three motors
        analog_tick();

        REG_MOTOR_FB_CURRENT.left = analog_get_value(ADC_MOTOR_LEFT_CURRENT);
        REG_MOTOR_FB_CURRENT.right = analog_get_value(ADC_MOTOR_RIGHT_CURRENT);
        REG_MOTOR_FB_CURRENT.flipper = analog_get_value(ADC_MOTOR_FLIPPER_CURRENT);

        // update the mosfet driving fault flag pin 1-good 2-fault
        REG_MOTOR_FAULT_FLAG.left = PORTDbits.RD1;
        REG_MOTOR_FAULT_FLAG.right = PORTEbits.RE5;

        REG_PWR_BAT_VOLTAGE.a = analog_get_value(ADC_CELL_A_VOLTAGE);
        REG_PWR_BAT_VOLTAGE.b = analog_get_value(ADC_CELL_B_VOLTAGE);

        REG_PWR_A_CURRENT = analog_get_value(ADC_CELL_A_CURRENT);
        REG_PWR_B_CURRENT = analog_get_value(ADC_CELL_B_CURRENT);
        REG_PWR_TOTAL_CURRENT = REG_PWR_A_CURRENT + REG_PWR_B_CURRENT;

        // read out measured motor periods.
        REG_MOTOR_FB_PERIOD_LEFT = (uint16_t)fabs(motor_tach_get_period(MOTOR_LEFT));
        REG_MOTOR_FB_PERIOD_RIGHT = (uint16_t)fabs(motor_tach_get_period(MOTOR_RIGHT));
    }

    // electrical subsystem
    if (tick_counter(&counters.electrical, g_settings.main.electrical_poll_ms)) {
        static uint16_t current_spike_counter;
        static uint16_t current_recover_counter;

        uint16_t trigger_thresh =
            g_settings.electrical.overcurrent_trigger_threshold_ma * 34 / 1000;
        uint16_t reset_thresh = g_settings.electrical.overcurrent_reset_threshold_ma * 34 / 1000;

        if (REG_PWR_A_CURRENT >= trigger_thresh || REG_PWR_B_CURRENT >= trigger_thresh) {
            // current spike
            current_spike_counter++;
            current_recover_counter = 0;

            if (current_spike_counter * g_settings.main.electrical_poll_ms >=
                g_settings.electrical.overcurrent_trigger_duration_ms) {
                drive_set_coast_lock(true);
                g_overcurrent = true;
            }
        } else {
            current_recover_counter++;
            if (REG_PWR_A_CURRENT <= reset_thresh && REG_PWR_B_CURRENT <= reset_thresh) {
                // low current state
                current_spike_counter = 0;
            }
            if (current_recover_counter * g_settings.main.electrical_poll_ms >=
                g_settings.electrical.overcurrent_reset_duration_ms)
                drive_set_coast_lock(false);
            g_overcurrent = false;
        }
    }

    if (tick_counter(&counters.i2c, g_settings.main.i2c_poll_ms)) {
        i2c_tick_all();
    }
    if (tick_counter(&counters.communication, g_settings.main.communication_poll_ms)) {
        uart_tick_result = uart_tick();
    }
    if (uart_tick_result.uart_flipper_calibrate_requested) {
        // note flipper calibration never returns.
        flipper_feedback_calibrate();
    }
    if (uart_tick_result.uart_fan_speed_requested) {
        counter_restart(&uart_fan_speed_timeout);
    }
    if (counter_tick(&uart_fan_speed_timeout) == COUNTER_EXPIRED) {
        // clear all the fan command
        REG_MOTOR_SIDE_FAN_SPEED = 0;
    }
    if (uart_tick_result.uart_motor_speed_requested) {
        counter_restart(&motor_speed_timeout);
    }
    // long time no data, clear everything and stop moving
    if (counter_tick(&motor_speed_timeout) == COUNTER_EXPIRED) {
        REG_MOTOR_VELOCITY.left = 0;
        REG_MOTOR_VELOCITY.right = 0;
        REG_MOTOR_VELOCITY.flipper = 0;

        MotorEfforts brake_all = {0};
        drive_set_efforts(brake_all);
    }

    if (tick_counter(&counters.flipper, g_settings.main.flipper_poll_ms)) {
        tick_flipper_feedback();
    }
}

//**********************************************

void set_firmware_build_time(void) {
    const unsigned char build_date[12] = __DATE__;
    const unsigned char build_time[12] = __TIME__;
    unsigned int i;

    for (i = 0; i < 12; i++) {
        if (build_date[i] == 0) {
            REG_MOTOR_FIRMWARE_BUILD.data[i] = ' ';
        } else {
            REG_MOTOR_FIRMWARE_BUILD.data[i] = build_date[i];
        }
        if (build_time[i] == 0) {
            REG_MOTOR_FIRMWARE_BUILD.data[i + 12] = ' ';
        } else {
            REG_MOTOR_FIRMWARE_BUILD.data[i + 12] = build_time[i];
        }
    }
}

void FANCtrlIni() {
    uint8_t a_byte;

    // reset the IC
    a_byte = 0b01011000;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x02, &a_byte));

    // auto fan speed control mode
    // writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x11,0b00111100);

    // manual fan speed control mode
    a_byte = 0;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x11, &a_byte));
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x12, &a_byte));

    // blast fan up to max duty cycle
    a_byte = 240;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &a_byte));

    block_ms(4000);

    // return fan to low duty cycle
    a_byte = 0;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &a_byte));
}