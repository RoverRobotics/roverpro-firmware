#include "hardware_definitions.h"
#include "i2clib.h"
#include "main.h"
#include "poll_i2c.h"

// A step may be reached by either falling through into it (first try)
// Or by jumping back to it (every retry)
// Note an i2c_result of I2C_ILLEGAL probably means there is a coding error, hence the breakpoint.
// If you want to check other I2C results, you can replace that check with (i2c_result != I2C_OKAY)
#define I2C_ASYNCHRONOUSLY(operationdef)                                                           \
    op = (operationdef);                                                                           \
    progress = I2C_PROGRESS_UNSTARTED;                                                             \
    resume_at = (__LINE__);                                                                        \
    stall_count = 0;                                                                               \
    case (__LINE__):                                                                               \
        i2c_result = i2c_tick(BUS, &op, &progress);                                                \
        if (i2c_result == I2C_NOTYET) {                                                            \
            stall_count++;                                                                         \
            return;                                                                                \
        }                                                                                          \
        BREAKPOINT_IF(i2c_result == I2C_ILLEGAL);                                                  \
        // fallthrough to next case

void re_init_i2c2(void) {
    i2c_disable(I2C_BUS2);

    _TRISF4 = 0;
    _TRISF5 = 0;
    _LATF4 = 0;
    _LATF5 = 0;

    i2c_enable(I2C_BUS2);
}

void re_init_i2c3(void) {
    i2c_disable(I2C_BUS3);

    _TRISE6 = 0;
    _TRISE7 = 0;
    _LATE6 = 0;
    _LATE7 = 0;

    i2c_enable(I2C_BUS3);
}

void i2c2_tick() {
    const I2CBus BUS = I2C_BUS2;
    static uint16_t stall_count = 0;
    static I2COperationDef op;
    static int resume_at = 0;
    static I2CProgress progress;
    static uint16_t a_word;
    static uint8_t a_byte;

    I2CResult i2c_result;
    if (stall_count * g_settings.main.i2c_poll_ms > g_settings.i2c.step_timeout_ms) {
        if (resume_at != 0) {
            re_init_i2c2();
        }
        resume_at = 0; // start from the beginning
    }
    switch (resume_at) {
    case 0:
        // Fan Controller read Temperature channel 1
        I2C_ASYNCHRONOUSLY(i2c_op_read_byte(FAN_CONTROLLER_ADDRESS, 0x00, &a_byte))
        g_state.i2c.temperature_sensor_valid[0] = (i2c_result == I2C_OKAY);
        g_state.i2c.temperature_sensor[0] = a_byte;

        // Fan Controller read Temperature channel 2
        // This tends to fail. Does it ever work?
        I2C_ASYNCHRONOUSLY(i2c_op_read_byte(FAN_CONTROLLER_ADDRESS, 0x01, &a_byte))
        g_state.i2c.temperature_sensor_valid[1] = (i2c_result == I2C_OKAY);
        g_state.i2c.temperature_sensor[1] = a_byte;

        a_byte = g_state.communication.fan_speed;
        I2C_ASYNCHRONOUSLY(i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &a_byte));

        // Smart Battery read RelativeStateOfCharge
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x0d, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_soc[0] = a_word;
        }

        // Smart Battery read BatteryStatus
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x16, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_status[0] = a_word;
        }

        // Smart Battery read BatteryMode
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x03, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_mode[0] = a_word;
        }

        // Smart Battery read Temperature
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x08, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_temperature[0] = a_word;
        }

        // Smart Battery read Voltage
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x09, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_voltage[0] = a_word;
        }

        // Smart Battery read Current
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x0a, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_current[0] = a_word;
        }

        resume_at = 0;
    }
}

void i2c3_tick() {
    const I2CBus BUS = I2C_BUS3;
    static uint16_t stall_count = 0;
    static I2COperationDef op;
    static int resume_at = 0;
    static I2CProgress progress;
    static uint16_t a_word;
    I2CResult i2c_result;

    if (stall_count * g_settings.main.i2c_poll_ms > g_settings.i2c.step_timeout_ms) {
        if (resume_at != 0) {
            re_init_i2c3();
        }
        resume_at = 0; //  start from the beginning
    }
    switch (resume_at) {
    case 0:
        // Smart Battery read RelativeStateOfCharge
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x0d, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_soc[1] = a_word;
        }

        // See Internal_Charger firmware for slave device logic.
        // Note the battery charger is expected to be unreachable except while charging
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_CHARGER_ADDRESS, 0xca, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.charger_state = a_word;
        } else {
            g_state.i2c.charger_state = 0;
        }

        // Smart Battery read BatteryStatus
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x16, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_status[1] = a_word;
        }

        // Smart Battery read BatteryMode
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x03, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_mode[1] = a_word;
        }

        // Smart Battery read Temperature
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x08, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_temperature[1] = a_word;
        }

        // Smart Battery read Voltage
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x09, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_voltage[1] = a_word;
        }

        // Smart Battery read Current
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x0a, &a_word))
        if (i2c_result == I2C_OKAY) {
            g_state.i2c.smartbattery_current[1] = a_word;
        }

        resume_at = 0;
    }
}
void i2c_tick_all() {
    /// Asynchronously query fan controller and battery A, which are on I2C bus 2
    i2c2_tick();

    /// Asynchronously query charger and battery B, which are on I2C bus 3
    i2c3_tick();
}