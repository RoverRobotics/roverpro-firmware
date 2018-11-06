#include <p24fxxxx.h>
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "i2clib.h"
#include "device_robot_motor_i2c.h"

bool I2C2XmitReset = false;
bool I2C3XmitReset = false;

bool I2C2TimerExpired = false;
bool I2C3TimerExpired = false;

// A step may be reached by either falling through into it (first try)
// Or by jumping back to it (every retry)
// Note an i2c_result of I2C_ILLEGAL probably means there is a coding error, hence the breakpoint.
// If you want to check other I2C results, you can replace that check with (i2c_result != I2C_OKAY)
#define I2C_ASYNCHRONOUSLY(operationdef)                                                           \
    op = (operationdef);                                                                           \
    progress = I2C_PROGRESS_UNSTARTED;                                                             \
    resume_at = (__LINE__);                                                                        \
    case (__LINE__):                                                                               \
        i2c_result = i2c_tick(BUS, &op, &progress);                                                \
        if (i2c_result == I2C_NOTYET) {                                                            \
            return;                                                                                \
        }                                                                                          \
        BREAKPOINT_IF(i2c_result == I2C_ILLEGAL);                                                  \
        // fallthrough to next case

void I2C2Update(void) {
    const I2CBus BUS = I2C_BUS2;

    static I2COperationDef op;
    static I2CProgress progress;
    static int resume_at = 0;
    static uint16_t a_word;
    static uint8_t a_byte;
    I2CResult i2c_result;

    if (I2C2XmitReset) {
        I2C2XmitReset = false;
        if (resume_at != 0) {
            // i.e. we didn't reach the end last time
            BREAKPOINT();
            re_init_i2c2();
        }
        resume_at = 0; // start from the beginning
    }
    switch (resume_at) {
    case 0:
        // Fan Controller read Temperature channel 1
        I2C_ASYNCHRONOUSLY(i2c_op_read_byte(FAN_CONTROLLER_ADDRESS, 0x00, &a_byte))
        REG_MOTOR_TEMP_STATUS.left = (i2c_result == I2C_OKAY);
        REG_MOTOR_TEMP.left = a_byte;

        // Fan Controller read Temperature channel 2
        // This tends to fail. Does it ever work?
        I2C_ASYNCHRONOUSLY(i2c_op_read_byte(FAN_CONTROLLER_ADDRESS, 0x01, &a_byte))
        REG_MOTOR_TEMP_STATUS.right = (i2c_result == I2C_OKAY);
        REG_MOTOR_TEMP.right = a_byte;

        // Fan Controller write PWM1 target duty cycle
        if (uart_has_new_fan_speed) {
            a_byte = REG_MOTOR_SIDE_FAN_SPEED;
        } else if (abs(uart_motor_velocity[0]) + abs(uart_motor_velocity[1]) > 10) {
            a_byte = REG_MOTOR_SIDE_FAN_SPEED = 240;
        } else {
            a_byte = REG_MOTOR_SIDE_FAN_SPEED = 0;
        }
        I2C_ASYNCHRONOUSLY(i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &a_byte));

        // Smart Battery read RelativeStateOfCharge
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x0d, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_ROBOT_REL_SOC_A = a_word;
        }

        // Smart Battery read BatteryStatus
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x16, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_STATUS_A = a_word;
        }

        // Smart Battery read BatteryMode
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x03, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_MODE_A = a_word;
        }

        // Smart Battery read Temperature
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x08, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_TEMP_A = a_word;
        }

        // Smart Battery read Voltage
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x09, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_VOLTAGE_A = a_word;
        }

        // Smart Battery read Current
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x0a, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_CURRENT_A = a_word;
        }

        I2C2TimerExpired = false; // reset the I2C2 update timer
        resume_at = 0;
    }
}

void I2C3Update(void) {
    const I2CBus BUS = I2C_BUS3;

    static I2COperationDef op;
    static I2CProgress progress;

    static int resume_at = 0;
    static uint16_t a_word;
    I2CResult i2c_result;

    if (I2C3XmitReset) {
        I2C3XmitReset = false;
        if (resume_at != 0) {
            // i.e. we didn't reach the end last time
            BREAKPOINT();
            re_init_i2c3();
        }
        resume_at = 0; //  start from the beginning
    }
    switch (resume_at) {
    case 0:
        // Smart Battery read RelativeStateOfCharge
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x0d, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_ROBOT_REL_SOC_B = a_word;
        }

        // See Internal_Charger firmware for slave device logic.
        // Note the battery charger is expected to be unreachable except while charging
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_CHARGER_ADDRESS, 0xca, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_MOTOR_CHARGER_STATE = a_word;
        } else {
            REG_MOTOR_CHARGER_STATE = 0;
        }

        // Smart Battery read BatteryStatus
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x16, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_STATUS_B = a_word;
        }

        // Smart Battery read BatteryMode
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x03, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_MODE_B = a_word;
        }

        // Smart Battery read Temperature
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x08, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_TEMP_B = a_word;
        }

        // Smart Battery read Voltage
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x09, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_VOLTAGE_B = a_word;
        }

        // Smart Battery read Current
        I2C_ASYNCHRONOUSLY(i2c_op_read_word(BATTERY_ADDRESS, 0x0a, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_CURRENT_B = a_word;
        }

        I2C3TimerExpired = false; // reset the I2C3 update timer
        resume_at = 0;
    }
}

//*********************************************//

void re_init_i2c2(void) {
    i2c_disable(I2C_BUS2);

    // New Bren-Tronics battery (or existing device interacting with the new
    // battery ties up the SMBus line (usually
    // seen after the robot is driven for a short time.  The only
    // way I've been able to recover from this is by either removing
    // the SMBus cable between the battery board and the power board,
    // or by changing the i2c pins to outputs when the i2c module is
    // disabled.
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
