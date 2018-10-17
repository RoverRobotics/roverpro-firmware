#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "debug_uart.h"
#include "i2clib.h"
#include "device_robot_motor_i2c.h"

int I2C2XmitReset = 0;
int I2C3XmitReset = 0;

int I2C2TimerExpired = 0;
int I2C3TimerExpired = 0;

// A step may be reached by either falling through into it (first try)
// Or by jumping back to it (every retry)
// Note an i2c_result of I2C_ILLEGAL probably means there is a coding error, hence the breakpoint.
// If you want to check other I2C results, you can replace that check with (i2c_result != I2C_OKAY)
#define I2C_SUB(operationdef)                                                                      \
    op = (operationdef);                                                                           \
    progress = I2C_PROGRESS_UNSTARTED;                                                             \
    case (__LINE__):                                                                               \
        resume_at = (__LINE__);                                                                    \
        i2c_result = i2c_tick(BUS, &op, &progress);                                                \
        if (i2c_result == I2C_NOTYET) {                                                            \
            return;                                                                                \
        }                                                                                          \
        BREAKPOINT_IF(i2c_result == I2C_ILLEGAL);                                                  \
        BREAKPOINT_IF(i2c_result != I2C_OKAY);                                                     \
        // fallthrough to next case

void I2C2Update(void) {
    const i2c_bus_t BUS = I2C_BUS2;

    static i2c_operationdef_t op;
    static i2c_progress_t progress;

    static int resume_at = 0;
    static uint16_t a_word;
    static uint8_t a_byte;
    i2c_result_t i2c_result;

    if (I2C2XmitReset) {
        I2C2XmitReset = false; // clear the flag
        resume_at = 0;         // start from the beginning
    }
    switch (resume_at) {
    default:
        BREAKPOINT();
    // fallthrough
    case -1:
        re_init_i2c2();
        // fallthrough
    case 0:
        // REG_MOTOR_TEMP.left = FanControl ReadByte 0x00 [Temperature channel 1]
        I2C_SUB(i2c_op_read_byte(FAN_CONTROLLER_ADDRESS, 0x00, &a_byte))
        REG_MOTOR_TEMP_STATUS.left = (i2c_result == I2C_OKAY);
        REG_MOTOR_TEMP.left = a_byte;

        // REG_MOTOR_TEMP.right = FanControl ReadByte 0x01 [Temperature channel 2]
        I2C_SUB(i2c_op_read_byte(FAN_CONTROLLER_ADDRESS, 0x01, &a_byte))
        REG_MOTOR_TEMP_STATUS.right = (i2c_result == I2C_OKAY);
        REG_MOTOR_TEMP.right = a_byte;

        // FanControl WriteByte 0x0b
        if (Xbee_SIDE_FAN_NEW) {
            a_byte = Xbee_SIDE_FAN_SPEED;
        } else if (abs(Xbee_MOTOR_VELOCITY[0]) + abs(Xbee_MOTOR_VELOCITY[1]) > 10) {
            a_byte = Xbee_SIDE_FAN_SPEED = 240;
        } else {
            a_byte = Xbee_SIDE_FAN_SPEED = 0;
        }
        I2C_SUB(i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &a_byte));

        // REG_ROBOT_REL_SOC_A = Battery ReadWord 0x0d [="RelativeStateOfCharge"]
        I2C_SUB(i2c_op_read_word(BATTERY_ADDRESS, 0x0d, &a_word))
        REG_ROBOT_REL_SOC_A = a_word;

        // Battery ReadWord 0x16 [="BatteryStatus"]
        I2C_SUB(i2c_op_read_word(BATTERY_ADDRESS, 0x16, &a_word));
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_STATUS_A = a_word;
        }

        // Battery ReadWord 0x03 [="BatteryMode"]
        I2C_SUB(i2c_op_read_word(BATTERY_ADDRESS, 0x03, &a_word));
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_MODE_A = a_word;
        }

        // Battery ReadWord 0x08 [="Temperature"]
        I2C_SUB(i2c_op_read_word(BATTERY_ADDRESS, 0x08, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_TEMP_A = a_word;
        }

        // Battery ReadWord 0x09 [="Voltage"]
        STEP(i2c_start(BUS))
        STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
        STEP(i2c_write_byte(BUS, 0x09))
        STEP(i2c_restart(BUS))
        STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
        STEP(i2c_check_ack(BUS, &ack))
        STEP(i2c_receive(BUS))
        STEP(i2c_read_byte(BUS, ACK, &a))
        STEP(i2c_receive(BUS))
        STEP(i2c_read_byte(BUS, NACK, &b))
        if (ack == ACK) {
            REG_BATTERY_VOLTAGE_A = a + (b << 8);
        }
        STEP(i2c_stop(BUS))

        // Battery ReadWord 0x0a [="Current"]
        STEP(i2c_start(BUS))
        STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
        STEP(i2c_write_byte(BUS, 0x0a))
        STEP(i2c_restart(BUS))
        STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
        STEP(i2c_check_ack(BUS, &ack))
        STEP(i2c_receive(BUS))
        STEP(i2c_read_byte(BUS, ACK, &a))
        STEP(i2c_receive(BUS))
        STEP(i2c_read_byte(BUS, NACK, &b))
        if (ack == ACK) {
            REG_BATTERY_CURRENT_A = a + (b << 8);
        }
        STEP(i2c_stop(BUS))

        I2C2TimerExpired = false; // reset the I2C2 update timer
        resume_at = -1;
    }
}

void I2C3Update(void) {
    const i2c_bus_t BUS = I2C_BUS3;

    static i2c_operationdef_t op;
    static i2c_progress_t progress;

    static int resume_at = 0;
    static uint16_t a_word;
    i2c_result_t i2c_result;

    if (I2C3XmitReset) {
        I2C3XmitReset = false; // clear the flag
        resume_at = 0;         //  start from the beginning
    }
    switch (resume_at) {
    default:
        BREAKPOINT();
    // fallthrough
    case -1:
        re_init_i2c3();
        // fallthrough
    case 0:

        // Battery ReadWord 0x0d [="RelativeStateOfCharge"]
        I2C_SUB(i2c_op_read_word(BATTERY_ADDRESS, 0x0d, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_ROBOT_REL_SOC_B = a_word;
        }

        // BatteryCharger ReadWord 0xca
        // Note the battery charger is expected to be unreachable except while charging
        I2C_SUB(i2c_op_read_word(BATTERY_CHARGER_ADDRESS, 0xca, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_MOTOR_CHARGER_STATE = a_word;
        }

        // Battery ReadWord 0x16 [="BatteryStatus"]
        I2C_SUB(i2c_op_read_word(BATTERY_ADDRESS, 0x16, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_STATUS_B = a_word;
        }

        // Battery ReadWord 0x03 [="BatteryMode"]
        I2C_SUB(i2c_op_read_word(BATTERY_ADDRESS, 0x03, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_MODE_B = a_word;
        }

        // Battery ReadWord 0x08 [="Temperature"]
        I2C_SUB(i2c_op_read_word(BATTERY_ADDRESS, 0x08, &a_word))
        if (i2c_result == I2C_OKAY) {
            REG_BATTERY_TEMP_B = a_word;
        }

        // Battery ReadWord 0x09 [="Voltage"]
        STEP(i2c_start(BUS))
        STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
        STEP(i2c_write_byte(BUS, 0x09))
        STEP(i2c_restart(BUS))
        STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
        STEP(i2c_check_ack(BUS, &ack))
        STEP(i2c_receive(BUS))
        STEP(i2c_read_byte(BUS, ACK, &a))
        STEP(i2c_receive(BUS))
        STEP(i2c_read_byte(BUS, NACK, &b))
        if (ack == ACK) {
            REG_BATTERY_VOLTAGE_B = a + (b << 8);
        }
        STEP(i2c_stop(BUS))

        // Battery ReadWord 0x0a [="Current"]
        STEP(i2c_start(BUS))
        STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
        STEP(i2c_write_byte(BUS, 0x0a))
        STEP(i2c_restart(BUS))
        STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
        STEP(i2c_check_ack(BUS, &ack))
        STEP(i2c_receive(BUS))
        STEP(i2c_read_byte(BUS, ACK, &a))
        STEP(i2c_receive(BUS))
        STEP(i2c_read_byte(BUS, NACK, &b))
        if (ack == ACK) {
            REG_BATTERY_CURRENT_B = a + (b << 8);
        }
        STEP(i2c_stop(BUS))

        I2C3TimerExpired = false; // reset the I2C3 update timer
        resume_at = -1;
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
