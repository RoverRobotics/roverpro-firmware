#include "communication.h"
#include "bytequeue.h"
#include "clock.h"
#include "cooling.h"
#include "drive.h"
#include "flipper.h"
#include "hardware_definitions.h"
#include "main.h"
#include "settings.h"
#include <math.h>
#include <xc.h>

#define RX_PACKET_SIZE 7
#define TX_PACKET_SIZE 5

#define settings g_settings.communication
#define state g_state.communication

/// In the OpenRover protocol, this byte signifies the start of a message
const uint8_t UART_START_BYTE = 253;

/// Checksum of a packet. An OpenRover packet should apply this to the payload
/// (contents of the message, excluding the start byte). It is calculated as
/// 255 - (sum of bytes % 255).
// TODO: this is a lousy checksum. Use a better checksum.
uint8_t checksum(size_t count, const uint8_t *data) {
    size_t i;
    uint16_t total = 0;
    for (i = 0; i < count; i++) {
        total += data[i];
    }
    uint8_t result = 255 - (total % 255);
    return result;
}

void uart_init() {
    U1MODEbits.UARTEN = 0;

    bq_try_resize(&g_state.communication.rx_q, g_settings.communication.rx_bufsize_bytes);
    bq_try_resize(&g_state.communication.tx_q, g_settings.communication.tx_bufsize_bytes);

    // Assign U1RX To U1RX, Uart receive channnel
    _U1RXR = U1RX_RPn;
    // Assign U1TX To U1TX/Uart Tx
    U1TX_RPn = 3; // 3 represents U1TX

    // Enable the UART.
    U1MODE = 0x0000;
    uint32_t baud_rate = g_settings.communication.baud_rate;
    if (baud_rate < FCY / 4.0F) {
        U1MODEbits.BRGH = 0; // High Baud Rate Select bit = off
        U1BRG = FCY / 16 / baud_rate - 1;
    } else {
        U1MODEbits.BRGH = 1;
        U1BRG = FCY / 4 / baud_rate - 1;
    }
    U1MODEbits.UARTEN = 1; // UART1 is enabled

    U1STAbits.UTXISEL1 = 1; // Call transmit interrupt when the transmit buffer is empty
    U1STAbits.UTXISEL0 = 0; // Call transmit interrupt when the transmit buffer is empty
    U1STAbits.UTXEN = 1;    // enable transmit
    IEC0bits.U1TXIE = 1;    // enable UART1 transmit interrupt
    IEC0bits.U1RXIE = 1;    // enable UART1 receive interrupt
}

/// UART receive Interrupt function
/// Transfer inbound data from the UART hardware buffer into a software buffer
/// Called by hardware interrupt and enabled by _U1RXIE
void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt() {
    // clear the interrupt flag
    _U1RXIF = 0;
    // While new received data is available and we have somewhere to put it
    while (U1STAbits.URXDA) {
        uint8_t a_byte = (uint8_t)U1RXREG;
        // Pop the next byte off the read queue
        bq_try_push(&g_state.communication.rx_q, 1, &a_byte);
    }
    // if receiver overflowed this resets the module
    if (U1STAbits.OERR) {
        // clear overflow error bit if it was set.
        U1STAbits.OERR = 0;
    }
}

/// UART transmit Interrupt function
/// Transfer outbound data from a software buffer into the UART hardware buffer
/// called by hardware interrupt and enabled by _U1TXIE
void __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt() {
    // clear the interrupt flag
    _U1TXIF = 0;
    // transmit data
    while (U1STAbits.UTXBF == 0) { // while transmit shift buffer is not full
        uint8_t a_byte;
        if (bq_try_pop(&g_state.communication.tx_q, 1, &a_byte)) {
            U1TXREG = a_byte;
        } else {
            return;
        }
    }
}

void uart_serialize_out_data(uint8_t *out_bytes, uint8_t uart_data_identifier) {
    uint16_t intval;

#define CASE(n, REGISTER)                                                                          \
    case (n):                                                                                      \
        intval = (uint16_t)(REGISTER);                                                             \
        out_bytes[0] = (intval >> 8);                                                              \
        out_bytes[1] = (intval);                                                                   \
        break;

    switch (uart_data_identifier) {
        CASE(0, g_state.analog.battery_current[0] + g_state.analog.battery_current[1])
        // CASE(2, REG_MOTOR_FB_RPM.left)
        // CASE(4, REG_MOTOR_FB_RPM.right)
        CASE(6, g_state.analog.flipper_sensors[0])
        CASE(8, g_state.analog.flipper_sensors[1])
        CASE(10, g_state.analog.motor_current[MOTOR_LEFT])
        CASE(12, g_state.analog.motor_current[MOTOR_RIGHT])
        CASE(14, g_state.drive.motor_encoder_count[MOTOR_LEFT])
        CASE(16, g_state.drive.motor_encoder_count[MOTOR_RIGHT])
        // case 18 deprecated
        CASE(20, g_state.i2c.temperature_sensor[0])
        CASE(22, g_state.i2c.temperature_sensor[1])
        CASE(24, g_state.analog.battery_voltage[BATTERY_A])
        CASE(26, g_state.analog.battery_voltage[BATTERY_B])
        CASE(28, g_state.drive.motor_encoder_period[MOTOR_LEFT])
        CASE(30, g_state.drive.motor_encoder_period[MOTOR_RIGHT])
        CASE(32, g_state.drive.motor_encoder_period[MOTOR_FLIPPER])
        CASE(34, g_state.i2c.smartbattery_soc[BATTERY_A])
        CASE(36, g_state.i2c.smartbattery_soc[BATTERY_B])
        CASE(38, g_state.i2c.charger_state)
        CASE(40, g_settings.firmware.release_version_flat)
        CASE(42, g_state.analog.battery_current[BATTERY_A])
        CASE(44, g_state.analog.battery_current[BATTERY_B])
        CASE(46, g_state.drive.flipper_angle)
        // CASE(48, g_state.communication.fan_speed)
        // CASE(50, REG_MOTOR_CLOSED_LOOP)
        CASE(52, g_state.i2c.smartbattery_status[BATTERY_A])
        CASE(54, g_state.i2c.smartbattery_status[BATTERY_B])
        CASE(56, g_state.i2c.smartbattery_mode[BATTERY_A])
        CASE(58, g_state.i2c.smartbattery_mode[BATTERY_B])
        CASE(60, g_state.i2c.smartbattery_temperature[BATTERY_A])
        CASE(62, g_state.i2c.smartbattery_temperature[BATTERY_B])
        CASE(64, g_state.i2c.smartbattery_voltage[BATTERY_A])
        CASE(66, g_state.i2c.smartbattery_voltage[BATTERY_B])
        CASE(68, g_state.i2c.smartbattery_current[BATTERY_A])
        CASE(70, g_state.i2c.smartbattery_current[BATTERY_B])
        CASE(72, g_state.drive.motor_status[MOTOR_LEFT])
        CASE(74, g_state.drive.motor_status[MOTOR_RIGHT])
        CASE(76, g_state.drive.motor_status[MOTOR_FLIPPER])
        CASE(78, g_state.i2c.fan_duty[0])
        CASE(80, g_state.i2c.fan_duty[1])
        CASE(82, get_system_fault())
    default:
        break;
    }
#undef CASE
}

float motor_uint8_to_float(uint8_t value) { return ((int8_t)(value - 125)) * (1 / 125.F); }

void uart_tick() {
    bool has_drive_command = false;

    // // debug:
    // UARTCommand test_verb = UART_COMMAND_NONE;
    // uint8_t test_arg = 0;
    // uint8_t rq_test_msg[7] = {253, 135, 135, 125, test_verb, test_arg, 0};
    // rq_test_msg[6] = checksum(5, rq_test_msg + 1);
    // bq_try_push(&g_state.communication.rx_q, sizeof(rq_test_msg), rq_test_msg);
    // // end debug

    while (bq_can_pop(&g_state.communication.rx_q, RX_PACKET_SIZE)) {
        uint8_t packet[RX_PACKET_SIZE] = {0};
        bq_try_pop(&g_state.communication.rx_q, 1, packet);
        if (packet[0] != UART_START_BYTE)
            continue;

        bq_try_pop(&g_state.communication.rx_q, RX_PACKET_SIZE - 1, packet + 1);
        uint8_t expected_checksum = checksum(RX_PACKET_SIZE - 2, packet + 1);
        if (expected_checksum != packet[RX_PACKET_SIZE - 1]) {
            // checksum mismatch. discard.
            continue;
        }

        g_state.communication.drive_command_timestamp = clock_now();
        g_state.communication.motor_effort[MOTOR_LEFT] = motor_uint8_to_float(packet[1]);
        g_state.communication.motor_effort[MOTOR_RIGHT] = motor_uint8_to_float(packet[2]);
        g_state.communication.motor_effort[MOTOR_FLIPPER] = motor_uint8_to_float(packet[3]);

        has_drive_command = true;

        UARTCommand verb = packet[4];
        uint8_t arg = packet[5];

        switch (verb) {
        case UART_COMMAND_RESTART:
            __asm__ volatile("reset");
            break;
        case UART_COMMAND_FLIPPER_CALIBRATE:
            if (arg == UART_COMMAND_FLIPPER_CALIBRATE) {
                // note flipper calibration never returns
                flipper_feedback_calibrate();
            }
            break;
        case UART_COMMAND_GET: {
            uint8_t out_packet[5];
            out_packet[0] = UART_START_BYTE;
            out_packet[1] = arg;
            uart_serialize_out_data(out_packet + 2, arg);
            out_packet[4] = checksum(3, out_packet + 1);
            if (!bq_try_push(&g_state.communication.tx_q, TX_PACKET_SIZE, out_packet)) {
                BREAKPOINT();
            }
            // Start transmission
            _U1TXIF = 1;
        } break;
        case UART_COMMAND_SETTINGS_RELOAD:
            g_settings = settings_load();
            break;
        case UART_COMMAND_SETTINGS_COMMIT:
            settings_save(&g_settings);
            break;
        case UART_COMMAND_SETTINGS_SET_POWER_POLL_INTERVAL:
            g_settings.main.power_poll_ms = arg;
            break;
        case UART_COMMAND_SETTINGS_SET_OVERCURRENT_TRIGGER_THRESHOLD:
            g_settings.power.overcurrent_trigger_threshold_ma = arg * 100;
            break;
        case UART_COMMAND_SETTINGS_SET_OVERCURRENT_TRIGGER_DURATION:
            g_settings.power.overcurrent_trigger_duration_ms = arg * 5;
            break;
        case UART_COMMAND_SETTINGS_SET_OVERCURRENT_RECOVER_THRESHOLD:
            g_settings.power.overcurrent_trigger_threshold_ma = arg * 100;
            break;
        case UART_COMMAND_SETTINGS_SET_OVERCURRENT_RECOVER_DURATION:
            g_settings.power.overcurrent_trigger_threshold_ma = arg * 5;
            break;
        case UART_COMMAND_SETTINGS_SET_PWM_FREQUENCY_KHZ:
            g_settings.drive.motor_pwm_frequency_hz = (float)arg * 0.001F;
            drive_init();
        case UART_COMMAND_SETTINGS_SET_PWM_FREQUENCY_HHZ:
            g_settings.drive.motor_pwm_frequency_hz = (float)arg * 0.01F;
            drive_init();
            break;
        case UART_COMMAND_SETTINGS_SET_BRAKE_ON_ZERO_SPEED_COMMAND:
            g_settings.drive.brake_on_zero_speed_command = (bool)arg;
            break;
        case UART_COMMAND_SETTINGS_SET_BRAKE_ON_DRIVE_TIMEOUT:
            g_settings.drive.brake_on_drive_timeout = (bool)arg;
            break;
        case UART_COMMAND_SETTINGS_SET_MOTOR_SLOW_DECAY_MODE:
            g_settings.drive.motor_slow_decay_mode = (bool)arg;
            break;
        case UART_COMMAND_SETTINGS_SET_TIME_TO_FULL_SPEED_DECISECONDS:
            g_settings.drive.time_to_full_speed = (float)arg * 0.1F;
            break;
        case UART_COMMAND_SETTINGS_SET_OVERSPEED_ENCODER_THRESHOLD_ENCODER_HHZ:
            g_settings.drive.hi_speed_encoder_hz = (float)arg * 10.0F;
            break;
        case UART_COMMAND_SETTINGS_SET_OVERSPEED_DURATION_DS:
            g_settings.drive.overspeed_fault_trigger_s = (float)arg * 0.1F;
            break;
        case UART_COMMAND_CLEAR_SYSTEM_FAULT:
            clear_system_fault();
            break;
        case UART_COMMAND_SETTINGS_SET_BRAKE_ON_FAULT:
            g_settings.drive.brake_on_fault = (bool)arg;
            break;
        default:
            // unknown or deprecated inbound command.
            break;
        }
    }

    _U1TXIF = 1;
}
