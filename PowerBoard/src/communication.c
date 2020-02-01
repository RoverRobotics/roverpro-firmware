#include "communication.h"
#include "bytequeue.h"
#include "clock.h"
#include "cooling.h"
#include "drive.h"
#include "flipper.h"
#include "hardware_definitions.h"
#include "main.h"
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

    free(state.overspeed_fault_times);
    state.overspeed_fault_times = calloc(settings.overspeed_runaway_limit, sizeof(uint64_t));
}

/// There are two types of overspeed conditions:
/// An overspeed fault is when we send too much effort to the motors for too long.
/// An overspeed runaway is when too many faults happen in too short a time.
bool should_obey_motor_commands(float left_effort, float right_effort) {
    uint64_t now = clock_now();

    float requested_effort = max(fabsf(left_effort), fabsf(right_effort));

    if (requested_effort <= settings.overspeed_runaway_reset_effort) {
        // we are stopped or nearly so.
        state.overspeed_runaway_reset_time = now;
        state.overspeed_fault_time = 0;
        state.overspeed_time = 0;
        return true;
    } else if (state.overspeed_runaway_reset_time < state.overspeed_runaway_time) {
        // we are in a runaway condition and the driver needs to
        // send a low speed before we obey again
        return false;
    } else if (
        now < state.overspeed_fault_time + seconds_to_ticks(settings.overspeed_fault_recover_s)) {
        // we are in a fault condition. Don't obey yet
        return false;
    } else if (requested_effort < settings.overspeed_fault_effort) {
        // we are not in a runaway condition and the driver has sent a low speed;
        state.overspeed_time = 0;
        return true;
    } else if (!state.overspeed_time) {
        // we've only been going fast for a short time. Allow it, but note the time.
        state.overspeed_time = now;
        return true;
    } else if (now < state.overspeed_time + seconds_to_ticks(settings.overspeed_fault_trigger_s)) {
        // we've only been going fast for a short time. Allow it for now...
        return true;
    } else {
        // uh oh. We've been going fast for a while now.
        size_t i;
        bool tmp_runaway = true;
        for (i = 0; i < settings.overspeed_runaway_limit; i++) {
            // if the value is before the most recent reset or too old, overwrite it.
            if (state.overspeed_fault_times[i] <= state.overspeed_runaway_reset_time ||
                state.overspeed_fault_times[i] +
                        seconds_to_ticks(settings.overspeed_runaway_history_s) <
                    now) {
                state.overspeed_fault_times[i] = now;
                tmp_runaway = false;
                break;
            }
        }

        // so we don't trigger a fault condition immediately on ending this oone.
        state.overspeed_time = 0;
        if (tmp_runaway) {
            state.overspeed_runaway_time = now;
        } else {
            state.overspeed_fault_time = now;
        }

        return false;
    }
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
        CASE(78, g_state.i2c.fan_target_duty[0])
        CASE(80, g_state.i2c.fan_target_duty[1])
    default:
        break;
    }
#undef CASE
}

static unsigned g_ustep = 0;

void uart_tick() {
    bool has_drive_command = false;

    // debug:
    /*
        {
        BREAKPOINT();
        UARTCommand test_verb = UART_COMMAND_NONE;
        uint8_t test_arg = 0;
        uint8_t rq_test_msg[7] = {253, 125, 125, 125, test_verb, test_arg, 0};
        rq_test_msg[6] = checksum(5, rq_test_msg + 1);

        bq_try_push(&g_state.communication.rx_q, sizeof(rq_test_msg), rq_test_msg);
        }
    */
    // end debug

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

        float tmp_left = (float)packet[1] / 125.0F - 1.0F;
        float tmp_right = (float)packet[2] / 125.0F - 1.0F;
        if (should_obey_motor_commands(tmp_left, tmp_right)) {
            g_state.communication.drive_command_timestamp = clock_now();
            g_state.communication.motor_effort[MOTOR_LEFT] = tmp_left;
            g_state.communication.motor_effort[MOTOR_RIGHT] = tmp_right;
            g_state.communication.motor_effort[MOTOR_FLIPPER] = (float)packet[3] / 125.0F - 1.0F;
            has_drive_command = true;
        }

        UARTCommand verb = packet[4];
        uint8_t arg = packet[5];

        switch (verb) {
        case UART_COMMAND_RESTART:
            __asm__ volatile("reset");
            break;
        case UART_COMMAND_FLIPPER_CALIBRATE:
            if (arg == UART_COMMAND_FLIPPER_CALIBRATE) {
                // note flipper calibration never returns.
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
            g_settings.communication.brake_on_zero_speed_command = (bool)arg;
            break;
        case UART_COMMAND_SETTINGS_SET_BRAKE_ON_DRIVE_TIMEOUT:
            g_settings.communication.brake_on_drive_timeout = (bool)arg;
            break;
        case UART_COMMAND_SETTINGS_SET_MOTOR_SLOW_DECAY_MODE:
            g_settings.drive.motor_slow_decay_mode = (bool)arg;
            break;
        case UART_COMMAND_SETTINGS_SET_TIME_TO_FULL_SPEED_DECISECONDS:
            g_settings.drive.time_to_full_speed = (float)arg * 0.1F;
            break;
        case UART_COMMAND_SETTINGS_SET_SPEED_LIMIT_PERCENT:
            g_settings.communication.overspeed_fault_effort = (float)arg * 0.01F;
        case UART_COMMAND_SET_DRIVE_MODE:
            // fallthrough
        default:
            // unknown inbound command.
            break;
        }
    }

    if (has_drive_command) {
        MotorChannel c;
        for (EACH_MOTOR_CHANNEL(c)) {
            g_state.communication.brake_when_stopped =
                g_settings.communication.brake_on_zero_speed_command;
        }
    } else if (
        clock_now() - g_state.communication.drive_command_timestamp >
        g_settings.communication.drive_command_timeout_ms * seconds_to_ticks(0.001F)) {
        // long time no motor commands. stop moving.
        MotorChannel c;
        for (EACH_MOTOR_CHANNEL(c)) {
            g_state.communication.motor_effort[c] = 0;
            g_state.communication.brake_when_stopped =
                g_settings.communication.brake_on_drive_timeout;
        }
    }

    _U1TXIF = 1;
}
