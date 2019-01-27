#ifndef UART_CONTROL_H
#define UART_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/// Commanded instruction as received over UART. Most of these commands have
/// additional data associated with them which are stored in shared globals.
/// These are not mutually exclusive.
typedef struct UARTTickResult {
    /// We have been requested to calibrate the flipper positional sensors
    bool uart_flipper_calibrate_requested;
    /// We have been commanded to change the fan speed (to the value in @ref
    /// REG_MOTOR_SIDE_FAN_SPEED)
    bool uart_fan_speed_requested;
    /// We have been commanded to adjust the motor speed (to the values in @ref REG_MOTOR_VELOCITY)
    bool uart_motor_speed_requested;
    /// We have been commanded to change the motor control scheme (to the value in @ref
    /// REG_MOTOR_CLOSED_LOOP)
    bool uart_motor_control_scheme_requested;
} UArtTickResult;

/// Initialize UART module
void uart_init();

/// If we have received data over UART, act on it.
/// May clear the inbound software buffer and populate the outbound software buffer
UArtTickResult uart_tick();

/// UART transmit Interrupt function
/// Transfer outbound data from a software buffer into the UART hardware buffer
void uart_tx_isf();

/// UART receive Interrupt function
/// Transfer inbound data from the UART hardware buffer into a software buffer
void uart_rx_isf();

#endif
