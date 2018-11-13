#ifndef UART_CONTROL_H
#define UART_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

typedef struct UARTTickResult {
  bool uart_flipper_calibrate_requested;
  bool uart_fan_speed_requested;
  bool uart_motor_speed_requested;
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
