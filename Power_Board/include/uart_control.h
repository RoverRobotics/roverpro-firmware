#ifndef UART_CONTROL_H
#define UART_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define UART_CONTROL

extern bool uart_flipper_calibrate_requested;
extern bool uart_has_new_fan_speed;
extern bool uart_has_new_data;

/// initialize UART module
void uart_init();
/// If we have received data over UART, act on it.
/// May clear the inbound software buffer and populate the outbound software buffer
void uart_tick();
/// UART transmit Interrupt function
/// Transfer outbound data from a software buffer into the UART hardware buffer
void uart_tx_isf();
/// UART receive Interrupt function
/// Transfer inbound data from the UART hardware buffer into a software buffer
void uart_rx_isf();

#endif