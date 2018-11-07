#ifndef UART_CONTROL_H
#define UART_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define UART_CONTROL 1

bool uart_flipper_calibrate_requested = false;
bool uart_has_new_fan_speed = false;

bool uart_has_new_data = false;

void uart_init();
void uart_tick();
void uart_tx_isf();
void uart_rx_isf();

#endif