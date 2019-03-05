#ifndef ANALOG_H
#define ANALOG_H

#include "stdint.h"

typedef enum {
    ADC_MOTOR_LEFT_CURRENT = 0,
    ADC_MOTOR_RIGHT_CURRENT,
    ADC_MOTOR_FLIPPER_CURRENT,

    ADC_CELL_A_CURRENT,
    ADC_CELL_B_CURRENT,

    ADC_CELL_A_VOLTAGE,
    ADC_CELL_B_VOLTAGE,

    ADC_FLIPPER_POTENTIOMETER_A,
    ADC_FLIPPER_POTENTIOMETER_B,
} AnalogChannel;
#define ANALOG_CHANNEL_COUNT 9

void analog_init();
void analog_tick();

/// Get the smoothed value from the given analog channel
/// We use a simple windowed average of the last 4 samples.
uint16_t analog_get_value(AnalogChannel c);

#endif