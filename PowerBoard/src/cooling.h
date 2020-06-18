/// @file
/// Cooling fan control

#ifndef COOLING_H
#define COOLING_H
#include <stdint.h>

extern const uint8_t FAN_DUTY_RATE_OF_CHANGE_INSTANTANEOUS;
extern const uint8_t FAN_DUTY_RATE_OF_CHANGE_DEFAULT;
extern const uint8_t FAN_DUTY_MAX;

void cooling_blast_fan();
void cooling_init();
#endif