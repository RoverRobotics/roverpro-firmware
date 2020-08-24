/// @file
/// Cooling fan control

#pragma once
#include <stdint.h>

extern const uint8_t FAN_DUTY_RATE_OF_CHANGE_INSTANTANEOUS;
extern const uint8_t FAN_DUTY_RATE_OF_CHANGE_DEFAULT;
extern const uint8_t FAN_DUTY_MAX;

void cooling_blast_fan();
void cooling_init();