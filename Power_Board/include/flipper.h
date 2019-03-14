/// @file
/// Translate flipper sensors (potentiometers) from independent readings into a meaningful angular
/// position measure

#ifndef FLIPPER_H
#define FLIPPER_H
#include "stdint.h"

/// Figures out the math needed to interpret potentiometer readings.
/// Saves this in Settings in NVM, then idles until the rover is turned off.
__attribute__((noreturn)) void flipper_feedback_calibrate();

/// Computes the actual flipper value and stores the result in _state.drive.flipper_angle
void tick_flipper_feedback();

#endif