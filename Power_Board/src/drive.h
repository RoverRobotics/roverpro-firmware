/// @file
/// Controls rover driving behavior.

#ifndef DRIVE_H
#define DRIVE_H
#include "stdbool.h"

void drive_init();
void drive_tick();
void drive_set_coast_lock(bool is_on);

#endif