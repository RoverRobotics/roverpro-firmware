#pragma once

#include "stdhdr.h"
#include <stdint.h>

/// Set up timers for the high-precision timestamp
void clock_init();

/// Get a high-precision timestamp
/// @returns 64-bit timestamp since startup
int64_t clock_now();

int64_t seconds_to_ticks(float seconds);
float ticks_to_seconds(int64_t ticks);
