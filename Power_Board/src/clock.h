#include "stdint.h"

/// Set up timers for the high-precision timestamp
void clock_init();

/// Get a high-precision timestamp
/// @returns 64-bit timestamp since startup
uint64_t clock_now();

/// number of clock ticks per millisecond
#define CLOCK_MS ((uint64_t)FCY / 1000)

/// number of clock ticks per microsecond
#define CLOCK_US ((uint64_t)FCY / 1000000UL)

/// number of clock ticks per second
#define CLOCK_S ((uint64_t)FCY)