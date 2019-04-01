#include "stdint.h"
void clock_init();
uint16_t clock_ticks_u16();
uint64_t clock_ticks();

/// number of clock ticks per millisecond
#define TICKS_PER_MS (FCY / 1000UL)

/// number of clock ticks per microsecond
#define TICKS_PER_US (FCY / 1000000UL)

/// number of clock ticks per second
#define TICKS_PER_S (FCY)