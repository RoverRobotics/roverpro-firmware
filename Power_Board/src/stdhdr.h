/// @file
/// Common declarations.

#ifndef STDHDR_H
#define STDHDR_H

#include "xc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/// Instruction clock frequency in Hz
#define FCY (16000000UL)

/// When built in release mode, does nothing.
/// When built in DEBUG, if a PICkit is attached, halts execution. Note the behavior of timers and
/// peripherals can be configgered in the IDE. When built in DEBUG, if no PICkit is attached,
/// restarts execution.
// The NOP is important. It prevents a problem with `switch{default:  BREAKPOINT()}`, which would
// otherwise crash. It also allows the debugger to stop on this line instead of overshooting.
#if (__DEBUG)
#define BREAKPOINT()                                                                               \
    {                                                                                              \
        __builtin_software_breakpoint();                                                           \
        __builtin_nop();                                                                           \
    }
#else
#define BREAKPOINT() ((void)0)
#endif

/// Conditional breakpoint
#define BREAKPOINT_IF(condition)                                                                   \
    if (condition) {                                                                               \
        BREAKPOINT();                                                                              \
    }

/// Block for the specified amount of time. Periodically resets the Watchdog Timer so a long wait
/// doesn't trigger a reset
void block_ms(uint32_t ms);
void block_us(uint32_t us);
void block_s(float seconds);
#endif
