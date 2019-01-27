/// @file
/// Defines a counter abstraction used mostly for scheduling.
/// The main event loop runs on a synchronous tick, so we can do things like
/// ```
/// Counter my_counter = {.max=3}
/// while (...) {
///    if (counter_tick(&my_counter) == COUNTER_EXPIRED) {
///      // ... do some work once every 3 iterations of the outer loop
///    }
/// }
/// ```

#ifndef COUNTER_H
#define COUNTER_H

#include <stdint.h>
#include <stdbool.h>

/// An abstraction for counting up, used mostly for scheduling.
typedef struct Counter {
    /// The maximum value this counter will attain.
    const uint16_t max;
    /// True if this counter should pause when it hits the maximum.
    /// otherwise, @ref count loops back to zero.
    const bool pause_on_expired : 1;
    /// true if the counter is paused and @ref counter_tick should do nothing.
    bool is_paused : 1;
    /// The current value of this counter. Should be less than max.
    uint16_t count;
} Counter;

/// The current state of this counter. Returned by @ref counter_tick
typedef enum CounterState {
    /// The counter is paused and the last tick did nothing
    COUNTER_PAUSED,
    /// The counter has incremented
    COUNTER_RUNNING,
    /// This counter has hit the end. Caller should do whatever the counter is counting up to
    /// The counter will either restart or pause on the next tick, based on the vaule of @ref
    /// .pause_on_expired
    COUNTER_EXPIRED,
} CounterState;

/// The counter is now running and its count is unchanged.
void counter_resume(Counter *c);

/// The counter is now runinng and its count is zero.
void counter_restart(Counter *c);

/// The counter is now paused and its count is zero.
void counter_stop(Counter *c);

/// Increment the given counter, if it's not paused
/// @return The new state after this tick.
CounterState counter_tick(Counter *c);

#endif // COUNTER_H
