#ifndef COUNTER_H
#define COUNTER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum CounterState {
  COUNTER_RUNNING,
  COUNTER_PAUSED,
  COUNTER_EXPIRED,
} CounterState;

typedef struct Counter {
  const uint16_t max;
  const bool pause_on_expired : 1;
  bool is_paused : 1;
  uint16_t count;
} Counter;

void counter_resume(Counter *c);
void counter_restart(Counter *c);
void counter_stop(Counter *c);
/// Increment the given counter, if it's not paused.
CounterState counter_tick(Counter *c);

#endif //COUNTER_H
