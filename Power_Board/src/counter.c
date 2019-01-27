#include "counter.h"

void counter_resume(Counter *c) { c->is_paused = false; }
void counter_restart(Counter *c) {
    c->is_paused = false;
    c->count = 0;
}
void counter_stop(Counter *c) {
    c->is_paused = true;
    c->count = 0;
}
CounterState counter_tick(Counter *c) {
    if (c->is_paused)
        return COUNTER_PAUSED;
    c->count++;
    if (c->count < c->max) {
        return COUNTER_RUNNING;
    } else {
        if (c->pause_on_expired)
            counter_stop(c);
        else
            counter_restart(c);
        return COUNTER_EXPIRED;
    }
}
