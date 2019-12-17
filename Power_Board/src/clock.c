#include "clock.h"
#include "stdhdr.h"
#include <xc.h>

/// Number of clock periods
static volatile uint64_t g_timer_overflow = UINT64_MAX;

/// Initialize the clock. Since this is a 16-bit timer running with 1:1 prescale,
/// it will overflow every 2**16 / FCY s = 4.1 ms
void clock_init(void) {
    g_timer_overflow = 0;
    T2CON = 0x00; // clear timer configuration
    TMR2 = 0;
    PR2 = UINT16_MAX; // set timer maximum count.
    _T2IF = 0;
    _T2IE = 1;
    T2CONbits.TON = 1; // turn on the 32-bit timer
}

uint64_t clock_now() {
    BREAKPOINT_IF(g_timer_overflow == UINT64_MAX);
    uint16_t ticks1 = TMR2;
    uint64_t oflow1 = g_timer_overflow;
    uint16_t ticks2 = TMR2;

    if (ticks1 < ticks2) {
        // no overflow happened while retrieving oflow1
        return (oflow1 << 16) | ticks1;
    } else if (_T2IF) {
        // overflow happened while retrieving ticks1
        // and the ISF has not yet run
        return ((g_timer_overflow + 1) << 16) | ticks2;
    } else {
        // overflow happened while retrieving ticks1,
        // but the ISF did run
        return (g_timer_overflow << 16) | ticks2;
    }
}

void __attribute((__interrupt__, auto_psv)) _T2Interrupt() {
    g_timer_overflow++;
    _T2IF = 0;
}