#include "clock.h"
#include "stdhdr.h"
#include <xc.h>

/// Number of clock periods
static volatile uint64_t g_timer_overflow = 0;

/// Initialize the clock. Since this is a 16-bit timer running with 1:1 prescale,
/// it will overflow every 2**16 / FCY s = 4.1 ms.
void clock_init(void) {
    g_timer_overflow = 0;
    T2CON = 0x00; // clear timer configuration
    TMR2 = 0;
    PR2 = UINT16_MAX; // set timer maximum count.
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    T2CONbits.TON = 1; // turn on the timer
}

int64_t clock_now() {
    // We want a 64-bit clock, but the hardware clock is only 16 bit (or 32 if we use two)
    // Instead, we just use a 16-bit clock and count up for the higher bits.

    IEC0bits.T2IE = 0; // briefly disable timer overflow interrupt
    // note that TMR2 is still counting up!
    uint16_t ticks_lo = TMR2;
    uint64_t ticks_hi;
    if (IFS0bits.T2IF && ticks_lo < 0xFFU) {
        // overflow happened but not yet represented in g_timer_overflow
        ticks_hi = ++g_timer_overflow;
         IFS0bits.T2IF = 0;
    } else {
        ticks_hi = g_timer_overflow;
    }
    IEC0bits.T2IE = 1;

    return (ticks_hi << 16U) | ticks_lo;
}

int64_t seconds_to_ticks(float seconds) { return seconds * FCY; }

float ticks_to_seconds(int64_t ticks) { return ((float)ticks) / FCY; }

void __attribute((__interrupt__, no_auto_psv)) _T2Interrupt() {
    g_timer_overflow++;
    IFS0bits.T2IF = 0;
}