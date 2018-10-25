/**
 * @file stdfunctions.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 *
 */
#include "stdhdr.h"

// these two lines needed for __delay_us
#define FCY 16000000UL // instruction clock
#include "libpic30.h"

int clamp(int value, int lo, int hi) { return (value < lo ? lo : value > hi ? hi : value); }

int mean(size_t count, int *values) {
    long total = 0;
    size_t i;
    for (i = 0; i < count; i++) {
        total += values[i];
    }
    return total / count;
}

long mean_l(size_t count, long *values) {
    // Note: I would total into a long long instead, but performance was surprisingly bad on PIC24!
    long total = 0;
    size_t i;
    for (i = 0; i < count; i++) {
        total += values[i];
    }
    return total / count;
}

void block_ms(uint16_t ms) {
    const int ms_chunk = 128;
    int i;
    ClrWdt();

    for (i = 0; i < (ms / ms_chunk); i++) {
        __delay_ms(ms_chunk);
        ClrWdt();
    }

    __delay_ms(ms % ms_chunk);
    ClrWdt();
}
