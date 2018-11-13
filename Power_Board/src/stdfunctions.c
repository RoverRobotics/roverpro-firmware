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

int16_t clamp(int16_t value, int16_t lo, int16_t hi) {
    return (value < lo ? lo : value > hi ? hi : value);
}
float clamp_f(float value, float lo, float hi) {
    return (value < lo ? lo : value > hi ? hi : value);
}

int16_t mean(size_t count, int16_t *values) {
    int32_t total = 0;
    size_t i;
    for (i = 0; i < count; i++) {
        total += values[i];
    }
    return total / count;
}

uint16_t mean_u(size_t count, uint16_t *values) {
    // Note: I would total into a long long instead,
    // but performance was surprisingly bad (560 us for division) on PIC24
    uint32_t total = 0;
    size_t i;
    for (i = 0; i < count; i++) {
        total += values[i];
    }
    total = total / count;
    return total;
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
