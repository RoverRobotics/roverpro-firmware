/**
 * @file stdfunctions.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 *
 */
#include "stdhdr.h"

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

// built in delay function
extern void __delay32(unsigned long cycles);
#define __delay_ms(d)                                                                              \
    { __delay32((unsigned long)(((unsigned long long)d) * (FCY) / 1000ULL)); }

void block_ms(uint16_t ms) {
    const int ms_chunk = 128;
    int i;
    __builtin_clrwdt();

    for (i = 0; i < (ms / ms_chunk); i++) {
        __delay_ms(ms_chunk);
        __builtin_clrwdt();
    }

    __delay_ms(ms % ms_chunk);
    __builtin_clrwdt();
}

extern int abs(int x) { return x >= 0 ? x : -x; }
