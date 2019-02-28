#include "stdhdr.h"

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