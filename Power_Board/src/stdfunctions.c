#include "stdhdr.h"

// built in delay function
extern void __delay32(unsigned long cycles);
void delay32wd(uint32_t cycles) {
    __builtin_clrwdt();
    // this should be long enough to not trigger the WDT
    const uint32_t chunk = 1000;
    if (cycles < chunk) {
        __delay32(cycles);
    } else {
        uint32_t n_chunks = cycles / chunk;
        uint32_t i;

        __builtin_clrwdt();
        for (i = 0; i < n_chunks; i++) {
            __delay32(chunk);
            __builtin_clrwdt();
        }
        __delay32(cycles - n_chunks * chunk);
        __builtin_clrwdt();
    }
    __builtin_clrwdt();
}

void block_ms(uint32_t ms) { delay32wd(ms * (FCY / 1000ULL)); }

void block_us(uint32_t us) { delay32wd(us * (FCY / 1000000ULL)); }

void block_s(float s) { delay32wd((uint32_t)(s * FCY)); }