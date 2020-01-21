#include "stdhdr.h"

// built in delay function
extern void __delay32(unsigned long cycles);
void delay32wd(uint32_t cycles) {
    __builtin_clrwdt();
    // this should be short enough to not trigger the WDT
    // if WDT has a prescaler of 128 and the LPRC clock has 31 kHz frequency,
    // and the processor has a frequency FCY = 16 MHz
    // we can safely wait (128 / (31 kHz)) * 16MHz = 66 0000 ~= 2**16 cycles
    const uint32_t CHUNK = (1U << 14U);
    if (cycles < CHUNK) {
        __delay32(cycles);
    } else {
        uint32_t n_chunks = cycles / CHUNK;
        uint32_t i;
        for (i = 0; i < n_chunks; i++) {
            __delay32(CHUNK);
            __builtin_clrwdt();
        }
        __delay32(cycles - n_chunks * CHUNK);
        __builtin_clrwdt();
    }
    __builtin_clrwdt();

    // TODO: this would be more elegant
    //    uint64_t end_time = clock_now() + cycles;
    //    do {
    //        __builtin_clrwdt();
    //    } while (end_time < clock_now());
}

void block_ms(uint32_t ms) { delay32wd(ms * (FCY / 1000ULL)); }

void block_us(uint32_t us) { delay32wd(us * (FCY / 1000000ULL)); }

void block_s(float s) { delay32wd((uint32_t)(s * FCY)); }