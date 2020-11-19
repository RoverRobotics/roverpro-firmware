/// Device-specific implementation details
#include "boot_user.h"
#include "i2clib.h"
#include "power.h"
#include <xc.h>

void uart_map_rx(uint16_t rpn) {
    // map that pin to UART RX
    _U1RXR = rpn;
}

static __attribute__((always_inline)) void uart_map_tx(uint16_t rpn) {
// this big case statement benefits greatly from inlining this function.
#define _RPxR(x) _RP##x##R
#define CASE(x)                                                                                    \
    case x:                                                                                        \
        _RPxR(x) = _RPOUT_U1TX;                                                                    \
        break;
    switch (rpn) {
        CASE(0)
        CASE(1)
        CASE(2)
        CASE(3)
        CASE(4)
        // no RP5R on this chip
        CASE(6)
        CASE(7)
        CASE(8)
        CASE(9)
        CASE(10)
        CASE(11)
        CASE(12)
        CASE(13)
        CASE(14)
        // no RP15R on this chip
        CASE(16)
        CASE(17)
        CASE(18)
        CASE(19)
        CASE(20)
        CASE(21)
        CASE(22)
        CASE(23)
        CASE(24)
        CASE(25)
        CASE(26)
        CASE(27)
        CASE(28)
        CASE(29)
        CASE(30)
    }
#undef CASE
#undef _RPxR
}

/**
 * @brief initializes the oscillator
 */
void initOsc(void) { CLKDIV = 0; }

/**
 * @brief initializes the pins
 */
void initPins(void) {
    /* no analog, all digital */
    AD1PCFGL = 0xffff;
    AD1PCFGH = 0x3;
}

/**
 * @brief initializes the UART
 */
void initUart(void) {
    U1MODE = 0;
    U1STA = 0x2000;
    uart_map_rx(RX_PIN);
    uart_map_tx(TX_PIN);

    if (UART_BAUD_RATE * 4 < FCY) {
        U1MODEbits.BRGH = 0; // High Baud Rate Select bit = off
        U1BRG = FCY / 16 / UART_BAUD_RATE - 1;
    } else {
        U1MODEbits.BRGH = 1;
        U1BRG = FCY / 4 / UART_BAUD_RATE - 1;
    }

    /* note UART module overrides the PORT, LAT, and TRIS bits, so no
     * need to set them */

    U1MODEbits.UARTEN = 1; /* enable UART */
    U1STAbits.UTXEN = 1;   /* transmit enabled */

    while (U1STAbits.URXDA)
        U1RXREG; /* clear anything in the buffer */
}

void initTimers(void) {
    T2CON = T3CON = 0;
    TMR2 = TMR3 = 0;
    PR2 = PR3 = 0xffff; // each timer will count up to this value

    T2CONbits.T32 = 1;      // merge timer 2 and timer 3 into a 32 bit timer
    T2CONbits.TCKPS = 0b11; // 256 prescale
    T2CONbits.TON = 1;
}

void pre_bootload_hook() {
    // set up power bus
    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);
    power_init();

    // EXTR = reset pin
    // SWR =  software reset
    if (!RCONbits.EXTR && !RCONbits.SWR) {
        try_start_app_hook();
    }

    // initialize peripherals needed for bootloader
    initOsc();
    initPins();
    initUart();
    initTimers();
}

void __attribute__((noload, noreturn, address(APPLICATION_START_ADDRESS))) app_entry_point() {
    __builtin_unreachable();
}

void try_start_app_hook() {
    uint16_t save_tblpag = TBLPAG;
    bool seen_valid_opcode = false;
    int i;
    // Pre-read the program and see if it appears to be executable code
    for (i = 0; i < 8; ++i) {
        uint32_t address = APPLICATION_START_ADDRESS + 2 * i;
        TBLPAG = address >> 16;
        uint8_t opcode = __builtin_tblrdhb((uint16_t)address);
        // 0x00 = NOP and 0xff = NOPR
        // they both do nothing.
        if (opcode != 0x00 && opcode != 0xff) {
            seen_valid_opcode = true;
            break;
        }
    }
    TBLPAG = save_tblpag;

    if (seen_valid_opcode) {
        RCONbits.SWR = 0;
        RCONbits.EXTR = 0;
        app_entry_point();
    }
}

uint32_t get_idle_time_ticks() {
    uint32_t n_ticks = 0;
    n_ticks |= (uint32_t)TMR2;
    n_ticks |= ((uint32_t)TMR3HLD) << 16; // the value of TMR3 when TMR2 was read
    return n_ticks;
}

inline uint32_t timer_ticks_from_milliseconds(uint32_t milliseconds) {
    return FCY / 256 / 1000 * milliseconds;
}

void bootload_loop_hook() {
    // If there was a hardware reset, allow the user extra time to bootload
    const uint32_t long_timeout_ticks = timer_ticks_from_milliseconds(BOOTLOAD_LONG_TIMEOUT_MS);
	const uint32_t short_timeout_ticks = timer_ticks_from_milliseconds(BOOTLOAD_SHORT_TIMEOUT_MS);
	
	uint32_t idle_time_ticks = get_idle_time_ticks();
	uint32_t timeout_ticks = (RCONbits.EXTR ? long_timeout_ticks : short_timeout_ticks);
	
    if (idle_time_ticks < timeout_ticks) {
        return;
    } else {
        try_start_app_hook();
    }
}

bool tryRxByte(uint8_t *outbyte) {
    if (U1STAbits.URXDA) {
        *outbyte = U1RXREG;
        return true;
    } else {
        return false;
    }
}

/// Device-specific implementations of bootloader operations
void erase_page(uint32_t address) {
    uint16_t offset;
    uint16_t temp_tblpag = TBLPAG;
    TBLPAG = (uint16_t)(address >> 16); // initialize PM Page Boundary
    offset = (uint16_t)(address >> 0);
    NVMCON = 0x4042; // page erase operation
    __builtin_tblwtl(offset, 0);
    __builtin_disi(5);
    __builtin_write_NVM();

    TBLPAG = temp_tblpag;
}

void read_words(uint32_t *words, uint32_t start_address, unsigned n_words) {
    uint16_t temp_tblpag = TBLPAG;
    uint32_t i;

    for (i = 0; i < n_words; ++i) {
        TBLPAG = (start_address + 2 * i) >> 16;
        uint16_t offset = (start_address + 2 * i) & 0xffff;
        words[i] =
            ((uint32_t)__builtin_tblrdh(offset) << 16 | (uint32_t)__builtin_tblrdl(offset) << 0);
    }

    TBLPAG = temp_tblpag;
}

void write_words(const uint32_t *words, uint32_t start_address, unsigned n_words) {
    uint16_t temp_tblpag = TBLPAG;
    uint32_t i = 0;
    while (i < n_words) {
        TBLPAG = (start_address + 2 * i) >> 16;

        // fill the write latches with data
        if (address_is_row_aligned(start_address + 2 * i) && i + _FLASH_ROW - 1 < n_words) {
            NVMCON = 0x4001;
            int j;
            for (j = 0; j < _FLASH_ROW; ++j) {
                uint16_t offset = (start_address + 2 * i) & 0x0000ffff;
                __builtin_tblwth(offset, (uint16_t)(words[i] >> 16));
                __builtin_tblwtl(offset, (uint16_t)(words[i] >> 0));

                ++i;
            }
        } else {
            NVMCON = 0x4003;
            uint16_t offset = (start_address + 2 * i) & 0x0000ffff;
            __builtin_tblwth(offset, (uint16_t)(words[i] >> 16));
            __builtin_tblwtl(offset, (uint16_t)(words[i] >> 0));

            ++i;
        }

        // write it out to NVM
        __builtin_disi(5);
        __builtin_write_NVM();
    }
    TBLPAG = temp_tblpag;
}