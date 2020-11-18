/// Device-specific implementation details
#include "boot_user.h"
#include "i2clib.h"
#include "power.h"
#include <xc.h>

void pre_bootload() {
    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);
    power_init();
}

void initOsc(void) { CLKDIV = 0; }

void initPins(void) {
    /* no analog, all digital */
    AD1PCFGL = 0xffff;
    AD1PCFGH = 0x3;
}

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

uint32_t get_idle_time_ticks() {
    uint32_t n_ticks = 0;
    n_ticks |= (uint32_t)TMR2;
    n_ticks |= ((uint32_t)TMR3HLD) << 16; // the value of TMR3 when TMR2 was read
    return n_ticks;
}

inline uint32_t timer_ticks_from_milliseconds(uint32_t milliseconds) {
    return FCY / 256 / 1000 * milliseconds;
}

/// Pre-read the program and see if it appears to be executable code
inline bool program_looks_ok(){
    uint16_t save_tblpag = TBLPAG;
    bool seen_valid_opcode = false;
    int i;
    for (i=0; i<8; ++i){
        uint32_t address = APPLICATION_START_ADDRESS + 2*i;
        TBLPAG = (address)>>16;
        uint8_t opcode = __builtin_tblrdhb((uint16_t)address);
        // 0x00 = NOP and 0xff = NOPR
        // they both do nothing.
        if (opcode != 0x00 && opcode != 0xff) {
            seen_valid_opcode = true;
        }
    }
    TBLPAG = save_tblpag;
    return seen_valid_opcode;
}

bool should_start_bootloader() {
    if (!program_looks_ok()){
        return true;
    }
    RCONBITS rcon = RCONbits;
    return rcon.EXTR || rcon.SWR;
}

bool should_continue_bootloader() {
    if (!program_looks_ok())
        return true;

    // If there was a hardware reset, allow the user extra time to bootload
    uint32_t bootload_duration_ms =
        (RCONbits.EXTR ? BOOTLOAD_LONG_TIMEOUT_MS : BOOTLOAD_SHORT_TIMEOUT_MS);

    return (get_idle_time_ticks() < timer_ticks_from_milliseconds(bootload_duration_ms));
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
void eraseByAddress(uint32_t address) {
    uint16_t offset;
    uint16_t tempTblPag = TBLPAG;
    TBLPAG = (uint16_t)((address & 0x00ff0000) >> 16); // initialize PM Page Boundary
    offset = (uint16_t)((address & 0x0000ffff) >> 0);
    NVMCON = 0x4042; // page erase operation
    __builtin_tblwtl(offset, 0);
    __builtin_disi(5);
    __builtin_write_NVM();

    TBLPAG = tempTblPag;
}

uint32_t readAddress(uint32_t address) {
    uint16_t offset;
    uint16_t tempTblPag = TBLPAG;
    uint32_t result = 0;
    // Set up pointer to the first memory location to be written
    TBLPAG = (uint16_t)((address & 0x00ff0000) >> 16); // initialize PM Page Boundary
    offset = (uint16_t)((address & 0x0000ffff) >> 0);  // initialize lower word of address

    result |= (((uint32_t)__builtin_tblrdh(offset)) << 16); // read from address high word
    result |= (((uint32_t)__builtin_tblrdl(offset)) << 0);  // read from address low word

    TBLPAG = tempTblPag;
    return result;
}

void writeInstr(uint32_t address, uint32_t instruction) {
    uint16_t tempTblPag = TBLPAG;

    uint16_t offset = (uint16_t)(address & 0x0000ffff);
    TBLPAG = (uint16_t)((address & 0xffff0000) >> 16); /* initialize PM Page Boundary */

    NVMCON = 0x4003; // Memory word program operation

    __builtin_tblwtl(offset, (uint16_t)((instruction & 0x0000ffff) >> 0));
    __builtin_tblwth(offset, (uint16_t)((instruction & 0x00ff0000) >> 16));
    __builtin_disi(5);
    __builtin_write_NVM();

    TBLPAG = tempTblPag;
}

void writeRow(uint32_t address, uint32_t *words) {

    // see "Row Programming in C with Built-in Functions (Unmapped Latches)"
    uint16_t i;
    uint16_t tempTblPag = TBLPAG;
    uint16_t offset = (uint16_t)(address & 0x0000ff80);
    TBLPAG = (uint16_t)((address & 0x00ff0000) >> 16); /* initialize PM Page Boundary */

    NVMCON = 0x4001; // Memory row program operation
    for (i = 0; i < _FLASH_ROW; i++) {
        __builtin_tblwtl(offset + i * 2, (uint16_t)((words[i] & 0x0000ffff) >> 0));
        __builtin_tblwth(offset + i * 2, (uint16_t)((words[i] & 0x00ff0000) >> 16));
    }
    __builtin_disi(5);
    __builtin_write_NVM();

    TBLPAG = tempTblPag;
}

void doubleWordWrite(uint32_t address, uint32_t *progDataArray) {
    writeInstr(address, progDataArray[0]);
    writeInstr(address + 2, progDataArray[1]);
}

void writeMax(uint32_t address, uint32_t *progData) {
    uint16_t i;
    uint16_t length = (uint16_t)(MAX_PROG_SIZE / _FLASH_ROW);

    for (i = 0; i < length; i++) {
        writeRow(address + ((i * _FLASH_ROW) << 1), &progData[i * _FLASH_ROW]);
    }
}