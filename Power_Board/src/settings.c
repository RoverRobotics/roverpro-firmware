#include "main.h"
#include "settings.h"
#include "stdhdr.h"
#include "xc.h"

// the address of a particular 24-bit word in memory.
// proper addresses have lsb 0
typedef uint_least24_t eeptr;

// these define how many addresses to skip in between addresses
// e.g. to get the start of the erase block at address x, use (x/BLOCK_STRIDE)*BLOCK_STRIDE
// 3 bytes per word, packed into the lowes bit
#define WORD_STRIDE 0x2UL
// 64 words per row
#define ROW_STRIDE 0x80UL
// 512 instructions per erase
#define BLOCK_STRIDE 0x400UL
// 16 bit offset in page
#define PAGE_STRIDE 0x10000UL

// although a word is 3 bytes, PSV only uses 2 of them
const int BYTES_PER_PSV_WORD = 2;

// Get an eeptr corresponding to the address of the given object in PSV
#define EEADDR(ptr) ((((eeptr)__builtin_tblpage(ptr)) << 16) | __builtin_tbloffset(ptr))

uint16_t eepage(eeptr addr) { return (uint16_t)(addr >> 16); }
uint16_t eeoffset(eeptr addr) { return (uint16_t)addr; }

const static __psv__ Settings settings_nvm __attribute__((space(auto_psv))) = { //
    .main = {.motor_poll_ms = 5,
             .electrical_poll_ms = 1,
             .i2c_poll_ms = 1,
             .communication_poll_ms = 1,
             .analog_readouts_poll_ms = 1,
             .motor_controller_poll_ms = 1},
    .communication =
        {
            .baud_rate = 57600,
            .motor_command_timeout_ms = 333,
            .fan_command_timeout_ms = 333,
        },
    .electrical =
        {
            .overcurrent_trigger_threshold_ma = 15000,
            .overcurrent_trigger_duration_ms = 10,
            .overcurrent_reset_threshold_ma = 10000,
            .overcurrent_reset_duration_ms = 100,
        },
    .flipper =
        {
            .is_calibrated = false,
        },
    .i2c =
        {
            .step_timeout_ms = 5,
        },
    .motor_controller =
        {
            .pid_p_weight = 0.0005,
            .pid_i_weight = 0.00003,
            .pid_d_weight = 0.0,
            .min_effort = -1.0,
            .max_effort = +1.0,
            .iir_alpha = 0.8,
        },
    .motor = {
        .pwm_hz = 1000,
        .motor_dead_time_between_directions_ms = 10,
    }};

const uint16_t NVMCON_ERASE_BLOCK = 0x4042;
const uint16_t NVMCON_WRITE_ROW = 0x4001;

void nvm_erase_block(eeptr dest) {
    uint16_t old_tblpag = TBLPAG;
    BREAKPOINT_IF(dest % BLOCK_STRIDE != 0);
    TBLPAG = eepage(dest);
    // assuming that eeprom_config fits in a single block
    NVMCON = NVMCON_ERASE_BLOCK;
    __builtin_tblwtl(eeoffset(dest), 0); // Dummy write to select the block
    __builtin_disi(5);
    __builtin_write_NVM();

    TBLPAG = old_tblpag;
}

void flush_psv_row() {
    // flush the current row to NVM
    NVMCON = NVMCON_WRITE_ROW;
    __builtin_disi(5);
    __builtin_write_NVM();
}

/// Copy the given value to the EEPROM program space.
/// We only take advantage of the 16 lowest bits of each instruction
/// We also assume the target address has already been erased
/// @param dest destination address in EEPROM
/// @param src source address in RAM
/// @param count number of bytes to copy. Generally sizeof(*src).
void psv_write(eeptr dest, const void *src, size_t count) {
    BREAKPOINT_IF(BLOCK_STRIDE <= count);

    uint16_t old_tblpag = TBLPAG;

    // number of bytes we are offset from the beginning of src
    size_t i_src;
    // pointer to the current address in dest
    eeptr dest_addr = dest;

    TBLPAG = eepage(dest_addr);
    while (1) {
        uint16_t value;
        if (i_src + 1 == count) {
            value = (*(uint8_t *)(src + i_src)) | 0xff00;
        } else {
            value = *(uint16_t *)(src + i_src);
        }
        __builtin_tblwth(dest_addr, 0xff);
        __builtin_tblwtl(dest_addr, value);

        i_src += BYTES_PER_PSV_WORD;
        dest_addr += WORD_STRIDE;

        if (i_src >= count) {
            flush_psv_row();
            break;
        }
        if (dest_addr % ROW_STRIDE == 0) {
            flush_psv_row();
        }
        if (dest_addr % PAGE_STRIDE == 0) {
            TBLPAG = eepage(dest_addr);
        }
    }
    TBLPAG = old_tblpag;
}

void settings_save(const Settings *settings) {
    // Equivalent to the (hypothetical but illegal) code: eeprom_config = *settings_current;
    psv_write(EEADDR(&settings_nvm), settings, sizeof(*settings));
}

Settings settings_load() { return settings_nvm; }