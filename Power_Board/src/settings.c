#include "libpic30.h"
#include "settings.h"
#include "stdhdr.h"
#include "version.GENERATED.h"
#include "xc.h"

/// Address in non-volatile memory. Points to a 24-bit instruction in memory
/// See the reference manual section "PIC24F Flash Program Memory" for more info.
typedef uint_least24_t eeptr;

/// Get the table page of the address; i.e. the value for TBLPAG
uint16_t eepage(eeptr addr) { return (uint16_t)(addr >> 16); }

/// Get the offset of the address; i.e. the first argument for __builtin_tbl[rd/wr][l/h]
uint16_t eeoffset(eeptr addr) { return (uint16_t)addr; }

// these define how many addresses to skip in between addresses
// e.g. to get the start of the erase block at address x, use (x/BLOCK_STRIDE)*BLOCK_STRIDE

/// Amount to increment NVM address between words
#define WORD_STRIDE 0x2UL
/// Amount to increment NVM address between rows; i.e. the basic unit of writing instructions
#define ROW_STRIDE (_FLASH_ROW * WORD_STRIDE)
/// Amount to increment NVM address between rows; i.e. the minimum size of an erase operation
/// Yes, I know they call it _FLASH_PAGE. this is incredibly confusing, but they mean the erase
/// block size
#define BLOCK_STRIDE (_FLASH_PAGE * WORD_STRIDE)
/// Amount to increment NVM address between pages; i.e. the maximum amount of NVM that can be mapped
/// to RAM at a time using PSV 16 bit offset in page
#define PAGE_STRIDE 0x10000UL

/// Although a word is 3 bytes, PSV only exposes 2 of them
/// This is in contrast to eds, which uses the full 3 bytes.
const int BYTES_PER_PSV_WORD = 2;

/// Get an eeptr corresponding to the address of the given object in PSV
#define EEADDR(ptr) ((((eeptr)__builtin_tblpage(ptr)) << 16) | __builtin_tbloffset(ptr))

/// The robot settings to be loaded when the robot starts up.
/// The values here are defaults. They may be changed by the @ref settings_save function
static __psv__ Settings g_settings_nvm __attribute__((space(auto_psv))) = {
    //
    .firmware =
        {
            .build_date = BUILD_DATE,
            .build_time = BUILD_TIME,
            .release_version_flat = RELEASE_VERSION_FLAT,
        },
    .main =
        {
            .drive_poll_ms = 5,
            .power_poll_ms = 1,
            .i2c_poll_ms = 1,
            .communication_poll_ms = 1,
            .analog_poll_ms = 1,
            .flipper_poll_ms = 8,
        },
    .communication =
        {
            .rx_bufsize_bytes = 256,
            .tx_bufsize_bytes = 256,
            .baud_rate = 57600,
            .drive_command_timeout_ms = 333,
            .fan_command_timeout_ms = 333,
        },
    .power =
        {
            .overcurrent_trigger_threshold_ma = 15000,
            .overcurrent_trigger_duration_ms = 10,
            .overcurrent_reset_threshold_ma = 10000,
            .overcurrent_reset_duration_ms = 100,
            .charging_battery_switch_ms = 10000,
        },
    .flipper =
        {
            .is_calibrated = false,
        },
    .i2c =
        {
            .step_timeout_ms = 10,
        },
    .drive =
        {
            .motor_pwm_frequency_khz = 8,
            .motor_protect_direction_delay_ms = 10,
        },
};

/// Erase the given block of NVM.
void nvm_erase_block(eeptr dest) {
    uint16_t old_tblpag = TBLPAG;
    BREAKPOINT_IF(dest % BLOCK_STRIDE != 0);
    TBLPAG = eepage(dest);
    // assuming that eeprom_config fits in a single block
    NVMCON = _FLASH_ERASE_CODE;          //  0x4042;
    __builtin_tblwtl(eeoffset(dest), 0); // Dummy write to select the block
    __builtin_disi(5);
    __builtin_write_NVM();

    TBLPAG = old_tblpag;
}

/// Copy the data from the write latches into an NVM row.
void flush_psv_row() {
    /// Value for NVMCON to begin a row write operation
    NVMCON = _FLASH_WRITE_ROW_CODE;
    __builtin_disi(5);
    __builtin_write_NVM();
}

/// Copy the given value to the EEPROM program space.
/// We only take advantage of the 16 lowest bits of each instruction (contrast with EDS which uses
/// the full 24 bits) We also assume the destination block has already been erased.
/// @param dest destination address in EEPROM
/// @param src source address in RAM
/// @param count number of bytes to copy. Generally sizeof(*src).
void psv_write(eeptr dest, const void *src, size_t count) {
    BREAKPOINT_IF(BLOCK_STRIDE <= count);

    uint16_t old_tblpag = TBLPAG;

    // number of bytes we are offset from the beginning of src
    size_t i_src = 0;
    // pointer to the current address in dest
    eeptr dest_addr = dest;

    TBLPAG = eepage(dest_addr);
    while (1) {
        uint16_t value;
        if (i_src + 1 == count) {
            value = (*(uint8_t *)(src + i_src)) | 0xff00;
        } else if (i_src < count) {
            value = *(uint16_t *)(src + i_src);
        } else {
            // this leaves the data in NVM unchanged
            value = 0xffff;
        }
        __builtin_tblwth(dest_addr, 0xff);
        __builtin_tblwtl(dest_addr, value);

        i_src += BYTES_PER_PSV_WORD;
        dest_addr += WORD_STRIDE;

        if (dest_addr % ROW_STRIDE == 0) {
            flush_psv_row();
        }
        if (i_src >= count) {
            break;
        } else if (dest_addr % PAGE_STRIDE == 0) {
            TBLPAG = eepage(dest_addr);
        }
    }
    TBLPAG = old_tblpag;
}

void settings_save(const Settings *settings) {
    // Equivalent to the (hypothetical but illegal) code: eeprom_config = *settings_current;
    psv_write(EEADDR(&g_settings_nvm), settings, sizeof(*settings));
}

Settings settings_load() { return g_settings_nvm; }