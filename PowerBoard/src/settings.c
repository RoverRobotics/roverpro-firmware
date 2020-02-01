#include "settings.h"
#include "stdhdr.h"
#include "version.GENERATED.h"
#include <xc.h>

// these define how many addresses to skip in between addresses
// e.g. to get the start of the erase block at address x, use (x/BLOCK_STRIDE)*BLOCK_STRIDE
#define WORDS_PER_ROW 64
#define WORDS_PER_PAGE 512

#define FLASH_ERASE_PAGE_CODE 0x4042
#define FLASH_WRITE_ROW_CODE 0x4001

/// Amount to increment NVM address between words
#define WORD_STRIDE 2
/// Amount to increment NVM address between rows; i.e. the basic unit of writing instructions
#define ROW_STRIDE (WORDS_PER_ROW * WORD_STRIDE)
/// Amount to increment NVM address between rows; i.e. the minimum size of an erase operation
/// Yes, I know they call it _FLASH_PAGE. this is incredibly confusing, but they mean the erase
/// block size
#define BLOCK_STRIDE (WORDS_PER_PAGE * WORD_STRIDE)

__psv__ union {
    Settings settings __attribute__((aligned(BLOCK_STRIDE)));
    // alignment to ensure this starts on a flash block boundary.
    // note that wrapping it in a union also ensures it ends on a flash page boundary.
} g_nvm_settings_page __attribute__((
    space(psv), // PSV space means put this in persistent data, don't cross tblpag boundaries, and
                // fill upper byte with 0
    address(0x8400), )) = {
    /// The robot settings to be loaded when the robot starts up.
    /// The values here are defaults. They may be changed by the @ref settings_save function
    .settings = {
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
                .brake_on_drive_timeout = false,
                .brake_on_zero_speed_command = true,
                .overspeed_runaway_limit = 2,
                .overspeed_fault_effort = 0.8F,
                .overspeed_fault_trigger_s = 1.0F,
                .overspeed_fault_recover_s = 1.0F,
                .overspeed_runaway_history_s = 30.0F,
                .overspeed_runaway_reset_effort = 0.05F,
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
                .motor_pwm_frequency_hz = 8000.0F,
                .time_to_full_speed = 0.7F,
                .motor_slow_decay_mode = false,
                .max_instantaneous_delta_effort = 0.2F,
            },
        .cooling =
            {
                .fan_lo_temperature = 40.0F,
                .fan_lo_duty_factor = 0.5F,
                .fan_hi_temperature = 50.0F,
                .fan_hi_duty_factor = 1.0F,
            },
    }};

/// Copy the settings to non-volatile memory
/// We store the data PSV-style, using only the 16 lowest bits of each 24-bit instruction
/// @param settings new value for settings
void settings_save(const Settings *settings) {
    // IMPORTANT NOTE ABOUT DEBUGGING THIS:
    // An NVM write (__builtin_write_NVM()) stalls for a long time. (1.4 ms when writing a row)
    // The disable interrupt counter DISICNT counts down during the cpu stall, so it will always be
    // zero after an NVM write. The watchdog timer *does not* count this stall time so we don't need
    // to keep resetting it.

    const uint8_t *settings_bytes = (uint8_t *)(settings);
    // Equivalent to the (hypothetical but illegal) code: g_nvm_settings_page.settings = *settings;
    // disable interrupts for the maximum number of cycles
    uint8_t old_tblpag = TBLPAG;

    // assertions that we can erase settings without harming other data
    // assert(__builtin_tbloffset(&g_nvm_settings_page) % BLOCK_STRIDE == 0);
    _Static_assert(sizeof(g_nvm_settings_page) == BLOCK_STRIDE, "Settings end at a block boundary");

    TBLPAG = __builtin_tblpage(&g_nvm_settings_page);
    uint16_t start_offset = __builtin_tbloffset(&g_nvm_settings_page);

    __builtin_tblwtl(start_offset, 0); // Dummy write to select the block
    NVMCON = FLASH_ERASE_PAGE_CODE;

    __builtin_disi(0x1fff);
    __builtin_write_NVM();

    uint16_t i = 0;
    while (i < BLOCK_STRIDE) {
        /*
                // write to non-psv byte
                __builtin_tblwth(start_offset + i, 0x00);
                uint8_t lobyte = (i < sizeof(Settings)) ? settings_bytes[i] : 0x00;
                uint8_t hibyte = (i + 1 < sizeof(Settings)) ? settings_bytes[i + 1] : 0x00;
                volatile uint16_t word = lobyte | (((uint16_t)hibyte) << 8U);
                __builtin_tblwtl(start_offset + i, word);
                i += 2;
        */

        __builtin_tblwthb(start_offset + i, 0x00);
        // write to data byte 0 (wtl = data byte)
        __builtin_tblwtlb(start_offset + i, i < sizeof(Settings) ? settings_bytes[i] : 0x00);
        ++i;
        // write to data byte 1 (wtl = data byte)
        __builtin_tblwtlb(start_offset + i, i < sizeof(Settings) ? settings_bytes[i] : 0x00);
        ++i;

        /// Copy the data from the write latches into an NVM row.
        if ((start_offset + i) % ROW_STRIDE == 0) {
            NVMCON = FLASH_WRITE_ROW_CODE;

            __builtin_disi(0x1fff);
            __builtin_write_NVM();
        }
    }
    NVMCON = 0;
    TBLPAG = old_tblpag;
}

Settings settings_load() { return g_nvm_settings_page.settings; }
