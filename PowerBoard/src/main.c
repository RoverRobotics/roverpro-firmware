/// @file
/// Robotex one-size-fits-all firmware. Designed for the PIC24FJ256GB106 only.
// Set configuration bits.
// Note we can't overwrite these with the bootloader, so this is only needed
// for writing firmware with the PICkit3

#include "main.h"
#include "../../bootypic/devices/pic24fj256gb106/config.h"
#include "motor.h"

#include "analog.h"
#include "clock.h"
#include "communication.h"
#include "cooling.h"
#include "drive.h"
#include "flipper.h"
#include "i2clib.h"
#include "poll_i2c.h"
#include "power.h"
#include "power2.h"

/// Perform initialization of all rover subsystems
void rover_init();

/// Do an incremental amount of work on all rover subsystems. Should take <1ms
void rover_main_loop();

/// Increment `value` until it hits limit,
/// at which point reset it to zero.
bool tick_counter(uint16_t *value, uint16_t limit) {
    if (++(*value) >= limit) {
        *value = 0;
        return true;
    } else {
        return false;
    }
}

void rover_init() {
    clock_init();

    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);

    power_init();
    uart_init();
    analog_init();

    drive_init();
    cooling_init();
    cooling_blast_fan();

    RCON = 0;
}

void rover_main_loop() {
    static struct {
        uint16_t power;
        uint16_t i2c;
        uint16_t communication;
        uint16_t analog;
        uint16_t flipper;
        uint16_t drive;
    } counters = {0};

    if (tick_counter(&counters.drive, g_settings.main.drive_poll_ms)) {
        drive_tick();
    }
    if (tick_counter(&counters.analog, g_settings.main.analog_poll_ms)) {
        analog_tick();
    }
    if (tick_counter(&counters.power, g_settings.main.power_poll_ms)) {
        power_tick();
    }
    if (tick_counter(&counters.i2c, g_settings.main.i2c_poll_ms)) {
        i2c_tick_all();
    }
    if (tick_counter(&counters.communication, g_settings.main.communication_poll_ms)) {
        uart_tick();
    }
    if (tick_counter(&counters.flipper, g_settings.main.flipper_poll_ms)) {
        tick_flipper_feedback();
    }
}

Settings g_settings;
State g_state = {{{0}}};

/// Set up and start Timer1: 1 kHz, no interrupts
void IniTimer1() {
    T1CON = 0x0000;         // clear register
    T1CONbits.TCKPS = 0b00; // 0b00 = 1:1 prescale
    TMR1 = 0;               // clear timer1 register
    uint16_t frequency_hz = 1000;
    PR1 = (FCY / frequency_hz) - 1;
    T1CONbits.TON = 1; // start timer
}

int main(void) {
    // make sure we start off in a default state
    AD1PCFGL = 0xffff;
    AD1PCFGH = 0x0003;

    TRISB = 0xffff;
    TRISC = 0xffff;
    TRISD = 0xffff;
    TRISE = 0xffff;
    TRISF = 0xffff;
    TRISG = 0xffff;

    g_settings = settings_load();

    rover_init();
    IniTimer1();
    while (1) {
        __builtin_clrwdt();
        if (_T1IF == 1) {
            _T1IF = 0;
            rover_main_loop();
        }
    }
}

void clear_fault() {
    g_state.drive.last_overspeed_fault_timestamp = 0;
    g_state.power.last_overcurrent_fault_timestamp = 0;
}

Fault get_fault() {
    Fault result = FAULT_NONE;

    if (g_state.drive.last_overspeed_fault_timestamp) {
        result |= FAULT_OVERSPEED;
    }
    if (g_state.power.last_overcurrent_fault_timestamp) {
        result |= FAULT_OVERCURRENT;
    }
    return result;
}
