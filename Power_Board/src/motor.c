/*---------------------------Dependencies-------------------------------------*/
#include "clock.h"
#include "hardware_definitions.h"
#include "main.h"
#include "motor.h"
#include "xc.h"

/*---------------------------Interrupt Service Routines (ISRs)----------------*/

void __attribute__((__interrupt__, auto_psv)) _CNInterrupt(void) {
    static bool last_tacho[MOTOR_CHANNEL_COUNT] = {0};

    bool tacho[MOTOR_CHANNEL_COUNT] = {_RD11, _RF3, _RD5};
    bool diro[MOTOR_CHANNEL_COUNT] = {M1_DIRO, M2_DIRO, M3_DIRO};

    uint64_t now = clock_now();
    MotorChannel c;
    for (EACH_MOTOR_CHANNEL(c)) {
        if (last_tacho[c] != tacho[c]) {
            g_state.drive.motor_encoder_period[c] =
                min((now - g_state.drive.last_encoder_timestamp[c]) / 256, UINT16_MAX);
            if ((diro[c] == 0) ^ (c == MOTOR_RIGHT)) {
                g_state.drive.motor_encoder_count[c]++;
            } else {
                g_state.drive.motor_encoder_count[c]--;
            }
            g_state.drive.last_encoder_timestamp[c] = now;
            last_tacho[c] = tacho[c];
        }
    }
    _CNIF = 0;
}

/*---------------------------Public Function Definitions----------------------*/
void motor_tach_init() {
    _CN14IE = 1; // M3_TACHO
    _CN56IE = 1; // M1_TACHO
    _CN71IE = 1; // M2_TACHO

    _CNIF = 0;
    _CNIE = 1;
}

void tach_tick() {
    _CNIE = 0;
    uint64_t now = clock_now();
    MotorChannel c;
    for (EACH_MOTOR_CHANNEL(c)) {
        uint64_t period_so_far = (now - g_state.drive.last_encoder_timestamp[c]) / 256;
        if (period_so_far > UINT16_MAX) {
            g_state.drive.motor_encoder_period[c] = 0;
        } else if (period_so_far > g_state.drive.motor_encoder_period[c]) {
            g_state.drive.motor_encoder_period[c] = period_so_far;
        }
    }
    _CNIE = 1;
}

/*---------------------------Private Function Definitions---------------------*/

/// PIC 24F DS39723 Output Compare with Dedicated Timers
typedef struct {
    /// Control Register 1
    volatile uint16_t *OCxCON1;
    /// Control Register 2
    volatile uint16_t *OCxCON2;
    /// Data Register
    volatile uint16_t *OCxR;
    /// Secondary Data Register
    volatile uint16_t *OCxRS;
    /// Internal Time Base Register
    volatile uint16_t *OCxTMR;
    /// Output value associated with this port for remappable output
    uint16_t RPnR;
} OutputCompareModule;

const OutputCompareModule OUTPUT_COMPARE_1 = {&OC1CON1, &OC1CON2, &OC1R, &OC1RS, &OC1TMR, 18};
const OutputCompareModule OUTPUT_COMPARE_2 = {&OC2CON1, &OC2CON2, &OC2R, &OC2RS, &OC2TMR, 19};
const OutputCompareModule OUTPUT_COMPARE_3 = {&OC3CON1, &OC3CON2, &OC3R, &OC3RS, &OC3TMR, 20};

void outputcompare_pwm_init(OutputCompareModule oc, uint16_t pwm_freq_khz) {
    // NOTE:
    // Refer to Data Sheet DS39723. This thing is complicated!

    // 1. Configure the OCx output for one of the
    // available Peripheral Pin Select pins.
    // done in I/O Init.

    // 3. Calculate the desired period and load it into the
    // OCxRS register.
    // PWM Period = [(OCxRS) + 1] * TCY * (Timer Prescale Value)
    // If OCxCON2.SYNCSEL<4:0> = 0x1F
    // If OCxCON2.SYNCSEL<4:0> = N (where N is the alternate value to select this as the Period
    // register) If OCxCON2.OCTRIG = 1
    *oc.OCxRS = FCY / 1000 / pwm_freq_khz - 1;

    *oc.OCxCON1 = (
        // Clock source. 0b111 = peripheral clock FCY
        (0b111 << _OC1CON1_OCTSEL_POSITION) |
        // OCM 0b110 = Edge PWM Edge-Aligned PWM mode: Output set high when OCxTMR = 0 and set low
        // when OCxTMR = OCxR
        (0b110 << _OC1CON1_OCM_POSITION));

    *oc.OCxCON2 = (
        // Trigger/Synchronization source
        // 0b11111 = This OCx module
        // SYNCSEL<4:0> = 0b11111 makes the timer reset when it reaches the value of
        //                OCxRS, making the OCx module use its own Sync signal.
        (0b11111 << _OC1CON2_SYNCSEL_POSITION) |
        // Trigger/Sync Select
        // 1 = Trigger OCx from source designated by SYNCSELx bits
        // 0 = Synchronize OCx with source designated by SYNCSELx bits
        (0b0 << _OC1CON2_OCTRIG_POSITION));
}

void outputcompare_pwm_set_duty(OutputCompareModule oc, float duty_factor) {
    BREAKPOINT_IF(duty_factor < 0.0F);
    BREAKPOINT_IF(duty_factor > 1.0F);
    *oc.OCxR = (uint16_t)(*oc.OCxRS * duty_factor + 0.5F);
}

MotorStatusFlag motor_update(MotorChannel channel, MotorStatusFlag status, float duty) {
    // NOTE: all the hardware flags are active-low. So we negate the values before assigning and
    // when retrieving
    switch (channel) {
    case (MOTOR_LEFT):
        M1_COAST = !(status & MOTOR_FLAG_COAST);
        M1_BRAKE = !(status & MOTOR_FLAG_BRAKE);
        M1_DIR = !(status & MOTOR_FLAG_REVERSE);
        M1_MODE = !(status & MOTOR_FLAG_DECAY_MODE);
        outputcompare_pwm_set_duty(OUTPUT_COMPARE_1, duty);
        return (status & ~MOTOR_FLAG_MASK_FEEDBACK) | (!M1_FF1 ? MOTOR_FLAG_FAULT1 : 0) |
               (!M1_FF2 ? MOTOR_FLAG_FAULT2 : 0);
    case (MOTOR_RIGHT):
        M2_COAST = !(status & MOTOR_FLAG_COAST);
        M2_BRAKE = !(status & MOTOR_FLAG_BRAKE);
        M2_DIR = !(status & MOTOR_FLAG_REVERSE);
        M2_MODE = !(status & MOTOR_FLAG_DECAY_MODE);
        outputcompare_pwm_set_duty(OUTPUT_COMPARE_2, duty);
        return (status & ~MOTOR_FLAG_MASK_FEEDBACK) | (!M2_FF1 ? MOTOR_FLAG_FAULT1 : 0) |
               (!M2_FF2 ? MOTOR_FLAG_FAULT2 : 0);
    case (MOTOR_FLIPPER):
        M3_COAST = !(status & MOTOR_FLAG_COAST);
        M3_BRAKE = !(status & MOTOR_FLAG_BRAKE);
        M3_DIR = !(status & MOTOR_FLAG_REVERSE);
        M3_MODE = !(status & MOTOR_FLAG_DECAY_MODE);
        outputcompare_pwm_set_duty(OUTPUT_COMPARE_3, duty);
        return (status & ~MOTOR_FLAG_MASK_FEEDBACK) | (!M3_FF1 ? MOTOR_FLAG_FAULT1 : 0) |
               (!M3_FF2 ? MOTOR_FLAG_FAULT2 : 0);
    }
    BREAKPOINT();
    return 0;
}

void motor_init(MotorChannel c) {
    uint16_t pwm_khz = g_settings.drive.motor_pwm_frequency_khz;
    motor_update(c, MOTOR_FLAG_COAST, 0.F);
    OutputCompareModule oc;
    switch (c) {
    case MOTOR_LEFT:
        // set directions of pins
        M1_DIR_EN(1);
        M1_BRAKE_EN(1);
        M1_MODE_EN(1);
        M1_COAST_EN(1);
        oc = OUTPUT_COMPARE_1;
        M1_PWM = oc.RPnR;
        break;
    case MOTOR_RIGHT:
        M2_DIR_EN(1);
        M2_BRAKE_EN(1);
        M2_MODE_EN(1);
        M2_COAST_EN(1);
        oc = OUTPUT_COMPARE_2;
        M2_PWM = oc.RPnR;
        break;
    case MOTOR_FLIPPER:
        M3_DIR_EN(1);
        M3_BRAKE_EN(1);
        M3_MODE_EN(1);
        M3_COAST_EN(1);
        oc = OUTPUT_COMPARE_3;
        M3_PWM = oc.RPnR;
        break;
    }
    outputcompare_pwm_init(oc, pwm_khz);
}