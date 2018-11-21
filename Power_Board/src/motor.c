/*---------------------------Dependencies-------------------------------------*/
#include "motor.h"
#include "stdhdr.h"
#include "hardware_definitions.h"
#include <p24Fxxxx.h>

/*---------------------------Macros-------------------------------------------*/
#define CAPTURE_BUFFER_COUNT 4
/*---------------------------Helper Function Prototypes-----------------------*/

/// Initialize PIC modules Timer4 and Timer5 together as a 32-bit timer
static void InitTimer4Timer5(void);
/// Set up PIC input capture module 1 for events on pin RPn
static void InitIC1(uint8_t RPn);
/// Set up PIC input capture module 2 for events on pin RPn
static void InitIC2(uint8_t RPn);
/// Set up PIC input capture module 3 for events on pin RPn
static void InitIC3(uint8_t RPn);
/// Set duty cycle for channel 1 (left motor). 0 <= Duty <= 1000
static void PWM1Duty(uint16_t duty);
/// Set duty cycle for channel 2 (right motor). 0 <= Duty <= 1000
static void PWM2Duty(uint16_t duty);
/// Set duty cycle for pwm channel 3 (flipper motor). 0 <= Duty <= 1000
static void PWM3Duty(uint16_t duty);

/// IC module constant: Clock source of Timer4 is the clock source of the capture counter
static const uint16_t IC_SELECT_TIMER4 = 0b010;

/// Motor direction constants for M1_DIR and M1_DIRO bits as well as other motors
typedef enum {
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_REVERSE = 0,
} MotorDir;

/*---------------------------Module Variables---------------------------------*/
typedef struct {
    /// high precision timestamp
    uint32_t timestamp;
    /// direction that the motor is spinning
    MotorDir dir;
} EncoderEvent;

size_t i_next_event[MOTOR_CHANNEL_COUNT];
static EncoderEvent event_buffer[MOTOR_CHANNEL_COUNT][CAPTURE_BUFFER_COUNT];

/*---------------------------Interrupt Service Routines (ISRs)----------------*/
void motor_tach_event_capture(uint8_t which, MotorDir dir, uint16_t captured_value_lo) {
    uint16_t captured_value_hi;

    // note reading TMR4 saves the value of TMR5 into TMR5HLD.
    // This is needed since reading TMR4 then TMR5 is not atomic
    if (TMR4 > captured_value_lo) {
        // TMR4 did not yet carry into TMR5
        captured_value_hi = TMR5HLD;
    } else {
        // TMR4 did carry into TMR5
        // We assume it only overflowed once, that is timer4 did not go through a whole
        // 2**16 * PCY = 2**16 / 16 MHz = 4 milliseconds cycle
        captured_value_hi = TMR5HLD - 1;
    }

    event_buffer[which][i_next_event[which]].timestamp =
        ((uint32_t)captured_value_hi << 16) | captured_value_lo;
    event_buffer[which][i_next_event[which]].dir = dir;

    i_next_event[which] = (i_next_event[which] + 1) % CAPTURE_BUFFER_COUNT;
}

// Interrupt function for PIC24 Input Capture module 1
void __attribute__((__interrupt__, auto_psv)) _IC1Interrupt(void) {
    _IC1IF = 0; // clear the source of the interrupt
    MotorDir dir = M1_DIRO;
    while (IC1CON1bits.ICBNE) {
        motor_tach_event_capture(0, dir, IC1BUF);

        // increase encoder count if motor is moving forward, decrease if backward
        if (dir == MOTOR_DIR_REVERSE)
            REG_MOTOR_ENCODER_COUNT.left--;
        else
            REG_MOTOR_ENCODER_COUNT.left++;
    }
}

// Interrupt function for PIC24 Input Capture module 2
void __attribute__((__interrupt__, auto_psv)) _IC2Interrupt(void) {
    _IC2IF = 0;
    MotorDir dir = M2_DIRO;
    while (IC2CON1bits.ICBNE) {
        motor_tach_event_capture(1, dir, IC2BUF);

        if (dir == MOTOR_DIR_REVERSE)
            REG_MOTOR_ENCODER_COUNT.right--;
        else
            REG_MOTOR_ENCODER_COUNT.right++;
    }
}

// Interrupt function for PIC24 Input Capture module 3
void __attribute__((__interrupt__, auto_psv)) _IC3Interrupt(void) {
    _IC3IF = 0;
    MotorDir dir = M3_DIRO;
    while (IC2CON1bits.ICBNE) {
        motor_tach_event_capture(2, dir, IC3BUF);
    }
}

/*---------------------------Public Function Definitions----------------------*/
void motor_tach_init() {
    InitTimer4Timer5();

    InitIC1(M1_TACHO_RPN);
    InitIC2(M2_TACHO_RPN);
    InitIC3(M3_TACHO_RPN);
}

float motor_tach_get_period(MotorChannel channel) {
    BREAKPOINT_IF(channel > MOTOR_CHANNEL_COUNT);

    // We are betting we can get through this before the data we're interested in is clobbered.
    size_t i = (i_next_event[channel] - 2 + CAPTURE_BUFFER_COUNT) % CAPTURE_BUFFER_COUNT;
    size_t j = (i_next_event[channel] + 1) % CAPTURE_BUFFER_COUNT;
    uint32_t timestamp = event_buffer[channel][j].timestamp;
    int32_t timestamp_interval = (int32_t)(timestamp - event_buffer[channel][i].timestamp);
    MotorDir dir = event_buffer[channel][j].dir;

    // if motor changed direction, can't trust encoder
    if (dir != event_buffer[channel][i].dir)
        return 0;

    // if it's been too long since last timestamp, can't trust encoder
    if (TMR5 - (timestamp >> 16) > 1)
        return 0;

    // adjust the sign according to the direction of motion
    if (dir)
        timestamp_interval = -timestamp_interval;

    // scale down to units of 16us
    return timestamp_interval / 256.f;
}

/*---------------------------Private Function Definitions---------------------*/
static void InitTimer4Timer5(void) {
    T4CON = T5CON = 0x00;   // clear timers configuration
    TMR4 = TMR5 = 0;        // reset both timers to 0
    PR4 = PR5 = UINT16_MAX; // set timer maximum count.
    T4CONbits.T32 = 1;      // mark timers as forming a 32-bit timer pair
    T4CONbits.TON = 1; // turn on the timer (don't need to enable T5, it is automatically enabled)
}

static void InitIC1(uint8_t RPn) {
    IC1CON1 = 0x00; // reset the input capture module

    // set up the input capture for "Normal Configuration"
    // configure the input capture
    IC1CON1bits.ICTSEL = 0b010; // use Timer4 as the time base
    IC1CON1bits.ICI = 0b00;     // fire the interrupt every capture event
    IC1CON1bits.ICM = 0b011;    // capture event on every rising edge

    _IC1R = RPn; // set the input pin to watch
    _IC1IF = 0;  // begin with the interrupt flag cleared
    _IC1IE = 1;  // enable this interrupt
}

static void InitIC2(uint8_t RPn) {
    IC2CON1 = 0x00; // reset the input capture module

    IC2CON1bits.ICTSEL = IC_SELECT_TIMER4;
    IC2CON1bits.ICI = 0b00;
    IC2CON1bits.ICM = 0b011;

    _IC2R = RPn;
    _IC2IF = 0;
    _IC2IE = 1;
}

static void InitIC3(uint8_t RPn) {
    IC3CON1 = 0x00; // reset the input capture module

    IC3CON1bits.ICTSEL = IC_SELECT_TIMER4;
    IC3CON1bits.ICI = 0b00;
    IC3CON1bits.ICM = 0b011;

    _IC3R = RPn;
    _IC2IF = 0;
    _IC2IE = 1;
}

void MotorsInit() {

    // set directions of pins
    M1_DIR_EN(1);
    M1_BRAKE_EN(1);
    M1_MODE_EN(1);
    M1_COAST_EN(1);

    M2_DIR_EN(1);
    M2_BRAKE_EN(1);
    M2_MODE_EN(1);
    M2_COAST_EN(1);

    M3_DIR_EN(1);
    M3_BRAKE_EN(1);
    M3_MODE_EN(1);
    M3_COAST_EN(1);

    MotorChannel i;
    for (EACH_MOTOR_CHANNEL(i))
        Coasting(i);
    M1_MODE = 1;
    M2_MODE = 1;
    M3_MODE = 1;
    PWM1Ini();
    PWM2Ini();
    PWM3Ini();
}

void PWM1Ini() {
    // 1. Configure the OCx output for one of the
    // available Peripheral Pin Select pins.
    // done in I/O Init.
    // 2. Calculate the desired duty cycles and load them
    // into the OCxR register.
    OC1R = 0;
    // 3. Calculate the desired period and load it into the
    // OCxRS register.
    /// period in units of ticks of the timer specified by the clock source OCTSEL
    OC1RS = 2000;
    // 4. Select the current OCx as the sync source by writing
    // 0x1F to SYNCSEL<4:0> (OCxCON2<4:0>),
    // and clearing OCTRIG (OCxCON2<7>).
    /// 0b11111 = This OC1 Module
    OC1CON2bits.SYNCSEL = 0x1F;
    /// 0 = Synchronize OC1 with Source designated with SYNCSEL1 bits
    OC1CON2bits.OCTRIG = CLEAR;
    // 5. Select a clock source by writing the
    // OCTSEL<2:0> (OCxCON<12:10>) bits.
    /// 0b000 = Timer2
    OC1CON1bits.OCTSEL = 0b000;
    // 6. Enable interrupts, if required, for the timer and
    // output compare modules. The output compare
    // interrupt is required for PWM Fault pin utilization.
    // No interrupt needed
    // 7. Select the desired PWM mode in the OCM<2:0>
    //(OCxCON1<2:0>) bits.
    /// 0b110 = Edge-Aligned PWM mode: Output set high when OCxTMR = 0 and set low when OCxTMR =
    /// OCxR
    OC1CON1bits.OCM = 0b110;
    // 8. If a timer is selected as a clock source, set the
    // TMRy prescale value and enable the time base by
    // setting the TON (TxCON<15>) bit.
    // Done in timer Init.
}

static void PWM1Duty(uint16_t Duty) { OC1R = Duty * 2; }

void PWM2Ini() {
    OC2R = 0;
    OC2RS = 2000;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.OCTRIG = CLEAR;
    OC2CON1bits.OCTSEL = 0b000; // Timer2
    OC2CON1bits.OCM = 0b110;
}

static void PWM2Duty(uint16_t Duty) { OC2R = Duty * 2; }

void PWM3Ini() {
    OC3R = 0;
    OC3RS = 2000;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.OCTRIG = CLEAR;
    OC3CON1bits.OCTSEL = 0b000; // Timer2
    OC3CON1bits.OCM = 0b110;
}

static void PWM3Duty(uint16_t Duty) { OC3R = Duty * 2; }

void Braking(MotorChannel Channel) {
    switch (Channel) {
    case MOTOR_LEFT:
        PWM1Duty(0);
        M1_COAST = Clear_ActiveLO;
        M1_BRAKE = Set_ActiveLO;
        break;
    case MOTOR_RIGHT:
        PWM2Duty(0);
        M2_COAST = Clear_ActiveLO;
        M2_BRAKE = Set_ActiveLO;
        break;
    case MOTOR_FLIPPER:
        PWM3Duty(0);
        M3_COAST = Clear_ActiveLO;
        M3_BRAKE = Set_ActiveLO;
        break;
    }
}

void Coasting(MotorChannel channel) {
    switch (channel) {
    case MOTOR_LEFT:
        PWM1Duty(0);
        M1_COAST = Set_ActiveLO;
        M1_BRAKE = Clear_ActiveLO;
        break;
    case MOTOR_RIGHT:
        PWM2Duty(0);
        M2_COAST = Set_ActiveLO;
        M2_BRAKE = Clear_ActiveLO;
        break;
    case MOTOR_FLIPPER:
        PWM3Duty(0);
        M3_COAST = Set_ActiveLO;
        M3_BRAKE = Clear_ActiveLO;
        break;
    }
}

void UpdateSpeed(MotorChannel channel, int16_t effort) {
    uint16_t duty = abs(effort);

    switch (channel) {
    case MOTOR_LEFT:
        M1_COAST = Clear_ActiveLO;
        M1_BRAKE = Clear_ActiveLO;
        M1_DIR = (effort >= 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
        PWM1Duty(duty);
        break;
    case MOTOR_RIGHT:
        M2_COAST = Clear_ActiveLO;
        M2_BRAKE = Clear_ActiveLO;
        M2_DIR = (effort >= 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
        PWM2Duty(duty);
        break;
    case MOTOR_FLIPPER:
        M3_COAST = Clear_ActiveLO;
        M3_BRAKE = Clear_ActiveLO;
        M3_DIR = (effort >= 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE;
        PWM3Duty(duty);
        break;
    }
}
