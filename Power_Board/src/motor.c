/*---------------------------Dependencies-------------------------------------*/
#include "main.h"
#include "motor.h"
#include "hardware_definitions.h"

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

/// Motor direction constants for M1_DIRO bits as well as other motors
typedef enum {
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_REVERSE = 0,
} MotorDir;

/// Set up and start Timer2: 30 kHz, no interrupts.
/// Timer2 is used as the clock source for motor PWM
void IniTimer2() {
    T2CON = 0x0000;         // stops timer2,16 bit timer,internal clock (Fosc/2)
    T2CONbits.TCKPS = 0b00; // 0b00 = 1:1 prescale
    TMR2 = 0;
    uint16_t FREQUENCY_HZ = 30000;
    PR2 = (FCY / FREQUENCY_HZ) - 1;
    IFS0bits.T2IF = 0; // clear interrupt flag
    T2CONbits.TON = 1; // start timer
}

/// A single event of the motor encoder. Three events in the same direction correspond to one revolution.
typedef struct {
    /// 32-bit timestamp high half
    uint16_t hi;
    /// 32-bit timestamp low half
    uint16_t lo;
    /// direction that the motor is spinning
    MotorDir dir;
} EncoderEvent;

/// Index into the event_ring_buffer for the next event capture
static volatile size_t i_next_event[MOTOR_CHANNEL_COUNT];

/// The most recent encoder events. We only need 2 encoder events to compute an interval,
/// and old values should be purged.
// todo: change this to a LIFO data structure
static volatile EncoderEvent event_ring_buffer[MOTOR_CHANNEL_COUNT][CAPTURE_BUFFER_COUNT];

/*---------------------------Interrupt Service Routines (ISRs)----------------*/

/// Get a single event and note it in the buffer.
void motor_tach_event_capture(MotorChannel channel, MotorDir dir, uint16_t captured_value_lo) {
    EncoderEvent event;

    event.lo = captured_value_lo;
    // note reading TMR4 saves the value of TMR5 into TMR5HLD.
    // This is needed since reading TMR4 then TMR5 is not atomic
    if (TMR4 >= captured_value_lo) {
        // TMR4 did not yet carry into TMR5
        event.hi = TMR5HLD;
    } else {
        // TMR4 did carry into TMR5
        // We assume it only overflowed once, that is timer4 did not go through a whole
        // 2**16 * PCY = 2**16 / 16 MHz = 4 milliseconds cycle
        event.hi = TMR5HLD - 1;
    }

    event.dir = dir;
    event_ring_buffer[channel][i_next_event[channel]] = event;
    i_next_event[channel] = (i_next_event[channel] + 1u) % CAPTURE_BUFFER_COUNT;
}

/// Interrupt function for PIC24 Input Capture module 1
void __attribute__((__interrupt__, auto_psv)) _IC1Interrupt(void) {
    MotorDir dir = (M1_DIRO ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD);
    while (IC1CON1bits.ICBNE) {
        motor_tach_event_capture(MOTOR_LEFT, dir, IC1BUF);

        if (dir == MOTOR_DIR_REVERSE)
            g_state.drive.motor_encoder_count[MOTOR_LEFT]--;
        else
            g_state.drive.motor_encoder_count[MOTOR_LEFT]++;
    }
    _IC1IF = 0; // clear the source of the interrupt
}

/// Interrupt function for PIC24 Input Capture module 2
void __attribute__((__interrupt__, auto_psv)) _IC2Interrupt(void) {
    // Note the motor direction is reversed here, since the motor is installed backwards
    MotorDir dir = (M2_DIRO ? MOTOR_DIR_FORWARD : MOTOR_DIR_REVERSE);
    while (IC2CON1bits.ICBNE) {
        motor_tach_event_capture(MOTOR_RIGHT, dir, IC2BUF);

        if (dir == MOTOR_DIR_REVERSE)
            g_state.drive.motor_encoder_count[MOTOR_RIGHT]--;
        else
            g_state.drive.motor_encoder_count[MOTOR_RIGHT]++;
    }
    _IC2IF = 0;
}

// Interrupt function for PIC24 Input Capture module 3
void __attribute__((__interrupt__, auto_psv)) _IC3Interrupt(void) {
    MotorDir dir = (M3_DIRO ? MOTOR_DIR_REVERSE : MOTOR_DIR_FORWARD);
    while (IC2CON1bits.ICBNE) {
        motor_tach_event_capture(MOTOR_FLIPPER, dir, IC3BUF);
    }
    _IC3IF = 0;
}

/*---------------------------Public Function Definitions----------------------*/
void motor_tach_init() {
    /// We use Timer4 + Timer5 as the basis of our tachometry.
    /// with a 16 MHz instruction clock a 256 prescale, and a 16 bit input capture, we can measure
    /// as fast as 1/(16MHz) * 256 = 16 us ~= 3.8 M RPM
    /// as slow as 1/(16MHz) * 256 * 2**16  = 1.1 s ~= 57 RPM
    InitTimer4Timer5();

    InitIC1(M1_TACHO_RPN);
    InitIC2(M2_TACHO_RPN);
    InitIC3(M3_TACHO_RPN);
}

MotorStatusFlag motor_update(MotorChannel channel, MotorStatusFlag status, uint16_t duty) {
	// NOTE: all the hardware flags are active-low. So we negate the values before assigning and when retrieving
	switch (channel){
		case (MOTOR_LEFT):
			M1_COAST = !(status & MOTOR_FLAG_COAST);
			M1_BRAKE = !(status & MOTOR_FLAG_BRAKE);
			M1_DIR   = !(status & MOTOR_FLAG_REVERSE);
			M1_MODE  = !(status & MOTOR_FLAG_DECAY_MODE);
			PWM1Duty(duty);
			return (status & ~MOTOR_FLAG_MASK_FEEDBACK) | (MOTOR_FLAG_FAULT1 * !M1_FF1) | (MOTOR_FLAG_FAULT2 * !M1_FF2);
		case (MOTOR_RIGHT):
			M2_COAST = !(status & MOTOR_FLAG_COAST);
			M2_BRAKE = !(status & MOTOR_FLAG_BRAKE);
			M2_DIR =   !(status & MOTOR_FLAG_REVERSE);
			M2_MODE =  !(status & MOTOR_FLAG_DECAY_MODE);
			PWM2Duty(duty);
			return (status & ~MOTOR_FLAG_MASK_FEEDBACK) | (MOTOR_FLAG_FAULT1 * !M2_FF1) | (MOTOR_FLAG_FAULT2 * !M2_FF2);
		case (MOTOR_FLIPPER):
			M3_COAST = !(status & MOTOR_FLAG_COAST);
			M3_BRAKE = !(status & MOTOR_FLAG_BRAKE);
			M3_DIR =   !(status & MOTOR_FLAG_REVERSE);
			M3_MODE =  !(status & MOTOR_FLAG_DECAY_MODE);
			PWM3Duty(duty);
			return (status & ~MOTOR_FLAG_MASK_FEEDBACK) | (MOTOR_FLAG_FAULT1 * !M3_FF1) | (MOTOR_FLAG_FAULT2 * !M3_FF2);
	}
	BREAKPOINT();
	return 0;
}	

float motor_tach_get_period(MotorChannel channel) {
    BREAKPOINT_IF(channel > MOTOR_CHANNEL_COUNT);
    // We are betting we can get through this before the data we're interested in is clobbered.

    // index of second-most-recent event
    size_t i = (i_next_event[channel] + CAPTURE_BUFFER_COUNT - 2) % CAPTURE_BUFFER_COUNT;

    EncoderEvent event0 = event_ring_buffer[channel][i];
    EncoderEvent event1 = event_ring_buffer[channel][(i + 1) % CAPTURE_BUFFER_COUNT];
    uint16_t interval = event1.lo - event0.lo;

    // if motor changed direction, can't trust encoder
    if (event0.dir != event1.dir)
        return 0;

    // if it's been too long since last timestamp, can't trust encoder
    if (TMR5 - event1.hi > 1)
        return 0;

    // if the motor is moving very slowly (<~1hz)
    if (event1.hi - event0.hi > 1)
        return 0;

    if (event1.hi - event0.hi == 1 && event1.lo > event0.lo)
        return 0;

    if (event1.dir == MOTOR_DIR_REVERSE)
        return -(float)interval;
    else
        return +(float)interval;
}

/*---------------------------Private Function Definitions---------------------*/
static void InitTimer4Timer5(void) {
    T4CON = T5CON = 0x00; // clear timers configuration
    TMR4 = 0;             // reset both timers to 0
    TMR5 = 0;
    PR4 = UINT16_MAX; // set timer maximum count.
    PR5 = UINT16_MAX;
    T4CONbits.TCKPS = 0b11; // prescale by 1:256
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
    // todo: move this to drive_init
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

    MotorChannel c;
    for (EACH_MOTOR_CHANNEL(c))
        motor_update(c, MOTOR_FLAG_COAST, 0);
    M1_MODE = 1;
    M2_MODE = 1;
    M3_MODE = 1;

    // Assign OC1 To Pin M1_AHI
    M1_PWM = 18; // 18 represents OC1

    // Assign OC2 To Pin M1_BHI
    M2_PWM = 19; // 19 represents OC2

    // Assign OC3 To Pin M2_AHI
    M3_PWM = 20; // 20 represents OC3

    IniTimer2();
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
    /// period in units of ticks of the timer \ecified by the clock source OCTSEL
    OC1RS = 2000;
    // 4. Select the current OCx as the sync source by writing
    // 0x1F to SYNCSEL<4:0> (OCxCON2<4:0>),
    // and clearing OCTRIG (OCxCON2<7>).
    /// 0b11111 = This OC1 Module
    OC1CON2bits.SYNCSEL = 0x1F;
    /// 0 = Synchronize OC1 with Source designated with SYNCSEL1 bits
    OC1CON2bits.OCTRIG = 0;
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
    OC2CON2bits.OCTRIG = 0;
    OC2CON1bits.OCTSEL = 0b000; // Timer2
    OC2CON1bits.OCM = 0b110;
}

static void PWM2Duty(uint16_t Duty) { OC2R = Duty * 2; }

void PWM3Ini() {
    OC3R = 0;
    OC3RS = 2000;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.OCTRIG = 0;
    OC3CON1bits.OCTSEL = 0b000; // Timer2
    OC3CON1bits.OCM = 0b110;
}

static void PWM3Duty(uint16_t Duty) { OC3R = Duty * 2; }
