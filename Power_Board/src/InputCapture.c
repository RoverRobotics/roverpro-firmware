/*==============================================================================
File: InputCapture.c
==============================================================================*/
//#define TEST_INPUT_CAPTURE
/*---------------------------Dependencies-------------------------------------*/
#include "./InputCapture.h"
#include <limits.h>  // for UINT_MAX macro
#include <stdbool.h> // for 'bool' boolean data type
#include "stdhdr.h"
#include <p24Fxxxx.h>
#include <device_robot_motor.h>

/*---------------------------Macros-------------------------------------------*/
#define MAX_NUM_IC_PINS 9

#define T4_TICKS_PER_MS 4 // milliseconds per timer4 tick
// TODO: IS THIS RIGHT????

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitTimer4(void);
static void InitIC1(const uint8_t RPn);
static void InitIC2(const uint8_t RPn);
static void InitIC3(const uint8_t RPn);
static void InitIC4(const uint8_t RPn);
static void InitIC5(const uint8_t RPn);
static void InitIC6(const uint8_t RPn);
static void InitIC7(const uint8_t RPn);
static void InitIC8(const uint8_t RPn);
static void InitIC9(const uint8_t RPn);

void IC1_ISR(void);
void IC2_ISR(void);

/*---------------------------Module Variables---------------------------------*/
static volatile bool is_timer3_running = false;
static volatile uint8_t RPns[MAX_NUM_IC_PINS] = {0};
static volatile uint32_t timeouts[MAX_NUM_IC_PINS] = {0}; // in units of [ms]
static volatile uint32_t elapsed_times[MAX_NUM_IC_PINS] = {0};
static volatile float periods[MAX_NUM_IC_PINS] = {0};
static volatile uint32_t time = 0; // running number of timer3 ticks

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_INPUT_CAPTURE
#include "./ConfigurationBits.h"

int main(void) {
    uint8_t RPn = 27;
    uint8_t timeout = 10; // [ms]
    IC_Init(kIC01, RPn, timeout);

    float my_square_wave_period;
    while (1) {
        Delay(1000);
        IC_UpdatePeriods();
        my_square_wave_period = IC_period(module);
    }

    return 0;
}
#endif

/*---------------------------Interrupt Service Routines (ISRs)----------------*/
void T4_ISR(void) {
    _T4IF = 0; // clear the source of the interrupt
    time++;
}

void IC1_ISR(void) {
    _IC1IF = 0; // clear the source of the interrupt
    elapsed_times[0] = 0;

    static uint16_t last_value = 0;
    uint16_t current_value = IC1BUF; // current running Timer3 tick value
    // (you must subtract off last value)

    // handle rollover, remove old offset
    if (last_value < current_value)
        periods[0] = current_value - last_value;
    else
        periods[0] = (UINT_MAX - last_value) + current_value;

    REG_MOTOR_FB_PERIOD_LEFT = periods[0];
    last_value = current_value;
    // increase encoder count if motor is moving forward, decrease if backward
    if (M1_DIRO)
        REG_MOTOR_ENCODER_COUNT.left++;
    else
        REG_MOTOR_ENCODER_COUNT.left--;
}

void IC2_ISR(void) {
    _IC2IF = 0;
    elapsed_times[1] = 0;

    static uint16_t last_value = 0;
    uint16_t current_value = IC2BUF;

    if (last_value < current_value)
        periods[1] = (current_value - last_value);
    else
        periods[1] = (UINT_MAX - last_value) + current_value;
    REG_MOTOR_FB_PERIOD_RIGHT = periods[1];
    last_value = current_value;
    // increase encoder count if motor is moving forward, decrease if backward
    if (M2_DIRO)
        REG_MOTOR_ENCODER_COUNT.right++;
    else
        REG_MOTOR_ENCODER_COUNT.right--;
}

/*---------------------------Public Function Definitions----------------------*/
void IC_Init(const kICModule module, const uint8_t RPn, const uint8_t timeout) {
    timeouts[module] = timeout;
    RPns[module] = RPn;

    // Switched timer3 to timer2 (timer 2 not used in regular power board firmware)
    // Timer 4 used in regular power board firmware, but only in IC interrupts.
    if (!is_timer3_running) {
        T5CONbits.TCKPS = 0b11; // configure prescaler to divide-by-256
        _T5IF = 0;
        //_T3IE = 1;
        T5CONbits.TON = 1; // turn on the timer
        is_timer3_running = true;

        InitTimer4();
    }

    // initialize the given input capture hardware module
    switch (module) {
    case kIC01:
        InitIC1(RPn);
        break;
    case kIC02:
        InitIC2(RPn);
        break;
    case kIC03:
        InitIC3(RPn);
        break;
    case kIC04:
        InitIC4(RPn);
        break;
    case kIC05:
        InitIC5(RPn);
        break;
    case kIC06:
        InitIC6(RPn);
        break;
    case kIC07:
        InitIC7(RPn);
        break;
    case kIC08:
        InitIC8(RPn);
        break;
    case kIC09:
        InitIC9(RPn);
        break;
    }
}

float IC_period(const kICModule module) { return periods[module]; }

void IC_UpdatePeriods(void) {
    // reset any periods if it has been too long
    static uint32_t last_time = 0;
    uint32_t current_time = time;
    uint8_t i;
    int32_t delta_time;
    for (i = 0; i < MAX_NUM_IC_PINS; i++) {
        delta_time = (current_time - last_time); // OK ON ROLLOVER?
        elapsed_times[i] += delta_time;
        // NB: be consistent in units of timer4 ticks
        if ((timeouts[i] * T4_TICKS_PER_MS) < elapsed_times[i]) {
            periods[i] = 0;
            elapsed_times[i] = 0;
        }
    }
    last_time = current_time;
}

/*---------------------------Private Function Definitions---------------------*/
static void InitTimer4(void) {
    T4InterruptUserFunction = T4_ISR;
    T4CONbits.TON = 0; // turn off the timer while we configure it
    T4CONbits.TCS = 0; // use the internal, system clock
    PR4 = 0x0fd0;
    T4CONbits.TCKPS = 0b00; // configure prescaler to divide-by-1
    _T4IF = 0;              // begin with the interrupt flag cleared
    _T4IE = 1;              // enable the interrupt
    T4CONbits.TON = 1;      // turn on the timer
}

static void InitIC1(const uint8_t RPn) {
    // clear the input capture FIFO buffer
    uint16_t temp;
    while (IC1CON1bits.ICBNE) {
        temp = IC1BUF;
    };

    // configure the input capture
    // IC1CON1bits.ICTSEL = 0;   // use Timer3 as the time base
    IC1CON1bits.ICTSEL = 0b011; // use Timer5 as the time base
    IC1CON1bits.ICI = 0b00;     // fire the interrupt every capture event
    IC1CON1bits.ICM = 0b011;    // capture event on every rising edge

    // PPS_MapPeripheral(RPn, INPUT, FN_IC1);
    _IC1R = RPn;

    IC1InterruptUserFunction = IC1_ISR;

    _IC1IF = 0; // begin with the interrupt flag cleared
    _IC1IE = 1; // enable this interrupt
}

static void InitIC2(const uint8_t RPn) {
    uint16_t temp;
    while (IC2CON1bits.ICBNE) {
        temp = IC2BUF;
    };

    //  IC2CON1bits.ICTSEL = 0;
    IC2CON1bits.ICTSEL = 0b011; // use Timer5 as the time base
    IC2CON1bits.ICI = 0b00;
    IC2CON1bits.ICM = 0b011;

    IC2InterruptUserFunction = IC2_ISR;

    // PPS_MapPeripheral(RPn, INPUT, FN_IC2);
    _IC2R = RPn;
    _IC2IF = 0;
    _IC2IE = 1;
}

static void InitIC3(const uint8_t RPn) {}

static void InitIC4(const uint8_t RPn) {}

static void InitIC5(const uint8_t RPn) {}

static void InitIC6(const uint8_t RPn) {}

static void InitIC7(const uint8_t RPn) {}

static void InitIC8(const uint8_t RPn) {}

static void InitIC9(const uint8_t RPn) {}
