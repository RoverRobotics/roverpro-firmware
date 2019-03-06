#include "main.h"
#include "analog.h"
#include "hardware_definitions.h"
#include "xc.h"
#include "math.h"

/// Number of samples to keep of running metrics for power management, like battery temperature and
/// voltage
#define ADC_SAMPLE_LENGTH 4
uint16_t adc_samples[ANALOG_CHANNEL_COUNT][ADC_SAMPLE_LENGTH] = {{0}};
size_t i_adc_sample = 0;

/// Configure and start analog/digital converter
void analog_init() {
    // make sure we start off in a default state
    AD1CON1 = 0x0000;
    AD1CON2 = 0x0000;
    AD1CON3 = 0x0000;

    // 1. Configure the A/D module:
    // a) Configure port pins as analog inputs and/or
    // select band gap reference inputs (AD1PCFGL<15:0> and AD1PCFGH<1:0>).
    // initialize all of the analog inputs
    M1_TEMP_EN(1);
    M1_CURR_EN(1);
    M2_TEMP_EN(1);
    M2_CURR_EN(1);
    M3_TEMP_EN(1);
    M3_CURR_EN(1);
    VCELL_A_EN(1);
    VCELL_B_EN(1);
    CELL_A_CURR_EN(1);
    CELL_B_CURR_EN(1);
    M3_POS_FB_1_EN(1);
    M3_POS_FB_2_EN(1);

    // b) Select voltage reference source to match
    // expected range on analog inputs (AD1CON2<15:13>).
    AD1CON2bits.VCFG = 0b000; // VR+: AVDD, VR-: AVSS
    // c) Select the analog conversion clock to
    // match desired data rate with processor clock (AD1CON3<7:0>).
    // Assuming FCY = 16 MHz, TCY = 1/FCY = 62.5 ns
    AD1CON3bits.ADRC = 0; // 0 = system clock, 1 = ADC internal RC clock
    AD1CON3bits.ADCS = 1; // only valid when ADRC =
    // TAD = TCY * (ADCS+1) = 125ns; at least 75ns required
    // d) Select the appropriate sample
    // conversion sequence (AD1CON1<7:5> and AD1CON3<12:8>).
    AD1CON1bits.SSRC = 0b111; // auto conversion
    AD1CON3bits.SAMC = 15;    // # TAD clock cycles in between start of sampling and conversion
    // auto-sample time = 15*TAD=1.875uS
    // e) Select how conversion results are
    // presented in the buffer (AD1CON1<9:8>).
    AD1CON1bits.FORM = 0b00; // integer (0000 00dd dddd dddd)
    // f) Select interrupt rate (AD1CON2<5:2>).
    //_AD1IE = 1;
    AD1CON2bits.SMPI = 11; // interrupt every SMPI+1 samples convert sequence
    // g) scan mode, select input channels (AD1CSSL<15:0>)
    // this will sequentially sample them into the buffers ADC1BUF0...
    // 0b1111111100001111 means
    // AN0...AN3 -> BUF0..BUF3;
    // AN4...AN7 ignored
    // AN8...AN15 -> BUF4..BUFB
    AD1CSSL = 0b1111111100001111;

    AD1CON2bits.CSCNA = 1;
    AD1CON1bits.ASAM = 1;
    // h) Turn on A/D module (AD1CON1<15>).
    AD1CON1bits.ADON = 1;

    // conversion takes time 12 TAD
    // so total time = (ADCS+1+SAMC+12)*TAD*numbits(adcssl) = (1+1+15)*62.5ns*12 = 12.75 us
}

static uint16_t get_sample(AnalogChannel c) {
    // on ADC but not used:
    // ADC1BUF0 <- AN0 = M2_Temperature
    // ADC1BUF2 <- AN2 = M1_Temperature
    // ADC1BUFA <- AN14 = M3_Temperature
    switch (c) {
    case ADC_MOTOR_LEFT_CURRENT:
        return ADC1BUF3;
    case ADC_MOTOR_RIGHT_CURRENT:
        return ADC1BUF1;
    case ADC_MOTOR_FLIPPER_CURRENT:
        return ADC1BUFB;
    case ADC_CELL_A_CURRENT:
        return ADC1BUF8;
    case ADC_CELL_B_CURRENT:
        return ADC1BUF9;
    case ADC_CELL_A_VOLTAGE:
        return ADC1BUF6;
    case ADC_CELL_B_VOLTAGE:
        return ADC1BUF7;
    case ADC_FLIPPER_POTENTIOMETER_A:
        return ADC1BUF4;
    case ADC_FLIPPER_POTENTIOMETER_B:
        return ADC1BUF5;
    }
    BREAKPOINT();
    return 0;
}

void analog_tick() {
    BREAKPOINT_IF(!IFS0bits.AD1IF);

    AnalogChannel c;
    for (c = 0; c < ANALOG_CHANNEL_COUNT; c++) {
        adc_samples[c][i_adc_sample] = get_sample(c);
    }
    i_adc_sample = (i_adc_sample + 1) % ADC_SAMPLE_LENGTH;

    g_state.analog.motor_current[MOTOR_LEFT] = analog_get_value(ADC_MOTOR_LEFT_CURRENT);
    g_state.analog.motor_current[MOTOR_RIGHT] = analog_get_value(ADC_MOTOR_RIGHT_CURRENT);
    g_state.analog.motor_current[MOTOR_FLIPPER] = analog_get_value(ADC_MOTOR_FLIPPER_CURRENT);

    g_state.analog.battery_voltage[BATTERY_A] = analog_get_value(ADC_CELL_A_VOLTAGE);
    g_state.analog.battery_voltage[BATTERY_B] = analog_get_value(ADC_CELL_B_VOLTAGE);

    g_state.analog.battery_current[BATTERY_A] = analog_get_value(ADC_CELL_A_CURRENT);
    g_state.analog.battery_current[BATTERY_B] = analog_get_value(ADC_CELL_B_CURRENT);

    g_state.analog.flipper_sensors[0] = analog_get_value(ADC_FLIPPER_POTENTIOMETER_A);
    g_state.analog.flipper_sensors[1] = analog_get_value(ADC_FLIPPER_POTENTIOMETER_B);

    IFS0bits.AD1IF = 0;
}

uint16_t analog_get_value(AnalogChannel c) {
    uint32_t total = 0;
    size_t i;
    for (i = 0; i < ADC_SAMPLE_LENGTH; i++) {
        total += adc_samples[c][i];
    }
    uint32_t mean = total / ADC_SAMPLE_LENGTH;
    BREAKPOINT_IF(mean > UINT16_MAX);
    return mean;
}