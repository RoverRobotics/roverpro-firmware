/**
 * @file periph_adc.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * ADC firmware driver for Microchip PIC24F
 *
 *
 *
 * Firmware MUST configure/enable it's ADC hardware AND enable interrupts
 */

#include <string.h>
#include "p24Fxxxx.h"

#include "periph_adc.h"

// max number of I2C bus callbacks
#define ADC_MAX_CALLBACKS 2 /*functions*/

struct ADC_CALLBACKS
{
   ADC_FUNC func;
   unsigned char addr_low;
   unsigned char addr_high;
} __adc_callbacks[ADC_MAX_CALLBACKS] = { { 0 } };

unsigned int __adc_callback_pos = 0;
   

// ADC driver states
typedef enum ADC_STATES_T
{
   ADC_IDLE = 0,

   ADC_COMPLETE = 1
} ADC_STATES;


// ****************************************************************************
// DRIVER
// ****************************************************************************

struct ADC_DRIVER
{
   ADC_STATES state;
   unsigned char addr;
   unsigned int  data;
} __adc_driver = { 0 };


// ****************************************************************************
// API DEFINITIONS
// ****************************************************************************


int registerADCReadCallback(ADC_FUNC func, unsigned char addr)
{
   if (__adc_callback_pos >= ADC_MAX_CALLBACKS) return -1;

   __adc_callbacks[__adc_callback_pos].func = func;
   __adc_callbacks[__adc_callback_pos].addr_low  = addr;
   __adc_callbacks[__adc_callback_pos].addr_high = addr;

   __adc_callback_pos++;

   return 0;
}

int registerADCReadCallbackRange(ADC_FUNC func, unsigned char addr_low,
                                                unsigned char addr_high)
{
   if (__adc_callback_pos >= ADC_MAX_CALLBACKS) return -1;

   __adc_callbacks[__adc_callback_pos].func = func;
   __adc_callbacks[__adc_callback_pos].addr_low  = addr_low;
   __adc_callbacks[__adc_callback_pos].addr_high = addr_high;

   __adc_callback_pos++;

   return 0;
}


int readADC(unsigned char addr)
{
   _CH0NA = addr;
   _SAMP = 1;
}

