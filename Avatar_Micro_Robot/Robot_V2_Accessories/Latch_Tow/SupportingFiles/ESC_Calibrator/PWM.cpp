/*=============================================================================
File: Pulse.cpp
=============================================================================*/
/*---------------------------Dependencies------------------------------------*/
#include "./PWM.h"
#include <Arduino.h>

/*---------------------------Helper Function Prototypes----------------------*/
static void ConfigurePins(void);
static void InitTimer1(unsigned int period);

/*---------------------------Public Function Definitions---------------------*/
void InitPWM(unsigned int period) {
  ConfigurePins();
  InitTimer1(period);
  UpdateDutyCycle(11, 0);
  UpdateDutyCycle(3, 0);
}

void UpdateDutyCycle(unsigned char pin, unsigned char duty_cycle) {
  unsigned int dummy = (unsigned int)((double)duty_cycle / 100 * (double)ICR1);
  if (pin == 3) OCR1A = dummy;
  else if (pin == 11) OCR1B = dummy;
}

/*---------------------------Helper Function Definitions---------------------*/
/*
Function: ConfigurePins()
Description: Configures the associated pins of Timer1 pins as digital outputs.
*/
static void ConfigurePins(void) {
  DDRB |= ((1 << DDB1) | (1 << DDB2));
}

/*
Function: InitTimer1()
Description: Initializes Timer2 to be used as the time base for both PWM
  output pins.
Notes:
  - f_PWM = f_CLK_IO / timer1_prescaler / ICR1 / 2
          = 16MHz / 64 / ICR1 / 2
          = 125kHz / ICR1
    duty_cycle_A = OCR1A / ICR1
    duty_cycle_B = OCR1B / ICR1
*/
static void InitTimer1(unsigned int period) {
  ICR1 = period;
  
  // configure for PWM mode 10, Phase Correct PWM with ICR1 as TOP (see p.136 of datasheet)
  TCCR1B |= (1 << WGM13);
  TCCR1A |= (1 << WGM11);
  TCCR1A &= ~(1 << WGM10); // BUG ALERT: bit WGM10 is NOT 0 by default as it is on power-up.  There  
                           // are additional settings somewhere in Arduino files that change this!!!  
  
  // configure for non-inverting mode on both pins (see Table 15-2, p.135)
  TCCR1A |= (1 << COM1A1); // toggle OC1A on compare match with OCR1A
  TCCR1A |= (1 << COM1B1); // toggle OC1B on compare match with OCR1B
  TCCR1B |= ((1 << CS11) | (1 << CS10));  // configure prescaler to divide-by-64 and start counting
}

/*---------------------------End of File-------------------------------------*/


