/*=============================================================================
File: PWM.h

Description: This module provides an interface to a hardware pulse-width
  modulation (PWM) module within the ATmega328.

Notes:
  - assumes a 16MHz oscillator
  - assumes exclusive use of Timer2 within the microcontroller
  - configures for Phase Correct PWM Mode
  - assumes exclusive use of arduino pins 3 and 11 
    (PORB1 and PORTB2 on the microcontroller)
  - performs no error checking on input parameters
 
 Author: Stellios Leventis (stellios.leventis@stanford.edu)
=============================================================================*/
#pragma once

/*****************************************************************************
  Function: InitPWM(unsigned int period)
  Parameters:
    unsigned int period,  period in units of [8us] at which to PWM
  Notes:
    - this function must be called before using any of the library functions
    - the units of period are increments of 8us
****************************************************************************/
void InitPWM(unsigned int period);

/****************************************************************************
  Function: UpdateDutyCycle()
  Parameters:
    unsigned char pin, which of the previously initialized pins to update
    double percent, the new duty cycle as a percentage (0-to-100 inclusive)
****************************************************************************/
void UpdateDutyCycle(unsigned char pin, unsigned char percent);
