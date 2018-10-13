/*=============================================================================
File: PWM.h 

Description: This file provides an interface to one of several Pulse-Width 
	Modulation (PWM) hardware modules.

Notes:
  - only remappable pins can be configured for PWM
  - assumes a 20MHz external oscillator
  - uses Timer2 as the time base
  - performs no error checking on input parameters
  - optimized for driving a servo motor (better resolution around 100Hz)

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
=============================================================================*/
#ifndef PWM_H
#define PWM_H
/*---------------------------Public Function Prototypes----------------------*/
/*
Function: InitPWM()
Parameters:
	unsigned char remappable_pin_num, the remappable pin numberon which to 
                                    output
	unsigned int ms, the period in units of milliseconds
Description: Initializes the specified PWM hardware module and begins
  outputting at 0% duty cycle on the specified pin.
Usage: InitPWM(21, 1000); // initialize RP21 to PWM at a 
                          // period of 1000ms ~=> 1s 
*/
void InitPWM(unsigned char pin, unsigned int ms);

/*
Function: UpdateDutyCycle()
Parameters:
	unsigned char duty_cycle, the percent high-time (0-to-100 inclusive)
Notes:
	- does NOT check whether the given pin is valid and has been 
    previously initialized
*/
void UpdateDutyCycle(unsigned char duty_cycle);

#endif
