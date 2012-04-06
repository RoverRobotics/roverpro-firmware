/*=============================================================================
File: PWM.h 

Description: This file provides an interface to one of several Pulse-Width 
	Modulation (PWM) hardware modules.

Notes:
  - assumes a 20MHz external oscillator
  - assumes exclusive use of the Timer associated with the given module number
  - configures for Phase Correct PWM Mode
  - assumes exclusive use of the pins associated with the given module number 
  - performs no error checking on input parameters  
=============================================================================*/
#ifndef PWM_H
#define PWM_H
/*---------------------------Public Function Prototypes----------------------*/
/*
Function: InitPWM()
Parameters:
	unsigned char remappable_pin_num, the remappable pin numberon which to 
                                    output
	unsigned int us, the period in units of microseconds
Description: Initializes the specified PWM hardware module and begins
  outputting at 0% duty cycle on the specified pin.
Usage: InitPWM(21, 1000); // initialize RP21 to PWM at a 
                          // period of 1000us => 1ms 
*/
void InitPWM(unsigned char pin, unsigned int us);

/*
Function: UpdateDutyCycle()
Parameters:
	unsigned char duty_cycle, the percent high-time (0-to-100 inclusive)
Notes:
	- does NOT check whether the given pin is valid and has been 
    previously initialized
	- only offers a finest resolution of 1%
*/
void UpdateDutyCycle(unsigned char duty_cycle);

#endif
