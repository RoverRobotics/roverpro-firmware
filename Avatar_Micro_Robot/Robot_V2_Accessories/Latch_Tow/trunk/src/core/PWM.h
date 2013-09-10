/*==============================================================================
File: PWM.h 

Description: This file provides an interface to two PWM pins.

Notes:
  - only remappable pins can be configured for PWM
  - assumes F_CY = 16MHz
  - uses Timer2 as the time base
	- uses output compare 1 and output compare 2
  - performs NO error checking on input parameters
	
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef PWM_H
#define PWM_H
/*---------------------------Public Function Prototypes-----------------------*/
/*******************************************************************************
Function: PWM_Init
Parameters:
	unsigned char pwm1RPn, 	the first remappable pin number on which to output
	unsigned char pwm2RPn,	the second remappable pin number on which to output
	unsigned int ms, 			 	the period of both PWM channels in milliseconds
Description: Initializes the output compare modules and begins outputting at 0% 
	on the specified pins.  To NOT initialize a pin, enter 0 for the RPn number
Usage: PWM_Init(20, 25, 1000); // configure RP20 and RP25 as PWM with a period 
                               // of 1000ms => 1s => 1Hz
*******************************************************************************/
void PWM_Init(unsigned char pwm1RPn, unsigned char pwm2RPn, unsigned int ms);


/*******************************************************************************
Function: PWM_UpdateDutyCycle
Parameters:
	float dutyCycle, 		the percent high-time (0-to-100 inclusive)
	unsigned char pin, 	the remappable pin number to update
Notes:
	- does NOT check whether the given pin is valid and has been 
    previously initialized
*******************************************************************************/
void inline PWM_UpdateDutyCycle(unsigned char pin, float dutyCycle);


/*******************************************************************************
Function: PWM_Deinit
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void PWM_Deinit(void);

#endif
