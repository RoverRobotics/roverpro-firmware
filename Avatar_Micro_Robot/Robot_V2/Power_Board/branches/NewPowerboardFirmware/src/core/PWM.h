/*==============================================================================
File: PWM.h 

Description: This file provides an interface to the PWM hardware modules
  available on the PIC24FJ256GB106.  As this was originally designed to drive
  motors, the scale of configurable PWM periods may be too small for some
  applications, most notably controlling servos.
  
Notes:
  - uses Timer2 as the time base for all channels
  - only remappable pins can be configured for PWM
  - assumes F_CY = 16MHz
  - performs NO error checking on input parameters
	- can NOT go slower than T_PWM = 64us
	
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef PWM_H
#define PWM_H
/*---------------------------Dependencies-------------------------------------*/
#include "./StandardHeader.h"

/*---------------------------Type Definitions---------------------------------*/
// PWM hardware module options
typedef enum {
	kPWM01 = 0,
	kPWM02,
	kPWM03,
	kPWM04,
	kPWM05,
	kPWM06,
	kPWM07,
	kPWM08,
	kPWM09
} kPWMModule;

/*---------------------------Public Function Prototypes-----------------------*/
/*******************************************************************************
Function: PWM_Init
Parameters:
  const kPWMModule module,  which of the available hardware modules  
	const uint8_t RPn,        the remappable pin number on which to output
	const uint16_t us, 	      the period of the PWM signal in microseconds
Description: Initializes the output compare module and begins outputting at 0% 
	on the specified pin.
*******************************************************************************/
void PWM_Init(const kPWMModule module, const uint8_t RPn, const uint16_t us);


/*******************************************************************************
Function: PWM_UpdateDutyCycle
Parameters:
  const kPWMModule module,  which of the available hardware modules
	const float duty_cycle, 	the percent high-time (0.0-to-1.0 inclusive)
Notes:
	- does NOT check whether the given pin is valid and has been 
    previously initialized
*******************************************************************************/
void inline PWM_UpdateDutyCycle(const kPWMModule module, 
                                const float duty_cycle);


/*******************************************************************************
Function: PWM_Deinit
Parameters:
  const kPWMModule module,  which of the available hardware modules  
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void PWM_Deinit(const kPWMModule module);

#endif
