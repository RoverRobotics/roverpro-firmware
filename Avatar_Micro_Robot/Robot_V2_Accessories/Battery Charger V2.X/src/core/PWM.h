/*==============================================================================
File: PWM.h 

Description: This file provides an interface to the PWM hardware modules
  available on the PIC24FJ256GB106.
  
Notes:
  - uses Timer2 as the time base for all channels
  - only remappable pins can be configured for PWM
  - assumes F_CY = 16MHz
  - performs NO error checking on input parameters
	
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
	const uint16_t ms, 	      the period of the PWM signal in milliseconds
Description: Initializes the output compare module and begins outputting at 0% 
	on the specified pin.  THE PWM PERIOD MUST BE THE SAME FOR ALL CHANNELS
Usage: PWM_Init(kPWM01, 25, 1000); // configure RP25 as PWM with a period 
                                   // of 1000ms => 1s => 1Hz
*******************************************************************************/
void PWM_Init(const kPWMModule module, const uint8_t RPn, const uint16_t ms);


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
