/*==============================================================================
File: InputCapture.h 

Description: This file provides an interface to up to nine (9) input capture 
 hardware modules on the PIC24FJ256GB106 to facilitate the capturing of 
 external pulse events.
 
Notes:
  - do NOT compete with PWM pins
  - uses hardware Timer3 as its time base
  - can measure signals up to about 60kHz (TODO: confirm this)
  - assumes the standard configuration bits and a 20MHz external resonator
	
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef INPUT_CAPTURE_H
#define INPUT_CAPTURE_H
/*---------------------------Dependencies-------------------------------------*/
#include "./StandardHeader.h"

/*---------------------------Type Definitions---------------------------------*/
// input capture hardware module options
typedef enum {
	kIC01 = 0,
	kIC02,
	kIC03,
	kIC04,
	kIC05,
	kIC06,
	kIC07,
	kIC08,
	kIC09,
} kICModule;

/*---------------------------Public Function Prototypes-----------------------*/
/*******************************************************************************
Function: IC_Init
Parameters:
  const kICModule module,   the input capture hardware module to use
  const uint8_t RPn,        the remappable pin number to monitor
  const uint8_t timeout,    the dead time in milliseconds after which the 
                      periods is reset to zero (0)
Notes:
  - BUG ALERT: ensure you do NOT overflow whatever stores the number of events
    for your chosen   
*******************************************************************************/
void IC_Init(const kICModule module, 
             const uint8_t RPn, 
             const uint8_t timeout);


/*******************************************************************************
Function: IC_frequency
Parameters:
  const kICModule module,   the input capture hardware module to use
Description: Returns the last-computed period in units of 16-microseconds (ms).  
  Returns zero (0) if no rising edges have been detected for more than the 
  chosen timeout time.
*******************************************************************************/
float IC_period(const kICModule module);

/*******************************************************************************
Function: IC_Deinit
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void IC_Deinit(void);

#endif
