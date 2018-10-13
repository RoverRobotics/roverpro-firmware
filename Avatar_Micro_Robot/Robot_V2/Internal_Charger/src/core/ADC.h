/*==============================================================================
File: ADC.h 

Description: This module provides an interface to the 10-bit hardware 
	analog-to-digital converter, AD1, of a PIC24F.
	
Notes:
	- Warning: does no error checking on initialization bit mask

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef ADC_H
#define ADC_H
/*---------------------------Dependencies-------------------------------------*/
#include "./StandardHeader.h"

/*---------------------------Type Definitions---------------------------------*/
typedef uint16_t kADCReading;

/*---------------------------Public Function Prototypes-----------------------*/
/*******************************************************************************
Function: ADC_Init
Parameters:
	uint16_t bit_mask,      a number interpreted as binary that describes which
                          of the 0-though-15 analog pins are desired to be
                          analog inputs
Usage: ADC_Init(0b0011); // initialize AN1, AN0
       ADC_Init((1 << THERMISTOR_PIN) |
                (1 << MOTOR_CURRENT_PIN));
*******************************************************************************/
void ADC_Init(uint16_t bit_mask);


/*******************************************************************************
Function: ADC_value
Parameters:
	uint8_t analog_input_index,   number of the analog pin from which
                                to get the result
Description: Returns the last updated value in the buffer for the given pin in 
  arbitrary units [au].  Note that this MAY NOT necessarily be the most current
	value
*******************************************************************************/
uint16_t ADC_value(uint8_t analog_input_index);


/*******************************************************************************
Function: ADC_Deinit
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void ADC_Deinit(void);

#endif
