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
/*---------------------------Public Function Prototypes-----------------------*/
/*******************************************************************************
Function: ADC_Init
Parameters:
	unsigned int bitMask,  a number interpreted as binary that describes which
  											 of the 0-though-15 analog pins are desired to be
                         analog inputs
Usage: ADC_Init(0b0011); // initialize AN1, AN0
*******************************************************************************/
void ADC_Init(unsigned int bitMask);


/*******************************************************************************
Function: ADC_GetConversion
Parameters:
	unsigned char analogInputIndex,   number of the analog pin from which
																		to get the result
Description: Returns the last updated value in the buffer for the given pin in 
  arbitrary units [au].  Note that this MAY NOT necessarily be the most current
	value
*******************************************************************************/
unsigned int ADC_GetConversion(unsigned char analogInputIndex);


/*******************************************************************************
Function: ADC_Deinit
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void ADC_Deinit(void);

#endif
