/*=============================================================================
File: ADC.h 

Description: This module sets up one of several 10-bit hardware 
	analog-to-digital converter.
Notes:
	1) Warning: does no error checking on initialization bit mask
=============================================================================*/
#ifndef ADC_H
#define ADC_H
/*---------------------------Macros and Definitions--------------------------*/

/*---------------------------Public Function Prototypes----------------------*/
/*
Function: InitADC()
Parameters:
	unsigned int bit_mask, a number interpreted as binary that describes which
  											 of the 0-though-15 analog pins are desired to be
                         analog inputs  
Description: Initializes the specified analog-to-digital-conversion (ADC)
	hardware module.
Usage: InitADC(0b0011); // initialize AN1, AN0
*/
void InitADC(unsigned int bit_mask);

/*
  Function: GetADC()
	Returns:
		unsigned int,	the last updated value in the buffer for this desired 
                  pin in arbitrary units [au]
	Parameters:	
		unsigned char analog_input_index, number of the analog pin from which
                                      to get the result
	Notes:
		1) returns the LAST UPDATED value, which may not necessarily be the
       most current value
*/
unsigned int GetADC(unsigned char analog_input_index);

#endif
