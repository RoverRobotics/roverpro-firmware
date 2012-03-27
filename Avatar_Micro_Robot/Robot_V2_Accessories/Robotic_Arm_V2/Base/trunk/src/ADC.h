/*=============================================================================
File: ADC.h 

Description: This module sets up the 8-bit hardware analog-to-digital 
  converter.  

Notes:
   1) The reference voltage is configured to V_ref = V_DD = GND
   2) 10-bit A/D conversion (0-to-1023 inclusive)
   5) The value returned reflects the last read value and may not 
      necessarily be the current value of interest, depending on speed 
      requirements of your application.

See Also: N/A
=============================================================================*/
#ifndef ADC_H
#define ADC_H

/*---------------------------Dependencies------------------------------------*/

/*---------------------------Macros and Definitions--------------------------*/

/*---------------------------Public Function Prototypes----------------------*/
/*
	Function: InitADC()
	Parameters: N/A
	TODO: make a function of what analog pins are desired (ie a bit mask)
*/
void InitADC(void);

/*
  Function: GetADC()
	Returns:
		unsigned int,	the last updated value to the buffer for this desired pin
	Parameters:	
		unsigned char analog_input_index, number of the analog pin from which
                                      to get the result
	Notes:
		1) returns the LAST UPDATED value
*/
unsigned int GetADC(unsigned char analog_input_index);

#endif
