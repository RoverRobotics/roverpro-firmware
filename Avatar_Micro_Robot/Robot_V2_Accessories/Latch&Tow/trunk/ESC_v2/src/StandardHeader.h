/*=============================================================================
File: StandardHeader.c

Description: This file provides organizes common dependencies, macros,
  type definitions, general functions and other generally useful facilities 
  across an application.  As this file will be included in nearly every file,
  it should be kept concise.  To emphasize the value placed on brevity, all
  functions are additionally implemented in this file.

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
=============================================================================*/
#ifndef STANDARD_HEADER_H
#define STANDARD_HEADER_H
/*---------------------------Dependencies------------------------------------*/
#include <p24FJ256GB106.h>

/*---------------------------Macros------------------------------------------*/
#define ON        1
#define OFF       0

#define YES       1
#define NO        0

#define INPUT     1
#define OUTPUT    0

#define CCW       1
#define CW        0

#define FALSE     0
#define TRUE      (!FALSE)

/*---------------------------Constants---------------------------------------*/

/*---------------------------Public Function Prototypes----------------------*/
/*
Function: Map()

Returns:
  unsigned int, the given value mapped to the new range

Parameters:
  int value,      the number to map
  int from_low,   the lower bound of the value's current range
  int from_high,  the upper bound of the value's current range
  int to_low,     the lower bound of the value's target range
  int to_high,    the upper bound of the value's target range

Description: Re-maps a number from one range to another.

Notes:
  - constrains values to within the range
  - be wary of overflow errors producing unexpected results
  - promotes data types and forces division before multiplication 
    to help mitigate unexpected overflow issues
TODO: put this function in StandardHeader.h/.c
*/
unsigned int Map(int value, int from_low, int from_high, 
                 int to_low, int to_high) {  
  // compute the linear interpolation
  unsigned int result = ((double)(value - from_low) / (double)(from_high - from_low))
                        * (double)(to_high - to_low) + to_low;
  
  // constrain the result to within a valid output range
  if (to_high < result) result = to_high;
  else if (result < to_low) result = to_low;
  
  return result;
}


/*
Description: Performs a BLOCKING pause to aid in debugging.  You weren't 
  thinking about actually leaving this in your code were you?

Parameters:
  unsigned int milliseconds, the approximate number of milliseconds for
                             which to pause.  The influence of the overhead 
                             of entering and exiting the loop reduces the 
                             longer you pause for.
Notes:
  - assumes f_osc = 32MHz
  - entirely blocking code
  - note the limit of the data type capping the maximum number of millieconds
    you can pause for (~65535ms)
*/
#define OPS_PER_MS            3200  // operations per millisecond, for current 
                                    // oscillator choice and 
                                    // configuration-bit settings
void Delay(unsigned int milliseconds) {
  unsigned int i,j;
	for (i = 0; i < OPS_PER_MS; i++) {
    for (j = 0; j < milliseconds; j++);
  }
}

#endif
