/*==============================================================================
File: StandardHeader.c

Description: This file organizes common dependencies, macros, type definitions, 
  general utility functions and other generally useful facilities 
  across an application.  As this file will be included in nearly every file,
  it should be kept concise.

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef STANDARD_HEADER_H
#define STANDARD_HEADER_H
/*---------------------------Dependencies-------------------------------------*/
#include <p24FJ256GB106.h>

/*---------------------------Macros-------------------------------------------*/
#define OFF                   0
#define ON                    1

#define OUTPUT                0
#define INPUT                 1

#define CW                    0
#define CCW                   1
#define NO_DIRECTION					2

#define FALSE     						0
#define TRUE      						(!FALSE)

#define NO                    0
#define YES                   (!NO)

/*---------------------------Constants----------------------------------------*/

/*---------------------------Public Function Prototypes-----------------------*/
/*
Function: Map
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
                 int to_low, int to_high);


/*
Function: Delay
Description: Performs a BLOCKING pause to aid in debugging.  You weren't 
  thinking about actually leaving this in your code were you?
Parameters:
  unsigned int milliseconds, the approximate number of milliseconds for
                             which to pause.  The influence of the overhead 
                             of entering and exiting the loop reduces the 
                             longer you pause for.
Notes:
  - assumes a 20MHz external oscillator
  - entirely blocking code
  - note the limit of the data type capping the maximum number of millieconds
    you can pause for (~65535ms)
*/
void Delay(unsigned int milliseconds);

#endif
