/*==============================================================================
File: Timers.h
 
Description: This module encapsulates a 16-bit software timer based 
	on Timer1 overflows.  See example usage in the test harness of the
  corresponding c-source file.

Notes: 
  - assumes global interrupts enabled externally
  - assumes exclusive use of Timer1
  - assumes f_osc = 32MHz
  - interrupt priority set to 1 (the lowest)
  - can count up to about 3 days (longest timer if started at startup) after
    initialization of this module

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef TIMERS_H
#define TIMERS_H

#include "./StandardHeader.h"

/*---------------------------Macros-------------------------------------------*/
#define MAX_NUM_TIMERS    8 // the maximum number of timer this module can 
                            // keep track of
      
/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: TMRS_Init
Description: Initializes the hardware timer
Notes:
	- this function must be called before using any of the library functions
*******************************************************************************/
void TMRS_Init(void);


/*******************************************************************************
Function: TMRS_StartTimer
Paramters:
	uint8_t timerIndex,	    the index of the timer
	uint32_t milliseconds,  the duration in units of milliseconds after
 												  which this timer will expire
*******************************************************************************/
void TMRS_StartTimer(uint8_t timerIndex, uint32_t milliseconds);


/*******************************************************************************
Function: TMRS_IsTimerExpired
Parameters:
	bool timerIndex,	  the index of the timer 
Description: Returns whether the given timer has expired
*******************************************************************************/
bool TMRS_IsTimerExpired(uint8_t timerIndex);


/*******************************************************************************
Function: TMRS_GetTime
Returns:
	uint32_t, the number of milliseconds passed since initialization
*******************************************************************************/
uint32_t TMRS_GetTime(void);


/*******************************************************************************
Function: TMRS_Deinit
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void TMRS_Deinit(void);


/*******************************************************************************
Function: TMRS_Pause
Parameters:
	uint32_t milliseconds, the number of milliseconds for which to pause
Description: Pauses for the given number of milliseconds by iterating until 
	the approximate amount of time has passed.
Notes:
	- blocking code!
*******************************************************************************/
void TMRS_Pause(uint32_t milliseconds);

#endif
