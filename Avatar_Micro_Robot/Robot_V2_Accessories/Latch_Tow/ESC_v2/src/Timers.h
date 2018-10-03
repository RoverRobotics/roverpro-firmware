/*=============================================================================
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
=============================================================================*/
#ifndef TIMERS_H
#define TIMERS_H

/*---------------------------Module Definitions------------------------------*/
#define MAX_NUM_TIMERS    8 // the maximum number of timer this module can 
                            // keep track of
      
/*---------------------------Public Functions--------------------------------*/
/****************************************************************************
Function: InitTimers()
Description: Initializes the hardware timer
Notes:
	1) this function must be called before using any of the library functions
*****************************************************************************/
void InitTimers(void);

/****************************************************************************
Function: StartTimer()
Paramters:
	unsigned char timer_number,	the timer number to start
	unsigned int new_time,			the time in units of milliseconds after which
 												  		this timer will expire 
Description: Starts the given timer number to expire at the given time
*****************************************************************************/
void StartTimer(unsigned char timer_number, unsigned int new_time);

/****************************************************************************
Function: InitBase()
Returns:
	unsigned char,	whether the timer has expired
Parameters:	
	unsigned char timer_number,	the index of the timer to check 
Description: Returns whether the given timer has expired
*****************************************************************************/
unsigned char IsTimerExpired(unsigned char timer_number);


/****************************************************************************
Function: GetTime()
Returns:
	unsigned int,
Description: 
*****************************************************************************/
unsigned int GetTime(void);

/****************************************************************************
Function: Pause()

Parameters:
	unsigned int milliseconds, the number of milliseconds for which to pause

Description: Pauses for the given number of milliseconds by iterating until 
	the approximate amount of time has passed.

Notes:
	- blocking code!
*****************************************************************************/
void Pause(unsigned int milliseconds);

#endif
