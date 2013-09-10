/*==============================================================================
 
File: Timers.h
 
Description: This module encapsulates a 16-bit software timer based 
    on Arduino's millis() function.  

Notes: 
  - assumes exclusive use of Timer0 through the millis() function

Author: Stellios Leventis (stellios.leventis@stanford.edu)
==============================================================================*/
#pragma once
//---------------------------Dependencies---------------------------------------
#include <stdint.h>

//---------------------------Macros---------------------------------------------
#define TMRS_MAX_N_TIMERS		16 	// the maximum number of timer this module can 
																// keep track of
                
//---------------------------Public Function Prototypes-------------------------
// Function: TMRS_Init
// Description: Initializes the timers module, ensuring all timers begin
//   expired.
void TMRS_Init(void);

// Function: TMRS_StartTimers
// Parameters:
//   uint8_t timer_number, the index of the timer to start 
//                         (0-to-TMRS_MAX_N_TIMERS inclusive)  
//   uint64_t ms,          the number of milliseconds for which to set 
//                         the timer
void TMRS_StartTimer(uint8_t timer_number, uint64_t ms);

// Function: TMRS_IsTimerExpired
// Description: Returns whether the given timer has expired
bool TMRS_IsTimerExpired(uint8_t timer_number);
