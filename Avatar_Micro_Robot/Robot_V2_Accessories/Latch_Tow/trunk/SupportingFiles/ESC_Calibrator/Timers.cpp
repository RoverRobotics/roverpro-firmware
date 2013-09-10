/*==============================================================================
File: Timers.cpp
==============================================================================*/
//---------------------------Dependencies---------------------------------------
#include "./Timers.h"
#include <Arduino.h>

//---------------------------Definitions----------------------------------------
typedef struct {
  uint64_t duration;   			// the duration of the timer
  uint64_t start_time;      // time at which the timer was started
} timer_t;

//---------------------------Module Variables-----------------------------------
static timer_t timers[TMRS_MAX_N_TIMERS];

//---------------------------Public Funcion Definitions-------------------------
void TMRS_Init(void) {
  // initialize all timers so they will be sure to begin expired
  uint8_t i;
	for (i = 0; i < TMRS_MAX_N_TIMERS; i++) {
	  timers[i].duration = 0;
		timers[i].start_time = 0;
	}
}

void TMRS_StartTimer(uint8_t timer_number, uint64_t new_time) {
  timers[timer_number].duration = new_time;
  timers[timer_number].start_time = millis();
}

bool TMRS_IsTimerExpired(uint8_t timer_number) {
  return (timers[timer_number].duration <
         (millis() - timers[timer_number].start_time)); // delta time
}
