/*=============================================================================
File: Timers.c
=============================================================================*/
//#define TEST_TIMERS

/*---------------------------Dependencies------------------------------------*/
#include "./Timers.h"
#include <p24FJ256GB106.h>

/*---------------------------Macros and Definitions--------------------------*/
/*
Type: timer_t
Description: A timer is comprised of a duration and a start time.
*/
typedef struct {
	unsigned int duration;        // the duration of the timer
	unsigned int start_time;      // time at which the timer was started
} timer_t;

#define INTERRUPTS_PER_TICK 	7 // BUG ALERT: 0-based indexing
#define TIMER_PERIOD 					250

/*---------------------------Helper Function Prototypes----------------------*/
static void Timer_ISR(void);

/*---------------------------Module Variables--------------------------------*/
static timer_t timers[MAX_NUM_TIMERS];
static unsigned int current_time;

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_TIMERS
#include "./ConfigurationBits.h"

//---Times
#define _500ms            500  // ticks ~= 500ms/(1.000ms/timer_tick)
//---Timers
#define HEARTBEAT_TIMER   1
#define HEARTBEAT_TIME    _500ms

int main(void) {
	InitTimers();
	StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME); // prime any timers that require it
	
	_TRISE5 = 0; _RE5 = 0;
  
	while (1) {
  	// toggle a pin on timer expirations
		if (IsTimerExpired(HEARTBEAT_TIMER)) {
			StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
			_RE5 ^= 1;
		}
	}
	
	return 0;
}
#endif
/*---------------------------End Test Harness--------------------------------*/

/*---------------------------Public Function Definitions---------------------*/
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {
  Timer_ISR();
}

/****************************************************************************
Notes:
	1) see p.143 of datasheet for initialization sequence
	2) ms_per_tick = ((f_osc/2)/prescaler)^-1*timer_period*interrupts_per_tick
                 = ((32MHz/2)/8)^-1*250*7
		             = 1ms?????TODO: why doesn't math work out?
*****************************************************************************/
void InitTimers(void) {
  T1CONbits.TCKPS1 = 0; T1CONbits.TCKPS0 = 1; // configure the timer prescaler ratio to 1/8 (see p.152 of datasheet)
  T1CONbits.TCS = 0;                // do NOT use the external clock
  if (T1CONbits.TCS == 0) T1CONbits.TGATE = 0;// DISABLE gated time accumulation
  if (T1CONbits.TCS == 1) T1CONbits.TSYNC = 1;// synchronize external clock input
  PR1 = TIMER_PERIOD;               // configure the timer period
	
	// configure the Timer1 interrupt
  _T1IP = 4;                        // configure interrupt priority
  IFS0bits.T1IF = 0;	              // clear the Timer1 Interrupt Flag
  IEC0bits.T1IE = 1;	              // enable the interrupt for Timer1 (Timer1 Interrupt Enable bit)
	
	T1CONbits.TON = 1;                // turn on timer1
}


void StartTimer(unsigned char timer_number, unsigned int new_time) {
	// schedule when the timer expires
	timers[timer_number].duration = new_time;
	timers[timer_number].start_time = GetTime();
}

unsigned char IsTimerExpired(unsigned char timer_number) {
  unsigned char result;
  // BUG ALERT: stop interrupts!  This line takes several clock cylces to execute
  //asm volatile ("disi #0x3FFF"); // disable interrupts
  result = (timers[timer_number].duration < (GetTime() - timers[timer_number].start_time));
  //asm volatile ("disi #0");      // enable interrupts
  
  return result;
}

unsigned int GetTime(void) {
	return current_time;
}

void Pause(unsigned int milliseconds) {
  unsigned int start_ticks = GetTime();
  while ((GetTime() - start_ticks) < milliseconds) {}
}

/*---------------------------Private Function Definitions--------------------*/
static void Timer_ISR(void) {
  static unsigned char dummy = 0;
  
  // only increment every fourth time
  if (INTERRUPTS_PER_TICK <= dummy++) {
    dummy = 0;
	  current_time++;   // increment the GetTime() tick counter
	}
	
	IFS0bits.T1IF = 0;  // clear the source of the interrupt
}

/*---------------------------End of File-------------------------------------*/
