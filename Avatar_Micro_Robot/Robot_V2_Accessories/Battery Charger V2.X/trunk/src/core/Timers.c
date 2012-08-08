/*==============================================================================
File: Timers.c
==============================================================================*/
//#define TEST_TIMERS
/*---------------------------Dependencies-------------------------------------*/
#include "./Timers.h"
#include <p24FJ256GB106.h>

/*---------------------------Macros and Type Definitions----------------------*/
/*
Type: timer_t
Description: A timer is comprised of a duration and a start time.
*/
typedef struct {
	unsigned long int duration;        // the duration of the timer
	unsigned long int startTime;       // time at which the timer was started
} timer_t;

#define INTERRUPTS_PER_TICK 	8
#define TIMER_PERIOD 					250

/*---------------------------Helper Function Prototypes-----------------------*/
static void Timer_ISR(void);

/*---------------------------Module Variables---------------------------------*/
static timer_t timers[MAX_NUM_TIMERS];
static unsigned long int currentTime;

/*---------------------------Test Harness-------------------------------------*/
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
/*---------------------------End Test Harness---------------------------------*/
/*---------------------------Public Function Definitions----------------------*/
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {
  Timer_ISR();
}

/*******************************************************************************
Notes:
	- see p.143 of datasheet for initialization sequence
	- ms_per_tick = ((f_osc/2)/prescaler)^-1*timer_period*interrupts_per_tick
                = ((32MHz/2)/8)^-1*250*8
		            = 1ms
*******************************************************************************/
void InitTimers(void) {
  T1CONbits.TCKPS = 0b01;           // configure the timer prescaler to 
	                                  // divide-by-8 (see p.162 of datasheet)
  T1CONbits.TCS = 0;                // do NOT use the external clock
  if (T1CONbits.TCS == 0) T1CONbits.TGATE = 0;// DISABLE gated time accumulation
  if (T1CONbits.TCS == 1) T1CONbits.TSYNC = 1;// synchronize external clock input
  PR1 = TIMER_PERIOD;               // configure the timer period
	
	// configure the Timer1 interrupt
  _T1IP = 4;                        // configure interrupt priority
  _T1IF = 0;	                      // begin with the interrupt flag cleared
  _T1IE = 1;	                      // enable the interrupt
	
	T1CONbits.TON = 1;                // turn on timer1
}


void StartTimer(unsigned char timerNumber, unsigned int newTime) {
	// schedule when the timer expires
	timers[timerNumber].duration = newTime;
	timers[timerNumber].startTime = GetTime();
}


unsigned char IsTimerExpired(unsigned char timerNumber) {
  // BUG ALERT: stop interrupts!  This line takes several clock cylces to execute
  unsigned char result;
  //asm volatile ("disi #0x3FFF"); // disable interrupts
  result = (timers[timerNumber].duration < 
	         (GetTime() - timers[timerNumber].startTime));
  //asm volatile ("disi #0");      // enable interrupts
  
  return result;
}


unsigned long int GetTime(void) {
	return currentTime;
}


void DeinitTimers(void) {
	T1CONbits.TON = 0;  // turn off timer1
	_T1IE = 0;          // disable the interrupt
}


void Pause(unsigned int milliseconds) {
  unsigned int startTicks = GetTime();
  while ((GetTime() - startTicks) < milliseconds) {}
}

/*---------------------------Private Function Definitions---------------------*/
static void Timer_ISR(void) {
  static unsigned char numTimesCalled = 0;
  
  IFS0bits.T1IF = 0;  // clear the source of the interrupt
  
  if (INTERRUPTS_PER_TICK <= ++numTimesCalled) {
    numTimesCalled = 0;
	  currentTime++;
	}
}
