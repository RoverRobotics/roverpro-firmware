/*==============================================================================
File: Timers.c
==============================================================================*/
//#define TEST_TIMERS
/*---------------------------Dependencies-------------------------------------*/
#include "./Timers.h"

/*---------------------------Macros and Type Definitions----------------------*/
/*
Type: timer_t
Description: A timer is comprised of a duration and a start time.
*/
typedef struct {
	uint32_t duration;        // the duration of the timer
	uint32_t startTime;       // time at which the timer was started
} timer_t;

#define INTERRUPTS_PER_TICK 	8
#define TIMER_PERIOD 					250

/*---------------------------Helper Function Prototypes-----------------------*/
static void TMRS_ISR(void);

/*---------------------------Module Variables---------------------------------*/
static timer_t timers[MAX_NUM_TIMERS];
static uint32_t currentTime;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_TIMERS
#include "./ConfigurationBits.h"

//---Times
#define _500ms            500  // ticks ~= 500ms/(1.000ms/timer_tick)
//---Timers
#define HEARTBEAT_TIMER   1
#define HEARTBEAT_TIME    _500ms

int main(void) {
	TMRS_Init();
	TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME); // prime any timers that require it
	
	_TRISE5 = 0; _RE5 = 0;
  
	while (1) {
  	// toggle a pin on timer expirations
		if (TMRS_IsTimerExpired(HEARTBEAT_TIMER)) {
			TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
			_RE5 ^= 1;
		}
	}
	
	return 0;
}
#endif
/*---------------------------End Test Harness---------------------------------*/
/*---------------------------Public Function Definitions----------------------*/
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {
  TMRS_ISR();
}

/*******************************************************************************
Notes:
	- see p.143 of datasheet for initialization sequence
	- ms_per_tick = ((f_osc/2)/prescaler)^-1*timer_period*interrupts_per_tick
                = ((32MHz/2)/8)^-1*250*8
		            = 1ms
*******************************************************************************/
void TMRS_Init(void) {
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


void TMRS_StartTimer(uint8_t timerNumber, uint32_t newTime) {
	// schedule when the timer expires
	timers[timerNumber].duration = newTime;
	timers[timerNumber].startTime = TMRS_GetTime();
}


bool TMRS_IsTimerExpired(uint8_t timerNumber) {
  // BUG ALERT: stop interrupts!  This line takes several clock cylces to execute
  bool result;
  //asm volatile ("disi #0x3FFF"); // disable interrupts
  result = (timers[timerNumber].duration < 
	         (TMRS_GetTime() - timers[timerNumber].startTime));
  //asm volatile ("disi #0");      // enable interrupts
  
  return result;
}


uint32_t TMRS_GetTime(void) {
	return currentTime;
}


void TMRS_Deinit(void) {
	T1CONbits.TON = 0;  // turn off timer1
	_T1IE = 0;          // disable the interrupt
}


void TMRS_Pause(unsigned long int milliseconds) {
  uint32_t startTicks = TMRS_GetTime();
  while ((TMRS_GetTime() - startTicks) < milliseconds) {}
}

/*---------------------------Private Function Definitions---------------------*/
static void TMRS_ISR(void) {
  static uint8_t numTimesCalled = 0;
  
  IFS0bits.T1IF = 0;  // clear the source of the interrupt
  
  if (INTERRUPTS_PER_TICK <= ++numTimesCalled) {
    numTimesCalled = 0;
	  currentTime++;
	}
}
