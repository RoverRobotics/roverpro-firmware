
#include "system.h"

unsigned int timer_ms = 0;
unsigned int timer_s = 0;
unsigned int timer_m = 0;
unsigned char increment_minute_flag = 0;
unsigned int minutes_since_last_update = 0;

/*********************************************************************
 * Function:        Timer 1 Initialization
 ********************************************************************/
void Timer1Init(void)
{	
	PR1 = 40000;					//Period, will interrupt at 100Hz
	
	IPC0bits.T1IP = 5;				//Interrupt priority 5
	T1CON = 0b1000000000010000;		//Timer1 on, cont in idle, no gate, 1:8 prescale, no external sync, int clock (Fosc/2)
	IFS0bits.T1IF = 0;				//Clear timer interrupt flag
}

/*********************************************************************
 * Function:        TimerIsOverflowEvent
 ********************************************************************/
unsigned char Timer1IsOverflowEvent(void)
{
	if (IFS0bits.T1IF)				//If timer has overflowed
	{		
		IFS0bits.T1IF = 0;			//Clear interrupt flag
		return(1);					//Return 1
	}
	return(0);						//Else the timer is still running, return 0
}


/*********************************************************************
 * Function:        block_ms
 * Description:		Blocking wait function, to be used for testing
 ********************************************************************/
void block_ms(unsigned int ms)
{
	unsigned int i;
	unsigned int j;

	for(i=0;i<3200;i++)

	{
		for(j=0;j<ms;j++);
	}
}


void init_timer2(void)
{
	//set prescale to 1:8
	T2CONbits.TCKPS = 0b01;

	//Get timer 2 clock from Fosc/2
	T2CONbits.TCS = 0;


	IFS0bits.T2IF = 0;	//clear interrupt flag
	IEC0bits.T2IE = 1;	//enable timer 2 interrupt
	//.002*(SYSCLK/2)/T3_PRESCALE - 1;
	PR2 = 12500;	//interrupt every 10 ms

	T2CONbits.TON = 1;


}

void  __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
		IFS0bits.T2IF = 0;	//clear interrupt flag
		timer_ms+=5;

		//This correction counts seconds correctly.
		//I'm not sure what's wrong with the PR2 I picked
		if(timer_ms >= 800)
		{
			timer_s++;
			timer_ms = 0;



		}
		if(timer_s >= 60)
		{
			timer_m++;
			timer_s = 0;
			increment_minute_flag = 1;

			
		}




}
