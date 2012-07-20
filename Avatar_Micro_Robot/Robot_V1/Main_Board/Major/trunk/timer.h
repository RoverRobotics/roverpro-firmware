

/*********************************************************************
 * Function:        TimerInit
 *
 * PreCondition:    None.
 *
 * Input:       	None.
 *                  
 * Output:      	None.
 *
 * Overview:        Initializes Timer0 for use.
 *
 ********************************************************************/
extern void Timer1Init(void);

/*********************************************************************
 * Function:        TimerIsOverflowEvent
 *
 * PreCondition:    None.
 *
 * Input:       	None.	
 *                  
 * Output:      	Status.
 *
 * Overview:        Checks for an overflow event, returns TRUE if 
 *					an overflow occured.
 *
 * Note:            This function should be checked at least twice
 *					per overflow period.
 ********************************************************************/
extern unsigned char Timer1IsOverflowEvent(void);

/*********************************************************************
 * Function:        block_ms
 *
 * PreCondition:    None.
 *
 * Input:       	Number of milliseconds to wait (unsigned int)	
 *                  
 * Output:      	Status.
 *
 * Overview:        Returns once the requisite number of milliseconds has elapsed
 *
 * Note:            
 ********************************************************************/
extern void block_ms(unsigned int ms);
void init_timer2(void);

/*********************************************************************
 * EOF
 ********************************************************************/

extern unsigned int timer_ms;
extern unsigned int timer_s;
extern unsigned int timer_m;
extern unsigned char increment_minute_flag;
extern unsigned int minutes_since_last_update;

