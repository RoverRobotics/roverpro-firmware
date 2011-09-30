#include <htc.h>



#define WDT_IN GPIO2
#define EN_3V3 GPIO4



__CONFIG(INTIO & WDTEN & BOREN & PROTECT & PWRTEN);


unsigned char check_ad(void);


unsigned char seconds = 0;
unsigned char eighth_seconds = 0;


//accounts for overflows, but only works properly if difference is less than 255
unsigned char return_difference(unsigned char first_byte, unsigned char second_byte)
{
	
	if(first_byte > second_byte)
	{
		return (0xff-first_byte)+second_byte;
	
	}
	else
	{
		
		return second_byte - first_byte;
		
	}	
	
}


void increment_counters(void)
{
	static unsigned char counter0 = 0;
	static unsigned char counter1 = 0;
	static unsigned char counter2 = 0;
	//static unsigned char minutes = 0;
	static unsigned char last_counter2 = 0;

	
	counter1++;
	eighth_seconds++;

	if(eighth_seconds > 250)
		eighth_seconds = 0;

	/*if(counter1 >= 8)
	{
		counter1 = 0;
		seconds++;
	}*/

/*	if(counter1==0)
	{
		EN_3V3 = 1;
		counter2++;		
		
	}
	
	
	//seconds don't really correspond to minutes at all
	if(return_difference(last_counter2,counter2)>=8)
	{
		
		last_counter2 = counter2;
		seconds++;
	}*/
	
	/*if( (counter2 >= 1) && (counter1 >= 219))
	{
				
		minutes++;
		counter1 = 0;
		counter2 = 0;
	}
	
	if(minutes == 60)
	{
		minutes = 0;
		
		//don't allow hours to overflow.  10 days should be enough time
		if(hours < 255)
			hours++;		
	}*/
	
	
	
	
	
	
}

//51 units per volt A/D
void interrupt my_isr(void)
{		


	if((TMR1IE)&&(TMR1IF)){
		
		increment_counters();
		TMR1IF = 0;
	}
}


unsigned char check_ad(void)
{
	unsigned char ad_value;
	unsigned char i;
	ADON = 1;
	for(i=0;i<255;i++);
	
	GODONE = 1;
	while(GODONE);
	//while(!ADIF);
	ADIF = 0;
	ad_value = ADRESH;
	
	ADON = 0;

	return ad_value;
	
	
	
	
}



void
main(void)
{
	unsigned char counter1, counter2, counter3,i,last_released;
	unsigned char new_wdt_in = 0;
	unsigned char last_wdt_in = 0;


	counter1=0;
	OPTION = 0xff;
	PIE1 = 0b00000001;		//enable TMR1 overflow interrupt
	TRISIO = 0b00001100;	//GP2, GP3(MCLR) inputs
	CMCON = 7;
	INTCON = 0b01000000;	//GIE, PEIE
	T1CON = 0b00010001;		//1:8 prescale, turn timer on
	//init a/d port GP2/AN2
	//ANSEL = 0b01010000;	//AN2 is analog, Fosc/16 (01010100)
	ANSEL = 0x00;	//all pins digital
	//ADCON0 = 8 | 0x01;	//select AN2, turn on module
	

	
	ei();
	EN_3V3 = 0;


	while(1){
		CLRWDT();	
		EN_3V3 = 0;
		new_wdt_in = WDT_IN;

		if(new_wdt_in && (last_wdt_in==0))
		{
			eighth_seconds = 0;
		}
		else if((new_wdt_in==0) && last_wdt_in)
		{
			eighth_seconds = 0;
		}
		last_wdt_in = new_wdt_in;

		
//		If the watchdog hasn't been kicked yet, and there have been two resets already,
//		Increase wait time, to allow for programming



		if(eighth_seconds >= 40)
		{
			EN_3V3 = 1;
			eighth_seconds = 0;
			while(eighth_seconds < 8)
			{
				CLRWDT();
			}
			//EN_3V3 = 0;

		}

	}
	
}

