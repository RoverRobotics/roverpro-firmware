#include <htc.h>

#define white_led GPIO5
#define ir_led GPIO4


//AD = V*255/5
//V = AD*5/255
#define TURN_IR_LED_OFF 102
#define TURN_IR_LED_ON 77



__CONFIG(INTIO & WDTDIS & BORDIS & UNPROTECT & PWRTEN);


unsigned char check_ad(void);


unsigned char ir_led_on = 0;
unsigned char ir_off_counter = 0;


//51 units per volt A/D
void interrupt my_isr(void)
{
	if((TMR1IE)&&(TMR1IF)){
	
		
		
		if(ir_led_on)
		{

			if(check_ad() >= TURN_IR_LED_OFF)
			{
				
				//use this counter to avoid feedback loop between voltage dips on turn on, and light sensor input
				ir_off_counter++;
				if(ir_off_counter > 10)
				{
					ir_led = 0;
					ir_led_on = 0;
				}
			}

			
			
			
			
		}
		else if(white_led == 0)
		{
			

			if(check_ad() <= TURN_IR_LED_ON )
			{
				
			
					ir_off_counter = 0;
					ir_led = 1;
					ir_led_on = 1;
			}
				
					
		
		}
		
		if(white_led)
		{
			ir_led = 0;
			ir_led_on = 0;
		}
		
		TMR1IF=0;
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
	counter1=0;
	PIE1 = 0b00000001;		//enable TMR1 overflow interrupt
	//OPTION = 0b11110111;	//prescaler is 1:256 on TMR0
	TRISIO = 0b00101100;	//GP3 (MCLR), GP5, GP2 input
	CMCON = 7;
	INTCON = 0b01000000;	//GIE, PEIE
	T1CON = 0b00010001;		//1:8 prescale, turn timer on
	//init a/d port GP2/AN2
	ANSEL = 0b01010100;	//AN2 is analog, Fosc/16 (01010100)
	ADCON0 = 8 | 0x01;	//select AN2, turn on module
	
	//white_led = 1;
	ir_led = 0;
	
	ei();
	
	while(1){

	}
	
}

