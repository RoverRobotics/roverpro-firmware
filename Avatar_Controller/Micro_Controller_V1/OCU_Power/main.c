#include <htc.h>

#define button GPIO5
#define mosfet_on GPIO1
#define power_relay GPIO0
//#define charging_voltage GPIO4

//12.2V: 12.2*100/400*255/5 = 155.55
#define MIN_VOLTAGE_OFF 172	//13.5V, actually 13.7V
#define MIN_VOLTAGE_ON 175
//8*100/400*255/5 = 102
#define NO_BATTERY 102

#define BATTERY_AD 8
#define THERM_AD 0x0C



#define WAIT_FOR_COOLDOWN 0
#define CHARGING_AFTER_COOLDOWN 1


__CONFIG(INTIO & WDTEN & BORDIS & PROTECT & PWRTEN);


unsigned char check_ad(unsigned char channel);
void toggle_power(void);
char wait_minutes(unsigned char target_minutes, unsigned char reset_counters);
void time_on_relay_offset(void);

//unsigned char counter = 0;
//unsigned char counter2 = 0;
unsigned char wait_for_cooldown = 0;
unsigned char minutes_counter = 0;
unsigned char relay_counter = 0;
unsigned char restart_charging = 0;
unsigned char last_button = 0;
unsigned char cooldown_start_counter = 0;
unsigned char waiting_flag = 0;
unsigned char battery_charging_state = CHARGING_AFTER_COOLDOWN;
unsigned char last_turned_off = 0;


unsigned char seconds = 0;
unsigned char minutes = 0;
unsigned char hours = 0;

unsigned char relay_off_counter = 0;
unsigned char relay_off_for_turnon = 0;
unsigned char relay_was_on = 0;

void handle_button(void)
{

	if(button)
	{
		if(last_button == 0)
		{
			if(power_relay == 1)
			{
				relay_was_on = 1;
				power_relay = 0;
				time_on_relay_offset();
			}
			else
			{
				relay_was_on = 0;
			}
		
			mosfet_on = 1;

			if(relay_was_on)
			{
				time_on_relay_offset();
				power_relay = 1;
			}
			last_turned_off = 0;

		}
	}		

	else
	{
		mosfet_on = 0;
		last_turned_off = 1;
		if(last_button)
 		{
 			
 			power_relay = 0;
 			restart_charging = 1;
 		}
	}
	
	last_button = button;
	
}


void time_on_relay_offset(void)
{
	unsigned char i = 0;
	unsigned char j = 0;
	
	for(i=0;i<0xff;i++)
	{
		for(j=0;j<20;j++)
		{
		}
	}
}

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
	if(counter1==0)
	{
		counter2++;		
		
	}
	
	
	//seconds don't really correspond to minutes at all
	if(return_difference(last_counter2,counter2)>=8)
	{
		last_counter2 = counter2;
		seconds++;
	}
	
	if( (counter2 >= 1) && (counter1 >= 219))
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
	}
	
	
	
	
	
	
}

void interrupt my_isr(void)
{
		
	static unsigned char wait_for_voltage_rise = 0;
	//unsigned char ad_value;
	unsigned char battery_voltage;
	unsigned char battery_temperature;
	static unsigned char overtemp_counter = 0;
	static unsigned char super_overtemp_counter = 0;
	static unsigned char cooldown_minutes_start = 0;
	static unsigned char charge_minutes_start = 0;
	static unsigned char cycle_hours_start = 0;
	static unsigned char cycles = 0;
	unsigned char cooldown_minutes;
	unsigned char charge_minutes;
	
	if((TMR1IE)&&(TMR1IF)){
		battery_voltage = check_ad(BATTERY_AD);
		battery_temperature = check_ad(THERM_AD);
		
		increment_counters();
		
	
		
		switch(battery_charging_state)
		{
		case WAIT_FOR_COOLDOWN:
			if(cycles < 5)
			{
				cooldown_minutes = 60;//60
			}
			else
			{
				cooldown_minutes = 180;//120
			}
			if(return_difference(cooldown_minutes_start,minutes) >= cooldown_minutes)
			{
				battery_charging_state = CHARGING_AFTER_COOLDOWN;
				power_relay = 1;
				charge_minutes_start = minutes;
				cycles++;
				
			}
			break;
			
		case CHARGING_AFTER_COOLDOWN:
			//after 90 minutes of charging, start monitoring temperature

			if(return_difference(cycle_hours_start,hours) > 10)
			{
				hours = 0;
				cycles = 0;
				cycle_hours_start = 0;
				
			}
			
			if(cycles < 5)
			{
				
				charge_minutes = 90;//90
			}
			else
			{
				charge_minutes = 180;//60
			}
			
			
			if(return_difference(charge_minutes_start,minutes) >= charge_minutes)
			{
				if(battery_temperature >= 110)
				{
					overtemp_counter++;
				}
				else
				{
					overtemp_counter = 0;
				}
			
			
				if(overtemp_counter >= 10)
				{
					battery_charging_state = WAIT_FOR_COOLDOWN;
					cycle_hours_start = hours;
					power_relay = 0;
					restart_charging = 0;
					wait_minutes(0,1);
					overtemp_counter = 0;  
					cooldown_minutes_start = minutes;

				}		
			}
			else
			{
				//Emergency charging override if the battery gets too hot
				if(battery_temperature >= 160)
				{
					super_overtemp_counter++;
				}
				else
				{
					super_overtemp_counter = 0;
				}
				if(super_overtemp_counter >= 10)
				{
					power_relay = 0;
					
				}
				else
				{
					if(restart_charging == 0)
						power_relay = 1;
					
				}
				
				
				
			}
			
	 		if(restart_charging)
	 		{
	 			relay_counter++;
	 			if(relay_counter >= 30)
	 			{
	 				relay_counter = 0;
	 				restart_charging = 0;
	 				power_relay = 1;
	 				
	 			}
	 			
	 			
	 			
	 		}	
			break;
			
		
		
		}
			
		
 		
 		
		
		if(battery_voltage <= MIN_VOLTAGE_OFF)
		{
			wait_for_voltage_rise = 1;
			mosfet_on = 0;
			
		}
		else if((battery_voltage > MIN_VOLTAGE_OFF) && (wait_for_voltage_rise == 0))
		{
			handle_button();
		
		}
		else if((battery_voltage >= MIN_VOLTAGE_ON) && (last_turned_off==1))
		{
			wait_for_voltage_rise = 0;
			handle_button();
			last_turned_off = 0;
			
			
		}
		else
		{
			mosfet_on = 0;

			//update last_turned off, so the previous case will occur.
			if(button)
				last_turned_off = 0;
			else
				last_turned_off = 1;
			
		}
	
		TMR1IF = 0;
	}

	
}


unsigned char check_ad(unsigned char channel)
{
	unsigned char ad_value;
	unsigned char i;
	ADCON0 = channel;
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

char wait_minutes(unsigned char target_minutes, unsigned char reset_counters)
{
	static unsigned char counter1 = 0;
	static unsigned char counter2 = 0;
	static unsigned char minutes = 0;
	static unsigned char last_triggered = 0;
	//counter2 is 10 @ 5 min, 40 s
	
	if(reset_counters)
	{
		counter1 = 0;
		counter2 = 0;
		minutes = 0;	
		last_triggered = 0;
		return 0;
	}
	
	//use this flag if we want to keep calling this function long after the time has expired, so
	//we won't have to worry about overflows of the counters returning false
	if(last_triggered)
		return 1;
	
	
	counter1++;
	if(counter1 == 0)
	{
		counter2++;
		if(counter2 == 0)
		{
			minutes++;
		}
	}
	if( (counter2 >= 1) && (counter1 >= 219))
	{
		minutes++;
		counter1 = 0;
		counter2 = 0;
	}
	if(minutes>=target_minutes)
	{

		last_triggered = 1;
		return 1;
	}
	return 0;
	
	
	
}

void
main(void)
{

	//ANSEL = 0x54;	//all pins digital
	PIE1 = 0b00000001;		//enable TMR1 overflow interrupt
	//OPTION = 0b11110111;	//prescaler is 1:256 on TMR0
	TRISIO = 0b00111100;	//0b00010001	GP2, GP4, GP5, MCLR inputgg
	CMCON = 7;
	INTCON = 0b01000000;	//GIE, PEIE
	
	T1CON = 0b00010001;		//1:8 prescale, turn timer on
	//T1CON = 0b00110001;
	//init a/d port GP2/AN2
	ANSEL = 0b01010100;	//AN2, AN3 are analog, Fosc/16 (01010100)
	ADCON0 = 8 | 0x01;	//select AN2, turn on module
	
	
	power_relay = 1;
	mosfet_on = 0;

	
	ei();
	while (1){
		CLRWDT();
	}
}
