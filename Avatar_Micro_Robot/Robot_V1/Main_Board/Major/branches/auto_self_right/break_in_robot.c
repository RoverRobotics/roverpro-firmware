#include "system.h"

#define BACKWARD 0
#define FORWARD 1
#define BACKWARD_STOP 2
#define FORWARD_STOP 3

#define MAX_BREAKIN_SPEED 20

void linear_speed_ramp(unsigned char direction);
void display_break_in_hours(void);

void break_in_robot(void)
{
	unsigned int operation_minutes = 0;
	unsigned char operation_FSM = FORWARD;

	block_ms(100);

	OSD_display_string("Break In",1,11,8);

	retrieve_last_good_break_in_counter();
	display_break_in_hours();
	//reset_hour_meter(0);
	block_ms(2000);
	

	kick_watchdogs();
	//ClrWdt();
	Relay_Resistor_Control = ON;
	block_ms(500);
	Relay_Power_Control = ON;
	Relay_Resistor_Control = OFF;


	
	//wait some time for controllers to initialize
	block_ms(1500);

	linear_speed_ramp(FORWARD);


	while(1)
	{


	kick_watchdogs();
	//ClrWdt();

		if(increment_minute_flag)
		{

	
			increment_minute_flag = 0;
			
			if((operation_FSM == FORWARD) || (operation_FSM == BACKWARD))
				increment_break_in_minute_counter(1);
			block_ms(50);
			display_break_in_hours();
			operation_minutes++;
			//block_ms(10);
			//display_voltage();
			//block_ms(10);
			

		}

		if(operation_minutes >= 10)
		{
			switch(operation_FSM)
			{
				case FORWARD:
					operation_FSM = FORWARD_STOP;
					linear_speed_ramp(FORWARD_STOP);
				break;
				case FORWARD_STOP:
					operation_FSM = BACKWARD;
					linear_speed_ramp(BACKWARD);
				break;
				case BACKWARD:
					operation_FSM = BACKWARD_STOP;
					linear_speed_ramp(BACKWARD_STOP);
				break;
				case BACKWARD_STOP:
					operation_FSM = FORWARD;
					linear_speed_ramp(FORWARD);
				break;

			}
			operation_minutes = 0;
		}




	}





}




void linear_speed_ramp(unsigned char direction)
{
	unsigned int i;
	if(direction == FORWARD)
	{
		for(i=0;i<MAX_BREAKIN_SPEED;i++)
		{
			OC2R = 37000+i*10;
			OC3R = 37000-i*10;
			block_ms(100);

		}

	}
	else if(direction == BACKWARD)
	{
		for(i=0;i<MAX_BREAKIN_SPEED;i++)
		{
			OC2R = 37000-i*10;
			OC3R = 37000+i*10;
			block_ms(100);
		}


	}

	else if(direction == FORWARD_STOP)
	{
		
		for(i=0;i<MAX_BREAKIN_SPEED;i++)
		{
			OC2R = 37000+MAX_BREAKIN_SPEED*10-i*10;
			OC3R = 37000-MAX_BREAKIN_SPEED*10+i*10;
			block_ms(100);
		}

		OC2R = 37000;
		OC3R = 37000;

		//cycle power on motor controlllers, in case there is an error state
		Relay_Power_Control = OFF;
		block_ms(1000);

		Relay_Resistor_Control = ON;
		block_ms(500);
		Relay_Power_Control = ON;
		Relay_Resistor_Control = OFF;





	}

	else if(direction == BACKWARD_STOP)
	{
		
		for(i=0;i<MAX_BREAKIN_SPEED;i++)
		{
			OC2R = 37000-MAX_BREAKIN_SPEED*10+i*10;
			OC3R = 37000+MAX_BREAKIN_SPEED*10-i*10;
			block_ms(100);
		}

		OC2R = 37000;
		OC3R = 37000;


		//cycle power on motor controlllers, in case there is an error state
		Relay_Power_Control = OFF;
		block_ms(1000);

		Relay_Resistor_Control = ON;
		block_ms(500);
		Relay_Power_Control = ON;
		Relay_Resistor_Control = OFF;
	}



}

