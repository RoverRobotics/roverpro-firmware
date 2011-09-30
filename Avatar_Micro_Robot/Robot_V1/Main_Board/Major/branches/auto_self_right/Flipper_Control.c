
#include "system.h"

/*****************************************************************************
 * Module Level Variables 
 *****************************************************************************/

static int Desired_Position;
static int Current_Position;
static float Integral;
static char last_flipper_direction = 0;
static char switch_last_hit = 0;


/*****************************************************************************
* Function: Flipper PI Control Loop
******************************************************************************/
void Flipper_PI(void)
{
	int Error;
	float Correction;

	Error = Desired_Position - Current_Position;

	// Calculations and scaling
	Error = Desired_Position - Current_Position;

	Integral = Error + Integral;
	if (Integral > (1000/Ki))
		Integral = 1000/Ki;
	if (Integral < (-1000/Ki))
		Integral = -1000/Ki;

	Correction = (Kp * Error) + (Ki * Integral);
	
	// Anti Wind Up
	if (Correction > 1000)
		Correction = 1000;
	// Anti Wind Down
	if (Correction < -1000)
		Correction = -1000;

	// Set PWM 1 to the desired duty cycle
	OC1R = Correction*SCALER + 37000;		//37000 is centered, prints to PWM1
}

/*****************************************************************************
* Function: Flipper Control Loop Handler
******************************************************************************/
void Run_Flipper_Control(unsigned char Dir, unsigned char Flip_On)
{
	Current_Position = Pull_Flip_Pos_AD();	//Pull current position
	
	if (Dir == 1)
		Desired_Position = Current_Position + 100;
	else 
		Desired_Position = Current_Position - 100;

	if (Flip_On == 0){
		Desired_Position = Current_Position;
		Integral = Integral/1.075;
	}
	
	Flipper_PI();							// Run PI calc + update motor

	//check if we overran
	if (Desired_Position > 1023)
		Desired_Position = Desired_Position - 1023;
	if (Desired_Position < 0)
		Desired_Position = Desired_Position + 1023;
}


unsigned char Is_Robot_Upside_Down(void)
{

	return (PORTDbits.RD2==0);

}

void Blocking_Self_Right(void)
{
	unsigned int i;
	int flipper_speed = -50;
	unsigned int last_change_counter = 0;
	unsigned int direction_changes = 0;
	Relay_Resistor_Control = ON;
	block_ms(200);
	Relay_Power_Control = ON;
	OSD_display_string("Self Righting...  ",2,1,18);
	//Set_Desired_Motor_Pulse_Quadrants(0x7F, 0x7F, 1, 0);
	
	for(i=0;i<800;i++)
	{
		set_motor_effort(flipper_speed,FLIPPER_MOTOR);
		kick_watchdogs();
		if(FLIPPER_SWITCH && last_change_counter > 30)
		{
			if((flipper_speed == -50) && (direction_changes >=3))
				return;
			flipper_speed*=-1;
			direction_changes++;
			last_change_counter = 0;
			block_ms(200);
		}
		last_change_counter++;

		if(Handle_Messages())
		{
//			if flipper toggle is being pressed and robot isn't necessarily upside down
//			return.
			if(((Return_Last_Datalink_Message(10) & 0b00010000)||(Return_Last_Datalink_Message(10) & 0b00001000)) && (Is_Robot_Upside_Down() == 0))
			{
				OSD_display_string("                  ",2,1,18);
				block_ms(10);
				return;
			}
		}
		block_ms(50);
	}

	


}


void handle_flipper(int PI_motor_effort)
{
	static unsigned int flipped_counter = 0;


	if(PORTFbits.RF3)
	{
		switch_last_hit = 1;
	}
	else
	{

		//counter is a sloppy way to make sure that flipper is far from the switch
		flipped_counter++;
		if(flipped_counter > 100)
		{
			switch_last_hit = 0;
			flipped_counter = 0;
		}
	}



	if(switch_last_hit)
	{
		if((PI_motor_effort > 0) && (last_flipper_direction == 1))
			PI_motor_effort = 0;
		else if((PI_motor_effort < 0) && (last_flipper_direction == -1))
			PI_motor_effort = 0;

	}
	else
	{
		if(PI_motor_effort > 0)
			last_flipper_direction = 1;
		else
			last_flipper_direction = -1;
	}

	set_motor_effort(PI_motor_effort,FLIPPER_MOTOR);


}
