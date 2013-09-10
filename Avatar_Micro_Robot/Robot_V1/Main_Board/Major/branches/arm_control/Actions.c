#include "system.h"

/*****************************************************************************
* Module Level Variables
******************************************************************************/
static unsigned char Flipper_Dir;
static unsigned char Flipper_On;

unsigned char robot_moving_flag = 0;

unsigned char OCU_battery_voltage = 0x00;

void reset_motor_controllers(void);

/*****************************************************************************
* Function: Update robot commands based on last good message
******************************************************************************/
void Update_Robot_Commands(unsigned char disable_driving)
{
	unsigned char L_LR,L_UD,R_LR,R_UD;
	unsigned char led_on, ocu_robot_talk, payload_button, flipper_up, flipper_down, left_trigger, right_trigger, select1, select0;
	static unsigned char payload_button_counter = 0;


	L_LR = Return_Last_Datalink_Message(6);
	L_UD = Return_Last_Datalink_Message(7);
	R_LR = Return_Last_Datalink_Message(8);
	R_UD = Return_Last_Datalink_Message(9);
	led_on = Return_Last_Datalink_Message(10) & 0b10000000;
	ocu_robot_talk = Return_Last_Datalink_Message(10) & 0b01000000;
	payload_button = Return_Last_Datalink_Message(10) & 0b00100000;
	flipper_up = Return_Last_Datalink_Message(10) & 0b00010000;
	flipper_down = Return_Last_Datalink_Message(10) & 0b00001000;
	left_trigger = Return_Last_Datalink_Message(10) & 0b00000100;
	right_trigger  = Return_Last_Datalink_Message(10) & 0b00000010;

	select1 = Return_Last_Datalink_Message(11) & 0b10000000;
	select0 = Return_Last_Datalink_Message(11) & 0b01000000;

	OCU_battery_voltage = Return_Last_Datalink_Message(11) & 0b00000111;
	//OCU_battery_voltage = 0x03;

	if(payload_button) {
		payload_button_counter++;
	}
	else
		payload_button_counter = 0;

	if((payload_button_counter > 60) && select0)
	{
		reset_payload();
		payload_button_counter = 0;
	}
	else if((payload_button_counter > 20) && (select0 == 0x00))
	{
		reset_motor_controllers();
		payload_button_counter = 0;
	}
	else
	{
		if(select0)
		{
			select_video_channel(1);
			handle_payload_message();
		}
		else
		{
			stop_payload_motion();
			select_video_channel(0);
	
		}
	}


	//turn off audio if the robot should be driving.
	//also implements some hysteresis in this behavior (to prevent clicking static as the joystick voltage varies)
	if(robot_moving_flag)
	{
		if( (L_LR < (127.5+AUDIO_DEADBAND_OFF)) && (L_LR > (127.5-AUDIO_DEADBAND_OFF)) )
			robot_moving_flag = 0;
		else if( (L_UD < (127.5-AUDIO_DEADBAND_OFF)) && (L_UD > (127.5+AUDIO_DEADBAND_OFF)) )
			robot_moving_flag = 0;


	}
	else
	{
		if( (L_LR > (127.5+AUDIO_DEADBAND_ON)) || (L_LR < (127.5 - AUDIO_DEADBAND_ON)) )
			robot_moving_flag = 1;
		else if( (L_UD > (127.5+AUDIO_DEADBAND_ON)) || (L_UD < (127.5-AUDIO_DEADBAND_ON)) )
			robot_moving_flag = 1;
		else if(flipper_up | flipper_down)
			robot_moving_flag = 1;


	}







	if(ocu_robot_talk)
	{
		Audio_pin = 1;		//RX
		Amp_pin = 1;		//Amp on
		Nop();
		Mic_pin = 0;

	}
	else if(robot_moving_flag)
	{
		Audio_pin = 1;
		Amp_pin = 0;
		Mic_pin = 0;

	}
	else{		//Robot is transmitting

		Audio_pin = 0;		//TX
		Amp_pin = 0;		//Amp off
		Mic_pin = 1;
	}




	if(led_on)
		Led_pin = ON;
	else
		Led_pin = OFF;

	if(flipper_up | flipper_down)
		Flipper_On = 1;
	else
		Flipper_On = 0;

	if (flipper_up)
	{
		Flipper_On = 1;
		Flipper_Dir = 0;

	}
	else if(flipper_down)
	{
		Flipper_On = 1;
		Flipper_Dir = 1;

	}
	else
		Flipper_On = 0;
	




if(select0 && (return_payload_type()==ARM))
{
		arm_in_use = 1;

}
else
	{
		arm_in_use = 0;

	}

		if(disable_driving)
			Set_Desired_Motor_Pulse_Quadrants(0x7F,0x7F,Flipper_On,Flipper_Dir);
		else
			Set_Desired_Motor_Pulse_Quadrants(L_LR,L_UD,Flipper_On,Flipper_Dir);

}

void Flipper_Control(void)
{
	Run_Flipper_Control(Flipper_Dir, Flipper_On);			//Flipper control loop
}


void reset_motor_controllers(void)
{
	Set_Desired_Motor_Pulse_Quadrants(0x7F,0x7F,0,0);
	Relay_Power_Control = OFF;
	OSD_display_string("Resetting Motors",2,1,16);

	kick_watchdogs();

	block_ms(2000);

	kick_watchdogs();
	Relay_Resistor_Control = ON;

	block_ms(500);
	Relay_Power_Control = ON;
	Relay_Resistor_Control = OFF;
	OSD_display_string("                ",2,1,16);
	block_ms(500);





}
