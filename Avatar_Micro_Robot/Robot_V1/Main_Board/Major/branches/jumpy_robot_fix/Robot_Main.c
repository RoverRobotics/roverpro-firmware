

/** INCLUDES *******************************************************/
#include "system.h"



  _CONFIG1(JTAGEN_OFF & GCP_ON & GWRP_OFF & BKBUG_OFF & COE_OFF & FWDTEN_ON & ICS_PGx2 & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS4096) 
//_CONFIG1(JTAGEN_OFF & GCP_ON & GWRP_OFF & BKBUG_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2 & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS4096) 
//_CONFIG2( 0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV5 & IOL1WAY_ON)
//_CONFIG2( IESO_OFF & PLLDIV_DIV5 & PLL96DIS_ON & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF & IOL1WAY_ON & DISUVREG_OFF & POSCMOD_HS)
//  _CONFIG2( IESO_ON & PLLDIV_DIV5 & PLL96DIS_ON & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF & IOL1WAY_ON & DISUVREG_OFF & POSCMOD_HS)
  _CONFIG2( IESO_ON & PLLDIV_DIV5 & 0xF7FF & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF & IOL1WAY_ON & DISUVREG_OFF & POSCMOD_HS)


void test_battery_life(void);
void check_clock_failure(void);

int main(void)
{   
static unsigned int Timeout_Counter;
unsigned int Robot_Upside_Down_Counter = 0;
unsigned char motors_off_flag = 1;
float motor_current_avg;
static char turn_speed_down_flag = 0;
static char voltage_display_counter = 0;
unsigned char first_datalink_message_received = 0;
unsigned char disable_driving = 0;
unsigned char voltage_not_updated_counter = 0;
unsigned char trigger_reset = 0;
unsigned char audio_pulse_state = 0;

	//block_ms(250);

	//test_battery_life();

	kick_watchdogs();
	
	//ClrWdt();
	Map_Pins();		//Set Tris registers and initial states
	Timer1Init();	//Setup and start Timer 1 system
	PWMInit();		//Setup and start PWM system

	Audio_pin = ON;
	Nop();

	Datalink_Init();



	// Test Code, Initializations





	

	Amp_pin = OFF;

	init_uart2();
	init_uart3();

	init_current_limit_atod();
	init_timer3();
	init_timer2();
	SPI_Init();



	check_clock_failure();

	block_ms(100);
	OSD_clear_screen();
	block_ms(100);

//	purge first payload message
	message_from_payload_ready = 0;

	OSD_display_string("Welcome",2,1,7);
	block_ms(1000);

	kick_watchdogs();
	//ClrWdt();

	//while(1);
	init_hour_meter_spi();
	//test_eeprom();
	//while(1);

	
	block_ms(500);

	kick_watchdogs();
	//ClrWdt();


	OSD_clear_screen();
	block_ms(5);

	display_hours();
	block_ms(10);
	display_voltage();
	block_ms(10);

	TRISEbits.TRISE5 = OUTPUT;
	LATEbits.LATE5 = OFF; 

/*	while(1)
	{
		for(i=0;i<4;i++)
		{

			video_switcher_channel_chars[0] = i+1;
			OSD_display_chars(video_switcher_channel_chars,4,4,2);
			select_video_channel(i);
			block_ms(3000);
	
		}
	}*/

	set_payload_type(PTZ);
	//set_payload_type(ARM);

	if(message_from_payload_ready)
	{

		message_from_payload_ready = 0;
		if(Is_payload_CRC_valid(message_from_payload,MESSAGE_FROM_PAYLOAD_LENGTH))
		{


			switch(message_from_payload[2])
			{
				case 0x04:
					switch(message_from_payload[3])
					{	
						case 0x00:
							break_in_robot();
						break;
						case 0x01:
							payload_robot_control();
						break;
						case 0x02:
							return_version_info();
						break;
						case 0x03:
							direct_motor_control();
						break;
						case 0x05:
							set_payload_type(ARM);
						break;
						case 0x06:
							unbind_robot();
						break;

					}
				break;

			}

		}



	}

	if(message_from_second_payload_ready)
	{

		message_from_second_payload_ready = 0;
		if(Is_payload_CRC_valid(message_from_second_payload,MESSAGE_FROM_PAYLOAD_LENGTH))
		{


			switch(message_from_second_payload[2])
			{
				case 0x04:
					switch(message_from_second_payload[3])
					{	
						case 0x08:
							set_second_payload_type(HITCH);
						break;
			
					}
				break;

			}

		}



	}

	kick_watchdogs();
	//ClrWdt();
	//break_in_robot();
	//send_reset_registers();


	//void send_reset_registers(void);
	set_stored_address();




	Audio_pin = 0;
	Amp_pin = 0;
	Mic_pin = 0;


    while(1)
    {
		//if(trigger_reset == 0)
		//{
			kick_watchdogs();
		//}
		//ClrWdt();


		if(Jumpiness_Interrupt_Count > 200)
		{
			Jumpiness_Interrupt_Count = 0;

			if( (ocu_robot_talk == 0) && robot_moving_flag)
			{
				//turn off audio transmitter
				if(audio_pulse_state)
				{
					audio_pulse_state = 1;

					Audio_pin = 1;
					Amp_pin = 0;
					Mic_pin = 0;
				}
				//turn on audio transmitter, but turn off audio sources
				else
				{
					audio_pulse_state = 0;

					Audio_pin = 0;
					Amp_pin = 0;
					Mic_pin = 0;
				}

			}


		}

		if(increment_minute_flag)
		{


			increment_minute_flag = 0;

			//only update the battery meter if the robot is stationary.
			//This prevents most erroneously low readings.  Also, if the
			//voltage has not been updated for 5 minutes (such as for continuous driving,
			//update it anyway.
			display_hours();
			if((!robot_moving_flag) || (voltage_not_updated_counter >= 5) )
			{
				block_ms(10);
				display_voltage();
				block_ms(10);
				voltage_not_updated_counter = 0;
			}
			else
			{
				voltage_not_updated_counter++;
			}
			increment_minute_counter(1);
			

			

		}

		//only run the speed/current loop if motors are on.
		//otherwise, speed will ramp up unnecessarily
		if(motors_off_flag == 0)
		{

			//
			//Current control stuff
			//
		
			//every 2ms, average in another A/D value (software low-pass)
			if(T3_Interrupt_Count >= last_T3_Interrupt_Count+2)
			{
				last_T3_Interrupt_Count = T3_Interrupt_Count;
				motor_current_avg = average_motor_current(NO);
				
	
			}
			//every 20ms, get averaged A/D values, and run current loop
			if(T3_Interrupt_Count > 20)
			{
				last_T3_Interrupt_Count = 0;
				T3_Interrupt_Count = 0;
				motor_current_avg = average_motor_current(YES);

				speed_control_loop_quadrants(0);
				//hijacking this timer interrupt to display voltage
				voltage_display_counter++;
				if(voltage_display_counter > 20)
				{
					//display_voltage();
					block_ms(10);
					voltage_display_counter = 0;
				}
			
			}	
			//end current control stuff
	

		}


		if(turn_speed_down_flag)
		{
			if(Current_Interrupt_Count < 15000)
				Max_Speed = 35+Current_Interrupt_Count/256;
			else 
			{
				Max_Speed = 100;	
				turn_speed_down_flag = 0;
			}
		}
		else
		{
			if(Current_Interrupt_Count < 5000)
			{
				if(motor_current_avg > 20)
				{
					turn_speed_down_flag = 1;
					Current_Interrupt_Count = 0;
				}
				Max_Speed = 25+Current_Interrupt_Count/128;
				
			}
			else
				Max_Speed = 100;
		}
		//don't allow to overflow
		if(Current_Interrupt_Count > 65000)
			Current_Interrupt_Count = 62000;


		if (Handle_Messages()){				//If a new valid message is loaded into the data registers

			//start responding to WDT again
			trigger_reset = 0;
			//OSD_clear_screen();
			//block_ms(2);
			
			//erases "Waiting for OCU" string without affecting any other text on the screen
			OSD_display_string("                           ",2,1,27);
			block_ms(10);

			//if the motors have been turned off due to a loss of OCU data signal, turn them back on, as a signal has been received
			if(motors_off_flag)
			{

					/*Relay_Resistor_Control = ON;
					block_ms(200);
					Relay_Power_Control = ON;
					motors_off_flag = 0;*/

					//with the new power board, there is a separate mosfet for the resistor

				//also in the new power board, power and resistor lines are crossed
				Relay_Resistor_Control = ON;
				block_ms(500);
				Relay_Power_Control = ON;
				Relay_Resistor_Control = OFF;
				motors_off_flag = 0;

				//wait some time for controllers to initialize
				block_ms(1500);
		
			}




			if(Is_Robot_Upside_Down())
			{
				Robot_Upside_Down_Counter++;
			}
			else
			{
				Robot_Upside_Down_Counter = 0;
			}

			if(Robot_Upside_Down_Counter < 3)
			{
				disable_driving = 0;
			}
			else
			{
				disable_driving = 1;
			}
				

			Update_Robot_Commands(disable_driving);
			Timeout_Counter = 0;


			if(first_datalink_message_received == 0)
			{
				block_ms(10);
				display_voltage();
				block_ms(10);
				first_datalink_message_received = 1;
			}
		}

		//Commands that run at 50 Hz
		if(Timer1IsOverflowEvent()){
			Timeout_Counter++;

			/*if(Is_Robot_Upside_Down())
			{
				Robot_Upside_Down_Counter++;
			}
			else
			{
				Robot_Upside_Down_Counter = 0;
			}

			//if the robot has been upside down for between 2 and 5 seconds
			if( Robot_Upside_Down_Counter >= 100 && Robot_Upside_Down_Counter <= 250)
			{

				//if the OCU link has been lost for between 2 and 5 seconds
				if(Timeout_Counter >=  100 && Timeout_Counter <= 250)
				{

					Blocking_Self_Right();
					Robot_Upside_Down_Counter = 251;
					Timeout_Counter = 251;

				}

			}*/
			
					
			if (Timeout_Counter <= 25){

				
			}
			else if (Timeout_Counter <= 200)
			{
				speed_control_loop_quadrants(1);
				Set_Desired_Motor_Pulse_Quadrants(0x7F, 0x7F, 0, 0);
			}
			//500 counts is 10 seconds, 50 counts / sec
			else if (Timeout_Counter <= 500){
				
				stop_payload_motion();
				speed_control_loop_quadrants(1);
				Set_Desired_Motor_Pulse_Quadrants(0x7F, 0x7F, 0, 0);
			
				Amp_pin = 0;				//Audio amp off
				Led_pin = 0;				// turn off led
				Mic_pin = 0;				// turns off mic

				OSD_display_string("Waiting for OCU...   ",2,1,21);
				block_ms(10);

				Relay_Resistor_Control = OFF;
				Relay_Power_Control = OFF;
				motors_off_flag = 1;
				if(Timeout_Counter > 30000)
					Timeout_Counter = 30000;

				
			}
			else if(Timeout_Counter <= 650)
			{

				//stop responding to the hardware WDT until it resets.
				trigger_reset = 1;
				ClrWdt();
			}
			else
			{
				//OSD_display_string("Timeout. Resetting Robot...",2,1,27);
				block_ms(50);
				
				ClrWdt();

			}//end runaway counter
		}//end 50Hz if
    }//end while loop
}//end main function




void check_clock_failure(void)
{


	unsigned char test_chars[4] = {0x4c,0x4c,0x4c,0x4c};

	kick_watchdogs();
	//ClrWdt();
	block_ms(20);
	kick_watchdogs();
	//ClrWdt();
	test_chars[1] = OSCCONbits.COSC;
//	OSD_display_chars(test_chars,4,4,4);
	block_ms(100);
	//OSD_display_string("testing",2,2,7);
	if(OSCCONbits.COSC == 0b000)
	{
		OSD_display_string("Robot warming up...",2,1,19);
		
	}
	while(OSCCONbits.COSC == 0b000)
	{
		//kick_watchdogs();
		//ClrWdt();
	}
/*	for (i=1;i<20000;i++)
	{
		ClrWdt();
		block_ms(2);
	}*/




}
