#include "include/Compiler.h"
#include "Controller.h"
#include "Audio.h"

#define INPUT 1


#define LEFT_JOYSTICK 0x00
#define RIGHT_JOYSTICK 0x01
#define HORIZONTAL_AXIS 0x00
#define VERTICAL_AXIS 0x01
#define JOYSTICK_DEADBAND 15

unsigned char Read_Joystick_Value(char joystick,char axis);
unsigned char Scale_Joystick(int joystick_value, char joystick, char axis);
unsigned char get_battery_voltage(void);

unsigned char OCU_ADDRESS_MSB = 0x00;
unsigned char OCU_ADDRESS_LSB = 0x00;
unsigned char ROBOT_ADDRESS_MSB = 0xaa;
unsigned char ROBOT_ADDRESS_LSB = 0xaa;


void Init_Controller(void)
{

	AD1PCFGL = 0xFFFF;	//set all to digital

	//Button2
	TRISBbits.TRISB15 = INPUT;
	AD1PCFGLbits.PCFG15 = 1;

	//Button3
	TRISBbits.TRISB14 = INPUT;
	AD1PCFGLbits.PCFG14 = 1;

	//Button4
	TRISBbits.TRISB13 = INPUT;
	AD1PCFGLbits.PCFG13 = 1;



	//flipper toggle
	TRISEbits.TRISE2 = INPUT;
	TRISEbits.TRISE1 = INPUT;


	
	TRISBbits.TRISB4 = INPUT;

	AD1CON1 = 0x00E0;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x1f03;
	AD1CHS = 0x0002; //choose AN2 as input 

	AD1PCFGLbits.PCFG2 = 0;
	AD1PCFGLbits.PCFG3 = 0;
	AD1PCFGLbits.PCFG4 = 0;
	AD1PCFGLbits.PCFG5 = 0;

	//battery voltage input
	AD1PCFGLbits.PCFG12 = 0;


	AD1CSSL = 0x0000;
	AD1CON1bits.ADON = 1;


}

void Construct_Controller_Message(void)
{

	unsigned int CRC;
	//unsigned int i;
	//static unsigned char last_audio_state = RECEIVE;
	static unsigned char button_states[2];
	static unsigned char previous_button_states[2];
	static unsigned char audio_state = AUDIO_RX;
	static unsigned char last_light_button;
	static unsigned char last_light_state = OFF;
	unsigned char battery_voltage = 0;
  static unsigned char talk_button_last_held = 1;


//header bytes
	Datalink_Send_Buffer[0] = 0xFF;
	Datalink_Send_Buffer[1] = 0xCC;

	//Source Address
	Datalink_Send_Buffer[2] = OCU_ADDRESS_MSB;
	Datalink_Send_Buffer[3] = OCU_ADDRESS_LSB;

	//Destination Address
	Datalink_Send_Buffer[4] = ROBOT_ADDRESS_MSB;
	Datalink_Send_Buffer[5] = ROBOT_ADDRESS_LSB;



	//joystick A/D values

	Datalink_Send_Buffer[6] = 255-Read_Joystick_Value(LEFT_JOYSTICK,HORIZONTAL_AXIS);
	Datalink_Send_Buffer[7] = Read_Joystick_Value(LEFT_JOYSTICK,VERTICAL_AXIS);


	Datalink_Send_Buffer[8] = 255-Read_Joystick_Value(RIGHT_JOYSTICK,HORIZONTAL_AXIS);
	Datalink_Send_Buffer[9] = Read_Joystick_Value(RIGHT_JOYSTICK,VERTICAL_AXIS);



	//button values

	button_states[0] = 0x00;
	button_states[1] = 0;



	if(PAYLOAD_BUTTON) button_states[0] |= PAYLOAD_BUTTON_MASK;
	if(FLIPPER_UP) button_states[0] |= FLIPPER_UP_MASK;
	if(FLIPPER_DOWN) button_states[0] |= FLIPPER_DOWN_MASK;
	if(LEFT_TRIGGER) button_states[0] |= LEFT_TRIGGER_MASK;
	if(RIGHT_TRIGGER) button_states[0] |= RIGHT_TRIGGER_MASK;






	if(LIGHT_BUTTON && (last_light_button == 0))
	{
		if(last_light_state == OFF)
		{
			last_light_state = ON;
		

		}
		else
		{
			last_light_state = OFF;
		}
	}
	
	if(last_light_state == ON)
			button_states[0] |= LIGHT_ON_MASK;

	last_light_button = LIGHT_BUTTON;



	if(SELECT_POS_1) button_states[1] |= 0x00;
	if(SELECT_POS_2) button_states[1] |= 0x40;

	//for OCU battery voltage
	battery_voltage = get_battery_voltage();
	//> 16       V*33/233 *255/3.3
	if(battery_voltage > 175)
		button_states[1] |= 0x04;
	//> 15
	else if(battery_voltage > 164)
		button_states[1] |= 0x03;
	//> 14.7
	else if(battery_voltage > 157)
		button_states[1] |= 0x02;
	//> 14.0
	else if(battery_voltage > 153)
		button_states[1] |= 0x01;
	//else, almost dead
	else
		button_states[1] |= 0x00;





	audio_state = AUDIO_ENABLE;

	if(Datalink_Send_Buffer[6] > (0x7f+JOYSTICK_DEADBAND) || Datalink_Send_Buffer[6] < (0x7f-JOYSTICK_DEADBAND))
		audio_state = AUDIO_DISABLE;
	else if(Datalink_Send_Buffer[7] > (0x7f+JOYSTICK_DEADBAND) || Datalink_Send_Buffer[7] < (0x7f-JOYSTICK_DEADBAND))
		audio_state = AUDIO_DISABLE;
	if(FLIPPER_UP || FLIPPER_DOWN)
		audio_state = AUDIO_DISABLE;

	if(audio_state == AUDIO_DISABLE)
	{
		Audio_Receive();
		Audio_Mute();

	}
	else
	{
		if(TALK_BUTTON)
		{
			 button_states[0] |= OCU_TO_ROBOT_TALK_MASK;
			Audio_Transmit();
			Audio_Mute();
      talk_button_last_held = 1;
		}
		else
		{
			Audio_Receive();
      if(talk_button_last_held)
        block_ms(50);
			Audio_Unmute();
      talk_button_last_held = 0;
		}
	}

	Datalink_Send_Buffer[10] = button_states[0];
	Datalink_Send_Buffer[11] = button_states[1];






	previous_button_states[0] = button_states[0];
	previous_button_states[1] = button_states[1];



	CRC = return_crc(Datalink_Send_Buffer,10);
	Datalink_Send_Buffer[12] = CRC >> 8;
	Datalink_Send_Buffer[13] = CRC&0xFF;

	Datalink_Send_Buffer[14] = 0xaa;



}

unsigned char get_battery_voltage(void)
{
	AD1CON1bits.ADON = 0;
	AD1CHS = 12;
	AD1CON1bits.ADON = 1;
	Nop();
	AD1CON1bits.SAMP = 1;

	while(AD1CON1bits.SAMP);

	while(!AD1CON1bits.DONE);
	return (ADC1BUF0>>2);


}

unsigned char Read_Joystick_Value(char joystick,char axis)
{
	int temp_joystick_value;
	AD1CON1bits.ADON = 0;
	if(joystick == LEFT_JOYSTICK)
	{
		//left horizontal (AD0)
		if(axis == HORIZONTAL_AXIS)
		{
			//ADCON0 = 0x00;
			AD1CHS = 0x002;
		}
		//left vertical (AD1)
		else
		{
			AD1CHS = 0x003;
		}
	}
	else
	{
		//right horizontal (AD2)
		if (axis == HORIZONTAL_AXIS)
		{
			AD1CHS = 0x004;	
		}
		//right vertical (AD3)
		else
		{	
			AD1CHS = 0x005;
		}
	}

	AD1CON1bits.ADON = 1;
	Nop();
	AD1CON1bits.SAMP = 1;

	while(AD1CON1bits.SAMP);

	while(!AD1CON1bits.DONE);
	temp_joystick_value = (ADC1BUF0>>2);

	if(temp_joystick_value <= 2)
		temp_joystick_value = 127;
	return temp_joystick_value;

}

unsigned char Scale_Joystick(int joystick_value, char joystick, char axis)
{
	int scaled_joystick_value;







	if(joystick == LEFT_JOYSTICK)
	{
		//left horizontal (AD0)
		if(axis == HORIZONTAL_AXIS)
		{
			scaled_joystick_value = (joystick_value - 127 + LH_OFFSET);
			if(scaled_joystick_value > 0)
			scaled_joystick_value = scaled_joystick_value*LH_SCALE_POS;
			else
			scaled_joystick_value = scaled_joystick_value*LH_SCALE_NEG;

		}
		//left vertical (AD1)
		else
		{
			scaled_joystick_value = (joystick_value - 127 + LV_OFFSET);
			if(scaled_joystick_value > 0)
			scaled_joystick_value = scaled_joystick_value*LV_SCALE_POS;
			else
			scaled_joystick_value = scaled_joystick_value*LV_SCALE_NEG;
		}
	}
	else
	{
		//right horizontal (AD2)
		if (axis == HORIZONTAL_AXIS)
		{
			scaled_joystick_value = (joystick_value - 127 + RH_OFFSET);
			if(scaled_joystick_value > 0)
			scaled_joystick_value = scaled_joystick_value*RH_SCALE_POS;
			else
			scaled_joystick_value = scaled_joystick_value*RH_SCALE_NEG;
		}
		//right vertical (AD3)
		else
		{	
			scaled_joystick_value = (joystick_value - 127 + RV_OFFSET);
			if(scaled_joystick_value > 0)
			scaled_joystick_value = scaled_joystick_value*RV_SCALE_POS;
			else
			scaled_joystick_value = scaled_joystick_value*RV_SCALE_NEG;
		}
	}



	scaled_joystick_value = scaled_joystick_value + 127;
	if(scaled_joystick_value > 0xFF) scaled_joystick_value = 0xFF;
	else if(scaled_joystick_value < 0) scaled_joystick_value = 0;

	return (unsigned char) scaled_joystick_value;



}
