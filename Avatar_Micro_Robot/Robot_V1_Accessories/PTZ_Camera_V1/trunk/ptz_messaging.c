#include "system.h"


void pan_tilt_zoom_camera(unsigned char pan_right, unsigned char tilt_up, unsigned char zoom_in, unsigned char zoom_out,unsigned char pan_speed, unsigned char tilt_speed);
void set_ptz_checksum(void);
void stop_motion(void);
void go_to_home(void);
void send_ptz_message(unsigned char byte1, unsigned char byte2, unsigned char byte3, unsigned char byte4);
void set_preset(void);
void display_menu(void);
void reset_camera(void);
void manual_centering(void);

unsigned char ptz_message[8] = {0xa0, 0, 0,0,0,0,0xaf,0};
unsigned char ptz_message_copy[8];

unsigned char homing = 0;


void init_uart2(void)
{

	//RP11 is U2TX
	RPOR5bits.RP11R = 5;

	//RP10 is U2RX
	RPINR19bits.U2RXR = 10;

	//16 prescaler
	U2MODEbits.BRGH = 0;

	//20e6/2/16/2400 - 1 = 259.41
	U2BRG = SYSCLK/32/2400-1;
	//U2BRG = 60;

	U2MODEbits.UARTEN = 1;

	U2STAbits.UTXEN = 1;

	


	IEC1bits.U2TXIE = 1;

	




}

void set_ptz_checksum(void)
{
	unsigned char i;
	unsigned char checksum = 0;

	for(i=0;i<7;i++)
	//for(i=1;i<6;i++)
	{
		checksum ^= ptz_message_copy[i];
	}
	ptz_message_copy[7] = checksum;


}


void stop_motion(void)
{

	send_ptz_message(0x00,0x00,0x00,0x00);

}

void go_to_home(void)
{

	//manual_centering();
	if(homing == 0)
	{
		stop_motion();
		block_ms(50);
	}
	//this 
	//send_ptz_message(0x00,0x07, 0x00, 0x22);
	//go to preset 1
	send_ptz_message(0x00,0x07,0x00,0x01);
	block_ms(50);
	homing = 1;
	

}

void manual_centering(void)
{

	unsigned char data_byte = 0;
	unsigned int breakout_counter = 0;
	static unsigned char pan_centered = 0;
	unsigned int pan_sensor_counter = 0;
	
	
	//first run of this function
	if(homing == 0)
	{
		pan_centered = 0;
	}


	//if pan isn't centered, spin around until the sensor is hit.  Otherwise, just tilt down and zoom out.
	if(pan_centered == 0)
	{
		//zoom wide
		data_byte |= 0x40;
		data_byte |= 0x10; //tilt down
		data_byte |= 0x02; //pan right
	
		send_ptz_message(0x00,data_byte,0x20,0x3F);
		//if(PAN_SENSOR)

		
		//pan until sensor is hit
		while(1)
		{
			
			if(PAN_SENSOR)
			{
				pan_sensor_counter++;
			}
			else
				pan_sensor_counter = 0;

			if(pan_sensor_counter > 10)
			{
				pan_centered = 1;
				stop_motion();
				block_ms(200);

				send_ptz_message(0x00,0x54,0x20,0x3F);
				block_ms(300);
				break;
			}

			block_ms(1);
			breakout_counter++;
			if(breakout_counter >= 3000)
			{
				break;
			}
			else if(breakout_counter >= 1200)
			{
				send_ptz_message(0x00,0x12,0x20,0x3F);
			}
	
		}

	}

	//stop pan, keep zooming out and tilting down
	data_byte = 0x50;
	send_ptz_message(0x00,data_byte,0x00,0x3F);
	homing = 1;







}

void pan_tilt_zoom_camera(unsigned char pan_right, unsigned char tilt_up, unsigned char zoom_in, unsigned char zoom_out,unsigned char pan_speed, unsigned char tilt_speed)
{
	unsigned char data_byte = 0x00;
	
	if(pan_right)
		data_byte |= 0x02;
	else
		data_byte |= 0x04;

	if(tilt_up)
		data_byte |= 0x08;
	else
		data_byte |= 0x10;

	if(zoom_in)
		data_byte |= 0x20;
	else if(zoom_out)
		data_byte |= 0x40;


	

	send_ptz_message(0x00, data_byte, pan_speed, tilt_speed);


}

void display_menu(void)
{
	send_ptz_message(0x00,0x03,0x00,95);
	block_ms(100);




}


void reset_camera(void)
{
	/*	//bring up menu
		block_ms(100);
		send_ptz_message(0x00,0x03,0x00,95);
		block_ms(100);
		//right
		send_ptz_message(0x00,0x02,0x3f,0x00);
		block_ms(100);
		//down
		send_ptz_message(0x00,0x10,0x00,0x3f);
		block_ms(100);
		//right
		send_ptz_message(0x00,0x02,0x3f,0x00);
		block_ms(100);
		//iris open
		send_ptz_message(0x04,0x00,0x00,0x00);
		block_ms(500);
		//if something goes wrong, and menu is still up, make it go away
		send_ptz_message(0x00,0x03,0x00,95);
		block_ms(100);

	//right,down,right,iris_open
	*/

	PTZ_POWER_CONTROL = 0;
	block_ms(500);
	PTZ_POWER_CONTROL = 1;
	//send_ptz_message(0x00,0x0f,0x00,0x00);
	//block_ms(50);
	

}


void navigate_menu(void)
{

	unsigned char joystick_h;
	unsigned char joystick_v;
	//unsigned char tilt_up = 0;
	//unsigned char pan_right = 0;
	unsigned data_byte = 0;
	//bytes 4 and 5 are joystick values
	joystick_h = message_from_robot[4];
	joystick_v = message_from_robot[5];



	if(joystick_h > 167)
		data_byte |= 0x02;
	else if(joystick_h < 87)
		data_byte |= 0x04;
	if(joystick_v > 167)
		data_byte |= 0x10;
	else if(joystick_v < 87)
		data_byte |= 0x08;

	//no
	if(message_from_robot[6]&0x80)
	{
		//iris close
		//send_ptz_message(0x80,0x00,0x00,0x00);
		send_ptz_message(0x08,0x00,0x00,0x00);
		//go_to_home();
		//block_ms(100);
		//send_ptz_message(0x00,0x00,0x00,0x00);
	}
	else if(message_from_robot[6]&0x40)
	{
		//iris open
		send_ptz_message(0x04,0x00,0x00,0x00);
		//block_ms(100);
		//send_ptz_message(0x00,0x00,0x00,0x00);
	}
	else
	{

		if( abs (joystick_h-127.5) > 50)
		{
				//pan only
				send_ptz_message(0x00,data_byte,0x10,0x00);
				//pan_tilt_zoom_camera(pan_right,tilt_up,0x00,0x00,0x10,0x00);
		}
		else if (abs(joystick_v-127.5) > 50)
		{
				//tilt only
				send_ptz_message(0x00,data_byte,0x00,0x10);
				//pan_tilt_zoom_camera(pan_right,tilt_up,0x00,0x00,0x00,0x10);
		}
		else
		{
			//stop motion
			send_ptz_message(0x00,0x00,0x00,0x00);
		}
	}



}

void set_preset(void)
{
	unsigned char joystick_h;
	unsigned char joystick_v;
	unsigned char tilt_up = 0;
	unsigned char pan_right = 0;
	unsigned char zoom_in = 0;
	unsigned char zoom_out = 0;
	unsigned char tilt_speed;
	unsigned char pan_speed;

	//bytes 4 and 5 are joystick values
	joystick_h = message_from_robot[4];
	joystick_v = message_from_robot[5];

	if(message_from_robot[6] & 0x08)
	{
	
		if(joystick_v > 127.5)
			tilt_up = 1;
		if(joystick_h < 127.5)
			pan_right = 1;
	
		tilt_speed = abs(joystick_v-127.5)/3;
		pan_speed = abs(joystick_h-127.5)/3;

		if(tilt_speed < 5)
			tilt_speed = 0;
		if(pan_speed < 5)
			pan_speed = 0;
	
		if(tilt_speed > 32)
			tilt_speed = 32;
		if(pan_speed > 32)
			pan_speed = 32;
	
		if(tilt_speed < 3)
			tilt_speed = 0;
		if(pan_speed < 3)
			pan_speed = 0;
	
	
	
		if(message_from_robot[6] & 0x80)
			zoom_in = 1;
		else if(message_from_robot[6] & 0x40)
			zoom_out = 1;
		
		/*if(zoom_in)
			data_byte |= 0x20;
		else if(zoom_out)
			data_byte |= 0x40;*/
		pan_tilt_zoom_camera(pan_right,tilt_up,zoom_in,zoom_out,pan_speed,tilt_speed);
		// send_ptz_message(0x00,data_byte,pan_speed,tilt_speed)
	}

	//set current position to preset 1
	else if(message_from_robot[6]&0x20)
	{
		send_ptz_message(0x00,0x03,0x00,0x01);
		block_ms(500);
		//iris open to confirm
		send_ptz_message(0x04,0x00,0x00,0x00);
		block_ms(100);

	}
	//go to preset 1, for testing
	else if(message_from_robot[6]&0x10)
	{
		send_ptz_message(0x00,0x07,0x00,0x01);
		block_ms(100);

	}
		
	







}

void decode_message_from_robot(void)
{
	unsigned char joystick_h;
	unsigned char joystick_v;
	unsigned char pan_right = 0;
	unsigned char tilt_up = 0;
	unsigned char zoom_in = 0;
	unsigned char zoom_out = 0;
	unsigned char tilt_speed = 0;
	unsigned char pan_speed = 0;

	//message types:
	//0x02 - PTZ control message (sent from robot)
	//0x03 - PTZ programming message (sent from computer)


	if(message_from_robot[2] == 0x02)
	{	
		joystick_h = message_from_robot[3];
		joystick_v = message_from_robot[4];
	
		if(joystick_v > 127.5)
			tilt_up = 1;
		if(joystick_h < 127.5)
			pan_right = 1;
	
		tilt_speed = abs(joystick_v-127.5)/3;
		pan_speed = abs(joystick_h-127.5)/3;
	
		if(tilt_speed > 32)
			tilt_speed = 32;
		if(pan_speed > 32)
			pan_speed = 32;
	
		if(tilt_speed < 3)
			tilt_speed = 0;
		if(pan_speed < 3)
			pan_speed = 0;
	
	
	
		if(message_from_robot[5] & 0x80)
			zoom_in = 1;
		else if(message_from_robot[5] & 0x40)
			zoom_out = 1;
		
		if(message_from_robot[5] & 0x20)
		{
			//homing = 1;
			go_to_home();
		}
		else
		{
			homing = 0;
			pan_tilt_zoom_camera(pan_right,tilt_up,zoom_in,zoom_out,pan_speed,tilt_speed);
		}
	}

	//camera programming mode -- used to change camera settings and presets
	else if(message_from_robot[2] == 0x03)
	{
		switch(message_from_robot[3])
		{
			
			case 0x00:
				display_menu();
			break;
			case 0x01:
				navigate_menu();
			break;
			case 0x02:
				set_preset();
			break;
			case 0x03:
				reset_camera();
			break;
		}

	}
//	send_ptz_message(0x00,0x02,0x20,0x00);

}

void send_ptz_message(unsigned char byte1, unsigned char byte2, unsigned char byte3, unsigned char byte4)
{
	unsigned char i;
	ptz_message[2] = byte1;
	ptz_message[3] = byte2;
	ptz_message[4] = byte3;
	ptz_message[5] = byte4;
	for(i=0;i<8;i++)
	{
		ptz_message_copy[i] = ptz_message[i];
	}
	set_ptz_checksum();


	U2TXREG = ptz_message_copy[0];

}





void  __attribute__((interrupt, auto_psv)) _U2TXInterrupt(void)
{

	static unsigned char byte_counter = 0;
	

	IFS1bits.U2TXIF = 0;
	byte_counter++;

	if(byte_counter >= PTZ_MESSAGE_LENGTH)
	{
		byte_counter = 0;

	}
	else
	{
		U2TXREG = ptz_message_copy[byte_counter];
	}



}

void block_ms(unsigned int ms)
{
	unsigned int i;
	unsigned int j;
	for(i=0;i<2000;i++)
	{

		for(j=0;j<ms;j++)
		{
		}
	}



}
