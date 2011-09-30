#include "system.h"


unsigned char message_to_payload[MESSAGE_TO_PAYLOAD_LENGTH];
unsigned int return_CRC(unsigned char* data, unsigned char length);
unsigned char message_from_payload_ready = 0;
unsigned char message_from_payload[MESSAGE_FROM_PAYLOAD_LENGTH];
unsigned char payload_type = NONE;
void send_arm_message(unsigned char joystick_h, unsigned char joystick_v, unsigned char left_trigger, unsigned char right_trigger, unsigned char toggle_up, unsigned char toggle_down, unsigned char payload_button);
void reset_PTZ(void);
void send_PTZ_message(unsigned char joystick_h, unsigned char joystick_v, unsigned char left_trigger, unsigned char right_trigger, unsigned char payload_button);
void reset_arm(void);
unsigned char arm_in_use = 0;
void send_PTZ_message(unsigned char joystick_h, unsigned char joystick_v, unsigned char left_trigger, unsigned char right_trigger, unsigned char payload_button)
{
	unsigned char data_to_send[MESSAGE_TO_PAYLOAD_LENGTH-4];
	unsigned char data_byte1 = 0;
	data_to_send[0] = 0x02;
	data_to_send[1] = joystick_h;
	data_to_send[2] = joystick_v;

	if(right_trigger) data_byte1 |= 0x80;
	if(left_trigger) data_byte1 |= 0x40;
	if(payload_button) data_byte1 |= 0x20;

	data_to_send[3] = data_byte1;
	data_to_send[4] = 0x00;

	send_message_to_payload(data_to_send);


	




}

void arm_drive(unsigned char Direction,unsigned char Speed)
{
	int desired_effort[3] = {0,0,0};
	unsigned char i;
	Set_Desired_Motor_Pulse_Quadrants(Direction,Speed,0,0);
	/*set_motor_effort(,RIGHT_MOTOR);
	set_motor_effort(,LEFT_MOTOR);
	set_motor_effort(127.5,FLIPPER_MOTOR)*/
	for(i=0;i<3;i++)
	{
		desired_effort[i] = Return_Desired_Motor_Effort_Quadrants(i)/4;
	}

	if(((desired_effort[LEFT_MOTOR] > 0) && (desired_effort[RIGHT_MOTOR] > 0)) || 
		((desired_effort[LEFT_MOTOR] < 0) && (desired_effort[RIGHT_MOTOR] < 0)))
	{
		desired_effort[LEFT_MOTOR] = -desired_effort[LEFT_MOTOR];
		desired_effort[RIGHT_MOTOR] = -desired_effort[RIGHT_MOTOR];
	}

	set_motor_effort(0,FLIPPER_MOTOR);
	set_motor_effort(desired_effort[RIGHT_MOTOR],RIGHT_MOTOR);
	set_motor_effort(desired_effort[LEFT_MOTOR],LEFT_MOTOR);



}


unsigned char return_payload_type(void)
{
	return payload_type;

}

void send_arm_message(unsigned char joystick_h, unsigned char joystick_v, unsigned char left_trigger, unsigned char right_trigger, unsigned char toggle_up, unsigned char toggle_down, unsigned char payload_button)
{

	unsigned char data_to_send[MESSAGE_TO_PAYLOAD_LENGTH-4];
	unsigned char data_byte1 = 0;
	data_to_send[0] = 0x00;
	data_to_send[1] = joystick_h;
	data_to_send[2] = joystick_v;

	if(left_trigger) data_byte1 |= 0x80;
	if(right_trigger) data_byte1 |= 0x40;
	if(toggle_up) data_byte1 |= 0x20;
	if(toggle_down) data_byte1 |= 0x10;

	data_to_send[3] = data_byte1;
	data_to_send[4] = 0x00;

	send_message_to_payload(data_to_send);

}

void handle_payload_message(void)
{
	unsigned char L_LR,L_UD,R_LR,R_UD;
	unsigned char led_on, ocu_robot_talk, payload_button, flipper_up, flipper_down, left_trigger, right_trigger;



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





	switch(payload_type)
	{
		case NONE:
		break;
		case PTZ:
			send_PTZ_message(R_LR,R_UD,left_trigger,right_trigger,payload_button);
		break;
		case ARM:
			send_arm_message(R_LR,R_UD,left_trigger,right_trigger,flipper_up,flipper_down,payload_button);
		break;

	}



}


void set_payload_type(unsigned char new_type)
{
	payload_type = new_type;
}

void payload_robot_control(void)
{

	unsigned char i;
	unsigned char joystick_counter = 25;

//	break_in_robot();
	for(i=0;i<14;i++)
	{
		Set_Last_Datalink_Message(0x00,i);
	}
	Set_Last_Datalink_Message(0xFF,0);
	Set_Last_Datalink_Message(0xCC,1);


	while(1)
	{
		kick_watchdogs();
		//ClrWdt();
		if(message_from_payload_ready)
		{
			message_from_payload_ready = 0;
			if(Is_payload_CRC_valid(message_from_payload,MESSAGE_FROM_PAYLOAD_LENGTH))
			{

				switch(message_from_payload[2])
				{
					case 0x00:
						Set_Last_Datalink_Message(message_from_payload[3],6);
						Set_Last_Datalink_Message(message_from_payload[4],7);
						joystick_counter++;
					break;
					case 0x01:
						Set_Last_Datalink_Message(message_from_payload[3],8);
						Set_Last_Datalink_Message(message_from_payload[4],9);
					break;
					case 0x02:

						Set_Last_Datalink_Message(message_from_payload[3],10);
						Set_Last_Datalink_Message(message_from_payload[4],11);

						Update_Robot_Commands(0);
						speed_control_loop_quadrants(0);

					break;

				}

						
			}
		}
		


	}




}

void stop_payload_motion(void)
{

	switch(payload_type)
	{
		case NONE:
		break;
		case PTZ:
			send_PTZ_message(127.5,127.5,0,0,1);
		break;
		case ARM:
			send_arm_message(127.5,127.5,0,0,0,0,0);
		break;

	}

		//send_PTZ_message(127.5,127.5,0x20);
		
}



void reset_payload(void)
{
	switch(payload_type)
	{
		case NONE:
		break;
		case PTZ:
			reset_PTZ();
		break;
		case ARM:
			reset_arm();
		break;

	}

}


//sends command that resets PTZ in software (to recalibrate)
void reset_PTZ(void)
{
	unsigned char data_to_send[MESSAGE_TO_PAYLOAD_LENGTH-4] = {0x03,0x03,0x00,0x00,0x000};

	send_message_to_payload(data_to_send);

}

void reset_arm(void)
{
	unsigned char data_to_send[MESSAGE_TO_PAYLOAD_LENGTH-4] = {0x03,0x03,0x00,0x00,0x000};

	send_message_to_payload(data_to_send);
}

void send_message_to_payload(unsigned char* data_to_send)
{
	unsigned int CRC;
	unsigned char i;

	CRC = return_CRC(data_to_send,MESSAGE_TO_PAYLOAD_LENGTH-4);

	message_to_payload[0] = 0xFF;
	message_to_payload[1] = 0xCC;

	message_to_payload[MESSAGE_TO_PAYLOAD_LENGTH-2] = CRC>>8;
	message_to_payload[MESSAGE_TO_PAYLOAD_LENGTH-1] = CRC&0xFF;
//	message_to_robot[MESSAGE_TO_ROBOT_LENGTH-2] = 0xaa;
//	message_to_robot[MESSAGE_TO_ROBOT_LENGTH-1] = 0xbb;
	

	for(i=0;i<MESSAGE_TO_PAYLOAD_LENGTH-4;i++)
	{
		message_to_payload[i+2] = data_to_send[i];
		
	}

	U2TXREG = message_to_payload[0];
//	IEC0bits.U2TXIE = 1;

}






void init_uart2(void)
{

	//RP2 is tx to payload
	RPOR1bits.RP2R = 5;
	//RP4 is RX from payload
	RPINR19bits.U2RXR = 4;

	U2MODE = 0x0000;

	//32e6/32/9600-1 = 103.17
	U2BRG = SYSCLK/32/9600-1;

	U2MODEbits.UARTEN = 1;

	U2STAbits.UTXEN = 1;

	IEC1bits.U2TXIE = 1;
	IEC1bits.U2RXIE = 1;




}

char Is_payload_CRC_valid(unsigned char* data, unsigned char length)
{
	
	



	unsigned int CRC_received = 0;
	unsigned int CRC_calculated = 0;

	unsigned char data_to_CRC[20];
	unsigned char i;
	unsigned char test_data[5];

	for(i=2;i<length-2;i++)
	{
		data_to_CRC[i-2] = data[i];
	}
	


	CRC_received = (data[length-2]<<8)+(data[length-1]);

	CRC_calculated = return_CRC(data_to_CRC,length-4);

	test_data[0] = CRC_received>>8;
	test_data[1] = CRC_received&0xFF;
	test_data[2] = CRC_calculated>>8;
	test_data[3] = CRC_calculated&0xFF;
	test_data[4] = 0xaa;

	//send_message_to_robot(test_data);



	if(CRC_received == CRC_calculated)
		return 1;
	
	return 0;


}

void return_version_info(void)
{
	const unsigned char builddate[12] = __DATE__; 
	const unsigned char build_time[12] = __TIME__;
	unsigned char version_string[8] = CODE_VERSION;
	
	unsigned char payload_data[5];

	unsigned char i;
	payload_data[0] = 0x04;
	//send version
	for(i=0;i<2;i++)
	{
		payload_data[1] = version_string[i*4];
		payload_data[2] = version_string[i*4+1];
		payload_data[3] = version_string[i*4+2];
		payload_data[4] = version_string[i*4+3];
		send_message_to_payload(payload_data);
		block_ms(100);
	}

	//send date
	for(i=0;i<3;i++)
	{
		payload_data[1] = builddate[i*4];
		payload_data[2] = builddate[i*4+1];
		payload_data[3] = builddate[i*4+2];
		payload_data[4] = builddate[i*4+3];
		
		send_message_to_payload(payload_data);
		block_ms(100);
	}
	for(i=0;i<3;i++)
	{
		payload_data[1] = build_time[i*4];
		payload_data[2] = build_time[i*4+1];
		payload_data[3] = build_time[i*4+2];
		payload_data[4] = build_time[i*4+3];
		
		send_message_to_payload(payload_data);
		block_ms(100);
	}

}

unsigned int return_CRC(unsigned char* data, unsigned char length)
{
	static unsigned int crc;
	unsigned char i;
	crc = 0;

	for(i=0;i<length;i++)
	{
		crc = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= data[i];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= (crc << 8) << 4;
		crc ^= ((crc & 0xff) << 4) << 1;
		crc &= 0xFFFF;
	}
	return crc; 
}



void  __attribute__((interrupt, auto_psv)) _U2TXInterrupt(void)
{

	static unsigned char byte_counter = 0;
	

	IFS1bits.U2TXIF = 0;
	byte_counter++;



	if(byte_counter >= MESSAGE_TO_PAYLOAD_LENGTH )
	{
		byte_counter = 0;

	}
	else
	{
		U2TXREG = message_to_payload[byte_counter];
	}



}


void  __attribute__((interrupt, auto_psv)) _U2RXInterrupt(void)
{

	static unsigned char message_index = 0;
	static unsigned char message_buffer[MESSAGE_FROM_PAYLOAD_LENGTH];
	unsigned char i;
	unsigned char new_byte;


	IFS1bits.U2RXIF = 0;

	new_byte = U2RXREG;
	
	message_buffer[message_index] = new_byte;

	//avoid reading in a new message until the last one has been processed
	if(message_from_payload_ready == 0)
	{
		switch(message_index)
		{
			case 0x00:
				if(new_byte != 0xFF)
					message_index = -1;
				break;
			case 0x01:
				if(new_byte != 0xCC)
					message_index = -1;
				break;
			case MESSAGE_FROM_PAYLOAD_LENGTH-1:
	
					for(i=0;i<MESSAGE_FROM_PAYLOAD_LENGTH;i++)
					{
						message_from_payload[i] = message_buffer[i];
					}
					message_from_payload_ready = 1;
					message_index = -1;
	
				break;
		}
		message_index++;
	}




}
