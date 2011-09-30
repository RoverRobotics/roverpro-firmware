#include "include/Compiler.h"
#include "Dynamixel.h"
#include "RS485.h"
#include "Timers.h"

#define WRITE_DATA 0x03

//unsigned char Dynamixel_position_msb[6];
//unsigned char Dynamixel_position_lsb[6];
int Dynamixel_position[6];
unsigned char Dynamixel_direction[6];
//unsigned char Dynamixel_speed_msb[6];
//unsigned char Dynamixel_speed_lsb[6];
int Dynamixel_speed[6];

unsigned char Dynamixel_direction[6];

void Dynamixel_Test(void)
{
//	TRISEbits.TRISE4 = 0;
//	PORTEbits.RE4 = 0;
	RS485_Init();
	while(1)
	{
	}
}

/*void block_ms2(unsigned int ms)
{
	unsigned int i;
	unsigned int j;

	for(i=0;i<3200;i++)

	{
		for(j=0;j<ms;j++);
	}
}*/


void Dynamixel_Init(void)
{
	unsigned char i;
	Init_Timer2();
	Init_Timer3();

	for(i=0;i<6;i++)
	{
		//Dynamixel_position_msb[i] = 0x01;
		//Dynamixel_position_lsb[i] = 0xFF;
		Dynamixel_position[i] = 512;
		Dynamixel_direction[i] = 1;
		Dynamixel_speed[i] = 0;
		//Dynamixel_speed_msb[i] = 0;
		//Dynamixel_speed_lsb[i] = 0xFF;
	}
	Dynamixel_direction[0] = 0;
	
}










unsigned char return_checksum(unsigned char* data, unsigned char length)
{
	unsigned char checksum;
	unsigned char i;
	checksum = 0;

	//skip first two bytes, add
	for(i=0;i<length;i++)
	{
		checksum += data[i+2];
	}

	return (0xFF - checksum);


}

/*void command_servos(unsigned char* direction, unsigned char* speed)
{
	unsigned char i;
	unsigned int position_int;


	for(i=0;i<6;i++)
	{

		Dynamixel_direction[i] = direction[i];
		Dynamixel_speed_msb[i] = 0x00;
		Dynamixel_speed_lsb[i] = 0xFF;


	}






}*/

/*void command_servo(unsigned char servo_address, unsigned char position)
{
	unsigned char message[11];
	unsigned char position_lsb;
	unsigned char position_msb;
	unsigned char speed_lsb;
	unsigned char speed_msb;
	unsigned int position_int;

	unsigned char i;

	speed_lsb = 0xFF;
	speed_msb = 0x00;
	//position_lsb = 0xFF;
	//position_msb = 0x00;

	position_int = (unsigned int)position*10;
	position_msb = (position_int>>8) & 0x03;
	position_lsb = position_int&0xFF;


	message[0] = 0xFF;
	message[1] = 0xFF;

	message[2] = servo_address;
	//length
	message[3] = 0x07;
	message[4] = WRITE_DATA;
	message[5] = 0x1E;
	message[6] = position_lsb;
	message[7] = position_msb;
	message[8] = speed_lsb;
	message[9] = speed_msb;
	message[10] = return_checksum(message,0x08);



	RS485_Send(message,11);



}*/

void servo_loop(void)
{
	unsigned int i;
	static char servo_number= 0x01;
	RS485_Message_Ready = 0;
	Dynamixel_Init();
	//command_servo(0x01,20);
	i=0;
	while(1)
	{
		
		/*if(RS485_Message_Ready)
		{
			command_servo(servo_number,i);
			RS485_Message_Ready = 0;
			if(i<99) i++;
			servo_number++;
			if(servo_number > 2) servo_number = 1;
		}*/



	}







}






void Send_Servo_Message(unsigned char servo_address)
{

	unsigned char message[11];
	unsigned char position_lsb;
	unsigned char position_msb;
	unsigned char speed_lsb;
	unsigned char speed_msb;
	int position_int;
	static unsigned int temp_position_int = 0x00;

	unsigned char i;


	//position_lsb = 0xFF;
	//position_msb = 0x00;

	//position_int = (int)Dynamixel_position_msb[servo_address-1]*0x100+Dynamixel_position_lsb[servo_address-1];
	//temp_position_int += 10;
	//position_int = temp_position_int;
	//increment the position, to simulate velocity control

	switch (Dynamixel_direction[servo_address-1])
	{
		//reverse
		case 0:
			Dynamixel_position[servo_address-1]-=POSITION_STEP;
		break;
		//stopped
		case 1:
			//do nothing -- hold position
		break;
		case 2:
			Dynamixel_position[servo_address-1]+=POSITION_STEP;
		break;

	}

	if(Dynamixel_position[servo_address-1] > 1023) Dynamixel_position[servo_address-1] = 1023;

	if(Dynamixel_position[servo_address-1] < 0) Dynamixel_position[servo_address-1] = 0;

	position_msb = (Dynamixel_position[servo_address-1]>>8) & 0x03;
	position_lsb = Dynamixel_position[servo_address-1]&0xFF;

	speed_msb = (Dynamixel_speed[servo_address-1]>>8) & 0x03;
	speed_lsb = Dynamixel_speed[servo_address-1]&0xFF;
	


	message[0] = 0xFF;
	message[1] = 0xFF;

	message[2] = servo_address;
	//message[2] = 0x02;
	//length
	message[3] = 0x07;
	message[4] = WRITE_DATA;
	message[5] = 0x1E;
	message[6] = position_lsb;
	message[7] = position_msb;
	//message[6] = 0x00;
	//message[7] = 0x01;
	message[8] = speed_lsb;
	message[9] = speed_msb;
	//message[8] = 0xFF;
	//message[9] = 0x00;
	message[10] = return_checksum(message,0x08);
	//message[10] = 0xd5;



	RS485_Send(message,11);

}
