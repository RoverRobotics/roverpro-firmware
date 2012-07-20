#include "system.h"

unsigned char CONTROLLER1 = 0;
unsigned char CONTROLLER2 = 0;
unsigned char ROBOT1 = 0xff;
unsigned char ROBOT2 = 0xff;
unsigned char hex_to_ascii(unsigned char hex_char);
unsigned char binding_to_OCU = 0;
void set_stored_address(void)
{
	unsigned char stored_address[2];
	unsigned int CRC_read;
	unsigned int CRC_calculated;
	unsigned char CRC1,CRC2;
	unsigned char already_written;
	unsigned char data_to_CRC[4];
	DataEEInit();
	Nop();
	stored_address[0] = DataEERead(0);
	Nop();
	stored_address[1] = DataEERead(1);
	Nop();
	CRC1 = DataEERead(2);
	Nop();
	CRC2 = DataEERead(3);
	Nop();
	already_written = DataEERead(4);
	Nop();



	data_to_CRC[2] = stored_address[0];
	data_to_CRC[3] = stored_address[1];

	CRC_calculated = return_crc(data_to_CRC,2);
	//CRC_calculated = 0xe615;

	CRC_read = (CRC1<<8)+(CRC2);


	if((stored_address[0] == 0xff) && (stored_address[1] == 0xff) && (already_written != 0xaa))
	{
		bind_to_OCU();
	}
	else if(CRC_read == CRC_calculated)
	{
		CONTROLLER1 = stored_address[0];
		CONTROLLER2 = stored_address[1];
		ROBOT1 = stored_address[0];
		ROBOT2 = stored_address[1];

		//display_robot_address(ROBOT1,ROBOT2,4,4);
		display_robot_address(stored_address[0],stored_address[1],2,1);
		/*block_ms(100);
		display_robot_address(CRC1,CRC2,4,4);
		block_ms(100);
		display_robot_address(CRC_calculated>>8,CRC_calculated&0xFF,5,5);
		block_ms(100);
		display_robot_address(CRC_read>>8,CRC_read&0xFF,6,6);*/
		block_ms(2000);
	}
	else
	{
		OSD_display_string("Invalid Robot Address",2,1,21);

	}






}

void unbind_robot(void)
{
	unsigned char i;
	DataEEInit();
	Nop();
	for(i=0;i<5;i++)
	{
		DataEEWrite(0xff,i);
		Nop();
	}
	bind_to_OCU();

}


void bind_to_OCU(void)
{

	unsigned char OCU_MSB, OCU_LSB, Robot_MSB, Robot_LSB;
	binding_to_OCU = 1;
	OSD_display_string("Waiting for OCU Bind...",2,1,23);
	block_ms(100);
	while(1)
	{
		kick_watchdogs();
		if (Handle_Messages()){
	

				OCU_MSB = Return_Last_Datalink_Message(2);
				OCU_LSB = Return_Last_Datalink_Message(3);
				Robot_MSB = Return_Last_Datalink_Message(4);
				Robot_LSB = Return_Last_Datalink_Message(5);

				//allow for demo OCU (0000ffff) to control robot one time
				if((OCU_MSB == 0x00) && (OCU_LSB == 0x00) && (Robot_MSB == 0xff) && (Robot_LSB == 0xff) )
				{
					CONTROLLER1 = 0x00;
					CONTROLLER1 = 0x00;
					ROBOT1 = 0xff;
					ROBOT1 = 0xff;
					OSD_display_string("                       ",2,1,23);
					block_ms(20);
					return;

				}
	
				if( (OCU_MSB == Robot_MSB) && (OCU_LSB == Robot_LSB))
				{
					store_address(Robot_MSB,Robot_LSB);
					binding_to_OCU = 0;
					return;
				}
	
			
		}
	}
}

void store_address(unsigned char MSB, unsigned char LSB)
{
	unsigned char data_to_CRC[4];
	unsigned int CRC_calculated = 0;
	

	OSD_display_string("Binding to Address ",2,1,19);
	block_ms(50);
	display_robot_address(MSB,LSB,2,1);

	data_to_CRC[2] = MSB;
	data_to_CRC[3] = LSB;

	CRC_calculated = return_crc(data_to_CRC,2);

	//DataEEInit();
	DataEEWrite(MSB,0);
	Nop();
	DataEEWrite(LSB,1);
	Nop();
	DataEEWrite((CRC_calculated >> 8),2);
	Nop();
	DataEEWrite((CRC_calculated & 0xff),3);
	Nop();

//	write a specific value to a known location,
//	so we know that the address has been written for
//	the first time.
	DataEEWrite(0xAA,4);

	Nop();

	CONTROLLER1 = MSB;
	CONTROLLER2 = LSB;
	ROBOT1 = MSB;
	ROBOT2 = LSB;

	block_ms(2000);
	OSD_clear_screen();
	block_ms(100);

}


void display_robot_address(unsigned char MSB, unsigned char LSB, unsigned char col, unsigned char row)
{

	unsigned char MSB_char1,MSB_char2,LSB_char1,LSB_char2;
	char robot_address_string[4] = {0x00,0x00,0x00,0x00};
	
	MSB_char1 = MSB/16;
	MSB_char2 = MSB-MSB_char1*16;
	LSB_char1 = LSB/16;
	LSB_char2 = LSB-LSB_char1*16;

	robot_address_string[0] = hex_to_ascii(MSB_char1);
	robot_address_string[1] = hex_to_ascii(MSB_char2);
	robot_address_string[2] = hex_to_ascii(LSB_char1);
	robot_address_string[3] = hex_to_ascii(LSB_char2);
	OSD_display_string("                         ",2,1,25);
	block_ms(20);
	OSD_display_string("Robot Address  ",col,row,15);
	block_ms(20);
	OSD_display_string(robot_address_string,col+15,row,4);
}

unsigned char hex_to_ascii(unsigned char hex_char)
{
	if(hex_char >= 10)
		hex_char+=55;
	else
		hex_char+=48;

	return hex_char;

}
