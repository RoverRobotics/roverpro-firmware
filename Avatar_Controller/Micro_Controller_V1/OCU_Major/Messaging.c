#include "include/Compiler.h"
#include "Messaging.h"
#include "Datalink.h"
#include "RS485.h"
#include "Dynamixel.h"

unsigned char message_data_length;
unsigned char message_type;
unsigned char message_data[10];

void Handle_Inputs()
{





}

char Handle_Messages(void)
{

	unsigned char i;

	//return false if there is no new message, or if the new message is invalid
	if(!Datalink_Buffer_Full) return 0x00;

	Datalink_Buffer_Full = 0x00;

	if(!Is_Message_Valid()) 
	{
		//reset Datalink_Buffer_Full, so it won't call the validation function every time the code runs
		//Datalink_Buffer_Full = 0x00;
		return 0x00;
	}


	//Handle_Inputs();
	switch(Last_Datalink_Message[6])
	{

		case 0x0A:
			Handle_Inputs();
		break;
		case 0xA0:
			//RS485_Send_String("0xA0 \r\n",7);
			Handle_Inputs();
		break;
		case 0xB7:
			Robot_Powered_On = 1;
		break;
		//invalid message byte
		default:
			//RS485_Send_String("default \r\n",10);
			return 0x00;
		break;
	}
	message_type = Last_Datalink_Message[6];
	message_data_length = Last_Datalink_Message[7];
	for(i=0;i<message_data_length;i++)
	{
		message_data[i] = Last_Datalink_Message[i+8];

	}





		return 0x01;

}


//checks if the message is valid.  Will eventually check sender and receiver, but now
//just checks CRC (since two start bytes have been checked already)
char Is_Message_Valid(void)
{


	
	unsigned int CRC;
	unsigned char incoming_CRC_LSB;
	unsigned char incoming_CRC_MSB;

	unsigned char checksum_string[4];



//	unsigned char print_hex_values[40];

	CRC = return_crc(Last_Datalink_Message,Last_Datalink_Message[7]+6);


	incoming_CRC_MSB = Last_Datalink_Message[Last_Datalink_Message[7]+8];
	incoming_CRC_LSB = Last_Datalink_Message[Last_Datalink_Message[7]+9];

	checksum_string[0] = incoming_CRC_MSB;
	checksum_string[1] = incoming_CRC_LSB;
	checksum_string[3] = CRC>>8;
	checksum_string[4] = CRC&0xFF;
	
	//check if the CRC matches
	if(((CRC>>8) == incoming_CRC_MSB) && ((CRC&0xFF) == incoming_CRC_LSB))
	{
		//RS485_Send_String("Good message \r\n",15);

		return 0x01;

	}
	//no CRC match; message is invalid
	else
	{
		//RS485_Send_String("Bad message \r\n",16);
		//RS485_Send(checksum_string,4);
		//RS485_Send(Last_Datalink_Message,16);
		return 0x00;
	}
	

}
