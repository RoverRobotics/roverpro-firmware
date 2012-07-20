#include "include/Compiler.h"
#include "Messaging.h"
#include "Definitions.h"
#include "HPDatalink.h"

unsigned char message_data_length;
unsigned char message_type;
unsigned char message_data[10];

char Handle_Messages(void)
{
	if(Datalink_Buffer_Full){						// If there is no new message, return 0
		Datalink_Buffer_Full = 0x00;					// Reset the flag
		if(Is_Message_Valid())				 			// If the message is invalid, return 0
			return 0x01;
	}
	return 0x00;									// If the message is new and valid, return 1
}


//checks if the message is valid.  Will eventually check sender and receiver, but now
//just checks CRC (since two start bytes have been checked already)
char Is_Message_Valid(void)
{
	unsigned int CRC;
	unsigned char incoming_CRC_LSB;
	unsigned char incoming_CRC_MSB;
	unsigned char checksum_string[4];

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
		return 0x01;
	}
	//no CRC match; message is invalid
	else
	{
		return 0x00;
	}
}

unsigned int return_crc(unsigned char* data, unsigned char length)
{
	static unsigned int crc;
	unsigned char i;
	crc = 0;

	for(i=0;i<length;i++)
	{
		crc = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= data[i+2];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= (crc << 8) << 4;
		crc ^= ((crc & 0xff) << 4) << 1;
		crc &= 0xFFFF;
	}
	return crc; 
}
