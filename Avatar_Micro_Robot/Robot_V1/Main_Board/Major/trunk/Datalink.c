
#include "system.h"

/*****************************************************************************
 * Module Level Variables 
 *****************************************************************************/
unsigned char Datalink_Buffer[15];
unsigned char Datalink_Buffer_Index;
unsigned char Datalink_Buffer_Full;

unsigned char message_data_length;
unsigned char message_type;
unsigned char message_data[10];

// Stored Message
static unsigned char Last_Datalink_Message[15];


// Transmit matrix and status
static unsigned char TX_reg[15];
static unsigned char TX_on;

unsigned char TX_count;

/*****************************************************************************
* Function: Datalink Initialization
******************************************************************************/
void Datalink_Init(void)
{
	//for LAN480H (19200 baud)
	U1BRG = 51;

	U1MODEbits.UARTEN = 1;		//enable UART1
	U1MODEbits.RTSMD = 1; 		//U1RTS pin in simplex mode -- may need to change this
	U1MODEbits.UEN = 0; 		//only enable Tx and Rx
	U1MODEbits.RXINV = 0; 		//idle high
	U1MODEbits.BRGH = 0;		//BRG 16 clocks per bit
	U1MODEbits.PDSEL = 0;		//8 bit no parity
	U1MODEbits.STSEL = 0;		//One stop bit

	U1STAbits.UTXEN = 1;		//Enable transmit
	
	RPINR18bits.U1RXR = 25; 		//RP2 for hactec

	RPOR10bits.RP20R = U1TX_IO;	//RP4 for hactec	

	//enable sending interrupt
	IEC0bits.U1RXIE = 1;
	IEC0bits.U1TXIE = 1;
}

/*****************************************************************************
* Function: Datalink Receive Interrupt
******************************************************************************/
void  __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void)
{	
	unsigned int i;
	
	Datalink_Buffer_Full = 0x00;						// Reset module level variable
	IFS0bits.U1RXIF = 0;								// Clear interrupt flag 
	Datalink_Buffer[Datalink_Buffer_Index] = U1RXREG;	// Populate Datalink_Buffer with current index (index is module level)
	Datalink_Buffer_Index++;							// Increment the index (setup for next byte)

//	if it is the address bytes, and we're in binding mode, don't check them
	if((Datalink_Buffer_Index >= 3) && (Datalink_Buffer_Index <=6) )
	{
		if(binding_to_OCU)
			return;
	}

	switch (Datalink_Buffer_Index)
	{
		case 1:											// Init1: needs to be 0xFF
			if(Datalink_Buffer[0] != 0xFF) 
				Datalink_Buffer_Index = 0;
			break;
		case 2:											// Init2: Needs to be 0xCC
			if(Datalink_Buffer[1] != 0xCC) 				
				Datalink_Buffer_Index = 0;
			break;
		case 3:											// Controller Address1: Defined in Definitions.h

				if(Datalink_Buffer[2] != CONTROLLER1)
					Datalink_Buffer_Index = 0;

			break;
		case 4:											// Controller Address2: Defined in Definitions.h
			if(Datalink_Buffer[3] != CONTROLLER2)
				Datalink_Buffer_Index = 0;
			break;
		case 5:											// Robot Address1: Defined in Definitions.h
			if(Datalink_Buffer[4] != ROBOT1)			
				Datalink_Buffer_Index = 0;
			break;
		case 6:											// Robot Address2: Defined in Definitions.h
			if(Datalink_Buffer[5] != ROBOT2)
				Datalink_Buffer_Index = 0;
			break;
														// At this point, we know that a message is incoming
														// and it is meant for this robot, now we just let
														// the interrupt load the remaining message into the 
														// Datalink_Buffer[]
		case 7:											// Left Horizontal Value: Fills buffer
			break;
		case 8:											// Left Vertical Value: Fills buffer
			break;	
		case 9:											// Right Horizontal Value: Fills buffer
			break;		
		case 10: 										// Right Vertical Value: Fills buffer
			break;
		case 11: 										// Button1: Fills buffer
			break;							
		case DATALINK_MESSAGE_LENGTH-1:										// CRC1: Fills buffer
			break;
		case DATALINK_MESSAGE_LENGTH:										// This is the last byte of the message
			Datalink_Buffer_Full = 0x01;				// Set the Buffer to Full
			Datalink_Buffer_Index = 0;					// Reset the index
			for(i=0;i<15;i++)							// Load the Buffer into a new array
			{
				Last_Datalink_Message[i] = Datalink_Buffer[i];
			}

			break;
		default:										// if something screwy happens, just reset the message
			//Datalink_Buffer_Index = 0;
			break;
	}		
}

/*****************************************************************************
* Function: Allows pulling of last datalink message
******************************************************************************/
unsigned char Return_Last_Datalink_Message(unsigned char ldm)
{
	return Last_Datalink_Message[ldm];
}

/*****************************************************************************
* Function: Message Handler
******************************************************************************/
char Handle_Messages(void)
{
	if(Datalink_Buffer_Full){						// If there is no new message, return 0
		Datalink_Buffer_Full = 0x00;					// Reset the flag
		if(Is_Message_Valid())				 			// If the message is invalid, return 0
			return 0x01;
	}
	return 0x00;									// If the message is new and valid, return 1
}

/*****************************************************************************
* Function: Check message validity
******************************************************************************/
//checks if the message is valid.
char Is_Message_Valid(void)
{
	unsigned int CRC;
	unsigned char incoming_CRC_LSB;
	unsigned char incoming_CRC_MSB;
	unsigned char checksum_string[4];

	unsigned int crc=0;
	unsigned char i;

	unsigned char ocu_address_1, ocu_address_2, robot_address_1, robot_address_2;

	for(i=2;i<(DATALINK_MESSAGE_LENGTH-2);i++)
	{
		crc = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= Last_Datalink_Message[i];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= (crc << 8) << 4;
		crc ^= ((crc & 0xff) << 4) << 1;
		crc &= 0xFFFF;
	}
	CRC = crc;

	incoming_CRC_MSB = Last_Datalink_Message[DATALINK_MESSAGE_LENGTH-2];
	incoming_CRC_LSB = Last_Datalink_Message[DATALINK_MESSAGE_LENGTH-1];

	checksum_string[0] = incoming_CRC_MSB;
	checksum_string[1] = incoming_CRC_LSB;
	checksum_string[3] = CRC>>8;
	checksum_string[4] = CRC&0xFF;
	
	ocu_address_1 = Last_Datalink_Message[2];
	ocu_address_2 = Last_Datalink_Message[3];
	robot_address_1 = Last_Datalink_Message[4];
	robot_address_2 = Last_Datalink_Message[5];


	
	//check that addresses match
	if(binding_to_OCU == 0)
	{
		if( (ocu_address_1 != CONTROLLER1) || (ocu_address_2 != CONTROLLER2) || (robot_address_1  != ROBOT1) || (robot_address_2 != ROBOT2))
		{
			return 0x00;
		}
	}


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


unsigned char Set_Last_Datalink_Message(unsigned char byte, unsigned char index)
{

	Last_Datalink_Message[index] = byte;

	return 0x01;
}

/*****************************************************************************
* Function: Calculate valid crc value based on data
******************************************************************************/
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


//Datalink receive interrupt -- called every time that a new byte has filled U1RXREG
void  __attribute__((interrupt, auto_psv)) _U1TXInterrupt(void)
{
	IFS0bits.U1TXIF = 0;								// Clear interrupt flag 
	
	if (TX_on == ON){
		TX_count ++;
		U1TXREG = TX_reg[TX_count];
	}

	if (TX_count > 11){
		TX_count = 0;
		TX_on = OFF;
	}
}
