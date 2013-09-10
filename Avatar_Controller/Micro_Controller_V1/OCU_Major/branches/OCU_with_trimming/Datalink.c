#include "include/Compiler.h"
#include "Datalink.h"

#include "RS485.h"
#include "Messaging.h"
#include "SPI.h"


unsigned char* char_to_hex(unsigned char* unconverted_array,unsigned char *new_array,unsigned char length);

unsigned char Datalink_Buffer[20];
unsigned char Last_Datalink_Message[20];

unsigned char Datalink_Buffer_Index;
unsigned char Datalink_Buffer_Full;
unsigned char Datalink_Send_Buffer[20];
unsigned char Datalink_Send_Index;
unsigned char Robot_Powered_On;




void Datalink_Init(void)
{

	//if BRGH = 0:
	//UxBRG = Fcy/16/Baud - 1 = 32MHz/2/16/9600 - 1 = 103
	//UxBRG = Fcy/16/Baud - 1 = 32MHz/2/16/38400 - 1 = 25
	//Baud = Fcy / (16*(UxBRG +1)) = 32e6/(16*(234+1)) = 9575.468
	//Baud = Fcy / (16*(UxBRG +1)) = 32e6/2/(16*(25+1)) = 38461.5
	//U1BRG = 25;
	//for LAN480H (19200 baud)
	U1BRG = 51;

	U1MODEbits.UARTEN = 1;		//enable UART1
	U1MODEbits.RTSMD = 1; 		//U1RTS pin in simplex mode -- may need to change this
	U1MODEbits.UEN = 0; 		//only enable Tx and Rx
	//note 1: UARTEN = 1, so I need to map to and RPn pin

	U1MODEbits.RXINV = 0; 		//idle high
	U1MODEbits.BRGH = 0;		//BRG 16 clocks per bit
	U1MODEbits.PDSEL = 0;		//8 bit no parity
	U1MODEbits.STSEL = 0;		//One stop bit

	U1STAbits.UTXEN = 1;		//Enable transmit
	
	//RP21 mapped to U1RX
	RPINR18bits.U1RXR = Datalink_Rx_Port;


	//Output function number for U1TX is 3
	//RP26 mapped to U1TX
	Datalink_Tx_POR = 3;



	//enable sending interrupt
	IEC0bits.U1RXIE = 1;

	Robot_Powered_On = 0x00;
}


void Datalink_Message_Send(void)
{


	
	Datalink_Send_Index = 0;

	

	U1TXREG = Datalink_Send_Buffer[0];

	IEC0bits.U1TXIE = 1;


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






//populates a vector with ascii codes of the hex bytes and a space (i.e. 0x00 adds 0x30 0x30 0x20 to the new array
unsigned char* char_to_hex(unsigned char* unconverted_array,unsigned char *new_array,unsigned char length)
{
	unsigned char i;
	unsigned char new_index;
	new_index = 0;
	for(i=0;i<length;i++)
	{
			//Add 0x30 to the MSN, if 0-9
			if((unconverted_array[i]>>4) < 0x0a)
			{
				new_array[new_index] = (unconverted_array[i]>>4)+0x30;
			}
			//Add 0x37 to the MSN, if A-F
			else
			{
				new_array[new_index] = (unconverted_array[i]>>4)+0x37;
			}

			new_index++;

			//Add 0x30 to the LSN, if 0-9
			if((unconverted_array[i]&0x0F) < 0x0a)
			{				
				new_array[new_index] = (unconverted_array[i]&0x0F)+0x30;
			}

			//Add 0x37 to the LSN, if A-F
			else
			{
				new_array[new_index] = (unconverted_array[i]&0x0F)+0x37;
			}
			new_index++;
			new_array[new_index] = ' ';
			new_index++;
	}	
		//Add a newline and return to the array
		new_array[new_index] = '\r';
		new_array[new_index+1] = '\n';
		return new_array;
}


void Datalink_loop(void)
{
}

void Datalink_Test(void)
{

	Datalink_Init();
//	RS485_Init();
	while(1)
	{
		if(Handle_Messages())
		{
			//RS485_Send(Last_Datalink_Message,17);
			//RS485_Send_String("Message Received \r\n",19);
		}
	}
}

//Datalink receive interrupt -- called every time that a new byte has filled U1RXREG
void  __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void)
{
		
	static unsigned char message_length;
	unsigned int i;
	Datalink_Buffer_Full = 0x00;
	//clear interrupt flag -- not sure if this does anything yet
	IFS0bits.U1RXIF = 0;
	Datalink_Buffer[Datalink_Buffer_Index] = U1RXREG;

	
	Datalink_Buffer_Index++;
			
	//If first byte is not the first header byte, reset message
	if(Datalink_Buffer_Index == 1)
	{
		if(Datalink_Buffer[0] != 0xFF) Datalink_Buffer_Index = 0;
	}
	//If 2nd byte is not the 2nd header byte, reset message
	else if(Datalink_Buffer_Index == 2)
	{
		if(Datalink_Buffer[1] != 0xCC) Datalink_Buffer_Index = 0;
	}
	//Get 8th byte for message length
	else if(Datalink_Buffer_Index == 8)
	{
		message_length = Datalink_Buffer[7];
	}

	//If message is finished (based on length given in 8th byte, set buffer_full flag
	//and reset buffer_index
	if(Datalink_Buffer_Index > message_length+9)
	{
		Datalink_Buffer_Full = 0x01;
		Datalink_Buffer_Index = 0;

		//load finished message into register, to decrease the chance of the message changing while we're reading it
		for(i=0;i<message_length+10;i++)
		{
			Last_Datalink_Message[i] = Datalink_Buffer[i];
		}
	}
}



unsigned char Is_Robot_Powered_On()
{

	return Robot_Powered_On;
}

void set_9xtend_channel(const ROM char *channel)
{
/*	unsigned char i;
	const ROM char *at_message;
	at_message = "ATDT,WR,CN\r";
	//wait 1 second
	block_ms(1000);
	//disable interrupts, for now
	IEC0bits.U1RXIE = 0;
	IEC0bits.U1TXIE = 0;
	//send +++
	for(i=0;i<3;i++)
	{
		U1TXREG = 0x2b;
		block_ms(100);
	}

	block_ms(1000);

	//need to send ATDT[address],WR,CN,<cr>
	for(i=0;i<4;i++)
	{
		U1TXREG = at_message[i];
		while(U1STAbits.UTXBF);
	}
	for(i=0;i<4;i++)
	{
		U1TXREG = channel[i];
		while(U1STAbits.UTXBF);
	}
	for(i=4;i<7;i++)
	{
		U1TXREG = at_message[i];
		while(U1STAbits.UTXBF);
	}
	block_ms(100);
		for(i=7;i<11;i++)
	{
		U1TXREG = at_message[i];
		while(U1STAbits.UTXBF);
	}
	OSD_display_string("Finished setting channel to",2,1,27);
	block_ms(100);
	OSD_display_string(channel,3,2,4);
	while(1);*/
}

void  __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void)
{

	//clear interrupt flag
	IFS0bits.U1TXIF = 0;
	Datalink_Send_Index++;
	if(Datalink_Send_Index >= 14)
	{
		//disable interupt
		IEC0bits.U1TXIE = 0;


		Datalink_Send_Index = 0;
	/*	for(i=0;i<20;i++)
		{
			Datalink_Send_Buffer[i] = Datalink_Next_Message[i];
		}*/


	}
	else
	{
		U1TXREG = Datalink_Send_Buffer[Datalink_Send_Index];
		//U1TXREG = 0xAA;
	}
}
