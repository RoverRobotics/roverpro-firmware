#include "include/Compiler.h"
#include "HPDatalink.h"
#include "Definitions.h"
#include "Servo_Messaging.h"

unsigned char Datalink_Buffer[20];
unsigned char Last_Datalink_Message[20];
unsigned char Datalink_Buffer_Index;
unsigned char Datalink_Buffer_Full;

static unsigned char U1TX_Message;
static unsigned char U1TX_Counter;
unsigned int FL;


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
	
	//RP4 mapped to U1RX
	RPINR18bits.U1RXR = 4;

	//Output function number for U1TX is 3
	//RP2 mapped to U1TX
	RPOR1bits.RP2R = U1TX_IO;

	//enable sending interrupt
	IEC0bits.U1RXIE = 1;
//	IEC0bits.U1TXIE = 1;
}


//Datalink receive interrupt -- called every time that a new byte has filled U1RXREG
void  __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void)
{	
	static unsigned char message_length;
	unsigned int i;
	
	Datalink_Buffer_Full = 0x00;						// Reset module level variable
	IFS0bits.U1RXIF = 0;								// Clear interrupt flag 
	Datalink_Buffer[Datalink_Buffer_Index] = U1RXREG;	// Populate Datalink_Buffer with current index (index is module level)
	Datalink_Buffer_Index++;							// Increment the index (setup for next byte)

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
		case 12:										// CRC1: Fills buffer
			break;
		case 13:										// This is the last byte of the message
			Datalink_Buffer_Full = 0x01;				// Set the Buffer to Full
			Datalink_Buffer_Index = 0;					// Reset the index
			for(i=0;i<message_length+10;i++)			// Load the Buffer into a new array
			{
				Last_Datalink_Message[i] = Datalink_Buffer[i];
			}
			break;
		default:									// if something screwy happens, just reset the message
			Datalink_Buffer_Index = 0;
			break;
	}		
}