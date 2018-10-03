#include "include/Compiler.h"
#include "Remote_Power.h"
#include "Datalink.h" //for CRC function
#include "RS485.h"
#include "Timers.h"

//0xEE 0xDD sender1 sender2 receiver1 receiver2 0xfc checksum1 checksum2

char Is_LP_Message_Valid(void);


unsigned char LP_Message[20];
//unsigned char Last_LP_Message[20];
//unsigned char LP_Buffer_Index;
//char LP_Message_Ready;
unsigned char LP_Message_Index;
unsigned char LP_Message_Sending;






void Power_Off(void)
{

	unsigned char i;

	LP_Message[7] = 0xFC;
	LP_Message[8] = 0xA3;
	LP_Message[9] = 0xD3;

	for(i=0;i<3;i++)
	{
		LP_Message_Index = 0;
		U3TXREG = LP_Message[LP_Message_Index];
		IEC5bits.U3TXIE = 1;
		LP_Message_Sending = 1;	
		while(LP_Message_Sending);
	}
}

void Remote_Power_Test(void)
{
//	unsigned int i;
	Remote_Power_Init();



	while(1)
	{




		block_ms(1000);
		Send_Turnon_Message();

		block_ms(1000);
		Power_Off();
		
		


	}


}

void Remote_Power_Init(void)
{

	//set RB3 to output (controls 2N7000 for low power datalink)
	LP_DATALINK_POWER_TRIS = 0;

	//Set AN pin to digital
	//AD1PCFGLbits.PCFG6 = 1;

	//Baud = Fcy / (16*(UxBRG +1)) = 9615
	//32e6/2/(16*(416+1)) = 2398
	U3BRG = 416;

	U3MODEbits.UARTEN = 1;	//enable UART1
	U3MODEbits.RTSMD = 1; //U1RTS pin in simplex mode -- may need to change this
	U3MODEbits.UEN = 0; //only enable Tx and Rx
	//note 1: UARTEN = 1, so I need to map to and RPn pin

	U3MODEbits.RXINV = 0; //idle high
	U3MODEbits.BRGH = 0;	//BRG 16 clocks per bit
	U3MODEbits.PDSEL = 0;	//8 bit no parity
	U3MODEbits.STSEL = 0;	//One stop bit

	U3STAbits.UTXEN = 1;	//Enable transmit
	

	//U3RX
	//RPINR17bits.U3RXR = LP_DATALINK_RX_PIN;

	//28 is function number for U3TX
	LP_DATALINK_POR = 28;


	//Set B6 to input (RX)
	//TRISBbits.TRISB6 = 1;

	//Turn on IRLZ34N to power datalink
	//XXXXXXXXXXXXXXXXXXXXXXXXXX

	//Output function number for U1TX is 3
	//RP3 mapped to U1TX
	//RPOR1 = 0x0300;

	//turn on low power datalink
	LP_DATALINK_POWER_LAT = 1;


	//IEC5bits.U3RXIE = 1;

	IEC5bits.U3TXIE = 1;



	LP_Message[0] = 0x00;
	LP_Message[1] = 0xEE;
	LP_Message[2] = 0xDD;
	LP_Message[3] = 0x11;
	LP_Message[4] = 0x12;
	LP_Message[5] = 0x13;
	LP_Message[6] = 0x14;
	LP_Message[7] = 0xFA;
	LP_Message[8] = 0xC3;
	LP_Message[9] = 0x15;
	LP_Message[10] = 0xAA;

	LP_Message_Sending = 0;

}


/*void  __attribute__((__interrupt__)) _U3RXInterrupt(void)
{
		
	//static unsigned char message_length;
	unsigned int i;
	//LP_Buffer_Full = 0x00;
	//clear interrupt flag -- not sure if this does anything yet
	IFS5bits.U3RXIF = 0;
	LP_Buffer[LP_Buffer_Index] = U3RXREG;

	
	LP_Buffer_Index++;
			
	//If first byte is not the first header byte, reset message
	if(LP_Buffer_Index == 1)
	{
		if(LP_Buffer[0] != 0xEE) LP_Buffer_Index = 0;
	}
	//If 2nd byte is not the 2nd header byte, reset message
	else if(LP_Buffer_Index == 2)
	{
		if(LP_Buffer[1] != 0xDD) 
		{
			LP_Buffer_Index = 0;
			RS485_Send_String("Transfer error \r\n",17);
		}
		//LP_Buffer_Full = 0x01;
	}
	//Message is finished
	else if(LP_Buffer_Index > 8)
	{

		//load finished message into register, to decrease the chance of the message changing while we're reading it
		for(i=0;i<9;i++)
		{
			Last_LP_Message[i] = LP_Buffer[i];
		}

		if(Is_LP_Message_Valid())
		{
			LP_Message_Ready = 0x01;
		}
		
			LP_Buffer_Index = 0;
		//LP_Buffer[9] = 0xAA;
		//LP_Buffer[10] = 0xAA;
		//RS485_Send(LP_Buffer,11);


	}

	
}*/


void Send_Turnon_Message(void)
{

	unsigned char i;


	LP_Message[7] = 0xFC;
	LP_Message[8] = 0xA3;
	LP_Message[9] = 0xD3;




	

	
	//a hack -- LP datalink receiver goes low after ~300ms in idle high mode.  Therefore, we need to 
	//send the message three times (since the message doesn't always get through if we send it once).
	//this could also be fixed by idling low.
	for(i=0;i<3;i++)
	{
		LP_Message_Index = 0;
		U3TXREG = LP_Message[LP_Message_Index];
		IEC5bits.U3TXIE = 1;
		LP_Message_Sending = 1;	
		while(LP_Message_Sending);
	}







}

//used only for testing LP datalink
void Send_Turnoff_Message(void)
{

	unsigned char i;

	LP_Message[7] = 0xFA;
	LP_Message[8] = 0xC3;
	LP_Message[9] = 0x15;


	

	
	//a hack -- LP datalink receiver goes low after ~300ms in idle high mode.  Therefore, we need to 
	//send the message three times (since the message doesn't always get through if we send it once).
	//this could also be fixed by idling low.
	for(i=0;i<3;i++)
	{
		LP_Message_Index = 0;
		U3TXREG = LP_Message[LP_Message_Index];
		IEC5bits.U3TXIE = 1;
		LP_Message_Sending = 1;	
		while(LP_Message_Sending);
	}







}

void  __attribute__((__interrupt__, auto_psv)) _U3TXInterrupt(void)
{
		
	//static unsigned char servo_index = 0;
	//clear interrupt flag
	IFS5bits.U3TXIF = 0;
	LP_Message_Index++;
	if(LP_Message_Index > 10)
	{
		//disable interupt
		IEC5bits.U3TXIE = 0;


		LP_Message_Index = 0;

		LP_Message_Sending = 0;

	}
	else
	{
		U3TXREG = LP_Message[LP_Message_Index];
	}
}
