#include "include/Compiler.h"
#include "Audio.h"

unsigned char Audio_Message[6];

void Audio_Test(void)
{
	Init_Audio();
	while(1)
	{
	}
}

	
void Init_Audio(void)		
{
	//Audio Chips: Pin directions and initializations


	TRISEbits.TRISE6 = OUTPUT;	//PTT
	ODCEbits.ODE6 = 1;

	TRISCbits.TRISC14 = OUTPUT;	//mic disable
	ODCCbits.ODC14 = 1;

	TRISEbits.TRISE5 = OUTPUT;	//mic disable
	ODCEbits.ODE5 = 1;



	Audio_Mute();
	Audio_Receive();




	
}


void Audio_Transmit(void)
{


	//set audio link to transmit, enable microphone
	LATEbits.LATE6 = 0;
	LATCbits.LATC14 = 1;

	

}

void Audio_Receive(void)
{

	//set audio link to transmit, enable microphone
	LATEbits.LATE6 = 1;
	LATCbits.LATC14 = 0;
}

void Audio_Mute(void)
{
	VOLUME = 0;

	
}
void Audio_Unmute(void)
{

	VOLUME = 1;

}

void  __attribute__((__interrupt__, auto_psv)) _U4TXInterrupt(void)
{
		
	static unsigned char message_index = 0;
	//clear interrupt flag
	IFS5bits.U4TXIF = 0;
	message_index++;
	if(message_index > 5)
	{
		//disable interupt
		IEC5bits.U4TXIE = 0;

		message_index = 0;

	}
	else
	{
		U4TXREG = Audio_Message[message_index];
		
	}
}
