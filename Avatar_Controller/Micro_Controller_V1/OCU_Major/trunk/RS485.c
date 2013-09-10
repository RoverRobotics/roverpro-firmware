#include "include/Compiler.h"
#include "RS485.h"

unsigned char RS485_Message[40];

unsigned char RS485_Message_Length;
unsigned char RS485_Message_Index;

unsigned char RS485_RX_Message[10];
unsigned char RS485_RX_Length;
unsigned char RS485_Message_Ready;

void RS485_test(void)
{
	unsigned int i;
	unsigned int j;
	unsigned char message[8];

	message[0] = 'h';
	message[1] = 'e';
	message[2] = 'l';
	message[3] = 'l';
	message[4] = 'o';
	message[5] = 'w';
	message[6] = 'o';
	message[7] = 'r';
	while(1)
	{
		RS485_Send(message,8);
		for(i=0;i<3200;i++)
		{
			for(j=0;j<1000;j++);
		}
	}
}

void RS485_Init(void)
{
	//UxBRG = Fcy/16/Baud - 1 = 32MHz/2/16/57142 - 1 = 16.5
	//32MHz/2/16/(17+1) = 55,555.56
	U2BRG = 51;

	U2MODEbits.UARTEN = 1;	//enable UART2
	U2MODEbits.RTSMD = 1; //U2RTS pin in simplex mode -- may need to change this
	U2MODEbits.UEN = 0; //only enable Tx and Rx

	U2MODEbits.RXINV = 0; //idle high
	U2MODEbits.BRGH = 0;	//BRG 16 clocks per bit
	U2MODEbits.PDSEL = 0;	//8 bit no parity
	U2MODEbits.STSEL = 0;	//One stop bit

	U2STAbits.UTXEN = 1;	//Enable transmit

	//map TX to RP23
	RPOR11bits.RP23R = 5;

	//map RP22 to RX
	RPINR19bits.U2RXR = 22;

	//no need for a pullup and open drain configuration,
	//since the MAX487 input high voltage is 2.0 V

	TRISDbits.TRISD4 = 0;

	//enable interrupt
	

	//set transmit enable/receive disable high
	LATDbits.LATD4 = 1;
}


void RS485_Send_Byte(char byte)
{
	PORTDbits.RD4 = 1;
	U2TXREG = byte;
	Nop();
	//blocking code
//	while(!U2STAbits.TRMT);
}


void RS485_Send(unsigned char* message,unsigned char length)
{
	unsigned char i;
	//for(i=0;i<10;i++) Servo_Message[i] = i;
	RS485_Message_Index = 0;
	//RS485_Message_Length = 10;
	RS485_Message_Length  = length;

	for(i=0;i<length;i++)
	{
		RS485_Message[i] = message[i];
	}

	U2TXREG = RS485_Message[RS485_Message_Index];

	//enable interrupt
	IEC1bits.U2TXIE = 1;
	IEC1bits.U2RXIE = 1;
}


void RS485_Send_String(const ROM char *data,unsigned char length)
{
	unsigned char i;
	for(i=0;i<length;i++)
	{
		RS485_Message[i]=data[i];

	}
	RS485_Message_Index = 0;
	RS485_Message_Length  = length;

	U2TXREG = RS485_Message[RS485_Message_Index];

	//enable interrupt
	IEC1bits.U2TXIE = 1;
}

void  __attribute__((__interrupt__)) _U2TXInterrupt(void)
{
		
	static unsigned char servo_index = 0;
	//clear interrupt flag
	IFS1bits.U2TXIF = 0;
	RS485_Message_Index++;
	if(RS485_Message_Index >= RS485_Message_Length)
	{
		//disable interupt
		IEC1bits.U2TXIE = 0;

		//leaving this line in causes the servo not to move.
		//I'm not sure why, since it should only be changing the bit after
		//the message is done transmitting
		//PORTDbits.RD4 = 0;

		//Enable interrupt and turn timer 2 on
		IEC0bits.T2IE = 1;
		T2CONbits.TON = 1;	
	}
	else
	{
		U2TXREG = RS485_Message[RS485_Message_Index];
	}
}



void  __attribute__((__interrupt__)) _U2RXInterrupt(void)
{
	//need some kind of timing interrupt, so I don't wait for a response
	//for once servo forever, in case it's not responding.  For testing,
	//I can just use a timer interrupt to start transmits.

	static unsigned char message_index = 0;
	//clear interrupt flag
	IFS1bits.U2RXIF = 0;

	//If interrupt is called for the first time after a new message has been received,
	//clear this flag, and reset the length counter
	
	RS485_RX_Message[message_index] = U2RXREG;

	
	message_index++;

	if(message_index >= 6)
	{
		message_index = 0;
		RS485_Message_Ready = 0x01;
		RS485_RX_Length = message_index;
		PORTDbits.RD4 = 1;
		//set to send again
	}
}