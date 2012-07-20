
#include "system.h"

/*****************************************************************************
 * Module Level Variables 
 *****************************************************************************/

//static unsigned char Last_Audio_State;

/*****************************************************************************
* Function: Audio Initialization
******************************************************************************/
/*void AudioInit(void)		
{
	//Audio Chips: Pin directions and initializations

	//Port Setup for WAP200H Audio Communication Module
	//RD1 = output, Power Down Pin, initialize high
	//RD3/RP22 = input , Audio Select 0 / TXD on audio module, RXD on PIC, initialize high
	//RD4/RP25 = output, Audio Select 1 / RXD on audio module, TXD on PIC, initialize high

	//map TX to RP25
	RPOR12bits.RP25R 	= U4TX_IO; 	// 30 is the value for Uart4 TX	

	//map RX to RP22
	RPINR27bits.U4RXR 	= 22;		// sets up Uart4's recieve register to RP24

	U4MODEbits.RTSMD  	= 1; 	//U4RTS pin in simplex mode -- may need to change this
	U4MODEbits.UEN    	= 0; 	//only enable Tx and Rx
	U4MODEbits.RXINV 	= 0; 	//idle high
	U4MODEbits.BRGH 	= 0;	//BRG 16 clocks per bit
	U4MODEbits.PDSEL 	= 0;	//8 bit no parity
	U4MODEbits.STSEL 	= 0;	//One stop bit

	//UxBRG = Fcy/16/Baud - 1 = 32MHz/2/16/9600 - 1 = 103.2
	//32MHz/2/16/(103+1) = 9615.38
	U4BRG = 103;
	U4MODEbits.UARTEN 	= 1;	//enable UART4
	U4STAbits.UTXEN 	= 1;	//Enable transmit
	IEC5bits.U4TXIE 	= 0;	// disable interrupt

	Audio_Message[1] = 0xFE;	// Start storing a message
	Audio_Message[2] = 0xFE;	// 0xFE,0xFE,0xFE,0xFE is the header
	Audio_Message[3] = 0xFE;
	Audio_Message[4] = 0xFE;
	Audio_Message[5] = 0x42;	// 42 --> Status Change
	Audio_Message[6] = 0x03;	// 03 --> Receive Mode

	Audio_pin = ON; 			// Transmitter Power on
	Amp_pin = OFF;				// Amplifier off
}*/

/*void Audio(unsigned int Command)
{
    switch (Command)                  
    {
	    case PD: 							// PD - Power Down Everything
			Audio_pin = OFF;
			Amp_pin = OFF;             
	        break;
	    case RECEIVE: 						// RECEIVE - Audio Module takes in signal, Speaker ON
			U4TXREG = 0xFE;
			Audio_pin = ON;
			Audio_Message[6] = 0x03;
			message_index = 1;
			IEC5bits.U4TXIE = 1;
			U4TXREG = Audio_Message[1];
			Amp_pin = ON;
			Last_Audio_Mode = RECEIVE;
	        break;
	    case TRANSMIT: 						// TRANSMIT - Audio Module outputs signal, Speaker OFF
			U4TXREG = 0xFE;
			Audio_pin = ON;	
			Amp_pin = OFF;
			Audio_Message[6] = 0x02;
			message_index = 1;
			IEC5bits.U4TXIE = 1;
			U4TXREG = Audio_Message[1];
			
			Last_Audio_Mode = TRANSMIT;
	        break;
	    default: 
			break;
    }
}*/

/*****************************************************************************
* Interrupt: Uart 4: Transmit serial audio command interrupt
******************************************************************************/
/*void  __attribute__((interrupt, auto_psv)) _U4TXInterrupt(void)
{	
	IFS5bits.U4TXIF = 0;	//clear interrupt flag
	message_index++;

	switch(message_index){
		case 1:
			U4TXREG = 0xFE;
//			U1TXREG = 0xFE;	
			break;
		case 2:
			U4TXREG = 0xFE;
//			U1TXREG = 0xFE;	
			break;
		case 3:
			U4TXREG = 0xFE;
//			U1TXREG = 0xFE;	
			break;
		case 4:
			U4TXREG = 0xFE;
//			U1TXREG = 0xFE;	
			break;
		case 5:
			U4TXREG = 0x42;
//			U1TXREG = 0x42;	
			break;
		case 6:
			if (Last_Audio_Mode == TRANSMIT){
				U4TXREG = 0x02;
//				U1TXREG = 0x02;
			}
			else{
				U4TXREG = 0x03;
//				U1TXREG = 0x03;
			}			
			break;
		default:
			IEC5bits.U4TXIE = 0;
			message_index = 1;
			break;
	}//end switch
}//end interrupt
*/
