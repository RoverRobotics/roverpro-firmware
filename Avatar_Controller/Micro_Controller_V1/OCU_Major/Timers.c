#include "include/Compiler.h"
#include "Timers.h"
#include "RS485.h"
#include "Dynamixel.h"

unsigned char event;
unsigned char Controller_State;
unsigned char osd_event;

void Init_Timer2(void)
{

//Set T2 and T3 to two individual 16-bit timers
T2CONbits.T32 = 0;

//set timer prescaler to 1:256
T2CONbits.TCKPS = 3;

//use internal clock as source
T2CONbits.TCS = 0;

//disable gated time accumulation
T2CONbits.TGATE = 0;

//timer period in PR2
//32MHz/2/256/PR2 = 
// PR2*2*256/32MHz = 
// 10ms * 32MHz /(2*256) = 625 (0x271)
//PR2 = 300;
PR2 = 800;
//PR2 = 0x0FFF;

//IEC0bits.T2IE = 1;

//turn on timer 2
//T2CONbits.TON = 1;
}


void Init_Timer3(void)
{


//set timer prescaler to 1:256
T3CONbits.TCKPS = 3;

//use internal clock as source
T3CONbits.TCS = 0;

//disable gated time accumulation
T3CONbits.TGATE = 0;

//timer period in PR2
//32MHz/2/256/PR2 = 
// PR2*2*256/32MHz = 
// 10ms * 32MHz /(2*256) = 625 (
//PR3 = 0x271;
//PR3 = 0xFFF0;
PR3 = 3000;


IEC0bits.T3IE = 1;


//turn on timer 3
T3CONbits.TON = 1;
}

void Init_Timer4(void)
{


//set timer prescaler to 1:256
T4CONbits.TCKPS = 3;

//use internal clock as source
T4CONbits.TCS = 0;

//disable gated time accumulation
T4CONbits.TGATE = 0;


//PRx = 1ms * 32MHz /(2*256) = 62.5 
//PR3 = 0x271;
//PR3 = 0xFFF0;
PR4 = 62;
//PR4 = 620;


IEC1bits.T4IE = 1;


//turn on timer 3
T4CONbits.TON = 1;
}


//This interrupt is the RS485 receive timeout
void  __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
	static char servo_index = 1;

	//Turn off timer, so we know that it
	//will start from 0 (or close to it)
	//for the next interrupt
	T2CONbits.TON = 0;

	//Disable interrupt
	IEC0bits.T2IE = 0;

	//disable interrupt flag
	IFS0bits.T2IF = 0;

	TMR2 = 0;

	//RS485_Message_Ready = 0x01;	

	if(servo_index <= NUM_DYNAMIXEL)
	{
		//Send_Servo_Message(servo_index);
	}
	else
	{
		//Since we didn't start sending a new message, this interrupt will not be enabled again
		//until T3 interrupt enables it again.  Reset the servo index for when that happens
		servo_index = 0;
	}
	servo_index++;
}

void block_ms(unsigned int ms)
{
	unsigned int i;
	unsigned int j;

	for(i=0;i<3200;i++)

	{
		for(j=0;j<ms;j++);
	}
}

//This interrupt causes the Dynamixel servos to get messages
void  __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
{

	//clear interrupt flag
	IFS0bits.T3IF = 0;

	//Since this interrupt has triggered, it is time to
	//start sending new positions to all of the servos
	//Enable T2 interrupt, which will initiate sending of the message
	IEC0bits.T2IE = 1;
	T2CONbits.TON = 1;
}



void  __attribute__((__interrupt__, auto_psv)) _T4Interrupt(void)
{

	static unsigned int elapsed_time = 0;
	static unsigned int last_turnon_message_time = 0;
	static unsigned char logo_displayed = 1;

	//clear interrupt flag
	IFS1bits.T4IF = 0;

	elapsed_time++;
	//event = NONE;

	switch (Controller_State)
	{
		case NOT_CONNECTED:

			if(elapsed_time > 2000 && logo_displayed)
			{
				osd_event = CLEAR_LOGO;
				logo_displayed = 0;
			}
			else if(elapsed_time - last_turnon_message_time > 100)
			{
		
				event = SEND_TURNON_MESSAGE;
				last_turnon_message_time = elapsed_time;
				
			}
		break;
		case CONNECTED:
			if(elapsed_time - last_turnon_message_time > 1000)
			{
				osd_event = CLEAR_SCREEN;
				IEC1bits.T4IE = 0;
			}
		break;
		default:
			//event = NONE;
		break;
	}

}
