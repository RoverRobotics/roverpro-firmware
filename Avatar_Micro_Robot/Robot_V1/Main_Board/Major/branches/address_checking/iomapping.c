

#include "system.h"

void Map_Pins(void)
{
//INPUTS **********************
	AD1PCFGL = 0b0001100000000000;	//only AN11 and AN12 are analog

	//set all ports as input (they should default this way, but let's make sure)

	TRISB = 0xFFFF;
	TRISC = 0xFFFF;
	TRISD = 0xFFFF;
	TRISE = 0xFFFF;
	TRISF = 0xFFFF;
	TRISG = 0xFFFF;
	
	//since every port defaults to an input, we do not have to explicitly set the TRIS bits for inputs



//OUTPUTS *********************
// Side 1
	TRISBbits.TRISB9 = OUTPUT;		//Relay_Power_Control
	Relay_Power_Control = 0;

	TRISBbits.TRISB10 = OUTPUT;		//Relay_Resistor_Control
	Relay_Resistor_Control = 0;


	TRISBbits.TRISB6 = OUTPUT;		//PWM_F, also PWM1
	LATBbits.LATB6 = 0;

	TRISBbits.TRISB7 = OUTPUT;		//PWM_L, also PWM2							prog
	LATBbits.LATB7 = 0;	
	
	TRISBbits.TRISB8 = OUTPUT;		//PWM_R, also PWM3
	LATBbits.LATB8 = 0;

	//watchdog timer kicking output
	TRISCbits.TRISC14 = OUTPUT;

// Side 2
	
// Side 3

	TRISDbits.TRISD5 = OUTPUT;		//URX on HACTECh
	LATDbits.LATD5 = 1;			

	TRISDbits.TRISD1 = OUTPUT;		//Microphone mute pin
	ODCDbits.ODD1 = OPEN_DRAIN;
	Mic_pin = 0;

// Side 4

	TRISEbits.TRISE3 = OUTPUT;		//Amp

	ODCEbits.ODE3 = OPEN_DRAIN;		
	Amp_pin = 1;

	TRISEbits.TRISE4 = OUTPUT;		//Audio_Toggle_line
	LATEbits.LATE4 = OFF;
	ODCEbits.ODE4 = OPEN_DRAIN;
	


	TRISEbits.TRISE5 = OUTPUT;		//LED
	LATEbits.LATE5 = OFF;

	ODCBbits.ODB2 = OPEN_DRAIN;
	TRISBbits.TRISB2 = OUTPUT;	//SELECT0 for video switcher
	
	ODCBbits.ODB3 = OPEN_DRAIN;
	LATBbits.LATB2 = 0;
	TRISBbits.TRISB3 = OUTPUT;	//SELECT1 for video switcher
	
	LATBbits.LATB3 = 0;


	//CS for MAX7456 OSD
	TRISBbits.TRISB4 = OUTPUT;
	CS_LAT = 0;



//programmable pins
	//make RP24 = SDI
	//RP25 = SD0
	//RP22 = SCK (output, since always master)
	//output function number for SCK1OUT = 8, SDO1 = 7, 
	//set
	RPOR9bits.RP18R = 0x08;	//SCK
	RPOR13bits.RP27R = 0x07;	//SDO






}





