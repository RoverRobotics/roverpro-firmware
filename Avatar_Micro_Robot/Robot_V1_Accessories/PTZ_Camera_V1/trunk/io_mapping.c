#include "system.h"

void init_io(void)
{

	//set all pins to digital
	AD1PCFG = 0xFFFF;
	//AD1PCFG = 0xFFFF;

	//ensure that all digital pins are inputs
	TRISA = 0xFFFF;
	TRISB = 0xFFFF;

/*	//U1_DATA_OUT is an output
	TRISBbits.TRISB14 = 0;

	//U2_DATA_OUT is an output
	TRISBbits.TRISB12 = 0;

	//RS485_TX is an output
	TRISBbits.TRISB11 = 0;*/

	//RS485_EN is an output
	TRISBbits.TRISB9 = 0;


	//not receiving from PTZ yet, so leave this high.
	LATBbits.LATB9 = 1;

	//PTZ power control is an output
	TRISBbits.TRISB13 = 0;
	LATBbits.LATB13 = 1;



	
}
