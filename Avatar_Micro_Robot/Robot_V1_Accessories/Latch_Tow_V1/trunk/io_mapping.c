#include "system.h"

void init_io(void)
{

	//set all pins to digital
	AD1PCFG = 0xFFFF;
	//AD1PCFG = 0xFFFF;

	//ensure that all digital pins are inputs
	TRISA = 0xFFFF;
	TRISB = 0xFFFF;

	//potentiometer inputs for joint positions
	//AD1PCFGbits.PCFG0 = 0;
	//AD1PCFGbits.PCFG1 = 0;

	/*#ifdef ARM_BASE

		//MOSFET control pins are outputs
		TRISBbits.TRISB6 = 0;
		TRISBbits.TRISB7 = 0;

		//high power voltage line -- make sure high power is plugged in
		AD1PCFGbits.PCFG4 = 0;

	#endif*/
	
	//TRISBbits.TRISB8 = 0;
	//RPOR4bits.RP8R = 18; //18 is the number for OC1

	//use B5 for testing
	//TRISBbits.TRISB5 = 0;
	//LATBbits.LATB5 = 0;

	TRISBbits.TRISB5 = 0;
	TRISBbits.TRISB6 = 0;
	TRISBbits.TRISB7 = 0;

	
}
