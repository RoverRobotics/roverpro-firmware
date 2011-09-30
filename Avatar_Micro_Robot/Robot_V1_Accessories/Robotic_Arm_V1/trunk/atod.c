#include "system.h"


void init_atod(void)
{


	AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;
	AD1CHS = 0x0000;


	

	AD1CON1bits.SSRC = 0b111;	//auto-convert

	AD1CON3 = 0x1F02;

	AD1CSSL = 0;
	AD1CON1bits.ADON = 1;



}

unsigned int get_ad_value(unsigned char port)
{
	
	AD1CON1bits.ADON = 0;

	AD1CHS = port;
	

	AD1CON1bits.ADON = 1;
	Nop();
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE);
	return  ADC1BUF0;

	



}
