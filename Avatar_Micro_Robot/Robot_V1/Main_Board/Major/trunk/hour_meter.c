#include "system.h"


unsigned int read_eeprom(unsigned char address);
void write_eeprom(unsigned char address, unsigned int data);
void lock_eeprom(void);
void unlock_eeprom(void);
void retrieve_last_good_counter(void);
//void reset_hour_meter(unsigned int value_to_set);

//default is 278528 hours
unsigned int last_good_counter_H = 0x00FF;
unsigned int last_good_counter_L = 0;

unsigned int last_good_break_in_counter_H = 0x00FF;
unsigned int last_good_break_in_counter_L = 0;

#define EWDS 0x100
#define EWEN 0x130
#define READ_EEPROM 0x180
#define WRITE_EEPROM 0x140
#define EEPROM_CS LATBbits.LATB13


void init_hour_meter_spi(void)
{




	//RP10 is SPI2 CLK (output)
	RPOR5bits.RP10R = 11; //SCK2OUT
	//RP29 is SPI2 DI (into EEPROM) (output)
	RPOR14bits.RP29R = 10; //SDO2
	//RP14 is SPI2 DO (out of EEPROM) (input)
	RPINR22bits.SDI2R = 14;

	//Can also assign SS2out, but I've never done that before
	//RB13 is SPI2 CS (output)
	TRISBbits.TRISB13 = OUTPUT;

	

	SPI2CON1 = 0;
	SPI2CON2 = 0;


	SPI2CON1bits.MSTEN = 1;

	//use 2 byte communication
	SPI2CON1bits.MODE16 = 1;



	SPI2STATbits.SPIROV = 0;	

	//rising edge clocks in data, so data should change on falling edge
	SPI2CON1bits.CKE = 1;

	SPI2CON1bits.SMP = 1;

	//clear the interrupt flag
	//IFS2bits.SPI2IF = 0;




	//Enable SPI subsystem
	SPI2STATbits.SPIEN = 1;
	EEPROM_CS = 0;

	//reset_hour_meter(60*40);
	//while(1);
	//block_ms(20);

	retrieve_last_good_counter();

}



void display_break_in_hours(void)
{

	unsigned int hours;
	unsigned int hours_tenths;
	unsigned char i;
	unsigned char hour_chars[11];

	unsigned char first_nonzero_digit = 0;

	//only calculate characters and update display every sixth time this function runs.  Otherwise, the display doesn't change.
	/*display_counter++;
	if(display_counter < 6)
		return;
	display_counter = 0;*/

	if(last_good_break_in_counter_H < 60)
	{
		hours = last_good_break_in_counter_L/60+last_good_break_in_counter_H*1092.25;
		hours_tenths = (last_good_break_in_counter_L - hours*60)/6;
	}
	else
	{
		hours = 0xFFFF;
		hours_tenths = 0x09;
	}

	OSD_display_string("Break In",1,11,8);
	block_ms(10);
	hour_chars[0] = (unsigned char)(hours/10000);
	hour_chars[1] = (unsigned char)(hours/1000 - hour_chars[0]*10);
	hour_chars[2] = (unsigned char)(hours/100 - hour_chars[0]*100 - hour_chars[1]*10);
	hour_chars[3] = (unsigned char)(hours/10 - hour_chars[0]*1000 - hour_chars[1]*100 - hour_chars[2]*10);
	hour_chars[4] = (unsigned char)(hours - hour_chars[0]*10000 - hour_chars[1]*1000 - hour_chars[2]*100 - hour_chars[3]*10);
	hour_chars[5] = 0x41;
	hour_chars[6] = (unsigned char)(hours_tenths);
	/*hour_chars[0] = 0x00;
	hour_chars[1] = 0x00;
	hour_chars[2] = 0x00;
	hour_chars[3] = 0x00;
	hour_chars[4] = (unsigned char)(last_good_break_in_counter_L/100);
	hour_chars[5] = (unsigned char)(last_good_break_in_counter_L/10-hour_chars[4]*10);
	hour_chars[6] = (unsigned char)(last_good_break_in_counter_L-hour_chars[4]*100-hour_chars[5]*10);*/
	for(i=0;i<7;i++)
	{
		if((hour_chars[i] == 0) && (first_nonzero_digit==0))
			hour_chars[i] = 0x00;
		else if(hour_chars[i] == 0)
			hour_chars[i] = 0x0a;
		else
			first_nonzero_digit = 1;
	}


	/*hour_chars[7] = 0x00;
	hour_chars[8] = 0x17;
	hour_chars[9] = 0x13;
	hour_chars[10] = 0x18;*/
	
	hour_chars[7] = 0x00;
	hour_chars[8] = 0x12;
	hour_chars[9] = 0x1C;
	hour_chars[10] = 0x1D;

	OSD_display_chars(hour_chars,1,12,11);
	



}


void test_eeprom(void)
{
//	unsigned int read_data;
	unsigned char stripped_data;
	unsigned char test2[4];
	unsigned char stripped_data2;
/*	test2[0] = 0x4d;
	test2[2] = 0x4d;
	test2[3] = 0x4e;
	AD1PCFGL = 0xFFFF;
//	
//	unsigned char stripped_data;
	unlock_eeprom();
	block_ms(50);
	write_eeprom(2,0x120c);
	block_ms(50);
	read_data = read_eeprom(2);
	stripped_data = (unsigned char)read_data&0xFF;
	stripped_data2 = (unsigned char)(read_data>>8);
	if(read_data == 0xFFFF)
		stripped_data = 0x07;
	if(read_data == 0x0000)
		stripped_data = 0x08;
	test2[2] = stripped_data;
	test2[1] = stripped_data2;
	zero_counter();
	OSD_display_chars(test2,4,4,4);*/
	//zero_counter();
	//while(1);
	retrieve_last_good_counter();
	while(1)
	{
		test2[0] = 0x02;
		stripped_data = (unsigned char)(last_good_counter_L >> 8);
		stripped_data2 = (unsigned char)(last_good_counter_L) & 0x00FF;
		if(stripped_data == 0xff)
			stripped_data = 0x03;
		if(stripped_data2 == 0xff)
			stripped_data2 = 0x03;
		test2[1] = (unsigned char)stripped_data;
		//test2[1] = 0x02;
		test2[2] = (unsigned char)stripped_data2;
		//test2[2] = 0x02;
		test2[3] = 0x02;
		//OSD_display_chars(test2,4,4,4);	
		display_hours();
		increment_minute_counter(1);
		//block_ms(100);

	}
	


}



void reset_hour_meter(unsigned int value_to_set)
{
	unsigned char i;
	unlock_eeprom();
	block_ms(1);
	for(i=0;i<6;i++)
	{
		//odd, so put in value
		if(i&0x01)
			write_eeprom(i,value_to_set);
		//even, so put in 0
		else
			write_eeprom(i,0x0000);
		block_ms(1);
	}

	lock_eeprom();
	block_ms(1);
	retrieve_last_good_counter();

}

void display_hours(void)
{

	unsigned int hours;
	unsigned int hours_tenths;
	unsigned char i;
	unsigned char hour_chars[11];
	static char display_counter = 99;
	unsigned char first_nonzero_digit = 0;

	//only calculate characters and update display every sixth time this function runs.  Otherwise, the display doesn't change.
	display_counter++;
	if(display_counter < 6)
		return;
	display_counter = 0;

	if(last_good_counter_H < 60)
	{
		hours = last_good_counter_L/60+last_good_counter_H*1092.25;
		hours_tenths = (last_good_counter_L - hours*60)/6;
	}
	else
	{
		hours = 0xFFFF;
		hours_tenths = 0x09;
	}


	hour_chars[0] = (unsigned char)(hours/10000);
	hour_chars[1] = (unsigned char)(hours/1000 - hour_chars[0]*10);
	hour_chars[2] = (unsigned char)(hours/100 - hour_chars[0]*100 - hour_chars[1]*10);
	hour_chars[3] = (unsigned char)(hours/10 - hour_chars[0]*1000 - hour_chars[1]*100 - hour_chars[2]*10);
	hour_chars[4] = (unsigned char)(hours - hour_chars[0]*10000 - hour_chars[1]*1000 - hour_chars[2]*100 - hour_chars[3]*10);
	hour_chars[5] = 0x41;
	hour_chars[6] = (unsigned char)(hours_tenths);

	for(i=0;i<7;i++)
	{
		if((hour_chars[i] == 0) && (first_nonzero_digit==0))
			hour_chars[i] = 0x00;
		else if(hour_chars[i] == 0)
			hour_chars[i] = 0x0a;
		else
			first_nonzero_digit = 1;
	}


	hour_chars[7] = 0x00;
	hour_chars[8] = 0x12;
	hour_chars[9] = 0x1C;
	hour_chars[10] = 0x1D;
	
	OSD_display_chars(hour_chars,1,12,11);
	



}

void retrieve_last_good_counter(void)
{
	unsigned int counter_H[3];
	unsigned int counter_L[3];
	unsigned char i;
	unsigned char j;
	//read_eeprom
	counter_H[0] = read_eeprom(0);
	counter_L[0] = read_eeprom(1);
	counter_H[1] = read_eeprom(2);
	counter_L[1] = read_eeprom(3);
	counter_H[2] = read_eeprom(4);
	counter_L[2] = read_eeprom(5);

	//last_good_counter_H = counter_H[0];
	//last_good_counter_L = counter_L[0];

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			if((counter_H[i] == counter_H[j]) && (counter_L[i] == counter_L[j]))
			{
				last_good_counter_H = counter_H[i];
				last_good_counter_L = counter_L[i];
				break;
			}
		}

	}
	//last_good_counter_L = 0x05;

}

void retrieve_last_good_break_in_counter(void)
{
	unsigned int counter_H[3];
	unsigned int counter_L[3];
	unsigned char i;
	unsigned char j;
	//read_eeprom
	counter_H[0] = read_eeprom(6);
	counter_L[0] = read_eeprom(7);
	counter_H[1] = read_eeprom(8);
	counter_L[1] = read_eeprom(9);
	counter_H[2] = read_eeprom(10);
	counter_L[2] = read_eeprom(11);

	//last_good_counter_H = counter_H[0];
	//last_good_counter_L = counter_L[0];

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			if((counter_H[i] == counter_H[j]) && (counter_L[i] == counter_L[j]))
			{
				last_good_break_in_counter_H = counter_H[i];
				last_good_break_in_counter_L = counter_L[i];
				break;
			}
		}

	}
	//last_good_counter_L = 0x05;

}

void increment_minute_counter(unsigned int increment)
{

	last_good_counter_L+=increment;
	if(last_good_counter_L < increment)
		last_good_counter_H++;

	unlock_eeprom();
	block_ms(1);
	write_eeprom(0,last_good_counter_H);
	block_ms(1);
	write_eeprom(1,last_good_counter_L);
	block_ms(1);
	write_eeprom(2,last_good_counter_H);
	block_ms(1);
	write_eeprom(3,last_good_counter_L);
	block_ms(1);
	write_eeprom(4,last_good_counter_H);
	block_ms(1);
	write_eeprom(5,last_good_counter_L);
	block_ms(1);
	lock_eeprom();


}

void increment_break_in_minute_counter(unsigned int increment)
{

	last_good_break_in_counter_L+=increment;
	if(last_good_break_in_counter_L < increment)
		last_good_break_in_counter_H++;

	unlock_eeprom();
	block_ms(1);
	write_eeprom(6,last_good_break_in_counter_H);
	block_ms(1);
	write_eeprom(7,last_good_break_in_counter_L);
	block_ms(1);
	write_eeprom(8,last_good_break_in_counter_H);
	block_ms(1);
	write_eeprom(9,last_good_break_in_counter_L);
	block_ms(1);
	write_eeprom(10,last_good_break_in_counter_H);
	block_ms(1);
	write_eeprom(11,last_good_break_in_counter_L);
	block_ms(1);
	lock_eeprom();


}


unsigned int read_eeprom(unsigned char address)
{
	unsigned int return_data;
	unsigned int dummy;
	SPI2STATbits.SPIROV = 0;
	EEPROM_CS = 1;
	SPI2BUF = READ_EEPROM | address;
	//SPI2BUF = READ_EEPROM;
	while(SPI2STATbits.SPITBF);
	while (!SPI2STATbits.SPIRBF);
	block_ms(5);
	dummy = SPI2BUF;
	dummy = SPI2BUF;
	SPI2BUF = 0x0000;
	while(SPI2STATbits.SPITBF);
	while (!SPI2STATbits.SPIRBF);
	return_data = SPI2BUF;
	EEPROM_CS = 0;
	return return_data;
	//return 0x04;
}



void write_eeprom(unsigned char address, unsigned int data)
{
	unsigned int dummy;
	SPI2STATbits.SPIROV = 0;
	EEPROM_CS = 1;
	SPI2BUF = WRITE_EEPROM | address;
	//SPI2BUF = WRITE_EEPROM;
	while(SPI2STATbits.SPITBF);
	while (!SPI2STATbits.SPIRBF);
	dummy = SPI2BUF;

	SPI2BUF = data;
	while(SPI2STATbits.SPITBF);
	while (!SPI2STATbits.SPIRBF);
	dummy = SPI2BUF;
	EEPROM_CS = 0;

}

void unlock_eeprom(void)
{
	unsigned int dummy;
	dummy = SPI2BUF;
	SPI2STATbits.SPIROV = 0;
	EEPROM_CS = 1;
	SPI2BUF = EWEN;
	while(SPI2STATbits.SPITBF);	
	while (!SPI2STATbits.SPIRBF);
	dummy = SPI2BUF;
	
	EEPROM_CS = 0;



}

void lock_eeprom(void)
{
	unsigned int dummy;
	EEPROM_CS = 1;
	SPI2BUF = EWDS;
	while(SPI2STATbits.SPITBF);	
	while (!SPI2STATbits.SPIRBF);
	dummy = SPI2BUF;
	EEPROM_CS = 0;

}

void increment_counter(void)
{
	//EWEN
	//WRITE
	//EWDS


}



void write_fake_hours(void)
{




}


void  __attribute__((__interrupt__, auto_psv)) _SPI2Interrupt(void)
{

	IFS2bits.SPI2IF = 0;
	SPI2STATbits.SPIROV = 0;












}
