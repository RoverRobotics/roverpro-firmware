#include "include/Compiler.h"
#include "SPI.h"
#include "RS485.h"
#include "Timers.h"
//unsigned char OSD_write_reg_16_bit(unsigned char reg,unsigned char value);
void CS_falling_edge(void);
void CS_rising_edge(void);
//void block_ms(unsigned int ms);
//void OSD_display_message(const ROM char *message, unsigned char col, unsigned char row, unsigned char length);
unsigned char OSD_lookup_char(unsigned char character);
unsigned char return_AD_value(unsigned char channel);

unsigned char MAX_write_reg(unsigned char reg,unsigned char value);

//module level variables used in OSD interrupt
unsigned char OSD_message[30];
unsigned char OSD_DMAH;
unsigned char OSD_DMAL;
unsigned char OSD_message_length;
unsigned char logo_reg[2950];
unsigned char VM0_value = 0x28;


void OSD_display_string(const ROM char *message, unsigned char col, unsigned char row, unsigned char length)
{

	unsigned int i;
	//unsigned char address_high;
	//unsigned char address_low;
	//unsigned char send_char;

	unsigned int address_int;

	//calculate 9 bit display address
	address_int = (unsigned int)row*30+col;
	
	//separate it into high and low byte
	OSD_DMAH = (address_int >> 8)&0x01;
	OSD_DMAL = address_int&0xFF;
	

	for(i=0; i < length; i++)
	{
		OSD_message[i] = OSD_lookup_char(message[i]);
	}

	OSD_message_length = length;
	//OSD_byte_counter = 0;
	CS_falling_edge();
	
	SPI1BUF = DMAH;
	IEC0bits.SPI1IE = 1;
}

void OSD_display_chars(unsigned char* message, unsigned char col, unsigned char row, unsigned char length)
{

	unsigned int i;

	//unsigned char send_char;

	unsigned int address_int;

	//calculate 9 bit display address
	address_int = (unsigned int)row*30+col;
	
	//separate it into high and low byte
	OSD_DMAH = (address_int >> 8)&0x01;

	OSD_DMAL = address_int&0xFF;
	

	for(i=0; i < length; i++)
	{
		OSD_message[i] = message[i];


	}

	OSD_message_length = length;
	
	//load SPI buffer to start chain of interrupts
	SPI1BUF = DMAH;
	IEC0bits.SPI1IE = 1;
}




unsigned char OSD_lookup_char(unsigned char character)
{

	//lower case letters
	if (character >= 0x61 && character <= 0x7A)
	{
		return character - 0x3c;
	}
	//numbers 1-9
	else if (character  >= 0x31 && character <= 0x39)
	{
		return character-0x30;
	}
	//upper case letters
	else if(character >= 0x41 && character <= 0x5A)
	{
		return character-0x36;
	}
	//number 0
	else if(character == 0x30)
	{
		return 0x0A;
	}
	//space
	else if (character == 0x20)
	{
		return 0x00;
	}

	//if character is 0x20 or invalid, return space
	return character;



}



void CS_falling_edge(void)
{
	//unsigned int i;
	CS_LAT = 0;
	Nop();
	

}

void CS_rising_edge(void)
{
	//unsigned int i;
	Nop();
	CS_LAT = 1;
	Nop();
	Nop();

}

/**********************************************************
**	SPI_Init
**	Initializes all of the registers necessary to use
**	the SPI subsystem, utilizing interrupts for sending.
**********************************************************/
void SPI_Init(void)
{


	SPI1CON1 = 0;
	SPI1CON2 = 0;
	
	//make RP24 = SDI
	//RP25 = SD0
	//RP22 = SCK (output, since always master)
	//output function number for SCK1OUT = 8, SDO1 = 7, 
	//set
	RPOR11bits.RP22R = 0x08;	//SCK
	RPOR11bits.RP23R = 0x07;	//SDO

	//SPI receive in RP23
	RPINR20bits.SDI1R = 23;

	SPI1CON1bits.MSTEN = 1;

	SPI1STATbits.SPIROV = 0;	

	//D1 is digital output, used for chip select
//	TRISDbits.TRISD1 = 0;
//	LATDbits.LATD1 = 1;
	

	SPI1CON1bits.CKE = 1;

	//clear the interrupt flag
	IFS0bits.SPI1IF = 0;

	//enable interrupt
	//IEC0bits.SPI1IE = 1;

	//Enable SPI subsystem
	SPI1STATbits.SPIEN = 1;

}


/*void block_ms(unsigned int ms)
{
	unsigned int i;
	unsigned int j;

	for(i=0;i<160;i++)

	{
		for(j=0;j<ms;j++);
	}
}*/


/***********************************************
**	rewrite_space(void)
**
**	For some reason, the first character (0x00)
**	kept getting rewritten.  This produced lines
**	all over the screen, necessitating this function.
**	Note that this function will no longer work, since
**	it relies on blocking code that was removed 11/19/208
***********************************************/
void rewrite_space(void)
{

	unsigned int i;

	//character address is 0x00
	MAX_write_reg(CMAH,0x00);
	
	//Each pixel in the character is assigned two bits, since 
	//there are three states (black, white, transparent) that
	//the pixel can take.  This loop sets all of the pixels to
	//11 (transparent)
	for(i=0;i<54;i++)
	{
		MAX_write_reg(CMAL,i);
		MAX_write_reg(CMDI,0xFF);
	}

	//write all of the data from shadow RAM to the actual registers
	MAX_write_reg(CMM,0xA0);
	block_ms(100);
}




void display_logo(void)
{
	unsigned char message[30];
	unsigned char i;
	//unsigned char j;
	unsigned char k;
	MAX_Internal_Synch();
	//message[1] = 0x83;

	for(i=0;i<29;i++)
	{
		message[i] = 0x83;
	}
	for(i=0;i<15;i++)
	{
		OSD_display_chars(message,0,i,30);
		block_ms(15);
		//block_ms(200);
	}

	for(i=0;i<6;i++)
	{
		for(k=0;k<9;k++)
		{

			message[k] = 0x4D+i*9+k;
		}

			OSD_display_chars(message,10,4+i,9);
			block_ms(5);
		
	}	



}
void OSD_loop(void)
{
	unsigned char byte_in[1];
	SPI_Init();
	//software_SPI_send_init();
	
	//rewrite_space();
	byte_in[0] = 0x77;
	while(1)
	{

	//OSD_display_message("OCU Online",2,1,10);
	//OSD_display_message("Waiting for robot",2,2,17);
	//OSD_display_message("Robot found",2,3,11);
	OSD_display_string("RoboteX Avatar",7,9,14);
	//OSD_display_chars(byte_in,14,6,1);

	block_ms(5);

	}
}

void MAX_Internal_Synch(void)
{
	//MAX_write_reg(VM0,0x38);
	//block_ms(5);
	VM0_value = 0x38;

}

void MAX_External_Synch(void)
{
	VM0_value = 0x28;

}


/*************************************************
**	SPI1Interrupt
**
**	This gets called every time an SPI byte is done sending.
**	It keeps sending bytes in the message until the message
**	has been sent.  To stop sending, this function does not load
**	the SPI1BUF after the last byte has been sent.
*************************************************/
void  __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt(void)
{

	static char OSD_byte_counter = 0;
	IFS0bits.SPI1IF = 0;
	SPI1STATbits.SPIROV = 0;

	//Send all of the bytes in the message in auto-increment mode
	if(OSD_byte_counter >= 6 && OSD_byte_counter <= 5+OSD_message_length)
	{
		CS_rising_edge();
		CS_falling_edge();
		SPI1BUF = OSD_message[OSD_byte_counter-6];
	}




	//Does some setup to start sending text to the screen
	else if(OSD_byte_counter <=5)
	{
			switch( OSD_byte_counter)
			{

				case 0x00:
					CS_rising_edge();
					CS_falling_edge();
					SPI1BUF = DMAH;
				break;
				case 0x01:
					SPI1BUF = OSD_DMAH;
				break;
				case 0x02:
					CS_rising_edge();
					CS_falling_edge();
					SPI1BUF = DMAL;
				break;
				case 0x03:
					SPI1BUF = OSD_DMAL;
				break;
				case 0x04:
					CS_rising_edge();
					CS_falling_edge();
					SPI1BUF = DMM;
				break;
				case 0x05:
					SPI1BUF = 0x01;
				break;

					
		
			}
	}
	else if (OSD_byte_counter >= 6+OSD_message_length)
	{
		//Sends 0xFF to stop auto-increment mode
		if(OSD_byte_counter == (6+OSD_message_length))
		{
			CS_rising_edge();
			CS_falling_edge();
			SPI1BUF = 0xFF;
		}
		//Turns on OSD
		else if(OSD_byte_counter == (7+OSD_message_length))
		{
			CS_rising_edge();
			CS_falling_edge();
			SPI1BUF = VM0;
		}
		else if(OSD_byte_counter == (8+OSD_message_length))
		{
			SPI1BUF = VM0_value;
			//SPI1BUF = 0x2C;
			//SPI1BUF = 0x08;
			
		}
		//End of message.  Reset byte counter, and do not load SPI1BUF.
		else
		{
			CS_rising_edge();
			OSD_byte_counter = -1;
		}


	}

	OSD_byte_counter++;

}










 unsigned char SPI_Transfer(unsigned char byte_in)
 {
 		SPI1BUF = byte_in;
 		Nop();
 		Nop();
 		Nop();
 		Nop();
 		Nop();
 		Nop();
 		Nop();
 		Nop();
 		while(SPI1STATbits.SPITBF);
 		while(!SPI1STATbits.SPIRBF);
 		//SPI1STATbits.SPIROV=0;
 		return SPI1BUF;
 		
 
 
 }

unsigned char MAX_write_reg(unsigned char reg,unsigned char value)
 {
	unsigned int i;
 	unsigned char result;
	CS_rising_edge();
	Nop();
	Nop();
 	CS_falling_edge();
	for(i=0;i<100;i++);
 	SPI_Transfer(reg);

	for(i=0;i<100;i++);
	SPI_Transfer(value);
	for(i=0;i<100;i++);
 	CS_rising_edge();
	for(i=0;i<100;i++);
 	return result;
 
 
 }



void disable_video(unsigned char ms)
{
	MAX_write_reg(VM0,0x00);
	//block_ms(ms);
	block_ms(500);
	MAX_write_reg(VM0,0x0c);
	block_ms(50);



}




void clear_screen(void)
{
	IEC0bits.SPI1IE = 0;
	CS_rising_edge();
	block_ms(5);
	//MAX_write_reg(VM0,0x02);
	//block_ms(300);
	MAX_write_reg(DMM,0x04);
	block_ms(100);


}

void ghetto_clear_screen(void)
{

	unsigned char i;
	unsigned char message[30];
	for(i=0;i<29;i++)
	{
		message[i] = 0x00;
	}
	for(i=0;i<15;i++)
	{
		OSD_display_chars(message,0,i,30);
		block_ms(10);
	}



}

void display_voltage(void)
{
	static unsigned char sample_counter = 0;
	static unsigned int sample_sum = 0;
	unsigned char ad_value;
	unsigned char voltage_tens;
	unsigned char voltage_ones;
	unsigned char voltage_tenths;
	static unsigned char clean_value = 0;
	unsigned char voltage[6];
	//unsigned char voltage_hundredths;
	OSD_display_string("          ",8,8,6);
	voltage[0] = 0x07;
	voltage[1] = 0x07;
	voltage[2] = 0x41; // period
	voltage[3] = 0x07;
	voltage[4] = 0x00; //space
	voltage[5] = 0x20; //V







	ad_value = return_AD_value(0x0b);





	sample_sum = sample_sum+ad_value;
	sample_counter = sample_counter+1;
	//voltage divider is currently 7.5k and 20k
	//3.3V = 0xFF = 3.3/( 7.5/27.5) = 12.1V
	//12.1V / 255 = .04745 V
	
	if(sample_counter >= 50)
	{
		clean_value = sample_sum/sample_counter;
		sample_sum = 0;
		sample_counter = 0;
	}

	if(clean_value == 0)
	{
		return;
	}
	voltage_tens = clean_value*.004745;
	voltage_ones = clean_value*.04745-voltage_tens*10;
	voltage_tenths = clean_value*.475-voltage_tens*100-voltage_ones*10;
	if(voltage_ones == 0) voltage_ones = 0x0a;
	if(voltage_tenths ==0) voltage_tenths = 0x0a;
	voltage[0] = voltage_tens;
	voltage[1] = voltage_ones;
	voltage[3] = voltage_tenths;

	OSD_display_chars(voltage,22,12,6);
	


}


unsigned char return_AD_value(unsigned char channel)
{
	if(channel > 15) return 0;
	
	AD1CON1bits.ADON = 0;
	AD1CHSbits.CH0SA = channel;

	AD1CON1bits.ADON = 1;
	Nop();
	AD1CON1bits.SAMP = 1;

	while(AD1CON1bits.SAMP);

	while(!AD1CON1bits.DONE);
	return ADC1BUF0>>2;


}


void write_logo(void)
{
	unsigned char i;
	unsigned char j;



	for(i=0;i<54;i++)
	{

		MAX_write_reg(CMAH,i+0x4D);
		for(j=0;j<54;j++)
		{
			MAX_write_reg(CMAL,j);
			MAX_write_reg(CMDI,logo_reg[i*54+j]);
		}
		//for(j=54;j<64;j++)
		//{
		//	MAX_write_reg(CMAL,j);
		//	MAX_write_reg(CMDI,0x55);
		//}
		MAX_write_reg(CMM,0xA0);
		block_ms(20);
	}


	MAX_write_reg(CMAH,0x83);
	for(i=0;i<54;i++)
	{
			MAX_write_reg(CMAL,i);
			MAX_write_reg(CMDI,0x00);
		

	}
		MAX_write_reg(CMM,0xA0);
		block_ms(20);

}
