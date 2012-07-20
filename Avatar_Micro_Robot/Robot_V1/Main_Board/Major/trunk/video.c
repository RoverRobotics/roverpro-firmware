#include "system.h"
//void OSD_display_string(const ROM char *message, unsigned char col, unsigned char row, unsigned char length);


//translates ascii character number to the number of the
//character stored in the MAX7456 memory
unsigned char OSD_lookup_char(unsigned char character);
void CS_rising_edge(void);
void CS_falling_edge(void);
void display_battery_levels(unsigned char robot_voltage, unsigned char OCU_voltage);

unsigned char OSD_message[255];
unsigned char OSD_DMAH;
unsigned char OSD_DMAL;
unsigned int OSD_message_length;
unsigned char VM0_value = 0x28;

unsigned char clear_screen = 0;
unsigned char OSD_state = NORMAL_OSD;

#define BATTERY_FULL 0xEE
#define BATTERY_3_4 0xEF
#define BATTERY_HALF 0xF0
#define BATTERY_1_4 0xF1
#define BATTERY_EMPTY 0xF2

//controls the MAX4311 video switcher, with 2 bits, to
//select channel 0-3 (only 0-2 currently connected)
void select_video_channel(unsigned char channel)
{


	switch(channel)
	{
		case 0:
			VIDEO_SEL0 = 0;
			VIDEO_SEL1 = 0;
		break;
		case 1:
			VIDEO_SEL0 = 1;
			VIDEO_SEL1 = 0;
		break;
		case 2:
			VIDEO_SEL0 = 0;
			VIDEO_SEL1 = 1;
		break;
		case 3:
			VIDEO_SEL0 = 1;
			VIDEO_SEL1 = 1;
		break;
		default:
			VIDEO_SEL0 = 0;
			VIDEO_SEL1 = 0;
	}


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
	

	SPI1CON1bits.MSTEN = 1;

	SPI1STATbits.SPIROV = 0;	

	SPI1CON1bits.CKE = 1;

	//clear the interrupt flag
	IFS0bits.SPI1IF = 0;


	//Enable SPI subsystem
	SPI1STATbits.SPIEN = 1;

}


void OSD_clear_screen(void)
{

	//clear_screen = 1;
	OSD_state = CLEAR_SCREEN;
	OSD_DMAH = 0;
	OSD_DMAL = 0;
	



	CS_rising_edge();
	CS_falling_edge();
	
	SPI1BUF = VM0;
	IEC0bits.SPI1IE = 1;

	block_ms(10);
	

	//re-enable the display
	OSD_state = ENABLE_DISPLAY;
	CS_rising_edge();
	CS_falling_edge();
	SPI1BUF = VM0;
	IEC0bits.SPI1IE = 1;	


}




void video_test(void)
{

	unsigned int i;


	unsigned int address_int;

	//calculate 9 bit display address
	address_int = (unsigned int)0;
	
	//separate it into high and low byte
	OSD_DMAH = (address_int >> 8)&0x01;
	OSD_DMAL = address_int&0xFF;
	

	for(i=0; i < 29; i++)
	{
		OSD_message[i] = 0x0c;
	}

	OSD_message_length = 29;
	CS_falling_edge();
	
	SPI1BUF = DMAH;
	IEC0bits.SPI1IE = 1;


}

void OSD_display_string(char *message, unsigned char col, unsigned char row, unsigned char length)
{

	unsigned int i;

	unsigned int address_int;


	OSD_state = NORMAL_OSD;

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

	CS_falling_edge();
	
	SPI1BUF = DMAH;
	IEC0bits.SPI1IE = 1;
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

//translates ascii character number to the number of the
//character stored in the MAX7456 memory
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
	
	else
	{
		switch(character){
			case ' ':
				return 0x00;
				break;
			case '0':
				return 0x0A;
				break;
			case '.':
				return 0x41;
			break;
			default:
				return 0x20;

		}

	}


}










void display_voltage(void)
{
	//static unsigned char sample_counter = 0;
	//static unsigned int sample_sum = 0;
	unsigned char ad_value;
	unsigned int ad_accumulator = 0;
	//unsigned char voltage_tens;
	//unsigned char voltage_ones;
	//unsigned char voltage_tenths;
	static unsigned char clean_value = 0;
	unsigned char voltage[6];
	unsigned char i;
	//unsigned char value_to_print[1];
	//unsigned char voltage_hundredths;
	//OSD_display_string("          ",8,8,6);
	voltage[0] = 0x07;
	voltage[1] = 0x07;
	voltage[2] = 0x41; // period
	voltage[3] = 0x07;
	voltage[4] = 0x00; //space
	voltage[5] = 0x20; //V





	//take 3 ADC measurements, average them to get rid of noise
	for(i=0;i<3;i++)
	{
		ad_accumulator += return_AD_value(0x0c);
		block_ms(20);
	}
	

	ad_value = ad_accumulator/3;
/*
	value_to_print[0] = (unsigned char)sample_counter;



	sample_sum = sample_sum+ad_value;
	sample_counter = sample_counter+1;
	//voltage divider is currently 7.5k and 20k
	//AD = V*2k/12k/3.3*255 = 12.88 V
	//V = AD/8.5 = .0776*AD
	

	if(sample_counter >= 5)
	{
		clean_value = sample_sum/sample_counter;
		sample_sum = 0;
		sample_counter = 0;
	}*/

	clean_value = ad_value;
	if(clean_value == 0)
	{
		//OSD_display_string("Got here1",2,1,9);
		//OSD_display_chars(value_to_print,2,1,1);
		return;
	}/*
	voltage_tens = clean_value*.00776;
	voltage_ones = clean_value*.0776-voltage_tens*10;
	voltage_tenths = clean_value*.776-voltage_tens*100-voltage_ones*10;
	if(voltage_ones == 0) voltage_ones = 0x0a;
	if(voltage_tenths ==0) voltage_tenths = 0x0a;
	voltage[0] = voltage_tens;
	voltage[1] = voltage_ones;
	voltage[3] = voltage_tenths;


	OSD_display_chars(voltage,24,10,4);

	block_ms(15);*/

	//.776*clean_value is equal to 10 time the voltage (i.e. 159 = 15.9V)
	//display_battery_levels(	.776*clean_value,0);
	display_battery_levels(clean_value,0);


}




unsigned char return_AD_value(unsigned char channel)
{
	//if(channel > 15) return 0;
	

	AD1CON1bits.ADON = 0;
	Nop();
	AD1CHS = channel;
	AD1CON1bits.ADON = 1;
	Nop();
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE);


	return ADC1BUF0>>2;


}

void OSD_display_chars(unsigned char* message, unsigned char col, unsigned char row, unsigned char length)
{

	unsigned int i;

	//unsigned char send_char;

	unsigned int address_int;

	OSD_state = NORMAL_OSD;

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



void display_battery_levels(unsigned char robot_voltage, unsigned char OCU_voltage)
{

	unsigned char OCU_voltage_message[7];
	unsigned char robot_voltage_message[7];
//	unsigned char OCU_voltage = 0;

//	OCU_voltage = Return_Last_Datalink_Message(10) & 0b00000111;

	OCU_voltage_message[0] = 0x19;	//O
	OCU_voltage_message[1] = 0x0D;	//C
	OCU_voltage_message[2] = 0x1F;	//U
	OCU_voltage_message[3] = 0x00;	//[space]
	OCU_voltage_message[4] = 0xED;	//battery left end
	OCU_voltage_message[5] = BATTERY_FULL;
	OCU_voltage_message[6] = 0xF3;	//battery right end
	

	robot_voltage_message[0] = 0x1C;	//R
	robot_voltage_message[1] = 0x19;	//O
	robot_voltage_message[2] = 0x0C;	//B
	robot_voltage_message[3] = 0x00;	//[space]
	robot_voltage_message[4] = 0xED;	//battery left end
	robot_voltage_message[5] = BATTERY_FULL;
	robot_voltage_message[6] = 0xF3;	//battery right end

	switch(OCU_battery_voltage)
	{
		case 0x00:
			OCU_voltage_message[5] = BATTERY_EMPTY;
		break;
		case 0x01:
			OCU_voltage_message[5] = BATTERY_1_4;
		break;
		case 0x02:
			OCU_voltage_message[5] = BATTERY_HALF;
		break;
		case 0x03:
			OCU_voltage_message[5] = BATTERY_3_4;
		break;
		case 0x04:
			OCU_voltage_message[5] = BATTERY_FULL;
		break;
	}

	//Robot voltage is now the straight AD value.  Before, it
	//was 10x the decimal voltage
	if(robot_voltage >= 196)
		robot_voltage_message[5] = BATTERY_FULL;
	else if(robot_voltage >= 190)
		robot_voltage_message[5] = BATTERY_3_4;
	else if(robot_voltage >= 186)
		robot_voltage_message[5] = BATTERY_HALF;
	else if(robot_voltage >= 184)
		robot_voltage_message[5] = BATTERY_1_4;
	else
	{
			robot_voltage_message[4] = 0x16;	//L
			robot_voltage_message[5] = 0x19;	//O
			robot_voltage_message[6] = 0x21;	//W
	}



	OSD_display_chars(robot_voltage_message,22,11,7);
	block_ms(20);
	OSD_display_chars(OCU_voltage_message,22,12,7);





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

	static int OSD_byte_counter = 0;
	IFS0bits.SPI1IF = 0;
	SPI1STATbits.SPIROV = 0;



	if(OSD_state == CLEAR_SCREEN)
	{
	
			if(OSD_byte_counter == 0)
				SPI1BUF = 0x26;
			
	
			else
			{
	
				CS_rising_edge();
				OSD_state = NORMAL_OSD;
				OSD_byte_counter = -1;
			}
	
			OSD_byte_counter++;
	
	
	
	
	}
	else if(OSD_state == NORMAL_OSD)
	{
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
				clear_screen = 0;
				OSD_byte_counter = -1;
			}
	
	
		}
	
		OSD_byte_counter++;
	}
	else if(OSD_state == ENABLE_DISPLAY)
	{

			if(OSD_byte_counter == 0)
				SPI1BUF = 0x2c;
			
	
			else
			{
	
				CS_rising_edge();
				OSD_state = NORMAL_OSD;
				OSD_byte_counter = -1;
			}
	
			OSD_byte_counter++;

	}
}
