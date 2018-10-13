#include "include/Compiler.h"
#include "address_generation.h"
#include "DEE Emulation 16-bit.h"
#include "Controller.h"
#include "Timers.h"
#include "Audio.h"

void generate_OCU_address(void);
void remove_OCU_address(void);
unsigned char wait_for_keys(unsigned char matching_byte);
void look_for_code(void);
unsigned char return_button_states(void);
void set_universal_address(void);

void set_stored_address(void)
{
	unsigned char stored_address[2];
	unsigned int CRC_read;
	unsigned int CRC_calculated;
	unsigned char CRC1,CRC2;
	unsigned char already_written;
	unsigned char data_to_CRC[4];
	DataEEInit();
	Nop();
	stored_address[0] = DataEERead(0);
	Nop();
	stored_address[1] = DataEERead(1);
	Nop();
	CRC1 = DataEERead(2);
	Nop();
	CRC2 = DataEERead(3);
	Nop();
	already_written = DataEERead(4);
	Nop();

	data_to_CRC[2] = stored_address[0];
	data_to_CRC[3] = stored_address[1];

	CRC_calculated = return_crc(data_to_CRC,2);

	CRC_read = (CRC1<<8)+(CRC2);




	if((stored_address[0] == 0xff) && (stored_address[1] == 0xff) && (already_written != 0xaa))
	{
		generate_OCU_address();
	}
	else if(CRC_read == CRC_calculated)
	{
		OCU_ADDRESS_MSB = stored_address[0];
		OCU_ADDRESS_LSB = stored_address[1];
		ROBOT_ADDRESS_MSB = stored_address[0];
		ROBOT_ADDRESS_LSB = stored_address[1];


	}

	if(TALK_BUTTON && LIGHT_BUTTON)
	{
		look_for_code();
		return;
	}	


}

void generate_OCU_address(void)
{
	unsigned char OCU_MSB, OCU_LSB;
	unsigned char data_to_CRC[4];
	unsigned int CRC_calculated = 0;

	T1CONbits.TON = 1;



	//Wait for green button to be pressed, to allow for varied TMR0 numbers
	while(PAYLOAD_BUTTON == 0)
	{
		ClrWdt();
		if(TALK_BUTTON && (LIGHT_BUTTON == 0))
		{
			set_universal_address();
			return;
		}

	}
//	srand(TMR1);
	OCU_MSB = TMR1%256;
	block_ms(100);
	//seed again after button is released
	while(PAYLOAD_BUTTON == 1)
	{
		ClrWdt();
	}
//	srand(TMR1);


//	OCU_LSB = rand()%256;
	OCU_LSB = TMR1%256;
	DataEEInit();
	Nop();


	data_to_CRC[2] = OCU_MSB;
	data_to_CRC[3] = OCU_LSB;

	CRC_calculated = return_crc(data_to_CRC,2);

	//DataEEInit();
	DataEEWrite(OCU_MSB,0);
	Nop();
	DataEEWrite(OCU_LSB,1);
	Nop();
	DataEEWrite((CRC_calculated >> 8),2);
	Nop();
	DataEEWrite((CRC_calculated & 0xff),3);
	Nop();

//	write a specific value to a known location,
//	so we know that the address has been written for
//	the first time.
	DataEEWrite(0xAA,4);

	Nop();

	OCU_ADDRESS_MSB = OCU_MSB;
	OCU_ADDRESS_LSB = OCU_LSB;
	ROBOT_ADDRESS_MSB = OCU_MSB;
	ROBOT_ADDRESS_LSB = OCU_LSB;



}

void set_universal_address(void)
{

	OCU_ADDRESS_MSB = 0x00;
	OCU_ADDRESS_LSB = 0x00;
	ROBOT_ADDRESS_MSB = 0xff;
	ROBOT_ADDRESS_LSB = 0xff;


}

void look_for_code(void)
{
	unsigned char button_byte = 0x00;
	unsigned char keys[5] = {0x80,0x40,0x80,0x40,0x20};
	unsigned char i;

	for(i=0;i<5;i++)
	{

		while(wait_for_keys(0x00) == 0);
		block_ms(100);
		if(wait_for_keys(keys[i]) == 0)
			return;
		block_ms(100);	
	}

	
	while(wait_for_keys(0x00) == 0);
	block_ms(100);



	while(wait_for_keys(0x06) == 0)
	{
		button_byte = return_button_states();
		if((button_byte == 0x06) || (button_byte == 0x04) || (button_byte == 0x02) || (button_byte == 0x00))
		{
		}
		else
			return;
	}
	
	remove_OCU_address();

	Audio_Receive();
	


	for(i=0;i<5;i++)
	{
		Audio_Unmute();
		block_ms(250);
		Audio_Mute();
		block_ms(250);
	}

}

void remove_OCU_address(void)
{

	unsigned char i;

	DataEEInit();
	Nop();

	for(i=0;i<5;i++)
	{
		DataEEWrite(0xff,i);
		Nop();
	}

}

unsigned char wait_for_keys(unsigned char matching_byte)
{
	unsigned char button_states = 0x00;
	while(1)
	{
		ClrWdt();
		button_states = return_button_states();
		if(button_states == matching_byte)
		{
			return 0x01;
		}
		else if(button_states == 0x00);
		else
		{
			return 0x00;
		}

	}
}

unsigned char return_button_states(void)
{
	unsigned char return_byte = 0x00;
	if(LIGHT_BUTTON) return_byte |= 0x80;
	if(TALK_BUTTON) return_byte |= 0x40;
	if(FLIPPER_UP) return_byte |= 0x20;
	if(FLIPPER_DOWN) return_byte |= 0x10;
	if(PAYLOAD_BUTTON) return_byte |= 0x08;
	if(LEFT_TRIGGER) return_byte |= 0x04;
	if(RIGHT_TRIGGER) return_byte |= 0x02;

	return return_byte;
}
