#include "include/Compiler.h"
#include "Video_Receiver.h"




void Init_Video(void)
{


	//I don't want it to check the address
	I2C1MSK = 0x3FF;

	I2C1CONbits.IPMIEN = 0;

	I2C1CONbits.SCLREL = 1;
	I2C1CONbits.DISSLW = 1;


	I2C1BRG = 0xFFFE;


	//Enable I2C module
	I2C1CONbits.I2CEN = 1;




}


void Init_Video_Slave(void)
{


	//I don't want it to check the address
	I2C1MSK = 0x3FF;

	I2C1CONbits.IPMIEN = 0;




	//Enable I2C module
	I2C1CONbits.I2CEN = 1;




}

void Test_Video(void)
{
	//unsigned char i;
	//unsigned int j;

	block_ms(500);
	Init_Video();

	Set_Video_Channel(2);
	while(1);
	while(1)
	{
		/*for(i=1;i<=4;i++)
		{
			Set_Video_Channel(i);
			//I2C1TRN = 0xAA;
			for(j=0;j<1000;j++)
			Nop();
			block_ms(250);
			
		}*/

		
	}




}
void Get_I2C_Data_Loop(void)
{
	static unsigned char i;
	unsigned char i2cdata[20];
	i=0;

	Datalink_Init();
	Init_Video_Slave();
	for(i=0;i<20;i++)
	{
		i2cdata[i] = 0x77;
	}
	while(1)
	{


		i=0;
		while(i<20)
		{
			while(!I2C1STATbits.RBF);
			i2cdata[i] = I2C1RCV;
			i++;
		}
		i=0;
		for(i=0;i<20;i++)
		{
			Datalink_Send_Buffer[i] = i2cdata[i];
		}
		Datalink_Message_Send();
		block_ms(100);
	}






}

void Set_Video_Channel(char channel)
{
unsigned char i2cdata[5];
unsigned int i;
//unsigned int j;

i2cdata[0] = 0xc2;
//i2cdata[0] = 0xaa;
i2cdata[1] = 0x2b;
i2cdata[2] = 0x6c;
i2cdata[3] = 0x8e;
i2cdata[4] = 0xf0;

#ifdef MHz_900
	switch(channel)
	{
	
		case 0x01:
			i2cdata[1] = 0x2b;
			i2cdata[2] = 0x6c;

		break;
		case 0x02:
			i2cdata[1] = 0x2d;
			i2cdata[2] = 0x9c;

		break;
		case 0x03:
			i2cdata[1] = 0x2e;
			i2cdata[2] = 0x8c;
		break;
		case 0x04:
			i2cdata[1] = 0x2f;
			i2cdata[2] = 0x7c;
		break;
		default:
			//default to channel 1
			i2cdata[1] = 0x2b;
			i2cdata[2] = 0x6c;
		break;
	}
#endif

#ifdef MHz_1200
	switch(channel)
	{
	
		case 0x01:
			i2cdata[1] = 0x30;
			i2cdata[2] = 0xbc;
		break;
		case 0x02:
			i2cdata[1] = 0x31;
			i2cdata[2] = 0xfc;
		break;
		case 0x03:
			i2cdata[1] = 0x33;
			i2cdata[2] = 0x3c;
		break;
		case 0x04:
			i2cdata[1] = 0x34;
			i2cdata[2] = 0x7c;
		break;
		case 0x05:
			i2cdata[1] = 0x2e;
			i2cdata[2] = 0x8c;
		break;
		case 0x06:
			i2cdata[1] = 0x2f;
			i2cdata[2] = 0x7c;
		break;
		case 0x07:
			i2cdata[1] = 0x35;
			i2cdata[2] = 0xbc;
		break;
		case 0x08:
			i2cdata[1] = 0x36;
			i2cdata[2] = 0xfc;
		break;
		default:
			//default to channel 1
			i2cdata[1] = 0x30;
			i2cdata[2] = 0xbc;
		break;
	}

#endif
	I2C1CONbits.SEN=1;
	Nop();
	while(I2C1CONbits.SEN);
	//block_ms(2);


	/*	I2C1TRN = i2cdata[0];
		while(I2C1STATbits.TBF);
		for(i=0;i<1000;i++) Nop();
		I2C1TRN = i2cdata[1];
		while(I2C1STATbits.TBF);
		for(i=0;i<1000;i++) Nop();
		I2C1TRN = i2cdata[2];
		while(I2C1STATbits.TBF);
		for(i=0;i<1000;i++) Nop();
		I2C1TRN = i2cdata[3];
		while(I2C1STATbits.TBF);
		for(i=0;i<1000;i++) Nop();
		I2C1TRN = i2cdata[4];
		while(I2C1STATbits.TBF);*/
	for(i=0;i<5;i++)
	{
		I2C1TRN = i2cdata[i];
	//while(I2C1STATbits.ACKSTAT);
	while(I2C1STATbits.TRSTAT);
		//while(I2C1STATbits.TRSTAT);
		//for(j=0;j<5000;j++) Nop();
		//while(I2C1STATbits.TRSTAT);
		//block_ms(2);
	}
	I2C1CONbits.PEN = 1;
	Nop();
	while(I2C1CONbits.PEN);







}
