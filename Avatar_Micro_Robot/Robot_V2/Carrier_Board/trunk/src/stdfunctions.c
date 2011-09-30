/**
 * @file stdfunctions.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 *
 */

#include "stdhdr.h"

void serial_delay_tick(ticks)
{
	while(ticks--) { };
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



void writeI2C( unsigned char add, unsigned char v) // write an integer v to address add
{
	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1(add << 1);
	IdleI2C1();

	// Write data byte
	MasterWriteI2C1( v); 
	IdleI2C1();

	// Terminate command sequence with a stop condition
	StopI2C1();
	IdleI2C1();

} //writeI2C

void writeI2CReg( unsigned char add, unsigned char v, unsigned char w) // write an integer v to address add
{
	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1(add << 1);
	IdleI2C1();

	// Write data byte
	MasterWriteI2C1( v); 
	IdleI2C1();

	// Write data byte
	MasterWriteI2C1( w); 
	IdleI2C1();

	// Terminate command sequence with a stop condition
	StopI2C1();
	IdleI2C1();

} //writeI2C


int readI2C( unsigned char add)  // read an integer from address add
{
	int r;

	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1((add << 1) | 0x01);
	IdleI2C1();

	__delay_us(100);

	IFS1bits.MI2C1IF = 0;

	r = (unsigned int)(MasterReadI2C1());

	// terminate read sequence (do not send ACK, send  STOP)

	StopI2C1(); 
	IdleI2C1();

	return r;
} //readI2C
