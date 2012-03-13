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



/*void writeI2C( unsigned char add, unsigned char v) // write an integer v to address add
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
*/

unsigned char readI2C1_Reg(unsigned char add, unsigned char reg)  // read an integer from address add
{
	unsigned char r;

	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1(add<<1);

	IdleI2C1();

	MasterWriteI2C1(reg);

	IdleI2C1();
	StopI2C1();
	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1((add << 1) | 0x01);
	IdleI2C1();

	__delay_us(100);

	IFS1bits.MI2C1IF = 0;

	//r = (unsigned char)(MasterReadI2C1());

	//AckI2C1();

	//IdleI2C1();

	r = (unsigned char)(MasterReadI2C1());	

	//IdleI2C1();
	NotAckI2C1();

	// terminate read sequence (do not send ACK, send  STOP)
	IdleI2C1();
	StopI2C1(); 
	IdleI2C1();

	return r;
} //readI2C


unsigned char readI2C2_Reg(unsigned char add, unsigned char reg)  // read an integer from address add
{
	unsigned char r;

	IdleI2C2();
	StartI2C2();
	IdleI2C2();

	MasterWriteI2C2(add<<1);

	IdleI2C2();

	MasterWriteI2C2(reg);

	IdleI2C2();
	StopI2C2();
	IdleI2C2();
	StartI2C2();
	IdleI2C2();

	MasterWriteI2C2((add << 1) | 0x01);
	IdleI2C2();

	__delay_us(100);

	_MI2C2IF = 0;

	//r = (unsigned char)(MasterReadI2C1());

	//AckI2C1();

	//IdleI2C1();

	r = (unsigned char)(MasterReadI2C2());	

	//IdleI2C1();
	NotAckI2C2();

	// terminate read sequence (do not send ACK, send  STOP)
	IdleI2C2();
	StopI2C2(); 
	IdleI2C2();

	return r;
} //readI2C

unsigned char readI2C3_Reg(unsigned char add, unsigned char reg)  // read an integer from address add
{
	unsigned char r;

	IdleI2C3();
	StartI2C3();
	IdleI2C3();

	MasterWriteI2C3(add<<1);

	IdleI2C3();

	MasterWriteI2C3(reg);

	IdleI2C3();
	StopI2C3();
	IdleI2C3();
	StartI2C3();
	IdleI2C3();

	MasterWriteI2C3((add << 1) | 0x01);
	IdleI2C3();

	__delay_us(100);

	_MI2C2IF = 0;

	//r = (unsigned char)(MasterReadI2C1());

	//AckI2C1();

	//IdleI2C1();

	r = (unsigned char)(MasterReadI2C3());	

	//IdleI2C1();
	NotAckI2C3();

	// terminate read sequence (do not send ACK, send  STOP)
	IdleI2C3();
	StopI2C3(); 
	IdleI2C3();

	return r;
} //readI2C

void writeI2C1Reg( unsigned char add, unsigned char v, unsigned char w) // write an integer v to address add
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

void writeI2C2Reg( unsigned char add, unsigned char v, unsigned char w) // write an integer v to address add
{
	IdleI2C2();
	StartI2C2();
	IdleI2C2();

	MasterWriteI2C2(add << 1);
	IdleI2C2();

	// Write data byte
	MasterWriteI2C2( v); 
	IdleI2C2();

	// Write data byte
	MasterWriteI2C2( w); 
	IdleI2C2();

	// Terminate command sequence with a stop condition
	StopI2C2();
	IdleI2C2();

} //writeI2C

void writeI2C3Reg( unsigned char add, unsigned char v, unsigned char w) // write an integer v to address add
{
	IdleI2C3();
	StartI2C3();
	IdleI2C3();

	MasterWriteI2C3(add << 1);
	IdleI2C3();

	// Write data byte
	MasterWriteI2C3( v); 
	IdleI2C3();

	// Write data byte
	MasterWriteI2C3( w); 
	IdleI2C3();

	// Terminate command sequence with a stop condition
	StopI2C3();
	IdleI2C3();

} //writeI2C

unsigned int readI2C2_Word(unsigned char add, unsigned char reg)  // read an integer from address add
{
	unsigned char a,b;

	IdleI2C2();
	StartI2C2();
	IdleI2C2();

	MasterWriteI2C2(add<<1);

	IdleI2C2();

	MasterWriteI2C2(reg);

	IdleI2C2();
/*	StopI2C2();
	IdleI2C2();
	StartI2C2();*/
	RestartI2C2();
	IdleI2C2();

	MasterWriteI2C2((add << 1) | 0x01);
	IdleI2C2();

	__delay_us(100);

	_MI2C2IF = 0;

	//r = (unsigned char)(MasterReadI2C1());

	//AckI2C1();

	//IdleI2C1();

	a = (unsigned char)(MasterReadI2C2());	

	AckI2C2();

	IdleI2C2();

	b = (unsigned char)(MasterReadI2C2());	

	//IdleI2C1();
	NotAckI2C2();

	// terminate read sequence (do not send ACK, send  STOP)
	IdleI2C2();
	StopI2C2(); 
	IdleI2C2();

	return a+(b<<8);
} //readI2C

unsigned int readI2C3_Word(unsigned char add, unsigned char reg)  // read an integer from address add
{
	unsigned char a,b;

	IdleI2C3();
	StartI2C3();
	IdleI2C3();

	MasterWriteI2C3(add<<1);

	IdleI2C3();

	MasterWriteI2C3(reg);

	IdleI2C3();
/*	StopI2C3();
	IdleI2C3();
	StartI2C3();*/
	RestartI2C3();
	IdleI2C3();

	MasterWriteI2C3((add << 1) | 0x01);
	IdleI2C3();

	__delay_us(100);

	_MI2C3IF = 0;

	//r = (unsigned char)(MasterReadI2C1());

	//AckI2C1();

	//IdleI2C1();

	a = (unsigned char)(MasterReadI2C3());	

	AckI2C3();

	IdleI2C3();

	b = (unsigned char)(MasterReadI2C3());	

	//IdleI2C1();
	NotAckI2C3();

	// terminate read sequence (do not send ACK, send  STOP)
	IdleI2C3();
	StopI2C3(); 
	IdleI2C3();

	return a+(b<<8);
} //readI2C

void writeI2C2Word( unsigned char add, unsigned char v, unsigned int w) // write an integer v to address add
{
	IdleI2C2();
	StartI2C2();
	IdleI2C2();

	MasterWriteI2C2(add << 1);
	IdleI2C2();

	// Write data byte
	MasterWriteI2C2( v); 
	IdleI2C2();

	// Write data low
	MasterWriteI2C2( w & 0xff); 
	IdleI2C2();

	//Write high data byte
	MasterWriteI2C2( w>>8); 
	IdleI2C2();

	// Terminate command sequence with a stop condition
	StopI2C2();
	IdleI2C2();

} //writeI2C
