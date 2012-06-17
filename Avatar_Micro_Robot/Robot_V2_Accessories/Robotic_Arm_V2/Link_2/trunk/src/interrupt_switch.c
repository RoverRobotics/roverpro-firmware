/**
 * @file interrupt_switch.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex carrier PIC firmware.
 *
 */

#include "interrupt_switch.h"

#pragma idata

void (*T1InterruptUserFunction)(void) = InterruptDummyFunction;
void (*T2InterruptUserFunction)(void) = InterruptDummyFunction;
void (*T3InterruptUserFunction)(void) = InterruptDummyFunction;
void (*T4InterruptUserFunction)(void) = InterruptDummyFunction;
void (*T5InterruptUserFunction)(void) = InterruptDummyFunction;
void (*IC1InterruptUserFunction)(void) = InterruptDummyFunction;
void (*IC2InterruptUserFunction)(void) = InterruptDummyFunction;
void (*IC3InterruptUserFunction)(void) = InterruptDummyFunction;
void (*IC4InterruptUserFunction)(void) = InterruptDummyFunction;
void (*IC5InterruptUserFunction)(void) = InterruptDummyFunction;
void (*IC6InterruptUserFunction)(void) = InterruptDummyFunction;
void (*ADC1InterruptUserFunction)(void) = InterruptDummyFunction;
void (*I2C1InterruptUserFunction)(void) = InterruptDummyFunction;
void (*U1TXInterruptUserFunction)(void) = InterruptDummyFunction;
void (*U1RXInterruptUserFunction)(void) = InterruptDummyFunction;
void (*U2RXInterruptUserFunction)(void) = InterruptDummyFunction;
void (*I2C2InterruptUserFunction)(void) = InterruptDummyFunction;
void (*U2TXInterruptUserFunction)(void) = InterruptDummyFunction;


#pragma code

// If nothing is set do nothing
void InterruptDummyFunction()
{
}

void  __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void)
{
	T1InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
{
 	T2InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
{
 	T3InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _T4Interrupt(void)
{
 	T4InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _T5Interrupt(void)
{
 	T5InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _IC1Interrupt(void)
{

 	IC1InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _IC2Interrupt(void)
{
 	IC2InterruptUserFunction();	
}

void  __attribute__((__interrupt__, auto_psv)) _IC3Interrupt(void)
{
 	IC3InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _IC4Interrupt(void)
{
 	IC4InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _IC5Interrupt(void)
{
 	IC5InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _IC6Interrupt(void)
{
 	IC6InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt(void)
{
 	ADC1InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void)
{
 	U1RXInterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _U2RXInterrupt(void)
{
 	U2RXInterruptUserFunction();
}


void  __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void)
{
 	U1TXInterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _U2TXInterrupt(void)
{
 	U2TXInterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _MI2C1Interrupt(void)
{
 	I2C1InterruptUserFunction();
}

void  __attribute__((__interrupt__, auto_psv)) _MI2C2Interrupt(void)
{
 	I2C2InterruptUserFunction();
}
