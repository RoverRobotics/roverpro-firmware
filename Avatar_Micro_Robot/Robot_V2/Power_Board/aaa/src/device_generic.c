/**
 * @file device_generic.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex generic PIC firmware.
 *
 */

#include "stdhdr.h"
#include "device_generic.h"

#pragma udata

uint16_t gPBusCurrent = 0;
uint16_t gPBus = 1;
uint8_t gBatAorB = 0;
uint8_t gCurrentLimiting = 0;
uint16_t gCurrentLimitingCounter = 0;
uint16_t t = 0;

#define mInitBAT_A_B() TRISE &= ~0x00C0; // RE6 & RE7
#define mBAT_A_On() LATE |= 0x0080;
#define mBAT_A_Off() LATE &= ~0x0080;
#define mBAT_B_On() LATE |= 0x0004;
#define mBAT_B_Off() LATE &= ~0x0004;

#pragma code

void DeviceGenericInit()
{
    /*mInitBAT_A_B();

	for (t = 0; t < 65000; t++)
	{
		mBAT_A_On();
		mBAT_B_On();
	}

	mBAT_A_Off();
	mBAT_B_Off();
*/
	AD1PCFGL = 0xFFFE; // configure ADC

	mInitAllLEDs();
}


void DeviceGenericProcessIO()
{
	if (REG_TEST_VAR2 >= 500)
	{
		mLED_1_On();
		mLED_2_On();
		mLED_3_On();
	}
	else
	{
		mLED_1_Off();
		mLED_2_Off();
		mLED_3_Off();
	}

    AD1CON1             = 0x00E4;   // Off, Auto sample start, auto-convert
    AD1CON2             = 0;        // AVdd, AVss, int every conversion, MUXA only
    AD1CON3             = 0x1F05;   // 31 Tad auto-sample, Tad = 5*Tcy
    AD1CHS              = 0x0;      // MUXA uses AN0
    AD1CSSL             = 0;        // No scanned inputs
    AD1CON1bits.ADON    = 1;        // Turn on module

    while(!AD1CON1bits.DONE);       // Wait for conversion to complete

	REG_TEST_VAR = (float)ADC1BUF0;		// get power bus current
    AD1CON1bits.ADON    = 0;        // Turn off module

	return;

	// 14.6 counts per amp
	if (gPBusCurrent > 146) {gCurrentLimiting = 1; gCurrentLimitingCounter = 0;}

	if (gCurrentLimiting)
	{
		mBAT_A_Off();
		mBAT_B_Off();
		gCurrentLimitingCounter++;
		if (gCurrentLimitingCounter > 40000)
		{
			gCurrentLimiting = 0;
			// disable current limiting so caps can charge
			for (t = 0; t < 32000; t++)
			{
				mBAT_A_On();
				mBAT_B_On();
			}
		}
	}
	else
	{
		if (gPBus)
		{
			mBAT_A_On();
			mBAT_B_On();
		}
		else
		{
			mBAT_A_Off();
			mBAT_B_Off();
		}
	}
}
