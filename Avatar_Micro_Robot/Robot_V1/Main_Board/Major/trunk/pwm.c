
#include "system.h"

/*****************************************************************************
 * Function: Initialize PWM System
 *****************************************************************************/
void PWMInit(void)
{
//0. Confirgure pin directions for PWM Pins
	//Done in iomapping.c

//1. Configure the OCx output for one of the available Peripheral Pin Select pins.
	RPOR3bits.RP6R = OC1_IO;	
	RPOR3bits.RP7R = OC2_IO;
	RPOR4bits.RP8R  = OC3_IO;

//2. Calculate the desired duty cycles and load them into the OCxR register.
	// 1.5 ms duty cycle
	OC1R = 40000*(1-.075);			// sets to neutral
	OC2R = 40000*(1-.075);
	OC3R = 40000*(1-.075);

//3. Calculate the desired period and load it into the OCxRS register.
	OC1RS = 40000;	//Period = [PRy+1]*Tcy*TimerPrescale
	OC2RS = 40000;
	OC3RS = 40000;

//4. Select the current OCx as the trigger/sync source by writing 0x1F to SYNCSEL<4:0> (OCxCON2<4:0>).
	OC1CON2bits.SYNCSEL = 0x1F;
	OC2CON2bits.SYNCSEL = 0x1F;
	OC3CON2bits.SYNCSEL = 0x1F;

//5. Select a clock source by writing the OCTSEL2<2:0> (OCxCON<12:10>) bits.
	OC1CON1bits.OCTSEL = 0b100; 				// Selects System Clock
	OC2CON1bits.OCTSEL = 0b100;
	OC3CON1bits.OCTSEL = 0b100;

//6. Enable interrupts, if required, for the timer and output compare modules. The output compare interrupt is required for PWM Fault pin utilization.
	//Interrupts not required	

//7. Select the desired PWM mode in the OCM<2:0> (OCxCON1<2:0>) bits.
	OC1CON1bits.OCM = 0b111;
	OC2CON1bits.OCM = 0b111;
	OC3CON1bits.OCM = 0b111;

//8. If a timer is selected as a clock source, set the TMRy prescale value and enable the time base by setting the TON (TxCON<15>) bit.
	//Timer is timer 1, setup in timer.c
}



