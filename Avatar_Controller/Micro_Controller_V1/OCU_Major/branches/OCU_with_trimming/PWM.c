#include "PWM.h"
#include "include/Compiler.h"

void PWM_Init(void)
{
	//pg. 164 - OCx output

	// PIN MAPPING AND INITIALIZATION
	RPOR14bits.RP28R = 18;		//set RP28 to OC1, function number 18
//	TRISBbits.TRISB4 = 0;		//B4 as output

	RPOR9bits.RP18R = 19;		//set RP18 to OC2, function number 19
//	TRISBbits.TRISB5 = 0;		//B5 as output
	
//	ODCBbits.ODB4 = 1;			//Set Servo line 1 as Open-drain
//	ODCBbits.ODB5 = 1;			//Set Servo line 2 as Open-drain

//	TRISBbits.TRISB2 = 0;		//RB2 line controls servos
	LATBbits.LATB2 = 1;			//Initialize Servo Signal line ON

//	TRISFbits.TRISF3 = 0;		//RF3 to Output to control Relay on/off
	LATFbits.LATF3 = 1;			//Initialize Relay ON


	//duty cycle in OCxR -- specifies time low
	// 1ms = (1999+1)*2/32MHz*8
	//39999 - 1999 = 38000
	//39999 - 2*1999 = 36001
	//on oscope -- closer to .9ms -- maybe clock is actually running at 36MHz?
	OC1R = 36001;
	OC2R = 36001;

	//period in OC1RS -- or should period go in PR2?
	OC1RS = 0x9C3F;	//20ms = ((39999+1)*2/32MHz*8
	OC2RS = 0x9C3F;
	
	//Select OC1 as trigger/sync source
	OC1CON2bits.SYNCSEL = 0x1F;
	OC2CON2bits.SYNCSEL = 0x1F;

	//Clock source
	// 0 = TMR2
	//7 = system clock
	OC1CON1bits.OCTSEL2 = 0;
	OC2CON1bits.OCTSEL2 = 0;

	//PWM mode
	// 7 = Center-aligned PWM
	// 6 = Edge-aligned PWM
	OC1CON1bits.OCM = 7;
	OC2CON1bits.OCM = 7;

	//PWM Period = ((PRy+1)*Tcy*Prescaler)
	//PRY = PWM Period / (Tcy*Prescaler) - 1
	//PRy = 20ms / (2/32MHz*8) - 1 = 39,999
	//I don't think that the value of PR2 changes anything
	PR2 = 0xFFF;

	//1 = 1:8 prescale
	//0 = 1:1 prescale
	T2CONbits.TCKPS = 1;

	//16383
	//turn on timer 2
	T2CONbits.TON = 1;
}

void PWM_Test(void)
{

	unsigned int i;
	unsigned int j;
	PWM_Init();
	
	while(1)
	{

//		Set_PWM_Duty_Cycle(PWM_channel_1,100);
//		Set_PWM_Duty_Cycle(PWM_channel_2,100);

		//block for 1 s
		for(i=0;i<3200;i++)	
		{
			for(j=0;j<1000;j++);
		}

//		Set_PWM_Duty_Cycle(PWM_channel_1,200);
//		Set_PWM_Duty_Cycle(PWM_channel_2,200);

		//block for 1s
		for(i=0;i<3200;i++)	
		{
			for(j=0;j<1000;j++);
		}
	}
}

void Set_PWM_HIGH_TIME(unsigned char channel, unsigned int duty_cycle_100X)
{
	if(channel == PWM_channel_1)
	{
		//duty_cycle_100X is 100X the duty cycle (i.e. for 1.52ms, duty_cycle_100x = 152)
		//factor of 23 found experimentally -- it is within 5% of being correct.
		//Analytical multiplier should be 19.99
		OC1R = 39999 - 20*duty_cycle_100X;
	}
	else if (channel == PWM_channel_2)
	{
		OC2R = 39999 - 20*duty_cycle_100X;
	}
}
