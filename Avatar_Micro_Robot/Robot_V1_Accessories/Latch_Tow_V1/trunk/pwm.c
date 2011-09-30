#include "system.h"

int servo_lockout = 0;

void init_pwm(void)
{

	T2CONbits.TCKPS =  0x01;	//prescale by 8
T2CONbits.TON = 1;
	//PR2 = .020*(SYSCLK/2)/T2_PRESCALE - 1;	//24999
	PR2 = 24999;
//	OC1RS = PR2*.9;
	//OC1R = 37000+0*10;
	//OC1R = 40000*(1-.075);			// sets to neutral
	//OC1RS = 40000;	//Period = [PRy+1]*Tcy*TimerPrescale
	//OC1RS = 1875;
	OC1RS = 1875;
	//OC1R = 20000;
	//40000 -> 25000
	//37000 -> 23125
	//OC1RS = (PR2+1)*T2_PRESCALE
	//OC1CON2bits.SYNCSEL = 0x1F;
	OC1CONbits.OCTSEL = 0; 				// Selects System Clock
	OC1CONbits.OCM = 0b110;


}

void set_motor_effort(int effort)
{


	//OC1RS = 37000+effort*10;
	OC1RS = 1875+6.25*effort;

}

void open_hitch(void)
{
	//OC1RS = 2500;
	//T2CONbits.TON = 1;
	MC_PHASE = 1;
	
}

void turn_off_servo_signal(void)
{

	T2CONbits.TON = 0;

}

void close_hitch(void)
{
/*	OC1RS = 1250;
	T2CONbits.TON = 1;
	//servo_lockout = 5;
	block_ms(500);
	T2CONbits.TON = 0;*/
	MC_PHASE = 0;

}