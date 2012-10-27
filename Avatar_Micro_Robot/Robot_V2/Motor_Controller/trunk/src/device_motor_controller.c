
#include "stdhdr.h"
#include "device_motor_controller.h"
#include "testing.h"

#define AHI_1_EN(a) (_TRISD0 = !a)
#define ALO_1_EN(a) (_TRISD1 = !a)
#define BHI_1_EN(a) (_TRISD2 = !a)
#define BLO_1_EN(a) (_TRISD3 = !a)
#define CHI_1_EN(a) (_TRISD4 = !a)
#define CLO_1_EN(a) (_TRISD5 = !a)

#define AHI_1_ON(a) (_LATD0 = a)
#define ALO_1_ON(a) (_LATD1 = a)
#define BHI_1_ON(a) (_LATD2 = a)
#define BLO_1_ON(a) (_LATD3 = a)
#define CHI_1_ON(a) (_LATD4 = a)
#define CLO_1_ON(a) (_LATD5 = a)

//RP23
#define BHI_1_OR _RP23R
#define CHI_1_OR _RP25R


static void init_pwm(void);
static void set_pwm_duty(unsigned int duty_cycle);

void Device_Motor_Controller_Process_IO(void)
{

  unsigned int i;

  //for some reason, low side must go high to charge back up
  //high side gate driver

  AHI_1_ON(1);
  //set_pwm_duty(50);
  block_ms(2000);
  AHI_1_ON(0);
//ALO_1_ON(1);
  //set_pwm_duty(0);
  block_ms(2000);
//ALO_1_ON(0);
  //BLO_1_ON(0);




}

void Device_Motor_Controller_Init(void)
{

  AHI_1_ON(0);
  ALO_1_ON(0);
  BHI_1_ON(0);
  BLO_1_ON(0);
  CHI_1_ON(0);
  CLO_1_ON(0);

  AHI_1_EN(1);
  ALO_1_EN(1);
  BHI_1_EN(1);
  BLO_1_EN(1);
  CHI_1_EN(1);
  CLO_1_EN(1);

  //set_pwm_duty(0);

  block_ms(5000);

  //init_pwm();


}

static void init_pwm(void)
{

	//Choose 400Hz (2.5ms period)
	//2.5ms = [PR2 + 1]*62.5ns*1
  //CHI_1_OR = 18; //OC1
  BHI_1_OR = 18;
	PR2 = 40001;	
	OC1RS = 4000;


	OC1CON2bits.SYNCSEL = 0x1f;	
	//use timer 2

	OC1CON1bits.OCTSEL2 = 0;

	//edge-aligned pwm mode
	OC1CON1bits.OCM = 6;

	//turn on timer 2
	T2CONbits.TON = 1;


}

static void set_pwm_duty(unsigned int duty_cycle)
{
	if(duty_cycle > 100)
		duty_cycle = 100;
  OC1R = duty_cycle*40;

}
