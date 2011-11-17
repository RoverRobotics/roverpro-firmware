#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "i2c.h"
#include "interrupt_switch.h"


//gets called after board is initialized.=
void test_function(void)
{



}

//try to drive the motors as hard as possible
//without killing the battery.
void motor_stress_test(void)
{

	unsigned int i;
	unsigned char direction_flag = 0;

	Cell_Ctrl(Cell_A,Cell_ON);
	Cell_Ctrl(Cell_B,Cell_ON);

	M1_COAST=Clear_ActiveLO;
	M1_DIR=HI;
	M1_BRAKE=Clear_ActiveLO;
	M1_MODE=1;

	M2_COAST=Clear_ActiveLO;
	M2_DIR=HI;
	M2_BRAKE=Clear_ActiveLO;
	M2_MODE=1;

	M3_COAST=Clear_ActiveLO;
	M3_DIR=HI;
	M3_BRAKE=Clear_ActiveLO;
	M3_MODE=1;

	while(1)
	{

		if(direction_flag)
		{
			direction_flag=0;
			M1_DIR=HI;
			M2_DIR=HI;
			M3_DIR=HI;
		}
		else
		{
			direction_flag = 1;
			M1_DIR=LO;
			M2_DIR=LO;
			M3_DIR=LO;
		}
		
		
		for(i=0;i<=2000;i+=100)
		{
			OC1R = i;
			OC2R = i;
			OC3R = i;
			block_ms(10);
			ClrWdt();
		}

		if(direction_flag)
		{
			M1_DIR=HI;
			M2_DIR=HI;
			M3_DIR=HI;
		}
		else
		{
			M1_DIR=LO;
			M2_DIR=LO;
			M3_DIR=LO;
		}

		block_ms(500);
		OC1R = 0;
		OC2R = 0;
		OC3R = 0;


	}


}
