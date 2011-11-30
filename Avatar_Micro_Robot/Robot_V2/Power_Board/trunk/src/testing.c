#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "i2c.h"
#include "interrupt_switch.h"
#include "testing.h"


void switched_sensor_wires(void);
void pulse_power_bus(void);

//gets called after board is initialized.=
void test_function(void)
{
	unsigned int i;

	motor_stress_test();

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

	OC1R=2000;
	OC2R=2000;
	while(1);


/*		for(i=0;i<=2000;i+=100)
		{

			OC1R = i;

			block_ms(100);
			ClrWdt();
		}*/


	REG_MOTOR_VELOCITY.right=1000;
	REG_MOTOR_VELOCITY.left=500;
//	REG_MOTOR_VELOCITY.flipper=1000;
	//motor_stress_test();
	//switched_sensor_wires();
	//pulse_power_bus();


}

//try to drive the motors as hard as possible
//without killing the battery.
void motor_stress_test(void)
{

	unsigned int i;
	unsigned char direction_flag = 0;
	block_ms(5000);

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
	/*	M2_DIR=LO;
		OC1R=2000;
		OC2R=2000;
		block_ms(1000);
		OC1R=0;
		OC2R=0;
		block_ms(2000);*/
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
			block_ms(30);
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

		//for 2 motors installed, with tracks, 10ms doesn't reset, 20ms does, 15ms does
		block_ms(20);
		OC1R = 0;
		OC2R = 0;
		OC3R = 0;


	}


}



void pulse_power_bus(void)
{

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

	OC1R = 0;
	OC2R = 0;
	OC3R = 0;

	block_ms(1000);

	OC1R = 2000;
	OC2R = 2000;
	OC3R = 2000;

	while(1)
	{
		Cell_Ctrl(Cell_A,Cell_ON);
		Cell_Ctrl(Cell_B,Cell_ON);
	
		block_ms(15);

		Cell_Ctrl(Cell_A,Cell_OFF);
		Cell_Ctrl(Cell_B,Cell_OFF);

		block_ms(1000);


	}


}

void switched_sensor_wires(void)
{
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

		OC1R = 2000;
		OC2R = 2000;
		OC3R = 2000;
	

		//10ms doesn't kill the battery, but 20ms does
		block_ms(10);
	
		OC1R = 0;
		OC2R = 0;
		OC3R = 0;
	
		block_ms(60);

	}	
	

}


