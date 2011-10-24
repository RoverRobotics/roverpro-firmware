#include "system.h"

unsigned int T3_Interrupt_Count;
unsigned int last_T3_Interrupt_Count;
unsigned int Current_Interrupt_Count = 0;
unsigned int Jumpiness_Interrupt_Count = 0;
int Max_Speed = 25;

void speed_control_loop_quadrants(char motors_off)
{
	static int PI_motor_effort[3] = {0,0,0};

	int desired_effort[3];

	static int E_speed[3] = {0,0,0};
	
	unsigned int i;

	//don't run this speed control loop if there is an arm
	//installed, and it is working.  This allows for a separate
//	control loop when the arm is being used.
	if(arm_in_use)
		return;

	for(i=0;i<3;i++)
	{
		desired_effort[i] = Return_Desired_Motor_Effort_Quadrants(i);

		//enforce a deadband
		if(abs(desired_effort[i]) <= 1)
			desired_effort[i] = 0;
		E_speed[i] = desired_effort[i] - PI_motor_effort[i];

	}



	#ifdef NO_ACCELERATION_CURVE
		for(i=0;i<2;i++)
		{
			set_motor_effort(desired_effort[i],i);
		
		}

		i = FLIPPER_MOTOR;

		if(desired_effort[i]-PI_motor_effort[i] > 0)
			speed_interval_sign[i] = 1;
		else if(desired_effort[i]-PI_motor_effort[i] < 0)
			speed_interval_sign[i] = -1;
		else
			speed_interval_sign[i] = 0;



			PI_motor_effort[i] = PI_motor_effort[i] + speed_interval[i]*speed_interval_sign[i];
		if(PI_motor_effort[i] > 50)
			PI_motor_effort[i] = 50;
		else if(PI_motor_effort[i] < -50)
			PI_motor_effort[i] = -50;

		if(abs(PI_motor_effort[i]) < speed_interval[i])
			PI_motor_effort[i] = 0;
		
		set_motor_effort(PI_motor_effort[i],i);
		return;

	#endif

//	for(i=0;i<2;i++)
//	PI_motor_effort[i] += .2*(float)E_speed[i];

	for(i=0;i<2;i++)
	{	
		//This is to ramp up the speed, since the Novak controllers jerk if the speed goes from 0 to something that will move the robot (such as 15)
		if(abs(PI_motor_effort[i]) <= 15)
		{
			if(E_speed[i] > 0 && desired_effort[i] > 0)
				PI_motor_effort[i] += 5;
			else if(E_speed[i] < 0 && desired_effort[i] < 0)
				PI_motor_effort[i] -= 5;
			else if(desired_effort[i] == 0)
				PI_motor_effort[i] = 0;
	
	
		}
		else
		{
			PI_motor_effort[i] += .2*(float)E_speed[i];
		}
	}

	PI_motor_effort[FLIPPER_MOTOR] += .2*E_speed[FLIPPER_MOTOR];


	if(((PI_motor_effort[LEFT_MOTOR] > 0) && (PI_motor_effort[RIGHT_MOTOR] < 0)) || ((PI_motor_effort[LEFT_MOTOR] < 0) && (PI_motor_effort[RIGHT_MOTOR] > 0)))
	{
		for(i=0;i<2;i++)
		{
			if(PI_motor_effort[i] > 35)
				PI_motor_effort[i] = 35;
			else if(PI_motor_effort[i] < -35)
				PI_motor_effort[i] = -35;
		}
	}

	




	//set limits for all motors
	for(i=0;i<3;i++)
	{
		if(PI_motor_effort[i] > Max_Speed)
			PI_motor_effort[i] = Max_Speed;
		else if(PI_motor_effort[i] < -Max_Speed)
			PI_motor_effort[i] = -Max_Speed;

		

	}

	//set speeds for drive motors
	for(i=0;i<2;i++)
		set_motor_effort(PI_motor_effort[i],i);

	handle_flipper(PI_motor_effort[i]);
	




}



/*void speed_control_loop_quadrants_brushed(char motors_off)
{
	static int PI_motor_effort[3] = {0,0,0};

	int desired_effort[3];

	static int E_speed[3] = {0,0,0};
	
	static unsigned int stop_counter = 0;

	int direction_change_threshold = 3;
	int speed_interval[3] = {4,4,10};
	int speed_interval_stall[3] = {2,2,2};

	static int last_max_speed[3] = {0,0,0};
	static int last_max_speed_counter[3] = {0,0,0};

	unsigned int stop_counter_max = 0;


	static int speed_interval_sign[3] = {0,0,0};
	
	static char stop_flag = 0;
	unsigned int i;
	static char last_positive[3] = {1,1,1};
	static int speed_interval_ratio[3] = {100,100,100};



	for(i=0;i<3;i++)
	{
		desired_effort[i] = Return_Desired_Motor_Effort_Quadrants(i);

		//enforce a deadband
		if(abs(desired_effort[i]) <= 3)
			desired_effort[i] = 0;

	}



	#ifdef NO_ACCELERATION_CURVE
		for(i=0;i<2;i++)
		{
			set_motor_effort(desired_effort[i],i);
		
		}

		i = FLIPPER_MOTOR;

		if(desired_effort[i]-PI_motor_effort[i] > 0)
			speed_interval_sign[i] = 1;
		else if(desired_effort[i]-PI_motor_effort[i] < 0)
			speed_interval_sign[i] = -1;
		else
			speed_interval_sign[i] = 0;



			PI_motor_effort[i] = PI_motor_effort[i] + speed_interval[i]*speed_interval_sign[i];
		if(PI_motor_effort[i] > 100)
			PI_motor_effort[i] = 100;
		else if(PI_motor_effort[i] < -100)
			PI_motor_effort[i] = -100;

		if(abs(PI_motor_effort[i]) < speed_interval[i])
			PI_motor_effort[i] = 0;
		
		set_motor_effort(PI_motor_effort[i],i);
		return;

	#endif


	if(abs(desired_effort[RIGHT_MOTOR]) > 50 && abs(desired_effort[LEFT_MOTOR]) > 50)
	{
		if(abs(desired_effort[LEFT_MOTOR]) > abs(desired_effort[RIGHT_MOTOR]))
		{
			if(desired_effort[LEFT_MOTOR] != 0 && desired_effort[RIGHT_MOTOR] != 0)
				speed_interval_ratio[RIGHT_MOTOR] = abs(100*desired_effort[RIGHT_MOTOR]/desired_effort[LEFT_MOTOR]);
			else
				speed_interval_ratio[RIGHT_MOTOR] = 100;
			speed_interval_ratio[LEFT_MOTOR] = 100;
			
		}
		else
		{
			if(desired_effort[RIGHT_MOTOR] != 0 && desired_effort[LEFT_MOTOR] != 0)
				speed_interval_ratio[LEFT_MOTOR] = abs(100*desired_effort[LEFT_MOTOR]/desired_effort[RIGHT_MOTOR]);
			else
				speed_interval_ratio[LEFT_MOTOR] = 100;
			speed_interval_ratio[RIGHT_MOTOR] = 100;
		}
	}
	else
	{
		speed_interval_ratio[LEFT_MOTOR] = 100;
		speed_interval_ratio[RIGHT_MOTOR] = 100;
	}





	//if a motor is changing directions, slow down until both motors are stopped.
	if(stop_flag)
	{
		desired_effort[RIGHT_MOTOR] = 0;
		desired_effort[LEFT_MOTOR] = 0;


		stop_counter++;


		if((abs(last_max_speed[LEFT_MOTOR])/last_max_speed_counter[LEFT_MOTOR] < 8) || abs(last_max_speed[RIGHT_MOTOR])/last_max_speed_counter[RIGHT_MOTOR] < 8)
			stop_counter_max = 0;
		else
			stop_counter_max = 100;

		if(stop_counter >= stop_counter_max)
		{

			stop_flag = 0;
			stop_counter = 0;
			last_positive[LEFT_MOTOR] = 2;
			last_positive[RIGHT_MOTOR] = 2;



		}


	}
	else
	{
		for(i=0;i<2;i++)
		{
	
			//check if any given motor is switching direction
			if((desired_effort[i] >= direction_change_threshold && (last_positive[i] == 0)) || (desired_effort[i] <= -direction_change_threshold && (last_positive[i] == 1)))
			{
				desired_effort[RIGHT_MOTOR] = 0;
				desired_effort[LEFT_MOTOR] = 0;

	
				//set flag so that we know that we should stop
				stop_flag = 1;

				last_positive[LEFT_MOTOR] = 2;
				last_positive[RIGHT_MOTOR] = 2;

			}


		//start off moving
			if(desired_effort[i] > 2 && PI_motor_effort[i] >= 0 && PI_motor_effort[i] <=25)
				PI_motor_effort[i] = 25;
			else if(desired_effort[i] < -2 && PI_motor_effort[i] <= 0 && PI_motor_effort[i] >= -25)
				PI_motor_effort[i] = -25;
		
	
		}


	}

	for(i=0;i<3;i++)
	{
		E_speed[i] = desired_effort[i] - PI_motor_effort[i];

		if(E_speed[i] > 0)
			speed_interval_sign[i] = 1;
		else if(E_speed[i] < 0)
			speed_interval_sign[i] = -1;
		else
			speed_interval_sign[i] = 0;

	}





	if(stop_flag == 0)
	{
		for(i=0;i<2;i++)
		{
			//decelerating from positive
			if(PI_motor_effort[i] > 0 && speed_interval_sign[i] < 0 && desired_effort[i] >= 0)
			
			{


				last_max_speed[i] = PI_motor_effort[i];
				last_max_speed_counter[i] = 1;

				PI_motor_effort[i] = desired_effort[i];
		
		
			}
			//decelerating from negative
			else if(PI_motor_effort[i] < 0 && speed_interval_sign[i] > 0 && desired_effort[i] <= 0)
			{
				last_max_speed[i] = PI_motor_effort[i];
				last_max_speed_counter[i] = 1;

				PI_motor_effort[i] = desired_effort[i];
		
			}
		}
	}
	else
	{
		for(i=0;i<2;i++)
			last_max_speed_counter[i]++;

	}




	//only applies to the right and left motors
	


	for(i=0;i<2;i++)
	{
	
		if(stop_flag)
		{

			PI_motor_effort[i] = 0;
		}
		else
		{
			if((abs(PI_motor_effort[LEFT_MOTOR]) < 10) || (abs(PI_motor_effort[RIGHT_MOTOR]) < 10))
			{

				PI_motor_effort[i] += speed_interval_stall[i]*speed_interval_sign[i]*speed_interval_ratio[i]/100;
				
			}
			else
			{
				PI_motor_effort[i] += speed_interval[i]*speed_interval_sign[i]*speed_interval_ratio[i]/100;
			}
		}
		

		if(stop_flag == 0)
		{
			if(desired_effort[i] > 5 && PI_motor_effort[i] >= 0 && desired_effort[i] < 25)
				PI_motor_effort[i] = 25;
			if(desired_effort[i] < -5 && PI_motor_effort[i] <= 0 && desired_effort[i] > -25)
				PI_motor_effort[i] = -25;
		}



		//if robot is turning
		if((PI_motor_effort[LEFT_MOTOR] > 0 && PI_motor_effort[RIGHT_MOTOR] < 0 ) || (PI_motor_effort[LEFT_MOTOR] < 0 && PI_motor_effort[RIGHT_MOTOR] > 0 ))
		{
			if(PI_motor_effort[i] > 50)
				PI_motor_effort[i] = 50;
			if(PI_motor_effort[i] < -50)
				PI_motor_effort[i] = -50;


		}
		//if robot is not turning
		else
		{	
			if(PI_motor_effort[i] > 100)
				PI_motor_effort[i] = 100;
			if(PI_motor_effort[i] < -100)
				PI_motor_effort[i] = -100;
		}



		set_motor_effort(PI_motor_effort[i],i);



		if(PI_motor_effort[i] > 2)
			last_positive[i] = 1;
		else if(PI_motor_effort[i] < -2)
			last_positive[i] = 0;
		else
			last_positive[i] = 2;

	}


	PI_motor_effort[FLIPPER_MOTOR] += speed_interval[i]*speed_interval_sign[i];
	set_motor_effort(PI_motor_effort[FLIPPER_MOTOR],FLIPPER_MOTOR);

	if(motors_off)
	{
		for(i=0;i<3;i++)
		{
			PI_motor_effort[i] = 0;
			
		}

	}


}*/



void init_current_limit_atod(void)
{

	AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;
	AD1CHS = 0x0000;

	AD1PCFGL = 0xFFFF;
//	AD1PCFGbits.PCFG3 = 0;
//	AD1PCFGbits.PCFG4 = 0;
//	AD1PCFGbits.PCFG5 = 0;
	AD1PCFGLbits.PCFG12 = 0;
	AD1PCFGLbits.PCFG11 = 0;
	

	AD1CON1bits.SSRC = 0b111;	//auto-convert

	AD1CON3 = 0x1F02;

	AD1CSSL = 0;
	AD1CON1bits.ADON = 1;
	







}




void init_timer3(void)
{

	//set prescale to 1:8
	T3CONbits.TCKPS = 0b01;

	//Get timer 2 clock from Fosc/2
	T3CONbits.TCS = 0;


	IFS0bits.T3IF = 0;	//clear interrupt flag
	IEC0bits.T3IE = 1;	//enable timer 2 interrupt

	PR3 = .002*(SYSCLK/2)/T3_PRESCALE - 1;

	T3CONbits.TON = 1;






}



/*void init_a_to_d(void)
{

	AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;
	AD1CHS0 = 0x0000;
	AD1CSSL =  0;
	AD1PCFG = 0xFFFF;
	AD1PCFGbits.PCFG3 = 0;
	AD1PCFGbits.PCFG4 = 0;
	AD1PCFGbits.PCFG5 = 0;
	
	AD1CON1 = 0x0000;

	AD1CON1bits.SSRC = 0b111;	//auto-convert

	AD1CON3 = 0x1F02;
	AD1CON1bits.ADON = 1;
	







}*/


unsigned int get_ad_value(int channel)
{

	unsigned int current_ad_value = 0;

	AD1CON1bits.ADON = 0;
	//AN3,4,5
	switch(channel)
	{
		case FLIPPER_MOTOR:
			AD1CHS = 3;
			break;
		case RIGHT_MOTOR:
			AD1CHS = 4;
			break;
		case LEFT_MOTOR:
			AD1CHS = 5;
			break;
	}


	AD1CON1bits.ADON = 1;
	Nop();
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE);
	current_ad_value =  ADC1BUF0;

	return current_ad_value;





}

float get_motor_current(void)
{
	unsigned int total_ad_value = 0;

	total_ad_value = get_ad_value(FLIPPER_MOTOR);

	return (float)total_ad_value/CURRENT_DIVISOR + CURRENT_OFFSET;


}

float average_motor_current(unsigned int reset)
{

	static float flipper_current_max = 0;
	float flipper_current_temp;

	if(reset==YES)
	{

		flipper_current_temp = flipper_current_max;
		flipper_current_max = 0;
		return flipper_current_temp;

	
	}
	else
	{
		flipper_current_temp = get_motor_current();
		if(flipper_current_temp > flipper_current_max)
			flipper_current_max = flipper_current_temp;
	}
	return 0;

}





void  __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
{

		IFS0bits.T3IF = 0;	//clear interrupt flag
		T3_Interrupt_Count++;
		Current_Interrupt_Count++;
		Jumpiness_Interrupt_Count++;

}
