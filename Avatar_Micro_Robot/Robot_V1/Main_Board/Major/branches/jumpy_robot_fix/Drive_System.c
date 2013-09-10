#include "system.h"


float Left_Motor_Pulse = 150;
float Right_Motor_Pulse = 150;
float Flipper_Motor_Pulse = 150;

int Mixed_Right = 0;
int Mixed_Left = 0;
int Mixed_Flipper = 0;





void Set_Desired_Motor_Pulse_Quadrants(unsigned char Direction, unsigned char Speed, unsigned char Flipper_On, unsigned char Flipper_Dir)

{

	int Direction_offset;
	int Direction_Exponential;
	int Speed_offset;
	int Speed_Exponential;
	int Magnitude;
	float Tangent;
	float Angle;
	int Direction_sign = -1;
	int Speed_sign = -1;


	//we flip the direction offset because the horizontal joystick direction is flipped
	Direction_offset = -((int)Direction*10-127.5*10)/10;
	Speed_offset = (Speed*10-127.5*10)/10;

	Direction_offset*=1.4;
	Speed_offset*=1.4;

	if(Direction_offset > 127)
		Direction_offset = 127;
	else if(Direction_offset <-127)
		Direction_offset = -127;

	if(Speed_offset > 127)
		Speed_offset = 127;
	else if(Speed_offset < -127)
		Speed_offset = -127;



	if(Direction_offset > 0)
		Direction_sign = 1;
	if(Speed_offset > 0)
		Speed_sign = 1;


	//exponential control

	Direction_Exponential = ((pow(Direction_offset,2)/127.5))*100/127.5*Direction_sign;
	Speed_Exponential = ((pow(Speed_offset,2)/127.5))*100/127.5*Speed_sign;


	//proportional control
	//Direction_Exponential = Direction_offset;
	//Speed_Exponential = Speed_offset;

	if(Direction_Exponential > 100)
		Direction_Exponential = 100;
	if(Direction_Exponential < -100)
		Direction_Exponential = -100;

	if(Speed_Exponential > 100)
		Speed_Exponential = 100;
	if(Speed_Exponential < -100)
		Speed_Exponential = -100;
	
	

	if(Direction_Exponential == 0)
	{
		if(Speed_Exponential > 0)
			Angle = 90;
		else
			Angle = -90;

		//not used in this case
		Tangent = 100;

	}
	else
	{
		Tangent = Speed_Exponential/Direction_Exponential;
		Angle = 57.3*(float)atan((float)Speed_Exponential/(float)Direction_Exponential);
	}


	
	Magnitude = sqrt(Speed_Exponential*Speed_Exponential+Direction_Exponential*Direction_Exponential);

	#ifdef QUADRANT_MIXING
		//|Tan| = 1, 45 deg
		//|Tan| = .4 @ 21.8 deg
		//full turning
		//if(Tangent > -.4 && Tangent < .4)
		if((Angle > -20) && (Angle < 20))
		{
			//right turn
			if(Direction_offset > 0)
			{
				Mixed_Right = -Magnitude;
				Mixed_Left = Magnitude;
			}
			//left turn
			else
			{
				Mixed_Right = Magnitude;
				Mixed_Left = -Magnitude;
			}	
	
	
		}
		//otherwise, straight mode
		else
		{
	
			//going straight
			if(Speed_Exponential > 0)
			{
				//forward, right adjustment
				if(Direction_Exponential >= 0)
				{	
					Mixed_Left = Magnitude;
	
	
					Mixed_Right = Magnitude*(Angle-MIN_ANGLE)/(90-MIN_ANGLE)*(1-MIN_EFFORT)+Magnitude*MIN_EFFORT;
	
	
				}
				//forward, left adjustment
				//in this quadrant, atan return -90 for straight forward, and -45 for a left correction
				else
				{
					
					Mixed_Left = Magnitude*(-Angle-MIN_ANGLE)/(90-MIN_ANGLE)*(1-MIN_EFFORT)+Magnitude*MIN_EFFORT;
					Mixed_Right = Magnitude;
	
	
	
	
				}
	
	
			}
	
			//reverse
			else
			{
				Mixed_Right = -Magnitude;
				Mixed_Left = -Magnitude;
	
			}
	
	
		}
	#endif //end quadrant mixing

	#ifdef NO_QUADRANT_MIXING
	
	
		if(Speed_Exponential >= 0)
		{
			//first quadrant
			if(Direction_Exponential >= 0)
			{
				Mixed_Left = Magnitude;
				Mixed_Right = Magnitude*(Angle-45)/45;

			}
			//second quadrant
			else
			{
				Mixed_Right = Magnitude;
				Mixed_Left = -Magnitude*(Angle+45)/45;
			}
		}
		else
		{
			//fourth quadrant
			if(Direction_Exponential >= 0)
			{
				Mixed_Right = -Magnitude;
				Mixed_Left = Magnitude*(Angle+45)/45;

			}
			//third quadrant
			else
			{
				Mixed_Left = -Magnitude;
				Mixed_Right = Magnitude*(-Angle+45)/45;
			}
		}

	#endif	//end no quadrant mixing
			
	if(Flipper_On == 1)
	{
		if(Flipper_Dir == 1)	//flippers up -- direction may be backwards
		{
			Mixed_Flipper = -100;
		}
		else if(Flipper_Dir == 0)
		{
			Mixed_Flipper = 100;
		}
		else
		{
			Mixed_Flipper = 0;

		}
	}
	else
	{
		Mixed_Flipper = 0;
	}



}



int Return_Desired_Motor_Effort_Quadrants(unsigned char motor)
{

	int return_value = 0;
	switch(motor)
	{
		case LEFT_MOTOR:
			return_value = Mixed_Left;
			break;
		case RIGHT_MOTOR:
			return_value = Mixed_Right;
			break;
		case FLIPPER_MOTOR:
			return_value = Mixed_Flipper;
			break;
	}

	if(return_value > 100) return_value = 100;
	else if(return_value < -100) return_value = -100;
	return return_value;

}


void set_motor_effort(int effort,unsigned char motor)
{
	int OCxR;

	//I may have gotten this wrong -- 100% should be forward either way.
	//2 ms (100%) is 36000, 1.5ms (-100%) is 37000, 1ms is 38000


	if(motor == LEFT_MOTOR)
		OCxR = 37000+effort*10;
	else
		OCxR = 37000-effort*10;

	//for other motors
	//OCxR = 37000-effort*10;

	switch(motor)
	{
		case LEFT_MOTOR:
			OC2R = OCxR;
			break;
		case RIGHT_MOTOR:
			OC3R = OCxR;
			break;
		case FLIPPER_MOTOR:
			OC1R = OCxR;
			break;
	}



}

