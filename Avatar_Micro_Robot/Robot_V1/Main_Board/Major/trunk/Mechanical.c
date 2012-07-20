#include "system.h"

//---------------------------- Motors ----------------------------------
// Motors receive analog control from the controller
// Set PWM1 to the Direction value
// Set PWM2 to the Speed value
//----------------------------------------------------------------------
void Motors(unsigned char Command,unsigned char Mode,unsigned char Direction,unsigned char Speed)
{
	unsigned int Direction_Pulse;
	unsigned int Speed_Pulse;
	float Scaled_Speed;
	float Scaled_Direction;
	float Left_Motor_Pulse;
	float Theta;
	float Vector;
	float Max_Pulse;
	float Right_Motor_Pulse;
	int Exp_Speed;
	int Exp_Direction;

	// Handle on/off variable pass
	switch (Command)
	{
	case ON:
		Motor_pin = ON;
		break;
	case OFF:
		Motor_pin = OFF;
		break;
	}

	// scale turn commands if we should
	if (STEER_MODE == EXPONENTIAL){						// Takes linear scale commands and puts them on exponential scale
		// Handle Left/Right Value
		if (Direction > CENTER){
			Exp_Direction = Direction - CENTER;
			Exp_Direction = (.00787)*pow(Exp_Direction,2);
			Exp_Direction = Exp_Direction + CENTER;
			if (Exp_Direction < CENTER)
				Exp_Direction = 127;
			else if (Exp_Direction > 255)
				Exp_Direction = 255;
			Direction = Exp_Direction;
		}
		else if (Direction < CENTER){
			Direction = 255 - Direction; 				// Flip the values
			Exp_Direction = Direction - CENTER;
			Exp_Direction = (.00787)*pow(Exp_Direction,2);
			Exp_Direction = Exp_Direction + CENTER;
			if (Exp_Direction < CENTER)
				Exp_Direction = 128;
			if (Exp_Direction > 255)
				Exp_Direction = 255;
			Direction = 255 - Exp_Direction;			// Unflips the values
		}
		if ((Direction < (CENTER + STEER_DEADZONE))&&(Direction > (CENTER - STEER_DEADZONE)))
			Direction = 127;							// Lock the Deadzone to eliminate stall
		// Handle Up/Down Value
		if (Speed > CENTER){
			Exp_Speed = Speed - CENTER;
			Exp_Speed = (.00787)*pow(Exp_Speed,2);
			Exp_Speed = Exp_Speed + CENTER;
			if (Exp_Speed < CENTER)
				Exp_Speed = 127;
			else if (Exp_Speed > 255)
				Exp_Speed = 255;
			Speed = Exp_Speed;
		}
		else if (Speed < CENTER){
			Speed = 255 - Speed;
			Exp_Speed = Speed - CENTER;
			Exp_Speed = (.00787)* pow(Exp_Speed,2);
			Exp_Speed = Exp_Speed + CENTER;
			if (Exp_Speed < CENTER)
				Exp_Speed = 128;
			else if (Exp_Speed > 255)
				Exp_Speed = 255;
			Speed = 255 - Exp_Speed;				
		}
		if ((Speed < (CENTER + STEER_DEADZONE))&&(Speed > (CENTER - STEER_DEADZONE)))
			Speed = 127;
	}
			
			
	// Incoming now is joystick up 255, joystick down 0, joystick left 255, joystick right 0
	//Quadrant 1
	if ((Speed >= CENTER)&&(Direction <= CENTER)){ 											// Quadrant 1 criteria, Left Motor goes from 150 to 200 based on vector length
	//Left
		Scaled_Speed = ((float)Speed - CENTER)/2.54;										// Speed goes 0-50
		Scaled_Direction = (CENTER - (float)Direction)/2.54;								// Throttle goes 0-50
		Vector = sqrt((Scaled_Speed*Scaled_Speed) + (Scaled_Direction*Scaled_Direction)); 	// Find the additional pulse for the left motor 0-50
		Left_Motor_Pulse = Vector + 150;													// Add additional pulse to center
	//Right	
		Theta = 57.2957 * atan(Scaled_Speed/Scaled_Direction);
		Max_Pulse = Theta * 1.11;
		if (Max_Pulse>50){
			Max_Pulse = Max_Pulse - 50;
			Right_Motor_Pulse = Max_Pulse*Vector*.02;
			Right_Motor_Pulse = Right_Motor_Pulse + 150;
		}
		else{
			Max_Pulse = 50 - Max_Pulse;
			Right_Motor_Pulse = Max_Pulse*Vector*.02;
			Right_Motor_Pulse = 150-Right_Motor_Pulse;
		}
	}//end Q1

	else if ((Speed >= CENTER)&&(Direction >= CENTER)){										// Quadrant 2 criteria
	//Left
		Scaled_Speed = ((float)Speed - CENTER)/2.54;										// Speed goes 0-50
		Scaled_Direction = ((float)Direction - CENTER)/2.54;								// Direction goes 0-50
		Theta = 57.2957*atan(Scaled_Speed/Scaled_Direction);								// Vector angle
		Max_Pulse = Theta * 1.11;															// Max Pulse (0-100)
		Vector = sqrt(Scaled_Speed*Scaled_Speed + Scaled_Direction*Scaled_Direction);		// Vector length
		if (Max_Pulse > 50){																// if left motor should move forward
			Max_Pulse = Max_Pulse - 50;														// now max pulse scales to 0-50
			Left_Motor_Pulse = Max_Pulse*(Vector*.02);										// LMP now has a pulse difference
			Left_Motor_Pulse = Left_Motor_Pulse + 150;										// LMP now centered
		}					
		else { 																				// else, the left motor must move backward
			Max_Pulse = 50 - Max_Pulse;														// scale pulse to 0-50
			Left_Motor_Pulse = Max_Pulse*(Vector*.02);										// Scale max pulse by vector
			Left_Motor_Pulse = 150 - Left_Motor_Pulse;										// Center LMP
		}
	//Right		
		Right_Motor_Pulse = Vector + 150;
	}//end Q2

	else if ((Speed <= CENTER)&&(Direction >= CENTER)){										// Quadrant 3 criteria
	//Left
		Scaled_Speed = (CENTER - (float)Speed)/2.54;										// Speed goes 0-50
		Scaled_Direction = ((float)Direction - CENTER)/2.54;								// Throttle goes 0-50
		Vector = sqrt(Scaled_Speed*Scaled_Speed + Scaled_Direction*Scaled_Direction);		// Find pulse below center for left motor
		Left_Motor_Pulse = 150 - Vector;													// Recenter
	//Right
		Theta = 57.2957 * atan(Scaled_Direction/Scaled_Speed);
		Max_Pulse = Theta * 1.11;
		if (Max_Pulse>50){
			Max_Pulse = Max_Pulse - 50;
			Right_Motor_Pulse = Max_Pulse*Vector*.02;
			Right_Motor_Pulse = Right_Motor_Pulse + 150;	
		}
		else{
			Max_Pulse = 50 - Max_Pulse;
			Right_Motor_Pulse = Max_Pulse*Vector*.02;
			Right_Motor_Pulse = 150 - Right_Motor_Pulse;
		}
	}//end Q3

	else if ((Speed <= CENTER)&&(Direction <= CENTER)){										// Quadrant 4 criteria
	//Left
		Scaled_Speed = (CENTER - (float)Speed)/2.54;
		Scaled_Direction = (CENTER - (float)Direction)/2.54;
		Theta = 57.2957*atan(Scaled_Direction/Scaled_Speed);
		Max_Pulse = Theta * 1.11;
		Vector = sqrt(Scaled_Speed*Scaled_Speed + Scaled_Direction*Scaled_Direction);
		if (Max_Pulse > 50){
			Max_Pulse = Max_Pulse - 50;
			Left_Motor_Pulse = Max_Pulse*Vector*.02;
			Left_Motor_Pulse = Left_Motor_Pulse + 150;	
		}
		else{
			Max_Pulse = 50 - Max_Pulse;
			Left_Motor_Pulse = Max_Pulse*(Vector*.02);
			Left_Motor_Pulse = 150 - Left_Motor_Pulse;
		}
	//Right
		Right_Motor_Pulse = 150 - Vector;
	}//end Q4

	//Trim limits
	if (Left_Motor_Pulse < 100)
		Left_Motor_Pulse = 100;
	else if (Left_Motor_Pulse > 200)
		Left_Motor_Pulse = 200;

	// comes in 100 to 200, writes to registers 36000 to 38000
	if (Left_Motor_Pulse > 150)
		Left_Motor_Pulse = (float)(Left_Motor_Pulse-150)*MOTOR_SCALE_L+150;
	else if (Left_Motor_Pulse < 150)
		Left_Motor_Pulse = 150-((float)(50-(Left_Motor_Pulse - 100))*MOTOR_SCALE_L);
	if (Right_Motor_Pulse > 150)
		Right_Motor_Pulse = (float)(Right_Motor_Pulse-150)*MOTOR_SCALE_R+150;
	else if (Right_Motor_Pulse < 150)
		Right_Motor_Pulse = 150-((float)(50-(Right_Motor_Pulse-100))*MOTOR_SCALE_R);

	OC2R = (Left_Motor_Pulse - 100)*20+36000;
	OC3R = (Right_Motor_Pulse - 100)*20+36000;	
}

