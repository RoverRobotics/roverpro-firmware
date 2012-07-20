
void set_motor_effort(int effort,unsigned char motor);


int Return_Desired_Motor_Effort_Quadrants(unsigned char motor);
void Set_Desired_Motor_Pulse_Quadrants(unsigned char Direction, unsigned char Speed, unsigned char Flipper_On, unsigned char Flipper_Dir);



#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define FLIPPER_MOTOR 2


#define QUADRANT_MIXING
//#define NO_QUADRANT_MIXING

//#define MIX1
#define MIX3



//MIN_ANGLE is the joystick angle at which the robot transitions to a full turn
//MIN_EFFORT is the minimum forward effort (0-1) that the robot uses at MIN_ANGLE on the slowest flipper

//Rock like the driving for the most part, but complained about the low degree of turning with the joytick at 45 deg.
#ifdef MIX1
	#define MIN_ANGLE 21.8
	#define MIN_EFFORT .5
#endif

#ifdef MIX2
	#define MIN_ANGLE 21.8
	#define MIN_EFFORT .25
#endif


//Eric liked this one.  I though that it was too difficult to control at high speeds,
//but it could probably be controllable with 10 or 15 minutes of practice.
#ifdef MIX3
	#define MIN_ANGLE 21.8
	#define MIN_EFFORT 0
#endif



