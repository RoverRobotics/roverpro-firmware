void command_servo(unsigned char servo_address, unsigned char position);
void servo_loop(void);
void Send_Servo_Message(unsigned char servo_address);
void Dynamixel_Test(void);

//extern unsigned char Dynamixel_position_msb[6];
//extern unsigned char Dynamixel_position_lsb[6];
extern int Dynamixel_position[6];
extern unsigned char Dynamixel_direction[6];
//extern unsigned char Dynamixel_speed_msb[6];
//extern unsigned char Dynamixel_speed_lsb[6];
extern int Dynamixel_speed[6];

#define NUM_DYNAMIXEL 2

#define POSITION_STEP 20
