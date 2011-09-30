void Flipper_PI(void);
void Run_Flipper_Control(unsigned char Dir, unsigned char Flip_On);
unsigned char Is_Robot_Upside_Down(void);
void Blocking_Self_Right(void);
void handle_flipper(int PI_motor_effort);

#define FLIPPER_SWITCH PORTFbits.RF3

#define Ki (0.30)
#define Kp (0.025)
#define SCALER (.70)


