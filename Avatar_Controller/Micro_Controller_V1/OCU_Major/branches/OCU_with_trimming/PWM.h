void PWM_Init(void);
void Set_PWM_HIGH_TIME(unsigned char channel, unsigned int duty_cycle_100X);

//used to test subsystem
void PWM_Test(void);


#define PWM_channel_1 0x00
#define PWM_channel_2 0x01
