void Update_Robot_Commands(unsigned char disable_driving);
void Flipper_Control(void);

extern unsigned char robot_moving_flag;

#define AUDIO_DEADBAND 10
#define AUDIO_DEADBAND_UPPER 127.5+AUDIO_DEADBAND
#define AUDIO_DEADBAND_LOWER 127.5-AUDIO_DEADBAND


#define AUDIO_DEADBAND_ON 15
#define AUDIO_DEADBAND_OFF 8

extern unsigned char OCU_battery_voltage;
extern unsigned char ocu_robot_talk;
