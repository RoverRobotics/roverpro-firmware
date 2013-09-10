void init_pwm();

void set_motor_effort(int effort);
void turn_off_servo_signal(void);
void open_hitch(void);
void close_hitch(void);

extern int servo_lockout;
