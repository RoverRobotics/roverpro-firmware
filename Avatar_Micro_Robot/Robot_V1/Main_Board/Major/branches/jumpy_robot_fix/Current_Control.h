//Uncomment this to disable all speed control
//#define NO_ACCELERATION_CURVE

#define CONTINUOUS_CURRENT_CYCLES 50*CONTINUOUS_CURRENT_SECONDS
#define MAX_AD_VOLTAGE 2.5

//For Brian's board, (.5+.185*current)*10/15 = 1/3 + 37/300*current
//1023/3.3 = 310 units/V
//1/3 + 37/300 * current = AD_value/310
//310/3 + 1147/30*current = AD_value
//current = (AD_value - 310/3)*30/1147 = AD_Value/38.23 - 2.70
//#define CURRENT_DIVISOR 38.23
//#define CURRENT_OFFSET	-2.70

//For 30A sensor, (.5+.133*current)*10/15 = 1/3 + 133/1500*current
//I = [AD_value*3.3/1023*(R1+R2)/R2 - Vout]/V_slope
#define CURRENT_DIVISOR 27.49
#define CURRENT_OFFSET	-3.76

#define T3_PRESCALE	8

#define SLOWEST_MOVING_EFFORT 10

#define SPEED_STATE_STARTUP 0

float average_motor_current(unsigned int reset);
void init_current_limit_atod(void);
void speed_control_loop_quadrants(char motors_off);

void init_timer3(void);


//This is used for timing the slow start, so we can guess
//if the robot is heavily loaded and has just popped out the battery
extern unsigned int Current_Interrupt_Count;
extern int Max_Speed;


//This is used for measuring current
extern unsigned int T3_Interrupt_Count;
extern unsigned int last_T3_Interrupt_Count;
extern unsigned int Jumpiness_Interrupt_Count;

