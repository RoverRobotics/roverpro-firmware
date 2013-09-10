void calibrate_joint(void);
//extern const int pot_angles[4];
extern int pot_angles[4];

extern unsigned char board_type;

void read_pot_calibration(void);
int return_joint_position(void);

//void form_next_payload_message(void);
void form_prev_payload_message(void);

void act_on_message(void);

void form_prev_payload_message(void);
void arm_control_loop(void);

void trigger_motor_reset(void);

#define EXTEND 0x00
#define RETRACT 0x01
#define ELBOW_ONLY 0x02


/*#define CENTER_ELBOW_SPEED_RETRACT 140
#define CENTER_SHOULDER_SPEED_RETRACT 130
#define CENTER_ELBOW_SPEED_EXTEND 60
#define CENTER_SHOULDER_SPEED_EXTEND 70

#define MAX_ELBOW_SPEED_EXTEND 150
#define MIN_ELBOW_SPEED_EXTEND 50
#define MAX_SHOULDER_SPEED_EXTEND 100
#define MIN_SHOULDER_SPEED_EXTEND 60

#define MAX_ELBOW_SPEED_RETRACT 150
#define MIN_ELBOW_SPEED_ERETRACT 50
#define MAX_SHOULDER_SPEED_RETRACT 140
#define MIN_SHOULDER_SPEED_RETRACT 100*/


/*#define CENTER_ELBOW_SPEED_RETRACT 160
#define CENTER_SHOULDER_SPEED_RETRACT 130
#define CENTER_ELBOW_SPEED_EXTEND 40
#define CENTER_SHOULDER_SPEED_EXTEND 70

#define MAX_ELBOW_SPEED_EXTEND 170
#define MIN_ELBOW_SPEED_EXTEND 30
#define MAX_SHOULDER_SPEED_EXTEND 100
#define MIN_SHOULDER_SPEED_EXTEND 60

#define MAX_ELBOW_SPEED_RETRACT 170
#define MIN_ELBOW_SPEED_ERETRACT 30
#define MAX_SHOULDER_SPEED_RETRACT 140
#define MIN_SHOULDER_SPEED_RETRACT 100*/

#define CENTER_ELBOW_SPEED_RETRACT 180
#define CENTER_SHOULDER_SPEED_RETRACT 160
#define CENTER_ELBOW_SPEED_EXTEND 20
#define CENTER_SHOULDER_SPEED_EXTEND 40

#define MAX_ELBOW_SPEED_EXTEND 200
#define MIN_ELBOW_SPEED_EXTEND 0
#define MAX_SHOULDER_SPEED_EXTEND 100
#define MIN_SHOULDER_SPEED_EXTEND 0

#define MAX_ELBOW_SPEED_RETRACT 200
#define MIN_ELBOW_SPEED_ERETRACT 0
#define MAX_SHOULDER_SPEED_RETRACT 200
#define MIN_SHOULDER_SPEED_RETRACT 100

#define KP_ELBOW_EXTEND 3
#define KP_ELBOW_RETRACT 3
#define KP_SHOULDER_EXTEND 3
#define KP_SHOULDER_RETRACT 3


#define SHOULDER_Kp 1
#define SHOULDER_Ki .1
#define ELBOW_Kp 1
#define ELBOW_Ki .1
