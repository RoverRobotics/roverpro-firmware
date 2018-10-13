void handle_closed_loop_control(unsigned int OverCurrent);
void closed_loop_control_init(void);
int return_closed_loop_control_effort(unsigned char motor);
void set_desired_velocities(int left, int right, int flipper);

#define MAX_NUM_CONTROLLERS   8
