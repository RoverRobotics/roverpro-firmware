void handle_closed_loop_control(unsigned int OverCurrent);
void closed_loop_control_init(void);
int return_closed_loop_control_effort(unsigned char motor);

#define MAX_NUM_CONTROLLERS   8
