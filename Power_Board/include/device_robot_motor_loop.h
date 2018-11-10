#include "device_robot_motor.h"
void handle_closed_loop_control(bool OverCurrent);
void closed_loop_control_init(void);
int return_closed_loop_control_effort(MotorChannel motor);
void set_desired_velocities(int left, int right, int flipper);
