#include "motor.h"
void pid_tick(bool OverCurrent);
void closed_loop_control_init(void);
int return_closed_loop_control_effort(MotorChannel motor);
void pid_set_desired_velocities(int left, int right, int flipper);
