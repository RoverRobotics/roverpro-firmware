#include "clock.h"
#include "drive.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "stdhdr.h"

/// Motors should not switch direction too abruptly. This is the number of drive ticks we have
/// ignored a motor command.
uint16_t g_dead_time_counter[MOTOR_CHANNEL_COUNT] = {0};

void drive_tick() {
    uint64_t t0 = g_state.drive.last_update_time;
    uint64_t t1 = g_state.drive.last_update_time = clock_now();
    bool overcurrent = g_state.power.overcurrent;

    MotorChannel c;
    for (EACH_MOTOR_CHANNEL(c)) {
		float effort;
        MotorStatusFlag motor_flag;
        if (overcurrent) {
			effort = 0.0F;
            motor_flag = MOTOR_FLAG_COAST;
        } else {
            float last_effort = g_state.drive.last_motor_effort[c];
			float max_delta_effort = min(
				(float)(t1 - t0) * (1.0F / CLOCK_S) / g_settings.drive.time_to_full_speed,
				g_settings.drive.max_instantaneous_delta_effort
			);
            effort = clamp(g_state.communication.motor_effort[c], last_effort - max_delta_effort,
                      last_effort + max_delta_effort);

            motor_flag = MOTOR_FLAG_NONE;
            if ((effort < 0.0F) ^ (c == MOTOR_RIGHT)) {
                motor_flag |= MOTOR_FLAG_REVERSE;
            }
            if (effort == 0.0F && g_state.communication.brake_when_stopped) {
                motor_flag |= MOTOR_FLAG_BRAKE;
            }
            if (g_settings.drive.motor_slow_decay_mode) {
                motor_flag |= MOTOR_FLAG_DECAY_MODE;
            }
        }
		// in fast (normal) decay mode, 0 duty = still, 1 = forward
        // in slow decay mode, 0 duty = reverse, 0.5 = still, 1 = forward
		float duty = (motor_flag & MOTOR_FLAG_DECAY_MODE) ? fabsf(effort) * 0.5F + 0.5F : fabsf(effort);
        g_state.drive.motor_status[c] = motor_update(c, motor_flag, duty);
		g_state.drive.last_motor_effort[c] = effort;
    }

    tach_tick();
}

void drive_init() {
    MotorChannel c;
    g_state.drive.last_update_time = clock_now();

    for (EACH_MOTOR_CHANNEL(c)) {
		g_state.drive.motor_status[c] = motor_update(c, MOTOR_FLAG_COAST, 0.0F);
		g_state.drive.last_motor_effort[c] = 0.0F;
        motor_init(c);
    }
    motor_tach_init();
}