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
    uint64_t delta_time = t1 - t0;
    float max_delta_effort = min((t1 - t0) * (1.0F / CLOCK_S) / g_settings.drive.time_to_full_speed,
                                 g_settings.drive.max_instantaneous_delta_effort);

    MotorChannel c;
    for (EACH_MOTOR_CHANNEL(c)) {
        float duty;
        MotorStatusFlag motor_flag;
        if (overcurrent) {
            duty = 0.F;
            motor_flag = MOTOR_FLAG_COAST;
        } else {
            float last_effort = g_state.drive.last_motor_effort[c];
            float smoothed_effort =
                clamp(g_state.communication.motor_effort[c], last_effort - max_delta_effort,
                      last_effort + max_delta_effort);
            g_state.drive.last_motor_effort[c] = smoothed_effort;

            motor_flag = MOTOR_FLAG_NONE;
            if ((smoothed_effort < 0.0F) ^ (c == MOTOR_RIGHT)) {
                motor_flag |= MOTOR_FLAG_REVERSE;
            }
            if (smoothed_effort == 0.0F) {
                motor_flag |=
                    g_state.communication.brake_when_stopped ? MOTOR_FLAG_BRAKE : MOTOR_FLAG_COAST;
            }

            if (g_settings.drive.motor_slow_decay_mode) {
                // in slow decay mode, 0 duty = reverse, 0.5 = still, 1 = forward
                duty = (fabsf(smoothed_effort) + 1.F) * 0.5F;
                motor_flag |= MOTOR_FLAG_DECAY_MODE;
            } else {
                duty = fabsf(smoothed_effort);
            }
        }
        g_state.drive.motor_status[c] = motor_update(c, motor_flag, duty);
    }

    // read out measured motor periods, taking the absolute value and dividing by 256 to match old
    // behavior.
    int64_t period = labs(motor_tach_get_period(MOTOR_LEFT) / 256);
    g_state.drive.motor_encoder_period[MOTOR_LEFT] =
        period > UINT16_MAX ? UINT16_MAX : (uint16_t)period;
    period = labs(motor_tach_get_period(MOTOR_RIGHT) / 256);
    g_state.drive.motor_encoder_period[MOTOR_RIGHT] =
        period > UINT16_MAX ? UINT16_MAX : (uint16_t)period;
}

void drive_init() {
    MotorChannel c;
    g_state.drive.last_update_time = clock_now();

    for (EACH_MOTOR_CHANNEL(c)) {
        g_state.drive.last_motor_effort[c] = 0.0F;
        motor_init(c);
    }
    motor_tach_init();
}