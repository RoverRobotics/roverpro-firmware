#include "drive.h"
#include "clock.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "settings.h"
#include "stdhdr.h"

void drive_overspeed_init() {
    g_state.drive.last_lo_speed_timestamp = clock_now();
    g_state.drive.last_overspeed_fault_timestamp = 0;
}

float get_motor_encoder_freq_hz(MotorChannel c) {
    uint16_t period = g_state.drive.motor_encoder_period[c];
    if (period == 0 || period == 0xFFFF)
        return 0.0F;
    return 1.0F / ticks_to_seconds(((int64_t)period) * 256);
}

void drive_overspeed_tick() {
    if (g_settings.drive.hi_speed_encoder_hz <= 0.0F ||
        g_settings.drive.overspeed_fault_trigger_s <= 0.0F)
        return;

    float overspeed_hz = g_settings.drive.hi_speed_encoder_hz;
    float lfreq = get_motor_encoder_freq_hz(MOTOR_LEFT);
    float rfreq = get_motor_encoder_freq_hz(MOTOR_RIGHT);
    // reasonable values for freq = about 200 Hz
    // max value (no wheels full throttle) = 1100 Hz

    if (lfreq <= overspeed_hz && rfreq <= overspeed_hz) {
        // okay. We're going at a slow speed
        g_state.drive.last_lo_speed_timestamp = clock_now();
    } else if (
        clock_now() < g_state.drive.last_lo_speed_timestamp +
                          seconds_to_ticks(g_settings.drive.overspeed_fault_trigger_s)) {
        // don't panic yet...
    } else {
        // panic!
        g_state.drive.last_overspeed_fault_timestamp = clock_now();
    }
}

void drive_tick() {
    uint64_t t0 = g_state.drive.last_update_time;
    uint64_t t1 = g_state.drive.last_update_time = clock_now();
    float target_effort[MOTOR_CHANNEL_COUNT] = {0};
    bool brake_on_zero_speed;

    float command_age_s =
        ticks_to_seconds(clock_now() - g_state.communication.drive_command_timestamp);
    if (get_system_fault() != FAULT_NONE) {
        brake_on_zero_speed = g_settings.drive.brake_on_fault;
    } else if (
        g_settings.communication.drive_command_timeout_ms > 0 &&
        g_settings.communication.drive_command_timeout_ms * 0.001F < command_age_s) {
        brake_on_zero_speed = g_settings.drive.brake_on_drive_timeout;
    } else {
        brake_on_zero_speed = g_settings.drive.brake_on_zero_speed_command;
        MotorChannel c;
        for (EACH_MOTOR_CHANNEL(c)) {
            target_effort[c] = g_state.communication.motor_effort[c];
        }
    }

    MotorChannel c;
    for (EACH_MOTOR_CHANNEL(c)) {
        float effort;
        MotorStatusFlag motor_flag;

        float last_effort = g_state.drive.last_motor_effort[c];
        float max_delta_effort =
            (float)(t1 - t0) / seconds_to_ticks(g_settings.drive.time_to_full_speed);
        effort =
            clamp(target_effort[c], last_effort - max_delta_effort, last_effort + max_delta_effort);

        motor_flag = MOTOR_FLAG_NONE;
        if ((effort < 0.0F) ^ (c == MOTOR_RIGHT)) {
            motor_flag |= MOTOR_FLAG_REVERSE;
        }
        if (effort == 0.0F) {
            motor_flag |= brake_on_zero_speed ? MOTOR_FLAG_BRAKE : MOTOR_FLAG_COAST;
        }
        if (g_settings.drive.motor_slow_decay_mode) {
            motor_flag |= MOTOR_FLAG_DECAY_MODE;
        }

        // in fast (normal) decay mode, 0 duty = still, 1 = forward
        // in slow decay mode, 0 duty = reverse, 0.5 = still, 1 = forward
        float duty =
            (motor_flag & MOTOR_FLAG_DECAY_MODE) ? fabsf(effort) * 0.5F + 0.5F : fabsf(effort);
        g_state.drive.motor_status[c] = motor_update(c, motor_flag, duty);
        g_state.drive.last_motor_effort[c] = effort;
    }

    tach_tick();
    drive_overspeed_tick();
}

void drive_init() {
    MotorChannel c;
    g_state.drive.last_update_time = clock_now();
    float pwm_frequency_hz = g_settings.drive.motor_pwm_frequency_hz;
    for (EACH_MOTOR_CHANNEL(c)) {
        g_state.drive.motor_status[c] = motor_update(c, MOTOR_FLAG_COAST, 0.0F);
        g_state.drive.last_motor_effort[c] = 0.0F;
        motor_init(c, pwm_frequency_hz);
    }
    motor_tach_init();
    drive_overspeed_init();
}