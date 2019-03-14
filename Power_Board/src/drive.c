#include "drive.h"
#include "motor.h"
#include "main.h"
#include "math.h"

/// Motors should not switch direction too abruptly. This is the number of drive ticks we have
/// ignored a motor command.
uint16_t dead_time_counter[MOTOR_CHANNEL_COUNT] = {0};
/// The motors should be turned off
bool coast_lock = false;

/// For motor state machine. The requested motor state based in inbound motor speed commands, which
/// informs state machine transitions
typedef enum MotorEvent {
    /// Motor has not been commanded
    NO_EVENT = 0xFF00,
    /// Motor has been commanded to stop
    STOP = 0xFF01,
    /// Motor has been commanded to move forward
    GO = 0xFF02,
    /// Motor has been commanded to move backward
    BACK = 0xFF03,
    COAST = 0xFF04,
} MotorEvent;

static MotorEvent motor_event[MOTOR_CHANNEL_COUNT] = {NO_EVENT, NO_EVENT, NO_EVENT};

MotorEvent effort_to_event(int16_t effort) {
    if (effort < 0)
        return BACK;
    else if (effort > 0)
        return GO;
    else
        return STOP;
}

void drive_set_coast_lock(bool is_on) { coast_lock = is_on; }

void drive_tick_motor(MotorChannel c, int16_t new_motor_effort) {
    MotorStatusFlag new_status;
    MotorEvent new_motor_event = effort_to_event(new_motor_effort);

    if (dead_time_counter[c] == UINT16_MAX && motor_event[c] == new_motor_event) {
        if (new_motor_effort == 0)
            new_status = MOTOR_FLAG_BRAKE;
        else if (new_motor_effort < 0)
            new_status = MOTOR_FLAG_REVERSE;
        else
            new_status = MOTOR_FLAG_NONE;

        // Note the motor direction here is reversed, since the motor is installed backwards :-)
        if (c == MOTOR_RIGHT)
            new_status ^= MOTOR_FLAG_REVERSE;
    } else {
        new_status = MOTOR_FLAG_COAST;
        dead_time_counter[c]++;
        if (dead_time_counter[c] * g_settings.main.drive_poll_ms >
            g_settings.drive.motor_protect_direction_delay_ms) {
            // next iteration we're going to stop coasting
            motor_event[c] = new_motor_event;
            dead_time_counter[c] = UINT16_MAX;
        }
    }

    uint16_t new_duty =
        ((new_status & MOTOR_FLAG_BRAKE || new_status & MOTOR_FLAG_COAST) ? 0
                                                                          : abs(new_motor_effort));
    g_state.drive.motor_status[c] = motor_update(c, new_status, new_duty);
}

void drive_tick() {
    MotorChannel c;
    if (coast_lock || g_state.power.overcurrent) {
        for (EACH_MOTOR_CHANNEL(c)) {
            g_state.drive.motor_status[c] = motor_update(c, MOTOR_FLAG_COAST, 0);
        }
    } else {
        for (EACH_MOTOR_CHANNEL(c)) {
            drive_tick_motor(c, g_state.communication.motor_effort[c]);
        }
    }

    // read out measured motor periods.
    g_state.drive.motor_encoder_period[MOTOR_LEFT] =
        (uint16_t)fabs(motor_tach_get_period(MOTOR_LEFT));
    g_state.drive.motor_encoder_period[MOTOR_RIGHT] =
        (uint16_t)fabs(motor_tach_get_period(MOTOR_RIGHT));
}

void drive_init() {
    MotorChannel c;
    for (EACH_MOTOR_CHANNEL(c)) {
        motor_init(c);
    }

    motor_tach_init();
}