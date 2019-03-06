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
    MotorEvent new_motor_event = effort_to_event(new_motor_effort);

    if (dead_time_counter[c] == UINT16_MAX && motor_event[c] == new_motor_event) {
        // We don't need to coast
        if (new_motor_event == STOP) {
            Braking(c);
        } else {
            UpdateSpeed(c, new_motor_effort);
        }
    } else {
        Coasting(c);
        dead_time_counter[c]++;
        if (dead_time_counter[c] * g_settings.main.drive_poll_ms >
            g_settings.drive.motor_protect_direction_delay_ms) {
            // next iteration we're going to stop coasting
            motor_event[c] = new_motor_event;
            dead_time_counter[c] = UINT16_MAX;
        }
    }
}

void drive_tick() {
    if (coast_lock || g_state.power.overcurrent) {
        Coasting(MOTOR_LEFT);
        Coasting(MOTOR_RIGHT);
        Coasting(MOTOR_FLIPPER);
    } else {
        drive_tick_motor(MOTOR_LEFT, g_state.communication.motor_effort[MOTOR_LEFT]);
        drive_tick_motor(MOTOR_RIGHT, g_state.communication.motor_effort[MOTOR_RIGHT]);
        drive_tick_motor(MOTOR_FLIPPER, g_state.communication.motor_effort[MOTOR_FLIPPER]);
    }

    // update the mosfet driving fault flag pin 1-good 2-fault
    g_state.drive.motor_fault_flags[MOTOR_LEFT] = _RD1;
    g_state.drive.motor_fault_flags[MOTOR_RIGHT] = _RE5;

    // read out measured motor periods.
    g_state.drive.motor_encoder_period[MOTOR_LEFT] =
        (uint16_t)fabs(motor_tach_get_period(MOTOR_LEFT));
    g_state.drive.motor_encoder_period[MOTOR_RIGHT] =
        (uint16_t)fabs(motor_tach_get_period(MOTOR_RIGHT));
}