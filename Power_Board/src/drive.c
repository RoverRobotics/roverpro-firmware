#include "drive.h"
#include "motor.h"
#include "main.h"

/// The PWM values being sent to each of the motors.
/// from -1000 to 1000
/// 0 indicates the motor should brake.
/// NOTE the motors may also be coasting, e.g. if
MotorEfforts motor_efforts = {0};
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

void drive_set_efforts(MotorEfforts new_efforts) { 
        REG_MOTOR_VELOCITY.left = new_efforts.left;
        REG_MOTOR_VELOCITY.right = new_efforts.right;
        REG_MOTOR_VELOCITY.flipper = new_efforts.flipper;
}

bool drive_is_approximately_stopped() {
    // TODO: usa actual motor speed, not commanded speed, for this
    return (abs(motor_efforts.left) < 10 && abs(motor_efforts.right) < 10 &&
            abs(motor_efforts.flipper) < 10);
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
    if (coast_lock) {
        Coasting(MOTOR_LEFT);
        Coasting(MOTOR_RIGHT);
        Coasting(MOTOR_FLIPPER);

    } else {
        drive_tick_motor(MOTOR_LEFT, motor_efforts.left);
        drive_tick_motor(MOTOR_RIGHT, motor_efforts.right);
        drive_tick_motor(MOTOR_FLIPPER, motor_efforts.flipper);
    }
}