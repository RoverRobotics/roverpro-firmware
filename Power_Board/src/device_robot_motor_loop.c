
#include "main.h"
#include "device_robot_motor_loop.h"
#include "motor.h"

#include "PID.h"

#define N_FILTERS 2
#define LMOTOR_FILTER 0
#define RMOTOR_FILTER 1
/*---------------------------Public Functions---------------------------------*/

/// Passes the input through an Infinite-Impulse-Response filter characterized by the parameter
/// alpha.
/// @param i the filter index
/// @param x the current sample to be filtered
/// @param alpha the knob on how much to filter, [0,1) alpha = t / (t + dT), where t = the low-pass
/// filter's time-constant, dT = the sample rate
/// @param should_reset whether to clear the history
float IIRFilter(const uint8_t i, const float x, const float alpha, const bool should_reset) {
    // see also:
    // http://dsp.stackexchange.com/questions/1004/low-pass-filter-in-non-ee-software-api-contexts
    // aka a 'leaky integrator'
    static float y_lasts[N_FILTERS] = {0};
    if (should_reset)
        y_lasts[i] = 0;
    float y = alpha * y_lasts[i] + (1.0f - alpha) * x;
    y_lasts[i] = y;

    return y;
}

/*---------------------------Controller Related-------------------------------*/

// PID controller values
#define LEFT_CONTROLLER 0
#define RIGHT_CONTROLLER 1

// OCU speed filter-related values
#define MAX_DESIRED_SPEED 900 // [au], caps incoming signal from OCU
#define MIN_ACHEIVABLE_SPEED 50

float DT_speed(MotorChannel motor);
static float GetNominalDriveEffort(float desired_speed);
static int16_t GetDesiredSpeed(MotorChannel motor);

static float closed_loop_effort[MOTOR_CHANNEL_COUNT] = {0};

static int desired_velocity_left = 0;
static int desired_velocity_right = 0;
static int desired_velocity_flipper = 0;

void closed_loop_control_init(void) {
    PID_Init(LEFT_CONTROLLER, g_settings.motor_controller.max_effort,
             g_settings.motor_controller.min_effort, g_settings.motor_controller.pid_p_weight,
             g_settings.motor_controller.pid_i_weight, g_settings.motor_controller.pid_d_weight);
    PID_Init(RIGHT_CONTROLLER, g_settings.motor_controller.max_effort,
             g_settings.motor_controller.min_effort, g_settings.motor_controller.pid_p_weight,
             g_settings.motor_controller.pid_i_weight, g_settings.motor_controller.pid_d_weight);
}

void pid_tick(bool OverCurrent) {
    static unsigned int stop_counter = 0;

    // If we have stopped the motors due to overcurrent, don't update speeds
    if (OverCurrent) {
        PID_Reset(MOTOR_LEFT);
        PID_Reset(MOTOR_RIGHT);
        return;
    }

    // Filter drive motor speeds
    float desired_speed_left = IIRFilter(LMOTOR_FILTER, GetDesiredSpeed(MOTOR_LEFT),
                                         g_settings.motor_controller.iir_alpha, false);
    float desired_speed_right = IIRFilter(RMOTOR_FILTER, GetDesiredSpeed(MOTOR_RIGHT),
                                          g_settings.motor_controller.iir_alpha, false);

    // if the user releases the joystick, come to a relatively quick stop by clearing the integral
    // term
    if ((abs(REG_MOTOR_VELOCITY.left) < 50) && (abs(REG_MOTOR_VELOCITY.right) < 50)) {
        PID_Reset_Integral(MOTOR_LEFT);
        PID_Reset_Integral(MOTOR_RIGHT);

        // If user releases joystick, reset the IIR filter
        IIRFilter(LMOTOR_FILTER, 0, g_settings.motor_controller.iir_alpha, true);
        IIRFilter(RMOTOR_FILTER, 0, g_settings.motor_controller.iir_alpha, true);
        desired_speed_left = 0;
        desired_speed_right = 0;
    }

    // update the flipper
    float desired_flipper_speed = desired_velocity_flipper / 1200.0f;

    closed_loop_effort[MOTOR_FLIPPER] = desired_flipper_speed;

    // update the left drive motor
    float nominal_effort_left = GetNominalDriveEffort(desired_speed_left);
    float actual_speed_left = DT_speed(MOTOR_LEFT);
    float effort_left = PID_ComputeEffort(LEFT_CONTROLLER, desired_speed_left, actual_speed_left,
                                          nominal_effort_left);

    closed_loop_effort[MOTOR_LEFT] = effort_left;

    // update the right drive motor
    float nominal_effort_right = GetNominalDriveEffort(desired_speed_right);
    float actual_speed_right = DT_speed(MOTOR_RIGHT);
    float effort_right = PID_ComputeEffort(RIGHT_CONTROLLER, desired_speed_right,
                                           actual_speed_right, nominal_effort_right);
    closed_loop_effort[MOTOR_RIGHT] = effort_right;

    // if the speed inputs are 0, reset controller after 1 second
    // TODO: fix controller so that we don't get these small offsets
    if ((desired_velocity_left == 0) && (desired_velocity_right == 0)) {
        stop_counter++;
        if (stop_counter > 100) {
            PID_Reset(MOTOR_LEFT);
            PID_Reset(MOTOR_RIGHT);
            stop_counter = 0;
        }
    } else
        stop_counter = 0;
}

int return_closed_loop_control_effort(MotorChannel motor) {
    return (int)(closed_loop_effort[motor] * 1000.0);
}

float DT_speed(MotorChannel motor) {
#define HZ_16US 100000.0f
    float period = 0;
    period = motor_tach_get_period(motor);
    if (period == 0)
        return 0.0f;
    else
        return (HZ_16US / period);
}

// Description: Returns the approximate steady-state effort required to
//   maintain the given desired speed of a drive motor.
static float GetNominalDriveEffort(float desired_speed) {
    // NB: transfer function found empirically (see spreadsheet for data)
    if (desired_speed == 0)
        return 0;
    else if (desired_speed < 0)
        return ((0.0007f * desired_speed) - 0.0067f);
    else
        return ((0.0007f * desired_speed) + 0.0067f);
}

// Description: Maps the incoming control data to suitable values
// Notes:
//   - special-cases turning in place to higher values to overcome
//     the additional torque b/c software change has too much overhead right now
static int16_t GetDesiredSpeed(MotorChannel motor) {
    int16_t temp_left = desired_velocity_left / 4;
    int16_t temp_right = desired_velocity_right / 4;

    switch (motor) {
    case MOTOR_LEFT:
        if (abs(temp_left) < MIN_ACHEIVABLE_SPEED) {
            return 0;
        }

        // if we are turning (sign bits do not match AND magnitudes are non-negligible)
        if (((temp_left >> 15) != (temp_right >> 15)) &&
            ((200 < abs(temp_left)) && (200 < abs(temp_right)))) {
            if (0 < temp_left)
                return 500;
            else
                return -500;
        }

        return clamp(temp_left, -MAX_DESIRED_SPEED, +MAX_DESIRED_SPEED);

    case MOTOR_RIGHT:
        if (abs(temp_right) < MIN_ACHEIVABLE_SPEED) {
            return 0;
        }

        if (((temp_left >> 15) != (temp_right >> 15)) &&
            ((200 < abs(temp_left)) && (200 < abs(temp_right)))) {
            if (0 < temp_right)
                return 500;
            else
                return -500;
        }

        return clamp(temp_right, -MAX_DESIRED_SPEED, +MAX_DESIRED_SPEED);

    case MOTOR_FLIPPER:
        return REG_MOTOR_VELOCITY.flipper;
    }

    return 0;
}

void pid_set_desired_velocities(int left, int right, int flipper) {
    desired_velocity_left = left;
    desired_velocity_right = right;
    desired_velocity_flipper = flipper;
}
