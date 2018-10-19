#include <p24fxxxx.h>
#include "stdhdr.h"
#include "device_robot_motor_loop.h"
#include "device_robot_motor.h"
#include "../closed_loop_control/core/InputCapture.h"

// supported motor options
typedef enum {
    kMotorLeft = 0,
    kMotorRight,
    kMotorFlipper,
} kMotor;

#define M1_TACHO_RPN 12 // RP12
#define M2_TACHO_RPN 16

#include "../closed_loop_control/PID.h"

/*---------------------------Helper Function Prototypes-----------------------*/
/*---------------------------IC Related---------------------------------------*/
#define MAX_NUM_IC_PINS 2
static volatile uint32_t timeouts[MAX_NUM_IC_PINS] = {0};

/*---------------------------PID Related--------------------------------------*/
/*---------------------------Filter Related-----------------------------------*/
float IIRFilter(const uint8_t i, const float x, const float alpha, const bool should_reset);

#define ALPHA 0.8
#define LMOTOR_FILTER 0
#define RMOTOR_FILTER 1
/*---------------------------Controller Related-------------------------------*/

// PID controller values
#define LEFT_CONTROLLER 0
#define RIGHT_CONTROLLER 1

#define MAX_EFFORT 1.00 // maximum control effort magnitude (can also be -1000)
#define MIN_EFFORT -1.00
#define K_P 0.0005   // proportional gain
#define K_I 0.00003  // integral gain
#define K_D 0.000000 // differential gain

// filters
#define ALPHA 0.8
#define LMOTOR_FILTER 0
#define RMOTOR_FILTER 1

#define XbeeTest

// OCU speed filter-related values
#define MAX_DESIRED_SPEED 900 // [au], caps incoming signal from OCU
#define MIN_ACHEIVABLE_SPEED 50

float DT_speed(const kMotor motor);
static float GetNominalDriveEffort(const float desired_speed);
static int16_t GetDesiredSpeed(const kMotor motor);

static float closed_loop_effort[3] = {0, 0, 0};

static int desired_velocity_left = 0;
static int desired_velocity_right = 0;
static int desired_velocity_flipper = 0;

void closed_loop_control_init(void) {
    Nop();
    Nop();
    IC_Init(kIC01, M1_TACHO_RPN, 1000);
    IC_Init(kIC02, M2_TACHO_RPN, 1000);
    PID_Init(LEFT_CONTROLLER, MAX_EFFORT, MIN_EFFORT, K_P, K_I, K_D);
    PID_Init(RIGHT_CONTROLLER, MAX_EFFORT, MIN_EFFORT, K_P, K_I, K_D);
}

// this runs every 10ms
void handle_closed_loop_control(unsigned int OverCurrent) {

    static unsigned int stop_counter = 0;

    // If we have stopped the motors due to overcurrent, don't update speeds
    if (OverCurrent) {
        PID_Reset(kMotorLeft);
        PID_Reset(kMotorRight);
        return;
    }

    // Filter drive motor speeds
    float desired_speed_left = IIRFilter(LMOTOR_FILTER, GetDesiredSpeed(kMotorLeft), ALPHA, false);
    // printf("%f|",desired_speed_left);
    float desired_speed_right =
        IIRFilter(RMOTOR_FILTER, GetDesiredSpeed(kMotorRight), ALPHA, false);
#ifndef XbeeTest
    // if the user releases the joystick, come to a relatively quick stop by clearing the integral
    // term
    if ((abs(REG_MOTOR_VELOCITY.left) < 50) && (abs(REG_MOTOR_VELOCITY.right) < 50)) {
        PID_Reset_Integral(kMotorLeft);
        PID_Reset_Integral(kMotorRight);

        // If user releases joystick, reset the IIR filter
        IIRFilter(LMOTOR_FILTER, 0, ALPHA, YES);
        IIRFilter(RMOTOR_FILTER, 0, ALPHA, YES);
        desired_speed_left = 0;
        desired_speed_right = 0;
    }
#endif

#ifdef XbeeTest
    // if the user releases the joystick, come to a relatively quick stop by clearing the integral
    // term
    if ((abs(Xbee_MOTOR_VELOCITY[0]) < 50) && (abs(Xbee_MOTOR_VELOCITY[1]) < 50)) {
        PID_Reset_Integral(kMotorLeft);
        PID_Reset_Integral(kMotorRight);

        // If user releases joystick, reset the IIR filter
        IIRFilter(LMOTOR_FILTER, 0, ALPHA, true);
        IIRFilter(RMOTOR_FILTER, 0, ALPHA, true);
        desired_speed_left = 0;
        desired_speed_right = 0;
    }
#endif

    // update the flipper
    float desired_flipper_speed = desired_velocity_flipper / 1200.0;
    // DT_set_speed(kMotorFlipper, desired_flipper_speed);
    closed_loop_effort[kMotorFlipper] = desired_flipper_speed;

    // update the left drive motor
    float nominal_effort_left = GetNominalDriveEffort(desired_speed_left);
    float actual_speed_left = DT_speed(kMotorLeft);
    float effort_left = PID_ComputeEffort(LEFT_CONTROLLER, desired_speed_left, actual_speed_left,
                                          nominal_effort_left);
    // printf("%f",effort_left);
    // DT_set_speed(kMotorLeft, effort_left);
    closed_loop_effort[kMotorLeft] = effort_left;
    // printf("%d",closed_loop_effort[kMotorLeft]);
    // DT_set_speed(kMotorLeft, nominal_effort_left);

    // update the right drive motor
    float nominal_effort_right = GetNominalDriveEffort(desired_speed_right);
    float actual_speed_right = DT_speed(kMotorRight);
    float effort_right = PID_ComputeEffort(RIGHT_CONTROLLER, desired_speed_right,
                                           actual_speed_right, nominal_effort_right);
    // DT_set_speed(kMotorRight, effort_right);
    closed_loop_effort[kMotorRight] = effort_right;
    // DT_set_speed(kMotorRight, nominal_effort_right);

    /*i++;
    if(i>=100)
    {
      i=0;
    }
    actual_speed_array[i] = actual_speed_right;*/

    // if the speed inputs are 0, reset controller after 1 second
    // TODO: fix controller so that we don't get these small offsets
    if ((desired_velocity_left == 0) && (desired_velocity_right == 0)) {
        stop_counter++;
        if (stop_counter > 100) {
            PID_Reset(kMotorLeft);
            PID_Reset(kMotorRight);
            stop_counter = 0;
        }
    } else
        stop_counter = 0;
}

int return_closed_loop_control_effort(unsigned char motor) {
    // if(motor==1) return 300;
    return (int)(closed_loop_effort[motor] * 1000.0);
    // return 0;
}

float DT_speed(const kMotor motor) {
#define HZ_16US 100000.0

    float period = 0;
    switch (motor) {
    case kMotorLeft: {
        period = IC_period(kIC01);
        if (period != 0) {
            if (M1_DIRO)
                return -(HZ_16US / period);
            else
                return (HZ_16US / period);
        }
        break;
    }
    case kMotorRight: {
        period = IC_period(kIC02);
        if (period != 0) {
            if (M2_DIRO)
                return (HZ_16US / period);
            else
                return -(HZ_16US / period);
        }
        break;
    }
    case kMotorFlipper: {
        period = IC_period(kIC03);
        if (period != 0) {
            //        if (M3_DIRO) return (HZ_16US / period);
            if (M3_DIR)
                return (HZ_16US / period);
            else
                return -(HZ_16US / period);
        }
        break;
    }
    }
    return 0;
}

// Description: Returns the approximate steady-state effort required to
//   maintain the given desired speed of a drive motor.
static float GetNominalDriveEffort(const float desired_speed) {
    // NB: transfer function found empirically (see spreadsheet for data)
    if (desired_speed == 0)
        return 0;

    if (desired_speed < 0)
        return ((0.0007 * desired_speed) - 0.0067);
    else
        return ((0.0007 * desired_speed) + 0.0067);
}

// Description: Maps the incoming control data to suitable values
// Notes:
//   - special-cases turning in place to higher values to overcome
//     the additional torque b/c software change has too much overhead right now
static int16_t GetDesiredSpeed(const kMotor motor) {
    int16_t temp_left = desired_velocity_left / 4;
    int16_t temp_right = desired_velocity_right / 4;

    switch (motor) {
    case kMotorLeft:
        if (abs(temp_left) < MIN_ACHEIVABLE_SPEED) {
            return 0;
        }

        // if we are turning (sign bits do not match AND magnitudes are non-negligible)
        if (((temp_left >> 15) != (temp_right >> 15)) &&
            ((200 < abs(temp_left)) && (200 < abs(temp_right)))) {
            // ClearMotorHistory();
            if (0 < temp_left)
                return 500;
            else
                return -500;
        }

        if (MAX_DESIRED_SPEED < temp_left)
            return MAX_DESIRED_SPEED;
        else if (temp_left < -MAX_DESIRED_SPEED)
            return -MAX_DESIRED_SPEED;
        else
            return temp_left;
    case kMotorRight:
        if (abs(temp_right) < MIN_ACHEIVABLE_SPEED) {
            return 0;
        }

        if (((temp_left >> 15) != (temp_right >> 15)) &&
            ((200 < abs(temp_left)) && (200 < abs(temp_right)))) {
            // ClearMotorHistory();
            if (0 < temp_right)
                return 500;
            else
                return -500;
        }

        if (MAX_DESIRED_SPEED < temp_right)
            return MAX_DESIRED_SPEED;
        else if (temp_right < -MAX_DESIRED_SPEED)
            return -MAX_DESIRED_SPEED;
        else
            return temp_right;
    case kMotorFlipper:
        return REG_MOTOR_VELOCITY.flipper;
    }

    return 0;
}

void set_desired_velocities(int left, int right, int flipper) {

    desired_velocity_left = left;
    desired_velocity_right = right;
    desired_velocity_flipper = flipper;
    // printf("%d,%d,%d",desired_velocity_left,desired_velocity_right,desired_velocity_flipper);
}
