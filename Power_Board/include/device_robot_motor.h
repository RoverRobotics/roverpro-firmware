#ifndef DEVICE_ROBOT_MOTOR_H
#define DEVICE_ROBOT_MOTOR_H

#include "stdhdr.h"
#include "hardware_definitions.h"


/*****************************************************************************/
//*-----------------------------------PWM------------------------------------*/

typedef enum MotorEvent {
    STOP = 0xFF01,
    GO = 0xFF02,
    BACK = 0xFF03,
    NO_EVENT = 0xFF00,
} MotorEvent;

typedef enum MotorState {
    /// Motor is moving in forward direction
    FORWARD = 0xEE00,
    /// Motor is applying break
    BRAKE = 0xEE01,
    /// Motor is transitioning between states
    SWITCH_DIRECTION = 0xEE02,
    /// Motor is moving backward
    BACKWARD = 0xEE03,
} MotorState;

// constant for timer.
// Since these are on timer1, they are in multiples of PR1
#define INTERVAL_MS_SPEED_UPDATE 5                  // 200Hz
#define INTERVAL_MS_USB_TIMEOUT 333                 // 3Hz--333ms
#define INTERVAL_MS_UART_FAN_SPEED_TIMEOUT 333      // 3Hz--333ms
#define INTERVAL_MS_SWITCH_DIRECTION 10             // 10ms
#define INTERVAL_MS_MOTOR_DIRECTION_STATE_MACHINE 1 // 1KHz
#define INTERVAL_MS_CURRENT_FEEDBACK 1              // 1KHz
#define INTERVAL_MS_I2C2 100
#define INTERVAL_MS_I2C3 100
#define INTERVAL_MS_SFREG 4             // 250Hz
#define INTERVAL_MS_BATTERY_CHECK 1     // 1KHz
#define INTERVAL_MS_BATTERY_RECOVER 100 // 100ms

/// Number of samples to keep of running metrics for power management, like battery temperature and
/// voltage
#define SAMPLE_LENGTH 4
/// Number of samples to keep of running metrics for speed control
#define SAMPLE_LENGTH_CONTROL 8

extern bool usb_new_data_received;

// INTERRUPTS:
/// Timer interrupt to enable ADC every tick
void Motor_T3Interrupt(void);
/// Analog/digital converter interrupt.
void Motor_ADC1Interrupt(void);

/*****************************************************************************/
//*--------------------------------General Functions-------------------------*/
// BEGIN initialization routines

/// Perform initialization of this module
void DeviceRobotMotorInit();
/// Initialize interrupts
void InterruptIni();
/// Remaps pins for UART and PWM
void PinRemap(void);

/// Configure fan controller with I2C commands
void FANCtrlIni();

/// Configure and start analog/digital converter
void IniAD();
/// Set up and start Timer1: 1 kHz, no interrupts
void IniTimer1();
/// Set up and start Timer2: 30 kHz, no interrupts.
/// Timer2 is used as the clock source for motor PWM
void IniTimer2();
/// Set up and start Timer3: 50 kHz, Interrupts enabled
/// Timer3 is used as the trigger source for ADC and back emf feedback
void IniTimer3();

/// Tick process which does a lot more than just control the motor
void Device_MotorController_Process();

/// Take the motor speed values from USB/UART and populate motor_target_speed.
/// If we are using closed loop control scheme, the PID effort is computed here.
/// either way we set motor_efforts based on REG_MOTOR_VELOCITY
void set_motor_control_scheme();

#endif
