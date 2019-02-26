#ifndef DEVICE_ROBOT_MOTOR_H
#define DEVICE_ROBOT_MOTOR_H

#include "stdhdr.h"
#include "hardware_definitions.h"

/// For motor state machine. The requested motor state based in inbound motor speed commands, which
/// informs state machine transitions
typedef enum MotorEvent {
    /// Motor has been commanded to stop
    STOP = 0xFF01,
    /// Motor has been commanded to move forward
    GO = 0xFF02,
    /// Motor has been commanded to move backward
    BACK = 0xFF03,
    /// Motor has not been commanded
    NO_EVENT = 0xFF00,
} MotorEvent;

/// For motor state machine. The current motor state.
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

/// Whether we have received new commands from the USB interface (not the UART)
extern bool usb_new_data_received;

/*****************************************************************************/
// BEGIN initialization routines

/// Perform initialization of this module
void DeviceRobotMotorInit();

/// Configure fan controller with I2C commands
void FANCtrlIni();

/// Configure and start analog/digital converter
void IniAD();

/// Set up and start Timer2: 30 kHz, no interrupts.
/// Timer2 is used as the clock source for motor PWM
void IniTimer2();
/// Set up and start Timer3: 50 kHz, Interrupts enabled
/// Timer3 is used as the trigger source for ADC and back-EMF feedback
void IniTimer3();

/// Tick process which does a lot more than just control the motor
void Device_MotorController_Process();

/// Take the motor speed values from USB/UART and populate motor_target_speed.
/// If we are using closed loop control scheme, the PID effort is computed here.
/// either way we set motor_efforts based on REG_MOTOR_VELOCITY
void set_motor_control_scheme();

#endif
