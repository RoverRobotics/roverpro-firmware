#ifndef DEVICE_ROBOT_MOTOR_H
#define DEVICE_ROBOT_MOTOR_H

#include "stdhdr.h"
#include "hardware_definitions.h"

// constant for timer.
// Since these are on timer1, they are in multiples of PR1
#define INTERVAL_MS_SPEED_UPDATE 5                  // 200Hz
#define INTERVAL_MS_USB_TIMEOUT 333                 // 3Hz--333ms
#define INTERVAL_MS_UART_FAN_SPEED_TIMEOUT 333      // 3Hz--333ms
#define INTERVAL_MS_SWITCH_DIRECTION 10             // 10ms
#define INTERVAL_MS_MOTOR_DIRECTION_STATE_MACHINE 1 // 1KHz
#define INTERVAL_MS_CURRENT_FEEDBACK 1              // 1KHz
#define INTERVAL_MS_SFREG 4                         // 250Hz

/// Whether we have received new commands from the USB interface (not the UART)
extern bool usb_new_data_received;

/*****************************************************************************/
// BEGIN initialization routines

/// Perform initialization of this module
void DeviceRobotMotorInit();

/// Configure fan controller with I2C commands
void FANCtrlIni();

/// Tick process which does a lot more than just control the motor
void Device_MotorController_Process();

/// Take the motor speed values from USB/UART and populate motor_target_speed.
/// If we are using closed loop control scheme, the PID effort is computed here.
/// either way we set motor_efforts based on REG_MOTOR_VELOCITY
void set_motor_control_scheme();

#endif
