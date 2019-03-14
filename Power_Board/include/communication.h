// clang-format off
/// @file
/// UART communication between the robot and an external computer
///
/// Communicate with the firmware over UART at baud rate 57600.
///
/// Each message to the rover is 7 bytes:
///
/// 1. Start byte = 253
/// 2. Left motor speed - (0-124 = backwards, 125 = hard brake, 126-250 = forward)
/// 3. Right motor speed
/// 4. Flipper motor speed
/// 5. Command Verb
/// 6. Command Argument
/// 7. Checksum = 255 - (sum of all bytes except start byte) % 255
///
/// The rover only responds if command verb is 10. All values are 16-bit integers, unsigned unless
/// noted below.
///
/// The response is 5 bytes:
/// 1. Start byte = 253
/// 2. Data Element #
/// 3. Value (hi byte)
/// 4. Value (lo byte)
/// 5. Checksum = 255 - (sum of all bytes except start byte) % 255
///
/// Upon receiving a message, the rover will set the motor speeds and may also take an additional
/// action specified by the command verb. If no valid message is received is received for a while
/// (333ms), the motors should come to a halt.
///
/// ### UART Command Verbs
///
/// See @ref UARTCommand fer details
///
/// ### UART Data Elements
///
/// | #    | Name                           | Allowable data                                               | Comments                                                     |
/// | ---- | ------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
/// | 0    | battery (A+B) current (external) [^1] | 34 = 1A                                                     | Total current from batteries                                 |
/// |~~2~~ | ~~left motor speed~~           |                                                              |                                                              |
/// |~~4~~ | ~~right motor speed~~          |                                                              |                                                              |
/// | 6    | flipper position 1             | relative position of 15 - 330 degrees of one sensor          | Flipper position sensor 1                                    |
/// | 8    | flipper position 2             | relative position of 15 - 330 degrees of one sensor          | Flipper position sensor 2                                    |
/// | 10   | left motor current             | 34 = 1A                                                      |                                                              |
/// | 12   | right motor current            | 34 = 1A                                                      |                                                              |
/// | 14   | left motor encoder count       | 0 - 65535                                                    | May overflow or underflow. Increments when motor driven forward, decrements backward |
/// | 16   | right motor encoder count      | 0 - 65535                                                    | May overflow or underflow. Increments when motor driven forward, decrements backward |
/// |~~18~~| ~~motors fault flag~~          | Bit flags                                                    | (value & 0x0100) = left motor (value & 0x0001) = right motor |
/// | 20   | left motor temperature         | degrees celsius                                              |                                                              |
/// |~~22~~| ~~right motor temperature~~    | degrees celsius                                              |                                                              |
/// | 24   | battery A voltage (external) [^1] | 58 = 1V                                                      |                                                              |
/// | 26   | battery B voltage (external) [^1] | 58 = 1V                                                      |                                                              |
/// | 28   | left motor encoder interval    |                                                              | 0 when motor stopped. Else proportional to motor period (inverse motor speed) |
/// | 30   | right motor encoder interval   |                                                              |                                                              |
/// |~~32~~|~~flipper motor encoder interval~~|                                                              |                                                              |
/// | 34   | battery A state of charge      | 0-100 %                                                      | 0 = battery empty; 100 = battery fully charged               |
/// | 36   | battery B state of charge      | 0-100 %                                                      |                                                              |
/// | 38   | battery charging state         | 0 or 0xdada=56026                                            | 0 = not charging; 0xdada = charging                          |
/// | 40   | release version                | Structured decimal                                           | XYYZZ, where X=major version, Y=minor version, Z = patch version.<br />e.g. 10502 = version 1.05.02<br />The value 16421 will be reported for pre-1.3 versions|
/// | 42   | battery A current (external) [^1] | 0-1023, 34 = 1A                                              |                                                              |
/// | 44   | battery B current (external) [^1] | 0-1023, 34 = 1A                                              |                                                              |
/// | 46   | motor flipper angle            | 0-360, degrees (actual data range needs to be tested)        | Flipper angle                                                |
/// | 48   | fan speed                      | 0-240,                                                       | Actual fan speed, reported by fan controller                 |
/// |~~50~~|~~drive mode~~                  | 0 (open loop) or 1 (closed loop)                             |                                                              |
/// | 52   | battery A status               | Bit flags                                                    | Alarm bits:<br />* 0x8000 OVER_CHARGED_ALARM<br />* 0x4000 TERMINATE_CHARGE_ALARM<br />* 0x1000 OVER_TEMP_ALARM<br />* 0x0800 TERMINATE_DISCHARGE_ALARM<br />* 0x0200 REMAINING_CAPACITY_ALARM<br />* 0x0100 REMAINING_TIME_ALARM<br />Status bits:<br />* 0x0080 INITIALIZED<br />* 0x0040 DISCHARGING<br />* 0x0020 FULLY_CHARGED<br />* 0x0010 FULLY_DISCHARGED |
/// | 54   | battery B status               | Bit flags                                                    |                                                              |
/// | 56   | battery A mode                 | Bit flags                                                    | Bit 7 (value & 0x80) gives whether battery needs a condition cycle. Other values probably useless |
/// | 58   | battery B mode                 | Bit flags                                                    | Bit 7 (value & 0x80) gives whether battery needs a condition cycle. Other values probably useless |
/// | 60   | battery A temperature          | Temperature of battery above absolute 0 in deciKelvins       |                                                              |
/// | 62   | battery B temperature          | Temperature of battery above absolute 0 in deciKelvins       |                                                              |
/// | 64   | battery A voltage (internal) [^1] | Voltage of battery in mV                                     |                                                              |
/// | 66   | battery B voltage (internal) [^1] | Voltage of battery in mV                                     |                                                              |
/// | 68   | battery A current (internal) [^1] | (signed) Current of battery in mA                            | negative values for discharge, positive for charging         |
/// | 70   | battery B current (internal) [^1] | (signed) Current of battery in mA                            | negative values for discharge, positive for charging         |
/// | 72   | Left motor status              | Bit flags (MotorStatusFlag)                                     |                                                              |
/// | 74   | Right motor status             | Bit flags (MotorStatusFlag)                                     |                                                              |
/// | 76   | Flipper motor status           | Bit flags (MotorStatusFlag)                                     |                                                              |
///
/// [^1]: for battery reporting, "internal" means the value comes from the SmartBattery's internal sensor. "external" means the value is reported by circuitry outside the SmartBattery
// clang-format on

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>

/// Initialize UART module. Will cause _U1TXInterrupt() and _U1RXInterrupt() to occasionally be
/// called, which transfer data between a the device's hardware UART module and various software
/// buffers.
void uart_init();

/// If we have received data over UART into the software buffers.
/// May clear the inbound software buffer and populate the outbound software buffer
void uart_tick();

/// 1-byte command "verb" associated with UART inbound command.
/// There is also a 1-byte argument associated with some of these
typedef enum UARTCommand {
    /// Robot should not do anything besides obey the speed commands
    UART_COMMAND_NONE = 0,
    /// Robot should respond with the data element specified by argument.
    UART_COMMAND_GET = 10,
    /// Robot should set the cooling fan speed to the arg (0-240) for a while (333ms)
    UART_COMMAND_SET_FAN_SPEED = 20,
    /// Robot should restart immediately into the bootloader. If no attempt is made to communicate
    /// with the bootloader (10 seconds), rover will proceed to normal operation
    UART_COMMAND_RESTART = 230,
    /// @deprecated
    /// If arg = 0, rover will be driven in open loop mode (commanded speeds will be the direction
    /// and effort of the motor)
    /// If arg=1, rover will be driven in closed loop mode (commanded speeds will be the intended
    /// speed of the motor)
    UART_COMMAND_SET_DRIVE_MODE = 240,
    /// If arg = 230, calibrate the flipper and save the results to NVM. Note the robot must be
    /// manually cycled before it will accept additional commands.
    UART_COMMAND_FLIPPER_CALIBRATE = 250,
    /// Forget settings and reload default startup values from NVM
    UART_COMMAND_SETTINGS_RELOAD = 1,
    /// Current settings should be saved as the new default startup values in NVM
    UART_COMMAND_SETTINGS_COMMIT = 2,

    /// Set power polling interval in ms
    UART_COMMAND_SETTINGS_SET_POWER_POLL_INTERVAL = 3,
    /// Set OC trigger threshold in units of 100mA
    UART_COMMAND_SETTINGS_SET_OVERCURRENT_TRIGGER_THRESHOLD = 4,
    /// Set OC trigger duration in units of 5ms
    UART_COMMAND_SETTINGS_SET_OVERCURRENT_TRIGGER_DURATION = 5,
    /// Set OC recover threshold in units of 100mA
    UART_COMMAND_SETTINGS_SET_OVERCURRENT_RECOVER_THRESHOLD = 6,
    /// Set OC recover duration in units of 5ms
    UART_COMMAND_SETTINGS_SET_OVERCURRENT_RECOVER_DURATION = 7,
    /// Set PWM frequency (arg = new frequency in kHz)
    UART_COMMAND_SETTINGS_SET_PWM_FREQUENCY = 8,
} UARTCommand;

#endif
