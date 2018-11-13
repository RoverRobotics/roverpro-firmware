#ifndef DEVICE_ROBOT_MOTOR_H
#define DEVICE_ROBOT_MOTOR_H

#include "stdhdr.h"

/*****************************************************************************/
//*-----------------------------General Purpose------------------------------*/
#define CLEAR 0
#define SET 1
#define HI 1
#define LO 0
#define Set_ActiveLO 0
#define Clear_ActiveLO 1

/*****************************************************************************/
//*-----------------------------------PWM------------------------------------*/
/////constant
/// Period for PR# when prescale is 1:1
/// = (FCY / period / prescale) -1
/// e.g. Period 1000Hz = (16000000 / 1000 / 1) - 1
typedef enum TimerPeriod {
    PERIOD_50HZ = 533332,
    PERIOD_67HZ = 319999,
    PERIOD_200HZ = 79999,
    PERIOD_300HZ = 53332,
    PERIOD_400HZ = 39999,
    PERIOD_500HZ = 31999,
    PERIOD_600HZ = 26666,
    PERIOD_700HZ = 22856,
    PERIOD_800HZ = 19999,
    PERIOD_900HZ = 17777,
    PERIOD_1000HZ = 15999,
    PERIOD_1100HZ = 14544,
    PERIOD_1200HZ = 13332,
    PERIOD_1300HZ = 12307,
    PERIOD_1400HZ = 11428,
    PERIOD_1500HZ = 10666,
    PERIOD_1600HZ = 9999,
    PERIOD_1700HZ = 9411,
    PERIOD_1800HZ = 8888,
    PERIOD_1900HZ = 8420,
    PERIOD_2000HZ = 7999,
    PERIOD_2100HZ = 7618,
    PERIOD_10000HZ = 1599,
    PERIOD_20000HZ = 799,
    PERIOD_30000HZ = 532,
    PERIOD_50000HZ = 319,
} TimerPeriod;

/*****************************************************************************/

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
#define INTERVAL_MS_SPEED_UPDATE 5             // 200Hz
#define TIMEOUT_MS_USB 333                     // 3Hz--333ms
#define INTERVAL_MS_UART_FAN_SPEED_TIMEOUT 333 // 3Hz--333ms
#define INTERVAL_MS_SWITCH_DIRECTION 10        // 10ms
#define INTERVAL_MS_STATE_MACHINE 1            // 1KHz
#define INTERVAL_MS_CURRENT_FEEDBACK 1         // 1KHz
#define INTERVAL_MS_I2C2 100
#define INTERVAL_MS_I2C3 100
#define INTERVAL_MS_SFREG 4             // 250Hz
#define INTERVAL_MS_BATTERY_CHECK 1     // 1KHz
#define INTERVAL_MS_BATTERY_RECOVER 100 // 100ms

// constant for special usage
typedef enum BatteryState {
    CELL_OFF = 0,
    CELL_ON = 1,
} BatteryState;
typedef enum BatteryChannel {
    CELL_A = 0,
    CELL_B = 1,
} BatteryChannel;
#define BATTERY_CHANNEL_COUNT 2
// constant for pins

// output pin mapping
#define M1_PWM _RP24R
#define M2_PWM _RP2R
#define M3_PWM _RP25R

// UART pins
#define U1RX_RPn 6
#define U1TX_RPn _RP7R

// Analog pins
#define M1_TEMP_EN(a) _PCFG2 = !(a)
#define M1_CURR_EN(a) _PCFG3 = !(a)
#define M2_TEMP_EN(a) _PCFG0 = !(a)
#define M2_CURR_EN(a) _PCFG1 = !(a)
#define M3_TEMP_EN(a) _PCFG14 = !(a)
#define M3_CURR_EN(a) _PCFG15 = !(a)

#define VCELL_A_EN(a) _PCFG10 = !(a)
#define VCELL_B_EN(a) _PCFG11 = !(a)
#define CELL_A_CURR_EN(a) _PCFG12 = !(a)
#define CELL_B_CURR_EN(a) _PCFG13 = !(a)

#define M3_POS_FB_1_EN(a) _PCFG8 = !(a)
#define M3_POS_FB_2_EN(a) _PCFG9 = !(a)

// Configure outputs

// Main power bus MOSFET control pins
#define CELL_A_MOS_EN(a) _TRISD3 = !(a)
#define CELL_B_MOS_EN(a) _TRISD2 = !(a)

#define M1_DIR_EN(a) _TRISD6 = !(a)
#define M1_BRAKE_EN(a) _TRISD7 = !(a)
#define M1_MODE_EN(a) _TRISD9 = !(a)
#define M1_COAST_EN(a) _TRISD10 = !(a)

#define M2_DIR_EN(a) _TRISB4 = !(a)
#define M2_BRAKE_EN(a) _TRISB5 = !(a)
#define M2_MODE_EN(a) _TRISG9 = !(a)
#define M2_COAST_EN(a) _TRISG8 = !(a)

#define M3_DIR_EN(a) _TRISE3 = !(a)
#define M3_BRAKE_EN(a) _TRISF0 = !(a)
#define M3_MODE_EN(a) _TRISE4 = !(a)
#define M3_COAST_EN(a) _TRISF1 = !(a)

// functional pins
#define M1_DIRO _RD0
#define M1_DIR _LATD6
#define M1_BRAKE _LATD7
#define M1_MODE _LATD9
#define M1_COAST _LATD10
#define M1_FF1 _RC14
#define M1_FF2 _RC13
#define M2_DIRO _RE5
#define M2_DIR _LATB4
#define M2_BRAKE _LATB5
#define M2_MODE _LATG9
#define M2_COAST _LATG8
#define M2_FF1 _RG6
#define M2_FF2 _RG7

// need to add M3_DIRO, M3_FF1, M3_FF2?
#define M3_DIR _LATE3
#define M3_BRAKE _LATF0
#define M3_MODE _LATE4
#define M3_COAST _LATF1

#define Cell_A_MOS _LATD3
#define Cell_B_MOS _LATD2

// other constant
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_FLIPPER,
} MotorChannel;
#define MOTOR_CHANNEL_COUNT 3
/// Helper macro for iterating all motors and storing the result in variable i.
/// e.g. int k; for (EACH_MOTOR_CHANNEL(k))
#define EACH_MOTOR_CHANNEL(i)                                                                      \
    i = 0;                                                                                         \
    i < MOTOR_CHANNEL_COUNT;                                                                       \
    i++

/// Number of samples to keep of running metrics for power management, like battery temperature and
/// voltage
#define SAMPLE_LENGTH 4
/// Number of samples to keep of running metrics for speed control
#define SAMPLE_LENGTH_CONTROL 8

/// Physical addresses of I2C devices
enum I2CDeviceAddress {
    /// I2C Bus 2 address of fan controller Maxim MAX6615AEE+. Datasheet with messaging info:
    /// https://www.mouser.com/datasheet/2/256/MAX6615-MAX6616-370370.pdf
    FAN_CONTROLLER_ADDRESS = 0b0011000,
    /// I2C Bus 2+3 batteries (one on I2C bus 2, one on bus 3). These are logically separate devices
    /// but they live in the same plastic housing.
    /// Batteries may be one of several models. Communicate using the SmartBattery Data
    /// specification http://sbs-forum.org/specs/sbdat110.pdf
    BATTERY_ADDRESS = 0b0001011,
    /// I2C Bus 2 EEPROM. Not in use.
    EEPROM_ADDRESS = 0b1010000,
    /// I2C Bus 2 Temperature Sensor. Not in use.
    TEMPERATURE_SENSOR_ADDRESS = 0b1001001,
    /// I2C Bus 3 Battery Charger. See Internal_Charger firmware for slave device logic. Should
    /// respond to I2C ReadWord 0xca with value 0xdada to indicate that charging is active.
    BATTERY_CHARGER_ADDRESS = 0b0001100,
};

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
void PWM1Ini(void); ///< Initialize PWM channel 1 (left motor)
void PWM2Ini(void); ///< Initialize PWM channel 2 (right motor)
void PWM3Ini(void); ///< Initialize PWM channel 3 (flipper motor)

// BEGIN utility functions
void PWM1Duty(uint16_t duty); ///< Set duty cycle for channel 1 (left motor). 0 <= Duty <= 1000
void PWM2Duty(uint16_t duty); ///< Set duty cycle for channel 2 (right motor).  0 <= Duty <= 1000
void PWM3Duty(
    uint16_t duty); ///< Set duty cycle for pwm channel 3 (flipper motor). 0 <= Duty <= 1000

/// Tell motor controller to coast motor
void Coasting(MotorChannel channel);
/// Tell motor controller to brake motor
void Braking(MotorChannel channel);
/// If overcurrent, Communicate new motor speeds to the motor controller.
void UpdateSpeed(MotorChannel channel, MotorState state);

void Device_MotorController_Process();
uint16_t GetDuty(int16_t target, MotorChannel channel);

/// Compute the current used by a given motor and populate it in current_for_control
/// Note this is computed as a windowed average of the values in motor_current_ad
void GetCurrent(MotorChannel Channel);

/// Enable/disable a particular battery on the power bus
void Cell_Ctrl(BatteryChannel Channel, BatteryState state);

/// Take the motor speed values from USB/UART and populate motor_target_speed.
/// If we are using PID, the PID effort is computed here
void USBInput();

#endif
