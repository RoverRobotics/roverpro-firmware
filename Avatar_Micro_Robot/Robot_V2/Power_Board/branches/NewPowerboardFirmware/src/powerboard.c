/*==============================================================================
File: Powerboard.c

Description: This is the overarching file that encapsulates the 
  application-level firmware for the powerboard (aka motor driver board) within
  the robot.

Features
  - passes control of the fan up to software
  - calibrate flipper
  - actuate flipper, left drive motor, right drive motor
  - pass SoC of the batteries up to software
  - closed-loop control
  - prevents overcurrenting of battery
  
Notes:
  - see USB diagram
  - see I2C diagram
  - the carrier board provides the regulated logic-level voltages to the
    power board
  - the temperature sensor (TMP112) is not used right now
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#define TEST_POWERBOARD
//---------------------------Dependencies---------------------------------------
#include "./core/StandardHeader.h"
#include "./core/Timers.h"              // for timers
#include "./core/ADC.h"                 // for A/D conversions
#include "./core/TWI.h"                 // for I2C/SMBus library
#include "./SMBusDevices/SBS.h"         // for smart battery system macros
#include "./SMBusDevices/MAX6615AEE.h"  // for fan controller macros
#include "./SMBusDevices/RXCharger.h"   // for internal charger macros
#include "./USB/RXUSBDevice.h"          // for USB communication
#include "./PID.h"                      // for motor speed control
#include "./Flash/DEE Emulation 16-bit.h"   // for MCU persistent storage
#include "./Filters.h"                  // for IIRFilter()
#include <string.h>                     // for strncmp()
#include <stdlib.h>                     // for abs()
#include <math.h>                       // for fabsf()
#include "./drivetrain.h"               // to control drive motors and flipper

//---------------------------Macros---------------------------------------------
#define POWERBOARD_PRODUCT_ID 0x03      // for USB

// power management pin(s) and threshold(s)
#define SIDEA_CONNECT_EN(a)	(_TRISD3 = !(a))
#define SIDEB_CONNECT_EN(a)	(_TRISD2 = !(a))
#define SIDEA_CONNECT 	    _LATD3
#define SIDEB_CONNECT 	    _LATD2

// PID controller values
#define LEFT_CONTROLLER     0
#define RIGHT_CONTROLLER    1

#define MAX_EFFORT          1.00        // maximum control effort magnitude (can also be -1000)
#define MIN_EFFORT          -1.00
#define K_P                 0.0003      // proportional gain
#define K_I                 0.00001     // integral gain
#define K_D                 0.0         // differential gain

// filters
#define ALPHA               0.8
#define LMOTOR_FILTER       0
#define RMOTOR_FILTER       1

// OCU speed filter-related values
#define MAX_DESIRED_SPEED   900         // [au], caps incoming signal from OCU

//---analog sensing pins (ANx)
#define M1_T_PIN            2           // motor1 temperature sensing
#define M1_I_PIN            3           // motor1 current sensing
#define M2_T_PIN            0           // motor2 temperature sensing
#define M2_I_PIN            1           // motor2 current sensing
#define M3_T_PIN            14          // motor3 temperature sensing
#define M3_I_PIN            15          // motor3 current sensing
#define SIDEA_I_PIN         12          // battery sideA current sensing
#define SIDEB_I_PIN         13          // battery sideB current sensing
#define M3_POS_FB_1_PIN     8           // motor3 position feeback1
#define M3_POS_FB_2_PIN     9           // motor3 position feedback2

// flipper calibration values
#define FLIPPER_CALIBRATION_SENTINEL    12345   // special speed value from software
#define FLIPPER_CALIBRATED  0xAA        // written to flash
// NB: These pot thresholds are very wide because the
// flipper pots are on a different 3.3V supply than the PIC
#define LOW_POT_THRESHOLD   33          // invalid below this threshold
#define HIGH_POT_THRESHOLD  990         // invalid above this threshold
#define FLIPPER_POT_OFFSET  -55

//---I2C 7-bit slave address(es)
#define TMP112_ADDRESS      0x49        // also test 0x48
#define FAN_ADDRESS         0x18        // pin-programmable

#define DEFAULT_DATA        0xFF        // default byte for communication

//---timer(s)
#define _10ms               10
#define _100ms              100
#define I2C2_TIMEOUT_TIMER  0
#define I2C2_TIMEOUT_TIME   (_100ms)
#define I2C3_TIMEOUT_TIMER  1
#define I2C3_TIMEOUT_TIME   (100*_100ms)
#define CONTROL_TIMER       2 
#define CONTROL_TIME        (_10ms)
#define SW_UPDATE_TIMER     3           // to update software
#define SW_UPDATE_TIME      (5*_10ms)
//#define CURRENT_CHECK_TIMER 4
//#define CURRENT_CHECK_TIME  4           // [ms]
#define SOCA_TIMER          5
#define SOCA_TIME           (55*_100ms) // NB: wait at least 5s before 
#define SOCB_TIMER          6           // requesting updated values
#define SOCB_TIME           (70*_100ms)
#define COOLDOWN_TIMER      7           // for after nearly overcurrenting
#define COOLDOWN_TIME       (_100ms)  // the battery
#define T1_TIMER            8           // for sampling thermistor1
#define T1_TIME             (_100ms) 
#define T2_TIMER            9           // for sampling thermistor2
#define T2_TIME             (_100ms)
#define CHARGER_CHECK_TIMER 10          // for checking the status of a
#define CHARGER_CHECK_TIME  (5*_10ms)   // potential internal charger
#define PBTOGGLE_TIMER      11
#define PBTOGGLE_TIME       (30*_100ms)

//---------------------------Constants------------------------------------------
// [au], maximum allowable current draw from an individual side
// 0.01S*(17A*0.001Ohm)*11k ~= 1.87V, (1.87 / 3.30) * 1023=
//static const kADCReading kMaxSideCurrent = 580;
static const kADCReading kMaxSideCurrent = 520;


// potential smart battery device names (reversed) used during initialization
static const int8_t old_bat_str[8] = {'0','9','5','2','-','B','B', 0x07};
static const int8_t new_bat_str[10] = {'B','1','9','7','0','7','-','T','B', 0x0A};
static const int8_t rx_bat_str[8] = {'X','E','T','O','B','O','R', 0x07};

// I = (au / 1023) * 3.30 / 0.01 / 11e3 / 0.001
//---------------------------Type Definitions-----------------------------------
// possible states
typedef enum {
  kWaiting = 0,
  kRunning,
  kCoolingDown,
} kPowerboardState;

// battery options
typedef enum {
  kBatteryBB2590 = 0,
  kBatteryBT70791B,
  kBatteryRoboteX,
  kBatteryUnknown,
} kBattery;

void InitPowerboard(void);
void ProcessPowerboardIO(void);

//---------------------------Helper Function Prototypes-------------------------
static void UpdateMotorSpeeds(void);
static void InitPins(void);
static void InitI2CDevices(void);
static void InitFan(void);
static bool DidApproachOvercurrent(void);
static void UpdateSensors(void);
static void UpdateBus2Sensors(void);
static void UpdateBus3Sensors(void);
static void UpdateSoftware(void);
//static inline float DecodeTemperatureData(TWIDevice* device);
static void set_fan_speed(const uint8_t speed);

//---startup-related
static void TurnOnPowerBus(const kBattery battery);
static kBattery DetermineBatteryType(void);
static void RunOldPowerup(void);
static void RunNewPowerup(void);
static void RunHybridPowerup(void);
static void CalibrateFlipper(void);
static uint16_t flipper_angle_offset(void);
static uint16_t flipper_angle(void);
static uint16_t CombinePotAngle(uint16_t pot1_value,
                                uint16_t pot2_value);

static void TogglePowerbus(void);

//---control-related
static float GetNominalDriveEffort(const float desired_speed);
static int16_t GetDesiredSpeed(const kMotor motor);

//---------------------------Module Variables-----------------------------------
static uint16_t lmotor_temperature = DEFAULT_DATA;
static uint16_t rmotor_temperature = DEFAULT_DATA;

static uint8_t dummyFanData[1] = {DEFAULT_DATA};
extern uint8_t dummySOCDataA[2] = {DEFAULT_DATA, DEFAULT_DATA};
extern uint8_t dummySOCDataB[2] = {DEFAULT_DATA, DEFAULT_DATA};
extern uint8_t dummyChargerStatusData[2] = {DEFAULT_DATA, DEFAULT_DATA};

static TWIDevice fan = {
  .address = FAN_ADDRESS,
  .subaddress = 0,
  .n_data_bytes = 1,  // fan controller is read-byte
  .data = dummyFanData
};

static TWIDevice sideA = {
  .address = SBS_SLAVE_ADDRESS,
  .subaddress = SBS_RELATIVE_SOC,
  .n_data_bytes = 2,
  .data = dummySOCDataA
};

// NB: this is on a different bus since has same slave address
static TWIDevice sideB = {
  .address = SBS_SLAVE_ADDRESS,
  .subaddress = SBS_RELATIVE_SOC,
  .n_data_bytes = 2,
  .data = dummySOCDataB
};

static TWIDevice charger = {
  .address = RXCHARGER_ADDRESS,
  .subaddress = RXCHARGER_STATUS,
  .n_data_bytes = 2,
  .data = dummyChargerStatusData
};

static volatile kPowerboardState state = kWaiting;

//---------------------------Test Harness---------------------------------------
#ifdef TEST_POWERBOARD
#include "./core/ConfigurationBits.h"
int main(void) {
  //_SWDTEN = 1; // enable the watchdog timer
  InitPowerboard();
  RXUSBDevice_Init(POWERBOARD_PRODUCT_ID);
  while (1) {
    //ClrWdt();
    ProcessPowerboardIO();
    RXUSBDevice_ProcessMessage();
  }
  
  return 0;
}
#endif

void InitPowerboard(void) {
  InitPins();
  // NB: we NEED TWI to determine the battery type
  TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
  TWI_Init(kTWI03, kTWIBaudRate100kHz, SMBUS);
  TurnOnPowerBus(DetermineBatteryType());

  // initialize any dependent module(s)
  ADC_Init((1 << M1_T_PIN)        |
           (1 << M1_I_PIN)        |
           (1 << M2_T_PIN)        |
           (1 << M2_I_PIN)        |
           (1 << M3_T_PIN)        |
           (1 << M3_I_PIN)        |
           (1 << SIDEA_I_PIN)     |
           (1 << SIDEB_I_PIN)     |
           (1 << M3_POS_FB_1_PIN) |
           (1 << M3_POS_FB_2_PIN));
  TMRS_Init();
  DT_Init(kMotorLeft);
  DT_Init(kMotorRight);
  DT_Init(kMotorFlipper);
 	PID_Init(LEFT_CONTROLLER, MAX_EFFORT, MIN_EFFORT, K_P, K_I, K_D);
 	PID_Init(RIGHT_CONTROLLER, MAX_EFFORT, MIN_EFFORT, K_P, K_I, K_D);
 	
 	InitI2CDevices();
}


void ProcessPowerboardIO(void) { 
  // ensure we do NOT exceed the current limit of the battery
  if (DidApproachOvercurrent()) {
    // stop driving all motors
    DT_set_speed(kMotorLeft, 0.0);
    DT_set_speed(kMotorRight, 0.0);
    DT_set_speed(kMotorFlipper, 0.0);
    
    // clear the history in the rolling averages
    IIRFilter(LMOTOR_FILTER, 0, 0, YES);
    IIRFilter(RMOTOR_FILTER, 0, 0, YES);
    
    // clear the history in the controllers
    PID_Reset(LEFT_CONTROLLER);
    PID_Reset(RIGHT_CONTROLLER);
    
    TMRS_StartTimer(COOLDOWN_TIMER, COOLDOWN_TIME);
    state = kCoolingDown;
  }
  
  // if software desires, calibrate the flipper position
  if (REG_MOTOR_VELOCITY.flipper == FLIPPER_CALIBRATION_SENTINEL) {
    CalibrateFlipper();
  }
  
  // only permit one side of the power bus to be on at a time
  if (charger.data[0] == RXCHARGER_ALIVE &&
      TMRS_IsTimerExpired(PBTOGGLE_TIMER)) {
    TMRS_StartTimer(PBTOGGLE_TIMER, PBTOGGLE_TIME);
    TogglePowerbus();
  }
  
  DT_Run();
  UpdateSensors();
  
  switch (state) {
    case kWaiting:
      state = kRunning;
      break;
    case kRunning:
      // update the motor speeds as desired by software
      if (TMRS_IsTimerExpired(CONTROL_TIMER)) {
        TMRS_StartTimer(CONTROL_TIMER, CONTROL_TIME);
        UpdateMotorSpeeds();
      }
      // update software with firmware feedback
      if (TMRS_IsTimerExpired(SW_UPDATE_TIMER)) {
        TMRS_StartTimer(SW_UPDATE_TIMER, SW_UPDATE_TIME);
        UpdateSoftware();
      }
      break;
    case kCoolingDown:
      if (TMRS_IsTimerExpired(COOLDOWN_TIMER)) {
        state = kRunning;
        return;
      }
      break;
  }
}

//---------------------------Helper Function Definitions------------------------
static void UpdateMotorSpeeds(void) {
  // update the flipper
  float desired_flipper_speed = REG_MOTOR_VELOCITY.flipper / 1200.0;
  DT_set_speed(kMotorFlipper, desired_flipper_speed);
 
  // update the left drive motor
  float desired_speed_left = IIRFilter(LMOTOR_FILTER, GetDesiredSpeed(kMotorLeft), ALPHA, NO);
  float nominal_effort_left = GetNominalDriveEffort(desired_speed_left);
  //float actual_speed_left = DT_speed(kMotorLeft);
  //float effort_left = PID_ComputeEffort(LEFT_CONTROLLER, desired_speed_left, actual_speed_left, nominal_effort_left);
  //DT_set_speed(kMotorLeft, effort_left);
  DT_set_speed(kMotorLeft, nominal_effort_left);
  
  // update the right drive motor
  float desired_speed_right = IIRFilter(RMOTOR_FILTER, GetDesiredSpeed(kMotorRight), ALPHA, NO);
  float nominal_effort_right = GetNominalDriveEffort(desired_speed_right);
  //float actual_speed_right = DT_speed(kMotorRight);
  //float effort_right = PID_ComputeEffort(RIGHT_CONTROLLER, desired_speed_right, actual_speed_right, nominal_effort_right);
  //DT_set_speed(kMotorRight, effort_right);
  DT_set_speed(kMotorRight, nominal_effort_right);
}

// Description: Maps the incoming control data to suitable values
// Notes:
//   - special-cases turning in place to higher values to overcome
//     the additional torque b/c software change has too much overhead right now
static int16_t GetDesiredSpeed(const kMotor motor) {
  int16_t temp_left = REG_MOTOR_VELOCITY.left;
  int16_t temp_right = REG_MOTOR_VELOCITY.right;
  
  switch (motor) {
    case kMotorLeft:
      // if the sign bits do not match AND magnitudes are non-negligible
      // we are turning in place
      if (((temp_left >> 15) != (temp_right >> 15)) && 
           ((200 < abs(temp_left)) && (200 < abs(temp_right)))) {
        if (0 < temp_left) return 500;
        else return -500;
      }
      if (MAX_DESIRED_SPEED < temp_left) return MAX_DESIRED_SPEED;
      else if (temp_right < -MAX_DESIRED_SPEED) return -MAX_DESIRED_SPEED;
      else return temp_left;
    case kMotorRight:
      if (((temp_left >> 15) != (temp_right >> 15)) &&
          ((200 < abs(temp_left)) && (200 < abs(temp_right)))) {
        if (0 < temp_right) return 500;
        else return -500;
      }
      if (MAX_DESIRED_SPEED < temp_right) return MAX_DESIRED_SPEED;
      else if (temp_right < -MAX_DESIRED_SPEED) return -MAX_DESIRED_SPEED;
      else return temp_right;
    case kMotorFlipper: return REG_MOTOR_VELOCITY.flipper;
  }
  
  return 0;
}

// Description: Returns the approximate steady-state effort required to 
//   maintain the given desired speed of a drive motor.
static float GetNominalDriveEffort(const float desired_speed) {
  // NB: transfer function found empirically (see spreadsheet for data)  
  if (desired_speed == 0) return 0;
  
  if (desired_speed < 0) return ((0.0007 * desired_speed) - 0.0067);
  else return ((0.0007 * desired_speed) + 0.0067);
}

static void UpdateSensors(void) {
  UpdateBus2Sensors();
  UpdateBus3Sensors();
}

static void UpdateBus2Sensors(void) {
  // 0 = waiting, 1 = waiting for device 1 response, 2 = waiting for device 2 response, ...
  static uint8_t bus2_state = 0;
  
  // if an error has occurred
  if (TWI_ErrorHasOccurred(kTWI02) || TMRS_IsTimerExpired(I2C2_TIMEOUT_TIMER)) {
	  // restart the I2C module
	  TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
	  // clear our currently-stored data from devices on this bus
	  sideA.data[0] = DEFAULT_DATA; sideA.data[1] = DEFAULT_DATA;
	  fan.data[0] = DEFAULT_DATA;
	  lmotor_temperature = DEFAULT_DATA; rmotor_temperature = DEFAULT_DATA;
	  bus2_state = 0;
    TMRS_StartTimer(I2C2_TIMEOUT_TIMER, I2C2_TIMEOUT_TIME);
	  return;
  }
	
  switch (bus2_state) {
    case 0:
      TMRS_StartTimer(I2C2_TIMEOUT_TIMER, I2C2_TIMEOUT_TIME);
    	// request data
      // poll the SoC of battery side A
      if (TWI_IsBusIdle(kTWI02) && TMRS_IsTimerExpired(SOCA_TIMER)) {
        TMRS_StartTimer(SOCA_TIMER, SOCA_TIME);
        TWI_RequestData(kTWI02, &sideA);
        bus2_state = 1;
        return;
      }
      // poll thermistor1 on the fan
      if (TWI_IsBusIdle(kTWI02) && TMRS_IsTimerExpired(T1_TIMER)) {
        TMRS_StartTimer(T1_TIMER, T1_TIME);
        fan.subaddress = MAX6615AEE_TEMP1;
        TWI_RequestData(kTWI02, &fan);
        bus2_state = 2;
        return;
      }
      // poll thermistor2 on the fan
      if (TWI_IsBusIdle(kTWI02) && TMRS_IsTimerExpired(T2_TIMER)) {
        TMRS_StartTimer(T2_TIMER, T2_TIME);
        fan.subaddress = MAX6615AEE_TEMP2;
        TWI_RequestData(kTWI02, &fan);
        bus2_state = 3;
        return;
      }
      
      // send commands
      // update the fan speed
    	if (TWI_IsBusIdle(kTWI02)) {
      	// BUG ALERT: THIS ONLY SUPPORTS 1 DEVICE TO SEND COMMANDS TO
        set_fan_speed(REG_MOTOR_SIDE_FAN_SPEED);
        bus2_state = 0;
      }
      break;
    case 1:
      if (TWI_IsNewDataAvailable(kTWI02)) {
        TWI_GetData(kTWI02, &sideA);
        bus2_state = 0;
      }
     break;
    case 2:
      if (TWI_IsNewDataAvailable(kTWI02)) {
        TWI_GetData(kTWI02, &fan);
        // NB: result has resolution of 1deg Celsius
        // NB: data comes in reversed
        lmotor_temperature = fan.data[0];
        bus2_state = 0;
      }
      break;
    case 3:
      if (TWI_IsNewDataAvailable(kTWI02)) {
        TWI_GetData(kTWI02, &fan);
        rmotor_temperature = fan.data[0];
        bus2_state = 0;
      }
      break;
    default: bus2_state = 0; break;
  }
}

// Description: This function manages the polling of devices on I2C bus 3
//   (see the powerpoint documenting devices on each bus).
//    TODO: MAKE ME BETTER.  DOES NOT CURRENTLY SCALE TO MANY DEVICES
//    TODO: break out serial_communication.h which keeps track of polling everything
//              and abstracts I2C, UART, and SPI-related
static void UpdateBus3Sensors(void) {
  static uint8_t bus3_state = 0;
  
  // if an error has occurred
  if (TWI_ErrorHasOccurred(kTWI03) || TMRS_IsTimerExpired(I2C3_TIMEOUT_TIMER)) {
	  // restart the I2C module
	  TWI_Init(kTWI03, kTWIBaudRate100kHz, SMBUS);
    // clear any currently-stored data from devices on this bus
	  sideB.data[0] = DEFAULT_DATA; sideB.data[1] = DEFAULT_DATA;
	  charger.data[0] = DEFAULT_DATA; charger.data[1] = DEFAULT_DATA;
	  bus3_state = 0;
	  TMRS_StartTimer(I2C3_TIMEOUT_TIMER, I2C3_TIMEOUT_TIME);
    return;
  }
  
  switch (bus3_state) {
    case 0:
      TMRS_StartTimer(I2C3_TIMEOUT_TIMER, I2C3_TIMEOUT_TIME);
      // poll the SoC of battery side B
      if (TWI_IsBusIdle(kTWI03) && TMRS_IsTimerExpired(SOCB_TIMER)) {
        TMRS_StartTimer(SOCB_TIMER, SOCB_TIME);
        TWI_RequestData(kTWI03, &sideB);
        bus3_state = 1;
        return;
      }
      // poll a potential internal charger
      if (TWI_IsBusIdle(kTWI03) && TMRS_IsTimerExpired(CHARGER_CHECK_TIMER)) {
        TMRS_StartTimer(CHARGER_CHECK_TIMER, CHARGER_CHECK_TIME);
        TWI_RequestData(kTWI03, &charger);
        bus3_state = 2;
        return;
      }
      break;
    case 1:
      if (TWI_IsNewDataAvailable(kTWI03)) {
        TWI_GetData(kTWI03, &sideB);
        bus3_state = 0;
      }
      break;
    case 2:
      if (TWI_IsNewDataAvailable(kTWI03)) {
        TWI_GetData(kTWI03, &charger);
        bus3_state = 0;
      }
      break;
    default: bus3_state = 0; break;
  }
}

// Description: Updates software with firmware feedback via shared USB
//   registers.
static void UpdateSoftware(void) {
  // update the state of charge of each battery side
  REG_ROBOT_REL_SOC_A = sideA.data[1];
  REG_ROBOT_REL_SOC_B = sideB.data[1];
  
  // update the current being pulled from the batteries
  REG_PWR_A_CURRENT = ADC_value(SIDEA_I_PIN);
  REG_PWR_B_CURRENT = ADC_value(SIDEB_I_PIN);
  
  // update the flipper position and angle
  REG_FLIPPER_FB_POSITION.pot1 = (ADC_value(M3_POS_FB_1_PIN) >> 2);  // disregard 2 lsb's from 10-bit A/D
 	REG_FLIPPER_FB_POSITION.pot2 = (ADC_value(M3_POS_FB_2_PIN) >> 2);
  REG_MOTOR_FLIPPER_ANGLE = flipper_angle();
  
  // update the current on the motors
 	REG_MOTOR_FB_CURRENT.left = ADC_value(M1_I_PIN);
 	REG_MOTOR_FB_CURRENT.right = ADC_value(M2_I_PIN);
  REG_MOTOR_FB_CURRENT.flipper = ADC_value(M3_I_PIN);
  
  // update the motor thermistors
  REG_MOTOR_TEMP.left = lmotor_temperature;
  REG_MOTOR_TEMP.right = rmotor_temperature;
  //REG_MOTOR_TEMP.board = ; // NOT YET IMPLEMENTED, UNUSED
  
  // update whether we're on the charging dock
  REG_MOTOR_CHARGER_STATE = (charger.data[1] << 8) + charger.data[0];
}

static bool DidApproachOvercurrent(void) {
  return ((kMaxSideCurrent < ADC_value(SIDEA_I_PIN)) ||
          (kMaxSideCurrent < ADC_value(SIDEB_I_PIN)));
}

/*
static inline float DecodeTemperatureData(TWIDevice* device) {
  static const float kCountsPerDegreeC = 16.0;
  
  // for TMP112 (see p.8 of datasheet)
  uint16_t placeholder = device->data[1];
  placeholder = (placeholder << 4) | (device->data[0] >> 4);
  
  // if the MSB is high (negative number), take the two's complement
  if (placeholder & (1 << 12)) placeholder = ~placeholder + 1;
  
  return (placeholder / kCountsPerDegreeC);
}
*/

static void InitPins(void) {
  // initialize any digital I/O pin(s)
	SIDEA_CONNECT_EN(1); SIDEB_CONNECT_EN(1);
}

static void InitI2CDevices(void) {
	InitFan();
}

// Description: Updates the side fan target speed.  Note that software sends up
//   to 240, so NO mapping is necessary.  Ensure the bus is idle before
//   attempting to send this speed!
static void set_fan_speed(const uint8_t speed) {
  // if the speed is different, update it
  static uint16_t last_speed = 0;
  if (speed != last_speed) {
    // 240/240 => 100%, see p.12 of datasheet
    uint8_t data[] = {speed};
    fan.subaddress = MAX6615AEE_PWM1_TARGET_DC;
	  TWI_WriteData(kTWI02, &fan, data);
    last_speed = speed;
  }
}

// Description: Initializes fan controller for manual speed control among 
//   other things.
// BUG ALERT: Some data is written that influences fan1 AND fan2
static void InitFan(void) {
  // reset the IC
  // disable spin-up; remote2 for temp ch2 source, meaning use our own thermistor; 
  // 0 = min duty cycle; fan2 PWM invert; fan1 PWM invert;
  // enable a timeout; 1 = reset; run (not standby);
	uint8_t data[] = {0b01011000};
	fan.subaddress = MAX6615AEE_CONFIG;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); ClrWdt();
	
	// set fan configuration for manual speed control mode
	data[0] = 0x00;
	fan.subaddress = MAX6615AEE_FAN_CONFIG;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); ClrWdt();
	
	// set the duty cycle rate-of-change to be instant for fan1 and fan2
	fan.subaddress = MAX6615AEE_DC_RATE;
	data[0] = 0x00;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); ClrWdt();
	
  // set the target duty cycle (initial fan speed) to 0%
  data[0] = 0;
  fan.subaddress = MAX6615AEE_PWM1_TARGET_DC;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); ClrWdt();
}


// Description: Eases the turn on of the power bus to avoid a current spike
//   that could cause the battery to go into an overcurrent condition.  Large
//   capacitors on this board make this an exception (I = C*dV/dt).  A separate,
//   empirically-found sequence is used for each battery as no known 
//   documentation is available.
// Responsible Engineer: Taylor Penn (taylor@robotex.com)
static void TurnOnPowerBus(const kBattery battery) {
  switch (battery) {
    case kBatteryBB2590: RunOldPowerup(); break;
    case kBatteryBT70791B: RunNewPowerup(); break;
    case kBatteryRoboteX: RunNewPowerup(); break;
    default: RunHybridPowerup(); break;
  }
}


static void RunOldPowerup(void) {
  uint8_t i;
	for (i = 0; i < 20; i++) {
		SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
		Delay(10); ClrWdt();
		SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
		Delay(40); ClrWdt();
	}
	SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
}


static void RunNewPowerup(void) {
  uint16_t i,j = 0;
  uint16_t k = 2000;
  uint16_t k0 = 2000;

  for (i = 0; i < 300; i++) {
		SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
    for (j = 0; j < k; j++) Nop();
		SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
    Delay(10);
    k = k0 + i * i / 4;
    ClrWdt();
  }
  SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
}


// Description: Turns on the power bus using the 'hybrid method.'  Note that
//   this method takes too long for the UpdateBox to work correctly.
static void RunHybridPowerup(void) {
  uint16_t i, j, k, k0, l;
  k = 2000;
  k0 = 2000;
  l = 0;
	for (i = 0; i < 200; i++) {
    for (l = 0; l < 3; l++) {
    	SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
      for (j = 0; j < k; j++) Nop();
    	SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
      break;
      if (20 < i) break;
      ClrWdt(); Delay(10);
    }
    ClrWdt(); Delay(40); ClrWdt();
    k = k0 + i * i / 4;
    if (20000 < k) k = 20000;
	}
	SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
}


// Description: Gets flipper angle offset from software and writes it to
// persistent memory for the future.
static void CalibrateFlipper(void) {
  // stop all the motors to prevent the robot from driving away
  DT_set_speed(kMotorLeft, 0.0);
  DT_set_speed(kMotorRight, 0.0);
  DT_set_speed(kMotorFlipper, 0.0);
  
  // map the current potentiometer values to an angle
  // NB: the operator at this point should've visually leveled the flippers
  uint16_t offset = CombinePotAngle(ADC_value(M3_POS_FB_1_PIN), 
                                    ADC_value(M3_POS_FB_2_PIN));
  
  // store the new offset to the MCU's internal flash
  DataEEInit(); Nop();
  DataEEWrite((offset & 0xff), 0); Nop();       // low byte
  DataEEWrite((offset >> 8), 1); Nop();         // high byte
  DataEEWrite(FLIPPER_CALIBRATED, 2); Nop();    // whether calibrated
  
  // wait here until the carrier board cuts our power, resetting us
  while (1) ClrWdt();
}


// Description: Returns the flipper angle offset previously stored in flash.
//   Returns 0 if no value has yet been stored.
static uint16_t flipper_angle_offset(void) {
  static uint16_t offset = 0;
  static bool initialized = NO;
  
  if (!initialized) {
    uint8_t angle_data[3];
    uint16_t i;
    DataEEInit(); Nop();
    for (i = 0; i < 3; i++) angle_data[i] = DataEERead(i); Nop();
  
    // if the flipper has already been calibrated
    if (angle_data[2] == FLIPPER_CALIBRATED) {
      offset = ((angle_data[1] << 8) + angle_data[0]);
    } else {
      offset = 0;
    }
    initialized = YES;
  }
  
  return offset;
}

// Description: Returns the flipper angle from the two potentiometers
//   synced with the flipper shaft.
// Responsible Engineer: Taylor Penn (taylor@robotex.com)
static uint16_t flipper_angle(void) {
  uint16_t combined_pot_angle = CombinePotAngle((ADC_value(M3_POS_FB_1_PIN)),
                                                (ADC_value(M3_POS_FB_2_PIN)));
  
  // pass along the error code if one was received
  if (combined_pot_angle == 10000) return 10000;
  else if (combined_pot_angle == 0xffff) return 0xffff;
  
  // if calibration failed, return angle with no offset
  // TODO: how will flipper_angle_offset() ever be 0xffff???
  if (flipper_angle_offset() == 0xffff) return combined_pot_angle;
  
  int16_t calibrated_pot_angle = combined_pot_angle - flipper_angle_offset();
  
  // rollover if required
  if (calibrated_pot_angle < 0) calibrated_pot_angle += 360;
  else if (360 <= calibrated_pot_angle) calibrated_pot_angle -= 360;
  
  return calibrated_pot_angle;
}


// Description: Returns the combined potentiometer angle to overcome limitation
//   of the dead zone of a single potentiometer.
// Responsible Engineer: Taylor Penn (taylor@robotex.com)
static uint16_t CombinePotAngle(uint16_t pot1_value,
                                uint16_t pot2_value) {
  int16_t combined_pot_angle = 0;
  
  // correct for pot2 turning the opposite direction
  pot2_value = 1023 - pot2_value;

  // if both pot values are invalid
  if (((pot1_value < LOW_POT_THRESHOLD) || (HIGH_POT_THRESHOLD < pot1_value)) && 
      ((pot2_value < LOW_POT_THRESHOLD) || (HIGH_POT_THRESHOLD < pot2_value))) {
    return 0xffff;  // return an invalid calibration sentinel
  } else if ((pot1_value < LOW_POT_THRESHOLD) || (HIGH_POT_THRESHOLD < pot1_value) ) {
    // if pot1 is beyond the linear range
    combined_pot_angle = pot2_value * 0.326 + 13.35; // 333.3deg/1023au => 0.326
  } else if ((pot2_value < LOW_POT_THRESHOLD) || (HIGH_POT_THRESHOLD < pot2_value)) {
    // if pot2 is beyond the linear range
    //333.3 degrees, 1023 total counts, 333.3/1023 = .326
    // 13.35 degrees + 45 degrees = 58.35 degrees
    combined_pot_angle = (int16_t)pot1_value * 0.326 + 13.35 + FLIPPER_POT_OFFSET;
  } else {
    // otherwise both pot1 and pot2 values are valid
    
    // determine which one is closest to the end of range
    int16_t temp1, temp2, temp_pot1_value = 0;
    temp1 = pot1_value - 512;
    temp2 = pot2_value - 512;

    // offset so that both pot values should be the same
    // FLIPPER_POT_OFFSET/333.33*1023 = 168.8 for 55 degrees
    temp_pot1_value = pot1_value - 168.8;

    int16_t pot_angle_1, pot_angle_2 = 0;
    pot_angle_1 = temp_pot1_value * 0.326 + 13.35;
    pot_angle_2 = pot2_value * 0.326 + 13.35;

    float scale_factor = 0;
    // if pot1 is closer to the end of range
    if (abs(temp2) < abs(temp1)) {
      scale_factor = (512 - abs(temp1)) / 512.0;
      combined_pot_angle = pot_angle_1 * scale_factor + 
                           pot_angle_2 * (1 - scale_factor);
    } else {
      // otherwise pot2 is closer to the end of range
      scale_factor = (512 - abs(temp2)) / 512.0;
      combined_pot_angle = pot_angle_2 * scale_factor + 
                           pot_angle_1 * (1 - scale_factor);
    }
  }
  
  // rollover if required
  if (360 < combined_pot_angle) combined_pot_angle -= 360;
  else if (combined_pot_angle < 0) combined_pot_angle  += 360;
  
  return (uint16_t)combined_pot_angle;
}


// Notes:
//   - must be called AFTER SMBus initialization and BEFORE we begin
//     polling any other devices on the bus
//   - this is blocking!
static kBattery DetermineBatteryType(void) {
  const uint8_t kMaxNumAttempts = 10; // number of attempts to read the 
                                      // device name before giving up
  uint8_t dataSideA1[8] = {0};
  uint8_t dataSideB1[8] = {0};
  TWIDevice tempSideA = {
    .address = SBS_SLAVE_ADDRESS,
    .subaddress = SBS_DEVICE_NAME,
    .n_data_bytes = 8,
    .data = dataSideA1
  };
  TWIDevice tempSideB = {
    .address = SBS_SLAVE_ADDRESS,
    .subaddress = SBS_DEVICE_NAME,
    .n_data_bytes = 8,
    .data = dataSideB1
  };

  uint16_t i = 0;
  do {
    // try to get the name from either side
    if (TWI_ErrorHasOccurred(kTWI02)) TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
    if (TWI_ErrorHasOccurred(kTWI03)) TWI_Init(kTWI03, kTWIBaudRate100kHz, SMBUS);
    if (TWI_IsBusIdle(kTWI02)) TWI_RequestData(kTWI02, &tempSideB);
    if (TWI_IsBusIdle(kTWI03)) TWI_RequestData(kTWI03, &tempSideA);
    Delay(5);
    if (TWI_IsNewDataAvailable(kTWI02)) TWI_GetData(kTWI02, &tempSideB);
    if (TWI_IsNewDataAvailable(kTWI03)) TWI_GetData(kTWI03, &tempSideA);
    
    Nop();
    Nop();
    // check whether we've received a known name
    if (!strncmp((char*)old_bat_str, (char*)tempSideA.data, 7) ||
        !strncmp((char*)old_bat_str, (char*)tempSideB.data, 7)) {
      return kBatteryBB2590;
    }
    if (!strncmp((char*)rx_bat_str, (char*)tempSideA.data, 7) ||
        !strncmp((char*)rx_bat_str, (char*)tempSideA.data, 7)) {
      return kBatteryRoboteX;
    }
  } while (i++ < kMaxNumAttempts);
  
  // check for other length string responses as well
  uint8_t dataSideA2[10] = {0};
  uint8_t dataSideB2[10] = {0};
  tempSideA.n_data_bytes = 10;
  tempSideA.data = dataSideA2;
  tempSideB.n_data_bytes = 10;
  tempSideB.data = dataSideB2;
  i = 0;
  do {
    if (TWI_IsBusIdle(kTWI02)) TWI_RequestData(kTWI02, &tempSideB);
    if (TWI_IsBusIdle(kTWI03)) TWI_RequestData(kTWI03, &tempSideA);
    Delay(5);
    if (TWI_IsNewDataAvailable(kTWI02)) TWI_GetData(kTWI02, &tempSideB);
    if (TWI_IsNewDataAvailable(kTWI03)) TWI_GetData(kTWI03, &tempSideA);
    
    Nop();
    Nop();
    if (!strncmp((char*)new_bat_str, (char*)tempSideA.data, 9) ||
        !strncmp((char*)new_bat_str, (char*)tempSideB.data, 9)) {
      return kBatteryBT70791B;
    }
  } while (i++ < kMaxNumAttempts);
  
  return kBatteryUnknown;
}


// Description: When we are on the charger, only leave one side of the power
//   bus on at a time.  This is so that one side of a battery doesn't charge 
//   the other side through the power bus.
static void TogglePowerbus(void) {
  static uint8_t pb_state = 0;
  switch (pb_state) {
    case 0:
      pb_state = 1;
      SIDEB_CONNECT = 0; SIDEA_CONNECT = 1;
      break;
    case 1:
      pb_state = 0;
      SIDEA_CONNECT = 0; SIDEB_CONNECT = 1;
      break;
    default: pb_state = 0; break;
  }
}
