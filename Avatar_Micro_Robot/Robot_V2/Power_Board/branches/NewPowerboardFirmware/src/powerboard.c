/*==============================================================================
File: Powerboard.c

Description: This is the overarching file that encapsulates the 
  application-level firmware for the powerboard (aka motor driver board) within
  the robot.

Notes:
  - TODO: move frequency of PWM for motors to ~30kHz
  - USB to each payload and to processor
  - see I2C diagram
  - the carrier board provides the regulated logic-level voltages to the
    power board
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#define TEST_POWERBOARD
/*---------------------------Dependencies-------------------------------------*/
#include "./core/StandardHeader.h"
#include "./core/Timers.h"            // for timers
#include "./core/PWM.h"               // for PWM generators
#include "./core/ADC.h"               // for A/D conversions
#include "./core/TWI.h"               // for I2C/SMBus library
#include "./SMBusDevices/SBS.h"       // for smart battery system macros
#include "./SMBusDevices/MAX6615AEE.h"// for fan controller macros
#include "./SMBusDevices/RXCharger.h" // for internal charger macros
#include "./core/InputCapture.h"      // for motor shaft speed feedback
#include "./USB/RXUSBDevice.h"        // for USB communication
#include "./PID.h"                    // for motor speed control
#include "./Flash/DEE Emulation 16-bit.h"   // for MCU flash reads/writes
#include "./Filters.h"                // for IIRFilter()
#include <string.h>                   // for strncmp()
#include <stdlib.h>                   // for abs()

/*---------------------------Macros-------------------------------------------*/
#define POWERBOARD_PRODUCT_ID 0x03  // for USB

// power management pin(s) and threshold(s)
#define SIDEA_CONNECT_EN(a)	(_TRISD3 = !(a))
#define SIDEB_CONNECT_EN(a)	(_TRISD2 = !(a))
#define SIDEA_CONNECT 	    _LATD3
#define SIDEB_CONNECT 	    _LATD2

// PID controller values
#define LEFT_CONTROLLER     0
#define RIGHT_CONTROLLER    1

#define MAX_DC              1.00    // maximum control effort magnitude (can also be -1000)
#define MIN_DC              0.00
#define K_P                 0.002   // proportional gain
#define K_I                 0.0002  // integral gain
#define K_D                 0.0     // differential gain

// GR = GR_motor_gearhead * GR_spurgear_reduction
//    = (1/17) * (1/4) => 0.01470588235 for high-speed version
//    = (1/24) * (1/4) => 0.01041666666 for high-torque version

// #define D_TRACK_WHEEL ~150mm
//#define MPS_PER_EVENT       0.64// (meters/second)/event, for high-torque version
// (1 events/ms)*(150mm)*1/24*1/4) ~=> 1.5m/s, (150*1/24*1/4)^-1=>0.64

// filters
#define ALPHA               0.1
#define LMOTOR_FILTER       0
#define RMOTOR_FILTER       1
#define FLIPPER_FILTER      2
#define ALPHA_I             0.8
#define SIDEA_I_FILTER      3
#define SIDEB_I_FILTER      4

// OCU speed filter-related values
#define MAX_DESIRED_SPEED   400     // [au], limits signal coming in from OCU

//---analog sensing pins (ANx)
#define M1_T_PIN            2       // motor1 temperature sensing
#define M1_I_PIN            3       // motor1 current sensing
#define M2_T_PIN            0       // motor2 temperature sensing
#define M2_I_PIN            1       // motor2 current sensing
#define M3_T_PIN            14      // motor3 temperature sensing
#define M3_I_PIN            15      // motor3 current sensing
#define SIDEA_I_PIN         12      // battery sideA current sensing
#define SIDEB_I_PIN         13      // battery sideB current sensing
#define M3_POS_FB_1_PIN     8       // motor3 position feeback1
#define M3_POS_FB_2_PIN     9       // motor3 position feedback2

// flipper calibration values
#define FLIPPER_CALIBRATION_SENTINEL  12345   // special message from software
#define FLIPPER_CALIBRATED  0xAA    // written to flash
// NB: These pot thresholds are very wide because the
// flipper pots are on a different 3.3V supply than the PIC
#define LOW_POT_THRESHOLD   33      // invalid below this threshold
#define HIGH_POT_THRESHOLD  990     // invalid above this threshold
#define FLIPPER_POT_OFFSET  -55
  
// motors
#define LEFT_MOTOR          0
#define RIGHT_MOTOR         1
#define FLIPPER             2

// motor PWM pin assignments
#define T_PWM               1   // [ms], 1/1ms => 1kHz
#define M1_PWM_PIN    	    24  //_RP24R
#define M2_PWM_PIN 	        2   //_RP2R
#define M3_PWM_PIN          25  //_RP25R

#define SLOW_DECAY_MODE     1   // lower ripple current, but
                                // slower dynamic response
                                // (see p.10 of A3931 datasheet)

//---motor driver inputs
#define M1_DIR_EN(a)		    (_TRISD6 = !(a))
#define M1_BRAKE_EN(a)		  (_TRISD7 = !(a))
#define M1_MODE_EN(a)	  	  (_TRISD9 = !(a))
#define M1_COAST_EN(a)		  (_TRISD10 = !(a))
#define M1_DIR 		          _LATD6
#define M1_TURN_BRAKE(a) 	  (_LATD7 = !(a))   // active-low
#define M1_SET_MODE_TO(a) 	(_LATD9 = (a))
#define M1_TURN_COAST(a) 	  (_LATD10 = !(a))  // active-low
#define M1_TACHO_RPN        12                // RP12         

#define M2_DIR_EN(a)		    (_TRISB4 = !(a))
#define M2_BRAKE_EN(a)		  (_TRISB5 = !(a))
#define M2_MODE_EN(a)		    (_TRISG9 = !(a))
#define M2_COAST_EN(a)		  (_TRISG8 = !(a))
#define M2_DIR 		          _LATB4
#define M2_TURN_BRAKE(a)    (_LATB5 = !(a))
#define M2_SET_MODE_TO(a)   (_LATG9 = (a))
#define M2_TURN_COAST(a)    (_LATG8 = !(a))
#define M2_TACHO_RPN        16         

#define M3_DIR_EN(a)		    (_TRISE3 = !(a))
#define M3_BRAKE_EN(a)		  (_TRISF0 = !(a))
#define M3_MODE_EN(a)		    (_TRISE4 = !(a))
#define M3_COAST_EN(a)		  (_TRISF1 = !(a))
#define M3_DIR 		          _LATE3
#define M3_TURN_BRAKE(a) 	  (_LATF0 = (!a))
#define M3_SET_MODE_TO(a)   (_LATE4 = (a))
#define M3_TURN_COAST(a) 	  (_LATF1 = (!a))
#define M3_TACHO_RPN        20

//---motor driver outputs
#define M1_DIRO		          _RD0
#define M1_FF1 		          _RC14
#define M1_FF2  	          _RC13

#define M2_DIRO		          _RE5  // whether the motor is moving clockwise or counter-clockwise
#define M2_FF1 		          _RG6
#define M2_FF2  	          _RG7

#define M3_DIRO             _RE0
#define M3_FF1              _RE2
#define M3_FF2              _RE1

//---I2C 7-bit slave address(es)
#define TMP112_ADDRESS      0x49  // also test 0x48
#define FAN_ADDRESS         0x18  // pin-programmable

//---timer(s)
#define _10ms               10
#define _100ms              100
//#define I2C_TIMEOUT_TIMER   0
//#define I2C_TIMEOUT_TIME    100   // [ms]
#define I2C_UPDATE_TIMER    1
#define I2C_UPDATE_TIME     (10*_100ms)
#define CONTROL_TIMER       2 
#define CONTROL_TIME        (_100ms)
#define SW_UPDATE_TIMER     3           // to update software
#define SW_UPDATE_TIME      (10*_100ms)  //(2*_10ms)   // => 50Hz
#define CURRENT_CHECK_TIMER 4
#define CURRENT_CHECK_TIME  (5)
#define SOCA_TIMER          5
#define SOCA_TIME           (55*_100ms) // NB: wait at least 5s before 
#define SOCB_TIMER          6           // requesting updated values
#define SOCB_TIME           (70*_100ms)
#define COOLDOWN_TIMER      7           // for after nearly overcurrenting
#define COOLDOWN_TIME       (_100ms)    // the battery
#define UPDATE_T_TIMER      8           // for updating the temperature
#define UPDATE_T_TIME       (_100ms)
#define CHARGER_CHECK_TIMER 9           // for checking the status of a
#define CHARGER_CHECK_TIME  (5*_10ms)   // potential internal charger
#define PBTOGGLE_TIMER      10
#define PBTOGGLE_TIME       (30*_100ms)

/*---------------------------Constants----------------------------------------*/
// [au], maximum anticipated noise on an A/D line
// (0.100V / 3.3V) * 1023=
static const kADCReading kMaxADCNoise = 31;

// [au], maximum allowable current draw from an individual side
// 0.01S*(17A*0.001Ohm)*11k ~= 1.87V, (1.87 / 3.30) * 1023=
static const kADCReading kMaxSideCurrent = 580;
// 800au => 23A?

// potential smart battery device names (reversed) used during initialization
static const int8_t old_bat_str[8] = {'0','9','5','2','-','B','B', 0x07};
static const int8_t new_bat_str[10] = {'B','1','9','7','0','7','-','T','B', 0x0A};
static const int8_t rx_bat_str[8] = {'X','E','T','O','B','O','R', 0x07};

// I = (au / 1023) * 3.30 / 0.01 / 11e3 / 0.001
/*---------------------------Type Definitions---------------------------------*/
// possible states
// TODO: encapsulate current direction in function?
typedef enum {
  kWaiting = 0,
  kRunning,
  kCoolingDown,
} kPowerboardState;

typedef enum {
  kBatteryBB2590 = 0,
  kBatteryBT70791B,
  kBatteryRoboteX,
  kBatteryUnknown,
} kBattery;
  
/*---------------------------Helper Function Prototypes-----------------------*/
void InitPowerboard(void);
void ProcessPowerboardIO(void);
static void UpdateMotorSpeeds(void);
static bool OvercurrentApproached(void);
static void InitPins(void);
static void WriteBuildTime(void);
static void InitI2CDevices(void);
static void InitFan(void);
static void InitMotors(void);
static bool OvercurrentApproached(void);
static void UpdateSensors(void);
static void UpdateBus2Sensors(void);
static void UpdateBus3Sensors(void);
static void UpdateSoftware(void);
//static inline float DecodeTemperatureData(TWIDevice* device);

//---startup-related
static void TurnOnPowerBus(kBattery battery);
static kBattery DetermineBatteryType(void);
static void RunOldPowerup(void);
static void RunNewPowerup(void);
static void RunHybridPowerup(void);
static void CalibrateFlipper(void);
static uint16_t GetStoredFlipperAngleOffset(void);
static uint16_t GetFlipperAngle(uint16_t pot1_value, uint16_t pot2_value);
static uint16_t CombinePotAngle(uint16_t pot1_value, uint16_t pot2_value);

static void TogglePowerbus(void);

//---control-related
static float GetNominalDriveEffort(const float desired_speed);
static int16_t GetDesiredSpeed(const uint8_t motor_index);

/*---------------------------Module Variables---------------------------------*/
//static uint8_t dummyTData[2] = {0};
static uint8_t dummySOCDataA[2] = {0};
static uint8_t dummySOCDataB[2] = {0};
static uint8_t dummyChargerStatusData[2] = {0};
/*
static TWIDevice thermometer = {
  .address = TMP112_ADDRESS,
  .subaddress = TMP112_TEMP, // subaddress which contains the sensor data
  .n_data_bytes = 2,         // number of data bytes the sensor will send us
  .data = dummyTData
};
*/

static TWIDevice fan = {
  .address = FAN_ADDRESS
};


extern TWIDevice sideA = {
  .address = SBS_SLAVE_ADDRESS,
  .subaddress = SBS_RELATIVE_SOC,
  .n_data_bytes = 2,
  .data = dummySOCDataA
};

// NB: this is on a different bus since has same slave address
extern TWIDevice sideB = {
  .address = SBS_SLAVE_ADDRESS,
  .subaddress = SBS_RELATIVE_SOC,
  .n_data_bytes = 2,
  .data = dummySOCDataB
};

extern TWIDevice charger = {
  .address = RXCHARGER_ADDRESS,
  .subaddress = RXCHARGER_STATUS,
  .n_data_bytes = 2,
  .data = dummyChargerStatusData
};
  
static volatile kPowerboardState state = kWaiting;
static volatile uint16_t flipper_angle_offset = 0;

// TODO: re-limit this scope after debugging!
extern float actual_speed_left = 0;
extern float actual_speed_right = 0;
extern float effort_left = 0;
extern float effort_right = 0;
extern float nominal_effort = 0;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_POWERBOARD
#include "./core/ConfigurationBits.h"
int main(void) {
  //_SWDTEN = 1; // enable the watchdog timer
  RXUSBDevice_Init(POWERBOARD_PRODUCT_ID);
  InitPowerboard();
  
  while (1) {
    //ClrWdt();
    ProcessPowerboardIO();
    RXUSBDevice_ProcessMessage();
  }
  
  return 0;
}
#endif

void InitPowerboard(void) {
  //WriteBuildTime();
  //flipper_angle_offset = GetStoredFlipperAngleOffset();
  InitPins();
	
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
  IC_Init(kIC01, M1_TACHO_RPN, 10);
  IC_Init(kIC02, M2_TACHO_RPN, 10);
  
  PWM_Init(kPWM01, M1_PWM_PIN, T_PWM);
  PWM_Init(kPWM02, M2_PWM_PIN, T_PWM);
  //PWM_Init(kPWM03, M3_PWM_PIN, T_PWM);
  
  TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
  TWI_Init(kTWI03, kTWIBaudRate100kHz, SMBUS);
  //InitI2CDevices();
  
  TurnOnPowerBus(DetermineBatteryType());
  
  InitMotors();
 	//PID_Init(LEFT_CONTROLLER, MAX_DC, MIN_DC, K_P, K_I, K_D);
 	//PID_Init(RIGHT_CONTROLLER, MAX_DC, MIN_DC, K_P, K_I, K_D);
 	
 	// prime any timers that require it
 	// NB: all timers begin expired, unless otherwise specified
 	TMRS_StartTimer(SW_UPDATE_TIMER, SW_UPDATE_TIME);
  TMRS_StartTimer(CONTROL_TIMER, CONTROL_TIME);
  TMRS_StartTimer(CURRENT_CHECK_TIMER, CURRENT_CHECK_TIME);
  //TMRS_StartTimer(UPDATE_T_TIMER, UPDATE_T_TIME);
  //TMRS_StartTimer(I2C_TIMEOUT_TIMER, I2C_TIMEOUT_TIME);  // begin the sensor updating process
  //TMRS_StartTimer(I2C_UPDATE_TIMER, I2C_UPDATE_TIME);
  //TMRS_StartTimer(SOCA_TIMER, 1);
  //TMRS_StartTimer(SOCB_TIMER, 1);
}


void ProcessPowerboardIO(void) {
  /*
  // ensure we do NOT exceed the current limit of the battery
  if (OvercurrentApproached()) {
    // turn off the power bus
    SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
    
    // stop driving all motors
    PWM_UpdateDutyCycle(kPWM01, 0);
    PWM_UpdateDutyCycle(kPWM02, 0);
    PWM_UpdateDutyCycle(kPWM03, 0);
    
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
  */
  
  IC_UpdatePeriods();
  UpdateSensors();
  
  switch (state) {
    case kWaiting:
      // TODO: move flipper calibration to initialization
      WriteBuildTime();
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
        TurnOnPowerBus(kBatteryUnknown);
        state = kRunning;
        return;
      }
      break;
  }
}


/*---------------------------Helper Function Definitions----------------------*/
static void UpdateMotorSpeeds(void) {
  float desired_left_speed = IIRFilter(LMOTOR_FILTER, REG_MOTOR_VELOCITY.left, ALPHA) / 1000.0;
  if (desired_left_speed < 0) desired_left_speed = 0;
  if (0.5 < desired_left_speed) desired_left_speed = 0.5;
  PWM_UpdateDutyCycle(kPWM01, desired_left_speed);
  actual_speed_left = IC_period(kIC01);
  
  float desired_right_speed = IIRFilter(RMOTOR_FILTER, REG_MOTOR_VELOCITY.right, ALPHA) / 1000.0;
  if (desired_right_speed < 0) desired_right_speed = 0;
  if (0.5 < desired_right_speed) desired_right_speed = 0.5;
  PWM_UpdateDutyCycle(kPWM02, desired_right_speed);
  actual_speed_right = IC_period(kIC02);
  
  //float desired_flipper_speed = IIRFilter(FLIPPER_FILTER, REG_MOTOR_VELOCITY.flipper, ALPHA) / 1200.0;
  //if (desired_flipper_speed < 0) desired_flipper_speed = 0;
  //PWM_UpdateDutyCycle(kPWM03, desired_flipper_speed);
  
  /*
  uint16_t desired_speed = 100;
  nominal_effort = GetNominalDriveEffort(desired_speed);
  //if (IC_period(kIC01) != 0) actual_speed_left = 100000.0 / IC_period(kIC01);
  //else actual_speed_left = 0;
  //effort_left = PID_ComputeOutput(LEFT_CONTROLLER, desired_speed, actual_speed_left, nominal_effort);
  //PWM_UpdateDutyCycle(M1_PWM_PIN, effort_left);
  PWM_UpdateDutyCycle(kPWM01, nominal_effort);
  
  //if (IC_period(kIC02) != 0) actual_speed_right = 100000.0 / IC_period(kIC02);
  //else actual_speed_right = 0;
  //effort_right = PID_ComputeOutput(RIGHT_CONTROLLER, desired_speed, actual_speed_right, nominal_effort);
  //PWM_UpdateDutyCycle(M2_PWM_PIN, effort_right);
  PWM_UpdateDutyCycle(kPWM02, nominal_effort);
  */
}


// Description: Caps incoming OCU commands to test control
static int16_t GetDesiredSpeed(const uint8_t motor_index) {
  switch (motor_index) {
    case LEFT_MOTOR:
      if (MAX_DESIRED_SPEED < REG_MOTOR_VELOCITY.left) {
        return MAX_DESIRED_SPEED;
      } else if (REG_MOTOR_VELOCITY.left < -MAX_DESIRED_SPEED) { 
        return -MAX_DESIRED_SPEED;
      }
      return REG_MOTOR_VELOCITY.left;
    case RIGHT_MOTOR:
      if (MAX_DESIRED_SPEED < REG_MOTOR_VELOCITY.right) {
        return MAX_DESIRED_SPEED;
      } else if (REG_MOTOR_VELOCITY.right < -MAX_DESIRED_SPEED) {
        return -MAX_DESIRED_SPEED;
      }
      return REG_MOTOR_VELOCITY.right;
    case FLIPPER:
      return REG_MOTOR_VELOCITY.flipper;
    default:
      return 0;
  }
}


// Description: Returns the approximate steady-state effort required to 
//   maintain the given desired speed of a drive motor.
static float GetNominalDriveEffort(const float desired_speed) {
  // NB: transfer function found empirically (see spreadsheet for data)  
  //if (desired_speed < 0) return ((1.446 * desired_speed) - 25.256);
  //else return ((1.446 * desired_speed) + 25.256);
  if (desired_speed == 0) return 0;
  
  if (desired_speed < 0) return ((0.0007 * desired_speed) - 0.0067);
  else return ((0.0007 * desired_speed) + 0.0067);
}


static void UpdateSensors(void) {
  UpdateBus2Sensors();
  UpdateBus3Sensors();
}


static void UpdateBus2Sensors(void) {
  // if an error has occurred, restart the I2C module
  if (TWI_ErrorHasOccurred(kTWI02)) {
	  TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
	  Nop();
    return;
  }
  if (TWI_IsBusIdle(kTWI02) && TMRS_IsTimerExpired(SOCA_TIMER)) {
    TMRS_StartTimer(SOCA_TIMER, SOCA_TIME);
    if (TWI_IsNewDataAvailable(kTWI02)) {
      TWI_GetData(kTWI02, &sideA);
      Nop();
      Nop();
    }
    TWI_RequestData(kTWI02, &sideA);
    return;
	}
	/*
  if (TWI_IsBusIdle(kTWI02) && TMRS_IsTimerExpired(UPDATE_T_TIMER)) {
    TMRS_StartTimer(UPDATE_T_TIMER, UPDATE_T_TIME);
    
    return;
  }
  */
}


static void UpdateBus3Sensors(void) {
  if (TWI_ErrorHasOccurred(kTWI03)) {
	  TWI_Init(kTWI03, kTWIBaudRate100kHz, SMBUS);
	  Nop();
    return;
  }
  
  if (TWI_IsBusIdle(kTWI03) && TMRS_IsTimerExpired(SOCB_TIMER)) {
    TMRS_StartTimer(SOCB_TIMER, SOCB_TIME);
    if (TWI_IsNewDataAvailable(kTWI03)) {
      TWI_GetData(kTWI03, &sideB);
      Nop();
      Nop();
    }
    TWI_RequestData(kTWI03, &sideB);
    return;
  }
  
  if (TWI_IsBusIdle(kTWI03) && TMRS_IsTimerExpired(CHARGER_CHECK_TIMER)) {
    TMRS_StartTimer(CHARGER_CHECK_TIMER, CHARGER_CHECK_TIME);
    if (TWI_IsNewDataAvailable(kTWI03)) {
      TWI_GetData(kTWI03, &charger);
      Nop();
    }
    TWI_RequestData(kTWI03, &charger);
    return;
  }
}

// Description: Updates software with firmware feedback via shared USB
//   registers.
static void UpdateSoftware(void) {
  // update the state of charge of each battery side
  REG_ROBOT_REL_SOC_A = (sideA.data[1] << 8) + sideA.data[0];
  REG_ROBOT_REL_SOC_B = (sideB.data[1] << 8) + sideB.data[0];
  
  // update the current being pulled from the batteries
  REG_PWR_A_CURRENT = ADC_value(SIDEA_I_PIN);
  REG_PWR_B_CURRENT = ADC_value(SIDEB_I_PIN);
  
  // update the flipper position and angle
  uint8_t temp1 = (ADC_value(M3_POS_FB_1_PIN) >> 2);  // disregard 2 lsb's from 10-bit A/D
  uint8_t temp2 = (ADC_value(M3_POS_FB_2_PIN) >> 2);
  REG_FLIPPER_FB_POSITION.pot1 = temp1;
 	REG_FLIPPER_FB_POSITION.pot2 = temp2;
  REG_MOTOR_FLIPPER_ANGLE = GetFlipperAngle(temp1, temp2);
  
  // update the current on the motors
 	REG_MOTOR_FB_CURRENT.left = ADC_value(M1_I_PIN);
 	REG_MOTOR_FB_CURRENT.right = ADC_value(M2_I_PIN);
  REG_MOTOR_FB_CURRENT.flipper = ADC_value(M3_I_PIN);
  
  // update whether we're on the charging dock
  REG_MOTOR_CHARGER_STATE = (charger.data[1] << 8) + charger.data[0];
}


static bool OvercurrentApproached(void) {
  if (TMRS_IsTimerExpired(CURRENT_CHECK_TIMER)) {
    TMRS_StartTimer(CURRENT_CHECK_TIMER, CURRENT_CHECK_TIME);
    return ((kMaxSideCurrent < IIRFilter(SIDEA_I_FILTER, ADC_value(SIDEA_I_PIN), ALPHA_I)) || 
           (kMaxSideCurrent < IIRFilter(SIDEB_I_FILTER, ADC_value(SIDEB_I_PIN), ALPHA_I)));
  }
  return 0;
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
  // initialize each of the motors
  M1_DIR_EN(1); Nop();
	M1_BRAKE_EN(1); Nop();
	M1_MODE_EN(1); Nop();
	M1_COAST_EN(1); Nop();
	M1_TURN_BRAKE(OFF); Nop();
	M1_TURN_COAST(ON); Nop();
	M1_SET_MODE_TO(SLOW_DECAY_MODE); Nop();
	
	M2_DIR_EN(1); Nop();
	M2_BRAKE_EN(1); Nop();
	M2_MODE_EN(1); Nop();
	M2_COAST_EN(1); Nop();
  M2_TURN_BRAKE(OFF); Nop();
  M2_TURN_COAST(ON); Nop();
	M2_SET_MODE_TO(SLOW_DECAY_MODE); Nop();
	
	M3_DIR_EN(1); Nop();
	M3_BRAKE_EN(1); Nop();
	M3_MODE_EN(1); Nop();
	M3_COAST_EN(1); Nop();
	M3_TURN_BRAKE(OFF); Nop();
	M3_TURN_COAST(ON); Nop();
  M3_SET_MODE_TO(SLOW_DECAY_MODE); Nop();
  	
  // initialize any digital I/O pin(s)
	SIDEA_CONNECT_EN(1); SIDEB_CONNECT_EN(1);
}


static void InitMotors(void) {
 	// initialize motor drivers
  M1_DIR = CCW; Nop(); Nop();
  M1_TURN_COAST(OFF); Nop(); Nop();
	
  M2_DIR = CW; Nop();
  M2_TURN_COAST(OFF); Nop();
  
  M3_DIR = CCW; Nop();
  M3_TURN_COAST(OFF); Nop();
}


static void InitI2CDevices(void) {
	InitFan();
}


void InitFan(void) {
  // set fan configuration
	uint8_t data[1] = {0b00011010};
	fan.subaddress = MAX6615AEE_CONFIG;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); //ClrWdt();
	
	// set the duty cycle rate-of-change to be instant for fan2
	fan.subaddress = MAX6615AEE_DC_RATE;
	data[1] = 0b00011100;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); //ClrWdt();
	
  // make fan2 start as a function of thermistor 1 data
  data[0] = 0b00011000;
  fan.subaddress = MAX6615AEE_FAN_CONFIG;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); //ClrWdt();
	
	// set fan2 start duty cycle = 120 / 240 => 50%
	data[0] = 120;
	fan.subaddress = MAX6615AEE_PWM2_START_DC;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); //ClrWdt();
	
	// set fan2 to turn on at or above 15C
	data[0] = 15;
	fan.subaddress = MAX6615AEE_2_TEMP_START;
	TWI_WriteData(kTWI02, &fan, data); Delay(5); //ClrWdt();
}


// Description: Eases the turn on of the power bus to avoid a current spike
//   that could cause the battery to go into an overcurrent condition.  Large
//   capacitors on this board make this an exception (I = C*dV/dt).  A separate,
//   empirically-found sequence is used for each battery as no documentation
//   comes with these batteries.
// Responsible Engineer: Taylor Penn (taylor@robotex.com)
static void TurnOnPowerBus(kBattery battery) {
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
void CalibrateFlipper(void) {
  // disable all the motors to prevent the robot from driving away
  PWM_UpdateDutyCycle(kPWM01, 0);
  PWM_UpdateDutyCycle(kPWM02, 0);
  PWM_UpdateDutyCycle(kPWM03, 0);
  M1_TURN_COAST(ON);
  M2_TURN_COAST(ON);
  M3_TURN_COAST(ON);
  
  // map the current potentiometer values to an angle
  // NB: the operator at this point should've visually leveled the flippers
  flipper_angle_offset = CombinePotAngle(ADC_value(M3_POS_FB_1_PIN), ADC_value(M3_POS_FB_2_PIN));
  
  // store the new offset to the MCU's internal flash
  DataEEInit(); Nop();
  DataEEWrite((flipper_angle_offset >> 8), 0); Nop();   // low byte
  DataEEWrite((flipper_angle_offset & 0xff), 1); Nop(); // high byte
  DataEEWrite(FLIPPER_CALIBRATED, 2); Nop();            // whether calibrated
  
  // don't do anything again ever
  while (1) ClrWdt();
}


// Description: Returns the flipper angle offset in flash.  Returns
//   0 if no value has yet been stored.
static uint16_t GetStoredFlipperAngleOffset(void) {
  uint8_t angle_data[3];
  uint16_t i;
  DataEEInit(); Nop();
  for (i = 0; i < 3; i++) {
    angle_data[i] = DataEERead(i); Nop();
  }

  // if the flipper has already been calibrated
  if (angle_data[2] == FLIPPER_CALIBRATED) {
    return (angle_data[0] * 256 + angle_data[1]);
  }
  
  return 0;  
}


// Description: Writes the build time stored in flash during compilation 
//   to a USB register.  This only needs to be run once.
static void WriteBuildTime(void) {
	const uint8_t build_date[12] = __DATE__; 
	const uint8_t build_time[12] = __TIME__;
  uint8_t i = 0;
	for (i = 0; i < 12; i++) {
		if (!build_date[i]) REG_ROBOT_FIRMWARE_BUILD.data[i] = ' ';
		else REG_ROBOT_FIRMWARE_BUILD.data[i] = build_date[i];
		
		if (!build_time[i]) REG_ROBOT_FIRMWARE_BUILD.data[i + 12] = ' ';
		else REG_ROBOT_FIRMWARE_BUILD.data[i + 12] = build_time[i];
	}
}

// Description: Returns the flipper angle from the two potentiometers
//   synced with the flipper shaft.
// Responsible Engineer: Taylor Penn (taylor@robotex.com)
static uint16_t GetFlipperAngle(uint16_t pot1_value, uint16_t pot2_value) {
  uint16_t combined_pot_angle = CombinePotAngle(pot1_value, pot2_value);
  
  // pass along the error code if one was received
  if (combined_pot_angle == 10000) return 10000;
  else if (combined_pot_angle == 0xffff) return 0xffff;
  
  // if calibration failed, return angle with no offset
  // TODO: flipper_angle_offset should be 0 if calibration failed
  if (flipper_angle_offset == 0xffff) return combined_pot_angle;
  
  int16_t calibrated_pot_angle = combined_pot_angle - flipper_angle_offset;
  
  // rollover if required
  if (calibrated_pot_angle < 0) calibrated_pot_angle += 360;
  else if (360 < calibrated_pot_angle) calibrated_pot_angle -= 360;
  
  return calibrated_pot_angle;
}


// Description: Returns the combined potentiometer angle to overcome limitation
//   of the dead zone of a single potentiometer.
// Responsible Engineer: Taylor Penn (taylor@robotex.com)
static uint16_t CombinePotAngle(uint16_t pot1_value, uint16_t pot2_value) {
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
//   - this is blocking
static kBattery DetermineBatteryType(void) {
  const uint8_t kMaxNumAttempts = 3;  // number of attempts to read the 
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
    if (TWI_IsBusIdle(kTWI02)) TWI_RequestData(kTWI02, &tempSideB);
    if (TWI_IsBusIdle(kTWI03)) TWI_RequestData(kTWI03, &tempSideA);
    Delay(5);
    if (TWI_IsNewDataAvailable(kTWI02)) TWI_GetData(kTWI02, &tempSideB);
    if (TWI_IsNewDataAvailable(kTWI03)) TWI_GetData(kTWI03, &tempSideA);
    
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
