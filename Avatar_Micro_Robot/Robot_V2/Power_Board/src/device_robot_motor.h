/*****************************************************************************/
//*-----------------------------General Purpose------------------------------*/
#define CLEAR 0
#define SET 1
#define HI 1
#define LO 0
#define Set_ActiveLO 0
#define Clear_ActiveLO 1

/*****************************************************************************/

/*****************************************************************************/
//*-----------------------------------PWM------------------------------------*/
/////constant
//****define frequency value for PRy, based on 1:1 prescale
#define Period50Hz 533332
#define Period67Hz 319999 // 15ms
#define Period200Hz 79999
#define Period300Hz 53332
#define Period400Hz 39999
#define Period500Hz 31999
#define Period600Hz 26666
#define Period700Hz 22856
#define Period800Hz 19999
#define Period900Hz 17777
#define Period1000Hz 15999
#define Period1100Hz 14544
#define Period1200Hz 13332
#define Period1300Hz 12307
#define Period1400Hz 11428
#define Period1500Hz 10666
#define Period1600Hz 9999
#define Period1700Hz 9411
#define Period1800Hz 8888
#define Period1900Hz 8420
#define Period2000Hz 7999
#define Period2100Hz 7618
#define Period10000Hz 1599
#define Period20000Hz 799
#define Period30000Hz 532
#define Period50000Hz 319

/*****************************************************************************/
//*----------------------------------UART1------------------------------------*/
// based on 32MHz system clock rate
#define BaudRate_9600_LOW 103 // BRGH=0
#define BaudRate_57600_LOW 16 // BRGH=0
#define BaudRate_57600_HI 68  // BRGH=1
#define BaudRate_115200_HI 34 // BRGH=1

typedef enum MotorEvent {
    Stop = 0xFF01,
    Go = 0xFF02,
    Back = 0xFF03,
    NoEvent = 0xFF00,
} MotorEvent;

typedef enum MotorState {
    Forward = 0xEE00,
    Brake = 0xEE01,
    Protection = 0xEE02,
    Backward = 0xEE03,
} MotorState;

typedef enum MotorState2 {
    Locked = 0xEE04,
    Unlocked = 0xEE05,
} MotorState2;

// constant for timer.
// Since these are on timer1, they are in multiples of PR1
#define SpeedUpdateTimer 5          // 200Hz
#define CurrentSurgeRecoverTimer 10 // 10ms
#define USBTimeOutTimer 333         // 3Hz--333ms
#define Xbee_FanSpeedTimer 333      // 3Hz--333ms
#define SwitchDirectionTimer 10     // 10ms
#define StateMachineTimer 1         // 1KHz
#define RPMTimer 1                  // 1KHz
#define CurrentFBTimer 1            // 1KHz
#define M3_POSFB_Timer 1            // 1KHz
#define CurrentProtectionTimer 1    // 1KHz
#define I2C2Timer 100
#define I2C3Timer 100
#define SFREGUpdateTimer 4    // 250Hz
#define BATVolCheckingTimer 1 // 1KHz
#define BATRecoveryTimer 100  // 100ms
#define MotorOffTimer 35      // 35ms motor off if there is a surge

#define CurrentLimit                                                                               \
    2300 ///< Total motor current at which we will declare an overcurrent threshold and kill the
         ///< motors
#define MotorSpeedTargetCoefficient_Normal 40
#define MotorSpeedTargetCoefficient_Turn 4
#define MotorSpeedTargetCoefficient_Low 5
#define HardTurning 1000
#define StartTurning 0

// constant for special usage
typedef enum { Cell_OFF = 0, Cell_ON = 1 } BatteryState;
typedef enum {
    Cell_A = 0,
    Cell_B,
} BatteryChannel;
#define BATTERY_CHANNEL_COUNT 2
// constant for pins

// output pin mapping
#define M1_PWM _RP24R
#define M2_PWM _RP2R
#define M3_PWM _RP25R

// XbeeTest Only
#define U1RX_RPn 6
#define U1TX_RPn _RP7R
// XbeeTest End

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
/** Helper macro for iterating all motors and storing the result in variable i */
#define EACH_MOTOR_CHANNEL(i)                                                                      \
    i = 0;                                                                                         \
    i < MOTOR_CHANNEL_COUNT;                                                                       \
    i++

typedef enum {
    ControlMode_Conservative = 0,
    ControlMode_Normal,
    ControlMode_Aggressive,
    ControlMode_Customized
} ControlMode;

#define BackEMFSampleRangeStart 1800 // BackEMF sampling range starts about 90% of PWM period
#define BackEMFSampleRangeEnd 1840   // BackEMF sampling range ends about 92% of PWM period

//#define BATVoltageLimit 650 //11.06V, 3.3V-1024, 430K-100K voltage divider, 1024->17.49V
#define BATVoltageLimit 800 // 11.06V, 3.3V-1024, 430K-100K voltage divider, 1024->17.49V

#define SAMPLE_LENGTH                                                                              \
    4 ///< Number of samples to keep of running metrics for power management, like battery
      ///< temperature and voltage
#define SAMPLE_LENGTH_CONTROL 8 ///< Number of samples to keep of running metrics for speed control

// I2C Device Address
#define FAN_CONTROLLER_ADDRESS 0x18 ///< I2C address of fan controller
#define BATTERY_ADDRESS 0x0b ///< I2C address of batteries (one battery on I2C bus 2, one on 3)
#define EEPROM_ADDRESS 0x50  ///< I2C address of EEPROM
#define BATTERY_CHARGER_ADDRESS 0x0c ///< I2C address of Battery Charger

#define Fan1LowTemp 45 // 45C fan1 start temperature
#define Fan2LowTemp 45 // 45C fan2 start temperature

/////variable

extern int16_t Robot_Motor_TargetSpeedUSB[MOTOR_CHANNEL_COUNT];

extern int gNewData;

/////function
void PWM1Ini(void);
void PWM1Duty(int Duty);

void PWM2Ini(void);      // initialize PWM channel 2
void PWM2Duty(int Duty); // set duty cycle for PWM channel 2
// Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM3Ini(void);      // initialize PWM channel 3
void PWM3Duty(int Duty); // set duty cycle for PWM channel 3
// Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM4Ini(void);      // initialize PWM channel 4
void PWM4Duty(int Duty); // set duty cycle for PWM channel 4
// Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM5Ini(void);      // initialize PWM channel 5
void PWM5Duty(int Duty); // set duty cycle for PWM channel 5
// Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM6Ini(void);      // initialize PWM channel 6
void PWM6Duty(int Duty); // set duty cycle for PWM channel 6
// Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM7Ini(void);      // initialize PWM channel 7
void PWM7Duty(int Duty); // set duty cycle for PWM channel 7
// Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM8Ini(void);      // initialize PWM channel 8
void PWM8Duty(int Duty); // set duty cycle for PWM channel 8
// Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM9Ini(void);      // initialize PWM channel 9
void PWM9Duty(int Duty); // set duty cycle for PWM channel 9
// Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

/*****************************************************************************/

/*****************************************************************************/
//*-----------------------------------Timer----------------------------------*/
void IniIC1();
void IniIC3();
void IniTimer1();
void IniTimer2();
void IniTimer3();
void IniTimer4();
void IniTimer5();
/*****************************************************************************/

/*****************************************************************************/
//*-----------------------------------A/D------------------------------------*/
void IniAD();

/*****************************************************************************/
//*--------------------------------General Functions-------------------------*/

void MC_Ini(void);
void ProtectHB(MotorChannel Channel);
void PinRemap(void);
void Braking(MotorChannel Channel);
void UpdateSpeed(MotorChannel Channel, int State);
void DeviceRobotMotorInit();
void Device_MotorController_Process();
int GetDuty(long CurrentState, long Target, MotorChannel Channel, ControlMode Mode);
void UART1Ini();
void GetRPM(MotorChannel Channel);
void GetCurrent(MotorChannel Channel);
void ClearSpeedCtrlData(MotorChannel Channel);
void ClearCurrentCtrlData(MotorChannel Channel);
void Cell_Ctrl(BatteryChannel Channel, BatteryState state);
void USBInput();
void I2C1Ini();
void I2C2Ini();
void I2C3Ini();
void TMPSensorICIni();
void I2C3Update();
void FANCtrlIni();
void InterruptIni();
void Motor_IC1Interrupt();
void Motor_IC2Interrupt();
void Motor_IC3Interrupt();
void Motor_IC4Interrupt();
void Motor_IC5Interrupt();
void Motor_IC6Interrupt();

void Motor_T2Interrupt(void);
void Motor_T3Interrupt(void);
void Motor_T4Interrupt(void);
void Motor_T5Interrupt(void);

void Motor_ADC1Interrupt(void);

// Testing functions
void Motor_U1TXInterrupt(void);
void Motor_U1RXInterrupt(void);
void TestIO(void);
void TestIC1();
void TestPWM(void);
void TestIC2();
void TestOC();

extern int Cell_A_Current[SAMPLE_LENGTH];
extern int Cell_B_Current[SAMPLE_LENGTH];
extern int16_t Xbee_MOTOR_VELOCITY[MOTOR_CHANNEL_COUNT];
extern uint8_t Xbee_SIDE_FAN_SPEED;
extern uint8_t Xbee_SIDE_FAN_NEW;
