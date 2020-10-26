/*****************************************************************************/
//*-----------------------------General Purpose------------------------------*/
#define BIT00LO 0b1111111111111110;
#define BIT01LO 0b1111111111111101;
#define BIT02LO 0b1111111111111011;
#define BIT03LO 0b1111111111110111;
#define BIT04LO 0b1111111111101111;
#define BIT05LO 0b1111111111011111;
#define BIT06LO 0b1111111110111111;
#define BIT07LO 0b1111111101111111;
#define BIT08LO 0b1111111011111111;
#define BIT09LO 0b1111110111111111;
#define BIT10LO 0b1111101111111111;
#define BIT11LO 0b1111011111111111;
#define BIT12LO 0b1110111111111111;
#define BIT13LO 0b1101111111111111;
#define BIT14LO 0b1011111111111111;
#define BIT15LO 0b0111111111111111;
#define BIT00HI 0b0000000000000001;
#define BIT01HI 0b0000000000000010;
#define BIT02HI 0b0000000000000100;
#define BIT03HI 0b0000000000001000;
#define BIT04HI 0b0000000000010000;
#define BIT05HI 0b0000000000100000;
#define BIT06HI 0b0000000001000000;
#define BIT07HI 0b0000000010000000;
#define BIT08HI 0b0000000100000000;
#define BIT09HI 0b0000001000000000;
#define BIT10HI 0b0000010000000000;
#define BIT11HI 0b0000100000000000;
#define BIT12HI 0b0001000000000000;
#define BIT13HI 0b0010000000000000;
#define BIT14HI 0b0100000000000000;
#define BIT15HI 0b1000000000000000;
#define CLEAR 0
#define SET	1
#define HI 1
#define LO 0
#define True 1
#define False 0
#define Set_ActiveLO 0
#define Clear_ActiveLO 1


/*****************************************************************************/

/*****************************************************************************/
//*-----------------------------------PWM------------------------------------*/
/////constant
//****define frequency value for PRy, based on 1:1 prescale
#define Period50Hz 533332
#define Period67Hz 319999 //15ms
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
//based on 32MHz system clock rate
#define BaudRate_9600_LOW 103  //BRGH=0
#define BaudRate_57600_LOW 16  //BRGH=0
#define BaudRate_57600_HI 68 //BRGH=1
#define BaudRate_115200_HI 34  //BRGH=1




//clock
//system clock 20MHz
#define SystemClock 20000000
//constant for event
#define Stop 0xFF01
#define Go 0xFF02
#define Back 0xFF03
#define NoEvent	0xFF00

//constant for state 
#define Forward 0xEE00
#define Brake 0xEE01
#define Protection 0xEE02
#define Backwards 0xEE03
#define Locked	0xEE04
#define Unlocked 0xEE05

//constant for timer
#define SpeedUpdateTimer 5  	//200Hz
//#define SpeedUpdateTimer 2  	//500Hz
#define CurrentCtrlTimer 1 	//1K
#define CurrentSurgeRecoverTimer 10 //10ms
#define USBTimeOutTimer 333 	//3Hz--333ms
#define Xbee_FanSpeedTimer 333	//3Hz--333ms
//#define USBTimeOutTimer 3333 	//0.3Hz--3333ms
#define SwitchDirectionTimer 10 	//10ms
#define StateMachineTimer 1 	//1KHz
#define RPMTimer 1 			//1KHz
#define CurrentFBTimer 1 		//1KHz
#define M3_POSFB_Timer 1 		//1KHz
#define CurrentProtectionTimer 1  //1KHz
//#define I2C2Timer 25
#define I2C2Timer 25
#define I2C3Timer 25 //4Hz, this is the default sample rate of TMPSensorIC
#define SFREGUpdateTimer 4 	//250Hz
#define BATVolCheckingTimer 1 	//1KHz
#define BATRecoveryTimer 100 	//100ms
#define MotorOffTimer 35 		//35ms motor off if there is a surge

#define CurrentLimit 2300
#define MotorSpeedTargetCoefficient_Normal 40
#define MotorSpeedTargetCoefficient_Turn 4
#define MotorSpeedTargetCoefficient_Low 5
#define MotorCurrentTargetCoefficient_Normal 0.6
#define MotorCurrentTargetCoefficient_Turn 0.6
#define CurrentSurgeLimit 1 	//sampling rate 1KHz,Alow surge 1 times (1ms) before shut down 
#define CurrentThreshold 800	
#define HardTurning 1000
#define StartTurning 0
#define TurningDutyCycle 400.0
//constant for special usage
#define Cell_ON 1
#define Cell_OFF 0
#define Cell_A 0
#define Cell_B 1

//constant for pins
//input pin mapping

#define M1_TACHO_RPn 12
#define M2_TACHO_RPn 16
#define M3_TACHO_RPn 20	//is this used in the software?


/*
#define Servo2_Input_RPn 19 		
#define Servo1_Input_RPn 27
*/

//XbeeTest Only
#define U1RX_RPn 6
//XbeeTest End

//output pin mapping
#define M1_PWM 	_RP24R
#define M2_PWM 	_RP2R
#define M3_PWM 	_RP25R


//XbeeTest Only
#define U1TX_RPn  	_RP7R
//XbeeTest End



//Analog pins
#define M1_TEMP_EN(a)		_PCFG2 = !a
#define M1_CURR_EN(a)		_PCFG3 = !a
#define M2_TEMP_EN(a)		_PCFG0 = !a
#define M2_CURR_EN(a)		_PCFG1 = !a
#define M3_TEMP_EN(a)		_PCFG14 = !a
#define M3_CURR_EN(a)		_PCFG15 = !a

#define VCELL_A_EN(a)		_PCFG10 = !a
#define VCELL_B_EN(a)		_PCFG11 = !a
#define CELL_A_CURR_EN(a)	_PCFG12 = !a
#define CELL_B_CURR_EN(a)	_PCFG13 = !a

#define M3_POS_FB_1_EN(a)	_PCFG8 = !a
#define M3_POS_FB_2_EN(a)	_PCFG9 = !a


//Configure outputs

//Main power bus MOSFET control pins
#define CELL_A_MOS_EN(a)	_TRISD3 = !a
#define CELL_B_MOS_EN(a)	_TRISD2 = !a

#define M1_DIR_EN(a)		_TRISD6 = !a
#define M1_BRAKE_EN(a)		_TRISD7 = !a
#define M1_MODE_EN(a)		_TRISD9 = !a
#define M1_COAST_EN(a)		_TRISD10 = !a

#define M2_DIR_EN(a)		_TRISB4 = !a
#define M2_BRAKE_EN(a)		_TRISB5 = !a
#define M2_MODE_EN(a)		_TRISG9 = !a
#define M2_COAST_EN(a)		_TRISG8 = !a

#define M3_DIR_EN(a)		_TRISE3 = !a
#define M3_BRAKE_EN(a)		_TRISF0 = !a
#define M3_MODE_EN(a)		_TRISE4 = !a
#define M3_COAST_EN(a)		_TRISF1 = !a

/*
#define I2C_CLK_RPn RPOR10bits.RP20R
#define I2C_DAT_RPn RPOR12bits.RP25R
*/
//functional pins
#define M1_DIRO		_RD0
#define M1_DIR 		_LATD6
#define M1_BRAKE 	_LATD7
#define M1_MODE 	_LATD9
#define M1_COAST 	_LATD10
#define M1_FF1 		_RC14
#define M1_FF2  	_RC13
#define M2_DIRO		_RE5
#define M2_DIR 		_LATB4
#define M2_BRAKE 	_LATB5
#define M2_MODE 	_LATG9
#define M2_COAST 	_LATG8
#define M2_FF1 		_RG6
#define M2_FF2  	_RG7

//need to add M3_DIRO, M3_FF1, M3_FF2?
#define M3_DIR 		_LATE3
#define M3_BRAKE 	_LATF0
#define M3_MODE 	_LATE4
#define M3_COAST 	_LATF1
 

//#define M3_Fault 	PORTBbits.RB2
//fan fails aren't connected anymore
/*#define Fan1_Fail 	
#define Fan2_Fail 	*/
#define Cell_A_MOS 	_LATD3
#define Cell_B_MOS 	_LATD2


//other constant
#define LMotor 0
#define RMotor 1
#define Flipper 2
#define ChannelA 0
#define ChannelB 1
#define ControlMode_Conservative 0
#define ControlMode_Normal 1
#define ControlMode_Agressive 2
#define ControlMode_Customized 3

#define BackEMFSampleRangeStart 1800 //BackEMF sampling range starts about 90% of PWM period
#define BackEMFSampleRangeEnd 1840 	//BackEMF sampling range ends about 92% of PWM period

//#define BATVoltageLimit 650 //11.06V, 3.3V-1024, 430K-100K voltage divider, 1024->17.49V
#define BATVoltageLimit 800 //11.06V, 3.3V-1024, 430K-100K voltage divider, 1024->17.49V

//control mode
#define SpeedControl 0
#define CurrentControl 1
// LMotor RMotor Flipper can never be changed
#define Plus 1
#define Minus 0
#define ThresholdRPM 9600
#define SampleLength 4
#define ShiftBits 2

//I2C Device Address
#define TMPSensorICAddressW 0b10010010
#define TMPSensorICAddressR 0b10010011
#define FANCtrlAddressW 0b00110000
#define FANCtrlAddressR 0b00110001
#define FAN_CONTROLLER_ADDRESS		0x18
#define	BATTERY_ADDRESS				0x0b
#define EEPROM_ADDRESS            0x50
#define BATTERY_CHARGER_ADDRESS 0x0c

#define Fan1LowTemp 45// 45C fan1 start temperature
#define Fan2LowTemp 45// 45C fan2 start temperature

//Subsystem control
#define Available 0
#define TMPSensorIC 1
#define FANCtrl 2


/////variable

extern int16_t Robot_Motor_TargetSpeedUSB[3];
extern int NEW_ROBOT_MOTOR_SPEED_RECEIVED;
extern int gNewData;
//PulseWidth[0]-unknown 
//PulseWidth[1]-Serov1
//PulseWidth[2]-Serov2


/////function

/////function

void PWM2Ini(void);//initialize PWM chnnel 2
void PWM2Duty(int Duty);//set duty cycle for PWM channel 2
//Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM3Ini(void);//initialize PWM chnnel 3
void PWM3Duty(int Duty);//set duty cycle for PWM channel 3
//Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM4Ini(void);//initialize PWM chnnel 4
void PWM4Duty(int Duty);//set duty cycle for PWM channel 4
//Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM5Ini(void);//initialize PWM chnnel 5
void PWM5Duty(int Duty);//set duty cycle for PWM channel 5
//Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM6Ini(void);//initialize PWM chnnel 6
void PWM6Duty(int Duty);//set duty cycle for PWM channel 6
//Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM7Ini(void);//initialize PWM chnnel 7
void PWM7Duty(int Duty);//set duty cycle for PWM channel 7
//Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM8Ini(void);//initialize PWM chnnel 8
void PWM8Duty(int Duty);//set duty cycle for PWM channel 8
//Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

void PWM9Ini(void);//initialize PWM chnnel 9
void PWM9Duty(int Duty);//set duty cycle for PWM channel 9
//Duty is 0~1024, 1024-100% duty cycle,0-0% duty cycle

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
void init_io(void);
void ProtectHB(int Channel);
void PinRemap(void);
unsigned int GetPulseWidth(int Channel);
int EventChecker();
int GetChannel();
int GetSpeed();
void Braking(int Channel);
void UpdateSpeed(int Channel,int State);
int GetExpTimer();
void DeviceRobotMotorInit();
void Device_MotorController_Process();
int GetDuty(long CurrentState, long Target, int RTCurrent, int Channel, int Mode);
void ServoInput();
void UART1Ini();
void UART1Tranmit( int data);
void GetRPM(int Channel);
void GetCurrent(int Channel);
void GetControlRPM(int Channel);
void GetControlCurrent(int Channel);
void Cell_Ctrl(int Channel, int state);
void ServoInput();
void ClearSpeedCtrlData(int Channel);
void ClearCurrentCtrlData(int Channel);
int GetMotorTargetCoefficient(int Current);
void I2C1Ini();
void I2C2Ini();
void I2C3Ini();
void TMPSensorICIni();
void I2C3Update();
void FANCtrlIni();
void Motor_I2C3ResigsterWrite(int8_t ICAddW, int8_t RegAdd, int8_t Data);
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

void  Motor_ADC1Interrupt(void);

void Motor_CNInterrupt(void);


//Testing functions
void Motor_U1TXInterrupt(void);
void Motor_U1RXInterrupt(void);
void TestIO(void);
void TestIC1();
void TestPWM(void);
void TestIC2();
void TestOC();

extern int Cell_A_Current[SampleLength];
extern int Cell_B_Current[SampleLength];
extern int16_t Xbee_MOTOR_VELOCITY[3];
extern uint8_t Xbee_SIDE_FAN_SPEED;
extern uint8_t Xbee_SIDE_FAN_NEW;