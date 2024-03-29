#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "i2c.h"
#include "interrupt_switch.h"
#include "testing.h"
#include "debug_uart.h"
#include "device_robot_motor_i2c.h"
#include "DEE Emulation 16-bit.h"
#include "device_robot_motor_loop.h"
#include "../closed_loop_control/core/InputCapture.h"
#include <math.h>

#define XbeeTest
#define BATProtectionON
#define XbeeTest_TX_Enable


//variables
//sub system variables
static long int Period1;
static long int Period2;
static long int Period3;
static long int Period4;
static long int Period5;
static long int Period6;
static long int Period7;
static long int Period8;
static long int Period9;
//****************************************************

//****************************************************


unsigned int PulseWidth[3]={0,0,0};
int Event[3]={Stop,Stop,Stop};
int StateLevel01[3]={Protection,Protection,Protection};
int StateLevel02[3]={Locked,Locked,Locked};
long TargetParameter[3];//target speed or position
unsigned int CurrentParameter[3];//Current speed(for left and right motor) or position (for flipper)
int SwitchDirectionTimerExpired[3]={False,False,False};
int SwitchDirectionTimerEnabled[3]={False,False,False};
int SwitchDirectionTimerCount[3]={0,0,0};
int SpeedUpdateTimerExpired[3]={False,False,False};
int SpeedUpdateTimerEnabled[3]={False,False,False};
int SpeedUpdateTimerCount[3]={0,0,0};
int USBTimeOutTimerExpired=False;
long USBTimeOutTimerCount=0;
int USBTimeOutTimerEnabled=True;
int StateMachineTimerEnabled=True;
int StateMachineTimerExpired=False;
int StateMachineTimerCount=0;
int RPMTimerExpired=False;
int RPMTimerEnabled=True;
int RPMTimerCount=0;
int CurrentFBTimerExpired=False;
int CurrentFBTimerEnabled=True;
int CurrentFBTimerCount=0;
int M3_POSFB_TimerExpired=False;
int M3_POSFB_TimerEnabled=True;
int M3_POSFB_timerCount=0;
int CurrentProtectionTimerEnabled=True;
int CurrentProtectionTimerExpired=False;
int CurrentProtectionTimerCount=0;
int MotorOffTimerEnabled=False;
int MotorOffTimerExpired=False;
int MotorOffTimerCount=0;
int CurrentCtrlTimerEnabled=True;
int CurrentCtrlTimerExpired=False;
int CurrentCtrlTimerCount=0;
int CurrentSurgeRecoverTimerEnabled=False;
int CurrentSurgeRecoverTimerExpired=False;
int CurrentSurgeRecoverTimerCount=0;
int I2C2TimerEnabled=True;
int I2C2TimerCount=0;
int I2C3TimerEnabled=True;
int I2C3TimerCount=0;
int SFREGUpdateTimerEnabled=True;
int SFREGUpdateTimerExpired=False;
int SFREGUpdateTimerCount=0;
int BATVolCheckingTimerEnabled=True;
int BATVolCheckingTimerExpired=False;
int BATVolCheckingTimerCount=0;
int BATRecoveryTimerEnabled=False;
int BATRecoveryTimerExpired=True;	//for initial powering of the power bus
int BATRecoveryTimerCount=0;
int closed_loop_control_timer_count = 0;
int closed_loop_control_timer = 10;
int Xbee_FanSpeedTimerEnabled=False;
int Xbee_FanSpeedTimerExipred=False;
int Xbee_FanSpeedTimerCount=0;


unsigned int ICLMotorOverFlowCount=0;
unsigned int ICRMotorOverFlowCount=0;
unsigned int LEncoderAOverFlowCount=0;
unsigned int LEncoderBOverFlowCount=0;
unsigned int REncoderAOverFlowCount=0;
unsigned int REncoderBOverFlowCount=0;

unsigned int LEncoderLastValue=0;
unsigned int LEncoderCurrentValue=0;
unsigned int REncoderLastValue=0;
unsigned int REncoderCurrentValue=0;

long Encoder_Interrupt_Counter[2] = {0,0};

long EncoderFBInterval[3][SampleLength]={{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int DIR[3][SampleLength]={{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int EncoderFBIntervalPointer[3]={0,0,0};
//long BackEMF[3][2][SampleLength]={{{0,0,0,0},{0,0,0,0}},{{0,0,0,0},{0,0,0,0}},{{0,0,0,0},{0,0,0,0}}};
//int BackEMFPointer=0;
long MotorCurrentAD[3][SampleLength]={{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int MotorCurrentADPointer=0;
long RealTimeCurrent[3]={0,0,0};
long Current4Control[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
int Current4ControlPointer[3]={0,0,0};
long ControlCurrent[3]={0,0,0};
int16_t CurrentRPM[3]={0,0,0};
long RPM4Control[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
int RPM4ControlPointer[3]={0,0,0};
long ControlRPM[3]={0,0,0};
long TotalCurrent;
//long BackEMFCOE[3][SampleLength]={{2932,2932,2932,2932},{2932,2932,2932,2932},{2932,2932,2932,2932}};
//long BackEMFCOEF[3]={2932,2932,2932};
//int BackEMFCOEPointer=0;
int EncoderICClock=10000;
long EnCount[3]={0,0,0};
//int Robot_Motor_TargetSpeedUSB[3];
int16_t Robot_Motor_TargetSpeedUSB[3]={0,0,0};
int NEW_ROBOT_MOTOR_SPEED_RECEIVED=False;
int Timer3Count=0;
//int BackEMFSampleEnabled=False;
int M3_POSFB=0;
int M3_POSFB_Array[2][SampleLength]={{0,0,0},{0,0,0}};
int M3_POSFB_ArrayPointer=0;
int Total_Cell_Current=0;
int Total_Cell_Current_Array[SampleLength]={0,0,0};
int Total_Cell_Current_ArrayPointer=0;
int InitialCellVoltage[2]={0,0};
int CellVoltage[2]={0,0};
int CellVoltageArray[2][SampleLength];
int CellVoltageArrayPointer=0;
int Cell_A_Current[SampleLength];
int Cell_B_Current[SampleLength];
//float SpeedCtrlKp[4][3]={{0.0002,0.0002,0.0002},{0.09,0.09,0.09},{0.09,0.09,0.09},{0.09,0.09,0.09}};//SpeedCtrlKp[i][j],i- control mode, j-LMotor, Right Motor, Flipper
float SpeedCtrlKp[4][3]={{0.2,0.2,0.2},{0.03,0.03,0.03},{0.09,0.09,0.09},{0.09,0.09,0.09}};//SpeedCtrlKp[i][j],i- control mode, j-LMotor, Right Motor, Flipper
float SpeedCtrlKi[4][3]={{0.01,0.01,0.01},{0.001,0.001,0.001},{0.2,0.2,0.2},{0.2,0.2,0.2}};
float SpeedCtrlKd[4][3]={{0.000,0.000,0.000},{0.001,0.001,0.001},{0.0,0.0,0.0},{0.0,0.0,0.0}};
/*float CurrentCtrlKp[4][3]={{1.5,1.5,1.5},{1.0,1.0,1.0},{1.5,1.5,1.5},{1.5,1.5,1.5}};
float CurrentCtrlKi[4][3]={{0.5,0.5,0.5},{0.01,0.01,0.01},{1.5,1.5,1.5},{1.5,1.5,1.5}};
float CurrentCtrlKd[4][3]={{0.00,0.00,0.00},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};*/
int ControlMode[3]={SpeedControl,SpeedControl,SpeedControl};
int SpeedCtrlMode[3]={ControlMode_Conservative,ControlMode_Conservative,ControlMode_Conservative};
long AccumulatedSpeedError[3]={0,0,0};
long AccumulatedCurrentError[3]={0,0,0};
long LastSpeedError[3]={0,0,0};
long LastCurrentError[3]={0,0,0};
long LastTarget[3]={0,0,0};
int OverCurrent=False;
int MotorDuty[3]={0,0,0};
int CurrentSurgeTimes=0;
int MotorSpeedTargetCoefficient[3];
float MotorCurrentTargetCoefficient=MotorCurrentTargetCoefficient_Normal;
float MaxDuty=1000.0;
int MotorRecovering=False;
int CurrentTooHigh=False;
long TargetDifference=0;
int CO;
//long BackEmfRPM[3];
//long BackEmfTemp3[3];
//long BackEmfTemp4[3];
long Debugging_Dutycycle[3];
long MotorTargetRPM[3];
long Debugging_MotorTempError[3];
int Timer5Count=0;
int8_t I2C3DataMSOut[20];//I2C3DataMSOut[0]--Lock indicator, 0-unlocked 1-locked;I2C3DataMSOut[1]--length of this packet
int I2C1Channel=Available;
int I2C2Channel=Available;
int I2C3Channel=Available;

unsigned int flipper_angle_offset = 0;
void calibrate_flipper_angle_sensor(void);
static void read_stored_angle_offset(void);

void turn_on_power_bus_new_method(void);
void turn_on_power_bus_old_method(void);
void turn_on_power_bus_hybrid_method(void);
int check_string_match(unsigned char *string1, unsigned char* string2, unsigned char length);

static void alternate_power_bus(void);


unsigned int adc_test_reg = 0;


#ifdef XbeeTest
 	#define XbeeTest_BufferLength 6
 	#define XbeeTest_StateIdle 0
 	#define XbeeTest_StateProcessing 1
	int XbeeTest_State=XbeeTest_StateIdle;
 	int16_t XbeeTest_Buffer[6];
 	int XbeeTest_BufferArrayPointer=0;
 	int XbeeTest_Temp;
 	uint16_t XbeeTest_Temp_u16;
 	#define XbeeTest_UART_Buffer_Length 4 //no more than 5 bytes based on 57600 baud and 1ms system loop
 	uint8_t XbeeTest_UART_Buffer[XbeeTest_UART_Buffer_Length];
 	uint8_t XbeeTest_UART_BufferPointer=0;
 	uint8_t XbeeTest_UART_DataNO=0;
	int16_t Xbee_MOTOR_VELOCITY[3];
	uint8_t Xbee_Incoming_Cmd[2];
	int Xbee_gNewData=0;
	uint8_t Xbee_StartBit=253;
	uint16_t EncoderInterval[3];// Encoder time interval
	uint16_t BuildNO=10009;
	uint8_t Xbee_SIDE_FAN_SPEED=0;
	uint8_t Xbee_SIDE_FAN_NEW=0; // if there is a cmd or no
	#define P1_Read_Register 10
	#define P1_Fan_Command 20
	#define P1_Low_Speed_Set 240
	#define P1_Calibration_Flipper 250
	#define P1_Restart 230
	uint8_t Xbee_Low_Speed_mode=0;
	uint8_t Xbee_Calibration=0;
#endif


void read_EEPROM_string(void);

void PWM1Duty(int Duty);
void PWM2Duty(int Duty);
void PWM3Duty(int Duty);
void PWM1Ini(void);
void PWM2Ini(void);
void PWM3Ini(void);

void set_firmware_build_time(void);

void initialize_i2c2_registers(void);
void initialize_i2c3_registers(void);

static unsigned int return_combined_pot_angle(unsigned int pot_1_value, unsigned int pot_2_value);
static unsigned int return_calibrated_pot_angle(unsigned int pot_1_value, unsigned int pot_2_value);


void handle_power_bus(void);

//invalid flipper pot thresholds.  These are very wide because the flipper pots are on a different 3.3V supply
//than the PIC
//If the flipper pot is below this threshold, it is invalid
#define LOW_POT_THRESHOLD 33
//If the flipper pot is above this threshold, it is invalid
#define HIGH_POT_THRESHOLD 990
#define FLIPPER_POT_OFFSET -55

void bringup_board(void)
{



}



//*********************************************//
//**chief functions
void ClearSpeedCtrlData(int Channel)
{

 	AccumulatedSpeedError[Channel]=0;
 	LastSpeedError[Channel]=0;
 	//printf("Control Data Cleared!\n");

}

void ClearCurrentCtrlData(int Channel)
{

 	AccumulatedCurrentError[Channel]=0;
 	LastCurrentError[Channel]=0;

}

void DeviceRobotMotorInit()
{
//local variables


  
  block_ms(100);
  ClrWdt();

  //turn_on_power_bus_old_method();
//  turn_on_power_bus_new_method();

	MC_Ini();

	#ifndef XbeeTest
// this will disable the Xbee Uart TX and RX setting, skip this.
		//init_debug_uart();
	#endif

  handle_power_bus();

  //turn_on_power_bus_hybrid_method();


	//initialize all modules


  	TMPSensorICIni();
 	FANCtrlIni();

	//this is a dead end that causes code not to build.
	//It feeds REG_ROBOT_BOARD_DATA which is not used
	//read_EEPROM_string();



 	//Call ProtectHB
	ProtectHB(LMotor);
	ProtectHB(RMotor);
	ProtectHB(Flipper);



	test_function();

	initialize_i2c2_registers();
	initialize_i2c3_registers();

  //read flipper position from flash, and put it into a module variable
  read_stored_angle_offset();

  //init variables for closed loop control
  closed_loop_control_init();

}

void GetCurrent(int Channel)
{
 	long temp=0;
 	int i;
 	//read the AD value
 	for(i=0;i<SampleLength;i++)
 	{
 	 	temp+=MotorCurrentAD[(int)Channel][i];
 	}
 	//average and save the value
 	temp>>=ShiftBits;//temp1/SampleLength
 	RealTimeCurrent[Channel]=temp;
 	Current4Control[Channel][Current4ControlPointer[Channel]]=temp;
 	Current4ControlPointer[Channel]++;
 	Current4ControlPointer[Channel]&=7;
}

void GetControlCurrent(int Channel)
{
 	long temp=0;
 	int i;
 	for(i=0;i<8;i++)
 	{
 		temp+=Current4Control[Channel][i];
 	}
 	temp>>=3;
 	ControlCurrent[Channel]=temp;
}

void GetControlRPM(int Channel)
{
 	long temp=0;
 	int i;
 	for(i=0;i<8;i++)
 	{
 		temp+=RPM4Control[Channel][i];
 	}
 	temp>>=3;
 	ControlRPM[Channel]=temp;
}

void GetRPM(int Channel)
{	
	uint32_t temp;
	if(Channel==0 && MotorDirection(0)>0){
		temp = IC_period(0);
		temp = RatioCommutationPeriodToMotorRpm / temp - CommutationPeriodToMotorRpmOffset;
		temp = temp >> 1;
// 		CurrentRPM[0] = (RatioCommutationPeriodToMotorRpm / IC_period(0)) - CommutationPeriodToMotorRpmOffset;
// 		CurrentRPM[0] = -CurrentRPM[0];
		CurrentRPM[0] = - (int16_t)temp;
	}

	
	if(Channel==0 && MotorDirection(0)==0){
		temp = IC_period(0);
		temp = RatioCommutationPeriodToMotorRpm / temp - CommutationPeriodToMotorRpmOffset;
		temp = temp >> 1;
		CurrentRPM[0] = (int16_t)temp;
// 		CurrentRPM[0] = (RatioCommutationPeriodToMotorRpm / IC_period(0)) - CommutationPeriodToMotorRpmOffset;
		
	}

	if(Channel==1 && MotorDirection(1)>0){
		temp = IC_period(1);
		temp = RatioCommutationPeriodToMotorRpm / temp - CommutationPeriodToMotorRpmOffset;
		temp = temp >> 1;
		CurrentRPM[1] = (int16_t) temp;
// 		CurrentRPM[1] = (RatioCommutationPeriodToMotorRpm / IC_period(1)) - CommutationPeriodToMotorRpmOffset;
		
	}
	if(Channel==1 && MotorDirection(1)==0){
		temp = IC_period(1);
// 		CurrentRPM[1] = (RatioCommutationPeriodToMotorRpm / IC_period(1)) - CommutationPeriodToMotorRpmOffset;
// 		CurrentRPM[1] = -CurrentRPM[1];
		temp = RatioCommutationPeriodToMotorRpm / temp - CommutationPeriodToMotorRpmOffset;
		temp = temp >> 1;
		CurrentRPM[1] = -(int16_t) temp;
	}
}


void Device_MotorController_Process()
{
 	int i;
 	long temp1,temp2;
	static int overcurrent_counter = 0;
  //check if software wants to calibrate flipper position
	#ifndef XbeeTest
  if(REG_MOTOR_VELOCITY.flipper == 12345)
  {
    calibrate_flipper_angle_sensor();
  }
	#endif

	#ifdef XbeeTest
  if(Xbee_Calibration==1)
  {
    calibrate_flipper_angle_sensor();
	//clear the flag just for safe
	Xbee_Calibration=0;
	Xbee_SIDE_FAN_SPEED=240;
	Xbee_SIDE_FAN_NEW=1; 
	Xbee_FanSpeedTimerEnabled=True;
	Xbee_FanSpeedTimerCount=0;
  }
	#endif

  IC_UpdatePeriods();
// 	I2C2Update();
//	I2C3Update();
 	//Check Timer
 	//Run control loop
	if(IFS0bits.T1IF==SET)
	{
 		PORTFbits.RF5=!PORTFbits.RF5;
 		//clear the flag
 		IFS0bits.T1IF=CLEAR;
 	 	//check LMotor,RMotor and Flipper
 		//start counting all the timers
 	 	if(StateMachineTimerEnabled==True)
 	 	{
 	 	 	StateMachineTimerCount++;
 	 	}
 		for (i=0;i<=2;i++)
 		{
 			if(SwitchDirectionTimerEnabled[i]==True)
 			{
 				SwitchDirectionTimerCount[i]++;
 			}
 			if(SpeedUpdateTimerEnabled[i]==True)
 			{
 			 	SpeedUpdateTimerCount[i]++;
 			}

 		}
 	 	if(CurrentFBTimerEnabled==True)
 	 	{
 	 	 	CurrentFBTimerCount++;
 	 	}
	 	if(RPMTimerEnabled==True)
 		{
 		 	RPMTimerCount++;
 	 	}
 	 	if(USBTimeOutTimerEnabled==True)
 	 	{
 		 	USBTimeOutTimerCount++;
 			//printf("USBTimeOutTimerCount:%ld\n",USBTimeOutTimerCount);
 	 	}
 	 	if(CurrentProtectionTimerEnabled==True)
 		{
 			CurrentProtectionTimerCount++;
 		}
 		if(MotorOffTimerEnabled==True)
 		{
 			MotorOffTimerCount++;
 		}
 		if(CurrentSurgeRecoverTimerEnabled==True)
 		{
 			CurrentSurgeRecoverTimerCount++;
 		}
 		if(I2C2TimerEnabled==True)
 		{
 			I2C2TimerCount++; 		
 		}
 		if(I2C3TimerEnabled==True)
 		{
 			I2C3TimerCount++; 		
 		}
 		if(SFREGUpdateTimerEnabled==True)
 		{
 			SFREGUpdateTimerCount++;
 		}
 		if(BATVolCheckingTimerEnabled==True)
 		{
 			BATVolCheckingTimerCount++;
 		}
 		if(BATRecoveryTimerEnabled==True)
 		{
 			BATRecoveryTimerCount++;
 		}
		#ifdef XbeeTest
			if(Xbee_FanSpeedTimerEnabled==True)
			{
				Xbee_FanSpeedTimerCount++;
				//printf("%d",Xbee_FanSpeedTimerCount);
			}
		#endif


    //this should run every 1ms
    closed_loop_control_timer_count++;
    alternate_power_bus();

	}



	//check if any timers are expired
  if(closed_loop_control_timer_count >= closed_loop_control_timer)
  {
    closed_loop_control_timer_count = 0;
    handle_closed_loop_control(OverCurrent);
  }
 	if(CurrentProtectionTimerCount>=CurrentProtectionTimer)
 	{
 		CurrentProtectionTimerExpired=True;
 	}
 	if(USBTimeOutTimerCount>=USBTimeOutTimer)
 	{
 	 	USBTimeOutTimerExpired=True;
 	}
 	for(i=0;i<=2;i++)
 	{
 	 	//check SwitchDirectionTimer
 	 	if(SwitchDirectionTimerCount[i]>=SwitchDirectionTimer)
 	 	{
 	 	 	SwitchDirectionTimerExpired[i]=True;
 	 	 	SwitchDirectionTimerEnabled[i]=False;//stop the timer
 	 	 	SwitchDirectionTimerCount[i]=0;//clear the timer
 	 	}
 	 	if(SpeedUpdateTimerCount[i]>=SpeedUpdateTimer)
 	 	{
 	 	 	SpeedUpdateTimerExpired[i]=True;
 	 	 	SpeedUpdateTimerCount[i]=0;
 	 	}
 	}
 	if(RPMTimerCount>=RPMTimer)
 	{
 	 	RPMTimerExpired=True;
 	 	//RPMTimerCount=0;
 	 	//RPMTimerEnabled=False;
 	}
 	if(CurrentFBTimerCount>=CurrentFBTimer)
 	{
 	 	CurrentFBTimerExpired=True;
 	 	CurrentFBTimerCount=0;
 	 	CurrentFBTimerEnabled=False;
 	}
 	if(StateMachineTimerCount>=StateMachineTimer)
 	{
 	 	StateMachineTimerExpired=True;
 	}
 	if(MotorOffTimerCount>=MotorOffTimer)
 	{
 		MotorOffTimerExpired=True;
 	}
 	if(CurrentSurgeRecoverTimerCount>=CurrentSurgeRecoverTimer)
 	{
 		CurrentSurgeRecoverTimerExpired=True;
 	}
 	if(I2C2TimerCount>=I2C2Timer)
 	{

		//i2c2 didn't finish last time -- init variables so that
		//the value doesn't just stay the same
		if(I2C2TimerExpired==True)
		{
      re_init_i2c2();
		}
  	I2C2TimerExpired=True;
 		I2C2TimerCount=0;
 		I2C2XmitReset=True;
 	}
 	if(I2C3TimerCount>=I2C3Timer)
 	{
		if(I2C3TimerExpired==True)
		{
      re_init_i2c3();
		}
 		I2C3TimerExpired=True;
 		I2C3TimerCount=0;
 		I2C3XmitReset=True;
 	}
 	if(SFREGUpdateTimerCount>=SFREGUpdateTimer)
 	{
 		SFREGUpdateTimerExpired=True;
 		SFREGUpdateTimerCount=0;
 	}
 	if(BATVolCheckingTimerCount>=BATVolCheckingTimer)
 	{
 		BATVolCheckingTimerExpired=True;
 		BATVolCheckingTimerCount=0;
 	}
 	if(BATRecoveryTimerCount>=BATRecoveryTimer)
 	{
 		BATRecoveryTimerExpired=True;
 		BATRecoveryTimerCount=0;
 	}
	#ifdef XbeeTest
		if(Xbee_FanSpeedTimerCount>=Xbee_FanSpeedTimer)
		{
			Xbee_FanSpeedTimerExipred=True;
			Xbee_FanSpeedTimerCount=0;
			Xbee_FanSpeedTimerEnabled=False;
			//printf("fan speed timer expired!");
		}
	#endif

 	//if any of the timers expired, excute relative codes
 	//Control timer expired
 	for(i=0;i<=2;i++)
 	{
 	 	if(SpeedUpdateTimerExpired[i]==True)
 	 	{
 	 	 	UpdateSpeed(i,StateLevel01[i]);
 	 	 	SpeedUpdateTimerExpired[i]=False;
// 	 	 	test();
 	 	}
 	}
//t3

 	if(RPMTimerExpired==True)
 	{
 	 	//RPMTimerEnabled=True;
		RPMTimerCount = 0;
		RPMTimerExpired=True;
 	 	RPMTimerExpired=False;
 		for(i=0;i<2;i++)//only two driving motors, no flipper
 		{
 	 		GetRPM(i);
 		}
 	}
//T6

 	if(CurrentFBTimerExpired==True)
 	{
 	 	//clear CurrentFBTimerExpired
 	 	CurrentFBTimerExpired=False;
 	 	CurrentFBTimerEnabled=True;
 		for(i=0;i<3;i++)
 		{
 			GetCurrent(i);
 		}
 		TotalCurrent=RealTimeCurrent[LMotor]+RealTimeCurrent[RMotor]+RealTimeCurrent[Flipper];
		//printf("\nTotal Current%ld:\n",TotalCurrent);
 	} 
 	if(MotorOffTimerExpired==True)
 	{
 		OverCurrent=False;
 		MotorOffTimerExpired=False;
 		MotorOffTimerEnabled=False;
 		MotorOffTimerCount=0;
 		CurrentSurgeRecoverTimerEnabled=True;
 		CurrentSurgeRecoverTimerCount=0;
 		CurrentSurgeRecoverTimerExpired=False;
 		MotorRecovering=True;
 	}
 	if(CurrentSurgeRecoverTimerExpired==True)
 	{
 		CurrentSurgeRecoverTimerEnabled=False;
 		CurrentSurgeRecoverTimerCount=0;
 		CurrentSurgeRecoverTimerExpired=False;
 		MotorRecovering=False;
 	}
 	if(I2C2TimerExpired==True)
 	{
 		//update data on I2C3, reset the I2C data accquiring sequence
 		I2C2Update();
 	}
 	if(I2C3TimerExpired==True)
 	{
 		//update data on I2C3, reset the I2C data accquiring sequence
 		I2C3Update();
 	}
  	if(SFREGUpdateTimerExpired==True)
 	{
 		SFREGUpdateTimerExpired=False;
	 	//update all the software registers
 		//
 		REG_MOTOR_FB_RPM.left=CurrentRPM[LMotor];
 		REG_MOTOR_FB_RPM.right=CurrentRPM[RMotor];
 		//update flipper motor position
 		temp1=0;
 		temp2=0;
 		for(i=0;i<SampleLength;i++)
 		{
 			temp1+=M3_POSFB_Array[0][i];
 			temp2+=M3_POSFB_Array[1][i];
 		}
 		REG_FLIPPER_FB_POSITION.pot1=temp1>>ShiftBits;
 		REG_FLIPPER_FB_POSITION.pot2=temp2>>ShiftBits;
    	REG_MOTOR_FLIPPER_ANGLE = return_calibrated_pot_angle(temp1>>ShiftBits, temp2>>ShiftBits);
 		//update current for all three motors
 		REG_MOTOR_FB_CURRENT.left=ControlCurrent[LMotor];
 		REG_MOTOR_FB_CURRENT.right=ControlCurrent[RMotor];
 		REG_MOTOR_FB_CURRENT.flipper=ControlCurrent[Flipper];
 		//update the encodercount for two driving motors
 		//REG_MOTOR_ENCODER_COUNT.left=Encoder_Interrupt_Counter[LMotor];
		REG_MOTOR_ENCODER_COUNT.left=0;
		//REG_MOTOR_ENCODER_COUNT.right=Encoder_Interrupt_Counter[RMotor];
 		REG_MOTOR_ENCODER_COUNT.right=0;
 		//update the mosfet driving fault flag pin 1-good 2-fault
 		REG_MOTOR_FAULT_FLAG.left=PORTDbits.RD1;
 		REG_MOTOR_FAULT_FLAG.right=PORTEbits.RE5;
 		//update temperatures for two motors
 		//done in I2C code
 		//update batter voltage
 		temp1=0;
 		temp2=0;
 		for(i=0;i<SampleLength;i++)
 		{
 			temp1+=CellVoltageArray[Cell_A][i];
 			temp2+=CellVoltageArray[Cell_B][i];
 		}
 		REG_PWR_BAT_VOLTAGE.a=temp1>>ShiftBits;
 		REG_PWR_BAT_VOLTAGE.b=temp2>>ShiftBits;

 		//update total current (out of battery)
 		temp1=0;
 		for(i=0;i<SampleLength;i++)
 		{
 			temp1+=Total_Cell_Current_Array[i];
 		}
 		REG_PWR_TOTAL_CURRENT=temp1>>ShiftBits;

 	}
 	if(BATVolCheckingTimerExpired==True)
 	{

	//added this so that we check voltage faster
 		temp1=0;
 		temp2=0;
 		for(i=0;i<SampleLength;i++)
 		{
 		/*	temp1+=CellVoltageArray[Cell_A][i];
 			temp2+=CellVoltageArray[Cell_B][i];*/
			temp1+=Cell_A_Current[i];
			temp2+=Cell_B_Current[i];
 		}
 	/*	REG_PWR_BAT_VOLTAGE.a=temp1>>ShiftBits;
 		REG_PWR_BAT_VOLTAGE.b=temp2>>ShiftBits;
 		REG_PWR_BAT_VOLTAGE.a=CellVoltageArray[Cell_A][0];
 		REG_PWR_BAT_VOLTAGE.b=temp2>>ShiftBits;*/
		REG_PWR_A_CURRENT = temp1>>ShiftBits;
		REG_PWR_B_CURRENT = temp2>>ShiftBits;

 		BATVolCheckingTimerExpired=False;
 		#ifdef BATProtectionON
		//.01*.001mV/A * 11000 ohms = .11 V/A = 34.13 ADC counts/A
		//set at 10A per side
		if( ((temp1 >> ShiftBits) >= 512) || ( (temp2 >> ShiftBits) >=512))
		{
			//Cell_Ctrl(Cell_A,Cell_OFF);
 			//Cell_Ctrl(Cell_B,Cell_OFF);
			overcurrent_counter++;
			if(overcurrent_counter > 10)
			{
				PWM1Duty(0);
				PWM2Duty(0);
				PWM3Duty(0);
	 			ProtectHB(LMotor);
				ProtectHB(RMotor);
				ProtectHB(Flipper);
	 			OverCurrent=True;
	 			BATRecoveryTimerCount=0;
	 			BATRecoveryTimerEnabled=True;
	 			BATRecoveryTimerExpired=False;
			}

		}
		//
		else if( ((temp1 >> ShiftBits) <= 341) || ( (temp2 >> ShiftBits) <= 341))
		{
			overcurrent_counter = 0;
		}
 /*		if(REG_PWR_BAT_VOLTAGE.a<=BATVoltageLimit || REG_PWR_BAT_VOLTAGE.b<=BATVoltageLimit)//Battery voltage too low, turn off the power bus
 		{
 			Cell_Ctrl(Cell_A,Cell_OFF);
 			Cell_Ctrl(Cell_B,Cell_OFF);
 			ProtectHB(LMotor);
			ProtectHB(RMotor);
			ProtectHB(Flipper);
 			OverCurrent=True;
 			BATRecoveryTimerCount=0;
 			BATRecoveryTimerEnabled=True;
 			BATRecoveryTimerExpired=False;
			//block_ms(10000);
 		}*/
 		#endif
 	}
 	if(BATRecoveryTimerExpired==True)
 	{
 		Cell_Ctrl(Cell_A,Cell_ON);
 		Cell_Ctrl(Cell_B,Cell_ON);
 		OverCurrent=False;
 		BATRecoveryTimerExpired=False;
 		BATRecoveryTimerCount=0;
 		BATRecoveryTimerEnabled=False;
 	}
	//xbee fan timer
	#ifdef XbeeTest
		if(Xbee_FanSpeedTimerExipred==True)
		{
			Xbee_FanSpeedTimerExipred=False;
			Xbee_FanSpeedTimerEnabled=False;
			Xbee_FanSpeedTimerCount=0;
			// clear all the fan command
			Xbee_SIDE_FAN_SPEED=0;
			Xbee_SIDE_FAN_NEW=0;
			//printf("clear side fan speed!");
		}
	#endif
 	
//T5

 	 EventChecker();
 	if(CurrentProtectionTimerExpired==True)
 	{
 		CurrentProtectionTimerCount=0;
 	 	CurrentProtectionTimerExpired=False;
 	}
//T4 

 	//update state machine
 	if(StateMachineTimerExpired==True)
 	{
 	 	// clear the flag
 		//printf("State Machine Updated!\n");
 	 	StateMachineTimerExpired=False;
 	 	StateMachineTimerCount=0;
		for(i=0;i<=2;i++)
 	 	{
			//update state machine
			//Switch (StateLevel01)
			switch (StateLevel01[i])
			{
				//Case Brake:
				case Brake:
					//brake motor
					Braking(i);
					//Switch (Event)
					switch (Event[i])
					{
						//Case Go:
						case Go:
							//Call StartHBProtection
							ProtectHB(i);
							//StateLevel01=Protection
							StateLevel01[i]=Protection;
							//StateLevel02=Locked
							StateLevel02[i]=Locked;
							//break
							break;
						//Case Back:
						case Back:
							//Call StartHBProtection
							ProtectHB(i);
							//StateLevel01=Protection
							StateLevel01[i]=Protection;
							//StateLevel02=Locked
							StateLevel02[i]=Locked;
							//break
							break;
						//Case Stop:
						case Stop:
							//Event=NoEvent
							Event[i]=NoEvent;
							//break
							break;
					}
					//break
					break;
				//Case Forward:
				case Forward:
					//Switch(Event)
					switch(Event[i])
					{
						//Case Go:
						case Go:
 	 						SpeedUpdateTimerEnabled[i]=True;
							break;
						//Case Back:
						case Back:
							//Call StartHBProtection
							ProtectHB(i);
							//StateLevel01=Protection
							StateLevel01[i]=Protection;
							//StateLevel02=Locked
							StateLevel02[i]=Locked;
							//break
							break;
						//Case Stop:
						case Stop:
							//Call StartHBProtection
							ProtectHB(i);
							//StateLevel01=Protection
							StateLevel01[i]=Protection;
							//StateLevel02=Locked
							StateLevel02[i]=Locked;
							//break
							break;
					}
					//break
					break;					
				//Case Backwards:
				case Backwards:
					//Switch (Event)
					switch(Event[i])
					{
						//Case Go:
						case Go:
							//Call StartHBProtection
							ProtectHB(i);
							//StateLevel01=Protection
							StateLevel01[i]=Protection;
							//StateLevel02=Locked	
							StateLevel02[i]=Locked;	
							//break
							break;
						//Case Back:
						case Back:
 							SpeedUpdateTimerEnabled[i]=True;
							//break
							break;
						//Case Stop:
						case Stop:
							//Call StartHBProtection
							ProtectHB(i);
							//StateLevel01=Protection
							StateLevel01[i]=Protection;
							//StateLevel02=Locked
							StateLevel02[i]=Locked;
							//break
							break;
					}
					//break
					break;
				//Case Protection
				case Protection:
					//if StateLevel02==Locked
					if(StateLevel02[i]==Locked)
					{
						if(SwitchDirectionTimerExpired[i]==True)
						{
							//StateLevel02=Unlocked
							StateLevel02[i]=Unlocked;
							SwitchDirectionTimerExpired[i]=False;
 							SwitchDirectionTimerCount[i]=0;
						}
					}
					//if StateLevel02==Unlocked
					if(StateLevel02[i]==Unlocked)
					{
						//switch (Event)
						switch (Event[i])
						{
							//Case Stop
							case Stop:
								//Stop motor
								Braking(i);
								//StateLevel01=Brake
								StateLevel01[i]=Brake;
								//break
								break;
							//Case Go
							case Go:
 								SpeedUpdateTimerEnabled[i]=True;
								//StateLevel01=Forward
								StateLevel01[i]=Forward;
								//break
								break;
							//Case Back
							case Back:
 								SpeedUpdateTimerEnabled[i]=True;
								//StateLevel01=Backwards
								StateLevel01[i]=Backwards;
								//break
								break;
						}
					}						
					//break
					break;
			}
		}
 	}
 	//test();//run testing code


}








//**Motor controll functions

void Braking(int Channel)
{
 	switch (Channel)
 	{
 	 	case LMotor:
 			PWM1Duty(0);
			M1_COAST=Clear_ActiveLO;
 			Nop();
 			M1_BRAKE=Set_ActiveLO;
 	 	 	break;
 	 	case RMotor:
 	 	 	PWM2Duty(0);
			M2_COAST=Clear_ActiveLO;
 			Nop();
 			M2_BRAKE=Set_ActiveLO;
 	 	 	break;
 	 	case Flipper:
 			PWM3Duty(0);
			M3_COAST=Clear_ActiveLO;
 			Nop();
 			M3_BRAKE=Set_ActiveLO;			
 	 	 	break;
 	} 	
}

//disable the corresponding transistors preparing a direction switch
void ProtectHB(int Channel)
{
 	 SpeedUpdateTimerEnabled[Channel]=False;
 	 SpeedUpdateTimerCount[Channel]=0;
	//start timer
 	SwitchDirectionTimerEnabled[Channel]=True;
	//coast the motor
 	ClearSpeedCtrlData(Channel);
 	ClearCurrentCtrlData(Channel);
	switch(Channel)
 	{
 	 	case LMotor:
 			M1_COAST=Set_ActiveLO;
 			PWM1Duty(0);
 			M1_BRAKE=Clear_ActiveLO;
 	 		break;
 		case RMotor:
 			M2_COAST=Set_ActiveLO;
 			PWM2Duty(0);
 			M2_BRAKE=Clear_ActiveLO;
 			break;
 		case Flipper:
 			M3_COAST=Set_ActiveLO;
 			PWM3Duty(0);
 			M3_BRAKE=Clear_ActiveLO;
 			break;
 	}
}

int GetMotorSpeedTargetCoefficient(int Current)
{
 	int result;
 	long temp;
/*
 	if(TotalCurrent<=CurrentThreshold)
 	{
 		result=MotorSpeedTargetCoefficient_Normal;
 	}else
 	{
 		result=1+MotorSpeedTargetCoefficient_Normal-(TotalCurrent-CurrentThreshold)/(CurrentLimit-CurrentThreshold)*(MotorSpeedTargetCoefficient_Normal-MotorSpeedTargetCoefficient_Low);
 		//result=5;
 	}
*/
 	result=MotorSpeedTargetCoefficient_Normal;
 	temp=TargetDifference;
 	//turning?
 	if(temp<HardTurning && temp>StartTurning)
 	{
 		result=MotorSpeedTargetCoefficient_Turn+(HardTurning-temp)*(MotorSpeedTargetCoefficient_Normal-MotorSpeedTargetCoefficient_Turn)/(HardTurning-StartTurning);
 	 	//MaxDuty=750.0;
 	}
 	else if(temp>=HardTurning)
 	{
 		result=MotorSpeedTargetCoefficient_Turn;
 		//MaxDuty=150.0;
 	}

 	//recovering protection
 	if(MotorRecovering==True)
 	{
 		result=MotorSpeedTargetCoefficient_Turn+(MotorSpeedTargetCoefficient_Normal-MotorSpeedTargetCoefficient_Turn)*CurrentSurgeRecoverTimerCount/CurrentSurgeRecoverTimer;
 	}
 	if(result>MotorSpeedTargetCoefficient_Normal) result=MotorSpeedTargetCoefficient_Normal;
 	if(result<MotorSpeedTargetCoefficient_Turn) result=MotorSpeedTargetCoefficient_Turn;
 	if(CurrentTooHigh==True) 
 	{
 		//MaxDuty=500.0;
 		//result=result/(1+TMR5/PR5);

 		//result=result/1.5;

 	}else
 	{
 		//MaxDuty=750.0;
 	}
 	CO=result;
 	return result; 	
}

int GetDuty(long CurrentState, long Target, int RTCurrent, int Channel, int Mode)
{
 	long TargetRPM;
// 	long TargetCurrent;
 	float result;
 	long TempSpeedError;
// 	long TempCurrentError;
/*
 	if(LastTarget[Channel]!=Target)
 	{
 		ClearSpeedCtrlData(Channel);
 	}
*/
// 	if(RTCurrent<=CurrentThreshold)

 	if(1)
 	{
 		//speed control
 		LastTarget[Channel]=Target;
 		MotorSpeedTargetCoefficient[Channel]=GetMotorSpeedTargetCoefficient(Channel);
 		TargetRPM=(labs(Target)*MotorSpeedTargetCoefficient[Channel])>>2;//abstract number-> RPM-- 17000 RPM (1000) is the top
 		MotorTargetRPM[Channel]=TargetRPM;
 		TempSpeedError=TargetRPM-labs(CurrentState);
 		AccumulatedSpeedError[Channel]+=TempSpeedError;
 		Debugging_MotorTempError[Channel]=TempSpeedError;
 		//PID Control
 		//printf("Temp Error:%ld,Kp:%f, Accumulated Error:%li, Ki:%f\n",TempSpeedError,SpeedCtrlKp[Mode][Channel],AccumulatedSpeedError[Channel],SpeedCtrlKi[Mode][Channel]);
 		result=TempSpeedError*SpeedCtrlKp[Mode][Channel]+AccumulatedSpeedError[Channel]*SpeedCtrlKi[Mode][Channel]+(TempSpeedError-LastSpeedError[Channel])*SpeedCtrlKd[Mode][Channel];
 		LastSpeedError[Channel]=TempSpeedError;
		if(result>=MaxDuty)
		{
		 	result=MaxDuty;
 			if(TempSpeedError>0)
 			{
				AccumulatedSpeedError[Channel]-=TempSpeedError;
 			}
		} 	
		if(result<0.0)
		{
			result=0.0;
			//AccumulatedSpeedError[Channel]-=TempSpeedError;
			AccumulatedSpeedError[Channel]=0;
		}
 	}/*else
 	{
 		//current control
 		if(MotorRecovering==True || CurrentTooHigh==True)
 		{
 			MotorCurrentTargetCoefficient=MotorCurrentTargetCoefficient_Turn;
 		}else
 		{
 			MotorCurrentTargetCoefficient=MotorCurrentTargetCoefficient_Normal*2*labs(TargetParameter[Channel])/(labs(TargetParameter[0])+labs(TargetParameter[1])+labs(TargetParameter[2]));
 		}
 		TargetCurrent=labs(Target)*MotorCurrentTargetCoefficient;
 		TempCurrentError=TargetCurrent-labs(RTCurrent);
 		AccumulatedCurrentError[Channel]+=TempCurrentError;
 	
 		//PID Control
 		//printf("Temp Error:%ld,Kp:%f, Accumulated Error:%li, Ki:%f\n",TempCurrentError,CurrentCtrlKp[Mode][Channel],AccumulatedCurrentError[Channel],CurrentCtrlKi[Mode][Channel]);
 		result=TempCurrentError*CurrentCtrlKp[Mode][Channel]+AccumulatedCurrentError[Channel]*CurrentCtrlKi[Mode][Channel]+(TempCurrentError-LastCurrentError[Channel])*CurrentCtrlKd[Mode][Channel];
 		LastCurrentError[Channel]=TempCurrentError;
		if(result>=MaxDuty)
		{
		 	result=MaxDuty;
 			if(TempCurrentError>0)
 			{
				AccumulatedCurrentError[Channel]-=TempCurrentError;
 			}
		} 	
		if(result<0.0)
		{
			result=0.0;
			AccumulatedCurrentError[Channel]-=TempCurrentError;
			//AccumulatedCurrentError[Channel]=0;
		}
 	}*/
 	//printf("Target RPM:%ld,Current RPM:%ld,Dutycycle:%f,Accumulated Error:%ld",TargetRPM,labs(CurrentState),result,AccumulatedCurrentError[Channel]);


 	if(Channel==Flipper)
 	{
 		MotorDuty[Channel]=Target;
 	 	return Target;
 	}else
 	{
 		MotorDuty[Channel]=result;
 		result=labs(Target);
 		if(result>=MaxDuty) result=MaxDuty;
 		return result;
 	}
}

void UpdateSpeed(int Channel,int State)
{
 	int Dutycycle;
 	int temp;
 	//int CtrlMode;
 	GetControlRPM(Channel);
 	GetControlCurrent(Channel);
	TargetDifference=labs(TargetParameter[LMotor]-TargetParameter[RMotor]);
 	temp=TargetDifference;
 	//turning?
	if(temp>=HardTurning)
 	{
 		SpeedCtrlMode[Channel]=ControlMode_Conservative;
 	}else
 	{
 		SpeedCtrlMode[Channel]=ControlMode_Normal;
 	}
 	switch (Channel)
 	{
 	 	case LMotor:
 	 	 	if(State==Forward)
 	 	 	{
 				if(OverCurrent==True)
 				{
 					Dutycycle=0;
 				}else
 				{
 					Dutycycle=GetDuty(ControlRPM[Channel],TargetParameter[Channel],ControlCurrent[Channel], Channel,SpeedCtrlMode[Channel]);
 				}
 				//printf("PWM2 %d\n",Dutycycle);
 	 	 	 	if(OverCurrent==True)
 				{
 					M1_COAST=Set_ActiveLO;
 				}else
 				{
 					M1_COAST=Clear_ActiveLO;
 				}
 				M1_BRAKE=Clear_ActiveLO;
 				M1_DIR=HI;
 				PWM1Duty(Dutycycle); 				
 	 	 	}
 	 	 	else if (State==Backwards)
 	 	 	{
 				if(OverCurrent==True)
 				{
 					Dutycycle=0;
 				}else
 				{
 					Dutycycle=GetDuty(ControlRPM[Channel],TargetParameter[Channel],ControlCurrent[Channel], Channel,SpeedCtrlMode[Channel]);
 				}
 				if(OverCurrent==True)
 				{
 					M1_COAST=Set_ActiveLO;
 				}else
 				{
 					M1_COAST=Clear_ActiveLO;
 				}
 				M1_BRAKE=Clear_ActiveLO;
 				M1_DIR=LO;
 				PWM1Duty(Dutycycle);
 	 	 	}
 	 	 	break;
 	 	case RMotor:
 	 	 	if(State==Forward)
 	 	 	{
 				if(OverCurrent==True)
 				{
 					Dutycycle=0;
 				}else
 				{
 					Dutycycle=GetDuty(ControlRPM[Channel],TargetParameter[Channel],ControlCurrent[Channel], Channel,SpeedCtrlMode[Channel]);
 				}
 				if(OverCurrent==True)
 				{
 					M2_COAST=Set_ActiveLO;
 				}else
 				{
 					M2_COAST=Clear_ActiveLO;
 				}
 				M2_BRAKE=Clear_ActiveLO;
 				M2_DIR=LO;
 				PWM2Duty(Dutycycle);
 	 	 	}
 	 	 	else if (State==Backwards)
 	 	 	{
 				if(OverCurrent==True)
 				{
 					Dutycycle=0;
 				}else
 				{
 					Dutycycle=GetDuty(ControlRPM[Channel],TargetParameter[Channel],ControlCurrent[Channel], Channel,SpeedCtrlMode[Channel]);
 				}
 				if(OverCurrent==True)
 				{
 					M2_COAST=Set_ActiveLO;
 				}else
 				{
 					M2_COAST=Clear_ActiveLO;
 				}
 				M2_BRAKE=Clear_ActiveLO;
 				M2_DIR=HI;
 				PWM2Duty(Dutycycle); 
 	 	 	}
 	 	 	break;
 	 	case Flipper:
 	 	 	if(State==Forward)
 	 	 	{
 				Dutycycle=GetDuty(CurrentParameter[Channel],TargetParameter[Channel],ControlCurrent[Channel],Channel,SpeedCtrlMode[Channel]);
 				M3_COAST=Clear_ActiveLO;
 				Nop();
 				M3_BRAKE=Clear_ActiveLO;
 				Nop();
 				M3_DIR=HI;
 				PWM3Duty(Dutycycle); 				
 	 	 	}
 	 	 	else if (State==Backwards)
 	 	 	{
 				Dutycycle=GetDuty(CurrentParameter[Channel],-TargetParameter[Channel],ControlCurrent[Channel],Channel,SpeedCtrlMode[Channel]);
 				M3_COAST=Clear_ActiveLO;
 				Nop();
 				M3_BRAKE=Clear_ActiveLO;
 				Nop();
 				M3_DIR=LO;
 				PWM3Duty(Dutycycle); 
 	 	 	}
 	 	 	break;
 	}
 	Debugging_Dutycycle[Channel]=Dutycycle;
}

void UART1Tranmit( int data)
{
 	while(U1STAbits.UTXBF==1);
 	IFS0bits.U1TXIF=0; 	
 	U1TXREG=data;
// 	M1_RESET=~M1_RESET;
}

//*********************************************//

void ServoInput()
{
 	long int ECSpeed;
	long int temp;
 	int i;
	int Tor=10;//tolerance

	for(i=0;i<3;i++)
	{	
		//1.5ms pulse width->Stop
		if(PulseWidth[i]<=3000+Tor && PulseWidth[i]>=3000-Tor)
		{
			Event[i]=Stop;//Get the event
 			TargetParameter[i]=0;
		}
		//1-1.5ms pulse width->Back
		else if(PulseWidth[i]>=2000 && PulseWidth[i]<=3000-Tor)
		{
			Event[i]=Back;//Get the event
			temp=PulseWidth[i];
			ECSpeed=(3000-temp)*1024/1000;
			TargetParameter[i]=-ECSpeed;//Save the speed
			//PulseWidth[i]=0;//clear pulse width info
		}		
		//1.5-2ms pulse width->Go
		else if(PulseWidth[i]>=3000+Tor && PulseWidth[i]<=4000)
		{
			Event[i]=Go;//Get the event
			temp=PulseWidth[i];
			ECSpeed=(temp-3000)*1024/1000;			
			TargetParameter[i]=ECSpeed;//Save the speed
			//PulseWidth[i]=0;//clear pulse width info
		}
  	}
 	//printf("%u,%u,%u\n",Event[LMotor],Event[RMotor],Event[Flipper]);	
}

int speed_control_loop(unsigned char i, int desired_speed)
{
	static int motor_speed[3] = {0,0,0};
//	static int motor_rpm[3] = {0,0,0};

//	return desired_speed;
/*	if(i != 2)
	{

		//motor_rpm[i] = CurrentRPM[i];
		if(i==0)
			motor_rpm[i] = REG_MOTOR_FB_RPM.left;
		else
			motor_rpm[i] = REG_MOTOR_FB_RPM.right;
		if(desired_speed == 0)
		{
			motor_rpm[i] = 0;
			return 0;
		}
	
		if(motor_rpm[i] < desired_speed)
		{
		motor_speed[i]++;
		//if(motor_rpm[i] >= desired_speed)
			//motor_speed[i] = desired_speed;
		if(motor_speed[i] > 1000)
			motor_speed[i] = 1000;
		}
		else if (motor_rpm[i] > desired_speed)
		{
			motor_speed[i]--;
			//if(motor_speed[i] <= desired_speed)
			//	motor_speed[i] = desired_speed;
			if(motor_speed[i] < -1000)
				motor_speed[i] = -1000;
		}
	
		
			
		if(OverCurrent)
		{
			motor_speed[0] = 0;
			motor_speed[1] = 0;
			motor_speed[2] = 0;
	
		}
	
		return motor_speed[i];


	}
	else
	{*/

		if(desired_speed == 0)
		{
			motor_speed[i] = 0;
			return 0;
		}
	
		if(motor_speed[i] < desired_speed)
		{
			motor_speed[i]++;
			if(motor_speed[i] >= desired_speed)
				motor_speed[i] = desired_speed;
		}
		else if (motor_speed[i] > desired_speed)
		{
			motor_speed[i]--;
			if(motor_speed[i] <= desired_speed)
				motor_speed[i] = desired_speed;
		}
	
		
			
		if(OverCurrent)
		{
			motor_speed[0] = 0;
			motor_speed[1] = 0;
			motor_speed[2] = 0;
	
		}
	
		return motor_speed[i];
//	}

}

void USBInput()
{
  	int i;
	static int USB_New_Data_Received;
	static unsigned int control_loop_counter = 0;
	static unsigned int flipper_control_loop_counter = 0;

  typedef enum {
    HIGH_SPEED = 0,
    DECEL_AFTER_HIGH_SPEED,
    LOW_SPEED,
    DECEL_AFTER_LOW_SPEED
  } motor_speed_state;

  static motor_speed_state state = HIGH_SPEED;
    

//Motors must decelerate upon switching speeds (if they're not already stopped.
//Failure to do so could cause large current spikes which will trigger the 
//protection circuitry in the battery, cutting power to the robot
  switch (state)
  {
    case HIGH_SPEED:

    	control_loop_counter++;
    	flipper_control_loop_counter++;

    	if(control_loop_counter > 5)
    	{
    		control_loop_counter = 0;
			#ifndef XbeeTest
				//printf("wrong loop");
    	 		Robot_Motor_TargetSpeedUSB[0]=speed_control_loop(0,REG_MOTOR_VELOCITY.left);
    	 		Robot_Motor_TargetSpeedUSB[1]=speed_control_loop(1,REG_MOTOR_VELOCITY.right);
			#endif
			#ifdef XbeeTest
				//printf("XL%d,XR,%d",Xbee_MOTOR_VELOCITY[0],Robot_Motor_TargetSpeedUSB[1]);
				Robot_Motor_TargetSpeedUSB[0]=speed_control_loop(0,Xbee_MOTOR_VELOCITY[0]);
				Robot_Motor_TargetSpeedUSB[1]=speed_control_loop(1,Xbee_MOTOR_VELOCITY[1]);
				//printf("L%d,R%d\n",Robot_Motor_TargetSpeedUSB[0],Robot_Motor_TargetSpeedUSB[1]);
			#endif
    	}
    
    	if(flipper_control_loop_counter > 15)
    	{
    		flipper_control_loop_counter  = 0;
			#ifndef XbeeTest
    			Robot_Motor_TargetSpeedUSB[2]=speed_control_loop(2,REG_MOTOR_VELOCITY.flipper);
			#endif
			#ifdef XbeeTest
				Robot_Motor_TargetSpeedUSB[2]=speed_control_loop(2,Xbee_MOTOR_VELOCITY[2]);
				//printf();
			#endif
    	}
	#ifdef XbeeTest
      if(REG_MOTOR_SLOW_SPEED == 1 )
        state = DECEL_AFTER_HIGH_SPEED;
	#endif
	#ifdef XbeeTest
      if( Xbee_Low_Speed_mode==1)
        state = DECEL_AFTER_HIGH_SPEED;
	#endif

    break;
  
    case DECEL_AFTER_HIGH_SPEED:

    	control_loop_counter++;

    	if(control_loop_counter > 3)
    	{
    		control_loop_counter = 0;

        //set speed to 1 (speed of 0 immediately stops motors)
    	 	Robot_Motor_TargetSpeedUSB[0]=speed_control_loop(0,1);
    	 	Robot_Motor_TargetSpeedUSB[1]=speed_control_loop(1,1);
    		Robot_Motor_TargetSpeedUSB[2]=speed_control_loop(2,1);
    	}

      //motors are stopped if speed falls below 1%
      if ( (abs(Robot_Motor_TargetSpeedUSB[0])<10) && (abs(Robot_Motor_TargetSpeedUSB[1])<10) && (abs(Robot_Motor_TargetSpeedUSB[2])<10) )
        state = LOW_SPEED;

    break;

    case LOW_SPEED:
		#ifndef XbeeTest
			set_desired_velocities(REG_MOTOR_VELOCITY.left,REG_MOTOR_VELOCITY.right,REG_MOTOR_VELOCITY.flipper);
		#endif
		#ifdef XbeeTest
			set_desired_velocities(Xbee_MOTOR_VELOCITY[0],Xbee_MOTOR_VELOCITY[1],Xbee_MOTOR_VELOCITY[2]);
		#endif
		//printf("low speed loop!");
  	 	Robot_Motor_TargetSpeedUSB[0]=return_closed_loop_control_effort(0);
  	 	Robot_Motor_TargetSpeedUSB[1]=return_closed_loop_control_effort(1);
      	Robot_Motor_TargetSpeedUSB[2]=return_closed_loop_control_effort(2);
  		//printf("AA:%d,%d,%d",Robot_Motor_TargetSpeedUSB[0],Robot_Motor_TargetSpeedUSB[1],Robot_Motor_TargetSpeedUSB[2]);
      control_loop_counter = 0;
      flipper_control_loop_counter = 0;
	#ifndef XbeeTest
      if(REG_MOTOR_SLOW_SPEED == 0)
        state = DECEL_AFTER_LOW_SPEED;
	#endif
	#ifdef XbeeTest
      if(Xbee_Low_Speed_mode==0)
        state = DECEL_AFTER_LOW_SPEED;
	#endif


    break;

    case DECEL_AFTER_LOW_SPEED:

      set_desired_velocities(0,0,0);
  
  	 	Robot_Motor_TargetSpeedUSB[0]=return_closed_loop_control_effort(0);
  	 	Robot_Motor_TargetSpeedUSB[1]=return_closed_loop_control_effort(1);
      Robot_Motor_TargetSpeedUSB[2]=return_closed_loop_control_effort(2);

      //motors are stopped if speed falls below 1%
      if ( (abs(Robot_Motor_TargetSpeedUSB[0])<10) && (abs(Robot_Motor_TargetSpeedUSB[1])<10) && (abs(Robot_Motor_TargetSpeedUSB[2])<10) )
        state = HIGH_SPEED;

    break;

  }

    //gNewData=!gNewData;

	//long time no data, clear everything
 	if(USBTimeOutTimerExpired==True)
 	{
		//printf("USB Timer Expired!");
 		USBTimeOutTimerExpired=False;
 		USBTimeOutTimerEnabled=False;
 		USBTimeOutTimerCount=0;
 		REG_MOTOR_VELOCITY.left=0;
 		REG_MOTOR_VELOCITY.right=0;
 		REG_MOTOR_VELOCITY.flipper=0;
		#ifdef XbeeTest
			Xbee_MOTOR_VELOCITY[0]=0;
			Xbee_MOTOR_VELOCITY[1]=0;
			Xbee_MOTOR_VELOCITY[2]=0;
		#endif
 		for(i=0;i<3;i++)
 		{
 			Robot_Motor_TargetSpeedUSB[i]=0;
			Event[i]=Stop;//Get the event
 			TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];
 			//ClearSpeedCtrlData(i);
 			//ClearCurrentCtrlData(i);
 		}
		#ifndef XbeeTest
			//send_debug_uart_string("USB Timeout Detected \r\n",23);
		#endif
 	}
	//printf("!\n");
#ifndef XbeeTest
	// if there is new data comming in, update all the data
 	if(USB_New_Data_Received!=gNewData)
 	{
 		USB_New_Data_Received=gNewData;
 		USBTimeOutTimerCount=0;
 		USBTimeOutTimerEnabled=True;
 		USBTimeOutTimerExpired=False;
		//printf("1");
 		//printf("Lmotor:%d",Robot_Motor_TargetSpeedUSB[0]);
		for(i=0;i<3;i++)
		{	
			//Robot_Motor_TargetSpeedUSB[i]==0 ->Stop
			if(Robot_Motor_TargetSpeedUSB[i]==0)
			{
				Event[i]=Stop;//Get the event
 				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];
				//printf("2");
			}
			//-1024<Robot_Motor_TargetSpeedUSB[i]<0 ->Back
			else if(Robot_Motor_TargetSpeedUSB[i]>-1024 && Robot_Motor_TargetSpeedUSB[i]<0)
			{
				Event[i]=Back;//Get the event
				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];
				//printf("3");
			}		
			//0<Robot_Motor_TargetSpeedUSB[i]<1024 ->Go
			else if(Robot_Motor_TargetSpeedUSB[i]>0 && Robot_Motor_TargetSpeedUSB[i]<1024)
			{
				Event[i]=Go;//Get the event
				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];//Save the speed
				//printf("4");
			}
  		}
 	}
#endif

#ifdef XbeeTest
 	if(USB_New_Data_Received!=Xbee_gNewData)
 	{
 		USB_New_Data_Received=Xbee_gNewData;
 		USBTimeOutTimerCount=0;
 		USBTimeOutTimerEnabled=True;
 		USBTimeOutTimerExpired=False;
		//printf("1");
 		//printf("LM:%d",Robot_Motor_TargetSpeedUSB[0]);
		for(i=0;i<3;i++)
		{	
			//Robot_Motor_TargetSpeedUSB[i]==0 ->Stop
			if(Robot_Motor_TargetSpeedUSB[i]==0)
			{
				Event[i]=Stop;//Get the event
 				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];
				//printf("2");
			}
			//-1024<Robot_Motor_TargetSpeedUSB[i]<0 ->Back
			else if(Robot_Motor_TargetSpeedUSB[i]>-1024 && Robot_Motor_TargetSpeedUSB[i]<0)
			{
				Event[i]=Back;//Get the event
				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];
				//printf("3");
			}		
			//0<Robot_Motor_TargetSpeedUSB[i]<1024 ->Go
			else if(Robot_Motor_TargetSpeedUSB[i]>0 && Robot_Motor_TargetSpeedUSB[i]<1024)
			{
				Event[i]=Go;//Get the event
				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];//Save the speed
				//printf("4");
			}
  		}
 	}
#endif
}

int EventChecker()
{
 	USBInput();
// 	ServoInput();
	return 1;
}



//**********************************************


void PinRemap(void)
{
	// Unlock Registers
	//clear the bit 6 of OSCCONL to
	//unlock Pin Re-map
	/*asm volatile	("push	w1				\n"
					"push	w2				\n"
					"push	w3				\n"
					"mov	#OSCCON, w1		\n"
					"mov	#0x46, w2		\n"
					"mov	#0x57, w3		\n"
					"mov.b	w2, [w1]		\n"
					"mov.b	w3, [w1]		\n"
					"bclr	OSCCON, #6		\n"
					"pop	w3				\n"
					"pop	w2				\n"
					"pop	w1");*/

	// Configure Input Functions
	//function=pin
	// Assign IC1 To L_Encoder_A
	//RPINR7bits.IC1R = M1_TACHO_RPn; 
	

	// Assign IC3 To Encoder_R1A
	//RPINR8bits.IC3R = M2_TACHO_RPn;
	

 	#ifdef XbeeTest
 	// Assign U1RX To U1RX, Uart receive channnel
		RPINR18bits.U1RXR = U1RX_RPn;
 	#endif

	//***************************	
	// Configure Output Functions
	//pin->function
	// Assign OC1 To Pin M1_AHI
	M1_PWM = 18; //18 represents OC1

	// Assign OC2 To Pin M1_BHI
	M2_PWM = 19; //19 represents OC2

	// Assign OC3 To Pin M2_AHI
	M3_PWM = 20; //20 represents OC3

 	// Assign OC4 To Pin M2_BHI
	//M2_BLO_RPn = 21; //21 represents OC4

 	// Assign OC5 To Pin M3_AHI
	//M3_ALO_RPn = 22; //22 represents OC5

 	// Assign OC6 To Pin M3_BHI
 	//M3_BLO_RPn = 23; //23 represents OC6

/*
 	// Assign I2C Clock (SPI1 Clock Output) to Pin I2C_CLK
 	I2C_CLK_RPn = 8; //8 represents SPI1 Clock Output

 	// Assign I2C Data (SPI1 Date Output) to Pin I2C_DAT
 	I2C_DAT_RPn = 7; //7 represents SPI1 Data Output
*/

 	#ifdef XbeeTest
 	// Assign U1TX To U1TX/Uart Tx
	U1TX_RPn = 3; //3 represents U1TX
 	#endif
	//***************************
	// Lock Registers
	//__builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to
	//lock Pin Re-map

}



void Cell_Ctrl(int Channel, int state)
{
 	switch (Channel)
 	{
 		case Cell_A:
 			switch(state)
 			{
 				case Cell_ON:
 					Cell_A_MOS=1;
 					break;
 				case Cell_OFF:
 					Cell_A_MOS=0;
 					break;
 			}
 			break;
 		case Cell_B:
 			switch(state)
 			{
 				case Cell_ON:
 					Cell_B_MOS=1;
 					break;
 				case Cell_OFF:
 					Cell_B_MOS=0;
 					break;
 			}
 			break;
 	}
}

void set_firmware_build_time(void)
{

	const unsigned char build_date[12] = __DATE__; 
	const unsigned char build_time[12] = __TIME__;
	unsigned int i;

	for(i=0;i<12;i++)
	{
		if((build_date[i] == 0))
		{
			REG_MOTOR_FIRMWARE_BUILD.data[i] = ' ';
		}
		else
		{
			REG_MOTOR_FIRMWARE_BUILD.data[i] = build_date[i];
		}
		if(build_time[i] == 0)
		{
			REG_MOTOR_FIRMWARE_BUILD.data[i+12] = ' ';
		}
		else
		{
			REG_MOTOR_FIRMWARE_BUILD.data[i+12] = build_time[i];
		}
	}

}


void MC_Ini(void)//initialzation for the whole program
{
	//make sure we start off in a default state
	AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;

	AD1PCFGL = 0xffff;
	TRISB = 0xffff;
	TRISC = 0xffff;
	TRISD = 0xffff;
	TRISE = 0xffff;
	TRISF = 0xffff;
	TRISG = 0xffff;

	//peripheral pin selection
	PinRemap();
	//peripheral pin selection end
	//*******************************************
	//initialize I/O port
	//AN15,AN14,AN1,AN0 are all digital

	//initialize all of the analog inputs
		AD1PCFG = 0xffff;
		M1_TEMP_EN(1);
		M1_CURR_EN(1);
		M2_TEMP_EN(1);
		M2_CURR_EN(1);
		M3_TEMP_EN(1);
		M3_CURR_EN(1);
		VCELL_A_EN(1);
		VCELL_B_EN(1);
		CELL_A_CURR_EN(1);
		CELL_B_CURR_EN(1);
		M3_POS_FB_1_EN(1);
		M3_POS_FB_2_EN(1);

	//initialize the digital outputs
		CELL_A_MOS_EN(1);
		CELL_B_MOS_EN(1);

		M1_DIR_EN(1);
		M1_BRAKE_EN(1);
		M1_MODE_EN(1);
		M1_COAST_EN(1);

		M2_DIR_EN(1);
		M2_BRAKE_EN(1);
		M2_MODE_EN(1);
		M2_COAST_EN(1);

		M3_DIR_EN(1);
		M3_BRAKE_EN(1);
		M3_MODE_EN(1);
		M3_COAST_EN(1);




 	//I/O initializing complete

 	//Initialize motor drivers
 	M1_MODE=1;
 	M2_MODE=1;
	M3_MODE=1;
	//*******************************************
 	InterruptIni();
	//initialize interrupt on change


 	//initialize AD
 	IniAD();
	//initialize timer
	IniTimer2();
	IniTimer3();
	IniTimer1();
// 	IniTimer4();
// 	IniTimer5();
	//initialize PWM sub module
 	PWM1Ini();
	PWM2Ini();
	PWM3Ini();
/*	PWM4Ini();
	PWM5Ini();
	PWM6Ini();
	PWM7Ini();
	PWM8Ini();
	PWM9Ini();*/

	//initialize input capture
	//IniIC1();
	//IniIC3();

 	I2C1Ini();
 	I2C2Ini();
 	I2C3Ini();

 	#ifdef XbeeTest
 	UART1Ini();
 	#endif

	set_firmware_build_time();

}

void InterruptIni()
{
 	//remap all the interrupt routines
// 	T2InterruptUserFunction=Motor_T2Interrupt;
 	T3InterruptUserFunction=Motor_T3Interrupt;
// 	T4InterruptUserFunction=Motor_T4Interrupt;
// 	T5InterruptUserFunction=Motor_T5Interrupt;
// 	IC1InterruptUserFunction=Motor_IC1Interrupt;
// 	IC3InterruptUserFunction=Motor_IC3Interrupt;
 	ADC1InterruptUserFunction=Motor_ADC1Interrupt;
 	#ifdef XbeeTest
 		U1TXInterruptUserFunction=Motor_U1TXInterrupt;
 		U1RXInterruptUserFunction=Motor_U1RXInterrupt;
 	#endif
}

void I2C3ResigsterWrite(int8_t ICAddW, int8_t RegAdd, int8_t Data)
{
 	IdleI2C3();
	StartI2C3();
	IdleI2C3();

	MasterWriteI2C3(ICAddW);
	IdleI2C3();

	MasterWriteI2C3(RegAdd);
 	IdleI2C3();
	MasterWriteI2C3(Data);
	IdleI2C3();
 	StopI2C3();
	IdleI2C3();

}

void FANCtrlIni()
{

  unsigned int i;

  block_ms(20);
  ClrWdt();

 	//reset the IC
 	writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x02,0b01011000);
  block_ms(20);
  ClrWdt();

 	//auto fan speed control mode
 	//writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x11,0b00111100);

	//manufal fan speed control mode
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x11,0x00);

	block_ms(20);

	writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x12,0);

	block_ms(20);

	//writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x13,0xff);

	block_ms(20);

	REG_MOTOR_SIDE_FAN_SPEED = 48;

	writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x0B,240);
  for(i=0;i<10;i++)
  {
  	ClrWdt();
    block_ms(250);
    ClrWdt();
  	block_ms(250);
  	ClrWdt();
  }
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x0B,0);

	block_ms(20);
	ClrWdt();

	//FAN is now controlled by temperature profile...
	uint8_t fan_lo_duty_cycle = (uint8_t)(FAN_MAX_DUTY * FAN_LO_DUTY_FACTOR);
    writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x07, fan_lo_duty_cycle);
	block_ms(20);
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x08, fan_lo_duty_cycle);
	block_ms(20);

    uint8_t fan_hi_duty_cycle = (uint8_t)(FAN_MAX_DUTY * FAN_HI_DUTY_FACTOR);
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x09, fan_hi_duty_cycle);
	block_ms(20);
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x0a, fan_hi_duty_cycle);
	block_ms(20);

	uint8_t fan_start_temp = (uint8_t)FAN_START_TEMP;
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x0F, fan_start_temp);
	block_ms(20);
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x10, fan_start_temp);
	block_ms(20);

	// set fan update rate
    uint8_t fan_duty_rate_of_change = 0b01001000;
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x12, fan_duty_rate_of_change);
    block_ms(20);

	// configure which temperature sensors to use
    uint8_t fan_config =
        0b00111100; // set both temperature inputs - it will use whichever is higher
    writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x11, fan_config);

	float duty_cycle_step =
        FAN_MAX_DUTY *
        (FAN_HI_DUTY_FACTOR - FAN_LO_DUTY_FACTOR) /
        (FAN_MAX_TEMP - FAN_START_TEMP) / 2;
	duty_cycle_step = ceil(duty_cycle_step);
	(duty_cycle_step > 255) ? duty_cycle_step = 255 : duty_cycle_step;
	(duty_cycle_step < 0) ? duty_cycle_step = 0 : duty_cycle_step;
    uint8_t duty_cycle_step_uint4 = (uint8_t)duty_cycle_step & 0xF;

    uint8_t duty_cycle_step_sizes = duty_cycle_step_uint4 << 4 | duty_cycle_step_uint4;
	writeI2C2Reg(FAN_CONTROLLER_ADDRESS, 0x13, duty_cycle_step_sizes);
    block_ms(20);
	
}

void TMPSensorICIni()
{
 	//use default settings
/*
 	I2C3DataMSOut[0]=1;//lock the I2C3 out data packet,
 	I2C3DataMSOut[1]=4;//packet length :3 int
 	I2C3DataMSOut[2]=(TMPSensorICAddress<<=1)&0xFE;//bit 1=0, write to slave
 	I2C3DataMSOut[3]=0b00000001;//configuration register
 	I2C3DataMSOut[4]=0b01100000;
 	I2C3DataMSOut[5]=0b10100000;
*/
}

void I2C1Ini()
{

}

void I2C2Ini()
{
	OpenI2C2(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, 0xff);

	IdleI2C2();
}

void I2C3Ini()
{

	OpenI2C3(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, 0xff);

	IdleI2C3();

}


/*****************************************************************************/
//*-----------------------------------Sub system-----------------------------*/

//*-----------------------------------PWM------------------------------------*/

//initialize PWM chnnel 1
void PWM1Ini(void)
{
//1. Configure the OCx output for one of the 
//available Peripheral Pin Select pins.
	//done in I/O Init.
//2. Calculate the desired duty cycles and load them
//into the OCxR register.
	OC1R=0;
//3. Calculate the desired period and load it into the
//OCxRS register.
	OC1RS=2000;
//4. Select the current OCx as the sync source by writing
//0x1F to SYNCSEL<4:0> (OCxCON2<4:0>),
//and clearing OCTRIG (OCxCON2<7>).
	OC1CON2bits.SYNCSEL=0x1F;
	OC1CON2bits.OCTRIG=CLEAR;
//5. Select a clock source by writing the
//OCTSEL<2:0> (OCxCON<12:10>) bits.
	OC1CON1bits.OCTSEL=0b000;//Timer2
//6. Enable interrupts, if required, for the timer and
//output compare modules. The output compare
//interrupt is required for PWM Fault pin utilization.
	//No interrupt needed
//7. Select the desired PWM mode in the OCM<2:0>
//(OCxCON1<2:0>) bits.
	OC1CON1bits.OCM=0b110;
//8. If a timer is selected as a clock source, set the
//TMRy prescale value and enable the time base by
//setting the TON (TxCON<15>) bit.
	//Done in timer Init.
	Period1=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 2
void PWM1Duty(int Duty)
{
	OC1R = Duty*2;
}
//****************************************************


//initialize PWM chnnel 2
void PWM2Ini(void)
{

	OC2R=0;
	OC2RS=2000;
	OC2CON2bits.SYNCSEL=0x1F;
	OC2CON2bits.OCTRIG=CLEAR;
	OC2CON1bits.OCTSEL=0b000;//Timer2
	OC2CON1bits.OCM=0b110;
	Period2=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 2
void PWM2Duty(int Duty)
{
	OC2R=Duty*2;
}
//****************************************************

//****************************************************
//initialize PWM chnnel 3
void PWM3Ini(void)
{
	OC3R=0;
	OC3RS=2000;
	OC3CON2bits.SYNCSEL=0x1F;
	OC3CON2bits.OCTRIG=CLEAR;
	OC3CON1bits.OCTSEL=0b000;//Timer2
	OC3CON1bits.OCM=0b110;
	Period3=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 3
void PWM3Duty(int Duty)
{
	OC3R=Duty*2;
}
//****************************************************

//****************************************************
//initialize PWM chnnel 4
void PWM4Ini(void)
{
	OC4R=0;
	OC4RS=Period30000Hz;
	OC4CON2bits.SYNCSEL=0x1F;
	OC4CON2bits.OCTRIG=CLEAR;
	OC4CON1bits.OCTSEL=0b000;//Timer2
	OC4CON1bits.OCM=0b110;
	Period4=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 4
void PWM4Duty(int Duty)
{
	OC4R=(Period4*Duty)>>10;
}
//****************************************************

//****************************************************
//initialize PWM chnnel 5
void PWM5Ini(void)
{
	OC5R=0;
	OC5RS=Period30000Hz;
	OC5CON2bits.SYNCSEL=0x1F;
	OC5CON2bits.OCTRIG=CLEAR;
	OC5CON1bits.OCTSEL=0b000;//Timer2
	OC5CON1bits.OCM=0b110;
	Period5=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 5
void PWM5Duty(int Duty)
{
	OC5R=(Period5*Duty)>>10;
}
//****************************************************

//****************************************************
//initialize PWM chnnel 6
void PWM6Ini(void)
{
	OC6R=0;
	OC6RS=Period30000Hz;
	OC6CON2bits.SYNCSEL=0x1F;
	OC6CON2bits.OCTRIG=CLEAR;
	OC6CON1bits.OCTSEL=0b000;//Timer2
	OC6CON1bits.OCM=0b110;
	Period6=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 6
void PWM6Duty(int Duty)
{
	OC6R=(Period6*Duty)>>10;
}
//****************************************************

//****************************************************
//initialize PWM chnnel 7
void PWM7Ini(void)
{
	OC7R=0;
	OC7RS=Period30000Hz;
	OC7CON2bits.SYNCSEL=0x1F;
	OC7CON2bits.OCTRIG=CLEAR;
	OC7CON1bits.OCTSEL=0b000;//Timer2
	OC7CON1bits.OCM=0b110;
	Period7=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 7
void PWM7Duty(int Duty)
{
	OC7R=(Period7*Duty)>>10;
}
//****************************************************

//****************************************************
//initialize PWM chnnel 8
void PWM8Ini(void)
{
	OC8R=0;
	OC8RS=Period30000Hz;
	OC8CON2bits.SYNCSEL=0x1F;
	OC8CON2bits.OCTRIG=CLEAR;
	OC8CON1bits.OCTSEL=0b000;//Timer2
	OC8CON1bits.OCM=0b110;
	Period8=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 8
void PWM8Duty(int Duty)
{
	OC8R=(Period8*Duty)>>10;
}
//****************************************************

//****************************************************
//initialize PWM chnnel 9
void PWM9Ini(void)
{
	OC9R=0;
	OC9RS=Period30000Hz;
	OC9CON2bits.SYNCSEL=0x1F;
	OC9CON2bits.OCTRIG=CLEAR;
	OC9CON1bits.OCTSEL=0b000;//Timer2
	OC9CON1bits.OCM=0b110;
	Period9=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 9
void PWM9Duty(int Duty)
{
	OC9R=(Period9*Duty)>>10;
}
//****************************************************


/*****************************************************************************/
//*-----------------------------------Timer----------------------------------*/

void IniTimer1()
{
	T1CON=0x0000;//clear register
 	T1CONbits.TCKPS=0b00;//1:1 prescale
	//T1CONbits.TCKPS=0b01;//timer stops,1:8 prescale,
	TMR1=0;//clear timer1 register
	PR1=Period1000Hz;//interrupt every 1ms
	T1CONbits.TON=SET;
}

void IniTimer2()
{
	T2CON=0x0000;//stops timer2,16 bit timer,internal clock (Fosc/2)
 	T2CONbits.TCKPS=0b00;//1:1 prescale
	TMR2=0;//clear timer1 register
 	PR2=Period30000Hz;
 	IFS0bits.T2IF=CLEAR;//clear the flag
 	//IEC0bits.T2IE=SET;// enable the interrupt
	T2CONbits.TON=SET;
}
void IniTimer3()
{
//timer 3 initialize
 	//A/D triger timer, for back emf feedback

	T3CON=0x0010;//stops timer3,1:8 prescale,16 bit timer,internal clock (Fosc/2)
	TMR3=0;//clear timer1 register
 	PR3=Period50000Hz;// timer 3 is 50 times faster than PWM timer (timer 2)
 	IFS0bits.T3IF=CLEAR; //clear interrupt flag
 	IEC0bits.T3IE=SET;//enable the interrupt
	T3CONbits.TON=SET;//start clock

//end timer 3 initialize

}

void IniTimer4()
{
/*	T4CON=0x0010;//stops timer4,1:8 prescale,16 bit timer,internal clock (Fosc/2)
	TMR4=0;//clear timer1 register
 	IFS1bits.T4IF = 0;	//clear interrupt flag
 	//IEC1bits.T4IE=SET;
	T4CONbits.TON=SET;*/

}

void IniTimer5()
{
	T5CON=0x0010;//stops timer3,1:8 prescale,16 bit timer,internal clock (Fosc/2)
	TMR5=0;//clear timer1 register
 	PR5=Period67Hz;// timer 5 -> 15ms
 	IFS1bits.T5IF=CLEAR; //clear interrupt flag
 	IEC1bits.T5IE=CLEAR;//disenable the interrupt
	T5CONbits.TON=CLEAR;//stop clock
}

void IniIC1()
{
	int temp;
//1. Configure the ICx input for one of the available
//Peripheral Pin Select pins.
	//Done before
//2. If Synchronous mode is to be used, disable the
//sync source before proceeding.
	//No need
//3. Make sure that any previous data has been
//removed from the FIFO by reading ICxBUF until
//the ICBNE bit (ICxCON1<3>) is cleared.
	while(IC1CON1bits.ICBNE==SET)
	{
	 	temp=IC1BUF;
 	}
//4. Set the SYNCSEL bits (ICxCON2<4:0>) to the
//desired sync/trigger source.
	IC1CON2bits.SYNCSEL=0b00000;// not snycronized to anything
//5. Set the ICTSEL bits (ICxCON1<12:10>) for the
//desired clock source.
	IC1CON1bits.ICTSEL=0b010;//timer 4
//6. Set the ICI bits (ICxCON1<6:5>) to the desired
//interrupt frequency
	IC1CON1bits.ICI=0b00; 	//interrupt on every capture event
//7. Select Synchronous or Trigger mode operation:
//a) Check that the SYNCSEL bits are not set to?0000?
/*
 	if(IC5CON2bits.SYNCSEL==CLEAR)
	{
	 	IC5CON2bits.SYNCSEL=0b10100; 	//sync with IC1
 	}
*/	
//b) For Synchronous mode, clear the ICTRIG
//bit (ICxCON2<7>).
	IC1CON2bits.ICTRIG=0b0;//synchronous mode
//c) For Trigger mode, set ICTRIG, and clear the
//TRIGSTAT bit (ICxCON2<6>).

//8. Set the ICM bits (ICxCON1<2:0>) to the desired
//operational mode.
	IC1CON1bits.ICM=0b011;//capture on every rising edge
//9. Enable the selected trigger/sync source.

	IFS0bits.IC1IF=CLEAR;//clear the interrupt flag	
	IEC0bits.IC1IE=SET;//start the interrupt
}


void IniIC3()
{
	int temp;
//1. Configure the ICx input for one of the available
//Peripheral Pin Select pins.
	//Done before
//2. If Synchronous mode is to be used, disable the
//sync source before proceeding.
	//No need
//3. Make sure that any previous data has been
//removed from the FIFO by reading ICxBUF until
//the ICBNE bit (ICxCON1<3>) is cleared.
	while(IC3CON1bits.ICBNE==SET)
	{
	 	temp=IC3BUF;
 	}
//4. Set the SYNCSEL bits (ICxCON2<4:0>) to the
//desired sync/trigger source.
	IC3CON2bits.SYNCSEL=0b00000;// not snycronized to anything
//5. Set the ICTSEL bits (ICxCON1<12:10>) for the
//desired clock source.
	IC3CON1bits.ICTSEL=0b010;//timer 4
//6. Set the ICI bits (ICxCON1<6:5>) to the desired
//interrupt frequency
	IC3CON1bits.ICI=0b00; 	//interrupt on every capture event
//7. Select Synchronous or Trigger mode operation:
//a) Check that the SYNCSEL bits are not set to?0000?
/*
 	if(IC5CON2bits.SYNCSEL==CLEAR)
	{
	 	IC5CON2bits.SYNCSEL=0b10100; 	//sync with IC1
 	}
*/	
//b) For Synchronous mode, clear the ICTRIG
//bit (ICxCON2<7>).
	IC3CON2bits.ICTRIG=0b0;//synchronous mode
//c) For Trigger mode, set ICTRIG, and clear the
//TRIGSTAT bit (ICxCON2<6>).

//8. Set the ICM bits (ICxCON1<2:0>) to the desired
//operational mode.
	IC3CON1bits.ICM=0b011;//capture on every rising edge
//9. Enable the selected trigger/sync source.

	IFS2bits.IC3IF=CLEAR;//clear the interrupt flag	
	IEC2bits.IC3IE=SET;//start the interrupt
}




/*****************************************************************************/


/*****************************************************************************/
//*-----------------------------------A/D------------------------------------*/
void IniAD()
{
//1. Configure the A/D module:
//a) Configure port pins as analog inputs and/or
//select band gap reference inputs (AD1PCFGL<15:0> and AD1PCFGH<1:0>).
 	//none
//b) Select voltage reference source to match
//expected range on analog inputs (AD1CON2<15:13>).
 	AD1CON2bits.VCFG=0b000;// VR+: AVDD, VR-: AVSS
//c) Select the analog conversion clock to
//match desired data rate with processor clock (AD1CON3<7:0>).
 	AD1CON3bits.ADCS=0b00000001;// TAD= 2TCY = 125 ns, at least 75ns required

//d) Select the appropriate sample
//conversion sequence (AD1CON1<7:5> and AD1CON3<12:8>).
 	AD1CON1bits.SSRC=0b111;//auto conversion
 	AD1CON3bits.SAMC=0b01111;//auto-sample time=15*TAD=1.875uS
//e) Select how conversion results are
//presented in the buffer (AD1CON1<9:8>).
 	AD1CON1bits.FORM=0b00;//integer (0000 00dd dddd dddd)
//f) Select interrupt rate (AD1CON2<5:2>).
 	AD1CON2bits.SMPI=0b1011;//interrupt every 12 samples convert sequence
//g) scan mode, select input channels (AD1CSSL<15:0>)
  //AD1CSSL=0b0011111100111111;
	AD1CSSL=0b1111111100001111;
 	AD1CON2bits.CSCNA=SET;
//h) Turn on A/D module (AD1CON1<15>).
 	AD1CON1bits.ADON=SET;
//2. Configure A/D interrupt (if required):
 	IEC0bits.AD1IE=SET;
//a) Clear the AD1IF bit.
 	IFS0bits.AD1IF=CLEAR;
//b) Select A/D interrupt priority.	
}




/*****************************************************************************/


/*****************************************************************************/
//*-----------------------------------UART------------------------------------*/
void UART1Ini()
{
 	// Write appropriate baud rate value to the UxBRG register.
 	U1BRG=BaudRate_57600_HI;
 	//Enable the UART.
 	U1MODE=0x0000;
 	//hight speed mode
 	U1MODEbits.BRGH=1;
 	U1STA=0x0000; 	
 	U1MODEbits.UARTEN=1;//UART1 is enabled
 	U1STAbits.UTXEN=1;//transmit enabled
 	IFS0bits.U1TXIF=0;//clear the transmit flag
 	IEC0bits.U1TXIE=1;//enable UART1 transmit interrupt
 	IEC0bits.U1RXIE=1;//enable UART1 receive interrupt
}




/*****************************************************************************/
//*-----------------------------------UART------------------------------------*/





/*****************************************************************************/





/***************************Interrupt routines***********************************/



void  Motor_IC1Interrupt(void)
{
 	//int temp,index;

 	IFS0bits.IC1IF=0;//clear the interrupt flag
 	//make sure pull all the data from the buffer 	
 	while(IC1CON1bits.ICBNE==SET)
 	{
 	 	LEncoderCurrentValue=IC1BUF;
 	 	EncoderFBInterval[LMotor][EncoderFBIntervalPointer[LMotor]]=LEncoderCurrentValue-LEncoderLastValue;
 		if(EncoderFBInterval[LMotor][EncoderFBIntervalPointer[LMotor]]<0 || LEncoderAOverFlowCount>0)
 	 	{
 			EncoderFBInterval[LMotor][EncoderFBIntervalPointer[LMotor]]+=65535*LEncoderAOverFlowCount;
 	 		LEncoderAOverFlowCount=0;
 	 	}
 	 	LEncoderLastValue=LEncoderCurrentValue;
 	 	EncoderFBIntervalPointer[LMotor]++; 
 	 	EncoderFBIntervalPointer[LMotor]&=(SampleLength-1);
 	}

	Encoder_Interrupt_Counter[LMotor]++; 	 	
}

void  Motor_IC3Interrupt(void)
{
 	IFS2bits.IC3IF=0;//clear the flag
 	//make sure pull all the data from the buffer 	
 	while(IC3CON1bits.ICBNE==SET)
 	{
 	 	REncoderCurrentValue=IC3BUF;
 	 	EncoderFBInterval[RMotor][EncoderFBIntervalPointer[RMotor]]=REncoderCurrentValue-REncoderLastValue;
 		if(EncoderFBInterval[RMotor][EncoderFBIntervalPointer[RMotor]]<0 || REncoderAOverFlowCount>0)
 	 	{
 			EncoderFBInterval[RMotor][EncoderFBIntervalPointer[RMotor]]+=65535*REncoderAOverFlowCount;
 	 		REncoderAOverFlowCount=0;
 	 	}
 	 	REncoderLastValue=REncoderCurrentValue;
 	 	EncoderFBIntervalPointer[RMotor]++; 
 	 	EncoderFBIntervalPointer[RMotor]&=(SampleLength-1);
 	} 

	Encoder_Interrupt_Counter[RMotor]++; 
}



void  Motor_T4Interrupt(void)
{
 	IFS1bits.T4IF = 0;	//clear interrupt flag
 	ICLMotorOverFlowCount++;
 	ICRMotorOverFlowCount++;
 	LEncoderAOverFlowCount++;
 	LEncoderBOverFlowCount++;
 	REncoderAOverFlowCount++;
 	REncoderBOverFlowCount++;
}


void  Motor_T2Interrupt(void)
{

//hardcode removed
// 	TRISFbits.TRISF1=0;
// 	PORTFbits.RF1=~PORTFbits.RF1;
 	IFS0bits.T2IF = 0;	//clear interrupt flag
 	//T3CONbits.TON=SET;
}




void  Motor_T3Interrupt(void)
{
 	int temp;
 	//PORTCbits.RC13=~PORTCbits.RC13;
 	//clear timer3 flage
 	IFS0bits.T3IF=CLEAR; //clear interrupt flag
 	temp=TMR2;
 	Timer3Count++;

//TODO: turn on timer


	if(Timer3Count>=0)
	{
		Timer3Count = 0;
		AD1CON1bits.ASAM=SET;
	}

}

void  Motor_T5Interrupt(void)
{
 	int i;
 	IFS1bits.T5IF=CLEAR; //clear interrupt flag
	Timer5Count++;
 	if (Timer5Count>=3) //slow down the whole system within 45 ms 
 	{
 		IEC1bits.T5IE=CLEAR;//disable the interrupt
		T5CONbits.TON=CLEAR;//stop clock
 		TMR5=0;//clear the timer register
		CurrentTooHigh=False;
 		Timer5Count=0;
 	}
 	if(OverCurrent==False && TotalCurrent>=CurrentLimit)
 	{
 		OverCurrent=True;
 		MotorOffTimerEnabled=True;
 		MotorOffTimerExpired=False;
 		MotorOffTimerCount=0;
 		for(i=0;i<3;i++)
 		{
			ClearSpeedCtrlData(i);
			ClearCurrentCtrlData(i);
		}
 			//printf("\n Total current:%ld,\n",TotalCurrent);
 		//coast left motor
 		M1_COAST=Set_ActiveLO;
 		PWM1Duty(0);
 		M1_BRAKE=Clear_ActiveLO;
 		//coast right motor
 		M2_COAST=Set_ActiveLO;
 		PWM2Duty(0);
 		M2_BRAKE=Clear_ActiveLO;
 		//coast flipper
 		M3_COAST=Set_ActiveLO;
 		PWM1Duty(0);
 		M3_BRAKE=Clear_ActiveLO;

 	}
 			
}

void  Motor_ADC1Interrupt(void)
{
//	unsigned int temp = 0;
 	//stop the conversion
 	AD1CON1bits.ASAM=CLEAR;
 	
 	//clear the flag
 	IFS0bits.AD1IF=CLEAR;
 	//load the value

// 		BackEMFSampleEnabled=False;

 	M3_POSFB_Array[0][M3_POSFB_ArrayPointer]=ADC1BUF4;
 	M3_POSFB_Array[1][M3_POSFB_ArrayPointer]=ADC1BUF5; 	
 	MotorCurrentAD[LMotor][MotorCurrentADPointer]=ADC1BUF3;
 	MotorCurrentAD[RMotor][MotorCurrentADPointer]=ADC1BUF1;
 	MotorCurrentAD[Flipper][MotorCurrentADPointer]=ADC1BUFB;
	Cell_A_Current[Total_Cell_Current_ArrayPointer] = ADC1BUF8;
	Cell_B_Current[Total_Cell_Current_ArrayPointer] = ADC1BUF9;
	Total_Cell_Current_Array[Total_Cell_Current_ArrayPointer]=Cell_A_Current[Total_Cell_Current_ArrayPointer]+Cell_B_Current[Total_Cell_Current_ArrayPointer];
	//adc_test_reg = ADC1BUF9;
 	CellVoltageArray[Cell_A][CellVoltageArrayPointer]=ADC1BUF6;
 	CellVoltageArray[Cell_B][CellVoltageArrayPointer]=ADC1BUF7;
 	TotalCurrent=MotorCurrentAD[LMotor][MotorCurrentADPointer]+MotorCurrentAD[RMotor][MotorCurrentADPointer]+MotorCurrentAD[Flipper][MotorCurrentADPointer];

/* 	if(TotalCurrent>=CurrentLimit)
 	{
 		//start protection timer
		T5CONbits.TON=SET;//start clock
 		IEC1bits.T5IE=SET;//enable the interrupt
		CurrentTooHigh=True;
 	}*/
 
 	//increase array pointer, prevent over flow
 	CellVoltageArrayPointer++;
 	CellVoltageArrayPointer&=(SampleLength-1);
 	Total_Cell_Current_ArrayPointer++;
 	Total_Cell_Current_ArrayPointer&=(SampleLength-1);
 	M3_POSFB_ArrayPointer++;
 	M3_POSFB_ArrayPointer&=(SampleLength-1);
// 	BackEMFPointer++;
// 	BackEMFPointer&=(SampleLength-1);
 	MotorCurrentADPointer++;
 	MotorCurrentADPointer&=(SampleLength-1);


 	#ifdef XbeeTest
	EncoderInterval[0]=IC_period(kIC01); //left motor encoder time interval
	EncoderInterval[1]=IC_period(kIC02); //right motor encoder time interval
	EncoderInterval[3]=IC_period(kIC03); //Encoder motor encoder time interval
 	//if(Xbee_Incoming_Cmd[0]== P1_Read_Register && U1STAbits.UTXBF==0 && XbeeTest_UART_BufferPointer==0 && (Xbee_MOTOR_VELOCITY[0]||Xbee_MOTOR_VELOCITY[1]||Xbee_MOTOR_VELOCITY[2])!=0)//if transmit reg is empty and last packet is sent
 	if(Xbee_Incoming_Cmd[0]== P1_Read_Register && U1STAbits.UTXBF==0 && XbeeTest_UART_BufferPointer==0)//if transmit reg is empty and last packet is sent
 	{
 		U1TXREG=Xbee_StartBit;//send out the index
 		//XbeeTest_Temp_u16=REG_PWR_TOTAL_CURRENT;
 		//XbeeTest_Temp_u16=REG_MOTOR_FB_RPM.left;
 		//XbeeTest_Temp_u16=REG_MOTOR_FB_RPM.right;
 		//XbeeTest_Temp_u16=REG_FLIPPER_FB_POSITION.pot1;
 		//XbeeTest_Temp_u16=REG_FLIPPER_FB_POSITION.pot2;
 		//XbeeTest_Temp_u16=REG_MOTOR_FB_CURRENT.left;
 		//XbeeTest_Temp_u16=REG_MOTOR_FB_CURRENT.right;
 		//XbeeTest_Temp_u16=REG_MOTOR_ENCODER_COUNT.left;
 		//XbeeTest_Temp_u16=REG_MOTOR_ENCODER_COUNT.right;
 		//XbeeTest_Temp_u16=REG_MOTOR_FAULT_FLAG.left;
 		//XbeeTest_Temp_u16=REG_MOTOR_FAULT_FLAG.right;
 		//XbeeTest_Temp_u16=REG_MOTOR_TEMP.left;
 		//XbeeTest_Temp_u16=REG_MOTOR_TEMP.right;
 		//XbeeTest_Temp_u16=REG_PWR_BAT_VOLTAGE.a;
 		//XbeeTest_Temp_u16=REG_PWR_BAT_VOLTAGE.b;
 		//XbeeTest_Temp_u16=REG_PWR_TOTAL_CURRENT;
 		//XbeeTest_Temp_u16=TotalCurrent;
		XbeeTest_UART_DataNO=Xbee_Incoming_Cmd[1];
		XbeeTest_UART_Buffer[0]=XbeeTest_UART_DataNO;
 		//XbeeTest_UART_Buffer[1]=(XbeeTest_Temp_u16>>8);//load the buffer
 		//XbeeTest_UART_Buffer[2]=XbeeTest_Temp_u16;
		//XbeeTest_UART_Buffer[1]=REG_MOTOR_ENCODER_COUNT.left;//load the buffer
 		//XbeeTest_UART_Buffer[2]=REG_MOTOR_ENCODER_COUNT.right;
 		switch (XbeeTest_UART_DataNO%60)
 		{ 
 		 	case 0:		//0-REG_PWR_TOTAL_CURRENT HI
 		 		XbeeTest_UART_Buffer[1]=REG_PWR_TOTAL_CURRENT>>8;
 		 		XbeeTest_UART_Buffer[2]=REG_PWR_TOTAL_CURRENT;
 				break;
// 		 	case 1:		//1-REG_PWR_TOTAL_CURRENT LO
// 				XbeeTest_UART_Buffer[3]=REG_PWR_TOTAL_CURRENT;
// 				break;
 		 	case 2:		//2-REG_MOTOR_FB_RPM.left HI
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_FB_RPM.left>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_FB_RPM.left;
 				break;
// 		 	case 3: 	//3-REG_MOTOR_FB_RPM.left LO
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_RPM.left;
// 				break;
 		 	case 4: 	//4-REG_MOTOR_FB_RPM.right HI
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_FB_RPM.right>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_FB_RPM.right;
 				break;
// 		 	case 5: 	//5-REG_MOTOR_FB_RPM.right LO
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_RPM.right;
// 				break;
 		 	case 6: 	//6-REG_FLIPPER_FB_POSITION.pot1 HI
 				XbeeTest_UART_Buffer[1]=REG_FLIPPER_FB_POSITION.pot1>>8;
				XbeeTest_UART_Buffer[2]=REG_FLIPPER_FB_POSITION.pot1;
 				break;
// 		 	case 7: 	//7-REG_FLIPPER_FB_POSITION.pot1 LO
// 				XbeeTest_UART_Buffer[3]=REG_FLIPPER_FB_POSITION.pot1;
// 				break;
 		 	case 8: 	//8-REG_FLIPPER_FB_POSITION.pot2 HI
 				XbeeTest_UART_Buffer[1]=REG_FLIPPER_FB_POSITION.pot2>>8;
				XbeeTest_UART_Buffer[2]=REG_FLIPPER_FB_POSITION.pot2;
 				break;
// 		 	case 9: 	//9-REG_FLIPPER_FB_POSITION.pot2 LO
// 				XbeeTest_UART_Buffer[3]=REG_FLIPPER_FB_POSITION.pot2;
// 				break;
 			case 10: 	//10-REG_MOTOR_FB_CURRENT.left HI
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_FB_CURRENT.left>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_FB_CURRENT.left;
 				break;
// 			case 11: 	//11-REG_MOTOR_FB_CURRENT.left LO
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_CURRENT.left;
// 				break;
 		 	case 12: 	//12-REG_MOTOR_FB_CURRENT.right HI
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_FB_CURRENT.right>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_FB_CURRENT.right;
 				break;
// 			case 13:	//13-REG_MOTOR_FB_CURRENT.right LO
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_CURRENT.right;
// 				break;
 			case 14:	//14-REG_MOTOR_ENCODER_COUNT.left HI
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_ENCODER_COUNT.left>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_ENCODER_COUNT.left;
 				break;
// 		 	case 15:	//15-REG_MOTOR_ENCODER_COUNT.left LO
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_ENCODER_COUNT.left;
// 				break;
 			case 16:	//16-REG_MOTOR_ENCODER_COUNT.right HI
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_ENCODER_COUNT.right>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_ENCODER_COUNT.right;
 				break;
// 		 	case 17:	//17-REG_MOTOR_ENCODER_COUNT.right LO
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_ENCODER_COUNT.right;
// 				break;
 		 	case 18: 	//18-REG_MOTOR_FAULT_FLAG.left
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_FAULT_FLAG.left;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_FAULT_FLAG.right;
 				break;
// 		 	case 19:	//19-REG_MOTOR_FAULT_FLAG.right
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FAULT_FLAG.right;
// 				break;
 		 	case 20: 	//20-REG_MOTOR_TEMP.left HI
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_TEMP.left>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_TEMP.left;
 				break;
// 			case 21:	//21-REG_MOTOR_TEMP.left LO
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_TEMP.left;
// 				break;
 			case 22:	//22-REG_MOTOR_TEMP.right HI
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_TEMP.right>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_TEMP.right;
 				break;
// 			case 23:	//23-REG_MOTOR_TEMP.right LO
// 				XbeeTest_UART_Buffer[3]=REG_MOTOR_TEMP.right;
// 				break;
 			case 24:	//24-REG_PWR_BAT_VOLTAGE.a HI
 				XbeeTest_UART_Buffer[1]=REG_PWR_BAT_VOLTAGE.a>>8;
				XbeeTest_UART_Buffer[2]=REG_PWR_BAT_VOLTAGE.a;
 				break;
// 			case 25:	//25-REG_PWR_BAT_VOLTAGE.a LO
// 				XbeeTest_UART_Buffer[3]=REG_PWR_BAT_VOLTAGE.a;
// 				break;
 		 	case 26:	//26-REG_PWR_BAT_VOLTAGE.b HI
 				XbeeTest_UART_Buffer[1]=REG_PWR_BAT_VOLTAGE.b>>8;
				XbeeTest_UART_Buffer[2]=REG_PWR_BAT_VOLTAGE.b;
 				break;
// 			case 27:	//27-REG_PWR_BAT_VOLTAGE.b LO
// 				XbeeTest_UART_Buffer[3]=REG_PWR_BAT_VOLTAGE.b;
// 				break;
 			case 28:	//28-EncoderInterval[0]
 				XbeeTest_UART_Buffer[1]=EncoderInterval[0]>>8;
				XbeeTest_UART_Buffer[2]=EncoderInterval[0];
 				break;
 			case 30:	//30-EncoderInterval[1]
 				XbeeTest_UART_Buffer[1]=EncoderInterval[1]>>8;
				XbeeTest_UART_Buffer[2]=EncoderInterval[1];
 				break;
 			case 32:	//32-EncoderInterval[2]
 				XbeeTest_UART_Buffer[1]=EncoderInterval[2]>>8;
				XbeeTest_UART_Buffer[2]=EncoderInterval[2];
 				break;
 			case 34:	//34-REG_ROBOT_REL_SOC_A
 				XbeeTest_UART_Buffer[1]=REG_ROBOT_REL_SOC_A>>8;
				XbeeTest_UART_Buffer[2]=REG_ROBOT_REL_SOC_A;
 				break;
 			case 36:	//36-REG_ROBOT_REL_SOC_B
 				XbeeTest_UART_Buffer[1]=REG_ROBOT_REL_SOC_B>>8;
				XbeeTest_UART_Buffer[2]=REG_ROBOT_REL_SOC_B;
 				break;
 			case 38:	//38-REG_MOTOR_CHARGER_STATE
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_CHARGER_STATE>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_CHARGER_STATE;
 				break;
 			case 40:	//40-BuildNO
 				XbeeTest_UART_Buffer[1]=BuildNO>>8;
				XbeeTest_UART_Buffer[2]=BuildNO;
 				break;
 			case 42:	//42-REG_PWR_A_CURRENT
 				XbeeTest_UART_Buffer[1]=REG_PWR_A_CURRENT>>8;
				XbeeTest_UART_Buffer[2]=REG_PWR_A_CURRENT;
 				break;
 			case 44:	//44-REG_PWR_B_CURRENT
 				XbeeTest_UART_Buffer[1]=REG_PWR_B_CURRENT>>8;
				XbeeTest_UART_Buffer[2]=REG_PWR_B_CURRENT;
 				break;
 			case 46:	//46-REG_MOTOR_FLIPPER_ANGLE
 				XbeeTest_UART_Buffer[1]=REG_MOTOR_FLIPPER_ANGLE>>8;
				XbeeTest_UART_Buffer[2]=REG_MOTOR_FLIPPER_ANGLE;
 				break;
 			case 48:	//46-REG_MOTOR_FLIPPER_ANGLE
 				XbeeTest_UART_Buffer[1]=Xbee_SIDE_FAN_SPEED>>8;
				XbeeTest_UART_Buffer[2]=Xbee_SIDE_FAN_SPEED;
 				break;
 			case 50:	//46-REG_MOTOR_FLIPPER_ANGLE
 				XbeeTest_UART_Buffer[1]=Xbee_Low_Speed_mode>>8;
				XbeeTest_UART_Buffer[2]=Xbee_Low_Speed_mode;
 				break;
 		}
		//add checksum of the package
		XbeeTest_UART_Buffer[3]=255-(XbeeTest_UART_Buffer[0]+XbeeTest_UART_Buffer[1]+XbeeTest_UART_Buffer[2])%255;
		// clear incoming command
		Xbee_Incoming_Cmd[0]=0;
		Xbee_Incoming_Cmd[1]=0;
 	}
	


// 	if((Xbee_MOTOR_VELOCITY[0]||Xbee_MOTOR_VELOCITY[1]||Xbee_MOTOR_VELOCITY[2])!=0)
// 	{
// 		XbeeTest_UART_DataNO++;//index++, if transmit reg is full, skip this
// 		if(XbeeTest_UART_DataNO==201)
// 		{
// 			XbeeTest_UART_DataNO=0;
// 		}
// 	}
 	#endif
}

void  Motor_U1TXInterrupt(void)
{


 	//clear the flag
 	IFS0bits.U1TXIF=0;
 	#ifdef XbeeTest
 	//transmit data
 	if(XbeeTest_UART_BufferPointer<XbeeTest_UART_Buffer_Length)
 	{
 	 	#ifdef XbeeTest_TX_Enable
			U1TXREG=XbeeTest_UART_Buffer[XbeeTest_UART_BufferPointer];
			XbeeTest_UART_BufferPointer++;
		#endif
 	}else
 	{
 		XbeeTest_UART_BufferPointer=0;
 	}
 	#endif
}

void Motor_U1RXInterrupt(void)
{
 	//clear the flag
 	IFS0bits.U1RXIF=0;
//XbeeTest code only
	#ifdef XbeeTest
	int i=0;
	int SumBytes=0;
 	XbeeTest_Temp=U1RXREG;
  	if(XbeeTest_State==XbeeTest_StateIdle)
 	{
		if(XbeeTest_Temp==Xbee_StartBit)
		{
			XbeeTest_State=XbeeTest_StateProcessing;
		}
 	}
 	else if(XbeeTest_State==XbeeTest_StateProcessing)
 	{
 		XbeeTest_Buffer[XbeeTest_BufferArrayPointer]=XbeeTest_Temp;
 		XbeeTest_BufferArrayPointer++;
 		if(XbeeTest_BufferArrayPointer>=XbeeTest_BufferLength)//end of the package
 		{
			//if all bytes add up together equals 255, it is a good pack, then process
			SumBytes=0;
			for (i=0;i<XbeeTest_BufferLength;i++)
			{
				//printf("%d:%d  ",i,XbeeTest_Buffer[i]);
				SumBytes+=XbeeTest_Buffer[i];
			}
			//printf("Sum:%d\n",SumBytes);
			if(SumBytes%255==0)
			{
				//input data range 0~250, 125 is stop, 0 is backwards full speed, 250 is forward full speed
 				//for Marge, the right and left is flipped, so left and right need to be flipped 	
		 		Xbee_MOTOR_VELOCITY[0]=(XbeeTest_Buffer[0]*8-1000); 
 				Xbee_MOTOR_VELOCITY[1]=(XbeeTest_Buffer[1]*8-1000);
				Xbee_MOTOR_VELOCITY[2]=(XbeeTest_Buffer[2]*8-1000);
				Xbee_Incoming_Cmd[0]=XbeeTest_Buffer[3];
				Xbee_Incoming_Cmd[1]=XbeeTest_Buffer[4];
				//see if this is a fan cmd
				switch (Xbee_Incoming_Cmd[0])
 				{ 
 		 			case P1_Fan_Command: //new fan command coming in
 		 				Xbee_SIDE_FAN_SPEED=Xbee_Incoming_Cmd[1];
						Xbee_SIDE_FAN_NEW=1;
						//Enable fan speed timer
						Xbee_FanSpeedTimerEnabled=True;
						Xbee_FanSpeedTimerCount=0;
						//printf("fan data received!");
 						break;
					case P1_Low_Speed_Set:
						Xbee_Low_Speed_mode=Xbee_Incoming_Cmd[1];
						break;
					case P1_Calibration_Flipper:
						if(Xbee_Incoming_Cmd[1]==P1_Calibration_Flipper)
						{
							Xbee_Calibration=1;
						}
						break;
					case P1_Restart:
			            asm volatile("RESET");
            			break;
				}
				//REG_MOTOR_VELOCITY.left=300; 
 				//REG_MOTOR_VELOCITY.right=800;
				//REG_MOTOR_VELOCITY.flipper=400;
				//printf("Motor Speed Set!!\n");
				//printf("Receive New Package! Xbee_gNewData=%d",Xbee_gNewData);
				//printf("L:%d,R:%d,F:%d/n",Xbee_MOTOR_VELOCITY[0],Xbee_MOTOR_VELOCITY[1],Xbee_MOTOR_VELOCITY[2]);
				Xbee_gNewData = !Xbee_gNewData;
			}
			//clear all the local buffer
			XbeeTest_BufferArrayPointer=0;
			XbeeTest_State=XbeeTest_StateIdle;
 		}
 	}		
	#endif
//XbeeTest code ends 	
}


void initialize_i2c2_registers(void)
{
	REG_MOTOR_TEMP.left = 255;
	REG_MOTOR_TEMP.right = 255;
	REG_MOTOR_TEMP.board = 255;
	REG_ROBOT_REL_SOC_A = 255;


}

void initialize_i2c3_registers(void)
{

	REG_ROBOT_REL_SOC_B = 255;

}

static unsigned int return_calibrated_pot_angle(unsigned int pot_1_value, unsigned int pot_2_value)
{
  unsigned int combined_pot_angle = 0;
  int calibrated_pot_angle = 0;

  combined_pot_angle = return_combined_pot_angle( pot_1_value, pot_2_value);

  //special case -- invalid reading
  if(combined_pot_angle == 10000)
    return 10000;
  else if(combined_pot_angle == 0xffff)
    return 0xffff;

  //if calibration didn't work right, return angle with no offset
  if(flipper_angle_offset == 0xffff)
    return combined_pot_angle;

  calibrated_pot_angle = combined_pot_angle - flipper_angle_offset;

  if(calibrated_pot_angle < 0)
  {
    calibrated_pot_angle+=360;
  }
  else if(calibrated_pot_angle >= 360)
  {
    calibrated_pot_angle-=360;
  }
  
  return calibrated_pot_angle;

}  

static unsigned int return_combined_pot_angle(unsigned int pot_1_value, unsigned int pot_2_value)
{
  //unsigned int pot_1_value, pot_2_value = 0;
  int combined_pot_angle = 0;
  int pot_angle_1 = 0;
  int pot_angle_2 = 0;
  int temp1 = 0;
  int temp2 = 0;
  float scale_factor = 0;
  int temp_pot1_value = 0;
  
  //correct for pot 2 turning the opposite direction
  pot_2_value = 1023-pot_2_value;

  //if both pot values are invalid
  if( ((pot_1_value < LOW_POT_THRESHOLD) || (pot_1_value > HIGH_POT_THRESHOLD)) && 
      ((pot_2_value < LOW_POT_THRESHOLD) || (pot_2_value > HIGH_POT_THRESHOLD) ))
  {
    return 0xffff;
  }
  //if pot 1 is out of linear range
  else if( (pot_1_value < LOW_POT_THRESHOLD) || (pot_1_value > HIGH_POT_THRESHOLD) )
  {
    //333.3 degrees, 1023 total counts, 333.3/1023 = .326
    combined_pot_angle = pot_2_value*.326+13.35;
  }
  //if pot 2 is out of linear range
  else if( (pot_2_value < LOW_POT_THRESHOLD) || (pot_2_value > HIGH_POT_THRESHOLD) )
  {
    //333.3 degrees, 1023 total counts, 333.3/1023 = .326
    //13.35 degrees + 45 degrees = 58.35 degrees
    combined_pot_angle = (int)pot_1_value*.326+13.35+FLIPPER_POT_OFFSET;

  }
  //if both pot 1 and pot 2 values are valid
  else
  {

    //figure out which one is closest to the end of range
    temp1 = pot_1_value - 512;
    temp2 = pot_2_value - 512;

    //offset, so that both pot values should be the same
    //FLIPPER_POT_OFFSET/333.33*1023 = 168.8 for 55 degrees
    temp_pot1_value = pot_1_value-168.8;

    pot_angle_1 = temp_pot1_value*.326+13.35;
    pot_angle_2 = pot_2_value*.326+13.35;

    if(pot_angle_1 < 0) pot_angle_1+=360;
    if(pot_angle_2 < 0) pot_angle_2+=360;
  
    //if pot1 is closer to the end of range
    if(abs(temp1) > abs(temp2) )
    {
      scale_factor = ( 512-abs(temp1) )/ 512.0;
      //combined_pot_value = (pot_1_value*scale_factor + pot_2_value*(1-scale_factor));
      combined_pot_angle = pot_angle_1*scale_factor + pot_angle_2*(1-scale_factor);

    }
    //if pot2 is closer to the end of range
    else
    {

      scale_factor = (512-abs(temp2) )/ 512.0;
      //combined_pot_value = (pot_2_value*scale_factor + pot_1_value*(1-scale_factor));
      combined_pot_angle = pot_angle_2*scale_factor + pot_angle_1*(1-scale_factor);

    }

    //333.3 degrees, 1023 total counts, 333.3/1023 = .326
    //combined_pot_angle = combined_pot_value*.326+13.35;

  }

    if(combined_pot_angle > 360)
    {
      combined_pot_angle-=360;
    }
    else if(combined_pot_angle < 0)
    {
      combined_pot_angle  += 360;
    }


  return (unsigned int)combined_pot_angle;


}

//read stored values from flash memory
static void read_stored_angle_offset(void)
{
  unsigned char angle_data[3];
  unsigned int i;
  DataEEInit();
  Nop();
  for(i=0;i<3;i++)
  {
    angle_data[i] = DataEERead(i);
    Nop();
  }

  //only use stored values if we have stored the calibrated values before.
  //we store 0xaa in position 8 so we know that the calibration has taken place.
  if(angle_data[2] == 0xaa)
  {
    flipper_angle_offset = angle_data[0]*256+angle_data[1];
  }

}

void calibrate_flipper_angle_sensor(void)
{
  flipper_angle_offset = return_combined_pot_angle(REG_FLIPPER_FB_POSITION.pot1,REG_FLIPPER_FB_POSITION.pot2);

  //set motor speeds to 0
	PWM1Duty(0);
	PWM2Duty(0);
	PWM3Duty(0);

  DataEEInit();
  Nop();
  
  DataEEWrite( (flipper_angle_offset>>8),0);
  Nop();
  DataEEWrite( (flipper_angle_offset&0xff),1);
  Nop();  

  //write 0xaa to index 2, so we can tell if this robot has been calibrated yet
  DataEEWrite(0xaa,2);
  Nop();

  //don't do anything again ever
  while(1)
  {
    ClrWdt();

  }
}
void turn_on_power_bus_immediate(){
	Cell_Ctrl(Cell_A,Cell_ON);
	Cell_Ctrl(Cell_B,Cell_ON);
}	
void turn_on_power_bus_new_method(void)
{
  unsigned int i = 0;
  unsigned int j = 0;
  unsigned int k = 2000;
  unsigned int k0 = 2000;

  for(i=0;i<300;i++)
  {

  		Cell_Ctrl(Cell_A,Cell_ON);
      Cell_Ctrl(Cell_B,Cell_ON);
      for(j=0;j<k;j++) Nop();
  		Cell_Ctrl(Cell_A,Cell_OFF);
      Cell_Ctrl(Cell_B,Cell_OFF);

      block_ms(10);

      k=k0+i*i/4;
      //k+=10;

      ClrWdt();
  
  }

  		Cell_Ctrl(Cell_A,Cell_ON);
      Cell_Ctrl(Cell_B,Cell_ON);


}

void turn_on_power_bus_old_method(void)
{

  unsigned int i;

	for(i=0;i<20;i++)
	{
		Cell_Ctrl(Cell_A,Cell_ON);
		Cell_Ctrl(Cell_B,Cell_ON);
		block_ms(10);
		Cell_Ctrl(Cell_A,Cell_OFF);
		Cell_Ctrl(Cell_B,Cell_OFF);	
		block_ms(40);
    ClrWdt();

	}

		Cell_Ctrl(Cell_A,Cell_ON);
		Cell_Ctrl(Cell_B,Cell_ON);

}

void turn_on_power_bus_hybrid_method(void)
{

  unsigned int i;
  unsigned int j;
  //k=20,000 is about 15ms
  unsigned int k = 2000;
  unsigned int k0 = 2000;
  unsigned int l = 0;

	for(i=0;i<200;i++)
	{

    for(l=0;l<3;l++)
    {
  		Cell_Ctrl(Cell_A,Cell_ON);
      Cell_Ctrl(Cell_B,Cell_ON);
      for(j=0;j<k;j++) Nop();
  		Cell_Ctrl(Cell_A,Cell_OFF);
      Cell_Ctrl(Cell_B,Cell_OFF);
      break;
      if(i>20) break;
  
      ClrWdt();
      block_ms(10);
    }



    ClrWdt();
    block_ms(40);
    ClrWdt();
    //k+=10;
    k = k0+i*i/4;
    /*if(i<10000)
    {
      k = k0+(i*i)/4;
    }
    else
    {
      k+=50;
    } */
    //k = 3000;
    //k+=50;
    if(k > 20000) k = 20000;
	}

 	Cell_Ctrl(Cell_A,Cell_ON);
 	Cell_Ctrl(Cell_B,Cell_ON);

}

void handle_power_bus(void)
{

  unsigned char battery_data1[20], battery_data2[20];
  unsigned char old_battery[7] = {'B','B','-','2','5','9','0'};
  unsigned char new_battery[9] = {'B','T','-','7','0','7','9','1','B'};
  unsigned char BT70791_CK[9] = {'B','T','-','7','0','7','9','1','C'};
  unsigned char custom_matthews_battery[7] = {'R','O','B','O','T','E','X'};
  unsigned int j;

  //enable outputs for power bus
  CELL_A_MOS_EN(1);
  CELL_B_MOS_EN(1);
  
  //initialize i2c buses
  I2C2Ini();
  I2C3Ini();
  
    // if the power bus is already active (like the bootloader did it)
    // then nothing to do here.
    if (Cell_A_MOS && Cell_B_MOS) {
        RCON = 0;
        return;
    }

    // _POR = "we are powering on from a black or brownout"
    // _EXTR = "our reset pin was hit"
    // If the system is "warm", we can just switch the power bus back on.
    if (!_POR && !_EXTR) {
        turn_on_power_bus_immediate();
        RCON = 0;
        return;
    }

  for(j=0;j<3;j++)
  {

    //Read "Device Name" from battery
    readI2C2_Block(0x0b, 0x21, 10, battery_data1); 
    readI2C3_Block(0x0b, 0x21, 10, battery_data2); 
  
    //If we're using the old battery (BB-2590)
    if( check_string_match(old_battery,battery_data1,7) || check_string_match(old_battery,battery_data2,7))
    {
      send_debug_uart_string("BATTERY:  BB-2590\r\n",19);
      block_ms(10);
      turn_on_power_bus_old_method();      
      return;
    }
  
  
    //If we're using the new battery (BT-70791B)
    if(check_string_match(new_battery,battery_data1,9) || check_string_match(new_battery,battery_data2,9))
    {
      send_debug_uart_string("BATTERY:  BT-70791B\r\n",21);
      block_ms(10);
      turn_on_power_bus_new_method();
      return;
    }
	
    //If we're using Bren-Tronics BT-70791C
    if(check_string_match(BT70791_CK,battery_data1,9) || check_string_match(BT70791_CK,battery_data2,9))
    {
      send_debug_uart_string("BATTERY:  BT-70791C\r\n",21);
      block_ms(10);
      turn_on_power_bus_new_method();
      return;
    }

  //if we're using the low lithium custom Matthew's battery
    if(check_string_match(custom_matthews_battery,battery_data1,7) || check_string_match(custom_matthews_battery,battery_data2,7))
    {
      send_debug_uart_string("BATTERY:  ROBOTEX\r\n",19);
      block_ms(10);
      turn_on_power_bus_old_method();
      return;
    }

    ClrWdt();
    block_ms(20);
    ClrWdt();
  }  

    //if we're using an unknown battery
    send_debug_uart_string("UNKNOWN BATTERY\r\n",17);
    block_ms(10);
    send_debug_uart_string((char *)battery_data1,20);
    block_ms(10);

    turn_on_power_bus_hybrid_method();


  Nop();
  Nop();

}

int check_string_match(unsigned char *string1, unsigned char* string2, unsigned char length)
{
  unsigned int i;
  int string_match = 1;
  for(i=0;i<length;i++)
  {
    if(string1[i] != string2[i]) string_match = 0;
  }

  return string_match;


}

//When robot is on the charger, only leave one side of the power bus on at a time.
//This is so that one side of a battery doesn't charge the other side through the power bus.
static void alternate_power_bus(void)
{
  static unsigned int alternate_counter = 0;
  static unsigned int toggle_state = 0;


  if(REG_MOTOR_CHARGER_STATE == 0xdada)
  {

    //if we're on the dock, immediately turn off one side of the power bus
    if(toggle_state == 0)
    {
      toggle_state = 1;
   	  Cell_Ctrl(Cell_B,Cell_ON);
      Cell_Ctrl(Cell_A,Cell_OFF);
    }

    alternate_counter++;
    
    if(alternate_counter > 10000)
    {
        alternate_counter = 0;
  
        //turn off side B, turn on side A
        if(toggle_state == 1)
        {
          toggle_state = 2;
       	  Cell_Ctrl(Cell_A,Cell_ON);
   			  Cell_Ctrl(Cell_B,Cell_OFF);
        }
        //turn on side B, turn off side A
        else if(toggle_state == 2)
        {
          toggle_state = 1;
   			  Cell_Ctrl(Cell_B,Cell_ON);
          Cell_Ctrl(Cell_A,Cell_OFF);
        }
        else
        {
          toggle_state = 0;
        }
    }
  }
  //robot has left the charger -- turn both cells on
  else
  {
    Cell_Ctrl(Cell_A,Cell_ON);
    Cell_Ctrl(Cell_B,Cell_ON);
    toggle_state = 0;

  }


}


