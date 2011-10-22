//////////////////////////////////////////////////////////////////////
//Program Description: This program is for motor controller board for 2011 Robot
// -it reads servo input signal/USB messages and controls H-bridge for brushed DC motor
//Author: 	Eric Zheng
//Starting Date:		October 15th, 2010
//Last modify:			Jan 20th, 2011
//Version info:	2.10
//Pin config:
/*
	Pin Name							Pin #	Description			Funtion	Input/Output	Digital/Analog	Comment
1	RP20/PMRD/CN14/RD5					53		L_Encoder_A			IC		Input			Digital			IC1
2	RP25/PMWR/CN13/RD4					52		L_Encoder_B			IC		Input			Digital			IC2
3	AN14/CTPLS/RP14/PMA1/CN32/RB14		29		M1_AHI				IO		Output			Digital	
4	RP16/USBID/CN71/RF3					33		M1_ALO				PWM		Output			Digital			OC1
5	RTCC/DMLN/RP2/CN53/RD8				42		M1_BHI				IO		Output			Digital	
6	AN15/RP29/REFO/PMA0/CN12/RB15		30		M1_BLO				PWM		Output			Digital			OC2
7	VCPCON/RP24/CN50/RD1				49		M1_Fault			IO		Input			Digital	
8	TMS/CVREF/AN10/PMA13/CN28/RB10		23		M1_LeftBackEMF		A/D		Input			Analog	
9	TDO/AN11/PMA12/CN29/RB11			24		M1_RightBackEMF		A/D		Input			Analog	
10	TCK/AN12/PMA11/CTED2/CN30/RB12		27		M1_Current			A/D		Input			Analog	
11	RP22/PMBE/CN52/RD3					51		R_Encoder_A			IC		Input			Digital			IC3
12	DPH/RP23/CN51/RD2					50		R_Encoder_B			IC		Input			Digital			IC4
13	C1IND/RP21/PMA5/CN8/RG6				4		M2_AHI				IO		Output			Digital	
14	C1INC/RP26/PMA4/CN9/RG7				5		M2_ALO				PWM		Output			Digital			OC3
15	C2IND/RP19/PMA3/CN10/RG8			6		M2_BHI				IO		Output			Digital	
16	RP27/PMA2/C2INC/CN11/RG9			8		M2_BLO				PWM		Output			Digital			OC4
17	PMD5/CN63/RE5						1		M2_Fault			IO		Input			Digital	
18	PGEC3/AN5/C1INA/VBUSON/RP18/CN7/RB5	11		M2_LeftBackEMF		A/D		Input		
19	PGED3/AN4/C1INB/USBOEN/RP28/CN6/RB4	12		M2_RightBackEMF		A/D		Input		
20	AN3/C2INA/VPIO/CN5/RB3				13		M2_Current			A/D		Input		
21	SOSCI/C3IND/CN1/RC13				47		M3_AHI				IO		Output			Digital	
22	DMH/RP11/INT0/CN49/RD0				46		M3_ALO				PWM		Output			Digital			OC5
23	SOSCO/T1CK/C3INC/RPI37/CN0/RC14		48		M3_BHI				IO		Output			Digital	
24	RP12/PMCS1/CN56/RD11				45		M3_BLO				PWM		Output			Digital			OC6
25	AN8/RP8/CN26/RB8					21		M3_Current			A/D		Input			Analog	
26	AN9/RP9/PMA7/CN27/RB9				22		M3_POSFB_1			A/D		Input			Analog	
27	TDI/AN13/PMA10/CTED1/CN31/RB13		28		M3_POSFB_2			A/D		Input			Analog	
28	C3INA/CN16/RD7						55		Fan1_Fail			IO		Input			Digital	
29	C3INB/CN15/RD6						54		Fan2_Fail			IO		Input			Digital	
30	SCL3/PMD6/CN64/RE6					2		FAN_I2C_SCL			I2C		Output			Digital	
31	SDA3/PMD7/CN65/RE7					3		FAN_I2C_SDA			I2C		Output			Digital	
32	VCMPST2/CN69/RF1					59		Cell_A_MOS			IO		Output			Digital	
33	VBUSST/VCMPST1/CN68/RF0				58		Cell_B_MOS			IO		Output			Digital	
34	AN2/C2INB/VMIO/RP13/CN4/RB2			14		Total_Cell_Current	AD		Input			Analog	
35	PGEC1/AN1/VREF-/RP1/CN3/RB1			15		V_Cell_A			AD		Input			Analog	
36	PGED1/AN0/VREF+/RP0/PMA6/CN2/RB0	16		V_Cell_B			AD		Input			Analog	
37	SDA2/RP10/PMA9/CN17/RF4				31		SMBUS_A_DA			I2C		Output			Digital	
38	SCL2/RP17/PMA8/CN18/RF5				32		SMBUS_A_CL			I2C		Output			Digital	
39	DPLN/SDA1/RP4/CN54/RD9				43		SMBUS_B_DA			I2C		Output			Digital	
40	SCL1/RP3/PMCS2/CN55/RD10			44		SMBUS_B_CL			I2C		Output			Digital	

							

PMD0/CN58/RE0 	 							Chip ID 			I/O 		Input			Digital
PMD1/CN59/RE1	 							Chip ID 			I/O 		Input			Digital
PMD2/CN60/RE2		 						Chip ID 			I/O 		Input			Digital
PMD3/CN61/RE3			 					Chip ID 			I/O 		Input			Digital
PMD4/CN62/RE4				 				Chip ID 			I/O 		Input			Digital

*/

/*
Timer Distribution:

Timer1: software running timer
Timer2: Output Compare-PWM
Timer3: A/D Triger
Timer4:	Input Capture
Timer5:	SurgeProtection
*/


/* USB communication:
Commend from Host to Device:
1-set speed for three motors
	ID: CMD_SET_ROBOT_MOTOR_VELOCITY
 	 	
2-set speed for two fans
 	ID: CMD_SET_ROBOT_FAN_VELOCITY
3-set control mode
 	ID: CMD_SET_ROBOT_CTRLMODE

Response from Device to Host:
1- current of three motors
 	ID: RSP_ROBOT_MOTOR_CURRENT
2- speed of three motors
 	ID: RSP_ROBOT_MOTOR_SPEED
3- encoder count of two motors
 	ID: RSP_ROBOT_MOTOR_ENCODERCOUNT
4- data of battery through SMBUS
 	ID: RSP_ROBOT_BATTERY




*/
//////////////////////////////////////////////////////////////////////


#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "i2c.h"
#include "interrupt_switch.h"

#define XbeeTest
#define BATProtectionON

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
int USBTimeOutTimerEnabled=False;
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
int I2C2TimerExpired=False;
int I2C2TimerCount=0;
int I2C3TimerEnabled=True;
int I2C3TimerExpired=False;
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

unsigned long Encoder_Interrupt_Counter[2] = {0,0};

long EncoderFBInterval[3][SampleLength]={{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int DIR[3][SampleLength]={{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int EncoderFBIntervalPointer[3]={0,0,0};
long BackEMF[3][2][SampleLength]={{{0,0,0,0},{0,0,0,0}},{{0,0,0,0},{0,0,0,0}},{{0,0,0,0},{0,0,0,0}}};
int BackEMFPointer=0;
long MotorCurrentAD[3][SampleLength]={{0,0,0,0},{0,0,0,0},{0,0,0,0}};
int MotorCurrentADPointer=0;
long RealTimeCurrent[3]={0,0,0};
long Current4Control[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
int Current4ControlPointer[3]={0,0,0};
long ControlCurrent[3]={0,0,0};
long CurrentRPM[3]={0,0,0};
long RPM4Control[3][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
int RPM4ControlPointer[3]={0,0,0};
long ControlRPM[3]={0,0,0};
long TotalCurrent;
long BackEMFCOE[3][SampleLength]={{2932,2932,2932,2932},{2932,2932,2932,2932},{2932,2932,2932,2932}};
long BackEMFCOEF[3]={2932,2932,2932};
int BackEMFCOEPointer=0;
int EncoderICClock=10000;
long EnCount[3]={0,0,0};
//int Robot_Motor_TargetSpeedUSB[3];
int16_t Robot_Motor_TargetSpeedUSB[3]={0,0,0};
int NEW_ROBOT_MOTOR_SPEED_RECEIVED=False;
int Timer3Count=0;
int BackEMFSampleEnabled=False;
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
//float SpeedCtrlKp[4][3]={{0.0002,0.0002,0.0002},{0.09,0.09,0.09},{0.09,0.09,0.09},{0.09,0.09,0.09}};//SpeedCtrlKp[i][j],i- control mode, j-LMotor, Right Motor, Flipper
float SpeedCtrlKp[4][3]={{0.2,0.2,0.2},{0.03,0.03,0.03},{0.09,0.09,0.09},{0.09,0.09,0.09}};//SpeedCtrlKp[i][j],i- control mode, j-LMotor, Right Motor, Flipper
float SpeedCtrlKi[4][3]={{0.01,0.01,0.01},{0.001,0.001,0.001},{0.2,0.2,0.2},{0.2,0.2,0.2}};
float SpeedCtrlKd[4][3]={{0.000,0.000,0.000},{0.001,0.001,0.001},{0.0,0.0,0.0},{0.0,0.0,0.0}};
float CurrentCtrlKp[4][3]={{1.5,1.5,1.5},{1.0,1.0,1.0},{1.5,1.5,1.5},{1.5,1.5,1.5}};
float CurrentCtrlKi[4][3]={{0.5,0.5,0.5},{0.01,0.01,0.01},{1.5,1.5,1.5},{1.5,1.5,1.5}};
float CurrentCtrlKd[4][3]={{0.00,0.00,0.00},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};
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
long BackEmfRPM[3];
long BackEmfTemp3[3];
long BackEmfTemp4[3];
long Debugging_Dutycycle[3];
long MotorTargetRPM[3];
long Debugging_MotorTempError[3];
int Timer5Count=0;
int8_t I2C3DataMSOut[20];//I2C3DataMSOut[0]--Lock indicator, 0-unlocked 1-locked;I2C3DataMSOut[1]--length of this packet
int I2C1Channel=Available;
int I2C2Channel=Available;
int I2C3Channel=Available;
int I2C2XmitReset=False;
int I2C3XmitReset=False;

unsigned int adc_test_reg = 0;


#ifdef XbeeTest
 	#define XbeeTest_BufferLength 3
 	#define XbeeTest_StateIdle 0
 	#define XbeeTest_StateProcessing 1
	int XbeeTest_State=XbeeTest_StateIdle;
 	int16_t XbeeTest_Buffer[3];
 	int XbeeTest_BufferArrayPointer=0;
 	int XbeeTest_Temp;
 	uint16_t XbeeTest_Temp_u16;
 	#define XbeeTest_UART_Buffer_Length 4 //no more than 5 bytes based on 57600 baud and 1ms system loop
 	uint8_t XbeeTest_UART_Buffer[XbeeTest_UART_Buffer_Length];
 	uint8_t XbeeTest_UART_BufferPointer=0;
 	uint8_t XbeeTest_UART_DataNO=0;
#endif






void bringup_board(void)
{

/*	unsigned int battery1_voltage = 0;
	unsigned int battery2_voltage = 0;
	//unsigned char BATTERY_ADDRESS = 0x16;
	unsigned char BATTERY_ADDRESS = 0x0b;
	battery1_voltage = 77;
	unsigned int battery1_rel_SOC = 0;
	unsigned int battery2_rel_SOC = 0;
	

	OpenI2C2(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, 0xff);

	IdleI2C2();

	OpenI2C3(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, 0xff);

	IdleI2C3();

	block_ms(1000);
	//set fan configuration
	writeI2C2Reg( FAN_CONTROLLER_ADDRESS,0x02,0b00011010);
	block_ms(50);

	//make thermistor 1 control fan 2, and vice versa
	writeI2C2Reg( FAN_CONTROLLER_ADDRESS,0x11,0b00011000);
	block_ms(50);

	//set fan start duty cycle -> 120/240 = 50%
	writeI2C2Reg( FAN_CONTROLLER_ADDRESS,0x07,120);
	block_ms(50);

	//fan turns on at 40C
	writeI2C2Reg( FAN_CONTROLLER_ADDRESS,0x10,40);
	block_ms(50);
	block_ms(200);

	battery1_voltage = readI2C2_Word(BATTERY_ADDRESS, 0x09);
	battery2_voltage = readI2C3_Word(BATTERY_ADDRESS, 0x09);

	battery1_rel_SOC = readI2C2_Word(BATTERY_ADDRESS, 0x0d);
	battery2_rel_SOC = readI2C3_Word(BATTERY_ADDRESS, 0x0d);


	while(1);*/
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

	unsigned int i;
	//wait for 10s.  This is so that we can differentiate between
	//the battery resetting (and restarting this code), and the 
	//current protection kicking in.
	block_ms(10000);

	//initialize all modules
	MC_Ini();

 	//Call ProtectHB
	ProtectHB(LMotor);
	ProtectHB(RMotor);
	ProtectHB(Flipper);
	//Start SwitchDirectionTimer
	//******************************//
	//test code part
	//printf("ABCDEFG");
	//TestIO();
	//TestPWM();
	//TestIC1();
	//while(1);
	//end test
	//******************************//


	for(i=0;i<20;i++)
	{
		Cell_Ctrl(Cell_A,Cell_ON);
		Cell_Ctrl(Cell_B,Cell_ON);
		block_ms(10);
		Cell_Ctrl(Cell_A,Cell_OFF);
		Cell_Ctrl(Cell_B,Cell_OFF);	
		block_ms(40);

	}
 //	Cell_Ctrl(Cell_A,Cell_ON);
 //	Cell_Ctrl(Cell_B,Cell_ON);


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

 	static long ENRPM=0;
 	static int ENDIR=0;

 	static long BemfRPM=0;
 	static int BemfDIR=0;
 	static long ltemp1=0;
 	static long ltemp2=0;
 	static long ltemp3=0;
 	static long ltemp4=0;
 	static long temp=0;
 	static long ltemp5=0;//save the Bemf AD value
 	static long ltemp6;
 	static int i,j;
 	static long LastEnCount[3]={0,0,0};

	ENRPM=0;
 	ENDIR=0;
	BemfRPM=0;
 	BemfDIR=0;
 	ltemp1=0;
 	ltemp2=0;
 	ltemp3=0;
 	ltemp4=0;
 	temp=0;
 	ltemp5=0;//save the Bemf AD value
 	ltemp6;
 	i=0;
 	j=0;
 	
//T1

 	//RPM Calculation
 	//encoder RPM Calculation
 	for(j=0;j<SampleLength;j++)
 	{
 	 	ltemp1+=EncoderFBInterval[Channel][j];
 		ltemp2+=DIR[Channel][j];
 		ltemp3+=BackEMF[Channel][ChannelA][j];
 		ltemp4+=BackEMF[Channel][ChannelB][j];
 	}


 	if(ltemp1>0)
 	{
 	 	ENRPM=24000000/ltemp1;
 	}
 	else
 	{
 		ENRPM=0;
 	}
	
	if(Channel == 0)
	{
		if(M1_DIRO)
			ENDIR = -1;
		else
			ENDIR = 1;
	}
	else if(Channel == 1)
	{

		if(M2_DIRO)
			ENDIR = 1;
		else
			ENDIR = -1;

	}

	CurrentRPM[Channel]=ENRPM*ENDIR;


	//if the input capture interrupt hasn't been called, the motors are not moving
 	if(Encoder_Interrupt_Counter[Channel] == LastEnCount[Channel])
 	{
 	 	CurrentRPM[Channel]=0;		
 	}

	LastEnCount[Channel] = Encoder_Interrupt_Counter[Channel];

//T3
/*
 	//printf("1- %d\n",temp1);
 	if(ltemp2>=0)
 	{
 		ENDIR=1;
 	}else
 	{
 		ENDIR=-1;
 	}
 	ltemp1>>=ShiftBits;// divided by 4
 	//printf("2- %d\n",temp1);
//T2

 	if(ltemp1>0)
 	{
 	 	ENRPM=24000000/ltemp1;
 	}
 	else
 	{
 		ENRPM=0;
 	}
 	BackEmfTemp3[Channel]=ltemp3;
 	BackEmfTemp4[Channel]=ltemp4;
 	if(ltemp3>ltemp4)
 	{
 		BemfDIR=1;
 		ltemp3>>=ShiftBits;
 		BemfRPM=((ltemp3*BackEMFCOEF[Channel])>>7);
 		ltemp5=ltemp3;
 	}
 	else
 	{
 		BemfDIR=-1;
 		ltemp4>>=ShiftBits;
 		BemfRPM=((ltemp4*BackEMFCOEF[Channel]))>>7;
 		ltemp5=ltemp4;
 	}
 	BackEmfRPM[Channel]=BemfRPM; 	
 	//if RPM by Backemf < ThresholdRPM, use backemf as RPM
 	//if RPM by Backemf > ThresholdRPM, use encoder as RPM
	
 	if(BemfRPM<ThresholdRPM)
 	{
 		CurrentRPM[Channel]=BemfRPM*BemfDIR;	
 	}
 	else
 	{
 		CurrentRPM[Channel]=ENRPM*ENDIR;
 	}


 	if(EnCount[Channel]==LastEnCount[Channel])
 	//if(ltemp5<100 && EnCount[Channel]==LastEnCount[Channel])
 	{
 	 	CurrentRPM[Channel]=0;
 		//printf("%ld\n",ltemp5); 			
 	}
 	LastEnCount[Channel]=EnCount[Channel];
 	//CurrentRPM[Channel]=ENRPM*ENDIR;
 	//if RPM>2000, update the coefficient
 	RPM4Control[Channel][RPM4ControlPointer[Channel]]=CurrentRPM[Channel];
 	RPM4ControlPointer[Channel]++;
 	RPM4ControlPointer[Channel]&=7;
 	if(CurrentRPM[Channel]>=2000 || CurrentRPM[Channel]<-2000)
 	{
 	 	temp=0;
 		BackEMFCOE[Channel][BackEMFCOEPointer]=(ENRPM<<7)/ltemp5;
 		BackEMFCOEPointer++;
 		BackEMFCOEPointer&=(SampleLength-1);
 	 	for(i=0;i<SampleLength;i++)
 		{
 			temp+=BackEMFCOE[Channel][i];
 			//printf("%d,%ld\n",i,BackEMFCOE[Channel][i]);
 		}
 	 	ltemp6=temp>>ShiftBits;
 	 	if(ltemp6>2500 && ltemp6<=5500)
 		{
 	 		BackEMFCOEF[Channel]=ltemp6;
 		}
 	}*/

}

void Device_MotorController_Process()
{
 	int i,j;
 	long temp1,temp2;
 	I2C2Update();
	I2C3Update();
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
	}



	//check if any timers are expired
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
 	 	RPMTimerCount=0;
 	 	RPMTimerEnabled=False;
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
  		I2C2TimerExpired=True;
 		I2C2TimerCount=0;
 		I2C2XmitReset=True;
 	}
 	if(I2C3TimerCount>=I2C3Timer)
 	{
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

 	//if any of the timers expired, excute relative codes
 	//Control timer expired
 	for(i=0;i<=2;i++)
 	{
 	 	if(SpeedUpdateTimerExpired[i]==True)
 	 	{
 	 	 	UpdateSpeed(i,StateLevel01[i]);
 	 	 	SpeedUpdateTimerExpired[i]=False;
 	 	 	test();
 	 	}
 	}
//t3

 	if(RPMTimerExpired==True)
 	{
 	 	RPMTimerEnabled=True;
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
 		//I2C2Update();
 	}
 	if(I2C3TimerExpired==True)
 	{
 		//update data on I2C3, reset the I2C data accquiring sequence
 		//I2C3Update();
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
 		//update current for all three motors
 		REG_MOTOR_FB_CURRENT.left=ControlCurrent[LMotor];
 		REG_MOTOR_FB_CURRENT.right=ControlCurrent[RMotor];
 		REG_MOTOR_FB_CURRENT.flipper=ControlCurrent[Flipper];
 		//update the encodercount for two driving motors
 		REG_MOTOR_ENCODER_COUNT.left=Encoder_Interrupt_Counter[LMotor];
 		REG_MOTOR_ENCODER_COUNT.right=Encoder_Interrupt_Counter[RMotor];
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
 			temp1+=CellVoltageArray[Cell_A][i];
 			temp2+=CellVoltageArray[Cell_B][i];
 		}
 		REG_PWR_BAT_VOLTAGE.a=temp1>>ShiftBits;
 		REG_PWR_BAT_VOLTAGE.b=temp2>>ShiftBits;
 		REG_PWR_BAT_VOLTAGE.a=CellVoltageArray[Cell_A][0];
 		REG_PWR_BAT_VOLTAGE.b=temp2>>ShiftBits;


 		BATVolCheckingTimerExpired=False;
 		#ifdef BATProtectionON
 		if(REG_PWR_BAT_VOLTAGE.a<=BATVoltageLimit || REG_PWR_BAT_VOLTAGE.b<=BATVoltageLimit)//Battery voltage too low, turn off the power bus
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
 		}
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


//*********************************************//
//I2C library
int CheckI2C2Idle()
{

 	if(I2C2CONbits.SEN||I2C2CONbits.PEN||I2C2CONbits.RCEN||I2C2CONbits.ACKEN||I2C2STATbits.TRSTAT)
 	{
 		return False;
 	}else
 	{
 		return True;
 	}
}

int CheckI2C3Idle()
{

 	if(I2C3CONbits.SEN||I2C3CONbits.PEN||I2C3CONbits.RCEN||I2C3CONbits.ACKEN||I2C3STATbits.TRSTAT)
 	{
 		return False;
 	}else
 	{
 		return True;
 	}
}

//fan controller, TMP112, battery, EEPROM
void I2C2Update()
{

 	static int StepNumber=1; 
	static unsigned char a,b;
 	//static int TMPHigh,TMPLow;

 	if(I2C2XmitReset==True)
 	{
 		I2C2XmitReset=False;//clear the flag
 		StepNumber=1;//go to step 1, start from the beginning
 		//assume all the data is good
 		REG_MOTOR_TEMP_STATUS.left=1;
 		REG_MOTOR_TEMP_STATUS.right=1;
 		REG_MOTOR_TEMP_STATUS.board=1;
 	}
 	switch(StepNumber)
 	{
 	 	case 1://make sure the module is idle
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 2://wait until start is complete, then transmit data
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=FANCtrlAddressW;//transmit data
 			}
 			break;
 		case 3://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C2STATbits.IWCOL=0;
 			 	REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 4://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 5://check ACK from slave
 			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			StepNumber++;
 			break;
 		case 6://send data
 			I2C2TRN=0x00;//transmit data
 			StepNumber++;
 			break;
 		case 7://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
 			 	REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 8://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
 		case 9:
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=FANCtrlAddressR;//transmit data
 			}
 			break;
 		case 10:
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 11://wait until receive is completed
 			if(I2C2CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C2STATbits.I2COV = 0;
 				REG_MOTOR_TEMP.left=I2C2RCV;
 			}
 			break;
 		case 12://stop the module
 			I2C2CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
 			StepNumber++;
 			break;
 	 	case 13://make sure the module is idle
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 14://wait until start is complete, then transmit data
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=FANCtrlAddressW;//transmit data
 			}
 			break;
 		case 15://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
 			 	REG_MOTOR_TEMP_STATUS.right=0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 16://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 17://check ACK from slave
 			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				REG_MOTOR_TEMP_STATUS.right=0;
 			}
 			StepNumber++;
 			break;
 		case 18://send data
 			I2C2TRN=0x01;//transmit register address data
 			StepNumber++;
 			break;
 		case 19://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
 			 	REG_MOTOR_TEMP_STATUS.right=0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 20://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
 		case 21:
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=FANCtrlAddressR;//transmit data
 			}
 			break;
 		case 22:
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 23://wait until receive is completed
 			if(I2C2CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C2STATbits.I2COV = 0;
 				REG_MOTOR_TEMP.right=I2C2RCV;
 			}
 			break;
 		case 24://stop the module
 			I2C2CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
 			StepNumber++;
 			break;
	 	case 25://make sure the module is idle
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 26://wait until start is complete, then transmit data
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=BATTERY_ADDRESS<<1;//transmit data
 			}
 			break;
 		case 27://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
 			 	//REG_MOTOR_TEMP_STATUS.left=0;
				StepNumber = 0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 28://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 29://check ACK from slave
 			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				//REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			StepNumber++;
 			break;
 		case 30://send data
 			I2C2TRN=0x0d;//transmit data
 			StepNumber++;
 			break;
 		case 31://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C2STATbits.IWCOL = 0;
 			 	//REG_MOTOR_TEMP_STATUS.left==0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 32://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
 		case 33:
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=(BATTERY_ADDRESS<<1)+1;//transmit data
 			}
 			break;
 		case 34:
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 35://wait until receive is completed
 			if(I2C2CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C2STATbits.I2COV = 0;
 				a=I2C2RCV;
				I2C2CONbits.ACKDT = 0; //ACK
				I2C2CONbits.ACKEN = 1;
 			}
 			break;
		case 36:
		 	if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
		break;
		case 37:
		 	if(I2C2CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C2STATbits.I2COV = 0;
 				b=I2C2RCV;
				REG_ROBOT_REL_SOC_A = a  + (b<<8);
				I2C2CONbits.ACKDT = 1; //NACK
				I2C2CONbits.ACKEN = 1;
 			}
		break;		
 		case 38://stop the module
			I2C2CONbits.ACKDT = 0; //ACK
 			I2C2CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
 			StepNumber++;
 			break;
 		case 39://Make sure the module is idle
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				StepNumber=1;//cycle ends, go to the first step
 				I2C2TimerExpired=False;//reset the I2C2 update timer
 			}
 			break;
 	}

}


void I2C3Update(void)
{

 	static int StepNumber=1; 
	static unsigned int a,b;
 	//static int TMPHigh,TMPLow;

 	if(I2C3XmitReset==True)
 	{
 		I2C3XmitReset=False;//clear the flag
 		StepNumber=1;//go to step 1, start from the beginning
 		//assume all the data is good

		I2C3CONbits.SEN = 0;
		I2C3CONbits.PEN = 0;
		I2C3CONbits.RCEN = 0;
		I2C3CONbits.ACKEN = 0;
		I2C3STATbits.TRSTAT = 0;

 	}
 	switch(StepNumber)
 	{
	 	case 1://make sure the module is idle
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 2://wait until start is complete, then transmit data
 			if(I2C3CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C3TRN=BATTERY_ADDRESS<<1;//transmit data
 			}
 			break;
 		case 3://wait for transmit to complete
 			if(I2C3STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C3STATbits.IWCOL = 0;
 			 	//REG_MOTOR_TEMP_STATUS.left=0;
				StepNumber = 0;
 			}
 			if(I2C3STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 4://make sure the module is idle
 			if(CheckI2C3Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 5://check ACK from slave
 			if(I2C3STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				//REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			StepNumber++;
 			break;
 		case 6://send data
 			I2C3TRN=0x0d;//transmit data
 			StepNumber++;
 			break;
 		case 7://wait for transmit to complete
 			if(I2C3STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C3STATbits.IWCOL = 0;
				StepNumber = 0;
 			 	//REG_MOTOR_TEMP_STATUS.left==0;
 			}
 			if(I2C3STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 8://make sure the module is idle
 			if(CheckI2C3Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C3CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
		case 9:
			 if(I2C3CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C3TRN=(BATTERY_ADDRESS<<1)+1;//transmit data
 			}
 			break;
 		case 10:
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 11://wait until receive is completed
 			if(I2C3CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C3STATbits.I2COV = 0;
 				a=I2C3RCV;
				I2C3CONbits.ACKDT = 0; //ACK
				I2C3CONbits.ACKEN = 1;
 			}
 			break;
		case 12:
		 	if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.RCEN=1;//enable receive model
 			}
		break;
		case 13:
		 	if(I2C3CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C3STATbits.I2COV = 0;
 				b=I2C3RCV;
				REG_ROBOT_REL_SOC_B = a  + (b<<8);
				I2C3CONbits.ACKDT = 1; //NACK
				I2C3CONbits.ACKEN = 1;
 			}
		break;		
 		case 14://stop the module
		 	if(CheckI2C3Idle()==True)
 			{
	 			I2C3CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
	 			StepNumber++;
			}
 			break;
		case 15:
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				StepNumber=1;//cycle ends, go to the first step
 				I2C3TimerExpired=False;//reset the I2C2 update timer
 			}
		break;

	} 

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
 	long TargetCurrent;
 	float result;
 	long TempSpeedError;
 	long TempCurrentError;
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
 	}else
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
 	}
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

void USBInput()
{
  	int i;
	static int USB_New_Data_Received;
	//update local usb message variable
 	Robot_Motor_TargetSpeedUSB[0]=REG_MOTOR_VELOCITY.left; 
 	Robot_Motor_TargetSpeedUSB[1]=REG_MOTOR_VELOCITY.right;
	Robot_Motor_TargetSpeedUSB[2]=REG_MOTOR_VELOCITY.flipper;

	/*i=-500;
	//i=20;
 	Robot_Motor_TargetSpeedUSB[0]=i; 
 	Robot_Motor_TargetSpeedUSB[1]=i;
 	Robot_Motor_TargetSpeedUSB[2]=i;*/
	//USBTimeOutTimerCount=0;
	
 	gNewData=!gNewData;


	//long time no data, clear everything
 	if(USBTimeOutTimerExpired==True)
 	{
 		USBTimeOutTimerExpired=False;
 		USBTimeOutTimerEnabled=False;
 		USBTimeOutTimerCount=0;
 		REG_MOTOR_VELOCITY.left=0;
 		REG_MOTOR_VELOCITY.right=0;
 		REG_MOTOR_VELOCITY.flipper=0;
 		for(i=0;i<3;i++)
 		{
 			Robot_Motor_TargetSpeedUSB[i]=0;
			Event[i]=Stop;//Get the event
 			TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];
 			//ClearSpeedCtrlData(i);
 			//ClearCurrentCtrlData(i);
 		}
 	}
	// if there is new data comming in, update all the data
 	if(USB_New_Data_Received!=gNewData)
 	{
 		USB_New_Data_Received=gNewData;
 		USBTimeOutTimerCount=0;
 		USBTimeOutTimerEnabled=True;
 		USBTimeOutTimerExpired=False;

		for(i=0;i<3;i++)
		{	
			//Robot_Motor_TargetSpeedUSB[i]==0 ->Stop
			if(Robot_Motor_TargetSpeedUSB[i]==0)
			{
				Event[i]=Stop;//Get the event
 				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];
			}
			//-1024<Robot_Motor_TargetSpeedUSB[i]<0 ->Back
			else if(Robot_Motor_TargetSpeedUSB[i]>-1024 && Robot_Motor_TargetSpeedUSB[i]<0)
			{
				Event[i]=Back;//Get the event
				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];
			}		
			//0<Robot_Motor_TargetSpeedUSB[i]<1024 ->Go
			else if(Robot_Motor_TargetSpeedUSB[i]>0 && Robot_Motor_TargetSpeedUSB[i]<1024)
			{
				Event[i]=Go;//Get the event
				TargetParameter[i]=Robot_Motor_TargetSpeedUSB[i];//Save the speed
			}
  		}
 	}
}

int EventChecker()
{
 	USBInput();
// 	ServoInput();
}



//**********************************************


void PinRemap(void)
{
	// Unlock Registers
	//clear the bit 6 of OSCCONL to
	//unlock Pin Re-map
	asm volatile	("push	w1				\n"
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
					"pop	w1");

	// Configure Input Functions
	//function=pin
	// Assign IC1 To L_Encoder_A
	RPINR7bits.IC1R = M1_TACHO_RPn; 
	
	// Assign IC2 To L_Encoder_B
//	RPINR7bits.IC2R = M2_TACHO_RPn;

	// Assign IC3 To Encoder_R1A
	RPINR8bits.IC3R = M2_TACHO_RPn;
	
/*
	// Assign IC5 To LMotor_In
	RPINR9bits.IC5R = Servo1_Input_RPn;

	// Assign IC6 To RMotor_In
	RPINR9bits.IC6R = Servo2_Input_RPn;

	// Assign U1RX To U1RX, Uart receive channnel
	RPINR18bits.U1RXR = U1RX_RPn;
*/
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
	__builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to
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
 					//TRISFbits.TRISF1=0;//ground the pin, turn on the mosfet
 					//PORTFbits.RF1=0;
 					Cell_A_MOS=1;
 					break;
 				case Cell_OFF:
 					//TRISFbits.TRISF1=0;//pin high, turn off the mosfet
 					//PORTFbits.RF1=1;
 					Cell_A_MOS=0;
 					break;
 			}
 			break;
 		case Cell_B:
 			switch(state)
 			{
 				case Cell_ON:
 					//TRISFbits.TRISF0=0;//ground the pin, turn on the mosfet
 					//PORTFbits.RF0=0;
 					Cell_B_MOS=1;
 					break;
 				case Cell_OFF:
 					//TRISFbits.TRISF0=0;//pin high, turn off the mosfet
 					//PORTFbits.RF0=1;
 					Cell_B_MOS=0;
 					break;
 			}
 			break;
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
		

/*	AD1PCFGL|=0b1111000000001000;
	AD1PCFGL&=0b1111000011001111;
	
	//PORTB
	TRISB&=0b0011111111111111;	
	TRISB|=0b0011111100111000;
	
	//PORTC
	TRISC&=0b1001111111111111;	
	//TRISC|=0b0000000000000000;

	//PORTD
	TRISD&=0b1111010011111100;	
	TRISD|=0b0000010011111100;

	//PORTE
	TRISE&=0b1111111111011111;	
	//TRISE|=0b0000000000100000;

	//PORTF
	TRISF&=0b1111111111110111;	
	//TRISF|=0b0000000000000000;	

	//PORTG
	TRISG&=0b1111110000111111;	
	//TRISG|=0b0000000000000000;	
	*/

//Xbee testing code, remove after testing is done
	#ifdef XbeeTest
 	/*	TRISFbits.TRISF5=0;//RF5 output
 		TRISDbits.TRISD9=0;//RD9 output
 		TRISDbits.TRISD10=1;//RD10 input	*/
 	#endif

//


 	//I/O initializing complete


	bringup_board();

// 	Cell_Ctrl(Cell_A,Cell_ON);
// 	Cell_Ctrl(Cell_B,Cell_ON);
 	//Initialize motor drivers
 	M1_MODE=1;
 	M2_MODE=1;
	//*******************************************
 	InterruptIni();
 	//initialize AD
 	IniAD();
	//initialize timer
	IniTimer2();
	IniTimer3();
	IniTimer1();
 	IniTimer4();
 	IniTimer5();
	//initialize PWM sub module
 	PWM1Ini();
	PWM2Ini();
	PWM3Ini();
	PWM4Ini();
	PWM5Ini();
	PWM6Ini();
	PWM7Ini();
	PWM8Ini();
	PWM9Ini();
	//initialize input capture
	IniIC1();
	IniIC2();
	IniIC3();
	IniIC4();
	//IniIC5();
 	//IniIC6();

 	I2C1Ini();
 	I2C2Ini();
 	I2C3Ini();
 	#ifdef XbeeTest
 	UART1Ini();
 	#endif
  	TMPSensorICIni();
 	FANCtrlIni();

}

void InterruptIni()
{
 	//remap all the interrupt routines
 	T2InterruptUserFunction=Motor_T2Interrupt;
 	T3InterruptUserFunction=Motor_T3Interrupt;
 	T4InterruptUserFunction=Motor_T4Interrupt;
 	T5InterruptUserFunction=Motor_T5Interrupt;
 	IC1InterruptUserFunction=Motor_IC1Interrupt;
// 	IC2InterruptUserFunction=Motor_IC2Interrupt;
 	IC3InterruptUserFunction=Motor_IC3Interrupt;
 //	IC4InterruptUserFunction=Motor_IC4Interrupt;
 	//IC5InterruptUserFunction=Motor_IC5Interrupt;
 	//IC6InterruptUserFunction=Motor_IC6Interrupt;
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
 	//reset the IC
 	I2C3ResigsterWrite(FANCtrlAddressW,0x02,0b01011000);

 	//auto fan speed control mode
 	I2C3ResigsterWrite(FANCtrlAddressW,0x11,0b00111100);

 	//for FAN1 starting temperature
 	I2C3ResigsterWrite(FANCtrlAddressW,0x0F,Fan1LowTemp);

 	//for FAN2 starting temperature
 	I2C3ResigsterWrite(FANCtrlAddressW,0x10,Fan2LowTemp);

 	//for duty-cycle step
 	I2C3ResigsterWrite(FANCtrlAddressW,0x13,0b11111111);

 	//for duty-cycle change rate
 	I2C3ResigsterWrite(FANCtrlAddressW,0x12,0b00100100);
	
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


/*
 	//No initial address
 	I2C3MSK = 0x3FF;
 	//turn off IPMI
 	I2C3CONbits.IPMIEN = 0;
 	//clock release control
 	I2C3CONbits.SCLREL = 1;
 	//disable slew rate control
 	I2C3CONbits.DISSLW = 1;
 	//Baud rate: 200kHz
 	I2C3BRG = 77;


 	//7 bits address
 	I2C3CONbits.A10M=0;
 	//enable SMbus input level
 	I2C3CONbits.SMEN=1;
 	//acknowledgement enabled
 	I2C3CONbits.ACKDT=1;


 	//enable I2C module
 	I2C3CONbits.I2CEN = 1;

//talk to TMPSensorIC
//1-Slave addresss with R/W indication
//2-Pointer register byte
//3-Data byte 1
//4-Data byte 2
*/
}

/***************************Test***********************************/
void test()
{
/*
 	int i;
 	for(i=0;i<3;i++)
 	{
 		GetControlRPM(i);
 		GetControlCurrent(i);
 	}
*/ 	
 	//printf("LMotor:%ld,%ld,%ld,%d--RMotor:%ld,%ld,%ld,%d\n",ControlRPM[LMotor],ControlCurrent[LMotor],EnCount[LMotor],MotorDuty[LMotor],ControlRPM[RMotor],ControlCurrent[RMotor],EnCount[RMotor],MotorDuty[RMotor]);
// 	printf("LMotor:%ld,%ld,%ld,%ld,%ld,%ld\n",ControlRPM[LMotor],ControlCurrent[LMotor],EnCount[LMotor],EncoderFBInterval[LMotor][0],EncoderFBInterval[LMotor][1],EncoderFBInterval[LMotor][2]);
// 	printf("LMotor EncoderFB Timer Interval: %ld,%ld,%ld,%ld\n",EncoderFBInterval[LMotor][0],EncoderFBInterval[LMotor][1],EncoderFBInterval[LMotor][2],EncoderFBInterval[LMotor][3]);
// 	printf("Direction: %d,%d,%d,%d\n",DIR[LMotor][0],DIR[LMotor][1],DIR[LMotor][2],DIR[LMotor][3]);
/*
 	printf("%ld,%ld,%ld\n",CurrentRPM[LMotor],BackEMFCOEF[LMotor],EnCount[LMotor]);
*/

// 	printf("%ld\n",MotorCurrent[LMotor]);
/*
 	for(i=0;i<SampleLength;i++)
 	{
 	 	printf("%d,%ld\n",i,MotorCurrentAD[LMotor][i]);
 	 	
 	}
*/
/*
  	long temp1,temp2;

 	IEC2bits.IC6IE=CLEAR;
 	IEC2bits.IC5IE=CLEAR;
 	IEC2bits.IC4IE=CLEAR;
 	IEC2bits.IC3IE=CLEAR;
 	IEC0bits.IC2IE=CLEAR;
 	IEC0bits.IC1IE=CLEAR;

 	//timing testing
 	PWM2Duty(600); 
 	PWM4Duty(200);
 	//while(1);
*/
/* 	
 	for(i=0;i<SampleLength;i++)
 	{
 	 	temp1=EncoderFBInterval[LMotor][i];
 	 	temp2=CurrentRPM[LMotor];
 	 	printf("%u,%lu,%d,%lu\n",i,temp1,DIR[LMotor][i],temp2);
 	 	//UART1Tranmit(temp);
 	 	//printf("%u,%u\n",LEncoderCurrentValue,LEncoderLastValue)
 	}
*/ 	

/*
 	printf("L A %ld\n",BackEMF[LMotor][ChannelA]);
// 	printf("L B %ld\n",BackEMF[LMotor][ChannelB]);
// 	printf("L C %ld\n",MotorCurrent[LMotor]);
// 	printf("R A %ld\n",BackEMF[RMotor][ChannelA]);
// 	printf("R B %ld\n",BackEMF[RMotor][ChannelB]);
// 	printf("R C %ld\n",MotorCurrent[RMotor]);
// 	printf("F A %ld\n",BackEMF[Flipper][ChannelA]);
// 	printf("F B %ld\n",BackEMF[Flipper][ChannelB]);
// 	printf("F C %ld\n",MotorCurrent[Flipper]);
// 	printf("T C %ld\n",TotalCurrent);
*/
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
	OC1RS=Period30000Hz;
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
	OC1R=(Period1*Duty)>>10;
}
//****************************************************


//initialize PWM chnnel 2
void PWM2Ini(void)
{

	OC2R=0;
	OC2RS=Period30000Hz;
	OC2CON2bits.SYNCSEL=0x1F;
	OC2CON2bits.OCTRIG=CLEAR;
	OC2CON1bits.OCTSEL=0b000;//Timer2
	OC2CON1bits.OCM=0b110;
	Period2=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 2
void PWM2Duty(int Duty)
{
	OC2R=(Period2*Duty)>>10;
}
//****************************************************

//****************************************************
//initialize PWM chnnel 3
void PWM3Ini(void)
{
	OC3R=0;
	OC3RS=Period30000Hz;
	OC3CON2bits.SYNCSEL=0x1F;
	OC3CON2bits.OCTRIG=CLEAR;
	OC3CON1bits.OCTSEL=0b000;//Timer2
	OC3CON1bits.OCM=0b110;
	Period3=Period30000Hz;//Period is used for all the other PWMxDuty() functions
}

//set duty cycle for PWM channel 3
void PWM3Duty(int Duty)
{
	OC3R=(Period3*Duty)>>10;
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
 	IEC0bits.T2IE=SET;// enable the interrupt
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
	T4CON=0x0010;//stops timer4,1:8 prescale,16 bit timer,internal clock (Fosc/2)
	TMR4=0;//clear timer1 register
 	IFS1bits.T4IF = 0;	//clear interrupt flag
 	//IEC1bits.T4IE=SET;
	T4CONbits.TON=SET;

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
//a) Check that the SYNCSEL bits are not set to‘00000’.
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

void IniIC2()
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
	while(IC2CON1bits.ICBNE==SET)
	{
	 	temp=IC2BUF;
 	}
//4. Set the SYNCSEL bits (ICxCON2<4:0>) to the
//desired sync/trigger source.
	IC2CON2bits.SYNCSEL=0b00000;// not snycronized to anything
//5. Set the ICTSEL bits (ICxCON1<12:10>) for the
//desired clock source.
	IC2CON1bits.ICTSEL=0b010;//timer 4
//6. Set the ICI bits (ICxCON1<6:5>) to the desired
//interrupt frequency
	IC2CON1bits.ICI=0b00; 	//interrupt on every capture event
//7. Select Synchronous or Trigger mode operation:
//a) Check that the SYNCSEL bits are not set to‘00000’.
/*
 	if(IC5CON2bits.SYNCSEL==CLEAR)
	{
	 	IC5CON2bits.SYNCSEL=0b10100; 	//sync with IC1
 	}
*/	
//b) For Synchronous mode, clear the ICTRIG
//bit (ICxCON2<7>).
	IC2CON2bits.ICTRIG=0b0;//synchronous mode
//c) For Trigger mode, set ICTRIG, and clear the
//TRIGSTAT bit (ICxCON2<6>).

//8. Set the ICM bits (ICxCON1<2:0>) to the desired
//operational mode.
 	
	IC2CON1bits.ICM=0b011;//capture on every rising edge
 	//IC2CON1bits.ICM=0b110;//disable first, should be enabled only by IC1
//9. Enable the selected trigger/sync source.

	IFS0bits.IC2IF=CLEAR;//clear the interrupt flag	
//	IEC0bits.IC2IE=SET;//start interrupt
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
//a) Check that the SYNCSEL bits are not set to‘00000’.
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
void IniIC4()
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
	while(IC4CON1bits.ICBNE==SET)
	{
	 	temp=IC4BUF;
 	}
//4. Set the SYNCSEL bits (ICxCON2<4:0>) to the
//desired sync/trigger source.
	IC4CON2bits.SYNCSEL=0b00000;// not snycronized to anything
//5. Set the ICTSEL bits (ICxCON1<12:10>) for the
//desired clock source.
	IC4CON1bits.ICTSEL=0b010;//timer 4
//6. Set the ICI bits (ICxCON1<6:5>) to the desired
//interrupt frequency
	IC4CON1bits.ICI=0b00; 	//interrupt on every capture event
//7. Select Synchronous or Trigger mode operation:
//a) Check that the SYNCSEL bits are not set to‘00000’.
/*
 	if(IC5CON2bits.SYNCSEL==CLEAR)
	{
	 	IC5CON2bits.SYNCSEL=0b10100; 	//sync with IC1
 	}
*/	
//b) For Synchronous mode, clear the ICTRIG
//bit (ICxCON2<7>).
	IC4CON2bits.ICTRIG=0b0;//synchronous mode
//c) For Trigger mode, set ICTRIG, and clear the
//TRIGSTAT bit (ICxCON2<6>).

//8. Set the ICM bits (ICxCON1<2:0>) to the desired
//operational mode.
	IC4CON1bits.ICM=0b011;//capture on every rising edge
//9. Enable the selected trigger/sync source.

	IFS2bits.IC4IF=CLEAR;//clear the interrupt flag	
//	IEC2bits.IC4IE=SET;//start the interrupt
}


void IniIC5()
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
	while(IC5CON1bits.ICBNE==SET)
	{
	 	temp=IC5BUF;
 	}
//4. Set the SYNCSEL bits (ICxCON2<4:0>) to the
//desired sync/trigger source.
	IC5CON2bits.SYNCSEL=0b00000;// not snycronized to anything
//5. Set the ICTSEL bits (ICxCON1<12:10>) for the
//desired clock source.
	IC5CON1bits.ICTSEL=0b010;//timer 4
//6. Set the ICI bits (ICxCON1<6:5>) to the desired
//interrupt frequency
	IC5CON1bits.ICI=0b00; 	//interrupt on every capture event
//7. Select Synchronous or Trigger mode operation:
//a) Check that the SYNCSEL bits are not set to‘00000’.
/*
 	if(IC5CON2bits.SYNCSEL==CLEAR)
	{
	 	IC5CON2bits.SYNCSEL=0b10100; 	//sync with IC1
 	}
*/	
//b) For Synchronous mode, clear the ICTRIG
//bit (ICxCON2<7>).
	IC5CON2bits.ICTRIG=0b0;//synchronous mode
//c) For Trigger mode, set ICTRIG, and clear the
//TRIGSTAT bit (ICxCON2<6>).

//8. Set the ICM bits (ICxCON1<2:0>) to the desired
//operational mode.
	IC5CON1bits.ICM=0b001;//capture on every edge
//9. Enable the selected trigger/sync source.

	IFS2bits.IC5IF=CLEAR;//clear the interrupt flag	
//	IEC2bits.IC5IE=SET;//start the interrupt
}

void IniIC6()
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
	while(IC6CON1bits.ICBNE==SET)
	{
	 	temp=IC6BUF;
 	}
//4. Set the SYNCSEL bits (ICxCON2<4:0>) to the
//desired sync/trigger source.
	IC6CON2bits.SYNCSEL=0b00000;// not snycronized to anything
//5. Set the ICTSEL bits (ICxCON1<12:10>) for the
//desired clock source.
	IC6CON1bits.ICTSEL=0b010;//timer 4
//6. Set the ICI bits (ICxCON1<6:5>) to the desired
//interrupt frequency
	IC6CON1bits.ICI=0b00; 	//interrupt on every capture event
//7. Select Synchronous or Trigger mode operation:
//a)
//b) For Synchronous mode, clear the ICTRIG
//bit (ICxCON2<7>).
	IC6CON2bits.ICTRIG=0b0;//synchronous mode
//c) For Trigger mode, set ICTRIG, and clear the
//TRIGSTAT bit (ICxCON2<6>).

//8. Set the ICM bits (ICxCON1<2:0>) to the desired
//operational mode.
	IC6CON1bits.ICM=0b001;//capture on every edge
//9. Enable the selected trigger/sync source.

	IFS2bits.IC6IF=CLEAR;//clear the interrupt flag	
//	IEC2bits.IC6IE=SET;//start the interrupt	

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

/*void  Motor_IC2Interrupt(void)
{
 	long temp1;
 	IFS0bits.IC2IF=0;//clear the interrupt flag

 	while(IC2CON1bits.ICBNE==SET)
 	{
 	 	temp1=IC2BUF;
 	 	if(temp1-LEncoderLastValue<0)
 	 	{
 	 	 	temp1+=65535*LEncoderBOverFlowCount;
 	 	 	LEncoderBOverFlowCount=0;
 	 	}
 	 	if((EncoderFBInterval[LMotor][(EncoderFBIntervalPointer[LMotor]-1)&(SampleLength-1)]/(temp1-LEncoderLastValue))<2)
 	 	{
 	 	 	DIR[LMotor][EncoderFBIntervalPointer[LMotor]]=-1;
 	 	}else
 	 	{
 	 	 	DIR[LMotor][EncoderFBIntervalPointer[LMotor]]=1;
 	 	}
 		EnCount[LMotor]+=DIR[LMotor][EncoderFBIntervalPointer[LMotor]];
	}
	 	
}*/

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

/*void  Motor_IC4Interrupt(void)
{
 	long temp1;
 	IFS2bits.IC4IF=0;//clear the flag
 	while(IC4CON1bits.ICBNE==SET)
 	{
 	 	temp1=IC4BUF;
 	 	if(temp1-REncoderLastValue<0)
 	 	{
 	 	 	temp1+=65535*REncoderBOverFlowCount;
 	 	 	REncoderBOverFlowCount=0;
 	 	}
 	 	if((EncoderFBInterval[RMotor][(EncoderFBIntervalPointer[RMotor]-1)&(SampleLength-1)]/(temp1-REncoderLastValue))<2)
 	 	{
 	 	 	DIR[RMotor][EncoderFBIntervalPointer[RMotor]]=-1;
 	 	}else
 	 	{
 	 	 	DIR[RMotor][EncoderFBIntervalPointer[RMotor]]=1;
 	 	}
 		EnCount[RMotor]+=DIR[RMotor][EncoderFBIntervalPointer[RMotor]];
	}
}*/

/*void  Motor_IC5Interrupt(void)
{

	static unsigned int IC5LastValue = 0;
	unsigned int IC5CurrentValue;
	unsigned int IC5CurrentPulseWidth;

	IFS2bits.IC5IF=0;//clear the interrupt flag
	IC5CurrentValue=IC5BUF;//save the current time
 	IC5CurrentPulseWidth=IC5CurrentValue+ICLMotorOverFlowCount*65535-IC5LastValue;
 	ICLMotorOverFlowCount=0;
 	if(IC5CurrentPulseWidth<=4800 && IC5CurrentPulseWidth>=1600)//measure high pulse width
	{
		//new input for LMotor
		//NewInput[LMotor]=True;
		PulseWidth[LMotor]=IC5CurrentPulseWidth;
		//PWM2Duty(IC5CurrentPulseWidth/3);
		//PWM2Duty(500);
	}	
	IC5LastValue=IC5CurrentValue;

}

void  Motor_IC6Interrupt(void)
{
	static unsigned int IC6LastValue = 0;
	unsigned int IC6CurrentValue;
	unsigned int IC6CurrentPulseWidth;

	IFS2bits.IC6IF=0;//clear the interrupt flag
	IC6CurrentValue=IC6BUF;//save the current time
 	IC6CurrentPulseWidth=IC6CurrentValue+ICRMotorOverFlowCount*65535-IC6LastValue;
 	ICRMotorOverFlowCount=0;
 	if(IC6CurrentPulseWidth<=4800 && IC6CurrentPulseWidth>=1600)//measure high pulse width
	{
		//new input for LMotor
		//NewInput[LMotor]=True;
		PulseWidth[RMotor]=IC6CurrentPulseWidth;
		//PWM2Duty(IC6CurrentPulseWidth/3);
		//PWM2Duty(500);
	}	
	IC6LastValue=IC6CurrentValue;

}*/


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

 /*	 if(temp>BackEMFSampleRangeStart && temp<BackEMFSampleRangeEnd)
 	{
 		AD1CON1bits.ASAM=SET;
 	}


 	if(Timer3Count>=5 || BackEMFSampleEnabled==True)
 	{
 		AD1CON1bits.ASAM=SET;
 	}
 	if(Timer3Count>=5)
 	{
 		Timer3Count=0;
 	}*/
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
 	//stop the conversion
 	AD1CON1bits.ASAM=CLEAR;
 	
 	//clear the flag
 	IFS0bits.AD1IF=CLEAR;
 	//load the value
// 	if(BackEMFSampleEnabled==True)
// 	{
 	/*	BackEMF[LMotor][ChannelB][BackEMFPointer]=1023-ADC1BUF8;
 		BackEMF[LMotor][ChannelA][BackEMFPointer]=1023-ADC1BUF9;
 		BackEMF[RMotor][ChannelA][BackEMFPointer]=1023-ADC1BUF5;
 		BackEMF[RMotor][ChannelB][BackEMFPointer]=1023-ADC1BUF4;*/
 		BackEMFSampleEnabled=False;
// 	}

	//BackEMF[LMotor][ChannelB][BackEMFPointer]=ADC1BUFD;
 	M3_POSFB_Array[0][M3_POSFB_ArrayPointer]=ADC1BUF4;
 	M3_POSFB_Array[1][M3_POSFB_ArrayPointer]=ADC1BUF5; 	
 	MotorCurrentAD[LMotor][MotorCurrentADPointer]=ADC1BUF3;
 	MotorCurrentAD[RMotor][MotorCurrentADPointer]=ADC1BUF1;
 	MotorCurrentAD[Flipper][MotorCurrentADPointer]=ADC1BUFB;
	Total_Cell_Current_Array[Total_Cell_Current_ArrayPointer]=ADC1BUF8+ADC1BUF9;
	adc_test_reg = ADC1BUF9;
 	CellVoltageArray[Cell_A][CellVoltageArrayPointer]=ADC1BUF6;
 	CellVoltageArray[Cell_B][CellVoltageArrayPointer]=ADC1BUF7;
 	TotalCurrent=MotorCurrentAD[LMotor][MotorCurrentADPointer]+MotorCurrentAD[RMotor][MotorCurrentADPointer]+MotorCurrentAD[Flipper][MotorCurrentADPointer];

//tesing code only
 	//REG_PWR_TOTAL_CURRENT=Total_Cell_Current_Array[Total_Cell_Current_ArrayPointer];
//testing code end

//testing code
 	//REG_MOTOR_FB_CURRENT.left=MotorCurrentAD[LMotor][MotorCurrentADPointer];
 	//REG_MOTOR_FB_CURRENT.right=MotorCurrentAD[RMotor][MotorCurrentADPointer];
 	//REG_MOTOR_FB_CURRENT.flipper=MotorCurrentAD[Flipper][MotorCurrentADPointer];
//tesing code end

 	if(TotalCurrent>=CurrentLimit)
 	{
 		//start protection timer
		T5CONbits.TON=SET;//start clock
 		IEC1bits.T5IE=SET;//enable the interrupt
		CurrentTooHigh=True;
 	}
 
 	//increase array pointer, prevent over flow
 	CellVoltageArrayPointer++;
 	CellVoltageArrayPointer&=(SampleLength-1);
 	Total_Cell_Current_ArrayPointer++;
 	Total_Cell_Current_ArrayPointer&=(SampleLength-1);
 	M3_POSFB_ArrayPointer++;
 	M3_POSFB_ArrayPointer&=(SampleLength-1);
 	BackEMFPointer++;
 	BackEMFPointer&=(SampleLength-1);
 	MotorCurrentADPointer++;
 	MotorCurrentADPointer&=(SampleLength-1);

 	#ifdef XbeeTest
 	if(U1STAbits.UTXBF==0 && XbeeTest_UART_BufferPointer==0 && (REG_MOTOR_VELOCITY.left||REG_MOTOR_VELOCITY.right||REG_MOTOR_VELOCITY.flipper)!=0)//if transmit reg is empty and last packet is sent
 	{
 		U1TXREG=255;//send out the index
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
 		XbeeTest_Temp_u16=TotalCurrent;
		XbeeTest_UART_Buffer[0]=XbeeTest_UART_DataNO;
 		XbeeTest_UART_Buffer[1]=(XbeeTest_Temp_u16>>8);//load the buffer
 		XbeeTest_UART_Buffer[2]=XbeeTest_Temp_u16;
 		switch (XbeeTest_UART_DataNO%40)
 		{ 
 		 	case 0:		//0-REG_PWR_TOTAL_CURRENT HI
 		 		XbeeTest_UART_Buffer[3]=REG_PWR_TOTAL_CURRENT>>8;
 				break;
 		 	case 1:		//1-REG_PWR_TOTAL_CURRENT LO
 				XbeeTest_UART_Buffer[3]=REG_PWR_TOTAL_CURRENT;
 				break;
 		 	case 2:		//2-REG_MOTOR_FB_RPM.left HI
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_RPM.left>>8;
 				break;
 		 	case 3: 	//3-REG_MOTOR_FB_RPM.left LO
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_RPM.left;
 				break;
 		 	case 4: 	//4-REG_MOTOR_FB_RPM.right HI
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_RPM.right>>8;
 				break;
 		 	case 5: 	//5-REG_MOTOR_FB_RPM.right LO
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_RPM.right;
 				break;
 		 	case 6: 	//6-REG_FLIPPER_FB_POSITION.pot1 HI
 				XbeeTest_UART_Buffer[3]=REG_FLIPPER_FB_POSITION.pot1>>8;
 				break;
 		 	case 7: 	//7-REG_FLIPPER_FB_POSITION.pot1 LO
 				XbeeTest_UART_Buffer[3]=REG_FLIPPER_FB_POSITION.pot1;
 				break;
 		 	case 8: 	//8-REG_FLIPPER_FB_POSITION.pot2 HI
 				XbeeTest_UART_Buffer[3]=REG_FLIPPER_FB_POSITION.pot2>>8;
 				break;
 		 	case 9: 	//9-REG_FLIPPER_FB_POSITION.pot2 LO
 				XbeeTest_UART_Buffer[3]=REG_FLIPPER_FB_POSITION.pot2;
 				break;
 			case 10: 	//10-REG_MOTOR_FB_CURRENT.left HI
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_CURRENT.left>>8;
 				break;
 			case 11: 	//11-REG_MOTOR_FB_CURRENT.left LO
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_CURRENT.left;
 				break;
 		 	case 12: 	//12-REG_MOTOR_FB_CURRENT.right HI
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_CURRENT.right>>8;
 				break;
 			case 13:	//13-REG_MOTOR_FB_CURRENT.right LO
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FB_CURRENT.right;
 				break;
 			case 14:	//14-REG_MOTOR_ENCODER_COUNT.left HI
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_ENCODER_COUNT.left>>8;
 				break;
 		 	case 15:	//15-REG_MOTOR_ENCODER_COUNT.left LO
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_ENCODER_COUNT.left;
 				break;
 			case 16:	//16-REG_MOTOR_ENCODER_COUNT.right HI
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_ENCODER_COUNT.right>>8;
 				break;
 		 	case 17:	//17-REG_MOTOR_ENCODER_COUNT.right LO
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_ENCODER_COUNT.right;
 				break;
 		 	case 18: 	//18-REG_MOTOR_FAULT_FLAG.left
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FAULT_FLAG.left;
 				break;
 		 	case 19:	//19-REG_MOTOR_FAULT_FLAG.right
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_FAULT_FLAG.right;
 				break;
 		 	case 20: 	//20-REG_MOTOR_TEMP.left HI
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_TEMP.left>>8;
 				break; 				
 			case 21:	//21-REG_MOTOR_TEMP.left LO
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_TEMP.left;
 				break;
 			case 22:	//22-REG_MOTOR_TEMP.right HI
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_TEMP.right>>8;
 				break;
 			case 23:	//23-REG_MOTOR_TEMP.right LO
 				XbeeTest_UART_Buffer[3]=REG_MOTOR_TEMP.right;
 				break;
 			case 24:	//24-REG_PWR_BAT_VOLTAGE.a HI
 				XbeeTest_UART_Buffer[3]=REG_PWR_BAT_VOLTAGE.a>>8;
 				break;
 			case 25:	//25-REG_PWR_BAT_VOLTAGE.a LO
 				XbeeTest_UART_Buffer[3]=REG_PWR_BAT_VOLTAGE.a;
 				break;
 		 	case 26:	//26-REG_PWR_BAT_VOLTAGE.b HI
 				XbeeTest_UART_Buffer[3]=REG_PWR_BAT_VOLTAGE.b>>8;
 				break;
 			case 27:	//27-REG_PWR_BAT_VOLTAGE.b LO
 				XbeeTest_UART_Buffer[3]=REG_PWR_BAT_VOLTAGE.b;
 				break;
	
 		}
 	}
 	if((REG_MOTOR_VELOCITY.left||REG_MOTOR_VELOCITY.right||REG_MOTOR_VELOCITY.flipper)!=0)
 	{
 		XbeeTest_UART_DataNO++;//index++, if transmit reg is full, skip this
 		if(XbeeTest_UART_DataNO==201)
 		{
 			XbeeTest_UART_DataNO=0;
 		}
 	}
 	#endif
}

void  Motor_U1TXInterrupt(void)
{
 	/*
 	#define XbeeTest_UART_Buffer_Length 3 //no more than 5 bytes based on 57600 baud and 1ms system loop
 	uint8_t XbeeTest_UART_Buffer[XbeeTest_UART_Buffer_Length];
 	uint8_t XbeeTest_UART_BufferPointer=0;
 	uint8_t XbeeTest_UART_DataNO=0;
 	*/

 	//clear the flag
 	IFS0bits.U1TXIF=0;
 	#ifdef XbeeTest
 	//transmit data
 	if(XbeeTest_UART_BufferPointer<XbeeTest_UART_Buffer_Length)
 	{
 	 	U1TXREG=XbeeTest_UART_Buffer[XbeeTest_UART_BufferPointer];
 		XbeeTest_UART_BufferPointer++;
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
 	/*
 	#define XbeeTest_BufferLength 3
 	#define XbeeTest_StateIdle 0
 	#define XbeeTest_StateProcessing 1
	int XbeeTest_State=XbeeTest_StateIdle;
 	int16_t XbeeTest_Buffer[3];
 	int XbeeTest_BufferArrayPointer=0;
 	int XbeeTest_Temp;
 	*/
 	//printf("I got it!!");
 	XbeeTest_Temp=U1RXREG;
  	if(XbeeTest_State==XbeeTest_StateIdle)
 	{
		if(XbeeTest_Temp==255)
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
 		//input data range 0~250, 125 is stop, 0 is backwards full speed, 250 is forward full speed
 			//for Marge, the right and left is flipped, so left and right need to be flipped
 			gNewData = !gNewData;//low speed commend
 		 	REG_MOTOR_VELOCITY.left=-(XbeeTest_Buffer[0]*8-1000); 
 			REG_MOTOR_VELOCITY.right=-(XbeeTest_Buffer[1]*8-1000);
			REG_MOTOR_VELOCITY.flipper=-(XbeeTest_Buffer[2]*8-1000);	
 			//clear all the local buffer
 			XbeeTest_BufferArrayPointer=0;
 			XbeeTest_State=XbeeTest_StateIdle;
 		}
 	}		
	#endif
//XbeeTest code ends 	
}