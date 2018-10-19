#include <p24fxxxx.h>
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "interrupt_switch.h"
#include "testing.h"
#include "debug_uart.h"
#include "device_robot_motor_i2c.h"
#include "DEE Emulation 16-bit.h"
#include "i2clib.h"
#include "device_robot_motor_loop.h"
#include "../closed_loop_control/core/InputCapture.h"

#define XbeeTest
#define BATProtectionON
#define XbeeTest_TX_Enable

// variables
// sub system variables
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

MotorEvent Event[MOTOR_CHANNEL_COUNT] = {Stop, Stop, Stop};
MotorState StateLevel01[MOTOR_CHANNEL_COUNT] = {Protection, Protection, Protection};
MotorState2 StateLevel02[MOTOR_CHANNEL_COUNT] = {Locked, Locked, Locked};

long TargetParameter[MOTOR_CHANNEL_COUNT]; ///< Target speed (for left/right motor) or position (for
///< flipper)
unsigned int CurrentParameter[MOTOR_CHANNEL_COUNT]; ///< Current speed(for left and right motor) or
///< position (for flipper)
bool SwitchDirectionTimerExpired[MOTOR_CHANNEL_COUNT] = {false, false, false};
bool SwitchDirectionTimerEnabled[MOTOR_CHANNEL_COUNT] = {false, false, false};
int SwitchDirectionTimerCount[MOTOR_CHANNEL_COUNT] = {0, 0, 0};
bool SpeedUpdateTimerExpired[MOTOR_CHANNEL_COUNT] = {false, false, false};
bool SpeedUpdateTimerEnabled[MOTOR_CHANNEL_COUNT] = {false, false, false};
int SpeedUpdateTimerCount[MOTOR_CHANNEL_COUNT] = {0, 0, 0};
bool USBTimeOutTimerExpired = false;
long USBTimeOutTimerCount = 0;
bool USBTimeOutTimerEnabled = true;
bool StateMachineTimerEnabled = true;
bool StateMachineTimerExpired = false;
int StateMachineTimerCount = 0;
bool RPMTimerExpired = false;
bool RPMTimerEnabled = true;
int RPMTimerCount = 0;
bool CurrentFBTimerExpired = false;
bool CurrentFBTimerEnabled = true;
int CurrentFBTimerCount = 0;
bool M3_POSFB_TimerExpired = false;
bool M3_POSFB_TimerEnabled = true;
int M3_POSFB_timerCount = 0;
bool CurrentProtectionTimerEnabled = true;
bool CurrentProtectionTimerExpired = false;
int CurrentProtectionTimerCount = 0;
bool MotorOffTimerEnabled = false;
bool MotorOffTimerExpired = false;
int MotorOffTimerCount = 0;
bool CurrentSurgeRecoverTimerEnabled = false;
bool CurrentSurgeRecoverTimerExpired = false;
int CurrentSurgeRecoverTimerCount = 0;
bool I2C2TimerEnabled = true;
int I2C2TimerCount = 0;
bool I2C3TimerEnabled = true;
int I2C3TimerCount = 0;
bool SFREGUpdateTimerEnabled = true;
bool SFREGUpdateTimerExpired = false;
int SFREGUpdateTimerCount = 0;
bool BATVolCheckingTimerEnabled = true;
bool BATVolCheckingTimerExpired = false;
int BATVolCheckingTimerCount = 0;
bool BATRecoveryTimerEnabled = false;
bool BATRecoveryTimerExpired = true; // for initial powering of the power bus
int BATRecoveryTimerCount = 0;
int closed_loop_control_timer_count = 0;
int closed_loop_control_timer = 10;
bool Xbee_FanSpeedTimerEnabled = false;
bool Xbee_FanSpeedTimerExpired = false;
int Xbee_FanSpeedTimerCount = 0;

unsigned int ICLMotorOverFlowCount = 0;
unsigned int ICRMotorOverFlowCount = 0;
unsigned int LEncoderAOverFlowCount = 0;
unsigned int LEncoderBOverFlowCount = 0;
unsigned int REncoderAOverFlowCount = 0;
unsigned int REncoderBOverFlowCount = 0;

unsigned int LEncoderLastValue = 0;
unsigned int LEncoderCurrentValue = 0;
unsigned int REncoderLastValue = 0;
unsigned int REncoderCurrentValue = 0;

long Encoder_Interrupt_Counter[2] = {0, 0};

long EncoderFBInterval[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH] = {{0}};
int DIR[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH] = {{0}};
int EncoderFBIntervalPointer[MOTOR_CHANNEL_COUNT] = {0};

long MotorCurrentAD[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH] = {{0}};
int MotorCurrentADPointer = 0;
long RealTimeCurrent[MOTOR_CHANNEL_COUNT] = {0};
long Current4Control[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH_CONTROL] = {{0}};
int Current4ControlPointer[MOTOR_CHANNEL_COUNT] = {0};
long ControlCurrent[MOTOR_CHANNEL_COUNT] = {0};
long CurrentRPM[MOTOR_CHANNEL_COUNT] = {0};
long RPM4Control[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH_CONTROL] = {{0}};
int RPM4ControlPointer[MOTOR_CHANNEL_COUNT] = {0};
long ControlRPM[MOTOR_CHANNEL_COUNT] = {0};
long TotalCurrent;
long EnCount[MOTOR_CHANNEL_COUNT] = {0};
int16_t Robot_Motor_TargetSpeedUSB[MOTOR_CHANNEL_COUNT] = {0};

int Timer3Count = 0;
int M3_POSFB = 0;
int M3_POSFB_Array[2][SAMPLE_LENGTH] = {{0}}; ///< Flipper Motor positional feedback data
int M3_POSFB_ArrayPointer = 0;
int Total_Cell_Current = 0;
int Total_Cell_Current_Array[SAMPLE_LENGTH] = {0};
int Total_Cell_Current_ArrayPointer = 0;
int InitialCellVoltage[BATTERY_CHANNEL_COUNT] = {0};
int CellVoltageArray[BATTERY_CHANNEL_COUNT][SAMPLE_LENGTH];
int CellVoltageArrayPointer = 0;
int Cell_A_Current[SAMPLE_LENGTH];
int Cell_B_Current[SAMPLE_LENGTH];
// float
// SpeedCtrlKp[4][3]={{0.0002,0.0002,0.0002},{0.09,0.09,0.09},{0.09,0.09,0.09},{0.09,0.09,0.09}};//SpeedCtrlKp[i][j],i-
// control mode, j-MOTOR_LEFT, Right Motor, MOTOR_FLIPPER
float SpeedCtrlKp[4][MOTOR_CHANNEL_COUNT] = {
    {0.2, 0.2, 0.2},
    {0.03, 0.03, 0.03},
    {0.09, 0.09, 0.09},
    {0.09, 0.09,
     0.09}}; // SpeedCtrlKp[i][j],i- control mode, j-MOTOR_LEFT, Right Motor, MOTOR_FLIPPER
float SpeedCtrlKi[4][MOTOR_CHANNEL_COUNT] = {
    {0.01, 0.01, 0.01}, {0.001, 0.001, 0.001}, {0.2, 0.2, 0.2}, {0.2, 0.2, 0.2}};
float SpeedCtrlKd[4][MOTOR_CHANNEL_COUNT] = {
    {0.000, 0.000, 0.000}, {0.001, 0.001, 0.001}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
/*float CurrentCtrlKp[4][3]={{1.5,1.5,1.5},{1.0,1.0,1.0},{1.5,1.5,1.5},{1.5,1.5,1.5}};
float CurrentCtrlKi[4][3]={{0.5,0.5,0.5},{0.01,0.01,0.01},{1.5,1.5,1.5},{1.5,1.5,1.5}};
float CurrentCtrlKd[4][3]={{0.00,0.00,0.00},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};*/
ControlMode SpeedCtrlMode[MOTOR_CHANNEL_COUNT] = {
    ControlMode_Conservative, ControlMode_Conservative, ControlMode_Conservative};
long AccumulatedSpeedError[MOTOR_CHANNEL_COUNT] = {0, 0, 0};
long AccumulatedCurrentError[MOTOR_CHANNEL_COUNT] = {0, 0, 0};
long LastSpeedError[MOTOR_CHANNEL_COUNT] = {0, 0, 0};
long LastCurrentError[MOTOR_CHANNEL_COUNT] = {0, 0, 0};
long LastTarget[MOTOR_CHANNEL_COUNT] = {0, 0, 0};
bool OverCurrent = false;
int CurrentSurgeTimes = 0;
int MotorSpeedTargetCoefficient[MOTOR_CHANNEL_COUNT];
float MaxDuty = 1000.0;
bool MotorRecovering = false;
long TargetDifference = 0;
long Debugging_Dutycycle[MOTOR_CHANNEL_COUNT];
long MotorTargetRPM[MOTOR_CHANNEL_COUNT];
long Debugging_MotorTempError[MOTOR_CHANNEL_COUNT];
int Timer5Count = 0;
unsigned int flipper_angle_offset = 0;
void calibrate_flipper_angle_sensor(void);
static void read_stored_angle_offset(void);

void turn_on_power_bus_new_method(void);
void turn_on_power_bus_old_method(void);
void turn_on_power_bus_hybrid_method(void);
bool check_string_match(unsigned char *string1, unsigned char *string2, unsigned char length);

static void alternate_power_bus(void);

#ifdef XbeeTest
#define XbeeTest_BufferLength 6
#define XbeeTest_StateIdle 0
#define XbeeTest_StateProcessing 1
int XbeeTest_State = XbeeTest_StateIdle;
int16_t XbeeTest_Buffer[XbeeTest_BufferLength];
int XbeeTest_BufferArrayPointer = 0;
int XbeeTest_Temp;
uint16_t XbeeTest_Temp_u16;
#define XbeeTest_UART_Buffer_Length                                                                \
    4 // no more than 5 bytes based on 57600 baud and 1ms system loop
uint8_t XbeeTest_UART_Buffer[XbeeTest_UART_Buffer_Length];
uint8_t XbeeTest_UART_BufferPointer = 0;
uint8_t XbeeTest_UART_DataNO = 0;
int16_t Xbee_MOTOR_VELOCITY[3];
uint8_t Xbee_Incoming_Cmd[2];
int Xbee_gNewData = 0;
uint8_t Xbee_StartBit = 253;
uint16_t EncoderInterval[MOTOR_CHANNEL_COUNT]; // Encoder time interval
uint16_t BuildNO = 40621;
uint8_t Xbee_SIDE_FAN_SPEED = 0;
uint8_t Xbee_SIDE_FAN_NEW = 0; // if there is a cmd or no
#define P1_Read_Register 10
#define P1_Fan_Command 20
#define P1_Low_Speed_Set 240
#define P1_Calibration_Flipper 250
uint8_t Xbee_Low_Speed_mode = 0;
uint8_t Xbee_Calibration = 0;
#endif

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

// invalid flipper pot thresholds.  These are very wide because the flipper pots are on a
// different 3.3V supply than the PIC If the flipper pot is below this threshold, it is invalid
#define LOW_POT_THRESHOLD 33
// If the flipper pot is above this threshold, it is invalid
#define HIGH_POT_THRESHOLD 990
#define FLIPPER_POT_OFFSET -55

//*********************************************//
//**chief functions
void ClearSpeedCtrlData(MotorChannel Channel) {

    AccumulatedSpeedError[Channel] = 0;
    LastSpeedError[Channel] = 0;
    // printf("Control Data Cleared!\n");
}

void ClearCurrentCtrlData(MotorChannel Channel) {

    AccumulatedCurrentError[Channel] = 0;
    LastCurrentError[Channel] = 0;
}

/** returns the given angle, normalized to between 0 and 360 */
int wrap_angle(int value) { return (value % 360 + 360) % 360; }

void DeviceRobotMotorInit() {
    // local variables

    MC_Ini();

    handle_power_bus();
    FANCtrlIni();

    ProtectHB(MOTOR_LEFT);
    ProtectHB(MOTOR_RIGHT);
    ProtectHB(MOTOR_FLIPPER);

    test_function();

    initialize_i2c2_registers();
    initialize_i2c3_registers();

    // read flipper position from flash, and put it into a module variable
    read_stored_angle_offset();

    // init variables for closed loop control
    closed_loop_control_init();
}

void GetCurrent(MotorChannel Channel) {
    long temp = 0;
    // read the AD value
    temp = mean_l(SAMPLE_LENGTH, MotorCurrentAD[Channel]);
    RealTimeCurrent[Channel] = temp;
    Current4Control[Channel][Current4ControlPointer[Channel]] = temp;
    Current4ControlPointer[Channel] = (Current4ControlPointer[Channel] + 1) % SAMPLE_LENGTH_CONTROL;
}

void GetRPM(MotorChannel Channel) {
    static long ENRPM = 0;
    static int ENDIR = 0;
    static long LastEnCount[MOTOR_CHANNEL_COUNT] = {0, 0, 0};
    long ltemp1 = 0;

    ENRPM = 0;
    ENDIR = 0;

    // RPM Calculation
    // encoder RPM Calculation
    ltemp1 = mean_l(SAMPLE_LENGTH, EncoderFBInterval[Channel]);

    if (ltemp1 > 0) {
        ENRPM = 24000000 / ltemp1;
        // T4

    } else {
        ENRPM = 0;
    }

    if (Channel == MOTOR_LEFT) {
        if (M1_DIRO)
            ENDIR = -1;
        else
            ENDIR = 1;
    } else if (Channel == MOTOR_RIGHT) {
        if (M2_DIRO)
            ENDIR = 1;
        else
            ENDIR = -1;
    }

    CurrentRPM[Channel] = ENRPM * ENDIR;

    // if the input capture interrupt hasn't been called, the motors are not moving
    if (Encoder_Interrupt_Counter[Channel] == LastEnCount[Channel]) {
        CurrentRPM[Channel] = 0;
    }
    // otherwise, update the RPM
    else {
        if (Channel == MOTOR_LEFT) {
            REG_MOTOR_FB_RPM.left = CurrentRPM[MOTOR_LEFT];
        } else if (Channel == MOTOR_RIGHT) {
            REG_MOTOR_FB_RPM.right = CurrentRPM[MOTOR_RIGHT];
        }
    }

    LastEnCount[Channel] = Encoder_Interrupt_Counter[Channel];
}

void Device_MotorController_Process() {
    MotorChannel i;
    long temp1, temp2;
    static int overcurrent_counter = 0;
    // check if software wants to calibrate flipper position
#ifndef XbeeTest
    if (REG_MOTOR_VELOCITY.flipper == 12345) {
        calibrate_flipper_angle_sensor();
    }
#endif

#ifdef XbeeTest
    if (Xbee_Calibration == 1) {
        calibrate_flipper_angle_sensor();
        // clear the flag just for safe
        Xbee_Calibration = 0;
        Xbee_SIDE_FAN_SPEED = 240;
        Xbee_SIDE_FAN_NEW = 1;
        Xbee_FanSpeedTimerEnabled = true;
        Xbee_FanSpeedTimerCount = 0;
    }
#endif

    IC_UpdatePeriods();
    // Check Timer
    // Run control loop
    if (IFS0bits.T1IF == SET) {
        PORTFbits.RF5 = !PORTFbits.RF5;
        // clear the flag
        IFS0bits.T1IF = CLEAR;
        // check MOTOR_LEFT,MOTOR_RIGHT and MOTOR_FLIPPER
        // start counting all the timers
        if (StateMachineTimerEnabled) {
            StateMachineTimerCount++;
        }
        for (EACH_MOTOR_CHANNEL(i)) {
            if (SwitchDirectionTimerEnabled[i]) {
                SwitchDirectionTimerCount[i]++;
            }
            if (SpeedUpdateTimerEnabled[i]) {
                SpeedUpdateTimerCount[i]++;
            }
        }
        if (CurrentFBTimerEnabled) {
            CurrentFBTimerCount++;
        }
        if (RPMTimerEnabled) {
            RPMTimerCount++;
        }
        if (USBTimeOutTimerEnabled) {
            USBTimeOutTimerCount++;
            // printf("USBTimeOutTimerCount:%ld\n",USBTimeOutTimerCount);
        }
        if (CurrentProtectionTimerEnabled) {
            CurrentProtectionTimerCount++;
        }
        if (MotorOffTimerEnabled) {
            MotorOffTimerCount++;
        }
        if (CurrentSurgeRecoverTimerEnabled) {
            CurrentSurgeRecoverTimerCount++;
        }
        if (I2C2TimerEnabled) {
            I2C2TimerCount++;
        }
        if (I2C3TimerEnabled) {
            I2C3TimerCount++;
        }
        if (SFREGUpdateTimerEnabled) {
            SFREGUpdateTimerCount++;
        }
        if (BATVolCheckingTimerEnabled) {
            BATVolCheckingTimerCount++;
        }
        if (BATRecoveryTimerEnabled) {
            BATRecoveryTimerCount++;
        }
#ifdef XbeeTest
        if (Xbee_FanSpeedTimerEnabled) {
            Xbee_FanSpeedTimerCount++;
            // printf("%d",Xbee_FanSpeedTimerCount);
        }
#endif

        // this should run every 1ms
        closed_loop_control_timer_count++;
        alternate_power_bus();
    }

    // check if any timers are expired
    if (closed_loop_control_timer_count >= closed_loop_control_timer) {
        closed_loop_control_timer_count = 0;
        handle_closed_loop_control(OverCurrent);
    }
    if (CurrentProtectionTimerCount >= CurrentProtectionTimer) {
        CurrentProtectionTimerExpired = true;
    }
    if (USBTimeOutTimerCount >= USBTimeOutTimer) {
        USBTimeOutTimerExpired = true;
    }
    for (EACH_MOTOR_CHANNEL(i)) {
        // check SwitchDirectionTimer
        if (SwitchDirectionTimerCount[i] >= SwitchDirectionTimer) {
            SwitchDirectionTimerExpired[i] = true;
            SwitchDirectionTimerEnabled[i] = false; // stop the timer
            SwitchDirectionTimerCount[i] = 0;       // clear the timer
        }
        if (SpeedUpdateTimerCount[i] >= SpeedUpdateTimer) {
            SpeedUpdateTimerExpired[i] = true;
            SpeedUpdateTimerCount[i] = 0;
        }
    }
    if (RPMTimerCount >= RPMTimer) {
        RPMTimerExpired = true;
        RPMTimerCount = 0;
        RPMTimerEnabled = false;
    }
    if (CurrentFBTimerCount >= CurrentFBTimer) {
        CurrentFBTimerExpired = true;
        CurrentFBTimerCount = 0;
        CurrentFBTimerEnabled = false;
    }
    if (StateMachineTimerCount >= StateMachineTimer) {
        StateMachineTimerExpired = true;
    }
    if (MotorOffTimerCount >= MotorOffTimer) {
        MotorOffTimerExpired = true;
    }
    if (CurrentSurgeRecoverTimerCount >= CurrentSurgeRecoverTimer) {
        CurrentSurgeRecoverTimerExpired = true;
    }
    if (I2C2TimerCount >= I2C2Timer) {
        I2C2TimerExpired = true;
        I2C2TimerCount = 0;
        I2C2XmitReset = true;
    }
    if (I2C3TimerCount >= I2C3Timer) {
        I2C3TimerExpired = true;
        I2C3TimerCount = 0;
        I2C3XmitReset = true;
    }
    if (SFREGUpdateTimerCount >= SFREGUpdateTimer) {
        SFREGUpdateTimerExpired = true;
        SFREGUpdateTimerCount = 0;
    }
    if (BATVolCheckingTimerCount >= BATVolCheckingTimer) {
        BATVolCheckingTimerExpired = true;
        BATVolCheckingTimerCount = 0;
    }
    if (BATRecoveryTimerCount >= BATRecoveryTimer) {
        BATRecoveryTimerExpired = true;
        BATRecoveryTimerCount = 0;
    }
#ifdef XbeeTest
    if (Xbee_FanSpeedTimerCount >= Xbee_FanSpeedTimer) {
        Xbee_FanSpeedTimerExpired = true;
        Xbee_FanSpeedTimerCount = 0;
        Xbee_FanSpeedTimerEnabled = false;
        // printf("fan speed timer expired!");
    }
#endif

    // if any of the timers expired, execute relative codes
    // Control timer expired
    for (EACH_MOTOR_CHANNEL(i)) {
        if (SpeedUpdateTimerExpired[i]) {
            UpdateSpeed(i, StateLevel01[i]);
            SpeedUpdateTimerExpired[i] = false;
            // 	 	 	test();
        }
    }
    // t3

    if (RPMTimerExpired) {
        RPMTimerEnabled = true;
        RPMTimerExpired = false;
        GetRPM(MOTOR_LEFT);
        GetRPM(MOTOR_RIGHT);
    }
    // T6

    if (CurrentFBTimerExpired) {
        // clear CurrentFBTimerExpired
        CurrentFBTimerExpired = false;
        CurrentFBTimerEnabled = true;
        for (EACH_MOTOR_CHANNEL(i)) {
            GetCurrent(i);
        }
        TotalCurrent = RealTimeCurrent[MOTOR_LEFT] + RealTimeCurrent[MOTOR_RIGHT] +
                       RealTimeCurrent[MOTOR_FLIPPER];
        // printf("\nTotal Current%ld:\n",TotalCurrent);
    }
    if (MotorOffTimerExpired) {
        OverCurrent = false;
        MotorOffTimerExpired = false;
        MotorOffTimerEnabled = false;
        MotorOffTimerCount = 0;
        CurrentSurgeRecoverTimerEnabled = true;
        CurrentSurgeRecoverTimerCount = 0;
        CurrentSurgeRecoverTimerExpired = false;
        MotorRecovering = true;
    }
    if (CurrentSurgeRecoverTimerExpired) {
        CurrentSurgeRecoverTimerEnabled = false;
        CurrentSurgeRecoverTimerCount = 0;
        CurrentSurgeRecoverTimerExpired = false;
        MotorRecovering = false;
    }
    if (I2C2TimerExpired) {
        I2C2Update();
    }
    if (I2C3TimerExpired) {
        I2C3Update();
    }
    if (SFREGUpdateTimerExpired) {
        SFREGUpdateTimerExpired = false;

        // update flipper motor position
        temp1 = mean(SAMPLE_LENGTH, M3_POSFB_Array[0]);
        temp2 = mean(SAMPLE_LENGTH, M3_POSFB_Array[1]);
        REG_FLIPPER_FB_POSITION.pot1 = temp1;
        REG_FLIPPER_FB_POSITION.pot2 = temp2;
        REG_MOTOR_FLIPPER_ANGLE = return_calibrated_pot_angle(temp1, temp2);

        // update current for all three motors
        REG_MOTOR_FB_CURRENT.left = ControlCurrent[MOTOR_LEFT];
        REG_MOTOR_FB_CURRENT.right = ControlCurrent[MOTOR_RIGHT];
        REG_MOTOR_FB_CURRENT.flipper = ControlCurrent[MOTOR_FLIPPER];
        // update the encodercount for two driving motors
        REG_MOTOR_ENCODER_COUNT.left = Encoder_Interrupt_Counter[MOTOR_LEFT];
        REG_MOTOR_ENCODER_COUNT.right = Encoder_Interrupt_Counter[MOTOR_RIGHT];
        // update the mosfet driving fault flag pin 1-good 2-fault
        REG_MOTOR_FAULT_FLAG.left = PORTDbits.RD1;
        REG_MOTOR_FAULT_FLAG.right = PORTEbits.RE5;
        // update temperatures for two motors
        // done in I2C code
        // update battery voltage
        REG_PWR_BAT_VOLTAGE.a = mean(SAMPLE_LENGTH, CellVoltageArray[Cell_A]);
        REG_PWR_BAT_VOLTAGE.b = mean(SAMPLE_LENGTH, CellVoltageArray[Cell_B]);

        // update total current (out of battery)
        REG_PWR_TOTAL_CURRENT = mean(SAMPLE_LENGTH, Total_Cell_Current_Array);
    }
    if (BATVolCheckingTimerExpired) {

        // added this so that we check voltage faster
        temp1 = mean(SAMPLE_LENGTH, Cell_A_Current);
        temp2 = mean(SAMPLE_LENGTH, Cell_B_Current);

        REG_PWR_A_CURRENT = temp1;
        REG_PWR_B_CURRENT = temp2;

        BATVolCheckingTimerExpired = false;
#ifdef BATProtectionON
        //.01*.001mV/A * 11000 ohms = .11 V/A = 34.13 ADC counts/A
        // set at 10A per side
        if ((temp1 >= 512) || (temp2 >= 512)) {
            // Cell_Ctrl(Cell_A,Cell_OFF);
            // Cell_Ctrl(Cell_B,Cell_OFF);
            overcurrent_counter++;
            if (overcurrent_counter > 10) {
                PWM1Duty(0);
                PWM2Duty(0);
                PWM3Duty(0);
                ProtectHB(MOTOR_LEFT);
                ProtectHB(MOTOR_RIGHT);
                ProtectHB(MOTOR_FLIPPER);
                OverCurrent = true;
                BATRecoveryTimerCount = 0;
                BATRecoveryTimerEnabled = true;
                BATRecoveryTimerExpired = false;
            }

        } else if (temp1 <= 341 || temp2 <= 341) {
            overcurrent_counter = 0;
        }

#endif
    }
    if (BATRecoveryTimerExpired) {
        Cell_Ctrl(Cell_A, Cell_ON);
        Cell_Ctrl(Cell_B, Cell_ON);
        OverCurrent = false;
        BATRecoveryTimerExpired = false;
        BATRecoveryTimerCount = 0;
        BATRecoveryTimerEnabled = false;
    }
// xbee fan timer
#ifdef XbeeTest
    if (Xbee_FanSpeedTimerExpired) {
        Xbee_FanSpeedTimerExpired = false;
        Xbee_FanSpeedTimerEnabled = false;
        Xbee_FanSpeedTimerCount = 0;
        // clear all the fan command
        Xbee_SIDE_FAN_SPEED = 0;
        Xbee_SIDE_FAN_NEW = 0;
        // printf("clear side fan speed!");
    }
#endif

    // T5

    USBInput();
    if (CurrentProtectionTimerExpired) {
        CurrentProtectionTimerCount = 0;
        CurrentProtectionTimerExpired = false;
    }
    // T4

    // update state machine
    if (StateMachineTimerExpired) {
        // clear the flag
        // printf("State Machine Updated!\n");
        StateMachineTimerExpired = false;
        StateMachineTimerCount = 0;
        for (EACH_MOTOR_CHANNEL(i)) {
            // update state machine
            switch (StateLevel01[i]) {
            case Brake:
                // brake motor
                Braking(i);
                switch (Event[i]) {
                case Go:
                    ProtectHB(i);
                    StateLevel01[i] = Protection;
                    StateLevel02[i] = Locked;
                    break;
                case Back:
                    ProtectHB(i);
                    StateLevel01[i] = Protection;
                    StateLevel02[i] = Locked;
                    break;
                case Stop:
                    Event[i] = NoEvent;
                    break;
                case NoEvent:
                    break;
                }
                break;
            case Forward:
                switch (Event[i]) {
                case Go:
                    SpeedUpdateTimerEnabled[i] = true;
                    break;
                case Back:
                    ProtectHB(i);
                    StateLevel01[i] = Protection;
                    StateLevel02[i] = Locked;
                    break;
                case Stop:
                    ProtectHB(i);
                    StateLevel01[i] = Protection;
                    StateLevel02[i] = Locked;
                    break;
                case NoEvent:
                    break;
                }
                break;
            case Backward:
                switch (Event[i]) {
                case Go:
                    ProtectHB(i);
                    StateLevel01[i] = Protection;
                    StateLevel02[i] = Locked;
                    break;
                case Back:
                    SpeedUpdateTimerEnabled[i] = true;
                    break;
                case Stop:
                    ProtectHB(i);
                    StateLevel01[i] = Protection;
                    StateLevel02[i] = Locked;
                    break;
                case NoEvent:
                    break;
                }
                break;
            case Protection:
                if (StateLevel02[i] == Locked) {
                    if (SwitchDirectionTimerExpired[i]) {
                        StateLevel02[i] = Unlocked;
                        SwitchDirectionTimerExpired[i] = false;
                        SwitchDirectionTimerCount[i] = 0;
                    }
                }
                if (StateLevel02[i] == Unlocked) {
                    switch (Event[i]) {
                    case Stop:
                        Braking(i);
                        StateLevel01[i] = Brake;
                        break;
                    case Go:
                        SpeedUpdateTimerEnabled[i] = true;
                        StateLevel01[i] = Forward;
                        break;
                    case Back:
                        SpeedUpdateTimerEnabled[i] = true;
                        StateLevel01[i] = Backward;
                        break;
                    case NoEvent:
                        break;
                    }
                }
                break;
            }
        }
    }

    // test();//run testing code
}

//**Motor control functions

void Braking(MotorChannel Channel) {
    switch (Channel) {
    case MOTOR_LEFT:
        PWM1Duty(0);
        M1_COAST = Clear_ActiveLO;
        Nop();
        M1_BRAKE = Set_ActiveLO;
        break;
    case MOTOR_RIGHT:
        PWM2Duty(0);
        M2_COAST = Clear_ActiveLO;
        Nop();
        M2_BRAKE = Set_ActiveLO;
        break;
    case MOTOR_FLIPPER:
        PWM3Duty(0);
        M3_COAST = Clear_ActiveLO;
        Nop();
        M3_BRAKE = Set_ActiveLO;
        break;
    }
}

// disable the corresponding transistors preparing a direction switch
void ProtectHB(MotorChannel Channel) {
    SpeedUpdateTimerEnabled[Channel] = false;
    SpeedUpdateTimerCount[Channel] = 0;
    // start timer
    SwitchDirectionTimerEnabled[Channel] = true;
    // coast the motor
    ClearSpeedCtrlData(Channel);
    ClearCurrentCtrlData(Channel);
    switch (Channel) {
    case MOTOR_LEFT:
        M1_COAST = Set_ActiveLO;
        PWM1Duty(0);
        M1_BRAKE = Clear_ActiveLO;
        break;
    case MOTOR_RIGHT:
        M2_COAST = Set_ActiveLO;
        PWM2Duty(0);
        M2_BRAKE = Clear_ActiveLO;
        break;
    case MOTOR_FLIPPER:
        M3_COAST = Set_ActiveLO;
        PWM3Duty(0);
        M3_BRAKE = Clear_ActiveLO;
        break;
    }
}

int GetMotorSpeedTargetCoefficient(int Current) {
    int result = MotorSpeedTargetCoefficient_Normal;
    long temp = TargetDifference;

    // turning?
    if (temp < HardTurning && temp > StartTurning) {
        result = MotorSpeedTargetCoefficient_Turn +
                 (HardTurning - temp) *
                     (MotorSpeedTargetCoefficient_Normal - MotorSpeedTargetCoefficient_Turn) /
                     (HardTurning - StartTurning);
        // MaxDuty=750.0;
    } else if (temp >= HardTurning) {
        result = MotorSpeedTargetCoefficient_Turn;
        // MaxDuty=150.0;
    }

    // recovering protection
    if (MotorRecovering) {
        result = MotorSpeedTargetCoefficient_Turn +
                 (MotorSpeedTargetCoefficient_Normal - MotorSpeedTargetCoefficient_Turn) *
                     CurrentSurgeRecoverTimerCount / CurrentSurgeRecoverTimer;
    }
    if (result > MotorSpeedTargetCoefficient_Normal)
        result = MotorSpeedTargetCoefficient_Normal;
    if (result < MotorSpeedTargetCoefficient_Turn)
        result = MotorSpeedTargetCoefficient_Turn;
    return result;
}

int GetDuty(long CurrentState, long Target, MotorChannel Channel, ControlMode Mode) {
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

    // speed control
    LastTarget[Channel] = Target;
    MotorSpeedTargetCoefficient[Channel] = GetMotorSpeedTargetCoefficient(Channel);
    TargetRPM = (labs(Target) * MotorSpeedTargetCoefficient[Channel]) >>
                2; // abstract number-> RPM-- 17000 RPM (1000) is the top
    MotorTargetRPM[Channel] = TargetRPM;
    TempSpeedError = TargetRPM - labs(CurrentState);
    AccumulatedSpeedError[Channel] += TempSpeedError;
    Debugging_MotorTempError[Channel] = TempSpeedError;
    // PID Control
    // printf("Temp Error:%ld,Kp:%f, Accumulated Error:%li,
    // Ki:%f\n",TempSpeedError,SpeedCtrlKp[Mode][Channel],AccumulatedSpeedError[Channel],SpeedCtrlKi[Mode][Channel]);
    result = TempSpeedError * SpeedCtrlKp[Mode][Channel] +
             AccumulatedSpeedError[Channel] * SpeedCtrlKi[Mode][Channel] +
             (TempSpeedError - LastSpeedError[Channel]) * SpeedCtrlKd[Mode][Channel];
    LastSpeedError[Channel] = TempSpeedError;
    if (result >= MaxDuty) {
        result = MaxDuty;
        if (TempSpeedError > 0) {
            AccumulatedSpeedError[Channel] -= TempSpeedError;
        }
    }
    if (result < 0.0) {
        result = 0.0;
        // AccumulatedSpeedError[Channel]-=TempSpeedError;
        AccumulatedSpeedError[Channel] = 0;
    }

    if (Channel == MOTOR_FLIPPER) {
        return Target;
    } else {
        result = labs(Target);
        if (result >= MaxDuty)
            result = MaxDuty;
        return result;
    }
}

void UpdateSpeed(MotorChannel Channel, int State) {
    int Dutycycle;
    int temp;

    ControlRPM[Channel] = mean_l(SAMPLE_LENGTH_CONTROL, RPM4Control[Channel]);
    ControlCurrent[Channel] = mean_l(SAMPLE_LENGTH_CONTROL, Current4Control[Channel]);

    TargetDifference = labs(TargetParameter[MOTOR_LEFT] - TargetParameter[MOTOR_RIGHT]);
    temp = TargetDifference;
    // turning?
    if (temp >= HardTurning) {
        SpeedCtrlMode[Channel] = ControlMode_Conservative;
    } else {
        SpeedCtrlMode[Channel] = ControlMode_Normal;
    }
    switch (Channel) {
    case MOTOR_LEFT:
        if (State == Forward) {
            if (OverCurrent) {
                Dutycycle = 0;
                M1_COAST = Set_ActiveLO;
            } else {
                Dutycycle = GetDuty(ControlRPM[Channel], TargetParameter[Channel], Channel,
                                    SpeedCtrlMode[Channel]);
                M1_COAST = Clear_ActiveLO;
            }
            M1_BRAKE = Clear_ActiveLO;
            M1_DIR = HI;
            PWM1Duty(Dutycycle);
        } else if (State == Backward) {
            if (OverCurrent) {
                Dutycycle = 0;
                M1_COAST = Set_ActiveLO;
            } else {
                Dutycycle = GetDuty(ControlRPM[Channel], TargetParameter[Channel], Channel,
                                    SpeedCtrlMode[Channel]);
                M1_COAST = Clear_ActiveLO;
            }
            M1_BRAKE = Clear_ActiveLO;
            M1_DIR = LO;
            PWM1Duty(Dutycycle);
        }
        break;
    case MOTOR_RIGHT:
        if (State == Forward) {
            if (OverCurrent) {
                Dutycycle = 0;
                M2_COAST = Set_ActiveLO;
            } else {
                Dutycycle = GetDuty(ControlRPM[Channel], TargetParameter[Channel], Channel,
                                    SpeedCtrlMode[Channel]);
                M2_COAST = Clear_ActiveLO;
            }
            M2_BRAKE = Clear_ActiveLO;
            M2_DIR = LO;
            PWM2Duty(Dutycycle);
        } else if (State == Backward) {
            if (OverCurrent) {
                Dutycycle = 0;
                M2_COAST = Set_ActiveLO;
            } else {
                Dutycycle = GetDuty(ControlRPM[Channel], TargetParameter[Channel], Channel,
                                    SpeedCtrlMode[Channel]);
                M2_COAST = Clear_ActiveLO;
            }
            M2_BRAKE = Clear_ActiveLO;
            M2_DIR = HI;
            PWM2Duty(Dutycycle);
        }
        break;
    case MOTOR_FLIPPER:
        if (State == Forward) {
            Dutycycle = GetDuty(CurrentParameter[Channel], TargetParameter[Channel], Channel,
                                SpeedCtrlMode[Channel]);
            M3_COAST = Clear_ActiveLO;
            Nop();
            M3_BRAKE = Clear_ActiveLO;
            Nop();
            M3_DIR = HI;
            PWM3Duty(Dutycycle);
        } else if (State == Backward) {
            Dutycycle = GetDuty(CurrentParameter[Channel], -TargetParameter[Channel], Channel,
                                SpeedCtrlMode[Channel]);
            M3_COAST = Clear_ActiveLO;
            Nop();
            M3_BRAKE = Clear_ActiveLO;
            Nop();
            M3_DIR = LO;
            PWM3Duty(Dutycycle);
        }
        break;
    }
    Debugging_Dutycycle[Channel] = Dutycycle;
}

//*********************************************//

/** Tweak the speed of motor i toward the desired speed
 *   - Speed of a motor will cut to zero if desired_speed is 0
 *   - Speed of *all* motors will cut to zero if the OverCurrent flag is set.
 * Returns the new value for the given motor speed */
int speed_control_loop(MotorChannel i, int desired_speed) {
    static int motor_speed[MOTOR_CHANNEL_COUNT] = {0, 0, 0};

    if (desired_speed == 0) {
        motor_speed[i] = 0;
        return 0;
    }

    motor_speed[i] = clamp(desired_speed, motor_speed[i] - 1, motor_speed[i] + 1);

    if (OverCurrent) {
        motor_speed[MOTOR_LEFT] = 0;
        motor_speed[MOTOR_RIGHT] = 0;
        motor_speed[MOTOR_FLIPPER] = 0;
    }

    return motor_speed[i];
}

void USBInput() {
    MotorChannel i;
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

    // Motors must decelerate upon switching speeds (if they're not already stopped.
    // Failure to do so could cause large current spikes which will trigger the
    // protection circuitry in the battery, cutting power to the robot
    switch (state) {
    case HIGH_SPEED:

        control_loop_counter++;
        flipper_control_loop_counter++;

        if (control_loop_counter > 5) {
            control_loop_counter = 0;
#ifndef XbeeTest
            Robot_Motor_TargetSpeedUSB[MOTOR_LEFT] =
                speed_control_loop(MOTOR_LEFT, REG_MOTOR_VELOCITY.left);
            Robot_Motor_TargetSpeedUSB[MOTOR_RIGHT] =
                speed_control_loop(MOTOR_RIGHT, REG_MOTOR_VELOCITY.right);
#endif
#ifdef XbeeTest
            // printf("XL%d,XR,%d",Xbee_MOTOR_VELOCITY[0],Robot_Motor_TargetSpeedUSB[1]);
            Robot_Motor_TargetSpeedUSB[MOTOR_LEFT] =
                speed_control_loop(MOTOR_LEFT, Xbee_MOTOR_VELOCITY[MOTOR_LEFT]);
            Robot_Motor_TargetSpeedUSB[MOTOR_RIGHT] =
                speed_control_loop(MOTOR_RIGHT, Xbee_MOTOR_VELOCITY[MOTOR_RIGHT]);
            // printf("L%d,R%d\n",Robot_Motor_TargetSpeedUSB[0],Robot_Motor_TargetSpeedUSB[1]);
#endif
        }

        if (flipper_control_loop_counter > 15) {
            flipper_control_loop_counter = 0;
#ifndef XbeeTest
            Robot_Motor_TargetSpeedUSB[MOTOR_FLIPPER] =
                speed_control_loop(MOTOR_FLIPPER, REG_MOTOR_VELOCITY.flipper);
#endif
#ifdef XbeeTest
            Robot_Motor_TargetSpeedUSB[MOTOR_FLIPPER] =
                speed_control_loop(MOTOR_FLIPPER, Xbee_MOTOR_VELOCITY[MOTOR_FLIPPER]);
            // printf();
#endif
        }
#ifdef XbeeTest
        if (REG_MOTOR_SLOW_SPEED == 1)
            state = DECEL_AFTER_HIGH_SPEED;
#endif
#ifdef XbeeTest
        if (Xbee_Low_Speed_mode == 1)
            state = DECEL_AFTER_HIGH_SPEED;
#endif

        break;

    case DECEL_AFTER_HIGH_SPEED:

        control_loop_counter++;

        if (control_loop_counter > 3) {
            control_loop_counter = 0;

            // set speed to 1 (speed of 0 immediately stops motors)
            Robot_Motor_TargetSpeedUSB[MOTOR_LEFT] = speed_control_loop(MOTOR_LEFT, 1);
            Robot_Motor_TargetSpeedUSB[MOTOR_RIGHT] = speed_control_loop(MOTOR_RIGHT, 1);
            Robot_Motor_TargetSpeedUSB[MOTOR_FLIPPER] = speed_control_loop(MOTOR_FLIPPER, 1);
        }

        // motors are stopped if speed falls below 1%
        if ((abs(Robot_Motor_TargetSpeedUSB[MOTOR_LEFT]) < 10) &&
            (abs(Robot_Motor_TargetSpeedUSB[MOTOR_RIGHT]) < 10) &&
            (abs(Robot_Motor_TargetSpeedUSB[MOTOR_FLIPPER]) < 10))
            state = LOW_SPEED;

        break;

    case LOW_SPEED:
#ifndef XbeeTest
        set_desired_velocities(REG_MOTOR_VELOCITY.left, REG_MOTOR_VELOCITY.right,
                               REG_MOTOR_VELOCITY.flipper);
#endif
#ifdef XbeeTest
        set_desired_velocities(Xbee_MOTOR_VELOCITY[MOTOR_LEFT], Xbee_MOTOR_VELOCITY[MOTOR_RIGHT],
                               Xbee_MOTOR_VELOCITY[MOTOR_FLIPPER]);
#endif
        // printf("low speed loop!");
        Robot_Motor_TargetSpeedUSB[MOTOR_LEFT] = return_closed_loop_control_effort(MOTOR_LEFT);
        Robot_Motor_TargetSpeedUSB[MOTOR_RIGHT] = return_closed_loop_control_effort(MOTOR_RIGHT);
        Robot_Motor_TargetSpeedUSB[MOTOR_FLIPPER] =
            return_closed_loop_control_effort(MOTOR_FLIPPER);
        // printf("AA:%d,%d,%d",Robot_Motor_TargetSpeedUSB[0],Robot_Motor_TargetSpeedUSB[1],Robot_Motor_TargetSpeedUSB[2]);
        control_loop_counter = 0;
        flipper_control_loop_counter = 0;
#ifndef XbeeTest
        if (REG_MOTOR_SLOW_SPEED == 0)
            state = DECEL_AFTER_LOW_SPEED;
#endif
#ifdef XbeeTest
        if (Xbee_Low_Speed_mode == 0)
            state = DECEL_AFTER_LOW_SPEED;
#endif

        break;

    case DECEL_AFTER_LOW_SPEED:

        set_desired_velocities(0, 0, 0);

        Robot_Motor_TargetSpeedUSB[MOTOR_LEFT] = return_closed_loop_control_effort(MOTOR_LEFT);
        Robot_Motor_TargetSpeedUSB[MOTOR_RIGHT] = return_closed_loop_control_effort(MOTOR_RIGHT);
        Robot_Motor_TargetSpeedUSB[MOTOR_FLIPPER] =
            return_closed_loop_control_effort(MOTOR_FLIPPER);

        // motors are stopped if speed falls below 1%
        if ((abs(Robot_Motor_TargetSpeedUSB[MOTOR_LEFT]) < 10) &&
            (abs(Robot_Motor_TargetSpeedUSB[MOTOR_RIGHT]) < 10) &&
            (abs(Robot_Motor_TargetSpeedUSB[MOTOR_FLIPPER]) < 10))
            state = HIGH_SPEED;

        break;
    }

    // long time no data, clear everything and stop moving
    if (USBTimeOutTimerExpired) {
        // printf("USB Timer Expired!");
        USBTimeOutTimerExpired = false;
        USBTimeOutTimerEnabled = false;
        USBTimeOutTimerCount = 0;
        REG_MOTOR_VELOCITY.left = 0;
        REG_MOTOR_VELOCITY.right = 0;
        REG_MOTOR_VELOCITY.flipper = 0;
#ifdef XbeeTest
        Xbee_MOTOR_VELOCITY[MOTOR_LEFT] = 0;
        Xbee_MOTOR_VELOCITY[MOTOR_RIGHT] = 0;
        Xbee_MOTOR_VELOCITY[MOTOR_FLIPPER] = 0;
#endif

        for (EACH_MOTOR_CHANNEL(i)) {
            Robot_Motor_TargetSpeedUSB[i] = 0;
            Event[i] = Stop;
            TargetParameter[i] = Robot_Motor_TargetSpeedUSB[i];
            // ClearSpeedCtrlData(i);
            // ClearCurrentCtrlData(i);
        }
#ifndef XbeeTest
        // send_debug_uart_string("USB Timeout Detected \r\n",23);
#endif
    }
    // printf("!\n");

#ifdef Xbeetest
    if (USB_New_Data_Received != Xbee_gNewData) {
        USB_New_Data_Received = Xbee_gNewData;
#else
    if (USB_New_Data_Received != gNewData) {
        USB_New_Data_Received = gNewData;
#endif
        // if there is new data coming in, update all the data
        USBTimeOutTimerCount = 0;
        USBTimeOutTimerEnabled = true;
        USBTimeOutTimerExpired = false;
        // printf("MOTOR_LEFT:%d",Robot_Motor_TargetSpeedUSB[0]);
        for (EACH_MOTOR_CHANNEL(i)) {
            if (Robot_Motor_TargetSpeedUSB[i] == 0)
                Event[i] = Stop;
            else if (-1024 < Robot_Motor_TargetSpeedUSB[i] && Robot_Motor_TargetSpeedUSB[i] < 0)
                Event[i] = Back;
            else if (0 < Robot_Motor_TargetSpeedUSB[i] && Robot_Motor_TargetSpeedUSB[i] < 1024)
                Event[i] = Go;

            TargetParameter[i] = Robot_Motor_TargetSpeedUSB[i];
        }
    }
}

//**********************************************

void PinRemap(void) {
    // Unlock Registers
    // clear the bit 6 of OSCCONL to
    // unlock Pin Re-map
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
    // function=pin
    // Assign IC1 To L_Encoder_A
    // RPINR7bits.IC1R = M1_TACHO_RPn;

    // Assign IC3 To Encoder_R1A
    // RPINR8bits.IC3R = M2_TACHO_RPn;

#ifdef XbeeTest
    // Assign U1RX To U1RX, Uart receive channnel
    RPINR18bits.U1RXR = U1RX_RPn;
#endif

    //***************************
    // Configure Output Functions
    // pin->function
    // Assign OC1 To Pin M1_AHI
    M1_PWM = 18; // 18 represents OC1

    // Assign OC2 To Pin M1_BHI
    M2_PWM = 19; // 19 represents OC2

    // Assign OC3 To Pin M2_AHI
    M3_PWM = 20; // 20 represents OC3

    // Assign OC4 To Pin M2_BHI
    // M2_BLO_RPn = 21; //21 represents OC4

    // Assign OC5 To Pin M3_AHI
    // M3_ALO_RPn = 22; //22 represents OC5

    // Assign OC6 To Pin M3_BHI
    // M3_BLO_RPn = 23; //23 represents OC6

    /*
            // Assign I2C Clock (SPI1 Clock Output) to Pin I2C_CLK
            I2C_CLK_RPn = 8; //8 represents SPI1 Clock Output

            // Assign I2C Data (SPI1 Date Output) to Pin I2C_DAT
            I2C_DAT_RPn = 7; //7 represents SPI1 Data Output
    */

#ifdef XbeeTest
    // Assign U1TX To U1TX/Uart Tx
    U1TX_RPn = 3; // 3 represents U1TX
#endif
    //***************************
    // Lock Registers
    //__builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to
    // lock Pin Re-map
}

void Cell_Ctrl(BatteryChannel Channel, BatteryState state) {
    switch (Channel) {
    case Cell_A:
        switch (state) {
        case Cell_ON:
            Cell_A_MOS = 1;
            break;
        case Cell_OFF:
            Cell_A_MOS = 0;
            break;
        }
        break;
    case Cell_B:
        switch (state) {
        case Cell_ON:
            Cell_B_MOS = 1;
            break;
        case Cell_OFF:
            Cell_B_MOS = 0;
            break;
        }
        break;
    }
}

void set_firmware_build_time(void) {

    const unsigned char build_date[12] = __DATE__;
    const unsigned char build_time[12] = __TIME__;
    unsigned int i;

    for (i = 0; i < 12; i++) {
        if (build_date[i] == 0) {
            REG_MOTOR_FIRMWARE_BUILD.data[i] = ' ';
        } else {
            REG_MOTOR_FIRMWARE_BUILD.data[i] = build_date[i];
        }
        if (build_time[i] == 0) {
            REG_MOTOR_FIRMWARE_BUILD.data[i + 12] = ' ';
        } else {
            REG_MOTOR_FIRMWARE_BUILD.data[i + 12] = build_time[i];
        }
    }
}

void MC_Ini(void) // initialzation for the whole program
{
    // make sure we start off in a default state
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

    // peripheral pin selection
    PinRemap();
    // peripheral pin selection end
    //*******************************************
    // initialize I/O port
    // AN15,AN14,AN1,AN0 are all digital

    // initialize all of the analog inputs
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

    // initialize the digital outputs
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

    // I/O initializing complete

    // Initialize motor drivers
    M1_MODE = 1;
    M2_MODE = 1;
    M3_MODE = 1;
    //*******************************************
    InterruptIni();
    // initialize AD
    IniAD();
    // initialize timer
    IniTimer2();
    IniTimer3();
    IniTimer1();
    // 	IniTimer4();
    // 	IniTimer5();
    // initialize PWM sub module
    PWM1Ini();
    PWM2Ini();
    PWM3Ini();
    /*	PWM4Ini();
            PWM5Ini();
            PWM6Ini();
            PWM7Ini();
            PWM8Ini();
            PWM9Ini();*/

    // initialize input capture
    // IniIC1();
    // IniIC3();

    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);

#ifdef XbeeTest
    UART1Ini();
#endif

    set_firmware_build_time();
}

void InterruptIni() {
    // remap all the interrupt routines
    // 	T2InterruptUserFunction=Motor_T2Interrupt;
    T3InterruptUserFunction = Motor_T3Interrupt;
    // 	T4InterruptUserFunction=Motor_T4Interrupt;
    // 	T5InterruptUserFunction=Motor_T5Interrupt;
    // 	IC1InterruptUserFunction=Motor_IC1Interrupt;
    // 	IC3InterruptUserFunction=Motor_IC3Interrupt;
    ADC1InterruptUserFunction = Motor_ADC1Interrupt;
#ifdef XbeeTest
    U1TXInterruptUserFunction = Motor_U1TXInterrupt;
    U1RXInterruptUserFunction = Motor_U1RXInterrupt;
#endif
}

#define FCY 16000000UL // instruction clock
#include "libpic30.h"

void FANCtrlIni() {
    uint8_t a_byte;

    // reset the IC
    a_byte = 0b01011000;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x02, &a_byte));

    // auto fan speed control mode
    // writeI2C2Reg(FAN_CONTROLLER_ADDRESS,0x11,0b00111100);

    // manual fan speed control mode
    a_byte = 0;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x11, &a_byte));
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x12, &a_byte));

    // TODO: this value is unused. Maybe make it right and update in our main I2C loop?
    REG_MOTOR_SIDE_FAN_SPEED = 48;

    // blast fan up to max duty cycle
    a_byte = 240;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &a_byte));

    block_ms(3000);

    // return fan to low duty cycle
    a_byte = 0;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &a_byte));
}

/*****************************************************************************/
//*-----------------------------------Sub system-----------------------------*/

//*-----------------------------------PWM------------------------------------*/

// initialize PWM channel 1
void PWM1Ini(void) {
    // 1. Configure the OCx output for one of the
    // available Peripheral Pin Select pins.
    // done in I/O Init.
    // 2. Calculate the desired duty cycles and load them
    // into the OCxR register.
    OC1R = 0;
    // 3. Calculate the desired period and load it into the
    // OCxRS register.
    OC1RS = 2000;
    // 4. Select the current OCx as the sync source by writing
    // 0x1F to SYNCSEL<4:0> (OCxCON2<4:0>),
    // and clearing OCTRIG (OCxCON2<7>).
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON2bits.OCTRIG = CLEAR;
    // 5. Select a clock source by writing the
    // OCTSEL<2:0> (OCxCON<12:10>) bits.
    OC1CON1bits.OCTSEL = 0b000; // Timer2
    // 6. Enable interrupts, if required, for the timer and
    // output compare modules. The output compare
    // interrupt is required for PWM Fault pin utilization.
    // No interrupt needed
    // 7. Select the desired PWM mode in the OCM<2:0>
    //(OCxCON1<2:0>) bits.
    OC1CON1bits.OCM = 0b110;
    // 8. If a timer is selected as a clock source, set the
    // TMRy prescale value and enable the time base by
    // setting the TON (TxCON<15>) bit.
    // Done in timer Init.
    Period1 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 2
void PWM1Duty(int Duty) { OC1R = Duty * 2; }
//****************************************************

// initialize PWM channel 2
void PWM2Ini(void) {

    OC2R = 0;
    OC2RS = 2000;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.OCTRIG = CLEAR;
    OC2CON1bits.OCTSEL = 0b000; // Timer2
    OC2CON1bits.OCM = 0b110;
    Period2 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 2
void PWM2Duty(int Duty) { OC2R = Duty * 2; }
//****************************************************

//****************************************************
// initialize PWM channel 3
void PWM3Ini(void) {
    OC3R = 0;
    OC3RS = 2000;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.OCTRIG = CLEAR;
    OC3CON1bits.OCTSEL = 0b000; // Timer2
    OC3CON1bits.OCM = 0b110;
    Period3 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 3
void PWM3Duty(int Duty) { OC3R = Duty * 2; }
//****************************************************

//****************************************************
// initialize PWM channel 4
void PWM4Ini(void) {
    OC4R = 0;
    OC4RS = Period30000Hz;
    OC4CON2bits.SYNCSEL = 0x1F;
    OC4CON2bits.OCTRIG = CLEAR;
    OC4CON1bits.OCTSEL = 0b000; // Timer2
    OC4CON1bits.OCM = 0b110;
    Period4 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 4
void PWM4Duty(int Duty) { OC4R = (Period4 * Duty) >> 10; }
//****************************************************

//****************************************************
// initialize PWM channel 5
void PWM5Ini(void) {
    OC5R = 0;
    OC5RS = Period30000Hz;
    OC5CON2bits.SYNCSEL = 0x1F;
    OC5CON2bits.OCTRIG = CLEAR;
    OC5CON1bits.OCTSEL = 0b000; // Timer2
    OC5CON1bits.OCM = 0b110;
    Period5 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 5
void PWM5Duty(int Duty) { OC5R = (Period5 * Duty) >> 10; }
//****************************************************

//****************************************************
// initialize PWM channel 6
void PWM6Ini(void) {
    OC6R = 0;
    OC6RS = Period30000Hz;
    OC6CON2bits.SYNCSEL = 0x1F;
    OC6CON2bits.OCTRIG = CLEAR;
    OC6CON1bits.OCTSEL = 0b000; // Timer2
    OC6CON1bits.OCM = 0b110;
    Period6 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 6
void PWM6Duty(int Duty) { OC6R = (Period6 * Duty) >> 10; }
//****************************************************

//****************************************************
// initialize PWM channel 7
void PWM7Ini(void) {
    OC7R = 0;
    OC7RS = Period30000Hz;
    OC7CON2bits.SYNCSEL = 0x1F;
    OC7CON2bits.OCTRIG = CLEAR;
    OC7CON1bits.OCTSEL = 0b000; // Timer2
    OC7CON1bits.OCM = 0b110;
    Period7 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 7
void PWM7Duty(int Duty) { OC7R = (Period7 * Duty) >> 10; }
//****************************************************

//****************************************************
// initialize PWM channel 8
void PWM8Ini(void) {
    OC8R = 0;
    OC8RS = Period30000Hz;
    OC8CON2bits.SYNCSEL = 0x1F;
    OC8CON2bits.OCTRIG = CLEAR;
    OC8CON1bits.OCTSEL = 0b000; // Timer2
    OC8CON1bits.OCM = 0b110;
    Period8 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 8
void PWM8Duty(int Duty) { OC8R = (Period8 * Duty) >> 10; }
//****************************************************

//****************************************************
// initialize PWM channel 9
void PWM9Ini(void) {
    OC9R = 0;
    OC9RS = Period30000Hz;
    OC9CON2bits.SYNCSEL = 0x1F;
    OC9CON2bits.OCTRIG = CLEAR;
    OC9CON1bits.OCTSEL = 0b000; // Timer2
    OC9CON1bits.OCM = 0b110;
    Period9 = Period30000Hz; // Period is used for all the other PWMxDuty() functions
}

// set duty cycle for PWM channel 9
void PWM9Duty(int Duty) { OC9R = (Period9 * Duty) >> 10; }
//****************************************************

/*****************************************************************************/
//*-----------------------------------Timer----------------------------------*/

void IniTimer1() {
    T1CON = 0x0000;         // clear register
    T1CONbits.TCKPS = 0b00; // 1:1 prescale
    // T1CONbits.TCKPS=0b01;//timer stops,1:8 prescale,
    TMR1 = 0;           // clear timer1 register
    PR1 = Period1000Hz; // interrupt every 1ms
    T1CONbits.TON = SET;
}

void IniTimer2() {
    T2CON = 0x0000;         // stops timer2,16 bit timer,internal clock (Fosc/2)
    T2CONbits.TCKPS = 0b00; // 1:1 prescale
    TMR2 = 0;               // clear timer1 register
    PR2 = Period30000Hz;
    IFS0bits.T2IF = CLEAR; // clear the flag
    // IEC0bits.T2IE=SET;// enable the interrupt
    T2CONbits.TON = SET;
}
void IniTimer3() {
    // timer 3 initialize
    // A/D triger timer, for back emf feedback

    T3CON = 0x0010;        // stops timer3,1:8 prescale,16 bit timer,internal clock (Fosc/2)
    TMR3 = 0;              // clear timer1 register
    PR3 = Period50000Hz;   // timer 3 is 50 times faster than PWM timer (timer 2)
    IFS0bits.T3IF = CLEAR; // clear interrupt flag
    IEC0bits.T3IE = SET;   // enable the interrupt
    T3CONbits.TON = SET;   // start clock

    // end timer 3 initialize
}

void IniTimer4() {
    /*	T4CON=0x0010;//stops timer4,1:8 prescale,16 bit timer,internal clock (Fosc/2)
            TMR4=0;//clear timer1 register
            IFS1bits.T4IF = 0;	//clear interrupt flag
            //IEC1bits.T4IE=SET;
            T4CONbits.TON=SET;*/
}

void IniTimer5() {
    T5CON = 0x0010;        // stops timer3,1:8 prescale,16 bit timer,internal clock (Fosc/2)
    TMR5 = 0;              // clear timer1 register
    PR5 = Period67Hz;      // timer 5 -> 15ms
    IFS1bits.T5IF = CLEAR; // clear interrupt flag
    IEC1bits.T5IE = CLEAR; // disenable the interrupt
    T5CONbits.TON = CLEAR; // stop clock
}

void IniIC1() {
    int temp;
    // 1. Configure the ICx input for one of the available
    // Peripheral Pin Select pins.
    // Done before
    // 2. If Synchronous mode is to be used, disable the
    // sync source before proceeding.
    // No need
    // 3. Make sure that any previous data has been
    // removed from the FIFO by reading ICxBUF until
    // the ICBNE bit (ICxCON1<3>) is cleared.
    while (IC1CON1bits.ICBNE == SET) {
        temp = IC1BUF;
    }
    // 4. Set the SYNCSEL bits (ICxCON2<4:0>) to the
    // desired sync/trigger source.
    IC1CON2bits.SYNCSEL = 0b00000; // not snycronized to anything
    // 5. Set the ICTSEL bits (ICxCON1<12:10>) for the
    // desired clock source.
    IC1CON1bits.ICTSEL = 0b010; // timer 4
    // 6. Set the ICI bits (ICxCON1<6:5>) to the desired
    // interrupt frequency
    IC1CON1bits.ICI = 0b00; // interrupt on every capture event
    // 7. Select Synchronous or Trigger mode operation:
    // a) Check that the SYNCSEL bits are not set to?0000?
    /*
            if(IC5CON2bits.SYNCSEL==CLEAR)
            {
                    IC5CON2bits.SYNCSEL=0b10100; 	//sync with IC1
            }
    */
    // b) For Synchronous mode, clear the ICTRIG
    // bit (ICxCON2<7>).
    IC1CON2bits.ICTRIG = 0b0; // synchronous mode
    // c) For Trigger mode, set ICTRIG, and clear the
    // TRIGSTAT bit (ICxCON2<6>).

    // 8. Set the ICM bits (ICxCON1<2:0>) to the desired
    // operational mode.
    IC1CON1bits.ICM = 0b011; // capture on every rising edge
    // 9. Enable the selected trigger/sync source.

    IFS0bits.IC1IF = CLEAR; // clear the interrupt flag
    IEC0bits.IC1IE = SET;   // start the interrupt
}

void IniIC3() {
    int temp;
    // 1. Configure the ICx input for one of the available
    // Peripheral Pin Select pins.
    // Done before
    // 2. If Synchronous mode is to be used, disable the
    // sync source before proceeding.
    // No need
    // 3. Make sure that any previous data has been
    // removed from the FIFO by reading ICxBUF until
    // the ICBNE bit (ICxCON1<3>) is cleared.
    while (IC3CON1bits.ICBNE == SET) {
        temp = IC3BUF;
    }
    // 4. Set the SYNCSEL bits (ICxCON2<4:0>) to the
    // desired sync/trigger source.
    IC3CON2bits.SYNCSEL = 0b00000; // not snycronized to anything
    // 5. Set the ICTSEL bits (ICxCON1<12:10>) for the
    // desired clock source.
    IC3CON1bits.ICTSEL = 0b010; // timer 4
    // 6. Set the ICI bits (ICxCON1<6:5>) to the desired
    // interrupt frequency
    IC3CON1bits.ICI = 0b00; // interrupt on every capture event
    // 7. Select Synchronous or Trigger mode operation:
    // a) Check that the SYNCSEL bits are not set to?0000?
    /*
            if(IC5CON2bits.SYNCSEL==CLEAR)
            {
                    IC5CON2bits.SYNCSEL=0b10100; 	//sync with IC1
            }
    */
    // b) For Synchronous mode, clear the ICTRIG
    // bit (ICxCON2<7>).
    IC3CON2bits.ICTRIG = 0b0; // synchronous mode
    // c) For Trigger mode, set ICTRIG, and clear the
    // TRIGSTAT bit (ICxCON2<6>).

    // 8. Set the ICM bits (ICxCON1<2:0>) to the desired
    // operational mode.
    IC3CON1bits.ICM = 0b011; // capture on every rising edge
    // 9. Enable the selected trigger/sync source.

    IFS2bits.IC3IF = CLEAR; // clear the interrupt flag
    IEC2bits.IC3IE = SET;   // start the interrupt
}

/*****************************************************************************/

/*****************************************************************************/
//*-----------------------------------A/D------------------------------------*/
void IniAD() {
    // 1. Configure the A/D module:
    // a) Configure port pins as analog inputs and/or
    // select band gap reference inputs (AD1PCFGL<15:0> and AD1PCFGH<1:0>).
    // none
    // b) Select voltage reference source to match
    // expected range on analog inputs (AD1CON2<15:13>).
    AD1CON2bits.VCFG = 0b000; // VR+: AVDD, VR-: AVSS
    // c) Select the analog conversion clock to
    // match desired data rate with processor clock (AD1CON3<7:0>).
    AD1CON3bits.ADCS = 0b00000001; // TAD= 2TCY = 125 ns, at least 75ns required

    // d) Select the appropriate sample
    // conversion sequence (AD1CON1<7:5> and AD1CON3<12:8>).
    AD1CON1bits.SSRC = 0b111;   // auto conversion
    AD1CON3bits.SAMC = 0b01111; // auto-sample time=15*TAD=1.875uS
    // e) Select how conversion results are
    // presented in the buffer (AD1CON1<9:8>).
    AD1CON1bits.FORM = 0b00; // integer (0000 00dd dddd dddd)
    // f) Select interrupt rate (AD1CON2<5:2>).
    AD1CON2bits.SMPI = 0b1011; // interrupt every 12 samples convert sequence
    // g) scan mode, select input channels (AD1CSSL<15:0>)
    // AD1CSSL=0b0011111100111111;
    AD1CSSL = 0b1111111100001111;
    AD1CON2bits.CSCNA = SET;
    // h) Turn on A/D module (AD1CON1<15>).
    AD1CON1bits.ADON = SET;
    // 2. Configure A/D interrupt (if required):
    IEC0bits.AD1IE = SET;
    // a) Clear the AD1IF bit.
    IFS0bits.AD1IF = CLEAR;
    // b) Select A/D interrupt priority.
}

/*****************************************************************************/

/*****************************************************************************/
//*-----------------------------------UART------------------------------------*/
void UART1Ini() {
    // Write appropriate baud rate value to the UxBRG register.
    U1BRG = BaudRate_57600_HI;
    // Enable the UART.
    U1MODE = 0x0000;
    // hight speed mode
    U1MODEbits.BRGH = 1;
    U1STA = 0x0000;
    U1MODEbits.UARTEN = 1; // UART1 is enabled
    U1STAbits.UTXEN = 1;   // transmit enabled
    IFS0bits.U1TXIF = 0;   // clear the transmit flag
    IEC0bits.U1TXIE = 1;   // enable UART1 transmit interrupt
    IEC0bits.U1RXIE = 1;   // enable UART1 receive interrupt
}

/*****************************************************************************/

/***************************Interrupt routines***********************************/

void Motor_IC1Interrupt(void) {
    // int temp,index;

    IFS0bits.IC1IF = 0; // clear the interrupt flag
    // make sure pull all the data from the buffer
    while (IC1CON1bits.ICBNE == SET) {
        LEncoderCurrentValue = IC1BUF;
        EncoderFBInterval[MOTOR_LEFT][EncoderFBIntervalPointer[MOTOR_LEFT]] =
            LEncoderCurrentValue - LEncoderLastValue;
        if (EncoderFBInterval[MOTOR_LEFT][EncoderFBIntervalPointer[MOTOR_LEFT]] < 0 ||
            LEncoderAOverFlowCount > 0) {
            EncoderFBInterval[MOTOR_LEFT][EncoderFBIntervalPointer[MOTOR_LEFT]] +=
                65535 * LEncoderAOverFlowCount;
            LEncoderAOverFlowCount = 0;
        }
        LEncoderLastValue = LEncoderCurrentValue;
        EncoderFBIntervalPointer[MOTOR_LEFT] =
            (EncoderFBIntervalPointer[MOTOR_LEFT] + 1) % SAMPLE_LENGTH;
    }

    Encoder_Interrupt_Counter[MOTOR_LEFT]++;
}

void Motor_IC3Interrupt(void) {
    IFS2bits.IC3IF = 0; // clear the flag
    // make sure pull all the data from the buffer
    while (IC3CON1bits.ICBNE == SET) {
        REncoderCurrentValue = IC3BUF;
        EncoderFBInterval[MOTOR_RIGHT][EncoderFBIntervalPointer[MOTOR_RIGHT]] =
            REncoderCurrentValue - REncoderLastValue;
        if (EncoderFBInterval[MOTOR_RIGHT][EncoderFBIntervalPointer[MOTOR_RIGHT]] < 0 ||
            REncoderAOverFlowCount > 0) {
            EncoderFBInterval[MOTOR_RIGHT][EncoderFBIntervalPointer[MOTOR_RIGHT]] +=
                65535 * REncoderAOverFlowCount;
            REncoderAOverFlowCount = 0;
        }
        REncoderLastValue = REncoderCurrentValue;
        EncoderFBIntervalPointer[MOTOR_RIGHT] =
            (EncoderFBIntervalPointer[MOTOR_RIGHT] + 1) % SAMPLE_LENGTH;
    }

    Encoder_Interrupt_Counter[MOTOR_RIGHT]++;
}

void Motor_T4Interrupt(void) {
    IFS1bits.T4IF = 0; // clear interrupt flag
    ICLMotorOverFlowCount++;
    ICRMotorOverFlowCount++;
    LEncoderAOverFlowCount++;
    LEncoderBOverFlowCount++;
    REncoderAOverFlowCount++;
    REncoderBOverFlowCount++;
}

void Motor_T2Interrupt(void) {

    // hardcode removed
    // 	TRISFbits.TRISF1=0;
    // 	PORTFbits.RF1=~PORTFbits.RF1;
    IFS0bits.T2IF = 0; // clear interrupt flag
    // T3CONbits.TON=SET;
}

void Motor_T3Interrupt(void) {
    int temp;
    // PORTCbits.RC13=~PORTCbits.RC13;
    // clear timer3 flage
    IFS0bits.T3IF = CLEAR; // clear interrupt flag
    temp = TMR2;
    Timer3Count++;

    // TODO: turn on timer

    if (Timer3Count >= 0) {
        Timer3Count = 0;
        AD1CON1bits.ASAM = SET;
    }
}

void Motor_T5Interrupt(void) {
    MotorChannel i;
    IFS1bits.T5IF = CLEAR; // clear interrupt flag
    Timer5Count++;
    if (Timer5Count >= 3) // slow down the whole system within 45 ms
    {
        IEC1bits.T5IE = CLEAR; // disable the interrupt
        T5CONbits.TON = CLEAR; // stop clock
        TMR5 = 0;              // clear the timer register
        Timer5Count = 0;
    }
    if (!OverCurrent && TotalCurrent >= CurrentLimit) {
        OverCurrent = true;
        MotorOffTimerEnabled = true;
        MotorOffTimerExpired = false;
        MotorOffTimerCount = 0;
        for (EACH_MOTOR_CHANNEL(i)) {
            ClearSpeedCtrlData(i);
            ClearCurrentCtrlData(i);
        }
        // printf("\n Total current:%ld,\n",TotalCurrent);
        // coast left motor
        M1_COAST = Set_ActiveLO;
        PWM1Duty(0);
        M1_BRAKE = Clear_ActiveLO;
        // coast right motor
        M2_COAST = Set_ActiveLO;
        PWM2Duty(0);
        M2_BRAKE = Clear_ActiveLO;
        // coast flipper
        M3_COAST = Set_ActiveLO;
        PWM1Duty(0);
        M3_BRAKE = Clear_ActiveLO;
    }
}

void Motor_ADC1Interrupt(void) {
    //	unsigned int temp = 0;
    // stop the conversion
    AD1CON1bits.ASAM = CLEAR;

    // clear the flag
    IFS0bits.AD1IF = CLEAR;
    // load the value

    // 		BackEMFSampleEnabled=false;

    M3_POSFB_Array[0][M3_POSFB_ArrayPointer] = ADC1BUF4;
    M3_POSFB_Array[1][M3_POSFB_ArrayPointer] = ADC1BUF5;
    MotorCurrentAD[MOTOR_LEFT][MotorCurrentADPointer] = ADC1BUF3;
    MotorCurrentAD[MOTOR_RIGHT][MotorCurrentADPointer] = ADC1BUF1;
    MotorCurrentAD[MOTOR_FLIPPER][MotorCurrentADPointer] = ADC1BUFB;
    Cell_A_Current[Total_Cell_Current_ArrayPointer] = ADC1BUF8;
    Cell_B_Current[Total_Cell_Current_ArrayPointer] = ADC1BUF9;
    Total_Cell_Current_Array[Total_Cell_Current_ArrayPointer] =
        Cell_A_Current[Total_Cell_Current_ArrayPointer] +
        Cell_B_Current[Total_Cell_Current_ArrayPointer];

    CellVoltageArray[Cell_A][CellVoltageArrayPointer] = ADC1BUF6;
    CellVoltageArray[Cell_B][CellVoltageArrayPointer] = ADC1BUF7;
    TotalCurrent = MotorCurrentAD[MOTOR_LEFT][MotorCurrentADPointer] +
                   MotorCurrentAD[MOTOR_RIGHT][MotorCurrentADPointer] +
                   MotorCurrentAD[MOTOR_FLIPPER][MotorCurrentADPointer];

    // increase array pointer, prevent over flow
    CellVoltageArrayPointer = (CellVoltageArrayPointer + 1) % SAMPLE_LENGTH;
    Total_Cell_Current_ArrayPointer = (Total_Cell_Current_ArrayPointer + 1) % SAMPLE_LENGTH;
    M3_POSFB_ArrayPointer = (M3_POSFB_ArrayPointer + 1) % SAMPLE_LENGTH;
    MotorCurrentADPointer = (MotorCurrentADPointer + 1) % SAMPLE_LENGTH;

#ifdef XbeeTest
    EncoderInterval[MOTOR_LEFT] = IC_period(kIC01);    // left motor encoder time interval
    EncoderInterval[MOTOR_RIGHT] = IC_period(kIC02);   // right motor encoder time interval
    EncoderInterval[MOTOR_FLIPPER] = IC_period(kIC03); // Encoder motor encoder time interval
    // if(Xbee_Incoming_Cmd[0]== P1_Read_Register && U1STAbits.UTXBF==0 &&
    // XbeeTest_UART_BufferPointer==0 &&
    // (Xbee_MOTOR_VELOCITY[0]||Xbee_MOTOR_VELOCITY[1]||Xbee_MOTOR_VELOCITY[2])!=0)//if transmit reg
    // is empty and last packet is sent
    if (Xbee_Incoming_Cmd[0] == P1_Read_Register && U1STAbits.UTXBF == 0 &&
        XbeeTest_UART_BufferPointer == 0) // if transmit reg is empty and last packet is sent
    {
        U1TXREG = Xbee_StartBit; // send out the index
        XbeeTest_UART_DataNO = Xbee_Incoming_Cmd[1];
        XbeeTest_UART_Buffer[0] = XbeeTest_UART_DataNO;

// CASE(n, REGISTER) populates the UART output buffer with the 16-bit integer value of the given
// register and breaks out of the switch statement
#define CASE(n, REGISTER)                                                                          \
    case (n):                                                                                      \
        XbeeTest_UART_Buffer[1] = (uint8_t)((REGISTER) >> 8 & 0xff);                               \
        XbeeTest_UART_Buffer[2] = (uint8_t)(REGISTER & 0xff);                                      \
        break;

        switch (XbeeTest_UART_DataNO) {
            CASE(0, REG_PWR_TOTAL_CURRENT)
            CASE(2, REG_MOTOR_FB_RPM.left)
            CASE(4, REG_MOTOR_FB_RPM.right)
            CASE(6, REG_FLIPPER_FB_POSITION.pot1)
            CASE(8, REG_FLIPPER_FB_POSITION.pot2)
            CASE(10, REG_MOTOR_FB_CURRENT.left)
            CASE(12, REG_MOTOR_FB_CURRENT.right)
            CASE(14, REG_MOTOR_ENCODER_COUNT.left)
            CASE(16, REG_MOTOR_ENCODER_COUNT.right)
        case 18: // 18-REG_MOTOR_FAULT_FLAG
            XbeeTest_UART_Buffer[1] = REG_MOTOR_FAULT_FLAG.left;
            XbeeTest_UART_Buffer[2] = REG_MOTOR_FAULT_FLAG.right;
            break;
            CASE(20, REG_MOTOR_TEMP.left)
            CASE(22, REG_MOTOR_TEMP.right)
            CASE(24, REG_PWR_BAT_VOLTAGE.a)
            CASE(26, REG_PWR_BAT_VOLTAGE.b)
            CASE(28, EncoderInterval[MOTOR_LEFT])
            CASE(30, EncoderInterval[MOTOR_RIGHT])
            CASE(32, EncoderInterval[MOTOR_FLIPPER])
            CASE(34, REG_ROBOT_REL_SOC_A)
            CASE(36, REG_ROBOT_REL_SOC_B)
            CASE(38, REG_MOTOR_CHARGER_STATE)
            CASE(40, BuildNO)
            CASE(42, REG_PWR_A_CURRENT)
            CASE(44, REG_PWR_B_CURRENT)
            CASE(46, REG_MOTOR_FLIPPER_ANGLE)
            CASE(48, Xbee_SIDE_FAN_SPEED)
            CASE(50, Xbee_Low_Speed_mode)
            CASE(52, REG_BATTERY_STATUS_A)
            CASE(54, REG_BATTERY_STATUS_B)
            CASE(56, REG_BATTERY_MODE_A)
            CASE(58, REG_BATTERY_MODE_B)
            CASE(60, REG_BATTERY_TEMP_A)
            CASE(62, REG_BATTERY_TEMP_B)
            CASE(64, REG_BATTERY_VOLTAGE_A)
            CASE(66, REG_BATTERY_VOLTAGE_B)
            CASE(68, REG_BATTERY_CURRENT_A)
            CASE(70, REG_BATTERY_CURRENT_B)

        default:
            XbeeTest_UART_Buffer[0] = 0;
            XbeeTest_UART_Buffer[1] = 0;
            break;
        }
#undef CASE
        // add checksum of the package
        XbeeTest_UART_Buffer[3] =
            255 -
            (XbeeTest_UART_Buffer[0] + XbeeTest_UART_Buffer[1] + XbeeTest_UART_Buffer[2]) % 255;
        // clear incoming command
        Xbee_Incoming_Cmd[0] = 0;
        Xbee_Incoming_Cmd[1] = 0;
    }
#endif
}

void Motor_U1TXInterrupt(void) {
    // clear the flag
    IFS0bits.U1TXIF = 0;

#ifdef XbeeTest
    // transmit data
    if (XbeeTest_UART_BufferPointer < XbeeTest_UART_Buffer_Length) {
#ifdef XbeeTest_TX_Enable
        U1TXREG = XbeeTest_UART_Buffer[XbeeTest_UART_BufferPointer];
        XbeeTest_UART_BufferPointer++;
#endif
    } else {
        XbeeTest_UART_BufferPointer = 0;
    }
#endif
}

void Motor_U1RXInterrupt(void) {
    // clear the flag
    IFS0bits.U1RXIF = 0;
    // XbeeTest code only
#ifdef XbeeTest
    int i = 0;
    int SumBytes = 0;
    XbeeTest_Temp = U1RXREG;
    if (XbeeTest_State == XbeeTest_StateIdle) {
        if (XbeeTest_Temp == Xbee_StartBit) {
            XbeeTest_State = XbeeTest_StateProcessing;
        }
    } else if (XbeeTest_State == XbeeTest_StateProcessing) {
        XbeeTest_Buffer[XbeeTest_BufferArrayPointer] = XbeeTest_Temp;
        XbeeTest_BufferArrayPointer++;
        if (XbeeTest_BufferArrayPointer >= XbeeTest_BufferLength) // end of the package
        {
            // if all bytes add up together equals 255, it is a good pack, then process
            SumBytes = 0;
            for (i = 0; i < XbeeTest_BufferLength; i++) {
                // printf("%d:%d  ",i,XbeeTest_Buffer[i]);
                SumBytes += XbeeTest_Buffer[i];
            }
            // printf("Sum:%d\n",SumBytes);
            if (SumBytes % 255 == 0) {
                // input data range 0~250, 125 is stop, 0 is backwards full speed, 250 is forward
                // full speed for Marge, the right and left is flipped, so left and right need to be
                // flipped
                Xbee_MOTOR_VELOCITY[MOTOR_LEFT] = (XbeeTest_Buffer[0] * 8 - 1000);
                Xbee_MOTOR_VELOCITY[MOTOR_RIGHT] = (XbeeTest_Buffer[1] * 8 - 1000);
                Xbee_MOTOR_VELOCITY[MOTOR_FLIPPER] = (XbeeTest_Buffer[2] * 8 - 1000);
                Xbee_Incoming_Cmd[0] = XbeeTest_Buffer[3];
                Xbee_Incoming_Cmd[1] = XbeeTest_Buffer[4];
                // see if this is a fan cmd
                switch (Xbee_Incoming_Cmd[0]) {
                case P1_Fan_Command: // new fan command coming in
                    Xbee_SIDE_FAN_SPEED = Xbee_Incoming_Cmd[1];
                    Xbee_SIDE_FAN_NEW = 1;
                    // Enable fan speed timer
                    Xbee_FanSpeedTimerEnabled = true;
                    Xbee_FanSpeedTimerCount = 0;
                    // printf("fan data received!");
                    break;
                case P1_Low_Speed_Set:
                    Xbee_Low_Speed_mode = Xbee_Incoming_Cmd[1];
                    break;
                case P1_Calibration_Flipper:
                    if (Xbee_Incoming_Cmd[1] == P1_Calibration_Flipper) {
                        Xbee_Calibration = 1;
                    }
                    break;
                }
                // REG_MOTOR_VELOCITY.left=300;
                // REG_MOTOR_VELOCITY.right=800;
                // REG_MOTOR_VELOCITY.flipper=400;
                // printf("Motor Speed Set!!\n");
                // printf("Receive New Package! Xbee_gNewData=%d",Xbee_gNewData);
                // printf("L:%d,R:%d,F:%d/n",Xbee_MOTOR_VELOCITY[0],Xbee_MOTOR_VELOCITY[1],Xbee_MOTOR_VELOCITY[2]);
                Xbee_gNewData = !Xbee_gNewData;
            }
            // clear all the local buffer
            XbeeTest_BufferArrayPointer = 0;
            XbeeTest_State = XbeeTest_StateIdle;
        }
    }
#endif
    // XbeeTest code ends
}

void initialize_i2c2_registers(void) {
    REG_MOTOR_TEMP.left = 255;
    REG_MOTOR_TEMP.right = 255;
    REG_MOTOR_TEMP.board = 255;
    REG_ROBOT_REL_SOC_A = 255;
}

void initialize_i2c3_registers(void) { REG_ROBOT_REL_SOC_B = 255; }

static unsigned int return_calibrated_pot_angle(unsigned int pot_1_value,
                                                unsigned int pot_2_value) {
    unsigned int combined_pot_angle = 0;
    int calibrated_pot_angle = 0;

    combined_pot_angle = return_combined_pot_angle(pot_1_value, pot_2_value);

    // special case -- invalid reading
    if (combined_pot_angle == 10000)
        return 10000;
    else if (combined_pot_angle == 0xffff)
        return 0xffff;

    // if calibration didn't work right, return angle with no offset
    if (flipper_angle_offset == 0xffff)
        return combined_pot_angle;

    calibrated_pot_angle = wrap_angle(combined_pot_angle - flipper_angle_offset);

    return calibrated_pot_angle;
}

static unsigned int return_combined_pot_angle(unsigned int pot_1_value, unsigned int pot_2_value) {
    // unsigned int pot_1_value, pot_2_value = 0;
    int combined_pot_angle = 0;
    int pot_angle_1 = 0;
    int pot_angle_2 = 0;
    int temp1 = 0;
    int temp2 = 0;
    float scale_factor = 0;
    int temp_pot1_value = 0;

    // correct for pot 2 turning the opposite direction
    pot_2_value = 1023 - pot_2_value;

    // if both pot values are invalid
    if (((pot_1_value < LOW_POT_THRESHOLD) || (pot_1_value > HIGH_POT_THRESHOLD)) &&
        ((pot_2_value < LOW_POT_THRESHOLD) || (pot_2_value > HIGH_POT_THRESHOLD))) {
        return 0xffff;
    }
    // if pot 1 is out of linear range
    else if ((pot_1_value < LOW_POT_THRESHOLD) || (pot_1_value > HIGH_POT_THRESHOLD)) {
        // 333.3 degrees, 1023 total counts, 333.3/1023 = .326
        combined_pot_angle = pot_2_value * .326 + 13.35;
    }
    // if pot 2 is out of linear range
    else if ((pot_2_value < LOW_POT_THRESHOLD) || (pot_2_value > HIGH_POT_THRESHOLD)) {
        // 333.3 degrees, 1023 total counts, 333.3/1023 = .326
        // 13.35 degrees + 45 degrees = 58.35 degrees
        combined_pot_angle = (int)pot_1_value * .326 + 13.35 + FLIPPER_POT_OFFSET;

    }
    // if both pot 1 and pot 2 values are valid
    else {

        // figure out which one is closest to the end of range
        temp1 = pot_1_value - 512;
        temp2 = pot_2_value - 512;

        // offset, so that both pot values should be the same
        // FLIPPER_POT_OFFSET/333.33*1023 = 168.8 for 55 degrees
        temp_pot1_value = pot_1_value - 168.8;

        pot_angle_1 = wrap_angle(temp_pot1_value * .326 + 13.35);
        pot_angle_2 = wrap_angle(pot_2_value * .326 + 13.35);

        // if pot1 is closer to the end of range
        if (abs(temp1) > abs(temp2)) {
            scale_factor = (512 - abs(temp1)) / 512.0;
            // combined_pot_value = (pot_1_value*scale_factor + pot_2_value*(1-scale_factor));
            combined_pot_angle = pot_angle_1 * scale_factor + pot_angle_2 * (1 - scale_factor);

        }
        // if pot2 is closer to the end of range
        else {

            scale_factor = (512 - abs(temp2)) / 512.0;
            // combined_pot_value = (pot_2_value*scale_factor + pot_1_value*(1-scale_factor));
            combined_pot_angle = pot_angle_2 * scale_factor + pot_angle_1 * (1 - scale_factor);
        }

        // 333.3 degrees, 1023 total counts, 333.3/1023 = .326
        // combined_pot_angle = combined_pot_value*.326+13.35;
    }

    combined_pot_angle = wrap_angle(combined_pot_angle);

    return (unsigned int)combined_pot_angle;
}

// read stored values from flash memory
static void read_stored_angle_offset(void) {
    unsigned char angle_data[3];
    unsigned int i;
    DataEEInit();
    Nop();
    for (i = 0; i < 3; i++) {
        angle_data[i] = DataEERead(i);
        Nop();
    }

    // only use stored values if we have stored the calibrated values before.
    // we store 0xaa in position 8 so we know that the calibration has taken place.
    if (angle_data[2] == 0xaa) {
        flipper_angle_offset = angle_data[0] * 256 + angle_data[1];
    }
}

void calibrate_flipper_angle_sensor(void) {
    flipper_angle_offset =
        return_combined_pot_angle(REG_FLIPPER_FB_POSITION.pot1, REG_FLIPPER_FB_POSITION.pot2);

    // set motor speeds to 0
    PWM1Duty(0);
    PWM2Duty(0);
    PWM3Duty(0);

    DataEEInit();
    Nop();

    DataEEWrite((flipper_angle_offset >> 8), 0);
    Nop();
    DataEEWrite((flipper_angle_offset & 0xff), 1);
    Nop();

    // write 0xaa to index 2, so we can tell if this robot has been calibrated yet
    DataEEWrite(0xaa, 2);
    Nop();

    // don't do anything again ever
    while (1) {
        ClrWdt();
    }
}

void turn_on_power_bus_new_method(void) {
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int k = 2000;
    unsigned int k0 = 2000;

    for (i = 0; i < 300; i++) {

        Cell_Ctrl(Cell_A, Cell_ON);
        Cell_Ctrl(Cell_B, Cell_ON);
        for (j = 0; j < k; j++)
            Nop();
        Cell_Ctrl(Cell_A, Cell_OFF);
        Cell_Ctrl(Cell_B, Cell_OFF);

        block_ms(10);

        k = k0 + i * i / 4;
        // k+=10;
    }

    Cell_Ctrl(Cell_A, Cell_ON);
    Cell_Ctrl(Cell_B, Cell_ON);
}

void turn_on_power_bus_old_method(void) {

    unsigned int i;

    for (i = 0; i < 20; i++) {
        Cell_Ctrl(Cell_A, Cell_ON);
        Cell_Ctrl(Cell_B, Cell_ON);
        block_ms(10);
        Cell_Ctrl(Cell_A, Cell_OFF);
        Cell_Ctrl(Cell_B, Cell_OFF);
        block_ms(40);
    }

    Cell_Ctrl(Cell_A, Cell_ON);
    Cell_Ctrl(Cell_B, Cell_ON);
}

void turn_on_power_bus_hybrid_method(void) {

    unsigned int i;
    unsigned int j;
    // k=20,000 is about 15ms
    unsigned int k = 2000;
    unsigned int k0 = 2000;
    unsigned int l = 0;

    for (i = 0; i < 200; i++) {

        for (l = 0; l < 3; l++) {
            Cell_Ctrl(Cell_A, Cell_ON);
            Cell_Ctrl(Cell_B, Cell_ON);
            for (j = 0; j < k; j++)
                Nop();
            Cell_Ctrl(Cell_A, Cell_OFF);
            Cell_Ctrl(Cell_B, Cell_OFF);
            break;
            if (i > 20)
                break;

            block_ms(10);
        }

        block_ms(40);
        // k+=10;
        k = k0 + i * i / 4;
        /*if(i<10000)
        {
          k = k0+(i*i)/4;
        }
        else
        {
          k+=50;
        } */
        // k = 3000;
        // k+=50;
        if (k > 20000)
            k = 20000;
    }

    Cell_Ctrl(Cell_A, Cell_ON);
    Cell_Ctrl(Cell_B, Cell_ON);
}

void handle_power_bus(void) {
#define BATTERY_DATA_LEN 20
    unsigned char battery_data1[BATTERY_DATA_LEN], battery_data2[BATTERY_DATA_LEN];
    unsigned char DEVICE_NAME_OLD_BATTERY[7] = {'B', 'B', '-', '2', '5', '9', '0'};
    unsigned char DEVICE_NAME_NEW_BATTERY[9] = {'B', 'T', '-', '7', '0', '7', '9', '1', 'B'};
    unsigned char DEVICE_NAME_BT70791_CK[9] = {'B', 'T', '-', '7', '0', '7', '9', '1', 'C'};
    unsigned char DEVICE_NAME_CUSTOM_BATTERY[7] = {'R', 'O', 'B', 'O', 'T', 'E', 'X'};
    unsigned int j;
    I2CResult result;
    I2COperationDef op;

    // enable outputs for power bus
    CELL_A_MOS_EN(1);
    CELL_B_MOS_EN(1);

    // initialize i2c buses
    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);

    for (j = 0; j < 3; j++) {
        // Read "Device Name" from battery
        op = i2c_op_read_block(BATTERY_ADDRESS, 0x21, battery_data1, BATTERY_DATA_LEN);
        result = i2c_synchronously_await(I2C_BUS2, op);
        if (result == I2C_OKAY)
            break;
        op = i2c_op_read_block(BATTERY_ADDRESS, 0x21, battery_data2, BATTERY_DATA_LEN);
        result = i2c_synchronously_await(I2C_BUS3, op);
        if (result == I2C_OKAY)
            break;
    }

    // If we're using the old battery (BB-2590)
    if (check_string_match(DEVICE_NAME_OLD_BATTERY, battery_data1,
                           sizeof(DEVICE_NAME_OLD_BATTERY)) ||
        check_string_match(DEVICE_NAME_OLD_BATTERY, battery_data2,
                           sizeof(DEVICE_NAME_OLD_BATTERY))) {
        send_debug_uart_string("BATTERY:  BB-2590\r\n", 19);
        block_ms(10);
        turn_on_power_bus_old_method();
        return;
    }

    // If we're using the new battery (BT-70791B)
    if (check_string_match(DEVICE_NAME_NEW_BATTERY, battery_data1,
                           sizeof(DEVICE_NAME_NEW_BATTERY)) ||
        check_string_match(DEVICE_NAME_NEW_BATTERY, battery_data2,
                           sizeof(DEVICE_NAME_NEW_BATTERY))) {
        send_debug_uart_string("BATTERY:  BT-70791B\r\n", 21);
        block_ms(10);
        turn_on_power_bus_new_method();
        return;
    }

    // If we're using Bren-Tronics BT-70791C
    if (check_string_match(DEVICE_NAME_BT70791_CK, battery_data1, sizeof(DEVICE_NAME_BT70791_CK)) ||
        check_string_match(DEVICE_NAME_BT70791_CK, battery_data2, sizeof(DEVICE_NAME_BT70791_CK))) {
        send_debug_uart_string("BATTERY:  BT-70791C\r\n", 21);
        block_ms(10);
        turn_on_power_bus_new_method();
        return;
    }

    // if we're using the low lithium custom Matthew's battery
    if (check_string_match(DEVICE_NAME_CUSTOM_BATTERY, battery_data1,
                           sizeof(DEVICE_NAME_CUSTOM_BATTERY)) ||
        check_string_match(DEVICE_NAME_CUSTOM_BATTERY, battery_data2,
                           sizeof(DEVICE_NAME_CUSTOM_BATTERY))) {
        send_debug_uart_string("BATTERY:  ROBOTEX\r\n", 19);
        block_ms(10);
        turn_on_power_bus_old_method();
        return;
    }

    // if we're using an unknown battery
    send_debug_uart_string("UNKNOWN BATTERY\r\n", 17);
    block_ms(10);
    send_debug_uart_string((char *)battery_data1, 20);
    block_ms(10);

    turn_on_power_bus_hybrid_method();
}

bool check_string_match(unsigned char *string1, unsigned char *string2, unsigned char length) {
    unsigned int i;
    for (i = 0; i < length; i++) {
        if (string1[i] != string2[i])
            return false;
    }

    return true;
}

// When robot is on the charger, only leave one side of the power bus on at a time.
// This is so that one side of a battery doesn't charge the other side through the power bus.
static void alternate_power_bus(void) {
    static unsigned int alternate_counter = 0;
    static unsigned int toggle_state = 0;

    if (REG_MOTOR_CHARGER_STATE == 0xdada) {

        // if we're on the dock, immediately turn off one side of the power bus
        if (toggle_state == 0) {
            toggle_state = 1;
            Cell_Ctrl(Cell_B, Cell_ON);
            Cell_Ctrl(Cell_A, Cell_OFF);
        }

        alternate_counter++;

        if (alternate_counter > 10000) {
            alternate_counter = 0;

            // turn off side B, turn on side A
            if (toggle_state == 1) {
                toggle_state = 2;
                Cell_Ctrl(Cell_A, Cell_ON);
                Cell_Ctrl(Cell_B, Cell_OFF);
            }
            // turn on side B, turn off side A
            else if (toggle_state == 2) {
                toggle_state = 1;
                Cell_Ctrl(Cell_B, Cell_ON);
                Cell_Ctrl(Cell_A, Cell_OFF);
            } else {
                toggle_state = 0;
            }
        }
    }
    // robot has left the charger -- turn both cells on
    else {
        Cell_Ctrl(Cell_A, Cell_ON);
        Cell_Ctrl(Cell_B, Cell_ON);
        toggle_state = 0;
    }
}
