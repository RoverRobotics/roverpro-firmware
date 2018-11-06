#include <p24Fxxxx.h>
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "interrupt_switch.h"
#include "testing.h"
#include "device_robot_motor_i2c.h"
#include "DEE Emulation 16-bit.h"
#include "i2clib.h"
#include "device_robot_motor_loop.h"
#include "InputCapture.h"

#define UART_CONTROL
#define BATProtectionON

// variables
// sub system variables
static long int Period1;
static long int Period2;
static long int Period3;
//****************************************************

MotorEvent Event[MOTOR_CHANNEL_COUNT] = {Stop, Stop, Stop};
MotorState StateLevel01[MOTOR_CHANNEL_COUNT] = {Protection, Protection, Protection};
MotorState2 StateLevel02[MOTOR_CHANNEL_COUNT] = {Locked, Locked, Locked};

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
bool CurrentFBTimerExpired = false;
bool CurrentFBTimerEnabled = true;
int CurrentFBTimerCount = 0;
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
bool uart_FanSpeedTimerEnabled = false;
bool uart_fan_speed_expired = false;
int uart_FanSpeedTimerCount = 0;

uint16_t MotorCurrentAD[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH] = {{0}};
int MotorCurrentADPointer = 0;
uint16_t Current4Control[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH_CONTROL] = {{0}};
int Current4ControlPointer[MOTOR_CHANNEL_COUNT] = {0};
uint16_t ControlCurrent[MOTOR_CHANNEL_COUNT] = {0};

int16_t motor_target_speed[MOTOR_CHANNEL_COUNT] = {0};

int Timer3Count = 0;
int M3_POSFB_Array[2][SAMPLE_LENGTH] = {{0}}; ///< Flipper Motor positional feedback data
int M3_POSFB_ArrayPointer = 0;
int Total_Cell_Current_Array[SAMPLE_LENGTH] = {0};
int Total_Cell_Current_ArrayPointer = 0;
int CellVoltageArray[BATTERY_CHANNEL_COUNT][SAMPLE_LENGTH];
int CellVoltageArrayPointer = 0;
int Cell_A_Current[SAMPLE_LENGTH];
int Cell_B_Current[SAMPLE_LENGTH];

bool OverCurrent = false;
int MaxDuty = 1000;
unsigned int flipper_angle_offset = 0;
void calibrate_flipper_angle_sensor(void);
static void read_stored_angle_offset(void);

void turn_on_power_bus_new_method(void);
void turn_on_power_bus_old_method(void);
void turn_on_power_bus_hybrid_method(void);
bool check_string_match(const unsigned char *string1, const unsigned char *string2,
                        unsigned char length);

static void alternate_power_bus(void);

#ifdef UART_CONTROL
#define UART_RECEIVE_BUFFER_LENGTH 6
typedef enum UARTState { UART_STATE_IDLE, UART_STATE_PROCESSING } UArtState;
int16_t uart_receive_buffer[UART_RECEIVE_BUFFER_LENGTH];
int i_uart_receive_buffer = 0;
// no more than 5 bytes based on 57600 baud and 1ms system loop
#define UART_SEND_BUFFER_LENGTH 4
uint8_t uart_send_buffer[UART_SEND_BUFFER_LENGTH];
uint8_t i_uart_send_buffer = 0;
uint8_t uart_data_identifier = 0;
int16_t uart_motor_velocity[3];
uint8_t uart_incoming_cmd[2];

bool uart_has_new_data = false;
const uint8_t UART_START_BYTE = 253;
uint16_t EncoderInterval[MOTOR_CHANNEL_COUNT]; // Encoder time interval
uint16_t BuildNO = 40621;
bool uart_has_new_fan_speed = false;
typedef enum UARTCommand {
    UART_COMMAND_GET = 10,
    UART_COMMAND_SET_FAN_SPEED = 20,
    UART_COMMAND_SET_MOTOR_SLOW_SPEED = 240,
    UART_COMMAND_FLIPPER_CALIBRATE = 250,
} UARTCommand;
bool uart_flipper_calibrate_requested = false;
#endif

void PWM1Duty(int Duty);
void PWM2Duty(int Duty);
void PWM3Duty(int Duty);
void PWM1Ini(void);
void PWM2Ini(void);
void PWM3Ini(void);

void set_firmware_build_time(void);

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

    // read flipper position from flash, and put it into a module variable
    read_stored_angle_offset();

    // init variables for closed loop control
    closed_loop_control_init();
}

void GetCurrent(MotorChannel Channel) {
    uint16_t temp = 0;
    // read the AD value
    temp = mean_u(SAMPLE_LENGTH, MotorCurrentAD[Channel]);
    Current4Control[Channel][Current4ControlPointer[Channel]] = temp;
    Current4ControlPointer[Channel] = (Current4ControlPointer[Channel] + 1) % SAMPLE_LENGTH_CONTROL;
}


void Device_MotorController_Process() {
    MotorChannel i;
    long temp1, temp2;
    static int overcurrent_counter = 0;
    // check if software wants to calibrate flipper position
#ifndef UART_CONTROL
    if (REG_MOTOR_VELOCITY.flipper == 12345) {
        calibrate_flipper_angle_sensor();
    }
#endif

#ifdef UART_CONTROL
    if (uart_flipper_calibrate_requested) {
        calibrate_flipper_angle_sensor();
        // clear the flag just for safe
        uart_flipper_calibrate_requested = false;
        REG_MOTOR_SIDE_FAN_SPEED = 240;
        uart_has_new_fan_speed = true;
        uart_FanSpeedTimerEnabled = true;
        uart_FanSpeedTimerCount = 0;
    }
#endif

    IC_UpdatePeriods();
    // Check Timer
    // Run control loop
    
    if (IFS0bits.T1IF == SET) {

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
        if (USBTimeOutTimerEnabled) {
            USBTimeOutTimerCount++;
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
#ifdef UART_CONTROL
        if (uart_FanSpeedTimerEnabled) {
            uart_FanSpeedTimerCount++;
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
    if (CurrentFBTimerCount >= CurrentFBTimer) {
        CurrentFBTimerExpired = true;
        CurrentFBTimerCount = 0;
        CurrentFBTimerEnabled = false;
    }
    if (StateMachineTimerCount >= StateMachineTimer) {
        StateMachineTimerExpired = true;
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
#ifdef UART_CONTROL
    if (uart_FanSpeedTimerCount >= UART_FAN_SPEED_TIMER) {
        uart_fan_speed_expired = true;
        uart_FanSpeedTimerCount = 0;
        uart_FanSpeedTimerEnabled = false;
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
    if (CurrentFBTimerExpired) {
        // clear CurrentFBTimerExpired
        CurrentFBTimerExpired = false;
        CurrentFBTimerEnabled = true;
        for (EACH_MOTOR_CHANNEL(i)) {
            GetCurrent(i);
        }
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
        // TODO: FIX THIS
        // REG_MOTOR_ENCODER_COUNT.left = ...
        // REG_MOTOR_ENCODER_COUNT.right = ...
        
        // update the mosfet driving fault flag pin 1-good 2-fault
        REG_MOTOR_FAULT_FLAG.left = PORTDbits.RD1;
        REG_MOTOR_FAULT_FLAG.right = PORTEbits.RE5;
        // update temperatures for two motors done in I2C code
        
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

#ifdef UART_CONTROL
    if (uart_fan_speed_expired) {
        uart_fan_speed_expired = false;
        uart_FanSpeedTimerEnabled = false;
        uart_FanSpeedTimerCount = 0;
        // clear all the fan command
        REG_MOTOR_SIDE_FAN_SPEED = 0;
        uart_has_new_fan_speed = 0;
    }
#endif
    // T5

    USBInput();

    // update state machine
    if (StateMachineTimerExpired) {
        // clear the flag
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

int GetDuty(long Target, MotorChannel Channel) {
    if (Channel == MOTOR_FLIPPER) {
        return Target;
    } else {
        return min(labs(Target), MaxDuty);
    }
}

void UpdateSpeed(MotorChannel Channel, int State) {
    int Dutycycle;

    ControlCurrent[Channel] = mean_u(SAMPLE_LENGTH_CONTROL, Current4Control[Channel]);

    switch (Channel) {
    case MOTOR_LEFT:
        if (State == Forward) {
            if (OverCurrent) {
                Dutycycle = 0;
                M1_COAST = Set_ActiveLO;
            } else {
                Dutycycle = GetDuty(motor_target_speed[Channel], Channel);
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
                Dutycycle = GetDuty(motor_target_speed[Channel], Channel);
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
                Dutycycle = GetDuty(motor_target_speed[Channel], Channel);
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
                Dutycycle = GetDuty(motor_target_speed[Channel], Channel);
                M2_COAST = Clear_ActiveLO;
            }
            M2_BRAKE = Clear_ActiveLO;
            M2_DIR = HI;
            PWM2Duty(Dutycycle);
        }
        break;
    case MOTOR_FLIPPER:
        if (State == Forward) {
            Dutycycle = GetDuty(motor_target_speed[Channel], Channel);
            M3_COAST = Clear_ActiveLO;
            Nop();
            M3_BRAKE = Clear_ActiveLO;
            Nop();
            M3_DIR = HI;
            PWM3Duty(Dutycycle);
        } else if (State == Backward) {
            Dutycycle = GetDuty(-motor_target_speed[Channel], Channel);
            M3_COAST = Clear_ActiveLO;
            Nop();
            M3_BRAKE = Clear_ActiveLO;
            Nop();
            M3_DIR = LO;
            PWM3Duty(Dutycycle);
        }
        break;
    }
}

//*********************************************//

void USBInput() {
    MotorChannel i;
    static bool USB_New_Data_Received;
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
#ifndef UART_CONTROL
            motor_target_speed[MOTOR_LEFT] = REG_MOTOR_VELOCITY.left;
            motor_target_speed[MOTOR_RIGHT] = REG_MOTOR_VELOCITY.right;
#endif
#ifdef UART_CONTROL
            motor_target_speed[MOTOR_LEFT] = uart_motor_velocity[MOTOR_LEFT];
            motor_target_speed[MOTOR_RIGHT] = uart_motor_velocity[MOTOR_RIGHT];
#endif
        }

        if (flipper_control_loop_counter > 15) {
            flipper_control_loop_counter = 0;
#ifndef UART_CONTROL
            motor_target_speed[MOTOR_FLIPPER] = REG_MOTOR_VELOCITY.flipper;
#endif
#ifdef UART_CONTROL
            motor_target_speed[MOTOR_FLIPPER] = uart_motor_velocity[MOTOR_FLIPPER];
#endif
        }

        if (REG_MOTOR_SLOW_SPEED)
            state = DECEL_AFTER_HIGH_SPEED;

        break;

    case DECEL_AFTER_HIGH_SPEED:

        control_loop_counter++;

        if (control_loop_counter > 3) {
            control_loop_counter = 0;

            // set speed to 1 (speed of 0 immediately stops motors)
            motor_target_speed[MOTOR_LEFT] = 1;
            motor_target_speed[MOTOR_RIGHT] = 1;
            motor_target_speed[MOTOR_FLIPPER] = 1;
        }

        // motors are stopped if speed falls below 1%
        if ((abs(motor_target_speed[MOTOR_LEFT]) < 10) &&
            (abs(motor_target_speed[MOTOR_RIGHT]) < 10) &&
            (abs(motor_target_speed[MOTOR_FLIPPER]) < 10))
            state = LOW_SPEED;

        break;

    case LOW_SPEED:
#ifndef UART_CONTROL
        set_desired_velocities(REG_MOTOR_VELOCITY.left, REG_MOTOR_VELOCITY.right,
                               REG_MOTOR_VELOCITY.flipper);
#endif
#ifdef UART_CONTROL
        set_desired_velocities(uart_motor_velocity[MOTOR_LEFT], uart_motor_velocity[MOTOR_RIGHT],
                               uart_motor_velocity[MOTOR_FLIPPER]);
#endif
        motor_target_speed[MOTOR_LEFT] = return_closed_loop_control_effort(MOTOR_LEFT);
        motor_target_speed[MOTOR_RIGHT] = return_closed_loop_control_effort(MOTOR_RIGHT);
        motor_target_speed[MOTOR_FLIPPER] = return_closed_loop_control_effort(MOTOR_FLIPPER);

        control_loop_counter = 0;
        flipper_control_loop_counter = 0;

        if (!REG_MOTOR_SLOW_SPEED)
            state = DECEL_AFTER_LOW_SPEED;

        break;

    case DECEL_AFTER_LOW_SPEED:

        set_desired_velocities(0, 0, 0);

        motor_target_speed[MOTOR_LEFT] = return_closed_loop_control_effort(MOTOR_LEFT);
        motor_target_speed[MOTOR_RIGHT] = return_closed_loop_control_effort(MOTOR_RIGHT);
        motor_target_speed[MOTOR_FLIPPER] = return_closed_loop_control_effort(MOTOR_FLIPPER);

        // motors are stopped if speed falls below 1%
        if ((abs(motor_target_speed[MOTOR_LEFT]) < 10) &&
            (abs(motor_target_speed[MOTOR_RIGHT]) < 10) &&
            (abs(motor_target_speed[MOTOR_FLIPPER]) < 10))
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
#ifdef UART_CONTROL
        uart_motor_velocity[MOTOR_LEFT] = 0;
        uart_motor_velocity[MOTOR_RIGHT] = 0;
        uart_motor_velocity[MOTOR_FLIPPER] = 0;
#endif

        for (EACH_MOTOR_CHANNEL(i)) {
            motor_target_speed[i] = 0;
            Event[i] = Stop;
        }
    }

#ifdef UART_CONTROL
    if (uart_has_new_data) {
        uart_has_new_data = false;
#else
    if (USB_New_Data_Received) {
        USB_New_Data_Received = false;
#endif
        // if there is new data coming in, update all the data
        USBTimeOutTimerCount = 0;
        USBTimeOutTimerEnabled = true;
        USBTimeOutTimerExpired = false;
        for (EACH_MOTOR_CHANNEL(i)) {
            if (motor_target_speed[i] == 0)
                Event[i] = Stop;
            else if (-1024 < motor_target_speed[i] && motor_target_speed[i] < 0)
                Event[i] = Back;
            else if (0 < motor_target_speed[i] && motor_target_speed[i] < 1024)
                Event[i] = Go;
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

#ifdef UART_CONTROL
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

#ifdef UART_CONTROL
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

    // initialize PWM sub module
    PWM1Ini();
    PWM2Ini();
    PWM3Ini();

    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);

#ifdef UART_CONTROL
    UART1Ini();
#endif

    set_firmware_build_time();
}

void InterruptIni() {
    // remap all the interrupt routines
    T3InterruptUserFunction = Motor_T3Interrupt;
    ADC1InterruptUserFunction = Motor_ADC1Interrupt;
#ifdef UART_CONTROL
    U1TXInterruptUserFunction = Motor_U1TXInterrupt;
    U1RXInterruptUserFunction = Motor_U1RXInterrupt;
#endif
}

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

/*****************************************************************************/
//*-----------------------------------Timer----------------------------------*/

void IniTimer1() {
    T1CON = 0x0000;         // clear register
    T1CONbits.TCKPS = 0b00; // 1:1 prescale
    TMR1 = 0;           // clear timer1 register
    PR1 = -1; // interrupt every 1ms
    T1CONbits.TON = SET; // timer on
}

void IniTimer2() {
    T2CON = 0x0000;         // stops timer2,16 bit timer,internal clock (Fosc/2)
    T2CONbits.TCKPS = 0b00; // 1:1 prescale
    TMR2 = 0;              
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

void Motor_T3Interrupt(void) {
    // PORTCbits.RC13=~PORTCbits.RC13;
    // clear timer3 flage
    IFS0bits.T3IF = CLEAR; // clear interrupt flag
    Timer3Count++;

    // TODO: turn on timer

    if (Timer3Count >= 0) {
        Timer3Count = 0;
        AD1CON1bits.ASAM = SET;
    }
}

void Motor_ADC1Interrupt(void) {
    AD1CON1bits.ASAM = CLEAR;
    // clear the flag
    IFS0bits.AD1IF = CLEAR;
    // load the value

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
    // increase array pointer, prevent over flow
    CellVoltageArrayPointer = (CellVoltageArrayPointer + 1) % SAMPLE_LENGTH;
    Total_Cell_Current_ArrayPointer = (Total_Cell_Current_ArrayPointer + 1) % SAMPLE_LENGTH;
    M3_POSFB_ArrayPointer = (M3_POSFB_ArrayPointer + 1) % SAMPLE_LENGTH;
    MotorCurrentADPointer = (MotorCurrentADPointer + 1) % SAMPLE_LENGTH;

#ifdef UART_CONTROL
    EncoderInterval[MOTOR_LEFT] = IC_period(kIC01);    // left motor encoder time interval
    EncoderInterval[MOTOR_RIGHT] = IC_period(kIC02);   // right motor encoder time interval
    EncoderInterval[MOTOR_FLIPPER] = IC_period(kIC03); // Encoder motor encoder time interval
    // if transmit reg is empty and last packet is sent
    if (uart_incoming_cmd[0] == UART_COMMAND_GET && U1STAbits.UTXBF == 0 &&
        i_uart_send_buffer == 0) // if transmit reg is empty and last packet is sent
    {
        U1TXREG = UART_START_BYTE; // send out the index
        uart_data_identifier = uart_incoming_cmd[1];
        uart_send_buffer[0] = uart_data_identifier;

// CASE(n, REGISTER) populates the UART output buffer with the 16-bit integer value of the given
// register and breaks out of the switch statement
#define CASE(n, REGISTER)                                                                          \
    case (n):                                                                                      \
        uart_send_buffer[1] = (uint8_t)((REGISTER) >> 8 & 0xff);                                   \
        uart_send_buffer[2] = (uint8_t)(REGISTER & 0xff);                                          \
        break;

        switch (uart_data_identifier) {
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
            uart_send_buffer[1] = REG_MOTOR_FAULT_FLAG.left;
            uart_send_buffer[2] = REG_MOTOR_FAULT_FLAG.right;
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
            CASE(48, REG_MOTOR_SIDE_FAN_SPEED)
            CASE(50, REG_MOTOR_SLOW_SPEED)
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
            uart_send_buffer[0] = 0;
            uart_send_buffer[1] = 0;
            break;
        }
#undef CASE
        // add checksum of the package
        uart_send_buffer[3] =
            255 - (uart_send_buffer[0] + uart_send_buffer[1] + uart_send_buffer[2]) % 255;
        // clear incoming command
        uart_incoming_cmd[0] = 0;
        uart_incoming_cmd[1] = 0;
    }
#endif
}

#ifdef UART_CONTROL
void Motor_U1TXInterrupt(void) {
    // clear the flag
    IFS0bits.U1TXIF = 0;

    // transmit data
    if (i_uart_send_buffer < UART_SEND_BUFFER_LENGTH) {
        U1TXREG = uart_send_buffer[i_uart_send_buffer];
        i_uart_send_buffer++;
    } else {
        i_uart_send_buffer = 0;
    }
}
#endif

#ifdef UART_CONTROL
void Motor_U1RXInterrupt(void) {
    static UArtState uart_state;
    uint8_t a_byte;
    // clear the flag
    IFS0bits.U1RXIF = 0;
    // UART_CONTROL code only

    int i = 0;
    a_byte = U1RXREG;
    switch (uart_state) {
    case UART_STATE_IDLE:
        if (a_byte == UART_START_BYTE) {
            uart_state = UART_STATE_PROCESSING;
        }
        break;
    case UART_STATE_PROCESSING:
        uart_receive_buffer[i_uart_receive_buffer] = a_byte;
        i_uart_receive_buffer++;
        if (i_uart_receive_buffer >= UART_RECEIVE_BUFFER_LENGTH) // end of the package
        {
            // if all bytes add up together equals 255, it is a good pack, then process
            int SumBytes = 0;
            for (i = 0; i < UART_RECEIVE_BUFFER_LENGTH; i++) {
                // printf("%d:%d  ",i,uart_receive_buffer[i]);
                SumBytes += uart_receive_buffer[i];
            }
            // printf("Sum:%d\n",SumBytes);
            if (SumBytes % 255 == 0) {
                // input data range 0~250, 125 is stop, 0 is backwards full speed, 250 is forward
                // full speed for Marge, the right and left is flipped, so left and right need to be
                // flipped
                uart_motor_velocity[MOTOR_LEFT] = (uart_receive_buffer[0] * 8 - 1000);
                uart_motor_velocity[MOTOR_RIGHT] = (uart_receive_buffer[1] * 8 - 1000);
                uart_motor_velocity[MOTOR_FLIPPER] = (uart_receive_buffer[2] * 8 - 1000);
                uart_incoming_cmd[0] = uart_receive_buffer[3];
                uart_incoming_cmd[1] = uart_receive_buffer[4];
                // see if this is a fan cmd
                switch (uart_incoming_cmd[0]) {
                case UART_COMMAND_SET_FAN_SPEED: // new fan command coming in
                    REG_MOTOR_SIDE_FAN_SPEED = uart_incoming_cmd[1];
                    uart_has_new_fan_speed = true;
                    // Enable fan speed timer
                    uart_FanSpeedTimerEnabled = true;
                    uart_FanSpeedTimerCount = 0;
                    // printf("fan data received!");
                    break;
                case UART_COMMAND_SET_MOTOR_SLOW_SPEED:
                    REG_MOTOR_SLOW_SPEED = uart_incoming_cmd[1];
                    break;
                case UART_COMMAND_FLIPPER_CALIBRATE:
                    if (uart_incoming_cmd[1] == UART_COMMAND_FLIPPER_CALIBRATE) {
                        uart_flipper_calibrate_requested = true;
                    }
                    break;
                }
                // REG_MOTOR_VELOCITY.left=300;
                // REG_MOTOR_VELOCITY.right=800;
                // REG_MOTOR_VELOCITY.flipper=400;
                uart_has_new_data = true;
            }
            // clear all the local buffer
            i_uart_receive_buffer = 0;
            uart_state = UART_STATE_IDLE;
            break;
        }
    }
}
#endif

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
        }

        block_ms(40);
        k = k0 + i * i / 4;
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
        turn_on_power_bus_old_method();
        return;
    }

    // If we're using the new battery (BT-70791B)
    if (check_string_match(DEVICE_NAME_NEW_BATTERY, battery_data1,
                           sizeof(DEVICE_NAME_NEW_BATTERY)) ||
        check_string_match(DEVICE_NAME_NEW_BATTERY, battery_data2,
                           sizeof(DEVICE_NAME_NEW_BATTERY))) {
        turn_on_power_bus_new_method();
        return;
    }

    // If we're using Bren-Tronics BT-70791C
    if (check_string_match(DEVICE_NAME_BT70791_CK, battery_data1, sizeof(DEVICE_NAME_BT70791_CK)) ||
        check_string_match(DEVICE_NAME_BT70791_CK, battery_data2, sizeof(DEVICE_NAME_BT70791_CK))) {
        turn_on_power_bus_new_method();
        return;
    }

    // if we're using the low lithium custom Matthew's battery
    if (check_string_match(DEVICE_NAME_CUSTOM_BATTERY, battery_data1,
                           sizeof(DEVICE_NAME_CUSTOM_BATTERY)) ||
        check_string_match(DEVICE_NAME_CUSTOM_BATTERY, battery_data2,
                           sizeof(DEVICE_NAME_CUSTOM_BATTERY))) {
        turn_on_power_bus_old_method();
        return;
    }

    // if we're using an unknown battery

    turn_on_power_bus_hybrid_method();
}

bool check_string_match(const unsigned char *string1, const unsigned char *string2,
                        unsigned char length) {
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
