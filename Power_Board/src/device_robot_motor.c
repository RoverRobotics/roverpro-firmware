#include <p24Fxxxx.h>
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "interrupt_switch.h"
#include "device_robot_motor_i2c.h"
#include "DEE Emulation 16-bit.h"
#include "i2clib.h"
#include "device_robot_motor_loop.h"
#include "InputCapture.h"
#include "uart_control.h"
#include <string.h>

//****************************************************

/// The requested motor state based in inbound motor speed commands.
MotorEvent motor_event[MOTOR_CHANNEL_COUNT] = {STOP, STOP, STOP};
/// The outbound commands we are sending to the motor
MotorState motor_state[MOTOR_CHANNEL_COUNT] = {SWITCH_DIRECTION, SWITCH_DIRECTION,
                                               SWITCH_DIRECTION};

typedef enum CounterState {
    COUNTER_RUNNING,
    COUNTER_PAUSED,
    COUNTER_EXPIRED,
} CounterState;

typedef struct Counter {
    uint16_t count;
    uint16_t max;
    bool is_paused : 1;
    bool pause_on_expired : 1;
} Counter;

void counter_restart(Counter *c) {
    c->is_paused = false;
    c->count = 0;
}
void counter_stop(Counter *c) {
    c->is_paused = true;
    c->count = 0;
}
CounterState counter_tick(Counter *c) {
    if (c->is_paused)
        return COUNTER_PAUSED;
    c->count++;
    if (c->count < c->max) {
        return COUNTER_RUNNING;
    } else {
        if (c->pause_on_expired)
            counter_stop(c);
        else
            counter_restart(c);
        return COUNTER_EXPIRED;
    }
}

Counter motor_switch_direction_timer[MOTOR_CHANNEL_COUNT] = {
    {.max = INTERVAL_MS_SWITCH_DIRECTION / INTERVAL_MS_STATE_MACHINE, .pause_on_expired = true, .is_paused = true},
    {.max = INTERVAL_MS_SWITCH_DIRECTION / INTERVAL_MS_STATE_MACHINE, .pause_on_expired = true, .is_paused = true},
    {.max = INTERVAL_MS_SWITCH_DIRECTION / INTERVAL_MS_STATE_MACHINE, .pause_on_expired = true, .is_paused = true},
};

Counter speed_update_timer[MOTOR_CHANNEL_COUNT] = {
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
};

Counter usb_timeout = {.max = TIMEOUT_MS_USB, .pause_on_expired = true, .is_paused = true};
Counter state_machine = {.max = INTERVAL_MS_STATE_MACHINE};
Counter current_fb = {.max = INTERVAL_MS_CURRENT_FEEDBACK};
Counter i2c2_timeout = {.max = INTERVAL_MS_I2C2};
Counter i2c3_timeout = {.max = INTERVAL_MS_I2C3};
Counter sfreg_update = {.max = INTERVAL_MS_SFREG};
Counter bat_vol_checking = {.max = INTERVAL_MS_BATTERY_CHECK};
Counter bat_recovery = {.max = INTERVAL_MS_BATTERY_RECOVER, .pause_on_expired = true, .is_paused = true};
Counter closed_loop_control = {.max = 10};
Counter uart_fan_speed_timeout = {
    .max = INTERVAL_MS_UART_FAN_SPEED_TIMEOUT, .pause_on_expired = true, .is_paused = true};

uint16_t motor_current_ad[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH] = {{0}};
int motor_current_ad_pointer = 0;
uint16_t current_for_control[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH_CONTROL] = {{0}};
int current_for_control_pointer[MOTOR_CHANNEL_COUNT] = {0};
int16_t motor_target_speed[MOTOR_CHANNEL_COUNT] = {0};

int timer3_count = 0;
int m3_posfb_array[2][SAMPLE_LENGTH] = {{0}}; ///< Flipper Motor positional feedback data
int m3_posfb_array_pointer = 0;
uint16_t total_cell_current_array[SAMPLE_LENGTH] = {0};
int total_cell_current_array_pointer = 0;
int cell_voltage_array[BATTERY_CHANNEL_COUNT][SAMPLE_LENGTH];
int cell_voltage_array_pointer = 0;
uint16_t cell_current[BATTERY_CHANNEL_COUNT][SAMPLE_LENGTH];

bool over_current = false;
const uint16_t MAX_DUTY = 1000;
uint16_t flipper_angle_offset = 0;
void calibrate_flipper_angle_sensor(void);
static void read_stored_angle_offset(void);

void turn_on_power_bus_new_method(void);
void turn_on_power_bus_old_method(void);
void turn_on_power_bus_hybrid_method(void);

static void power_bus_tick(void);

void set_firmware_build_time(void);

static uint16_t return_combined_pot_angle(uint16_t pot_1_value, uint16_t pot_2_value);
static uint16_t return_calibrated_pot_angle(uint16_t pot_1_value, uint16_t pot_2_value);

void power_bus_init(void);

// invalid flipper pot thresholds.  These are very wide because the flipper pots are on a
// different 3.3V supply than the PIC If the flipper pot is below this threshold, it is invalid
#define LOW_POT_THRESHOLD 33
// If the flipper pot is above this threshold, it is invalid
#define HIGH_POT_THRESHOLD 990
#define FLIPPER_POT_OFFSET -55

//*********************************************//

/** returns the given angle, normalized to between 0 and 360 */
uint16_t wrap_angle(int16_t value) { return (uint16_t)((value % 360 + 360) % 360); }

void DeviceRobotMotorInit() {
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

    // initialize timers
    IniTimer2();
    IniTimer3();
    IniTimer1();

    // initialize PWM sub module
    PWM1Ini();
    PWM2Ini();
    PWM3Ini();

    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);

    uart_init();

    set_firmware_build_time();

    power_bus_init();
    FANCtrlIni();

    Coasting(MOTOR_LEFT);
    Coasting(MOTOR_RIGHT);
    Coasting(MOTOR_FLIPPER);

    // read flipper position from flash, and put it into a module variable
    read_stored_angle_offset();

    // init variables for closed loop control
    closed_loop_control_init();
}

void GetCurrent(MotorChannel Channel) {
    uint16_t temp = 0;
    temp = mean_u(SAMPLE_LENGTH, motor_current_ad[Channel]);
    current_for_control[Channel][current_for_control_pointer[Channel]] = temp;
    current_for_control_pointer[Channel] =
        (current_for_control_pointer[Channel] + 1) % SAMPLE_LENGTH;
}

void Device_MotorController_Process() {
    UArtTickResult uart_tick_result;
    MotorChannel i;
    long temp1, temp2;
    static int overcurrent_counter = 0;
    // check if software wants to calibrate flipper position

    IC_UpdatePeriods();
    // Check Timer
    // Run control loop

    if (IFS0bits.T1IF == SET) {
        // Clear interrupt flag
        IFS0bits.T1IF = CLEAR;

        // this should run every 1ms
        power_bus_tick();

        // check if any timers are expired
        if (counter_tick(&closed_loop_control) == COUNTER_EXPIRED) {
            handle_closed_loop_control(over_current);
        }

        // if any of the timers expired, execute relative codes
        // Control timer expired
        for (EACH_MOTOR_CHANNEL(i)) {
            if (counter_tick(&speed_update_timer[i]) == COUNTER_EXPIRED) {
                UpdateSpeed(i, motor_state[i]);
            }
        }
        if (counter_tick(&current_fb) == COUNTER_EXPIRED) {
            for (EACH_MOTOR_CHANNEL(i)) {
                GetCurrent(i);
            }
        }

        if (counter_tick(&sfreg_update) == COUNTER_EXPIRED) {
            // update flipper motor position
            temp1 = mean(SAMPLE_LENGTH, m3_posfb_array[0]);
            temp2 = mean(SAMPLE_LENGTH, m3_posfb_array[1]);
            REG_FLIPPER_FB_POSITION.pot1 = temp1;
            REG_FLIPPER_FB_POSITION.pot2 = temp2;
            REG_MOTOR_FLIPPER_ANGLE = return_calibrated_pot_angle(temp1, temp2);

            // update current for all three motors
            REG_MOTOR_FB_CURRENT.left =
                mean_u(SAMPLE_LENGTH_CONTROL, current_for_control[MOTOR_LEFT]);
            REG_MOTOR_FB_CURRENT.right =
                mean_u(SAMPLE_LENGTH_CONTROL, current_for_control[MOTOR_RIGHT]);
            REG_MOTOR_FB_CURRENT.flipper =
                mean_u(SAMPLE_LENGTH_CONTROL, current_for_control[MOTOR_FLIPPER]);

            // update the mosfet driving fault flag pin 1-good 2-fault
            REG_MOTOR_FAULT_FLAG.left = PORTDbits.RD1;
            REG_MOTOR_FAULT_FLAG.right = PORTEbits.RE5;
            // update temperatures for two motors done in I2C code

            // update battery voltage
            REG_PWR_BAT_VOLTAGE.a = mean(SAMPLE_LENGTH, cell_voltage_array[CELL_A]);
            REG_PWR_BAT_VOLTAGE.b = mean(SAMPLE_LENGTH, cell_voltage_array[CELL_B]);

            // update total current (out of battery)
            REG_PWR_TOTAL_CURRENT = mean_u(SAMPLE_LENGTH, total_cell_current_array);
        }

        if (counter_tick(&bat_vol_checking) == COUNTER_EXPIRED) {
            // added this so that we check voltage faster
            REG_PWR_A_CURRENT = mean_u(SAMPLE_LENGTH, cell_current[CELL_A]);
            REG_PWR_B_CURRENT = mean_u(SAMPLE_LENGTH, cell_current[CELL_B]);

            //.01*.001mV/A * 11000 ohms = .11 V/A = 34.13 ADC counts/A
            // set at 10A per side
            if (REG_PWR_A_CURRENT >= 512 || REG_PWR_B_CURRENT >= 512) {
                overcurrent_counter++;
                if (overcurrent_counter > 10) {
                    PWM1Duty(0);
                    PWM2Duty(0);
                    PWM3Duty(0);
                    Coasting(MOTOR_LEFT);
                    Coasting(MOTOR_RIGHT);
                    Coasting(MOTOR_FLIPPER);
                    over_current = true;
                    bat_recovery.is_paused = false;
                }

            } else if (REG_PWR_A_CURRENT <= 341 || REG_PWR_B_CURRENT <= 341) {
                overcurrent_counter = 0;
            }
        }
        if (counter_tick(&bat_recovery) == COUNTER_EXPIRED) {
            over_current = false;
        }

        {
            bool should_reset = (counter_tick(&i2c2_timeout) == COUNTER_EXPIRED);
            bool did_finish = false;
            i2c2_tick(should_reset, &did_finish);
            if (did_finish) {
                counter_restart(&i2c2_timeout);
            }
        }
        {
            bool should_reset = (counter_tick(&i2c3_timeout) == COUNTER_EXPIRED);
            bool did_finish = false;
            i2c3_tick(should_reset, &did_finish);
            if (did_finish) {
                counter_restart(&i2c3_timeout);
            }
        }
        uart_tick_result = uart_tick();
        USBInput();

        if (uart_tick_result.uart_flipper_calibrate_requested) {
            // note flipper calibration never returns.
            calibrate_flipper_angle_sensor();
        }
        if (uart_tick_result.uart_fan_speed_requested) {
            counter_restart(&uart_fan_speed_timeout);
        }
        if (counter_tick(&uart_fan_speed_timeout) == COUNTER_EXPIRED) {
            // clear all the fan command
            REG_MOTOR_SIDE_FAN_SPEED = 0;
        }
        if (uart_tick_result.uart_motor_speed_requested) {
            counter_restart(&usb_timeout);
        }
        // long time no data, clear everything and stop moving
        if (counter_tick(&usb_timeout) == COUNTER_EXPIRED) {
            REG_MOTOR_VELOCITY.left = 0;
            REG_MOTOR_VELOCITY.right = 0;
            REG_MOTOR_VELOCITY.flipper = 0;

            for (EACH_MOTOR_CHANNEL(i)) {
                motor_target_speed[i] = 0;
            }
        }

        for (EACH_MOTOR_CHANNEL(i)) {
            if (motor_target_speed[i] == 0)
                motor_event[i] = STOP;
            else if (-1024 < motor_target_speed[i] && motor_target_speed[i] < 0)
                motor_event[i] = BACK;
            else if (0 < motor_target_speed[i] && motor_target_speed[i] < 1024)
                motor_event[i] = GO;
        }

        // update state machine
        // If the new motor commands reverse or brake a motor, coast that motor for a few ticks
        // (SWITCH_DIRECTION mode) Then once the switch direction timer elapses, start the motor in
        // that new direction If we are moving forward or in reverse and the intended speed changes,
        // update the pwm command to the motor
        if (counter_tick(&state_machine) == COUNTER_EXPIRED) {
            for (EACH_MOTOR_CHANNEL(i)) {
                // update state machine
                switch (motor_state[i]) {
                case BRAKE:
                    // brake motor
                    Braking(i);
                    switch (motor_event[i]) {
                    case GO:
                        // fallthrough
                    case BACK:
                        motor_state[i] = SWITCH_DIRECTION;
                        counter_stop(&speed_update_timer[i]);
                        counter_restart(&motor_switch_direction_timer[i]);
                        Coasting(i);
                        break;
                    case STOP:
                        motor_event[i] = NO_EVENT;
                        break;
                    case NO_EVENT:
                        break;
                    }
                    break;
                case FORWARD:
                    switch (motor_event[i]) {
                    case GO:
                        speed_update_timer[i].is_paused = false;
                        break;
                    case BACK:
                        // fallthrough
                    case STOP:
                        motor_state[i] = SWITCH_DIRECTION;
                        counter_stop(&speed_update_timer[i]);
                        counter_restart(&motor_switch_direction_timer[i]);
                        break;
                    case NO_EVENT:
                        break;
                    }
                    break;
                case BACKWARD:
                    switch (motor_event[i]) {
                    case GO:
                        // fallthrough
                    case STOP:
                        motor_state[i] = SWITCH_DIRECTION;
                        counter_stop(&speed_update_timer[i]);
                        counter_restart(&motor_switch_direction_timer[i]);
                        break;
                    case BACK:
                        speed_update_timer[i].is_paused = false;
                        break;
                    case NO_EVENT:
                        break;
                    }
                    break;
                case SWITCH_DIRECTION:
                    if (counter_tick(&motor_switch_direction_timer[i]) == COUNTER_EXPIRED) {
                        switch (motor_event[i]) {
                        case STOP:
                            Braking(i);
                            motor_state[i] = BRAKE;
                            break;
                        case GO:
                            counter_restart(&speed_update_timer[i]);
                            motor_state[i] = FORWARD;
                            break;
                        case BACK:
                            counter_restart(&speed_update_timer[i]);
                            motor_state[i] = BACKWARD;
                            break;
                        case NO_EVENT:
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }
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
void Coasting(MotorChannel channel) {
    switch (channel) {
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

uint16_t GetDuty(int16_t target, MotorChannel channel) {
    if (channel == MOTOR_FLIPPER) {
        return (uint16_t)abs(target);
    } else {
        return min(abs(target), MAX_DUTY);
    }
}

void UpdateSpeed(MotorChannel channel, MotorState state) {
    uint16_t duty;

    switch (channel) {
    case MOTOR_LEFT:
        if (over_current) {
            duty = 0;
            M1_COAST = Set_ActiveLO;
        } else {
            duty = GetDuty(motor_target_speed[channel], channel);
            M1_COAST = Clear_ActiveLO;
        }
        M1_BRAKE = Clear_ActiveLO;
        M1_DIR = (state == FORWARD) ? HI : LO;
        PWM1Duty(duty);
        break;
    case MOTOR_RIGHT:
        if (over_current) {
            duty = 0;
            M2_COAST = Set_ActiveLO;
        } else {
            duty = GetDuty(motor_target_speed[channel], channel);
            M2_COAST = Clear_ActiveLO;
        }
        M2_BRAKE = Clear_ActiveLO;
        M2_DIR = (state == FORWARD) ? HI : LO;
        PWM2Duty(duty);
        break;
    case MOTOR_FLIPPER:
        duty = GetDuty(motor_target_speed[channel], channel);
        M3_COAST = Clear_ActiveLO;
        M3_BRAKE = Clear_ActiveLO;
        M3_DIR = (state == FORWARD) ? HI : LO;
        PWM3Duty(duty);
        break;
    }
}

//*********************************************//

void USBInput() {
    static uint16_t control_loop_counter = 0, flipper_control_loop_counter = 0;

    typedef enum {
        /// We are in normal control mode
        HIGH_SPEED = 0,
        /// We were in normal control mode and have received a request to change to low speed mode
        DECEL_AFTER_HIGH_SPEED,
        /// We are in low speed (PID) mode
        LOW_SPEED,
        /// We were in low speed mode and have received a request to switch to normal control mode
        DECEL_AFTER_LOW_SPEED
    } motor_speed_state;

    static motor_speed_state state = HIGH_SPEED;

    // If switching between low speed mode and high speed mode (REG_MOTOR_SLOW_SPEED)
    // Motors must decelerate upon switching speeds (if they're not already stopped.
    // Failure to do so could cause large current spikes which will trigger the
    // protection circuitry in the battery, cutting power to the robot
    switch (state) {
    case HIGH_SPEED:
        control_loop_counter++;
        flipper_control_loop_counter++;

        if (control_loop_counter > 5) {
            control_loop_counter = 0;
            motor_target_speed[MOTOR_LEFT] = REG_MOTOR_VELOCITY.left;
            motor_target_speed[MOTOR_RIGHT] = REG_MOTOR_VELOCITY.right;
        }

        if (flipper_control_loop_counter > 15) {
            flipper_control_loop_counter = 0;
            motor_target_speed[MOTOR_FLIPPER] = REG_MOTOR_VELOCITY.flipper;
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
        set_desired_velocities(REG_MOTOR_VELOCITY.left, REG_MOTOR_VELOCITY.right,
                               REG_MOTOR_VELOCITY.flipper);
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
}

//**********************************************

void PinRemap(void) {
    // Assign U1RX To U1RX, Uart receive channnel
    RPINR18bits.U1RXR = U1RX_RPn;
    // Assign U1TX To U1TX/Uart Tx
    U1TX_RPn = 3; // 3 represents U1TX

    //***************************
    // Configure Output Functions
    // pin->function
    // Assign OC1 To Pin M1_AHI
    M1_PWM = 18; // 18 represents OC1

    // Assign OC2 To Pin M1_BHI
    M2_PWM = 19; // 19 represents OC2

    // Assign OC3 To Pin M2_AHI
    M3_PWM = 20; // 20 represents OC3
}

void Cell_Ctrl(BatteryChannel Channel, BatteryState state) {
    switch (Channel) {
    case CELL_A:
        Cell_A_MOS = state;
        break;
    case CELL_B:
        Cell_B_MOS = state;
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

void InterruptIni() {
    // TODO: delete this. We don't need to dynamically remap interrupts
    // remap all the interrupt routines
    T3InterruptUserFunction = Motor_T3Interrupt;
    ADC1InterruptUserFunction = Motor_ADC1Interrupt;
    U1TXInterruptUserFunction = uart_tx_isf;
    U1RXInterruptUserFunction = uart_rx_isf;
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

void PWM1Ini(void) {
    // 1. Configure the OCx output for one of the
    // available Peripheral Pin Select pins.
    // done in I/O Init.
    // 2. Calculate the desired duty cycles and load them
    // into the OCxR register.
    OC1R = 0;
    // 3. Calculate the desired period and load it into the
    // OCxRS register.
    /// period in units of ticks of the timer specified by the clock source OCTSEL
    OC1RS = 2000;
    // 4. Select the current OCx as the sync source by writing
    // 0x1F to SYNCSEL<4:0> (OCxCON2<4:0>),
    // and clearing OCTRIG (OCxCON2<7>).
    /// 0b11111 = This OC1 Module
    OC1CON2bits.SYNCSEL = 0x1F;
    /// 0 = Synchronize OC1 with Source designated with SYNCSEL1 bits
    OC1CON2bits.OCTRIG = CLEAR;
    // 5. Select a clock source by writing the
    // OCTSEL<2:0> (OCxCON<12:10>) bits.
    /// 0b000 = Timer2
    OC1CON1bits.OCTSEL = 0b000;
    // 6. Enable interrupts, if required, for the timer and
    // output compare modules. The output compare
    // interrupt is required for PWM Fault pin utilization.
    // No interrupt needed
    // 7. Select the desired PWM mode in the OCM<2:0>
    //(OCxCON1<2:0>) bits.
    /// 0b110 = Edge-Aligned PWM mode: Output set high when OCxTMR = 0 and set low when OCxTMR =
    /// OCxR
    OC1CON1bits.OCM = 0b110;
    // 8. If a timer is selected as a clock source, set the
    // TMRy prescale value and enable the time base by
    // setting the TON (TxCON<15>) bit.
    // Done in timer Init.
}

void PWM1Duty(uint16_t Duty) { OC1R = Duty * 2; }

void PWM2Ini(void) {
    OC2R = 0;
    OC2RS = 2000;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.OCTRIG = CLEAR;
    OC2CON1bits.OCTSEL = 0b000; // Timer2
    OC2CON1bits.OCM = 0b110;
}

void PWM2Duty(uint16_t Duty) { OC2R = Duty * 2; }

void PWM3Ini(void) {
    OC3R = 0;
    OC3RS = 2000;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.OCTRIG = CLEAR;
    OC3CON1bits.OCTSEL = 0b000; // Timer2
    OC3CON1bits.OCM = 0b110;
}

void PWM3Duty(uint16_t Duty) { OC3R = Duty * 2; }

/*****************************************************************************/
//*-----------------------------------Timer----------------------------------*/

void IniTimer1() {
    T1CON = 0x0000;         // clear register
    T1CONbits.TCKPS = 0b00; // 0b00 = 1:1 prescale
    TMR1 = 0;               // clear timer1 register
    PR1 = PERIOD_1000HZ;    // interrupt every 1ms
    T1CONbits.TON = SET;    // start timer
}

void IniTimer2() {
    T2CON = 0x0000;         // stops timer2,16 bit timer,internal clock (Fosc/2)
    T2CONbits.TCKPS = 0b00; // 0b00 = 1:1 prescale
    TMR2 = 0;
    PR2 = PERIOD_30000HZ;
    IFS0bits.T2IF = CLEAR; // clear interrupt flag
    T2CONbits.TON = SET;   // start timer
}
void IniTimer3() {
    T3CON = 0x0010;        // stops timer3,1:8 prescale,16 bit timer,internal clock (Fosc/2)
    TMR3 = 0;              // clear timer1 register
    PR3 = PERIOD_50000HZ;  // 50k hz / 8 prescale = 6250 Hz
    IFS0bits.T3IF = CLEAR; // clear interrupt flag
    IEC0bits.T3IE = SET;   // enable the interrupt
    T3CONbits.TON = SET;   // start timer
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

void Motor_T3Interrupt(void) {
    // TODO: I think the purpose here is just to enable the auto-sample bit ASAM
    // so that we don't have to set the SAMP bit and potentially set it at the wrong time.
    // I think this isn't needed as long as SSRC = 0b111

    // PORTCbits.RC13=~PORTCbits.RC13;
    // clear timer3 flage
    IFS0bits.T3IF = CLEAR; // clear interrupt flag
    timer3_count++;

    if (timer3_count >= 0) {
        timer3_count = 0;
        AD1CON1bits.ASAM = SET;
    }
}

void Motor_ADC1Interrupt(void) {
    // Clear interrupt flag
    IFS0bits.AD1IF = CLEAR;
    // disable auto-sample
    AD1CON1bits.ASAM = CLEAR;

    // harvest all the values from the ADC converter
    m3_posfb_array[0][m3_posfb_array_pointer] = ADC1BUF4;
    m3_posfb_array[1][m3_posfb_array_pointer] = ADC1BUF5;
    m3_posfb_array_pointer = (m3_posfb_array_pointer + 1) % SAMPLE_LENGTH;

    motor_current_ad[MOTOR_LEFT][motor_current_ad_pointer] = ADC1BUF3;
    motor_current_ad[MOTOR_RIGHT][motor_current_ad_pointer] = ADC1BUF1;
    motor_current_ad[MOTOR_FLIPPER][motor_current_ad_pointer] = ADC1BUFB;
    motor_current_ad_pointer = (motor_current_ad_pointer + 1) % SAMPLE_LENGTH;

    cell_current[CELL_A][total_cell_current_array_pointer] = ADC1BUF8;
    cell_current[CELL_B][total_cell_current_array_pointer] = ADC1BUF9;
    total_cell_current_array[total_cell_current_array_pointer] =
        cell_current[CELL_A][total_cell_current_array_pointer] +
        cell_current[CELL_B][total_cell_current_array_pointer];
    total_cell_current_array_pointer = (total_cell_current_array_pointer + 1) % SAMPLE_LENGTH;

    cell_voltage_array[CELL_A][cell_voltage_array_pointer] = ADC1BUF6;
    cell_voltage_array[CELL_B][cell_voltage_array_pointer] = ADC1BUF7;
    cell_voltage_array_pointer = (cell_voltage_array_pointer + 1) % SAMPLE_LENGTH;
}

static uint16_t return_calibrated_pot_angle(uint16_t pot_1_value, uint16_t pot_2_value) {
    uint16_t combined_pot_angle = 0;
    uint16_t calibrated_pot_angle = 0;

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

static uint16_t return_combined_pot_angle(uint16_t pot_1_value, uint16_t pot_2_value) {
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

    DataEEWrite((flipper_angle_offset >> 8), 0);
    DataEEWrite((flipper_angle_offset & 0xff), 1);

    // write 0xaa to index 2, so we can tell if this robot has been calibrated yet
    DataEEWrite(0xaa, 2);

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

        Cell_Ctrl(CELL_A, CELL_ON);
        Cell_Ctrl(CELL_B, CELL_ON);
        for (j = 0; j < k; j++)
            Nop();
        Cell_Ctrl(CELL_A, CELL_OFF);
        Cell_Ctrl(CELL_B, CELL_OFF);

        block_ms(10);

        k = k0 + i * i / 4;
    }

    Cell_Ctrl(CELL_A, CELL_ON);
    Cell_Ctrl(CELL_B, CELL_ON);
}

void turn_on_power_bus_old_method(void) {
    unsigned int i;

    for (i = 0; i < 20; i++) {
        Cell_Ctrl(CELL_A, CELL_ON);
        Cell_Ctrl(CELL_B, CELL_ON);
        block_ms(10);
        Cell_Ctrl(CELL_A, CELL_OFF);
        Cell_Ctrl(CELL_B, CELL_OFF);
        block_ms(40);
    }

    Cell_Ctrl(CELL_A, CELL_ON);
    Cell_Ctrl(CELL_B, CELL_ON);
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
            Cell_Ctrl(CELL_A, CELL_ON);
            Cell_Ctrl(CELL_B, CELL_ON);
            for (j = 0; j < k; j++)
                Nop();
            Cell_Ctrl(CELL_A, CELL_OFF);
            Cell_Ctrl(CELL_B, CELL_OFF);
            break;
        }

        block_ms(40);
        k = k0 + i * i / 4;
        if (k > 20000)
            k = 20000;
    }

    Cell_Ctrl(CELL_A, CELL_ON);
    Cell_Ctrl(CELL_B, CELL_ON);
}

const char DEVICE_NAME_OLD_BATTERY[7] = "BB-2590";
const char DEVICE_NAME_NEW_BATTERY[9] = "BT-70791B";
const char DEVICE_NAME_BT70791_CK[9] = "BT-70791C";
const char DEVICE_NAME_CUSTOM_BATTERY[7] = "ROBOTEX";

void power_bus_init(void) {
#define BATTERY_DATA_LEN 10
    char battery_data1[BATTERY_DATA_LEN] = {0};
    char battery_data2[BATTERY_DATA_LEN] = {0};
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
    if (strcmp(DEVICE_NAME_OLD_BATTERY, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_OLD_BATTERY, battery_data2) == 0) {
        turn_on_power_bus_old_method();
        return;
    }

    // If we're using the new battery (BT-70791B)
    if (strcmp(DEVICE_NAME_NEW_BATTERY, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_NEW_BATTERY, battery_data2) == 0) {
        turn_on_power_bus_new_method();
        return;
    }

    // If we're using Bren-Tronics BT-70791C
    if (strcmp(DEVICE_NAME_BT70791_CK, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_BT70791_CK, battery_data2) == 0) {
        turn_on_power_bus_new_method();
        return;
    }

    // if we're using the low lithium custom Matthew's battery
    if (strcmp(DEVICE_NAME_CUSTOM_BATTERY, battery_data1) == 0 ||
        strcmp(DEVICE_NAME_CUSTOM_BATTERY, battery_data2) == 0) {
        turn_on_power_bus_old_method();
        return;
    }

    // if we're using an unknown battery
    turn_on_power_bus_hybrid_method();
}

// When robot is on the charger, only leave one side of the power bus on at a time.
// This is so that one side of a battery doesn't charge the other side through the power bus.
static void power_bus_tick(void) {
    static BatteryChannel active_battery = CELL_A;
    static Counter counter = {.max = 10000};

    if (REG_MOTOR_CHARGER_STATE == 0xdada) {
        if (counter_tick(&counter) == COUNTER_EXPIRED) {
            // Toggle active battery
            active_battery = (active_battery == CELL_A ? CELL_B : CELL_A);
        }

        // if we're on the dock, turn off one side of the power bus
        switch (active_battery) {
        case CELL_A:
            Cell_Ctrl(CELL_A, CELL_ON);
            Cell_Ctrl(CELL_B, CELL_OFF);
            break;
        case CELL_B:
            Cell_Ctrl(CELL_B, CELL_ON);
            Cell_Ctrl(CELL_A, CELL_OFF);
            break;
        }
    } else {
        // Not charging. use both batteries
        Cell_Ctrl(CELL_B, CELL_ON);
        Cell_Ctrl(CELL_A, CELL_OFF);
    }
}
