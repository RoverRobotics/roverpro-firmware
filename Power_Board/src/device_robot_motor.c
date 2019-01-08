/// @file high-level motor control and main event loop.

#include <p24Fxxxx.h>
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "interrupt_switch.h"
#include "device_robot_motor_i2c.h"
#include "DEE Emulation 16-bit.h"
#include "i2clib.h"
#include "device_robot_motor_loop.h"
#include "motor.h"
#include "uart_control.h"
#include "counter.h"
#include "device_power_bus.h"
#include "math.h"

bool SHOULD_SKIP_BOOTLOADER __attribute__((persistent, address(0x400)));

//****************************************************
Counter speed_update_timer[MOTOR_CHANNEL_COUNT] = {
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
    {.max = INTERVAL_MS_SPEED_UPDATE, .pause_on_expired = true},
};

/// count of how many millis since last inbound motor command. When this expires, we should stop the
/// motors
Counter motor_speed_timeout = {
    .max = INTERVAL_MS_USB_TIMEOUT, .pause_on_expired = true, .is_paused = true};
/// count of how many millis since last motor direction update.
Counter motor_direction_state_machine = {.max = INTERVAL_MS_MOTOR_DIRECTION_STATE_MACHINE};
/// count of how many millis since last time we measured the current to the motors
Counter current_fb = {.max = INTERVAL_MS_CURRENT_FEEDBACK};
/// count of how many millis since I2C2 finished. If we don't reach the end, we want to forcibly
/// restart the I2C2 bus
Counter i2c2_timeout = {.max = INTERVAL_MS_I2C2};
/// count of how many millis since I2C3 finished. If we don't reach the end, we want to forcibly
/// restart the I2C3 bus
Counter i2c3_timeout = {.max = INTERVAL_MS_I2C3};
/// count of how many millis since we have updated various outbound diagnostic data registers
Counter sfreg_update = {.max = INTERVAL_MS_SFREG};
/// count of how many millis since we have checked the battery currents and whether we are drawing
/// too much current
Counter battery_check = {.max = INTERVAL_MS_BATTERY_CHECK};
/// how many millis since we entered an overcurrent condition. Disable the overcurrent condition
/// when this expires
Counter over_current_recovery = {
    .max = INTERVAL_MS_BATTERY_RECOVER, .pause_on_expired = true, .is_paused = true};
/// how many millis since we last ran the PID filter to compute motor effort for closed loop control
Counter closed_loop_control = {.max = 10};
Counter uart_fan_speed_timeout = {
    .max = INTERVAL_MS_UART_FAN_SPEED_TIMEOUT, .pause_on_expired = true, .is_paused = true};

uint16_t motor_current_ad[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH] = {{0}};
int motor_current_ad_pointer = 0;
uint16_t current_for_control[MOTOR_CHANNEL_COUNT][SAMPLE_LENGTH_CONTROL] = {{0}};
int current_for_control_pointer[MOTOR_CHANNEL_COUNT] = {0};
int16_t motor_efforts[MOTOR_CHANNEL_COUNT] = {0};

int m3_posfb_array[2][SAMPLE_LENGTH] = {{0}}; ///< Flipper Motor positional feedback data
int m3_posfb_array_pointer = 0;
uint16_t total_cell_current_array[SAMPLE_LENGTH] = {0};
int total_cell_current_array_pointer = 0;
int cell_voltage_array[BATTERY_CHANNEL_COUNT][SAMPLE_LENGTH];
int cell_voltage_array_pointer = 0;
uint16_t cell_current[BATTERY_CHANNEL_COUNT][SAMPLE_LENGTH];

bool over_current = false;
uint16_t flipper_angle_offset = 0;
void calibrate_flipper_angle_sensor(void);
static void read_stored_angle_offset(void);

void set_firmware_build_time(void);

static uint16_t return_combined_pot_angle(uint16_t pot_1_value, uint16_t pot_2_value);
static uint16_t return_calibrated_pot_angle(uint16_t pot_1_value, uint16_t pot_2_value);

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

    // I/O initializing complete

    // Initialize motor drivers
    //*******************************************

    InterruptIni();
    IniAD();

    // initialize timers
    IniTimer2();
    IniTimer3();
    IniTimer1();

    // initialize motor control
    MotorsInit();
    // initialize motor tachometer feedback
    motor_tach_init();

    i2c_enable(I2C_BUS2);
    i2c_enable(I2C_BUS3);

    uart_init();

    set_firmware_build_time();

    power_bus_init();
    FANCtrlIni();

    // read flipper position from flash, and put it into a module variable
    read_stored_angle_offset();

    // init variables for closed loop control
    closed_loop_control_init();
}

void Device_MotorController_Process() {
    UArtTickResult uart_tick_result;
    MotorChannel i;
    long temp1, temp2;

    if (IFS0bits.T1IF == SET) {
        // Clear interrupt flag
        IFS0bits.T1IF = CLEAR;

        // this should run every 1ms
        power_bus_tick();

        // check if any timers are expired

        if (counter_tick(&closed_loop_control) == COUNTER_EXPIRED) {
            pid_tick(over_current);
        }

        // if any of the timers expired, execute relative codes
        // Control timer expired
        for (EACH_MOTOR_CHANNEL(i)) {
            if (counter_tick(&speed_update_timer[i]) == COUNTER_EXPIRED) {
                if (over_current)
                    Coasting(i);
            } else {
                UpdateSpeed(i, motor_efforts[i]);
            }
        }
    }
    if (counter_tick(&current_fb) == COUNTER_EXPIRED) {
        /// Compute the current used by a given motor and populate it in current_for_control
        /// This is computed as a windowed average of the values in motor_current_ad
        for (EACH_MOTOR_CHANNEL(i)) {
            current_for_control[i][current_for_control_pointer[i]] =
                mean_u(SAMPLE_LENGTH, motor_current_ad[i]);
            current_for_control_pointer[i] =
                (current_for_control_pointer[i] + 1) % SAMPLE_LENGTH_CONTROL;
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
        REG_MOTOR_FB_CURRENT.left = mean_u(SAMPLE_LENGTH_CONTROL, current_for_control[MOTOR_LEFT]);
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

        // read out measured motor periods.
        REG_MOTOR_FB_PERIOD_LEFT = (uint16_t)fabs(motor_tach_get_period(MOTOR_LEFT));
        REG_MOTOR_FB_PERIOD_RIGHT = (uint16_t)fabs(motor_tach_get_period(MOTOR_RIGHT));
    }

    if (counter_tick(&battery_check) == COUNTER_EXPIRED) {
        static int overcurrent_counter = 0;
        REG_PWR_A_CURRENT = mean_u(SAMPLE_LENGTH, cell_current[CELL_A]);
        REG_PWR_B_CURRENT = mean_u(SAMPLE_LENGTH, cell_current[CELL_B]);

        //.01*.001mV/A * 11000 ohms = .11 V/A = 34.13 ADC counts/A
        // set at 10A per side
        if (REG_PWR_A_CURRENT >= 512 || REG_PWR_B_CURRENT >= 512) {
            overcurrent_counter++;
            if (overcurrent_counter > 10) {
                Coasting(MOTOR_LEFT);
                Coasting(MOTOR_RIGHT);
                Coasting(MOTOR_FLIPPER);
                over_current = true;
                counter_restart(&over_current_recovery);
            }

        } else if (REG_PWR_A_CURRENT <= 341 && REG_PWR_B_CURRENT <= 341) {
            overcurrent_counter = 0;
        }
    }

    if (counter_tick(&over_current_recovery) == COUNTER_EXPIRED) {
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
    set_motor_control_scheme();

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
        counter_restart(&motor_speed_timeout);
    }
    // long time no data, clear everything and stop moving
    if (counter_tick(&motor_speed_timeout) == COUNTER_EXPIRED) {
        REG_MOTOR_VELOCITY.left = 0;
        REG_MOTOR_VELOCITY.right = 0;
        REG_MOTOR_VELOCITY.flipper = 0;

        for (EACH_MOTOR_CHANNEL(i)) {
            motor_efforts[i] = 0;
        }
    }

    // update motor direction state machine
    // If the new motor commands reverse or brake a motor, coast that motor for a few ticks
    // (motor_state[i] = SWITCH_DIRECTION) Then once the switch direction timer elapses, start
    // the motor in that new direction. If we are moving forward or in reverse and the intended
    // speed changes, update the pwm command to the motor
    if (counter_tick(&motor_direction_state_machine) == COUNTER_EXPIRED) {
        static Counter motor_switch_direction_timer[MOTOR_CHANNEL_COUNT] = {
            {.max = INTERVAL_MS_SWITCH_DIRECTION / INTERVAL_MS_MOTOR_DIRECTION_STATE_MACHINE,
             .pause_on_expired = true,
             .is_paused = true},
            {.max = INTERVAL_MS_SWITCH_DIRECTION / INTERVAL_MS_MOTOR_DIRECTION_STATE_MACHINE,
             .pause_on_expired = true,
             .is_paused = true},
            {.max = INTERVAL_MS_SWITCH_DIRECTION / INTERVAL_MS_MOTOR_DIRECTION_STATE_MACHINE,
             .pause_on_expired = true,
             .is_paused = true},
        };

        /// The outbound commands we are sending to the motor
        static MotorState motor_state[MOTOR_CHANNEL_COUNT] = {SWITCH_DIRECTION, SWITCH_DIRECTION,
                                                              SWITCH_DIRECTION};
        static MotorEvent motor_event[MOTOR_CHANNEL_COUNT] = {STOP, STOP, STOP};

        for (EACH_MOTOR_CHANNEL(i)) {
            /// The requested motor state based in inbound motor speed commands.
            if (motor_efforts[i] == 0)
                motor_event[i] = STOP;
            else if (-1024 < motor_efforts[i] && motor_efforts[i] < 0)
                motor_event[i] = BACK;
            else if (0 < motor_efforts[i] && motor_efforts[i] < 1024)
                motor_event[i] = GO;
            else {
                BREAKPOINT();
            }

            // update state machine
            switch (motor_state[i]) {
            case BRAKE:
                // brake motor
                Braking(i);
                switch (motor_event[i]) {
                case GO: // fallthrough
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
                    counter_resume(&speed_update_timer[i]);
                    break;
                case BACK: // fallthrough
                case STOP:
                    motor_state[i] = SWITCH_DIRECTION;
                    counter_stop(&speed_update_timer[i]);
                    counter_restart(&motor_switch_direction_timer[i]);
                    Coasting(i);
                    break;
                case NO_EVENT:
                    break;
                }
                break;
            case BACKWARD:
                switch (motor_event[i]) {
                case GO: // fallthrough
                case STOP:
                    motor_state[i] = SWITCH_DIRECTION;
                    counter_stop(&speed_update_timer[i]);
                    counter_restart(&motor_switch_direction_timer[i]);
                    Coasting(i);
                    break;
                case BACK:
                    counter_resume(&speed_update_timer[i]);
                    break;
                case NO_EVENT:
                    break;
                }
                break;
            case SWITCH_DIRECTION:
                counter_resume(&motor_switch_direction_timer[i]);
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

//*********************************************//

void set_motor_control_scheme() {
    static uint16_t control_loop_counter = 0, flipper_control_loop_counter = 0;

    typedef enum {
        /// We are in normal control mode
        OPEN_LOOP = 0,
        /// We were in normal control mode and have received a request to change to low speed mode
        OPEN_TO_CLOSED_LOOP,
        /// We are in low speed (PID) mode
        CLOSED_LOOP,
        /// We were in low speed mode and have received a request to switch to normal control mode
        CLOSED_TO_OPEN_LOOP
    } motor_control_scheme;

    /// The motor control scheme we are currently using.
    /// Contrast with REG_MOTOR_CLOSED_LOOP, which is the control scheme we have last been requested
    /// to use.
    static motor_control_scheme scheme = OPEN_LOOP;

    // If switching between open loop and closed loop control schemes (REG_MOTOR_CLOSED_LOOP)
    // Motors must decelerate upon switching speeds (if they're not already stopped.
    // Failure to do so could cause large current spikes which will trigger the
    // protection circuitry in the battery, cutting power to the robot
    switch (scheme) {
    case OPEN_LOOP:
        motor_efforts[MOTOR_LEFT] = REG_MOTOR_VELOCITY.left;
        motor_efforts[MOTOR_RIGHT] = REG_MOTOR_VELOCITY.right;
        motor_efforts[MOTOR_FLIPPER] = REG_MOTOR_VELOCITY.flipper;

        if (REG_MOTOR_CLOSED_LOOP)
            scheme = OPEN_TO_CLOSED_LOOP;
        break;
    case OPEN_TO_CLOSED_LOOP:
        control_loop_counter++;

        if (control_loop_counter > 3) {
            control_loop_counter = 0;

            // set speed to 1 (speed of 0 immediately stops motors)
            motor_efforts[MOTOR_LEFT] = 1;
            motor_efforts[MOTOR_RIGHT] = 1;
            motor_efforts[MOTOR_FLIPPER] = 1;
        }

        // motors are stopped if speed falls below 1%
        if ((abs(motor_efforts[MOTOR_LEFT]) < 10) && (abs(motor_efforts[MOTOR_RIGHT]) < 10) &&
            (abs(motor_efforts[MOTOR_FLIPPER]) < 10))
            scheme = CLOSED_LOOP;

        break;
    case CLOSED_LOOP:
        pid_set_desired_velocities(REG_MOTOR_VELOCITY.left, REG_MOTOR_VELOCITY.right,
                                   REG_MOTOR_VELOCITY.flipper);
        motor_efforts[MOTOR_LEFT] = return_closed_loop_control_effort(MOTOR_LEFT);
        motor_efforts[MOTOR_RIGHT] = return_closed_loop_control_effort(MOTOR_RIGHT);
        motor_efforts[MOTOR_FLIPPER] = return_closed_loop_control_effort(MOTOR_FLIPPER);

        control_loop_counter = 0;
        flipper_control_loop_counter = 0;

        if (!REG_MOTOR_CLOSED_LOOP)
            scheme = CLOSED_TO_OPEN_LOOP;

        break;

    case CLOSED_TO_OPEN_LOOP:
        pid_set_desired_velocities(0, 0, 0);

        motor_efforts[MOTOR_LEFT] = return_closed_loop_control_effort(MOTOR_LEFT);
        motor_efforts[MOTOR_RIGHT] = return_closed_loop_control_effort(MOTOR_RIGHT);
        motor_efforts[MOTOR_FLIPPER] = return_closed_loop_control_effort(MOTOR_FLIPPER);

        // motors are stopped if speed falls below 1%
        if ((abs(motor_efforts[MOTOR_LEFT]) < 10) && (abs(motor_efforts[MOTOR_RIGHT]) < 10) &&
            (abs(motor_efforts[MOTOR_FLIPPER]) < 10))
            scheme = OPEN_LOOP;

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

    block_ms(4000);

    // return fan to low duty cycle
    a_byte = 0;
    i2c_synchronously_await(I2C_BUS2, i2c_op_write_byte(FAN_CONTROLLER_ADDRESS, 0x0b, &a_byte));
}

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
    // clear timer3 flag
    IFS0bits.T3IF = CLEAR; // clear interrupt flag
    AD1CON1bits.ASAM = SET;
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
    for (i = 0; i < 3; i++) {
        angle_data[i] = DataEERead(i);
    }

    // only use stored values if we have stored the calibrated values before.
    // we store 0xaa in position 8 so we know that the calibration has taken place.
    if (angle_data[2] == 0xaa) {
        flipper_angle_offset = angle_data[0] * 256 + angle_data[1];
    }
}

void calibrate_flipper_angle_sensor(void) {
    MotorChannel i;
    // Coast all the motors
    for (EACH_MOTOR_CHANNEL(i)) {
        Coasting(i);
    }
    flipper_angle_offset =
        return_combined_pot_angle(REG_FLIPPER_FB_POSITION.pot1, REG_FLIPPER_FB_POSITION.pot2);

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
