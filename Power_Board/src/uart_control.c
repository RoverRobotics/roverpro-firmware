#include "stdhdr.h"
#include "uart_control.h"
#include "device_robot_motor.h"

#define UART_SEND_BUFFER_LENGTH 4
#define UART_RECEIVE_BUFFER_LENGTH 6
const uint8_t UART_START_BYTE = 253;

bool uart_FanSpeedTimerEnabled = false;
bool uart_fan_speed_expired = false;
int uart_FanSpeedTimerCount = 0;

typedef enum UARTState { UART_STATE_IDLE, UART_STATE_PROCESSING } UArtState;
uint8_t uart_receive_buffer[UART_RECEIVE_BUFFER_LENGTH];
int i_uart_receive_buffer = 0;
// no more than 5 bytes based on 57600 baud and 1ms system loop

uint8_t uart_send_buffer[UART_SEND_BUFFER_LENGTH];
uint8_t i_uart_send_buffer = 0;
uint8_t uart_data_identifier = 0;
int16_t uart_motor_velocity[3];
uint8_t uart_incoming_cmd[2];
uint16_t UART_BUILD_NUMBER = 40621;

typedef enum UARTCommand {
    UART_COMMAND_GET = 10,
    UART_COMMAND_SET_FAN_SPEED = 20,
    UART_COMMAND_SET_MOTOR_SLOW_SPEED = 240,
    UART_COMMAND_FLIPPER_CALIBRATE = 250,
} UARTCommand;


void uart_rx_isf(){
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

void uart_tx_isf(){
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

void uart_tick(){

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
            CASE(28, REG_MOTOR_FB_PERIOD_LEFT)
            CASE(30, REG_MOTOR_FB_PERIOD_RIGHT)
            CASE(32, REG_MOTOR_FB_PERIOD_FLIPPER)
            CASE(34, REG_ROBOT_REL_SOC_A)
            CASE(36, REG_ROBOT_REL_SOC_B)
            CASE(38, REG_MOTOR_CHARGER_STATE)
            CASE(40, UART_BUILD_NUMBER)
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

}