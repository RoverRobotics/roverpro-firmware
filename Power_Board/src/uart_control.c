#include "stdhdr.h"
#include "uart_control.h"
#include "device_robot_motor.h"

#define UART_SEND_BUFFER_LENGTH 4
#define UART_RECEIVE_BUFFER_LENGTH 6
const uint8_t UART_START_BYTE = 253;

// based on 32MHz system clock rate
enum UART_BAUD_RATE {
    BaudRate_9600_LOW = 103, // BRGH=0
    BaudRate_57600_LOW = 16, // BRGH=0
    BaudRate_57600_HI = 68,  // BRGH=1
    BaudRate_115200_HI = 34, // BRGH=1
};
bool uart_FanSpeedTimerEnabled = false;
bool uart_fan_speed_expired = false;
int uart_FanSpeedTimerCount = 0;

typedef enum UARTState { UART_STATE_IDLE, UART_STATE_PROCESSING, UART_STATE_DONE } UARTState;
uint8_t uart_receive_buffer[UART_RECEIVE_BUFFER_LENGTH];
int16_t i_uart_receive_buffer = 0;

uint8_t uart_send_buffer[UART_SEND_BUFFER_LENGTH];
int16_t i_uart_send_buffer = 0;

uint8_t uart_data_identifier = 0;
int16_t uart_motor_velocity[3];
uint8_t uart_incoming_cmd[2];
uint16_t UART_BUILD_NUMBER = 40621;
bool uart_flipper_calibrate_requested = false;
bool uart_has_new_fan_speed = false;
bool uart_has_new_data = false;
static bool has_seen_start_byte = false;

typedef enum UARTCommand {
    UART_COMMAND_GET = 10,
    UART_COMMAND_SET_FAN_SPEED = 20,
    UART_COMMAND_SET_MOTOR_SLOW_SPEED = 240,
    UART_COMMAND_FLIPPER_CALIBRATE = 250,
} UARTCommand;

//*----------------------------------UART1------------------------------------*/

void uart_init() {
    // Write appropriate baud rate value to the UxBRG register.
    U1BRG = BaudRate_57600_HI;
    // Enable the UART.
    U1MODE = 0x0000;
    U1MODEbits.BRGH = 1; // High Baud Rate Select bit = High speed (low speed = 16x BRG)
    U1STA = 0x0000;
    U1MODEbits.UARTEN = 1; // UART1 is enabled
    U1STAbits.UTXEN = 1;   // transmit enabled
    IEC0bits.U1TXIE = 1;   // enable UART1 transmit interrupt
    IEC0bits.U1RXIE = 1;   // enable UART1 receive interrupt
}

void uart_rx_isf() {
    uint8_t a_byte;
    // clear the interrupt flag
    IFS0bits.U1RXIF = 0;
    while (U1STAbits.URXDA) {
        // While new received data is available
        a_byte = (uint8_t)U1RXREG;

        if (i_uart_receive_buffer == -1 && a_byte == UART_START_BYTE) {
            // Throw out the newly read byte but move to the beginning of the read buffer
            i_uart_receive_buffer++;
        } else if (i_uart_receive_buffer < UART_RECEIVE_BUFFER_LENGTH) {
            // save the newly read byte
            uart_receive_buffer[i_uart_receive_buffer] = a_byte;
            i_uart_receive_buffer++;
        } else {
            // receive buffer full. throwing out new data
            BREAKPOINT();
        }
    }
    U1STAbits.OERR = 0;
}

void uart_tx_isf() {
    // i_uart_send_buffer
    // -1 = ready to send start byte
    // 0..UART_SEND_BUFFER_LEN-1 = ready to send data
    // UART_SEND_BUFFER_LEN = done sending data. buffer is empty
    uint8_t a_byte;
    // clear the interrupt flag
    IFS0bits.U1TXIF = 0;
    // transmit data
    while (!U1STAbits.UTXBF) {
        // while transmit buffer is not full
        if (i_uart_send_buffer == -1) {
            // Transmit the start byte
            a_byte = UART_START_BYTE;
            i_uart_send_buffer++;
        } else if (i_uart_send_buffer < UART_SEND_BUFFER_LENGTH) {
            // transmit the next data byte
            a_byte = uart_send_buffer[i_uart_send_buffer];
            i_uart_send_buffer++;
        } else {
            // no data to send
            a_byte = 0;
            U1STAbits.UTXEN = 0;
        }
        U1TXREG = a_byte;
    }
}

void uart_serialize_out_data(uint8_t arg) {

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
        uart_send_buffer[1] = 0;
        uart_send_buffer[2] = 0;
        break;
    }
#undef CASE
}

void uart_tick() {
    if (i_uart_receive_buffer >= UART_RECEIVE_BUFFER_LENGTH) // end of the package
    {
        int i;
        int SumBytes = 0;
        UARTCommand command;
        uint8_t arg;
        for (i = 0; i < UART_RECEIVE_BUFFER_LENGTH; i++) {
            SumBytes += uart_receive_buffer[i];
        }
        if (SumBytes % 255 != 0) {
            i_uart_receive_buffer = 0;
            has_seen_start_byte = false;
            // checksum failed. Ignore this packet
            return;
        }

        uart_motor_velocity[MOTOR_LEFT] = (uart_receive_buffer[0] * 8 - 1000);
        uart_motor_velocity[MOTOR_RIGHT] = (uart_receive_buffer[1] * 8 - 1000);
        uart_motor_velocity[MOTOR_FLIPPER] = (uart_receive_buffer[2] * 8 - 1000);
        command = uart_receive_buffer[3];
        arg = uart_receive_buffer[4];

        // reset the receive buffer
        i_uart_receive_buffer = 0;
        has_seen_start_byte = false;

        switch (command) {
        case UART_COMMAND_SET_FAN_SPEED: // new fan command coming in
            REG_MOTOR_SIDE_FAN_SPEED = arg;
            uart_has_new_fan_speed = true;
            // Enable fan speed timer
            uart_FanSpeedTimerEnabled = true;
            uart_FanSpeedTimerCount = 0;
            break;
        case UART_COMMAND_SET_MOTOR_SLOW_SPEED:
            REG_MOTOR_SLOW_SPEED = arg;
            break;
        case UART_COMMAND_FLIPPER_CALIBRATE:
            if (arg == UART_COMMAND_FLIPPER_CALIBRATE) {
                uart_flipper_calibrate_requested = true;
            }
            break;
        case UART_COMMAND_GET:
            // todo: don't do anything if send buffer is not empty
            uart_send_buffer[0] = arg;
            uart_serialize_out_data(arg);
            uart_send_buffer[3] = (uint8_t)(
                255 - (uart_send_buffer[0] + uart_send_buffer[1] + uart_send_buffer[2]) % 255);

            i_uart_send_buffer = -1;
            U1STAbits.UTXEN = 1;

        default:
            BREAKPOINT();
        }
        uart_has_new_data = true;
    }
}
