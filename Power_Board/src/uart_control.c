#include "stdhdr.h"
#include "uart_control.h"
#include "device_robot_motor.h"

#define UART_TX_BUFFER_LENGTH 4
#define UART_RX_BUFFER_LENGTH 6
const uint8_t UART_START_BYTE = 253;

// based on 32MHz system clock rate
enum UARTBaudRate {
    BAUD_RATE_9600_LOW = 103, // BRGH=0
    BAUD_RATE_57600_LOW = 16, // BRGH=0
    BAUD_RATE_57600_HI = 68,  // BRGH=1
    BAUD_RATE_115200_HI = 34, // BRGH=1
};

/// Software buffer for receiving UART data
uint8_t uart_rx_buffer[UART_RX_BUFFER_LENGTH];

/// index into UART receive buffer
/// -1 = ready to receive start byte
/// 0 ... UART_RX_BUFFER_LENGTH-1 = ready to receive the i'th data byte
/// UART_RX_BUFFER_LENGTH = Done sending data. Buffer contents are stable and can be safely read
int16_t i_uart_rx_buffer = -1;

/// Software buffer for transmitting UART data
uint8_t uart_tx_buffer[UART_TX_BUFFER_LENGTH];

/// index into UART transmit buffer
/// -1 = ready to send start byte
/// 0 ... UART_TX_BUFFER_LENGTH-1 = ready to send the i'th data byte
/// UART_TX_BUFFER_LENGTH = Done sending data. Buffer contents are stable and can be safely overwritten
int16_t i_uart_tx_buffer = UART_RX_BUFFER_LENGTH;

const uint16_t UART_BUILD_NUMBER = 40621;

/// 1-byte command "verb" associated with UART inbound command.
/// There is also a 1-byte argument associated
typedef enum UARTCommand {
  /// Robot should return the data element specified by argument
    UART_COMMAND_GET = 10,
    /// Robot should set the fan speed to argument
    UART_COMMAND_SET_FAN_SPEED = 20,
    /// Robot should set the closed loop mode to argument
    UART_COMMAND_SET_CLOSED_LOOP = 240,
    /// Robot should calibrate the flipper
    UART_COMMAND_FLIPPER_CALIBRATE = 250,
} UARTCommand;

void uart_init() {
    // Write appropriate baud rate value to the UxBRG register.
    U1BRG = BAUD_RATE_57600_HI;
    // Enable the UART.
    U1MODE = 0x0000;
    U1MODEbits.BRGH = 1; // High Baud Rate Select bit = High speed (low speed = 16x BRG)
    U1STA = 0x0000;
    U1MODEbits.UARTEN = 1; // UART1 is enabled
    IEC0bits.U1TXIE = 1;   // enable UART1 transmit interrupt
    IEC0bits.U1RXIE = 1;   // enable UART1 receive interrupt
}

// Configure interrupts.
// Hardware UART RX interrupt enabled by U1RXIE
void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt() { uart_rx_isf(); };
// Hardware UART TX interrupt, enabled by U1TXIE
void __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt() { uart_tx_isf(); };

void uart_rx_isf() {
    // clear the interrupt flag
    IFS0bits.U1RXIF = 0;
    // While new received data is available
    while (U1STAbits.URXDA && i_uart_rx_buffer < UART_RX_BUFFER_LENGTH) {
        // Pop the next byte of the read queue
        uint8_t a_byte = (uint8_t) U1RXREG;
        if (i_uart_rx_buffer == -1 && a_byte == UART_START_BYTE) {
            // If we have not seen a start byte and the next byte is a start byte,
            // Throw out the newly read byte and move to the beginning of the read buffer
            i_uart_rx_buffer = 0;
        } else if (i_uart_rx_buffer > -1) {
            // If we are getting a data byte, save it to the read buffer and increment the value
            uart_rx_buffer[i_uart_rx_buffer] = a_byte;
            i_uart_rx_buffer++;
        }
    }
    // check Receive Buffer Overrun Error Status bit
    if (U1STAbits.OERR ) {
        BREAKPOINT();
        // clear overflow error bit if it was set.
        U1STAbits.OERR = 0;
    }
}

void uart_tx_isf() {
    // clear the interrupt flag
    IFS0bits.U1TXIF = 0;
    // transmit data
    while (!U1STAbits.UTXBF) {
        // while transmit buffer is not full
        if (i_uart_tx_buffer == -1) {
            // Transmit the start byte
            U1TXREG = UART_START_BYTE;
            i_uart_tx_buffer = 0;
        } else if (i_uart_tx_buffer < UART_TX_BUFFER_LENGTH) {
            // Transmit the next data byte
            U1TXREG = uart_tx_buffer[i_uart_tx_buffer];
            i_uart_tx_buffer++;
        } else {
            // No data to send
			return;
        }
    }
}

void uart_serialize_out_data(uint8_t uart_data_identifier) {

// CASE(n, REGISTER) populates the UART output buffer with the 16-bit integer value of the given
// register and breaks out of the switch statement
#define CASE(n, REGISTER)                                                                          \
    case (n):                                                                                      \
        uart_tx_buffer[1] = (uint8_t)((REGISTER) >> 8 & 0xff);                                   \
        uart_tx_buffer[2] = (uint8_t)(REGISTER & 0xff);                                          \
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
        uart_tx_buffer[1] = REG_MOTOR_FAULT_FLAG.left;
        uart_tx_buffer[2] = REG_MOTOR_FAULT_FLAG.right;
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
        CASE(50, REG_MOTOR_CLOSED_LOOP)
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
        uart_tx_buffer[1] = 0;
        uart_tx_buffer[2] = 0;
        break;
    }
#undef CASE
}

UArtTickResult uart_tick() {
    UArtTickResult result = {0};

    if (i_uart_rx_buffer >= UART_RX_BUFFER_LENGTH) // end of the package
    {
        int i;
        int sum_bytes = 0;
        UARTCommand command;
        uint8_t arg;
        for (i = 0; i < UART_RX_BUFFER_LENGTH; i++) {
            sum_bytes += uart_rx_buffer[i];
        }
        if (sum_bytes % 255 != 0) {
            i_uart_rx_buffer = -1;
            // checksum failed. Ignore this packet
            return result;
        }

        REG_MOTOR_VELOCITY.left = (uart_rx_buffer[0] * 8 - 1000);
        REG_MOTOR_VELOCITY.right = (uart_rx_buffer[1] * 8 - 1000);
        REG_MOTOR_VELOCITY.flipper = (uart_rx_buffer[2] * 8 - 1000);
        command = uart_rx_buffer[3];
        arg = uart_rx_buffer[4];

        // We have used all the data in the receive buffer.
        // Mark the receive buffer as empty.
        i_uart_rx_buffer = -1;

        switch (command) {
        case UART_COMMAND_SET_FAN_SPEED: // new fan command coming in
            REG_MOTOR_SIDE_FAN_SPEED = arg;
            result.uart_fan_speed_requested = true;
            break;
        case UART_COMMAND_SET_CLOSED_LOOP:
            REG_MOTOR_CLOSED_LOOP = arg;
            result.uart_motor_control_scheme_requested = true;
            break;
        case UART_COMMAND_FLIPPER_CALIBRATE:
            if (arg == UART_COMMAND_FLIPPER_CALIBRATE) {
                result.uart_flipper_calibrate_requested = true;
            }
            break;
        case UART_COMMAND_GET:
            if (i_uart_tx_buffer < UART_TX_BUFFER_LENGTH) {
                // we still have data in the send buffer!
                BREAKPOINT();
            } else {
                uart_tx_buffer[0] = arg;
                uart_serialize_out_data(arg);
                uart_tx_buffer[3] = (uint8_t)(
                    255 - (uart_tx_buffer[0] + uart_tx_buffer[1] + uart_tx_buffer[2]) % 255);

                // We have populated the send buffer
                // Mark the send buffer as ready to be sent
                i_uart_tx_buffer = -1;
				
				// If Transmit Shift Register is Empty, we aren't waiting on an interrupt and should start
				// the transmission now.
				if (U1STAbits.TRMT) {
					// Start the transmission
					uart_tx_isf();
				}
            }
            break;
        default:
            BREAKPOINT();
            // unknown inbound command.
        }
        result.uart_motor_speed_requested = true;
    }

    return result;
}
