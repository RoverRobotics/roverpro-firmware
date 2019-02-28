#include "xc.h"
#include "main.h"
#include "uart_control.h"
#include "device_robot_motor.h"
#include "settings.h"
#include "drive.h"
#include "version.GENERATED.h"
#include "flipper.h"

#define RX_PACKET_SIZE 7
#define TX_PACKET_SIZE 5

static uint16_t ticks_since_last_drive_command = UINT16_MAX;
static uint16_t ticks_since_last_fan_command = UINT16_MAX;

/// A ring queue used to buffer incoming and outgoing data
/// It is safe for data to be enqueued by one function and dequeued by an interrupt or vice versa.
typedef struct Queue {
    /// size of the buffer that data will be stored in. Should be a power of 2
    const size_t buflen;
    /// number of bytes that have been enqueued over the lifetime of this queue
    volatile int16_t n_enqueued;
    /// number of bytes that have been dequeued over the lifetime of this queue
    volatile int16_t n_dequeued;
    /// physical buffer that the data will be stored in
    uint8_t *const buffer;
} Queue;

/// @return number of bytes that can be enqueued
int16_t n_can_enqueue(Queue *q) { return q->buflen - q->n_enqueued + q->n_dequeued; };
/// @return number of bytes that can be dequeued
int16_t n_can_dequeue(Queue *q) { return q->n_enqueued - q->n_dequeued; };

/// Add one byte to the queue
void enqueue(Queue *q, uint8_t value) {
    if (n_can_enqueue(q) >= 1) {
        q->buffer[q->n_enqueued % q->buflen] = value;
        q->n_enqueued++;
    } else {
        BREAKPOINT();
    }
};

/// Remove one byte from the queue
/// @return the byte removed
uint8_t dequeue(Queue *q) {
    if (n_can_dequeue(q) >= 1) {
        return q->buffer[q->n_dequeued % q->buflen];
        q->n_dequeued++;
    } else {
        BREAKPOINT();
        return 0;
    }
};

/// In the OpenRover protocol, this byte signifies the start of a message
const uint8_t UART_START_BYTE = 253;

/// Baud rate for the serial device
#define UART_BAUD_RATE 57600

/// Checksum of a packet. An OpenRover packet should apply this to the payload
/// (contents of the message, excluding the start byte). It is calculated as
/// 255 - (sum of bytes % 255).
// TODO: this is a lousy checksum. Use a better checksum.
uint8_t checksum(size_t count, const uint8_t *data) {
    int i;
    uint16_t total = 0;
    for (i = 0; i < count; i++) {
        total += data[i];
    }
    return 255 - (total % 255);
}

/// Size, in bytes, of software buffer for receiving UART data
#define UART_RX_BUFLEN 256
/// Underlying buffer for inbound UART data
uint8_t uart_rx_buffer[UART_RX_BUFLEN];
/// Queue for inbound UART data
static Queue uart_rx_queue = {.buflen = UART_RX_BUFLEN, .buffer = uart_rx_buffer};

/// Size, in bytes, of software buffer for sending UART data
#define UART_TX_BUFLEN 256
/// Underlying buffer for outbound UART data
uint8_t uart_tx_buffer[UART_TX_BUFLEN];
/// Queue for outbound UART data
static Queue uart_tx_queue = {.buflen = UART_TX_BUFLEN, .buffer = uart_tx_buffer};

void uart_enqueue_debug_string(char *value) {
    int i;
    int n = n_can_enqueue(&uart_tx_queue);
    if (n < 2)
        return;
    enqueue(&uart_tx_queue, UART_START_BYTE);
    for (i = 0; i < n - 2 && value[i] != 0; i++) {
        enqueue(&uart_tx_queue, value[i]);
    }
    enqueue(&uart_tx_queue, 0);
}

void uart_init() {
    // Assign U1RX To U1RX, Uart receive channnel
    _U1RXR = U1RX_RPn;
    // Assign U1TX To U1TX/Uart Tx
    U1TX_RPn = 3; // 3 represents U1TX

    // Enable the UART.
    U1MODE = 0x0000;
    if (UART_BAUD_RATE < FCY / 4.0f) {
        U1MODEbits.BRGH = 0; // High Baud Rate Select bit = off
        U1BRG = FCY / 16 / UART_BAUD_RATE - 1;
    } else {
        U1MODEbits.BRGH = 1;
        U1BRG = FCY / 4 / UART_BAUD_RATE - 1;
    }
    U1MODEbits.UARTEN = 1; // UART1 is enabled

    U1STAbits.UTXISEL1 = 1; // Call transmit interrupt when the transmit buffer is empty
    U1STAbits.UTXISEL0 = 0; // Call transmit interrupt when the transmit buffer is empty
    U1STAbits.UTXEN = 1;    // enable transmit
    IEC0bits.U1TXIE = 1;    // enable UART1 transmit interrupt
    IEC0bits.U1RXIE = 1;    // enable UART1 receive interrupt
}

/// UART receive Interrupt function
/// Transfer inbound data from the UART hardware buffer into a software buffer
/// Called by hardware interrupt and enabled by _U1RXIE
void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt() {
    // clear the interrupt flag
    _U1RXIF = 0;
    // While new received data is available and we have somewhere to put it
    while (U1STAbits.URXDA) {
        uint8_t a_byte = (uint8_t)U1RXREG;
        // Pop the next byte off the read queue
        if (n_can_enqueue(&uart_rx_queue) > 0) {
            enqueue(&uart_rx_queue, a_byte);
        } else {
            // Nowhere to put the new data. Oh well.
        }
    }
    // if receiver overflowed this resets the module
    if (U1STAbits.OERR) {
        // clear overflow error bit if it was set.
        U1STAbits.OERR = 0;
    }
}

/// UART transmit Interrupt function
/// Transfer outbound data from a software buffer into the UART hardware buffer
/// called by hardware interrupt and enabled by _U1TXIE
void __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt() {
    // clear the interrupt flag
    _U1TXIF = 0;
    // transmit data
    while (U1STAbits.UTXBF == 0 &&
           n_can_dequeue(&uart_tx_queue) > 0) { // while transmit shift buffer is not full
        U1TXREG = dequeue(&uart_tx_queue);
    }
}

void uart_serialize_out_data(uint8_t *out_bytes, uint8_t uart_data_identifier) {
    uint16_t intval;

#define CASE(n, REGISTER)                                                                          \
    case (n):                                                                                      \
        intval = (uint16_t)REGISTER;                                                               \
        out_bytes[0] = (intval >> 8);                                                              \
        out_bytes[1] = (intval);                                                                   \
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
    case 18:
        out_bytes[0] = REG_MOTOR_FAULT_FLAG.left;
        out_bytes[1] = REG_MOTOR_FAULT_FLAG.right;
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
        CASE(40, RELEASE_VERSION_FLAT)
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
        break;
    }
#undef CASE
}

UArtTickResult uart_tick() {
    UArtTickResult result = {0};
    bool has_drive_command = false;
    bool has_fan_command = false;
    
    while (n_can_dequeue(&uart_rx_queue) >= RX_PACKET_SIZE) {
        size_t i;
        uint8_t packet[RX_PACKET_SIZE];
        uint8_t out_packet[TX_PACKET_SIZE];

        packet[0] = dequeue(&uart_rx_queue);
        if (packet[0] != UART_START_BYTE) {
            continue;
        }
        for (i = 1; i < RX_PACKET_SIZE; i++) {
            packet[i] = dequeue(&uart_rx_queue);
        }
        if (checksum(RX_PACKET_SIZE - 2, &(packet[1])) != packet[RX_PACKET_SIZE - 1]) {
            continue;
        }
        MotorEfforts efforts = {.left=packet[1]*8-1000,
	        .right = packet[2]*8-1000,
	        .flipper = packet[3]*8-1000, };
        drive_set_efforts(efforts);
        has_drive_command = true;
        UARTCommand verb = packet[4];
        uint8_t arg = packet[5];

        switch (verb) {
        case UART_COMMAND_SET_FAN_SPEED:
            REG_MOTOR_SIDE_FAN_SPEED = arg;
            result.uart_fan_speed_requested = true;
            has_fan_command = true;
            break;
        case UART_COMMAND_RESTART:
            asm volatile("RESET");
            break;
        case UART_COMMAND_FLIPPER_CALIBRATE:
            if (arg == UART_COMMAND_FLIPPER_CALIBRATE) {
	             // note flipper calibration never returns.
	             flipper_feedback_calibrate();
            }
            break;
        case UART_COMMAND_GET:
            out_packet[0] = UART_START_BYTE;
            out_packet[1] = arg;
            uart_serialize_out_data(&out_packet[2], arg);
            out_packet[4] = checksum(3, &out_packet[1]);

            if (n_can_enqueue(&uart_tx_queue) >= TX_PACKET_SIZE) {
                for (i = 0; i < TX_PACKET_SIZE; i++) {
                    enqueue(&uart_tx_queue, out_packet[i]);
                }
                // Start transmission
                _U1TXIF = 1;
            }
            break;
        case UART_COMMAND_SETTINGS_RELOAD:
            g_settings = settings_load();
            break;
        case UART_COMMAND_SETTINGS_COMMIT:
            settings_save(&g_settings);
            break;
        case UART_COMMAND_SET_DRIVE_MODE:
            break;
            // fallthrough
        default:
            // unknown inbound command.
            BREAKPOINT();
            break;
        }
        result.uart_motor_speed_requested = true;
    }
    
    if (!has_fan_command && ticks_since_last_fan_command != UINT16_MAX){
    	if (++ticks_since_last_fan_command * g_settings.main.communication_poll_ms > g_settings.communication.fan_command_timeout_ms)
   		{
	   		REG_MOTOR_SIDE_FAN_SPEED = 0;
			ticks_since_last_fan_command = UINT16_MAX;
    	}	
    }
        
    if (!has_drive_command && ticks_since_last_drive_command != UINT16_MAX){
	    if (++ticks_since_last_drive_command * g_settings.main.communication_poll_ms > g_settings.communication.drive_command_timeout_ms)
   		{    	
	   		// long time no motor commands. stop moving.
	   		REG_MOTOR_VELOCITY.left = 0;
        	REG_MOTOR_VELOCITY.right = 0;
        	REG_MOTOR_VELOCITY.flipper = 0;
    		ticks_since_last_drive_command = UINT16_MAX;
    	}
    }

    _U1TXIF = 1;
    return result;
}
