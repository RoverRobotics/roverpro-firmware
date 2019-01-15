/**
 * @file main.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex one-size-fits-all firmware. Designed for the PIC24FJ256GB106 only.
 * Device running at 32MHz:
 *     Input Oscillator 20MHz
 *     PLLDIV = 5 (4MHz)
 *     PLL generates 48MHz for USB and 32MHZ for system clock
 *     CPDIV = 0 (CPU clock 32MHz)
 *     instruction clock 16MHz
 *
 */

#include "stdhdr.h"
#include "device_robot_motor.h"
#include "motor.h"

// -------------------------------------------------------------------------
// PIC24FJ256GB106 FLASH CONFIGURATION
// -------------------------------------------------------------------------

// Set configuration bits.
// Note we can't overwrite these with the bootloader, so this is only needed
// for writing firmware with the PICkit3
#include "../../bootypic/devices/pic24fj256gb106/config.h"

// -------------------------------------------------------------------------
// BOOTLOADER
// -------------------------------------------------------------------------

typedef enum COMMAND_T {
    DEVICE_OCU_INIT,
    DEVICE_CARRIER_INIT,

    DEVICE_OCU_PROCESS_IO,
    DEVICE_CARRIER_PROCESS_IO
    // etc.....
} COMMAND;

// -------------------------------------------------------------------------
// GLOBAL VARIABLES
// -------------------------------------------------------------------------

bool usb_new_data_received;
int gpio_id = 0;
int gRegisterCount = 0;
// uint8_t OutPacket[OUT_PACKET_LENGTH];
// uint8_t InPacket[IN_PACKET_LENGTH];
uint8_t OutPacket[0xff];
uint8_t InPacket[0xff];
USB_HANDLE USBGenericOutHandle = 0;
USB_HANDLE USBGenericInHandle = 0;

// -------------------------------------------------------------------------
// PROTOTYPES
// -------------------------------------------------------------------------

static void InitializeSystem(void);
void USBDeviceTasks(void);
void USBSendPacket(uint8_t command);
void ProcessIO(void);
extern USB_DEVICE_DESCRIPTOR device_dsc;

// -------------------------------------------------------------------------
// CODE
// -------------------------------------------------------------------------

int main(void) {
    InitializeSystem();

    while (1) {
        ProcessIO();
    }
}

static void InitializeSystem(void) {
    gpio_id = PORTE & 0x001F; ///< 5-bit board ID (RE0 to RE4)

    // get number of registers
    while (registers[gRegisterCount].ptr != 0) {
        gRegisterCount++;
    }

    // ---------------------------------------------------------------------
    // DEVICE SPECIFIC INITIALIZATION HERE
    // ---------------------------------------------------------------------

    device_dsc.idProduct = DEVICE_MOTOR;

    DeviceRobotMotorInit();

    // ---------------------------------------------------------------------
    // GENERIC INITIALIZATION HERE
    // ---------------------------------------------------------------------

    USBDeviceInit();
    USBDeviceAttach();
}

void ProcessIO(void) {
    uint16_t n = 0;
    uint16_t cur_word, reg_index;
    uint16_t reg_size;
    uint16_t i = 0;
    static unsigned int message_counter = 0;

    __builtin_clrwdt();

    // ---------------------------------------------------------------------
    // DEVICE SPECIFIC I/O PROCESS HERE
    // ---------------------------------------------------------------------

    // we got rid of id pins, so force motor controller to run
    Device_MotorController_Process();

    // ---------------------------------------------------------------------
    // GENERIC I/O PROCESS HERE
    // ---------------------------------------------------------------------

    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1))
        return;

    if (!USBHandleBusy(USBGenericOutHandle)) {
        usb_new_data_received = true;
        i = 0; // reset IN packet pointer

        // PARSE INCOMING PACKET ----------------------------------------------
        while (1) {
            if ((n + 2) > OUT_PACKET_LENGTH)
                break; // overflow

            cur_word = OutPacket[n] + (OutPacket[n + 1] << 8); // get register value
            n += 2;                                            // move OUT packet pointer

            if ((cur_word == PACKET_TERMINATOR) || n > OUT_PACKET_LENGTH)
                break; // end of list

            reg_index = cur_word & ~DEVICE_READ;

            if (reg_index >= gRegisterCount)
                break; // bad packet

            reg_size = registers[reg_index].size;

            if ((cur_word & DEVICE_READ) == DEVICE_READ) {
                if ((i + 2) > IN_PACKET_LENGTH)
                    break; // overflow
                InPacket[i + 1] = reg_index >> 8;
                InPacket[i] = reg_index & 0xff;
                i = i + 2; // move IN packet pointer
                if ((i + reg_size) > IN_PACKET_LENGTH)
                    break; // overflow
                memcpy(InPacket + i, registers[reg_index].ptr, reg_size);
                i = i + reg_size; // move IN packet pointer
            } else {
                if ((n + reg_size) > OUT_PACKET_LENGTH)
                    break; // overflow
                memcpy(registers[reg_index].ptr, OutPacket + n, reg_size);
                n += reg_size; // move OUT packet pointer
            }
        }

        if ((i + 2) > IN_PACKET_LENGTH)
            goto crapout5;

        InPacket[i + 1] = PACKET_TERMINATOR >> 8;
        InPacket[i] = PACKET_TERMINATOR & 0xff;
        i += 2;

        if (!USBHandleBusy(USBGenericInHandle) && (i > 0)) {
            USBGenericInHandle = USBTxOnePacket((BYTE)USBGEN_EP_NUM, (BYTE *)&InPacket, (WORD)i);
        }

    crapout5:
        // Arm USB hardware to receive next packet.
        USBGenericOutHandle =
            USBRxOnePacket((BYTE)USBGEN_EP_NUM, (BYTE *)&OutPacket, (WORD)(OUT_PACKET_LENGTH));

        // check first few messages for invalid motor velocities
        if (message_counter < 3) {
            message_counter++;

            // if any of the first few motor velocity values are out of bounds, stop responding and
            // print message
            if ((abs(REG_MOTOR_VELOCITY.left) > 1000) || (abs(REG_MOTOR_VELOCITY.right) > 1000) ||
                (abs(REG_MOTOR_VELOCITY.flipper) > 1000)) {

                // Stop motors
                // coast left motor
                Coasting(MOTOR_LEFT);
                Coasting(MOTOR_RIGHT);
                Coasting(MOTOR_FLIPPER);

                BREAKPOINT(); // Initial motor velocities out of bounds!
                while (1) {
                    // Stop motors forever
                    __builtin_clrwdt();
                }
            }
        }
    }
}

void USBCBInitEP(void) {
    USBEnableEndpoint(USBGEN_EP_NUM, USB_OUT_ENABLED | USB_IN_ENABLED | USB_DISALLOW_SETUP);
    USBGenericOutHandle =
        USBRxOnePacket((BYTE)USBGEN_EP_NUM, (BYTE *)&OutPacket, (WORD)(OUT_PACKET_LENGTH));
}

BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size) {
    switch (event) {
    case EVENT_CONFIGURED:
        USBCBInitEP();
        break;
    case EVENT_SET_DESCRIPTOR:
        break;
    case EVENT_EP0_REQUEST:
        break;
    case EVENT_SOF:
        break;
    case EVENT_SUSPEND:
        break;
    case EVENT_RESUME:
        break;
    case EVENT_BUS_ERROR:
        break;
    case EVENT_TRANSFER:
        break;
    default:
        break;
    }
    return TRUE;
}
