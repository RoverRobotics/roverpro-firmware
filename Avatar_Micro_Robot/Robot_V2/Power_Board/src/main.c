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
#include "PwrMgnt.h"
#include "device_robot_motor.h"

// -------------------------------------------------------------------------
// PIC24FJ256GB106 FLASH CONFIGURATION
// -------------------------------------------------------------------------

_CONFIG1(JTAGEN_OFF &
#if __DEBUG
             GCP_OFF
#else
             GCP_ON
#endif
                 &GWRP_OFF &COE_OFF &FWDTEN_ON &ICS_PGx2 &WDTPS_PS128)

_CONFIG2(IESO_OFF &FCKSM_CSDCMD &OSCIOFNC_ON &POSCMOD_HS &FNOSC_PRIPLL &PLLDIV_DIV5 &IOL1WAY_ON)

// WDT timeout = 128/31 kHz * WDTPS
// 128/31e3*128 = .53 seconds

// -------------------------------------------------------------------------
// BOOTLOADER
// -------------------------------------------------------------------------

#define PF __attribute__((section("programmable"))) // programmable function
#define FIRST_PROGRAMMABLE_FUNC __attribute__((address(0xF00)))

typedef enum COMMAND_T {
    DEVICE_OCU_INIT,
    DEVICE_CARRIER_INIT,

    DEVICE_OCU_PROCESS_IO,
    DEVICE_CARRIER_PROCESS_IO
    // etc.....
} COMMAND;

void PF FIRST_PROGRAMMABLE_FUNC callFunc(COMMAND command, void *params) { return; }

// -------------------------------------------------------------------------
// GLOBAL VARIABLES
// -------------------------------------------------------------------------

#pragma udata

int gNewData;
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

#pragma code

int PF main(void) {
    InitializeSystem();

    while (1) {
        ProcessIO();
    }
}

static void InitializeSystem(void) {
    gpio_id = PORTE & 0x001F; ///< 5-bit board ID (RE0 to RE4)

    /*
    // override clock settings
    CLKDIVbits.RCDIV = 0;
    CLKDIVbits.CPDIV = 0;
    CLKDIVbits.DOZEN = 0;
    CLKDIVbits.DOZE = 0;*/

    // get number of registers
    while (registers[gRegisterCount].ptr != 0) {
        gRegisterCount++;
    }

    // ---------------------------------------------------------------------
    // DEVICE SPECIFIC INITIALIZATION HERE
    // ---------------------------------------------------------------------

    device_dsc.idProduct = DEVICE_MOTOR;

    // we got rid of the ID pins, so force robot motor to init
    DeviceRobotMotorInit();

    /*	switch (gpio_id)
            {
                    case DEVICE_OCU:
                            DeviceOcuInit();
                            break;

                    case DEVICE_CARRIER:
                            DeviceCarrierInit();
                            break;

                    case DEVICE_MOTOR:
                            DeviceRobotMotorInit();
                            break;

                    case DEVICE_ARM_BASE:
                            DeviceArmBaseInit();
                            break;

                    case DEVICE_ARM_SHOLDER:
                            DeviceArmSholderInit();
                            break;

                    case DEVICE_ARM_HAND:
                            DeviceArmHandInit();
                            break;

                    case DEVICE_GENERIC:
                    default:
                            DeviceGenericInit();
                            break;
            }*/

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

    ClrWdt();

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
        gNewData = !gNewData; // toggle new data flag for those watching
        i = 0;                // reset IN packet pointer

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

                send_debug_uart_string("Initial motor velocities out of bounds!\r\n", 41);
                ClrWdt();
                block_ms(100);
                ClrWdt();
                send_debug_uart_string("Stopping motors forever.\r\n", 26);

                while (1) {
                    ClrWdt();
                }
            }
        }

        // motor velocities coming from software can actually be higher than 1000 (highest I saw was
        // 1200), so this is commented out

        // if any motor velocities are out of bounds, set all motor velocities to zero
        /*if( (abs(REG_MOTOR_VELOCITY.left ) > 1000) || (abs(REG_MOTOR_VELOCITY.right ) > 1000) ||
        (abs(REG_MOTOR_VELOCITY.flipper ) > 1000) )
        {
          REG_MOTOR_VELOCITY.left = 0;
          REG_MOTOR_VELOCITY.right = 0;
          REG_MOTOR_VELOCITY.flipper = 0;
        }*/
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
