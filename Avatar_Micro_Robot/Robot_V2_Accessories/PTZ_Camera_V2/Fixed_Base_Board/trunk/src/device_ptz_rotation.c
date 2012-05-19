#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "interrupt_switch.h"
#include <stdio.h>
#include <math.h>

#define TILT_USB_WATCHDOG_ON

#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
	#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

// -------------------------------------------------------------------------
// Type declarations
// -------------------------------------------------------------------------

typedef enum {IN, OUT} ZOOM_DIRECTION;

typedef enum {ZOOM_STATE_STOP, ZOOM_STATE_IN, ZOOM_STATE_OUT} ZOOM_STATE;

typedef enum {MESSAGE_ACKNOWLEDGED, MESSAGE_SENT} UART_MESSAGE_STATE;

typedef enum {ZOOM_MESSAGE} LAST_MESSAGE_TYPE;

typedef struct
{
	int16_t pan, tilt, zoom;
} REG_CAMERA_VEL_TYPE;

typedef struct
{
	int16_t pan, tilt, zoom, digitalZoom;
} REG_CAMERA_POS_TYPE;

// -------------------------------------------------------------------------
// UART Buffer / Message Constants
// -------------------------------------------------------------------------

#define UART_RX_BUFFER_SIZE 8
#define UART2_RX_BUFFER_SIZE 7
#define UART2_TX_BUFFER_SIZE 7
#define UART_TX_BUFFER_SIZE 20

#define RX_START_CHAR 0x77
#define CAMERA_MESSAGE_START_BYTE 0xA3
#define CAMERA_MESSAGE_END_BYTE 0xAF
#define CAMERA_MESSAGE_QUERY_ZOOM 0x82
#define CAMERA_MESSAGE_QUERY_FOCUS 0x83

#define USB_TIMEOUT 100

//2500 is about 20 seconds
#define ZOOM_MODULE_INIT_MS 2500

//If the error is below this value, closed loop control is disabled
#define TILT_POSITION_DEADBAND 2


static volatile unsigned char uartRxData[UART_RX_BUFFER_SIZE];          
static volatile int uartRxIndex = 0;
static volatile unsigned char uart2RxData[UART2_RX_BUFFER_SIZE];          
static volatile int uart2RxIndex = 0;
static volatile unsigned char uart2TxData[UART2_TX_BUFFER_SIZE];
static volatile int uart2TxIndex = 0;
static volatile unsigned char uartTxData[UART_TX_BUFFER_SIZE];
static volatile unsigned int uartTxDataLength = 0;
static volatile int uartTxIndex = 0;

// -------------------------------------------------------------------------
// Hardware Mappings
// -------------------------------------------------------------------------
#define U2RX_RPn 27
#define U1RX_RPn 9

#define U2TX_RPn  	RPOR9bits.RP19R
#define U1TX_RPn  	RPOR4bits.RP8R
#define PWM1_RPn	RPOR14bits.RP29R

#define BaudRate_9600_LOW 103  //BRGH=0
#define BaudRate_38400_LOW 25
#define BaudRate_57600_LOW 16  //BRGH=0
#define BaudRate_57600_HI 68 //BRGH=1
#define BaudRate_115200_HI 34  //BRGH=1

//*-----------------------------------PWM------------------------------------*/
/////constant
//****define frequency value for PRy, based on 1:1 prescale
#define Period50Hz 533332
#define Period67Hz 319999 //15ms
#define Period200Hz 79999
#define Period300Hz 53332
#define Period400Hz 39999
#define Period500Hz 31999
#define Period600Hz 26666
#define Period700Hz 22856
#define Period800Hz 19999
#define Period900Hz 17777
#define Period1000Hz 15999
#define Period1100Hz 14544
#define Period1200Hz 13332
#define Period1300Hz 12307
#define Period1400Hz 11428
#define Period1500Hz 10666
#define Period1600Hz 9999
#define Period1700Hz 9411
#define Period1800Hz 8888
#define Period1900Hz 8420
#define Period2000Hz 7999
#define Period2100Hz 7618
#define Period10000Hz 1599
#define Period20000Hz 799
#define Period30000Hz 532
#define Period50000Hz 319

// -------------------------------------------------------------------------
// Motor Limits
// -------------------------------------------------------------------------
#define TILT_STEPPER_LOWER 490
#define TILT_STEPPER_UPPER 780
#define TILT_ANGLE_LOWER 5.0f
#define TILT_ANGLE_UPPER 87.0f


// -------------------------------------------------------------------------
// Ticks variable / Time constants
// -------------------------------------------------------------------------

volatile static unsigned long ticks = 0;

#define ZOOM_STOP_TIMEOUT 20
#define QUERY_ZOOM_INTERVAL 30
#define RESET_TIME 700
#define COMMAND_TIMEOUT 300

static uint8_t queryZoomTrigger=0;
static uint8_t queryFocusTrigger=0;
static unsigned long queryZoomTicks;
static unsigned long queryFocusTicks;
static unsigned long commandSendTime;

// -------------------------------------------------------------------------
// Local State Variables
// -------------------------------------------------------------------------

static int messageSent = 0;
static uint8_t zoomQueried = 0;
static uint8_t focusQueried = 0;
static int lastZoomSpeed = 0;
static uint16_t analogZoomLevel = 0;
static uint16_t digitalZoomLevel = 0;
static uint16_t focusLevel = 0;
static LAST_MESSAGE_TYPE cameraLastMessageType;
static UART_MESSAGE_STATE cameraMessageState;
static ZOOM_STATE cameraZoomState;
static uint8_t lensRecover = 0;
static uint8_t lensResetEdge = 0;
static float potentiometer = 0;
static int lastTiltVelocity = 0;
static uint32_t tiltHoldPosition = TILT_STEPPER_LOWER;
static int tiltPidSum = 0;
static int tiltPidLastProp = 0;
static uint8_t autoFocus = 1;
static uint16_t shutterSpeed = 0x0111;

// -------------------------------------------------------------------------
// *********** Registers (move to register.h)
// *****************************************************
// -------------------------------------------------------------------------

uint8_t REG_CAMERA_RESET;
uint8_t REG_CAMERA_REBOOT;
uint8_t REG_CAMERA_LENS_RESET;
uint8_t REG_CAMERA_NIGHT_MODE_ENABLE;
uint8_t REG_CAMERA_DIGITAL_ZOOM_ENABLE;

// receive data from the camera (ACK packets)
void  __attribute__((__interrupt__, auto_psv)) _U2RXInterrupt(void)
{
 	IFS1bits.U2RXIF=0;
	if(U2STAbits.OERR)
		U2STAbits.OERR = 0;
	while(U2STAbits.URXDA)
	{
		unsigned char data = U2RXREG;
		if(uart2RxIndex == 0)
		{
			if (data == CAMERA_MESSAGE_START_BYTE)
			{
				uart2RxData[uart2RxIndex] = data;
				uart2RxIndex = (uart2RxIndex+1)%UART2_RX_BUFFER_SIZE;
			}	
		}
		else
		{
			uart2RxData[uart2RxIndex] = data;
			uart2RxIndex = (uart2RxIndex+1)%UART2_RX_BUFFER_SIZE;
		}
	}
}

// used to transmit telemetry back to the robot/computer
static void TXInterrupt()
{
   	IFS0bits.U1TXIF=0;
	// keep transmitting until we're done with the tx buffer
	if(uartTxIndex < uartTxDataLength)
	{
		U1TXREG = uartTxData[uartTxIndex];
		uartTxIndex++;
	}
	else
	{
		uartTxIndex = 0;
		// disable interrupt until we want to transmit again
		IEC0bits.U1TXIE=0;
	}	
}

// used to transmit to the camera
void  __attribute__((__interrupt__, auto_psv)) _U2TXInterrupt(void)
{
   	IFS1bits.U2TXIF=0;
	if(uart2TxIndex < UART2_TX_BUFFER_SIZE)
	{
		//U2STAbits.UTXEN=1;
		U2TXREG = uart2TxData[uart2TxIndex];
		uart2TxIndex++;
	}
	else
	{
		uart2TxIndex = 0;
		IEC1bits.U2TXIE=0;
	}	
}

// receive a packet from the computer / robot
static void RXInterrupt()
{
	
	IFS0bits.U1RXIF=0;
	if(U1STAbits.OERR)
		U1STAbits.OERR = 0;
	while(U1STAbits.URXDA)
	{
		unsigned char data = U1RXREG;
		if(data == RX_START_CHAR)
			uartRxIndex = 0;
		uartRxData[uartRxIndex] = data;
		uartRxIndex = (uartRxIndex+1)%UART_RX_BUFFER_SIZE;
	}
}

// count up with the ticks variable when the timer 1 interrupt fires every 1ms
static void T1Interrupt()
{
	ticks++;
  USB_timeout_ms++;
	IFS0bits.T1IF=0;
}

static void ADC1Interrupt()
{
	if(AD1CON1bits.DONE)
		potentiometer = potentiometer*0.8f + ((float)ADC1BUF0)*0.2f;
	IFS0bits.AD1IF = 0; 
}

static void UART1Ini()
{
 	// Write appropriate baud rate value to the UxBRG register.
 	U1BRG=BaudRate_38400_LOW;
 	//Enable the UART.
 	U1MODE=0x0000;
 	//high speed mode
 	U1MODEbits.BRGH=0;
	// even parity
	U1MODEbits.PDSEL1=0;
	U1MODEbits.PDSEL0=1;
	U1MODEbits.LPBACK=0;
 	U1STA=0x0000; 	
 	U1MODEbits.UARTEN=1;//UART1 is enabled
 	U1STAbits.UTXEN=1;//transmit enabled
 	IFS0bits.U1TXIF=0;//clear the transmit flag
	IFS0bits.U1RXIF=0;
 	IEC0bits.U1TXIE=0;//disable UART1 transmit interrupt
 	IEC0bits.U1RXIE=1;//enable UART1 receive interrupt
}

static void UART2Ini()
{
 	// Write appropriate baud rate value to the UxBRG register.
 	U2BRG=BaudRate_38400_LOW;
 	//Enable the UART.
 	U2MODE=0x0000;
 	//hight speed mode
 	U2MODEbits.BRGH=0;
	// even parity
	U2MODEbits.PDSEL1=0;
	U2MODEbits.PDSEL0=1;
	U2MODEbits.LPBACK=0;
 	U2STA=0x0000; 	
 	U2MODEbits.UARTEN=1;//UART2 is enabled
	
 	U2STAbits.UTXEN=1;//transmit enabled
 	IFS1bits.U2TXIF=0;//clear the transmit flag
	IFS1bits.U2RXIF=0;
 	IEC1bits.U2TXIE=0;//disable UART2 transmit interrupt
 	IEC1bits.U2RXIE=1;//enable UART2 receive interrupt
}

static void PWM1Set(int duty, int period)
{
	int pr2Value = ((period<<4)-10);
	OC1R = pr2Value - duty*(pr2Value/100);
	OC1RS = ((period<<4)-2);
}


static void IniPWM1()
{
    T2CONbits.TCKPS = 0b01;    

    //_RP25R = 18;      //OC1

    //AP8803: dimming frequency must be below 500Hz
    //Choose 400Hz (2.5ms period)
    //2.5ms = [PR2 + 1]*62.5ns*1
   
	//PWM1SetPeriod(500000);
	PR2 = ((262000<<4)-1);

    OC1CON2bits.SYNCSEL = 0x1f;                   
    //use timer 2
    OC1CON1bits.OCTSEL2 = 0;
    
    //edge-aligned pwm mode
    OC1CON1bits.OCM = 6;
    
    //turn on timer 2
    T2CONbits.TON = 1;

	//Max out counter width 
	
	PWM1Set(0, 260000);
}

static void Stepper1SetDir(int dir)
{
	if(dir)
		PORTBbits.RB13=1;
	else
		PORTBbits.RB13=0;
}

static void IniTimer1()
{
	T1CON=0x0000;//clear register
	T1CONbits.TCKPS=0b01;//timer stops,1:8 prescale,
	TMR1=0;//clear timer1 register
	PR1=Period1000Hz;//interrupt every 1ms
	T1CONbits.TON=SET;
	IFS0bits.T1IF=0;//clear timer interrupt flag
 	IEC0bits.T1IE=1;//enable timer1 interrupt
 
}

static void ADC1Ini()
{
	AD1CON1 = 0x80E4;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x1F05;
	AD1CHS = 0;
	AD1PCFGbits.PCFG0 = 0;
	AD1CSSL = 0;
	IEC0bits.AD1IE=1;
	
}

static void PinRemap(void)
{
	// Unlock Registers
	//clear the bit 6 of OSCCONL to
	//unlock Pin Re-map
	asm volatile	("push	w1				\n"
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
					"pop	w1");

	
	// UART for debugging
 	RPINR18bits.U1RXR = U1RX_RPn;
 
	U1TX_RPn = 3; //3 represents U1TX

	// UART for camera
	RPINR19bits.U2RXR = U2RX_RPn;
 
	U2TX_RPn = 5; //0 represents U2TX

	PWM1_RPn = 18;
	
	//***************************
	// Lock Registers
	__builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to
	//lock Pin Re-map

}

static void PTZIni() 
{
	//peripheral pin selection
	PinRemap();
	//peripheral pin selection end
	//*******************************************
	//We're not using any analog inputs
	AD1PCFGL = 0xFFFF;
	AD1PCFGH = 0xFFFF;

	//PORTB
	TRISB&=0b0011111111110110;	
	TRISB|=0b0011111100110010;
	
	//PORTC
	TRISC&=0b1001111111111111;	

	//PORTD
	TRISD&=0b1111010001111100;	
	TRISD|=0b0000010001111100;

	//PORTE
	TRISE&=0b1111111111011111;	
	//TRISE|=0b0000000000100000;

	//PORTF
	TRISF&=0b1111111111110111;	
	//TRISF|=0b0000000000000000;	

	//PORTG
	TRISG&=0b1111110000111111;	

    TRISFbits.TRISF5=0;//RF5 output
 	TRISDbits.TRISD9=0;//RD9 output
 	TRISDbits.TRISD10=1;//RD10 input	
 	
	//uart2
	TRISBbits.TRISB8=0;
	TRISBbits.TRISB9=1;
	TRISGbits.TRISG9=1;
	TRISGbits.TRISG8=0;

	//vbat
	TRISBbits.TRISB3=0;
	PORTBbits.RB3=1;

	//PWM1
	TRISBbits.TRISB15=0;

	// STEPPER MOTOR ~ENABLE
	TRISBbits.TRISB11=0;
	PORTBbits.RB11=0;
	// STEPPER MOTOR DIR
	TRISBbits.TRISB13=0;
	PORTBbits.RB13=0;
	// STEPPER MOTOR MS1
	TRISBbits.TRISB14=0;
	PORTBbits.RB14=1;
	// STEPPER MOTOR MS2
	TRISBbits.TRISB12=0;
	PORTBbits.RB12=1;
	// ADC
	TRISBbits.TRISB0=1;

	// set up interrupt handlers for UART1 and Timer1 (UART2 is done separately for now)
	U1RXInterruptUserFunction = RXInterrupt;
	U1TXInterruptUserFunction = TXInterrupt;
	T1InterruptUserFunction = T1Interrupt;
	ADC1InterruptUserFunction = ADC1Interrupt;

 	UART1Ini();
	UART2Ini();
	ADC1Ini();
	IniTimer1();

	IniPWM1();

	//initialize and turn on rotation fan
	_TRISG7 = 0;
	_LATG7 = 1;
}


// Blocking UART transmit - for debugging only
static void UART1Transmit( int data)
{
 	while(U1STAbits.UTXBF==1);
 	IFS0bits.U1TXIF=0; 	
 	U1TXREG=data;
}

// Blocking UART2 transmit - for debugging only
static void UART2Transmit( int data)
{
 	while(U2STAbits.UTXBF==1);
 	IFS1bits.U2TXIF=0; 	
 	U2TXREG=data;
}

// Send a register to the computer
static void UARTSendRegister(char *message, int len)
{
	IEC0bits.U1TXIE=0;
	memcpy(uartTxData, message, len);
	uartTxDataLength = len;
    IFS0bits.U1TXIF = 1;
	IEC0bits.U1TXIE = 1;
}

// checksum required by the camera
static unsigned char CameraChecksum(char *data, int len)
{
	int i, sum;
	sum = 0;
	for(i=0;i<len;i++)
	{
		sum+=data[i];
	}
	return (~sum) & 0xFF;
}

// Send a command to the camera
// uses interrupts, non-blocking
static void CameraSendCommand(char *message)
{
	IEC1bits.U2TXIE=0;
	memcpy(uart2TxData, message, 7);
	uart2TxData[5] = CameraChecksum(message+1, 4);
	uart2TxIndex = 0;
    IFS1bits.U2TXIF = 1;
	IEC1bits.U2TXIE=1;

	// update message information - for state/timeouts
	cameraMessageState = MESSAGE_SENT;
	commandSendTime = ticks;
}

// Self-explanatory camera commands
// Non-blocking, so be sure to wait for an ack before sending another command!
static void CameraQueryZoom()
{
    char message[7] = {0xA3, 0x82, 0x00, 0x00, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraQueryFocus()
{
    char message[7] = {0xA3, 0x83, 0x00, 0x00, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraDayMode()
{
    char message[7] = {0xA3, 0x27, 0x01, 0x32, 0x80, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraNightMode()
{
	char message[7] = {0xA3, 0x27, 0x41, 0x32, 0x80, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraInvert()
{
	char message[7] = {0xA3, 0x44, 0x01, 0x01, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraAFOn()
{
	char message[7] = {0xA3, 0x1C, 0x01, 0x40, 0x8E, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraAFOff()
{
	char message[7] = {0xA3, 0x1C, 0x00, 0x40, 0x8E, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraSetFocus(uint16_t focus)
{
    char message[7] = {0xA3, 0x1B, (uint8_t)(focus>>8), (uint8_t)focus, 0x8E, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraLensInit()
{
	char message[7] = {0xA3, 0x31, 0x00, 0x00, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraZoom(ZOOM_DIRECTION direction, uint8_t velocity)
{
	char message[7] = {0xA3, 0x11, direction, velocity, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraZoomStop()
{
	char message[7] = {0xA3, 0x10, 0x00, 0x00, 0x00, 0x00, 0xAF}; 
	CameraSendCommand(message);
}

static void CameraFocus(ZOOM_DIRECTION direction)
{
	char message[7] = {0xA3, 0x1A, direction, 0x06, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraFocusStop()
{
	char message[7] = {0xA3, 0x19, 0x00, 0x00, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraReset()
{
	char message[7] = {0xA3, 0x02, 0x00, 0x00, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraSave()
{
	char message[7] = {0xA3, 0x04, 0x00, 0x00, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

static void CameraShutterSetup(uint16_t shutter)
{
	char message[7] = {0xA3, 0x24, (uint8_t)(shutter>>8), (uint8_t)shutter, 0x00, 0x00, 0xAF};
	CameraSendCommand(message);
}

// Default configuration specified here
// Code can be blocking as it is only run once at the beginning
static void DefaultConfiguration()
{
	// turning on/configuring autofocus includes turning on digital zoom
	CameraAFOn();
	
	// wait until we've sent the whole message
	while(U2STAbits.TRMT==0);

	// Invert Image
    CameraInvert();
    while(U2STAbits.TRMT==0);
	CameraShutterSetup(shutterSpeed);
	while(U2STAbits.TRMT==0);
	CameraSave();
	while(U2STAbits.TRMT==0);
}

void DevicePTZRotationInit()
{

	PTZIni();
  //If we don't wait for the camera to power up fully, it won't properly
  //read and store the default configuration.  This is kind of a sloppy way to
  //do it, so we'll probably want to fix this later.
  //This issue was causing the camera image to be inverted when the PTZ was first
  //assembled.  Since the board was programmed without the zoom module, it never
  //was able to read the default config messages to save them.  Transmission likely
  //failed before as well, but the configuration was stored in the zoom module from
  //when the rotation board was programmed.
  //block_ms(10000);
	DefaultConfiguration();
}

static void Input()
{
/*	// we got a full rx packet (containing a register)
	if((uartRxData[0] == RX_START_CHAR) && (uartRxIndex == 0))
	{
		// copy packet into register
		memcpy(&REG_CAMERA_VEL, uartRxData+1, sizeof(REG_CAMERA_VEL));
		memcpy(&REG_CAMERA_LENS_RESET, uartRxData+1+sizeof(REG_CAMERA_VEL), sizeof(REG_CAMERA_LENS_RESET));
	}*/

	// make sure the packet is a legitimate camera ack packet
	// not checking checksum
	if((uart2RxData[0] == CAMERA_MESSAGE_START_BYTE) && (uart2RxData[6] == CAMERA_MESSAGE_END_BYTE) /*&& (CameraChecksum(uart2RxData+1, 4) == uart2RxData[5])*/)
	{
		// we only care if we're in the sent state
		if(cameraMessageState == MESSAGE_SENT)
		{
			// go to the acknowledged state
			uart2RxData[0] = 0;
			uart2RxData[6] = 0;
			cameraMessageState = MESSAGE_ACKNOWLEDGED;

			// if it's a data packet for focus or zoom, store the data in a register
			if(zoomQueried && (uart2RxData[1] == CAMERA_MESSAGE_QUERY_ZOOM))
			{
				zoomQueried = 0;
				REG_CAMERA_POS_ROT.zoom = (((uint16_t)uart2RxData[2])<<8) | uart2RxData[3];
				REG_CAMERA_POS_ROT.digitalZoom =  (uint16_t)(uart2RxData[4]);
			}
			else if(focusQueried && (uart2RxData[1] == CAMERA_MESSAGE_QUERY_FOCUS))
			{
				focusQueried = 0;
				REG_CAMERA_FOCUS = (((uint16_t)uart2RxData[2])<<8) | uart2RxData[3];
			}
		}
	} 

	// if we've acknowledged a packet, poll for zoom and focus on their respective intervals
	if(cameraMessageState == MESSAGE_ACKNOWLEDGED)
	{
		if(queryZoomTrigger)
		{
			CameraQueryZoom();
			zoomQueried = 1;
			messageSent = 1;
			queryZoomTrigger = 0;
		}
		else if(queryFocusTrigger)
		{
			CameraQueryFocus();
			focusQueried = 1;
			messageSent = 1;
			queryFocusTrigger = 0;
		}
	}
}

// Do the lens recovery action
static void Recovery()
{
	// only reset the lens on the rising edge of the lens reset signal
	// lensResetEdge is set to 1 after the reset register goes high
	// if it isn't one, that means that a rising edge event occured on the Lens Reset Register
	if(REG_CAMERA_LENS_RESET == 1 && !lensResetEdge) 
	{
		lensResetEdge = 1;
		lensRecover = 1;
	}
	else if(REG_CAMERA_LENS_RESET == 0)
	{
		lensResetEdge = 0;
	}
	if(lensRecover && (cameraMessageState == MESSAGE_ACKNOWLEDGED))
	{
		// Re-init the lens
		CameraLensInit();
		lensRecover = 0;
	}
}

// update the software timers based on the master tick count
static void TimerUpdate()
{
	// update focus and zoom timers on the same interval
	// if we're still waiting on a zoom trigger - keep resetting the timer.  This will cause zoom and focus events to become spaced out.
	if(((ticks-queryZoomTicks) > QUERY_ZOOM_INTERVAL) || queryZoomTrigger)
	{
		queryZoomTrigger = 1;
		queryZoomTicks = ticks;
	}
	else if(((ticks-queryFocusTicks) > QUERY_ZOOM_INTERVAL) || queryFocusTrigger)
	{
		queryFocusTrigger = 1;
		queryFocusTicks = ticks;
	}

	// handle timeout case - go from SENT to ACKNOWLEDGED after a timeout period
	if((cameraMessageState == MESSAGE_SENT) && ((ticks-commandSendTime)>COMMAND_TIMEOUT))
	{
		cameraMessageState = MESSAGE_ACKNOWLEDGED;
	}
}

static void InputSummation()
{
}

// Zoom State Machine
static void ZoomControl()
{
	// if we have an ack'ed message to act upon
	if(cameraMessageState == MESSAGE_ACKNOWLEDGED)
	{
		ZOOM_STATE newZoomState; 
		// send a new message if we change our zoom speed as well 
		uint8_t zoomSpeedChanged = (lastZoomSpeed != REG_CAMERA_VEL_ROT.zoom);
		lastZoomSpeed = REG_CAMERA_VEL_ROT.zoom; 
		// Look at newly sync'ed registers to zoom in or out
		if(REG_CAMERA_VEL_ROT.zoom > 0)
		{
			newZoomState = ZOOM_STATE_IN;
		}
		else if(REG_CAMERA_VEL_ROT.zoom < 0)
		{
			newZoomState = ZOOM_STATE_OUT;
		}
		else
		{
			newZoomState = ZOOM_STATE_STOP;
		}

		// if we change states, send a message indicating so to the camera		
		if((cameraZoomState != newZoomState) || zoomSpeedChanged)  /* || (cameraLastMessageType != ZOOM_MESSAGE)*/
		{
			cameraZoomState = newZoomState;
			switch(cameraZoomState)
			{
				case ZOOM_STATE_STOP:
					CameraZoomStop();
				break;
				case ZOOM_STATE_IN:
					// might want to bound this input..it will roll over so it's not a gigantic deal
					CameraZoom(IN, abs(REG_CAMERA_VEL_ROT.zoom));
				break;
				case ZOOM_STATE_OUT:
					CameraZoom(OUT, abs(REG_CAMERA_VEL_ROT.zoom));
				break;
			}	
		}
	
	}

	if((cameraMessageState == MESSAGE_ACKNOWLEDGED))
	{
		if((REG_CAMERA_FOCUS_SET != 0) && (REG_CAMERA_FOCUS_SET != REG_CAMERA_FOCUS))
		{
			CameraSetFocus(REG_CAMERA_FOCUS_SET);
		}
		else if((!REG_CAMERA_FOCUS_MANUAL) != autoFocus)
		{
			autoFocus = !REG_CAMERA_FOCUS_MANUAL;
			if(autoFocus)
				CameraAFOn();
			else
				CameraAFOff();
		}
        else if(REG_CAMERA_SHUTTER != shutterSpeed)
        {
            CameraShutterSetup(shutterSpeed);
        }
	}
}


static void TiltControl()
{
	int absTiltVelocity = 0;
	int tiltDir = REG_CAMERA_VEL_ROT.tilt>=0;
	int pwmDuty = 40;


  #ifdef TILT_USB_WATCHDOG_ON
  if(USB_timeout_ms >= ZOOM_MODULE_INIT_MS)
  {
     USB_timeout_ms = USB_TIMEOUT+1;
    // Invert Image
    CameraInvert();
    while(U2STAbits.TRMT==0);
    CameraShutterSetup(shutterSpeed);
    while(U2STAbits.TRMT==0);
    CameraSave();
    while(U2STAbits.TRMT==0);

  }
    //stop moving tilt if USB times out
   else if(USB_timeout_ms >= USB_TIMEOUT)
    {
      USB_timeout_ms = USB_TIMEOUT+1;
      REG_CAMERA_VEL_ROT.tilt = 0;
    }
  #endif

	if((lastTiltVelocity!=0) && (REG_CAMERA_VEL_ROT.tilt==0))
	{
		tiltHoldPosition = potentiometer;
		pwmDuty = 0;
		PORTBbits.RB11=1;
		T2CONbits.TON = 0;
	}
	else if(REG_CAMERA_VEL_ROT.tilt != 0)
	{
		absTiltVelocity = abs(REG_CAMERA_VEL_ROT.tilt);
		int relativeAngle = max((int)potentiometer - TILT_STEPPER_LOWER,0);
		REG_CAMERA_POS_ROT.tilt = (uint16_t)((((float)(relativeAngle))/((float)(TILT_STEPPER_UPPER-TILT_STEPPER_LOWER)))*(TILT_ANGLE_UPPER-TILT_ANGLE_LOWER) + TILT_ANGLE_LOWER);
		PORTBbits.RB11=0;
		T2CONbits.TON = 1;		
	}
	else
	{
		int potProp = (int)potentiometer-(int)tiltHoldPosition;
		int potDiff = potProp-tiltPidLastProp;
                
		tiltPidLastProp = potProp;
		tiltPidSum = (potProp) + (int)(((float)tiltPidSum)*0.90f);
		int pidResult = (potProp<<5) + (potDiff) + tiltPidSum;
		tiltDir = (pidResult) < 0;
		absTiltVelocity = min(1000,abs(pidResult));
		if(abs(potProp) < TILT_POSITION_DEADBAND)
                {
			PORTBbits.RB11=0;
			T2CONbits.TON = 0;
			pwmDuty = 0;
                }
                else
		{
			PORTBbits.RB11=0;
			T2CONbits.TON = 1;
		}
	}

	lastTiltVelocity = REG_CAMERA_VEL_ROT.tilt;	

	if(((potentiometer > TILT_STEPPER_UPPER) && (REG_CAMERA_VEL_ROT.tilt>=0)) || ((potentiometer < TILT_STEPPER_LOWER) && (REG_CAMERA_VEL_ROT.tilt<0)))
		absTiltVelocity = 0;
	Stepper1SetDir(tiltDir);
	PWM1Set(pwmDuty, (int)(262000.0f*(1.0f/(5.0f*((float)absTiltVelocity+1.0f)))));
}

// Main process
void DevicePTZRotationProcessIO()
{
	TimerUpdate();
	Input();
	Recovery();
	InputSummation();
	ZoomControl();
	TiltControl();
}
