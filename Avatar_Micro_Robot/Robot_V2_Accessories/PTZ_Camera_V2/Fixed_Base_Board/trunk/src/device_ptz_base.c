#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "interrupt_switch.h"
#include <stdio.h>
#include <math.h>

#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
	#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

// -------------------------------------------------------------------------
// Type declarations
// -------------------------------------------------------------------------


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
#define PAN_USB_WATCHDOG_ON

#define PAN_POSITION_DEADBAND 5

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
#define PWM1_RPn	RPOR1bits.RP2R

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

#define PAN_POTENTIOMETER_MAX 0x450

// -------------------------------------------------------------------------
// Local State Variables
// -------------------------------------------------------------------------

static uint32_t ticks = 0;
static float potentiometer = 0;
static float potentiometer2 = 0;
static int32_t potentiometerMerged = 0;

float debug_pot = 0.0f;
float debug_pot2 = 0.0f;

static int lastPanVelocity = 0;
static uint32_t panHoldPosition = 530;
static float panPidSum = 0;
static int panPidLastProp = 0;

// -------------------------------------------------------------------------
/*
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
*/
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
/*
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
*/
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
	{
		potentiometer = ((float)ADC1BUF0)*0.01f + potentiometer*0.99f;
		potentiometer2 = ((float)ADC1BUF1)*0.01f + potentiometer2*0.99f;

        debug_pot = potentiometer;
        debug_pot2 = potentiometer2;
	}
	
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
  //there's some weird difference in the math (rounding?), so add a special case for
  //a duty cycle of 0
  if(duty == 0)
  {
    OC1R = ((period<<4)-2);
  }
  else
  {
	  OC1R = pr2Value - duty*(pr2Value/100);
  }
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
		PORTDbits.RD7=1;
	else
		PORTDbits.RD7=0;
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
	AD1PCFGbits.PCFG1 = 0;
	AD1CSSL = 0x0003;
	IEC0bits.AD1IE=1;
	AD1CON2bits.CSCNA=1;
	AD1CON2bits.SMPI0=1;
	
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
// 	RPINR18bits.U1RXR = U1RX_RPn;
 
//	U1TX_RPn = 3; //3 represents U1TX

	// UART for camera
	RPINR19bits.U2RXR = U2RX_RPn;
 
	U2TX_RPn = 5; //0 represents U2TX

	PWM1_RPn = 18;
	
	//***************************
	// Lock Registers
	__builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to
	//lock Pin Re-map

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

void DevicePTZBaseInit()
{
	//peripheral pin selection
	PinRemap();
	//peripheral pin selection end
	//*******************************************
	//Using channels 0 and 1
	AD1PCFGL = 0xFFFC;
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
	TRISDbits.TRISD8=0;

	// board enable (fixed board ONLY)
	TRISBbits.TRISB3=0;
	PORTBbits.RB3=1;

	// fixed board fan enable
	TRISDbits.TRISD0=0;
	PORTDbits.RD0=1;

	// STEPPER MOTOR ~ENABLE
	TRISDbits.TRISD10=0;
	PORTDbits.RD10=0;
	// STEPPER MOTOR DIR
	TRISDbits.TRISD7=0;
	PORTDbits.RD7=0;
	// STEPPER MOTOR MS1
	TRISDbits.TRISD4=0;
	PORTDbits.RD4=1;
	// STEPPER MOTOR MS2
	TRISDbits.TRISD5=0;
	PORTDbits.RD5=1;
	// STEPPER TURBO
	TRISDbits.TRISD6=0;
	PORTDbits.RD6=1;
	// STEPPER MOTOR RESET
	TRISDbits.TRISD9=0;
	PORTDbits.RD9=1;
	// STEPPER MOTOR SLEEP
	TRISDbits.TRISD11=0;
	PORTDbits.RD11=1;
	// ADC 0
	TRISBbits.TRISB0=1;
	// ADC 1
	TRISBbits.TRISB1=1;

	// set up interrupt handlers for UART1 and Timer1 (UART2 is done separately for now)
	U1RXInterruptUserFunction = RXInterrupt;
	U1TXInterruptUserFunction = TXInterrupt;
	T1InterruptUserFunction = T1Interrupt;
	ADC1InterruptUserFunction = ADC1Interrupt;

// 	UART1Ini();
	UART2Ini();
	ADC1Ini();
	IniTimer1();

	IniPWM1();
}

void InputSummation()
{
	if((potentiometer > 0xBF) && (potentiometer < 0x128))
		potentiometerMerged = 0x3FF+max(-1*((int)(((int)potentiometer-((int)0x128)))),0)-25;
	else
		potentiometerMerged = potentiometer2;
	potentiometerMerged=max((int)potentiometerMerged,0);
	REG_CAMERA_POS_BASE = (360*min(potentiometerMerged, PAN_POTENTIOMETER_MAX))/PAN_POTENTIOMETER_MAX;
}

int panDisplacement(int angle1, int angle2)
{
	int disp1 = (abs((angle1-angle2))%PAN_POTENTIOMETER_MAX);
	int disp2 = (abs(PAN_POTENTIOMETER_MAX-(angle1-angle2))%PAN_POTENTIOMETER_MAX);
	if(disp1>=(disp2))
		return disp2;	
	else
		return disp1*-1;		
}

void PanControl()
{
	static int lastPotentiometer = 0;
	static int lastPotentiometer2 = 0;
	static int potentiometerAvg = 0;
	int absPanVelocity = 0;
	int panDir = 1;
	int potDiff=0;
  int pwmDuty = 40;

  //stop moving pan if USB times out
  #ifdef PAN_USB_WATCHDOG_ON
 if(USB_timeout_ms >= USB_TIMEOUT)
  {
    USB_timeout_ms = USB_TIMEOUT+1;
    REG_CAMERA_VEL_BASE = 0;
  }
  #endif

  panDir = REG_CAMERA_VEL_BASE>=0;
  

	if((lastPanVelocity!=0) && (REG_CAMERA_VEL_BASE==0))
	{
		panHoldPosition = potentiometerMerged;
		PORTDbits.RD10=1;
	}
	else if(REG_CAMERA_VEL_BASE != 0)
	{
		absPanVelocity = abs(REG_CAMERA_VEL_BASE);
		pwmDuty = 10;
		PORTDbits.RD10=0;
	}
	else
	{
		int potProp = panDisplacement(potentiometerMerged,panHoldPosition);
		potDiff = potProp-panPidLastProp;
		panPidLastProp = potProp;
                
		panPidSum = 0.1f*((float)potProp) + (((float)panPidSum)*0.9f);
		int pidResult = 8*(potProp) + 0*(potDiff) + 0*(int)panPidSum;
		panDir = (pidResult) > 0;
		absPanVelocity = min(1000,abs(pidResult));
                if((abs(potProp) < PAN_POSITION_DEADBAND))
                {
                    panPidSum = 0.0f;
                    panPidLastProp = 0;
					PORTDbits.RD10=0;
					pwmDuty = 0;
                }
                else
		{
			PORTDbits.RD10=0;
		}
	}

	lastPanVelocity = REG_CAMERA_VEL_BASE;	

	
	Stepper1SetDir(panDir);

	PWM1Set(pwmDuty, (int)(262000.0f*(1.0f/(5.0f*((float)absPanVelocity+1.0f)))));

	lastPotentiometer = potentiometerMerged;
}

// Main process
void DevicePTZBaseProcessIO()
{
	InputSummation();
	PanControl();

  if(REG_CAMERA_BASE_POWER_DOWN)
  {
    _LATB3 = 0;
    block_ms(200);
    REG_CAMERA_BASE_POWER_DOWN = 0;
  }
  else
    _LATB3 = 1;

}
