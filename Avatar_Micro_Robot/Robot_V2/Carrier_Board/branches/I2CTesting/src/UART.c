/*==============================================================================

File: UART.c
Description: Provides an interface to the UART hardware modules.
Notes:
F_OSC = 32MHz (given our 20MHz crystal, dividy-by-5 from PLL, and 
               multiply-by-8 from somewhere)
F_CY = F_OSC / 2 = 16MHz
==============================================================================*/
//#define TEST_UART
/*---------------------------Dependencies-------------------------------------*/
#include "./UART.h"
#include "./PPS.h"
#include "./StandardHeader.h"

// make ISR's an empty function by default
void DummyISR(void);
void (*U1TX_UserISR)(void) = DummyISR;
void (*U1RX_UserISR)(void) = DummyISR;

/*---------------------------Helper Function Prototypes-----------------------*/
static void ConfigureBaudRate(UARTBaudRate baudRate);

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_UART
#include "./Pause.h"
#include "./Protocol.h"
#include "./ConfigurationBits.h"

#define RS485_OUTEN_EN(a) (_TRISD6 = !(a))
#define RS485_MODE(a)     (_LATD6 = (a))
#define TX_MODE           1
#define RX_MODE           0
#define MY_TX_PIN             20
#define MY_RX_PIN             25

#define TX_PACKET_LENGTH      9

static unsigned char rx_packet[15] = {0};

int main(void) {
  // enable any external IC's
  RS485_OUTEN_EN(1);
  RS485_MODE(TX_MODE);
  
  // assign any application-dependent ISR's
  U1TX_UserISR = U1TX_ISR;  // these must be BEFORE InitUART()
  U1RX_UserISR = U1RX_ISR;
  
  UART_Init(MY_TX_PIN, MY_RX_PIN, kUARTBaudRate9600);
    
  while (1) {
    /*
    // test slow transmission, inspecting result in debugger
    unsigned char i = 0;
    for (i = 0; i < packet_length; i++) {
      UART_TransmitByte(packet[i]);
      Pause(1000);
    }
    */
    
    // inspect rx_packet in debugger window
    // see if anything gets send to python script
    unsigned char dummyBreakPoint = 0;
 	
    // test reception from python script--can we get in here?
  }
  
  return 0;
}


void U1TX_ISR(void) {
  static unsigned char i = 0;
  _U1TXIF = 0;
  
  if (i++ < TX_PACKET_LENGTH) TransmitByte(i);
  else i = 0;
}


void U1RX_ISR(void) {
  _U1RXIF = 0;  
  rx_packet[i] = GetRxByte();
}
#endif

/*---------------------------Public Function Definitions----------------------*/
/*
Notes:
	- by default, configures to 1 stop bit, 8-bit transmission, no parity
*/
void InitUART(unsigned char Tx_pin, unsigned char Rx_pin, 
              UARTBaudRate baudRate) {
	MapPeripheral(Tx_pin, OUTPUT, FN_U1TX);
	MapPeripheral(Rx_pin, INPUT, FN_U1RX);
	ConfigureBaudRate(baud_rate);
	
	_U1RXIP = 6;            // configure interrupt priority
	_U1TXIP = 5;            // Note: 7 is highest priority interrupt
	
	_U1TXIF = 0;            // begin with any interrupt flags cleared
	_U1RXIF = 0;

	IEC0bits.U1TXIE = 1;    // enable UART1 Tx interrupt
	IEC0bits.U1RXIE = 1;    // enable UART1 Rx interrupt
	U1MODEbits.UARTEN = 1;  // enable UART1
	U1STAbits.UTXEN = 1;    // enable transmission
	// BUG ALERT: the UxTXIF bit is set when the module is first enabled
}


void inline TransmitByte(unsigned char message) {
  U1TXREG = message;
}


char inline GetRxByte(void) {
  return (char)U1RXREG;
}


void DummyISR(void) {
}

  
void  __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void) {
 	U1TX_UserISR();
}


void  __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void) {
 	U1RX_UserISR();
}


/*---------------------------Private Function Definitions---------------------*/
/*
Notes:
  - see also Table 21-2 of PIC24F family reference manual
  - UxBRG = F_CY / (16 * desired_baud_rate) - 1 (see p.200 of datasheet)
          = 16MHz / (4 * 9600) - 1
         ~= 416
*/
static void ConfigureBaudRate(UARTBaudRate baudRate) {
  U1MODEbits.BRGH = 1;		// configure for high precision baud rate
  switch (baudRate) {
    case kUARTBaudRate5760:   U1BRG = 68; break;
    case kUARTBaudRate9600:   U1BRG = 416; break;  // 0.07% error
    case kUARTBaudRate115200: U1BRG = 34; break; // 0.62% error
  }
  
}
