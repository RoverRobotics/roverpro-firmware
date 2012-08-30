/*******************************************************************************
File: Boom.c

Description: Overarching file encompassing the application-level logic of the 
  boom-camera module.
*******************************************************************************/
#define TEST_BOOM
/*---------------------------Dependencies-------------------------------------*/
#include "./core/StandardHeader.h"
#include "./core/Timers.h"
#include "./core/UART.h"
#include "./USB/MyUSB.h" // for USB

/*---------------------------Macros-------------------------------------------*/
//---power management
#define ENABLE_VBAT(a)            (_TRISB3 = !(a))
#define TURN_VBAT(a)              (_RB3 = (a))
#define ENABLE_FAN(a)             (_TRISD0 = !(a))	// NOTE: 5V regulator attached to fan pins for current hack!!!
#define TURN_FAN(a)								(_RD0 = (a))

//---indicators
#define ENABLE_HEARTBEAT(a)       (_TRISE5 = (a))
#define HEARTBEAT_PIN             (_RE5)

//---timers
#define _100ms										100
#define HEARTBEAT_TIMER           0
#define HEARTBEAT_TIME            500
#define TX_TIMER                  1

//---NM33 protocol
#define MAX_RX_MESSAGE_LENGTH     25
#define MAX_TX_MESSAGE_LENGTH			24

typedef enum {
	kMinPan = 0,
	kMaxPan = 200,//359, // TODO: how can we tranmist 359 with a uchar?
	kMinTilt = 0,
	kMaxTilt = 90,
	kMinZoom = 10,
	kMaxZoom = 130
} kNM33Limits;


/*---------------------------Type Definitions---------------------------------*/
typedef enum {
  kInitializing = 0,
  kViewing
} BoomState;

/*---------------------------Helper Function Prototypes-----------------------*/
void InitBoom(void);
void ProcessBoomIO(void);
static void InitPins(void);
static void DeinitPins(void);
static void BuildMessage(uint8_t objectID, uint8_t pan, uint8_t tilt, 
                         uint8_t zoom, uint8_t roll, uint8_t message[],
                         uint8_t *messageLength);
static void BuildModeMessage(uint8_t message[], uint8_t *messageLength);

/*---------------------------Module Variables---------------------------------*/
static uint8_t rxMessage[MAX_RX_MESSAGE_LENGTH] = {0};
static uint8_t txMessage[MAX_TX_MESSAGE_LENGTH] = {0};
static uint8_t txMessageLength = 0;

volatile BoomState state = kInitializing;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_BOOM
#include "./core/ConfigurationBits.h"

void U1TX_ISR(void) {
  static uint8_t i = 1;
  _U1TXIF = 0;
  
  if (++i < txMessageLength) UART_TransmitByte(txMessage[i]);
  else i = 0;
  
  /*
	// NB: since already transferred first byte to get in here
  UART_TransmitByte(tx_packet[i]);
  i++;
  if (MAX_TX_PACKET_LENGTH <= i) i = 1; // rollover if we sent the last byte
  */
}


void U1RX_ISR(void) {
  static uint8_t i;
  _U1RXIF = 0;
  if (MAX_RX_MESSAGE_LENGTH <= i) i = 0;
	rxMessage[i] = UART_GetRxByte();
	i++;
}


int main(void) {
  InitBoom();
  
  //SWDTEN = 1;
  while (1) {
    ProcessBoomIO();
  	//ClrWdt(); // clear the software WDT
  }
  
  return 0;
}

#endif
/*---------------------------Public Function Definitions----------------------*/
void InitBoom(void) {
	InitPins();
  
  TMRS_Init();
	
	// assign any application-dependent ISR's
	U1TX_UserISR = U1TX_ISR;  // NB: these must be BEFORE UART initialization
  U1RX_UserISR = U1RX_ISR;
	uint8_t UART1TX_PIN = 8; 
	uint8_t UART1RX_PIN = 9;
  UART_Init(UART1TX_PIN, UART1RX_PIN, kUARTBaudRate115200);
  TMRS_StartTimer(TX_TIMER, _100ms);
  
  TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
  
  // for USB
  uint16_t productID = 0x0012;
  USBDevice_Init(productID);
}


void ProcessBoomIO(void) {
  /*
  static int modeAttemptCounter = 0;
  
  // toggle a pin to indicate normal operation
  if (TMRS_TimerIsExpired(HEARTBEAT_TIMER)) {
    TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
    HEARTBEAT_PIN ^= 1;
  }
  
  switch (state) {
    case kInitializing:
      if (TMRS_TimerIsExpired(TX_TIMER)) {
		    TMRS_StartTimer(TX_TIMER, (50 * _100ms));
		    BuildModeMessage(txMessage, &txMessageLength);
        UART_TransmitByte(txMessage[0]);
        modeAttemptCounter++;
      }
      
      if (3 < modeAttemptCounter) state = kViewing;
      
      break;
    case kViewing:
    
      if (TMRS_TimerIsExpired(TX_TIMER)) {
		    TMRS_StartTimer(TX_TIMER, (10*_100ms));
		    static uint8_t objectID = 1;
    		static uint8_t pan = 180;
    		static uint8_t tilt = 15;
    		static uint8_t zoom = 50;
    		static uint8_t roll = 90;
    		zoom += 10;
    		if (kMaxZoom < zoom) zoom = kMinZoom;
    		BuildMessage(objectID, pan, tilt, zoom, roll, txMessage, &txMessageLength);
    		UART_TransmitByte(txMessage[0]);
      }
      break;
    default:
      ENABLE_HEARTBEAT(0);  // indicate an error
      break;
  }
  */
  
  // for USB
  USBDevice_ProcessMessage();
}

/*---------------------------Helper Function Definitions----------------------*/
static void InitPins(void) {
	DeinitPins();
  
  ENABLE_VBAT(YES); TURN_VBAT(ON);
  ENABLE_FAN(YES); TURN_FAN(ON);
  ENABLE_HEARTBEAT(YES); HEARTBEAT_PIN = 0;
}


static void DeinitPins(void) {
  // return any resources consumed by dependent modules
	TMRS_Deinit();
	
	// configure all I/O pins as digital outputs and ground them
  AD1CON1 = 0x0000; Nop();
	AD1CON2 = 0x0000; Nop();
	AD1CON3 = 0x0000; Nop();
  AD1PCFGL = 0xffff; Nop();
	TRISB = 0x0000; Nop(); PORTB = 0x0000;
	TRISC = 0x0000; Nop(); PORTC = 0x0000;
	TRISD = 0x0000; Nop(); PORTD = 0x0000;
	TRISE = 0x0000; Nop(); PORTE = 0x0000;
	TRISF = 0x0000; Nop(); PORTF = 0x0000;
	TRISG = 0x0000; Nop(); PORTG = 0x0000;
}

/*---------------------------Public Function Definitions----------------------*/
static void BuildModeMessage(uint8_t message[], uint8_t *messageLength) {
  message[0] = 0x1B;      // ESC
  message[1] = 0x73;      // s
  message[2] = 0x65;      // e
  message[3] = 0x74;      // t
  
  message[4] = 0x6D;      // m
  message[5] = 0x6F;      // o
  message[6] = 0x64;      // d
  message[7] = 0x65;      // e
  message[8] = 0x20;      // SPACE
  message[9] = 0x31;      // 1
  message[10] = 0x0A;     // LF
  
  (*messageLength) = 11;
}


/*
Function: BuildMessage
Parameters:
  char objectID,  the id of the camera (associated with a taken image)
  char pan,       pan setting
  char tilt,      tilt setting
  char zoom,      zoom setting
  char roll,      roll setting
  char message[], pointer to the message to populate 
Description: Populates the given message in the protocol
*/
static void BuildMessage(uint8_t objectID, uint8_t pan, uint8_t tilt, 
                         uint8_t zoom, uint8_t roll, uint8_t message[],
                         uint8_t *messageLength) {
  message[0] = 0x1B;      // ESC
  message[1] = 0x73;      // s
  message[2] = 0x65;      // e
  message[3] = 0x74;      // t
  message[4] = 0x6C;      // l
  message[5] = 0x6F;      // o
  message[6] = 0x63;      // c
  message[7] = 0x61;      // a
  message[8] = 0x74;      // t
  message[9] = 0x69;      // i
  message[10] = 0x6F;     // o
  message[11] = 0x6E;     // n
  message[12] = 0x31;     // 1
  message[13] = 0x20;     // SPACE
  message[14] = objectID; // objectID
  message[15] = 0x20;     // SPACE
  message[16] = pan;      // pan
  message[17] = 0x20;     // SPACE
  message[18] = tilt;     // tilt
  message[19] = 0x20;     // SPACE
  message[20] = zoom;     // zoom
  message[21] = 0x20;     // SPACE
  message[22] = roll;     // roll
  message[23] = 0x0A;     // LF
  
  (*messageLength) = 24;
}
