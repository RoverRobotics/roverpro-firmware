/*==============================================================================
File: NM33.c
==============================================================================*/
//#define TEST_NM33
/*---------------------------Dependencies-------------------------------------*/
#include "./NM33.h"
#include "./core/StandardHeader.h" // for Delay()
#include "./core/UART.h"

/*---------------------------Macros-------------------------------------------*/
#define MAX_RX_MESSAGE_LENGTH     25
#define MAX_TX_MESSAGE_LENGTH			24

/*---------------------------Helper Function Prototypes-----------------------*/
static void U1TX_ISR(void);
static void U1RX_ISR(void);
static void BuildModeMessage(uint8_t message[], uint8_t *messageLength);
static void BuildLocationMessage(uint8_t objectID, uint8_t pan, uint8_t tilt, 
                                 uint8_t zoom, uint8_t roll, uint8_t message[],
                                 uint8_t *messageLength);

/*---------------------------Module Variables---------------------------------*/ 
static uint8_t rxMessage[MAX_RX_MESSAGE_LENGTH] = {0};
static uint8_t txMessage[MAX_TX_MESSAGE_LENGTH] = {0};
static uint8_t txMessageLength = 0;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_NM33
#include "./ConfigurationBits.h"

#define MY_TX_PIN   8
#define MY_RX_PIN   9

int main(void) {
  NM33_Init(MY_TX_PIN, MY_RX_PIN);
  
  uint8_t pan = 50;
  uint8_t tilt = 0;
  uint8_t zoom = 50;
  
  while (1) {
    Delay(1000);
    pan += 10;
    if (kNM33LimitsMaxPan < pan) pan = kNM33LimitsMinPan;
    NM33_SetLocation(pan, tilt, zoom);
  }
  
  return 0;
}
#endif

/*---------------------------Public Function Definitions----------------------*/
void NM33_Init(uint8_t txPin, uint8_t rxPin) {
	// assign any application-dependent ISR's
	U1TX_UserISR = U1TX_ISR;  // NB: these must be BEFORE UART initialization
  U1RX_UserISR = U1RX_ISR;
	uint8_t UART1TX_PIN = txPin;  // 8 
	uint8_t UART1RX_PIN = rxPin;  // 9
  UART_Init(UART1TX_PIN, UART1RX_PIN, kUARTBaudRate115200);
  
  // wait for the camera to boot up
  Delay(1500);
  BuildModeMessage(txMessage, &txMessageLength);
  UART_TransmitByte(txMessage[0]);
}


void NM33_SetLocation(uint8_t pan, uint8_t tilt, uint8_t zoom) {
  uint8_t roll = 90;    // hard-code roll for now
  uint8_t ID = 1;       // hard-code object ID for now
  
  BuildLocationMessage(ID, pan, tilt, zoom, roll, txMessage, &txMessageLength);
  UART_TransmitByte(txMessage[0]);  // begin the transmission sequence
}


/*
void NM33_Deinit(void) {
  UART_Deinit(void);
}
*/


/*---------------------------Private Function Definitions---------------------*/
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
static void BuildLocationMessage(uint8_t objectID, uint8_t pan, uint8_t tilt, 
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


static void U1TX_ISR(void) {
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


static void U1RX_ISR(void) {
  static uint8_t i;
  _U1RXIF = 0;
  if (MAX_RX_MESSAGE_LENGTH <= i) i = 0;
	rxMessage[i] = UART_GetRxByte();
	i++;
}
