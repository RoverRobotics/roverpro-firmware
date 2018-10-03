/*==============================================================================
File: NM33.c

Notes:
  - can only support object ID's of only 1 character long
  - can only support the "setlocation2" type of message
  - see also: http://www.asciitable.com/
==============================================================================*/
//#define TEST_NM33
//---------------------------Dependencies---------------------------------------
#include "./NM33.h"
#include "./core/StandardHeader.h"
#include "./core/UART.h"

//---------------------------Macros---------------------------------------------
#define MAX_RX_MESSAGE_LENGTH     40
#define MAX_TX_MESSAGE_LENGTH			40
#define MAX_NUM_DIGITS            3

// default message parameters
#define DEFAULT_ID                3
#define DEFAULT_ROLL              90

#define SPACE_IN_ASCII            0x20
#define ESC_IN_ASCII              0x1B
#define LF_IN_ASCII               0x0A  // line feed, new line

//---------------------------Helper Function Prototypes-------------------------
static void U1TX_ISR(void);
static void U1RX_ISR(void);
static void BuildLocationMessage(uint8_t object_id, uint16_t pan, uint8_t tilt, 
                                 uint8_t zoom, uint16_t roll, uint8_t message[],
                                 uint8_t* message_length);
static void IntToAscii(uint16_t n, uint8_t digits_str[], uint8_t* digits_str_length);
static inline uint8_t DigitToAscii(uint8_t digit);
static void MyMemCpy(uint8_t src[], uint8_t src_length, 
                     uint8_t* dest_start_index, uint8_t dest[]);

//---------------------------Module Variables----------------------------------- 
static uint8_t rx_message[MAX_RX_MESSAGE_LENGTH] = {0};
static uint8_t tx_message[MAX_TX_MESSAGE_LENGTH] = {0};
static uint8_t tx_message_length = 0;
static bool is_receptive = 0;

//---------------------------Test Harness---------------------------------------
#ifdef TEST_NM33
#include "./core/StandardHeader.h"  // for configuration bits

#define MY_TX_PIN   8
#define MY_RX_PIN   9

int main(void) {
  // TODO: UPDATE UNIT TEST
  NM33_Init(MY_TX_PIN, MY_RX_PIN);
  
  uint16_t pan = 50;
  
  while (1) {
    Delay(100);
    pan += 1;
    if (kNM33LimitMaxPan < pan) pan = kNM33LimitMinPan;
    //if (NM33_IsReceptive()) NM33_set_location(pan, DEFAULT_TILT, DEFAULT_ZOOM);
    NM33_set_location(pan, DEFAULT_TILT, DEFAULT_ZOOM);
  }
  
  return 0;
}
#endif

//---------------------------Public Function Definitions------------------------
void NM33_Init(uint8_t txPin, uint8_t rxPin) {
	// assign any application-dependent ISR's
	U1TX_UserISR = U1TX_ISR;  // NB: these must be BEFORE UART initialization
  U1RX_UserISR = U1RX_ISR;
	uint8_t UART1TX_PIN = txPin;
	uint8_t UART1RX_PIN = rxPin;
  UART_Init(UART1TX_PIN, UART1RX_PIN, kUARTBaudRate115200);
  // wait for the camera to boot up and for the initial stream of 
  // characters to come in
  Delay(5000);
  is_receptive = 1;
  
  // move to default location
  NM33_set_location(DEFAULT_PAN, DEFAULT_TILT, DEFAULT_ZOOM);
}

bool NM33_IsReceptive(void) {
  // TODO: NOT YET IMPLEMENTED
  return is_receptive;
}

void NM33_set_location(uint16_t pan, uint8_t tilt, uint8_t zoom) {
  BuildLocationMessage(DEFAULT_ID, pan, tilt, zoom, DEFAULT_ROLL,
                       tx_message, &tx_message_length);
  UART_TransmitByte(tx_message[0]);  // begin the transmission sequence
  is_receptive = 0;
}

void NM33_Deinit(void) {
  is_receptive = 0;
  UART_Deinit();
}

//---------------------------Private Function Definitions-----------------------
// Function: BuildLocationMessage
// Parameters:
//   uint8_t object_id, the ID of the camera (associated with a taken image)
//   uint16_t pan,      pan setting
//   uint8_t tilt,      tilt setting
//   uint8_t zoom,      zoom setting
//   uint8_t roll,      roll setting
//   uint8_t message[], pointer to the message to populate
//   uint8_t* message_length, pointer to the message_length to update 
// Description: Populates the given message in the protocol with the given
//   values and a constant mode.
static void BuildLocationMessage(uint8_t object_id, uint16_t pan, uint8_t tilt, 
                                 uint8_t zoom, uint16_t roll, uint8_t message[],
                                 uint8_t* message_length) {
  // convert each numeric parameter to a series of ascii characters
  uint8_t pan_str[MAX_NUM_DIGITS] = {0};
  uint8_t pan_str_length = 0;
  uint8_t tilt_str[MAX_NUM_DIGITS] = {0};
  uint8_t tilt_str_length = 0;
  uint8_t zoom_str[MAX_NUM_DIGITS] = {0};
  uint8_t zoom_str_length = 0;
  uint8_t roll_str[MAX_NUM_DIGITS] = {0};
  uint8_t roll_str_length = 0;
  
  IntToAscii(pan, pan_str, &pan_str_length);
  IntToAscii(tilt, tilt_str, &tilt_str_length);
  IntToAscii(zoom, zoom_str, &zoom_str_length);
  IntToAscii(roll, roll_str, &roll_str_length);
  object_id = DigitToAscii(object_id);
  
  // populate the variable-length message
  message[0] = ESC_IN_ASCII;
  message[1] = 0x73;            // s
  message[2] = 0x65;            // e
  message[3] = 0x74;            // t
  message[4] = 0x6C;            // l
  message[5] = 0x6F;            // o
  message[6] = 0x63;            // c
  message[7] = 0x61;            // a
  message[8] = 0x74;            // t
  message[9] = 0x69;            // i
  message[10] = 0x6F;           // o
  message[11] = 0x6E;           // n
  message[12] = 0x32;           // 2
  message[13] = SPACE_IN_ASCII;
  message[14] = object_id;      // object ID
  message[15] = SPACE_IN_ASCII;
  message[16] = 0x31;           // '1', mode
  message[17] = SPACE_IN_ASCII;
  // pan
  uint8_t i = 18;
  MyMemCpy(pan_str, pan_str_length, &i, message);
  message[i++] = SPACE_IN_ASCII;
  // tilt
  MyMemCpy(tilt_str, tilt_str_length, &i, message);
  message[i++] = SPACE_IN_ASCII;
  // zoom
  MyMemCpy(zoom_str, zoom_str_length, &i, message);
  message[i++] = SPACE_IN_ASCII;
  // roll
  MyMemCpy(roll_str, roll_str_length, &i, message);
  message[i++] = LF_IN_ASCII;
  (*message_length) = i;
}

static void U1TX_ISR(void) {
  static uint8_t i = 0;
  _U1TXIF = 0;
  
	// NB: already transferred first byte to get in here
  // continue transmitting as long as there are bytes to send
  i++;
  if (i < tx_message_length) UART_TransmitByte(tx_message[i]);
  else i = 0;
}

static void U1RX_ISR(void) {
  _U1RXIF = 0;
  static uint8_t i = 0;
  //const uint8_t kNumBytesNotEchoed = 0;
  rx_message[i] = U1RXREG;
  i++;
  if (1 <= i) {
    i = 0;
  }
  // see p.6 section (4) of NM33 Command Description
  // keep track of when the camera finishes its echoed-response
  
  // NB: on startup we receive a large message
  // before transmitting anything, during which tx_message_length = 0
  /*
    if (tx_message_length < (i + kNumBytesNotEchoed)) {
    // tested 1, tested 2, tested 3
    // NB: do NOT count ESC or LF which is NOT echoed back
    i = 0;
    is_receptive = 1;
  }
  */
}

static inline uint8_t DigitToAscii(uint8_t digit) {
  const uint8_t kOffsetToIntegers = 48;
  return (digit + kOffsetToIntegers);
}

// Description: Populates the given statically-declared array with 
//   digits as characters.  For example,
// IntToAscii(123, my_string, my_string_length) populates output as
//   my_string[2] = 1
//   my_string[1] = 2
//   my_string[0] = 3
//   my_string_length = 3
static void IntToAscii(uint16_t n, uint8_t digits_str[],
                       uint8_t* digits_str_length) {
  uint8_t count = 0;
  do {
    digits_str[count++] = DigitToAscii(n % 10);
    n /= 10;
  } while (0 < n);
  
  (*digits_str_length) = count;
}

static void MyMemCpy(uint8_t src[], uint8_t src_length, 
                     uint8_t* dest_start_index, uint8_t dest[]) {
  uint8_t i, j = 0;
  for (i = (*dest_start_index); i < ((*dest_start_index) + src_length); i++) {
    dest[i] = src[(src_length - 1) - j];
    j++;
  }
  
  (*dest_start_index) = i;
}
