/*=============================================================================

File: UART.c
Description: Provides an interface to the UART hardware modules.


Notes:
F_OSC = 32MHz (given our 20MHz crystal, dividy-by-5 from PLL, and 
               multiply-by-8 from somewhere)
F_CY = F_OSC / 2 = 16MHz
=============================================================================*/
#define TEST_COMMUNICATION

/*---------------------------Dependencies------------------------------------*/
#include "./UART.h"
#include "./Protocol.h"
#include "./ConfigurationBits.h"
#include <p24FJ256GB106.h>

/*---------------------------Macros and Type Definitions---------------------*/
#define THIS_DEVICE           BASE
#define DEVICE_DATA_LENGTH    BASE_DATA_LENGTH
#define UNKNOWN_LENGTH        0xff

#define RS485_OUTEN_EN(a)     (_TRISD6 = !(a))
#define RS485_OUTEN_ON(a)     (_LATD6 = (a))

typedef enum {
  WAITING = 0,
  TRANSMITTING,
  RECEIVING,
} state_t;

/*---------------------------Helper Function Prototypes----------------------*/
static void U1TX_ISR(void);
static void U1RX_ISR(void);
static void TransmitPacket(void);

/*---------------------------Module Variables--------------------------------*/
static unsigned char Tx_packet[MAX_PACKET_LENGTH] = {0};
static unsigned char Tx_packet_length = 0;
static unsigned char Rx_packet[MAX_PACKET_LENGTH] = {0};
static unsigned char Rx_packet_length = UNKNOWN_LENGTH;

//static state_t state = WAITING;
static unsigned char is_rx_packet_avail = 0; 

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_COMMUNICATION
#include "./Pause.h"

int main(void) {
  unsigned char data_out[DEVICE_DATA_LENGTH] = {0, 0xBE, 0xEF, 0xEE};
  
  RS485_OUTEN_EN(1); RS485_OUTEN_ON(1); // turn on the RS485 IC
  InitUART(20, 25, 9600);
  
  while (1) {
    
    // transmission test
    //data_out[0]++;
    BuildPacket(THIS_DEVICE, data_out, Tx_packet, &Tx_packet_length);
    TransmitPacket();
    Pause(100);
    
    /*
    // reception test
    if (is_rx_packet_avail) {
      is_rx_packet_avail = 0;
      unsigned char data_in[MAX_DATA_LENGTH];
      unsigned char data_length = 0;
      GetData(Rx_packet, data_in, &data_length);
      
      // inspect data for debugging
      unsigned char dummy = data_in[0];
      dummy = data_in[1];
      dummy = data_in[2];
      dummy = data_in[3];
      dummy = data_in[4];
      
      _U1RXIE = 1;        // enable the Rx interrupt
    }
    */
  }
  
  return 0;
}
#endif
/*---------------------------End Test Harness--------------------------------*/

/*---------------------------Public Function Definitions---------------------*/
void  __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void) {
 	U1TX_ISR();
}

void  __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void) {
 	U1RX_ISR();
}

/*---------------------------Private Function Definitions--------------------*/
static void TransmitPacket(void) {
  //state = TRANSMITTING;
  _U1TXIE = 1;          // enable the Tx interrupt
  TransmitByte(Tx_packet[0]);
}

/*
Function: U1TX_ISR()
*/
static void U1TX_ISR(void) {
  static unsigned char i_packet = 0;
  _U1TXIF = 0;          // clear the source of the interrupt

  // Note: ++i_packet since sending the first byte got us in here
  if (Tx_packet_length < ++i_packet) {
    i_packet = 0;       // reset the packet index 
    _U1TXIE = 0;        // disable the Tx interrupt
    //state = WAITING;
    return;
  }

  TransmitByte(Tx_packet[i_packet]);
}


static void U1RX_ISR(void) {
  static unsigned char i_packet = 0;
  _U1RXIF = 0;          // clear the source of the interrupt
  Rx_packet[i_packet] = GetRxByte();
  i_packet++;
  
  if (i_packet == DEVICE_INDEX)
    Rx_packet_length = GetDataLength(Rx_packet[i_packet]) + NUM_PREFIX_BYTES 
                       + NUM_DEVICE_BYTES + NUM_SUFFIX_BYTES;
  
  // if this is the last byte
  if ((Rx_packet_length != UNKNOWN_LENGTH) && (Rx_packet_length < i_packet)) {  
    //state = WAITING;
    is_rx_packet_avail = 1;
    Rx_packet_length = UNKNOWN_LENGTH;
    _U1RXIE = 0;        // disable the Rx interrupt
  }

}

/*---------------------------End of File-------------------------------------*/



