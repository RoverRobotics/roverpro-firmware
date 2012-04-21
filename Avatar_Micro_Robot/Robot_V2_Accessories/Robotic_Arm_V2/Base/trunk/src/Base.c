/*=============================================================================
File: Base.c

Description: This is the overarching file that encapsulates the 
  application-level firmware.  It integrates any dependent firmware modules and 
  interfaces with external components.  It is networked with the Avatar on 
  which it is mounted as well as Link_1 and Link_2.
  
Notes:
  - interrupt priorities were ensured to NOT conflict across dependent modules
    _U1RXIP = 6;
    _U1TXIP = 5;
    _T1IP = 4;
    _AD1IP = 3;
    (7 = highest priority, 0 = interrupt disabled)

Author: Stellios Leventis (sleventis@robotex.com)
=============================================================================*/
#define TEST_BASE
/*---------------------------Dependencies------------------------------------*/
#include <p24FJ256GB106.h>
#include "./Timers.h"
#include "./ADC.h"
#include "./PWM.h"
#include "./UART.h"
#include "./Protocol.h"

/*---------------------------Wiring Macros-----------------------------------*/
#define OUTPUT                0
#define INPUT                 1  
#define OFF                   0
#define ON                    1

// Turrent Motor and associated BLDC Driver IC
//#define TR_FF1                (_RD1)           // Fault Flags
//#define TR_FF2                (_RD2)
//#define ENABLE_DIRO(a)        (_TRISB14 = (a)) // set to 1 for input
//#define IS_ROTATING_CCW       (_RB14)          // TODO: confirm CCW or CW
#define ENABLE_TR_BRAKE(a)    (_TRISD7 = !(a))
#define TURN_TR_BRAKE(a)      (_LATD7 = !(a))
#define ENABLE_MODE(a)        (_TRISD8 = !(a))   // current-decay mode
#define SET_MODE_TO(a)        (_LATD8 = (a))
#define SLOW_CURRENT_DECAY    1                  // lower ripple current, but
                                                 // slower dynamic response
                                                 // (see p.10 of A3931 datasheet)
#define ENABLE_COAST(a)       (_TRISD9 = !(a))
#define TURN_COAST(a)      (_LATD9 = !(a))
#define ENABLE_TURRET_DIR(a)  (_TRISD10 = !(a))
#define SET_TURRET_DIR_TO(a)  (_RD10 = a)
#define ENABLE_POWER_BUS(a)   (_TRISD11 = !(a))
#define TURN_POWER_BUS(a)     (_LATD11 = (a))
#define TR_ANALOG_PIN_1       0                  // two potentiometers for feedback
#define TR_ANALOG_PIN_2       1

#define TURRET_PWM_PIN        21
#define T_PWM                 500                // [us], period of the PWM signal (~2-to-8kHz)
#define CW                    0
#define CCW                   1
#define MAX_DC                50
#define MIN_DC                15

// RS485 IC
#define RS485_OUTEN_EN(a)     (_TRISD6 = !(a))
#define RS485_MODE(a)         (_LATD6 = (a))    // 0 for Rx, 1 for Tx
#define RX_MODE               0
#define TX_MODE               1
#define MY_TX_PIN             20
#define MY_RX_PIN             25
#define BAUD_RATE             9600              // [pulse/s]

// Shoulder Potentiometers
#define SH_ANALOG_PIN_1       4
#define SH_ANALOG_PIN_2       5

// Temperature Sensor
#define TEMP_ANALOG_PIN       8

// Debugging
#define CONFIGURE_HEARTBEAT_PIN(a)   (_TRISE5 = (a))
#define HEARTBEAT_PIN         (_RE5)
#define _500ms                500
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        _500ms

// Timeout Timer
#define TIMEOUT_TIMER         1
#define TIMEOUT_TIME          500

/*---------------------------Helper Function Prototypes----------------------*/
static void InitBase(void);
static void ReadBoardID(void);
static void UpdateTxPacketLength(void);
static void UpdateTurretSpeed(signed char speed);
static unsigned char IsMyTurnToTransmit(void);
static void TransmitPacket(void);

/*---------------------------Module Variables--------------------------------*/
static unsigned char board_ID = UNKNOWN_DEVICE;
static unsigned char Tx_packet[MAX_PACKET_LENGTH] = {0};
static unsigned char Tx_packet_length = 0;
static unsigned char Rx_packet[MAX_PACKET_LENGTH] = {0};
static unsigned char Rx_packet_length = INVALID_LENGTH;
static unsigned char data_in[MAX_PACKET_LENGTH] = {0};
static unsigned char data_in_length = INVALID_LENGTH;

static unsigned char is_rx_packet_avail = 0; 

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_BASE
#include "./ConfigurationBits.h"

/*
Link_2 sends motor speed commands at 10Hz (every 100ms).  The Base and
Link_1 wait for the message from the preceding device to finish before 
sending their own.  This means each device's main loop rate must take 
NO LONGER THAN ~30ms.

TODO: time the approximate speed of my while loop by toggling a debugging LED
and inspecing that period on the oscilloscope.
*/
int main(void) {
	unsigned char data_out[BASE_DATA_LENGTH] = {0, 0, 0, 0};
	
	InitBase();
	
  while (1) {
  	// toggle a pin to indicate normal operation
		if (IsTimerExpired(HEARTBEAT_TIMER)) {
			HEARTBEAT_PIN ^= 1;
      StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
		}
		
		// if we lose communication with Link_2, turn everything off as a 
		// safety measure and begin looking for another message 
		if (IsTimerExpired(TIMEOUT_TIMER)) {
		  TURN_TR_BRAKE(ON);
		  UpdateTurretSpeed(0);
		  RS485_MODE(RX_MODE);
		}
		
  	if (IsMyTurnToTransmit()) {
    	TURN_TR_BRAKE(OFF); 
    	StartTimer(TIMEOUT_TIMER, TIMEOUT_TIME);
    		
      // Note: neglecting two LSB's from 10-bit A/D result
      data_out[0] = (GetADC(TR_ANALOG_PIN_1) >> 2); 
      data_out[1] = (GetADC(TR_ANALOG_PIN_2) >> 2);
      data_out[2] = (GetADC(SH_ANALOG_PIN_1) >> 2);
      data_out[3] = (GetADC(SH_ANALOG_PIN_2) >> 2);
      BuildPacket(board_ID, data_out, Tx_packet, &Tx_packet_length);
      TransmitPacket();
      
      // execute any commands
      UpdateTurretSpeed(data_in[0]);
    }
    
	}
	
	return 0;
}

#endif
/*---------------------------End Test Harness--------------------------------*/
void U1TX_ISR(void) {
  static unsigned char i_packet = 0;
  _U1TXIF = 0;

  // Note: ++i_packet since sending the first byte got us in here
  if (Tx_packet_length < ++i_packet) {
    i_packet = 0;
    _U1TXIE = 0;
    RS485_MODE(RX_MODE);
    return;
  }

  TransmitByte(Tx_packet[i_packet]);
}


void U1RX_ISR(void) {
  static unsigned char i_packet = 0;
  _U1RXIF = 0;
  
  unsigned char current_byte = GetRxByte();
  
  // restart if at any point the packet is invalid
  if ( ((i_packet == 0) && (current_byte != HEADER_H)) || 
       ((i_packet == 1) && (current_byte != HEADER_L)) ||
       ((i_packet == 2) && !((current_byte == BASE) || 
                             (current_byte == LINK_1) || 
                             (current_byte == LINK_2))) ) {
    i_packet = 0;
    return;
  }

  Rx_packet[i_packet] = current_byte;
  
  if (i_packet == 2) {
    Rx_packet_length = NUM_PREFIX_BYTES + NUM_DEVICE_BYTES +
                       GetDataLength(current_byte) + NUM_SUFFIX_BYTES;
  }

  i_packet++;
    
  // if this is the last byte ('<= 'to be safer than '==')
  if ((Rx_packet_length != INVALID_LENGTH) && (Rx_packet_length <= i_packet)) {
    GetData(Rx_packet, data_in, &data_in_length);
    if (data_in_length != INVALID_LENGTH) is_rx_packet_avail = 1;
    i_packet = 0;
    Rx_packet_length = INVALID_LENGTH;
    return;
  }
}


/*---------------------------Helper Function Definitions---------------------*/
/*****************************************************************************
Function: InitBase()
Description: Initializes any I/O pins and dependent modules required by the
	 the Base PCB.
******************************************************************************/
static void InitBase(void) {
  CONFIGURE_HEARTBEAT_PIN(OUTPUT); HEARTBEAT_PIN = 0;
  
  ReadBoardID();
  
  // configure any associated external IC's
  RS485_MODE(RX_MODE);
  RS485_OUTEN_EN(1);

  
  // assign any application-dependent ISR's
  U1TX_UserISR = U1TX_ISR;  // BUG ALERT: these must be BEFORE InitUART()
  U1RX_UserISR = U1RX_ISR;
  
	// initialize any dependent moduels
	InitTimers();
	unsigned int analog_bit_mask = ( (1 << TEMP_ANALOG_PIN) |
                                   (1 << SH_ANALOG_PIN_2) |
                                   (1 << SH_ANALOG_PIN_1) |
                                   (1 << TR_ANALOG_PIN_2) |
                                   (1 << TR_ANALOG_PIN_1) );
  InitADC(analog_bit_mask);
  InitPWM(TURRET_PWM_PIN, T_PWM);
  
  InitUART(MY_TX_PIN, MY_RX_PIN, BAUD_RATE);  // BUG ALERT: the UxTXIF is set
                                              // when the module is enabled
  UpdateTxPacketLength();                     // do this after InitUART() so initial Tx interrupt
                                              // does nothing
  
  // initialize the turret motor controller
  ENABLE_TR_BRAKE(1); TURN_TR_BRAKE(OFF);
  ENABLE_COAST(1); TURN_COAST(OFF);
  ENABLE_MODE(1); SET_MODE_TO(SLOW_CURRENT_DECAY);
  ENABLE_TURRET_DIR(1); SET_TURRET_DIR_TO(CCW);
  ENABLE_POWER_BUS(1); TURN_POWER_BUS(ON);
  
	// prime any timers that require it
	StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
	StartTimer(TIMEOUT_TIMER, TIMEOUT_TIME);
}

/*****************************************************************************
Function: ReadBoardID()
Description: Reads the board identification number--set in hardware by pull-up
  and pull-down resistors--into a module level variable.
******************************************************************************/
static void ReadBoardID(void) {
  _TRISE4 = 1; _TRISE3 = 1; _TRISE2 = 1; _TRISE1 = 1; _TRISE0 = 1;
  board_ID = ((_RE4 << 4) | (_RE3 << 3) | (_RE2 << 2) | (_RE1 << 1) | (_RE0 << 0));
}

/*****************************************************************************
Function: UpdateTxPacketLength()
Description: Updates the transmission packet length given the board ID.
******************************************************************************/
static void UpdateTxPacketLength(void) {
  Tx_packet_length = NUM_PREFIX_BYTES + NUM_DEVICE_BYTES + NUM_SUFFIX_BYTES;
  
  switch (board_ID) {
    case BASE: Tx_packet_length += BASE_DATA_LENGTH; break;
    case LINK_1: Tx_packet_length += LINK_1_DATA_LENGTH; break;
    case LINK_2: Tx_packet_length += LINK_2_DATA_LENGTH; break;
    default: Tx_packet_length = INVALID_LENGTH; break;
  }
}
  
/*****************************************************************************
Function: UpdateTurretSpeed()
Description: Maps a speed given as an integer value between -100 and 100 to 
 to a valid duty cycle between 0 and 100.
TODO: incorporate braking?
******************************************************************************/
static void UpdateTurretSpeed(signed char speed) {
  if (speed < 0) {
    SET_TURRET_DIR_TO(CW);
    speed = -speed;
  } else {
    SET_TURRET_DIR_TO(CCW);
  }
  
  // cap the duty cycle
  if (MAX_DC < speed) speed = MAX_DC;
  else if (speed < MIN_DC) speed = MIN_DC;
  
  UpdateDutyCycle((unsigned char)speed);
}

static void TransmitPacket(void) {
  RS485_MODE(TX_MODE);
  _U1TXIE = 1;                // enable the Tx interrupt
  TransmitByte(Tx_packet[0]); // begin the transmission sequence 
}

static unsigned char IsMyTurnToTransmit(void) {
  if (!is_rx_packet_avail) return 0;
  is_rx_packet_avail = 0;
  
  switch (board_ID) {
    case BASE: return (Rx_packet[DEVICE_INDEX] == LINK_2);
    case LINK_1: return (Rx_packet[DEVICE_INDEX] == BASE);
    case LINK_2: return (Rx_packet[DEVICE_INDEX] == LINK_1);
    default: return 0;
  }
}
  
/*---------------------------End of File-------------------------------------*/
