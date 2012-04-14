/*=============================================================================
 File: Base.c
=============================================================================*/
#define TEST_BASE
/*---------------------------Dependencies------------------------------------*/
#include "./Base.h"
#include <p24FJ256GB106.h>
#include "./ConfigurationBits.h"
#include "./Timers.h"
#include "./ADC.h"
#include "./PWM.h"
#include "./UART.h"
#include "./Protocol.h"


/*---------------------------Type Definitions--------------------------------*/
/*
typedef enum {
  WAITING = 0,
  TRANSMITTING,
  RECEIVING,
} state_t;

typedef enum {
	EV_NO_EVENT = 0,
	EV_ENTRY,
	EV_EXIT
// TODO: layout events for all state machines in separate file
} event_t;
*/
/*---------------------------Wiring Macros-----------------------------------*/
#define OFF                   0
#define ON                    1

// Turrent Motor and associated BLDC Driver IC
//#define TR_FF1                (_RD1)           // Fault Flags
//#define TR_FF2                (_RD2)
//#define ENABLE_DIRO(a)        (_TRISB14 = (a)) // set to 1 for input
//#define IS_ROTATING_CCW       (_RB14)          // TODO: confirm CCW or CW
#define ENABLE_TR_BRAKE(a)    (_TRISD7 = !(a))
#define TURN_TR_BRAKE(a)      (_LATD7 = (a))
#define ENABLE_MODE(a)        (_TRISD8 = !(a))   // current-decay mode
#define SET_MODE_TO(a)        (_LATD8 = (a))
#define SLOW_CURRENT_DECAY    1                  // lower ripple current, but
                                                 // slower dynamic response
                                                 // (see p.10 of A3931 datasheet)
//#define ENABLE_COAST(a)       (_TRISD9 = (a))
//#define TURN_ON_COAST(a)      (_LATD9 = !(a))
#define ENABLE_TURRET_DIR(a)  (_TRISD10 = (a))
#define SET_TURRET_DIR_TO(a)  (_RD10 = a)
#define ENABLE_POWER_BUS(a)   (_TRISD11 = (a))
#define TURN_POWER_BUS(a)     (_LATD11 = (a))
#define TR_ANALOG_PIN_1       0                  // two potentiometers for feedback
#define TR_ANALOG_PIN_2       1

#define TURRET_PWM_PIN        21
#define T_PWM                 500 // [us], period of the PWM signal (~2-to-8kHz)
#define CW                    0
#define CCW                   1


// RS485 IC
#define RS485_OUTEN_EN(a)     (_TRISD6 = !(a))
#define RS485_OUTEN_ON(a)     (_LATD6 = (a))    // 0 for Rx, 1 for Tx
#define MY_TX_PIN             20
#define MY_RX_PIN             25
#define BAUD_RATE             9600              // [pulse/s]

// Shoulder Potentiometers
#define SH_ANALOG_PIN_1       4
#define SH_ANALOG_PIN_2       5

// Temperature Sensor
#define TEMP_ANALOG_PIN       8

// Debugging
#define ENABLE_HEARTBEAT_PIN(a)   (_TRISE5 = (a))
#define HEARTBEAT_PIN         (_RE5)
#define _500ms                500
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        _500ms

/*---------------------------Helper Function Prototypes----------------------*/
static void InitBase(void);
static void ConfigurePins(void);
static void ReadBoardID(void);
static void UpdateTurretSpeed(signed char speed);

/*---------------------------Module Variables--------------------------------*/
// use volatile to prevent compiler from optimizing away variables that get 
// updated in ISR's
static unsigned char board_ID = UNKNOWN_DEVICE;
static volatile unsigned char Tx_packet[MAX_PACKET_LENGTH] = {0};
static volatile unsigned char Tx_packet_length = 0;
static volatile unsigned char Rx_packet[MAX_PACKET_LENGTH] = {0};
static volatile unsigned char Rx_packet_length = INVALID_LENGTH;

//static state_t state = WAITING;
static volatile unsigned char is_rx_packet_avail = 0; 

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_BASE

/*
Link_2 sends motor speed commands at 10Hz (every 100ms).  The Base and
Link_1 wait for the message from the preceding device to finish before 
sending their own.  This means each device's main loop rate must take 
NO LONGER THAN ~30ms.

TODO: time the approximate speed of my while loop by toggling a debugging LED
and inspecing that period on the oscilloscope.
*/
int main() {
	InitBase();
  while (1) {
  	// toggle a pin to indicate normal operation
		if (IsTimerExpired(HEARTBEAT_TIMER)) {
			HEARTBEAT_PIN ^= 1;
		  StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
		}
		
  	// update motor PWM to most recent message command
  	/*
  	GetData(packet, data, &data_length_ptr);
    */
    // UpdateTurretSpeed(new_speed);
  	
  	// transmit my feedback
  	/*
    if (IsMyTurnToTransmit()) {
  	  BuildPacket(board_ID, data_out, Tx_packet, &Tx_packet_length);
      TransmitPacket();
    }
    */
                 
	}
	
	return 0;
}

#endif
/*---------------------------End Test Harness--------------------------------*/
// TODO: test if these still work as static
void U1TX_ISR(void) {
  static unsigned char i_packet = 0;
  _U1TXIF = 0;

  // Note: ++i_packet since sending the first byte got us in here
  if (Tx_packet_length < ++i_packet) {
    i_packet = 0;
    _U1TXIE = 0;
    //state = WAITING;
    return;
  }

  TransmitByte(Tx_packet[i_packet]);
}


void U1RX_ISR(void) {
  static unsigned char i_packet = 0;
  _U1RXIF = 0;
  Rx_packet[i_packet] = GetRxByte();
  i_packet++;
  
  if (i_packet == DEVICE_INDEX)
    Rx_packet_length = GetDataLength(Rx_packet[i_packet]) + NUM_PREFIX_BYTES 
                       + NUM_DEVICE_BYTES + NUM_SUFFIX_BYTES;
  
  // if this is the last byte
  if ((Rx_packet_length != INVALID_LENGTH) && (Rx_packet_length < i_packet)) {  
    //state = WAITING;
    is_rx_packet_avail = 1;
    Rx_packet_length = INVALID_LENGTH;
    //_U1RXIE = 0;
  }
}


/*---------------------------Helper Function Definitions---------------------*/
/*****************************************************************************
Function: InitBase()
Description: Initializes any I/O pins and dependent modules required by the
	 the Base PCB.
******************************************************************************/
static void InitBase(void) {
  ENABLE_HEARTBEAT_PIN(1); HEARTBEAT_PIN = 0;
  
  // initialize the turret motor controller
  ENABLE_TR_BRAKE(1); TURN_TR_BRAKE(OFF);
  ENABLE_MODE(1); SET_MODE_TO(SLOW_CURRENT_DECAY);
  ENABLE_TURRET_DIR(1); SET_TURRET_DIR_TO(CCW);
  ENABLE_POWER_BUS(1); TURN_POWER_BUS(ON);
  
  ReadBoardID();
	
	// initialize any dependent moduels
	InitTimers();
	unsigned int analog_bit_mask = ( (1 << TEMP_ANALOG_PIN) |
                                   (1 << SH_ANALOG_PIN_2) |
                                   (1 << SH_ANALOG_PIN_1) |
                                   (1 << TR_ANALOG_PIN_2) |
                                   (1 << TR_ANALOG_PIN_1) );
  InitADC(analog_bit_mask);
  InitPWM(TURRET_PWM_PIN, T_PWM);
  U1TX_UserISR = U1TX_ISR;      // use my own functions for the ISR's
  U1RX_UserISR = U1RX_ISR;
  InitUART(MY_TX_PIN, MY_RX_PIN, BAUD_RATE);
  
	// prime any timers that require it
	StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
}

/*****************************************************************************
Function: ReadBoardID()
Description: Reads the board identification number--set in hardware by pull-up
  and pull-down resistors--into a module-level variable.
******************************************************************************/
static void ReadBoardID(void) {
  _TRISE4 = 1; _TRISE3 = 1; _TRISE2 = 1; _TRISE1 = 1; _TRISE0 = 1;
  board_ID = (_RE4 << 4) | (_RE3 << 3) | (_RE2 << 2) | (_RE1 << 1) | (_RE0 << 0);
  
  switch (board_ID) {
    case BASE: Tx_packet_length = BASE_DATA_LENGTH; break;
    case LINK_1: Tx_packet_length = LINK_1_DATA_LENGTH; break;
    case LINK_2: Tx_packet_length = LINK_2_DATA_LENGTH; break;
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
  if (speed < 0) SET_TURRET_DIR_TO(CW);
  else SET_TURRET_DIR_TO(CCW);
  
  if (100 < speed) speed = 100;
  else if (speed < -100) speed = -100;
  if (speed < 0) speed = -speed;
  
  UpdateDutyCycle((unsigned char)speed);
}

static void TransmitPacket(void) {
  //state = TRANSMITTING;
  _U1TXIE = 1;          // enable the Tx interrupt
  TransmitByte(Tx_packet[0]);
}

/*---------------------------End of File-------------------------------------*/
