/*==============================================================================
File: I2C.c

Notes:
	- 'clock stretching' = when the slave is allowed to hold the clock line low 
	  (SCL) that is otherwise controlled exclusively by the master.  This is 
		especially important if the slave is a microcontroller but is NOT
		ALLOWED in this version.
		
See Also: 
  - Section 24 PIC24F Family Reference Manual 
  - I2C_Protocol.png describing a typical read sequence in single-master mode
  
TODO: add timeouts in each state in case slave stops responding

Responsible Engineer(s): Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
//#define TEST_I2C
/*---------------------------Dependencies-------------------------------------*/
#include "I2C_new.h"
#include "./StandardHeader.h"

/*---------------------------Type Definitions---------------------------------*/
// states in which the master can be
typedef enum {
  kMasterStateWaiting = 0,
  kMasterStateStarting,
  kMasterStateWaitingForAck,
  kMasterStateReceiving,
  kMasterStateFinishingAck,
  kMasterStateStopping
} MasterState;

// indications the master can dictate 
// (1 = read, 0 = write from perspective of master)
typedef enum {
  kIndicationWrite = 0,
  kIndicationRead = 1,
} Indication;

/*---------------------------Macros-------------------------------------------*/
#define ACK							  0	// acknowledged
#define NACK						  1	// NOT-acknowledged, error

const unsigned char rxDataLength = 1; // TODO: make so can support many-byte reads

/*---------------------------Helper Function Prototypes-----------------------*/
static void ConfigureBaudRate(BaudRate baudRate);
static unsigned char ErrorHasOccurred(void);

/*---------------------------Macros-------------------------------------------*/
static unsigned char buffer[2] = {0, 0};
static unsigned char currentBufferIndex = 0;
static unsigned char isNewDataAvailable = NO;
static unsigned char indication = kIndicationRead;
static unsigned char slaveAddress = 0;

static MasterState state = kMasterStateWaiting;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_I2C
//#include "./ConfigurationBits.h"

#define ENABLE_3V3(a)	            (_TRISC14 = (a))
#define TURN_3V3(a) 	            (_RC14 = (a))

// device addresses on the I2C bus
#define ACCELEROMETER_ADDRESS     0x53
#define TEMP_SENSOR_0_ADDRESS     0x48
#define TEMP_SENSOR_1_ADDRESS     0x49
#define HUMIDITY_SENSOR_ADDRESS   0x1E
#define EEPROM_ADDRESS            0x50 // to 0x57 (0x50 to 0x53 address four 256-byte blocks)
#define FAN_CONTROLLER_ADDRESS		0x18

int main(void) {
  // give power to the sensors
  ENABLE_3V3(1); Nop(); Nop();
	TURN_3V3(ON);
  //extern int dummyData = 0; // TODO: delete this after testing!!!
  
  I2C_Init(kBaudRate100kHz);
  while (1) {
    I2C_RequestData(TEMP_SENSOR_0_ADDRESS);
    
    if (I2C_IsDataAvailable(TEMP_SENSOR_0_ADDRESS)) {
      //dummyData = I2C_GetData(TEMP_SENSOR_0_ADDRESS);
    }
    
    ClrWdt();
  }
  
  return 0;
}
#endif
/*---------------------------Public Function Definitions----------------------*/
void I2C_Init(const BaudRate baudRate) {
	isNewDataAvailable = NO;
	
	I2C1CONbits.I2CEN = 0;	// disable the I2C module while we configure it
	// the state of the relevant port I/O pins are overridden when I2C is enabled
  
	ConfigureBaudRate(baudRate);
	
	I2C1CONbits.SMEN = 0;		// DISABLE thresholds compliant with SMBus specs
	I2C1CONbits.STREN = 0;	// DISABLE receive clock stretching
	I2C1CONbits.GCEN = 0;	  // DISABLE an interrupt to fire when a general call
													// address is received
	
	_MI2C1IF = 0;						// begin with the interrupt cleared
	_MI2C1IE = 1;						// enable I2C interrupts for master-related events
	I2C1CONbits.I2CEN = 1;	// enable the I2C module
	
	//ResetI2CBus();					// set the bus to idle
}

inline unsigned char I2C_IsBusIdle(void) {
  // see p.53 of Section 24 of PIC24F Family Reference Manual
  return (!I2C1CONbits.SEN && !I2C1CONbits.RSEN && !I2C1CONbits.PEN && 
          !I2C1CONbits.RCEN && !I2C1CONbits.ACKEN && !I2C1STATbits.TRSTAT);  
}


void I2C_RequestData(const char deviceAddress) {
  slaveAddress = deviceAddress;
  indication = kIndicationRead;
  I2C1CONbits.SEN = 1;	  // start the start event
  state = kMasterStateStarting;
}


unsigned char I2C_IsNewDataAvailable(const char deviceAddress) {
  return isNewDataAvailable;
}


int I2C_GetData(const char slaveAddress) {
  // Note: most significant byte is read second
  int temp = ((buffer[1]) << 8) | (buffer[0]);
  return temp;
}


/*
unsigned char IsDeviceOnI2CBus(const unsigned char deviceAddress) {
  // TODO: need to make non-blocking
  I2C_RequestData(deviceAddress);
  while (!I2C_IsDataAvailable(deviceAddress)) {};
  return YES;
}
*/

/*---------------------------Interrupt Service Routines-----------------------*/
/*
Description: This interrupt service routine contains a state machine that
  comprises the core of this module.
*/
void __attribute__((__interrupt__, auto_psv)) _MI2C1Interrupt(void) {
  IFS1bits.MI2C1IF = 0;         // clear the source of the interrupt
  
  //if (ErrorHasOcurred()) RefreshI2C();
    
  switch (state) {
    case kMasterStateWaiting:
      // should never get here
      break;
    case kMasterStateStarting:
 	    // poll the SEN bit to determine when the start event completed
 	    if (!I2C1CONbits.SEN) {
   	    // transmit the slave address along with the indication
   	    I2C1TRN = ((slaveAddress << 1) | indication);
   	    state = kMasterStateWaitingForAck;
 	    }
 	    break;
 	  case kMasterStateWaitingForAck:
 	    if (I2C1STATbits.ACKSTAT == ACK) {
   	    I2C1CONbits.RCEN = 1;
   	    state = kMasterStateReceiving;
   	  }
 	    break;
 	  case kMasterStateReceiving:
      // if the receive buffer is full
      if (I2C1STATbits.RBF) {
        buffer[currentBufferIndex++] = I2C1RCV; // get the buffer contents
        // acknowledge the slave with the contents of the ACKDT bit
        I2C1CONbits.ACKDT = ACK;
        I2C1CONbits.ACKEN = 1;
        state = kMasterStateFinishingAck;
      }
      if (rxDataLength <= currentBufferIndex) currentBufferIndex = 0;
 	    break;
 	  case kMasterStateFinishingAck:
 	    if (!I2C1CONbits.ACKEN) {   // wait for the master (us) to finish acknowledging the slave
   	    I2C1CONbits.PEN = 1;      // start the stop event
   	    state = kMasterStateStopping;
   	  }
 	    break;
 	  // Note: we don't actually need this state since the user should 
 	  // check if the bus is idle before requesting data, but it is included
 	  // here for clarity of the sequence
 	  case kMasterStateStopping:
 	    // wait for the confirmation of hardware clearing the stop bit
 	    if (!I2C1CONbits.PEN) {
   	    isNewDataAvailable = YES;
   	    state = kMasterStateWaiting;
 	    }
 	    break;
 	  default:
 	    break;
  }
}

/*
void __attribute__((__interrupt__, auto_psv)) _SI2C1Interrupt(void) {
  // do something
}
*/

/*---------------------------Helper Function Definitions----------------------*/
static void ConfigureBaudRate(BaudRate baudRate) {
  // see Table 16-1
	switch (baudRate) {
    case kBaudRate100kHz: I2C1BRG = 157; break;
    case kBaudRate400kHz: I2C1BRG = 37; break;
    case kBaudRate1MHz:		I2C1BRG = 13; break;
  }
}

  
static unsigned char ErrorHasOccurred(void) {
  // check whether there has been a collision or receive overflow
  return (I2C1STATbits.BCL || I2C1STATbits.I2COV || I2C1STATbits.IWCOL);
}


static void RefreshI2C(void) {
  // clear any previous errors, status flags and buffers
  I2C1STATbits.BCL = 0;		// clear 'bus-collision-error-while-master' flag
  I2C1STATbits.IWCOL = 0;	// clear 'write-collision-error' flag
  I2C1STATbits.I2COV = 0; // clear 'receive-overflow-error' flag
  I2C1CONbits.RCEN = 0;		// clear 'currently-receiving' flag
  char temp = I2C1RCV;		// read the hardware Rx buffer to ensure it begins cleared
  buffer[0] = 0; buffer[1] = 0; // clear the software Rx buffer

  // abort sending the rest of the pending message and prepare to 
  // resend the entire message sequence
  int dummyBreakpoint = 0;
  dummyBreakpoint = 1;
  
  I2C_RequestData(slaveAddress);
}
