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

Responsible Engineer(s): Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
//#define TEST_I2C
/*---------------------------Dependencies-------------------------------------*/
#include "I2C.h"
#include "./StandardHeader.h"

/*---------------------------Type Definitions---------------------------------*/
// states in which the master can be
typedef enum {
  kWaiting = 0,
  kStarting,
  kSelectingDevice,
  kSelectingRegister,
  kWriting,
  kShortstopping,
  kRestarting,
  kReselectingDevice,
  kReading,
  kFinishingAck,
  kStopping
} MasterState;

// indications the master can dictate 
// (1 = read, 0 = write from perspective of master)
typedef enum {
  kIndicationWrite = 0,
  kIndicationRead = 1
} Indication;

/*---------------------------Macros-------------------------------------------*/
#define ACK							          0 // acknowledged
#define NACK						          1	// NOT-acknowledged, error

/*---------------------------Helper Function Prototypes-----------------------*/
static void ConfigureBaudRate(I2CBaudRate baudRate);

/*---------------------------Module Variables---------------------------------*/
static volatile unsigned char isNewDataAvailable = NO;
static volatile unsigned char errorHasOccurred = NO;
static volatile unsigned char slaveAddress = 0;
static volatile unsigned char slaveSubaddress = 0;
static volatile unsigned char indication = kIndicationRead;
static volatile unsigned char remainingRxBytes = 0;    // the number of bytes to 
                                              // receive from slave
static volatile unsigned char remainingTxBytes = 0;
static volatile unsigned char logicalLength = 0;       // Rx buffer logical length
static volatile unsigned char buffer[I2C_MAX_DATA_LENGTH] = {0};
static volatile MasterState state = kWaiting;

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
  /*
  // give power to the sensors
  ENABLE_3V3(1); Nop(); Nop();
	TURN_3V3(ON);
  int sensorData[3] = {0};
  
  I2C_Init(kBaudRate100kHz);
  while (1) {
    I2C_RequestData(TEMP_SENSOR_0_ADDRESS);
    
    if (I2C_IsNewDataAvailable(TEMP_SENSOR_0_ADDRESS) && I2C_IsBusIdle()) {
      I2C_GetData(TEMP_SENSOR_0_ADDRESS, l);
    }
    
  }
  */
  
  return 0;
}
#endif
/*---------------------------Public Function Definitions----------------------*/
void I2C_Init(const I2CBaudRate baudRate) {	
	I2C1CONbits.I2CEN = 0;	// disable the I2C module while we configure it
	
	// Note: the state of the port I/O pins are overridden when I2C is enabled
  I2C_RefreshModule();
	ConfigureBaudRate(baudRate);
	
	I2C1CONbits.SMEN = 0;		// DISABLE thresholds compliant with SMBus specs
	I2C1CONbits.STREN = 0;	// DISABLE receive clock stretching
	I2C1CONbits.GCEN = 0;	  // DISABLE an interrupt to fire when a general call
													// address is received
	
	_MI2C1IF = 0;						// begin with the interrupt cleared
	_MI2C1IE = 1;						// enable I2C interrupts for master-related events
	I2C1CONbits.I2CEN = 1;	// enable the I2C module
}


inline unsigned char I2C_IsBusIdle(void) {
  // see p.53 of Section 24 of PIC24F Family Reference Manual
  return (!I2C1CONbits.SEN && !I2C1CONbits.RSEN && !I2C1CONbits.PEN && 
          !I2C1CONbits.RCEN && !I2C1CONbits.ACKEN && !I2C1STATbits.TRSTAT);  
}


void I2C_RequestData(const I2CDevice *device) {
  slaveAddress = device->address;
  slaveSubaddress = device->subaddress;
  indication = kIndicationRead;
  remainingRxBytes = device->numDataBytes;
  logicalLength = device->numDataBytes;
  I2C1CONbits.SEN = 1;	  // start the start event
  state = kStarting;
}


unsigned char I2C_IsNewDataAvailable(void) {
  if (isNewDataAvailable) {
    isNewDataAvailable = NO;
    return YES;
  }
  
  return NO;
}


void I2C_GetData(I2CDevice *device) {
  // copy in as much of the buffer as is relevant
  unsigned char i;
  for (i = 0; i < logicalLength; i++) device->data[i] = buffer[i];
  
  // clear out irrelevant data?
  //for (i = logicalLength: i < I2C_MAX_DATA_LENGTH; i++) buffer[i] = 0;
}


void I2C_WriteData(const I2CDevice *device, const unsigned char data[]) {
  slaveAddress = device->address;
  slaveSubaddress = device->subaddress;
  indication = kIndicationWrite;
  // BUG ALERT: subtract off the null-terminator
  remainingTxBytes = sizeof(data) / sizeof(unsigned char) - 1;
  logicalLength = remainingTxBytes;
  
  // copy in the data to write
  unsigned char i;
  for (i = 0; i < remainingTxBytes; i++) buffer[i] = data[i];
  
  I2C1CONbits.SEN = 1;	  // start the start event
  state = kStarting;
}


void I2C_RefreshModule(void) {
  // clear any software variables
  isNewDataAvailable = NO;
  slaveAddress = 0;
  slaveSubaddress = 0;
  indication = 0;
  remainingRxBytes = 0;
  remainingTxBytes = 0;
  logicalLength = 0;
  unsigned char i;
  for (i = 0; i < I2C_MAX_DATA_LENGTH; i++) buffer[i] = 0;
  state = kWaiting;
  
  // clear any hardware errors, status flags and buffers
  I2C1STATbits.BCL = 0;		// clear 'bus-collision-error-while-master' flag
  I2C1STATbits.IWCOL = 0;	// clear 'write-collision-error' flag
  I2C1STATbits.I2COV = 0; // clear 'receive-overflow-error' flag
  I2C1CONbits.RCEN = 0;		// clear 'currently-receiving' flag
  char temp;
  temp = I2C1RCV;		      // read to ensure the Rx buffer begins cleared
}


unsigned char I2C_ErrorHasOccurred(void) {
  // check whether there has been a collision or receive overflow
  return (I2C1STATbits.BCL || I2C1STATbits.I2COV || I2C1STATbits.IWCOL || 
         errorHasOccurred);
}


void I2C_Deinit(void) {
  I2C1CONbits.I2CEN = 0;  // turn off I2C and restore consumed pins
  I2C_RefreshModule();
}
  
/*---------------------------Interrupt Service Routines-----------------------*/
/*
Description: This ISR contains a state machine that constitues the core of this
  module.  It is written to be event-driven so that it can be ported to the
  main thread if needed.
*/
void __attribute__((__interrupt__, auto_psv)) _MI2C1Interrupt(void) {
  IFS1bits.MI2C1IF = 0;
  
  switch (state) {
    case kWaiting:
      errorHasOccurred = YES;     // should never get here in interrupt
      break;
    case kStarting:
 	    // wait for confirmation of the hardware clearing the start bit
 	    if (!I2C1CONbits.SEN) {
   	    // transmit the slave address along with the indication
   	    I2C1TRN = ((slaveAddress << 1) | kIndicationWrite);
   	    state = kSelectingDevice;
 	    }
 	    break;
 	  case kSelectingDevice:
 	    // wait for confirmation of the slave acknowledging
 	    if (I2C1STATbits.ACKSTAT == ACK) {
   	    // transmit the slave sub-address
   	    I2C1TRN = slaveSubaddress;
     	  state = kSelectingRegister;
 	    }
 	    break;
 	  case kSelectingRegister:
 	    // wait for confirmation of the slave acknowledging
 	    if (I2C1STATbits.ACKSTAT == ACK) {
        if (indication == kIndicationRead) {
          I2C1CONbits.PEN = 1;    // start a restart event. PEN, then SEN
          state = kShortstopping;
   	    } else if (indication == kIndicationWrite) {
   	      I2C1TRN = buffer[(logicalLength - remainingTxBytes)];
   	      state = kWriting;
 	      }
 	    }
 	    break;
 	  case kWriting:
 	    // wait for confirmation of the slave acknowledging
   	  if (I2C1STATbits.ACKSTAT == ACK) {  
     	  if (!remainingTxBytes) {
     	    I2C1CONbits.PEN = 1;    // start the stop event
     	    state = kStopping;
     	    return;
     	  }
   	    I2C1TRN = buffer[(logicalLength - remainingTxBytes)];
   	    remainingTxBytes--;
   	  }
 	    break;
 	  case kShortstopping:
 	    if (!I2C1CONbits.PEN) {
   	    I2C1CONbits.SEN = 1;
   	    state = kRestarting;
   	  }
 	    break;
 	  case kRestarting:
 	    if (!I2C1CONbits.SEN) {
   	    I2C1TRN = (slaveAddress << 1) | kIndicationRead;
   	    state = kReselectingDevice;
   	  }
 	    break;
 	  case kReselectingDevice:
 	    if (I2C1STATbits.ACKSTAT == ACK) {
   	    I2C1CONbits.RCEN = 1;     // enable reception
 	      state = kReading;
 	    }
 	    break;
 	  case kReading:
      // if the receive buffer is full
      if (I2C1STATbits.RBF) {
        // store the buffer contents first-in-last (first byte arrives last)
        buffer[--remainingRxBytes] = I2C1RCV;
        
        // acknowledge the slave
        if (remainingRxBytes) I2C1CONbits.ACKDT = ACK;
        else I2C1CONbits.ACKDT = NACK;
        
        I2C1CONbits.ACKEN = 1;    // send the contents of ACKDT
        state = kFinishingAck;
      }
 	    break;
 	  case kFinishingAck:
 	    if (!I2C1CONbits.ACKEN) {   // wait to finish acknowledging the slave
   	    if (remainingRxBytes) {
     	    I2C1CONbits.RCEN = 1;   // re-enable reception
     	    state = kReading;
     	  } else {
   	      I2C1CONbits.PEN = 1;    // start the stop event
   	      state = kStopping;
   	    }
   	  }
 	    break;
 	  case kStopping:
 	    // wait for the confirmation of hardware clearing the stop bit
 	    if (!I2C1CONbits.PEN) {
   	    if (indication == kIndicationRead) isNewDataAvailable = YES;
   	    state = kWaiting;
 	    }
 	    break;
 	  default:
 	    errorHasOccurred = YES;     // indicate an error
 	    break;
  }
}

/*
void __attribute__((__interrupt__, auto_psv)) _SI2C1Interrupt(void) {
  // do something
}
*/
/*---------------------------Helper Function Definitions----------------------*/
static void ConfigureBaudRate(I2CBaudRate baudRate) {
  // see Table 16-1 of datasheet
	switch (baudRate) {
    case kI2CBaudRate100kHz: I2C1BRG = 157; break;
    case kI2CBaudRate400kHz: I2C1BRG = 37; break;
    case kI2CBaudRate1MHz:   I2C1BRG = 13; break;
  }
}
