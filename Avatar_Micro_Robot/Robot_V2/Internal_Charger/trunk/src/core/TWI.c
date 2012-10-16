/*==============================================================================
File: TWI.c

Notes:
	- 'clock stretching' = when the slave is allowed to hold the clock line low 
	  (SCL) that is otherwise controlled exclusively by the master.  This is 
		especially important if the slave is a microcontroller but is NOT
		ALLOWED in this version.
		
See Also:
  - Section 24 PIC24F Family Reference Manual
==============================================================================*/
//#define TEST_TWI
/*---------------------------Dependencies-------------------------------------*/
#include "./TWI.h"

/*---------------------------Macros-------------------------------------------*/
#define ACK							          0 // acknowledged
#define NACK						          1	// NOT-acknowledged, error

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

/*---------------------------Helper Function Prototypes-----------------------*/
static void ConfigureBaudRate(kTWIBaudRate baud_rate);

/*---------------------------Module Variables---------------------------------*/
static volatile uint8_t isNewDataAvailable = NO;
static volatile uint8_t errorHasOccurred = NO;
static volatile uint8_t slaveAddress = 0;
static volatile uint8_t slaveSubaddress = 0;
static volatile uint8_t indication = kIndicationRead;
static volatile uint8_t remainingRxBytes = 0;    // the number of bytes to 
                                              // receive from slave
static volatile uint8_t remainingTxBytes = 0;
static volatile uint8_t logicalLength = 0;       // Rx buffer logical length
static volatile uint8_t buffer[TWI_MAX_DATA_LENGTH] = {0};
static volatile MasterState state = kWaiting;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_TWI
// TODO: RE_WRITE UNIT TEST NOW THAT PUBLIC INTERFACE HAS CHANGED!!!!
#include "./ConfigurationBits.h"

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
void TWI_Init(const kTWIModule module,
              const kTWIBaudRate baudRate, 
              const bool isSMBus) {
  switch (module) {
    case kTWI01: break;
    case kTWI02: break;
    case kTWI03: break;
  }
  
	I2C2CONbits.I2CEN = 0;	// disable the I2C module while we configure it
	
	// Note: the state of the port I/O pins are overridden when I2C is enabled
  TWI_RefreshModule(module);
	ConfigureBaudRate(baudRate);
	
	I2C2CONbits.SMEN = isSMBus;		// configure I2C/SMBus thresholds
	I2C2CONbits.STREN = 0;	// DISABLE receive clock stretching
	I2C2CONbits.GCEN = 0;	  // DISABLE an interrupt to fire when a general call
													// address is received
	
	_MI2C2IF = 0;						// begin with the interrupt cleared
	_MI2C2IE = 1;						// enable I2C interrupts for master-related events
	I2C2CONbits.I2CEN = 1;	// enable the I2C module
}


void TWI_RequestData(const kTWIModule module, const TWIDevice* device) {
  slaveAddress = device->address;
  slaveSubaddress = device->subaddress;
  indication = kIndicationRead;
  remainingRxBytes = device->numDataBytes;
  logicalLength = device->numDataBytes;
  I2C2CONbits.SEN = 1;	  // start the start event
  state = kStarting;
}


inline bool TWI_IsBusIdle(const kTWIModule module) {
  // see p.53 of Section 24 of PIC24F Family Reference Manual
  return (!I2C2CONbits.SEN && !I2C2CONbits.RSEN && !I2C2CONbits.PEN && 
          !I2C2CONbits.RCEN && !I2C2CONbits.ACKEN && !I2C2STATbits.TRSTAT);  
}


inline bool TWI_IsNewDataAvailable(const kTWIModule module) {
  if (isNewDataAvailable) {
    isNewDataAvailable = NO;
    return YES;
  }
  
  return NO;
}


void TWI_GetData(const kTWIModule module, TWIDevice* device) {
  // copy in as much of the buffer as is relevant
  uint8_t i;
  for (i = 0; i < logicalLength; i++) device->data[i] = buffer[i];
  
  // clear out irrelevant data?
  //for (i = logicalLength: i < TWI_MAX_DATA_LENGTH; i++) buffer[i] = 0;
}


void TWI_WriteData(const kTWIModule module, 
                   const TWIDevice* device, 
                   const uint8_t data[]) {
  slaveAddress = device->address;
  slaveSubaddress = device->subaddress;
  indication = kIndicationWrite;
  // BUG ALERT: subtract off the null-terminator
  remainingTxBytes = sizeof(data) / sizeof(unsigned char) - 1;
  logicalLength = remainingTxBytes;
  
  // copy in the data to write
  uint8_t i;
  for (i = 0; i < remainingTxBytes; i++) buffer[i] = data[i];
  
  I2C2CONbits.SEN = 1;	  // start the start event
  state = kStarting;
}


bool TWI_ErrorHasOccurred(const kTWIModule module) {
  // check whether there has been a collision or receive overflow
  return (I2C2STATbits.BCL || I2C2STATbits.I2COV || I2C2STATbits.IWCOL || 
         errorHasOccurred);
}


void TWI_RefreshModule(const kTWIModule module) {
  // clear any software variables
  isNewDataAvailable = NO;
  slaveAddress = 0;
  slaveSubaddress = 0;
  indication = 0;
  remainingRxBytes = 0;
  remainingTxBytes = 0;
  logicalLength = 0;
  uint8_t i;
  for (i = 0; i < TWI_MAX_DATA_LENGTH; i++) buffer[i] = 0;
  state = kWaiting;
  
  // clear any hardware errors, status flags and buffers
  I2C2STAT = 0x0000;
  I2C2CONbits.RCEN = 0;		// clear 'currently-receiving' flag
  int8_t temp;
  temp = I2C2RCV;		      // read to ensure the Rx buffer begins cleared
}


void TWI_Deinit(const kTWIModule module) {
  I2C2CONbits.I2CEN = 0;  // turn off I2C and restore consumed pins
  TWI_RefreshModule(module);
  
  // restore any registers to their startup defaults
  I2C2CON = 0x0000; // Note: default actually leaves I2C on, but keeping off
  I2C2TRN = 0x00ff;
  I2C2BRG = 0x0000;
  I2C2STAT = 0x0000;
}
  
/*---------------------------Interrupt Service Routines-----------------------*/
/*
Description: This ISR contains a state machine that constitues the core of this
  module.  It is written to be event-driven so that it can be ported to the
  main thread if needed.
*/
void __attribute__((__interrupt__, auto_psv)) _MI2C2Interrupt(void) {
  IFS3bits.MI2C2IF = 0;
  
  switch (state) {
    case kWaiting:
      errorHasOccurred = YES;     // should never get here in interrupt
      break;
    case kStarting:
 	    // wait for confirmation of the hardware clearing the start bit
 	    if (!I2C2CONbits.SEN) {
   	    // transmit the slave address along with the indication
   	    I2C2TRN = ((slaveAddress << 1) | kIndicationWrite);
   	    state = kSelectingDevice;
 	    }
 	    break;
 	  case kSelectingDevice:
 	    // wait for confirmation of the slave acknowledging
 	    if (I2C2STATbits.ACKSTAT == ACK) {
   	    // transmit the slave sub-address
   	    I2C2TRN = slaveSubaddress;
     	  state = kSelectingRegister;
 	    }
 	    break;
 	  case kSelectingRegister:
 	    // wait for confirmation of the slave acknowledging
 	    if (I2C2STATbits.ACKSTAT == ACK) {
        if (indication == kIndicationRead) {
          I2C2CONbits.PEN = 1;    // start a restart event. PEN, then SEN
          state = kShortstopping;
   	    } else if (indication == kIndicationWrite) {
   	      I2C2TRN = buffer[(logicalLength - remainingTxBytes)];
   	      state = kWriting;
 	      }
 	    }
 	    break;
 	  case kWriting:
 	    // wait for confirmation of the slave acknowledging
   	  if (I2C2STATbits.ACKSTAT == ACK) {  
     	  if (!remainingTxBytes) {
     	    I2C2CONbits.PEN = 1;    // start the stop event
     	    state = kStopping;
     	    return;
     	  }
   	    I2C2TRN = buffer[(logicalLength - remainingTxBytes)];
   	    remainingTxBytes--;
   	  }
 	    break;
 	  case kShortstopping:
 	    if (!I2C2CONbits.PEN) {
   	    I2C2CONbits.SEN = 1;
   	    state = kRestarting;
   	  }
 	    break;
 	  case kRestarting:
 	    if (!I2C2CONbits.SEN) {
   	    I2C2TRN = (slaveAddress << 1) | kIndicationRead;
   	    state = kReselectingDevice;
   	  }
 	    break;
 	  case kReselectingDevice:
 	    if (I2C2STATbits.ACKSTAT == ACK) {
   	    I2C2CONbits.RCEN = 1;     // enable reception
 	      state = kReading;
 	    }
 	    break;
 	  case kReading:
      // if the receive buffer is full
      if (I2C2STATbits.RBF) {
        // store the buffer contents first-in-last (first byte arrives last)
        buffer[--remainingRxBytes] = I2C2RCV;
        
        // acknowledge the slave
        if (remainingRxBytes) I2C2CONbits.ACKDT = ACK;
        else I2C2CONbits.ACKDT = NACK;
        
        I2C2CONbits.ACKEN = 1;    // send the contents of ACKDT
        state = kFinishingAck;
      }
 	    break;
 	  case kFinishingAck:
 	    if (!I2C2CONbits.ACKEN) {   // wait to finish acknowledging the slave
   	    if (remainingRxBytes) {
     	    I2C2CONbits.RCEN = 1;   // re-enable reception
     	    state = kReading;
     	  } else {
   	      I2C2CONbits.PEN = 1;    // start the stop event
   	      state = kStopping;
   	    }
   	  }
 	    break;
 	  case kStopping:
 	    // wait for the confirmation of hardware clearing the stop bit
 	    if (!I2C2CONbits.PEN) {
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
void __attribute__((__interrupt__, auto_psv)) _SI2C2Interrupt(void) {
  // do something
}
*/
/*---------------------------Helper Function Definitions----------------------*/
static void ConfigureBaudRate(kTWIBaudRate baudRate) {
  // see Table 16-1 of datasheet
	switch (baudRate) {
    case kTWIBaudRate100kHz: I2C2BRG = 157; break;
    case kTWIBaudRate400kHz: I2C2BRG = 37;  break;
    case kTWIBaudRate1MHz:   I2C2BRG = 13;  break;
  }
}
