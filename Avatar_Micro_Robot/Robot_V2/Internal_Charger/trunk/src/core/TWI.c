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

#define N_I2C_MODULES             3 // the maximum number of I2C modules
                                    // available on this PIC

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
static void ConfigureBaudRate(const kTWIModule module, kTWIBaudRate baud_rate);

/*---------------------------Module Variables---------------------------------*/
static volatile uint8_t new_data_flags[N_I2C_MODULES] = {0};
static volatile uint8_t error_flags[N_I2C_MODULES] = {0};
static volatile uint8_t slave_addresses[N_I2C_MODULES] = {0};
static volatile uint8_t slave_subaddresses[N_I2C_MODULES] = {0};
static volatile uint8_t indications[N_I2C_MODULES] = {kIndicationRead};

// the number of bytes to receive from slave
static volatile uint8_t remaining_rx_bytes[N_I2C_MODULES] = {0};

static volatile uint8_t remaining_tx_bytes[N_I2C_MODULES] = {0};

// Rx buffer logical length
static volatile uint8_t logical_lengths[N_I2C_MODULES] = {0};

static volatile uint8_t buffers[N_I2C_MODULES][TWI_MAX_DATA_LENGTH] = {{0}};

static volatile MasterState states[N_I2C_MODULES] = {kWaiting};

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
              const kTWIBaudRate baud_rate, 
              const bool is_SMBus) {
  switch (module) {
    case kTWI01:
      I2C1CONbits.I2CEN = 0;	// disable the I2C module while we configure it
  	
    	// Note: the state of the port I/O pins are overridden when I2C is enabled
      TWI_Refresh(kTWI01);
    	ConfigureBaudRate(kTWI01, baud_rate);
    	
    	I2C1CONbits.SMEN = is_SMBus; // configure I2C/SMBus thresholds
    	I2C1CONbits.STREN = 0;	// DISABLE receive clock stretching
    	I2C1CONbits.GCEN = 0;	  // DISABLE an interrupt to fire when a general call
    													// address is received
    	
    	_MI2C1IF = 0;						// begin with the interrupt cleared
    	_MI2C1IE = 1;						// enable I2C interrupts for master-related events
    	I2C1CONbits.I2CEN = 1;	// enable the I2C module
	    break;
    case kTWI02:
      I2C2CONbits.I2CEN = 0;
      TWI_Refresh(kTWI02);
	    ConfigureBaudRate(kTWI02, baud_rate);
	    
	    I2C2CONbits.SMEN = is_SMBus;
	    I2C2CONbits.STREN = 0;
	    I2C2CONbits.GCEN = 0;
	
	    _MI2C2IF = 0;
	    _MI2C2IE = 1;
	    I2C2CONbits.I2CEN = 1;
      break;
    case kTWI03:
      I2C3CONbits.I2CEN = 0;
      TWI_Refresh(kTWI03);
	    ConfigureBaudRate(kTWI03, baud_rate);
	    
	    I2C3CONbits.SMEN = is_SMBus;
	    I2C3CONbits.STREN = 0;
	    I2C3CONbits.GCEN = 0;
	
	    _MI2C3IF = 0;
	    _MI2C3IE = 1;
	    I2C3CONbits.I2CEN = 1;
	    break;
  }
}


void TWI_RequestData(const kTWIModule module, const TWIDevice* device) {
  switch (module) {
    case kTWI01:
      // populate the device from which to request data
      slave_addresses[kTWI01] = device->address;
      slave_subaddresses[kTWI01] = device->subaddress;
      
      // prepare to communicate
      indications[kTWI01] = kIndicationRead;
      remaining_rx_bytes[kTWI01] = device->n_data_bytes;
      logical_lengths[kTWI01] = device->n_data_bytes;
      states[kTWI01] = kStarting;
            
      // start the start event to initiate the conversation
      I2C1CONbits.SEN = 1;
      break;
    case kTWI02:
      slave_addresses[kTWI02] = device->address;
      slave_subaddresses[kTWI02] = device->subaddress;
      indications[kTWI02] = kIndicationRead;
      remaining_rx_bytes[kTWI02] = device->n_data_bytes;
      logical_lengths[kTWI02] = device->n_data_bytes;
      states[kTWI02] = kStarting;
      I2C2CONbits.SEN = 1;
      break;
    case kTWI03:
      slave_addresses[kTWI03] = device->address;
      slave_subaddresses[kTWI03] = device->subaddress;
      indications[kTWI03] = kIndicationRead;
      remaining_rx_bytes[kTWI03] = device->n_data_bytes;
      logical_lengths[kTWI03] = device->n_data_bytes;
      states[kTWI03] = kStarting;
      I2C3CONbits.SEN = 1;
      break;
  }
}


inline bool TWI_IsBusIdle(const kTWIModule module) {
  // see p.53 of Section 24 of PIC24F Family Reference Manual
  switch (module) {
    case kTWI01:
      return (!I2C1CONbits.SEN && !I2C1CONbits.RSEN && !I2C1CONbits.PEN && 
              !I2C1CONbits.RCEN && !I2C1CONbits.ACKEN && !I2C1STATbits.TRSTAT);
    case kTWI02:
      return (!I2C2CONbits.SEN && !I2C2CONbits.RSEN && !I2C2CONbits.PEN && 
              !I2C2CONbits.RCEN && !I2C2CONbits.ACKEN && !I2C2STATbits.TRSTAT);  
    case kTWI03:
      return (!I2C3CONbits.SEN && !I2C3CONbits.RSEN && !I2C3CONbits.PEN && 
              !I2C3CONbits.RCEN && !I2C3CONbits.ACKEN && !I2C3STATbits.TRSTAT);
    default:
      return NO;
  }
}


inline bool TWI_IsNewDataAvailable(const kTWIModule module) {
  return new_data_flags[module];
}


void TWI_GetData(const kTWIModule module, const TWIDevice* device) {
  // copy in as much of the buffer as is relevant
  uint8_t i;
  for (i = 0; i < logical_lengths[module]; i++) {
    device->data[i] = buffers[module][i];
  }
  
  new_data_flags[module] = 0;
  
  // clear out irrelevant data?
  //for (i = logicalLength: i < TWI_MAX_DATA_LENGTH; i++) buffer[i] = 0;
}


void TWI_WriteData(const kTWIModule module,
                   const TWIDevice* device,
                   const uint8_t data[]) {
  // populate the firmware variables
  slave_addresses[module] = device->address;
  slave_subaddresses[module] = device->subaddress;
  indications[module] = kIndicationWrite;
  remaining_tx_bytes[module] = sizeof(data) / sizeof(uint8_t);
  logical_lengths[module] = remaining_tx_bytes[module];
  
  //---copy in the data to write
  uint8_t i;
  for (i = 0; i < remaining_tx_bytes[module]; i++) {
    buffers[module][i] = data[i];
  }
  
  states[module] = kStarting;
  
  // start the start event  
  switch (module) {
    case kTWI01: I2C1CONbits.SEN = 1; break;
    case kTWI02: I2C2CONbits.SEN = 1; break;
    case kTWI03: I2C3CONbits.SEN = 1; break;
  }
}


bool TWI_ErrorHasOccurred(const kTWIModule module) {
  // check whether there has been a collision or receive overflow
  switch (module) {
    case kTWI01:
      return (I2C1STATbits.BCL || I2C1STATbits.I2COV || 
              I2C1STATbits.IWCOL || error_flags[kTWI01]);
    case kTWI02:
      return (I2C2STATbits.BCL || I2C2STATbits.I2COV ||
              I2C2STATbits.IWCOL || error_flags[kTWI02]);
    case kTWI03:
      return (I2C3STATbits.BCL || I2C3STATbits.I2COV || 
              I2C3STATbits.IWCOL || error_flags[kTWI03]);
    default:
      return YES;
  }
}


void TWI_Refresh(const kTWIModule module) {
  // clear any firmware variables
  new_data_flags[module] = 0;
  error_flags[module] = 0;
  slave_addresses[module] = 0;
  slave_subaddresses[module] = 0;
  indications[module] = 0;
  remaining_rx_bytes[module] = 0;
  remaining_tx_bytes[module] = 0;
  logical_lengths[module] = 0;
  uint8_t i;
  for (i = 0; i < TWI_MAX_DATA_LENGTH; i++) buffers[module][i] = 0;
  states[module] = kWaiting;
  
  // clear any hardware errors, status flags and buffers
  int8_t temp;
  switch (module) {
    case kTWI01:
      I2C1STAT = 0x0000;
      I2C1CONbits.RCEN = 0;		// clear 'currently-receiving' flag
      temp = I2C1RCV;		      // read to ensure the Rx buffer begins cleared
      break;
    case kTWI02:
      I2C2STAT = 0x0000;
      I2C2CONbits.RCEN = 0;
      temp = I2C2RCV;
      break;
    case kTWI03:
      I2C3STAT = 0x0000;
      I2C3CONbits.RCEN = 0;
      temp = I2C3RCV;
      break;
  }
  
  /*
  // reset any pins?
  switch (module) {
    case kTWI01:
      I2C1CONbits.I2CEN = 0;
      _TRISD9 = 0; _LATD9 = 0;
      _TRISD10 = 0; _LATD10 = 0;
      I2C1CONbits.I2CEN = 1;
      break;
    case kTWI02:
      I2C2CONbits.I2CEN = 0;
      _TRISF4 = 0; _LATF4 = 0;
      _TRISF5 = 0; _LATF5 = 0;
      I2C2CONbits.I2CEN = 1;
      break;
    case kTWI03:
      I2C3CONbits.I2CEN = 0;
      _TRISE6 = 0; _LATE6 = 0;
      _TRISE7 = 0; _LATE7 = 0;
      I2C3CONbits.I2CEN = 1;    
      break;
  }
  */
}


void TWI_Deinit(const kTWIModule module) {
  switch (module) {
    case kTWI01:
      I2C1CONbits.I2CEN = 0;  // turn off I2C and restore consumed pins
      TWI_Refresh(kTWI01);
  
      // restore any registers to their startup defaults
      I2C1CON = 0x0000; // Note: default actually leaves I2C on, but keeping off
      I2C1TRN = 0x00ff;
      I2C1BRG = 0x0000;
      I2C1STAT = 0x0000;
    case kTWI02:
      I2C2CONbits.I2CEN = 0;
      TWI_Refresh(kTWI02);
      
      I2C2CON = 0x0000;
      I2C2TRN = 0x00ff;
      I2C2BRG = 0x0000;
      I2C2STAT = 0x0000;
    case kTWI03:
      I2C3CONbits.I2CEN = 0;
      TWI_Refresh(kTWI03);
      
      I2C3CON = 0x0000;
      I2C3TRN = 0x00ff;
      I2C3BRG = 0x0000;
      I2C3STAT = 0x0000;
  }
}
  
/*---------------------------Interrupt Service Routines-----------------------*/
/*
Description: This ISR contains a state machine that constitues the core of this
  module.  It is written to be event-driven so that it can be ported to the
  main thread if needed.  It is also unfortunately long but meant to be kept
  lean.
*/
void __attribute__((__interrupt__, auto_psv)) _MI2C1Interrupt(void) {
  IFS1bits.MI2C1IF = 0;
  
  switch (states[kTWI01]) {
    case kWaiting:
      error_flags[kTWI01] = 1;    // should never get here in interrupt
      break;
    case kStarting:
 	    // wait for confirmation of the hardware clearing the start bit
 	    if (!I2C1CONbits.SEN) {
   	    // transmit the slave address along with the indication
   	    I2C1TRN = ((slave_addresses[kTWI01] << 1) | kIndicationWrite);
   	    states[kTWI01] = kSelectingDevice;
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kSelectingDevice:
 	    // wait for confirmation of the slave acknowledging
 	    if (I2C1STATbits.ACKSTAT == ACK) {
   	    // transmit the slave sub-address
   	    I2C1TRN = slave_subaddresses[kTWI01];
     	  states[kTWI01] = kSelectingRegister;
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kSelectingRegister:
 	    // wait for confirmation of the slave acknowledging
 	    if (I2C1STATbits.ACKSTAT == ACK) {
        if (indications[kTWI01] == kIndicationRead) {
          I2C1CONbits.PEN = 1;    // start a restart event. PEN, then SEN
          states[kTWI01] = kShortstopping;
   	    } else if (indications[kTWI01] == kIndicationWrite) {
   	      I2C1TRN = buffers[kTWI01][(logical_lengths[kTWI01] - 
   	                                 remaining_tx_bytes[kTWI01])];
   	      //remaining_tx_bytes[kTWI01]--;
   	      states[kTWI01] = kWriting;
 	      }
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kWriting:
 	    // wait for confirmation of the slave acknowledging
   	  if (I2C1STATbits.ACKSTAT == ACK) {  
     	  if (!remaining_tx_bytes[kTWI01]) {
     	    I2C1CONbits.PEN = 1;    // start the stop event
     	    states[kTWI01] = kStopping;
     	    return;
     	  }
   	    I2C1TRN = buffers[kTWI01][(logical_lengths[kTWI01] - 
   	                              remaining_tx_bytes[kTWI01])];
   	    remaining_tx_bytes[kTWI01]--;
   	  } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kShortstopping:
 	    if (!I2C1CONbits.PEN) {
   	    I2C1CONbits.SEN = 1;
   	    states[kTWI01] = kRestarting;
   	  }
 	    break;
 	  case kRestarting:
 	    if (!I2C1CONbits.SEN) {
   	    I2C1TRN = (slave_addresses[kTWI01] << 1) | kIndicationRead;
   	    states[kTWI01] = kReselectingDevice;
   	  } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kReselectingDevice:
 	    if (I2C1STATbits.ACKSTAT == ACK) {
   	    I2C1CONbits.RCEN = 1;     // enable reception
 	      states[kTWI01] = kReading;
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kReading:
      // if the receive buffer is full
      if (I2C1STATbits.RBF) {
        // store the buffer contents first-in-last (first byte arrives last)
        buffers[kTWI01][--(remaining_rx_bytes[kTWI01])] = I2C1RCV;
        
        // acknowledge the slave
        if (remaining_rx_bytes[kTWI01]) I2C1CONbits.ACKDT = ACK;
        else I2C1CONbits.ACKDT = NACK;
        
        I2C1CONbits.ACKEN = 1;    // send the contents of ACKDT
        states[kTWI01] = kFinishingAck;
      } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kFinishingAck:
 	    if (!I2C1CONbits.ACKEN) {   // wait to finish acknowledging the slave
   	    if (remaining_rx_bytes[kTWI01]) {
     	    I2C1CONbits.RCEN = 1;   // re-enable reception
     	    states[kTWI01] = kReading;
     	  } else {
   	      I2C1CONbits.PEN = 1;    // start the stop event
   	      states[kTWI01] = kStopping;
   	    }
   	  } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kStopping:
 	    // wait for the confirmation of hardware clearing the stop bit
 	    if (!I2C1CONbits.PEN) {
   	    if (indications[kTWI01] == kIndicationRead) new_data_flags[kTWI01] = 1;
   	    states[kTWI01] = kWaiting;
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  default:
 	    error_flags[kTWI01] = 1;  // indicate an error
 	    break;
  }
}


void __attribute__((__interrupt__, auto_psv)) _MI2C2Interrupt(void) {
  IFS3bits.MI2C2IF = 0;
  
  switch (states[kTWI02]) {
    case kWaiting:
      error_flags[kTWI02] = 1;
      break;
    case kStarting:
 	    if (!I2C2CONbits.SEN) {
   	    I2C2TRN = ((slave_addresses[kTWI02] << 1) | kIndicationWrite);
   	    states[kTWI02] = kSelectingDevice;
 	    } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kSelectingDevice:
 	    if (I2C2STATbits.ACKSTAT == ACK) {
   	    I2C2TRN = slave_subaddresses[kTWI02];
     	  states[kTWI02] = kSelectingRegister;
 	    } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kSelectingRegister:
 	    if (I2C2STATbits.ACKSTAT == ACK) {
        if (indications[kTWI02] == kIndicationRead) {
          I2C2CONbits.PEN = 1;
          states[kTWI02] = kShortstopping;
   	    } else if (indications[kTWI02] == kIndicationWrite) {
   	      I2C2TRN = buffers[kTWI02][(logical_lengths[kTWI02] - 
   	                                 remaining_tx_bytes[kTWI02])];
   	      //remaining_tx_bytes[kTWI02]--;
   	      states[kTWI02] = kWriting;
 	      }
 	    } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kWriting:
   	  if (I2C2STATbits.ACKSTAT == ACK) {
     	  if (!(remaining_tx_bytes[kTWI02])) {
     	    I2C2CONbits.PEN = 1;
     	    states[kTWI02] = kStopping;
     	    return;
     	  }
   	    I2C2TRN = buffers[kTWI02][logical_lengths[kTWI02] - 
   	                              remaining_tx_bytes[kTWI02]];
   	    remaining_tx_bytes[kTWI02]--;
   	  } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kShortstopping:
 	    if (!I2C2CONbits.PEN) {
   	    I2C2CONbits.SEN = 1;
   	    states[kTWI02] = kRestarting;
   	  } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kRestarting:
 	    if (!I2C2CONbits.SEN) {
   	    I2C2TRN = (slave_addresses[kTWI02] << 1) | kIndicationRead;
   	    states[kTWI02] = kReselectingDevice;
   	  } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kReselectingDevice:
 	    if (I2C2STATbits.ACKSTAT == ACK) {
   	    I2C2CONbits.RCEN = 1;
 	      states[kTWI02] = kReading;
 	    } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kReading:
      if (I2C2STATbits.RBF) {
        buffers[kTWI02][--(remaining_rx_bytes[kTWI02])] = I2C2RCV;

        if (remaining_rx_bytes[kTWI02]) I2C2CONbits.ACKDT = ACK;
        else I2C2CONbits.ACKDT = NACK;
        
        I2C2CONbits.ACKEN = 1;
        states[kTWI02] = kFinishingAck;
      } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kFinishingAck:
 	    if (!I2C2CONbits.ACKEN) {
   	    if (remaining_rx_bytes[kTWI02]) {
     	    I2C2CONbits.RCEN = 1;
     	    states[kTWI02] = kReading;
     	  } else {
   	      I2C2CONbits.PEN = 1;
   	      states[kTWI02] = kStopping;
   	    }
   	  } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  case kStopping:
 	    if (!I2C2CONbits.PEN) {
   	    if (indications[kTWI02] == kIndicationRead) new_data_flags[kTWI02] = 1;
   	    states[kTWI02] = kWaiting;
 	    } else {
 	      TWI_Refresh(kTWI02);
 	    }
 	    break;
 	  default:
 	    error_flags[kTWI02] = 1;
 	    break;
  }
}



void __attribute__((__interrupt__, auto_psv)) _MI2C3Interrupt(void) {
  IFS5bits.MI2C3IF = 0;
  
  switch (states[kTWI03]) {
    case kWaiting:
      error_flags[kTWI03] = 1;
      break;
    case kStarting:
 	    if (!I2C3CONbits.SEN) {
   	    I2C3TRN = ((slave_addresses[kTWI03] << 1) | kIndicationWrite);
   	    states[kTWI03] = kSelectingDevice;
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kSelectingDevice:
 	    if (I2C3STATbits.ACKSTAT == ACK) {
   	    I2C3TRN = slave_subaddresses[kTWI03];
     	  states[kTWI03] = kSelectingRegister;
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kSelectingRegister:
 	    if (I2C3STATbits.ACKSTAT == ACK) {
        if (indications[kTWI03] == kIndicationRead) {
          I2C3CONbits.PEN = 1;
          states[kTWI03] = kShortstopping;
   	    } else if (indications[kTWI03] == kIndicationWrite) {
   	      I2C3TRN = buffers[kTWI03][(logical_lengths[kTWI03] - 
   	                                 remaining_tx_bytes[kTWI03])];
   	      //remaining_tx_bytes[kTWI03]--;
   	      states[kTWI03] = kWriting;
 	      }
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kWriting:
   	  if (I2C3STATbits.ACKSTAT == ACK) {  
     	  if (!remaining_tx_bytes[kTWI03]) {
     	    I2C3CONbits.PEN = 1;
     	    states[kTWI03] = kStopping;
     	    return;
     	  }
   	    I2C3TRN = buffers[kTWI03][(logical_lengths[kTWI03] - 
   	                              remaining_tx_bytes[kTWI03])];
   	    remaining_tx_bytes[kTWI03]--;
   	  } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kShortstopping:
 	    if (!I2C3CONbits.PEN) {
   	    I2C3CONbits.SEN = 1;
   	    states[kTWI03] = kRestarting;
   	  } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kRestarting:
 	    if (!I2C3CONbits.SEN) {
   	    I2C3TRN = (slave_addresses[kTWI03] << 1) | kIndicationRead;
   	    states[kTWI03] = kReselectingDevice;
   	  } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kReselectingDevice:
 	    if (I2C3STATbits.ACKSTAT == ACK) {
   	    I2C3CONbits.RCEN = 1;
 	      states[kTWI03] = kReading;
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kReading:
      if (I2C3STATbits.RBF) {
        buffers[kTWI03][--(remaining_rx_bytes[kTWI03])] = I2C3RCV;

        if (remaining_rx_bytes[kTWI03]) I2C3CONbits.ACKDT = ACK;
        else I2C3CONbits.ACKDT = NACK;
        
        I2C3CONbits.ACKEN = 1;
        states[kTWI03] = kFinishingAck;
      } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kFinishingAck:
 	    if (!I2C3CONbits.ACKEN) {
   	    if (remaining_rx_bytes[kTWI03]) {
     	    I2C3CONbits.RCEN = 1;
     	    states[kTWI03] = kReading;
     	  } else {
   	      I2C3CONbits.PEN = 1;
   	      states[kTWI03] = kStopping;
   	    }
   	  } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  case kStopping:
 	    if (!I2C3CONbits.PEN) {
   	    if (indications[kTWI03] == kIndicationRead) new_data_flags[kTWI03] = 1;
   	    states[kTWI03] = kWaiting;
 	    } else {
 	      TWI_Refresh(kTWI01);
 	    }
 	    break;
 	  default:
 	    error_flags[kTWI03] = 1;
 	    break;
  }
}


/*
void __attribute__((__interrupt__, auto_psv)) _SI2C2Interrupt(void) {
  // do something
}
*/
/*---------------------------Helper Function Definitions----------------------*/
static void ConfigureBaudRate(const kTWIModule module, kTWIBaudRate baud_rate) {
  // see Table 16-1 of datasheet
  switch (module) {
    case kTWI01:
      switch (baud_rate) {
        case kTWIBaudRate100kHz: I2C1BRG = 157; break;
        case kTWIBaudRate400kHz: I2C1BRG = 37;  break;
        case kTWIBaudRate1MHz:   I2C1BRG = 13;  break;
      }
      break;
    case kTWI02:
      switch (baud_rate) {
        case kTWIBaudRate100kHz: I2C2BRG = 157; break;
        case kTWIBaudRate400kHz: I2C2BRG = 37;  break;
        case kTWIBaudRate1MHz:   I2C2BRG = 13;  break;
      }
      break;
    case kTWI03:
      switch (baud_rate) {
        case kTWIBaudRate100kHz: I2C3BRG = 157; break;
        case kTWIBaudRate400kHz: I2C3BRG = 37;  break;
        case kTWIBaudRate1MHz:   I2C3BRG = 13;  break;
      }
      break;
  }
}
