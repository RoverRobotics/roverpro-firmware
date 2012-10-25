/*==============================================================================
File: TWISlave.c
==============================================================================*/
//#define TEST_TWI_SLAVE
/*---------------------------Dependencies-------------------------------------*/
#include "./TWISlave.h"

// TODO: is there a cleaner way of doing this?
uint8_t DummyFn(const uint8_t command, const uint8_t response_index);
extern uint8_t (*I2C1_UserProtocol)(const uint8_t command, 
                                    const uint8_t response_index) = DummyFn;
extern uint8_t (*I2C2_UserProtocol)(const uint8_t command, 
                                    const uint8_t response_index) = DummyFn;
extern uint8_t (*I2C3_UserProtocol)(const uint8_t command, 
                                    const uint8_t response_index) = DummyFn;     
                                    
/*---------------------------Macros-------------------------------------------*/
#define N_I2C_MODULES             3 // the maximum number of I2C modules
                                    // available on this PIC

/*---------------------------Type Definitions---------------------------------*/
// states in which the slave can be
typedef enum {
  kWaiting = 0,
  kReceiving,
  kTransmitting
} kSlaveState;

/*---------------------------Helper Function Prototypes-----------------------*/


/*---------------------------Module Variables---------------------------------*/
static uint8_t rx_buffer[2] = {0};
static volatile uint8_t n_remaining_bytes = 0;
static volatile kSlaveState slave_state = kWaiting;
  
/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_TWI_SLAVE
// TODO: RE_WRITE UNIT TEST NOW THAT PUBLIC INTERFACE HAS CHANGED!!!!
#include "./ConfigurationBits.h"

int main(void) {
  
  return 0;
}
#endif
/*---------------------------Public Function Definitions----------------------*/
uint8_t DummyFn(const uint8_t command, const uint8_t response_index) {
  return 0;
}

void TWISlave_Init(const kTWISlaveModule module,
              const uint8_t slave_address,
              const bool is_SMBus) {
  switch (module) {
    case kTWISlave01:
      I2C1CONbits.I2CEN = 0;	// disable the I2C module while we configure it
      I2C1CONbits.SMEN = is_SMBus; // select I2C or SMBus thresholds
      I2C1CONbits.STREN = 0;	// DISABLE receive clock stretching
      I2C1CONbits.GCEN = 0;	  // DISABLE an interrupt to fire on general calls
      I2C1ADD = slave_address;// configure the slave address
      _SI2C1IF = 0;						// begin with the interrupt cleared
      _SI2C1IE = 1;						// enable I2C interrupts for slave-related events
      I2C1CONbits.I2CEN = 1;	// enable the I2C module
	    break;
    case kTWISlave02:
      I2C2CONbits.I2CEN = 0;
      I2C2CONbits.SMEN = is_SMBus;
      I2C2CONbits.STREN = 0;
      I2C2CONbits.GCEN = 0;
      I2C2ADD = slave_address;
      _SI2C2IF = 0;
      _SI2C2IE = 1;
      I2C2CONbits.I2CEN = 1;
      break;
    case kTWISlave03:
      I2C3CONbits.I2CEN = 0;
      I2C3CONbits.SMEN = is_SMBus;
      I2C3CONbits.STREN = 0;
      I2C3CONbits.GCEN = 0;
      I2C3ADD = slave_address;
      _SI2C3IF = 0;
      _SI2C3IE = 1;
      I2C3CONbits.I2CEN = 1;
	    break;
  }
}


void TWISlave_Reset(const kTWISlaveModule module) {
  uint8_t temp;
  switch (module) {
    case kTWISlave01: 
      I2C1CONbits.I2CEN = 0;	// turn off the I2C module
      _SI2C1IF = 0;           // ensure the interrupt is cleared
      I2C1CONbits.SCLREL = 1; // ensure we are not holding the clock
      temp = I2C1RCV;         // read the rx buffer to clear an rx overrun error
      I2C1STATbits.I2COV = 0; // clear a potential rx overrun error
      I2C1STATbits.IWCOL = 0; // clear a potential Write-Collision error
      I2C1CONbits.I2CEN = 1;  // turn back on the I2C module
      break;
    case kTWISlave02:
      I2C2CONbits.I2CEN = 0;
      _SI2C2IF = 0;
      I2C2CONbits.SCLREL = 1;
      temp = I2C2RCV;
      I2C2STATbits.I2COV = 0;
      I2C2STATbits.IWCOL = 0;
      I2C2CONbits.I2CEN = 1;
      break;
    case kTWISlave03:
      I2C3CONbits.I2CEN = 0;
      _SI2C3IF = 0;
      I2C3CONbits.SCLREL = 1;
      temp = I2C3RCV;
      I2C3STATbits.I2COV = 0;
      I2C3STATbits.IWCOL = 0;
      I2C3CONbits.I2CEN = 1;
      break;
  }
  
  slave_state = kWaiting;
}

/*---------------------------Interrupt Service Routines-----------------------*/
// Notes:
//  - assumes write-word, read-word
//  - assumes this device is ONLY read from (e.g. a sensor)
void __attribute__((__interrupt__, auto_psv)) _SI2C1Interrupt(void) {
  _SI2C1IF = 0;               // clear the source of the interrupt
  /*
  // basic test
  if (I2C1STATbits.RBF) {
    rx_buffer[0] = I2C1RCV;
  }
  
  if (I2C1STATbits.R_W) {
    I2C1TRN = 0x33;
    I2C1CONbits.SCLREL = 1;   // release control of the clock
  }
  */
  switch (slave_state) {
    case kWaiting:
      // if reception is complete AND what we have received is the device address
      if (I2C1STATbits.RBF && !I2C1STATbits.D_A) {
        rx_buffer[0] = I2C1RCV;
        slave_state = kReceiving;
      } else {      
        // otherwise this is unexpected, so reset
        TWISlave_Reset(kTWISlave01);
      }
      break;
    case kReceiving:
      // if reception is complete AND what we've received is data (the command)
      if (I2C1STATbits.RBF && I2C1STATbits.D_A) {
        rx_buffer[1] = I2C1RCV;
        
        // assume we always respond after this
        n_remaining_bytes = 2;
        I2C1TRN = I2C1_UserProtocol(rx_buffer[1], 0);
        n_remaining_bytes--;
        slave_state = kTransmitting;
      } else {
        TWISlave_Reset(kTWISlave01);
      }
      break;
    case kTransmitting:
      // if transmition is complete
      if (!I2C1STATbits.TBF) {
        if (!n_remaining_bytes) {
          I2C1CONbits.SCLREL = 1; // release control of the clock
          slave_state = kWaiting;
        }
        
        I2C1CONbits.SCLREL = 1;   // release control of the clock
        I2C1TRN = I2C1_UserProtocol(rx_buffer[1], 1);
        n_remaining_bytes--;
      } else {
        TWISlave_Reset(kTWISlave01);
      }
      break;
  }
}

void __attribute__((__interrupt__, auto_psv)) _SI2C2Interrupt(void) {
  _SI2C2IF = 0;
  // TODO: port in same basic state machine from I2C1 interrupt
}


void __attribute__((__interrupt__, auto_psv)) _SI2C3Interrupt(void) {
  _SI2C3IF = 0;
   // TODO: port in same basic state machine from I2C1 interrupt
}

