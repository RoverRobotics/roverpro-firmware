/*==============================================================================
File: InternalCharger.c

Description: N/A

Notes:
  - N/A
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#define TEST_INTERNALCHARGER
/*---------------------------Dependencies-------------------------------------*/
#include "./core/StandardHeader.h"
#include "./core/Timers.h"          // for timers
#include "./core/TWISlave.h"

/*---------------------------Macros-------------------------------------------*/
// indicator(s)
#define HEARTBEAT_EN(a)       (_TRISE5 = !(a))
#define HEARTBEAT_PIN         _RE5

// timer(s)
#define _10ms                 10
#define _100ms                100
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        (5*_100ms)

#define MY_SLAVE_ADDRESS      0x0C

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitCharger(void);
static void InitPins(void);
static void ProcessChargerIO(void);
static uint8_t MyI2C1Response(const uint8_t command, 
                              const uint8_t response_index);

/*---------------------------Module Variables---------------------------------*/

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_INTERNALCHARGER
#include "./core/ConfigurationBits.h"
int main(void) {
  InitCharger();
  
  while (1) {
    ProcessChargerIO();
  }
  
  return 0;
}
#endif

/*---------------------------Helper Function Definitions----------------------*/
static void InitCharger(void) {
  InitPins();
  
  // initialize any dependent modules
  I2C1_UserProtocol = MyI2C1Response;
  TWISlave_Init(kTWISlave01, MY_SLAVE_ADDRESS, I2C);
  TMRS_Init();

 	// prime any timers that require it
  TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
}


static void InitPins(void) {
  // configure and initialize indicator(s)
  HEARTBEAT_EN(1); HEARTBEAT_PIN = 0;
}


static void ProcessChargerIO(void) {
  // toggle a pin to indicate normal operation
  if (TMRS_IsTimerExpired(HEARTBEAT_TIMER)) {
    TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
    HEARTBEAT_PIN ^= 1;
  }
}


/*
Description: Maps commands to internal variables.  This function
  essentially implements the I2C-based protocol between the devices.
Parameters:
  uint8_t command,        the subaddress of whose contents are requested
  uint8_t response_index, which byte (0th or 1st) of the response
*/
static uint8_t MyI2C1Response(const uint8_t command, 
                              const uint8_t response_index) {
  #define CMD_CHARGER_ALIVE     0xCA  
  #define ERROR_RESPONSE        0xFF
  uint8_t response[2] = {0xDA, 0x00}; // 'da' = yes in Russian
  
  switch (command) {
    case CMD_CHARGER_ALIVE: return response[0];
    default: return ERROR_RESPONSE;
  }
}
