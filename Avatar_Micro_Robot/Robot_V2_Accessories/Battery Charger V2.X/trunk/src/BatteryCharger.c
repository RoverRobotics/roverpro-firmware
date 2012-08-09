/*******************************************************************************
File: BatteryCharger.c

Description: Overarching file encompassing the application-level logic of the 
  battery charger (external to the robot).
*******************************************************************************/
#define TEST_CHARGER
/*---------------------------Dependencies-------------------------------------*/
#include "./core/StandardHeader.h"
#include "./core/Timers.h"
#include "./core/ADC.h"          // for analog sensing of current and voltage
#include "./core/TWI.h"
#include "./SMBusDeviceHeaders/BQ242745.h"

/*---------------------------Macros-------------------------------------------*/
// wiring macros
//---battery charger
#define ENABLE_5V(a)              (_TRISB12 = (a))
#define TURN_5V(a)                (_RB12 = (a))
#define ENABLE_CHARGE_A(a)        (_TRISB10 = (a))
#define TURN_CHARGE_A(a)          (_RB10 = (a))
#define ENABLE_CHARGE_B(a)        (_TRISB11 = (a))
#define TURN_CHARGE_B(a)          (_RB11 = (a))
#define ENABLE_CONNECT_BATTERY(a) (_TRISB9 = (a))
#define CONNECT_BATTERY(a)        (_RB9 = (a))
#define ENABLE_BQ242745_EN(a)     (_TRISB13 = (a))
#define TURN_BQ242745(a)          (_RB13 = (a))

//--analog sensor channels
#define I_SENSE_A_CH              7  
#define I_SENSE_B_CH              8
#define V_SENSE_A_CH              14
#define V_SENSE_B_CH              15

//---indicators
#define ENABLE_HEARTBEAT(a)       (_TRISE5 = (a))
#define HEARTBEAT_PIN             (_RE5)
#define ENABLE_RED_LED(a)         (_TRISB2 = (a))
#define TURN_RED_LED(a)           (_RB2 = (a))
#define ENABLE_GREEN_LED(a)       (_TRISB3 = (a))
#define TURN_GREEN_LED(a)         (_RB3 = (a))

// timers
#define HEARTBEAT_TIMER           0
#define HEARTBEAT_TIME            500
#define SMB1_TIMEOUT_TIMER        1
#define SMB1_TIMEOUT_TIME         10
#define SMB2_TIMEOUT_TIMER        2
#define SMB2_TIMEOUT_TIME         10
#define SMB3_TIMEOUT_TIMER        3
#define SMB3_TIMEOUT_TIME         10

/*---------------------------Type Definitions---------------------------------*/
typedef enum {
  kWaiting = 0,
  kCharging
} ChargerState;

/*---------------------------Helper Function Prototypes-----------------------*/
void InitBatteryCharger(void);
void ProcessBatteryChargerIO(void);

static void InitPins(void);
static void DeinitPins(void);
//static void InitBattery(void);
//static void UpdateSMBusSensors(void);
//static inline float DecodeCellData(TWIDevice *device);

/*---------------------------Module Variables---------------------------------*/
static unsigned char dummyData1[2] = {0};
static TWIDevice charger = {
  .address = BQ242745_DEVICE_ID,
  .subaddress = 0x00, // subaddress which contains the sensor data
  .numDataBytes = 2,  // number of data bytes the sensor will send us
  .data = dummyData1
};

volatile ChargerState state = kWaiting;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_CHARGER
#include "./core/ConfigurationBits.h"

int main(void) {

  InitBatteryCharger();
  
  //SWDTEN = 1;
  while (1) {
    ProcessBatteryChargerIO();
  	//ClrWdt(); // clear the software WDT
  }
  
  return 0;
}
#endif
/*---------------------------Public Function Definitions----------------------*/
void InitBatteryCharger(void) {
	InitPins();

  InitTimers();
  ADC_Init((1 << I_SENSE_A_CH) |
           (1 << I_SENSE_B_CH) |
           (1 << V_SENSE_A_CH) |
           (1 << V_SENSE_B_CH));
  TWI_Init(kBaudRate100kHz, YES);
  //InitBattery();
  
  StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
  StartTimer(SMB1_TIMEOUT_TIMER, SMB1_TIMEOUT_TIME);
  StartTimer(SMB2_TIMEOUT_TIMER, SMB2_TIMEOUT_TIME);
  StartTimer(SMB3_TIMEOUT_TIMER, SMB3_TIMEOUT_TIME);  
}


void ProcessBatteryChargerIO(void) {
  
  // toggle a pin to indicate normal operation
  if (IsTimerExpired(HEARTBEAT_TIMER)) {
    StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
    HEARTBEAT_PIN ^= 1;
  }  
  
  switch (state) {
    case kWaiting:
      break;
    case kCharging:
      break;
    default:
      ENABLE_HEARTBEAT(0);  // indicate an error
      break;
  }  
}


/*---------------------------Private Function Definitions---------------------*/
static inline float DecodeBQ242745Data(TWIDevice *device) {
  int placeholder = (device->data[0] << 8) | (device->data[1]);   
  return ((float)placeholder) * 16.0 / 4096.0;
}


static void InitPins(void) {
	DeinitPins();
  
  //---battery charger IC
  ENABLE_5V(YES); TURN_5V(ON);
  ENABLE_CHARGE_A(YES); TURN_CHARGE_A(ON);
  ENABLE_CHARGE_B(YES); TURN_CHARGE_B(ON);
  ENABLE_CONNECT_BATTERY(YES); CONNECT_BATTERY(YES);
  ENABLE_BQ242745_EN(YES); TURN_BQ242745(ON);

  //---indicators
  ENABLE_HEARTBEAT(YES); HEARTBEAT_PIN = 0;
  ENABLE_RED_LED(YES); TURN_RED_LED(OFF);
  ENABLE_GREEN_LED(YES); TURN_GREEN_LED(OFF);
}


static void DeinitPins(void) {
  // configure all I/O pins as digital outputs and ground them
  AD1CON1 = 0x0000; Nop(); 
	AD1CON2 = 0x0000; Nop(); 
	AD1CON3 = 0x0000; Nop(); 
  AD1PCFGL = 0xffff; Nop(); 
	TRISB = 0x0000; Nop(); PORTB = 0x0000;
	TRISC = 0x0000; Nop(); PORTC = 0x0000;
	TRISD = 0x0000; Nop(); PORTD = 0x0000;
	TRISE = 0x0000; Nop(); PORTE = 0x0000;
	TRISF = 0x0000; Nop(); PORTF = 0x0000;
	TRISG = 0x0000; Nop(); PORTG = 0x0000;
  
  // return any resources consumed by dependent modules
	DeinitTimers();
	ADC_Deinit();
	TWI_Deinit();
}
