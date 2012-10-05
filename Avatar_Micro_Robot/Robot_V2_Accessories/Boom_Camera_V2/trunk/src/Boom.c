/*******************************************************************************
File: Boom.c

Description: Overarching file encompassing the application-level logic of the 
  boom-camera module.
*******************************************************************************/
#define TEST_BOOM
/*---------------------------Dependencies-------------------------------------*/
#include "./core/StandardHeader.h"
#include "./core/Timers.h"
#include "./USB/RXUSBDevice.h"      // for USB firmware/software-shared 
                                    // registers
#include "./NM33.h"                 // for interface to wide-angle camera

/*---------------------------Macros-------------------------------------------*/
#define BOOM_PRODUCT_ID           0x0012  // RoboteX's boom camera product ID

//---power management
#define ENABLE_VBAT(a)            (_TRISB3 = !(a))
#define TURN_VBAT(a)              (_RB3 = (a))
#define ENABLE_FAN(a)             (_TRISD0 = !(a))	// NOTE: 5V regulator attached to fan pins for current hack!!!
#define TURN_FAN(a)								(_RD0 = (a))

//---indicators
#define ENABLE_HEARTBEAT(a)       (_TRISE5 = (a))
#define HEARTBEAT_PIN             (_RE5)

//---timers
#define _100ms										100
#define HEARTBEAT_TIMER           0
#define HEARTBEAT_TIME            500
#define CAM_TX_TIMER              1
#define CAM_TX_TIME               (_100ms)
#define USB_INIT_TIMER            2
#define USB_INIT_TIME             (_100ms)

//---NM33 pin assignments
#define MY_TX_PIN                 8
#define MY_RX_PIN                 9

/*---------------------------Type Definitions---------------------------------*/
typedef enum {
  kInitializing = 0,
  kViewing,
} BoomState;

/*---------------------------Helper Function Prototypes-----------------------*/
void InitBoom(void);
void ProcessBoomIO(void);
static void InitPins(void);
static void DeinitPins(void);

/*---------------------------Module Variables---------------------------------*/
volatile BoomState state = kViewing;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_BOOM
#include "./core/ConfigurationBits.h"

int main(void) {
  InitBoom();
  
  //SWDTEN = 1;
  while (1) {
    ProcessBoomIO();
  	//ClrWdt(); // clear the software WDT
  }
  
  return 0;
}

#endif
/*---------------------------Public Function Definitions----------------------*/
void InitBoom(void) {
	InitPins();
	
  RXUSBDevice_Init(BOOM_PRODUCT_ID);
  
  NM33_Init(MY_TX_PIN, MY_RX_PIN);
  
  TMRS_Init();
  TMRS_StartTimer(CAM_TX_TIMER, CAM_TX_TIME);
  TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
  TMRS_StartTimer(USB_INIT_TIMER, USB_INIT_TIME);
}


void ProcessBoomIO(void) {
  switch (state) {
    case kInitializing:
      // allow USB communication to be established before checking registers
      // to see if it should power down
      if (TMRS_IsTimerExpired(USB_INIT_TIMER)) state = kViewing;
      break;
    case kViewing:
      /*
      // power down if we lose connection or if software desires it
      if (REG_BOOM_POWER_DOWN) {
        TURN_FAN(OFF);
        TURN_VBAT(OFF);
        while (1) {};
      }
      */
      
      // toggle a pin to indicate normal operation
      if (TMRS_IsTimerExpired(HEARTBEAT_TIMER)) {
        TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
        HEARTBEAT_PIN ^= 1;
      }
      
      // transmit the latest desired position to the camera
      if (TMRS_IsTimerExpired(CAM_TX_TIMER)) {
        TMRS_StartTimer(CAM_TX_TIMER, CAM_TX_TIME);
        static uint16_t pan = 0;
        static uint8_t tilt = 90;
        static uint8_t zoom = 50;
        
        pan += 1;
        if (kNM33LimitMaxPan < pan) pan = kNM33LimitMinPan;
        
        //tilt += 10;
        //if (kNM33LimitMaxTilt < tilt) tilt = kNM33LimitMinTilt;
        
        //zoom += 10;
        //if (kNM33LimitMaxZoom < zoom) zoom = kNM33LimitMinZoom;
        
        NM33_set_location(pan, tilt, zoom);
      }
  
      break;
    default:
      ENABLE_HEARTBEAT(0);  // indicate an error
      break;
  }
  
  RXUSBDevice_ProcessMessage();
}

/*---------------------------Helper Function Definitions----------------------*/
static void InitPins(void) {
	//DeinitPins();
  ENABLE_VBAT(YES); TURN_VBAT(ON);
  ENABLE_FAN(YES); TURN_FAN(ON);
  ENABLE_HEARTBEAT(YES); HEARTBEAT_PIN = 0;
}


static void DeinitPins(void) {
  // return any resources consumed by dependent modules
	TMRS_Deinit();
//	NM33_Deinit();
	
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
}
