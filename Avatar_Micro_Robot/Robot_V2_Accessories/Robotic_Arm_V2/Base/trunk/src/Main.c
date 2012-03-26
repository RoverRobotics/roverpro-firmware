/****************************************************************************
 File: Main.c

 Description: This is the overarching file encapsulating the Base PCB.
******************************************************************************/
#define TestMain

/*------------------------Dependencies---------------------------------------*/
#include <p24FJ256GB106.h>

#include "Base.h"
#include "ConfigurationBits.h"
//#include "Timers.h"
//#include "MasterSM.h"
//#include "CheckEvents.h"

/*---------------------------Helper Function Prototypes----------------------*/
static void InitMain(void);
static void ConfigurePins(void);

/*---------------------------Test Harness------------------------------------*/
#ifdef TestMain

int main(void) {
  InitMain();
  
  while (1) {
    RunMasterSM(CheckEvents());
  }
  
  return 0;
}  

#endif
/*---------------------------End Test Harness--------------------------------*/

/*---------------------------Private Functions-------------------------------*/
static void InitMain(void) {
  ConfigurePins();
  
  // initialize any dependent modules
	InitBase();

  // prime any timers that require it
}

static void ConfigurePins(void) {
  // configure and initialize any I/O pin(s)
  //TRISBbits.TRISB15 = 0; ANSBbits.ANSB15 = 0; DEBUG_0 = 0;
}


/*---------------------------End of File-------------------------------------*/