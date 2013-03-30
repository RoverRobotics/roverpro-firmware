#include "home_office.h"
#include "charging.h"
#include "./core/ConfigurationBits.h"
#include "motor_control.h"


static void InitPins(void);
static void start_up(void);






void main(void)
{
  InitPins();
  start_up();
  ESC_Init();

  motor_control_test_function();

  while(1)
  {
    ClrWdt();

  }

}


static void InitPins(void)
{
  TRISB = 0xffff;
  TRISC = 0xffff;
  TRISD = 0xffff;
  TRISE = 0xffff;
  TRISF = 0xffff;

 


  V5_ON(0);
  SYS_BUS_ON(0);
  BQ24745_ON(0);

  V5_EN(1);
  SYS_BUS_EN(1);
  BQ24745_EN(1); 


}

static void start_up(void)
{

  unsigned int i = 0;

  //if battery is too low and not on the dock, this will shut down the robot
  for(i=0;i<20;i++)
  {
    battery_FSM();
  }

}

