#include "Testing.h"
#include "ESC.h"

void motor_temperature_test(void)
{
  static unsigned int ten_ms_counter = 0;
  static unsigned int second_counter_local = 0;
  static unsigned int total_seconds = 180;

  ClrWdt();


  //turn off speed input interrupt
  //_IC1IE = 0;
  
  ten_ms_counter++;
  if(ten_ms_counter >= 100)
  {
    ten_ms_counter = 0;
    second_counter_local++;
  }  

  if(second_counter_local < 10)
  //if(second_counter_local < 0)
  {
    ESC_set_speed(0);
  }
  else if(second_counter_local == 10)
  {
    ESC_StartMotor(200);
  }
  else if(second_counter_local < (total_seconds+10))
  {
    ESC_set_speed(400);
  }
  else
  {
    ESC_set_speed(0);
    while(1)
      ClrWdt();
  }


}
