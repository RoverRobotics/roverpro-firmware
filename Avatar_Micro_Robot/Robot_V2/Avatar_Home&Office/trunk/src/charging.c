#include "charging.h"
#include "home_office.h"
#include "./core/StandardHeader.h"

//12V:
//12 * (2k/12k) * 1023/3.3 = 620
#define MIN_LOW_BATT_ADC_COUNTS 620  
//14V 
#define MIN_LOW_BATT_CHARGING_ADC_COUNTS  723

static unsigned int return_battery_voltage(void);
static unsigned char safe_to_charge(void);


void battery_FSM(void)
{
  typedef enum {
    sLowBatteryOnCharger = 0,
    sFullBatteryOnCharger,
    sOffCharger,
  } sBatteryState;

  unsigned int low_battery_counter = 0;

  static sBatteryState state = sOffCharger;

  switch(state)
  {
    case sLowBatteryOnCharger:

      if(BQ24745_ACOK()==0)
      {
        state = sOffCharger;
      }

      if(return_battery_voltage() > MIN_LOW_BATT_ADC_COUNTS)
      {
        SYS_BUS_ON(1);
        V5_ON(1);
        state = sFullBatteryOnCharger;
      }

      if(safe_to_charge())
      {
        BQ24745_ON(1);
        //charge current and stuff
      }

    break;

    case sFullBatteryOnCharger:
      if(BQ24745_ACOK()==0)
      {
        state = sOffCharger;
      }
      
    break;
    case sOffCharger:

      if(BQ24745_ACOK())
      {

        state = sLowBatteryOnCharger;
      }


      if(return_battery_voltage() < MIN_LOW_BATT_ADC_COUNTS)
      {

        low_battery_counter++;
        if(low_battery_counter > 5)
        {
          SYS_BUS_ON(0);
        }
      }
      else
      {
        low_battery_counter = 0;
      }

    break;


  }




}


static unsigned int return_battery_voltage(void)
{

  return 1000;
}

static unsigned char safe_to_charge(void)
{



  return 1;
}
