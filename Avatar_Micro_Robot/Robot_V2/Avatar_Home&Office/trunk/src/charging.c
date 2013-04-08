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
    sLowBatteryOffCharger,
    sFullBatteryOffCharger,
  } sBatteryState;

  unsigned int low_battery_counter = 0;

  static sBatteryState state = sLowBatteryOffCharger;

  switch(state)
  {
    case sLowBatteryOnCharger:

      if(BQ24745_ACOK()==0)
      {
        state = sLowBatteryOffCharger;
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
        state = sLowBatteryOffCharger;
      }
      
    break;
    case sLowBatteryOffCharger:

      if(BQ24745_ACOK())
      {

        state = sLowBatteryOnCharger;
      }
      else if (return_battery_voltage() > MIN_LOW_BATT_ADC_COUNTS)
      {
        SYS_BUS_ON(1);
        V5_ON(1);
        state = sFullBatteryOffCharger;
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
    case sFullBatteryOffCharger:
      if(return_battery_voltage() < MIN_LOW_BATT_ADC_COUNTS)
      {
        state = sLowBatteryOffCharger;
      }
    break;


  }




}


static unsigned int return_battery_voltage(void)
{

  //6:1 voltage divider:
  //ADC*3.3/1023*6
  //ADC = Voltage*1023/3.3/6
  return ADC1BUF2;
  //return 1000;
}

static unsigned char safe_to_charge(void)
{



  return 1;
}
