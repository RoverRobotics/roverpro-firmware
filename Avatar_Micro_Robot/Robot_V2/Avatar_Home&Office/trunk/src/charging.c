#include "charging.h"
#include "home_office.h"
#include "motor_control.h"
#include "./core/StandardHeader.h"

//voltage formula:
//V*(2k/12k)*1023/3.3
#define BATT_12V_CALC   620
#define BATT_14V_CALC   723
#define BATT_18V_CALC   930

#define BATT_12V_EXP    584
#define BATT_14V_EXP    646
#define BATT_18V_EXP    736


#define BATTERY_METER_0     650
#define BATTERY_METER_25    700
#define BATTERY_METER_50    725
#define BATTERY_METER_75    775
#define BATTERY_METER_100   800 

#define MIN_LOW_BATT_ADC_COUNTS             BATT_12V_CALC      
#define MIN_LOW_BATT_CHARGING_ADC_COUNTS    BATT_14V_CALC
#define BATTERY_FULLY_CHARGED_ADC_COUNTS    BATT_18V_CALC

#define _4_HOURS  1.44e6
#define _24_HOURS  8.44e6

//forg charger


#define BQ24745_I2C_ADD               0x09
#define BQ24745_INPUT_CURRENT_REG     0x3F
#define BQ24745_CHG_VOLTAGE_REG       0x15
#define BQ24745_CHG_CURRENT_REG       0x14
//#define BQ24745_INPUT_CURRENT         0x0400
#define BQ24745_INPUT_CURRENT         1536

//#define BQ24745_CHG_VOLTAGE           18432 //actually charges
//#define BQ24745_CHG_VOLTAGE           20000 //doesn't charge
//#define BQ24745_CHG_VOLTAGE           19968 //still doesn't charge
#define BQ24745_CHG_VOLTAGE           19200


static int i2c3_interrupt_state = 0;

static char charging_state = 0;
static char battery_meter = 0;






//Formula: R/(10k+R)*1023
//R found experimentally
//R @ 40C: 6.55k
//R @ 45C: 5.26k
//R @ 50C: 4.43k
#define THERMISTOR_40C      405
#define THERMISTOR_45C      353
#define THERMISTOR_50C      314

//disallow charging at 10C
//10C = 20.240k
//20.24/(10+20.24)*1023 = 684.7
#define MAX_THERM_ADC_COUNTS 685




static unsigned int return_battery_voltage(void);
static unsigned int return_max_charging_current(void);
static void set_ADC_values(void);
static void charge_battery(unsigned int charge_current);
static unsigned char check_delta_v(unsigned char reset);

static unsigned char did_charge_timeout(unsigned char reset);

static void handle_turnoff(void);


static void update_battery_meter(void);

static unsigned char battery_fully_charged = 0;

static unsigned int battery_voltage = 0;
static unsigned int B1_THERM = 0;
static unsigned int B2_THERM = 0;


//i2c variables
static unsigned char i2c3_register;
static unsigned char i2c3_tx_byte1;
static unsigned char i2c3_tx_byte2;
static unsigned int i2c3_slave_address;
static unsigned int i2c3_transmit_word_completed = 1;


void battery_FSM(void)
{
  typedef enum {
    sLowBatteryOnCharger = 0,
    sFullBatteryOnCharger,
    sLowBatteryOffCharger,
    sFullBatteryOffCharger,
    sTrickleCharging,
  } sBatteryState;

  static unsigned int low_battery_counter = 0;

  static sBatteryState state = sLowBatteryOffCharger;

  static unsigned long charging_restart_counter = 0;

  handle_turnoff();

  if(BQ24745_ACOK())
  {
    charging_state |= 0x80;
  }
  else
  {
    //charging_state &= ~(0x80);
    charging_state = 0;
  }

  set_ADC_values();
  update_battery_meter();

  switch(state)
  {
    case sLowBatteryOnCharger:

      battery_fully_charged = 0;
      charging_state = 0x80;
      charging_restart_counter = 0;

      if(BQ24745_ACOK()==0)
      {
        state = sLowBatteryOffCharger;
      }

      if(return_battery_voltage() > MIN_LOW_BATT_CHARGING_ADC_COUNTS)
      {
        SYS_BUS_ON(1);
        V5_ON(1);
        state = sFullBatteryOnCharger;
      }

    charge_battery(return_max_charging_current());

    if(did_charge_timeout(0))
    {
      state = sTrickleCharging;
      charging_state |= 0x10;
    }

    break;

    case sFullBatteryOnCharger:
      if(BQ24745_ACOK()==0)
      {
        state = sLowBatteryOffCharger;
      }



      if(return_max_charging_current() == 0)
      {
        state = sTrickleCharging;
        charge_battery(0);
        charging_state |= 0x20;
      }

      else if(check_delta_v(0))
      {
        state = sTrickleCharging;
        charge_battery(0);
        charging_state |= 0x40;
      }

    else if(did_charge_timeout(0))
    {
      state = sTrickleCharging;
      charging_state |= 0x10;
    }
      else
      {
        charge_battery(return_max_charging_current());
      }

    break;

    case sTrickleCharging:


      if(BQ24745_ACOK()==0)
      {
        state = sLowBatteryOffCharger;
      }


      if(return_max_charging_current())
      {
          charge_battery(128);
      }
      else
      {
          charge_battery(0);
      }

      charging_restart_counter++;
      if(charging_restart_counter > _24_HOURS)
      {
        charging_restart_counter = 0;
        state = sLowBatteryOnCharger;
        did_charge_timeout(1);
      }


    break;

    break;
    case sLowBatteryOffCharger:

      check_delta_v(1);
      did_charge_timeout(1);
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
        if(low_battery_counter > 1000)
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
      low_battery_counter = 0;
      if(return_battery_voltage() < MIN_LOW_BATT_ADC_COUNTS)
      {
        state = sLowBatteryOffCharger;
      }
      if(BQ24745_ACOK())
      {

        state = sLowBatteryOnCharger;
      }
    break;


  }




}


static unsigned int return_battery_voltage(void)
{

  //6:1 voltage divider:
  //ADC*3.3/1023*6
  //ADC = Voltage*1023/3.3/6
  return battery_voltage;
  //return 1000;
}

static unsigned int return_max_charging_current(void)
{

  //return 0 if either battery is too hot:
  if( (B1_THERM <= THERMISTOR_45C) || (B2_THERM <= THERMISTOR_45C) )
    return 0;

  /*if( (B1_THERM <= THERMISTOR_40C) || (B2_THERM <= THERMISTOR_40C) )
  {
    return 1024;
  }*/


  //return 1 if either battery has a thermistor and the temperature isn't too cold:
  if(B1_THERM < MAX_THERM_ADC_COUNTS)
  {
    return 1536;
  }

  if(B2_THERM < MAX_THERM_ADC_COUNTS)
  {
    return 1536;
  }


  return 0;
}

static void charge_battery(unsigned int charge_current)
{
  static unsigned int last_charge_current = 0;
  static unsigned int register_write_state = 0;
  static unsigned int i2c3_timeout_counter = 0;

  if(charge_current > 0)
  {  
    BQ24745_ON(1);
  }
  else
  {
    BQ24745_ON(0);
  }

  if(charge_current != last_charge_current)
  {
    register_write_state = 0;
  }
  

  if(i2c3_transmit_word_completed)
  {
    i2c3_timeout_counter = 0;
    i2c3_transmit_word_completed = 0;
    i2c3_interrupt_state = 0;
    i2c3_slave_address = BQ24745_I2C_ADD;

    switch(register_write_state)
    {
      case 0:
        i2c3_register = BQ24745_CHG_CURRENT_REG;
        i2c3_tx_byte1 = charge_current & 0xff;
        i2c3_tx_byte2 = charge_current >> 8;
        register_write_state++;
      break;
      case 1:
        i2c3_register = BQ24745_CHG_VOLTAGE_REG;
        i2c3_tx_byte1 = BQ24745_CHG_VOLTAGE & 0xff;
        i2c3_tx_byte2 = BQ24745_CHG_VOLTAGE >> 8;
        register_write_state++;
      break;
      case 2:
        i2c3_register = BQ24745_INPUT_CURRENT_REG;
        i2c3_tx_byte1 = BQ24745_INPUT_CURRENT & 0xff;
        i2c3_tx_byte2 = BQ24745_INPUT_CURRENT >> 8;
        register_write_state = 0;
      break;
    }
    _MI2C3IE = 1;
    I2C3CONbits.SEN = 1;

  }
  else
  {
    i2c3_timeout_counter++;
  }


  last_charge_current = charge_current;

}

static void set_ADC_values(void)
{
  battery_voltage = ADC1BUF2;
  B1_THERM = ADC1BUF0;
  B2_THERM = ADC1BUF1;

}

static void i2c3_ISR(void)
{
  



  switch(i2c3_interrupt_state)
		{
			//start condition has finished, now we send the slave address (with read bit set to 0)
			case 0x00:
	
					I2C3TRN = (i2c3_slave_address<<1);
					i2c3_interrupt_state++;
	
			break;
	
			//slave address written, now we send register address
			case 0x01:
	
				I2C3TRN = i2c3_register;
				i2c3_interrupt_state++;
	
			break;
	
			//after the slave register has been sent, we send another start condition
			case 0x02:
	
				I2C3TRN = i2c3_tx_byte1;
				i2c3_interrupt_state++;
	
			break;
	
			//start condition completed; we send the slave address again, this time with the read bit set
			case 0x03:
	
				I2C3TRN = i2c3_tx_byte2;
				i2c3_interrupt_state++;
	
			break;
	
			//Send stop condition.
			case 0x04:
	
				I2C3CONbits.PEN = 1;
				i2c3_interrupt_state++;
	
			break;
	
			//stop has finished.  Alert (though module variable) that a new word is ready.
			case 0x05:
	
					//need more general descriptor
					i2c3_transmit_word_completed = 1;
	
			break; 
	
			//gets here if message wasn't acked properly, and we sent a stop condition
			case 0x06:
	
			break;
	
	
	
		}
}

void  __attribute__((__interrupt__, auto_psv)) _MI2C3Interrupt(void)
{
  _MI2C3IF = 0;
  i2c3_ISR();
}

void i2c3_init(void)
{

	I2C3CON = 0x1000;

	I2C3CONbits.I2CEN = 1;

	I2C3BRG = 0xff;

	//initialize I2C interrupts

  _MI2C3IE = 0;

}

static unsigned char check_delta_v(unsigned char reset)
{
  static unsigned int counter = 0;
  static unsigned int flat_voltage_counter = 0;
  static int average_voltage = 0;
  static int last_average_voltage = 0;
  
  //1V per hour = 310 ADC counts per hour = 5.17 ADC counts per minute
  //.5V per hour = 155 counts per hour = 2.58 counts per minute
  //static int rise_threshold = 1;
  static int fall_threshold = 5;
  static int voltage_latch_threshold = 2;

  static int debug_average_voltage = 0;
  static int last_last_average_voltage = 0;

  unsigned int num_samples = 10;


  if(reset)
  {
    counter = 0;
    flat_voltage_counter = 0;
    last_average_voltage = 0;
    average_voltage = 0;
    return 0;
  }

  //return 0;

  if(counter < num_samples)
  {
    average_voltage+=return_battery_voltage();
  }
  else if(counter == num_samples)
  {
    average_voltage = average_voltage/num_samples;
  }

  counter++;



  //every second, check for rise
  if(counter > 100)
  {
    counter = 0;


    if(last_average_voltage == 0) last_average_voltage = average_voltage;

    if(average_voltage <= (last_average_voltage-fall_threshold) )
    //if(average_voltage < (last_average_voltage+rise_threshold))
    {
      flat_voltage_counter++;
    }
    else
    {
      flat_voltage_counter = 0;
      last_last_average_voltage = last_average_voltage;
      if(average_voltage > last_average_voltage)
      {
        //prevent voltage spike from raising the average too much
        if( (average_voltage-last_average_voltage) <= voltage_latch_threshold)
          last_average_voltage = average_voltage;
        else
          last_average_voltage = last_average_voltage + voltage_latch_threshold;
      }
    }
    debug_average_voltage = average_voltage;
    average_voltage = 0;
  }

  if(flat_voltage_counter > 10)
  {
    /*charge_battery(0);
    Nop();
    while(1)  ClrWdt();
    Nop()*/;
    return 1;
  }

  return 0;



}

static void update_battery_meter(void)
{

  static unsigned int current_voltage = 0;
  //static unsigned char last_battery_meter = 0;
  static unsigned int battery_levels[5] = {BATTERY_METER_0, BATTERY_METER_25, BATTERY_METER_50, BATTERY_METER_75, BATTERY_METER_100};
  static unsigned int battery_meter_hysteresis = 10;
  static unsigned int motors_stopped_counter = 0;

  current_voltage = return_battery_voltage();

  /*charging_state &= 0b11111100;
  charging_state |= current_voltage >> 8;
  battery_meter = current_voltage&0xff;
  return;*/

  if(are_motors_stopped())
  {
    motors_stopped_counter++;
    if(motors_stopped_counter > 100)
    {

      if(current_voltage > battery_levels[(unsigned char)battery_meter]+battery_meter_hysteresis)
        battery_meter++;
      else if(current_voltage < (battery_levels[(unsigned char)battery_meter]) )
        battery_meter--;

      if(battery_meter > 4)
        battery_meter = 4;
      else if(battery_meter < 0)
        battery_meter = 0;

      motors_stopped_counter = 0;
      
    }
  }
  else
    motors_stopped_counter = 0;
  



}

static unsigned char did_charge_timeout(unsigned char reset)
{
  static unsigned long charge_timeout = 0;

  if(reset)
  {
    charge_timeout = 0;
    return 0;
  }

  charge_timeout++;
  if(charge_timeout > _4_HOURS)
    return 1;

  return 0;
  


}

static void handle_turnoff(void)
{
  static unsigned int turnoff_counter = 0;

  if(PWR_BUTTON())
    turnoff_counter++;
  else
    turnoff_counter = 0;

  if(turnoff_counter > 100)
  {
    V5_ON(0);
    SYS_BUS_ON(0);
    while(PWR_BUTTON())
    {
      ClrWdt();
    }
    while(BQ24745_ACOK()==0)
    {
      ClrWdt();
    }
  }



}

unsigned char return_battery_meter(void) {return battery_meter;}
unsigned char return_charging_state(void) {return charging_state;}
