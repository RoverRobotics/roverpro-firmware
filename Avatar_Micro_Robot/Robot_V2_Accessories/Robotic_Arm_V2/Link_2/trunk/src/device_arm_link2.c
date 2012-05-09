#include "device_arm_link2.h"
#include "stdhdr.h"

#define USB_TIMEOUT_ENABLED

#define GRIPPER_BRAKE_EN(a)   _TRISB15 = !a
#define GRIPPER_BRAKE_ON(a)   _LATB15 = a

#define GRIPPER_MODE_EN(a)    _TRISC13 = !a
#define GRIPPER_MODE_ON(a)   _LATC13 = a

#define GRIPPER_COAST_EN(a)   _TRISC14 = !a
#define GRIPPER_COAST_ON(a)   _LATC14 = a

#define GRIPPER_DIR_EN(a)     _TRISD0 = !a
#define GRIPPER_DIR_ON(a)     _LATD0 = a

#define GRIPPER_FF1()         _RB13
#define GRIPPER_FF2()         _RB12
#define GRIPPER_DIRO()        _RB14

#define GRIPPER_PWM_OR        _RP19R

#define WRIST_BRAKE_EN(a)     _TRISD7 = !a
#define WRIST_BRAKE_ON(a)     _LATD7 = a

#define WRIST_MODE_EN(a)      _TRISD8 = !a
#define WRIST_MODE_ON(a)      _LATD8 = a

#define WRIST_COAST_EN(a)     _TRISD9 = !a
#define WRIST_COAST_ON(a)     _LATD9 = a

#define WRIST_DIR_EN(a)       _TRISD10 = !a
#define WRIST_DIR_ON(a)       _LATD10 = a

#define WRIST_FF1()         _RB13
#define WRIST_FF2()         _RB12
#define WRIST_DIRO()        _RB14

#define WRIST_PWM_OR        _RP21R

#define GRIPPER_POT_EN(a)       _PCFG4 = !a
#define GRIPPER_ACT_POT_EN(a)   _PCFG5 = !a
#define ELBOW_POT_1_EN(a)       _PCFG0 = !a
#define ELBOW_POT_2_EN(a)       _PCFG1 = !a
#define WRIST_POT_1_EN(a)       _PCFG2 = !a
#define WRIST_POT_2_EN(a)       _PCFG3 = !a
#define THERMISTOR_EN(a)        _PCFG8 = !a

#define GRIPPER_POT_CH          4
#define GRIPPER_ACT_POT_CH      5
#define ELBOW_POT_1_CH          0
#define ELBOW_POT_2_CH          1
#define WRIST_POT_1_CH          2
#define WRIST_POT_2_CH          3
#define THERMISTOR_CH           8

#define RS485_DI_OR           _RP20R
#define RS485_RO_PIN          25

#define RS485_OUTEN_EN(a)     _TRISD6 = !a
#define RS485_OUTEN_ON(a)     _LATD6 = a

#define RS485_TX_LENGTH 10
#define RS485_RX_LENGTH 9

#define RS485_BASE_LENGTH 9
#define RS485_LINK1_LENGTH 5

#define MAX_WRIST_SPEED 50
#define MAX_GRIPPER_SPEED 30

#define MAX_GRIPPER_ACT 900
#define MIN_GRIPPER_ACT 500


//number of ADC counts that gripper pot differs from gripper actuator pot
//gripper potentiometer is ~8.5mm closer to the gripper
//8.5mm*1023counts/45mm = 193 counts
#define GRIPPER_OFFSET 193

//max number of ADC counts the gripper can slip the clutch before the motor
//stops driving it in that direction
//4.25mm*1023 counts/45mm = 96.6 counts
#define MAX_GRIPPER_SLIP 96
#define GRIPPER_SLIP_HYSTERESIS 20 

#define USB_TIMEOUT_COUNTS 5


void set_gripper_velocity(int velocity);
void set_wrist_velocity(int velocity);
unsigned int return_adc_value(unsigned char ch);
void RS485_RX_ISR(void);
void RS485_TX_ISR(void);
unsigned int return_CRC(unsigned char* data, unsigned char length);
char Is_CRC_valid(unsigned char* data, unsigned char length);
void send_rs485_message(void);
void motor_accel_loop(int desired_gripper_velocity, int desired_wrist_velocity);
int return_adjusted_gripper_velocity(void);

void test_arm_motors(void);
void test_arm_motors_send_i2c(void);

void calibrate_angle_sensor(void);
unsigned int return_calibrated_angle(unsigned char pot_1_ch, unsigned char pot_2_ch, unsigned int offset_angle);
unsigned int return_combined_pot_angle(unsigned char pot_1_ch, unsigned char pot_2_ch);
unsigned int wrist_angle_offset = 0;
unsigned int wrist_angle = 0;
unsigned int test_wrist_pot1_angle = 0;
unsigned int test_wrist_pot2_angle = 0;
unsigned int test_wrist_pot1_value = 0;
unsigned int test_wrist_pot2_value = 0;

char gripper_clutch_reset_direction = 0;
unsigned char gripper_clutch_reset_in_progress = 0;
char gripper_direction_latch = 0;
void infinite_gripper_test_loop(void);


unsigned int gripper_pot_value, gripper_act_pot_value, elbow_pot_1_value, elbow_pot_2_value, wrist_pot_1_value, wrist_pot_2_value, thermistor_value = 0;
unsigned char rs485_tx_message[RS485_TX_LENGTH];
unsigned char rs485_tx_buffer[RS485_TX_LENGTH];
unsigned char rs485_rx_message[RS485_RX_LENGTH];
unsigned char rs485_rx_buffer[RS485_RX_LENGTH];

unsigned char rs485_base_message[RS485_BASE_LENGTH];
unsigned char rs485_link1_message[RS485_LINK1_LENGTH];

unsigned char rs485_transmitting = 0;

unsigned char rs485_link1_message_ready = 0;
unsigned char rs485_base_message_ready = 0;

unsigned char USB_timeout_counter = 0;




void Arm_Link2_Init(void)
{

  AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;

	AD1PCFGL = 0xffff;
	TRISB = 0xffff;
	TRISC = 0xffff;
	TRISD = 0xffff;
	TRISE = 0xffff;
	TRISF = 0xffff;
	TRISG = 0xffff;

  GRIPPER_BRAKE_ON(0);
  GRIPPER_MODE_ON(0);
  GRIPPER_COAST_ON(0);
  GRIPPER_DIR_ON(0);
  GRIPPER_DIR_ON(0);

  WRIST_BRAKE_ON(0);
  WRIST_MODE_ON(0);
  WRIST_COAST_ON(0);
  WRIST_DIR_ON(0);
  WRIST_DIR_ON(0);

  RS485_OUTEN_ON(0);

  GRIPPER_BRAKE_EN(1);
  GRIPPER_MODE_EN(1);
  GRIPPER_COAST_EN(1);
  GRIPPER_DIR_EN(1);
  GRIPPER_DIR_EN(1);

  WRIST_BRAKE_EN(1);
  WRIST_MODE_EN(1);
  WRIST_COAST_EN(1);
  WRIST_DIR_EN(1);
  WRIST_DIR_EN(1);

  GRIPPER_MODE_ON(1);
  GRIPPER_DIR_ON(1);
  GRIPPER_BRAKE_ON(1);

  WRIST_MODE_ON(1);
  WRIST_DIR_ON(1);
  WRIST_BRAKE_ON(1);

  GRIPPER_POT_EN(1);
  GRIPPER_ACT_POT_EN(1);

  ELBOW_POT_1_EN(1);
  ELBOW_POT_2_EN(1);
  WRIST_POT_1_EN(1);
  WRIST_POT_2_EN(1);
  THERMISTOR_EN(1);

  RS485_OUTEN_EN(1);

	//initialize ADC
	AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;
	
	AD1CON3bits.ADCS = 0xff;
	AD1CON3bits.SAMC = 0x1f;

	//auto convert
	AD1CON1bits.SSRC = 7;

  AD1CON1bits.ADON = 1;

  //setup UART
	RS485_DI_OR = 3;

  U1RXInterruptUserFunction = RS485_RX_ISR;
  U1TXInterruptUserFunction = RS485_TX_ISR;
	
	//set U1RX to GPS rx pin
	RPINR18bits.U1RXR = RS485_RO_PIN;	
	
	U1BRG = 103;
	
	IEC0bits.U1RXIE = 1;
  _U1TXIE = 1;
	
	U1MODEbits.UARTEN = 1;

  U1STAbits.UTXEN = 1;


  GRIPPER_PWM_OR = 18;	//OC1
	WRIST_PWM_OR	= 19;	//OC2

	OC1R=0;
	OC1RS=2000;
	OC1CON2bits.SYNCSEL=0x1F;
	OC1CON2bits.OCTRIG=CLEAR;
	OC1CON1bits.OCTSEL=0b000;//Timer2
	OC1CON1bits.OCM=0b110;

	OC2R=0;
	OC2RS=2000;
	OC2CON2bits.SYNCSEL=0x1F;
	OC2CON2bits.OCTRIG=CLEAR;
	OC2CON1bits.OCTSEL=0b000;//Timer2
	OC2CON1bits.OCM=0b110;

  set_gripper_velocity(0);
  set_wrist_velocity(0);

  T2CONbits.TON = 1;

  GRIPPER_COAST_ON(1);
  WRIST_COAST_ON(1);

  calibrate_angle_sensor();

  //test_arm_motors();
  //infinite_gripper_test_loop();

}

void Link2_Process_IO(void)
{

  unsigned int i;
  int adjusted_gripper_velocity = 0;

  gripper_pot_value = return_adc_value(GRIPPER_POT_CH);
  gripper_act_pot_value = return_adc_value(GRIPPER_ACT_POT_CH);
  elbow_pot_1_value = return_adc_value(ELBOW_POT_1_CH);
  elbow_pot_2_value = return_adc_value(ELBOW_POT_2_CH);
  wrist_pot_1_value = return_adc_value(WRIST_POT_1_CH);
  wrist_pot_2_value = return_adc_value(WRIST_POT_2_CH);
  thermistor_value = return_adc_value(THERMISTOR_CH);

  REG_ARM_JOINT_POSITIONS.gripper = gripper_pot_value;
  REG_ARM_JOINT_POSITIONS.gripper_actuator = gripper_act_pot_value;

  send_rs485_message();

  //wait for message to finish sending
  while(rs485_transmitting);
  //wait for final byte to finish before changing to RX mode
  while(U1STAbits.TRMT == 0);
  RS485_OUTEN_ON(0);

  //clear receive overrun error, so receive doesn't stop
  U1STAbits.OERR = 0;

  //pretty dumb control loop
  for(i=0;i<10;i++)
  {
    adjusted_gripper_velocity = return_adjusted_gripper_velocity();

    //motor_accel_loop(adjusted_gripper_velocity, REG_ARM_MOTOR_VELOCITIES.wrist);
    motor_accel_loop(0, REG_ARM_MOTOR_VELOCITIES.wrist);

    block_ms(10);
  }
  
  wrist_angle = return_calibrated_angle(WRIST_POT_1_CH, WRIST_POT_2_CH, wrist_angle_offset);

  #ifdef USB_TIMEOUT_ENABLED
    USB_timeout_counter++;
    if(USB_timeout_counter > USB_TIMEOUT_COUNTS)
    {
      USB_timeout_counter = USB_TIMEOUT_COUNTS+1;
      REG_ARM_MOTOR_VELOCITIES.turret = 0;
      REG_ARM_MOTOR_VELOCITIES.shoulder = 0;
      REG_ARM_MOTOR_VELOCITIES.elbow = 0;
      REG_ARM_MOTOR_VELOCITIES.wrist = 0;
      REG_ARM_MOTOR_VELOCITIES.gripper = 0;
    }
  #endif

 }

void set_gripper_velocity(int velocity)
{

  if(velocity > 100)
    velocity = 100;
  else if(velocity < -100)
    velocity = -100;

  if(velocity > 0)
    GRIPPER_DIR_ON(1);
  else
    GRIPPER_DIR_ON(0);

  OC1R = abs(velocity)*20;


}

void set_wrist_velocity(int velocity)
{

  if(velocity > 100)
    velocity = 100;
  else if(velocity < -100)
    velocity = -100;

  if(velocity > 0)
    WRIST_DIR_ON(1);
  else
    WRIST_DIR_ON(0);

  OC2R = abs(velocity)*20;


}

unsigned int return_adc_value(unsigned char ch)
{

	unsigned int return_value = 0;
	AD1CON1bits.ADON = 0;
	AD1CHS0bits.CH0SA = ch;
	AD1CON1bits.ADON = 1;	
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE);
	return_value = ADC1BUF0;
	return return_value;
}


void RS485_RX_ISR(void)
{

  static unsigned char message_index = 0;
  static unsigned char current_length = 100;
  static unsigned char current_device = 0x00;
  unsigned char i;
  _U1RXIF = 0;

  //if we've gotten to the end of a message
  if(message_index >= current_length)
  {
    if(current_device == DEVICE_ARM_BASE)
    {
      for(i=0;i<RS485_BASE_LENGTH;i++)
      {
        rs485_base_message[i] = rs485_rx_buffer[i];
      }
      rs485_base_message_ready = 1;
    }
    else if(current_device == DEVICE_ARM_LINK1)
    {
      for(i=0;i<RS485_LINK1_LENGTH;i++)
      {
        rs485_link1_message[i] = rs485_rx_buffer[i];
      }
      rs485_link1_message_ready = 1;
    }
    current_device = 0x00;
    message_index = 0;
    current_length = 100;
    return;
  }

  rs485_rx_buffer[message_index] = U1RXREG;

  switch(message_index)
  {
    case 0x00:
      if(rs485_rx_buffer[0] != 0xff)
      {
        message_index = 0;
        return;
      }
    break;
    case 0x01:
//    If the second byte isn't 0xcc
      if(rs485_rx_buffer[1] != 0xcc)
      {
//      If it's actually 0xff, try starting message with this byte
//      (so we don't miss a valid message preceded by 0xff
        if(rs485_rx_buffer[1] == 0xff)
        {
          rs485_rx_buffer[0] = 0xff;
          message_index = 1;
          break;
        }
        else
        {
          message_index = 0;
          return; 
        }
      }
    break;
    case 0x02:
      //if we're dealing with a message from the base
      if( (rs485_rx_buffer[2] == 0x0b) )
      {
        current_length = RS485_BASE_LENGTH;
        current_device = DEVICE_ARM_BASE;
      }
      //if it's a message from Link 1
      else if( (rs485_rx_buffer[2] == 0x0c) )
      {
        current_length = RS485_LINK1_LENGTH;
        current_device = DEVICE_ARM_LINK1;
      }
      //if it's an invalid device
      else
      {
        message_index = 0;
        return;
      }
    break;
    
    
        
  }
  message_index++;


}


void RS485_TX_ISR(void)
{
  static unsigned char message_index = 0;
  _U1TXIF = 0;
  
  //start at 1, since we already sent the first byte
  message_index++;

  if(message_index >= RS485_TX_LENGTH)
  {
    message_index = 0;
    _U1TXIE = 0;
    rs485_transmitting = 0;
    return;
  } 

  U1TXREG = rs485_tx_buffer[message_index];

}



unsigned int return_CRC(unsigned char* data, unsigned char length)
{
	static unsigned int crc;
	unsigned char i;
	crc = 0;

	for(i=0;i<length;i++)
	{
		crc = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= data[i];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= (crc << 8) << 4;
		crc ^= ((crc & 0xff) << 4) << 1;
		crc &= 0xFFFF;
	}
	return crc; 
}

char Is_CRC_valid(unsigned char* data, unsigned char length)
{
	
	unsigned int CRC_received = 0;
	unsigned int CRC_calculated = 0;

	unsigned char data_to_CRC[20];
	unsigned char i;

	for(i=2;i<length-2;i++)
	{
		data_to_CRC[i-2] = data[i];
	}

	CRC_received = (data[length-2]<<8)+(data[length-1]);

	CRC_calculated = return_CRC(data_to_CRC,length-4);

	if(CRC_received == CRC_calculated)
		return 1;
	
	return 0;


}

void send_rs485_message(void)
{
  unsigned char data_to_CRC[RS485_TX_LENGTH-4];
  unsigned int CRC;
  unsigned char i;

  rs485_transmitting = 1;
 /* for(i=0;i<rs485_tx_length-2;i++)
  {
    data_to_CRC 
  }*/

  RS485_OUTEN_ON(1);

  //make sure motor velocities are between -100 and 100 -- if not, set to 0
  if( abs(REG_ARM_MOTOR_VELOCITIES.turret) > 100 )
    REG_ARM_MOTOR_VELOCITIES.turret = 0;

  if( abs(REG_ARM_MOTOR_VELOCITIES.shoulder) > 100 )
    REG_ARM_MOTOR_VELOCITIES.shoulder = 0;

  if( abs(REG_ARM_MOTOR_VELOCITIES.elbow) > 100 )
    REG_ARM_MOTOR_VELOCITIES.elbow = 0;

  if( abs(REG_ARM_MOTOR_VELOCITIES.wrist) > 100 )
    REG_ARM_MOTOR_VELOCITIES.wrist = 0;

  if( abs(REG_ARM_MOTOR_VELOCITIES.gripper) > 100 )
    REG_ARM_MOTOR_VELOCITIES.gripper = 0;


  //load motor velocities into array
  data_to_CRC[0] = 0x0d;
  data_to_CRC[1] = REG_ARM_MOTOR_VELOCITIES.turret+100;
  data_to_CRC[2] = REG_ARM_MOTOR_VELOCITIES.shoulder+100;
  data_to_CRC[3] = REG_ARM_MOTOR_VELOCITIES.elbow+100;
  data_to_CRC[4] = REG_ARM_MOTOR_VELOCITIES.wrist+100;
  data_to_CRC[5] = REG_ARM_MOTOR_VELOCITIES.gripper+100;

  for(i=0;i<RS485_TX_LENGTH-4;i++)
  {
    rs485_tx_buffer[i+2] = data_to_CRC[i];
  }

  CRC = return_CRC(data_to_CRC,RS485_TX_LENGTH-4);

  rs485_tx_buffer[0] = 0xff;
  rs485_tx_buffer[1] = 0xcc;
  rs485_tx_buffer[RS485_TX_LENGTH-2] = CRC>>8;
  rs485_tx_buffer[RS485_TX_LENGTH-1] = CRC&0xFF;

  /*for(i=0;i<RS485_TX_LENGTH;i++)
  {
    rs485_tx_buffer[i] = rs485_tx_message[i];
  }*/

  U1TXREG = rs485_tx_buffer[0];
  _U1TXIE = 1;


}

void motor_accel_loop(int desired_gripper_velocity, int desired_wrist_velocity)
{
  static int gripper_velocity;
  static int wrist_velocity;
  unsigned char step_size = 2;

  /*if( abs(desired_gripper_velocity) < 15)
  {
    gripper_velocity = 0;
    set_gripper_velocity(0);
  }
  if( abs(desired_wrist_velocity) < 15 )
  {
    wrist_velocity = 0;
    set_wrist_velocity(0);
  }*/

  if(desired_gripper_velocity > (gripper_velocity + step_size))
    gripper_velocity+=step_size;
  else if (desired_gripper_velocity < (gripper_velocity - step_size) )
    gripper_velocity-=step_size;

  if(desired_wrist_velocity > (wrist_velocity + step_size) )
    wrist_velocity+=step_size;
  else if (desired_wrist_velocity < (wrist_velocity - step_size) )
    wrist_velocity-=step_size;

  if(gripper_velocity > MAX_GRIPPER_SPEED)
    gripper_velocity = MAX_GRIPPER_SPEED;
  else if(gripper_velocity < -MAX_GRIPPER_SPEED)
    gripper_velocity = -MAX_GRIPPER_SPEED;
  
  if(wrist_velocity > MAX_WRIST_SPEED)
    wrist_velocity = MAX_WRIST_SPEED;
  else if(wrist_velocity < -MAX_WRIST_SPEED)
    wrist_velocity = -MAX_WRIST_SPEED;


  if(abs(gripper_velocity) < 15)
    set_gripper_velocity(0);
  else
    set_gripper_velocity(gripper_velocity);

  if(abs(wrist_velocity) < 15)
    set_wrist_velocity(0);
  else
    set_wrist_velocity(wrist_velocity);


}

void calibrate_angle_sensor(void)
{
  wrist_angle_offset = return_combined_pot_angle(WRIST_POT_1_CH, WRIST_POT_2_CH);

}
unsigned int return_calibrated_angle(unsigned char pot_1_ch, unsigned char pot_2_ch, unsigned int offset_angle)
{
  unsigned int combined_pot_angle = 0;
  unsigned int calibrated_angle = 0;
  combined_pot_angle = return_combined_pot_angle(pot_1_ch, pot_2_ch);


  if(combined_pot_angle < offset_angle)
  {
    calibrated_angle = 365+combined_pot_angle-offset_angle;
  }
  else
  {
    calibrated_angle = combined_pot_angle-offset_angle;
  }

  return calibrated_angle;

}

unsigned int return_combined_pot_angle(unsigned char pot_1_ch, unsigned char pot_2_ch)
{
  unsigned int combined_pot_value = 0;
  unsigned int pot_1_value, pot_2_value = 0;
  unsigned int combined_pot_angle = 0;
  int temp1 = 0;
  int temp2 = 0;
  float scale_factor = 0;
  
  pot_1_value = return_adc_value(pot_1_ch);
  pot_2_value = 1023-return_adc_value(pot_2_ch);


  test_wrist_pot1_value = pot_1_value;
  test_wrist_pot2_value = pot_2_value;

 // test_wrist_pot1_angle = pot_1_value*.326+341.7;
  test_wrist_pot1_angle = pot_1_value*.326+58.35;
  if(test_wrist_pot1_angle > 360)
  {
    test_wrist_pot1_angle = test_wrist_pot1_angle -360;
  }
  test_wrist_pot2_angle = pot_2_value*.326+13.35;

  //!!!!!!need to get full angle range out of this
  //right now we only have 333 degrees
  //maybe multiply by 360/333.3 somewhere?

  //if pot 1 is out of linear range
  if( (pot_1_value < 15) || (pot_1_value > 1015) )
  {
    //333.3 degrees, 1023 total counts, 333.3/1023 = .326
    combined_pot_angle = pot_2_value*.326+13.35;
  }
  //if pot 2 is out of linear range
  else if( (pot_2_value < 15) || (pot_2_value > 1015) )
  {
    //333.3 degrees, 1023 total counts, 333.3/1023 = .326
    //13.35 degrees + 45 degrees = 58.35 degrees
    combined_pot_angle = pot_1_value*.326+58.35;
    if(combined_pot_angle > 360)
    {
      combined_pot_angle = combined_pot_angle - 360;
    }
  }
  //if both pot 1 and pot 2 values are valid
  else
  {

    //figure out which one is closest to the end of range
    temp1 = pot_1_value - 512;
    temp2 = pot_2_value - 512;

    //offset, so that both pot values should be the same
    //45/333.33*1023 = 138.1
    pot_1_value = pot_1_value+138;
    if(pot_1_value > 1023)
      pot_1_value = pot_1_value-1023;
  

    //if pot1 is closer to the end of range
    if(abs(temp1) > abs(temp2) )
    {
      scale_factor = ( 512-abs(temp1) )/ 512.0;
      combined_pot_value = (pot_1_value*scale_factor + pot_2_value*(1-scale_factor));

    }
    //if pot2 is closer to the end of range
    else
    {

      scale_factor = (512-abs(temp2) )/ 512.0;
      combined_pot_value = (pot_2_value*scale_factor + pot_1_value*(1-scale_factor));

    }

    //333.3 degrees, 1023 total counts, 333.3/1023 = .326
    combined_pot_angle = combined_pot_value*.326+13.35;

  }




  return combined_pot_angle;


}


//make sure gripper actuator doesn't hit hard stop.  Also, stop the
//gripper motor from slipping the clutch too far.
//
//Note:   Closest to the gripper = 1023
//        Furthest from the gripper = 0
//        Positive gripper motor speed moves the motor
//        closest to the gripper (to open)
int return_adjusted_gripper_velocity(void)
{
  int adjusted_gripper_velocity = 0;
  adjusted_gripper_velocity = REG_ARM_MOTOR_VELOCITIES.gripper;
  unsigned int adjusted_gripper_pot_value = 0;
  int gripper_clutch_slip = 0;
  int input_gripper_velocity = 0;

  REG_ARM_JOINT_POSITIONS.gripper = return_adc_value(GRIPPER_POT_CH);
  REG_ARM_JOINT_POSITIONS.gripper_actuator = return_adc_value(GRIPPER_ACT_POT_CH);



  input_gripper_velocity = REG_ARM_MOTOR_VELOCITIES.gripper;

  adjusted_gripper_velocity = input_gripper_velocity;

  //don't move the gripper motor too close to a hard stop

  //gripper closed hard stop
  if(REG_ARM_JOINT_POSITIONS.gripper_actuator < MIN_GRIPPER_ACT)
  {
    if(REG_ARM_MOTOR_VELOCITIES.gripper < 0)
    {
      adjusted_gripper_velocity = 0;
      gripper_direction_latch = -1;
    }
  }
  //gripper open hard stop
  else if(REG_ARM_JOINT_POSITIONS.gripper_actuator > MAX_GRIPPER_ACT)
  {
    if(REG_ARM_MOTOR_VELOCITIES.gripper > 0)
    {
      adjusted_gripper_velocity = 0;
      gripper_direction_latch = 1;
    }
  }

  //if the clutch has slipped too much, don't let the actuator slip
  //it further
  if(REG_ARM_JOINT_POSITIONS.gripper > GRIPPER_OFFSET)
  {
    adjusted_gripper_pot_value = REG_ARM_JOINT_POSITIONS.gripper-GRIPPER_OFFSET;
  }
  else
  {
    adjusted_gripper_pot_value = 0;
  }

  gripper_clutch_slip = adjusted_gripper_pot_value-REG_ARM_JOINT_POSITIONS.gripper_actuator;

  //if the gripper is opening, and there is too much negative slip, stop the motor
  if( (adjusted_gripper_velocity > 0) && (gripper_clutch_slip < -MAX_GRIPPER_SLIP) )
  {
    adjusted_gripper_velocity = 0;
    gripper_clutch_reset_direction = -1;
    gripper_clutch_reset_in_progress = 1;
  }
  //if the gripper is closing, and there is too much positive slip, stop the motor
  else if( (adjusted_gripper_velocity < 0) && (gripper_clutch_slip > MAX_GRIPPER_SLIP) )
  {
    adjusted_gripper_velocity = 0;
    gripper_clutch_reset_direction = 1;
    gripper_clutch_reset_in_progress = 1;
  }
  //clutch hasn't slipped too much



    if(gripper_clutch_reset_direction == 1)
    {
      //if the gripper is opening, and the slip has swung the other direction
      if ( (adjusted_gripper_velocity > 0) && (gripper_clutch_slip <= 0))
      {
        gripper_clutch_reset_in_progress = 0;
        adjusted_gripper_velocity = 0;
        //gripper_direction_latch = 1;
      }
    }
    //gripper needs to close to reset clutch
    else if(gripper_clutch_reset_direction == -1)
    {
      if ( (adjusted_gripper_velocity < 0) && (gripper_clutch_slip >= 0))
      {
        gripper_clutch_reset_in_progress = 0;
        adjusted_gripper_velocity = 0;
       // gripper_direction_latch = -1;
      }
  
    }



  
  //if the clutch has been reset, don't keep moving it past the reset point
  //once the motor reverses direction, clear the gripper_clutch_reset_direction variable
  //so that non-reset behavior resumes (e.g. slipping to MAX_GRIPPER_SLIP)

  //if the clutch has been reset, but the direction flag hasn't been cleared
  if( (gripper_clutch_reset_direction != 0) && (gripper_clutch_reset_in_progress == 0) )
  {
    //if we had to close the gripper to reset, don't keep closing past the center point of the clutch
    if(gripper_clutch_reset_direction == 1)
    {
      if(adjusted_gripper_velocity < 0)
        gripper_clutch_reset_direction = 0;
      else
        adjusted_gripper_velocity = 0;

    }
    //if we had to open the gripper to reset, don't keep opening past the center point of the clutch
    else if (gripper_clutch_reset_direction == -1)
    {
      if(adjusted_gripper_velocity > 0)
        gripper_clutch_reset_direction = 0;
      else
        adjusted_gripper_velocity = 0;
    }
  }



  //if we hit the travel limit of the actuator,
  //don't move in that direction until the motor spins in the other direction
  if(gripper_direction_latch == 1)
  {
    if(adjusted_gripper_velocity > 0)
      adjusted_gripper_velocity = 0;
    else if(adjusted_gripper_velocity < 0)
      gripper_direction_latch = 0;
  }
  else if(gripper_direction_latch == -1)
  {
    if(adjusted_gripper_velocity < 0)
      adjusted_gripper_velocity = 0;
    else if(adjusted_gripper_velocity > 0)
      gripper_direction_latch = 0;
  }
  
  return adjusted_gripper_velocity;

}

void test_arm_motors(void)
{
  unsigned int i,j;

	IEC0bits.U1RXIE = 0;


  block_ms(3000);

  REG_ARM_MOTOR_VELOCITIES.turret = 20;
  test_arm_motors_send_i2c();
  REG_ARM_MOTOR_VELOCITIES.turret = -20;
  test_arm_motors_send_i2c();
  REG_ARM_MOTOR_VELOCITIES.turret = 0;
  REG_ARM_MOTOR_VELOCITIES.shoulder = 20;
  test_arm_motors_send_i2c();
  REG_ARM_MOTOR_VELOCITIES.shoulder = -20;
  test_arm_motors_send_i2c();
  REG_ARM_MOTOR_VELOCITIES.shoulder = 0;
  REG_ARM_MOTOR_VELOCITIES.elbow = 20;
  test_arm_motors_send_i2c();
  REG_ARM_MOTOR_VELOCITIES.elbow = -20;
  test_arm_motors_send_i2c();
  REG_ARM_MOTOR_VELOCITIES.elbow = 0;
  test_arm_motors_send_i2c();

  REG_ARM_MOTOR_VELOCITIES.wrist = 20;
  for(i=0;i<30;i++)
  {
    ClrWdt();
    motor_accel_loop(0, REG_ARM_MOTOR_VELOCITIES.wrist);
    block_ms(10);
  }
  REG_ARM_MOTOR_VELOCITIES.wrist = -20;
  motor_accel_loop(0, 0);
  block_ms(100);
  for(i=0;i<30;i++)
  {
    ClrWdt();
    motor_accel_loop(0, REG_ARM_MOTOR_VELOCITIES.wrist);
    block_ms(10);
  }
  set_wrist_velocity(0);

  while(1)
  {
    ClrWdt();
  }

}
void test_arm_motors_send_i2c(void)
{
  unsigned int i;

  
  for(i=0;i<3;i++)
  {
    RS485_OUTEN_ON(1);
    ClrWdt();
    send_rs485_message();
    //wait for message to finish sending
    while(rs485_transmitting);
    //wait for final byte to finish before changing to RX mode
    while(U1STAbits.TRMT == 0);
    RS485_OUTEN_ON(0);
    block_ms(100);
  }

}

void infinite_gripper_test_loop(void)
{
  int adjusted_gripper_velocity = 0;
  unsigned int i,j;
  while(1)
  {

    REG_ARM_MOTOR_VELOCITIES.gripper = 20;
    for(j=0;j<30;j++)
    {
      for(i=0;i<10;i++)
      {
        adjusted_gripper_velocity = return_adjusted_gripper_velocity();
    
        motor_accel_loop(adjusted_gripper_velocity, REG_ARM_MOTOR_VELOCITIES.wrist);
        //motor_accel_loop(0, REG_ARM_MOTOR_VELOCITIES.wrist);
    
        block_ms(10);
      }
    }

    REG_ARM_MOTOR_VELOCITIES.gripper = -20;
    for(j=0;j<10;j++)
    {
      for(i=0;i<10;i++)
      {
        adjusted_gripper_velocity = return_adjusted_gripper_velocity();
    
        motor_accel_loop(adjusted_gripper_velocity, REG_ARM_MOTOR_VELOCITIES.wrist);
        //motor_accel_loop(0, REG_ARM_MOTOR_VELOCITIES.wrist);
    
        block_ms(10);
      }
    }

  }

}

