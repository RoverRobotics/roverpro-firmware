#include "device_arm_link2.h"
#include "stdhdr.h"
#include "DEE Emulation 16-bit.h"
#include "debug_uart.h"

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
#define MIN_WRIST_SPEED 10

#define MAX_GRIPPER_SPEED 20
#define MIN_GRIPPER_SPEED 5

#define MAX_GRIPPER_ACT 950
#define MIN_GRIPPER_ACT 525

#define GRIPPER_FULLY_OPEN 950

#define GRIPPER_CLOSE_SLIP_HYSTERESIS 5

//angle wrist has to move so that we know it isn't stalled
#define WRIST_STALL_DEADBAND 10


//number of ADC counts that gripper pot differs from gripper actuator pot
//gripper potentiometer is ~8.5mm closer to the gripper
//8.5mm*1023counts/45mm = 193 counts
#define GRIPPER_OFFSET 193

//max number of ADC counts the gripper can slip the clutch before the motor
//stops driving it in that direction
//4.25mm*1023 counts/45mm = 96.6 counts
//#define MAX_GRIPPER_SLIP 96
//5.25mm*1023 counts/45mm = 119.35
#define MAX_GRIPPER_SLIP 125
#define MAX_FULLY_OPEN_GRIPPER_SLIP 105
#define GRIPPER_SLIP_HYSTERESIS 20

#define NORMAL_OPERATION 0
#define POSITIVE_OVERSLIP_RESET 1
#define NEGATIVE_OVERSLIP_RESET 2
#define POSITIVE_OVERSLIP_RESET_COMPLETE 3
#define NEGATIVE_OVERSLIP_RESET_COMPLETE 4

#define POT_LOW_THRESHOLD 15
#define POT_HIGH_THRESHOLD 1008

int return_adjusted_wrist_velocity(void);
int return_angle_difference(unsigned int angle_1, unsigned int angle_2);

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

static char read_rs485_angle_values(void);

void calibrate_angle_sensor(void);
static unsigned int return_calibrated_angle(unsigned int uncalibrated_angle, unsigned int offset_angle, char direction);
unsigned int return_combined_pot_angle(unsigned char pot_1_ch, unsigned char pot_2_ch);
unsigned int wrist_angle_offset = 0;
unsigned int test_wrist_pot1_angle = 0;
unsigned int test_wrist_pot2_angle = 0;
unsigned int test_wrist_pot1_value = 0;
unsigned int test_wrist_pot2_value = 0;

char gripper_clutch_reset_direction = 0;
unsigned char gripper_clutch_reset_in_progress = 0;
char gripper_direction_latch = 0;
void infinite_gripper_test_loop(void);
char gripper_actuator_overtravel_direction = 0;
char gripper_clutch_overtravel_direction = 0;


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

unsigned int uncalibrated_turret_angle = 0;
unsigned int uncalibrated_shoulder_angle = 0;

static unsigned int elbow_angle_offset = 0;
static unsigned int shoulder_angle_offset = 0;
static unsigned int turret_angle_offset = 0;

static void update_joint_angles(void);

static void read_stored_angle_offsets(void);

static void enter_debug_mode(void);




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



  read_stored_angle_offsets();

  //test_arm_motors();
  //infinite_gripper_test_loop();
  /*while(1)
  {
    ClrWdt();
    motor_accel_loop(REG_ARM_MOTOR_VELOCITIES.gripper, 0);
  }*/

}

void Link2_Process_IO(void)
{

  unsigned int i;
  int adjusted_gripper_velocity = 0;
  int adjusted_wrist_velocity = 0;
  static int last_adjusted_wrist_velocity = 0;
  //if we receive these set speeds, run calibration
  if( (REG_ARM_MOTOR_VELOCITIES.turret == 123) && (REG_ARM_MOTOR_VELOCITIES.shoulder == 456) && (REG_ARM_MOTOR_VELOCITIES.elbow == 789) )
    calibrate_angle_sensor();

  //if we receive another special set of speeds, go into debug mode
  if( (REG_ARM_MOTOR_VELOCITIES.turret == 987) && (REG_ARM_MOTOR_VELOCITIES.shoulder == 654) && (REG_ARM_MOTOR_VELOCITIES.elbow == 321) )
    enter_debug_mode();

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
    adjusted_wrist_velocity = return_adjusted_wrist_velocity();

    //exponential control due to jerkiness
    //adjusted_wrist_velocity = (adjusted_wrist_velocity*adjusted_wrist_velocity)/50;
    //adjusted_wrist_velocity/=3;
    if(adjusted_wrist_velocity >= MIN_WRIST_SPEED+10)
		adjusted_wrist_velocity = 15;
    else if( (adjusted_wrist_velocity >= MIN_WRIST_SPEED) && (last_adjusted_wrist_velocity==15) )
        adjusted_wrist_velocity = 15;    
    else if(adjusted_wrist_velocity <= -MIN_WRIST_SPEED-10)
		adjusted_wrist_velocity = -15;
    else if( (adjusted_wrist_velocity <= -MIN_WRIST_SPEED) && (last_adjusted_wrist_velocity == -15) )
        adjusted_wrist_velocity = -15;
    else
        adjusted_wrist_velocity = 0;

     last_adjusted_wrist_velocity = adjusted_wrist_velocity;
    //if the gripper is fully open, the wrist joint is too
    //jammed to move
    /*if(REG_ARM_JOINT_POSITIONS.gripper >= GRIPPER_FULLY_OPEN)
      adjusted_wrist_velocity = 0;*/

    motor_accel_loop(adjusted_gripper_velocity, adjusted_wrist_velocity);
    //motor_accel_loop(0, REG_ARM_MOTOR_VELOCITIES.wrist);

    block_ms(10);
  }
  
  //get raw angle values from RS-485 message and ADC
  //apply the offset, and populate the REG_ARM_JOINT_POSITIONS registers
  update_joint_angles();

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
 /* if(message_index >= current_length)
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
  }*/

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
      if(rs485_link1_message_ready==0)
      {
        for(i=0;i<RS485_LINK1_LENGTH;i++)
        {
          rs485_link1_message[i] = rs485_rx_buffer[i];
        }
        rs485_link1_message_ready = 1;
      }
    }
    current_device = 0x00;
    message_index = 0;
    current_length = 100;
    rs485_rx_buffer[2] = 0x00;
    //return;
  }


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

//Don't factor in 
  if(desired_wrist_velocity > (wrist_velocity) )
    wrist_velocity+=step_size;
  else if (desired_wrist_velocity < (wrist_velocity) )
    wrist_velocity-=step_size;

  //don't allow speed to exceed desired speed
  if( (wrist_velocity > 0) && (wrist_velocity > desired_wrist_velocity) )
    wrist_velocity = desired_wrist_velocity;
  else if( (wrist_velocity < 0) && (wrist_velocity < desired_wrist_velocity) )
    wrist_velocity = desired_wrist_velocity;


//don't allow speed to exceed max settings
  if(gripper_velocity > MAX_GRIPPER_SPEED)
    gripper_velocity = MAX_GRIPPER_SPEED;
  else if(gripper_velocity < -MAX_GRIPPER_SPEED)
    gripper_velocity = -MAX_GRIPPER_SPEED;
  
  if(wrist_velocity > MAX_WRIST_SPEED)
    wrist_velocity = MAX_WRIST_SPEED;
  else if(wrist_velocity < -MAX_WRIST_SPEED)
    wrist_velocity = -MAX_WRIST_SPEED;


  if(abs(gripper_velocity) < MIN_GRIPPER_SPEED)
    set_gripper_velocity(0);
  else
    set_gripper_velocity(gripper_velocity);

  if(abs(wrist_velocity) < MIN_WRIST_SPEED)
    set_wrist_velocity(0);
  else
    set_wrist_velocity(wrist_velocity);


}

void calibrate_angle_sensor(void)
{

  //set all motor speeds to zero
  REG_ARM_MOTOR_VELOCITIES.turret = 0;
  REG_ARM_MOTOR_VELOCITIES.shoulder = 0;
  REG_ARM_MOTOR_VELOCITIES.elbow = 0;
  REG_ARM_MOTOR_VELOCITIES.wrist = 0;
  REG_ARM_MOTOR_VELOCITIES.gripper = 0;

  set_wrist_velocity(0);
  set_gripper_velocity(0);

  while(1)
  {
    send_rs485_message();
    //wait for message to finish sending
    while(rs485_transmitting);
    //wait for final byte to finish before changing to RX mode
    while(U1STAbits.TRMT == 0);
    RS485_OUTEN_ON(0);
    
    //clear receive overrun error, so receive doesn't stop
    U1STAbits.OERR = 0;
    ClrWdt();

    block_ms(100);
    //if turret and shoulder angles are read successfully, break from loop
    if(read_rs485_angle_values())
      break;

  }
  wrist_angle_offset = return_combined_pot_angle(WRIST_POT_1_CH, WRIST_POT_2_CH);
  turret_angle_offset = uncalibrated_turret_angle;
  shoulder_angle_offset = uncalibrated_shoulder_angle;
  elbow_angle_offset = return_combined_pot_angle(ELBOW_POT_1_CH, ELBOW_POT_2_CH);

  //don't calibrate if any of the angles are invalid
  if( (wrist_angle_offset == 0xffff) || (turret_angle_offset == 0xffff) || (shoulder_angle_offset == 0xffff) || (elbow_angle_offset == 0xffff) )
    return;

  DataEEInit();
  Nop();
  
  DataEEWrite( (turret_angle_offset>>8),0);
  Nop();
  DataEEWrite( (turret_angle_offset&0xff),1);
  Nop();

  DataEEWrite( (shoulder_angle_offset>>8),2);
  Nop();
  DataEEWrite( (shoulder_angle_offset&0xff),3);
  Nop();

  DataEEWrite( (elbow_angle_offset>>8),4);
  Nop();
  DataEEWrite( (elbow_angle_offset&0xff),5);
  Nop();

  DataEEWrite( (wrist_angle_offset>>8),6);
  Nop();
  DataEEWrite( (wrist_angle_offset&0xff),7);
  Nop();

  //write 0xaa to index 8, so we can tell if this arm has been calibrated yet
  DataEEWrite(0xaa,8);
  Nop();

  REG_ARM_MOTOR_VELOCITIES.turret = 20;
  send_rs485_message();
  block_ms(50);
  send_rs485_message();
  block_ms(50);
  ClrWdt();
  //block_ms(100);
  REG_ARM_MOTOR_VELOCITIES.turret = 0;
  while(1)
  {
    ClrWdt();
    send_rs485_message();
    block_ms(50);
  }

}
static unsigned int return_calibrated_angle(unsigned int uncalibrated_angle, unsigned int offset_angle, char direction)
{
  unsigned int calibrated_angle = 0;

  //if our angle reading is invalid, return an invalid value
  if(uncalibrated_angle == 0xffff)
    return 0xffff;

  //if our stored angle is bad, return 0xfffe
  if(offset_angle == 0xffff)
    return 0xfffe;

  //if stored angle is greater than 360 (should never happen) return 0xfffd
  if(offset_angle > 360)
    return 0xfffd;

  if(uncalibrated_angle < offset_angle)
  {
    calibrated_angle = 360+uncalibrated_angle-offset_angle;
  }
  else
  {
    calibrated_angle = uncalibrated_angle-offset_angle;
  }

  if(direction > 0)
    return calibrated_angle;
  else
    return 360-calibrated_angle;

  return calibrated_angle;

}

unsigned int return_combined_pot_angle(unsigned char pot_1_ch, unsigned char pot_2_ch)
{
  unsigned int pot_1_value, pot_2_value = 0;
  int combined_pot_angle = 0;
  int temp1 = 0;
  int temp2 = 0;
  float scale_factor = 0;
  int pot_1_angle, pot_2_angle;
  
  pot_1_value = return_adc_value(pot_1_ch);
  pot_2_value = 1023-return_adc_value(pot_2_ch);

  //333.3 degrees, 1023 total counts, 333.3/1023 = .326
  //13.35 degrees + 45 degrees = 58.35 degrees
  pot_1_angle = pot_1_value*.326+58.35;
  //wrap around if number is over 360
  if(pot_1_angle > 360)
    pot_1_angle-=360;

  //333.3 degrees, 1023 total counts, 333.3/1023 = .326
  pot_2_angle = pot_2_value*.326+13.35;


  //if both pots are invalid
  if( ( (pot_1_value < POT_LOW_THRESHOLD) || (pot_1_value > POT_HIGH_THRESHOLD) ) &&
    ( (pot_2_value < POT_LOW_THRESHOLD) || (pot_2_value > POT_HIGH_THRESHOLD) ) )
  {
    return 0xffff;
  }
  //if pot 1 is out of linear range
  else if( (pot_1_value < POT_LOW_THRESHOLD) || (pot_1_value > POT_HIGH_THRESHOLD) )
  {
    combined_pot_angle = pot_2_angle;
  }
  //if pot 2 is out of linear range
  else if( (pot_2_value < POT_LOW_THRESHOLD) || (pot_2_value > POT_HIGH_THRESHOLD) )
  {
    combined_pot_angle = pot_1_angle;
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
      //combined_pot_value = (pot_1_value*scale_factor + pot_2_value*(1-scale_factor));
      combined_pot_angle = (pot_1_angle*scale_factor + pot_2_angle*(1-scale_factor));

    }
    //if pot2 is closer to the end of range
    else
    {

      scale_factor = (512-abs(temp2) )/ 512.0;
 //     combined_pot_value = (pot_2_value*scale_factor + pot_1_value*(1-scale_factor));
      combined_pot_angle = (pot_2_angle*scale_factor + pot_1_angle*(1-scale_factor));

    }

    //333.3 degrees, 1023 total counts, 333.3/1023 = .326
    //combined_pot_angle = combined_pot_value*.326+13.35;

  }

  if(combined_pot_angle >= 360)
    combined_pot_angle-=360;
  else if(combined_pot_angle < 0)
    combined_pot_angle+=360;



  return (unsigned int)combined_pot_angle;


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
  static unsigned char gripper_clutch_state = 0;
  static int last_gripper_velocity = 0;
  static unsigned int last_gripper_pot_value = 0;
  static unsigned int gripper_close_slip_counter = 0;
  static unsigned int gripper_pos_sample_timer = 0;

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
      gripper_actuator_overtravel_direction = -1;
    }
  }
  //gripper open hard stop
  else if(REG_ARM_JOINT_POSITIONS.gripper_actuator > MAX_GRIPPER_ACT)
  {
    if(REG_ARM_MOTOR_VELOCITIES.gripper > 0)
    {
      adjusted_gripper_velocity = 0;
      gripper_direction_latch = 1;
      gripper_actuator_overtravel_direction = 1;
    }
  }
  else
  {
      gripper_actuator_overtravel_direction = 0;
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

  //Positive clutch slip moves clutch closer to the gripper, relative to the nut
  //(brass key closer to front of clutch)
  //Negative clutch slip moves clutch farther from the gripper, relative to the nut
  //(brass key closer to back of clutch)
  gripper_clutch_slip = adjusted_gripper_pot_value-REG_ARM_JOINT_POSITIONS.gripper_actuator;

  switch(gripper_clutch_state)
  {

    case NORMAL_OPERATION:
      gripper_clutch_overtravel_direction = 0;
      //if gripper is opening
      if(adjusted_gripper_velocity > 0)
      {
        //if the gripper is close to being fully open
        if(REG_ARM_JOINT_POSITIONS.gripper >= GRIPPER_FULLY_OPEN)
        {
          //if the clutch slip is too high, go into negative overslip reset and stop the motor
          if(gripper_clutch_slip <= -MAX_FULLY_OPEN_GRIPPER_SLIP)
          {  
            adjusted_gripper_velocity = 0;
            gripper_clutch_state = NEGATIVE_OVERSLIP_RESET;             
          
          }
        }
        //if the gripper isn't close to being fully open
        else
        {
          //if the gripper is opening (but not fully open), and there is too much negative slip, stop the motor
          if(gripper_clutch_slip <= -MAX_GRIPPER_SLIP)
          {  
            adjusted_gripper_velocity = 0;
            gripper_clutch_state = NEGATIVE_OVERSLIP_RESET;             
          
          }
        }
          
  
      }
      //if the gripper is closing, and there is too much positive slip, stop the motor
      else if( (adjusted_gripper_velocity < 0) && (gripper_clutch_slip >= MAX_GRIPPER_SLIP) )
      {
        adjusted_gripper_velocity = 0;
        gripper_clutch_state = POSITIVE_OVERSLIP_RESET;
      }
    break;

    //gripper has slipped too much when closing -- only allow the motor to opening the gripper,
    //unless the slip decreases below the maximum slip value, plus hysteresis
    case POSITIVE_OVERSLIP_RESET:
      gripper_clutch_overtravel_direction = 1;
      //if gripper is opening, stop motors when the clutch is centered
      if ( (adjusted_gripper_velocity > 0) && (gripper_clutch_slip <= 0))
      {
        adjusted_gripper_velocity = 0;
        gripper_clutch_state = POSITIVE_OVERSLIP_RESET_COMPLETE;
      }
      //only disallow closing the gripper if the slip is close to the max slip amount
      else if( (adjusted_gripper_velocity < 0) && (gripper_clutch_slip >= (MAX_GRIPPER_SLIP-GRIPPER_SLIP_HYSTERESIS) ) )
      //else if( (adjusted_gripper_velocity < 0) && (gripper_clutch_slip >=(MAX_GRIPPER_SLIP-GRIPPER_SLIP_HYSTERESIS) ) )
      {
        adjusted_gripper_velocity = 0;
      }

    break;
    case POSITIVE_OVERSLIP_RESET_COMPLETE:
      //wait for command to close the gripper before changing states
      //(even if this value is adjusted to zero for some reason
      if(input_gripper_velocity < 0)
      {
        gripper_clutch_state = NORMAL_OPERATION;
      }
      //don't allow the gripper the keep opening
      else if(adjusted_gripper_velocity > 0)
      {
        adjusted_gripper_velocity = 0;
      }
    break;
    //gripper has slipped too much when opening
    case NEGATIVE_OVERSLIP_RESET:
      gripper_clutch_overtravel_direction = -1;
      //if gripper is closing, stop motors when the clutch is centered
      /*if ( (adjusted_gripper_velocity < 0) && (gripper_clutch_slip >= 0))
      {
        //gripper_clutch_reset_in_progress = 0;
        adjusted_gripper_velocity = 0;
        //gripper_direction_latch = -1;
        gripper_clutch_state = NEGATIVE_OVERSLIP_RESET_COMPLETE;
      }
      //only disallow opening the gripper if the slip is close to the max slip amount
      else if( (adjusted_gripper_velocity > 0) && (gripper_clutch_slip <= (-MAX_GRIPPER_SLIP+GRIPPER_SLIP_HYSTERESIS) ) )
      {
        adjusted_gripper_velocity = 0;
      }*/
      if( (adjusted_gripper_velocity > 0) && (gripper_clutch_slip <= (-MAX_GRIPPER_SLIP+GRIPPER_SLIP_HYSTERESIS) ) )
      {
        adjusted_gripper_velocity = 0;
      }
      else if(input_gripper_velocity < 0)
      {
          gripper_clutch_state = NORMAL_OPERATION;
      }
    break;
    case NEGATIVE_OVERSLIP_RESET_COMPLETE:
      //wait for command to open the gripper before changing states
      //(even if this value is adjusted to zero for some reason
      if(input_gripper_velocity > 0)
      {
        gripper_clutch_reset_in_progress = 0;
        gripper_clutch_state = NORMAL_OPERATION;
      }
      //don't allow the gripper the keep closing
      else if(adjusted_gripper_velocity < 0)
      {
        adjusted_gripper_velocity = 0;
      }
    break;
    default:
      //logic error.  Stop motor and reset state machine
      adjusted_gripper_velocity = 0;
      gripper_clutch_state = NORMAL_OPERATION;
    break;
  }

  //let's run this slower, so the difference in gripper position is larger
  gripper_pos_sample_timer++;
  if(gripper_pos_sample_timer%10 == 0)
  {
    //if gripper is closing
    if(adjusted_gripper_velocity < 0)
    {
      
      if( abs((int)REG_ARM_JOINT_POSITIONS.gripper-(int)last_gripper_pot_value) < GRIPPER_CLOSE_SLIP_HYSTERESIS )
      {
        if(last_gripper_velocity < 0)
          gripper_close_slip_counter++;
      }
      else
      {
        gripper_close_slip_counter = 0;
      }
  
      if(gripper_close_slip_counter > 10)
      {
        adjusted_gripper_velocity = 0;
        gripper_direction_latch = -1;
      }
  
    }
    else if(input_gripper_velocity > 0)
    {
      gripper_close_slip_counter = 0;
    }
  
    last_gripper_pot_value = REG_ARM_JOINT_POSITIONS.gripper;
    last_gripper_velocity = adjusted_gripper_velocity;
    
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
  unsigned int i;

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

  REG_ARM_MOTOR_VELOCITIES.gripper = -20;
  while(1)
  {


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

    /*REG_ARM_MOTOR_VELOCITIES.gripper = -20;
    for(j=0;j<10;j++)
    {
      for(i=0;i<10;i++)
      {
        adjusted_gripper_velocity = return_adjusted_gripper_velocity();
    
        motor_accel_loop(adjusted_gripper_velocity, REG_ARM_MOTOR_VELOCITIES.wrist);
        //motor_accel_loop(0, REG_ARM_MOTOR_VELOCITIES.wrist);
    
        block_ms(10);
      }
    }*/

  }

}

int return_adjusted_wrist_velocity(void)
{
  int adjusted_wrist_velocity = 0;
  static unsigned int last_wrist_angle = 0;
  static unsigned int wrist_pos_sample_timer = 0;
  unsigned int wrist_angle = 0;
  static char wrist_direction_latch = 0;
  static int last_wrist_velocity = 0;
  static unsigned int wrist_stall_counter = 0;

  adjusted_wrist_velocity = REG_ARM_MOTOR_VELOCITIES.wrist;
  
  wrist_angle = return_combined_pot_angle(WRIST_POT_1_CH, WRIST_POT_2_CH);

  //if wrist is stopped, reset all counters
  if(adjusted_wrist_velocity < MIN_WRIST_SPEED)
    wrist_stall_counter = 0;
  
  //let's run this slower, so the difference in gripper position is larger
  wrist_pos_sample_timer++;
  if(wrist_pos_sample_timer > 10)
  {
    wrist_pos_sample_timer = 0;
    //if wrist is turning counterclockwise (from camera's perspective)
    if(adjusted_wrist_velocity < 0)
    {
      
      if( return_angle_difference(wrist_angle, last_wrist_angle) > WRIST_STALL_DEADBAND )
      {
        wrist_stall_counter = 0;
        last_wrist_angle = wrist_angle;
      }
      else
      {
        if(last_wrist_velocity < 0)
          wrist_stall_counter++;
      }
  
      if(wrist_stall_counter > 10)
      {
        adjusted_wrist_velocity = 0;
        wrist_direction_latch = -1;
        wrist_stall_counter = 0;
      }
  
    }
    //if wrist is turning clockwise (from camera's perspective)
    else if(adjusted_wrist_velocity > 0)
    {
     if( return_angle_difference(wrist_angle, last_wrist_angle) < -WRIST_STALL_DEADBAND )
      {
        wrist_stall_counter = 0;
        last_wrist_angle = wrist_angle;
      }
      else
      {
        if(last_wrist_velocity > 0)
          wrist_stall_counter++;

      }
  
      if(wrist_stall_counter > 10)
      {
        adjusted_wrist_velocity = 0;
        wrist_direction_latch = 1;
        wrist_stall_counter = 0;
      }
    }
  
    //last_wrist_angle = wrist_angle;
    last_wrist_velocity = adjusted_wrist_velocity;
    
  }


  //if the wrist stalls, don't move again until the direction is 
  //reversed
  if(wrist_direction_latch == 1)
  {
    if(adjusted_wrist_velocity > 0)
      adjusted_wrist_velocity = 0;
    else if(adjusted_wrist_velocity < 0)
      wrist_direction_latch = 0;
  }
  else if(wrist_direction_latch == -1)
  {
    if(adjusted_wrist_velocity < 0)
      adjusted_wrist_velocity = 0;
    else if(adjusted_wrist_velocity > 0)
      wrist_direction_latch = 0;
  }

  return adjusted_wrist_velocity; 
 

}

//implements angle1-angle2 to the closest angle
//i.e. angle1=350, angle2=0, difference would be 350 originally way, but -10 this way
//only really works if the angle difference is less than 180
int return_angle_difference(unsigned int angle_1, unsigned int angle_2)
{
  int angle_difference = 0;

  angle_difference = (int)angle_1-(int)angle_2;
  
  if(angle_difference > 180)
    angle_difference = angle_difference-360;
  else if(angle_difference < -180)
    angle_difference = angle_difference+360;

  return angle_difference;

}

static char read_rs485_angle_values(void)
{
  if(rs485_base_message_ready)
  {
    rs485_base_message_ready = 0;
    if(Is_CRC_valid(rs485_base_message,RS485_BASE_LENGTH))
    {
      if(rs485_base_message[3] == 0xff && rs485_base_message[4] == 0xff)
        uncalibrated_turret_angle = 0xffff;
      else
        uncalibrated_turret_angle = rs485_base_message[3]*256+rs485_base_message[4];

      if(rs485_base_message[5] == 0xff && rs485_base_message[6] == 0xff)
        uncalibrated_shoulder_angle = 0xffff;
      else
        uncalibrated_shoulder_angle = rs485_base_message[5]*256+rs485_base_message[6];

      return 1;
    }
    
  }
  return 0;
}


static void update_joint_angles(void)
{
  //parse message from base, and populate uncalibrated_X_angle variables
  read_rs485_angle_values();

  REG_ARM_JOINT_POSITIONS.wrist = return_calibrated_angle(return_combined_pot_angle(WRIST_POT_1_CH, WRIST_POT_2_CH),wrist_angle_offset,1);
  REG_ARM_JOINT_POSITIONS.turret = return_calibrated_angle(uncalibrated_turret_angle,turret_angle_offset,1);
  REG_ARM_JOINT_POSITIONS.shoulder = return_calibrated_angle(uncalibrated_shoulder_angle,shoulder_angle_offset,-1);
  REG_ARM_JOINT_POSITIONS.elbow = return_calibrated_angle(return_combined_pot_angle(ELBOW_POT_1_CH, ELBOW_POT_2_CH),elbow_angle_offset,-1);



}

//read stored values from flash memory
static void read_stored_angle_offsets(void)
{
  unsigned char angle_data[9];
  unsigned int i;
  DataEEInit();
  Nop();
  for(i=0;i<9;i++)
  {
    angle_data[i] = DataEERead(i);
    Nop();
  }

  //only use stored values if we have stored the calibrated values before.
  //we store 0xaa in position 8 so we know that the calibration has taken place.
  if(angle_data[8] == 0xaa)
  {
    turret_angle_offset = angle_data[0]*256+angle_data[1];
    shoulder_angle_offset = angle_data[2]*256+angle_data[3];
    elbow_angle_offset = angle_data[4]*256+angle_data[5];
    wrist_angle_offset = angle_data[6]*256+angle_data[7];
  }

}

//initialized uart to return debug information
static void enter_debug_mode(void)
{
  unsigned int i;
  unsigned char eeprom_flags = 0;
  unsigned char angle_data[9];

  init_debug_uart();

  while(1)
  {
    ClrWdt();

    update_joint_angles();

    send_rs485_message();
  
    //wait for message to finish sending
    while(rs485_transmitting);
    //wait for final byte to finish before changing to RX mode
    while(U1STAbits.TRMT == 0);
    RS485_OUTEN_ON(0);
  
    //clear receive overrun error, so receive doesn't stop
    U1STAbits.OERR = 0;
    
    display_int_in_dec("TUR               \r\n",REG_ARM_JOINT_POSITIONS.turret);
    block_ms(20);
    display_int_in_dec("SHO               \r\n",REG_ARM_JOINT_POSITIONS.shoulder);
    block_ms(20);
    display_int_in_dec("ELB               \r\n",REG_ARM_JOINT_POSITIONS.elbow);
    block_ms(20);
    display_int_in_dec("WRI               \r\n",REG_ARM_JOINT_POSITIONS.wrist);
    block_ms(20);


    ClrWdt();
    display_int_in_dec("TUR_OFF           \r\n",turret_angle_offset);
    block_ms(20);
    display_int_in_dec("SHO_OFF           \r\n",shoulder_angle_offset);
    block_ms(20);
    display_int_in_dec("ELB_OFF           \r\n",elbow_angle_offset);
    block_ms(20);
    display_int_in_dec("WRI_OFF         \r\n\r\n",wrist_angle_offset);
    block_ms(20);

  for(i=0;i<9;i++)
  {
    ClrWdt();

    angle_data[i] = DataEERead(i);
    Nop();

    eeprom_flags = 0;
    if(dataEEFlags.addrNotFound) eeprom_flags |= 0x80;
    if(dataEEFlags.expiredPage) eeprom_flags |= 0x40;
    if(dataEEFlags.packBeforePageFull) eeprom_flags |= 0x20;
    if(dataEEFlags.packBeforeInit) eeprom_flags |= 0x10;
    if(dataEEFlags.packSkipped) eeprom_flags |= 0x08;
    if(dataEEFlags.IllegalAddress) eeprom_flags |= 0x04;
    if(dataEEFlags.pageCorrupt) eeprom_flags |= 0x02;
    if(dataEEFlags.writeError) eeprom_flags |= 0x01;

    display_int_in_dec("EEREAD              ",i);
    block_ms(20);
    
    display_int_in_hex("EEFLAGS             ",eeprom_flags);
    block_ms(20);
    ClrWdt();
    display_int_in_dec("READ:             \r\n",angle_data[i]);
    block_ms(20);
  }
  for(i=0;i<20;i++)
  {
    ClrWdt();
    block_ms(100);
  }
  send_debug_uart_string("\r\n\r\n\r\n", 6);
  block_ms(20);

  }

}

