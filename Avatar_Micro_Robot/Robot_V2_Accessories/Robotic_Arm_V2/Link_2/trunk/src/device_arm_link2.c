#include "device_arm_link2.h"
#include "stdhdr.h"

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

void set_gripper_velocity(int velocity);
void set_wrist_velocity(int velocity);
unsigned int return_adc_value(unsigned char ch);
void RS485_RX_ISR(void);
void RS485_TX_ISR(void);
unsigned int return_CRC(unsigned char* data, unsigned char length);
char Is_CRC_valid(unsigned char* data, unsigned char length);
void send_rs485_message(void);
void motor_accel_loop(int desired_gripper_velocity, int desired_wrist_velocity);

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

}

void Link2_Process_IO(void)
{

  unsigned int i;

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
    //don't move the gripper motor unless the potentiometer is in the correct range:
//    if( (gripper_act_pot_value > 300) && (gripper_act_pot_value < 700) )
      motor_accel_loop(REG_ARM_MOTOR_VELOCITIES.gripper, REG_ARM_MOTOR_VELOCITIES.wrist);
//    else
//      motor_accel_loop(0, REG_ARM_MOTOR_VELOCITIES.wrist);
    block_ms(10);
  }
  

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
  block_ms(50);
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

  if( abs(desired_gripper_velocity) < 15)
  {
    gripper_velocity = 0;
    set_gripper_velocity(0);
  }
  if( abs(desired_wrist_velocity) < 15 )
  {
    wrist_velocity = 0;
    set_wrist_velocity(0);
  }

  if(desired_gripper_velocity > (gripper_velocity + step_size))
    gripper_velocity+=step_size;
  else if (desired_gripper_velocity < (gripper_velocity - step_size) )
    gripper_velocity-=step_size;

  if(desired_wrist_velocity > (wrist_velocity + step_size) )
    wrist_velocity+=step_size;
  else if (desired_wrist_velocity < (wrist_velocity - step_size) )
    wrist_velocity-=step_size;


  set_gripper_velocity(gripper_velocity);
  set_wrist_velocity(wrist_velocity);


}

