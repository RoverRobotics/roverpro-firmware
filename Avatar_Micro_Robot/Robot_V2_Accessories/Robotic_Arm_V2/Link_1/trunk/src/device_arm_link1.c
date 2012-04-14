#include "device_arm_link1.h"
#include "stdhdr.h"

#define ELBOW_BRAKE_EN(a)   _TRISB15 = !a
#define ELBOW_BRAKE_ON(a)   _LATB15 = a

#define ELBOW_MODE_EN(a)    _TRISC13 = !a
#define ELBOW_MODE_ON(a)   _LATC13 = a

#define ELBOW_COAST_EN(a)   _TRISC14 = !a
#define ELBOW_COAST_ON(a)   _LATC14 = a

#define ELBOW_DIR_EN(a)     _TRISD0 = !a
#define ELBOW_DIR_ON(a)     _LATD0 = a

#define ELBOW_FF1()         _RB13
#define ELBOW_FF2()         _RB12
#define ELBOW_DIRO()        _RB14

#define ELBOW_PWM_OR        _RP19R

#define SHOULDER_BRAKE_EN(a)     _TRISD7 = !a
#define SHOULDER_BRAKE_ON(a)     _LATD7 = a

#define SHOULDER_MODE_EN(a)      _TRISD8 = !a
#define SHOULDER_MODE_ON(a)      _LATD8 = a

#define SHOULDER_COAST_EN(a)     _TRISD9 = !a
#define SHOULDER_COAST_ON(a)     _LATD9 = a

#define SHOULDER_DIR_EN(a)       _TRISD10 = !a
#define SHOULDER_DIR_ON(a)       _LATD10 = a

#define SHOULDER_FF1()         _RD2
#define SHOULDER_FF2()         _RD1
#define SHOULDER_DIRO()        _RD3

#define SHOULDER_PWM_OR        _RP21R

#define ELBOW_TEMP_EN(a)       _PCFG1 = !a
#define SHOULDER_TEMP_EN(a)   _PCFG2 = !a
#define THERMISTOR_EN(a)        _PCFG0 = !a

#define ELBOW_TEMP_CH         1
#define SHOULDER_TEMP_CH      2
#define THERMISTOR_CH         0

#define RS485_DI_OR           _RP20R
#define RS485_RO_PIN          25

#define RS485_OUTEN_EN(a)     _TRISD6 = !a
#define RS485_OUTEN_ON(a)     _LATD6 = a

#define RS485_TX_LENGTH 5
#define RS485_RX_LENGTH 10

#define RS485_BASE_LENGTH 9
#define RS485_LINK1_LENGTH 5
#define RS485_LINK2_LENGTH 10

#define MAX_ELBOW_SPEED 50
#define MAX_SHOULDER_SPEED 50

static void set_elbow_velocity(int velocity);
static void set_shoulder_velocity(int velocity);
static unsigned int return_adc_value(unsigned char ch);
static void RS485_RX_ISR(void);
static void RS485_TX_ISR(void);
static unsigned int return_CRC(unsigned char* data, unsigned char length);
static char Is_CRC_valid(unsigned char* data, unsigned char length);
static void send_rs485_message(void);
static void motor_accel_loop(int desired_shoulder_velocity, int desired_elbow_velocity);

int shoulder_motor_velocity, elbow_motor_velocity = 0;
static unsigned char rs485_tx_message[RS485_TX_LENGTH];
static unsigned char rs485_tx_buffer[RS485_TX_LENGTH];
static unsigned char rs485_rx_message[RS485_RX_LENGTH];
static unsigned char rs485_rx_buffer[RS485_RX_LENGTH];

static unsigned char rs485_base_message[RS485_BASE_LENGTH];
static unsigned char rs485_link1_message[RS485_LINK1_LENGTH];
static unsigned char rs485_link2_message[RS485_LINK2_LENGTH];

static unsigned char rs485_transmitting = 0;

static unsigned char rs485_link2_message_ready = 0;
static unsigned char rs485_link1_message_ready = 0;
static unsigned char rs485_base_message_ready = 0;

static unsigned int messaging_timeout_counter = 0;




void Arm_Link1_Init(void)
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

  ELBOW_BRAKE_ON(0);
  ELBOW_MODE_ON(0);
  ELBOW_COAST_ON(0);
  ELBOW_DIR_ON(0);
  ELBOW_DIR_ON(0);

  SHOULDER_BRAKE_ON(0);
  SHOULDER_MODE_ON(0);
  SHOULDER_COAST_ON(0);
  SHOULDER_DIR_ON(0);
  SHOULDER_DIR_ON(0);

  RS485_OUTEN_ON(0);

  ELBOW_BRAKE_EN(1);
  ELBOW_MODE_EN(1);
  ELBOW_COAST_EN(1);
  ELBOW_DIR_EN(1);
  ELBOW_DIR_EN(1);

  SHOULDER_BRAKE_EN(1);
  SHOULDER_MODE_EN(1);
  SHOULDER_COAST_EN(1);
  SHOULDER_DIR_EN(1);
  SHOULDER_DIR_EN(1);

  ELBOW_MODE_ON(1);
  ELBOW_DIR_ON(1);
  ELBOW_BRAKE_ON(1);

  SHOULDER_MODE_ON(1);
  SHOULDER_DIR_ON(1);
  SHOULDER_BRAKE_ON(1);



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


  ELBOW_PWM_OR = 18;	//OC1
	SHOULDER_PWM_OR	= 19;	//OC2

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

  set_elbow_velocity(0);
  set_shoulder_velocity(0);

  T2CONbits.TON = 1;

  ELBOW_COAST_ON(1);
  SHOULDER_COAST_ON(1);

}

void Link1_Process_IO(void)
{

  messaging_timeout_counter++;
 
//  rs485_base_message_ready = 1;
 // rs485_link2_message_ready = 1;


  //if the base has just transmitted
  if(rs485_base_message_ready)
  {
     //see if there was a link2 message -- if so, send it
    if(rs485_link2_message_ready)
    {
      rs485_base_message_ready = 0;
      rs485_link2_message_ready = 0;
      if(Is_CRC_valid(rs485_link2_message,RS485_LINK2_LENGTH))
      {
        if(rs485_link2_message[4] > 200)
          rs485_link2_message[4] = 100;
        if(rs485_link2_message[5] > 200)
          rs485_link2_message[5] = 100;     
        shoulder_motor_velocity = rs485_link2_message[4]-100;
        elbow_motor_velocity = rs485_link2_message[5]-100;
        messaging_timeout_counter = 0;
        send_rs485_message();
        
        //wait for message to finish sending
        while(rs485_transmitting);
        //wait for final byte to finish before changing to RX mode
        while(U1STAbits.TRMT == 0);
        //block_ms(2);
        RS485_OUTEN_ON(0);


      }
    }


  }

  //Just in case transmitting has just finished, switch direction of RS485
  /*if(rs485_transmitting == 0)
  {


  }*/


  //block_ms(500);


  //clear receive overrun error, so receive doesn't stop
  U1STAbits.OERR = 0;

  //pretty dumb control loop

  if(messaging_timeout_counter > 100)
  {
    messaging_timeout_counter = 101;
    motor_accel_loop(0, 0);
  }
  else
    motor_accel_loop(shoulder_motor_velocity, elbow_motor_velocity);
  
  block_ms(5);


 }

static void set_elbow_velocity(int velocity)
{

  if(velocity > 100)
    velocity = 100;
  else if(velocity < -100)
    velocity = -100;

  if(velocity > 0)
    ELBOW_DIR_ON(1);
  else
    ELBOW_DIR_ON(0);

  OC1R = abs(velocity)*20;


}

static void set_shoulder_velocity(int velocity)
{

  if(velocity > 100)
    velocity = 100;
  else if(velocity < -100)
    velocity = -100;

  if(velocity > 0)
    SHOULDER_DIR_ON(1);
  else
    SHOULDER_DIR_ON(0);

  OC2R = abs(velocity)*20;


}

static unsigned int return_adc_value(unsigned char ch)
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


static void RS485_RX_ISR(void)
{

  static unsigned char message_index = 0;
  static unsigned char current_length = 100;
  static unsigned char current_device = 0x00;
  unsigned char i;
  _U1RXIF = 0;

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
      else if( (rs485_rx_buffer[2] == 0x0d) )
      {
        current_length = RS485_LINK2_LENGTH;
        current_device = DEVICE_ARM_LINK2;
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
    else if(current_device == DEVICE_ARM_LINK2)
    {
      if(rs485_link2_message_ready==0)
      {
        for(i=0;i<RS485_LINK2_LENGTH;i++)
        {
          rs485_link2_message[i] = rs485_rx_buffer[i];
        }
        rs485_link2_message_ready = 1;
      }
    }
    current_device = 0x00;
    message_index = 0;
    current_length = 100;
    rs485_rx_buffer[2] = 0x00;
    //return;
  }

}


static void RS485_TX_ISR(void)
{
  static unsigned char message_index = 0;
  _U1TXIF = 0;
  
  //start at 1, since we already sent the first byte
  message_index++;

  if(message_index >= RS485_LINK1_LENGTH)
  {
    message_index = 0;
    _U1TXIE = 0;
    rs485_transmitting = 0;
    return;
  } 

  U1TXREG = rs485_tx_buffer[message_index];

}



static unsigned int return_CRC(unsigned char* data, unsigned char length)
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

static char Is_CRC_valid(unsigned char* data, unsigned char length)
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

static void send_rs485_message(void)
{
  unsigned char data_to_CRC[RS485_TX_LENGTH-4];
  unsigned int CRC;
  unsigned char i;

  rs485_transmitting = 1;
 /* for(i=0;i<rs485_tx_length-2;i++)
  {
    data_to_CRC 
  }*/
  block_ms(2);
  RS485_OUTEN_ON(1);
  block_ms(2);

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
  data_to_CRC[0] = DEVICE_ARM_LINK1;

  for(i=0;i<RS485_LINK1_LENGTH-4;i++)
  {
    rs485_tx_buffer[i+2] = data_to_CRC[i];
  }

  CRC = return_CRC(data_to_CRC,RS485_LINK1_LENGTH-4);


  rs485_tx_buffer[0] = 0xff;
  rs485_tx_buffer[1] = 0xcc;
  rs485_tx_buffer[RS485_LINK1_LENGTH-2] = CRC>>8;
  rs485_tx_buffer[RS485_LINK1_LENGTH-1] = CRC&0xFF;

  /*for(i=0;i<RS485_TX_LENGTH;i++)
  {
    rs485_tx_buffer[i] = rs485_tx_message[i];
  }*/

  U1TXREG = rs485_tx_buffer[0];
  _U1TXIE = 1;


}

static void motor_accel_loop(int desired_shoulder_velocity, int desired_elbow_velocity)
{
  static int shoulder_velocity;
  static int elbow_velocity;
  unsigned char step_size = 2;

  /*if( abs(desired_shoulder_velocity) < 15)
  {
    desired_shoulder_velocity = 0;
    shoulder_velocity = 0;
    set_shoulder_velocity(0);
  }
  if( abs(desired_elbow_velocity) < 15 )
  {
    desired_elbow_velocity = 0;
    elbow_velocity = 0;
    set_elbow_velocity(0);
  }*/

  if(desired_shoulder_velocity > (shoulder_velocity + step_size))
    shoulder_velocity+=step_size;
  else if (desired_shoulder_velocity < (shoulder_velocity - step_size) )
    shoulder_velocity-=step_size;

  if(desired_elbow_velocity > (elbow_velocity + step_size) )
    elbow_velocity+=step_size;
  else if (desired_elbow_velocity < (elbow_velocity - step_size) )
    elbow_velocity-=step_size;


  if(shoulder_velocity > MAX_SHOULDER_SPEED)
    shoulder_velocity = MAX_SHOULDER_SPEED;
  else if (shoulder_velocity < -MAX_SHOULDER_SPEED)
    shoulder_velocity = -MAX_SHOULDER_SPEED;

  if(elbow_velocity > MAX_ELBOW_SPEED)
    elbow_velocity = MAX_ELBOW_SPEED;
  if(elbow_velocity < -MAX_ELBOW_SPEED)
    elbow_velocity = -MAX_ELBOW_SPEED;


  if(abs(shoulder_velocity) < 15)
    set_shoulder_velocity(0);
  else
    set_shoulder_velocity(shoulder_velocity);

  if(abs(elbow_velocity) < 15)
    set_elbow_velocity(0);
  else
    set_elbow_velocity(elbow_velocity);


}

