#include "device_arm_base.h"
#include "stdhdr.h"


#define POWER_BUS_EN(a)  (_TRISD11=!a)
#define POWER_BUS_ON(a)   (_LATD11=a)

#define TURRET_BRAKE_EN(a)     _TRISD7 = !a
#define TURRET_BRAKE_ON(a)     _LATD7 = a

#define TURRET_MODE_EN(a)      _TRISD8 = !a
#define TURRET_MODE_ON(a)      _LATD8 = a

#define TURRET_COAST_EN(a)     _TRISD9 = !a
#define TURRET_COAST_ON(a)     _LATD9 = a

#define TURRET_DIR_EN(a)       _TRISD10 = !a
#define TURRET_DIR_ON(a)       _LATD10 = a

#define TURRET_FF1()         _RD2
#define TURRET_FF2()         _RD1
#define TURRET_DIRO()        _RD3

#define TURRET_PWM_OR        _RP21R

#define RS485_DI_OR           _RP20R
#define RS485_RO_PIN          25

#define RS485_OUTEN_EN(a)     _TRISD6 = !a
#define RS485_OUTEN_ON(a)     _LATD6 = a

#define RS485_TX_LENGTH 9
#define RS485_RX_LENGTH 10

#define RS485_BASE_LENGTH 9
#define RS485_LINK1_LENGTH 5
#define RS485_LINK2_LENGTH 10

#define MAX_TURRET_SPEED 50

static void motor_accel_loop(int desired_turret_velocity);
static void set_turret_velocity(int velocity);
static void RS485_RX_ISR(void);
static void RS485_TX_ISR(void);
static void send_rs485_message(void);
static char Is_CRC_valid(unsigned char* data, unsigned char length);
static unsigned int return_CRC(unsigned char* data, unsigned char length);

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
int turret_motor_velocity;

void Arm_Base_Init(void)
{

  unsigned int i;

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

  TURRET_BRAKE_ON(0);
  TURRET_MODE_ON(0);
  TURRET_COAST_ON(0);
  TURRET_DIR_ON(0);
  TURRET_DIR_ON(0);

  RS485_OUTEN_ON(0);
  POWER_BUS_ON(0);

  TURRET_BRAKE_EN(1);
  TURRET_MODE_EN(1);
  TURRET_COAST_EN(1);
  TURRET_DIR_EN(1);
  TURRET_DIR_EN(1);

  POWER_BUS_EN(1);
  RS485_OUTEN_EN(1);

  TURRET_MODE_ON(1);
  TURRET_DIR_ON(1);
  TURRET_BRAKE_ON(1);

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

	TURRET_PWM_OR	= 18;	//OC1

	OC1R=0;
	OC1RS=2000;
	OC1CON2bits.SYNCSEL=0x1F;
	OC1CON2bits.OCTRIG=CLEAR;
	OC1CON1bits.OCTSEL=0b000;//Timer2
	OC1CON1bits.OCM=0b110;

  set_turret_velocity(0);
  T2CONbits.TON = 1;
  TURRET_COAST_ON(1);

  for(i=0;i<50;i++)
  {
    POWER_BUS_ON(1);
    block_ms(10);
    POWER_BUS_ON(0);
    block_ms(30);
  }
  POWER_BUS_ON(1);

}

void Base_Process_IO(void)
{


     messaging_timeout_counter++;
     //see if there was a link2 message -- if so, send it
    if(rs485_link2_message_ready)
    {
      rs485_link2_message_ready = 0;
      if(Is_CRC_valid(rs485_link2_message,RS485_LINK2_LENGTH))
      {

        if(rs485_link2_message[3] > 200)
          rs485_link2_message[3] = 100;
        turret_motor_velocity = rs485_link2_message[3]-100;
        messaging_timeout_counter = 0;

        send_rs485_message();
        
        //wait for message to finish sending
        while(rs485_transmitting);
        //wait for final byte to finish before changing to RX mode
        while(U1STAbits.TRMT == 0);
        //block_ms(2);
        RS485_OUTEN_ON(0);


      }
      else
      {
        Nop();
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
    motor_accel_loop(0);
    messaging_timeout_counter = 101;
  }
  else
    motor_accel_loop(turret_motor_velocity);
  
  block_ms(5);

}
static void set_turret_velocity(int velocity)
{

  if(velocity > 100)
    velocity = 100;
  else if(velocity < -100)
    velocity = -100;

  if(velocity > 0)
    TURRET_DIR_ON(1);
  else
    TURRET_DIR_ON(0);

  OC1R = abs(velocity)*20;


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

  if(message_index >= RS485_BASE_LENGTH)
  {
    message_index = 0;
    _U1TXIE = 0;
    rs485_transmitting = 0;
    return;
  } 

  U1TXREG = rs485_tx_buffer[message_index];

}

static void send_rs485_message(void)
{
  unsigned char data_to_CRC[RS485_BASE_LENGTH-4];
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


  //load motor velocities into array
  data_to_CRC[0] = DEVICE_ARM_BASE;
  data_to_CRC[1] = 0x01;
  data_to_CRC[2] = 0x02;
  data_to_CRC[3] = 0x03;
  data_to_CRC[4] = 0x04;

  for(i=0;i<RS485_BASE_LENGTH-4;i++)
  {
    rs485_tx_buffer[i+2] = data_to_CRC[i];
  }

  CRC = return_CRC(data_to_CRC,RS485_BASE_LENGTH-4);


  rs485_tx_buffer[0] = 0xff;
  rs485_tx_buffer[1] = 0xcc;
  rs485_tx_buffer[RS485_BASE_LENGTH-2] = CRC>>8;
  rs485_tx_buffer[RS485_BASE_LENGTH-1] = CRC&0xFF;

  /*for(i=0;i<RS485_TX_LENGTH;i++)
  {
    rs485_tx_buffer[i] = rs485_tx_message[i];
  }*/

  U1TXREG = rs485_tx_buffer[0];
  _U1TXIE = 1;


}

static void motor_accel_loop(int desired_turret_velocity)
{
  static int turret_velocity;

  unsigned char step_size = 2;

  if(desired_turret_velocity > (turret_velocity + step_size))
    turret_velocity+=step_size;
  else if (desired_turret_velocity < (turret_velocity - step_size) )
    turret_velocity-=step_size;

  if(turret_velocity > MAX_TURRET_SPEED)
     turret_velocity = MAX_TURRET_SPEED;
  else if(turret_velocity < -MAX_TURRET_SPEED)
    turret_velocity = -MAX_TURRET_SPEED;


  if(abs(turret_velocity) < 15)
    set_turret_velocity(0);
  else
    set_turret_velocity(turret_velocity);


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
