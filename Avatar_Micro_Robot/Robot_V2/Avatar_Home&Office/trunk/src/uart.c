#include "uart.h"
#include "./core/StandardHeader.h"
#include "home_office.h"
#include "charging.h"
#include "DEE/DEE Emulation 16-bit.h"

#define MAX_INCOMING_MESSAGE_LENGTH 9
#define MAX_OUTGOING_MESSAGE_LENGTH 9

#define MESSAGE_BATTERY_INFO 0x01
#define MESSAGE_SERIAL_NUMBER   0x07
#define MESSAGE_SERIAL_NUM_LEN  9

//For Bluetooth control
#define BLE112_RX_PIN 19
//for XBee control
//#define BLE112_RX_PIN 6
#define BLE112_TX_OR _RP27R


#define UART_MOTOR_CMD      0x00

#define UART_MOTOR_CMD_LEN  7

#define ROBOT_ID_REQ        0x08
#define ROBOT_ID_REQ_LEN    5

#define ROBOT_ID_SET        0x06
#define ROBOT_ID_SET_LEN    9

static char Is_CRC_valid(unsigned char* data, unsigned char length);
static unsigned int return_CRC(unsigned char* data, unsigned char length);

static unsigned long return_robot_id(void);
static void send_robot_id(void);
static void set_robot_id(unsigned char b1, unsigned char b2, unsigned char b3, unsigned char  b4);

static unsigned char BLE112_in_buffer[MAX_INCOMING_MESSAGE_LENGTH];
static unsigned char BLE112_in_message[MAX_INCOMING_MESSAGE_LENGTH];
static unsigned char BLE112_tx_buffer[MAX_OUTGOING_MESSAGE_LENGTH];

static char BLE112_message_ready = 0;

static unsigned int BLE112_tx_message_length = 0;
static unsigned int BLE112_transmitting = 0; 

char last_motor_commands[2] = {0,0};

unsigned char new_motor_control_message_flag = 0;
static unsigned char send_robot_id_flag = 0;


void init_uart(void)
{

	RPINR18bits.U1RXR = BLE112_RX_PIN;
  BLE112_TX_OR = 3;
	
	U1BRG = 103;
	


	_U1RXIE = 1;
  _U1TXIE = 0;
	
	U1MODEbits.UARTEN = 1;

  U1STAbits.UTXEN = 1;

  


}

void messaging_FSM(void)
{
  static unsigned int battery_message_counter = 0;
  parse_UART_message();

  if(BLE112_transmitting)
    return;


  if(send_robot_id_flag)
  {
    send_robot_id();
    send_robot_id_flag = 0;
    return;
  }

  if(battery_message_counter >= 100)
  {
    battery_message_counter = 0;
    send_battery_message();
  }

  battery_message_counter++;

}

void parse_UART_message(void)
{
  if(BLE112_message_ready)
  {
    if(BLE112_in_message[2] == UART_MOTOR_CMD)
    {
      if(Is_CRC_valid(BLE112_in_message,UART_MOTOR_CMD_LEN))
      {
        if( (BLE112_in_message[3] <= 200) && (BLE112_in_message[4] <= 200) )
        {
          last_motor_commands[0] = 100-BLE112_in_message[3];
          last_motor_commands[1] = BLE112_in_message[4]-100;
          new_motor_control_message_flag = 1;
        }

      }
    }
    else if(BLE112_in_message[2] == ROBOT_ID_REQ)
    {
      if(Is_CRC_valid(BLE112_in_message,ROBOT_ID_REQ_LEN))
      {
        send_robot_id_flag = 1;
      }
    }
    else if(BLE112_in_message[2] == ROBOT_ID_SET)
    {
      set_robot_id(BLE112_in_message[3],BLE112_in_message[4],BLE112_in_message[5],BLE112_in_message[6]);
    }
    BLE112_message_ready = 0;

  }



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

void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void) {

  static unsigned char message_index = 0;
  static unsigned char current_length = 100;
  unsigned char i;
  _U1RXIF = 0;
  BLE112_in_buffer[message_index] = U1RXREG;

  switch(message_index)
  {
    case 0x00:
      if(BLE112_in_buffer[0] != 0xff)
      {
        message_index = 0;
        return;
      }
    break;
    case 0x01:
//    If the second byte isn't 0xcc
      if(BLE112_in_buffer[1] != 0xcc)
      {
//      If it's actually 0xff, try starting message with this byte
//      (so we don't miss a valid message preceded by 0xff
        if(BLE112_in_buffer[1] == 0xff)
        {
          BLE112_in_buffer[0] = 0xff;
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
      switch(BLE112_in_buffer[2]) {
        case UART_MOTOR_CMD:
          current_length = UART_MOTOR_CMD_LEN;
        break;
        case ROBOT_ID_REQ:
          current_length = ROBOT_ID_REQ_LEN;
        break;
        case ROBOT_ID_SET:
          current_length = ROBOT_ID_SET_LEN;
        break;
        default:
          message_index = 0;
        break;
      }
      break;

  }

  message_index++;

 //if we've gotten to the end of a message
  if(message_index >= current_length)
  {

    if(BLE112_message_ready == 0)
    {
      for(i=0;i<current_length;i++)
      {
        BLE112_in_message[i] = BLE112_in_buffer[i];
      }
  
      message_index = 0;
      current_length = 100;
      BLE112_message_ready = 1;
    }
  }

}

void send_battery_message(void)
{
  unsigned int CRC = 0;

  if(BLE112_transmitting)
    return;

  BLE112_tx_message_length = 7;

  BLE112_tx_buffer[0] = 0xff;
  BLE112_tx_buffer[1] = 0xcc;
  BLE112_tx_buffer[2] = MESSAGE_BATTERY_INFO;
  BLE112_tx_buffer[3] = return_charging_state();
  BLE112_tx_buffer[4] = return_battery_meter();;
  
  CRC = return_CRC(BLE112_tx_buffer,7-4);
  
  BLE112_tx_buffer[5] = CRC>>8;
  BLE112_tx_buffer[6] = CRC&0xff;

  U1TXREG = BLE112_tx_buffer[0];
  _U1TXIE = 1;

}

static void send_robot_id(void)
{
  unsigned int CRC = 0;
  unsigned long robot_id = 0;



  robot_id = return_robot_id();

  BLE112_tx_message_length = MESSAGE_SERIAL_NUM_LEN;

  BLE112_tx_buffer[0] = 0xff;
  BLE112_tx_buffer[1] = 0xcc;
  BLE112_tx_buffer[2] = MESSAGE_SERIAL_NUMBER;
  BLE112_tx_buffer[3] = (robot_id>>24) & 0xff;
  BLE112_tx_buffer[4] = (robot_id>>16) & 0xff;
  BLE112_tx_buffer[5] = (robot_id>>8) & 0xff;
  BLE112_tx_buffer[6] = robot_id&0xff;
  
  CRC = return_CRC(BLE112_tx_buffer,MESSAGE_SERIAL_NUM_LEN-4);
  
  BLE112_tx_buffer[7] = CRC>>8;
  BLE112_tx_buffer[8] = CRC&0xff;

  U1TXREG = BLE112_tx_buffer[0];
  _U1TXIE = 1;

}

static void set_robot_id(unsigned char b1, unsigned char b2, unsigned char b3, unsigned char  b4)
{

    unsigned char data_bytes[6];
    unsigned int CRC = 0;
    unsigned int i;

    data_bytes[0] = b1;
    data_bytes[1] = b2;
    data_bytes[2] = b3; 
    data_bytes[3] = b4;

    CRC = return_CRC(data_bytes,4);
  
    data_bytes[4] = (CRC>>8)&0xff;
    data_bytes[5] = CRC&0xff;

    DataEEInit();
    dataEEFlags.val = 0;

    Nop();
    Nop();

    for(i=0;i<6;i++)
    {
      DataEEWrite(data_bytes[i],i);
      Nop();
      Nop();
    }



}

static unsigned long return_robot_id(void)
{
  unsigned long robot_id = 0;
  unsigned int i;
  unsigned char bytes_for_CRC[8];
  unsigned char new_byte = 0;

  DataEEInit();
  dataEEFlags.val = 0;

  Nop();
  Nop();

  for(i=0;i<4;i++)
  {
    new_byte = DataEERead(i);
    robot_id = (robot_id<<8)+new_byte;
    bytes_for_CRC[i+2] = new_byte;
    //robot_id=DataEERead(3);
    Nop();
    Nop();
  }

  for(i=4;i<6;i++)
  {
    bytes_for_CRC[i+2] = DataEERead(i);
    Nop();
    Nop();
  }

//  return 3198;

  if(Is_CRC_valid(bytes_for_CRC,8))
  {
    return robot_id;
  }
  else
  {
    return 0xffffffff;
  }
}

void __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void) {

  static unsigned char message_index = 0;
  _U1TXIF = 0;
  
  //start at 1, since we already sent the first byte
  message_index++;

  if(message_index >= BLE112_tx_message_length)
  {
    message_index = 0;
    _U1TXIE = 0;
    BLE112_transmitting = 0;
    return;
  } 

  U1TXREG = BLE112_tx_buffer[message_index];

}
