#include "uart.h"
#include "./core/StandardHeader.h"
#include "home_office.h"

#define MAX_INCOMING_MESSAGE_LENGTH 7
#define MAX_OUTGOING_MESSAGE_LENGTH 7

//For Bluetooth control
#define BLE112_RX_PIN 19
//for XBee control
//#define BLE112_RX_PIN 6


#define UART_MOTOR_CMD      0x00
#define UART_MOTOR_CMD_LEN  7

static char Is_CRC_valid(unsigned char* data, unsigned char length);
static unsigned int return_CRC(unsigned char* data, unsigned char length);

static unsigned char BLE112_in_buffer[MAX_INCOMING_MESSAGE_LENGTH];
static unsigned char BLE112_in_message[MAX_INCOMING_MESSAGE_LENGTH];

static char BLE112_message_ready = 0;

char last_motor_commands[2] = {0,0};

unsigned char new_motor_control_message_flag = 0;


void init_uart(void)
{

	RPINR18bits.U1RXR = BLE112_RX_PIN;	
	
	U1BRG = 103;
	
	IEC0bits.U1RXIE = 1;
  //_U1TXIE = 1;
	
	U1MODEbits.UARTEN = 1;


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
  static unsigned char current_device = 0x00;
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
      if(BLE112_in_buffer[2] == UART_MOTOR_CMD)
      {
        current_length = UART_MOTOR_CMD_LEN;
      }
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



}