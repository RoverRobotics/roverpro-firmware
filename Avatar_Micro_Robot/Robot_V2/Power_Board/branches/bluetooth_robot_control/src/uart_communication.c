#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "uart_communication.h"

#define U1RX_MESSAGE_LENGTH 7

void uart_control_tx_interrupt(void);
void uart_control_rx_interrupt(void);
unsigned int return_CRC(unsigned char* data, unsigned char length);
char Is_CRC_valid(unsigned char* data, unsigned char length);

unsigned char uart_message_in_buffer[20];
unsigned char uart_message_in[20];
unsigned char uart_message_out[20];

static unsigned char new_message_available = 0;

void init_uart_control(void)
{

  unsigned int i;

  for(i=0;i<20;i++)
  {
    uart_message_in[i] = 0;
  }

	U1TXInterruptUserFunction = uart_control_tx_interrupt;
  U1RXInterruptUserFunction = uart_control_rx_interrupt;
  uart_message_out[0] = 0x61;
  uart_message_out[1] = 0x61;
  uart_message_out[2] = 0x61;

	U1TX_RPn = 3;
	RPINR18bits.U1RXR = U1RX_RPn;
  

 	U1BRG=103;
 	//Enable the UART.
 	U1MODE=0x0000;
 	//hight speed mode
 	U1MODEbits.BRGH=0;
 	U1STA=0x0000; 	
 	U1MODEbits.UARTEN=1;//UART1 is enabled
 	U1STAbits.UTXEN=1;//transmit enabled
 	IFS0bits.U1TXIF=0;//clear the transmit flag
 	IEC0bits.U1TXIE=0;//enable UART1 transmit interrupt
 	IEC0bits.U1RXIE=1;//enable UART1 receive interrupt

}

void uart_control_tx_interrupt(void)
{
	_U1TXIF = 0;
  static unsigned char index = 1;
  
  if(index >= 20)
  {
    index = 1;
    return;
  }

  U1TXREG = uart_message_out[index];
 
  index++;


}

void uart_control_rx_interrupt(void)
{
  static unsigned char index = 0;
  unsigned char new_byte;
  unsigned int i;

	U1STAbits.OERR = 0;

	_U1RXIF = 0;

  new_byte = U1RXREG;

  if(new_message_available) return;

  if(index == 0)
  {
    if(new_byte != 0xff)
    {
      index = 0;
      return;
    }
  }
  else if(index == 1)
  {
    if(new_byte != 0xcc)
    {
      index = 0;
      return;
    }
  }
  
  uart_message_in_buffer[index] = new_byte;

  if(index >= U1RX_MESSAGE_LENGTH-1)
  {

    if(Is_CRC_valid(uart_message_in_buffer, U1RX_MESSAGE_LENGTH))
    {
      for(i=0;i<U1RX_MESSAGE_LENGTH;i++)
      {
        uart_message_in[i] = uart_message_in_buffer[i];
      }
      new_message_available = 1;
    }
    index = 0;

  }
  else
  {
    index++;
  }
  
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

void handle_uart_communication(void)
{
  static unsigned int message_timeout_counter = 0;
  static unsigned int i;

  message_timeout_counter++;
  if(message_timeout_counter > 333)
  {
    REG_MOTOR_VELOCITY.left = 0;
    REG_MOTOR_VELOCITY.right = 0;
    REG_MOTOR_VELOCITY.flipper = 0;

  }
  if(new_message_available)
  {
   /* for(i=0;i<20;i++)
    {
      uart_message_out[i] = uart_message_in[i];
    }

  U1TXREG = uart_message_out[0];*/

  new_message_available = 0;
  message_timeout_counter = 0;
  REG_MOTOR_VELOCITY.left = ((signed int)uart_message_in[2]-100)*10;
  REG_MOTOR_VELOCITY.right = ((signed int)uart_message_in[3]-100)*10;
  REG_MOTOR_VELOCITY.flipper = ((signed int)uart_message_in[4]-100)*10;
  }


}
