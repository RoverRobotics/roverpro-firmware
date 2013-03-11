#include "stdhdr.h"
#include "debug_uart.h"

unsigned char sending_lcd_string = 0;
unsigned int lcd_string_length = 0;
unsigned char lcd_string[100];


void init_debug_uart(void);
void debug_uart_tx_interrupt(void);
void int_to_string(unsigned int input, char* output);
void display_register_value(char* description);
void toggle_white_led(void);


void test_init(void)
{

}


void init_debug_uart(void)
{
	U1TXInterruptUserFunction = debug_uart_tx_interrupt;

 	// Write appropriate baud rate value to the UxBRG register.
	PGED2_RPn = U1TX_NUM;
	//UxBRG = Fcy/4/baud-1 = 16e6/4/38400
 	//U1BRG=104; //38400
	//U1BRG = 417; //9600
	U1BRG = 68; //57600
 	//Enable the UART.
 	U1MODE=0x0000;
 	//hight speed mode
 	U1MODEbits.BRGH=1;
 	U1STA=0x0000; 	
 	U1MODEbits.UARTEN=1;//UART1 is enabled
 	U1STAbits.UTXEN=1;//transmit enabled
 	_U1TXIF=0;//clear the transmit flag
 	_U1TXIE=1;//enable UART1 transmit interrupt
 	_U1RXIE=0;//enable UART1 receive interrupt

	ClrWdt();

	/*send_debug_uart_string("\r\nFirmware build:  ",19);
	block_ms(10);
	//send_debug_uart_string(REG_ROBOT_FIRMWARE_BUILD.data,24);
	block_ms(10);
	send_debug_uart_string("\r\n",2);
	block_ms(10);*/
	display_register_value("RCON              \r\n");
	

	
	ClrWdt();
	

}


void display_register_value(char* description)
{
	char string_to_display[20] = "                   \n";
	char register_string[6];
	unsigned int i;
	unsigned int register_value;

	for(i=0;i<20;i++)
	{
		string_to_display[i] = description[i];
	}

	register_value = RCON;
	//register_value = 0xabcd;

	int_to_string(register_value,(char*)register_string);

	for(i=0;i<6;i++)
	{
		string_to_display[i+10] = register_string[i];
	}

	send_debug_uart_string(string_to_display,20);

	//reset RCON
	RCON = 0x0000;


}

void display_int_in_hex(char* description, unsigned int int_to_display)
{
	char string_to_display[20] = "                   \n";
	char int_string[6];
	unsigned int i;
//	unsigned int register_value;

	for(i=0;i<20;i++)
	{
		string_to_display[i] = description[i];
	}

	//register_value = RCON;
	//register_value = 0xabcd;

	int_to_string(int_to_display,(char*)int_string);

	for(i=0;i<6;i++)
	{
		string_to_display[i+10] = int_string[i];
	}

	send_debug_uart_string(string_to_display,20);

}

void int_to_dec_string(unsigned int int_to_display,char* int_string)
{

  unsigned int digits[6];
  unsigned int i;
  char first_nonzero_flag = 0;

  digits[0] = int_to_display/1e5;
  digits[1] = int_to_display/1e4-digits[0]*10;
  digits[2] = int_to_display/1000 - digits[0]*100 - digits[1]*10;
  digits[3] = int_to_display/100 - digits[0]*1000 - digits[1]*100 - digits[2]*10;
  digits[4] = int_to_display/10 - digits[0]*1e4 - digits[1]*1000 - digits[2]*100 - digits[3]*10;
  digits[5] = int_to_display - digits[0]*1e5 - digits[1]*1e4 - digits[2]*1000 - digits[3]*100 - digits[4]*10;

  //apply ASCII offset
  for(i=0;i<6;i++)
  {
    if( (digits[i] == 0) && (i != 5) && (first_nonzero_flag==0) )
    {
      int_string[i] = ' ';
    }
    else
    {
      int_string[i] = digits[i]+48;
      first_nonzero_flag = 1;
    }
  }


}

void display_int_in_dec(char* description, unsigned int int_to_display)
{
	char string_to_display[20] = "                   \n";
	char int_string[6];
	unsigned int i;
//	unsigned int register_value;

	for(i=0;i<20;i++)
	{
		string_to_display[i] = description[i];
	}

	//register_value = RCON;
	//register_value = 0xabcd;

	int_to_dec_string(int_to_display,(char*)int_string);

	for(i=0;i<6;i++)
	{
		string_to_display[i+10] = int_string[i];
	}

	send_debug_uart_string(string_to_display,20);

}


void display_board_number(void)
{
	char board_number[5];
	unsigned int i;

	for(i=0;i<5;i++)
	{
		board_number[i] = REG_ROBOT_BOARD_DATA.data[i+18];
	}

	send_debug_uart_string("\r\nBoard number:  ",17);
	block_ms(10);
	send_debug_uart_string(board_number,5);
	block_ms(10);
	send_debug_uart_string("\r\n",2);
	block_ms(10);


}

void int_to_string(unsigned int input, char* output)
{
//	char output[6] = "0x0000";

	char nibble = '0';
	int i;

	output[0] = '0';
	output[1] = 'x';
	output[2] = '0';
	output[3] = '0';
	output[4] = '0';
	output[5] = '0';
	

	for(i=0;i<4;i++)
	{
		nibble='Z';
		switch(input&0x0f)
		{
			case 0x0a:
				nibble = 'A';
			break;
			case 0x0b:
				nibble = 'B';
			break;
			case 0x0c:
				nibble = 'C';
			break;
			case 0x0d:
				nibble = 'D';
			break;
			case 0x0e:
				nibble = 'E';
			break;
			case 0x0f:
				nibble = 'F';
			break;
			default:
				if( (input&0x0f) < 0x0a )
					nibble = (input&0x0f)+0x30;
				else
					nibble = 'Z';
			break;
		}
		output[5-i] = nibble;
		input=input>>4;
	}

}

void send_debug_uart_string(char* input_string, unsigned char len)
{
	unsigned int i;

	if(sending_lcd_string)
		return;


	for(i=0;i<len;i++)
	{
		if(lcd_string[i] == 0x7c)
			lcd_string[i] = 'x';
		if(lcd_string[i] == 0xfe)
			lcd_string[i] = 'y';
		lcd_string[i] = input_string[i];
	}

	lcd_string_length = len;

	sending_lcd_string = 1;

	_U1TXIE=1;
	U1TXREG = input_string[0];

}

void debug_uart_tx_interrupt(void)
{

	static unsigned int i = 1;

	_U1TXIF=0;

	if(i >= lcd_string_length)
	{
		i=1;
		_U1TXIE=0;
		sending_lcd_string = 0;
		return;
	}

	U1TXREG = lcd_string[i];
	i++;

}

void print_loop_number(void)
{
	static unsigned int loop = 0;
	unsigned int i;
	static unsigned int display_counter = 0;
	char loop_string[16] = "Loop          \r\n";
//	char loop_string[16] = "Loop            ";
//	toggle_white_led();

	char int_string[6];

	loop++;

	if(loop%100 != 0) return;



	int_to_string(display_counter,(char*)int_string);

	for(i=0;i<6;i++)
	{
		loop_string[i+6] = int_string[i];
	}

	send_debug_uart_string(loop_string,16);

	display_counter++;	

}

void toggle_white_led(void)
{
	static unsigned char white_led_on = 0;


	if(white_led_on)
	{
		white_led_on = 0;
		OC1R = 39990;
	}
	else
	{
		white_led_on = 1;
		OC1R = 20000;
	}

}
