#include "debug_uart.h"
#include "interrupt_switch.h"
#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"

void int_to_string(unsigned int input, char* output);
void debug_uart_tx_interrupt(void);
unsigned char sending_debug_uart_string = 0;
unsigned int debug_uart_string_length = 0;
unsigned char debug_uart_string[100];

void init_debug_uart(void)
{
	U1TXInterruptUserFunction = debug_uart_tx_interrupt;

 	// Write appropriate baud rate value to the UxBRG register.
	U1TX_RPn = 3;
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
 	IFS0bits.U1TXIF=0;//clear the transmit flag
 	IEC0bits.U1TXIE=1;//enable UART1 transmit interrupt
 	IEC0bits.U1RXIE=0;//enable UART1 receive interrupt

	ClrWdt();

	send_debug_uart_string("\r\nFirmware build:  ",19);
	block_ms(10);
	send_debug_uart_string(REG_MOTOR_FIRMWARE_BUILD.data,24);
	block_ms(10);
	send_debug_uart_string("\r\n",2);
	block_ms(10);
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

	if(sending_debug_uart_string)
		return;


	for(i=0;i<len;i++)
	{
		if(debug_uart_string[i] == 0x7c)
			debug_uart_string[i] = 'x';
		if(debug_uart_string[i] == 0xfe)
			debug_uart_string[i] = 'y';
		debug_uart_string[i] = input_string[i];
	}

	debug_uart_string_length = len;

	sending_debug_uart_string = 1;

	IEC0bits.U1TXIE=1;
	U1TXREG = input_string[0];

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

void debug_uart_tx_interrupt(void)
{

	static unsigned int i = 1;

	IFS0bits.U1TXIF=0;

	if(i >= debug_uart_string_length)
	{
		i=1;
		IEC0bits.U1TXIE=0;
		sending_debug_uart_string = 0;
		return;
	}

	U1TXREG = debug_uart_string[i];
	i++;

}