#include "stdhdr.h"
#include "testing.h"

unsigned char sending_lcd_string = 0;
unsigned int lcd_string_length = 0;
unsigned char lcd_string[100];


void init_lcd_uart(void);
void lcd_tx_interrupt(void);
void int_to_string(unsigned int input, char* output);
void display_register_value(char* description);

void test_init(void)
{
	init_lcd_uart();
}


void init_lcd_uart(void)
{
	U1TXInterruptUserFunction = lcd_tx_interrupt;

 	// Write appropriate baud rate value to the UxBRG register.
	U1TX_RPn = 3;
	//UxBRG = Fcy/4/baud-1 = 16e6/4/38400
 	U1BRG=104; //38400
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

	send_lcd_string("Initializing LCD UART  ",24);
	block_ms(30);
	display_register_value("RCON                ");
	

	

	

}


void display_register_value(char* description)
{
	char string_to_display[20] = "                    ";
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
		string_to_display[i+12] = register_string[i];
	}

	send_lcd_string(string_to_display,20);

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

void send_lcd_string(char* input_string, unsigned char len)
{
	unsigned int i;

	if(sending_lcd_string)
		return;


	for(i=0;i<len;i++)
	{
		lcd_string[i] = input_string[i];
	}

	lcd_string_length = len;

	sending_lcd_string = 1;

	IEC0bits.U1TXIE=1;
	U1TXREG = input_string[0];

}

void lcd_tx_interrupt(void)
{

	static unsigned int i = 1;

	IFS0bits.U1TXIF=0;

	if(i >= lcd_string_length)
	{
		i=1;
		IEC0bits.U1TXIE=0;
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
	char loop_string[14] = "Loop          ";

	char int_string[6];

	loop++;

	if(loop%100 != 0) return;



	int_to_string(display_counter,(char*)int_string);

	for(i=0;i<6;i++)
	{
		loop_string[i+6] = int_string[i];
	}

	send_lcd_string(loop_string,14);

	display_counter++;	

}
