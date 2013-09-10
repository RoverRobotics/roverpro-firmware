/*==============================================================================
File: TestCode.c
==============================================================================*/
/*---------------------------Dependencies-------------------------------------*/
#include "./StandardHeader.h" // for Delay() function
#include "./UART.h"
#include "./TestCode.h"

/*---------------------------Macros-------------------------------------------*/
#define U1TX_RPn          _RP7R

/*---------------------------Helper Function Prototypes-----------------------*/
static void LCD_Tx_ISR(void);
static void IntToString(unsigned int input, char *output);

/*---------------------------Module Variables---------------------------------*/
unsigned char sending_lcd_string = 0;
unsigned int lcd_string_length = 0;
unsigned char lcd_string[100];

/*---------------------------Public Function Definitions----------------------*/
void InitTestCode(void) {
	// initialize the UART module
	U1TX_UserISR = LCD_Tx_ISR;
  UART_Init(U1TX_RPn, 0, kUARTBaudRate57600);
	
  // print hardware information
  ClrWdt();
	SendLCDString("\r\nFirmware build:  ", 19);
	Delay(10);
	SendLCDString((char *)REG_ROBOT_FIRMWARE_BUILD.data, 24);
	Delay(10);
	SendLCDString("\r\n",2);
	Delay(10);
	//SendLCDString("RCON              \r\n");
	ClrWdt();
}


void DisplayInt(char *description, unsigned int intToDisplay) {
	char stringToDisplay[20] = "                   \n";
	char intString[6];

	unsigned int i;
	for (i = 0; i < 20; i++) stringToDisplay[i] = description[i];

	IntToString(intToDisplay,(char *)intString);

	for (i = 0; i < 6; i++) stringToDisplay[i+10] = intString[i];
	
	SendLCDString(stringToDisplay, 20);
}


void DisplayBoardNumber(void) {
	char boardNumber[5];
	unsigned int i;

	for (i = 0; i < 5; i++) boardNumber[i] = REG_ROBOT_BOARD_DATA.data[i+18];

	SendLCDString("\r\nBoard number:  ", 17);
	Delay(10);
	SendLCDString(boardNumber, 5);
	Delay(10);
	SendLCDString("\r\n", 2);
	Delay(10);
}


void SendLCDString(char inputString[], unsigned char length) {
	if (sending_lcd_string) return;
  // will this work?: unsigned int length = sizeof(inputString) / sizeof(char);
  unsigned int i;
	for (i = 0; i < length; i++) {
		if (lcd_string[i] == 0x7c) lcd_string[i] = 'x';
		if (lcd_string[i] == 0xfe) lcd_string[i] = 'y';
		lcd_string[i] = inputString[i];
	}

	lcd_string_length = length;
	sending_lcd_string = 1;

	IEC0bits.U1TXIE=1;
	U1TXREG = inputString[0];
}


void PrintLoopNumber(void) {
	static unsigned int loop = 0;
	unsigned int i;
	static unsigned int displayCounter = 0;
	char loopString[16] = "Loop          \r\n";
	char intString[6];

	loop++;

	if ((loop % 100) != 0) return;

	IntToString(displayCounter, (char*)intString);
	for (i = 0; i < 6; i++) loopString[i + 6] = intString[i];

	SendLCDString(loopString, 16);
	displayCounter++;	
}


/*---------------------------Helper Function Definitions----------------------*/
static void LCD_Tx_ISR(void) {
	IFS0bits.U1TXIF = 0;

	static unsigned int i = 1;
	if (lcd_string_length <= i) {
		i = 1;
		IEC0bits.U1TXIE = 0;
		sending_lcd_string = 0;
		return;
	}

	U1TXREG = lcd_string[i];
	i++;
}


static void IntToString(unsigned int input, char *output) {
	output[0] = '0';
	output[1] = 'x';
	output[2] = '0';
	output[3] = '0';
	output[4] = '0';
	output[5] = '0';
	
	char nibble = '0';
	int i;
	for (i = 0; i < 4; i++) {
		nibble='Z';
		switch(input&0x0f) {
			case 0x0a: nibble = 'A'; break;
			case 0x0b: nibble = 'B'; break;
			case 0x0c: nibble = 'C'; break;
			case 0x0d: nibble = 'D'; break;
			case 0x0e: nibble = 'E'; break;
			case 0x0f: nibble = 'F'; break;
			default:
				if ((input & 0x0f) < 0x0a) nibble = (input&0x0f) + 0x30;
				else nibble = 'Z';
			  break;
		}
		output[5 - i] = nibble;
		input = input >> 4;
	}
}
