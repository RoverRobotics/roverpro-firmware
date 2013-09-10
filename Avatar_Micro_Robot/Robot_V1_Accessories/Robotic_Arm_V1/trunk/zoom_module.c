#include "system.h"

unsigned char zoom_tx[ZOOM_TX_LENGTH];

void zoom_in(void)
{
	unsigned int i;
	zoom_tx[1] = 0x11;
	zoom_tx[2] = 0x00;
	for(i=0;i<ZOOM_TX_LENGTH;i++)
	{
		next_payload_tx[i] = zoom_tx[i];
	}
	U2TXREG = next_payload_tx[0];
	IEC1bits.U2TXIE = 1;
}


void zoom_out(void)
{
	unsigned int i;
	zoom_tx[1] = 0x11;
	zoom_tx[2] = 0x01;
	for(i=0;i<ZOOM_TX_LENGTH;i++)
	{
		next_payload_tx[i] = zoom_tx[i];
	}

	U2TXREG = next_payload_tx[0];
	IEC1bits.U2TXIE = 1;

}

void digital_zoom_on(void)
{
	unsigned int i;
	zoom_tx[1] = 0x1c;
	zoom_tx[2] = 0x02;
	zoom_tx[3] = 0x80;
	zoom_tx[4] = 0x88;
	for(i=0;i<ZOOM_TX_LENGTH;i++)
	{
		next_payload_tx[i] = zoom_tx[i];
	}

	U2TXREG = next_payload_tx[0];
	IEC1bits.U2TXIE = 1;



}

void zoom_stop(void)
{
	unsigned int i;
	zoom_tx[1] = 0x10;
	zoom_tx[2] = 0x00;
	for(i=0;i<ZOOM_TX_LENGTH;i++)
	{
		next_payload_tx[i] = zoom_tx[i];
	}

	U2TXREG = next_payload_tx[0];
	IEC1bits.U2TXIE = 1;

}

void init_zoom(void)
{
	zoom_tx[0] = 0xa0;
	zoom_tx[1] = 0x11;
	zoom_tx[2] = 0x00;
	zoom_tx[3] = 0x05;
	zoom_tx[4] = 0x00;
	zoom_tx[5] = 0xaf;

	//data, parity, stop bits

	U2MODEbits.UEN = 0x00;
	U2MODEbits.BRGH = 0;
	U2MODEbits.PDSEL = 0x00;
	U2MODEbits.STSEL = 0;

	
	//UART2 receive mapped to RP16
	//RPINR19bits.U2RXR = 16;

	//RP16 mapped to UART2 TX
	RPOR8bits.RP16R = 5;

	//RP13 is RX

	//RP12 is TX


	//20e6/32/9600 -1 = 64.10
	U2BRG = SYSCLK/32/38400-1;


	//might need to check OERR bit at some point

	U2MODEbits.UARTEN = 1;


	//enable receive interrupts
	IEC1bits.U2RXIE = 1;


	U2STAbits.UTXEN = 1;
	

}
