#include "system.h"

unsigned int return_CRC(unsigned char* data, unsigned char length);
//char Is_CRC_valid(unsigned char* data, unsigned char length);
//void send_message_to_robot(unsigned char* data_to_send);

//unsigned char message_to_robot[MESSAGE_TO_ROBOT_LENGTH] = {0,0,0,0,0,0,0,0};

//unsigned char message_from_robot[MESSAGE_FROM_ROBOT_LENGTH] = {0,0,0,0,0,0,0,0};

unsigned char previous_payload_rx[PREV_PAYLOAD_RX_LENGTH];
unsigned char previous_payload_tx[PREV_PAYLOAD_TX_LENGTH];
unsigned char next_payload_rx[NEXT_PAYLOAD_RX_LENGTH];
unsigned char next_payload_tx[NEXT_PAYLOAD_TX_LENGTH];

//unsigned char message_from_robot_ready = 0;
unsigned char prev_payload_msg_ready = 0;
unsigned char next_payload_msg_ready = 0;


//used by main function to pass messages
//load array with error values, so that joints won't move if there is a problem with the next node

//idle message for joints other than the base
unsigned char prev_payload_rx_last_good[PREV_PAYLOAD_RX_LENGTH] = {0xff, 0xcc, 0x00, 100, 100, 0x00, 0x00, 0x00, 0x00}; 


unsigned char next_payload_rx_last_good[NEXT_PAYLOAD_RX_LENGTH] = {0xff,0xcc,0x0f,0xff,0x0f,0xff,0x00,0x00,0x00};



unsigned char every_other_flag = 0;

void send_message_to_prev_payload(unsigned char* data_to_send)
{
	unsigned int CRC;
	unsigned char i;


	CRC = return_CRC(data_to_send,PREV_PAYLOAD_TX_LENGTH-4);

	previous_payload_tx[0] = 0xFF;
	previous_payload_tx[1] = 0xCC;

	previous_payload_tx[PREV_PAYLOAD_TX_LENGTH-2] = CRC>>8;
	previous_payload_tx[PREV_PAYLOAD_TX_LENGTH-1] = CRC&0xFF;
//	message_to_robot[MESSAGE_TO_ROBOT_LENGTH-2] = 0xaa;
//	message_to_robot[MESSAGE_TO_ROBOT_LENGTH-1] = 0xbb;
	

	for(i=0;i<PREV_PAYLOAD_TX_LENGTH-4;i++)
	{
		previous_payload_tx[i+2] = data_to_send[i];
		
	}

	U1TXREG = previous_payload_tx[0];
	IEC0bits.U1TXIE = 1;

}



void send_payload_ID(void)
{
	//this message will be for the hitch.  Right now it defaults to PTZ.
//	unsigned char id_data[5] = {0x04,0x07,0x00,0x00,0x00};
	unsigned char id_data[5] = {0x04,0x08,0x00,0x00,0x00};
	send_message_to_prev_payload(id_data);


}


void send_message_to_next_payload(unsigned char* data_to_send)
{
	unsigned int CRC;
	unsigned char i;

	CRC = return_CRC(data_to_send,NEXT_PAYLOAD_TX_LENGTH-4);

	next_payload_tx[0] = 0xFF;
	next_payload_tx[1] = 0xCC;

	

	next_payload_tx[NEXT_PAYLOAD_TX_LENGTH-2] = CRC>>8;
	next_payload_tx[NEXT_PAYLOAD_TX_LENGTH-1] = CRC&0xFF;
//	message_to_robot[MESSAGE_TO_ROBOT_LENGTH-2] = 0xaa;
//	message_to_robot[MESSAGE_TO_ROBOT_LENGTH-1] = 0xbb;
	

	for(i=0;i<NEXT_PAYLOAD_TX_LENGTH-4;i++)
	{
		next_payload_tx[i+2] = data_to_send[i];
		
	}

	U2TXREG = next_payload_tx[0];
	IEC1bits.U2TXIE = 1;

}


void init_usart1(void)
{

	//data, parity, stop bits

	U1MODEbits.UEN = 0x00;
	U1MODEbits.BRGH = 0;
	U1MODEbits.PDSEL = 0x00;
	U1MODEbits.STSEL = 0;

	
	//UART1 receive mapped to RP15
	RPINR18bits.U1RXR = 15;

	//RP14 mapped to UART1 TX
	RPOR7bits.RP14R = 3;


	//20e6/32/9600 -1 = 64.10
	U1BRG = SYSCLK/32/9600-1;


	//might need to check OERR bit at some point

	U1MODEbits.UARTEN = 1;


	//enable receive interrupts
	IEC0bits.U1RXIE = 1;


	U1STAbits.UTXEN = 1;
}

void init_usart2(void)
{
	
	//data, parity, stop bits

	U2MODEbits.UEN = 0x00;
	U2MODEbits.BRGH = 0;
	U2MODEbits.PDSEL = 0x00;
	U2MODEbits.STSEL = 0;

	
	//UART2 receive mapped to RP13
	RPINR19bits.U2RXR = 13;

	//RP14 mapped to UART2 TX
	RPOR6bits.RP12R = 5;

	//RP13 is RX

	//RP12 is TX


	//20e6/32/9600 -1 = 64.10
	U2BRG = SYSCLK/32/9600-1;


	//might need to check OERR bit at some point

	U2MODEbits.UARTEN = 1;


	//enable receive interrupts
	IEC1bits.U2RXIE = 1;


	U2STAbits.UTXEN = 1;
}


void  __attribute__((interrupt, auto_psv)) _U1TXInterrupt(void)
{

	static unsigned char message_index = 0;

	IFS0bits.U1TXIF = 0;								// Clear interrupt flag 
	
	
	message_index++;
	//if there are still 
	if(message_index < PREV_PAYLOAD_TX_LENGTH)
	{
		U1TXREG = previous_payload_tx[message_index];
	}
	//if the bytes are all sent, don't load TXREG.  This won't trigger the interrupt until TXREG is loaded again in an external function.
	else
	{
		message_index = 0;
		IEC0bits.U1TXIE = 0;
		
	}
}

//used for zoom module
/*void  __attribute__((interrupt, auto_psv)) _U1TXInterrupt(void)
{

	static unsigned char message_index = 0;

	IFS1bits.U2TXIF = 0;								// Clear interrupt flag 
	
	
	message_index++;
	//if there are still 
	if(message_index < PREV_PAYLOAD_TX_LENGTH)
	{
		U2TXREG = previous_payload_tx[message_index];
	}
	//if the bytes are all sent, don't load TXREG.  This won't trigger the interrupt until TXREG is loaded again in an external function.
	else
	{
		message_index = 0;
		IEC0bits.U2TXIE = 0;
		
	}
}*/




void  __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void)
{
	static int message_index = 0;
	unsigned char new_byte;
	static unsigned char message_buffer[PREV_PAYLOAD_RX_LENGTH];

	unsigned char i;

	IFS0bits.U1RXIF = 0;


	//
	//while(1)
	//{

/*	if(every_other_flag)
	{
		LATBbits.LATB9 = 1;
		every_other_flag = 0;

	}
	else
	{
		LATBbits.LATB9 = 0;
		every_other_flag = 1;

	}*/


	//}
	


	new_byte = U1RXREG;

	//U1STAbits.OERR = 0;
	message_buffer[message_index] = new_byte;

	//only allow the receipt of new messages if the old message has been read
	if(prev_payload_msg_ready == 0)
	{
		switch(message_index)
		{
			case 0x00:
				if(new_byte != 0xFF)
					message_index = -1;
				break;
			case 0x01:
				LATBbits.LATB5 = 1;
				if(new_byte != 0xCC)
					message_index = -1;
				break;
			case PREV_PAYLOAD_RX_LENGTH-1:
	
					for(i=0;i<PREV_PAYLOAD_RX_LENGTH;i++)
					{
						previous_payload_rx[i] = message_buffer[i];
					}
					prev_payload_msg_ready = 1;
					message_index = -1;
	
				break;
		}
		message_index++;
	}
			
				
}



void  __attribute__((interrupt, auto_psv)) _U2RXInterrupt(void)
{
	static int message_index = 0;
	unsigned char new_byte;
	static unsigned char message_buffer[NEXT_PAYLOAD_RX_LENGTH];

	unsigned char i;

	IFS1bits.U2RXIF = 0;



	new_byte = U2RXREG;


	message_buffer[message_index] = new_byte;

	//only allow the receipt of new messages if the old message has been read
	if(next_payload_msg_ready == 0)
	{
		switch(message_index)
		{
			case 0x00:
				if(new_byte != 0xFF)
					message_index = -1;
				break;
			case 0x01:
				if(new_byte != 0xCC)
					message_index = -1;
				break;
			case NEXT_PAYLOAD_RX_LENGTH-1:
	
					for(i=0;i<NEXT_PAYLOAD_RX_LENGTH;i++)
					{
						next_payload_rx[i] = message_buffer[i];
					}
					next_payload_msg_ready = 1;
					message_index = -1;
	
				break;
		}
		message_index++;
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
	unsigned char test_data[5];

	for(i=2;i<length-2;i++)
	{
		data_to_CRC[i-2] = data[i];
	}
	


	CRC_received = (data[length-2]<<8)+(data[length-1]);

	CRC_calculated = return_CRC(data_to_CRC,length-4);

	test_data[0] = CRC_received>>8;
	test_data[1] = CRC_received&0xFF;
	test_data[2] = CRC_calculated>>8;
	test_data[3] = CRC_calculated&0xFF;
	test_data[4] = 0xaa;

	//send_message_to_robot(test_data);



	if(CRC_received == CRC_calculated)
		return 1;
	
	return 0;


}
