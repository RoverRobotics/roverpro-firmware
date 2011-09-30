//#include "include\Compiler.h"
#include "p24fxxxx.h"
_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_ON & ICS_PGx2 & WDTPS_PS16384) 
_CONFIG2(IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRI & IOL1WAY_OFF)


#include "system.h"


void block_ms(unsigned int ms)
{
	unsigned int i;
	unsigned int j;
	for(i=0;i<2000;i++)
	{

		for(j=0;j<ms;j++)
		{
		}
	}



}


int main() {

	unsigned char calibrate_message[5] = {0xff,0x00,0x00,0x00,0x00};
	unsigned char i;
	unsigned int hitch_open_counter = 0;
	unsigned int hitch_locked_open_counter = 0;
	unsigned char hitch_locked_open_latch = 0;
	unsigned int payload_id_counter = 0;
//	unsigned char hitch_last_open = 0;
//	int servo_lockout = 0;


//	these arrays are used as a buffer.  We might not need these, if we
//	can use msg_ready flags intelligently (clear them once we've read the message).
//	We can make message receipt faster if we do buffer them (so that the interrupt
//	Isn't waiting forever for the flag to be cleared.
	unsigned char prev_payload_rx_buffer[PREV_PAYLOAD_RX_LENGTH];
	unsigned char next_payload_rx_buffer[PREV_PAYLOAD_RX_LENGTH];

	//unsigned char test_message[9] = {0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19};

	block_ms(20);
	init_io();

	MC_ENABLE = 1;
	close_hitch();
	block_ms(1000);


	init_usart1();


	for(i=0;i<20;i++)
	{
		send_payload_ID();
		block_ms(20);
	}


	while(1)
	{

		ClrWdt();

		//send payload message every 500ms, in case robot
		//resets, but hitch doesn't
		payload_id_counter++;
		if(payload_id_counter > 5)
		{
			send_payload_ID();
			payload_id_counter = 0;
		}

/*		if( (hitch_open_counter > 0) )
		{
			hitch_locked_open_latch = 1;

		}


		if(hitch_locked_open_latch)
		{

			hitch_locked_open_counter++;
			if(hitch_locked_open_counter > 50)
			{
				hitch_locked_open_counter = 0;
				hitch_locked_open_latch = 0;
			}
			
		}*/	


		if(prev_payload_msg_ready)
		{
			prev_payload_msg_ready = 0;
			for(i=0;i<PREV_PAYLOAD_RX_LENGTH;i++)
			{
				prev_payload_rx_buffer[i] = previous_payload_rx[i];
			}
			if(Is_CRC_valid(prev_payload_rx_buffer, PREV_PAYLOAD_RX_LENGTH))
			{
				//green button is pressed, move actuator
				if(prev_payload_rx_buffer[5] & 0x80)
				{
					hitch_open_counter++;
					if(hitch_open_counter > 10)
					{
						open_hitch();
					}
					if(hitch_open_counter > 100)
						hitch_open_counter = 100;

		
				}
				else
				{
					close_hitch();
					hitch_open_counter = 0;
				}

			}
			
		}


	//	if(hitch_locked_open_latch)
	//		open_hitch();


//		I'll want this as a timer interrupt so that we're not limited as to the speed
//		of messages we get.
		block_ms(100);

	}

	return 0;
}
