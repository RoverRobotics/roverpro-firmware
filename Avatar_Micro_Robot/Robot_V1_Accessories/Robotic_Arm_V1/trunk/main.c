//#include "include\Compiler.h"
#include "p24fxxxx.h"
_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_OFF & ICS_PGx2) 
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

	unsigned int missed_message_counter_upstream = 0;

//	these arrays are used as a buffer.  We might not need these, if we
//	can use msg_ready flags intelligently (clear them once we've read the message).
//	We can make message receipt faster if we do buffer them (so that the interrupt
//	Isn't waiting forever for the flag to be cleared.
	unsigned char prev_payload_rx_buffer[PREV_PAYLOAD_RX_LENGTH];
	unsigned char next_payload_rx_buffer[PREV_PAYLOAD_RX_LENGTH];

	//unsigned char test_message[9] = {0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19};

	block_ms(20);
	init_io();
	init_usart1();
	init_atod();
	init_pwm();

	if(PORTAbits.RA8 == 0)
	{
		if(PORTAbits.RA9)
		{
			board_type = ELBOW;
		}
		else
		{
			board_type = SHOULDER;
		}
	}
	else
	{
		board_type = BASE;
	}

	if(board_type == ELBOW)
		init_zoom();
	else
		init_usart2();


	/*while(1)
	{
		zoom_in();
		block_ms(2000);
		zoom_out();
		block_ms(2000);
	}*/



	if(board_type == BASE)
	{
		for(i=0;i<20;i++)
		{
			send_payload_ID();
			block_ms(20);
		}
		TRISBbits.TRISB6 = 0;
		TRISBbits.TRISB7 = 0;
		POWER_MOSFET = 0;
		RESISTOR_MOSFET = 1;
		block_ms(1000);
		POWER_MOSFET = 1;
		RESISTOR_MOSFET = 0;

		//modify idle message for the base
		prev_payload_rx_last_good[3] = 0x7f;
		prev_payload_rx_last_good[4] = 0x7f;
	
	}
	else
	{
			read_pot_calibration();
	}
	





//	initialize last good payload message
//	I'll have to use more intelligent values later,
//	or wait until the first message is received before
//	using these buffers
	/*for(i=0;i<PREV_PAYLOAD_RX_LENGTH;i++)
	{
		prev_payload_rx_last_good[i] = i;
	}

	for(i=0;i<NEXT_PAYLOAD_RX_LENGTH;i++)
	{
		next_payload_rx_last_good[i] = i;
	}*/

	/*block_ms(3000);
	digital_zoom_on();
	block_ms(1000);*/

	while(1)
	{


//		if a message is ready, buffer it, then check if it's good.  If it is
//		good, then put it in another buffer.  This way we can be sure that we
//		keep the last good message value at all times.  We won't worry about
//		timing -- just send the last good value back down the line.
//		We'll do this on both previous and next payload messages.
		if(prev_payload_msg_ready)
		{
			prev_payload_msg_ready = 0;
			for(i=0;i<PREV_PAYLOAD_RX_LENGTH;i++)
			{
				prev_payload_rx_buffer[i] = previous_payload_rx[i];
			}
			if(Is_CRC_valid(prev_payload_rx_buffer, PREV_PAYLOAD_RX_LENGTH))
			{

				if(prev_payload_rx_buffer[2] == 0xff)
				{
					block_ms(100);
					calibrate_joint();
					block_ms(100);

					for(i=0;i<4;i++)
					{
						send_message_to_next_payload(calibrate_message);
						block_ms(50);
					}
					read_pot_calibration();
					block_ms(200);
					
				}
				//0x00 is for control messages
				else if(prev_payload_rx_buffer[2] == 0x00)
				{

					for(i=0;i<PREV_PAYLOAD_RX_LENGTH;i++)
					{
						prev_payload_rx_last_good[i] = prev_payload_rx_buffer[i];
					}
				}
				else if(prev_payload_rx_buffer[2] == 0x03)
				{
					POWER_MOSFET = 0;
					RESISTOR_MOSFET = 0;
					block_ms(500);
					RESISTOR_MOSFET = 1;
					block_ms(500);
					POWER_MOSFET = 1;
					RESISTOR_MOSFET = 0;


				}

			}
			
		}

//		Do the same for the next payload message
		if(next_payload_msg_ready)
		{
			next_payload_msg_ready = 0;

			for(i=0;i<NEXT_PAYLOAD_RX_LENGTH;i++)
			{
				next_payload_rx_buffer[i] = next_payload_rx[i];
			}
			if(Is_CRC_valid(next_payload_rx_buffer, NEXT_PAYLOAD_RX_LENGTH))
			{
				if(board_type == BASE)
					missed_message_counter_upstream = 0;

				for(i=0;i<NEXT_PAYLOAD_RX_LENGTH;i++)
				{
					next_payload_rx_last_good[i] = next_payload_rx_buffer[i];
				}

			}


			
		}
		else
		{

			if(board_type == BASE)
			{
				missed_message_counter_upstream++;
				if(missed_message_counter_upstream > 10)
					trigger_motor_reset();
			}
		}




//		reads joint values, and inserts them into the message
//		from the next payload, then sends it back down the line

		form_prev_payload_message();


		//block_ms(500);




//		sets motor speeds, and starts sending message
//		to next paylaod
		if(board_type == BASE)
		{
			arm_control_loop();
		}
		else
		{
			act_on_message();
		}
			




//		I'll want this as a timer interrupt so that we're not limited as to the speed
//		of messages we get.
		block_ms(100);
		//form_next_payload_message();

		

	}

	return 0;
}
