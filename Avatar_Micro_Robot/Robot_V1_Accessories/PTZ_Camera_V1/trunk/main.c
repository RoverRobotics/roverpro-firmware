//#include "include\Compiler.h"
#include "p24fxxxx.h"
_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_OFF & ICS_PGx2) 
_CONFIG2(IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRI & IOL1WAY_OFF)


#include "system.h"





int main() {
	//char test_message[4] = {0xaa,0xbb,0xcc,0x01};
	//unsigned char break_in_message[4] = {0x04,0x00,0x00,0x00};
	init_io();
	init_usart1();
	init_uart2();

	while(1)
	{
		/*while(1)
		{
		send_message_to_robot(break_in_message);
		block_ms(50);
		}*/

		if(message_from_robot_ready)
		{

			message_from_robot_ready = 0;
			//send_message_to_robot(test_message);

			if(Is_CRC_valid(message_from_robot, MESSAGE_FROM_ROBOT_LENGTH))
			{
				//LATBbits.LATB9 = 0;
				decode_message_from_robot();
			}

		}

	}

	return 0;
}
