#include "include/Compiler.h"
//#include "address_generation.h"
#include "joystick_trimming.h"
#include "DEE Emulation 16-bit.h"
#include "Controller.h"
#include "Timers.h"
//#include "Audio.h"

int joystick_trim_factor = 0;

void joystick_trimming_loop(void)
{

	static unsigned char last_flipper_state = 2;
	unsigned char current_payload_button = 0;
	static unsigned char last_payload_button = 0;

	joystick_trim_factor = 0;


	//we want the same address for 
	//debugging -- remove this later.
	/*OCU_ADDRESS_MSB = 0xAB;
	OCU_ADDRESS_LSB = 0xCD;
	ROBOT_ADDRESS_MSB = 0xAB;
	ROBOT_ADDRESS_LSB = 0xCD;*/

	while(1)
	{
		
		ClrWdt();

		Construct_Controller_Message();	
		
		//disable flipper command when in trimming mode
		Datalink_Send_Buffer[10] = 0x00;

		Datalink_Message_Send();

		if(FLIPPER_UP)
		{
			if(last_flipper_state != 0)
			{
				last_flipper_state = 0;
				joystick_trim_factor++;

			}

		}
		else if(FLIPPER_DOWN)
		{

			if(last_flipper_state != 1)
			{
				last_flipper_state = 1;
				joystick_trim_factor--;

			}

		}
		else
		{
			last_flipper_state = 2;

		}

		current_payload_button = PAYLOAD_BUTTON;


		//if rising edge on green button, save state
		if(current_payload_button && (last_payload_button == 0))
		{

				DataEEWrite(joystick_trim_factor,5);
				Nop();

		}

		last_payload_button = current_payload_button;

		block_ms(100);


	}



}
