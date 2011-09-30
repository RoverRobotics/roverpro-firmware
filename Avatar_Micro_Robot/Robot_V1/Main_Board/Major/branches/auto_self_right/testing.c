#include "system.h"



void direct_motor_control(void)
{
	int right_effort;
	int left_effort;
	int flipper_effort;
	unsigned char ad_value;
	unsigned char data_to_send[MESSAGE_TO_PAYLOAD_LENGTH-4];
	unsigned int ad_accumulator;
	unsigned int i;
	static unsigned char motors_on = 0;

	data_to_send[0] = 0x00;
	data_to_send[1] = 0x00;
	data_to_send[2] = 0x00;
	data_to_send[3] = 0x00;
	data_to_send[4] = 0x00;


	//turn on motors, but set speed to 0
	Relay_Resistor_Control = ON;
	block_ms(500);
	Relay_Power_Control = ON;
	Relay_Resistor_Control = OFF;

	block_ms(2500);
	motors_on = 1;
	while(1)
	{

		kick_watchdogs();
		//ClrWdt();
		if(message_from_payload_ready)
		{
			message_from_payload_ready = 0;
			if(Is_payload_CRC_valid(message_from_payload,MESSAGE_FROM_PAYLOAD_LENGTH))
			{
				if(message_from_payload[2] == 0x01)
				{
					right_effort = (int)message_from_payload[3]-100;
					left_effort = (int)message_from_payload[4]-100;
					flipper_effort = (int)message_from_payload[5]-100;

					if(message_from_payload[6] == 0x00)
					{
						Relay_Power_Control = OFF;
						Relay_Resistor_Control = OFF;
						right_effort = 0;
						left_effort = 0;
						flipper_effort = 0;
						motors_on = 0;

					}
					else if((motors_on == 0) && (message_from_payload[6] & 0x01))
					{
						kick_watchdogs();
						//ClrWdt();
						Relay_Resistor_Control = ON;
						block_ms(500);
						Relay_Power_Control = ON;
						Relay_Resistor_Control = OFF;
						block_ms(100);
						motors_on = 1;

					}

					set_motor_effort(right_effort,RIGHT_MOTOR);
					set_motor_effort(left_effort,LEFT_MOTOR);
					set_motor_effort(flipper_effort,FLIPPER_MOTOR);

						
				}



				ad_accumulator = 0;
		
				//take 10 ADC measurements, average them to get rid of noise
				for(i=0;i<10;i++)
				{
					AD1CON1bits.ADON = 0;
					AD1CHS = 11;
					AD1CON1bits.ADON = 1;
					Nop();
					AD1CON1bits.SAMP = 1;
					while(!AD1CON1bits.DONE);
					ad_accumulator +=  ADC1BUF0>>2;
					block_ms(10);
				}
				ad_value = (ad_accumulator/10);
				data_to_send[3] = ad_value;
				send_message_to_payload(data_to_send);

			}
		}





	}


}


void test_battery_life(void)
{

	unsigned int ad_accumulator = 0;
	unsigned char battery_voltage = 0;
	unsigned char data_to_send[MESSAGE_TO_PAYLOAD_LENGTH-4];
	unsigned char i;

	kick_watchdogs();
	//ClrWdt();
	Map_Pins();		//Set Tris registers and initial states
	Timer1Init();	//Setup and start Timer 1 system
	init_timer3();
	init_timer2();
	init_uart2();
	init_current_limit_atod();
	//set audio link to transmit
	Audio_pin = 0;		//TX
	Amp_pin = 0;		//Amp off
	Mic_pin = 1;


	//turn on motors, but set speed to 0
	Relay_Resistor_Control = ON;
	block_ms(500);
	Relay_Power_Control = ON;
	Relay_Resistor_Control = OFF;
	block_ms(2500);
	Set_Desired_Motor_Pulse_Quadrants(0x7F, 0x7F, 0, 0);

	data_to_send[0] = 0x0a;
	data_to_send[1] = 0x0b;
	data_to_send[2] = 0x0c;
	data_to_send[3] = 0x0d;
	data_to_send[4] = 0x0e;



	while(1)
	{
		kick_watchdogs();
		//ClrWdt();
		ad_accumulator = 0;

		//take 3 ADC measurements, average them to get rid of noise
		for(i=0;i<3;i++)
		{
			ad_accumulator += return_AD_value(0x0c);
			block_ms(20);
		}
		battery_voltage = (ad_accumulator/3);
		data_to_send[3] = battery_voltage;
		send_message_to_payload(data_to_send);
		block_ms(2000);

	}

}


void send_reset_registers(void)
{
	unsigned int RCON_COPY = 0xaa;
	unsigned char data_to_send[MESSAGE_TO_PAYLOAD_LENGTH-4];
	kick_watchdogs();
	//ClrWdt();	
	data_to_send[0] = 0x0a;
	data_to_send[1] = 0x0b;
	data_to_send[2] = 0x0c;
	data_to_send[3] = 0x0d;
	data_to_send[4] = 0x0e;
	block_ms(1000);
	RCON_COPY = RCON;
	data_to_send[0] = RCON_COPY>>8;
	data_to_send[1] = RCON_COPY&0xff;
	while(1)
	{
		kick_watchdogs();
		//ClrWdt();
		send_message_to_payload(data_to_send);
		block_ms(1000);

	}

}
