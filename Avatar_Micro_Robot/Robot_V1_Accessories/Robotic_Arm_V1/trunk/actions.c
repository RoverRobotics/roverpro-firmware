#include "system.h"
#include "libpic30.h"

unsigned char board_type = 0x00;

void write_program_memory(_prog_addressT p_address, unsigned int data1, unsigned int data2);
void mix_joints(unsigned char direction);
//pot1 is on top of the board and increases clockwise 
unsigned int pot1_zero_angle;
unsigned int pot2_zero_angle;
//const int pot_angles[512] = 0;
//const int pot_angles[4] = {0,0,0,0};
int zero_angles[4] = {0,0,0,0};
int elbow_shoulder_difference(unsigned char elbow_angle, unsigned char shoulder_angle);
void joints_PI_control(unsigned char direction, int elbow_angle, int shoulder_angle);

int new_elbow_speed = 100;
int new_shoulder_speed = 100;
int new_gripper_speed = 100;
unsigned char elbow_neutral_counter = 0;
unsigned char shoulder_neutral_counter = 0;
//int __attribute__((space(prog))) memory_data[1024];
void calibrate_joint(void)
{
//	unsigned int i;
	unsigned int pot1_value;
	unsigned int pot2_value;


	pot1_value = get_ad_value(0)>>2;
	pot2_value = get_ad_value(1)>>2;

	
	DataEEInit();
	DataEEWrite(pot1_value*1.31,0);
	DataEEWrite(pot2_value*1.31,1);


}

//correct for wraparound from zero to max
int elbow_shoulder_difference(unsigned char elbow_angle, unsigned char shoulder_angle)
{
	int corrected_shoulder_angle, corrected_elbow_angle;

	corrected_shoulder_angle = shoulder_angle;
	corrected_elbow_angle = elbow_angle;


//	We don't expect the elbow to ever be extended farther than straight (180).  If it does this, we'll have to move
//	it with the joystick.
	if(elbow_angle > 230)
	{
		corrected_elbow_angle = 241 - (int)elbow_angle;
	}

	if(shoulder_angle > 150)
	{
		//must be folded, or robot not upright (gripper would contact ground in this state
		if(elbow_angle < 100)
		{
			corrected_shoulder_angle = 165 - (int)shoulder_angle;
		}
			
	}
	
	return corrected_elbow_angle - corrected_shoulder_angle;
	



}

// 0xff 0xcc 0x00 elbow_motor_effort gripper_motor_effort DC DC CRC1 CRC2
void act_on_message(void)
{

	unsigned int i;
	int new_motor_effort = 0;
	unsigned char buffer[NEXT_PAYLOAD_TX_LENGTH];
		
	
	
	if(board_type == ELBOW)
		new_motor_effort = ((int)prev_payload_rx_last_good[3])-100;
	else if(board_type == SHOULDER)
		new_motor_effort = ((int)prev_payload_rx_last_good[4])-100;



	set_motor_effort(new_motor_effort);

	//maybe should be -4 instead of -2
	for(i=0;i<NEXT_PAYLOAD_TX_LENGTH-2;i++)
	{
		buffer[i] = prev_payload_rx_last_good[i+2];

	}
	if(board_type == SHOULDER)
	{
		send_message_to_next_payload(buffer);
	}
	else if(board_type == ELBOW)
	{
		if(prev_payload_rx_last_good[5] & 0x80)
			zoom_in();
		else if(prev_payload_rx_last_good[5] & 0x40)
			zoom_out();
		else
			zoom_stop();

	}

}

void form_prev_payload_message(void)
{

	unsigned char previous_payload_tx_msg[PREV_PAYLOAD_TX_LENGTH-4];
	unsigned int i;
	int joint_position;
	

	/*for(i=0;i< PREV_PAYLOAD_TX_LENGTH-4;i++)
	{
		previous_payload_tx_msg[i] = i;
	}*/

	for(i=0;i< PREV_PAYLOAD_TX_LENGTH-4;i++)
	{
		previous_payload_tx_msg[i] = next_payload_rx_last_good[i+2];
	}
	
	joint_position = return_joint_position();
	
	if(board_type == ELBOW)
	{
			previous_payload_tx_msg[0] = joint_position>>8;
			previous_payload_tx_msg[1] = joint_position&0xff;
			previous_payload_tx_msg[4] = 0x00;	//gripper current		
	}
	else if(board_type == SHOULDER)
	{

			previous_payload_tx_msg[0] = next_payload_rx_last_good[2];
			previous_payload_tx_msg[1] = next_payload_rx_last_good[3];
			//previous_payload_tx_msg[0] = 0xaa;
			//previous_payload_tx_msg[1] = 0xbb;
			previous_payload_tx_msg[4] = next_payload_rx_last_good[6];
			previous_payload_tx_msg[2] = joint_position>>8;
			previous_payload_tx_msg[3] = joint_position&0xff;	
	}


	send_message_to_prev_payload(previous_payload_tx_msg);

}



/*void form_next_payload_message(void)
{
	unsigned char next_payload_tx_msg[NEXT_PAYLOAD_TX_LENGTH];
	unsigned int i;
	unsigned char buffer[PREV_PAYLOAD_RX_LENGTH];
	//next_payload_tx_msg[0] = 0xff;
	//next_payload_tx_msg[1] = 0xcc;

	for(i=0;i< NEXT_PAYLOAD_TX_LENGTH;i++)
	{
		next_payload_tx_msg[i] = previous_payload_rx[i];
	}

}*/


void read_pot_calibration(void)

{



	DataEEInit();

	pot1_zero_angle = DataEERead(0);
	pot2_zero_angle = DataEERead(1);



}


//0xFF 0xCC joystick_RH joystick_RV buttons 0x00 CRC1 CRC2
//buttons:
//7: L Trigger
//6: R Trigger
//5: Toggle UP
//4: Toggle Down
//3: Green button
//2:
//1:
//0:
// 0xFF 0xCC Speed_Gripper Speed_Elbow 0x00 0x00 CRC1 CRC2
void arm_control_loop(void)
{
	unsigned char buffer[NEXT_PAYLOAD_TX_LENGTH] = {0,0,0,0,0,0,0,0,0};
	//unsigned char new_elbow_speed = 100;
	unsigned char new_gripper_speed = 100;
	//unsigned char new_shoulder_speed = 100;
	char new_motor_effort = 0;
	static unsigned char last_direction = EXTEND;
	static unsigned char change_direction_counter = 0;
	//static unsigned char elbow_change_direction_counter = 0;
	//static char last_elbow_direction = 0;

//	right now, toggle will control shoulder, and left joystick will control elbow

//	toggle up, toggle down
	if(prev_payload_rx_last_good[5] & 0x20)
	{
		if(last_direction != EXTEND)
		{
			change_direction_counter++;
			new_shoulder_speed = 100;
			new_elbow_speed = 100;
		}
		else
		{
			change_direction_counter = 0;

		}
		if((change_direction_counter > 10) || (change_direction_counter == 0))
		{
			mix_joints(EXTEND);
			last_direction = EXTEND;
		}
	}
	else if(prev_payload_rx_last_good[5] & 0x10)
	{
		if(last_direction != RETRACT)
		{

			new_shoulder_speed = 100;
			new_elbow_speed = 100;
			change_direction_counter++;
		}
		else
			change_direction_counter = 0;
		if((change_direction_counter > 10) || (change_direction_counter == 0))
		{
			mix_joints(RETRACT);
			last_direction = RETRACT;
		}

	}
	else
	{
		new_shoulder_speed = 100;
		last_direction = ELBOW_ONLY;
		//elbow_neutral_counter = 0;
		//shoulder_neutral_counter = 0;

		if(prev_payload_rx_last_good[5] & 0x80)
		{
			new_gripper_speed = 130;
		}
		else if(prev_payload_rx_last_good[5] & 0x40)
		{
			new_gripper_speed = 70;
		}
		
	
		//new_elbow_speed = ((int)prev_payload_rx_last_good[4]-127.5)/3+100;
		//new_elbow_speed = ((int)prev_payload_rx_last_good[4]-127.5)/2+100;
		new_elbow_speed = (-(int)prev_payload_rx_last_good[4]+127.5)*.7+100;
		new_shoulder_speed = (-(int)prev_payload_rx_last_good[3]+127.5)*.7+100;
	

	}


	/*if(elbow_change_direction_counter > 0)
	{
		new_elbow_speed = 100;
		elbow_change_direction_counter++;
		if(elbow_change_direction_counter > 5)
			elbow_change_direction_counter = 0;
	}
	else
	{
		if(new_elbow_speed > 100) 
		{
			if(last_elbow_direction == -1)
			{
				elbow_change_direction_counter = 1;
				new_elbow_speed = 100;
			}
			last_elbow_direction = 1;
		}
		else if(new_elbow_speed < 100)
		{
			if(last_elbow_direction == 1)
			{
				elbow_change_direction_counter = 1;
				new_elbow_speed = 100;
			}
			last_elbow_direction = -1;
		}
	}*/

		if(prev_payload_rx_last_good[5] & 0x08)
			buffer[3] = 0x80;
		else if(prev_payload_rx_last_good[5] & 0x04)
			buffer[3] = 0x40;
		buffer[2] = new_elbow_speed;
		buffer[1] = new_gripper_speed;
	
		new_motor_effort = (int)new_shoulder_speed-100;
		send_message_to_next_payload(buffer);
		set_motor_effort(new_motor_effort);
}

void trigger_motor_reset(void)
{
		/*POWER_MOSFET = 0;
		block_ms(1000);
		RESISTOR_MOSFET = 1;
		block_ms(500);
		POWER_MOSFET = 1;
		RESISTOR_MOSFET = 0;*/

}

void joints_PI_control(unsigned char direction, int elbow_angle, int shoulder_angle)
{

	int elbow_error, shoulder_error;
	static int shoulder_error_accumulator = 0;
	static int elbow_error_accumulator = 0;
	static unsigned char last_direction = 0xff;
	//int elbow_speed, shoulder_speed;
	static int elbow_center_speed = 100;
	static int shoulder_center_speed = 100;

	elbow_error = elbow_shoulder_difference(elbow_angle,shoulder_angle);
	shoulder_error = -elbow_shoulder_difference(elbow_angle,shoulder_angle);

	//shoulder_error = 0;

	if(direction == EXTEND && last_direction != EXTEND)
	{
		shoulder_error_accumulator = 0;
		elbow_error_accumulator = 0;
		//\new_elbow_speed = 30;
		elbow_center_speed = 20;
		new_elbow_speed = 100;
		new_shoulder_speed = 40;
		shoulder_center_speed = 40;
		//new_shoulder_speed = 100;
		

	}
	else if (direction == RETRACT && last_direction != RETRACT)
	{
		shoulder_error_accumulator = 0;
		elbow_error_accumulator = 0;
		//new_elbow_speed = 170;
		elbow_center_speed = 180;
		new_elbow_speed = 100;
		new_shoulder_speed = 160;
		shoulder_center_speed = 160;
		//new_shoulder_speed = 100;
	}

	//elbow_error = 0;
	//shoulder_error = 0;


	elbow_error_accumulator += elbow_error;
	shoulder_error_accumulator += shoulder_error;


	if(elbow_error < 2 && elbow_error > -2)
	{
		elbow_error_accumulator = 0;
		shoulder_error_accumulator = 0;
	}

	shoulder_error = 0;
	elbow_error = 0;
	new_shoulder_speed = shoulder_center_speed+( SHOULDER_Kp*shoulder_error + SHOULDER_Ki*shoulder_error_accumulator);
	new_elbow_speed = elbow_center_speed+(ELBOW_Kp*elbow_error + ELBOW_Ki*elbow_error_accumulator);

	if(new_shoulder_speed > 200)
		new_shoulder_speed = 200;
	if(new_elbow_speed > 200)
		new_elbow_speed = 200;

	if(new_shoulder_speed < 0)
		new_shoulder_speed = 0;
	if(new_elbow_speed < 0)
		new_elbow_speed = 0;

	if(direction == EXTEND)
	{
		if(new_shoulder_speed > 100)
			new_shoulder_speed = 100;
	}
	else if(direction == RETRACT)
	{

		if(new_shoulder_speed < 100)
			new_shoulder_speed = 100;

	}

	if(new_elbow_speed > 90 && new_elbow_speed < 110)
		new_elbow_speed = 100;
	if(new_shoulder_speed > 90 && new_shoulder_speed < 110)
		new_shoulder_speed = 100;

	last_direction = direction;
}

void mix_joints(unsigned char direction)
{


	int elbow_angle, shoulder_angle;
	static unsigned char error_counter = 0;
	static unsigned char last_elbow_speed = 0;
	static unsigned char last_shoulder_speed = 0;
	int elbow_speed, shoulder_speed;





/*	if(direction == EXTEND)
	{
		new_elbow_speed = 130;
		new_shoulder_speed = 130;
	}
	else if(direction == RETRACT)
	{
		new_elbow_speed = 70;
		new_shoulder_speed = 130;
	}
	return;*/

	elbow_angle = next_payload_rx_last_good[3];
	shoulder_angle = next_payload_rx_last_good[5];



	if((elbow_angle == 0xff) || (shoulder_angle == 0xff))
	{
		elbow_speed = last_elbow_speed;
		shoulder_speed = last_shoulder_speed;
		error_counter++;

	}
	else
	{
		error_counter = 0;

		joints_PI_control(direction, elbow_angle, shoulder_angle);

		/*
		if(direction == RETRACT)
		{
			//elbow_speed = 130 - 3*((int)shoulder_angle - (int)elbow_angle);
			//shoulder_speed = 130 - 3*((int)elbow_angle - (int)shoulder_angle);

			
			elbow_speed = CENTER_ELBOW_SPEED_RETRACT + 3*elbow_shoulder_difference(elbow_angle,shoulder_angle);
			shoulder_speed = CENTER_SHOULDER_SPEED_RETRACT - 3*elbow_shoulder_difference(elbow_angle,shoulder_angle);


			if(elbow_speed > MAX_ELBOW_SPEED_RETRACT)
				elbow_speed = MAX_ELBOW_SPEED_RETRACT;
			else if(elbow_speed < MIN_ELBOW_SPEED_ERETRACT)
				elbow_speed = MIN_ELBOW_SPEED_ERETRACT;
			if(shoulder_speed > MAX_SHOULDER_SPEED_RETRACT)
				shoulder_speed = MAX_SHOULDER_SPEED_RETRACT;
			else if(shoulder_speed < MIN_SHOULDER_SPEED_RETRACT)
				shoulder_speed = MIN_SHOULDER_SPEED_RETRACT;
			
		}
		else if(direction == EXTEND)
		{
			elbow_speed = CENTER_ELBOW_SPEED_EXTEND + 3*elbow_shoulder_difference(elbow_angle,shoulder_angle);
			shoulder_speed = CENTER_SHOULDER_SPEED_EXTEND - 3*elbow_shoulder_difference(elbow_angle,shoulder_angle);
		//	shoulder_speed = 70;
			if(elbow_speed > MAX_ELBOW_SPEED_EXTEND)
				elbow_speed = MAX_ELBOW_SPEED_EXTEND;
			else if(elbow_speed < MIN_ELBOW_SPEED_EXTEND)
				elbow_speed = MIN_ELBOW_SPEED_EXTEND;
			if(shoulder_speed > MAX_SHOULDER_SPEED_EXTEND)
				shoulder_speed = MAX_SHOULDER_SPEED_EXTEND;
			else if(shoulder_speed < MIN_SHOULDER_SPEED_EXTEND)
				shoulder_speed = MIN_SHOULDER_SPEED_EXTEND;
		}*/
	}
	if(error_counter > 10)
	{
		elbow_speed = 100;
		shoulder_speed = 100;
	}
	



	/*new_elbow_speed = elbow_speed;
	new_shoulder_speed = shoulder_speed;


	last_elbow_speed = new_elbow_speed;
	last_shoulder_speed = new_shoulder_speed;*/
	

}




int return_joint_position(void)
{

	int pot1_value;
	int pot2_value;
//	int joint_angle_1;
//	int joint_angle_2;
	int pot_angle_1, pot_angle_2;
	int pot_angle;
	int joint_angle;
//	unsigned char test_message[8] = {0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18};
	pot1_value = get_ad_value(0)>>2;
	pot2_value = get_ad_value(1)>>2;

	//333.3/255 = 1.31
	//potentiometer flat up is halfway point, clockwise increases voltage
	pot_angle_1 = -pot1_value*1.31+pot1_zero_angle;
//	pot_angle_1 = (pot1_value - pot1_zero_angle)*1.31;

	pot_angle_2 = -pot2_zero_angle + pot2_value*1.31;
//	pot_angle_2 = (pot2_zero_angle - pot2_value)*1.31;

	//send_message_to_prev_payload(test_message);
//	block_ms(100);
	if(pot_angle_1 < 0)
	{
		pot_angle_1+=360;

	}
	else if(pot_angle_1 > 360)
		pot_angle_1-=360;
	if(pot_angle_2 < 0)
	{
		pot_angle_2+=360;
	}
	else if(pot_angle_2 > 360)
		pot_angle_2-=360;



//	If one pot is in the dead zone, use the other value
	if((pot1_value < 10) || (pot1_value > 245))
	{

		pot_angle = pot_angle_2;

	}
	else if((pot2_value < 10) || (pot2_value > 245))
	{

		/*if(pot1_value < 55 || pot1_value > 200)
		{
			return 0x0fff;
		}*/

		pot_angle = pot_angle_1;
	}
	else
	{
//		If the values are close to wrapping around, use the small one.  Otherwise, make sure they're close and average.
		if((pot_angle_1 < 4) && (pot_angle_2 > 356))
		{
			pot_angle = pot_angle_1;
		}
		else if((pot_angle_2 < 4) && (pot_angle_1 > 356))
		{

			pot_angle = pot_angle_2;
		}
		else if(abs(pot_angle_1-pot_angle_2) > 8)
		{
			//return 0x0fff;
			pot_angle = 0x0fff;
		}
		else
		{
			//average potentiometer readings.  I will want to add some error checking to make sure they agree later.
			pot_angle = (pot_angle_1 + pot_angle_2)/2;
		}
	}


/*	test_message[0] = pot_angle_1>>8;
	test_message[1] = pot_angle_1&0xff;
	test_message[2] = pot_angle_2>>8;
	test_message[3] = pot_angle_2 & 0xff;
	test_message[4] = pot_angle >>8;
	test_message[5] = pot_angle & 0xff;*/
	//send_message_to_prev_payload(test_message);	


	if(pot_angle == 0x0fff)
	{
		return 0x0fff;

	}
	else
	{

		if(board_type == SHOULDER)
		{
			//this should work for angles up to 164.57 degrees
			joint_angle = pot_angle/2.1875;
			if(joint_angle > 164) joint_angle = 164;
		}
		else if(board_type == ELBOW)
		{
			//this only works for angles up to 240 degrees, which doesn't cover the full range of the elbow
			joint_angle = pot_angle/1.5;
			if(joint_angle > 240) joint_angle = 240;
		}
		//correct if out of bounds
		if(joint_angle < 0) joint_angle = 0;
		
		
		return joint_angle;
	}

	


}
