

/**
 * @file usbinterface.cc
 * @author Robotex Inc.
 *
 * usbinterface - PIC firware interface over USB
 *
 */


#define NUM_TEST_CYCLES 200

#include <stdio.h>
#include <sys/types.h>
#include <libusb-1.0/libusb.h>
#include <vector>
#include <unistd.h>
#include <cstring>
#include <cstdlib>


#include "firmware/usb_config.h"

//for getting keypress
#include <ncurses.h>



//for timing usb polling
#include <time.h>

//for the register descriptions
#include "firmware/usb_config.h"


//for sleep command
#include "unistd.h"

#define DEBUG
#define PRINT_ALL_USB

// from the perspective of software
typedef uint8_t IN_PACKET[980];
typedef uint8_t OUT_PACKET[100];


/**********Function Prototypes******/
//return true if key pressed
// whether the keyboard was hit, true/false
int kbhit(void);

void hitch_control_through_keyboard(void);
void cycle_n_times(int times_to_cycle);

bool handle_usb_communication(void);

//other namespaces are for registers.h file
using namespace std;
using namespace rbx;
using namespace rbx::telemetry;


void init_joystick(void);
void update_joystick(void);

int return_register_index(void* register_to_check);


void watch_register_value(void* register_to_check, char* register_name);

int setup_usb_device(void);
void clean_up_USB(void);
void incoming_callback(struct libusb_transfer *transfer);
void outgoing_callback(struct libusb_transfer *transfer);
void reinit_USB(void);

void setup_ncurses(void);
void cleanup_ncurses(void);

void print_float(uint8_t* data);
void print_string(uint8_t* data,int length);
void print_uint(uint8_t* data);

unsigned int register_indices[100];
char* register_names[100];
unsigned int register_data_lengths[100];
unsigned int register_watch_length = 0;

void parse_incoming_message(uint8_t* incoming_message);
int return_watch_array_index(unsigned int register_index);



//global variables for libusb stuff
   libusb_device **devs;
   struct libusb_device_descriptor desc;
   vector<libusb_device_handle*> robotex_device_handles;
   libusb_device_handle *device_handle;
   libusb_transfer* incoming_transfer;
   libusb_transfer* outgoing_transfer;
   IN_PACKET in_packet;
   //uint16_t out_packet[8];
   OUT_PACKET out_packet;

  ssize_t num_devices;

int main(int argn, char **argc)
{

	unsigned int register_index = 0;
	unsigned int i;

	struct tm *current;
	time_t now;
	time(&now);
	current = localtime(&now);


	setup_usb_device();


	//hitch_control_through_keyboard();
	cycle_n_times(NUM_TEST_CYCLES);
	
	clean_up_USB();

   return 0;
}

//find the actual index that gets sent over USB
int return_register_index(void* register_to_check)
{
	unsigned int i = 0;
	//loop through all registers
	while(registers[i].ptr)
	{

	//	printf("checking %d against %d \n",registers[i].ptr,register_to_check);
		//check pointer to variable in registers[] to the variable we passed to this function
		//if it matches, return the index
		if( registers[i].ptr == register_to_check )
		{
			printf("result is %d\n",i);
			return i;
		}
		i++;
	}
	return 0; 
}

void parse_incoming_message(uint8_t* incoming_message)
{
	unsigned int i = 0;
	unsigned int j = 0;
	int register_index;
	int watch_array_index = 0;
	uint8_t temp_data[200];
	
	printf("\n\n\n\n\n");
	while(1)
	{
		register_index = (incoming_message[i+1] << 8) | incoming_message[i];
		i+=2;
		if(register_index == 0xffff) break;
		if(i>100) break;
		watch_array_index = return_watch_array_index(register_index);
		if(watch_array_index == -1) {
			printf("Register %i not found!\n",register_index);
			break;
			}
		//printf( "%i at position %i\n",register_index,i);
			for(j=0;j<register_data_lengths[watch_array_index];j++)
				
			{
				
				temp_data[j] = incoming_message[i+j];
			}
		
		
		printf("%s: ",register_names[watch_array_index]);
		if(register_names[watch_array_index] == "REG_OCU_TEMP_1")
		{			
			print_float(temp_data);
		}
		else if(register_names[watch_array_index] == "REG_OCU_GPS_MESSAGE")
			print_string(temp_data,100);
		else if(1)
			print_uint(temp_data);
		else
		{
			for(j=0;j<register_data_lengths[watch_array_index];j++)
				
			{
				
				printf("%x ",incoming_message[i+j]); 
			}
		}
			printf("\n");
		
		i+=register_data_lengths[watch_array_index];

	}
	

}

// array of indices we're looking at, to avoid hardcoding
int return_watch_array_index(unsigned int register_index)
{
	for(unsigned int i=0;i<register_watch_length;i++)
	{
		if(register_index == register_indices[i])
			return i;
	}
	return -1;

}

void watch_register_value(void* register_to_check, char* register_name)
{
	static unsigned int i = 0;
	unsigned int register_index = return_register_index(register_to_check);
	register_indices[i] = register_index;
	register_names[i] = register_name;
	//printf("%s: %d\n",register_name,register_index);
	register_data_lengths[i] = telemetry::registers[register_index].size;
	printf("also, %s, %d, length: %d\n",register_names[i],register_indices[i],register_data_lengths[i]);

	i++;
	
	register_watch_length = i;
}


// look into if you can convert 4 chars to a float, this might be irrelevant for hitch testing
void print_float(uint8_t* data)
{
	unsigned int i;
	float temp = 0;
	char input_data[4];
	/*for(i=0;i<4;i++)
	{
		input_data[i] = data[i];
		printf("xxx%x\n",input_data[i]);
	}*/
	
	memcpy(&temp,data,4);
	//temp = atof(input_data);
	printf("%f",temp);
	//temp = data[0]<<32 + data[1]<<24 + data[2]<<16 + data[1]<<8 + data[0];
}
void print_uint(uint8_t* data) {
	uint16_t temp;
	memcpy(&temp,data,2);
	printf("%i",temp);
}

void print_string(uint8_t* data,int length) {
	//char* temp[200];
	//memcpy(&temp,data,length);
	printf("%s",data);
}

/*
Gets the first device for our USB vendor ID.
?puts all RoboteX devices into an array?
*/
int setup_usb_device(void) {
   printf("beginning of setup_usb_device\n");
   libusb_init(NULL);
   // get list of USB devices

   num_devices = libusb_get_device_list(NULL, &devs);

   #ifdef DEBUG
      printf("num_devices=%i",num_devices);
   #endif


   // check for Robotex device

   for (ssize_t t = 0; t < num_devices; t++)
   {

      libusb_get_device_descriptor(devs[t], &desc);
	  #ifdef PRINT_ALL_USB
		printf("Vendor: %04x Product: %04x\n",desc.idVendor,desc.idProduct);
	
	  #endif
	  
      if (desc.idVendor == 0x2694)
      {

         #ifdef DEBUG
            printf("RoboteX Device\n");
            printf("\tdevice.idProduct: %04x\n", desc.idProduct);
            printf("\tbus: %d\n", libusb_get_bus_number(devs[t]));
            printf("\tdevice: %d\n", libusb_get_device_address(devs[t]));
            printf("\tname: ");
         switch (desc.idProduct)
         {
            case 0x0:
               printf("Robotex Generic PIC\n");
               break;
            case 0x1:
               printf("Robotex OCU\n");
               break;
            case 0x2:
               printf("Robotex Avatar Carrier Board\n");
               break;
            case 0x3:
               printf("Robotex Motor Controller Board\n");
               break;
            case 0x4:
               printf("Robotex Arm Base Controller\n");
               break;
            case 0x5:
               printf("Robotex Arm Shoulder Controller\n");
               break;
            case 0x6:
               printf("Robotex Arm Hand Controller\n");
               break;
	case 0x0f:
		printf("Hitch\r\n");
	break;
            default:
               printf("unsupported device\n");
               continue;
         }
         #endif
         //ASSERT( libusb_open(devs[t], &device_handle) );
         libusb_open(devs[t], &device_handle);
         robotex_device_handles.push_back(device_handle);
      }
   }
   libusb_free_device_list(devs, 1);

   if (robotex_device_handles.size() == 0)
   {
      printf("no devices found\n");
      return 0;
   }

   // do stuff with devices

   if( libusb_kernel_driver_active(robotex_device_handles[0], 0) == 1)
   {
      printf("kernel driver already attached\n");

      if( libusb_detach_kernel_driver(robotex_device_handles[0], 0) == 0)
      {
         printf("kernel driver successfully detached\n");
      }
      else
      {
         printf("unable to detach kernel driver\n");
         return 0;
      }
   }

//   ASSERT( libusb_set_configuration(robotex_device_handles[0], 1) );
   libusb_set_configuration(robotex_device_handles[0], 1);

//   ASSERT( libusb_claim_interface(robotex_device_handles[0], 0) );
   libusb_claim_interface(robotex_device_handles[0], 0);

   #ifdef DEBUG
      printf("\n%i device(s) opened\n\n", robotex_device_handles.size());
   #endif
   outgoing_transfer = libusb_alloc_transfer(1);

 //  ASSERT( outgoing_transfer != 0 );

   libusb_fill_iso_transfer(outgoing_transfer,           // transfer
                            robotex_device_handles[0],   // dev_handle
                            0x01,                        // endpoint
                            (unsigned char*)&out_packet, // buffer
                            sizeof(OUT_PACKET),          // length
                            1,                           // num_iso_packets
                            outgoing_callback,                           // callback
                            0,                           // user_data
                            //0);                          // timeout
                            0);
  
  // TO CONFIGURE LIBUSB 
  libusb_set_iso_packet_lengths(outgoing_transfer, sizeof(OUT_PACKET));
   incoming_transfer = libusb_alloc_transfer(1);
  // ASSERT( incoming_transfer != 0 );

   libusb_fill_iso_transfer(incoming_transfer,           // transfer
                            robotex_device_handles[0],   // dev_handle
                            0x81,                        // endpoint
                            (unsigned char*)&in_packet,  // buffer
                            sizeof(IN_PACKET),           // length
                            1,                           // num_iso_packets
                            incoming_callback,    // callback
                            0,                           // user_data
                            0);                          // timeout
   libusb_set_iso_packet_lengths(incoming_transfer, sizeof(IN_PACKET));
   printf("end of usb setup\n");
}

int kbhit(void) {
	int ch = getch();
	if(ch != ERR){
		ungetch(ch);
		return 1;
	} else {
	return 0;
	}
}

char kbhit2(void){
	printf("kbhit2 \r\n");
	int ch = getchar();
	printf("getchar done");
	int last_ch = 0xff;
	while(ch != EOF)
	{
		printf("in while loop");
		last_ch = ch;
		ch = getchar();
		printf("%x   ",ch);
	}
	printf("end kbhit2");
	return (char)last_ch;

}


char kbhit3(void){

	int ch = getch();

	int last_ch = 0xff;

	refresh();
	while(ch != ERR)
	{
		last_ch = ch;
		ch = getch();

	}
	return last_ch;

}

void cycle_n_times(int times_to_cycle) {
	int cycles = 0;
	int i,j;
	char reg_hitch_open = 0;
	int cycles_to_add = 0;
	int attempted_cycles = 0;

	int current_position = 0;
	int last_position = 0;

	int expected_hitch_position = 100;

	int hitch_position_index = return_register_index(&telemetry::REG_HITCH_POSITION);
	int hitch_open_index = return_register_index(&telemetry::REG_HITCH_OPEN);

	int position_failure_counter = 0;

	//for user input
	//setup_ncurses();

	sleep(5); // in units of seconds
	while(1)
	{

		//refresh();

			/*if(kbhit3() == 'p')
			{
				printf("\r\n\r\n Program Paused.  Press 's' to start.\r\n");
				while(1)
				{
					if(kbhit3() == 's')
						break;
				}
				printf("\r\n Program Resumed.\r\n");
			}*/





		cycles_to_add = 1;

		reg_hitch_open = 1;
		expected_hitch_position = 0;
		for(i=0;i<2;i++)
		{

			// build the packet to send to firmware
			for(j=0;j<50;j++)
			{
				out_packet[0] = hitch_position_index;	// position into usb_register index
				out_packet[1] = 0x80; // read index
				out_packet[2] = hitch_open_index;
				out_packet[3] = 0x00; // write index
				out_packet[4] = reg_hitch_open;	// character
				out_packet[5] = 0xff;
				out_packet[6] = 0xff;
				out_packet[7] = 0xff;

				//if the USB message fails, don't count this cycle
				if(!handle_usb_communication())
				{
					//print values in case program fails unexpectedly
					printf("\r\nUSB Error:  cycles: %d, attempted cycles: %d \r\n",cycles,attempted_cycles);
					//due to USB error, don't count this as a cycle
					cycles_to_add = 0;
				}
				usleep(100000);
				if(in_packet[2] == expected_hitch_position)
					break;
			}

			current_position = in_packet[2];
			//leave funciton if the hitch has been stuck for the last three cycles
			if(last_position == current_position)
			{
				if(position_failure_counter >=5)
				{
					printf("Breaking due to position.  Position is %d\r\n",in_packet[2]);
					break;
				}
				position_failure_counter++;
	
				//don't count this as a cycle
				cycles_to_add = 0;
			}
			else
				position_failure_counter = 0;

			last_position = current_position;

			reg_hitch_open = 0;
			expected_hitch_position = 100;
			sleep(1);
		}


	cycles+=cycles_to_add;	
	attempted_cycles++;

	printf("\r\n\r\nPosition is: %d\r\n",in_packet[2]);
	printf("Cycles completed: %d\r\n",cycles);
	printf("Cycles attempted: %d\r\n",attempted_cycles);
	


		if(cycles >= times_to_cycle)
			break;
		
	
	}
	printf("Leaving cycle function\r\n");
	//cleanup_ncurses();

}

void hitch_control_through_keyboard(void)
{
	uint8_t reg_hitch_open = 0;
	
	int hitch_position_index = return_register_index(&telemetry::REG_HITCH_POSITION);
	int hitch_open_index = return_register_index(&telemetry::REG_HITCH_OPEN);
	char new_char = 0xff;

	printf("Starting hitch control through keyboard\r\n");

	//setup ncurses for getting keypress
	initscr();
	cbreak();
	noecho();
	nodelay(stdscr, TRUE);
	scrollok(stdscr, TRUE);


	while(1)
	{
	
		//doupdate();
		refresh();
		if(kbhit())
		{
			new_char = getch();
			if(new_char == 'o')
			{
				reg_hitch_open = 1;
			}
			else if(new_char == 'c')
			{
				reg_hitch_open = 0;
			}
			else if (new_char == 'q')
			{
				break;
			}
		}
		//note that ints are LS byte first, MS byte second
		out_packet[0] = hitch_position_index;
		out_packet[1] = 0x80;
		out_packet[2] = hitch_open_index;
		out_packet[3] = 0x00;
		out_packet[4] = reg_hitch_open;
		out_packet[5] = 0xff;
		out_packet[6] = 0xff;
		out_packet[7] = 0xff;
		
		printf("REG_HITCH_POSITION is %d, REG_HITCH_OPEN is %d \r\n",in_packet[2], reg_hitch_open);
	

		handle_usb_communication();
	
		sleep(.1);

	
	}

	printf("Ending hitch control through keyboard \r\n");

	echo();
	nocbreak();

}

//called every time a USB message should be sent and received
bool handle_usb_communication(void)
{

	int temp = 0;
	//printf("1 ");
	temp = libusb_submit_transfer(outgoing_transfer);


	if(temp < 0)
	{
		reinit_USB();
		return 0;
	}


	//printf("2: %i ", temp);
	temp = libusb_handle_events(NULL);


	//printf("3: %i",temp);
	temp = libusb_submit_transfer(incoming_transfer); 
	//printf("4: %i",temp);
	temp = libusb_handle_events(NULL);
	//printf("5: %i\r\n",temp);

	return 1;
}

//to be called when program exits
void clean_up_USB(void)
{
   libusb_release_interface(robotex_device_handles[0], 0);
   
   libusb_free_transfer(incoming_transfer);

   libusb_free_transfer(outgoing_transfer);

   for (size_t t = 0; t < robotex_device_handles.size(); t++)
      libusb_close(robotex_device_handles[t]);

   libusb_exit(NULL);

	//remove device from vector
  robotex_device_handles.pop_back();

}


void incoming_callback(struct libusb_transfer *transfer) {
	//printf("IN_C ");
}

void outgoing_callback(struct libusb_transfer *transfer) {
	//printf("OUT_C\r\n");
}

void reinit_USB(void) {
	int retry_counter = 0;
	printf("\r\n Reinitializing USB \r\n");
	clean_up_USB();
	//keep trying to init until the device is found
	while(setup_usb_device()==0)
	{
		//usleep(100000);
		sleep(1);
		if(retry_counter > 10)
		{
			printf("USB reinit failed\r\n");
			break;
		}
		retry_counter++;
	}

}

void setup_ncurses(void)
{

//setup ncurses for getting keypress
	initscr();
	cbreak();
	//noecho();
	nodelay(stdscr, TRUE);
	scrollok(stdscr, TRUE);
	//echo();
	//nocbreak();
}


//so the terminal isn't messed up when the program ends
void cleanup_ncurses(void)
{

	refresh();

	echo();
	nocbreak();

}
