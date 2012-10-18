/*==============================================================================
File: ArmTest.cpp

Description:  This file facilitates the testing and debugging during assembly
  of the arm.  It streams data from each of the sensors and provides an 
  interface to control the arm through an Xbox controller.

Notes:
  - make sure to run shell script to suppress USB bug (forces device to be
    full-speed => 12-megabit)

Responsible Engineer(s): 
Taylor Penn taylor@robotex.com | Stellios Leventis sleventis@robotex.com
==============================================================================*/
//#define DEBUGGING_MODE
/*---------------------------Dependencies-------------------------------------*/
#include <iostream>     // for cin, user input, etc
#include <sys/types.h>
#include <libusb-1.0/libusb.h>
#include <vector>
#include <unistd.h>
#include <cstdlib>
#include <stdio.h>
#include <time.h>
#include "./firmware/usb_config.h" // for our register definitions
//#include "./SDL/SDL.h"	// for Xbox controller inteface
#include <time.h> 	    // for timing USB polling
#include "unistd.h"	    // for blocking sleep() function
//#include "./XboxController.hpp"			// for Xbox controller interface

/*---------------------------Macros-------------------------------------------*/
#define CLUTCH_OFFSET           193 // [au], A/D counts when clutch is centered
                                    // used for computing slip
#define DEFAULT_GRIPPER_OPEN_SPEED  20
#define DEFAULT_GRIPPER_CLOSE_SPEED -20

// USB-protocol-related
#define RBX_INVALID_SENSOR_DATA 0xff
#define RBX_VENDOR_ID           0x2694
#define PACKET_SIZE             16    // 50
#define FOOTER_L                0xff
#define FOOTER_H                0xff


/*  USB Protocol.  Each determines an character (8-bits)
A
A
TURRET_ENCODER_L
TURRET_ENCODER_H
SHOULDER_ENCODER_L
SHOULDER_ENCODER_H
ELBOW_ENCODER_L
ELBOW_ENCODER_H
WRIST_ENCODER_L
WRIST_ENCODER_H
A
A
A
A
FOOTER_L 
FOOTER_H
*/

/*---------------------------Type Definitions---------------------------------*/
// from the perspective of software
typedef uint8_t in_packet_t[240];
typedef uint8_t out_packet_t[240];

using namespace std;
using namespace rbx;
using namespace rbx::telemetry;

/*---------------------------Constants----------------------------------------*/
/*---------------------------Module Variables---------------------------------*/
// USB-related variables
unsigned int register_indices[100];
char *register_names[100];
unsigned int register_data_lengths[100];
ssize_t num_devices;

// global variables for libusb stuff
libusb_device **devs;
struct libusb_device_descriptor desc;
vector<libusb_device_handle*> robotex_device_handles;
libusb_device_handle *device_handle;
libusb_transfer *incoming_transfer;
libusb_transfer *outgoing_transfer;
in_packet_t in_packet;              // from firmware
out_packet_t out_packet;            // to firmware

/*---------------------------Helper Function Prototypes-----------------------*/
int Map(int value, int fromLow, int fromHigh, int toLow, int toHigh);
//void BuildPacket(XboxController *pController);
void PressEnterToContinue(void);

//static void UpdateControllerValues(XboxController *pController);
static void PrintFirmwareFeedback(void);
unsigned int return_checksum(unsigned char* data, unsigned char length);

// our-USB-protocol-related
static void InitRoboteXDevice(void);
static int GetRegisterIndex(void *pRegister);

// USB-related
static void PrintUSBErrorString(int errorCode);
static int InitUSBDevice(void);
static void CleanupUSB(void);
static void ReinitUSB(void);

// communication-protocol-related
static int HandleUSBCommunication(void);
//---to satisfy libusb module, empty function
void incoming_callback(struct libusb_transfer *transfer);
void outgoing_callback(struct libusb_transfer *transfer);

void TurnOnCameraPower(void);
void print_battery_registers(void);
void test_battery_registers(void);
void print_gps_data(void);

int main(int argn, char *argc[]) {
  //XboxController myController;

  char* input_argument = argc[1];

  //PressEnterToContinue();

  if(input_argument[0] == 'p')
  {
    TurnOnCameraPower();
  }
  else if(input_argument[0] == 'b')
  {
    print_battery_registers();
  }
  else if(input_argument[0] == 't')
  {
    test_battery_registers();
  }
  else if(input_argument[0] == 'g')
  {
    print_gps_data();
  }
 
  return EXIT_SUCCESS;
}


/*---------------------------Helper Function Definitions----------------------*/
static void InitRoboteXDevice(void) {


  // search until the device is found through USB interface  
  // make sure to break if the "back" button is pressed on the controller
  while (!InitUSBDevice()) {
	      //UpdateControllerValues(&myController);
  }
}


/*
Description: Updates the module-level buffers that store the controller data
*/



/*******TODO: move this to register/firmware-related file*******/
/*
Description: Linearly searches over the registers array looking for a pointer
  match.
*/
static int GetRegisterIndex(void *pRegister) {
	unsigned int i = 0;
	while (registers[i].ptr) {
		if (registers[i].ptr == pRegister) return i;
		i++;
	}

	printf("Register not found\r\n");
	return 0; 
}

/*---------------------------TODO: move this to general USB module------------*/
/*
Gets the first device for our USB vendor ID.
?puts all RoboteX devices into an array?
TODO: decompose this function
*/
static int InitUSBDevice(void) {
  unsigned char is_unsupported_device = 0;
	int errorCode = 0;
  printf("\r\nSetting up USB device... ");
  libusb_init(NULL);
  
  // get list of USB devices
  num_devices = libusb_get_device_list(NULL, &devs);

  // check for Robotex device
  for (ssize_t t = 0; t < num_devices; t++) {
    libusb_get_device_descriptor(devs[t], &desc);
	  
    if (desc.idVendor == RBX_VENDOR_ID) {
	    is_unsupported_device = 0;
      printf("\r\nRoboteX Device\n");
      printf("\tdevice.idProduct: %04x\n", desc.idProduct);
      printf("\tbus: %d\n", libusb_get_bus_number(devs[t]));
      printf("\tdevice: %d\n", libusb_get_device_address(devs[t]));
      printf("\tname: ");
      switch (desc.idProduct) {
        case 0x00: printf("Robotex Generic PIC\n"); break;
        case 0x01: printf("Robotex OCU\n"); break;
        case 0x02: printf("Robotex Avatar Carrier Board\n"); break;
        case 0x03: printf("Robotex Motor Controller Board\n"); break;
        case 0x04: printf("Robotex Arm Base Controller\n"); break;
        case 0x05:
          is_unsupported_device = 1;
          printf("PTZ Fixed Board \n");
          break;
        case 0x06: printf("Robotex Arm Hand Controller\n"); break;
	      case 0x07:
		      is_unsupported_device = 1;
		      printf("PTZ Camera\r\n");
	        break;
	      case 0x08 :
		      is_unsupported_device = 1;
		      printf("Arm EMPIA");
	        break;
	      case 0x0a: printf("PTZ Rotation Board\r\n"); break;
	      case 0x0d: printf("Arm\r\n"); break;
	      case 0x0f: printf("Hitch\r\n"); break;
        default:
          printf("unsupported device\n");
		      is_unsupported_device = 1;
          continue;
      }

      if (!is_unsupported_device) {
        errorCode = libusb_open(devs[t], &device_handle);
	      printf("added device %x:%x, returned %i\r\n", desc.idVendor, desc.idProduct, errorCode);
        robotex_device_handles.push_back(device_handle);
	    } else {
		    printf("not adding unsupported device\r\n");
	    }
    }
  }

  libusb_free_device_list(devs, 1);

  if (robotex_device_handles.size() == 0) {
    printf("no devices found\n");
    return 0;
  } else {
		printf("found %i devices \r\n", robotex_device_handles.size());
	}

  // do stuff with devices


  if (libusb_kernel_driver_active(robotex_device_handles[0], 0)) {
    printf("kernel driver already attached\n");

    if (!libusb_detach_kernel_driver(robotex_device_handles[0], 0)) {
      printf("kernel driver successfully detached\n");
    } else {
      printf("unable to detach kernel driver\n");
      return 0;
    }
  } else {
		printf("kernel driver not attached\r\n");
	}

  errorCode = libusb_set_configuration(robotex_device_handles[0], 1);
	printf("libusb_set_configuration: %i\r\n", errorCode);

	errorCode = libusb_claim_interface(robotex_device_handles[0], 0);
	printf("libusb_claim_interface: %i\r\n", errorCode);

  outgoing_transfer = libusb_alloc_transfer(1);
  libusb_fill_iso_transfer(outgoing_transfer,           // transfer
                            robotex_device_handles[0],  // dev_handle
                            0x01,                       // endpoint
                            (uint8_t*)&out_packet,      // buffer
                            sizeof(out_packet_t),       // length
                            1,                          // num_iso_packets
                            outgoing_callback,          // callback
                            0,                          // user_data
                            0);                         // timeout
  
  // configure libusb 
  libusb_set_iso_packet_lengths(outgoing_transfer, sizeof(out_packet_t));
  incoming_transfer = libusb_alloc_transfer(1);

  libusb_fill_iso_transfer(incoming_transfer,           // transfer
                           robotex_device_handles[0],   // dev_handle
                           0x81,                        // endpoint
                           (uint8_t*)&in_packet,        // buffer
                           sizeof(in_packet_t),         // length
                           1,                           // num_iso_packets
                           incoming_callback,           // callback
                           0,                           // user_data
                           0);                          // timeout
  libusb_set_iso_packet_lengths(incoming_transfer, sizeof(in_packet_t));
  printf("end of usb setup\n");
}


/*
Description: Called each time a USB message is sent or received.  Returns an
  error code if there was a hiccup, 1 otherwise.
*/
static int HandleUSBCommunication(void) {
	timeval wait_time;
	wait_time.tv_sec = 1;
	wait_time.tv_usec = 0;
	int temp = libusb_submit_transfer(outgoing_transfer);

	if (temp < 0) {
		PrintUSBErrorString(temp);

    if (temp == LIBUSB_ERROR_NO_DEVICE || 
        temp == LIBUSB_ERROR_NOT_FOUND) {
      return 0;
    }
	}
	
  temp = libusb_handle_events_timeout(NULL, &wait_time);
	temp = libusb_submit_transfer(incoming_transfer);	
  temp = libusb_handle_events_timeout(NULL, &wait_time);

	return 1;
}

static void PrintUSBErrorString(int errorCode) {
	switch (errorCode) {
		case LIBUSB_ERROR_IO:	
    	printf("libusb error error IO: %d\r\n", LIBUSB_ERROR_IO);
			break;
		case LIBUSB_ERROR_INVALID_PARAM:
    	printf("libusb error invalid parameter: %d\r\n", LIBUSB_ERROR_INVALID_PARAM);
			break;
    case LIBUSB_ERROR_ACCESS:
			printf("libusb error access: %d\r\n", LIBUSB_ERROR_ACCESS);
			break;
    case LIBUSB_ERROR_NO_DEVICE:
			printf("libusb error error no device: %d\r\n", LIBUSB_ERROR_NO_DEVICE);
		  break;
    case LIBUSB_ERROR_NOT_FOUND:
			printf("libusb error error not found: %d\r\n", LIBUSB_ERROR_NOT_FOUND);
      break;
    case LIBUSB_ERROR_BUSY:
		  printf("libusb error busy: %d\r\n", LIBUSB_ERROR_BUSY);
      break;
    case LIBUSB_ERROR_TIMEOUT:
		  printf("libusb error error timeout: %d\r\n", LIBUSB_ERROR_TIMEOUT);
      break;
    case LIBUSB_ERROR_OVERFLOW:
		  printf("libusb error error overflow: %d\r\n", LIBUSB_ERROR_OVERFLOW);
      break;
    case LIBUSB_ERROR_PIPE:
		  printf("libusb error error pipe: %d\r\n", LIBUSB_ERROR_PIPE);
      break;
    case LIBUSB_ERROR_INTERRUPTED:
		  printf("libusb error interrupted: %d\r\n", LIBUSB_ERROR_INTERRUPTED);
      break;
    case LIBUSB_ERROR_NO_MEM:
		  printf("libusb error error no memory: %d\r\n", LIBUSB_ERROR_NO_MEM);
      break;
    case LIBUSB_ERROR_NOT_SUPPORTED:
		  printf("libusb error error not supported: %d\r\n", LIBUSB_ERROR_NOT_SUPPORTED);
      break;
    case LIBUSB_ERROR_OTHER:
		  printf("libusb error error other: %d\r\n", LIBUSB_ERROR_OTHER);
      break;
    default:
      printf("Unrecognized USB error");
      break;
	}
}

static void CleanupUSB(void) {
  libusb_release_interface(robotex_device_handles[0], 0); 
  libusb_free_transfer(incoming_transfer);
  libusb_free_transfer(outgoing_transfer);

  for (size_t t = 0; t < robotex_device_handles.size(); t++) {
    libusb_close(robotex_device_handles[t]);
  }

  libusb_exit(NULL);
	robotex_device_handles.pop_back(); // remove device from vector
}

// callback functions for libusb, useful for debugging
void incoming_callback(struct libusb_transfer *transfer) {
	//printf("IN_C ");
}


void outgoing_callback(struct libusb_transfer *transfer) {
	//printf("OUT_C\r\n");
}


static void ReinitUSB(void) {
	int attemptCounter = 0;
	printf("\r\nReinitializing USB...");
	CleanupUSB();
	
  // keep trying until the device is found
	while (InitUSBDevice() == 0) {
		sleep(1);
		if (10 < attemptCounter) {
			printf("USB reinitialization failed\r\n");
			break;
		}
		attemptCounter++;
	}

  printf("USB reinitialization succeeded\r\n");
}



/************TODO: move this to general utility function module****************/
void PressEnterToContinue(void) {
  // TODO: guard against invalid input
  printf("\r\n\r\nEnter any key to continue: ");    
  unsigned char input;
  cin >> input;
  cin.ignore();
}

void print_battery_registers(void) {

  int rel_soc_left_index = GetRegisterIndex(&telemetry::REG_OCU_REL_SOC_L);
  int rel_soc_right_index = GetRegisterIndex(&telemetry::REG_OCU_REL_SOC_R);
  int ocu_batt_current_index = GetRegisterIndex(&telemetry::REG_OCU_BATT_CURRENT);
  int abs_soc_right_index = GetRegisterIndex(&telemetry::REG_OCU_BATT_ABS_SOC);
  int rel_soc_left = 0;
  int rel_soc_right = 0;
  int16_t batt_current = 0;
  int abs_soc_right = 0;

  unsigned int checksum;

  InitRoboteXDevice();

  while(1)
  {

    out_packet[0] = rel_soc_left_index;
    out_packet[1] = 0x80;
    out_packet[2] = rel_soc_right_index;
    out_packet[3] = 0x80;
    out_packet[4] = ocu_batt_current_index;
    out_packet[5] = 0x80;
    out_packet[6] = abs_soc_right_index;
    out_packet[7] = 0x80;
    out_packet[8] = 0xff;
    out_packet[9] = 0xff;

    checksum = return_checksum(out_packet,10);

    out_packet[10] = checksum&0xff;
    out_packet[11] = checksum>>8;

    if (!HandleUSBCommunication())
      return;

    rel_soc_left = in_packet[2]+in_packet[3]*256;
    rel_soc_right = in_packet[6]+in_packet[7]*256;
    batt_current = in_packet[10]+in_packet[11]*256;
    abs_soc_right = in_packet[14]+in_packet[15]*256;

    printf("REG_OCU_REL_SOC_L:  %i\r\n",rel_soc_left);
    printf("REG_OCU_REL_SOC_R:  %i\r\n",rel_soc_right);
    printf("REG_OCU_BATT_CURRENT:  %i\r\n",batt_current);
    printf("abs_soc_right:  %i\r\n",abs_soc_right);  
    printf("\r\n\r\n\r\n");

    if( (rel_soc_left <= 11 && rel_soc_left > 0) || (rel_soc_right <=11 && rel_soc_right > 0) )
      system("sudo shutdown -h now");

    usleep(100000);

  }

}

void test_battery_registers(void) {

  int rel_soc_left_index = GetRegisterIndex(&telemetry::REG_OCU_REL_SOC_L);
  int rel_soc_right_index = GetRegisterIndex(&telemetry::REG_OCU_REL_SOC_R);
  int ocu_batt_current_index = GetRegisterIndex(&telemetry::REG_OCU_BATT_CURRENT);
  int abs_soc_right_index = GetRegisterIndex(&telemetry::REG_OCU_BATT_ABS_SOC);
  int rel_soc_left = 0;
  int rel_soc_right = 0;
  int16_t batt_current = 0;
  int abs_soc_right = 0;
  int first_run_flag = 1;
  int rel_soc_left_initial = 0;
  int rel_soc_right_initial = 0;
  int batt_current_initial = 0;
  int rel_soc_diff_initial, rel_soc_diff_final,rel_soc_diff_left,rel_soc_diff_right;

  unsigned int checksum;
  
  /*struct tm *current;
  time_t now;
  time(&now);
  current = localtime(&now);*/
  
  timeval thetime;
  gettimeofday(&thetime,NULL);
  int last_s = thetime.tv_sec;
  int this_s = 0;
  int elapsed_s;

int stop_printing = 0;

  InitRoboteXDevice();

  while(1)
  {

    out_packet[0] = rel_soc_left_index;
    out_packet[1] = 0x80;
    out_packet[2] = rel_soc_right_index;
    out_packet[3] = 0x80;
    out_packet[4] = ocu_batt_current_index;
    out_packet[5] = 0x80;
    out_packet[6] = abs_soc_right_index;
    out_packet[7] = 0x80;
    out_packet[8] = 0xff;
    out_packet[9] = 0xff;

    checksum = return_checksum(out_packet,10);

    out_packet[10] = checksum&0xff;
    out_packet[11] = checksum>>8;

    if (!HandleUSBCommunication())
	{
	  printf("USB Communication Error\r\n");
	}
	else
	{

		rel_soc_left = in_packet[2]+in_packet[3]*256;
		rel_soc_right = in_packet[6]+in_packet[7]*256;
		batt_current = in_packet[10]+in_packet[11]*256;
		abs_soc_right = in_packet[14]+in_packet[15]*256;
		
		if(first_run_flag)
		{
			rel_soc_left_initial = rel_soc_left;
			rel_soc_right_initial = rel_soc_right;
			batt_current_initial = batt_current;
			first_run_flag = 0;
		}
		if(stop_printing == 0)
		{
		printf("REG_OCU_REL_SOC_L:  %i\r\n",rel_soc_left);
		printf("REG_OCU_REL_SOC_R:  %i\r\n",rel_soc_right);
		printf("REG_OCU_BATT_CURRENT:  %i\r\n",batt_current);
		printf("abs_soc_right:  %i\r\n",abs_soc_right);  
		printf("\r\n\r\n\r\n");
		}

		if( (rel_soc_left <= 11 && rel_soc_left > 0) || (rel_soc_right <=11 && rel_soc_right > 0) )
		  system("sudo shutdown -h now");
		  
		gettimeofday(&thetime,NULL);
		this_s = thetime.tv_sec;

		elapsed_s = this_s - last_s;
		
		//if 10 minutes has elapsed
		if( (elapsed_s > 600) && (stop_printing == 0))
		{
		
			rel_soc_diff_initial = abs(rel_soc_left_initial-rel_soc_right_initial)*100/ ( (rel_soc_left_initial+rel_soc_right_initial)/2);
			rel_soc_diff_final = abs(rel_soc_left-rel_soc_right)*100/ ( (rel_soc_left+rel_soc_right)/2);
			rel_soc_diff_left = rel_soc_left - rel_soc_left_initial;
			rel_soc_diff_right = rel_soc_right - rel_soc_right_initial;
			printf("TEST COMPLETE\r\n\r\n");
			if( (rel_soc_left == 100) || (rel_soc_left == 0) || (rel_soc_left == 0xffff) )
				printf("LEFT BATTERY LIKELY BAD\r\n");
			if( (rel_soc_right == 100) || (rel_soc_right == 0) || (rel_soc_right == 0xffff) )
				printf("RIGHT BATTERY LIKELY BAD\r\n\r\n");		
			printf("Left battery start: %i%%     End: %i%%   Diff: %i%%\r\n",rel_soc_left_initial,rel_soc_left,rel_soc_diff_left);
			printf("Right battery start: %i%%     End: %i%%   Diff: %i%%\r\n",rel_soc_right_initial,rel_soc_right,rel_soc_diff_right);
			printf("Battery start difference: %i%%\r\n",rel_soc_diff_initial);
			printf("Battery end difference: %i%%\r\n",rel_soc_diff_final);
			
		stop_printing = 1;
		}
	}

    usleep(100000);

  }

}

void TurnOnCameraPower(void)  {
  
  int i;
	int camera_power_index = GetRegisterIndex(&telemetry::REG_OCU_CAMERA_POWER_ON);
  unsigned int checksum;
	
	        InitRoboteXDevice();

        printf("\r\n\r\n\r\nTurning on power to cameras...\r\n\r\n");

	for(i=0;i<30;i++)
	{
		out_packet[0] = camera_power_index;
		out_packet[1] = 0x00;
		out_packet[2] = 0x01;
		out_packet[3] = 0xff;
		out_packet[4] = 0xff;
		
		//hard code checksum -- 107+1+255+255 = 618 = 0x026a
		//out_packet[5] = 0x6a;
		//out_packet[6] = 0x02;

         checksum = return_checksum(out_packet,5);
    out_packet[5] = checksum&0xff;
    out_packet[6] = checksum>>8;

		if (!HandleUSBCommunication())
			return;

            usleep(100000);

	}
	  printf("Cameras powered.\r\n");
  
  
  }
  
  void print_gps_data(void)
  {
	int gps_data_index = GetRegisterIndex(&telemetry::REG_OCU_GPS_MESSAGE);
	unsigned int checksum;
	unsigned int i;
	InitRoboteXDevice();

	out_packet[0] = gps_data_index;
    out_packet[1] = 0x80;
	out_packet[2] = 0xff;
	out_packet[3] = 0xff; 
	checksum = return_checksum(out_packet,4);
	out_packet[4] = checksum&0xff;
    out_packet[5] = checksum>>8;
	
	while(1)
	{
			if (!HandleUSBCommunication())
			return;
			for(i=0;i<50;i++)
			{
				printf("%c",in_packet[i]);
			}
	
	}
	
	
  }


int Map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
  // compute the linear interpolation
  int result = ((double)(value - fromLow) / (double)(fromHigh - fromLow))
               * (double)(toHigh - toLow) + toLow;

  // constrain the result to within a valid output range
  if (toHigh < result) result = toHigh;
  else if (result < toLow) result = toLow;

  return result;
}

unsigned int return_checksum(unsigned char* data, unsigned char length)
{
  unsigned char i;
  unsigned int checksum = 0;
  for(i=0;i<length;i++)
  {
    checksum+=data[i];
  }

  return checksum;
}





