/*==============================================================================
File: ArmTest.cpp

Description:  This file facilitates the testing and debugging during assembly
  of the arm.  It streams data from each of the sensors and provides an 
  interface to control the arm through an Xbox controller.

Notes:
  - make sure to run shell script to suppress USB bug (forces device to be
    full-speed => 12-megabit)

Responsible Engineer: Stellios Leventis sleventis@robotex.com
==============================================================================*/
//#define DEBUGGING_MODE
/*---------------------------Dependencies-------------------------------------*/
#include <iostream>     // for cin, user input, etc
#include <sys/types.h>
#include <libusb-1.0/libusb.h>
#include <vector>
#include <unistd.h>
#include <cstdlib>
#include "./firmware/usb_config.h" // for our register definitions
#include "./SDL/SDL.h"	// for Xbox controller inteface
#include <time.h> 	    // for timing USB polling
#include "unistd.h"	    // for blocking sleep() function
#include "./XboxController.hpp"			// for Xbox controller interface

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


// incoming data packet breakdown
#define TURRET_ENCODER_L_INDEX    2
#define TURRET_ENCODER_H_INDEX    3
#define SHOULDER_ENCODER_L_INDEX  4
#define SHOULDER_ENCODER_H_INDEX  5
#define ELBOW_ENCODER_L_INDEX     6
#define ELBOW_ENCODER_H_INDEX     7
#define WRIST_ENCODER_L_INDEX     8
#define WRIST_ENCODER_H_INDEX     9
#define GRIPPER_ACT_POS_L_INDEX   10
#define GRIPPER_ACT_POS_H_INDEX   11
#define GRIPPER_POS_L_INDEX       12
#define GRIPPER_POS_H_INDEX       13
#define FOOTER_L_INDEX            14
#define FOOTER_H_INDEX            15

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
void BuildPacket(XboxController *pController);
void PressEnterToContinue(void);

static void UpdateControllerValues(XboxController *pController);
static void PrintFirmwareFeedback(void);

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

int main(int argn, char *argc[]) {
  XboxController myController;

  PressEnterToContinue();

  while (true) {
    InitRoboteXDevice();

    while (true) {
      UpdateControllerValues(&myController);
      BuildPacket(&myController);
      if (!HandleUSBCommunication()) break;
      PrintFirmwareFeedback();
	  }
    
    CleanupUSB();
  }

  return EXIT_SUCCESS;
}


/*---------------------------Helper Function Definitions----------------------*/
static void InitRoboteXDevice(void) {
  // search until the device is found through USB interface  
  while (!InitUSBDevice());
}


/*
Description: Updates the module-level buffers that store the controller data
*/
static void UpdateControllerValues(XboxController *pController) {
	(*pController).getUpdatedValues();
  
  // map the values to a valid range if required
  (*pController).leftStick.x= Map((*pController).leftStick.x, 
																		MIN_AXIS_VALUE, MAX_AXIS_VALUE, -50, 50);
  (*pController).leftStick.y = Map((*pController).leftStick.y, 
																 		MIN_AXIS_VALUE, MAX_AXIS_VALUE, -50, 50);
  (*pController).rightStick.x= Map((*pController).rightStick.x, 
																 		MIN_AXIS_VALUE, MAX_AXIS_VALUE, -50, 50);
  (*pController).rightStick.y = Map((*pController).rightStick.y, 
																		MIN_AXIS_VALUE, MAX_AXIS_VALUE, -50, 50);

  // display the result to validate the sensor value
  printf("\r\nController\r\n");
  printf("--------------------\r\n");
  printf("'Y' Button: %d\r\n", (*pController).Y);
  printf("'A' Button: %d\r\n", (*pController).A);
  printf("leftStick.x: %d\r\n", (*pController).leftStick.x);
  printf("leftStick.y: %d\r\n", (*pController).leftStick.y);
  printf("rightStick.x: %d\r\n", (*pController).rightStick.x);
  printf("rightStick.y: %d\r\n", (*pController).rightStick.y);
  printf("\r\n");
}


static void PrintFirmwareFeedback(void) {
  if (in_packet[0] != 0x00) {
    #ifdef DEBUGGING_MODE
      printf("Incoming Packet: ");
      for (size_t i = 0; i < PACKET_SIZE; i++) printf("%x ", in_packet[i]);
      printf("\r\n");
    #endif

    // parse the incoming packet
    int turretEncoder = (in_packet[TURRET_ENCODER_H_INDEX] << 8) + in_packet[TURRET_ENCODER_L_INDEX];
    int shoulderEncoder = (in_packet[SHOULDER_ENCODER_H_INDEX] << 8) + 
                          in_packet[SHOULDER_ENCODER_L_INDEX];
    int elbowEncoder = (in_packet[ELBOW_ENCODER_H_INDEX] << 8) + in_packet[ELBOW_ENCODER_L_INDEX];
    int wristEncoder = (in_packet[WRIST_ENCODER_H_INDEX] << 8) + in_packet[WRIST_ENCODER_L_INDEX];
    
    int gripperActuatorPosition = (in_packet[GRIPPER_ACT_POS_H_INDEX] << 8) + 
                                  in_packet[GRIPPER_ACT_POS_L_INDEX];
	  int gripperPosition = (in_packet[GRIPPER_POS_H_INDEX] << 8) + 
                          in_packet[GRIPPER_POS_L_INDEX];
    int gripperClutchSlip = gripperPosition - gripperActuatorPosition - CLUTCH_OFFSET;

    // print the parsed information with context
    printf("\r\nJoint Angles\r\n");
    printf("--------------------\r\n");
    printf("turret: %d\r\n", turretEncoder);
    printf("shoulder: %d\r\n", shoulderEncoder);
    printf("elbow: %d\r\n", elbowEncoder);
    printf("wrist: %d\r\n", wristEncoder);
    
    printf("\r\nGripper\r\n");
    printf("--------------------\r\n");
	  printf("Position: %d\r\n", gripperPosition);
	  printf("Actuator Position: %d\r\n", gripperActuatorPosition);
	  printf("Slip: %d\r\n", gripperClutchSlip);
	}
}


void BuildPacket(XboxController *pController) {
  // map the controller data to arm data
  int turretSpeed = (*pController).leftStick.x;
  int shoulderSpeed = (*pController).leftStick.y;
  int elbowSpeed = (*pController).rightStick.y;
  int wristSpeed = (*pController).rightStick.x;
  int gripperSpeed = 0;
  gripperSpeed = (*pController).A ? DEFAULT_GRIPPER_OPEN_SPEED : gripperSpeed;  
  gripperSpeed = (*pController).Y ? DEFAULT_GRIPPER_CLOSE_SPEED : gripperSpeed;

  // only enable data on one axis per stick
  (abs(turretSpeed) < abs(shoulderSpeed)) ? (turretSpeed = 0) : (shoulderSpeed = 0);  
  (abs(elbowSpeed) < abs(wristSpeed)) ? (elbowSpeed = 0) : (wristSpeed = 0);


  // build the outgoing packet for the firmware
  int armSpeedIndex = GetRegisterIndex(&telemetry::REG_ARM_MOTOR_VELOCITIES);
	int armPositionIndex = GetRegisterIndex(&telemetry::REG_ARM_JOINT_POSITIONS);
  //---request arm joint data
  out_packet[0] = armPositionIndex;	       // position into usb_register index
  out_packet[1] = (1 << 7);                // read index

  //---transmit desired speeds
  out_packet[2] = armSpeedIndex;
  out_packet[3] = 0x00;                    // write index
  out_packet[4] = (turretSpeed & 0xff);    // lower byte of outgoing data
  out_packet[5] = (turretSpeed >> 8);      // upper byte of outgoing data
  out_packet[6] = (shoulderSpeed & 0xff);
  out_packet[7] = (shoulderSpeed >> 8);
  out_packet[8] = (elbowSpeed & 0xff);
  out_packet[9] = (elbowSpeed >> 8);
  out_packet[10] = (wristSpeed & 0xff);
  out_packet[11] = (wristSpeed >> 8);
  out_packet[12] = (gripperSpeed & 0xff);
  out_packet[13] = (gripperSpeed >> 8);
  out_packet[14] = FOOTER_L;
  out_packet[15] = FOOTER_H;
}

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
      return temp;
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


int Map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
  // compute the linear interpolation
  int result = ((double)(value - fromLow) / (double)(fromHigh - fromLow))
               * (double)(toHigh - toLow) + toLow;

  // constrain the result to within a valid output range
  if (toHigh < result) result = toHigh;
  else if (result < toLow) result = toLow;

  return result;
}





