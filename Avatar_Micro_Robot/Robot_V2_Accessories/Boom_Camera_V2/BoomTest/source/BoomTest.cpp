/*==============================================================================
File: BoomTest.cpp

Description:  This file facilitates the testing and debugging during assembly
  of the boom camera.  It streams video from the boom camera through VLC 
  player controlled by the user through an Xbox controller.

Notes:
  - make sure to run shell script to suppress USB bug (forces device to be
    full-speed => 12-megabit)

Responsible Engineer(s):
Taylor Penn taylor@robotex.com | Stellios Leventis sleventis@robotex.com
==============================================================================*/
#define DEBUGGING_MODE
/*---------------------------Dependencies-------------------------------------*/
#include <iostream>                 // for cin, user input, etc
#include <sys/types.h>
#include <libusb-1.0/libusb.h>
#include <vector>
#include <unistd.h>
#include <cstdlib>
#include "./firmware/usb_config.h"  // for our register definitions
#include "./SDL/SDL.h"              // for Xbox controller inteface
#include <time.h> 		              // for timing USB polling
#include "unistd.h"		              // for blocking sleep() function
#include "./XboxController.hpp"			// for Xbox controller interface

/*---------------------------Macros-------------------------------------------*/
// camera position limits
typedef enum {
  kMinPan = 0,
  kMaxPan = 359,
  kMinTilt = 0,
  kMaxTilt = 90,
  kMinZoom = 10,
  kMaxZoom = 130
} kCameraPositionLimit;

// OCU speed limits
typedef enum {
  kOCUJoystickMin = -1000,
  kOCUJoystickMax = 1000,
  kOCUToggleMin = -7,
  kOCUToggleMax = 7
} kOCUSpeedLimit;

// indications written to the msb (bit) of the two-byte usb address
// 1 = read, 0 = write from perspective of software (us)
typedef enum {
  kIndicationWrite = 0,
  kIndicationRead = 1
} kIndication;

// USB-protocol-related
#define RX_VENDOR_ID            0x2694  // RoboteX USB-registered vendor ID
#define FOOTER_L                0xff
#define FOOTER_H                0xff

/*---------------------------Type Definitions---------------------------------*/
typedef uint8_t in_packet_t[240];       // 'in' = from firmware
typedef uint8_t out_packet_t[240];      // 'out' = to firmware

using namespace std;
using namespace rbx;
using namespace rbx::telemetry;

/*---------------------------Constants----------------------------------------*/
/*---------------------------Module Variables---------------------------------*/
// USB-related variables
uint16_t register_indices[100];
int8_t *register_names[100];
uint16_t register_data_lengths[100];
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
int16_t Map(int16_t value, int16_t fromLow, int16_t fromHigh, 
            int16_t toLow, int16_t toHigh);
void BuildPacket(XboxController* pController);

static void UpdateControllerValues(XboxController* pController);

// our-USB-protocol-related
static void InitRoboteXDevice(void);
static int GetRegisterIndex(void* pRegister);

// USB-related
static void PrintUSBErrorString(int errorCode);
static int InitUSBDevice(void);
static void CleanupUSB(void);
static void ReinitUSB(void);

// communication-protocol-related
static int HandleUSBCommunication(void);
//---to satisfy libusb module, empty function
void incoming_callback(struct libusb_transfer* transfer);
void outgoing_callback(struct libusb_transfer* transfer);

void enable_debug_uart(void);

int main(int argn, char* argc[]) {
  XboxController myController;
  char* input_argument = argc[1];

  while (true) {
    InitRoboteXDevice();
    
    while (true) {
      UpdateControllerValues(&myController);
      BuildPacket(&myController);
      if (!HandleUSBCommunication()) break;
      if (myController.Back) return 0;
	  }
    
    CleanupUSB();
  }

  return EXIT_SUCCESS;
}


/*---------------------------Helper Function Definitions----------------------*/
static void InitRoboteXDevice(void) {
  XboxController myController;

  // search until the device is found through USB interface  
  // make sure to break if the "back" button is pressed on the controller
  while (!InitUSBDevice()) {
    UpdateControllerValues(&myController);
    BuildPacket(&myController);
    if (myController.Back) return;
  }
}


/*
Description: Updates the module-level buffers that store the controller data
*/
static void UpdateControllerValues(XboxController* pController) {
	(*pController).getUpdatedValues();
  
  // map the values to a valid range if required
  (*pController).leftStick.x = Map((*pController).leftStick.x,
                                   kMinAxisValue, kMaxAxisValue,
                                   kOCUJoystickMin, kOCUJoystickMax);
  (*pController).leftStick.y = Map((*pController).leftStick.y,
                                   kMinAxisValue, kMaxAxisValue,
                                   kOCUJoystickMin, kOCUJoystickMax);
  (*pController).rightStick.x = Map((*pController).rightStick.x,
                                    kMinAxisValue, kMaxAxisValue,
                                    kOCUToggleMin, kOCUToggleMax);

  // display the result to validate the controller sensor value
  printf("\r\nController\r\n");
  printf("--------------------\r\n");
  printf("leftStick.x: %d\r\n", (*pController).leftStick.x);
  printf("leftStick.y: %d\r\n", (*pController).leftStick.y);
  printf("rightStick.x: %d\r\n\r\n", (*pController).rightStick.x);
}


void BuildPacket(XboxController* pController) {
  // map the controller data to device data
  int16_t panSpeed = (*pController).leftStick.x;
  int16_t tiltSpeed = (*pController).leftStick.y;
  int16_t zoomSpeed = (*pController).rightStick.x;

  // only enable data on one axis per stick
  //(abs(panSpeed) < abs(tiltSpeed)) ? (tiltSpeed = 0) : (panSpeed = 0);
  
  // build the outgoing packet for the firmware
  int16_t boomPanSpeedIndex = GetRegisterIndex(&telemetry::REG_BOOM_VEL_PAN);
  int16_t boomTiltSpeedIndex = GetRegisterIndex(&telemetry::REG_BOOM_VEL_TILT);
  int16_t boomZoomSpeedIndex = GetRegisterIndex(&telemetry::REG_BOOM_VEL_ZOOM);
  //int16_t boomPowerIndex = GetRegisterIndex(&telemetry::REG_BOOM_POWER_DOWN);

  printf("--------------------\r\n");
  printf("panSpeed: %d\r\n", panSpeed);
  printf("tiltSpeed: %d\r\n", tiltSpeed);
  printf("zoomSpeed: %d\r\n\r\n", zoomSpeed);

  //---transmit desired data
  out_packet[0] = (boomPanSpeedIndex & 0xff);
  out_packet[1] = 0x00;//(boomPanSpeedIndex >> 8) | kIndicationWrite;
  out_packet[2] = (panSpeed & 0xff);    // lower byte of outgoing data
  out_packet[3] = (panSpeed >> 8);      // upper byte of outgoing data
  
  out_packet[4] = (boomTiltSpeedIndex & 0xff);
  out_packet[5] = 0x00;//(boomTiltSpeedIndex >> 8) | kIndicationWrite;
  out_packet[6] = (tiltSpeed & 0xff);   // lower byte of outgoing data
  out_packet[7] = (tiltSpeed >> 8);     // upper byte of outgoing data

  out_packet[8] = (boomZoomSpeedIndex & 0xff);
  out_packet[9] = 0x00;//(boomZoomSpeedIndex >> 8) | kIndicationWrite;
  out_packet[10] = (zoomSpeed & 0xff);   // lower byte of outgoing data
  out_packet[11] = (zoomSpeed >> 8);     // upper byte of outgoing data
  
  out_packet[12] = FOOTER_L;
  out_packet[13] = FOOTER_H;
}

/*******TODO: move this to register/firmware-related file*******/
/*
Description: Linearly searches over the registers array (see registers.h file)
  looking for a pointer match.
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
  bool is_unsupported_device = 0;
	int errorCode = 0;
  printf("\r\nSetting up USB device... ");
  libusb_init(NULL);
  
  // get list of USB devices
  num_devices = libusb_get_device_list(NULL, &devs);

  // check for Robotex device
  for (ssize_t t = 0; t < num_devices; t++) {
    libusb_get_device_descriptor(devs[t], &desc);
	  
    if (desc.idVendor == RX_VENDOR_ID) {
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
        case 0x12: 
          printf("Boom Camera\r\n");
          break;
        case 0x13:
          is_unsupported_device = 1;
          printf("Boom EMPIA Chip\r\n");
          break;
        default:
          printf("unsupported device\n");
		      is_unsupported_device = 1;
          continue;  // why continue?
      }

      if (!is_unsupported_device) {
        errorCode = libusb_open(devs[t], &device_handle);
	      printf("added device 0x%x:0x%x\r\n", desc.idVendor, desc.idProduct);
        PrintUSBErrorString(errorCode);
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
			printf("libusb error no device: %d\r\n", LIBUSB_ERROR_NO_DEVICE);
		  break;
    case LIBUSB_ERROR_NOT_FOUND:
			printf("libusb error not found: %d\r\n", LIBUSB_ERROR_NOT_FOUND);
      break;
    case LIBUSB_ERROR_BUSY:
		  printf("libusb error busy: %d\r\n", LIBUSB_ERROR_BUSY);
      break;
    case LIBUSB_ERROR_TIMEOUT:
		  printf("libusb error timeout: %d\r\n", LIBUSB_ERROR_TIMEOUT);
      break;
    case LIBUSB_ERROR_OVERFLOW:
		  printf("libusb error overflow: %d\r\n", LIBUSB_ERROR_OVERFLOW);
      break;
    case LIBUSB_ERROR_PIPE:
		  printf("libusb error pipe: %d\r\n", LIBUSB_ERROR_PIPE);
      break;
    case LIBUSB_ERROR_INTERRUPTED:
		  printf("libusb error interrupted: %d\r\n", LIBUSB_ERROR_INTERRUPTED);
      break;
    case LIBUSB_ERROR_NO_MEM:
		  printf("libusb error no memory: %d\r\n", LIBUSB_ERROR_NO_MEM);
      break;
    case LIBUSB_ERROR_NOT_SUPPORTED:
		  printf("libusb error not supported: %d\r\n", LIBUSB_ERROR_NOT_SUPPORTED);
      break;
    case LIBUSB_ERROR_OTHER:
		  printf("libusb error other: %d\r\n", LIBUSB_ERROR_OTHER);
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
void incoming_callback(struct libusb_transfer* transfer) {}
void outgoing_callback(struct libusb_transfer* transfer) {}


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


int16_t Map(int16_t value, int16_t fromLow, int16_t fromHigh, 
            int16_t toLow, int16_t toHigh) {
  // compute the linear interpolation
  int result = ((double)(value - fromLow) / (double)(fromHigh - fromLow))
               * (double)(toHigh - toLow) + toLow;

  // constrain the result to within a valid output range
  if (toHigh < result) result = toHigh;
  else if (result < toLow) result = toLow;

  return result;
}





