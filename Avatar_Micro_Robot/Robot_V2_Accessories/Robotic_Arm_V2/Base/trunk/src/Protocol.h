/*=============================================================================
File: Protocol.h 

Description: This file encapsulates the communication protocol.  This protocol
  layer is completely hardware-agnostic and employs a hash function to detect
  accidental changes to the raw data.
  
Notes:
  - Packet Structure: HEADER_H HEADER_L DEVICE DATA ... DATA CRC_H CRC_L
    where suffix _H indicates a high byte and _L indicates a low byte
  - the CRC is computed on DEVICE and all DATA bytes 
=============================================================================*/
#ifndef PROTOCOL_H
#define PROTOCOL_H

/*---------------------------Macros------------------------------------------*/
// packet framing constants
#define HEADER_H            0xFF
#define HEADER_L            0xCC

// devices
#define UNKNOWN_DEVICE      0x00
#define BASE                0x0B
#define LINK_1              0x0C
#define LINK_2              0x0D

// number of elements (bytes) of the data sent by each of the devices (1-based counting)
#define BASE_DATA_LENGTH    4
#define LINK_1_DATA_LENGTH  0
#define LINK_2_DATA_LENGTH  5
#define INVALID_LENGTH      0xFF
#define MAX_DATA_LENGTH     5

#define MAX_PACKET_LENGTH   15  // the maximum number of anticipated bytes our
                                // packet will ever hold (to avoid dynamic 
                                // memory management)
#define NUM_PREFIX_BYTES    2
#define NUM_SUFFIX_BYTES    2
#define NUM_DEVICE_BYTES    1


#define DEVICE_INDEX        2   // device index in the protocol

#define TURRET_SPEED_INDEX  3
/*---------------------------Public Function Prototypes----------------------*/
/*
Function: BuildPacket()
Parameters:
  unsigned char device, the device for which to build the packet (see macros)
  unsigned char data[], the data array
  unsigned char packet[], array into which the resulting packet is placed
  unsigned char *packet_length_ptr, packet length to update
Description: Builds a packet from the given data as specified by the protocol 
Notes:
  - WARNING: data MUST be the appropriate length for the device
*/
void BuildPacket(unsigned char device, unsigned char data[],
                 unsigned char packet[], unsigned char *packet_length_ptr);

/*
Function: GetData()
Parameters:
  unsigned char packet[], a received packet
  unsigned char data[], the data array to be updated
  unsigned char *data_length_ptr, pointer to the data length to be updated
Description: Parses the packet for the data and validates the result.  The 
  resulting data length is updated to INVALID_LENGTH as a sentinel for 
  failing the CRC.
*/
void GetData(unsigned char packet[], unsigned char data[],
             unsigned char *data_length_ptr);

/*
Function: GetDataLength()
Description: Determines the data length for the given device. Returns
  a sentinel for an unrecognized device, INVALID_LENGTH.
*/
unsigned char GetDataLength(unsigned char device);


#endif
