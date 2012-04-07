/*=============================================================================
File: Protocol.h 

Description: This file encapsulates the communication protocol.

Notes:
  - Packet Structure: HEADER_1 HEADER_2 DATA ... DATA CRC_1 CRC_2
    where CRC_1 is the first byte of the Cyclic Redundancy Check (CRC)
=============================================================================*/
#ifndef PROTOCOL_H
#define PROTOCOL_H

/*---------------------------Macros------------------------------------------*/
// packet framing constants
#define HEADER_1          0xFF
#define HEADER_2          0xCC

// devices
#define BASE              0x0B
#define LINK_1            0x0C
#define LINK_2            0x0D

#define MAX_PACKET_SIZE  25  // the maximum number of anticipated bytes our
                              // packet will ever hold (to avoid dynamic 
                              // memory management)
#define NUM_PREFIX_BYTES  2   // where prefix/suffix mean anything that is NOT data
#define NUM_SUFFIX_BYTES  2

/*---------------------------Public Function Prototypes----------------------*/
/*
Function: BuildPacket()
Parameters:
  unsigned char data[], the data array
  unsigned char* packet_ptr[], pointer to the array into which the resulting
                               packet should be placed
Description: Builds a packet from the given data as specified by the protocol 
*/
void BuildPacket(unsigned char data[], unsigned char data_length,
                 unsigned char* packet_ptr[], unsigned char* packet_length_ptr);

/*
Function: GetData()
Parameters:
  unsigned char packet[], the full packet
  unsigned char* data_ptr[], the output data array passed by reference
  unsigned char* data_length_ptr,
Description: Parses the packet for the data and validates the result.  The 
  result is updated to 0 as a sentinel for invalid data.
*/
void GetData(unsigned char packet[], unsigned char* data_ptr[], 
             unsigned char* data_length_ptr);

#endif
