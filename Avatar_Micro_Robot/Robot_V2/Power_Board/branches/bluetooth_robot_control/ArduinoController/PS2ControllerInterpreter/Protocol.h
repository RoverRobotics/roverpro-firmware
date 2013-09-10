/*=============================================================================
File: Protocol.h 

Description: This file encapsulates the communication protocol.  This protocol
  layer is completely hardware-agnostic and employs a hash function to detect
  accidental changes to the raw data.
  
Notes:
  - Packet Structure: 
      HEADER_H HEADER_L DATA ... DATA CRC_H CRC_L
      where suffix _H indicates a high byte and _L indicates a low byte
  - the CRC is computed only on DATA bytes 
=============================================================================*/
#ifndef PROTOCOL_H
#define PROTOCOL_H

/*---------------------------Dependencies-------------------------------------*/
#include <stdint.h>  // for uintN_t data types

/*---------------------------Macros------------------------------------------*/
// packet framing constants
#define PROT_HEADER_H            0xFF
#define PROT_HEADER_L            0xCC

// number of elements (bytes) of the data sent by each of the devices (1-based counting)
#define PROT_INVALID_LENGTH      0xFF
#define PROT_MAX_DATA_LENGTH     16

#define PROT_MAX_PACKET_LENGTH   20  // the maximum number of anticipated bytes our
                                     // packet will ever hold (to avoid dynamic 
                                     // memory management)
#define PROT_N_PREFIX_BYTES      2
#define PROT_N_SUFFIX_BYTES      2

/*---------------------------Public Function Prototypes----------------------*/
// Function: PROT_BuildPacket
// Parameters:
//   uint8_t data[],             the data array
//   uint8_t data_length,        the length of the data array
//   uint8_t packet[],           array into which the resulting packet is placed
//   uint8_t* packet_length_ptr, packet length to update
// Description: Builds a packet from the given data as specified by the protocol 
void PROT_BuildPacket(uint8_t data[], uint8_t data_length,
                      uint8_t packet[], uint8_t* packet_length_ptr);


// Function: PROT_GetData
// Parameters:
//   uint8_t packet[],           a received packet
//   uint8_t data[],             the data array to be updated
//   uint8_t* data_length_ptr,   pointer to the data length to be updated
// Description: Parses the packet for the data and validates the result.  The 
//   resulting data length is updated to INVALID_LENGTH as a sentinel for 
//   failing the CRC.
void PROT_GetData(uint8_t packet[], uint8_t data[], uint8_t* data_length_ptr);
#endif
