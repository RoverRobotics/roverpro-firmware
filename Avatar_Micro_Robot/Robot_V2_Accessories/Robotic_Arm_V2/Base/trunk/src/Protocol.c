/*=============================================================================
File: Protocol.c

Notes:
  - could speed up with more deep copies, but with more levels of indirection
    the code becomes less clean to read and more prone to bugs
=============================================================================*/
/*---------------------------Dependencies------------------------------------*/
#include "./Protocol.h"
//#include <string.h>   // for memset()

/*---------------------------Helper Function Prototypes----------------------*/
static unsigned int ComputeCRC(unsigned char data[], 
                               unsigned char logical_length);
static char IsCRCValid(unsigned char data[], unsigned char logical_length, 
                       unsigned int CRC);
static void ParsePacket(unsigned char packet[], unsigned char* data_ptr[], 
                        unsigned char* data_length_ptr, unsigned int* CRC_ptr);

/*---------------------------Public Function Definitions---------------------*/
void BuildPacket(unsigned char data[], unsigned char data_length,
                 unsigned char* packet_ptr[], unsigned char* packet_length_ptr) {
  *packet_length_ptr = data_length + NUM_PREFIX_BYTES + NUM_SUFFIX_BYTES;
  // ensure the output array begins cleared
  //memset(*(void**)packet_ptr, 0, packet_length);

  
}


void GetData(unsigned char packet[], unsigned char* data_ptr[], 
             unsigned char* data_length_ptr) {
  unsigned int CRC = 0;
  ParsePacket(packet, data_ptr, data_length_ptr, &CRC);
  if (!IsCRCValid(*data_ptr, *data_length_ptr, CRC)) *data_ptr = 0;
}

/*---------------------------Private Function Definitions--------------------*/
/*
Function: ComputeCRC()
Parameters:
  unsigned char data[], the data on which to compute the CRC
  unsigned char logical_length, the meaningful number of bytes in the array
Description: Computes the cyclic redundancy check value (CRC) as part of a 
  sequence to detect accidental changes to raw data.
Notes:
  - employing CRC-8??
  - see also http://en.wikipedia.org/wiki/Cyclic_redundancy_check
TODO: make sure this makes sense to me
*/
static unsigned int ComputeCRC(unsigned char data[], 
                               unsigned char logical_length) {
	static unsigned int CRC;
	unsigned char i;
	CRC = 17;  // AS LONG AS CRC is NOT initialized to 0, we can check for 0 later on?

	for (i = 0; i < logical_length; i++) {
		CRC = (unsigned char)(CRC >> 8) | (CRC << 8);
		CRC ^= data[i];
		CRC ^= (unsigned char)(CRC & 0xFF) >> 4;
		CRC ^= (CRC << 8) << 4;
		CRC ^= ((CRC & 0xff) << 4) << 1;
		CRC &= 0xFFFF;
	}
	
	return CRC;
}

/*
Function: IsCRCValid()
Parameters:
  unsigned char data[], the data to validate
  unsigned char logical_length, the meaningful number of bytes in the array
Description: Checks whether the CRC within the given packet matches the 
  CRC produced by the data.
*/
static char IsCRCValid(unsigned char data[], unsigned char logical_length, 
                       unsigned int CRC) {
	return (CRC == ComputeCRC(data, logical_length));
}

/*
Function: ParsePacket()
Description: Parses out the data and the checksum from the packet and places
  the result in the contents of data_ptr and CRC_ptr respectively.
Notes:
  - assumes the suffix is ENTIRELY COMPRISED OF 2 CRC BYTES
*/
static void ParsePacket(unsigned char packet[], unsigned char* data_ptr[], 
                        unsigned char* data_length_ptr, unsigned int* CRC_ptr) {
  // parse the data
  unsigned char i = 0;
  
  unsigned char packet_length = 0;
  // count from the end of the array until we see a non-zero value to determine its logical length
  // this assumes that the arrays are initialized to 0 and 
  while (packet[MAX_PACKET_SIZE - 1 - i] == 0) i++; 
  packet_length = i;
  *data_length_ptr = packet_length - NUM_PREFIX_BYTES - NUM_SUFFIX_BYTES;
  
  for (i = 0; i < (packet_length - NUM_SUFFIX_BYTES); i++)
		(*data_ptr)[i + NUM_PREFIX_BYTES] = packet[i];
  
  // parse the checksum
  *CRC_ptr = (packet[packet_length - NUM_SUFFIX_BYTES] << sizeof(char)) + 
             packet[packet_length - 1];
}
