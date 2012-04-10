/*=============================================================================
File: Protocol.c

Notes:
    - employing CRC-CCITT (XModem)
    - http://www.lammertbies.nl/comm/info/crc-calculation.html
=============================================================================*/
/*---------------------------Dependencies------------------------------------*/
#include "./Protocol.h"

/*---------------------------Helper Function Prototypes----------------------*/
static unsigned int ComputeCRC(unsigned char data[], 
                               unsigned char logical_length);
static char IsCRCValid(unsigned char data[], unsigned char logical_length, 
                       unsigned int CRC);
static void ParsePacket(unsigned char packet[], unsigned char data[], 
                        unsigned char *data_length, unsigned int *CRC_ptr);

/*---------------------------Public Function Definitions---------------------*/
void BuildPacket(unsigned char device, unsigned char data[],
                 unsigned char packet[], unsigned char *packet_length_ptr) {
  unsigned char data_length = GetDataLength(device);
  *packet_length_ptr = data_length + NUM_PREFIX_BYTES + NUM_DEVICE_BYTES
                       + NUM_SUFFIX_BYTES;
 
  // add the header and device
  packet[0] = HEADER_H; packet[1] = HEADER_L; packet[2] = device;
  
  // add the data
  unsigned char i; 
  for (i = 0; i < data_length; i++)
    packet[3 + i] = data[i];
  
  // add the CRC
  unsigned int CRC = ComputeCRC(&packet[NUM_PREFIX_BYTES], 
                               (data_length + NUM_DEVICE_BYTES));
  packet[*packet_length_ptr - 1] = (CRC & 0xFF); // low byte
  packet[*packet_length_ptr - 2] = (CRC >> 8);   // high byte
}


void GetData(unsigned char packet[], unsigned char data[], 
             unsigned char *data_length_ptr) {
  unsigned int CRC = 0;
  ParsePacket(packet, data, data_length_ptr, &CRC);

  if (!IsCRCValid(data, *data_length_ptr, CRC)) data = 0;
}


unsigned char GetDataLength(unsigned char device) {
  switch (device) {
    case BASE: return BASE_DATA_LENGTH;
    case LINK_1: return LINK_1_DATA_LENGTH;
    case LINK_2: return LINK_2_DATA_LENGTH;
    default: return INVALID_LENGTH;
  }
}

/*---------------------------Private Function Definitions--------------------*/
/*
Function: ComputeCRC()
Parameters:
  unsigned char data[], the data on which to compute the CRC
  unsigned char data_length, the meaningful number of bytes in the array
Description: Computes the cyclic redundancy check value (CRC) as part of a 
  sequence to detect accidental changes to raw data.
Notes:
  - employing CRC-CCITT (XModem)
  - http://www.lammertbies.nl/comm/info/crc-calculation.html
  - see also http://en.wikipedia.org/wiki/Cyclic_redundancy_check
*/
static unsigned int ComputeCRC(unsigned char data[], 
                               unsigned char data_length) {
	unsigned char i;
	unsigned int CRC = 0;   // AS LONG AS CRC is NOT initialized to 0, 
	                        // it can never be 0?
  
  for (i = 0; i < data_length; i++) {
		CRC = (unsigned char)(CRC >> 8) | (CRC << 8);
		CRC ^= data[i];
		CRC ^= (unsigned char)(CRC & 0xFF) >> 4;
		CRC ^= (CRC << 8) << 4;
		CRC ^= ((CRC & 0xFF) << 4) << 1;
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
  the result in the contents of the given data array and CRC respectively.  
  Prematurely returns if invalid data is found at any step of the way, marking 
  the CRC as 0 as a sentinel for invalid data.
Notes:
  - WARNING: assumes the suffix is entirely comprised of two CRC bytes
*/
static void ParsePacket(unsigned char packet[], unsigned char data[], 
                        unsigned char *data_length_ptr, unsigned int *CRC_ptr) {
  // bail if we have an invalid header
  if ((packet[0] != HEADER_H) && (packet[1] != HEADER_L)) {
    *CRC_ptr = 0;  // sentinel for invalid?
    return;
  }

  // bail if we have an invalid device
  *data_length_ptr = GetDataLength(packet[2]);
  if (*data_length_ptr == INVALID_LENGTH) {
    *CRC_ptr = 0;
    return;
  }

  // parse the data (deep copy the contents over)
  // note: this is slower than assigning to index in array, but level-of-indirection error
  unsigned char i;
  for (i = 0; i < (*data_length_ptr); i++)
    data[i] = packet[i + NUM_PREFIX_BYTES + NUM_DEVICE_BYTES];
  
  // parse the checksum (high byte + low byte)
  unsigned char packet_length = NUM_PREFIX_BYTES + NUM_DEVICE_BYTES 
                                + (*data_length_ptr) + NUM_SUFFIX_BYTES;
  *CRC_ptr = (packet[packet_length - 2] << 8)
             + packet[packet_length - 1];
}
