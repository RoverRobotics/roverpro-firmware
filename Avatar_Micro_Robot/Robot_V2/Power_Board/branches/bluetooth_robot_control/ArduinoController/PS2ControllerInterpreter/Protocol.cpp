/*=============================================================================
File: Protocol.c
=============================================================================*/
/*---------------------------Dependencies------------------------------------*/
#include "./Protocol.h"

/*---------------------------Helper Function Prototypes----------------------*/
static uint16_t ComputeCRC(uint8_t data[], uint8_t logical_length);
static int8_t IsCRCValid(uint8_t data[], uint8_t logical_length, uint16_t CRC);
static void ParsePacket(uint8_t packet[], uint8_t data[], 
                        uint8_t* data_length, uint16_t* CRC_ptr);

/*---------------------------Public Function Definitions---------------------*/
void PROT_BuildPacket(uint8_t data[], uint8_t data_length,
                      uint8_t packet[], uint8_t* packet_length_ptr) {
  *packet_length_ptr = data_length + PROT_N_PREFIX_BYTES + PROT_N_SUFFIX_BYTES;
 
  // add the header
  packet[0] = PROT_HEADER_H;
  packet[1] = PROT_HEADER_L;
  
  // add the data (deep copy each element over)
  uint8_t i;
  for (i = 0; i < data_length; i++) packet[PROT_N_PREFIX_BYTES + i] = data[i];
  
  // add the CRC to the end
  uint16_t CRC = ComputeCRC(&packet[PROT_N_PREFIX_BYTES], data_length);
  packet[*packet_length_ptr - 1] = (CRC & 0xFF); // low byte
  packet[*packet_length_ptr - 2] = (CRC >> 8);   // high byte
}

  
void PROT_GetData(uint8_t packet[], uint8_t data[], uint8_t* data_length_ptr) {
  uint16_t CRC = 0;
  ParsePacket(packet, data, data_length_ptr, &CRC);
  
  if (*data_length_ptr == PROT_INVALID_LENGTH) return;
  
  if (!IsCRCValid(&packet[PROT_N_PREFIX_BYTES], (*data_length_ptr), CRC)) {
    *data_length_ptr = PROT_INVALID_LENGTH;
  }
}

/*---------------------------Private Function Definitions--------------------*/
// Function: ComputeCRC()
// Parameters:
//   uint8_t data[], the data on which to compute the CRC
//   uint8_t data_length, the length of the data
// Description: Computes the cyclic redundancy check value (CRC) as part of a 
//   sequence to detect accidental changes to raw data.
// Notes:
//   - employing CRC-CCITT (XModem)
//   - http://www.lammertbies.nl/comm/info/crc-calculation.html
//   - http://en.wikipedia.org/wiki/Cyclic_redundancy_check
static uint16_t ComputeCRC(uint8_t data[], uint8_t data_length) {
  uint16_t CRC = 0;
  uint8_t i;
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

// Function: IsCRCValid
// Parameters:
//   uint8_t data[], the data to validate
//   uint8_t data_length,
// Description: Checks whether the CRC within the given packet matches the 
//   CRC produced by the data.
static int8_t IsCRCValid(uint8_t data[], uint8_t data_length, uint16_t CRC) {
  return (CRC == ComputeCRC(data, data_length));
}


// Function: ParsePacket
// Description: Parses out the data and the checksum from the packet and places
//   the result in the contents of the given data array and CRC respectively.  
//   Prematurely returns if invalid data is found at any step of the way, marking 
//   the data length as INVALID_LENGTH as a sentinel for failure.
// Notes:
//   - WARNING: assumes the suffix is entirely comprised of two CRC bytes
static void ParsePacket(uint8_t packet[], uint8_t data[], 
                        uint8_t* data_length_ptr, uint16_t* CRC_ptr) {
  // bail if we have an invalid header
  if ((packet[0] != PROT_HEADER_H) && (packet[1] != PROT_HEADER_L)) {
    *data_length_ptr = PROT_INVALID_LENGTH;
    return;
  }

  // parse the data
  uint8_t i = 0;
  for (i = 0; i < (*data_length_ptr); i++) {
    data[i] = packet[i + PROT_N_PREFIX_BYTES];
  }
  
  // parse the checksum (high byte + low byte)
  unsigned char packet_length = PROT_N_PREFIX_BYTES + 
                                (*data_length_ptr) + PROT_N_SUFFIX_BYTES;
  *CRC_ptr = (packet[packet_length - 2] << 8)
             + packet[packet_length - 1];
}
