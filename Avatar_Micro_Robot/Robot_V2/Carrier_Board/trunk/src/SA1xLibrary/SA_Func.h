
// ----------------------------------------------------------------------------
//         ATMEL Microcontroller Software Support  -  Colorado Springs, CO -
// ----------------------------------------------------------------------------
// DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
// DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------
// File Name           : SA_Func.h
// Object              : declarations, prototypes for CryptoAuth device
//								 Functional API
// Creation            : 02/02/2009
// Updated				  : 09/17/2009
// ----------------------------------------------------------------------------

/** \file
	\brief This file holds the definitions and prototypes for the Functional Layer.

     This layer implements commands from the data sheet. Names for functions that
     implement data sheet commands start with SAF_ for identification. Names for
     helper functions not in the data sheet but convenient start with SAFI_.
*/

#ifndef SA_FUNC_H
#define SA_FUNC_H

#include <stdint.h>	//!< C99 standard typedefs
#include <string.h>

#include "SA_Comm.h"

#define KEYSIZE				32	//!< size of personalization and SA100 key in bytes
#define CHALLENGESIZE		32	//!< size of challenge in bytes
#define DIGESTSIZE			32	//!< size of digest in bytes
#define SEEDSIZE				16	//!< size of seed in bytes
#define SECRETSIZE			8	//!< size of secret in bytes
#define MAPSIZE				11	//!< size of fuse map in bytes (SA102)
#define MAC_OPCODE			8	//!< MAP command code for SA102

#define SECRET_STATUS		(1 << 4) //!< Mode flag to include secret and status fuses in the SHA256 calculation
#define SECRET					(1 << 5) //!< Mode flag to include secret fuses in the SHA256 calculation
#define SERIALNUM				(1 << 6) //!< Mode flag to include serial number in the SHA256 calculation

//! CryptoAuth BurnSecureEnable bit position (at fuse address 87)
#define BURN_SECURE_ENABLE	0x80

//! Param2 high byte in BurnFuse and BurnSecure command when supply voltage is below 4.5 V
#define LOW_VOLTAGE_PARAM	0x80


/*
 * type definitions
 */

//////////////////////////////////////////////////////////////////////
// structures and unions for MAC command

//! MAC command structure
typedef PACKED_ATTRIBUTE1 struct _MACcmd_str
{
	uint8_t		size;								//!< packet size
	uint8_t		ordinal;							//!< command code
	uint8_t		mode;								//!< what to include in the SHA256 calculation
	uint16_t		KeyID;							//!< what key to use in the SHA256 calculation
	uint8_t		challenge[CHALLENGESIZE];	//!< array of 32 challenge bytes
} PACKED_ATTRIBUTE2 MACcmd_str;

//! MAC command union
typedef PACKED_ATTRIBUTE1 union _MACcmd
{
	uint8_t		bytes[sizeof(MACcmd_str)];	//!< command as byte array
	MACcmd_str	MACin;							//!< command as command structure
} PACKED_ATTRIBUTE2 MACcmd;

//! MAC response structure
typedef PACKED_ATTRIBUTE1 struct _MACout
{
	uint8_t		count;							//!< packet size
	uint8_t		digest[DIGESTSIZE];			//!< authentication digest
	uint16_t		CRC;								//!< CRC over entire response
} PACKED_ATTRIBUTE2 MACout;


//////////////////////////////////////////////////////////////////////
// structures and unions for GenPersonalizationKey command

//! GenPersonalizationKey command structure
typedef PACKED_ATTRIBUTE1 struct _GPKcmd_str
{
	uint8_t		size;								//!< packet size
	uint8_t		ordinal;							//!< command code
	uint8_t		zero;								//!< ignored by device
	uint16_t		KeyID;							//!< what key to use in the SHA256 calculation
	uint8_t		seed[SEEDSIZE];				//!< array of 16 seed bytes
} PACKED_ATTRIBUTE2  GPKcmd_str;

//! GenPersonalizationKey command union
typedef PACKED_ATTRIBUTE1 union _GPKcmd
{
	uint8_t		bytes[sizeof(GPKcmd_str)];	//!< command as byte array
	GPKcmd_str	GPKin;							//!< command as command structure

} PACKED_ATTRIBUTE2 GPKcmd;

//! GenPersonalizationKey response structure
typedef PACKED_ATTRIBUTE1 struct _GPKout
{
	uint8_t		count;							//!< packet size
	uint8_t		returnCode;						//!< command execution status
	uint16_t		CRC;								//!< CRC over entire response
} PACKED_ATTRIBUTE2 GPKout;


//////////////////////////////////////////////////////////////////////
// structures and unions for BurnSecure command

//! BurnSecure command structure
typedef PACKED_ATTRIBUTE1 struct _BurnSecure_str
{
	uint8_t		size;								//!< packet size
	uint8_t		ordinal;							//!< command code
	uint8_t		decrypt;							//!< If this flag is set the map is encrypted.
	uint8_t		lowVoltage;						//!< If supply voltage is below 4.5 V, a 0x60 has to be sent, otherwise a 0.
	uint8_t		zero;								//!< always zero
	uint8_t		map[MAPSIZE];					//!< array of eleven map bytes
} PACKED_ATTRIBUTE2 BurnSecure_str;

//! BurnSecure command union
typedef  PACKED_ATTRIBUTE1 union _BurnSecurecmd
{
	uint8_t			bytes[sizeof(BurnSecure_str)];	//!< command as byte array
	BurnSecure_str	BurnSecureIn;							//!< command as command structure
} PACKED_ATTRIBUTE2 BurnSecurecmd;


//////////////////////////////////////////////////////////////////////
// structures and unions for LoadSram command

//! LoadSram command structure
typedef PACKED_ATTRIBUTE1 struct _LoadSram_str
{
	uint8_t		size;								//!< packet size
	uint8_t		ordinal;							//!< command code
	uint8_t		zero;								//!< Device ignores param1.
	uint16_t		zeroes;							//!< Device ignores param2.
	uint8_t		key[KEYSIZE];					//!< array of 32 key bytes
} PACKED_ATTRIBUTE2 LoadSram_str;

//! LoadSram command union
typedef  PACKED_ATTRIBUTE1 union _LoadSramCmd
{
	uint8_t			bytes[sizeof(LoadSram_str)];	//!< command as byte array
	LoadSram_str	LoadSramIn;							//!< command as command structure
} PACKED_ATTRIBUTE2 LoadSramCmd;


//////////////////////////////////////////////////////////////////////
// structures and unions for BurnFuse command

//! BurnFuse command structure
typedef PACKED_ATTRIBUTE1 struct _BurnFuse_str
{
	uint8_t		size;								//!< packet size
	uint8_t		ordinal;							//!< command code
	uint8_t		fuseNumber;						//!< address of fuse (0 to 87 for SA100, 64 to 87 for others)
	uint8_t		lowVoltage;						//!< If supply voltage is below 4.5 V, a 0x60 has to be sent, otherwise a 0.
	uint8_t		zero;								//!< always zero
} PACKED_ATTRIBUTE2 BurnFuse_str;

//! BurnFuse command union
typedef PACKED_ATTRIBUTE1 union _BurnFusecmd
{
	uint8_t			bytes[sizeof(BurnFuse_str)];	//!< command as byte array
	BurnFuse_str	BurnFuseIn;							//!< command as command structure
} PACKED_ATTRIBUTE2 BurnFusecmd;

//! BurnFuse response structure
typedef PACKED_ATTRIBUTE1 struct _SA_BurnFuse_out
{
	uint8_t		count;							//!< packet size
	uint8_t		returnCode;						//!< command execution status
	uint16_t		CRC;								//!< CRC over entire response
} PACKED_ATTRIBUTE2  SA_BurnFuse_out;

//! BurnSecure response structure
typedef PACKED_ATTRIBUTE1 struct _SA_BurnSecure_out
{
	uint8_t		count;							//!< packet size
	uint8_t		returnCode;						//!< command execution status
	uint16_t		CRC;								//!< CRC over entire response
} PACKED_ATTRIBUTE2 SA_BurnSecure_out;


//////////////////////////////////////////////////////////////////////
// structures and unions for PauseLong command

//! PauseLong command structure
typedef PACKED_ATTRIBUTE1 struct _pauseLong_str
{
	uint8_t		size;								//!< packet size
	uint8_t		ordinal;							//!< command code
	uint8_t		selector;						//!< fuses 84 to 87 matching this value
	uint16_t		zero;								//!< Device ignores param2.
} PACKED_ATTRIBUTE2  pauseLong_str;

//! PauseLong command union
typedef PACKED_ATTRIBUTE1 union _pauseLongcmd
{
	uint8_t			bytes[sizeof(pauseLong_str)];	//!< command as byte array
	pauseLong_str	pauseLongIn;						//!< command as command structure
} PACKED_ATTRIBUTE2 pauseLongcmd;

//! PauseLong response structure
typedef PACKED_ATTRIBUTE1 struct _SA_pauseLong_out
{
	uint8_t		count;							//!< packet size
	uint8_t		returnCode;						//!< command execution status
	uint16_t		CRC;								//!< CRC over entire response
} PACKED_ATTRIBUTE2 SA_pauseLong_out;


//////////////////////////////////////////////////////////////////////
// structures and unions for SA_Read command

//! Read command structure
typedef PACKED_ATTRIBUTE1 struct _readCmd_str
{
	uint8_t		size;								//!< packet size
	uint8_t		ordinal;							//!< command code
	uint8_t		mode;								//!< what section to read, fuse, ROM, or MemValid (SA100)
	uint16_t		address;							//!< what byte address to read from (every address increment is four bytes)
} PACKED_ATTRIBUTE2 readCmd_str;

//! Read command union
typedef PACKED_ATTRIBUTE1 union _readCmd
{
	uint8_t		bytes[sizeof(readCmd_str)];	//!< command as byte array
	readCmd_str	readCmdin;							//!< command as command structure
} PACKED_ATTRIBUTE2 readCmd;

//! Read response structure
typedef PACKED_ATTRIBUTE1 struct _SA_Read_out
{
	uint8_t		count;								//!< packet size
	uint8_t		SAdata[4];							//!< four bytes read from the device
	uint16_t		CRC;									//!< CRC over entire response
} PACKED_ATTRIBUTE2 SA_Read_out;

//! Read response data
typedef PACKED_ATTRIBUTE1 struct _SA_readBuf
{
	uint8_t		bytes[sizeof(((SA_Read_out*)0)->SAdata)];	//!< data field of response
} PACKED_ATTRIBUTE2 SA_readBuf;

//! response structure for reading serial number
typedef PACKED_ATTRIBUTE1 struct _snBuf_str
{
	uint8_t		ROMdata[2];							//!< ROM part of serial number
	uint8_t		fusedata[4];						//!< fuse part of serial number
} PACKED_ATTRIBUTE2 snBuf_str;

//! response union for reading serial number
typedef  PACKED_ATTRIBUTE1 union _snBuf
{
	uint8_t		bytes[sizeof(snBuf_str)];		//!< response as byte array
	snBuf_str	sn_data;								//!< response as structure
} PACKED_ATTRIBUTE2 SA_snBuf;

//! response structure for reading manufacturer id
typedef PACKED_ATTRIBUTE1 struct _mfrID_str
{
	uint8_t		ROMdata[2];							//!< ROM part of manufacturing id
	uint8_t		fusedata[1];						//!< fuse part of manufacturing id
} PACKED_ATTRIBUTE2 mfrID_str;

//! response union for reading manufacturer id
typedef  PACKED_ATTRIBUTE1 union _SA_mfrID_buf
{
	uint8_t		bytes[sizeof(mfrID_str)];		//!< response as byte array
	mfrID_str	mfrID_data;							//!< response as structure
} PACKED_ATTRIBUTE2 SA_mfrID_buf;

//! response structure for reading status fuses
typedef PACKED_ATTRIBUTE1 struct _SA_statusBuf
{
	uint8_t		bytes[3];							//! fuse bit values lumped into bytes
} PACKED_ATTRIBUTE2 SA_statusBuf;

//! response structure for reading revision
typedef PACKED_ATTRIBUTE1 struct _SA_revNumBuf
{
	uint8_t		bytes[4];							//!< ROM revision of device.
} PACKED_ATTRIBUTE2 SA_revNumBuf;

//! response structure for reading MemValid flag (SA100)
typedef PACKED_ATTRIBUTE1 struct _SA_memValidBuf
{
	uint8_t		sramValid;							//! MemValid flag
} PACKED_ATTRIBUTE2 SA_memValidBuf;

//////////////////////////////////////////////////////////////////////
// structures and unions for Host0 command

//! Host0 command structure
typedef PACKED_ATTRIBUTE1 struct _Host0Cmd_str
{
	uint8_t		size;									//!< packet size
	uint8_t		ordinal;								//!< command code
	uint8_t		overwrite;							//!< If non-zero, overwrite part of internally generated key with secret fuses.
	uint16_t		keyID;								//!< what key to use in the SHA256 calculation
	uint8_t		challenge[CHALLENGESIZE];		//!< array of 32 challenge bytes that had been sent to the client
} PACKED_ATTRIBUTE2 Host0Cmd_str;

//! Host0 command union
typedef  PACKED_ATTRIBUTE1 union _Host0_cmd
{
	uint8_t			bytes[sizeof(Host0Cmd_str)];	//!< command as byte array
	Host0Cmd_str	Host0CmdIn;							//!< command as command structure
} PACKED_ATTRIBUTE2 Host0_cmd;

//! Host0 response structure
typedef PACKED_ATTRIBUTE1 struct _Host0Out
{
	uint8_t		count;								//!< packet size
	uint8_t		H0data[1];							//!< command execution status
	uint16_t		CRC;									//!< CRC over entire response
} PACKED_ATTRIBUTE2 Host0Out;

//! structure for Host0 response data (status byte)
typedef PACKED_ATTRIBUTE1 struct _Host0Buf
{
	uint8_t		bytes[sizeof(((Host0Out *)0)->H0data)]; //!< status byte
} PACKED_ATTRIBUTE2 Host0Buf;


//////////////////////////////////////////////////////////////////////
// structures and unions for Host1 command

//! structure for Host1 command parameter "OtherInfo"
typedef PACKED_ATTRIBUTE1 struct _Host1OtherInfo_str
{
	uint8_t		opCode;								//!< command code for MAC command
	uint8_t		clientMode;							//!< mode used in MAC command
	uint16_t		keyId;								//!< what key the client used in the SHA256 calculation
	uint8_t		statusFuses[3];					//!< status fuses of client
	uint8_t		serialNumberFuse[4];				//!< fuse part of client's serial number
	uint8_t		serialNumberRom[2];				//!< ROM part of client's serial number
} PACKED_ATTRIBUTE2 Host1OtherInfo_str;

//! structure for Host1 command
typedef PACKED_ATTRIBUTE1 struct _Host1Cmd_str
{
	uint8_t		size;												//!< packet size
	uint8_t		ordinal;											//!< command code
	uint8_t		mode;												//!< what mode to use for the SHA256 calculation
	uint16_t		zero;												//!< Device ignores param2.
	uint8_t		otherInfo[sizeof(Host1OtherInfo_str)];	//!< certain data used in the MAC command
} PACKED_ATTRIBUTE2 Host1Cmd_str;

//! Host1 command union
typedef  PACKED_ATTRIBUTE1 union _Host1_cmd
{
	uint8_t			bytes[sizeof(Host1Cmd_str)];	//!< command as byte array
	Host1Cmd_str	Host1CmdIn;							//!< command as command structure
} PACKED_ATTRIBUTE2 Host1_cmd;

//! Host1 response structure
typedef PACKED_ATTRIBUTE1 struct _Host1Out
{
	uint8_t		count;								//!< packet size
	uint8_t		H1data[1];							//!< command execution status
	uint16_t		CRC;									//!< CRC over entire response
}  PACKED_ATTRIBUTE2 Host1Out;

//! Host1 response data (status byte)
typedef PACKED_ATTRIBUTE1 struct _Host1Buf
{
	uint8_t		bytes[sizeof(((Host1Out *) 0)->H1data)];	//!< status byte
} PACKED_ATTRIBUTE2 Host1Buf;


//////////////////////////////////////////////////////////////////////
// structures and unions for Host2 command

//! Host2 command structure
typedef PACKED_ATTRIBUTE1 struct _Host2Cmd_str
{
	uint8_t		size;									//!< packet size
	uint8_t		ordinal;								//!< command code
	uint8_t		zero1;								//!< Device ignores param1.
	uint16_t		zero2;								//!< Device ignores param2.
	uint8_t		clientResponse[DIGESTSIZE];	//!< MAC command response
}  PACKED_ATTRIBUTE2 Host2Cmd_str;

//! Host2 command union
typedef  PACKED_ATTRIBUTE1 union _Host2_cmd
{
	uint8_t			bytes[sizeof(Host2Cmd_str)];	//!< command as byte array
	Host2Cmd_str	Host2CmdIn;							//!< command as command structure
} PACKED_ATTRIBUTE2 Host2_cmd;

//! Host2 response structure
typedef PACKED_ATTRIBUTE1 struct _Host2Out
{
	uint8_t		count;								//!< packet size
	uint8_t		H2data[1];							//!< command execution status
	uint16_t		CRC;									//!< CRC over entire response
} PACKED_ATTRIBUTE2 Host2Out;

//! HA_Host2 response data (status byte)
typedef PACKED_ATTRIBUTE1 struct _Host2Buf
{
	uint8_t		bytes[sizeof(((Host2Out*)0)->H2data)];	//!< status byte
} PACKED_ATTRIBUTE2 Host2Buf;


//////////////////////////////////////////////////////////////////////
// structures and unions for PauseShort command

//! PauseShort command structure
typedef PACKED_ATTRIBUTE1 struct _PauseShort_str
{
	uint8_t		size;									//!< packet size
	uint8_t		ordinal;								//!< command code
	uint8_t		ignored[3];							//!< Device ignores param1 and param2.
} PACKED_ATTRIBUTE2 PauseShort_str;

//! PauseShort command union
typedef  PACKED_ATTRIBUTE1 union _PauseShort_cmd
{
	uint8_t			bytes[sizeof(PauseShort_str)];	//!< command as byte array
	PauseShort_str	PauseShortIn;							//!< command as command structure
} PACKED_ATTRIBUTE2 PauseShort_cmd;


//////////////////////////////////////////////////////////////////////

int8_t SAF_MAC(uint8_t Mode, uint16_t KeyID, uint8_t *Challenge, uint8_t *DigestOut);
int8_t SAF_Read(uint8_t Mode, uint16_t Address, SA_readBuf *OutBuf);
int8_t SAF_BurnFuse(uint8_t FuseNumber);
int8_t SAF_BurnSecure(uint8_t decryptOpt, uint8_t *mapData);
int8_t SAF_LoadSram(uint8_t *keyData);
int8_t SAF_GenPersonalizationKey(uint16_t KeyID, uint8_t *Seed);
int8_t SAF_PauseLong(uint8_t Selector);
int8_t SAF_Host0(uint8_t Overwrite, uint16_t KeyID, uint8_t *Challenge);
int8_t SAF_Host1(Host1OtherInfo_str *OtherInfo);
int8_t SAF_Host2(uint8_t *ClientResponse);
int8_t SAF_PauseShort(void);
int8_t SAFI_MAC(uint8_t *Key, uint8_t *Challenge, uint8_t *CustID, uint8_t LowFuses, uint8_t *SAFI_MaskSN, uint8_t *HighFuses);
int8_t SAFI_ReadStatus(SA_statusBuf *outBuf);
int8_t SAFI_ReadRevision(SA_revNumBuf *outBuf);
int8_t SAFI_ReadManufacturerId(SA_mfrID_buf *outBuf);
int8_t SAFI_ReadSerialNumber(SA_snBuf *outBuf);
int8_t SAFI_ReadMemValid(SA_memValidBuf* outBuf);
int8_t SAF_Init(uint8_t initOptions);

uint8_t SAFI_DiscoverDevices(void);
uint8_t SAFI_GetDeviceCount(uint8_t clientsOnly);


#endif
