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

/** \file
 *	\brief	This file contains functions that implement CryptoAuth commands.
 *	\date		Nov 12, 2009
 */
#include "SA_Func.h"
#include "SA_Status.h"

// CryptoAuth command templates - smaller than building up in code
// GF: This way they get copied to RAM. If we want to save RAM we have to
// insert PROGMEM before the equal sign.
// byte count, op-code, mode, address low, address high
/** \brief command buffer to read ROM area 0 */
static uint8_t cmd_readROM_00[]		= {0x07, 0x02, 0x00, 0x00, 0x00};

/** \brief command buffer to read ROM area 1 */
static uint8_t cmd_readROM_01[]		= {0x07, 0x02, 0x00, 0x01, 0x00};

/** \brief command buffer to read fuse area 2 */
static uint8_t cmd_readFuse_02[]		= {0x07, 0x02, 0x01, 0x02, 0x00};

/** \brief command buffer to read fuse area 3 */
static uint8_t cmd_readFuse_03[]		= {0x07, 0x02, 0x01, 0x03, 0x00};

/** \brief command buffer to read MemValid flag (SA100 only) */
static uint8_t cmd_readMemValid[]	= {0x07, 0x02, 0x03, 0x00, 0x00};

/** \brief command buffer for the MAC command */
static uint8_t cmd_MAC[]				= {0x27, 0x08, 0x99, 0xAA, 0xBB,
													0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
													0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
													0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC,
													0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

/** \brief command buffer to generate personalization key */
static uint8_t cmd_GPK[]				= {0x17, 0x20, 0x00, 0xAA, 0xAA,
													0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB,
													0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB, 0xBB};

/** \brief command buffer to burn fuse */
static uint8_t cmd_burnFuse[]			= {0x07, 0x04, 0x40, 0x00, 0x00};

/** \brief command buffer to burn fuse map */
static uint8_t cmd_burnSecure[]		= {0x12, 0x10, 0xFE, 0x00, 0x00,
													0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
													0x00, 0x00, 0x00};

/** \brief command buffer to load SRAM (SA100 only) */
static uint8_t cmd_loadSram[]			= {0x27, 0x10, 0x00, 0x00, 0x00,
													0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
													0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
													0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
													0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/** \brief command buffer for a PauseLong command */
static uint8_t cmd_pauseLong[]		= {0x07, 0x01, 0x00, 0x00, 0x00};

/** \brief Host 0 command */
static uint8_t cmd_host0[]				= {0x27, 0x08, 0x00, 0x00, 0x00, 0x00};

/** \brief Host 1 command */
static uint8_t cmd_host1[]				= {0x14, 0x40, 0x00, 0x00, 0x00, 0x00};

/** \brief Host 2 command */
static uint8_t cmd_host2[]				= {0x27, 0x80, 0x00, 0x00, 0x00, 0x00};

/** \brief Pause Short command */
static uint8_t cmd_pauseShort[]		= {0x00, 0x01, 0x00, 0x00, 0x00};

//! number of devices in device array
static uint8_t deviceCount				= 0;


/** \brief This function sends a MAC command to the device.
 *
 * \param[in]	Mode what to include in the MAC calculation
 * \param[in]	KeyID key id (0 for SA100)
 * \param[in]	Challenge pointer to challenge data
 * \param[out]	DigestOut pointer to response data (digest)
 * \return		status of the operation
 */
int8_t SAF_MAC(uint8_t Mode, uint16_t KeyID, uint8_t *Challenge, uint8_t *DigestOut)
{
	MACcmd cmdData;
	MACout retData;
	uint8_t size = sizeof(retData); // In: size of output buffer / Out: size of returned data.
	int8_t status;

	if (!Challenge || !DigestOut)
		return SA_BADPARM;

	// Copy command data.
	memmove(cmdData.bytes, cmd_MAC, sizeof(cmdData));

	// Copy the input parameters.
	cmdData.MACin.mode = Mode;

#ifndef BIGENDIAN
	cmdData.MACin.KeyID = (KeyID << 8) | (KeyID >> 8);
#else
	cmdData.MACin.KeyID = KeyID;
#endif

	memmove(cmdData.MACin.challenge, Challenge, sizeof(cmdData.MACin.challenge));

	// Transfer the command to the chip.
	status = SAC_SendAndReceive(cmdData.bytes,			// command data
										(uint8_t *) &retData,	// receive buffer
										SA_DELAY_EXEC_MAC,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	// Check returned parameters.
	if (size != sizeof(retData))
		return SA_BADSIZE;

	// Copy to output.
	memmove(DigestOut, retData.digest, sizeof(retData.digest));

	return status;
}


/**\brief		This function reads from Fuse, ROM, or MemValid area.
 *					No parameter except valid pointer checking is performed.
 * \param[in]	Mode fuse (0), ROM (1), or MemValid (3)
 * \param[in]	Address address
 * \param[out]	OutBuf pointer to response buffer
 * \return		status of the operation
 */
int8_t SAF_Read(uint8_t Mode, uint16_t Address, SA_readBuf *OutBuf)
{
	SA_Read_out tmp; // Return buffer.
	readCmd cmdData;
	uint8_t size = sizeof(tmp); // In: size of buffer / Out: number of returned bytes.
	int8_t status = SA_FUNCFAIL;

	if (!OutBuf)
		return SA_BADPARM;

	// Copy command data.
	memmove(cmdData.bytes, cmd_readROM_00, sizeof(cmdData)); // Any read command (op-code = 2) will do.

	// Copy the input parameters.
	cmdData.readCmdin.mode = Mode;
	cmdData.readCmdin.address = Address;

	status = SAC_SendAndReceive(cmdData.bytes,			// command data
										(uint8_t *) &tmp,			// receive buffer
										SA_DELAY_EXEC_READ,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	// Check returned parameters.
	if (size != sizeof(SA_Read_out))
		return SA_BADSIZE;

	// Copy to output.
	memmove(OutBuf->bytes, tmp.SAdata, sizeof(OutBuf->bytes));

	return status;
}


/** \brief This function reads status fuses.
 *
 * \param[out]	outBuf pointer to response data
 * \return		status of the operation
 */
int8_t SAFI_ReadStatus(SA_statusBuf *outBuf)
{
	SA_Read_out tmp; // Return buffer.
	uint8_t size = sizeof(tmp); // In: size of buffer / Out: number of returned bytes.
	int8_t status;

	if (!outBuf)
		return SA_BADPARM;

	status = SAC_SendAndReceive(cmd_readFuse_02,			// command data
										(uint8_t *) &tmp,			// receive buffer
										SA_DELAY_EXEC_READ,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	// Check returned parameters.
	if (size != sizeof(SA_Read_out))
		return SA_BADSIZE;

	// Copy to output.
	memmove(outBuf->bytes, tmp.SAdata, sizeof(outBuf->bytes));

	return status;
}


/** \brief This function reads the MemValid flag (SA100 only).
 * \param[out]	outBuf pointer to response data
 * \return		status of the operation
 */
int8_t SAFI_ReadMemValid(SA_memValidBuf *outBuf)
{
	SA_Read_out tmp; // Return buffer.
	uint8_t size = sizeof(tmp); // In: size of buffer / Out: number of returned bytes.

	int8_t status;

	if (!outBuf)
		return SA_BADPARM;

	status = SAC_SendAndReceive(cmd_readMemValid,		// command data
										(uint8_t *) &tmp,			// receive buffer
										SA_DELAY_EXEC_READ,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	// Check returned parameters.
	if (size != sizeof(SA_Read_out))
		return SA_BADSIZE;

	// Copy to output. Only the LSB is important.
	outBuf->sramValid = tmp.SAdata[3];

	return status;
}


/** \brief This function reads the device revision.
 * \param[out]	outBuf pointer to read data
 * \return		status of the operation
 */
int8_t SAFI_ReadRevision(SA_revNumBuf *outBuf)
{
	SA_Read_out tmp; // Return buffer.
	uint8_t size = sizeof(tmp); // In: size of buffer / Out: number of returned bytes.

	int8_t status;

	if (!outBuf)
		return SA_BADPARM;

	status = SAC_SendAndReceive(cmd_readROM_01,			// command data
										(uint8_t *) &tmp,			// receive buffer
										SA_DELAY_EXEC_READ,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	// Check returned parameters.
	if (size != sizeof(SA_Read_out))
		return SA_BADSIZE;

	// Copy to output.
	memmove(outBuf->bytes, tmp.SAdata, sizeof(outBuf->bytes));

	return status;
}


/** \brief This function reads the three-byte manufacturer id.
 *
 * ManufacturerId is 24 bits / 3 bytes (16 bits from ROM, 8 bits from fuses).
 * It consists of bytes 0 and 1 of ROM address 0 combined with fuse[88 to 95].
 * \param[out]	outBuf pointer to read data
 * \return		status of the operation
 */
int8_t SAFI_ReadManufacturerId(SA_mfrID_buf *outBuf)
{
	SA_Read_out tmp; // Return buffer.
	uint8_t size = sizeof(tmp); // In: size of buffer / Out: number of returned bytes.
	int8_t status;

	if (!outBuf)
		return SA_BADPARM;

	// We need to:
	//
	//		do a ROM read 0,
	// 			- grab bytes 0,1
	//
	//		do a fuse read "02"
	//			- grab byte 3
	//
	//		return 3 bytes MfrID
	//
	status = SAC_SendAndReceive(cmd_readROM_00,			// command data
										(uint8_t*) &tmp,			// receive buffer
										SA_DELAY_EXEC_READ,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
	  return status;

	// Check returned parameters.
	//
	if (size != sizeof(SA_Read_out))
		return SA_BADSIZE;

	// Copy to output.
	//
	memmove(outBuf->mfrID_data.ROMdata,	tmp.SAdata,	sizeof(outBuf->mfrID_data.ROMdata));
	memset((void *) &tmp, 0, sizeof(tmp));

	// Now do a read fuse 02.
	//
	status = SAC_SendAndReceive(cmd_readFuse_02,			// command data
										(uint8_t*) &tmp,			// receive buffer
										SA_DELAY_EXEC_READ,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	// Check returned parameters.
	if (size != sizeof(SA_Read_out))
		return SA_BADSIZE;

	// Copy to output.
	outBuf->mfrID_data.fusedata[0] = tmp.SAdata[3];

	return status;
}


/** \brief This function reads the three-byte serial number from the device.
 *
 * Serial number is 48 bits / 6 bytes (16 bits from ROM, 32 bits from fuses).
 * It consists of bytes 2 and 3 of ROM address 0 combined with fuse[96 to 127].
 * \param[out]	outBuf pointer to read data
 * \return		status of the operation
 */
int8_t SAFI_ReadSerialNumber(SA_snBuf* outBuf)
{
	SA_Read_out tmp; // Return buffer.
	uint8_t size = sizeof(tmp); // In: size of buffer / Out: number of returned bytes.
	int8_t status;
	// We need to
	//
	//		do a ROM read 0,
	//			- grab bytes 2 and 3,
	//
	//		do a fuse read 03,
	//			- grab all 4 bytes (0 to 3),
	//
	//		and return 6 bytes SN.
	//
	if (!outBuf)
		return SA_BADPARM;

	memset((void *) &tmp, 0, sizeof(tmp));

	status = SAC_SendAndReceive(cmd_readROM_00,			// command data
										(uint8_t*) &tmp,			// receive buffer
										SA_DELAY_EXEC_READ,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	// Check returned parameters.
	if (size != sizeof(SA_Read_out))
		return SA_BADSIZE;

	// Copy to output.
	memmove(outBuf->sn_data.ROMdata,	tmp.SAdata + 2, sizeof(outBuf->sn_data.ROMdata));
	memset((void *) &tmp, 0, sizeof(tmp));

	// Now do a read fuse 03.
	status = SAC_SendAndReceive(cmd_readFuse_03,			// command data
										(uint8_t*) &tmp,			// receive buffer
										SA_DELAY_EXEC_READ,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	// Check returned parameters.
	if (size != sizeof(SA_Read_out))
		return SA_BADSIZE;

	// Copy to output.
	memmove(outBuf->sn_data.fusedata, tmp.SAdata, sizeof(outBuf->sn_data.fusedata));

	return status;
}


/** \brief This function burns one fuse.
 * \param[in]	FuseNumber fuse bit index
 * \return		status of the operation
 */
int8_t SAF_BurnFuse(uint8_t FuseNumber)
{
	SA_BurnFuse_out retData;
	BurnFusecmd cmdData;
	uint8_t size = sizeof(retData);
	int8_t status;

	// Copy command data.
	memmove(cmdData.bytes, cmd_burnFuse, sizeof(cmdData));

	// Copy the input parameters.
	cmdData.BurnFuseIn.fuseNumber = FuseNumber;
	cmdData.BurnFuseIn.zero = 0;

#ifdef LOW_VOLTAGE
	cmdData.BurnFuseIn.lowVoltage = LOW_VOLTAGE_PARAM;

	// Send the request to the chip.
	status = SAC_SendAndReceive(cmdData.bytes,				// command data
										(uint8_t*) &retData,			// receive buffer
										SA_DELAY_EXEC_FUSE_LOW_V,	// execution delay
										WAKE_FLAG | SLEEP_FLAG,		// send wakeup / sleep
										&size);							// in/out: buffer/data size
#else
	cmdData.BurnFuseIn.lowVoltage = 0;

	// Send the request to the chip.
	status = SAC_SendAndReceive(cmdData.bytes,			// command data
										(uint8_t*) &retData,		// receive buffer
										SA_DELAY_EXEC_FUSE,		// execution delay
										WAKE_FLAG | SLEEP_FLAG,	// send wakeup / sleep
										&size);						// in/out: buffer/data size
#endif

	// Check consistency of response.
	if (status != SA_SUCCESS)
		return status;

	return (retData.returnCode ? SA_NOTALLOWED : SA_SUCCESS);
}


/** \brief	This function burns any combination of the first 88 fuse bits.
 *
 *				When using low voltage and an encrypted map, the map should not contain
 *				more than 14 fuses to be blown. Otherwise, the watchdog timer might
 *				trigger before all fuses have been blown.
 *
 * \param[in]	decryptOpt	If this flag is true the Map is encrypted with the digest
 *									generated by the GenPersonalizationKey command.
 * \param[in]	mapData		Pointer to fuse map. Fuse bits set to one in the map are blown
 *									(set to zero; reverse logic).
 * \return		status of the operation
 */
int8_t SAF_BurnSecure(uint8_t decryptOpt, uint8_t *mapData)
{
	uint8_t				wakeSleep = SLEEP_FLAG; // wakeup / sleep flag option
	BurnSecurecmd		cmdData;
	SA_BurnSecure_out	retData;
	int8_t				status = SA_FUNCFAIL;
	uint8_t				size = sizeof(retData); // in: size of output buffer / out: size of returned data
#ifndef LOW_VOLTAGE
	uint8_t deviceId, hostId;
#endif
	if (!mapData)
		return SA_BADPARM;

	// Initialize copy of command data.
	memmove(cmdData.bytes, cmd_burnSecure, sizeof(cmdData));

	// Copy the input parameters.
	cmdData.BurnSecureIn.decrypt = decryptOpt & 0x01; // ensure legal value is passed

	memmove(&cmdData.BurnSecureIn.map, mapData, sizeof(cmdData.BurnSecureIn.map));

/*
	* if using encrypted map, do not send wakeup
	* as burnSecure must be run in same wakeup cycle as genPersonalizationKey
*/
   wakeSleep = SLEEP_FLAG; // must be run in same wakeup cycle
	if (!decryptOpt)
		wakeSleep |= WAKE_FLAG;

#ifdef LOW_VOLTAGE
	cmdData.BurnSecureIn.lowVoltage = LOW_VOLTAGE_PARAM;
	status = SAC_SendAndReceive(cmdData.bytes,					// command data
										(uint8_t*) &retData,				// receive buffer
										SA_DELAY_EXEC_SECURE_LOW_V,	// execution delay
										wakeSleep,							// send wakeup / sleep
										&size);								// in/out: buffer/data size
#else

	SAP_GetDeviceID(&deviceId);
	SAP_GetHostDeviceID(&hostId);
	status = SAC_SendAndReceive(cmdData.bytes,		// command data
										(uint8_t*) &retData,	// receive buffer
										deviceId == hostId ? SA_DELAY_EXEC_SECURE_HOST : SA_DELAY_EXEC_SECURE, // execution delay
										wakeSleep,				// send wakeup / sleep
										&size);					// in/out: buffer/data size
#endif

	// Check returned parameters.
	if (status != SA_SUCCESS)
		return status;

	return (retData.returnCode ? SA_NOTALLOWED : SA_SUCCESS);
}


/** \brief	This function writes 256 bits into the battery backed SRAM
 *				and locks this memory against further modification (SA100 only).
 * \param[in]	keyData pointer to encrypted value
 * \return		status of the operation
 */
int8_t SAF_LoadSram(uint8_t *keyData)
{
	LoadSramCmd cmdData;
	int8_t      status = SA_FUNCFAIL;

	// The return data for SAF_LoadSram and SAF_BurnSecure are the same.
	SA_BurnSecure_out retData;
	uint8_t size = sizeof(retData); // in: size of output buffer / out: size of returned data

	if (!keyData)
		return SA_BADPARM;

	// Copy command data.
	memmove(cmdData.bytes, cmd_loadSram, sizeof(cmdData));

	// Copy the input parameters
	memmove(&cmdData.LoadSramIn.key, keyData, sizeof(cmdData.LoadSramIn.key));

	/*
	* Do not send wakeup as LoadSram must be run in same wakeup cycle
	* as GenPersonalizationKey.
	*/
	status = SAC_SendAndReceive(cmdData.bytes,			// command data
										(uint8_t*) &retData,		// receive buffer
										SA_DELAY_EXEC_PERSON,	// execution delay
										SLEEP_FLAG,					// send sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	return (retData.returnCode ? SA_NOTALLOWED : SA_SUCCESS);
}


/** \brief This function generates a personalization key.
 * \param[in]	KeyID key id
 * \param[in]	Seed pointer to seed
 * \return		status of the operation
 */
int8_t SAF_GenPersonalizationKey(uint16_t KeyID, uint8_t* Seed)
{
	GPKcmd cmdData;
	GPKout retData;
	uint8_t size = sizeof(retData);
	int8_t status = SA_FUNCFAIL;

	if (!Seed)
		return SA_BADPARM;

	// Copy command data.
	memmove(cmdData.bytes, cmd_GPK, sizeof(GPKcmd));

	// Copy the input parameters
	//
#ifndef BIGENDIAN
	cmdData.GPKin.KeyID = (KeyID << 8) | (KeyID >> 8);
#else
	cmdData.GPKin.KeyID = KeyID;
#endif

	memmove(cmdData.GPKin.seed, Seed, sizeof(cmdData.GPKin.seed));

	status = SAC_SendAndReceive(cmdData.bytes,			// command data
										(uint8_t *) &retData,	// receive buffer
										SA_DELAY_EXEC_PERSON,	// execution delay
										WAKE_FLAG,					// never send sleep
										&size);						// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	return (retData.returnCode ? SA_NOTALLOWED : SA_SUCCESS);
}


/** \brief This function sends a PauseLong command.
 * \param[in]	Selector which device (last four status fuses) to pause
 * \return		status of the operation
 */
int8_t SAF_PauseLong(uint8_t Selector)
{
	pauseLongcmd cmdData;

	// Copy command data
	memmove(cmdData.bytes, cmd_pauseLong, sizeof(cmd_pauseLong));

	// Copy the input parameters
	cmdData.pauseLongIn.selector = Selector;

	// Send command data and receive response
	return SAC_SendData(cmdData.bytes, WAKE_FLAG) ? SA_COMMFAIL : SA_SUCCESS;
}


/** \brief This function initializes layers below.
 *
 * \param[in]	initOptions options
 * \return		status of the operation
 */
int8_t SAF_Init(uint8_t initOptions)
{
	return SAC_Init(initOptions);
}


/** \brief This function executes the Host0 opcode (0x08).
 *
 * It concatenates the key stored in the AT88SA10HS with an input 256 bit challenge and
 * generates the digest of this message. The result is left in internal memory and cannot be read.
 * In general, the challenge should be a random number generated by the host system, which will
 * be sent to both the host (AT88SA10HS) and client (AT88SA100S or AT88SA102S).
 * \param[in]	Overwrite If flag is non-zero, overwrite part of internally generated key with secret fuses.
 * \param[in]	KeyID key id
 * \param[in]	Challenge pointer to challenge that was sent to client
 * \return		status of the operation
 */
int8_t SAF_Host0(uint8_t Overwrite, uint16_t KeyID, uint8_t *Challenge)
{
	Host0Out		tmp; // Return buffer.
	Host0_cmd	cmdData;
	uint8_t		size = sizeof(tmp); // In: size of buffer / Out: number of returned bytes.
	int8_t		status = SA_FUNCFAIL;

	if (!Challenge)
		return SA_BADPARM;

	// Initialize copy of command data.
	memset(cmdData.bytes, 0, sizeof(Host0Cmd_str));
	memmove(cmdData.bytes, cmd_host0, sizeof(cmdData));

	// Set the input parameters
	cmdData.Host0CmdIn.size 		= sizeof(Host0_cmd) + sizeof(uint16_t);
	cmdData.Host0CmdIn.overwrite	= Overwrite & 1;

#ifndef BIGENDIAN
	cmdData.Host0CmdIn.keyID = (KeyID << 8) | (KeyID >> 8);
#else
	cmdData.Host0CmdIn.keyID = KeyID;
#endif

	// Move the challenge into the buffer
	//
	memmove(cmdData.Host0CmdIn.challenge, Challenge, sizeof(cmdData.Host0CmdIn.challenge));

	// Send the request
	//
	status = SAC_SendAndReceive(cmdData.bytes,		// command data
										(uint8_t*) &tmp,		// receive buffer
										SA_DELAY_EXEC_HOST0,	// execution delay
										WAKE_FLAG,				// send wakeup
										&size);					// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	return tmp.H0data[0] ? SA_NOTALLOWED : SA_SUCCESS;
}


/** \brief This function executes the Host1 opcode (0x40).
 *
 * It completes the two block SHA-256 digest started by HOST0 and leaves the resulting digest
 * within the internal memory of the AT88SA10HS. This command returns an error if HOST0
 * has not been successfully run previously within this wake cycle.\n
 * As a security precaution, this command does not return the digest. A subsequent command
 * is required to compare the response generated by the client with the one generated by the host.
 * \param[in]	OtherInfo pointer to data needed for sha256 calculation
 * \return		status of the operation
 */
int8_t SAF_Host1(Host1OtherInfo_str *OtherInfo)
{
	Host1Out		tmp; // Return buffer.
	Host1_cmd	cmdData;
	uint8_t		size = sizeof(tmp); // In: size of buffer / Out: number of returned bytes.
	int8_t		status;

#ifndef BIGENDIAN
	uint16_t savedKeyId;
#endif

	if (!OtherInfo)
		return SA_BADPARM;

	// Initialize copy of command data.
	//
	memset(cmdData.bytes, 0, sizeof(Host1Cmd_str));
	memmove(cmdData.bytes, cmd_host1, sizeof(cmdData));

	// Set the input parameters
	//
	cmdData.Host1CmdIn.size = sizeof(Host1_cmd) + sizeof(uint16_t);
	cmdData.Host1CmdIn.mode	= (OtherInfo->clientMode & SECRET) || (OtherInfo->clientMode & SECRET_STATUS) ? 0x20 : 0;

#ifndef BIGENDIAN
	// Swap key id.
	savedKeyId = OtherInfo->keyId;
	OtherInfo->keyId = (OtherInfo->keyId >> 8) | (OtherInfo->keyId << 8);
#endif

	// Move the challenge into the buffer
	//
	memmove(cmdData.Host1CmdIn.otherInfo, OtherInfo, sizeof(cmdData.Host1CmdIn.otherInfo));

#ifndef BIGENDIAN
	// Reverse key id swapping so that it is correct for next use.
	OtherInfo->keyId = savedKeyId;
#endif

	// Send the request
	//
	status = SAC_SendAndReceive(cmdData.bytes,		// command data
										(uint8_t *) &tmp,		// receive buffer
										SA_DELAY_EXEC_HOST1,	// execution delay
										0,							// no sleep or wakeup for this command
										&size); 					// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	return (tmp.H1data[0] ? SA_NOTALLOWED : SA_SUCCESS);
}


/** \brief This function executes the Host2 opcode (0x80).
 *
 * It compares the value previously generated by the AT88SA10HS using HOST0 and HOST1
 * with that on the input stream coming from the client and returns status to indicate
 * whether or not the two matched. This command returns an error if HOST1 has not been
 * previously successfully run within this wake cycle.\n
 * If the two digests do not match, the AT88SA10HS provides no information as to the
 * source of the mismatch, which must be deduced from the inputs to the three HOSTX commands.
 * On a match failure, the entire set of HOST0, HOST1, and HOST2 commands must be re-executed.
 * HOST2 cannot be repeatedly executed.
 * \param[in]	ClientResponse pointer to digest response from client MAC command
 * \return		status of the operation
 ****************************************************************
 */
int8_t SAF_Host2(uint8_t *ClientResponse)
{
	Host2Out		tmp; // Return buffer.
	Host2_cmd	cmdData;
	uint8_t		size = sizeof(tmp);
	int8_t		status = SA_FUNCFAIL;

	if (!ClientResponse)
		return SA_BADPARM;

	// Initialize copy of command data.
	memset(cmdData.bytes, 0, sizeof(Host2Cmd_str));
	memmove(cmdData.bytes, cmd_host2, sizeof(cmdData));

	// Set the command length + CRC
	cmdData.Host2CmdIn.size = sizeof(Host2_cmd) + sizeof(uint16_t);

	// Move the digest response into the buffer
	memmove(cmdData.Host2CmdIn.clientResponse, ClientResponse, sizeof(cmdData.Host2CmdIn.clientResponse));

	// Send the request
	status = SAC_SendAndReceive(cmdData.bytes,		// command data
										(uint8_t *) &tmp,		// receive buffer
										SA_DELAY_EXEC_HOST2,	// execution delay
										SLEEP_FLAG,				// only sleep for this command
										&size);					// in/out: buffer/data size
	if (status != SA_SUCCESS)
		return status;

	return (tmp.H2data[0] ? SA_NOTALLOWED : SA_SUCCESS);
}


/** \brief This function sends a PauseShort command to the host.
 * \return status of the operation
 */
int8_t SAF_PauseShort(void)
{
	PauseShort_cmd	cmdData;

	// Initialize copy of command data
	memset(cmdData.bytes, 0, sizeof(PauseShort_str));
	memmove(cmdData.bytes, cmd_pauseShort, sizeof(PauseShort_cmd));

	// Set the size + CRC
	cmdData.PauseShortIn.size = sizeof(PauseShort_cmd) + sizeof(uint16_t);

	// Send / receive command data
	return SAC_SendData(cmdData.bytes, WAKE_FLAG);
}


/** \brief Gets the number of devices registered in the device array.
 *
 * \param[in]	clientsOnly return only number of client devices
 * \return		number of devices (including potential host or not)
 */
uint8_t SAFI_GetDeviceCount(uint8_t clientsOnly)
{
	// Search for hosts.
	uint8_t nHosts = 0, i;
	DiscoveryDeviceInfo_t *deviceInfo = (void *) 0;
	uint8_t maxDeviceId = SAP_GetMaxDeviceId();

	if (!clientsOnly)
		return deviceCount;

	for (i = 0; i <= maxDeviceId; i++)
		deviceInfo = SAP_GetDeviceInfo(i);
		if (deviceInfo->deviceType == SA_DeviceTypeSA10HS)
			nHosts++;

		return deviceCount - nHosts;
}


/** \brief This function tries to find devices by using communication channel information
 * supplied in the device info structure.
 *
 * This routine supports different or same devices on different CPU port pins.
 * It communicates on all pins one at a time trying to find out which device is present
 * if at all. It then supplies device information in the device array structure.
 *
 * \return the number of devices found
 */
uint8_t SAFI_DiscoverDevices(void)
{
	int8_t status = SA_FUNCFAIL;
	SA_memValidBuf memValidData;
	DiscoveryDeviceInfo_t *deviceInfo = (void *) 0;
	uint8_t hostId = NO_HOST_ID;
	uint8_t deviceId = NO_DEVICE_ID;
	uint8_t maxDeviceId = SAP_GetMaxDeviceId();

	// Try to communicate using device id.
	for (deviceId = 0; deviceId <= maxDeviceId; deviceId++) {

		deviceInfo = SAP_GetDeviceInfo(deviceId);

		SAP_SetDeviceID(deviceId);

		// Try to find a client on this pin.
		SAP_SetHostDeviceID(NO_HOST_ID);

		status = SAFI_ReadSerialNumber((SA_snBuf *) (deviceInfo->serialNumber));
		if (status == SA_SUCCESS) {
			// Found client.
			status = SAFI_ReadStatus((SA_statusBuf *) (deviceInfo->statusFuses));
			if (status != SA_SUCCESS)
				// Something went wrong. We could read the serial number,
				// but not the status fuses.
				continue;

			deviceInfo->deviceId = deviceId;
			deviceInfo->deviceAddress = (deviceInfo->statusFuses[2] >> 4) & 0x0F;
			deviceCount++;

			// Check whether the client is an SA100S or SA102S device by
			// reading the MemValid flag.
			status = SAFI_ReadMemValid(&memValidData);
			if (status == SA_SUCCESS) {
				// Found SA100S. Update device info.
				deviceInfo->deviceType = SA_DeviceTypeSA100;
				deviceInfo->memValid = memValidData.sramValid;
			}
			else {
					// Must be an SA102S device.
				deviceInfo->deviceType = SA_DeviceTypeSA102;
				deviceInfo->isConfigured = (deviceInfo->statusFuses[2] & BURN_SECURE_ENABLE) ? 0 : 0xFF;
			}
			// If there are several clients, the physical layer remembers only the last one.
			// If the application wants to use other clients, it has to cycle through the list
			// of devices (DiscoveryDeviceInfo_t) to find them. It then can call the function below
			// with the desired device id.
			SAP_SetClientDeviceID(deviceId);
		}
		else {
		// No client found on this pin. Look for a host.
			SAP_SetHostDeviceID(deviceId);
			status = SAFI_ReadSerialNumber((SA_snBuf *) (deviceInfo->serialNumber));
			if (status != SA_SUCCESS)
				// no host found
				continue;

			// Host found. Get its status bytes.
			status = SAFI_ReadStatus((SA_statusBuf *) (deviceInfo->statusFuses));
			if (status != SA_SUCCESS)
				continue;

			deviceInfo->deviceType = SA_DeviceTypeSA10HS;
			deviceInfo->deviceId = hostId = deviceId;
			deviceInfo->deviceAddress = (deviceInfo->statusFuses[2] >> 4) & 0x0F;
			deviceInfo->isConfigured = (deviceInfo->statusFuses[2] & BURN_SECURE_ENABLE) ? 0 : 0xFF;
			deviceCount++;
		}
	}

	// Update physical layer with host id.
	SAP_SetHostDeviceID(hostId);

	return deviceCount;
}
