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
/**	\file
*	\brief	This file contains functions that implement the API layer for
				CryptoAuthentication devices SA100S, SA102S, and SA10HS.
*	\date	Nov 12, 2009
*/
#include <string.h>

// header files
//
#include "SA_API.h"
#include "sha256.h"


/* globals */

//! SA1x library version
version_t libVersion = {0x01, 0x01, 0x00};


/* API layer functions */

/**\brief		This function sends a MAC command.
*	\param[in]	Mode what to include in the MAC calculation
*	\param[in]	KeyID key id
*	\param[in]	Challenge pointer to 32 bytes of challenge data
*	\param[out]	Response pointer to 32 bytes of response data (MAC)
*	\return		status of the operation
*/
int8_t SA_DoMAC(uint8_t Mode, uint16_t KeyID, uint8_t *Challenge, uint8_t *Response)
{
	return SAF_MAC(Mode, KeyID, Challenge, Response);
}


/**	\brief	This function takes a digest generated by SA_GenMac, or SA_GenMac100,
*					or externally, and sends a MAC command to the device to compare.
*
*	\param[in]	Mode what to include in the MAC calculation
*	\param[in]	KeyID key id
*	\param[in]	Challenge   pointer to 32 bytes of challenge data
*	\param[in]	ExpectedMAC pointer to 32 bytes of expected MAC data (can be supplied by SA_GenMAC)
*	\param[out]	Success pointer to one byte of comparison result
*	\return		status of the operation
*/
int8_t SA_MatchMAC(uint8_t Mode, uint16_t KeyID, uint8_t *Challenge,	uint8_t *ExpectedMAC, int8_t *Success)
{
	uint8_t MACbuf[32];
	if (!ExpectedMAC || !Success)
		// Other pointer arguments are checked in SAF_MAC.
		return SA_BADPARM;

	*Success = SAF_MAC(Mode, KeyID, Challenge, MACbuf);
	if (*Success != SA_SUCCESS)
		return *Success;

	if (memcmp(ExpectedMAC, MACbuf, sizeof(MACbuf)))
		*Success = SA_BADPARM;

	return SA_SUCCESS;
}


/**	\brief	This function generates a MAC for a SA102S device.
*
* This function takes as parameters the needed but not readable values for a host-side MAC calculation,
* reads the needed and available values from the chip, and generates a MAC.
*	\param[in]	KeyID key id
*	\param[in]	Key pointer to 32 bytes of key
*	\param[in]	Challenge pointer to 32 bytes of challenge
*	\param[in]	Mode what to include in the MAC generation
*	\param[in]	Secret pointer to 16 bytes of secret seed
*	\param[out]	MAC pointer to 32 bytes of calculated MAC
*	\return		status of the operation
*/
int8_t SA_GenMac(uint16_t KeyID, uint8_t *Key, uint8_t *Challenge, uint8_t Mode, uint8_t *Secret, uint8_t *MAC)
{
	// buffers for visible chip data
	SA_snBuf snData;
	SA_statusBuf statusData;
	SA_mfrID_buf mfrIDdata;
	uint8_t secretBuf[SECRETSIZE];
	sha256_ctx ctx;
	uint8_t opCode = MAC_OPCODE;
	int8_t retCode = SA_SUCCESS;
#ifndef BIGENDIAN
	uint8_t swappedKeyID[2];
#endif

	if (!Key || !Challenge || !MAC || (((Mode & SECRET) || (Mode & SECRET_STATUS)) && !Secret))
		return SA_BADPARM;

	// Initialize buffers.
	memset(snData.bytes, 0, sizeof(snData.bytes));
	memset(statusData.bytes, 0, sizeof(statusData.bytes));
	memset(mfrIDdata.bytes, 0, sizeof(mfrIDdata.bytes));
	memset(secretBuf, 0, sizeof(secretBuf));

/*
		* acquire the visible chip data for this mode
		*
		* operational mode selections depend upon both
		* the input mode byte and the current chip state
		*
		* inclusion of secret and status fuses for MAC
		* input data depends upon fuse bit 87 being burned
		* (so BurnSecure must have been performed).
		* Otherwise, these fuse data are replaced by 0's.
		*
		*

		7 6 5 4 3 2 1 0	bit
		0 X X X 0 0 0 0
		      ---------- include secret and status fuses (bit 4)
		    ------------ include secret fuses only (bit 5)
		  -------------- include serial number (ROM and fuse, bit 6)

		0x10				secret, status
		0x20				secret
		0x30				secret, status
		0x40				serialNum
		0x50				serialNum, secret, status
		0x60				serialNum, secret
		0x70				serialNum, secret, status

		mode 10, 30 equivalent
		mode 50, 70 equivalent

		so really 5 modes:

		secret							0x20
		secret, status					0x10, 0x30
		serialNum						0x40
		serialNum, secret				0x60
		serialNum, secret, status	0x50, 0x70

*/

	// Only add optional data if mode != 00.
	if (Mode != 0x00) {
			// Include serialNum if selected.
			if (Mode & SERIALNUM) {
				if ((retCode = SAFI_ReadSerialNumber(&snData)) != SA_SUCCESS)
					return retCode;
			}

			// Read the status fuses to get the state of fuse 87.
			if ((retCode = SAFI_ReadStatus(&statusData)) != SA_SUCCESS)
				return retCode;

			// Check status of BurnSecureEnable flag (fuse 87).
			if (statusData.bytes[sizeof(statusData.bytes) - 1] & BURN_SECURE_ENABLE) // fuse 87 is high bit of last status byte
				memset(statusData.bytes, 0, sizeof(statusData.bytes)); // set: zero status bytes, skip other reads
			else {
				/* Include chip data the mode has selected. */

				// Check for inclusion of status fuses.
				if (!(Mode & SECRET_STATUS)) {
					// Not included: Clear status buffer.
					memset(statusData.bytes, 0, sizeof(statusData.bytes));
				}

				// Check for inclusion of secret fuses.
				if ((Mode & SECRET) || (Mode & SECRET_STATUS))
					memmove(secretBuf, Secret, sizeof(secretBuf));
			}
	}

	// Always include MfrID.
	if ((retCode = SAFI_ReadManufacturerId(&mfrIDdata)) != SA_SUCCESS)
		return retCode;

	/*
		* All the optional parameters to the MAC operation have been
		* dealt with, now perform a SHA-256 on the data to get the MAC result.
	*/

	// Initialize the hash context.
	sha256_init(&ctx);

	// key data, challenge, input parameters
	sha256_update(&ctx, Key, KEYSIZE);
	sha256_update(&ctx, Challenge, CHALLENGESIZE);
	sha256_update(&ctx, &opCode, sizeof(opCode));
	sha256_update(&ctx, &Mode, sizeof(Mode));

#ifndef BIGENDIAN
	swappedKeyID[0] = (uint8_t) (KeyID >> 8);
	swappedKeyID[1] = (uint8_t) (KeyID & 0xFF);
	sha256_update(&ctx, swappedKeyID, sizeof(KeyID));
#else
	sha256_update(&ctx, (uint8_t *) &KeyID, sizeof(KeyID));
#endif

	// fuse data
	sha256_update(&ctx, secretBuf, SECRETSIZE);
	sha256_update(&ctx, statusData.bytes, sizeof(statusData.bytes));
	sha256_update(&ctx, mfrIDdata.mfrID_data.fusedata, sizeof(mfrIDdata.mfrID_data.fusedata));
	sha256_update(&ctx, snData.sn_data.fusedata, sizeof(snData.sn_data.fusedata));

	// ROM data
	sha256_update(&ctx, mfrIDdata.mfrID_data.ROMdata, sizeof(mfrIDdata.mfrID_data.ROMdata));
	sha256_update(&ctx, snData.sn_data.ROMdata, sizeof(snData.sn_data.ROMdata));

	// final round
	sha256_final(&ctx, MAC);

	return SA_SUCCESS;
}


/**	\brief	This function generates a MAC for a SA100 device.
*
* This function takes as parameters the needed but not readable values for a host-side MAC calculation,
* reads the needed and available values from the chip, and generates a MAC.
*	\param[in]	Key pointer to 32 bytes of key data
*	\param[in]	Challenge pointer to 32 bytes of challenge
*	\param[in]	Mode what to include in the MAC generation
*	\param[out]	MAC pointer to 32 bytes of calculated MAC
*	\return		status of the operation
*/
int8_t SA_GenMac100(uint8_t *Key, uint8_t *Challenge, uint8_t Mode, uint8_t *MAC)
{
	// Buffers for visible chip data.
	SA_snBuf snData;
	SA_statusBuf statusData;
	SA_memValidBuf memValidData;
	SA_mfrID_buf mfrIDdata;
	uint8_t secretBuf[SECRETSIZE];
	sha256_ctx ctx;
	uint8_t opCode = MAC_OPCODE;
	uint8_t keyID[2] = {0, 0};	// Key id is always 0.
	int8_t retCode = SA_SUCCESS;

	if (!Key || !Challenge || !MAC)
		return SA_BADPARM;

	// Read and check MemValid.
	memValidData.sramValid = 0;
	if ((retCode = SAFI_ReadMemValid(&memValidData)) != SA_SUCCESS)
		return retCode;

	if (!memValidData.sramValid)
		return SA_NOTALLOWED;

	// Initialize buffers.
	memset(snData.bytes, 0, sizeof(snData.bytes));
	memset(statusData.bytes, 0, sizeof(statusData.bytes));
	memset(mfrIDdata.bytes, 0, sizeof(mfrIDdata.bytes));
	memset(secretBuf, 0, sizeof(secretBuf));

	/*
		* acquire the visible chip data for this mode
		*

		7 6 5 4 3 2 1 0  bit
		0 X 0 0 0 0 0 0
		  ------------- include serial number (ROM and fuse, bit 6)

		0x40          serialNum
	*/

	 if (Mode & SERIALNUM) {
			// Add serial number.
			if ((retCode = SAFI_ReadSerialNumber(&snData)) != SA_SUCCESS)
				 return retCode;
	 }

	 // Always include MfrID.
	 if ((retCode = SAFI_ReadManufacturerId(&mfrIDdata)) != SA_SUCCESS)
			return retCode;

	/*
		* all the optional parameters to the MAC operation have been
		* dealt with, now perform a SHA-256 on the data to get the
		* MAC result.
	*/

	 // Initialize the hash context.
	 sha256_init(&ctx);

	 // key data, challenge, input parameters
	 sha256_update(&ctx, Key, KEYSIZE);
	 sha256_update(&ctx, Challenge, CHALLENGESIZE);
	 sha256_update(&ctx, &opCode, sizeof(opCode));
	 sha256_update(&ctx, &Mode, sizeof(Mode));
	 sha256_update(&ctx, keyID, sizeof(keyID)); // Key id is always 0.

	 // fuse data
	 sha256_update(&ctx, secretBuf, SECRETSIZE);  // all zeroes
	 sha256_update(&ctx, statusData.bytes, sizeof(statusData.bytes)); // all zeroes
	 sha256_update(&ctx, mfrIDdata.mfrID_data.fusedata, sizeof(mfrIDdata.mfrID_data.fusedata));
	 sha256_update(&ctx, snData.sn_data.fusedata, sizeof(snData.sn_data.fusedata));

	 // ROM data
	 sha256_update(&ctx, mfrIDdata.mfrID_data.ROMdata, sizeof(mfrIDdata.mfrID_data.ROMdata));
	 sha256_update(&ctx, snData.sn_data.ROMdata, sizeof(snData.sn_data.ROMdata)); // zeroes if mode == 0

	 // final round
	 sha256_final(&ctx, MAC);

	 return SA_SUCCESS;
}


/**	\brief	This function performs a read on specific chip area.
*	\param[in]	Select what to read
*	\param[out]	Buffer pointer to 1 to 6 bytes of response data
*	\return		status of the operation
*/
int8_t SA_Read(readSelect Select, uint8_t *Buffer)
{
	switch (Select) {
		case getSN:
				return SAFI_ReadSerialNumber((SA_snBuf*) Buffer);

		case getStatus:
			return SAFI_ReadStatus((SA_statusBuf*) Buffer);

		case getMfrID:
			return SAFI_ReadManufacturerId((SA_mfrID_buf*) Buffer);

		case getRevNum:
			return SAFI_ReadRevision((SA_revNumBuf*) Buffer);

		case getMemValid:
			return SAFI_ReadMemValid((SA_memValidBuf*) Buffer);

		default:
			return SA_BADPARM; // illegal selection value
	}
}


/**	\brief	This function performs a burn command that sets the selected fuse bit to zero.
*	\param[in]	FuseNumber fuse index 0 (SA100) or 64 to 87
*	\return		status of the operation
*/
int8_t SA_Burn(uint8_t FuseNumber)
{
	// For SA102S, the value passed in must be the address of a status fuse, i.e. 64 to 87 inclusive.
	// SA100S allows fuse numbers below 64.
	uint8_t fuseNumberMin = (SAP_GetDeviceType() == SA_DeviceTypeSA100) ? 0 : 64;

	if ((FuseNumber < fuseNumberMin) || (FuseNumber > 87))
		return SA_BADPARM;

	return SAF_BurnFuse(FuseNumber);
}


/**	\brief	This function burns the supplied pattern of the first 88 fuse bits. This function is not supported by SA100.
*	\param[in]	Decrypt If this flag is true the Map is encrypted with the digest generated
*					by the GenPersonalizationKey command.
*	\param[in]	Map pointer to 11 bytes of fuse map. Fuse bits set to one in the map are blown (set to zero; reverse logic).
*	\param[in]	PersKeyID key id used in the GenPersonalizationKey command. Only needed if Decrypt flag is set.
*	\param[in]	Seed pointer to 16 bytes of Seed data used in the GenPersonalizationKey command. Only needed if Decrypt flag is set.
*	\return		status of the operation
*/
int8_t SA_BurnSecure(int8_t Decrypt, uint8_t *Map, uint16_t PersKeyID, uint8_t *Seed)
{
	SA_statusBuf statusData;
	int8_t retCode;

#ifdef LOW_VOLTAGE
	uint8_t nFuses = 0;
	uint8_t fuseMask;
	int8_t fusePosition, fusePositionPrevious = -1;
	uint8_t mapIndex = 0;
	uint8_t mapStartIndex = mapIndex;
	uint8_t partialMap[MAPSIZE];
#endif

	if (!Map)
		return SA_BADPARM;

	if (Decrypt)
			// Check input parameters.
			if (!Seed)
				// If Decrypt != 0, then Seed must be non-NULL.
				return SA_BADPARM;

	// Check BurnSecureEnable fuse (87).

	retCode = SAFI_ReadStatus(&statusData);
	if (retCode != SA_SUCCESS)
		return retCode;

	if (!(statusData.bytes[2] & BURN_SECURE_ENABLE))
		return SA_FUNCFAIL;

	if (Decrypt) {
		// Pad personalization key.
		Seed[SZ_GPKSEED - 1] |= 0x01; // add "1" pad to GPKSeed

		// Have CryptoAuthentication generate the Personalization Key.
		retCode = SAF_GenPersonalizationKey(PersKeyID, Seed);
		if (retCode != SA_SUCCESS)
			return retCode;
	}

#ifndef LOW_VOLTAGE
	// Perform BurnSecure with input Map.
	return SAF_BurnSecure(Decrypt, Map);

#else
	// Since blowing a fuse can take 190 ms and the minimum watchdog time is
	// 3000 ms, we burn only 14 fuses per burn cycle.
	// That way, we have 340 ms for communication and execution per set which amounts to
	// about 160 ms (GenPersonalizationKey plus BurnSecure commands and responses plus
	// their execution times).
	//
	// If the map is encrypted, the caller has to make sure that the time it takes
	// to blow all fuses in the map does not exceed the watchdog timeout. If there are
	// more fuses to be blown, the caller could split the original map into several maps
	// whose number of fuses to be blown does not exceed the watchdog timeout. But this
	// approach creates a security risk since parts of the maps contain known bit patterns.
	// (Stretches of the map contain zeroes.)
	if (Decrypt)
		// When the map is encrypted, we cannot junk it up since decryption requires sending
		// a GenPersonalizationKey command first.
		return SAF_BurnSecure(Decrypt, Map);

	// Let's blow a maximum of 14 fuses at a time.

	do {
		// Calculate which junk of bytes we can burn at a time.
		for (fuseMask = 1, fusePosition = 0; fusePosition < 8; fuseMask <<= 1, fusePosition++) {
			if (mapIndex == MAPSIZE) {
				// We reached the end of the map. Exit if no more fuses are left to be blown.
				if (!nFuses)
					break;
			}
			else if (fuseMask & Map[mapIndex]) {
				if (++nFuses < DLY_EXEC_FUSE_COUNT)
					// Look at next bit in map.
					continue;
			}
			else
				// Look at next bit in map.
				continue;

			// Create partial map.
			memset(partialMap, 0, MAPSIZE);
			memcpy(partialMap + mapStartIndex, Map + mapStartIndex, mapIndex - mapStartIndex);

			// Mask first byte of map part.
			partialMap[mapStartIndex] &= (0xFF << (fusePositionPrevious + 1));

			// Mask last byte of map part as long as we have not reached the end of the map yet.
			if (mapIndex < MAPSIZE)
				partialMap[mapIndex] = (Map[mapIndex] & (0xFF >> (7 - fusePosition)));

			fusePositionPrevious = fusePosition;

			// Burn partial map.
			retCode = SAF_BurnSecure(Decrypt, partialMap);
			if (retCode != SA_SUCCESS)
				return retCode;

			// Continue at this map index
			mapStartIndex = mapIndex;
			nFuses = 0;
		}
	} while (mapIndex++ < MAPSIZE);

	return retCode;
#endif
}


/**	\brief	This function blows the BurnSecure fuse using the BurnSecure command.
*
* Since the map is all zeroes, not blowing any fuses except the BurnSecureEnable fuse,
* we do this in the clear.
*	\return		status of the BurnSecure function
*/
int8_t SA_DisableBurnSecure(void)
{
	uint8_t Map[MAPSIZE] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, BURN_SECURE_ENABLE};
	return SAF_BurnSecure(0, Map);
}


/**	\brief	This function writes 256 bits into the battery backed SRAM and
					locks this memory against further modification (SA100S only).
*
* This function checks the MemValid flag first and will return an error if it is set.\n
* The value in the battery backed SRAM cannot be read, it must be verified via the MAC command. If the SRAM is already valid
* then this command will fail with an error response. The only way to unlock the SRAM is to remove power from the device.
* The input data is always decrypted using the decryption digest previously generated by GenPersonalizationKey prior to
* being written into the battery backed SRAM.\n
*	\note Both the GenPersonalizationKey and LoadSram commands must be run consecutively within a single wake cycle prior to
*			the expiration of the watchdog timer. If any command is inserted between these two operations, LoadSram will fail.
*
*	\param[in]	Key pointer to 256 bit key
*	\param[in]	PersKeyID Key id used in the GenPersonalizationKey command
*	\param[in]	Seed pointer to 16 bytes of Seed data used in the GenPersonalizationKey command
*	\return		status of the operation
*/
int8_t SA_LoadSram(uint8_t *Key, uint16_t PersKeyID, uint8_t *Seed)
{
	SA_memValidBuf memValidData;
	int8_t retCode;

	if (!Key || !Seed)
		return SA_BADPARM;


	retCode = SAFI_ReadMemValid(&memValidData);
	if (retCode != SA_SUCCESS)
		return retCode;

	if (memValidData.sramValid)
		return SA_NOTALLOWED;

	// Pad personalization key.
	Seed[SZ_GPKSEED - 1] |= 0x01; // add "1" pad to GPKSeed

	// Have CryptoAuthentication generate the Personalization Key.
	retCode = SAF_GenPersonalizationKey(PersKeyID, Seed);
	if (retCode)
		return retCode;

	// Load key into SRAM.
	return SAF_LoadSram(Key);
}


/**	\brief	This function generates a mask that encrypts ("xores") the personalization key.
*
*	\param[in]	PersonalizationKey pointer to 32 bytes of encryption key data
*	\param[in]	Seed pointer to 16 bytes of seed data
*	\param[out]	PersMask pointer 32 bytes of mask data to encrypt data with
*	\return		status of the operation
*/
int8_t SAI_GenPersonalizationMask(uint8_t *PersonalizationKey, uint8_t *Seed, uint8_t *PersMask)
{
	uint8_t GPKFixed[SZ_GPKFIXED] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	uint8_t GPKLength[SZ_GPKLENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xBF};
	sha256_ctx ctx;

	if (!PersonalizationKey || !Seed || !PersMask)
		return SA_BADPARM;

	// Pad personalization key.
	Seed[SZ_GPKSEED - 1] |= 0x01; // add "1" pad to GPKSeed

	// Generate the Personalization Mask

	// sha256 context
	sha256_init(&ctx);

	// Generate SHA256.
	sha256_update(&ctx, PersonalizationKey, SZ_PERSKEY);
	sha256_update(&ctx, GPKFixed, SZ_GPKFIXED);
	sha256_update(&ctx, Seed, SZ_GPKSEED);
	sha256_update(&ctx, GPKLength, SZ_GPKLENGTH);

	sha256_noPad(&ctx, PersMask);

	return SA_SUCCESS;
}


/**	\brief	This function produces the encrypted fuse map for SA_BurnSecure in encrypted mode.
*	\param[in]	Map pointer to 11 bytes of fuse map (in the clear)
*	\param[in]	PersonalizationKey pointer to 32 bytes of personalization key. Only first 11 bytes are used.
*	\param[in]	Seed pointer to 16 bytes of seed data
*	\param[out]	EncryptedMap pointer to 11 bytes of encrypted fuse map
*	\return	status of the operation
*/
int8_t SA_GenPersonalizationKey(uint8_t *Map, uint8_t *PersonalizationKey, uint8_t *Seed, uint8_t *EncryptedMap)
{
	uint8_t persMask[DIGESTSIZE];
	int8_t retCode;
	uint8_t i;

	if (!Map || !EncryptedMap)
		return SA_BADPARM;

	retCode = SAI_GenPersonalizationMask(PersonalizationKey, Seed, persMask);
	if (retCode != SA_SUCCESS)
		return retCode;

	// Encrypt the map.
	for (i = 0; i < MAPSIZE; i++)
		EncryptedMap[i] = (Map[i] ^ persMask[i]);

	return SA_SUCCESS;
}


/** \brief        This function produces the encrypted key for SA_LoadSram.
 *
 *  \param[in]    Key pointer to 32 bytes of clear key data to be stored with LoadSRAM command
 *  \param[in]    PersonalizationKey pointer to 32 bytes of personalization key to encrypt the key
 *  \param[in]    Seed pointer to 16 bytes of seed data
 *  \param[out]   EncryptedKey pointer to 32 bytes of encrypted key data
 *  \return			status of the operation
 */
int8_t SA_GenPersonalizationKey100(uint8_t *Key, uint8_t *PersonalizationKey, uint8_t *Seed, uint8_t *EncryptedKey)
{
	uint8_t persMask[DIGESTSIZE];
	int8_t retCode;
	uint8_t i;

	if (!Key || !EncryptedKey)
		return SA_BADPARM;

	retCode = SAI_GenPersonalizationMask(PersonalizationKey, Seed, persMask);
	if (retCode != SA_SUCCESS)
		return retCode;

	// Encrypt the key.
	for (i = 0; i < KEYSIZE; i++)
		EncryptedKey[i] = (Key[i] ^ persMask[i]);

	return SA_SUCCESS;
}


/**	\brief	This function sends a PauseLong command.
*
*  This command is used to coordinate between chips on the same pin.
*  To communicate only with one device, the following command / token / flag sequence
*  has to be performed:\n
*  Send Wakeup token -> Wakes up all devices.\n
*  Send PauseLong to all devices except the one that we want to talk to. ->
*  All other devices go into Pause state and subsequently into Sleep state
*  after their internal watchdog timer has expired.\n
*  Communicate with the selected device. -> Device executes command(s) and sends response(s).\n
*  Send Sleep flag. -> Sends the communicating device to sleep.\n
*	\param[in]	ChipSelect which device to put to sleep. Fuses 84 to 87 constitute the select ID.
*	\return		status of the operation
*/
int8_t SA_PauseLong(uint8_t ChipSelect)
{
	return SAF_PauseLong(ChipSelect);
}


/**	\brief	This function initializes the library and discovers devices.
*	\param[in]	initOptions initialization parameter for layers below
*	\return		status of the operation
*/
int8_t SA_Init(uint8_t initOptions)
{
	uint8_t deviceCount;

	int8_t status = SAF_Init(initOptions);
	if (status != SA_SUCCESS)
		return status;

	deviceCount = SAFI_DiscoverDevices();

	return deviceCount ? SA_SUCCESS : SA_NODEVICES;
}


/**	\brief	This function gets the library version.
*	\return pointer to library version (three bytes)
*/
version_t *SA_GetLibraryVersion(void)
{
	return &libVersion;
}


/**	\brief	This function lets the Host device SA10HS execute the HOST0, HOST1, and HOST2 commands in one wakeup cycle.
*	\param[in]	Overwrite If non-zero, overwrite part of internally generated key with secret fuses.
*	\param[in]	Challenge pointer to 32 bytes of challenge data sent to client
*	\param[in]	OtherInfo pointer to 13 bytes of additional data containing client device information (op-code, serial number, etc.)
*	\param[in]	Response pointer to 32 bytes of response data received from client
*	\return		status of the operation
*/
int8_t SA_HostCommand(uint8_t Overwrite, uint8_t *Challenge, Host1OtherInfo_str *OtherInfo, uint8_t *Response)
{
	int8_t status;

	if (!Challenge || !OtherInfo || !Response)
		return SA_BADPARM;

	status = SAF_Host0(Overwrite, OtherInfo->keyId, Challenge);
	if (status != SA_SUCCESS)
		return status;

	OtherInfo->opCode = MAC_OPCODE;
	status = SAF_Host1(OtherInfo);
	if (status != SA_SUCCESS)
		return status;

	return SAF_Host2(Response);
}
