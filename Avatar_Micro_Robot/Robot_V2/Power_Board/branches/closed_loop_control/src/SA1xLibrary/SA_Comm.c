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
//
/** \file
 *  \brief 	driver for sending command and receiving response packets
 *  \date 	Sept 10, 2009
*/
#include <ctype.h>
#include <stdint.h>
#include <string.h>

// support header files for CryptoAuth library
//
#include "SA_Comm.h"

/*
 * communication layer globals
 */
uint8_t CRCbad;						//!< CRC error has been detected

#ifndef AVR_UART
	uint8_t	byteBuf[8];				//!< debug: CryptoAuth byte buffer
#endif

//! the CryptoAuthentication IO buffer
uint8_t IObuf[IO_BUF_SIZE];		// note: AVR-GCC bug: throws away UDR if assigned to local var!!




/** \brief This function calculates a 16-bit CRC.
 *
 * \param[in] pData pointer to data for which CRC should be calculated
 * \param[in] _LEN number of bytes in buffer
 * \return
 */
uint16_t SACI_CRC(uint8_t *pData, uint8_t _LEN)
{
	uint8_t counter;
	uint16_t _CRC = 0x0000;
	uint16_t poly = 0x8005;
	uint16_t i;
	uint8_t dataBit, crcBit;

	for (counter = 0; counter < _LEN; counter++) {
		for (i = 0x1; i < 0x100; i <<= 1) {
			dataBit = (pData[counter] & (uint8_t) i) ? 1 : 0;
			crcBit = (0x8000 & _CRC) ? 1 : 0;
			_CRC <<= 1;
			if (dataBit ^ crcBit)
				_CRC ^= poly;
		}
	}
#ifdef BIGENDIAN
	_CRC = (_CRC << 8) | (_CRC >> 8);  // flip byte order
#endif

	return _CRC;
}


/** \brief This function checks whether the currently selected device is a host device.
 *
 * @return 0xFF if the device is a host, 0 if it is a client
 */
uint8_t SACI_IsHost(void)
{
	uint8_t hostId = NO_HOST_ID, deviceId = NO_DEVICE_ID;
	SAP_GetHostDeviceID(&hostId);
	SAP_GetDeviceID(&deviceId);
	return (deviceId == hostId ? 0xFF : 0);
}


/** \brief	This function sends one byte to the CryptoAuth chip via bit-bang or UART.
 *  \param[in]	byteData
 *  \return		void
 ****************************************************************
 */
void SAC_SendByte(uint8_t byteData)
{
	uint8_t	i;

	for (i = 0; i < 8; i++)
	{
		if (byteData & (1 << i))
			SAP_SendOne();
		else
			SAP_SendZero();
	}
}


/** \brief	This function receives one byte.
 *
 *				When using a UART driver, it assembles a CryptoAuthentication "byte"
 *				from eight received UART characters - every character
 * 			represents a CryptoAuthentication "bit".
 *	 \param[out]	ByteData pointer to received byte
 *  \return			status of the operation
 */
int8_t SAC_ReceiveByte(uint8_t *ByteData)
{
	uint8_t i, returnByte = 0;
	int8_t status = SA_FUNCFAIL;

	// read 8 "bits"
	for (i = 0; i < 8; i++)
	{
		status = SAP_GetBit(&byteBuf[i]);
		if (status != SA_SUCCESS)
			return status;

		// GetBit should return a 1 or 0 to make bit-banging as responsive as possible.
		if ((byteBuf[i] ^ 0x7F) & 0x7C) 	// any but the 2 LSBs == 0
			returnByte &= ~(1 << i);
		else
			returnByte |= (1 << i);
	}

	*ByteData = returnByte;
	return status;
}


/** \brief This function sends a Sleep flag to the device.
 *
 * \return void
 */
void SAC_SendSleepFlag(void)
{
	SAP_DisableIoTimer();

	SAP_TxMode();

	// Put CryptoAuth to sleep. Sleep flag is same for client and host.
	SAC_SendByte(FLAG_SLEEP);

	// Let transmission finish
	SAP_WaitForXmitComplete();

	// Switch pin back to input.
	SAP_RxMode();

	// Mandatory sleep->wake delay (not spec'd)
	SAP_ExecDelay(SA_DELAY_SLEEP_WAKE);
}


/** \brief This function performs a wakeup sequence.
 *
 * 1. Generate wakeup pulse
 * 2. Send transmit request flag
 * 3. Receive response to wake
 * 4. Verify received CRC
 *  \return status of the operation
 */
int8_t SAC_Wake(void)
{
	/*
	* Wakeup the chip:
	* 1. Generate wakeup pulse.
	* 2. Send transmit request flag.
	* 3. Receive response to wake.
	* 4. Verify received CRC.
	*/

	int8_t status = SA_FUNCFAIL;
	uint8_t i;
	uint8_t isHost = SACI_IsHost();

	// Debug: clear IObuf
	memset(IObuf, 0, sizeof(IObuf));

	// Enable transmit mode
	SAP_TxMode();

	// Send wakeup token, transmit flag
	SAP_Wakeup();

	SAC_SendByte(isHost ? FLAG_HOST_TRANSMIT : FLAG_CLIENT_TRANSMIT);

	// Let transmission finish
	SAP_WaitForXmitComplete();

	// Enable receive mode
	SAP_RxMode();

	// Enable IO timeout counter
	SAP_EnableIoTimer();

	// Get size (count) and extract block size
	status = SAC_ReceiveByte(&mIOblockOutA->Count);
	if (status != SA_SUCCESS) {
		// It cannot hurt to send a sleep flag in case we woke up the wrong device,
		// mixing up client with host.
		SAC_SendSleepFlag();
		return status;
	}

	// Read entire block
	for (i = 0; i < bytesToGet; i++)
	{
		status = SAC_ReceiveByte(&mIOblockOutA->Data[i]);
		if (status != SA_SUCCESS)
		{
			SAC_SendSleepFlag();
			return status;
		}
	}

	// Disable IO timeout counter
	SAP_DisableIoTimer();

	// Check returned CRC
	if (mIOblockOutB->_CRC16 != SACI_CRC((uint8_t*)	mIOblockOutA, mIOblockOutA->Count - CRCsize))
	{
		CRCbad = 1;
		return SA_BAD_CRC;
	}
	return SA_SUCCESS;
}


/** \brief	This function receives a CryptoAuthentication command block and
 *				generates wakeup / sleep tokens if needed.
 *	 \param[in]			commandData pointer to tx data
 *  \param[out]		responseData pointer to rx data
 *  \param[in]			execDelay how long it takes the device to execute this command
 *	 \param[in]			wakeSleepCrc Bit flags indicate whether to wakeup the device, put it to sleep, or supress the CRC
 *  \param[in, out]	size in: expected size, out: size of response
 *  \return				the status of the operation
 */
int8_t SAC_SendAndReceive(uint8_t	*commandData,		// data to send
									uint8_t	*responseData,		// where to put output data
									SA_Delay	execDelay,			// command execution delay
									uint8_t	wakeSleepCrc,		// flags: send wakeup/sleep/no_crc tokens
									uint8_t	*size)				// in: output buffer size / out: bytes returned
{
	uint8_t i;
	int8_t status = SA_FUNCFAIL;
	uint8_t isHost = SACI_IsHost();

	if (!commandData || !responseData || !size)
		return SA_BADPARM;

	if (((IOblockInA *) commandData)->Count > sizeof(IObuf))
		return SA_INVALID_SIZE;

	// Clear any previous CRC errors.
	CRCbad = 0;

	// Debug: clear IObuf
	status = SAC_SendData(commandData, wakeSleepCrc);
	if (status != SA_SUCCESS)
	{
		SAC_SendSleepFlag();
		return status;
	}

	// Wait execution delay time
	SAP_ExecDelay(execDelay);

	// Debug: clear IObuf
	memset(IObuf, 0, sizeof(IObuf));

	// Now get the response
   SAC_SendByte(isHost ? FLAG_HOST_TRANSMIT : FLAG_CLIENT_TRANSMIT);

	// Let transmission of transmit flag finish
	SAP_WaitForXmitComplete();

	// Enable receive mode
	SAP_RxMode();

	// Enable IO timeout counter
	SAP_EnableIoTimer();

	// Get block size (count)
	status = SAC_ReceiveByte(&mIOblockOutA->Count);
	if (status != SA_SUCCESS)
	{
		SAP_DisableIoTimer();
		SAC_SendSleepFlag();
		return status;
	}

	// Read entire block
	// Since reading the first byte succeeded, we don't bother to check for error here.
	for (i = 0; i < bytesToGet; i++)
		SAC_ReceiveByte(&mIOblockOutA->Data[i]);

	// Disable IO timeout counter
	SAP_DisableIoTimer();

	if (wakeSleepCrc & SLEEP_FLAG) {
		SAC_SendSleepFlag();
	}

	// Check returned CRC
	if (mIOblockOutB->_CRC16 != SACI_CRC((uint8_t*)	mIOblockOutA, mIOblockOutA->Count - CRCsize))
	{
		CRCbad = 1;
		return SA_BAD_CRC;
	}

	// Copy response to output if possible
	if (*size < mIOblockOutA->Count)
	{
		*size = mIOblockOutA->Count;
		return SA_INVALID_SIZE;
	}

	// Clear, then copy data to response buffer
	memset(responseData, 0, *size);
	memmove(responseData, mIOblockOut, mIOblockOutA->Count);

	// Update output size
	*size = mIOblockOutA->Count;
	return status;
}


/** \brief	This function sends a CryptoAuthentication command block without receiving a response,
 * 			and generates a wakeup token if needed.
 *	 \param[in]	commandData pointer to command packet
 *	 \param[in]	wakeSleepCrc Bit flags indicate whether to wakeup the device, put it to sleep, or supress the CRC
 *  \return		status of the operation
 */
int8_t SAC_SendData(uint8_t *commandData, uint8_t wakeSleepCrc)
{
	uint8_t i;
	int8_t status = SA_FUNCFAIL;
	uint8_t isHost = SACI_IsHost();

	if (!commandData)
		return SA_BADPARM;

	// Clear any previous CRC errors.
	CRCbad = 0;

	if (wakeSleepCrc & WAKE_FLAG)
	{
		status = SAC_Wake();
		if (status != SA_SUCCESS)
		{
			SAC_SendSleepFlag();
			return status;
		}
	}
	// Disable IO timeout counter
	// SAC_Wake disables the timer before returning.
	else
		SAP_DisableIoTimer();

	// Send a CryptoAuth command:
	// Prepare command bytes to send
	//	Send command flag
	//	Send command bytes
	//	Wait CryptoAuth execution delay
	//	Send transmit flag

   // Copy command data to IObuf
	if (((IOblockInA*) commandData)->Count <= sizeof(IObuf) )
		memmove(IObuf, commandData, ((IOblockInA*) commandData)->Count);
	else
		return SA_INVALID_SIZE;

	// Check to see if the CRC calculation is required (code is executed when bit is not set)
	if (!(wakeSleepCrc & NO_CRC_CALC_FLAG))
	{
		// Calculate and insert output CRC
		mIOblockInB->_CRC16 = SACI_CRC((uint8_t *) mIOblockInA, mIOblockInA->Count - CRCsize);
	}
	// Enable transmit mode
	SAP_TxMode();

	// Send command
	SAC_SendByte(isHost ? FLAG_HOST_COMMAND : FLAG_CLIENT_COMMAND);

	for (i = 0; i < IObuf[0]; i++)
		SAC_SendByte(IObuf[i]);

	// Let transmission finish
	SAP_WaitForXmitComplete();

	// Don't attempt to receive response
	return SA_SUCCESS;
}


/** \brief	This function initializes timer and communication hardware.
 *
 * There is no mechanism that prevents multiple calls
 * to this routine that in turn initializes hardware peripherals.
 * This might lead to unexpected behavior depending on the
 * system and hardware configuration if such multiple calls
 * are not fully evaluated.
 *
 *	 \param[in]	initOptions initialization options
 *  \return		status of the operation
 ****************************************************************
*/
int8_t SAC_Init(uint8_t initOptions)
{
	// Initialize hardware driver
	SAP_Init();

	return SA_SUCCESS;
}
