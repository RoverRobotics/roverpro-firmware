
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

// Doxygen header
//
/** \file
 *  \brief	This file holds the definitions and prototypes for the generic
 *				(hardware independent) Communication Layer.
 *  \date 	Sept 10, 2009
*/
#ifndef SA_COMM_H
#define SA_COMM_H

#include <stdint.h>					// C99 standard typedefs

#include "SA_Phys.h"


#define IO_BUF_SIZE		88			//!< CryptoAuth IO buffer

// external declarations
extern uint8_t CRCbad;				//!< flag indicating that CRC check failed

#ifndef false
	#define false			0			//!< boolean definition for false
#endif

#ifndef true
	#define true			1			//!< boolean definition for true
#endif

// These IO flags are common to all CryptoAuthentication client devices.
#define FLAG_CLIENT_COMMAND	(uint8_t) 0x77	//!< This flag tells the client that a command follows.
#define FLAG_CLIENT_TRANSMIT	(uint8_t) 0x88	//!< This flag requests the client to send the result of last operation.
#define FLAG_SLEEP				(uint8_t) 0xCC	//!< This flag requests the client to go to sleep.

// These IO flags are used by CryptoAuthentication host device.
#define FLAG_HOST_COMMAND		(uint8_t) 0x66	//!< Tells the host that a command follows.
#define FLAG_HOST_TRANSMIT		(uint8_t) 0x99	//!< Requests the host to send the result of last operation.


/*
 * Definitions for UART based communications
 */

#define WAKE_TOKEN				(uint8_t) 0xF0	//!< byte value when sending wake token through UART
#define ZERO_BIT					(uint8_t) 0x7D	//!< byte value when sending a 'zero' bit through UART
#define ONE_BIT					(uint8_t) 0x7F	//!< byte value when sending a 'one' bit through UART


/*
 * CryptoAuth IO flags
 */

/** \brief The IO flags below are used for all CryptoAuthentication devices.
 *
 * They control sending of wake token / sleep flag for every send / response pair.
*/
#define WAKE_SYNC					(uint8_t) 0x11		//!< wakeup sync
#define WAKE_FLAG					(uint8_t) 0x01		//!< flag: send wakeup token before command
#define SLEEP_FLAG				(uint8_t) 0x02		//!< flag: send sleep flag after command
#define NO_CRC_CALC_FLAG		(uint8_t) 0x04		//!< flag: suppress CRC calculation for this command

#define CRCsize					sizeof(uint16_t)	//!< 16-bit CRC


/*
 *  type definitions
 */

//! generic CryptoAuth command structure
typedef PACKED_ATTRIBUTE1 struct __cmdFixed
{
	uint8_t		Opcode;			//!< command code
	uint8_t		Param1;			//!< first parameter
	uint16_t		Param2;			//!< second parameter
} PACKED_ATTRIBUTE2 _cmdFixed;


//! structure for command bytes plus one data byte
typedef PACKED_ATTRIBUTE1 struct _cmdPacket
{
	_cmdFixed	cmdFixed;		//!< now we can do sizeof(cmdFixed)
	uint8_t		Data[1];			//!< marker: array of 1 means no casting / "anding" to get address
} PACKED_ATTRIBUTE2 cmdPacket;


//! structure for entire tx packet excluding CRC
typedef PACKED_ATTRIBUTE1 struct _IOblockInA
{
	uint8_t		Count;			//!< total number of bytes in tx packet (including Count and CRC)
	cmdPacket	Packet;			//!< command packet structure
} PACKED_ATTRIBUTE2 IOblockInA;


//! structure for tx CRC
typedef PACKED_ATTRIBUTE1 struct _IOblockInB
{
	uint16_t	_CRC16;				//!< two-byte CRC
} PACKED_ATTRIBUTE2 IOblockInB;


//! structure for response
typedef PACKED_ATTRIBUTE1 struct _IOblockOutA
{
	uint8_t		Count;			//!< total number of bytes in rx packet (including Count and CRC)
	uint8_t		Data[1];			//!< status byte
} PACKED_ATTRIBUTE2 IOblockOutA;


//! structure for rx CRC
typedef PACKED_ATTRIBUTE1 struct _IOblockOutB
{
	uint16_t	_CRC16;				//!< two-byte CRC
} PACKED_ATTRIBUTE2 IOblockOutB;


// pointer macros to access IO data
//
//!< pointer definition for tx buffer
#define mIOblockInA ((IOblockInA *)		IObuf)

//!< pointer definition for CRC of tx buffer
#define mIOblockInB ((IOblockInB *)	(mIOblockInA->Packet.Data + 					/* start at the variable part */ 	\
												 mIOblockInA->Count - 							/* add the Count */					\
												 sizeof(mIOblockInA->Packet.cmdFixed) -	/* subtract the fixed part out */	\
												 CRCsize  -										   /* CRC itself not part of length*/	\
												 sizeof(mIOblockInA->Packet.Data)))			/* subtract the marker size */


//!< pointer definition for rx buffer
#define mIOblockOutA ((IOblockOutA *) 	IObuf)

//!< pointer definition for CRC of rx buffer
#define mIOblockOutB ((IOblockOutB *)	(mIOblockOutA->Data + \
													(mIOblockOutA->Count - \
													 CRCsize - \
													 sizeof(mIOblockOutA->Data))))

//!< definition for total number of rx bytes minus 1 for the Count byte
#define bytesToGet (mIOblockOutA->Count - sizeof(mIOblockOutA->Count))			/* already got Count */

//!< for grabbing entire structure
#define mIOblockOut (uint8_t*) IObuf


// Function prototypes
//
void SAC_SendSleepFlag(void);
void SAC_SendByte(uint8_t byteData);
int8_t SAC_ReceiveByte(uint8_t *ByteData)  INLINE_ATTRIBUTE;
int8_t SAC_SendAndReceive(uint8_t *commandData, uint8_t *responseData, SA_Delay execDelay, uint8_t wakeSleepCrc, uint8_t *size);
int8_t SAC_SendData(uint8_t *commandData, uint8_t wakeSleepCrc);
int8_t SAC_Init(uint8_t initOptions);
int8_t SAC_Wake(void);


#endif
