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
 *  \brief 	declarations, prototypes for communication layer of CryptoAuthentication library
 *  \date 	Jan 13, 2010
*/
#ifndef SA_PHYS_H
#define SA_PHYS_H

#include <stdint.h>							// C99 standard typedefs

#include "SA_Status.h"
#include "compiler.h"

//#define BIGENDIAN								// define if compiling for big-endian machine

// execution delay constants (in microseconds or milliseconds)
#define DLY_EXEC_MAXIMUM_MS	200		//!< default execution delay


#define DLY_PARSE					  100		//!< parse time: minimum time to error response in us
#define DLY_EXEC_MAC				30000		//!< MAC execution time in us
#define DLY_EXEC_READ			 3000		//!< read memory execution time in us
#define DLY_EXEC_FUSE			  700		//!< BurnFuse execution time in us
#define DLY_EXEC_FUSE_LOW_V	  190		//!< BurnFuse execution time in ms when supply voltage is below 4.5 V
#define DLY_EXEC_SECURE			36000		//!< maximum BurnSecure execution time for SA102 device in us
#define DLY_EXEC_SECURE_HOST	26000		//!< maximum BurnSecure execution time for 10HS device in us

//! We blow up to 14 fuses at a time when using low voltage.
#define DLY_EXEC_FUSE_COUNT	(14)

//! Execution delay for BurnSecure at low voltage (ms)
#define DLY_EXEC_SECURE_LOW_V (DLY_EXEC_FUSE_COUNT * DLY_EXEC_FUSE_LOW_V)

#define DLY_PERSON				15000		//!< maximum GenPersonalizationKey execution time in us
#define DLY_HOST0					13000		//!< maximum Host0 execution time in us
#define DLY_HOST1					 7000		//!< maximum Host1 execution time in us
#define DLY_HOST2					  500		//!< maximum Host2 execution time in us
#define DLY_SLEEP_WAKE			  100		//!< minimum delay between commands when sleep is forced in us
#define DLY_WAKE_PULSE			   80		//!< width of wake pulse in us
#define DLY_WAKE					 2500		//!< delay between wake pulse and transmit flag in us

#define MAX_BIT_TIME				86000UL	//!< Maximum bit time in ns


//! named delays for CryptoAuth execution times
typedef enum
{
	SA_DELAY_PARSE,							//!< command parsing delay
	SA_DELAY_EXEC_MAC,						//!< MAC execution delay
	SA_DELAY_EXEC_READ,						//!< read delay
	SA_DELAY_EXEC_FUSE,						//!< fuse blowing delay
	SA_DELAY_EXEC_FUSE_LOW_V,				//!< fuse blowing delay at low voltage
	SA_DELAY_EXEC_SECURE,					//!< BurnSecure execution delay for client
   SA_DELAY_EXEC_SECURE_HOST,				//!< BurnSecure execution delay for host
	SA_DELAY_EXEC_SECURE_LOW_V,			//!< BurnSecure execution delay when using low voltage; depends on number of fuses to be burned
	SA_DELAY_EXEC_PERSON,					//!< GenPersonalizationKey execution delay
	SA_DELAY_EXEC_HOST0,						//!< Host0 execution delay
	SA_DELAY_EXEC_HOST1,						//!< Host1 execution delay
   SA_DELAY_EXEC_HOST2,						//!< Host2 execution delay
	SA_DELAY_SLEEP_WAKE,						//!< minimum time to wait between sending a Sleep flag and sending a Wakeup token
	SA_DELAY_EXEC_MAXIMUM					//!< maximum execution delay (for debugging)
} SA_Delay;


#define NO_HOST_ID					0xFF	//!< This value for a host id indicates that there is no host device in the system.
#define NO_DEVICE_ID					0xFF	//!< This value indicates that no device is registered at this device info array index.


//////////////////////////////////////////////////////////////////////
// structures for device discovery
/** \brief Device Information structure */
typedef struct
{
	uint8_t deviceId;							//!< device id indicates what CPU port and pin to use for communication
	uint8_t deviceAddress;					//!< used in Pause commands
	enum {SA_DeviceTypeUnknown,
			SA_DeviceTypeSA100,
			SA_DeviceTypeSA102,
			SA_DeviceTypeSA10HS}
	deviceType;									//!< what device this structure pertains to
	uint8_t isConfigured;
	uint8_t memValid;							//!< indicates whether key in SA100 is valid
	uint8_t serialNumber[6];				//!< serial number of device (first two bytes from ROM section; last four bytes from fuse section)
	uint8_t statusFuses[3];					//!< status fuses of device (three bytes)
} DiscoveryDeviceInfo_t;


// Function Prototypes
//
void SAP_Init(void);
void SAP_SetupRxTimer(void);
void SAP_Wakeup(void);

// Device Information Functions
//
uint8_t SAP_GetMaxDeviceId(void);
DiscoveryDeviceInfo_t *SAP_GetDeviceInfo(uint8_t deviceId);
uint8_t SAP_GetDeviceType(void);
int8_t SAP_SetClientDeviceID(uint8_t id);
int8_t SAP_GetClientDeviceID(uint8_t *id);
int8_t SAP_SetHostDeviceID(uint8_t id);
int8_t SAP_GetHostDeviceID(uint8_t *id);
int8_t SAP_SetDeviceID(uint8_t id);
int8_t SAP_GetDeviceID(uint8_t *id);


// Communication Functions (mostly inlined)
//
void SAP_RxMode(void) INLINE_ATTRIBUTE;
void SAP_TxMode(void) INLINE_ATTRIBUTE;
void SAP_SendOne(void) INLINE_ATTRIBUTE;
void SAP_SendZero(void) INLINE_ATTRIBUTE;
void SAP_WaitForXmitComplete(void) INLINE_ATTRIBUTE;
void SAP_ExecDelay(SA_Delay delay) INLINE_ATTRIBUTE;
void SAP_SetupIoTimeoutTimer(uint16_t timeout);
void SAP_EnableIoTimer(void) INLINE_ATTRIBUTE;
void SAP_DisableIoTimer(void) INLINE_ATTRIBUTE;
int8_t SAP_GetBit(uint8_t *bitData) INLINE_ATTRIBUTE;


#endif
