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
/**	\file SA_API.h
*	\brief This file holds the definitions and prototypes for the API Layer.
*
*	The API lets an application do complex functions with a
*	single call. It is written in ANSI C for portability. It is possible in
*	a resource constrained environment to dispense with the layer and
*	call the Functional Layer directly, or to move the API layer off-chip.
*	Users doing either of the above should use the layer to understand
*	what to call in what order.
*	\date 	May 5, 2009
*/
#ifndef SA_API_H
#define SA_API_H

#include <stdint.h>	// C99 standard typedefs
#include "SA_Func.h"

/*
* typedefs
*/

//! SA_Read selection values
typedef enum { getSN, getStatus, getMfrID, getRevNum, getMemValid} readSelect;

//! firmware version
typedef struct _version_t
{
	uint8_t		major;
	uint8_t		minor;
	uint8_t		release;
} version_t;

//! GenPersonalizationKey input values
typedef struct _genPersKeyIn
{
	uint8_t		KeyData[32];
	uint8_t		GPKFixed[8];
	uint8_t		GPKSeed[16];
	uint8_t		GPKLength[8];
} genPersKeyIn;

#define SZ_PERSKEY	sizeof(((genPersKeyIn *)0)->KeyData)
#define SZ_GPKFIXED	sizeof(((genPersKeyIn *)0)->GPKFixed)
#define SZ_GPKSEED	sizeof(((genPersKeyIn *)0)->GPKSeed)
#define SZ_GPKLENGTH	sizeof(((genPersKeyIn *)0)->GPKLength)


/**
 *  function prototypes
 */

int8_t SA_DoMAC(uint8_t Mode, uint16_t KeyID, uint8_t *Challenge, uint8_t *Response);
int8_t SA_MatchMAC(uint8_t Mode, uint16_t KeyID, uint8_t *Challenge, uint8_t *ExpectedMAC, int8_t *Success);
int8_t SA_GenMac(uint16_t KeyID, uint8_t *Key, uint8_t *Challenge, uint8_t Mode, uint8_t *Secret, uint8_t *MAC);
int8_t SA_GenMac100(uint8_t *Key, uint8_t *Challenge, uint8_t Mode, uint8_t *MAC);
int8_t SA_Read(readSelect Select, uint8_t *Buffer);
int8_t SA_Burn(uint8_t FuseNumber);
int8_t SA_GenPersonalizationKey(uint8_t *Map, uint8_t *PersonalizationKey, uint8_t* Seed, uint8_t *EncryptedMap);
int8_t SA_BurnSecure(int8_t Decrypt, uint8_t *Map, uint16_t PersKeyID, uint8_t *Seed);
int8_t SA_DisableBurnSecure(void);
int8_t SA_GenPersonalizationKey100(uint8_t *Key, uint8_t *PersonalizationKey, uint8_t *Seed, uint8_t *EncryptedKey);
int8_t SA_LoadSram(uint8_t *Key, uint16_t PersKeyID, uint8_t *Seed);
int8_t SA_PauseLong(uint8_t ChipSelect);
int8_t SA_Init(uint8_t initOptions);
int8_t SA_HostCommand(uint8_t Overwrite, uint8_t *Challenge, Host1OtherInfo_str *OtherInfo, uint8_t *Response);

version_t *SA_GetLibraryVersion(void);

#endif
