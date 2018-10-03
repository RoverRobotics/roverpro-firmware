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
 *  \brief 	status / return code declarations
 *  \date 	Sept 16, 2009
*/
#ifndef SA_STATUS_H
#define SA_STATUS_H

#include <stdint.h> // C99 standard typedefs

#define SA_SUCCESS		(int8_t) 0x00 //!< Function succeeded. Device status, if available, was okay.

#define SA_GENFAIL		(int8_t) 0xD0 //!< unspecified error
#define SA_BADPARM		(int8_t) 0xD1 //!< bad argument (out of range, null pointer, etc.)
#define SA_NOTALLOWED	(int8_t) 0xD2 //!< response status byte indicates error

#define SA_FUNCFAIL		(int8_t) 0xE0 //!< Function could not execute due to incorrect condition / state.

#define SA_COMMFAIL		(int8_t) 0xF0 //!< Communication with device failed.
#define SA_TIMEOUT		(int8_t) 0xF1 //!< Timed out while waiting for response.
#define SA_BADSIZE		(int8_t) 0xF2 //!< Size value in response was different than expected.
#define SA_HWFAIL		   (int8_t) 0xF3 //!< Hardware failure, for instance setting up a timer or port.
#define SA_NODEVICES    (int8_t) 0xF4 //!< No devices were found during library initialization.
#define SA_INVALID_SIZE	(int8_t) 0xF5 //!< Could not copy response because receive buffer was too small.
#define SA_BAD_CRC		(int8_t) 0xF6 //!< Incorrect CRC received.


#endif
