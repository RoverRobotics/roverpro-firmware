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
  * \brief This file contains template functions for the physical part of the 
  *  communication layer.
  * \date  6/3/2009
  * first implementation
  *
  * \date   7/30/2009
  * changed to a generic template
  *
  * \date 01/25/2010
  * adjusted to current AVR implementation
  */

#include <stdint.h>

#include "SA_Status.h"
#include "SA_Phys.h"


//! This macro defines the maximum device id. This template assumes one client and one host device.
#define MAX_DEVICE_ID  (1)

//! declaration of the variable indicating which device to talk to
uint8_t deviceID;

//! device id for client currently in use
uint8_t clientDeviceId;

//! device id for host currently in use
uint8_t hostDeviceId;

//! array of device information structures
DiscoveryDeviceInfo_t devices[MAX_DEVICE_ID + 1];


/** \brief This function gets the maximum device id in the system / number of devices minus one.
 *
 * \return maximum device id (size of device information array minus one)
 */
uint8_t SAP_GetMaxDeviceId(void)
{
   return MAX_DEVICE_ID;
}


/** \brief This function gets the device information of the chosen device.
 *
 * \param[in] deviceId index into the "devices" array
 * \return pointer to device information structure
 */
DiscoveryDeviceInfo_t *SAP_GetDeviceInfo(uint8_t deviceId)
{
   if (deviceId > MAX_DEVICE_ID)
      return (void *) 0;

   return &devices[deviceId];
}


/** \brief This function gets the type of the currently selected client.
 *
 * \return device type
 */
uint8_t SAP_GetDeviceType(void)
{
   if (deviceID == NO_DEVICE_ID)
      return SA_DeviceTypeUnknown;

   return devices[deviceID].deviceType;
}


/** \brief This function sets the client device id.
 *
 *  \param[in]  id identifier for client device
 *  \return     the status of the operation
 */
int8_t SAP_SetClientDeviceID(uint8_t id)
{
   if (id > MAX_DEVICE_ID && id != NO_DEVICE_ID)
      return SA_BADPARM;

   clientDeviceId = id;

   return SA_SUCCESS;
}


/** \brief This function gets the client device id.
 *
 * \param[out] id pointer to id for currently selected client device
 * @return status of the operation
 */
int8_t SAP_GetClientDeviceID(uint8_t *id)
{
   *id = clientDeviceId;
   return (clientDeviceId > MAX_DEVICE_ID) ? SA_GENFAIL : SA_SUCCESS;
}


/** \brief This function sets the host device id.
 *
 * @param[in] id identifier for host device
 * @return status of the operation
 */
int8_t SAP_SetHostDeviceID(uint8_t id)
{
   if (id > MAX_DEVICE_ID && id != NO_HOST_ID)
      return SA_BADPARM;

   hostDeviceId = id;

   return SA_SUCCESS;
}


/** \brief This function gets the host device id.
 *
 *  \param[out] id pointer to id for host device
 *  \return status of the operation
 */
int8_t SAP_GetHostDeviceID(uint8_t *id)
{
   *id = hostDeviceId;
   return (hostDeviceId > MAX_DEVICE_ID) ? SA_GENFAIL : SA_SUCCESS;
}


/** \brief This function sets the device id. Communication functions will use this device id.
 *
 *  \param[in] id device id
 *  \return status of the operation
 ****************************************************************
 */
int8_t SAP_SetDeviceID(uint8_t id)
{
   if (id > MAX_DEVICE_ID)
      return SA_BADPARM;

   deviceID = id;

   return SA_SUCCESS;
}


/** \brief This function gets the device id currently in use.
 *
 *  \param[out] id pointer to device identifier
 *  \return status of the operation
 */
int8_t SAP_GetDeviceID(uint8_t *id)
{
   *id = deviceID;
   return (deviceID > MAX_DEVICE_ID) ? SA_GENFAIL : SA_SUCCESS;
}


/** \brief This function configures signal pin as output.
 *  \return void
 */
void SAP_TxMode(void)
{
}


/** \brief This function configures signal pin as input without pull-up.
 *  \return void
 */
void SAP_RxMode(void)
{
}

/** \brief This function initializes hardware-dependent part of communication layer.
 *
 *  Initializes communication time-out timer and puts the physical layer into
 *  transmit mode.
 *
 *  \return void
 */
void SAP_Init(void)
{

}

/** \brief This function puts a "one" bit on the signal wire.
 *  \return void
 */
void SAP_SendOne(void)
{
}


/** \brief This function puts a "zero" bit on the signal wire.
 *  \return void
 */
void SAP_SendZero(void)
{
}


/** \brief  This function waits until transmission is complete.
 *
 *    For a bit-banging version this will be only a dummy function that is
 *    called from the comm layer. For the UART version, this function
 *    has to check the tx empty flag.
 *
 *  \return void
 */
void SAP_WaitForXmitComplete(void)
{
}


/** \brief This function reads a bit from the signal wire.
 *
 * \param[out] bitData byte pointer to bit value
 * \return status (SA_SUCCESS or SA_TIMEOUT)
 */
int8_t SAP_GetBit(uint8_t *bitData)
{
  return SA_FUNCFAIL;
}


/** \brief This function wakes up CA device.
 *
 *  Sends a wake-up sequence over the signal wire.
 *  \return void
 */
void SAP_Wakeup(void)
{
}


/** \brief This function generates an execution delay.
 *
 * This is needed to keep AVR library from pulling in about 5 kB of
 * floating point library for delays using run-time parameters.
 *
 *  \param[in] delay enumerated delay
 *  \return void
 */
void SAP_ExecDelay(SA_Delay delay)
{
}


/** \brief This function initializes timer for IO timeout detection.

	* This function is not needed if loop counters are used 
   * instead of timer interrupts.
 *  \return void
 */
void SAP_SetupIOTimeoutTimer(void)
{
}


/** \brief This function enables communication IO timeout timer.

	* This function is not needed if loop counters are used 
   * instead of timer interrupts.
 *  \return void
 */
void SAP_EnableIoTimer(void)
{
}

/** \brief This function disables communication IO timeout timer.

	* This function is not needed if loop counters are used 
   * instead of timer interrupts.
 *  \return void
 */
void SAP_DisableIoTimer(void)
{
}


/** \brief This function handles communication time-out and sets time-out flag.
 *
 *  This function is called from a timer ISR and is not needed if 
 *  loop counters are used instead of timer interrupts.
 *  \return void
 */
void SAP_TimeoutHandler(void)
{
}
