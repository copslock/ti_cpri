/**
 *   @file  icss_emacStormControl.h
 *
 *   @brief
 *      Include file for storm prevention implementation
 *
 */


/* Copyright (C) {2016} Texas Instruments Incorporated - http://www.ti.com/ 
*
*   Redistribution and use in source and binary forms, with or without 
*   modification, are permitted provided that the following conditions 
*   are met:
*
*     Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*
*     Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the   
*     distribution.
*
*     Neither the name of Texas Instruments Incorporated nor the names of
*     its contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ICSS_EMAC_STORMCONTROL__H
#define ICSS_EMAC_STORMCONTROL__H

#include <stdlib.h>

#include <stdint.h>

#include "ti/drv/icss_emac/icss_emacDrv_Def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* @def DEFAULT_CREDITS
*      (packets per microsec) This is 1% of the bandwidth
*/
#define DEFAULT_CREDITS   (2000U)
/**
* @def DEFAULT_PKT_SIZE
*      default packet size used to compute number of BC/MC packets allowed in a 100ms window
*/
#define DEFAULT_PKT_SIZE  60
/**
* @def MAX_PERCENTAGE
*      Max number of BC/MC packets blocked expressed as a percentage
*/
#define MAX_PERCENTAGE    50


/**
 * @brief Storm prevention control variables
 */
typedef struct {
    /** enable/disable storm prevention*/
    uint16_t suppressionEnabled;
    /** Number of packets allowed in a time interval*/
    uint16_t credits;
} stormPrevention_t;

/**
* @brief Initialize Storm Prevention
* @param portnum Port Number
* @param icssEmacHandle Pointer to ICSS EMAC Handle
* @retval none
*/
void ICSS_EmacInitStormPreventionTable(uint8_t portnum, ICSS_EmacHandle icssEmacHandle);
/**
* @brief Disable Storm Prevention for a particular port
* @param portnum  Port number for which Storm Prevention must be disabled
* @param icssEmacHandle Pointer to ICSS EMAC Handle
* @retval none
*/
void ICSS_EmacDisableStormPrevention(uint8_t portnum, ICSS_EmacHandle icssEmacHandle);

/**
* @brief   Set the credit value for broadcast packets used in storm prevention
*
* @param creditValue number of packets allowed in a given time
* @param stormPrevPtr Pointer to Storm Prevention member instance for that port
*
* @retval none
*/
void setCreditValue(uint16_t creditValue, stormPrevention_t* stormPrevPtr);

/**
* @brief Enable Storm Prevention for a particular port
* @param portnum  Port number for which Storm Prevention must be disabled
* @param icssEmacHandle Pointer to ICSS EMAC Handle
* @retval none
*/
void ICSS_EmacEnableStormPrevention(uint8_t portnum, ICSS_EmacHandle icssEmacHandle);


/**
* @brief   Reset the credits at the end of time period, this time period is common to both broadcast and multicast packets
* @param icssEmacHandle Pointer to ICSS EMAC Handle
* @retval none
*/
void ICSS_EmacResetStormPreventionCounter(ICSS_EmacHandle icssEmacHandle);


#ifdef __cplusplus
}
#endif

#endif /* _ICSS_EMAC_STORMCONTROL_H_ */
