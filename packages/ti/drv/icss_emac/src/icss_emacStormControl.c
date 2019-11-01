/**
* @file icss_emacStormControl.c
*
* @brief Storm prevention implementation
* @details      Contains functions for implementing a storm prevention scheme for
*               the ICSS based ethernet switch. This is done through a credit based
*               scheme where only a particular number of packets are allowed in a
*               given amount of time, this is specified through a credit which expires
*               at the end of a time period.
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


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */


#include <ti/drv/icss_emac/icss_emacDrv.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/**Storm prevention control information*/
/*stormPrevention controlBMcast[2];*/

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ICSS_EmacInitStormPreventionTable(uint8_t portnum, ICSS_EmacHandle icssEmacHandle) {

    stormPrevention_t* stormPrevPtr;

    if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EmacObject*)icssEmacHandle->object)->emacInitcfg->portMask) {
        portnum = 1U;
    }

    stormPrevPtr = (stormPrevention_t*)(((ICSS_EmacObject*)icssEmacHandle->object)->stormPrevPtr);
    stormPrevPtr += (portnum - 1U);

    setCreditValue(DEFAULT_CREDITS, stormPrevPtr);
    ICSS_EmacEnableStormPrevention(portnum, icssEmacHandle);
    
    /* Reset credit value during init */
    ICSS_EmacResetStormPreventionCounter(icssEmacHandle);
}

void setCreditValue(uint16_t creditValue, stormPrevention_t* stormPrevPtr) {

    stormPrevPtr->credits = creditValue;
}

void ICSS_EmacDisableStormPrevention(uint8_t portnum, ICSS_EmacHandle icssEmacHandle) {
    uint16_t *controlPointer;
    stormPrevention_t* stormPrevPtr;
    ICSS_EmacFwStaticMmap *pStaticMMap = (&((ICSS_EmacObject*)icssEmacHandle->object)->fwStaticMMap);

    uint32_t temp_addr = 0U;

    if (portnum <= (uint8_t)ICSS_EMAC_PORT_2) 
    {
        if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EmacObject*)icssEmacHandle->object)->emacInitcfg->portMask) {
            portnum = 1U;
        }

        stormPrevPtr = (stormPrevention_t*)(((ICSS_EmacObject*)icssEmacHandle->object)->stormPrevPtr);
        stormPrevPtr += (portnum - 1U);

        stormPrevPtr->suppressionEnabled = 0;

        if(portnum == (uint8_t)ICSS_EMAC_PORT_1) {
            temp_addr = ((((ICSS_EmacHwAttrs*)icssEmacHandle->hwAttrs)->emacBaseAddrCfg)->dataRam0BaseAddr + pStaticMMap->stormPreventionOffset);
            controlPointer = (uint16_t*)(temp_addr);
        } else {
            temp_addr = ((((ICSS_EmacHwAttrs*)icssEmacHandle->hwAttrs)->emacBaseAddrCfg)->dataRam1BaseAddr + pStaticMMap->stormPreventionOffset);
            controlPointer = (uint16_t*)(temp_addr);
        }
        *(controlPointer) = stormPrevPtr->suppressionEnabled;
    }
    return;
}

void ICSS_EmacEnableStormPrevention(uint8_t portnum, ICSS_EmacHandle icssEmacHandle) {

    uint16_t *controlPointer;
    stormPrevention_t* stormPrevPtr;
    uint32_t temp_addr = 0U;
    ICSS_EmacFwStaticMmap *pStaticMMap = (&((ICSS_EmacObject*)icssEmacHandle->object)->fwStaticMMap);


    if (portnum <= (uint8_t)ICSS_EMAC_PORT_2) 
    {
        if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EmacObject*)icssEmacHandle->object)->emacInitcfg->portMask) {
            portnum = 1u;
        }

        /*get the pointer to the correct port*/
        stormPrevPtr = (stormPrevention_t*)(((ICSS_EmacObject*)icssEmacHandle->object)->stormPrevPtr);
        stormPrevPtr += (portnum - 1u);

        stormPrevPtr->suppressionEnabled = 1;

        if(portnum == (uint8_t)ICSS_EMAC_PORT_1) {
            temp_addr = ((((ICSS_EmacHwAttrs*)icssEmacHandle->hwAttrs)->emacBaseAddrCfg)->dataRam0BaseAddr + pStaticMMap->stormPreventionOffset);
            controlPointer = (uint16_t*)(temp_addr);
        } else {
            temp_addr = ((((ICSS_EmacHwAttrs*)icssEmacHandle->hwAttrs)->emacBaseAddrCfg)->dataRam1BaseAddr +  pStaticMMap->stormPreventionOffset);
            controlPointer = (uint16_t*)(temp_addr);
        }
        *(controlPointer) = stormPrevPtr->suppressionEnabled;
    }
    return;
}

void ICSS_EmacResetStormPreventionCounter(ICSS_EmacHandle icssEmacHandle){
    uint32_t *controlPointer;
    uint32_t controlWord;

    uint32_t temp_addr = 0U;
    uint32_t temp_var1 = 0u;
    stormPrevention_t* stormPrevPtr = (stormPrevention_t*)(((ICSS_EmacObject*)icssEmacHandle->object)->stormPrevPtr);
    ICSS_EmacFwStaticMmap *pStaticMMap = (&((ICSS_EmacObject*)icssEmacHandle->object)->fwStaticMMap);

    if(stormPrevPtr->suppressionEnabled) {
        temp_addr = ((((ICSS_EmacHwAttrs*)icssEmacHandle->hwAttrs)->emacBaseAddrCfg)->dataRam0BaseAddr +  pStaticMMap->stormPreventionOffset);
        controlPointer = (uint32_t*)(temp_addr);
        temp_var1 = (((uint32_t)(stormPrevPtr->suppressionEnabled)) | ((uint32_t)(((uint32_t)(stormPrevPtr->credits)) << 8U)));
        controlWord = ((uint32_t)(temp_var1));
        *(controlPointer) = controlWord;
    }

    if(ICSS_EMAC_MODE_SWITCH == ((ICSS_EmacObject*)icssEmacHandle->object)->emacInitcfg->portMask) 
    {
        /*Access next port member*/
        stormPrevPtr += 1;

        if(stormPrevPtr->suppressionEnabled) {
            temp_addr = ((((ICSS_EmacHwAttrs*)icssEmacHandle->hwAttrs)->emacBaseAddrCfg)->dataRam1BaseAddr +  pStaticMMap->stormPreventionOffset);
            controlPointer = (uint32_t*)(temp_addr);
            temp_var1 = (((uint32_t)(stormPrevPtr->suppressionEnabled)) | ((uint32_t)(((uint32_t)(stormPrevPtr->credits)) << 8U)));
            controlWord = ((uint32_t)(temp_var1));
            *(controlPointer) = controlWord;
        }
    }
    return;
}
/**
* @brief   returns true if storm prevention is enabled
*
* @param stormPrevPtr Pointer to Storm Prevention member instance for that port
*
* @retval Status. 1 if storm prevention is enabled on any port else 0
*/
inline uint8_t ifStormPreventionEnabled(const stormPrevention_t* stormPrevPtr); /* for misra warning */ 
inline uint8_t ifStormPreventionEnabled(const stormPrevention_t* stormPrevPtr) 
{
    return stormPrevPtr->suppressionEnabled;
}

