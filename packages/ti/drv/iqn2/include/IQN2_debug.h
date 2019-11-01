/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** 
 * @file IQN2_debug.h
 *
 * @brief Header file for IQN2 H/W debug setup
 * 
*/


/** @addtogroup IQN2_FUNCTION  IQN2 Functions
 *  @{
 */


#ifndef __IQN2_DEBUG_H
#define __IQN2_DEBUG_H


#ifdef __cplusplus
extern "C" {
#endif

/** 
 *   @n@b IQN2_enableException
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to enable
 *   errors and alarms at IQN2 level.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *   
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies    
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_enableException(hIqn2);
     @endverbatim
 * 
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enableException(
		IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_disableException
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to disable
 *   errors and alarms at IQN2 level.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_disableException(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_disableException(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_enablePktdmaException
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to enable
 *   errors and alarms at IQN2 PKTDMA level.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_enablePktdmaException(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enablePktdmaException(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_disablePktdmaException
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to disable
 *   errors and alarms at IQN2 PKTDMA level.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_disablePktdmaException(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_disablePktdmaException(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_enableTopException
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to enable
 *   errors at alarms at IQN2 TOP level for EV0 or EV1. It should
 *   be noted that PktDMA starvation error exceptions are only
 *   available on PKTDMA_STARVE interrupt.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_enableTopException(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enableTopException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_disableTopException
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to disable
 *   errors at alarms at IQN2 TOP level for EV0 or EV1. It should
 *   be noted that PktDMA starvation error exceptions are not
 *   disabled by this API. IQN2_disablePktdmaException() exists for
 *   that matter.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_disableTopException(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_disableTopException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_enableAt2Exception
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to enable
 *   errors at alarms at IQN2 AT2 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_enableAt2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enableAt2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_disableAt2Exception
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to disable
 *   errors at alarms at IQN2 AT2 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_disableAt2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_disableAt2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_enableIqs2Exception
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to enable
 *   errors at alarms at IQN2 IQS2 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_enableIqs2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enableIqs2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_disableIqs2Exception
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to disable
 *   errors at alarms at IQN2 IQS2 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_disableIqs2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_disableIqs2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_enableAid2Exception
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to enable
 *   errors at alarms at IQN2 AID2 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_enableAid2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enableAid2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_disableAid2Exception
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to disable
 *   errors at alarms at IQN2 AID2 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_disableAid2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_disableAid2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_enableDio2Exception
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to enable
 *   errors at alarms at IQN2 DIO2 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_enableDio2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enableDio2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_disableDio2Exception
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to disable
 *   errors at alarms at IQN2 DIO2 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_disableDio2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_disableDio2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_enableAilException
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to enable
 *   errors at alarms at IQN2 AIL0/AIL1 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_enableAilException(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enableAilException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_disableAilException
 *
 *   @b Description
 *   @n This function configures IQN2 HW registers to disable
 *   errors at alarms at IQN2 AIL0/AIL1 level for EV0 or EV1.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_disableAilException(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_disableAilException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_getException
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 level. Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_getException(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getException(
		IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_getPsrException
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 TOP/PSR level.Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_getPsrException(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getPsrException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_getPktDMAException
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 TOP/PktDMA level.Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount. It should be noted that PktDMA starvation error
 *   exceptions are only available on PKTDMA_STARVE interrupt.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_getPktDMAException(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getPktDMAException(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_getAt2Exception
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 AT2 level.Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_getAt2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getAt2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_getIqs2Exception
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 IQS2 level.Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_getIqs2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getIqs2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_getAid2Exception
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 AID2 level.Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_getAid2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getAid2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_getDfeException
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 AID2/DFE level.Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_getDfeException(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getDfeException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_getDio2Exception
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 DIO2 level.Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            IQN2_getDio2Exception(hIqn2,interruptEventNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getDio2Exception(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum
);

/**
 *   @n@b IQN2_getAilException
 *
 *   @b Description
 *   @n This function checks for IQN2 HW errors at alarms at IQN2 AIL0 or 1 level.Exceptions are then
 *   accumulated in hIqn2->iqn2EeCount.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2                Pointer to a IQN2_ConfigObj instance.
       interruptEventNum    Interrupt number, 0 or 1.
       ailNum               AIL instance number, 0 or 1.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  H/W state
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            uint32_t             interruptEventNum
            Iqn2Fl_AilInstance   ailNum
            IQN2_getAilException(hIqn2,interruptEventNum,ailNum);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_getAilException(
        IQN2_ConfigHandle  hIqn2,
        uint32_t           interruptEventNum,
        Iqn2Fl_AilInstance ailNum
);

/**
 *   @n@b IQN2_printException
 *
 *   @b Description
 *   @n This function prints out IQN2 HW errors at alarms at IQN2 level.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_printException(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_printException(
		IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_printStatus
 *
 *   @b Description
 *   @n This function calls IQN2_getException and IQN2_printException.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_printStatus(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_printStatus(
		IQN2_ConfigHandle  hIqn2
);


/**
 *   @n@b IQN2_resetException
 *
 *   @b Description
 *   @n This function resets IQN2LLD exception counters.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  None.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n  SW counters
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_resetException(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_resetException(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_captureException
 *
 *   @b Description
 *   @n This function captures the exception counts into a supplied destination storage
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
       capturePtr   Pointer to the IQN2_EeCountObj structure to capture the exception counters
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is configured and started. Exceptions were enabled.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle  hIqn2;
            IQN2_EeCountObj      capture;
            IQN2_captureException(hIqn2, &capture);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_captureException (
        IQN2_ConfigHandle  hIqn2,
        IQN2_EeCountObj *capturePtr
);

/**
 *   @n@b IQN2_enableAilDataTrace
 *
 *   @b Description
 *   @n This function enables IQN2 AIL data trace.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
       hDataTrace  Pointer to a IQN2_AilDataTraceObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  The IQN2 is started and AIL0-1 traffic is up.
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_AilDataTraceHandle hDataTrace;
            IQN2_enableAilDataTrace(hIqn2, hDataTrace);
     @endverbatim
 *
 */
#ifndef __IQN2_DEBUG_C
extern
#endif
void IQN2_enableAilDataTrace(
        IQN2_ConfigHandle        hIqn2,
        IQN2_AilDataTraceHandle  hDataTrace
);


/**
 *   @n@b IQN2_disableDataTrace
 *
 *   @b Description
 *   @n This function disables IQN2 AIL data trace.
 *
 *   @b Arguments
 *   @verbatim
       hIqn2        Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ConfigHandle    hIqn2;
            IQN2_disableDataTrace(hIqn2);
     @endverbatim
 *
 */
#ifndef __AIF_DEBUG_C
extern
#endif
void IQN2_disableDataTrace(
        IQN2_ConfigHandle        hIqn2
);



#ifdef __cplusplus
}
#endif





#endif //__IQN2_DEBUG_H

/** @} */ // end of module additions
