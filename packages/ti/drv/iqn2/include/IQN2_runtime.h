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
 * @file IQN2_runtime.h
 *
 * @brief Header file for IQN2 H/W runtime operations
 * 
*/


/** @addtogroup IQN2_FUNCTION  IQN2 Functions
 *  @{
 */


#ifndef __IQN2_RUNTIME_H
#define __IQN2_RUNTIME_H

#ifdef __cplusplus
extern "C" {
#endif
/** 
 *   @n@b IQN2_at2GetCapturedBCN
 *
 *   @b Description
 *   @n This function returns the AT2 BCN captured value. Usage: a sync event from one of the sync source was detected. BCN value was captured
 *   for use by APP SW to calculate and correct RADT alignment.
 *
 *   @b Arguments
 *   @verbatim
		hIqn2           Pointer to a IQN2_ConfigObj instance.
		hCapturedValue  Pointer for the return captured value (1)
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
            uint32_t  capturedValue;
            IQN2_at2GetCapturedBCN(hIqn2,&capturedValue);
     @endverbatim
 * 
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_at2GetCapturedBCN(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValue
);


/**
 *   @n@b IQN2_at2AdjustBCNOffset
 *
 *   @b Description
 *   @n This function returns the AT2 BCN captured value. Usage: a sync event from one of the sync source was detected. BCN value was captured
 *   for use by APP SW to calculate and correct RADT alignment.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValue    Pointer for the offset value (1)
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
            uint32_t  offset;
            IQN2_at2AdjustBCNOffset(hIqn2,&offset);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_at2AdjustBCNOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValue
);

/**
 *   @n@b IQN2_ailUatGetCapturedBCN
 *
 *   @b Description
 *   @n This function returns the AIL UAT BCN captured value. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured values (2)
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
            uint32_t  capturedValue[2];
            IQN2_ailUatGetCapturedBCN(hIqn2,&capturedValue[0]);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailUatGetCapturedBCN(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
);

/**
 *   @n@b IQN2_ailUatAdjustBCNOffset
 *
 *   @b Description
 *   @n This function adjuts the AIL UAT BCN offset value. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (2)
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
            uint32_t  offsetValues[2];
            IQN2_ailUatAdjustBCNOffset(hIqn2,&offsetValues[0]);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailUatAdjustBCNOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
);

/**
 *   @n@b IQN2_ailUatGetCapturedEgrRADTs
 *
 *   @b Description
 *   @n This function returns the AIL UAT Egress RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured value (2*8)
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
            uint32_t  capturedValues[2][8];
            IQN2_ailUatGetCapturedEgrRADTs(hIqn2,capturedValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailUatGetCapturedEgrRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t**         hCapturedValues
);

/**
 *   @n@b IQN2_ailUatAdjustEgrRADTsOffset
 *
 *   @b Description
 *   @n This function sets the AIL UAT Egress RADT offset values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (2*8)
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
            uint32_t  offsetValues[2][8];
            IQN2_ailUatAdjustEgrRADTsOffset(hIqn2,offsetValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailUatAdjustEgrRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t**         hOffsetValues
);

/**
 *   @n@b IQN2_ailUatGetCapturedIngRADTs
 *
 *   @b Description
 *   @n This function returns the AIL UAT Ingress RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured value (2*8)
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
            uint32_t  capturedValues[2][8];
            IQN2_ailUatGetCapturedIngRADTs(hIqn2,capturedValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailUatGetCapturedIngRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t**         hCapturedValues
);

/**
 *   @n@b IQN2_ailUatAdjustIngRADTsOffset
 *
 *   @b Description
 *   @n This function sets the AIL UAT Ingress RADT offset values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (2*8)
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
            uint32_t  offsetValues[2][8];
            IQN2_ailUatAdjustIngRADTsOffset(hIqn2,offsetValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailUatAdjustIngRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t**         hOffsetValues
);

/**
 *   @n@b IQN2_aid2UatGetCapturedEgrRADTs
 *
 *   @b Description
 *   @n This function returns the AID2 UAT Egress RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured value (8)
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
            uint32_t  capturedValues[8];
            IQN2_aid2UatGetCapturedEgrRADTs(hIqn2,capturedValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2UatGetCapturedEgrRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
);

/**
 *   @n@b IQN2_aid2UatAdjustEgrRADTsOffset
 *
 *   @b Description
 *   @n This function sets the AID2 UAT Egress RADT offset values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (8)
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
            uint32_t  offsetValues[8];
            IQN2_aid2UatAdjustEgrRADTsOffset(hIqn2,offsetValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2UatAdjustEgrRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
);

/**
 *   @n@b IQN2_aid2UatGetCapturedIngRADTs
 *
 *   @b Description
 *   @n This function returns the AID2 UAT Ingress RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured value (8)
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
            uint32_t  capturedValues[8];
            IQN2_aid2UatGetCapturedIngRADTs(hIqn2,capturedValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2UatGetCapturedIngRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*         hCapturedValues
);

/**
 *   @n@b IQN2_aid2UatAdjustIngRADTsOffset
 *
 *   @b Description
 *   @n This function sets the AID2 UAT Ingress RADT offset values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (8)
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
            uint32_t  offsetValues[8];
            IQN2_aid2UatAdjustIngRADTsOffset(hIqn2,offsetValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2UatAdjustIngRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
);


/**
 *   @n@b IQN2_dio2UatGetCapturedEgrRADTs
 *
 *   @b Description
 *   @n This function returns the DIO2 UAT Egress RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured value (8)
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
            uint32_t  capturedValues[8];
            IQN2_dio2UatGetCapturedEgrRADTs(hIqn2,capturedValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2UatGetCapturedEgrRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
);

/**
 *   @n@b IQN2_dio2UatAdjustEgrRADTsOffset
 *
 *   @b Description
 *   @n This function sets the DIO2 UAT Egress RADT offset values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (8)
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
            uint32_t  offsetValues[8];
            IQN2_dio2UatAdjustEgrRADTsOffset(hIqn2,offsetValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2UatAdjustEgrRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
);

/**
 *   @n@b IQN2_dio2UatGetCapturedIngRADTs
 *
 *   @b Description
 *   @n This function returns the AID2 UAT Ingress RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured value (8)
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
            uint32_t  capturedValues[8];
            IQN2_dio2UatGetCapturedIngRADTs(hIqn2,capturedValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2UatGetCapturedIngRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*         hCapturedValues
);

/**
 *   @n@b IQN2_dio2UatAdjustIngRADTsOffset
 *
 *   @b Description
 *   @n This function sets the DIO2 UAT Ingress RADT offset values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (8)
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
            uint32_t  offsetValues[8];
            IQN2_dio2UatAdjustIngRADTsOffset(hIqn2,offsetValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2UatAdjustIngRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
);

/**
 *   @n@b IQN2_dio2UatGetCapturedEgrEngineRADTs
 *
 *   @b Description
 *   @n This function returns the DIO2 UAT Egress DIO engine RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured value (3)
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
            uint32_t  capturedValues[3];
            IQN2_dio2UatGetCapturedEgrEngineRADTs(hIqn2,capturedValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2UatGetCapturedEgrEngineRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hCapturedValues
);

/**
 *   @n@b IQN2_dio2UatAdjustEgrEngineRADTsOffset
 *
 *   @b Description
 *   @n This function sets the DIO2 UAT Egress DIO engine RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (3)
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
            uint32_t  offsetValues[3];
            IQN2_dio2UatAdjustEgrEngineRADTsOffset(hIqn2,offsetValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2UatAdjustEgrEngineRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
);

/**
 *   @n@b IQN2_dio2UatGetCapturedIngEngineRADTs
 *
 *   @b Description
 *   @n This function returns the AID2 UAT Ingress DIO engine RADT captured values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hCapturedValue  Pointer for the return captured value (3)
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
            uint32_t  capturedValues[3];
            IQN2_dio2UatGetCapturedIngEngineRADTs(hIqn2,capturedValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2UatGetCapturedIngEngineRADTs(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*         hCapturedValues
);

/**
 *   @n@b IQN2_dio2UatAdjustIngEngineRADTsOffset
 *
 *   @b Description
 *   @n This function sets the DIO2 UAT Ingress DIO engine RADT offset values. Usage: Micro AT Re-synchronization procedure.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        hOffsetValues   Pointer for the offset values (3)
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
            uint32_t  offsetValues[3];
            IQN2_dio2UatAdjustIngEngineRADTsOffset(hIqn2,offsetValues);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2UatAdjustIngEngineRADTsOffset(
        IQN2_ConfigHandle  hIqn2,
        uint32_t*          hOffsetValues
);


/**
 *   @n@b IQN2_dio2DisableEngines
 *
 *   @b Description
 *   @n This function disables all DIO engines currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
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
            IQN2_dio2DisableEngines(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2DisableEngines(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_dio2EnableEngines
 *
 *   @b Description
 *   @n This function enables all DIO engines in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
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
            IQN2_dio2EnableEngines(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2EnableEngines(
        IQN2_ConfigHandle  hIqn2
);


/**
 *   @n@b IQN2_dio2DisableIngressChannels
 *
 *   @b Description
 *   @n This function disables all DIO Efe/Ingress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_dio2DisableIngressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2DisableIngressChannels(
        IQN2_ConfigHandle  hIqn2
);


/**
 *   @n@b IQN2_dio2EnableIngressChannels
 *
 *   @b Description
 *   @n This function enables all DIO Efe/Ingress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_dio2EnableIngressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2EnableIngressChannels(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_dio2DisableEgressChannels
 *
 *   @b Description
 *   @n This function disables all DIO Ife/Egress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_dio2DisableEgressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2DisableEgressChannels(
        IQN2_ConfigHandle  hIqn2
);


/**
 *   @n@b IQN2_dio2EnableEgressChannels
 *
 *   @b Description
 *   @n This function enables all DIO Ife/Egress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n IQN2 HW started
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_dio2EnableEgressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_dio2EnableEgressChannels(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_ailDisableIngressChannels
 *
 *   @b Description
 *   @n This function disables all AIL Ife/Ingress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ailDisableIngressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailDisableIngressChannels(
        IQN2_ConfigHandle  hIqn2
);


/**
 *   @n@b IQN2_ailEnableIngressChannels
 *
 *   @b Description
 *   @n This function enables all AIL Ife/Ingress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ailEnableIngressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailEnableIngressChannels(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_ailDisableEgressChannels
 *
 *   @b Description
 *   @n This function disables all AIL Efe/Egress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ailDisableEgressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailDisableEgressChannels(
        IQN2_ConfigHandle  hIqn2
);


/**
 *   @n@b IQN2_ailEnableEgressChannels
 *
 *   @b Description
 *   @n This function enables all AIL Efe/Egress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n IQN2 HW started
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_ailEnableEgressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_ailEnableEgressChannels(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_aid2DisableIngressChannels
 *
 *   @b Description
 *   @n This function disables all AID2 Ife/Ingress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_aid2DisableIngressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2DisableIngressChannels(
        IQN2_ConfigHandle  hIqn2
);


/**
 *   @n@b IQN2_aid2EnableIngressChannels
 *
 *   @b Description
 *   @n This function enables all AID2 Ife/Ingress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_aid2EnableIngressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2EnableIngressChannels(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_aid2DisableEgressChannels
 *
 *   @b Description
 *   @n This function disables all AID2 Efe/Egress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_aid2DisableEgressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2DisableEgressChannels(
        IQN2_ConfigHandle  hIqn2
);


/**
 *   @n@b IQN2_aid2EnableEgressChannels
 *
 *   @b Description
 *   @n This function enables all AID2 Efe/Egress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_aid2EnableEgressChannels(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2EnableEgressChannels(
        IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_disableIngressChannel
 *
 *   @b Description
 *   @n This function disables one of Ife/Ingress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        index           Channel index.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_disableIngressChannel(hIqn2, index);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_disableIngressChannel(
        IQN2_ConfigHandle  hIqn2,
        uint32_t index
);


/**
 *   @n@b IQN2_enableIngressChannel
 *
 *   @b Description
 *   @n This function enables one of the Ife/Ingress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        index           Channel index.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_enableIngressChannel(hIqn2,index);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_enableIngressChannel(
        IQN2_ConfigHandle  hIqn2,
        uint32_t index
);

/**
 *   @n@b IQN2_disableEgressChannel
 *
 *   @b Description
 *   @n This function disables one of the Efe/Egress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        index           Channel index.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_disableEgressChannel(hIqn2,index);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_disableEgressChannel(
        IQN2_ConfigHandle  hIqn2,
        uint32_t index
);


/**
 *   @n@b IQN2_enableEgressChannel
 *
 *   @b Description
 *   @n This function enables one of the Efe/Egress channels currently in use by LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        index           Channel index.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n IQN2 HW started
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_enableEgressChannel(hIqn2, index);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_enableEgressChannel(
        IQN2_ConfigHandle  hIqn2,
        uint32_t index
);


/**
 *   @n@b IQN2_iqs2EgressAid2Reconfigure
 *
 *   @b Description
 *   @n This function is used to reconfigure the mapping between PKTDMA or DIO channels and AID channels.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        dma_channel           Pktdma or Dio Channel index.
        aid_channel		aid channel
        pktDmaOrDio		select pktDma (0) or Dio engine (1) as the source.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n IQN2 HW started
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_iqs2EgressAid2Reconfigure(hIqn2, index, aid_channel, pktDmaOrDio);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_iqs2EgressAid2Reconfigure(
        IQN2_ConfigHandle  hIqn2,
        uint32_t dma_channel,
        uint32_t aid_channel
);

/**
 *   @n@b IQN2_iqs2IngressAid2Reconfigure
 *
 *   @b Description
 *   @n This function is used to reconfigure the mapping between PKTDMA or DIO channels and AID channels.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        dma_channel     Pktdma or Dio Channel index.
        aid_channel		aid channel
        pktDmaOrDio		select pktDma (0) or Dio engine (1) as the destination.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n IQN2 HW started
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_iqs2IngressAid2Reconfigure(hIqn2, index, aid_channel, pktDmaOrDio);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_iqs2IngressAid2Reconfigure(
        IQN2_ConfigHandle  hIqn2,
        uint32_t dma_channel,
        uint32_t aid_channel
);

/**
 *   @n@b IQN2_aid2EfeTdmEnable
 *
 *   @b Description
 *   @n This function is used to enable Radio standards TDM LUT.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        radStdIndex     radio standard index
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n IQN2 HW started
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_aid2EfeTdmEnable(hIqn2, radStdIndex);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2EfeTdmEnable(
		IQN2_ConfigHandle  hIqn2,
		uint32_t radStdIndex
);

/**
 *   @n@b IQN2_aid2EfeTdmReconfigureLength
 *
 *   @b Description
 *   @n This function is used to reconfigure the legnth of a radio standard TDM LUT.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        radStdIndex     radio standard index
        len				Lenght of TDM LUT
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n IQN2 HW started
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_aid2EfeTdmReconfigureLength(hIqn2, radStdIndex, len);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2EfeTdmReconfigureLength(
		IQN2_ConfigHandle  hIqn2,
		uint32_t radStdIndex,
		uint32_t len
);

/**
 *   @n@b IQN2_aid2EfeTdmDisableAll
 *
 *   @b Description
 *   @n This function is used to disable all the radio standard TDM LUT.
 *
 *   @b Arguments
 *   @verbatim
 *       hIqn2           Pointer to a IQN2_ConfigObj instance.
 *   @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n
 *
 *   <b> Post Condition </b>
 *   @n IQN2 HW started
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_aid2EfeTdmDisableAll(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_aid2EfeTdmDisableAll(
		IQN2_ConfigHandle  hIqn2
);

/**
 *   @n@b IQN2_enableAt2Event
 *
 *   @b Description
 *   @n This function enables a given AT2 event, assuming this event was already initialized using IQN2_initAt2Event() function.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        EventSelect     Event number to enable.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            uint32_t EventSelect;
            IQN2_enableAt2Event(hIqn2,EventSelect);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_enableAt2Event(
        IQN2_ConfigHandle    hIqn2,
        uint32_t             EventSelect
);

/**
 *   @n@b IQN2_disableAt2Event
 *
 *   @b Description
 *   @n This function disables a given AT2 event.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        EventSelect     Event number to disable.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            uint32_t EventSelect;
            IQN2_disableAt2Event(hIqn2,EventSelect);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_disableAt2Event(
        IQN2_ConfigHandle    hIqn2,
        uint32_t             EventSelect
);

/**
 *   @n@b IQN2_disableRadioTimers
 *
 *   @b Description
 *   @n This function disables all 8 radio timers.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_disableRadioTimers(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_disableRadioTimers(
        IQN2_ConfigHandle    hIqn2
);


/**
 *   @n@b IQN2_enableRadioTimers
 *
 *   @b Description
 *   @n This function enables all configured radio timers via LLD.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            IQN2_enableRadioTimers(hIqn2);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_enableRadioTimers(
        IQN2_ConfigHandle    hIqn2
);

/**
 *   @n@b IQN2_resyncProcedureReset
 *
 *   @b Description
 *   @n This function resets static variables used by the resync procedure. It is required to call this function if the EE
 *   counters are reset.
 *
 *   @b Arguments
 *   @verbatim
     @endverbatim
 *
 *   <b> Return Value </b>  none
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
            IQN2_resyncProcedureReset();
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
void
IQN2_resyncProcedureReset(
);

/**
 *   @n@b IQN2_resyncProcedure
 *
 *   @b Description
 *   @n This function SW readjusts BCN offset or offset comparators for RADTs and uATs. This function is to be used
 *   when a new sync arrives while the BCN and RADT timers are already counting. An important note is that all radio
 *   timers are disabled by this API call, and that the re-enabling of the radio timers needs to be done by the
 *   customer application using the IQN2_enableRadioTimers() API.
 *
 *   @b Arguments
 *   @verbatim
        hIqn2           Pointer to a IQN2_ConfigObj instance.
        delay           Extra positive or negative delay to add while adjusting AT and uAT timers.
     @endverbatim
 *
 *   <b> Return Value </b>  returns whether external sync was detected and processed (1 or 0)
 *
 *   <b> Pre Condition </b>
 *   @n IQN2 HW started
 *
 *   <b> Post Condition </b>
 *   @n
 *
 *   @b Modifies
 *   @n
 *
 *   @b Example
 *   @verbatim
            int32_t delay;
            IQN2_resyncProcedure(hIqn2,delay);
     @endverbatim
 *
 */
#ifndef __IQN2_RUNTIME_C
extern
#endif
uint32_t
IQN2_resyncProcedure(
        IQN2_ConfigHandle hIqn2,
        int32_t           delay
);

#ifdef __cplusplus
}
#endif


#endif //__IQN2_RUNTIME_H

/** @} */ // end of module additions
