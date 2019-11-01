/********************************************************************
* Copyright (C) 2012-2013 Texas Instruments Incorporated.
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
 * @defgroup DFE_FL_AUTOCP_API AUTOCP
 * @ingroup DFE_FL_API
 */

/** @file dfe_fl_autocp.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_AUTOCP CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
/* =============================================================================
 * Revision History
 * ===============
 *
 *
 * =============================================================================
 */

/**
 * @defgroup DFE_FL_AUTOCP_DATASTRUCT DFE Autocp Data Structures
 * @ingroup DFE_FL_AUTOCP_API
 */

/**
 * @defgroup DFE_FL_AUTOCP_ENUM DFE Autocp Enumverated Data Types
 * @ingroup DFE_FL_AUTOCP_API
 */

/**
 * @defgroup DFE_FL_AUTOCP_FUNCTION DFE Autocp Functions
 * @ingroup DFE_FL_AUTOCP_API
 */
#ifndef _DFE_FL_AUTOCP_H_
#define _DFE_FL_AUTOCP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drv/dfe/dfe_fl.h>
#include <ti/csl/cslr_dfe_autocp.h>

/**
 * @addtogroup DFE_FL_AUTOCP_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
    /**
     * init
     */
    DFE_FL_AUTOCP_CMD_CFG_INITS,
    
        
    DFE_FL_AUTOCP_CMD_MAX_VALUE
} DfeFl_AutocpHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    DFE_FL_AUTOCP_QUERY_MAX_VALUE
} DfeFl_AutocpHwStatusQuery;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_AUTOCP_DATASTRUCT
 * @{
 */

/** @brief overlay register pointer to AUTOCP instance
 */
typedef CSL_DFE_AUTOCP_REGS *DfeFl_AutocpRegsOvly;

/** @brief AUTOCP Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle           hDfe;
    
    /// pointer to register base address of a AUTOCP instance
    DfeFl_AutocpRegsOvly   regs;
   
    /// This is the instance of AUTOCP being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_AutocpObj;

/** @brief handle pointer to AUTOCP object
 */
typedef DfeFl_AutocpObj *DfeFl_AutocpHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_AUTOCP_FUNCTION
 * @{
 */

DfeFl_AutocpHandle dfeFl_AutocpOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_AutocpObj            *pDfeAutocpObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_AutocpClose(DfeFl_AutocpHandle hDfeAutocp);

DfeFl_Status  dfeFl_AutocpHwControl
(
    DfeFl_AutocpHandle         hDfeAutocp,
    DfeFl_AutocpHwControlCmd   ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_AutocpGetHwStatus
(
    DfeFl_AutocpHandle         hDfeAutocp,
    DfeFl_AutocpHwStatusQuery  queryId,
    void                        *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_AUTOCP_H_ */
