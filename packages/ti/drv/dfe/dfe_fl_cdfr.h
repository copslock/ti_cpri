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
 *  @defgroup DFE_FL_CDFR_API CDFR
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_cdfr.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_CDFR CSL
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
 * @defgroup DFE_FL_CDFR_DATASTRUCT DFE Cdfr Data Structures
 * @ingroup DFE_FL_CDFR_API
 */

/**
 * @defgroup DFE_FL_CDFR_ENUM DFE Cdfr Enumverated Data Types
 * @ingroup DFE_FL_CDFR_API
 */

/**
 * @defgroup DFE_FL_CDFR_FUNCTION DFE Cdfr Functions
 * @ingroup DFE_FL_CDFR_API
 */

#ifndef _DFE_FL_CDFR_H_
#define _DFE_FL_CDFR_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/csl/cslr_dfe_cdfr.h>

/**
 * @addtogroup DFE_FL_CDFR_ENUM
 * @{
 */


/** @brief control commands
 */
typedef enum
{
    /**
     * init
     */
    DFE_FL_CDFR_CMD_CFG_INITS,
        
        
    DFE_FL_CDFR_CMD_MAX_VALUE
} DfeFl_CdfrHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    DFE_FL_CDFR_QUERY_MAX_VALUE
} DfeFl_CdfrHwStatusQuery;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CDFR_DATASTRUCT
 * @{
 */

/** @brief overlay register pointer to CDFR instance
 */
typedef CSL_DFE_CDFR_REGS *DfeFl_CdfrRegsOvly;

/** @brief CDFR Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle           hDfe;
    
    /// pointer to register base address of a CDFR instance
    DfeFl_CdfrRegsOvly      regs;
   
    /// This is the instance of CDFR being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_CdfrObj;

/** @brief handle pointer to CDFR object
 */
typedef DfeFl_CdfrObj *DfeFl_CdfrHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CDFR_FUNCTION
 * @{
 */

DfeFl_CdfrHandle dfeFl_CdfrOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_CdfrObj              *pDfeCdfrObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_CdfrClose(DfeFl_CdfrHandle hDfeCdfr);

DfeFl_Status  dfeFl_CdfrHwControl
(
    DfeFl_CdfrHandle            hDfeCdfr,
    DfeFl_CdfrHwControlCmd      ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_CdfrGetHwStatus
(
    DfeFl_CdfrHandle            hDfeCdfr,
    DfeFl_CdfrHwStatusQuery     queryId,
    void                        *arg
);

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_CDFR_H_ */
