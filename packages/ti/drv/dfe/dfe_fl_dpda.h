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
 *  @defgroup DFE_FL_DPDA_API DPDA
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_dpda.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_DPDA CSL
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
 * @defgroup DFE_FL_DPDA_DATASTRUCT DFE Dpda Data Structures
 * @ingroup DFE_FL_DPDA_API
 */

/**
 * @defgroup DFE_FL_DPDA_ENUM DFE Dpda Enumverated Data Types
 * @ingroup DFE_FL_DPDA_API
 */

/**
 * @defgroup DFE_FL_DPDA_FUNCTION DFE Dpda Functions
 * @ingroup DFE_FL_DPDA_API
 */

#ifndef _DFE_FL_DPDA_H_
#define _DFE_FL_DPDA_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/csl/cslr_dfe_dpda.h>

/**
 * @addtogroup DFE_FL_DPDA_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
	/// configure DPDA inits, including ssel, clk_gate, init_state and clear_data
    DFE_FL_DPDA_CMD_CFG_INITS,

    /// enable interrupt read complete
    DFE_FL_DPDA_CMD_ENB_INT_READ_COMPLETE_INTR,
    /// disable interrupt read complete
    DFE_FL_DPDA_CMD_DIS_INT_READ_COMPLETE_INTR,
    /// force set interrupt read complete
    DFE_FL_DPDA_CMD_SET_FORCE_INT_READ_COMPLETE_INTR,
    /// clear set interrupt read complete
    DFE_FL_DPDA_CMD_CLR_FORCE_INT_READ_COMPLETE_INTR,
    /// clear interrupt read complete status
    DFE_FL_DPDA_CMD_CLR_INT_READ_COMPLETE_INTR_STATUS,
    /// enable interrupt porcessed
    DFE_FL_DPDA_CMD_ENB_INT_PROCESSED_INTR,
    /// disable interrupt processed
    DFE_FL_DPDA_CMD_DIS_INT_PROCESSED_INTR,
    /// force set interrupt processed
    DFE_FL_DPDA_CMD_SET_FORCE_INT_PROCESSED_INTR,
    /// clear set interrupt processed
    DFE_FL_DPDA_CMD_CLR_FORCE_INT_PROCESSED_INTR,
    /// clear interrupt processed status
    DFE_FL_DPDA_CMD_CLR_INT_PROCESSED_INTR_STATUS,
    /// clear interrupt status for instruction generator going from RUNNING to IDLE
    DFE_FL_DPDA_CMD_CLR_IDLE_INTR_STATUS,

    /// set DSP interrupt master control
    DFE_FL_DPDA_CMD_SET_DSP_INTR,
    /// set DSP antenna enable
    DFE_FL_DPDA_CMD_SET_DSP_ANT_EN,
    /// set poly2lut scale
    DFE_FL_DPDA_CMD_SET_JACOB_P2LSCALE,
    /// set input scale
    DFE_FL_DPDA_CMD_SET_JACOB_INPUTSCALE,
    /// set offset exponent when converting custom floating point to integers
    DFE_FL_DPDA_CMD_SET_EXP_FP2I,
    /// set offset exponent when converting integers to custom floating point
    DFE_FL_DPDA_CMD_SET_EXP_I2FP,
    /// set interrupt address DPD
    DFE_FL_DPDA_CMD_SET_INTR_ADDRESS_DPD,
    /// set antenna enabled DPD
    DFE_FL_DPDA_CMD_SET_ANT_ENABLED_DPD,
    /// set new interrupt DPD
    DFE_FL_DPDA_CMD_SET_NEW_INT_DPD,
    /// set param1 DPD
    DFE_FL_DPDA_CMD_SET_PARAM1_DPD,
    /// set param2 DPD
    DFE_FL_DPDA_CMD_SET_PARAM2_DPD,
    /// set igreg memory
    DFE_FL_DPDA_CMD_SET_IGREG,
    /// set scalar memory
    DFE_FL_DPDA_CMD_SET_SCALAR,
    /// set lut master
    DFE_FL_DPDA_CMD_SET_LUT_MASTER,
    /// set basis lut memory
    DFE_FL_DPDA_CMD_SET_LUT,
    /// set iram memory
    DFE_FL_DPDA_CMD_SET_IRAM,
    /// set solution memory
    DFE_FL_DPDA_CMD_SET_PREG,
    DFE_FL_DPDA_CMD_MAX_VALUE
} DfeFl_DpdaHwControlCmd;

/** @brief query commands
 */
typedef enum
{
	/// get interrupt read complete status
	DFE_FL_DPDA_QUERY_GET_INT_READ_COMPLETE_INTR_STATUS,
	/// get interrupt processed status
	DFE_FL_DPDA_QUERY_GET_INT_PROCESSED_INTR_STATUS,
	/// get interrupt status for instruction generator going from RUNNING to IDLE
	DFE_FL_DPDA_QUERY_GET_IDLE_INTR_STATUS,
    /// get lut master
    DFE_FL_DPDA_QUERY_GET_LUT_MASTER,
    /// get basis lut memory
    DFE_FL_DPDA_QUERY_GET_LUT,
    /// get iram memory
    DFE_FL_DPDA_QUERY_GET_IRAM,
    /// get solution memory
    DFE_FL_DPDA_QUERY_GET_PREG,
    /// get solution memory
    DFE_FL_DPDA_QUERY_GET_SCALAR,
	DFE_FL_DPDA_QUERY_MAX_VALUE
} DfeFl_DpdaHwStatusQuery;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_DPDA_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_DPDA_CMD_SET_LUT_MASTER
 *      DFE_FL_DPDA_CMD_SET_IRAM
 */
typedef struct
{
    /// number of entries
	uint32_t numEntry;
	/// pointer to the data
    uint32_t *data;
} DfeFl_DpdaGeneric;

/** @brief argument for runtime control,
 *      DFE_FL_DPDA_CMD_SET_LUT
 */
typedef struct
{
    /// index id
	uint32_t idx;
	/// lut values
    DfeFl_DpdaGeneric lut;
} DfeFl_DpdaLut;

/** @brief argument for runtime control,
 *      DFE_FL_DPDA_CMD_SET_SCALAR
 */
typedef struct
{
	/// number of entries
	uint32_t numEntry;
	/// pointer to the iedata
    uint32_t *iedata;
    /// pointer to the qdata
    uint32_t *qdata;
} DfeFl_DpdaScalar;

/** @brief argument for runtime control,
 *      DFE_FL_DPDA_CMD_SET_PREG
 *      DFE_FL_DPDA_CMD_SET_SCALAR
 */
typedef struct
{
    /// index id
	uint32_t idx;
	/// number of entries
	uint32_t numEntry;
	/// pointer to the iedata
    uint32_t *iedata;
    /// pointer to the qdata
    uint32_t *qdata;
} DfeFl_DpdaPreg;

/** @brief argument for runtime control,
 *    DFE_FL_DPDA_CMD_ENB_INT_READ_COMPLETE_INTR,
 *    DFE_FL_DPDA_CMD_CLR_INT_READ_COMPLETE_INTR_STATUS
 *    DFE_FL_DPDA_CMD_CLR_INT_PROCESSED_INTR_STATUS
 *    DFE_FL_DPDA_CMD_ENB_INT_PROCESSED_INTR
 *    DFE_FL_DPDA_QUERY_GET_INT_READ_COMPLETE_INTR_STATUS
 */
typedef struct
{
	/// DPDA read complete status
	uint32_t readCompleteStatus;
	/// DPDA Interrupt processed status
	uint32_t intProcessedStatus;
} DfeFl_DpdaIntrStatus;

/** @brief overlay register pointer to DPDA instance
 */
typedef CSL_DFE_DPDA_REGS *DfeFl_DpdaRegsOvly;

/** @brief DPDA Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle           hDfe;
    
    /// pointer to register base address of a DPDA instance
    DfeFl_DpdaRegsOvly      regs;
   
    /// This is the instance of DPDA being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_DpdaObj;

/** @brief handle pointer to DPDA object
 */
typedef DfeFl_DpdaObj *DfeFl_DpdaHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_DPDA_FUNCTION
 * @{
 */

DfeFl_DpdaHandle dfeFl_DpdaOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_DpdaObj              *pDfeDpdaObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_DpdaClose(DfeFl_DpdaHandle hDfeDpda);

DfeFl_Status  dfeFl_DpdaHwControl
(
    DfeFl_DpdaHandle            hDfeDpda,
    DfeFl_DpdaHwControlCmd      ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_DpdaGetHwStatus
(
    DfeFl_DpdaHandle            hDfeDpda,
    DfeFl_DpdaHwStatusQuery     queryId,
    void                        *arg
);

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_DPDA_H_ */
