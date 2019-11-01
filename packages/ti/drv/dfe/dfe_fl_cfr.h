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
 *  @defgroup DFE_FL_CFR_API CFR
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_cfr.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_CFR CSL
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
 * @defgroup DFE_FL_CFR_DATASTRUCT DFE Cfr Data Structures
 * @ingroup DFE_FL_CFR_API
 */

/**
 * @defgroup DFE_FL_CFR_ENUM DFE Cfr Enumverated Data Types
 * @ingroup DFE_FL_CFR_API
 */

/**
 * @defgroup DFE_FL_CFR_FUNCTION DFE Cfr Functions
 * @ingroup DFE_FL_CFR_API
 */

#ifndef _DFE_FL_CFR_H_
#define _DFE_FL_CFR_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/csl/cslr_dfe_cfr.h>

/**
 * @addtogroup DFE_FL_CFR_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
	/// configure CFR inits, including ssel, clk_gate, init_state and clear_data
    DFE_FL_CFR_CMD_CFG_INITS,
    /// set preCFR gain ssel
    DFE_FL_CFR_CMD_SET_PREM_SSEL,
    /// set postCFR gain ssel
    DFE_FL_CFR_CMD_SET_POSTM_SSEL,
    /// set preCFR gain
    DFE_FL_CFR_CMD_SET_PREGAIN,
    /// set postCFR gain
    DFE_FL_CFR_CMD_SET_POSTGAIN,
    /// set PA protection gain
    DFE_FL_CFR_CMD_SET_PROTPAGAIN,

    /// enable PDC0 interrupt
    DFE_FL_CFR_CMD_ENB_PDC0_INTR,
    /// enable PDC1 interrupt
    DFE_FL_CFR_CMD_ENB_PDC1_INTR,
    /// enable AGC inout interrupt
    DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR,
    /// enable AGC sync interrupt
    DFE_FL_CFR_CMD_ENB_AGC_SYNC_INTR,
    /// enable DTH interrupt
    DFE_FL_CFR_CMD_ENB_DTH_INTR,
    /// disable PDC0 interrupt
    DFE_FL_CFR_CMD_DIS_PDC0_INTR,
    /// disable PDC1 interrupt
    DFE_FL_CFR_CMD_DIS_PDC1_INTR,
    /// disable AGC inout interrupt
    DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR,
    /// disable AGC sync interrupt
    DFE_FL_CFR_CMD_DIS_AGC_SYNC_INTR,
    /// disable DTH interrupt
    DFE_FL_CFR_CMD_DIS_DTH_INTR,
    /// force set PDC0 interrupt
    DFE_FL_CFR_CMD_SET_FORCE_PDC0_INTR,
    /// force set PDC1 interrupt
    DFE_FL_CFR_CMD_SET_FORCE_PDC1_INTR,
    /// force set AGC inout interrupt
    DFE_FL_CFR_CMD_SET_FORCE_AGC_INOUT_INTR,
    /// force set AGC sync interrupt
    DFE_FL_CFR_CMD_SET_FORCE_AGC_SYNC_INTR,
    /// force set DTH interrupt
    DFE_FL_CFR_CMD_SET_FORCE_DTH_INTR,
    /// clear set PDC0 interrupt
    DFE_FL_CFR_CMD_CLR_FORCE_PDC0_INTR,
    /// clear set  PDC1 interrupt
    DFE_FL_CFR_CMD_CLR_FORCE_PDC1_INTR,
    /// clear set AGC inout interrupt
    DFE_FL_CFR_CMD_CLR_FORCE_AGC_INOUT_INTR,
    /// clear set AGC sync interrupt
    DFE_FL_CFR_CMD_CLR_FORCE_AGC_SYNC_INTR,
    /// clear set DTH interrupt
    DFE_FL_CFR_CMD_CLR_FORCE_DTH_INTR,
    /// clear PDC0 interrupt status
    DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS,
    /// clear PDC1 interrupt status
    DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS,
    /// clear AGC inout interrupt status
    DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS,
    /// clear AGC sync interrupt status
    DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS,
    /// clear DTH interrupt status
    DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS,

    /// set ssel for updating LUT tables for the base filter
    DFE_FL_CFR_CMD_LUT_SSEL,
    /// set ssel for updating LUT tables for the half delayed filter
    DFE_FL_CFR_CMD_HDLY_SSEL,
    /// update the LUT table
    DFE_FL_CFR_CMD_UPDT_LUT_COEFF,
        
    DFE_FL_CFR_CMD_MAX_VALUE
} DfeFl_CfrHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /// get cfr busy status
	DFE_FL_CFR_QUERY_GET_BUSY_STATUS,
	/// get PDC0 interrupt status
    DFE_FL_CFR_QUERY_GET_PDC0_INTR_STATUS,
    /// get PDC1 interrupt status
    DFE_FL_CFR_QUERY_GET_PDC1_INTR_STATUS,
    /// get AGC inout interrupt status
    DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS,
    /// get AGC sync interrupt status
    DFE_FL_CFR_QUERY_GET_AGC_SYNC_INTR_STATUS,
    /// get DTH interrupt status
    DFE_FL_CFR_QUERY_GET_DTH_INTR_STATUS,

    DFE_FL_CFR_QUERY_MAX_VALUE
} DfeFl_CfrHwStatusQuery;


/** @brief cfr path
 */
typedef enum
{
	/// CFR path 0
    DFE_FL_CFR_PATH_0,
    /// CFR path 1
    DFE_FL_CFR_PATH_1,
    /// all CFR pathes
    DFE_FL_CFR_PATH_ALL = 0x03u
} DfeFl_CfrPath;

/** @brief cfr pdc interrupt type
 */
typedef enum
{
	/// PDC interrupt A0S0
	DFE_FL_CFR_CPAGE_A0S0 = 0,
	/// PDC interrupt A0S1
	DFE_FL_CFR_CPAGE_A0S1,
	/// PDC interrupt A1S0
	DFE_FL_CFR_CPAGE_A1S0,
	/// PDC interrupt A1S1
	DFE_FL_CFR_CPAGE_A1S1,
	/// PDC interrupt LUTS update done
	DFE_FL_CFR_LUTS_UPDT_DONE
} DfeFl_CfrPdcIntr;

/** @brief cfr agc inout interrupt type
 */
typedef enum
{
	/// AGC inout interrupt min in threshold
	DFE_FL_CFR_AGC_MIN_IN_TH = 0,
	/// AGC inout interrupt min out threshold
	DFE_FL_CFR_AGC_MIN_OUT_TH,
	/// AGC inout interrupt max in threshold
	DFE_FL_CFR_AGC_MAX_IN_TH,
	/// AGC inout interrupt max out threshold
	DFE_FL_CFR_AGC_MAX_OUT_TH
} DfeFl_CfrAgcInOutIntr;

/** @brief cfr agc sync interrupt type
 */
typedef enum
{
	/// AGC sync0 interrupt
	DFE_FL_CFR_AGC_SYNC0 = 0,
	/// AGC sync1 interrupt
	DFE_FL_CFR_AGC_SYNC1
} DfeFl_CfrAgcSyncIntr;

/** @brief cfr pdc interrupt type
 */
typedef enum
{
	/// DTH power change interrupt
	DFE_FL_CFR_DTH_PWR_CNG = 0,
	/// DTE sync0 interrupt
	DFE_FL_CFR_DTH_SYNC0
} DfeFl_CfrDthIntr;

typedef enum
{
	/// PDC0_I
	DFE_FL_CFR_PDC0_I = 0,
	/// PDC0_Q
	DFE_FL_CFR_PDC0_Q,
	/// PDC0_IQ
	DFE_FL_CFR_PDC0_IQ,
	/// PDC1_I
	DFE_FL_CFR_PDC1_I,
	/// PDC1_Q
	DFE_FL_CFR_PDC1_Q,
	/// PDC1_IQ
	DFE_FL_CFR_PDC1_IQ,
	/// PDC01_I
	DFE_FL_CFR_PDC01_I,
	/// PDC01_Q
	DFE_FL_CFR_PDC01_Q,
	/// PDC01_IQ
	DFE_FL_CFR_PDC01_IQ
} DfeFl_CfrLutLocation;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CFR_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_CFR_CMD_SET_PREM_SSEL
 *      DFE_FL_CFR_CMD_SET_POSTM_SSEL
 */
typedef struct
{
	/// CFR path
    uint32_t path;
    /// ssel value
    uint32_t ssel;
} DfeFl_CfrMultSsel;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_CMD_SET_PREGAIN
 *      DFE_FL_CFR_CMD_SET_POSTGAIN
 *      DFE_FL_CFR_CMD_SET_PROTPAGAIN
 */
typedef struct
{
    /// CFR path
	uint32_t path;
	/// gain value
    uint32_t gain;
} DfeFl_CfrMultGain;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_QUERY_GET_BUSY_STATUS
 */
typedef struct
{
    /// CFR path
	uint32_t path;
	/// busy status
    uint32_t status;
} DfeFl_CfrBusyStatus;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR
 */
typedef struct
{
	/// CFR path
	uint32_t path;
	/// AGC inout interrupt type
	DfeFl_CfrAgcInOutIntr intr;
} DfeFl_CfrAgcInOutIntrCfg;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_CMD_ENB_AGC_SYNC_INTR
 */
typedef struct
{
	/// CFR path
	uint32_t path;
	/// AGC sync interrupt type
	DfeFl_CfrAgcSyncIntr intr;
} DfeFl_CfrAgcSyncIntrCfg;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_CMD_ENB_DTH_INTR
 */
typedef struct
{
	/// CFR path
	uint32_t path;
	/// DTH interrupt type
	DfeFl_CfrDthIntr intr;
} DfeFl_CfrDthIntrCfg;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_QUERY_GET_PDC0_INTR_STATUS
 *      DFE_FL_CFR_QUERY_GET_PDC1_INTR_STATUS
 */
typedef struct
{
	/// PDC interrupt type
	DfeFl_CfrPdcIntr intrcfg;
	/// PDC interrupt status
	uint32_t status;
}DfeFl_CfrPdcIntrStatus;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS
 */
typedef struct
{
	/// AGC inout interrupt type
	DfeFl_CfrAgcInOutIntrCfg intrcfg;
	/// AGC interrupt status
	uint32_t status;
} DfeFl_CfrAgcInOutIntrStatus;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_QUERY_GET_AGC_SYNC_INTR_STATUS
 */
typedef struct
{
	/// AGC sync interrupt type
	DfeFl_CfrAgcSyncIntrCfg intrcfg;
	/// AGC sync interrupt status
	uint32_t status;
} DfeFl_CfrAgcSyncIntrStatus;

/** @brief argument for runtime control,
 *      DFE_FL_CFR_QUERY_GET_DTH_INTR_STATUS
 */
typedef struct
{
	/// DTH interrupt type
	DfeFl_CfrDthIntrCfg intrcfg;
	/// DTH interrupt status
	uint32_t status;
} DfeFl_CfrDthIntrStatus;

/** @brief argument for runtime control,
 *
 */
typedef struct
{
	/// PDC interrupt
	DfeFl_CfrPdcIntrStatus cfrPdc[2];
	/// AGC inout interrupt
	DfeFl_CfrAgcInOutIntrStatus cfrAgcInOut;
	/// AGC sync interrupt
	DfeFl_CfrAgcSyncIntrStatus cfrAgcSync;
	/// DTH interrupt type
	DfeFl_CfrDthIntrStatus cfrDth;
} DfeFl_CfrCfrIntrStatus;

/** @brief argument for runtime control,
 * 	    DFE_FL_CFR_CMD_UPDT_LUT_COEFF
 */
typedef struct
{
	/// CFR lut location
	DfeFl_CfrLutLocation location;
	/// number of coeffs
	uint32_t numCoeff;
	/// pointer to the lut coeffs
	uint32_t *coeff;
} DfeFl_CfrLutCoeffCfg;

/** @brief overlay register pointer to CFR instance
 */
typedef CSL_DFE_CFR_REGS *DfeFl_CfrRegsOvly;

/** @brief CFR Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle           hDfe;
    
    /// pointer to register base address of a CFR instance
    DfeFl_CfrRegsOvly      regs;
   
    /// This is the instance of CFR being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_CfrObj;

/** @brief handle pointer to CFR object
 */
typedef DfeFl_CfrObj *DfeFl_CfrHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_CFR_FUNCTION
 * @{
 */

DfeFl_CfrHandle dfeFl_CfrOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_CfrObj              *pDfeCfrObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_CfrClose(DfeFl_CfrHandle hDfeCfr);

DfeFl_Status  dfeFl_CfrHwControl
(
    DfeFl_CfrHandle            hDfeCfr,
    DfeFl_CfrHwControlCmd      ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_CfrGetHwStatus
(
    DfeFl_CfrHandle            hDfeCfr,
    DfeFl_CfrHwStatusQuery     queryId,
    void                        *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_CFR_H_ */
