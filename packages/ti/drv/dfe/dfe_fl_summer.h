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
 *  @defgroup DFE_FL_SUMMER_API SUMMER
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_summer.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_SUMMER CSL
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
 * @defgroup DFE_FL_SUMMER_DATASTRUCT DFE Summer Data Structures
 * @ingroup DFE_FL_SUMMER_API
 */

/**
 * @defgroup DFE_FL_SUMMER_ENUM DFE Summer Enumverated Data Types
 * @ingroup DFE_FL_SUMMER_API
 */

/**
 * @defgroup DFE_FL_SUMMER_FUNCTION DFE Summer Functions
 * @ingroup DFE_FL_SUMMER_API
 */

#ifndef _DFE_FL_SUMMER_H_
#define _DFE_FL_SUMMER_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/csl/cslr_dfe_summer.h>

/**
 * @addtogroup DFE_FL_SUMMER_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
	/// set shift value
	DFE_FL_SUMMER_CMD_SET_SHIFT = 0,
	/// set summer mapping selection
    DFE_FL_SUMMER_CMD_SET_SEL,
    /// configure summer inits, including ssel, init_state and clear_data
	DFE_FL_SUMMER_CMD_CFG_INITS,
    /// set one or two streams on cfr
    DFE_FL_SUMMER_CMD_SET_NUMANT,
    /// set summer ssel
    DFE_FL_SUMMER_CMD_SET_SUMMER_SSEL,
        
    DFE_FL_SUMMER_CMD_MAX_VALUE
} DfeFl_SummerHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /// get summer shift value
	DFE_FL_SUMMER_QUERY_GET_SHIFT = 0,
	/// get summer mapping selection
    DFE_FL_SUMMER_QUERY_GET_SEL,
    /// get summer inits
    DFE_FL_SUMMER_QUERY_GET_INITS,
    /// get one or two streams on cfr
    DFE_FL_SUMMER_QUERY_GET_NUMANT,
    /// get summer ssel
    DFE_FL_SUMMER_QUERY_GET_SUMMER_SSEL,

    DFE_FL_SUMMER_QUERY_MAX_VALUE
} DfeFl_SummerHwStatusQuery;

/** @brief cfr string */
typedef enum
{
    /// cfr0 stream0
	DFE_FL_SUMMER_CFR0_STR0 = 0 ,
	/// cfr0 stream1
    DFE_FL_SUMMER_CFR0_STR1,
    /// cfr1 stream0
    DFE_FL_SUMMER_CFR1_STR0,
    /// cfr1 stream1
    DFE_FL_SUMMER_CFR1_STR1,
    /// cfr2 stream0
    DFE_FL_SUMMER_CFR2_STR0,
    /// cfr2 stream1
    DFE_FL_SUMMER_CFR2_STR1,
    /// cfr3 stream0
    DFE_FL_SUMMER_CFR3_STR0,
    /// cfr3 stream1
    DFE_FL_SUMMER_CFR3_STR1
} DfeFl_SummerCfrStr;


/** @brief dduc */
typedef enum
{
    /// dduc0
	DFE_FL_SUMMER_DDUC0 = 0 ,
	/// dduc1
    DFE_FL_SUMMER_DDUC1,
    /// dduc2
    DFE_FL_SUMMER_DDUC2,
    /// dduc3
    DFE_FL_SUMMER_DDUC3
} DfeFl_SummerDduc;

/** @brief dduc port */
typedef enum
{
    /// dduc port0
	DFE_FL_SUMMER_PORT0 = 0 ,
	/// dduc port1
    DFE_FL_SUMMER_PORT1,
    /// dduc port2
    DFE_FL_SUMMER_PORT2
} DfeFl_SummerDducPort;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_SUMMER_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_SUMMER_CMD_SET_SHIFT
 */
typedef struct
{
	/// cfr stream index
	DfeFl_SummerCfrStr idx;
	/// data
    uint32_t data;
} DfeFl_SummerShiftCfg;

/** @brief argument for runtime control,
 *      DFE_FL_SUMMER_CMD_SET_SEL
 */
typedef struct
{
	/// cfr stream index
	DfeFl_SummerCfrStr idx;
	/// dduc index
	DfeFl_SummerDduc iDduc;
	/// dduc port index
	DfeFl_SummerDducPort iPort;
	/// data
    uint32_t data;
} DfeFl_SummerSelCfg;

/** @brief overlay register pointer to SUMMER instance
 */
typedef CSL_DFE_SUMMER_REGS *DfeFl_SummerRegsOvly;

/** @brief SUMMER Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle           hDfe;
    
    /// pointer to register base address of a Summer instance
    DfeFl_SummerRegsOvly   regs;
   
    /// This is the instance of Summer being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_SummerObj;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_SUMMER_FUNCTION
 * @{
 */

/** @brief handle pointer to SUMMER object
 */
typedef DfeFl_SummerObj *DfeFl_SummerHandle;

DfeFl_SummerHandle dfeFl_SummerOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_SummerObj            *pDfeSummerObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_SummerClose(DfeFl_SummerHandle hDfeSummer);

DfeFl_Status  dfeFl_SummerHwControl
(
    DfeFl_SummerHandle         hDfeSummer,
    DfeFl_SummerHwControlCmd   ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_SummerGetHwStatus
(
    DfeFl_SummerHandle         hDfeSummer,
    DfeFl_SummerHwStatusQuery  queryId,
    void                        *arg
);

/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_SUMMER_H_ */
