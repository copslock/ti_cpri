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
 * @defgroup DFE_FL_API DFE FL
 */
/** @file dfe_fl.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */

/**
 * @defgroup DFE_FL_DATASTRUCT DFE Data Structures
 * @ingroup DFE_FL_API
 */

/**
 * @defgroup DFE_FL_SYMBOL DFE Symbols Defined
 * @ingroup DFE_FL_API
 */

/**
 * @defgroup DFE_FL_FUNCTION DFE Functions
 * @ingroup DFE_FL_API
 */


#ifndef _DFE_FL_H_
#define _DFE_FL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
//#include <ti/csl/csl.h>
#include <ti/csl/cslr.h>

/**
 * @addtogroup DFE_FL_SYMBOL
 * @{
 */

/**************************************************************************
* Peripheral Instance counts
**************************************************************************/
/// total BB blocks
#define DFE_FL_BB_PER_CNT          1
/// total DDUC blocks
#define DFE_FL_DDUC_PER_CNT        4
/// total SUMMER blocks
#define DFE_FL_SUMMER_PER_CNT      1
/// total AUTOCP blocks
#define DFE_FL_AUTOCP_PER_CNT      1
/// total CFR blocks
#define DFE_FL_CFR_PER_CNT         2
/// total CDFR blocks
#define DFE_FL_CDFR_PER_CNT        1
/// total DPD blocks
#define DFE_FL_DPD_PER_CNT         1
/// total DPDA blocks
#define DFE_FL_DPDA_PER_CNT        1
/// total TX blocks
#define DFE_FL_TX_PER_CNT          1
/// total RX blocks
#define DFE_FL_RX_PER_CNT          1
/// total CB blocks
#define DFE_FL_CB_PER_CNT          1
/// total JESD blocks
#define DFE_FL_JESD_PER_CNT        1
/// total FB blocks
#define DFE_FL_FB_PER_CNT          1
/// total MISC blocks
#define DFE_FL_MISC_PER_CNT        1

/**************************************************************************\
* Peripheral Instance definitions.
\**************************************************************************/

/** @brief Instance number of BB */
#define DFE_FL_BB_0                0

/** @brief Instance number of DDUC0 */
#define DFE_FL_DDUC_0              0
/** @brief Instance number of DDUC1 */
#define DFE_FL_DDUC_1              1
/** @brief Instance number of DDUC2 */
#define DFE_FL_DDUC_2              2
/** @brief Instance number of DDUC3 */
#define DFE_FL_DDUC_3              3

/** @brief Instance number of SUMMER */
#define DFE_FL_SUMMER_0            0

/** @brief Instance number of AUTOCP */
#define DFE_FL_AUTOCP_0            0

/** @brief Instance number of CFR0 */
#define DFE_FL_CFR_0               0
/** @brief Instance number of CFR1 */
#define DFE_FL_CFR_1               1

/** @brief Instance number of CDFR */
#define DFE_FL_CDFR_0              0

/** @brief Instance number of DPD */
#define DFE_FL_DPD_0               0

/** @brief Instance number of DPDA */
#define DFE_FL_DPDA_0              0

/** @brief Instance number of TX */
#define DFE_FL_TX_0                0

/** @brief Instance number of RX */
#define DFE_FL_RX_0                0

/** @brief Instance number of CB */
#define DFE_FL_CB_0                0

/** @brief Instance number of JESD */
#define DFE_FL_JESD_0              0

/** @brief Instance number of FB */
#define DFE_FL_FB_0                0

/** @brief Instance number of MISC */
#define DFE_FL_MISC_0              0

/**************************************************************************\
* Peripheral Offset within DFE
\**************************************************************************/

/** @brief offset of BB instance */
#define DFE_FL_BB_0_OFFSET         0x0000000

/** @brief offset of DDUC1 instance */
#define DFE_FL_DDUC_0_OFFSET       0x0100000
/** @brief offset of DDUC2 instance */
#define DFE_FL_DDUC_1_OFFSET       0x0180000
/** @brief offset of DDUC3 instance */
#define DFE_FL_DDUC_2_OFFSET       0x0200000
/** @brief offset of DDUC4 instance */
#define DFE_FL_DDUC_3_OFFSET       0x0280000

/** @brief offset of SUMMEr instance */
#define DFE_FL_SUMMER_0_OFFSET     0x0900000

/** @brief offset of CFR0 instance */
#define DFE_FL_CFR_0_OFFSET        0x0b00000
/** @brief offset of CFR1 instance */
#define DFE_FL_CFR_1_OFFSET        0x0b80000

/** @brief offset of SUMMEr instance */
#define DFE_FL_AUTOCP_0_OFFSET     0x0f00000

/** @brief offset of CDFR instance */
#define DFE_FL_CDFR_0_OFFSET       0x1000000

/** @brief offset of DPD instance */
#define DFE_FL_DPD_0_OFFSET        0x1300000

/** @brief offset of DPDA instance */
#define DFE_FL_DPDA_0_OFFSET       0x1500000

/** @brief offset of TX instance */
#define DFE_FL_TX_0_OFFSET         0x1600000

/** @brief offset of RX instance */
#define DFE_FL_RX_0_OFFSET         0x1a00000

/** @brief offset of CB instance */
#define DFE_FL_CB_0_OFFSET         0x1c00000

/** @brief offset of JESD instance */
#define DFE_FL_JESD_0_OFFSET       0x1d00000

/** @brief offset of FB instance */
#define DFE_FL_FB_0_OFFSET         0x1e80000

/** @brief offset of MISC instance */
#define DFE_FL_MISC_0_OFFSET       0x1f00000

/**************************************************************************
* DFE error code (no more than 32 codes)
**************************************************************************/
//#define EDFE_FL_SYNC_TIMEOUT       (CSL_EDFE_FIRST - 0)

#ifndef NULL
#define NULL            ((void*)0)
#endif

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_DATASTRUCT
 * @{
 */

typedef int16_t           DfeFl_InstNum;


/**
 *
 * @brief dfe fl return and error codes
 *
 *  */
typedef enum
{
	/** Sync timeout */
	DFE_FL_SYNC_TIMEOUT = -10,
    /** Action not supported */
    DFE_FL_NOTSUPPORTED = -9,
    /** Invalid query */
    DFE_FL_INVQUERY = -8,
    /** Invalid command */
    DFE_FL_INVCMD = -7,
    /** Invalid parameters */
    DFE_FL_INVPARAMS = -6,
    /** Handle passed to DFE FL was invalid */
    DFE_FL_BADHANDLE = -5,
    /** Encoutered DFE FL system resource overflow */
    DFE_FL_OVFL = -4,
    /** Unused code */
    DFE_FL_UNUSED = -3,
    /** DFE FL Peripheral resource is already in use */
    DFE_FL_INUSE = -2,
    /** DFE FL Generic Failure */
    DFE_FL_FAIL = -1,
    /** DFE FL successful return code */
    DFE_FL_SOK = 1
} DfeFl_Status;

/**************************************************************************\
* DFE top level structs
\**************************************************************************/
typedef volatile uint32_t * DfeFl_RegsOvly;

/**
 * @brief Sub-block inits config
 */
typedef struct {
    uint32_t ssel;
    uint32_t initClkGate;
    uint32_t initState;
    uint32_t clearData;
} DfeFl_SublkInitsConfig;

/**
 * @brief clock gate
 */
typedef struct
{
    uint32_t timeStep;
    uint32_t resetInterval;
    uint32_t tddPeriod;
    uint32_t tddOn0;
    uint32_t tddOff0;
    uint32_t tddOn1;
    uint32_t tddOff1;
} DfeFl_ClockGateConfig;
 
/**
 * @brief Module specific context information. 
 */
typedef struct {
    /** 
     *  The below declaration is just a place-holder for future implementation.
     */
    uint32_t contextInfo;
} DfeFl_Context;

/** @brief Module specific parameters. 
 */
typedef struct {
    /** Bit mask to be used for module specific parameters. The below
     *  declaration is just a place-holder for future implementation.
     */
    uint32_t flags;
} DfeFl_Param;

/** @brief This structure contains the base-address information for the
 *         peripheral instance
 */
typedef struct {
    /** Base-address of the configuration registers of the peripheral
     */
    DfeFl_RegsOvly regs;
} DfeFl_BaseAddress;

/**
 * @brief  DFE object structure.
 */
typedef struct {
    /** Pointer to the register overlay structure of the DFE */
    DfeFl_RegsOvly regs;

    /** Instance of DFE being referred by this object  */
    DfeFl_InstNum dfeNum;
} DfeFl_Obj;


/**
 * @brief This data type is used to return the handle to the CSL of DFE
 */
typedef DfeFl_Obj *DfeFl_Handle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_FUNCTION
 * @{
 */

/**************************************************************************\
* DFE top level APIs
\**************************************************************************/
extern DfeFl_Status dfeFl_Init (
    DfeFl_Context *pContext
);

extern DfeFl_Handle dfeFl_Open (
    DfeFl_Obj   *pDfeObj,
    DfeFl_InstNum  dfeNum,
    DfeFl_Param *pDfeParam,
    uint32_t       dfeBaseAddr,
    DfeFl_Status   *status

);

extern DfeFl_Status dfeFl_GetBaseAddress (
    DfeFl_InstNum        dfeNum,
    DfeFl_Param       *pDfeParam,
    DfeFl_BaseAddress *pBaseAddress
);

extern DfeFl_Status dfeFl_GetPID (
    DfeFl_Handle hDfe,
    uint32_t *pid
);

extern DfeFl_Status dfeFl_Close (
    DfeFl_Handle hDfe
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_H_ */
