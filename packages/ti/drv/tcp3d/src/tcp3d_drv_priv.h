/*
 *
 * Copyright (C) 2010, 2014 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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



#ifndef _TCP3D_DRV_PRIV_H_
#define _TCP3D_DRV_PRIV_H_

/**
 *  Local Compile Flags
 */
#define TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE    0

/**
 * Local includes
 */
#include <tcp3d_drv_types.h>
#include <ti/drv/tcp3d/src/tcp3d_utils.h>

#include <ti/csl/soc.h>
#include <ti/csl/cslr_tpcc.h>
#include <ti/csl/csl_cpIntcAux.h>

#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
#include <ti/csl/csl_edma3.h>
#else
#include <ti/csl/cslr_tpcc.h>
#endif

/**
 *  Local Defines
 */
#define LINK_CH_IDX_INCFG           0
#define LINK_CH_IDX_LLR             1
#define LINK_CH_IDX_HD              2
#define LINK_CH_IDX_STS             3
#define LINK_CH_IDX_SD              4

#define LINK_CH_IDX_LAST            LINK_CH_IDX_SD

#define LINK_CH_IDX_PAUSE           (LINK_CH_IDX_LAST + 1)
#define LINK_CH_IDX_REVT            (LINK_CH_IDX_LAST + 2)
#define LINK_CH_IDX_L2P             (LINK_CH_IDX_LAST + 3)
#define LINK_CH_IDX_NTF             (LINK_CH_IDX_LAST + 4)
#define LINK_CH_IDX_NTFD            (LINK_CH_IDX_LAST + 5)
#define LINK_CH_IDX_WRAP            (LINK_CH_IDX_LAST + 6)
#define LINK_CH_IDX_NEXTCB_DUMMY    (LINK_CH_IDX_LAST + 7)
#define LINK_CH_IDX_NEXTCB_NTFD     (LINK_CH_IDX_LAST + 8)

#define ONE_OVER_LINK_CB_Q15        (32768/TCP3D_DRV_LINK_CB)

/**
 *  Local Macros
 */
/**
 * @brief   Macro for getting the global map for L2 memory addresses 
 */
#define L2GLBMAP(coreID, addr)  \
    ( ( ((uint32_t)(addr) >= 0x00800000) && ((uint32_t)(addr) < 0x00900000) ) ? \
      ( (uint32_t)(addr) | (uint32_t)((0x10 | (coreID & 0x3)) << 24) ) : \
      (uint32_t)(addr) )

/**
 * @brief   Macro for getting the code block index using division by TCP3D_DRV_LINK_CB 
 */
#define GET_CB_IDX(input)   ((_smpy((input), ONE_OVER_LINK_CB_Q15) + 32768)>>16)

/**
 *  Local Structures
 */
/**
 * @brief   Structure for keeping the local variables
 */
typedef struct Tcp3d_Internal
{
    uint8_t             constantOne;/**< variable set to 1 at init time and
                                        used by PAUSE channels */
    Tcp3d_State         pauseState; /**< variable set to TCP3D_DRV_STATE_PAUSE
                                        and used by PAUSE channels */
    uint32_t            lastOpt[2];
    uint32_t            lastLink[2];
    EDMA3_DRV_PaRAMRegs *startPrmPtr;
    EDMA3_DRV_PaRAMRegs *pingPtrL2p;
    EDMA3_DRV_PaRAMRegs *pongPtrL2p;
    uint8_t             pingVar[2];
    uint8_t             pongVar[2];
    EDMA3_DRV_PaRAMRegs revtPrm[2];
} Tcp3d_Internal;

/**
 *  Local Functions
 */
static EDMA3_DRV_Result Tcp3d_getEdmaChParamAddr (IN Tcp3d_Instance *tcp3dInst);
static EDMA3_DRV_Result Tcp3d_enableEdmaChannels (IN Tcp3d_Instance *tcp3dInst);
static EDMA3_DRV_Result Tcp3d_initEdmaChParam (IN Tcp3d_Instance    *tcp3dInst);
static EDMA3_DRV_Result Tcp3d_resetEdmaChParam (IN Tcp3d_Instance   *tcp3dInst,
                                                IN uint32_t         pingNumCBs,
                                                IN uint32_t         pongNumCBs);
static void Tcp3d_initPseudoParam ( IN Tcp3d_Instance   *tcp3dInst,
                                    IN uint32_t         codeBlocks,
                                    IN Tcp3d_Config     *pingConfig,
                                    IN Tcp3d_Config     *pongConfig);
static void Tcp3d_resetPseudoParam (IN Tcp3d_Instance   *tcp3dInst,
                                    IN uint32_t         codeBlocks);
static void Tcp3d_setLocalVariables (IN Tcp3d_Instance  *tcp3dInst);
static void Tcp3d_resetRuntimeVariables (IN Tcp3d_Instance  *tcp3dInst);
static void Tcp3d_updatePingListVariables ( INOUT  Tcp3d_Instance *inst,
                                            IN     int32_t        pingOutIdx );
static void Tcp3d_updatePongListVariables ( INOUT  Tcp3d_Instance *inst,
                                            IN     int32_t        pongOutIdx );

#endif /* _TCP3D_DRV_PRIV_H_ */
