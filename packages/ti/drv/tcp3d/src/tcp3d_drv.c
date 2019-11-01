/**
 *  \file   tcp3d_drv.c
 *
 *  \brief  TCP3D Driver functions.
 *
 *  Copyright (C) Texas Instruments Incorporated 2009, 2014
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
 *  Include Files
 */
/* TCP3D driver includes */
#include <ti/drv/tcp3d/tcp3d_drv.h>
#include <ti/drv/tcp3d/src/tcp3d_drv_priv.h>

/* TCP3D Types and OSAL defintions: These files can be overriden by customers
 * to point to their copies. Because of this the files have not been explicitly 
 * specified to include the driver path.*/
#include <tcp3d_drv_types.h>
#include <tcp3d_osal.h>

#define TWO_PATHS           0

#define TPCC_REVT_REGS      0
#define TPCC_L2P_REGS       1

#define PING_INDEX          0
#define PONG_INDEX          1

/** @brief Global Variable which describes the TCP3D Driver Version Information */
const char   Tcp3dDrvVersionStr[] = TCP3D_DRV_VERSION_STR ":" __DATE__  ":" __TIME__;

/****************************************************************************
 *              TCP3D Driver Functions                                      *
 ****************************************************************************/
/*******************************************************************************
 * Enables interrupt generation by EDMA CC. Sets two bits of L2P channels  
 * in the TPCC_IESR/TPCC_IESRH.
 ******************************************************************************/
static void Tcp3d_enableEdmaL2pIntr(Tcp3d_Instance *inst)
{
#if UNTESTED_CODE
    *inst->intEnSetReg[TPCC_L2P_REGS] = (inst->l2pChMaskPing | inst->l2pChMaskPong);
#ifdef SIM_WORKAROUND
    /**
     * WORKAROUND: Current Simulator does not generate interrupt
     * when there is a pending IPR bit and we enable the IER in the
     * EDMA register space. So, We are forcing the EDMA to generate
     * the interrupt if the IPR has valid bits. 
     */
    if ( inst->intPendReg[TPCC_L2P_REGS] &&
         (inst->l2pChMaskPing | inst->l2pChMaskPong) )
    {
        inst->tpccShadowRegs->TPCC_IEVAL = 1;
    }
#endif
#else
    /* Set/Clear ITCINT bit in OPT field based on drvCtrl->intrFlag */
    inst->pingPtrL2p->opt |= (1 << CSL_TPCC_PARAM_OPT_ITCINTEN_SHIFT);
    inst->pongPtrL2p->opt |= (1 << CSL_TPCC_PARAM_OPT_ITCINTEN_SHIFT);
    inst->pingPtrL2p->opt |= (1 << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);
    inst->pongPtrL2p->opt |= (1 << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);
#endif
    inst->pingL2pEnCntr++;
}

/*******************************************************************************
 * Disables interrupt generation by EDMA CC. Sets two bits of L2P channels 
 * in the TPCC_IECR/TPCC_IECRH.
 ******************************************************************************/
static void Tcp3d_disableEdmaL2pIntr(Tcp3d_Instance *inst)
{
#if UNTESTED_CODE
    *inst->intEnClrReg[TPCC_L2P_REGS] = (inst->l2pChMaskPing | inst->l2pChMaskPong);      
#else
    /* Set/Clear ITCINT bit in OPT field based on drvCtrl->intrFlag */
    inst->pingPtrL2p->opt &= ~(1 << CSL_TPCC_PARAM_OPT_ITCINTEN_SHIFT);
    inst->pongPtrL2p->opt &= ~(1 << CSL_TPCC_PARAM_OPT_ITCINTEN_SHIFT);
    inst->pingPtrL2p->opt &= ~(1 << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);
    inst->pongPtrL2p->opt &= ~(1 << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);
#endif
    inst->pingL2pEnCntr--;
}

/*******************************************************************************
 * Clears pending interrupt generated by EDMA CC. Sets two bits of L2P channels 
 * in the TPCC_ICR/TPCC_ICRH.
 ******************************************************************************/
static void Tcp3d_clearEdmaL2pIntr(Tcp3d_Instance *inst)
{
#if UNTESTED_CODE
    *inst->clrIntPendReg[TPCC_L2P_REGS] = (inst->l2pChMaskPing | inst->l2pChMaskPong);      
#endif
    inst->pingL2pEnCntr=0;
}

/*******************************************************************************
 * Enables interrupt generation by EDMA CC. Sets two bits of REVT channels  
 * in the TPCC_IESR/TPCC_IESRH.
 ******************************************************************************/
static void Tcp3d_enableEdmaPauseIntr(Tcp3d_Instance *inst)
{
    *inst->intEnSetReg[TPCC_REVT_REGS] = (inst->pauseChMaskPing | inst->pauseChMaskPong);
#ifdef SIM_WORKAROUND
    /**
     * WORKAROUND: Current Simulator does not generate interrupt
     * when there is a pending IPR bit and we enable the IER in the
     * EDMA register space. So, We are forcing the EDMA to generate
     * the interrupt if the IPR has valid bits. 
     */
    if ( inst->intPendReg[TPCC_REVT_REGS] &&
         (inst->pauseChMaskPing | inst->pauseChMaskPong) )
    {
        inst->tpccShadowRegs->TPCC_IEVAL = 1;
    }
#endif
    inst->pingPauseEnCntr++;
}

/*******************************************************************************
 * Disables interrupt generation by EDMA CC. Sets two bits of REVT channels 
 * in the TPCC_IECR/TPCC_IECRH.
 ******************************************************************************/
static void Tcp3d_disableEdmaPauseIntr(Tcp3d_Instance *inst)
{
    *inst->intEnClrReg[TPCC_REVT_REGS] = (inst->pauseChMaskPing | inst->pauseChMaskPong);      
    inst->pingPauseEnCntr--;
}

/*******************************************************************************
 * Clears pending interrupt generated by EDMA CC. Sets two bits of REVT channels 
 * in the TPCC_ICR/TPCC_ICRH.
 ******************************************************************************/
static void Tcp3d_clearEdmaPauseIntr(Tcp3d_Instance *inst)
{
    *inst->clrIntPendReg[TPCC_REVT_REGS] = (inst->pauseChMaskPing | inst->pauseChMaskPong);      
    inst->pingPauseEnCntr=0;
}

/**
 *  @brief      TCP3D Driver function for providing the number of buffers
 *              required.
 */
Tcp3d_Result Tcp3d_getNumBuf (IN Tcp3d_SizeCfg  *cfg,
                              OUT int16_t         *nbufs)
{
    *nbufs = TCP3D_DRV_NUM_BUF;

    return ( TCP3D_DRV_NO_ERR );
}

/**
 *  @brief      TCP3D Driver function for providing the attributes of all the
 *              number of buffers requested through the structure of type
 *              Tcp3d_MemBuffer provided.
 */
Tcp3d_Result Tcp3d_getBufDesc ( IN Tcp3d_SizeCfg    *cfg,
                                OUT Tcp3d_MemBuffer bufs[])
{
    bufs[TCP3D_DRV_INST_BUFN].size       = sizeof(Tcp3d_Instance);
    bufs[TCP3D_DRV_INST_BUFN].log2align  = 2;
    bufs[TCP3D_DRV_INST_BUFN].mclass     = Tcp3d_BufClass_L2RAM;
    bufs[TCP3D_DRV_INST_BUFN].volat      = TRUE;

    bufs[TCP3D_DRV_PSEUDO_PARAM_BUFN].size       = (cfg->maxCodeBlocks<<5)*TCP3D_DRV_LINK_CB;
    bufs[TCP3D_DRV_PSEUDO_PARAM_BUFN].log2align  = 2;
    bufs[TCP3D_DRV_PSEUDO_PARAM_BUFN].mclass     = Tcp3d_BufClass_L2RAM;
    bufs[TCP3D_DRV_PSEUDO_PARAM_BUFN].volat      = TRUE;

    return ( TCP3D_DRV_NO_ERR );
}

/**
 *  @brief      TCP3D Driver Initialization function which must be called only
 *              once to initialize the driver instance and other required
 *              resources needed for the driver functionality.
 */
Tcp3d_Result Tcp3d_init( IN  Tcp3d_MemBuffer     bufs[],
                         IN  Tcp3d_InitParams    *drvInitParams)
{
    Tcp3d_Result            tcp3dResult = TCP3D_DRV_NO_ERR;
    Tcp3d_Instance          *tcp3dInst;
    uint32_t                modeReg = 0, endReg = 0, exeRegP0 = 0, exeRegP1 =0;
    int32_t                 idx, locBufSize[TCP3D_DRV_NUM_BUF];
    CSL_Tcp3d_cfgRegs       *tcp3dCfgRegs;

    /* Check for valid instance number */
    if ( drvInitParams->instNum >= TCP3D_DRV_MAX_NUM_INSTANCES )
    {
        /* Return error */
        tcp3dResult = TCP3D_DRV_INVALID_INSTANCE_NUMBER;

        return ( tcp3dResult );
    }

    /* compute the buffer sizes table */
    locBufSize[TCP3D_DRV_INST_BUFN] = sizeof(Tcp3d_Instance);
    locBufSize[TCP3D_DRV_PSEUDO_PARAM_BUFN] = (drvInitParams->maxCodeBlocks<<5)*TCP3D_DRV_LINK_CB;

    /* Check if all buffers have valid addresses and sizes */
    for ( idx = 0; idx < TCP3D_DRV_NUM_BUF; idx++)
    {
        if ( bufs[idx].base == NULL || bufs[idx].size < locBufSize[idx] )
            tcp3dResult = TCP3D_DRV_INVALID_BUFF;
    }

    /* Initialize the instance if the buffer addresses are not NULL */
    if ( tcp3dResult == TCP3D_DRV_NO_ERR )
    {
        /* Initialize the driver Instance */
        tcp3dInst = (Tcp3d_Instance *) bufs[TCP3D_DRV_INST_BUFN].base;

        /* Initialize the pseudo PaRAM array pointer */
        tcp3dInst->pseudoParamBufPtr = (EDMA3_DRV_PaRAMRegs *) bufs[TCP3D_DRV_PSEUDO_PARAM_BUFN].base;
    }
    else
    {
        /* Return the error */
        return ( tcp3dResult );
    }

    /* Get the address of TCP3D configuration registers base address */
    tcp3dCfgRegs                    = drvInitParams->tcp3dCfgRegs;

    /* Update the instace with the input parameters */
    tcp3dInst->mode                 = drvInitParams->ctrlParams.mode;
    tcp3dInst->doubleBuffer         = drvInitParams->ctrlParams.doubleBuf;
    tcp3dInst->edmaHnd              = drvInitParams->edmaHnd;
    tcp3dInst->edmaRegionId         = drvInitParams->edmaRegionId;
    tcp3dInst->maxCodeBlocks        = drvInitParams->maxCodeBlocks;
    tcp3dInst->notificationEventNum = drvInitParams->notificationEventNum;
    tcp3dInst->cpIntc0RegsBase      = drvInitParams->cpIntc0RegsBase;
    tcp3dInst->tpccShadowRegs       = drvInitParams->edma3ShadowRegsBase;
    tcp3dInst->instNum              = drvInitParams->instNum;
    tcp3dInst->coreId               = drvInitParams->coreID;

    /* Verify the REVT channels */
    if ( ( drvInitParams->pingCh[0] != drvInitParams->pingConfig.revtCh ) ||
         ( drvInitParams->pongCh[0] != drvInitParams->pongConfig.revtCh ) )
    {
        /* Return error */
        tcp3dResult = TCP3D_DRV_INVALID_EDMA_CH;

        return ( tcp3dResult );
    }

    for ( idx = 0; idx < TCP3D_DRV_MAX_CH_PER_PATH; idx++ )
    {
        /* Copy PING Channels */
        tcp3dInst->pingCh[idx]    = drvInitParams->pingCh[idx];

        /* Copy PONG Channels */
        tcp3dInst->pongCh[idx]    = drvInitParams->pongCh[idx];
    }

    for ( idx = 0; idx < (TCP3D_DRV_MAX_LINK_CH>>1); idx++ )
    {
        /* Copy PING Channels */
        tcp3dInst->pingLinkCh[idx]  = drvInitParams->linkCh[idx];

        /* Copy PONG Channels */
        tcp3dInst->pongLinkCh[idx]  = drvInitParams->linkCh[idx+(TCP3D_DRV_MAX_LINK_CH>>1)];
    }

    /* Set the Driver variables to defaults */
    tcp3dInst->constantOne          = 1;
    tcp3dInst->pauseState           = TCP3D_DRV_STATE_PAUSE;

    /* Initialize the driver variables */
    tcp3dInst->maxPingCbCnt         = (tcp3dInst->maxCodeBlocks >> 1);
    tcp3dInst->maxPongCbCnt         = (tcp3dInst->maxCodeBlocks >> 1);
    tcp3dInst->maxPingCbIdx         = ((tcp3dInst->maxPingCbCnt - 1) << 1);
    tcp3dInst->maxPongCbIdx         = ((tcp3dInst->maxPongCbCnt << 1) - 1);

    /* Reset run-time variables */
    Tcp3d_resetRuntimeVariables(tcp3dInst);

    /* EDMA3 - get PaRAM addresses of all physical channels */
    if ( EDMA3_DRV_SOK != Tcp3d_getEdmaChParamAddr(tcp3dInst) )
    {
        tcp3dResult = TCP3D_DRV_FAIL_EDMA_GET_PARAM_ADDR;

        return ( tcp3dResult );
    }

    /* Initialize the EDMA PaRAM memory for the physical channels */
    if ( EDMA3_DRV_SOK != Tcp3d_initEdmaChParam(tcp3dInst) )
    {
        tcp3dResult = TCP3D_DRV_FAIL_EDMA_PARAM_INIT;

        return ( tcp3dResult );
    }

    /* EDMA3 - Enable the EVENT triggered channels */
    if ( EDMA3_DRV_SOK != Tcp3d_enableEdmaChannels(tcp3dInst) )
    {
        tcp3dResult = TCP3D_DRV_FAIL_EDMA_ENABLE_CHANNEL;

        return ( tcp3dResult );
    }

    /* Initialize the pseudo PaRAM memory */
    Tcp3d_initPseudoParam ( tcp3dInst,
                            tcp3dInst->maxCodeBlocks,
                            &drvInitParams->pingConfig,
                            &drvInitParams->pongConfig);

    /* Prepare the control registers for both P0 and P1 processes */
    Tcp3d_prepControlRegs ( &drvInitParams->ctrlParams,
                                &modeReg,
                                &endReg,
                                &exeRegP0,
                                &exeRegP1);

    /* 
     * Soft Reset the TCP3D.
     * Insert NOPs b/w the reset calls in HW to work properly.
     */
    tcp3dCfgRegs->TCP3_SOFT_RESET = 1;
    tcp3dCfgRegs->TCP3_SOFT_RESET = 0;

    /**
     * Write to the Control registers with the prepared values to start
     * the TCP3D state machine.
     */
    tcp3dCfgRegs->TCP3_MODE     = modeReg;
    tcp3dCfgRegs->TCP3_END      = endReg;
    tcp3dCfgRegs->TCP3_EXE_P0   = exeRegP0;
    if ( tcp3dInst->doubleBuffer != CSL_TCP3D_CFG_TCP3_MODE_IN_MEM_DB_EN_ENABLE )
        tcp3dCfgRegs->TCP3_EXE_P1   = exeRegP1;

    /* Set Local Variables used in the runtime APIs */
    Tcp3d_setLocalVariables(tcp3dInst);

    /* Disable the interrupts */
    Tcp3d_disableEdmaL2pIntr(tcp3dInst);
    Tcp3d_disableEdmaPauseIntr(tcp3dInst);

    /* Clear pending interrupts */
    Tcp3d_clearEdmaL2pIntr(tcp3dInst);
    Tcp3d_clearEdmaPauseIntr(tcp3dInst);

    /* Reset the EDMA Channels */
    Tcp3d_resetEdmaChParam (tcp3dInst,
                            tcp3dInst->maxPingCbCnt,
                            tcp3dInst->maxPongCbCnt);

    /* Change the state */
    tcp3dInst->state    = TCP3D_DRV_STATE_INIT;

    return ( tcp3dResult );

} /* end of - Tcp3d_init() function */

/**
 * @brief      TCP3D Driver Deinitialization function.
 */
Tcp3d_Result Tcp3d_deInit ( IN Tcp3d_Instance *inst )
{
    /* Disable interrupts */
    Tcp3d_disableEdmaL2pIntr(inst);        
    Tcp3d_disableEdmaPauseIntr(inst);        

    /* Clear pending interrupts */
    Tcp3d_clearEdmaL2pIntr(inst);
    Tcp3d_clearEdmaPauseIntr(inst);

    return ( TCP3D_DRV_NO_ERR );
}

/**
 *  @brief      TCP3D Driver function called to reset the driver at sub-frame
 *              boundary. This function does the following:
 *              1) Set the instance with the passed values - for example number
 *                  of blocks for decoding in this subframe which is needed to
 *                  for checking the boundary and a new status array pointer
 *                  where the status register values for each code block are
 *                  trasferred.
 *              2) Initialize all the run-time instance variables to default.
 *              3) Initialize the pseudo PaRAM memory with all the defaults
 *                  based on mode.
 *              4) Reset the EDMA channels with default values. 
 */
Tcp3d_Result Tcp3d_reset (  IN Tcp3d_Instance  *tcp3dInst,
                            IN uint32_t        codeBlocks)
{
    Tcp3d_Result            tcp3dResult = TCP3D_DRV_NO_ERR;

    /* First check for valid statte */
    if ( ( tcp3dInst->pingStop != 1 ) && ( tcp3dInst->pongStop != 1) )
        tcp3dResult = TCP3D_DRV_INVALID_STATE;

    /* Check if the codeblocks value is valid */
    if ( ( codeBlocks > tcp3dInst->maxCodeBlocks ) || ( codeBlocks < 2 ) )
        tcp3dResult = TCP3D_DRV_INVALID_PARAMETER;

    /* Return if any error found */
    if ( tcp3dResult != TCP3D_DRV_NO_ERR )
        return ( tcp3dResult );

    /* update the maxCodeBlocks and dependent variables, if needed */
    if ( codeBlocks != NULL )
    {
        tcp3dInst->maxCodeBlocks    = codeBlocks;
        tcp3dInst->maxPingCbCnt     = (codeBlocks >> 1);
        tcp3dInst->maxPongCbCnt     = (codeBlocks >> 1);
        /* max position index in pseudo Param Buffer */
        tcp3dInst->maxPingCbIdx     = ((tcp3dInst->maxPingCbCnt - 1) << 1);
        tcp3dInst->maxPongCbIdx     = ((tcp3dInst->maxPongCbCnt << 1) - 1);
    }

    /* Reset run-time variables */
    Tcp3d_resetRuntimeVariables(tcp3dInst);

    /* Reset the pseudo PaRAM memory */
    Tcp3d_resetPseudoParam(tcp3dInst, tcp3dInst->maxCodeBlocks);

    /* Reset the EDMA Channels */
    Tcp3d_resetEdmaChParam (tcp3dInst,
                            tcp3dInst->maxPingCbCnt,
                            tcp3dInst->maxPongCbCnt);

    /* Disable the interrupts */
    Tcp3d_disableEdmaL2pIntr(tcp3dInst);
    Tcp3d_disableEdmaPauseIntr(tcp3dInst);

    /* Clear pending interrupts */
    Tcp3d_clearEdmaL2pIntr(tcp3dInst);
    Tcp3d_clearEdmaPauseIntr(tcp3dInst);

    /* Change the state */
    tcp3dInst->state = TCP3D_DRV_STATE_INIT;

    return ( tcp3dResult );
} /* end of - Tcp3d_reset() function */

/**
 *  @brief      TCP3D Driver function for enqueuing the code blocks to the input
 *              list. Here the input list is a pseudo PaRAM list consisting of
 *              actual PaRAM entries for INCFG, LLR, HD, SD & STS transfers.
 */
Tcp3d_Result Tcp3d_enqueueCodeBlock(IN  Tcp3d_Instance  *tcp3dInst,
                                    IN  uint32_t          blockLength,
                                    IN  uint32_t          *inputConfigPtr,
                                    IN  int8_t            *llrPtr,
                                    IN  uint32_t          llrOffset,
                                    IN  uint32_t          *hdPtr,
                                    IN  int8_t            *sdPtr,
                                    IN  uint32_t          sdOffset,
                                    IN  uint32_t          *statusPtr,
                                    IN  uint8_t           ntfEventFlag)
{
    EDMA3_DRV_PaRAMRegs     *lastOutPrm;
    int32_t                 blockIndex;
    int32_t                 pathFlag;
    Tcp3d_Result            tcp3dResult = TCP3D_DRV_NO_ERR;
    Tcp3d_Result            tcp3dResult2 = TCP3D_DRV_NO_ERR;
    EDMA3_DRV_PaRAMRegs     *prmCfg;
    EDMA3_DRV_PaRAMRegs     *prmLlr;
    EDMA3_DRV_PaRAMRegs     *prmHd;
    EDMA3_DRV_PaRAMRegs     *prmSts;
    EDMA3_DRV_PaRAMRegs     *prmSd;
    EDMA3_DRV_PaRAMRegs     *prevLastPrmPtr;
    EDMA3_DRV_PaRAMRegs     *prmWrap;

    uint16_t                ntfLink[2];
    uint16_t                stsLink[2];
    uint16_t                sdLink[2];
    uint32_t                wrapLink[2];
    uint16_t                chainToNextCbDummyLink[2];
    uint16_t                chainToNextCbNtfdLink[2];

    ntfLink[PING_INDEX]  = tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_NTF];
    ntfLink[PONG_INDEX]  = tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_NTF];
    stsLink[PING_INDEX]  = tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_STS];
    stsLink[PONG_INDEX]  = tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_STS];
    sdLink[PING_INDEX]   = tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_SD];
    sdLink[PONG_INDEX]   = tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_SD];
    wrapLink[PING_INDEX] = tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_WRAP];
    wrapLink[PONG_INDEX] = tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_WRAP];
    chainToNextCbDummyLink[PING_INDEX] = tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_NEXTCB_DUMMY];
    chainToNextCbDummyLink[PONG_INDEX] = tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_NEXTCB_DUMMY];
    chainToNextCbNtfdLink[PING_INDEX] = tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_NEXTCB_NTFD];
    chainToNextCbNtfdLink[PONG_INDEX] = tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_NEXTCB_NTFD];

#if TWO_PATHS
    /* get next available path index */
    pathFlag = tcp3dInst->nextCodeBlockIndex;
    if ( pathFlag )
    {
        if ( tcp3dInst->pongFreeCnt )
        {
            blockIndex = tcp3dInst->nextPongInIdx;
        }
        else if ( tcp3dInst->pingFreeCnt )
        {
            blockIndex = tcp3dInst->nextPingInIdx;
            pathFlag = 0;
        }
    }
    else
    {
        if ( tcp3dInst->pingFreeCnt )
        {
            blockIndex = tcp3dInst->nextPingInIdx;
        }
        else if ( tcp3dInst->pongFreeCnt )
        {
            blockIndex = tcp3dInst->nextPongInIdx;
            pathFlag = 1;
        }
    }

    /* update the next path flag */
    if ( pathFlag )
        tcp3dInst->nextCodeBlockIndex = 0;
    else
        tcp3dInst->nextCodeBlockIndex = 1;

    /* add new block to the list until available capacity */
    if ( tcp3dInst->pingFreeCnt || tcp3dInst->pongFreeCnt )
#else
    /* path flag from the blockIndex value */
    blockIndex  = tcp3dInst->nextCodeBlockIndex;
    pathFlag    = (blockIndex & 1);

    /* add new block to the list until available capacity */
    if ( ( ( pathFlag == 0 ) && tcp3dInst->pingFreeCnt ) ||
         ( ( pathFlag == 1 ) && tcp3dInst->pongFreeCnt ) )
#endif
    {
        /* Update the pointers */
        prmCfg  = &tcp3dInst->pseudoParamBufPtr[blockIndex*TCP3D_DRV_LINK_CB];
        prmLlr  = prmCfg + LINK_CH_IDX_LLR;
        prmHd   = prmCfg + LINK_CH_IDX_HD;
        prmSts  = prmCfg + LINK_CH_IDX_STS;
        prmSd   = prmCfg + LINK_CH_IDX_SD;
    
        /**
         * Update addresses first
         */
        prmCfg->srcAddr     = (uint32_t) inputConfigPtr;
        prmLlr->srcAddr     = (uint32_t) llrPtr;
        prmHd->destAddr     = (uint32_t) hdPtr;
        prmSts->destAddr    = (uint32_t) statusPtr;
        prmSd->destAddr     = (uint32_t) sdPtr;
        /**
         * Update counts for needed
         */
        prmSd->aCnt         = blockLength;
        if ( ( tcp3dInst->mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_LTE) ||
             ( tcp3dInst->mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_WIMAX ) )
        {
            prmLlr->aCnt        = (blockLength>>1);
            prmLlr->srcBIdx     = (blockLength>>1);
            prmLlr->srcCIdx     = llrOffset;
            prmHd->aCnt         = (blockLength>>3);
            prmSd->destBIdx     = sdOffset;
        }
        else
        {
            prmLlr->aCnt        = COMPUTE_KOUT(blockLength);
            prmLlr->srcBIdx     = llrOffset;
            prmHd->aCnt         = COMPUTE_HD_BYTE_SIZE(blockLength);
        }

        /**
         * Link Status & Soft decisions if avaiable.
         * NOTE: For wrap-around case, load OPT & LINK fields with reset values
         */
        /* load HD OPT reset value */
        prmHd->opt          = tcp3dInst->resetHdOpt[pathFlag];
        /* Check if STS is available */
        if ( statusPtr != NULL )
        {
            /* update HD link */
            prmHd->linkAddr     = stsLink[pathFlag];
            /* load STS OPT reset value */
            prmSts->opt         = tcp3dInst->resetStsOpt[pathFlag];
            /* Check if SD is available */
            if ( sdPtr != NULL )
            {
                /* load SD OPT reset value */
                prmSd->opt          = tcp3dInst->resetSdOpt[pathFlag];
                /* update Status link */
                prmSts->linkAddr    = sdLink[pathFlag];
                /* load SD link reset value */
                prmSd->linkAddr     = tcp3dInst->resetSdLink[pathFlag];
                /* last param pointer */
                lastOutPrm = prmSd;
            }
            else
            {
                /* link reset value */
                prmSts->linkAddr    = tcp3dInst->resetStsLink[pathFlag];
                /* last param pointer */
                lastOutPrm = prmSts;
            }
        }
        else if ( sdPtr != NULL )
        {
            /* update HD link */
            prmHd->linkAddr     = sdLink[pathFlag];
            /* load SD OPT reset value */
            prmSd->opt          = tcp3dInst->resetSdOpt[pathFlag];
            /* load SD link reset value */
            prmSd->linkAddr     = tcp3dInst->resetSdLink[pathFlag];
            /* last param pointer */
            lastOutPrm = prmSd;
        }
        else
        {
            /* load HD link reset value */
            prmHd->linkAddr     = tcp3dInst->resetHdLink[pathFlag];
            /* last param pointer */
            lastOutPrm = prmHd;
        }

        /* Add the WRAP param at the end of the output PaRAM */
        if ( blockIndex >= (tcp3dInst->maxCodeBlocks-2) )
        {
            /* get the pointer for the wrap param for the appropriate path */
            prmWrap = (EDMA3_DRV_PaRAMRegs *) wrapLink[pathFlag];
            /* link with last out param */
            lastOutPrm->linkAddr   = 0xFFFF & wrapLink[pathFlag];
            /* change the last param pointer */
            lastOutPrm = prmWrap;
        }

        /**
         * If Interrupt nofication is requested, change the default link 
         * to LINK_CH_IDX_NTF for the last PaRAM.
         */
        if ( ntfEventFlag )
        {
            lastOutPrm->linkAddr = ntfLink[pathFlag];
        }

        /**
         * Chain the block to previous one in each path. This is done for blocks
         * starting from the second one in each path.
         * 
         * Since the blockIndex is tracking linearly the pseudo param buffer
         * and ping & pong lists are used as interleaved lists, chaining is
         * done starting from index 2 onwards till max.
         */
        if ( blockIndex > 1 )
        {
            /* Get the previous last output PaRAM, used in chaining */
            prevLastPrmPtr  = tcp3dInst->lastParam[pathFlag];
    
            /**
             * If previous block has notification, change the previous param
             * link to NTFD, otherwise change to dummy REVT link PaRAM.
             */
            if ( tcp3dInst->prevNtfFlag[pathFlag] )
            {
                /* Change the LINK to interrupt Notify PaRAM */
                prevLastPrmPtr->linkAddr = chainToNextCbNtfdLink[pathFlag];
            }
            else
            {
                /* Change the LINK to dummy REVT PaRAM */
                prevLastPrmPtr->linkAddr = chainToNextCbDummyLink[pathFlag];
            }
        }

        /* Store ntfEventFlag, used in chaining next block */
        tcp3dInst->prevNtfFlag[pathFlag] = ntfEventFlag;

        /* Store the last output PaRAM, used in chaining next block */
        tcp3dInst->lastParam[pathFlag] = lastOutPrm;

        /* Increment the ping/pong load counter */
        if ( pathFlag )
        {
            tcp3dInst->pongLoadCnt++;
            tcp3dInst->pongFreeCnt--;
            tcp3dInst->nextPongInIdx += 2;
        }
        else
        {
            tcp3dInst->pingLoadCnt++;
            tcp3dInst->pingFreeCnt--;
            tcp3dInst->nextPingInIdx += 2;
        }

        /* Reset index when reached maximum */
        if ( tcp3dInst->nextPingInIdx > tcp3dInst->maxPingCbIdx )
        {
            tcp3dInst->nextPingInIdx = 0;
        }
        if ( tcp3dInst->nextPongInIdx > tcp3dInst->maxPongCbIdx )
        {
            tcp3dInst->nextPongInIdx = 1;
        }

#if !TWO_PATHS
        /* Increment the index value until reaching the last block */
        tcp3dInst->nextCodeBlockIndex++;

        /* Reset index when reached maximum */
        if ( tcp3dInst->nextCodeBlockIndex >= tcp3dInst->maxCodeBlocks )
        {
            tcp3dInst->nextCodeBlockIndex = 0;
        }
#endif
    } /* if enqueue possible */
    else
    {
        tcp3dResult = TCP3D_DRV_INPUT_LIST_FULL;
    } /* if enqueue not possible */

    /* Call Start function as needed */
    if ( ( tcp3dInst->startFlag ) &&
         ( (tcp3dInst->pingStop) || (tcp3dInst->pongStop) ) )
    {
        tcp3dResult2 = Tcp3d_start(tcp3dInst, TCP3D_DRV_START_AUTO);

        /* If start returned an error, generate an error message. */
        if ( TCP3D_DRV_NO_ERR != tcp3dResult2 )
        {
            Tcp3d_osalLog("Enqueue: Tcp3d_start function returned error with value : %d\n", tcp3dResult2);
        }
    }

    return ( tcp3dResult );
} /* end of - Tcp3d_enqueueCodeBlock() function */

/**
 *  @brief      This API could be used for starting the driver to start doing
 *              EDMA transfers to TCP3 decoder for decoding from the pseudo
 *              PaRAM list.
 *
 *              This function is executed at the application task thread for
 *              starting either the PING or PONG path execution.
 */
Tcp3d_Result Tcp3d_start (  INOUT Tcp3d_Instance    *inst,
                            IN    uint8_t           startMode)
{
    EDMA3_DRV_Result        result = EDMA3_DRV_SOK;
    EDMA3_DRV_PaRAMRegs     *currPrmPtr1, *currPrmPtr2;
    Tcp3d_Result            tcp3dResult = TCP3D_DRV_NO_ERR;
    uint16_t                startNeeded = 0; /* 0 - not started, 1 - started */  
    int32_t                 pingOutIdx, pongOutIdx;

    /* Set startFlag if first call to the function is made with AUTO mode */
    if ( ( startMode == TCP3D_DRV_START_AUTO) && ( !inst->startFlag ) )
    {
        inst->startFlag = startMode;
    }

    /**
     * Return immediately, if both stop flags are not set.
     * This is possible in two scenarios
     *  1) After the init and before calling start with AUTO mode.
     *  2) In steady state when both ping and pong decoders are busy. 
     */
    if ( ( ( inst->pingStop == NULL ) && ( inst->pongStop == NULL ) ) ||
         ( !inst->startFlag ) )
    {
        return ( tcp3dResult );
    }

    /**
     * Check startMode parameter to see if start needed
     */
    if ( startMode == TCP3D_DRV_START_AUTO )
    {
        /**
         * Start is needed in the following cases.
         *  - if the current out index is less than next in index
         *  - if not, in the wrap case when blocks are loaded from beginning
         *      of the list where out index could be greater than next in index.
         *      Here check for the load count.
         * 
         * Two LSB bits of the variable startNeeded are used for indicating the
         * need to do start. LSB0 is used for PING and LSB1 is used for PONG. 
         */
        /* PING STOP */
        if ( inst->pingStop )
        {
            /**
             * Only update the list variables if the driver is stopped 
             * for the ping side.
             */
            /**
             * Read the source address of L2P Channel PaRAM to get the current
             * pseudo PaRAM pointer for PING paths. Then compare with the 
             * start pointer to get the out index. 
             */
            currPrmPtr1 = (EDMA3_DRV_PaRAMRegs *) inst->pingPtrL2p->srcAddr;
            pingOutIdx  = GET_CB_IDX(currPrmPtr1 - inst->startPrmPtr);

            /* Update the counters and indexes using the current out indexes */
            Tcp3d_updatePingListVariables( inst, pingOutIdx );

            if ( pingOutIdx < inst->nextPingInIdx )
            {
                startNeeded |= 1;
            }
            else if ( inst->pingLoadCnt > 0 )
            {
                startNeeded |= 1;
            }
        }

        /* PONG STOP */
        if ( inst->pongStop )
        {
            /**
             * Only update the list variables if the driver is stopped 
             * for the pong side.
             */
            /**
             * Read the source address of L2P Channel PaRAM to get the current
             * pseudo PaRAM pointer for PONG paths. Then compare with the 
             * start pointer to get the out index. 
             */
            currPrmPtr2 = (EDMA3_DRV_PaRAMRegs *) inst->pongPtrL2p->srcAddr;
            pongOutIdx  = GET_CB_IDX(currPrmPtr2 - inst->startPrmPtr);

            /* Update the counters and indexes using the current out indexes */
            Tcp3d_updatePongListVariables( inst, pongOutIdx );

            if ( pongOutIdx < inst->nextPongInIdx )
            {
                startNeeded |= 2;
            }
            else if ( inst->pongLoadCnt > 0 )
            {
                startNeeded |= 2;
            }
        }
    }
    else if ( ( startMode == TCP3D_DRV_START_PING ) && ( inst->pingStop ) )
    {
        startNeeded |= 1;
    }
    else if ( ( startMode == TCP3D_DRV_START_PONG ) && ( inst->pongStop ) )
    {
        startNeeded |= 2;
    }

    /* If LSB0 is set, start PING */
    if ( startNeeded & 0x1 )
    {
        /**
         *  Clear the wrap adjust flags,
         *  when the last block decoding in the ping list is detected.
         */
        if (inst->pingLastOutFlag)
        {
            inst->pingLastOutFlag = 0;
            inst->pingWrapCheck = 1;
        }

        /* increment counter */
        inst->pingStartCntr++;
        /* Clear the stop flag */
        inst->pingStop = 0;
        /* Enable L2P channel in the PING path */
        result |= EDMA3_DRV_enableTransfer( inst->edmaHnd,
                                            inst->pingCh[TCP3D_DRV_CH_IDX_L2P],
                                            EDMA3_DRV_TRIG_MODE_MANUAL);
    }

    /* If LSB1 is set, start PONG */
    if ( startNeeded & 0x2 )
    {
        /**
         *  Clear the wrap adjust flags,
         *  when the last block decoding in the pong list is detected.
         */
        if (inst->pongLastOutFlag)
        {
            inst->pongLastOutFlag = 0;
            inst->pongWrapCheck = 1;
        }

        /* increment counter */
        inst->pongStartCntr++;
        /* Clear the stop flag */
        inst->pongStop = 0;

        /* Enable L2P channel in the PONG path */
        result |= EDMA3_DRV_enableTransfer( inst->edmaHnd,
                                            inst->pongCh[TCP3D_DRV_CH_IDX_L2P],
                                            EDMA3_DRV_TRIG_MODE_MANUAL);
    }

    /* Update the return status, if any EDMA starts fail */
    if ( result != EDMA3_DRV_SOK )
    {
        tcp3dResult = TCP3D_DRV_FAIL_EDMA_ENABLE_CHANNEL;
    }

    /* Change the state to RUNNING, if start successful */
    if ( ( startNeeded ) && ( result == EDMA3_DRV_SOK ) )
    {
        inst->state = TCP3D_DRV_STATE_RUNNING;
    }

    return ( tcp3dResult );

} /* end of - Tcp3d_start() function */

/**
 *  @brief      This API could be used for querying the TCP3D driver to get
 *              updates or take appropriate actions.
 * 
 *  \note       This API is not fully scoped currently and the possible query
 *              commands and their actions are open as of now.
 */
Tcp3d_Result Tcp3d_status ( IN    Tcp3d_Instance    *inst,
                            INOUT Tcp3d_Sts         *drvStatus )
{
    Tcp3d_Result            tcp3dResult = TCP3D_DRV_NO_ERR;
    EDMA3_DRV_PaRAMRegs     *currPrmPtr1, *currPrmPtr2;

    /* Check the control command */
    switch ( drvStatus->cmd )
    {
        case TCP3D_DRV_GET_STATE :
            /* Read state value from driver instance */
            drvStatus->state = inst->state;
            break;

        case TCP3D_DRV_GET_MIN_OUT_IDX:
            /**
             * Get the L2P Channel PaRAM address and then read the source
             * address from the PaRAM to get the index to the pseudo PaRAM
             * current read pointer.
             * 
             * Compute the minimum index by comparing the current indexes with
             * the pseudo PaRAM start pointer.
             */
            currPrmPtr1 = (EDMA3_DRV_PaRAMRegs *) inst->pingPtrL2p->srcAddr;
            currPrmPtr2 = (EDMA3_DRV_PaRAMRegs *) inst->pongPtrL2p->srcAddr;
            drvStatus->prmOutIdx = (MIN(currPrmPtr1, currPrmPtr2) - inst->startPrmPtr)>>2;
            break;

        case TCP3D_DRV_GET_PING_OUT_IDX:
            /**
             * Read the source address of L2P Channel PaRAM to get the current
             * pseudo PaRAM pointer for PING path. Then compare with the start
             * pointer for the index.
             */
            currPrmPtr1 = (EDMA3_DRV_PaRAMRegs *) inst->pingPtrL2p->srcAddr;
            drvStatus->prmOutIdx = (currPrmPtr1 - inst->startPrmPtr)>>2;
            break;

        case TCP3D_DRV_GET_PONG_OUT_IDX:
            /**
             * Read the source address of L2P Channel PaRAM to get the current
             * pseudo PaRAM pointer for PONG path. Then compare with the start
             * pointer for the index.
             */
            currPrmPtr2 = (EDMA3_DRV_PaRAMRegs *) inst->pongPtrL2p->srcAddr;
            drvStatus->prmOutIdx = (currPrmPtr2 - inst->startPrmPtr)>>2;
            break;

        default:
            /* If invalid command passed, flag error */
            tcp3dResult = TCP3D_DRV_FAIL;
            break;
    }

    return (tcp3dResult);

} /* end of - Tcp3d_status() function */

/**
 *  \brief      This API could be used for change or update the TCP3D driver
 *              instance values which are set during the init time.
 *  
 *              Currently, there are few commands supported with some the
 *              limitation that they are allowed only when the driver is in
 *              IDLE state.
 * 
 *  \note       -# This API is not fully scoped currently and the possible
 *              control commands and their actions are open as of now.
 *              -# We may need to protect the instance value updations, once
 *              they are allowed to change in any state.
 */
Tcp3d_Result Tcp3d_control (IN Tcp3d_Instance   *inst,
                            IN Tcp3d_Ctrl       *drvCtrl)
{
    Tcp3d_Result            tcp3dResult = TCP3D_DRV_NO_ERR;

    /* Check the control command */
    switch ( drvCtrl->cmd )
    {
        case TCP3D_DRV_SET_L2P_INT :
            if ( drvCtrl->intrFlag )
            {
                Tcp3d_enableEdmaL2pIntr(inst);
            }
            else
            { 
                Tcp3d_disableEdmaL2pIntr(inst);
            }
            break;
        case TCP3D_DRV_SET_REVT_INT :
            if ( drvCtrl->intrFlag )
            {
                Tcp3d_enableEdmaPauseIntr(inst);
            }
            else
            { 
                Tcp3d_disableEdmaPauseIntr(inst);
            }
            break;

        case TCP3D_DRV_CLR_REVT_INT :
            Tcp3d_clearEdmaPauseIntr(inst);
            break;

        case TCP3D_DRV_SET_PING_L2P_INT :
        case TCP3D_DRV_SET_PONG_L2P_INT :
        case TCP3D_DRV_SET_PING_PAUSE_INT :
        case TCP3D_DRV_SET_PONG_PAUSE_INT :
        default:
            /* If invalid command passed, flag error */
            tcp3dResult = TCP3D_DRV_FAIL;
            break;
    }

    return (tcp3dResult);

} /* end of - Tcp3d_control() function */

/*******************************************************************************
 ******************************************************************************/
static void Tcp3d_setLocalVariables (IN Tcp3d_Instance   *tcp3dInst)
{
    // NOTE: Removed after the shadow registers base is set during init
    // CSL_TpccRegs            *tpcc2Regs = (CSL_TpccRegs *) CSL_EDMACC_2_REGS;
    EDMA3_DRV_PaRAMRegs     *prm;

    /* Set EDMA PaRAM pointers */
    tcp3dInst->startPrmPtr = (EDMA3_DRV_PaRAMRegs *) L2GLBMAP(tcp3dInst->coreId, \
                                                tcp3dInst->pseudoParamBufPtr);
    tcp3dInst->pingPtrL2p = (EDMA3_DRV_PaRAMRegs *) tcp3dInst->pingChParamAddr[TCP3D_DRV_CH_IDX_L2P];
    tcp3dInst->pongPtrL2p = (EDMA3_DRV_PaRAMRegs *) tcp3dInst->pongChParamAddr[TCP3D_DRV_CH_IDX_L2P];

    /* Store pointers for the end of list (PING starts first in the list)*/
    prm = &tcp3dInst->pseudoParamBufPtr[(tcp3dInst->maxCodeBlocks-2)*TCP3D_DRV_LINK_CB];
    if ( tcp3dInst->maxCodeBlocks & 1 )
    {  
        tcp3dInst->endListParam[PING_INDEX] = prm;
        tcp3dInst->endListParam[PONG_INDEX] = prm+TCP3D_DRV_LINK_CB;
    }
    else
    { 
        tcp3dInst->endListParam[PING_INDEX] = prm+TCP3D_DRV_LINK_CB;
        tcp3dInst->endListParam[PONG_INDEX] = prm;
    }

    /**
     * Set interrupt enable/disable mask for channels using the shadow region
     * registers.
     */
    /* Get EDMA Controller shadow registers pointer */
    // NOTE: Removed after the value is set through init sequence
    // tcp3dInst->tpccShadowRegs = &tpcc2Regs->SHADOW[tcp3dInst->edmaRegionId];

    /* REVT channel mask and registers (for PAUSE interrupt) */
    tcp3dInst->pauseChMaskPing = 1 << (tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT] & 0x1f);
    tcp3dInst->pauseChMaskPong = 1 << (tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT] & 0x1f);
    if ( tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT] < 32 )
    {
        tcp3dInst->intEnClrReg[TPCC_REVT_REGS]      = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IECR;
        tcp3dInst->intEnSetReg[TPCC_REVT_REGS]      = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IESR;
        tcp3dInst->clrIntPendReg[TPCC_REVT_REGS]    = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_ICR;
        tcp3dInst->intPendReg[TPCC_REVT_REGS]       = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IPR;
    }
    else
    {
        tcp3dInst->intEnClrReg[TPCC_REVT_REGS]      = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IECRH;
        tcp3dInst->intEnSetReg[TPCC_REVT_REGS]      = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IESRH;
        tcp3dInst->clrIntPendReg[TPCC_REVT_REGS]    = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_ICRH;
        tcp3dInst->intPendReg[TPCC_REVT_REGS]       = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IPRH;
    }
    /* L2P channel mask and registers (for L2P interrupt) */
    tcp3dInst->l2pChMaskPing = 1 << (tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_L2P] & 0x1f);
    tcp3dInst->l2pChMaskPong = 1 << (tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_L2P] & 0x1f);
    if ( tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_L2P] < 32 )
    {
        tcp3dInst->intEnClrReg[TPCC_L2P_REGS]      = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IECR;
        tcp3dInst->intEnSetReg[TPCC_L2P_REGS]      = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IESR;
        tcp3dInst->clrIntPendReg[TPCC_L2P_REGS]    = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_ICR;
        tcp3dInst->intPendReg[TPCC_L2P_REGS]       = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IPR;
    }
    else
    {
        tcp3dInst->intEnClrReg[TPCC_L2P_REGS]      = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IECRH;
        tcp3dInst->intEnSetReg[TPCC_L2P_REGS]      = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IESRH;
        tcp3dInst->clrIntPendReg[TPCC_L2P_REGS]    = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_ICRH;
        tcp3dInst->intPendReg[TPCC_L2P_REGS]       = (uint32_t *) &tcp3dInst->tpccShadowRegs->TPCC_IPRH;
    }

    /* Clear countes */
    tcp3dInst->pingStartCntr = 0;
    tcp3dInst->pongStartCntr = 0;
    tcp3dInst->pingPauseEnCntr = 0;
    tcp3dInst->pingL2pEnCntr = 0;
    tcp3dInst->pingIntr = 0;
    tcp3dInst->pongIntr = 0;
}

static void Tcp3d_resetRuntimeVariables (IN Tcp3d_Instance   *tcp3dInst)
{
    /* Initialize the driver instace run-time variables */
    tcp3dInst->nextCodeBlockIndex       = 0;
    tcp3dInst->pingStop                 = 1;
    tcp3dInst->pongStop                 = 1;
    tcp3dInst->startFlag                = 0;
    tcp3dInst->prevNtfFlag[PING_INDEX]  = 0;
    tcp3dInst->prevNtfFlag[PONG_INDEX]  = 0;
    tcp3dInst->pingLoadCnt              = 0;
    tcp3dInst->pongLoadCnt              = 0;
    tcp3dInst->prevPingOutIdx           = 0;
    tcp3dInst->prevPongOutIdx           = 1;
    tcp3dInst->nextPingInIdx            = 0;
    tcp3dInst->nextPongInIdx            = 1;
    tcp3dInst->pingWrapCheck            = 1;
    tcp3dInst->pongWrapCheck            = 1;
    tcp3dInst->pingLastOutFlag          = 0;
    tcp3dInst->pongLastOutFlag          = 0;
    tcp3dInst->pingFreeCnt              = tcp3dInst->maxPingCbCnt;
    tcp3dInst->pongFreeCnt              = tcp3dInst->maxPongCbCnt;
}

static void Tcp3d_updatePingListVariables ( INOUT  Tcp3d_Instance *inst,
                                            IN     int32_t        pingOutIdx )
{
    int32_t                 indexDiff1;

    /**
     * Load count adjustment is done following the steps described below.
     * 
     * step1 : get index difference between current and previous indexes
     * step2 : convert the index difference to count
     * step3 : reduce the load count by index difference. It is possible that
     *          the diffrence could be negative, which gets corrected after
     *          step4 is completed.
     * step4 : wrap is detected, reduce the load count by maximum one time.
     *          The wrap detection is done either of the cases.
     *              1) when index difference is negative
     *              2) the last block decoding is detected
     * 
     * NOTES:
     *  - At reset/init, checking for wrap is enabled.
     *  - Once the adjustment is done, checking is disabled until the last
     *      block decoding is done.
     */

    /* Adjust the loaded count - PING */
    /* step1 */
    indexDiff1 = ( pingOutIdx - inst->prevPingOutIdx );
    /* step2, step3 */
    inst->pingLoadCnt -= (indexDiff1>>1);
    /* step4 */
    if ( ( (indexDiff1 < 0) || (inst->pingLastOutFlag) ) && 
		    (inst->pingWrapCheck) )
    {
        inst->pingLoadCnt   -= inst->maxPingCbCnt;
        inst->pingWrapCheck  = 0;
    }

    /* update free counts - can be negative */
    inst->pingFreeCnt       = ( inst->maxPingCbCnt - inst->pingLoadCnt );

    /* update previous out index */
    inst->prevPingOutIdx    = pingOutIdx;
}

static void Tcp3d_updatePongListVariables ( INOUT  Tcp3d_Instance *inst,
                                            IN     int32_t        pongOutIdx )
{
    int32_t                 indexDiff2;

    /**
     * Load count adjustment is done following the steps described below.
     * 
     * step1 : get index difference between current and previous indexes
     * step2 : convert the index difference to count
     * step3 : reduce the load count by index difference. It is possible that
     *          the diffrence could be negative, which gets corrected after
     *          step4 is completed.
     * step4 : wrap is detected, reduce the load count by maximum one time.
     *          The wrap detection is done either of the cases.
     *              1) when index difference is negative
     *              2) the last block decoding is detected
     * 
     * NOTES:
     *  - At reset/init, checking for wrap is enabled.
     *  - Once the adjustment is done, checking is disabled until the last
     *      block decoding is done.
     */


    /* Adjust the loaded count - PONG */
    /* step1 */
    indexDiff2 = ( pongOutIdx - inst->prevPongOutIdx );
    /* step2, step3 */
    inst->pongLoadCnt -= (indexDiff2>>1);
    /* step4 */
    if ( ( (indexDiff2 < 0) || (inst->pongLastOutFlag) ) && 
		    (inst->pongWrapCheck) )
    {
        inst->pongLoadCnt   -= inst->maxPongCbCnt;
        inst->pongWrapCheck  = 0;
    }

    /* update free counts - can be negative */
    inst->pongFreeCnt       = ( inst->maxPongCbCnt - inst->pongLoadCnt );

    /* update previous out index */
    inst->prevPongOutIdx    = pongOutIdx;
}

/**
 * @brief   Function to get the physical addresses of all the EDMA3 channels
 *          used in TCP3D driver. 
 */
static EDMA3_DRV_Result Tcp3d_getEdmaChParamAddr(IN Tcp3d_Instance *tcp3dInst)
{ 
    EDMA3_DRV_Result    result = EDMA3_DRV_SOK;    
    int32_t               cnt;

    for ( cnt = 0; cnt < TCP3D_DRV_MAX_CH_PER_PATH; cnt++ )
    {
        /* Get PaRAM address for PING physical channel */ 
        result |= EDMA3_DRV_getPaRAMPhyAddr(tcp3dInst->edmaHnd,
                                            tcp3dInst->pingCh[cnt],
                                            &tcp3dInst->pingChParamAddr[cnt]);
                
        /* Get PaRAM address for PONG physical channel */ 
        result |= EDMA3_DRV_getPaRAMPhyAddr(tcp3dInst->edmaHnd,
                                            tcp3dInst->pongCh[cnt],
                                            &tcp3dInst->pongChParamAddr[cnt]);
    }

    for ( cnt = 0; cnt < (TCP3D_DRV_MAX_LINK_CH>>1); cnt++ )
    {
        /* Get PaRAM address for PING Link channel */ 
        result |= EDMA3_DRV_getPaRAMPhyAddr(tcp3dInst->edmaHnd,
                                            tcp3dInst->pingLinkCh[cnt],
                                            &tcp3dInst->pingLinkChParamAddr[cnt]);
                
        /* Get PaRAM address for PONG link channel */ 
        result |= EDMA3_DRV_getPaRAMPhyAddr(tcp3dInst->edmaHnd,
                                            tcp3dInst->pongLinkCh[cnt],
                                            &tcp3dInst->pongLinkChParamAddr[cnt]);
    }

    return ( result );

} /* end of - Tcp3d_getEdmaChParamAddr() function */

/**
 * @brief   Enabling the Event triggered EDMA3 channels 
 */
static EDMA3_DRV_Result Tcp3d_enableEdmaChannels(Tcp3d_Instance *tcp3dInst)
{
    EDMA3_DRV_Result    result = EDMA3_DRV_SOK;
    
    /* Enable PING channels */
    result |= EDMA3_DRV_enableTransfer( tcp3dInst->edmaHnd,
                                        tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                        EDMA3_DRV_TRIG_MODE_EVENT);

    /* Enable PoNG channels */
    result |= EDMA3_DRV_enableTransfer( tcp3dInst->edmaHnd,
                                        tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT],
                                        EDMA3_DRV_TRIG_MODE_EVENT);

    return ( result );
} /* end of - Tcp3d_enableEdmaChannels() function */ 

/**
 * @brief   Initialize the EDMA channels PaRAM memory with default values
 */
static EDMA3_DRV_Result Tcp3d_initEdmaChParam (IN Tcp3d_Instance  *tcp3dInst)
{
    EDMA3_DRV_PaRAMRegs     paramSet = {0,0,0,0,0,0,0,0,0,0,0,0,0};
    EDMA3_DRV_PaRAMRegs     *prm = &paramSet;
    EDMA3_DRV_Result        status = EDMA3_DRV_SOK;
    CSL_CPINTC_RegsOvly     cpintc0Regs = (CSL_CPINTC_RegsOvly) tcp3dInst->cpIntc0RegsBase;

    /* Channel - REVT 0 (dummy PaRAM) */
    /* chain to REVT 0
       link to ping link cfg channel 
       A-sync
       ACNT = 1
       BCNT = 0
       CCNT = 0
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    /* First set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    prm->opt = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_EARLY,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    prm->opt = 0;
    /* Enable Final transfer completion chain */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* Program the TCC */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Early Trasfer Completion */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);
    /* A Sync Transfer Mode */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
    /* Src & Dest are in INCR modes */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
#endif
    prm->srcAddr    = NULL;
    prm->destAddr   = NULL;
    prm->aCnt       = 1;
    prm->bCnt       = 0;
    prm->cCnt       = 0;
    prm->bCntReload = 0;
    prm->srcBIdx    = 0;
    prm->destBIdx   = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_INCFG]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                prm);

    /* Link Channel - set the reload Link PaRAM */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_REVT],
                                prm);

    /* Copy to use in reset function */
    Tcp3d_memcpy(&tcp3dInst->revtPrm[PING_INDEX], prm, sizeof(EDMA3_DRV_PaRAMRegs));

    /* Channel - REVT 1 (dummy PaRAM) */
    /* chain to REVT 1
       link to ping link cfg channel 
       A-sync
       ACNT = 1
       BCNT = 1
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_INCFG]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT],
                                prm);

    /* Link Channel - set the reload Link PaRAM */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_REVT],
                                prm);

    /* Copy to use in reset */
    Tcp3d_memcpy(&tcp3dInst->revtPrm[PONG_INDEX], prm, sizeof(EDMA3_DRV_PaRAMRegs));

    /* Channel - L2P 0 */
    /* chain to REVT 0
       link to ping link l2p channel 
       AB-sync
       ACNT = 32
       BCNT = 4
       CCNT = pingNumCBs
       scrBIDX = 32
       desBIDX = 32
       scrCIDX = 32*4*2
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    /* Set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    prm->opt = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_EN,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_AB,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    prm->opt = 0;
    /* Enable Intermediate & Final transfer completion chain */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_ITCCHEN_SHIFT);
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* Program the TCC */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* AB Sync Transfer Mode */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
    /* Src & Dest are in INCR modes */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
#endif
    prm->srcAddr    = NULL;
    prm->destAddr   = (uint32_t)(tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_INCFG]);
    prm->aCnt       = 32;
    prm->bCnt       = TCP3D_DRV_LINK_CB;
    prm->cCnt       = NULL;
    prm->bCntReload = TCP3D_DRV_LINK_CB;
    prm->srcBIdx    = 32;
    prm->destBIdx   = 32;
    prm->srcCIdx    = (32<<1)*TCP3D_DRV_LINK_CB;
    prm->destCIdx   = 0;
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_L2P]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_L2P],
                                prm);

    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_L2P],
                                prm);

    /* Copy to use in reset function */
    Tcp3d_memcpy(&tcp3dInst->l2pPrm[PING_INDEX], prm, sizeof(EDMA3_DRV_PaRAMRegs));

    /* Channel - L2P 1 */
    /* chain to REVT 1
       link to ping link l2p channel 
       AB-sync
       ACNT = 32
       BCNT = 4
       CCNT = pingNumCBs
       scrBIDX = 32
       desBIDX = 32
       scrCIDX = 32*4*2
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    prm->destAddr   = (uint32_t)(tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_INCFG]);
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_L2P]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_L2P],
                                prm);

    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_L2P],
                                prm);

    /* Copy to use in reset function */
    Tcp3d_memcpy(&tcp3dInst->l2pPrm[PONG_INDEX], prm, sizeof(EDMA3_DRV_PaRAMRegs));

    /* Link Channel - pause 0 */
    /* chain to REVT 0
       link to ping link revt channel 
       AB-sync
       ACNT = 1
       BCNT = 2
       CCNT = 1  
       scrBIDX = offset b/w tcp3dInst->pauseState & tcp3dInst->constantOne
       scrCIDX = 0
       desBIDX = offet b/w tcp3dInst->state & tcp3dInst->pingStop
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    /* Set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    prm->opt = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_DIS,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_EN,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_AB,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    prm->opt = 0;
    /* Enable Intermediate & Final transfer completion interrupt */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);
    /* Program the TCC */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* AB Sync Transfer Mode */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
    /* Src & Dest are in INCR modes */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
#endif
    prm->srcAddr    = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->constantOne);
    prm->destAddr   = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->pingStop);
    prm->aCnt       = 1;
    prm->bCnt       = 2;
    prm->cCnt       = 1;
    prm->bCntReload = 0;
    prm->srcBIdx    = ((uint8_t *)&tcp3dInst->pauseState - &tcp3dInst->constantOne);
    prm->destBIdx   = ((uint8_t *)&tcp3dInst->state - &tcp3dInst->pingStop);
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_REVT]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_PAUSE],
                                prm);

    /* Link Channel - pause 1 */
    /* chain to REVT 1
       link to ping link revt channel 
       AB-sync
       ACNT = 1
       BCNT = 2
       CCNT = 1  
       scrBIDX = offset b/w tcp3dInst->pauseState & tcp3dInst->constantOne
       scrCIDX = 0
       desBIDX = offet b/w tcp3dInst->state & tcp3dInst->pingStop
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    prm->destAddr   = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->pongStop);
    prm->destBIdx   = ((uint8_t *)&tcp3dInst->state - &tcp3dInst->pongStop);
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_REVT]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_PAUSE],
                                prm);

    /* Link Channel - INT 0 (notification PaRAM) */
    /* chain to REVT 0
       link to ping link cfg channel 
       A-sync
       ACNT = 4
       BCNT = 1
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    /* First set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    prm->opt = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_EARLY,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    prm->opt = 0;
    /* Enable Final transfer completion chain */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* Program the TCC */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Early Trasfer Completion */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);
    /* A Sync Transfer Mode */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
    /* Src & Dest are in INCR modes */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
#endif
    prm->srcAddr    = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->notificationEventNum);
    prm->destAddr   = (uint32_t)(&cpintc0Regs->STATUS_SET_INDEX_REG);
    prm->aCnt       = 4;
    prm->bCnt       = 1;
    prm->cCnt       = 1;
    prm->bCntReload = 0;
    prm->srcBIdx    = 0;
    prm->destBIdx   = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;
    prm->linkAddr   = tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_PAUSE];

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_NTF],
                                prm);

    prm->linkAddr   = tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_INCFG];
    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_NTFD],
                                prm);

    /* Link Channel - INT 1 (notification PaRAM) */
    /* chain to REVT 1
       link to ping link cfg channel 
       A-sync
       ACNT = 4
       BCNT = 1
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    prm->linkAddr   = tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_PAUSE];

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_NTF],
                                prm);

    /* Now, write the PaRAM Set. */
    prm->linkAddr   = tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_INCFG];
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_NTFD],
                                prm);

    /* Link Channel - Wrap 0 */
    /* chain to REVT 0
       link to ping link l2p channel 
       A-sync
       ACNT = 4
       BCNT = 1
       CCNT = 1  
       scrBIDX = 0
       scrCIDX = 0
       desBIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    /* Set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    prm->opt = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    prm->opt = 0;
    /* Enable Final transfer completion chain */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* Program the TCC */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* A Sync Transfer Mode */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
    /* Src & Dest are in INCR modes */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
#endif
    prm->srcAddr    = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->maxPingCbCnt);
    prm->destAddr   = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->pingLastOutFlag);
    prm->aCnt       = 4;
    prm->bCnt       = 1;
    prm->cCnt       = 1;
    prm->bCntReload = 0;
    prm->srcBIdx    = 0;
    prm->destBIdx   = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_PAUSE]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_WRAP],
                                prm);

    /* Link Channel - Wrap 1 */
    /* chain to REVT 1
       link to ping link l2p channel 
       A-sync
       ACNT = 4
       BCNT = 1
       CCNT = 1  
       scrBIDX = 0
       scrCIDX = 0
       desBIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    prm->srcAddr    = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->maxPongCbCnt);
    prm->destAddr   = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->pongLastOutFlag);
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_PAUSE]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_WRAP],
                                prm);

                                
    /* Link Channel - NEXTCB Dummy 0 (LINK_CH_IDX_NEXTCB_DUMMY PaRAM) */
    /* chain to L2P
       link to ping dummy REVT channel 
       A-sync
       ACNT = 1
       BCNT = 0
       CCNT = 0
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    /* First set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    prm->opt = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_L2P],
                                    CSL_EDMA3_TCC_EARLY,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    prm->opt = 0;
    /* Enable Final transfer completion chain */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* Program the TCC */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_L2P]);
    /* Early Trasfer Completion */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);
    /* A Sync Transfer Mode */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
    /* Src & Dest are in INCR modes */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
#endif
    prm->srcAddr    = NULL;
    prm->destAddr   = NULL;
    prm->aCnt       = 1;
    prm->bCnt       = 0;
    prm->cCnt       = 0;
    prm->bCntReload = 0;
    prm->srcBIdx    = 0;
    prm->destBIdx   = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_REVT]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_NEXTCB_DUMMY],
                                prm);

                                
    /* Link Channel - NEXTCB 1 (LINK_CH_IDX_NEXTCB_DUMMY PaRAM) */
    /* chain to L2P
       link to pong dummy REVT channel
       A-sync
       ACNT = 1
       BCNT = 1
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_L2P]);
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_REVT]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_NEXTCB_DUMMY],
                                prm);
                                

    /* Link Channel - NEXTCB Notify Dummy 0 (LINK_CH_IDX_NEXTCB_NTFD PaRAM) */
    /* chain to L2P
       link to NotifyD channel 
       A-sync
       ACNT = 1
       BCNT = 0
       CCNT = 0
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    /* First set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    prm->opt = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_L2P],
                                    CSL_EDMA3_TCC_EARLY,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    prm->opt = 0;
    /* Enable Final transfer completion chain */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* Program the TCC */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_L2P]);
    /* Early Trasfer Completion */
    prm->opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);
    /* A Sync Transfer Mode */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
    /* Src & Dest are in INCR modes */
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    prm->opt &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
#endif
    prm->srcAddr    = NULL;
    prm->destAddr   = NULL;
    prm->aCnt       = 1;
    prm->bCnt       = 0;
    prm->cCnt       = 0;
    prm->bCntReload = 0;
    prm->srcBIdx    = 0;
    prm->destBIdx   = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_NTFD]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_NEXTCB_NTFD],
                                prm);

                                
    /* Link Channel - NEXTCB Notify Dummy 1 (LINK_CH_IDX_NEXTCB_NTFD PaRAM) */
    /* chain to L2P
       link to NotifyD channel
       A-sync
       ACNT = 1
       BCNT = 1
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Fill the PaRAM Set with transfer specific information */
    CSL_FINS(prm->opt, TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_L2P]);
    prm->linkAddr   = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_NTFD]);

    /* Now, write the PaRAM Set. */
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_NEXTCB_NTFD],
                                prm);																																															

    return ( status );

} /* end of - Tcp3d_initEdmaChParam() function */

/**
 * @brief   Resets the the EDMA channels PaRAM memory with default values
 */
static EDMA3_DRV_Result Tcp3d_resetEdmaChParam( IN Tcp3d_Instance  *tcp3dInst,
                                                IN uint32_t          pingNumCBs,
                                                IN uint32_t          pongNumCBs)
{
    EDMA3_DRV_Result        status = EDMA3_DRV_SOK;
    EDMA3_DRV_PaRAMRegs     *prm;

    /* L2P 0 */
    prm = &tcp3dInst->l2pPrm[PING_INDEX];
    prm->srcAddr    = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->pseudoParamBufPtr[0]);
    prm->cCnt       = pingNumCBs;
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_L2P],
                                prm);

    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingLinkCh[LINK_CH_IDX_L2P],
                                prm);

    /* L2P 1 */
    prm = &tcp3dInst->l2pPrm[PONG_INDEX];
    prm->srcAddr    = L2GLBMAP(tcp3dInst->coreId, &tcp3dInst->pseudoParamBufPtr[TCP3D_DRV_LINK_CB]);
    prm->cCnt       = pongNumCBs;
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_L2P],
                                prm);

    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongLinkCh[LINK_CH_IDX_L2P],
                                prm);

    /* REVT 0 */
    prm = &tcp3dInst->revtPrm[PING_INDEX];
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                prm);

    /* REVT 1 */
    prm = &tcp3dInst->revtPrm[PONG_INDEX];
    status |= EDMA3_DRV_setPaRAM(tcp3dInst->edmaHnd,
                                tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT],
                                prm);

    return ( status );

} /* end of - Tcp3d_resetEdmaChParam() function */

/**
 * @brief   Initialize the Pseudo PaRAM memory with default values
 */
static void Tcp3d_initPseudoParam ( IN  Tcp3d_Instance  *tcp3dInst,
                                    IN  uint32_t          codeBlocks,
                                    IN  Tcp3d_Config    *pingConfig,
                                    IN  Tcp3d_Config    *pongConfig)
{
    int32_t                   cnt, flag;
    EDMA3_DRV_PaRAMRegs     prmSet[TCP3D_DRV_LINK_CB];
    EDMA3_DRV_PaRAMRegs     *prm;
    uint8_t                   mode = tcp3dInst->mode;
    uint32_t                  incfgOpt[2];
    uint32_t                  incfgLink[2];
    uint32_t                  incfgStartAddress[2];
    uint32_t                  stsOpt[2];
    uint32_t                  stsLink[2];
    uint32_t                  stsStartAddress[2];
    uint32_t                  hdOpt[2];
    uint32_t                  hdLink[2];
    uint32_t                  hdStartAddress[2];
    uint32_t                  llrOpt[2];
    uint32_t                  llrLink[2];
    uint32_t                  llrStartAddress[2];
    uint32_t                  sdOpt[2];
    uint32_t                  sdLink[2];
    uint32_t                  sdStartAddress[2];

    incfgStartAddress[PING_INDEX] = pingConfig->inCfgStart;
    incfgStartAddress[PONG_INDEX] = pongConfig->inCfgStart;

    incfgLink[PING_INDEX] = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_LLR]);
    incfgLink[PONG_INDEX] = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_LLR]);

    llrStartAddress[PING_INDEX] = pingConfig->llrStart;
    llrStartAddress[PONG_INDEX] = pongConfig->llrStart;

    llrLink[PING_INDEX] = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_HD]);
    llrLink[PONG_INDEX] = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_HD]);

    hdStartAddress[PING_INDEX] = pingConfig->hdStart;
    hdStartAddress[PONG_INDEX] = pongConfig->hdStart;

    hdLink[PING_INDEX] = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_PAUSE]);
    hdLink[PONG_INDEX] = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_PAUSE]);

    stsStartAddress[PING_INDEX] = pingConfig->stsStart;
    stsStartAddress[PONG_INDEX] = pongConfig->stsStart;

    stsLink[PING_INDEX] = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_PAUSE]);
    stsLink[PONG_INDEX] = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_PAUSE]);

    sdStartAddress[PING_INDEX] = pingConfig->sdStart;
    sdStartAddress[PONG_INDEX] = pongConfig->sdStart;

    sdLink[PING_INDEX] = (0xFFFFu) & (tcp3dInst->pingLinkChParamAddr[LINK_CH_IDX_PAUSE]);
    sdLink[PONG_INDEX] = (0xFFFFu) & (tcp3dInst->pongLinkChParamAddr[LINK_CH_IDX_PAUSE]);

    /**
     * INCFG Ping & Pong - fill all the initial values
     */
    /* chain to REVT
       link to link llr channel 
       A-sync
       ACNT = 60
       BCNT = 1
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    incfgOpt[PING_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );

    incfgOpt[PONG_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    incfgOpt[PING_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    incfgOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    incfgOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(incfgOpt[PING_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Final transfer completion chain */
    incfgOpt[PING_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* A Sync Transfer Mode */
    incfgOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

    incfgOpt[PONG_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    incfgOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    incfgOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(incfgOpt[PONG_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Final transfer completion chain */
    incfgOpt[PONG_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* AB Sync Transfer Mode */
    incfgOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
#endif
    prm = &prmSet[LINK_CH_IDX_INCFG];
    prm->srcAddr    = NULL;
    prm->aCnt       = 60;
    prm->bCnt       = 1;
    prm->cCnt       = 1;
    prm->bCntReload = 0;
    prm->srcBIdx    = 0;
    prm->destBIdx   = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;

    /**
     * LLR Ping & Pong - fill all the initial values
     */
    /* chain to REVT
       link to link hd channel 
       AB-sync
       ACNT = NULL (updated during enque operation)
       BCNT = NULL (updated during enque operation)
       CCNT = NULL (updated during enque operation)
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    llrOpt[PING_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_DIS,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_AB,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );

    llrOpt[PONG_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_DIS,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_AB,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    llrOpt[PING_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    llrOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    llrOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(llrOpt[PING_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Intermediate & Final transfer completion chain */
    llrOpt[PING_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_ITCCHEN_SHIFT);
    /* AB Sync Transfer Mode */
    llrOpt[PING_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

    llrOpt[PONG_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    llrOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    llrOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(llrOpt[PONG_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Intermediate & Final transfer completion chain */
    llrOpt[PONG_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_ITCCHEN_SHIFT);
    /* AB Sync Transfer Mode */
    llrOpt[PONG_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
#endif
    prm = &prmSet[LINK_CH_IDX_LLR];
    prm->srcAddr    = NULL;
    prm->aCnt       = NULL;
    prm->bCntReload = 0;
    if ( ( mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_LTE) ||
         ( mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_WIMAX ) )
    {
        prm->bCnt       = 2;
        prm->cCnt       = 3;
        prm->srcBIdx    = NULL;
        prm->destBIdx   = 0x1000;
        prm->srcCIdx    = NULL;
        prm->destCIdx   = 0x2000;
    }
    else
    {
        prm->bCnt       = 3;
        prm->cCnt       = 1;
        prm->srcBIdx    = NULL;
        prm->destBIdx   = 0x2000;
        prm->srcCIdx    = 0;
        prm->destCIdx   = 0;
        llrOpt[PING_INDEX]      &= ~(1 << CSL_TPCC_PARAM_OPT_ITCCHEN_SHIFT);
        llrOpt[PONG_INDEX]      &= ~(1 << CSL_TPCC_PARAM_OPT_ITCCHEN_SHIFT);
    }

    /**
     * HD Ping & Pong - fill all the initial values
     */
    /* chain to REVT
       link to link pause channel 
       A-sync
       ACNT = NULL (updated during enque operation)
       BCNT = 1
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    hdOpt[PING_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );

    hdOpt[PONG_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    hdOpt[PING_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    hdOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    hdOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(hdOpt[PING_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Intermediate & Final transfer completion chain */
    hdOpt[PING_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* A Sync Transfer Mode */
    hdOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

    hdOpt[PONG_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    hdOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    hdOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(hdOpt[PONG_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Intermediate & Final transfer completion chain */
    hdOpt[PONG_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* A Sync Transfer Mode */
    hdOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
#endif
    prm = &prmSet[LINK_CH_IDX_HD];
    prm->destAddr   = NULL;
    prm->aCnt       = NULL;
    prm->bCnt       = 1;
    prm->cCnt       = 1;
    prm->bCntReload = 0;
    prm->srcBIdx    = 0;
    prm->destBIdx   = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;

    /**
     * STS Ping & Pong - fill all the initial values
     */
    /* chain to REVT
       link to link sd channel 
       A-sync
       ACNT = 12
       BCNT = 1
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    stsOpt[PING_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );

    stsOpt[PONG_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    stsOpt[PING_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    stsOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    stsOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(stsOpt[PING_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Final transfer completion chain */
    stsOpt[PING_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* A Sync Transfer Mode */
    stsOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

    stsOpt[PONG_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    stsOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    stsOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(stsOpt[PONG_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Final transfer completion chain */
    stsOpt[PONG_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* A Sync Transfer Mode */
    stsOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
#endif
    prm = &prmSet[LINK_CH_IDX_STS];
    prm->destAddr   = NULL;
    prm->aCnt       = 12;
    prm->bCnt       = 1;
    prm->cCnt       = 1;
    prm->bCntReload = 0;
    prm->srcBIdx    = 0;
    prm->destBIdx   = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;

    /**
     * SD Ping & Pong - fill all the initial values
     */
    /* chain to REVT
       link to link pause channel 
       A-sync / AB-sync based on mode
       ACNT = NULL (updated during enque operation)
       BCNT = 1/3 based on mode 
       CCNT = 1
       scrBIDX = 0
       desBIDX = 0
       scrCIDX = 0
       desCIDX = 0 */
    /* Set OPT field with appropriate values */
#if TCP3D_DRV_USE_CSL_EDMA3_OPT_MAKE
    sdOpt[PING_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );

    sdOpt[PONG_INDEX] = CSL_EDMA3_OPT_MAKE ( CSL_EDMA3_ITCCH_DIS,
                                    CSL_EDMA3_TCCH_EN,
                                    CSL_EDMA3_ITCINT_DIS,
                                    CSL_EDMA3_TCINT_DIS,
                                    tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT],
                                    CSL_EDMA3_TCC_NORMAL,
                                    CSL_EDMA3_FIFOWIDTH_NONE,
                                    CSL_EDMA3_STATIC_DIS,
                                    CSL_EDMA3_SYNC_A,
                                    CSL_EDMA3_ADDRMODE_INCR,
                                    CSL_EDMA3_ADDRMODE_INCR );
#else
    sdOpt[PING_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    sdOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    sdOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(sdOpt[PING_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pingCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Intermediate & Final transfer completion chain */
    sdOpt[PING_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* A Sync Transfer Mode */
    sdOpt[PING_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

    sdOpt[PONG_INDEX] = 0;
    /* Src & Dest are in INCR modes */
    sdOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SAM_SHIFT);
    sdOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_DAM_SHIFT);
    /* Program the TCC */
    CSL_FINS(sdOpt[PONG_INDEX], TPCC_PARAM_OPT_TCC, tcp3dInst->pongCh[TCP3D_DRV_CH_IDX_REVT]);
    /* Enable Intermediate & Final transfer completion chain */
    sdOpt[PONG_INDEX] |= (1 << CSL_TPCC_PARAM_OPT_TCCHEN_SHIFT);
    /* A Sync Transfer Mode */
    sdOpt[PONG_INDEX] &= ~(1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
#endif
    prm = &prmSet[LINK_CH_IDX_SD];
    prm->destAddr   = NULL;
    prm->aCnt       = NULL;
    prm->cCnt       = 1;
    prm->bCntReload = 0;
    prm->srcCIdx    = 0;
    prm->destCIdx   = 0;
    if ( ( mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_LTE) ||
         ( mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_WIMAX ) )
    {
        prm->bCnt       = 3;
        prm->srcBIdx    = 0x2000;
        prm->destBIdx   = NULL;
        sdOpt[PING_INDEX]       |= (1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
        sdOpt[PONG_INDEX]       |= (1 << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);
    }
    else
    {
        prm->bCnt       = 1;
        prm->srcBIdx    = 0;
        prm->destBIdx   = 0;
    }

    for ( cnt = 0; cnt < codeBlocks; cnt++ )
    {
        flag = cnt & 1;

        prm = &prmSet[LINK_CH_IDX_INCFG];
        prm->opt        = incfgOpt[flag];
        prm->destAddr   = incfgStartAddress[flag];
        prm->linkAddr   = incfgLink[flag];

        prm = &prmSet[LINK_CH_IDX_LLR];
        prm->opt        = llrOpt[flag];
        prm->destAddr   = llrStartAddress[flag];
        prm->linkAddr   = llrLink[flag];

        prm = &prmSet[LINK_CH_IDX_STS];
        prm->opt        = stsOpt[flag];
        prm->srcAddr    = stsStartAddress[flag];
        prm->linkAddr   = stsLink[flag];

        prm = &prmSet[LINK_CH_IDX_HD];
        prm->opt        = hdOpt[flag];
        prm->srcAddr    = hdStartAddress[flag];
        prm->linkAddr   = hdLink[flag];
    
        prm = &prmSet[LINK_CH_IDX_SD];
        prm->opt        = sdOpt[flag];
        prm->srcAddr    = sdStartAddress[flag];
        prm->linkAddr   = sdLink[flag];

        Tcp3d_memcpy(&tcp3dInst->pseudoParamBufPtr[cnt*TCP3D_DRV_LINK_CB], &prmSet[0], 32*TCP3D_DRV_LINK_CB);

    } /* end of - for ( cnt = 0; cnt < codeBlocks; cnt++ ) */

    /* Store the lastOpt & lastLink values for use in enque function */
    tcp3dInst->resetHdOpt[PING_INDEX]    = hdOpt[PING_INDEX];
    tcp3dInst->resetHdOpt[PONG_INDEX]    = hdOpt[PONG_INDEX];
    tcp3dInst->resetHdLink[PING_INDEX]   = hdLink[PING_INDEX];
    tcp3dInst->resetHdLink[PONG_INDEX]   = hdLink[PONG_INDEX];

    tcp3dInst->resetStsOpt[PING_INDEX]    = stsOpt[PING_INDEX];
    tcp3dInst->resetStsOpt[PONG_INDEX]    = stsOpt[PONG_INDEX];
    tcp3dInst->resetStsLink[PING_INDEX]   = stsLink[PING_INDEX];
    tcp3dInst->resetStsLink[PONG_INDEX]   = stsLink[PONG_INDEX];

    tcp3dInst->resetSdOpt[PING_INDEX]    = sdOpt[PING_INDEX];
    tcp3dInst->resetSdOpt[PONG_INDEX]    = sdOpt[PONG_INDEX];
    tcp3dInst->resetSdLink[PING_INDEX]   = sdLink[PING_INDEX];
    tcp3dInst->resetSdLink[PONG_INDEX]   = sdLink[PONG_INDEX];
} /* end of - Tcp3d_initPseudoParam() function */

/**
 * @brief   Resets Pseudo PaRAM memory with default values
 */
static void Tcp3d_resetPseudoParam (IN  Tcp3d_Instance  *tcp3dInst,
                                    IN  uint32_t          codeBlocks)
{
    int32_t                   cnt;
    EDMA3_DRV_PaRAMRegs     *prm1 = &tcp3dInst->pseudoParamBufPtr[LINK_CH_IDX_HD];
    EDMA3_DRV_PaRAMRegs     *prm2 = &tcp3dInst->pseudoParamBufPtr[LINK_CH_IDX_STS];
    EDMA3_DRV_PaRAMRegs     *prm3 = &tcp3dInst->pseudoParamBufPtr[LINK_CH_IDX_SD];

    for ( cnt = 0; cnt < codeBlocks; cnt++ )
    {
        prm1->opt        = tcp3dInst->resetHdOpt[cnt & 1];
        prm1->linkAddr   = tcp3dInst->resetHdLink[cnt & 1];
        prm2->opt        = tcp3dInst->resetStsOpt[cnt & 1];
        prm2->linkAddr   = tcp3dInst->resetStsLink[cnt & 1];
        prm3->opt        = tcp3dInst->resetSdOpt[cnt & 1];
        prm3->linkAddr   = tcp3dInst->resetSdLink[cnt & 1];

        prm1 +=TCP3D_DRV_LINK_CB;
        prm2 +=TCP3D_DRV_LINK_CB;
        prm3 +=TCP3D_DRV_LINK_CB;
    } /* end of - for ( cnt = 0; cnt < codeBlocks; cnt++ ) */
} /* end of - Tcp3d_resetPseudoParam() function */

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version information of the TCP3D Driver.
 *
 *  @retval
 *      Version Information.
 */
uint32_t Tcp3d_getVersion (void)
{
    return TCP3D_DRV_VERSION_ID;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version string for the TCP3D Driver.
 *
 *  @retval
 *      Version String.
 */
const char* Tcp3d_getVersionStr (void)
{
    return Tcp3dDrvVersionStr;
}

/* end of file */
