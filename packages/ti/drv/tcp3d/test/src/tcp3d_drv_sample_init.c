/*
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
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



#include "tcp3d_drv_sample.h"

Tcp3d_MemBuffer         *bufs;
Int16                   nbufs;

/**
 * NOTE: All the configuration values whether they are used in the current
 *      driver or not. Unused values are kept as place holders for future use.
 */ 
Void fillConfig(Tcp3d_InitParams *drvInitParams, UInt32 perId)
{
    UInt32      baseDataRegs;

    if ( perId < CSL_TCP3D_PER_CNT )
    {
        /* Set the notification Event number of CP_INTC0 */
        drvInitParams->notificationEventNum     = getNotifyEventNum(perId);

        /* Set the Control Register base address */
        drvInitParams->tcp3dCfgRegs             = (CSL_Tcp3d_cfgRegs *) getTcp3dCfgRegsBase(perId);

        /* Set REVT channel numbers */
        drvInitParams->pingConfig.revtCh        = getRevt0ChannelNum(perId);
        drvInitParams->pongConfig.revtCh        = getRevt1ChannelNum(perId);

        /* Set the TCP3D PING addresses */
        baseDataRegs = getTcp3dDataRegsBase(perId);
        drvInitParams->pingConfig.inCfgStart    = baseDataRegs + CSL_TCP3D_DMA_TCP3D_IC_CFG0_P0_OFFSET;
        drvInitParams->pingConfig.llrStart      = baseDataRegs + CSL_TCP3D_DMA_TCP3D_SYS_P0_OFFSET;
        drvInitParams->pingConfig.interStart    = baseDataRegs + CSL_TCP3D_DMA_TCP3D_INTER_P0_OFFSET;
        drvInitParams->pingConfig.hdStart       = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_HD0_OFFSET;
        drvInitParams->pingConfig.stsStart      = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_STS0_P0_OFFSET;
        drvInitParams->pingConfig.sdStart       = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_SO0_OFFSET;
    
        /* Set the TCP3D PONG addresses */
        drvInitParams->pongConfig.inCfgStart    = baseDataRegs + CSL_TCP3D_DMA_TCP3D_IC_CFG0_P1_OFFSET;
        drvInitParams->pongConfig.llrStart      = baseDataRegs + CSL_TCP3D_DMA_TCP3D_SYS_P1_OFFSET;
        drvInitParams->pongConfig.interStart    = baseDataRegs + CSL_TCP3D_DMA_TCP3D_INTER_P1_OFFSET;
        if ( drvInitParams->ctrlParams.doubleBuf == CSL_TCP3D_CFG_TCP3_MODE_IN_MEM_DB_EN_ENABLE )
        {
            drvInitParams->pongConfig.hdStart   = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_HD0_OFFSET;
            drvInitParams->pongConfig.sdStart   = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_SO0_OFFSET;
            drvInitParams->pongConfig.stsStart  = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_STS0_P0_OFFSET;
        }
        else
        {
            drvInitParams->pongConfig.hdStart   = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_HD1_OFFSET;
            drvInitParams->pongConfig.sdStart   = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_SO1_OFFSET;
            drvInitParams->pongConfig.stsStart  = baseDataRegs + CSL_TCP3D_DMA_TCP3D_OUT_STS0_P1_OFFSET;
        }
    }
    else
    {
        System_printf("Wrong Instance ID passed\n");
        System_exit(0);
    }
}

Tcp3d_Instance* tcp3dSampleInit(
                    IHeap_Handle        dataHeap,
                    UInt8               instNum,
                    UInt32              testMaxBlocks,
                    UInt32              testMode,
                    UInt32              testDoubleBuffer,
                    UInt32              testLteCrcSel,
                    UInt32              dspCoreID,
                    EDMA3_DRV_Handle    hEdma,
                    UInt32              tpccRegionUsed,
                    EDMA_CONFIG         *edmaConfig,
                    Tcp3d_Result        *errCode)
{
    Tcp3d_Result        tcp3dResult = TCP3D_DRV_NO_ERR;
    Tcp3d_SizeCfg       sizeCfg;
    Tcp3d_InitParams    drvInitParams;
    Int32               cnt;
    UInt32              numBytes;
    UInt8               align;

    /*
     *  Setup TCP3D Driver Initialization Sequence
     */
    /* Step 1: Set parameters in the sizeCfg structure for TCP3D driver
     *          memory allocation. This will be used with the driver functions
     *          Tcp3d_getNumBuf() & Tcp3d_getBufDesc() */
    sizeCfg.maxCodeBlocks   = testMaxBlocks;
    sizeCfg.mode            = testMode;

    /* Step 2: Get Number of buffers required for TCP3D Driver */
    tcp3dResult = Tcp3d_getNumBuf(&sizeCfg, &nbufs);
    if ( tcp3dResult != TCP3D_DRV_NO_ERR )
    {
        System_printf("Get Num Bufs failed\n");
    }
    else
    {
        System_printf("\t Tcp3d_getNumBuf() passed\n");
    }

    /* Step 3: Allocate memory for buffer descriptor structure */
    numBytes = nbufs * sizeof (Tcp3d_MemBuffer);
    bufs = (Tcp3d_MemBuffer *) Memory_alloc(dataHeap, numBytes, 0, NULL);       
    if ( bufs == NULL )
    {
        System_printf("Memory allocation failed !!! (DRV MEMBUFS)\n");
        System_exit(0);
    }

    /* Step 4: Get TCP3D Driver buffer descriptor requirements */
    tcp3dResult = Tcp3d_getBufDesc(&sizeCfg, bufs);
    if ( tcp3dResult != TCP3D_DRV_NO_ERR )
    {
        System_printf("Get Buf Descriptor failed\n");
    }
    else
    {
        System_printf("\t Tcp3d_getBufDesc() passed\n");
    }

    /* Step 5: Allocate memory for buffers based on the requirements given */
    /*
     * NOTE:
     *  1) The memory class type is NOT used for allocation.
     *  2) The memory allocation is always done from the data Heap. See the
     *      BIOS config file more details (drvHeap).
     */
    for (cnt = 0; cnt < nbufs; ++cnt)
    {
        numBytes = bufs[cnt].size;
        align = 1<<bufs[cnt].log2align;
        
        bufs[cnt].base = (Tcp3d_MemBuffer *) Memory_alloc ( dataHeap,
                                                            numBytes,
                                                            align,
                                                            NULL);

        if ( bufs[cnt].base == NULL )
        {
            System_printf("Memory allocation failed !!! (DRV BUF)\n");
            System_exit(0);
        }
    }

    /* Step 7: Set TCP3D Driver init parameters */
    drvInitParams.coreID                    = dspCoreID;
    drvInitParams.maxCodeBlocks             = testMaxBlocks;
    drvInitParams.instNum                   = instNum;

    /* Set the CP_INTC0 registers base address */
    drvInitParams.cpIntc0RegsBase          = (void *) getCpIntc0RegsBase();

    /* Set the EDMA variables/resoureces */
    drvInitParams.edmaHnd                  = hEdma;
    drvInitParams.edmaRegionId             = tpccRegionUsed;
    drvInitParams.edma3ShadowRegsBase      = (CSL_TPCC_ShadowRegs *) getEdma3ShadowRegsBase(tpccRegionUsed);
    for (cnt = 0; cnt < TCP3D_DRV_MAX_CH_PER_PATH; ++cnt)
    {
        drvInitParams.pingCh[cnt]          = edmaConfig->pingChRes[cnt].chNo;
        drvInitParams.pongCh[cnt]          = edmaConfig->pongChRes[cnt].chNo;
    }
    for (cnt = 0; cnt < TCP3D_DRV_MAX_LINK_CH; ++cnt)
    {
        drvInitParams.linkCh[cnt]          = edmaConfig->linkChRes[cnt].chNo;
    }

    /* Set the Control Register parameters */
    drvInitParams.ctrlParams.mode          = testMode;
    drvInitParams.ctrlParams.doubleBuf     = testDoubleBuffer;
    drvInitParams.ctrlParams.intTable      = CSL_TCP3D_CFG_TCP3_MODE_ITG_EN_ENABLE;
    drvInitParams.ctrlParams.autoTrig      = CSL_TCP3D_CFG_TCP3_MODE_AUTO_TRIG_EN_ENABLE;
    drvInitParams.ctrlParams.errIgnore     = CSL_TCP3D_CFG_TCP3_MODE_ERROR_IGNORE_EN_DONT_STOP;
    drvInitParams.ctrlParams.lteCrcSel     = testLteCrcSel;
#ifdef _LITTLE_ENDIAN
    drvInitParams.ctrlParams.endInt        = CSL_TCP3D_CFG_TCP3_END_ENDIAN_INTR_32_BIT_PACKED;
    drvInitParams.ctrlParams.endInData     = CSL_TCP3D_CFG_TCP3_END_ENDIAN_INDATA_32_BIT_PACKED;
#else
    drvInitParams.ctrlParams.endInt        = CSL_TCP3D_CFG_TCP3_END_ENDIAN_INTR_16_BIT_NATIVE;
    drvInitParams.ctrlParams.endInData     = CSL_TCP3D_CFG_TCP3_END_ENDIAN_INDATA_8_BIT_NATIVE;
#endif
    drvInitParams.ctrlParams.exeP0cmd      = CSL_TCP3D_CFG_TCP3_EXE_P0_EXE_CMD_ENABLE;
    if ( testDoubleBuffer != CSL_TCP3D_CFG_TCP3_MODE_IN_MEM_DB_EN_ENABLE )
        drvInitParams.ctrlParams.exeP1cmd      = CSL_TCP3D_CFG_TCP3_EXE_P1_EXE_CMD_ENABLE;

    fillConfig(&drvInitParams, instNum);

    /* Step 8: Call the TCP3D Driver init function */
    tcp3dResult = Tcp3d_init ( bufs, &drvInitParams);

    if ( tcp3dResult != TCP3D_DRV_NO_ERR)
    {
        System_printf("TCP3D Driver Init failed\n");
        System_exit(0);
    }
    else
    {
        System_printf("\t Tcp3d_init() passed\n");
    }

    *errCode = tcp3dResult;

    /* Initialize the TCP3D driver instance variable */
    return ((Tcp3d_Instance* )bufs[TCP3D_DRV_INST_BUFN].base);
}

Tcp3d_Result tcp3dSampleDeinit( IHeap_Handle    dataHeap,
                                UInt8           instNum,
                                Tcp3d_Instance  *tcp3dInst)
{
    Int32               cnt;
    Tcp3d_Result        tcp3dResult = TCP3D_DRV_NO_ERR;

    tcp3dResult = Tcp3d_deInit(tcp3dInst);

    if ( tcp3dResult != TCP3D_DRV_NO_ERR)
    {
        System_printf("TCP3D Driver De-Init failed\n");
        System_exit(0);
    }
    else
    {
        System_printf("\t Tcp3d_deInit() passed\n");
    }

    /* Free memory allocated for TCP3D Driver Initialization sequence */
    /* NOTE:
     *  It is assumed that the nbufs and bufs[] values are preserved from init.  
     */
    for (cnt = 0; cnt < nbufs; ++cnt)
    {
        Memory_free(dataHeap, bufs[cnt].base, bufs[cnt].size); 
    }
    Memory_free(dataHeap, bufs, nbufs*sizeof(Tcp3d_MemBuffer));       

    return (tcp3dResult);
}

#if EDMA_LOCAL_COMP_ISR // flag defined in sample.h file
/*******************************************************************************
 ******************************************************************************/
/**
 * Fill the tables for allocated TCC & tccCB params, used with local
 * EDMA3 call back ISR routine (see in sample_int_reg.c file).
 */
Void updateAllocatedTccsLoc(  EDMA_CONFIG         *edmaConfig)
{
    Int     i;
    UInt32  tcc;

    allocatedTCCsLoc[0] = 0u;
    allocatedTCCsLoc[1] = 0u;

    for (i=0;i<TCP3D_DRV_MAX_CH_PER_PATH;i++)
    {
        if( edmaConfig->pingChRes[i].cbFunc != NULL)
        {
            tcc = edmaConfig->pingChRes[i].tccNo;
            edma3IntrParamsLoc[tcc].tccCb = edmaConfig->pingChRes[i].cbFunc;
            edma3IntrParamsLoc[tcc].cbData = edmaConfig->pingChRes[i].cbData;
            if (tcc < 32u)
                allocatedTCCsLoc[0u] |= (0x1u << tcc);
            else
                allocatedTCCsLoc[1u] |= (0x1u << (tcc - 32u));
        }
        if( edmaConfig->pongChRes[i].cbFunc != NULL)
        {
            tcc = edmaConfig->pongChRes[i].tccNo;
            edma3IntrParamsLoc[tcc].tccCb = edmaConfig->pongChRes[i].cbFunc;
            edma3IntrParamsLoc[tcc].cbData = edmaConfig->pongChRes[i].cbData;
            if (tcc < 32u)
                allocatedTCCsLoc[0u] |= (0x1u << tcc);
            else
                allocatedTCCsLoc[1u] |= (0x1u << (tcc - 32u));
        }
    }
}
#endif

/*******************************************************************************
 ******************************************************************************/
Void openEdmaChannels ( EDMA3_DRV_Handle    hEdma,
                        UInt8               perId,
                        EDMA_CONFIG         *edmaConfig)
{
    EDMA3_DRV_Result    result, status = EDMA3_DRV_SOK;
    Int32               i;

    if ( perId < CSL_TCP3D_PER_CNT )
    {
        edmaConfig->pingChRes[0].chNo    = getRevt0ChannelNum(perId);
        edmaConfig->pongChRes[0].chNo    = getRevt1ChannelNum(perId);
        edmaConfig->pingChRes[1].chNo    = EDMA3_DRV_DMA_CHANNEL_ANY;
        edmaConfig->pongChRes[1].chNo    = EDMA3_DRV_DMA_CHANNEL_ANY;
    }
    else
    {
        System_printf("Wrong Instance ID passed\n");
        System_exit(0);
    }

    /* Fille the edmaConfig structure with defaults */
    for (i=0;i<TCP3D_DRV_MAX_CH_PER_PATH;i++)
    {
        /* PING channel defaults */
        edmaConfig->pingChRes[i].tccNo   = EDMA3_DRV_TCC_ANY;
        edmaConfig->pingChRes[i].cbFunc  = NULL;
        edmaConfig->pingChRes[i].cbData  = NULL;
    
        /* PONG channel defaults */
        edmaConfig->pongChRes[i].tccNo   = EDMA3_DRV_TCC_ANY;
        edmaConfig->pongChRes[i].cbFunc  = NULL;
        edmaConfig->pongChRes[i].cbData  = NULL;
    }

    /**
     * Open all the Physical Channels and then register call backs 
     */ 
    for(i=0; i<TCP3D_DRV_MAX_CH_PER_PATH; i++)
    {
        result = EDMA3_DRV_requestChannel (hEdma, 
                                           &edmaConfig->pingChRes[i].chNo, 
                                           &edmaConfig->pingChRes[i].tccNo,
                                           (EDMA3_RM_EventQueue)0,
                                           edmaConfig->pingChRes[i].cbFunc, 
                                           edmaConfig->pingChRes[i].cbData);
#if DEBUG_PRINT
        System_printf("\tEDMA channel %d open (result = %d)\n", edmaConfig->pingChRes[i].chNo, result);
#endif
        status |= result;

        result = EDMA3_DRV_requestChannel (hEdma, 
                                           &edmaConfig->pongChRes[i].chNo, 
                                           &edmaConfig->pongChRes[i].tccNo,
                                           (EDMA3_RM_EventQueue)0,
                                           edmaConfig->pongChRes[i].cbFunc, 
                                           edmaConfig->pongChRes[i].cbData);
#if DEBUG_PRINT
        System_printf("\tEDMA channel %d open (result = %d)\n", edmaConfig->pongChRes[i].chNo, result);
#endif
        status |= result;
    } /* end of - for(i=0; i<TCP3D_DRV_MAX_CH_PER_PATH; i++) */

    /**
     * Open all the Link Channels 
     */
    for(i=0; i<TCP3D_DRV_MAX_LINK_CH; i++)
    {
        /* Load the local variable with default values */
        edmaConfig->linkChRes[i].chNo = EDMA3_DRV_LINK_CHANNEL; 

        result = EDMA3_DRV_requestChannel (hEdma, 
                                           &edmaConfig->linkChRes[i].chNo, 
                                           &edmaConfig->linkChRes[i].tccNo,
                                           (EDMA3_RM_EventQueue)0,
                                           edmaConfig->linkChRes[i].cbFunc, 
                                           edmaConfig->linkChRes[i].cbData);
#if DEBUG_PRINT
        System_printf("\tEDMA link channel %d open (result = %d)\n", edmaConfig->linkChRes[i].chNo, result);
#endif
        status |= result;
    } /* end of - for(i=0; i<TCP3D_DRV_MAX_LINK_CH; i++) */

#if EDMA_LOCAL_COMP_ISR // flag defined in sample.h file
    /* This must be called after opening channels */
    updateAllocatedTccsLoc(edmaConfig);
#endif

    if ( status != EDMA3_DRV_SOK )
        System_exit(0);
    
} /* openEdmaChannels() */

/*******************************************************************************
 ******************************************************************************/
Void closeEdmaChannels( EDMA3_DRV_Handle    hEdma,
                        UInt8               perId,
                        EDMA_CONFIG         *edmaConfig)
{
    EDMA3_DRV_Result    result, status = EDMA3_DRV_SOK;
    Int32               i;

    /**
     * Close all the Physical Channels and do unregister call backs before
     */
    for(i=0; i<TCP3D_DRV_MAX_CH_PER_PATH; i++)
    {
        result = EDMA3_DRV_freeChannel (hEdma, edmaConfig->pingChRes[i].chNo);
#if DEBUG_PRINT
        System_printf("\tEDMA channel %d close (result = %d)\n", edmaConfig->pingChRes[i].chNo, result);
#endif
        status |= result;
        
        result = EDMA3_DRV_freeChannel (hEdma, edmaConfig->pongChRes[i].chNo);
#if DEBUG_PRINT
        System_printf("\tEDMA channel %d close (result = %d)\n", edmaConfig->pongChRes[i].chNo, result);
#endif
        status |= result;
    } /* end of - for(i=0; i<TCP3D_DRV_MAX_CH_PER_PATH; i++) */

    /**
     * Close all the Link Channels 
     */
    for(i=0; i<TCP3D_DRV_MAX_LINK_CH; i++)
    {
        result = EDMA3_DRV_freeChannel (hEdma, edmaConfig->linkChRes[i].chNo);
#if DEBUG_PRINT
        System_printf("\tEDMA link channel %d close (result = %d)\n", edmaConfig->linkChRes[i].chNo, result);
#endif
        status |= result;
    }

    if ( status != EDMA3_DRV_SOK )
        System_exit(0);
    
} /* closeEdmaChannels() */

/* End of File */
