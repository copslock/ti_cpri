/** 
 *   @file  fftc_lld.c
 *
 *   @brief  
 *      This file contains the FFTC Configuration and Helper APIs useful in 
 *      setting up the FFTC engine for DFT/IDFT calculations. It also contains
 *      various APIs required to retrieve and setup the FFTC error keeping related 
 *      registers.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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
/* FFTC LLD types include */
#include <fftc_types.h>

/* FFTC LLD include */
#include <ti/drv/fftc/fftc_lld.h>

/** @addtogroup FFTC_LLD_DATASTRUCT
 @{ */

/**
 *  @brief  Fftc_DftBlockSizeTable
 *         
 *          Array of all supported DFT block sizes
 *          by the FFTC engine.
 */
uint32_t  Fftc_DftBlockSizeTable [50] = {
    4, 8,
    16, 32, 64, 128, 256, 512,
    1024, 2048, 4096, 8192, 12, 24,
    48, 96, 192, 384, 768, 1536,
    3072, 6144, 36, 72, 144, 288,
    576, 1152, 108, 216, 432, 864,
    324, 648, 1296, 972, 60, 120,
    240, 480, 960, 180, 360, 720,
    540, 1080, 300, 600, 900, 1200
};

/**
@}
*/

/** @addtogroup FFTC_LLD_FUNCTION
 @{ */

/**
 * ============================================================================
 *  @n@b Fftc_mapDFTSizeToIndex
 *
 *  @b  brief
 *  @n  This API takes DFT block size as input and returns the corresponding
 *      index into the DFT table.
 *
 *  @param[in]    
        dftBlockSize        The DFT/IDFT block size in bytes that needs to be
                            translated into a DFT block size table index. 
                            Please consult the FFTC User Guide for all valid block 
                            size values recognized by the FFTC Engine.
 *
 *  @return  int32_t
 *  @li                     -1  -   Invalid DFT size specified
 *  @li                     >=0 -   A valid index (ranging between 0 and 49) into 
 *                                  the DFT size table for the size specified.
 *
 *  @pre
 *  @n  None.
 *
 *  @post
 *  @n  None.
 * 
 *  @code
        int32_t     dftIndex;

        if ((dftIndex = Fftc_mapDFTSizeToIndex (512)) != -1)
        {
            // Proceed with FFTC configuration.
            ...
        }
        else
        {
            // DFT block size specified not a legal
            // value. error.
        }          
     @endcode
 * ============================================================================
 */
int32_t Fftc_mapDFTSizeToIndex 
(
    uint32_t                    dftBlockSize
)
{
    switch (dftBlockSize)
    {
        case    4:
        {
            return 0;
        }
        case    8:
        {
            return 1;
        }
        case    16:
        {
            return 2;
        }
        case    32:
        {
            return 3;
        }
        case    64:
        {
            return 4;
        }
        case    128:
        {
            return 5;
        }
        case    256:
        {
            return 6;
        }
        case    512:
        {
            return 7;
        }
        case    1024:
        {
            return 8;
        }
        case    2048:
        {
            return 9;
        }
        case    4096:
        {
            return 10;
        }
        case    8192:
        {
            return 11;
        }
        case    12:
        {
            return 12;
        }
        case    24:
        {
            return 13;
        }
        case    48:
        {
            return 14;
        }
        case    96:
        {
            return 15;
        }
        case    192:
        {
            return 16;
        }
        case    384:
        {
            return 17;
        }
        case    768:
        {
            return 18;
        }
        case    1536:
        {
            return 19;
        }        
        case    3072:
        {
            return 20;
        }
        case    6144:
        {
            return 21;
        }
        case    36:
        {
            return 22;
        }
        case    72:
        {
            return 23;
        }
        case    144:
        {
            return 24;
        }
        case    288:
        {
            return 25;
        }
        case    576:
        {
            return 26;
        }
        case    1152:
        {
            return 27;
        }
        case    108:
        {
            return 28;
        }
        case    216:
        {
            return 29;
        }        
        case    432:
        {
            return 30;
        }
        case    864:
        {
            return 31;
        }
        case    324:
        {
            return 32;
        }
        case    648:
        {
            return 33;
        }
        case    1296:
        {
            return 34;
        }
        case    972:
        {
            return 35;
        }
        case    60:
        {
            return 36;
        }
        case    120:
        {
            return 37;
        }
        case    240:
        {
            return 38;
        }
        case    480:
        {
            return 39;
        }
        case    960:
        {
            return 40;
        }
        case    180:
        {
            return 41;
        }
        case    360:
        {
            return 42;
        }
        case    720:
        {
            return 43;
        }
        case    540:
        {
            return 44;
        }
        case    1080:
        {
            return 45;
        }
        case    300:
        {
            return 46;
        }
        case    600:
        {
            return 47;
        }
        case    900:
        {
            return 48;
        }
        case    1200:
        {
            return 49;
        }        
        case    63:
        {
            /* DFT size 0x3f / 63 corresponds to DFT index
             * 63 too. DFT index 63 has a special meaning. 
             * This indicates that the FFTC queue should use
             * the global DFT size list.
             */
            return 63;                
        }
        default:
        {
            return -1;
        }
    }
}

/******************************************************************************
 ******************   FFTC CONFIGURATION HELPER APIs    ***********************
 ******************************************************************************/

/**
 * ============================================================================
 *  @n@b Fftc_compileQueueLocalConfigParams
 *
 *  @b  brief
 *  @n  This API translates the FFTC queue specific local configuration provided 
 *      by the driver/application in 'pFFTLocalCfg' parameter into a format 
 *      understood by the FFTC engine and populates the output parameter 'pData' 
 *      with it. The data obtained from this API with an appropriate control header
 *      can thus be used for sending queue configuration to FFTC engine through 
 *      CPPI. This API assumes that the output parameter passed 'pData'
 *      has been allocated memory.
 *
 *  @param[in]    
        pFFTLocalCfg        Input local configuration structure with all necessary
                            options set by the application/driver.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the queue configuration provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient local configuration provided.
 *  @li                     0   -   Successfully compiled the local configuration
 *                                  parameters.    
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * 
 *  @code
        Fftc_QLocalCfg          fftLocalCfg;
        uint8_t                 localCfgData[256];
        uint32_t                dataLen;

        ...

        // Setup the queue configuration parameters
        fftLocalCfg.destQRegConfig.cppiDestQNum     =   3;
        fftLocalCfg.destQRegConfig.bOutputFFTShift  =   0;
        ...
        fftLocalCfg.controlRegConfig.dftSize        =   512;
        fftLocalCfg.controlRegConfig.dftMode        =   Fftc_DFTMode_DFT;
        ...


        // Compile the configuration into FFT engine compatible 
        // format.
        if(Fftc_compileQueueLocalConfigParams (&fftLocalCfg, localCfgData, &dataLen) > 0)
        {
            // FFT local configuration compilation failed.
            // do error recovery.
            ...
        }
        else
        {
            // proceed formulating the FFTC control header in 
            // CPPI packet.
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_compileQueueLocalConfigParams 
(
    Fftc_QLocalCfg*             pFFTLocalCfg, 
    uint8_t*                    pData, 
    uint32_t*                   pLen
)
{
    Fftc_QLocalCfgParams*       pFFTLocalCfgParams   = NULL;
    int32_t                     dftIndex, word;

#ifdef FFTC_DRV_DEBUG
    /* Validate Input */        
    if (!pFFTLocalCfg || !pData || !pLen)
        return -1;
#endif

    pFFTLocalCfgParams = (Fftc_QLocalCfgParams *)(pData);

    /* The FFTC Local Configuration must be compiled in the following order:
     *  1. FFTC Queue x Destination Queue Register
     *  2. FFTC Queue x Scaling & Shifting Register
     *  3. FFTC Queue x Cyclic Prefix Register
     *  4. FFTC Queue x Control Register
     *  5. FFTC Queue x LTE Frequency Shift Register
     */

    /* Step 1. Compile the configuration for FFTC Queue x Destination Queue Register.
     */
    pFFTLocalCfgParams->queuexDestQ =   
                                        CSL_FMK (FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_OUTPUT, 
                                                pFFTLocalCfg->destQRegConfig.bOutputFFTShift) |
                                        CSL_FMK (FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_INPUT, 
                                                pFFTLocalCfg->destQRegConfig.bInputFFTShift) |
                                        CSL_FMK (FFTC_Q0_DEST_FFTC_VARIABLE_SHIFT_INPUT, 
                                                pFFTLocalCfg->destQRegConfig.inputShiftVal) |
                                        CSL_FMK (FFTC_Q0_DEST_DEFAULT_DEST, 
                                                pFFTLocalCfg->destQRegConfig.cppiDestQNum);

    /* Step 2.Fill in the Queue x Scaling and Shift Register configuration. */
    word =  CSL_FMK (FFTC_Q0_SCALE_SHIFT_DYNAMIC_SCALING_ENABLE, 
                    pFFTLocalCfg->scalingShiftingRegConfig.bDynamicScaleEnable) |
            CSL_FMK (FFTC_Q0_SCALE_SHIFT_OUTPUT_SCALING, 
                    pFFTLocalCfg->scalingShiftingRegConfig.outputScaleVal);

    /* The scaling factors at various butterfly stages are only configurable
     * when FFT is configured to be in "Static" mode.
     */
    if (!pFFTLocalCfg->scalingShiftingRegConfig.bDynamicScaleEnable)
    {
        word |= 
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_OUT_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.radixScalingValLast) |
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_6_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[6]) |
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_5_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[5]) |
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_4_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[4]) |
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_3_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[3]) |
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_2_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[2]) |
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_1_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[1]) |
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_0_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[0]);            

        /* Configure the LTE Frequency Shift Scaling only if LTE Frequency Shift is enabled */
        if (pFFTLocalCfg->freqShiftRegConfig.bFreqShiftEnable)
        {
            word |= CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_LTE_SHIFT_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.freqShiftScaleVal);
        }
    }
    pFFTLocalCfgParams->queuexScaleShift    =   word;

    /* Step 3. Fill in the configuration for Queue x Cyclic Prefix Register */
    word = CSL_FMK (FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_EN, 
                    pFFTLocalCfg->cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable);
    if (pFFTLocalCfg->cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable)
    {
        word |= CSL_FMK (FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_OFFSET, 
                        pFFTLocalCfg->cyclicPrefixRegConfig.cyclicPrefixRemoveNum); 
    }
    if (pFFTLocalCfg->cyclicPrefixRegConfig.bCyclicPrefixAddEnable)
    {
        word |= CSL_FMK (FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_ADDITION, 
                        pFFTLocalCfg->cyclicPrefixRegConfig.cyclicPrefixAddNum);
    }
    pFFTLocalCfgParams->queuexCyclicPrefix  =   word;

    /* Step 4. Set up the configuration for Queue x Control Register */
    word =  CSL_FMK (FFTC_Q0_CONTROL_SUPPRESSED_SIDE_INFO, 
                    pFFTLocalCfg->controlRegConfig.bSupressSideInfo) |
            CSL_FMK (FFTC_Q0_CONTROL_DFT_IDFT_SELECT, 
                    pFFTLocalCfg->controlRegConfig.dftMode);
    if (pFFTLocalCfg->controlRegConfig.bZeroPadEnable)
    {
        word |= CSL_FMK (FFTC_Q0_CONTROL_ZERO_PAD_MODE, 
                        pFFTLocalCfg->controlRegConfig.zeroPadMode) |
                CSL_FMK (FFTC_Q0_CONTROL_ZERO_PAD_VAL, 
                        pFFTLocalCfg->controlRegConfig.zeroPadFactor);
    }

    /* Get the corresponding DFT index for the DFT block size provided. */
    if ((dftIndex = Fftc_mapDFTSizeToIndex (pFFTLocalCfg->controlRegConfig.dftSize)) != -1)
    {
        word |= CSL_FMK (FFTC_Q0_CONTROL_DFT_SIZE, dftIndex);
    }
    else
    {
        /* Invalid DFT size provided. Return error. */            
        return -1;            
    }
    pFFTLocalCfgParams->queuexControl   =   word;

    /* Step 5. Finally fill in the configuration for LTE Frequency Shift Register */
    word =  CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_EN, 
                    pFFTLocalCfg->freqShiftRegConfig.bFreqShiftEnable);              
    if (pFFTLocalCfg->freqShiftRegConfig.bFreqShiftEnable)
    {
        word |= CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_DIR, 
                        pFFTLocalCfg->freqShiftRegConfig.freqShiftDirection) |
                CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_FACTOR, 
                        pFFTLocalCfg->freqShiftRegConfig.freqShiftMultFactor) |
                CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_PHASE, 
                        pFFTLocalCfg->freqShiftRegConfig.freqShiftInitPhase) |
                CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_INDEX, 
                        pFFTLocalCfg->freqShiftRegConfig.freqShiftIndex);              
    }
    pFFTLocalCfgParams->queuexLteFreq   =   word;
    
    /* Configuration Done. Return Success. 
     * Return the number of bytes configured in the output
     * parameter data pointer passed. We always fill
     * 5 32-bit words worth data. Hence return (5 * 32 / 8) bytes.
     */
    *pLen += (( 5 * 32 ) / 8);

    return  0;
}


/**
 * ============================================================================
 *  @n@b Fftc_recompileQueueLocalDFTParams
 *
 *  @b  brief
 *  @n  This API just configures 'DFT_IDFTelect' and 'DFTize' bits in the
 *      Queue x Control Register assuming that everything else in the input local
 *      configuration parameters has not changed. This API assumes that the input
 *      configuration parameters pointer passed to this API 'pData' has been 
 *      compiled fully at least once before using the API 
 *      @a Fftc_compileQueueLocalConfigParams().
 *
 *  @param[in]      
        dftSize             DFT block size in bytes to be configured

 *  @param[in]      
        dftMode             Indicates whether DFT/IDFT transformation
                            must be performed. 0 indicates IDFT, 1 for DFT.

 *  @param[in,out]                             
        pData               Data buffer to be filled in with the DFT parameters 
                            provided.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient local configuration provided.
 *  @li                     0   -   Successfully configured.
 *
 *  @pre
 *  @n  @a Fftc_compileQueueLocalConfigParams() API must have been called before
 *      to populate the 'pData' with complete local configuration.
 *
 *  @post
 *  @n  The output params buffer 'pData' is populated accordingly.
 * 
 *  @code
        Fftc_QLocalCfg          fftLocalCfg;
        uint8_t                 localCfgData[256];

        ...

        // Setup the queue configuration parameters
        fftLocalCfg.destQRegConfig.cppiDestQNum =   3;
        fftLocalCfg.destQRegConfig.bOutputFFTShift    = 0;
        ...
        fftLocalCfg.controlRegConfig.dftSize          =   512;
        fftLocalCfg.controlRegConfig.dftMode          =   Fftc_DFTMode_IDFT;
        ...


        // Compile the configuration into FFT engine compatible 
        // format.
        if(Fftc_compileQueueLocalConfigParams (&fftLocalCfg, localCfgData) > 0)
        {
            // FFT local configuration compilation failed.
            // do error recovery.
            ...
        }
        else
        {
            // proceed formulating the FFTC control header in 
            // CPPI packet.
            ...
        }

        Fftc_recompileQueueLocalDFTParams (6144, Fftc_DFTMode_DFT,  localCfgData);

     @endcode
 * ============================================================================
 */
int32_t Fftc_recompileQueueLocalDFTParams 
(
    int32_t                     dftSize, 
    Fftc_DFTMode                dftMode, 
    uint8_t*                    pData
)
{
    Fftc_QLocalCfgParams*       pFFTLocalCfgParams   = NULL;
    int32_t                     dftIndex;

#ifdef FFTC_DRV_DEBUG
    /* Validate Input */        
    if (!pData)
        return -1;
#endif

    pFFTLocalCfgParams = (Fftc_QLocalCfgParams *)(pData);            

    /* Get the corresponding DFT index for the DFT block size provided. */
    if ((dftIndex = Fftc_mapDFTSizeToIndex (dftSize)) != -1)
    {
        CSL_FINS (pFFTLocalCfgParams->queuexControl, FFTC_Q0_CONTROL_DFT_SIZE, dftIndex);
    }
    else
    {
        /* Invalid DFT block size passed, return error */            
        return -1;            
    }    

    /* Configure the IDFT/DFT Select bit */
    CSL_FINS (pFFTLocalCfgParams->queuexControl, FFTC_Q0_CONTROL_DFT_IDFT_SELECT, dftMode);

    /* Return success */
    return 0;
}


/**
 * ============================================================================
 *  @n@b Fftc_recompileQueueLocalCyclicPrefixParams
 *
 *  @b  brief
 *  @n  This API just configures 'cyclic_prefix_addition' bits in the
 *      Queue x Cyclic Prefix Register assuming that everything else in the input 
 *      local configuration parameters has not changed. This API assumes that the 
 *      input configuration parameters pointer passed to this API 'pData' has been 
 *      compiled fully at least once before using the API 
 *      @a Fftc_compileQueueLocalConfigParams().
 *
 *  @param[in]    
        cyclicPrefixLen     Number of samples of cyclic prefix to add.

 *  @param[in,out]    
        pData               Data buffer that needs to be modified with the cyclic 
                            prefix parameter provided.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient local configuration provided.
 *  @li                     0   -   Successfully configured the data buffer.
 *
 *  @pre
 *  @n  @a Fftc_compileQueueLocalConfigParams() API must have been called at least once
 *      to initialize the 'pData' with complete local configuration.
 *
 *  @post
 *  @n  The output params buffer 'pData' is populated accordingly.
 * 
 *  @code
        Fftc_QLocalCfg          fftLocalCfg;
        uint8_t                 localCfgData[256];

        ...

        // Setup the queue configuration parameters
        fftLocalCfg.destQRegConfig.cppiDestQNum     =   3;
        fftLocalCfg.destQRegConfig.bOutputFFTShift  =   0;
        ...
        fftLocalCfg.controlRegConfig.dftSize        =   512;
        fftLocalCfg.controlRegConfig.dftMode        =   Fftc_DFTMode_DFT;
        ...


        // Compile the configuration into FFT engine compatible 
        // format.
        if(Fftc_compileQueueLocalConfigParams (&fftLocalCfg, localCfgData) > 0)
        {
            // FFT local configuration compilation failed.
            // do error recovery.
            ...
        }
        else
        {
            // proceed formulating the FFTC control header in 
            // CPPI packet.
            ...
        }

        Fftc_recompileQueueLocalCyclicPrefixParams (32,  localCfgData);

     @endcode
 * ============================================================================
 */
int32_t Fftc_recompileQueueLocalCyclicPrefixParams 
(
    int32_t                     cyclicPrefixLen, 
    uint8_t*                    pData
)
{
    Fftc_QLocalCfgParams*       pFFTLocalCfgParams   = NULL;

#ifdef FFTC_DRV_DEBUG
    /* Validate Input */        
    if (!pData)
        return -1;
#endif

    pFFTLocalCfgParams = (Fftc_QLocalCfgParams *)(pData);            

    CSL_FINS (pFFTLocalCfgParams->queuexCyclicPrefix, FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_ADDITION, cyclicPrefixLen);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_createControlHeader
 *
 *  @b  brief
 *  @n  This API compiles the FFTC Control Header as per the input configuration
 *      specified in 'pFFTCfgCtrlHdr' and stores it in the output parameter
 *      specified 'pData'. This API assumes that the output parameter passed 'pData'
 *      has been allocated memory before passing it to this API.
 *
 *  @param[in]
        pFFTCfgCtrlHdr      FFTC control header configuration as specified by the
                            application/driver that needs to be formatted for FFTC
                            engine format.

 *  @param[out]
        pData               Data buffer to be filled in with the formatted FFTC 
                            control header.

 *  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.                            
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid configuration handle provided.
 *  @li                     0   -   Successfully compiled the control header.
 *
 *  @pre
 *  @n  The data buffer pointer passed to this API 'pData' must have been allocated
 *      memory before passing to this API. 'pLen' should be a valid pointer.
 *
 *  @post
 *  @n  The output params structure 'pData' is populated accordingly and the number
 *      of bytes configured by this API is updated in the 'pLen' parameter.
 * 
 *  @code
        Fftc_ControlHdr             fftCfgCtrlHdr;
        uint8_t                     CtrlHdrData[256];
        uint32_t                    dataLen;

        ...

        // Setup the FFT control header configuration.
        fftCfgCtrlHdr.psFieldLen            =   4;
        fftCfgCtrlHdr.dftSizeListLen        =   5;
        fftCfgCtrlHdr.bPSPassThruPresent    =   1;
        fftCfgCtrlHdr.bDFTSizeListPresent   =   1;
        fftCfgCtrlHdr.bLocalConfigPresent   =   1;

        if (Fftc_createControlHeader(&fftCfgCtrlHdr, CtrlHdrData, &dataLen) != 0)
        {
            // Error returned by the API. Invalid Data Handle?
            // exit
        }
        else
        {
            //Proceed to putting together the rest of
            //the queue local configuration and pass
            //through data.
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_createControlHeader 
(
    Fftc_ControlHdr*            pFFTCfgCtrlHdr, 
    uint8_t*                    pData, 
    uint32_t*                   pLen
)
{
    uint32_t*  pCtrlHdr   = NULL;

#ifdef FFTC_DRV_DEBUG
    /* Validate Input */
    if (!pData || !pLen)
        return -1;
#endif

    pCtrlHdr = (uint32_t *)(pData);

    /* Compile the FFTC Control Header */
    *pCtrlHdr = (((pFFTCfgCtrlHdr->psFieldLen << 0x00000018) & (0x07000000)) |          /* populate the Protocol Specific Field length */
                ((pFFTCfgCtrlHdr->dftSizeListLen << 0x00000010) & (0x001F0000)) |       /* populate the DFT size list length */
                ((pFFTCfgCtrlHdr->bPSPassThruPresent << 0x00000002) & (0x00000004)) |   /* populate the debug halt enable bit */
                ((pFFTCfgCtrlHdr->bDFTSizeListPresent << 0x00000001) & (0x00000002)) |  /* populate the DFT list sizes present bit */
                ((pFFTCfgCtrlHdr->bLocalConfigPresent << 0x00000000) & (0x00000001)));  /* populate the local config present bit */

    /* Update the number of bytes configured.
     * Control Header is always 32-bits long. So length is 4 bytes */
    *pLen += 4;

    /* Control Header compilation succesful. Return 0 */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_createDftSizeList
 *
 *  @b  brief
 *  @n  This API compiles the FFTC DFT size list specified by 'pDftSizeList' and 
 *      'dftSizeListLen' input parameters and stores it in the output parameter
 *      specified 'pData'. This API assumes that the output parameter passed 'pData'
 *      has been allocated memory before passing it to this API.
 *
 *      The DFT sizes passed to this API in 'pDftSizeList' are assumed to be valid
 *      values and are not validated by this API.
 *
 *  @param[in]
        pDftSizeList        List of DFT block sizes that needs to be formatted
                            to FFTC H/w format.

 *  @param[in]
        dftSizeListLen      Number of DFT block sizes specified in 'pDftSizeList'
                            input. The maximum number of FFT blocks supported by
                            hardware is 128. The caller needs to ensure that this
                            limit is not exceeded.

 *  @param[out]
        pData               Data buffer to be filled in with the formatted FFTC 
                            size list configuration.

 *  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.                            
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid configuration handle provided.
 *  @li                     0   -   Successfully compiled the size list.
 *
 *  @pre
 *  @n  The data buffer pointer passed to this API 'pData' must have been allocated
 *      memory before passing to this API. 'pLen' should be a valid pointer. All
 *      DFT block sizes specified in'pDftSizeList' must be valid values. Please 
 *      consult FFTC Users guide for legal values.
 *
 *  @post
 *  @n  The output params structure 'pData' is populated accordingly and the number
 *      of bytes configured by this API is updated in the 'pLen' parameter.
 * 
 *  @code
        uint16_t                      dftSizeList[32];
        uint32_t                      numDftBlocks, i;
        uint8_t                       fftRequestData[256];
        uint32_t                      dataLen;

        ...

        // Setup the DFT size list configuration.
        numDftBlocks = 32;
        for (i = 0; i < numDftBlocks; i ++)
            dftSizeList [i] = 0x80; // program the DFT block size in the same order as they
                                   // will appear in data.

        if (Fftc_createDftSizeList(dftSizeList, numDftBlocks, fftRequestData, &dataLen) != 0)
        {
            // Error returned by the API. Invalid Data Handle?
            // exit
        }
        else
        {
            //DFT size list compilation successful.
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_createDftSizeList 
(
    uint16_t*                     pDftSizeList, 
    uint32_t                      dftSizeListLen, 
    uint8_t*                      pData, 
    uint32_t*                     pLen
)
{
    uint32_t                      dftListGroupVal;
    uint32_t                      i, grpNum, blockNum, numGroups, numSizesLeft, dataOffset;

#ifdef FFTC_DRV_DEBUG
    /* Validate input */
    if (!pDftSizeList || !pData)
        return -1;
#endif

    /* Each DFT Size List Group consists of 5 DFT sizes */
    numGroups           = dftSizeListLen / 5;     
    numSizesLeft        = dftSizeListLen % 5;

    /* Initialize our running data offset to 0 */
    dataOffset          = 0;

    /* Format DFT block sizes into groups of 5 sizes each. */
    for (grpNum = 0, blockNum = 0; grpNum < numGroups; grpNum ++)
    {
        dftListGroupVal =   CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_0, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum])) |
                            CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_1, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + 1])) |
                            CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_2, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + 2])) |
                            CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_3, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + 3])) |
                            CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_4, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + 4]));

        *(uint32_t *)(pData + dataOffset)     =   dftListGroupVal;
        *pLen                               +=  sizeof (dftListGroupVal);
        dataOffset                          +=  sizeof (dftListGroupVal);

        blockNum                            += 5;
    }

    for (i = 0; i < numSizesLeft; i ++)
    {
        if (i == 0)
        {
            dftListGroupVal =   CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_0, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + i]));
        }
        else if (i == 1)
        {
            dftListGroupVal |=  CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_1, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + i]));
        }
        else if (i == 2)
        {
            dftListGroupVal |=  CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_2, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + i]));
        }
        else if (i == 3)
        {
            dftListGroupVal |=  CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_3, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + i]));
        }
    }
    if (numSizesLeft)
    {
        /* Done compiling. Return the updated length */
        *(uint32_t *)(pData + dataOffset)     =   dftListGroupVal;
        *pLen                               +=  sizeof (dftListGroupVal);
    }

    /* Return Success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_modifyLocalCfgPresentControlHeader
 *
 *  @b  brief
 *  @n  This API modifies a pre-compiled FFTC Control header obtained from
 *      @a Fftc_createControlHeader() API, to just toggle the 
 *      "Local Configuration Data Present" bit. It leaves the rest of the
 *      header unchanged. This API assumes that the output parameter passed 'pData'
 *      has been allocated memory and properly setup using @a Fftc_createControlHeader() 
 *      API before.
 *
 *  @param[in]   
        bLocalConfigPresent uint8_tean flag, when set to 1 indicates that a local
                            configuration follows the control header.
                            
 *  @param[in,out]   
        pData               Control Header buffer to be modified with the local 
                            config bit.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid configuration handle provided.
 *  @li                     0   -   Successfully compiled the control header.
 *
 *  @pre
 *  @n  The data buffer passed to this API 'pData' must be initialized using
 *      @a Fftc_createControlHeader() before calling this API to modify
 *      the local configuration bit.
 *
 *  @post
 *  @n  The output params structure 'pData' is populated accordingly.
 * 
 *  @code
        Fftc_ControlHdr             fftCfgCtrlHdr;
        uint8_t                     CtrlHdrData[256];

        ...

        // Setup the FFT control header configuration.
        fftCfgCtrlHdr.psFieldLen            =   4;
        fftCfgCtrlHdr.dftSizeListLen        =   5;
        fftCfgCtrlHdr.bPSPassThruPresent    =   1;
        fftCfgCtrlHdr.bDFTSizeListPresent   =   1;
        fftCfgCtrlHdr.bLocalConfigPresent   =   1;

        if (Fftc_createControlHeader(&fftCfgCtrlHdr, CtrlHdrData) != 0)
        {
            // Error returned by the API. Invalid Data Handle?
            // exit
        }
        else
        {
            //Proceed to putting together the rest of
            //the queue local configuration and pass
            //through data.
            ...
        }

        if (Fftc_modifyLocalCfgPresentControlHeader (0, CtrlHdrData) != 0)
        {
            //Error. Invalid Control header data pointer??
            //exit
        }
        else
        {
            //Configuration successful. Proceed to next step.
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_modifyLocalCfgPresentControlHeader 
(
    int32_t                       bLocalConfigPresent, 
    uint8_t*                      pData
)
{
    uint32_t*                     pCtrlHdr   = NULL;

#ifdef FFTC_DRV_DEBUG
    /* Validate Input */
    if (!pData)
        return -1;
#endif

    pCtrlHdr = (uint32_t *)(pData);

    /* Just toggle the "local config present" bit in the
     * control header without modifying anything else.
     */
    *pCtrlHdr = ((*pCtrlHdr & ~0x00000001) | 
                ((bLocalConfigPresent << 0x00000000) & (0x00000001)));

    /* Control Header compilation succesful. Return 0 */
    return 0;
}

/******************************************************************************
 ***********************   FFTC MMR ACCESS APIs    ****************************
 ******************************************************************************/

/**
 * ============================================================================
 *  @n@b Fftc_lldOpen
 *
 *  @b  brief
 *  @n  This API initializes the output param 'pFFTCLldObj' with the configuration 
 *      register overlay address for the FFTC peripheral instance corresponding 
 *      to the instance number passed in 'instNum' parameter. This API MUST be
 *      called to initialize the FFTC LLD object corresponding to a specific 
 *      FFTC peripheral instance before any of the FFTC LLD APIs are invoked 
 *      to configure that instance MMR.
 *
 *  @param[in]    
        instNum             FFTC peripheral instance number for which the LLD
                            object needs to be initialized.
 *  @param[in]    
        cfgRegs             Configuration registers (MMR) base address for this
                            FFTC instance.

 *  @param[out]    
        pFFTCLldObj         Pointer to FFTC LLD Object structure that needs to be
                            initialized with the instance's configuration register 
                            overlay base address.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid LLD object handle/ invalid instance number
 *                                  provided.
 *  @li                     0   -   Successfully initialized the LLD object with
 *                                  the appropriate base address for the instance 
 *                                  number provided.
 *
 *  @pre
 *  @n  The output FFTC LLD object structure 'pFFTCLldObj' passed must be a valid
 *      pointer. 
 *
 *  @post
 *  @n  The FFTC LLD object pointer passed is populated with appropriate base address
 *      for the configuration registers for the corresponding instance number.
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;

        if (Fftc_lldOpen (CSL_FFTC_A, 0x021F0000, &fftcLldObj) != 0)
        {
            // Error opening FFTC LLD for CSL_FFTC_A instance
            ...
        }
        else
        {
            // Successfully opened FFTC LLD for instance A
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_lldOpen 
(
    uint8_t                     instNum,
    void*                       cfgRegs,
    Fftc_LldObj*                pFFTCLldObj
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the FFTC LLD Object Handle passed */        
    if (!pFFTCLldObj || instNum >= CSL_FFTC_PER_CNT)
        return -1;
#endif

    pFFTCLldObj->instNum    =   instNum;
    pFFTCLldObj->cfgRegs    =   (CSL_FftcRegsOvly) cfgRegs;
            
    /* FFTC LLD Open successful. Return success. */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_lldClose
 *
 *  @b  brief
 *  @n  This API resets the contents of the FFTC LLD Object handle passed to 
 *      this function. The FFTC Object handle is no longer valid for use with
 *      any of the FFTC LLD MMR access APIs.
 *
 *  @param[out]    
        pFFTCLldObj         Pointer to FFTC LLD Object structure that needs to be
                            de-initialized.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid LLD object handle provided.
 *  @li                     0   -   Successfully de-initialized the LLD object provided.
 *
 *  @pre
 *  @n  The output FFTC LLD object structure 'pFFTCLldObj' passed must be a valid
 *      pointer.
 *
 *  @post
 *  @n  The FFTC LLD object pointer's contents are all de-initialized. This object 
 *      pointer is no longer valid for use with any of the FFTC LLD MMR access
 *      APIs. 
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;

        if (Fftc_lldClose (&fftcLldObj) != 0)
        {
            // Error closing FFTC LLD for CSL_FFTC_A instance
            ...
        }
        else
        {
            // Successfully closed FFTC LLD for instance A
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_lldClose 
(
    Fftc_LldObj*                pFFTCLldObj
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the FFTC LLD Object Handle passed */        
    if (!pFFTCLldObj)
        return -1;
#endif

    pFFTCLldObj->instNum    =   0;
    pFFTCLldObj->cfgRegs    =   NULL;
            
    /* FFTC LLD close successful. Return success. */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_readPidReg
 *
 *  @b  brief
 *  @n  This API loads the contents of FFTC PID Register in the output parameter
 *      structure passed 'pPIDCfg'. This API assumes that 'pPIDCfg' is a valid
 *      pointer.
 *
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.

 *  @param[out]    
        pPIDCfg             Handle to FFTC PID configuration structure that
                            needs to be filled in with contents of the PID register.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid configuration handle provided.
 *  @li                     0   -   Successfully loaded the register contents
 *                                  into the output param structure.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output PID configuration structure 
 *      'pPIDCfg' passed must be a valid pointer.
 *
 *  @post
 *  @n  Contents of FFTC PID register are loaded into 'pPIDCfg'.
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;
        Fftc_PeripheralIdParams     pidCfg;

        if (Fftc_readPidReg (&fftcLldObj, &pidCfg) != 0)
        {
            // Error reading PID register.
            ...
        }
        else
        {
            // Successfully read PID Register contents
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_readPidReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_PeripheralIdParams*    pPIDCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the PID configuration handle */        
    if (!pPIDCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    pPIDCfg->function   =   CSL_FEXT (pFFTCLldObj->cfgRegs->PID, FFTC_PID_PID);
    pPIDCfg->rtlVersion =   CSL_FEXT (pFFTCLldObj->cfgRegs->PID, FFTC_PID_RTL);
    pPIDCfg->majorNum   =   CSL_FEXT (pFFTCLldObj->cfgRegs->PID, FFTC_PID_MAJOR);
    pPIDCfg->customNum  =   CSL_FEXT (pFFTCLldObj->cfgRegs->PID, FFTC_PID_CUSTOM);
    pPIDCfg->minorNum   =   CSL_FEXT (pFFTCLldObj->cfgRegs->PID, FFTC_PID_MINOR);

    /* FFTC Peripheral ID Register read successful. */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_readGlobalConfigReg
 *
 *  @b  brief
 *  @n  This API loads the contents of FFTC Configuration Register in the output 
 *      parameter structure handle passed 'pFFTGlobalCfg'. This API assumes that 
 *      'pFFTGlobalCfg' is a valid pointer.
 *
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.

 *  @param[out]    
        pFFTGlobalCfg       Handle to FFTC Global configuration structure that
                            needs to be filled in with contents of the FFTC Global
                            Configuration register.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid configuration handle provided.
 *  @li                     0   -   Successfully loaded the register contents
 *                                  into the output param structure.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output Global configuration 
 *      structure 'pFFTGlobalCfg' passed must be a valid pointer.
 *
 *  @post
 *  @n  Contents of FFTC Global Configuration register are loaded into 
 *      'pFFTGlobalCfg'.
 * 
 *  @code
        Fftc_LldObj      fftcLldObj;
        Fftc_GlobalCfg   fftcGlobalCfg;

        if (Fftc_readGlobalConfigReg (&fftcLldObj, &fftcGlobalCfg) != 0)
        {
            // Error reading FFTC Configuration register.
            ...
        }
        else
        {
            // Successfully read FFTC global configuration register.
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_readGlobalConfigReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_GlobalCfg*             pFFTGlobalCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the global configuration handle passed. */        
    if (!pFFTGlobalCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    pFFTGlobalCfg->queueFlowidOverwrite[3]  =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_Q3_FLOWID_OVERWRITE);          
    pFFTGlobalCfg->queueFlowidOverwrite[2]  =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_Q2_FLOWID_OVERWRITE);          
    pFFTGlobalCfg->queueFlowidOverwrite[1]  =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_Q1_FLOWID_OVERWRITE);          
    pFFTGlobalCfg->queueFlowidOverwrite[0]  =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_Q0_FLOWID_OVERWRITE);          
    pFFTGlobalCfg->starvationPeriodVal      =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_STARVATION_PERIOD);          
    pFFTGlobalCfg->queuePriority[3]         =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_QUEUE_3_PRIORITY);          
    pFFTGlobalCfg->queuePriority[2]         =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_QUEUE_2_PRIORITY);          
    pFFTGlobalCfg->queuePriority[1]         =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_QUEUE_1_PRIORITY);          
    pFFTGlobalCfg->queuePriority[0]         =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_QUEUE_0_PRIORITY);          
    pFFTGlobalCfg->bDisableFFT              =   CSL_FEXT (pFFTCLldObj->cfgRegs->CONFIG, FFTC_CONFIG_FFT_DISABLE);          

    /* FFTC Configuration Register successfully read. */
    return 0;
}


/**
 * ============================================================================
 *  @n@b Fftc_writeGlobalConfigReg
 *
 *  @b  brief
 *  @n  This API configures the FFTC Configuration Register with the configuration
 *      passed in 'pFFTGlobalCfg' input parameter.
 *
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.

 *  @param[in]    
        pFFTGlobalCfg       Handle to input FFTC Global configuration parameters.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid configuration handle provided.
 *  @li                     0   -   Successfully configured the register.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The input configuration structure 
 *      handle passed, 'pFFTGlobalCfg' must be a valid pointer and must contain 
 *      valid values as specified in the FFTC User Guide.
 *
 *  @post
 *  @n  FFTC Configuration Register configured.
 * 
 *  @code
        Fftc_LldObj      fftcLldObj;
        Fftc_GlobalCfg   globalCfg;

        // setup the global configuration params
        globalCfg.starvationPeriod      =   0x10;
        globalCfg.queue3Priority        =   0x03;
        globalCfg.queue2Priority        =   0x03;
        globalCfg.queue1Priority        =   0x03;
        globalCfg.queue0Priority        =   0x00;
        globalCfg.bDisableFFT           =   0x00;

        if (Fftc_writeGlobalConfigReg (&fftcLldObj, &globalCfg) != 0)
        {
            // Error configuring Global Cfg register.
            ...
        }
        else
        {
            // Successfully configured the register
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_writeGlobalConfigReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_GlobalCfg*             pFFTGlobalCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the global configuration input handle passed. */        
    if (!pFFTGlobalCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    pFFTCLldObj->cfgRegs->CONFIG =  CSL_FMK (FFTC_CONFIG_Q3_FLOWID_OVERWRITE, pFFTGlobalCfg->queueFlowidOverwrite[3]) |
                                    CSL_FMK (FFTC_CONFIG_Q2_FLOWID_OVERWRITE, pFFTGlobalCfg->queueFlowidOverwrite[2]) |
                                    CSL_FMK (FFTC_CONFIG_Q1_FLOWID_OVERWRITE, pFFTGlobalCfg->queueFlowidOverwrite[1]) |
                                    CSL_FMK (FFTC_CONFIG_Q0_FLOWID_OVERWRITE, pFFTGlobalCfg->queueFlowidOverwrite[0]) |        
                                    CSL_FMK (FFTC_CONFIG_STARVATION_PERIOD, pFFTGlobalCfg->starvationPeriodVal) |          
                                    CSL_FMK (FFTC_CONFIG_QUEUE_3_PRIORITY, pFFTGlobalCfg->queuePriority[3]) |         
                                    CSL_FMK (FFTC_CONFIG_QUEUE_2_PRIORITY, pFFTGlobalCfg->queuePriority[2]) |          
                                    CSL_FMK (FFTC_CONFIG_QUEUE_1_PRIORITY, pFFTGlobalCfg->queuePriority[1]) |          
                                    CSL_FMK (FFTC_CONFIG_QUEUE_0_PRIORITY, pFFTGlobalCfg->queuePriority[0]) |         
                                    CSL_FMK (FFTC_CONFIG_FFT_DISABLE, pFFTGlobalCfg->bDisableFFT);          

    /* FFTC Configuration Register successfully set. */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_doSoftwareReset
 *
 *  @b  brief
 *  @n  This API configures the FFTC Control Register to perform a software reset
 *      on the FFTC engine. The software reset resets the FFTC state machine and all
 *      FFTC configuration registers to initial values.
 *     
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *     
 *  @return     
 *  @n  None.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API.
 *
 *  @post
 *  @n  FFTC Control Register's 'software reset' bit configured
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;

        ...

        // Issue a software reset to the FFTC engine
        Fftc_doSoftwareReset (&fftcLldObj);

        ...
        
     @endcode
 * ============================================================================
 */
void Fftc_doSoftwareReset 
(
    Fftc_LldObj*                pFFTCLldObj
)
{
#ifdef FFTC_DRV_DEBUG
    if (!pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return;             
#endif
        
    /* Configure the FFTC Control Register's software reset bit to 1 */
    CSL_FINS (pFFTCLldObj->cfgRegs->CONTROL, FFTC_CONTROL_RESTART_BIT, 1);          

    return;
}


/**
 * ============================================================================
 *  @n@b Fftc_doSoftwareContinue
 *
 *  @b  brief
 *  @n  This API configures the FFTC Control Register to continue FFTC
 *      engine processing without resetting any state machines or configuration
 *      registers.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *       
 *  @return     
 *  @n  None.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API.
 *
 *  @post
 *  @n  FFTC Control Register's Continue bit set.
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;
        ...

        // Check if FFTC engine halted, if so resume it
        if (Fftc_isHalted(&fftcLldObj) == 1)
        {
            // Let FFTC continue its processing
            Fftc_doSoftwareContinue (&fftcLldObj);      
        }

        ...
        
     @endcode
 * ============================================================================
 */
void Fftc_doSoftwareContinue 
(
    Fftc_LldObj*                pFFTCLldObj
)
{ 
#ifdef FFTC_DRV_DEBUG
    if (!pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return;             
#endif

    /* Configure the FFTC Control Register's Continue bit */
    CSL_FINS (pFFTCLldObj->cfgRegs->CONTROL, FFTC_CONTROL_FFTC_CONTINUE, 1);          

    return;
}

/**
 * ============================================================================
 *  @n@b Fftc_isHalted
 *
 *  @b  brief
 *  @n  This API reads the FFTC Status Register and returns 1 to indicate that
 *      the FFTC engine is halted on error or emulation and 0 otherwise.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid FFTC instance object
 *  @li                     0   -   FFTC engine not halted
 *  @li                     1   -   FFTC engine halted
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API.
 *
 *  @post
 *  @n  None.
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;
        ...

        // Check if FFTC engine halted, if so resume it
        if (Fftc_isHalted(&fftcLldObj) == 1)
        {
            // Let FFTC continue its processing
            Fftc_write_ctrlReg (&fftcLldObj, 1, 0);      
        }

        ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_isHalted 
(
    Fftc_LldObj*                pFFTCLldObj
)
{
#ifdef FFTC_DRV_DEBUG
    if (!pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;             
#endif

    /* Read the "FFTC_halted" bit from FFTC Status Register.
     * Return "1" when halted , "0" otherwise
     */
    return CSL_FEXT (pFFTCLldObj->cfgRegs->STATUS, FFTC_STATUS_FFTC_HALTED);            
}

/**
 * ============================================================================
 *  @n@b Fftc_writeEmulationControlReg
 *
 *  @b  brief
 *  @n  This API configures the FFTC Emulation Control Register as per the
 *      input specified in 'pEmulationCfg' input parameter.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *       
 *  @param[in]    
        pEmulationCfg       Input parameter handle that holds the emulation
                            settings for FFTC engine.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input handle.
 *  @li                     0   -   Successfully configured the register.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The input configuration structure 
 *      handle passed, 'pEmulationCfg' must be a valid pointer and is assumed to 
 *      contain valid values as defined in the FFTC User Guide.
 *
 *  @post
 *  @n  FFTC Emulation Control Register configured.
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;
        Fftc_EmulationControlParams emulCfg;

        emulCfg.bEmuRtSel       =   0x0;
        emulCfg.bEmuSoftStop    =   0x1;
        emulCfg.bEmuFreeRun     =   0x0;

        if (Fftc_writeEmulationControlReg (&fftcLldObj, &emulCfg) != 0)
        {
            // Invalid emulation config handle passed ??
            ...
        }
        else
        {
            // Continue processing.
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_writeEmulationControlReg 
(
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_EmulationControlParams*        pEmulationCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the input handle */
    if (!pEmulationCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    pFFTCLldObj->cfgRegs->EMU_CONTROL = CSL_FMK (FFTC_EMU_CONTROL_EMU_RT_SEL, pEmulationCfg->bEmuRtSel) |
                                        CSL_FMK (FFTC_EMU_CONTROL_EMU_SOFT_STOP, pEmulationCfg->bEmuSoftStop) |
                                        CSL_FMK (FFTC_EMU_CONTROL_EMU_FREERUN, pEmulationCfg->bEmuFreeRun);

    /* FFTC Emulation Control Register configuration succesful */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_readEmulationControlReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Emulation Control Register and populates the
 *      output parameter 'pEmulationCfg' with its contents.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *      
 *  @param[out]    
        pEmulationCfg       Output parameter handle that needs to be filled with 
                            the emulation settings from FFTC engine.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input handle.
 *  @li                     0   -   Successfully populated the output param handle.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output param handle passed 
 *      'pEmulationCfg' must be a valid pointer.
 *
 *  @post
 *  @n  None.
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;
        Fftc_EmulationControlParams emulCfg;

        if (Fftc_readEmulationControlReg (&fftcLldObj, &emulCfg) != 0)
        {
            // Invalid emulation config handle passed ??
            ...
        }
        else
        {
            // Continue processing.
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_readEmulationControlReg 
(
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_EmulationControlParams*        pEmulationCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the output parameter handle to which the 
     * emulation register contents will be pupulated.
     */
    if (!pEmulationCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    pEmulationCfg->bEmuRtSel       =   CSL_FEXT (pFFTCLldObj->cfgRegs->EMU_CONTROL, FFTC_EMU_CONTROL_EMU_RT_SEL);
    pEmulationCfg->bEmuSoftStop    =   CSL_FEXT (pFFTCLldObj->cfgRegs->EMU_CONTROL, FFTC_EMU_CONTROL_EMU_SOFT_STOP);
    pEmulationCfg->bEmuFreeRun     =   CSL_FEXT (pFFTCLldObj->cfgRegs->EMU_CONTROL, FFTC_EMU_CONTROL_EMU_FREERUN);

    /* FFTC Emulation Control Register retrieval succesful */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_writeEoiReg
 *
 *  @b  brief
 *  @n  This API configures the FFTC EOI Register as per the EOI Value
 *      provided as input to acknowledge an interrupt.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *      
 *  @param[in]    
        eoiVal              EOI value to appropriately acknowledge an FFTC 
                            interrupt received by the application.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input handle.
 *  @li                     0   -   Successfully wrote EOI.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API.
 *
 *  @post
 *  @n  FFTC EOI Register configured.
 * 
 *  @code
    Fftc_LldObj                 fftcLldObj;
    ...

    Fftc_writeEoiReg (&fftcLldObj, 1);

    ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_writeEoiReg 
(
    Fftc_LldObj*                    pFFTCLldObj,
    int32_t                         eoiVal
)
{
#ifdef FFTC_DRV_DEBUG
    if(!pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;            
#endif
        
    CSL_FINS (pFFTCLldObj->cfgRegs->EOI, FFTC_EOI_EOI_VAL, eoiVal); 
    
    /* return success. */
    return 0;           
}

/**
 * ============================================================================
 *  @n@b Fftc_readEoiReg
 *
 *  @b  brief
 *  @n  This API reads the contents of FFTC End of Interrupt (EOI) Register and 
 *      returns its contents to the caller.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *      
 *  @return     int32_t
 *  @li                     -1  -   Invalid input handle.
 *  @li                     >=0 -   EOI value read from the FFTC End of Interrupt Register.            
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. 
 *
 *  @post
 *  @n  None.
 * 
 *  @code
    Fftc_LldObj                 fftcLldObj;

    ...

    Fftc_readEoiReg (&fftcLldObj);

    ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_readEoiReg 
(
    Fftc_LldObj*                pFFTCLldObj
)
{
#ifdef FFTC_DRV_DEBUG
    if (!pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    return CSL_FEXT (pFFTCLldObj->cfgRegs->EOI, FFTC_EOI_EOI_VAL); 
}

/**
 * ============================================================================
 *  @n@b Fftc_clearQueueClippingDetectReg
 *
 *  @b  brief
 *  @n  This API clears the contents (clipping count) of the FFTC Queue  
 *      Clipping Detect Register for a given queue id.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *      
 *  @param[in]    
        qNum                FFTC Queue ID for which the clipping count must be reset. 
                            
 *
 *  @return     void
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The input parameter 'qNum' must 
 *      be a valid queue id between 0 and 3.
 *
 *  @post
 *  @n  FFTC Queue X Clipping Detect Register contents cleared.
 * 
 *  @code
    Fftc_LldObj                 fftcLldObj;
    ...

    if (Fftc_clearQueueClippingDetectReg (&fftcLldObj, 1) != 0)
    {
        // FFTC queue 1 clipping detect register clear failed.
    }
    else
    {
        // FFTC Queue 1 clipping detect register clear successful.
    }
    ...
        
     @endcode
 * ============================================================================
 */
void Fftc_clearQueueClippingDetectReg 
(
    Fftc_LldObj*                pFFTCLldObj, 
    Fftc_QueueId                qNum
)
{
#ifdef FFTC_DRV_DEBUG
    if (!pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return;            
#endif

    CSL_FINS (pFFTCLldObj->cfgRegs->CLIP_Q[qNum], FFTC_CLIP_Q_CLIPPING_COUNT, 1); 
    
    /* Return success. */
    return;           
}

/**
 * ============================================================================
 *  @n@b Fftc_readQueueClippingDetectReg
 *
 *  @b  brief
 *  @n  This API reads the contents of FFTC Queue x Clipping Detect Register and 
 *      returns its contents, the clipping counter to the caller.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *      
 *  @param[in]    
        qNum                FFTC Queue ID for which the clipping count must be read. 
 *      
 *  @return     int32_t
 *                      -1  -   Invalid FFTC LLD instance object handle.
 *  @li                 >0  -   Clipping count on success, i.e., the number of 
 *                              FFT blocks where one or more clipping events were 
 *                              detected for the queue id specified.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The input parameter 'qNum' must 
 *      be a valid queue id between 0 and 3.
 *
 *  @post
 *  @n  None.
 * 
 *  @code
    Fftc_LldObj                 fftcLldObj;
    uint32_t                    clipping_count;
    ...

    if ((clipping_count = Fftc_readQueueClippingDetectReg (&fftcLldObj, 1)) == -1)
    {
        // queue 1 clipping detect register read failed.
    }
    else
    {
        // retrieved clipping count successfully.
    }
    ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_readQueueClippingDetectReg 
(
    Fftc_LldObj*                pFFTCLldObj, 
    Fftc_QueueId                qNum
)
{
#ifdef FFTC_DRV_DEBUG
    if (!pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;            
#endif

    return CSL_FEXT (pFFTCLldObj->cfgRegs->CLIP_Q[qNum], FFTC_CLIP_Q_CLIPPING_COUNT); 
}

/**
 * ============================================================================
 *  @n@b Fftc_readBlockDestQStatusReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Block X Destination Queue Status Register for the 
 *      last three blocks processed by FFT engine and returns the retrieved status 
 *      in the output parameter passed 'pFFTDestQStatus'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *      
 *  @param[out]
        pFFTDestQStatus     Output parameter handle that will be filled in with 
                            all the blocks destination queue register status.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input parameters.
 *  @li                     0   -   Successfully populated 'pFFTDestQStatus'
 *                                  with block status.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output param handle passed 
 *      'pFFTDestQStatus' must be a valid pointer and must have been allocated 
 *      space enough to hold the destination queue status for all the three 
 *      blocks/buffers.
 *
 *  @post
 *  @n  'pFFTDestQStatus' is populated with the FFTC Block X Destination Queue
 *      Status Register contents for all three blocks/buffers.
 * 
 *  @code
 *      Fftc_LldObj             fftcLldObj;
        Fftc_DestQStatusReg     destQStatus[FFTC_NUM_INTERNAL_BUFFERS];

        ...

        // Retrieve the destination queue status register for all the
        // three blocks/buffers.
        if(Fftc_readBlockDestQStatusReg (&fftcLldObj, destQStatus) != 0)
        {
            // FFT destination queue status read failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readBlockDestQStatusReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_DestQStatusReg*        pFFTDestQStatus
)
{
    int32_t                     tmpWord [3];        

#ifdef FFTC_DRV_DEBUG
    /* Valid the output parameter handle passed. */        
    if (!pFFTDestQStatus || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* Get a snapshot of the block status registers */
    tmpWord[0] =   pFFTCLldObj->cfgRegs->B0_DEST_STAT;
    tmpWord[1] =   pFFTCLldObj->cfgRegs->B1_DEST_STAT;
    tmpWord[2] =   pFFTCLldObj->cfgRegs->B2_DEST_STAT;

    pFFTDestQStatus[0].bOutputFFTShift      =   CSL_FEXT (tmpWord[0], FFTC_B0_DEST_STAT_FFTC_SHIFT_LEFT_RIGHT_OUTPUT);
    pFFTDestQStatus[0].bInputFFTShift       =   CSL_FEXT (tmpWord[0], FFTC_B0_DEST_STAT_FFTC_SHIFT_LEFT_RIGHT_INPUT);
    pFFTDestQStatus[0].inputShiftVal        =   CSL_FEXT (tmpWord[0], FFTC_B0_DEST_STAT_FFTC_VARIABLE_SHIFT_INPUT);
    pFFTDestQStatus[0].cppiDestQNum         =   CSL_FEXT (tmpWord[0], FFTC_B0_DEST_STAT_DEFAULT_DEST);

    pFFTDestQStatus[1].bOutputFFTShift      =   CSL_FEXT (tmpWord[1], FFTC_B1_DEST_STAT_FFTC_SHIFT_LEFT_RIGHT_OUTPUT);
    pFFTDestQStatus[1].bInputFFTShift       =   CSL_FEXT (tmpWord[1], FFTC_B1_DEST_STAT_FFTC_SHIFT_LEFT_RIGHT_INPUT);
    pFFTDestQStatus[1].inputShiftVal        =   CSL_FEXT (tmpWord[1], FFTC_B1_DEST_STAT_FFTC_VARIABLE_SHIFT_INPUT);
    pFFTDestQStatus[1].cppiDestQNum         =   CSL_FEXT (tmpWord[1], FFTC_B1_DEST_STAT_DEFAULT_DEST);

    pFFTDestQStatus[2].bOutputFFTShift      =   CSL_FEXT (tmpWord[2], FFTC_B2_DEST_STAT_FFTC_SHIFT_LEFT_RIGHT_OUTPUT);
    pFFTDestQStatus[2].bInputFFTShift       =   CSL_FEXT (tmpWord[2], FFTC_B2_DEST_STAT_FFTC_SHIFT_LEFT_RIGHT_INPUT);
    pFFTDestQStatus[2].inputShiftVal        =   CSL_FEXT (tmpWord[2], FFTC_B2_DEST_STAT_FFTC_VARIABLE_SHIFT_INPUT);
    pFFTDestQStatus[2].cppiDestQNum         =   CSL_FEXT (tmpWord[2], FFTC_B2_DEST_STAT_DEFAULT_DEST);

    return 0;        
}


/**
 * ============================================================================
 *  @n@b Fftc_readBlockShiftStatusReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Block X Scaling & Shifting Status Register for the 
 *      last three blocks processed by FFT engine and returns the retrieved status 
 *      in the output parameter passed 'pFFTShiftStatus'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[out]
        pFFTShiftStatus     Output parameter handle that will be filled in with 
                            all the blocks scaling and shifting register status.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input parameters.
 *  @li                     0   -   Successfully populated 'pFFTShiftStatus'
 *                                  with block status.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output param handle passed 
 *      'pFFTShiftStatus' must be a valid pointer and must have been allocated 
 *      space enough to hold the scaling & shifting status for all the three blocks/buffers.
 *
 *  @post
 *  @n  'pFFTShiftStatus' is populated with the FFTC Block X Scaling and Shifting 
 *      Status register contents for all three buffers.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ScalingShiftingStatusReg   shiftStatus[FFTC_NUM_INTERNAL_BUFFERS];

        ...

        // Retrieve the scaling and shifting status register for all the
        // three blocks/buffers.
        if(Fftc_readBlockShiftStatusReg (&fftcLldObj, shiftStatus) != 0)
        {
            // FFTC scaling and shifting status read failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readBlockShiftStatusReg 
(
    Fftc_LldObj*                        pFFTCLldObj,
    Fftc_ScalingShiftingStatusReg*      pFFTShiftStatus
)
{
    int32_t                             tmpWord [3];        

#ifdef FFTC_DRV_DEBUG
    /* Valid the output parameter handle passed. */        
    if (!pFFTShiftStatus || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* Get a snapshot of the block status registers */
    tmpWord[0] =   pFFTCLldObj->cfgRegs->B0_SHIFT_STAT;
    tmpWord[1] =   pFFTCLldObj->cfgRegs->B1_SHIFT_STAT;
    tmpWord[2] =   pFFTCLldObj->cfgRegs->B2_SHIFT_STAT;

    pFFTShiftStatus[0].bDynamicScaleEnable  =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_DYNAMIC_SCALING_ENABLE);
    pFFTShiftStatus[0].outputScaleVal       =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_OUTPUT_SCALING);
    pFFTShiftStatus[0].radixScalingValLast  =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_OUT_SCALING);
    pFFTShiftStatus[0].radixScalingVal[6]   =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_6_SCALING);
    pFFTShiftStatus[0].radixScalingVal[5]   =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_5_SCALING);
    pFFTShiftStatus[0].radixScalingVal[4]   =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_4_SCALING);
    pFFTShiftStatus[0].radixScalingVal[3]   =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_3_SCALING);
    pFFTShiftStatus[0].radixScalingVal[2]   =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_2_SCALING);
    pFFTShiftStatus[0].radixScalingVal[1]   =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_1_SCALING);
    pFFTShiftStatus[0].radixScalingVal[0]   =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_0_SCALING);
    pFFTShiftStatus[0].freqShiftScaleVal    =   CSL_FEXT (tmpWord[0], FFTC_B0_SHIFT_STAT_STAGE_LTE_SHIFT_SCALING);

                                                    
    pFFTShiftStatus[1].bDynamicScaleEnable  =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_DYNAMIC_SCALING_ENABLE);
    pFFTShiftStatus[1].outputScaleVal       =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_OUTPUT_SCALING);
    pFFTShiftStatus[1].radixScalingValLast  =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_OUT_SCALING);
    pFFTShiftStatus[1].radixScalingVal[6]   =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_6_SCALING);
    pFFTShiftStatus[1].radixScalingVal[5]   =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_5_SCALING);
    pFFTShiftStatus[1].radixScalingVal[4]   =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_4_SCALING);
    pFFTShiftStatus[1].radixScalingVal[3]   =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_3_SCALING);
    pFFTShiftStatus[1].radixScalingVal[2]   =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_2_SCALING);
    pFFTShiftStatus[1].radixScalingVal[1]   =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_1_SCALING);
    pFFTShiftStatus[1].radixScalingVal[0]   =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_0_SCALING);
    pFFTShiftStatus[1].freqShiftScaleVal    =   CSL_FEXT (tmpWord[1], FFTC_B1_SHIFT_STAT_STAGE_LTE_SHIFT_SCALING);
                                                        

    pFFTShiftStatus[2].bDynamicScaleEnable  =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_DYNAMIC_SCALING_ENABLE);
    pFFTShiftStatus[2].outputScaleVal       =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_OUTPUT_SCALING);
    pFFTShiftStatus[2].radixScalingValLast  =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_OUT_SCALING);
    pFFTShiftStatus[2].radixScalingVal[6]   =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_6_SCALING);
    pFFTShiftStatus[2].radixScalingVal[5]   =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_5_SCALING);
    pFFTShiftStatus[2].radixScalingVal[4]   =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_4_SCALING);
    pFFTShiftStatus[2].radixScalingVal[3]   =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_3_SCALING);
    pFFTShiftStatus[2].radixScalingVal[2]   =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_2_SCALING);
    pFFTShiftStatus[2].radixScalingVal[1]   =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_1_SCALING);
    pFFTShiftStatus[2].radixScalingVal[0]   =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_0_SCALING);
    pFFTShiftStatus[2].freqShiftScaleVal    =   CSL_FEXT (tmpWord[2], FFTC_B2_SHIFT_STAT_STAGE_LTE_SHIFT_SCALING);

    return 0;        
}


/**
 * ============================================================================
 *  @n@b Fftc_readBlockCyclicPrefixStatusReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Block X Cyclic Prefix Status Register for the 
 *      last three blocks processed by FFT engine and returns the 
 *      retrieved status in the output parameter passed 'pFFTCyclicStatus'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[out]
        pFFTCyclicStatus    Output parameter handle that will be filled in with 
                            all the blocks cyclic prefix status register.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input parameters.
 *  @li                     0   -   Successfully populated 'pFFTCyclicStatus'
 *                                  with block status.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output param handle passed 
 *      'pFFTCyclicStatus' must be a valid pointer and must have been allocated 
 *      space enough to hold the cyclic prefix status for all the three blocks/buffers.
 *
 *  @post
 *  @n  'pFFTCyclicStatus' is populated with the FFTC Block X Cyclic Prefix
 *      Status register contents for all three buffers.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_CyclicPrefixStatusReg      cyclicStatus[FFTC_NUM_INTERNAL_BUFFERS];

        ...

        // Retrieve the cyclic prefix status register for all the
        // three blocks/buffers.
        if(Fftc_readBlockCyclicPrefixStatusReg (&fftcLldObj, cyclicStatus) != 0)
        {
            // FFTC cyclic prefix status read failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readBlockCyclicPrefixStatusReg 
(
    Fftc_LldObj*                    pFFTCLldObj,
    Fftc_CyclicPrefixStatusReg*     pFFTCyclicStatus
)
{
    int32_t                         tmpWord [3];        

#ifdef FFTC_DRV_DEBUG
    /* Valid the output parameter handle passed. */        
    if (!pFFTCyclicStatus || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* Get a snapshot of the block status registers */
    tmpWord[0] =   pFFTCLldObj->cfgRegs->B0_PREFIX_STAT;
    tmpWord[1] =   pFFTCLldObj->cfgRegs->B1_PREFIX_STAT;
    tmpWord[2] =   pFFTCLldObj->cfgRegs->B2_PREFIX_STAT;

    pFFTCyclicStatus[0].bCyclicPrefixRemoveEnable   =   CSL_FEXT (tmpWord[0], FFTC_B0_PREFIX_STAT_CYCLIC_PREFIX_REMOVE_EN);
    pFFTCyclicStatus[0].cyclicPrefixRemoveNum       =   CSL_FEXT (tmpWord[0], FFTC_B0_PREFIX_STAT_CYCLIC_PREFIX_REMOVE_OFFSET);
    pFFTCyclicStatus[0].cyclicPrefixAddNum          =   CSL_FEXT (tmpWord[0], FFTC_B0_PREFIX_STAT_CYCLIC_PREFIX_ADDITION);
 
    pFFTCyclicStatus[1].bCyclicPrefixRemoveEnable   =   CSL_FEXT (tmpWord[1], FFTC_B1_PREFIX_STAT_CYCLIC_PREFIX_REMOVE_EN);
    pFFTCyclicStatus[1].cyclicPrefixRemoveNum       =   CSL_FEXT (tmpWord[1], FFTC_B1_PREFIX_STAT_CYCLIC_PREFIX_REMOVE_OFFSET);
    pFFTCyclicStatus[1].cyclicPrefixAddNum          =   CSL_FEXT (tmpWord[1], FFTC_B1_PREFIX_STAT_CYCLIC_PREFIX_ADDITION);

    pFFTCyclicStatus[2].bCyclicPrefixRemoveEnable   =   CSL_FEXT (tmpWord[2], FFTC_B2_PREFIX_STAT_CYCLIC_PREFIX_REMOVE_EN);
    pFFTCyclicStatus[2].cyclicPrefixRemoveNum       =   CSL_FEXT (tmpWord[2], FFTC_B2_PREFIX_STAT_CYCLIC_PREFIX_REMOVE_OFFSET);
    pFFTCyclicStatus[2].cyclicPrefixAddNum          =   CSL_FEXT (tmpWord[2], FFTC_B2_PREFIX_STAT_CYCLIC_PREFIX_ADDITION);

    return 0;        
}


/**
 * ============================================================================
 *  @n@b Fftc_readBlockControlStatusReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Block X Control Status Register for the 
 *      last three blocks processed by FFT engine and returns the retrieved status 
 *      in the output parameter passed 'pFFTControlStatus'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[out]
        pFFTControlStatus   Output parameter handle that will be filled in with 
                            all the blocks control status register.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input parameters.
 *  @li                     0   -   Successfully populated 'pFFTControlStatus'
 *                                  with block status.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output param handle passed 
 *      'pFFTControlStatus' must be a valid pointer and must have been allocated 
 *      space enough to hold the control status for all the three blocks/buffers.
 *
 *  @post
 *  @n  'pFFTControlStatus' is populated with the FFTC Block X Control
 *      Status register contents for all three buffers.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ControlStatusReg           controlStatus[FFTC_NUM_INTERNAL_BUFFERS];

        ...

        // Retrieve the control status register for all the
        // three blocks/buffers.
        if(Fftc_readBlockControlStatusReg (&fftcLldObj, controlStatus) != 0)
        {
            // FFTC control status read failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readBlockControlStatusReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_ControlStatusReg*      pFFTControlStatus
)
{
    int32_t                     tmpWord [3];        

#ifdef FFTC_DRV_DEBUG
    /* Valid the output parameter handle passed. */        
    if (!pFFTControlStatus || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* Get a snapshot of the block status registers */
    tmpWord[0] =   pFFTCLldObj->cfgRegs->B0_CNTRL_STAT;
    tmpWord[1] =   pFFTCLldObj->cfgRegs->B1_CNTRL_STAT;
    tmpWord[2] =   pFFTCLldObj->cfgRegs->B2_CNTRL_STAT;

    pFFTControlStatus[0].dftSize            =   CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_DFT_SIZE);
    pFFTControlStatus[0].dftMode            =   (Fftc_DFTMode) CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_DFT_IDFT_SELECT);
    pFFTControlStatus[0].bSupressSideInfo   =   CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_SUPPRESSED_SIDE_INFO);
    pFFTControlStatus[0].inputQNum          =   (Fftc_QueueId) CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_INPUT_QUEUE_NUM);
    pFFTControlStatus[0].bIsSOP             =   CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_SOP);
    pFFTControlStatus[0].bIsEOP             =   CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_EOP);
    pFFTControlStatus[0].bIsBlockError      =   CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_BLOCK_ERROR);
    pFFTControlStatus[0].zeroPadFactor      =   CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_ZERO_PAD_VAL);
    pFFTControlStatus[0].zeroPadMode        =   (Fftc_ZeroPadMode) CSL_FEXT (tmpWord[0], FFTC_B0_CNTRL_STAT_ZERO_PAD_MODE);

    pFFTControlStatus[1].dftSize            =   CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_DFT_SIZE);
    pFFTControlStatus[1].dftMode            =   (Fftc_DFTMode) CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_DFT_IDFT_SELECT);
    pFFTControlStatus[1].bSupressSideInfo   =   CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_SUPPRESSED_SIDE_INFO);
    pFFTControlStatus[1].inputQNum          =   (Fftc_QueueId) CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_INPUT_QUEUE_NUM);
    pFFTControlStatus[1].bIsSOP             =   CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_SOP);
    pFFTControlStatus[1].bIsEOP             =   CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_EOP);
    pFFTControlStatus[1].bIsBlockError      =   CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_BLOCK_ERROR);
    pFFTControlStatus[1].zeroPadFactor      =   CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_ZERO_PAD_VAL);
    pFFTControlStatus[1].zeroPadMode        =   (Fftc_ZeroPadMode) CSL_FEXT (tmpWord[1], FFTC_B1_CNTRL_STAT_ZERO_PAD_MODE);


    pFFTControlStatus[2].dftSize            =   CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_DFT_SIZE);
    pFFTControlStatus[2].dftMode            =   (Fftc_DFTMode) CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_DFT_IDFT_SELECT);
    pFFTControlStatus[2].bSupressSideInfo   =   CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_SUPPRESSED_SIDE_INFO);
    pFFTControlStatus[2].inputQNum          =   (Fftc_QueueId) CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_INPUT_QUEUE_NUM);
    pFFTControlStatus[2].bIsSOP             =   CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_SOP);
    pFFTControlStatus[2].bIsEOP             =   CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_EOP);
    pFFTControlStatus[2].bIsBlockError      =   CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_BLOCK_ERROR);
    pFFTControlStatus[2].zeroPadFactor      =   CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_ZERO_PAD_VAL);
    pFFTControlStatus[2].zeroPadMode        =   (Fftc_ZeroPadMode) CSL_FEXT (tmpWord[2], FFTC_B2_CNTRL_STAT_ZERO_PAD_MODE);

    return 0;        
}


/**
 * ============================================================================
 *  @n@b Fftc_readBlockFreqShiftStatusReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Block X LTE Frequency Shift Status Register for the 
 *      last three blocks processed by FFT engine and returns the retrieved status 
 *      in the output parameter passed 'pFFTFreqShiftStatus'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[out]
        pFFTFreqShiftStatus Output parameter handle that will be filled in with 
                            all the blocks frequency shift status register contents.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input parameters.
 *  @li                     0   -   Successfully populated 'pFFTFreqShiftStatus'
 *                                  with block status.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output param handle passed 
 *      'pFFTFreqShiftStatus' must be a valid pointer and must have been allocated 
 *      space enough to hold the frequency shift status for all the three blocks/buffers.
 *
 *  @post
 *  @n  'pFFTFreqShiftStatus' is populated with the FFTC Block X LTE Frequency Shift
 *      Status register contents for all three buffers.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_FreqShiftStatusReg         freqShiftStatus[FFTC_NUM_INTERNAL_BUFFERS];

        ...

        // Retrieve the frequency shift status register for all the
        // three blocks/buffers.
        if(Fftc_readBlockFreqShiftStatusReg (&fftcLldObj, freqShiftStatus) != 0)
        {
            // FFTC LTE Frequency shift status read failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readBlockFreqShiftStatusReg 
(
    Fftc_LldObj*                    pFFTCLldObj,
    Fftc_FreqShiftStatusReg*        pFFTFreqShiftStatus
    
)
{
    int32_t                         tmpWord [3];        

#ifdef FFTC_DRV_DEBUG
    /* Valid the output parameter handle passed. */        
    if (!pFFTFreqShiftStatus || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* Get a snapshot of the block status registers */
    tmpWord[0] =   pFFTCLldObj->cfgRegs->B0_FREQ_STAT;
    tmpWord[1] =   pFFTCLldObj->cfgRegs->B1_FREQ_STAT;
    tmpWord[2] =   pFFTCLldObj->cfgRegs->B2_FREQ_STAT;

    pFFTFreqShiftStatus[0].bFreqShiftEnable     =   CSL_FEXT (tmpWord[0], FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_EN);
    pFFTFreqShiftStatus[0].freqShiftIndex       =   (Fftc_FreqShiftIndex) CSL_FEXT (tmpWord[0], FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_INDEX);             
    pFFTFreqShiftStatus[0].freqShiftInitPhase   =   CSL_FEXT (tmpWord[0], FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_PHASE);
    pFFTFreqShiftStatus[0].freqShiftMultFactor  =   CSL_FEXT (tmpWord[0], FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_FACTOR);
    pFFTFreqShiftStatus[0].freqShiftDirection   =   (Fftc_FreqShiftDir) CSL_FEXT (tmpWord[0], FFTC_B0_FREQ_STAT_LTE_FREQ_SHIFT_DIR);


    pFFTFreqShiftStatus[1].bFreqShiftEnable     =   CSL_FEXT (tmpWord[1], FFTC_B1_FREQ_STAT_LTE_FREQ_SHIFT_EN);
    pFFTFreqShiftStatus[1].freqShiftIndex       =   (Fftc_FreqShiftIndex) CSL_FEXT (tmpWord[1], FFTC_B1_FREQ_STAT_LTE_FREQ_SHIFT_INDEX);             
    pFFTFreqShiftStatus[1].freqShiftInitPhase   =   CSL_FEXT (tmpWord[1], FFTC_B1_FREQ_STAT_LTE_FREQ_SHIFT_PHASE);
    pFFTFreqShiftStatus[1].freqShiftMultFactor  =   CSL_FEXT (tmpWord[1], FFTC_B1_FREQ_STAT_LTE_FREQ_SHIFT_FACTOR);
    pFFTFreqShiftStatus[1].freqShiftDirection   =   (Fftc_FreqShiftDir) CSL_FEXT (tmpWord[1], FFTC_B1_FREQ_STAT_LTE_FREQ_SHIFT_DIR);

    pFFTFreqShiftStatus[2].bFreqShiftEnable     =   CSL_FEXT (tmpWord[2], FFTC_B2_FREQ_STAT_LTE_FREQ_SHIFT_EN);
    pFFTFreqShiftStatus[2].freqShiftIndex       =   (Fftc_FreqShiftIndex) CSL_FEXT (tmpWord[2], FFTC_B2_FREQ_STAT_LTE_FREQ_SHIFT_INDEX);             
    pFFTFreqShiftStatus[2].freqShiftInitPhase   =   CSL_FEXT (tmpWord[2], FFTC_B2_FREQ_STAT_LTE_FREQ_SHIFT_PHASE);
    pFFTFreqShiftStatus[2].freqShiftMultFactor  =   CSL_FEXT (tmpWord[2], FFTC_B2_FREQ_STAT_LTE_FREQ_SHIFT_FACTOR);
    pFFTFreqShiftStatus[2].freqShiftDirection   =   (Fftc_FreqShiftDir) CSL_FEXT (tmpWord[2], FFTC_B2_FREQ_STAT_LTE_FREQ_SHIFT_DIR);

    /* Return success. */
    return 0;
}


/**
 * ============================================================================
 *  @n@b Fftc_readBlockPktSizeStatusReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Block X Packet Size Status Register for the 
 *      last three blocks processed by FFT engine and returns the retrieved status 
 *      in the output parameter passed 'pFFTPktSizeStatus'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[out]
        pFFTPktSizeStatus   Output parameter handle that will be filled in with 
                            all the blocks packet size status register contents.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input parameters.
 *  @li                     0   -   Successfully populated 'pFFTPktSizeStatus'
 *                                  with block status.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output param handle passed 
 *      'pFFTPktSizeStatus' must be a valid pointer and must have been allocated 
 *      space enough to hold the packet size status for all the three blocks/buffers.
 *
 *  @post
 *  @n  'pFFTPktSizeStatus' is populated with the FFTC Block X Packet Size
 *      Status register contents for all three buffers.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_PktSizeStatusReg           pktSizeStatus[FFTC_NUM_INTERNAL_BUFFERS];

        ...

        // Retrieve the packet size status register for all the
        // three blocks/buffers.
        if(Fftc_readBlockPktSizeStatusReg (&fftcLldObj, pktSizeStatus) != 0)
        {
            // FFTC packet size status read failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readBlockPktSizeStatusReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_PktSizeStatusReg*      pFFTPktSizeStatus
)
{
#ifdef FFTC_DRV_DEBUG
    /* Valid the output parameter handle passed. */        
    if (!pFFTPktSizeStatus || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    pFFTPktSizeStatus[0].pktSize    =   CSL_FEXT (pFFTCLldObj->cfgRegs->B0_PSIZE_STAT, FFTC_B0_PSIZE_STAT_PACKET_SIZE);
    pFFTPktSizeStatus[1].pktSize    =   CSL_FEXT (pFFTCLldObj->cfgRegs->B1_PSIZE_STAT, FFTC_B1_PSIZE_STAT_PACKET_SIZE);
    pFFTPktSizeStatus[2].pktSize    =   CSL_FEXT (pFFTCLldObj->cfgRegs->B2_PSIZE_STAT, FFTC_B2_PSIZE_STAT_PACKET_SIZE);

    /* Return Success. */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_readBlockTagStatusReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Block X Tag Status Register for the 
 *      last three blocks processed by FFT engine and returns the 
 *      retrieved status in the output parameter passed 'pFFTTagStatus'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[out]
        pFFTTagStatus       Output parameter handle that will be filled in with 
                            all the blocks tag status register contents.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input parameters.
 *  @li                     0   -   Successfully populated 'pFFTTagStatus'
 *                                  with block status.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output param handle passed 
 *      'pFFTTagStatus' must be a valid pointer and must have been allocated 
 *      space enough to hold the tag status for all the three blocks/buffers.
 *
 *  @post
 *  @n  'pFFTTagStatus' is populated with the FFTC Block X Tag Status register 
 *      contents for all three buffers.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_TagStatusReg               tagStatus[FFTC_NUM_INTERNAL_BUFFERS];

        ...

        // Retrieve the tag status register for all the
        // three blocks/buffers.
        if(Fftc_readBlockTagStatusReg (&fftcLldObj, tagStatus) != 0)
        {
            // FFTC tag status read failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readBlockTagStatusReg 
(
    Fftc_LldObj*            pFFTCLldObj,
    Fftc_TagStatusReg*      pFFTTagStatus
)
{
    int32_t                 tmpWord [3];

#ifdef FFTC_DRV_DEBUG
    /* Valid the output parameter handle passed. */        
    if (!pFFTTagStatus || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* Get a snapshot of block status registers */
    tmpWord [0] =   pFFTCLldObj->cfgRegs->B0_DESTTAG_STAT;
    tmpWord [1] =   pFFTCLldObj->cfgRegs->B1_DESTTAG_STAT;
    tmpWord [2] =   pFFTCLldObj->cfgRegs->B2_DESTTAG_STAT;

    pFFTTagStatus[0].destTag    =   CSL_FEXT (tmpWord [0], FFTC_B0_DESTTAG_STAT_DEST_TAG);        
    pFFTTagStatus[0].flowId     =   CSL_FEXT (tmpWord [0], FFTC_B0_DESTTAG_STAT_FLOW_ID);        
    pFFTTagStatus[0].srcId      =   CSL_FEXT (tmpWord [0], FFTC_B0_DESTTAG_STAT_SRC_ID);        

    pFFTTagStatus[1].destTag    =   CSL_FEXT (tmpWord [1], FFTC_B1_DESTTAG_STAT_DEST_TAG);        
    pFFTTagStatus[1].flowId     =   CSL_FEXT (tmpWord [1], FFTC_B1_DESTTAG_STAT_FLOW_ID);        
    pFFTTagStatus[1].srcId      =   CSL_FEXT (tmpWord [1], FFTC_B1_DESTTAG_STAT_SRC_ID);        

    pFFTTagStatus[2].destTag    =   CSL_FEXT (tmpWord [2], FFTC_B2_DESTTAG_STAT_DEST_TAG);        
    pFFTTagStatus[2].flowId     =   CSL_FEXT (tmpWord [2], FFTC_B2_DESTTAG_STAT_FLOW_ID);        
    pFFTTagStatus[2].srcId      =   CSL_FEXT (tmpWord [2], FFTC_B2_DESTTAG_STAT_SRC_ID);        

    /* Return success. */
    return 0;
}


/******************************************************************************
 ******************   FFTC ERROR REGISTER ACCESS APIs    **********************
 ******************************************************************************/

/**
 * ============================================================================
 *  @n@b Fftc_readErrorIntRawStatusReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Error Interrupt Raw Status and Set Register
 *      contents to retrieve a snapshot of all the errors that have been 
 *      encountered by the FFTC engine irrespective of whether the corresponding
 *      interrupt is enabled.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[in]    
        qNum                FFTC Queue Number (0-3) for which the error interrupt
                            status needs to be retrieved.
 *  @param[out]    
        pErrorCfg           Output error params structure that will be filled in
                            by this API with the various errors raw status.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid output param handle passed / invalid
 *                                  instance handle passed.
 *  @li                     0   -   Success.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output error params structure 
 *      handle 'pErrorCfg' passed must be a valid pointer.
 *
 *  @post
 *  @n  None
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ErrorParams                errorStatus;
        
        ...

        if (Fftc_readErrorIntRawStatusReg (&fftcLldObj, 0, &errorStatus) != 0)
        {
            // Error retrieving FFTC error status.
            // Check the input handle we passed.
            ...
        }
        else
        {  
            // Error Status retrieved successfully.
            ...
        }

        ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_readErrorIntRawStatusReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_QueueId                qNum, 
    Fftc_ErrorParams*           pErrorCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the output parameter handle to which the 
     * error interrupt raw status and set register contents will 
     * be pupulated.
     */
    if (!pErrorCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;     
#endif

    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
                
            pErrorCfg->bIsIntOnEOP             =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T0);
            pErrorCfg->bIsDebugHalt            =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T0);
            pErrorCfg->bIsConfigWordError      =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T0);
            pErrorCfg->bIsDescBufferError      =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T0);
            pErrorCfg->bIsEopError             =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_EOP_ERROR_STATUS_T0);
            pErrorCfg->bIsConfigInvalError     =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T0);
                
            break;
        }
        case    Fftc_QueueId_1:
        {
            pErrorCfg->bIsIntOnEOP             =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T1);
            pErrorCfg->bIsDebugHalt            =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T1);
            pErrorCfg->bIsConfigWordError      =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T1);
            pErrorCfg->bIsDescBufferError      =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T1);
            pErrorCfg->bIsEopError             =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_EOP_ERROR_STATUS_T1);
            pErrorCfg->bIsConfigInvalError     =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T1);
                
            break;
        }
        case    Fftc_QueueId_2:
        {
            pErrorCfg->bIsIntOnEOP             =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T2);
            pErrorCfg->bIsDebugHalt            =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T2);
            pErrorCfg->bIsConfigWordError      =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T2);
            pErrorCfg->bIsDescBufferError      =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T2);
            pErrorCfg->bIsEopError             =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_EOP_ERROR_STATUS_T2);
            pErrorCfg->bIsConfigInvalError     =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T2);
                
            break;
        }
        case    Fftc_QueueId_3:
        {
            pErrorCfg->bIsIntOnEOP             =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_INT_ON_EOP_STATUS_T3);
            pErrorCfg->bIsDebugHalt            =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_DEBUG_HALT_STATUS_T3);
            pErrorCfg->bIsConfigWordError      =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_CONFIG_WORD_ERROR_STATUS_T3);
            pErrorCfg->bIsDescBufferError      =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_DESC_BUFFER_ERROR_STATUS_T3);
            pErrorCfg->bIsEopError             =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_EOP_ERROR_STATUS_T3);
            pErrorCfg->bIsConfigInvalError     =   CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_STAT, FFTC_ERROR_STAT_CONFIG_INVALID_ERROR_STATUS_T3);
                
            break;
        }
        default:
        {
            /* Invalid FFTC Tx Queue Number specified. */
            return -1;
        }
    }

    /* FFTC Error Interrupt Raw Status and Set Register retrieval succesful */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_clearErrorIntRawStatusReg
 *
 *  @b  brief
 *  @n  This API clears the various error status bits of FFTC Error Interrupt Raw 
 *      Status and Set Register corresponding to the bit fields enabled in the
 *      input error params structure.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[in]    
        qNum                FFTC Queue Number (0-3) for which the error status
                            bits needs to be cleared.
 *  @param[in]
        pErrorCfg           Input error params structure that specifies which of
                            the error status bits need to be cleared.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input error params/instance handle passed.
 *  @li                     0   -   Successfully cleared the status bits.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The input error params structure 
 *      handle passed must be a valid pointer with all its fields set up to 
 *      0/1 to indicate whether to clear the corresponding error bit in the register.
 *
 *  @post
 *  @n  FFTC Error Interrupt Raw Status and Set Register contents
 *      cleared as per input provided in 'pErrorCfg'.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ErrorParams                errorParams;
        
        errorParams.bIsIntOnEOP        =   0;
        errorParams.bIsDebugHalt       =   1;
        errorParams.bIsConfigWordError =   1;
        errorParams.bIsDescBufferError =   1;
        errorParams.bIsEopError        =   0;
        errorParams.bIsConfigInvalError=   0;

        if (Fftc_clearErrorIntRawStatusReg (&fftcLldObj, 0, &errorParams) != 0)
        {
            // Error clearing FFTC error status.
            // Check the input handle we passed.
            ...
        }
        else
        {  
            // Error Status cleared successfully.
            ...
        }

        ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_clearErrorIntRawStatusReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_QueueId                qNum, 
    Fftc_ErrorParams*           pErrorCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the input parameter handle */
    if (!pErrorCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;        
#endif

    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_INT_ON_EOP_CLR_T0, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_DEBUG_HALT_CLR_T0, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_CONFIG_WORD_ERROR_CLR_T0, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_DESC_BUFFER_ERROR_CLR_T0, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_EOP_ERROR_CLR_T0, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_CONFIG_INVALID_ERROR_CLR_T0, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_1:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_INT_ON_EOP_CLR_T1, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_DEBUG_HALT_CLR_T1, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_CONFIG_WORD_ERROR_CLR_T1, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_DESC_BUFFER_ERROR_CLR_T1, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_EOP_ERROR_CLR_T1, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_CONFIG_INVALID_ERROR_CLR_T1, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_2:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_INT_ON_EOP_CLR_T2, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_DEBUG_HALT_CLR_T2, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_CONFIG_WORD_ERROR_CLR_T2, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_DESC_BUFFER_ERROR_CLR_T2, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_EOP_ERROR_CLR_T2, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_CONFIG_INVALID_ERROR_CLR_T2, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_3:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_INT_ON_EOP_CLR_T3, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_DEBUG_HALT_CLR_T3, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_CONFIG_WORD_ERROR_CLR_T3, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_DESC_BUFFER_ERROR_CLR_T3, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_EOP_ERROR_CLR_T3, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_CLR, FFTC_ERROR_CLR_CONFIG_INVALID_ERROR_CLR_T3, pErrorCfg->bIsConfigInvalError);

            break;
        }
        default:
        {
            /* Invalid FFTC Tx Queue Number specified. */
            return -1;
        }
    }

    /* FFTC Error Interrupt Clear Register configuration succesful */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_writeErrorIntEnableSetReg
 *
 *  @b  brief
 *  @n  This API sets up the interrupt enable status bits for the various FFTC 
 *      errors corresponding to the bit fields enabled in the input error configuration
 *      structure. 
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[in]    
        qNum                FFTC Queue Number (0-3) for which the interrupt error 
                            status bits needs to be set.
 *  @param[in]   
        pErrorCfg           Input error params structure that specifies the error 
                            status bits for which the interrupt needs to be
                            enabled.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input error params/instance handle passed.
 *  @li                     0   -   Successfully setup the interrupt status bits.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. Input param handle 'pErrorCfg' 
 *      must be a valid pointer with all fields configured to either 0/1 to 
 *      indicate which of the error bits need to be setup for interrupts in 
 *      the register.
 *
 *  @post
 *  @n  FFTC Error Interrupt Enable and Set Register configured.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ErrorParams                errorParams;
        
        errorParams.bIsIntOnEOP        =   0;
        errorParams.bIsDebugHalt       =   1;
        errorParams.bIsConfigWordError =   1;
        errorParams.bIsDescBufferError =   1;
        errorParams.bIsEopError        =   0;
        errorParams.bIsConfigInvalError=   0;

        if (Fftc_writeErrorIntEnableSetReg (&fftcLldObj, 0, &errorParams) != 0)
        {
            // Error setting up FFTC error interrupt enable status.
            // Check the input handle we passed.
            ...
        }
        else
        {  
            // Error Interrupt Status setup successfully.
            ...
        }

        ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_writeErrorIntEnableSetReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_QueueId                qNum, 
    Fftc_ErrorParams*           pErrorCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the input handle */        
    if (!pErrorCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_INT_ON_EOP_EN_T0, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DEBUG_HALT_EN_T0, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T0, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T0, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_EOP_ERROR_EN_T0, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T0, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_1:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_INT_ON_EOP_EN_T1, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DEBUG_HALT_EN_T1, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T1, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T1, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_EOP_ERROR_EN_T1, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T1, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_2:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_INT_ON_EOP_EN_T2, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DEBUG_HALT_EN_T2, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T2, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T2, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_EOP_ERROR_EN_T2, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T2, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_3:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_INT_ON_EOP_EN_T3, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DEBUG_HALT_EN_T3, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T3, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T3, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_EOP_ERROR_EN_T3, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T3, pErrorCfg->bIsConfigInvalError);

            break;
        }
        default:
        {
            /* Invalid FFTC Tx Queue Number specified. */
            return -1;
        }
    }

    /* FFTC Error Interrupt Enable and Set Register configuration succesful */
    return 0;
}


/**
 * ============================================================================
 *  @n@b Fftc_readErrorIntEnableSetReg
 *
 *  @b  brief
 *  @n  This API reads the FFTC Error and Interrupt Enable and Set register and
 *      populates its contents into the output error configuration structure. 
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[in]    
        qNum                FFTC Queue Number (0-3) for which the interrupt error 
                            configuration bits needs to be read.
 *  @param[out]   
        pErrorCfg           Output error params structure that will need to be filled
                            with the register contents.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid output error params/instance handle passed.
 *  @li                     0   -   Successfully read the interrupt enable bits.   
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. Output param handle 'pErrorCfg' must 
 *      be a valid pointer and must have been allocated memory before passing to 
 *      this API.
 *
 *  @post
 *  @n  FFTC Error Interrupt Enable and Set Register read and its values populated
 *      into the 'pErroCfg' output param structure.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ErrorParams                errorParams;
       
        // Read the error configuration for queue 0 
        if (Fftc_readErrorIntEnableSetReg (&fftcLldObj, 0, &errorParams) != 0)
        {
            // Error reading FFTC error interrupt enable set register for queue 0.
            // Check the output handle we passed.
            ...
        }
        else
        {  
            // Error Interrupt enable register contents successfully
            // populated in 'errorParams'
            ...
        }

        ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_readErrorIntEnableSetReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_QueueId                qNum, 
    Fftc_ErrorParams*           pErrorCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the output handle */        
    if (!pErrorCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
            pErrorCfg->bIsIntOnEOP         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_INT_ON_EOP_EN_T0);
            pErrorCfg->bIsDebugHalt        = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DEBUG_HALT_EN_T0);
            pErrorCfg->bIsConfigWordError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T0);
            pErrorCfg->bIsDescBufferError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T0);
            pErrorCfg->bIsEopError         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_EOP_ERROR_EN_T0);
            pErrorCfg->bIsConfigInvalError = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T0);

            break;
        }
        case    Fftc_QueueId_1:
        {
            pErrorCfg->bIsIntOnEOP         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_INT_ON_EOP_EN_T1);
            pErrorCfg->bIsDebugHalt        = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DEBUG_HALT_EN_T1);
            pErrorCfg->bIsConfigWordError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T1);
            pErrorCfg->bIsDescBufferError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T1);
            pErrorCfg->bIsEopError         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_EOP_ERROR_EN_T1);
            pErrorCfg->bIsConfigInvalError = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T1);

            break;
        }
        case    Fftc_QueueId_2:
        {
            pErrorCfg->bIsIntOnEOP         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_INT_ON_EOP_EN_T2);
            pErrorCfg->bIsDebugHalt        = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DEBUG_HALT_EN_T2);
            pErrorCfg->bIsConfigWordError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T2);
            pErrorCfg->bIsDescBufferError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T2);
            pErrorCfg->bIsEopError         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_EOP_ERROR_EN_T2);
            pErrorCfg->bIsConfigInvalError = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T2);

            break;
        }
        case    Fftc_QueueId_3:
        {
            pErrorCfg->bIsIntOnEOP         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_INT_ON_EOP_EN_T3);
            pErrorCfg->bIsDebugHalt        = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DEBUG_HALT_EN_T3);
            pErrorCfg->bIsConfigWordError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_WORD_ERROR_EN_T3);
            pErrorCfg->bIsDescBufferError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_DESC_BUFFER_ERROR_EN_T3);
            pErrorCfg->bIsEopError         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_EOP_ERROR_EN_T3);
            pErrorCfg->bIsConfigInvalError = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_EN_CONFIG_INVALID_ERROR_EN_T3);

            break;
        }
        default:
        {
            /* Invalid FFTC Tx Queue Number specified. */
            return -1;
        }
    }

    /* FFTC Error Interrupt Enable and Set Register Read succesful */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_clearErrorIntEnableReg
 *
 *  @b  brief
 *  @n  This API clears the various interrupt enable bits of FFTC Error Interrupt 
 *      Enable and Set Register corresponding to the bit fields enabled in the
 *      input error params structure.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[in]    
        qNum                FFTC Queue Number (0-3) for which the interrupt error 
                            bits needs to be cleared.
 *  @param[in]
        pErrorCfg           Input error params structure that specifies which of
                            the interrupt error bits need to be cleared.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input error params/instance handle passed.
 *  @li                     0   -   Successfully cleared the interrupt enable bits. 
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The input error params handle 
 *      'pErrorCfg' must be a valid pointer with all its bit field positions 
 *      initialized to either 0/1 to indicate if the corresponding interrupt 
 *      error bit must be cleared in the register.
 *      
 *  @post
 *  @n  FFTC Error Interrupt Enable and Set Register contents cleared.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ErrorParams                errorParams;
        
        errorParams.bIsIntOnEOP        =   0;
        errorParams.bIsDebugHalt       =   1;
        errorParams.bIsConfigWordError =   1;
        errorParams.bIsDescBufferError =   1;
        errorParams.bIsEopError        =   0;
        errorParams.bIsConfigInvalError=   0;

        if (Fftc_clearErrorIntEnableReg (&fftcLldObj, 0, &errorParams) != 0)
        {
            // Error clearing FFTC error interrupt enable register.
            // Check the input handle we passed.
            ...
        }
        else
        {  
            // Error interrupt enable cleared successfully.
            ...
        }

        ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_clearErrorIntEnableReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_QueueId                qNum, 
    Fftc_ErrorParams*           pErrorCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the input handle */        
    if (!pErrorCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_INT_ON_EOP_EN_CLR_T0, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_DEBUG_HALT_EN_CLR_T0, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_CONFIG_WORD_ERROR_EN_CLR_T0, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_DESC_BUFFER_ERROR_EN_CLR_T0, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_EOP_ERROR_EN_CLR_T0, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_CONFIG_INVALID_ERROR_EN_CLR_T0, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_1:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_INT_ON_EOP_EN_CLR_T1, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_DEBUG_HALT_EN_CLR_T1, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_CONFIG_WORD_ERROR_EN_CLR_T1, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_DESC_BUFFER_ERROR_EN_CLR_T1, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_EOP_ERROR_EN_CLR_T1, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_CONFIG_INVALID_ERROR_EN_CLR_T1, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_2:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_INT_ON_EOP_EN_CLR_T2, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_DEBUG_HALT_EN_CLR_T2, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_CONFIG_WORD_ERROR_EN_CLR_T2, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_DESC_BUFFER_ERROR_EN_CLR_T2, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_EOP_ERROR_EN_CLR_T2, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_CONFIG_INVALID_ERROR_EN_CLR_T2, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_3:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_INT_ON_EOP_EN_CLR_T3, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_DEBUG_HALT_EN_CLR_T3, pErrorCfg->bIsDebugHalt);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_CONFIG_WORD_ERROR_EN_CLR_T3, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_DESC_BUFFER_ERROR_EN_CLR_T3, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_EOP_ERROR_EN_CLR_T3, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN_CLR, FFTC_ERROR_EN_CLR_CONFIG_INVALID_ERROR_EN_CLR_T3, pErrorCfg->bIsConfigInvalError);

            break;
        }
        default:
        {
            /* Invalid FFTC Tx Queue Number specified. */
            return -1;
        }
    }

    /* FFTC Error Interrupt Enable and Set Register configuration succesful */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_writeHaltOnErrorReg
 *
 *  @b  brief
 *  @n  This API sets up the Halt bit for the various FFTC errors corresponding 
 *      to the bit fields enabled in the input error configuration structure.
 *      The Halt bit indicates if the FFTC engine should halt when the corresponding 
 *      error in FFTC Error Interrupt Enables Status register is encountered. 
 *
 *      Note: The 'bIsDebugHalt' bit value of the input error params structure is
 *      ignored and is always set to 1 in the FFTC Halt on error register, since
 *      a Debug Halt always generates a halt whether or not enabled here.
 *       
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[in]    
        qNum                FFTC Queue Number (0-3) for which the interrupt halt 
                            bit needs to be set.
 *  @param[in]   
        pErrorCfg           Input error params structure that specifies the error 
                            status bits for which the fftc halt needs to be
                            enabled.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input error params/instance handle passed.
 *  @li                     0   -   Successfully setup the interrupt halt bits. 
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. Input param handle pErrorCfg must 
 *      be a valid pointer with all fields configured to either 0/1 to indicate 
 *      which of the error bits need to be setup for fftc halts in the register.
 *
 *  @post
 *  @n  FFTC Halt on Error Register configured.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ErrorParams                errorParams;
        
        errorParams.bIsIntOnEOP        =   0;
        errorParams.bIsConfigWordError =   1;
        errorParams.bIsDescBufferError =   1;
        errorParams.bIsEopError        =   0;
        errorParams.bIsConfigInvalError=   0;

        if (Fftc_writeHaltOnErrorReg (&fftcLldObj, 0, &errorParams) != 0)
        {
            // Error setting up FFTC halt on error register.
            // Check the input handle we passed.
            ...
        }
        else
        {  
            // halt on error setup successful.
            ...
        }

        ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_writeHaltOnErrorReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_QueueId                qNum, 
    Fftc_ErrorParams*           pErrorCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the input handle */        
    if (!pErrorCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_INT_ON_EOP_HALT_T0, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DEBUG_HARD_STOP_HALT_T0, 1);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_WORD_ERROR_HALT_T0, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DESC_BUFFER_ERROR_HALT_T0, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_EOP_ERROR_HALT_T0, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_INVALID_ERROR_HALT_T0, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_1:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_INT_ON_EOP_HALT_T1, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DEBUG_HARD_STOP_HALT_T1, 1);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_WORD_ERROR_HALT_T1, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DESC_BUFFER_ERROR_HALT_T1, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_EOP_ERROR_HALT_T1, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_INVALID_ERROR_HALT_T1, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_2:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_INT_ON_EOP_HALT_T2, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DEBUG_HARD_STOP_HALT_T2, 1);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_WORD_ERROR_HALT_T2, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DESC_BUFFER_ERROR_HALT_T2, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_EOP_ERROR_HALT_T2, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_INVALID_ERROR_HALT_T2, pErrorCfg->bIsConfigInvalError);

            break;
        }
        case    Fftc_QueueId_3:
        {
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_INT_ON_EOP_HALT_T3, pErrorCfg->bIsIntOnEOP);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DEBUG_HARD_STOP_HALT_T3, 1);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_WORD_ERROR_HALT_T3, pErrorCfg->bIsConfigWordError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DESC_BUFFER_ERROR_HALT_T3, pErrorCfg->bIsDescBufferError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_EOP_ERROR_HALT_T3, pErrorCfg->bIsEopError);
            CSL_FINS (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_INVALID_ERROR_HALT_T3, pErrorCfg->bIsConfigInvalError);

            break;
        }
        default:
        {
            /* Invalid FFTC Tx Queue Number specified. */
            return -1;
        }
    }

    /* FFTC Halt on Error Register read succesful */
    return 0;
}


/**
 * ============================================================================
 *  @n@b Fftc_readHaltOnErrorReg
 *
 *  @b  brief
 *  @n  This API reads the Halt bit for the various FFTC errors from the FFTC Halt
 *      On Error Register. The Halt bit indicates if the FFTC engine should halt 
 *      when the corresponding error in FFTC Error Interrupt Enables Status 
 *      register is encountered. 
 *
 *      Note: The 'bIsDebugHalt' bit value of the output error params structure is
 *      always set to 1, since a Debug Halt always generates a halt whether or 
 *      not enabled here.
 *       
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 *      
 *  @param[in]    
        qNum                FFTC Queue Number (0-3) for which the interrupt halt 
                            bit needs to be read.
 *  @param[out]   
        pErrorCfg           Output error params structure with contents of the
                            FFTC Halt on error register.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid output error params/instance handle passed.
 *  @li                     0   -   Successfully read the interrupt halt bits.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. Output param handle 'pErrorCfg' 
 *      must be a valid pointer and must have been allocated memory before 
 *      passing to this API.
 *
 *  @post
 *  @n  FFTC Halt on Error Register contents populated into 'pErroCfg' output
 *      params structure.
 * 
 *  @code
 *      Fftc_LldObj                     fftcLldObj;
        Fftc_ErrorParams                errorParams;
       
        // read Halt on Error register configuration for queue 0
        if (Fftc_readHaltOnErrorReg (&fftcLldObj, 0, &errorParams) != 0)
        {
            // Error reading FFTC halt on error register.
            // Check the input handle we passed.
            ...
        }
        else
        {  
            // halt on error read successful.
            ...
        }

        ...
        
     @endcode
 * ============================================================================
 */
int32_t Fftc_readHaltOnErrorReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    Fftc_QueueId                qNum, 
    Fftc_ErrorParams*           pErrorCfg
)
{
#ifdef FFTC_DRV_DEBUG
    /* Validate the output handle */        
    if (!pErrorCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
            pErrorCfg->bIsIntOnEOP         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_INT_ON_EOP_HALT_T0);
            pErrorCfg->bIsDebugHalt        = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DEBUG_HARD_STOP_HALT_T0);
            pErrorCfg->bIsConfigWordError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_WORD_ERROR_HALT_T0);
            pErrorCfg->bIsDescBufferError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DESC_BUFFER_ERROR_HALT_T0);
            pErrorCfg->bIsEopError         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_EOP_ERROR_HALT_T0);
            pErrorCfg->bIsConfigInvalError = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_INVALID_ERROR_HALT_T0);

            break;
        }
        case    Fftc_QueueId_1:
        {
            pErrorCfg->bIsIntOnEOP         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_INT_ON_EOP_HALT_T1);
            pErrorCfg->bIsDebugHalt        = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DEBUG_HARD_STOP_HALT_T1);
            pErrorCfg->bIsConfigWordError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_WORD_ERROR_HALT_T1);
            pErrorCfg->bIsDescBufferError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DESC_BUFFER_ERROR_HALT_T1);
            pErrorCfg->bIsEopError         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_EOP_ERROR_HALT_T1);
            pErrorCfg->bIsConfigInvalError = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_INVALID_ERROR_HALT_T1);

            break;
        }
        case    Fftc_QueueId_2:
        {
            pErrorCfg->bIsIntOnEOP         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_INT_ON_EOP_HALT_T2);
            pErrorCfg->bIsDebugHalt        = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DEBUG_HARD_STOP_HALT_T2);
            pErrorCfg->bIsConfigWordError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_WORD_ERROR_HALT_T2);
            pErrorCfg->bIsDescBufferError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DESC_BUFFER_ERROR_HALT_T2);
            pErrorCfg->bIsEopError         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_EOP_ERROR_HALT_T2);
            pErrorCfg->bIsConfigInvalError = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_INVALID_ERROR_HALT_T2);

            break;
        }
        case    Fftc_QueueId_3:
        {
            pErrorCfg->bIsIntOnEOP         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_INT_ON_EOP_HALT_T3);
            pErrorCfg->bIsDebugHalt        = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DEBUG_HARD_STOP_HALT_T3);
            pErrorCfg->bIsConfigWordError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_WORD_ERROR_HALT_T3);
            pErrorCfg->bIsDescBufferError  = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_DESC_BUFFER_ERROR_HALT_T3);
            pErrorCfg->bIsEopError         = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_EOP_ERROR_HALT_T3);
            pErrorCfg->bIsConfigInvalError = CSL_FEXT (pFFTCLldObj->cfgRegs->ERROR_EN, FFTC_ERROR_HALT_CONFIG_INVALID_ERROR_HALT_T3);

            break;
        }
        default:
        {
            /* Invalid FFTC Tx Queue Number specified. */
            return -1;
        }
    }

    /* FFTC Halt on Error Register read succesful */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Fftc_writeQueueConfigRegs
 *
 *  @b  brief
 *  @n  This API configures the FFTC Queue Specific Registers :- Queue x
 *      Destination queue Register, Queue x Scaling and Shifting Register,
 *      Queue x Cyclic Prefix Register, Queue x Control Register and Queue x
 *      LTE Frequency Shift Register for any given queue id 'qNum' using the
 *      input specified by 'pFFTLocalCfg'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *      
 *  @param[in]
        qNum                Specifies which of the 4 FFTC queues (0-3) MMRs to 
                            configure

 *  @param[in]
        pFFTLocalCfg        Input parameter handle that holds the local configuration
                            settings for the specific queue processing.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid input handle.
 *  @li                     0   -   Successfully configured the registers.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The input queue local configuration 
 *      param handle passed must be a valid pointer and should contain valid values 
 *      as defined by the FFTC User Guide.
 *
 *  @post
 *  @n  Queue x Destination queue Register, 
 *      Queue x Scaling and Shifting Register,
 *      Queue x Cyclic Prefix Register, 
 *      Queue x Control Register and 
 *      Queue x LTE Frequency Shift Register configured.
 * 
 *  @code
        Fftc_LldObj                 fftcLldObj;
        Fftc_QLocalCfg              fftLocalCfg;

        ...

        // Setup the queue configuration parameters
        fftLocalCfg.destQRegConfig.cppiDestQNum     =   3;
        fftLocalCfg.destQRegConfig.bOutputFFTShift  =   0;
        ...
        fftLocalCfg.controlRegConfig.dftSize        =   512;
        fftLocalCfg.controlRegConfig.dftMode        =   Fftc_DFTMode_DFT;
        ...

        // Configure the queue 0 MMRs.
        if(Fftc_writeQueueConfigRegs (&fftcLldObj, 0, &fftLocalCfg) != 0)
        {
            // FFT queue 0 configuration failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_writeQueueConfigRegs 
(
    Fftc_LldObj*                pFFTCLldObj, 
    Fftc_QueueId                qNum, 
    Fftc_QLocalCfg*             pFFTLocalCfg
)
{
    int32_t                     dftIndex, word[5] = {0, 0, 0, 0, 0};

#ifdef FFTC_DRV_DEBUG
    /* Validate Input */        
    if (!pFFTLocalCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* The FFTC Local Configuration consists of configuration of the following 
     * queue specific registers:
     *  1. FFTC Queue x Destination Queue Register
     *  2. FFTC Queue x Scaling & Shifting Register
     *  3. FFTC Queue x Cyclic Prefix Register
     *  4. FFTC Queue x Control Register
     *  5. FFTC Queue x LTE Frequency Shift Register
     */
    /* Step 1. Configure the FFTC Queue 'n' Destination Queue Register  */
    word[0] =   CSL_FMK (FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_OUTPUT, 
                        pFFTLocalCfg->destQRegConfig.bOutputFFTShift) |
                CSL_FMK (FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_INPUT,
                        pFFTLocalCfg->destQRegConfig.bInputFFTShift) |
                CSL_FMK (FFTC_Q0_DEST_FFTC_VARIABLE_SHIFT_INPUT,
                        pFFTLocalCfg->destQRegConfig.inputShiftVal) |
                CSL_FMK (FFTC_Q0_DEST_DEFAULT_DEST, 
                        pFFTLocalCfg->destQRegConfig.cppiDestQNum);

    /* Step 2. Configure the Queue 'n' Scaling and Shift Register configuration. */
    word [1] =  CSL_FMK (FFTC_Q0_SCALE_SHIFT_DYNAMIC_SCALING_ENABLE, 
                        pFFTLocalCfg->scalingShiftingRegConfig.bDynamicScaleEnable) |
                CSL_FMK (FFTC_Q0_SCALE_SHIFT_OUTPUT_SCALING, 
                        pFFTLocalCfg->scalingShiftingRegConfig.outputScaleVal);

    /* The scaling factors at various butterfly stages are only configurable
     * when FFT is configured to be in "Static" mode.
     */
    if (!pFFTLocalCfg->scalingShiftingRegConfig.bDynamicScaleEnable)
    {
        word [1] |= CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_OUT_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.radixScalingValLast) |
                    CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_6_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[6]) |
                    CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_5_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[5]) |            
                    CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_4_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[4]) |            
                    CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_3_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[3]) |
                    CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_2_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[2]) |
                    CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_1_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[1]) |
                    CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_0_SCALING, 
                            pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[0]);            

        /* Configure the LTE Frequency Shift Scaling only if LTE Frequency Shift is enabled */
        if (pFFTLocalCfg->freqShiftRegConfig.bFreqShiftEnable)
        {
            word [1] |= CSL_FMK (FFTC_Q0_SCALE_SHIFT_STAGE_LTE_SHIFT_SCALING, 
                                pFFTLocalCfg->scalingShiftingRegConfig.freqShiftScaleVal);
        }
    }

    /* Step 3. Configure Queue 'n' Cyclic Prefix Register */
    word [2] =  CSL_FMK (FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_EN, 
                        pFFTLocalCfg->cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable);
    if (pFFTLocalCfg->cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable)
    {
        word [2] |= CSL_FMK (FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_OFFSET, 
                            pFFTLocalCfg->cyclicPrefixRegConfig.cyclicPrefixRemoveNum); 
    }
    if (pFFTLocalCfg->cyclicPrefixRegConfig.bCyclicPrefixAddEnable)
    {
        word [2] |= CSL_FMK (FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_ADDITION, 
                            pFFTLocalCfg->cyclicPrefixRegConfig.cyclicPrefixAddNum);
    }            

    /* Step 4. Configure Queue 'n' Control Register */
    word [3] =  CSL_FMK (FFTC_Q0_CONTROL_SUPPRESSED_SIDE_INFO, 
                        pFFTLocalCfg->controlRegConfig.bSupressSideInfo) |
                CSL_FMK (FFTC_Q0_CONTROL_DFT_IDFT_SELECT, 
                        pFFTLocalCfg->controlRegConfig.dftMode);
    if (pFFTLocalCfg->controlRegConfig.bZeroPadEnable)
    {
        word [3] |= CSL_FMK (FFTC_Q0_CONTROL_ZERO_PAD_MODE, 
                            pFFTLocalCfg->controlRegConfig.zeroPadMode) |
                    CSL_FMK (FFTC_Q0_CONTROL_ZERO_PAD_VAL, 
                            pFFTLocalCfg->controlRegConfig.zeroPadFactor);
    }

    /* Get the corresponding DFT index for the DFT block size provided. */
    if ((dftIndex = Fftc_mapDFTSizeToIndex (pFFTLocalCfg->controlRegConfig.dftSize)) != -1)
    {
        word [3] |= CSL_FMK (FFTC_Q0_CONTROL_DFT_SIZE, dftIndex);
    }
    else
    {
        /* Invalid DFT size specified. Return error */            
        return -1;            
    }            

    /* Step 5. Finally configure the Queue 'n' LTE Frequency Shift Register */
    word [4] =  CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_EN, 
                        pFFTLocalCfg->freqShiftRegConfig.bFreqShiftEnable);              
    if (pFFTLocalCfg->freqShiftRegConfig.bFreqShiftEnable)
    {
        word [4] |= CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_DIR, 
                            pFFTLocalCfg->freqShiftRegConfig.freqShiftDirection) |
                    CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_FACTOR, 
                            pFFTLocalCfg->freqShiftRegConfig.freqShiftMultFactor) |
                    CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_PHASE, 
                            pFFTLocalCfg->freqShiftRegConfig.freqShiftInitPhase) |              
                    CSL_FMK (FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_INDEX, 
                            pFFTLocalCfg->freqShiftRegConfig.freqShiftIndex);              
    }

    /* Configure the Queue registers for the queue number specified */
    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
            pFFTCLldObj->cfgRegs->Q0_DEST           =   word[0];              
            pFFTCLldObj->cfgRegs->Q0_SCALE_SHIFT    =   word[1];              
            pFFTCLldObj->cfgRegs->Q0_CYCLIC_PREFIX  =   word[2];              
            pFFTCLldObj->cfgRegs->Q0_CONTROL        =   word[3];              
            pFFTCLldObj->cfgRegs->Q0_LTE_FREQ       =   word[4];              

            break;
        }
        case    Fftc_QueueId_1:
        {
            pFFTCLldObj->cfgRegs->Q1_DEST           =   word[0];              
            pFFTCLldObj->cfgRegs->Q1_SCALE_SHIFT    =   word[1];              
            pFFTCLldObj->cfgRegs->Q1_CYCLIC_PREFIX  =   word[2];              
            pFFTCLldObj->cfgRegs->Q1_CONTROL        =   word[3];              
            pFFTCLldObj->cfgRegs->Q1_LTE_FREQ       =   word[4];              

            break;
        }
        case    Fftc_QueueId_2:
        {
            pFFTCLldObj->cfgRegs->Q2_DEST           =   word[0];              
            pFFTCLldObj->cfgRegs->Q2_SCALE_SHIFT    =   word[1];              
            pFFTCLldObj->cfgRegs->Q2_CYCLIC_PREFIX  =   word[2];              
            pFFTCLldObj->cfgRegs->Q2_CONTROL        =   word[3];              
            pFFTCLldObj->cfgRegs->Q2_LTE_FREQ       =   word[4];              

            break;
        }
        case    Fftc_QueueId_3:
        {
            pFFTCLldObj->cfgRegs->Q3_DEST           =   word[0];              
            pFFTCLldObj->cfgRegs->Q3_SCALE_SHIFT    =   word[1];              
            pFFTCLldObj->cfgRegs->Q3_CYCLIC_PREFIX  =   word[2];              
            pFFTCLldObj->cfgRegs->Q3_CONTROL        =   word[3];              
            pFFTCLldObj->cfgRegs->Q3_LTE_FREQ       =   word[4];              

            break;
        }
        default:
        {
            /* Can never be here. */
            return -1;               
        }
    }

    /* Configuration Done. Return Success. */
    return  0;
}

/**
 * ============================================================================
 *  @n@b Fftc_readQueueConfigRegs
 *
 *  @b  brief
 *  @n  This API reads the FFTC Queue Specific Registers :- Queue x
 *      Destination queue Register, Queue x Scaling and Shifting Register,
 *      Queue x Cyclic Prefix Register, Queue x Control Register and Queue x
 *      LTE Frequency Shift Register for a given queue id 'qNum' and fills in 
 *      the retrieved configuration in the output parameter passed 'pFFTLocalCfg'.
 *      
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object.
 *      
 *  @param[in]
        qNum                Specifies which of the 4 FFTC queues (0-3) MMRs to 
                            read

 *  @param[out]
        pFFTLocalCfg        Output parameter handle that will be filled in with the
                            queue local configuration snapshot read from the MMRs for
                            the specified FFTC queue.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid output parameter handle 'pFFTLocalCfg'/
 *                                  invalid FFTC LLD object instance handle.
 *  @li                     0   -   Successfully populated 'pFFTLocalCfg' with a 
 *                                  snapshot of queue local MMR config.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The output queue local configuration 
 *      param handle passed must be a valid pointer.
 *
 *  @post
 *  @n  'pFFTLocalCfg' is populated with the configuration from the following queue
 *      local configuration registers:
 *      Queue x Destination queue Register, 
 *      Queue x Scaling and Shifting Register,
 *      Queue x Cyclic Prefix Register, 
 *      Queue x Control Register and 
 *      Queue x LTE Frequency Shift Register.
 * 
 *  @code
 *      Fftc_LldObj                 fftcLldObj;
        Fftc_QLocalCfg              qCfg;

        ...

        // Retrieve the queue 0 MMRs configuration.
        if(Fftc_readQueueConfigRegs (&fftcLldObj, 0, &qCfg) != 0)
        {
            // FFT queue 0 configuration read failed.
            // do error recovery.
            ...
        }
        else
        {
            // Continue Processing
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readQueueConfigRegs 
(
    Fftc_LldObj*                pFFTCLldObj, 
    Fftc_QueueId                qNum, 
    Fftc_QLocalCfg*             pFFTLocalCfg
)
{
    int32_t                     dftIndex, word [5] = {0, 0, 0, 0, 0};

#ifdef FFTC_DRV_DEBUG
    /* Validate Input */        
    if (!pFFTLocalCfg || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* The FFTC Local Configuration consists of configuration of the following 
     * queue specific registers:
     *  1. FFTC Queue x Destination Queue Register
     *  2. FFTC Queue x Scaling & Shifting Register
     *  3. FFTC Queue x Cyclic Prefix Register
     *  4. FFTC Queue x Control Register
     *  5. FFTC Queue x LTE Frequency Shift Register
     */
    switch (qNum)
    {
        case    Fftc_QueueId_0:
        {
            word[0] =   pFFTCLldObj->cfgRegs->Q0_DEST;              
            word[1] =   pFFTCLldObj->cfgRegs->Q0_SCALE_SHIFT;              
            word[2] =   pFFTCLldObj->cfgRegs->Q0_CYCLIC_PREFIX;              
            word[3] =   pFFTCLldObj->cfgRegs->Q0_CONTROL;              
            word[4] =   pFFTCLldObj->cfgRegs->Q0_LTE_FREQ;              

            break;
        }
        case    Fftc_QueueId_1:
        {
            word[0] =   pFFTCLldObj->cfgRegs->Q1_DEST;              
            word[1] =   pFFTCLldObj->cfgRegs->Q1_SCALE_SHIFT;              
            word[2] =   pFFTCLldObj->cfgRegs->Q1_CYCLIC_PREFIX;              
            word[3] =   pFFTCLldObj->cfgRegs->Q1_CONTROL;              
            word[4] =   pFFTCLldObj->cfgRegs->Q1_LTE_FREQ;              

            break;
        }
        case    Fftc_QueueId_2:
        {
            word[0] =   pFFTCLldObj->cfgRegs->Q2_DEST;              
            word[1] =   pFFTCLldObj->cfgRegs->Q2_SCALE_SHIFT;              
            word[2] =   pFFTCLldObj->cfgRegs->Q2_CYCLIC_PREFIX;              
            word[3] =   pFFTCLldObj->cfgRegs->Q2_CONTROL;              
            word[4] =   pFFTCLldObj->cfgRegs->Q2_LTE_FREQ;              

            break;
        }
        case    Fftc_QueueId_3:
        {
            word[0] =   pFFTCLldObj->cfgRegs->Q3_DEST;              
            word[1] =   pFFTCLldObj->cfgRegs->Q3_SCALE_SHIFT;              
            word[2] =   pFFTCLldObj->cfgRegs->Q3_CYCLIC_PREFIX;              
            word[3] =   pFFTCLldObj->cfgRegs->Q3_CONTROL;              
            word[4] =   pFFTCLldObj->cfgRegs->Q3_LTE_FREQ;              

            break;
        }
        default:
        {
            /* Can never be here. */
            return -1;               
        }
    }

    /* Step 1. Read the FFTC Queue 'n' Destination Queue Register  */
    pFFTLocalCfg->destQRegConfig.bOutputFFTShift                =   CSL_FEXT (word [0], FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_OUTPUT);
    pFFTLocalCfg->destQRegConfig.bInputFFTShift                 =   CSL_FEXT (word [0], FFTC_Q0_DEST_FFTC_SHIFT_LEFT_RIGHT_INPUT);
    pFFTLocalCfg->destQRegConfig.inputShiftVal                  =   CSL_FEXT (word [0], FFTC_Q0_DEST_FFTC_VARIABLE_SHIFT_INPUT);
    pFFTLocalCfg->destQRegConfig.cppiDestQNum                   =   CSL_FEXT (word [0], FFTC_Q0_DEST_DEFAULT_DEST);

    /* Step 2. Read the Queue 'n' Scaling and Shift Register configuration. */
    pFFTLocalCfg->scalingShiftingRegConfig.bDynamicScaleEnable  =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_DYNAMIC_SCALING_ENABLE);            
    pFFTLocalCfg->scalingShiftingRegConfig.outputScaleVal       =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_OUTPUT_SCALING);
    pFFTLocalCfg->scalingShiftingRegConfig.radixScalingValLast  =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_OUT_SCALING);            
    pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[6]   =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_6_SCALING);            
    pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[5]   =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_5_SCALING);            
    pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[4]   =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_4_SCALING);            
    pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[3]   =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_3_SCALING);            
    pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[2]   =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_2_SCALING);            
    pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[1]   =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_1_SCALING);            
    pFFTLocalCfg->scalingShiftingRegConfig.radixScalingVal[0]   =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_0_SCALING);            
    pFFTLocalCfg->scalingShiftingRegConfig.freqShiftScaleVal    =   CSL_FEXT (word [1], FFTC_Q0_SCALE_SHIFT_STAGE_LTE_SHIFT_SCALING);

    /* Step 3. Read Queue 'n' Cyclic Prefix Register */
    pFFTLocalCfg->cyclicPrefixRegConfig.bCyclicPrefixRemoveEnable=   CSL_FEXT (word [2], FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_EN);
    pFFTLocalCfg->cyclicPrefixRegConfig.cyclicPrefixRemoveNum   =   CSL_FEXT (word [2], FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_REMOVE_OFFSET); 
    pFFTLocalCfg->cyclicPrefixRegConfig.cyclicPrefixAddNum      =   CSL_FEXT (word [2], FFTC_Q0_CYCLIC_PREFIX_CYCLIC_PREFIX_ADDITION);

    /* Step 4. Read Queue 'n' Control Register */
    pFFTLocalCfg->controlRegConfig.zeroPadMode                  =   (Fftc_ZeroPadMode) CSL_FEXT (word [3], FFTC_Q0_CONTROL_ZERO_PAD_MODE);
    pFFTLocalCfg->controlRegConfig.zeroPadFactor                =   CSL_FEXT (word [3], FFTC_Q0_CONTROL_ZERO_PAD_VAL);
    pFFTLocalCfg->controlRegConfig.bSupressSideInfo             =   CSL_FEXT (word [3], FFTC_Q0_CONTROL_SUPPRESSED_SIDE_INFO);
    pFFTLocalCfg->controlRegConfig.dftMode                      =   (Fftc_DFTMode) CSL_FEXT (word [3], FFTC_Q0_CONTROL_DFT_IDFT_SELECT);
    dftIndex                                                    =   CSL_FEXT (word [3], FFTC_Q0_CONTROL_DFT_SIZE);
    /* Get the corresponding DFT size for the DFT index read from the register. */
    pFFTLocalCfg->controlRegConfig.dftSize                      =   Fftc_DftBlockSizeTable [dftIndex];

    /* Step 5. Finally read the Queue 'n' LTE Frequency Shift Register */
    pFFTLocalCfg->freqShiftRegConfig.bFreqShiftEnable           =   CSL_FEXT (word [4], FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_EN);              
    pFFTLocalCfg->freqShiftRegConfig.freqShiftDirection         =   (Fftc_FreqShiftDir) CSL_FEXT (word [4], FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_DIR);
    pFFTLocalCfg->freqShiftRegConfig.freqShiftMultFactor        =   CSL_FEXT (word [4], FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_FACTOR);              
    pFFTLocalCfg->freqShiftRegConfig.freqShiftInitPhase         =   CSL_FEXT (word [4], FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_PHASE);              
    pFFTLocalCfg->freqShiftRegConfig.freqShiftIndex             =   (Fftc_FreqShiftIndex) CSL_FEXT (word [4], FFTC_Q0_LTE_FREQ_LTE_FREQ_SHIFT_INDEX);              
    /* Configuration Done. Return Success. */
    return  0;
}

/**
 * ============================================================================
 *  @n@b Fftc_writeDftSizeListGroupReg
 *
 *  @b  brief
 *  @n  This API compiles the FFTC DFT size list specified by 'pDftSizeList' and 
 *      'dftSizeListLen' input parameters and writes the DFT Size List Group 
 *      Registers accordingly.
 *
 *      This API doesnt validate the DFT block sizes specified in 'pDftSizeList'
 *      input parameter. 
 *
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 * 
 *  @param[in]
        pDftSizeList        List of DFT block sizes that needs to be programmed
                            to FFTC engine.

 *  @param[in]
        dftSizeListLen      Number of DFT block sizes specified in 'pDftSizeList'
                            input. The maximum number of FFT blocks supported by
                            hardware is 128. The caller needs to ensure that this
                            limit is not exceeded.

 *  @return     int32_t
 *  @li                     -1  -   Invalid configuration/handles provided.
 *  @li                     0   -   Successfully wrote the DFT size list to the 
 *                                  register.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The DFT block sizes passed in 
 *      'pDftSizeList' should be all legal values as per the FFTC user guide. 
 *
 *  @post
 *  @n  The DFT Size List Group Registers are programmed according to input passed.
 * 
 *  @code
 *      Fftc_LldObj         fftcLldObj;
        uint16_t            dftSizeList[32];
        uint32_t            numDftBlocks, i;

        ...

        // Setup the DFT size list configuration.
        numDftBlocks = 32;
        for (i = 0; i < numDftBlocks; i ++)
            dftSizeList [i] = xxx; // program the DFT block size in the same order as they
                                   // will appear in data.

        if (Fftc_writeDftSizeListGroupReg(&fftcLldObj, dftSizeList, numDftBlocks) != 0)
        {
            // Error returned by the API. 
            // exit
        }
        else
        {
            //DFT size list configuration successful.
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_writeDftSizeListGroupReg 
(
    Fftc_LldObj*                pFFTCLldObj, 
    uint16_t*                   pDftSizeList, 
    uint32_t                    dftSizeListLen
)
{
    uint32_t                    dftListGroupVal;
    uint32_t                    i, grpNum, blockNum = 0, numGroups, numSizesLeft;

#ifdef FFTC_DRV_DEBUG
    /* Validate input */
    if (!pDftSizeList || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* Each DFT Size List Group consists of 5 DFT sizes */
    numGroups           = dftSizeListLen / 5;     
    numSizesLeft        = dftSizeListLen % 5;

    /* Format DFT block sizes into groups of 5 sizes each. */
    for (grpNum = 0; grpNum < numGroups; grpNum ++)
    {
        /* Configure the DFT size list group register i with this value */
        pFFTCLldObj->cfgRegs->DFT_LIST_G [grpNum]   =  CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_0, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum])) |
                                            CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_1, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + 1])) |
                                            CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_2, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + 2])) |
                                            CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_3, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + 3])) |
                                            CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_4, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + 4]));

        blockNum += 5;
    }

    for (i = 0; i < numSizesLeft; i ++)
    {
        if (i == 0)
        {
            dftListGroupVal =   CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_0, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + i]));
        }
        else if (i == 1)
        {
            dftListGroupVal |=  CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_1, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + i]));
        }
        else if (i == 2)
        {
            dftListGroupVal |=  CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_2, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + i]));
        }
        else if (i == 3)
        {
            dftListGroupVal |=  CSL_FMK (FFTC_DFT_LIST_G_DFT_SIZE_3, Fftc_mapDFTSizeToIndex (pDftSizeList [blockNum + i]));
        }
    }
    if (numSizesLeft)
        pFFTCLldObj->cfgRegs->DFT_LIST_G [grpNum]   = dftListGroupVal;

    /* Return Success. */
    return 0;
}


/**
 * ============================================================================
 *  @n@b Fftc_readDftSizeListGroupReg
 *
 *  @b  brief
 *  @n  This API reads all of the 26 DFT Size List Group Registers and returns
 *      the contents of these registers to the caller in the output parameter
 *      'pDftSizeList'. 
 *
 *  @param[in]    
        pFFTCLldObj         FFTC LLD instance object. 
 * 
 *  @param[out]
        pDftSizeList        List of DFT block sizes read from the FFTC engine H/w.

 *  @return     int32_t
 *  @li                     -1  -   Invalid input configuration/handles provided.
 *  @li                     0   -   Successfully read the DFT size list to
 *                                  the output parameter 'pDftSizeList'.
 *
 *  @pre
 *  @n  @a Fftc_lldOpen () must be called to obtain the register overlay handle for 
 *      FFTC instance before calling this API. The application must have allocated 
 *      enough space for 'pDftSizeList' output parameter to hold all the 128 
 *      DFT block sizes from the hardware. Each DFT block size needs 16 bits to 
 *      represent it and there can be at most 128 block sizes, so the output 
 *      parameter should have been reserved space for 2 bytes (16 bits) * 128.
 *
 *  @post
 *  @n  The output parameter 'pDftSizeList' is populated with contents of DFT Size 
 *      List Group Registers.
 * 
 *  @code
 *      Fftc_LldObj                 fftcLldObj;  
        uint16_t                    dftSizeList[128];

        ...

        // Read the DFT size list configuration.
        if (Fftc_readDftSizeListGroupReg(&fftcLldObj, dftSizeList) != 0)
        {
            // Error returned by the API. 
            // exit
        }
        else
        {
            //DFT size list read successful.
            ...
        }
     @endcode
 * ============================================================================
 */
int32_t Fftc_readDftSizeListGroupReg 
(
    Fftc_LldObj*                pFFTCLldObj,
    uint16_t*                   pDftSizeList
)
{
    uint32_t                    grpNum, blockNum, numGroups;

#ifdef FFTC_DRV_DEBUG
    /* Validate input */
    if (!pDftSizeList || !pFFTCLldObj || !pFFTCLldObj->cfgRegs)
        return -1;
#endif

    /* Each DFT Size List Group consists of 5 DFT sizes 
     * except for the last group that contains only
     * 3 DFT sizes.
     * Number of DFT blocks = 128
     *
     * hence total number of groups = 128 / 5 + 1
     */
    numGroups           = FFTC_MAX_NUM_BLOCKS / 5 + 1;     

    blockNum = 0;

    /* Read contents of DFT size list group registers. */
    for (grpNum = 0; grpNum < numGroups; grpNum ++)
    {
        /* Configure the DFT size list group register i with this value */
        pDftSizeList [blockNum]             =   Fftc_DftBlockSizeTable [CSL_FEXT (pFFTCLldObj->cfgRegs->DFT_LIST_G [grpNum], 
                                                                        FFTC_DFT_LIST_G_DFT_SIZE_0)]; 
        pDftSizeList [blockNum + 1]         =   Fftc_DftBlockSizeTable [CSL_FEXT (pFFTCLldObj->cfgRegs->DFT_LIST_G [grpNum], 
                                                                        FFTC_DFT_LIST_G_DFT_SIZE_1)];
        pDftSizeList [blockNum + 2]         =   Fftc_DftBlockSizeTable [CSL_FEXT (pFFTCLldObj->cfgRegs->DFT_LIST_G [grpNum], 
                                                                        FFTC_DFT_LIST_G_DFT_SIZE_2)];

        /* All groups have 5 each DFT sizes except for the last group that has only 3. */
        if (grpNum != numGroups - 1)
        {
            pDftSizeList [blockNum + 3]     =   Fftc_DftBlockSizeTable [CSL_FEXT (pFFTCLldObj->cfgRegs->DFT_LIST_G [grpNum], 
                                                                        FFTC_DFT_LIST_G_DFT_SIZE_3)];
            pDftSizeList [blockNum + 4]     =   Fftc_DftBlockSizeTable [CSL_FEXT (pFFTCLldObj->cfgRegs->DFT_LIST_G [grpNum], 
                                                                        FFTC_DFT_LIST_G_DFT_SIZE_4)];
        }

        blockNum += 5;
    }

    /* Return Success. */
    return 0;
}

/**
@}
*/
