/**
 *   @file  multicore_app_managed.c
 *
 *   @brief  
 *      FFTC example that demonstrates usage of FFTC driver's higher layer
 *      APIs using application managed configuration in a multi-core scenario.
 *
 *      Uses FFTC, CPPI, QMSS pre-built libraries and BIOS/XDC as 
 *      OS. 
 *
 *      The example does the following:
 *          *   Master core sets up the system leve init.
 *          *   Each core does:
 *              *   Initializes and sets up the FFTC driver 
 *              *   Opens a Tx queue 0 and sets up with default configuration.
 *              *   Opens a Rx flow.
 *              *   Sets up request, result buffers using Host mode descriptors.
 *              *   Submits a FFTC request.
 *              *   Polls and waits on a reply from the FFT engine for it to process.
 *              *   Parses the reply and verifies it if its correct.
 *
 *      The multi-core setup is as follows:
 *          Submit Request by           Process Results by
 *         ---------------------      ---------------------
 *          core 0                      core 1
 *          core 1                      core 2
 *          core 2                      core 3
 *          core 3                      core 0
 *
 *          core 0 --> core 1 --> core 2 --> core 3 --> core 0
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
/* FFTC types include */
#include <fftc_types.h>

/* FFTC include */
#include <ti/drv/fftc/fftc.h>
#include <ti/drv/fftc/fftc_osal.h>

/* FFTC example application input data/ configuration files */
#include <fftc_cfg_16.h>

/* Chip Level definitions include */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_qm_queue.h>

/* PSC CSL definitions include */
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

/* CACHE CSL includes */
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

/* XDC/BIOS includes */
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h> 

/* QM Accumalator firmware include */
#include <ti/drv/qmss/qmss_firmware.h>

/* Standard definitions include */
#include <string.h>

/**************************************************************
************************** DEFINITIONS ************************
***************************************************************/

/** Number of host descriptors used by the FFTC example */
#define     FFTC_NUM_HOST_DESC          32

/** Host descriptor size. 
 *
 *  Big enough to hold the mandatory fields of the 
 *  host descriptor and 16 bytes of PS info if needed.
 * 
 *  = 32 Host desc + 16 PS Info = 48 bytes
 *  Round it off to next multiple of 16 = 64 bytes
 * 
 */
#define     FFTC_SIZE_HOST_DESC         64 

/** Size of each example sample in bytes 
 *
 *  = sizeof (Cplx16) = 4 bytes
 */
#define     FFTC_SAMPLE_SIZE            4

/** 
 *  @brief  Cplx16
 *
 *          Structure to represent the FFT data input/
 *          output format.
 */     
#ifndef xdc_target__bigEndian
typedef struct _Cplx16
{
    /** Imaginary part of the FFT data */        
    Int16       imag;

    /** Real part of the FFT data */        
    Int16       real;
} Cplx16;
#else
typedef struct _Cplx16
{
    /** Real part of the FFT data */        
    Int16       real;

    /** Imaginary part of the FFT data */        
    Int16       imag;
} Cplx16;
#endif

/** 
 *  @brief  FFT_ExampleCfg
 *
 *          Structure to hold the FFT example input vector,
 *          output vector.
 */     
typedef struct _FFT_ExampleCfg
{
    /** Number of FFT blocks in the example vector */        
    UInt32              numBlocks;

    /** FFT configuration for the example */        
    Fftc_QLocalCfg      fftcQCfg;

    /** Number of input example samples */        
    UInt32              numInputSamples;

    /** Input FFT example vector data */        
    Cplx16*             pFftInputData [FFTC_MAX_NUM_BLOCKS];

    /** Number of output example samples */        
    UInt32              numOutputSamples;

    /** Expected Output FFT result data */        
    Cplx16*             pFftOutputData [FFTC_MAX_NUM_BLOCKS];

    /** Expected Clipping detect value */
    UInt32              bClippingDetected [FFTC_MAX_NUM_BLOCKS];

    /** Expected Block exponent value */
    UInt32              blockExpVal [FFTC_MAX_NUM_BLOCKS];

    /** Expected cycle count value */
    UInt32              numClockCycles [FFTC_MAX_NUM_BLOCKS];
} FFT_ExampleCfg;    

/**************************************************************
************************* GLOBAL VARIABLES ********************
***************************************************************/

/* Host Descriptor Region - [Size of descriptor * Number of descriptors] 
 *
 * MUST be 16 byte aligned.
 */
#pragma DATA_ALIGN (hostDesc, 16)
UInt8                                   hostDesc[FFTC_SIZE_HOST_DESC * FFTC_NUM_HOST_DESC];

/* FFTC driver handle for this application */
Fftc_DrvHandle                          hFFTC;

/* FFTC instance number that the example uses */
UInt8                                   fftcInstNum;

/* Memory usage stats */
extern UInt32                           fftcMallocCounter;
extern UInt32                           fftcFreeCounter;
extern UInt32                           fftcCppiMallocCounter;
extern UInt32                           fftcCppiFreeCounter;
extern UInt32                           fftcQmssMallocCounter;
extern UInt32                           fftcQmssFreeCounter;

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams          qmssGblCfgParams;

/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams          cppiGblCfgParams;

#define                                 NUM_CORES           4
#define                                 SYS_INIT_CORE       0
#define                                 FFTC_APP_SEM        9

/* Number of packets to run through the example for test */
#define									NUM_PACKETS			1

/* Used to synchronize System Init on all cores */
#pragma DATA_SECTION (bIsSysInitDone, ".fftc");
static UInt32                           bIsSysInitDone 		= 0;

/* Used to synchronize System De-Init on all cores */
#pragma DATA_SECTION (bIsCoreExampleDone, ".fftc");
static UInt32                           bIsCoreExampleDone 	= 0;

/* Used to synchronize driver configuration on all cores */
#pragma DATA_SECTION (bIsCoreInitDone, ".fftc");
static UInt32                           bIsCoreInitDone 	= 0;

/* Global free descriptor queue handle for this application instance */
Qmss_QueueHnd                           hGlblFDQ;

#define FFTC_CONTROL_HDR_SIZE           4
#define FFTC_QUEUE_CONFIG_SIZE          20

/**************************************************************
**************** EXAMPLE APP FUNCTIONS ************************
***************************************************************/

/**
 * ============================================================================
 *  @n@b convert_coreLocalToGlobalAddr
 *
 *  @b  brief
 *  @n  Utility function which converts a local GEM L2 memory address 
 *      to global memory address.
 *
 *  @param[in]  l2addr
 *      Local address to be converted
 *
 *  @return     UInt32
 *      Computed L2 global Address
 * 
 * ============================================================================
 */
static UInt32 convert_coreLocalToGlobalAddr 
(
    UInt32                      addr
)
{
	UInt32 coreNum;

    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM); 

    /* Compute the global address. */
    return ((1 << 28) | (coreNum << 24) | (addr & 0x00ffffff));
}    

/**	============================================================================
 *  @b Description
 *  @n  
 *      Utility function that is required by the IPC module to set the proc Id.
 *      The proc Id is set via this function instead of hard coding it in the 
 * 		.cfg file
 *
 *  @retval
 *      Not Applicable.
 * ============================================================================
 */
Void myStartupFxn (Void)
{
	MultiProc_setLocalId (CSL_chipReadReg (CSL_CHIP_DNUM));
}

#ifndef SIMULATOR_SUPPORT
static Int32 enable_fftc_instance (UInt32 pwrDmnNum,
                                   UInt32 moduleNum)
{
/* FFTC power domain is turned OFF by default. It
     * needs to be turned on before doing any FFTC device
     * register access.
     */
    /* Set FFTC Power domain to ON */        
    CSL_PSC_enablePowerDomain (pwrDmnNum);

    /* Enable the clocks too for FFTC */
    CSL_PSC_setModuleNextState (moduleNum, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (pwrDmnNum);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (pwrDmnNum));

    /* Return FFTC PSC status */
    if ((CSL_PSC_getPowerDomainState(pwrDmnNum) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (moduleNum) == PSC_MODSTATE_ENABLE))
    {
        /* FFTC ON. Ready for use */            
        return 0;
    }
    else
    {
        /* FFTC Power on failed. Return error */            
        return -1;            
    }
}
#endif

/**
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for FFTC. 
 *
 *  @retval
 *      Not Applicable.
 */
static Int32 enable_fftc (void)
{
#ifndef SIMULATOR_SUPPORT
    Int32 result1 = 0;
    Int32 result2 = 0;

    /* FFTC power domain is turned OFF by default. It
     * needs to be turned on before doing any FFTC device
     * register access.
     */
#if defined(DEVICE_K2L)
    result1 = enable_fftc_instance(CSL_PSC_PD_FFTC_0,CSL_PSC_LPSC_FFTC_0);
    result2 = enable_fftc_instance(CSL_PSC_PD_FFTC_1,CSL_PSC_LPSC_FFTC_1);

    return ((result1 + result2));

#else
    result1 = enable_fftc_instance(CSL_PSC_PD_FFTC_01,CSL_PSC_LPSC_FFTC_0);
    result2 = enable_fftc_instance(CSL_PSC_PD_FFTC_01,CSL_PSC_LPSC_FFTC_1);

    return ((result1 + result2));
#endif
#else
    /* No power up needed on Sim */
    return 0;
#endif
}


/**
 * ============================================================================
 *  @n@b system_init
 *
 *  @b  brief
 *  @n  This API is called only once and during system bring up. This API 
 *      initializes the CPPI,QMSS LLDs and FFTC driver.
 *
 *  @param[in]  coreNum
 *      Current DSP core number that is doing the system init
 *
 *  @param[in]  fftcInstNum
 *      FFTC instance number to use
 *
 *  @return     Int32
 *  @li         -1  -   Invalid configuration, system bring up failed
 *  @li         0   -   System Init succeeded
 *
 * ============================================================================
 */
Int32 system_init 
(
    UInt32                      coreNum,
    UInt8                       fftcInstNum
)
{
    Int32                       result, i;
    Qmss_MemRegInfo             memCfg;
    Qmss_InitCfg                qmssInitConfig;
	Fftc_GlobalCfg      		fftcGlobalCfg;
    Fftc_DeviceCfg              fftcDevCfg;
	Fftc_RetVal					retVal;

    /* Initialize QMSS */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up QMSS configuration.
     *  
     * Use internal linking RAM.
     */
    qmssInitConfig.linkingRAM0Base  =   0;   
    qmssInitConfig.linkingRAM0Size  =   0;
    qmssInitConfig.linkingRAM1Base  =   0;
    qmssInitConfig.maxDescNum       =   FFTC_NUM_HOST_DESC;

#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId 	= Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size 	= sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId 	= Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size 	= sizeof (acc48_le);
#endif

    /* Initialize the Queue Manager */
    if ((result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams)) != QMSS_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem, Error code : %d\n", result);
        return -1;
    }

    /* Start Queue manager on this core */
    Qmss_start ();

    /* Initialize CPPI LLD */
    if ((result = Cppi_init (&cppiGblCfgParams)) != CPPI_SOK)
    {
        System_printf ("Error initializing CPPI LLD, Error code : %d\n", result);
        return -1;
    }
    
    /* Setup the descriptor memory regions. 
     *
     * The Descriptor base addresses MUST be global addresses and
     * all memory regions MUST be setup in ascending order of the
     * descriptor base addresses.
     */

    /* Initialize and setup FFTC Host Descriptors required for example */
    memset (hostDesc, 0, FFTC_SIZE_HOST_DESC * FFTC_NUM_HOST_DESC);
    memCfg.descBase         =   (UInt32 *) convert_coreLocalToGlobalAddr ((UInt32) hostDesc);
    memCfg.descSize         =   FFTC_SIZE_HOST_DESC;
    memCfg.descNum          =   FFTC_NUM_HOST_DESC;
    memCfg.manageDescFlag   =   Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memCfg.memRegion        =   (Qmss_MemRegion) QMSS_PARAM_NOT_SPECIFIED;
    memCfg.startIndex       =   0;

    /* Insert Host Descriptor memory region */
    result = Qmss_insertMemoryRegion(&memCfg);
    if (result == QMSS_MEMREGION_ALREADY_INITIALIZED)
    {
        System_printf ("[Core %d]: Memory Region %d already Initialized \n", coreNum, memCfg.memRegion);
    }
    else if (result < QMSS_SOK)
    {
        System_printf ("[Core %d]: Error: Inserting memory region %d, Error code : %d\n", coreNum, memCfg.memRegion, result);
        return -1;
    }    

    /* Initialize FFTC driver */
    /* Set up the FFTC Configuration Register */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++)
    {
        fftcGlobalCfg.queueFlowidOverwrite [i]   =   0;
        fftcGlobalCfg.queuePriority [i]          =   0;
    }
    fftcGlobalCfg.starvationPeriodVal            =   0x0;
    fftcGlobalCfg.bDisableFFT                    =   0;
    
    /* Initialize FFTC driver for instance number specified. */
    fftcDevCfg.cpdmaNum     =   Cppi_CpDma_FFTC_A_CPDMA;
    fftcDevCfg.baseQueueNum =   QMSS_FFTC_A_QUEUE_BASE;            
    fftcDevCfg.cfgRegs      =   (Void *) (CSL_FFTC_0_CFG_REGS);
    retVal = Fftc_init (fftcInstNum, &fftcGlobalCfg, &fftcDevCfg);
    if (retVal == FFTC_RETVAL_SUCCESS)
    {
        System_printf ("[Core %d]: FFTC instance %d successfully initialized \n", coreNum, fftcInstNum);            
    }
    else
    {
        System_printf ("[Core %d]: FFTC init failed on instance %d \n", coreNum, fftcInstNum);
        return -1;
    }

    /* System Init Done. Return success */
    return 0;
}    

/**
 * ============================================================================
 *  @n@b system_deInit
 *
 *  @b  brief
 *  @n  This API is to be called at system shutdown. This API 
 *      de-initializes and closes the CPPI,QMSS LLDs and FFTC driver.
 *
 *  @param[in]  
 *      coreNum     DSP core on which the example is being run
 *
 *  @return     None
 * ============================================================================
 */
Void system_deInit 
(
    UInt32              coreNum
)
{
    /* Teardown the FFTC instance */
	Fftc_deInit (CSL_FFTC_0);
	
    /* Exit CPPI LLD */        
    Cppi_exit ();
}

/**
 * ============================================================================
 *  @n@b fftc_setup
 *
 *  @b  brief
 *  @n  This API sets up the global configuration of FFTC driver.
 *
 *  @param[in]  coreNum
 *      Current DSP core number that is doing the system init
 *
 *  @param[in]  fftcInstNum
 *      FFTC instance number to use
 *
 *  @return     Int32
 *              -1      -   FFTC setup failed.
 *              0       -   FFTC setup successful.
 * ============================================================================
 */
Int32 fftc_setup 
(
    UInt32                  coreNum,
    UInt8                   fftcInstNum
)
{
    Fftc_RetVal             retVal;                
    Fftc_DrvCfg             fftcInitCfg;
    Cppi_DescCfg            cppiDescCfg;
    UInt32                  numAllocated;

    /* Application will manage its descriptors. So, no need to setup
     * driver with descriptor configuration here. 
     */        
    fftcInitCfg.cppiNumFreeDescCfg                  =   0;

    /* Setup the descriptors required by this app and open the driver. */
    hFFTC = Fftc_open (fftcInstNum, &fftcInitCfg, &retVal);
    if (retVal == FFTC_RETVAL_SUCCESS)
    {
        System_printf ("[Core %d]: FFTC successfully opened \n", coreNum);            
    }
    else
    {
        System_printf ("[Core %d]: FFTC open failed \n", coreNum);
        return -1;
    }

    /* Setup a queue with some free descriptors for use by the application on this core. */
    memset (&cppiDescCfg, 0, sizeof (cppiDescCfg));
    cppiDescCfg.memRegion               =   Qmss_MemRegion_MEMORY_REGION0;
    cppiDescCfg.descNum                 =   FFTC_NUM_HOST_DESC/NUM_CORES;
    cppiDescCfg.destQueueNum            =   QMSS_PARAM_NOT_SPECIFIED;     
    cppiDescCfg.queueType               =   Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    cppiDescCfg.initDesc                =   Cppi_InitDesc_INIT_DESCRIPTOR;
    cppiDescCfg.descType                =   Cppi_DescType_HOST;
    cppiDescCfg.returnPushPolicy        =   Qmss_Location_TAIL;    
    cppiDescCfg.cfg.host.returnPolicy   =   Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;    
    cppiDescCfg.cfg.host.psLocation     =   Cppi_PSLoc_PS_IN_DESC;         
    cppiDescCfg.returnQueue.qMgr        =   QMSS_PARAM_NOT_SPECIFIED;    
    cppiDescCfg.returnQueue.qNum        =   QMSS_PARAM_NOT_SPECIFIED; 
    cppiDescCfg.epibPresent             =   Cppi_EPIB_NO_EPIB_PRESENT;

    if ((hGlblFDQ = Cppi_initDescriptor (&cppiDescCfg, &numAllocated)) <= 0)
    {
        System_printf ("Error setting up Global FDQ, Error: %d \n", hGlblFDQ);
        return -1;
    }        
    else
    {
        System_printf ("Global FDQ %d successfully setup with %d descriptors\n", hGlblFDQ, numAllocated);
    }

    /* FFTC driver global setup done. Return success. */
    return 0;
}


/**
 * ============================================================================
 *  @n@b fftc_parse_exampleCfg
 *
 *  @b  brief
 *  @n  This API reads the Input configuration file and retrieves the FFT
 *      configuration parameters for one packet.
 *
 *  @param[in]  
 *      cfgFile         Name of the config file to read the FFT configuration 
 *                      from.
 *
 *  @param[in]
 *      pFFTAppCfg      Configuration structure into which the FFT params need
 *                      to be saved.
 *
 *  @return     Int32
 *              -1      -   FFTC configuration read failed.
 *              0       -   FFTC configuration read successful.
 * ============================================================================
 */
Int32 fftc_parse_exampleCfg (Int16* cfgFile, FFT_ExampleCfg* pFFTAppCfg)
{
    Int32                       tmp, i, j;
    Int16                       *cfgReadPtr = cfgFile;
    Cplx16*                     xread;

    /* Initialize the example configuration */
    memset (pFFTAppCfg, 0, sizeof (FFT_ExampleCfg));
    
    /* Get the number of FFT blocks */
    pFFTAppCfg->numBlocks                                              =   (UInt16) *cfgReadPtr++;

    /* Scan the DFT size in words */
    pFFTAppCfg->fftcQCfg.controlRegConfig.dftSize                      =   (UInt16) *cfgReadPtr++;
    
    /* Supress error info? 
     *
     * By default disable side-band info
     */
    pFFTAppCfg->fftcQCfg.controlRegConfig.bSupressSideInfo	            =   1;

    /* Read DFT size */
    pFFTAppCfg->numInputSamples = pFFTAppCfg->fftcQCfg.controlRegConfig.dftSize;
    cfgReadPtr += 3;

    /* Get the rest of FFT Queue configuration */
    pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.bDynamicScaleEnable  =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingValLast  =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[0]   =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[1]   =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[2]   =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[3]   =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[4]   =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[5]   =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[6]   =   (UInt16) *cfgReadPtr++;
    cfgReadPtr += 3;
	tmp = (UInt16) *cfgReadPtr++;
	if (tmp == 0)
		pFFTAppCfg->fftcQCfg.controlRegConfig.dftMode                 	=   Fftc_DFTMode_DFT;
	else
		pFFTAppCfg->fftcQCfg.controlRegConfig.dftMode                 	=   Fftc_DFTMode_IDFT;
	pFFTAppCfg->fftcQCfg.controlRegConfig.bEmulateDSP16x16             	=   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.outputScaleVal       	=   (UInt16) *cfgReadPtr++;
	
    pFFTAppCfg->fftcQCfg.destQRegConfig.bInputFFTShift                  =   (UInt16) *cfgReadPtr++;
    pFFTAppCfg->fftcQCfg.destQRegConfig.bOutputFFTShift                 =   (UInt16) *cfgReadPtr++;
	
    pFFTAppCfg->fftcQCfg.controlRegConfig.bZeroPadEnable               =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.controlRegConfig.zeroPadMode                  =   (Fftc_ZeroPadMode) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.controlRegConfig.zeroPadFactor                =   (UInt16) *cfgReadPtr++;

    pFFTAppCfg->fftcQCfg.freqShiftRegConfig.bFreqShiftEnable           =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.scalingShiftingRegConfig.freqShiftScaleVal    =   (UInt16) *cfgReadPtr++;
    tmp = (UInt16) *cfgReadPtr++;
    if (tmp == 8192)
    {
	    pFFTAppCfg->fftcQCfg.freqShiftRegConfig.freqShiftIndex         =   Fftc_FreqShiftIndex_16384;
    }
    else
    {
	    pFFTAppCfg->fftcQCfg.freqShiftRegConfig.freqShiftIndex         =   Fftc_FreqShiftIndex_12288;
    }
	pFFTAppCfg->fftcQCfg.freqShiftRegConfig.freqShiftMultFactor        =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.freqShiftRegConfig.freqShiftInitPhase         =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.freqShiftRegConfig.freqShiftDirection         =   (Fftc_FreqShiftDir) *cfgReadPtr++;
	
    pFFTAppCfg->fftcQCfg.cyclicPrefixRegConfig.bCyclicPrefixAddEnable  =   (UInt16) *cfgReadPtr++;
	pFFTAppCfg->fftcQCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum      =   (UInt16) *cfgReadPtr++;

    /* Always set Dest Queue Number in the queue configuration to 0x3FFF, 
     * since the destination queue number is setup during flow setup.
     */
    pFFTAppCfg->fftcQCfg.destQRegConfig.cppiDestQNum                   = 0x3fff;    

    /* Get the input and output data info for each of the blocks now */
    for (i = 0; i < pFFTAppCfg->numBlocks; i ++)
    {
        /* Skip over the block number */
        cfgReadPtr += 1;

        /* Allocate memory for the input data for this block */
        if (!(pFFTAppCfg->pFftInputData [i] = (Cplx16 *) Osal_fftcMalloc (sizeof (Cplx16) *  pFFTAppCfg->numInputSamples, FALSE)))
        {
            System_printf ("Error allocating memory for input data block: %d \n", i);

            /* Free up all the input data blocks allocated so far */
            for (j = 0; j < i-1; j ++)
            {
                Osal_fftcFree (pFFTAppCfg->pFftInputData [j], sizeof (Cplx16) *  pFFTAppCfg->numInputSamples, FALSE);                  
            }

            /* Return error */
            return -1;
        }
   
        /* Read input data for this block. */
        xread = pFFTAppCfg->pFftInputData [i];
	    for (j = 0; j < pFFTAppCfg->numInputSamples; j++)  
	    {
		    xread[j].real               =   *cfgReadPtr++;
	    }
	
	    for (j = 0; j < pFFTAppCfg->numInputSamples; j++)
	    {
		    xread[j].imag               =   *cfgReadPtr++;
	    }

        pFFTAppCfg->blockExpVal [i]        = (UInt16) *cfgReadPtr++;
	    pFFTAppCfg->bClippingDetected [i]  = (UInt16) *cfgReadPtr++;
	    pFFTAppCfg->numClockCycles [i]     = (UInt16) *cfgReadPtr++;

        pFFTAppCfg->numOutputSamples   = pFFTAppCfg->numInputSamples;
        if (pFFTAppCfg->fftcQCfg.cyclicPrefixRegConfig.bCyclicPrefixAddEnable)
            pFFTAppCfg->numOutputSamples += pFFTAppCfg->fftcQCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum;

        /* Allocate memory for the output data for this block */
        if (!(pFFTAppCfg->pFftOutputData [i] = (Cplx16 *) Osal_fftcMalloc (sizeof (Cplx16) *  pFFTAppCfg->numOutputSamples, FALSE)))
        {
            System_printf ("Error allocating memory for output data block: %d \n", i);

            /* Free up all the input data blocks allocated so far */
            for (j = 0; j < i; j ++)
            {
                Osal_fftcFree (pFFTAppCfg->pFftInputData [j], sizeof (Cplx16) *  pFFTAppCfg->numInputSamples, FALSE);                  
            }

            /* Return error */
            return -1;
        }
    

        /* Read output data */
        xread = pFFTAppCfg->pFftOutputData [i];
	    for (j = 0; j < pFFTAppCfg->numOutputSamples; j++)
	    {
		    xread[j].real               =   *cfgReadPtr++;
	    }

	    for (j = 0; j < pFFTAppCfg->numOutputSamples; j++)
	    {
		    xread[j].imag               =   *cfgReadPtr++;
	    }    
    }

    /* Return Success */
    return 0;    
}

/** ============================================================================
 *   @n@b deallocate_fdq
 *
 *   @b Description
 *   @n This API cleans up the Tx/Rx FDQ passed. It restores the free descriptors 
 *      from Tx/Rx FDQ to the application's global FDQ and closes the FDQ. 
 *
 *   @param[in]  
 *   @n hFDQ            Tx/Rx FDQ handle obtained earlier from @a allocate_fdq () 
 *                      API.
 *
 *   @param[in]  
 *   @n hGlblFDQ        Global FDQ to which to restore the descriptors from the
 *                      Tx/Rx FDQ being de-allocated.
 *
 *   @param[in]  
 *   @n numDesc         Number of descriptors setup on this Rx/Tx FDQ.
 * 
 *   @param[in]  
 *   @n buffSize        Size of pre-allocated buffers linked to the descriptor.
 * 
 *   @return        
 *   @n None.
 * =============================================================================
 */
Void deallocate_fdq 
(
    Qmss_QueueHnd       hFDQ, 
    Qmss_QueueHnd       hGlblFDQ, 
    UInt32              numDesc, 
    UInt32              buffSize
)
{
    UInt8               i;
    Cppi_Desc*          pCppiDesc;
    UInt8*              pDataBuffer;
    UInt32              dataBufferLen;

    for (i = 0; i < numDesc && hFDQ; i ++)
    {
        if ((pCppiDesc = Qmss_queuePop (hFDQ)) == NULL)
        {
            break;                
        }

        /* The descriptor address returned from the hardware has the 
         * descriptor size appended to the address in the last 4 bits.
         *
         * To get the true descriptor size, always mask off the last 
         * 4 bits of the address.
         */
	    pCppiDesc = (Void*) (QMSS_DESC_PTR (pCppiDesc));    
        Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pDataBuffer, &dataBufferLen);
        Cppi_setDataLen (Cppi_DescType_HOST, pCppiDesc, buffSize);
        Fftc_osalFree (pDataBuffer, buffSize, TRUE);

        //Cppi_linkNextBD (Cppi_DescType_HOST, pCppiDesc, NULL);

        /* Push descriptor back to free queue */
        Qmss_queuePushDescSize (hGlblFDQ, pCppiDesc, 256);           
    }
	Qmss_queueClose (hFDQ);

    return;        
}

/** ============================================================================
 *   @n@b allocate_fdq
 *
 *   @b Description
 *   @n This API opens a Tx/Rx Free Descriptor queue (FDQ) and sets it up with
 *      'numDesc' number of free descriptors populated with pre-allocated buffers.
 *
 *   @param[in]  
 *   @n hGlblFDQ        Global FDQ set up for the test application. This API
 *                      uses the required number of FDs from this queue.
 *
 *   @param[in]  
 *   @n numDesc         Number of descriptors to setup for this Rx/Tx FDQ.
 * 
 *   @param[in]  
 *   @n buffSize        Size of data buffer to allocate. The data buffer is 
 *                      linked to descriptor at FDQ allocation time.
 * 
 *   @return        Qmss_QueueHnd
 *   @n NULL    -       Error setting up Tx/Rx FDQ with descriptors.
 *   @n >0      -       Succesfully set up a Tx/Rx FDQ with 'numDesc' descriptors.
 *                      All descriptors are linked with pre-allocated buffers of
 *                      size 'buffSize'. FDQ handle returned.
 * =============================================================================
 */
Qmss_QueueHnd allocate_fdq
(
    Qmss_QueueHnd       hGlblFDQ, 
    UInt32              numDesc, 
    UInt32              buffSize
)
{
    Qmss_QueueHnd       hFDQ;
    UInt8               isAllocated, i;
    Cppi_Desc*          pCppiDesc;
    UInt8*              pDataBuffer;
    Qmss_Queue          txComplnQInfo;

    if ((hFDQ = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
	{
        System_printf ("Error opening FDQ \n");
		return NULL;
	}    

    /* Use Tx FDQ as the Tx completion queue */            
    txComplnQInfo = Qmss_getQueueNumber (hFDQ);

    for (i = 0; i < numDesc; i ++)
    {
        if ((pCppiDesc = Qmss_queuePop (hGlblFDQ)) == NULL)
        {
            break;                
        }

        /* The descriptor address returned from the hardware has the 
         * descriptor size appended to the address in the last 4 bits.
         *
         * To get the true descriptor size, always mask off the last 
         * 4 bits of the address.
         */
	    pCppiDesc = (Void*) (QMSS_DESC_PTR (pCppiDesc));    
        if ((pDataBuffer =   (UInt8 *) Fftc_osalMalloc (buffSize, TRUE)) == NULL)
        {
        	System_printf ("OOM error while allocating buffers\n");
            break;                
        }

        Cppi_setData (Cppi_DescType_HOST, pCppiDesc, pDataBuffer, buffSize);

        /* Save original buffer information */
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, pCppiDesc, pDataBuffer, buffSize);

        /* Setup the completion queue for the descriptor */
        Cppi_setReturnQueue (Cppi_DescType_HOST, pCppiDesc, txComplnQInfo);

        /* Set packet length */
        Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, buffSize);

        Cppi_linkNextBD (Cppi_DescType_HOST, pCppiDesc, NULL);

        /* Push descriptor back to free queue */
        Qmss_queuePushDescSize (hFDQ, pCppiDesc, 256);           
    }
    if (i !=  numDesc)
	{
        /* Error populating FDQ. Cleanup and return */            
        deallocate_fdq (hFDQ, hGlblFDQ, i, buffSize);
        return NULL;
	}            
    else
    {
        /* Success. Return FDQ handle. */            
        return hFDQ;            
    }
}

/**
 * ============================================================================
 *  @n@b add_fftc_config
 *
 *  @b  brief
 *  @n  Given the FFT configuration parameters, a packet descriptor and data buffer,
 *      this API builds the FFTC packet by adding a relevant FFTC control header and 
 *      appending any other configuration needed such as the queue configuration, 
 *      block size information, tag information etc. On success, the API returns
 *      number of bytes of control information added to packet by this API. 
 *
 *  @param[in]  pDFTBlockSizeInfo
 *      Block size info. Indicates if the data consists of mixed size DFT blocks 
 *      or not and the number of code blocks.
 *
 *  @param[in]  pFFTCQConfig
 *      FFT/IFFT params for the packet
 *
 *  @param[in]  psInfoLen
 *      Length of Protocol Specific (PS) information that application wishes to
 *      forward to receiver.
 *
 *  @param[in]  destnFlowId
 *      Rx flow that should be used for the FFT/IFFT result corresponding to
 *      this packet.
 *
 *  @param[in]  destnTagInfo
 *      A 16-bit unique identifier (or any application specific info) to
 *      be passed to receiver.
 *
 *  @param[in]  pCppiDesc
 *      CPPI descriptor corresponding to the packet
 *
 *  @param[in]  pDataBuffer
 *      Data buffer to which the header and the other control info
 *      must be added.
 *
 *  @return     Int32
 *  @li         -1  -   Invalid inputs passed
 *  @li         >0  -   Number of bytes of control info added to the data buffer
 *                      by this API.
 * ============================================================================
 */
static Int32 add_fftc_config
(
    Fftc_BlockInfo*         pDFTBlockSizeInfo,
    Fftc_QLocalCfg*         pFFTCQConfig, 
    UInt32                  psInfoLen,
    UInt8                   destnFlowId,
    UInt16                  destnTagInfo,
    Cppi_Desc*              pCppiDesc,
    UInt8*                  pDataBuffer
)
{
    Fftc_ControlHdr         fftcCtrlHdrObj;
    UInt32                  dataBuffLenUsed = 0, tmpLen;
    Cppi_DescTag            cppiTagCfg;

    /* Add the FFTC control header if either:
     *
     * (1) FFT/IFFT configuration parameters need to be sent on a per
     *     packet basis
     * (2) If code blocks of payload are of mixed/varying sizes and hence
     *     a list of their block sizes must be sent as configuration 
     *     info to the engine
     * (3) If application specifi PS info is to be passed from Tx to Rx.
     */
    if (pFFTCQConfig || !pDFTBlockSizeInfo->bIsEqualSize || psInfoLen)
    {
        /* Enable Bit 0 of the PS flags to indicate presence of control header. */
        Cppi_setPSFlags (Cppi_DescType_HOST, pCppiDesc, (1 << 0));  

        /* Build the control header as per the configuration specified. */

        /* Convert the PS data length from bytes to number of words */
        fftcCtrlHdrObj.psFieldLen               =   (psInfoLen / 4);          
        if (psInfoLen)        
        {
            /* Enable PS Pass through data present bit */                           
            fftcCtrlHdrObj.bPSPassThruPresent   =   1;  
        }
        else
            fftcCtrlHdrObj.bPSPassThruPresent   =   0;  

        if (pDFTBlockSizeInfo->numBlocks && !pDFTBlockSizeInfo->bIsEqualSize)
        {
            /* Enable DFT size list if a valid DFT size list length passed and if the DFT list is a mixed block size */                
            fftcCtrlHdrObj.bDFTSizeListPresent  =   1;  

            /* Convert the DFT size list length from bytes to number of words */
            if (pDFTBlockSizeInfo->numBlocks % 5)
                fftcCtrlHdrObj.dftSizeListLen   =   ((pDFTBlockSizeInfo->numBlocks / 5) + 1);             
            else
                fftcCtrlHdrObj.dftSizeListLen   =   (pDFTBlockSizeInfo->numBlocks / 5);             
        }
        else
        {
            fftcCtrlHdrObj.bDFTSizeListPresent  =   0;
            
            /* Convert the DFT size list length from bytes to number of words */
            fftcCtrlHdrObj.dftSizeListLen       =   0;
        }             
              
        if (pFFTCQConfig)
        {
            /* Enable the FFT queue configuration present bit */                
            fftcCtrlHdrObj.bLocalConfigPresent  =   1; 
        }
        else
            fftcCtrlHdrObj.bLocalConfigPresent  =   0; 

        /* Add the FFTC control header to the buffer */
        tmpLen = 0;
        if (Fftc_createControlHeader (&fftcCtrlHdrObj, pDataBuffer, &tmpLen) != 0)
        {
            System_printf("FFTC Control Header creation failed \n");
            return -1;
        }
        /* Increment data buffer pointer to move over the control header and also increment 
         * our data buffer length counter 
         */
        pDataBuffer     += tmpLen;  
        dataBuffLenUsed +=  tmpLen;

        /* Add in the FFT Queue configuration parameters */
        tmpLen = 0;
        if (pFFTCQConfig)
        {
            /* If a DFT size list is being specified, overwrite the DFT Size field
             * of the Control register to '0x3F' (63) to indicate to the hardware
             * to use DFT size list specified in the packet.
             */
            if (pDFTBlockSizeInfo->numBlocks && !pDFTBlockSizeInfo->bIsEqualSize)
            {
                /* Use DFT size list to process blocks. */                    
                pFFTCQConfig->controlRegConfig.dftSize      =   FFTC_DEF_DFT_SIZE;            
            }
                
            if (Fftc_compileQueueLocalConfigParams (pFFTCQConfig, (UInt8 *) pDataBuffer, &tmpLen) != 0)
            {
                System_printf ("FFTC Queue local configuration creation failed \n");
                return -1;
            }

            /* Increment pointer to move over the FFTC configuration  */
            pDataBuffer     += tmpLen;      
            dataBuffLenUsed +=  tmpLen;
        }

        /* Add in the DFT size list configuration */
        tmpLen = 0;
        if (pDFTBlockSizeInfo->numBlocks && !pDFTBlockSizeInfo->bIsEqualSize)
        {
            if ((Fftc_createDftSizeList (pDFTBlockSizeInfo->blockSizes, pDFTBlockSizeInfo->numBlocks, 
                                        pDataBuffer, &tmpLen)) != 0)                
            {
                System_printf ("FFTC DFT Size list creation failed \n");
                return -1;
            }

            /* Increment pointer to move over the DFT Size List  */            
            pDataBuffer     += tmpLen;      
            dataBuffLenUsed +=  tmpLen;
        }

        /* Configure PS location in BD to indicate that PS is in the data itself. */
        Cppi_setPSLocation (Cppi_DescType_HOST, pCppiDesc, Cppi_PSLoc_PS_IN_SOP);

    	/* Set PS words length in the descriptor */
    	Cppi_setPSLen (Cppi_DescType_HOST, pCppiDesc, psInfoLen + dataBuffLenUsed);
    }

    /* Populate the tag info as follows:
     *
     * Source Tag (15:8)    -   Not used by CDMA. 
     *
     * Source Tag (7:0)     -   Set to Rx flow Id that should receive the result corresponding
     *                          to this request.
     *
     * Destn Tag (15:8)     -   Set this to the request Id higher order 8 bits passed by the
     *                          application.
     *
     * Destn Tag (7:0)      -   Set this to the request Id lower order 8 bits passed by the
     *                          application.
     */
    cppiTagCfg.srcTagHi     =   0;
    cppiTagCfg.srcTagLo     =   (destnFlowId & 0x00FF);
    cppiTagCfg.destTagHi    =   ((destnTagInfo & 0xFF00) >> 8) ;
    cppiTagCfg.destTagLo    =   (destnTagInfo & 0x00FF);
    Cppi_setTag (Cppi_DescType_HOST, pCppiDesc, &cppiTagCfg);       

    return dataBuffLenUsed;
}

/**
 * ============================================================================
 *  @n@b fftc_run_app_managed_example
 *
 *  @b  brief
 *  @n  This API demonstrates the use of driver's application managed mode to
 *      send/receive packets to FFTC engine.
 *
 *  @param[in]  
 *      coreNum         DSP core number on which the example is being run.
 *
 *  @param[in]      
 *      txQNum          FFTC Transmit queue using which the FFT processing
 *                      must be done.
 *
 *  @return     Int32
 *              -1      -   FFTC example failed.
 *              0       -   FFTC example successful.
 * ============================================================================
 */
Int32 fftc_run_app_managed_example (UInt32 coreNum, UInt32 txQNum)
{
    Fftc_ResultHandle           hResultInfo;
    Fftc_TxHandle               hTxObj 	= 	NULL;
    Fftc_RxHandle               hRxObj	=	NULL;
    Fftc_TxCfg                  txCfg;    
    Fftc_RxCfg                  rxCfg;    
    UInt32                      buffLen, blockExpVal, clipping, bIsErrorFound = 0, dataOffset = 0;
    UInt32                      requestDataLen, resultLen, rxPSInfoLen, txPSInfoLen = 0, numRxPackets;
    UInt32                      dataBufferLen, txNumDesc, rxNumDesc, txBuffSize, rxBuffSize;
    UInt16                      myQNum, destQNum, rxDestnTagInfo;
    Fftc_BlockInfo              blockInfo;
    UInt8                       *pReqBuffer, *pResultBuffer, *pResultPSInfo, rxFlowId, rxSrcId;
    Fftc_Result*                pFFTResult = NULL;
    Cplx16                      *xout, *xoutread;
    UInt16                      i, j, pktCounter = 0;
    Int32                       retVal;
    FFT_ExampleCfg*             pFFTAppCfg = NULL;
    Qmss_QueueHnd               hTxFDQ = NULL, hRxFDQ = NULL;
    Cppi_Desc*                  pCppiDesc;
    Int16*                      pCfgFile = inputConfig_16;

    /* Initialize the block info */
    memset (&blockInfo, 0, sizeof(blockInfo));

    /* Allocate memory for the example configuration */
    if (!(pFFTAppCfg = (FFT_ExampleCfg *) Osal_fftcMalloc (sizeof (FFT_ExampleCfg), FALSE)))
    {
        System_printf ("[Core %d]: Error allocating memory for example configuration \n", coreNum);
        return -1;
    }

    /* Read the FFT input vector and configuration for this example 
     * from the corresponding input configuration file.
     */
    if (fftc_parse_exampleCfg (pCfgFile, pFFTAppCfg) < 0)
    {
        return -1;         
    }

    /* Populate the FFT block size info */
    blockInfo.numBlocks         =   pFFTAppCfg->numBlocks;
    blockInfo.bIsEqualSize      =   1;
    blockInfo.blockSizes        =   (UInt16 *) Osal_fftcMalloc (sizeof(UInt16) * 1, FALSE);
    blockInfo.blockSizes[0]     =   pFFTAppCfg->numInputSamples;     

    /* Pick a destination flow and a Rx queue to receive the FFT result. */
    switch (coreNum)
    {
        case 0: 
        {
            myQNum      =   708;
            destQNum    =   709;
            break;
        }
        case 1: 
        {
            myQNum      =   709;                
            destQNum    =   710;
            break;
        }
        case 2: 
        {
            myQNum      =   710;                
            destQNum    =   711;
            break;
        }
        case 3: 
        {
            myQNum      =   711;                
            destQNum    =   708;
            break;
        }
        default:
        {
            goto error;
        }
    }

    System_printf ("--------------------------------------------------\n");        
    System_printf ("FFTC Application Managed Example START on Core %d \n", coreNum);
    System_printf ("Sample Size:             %d \n", pFFTAppCfg->numInputSamples);
    System_printf ("Number of Blocks:        %d \n", pFFTAppCfg->numBlocks);
    System_printf ("Tx Queue:                %d \n", txQNum);
    System_printf ("Descriptor Type:         Host \n");
    System_printf ("--------------------------------------------------\n");         

    /* Setup Rx side:
     *  -   Setup a Rx FDQ, Rx flow to receive data
     *  -   Open an Rx queue on which FFTC results are to be received
     *
     *  Driver Rx objects can be used to setup the above using application
     *  specified configuration.
     */
    rxNumDesc           =   2;
    rxBuffSize          =   (pFFTAppCfg->numOutputSamples * FFTC_SAMPLE_SIZE + 4) * pFFTAppCfg->numBlocks + 4;
    if ((hRxFDQ = allocate_fdq (hGlblFDQ, rxNumDesc, rxBuffSize)) == NULL)
    {
        System_printf ("[Core %d]: Error opening Rx FDQ \n", coreNum);            
        goto error;
    }
    else
    {
        System_printf ("[Core %d]: Rx FDQ %d successfully setup with %d descriptors\n", coreNum, hRxFDQ, rxNumDesc);
    }
    
    /* Setup Rx flow configuration. This flow will be used to retrieve FFT results
     * from a destination queue.
     *
     * Rx flow Configuration Params:
     * -----------------------------
     *      -   Use Host descriptors
     *      -   No PS Info 
     *      -   Copy the Tx packet's destination tag to Rx packet's destination tag.
     *      -   Copy the Flow Id to Rx packet's source tag lower order 8 bits.
     *      -   Copy the Tx packet's source tag higher order bits to Rx packet's source tag higher order 8 bits.
     *      -   Use the Rx FDQ just setup for Rx free descriptors.
     */
    rxCfg.cppiRxQNum                            =   myQNum;
    rxCfg.useFlowId                             =   -1;
    rxCfg.bManageRxFlowCfg                      =   0;  /* Application will manage the Rx FDQ/flow configuration */

    memset (&rxCfg.rxFlowCfg.fullCfg, 0, sizeof(Cppi_RxFlowCfg));
    rxCfg.rxFlowCfg.fullCfg.flowIdNum           =   -1; /* Let driver pick next available flow id */
    rxCfg.rxFlowCfg.fullCfg.rx_dest_qmgr        =   0;    
    rxCfg.rxFlowCfg.fullCfg.rx_dest_qnum        =   myQNum;  
    rxCfg.rxFlowCfg.fullCfg.rx_desc_type        =   Cppi_DescType_HOST; 
    rxCfg.rxFlowCfg.fullCfg.rx_ps_location      =   Cppi_PSLoc_PS_IN_DESC;  
    rxCfg.rxFlowCfg.fullCfg.rx_psinfo_present   =   0;           
    rxCfg.rxFlowCfg.fullCfg.rx_error_handling   =   0;              //  Drop the packet, do not retry on starvation
    rxCfg.rxFlowCfg.fullCfg.rx_einfo_present    =   0;              //  By default no EPIB info
    
    rxCfg.rxFlowCfg.fullCfg.rx_dest_tag_lo_sel  =   4;              //  Pick dest tag 7:0 bits from the PD dest_tag        
    rxCfg.rxFlowCfg.fullCfg.rx_dest_tag_hi_sel  =   5;              //  Pick the dest tag 15:8 bits from the PD dest_tag
    rxCfg.rxFlowCfg.fullCfg.rx_src_tag_lo_sel   =   2;              //  Pick the src tag 7:0 bits from the PD flow_id 7:0
    rxCfg.rxFlowCfg.fullCfg.rx_src_tag_hi_sel   =   4;              //  Pick the src tag 15:8 bits from the PD src_tag 7:0 

    rxCfg.rxFlowCfg.fullCfg.rx_size_thresh0_en  =   0;    
    rxCfg.rxFlowCfg.fullCfg.rx_size_thresh1_en  =   0;    
    rxCfg.rxFlowCfg.fullCfg.rx_size_thresh2_en  =   0;    

    rxCfg.rxFlowCfg.fullCfg.rx_size_thresh0     =   0x0;
    rxCfg.rxFlowCfg.fullCfg.rx_size_thresh1     =   0x0;
    rxCfg.rxFlowCfg.fullCfg.rx_size_thresh2     =   0x0;

    rxCfg.rxFlowCfg.fullCfg.rx_fdq0_sz0_qmgr    =   0; 
    rxCfg.rxFlowCfg.fullCfg.rx_fdq0_sz0_qnum    =   hRxFDQ;    
    rxCfg.rxFlowCfg.fullCfg.rx_fdq0_sz1_qnum    =   0x0; 
    rxCfg.rxFlowCfg.fullCfg.rx_fdq0_sz1_qmgr    =   0x0;
    rxCfg.rxFlowCfg.fullCfg.rx_fdq0_sz2_qnum    =   0x0;
    rxCfg.rxFlowCfg.fullCfg.rx_fdq0_sz2_qmgr    =   0x0;
    rxCfg.rxFlowCfg.fullCfg.rx_fdq0_sz3_qnum    =   0x0;
    rxCfg.rxFlowCfg.fullCfg.rx_fdq0_sz3_qmgr    =   0x0;

    rxCfg.rxFlowCfg.fullCfg.rx_fdq1_qnum        =   hRxFDQ;  
    rxCfg.rxFlowCfg.fullCfg.rx_fdq1_qmgr        =   0;
    rxCfg.rxFlowCfg.fullCfg.rx_fdq2_qnum        =   hRxFDQ;  
    rxCfg.rxFlowCfg.fullCfg.rx_fdq2_qmgr        =   0;
    rxCfg.rxFlowCfg.fullCfg.rx_fdq3_qnum        =   hRxFDQ;  
    rxCfg.rxFlowCfg.fullCfg.rx_fdq3_qmgr        =   0;    

    /* Setup interrupt configuration */
    rxCfg.bUseInterrupts                        =   0;  // Disable interrupts. Use polling to retrieve results

    /* Open the Rx Object with configuration specified */
    if ((hRxObj = Fftc_rxOpen (hFFTC, &rxCfg)) == NULL)
    {
        System_printf ("[Core %d]: Rx object open failed \n", coreNum);
        goto error;
    }
    else
    {
        System_printf ("[Core %d]: Rx flow %d opened successfully using Rx queue %d \n", coreNum,  
                        Fftc_rxGetFlowId (hRxObj), Fftc_rxGetRxQueueNumber (hRxObj));
    }  

    /* Setup Tx side:
     *  -   Open FFTC Tx queue using which data would be sent 
     *  -   Setup a Tx FDQ and initialize it with some Tx FDs
     *
     * Setup a Tx Object. This object will be used to submit FFT requests.
     *
     * Tx object Configuration Params:
     * -----------------------------
     *  -   Driver does not managed Tx FDs (bManageReqBuffers = 0). They are managed by application.
     *  -   Open queue in shared mode (bSharedMode = 1) to enable re-programming 
     *      queue using CPPI packets
     *  
     */
    memset (&txCfg, 0, sizeof (txCfg));
    txCfg.txQNum                =   (Fftc_QueueId) txQNum; 
    txCfg.bSharedMode           =   1;      
    txCfg.fftQCfg               =   pFFTAppCfg->fftcQCfg;
    txCfg.bManageReqBuffers     =   0;      
    if ((hTxObj = Fftc_txOpen (hFFTC, &txCfg)) == NULL)
    {
        System_printf ("[Core %d]: FFTC Tx Open failed \n", coreNum);
        goto error;
    }

    txNumDesc           =   2;

    /* FFTC configuration can be sent in 2 possible ways using CPPI host descriptors:
     *
     * (1): FFTC control header + FFT/IFFT configuration + Payload all in one contiguous buffer
     *      of a single host descriptor.
     *
     * (2): FFTC control header + FFT/IFFT configuration in one host descriptor and just the payload
     *      in another descriptor. These 2 descriptors linked to each other before transmit.
     *
     * For simplicity sake, we implement here option (1), i.e., FFTC configuration and payload all in 
     * one packet. Allocate Tx buffers big enough to hold configuration + payload.
     */
    txBuffSize          =   (pFFTAppCfg->numInputSamples * FFTC_SAMPLE_SIZE * pFFTAppCfg->numBlocks);

    /* Allocate space for control header + FFT configuration + maximum size user PS Info */            
    txBuffSize          +=  FFTC_CONTROL_HDR_SIZE + FFTC_QUEUE_CONFIG_SIZE + (FFTC_MAX_NUM_PS_WORDS * sizeof(UInt32));            

    /* Allocate space for DFT block size list only if we have a mixed size DFT block data to be sent. */
    if (! blockInfo.bIsEqualSize)
    {
        if (blockInfo.numBlocks % 5) 
        {
            txBuffSize  +=  ((blockInfo.numBlocks / 5 + 1) * 4);                  
        }
        else
        {
            txBuffSize  +=  ((blockInfo.numBlocks / 5) * 4);                  
        }
    }

    if ((hTxFDQ = allocate_fdq (hGlblFDQ, txNumDesc, txBuffSize)) == NULL)
    {
        System_printf ("[Core %d]: Error opening Tx FDQ \n", coreNum);            
        goto error;
    }
    else
    {
        System_printf ("[Core %d]: Tx FDQ %d successfully setup with %d descriptors\n", coreNum, hTxFDQ, txNumDesc);
    }

	/* Signal that we are done with Tx/Rx configuration. 
	 * Ready to send/receive.
	 */
    while ((CSL_semAcquireDirect (FFTC_APP_SEM)) == 0);
    Fftc_osalBeginMemAccess ((void *)&bIsCoreInitDone, sizeof(UInt32));
    bIsCoreInitDone ++;
    Fftc_osalEndMemAccess ((void *)&bIsCoreInitDone, sizeof(UInt32));
    CSL_semReleaseSemaphore (FFTC_APP_SEM);    
    
   /* Used to synchronize the Rx flow creation on all cores.
    * This is done to ensure that the Rx flow to which this
    * core (sender) is sending data is actually up.
    */    
    do 
    {
        Fftc_osalBeginMemAccess ((void *)&bIsCoreInitDone, sizeof(UInt32));
        Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
    } while (bIsCoreInitDone != bIsSysInitDone);

    /* -------------------------------------
     * Submit FFT request 
     * -------------------------------------
     */
    /* Run the example in a loop */
	while (pktCounter != NUM_PACKETS)
	{
        txPSInfoLen 	=   0;
        dataOffset	    =	0;

        /* Build and send a packet with FFT/IFFT parameters for FFTC engine processing */
        if ((pCppiDesc = (Cppi_Desc*) Qmss_queuePop (hTxFDQ)) == NULL)
        {
            System_printf ("[Core %d]: Out of Tx FDs! \n", coreNum);
		    goto error;
        }
        pCppiDesc = (Cppi_Desc *) (QMSS_DESC_PTR (pCppiDesc));    
        Cppi_getData (Cppi_DescType_HOST, pCppiDesc, &pReqBuffer, &dataBufferLen);
       
        /* Initialize the buffer */
	    memset (pReqBuffer, 0, dataBufferLen); 

        /* Add FFTC configuration to the packet */
        if ((dataOffset = add_fftc_config (&blockInfo, 
                                           &pFFTAppCfg->fftcQCfg, 
                                           txPSInfoLen,  
                                           Fftc_findFlowIdByQueueNumber (hFFTC, destQNum),                            
                                           pktCounter,                                        
                                           pCppiDesc,
                                           pReqBuffer)) <= 0)
        {
            System_printf ("[Core %d]: Error populating Tx packet \n", coreNum);

            Qmss_queuePushDescSize (hTxFDQ, pCppiDesc, 256);
            goto error;
        }

        /* Add the payload/request data to the packet
         *
         * The request buffer MUST be populated in the following order:
         *
         * <PS Info to be passed to receiver> then followed by <FFT request data>
         *
         * We dont have any PS info to add for now. So, just add request data.
         */
        requestDataLen    =   0;   
        for (i = 0; i < pFFTAppCfg->numBlocks; i ++)
        {
            /* Copy the FFT request block 
             * Each block size = number of samples * size of each sample
             *                 = pFFTAppCfg->numInputSamples * FFTC_SAMPLE_SIZE
             */                
            memcpy ((Ptr) (pReqBuffer + dataOffset), 
                    (Ptr) pFFTAppCfg->pFftInputData [i], 
                    pFFTAppCfg->numInputSamples * FFTC_SAMPLE_SIZE);
        
            /* Increment the data offset so that the next block data can 
             * be copied to the correct location in request buffer.
             *
             * Also increment our request (payload data) length counter.
             */
            dataOffset      +=  (pFFTAppCfg->numInputSamples * FFTC_SAMPLE_SIZE);
            requestDataLen  +=  (pFFTAppCfg->numInputSamples * FFTC_SAMPLE_SIZE);
        }
        Cppi_setPacketLen (Cppi_DescType_HOST, pCppiDesc, requestDataLen);
        
        System_printf ("\n[Core %d]: Submitting FFT Request ... \n", coreNum);

        /* Submit the FFT request for processing */
        if (Fftc_txSubmitRequest (hTxObj, 
                                  pCppiDesc, 
                                  FFTC_SIZE_HOST_DESC) < 0)
        {
            System_printf ("[Core %d]: Unable to submit request \n", coreNum);

            Qmss_queuePushDescSize (hTxFDQ, pCppiDesc, 256);
            goto error;
        }
        else
        {
            System_printf ("[Core %d]: Submitted request %d \n", coreNum, pktCounter);       
        }

        /* -------------------------------------
         * Wait on FFT result and verify Result
         * -------------------------------------
         */
        System_printf ("\n[Core %d]: Waiting for Result from FFTC ... \n", coreNum);
        for (i = 0; i < 100; i ++);
        while ((numRxPackets = Fftc_rxGetNumPendingResults (hRxObj)) == 0);

        for (i = 0; i < numRxPackets; i ++)
        {
            /* Get the raw result from the engine. */   
            if ((retVal = Fftc_rxGetResult (hRxObj, 
                                            &hResultInfo,
                                            &pResultBuffer, 
                                            &resultLen, 
                                            &pResultPSInfo, 
                                            &rxPSInfoLen,
                                            &rxFlowId,
                                            &rxSrcId,
                                            &rxDestnTagInfo
                                            )) != FFTC_RETVAL_SUCCESS)
            {
                System_printf ("[Core %d]: Invalid FFT result : %d \n", coreNum, retVal);
                goto error;
            }

            /* Allocate memory to hold the formatted result */
            pFFTResult = (Fftc_Result *) Osal_fftcMalloc (sizeof (Fftc_Result), FALSE);
    
            /* Get the formatted result. */
            if ((retVal = Fftc_rxParseResult (hRxObj, 
                                          hResultInfo, 
                                          pResultBuffer, 
                                          resultLen, 
                                          &blockInfo,
                                          pFFTAppCfg->fftcQCfg.controlRegConfig.bSupressSideInfo,
                                          pFFTAppCfg->fftcQCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum,
                                          pFFTResult)) != FFTC_RETVAL_SUCCESS)
            {
                System_printf ("[Core %d]: Error parsing result, error: %d \n", coreNum, retVal);
                goto error;
            }

            /* verify result */
            System_printf("[Core %d]: Result Info:: Id: %d Error detected: %d Number of FFTC Result Blocks: %d \n", 
                            coreNum, pFFTResult->destnTagInfo, pFFTResult->bErrorDetected, pFFTResult->numFFTCBlocks);

            /* Get the per block result and compare against the expected values
             * to see if the FFT operation succeeded.
             */
            for (j = 0; j < pFFTResult->numFFTCBlocks; j++)
            {
                System_printf ("\n[Core %d]: ********* Block %d Result *********\n", coreNum, j);

                /* Results got */                
                xout            =   (Cplx16 *) pFFTResult->fftBlockResult [j].pBuffer;
                buffLen         =   pFFTResult->fftBlockResult [j].bufferLen;
                blockExpVal     =   pFFTResult->fftBlockResult [j].blockExponentVal;
                clipping        =   pFFTResult->bClippingDetected;

                /* Expected result */
                xoutread        =   (Cplx16 *) pFFTAppCfg->pFftOutputData [j];

                System_printf ("\n[Core %d]: FFT Result details: blockExp: %d Clipping: %d Result buffer len: %d \n", 
                                coreNum, blockExpVal, clipping, buffLen);

                System_printf ("\n[Core %d]: Comparing FFT Result from engine against expected ... \n", coreNum);

                /* Compare the FFT result data block against expected */
	            for (i = 0; i < pFFTAppCfg->numOutputSamples; i++)
	            {
                    /* LSB is not correct. FFTC Sim issue.
                     *
                     * For now, mask off the LSB and compare for
                     * correctness of the FFT processing.
                     */
                    if (((xout[i].real != xoutread[i].real) && 
                        ((xout[i].real - xoutread[i].real != -1) && (xout[i].real - xoutread[i].real != 1))) ||
                        ((xout[i].imag != xoutread[i].imag) && 
                        ((xout[i].imag - xoutread[i].imag != -1) && (xout[i].imag - xoutread[i].imag != 1))))
                    {
#ifdef FFTC_TEST_DEBUG
                        System_printf ("[Core %d]: Real data sample %d: example failed, read: %d actual: %d \n",
                                        coreNum, i, xout[i].real, xoutread[i].real);
                        System_printf ("[Core %d]: Imag data sample %d: example failed, read: %d actual: %d \n",
                                        coreNum, i, xout[i].imag, xoutread[i].imag);
#endif
                    
                        bIsErrorFound ++;
                    }
                }

                if (!bIsErrorFound)
                    System_printf ("[Core %d]: FFT Result Correct !! \n", coreNum);
                else
                    System_printf ("[Core %d]: FFT Result Wrong !! \n", coreNum);			                

                /* Compare the block exponent value got against expected value */
                if (!pFFTAppCfg->fftcQCfg.controlRegConfig.bSupressSideInfo && blockExpVal != pFFTAppCfg->blockExpVal [j])
                {
                    System_printf("[Core %d]: Block Exponent mismatch: example failed, read: %d actual: %d \n",
                                coreNum, blockExpVal, pFFTAppCfg->blockExpVal [j]);
                    bIsErrorFound ++;
                }
                else    
                    System_printf ("[Core %d]: Block Exponent Value Correct !! \n", coreNum);

                /* Compare the clipping detected value got against expected value */
                if (!pFFTAppCfg->fftcQCfg.controlRegConfig.bSupressSideInfo && clipping != pFFTAppCfg->bClippingDetected [j])
                {
                    System_printf("[Core %d]: Clipping Detect mismatch: example failed, read: %d actual: %d \n",
                                coreNum, clipping, pFFTAppCfg->bClippingDetected [j]);
                    bIsErrorFound ++;
                }
                else
                    System_printf ("[Core %d]: Clipping Detect Value Correct !! \n", coreNum);
            }

            /* Done using the result buffer, return the descriptor to Rx FDQ */
            if (Fftc_rxFreeResult (hRxObj, 
                                  hResultInfo
                                  ) < 0)
            {
                System_printf ("[Core %d]: Error freeing result : %d \n", coreNum, i);
                goto error;
            }

		    /* Free the formatted FFT result info memory */
		    if (pFFTResult)
            {
			    Osal_fftcFree (pFFTResult, sizeof (Fftc_Result), FALSE);
                pFFTResult = NULL;
            }
        }
			
	    pktCounter ++;
	}
    	
	if (bIsErrorFound == 0)
	{
		System_printf ("\nFFTC Application Done !!All results received correctly.  \n");                
	}
	else
	{
		System_printf ("\nFFTC Application FAILED !! Num errors = %d \n", bIsErrorFound);                
	}
	
    /* Free up block info memory allocated */
    Osal_fftcFree (blockInfo.blockSizes, sizeof(UInt16) * 1, FALSE);

    /* Free the FFT input, output data and example configuration memory allocated */
    for (i = 0; i < pFFTAppCfg->numBlocks; i ++)
    {
        Osal_fftcFree (pFFTAppCfg->pFftInputData [i], sizeof(Cplx16) * pFFTAppCfg->numInputSamples, FALSE);
        Osal_fftcFree (pFFTAppCfg->pFftOutputData [i], sizeof(Cplx16) * pFFTAppCfg->numOutputSamples, FALSE);
    }
    Osal_fftcFree (pFFTAppCfg, sizeof(*pFFTAppCfg), FALSE);

    /* Close all FFTC handles */
    if (hRxObj)
    	Fftc_rxClose (hRxObj);

    if (hRxFDQ)
        deallocate_fdq (hRxFDQ, hGlblFDQ, rxNumDesc, rxBuffSize);

	if (hTxObj)    	
    	Fftc_txClose (hTxObj);

    if (hTxFDQ)
        deallocate_fdq (hTxFDQ, hGlblFDQ, txNumDesc, txBuffSize);

	/* Return success. */
	return 0;        

error:
    /* Free the FFT result info memory */
    if (pFFTResult)
        Osal_fftcFree (pFFTResult, sizeof (Fftc_Result), FALSE);

    /* Free up block info memory */
    if (blockInfo.blockSizes)
        Osal_fftcFree (blockInfo.blockSizes, sizeof(UInt16) * 1, FALSE);

    /* Free the FFT input, output data and example configuration memory allocated */
    for (i = 0; i < pFFTAppCfg->numBlocks; i ++)
    {
        Osal_fftcFree (pFFTAppCfg->pFftInputData [i], sizeof(Cplx16) * pFFTAppCfg->numInputSamples, FALSE);
        Osal_fftcFree (pFFTAppCfg->pFftOutputData [i], sizeof(Cplx16) * pFFTAppCfg->numOutputSamples, FALSE);
    }
    Osal_fftcFree (pFFTAppCfg, sizeof(*pFFTAppCfg), FALSE);

    /* Close all FFTC handles */
    if (hRxObj)
    	Fftc_rxClose (hRxObj);

    if (hRxFDQ)
        deallocate_fdq (hRxFDQ, hGlblFDQ, rxNumDesc, rxBuffSize);

	if (hTxObj)    	
    	Fftc_txClose (hTxObj);

    if (hTxFDQ)
        deallocate_fdq (hTxFDQ, hGlblFDQ, txNumDesc, txBuffSize);

    /* Return error */
    return -1;
}


/**
 * ============================================================================
 *  @n@b fftc_app
 *
 *  @b  brief
 *  @n  This application illustrates the usage of FFTC driver Higher layer APIs. 
 *      The application submits a multi-block FFT request of sample size 16 using 
 *      CPPI Host descriptor and verifies that the FFT result obtained from the 
 *      engine is in fact what is expected.
 *
 *  @return     
 *  @n  None
 * ============================================================================
 */
void fftc_app (void)
{
    UInt32                      coreNum, queueNum;

    System_printf ("**************************************************\n");
    System_printf ("******** FFTC Multi Core Example Start **********\n");
    System_printf ("**************************************************\n");

    /* Get the core number on which the example is being run */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Driver has been tested only with L1D cache on CX. 
     * Not tested with L2 caches.
     */
    CACHE_setL1DSize (CACHE_L1_MAXIM3);
    CACHE_setL2Size (CACHE_0KCACHE);
    
    /* Initialize the heap in shared memory. Using BIOS IPC module to do that */ 
    Ipc_start();

    System_printf ("Core %d : L1D cache size %d. L2 cache size %d.\n", coreNum, CACHE_getL1DSize(), CACHE_getL2Size());

    /* Power on FFTC */
    if (enable_fftc () != 0)
    {
        System_printf ("[Core %d]: FFTC Power enable failed \n", coreNum);
        return;
    }
    
    /* Pick an FFTC peripheral instance to run the example on. */
    fftcInstNum     =   CSL_FFTC_0;
    
    /* Initialize the system:
     *      -   Init CPPI and QMSS libraries 
     *      -   Init the FFTC instance we are using
     */
    if (coreNum == SYS_INIT_CORE)
    {
        /* Initialize the system:
         *      -   Init CPPI and QMSS libraries 
         *      -   Init the FFTC driver 
         */
        if (system_init (coreNum, fftcInstNum) != 0)
        {
            System_printf ("[Core %d]: FFTC Example system init failed \n", coreNum);
            return;
        }

        while ((CSL_semAcquireDirect (FFTC_APP_SEM)) == 0);
        Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        bIsSysInitDone ++;
        Fftc_osalEndMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        CSL_semReleaseSemaphore (FFTC_APP_SEM);
    }
    else
    {
        System_printf ("[Core %d]: Waiting for Sys Init to be completed ... \n", coreNum);
        do 
        {
            Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        } while (!bIsSysInitDone);

        /* Start Queue manager on this core */
        Qmss_start ();
        
        while ((CSL_semAcquireDirect (FFTC_APP_SEM)) == 0);
        Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        bIsSysInitDone ++;
        Fftc_osalEndMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        CSL_semReleaseSemaphore (FFTC_APP_SEM);        
    }

    /* Setup and open the FFTC driver for use for this example app. */
    if (fftc_setup (coreNum, fftcInstNum) != 0)
    {
        System_printf ("[Core %d]: FFTC Example setup failed \n", coreNum);
        goto cleanup_and_exit;
    }

    /* Lets run 16 sample size request using FFTC Tx queue 0. */
    queueNum    =   0;

    /* Run the multicore example using application managed configuration */
    fftc_run_app_managed_example (coreNum, queueNum);

    /* Close the FFTC driver and free up any resources
     * allocated for this example.
     */
    Fftc_close (hFFTC);
    
cleanup_and_exit:
    if (coreNum == SYS_INIT_CORE)
    {
        while ((CSL_semAcquireDirect (FFTC_APP_SEM)) == 0);
        Fftc_osalBeginMemAccess ((void *)&bIsCoreExampleDone, sizeof(UInt32));
        bIsCoreExampleDone ++;
        Fftc_osalEndMemAccess ((void *)&bIsCoreExampleDone, sizeof(UInt32));
        CSL_semReleaseSemaphore (FFTC_APP_SEM);
            	
        System_printf ("[Core %d]: Waiting for other cores to finish ... \n", coreNum);
        do 
        {
            Fftc_osalBeginMemAccess ((void *)&bIsCoreExampleDone, sizeof(UInt32));
            Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        } while (bIsCoreExampleDone != bIsSysInitDone);

        /* Test Done. De-init the system */
        system_deInit (coreNum);
    }
    else
    {
        while ((CSL_semAcquireDirect (FFTC_APP_SEM)) == 0);
        Fftc_osalBeginMemAccess ((void *)&bIsCoreExampleDone, sizeof(UInt32));
        bIsCoreExampleDone ++;
        Fftc_osalEndMemAccess ((void *)&bIsCoreExampleDone, sizeof(UInt32));
        CSL_semReleaseSemaphore (FFTC_APP_SEM);
        
        do 
        {
            Fftc_osalBeginMemAccess ((void *)&bIsCoreExampleDone, sizeof(UInt32));
            Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        } while (bIsCoreExampleDone != bIsSysInitDone);
    }

    /* Dump memory usage stats */
    System_printf ("FftcAlloc Cnt:\t\t%d FftcFree Cnt:\t%d"
                  "\nCppiMalloc Cnt:\t\t%d CppiFree Cnt:\t\t%d"
                  "\nQmssAlloc Cnt:\t\t%d QmssFree Cnt:\t\t%d\n", 
                  fftcMallocCounter, fftcFreeCounter,
                  fftcCppiMallocCounter, fftcCppiFreeCounter, 
                  fftcQmssMallocCounter, fftcQmssFreeCounter);    

    System_printf ("FFTC Example Application Unloaded successfully \n");

    System_printf ("**************************************************\n");
    System_printf ("********* FFTC Multi Core Example End ***********\n");
    System_printf ("**************************************************\n");

    System_exit(0); 
}

/** ============================================================================
 *   @n@b main
 *
 *   @b Description
 *   @n Entry point for the FFTC-CPPI example application.  
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return
 *   @n None
 *
 * =============================================================================
 */
void main(void)
{
    Task_Params                	fftcAppTaskParams;
    	
    Task_Params_init(&fftcAppTaskParams);

    /* Create the FFTC example task */
    Task_create((Task_FuncPtr)&fftc_app, &fftcAppTaskParams, NULL);

    /* Start the BIOS Task scheduler */
	BIOS_start ();
    
    return;        
}
