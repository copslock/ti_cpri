 /** 
 *   @file  test_main.c
 *
 *   @brief  
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, 2014, Texas Instruments, Inc.
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
/* FFTC test application include */
#include "fftc_test.h"

/* FFTC test application input data/ configuration files */
#include "fftc_testcfg_16.h"
#include "fftc_testcfg_48.h"
#include "fftc_testcfg_540.h"
#include "fftc_testcfg_2048.h"
#include "fftc_testcfg_zeropad_inputshift_cpadd_2048.h"
#include "fftc_testcfg_DFTlist.h"

#include <ti/csl/csl_qm_queue.h>

/* QM Accumalator firmware include */
#include <ti/drv/qmss/qmss_firmware.h>

/* CSL INTC include */
#include <ti/csl/src/intc/csl_intc.h>

/* CSL Cache Include */
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

/**************************************************************
************************* GLOBAL VARIABLES ********************
***************************************************************/

/* Monolithic Descriptor Region - [Size of descriptor * Number of descriptors] 
 *
 * MUST be 16 byte aligned.
 */
#pragma DATA_ALIGN (monoDesc, 16)
UInt8                                   monoDesc[FFTC_TEST_SIZE_MONOLITHIC_DESC * FFTC_TEST_NUM_MONOLITHIC_DESC];


/* Host Descriptor Region - [Size of descriptor * Number of descriptors] 
 *
 * MUST be 16 byte aligned.
 */

#pragma DATA_ALIGN (hostDesc, 16)
UInt8                                   hostDesc[FFTC_TEST_SIZE_HOST_DESC * FFTC_TEST_NUM_HOST_DESC];

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


UInt32                                  totalNumTestsPass = 0, totalNumTestsFail = 0;
UInt32                                  numInstanceTasksCreated = 0;

/* Used to synchronize System Init on all cores */
#pragma DATA_SECTION (bIsSysInitDone, ".fftc");
static volatile UInt32                           bIsSysInitDone         = 0;

/* Used to synchronize System De-Init on all cores */
#pragma DATA_SECTION (bIsCoreTestDone, ".fftc");
static volatile UInt32                           bIsCoreTestDone    = 0;

/* Used to synchronize driver configuration on all cores */
#pragma DATA_SECTION (bIsInitTestDone, ".fftc");
static volatile UInt32                           bIsInitTestDone    = 0;

/* Tests multiple instances when enabled */
//#define TEST_MULTIPLE_INSTANCES

/* Builds multicore test when enabled */
//#define   TEST_MULTICORE

/* XMC and MSMC address translations are not supported in Sim. 
 * Don't enable L2 caches. Use L1D caches. 
 */
#undef  L2_CACHE  

#define MAPPED_VIRTUAL_ADDRESS      0x81000000

/* MPAX segment 2 registers */
#define XMPAXL2                     0x08000010 
#define XMPAXH2                     0x08000014

/**************************************************************
************************* TEST FUNCTIONS **********************
***************************************************************/
/**
 * ============================================================================
 *  @n@b init_done
 *
 *  @b  brief
 *  @n  
 *      Utility function to mark a core's test setup done status.
 *
 * ============================================================================
 */
Void init_done ()
{
    /* Signal that we are done with Tx/Rx configuration. 
     * Ready to send/receive.
     */
    while ((CSL_semAcquireDirect (FFTC_APP_SEM)) == 0);
    Fftc_osalBeginMemAccess ((void *)&bIsInitTestDone, sizeof(UInt32));    
    bIsInitTestDone ++;
    Fftc_osalEndMemAccess ((void *)&bIsInitTestDone, sizeof(UInt32));
    CSL_semReleaseSemaphore (FFTC_APP_SEM);     
}

/**
 * ============================================================================
 *  @n@b wait_init_done
 *
 *  @b  brief
 *  @n  
 *      Utility function that when called blocks the calling task until
 *      all the core's test setup is not done.
 *
 * ============================================================================
 */
Void wait_init_done ()
{
    
   /* Used to synchronize the Rx flow creation on all cores.
    * This is done to ensure that the Rx flow to which this
    * core (sender) is sending data is actually up.
    */    
    do 
    {
        Fftc_osalBeginMemAccess ((void *)&bIsInitTestDone, sizeof(UInt32));
        Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
    }while (bIsInitTestDone != bIsSysInitDone);
}

/**
 * ============================================================================
 *  @n@b myStartupFxn
 *
 *  @b  brief
 *  @n  
 *      Utility function that is required by the IPC module to set the proc Id.
 *      The processor Id is set via this function instead of hard coding it in the 
 *      .cfg file
 *
 * ============================================================================
 */
Void myStartupFxn (Void)
{
    MultiProc_setLocalId (CSL_chipReadReg (CSL_CHIP_DNUM));
}

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
 *  @return     Int32
 *  @li         -1  -   Invalid configuration, system bring up failed
 *  @li         0   -   System Init succeeded
 *
 * ============================================================================
 */
Int32 system_init 
(
    UInt32                      coreNum
)
{
    Int32                       result, i, fftcInstNum;
    Qmss_MemRegInfo             memCfg;
    Qmss_InitCfg                qmssInitConfig;
    Fftc_GlobalCfg              fftcGlobalCfg;
    Fftc_DeviceCfg              fftcDevCfg;
    Fftc_RetVal                 retVal;
    
    /* Initialize QMSS */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up QMSS configuration.
     *  
     * Use internal linking RAM.
     */
    qmssInitConfig.linkingRAM0Base  =   0;   
    qmssInitConfig.linkingRAM0Size  =   0;
    qmssInitConfig.linkingRAM1Base  =   0;
    qmssInitConfig.maxDescNum       =   FFTC_TEST_NUM_MONOLITHIC_DESC + FFTC_TEST_NUM_HOST_DESC;

#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_le);
#endif

    /* Initialize the Queue Manager */
    if ((result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams)) != QMSS_SOK)
    {
        Fftc_osalLog ("[Core %d]: Error initializing Queue Manager SubSystem, Error code : %d\n", coreNum, result);
        return -1;
    }

    /* Start Queue manager on this core */
    Qmss_start ();

    /* Initialize CPPI LLD */
    if ((result = Cppi_init (&cppiGblCfgParams)) != CPPI_SOK)
    {
        Fftc_osalLog ("[Core %d]: Error initializing CPPI LLD, Error code : %d\n", coreNum, result);
        return -1;
    }
    
    
    /* Setup the descriptor memory regions. 
     *
     * The Descriptor base addresses MUST be global addresses and
     * all memory regions MUST be setup in ascending order of the
     * descriptor base addresses.
     */

    /* Initialize and setup FFTC Monolithic Descriptors required for test */
    memset (monoDesc, 0, FFTC_TEST_SIZE_MONOLITHIC_DESC * FFTC_TEST_NUM_MONOLITHIC_DESC);
    memCfg.descBase         =   (UInt32 *) convert_coreLocalToGlobalAddr ((UInt32) monoDesc);
    memCfg.descSize         =   FFTC_TEST_SIZE_MONOLITHIC_DESC;
    memCfg.descNum          =   FFTC_TEST_NUM_MONOLITHIC_DESC;
    memCfg.manageDescFlag   =   Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memCfg.memRegion        =   Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memCfg.startIndex       =   0;

    /* Insert Monolithic Descriptor memory region */
    result = Qmss_insertMemoryRegion(&memCfg);
    if (result == QMSS_MEMREGION_ALREADY_INITIALIZED)
    {
        Fftc_osalLog ("[Core %d]: Memory Region %d already Initialized \n", coreNum, memCfg.memRegion);
    }
    else if (result < QMSS_SOK)
    {
        Fftc_osalLog ("[Core %d]: Error! Inserting memory region %d, Error code : %d\n", coreNum, memCfg.memRegion, result);
        return -1;
    }    

    /* Initialize and setup FFTC Host Descriptors required for test */
    memset (hostDesc, 0, FFTC_TEST_SIZE_HOST_DESC * FFTC_TEST_NUM_HOST_DESC);
    memCfg.descBase         =   (UInt32 *) convert_coreLocalToGlobalAddr ((UInt32) hostDesc);
    memCfg.descSize         =   FFTC_TEST_SIZE_HOST_DESC;
    memCfg.descNum          =   FFTC_TEST_NUM_HOST_DESC;
    memCfg.manageDescFlag   =   Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memCfg.memRegion        =   Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memCfg.startIndex       =   0;

    /* Insert Host Descriptor memory region */
    result = Qmss_insertMemoryRegion(&memCfg);
    if (result == QMSS_MEMREGION_ALREADY_INITIALIZED)
    {
        Fftc_osalLog ("[Core %d]: Memory Region %d already Initialized \n", coreNum, memCfg.memRegion);
    }
    else if (result < QMSS_SOK)
    {
        Fftc_osalLog ("[Core %d]: Error! Inserting memory region %d, Error code : %d\n", coreNum, memCfg.memRegion, result);
        return -1;
    }    

    /* Set up the FFTC Configuration Register */
    for (i = 0; i < FFTC_MAX_NUM_TXQUEUES; i ++)
    {
        fftcGlobalCfg.queueFlowidOverwrite [i]   =   0;
        fftcGlobalCfg.queuePriority [i]          =   0;
    }
    fftcGlobalCfg.starvationPeriodVal            =   0x0;
    fftcGlobalCfg.bDisableFFT                    =   0;
    
    /* Initialize FFTC driver for instance number specified. */
    fftcInstNum             =   CSL_FFTC_0;    
    fftcDevCfg.cpdmaNum     =   Cppi_CpDma_FFTC_A_CPDMA;
    fftcDevCfg.baseQueueNum =   QMSS_FFTC_A_QUEUE_BASE;            
    fftcDevCfg.cfgRegs      =   (Void *) (CSL_FFTC_0_CFG_REGS);
    retVal = Fftc_init (fftcInstNum, &fftcGlobalCfg, &fftcDevCfg);
    if (retVal == FFTC_RETVAL_SUCCESS)
    {
        Fftc_osalLog ("[Core %d]: FFTC instance %d successfully initialized \n", coreNum, fftcInstNum);            
    }
    else
    {
        Fftc_osalLog ("[Core %d]: FFTC init failed on instance %d \n", coreNum, fftcInstNum);
        return -1;
    }
    
    /* Initialize FFTC driver for instance number specified. */
    fftcInstNum             =   CSL_FFTC_1;
    fftcDevCfg.cpdmaNum     =   Cppi_CpDma_FFTC_B_CPDMA;
    fftcDevCfg.baseQueueNum =   QMSS_FFTC_B_QUEUE_BASE;            
    fftcDevCfg.cfgRegs      =   (Void *) (CSL_FFTC_1_CFG_REGS);
    retVal = Fftc_init (fftcInstNum, &fftcGlobalCfg, &fftcDevCfg);
    if (retVal == FFTC_RETVAL_SUCCESS)
    {
        Fftc_osalLog ("[Core %d]: FFTC instance %d successfully initialized \n", coreNum, fftcInstNum);            
    }
    else
    {
        Fftc_osalLog ("[Core %d]: FFTC init failed on instance %d \n", coreNum, fftcInstNum);
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
 *  @param[in]  coreNum
 *      Current DSP core number that is doing the system init
 *
 *  @param[in]  None
 *
 *  @return     None
 * ============================================================================
 */
Void system_deInit
(
    UInt32              coreNum
)
{
    /* Teardown both the FFTC instances */
    if (Fftc_isInitialized (CSL_FFTC_0))
        Fftc_deInit (CSL_FFTC_0);
        
    if (Fftc_isInitialized (CSL_FFTC_1))
        Fftc_deInit (CSL_FFTC_1);

    /* Exit CPPI LLD */        
    Cppi_exit ();
}

/**
 * ============================================================================
 *  @n@b fftc_setup
 *
 *  @b  brief
 *  @n  This API sets up the configuration of FFTC driver and opens it 
 *      for the test.
 *
 *  @param[in]  coreNum
 *      Current DSP core number that is doing the system init
 *
 *  @param[in]  fftcInstNum
 *      FFTC instance number to use
 *
 *  @return     Int32
 *              NULL     -   FFTC setup failed.
 *              >0       -   FFTC setup successful.
 * ============================================================================
 */
Fftc_DrvHandle fftc_setup 
(
    UInt32                  coreNum,
    UInt8                   fftcInstNum
)
{
    Fftc_RetVal             retVal;                
    Fftc_DrvCfg             fftcInitCfg;
    Fftc_DrvHandle          hFFTC;

    /* Initialize driver configuration before use */
    memset (&fftcInitCfg, 0, sizeof (Fftc_DrvCfg));

    /* Setup the FFTC Driver Init Descriptor Params */        

    /* CPPI library gives all the memory regions in the order
     * of insertion, for now let's assume we got Mem region
     * 0 for for Host.
     */
    fftcInitCfg.cppiNumFreeDescCfg                  =   2;
    fftcInitCfg.cppiFreeDescCfg[0].descMemRegion    =   Qmss_MemRegion_MEMORY_REGION0;
    fftcInitCfg.cppiFreeDescCfg[0].numDesc          =   FFTC_TEST_NUM_MONOLITHIC_DESC/(2 * NUM_CORES);
    fftcInitCfg.cppiFreeDescCfg[0].descSize         =   FFTC_TEST_SIZE_MONOLITHIC_DESC;
    fftcInitCfg.cppiFreeDescCfg[0].descType         =   Cppi_DescType_MONOLITHIC;

    fftcInitCfg.cppiFreeDescCfg[1].descMemRegion    =   Qmss_MemRegion_MEMORY_REGION1;
    fftcInitCfg.cppiFreeDescCfg[1].numDesc          =   FFTC_TEST_NUM_HOST_DESC/(2 * NUM_CORES);
    fftcInitCfg.cppiFreeDescCfg[1].descSize         =   FFTC_TEST_SIZE_HOST_DESC;
    fftcInitCfg.cppiFreeDescCfg[1].descType         =   Cppi_DescType_HOST;

    /* Setup the descriptors required by this app and
     * open the driver.
     */
    hFFTC = Fftc_open (fftcInstNum, &fftcInitCfg, &retVal);
    if (retVal == FFTC_RETVAL_SUCCESS)
    {
        Fftc_osalLog ("[Core %d]: FFTC successfully opened \n", coreNum);            
    }
    else
    {
        Fftc_osalLog ("[Core %d]: FFTC open failed \n", coreNum);
        return NULL;
    }

    /* FFTC driver global setup done. Return driver handle. */
    return hFFTC;
}

/**
 * ============================================================================
 *  @n@b fftc_parse_testCfg
 *
 *  @b  brief
 *  @n  This API reads the input configuration file and retrieves the FFT
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
 *  @param[in]
 *      pBlockInfo      FFTC Input block info.
 *
 *  @return     Int32
 *              -1      -   FFTC configuration read failed.
 *              0       -   FFTC configuration read successful.
 * ============================================================================
 */
Int32 fftc_parse_testCfg (UInt32 testCaseId, FFT_TestCfg* pFFTTestCfg, Fftc_BlockInfo* pBlockInfo)
{
    Int32                       tmp, i, j, bIsMixedSizeDFT;
    Int16                       *cfgReadPtr = NULL;
    Cplx16*                     xread;
    UInt32                      coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    switch (testCaseId)
    {
        case 1: 
        {
            cfgReadPtr  =   inputConfig_16;
            break;                        
        }
        case 2: 
        {
            cfgReadPtr  =   inputConfig_48;
            break;                        
        }
        case 3: 
        {
            cfgReadPtr  =   inputConfig_540;
            break;                        
        }
        case 4: 
        {
            cfgReadPtr  =   inputConfig_2048;
            break;                        
        }
        case 5: 
        {
            cfgReadPtr  =   inputConfig_dftList;
            break;                        
        }
        case 6: 
        {
            cfgReadPtr  =   inputConfig_zeropad_inputshift_cpadd_2048;
            break;                        
        }
    }

    /* Initialize the test configuration */
    memset (pFFTTestCfg, 0, sizeof (FFT_TestCfg));
    
    /* Get the number of FFT blocks */
    pFFTTestCfg->numBlocks                                              =   (UInt16) *cfgReadPtr++;

    /* Scan the DFT size in words */
    pFFTTestCfg->fftcQCfg.controlRegConfig.dftSize                      =   (UInt16) *cfgReadPtr++;
    
    /* Supress error info? 
     *
     * By default disable side-band info
     */
    pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo             =   0;

    cfgReadPtr += 3;

    /* Get the rest of FFT Queue configuration */
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.bDynamicScaleEnable  =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingValLast  =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[0]   =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[1]   =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[2]   =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[3]   =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[4]   =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[5]   =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.radixScalingVal[6]   =   (UInt16) *cfgReadPtr++;
    cfgReadPtr += 3;
    tmp = (UInt16) *cfgReadPtr++;
    if (tmp == 0)
        pFFTTestCfg->fftcQCfg.controlRegConfig.dftMode                    =   Fftc_DFTMode_DFT;
    else
        pFFTTestCfg->fftcQCfg.controlRegConfig.dftMode                    =   Fftc_DFTMode_IDFT;

    pFFTTestCfg->fftcQCfg.controlRegConfig.bEmulateDSP16x16             =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.outputScaleVal       =   (UInt16) *cfgReadPtr++;

    tmp = (UInt16) *cfgReadPtr++;
    if (tmp > 1)
    {
        /* Variable input shift feature being configured. Turn off the left/right input shifting */ 
        pFFTTestCfg->fftcQCfg.destQRegConfig.bInputFFTShift             =   0;
        pFFTTestCfg->fftcQCfg.destQRegConfig.inputShiftVal              =   tmp/2; /* Config file has the shift val doubled. */
        pFFTTestCfg->fftcQCfg.controlRegConfig.bSupressSideInfo         =   1;
    }
    else
    {
        /* Left/right input shifting being configured. Turn off variable input shifting */
        pFFTTestCfg->fftcQCfg.destQRegConfig.bInputFFTShift             =   tmp;
        pFFTTestCfg->fftcQCfg.destQRegConfig.inputShiftVal              =   0; 
    }
    pFFTTestCfg->fftcQCfg.destQRegConfig.bOutputFFTShift                =   (UInt16) *cfgReadPtr++;
    
    pFFTTestCfg->fftcQCfg.controlRegConfig.bZeroPadEnable               =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.controlRegConfig.zeroPadMode                  =   (Fftc_ZeroPadMode) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.controlRegConfig.zeroPadFactor                =   (UInt16) *cfgReadPtr++;

    pFFTTestCfg->fftcQCfg.freqShiftRegConfig.bFreqShiftEnable           =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.scalingShiftingRegConfig.freqShiftScaleVal    =   (UInt16) *cfgReadPtr++;
    tmp = (UInt16) *cfgReadPtr++;
    if (tmp == 8192)
    {
        pFFTTestCfg->fftcQCfg.freqShiftRegConfig.freqShiftIndex         =   Fftc_FreqShiftIndex_16384;
    }
    else
    {
        pFFTTestCfg->fftcQCfg.freqShiftRegConfig.freqShiftIndex         =   Fftc_FreqShiftIndex_12288;
    }
    pFFTTestCfg->fftcQCfg.freqShiftRegConfig.freqShiftMultFactor        =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.freqShiftRegConfig.freqShiftInitPhase         =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.freqShiftRegConfig.freqShiftDirection         =   (Fftc_FreqShiftDir) *cfgReadPtr++;
    
    pFFTTestCfg->fftcQCfg.cyclicPrefixRegConfig.bCyclicPrefixAddEnable  =   (UInt16) *cfgReadPtr++;
    pFFTTestCfg->fftcQCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum      =   (UInt16) *cfgReadPtr++;

    /* Always set Dest Queue Number in the queue configuration to 0x3FFF, 
     * since the destination queue number is setup during flow setup.
     */
    pFFTTestCfg->fftcQCfg.destQRegConfig.cppiDestQNum                   =   0x3fff;    

    /* Initialize the block info.
     *
     */
    memset (pBlockInfo, 0, sizeof(Fftc_BlockInfo));
    pBlockInfo->numBlocks   =   pFFTTestCfg->numBlocks;
    pBlockInfo->blockSizes  =   (UInt16 *) Fftc_osalMalloc (sizeof(UInt16) * pFFTTestCfg->numBlocks, FALSE);

    /* Get the input and output data info for each of the blocks now */
    for (i = 0; i < pFFTTestCfg->numBlocks; i ++)
    {
        /* Read the block number */
        tmp = (UInt16) *cfgReadPtr++;

        /* Check if this configuration is a mixed size DFT configuration or
         * if all the block sizes are of the same size.
         *
         * For mixed size DFT configuration, the block number is replaced with
         * DFT size for that block. We use this data to figure out whether its
         * a mixed size DFT or not.
         */
        if (i == 0)
        {
            if (tmp != 1)
            {
                /* If we are here, indicates that the configuration is
                 * mixed size DFT block list.
                 */
                bIsMixedSizeDFT = 1;                
                pBlockInfo->bIsEqualSize    =   0;
            }
            else
            {
                /* Indicates that all blocks are of same size. */                    
                bIsMixedSizeDFT = 0;                
                pBlockInfo->bIsEqualSize    =   1;
            }
        }

        if (!bIsMixedSizeDFT)
        {
            /* Caclulate the actual number of data samples 
             * 
             * Number of valid data samples = DFT block size - zero pad factor. 
             *
             * The config file has data to match DFT size (include zero pad samples). Lets just read
             * valid data and leave out zero pad samples.
             */
            if (pFFTTestCfg->fftcQCfg.controlRegConfig.bZeroPadEnable)
            {
                pFFTTestCfg->numInputSamples [i]    =   (pFFTTestCfg->fftcQCfg.controlRegConfig.dftSize - 
                                                         pFFTTestCfg->fftcQCfg.controlRegConfig.zeroPadFactor);
            }
            else
            {
                pFFTTestCfg->numInputSamples [i]    =   (pFFTTestCfg->fftcQCfg.controlRegConfig.dftSize);
            }

            /* Populate the FFT block size info */
            pBlockInfo->blockSizes[i]               =   pFFTTestCfg->fftcQCfg.controlRegConfig.dftSize;             
        }
        else
        {
            /* Read DFT size for this block. */
            if (pFFTTestCfg->fftcQCfg.controlRegConfig.bZeroPadEnable)
            {
                pFFTTestCfg->numInputSamples [i]    =   (tmp - pFFTTestCfg->fftcQCfg.controlRegConfig.zeroPadFactor);
            }
            else
            {
                pFFTTestCfg->numInputSamples [i]    =   (tmp);              
            }
            
            /* Populate the FFT block size info */
            pBlockInfo->blockSizes[i]               =   tmp;             
        }

        /* Allocate memory for the input data for this block */
        if (!(pFFTTestCfg->pFftInputData [i] = (Cplx16 *) Fftc_osalMalloc (sizeof (Cplx16) *  pFFTTestCfg->numInputSamples[i], FALSE)))
        {
            Fftc_osalLog ("[Core %d]: Error allocating memory for input data block: %d \n", coreNum, i);

            /* Free up all the input data blocks allocated so far */
            for (j = 0; j < i-1; j ++)
            {
                Fftc_osalFree (pFFTTestCfg->pFftInputData [j], sizeof (Cplx16) *  pFFTTestCfg->numInputSamples[i], FALSE);                  
            }

            /* Return error */
            return -1;
        }
   
        /* Read input data for this block. */
        xread = pFFTTestCfg->pFftInputData [i];
        for (j = 0; j < pFFTTestCfg->numInputSamples[i]; j++)  
        {   
            xread[j].real               =   *cfgReadPtr++;
        }
        /* Read only valid samples. Ignore the zero pad samples */
        for (j = 0; pFFTTestCfg->fftcQCfg.controlRegConfig.bZeroPadEnable && j < pFFTTestCfg->fftcQCfg.controlRegConfig.zeroPadFactor; j++)  
            *cfgReadPtr++;
    
        for (j = 0; j < pFFTTestCfg->numInputSamples[i]; j++)
        {
            xread[j].imag               =   *cfgReadPtr++;
        }
        /* Read only valid samples. Ignore the zero pad samples */
        for (j = 0; pFFTTestCfg->fftcQCfg.controlRegConfig.bZeroPadEnable && j < pFFTTestCfg->fftcQCfg.controlRegConfig.zeroPadFactor; j++)  
            *cfgReadPtr++;

        pFFTTestCfg->blockExpVal [i]        = (UInt16) *cfgReadPtr++;
        pFFTTestCfg->bClippingDetected [i]  = (UInt16) *cfgReadPtr++;
        pFFTTestCfg->numClockCycles [i]     = (UInt16) *cfgReadPtr++;

        /* Account for zero pad samples too. */
        pFFTTestCfg->numOutputSamples[i]  = pFFTTestCfg->numInputSamples[i];
        if (pFFTTestCfg->fftcQCfg.controlRegConfig.bZeroPadEnable)
            pFFTTestCfg->numOutputSamples[i] += pFFTTestCfg->fftcQCfg.controlRegConfig.zeroPadFactor;
        if (pFFTTestCfg->fftcQCfg.cyclicPrefixRegConfig.bCyclicPrefixAddEnable)
            pFFTTestCfg->numOutputSamples[i] += pFFTTestCfg->fftcQCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum;

        /* Allocate memory for the output data for this block */
        if (!(pFFTTestCfg->pFftOutputData [i] = (Cplx16 *) Fftc_osalMalloc (sizeof (Cplx16) *  pFFTTestCfg->numOutputSamples[i], FALSE)))
        {
            Fftc_osalLog ("[Core %d]: Error allocating memory for output data block: %d \n", coreNum, i);

            /* Free up all the input data blocks allocated so far */
            for (j = 0; j < i; j ++)
            {
                Fftc_osalFree (pFFTTestCfg->pFftInputData [j], sizeof (Cplx16) *  pFFTTestCfg->numInputSamples[i], FALSE);                  
            }   

            /* Return error */
            return -1;
        }

        /* Read output data */
        xread = pFFTTestCfg->pFftOutputData [i];
        for (j = 0; j < pFFTTestCfg->numOutputSamples[i]; j++)
        {
            xread[j].real               =   *cfgReadPtr++;
        }

        for (j = 0; j < pFFTTestCfg->numOutputSamples[i]; j++)
        {
            xread[j].imag               =   *cfgReadPtr++;
        }    
    }

    /* Return Success */
    return 0;
}

Int32 fftc_clean_testCfg (FFT_TestCfg* pFFTTestCfg,Fftc_BlockInfo* pBlockInfo)
{
    Int32       i;
    
    /* Free up block info memory allocated */
    Fftc_osalFree (pBlockInfo->blockSizes, sizeof(UInt16) * 1, FALSE);

    /* Free the FFT input, output data and test configuration memory allocated */
    for (i = 0; i < pFFTTestCfg->numBlocks; i ++)
    {
        Fftc_osalFree (pFFTTestCfg->pFftInputData [i], sizeof(Cplx16) * pFFTTestCfg->numInputSamples[0], FALSE);
        Fftc_osalFree (pFFTTestCfg->pFftOutputData [i], sizeof(Cplx16) * pFFTTestCfg->numOutputSamples[0], FALSE);
    }
    Fftc_osalFree (pFFTTestCfg, sizeof(*pFFTTestCfg), FALSE);

    return 0;
}


/**
 * ============================================================================
 *  @n@b test_fftc
 *
 *  @b  brief
 *  @n  This API sets up the test environment, test parameters, calls the test 
 *      core API to actually run the test and finally de-initializes the system.
 *
 *  @return     
 *  @n  None
 * ============================================================================
 */
Void test_fftc 
(
    UInt8               fftcInstNum 
)
{
    UInt32              coreNum;
    Fftc_DrvHandle      hFFTC;

    /* Get the core number on which the test is being run */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Setup FFTC interrupt configuration for this core 
     * and open the FFTC driver for use for this test app. 
     */
    if ((hFFTC = fftc_setup (coreNum, fftcInstNum)) == NULL)
    {
        Fftc_osalLog ("[Core %d]: FFTC Test setup failed for FFTC instance %d \n", coreNum, fftcInstNum);
        goto cleanup_and_exit;
    }

    /* Run all the tests back to back. */
#if defined(TEST_MULTIPLE_INSTANCES)
    Fftc_osalLog ("-------------- Testing FFTC Instance: %d Start --------------\n", fftcInstNum);
    if (coreNum == SYS_INIT_CORE)
    {
        test_host_singlecore_multiInst (fftcInstNum, hFFTC);
        test_host_singlecore_queueshare (fftcInstNum, hFFTC);
    }
#elif defined(TEST_MULTICORE)
    test_multicore (hFFTC);
#else
    if (coreNum == SYS_INIT_CORE)
    {
        test_fftc_lld ();
        test_mono_singlecore (hFFTC);
        test_mono_singlecore_psinfo (hFFTC);
        test_host_singlecore_poll (hFFTC);
        test_host_singlecore (hFFTC);
        test_host_singlecore_psinfo (hFFTC);
        test_singlecore_shift (hFFTC);
        test_singlecore_dftlist (hFFTC);
        test_host_singlecore_flowshare (fftcInstNum, hFFTC);
    }    
#endif

    Fftc_osalLog ("-------------- Testing FFTC Instance: %d Complete --------------\n", fftcInstNum);
    Fftc_osalLog ("Num Test Failures on Instance %d = %d\n", fftcInstNum, totalNumTestsFail);

    while ((CSL_semAcquireDirect (FFTC_APP_SEM)) == 0);
    Fftc_osalBeginMemAccess ((void *)&bIsCoreTestDone, sizeof(UInt32));
    bIsCoreTestDone ++;
    Fftc_osalEndMemAccess ((void *)&bIsCoreTestDone, sizeof(UInt32));
    CSL_semReleaseSemaphore (FFTC_APP_SEM);
                
    Fftc_osalLog ("Waiting for other cores to finish ... \n");
    do 
    {
        Fftc_osalBeginMemAccess ((void *)&bIsCoreTestDone, sizeof(UInt32));
        Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
    } while (bIsCoreTestDone < bIsSysInitDone);
    
    if (coreNum == SYS_INIT_CORE)
    {
        /* Close the FFTC driver and free up any resources
         * allocated for this test.
         */
        Fftc_close (hFFTC);
    
        /* Wait for tasks instances to be completed before continuing
         * with deInit and eventual System_exit call.
         */
        numInstanceTasksCreated--;

        while (numInstanceTasksCreated)
        {
            Task_yield();
        }

        /* Test Done. De-init the system */
        system_deInit (coreNum);
    }
    else
    {
        /* Close the FFTC driver and free up any resources
         * allocated for this test.
         */
        Fftc_close (hFFTC);
    }

    Fftc_osalLog ("**************************************************\n");
    Fftc_osalLog ("****************** FFTC Testing End **************\n");
    Fftc_osalLog ("**************************************************\n");


cleanup_and_exit:
    Fftc_osalLog ("[Core %d]: FFTC Test Unloaded successfully \n", coreNum);

    /* Dump memory usage stats */
    Fftc_osalLog ("FftcAlloc Cnt:\t\t%d FftcFree Cnt:\t%d"
                  "\nCppiMalloc Cnt:\t\t%d CppiFree Cnt:\t\t%d"
                  "\nQmssAlloc Cnt:\t\t%d QmssFree Cnt:\t\t%d\n", 
                  fftcMallocCounter, fftcFreeCounter,
                  fftcCppiMallocCounter, fftcCppiFreeCounter, 
                  fftcQmssMallocCounter, fftcQmssFreeCounter);

    Task_exit();
}

/** ============================================================================
 *   @n@b main
 *
 *   @b Description
 *   @n Entry point for the FFTC-CPPI test application. This application tests 
 *      FFTC engine processing using the FFTC driver's Higher layer APIs. 
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
    Task_Params                 testTaskParams;
    UInt32                      coreNum;
    UInt8                       fftcInstNum;
#ifdef L2_CACHE
    uint32_t                    *xmpaxPtr;
#endif

    /* Get the core number on which the test is being run */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
            
    Fftc_osalLog ("**************************************************\n");
    Fftc_osalLog ("*************** FFTC Testing Start ***************\n");
    Fftc_osalLog ("**************************************************\n");
    
    Fftc_osalLog ("Using FFTC Driver version: 0x%x Version Info: %s \n", Fftc_getVersionID (), 
                    Fftc_getVersionStr ());

#ifdef L2_CACHE
    /* Set L2 cache to 512KB */
    CACHE_setL2Size (CACHE_512KCACHE);
#endif

    /* Driver has been tested only with L1D cache on CX. 
     * Not tested with L2 caches.
     */
#ifdef TEST_MULTICORE
    CACHE_setL1DSize (CACHE_L1_0KCACHE);
#else
    CACHE_setL1DSize (CACHE_L1_MAXIM3);
#endif

    Fftc_osalLog ("Core %d : L1D cache size %d. L2 cache size %d.\n", coreNum, CACHE_getL1DSize(), CACHE_getL2Size());

    /* Initialize the heap in shared memory. Using BIOS IPC module to do that */ 
    Ipc_start();

    /* Do the system level initialization stuff:
     *      -   Init CPPI and QMSS libraries 
     *      -   Init the FFTC instance we are using
     */
    if (coreNum == SYS_INIT_CORE)
    {
#ifdef L2_CACHE
        /* Define an MPAX segment in the virtual (CGEM) address space. 
         * Map MSMC physical address to virtual address.
         * Configure all +rwx permissions.
         */

        /* Phy address base: 0x0C00 0000
         * Size: 1MB (0x13 according to encoding)
         * Virtual address: 0x8100 0000 
         * Permission: 0xFF
         * MAR used: (0x8100 0000 >> 24) = 129
         */

        /* map using MPAX segment 2 registers */
        xmpaxPtr  = (uint32_t *)(XMPAXH2);
        *xmpaxPtr = ((MAPPED_VIRTUAL_ADDRESS >> 12) << 12) | (0x13);

        xmpaxPtr  = (uint32_t *)(XMPAXL2);
        *xmpaxPtr = ((0x0c000000 >> 12) << 8) | (0xFF);
    
        /* Enable caching for MAR 129. CSL does not define these MARS. Define a macro */
        CACHE_enableCaching ((MAPPED_VIRTUAL_ADDRESS) >> 24);
#endif 
    
        /* Power on FFTC */
        if (enable_fftc () != 0)
        {
            Fftc_osalLog ("[Core %d]: FFTC Power enable failed \n", coreNum);
            System_exit(0);
        }
        
        if (system_init (coreNum) != 0)
        {
            Fftc_osalLog ("[Core %d]: FFTC Test system init failed \n", coreNum);
            System_exit(0);
        }

        while ((CSL_semAcquireDirect (FFTC_APP_SEM)) == 0);
        Fftc_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        bIsSysInitDone ++;
        Fftc_osalEndMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        CSL_semReleaseSemaphore (FFTC_APP_SEM);

        fftcInstNum = CSL_FFTC_0;
        Task_Params_init(&testTaskParams);
        testTaskParams.arg0 =   fftcInstNum;

        numInstanceTasksCreated++;
        /* Create the FFTC test task */
        Task_create((Task_FuncPtr)&test_fftc, &testTaskParams, NULL);

#ifdef TEST_MULTIPLE_INSTANCES
        fftcInstNum = CSL_FFTC_1;
        Task_Params_init(&testTaskParams);
        testTaskParams.arg0 =   fftcInstNum;

        numInstanceTasksCreated++;
        /* Create the FFTC test task */
        Task_create((Task_FuncPtr)&test_fftc, &testTaskParams, NULL);
#endif
    }
    else
    {
        Fftc_osalLog ("Waiting for Sys Init to be completed ... \n");
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

        fftcInstNum = CSL_FFTC_0;
        Task_Params_init(&testTaskParams);
        testTaskParams.arg0 =   fftcInstNum;

        /* Create the FFTC test task */
        Task_create((Task_FuncPtr)&test_fftc, &testTaskParams, NULL);
    }

    /* Start the BIOS Task scheduler */
    BIOS_start ();
  
    return;        
}
