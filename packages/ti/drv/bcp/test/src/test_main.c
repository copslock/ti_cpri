 /** 
 *   @file  test_main.c
 *
 *   @brief  
 *          BCP Driver Unit test suite. Runs a series of unit tests to test various 
 *          APIs and application programming knobs exported by the BCP driver.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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
/* BCP test application include */
#include "bcp_test.h"

/* BCP test application input data/ configuration files */

/* QM Accumalator firmware include */
#include <ti/drv/qmss/qmss_firmware.h>

/* CSL INTC include */
#include <ti/csl/src/intc/csl_intc.h>

/* CSL Cache Include */
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

#include <ti/csl/csl_tsc.h>
#include <stdlib.h>

/**************************************************************
************************* GLOBAL VARIABLES ********************
***************************************************************/

/* Host Descriptor Region - [Size of descriptor * Number of descriptors] 
 *
 * MUST be 16 byte aligned.
 */

#pragma DATA_SECTION (hostDesc, ".testData");
#pragma DATA_ALIGN (hostDesc, 16)
UInt8                                   hostDesc[BCP_TEST_SIZE_HOST_DESC * BCP_TEST_NUM_HOST_DESC];

/* Memory usage stats */
extern UInt32                           bcpMallocCounter;
extern UInt32                           bcpFreeCounter;
extern UInt32                           bcpCppiMallocCounter;
extern UInt32                           bcpCppiFreeCounter;
extern UInt32                           bcpQmssMallocCounter;
extern UInt32                           bcpQmssFreeCounter;

UInt32                                  totalNumTestsPass = 0, totalNumTestsFail = 0;

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams          qmssGblCfgParams;

/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams          cppiGblCfgParams;

/* BCP device initialization function */
extern Int32 Bcp_initDevice (Bcp_LldObj*);

#ifndef DEVICE_K2L
/* SRIO device initialization function */
extern Int32 SrioDevice_init (Void);
#endif /* DEVICE_K2L */

/* BCP transport layer callback functions. The transport being used here
 * between BCP device and remote device is SRIO. 
 */
extern Void* srio_tx_open (Void* hVoid);
extern Int32 srio_tx_close (Void* hVoid);
extern Void* srio_rx_open (Void* hVoid);
extern Int32 srio_rx_close (Void* hVoid);
extern Int32 srio_send (Void* hVoid, Void* pPkt, UInt32 pktSize, Void* pRemoteAddress);
extern Int32 srio_recv (Void* hVoid, Void** pPkt);
extern Int32 srio_freeRecvBuffer (Void* hVoid, Void* pPkt, UInt32 pktSize);

/* Define this to indicate that BCP is local to the device */
//#define BCP_LOCAL_DEVICE

/* Define this to build remote BCP test application.Currently supported only on Simulator */
//#define RUN_REMOTE_USECASE_TEST

/* Enable this to build application for run on simulator. */
//#define SIMULATOR_SUPPORT

/* SRIO device Ids for BCP Local and Remote devices. Used to run remote BCP testcase */
#ifdef BCP_LOCAL_DEVICE

/* Current device id */
const uint32_t DEVICE_ID1_16BIT      =   0xBEEF;
const uint32_t DEVICE_ID1_8BIT       =   0xAB;

/* BCP device id */
const uint32_t DEVICE_ID_BCP_16BIT  =   0xBEEF;
const uint32_t DEVICE_ID_BCP_8BIT   =   0xAB;

#else

/* Current device id */
const uint32_t DEVICE_ID1_16BIT     =   0x4560;
const uint32_t DEVICE_ID1_8BIT      =   0xCD;

/* BCP device id */
const uint32_t DEVICE_ID_BCP_16BIT  =   0xBEEF;
const uint32_t DEVICE_ID_BCP_8BIT   =   0xAB;
#endif

const uint32_t DEVICE_ID2_16BIT    = 0x4561;
const uint32_t DEVICE_ID2_8BIT     = 0xCD;
const uint32_t DEVICE_ID3_16BIT    = 0x1234;
const uint32_t DEVICE_ID3_8BIT     = 0x12;
const uint32_t DEVICE_ID4_16BIT    = 0x5678;
const uint32_t DEVICE_ID4_8BIT     = 0x56;

#define SYSINIT_CORE    0

/* Used to synchronize System Init on all cores */
#pragma DATA_SECTION (bIsSysInitDone, ".bcp");
static volatile UInt32                           bIsSysInitDone 	    = 0;

/* Used to synchronize System De-Init on all cores */
#pragma DATA_SECTION (bIsCoreTestDone, ".bcp");
static volatile UInt32                           bIsCoreTestDone 	= 0;

#define BCP_APP_SEM 7

/**************************************************************
************************* TEST FUNCTIONS **********************
***************************************************************/

/**
 * ============================================================================
 *  @n@b dump_memory_usage
 *
 *  @b  brief
 *  @n  Utility function to dump application memory usage stats.
 *
 * ============================================================================
 */
Void dump_memory_usage ()
{
    Bcp_osalLog ("BcpAlloc Cnt:\t\t%d BcpFree Cnt:\t%d"
                  "\nCppiMalloc Cnt:\t\t%d CppiFree Cnt:\t%d"
                  "\nQmssAlloc Cnt:\t\t%d QmssFree Cnt:\t%d\n", 
                  bcpMallocCounter, bcpFreeCounter,
                  bcpCppiMallocCounter, bcpCppiFreeCounter, 
                  bcpQmssMallocCounter, bcpQmssFreeCounter);
    
    return;	
}

/**
 * ============================================================================
 *  @n@b myStartupFxn
 *
 *  @b  brief
 *  @n  
 *      Utility function that is required by the IPC module to set the proc Id.
 *      The processor Id is set via this function instead of hard coding it in the 
 * 		.cfg file
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
UInt32 convert_coreLocalToGlobalAddr 
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

/**
 * ============================================================================
 *  @n@b enable_bcp
 *
 *  @b  brief
 *  @n  This function enables the power/clock domains for BCP. 
 *
 *  @param[in]  
 *
 *  @return     Int32
 *          -1  -   BCP Power/Clock enable failed
 *          0   -   BCP Successfully enabled.
 * 
 * ============================================================================
 */
static Int32 enable_bcp (Void)
{
#ifndef SIMULATOR_SUPPORT
    /* BCP power domain is turned OFF by default. It
     * needs to be turned on before doing any BCP device
     * register access.
     */
    /* Set BCP Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_BCP);

    /* Enable the clocks too for BCP */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_BCP, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_BCP);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_BCP));

    /* Return BCP PSC status */
    if ((CSL_PSC_getPowerDomainState(CSL_PSC_PD_BCP) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (CSL_PSC_LPSC_BCP) == PSC_MODSTATE_ENABLE))
    {
        /* BCP ON. Ready for use */            
        return 0;
    }
    else
    {
        /* BCP Power on failed. Return error */            
        return -1;            
    }
#else
    /* No power up needed on Sim */
    return 0;
#endif
}

/**
 * ============================================================================
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for SRIO. 
 *
 *  @retval
 *      Not Applicable.
 * ============================================================================
 */
#ifndef DEVICE_K2L
static Int32 enable_srio (void)
{
#ifndef SIMULATOR_SUPPORT
    /* SRIO power domain is turned OFF by default. It
     * needs to be turned on before doing any SRIO device
     * register access.
     *
     * This is required only on Silicon. Not required for simulator.
     */

    /* Set SRIO Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_SRIO);

    /* Enable the clocks too for SRIO */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_SRIO, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_SRIO);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_SRIO));

    /* Return SRIO PSC status */
    if ((CSL_PSC_getPowerDomainState(CSL_PSC_PD_SRIO) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (CSL_PSC_LPSC_SRIO) == PSC_MODSTATE_ENABLE))
    {
        /* SRIO ON. Ready for use */            
        return 0;
    }
    else
    {
        /* SRIO Power on failed. Return error */            
        return -1;            
    }
#else
    /* PSC is not supported on simulator. Return success always */
    return 0;
#endif
}
#endif /* DEVICE_K2L */

/**
 * ============================================================================
 *  @n@b system_init
 *
 *  @b  brief
 *  @n  This API is called only once and during system bring up. This API 
 *      initializes the CPPI,QMSS LLDs and BCP driver.
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
    Int32                       result;
    UInt8                       bcpInstNum;
    Qmss_MemRegInfo             memCfg;
    Qmss_InitCfg                qmssInitConfig;
	Bcp_InitCfg      		    bcpInitCfg;
	Bcp_RetVal					retVal;
    Bcp_DrvMode                 mode;
#ifdef BCP_LOCAL_DEVICE
    Bcp_LldObj*                 pBcpLldObj;
#endif
    
    /* Initialize QMSS */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up QMSS configuration. Use internal linking RAM. */
    qmssInitConfig.linkingRAM0Base          =   0;   
    qmssInitConfig.linkingRAM0Size          =   0;
    qmssInitConfig.linkingRAM1Base          =   0;
    qmssInitConfig.maxDescNum               =   BCP_TEST_NUM_HOST_DESC;
#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId   =   Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware =   &acc48_be;
    qmssInitConfig.pdspFirmware[0].size     =   sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId   =   Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware =   &acc48_le;
    qmssInitConfig.pdspFirmware[0].size     =   sizeof (acc48_le);
#endif

    /* Initialize the Queue Manager */
    if ((result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams)) != QMSS_SOK)
    {
        Bcp_osalLog ("[Core %d]: Error initializing QMSS LLD. Error code : %d\n", coreNum, result);
        return -1;
    }

    /* Start Queue manager on this core */
    Qmss_start ();

    /* Initialize CPPI LLD */
    if ((result = Cppi_init (&cppiGblCfgParams)) != CPPI_SOK)
    {
        Bcp_osalLog ("[Core %d]: Error initializing CPPI LLD. Error code : %d\n", coreNum, result);
        return -1;
    }
    
    /* Setup the descriptor memory regions. 
     *
     * The Descriptor base addresses MUST be global addresses and
     * all memory regions MUST be setup in ascending order of the
     * descriptor base addresses.
     */
    memset (hostDesc, 0, BCP_TEST_SIZE_HOST_DESC * BCP_TEST_NUM_HOST_DESC);
    memCfg.descBase         =   (UInt32 *) convert_coreLocalToGlobalAddr ((UInt32) hostDesc);
    memCfg.descSize         =   BCP_TEST_SIZE_HOST_DESC;
    memCfg.descNum          =   BCP_TEST_NUM_HOST_DESC;
    memCfg.manageDescFlag   =   Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memCfg.memRegion        =   Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memCfg.startIndex       =   0;

    /* Insert Host Descriptor memory region */
    result = Qmss_insertMemoryRegion(&memCfg);
    if (result == QMSS_MEMREGION_ALREADY_INITIALIZED)
    {
        Bcp_osalLog ("[Core %d]: Memory Region %d already Initialized \n", coreNum, memCfg.memRegion);
    }
    else if (result < QMSS_SOK)
    {
        Bcp_osalLog ("[Core %d]: Error! Inserting memory region %d, Error code : %d\n", coreNum, memCfg.memRegion, result);
        return -1;
    }    

    /* Initialize BCP driver */
    bcpInstNum  =   CSL_BCP;
#ifdef BCP_LOCAL_DEVICE
    mode        =   Bcp_DrvMode_LOCAL;
#else
    mode        =   Bcp_DrvMode_REMOTE;
#endif
    bcpInitCfg.cpdmaNum                     =   Cppi_CpDma_BCP_CPDMA;
    bcpInitCfg.baseTxQueueNum               =   QMSS_BCP_QUEUE_BASE;
    bcpInitCfg.cfgRegsBaseAddress           =   CSL_BCP_CFG_REGS;
    bcpInitCfg.BcpTunnel_txOpen             =   &srio_tx_open;
    bcpInitCfg.BcpTunnel_txClose            =   &srio_tx_close;
    bcpInitCfg.BcpTunnel_rxOpen             =   &srio_rx_open;   
    bcpInitCfg.BcpTunnel_rxClose            =   &srio_rx_close;
    bcpInitCfg.BcpTunnel_send               =   &srio_send;
    bcpInitCfg.BcpTunnel_recv               =   &srio_recv;
    bcpInitCfg.BcpTunnel_freeRecvBuffer     =   &srio_freeRecvBuffer;
    retVal = Bcp_init (bcpInstNum, mode, &bcpInitCfg);
    if (retVal == BCP_RETVAL_SUCCESS)
    {
        Bcp_osalLog ("[Core %d]: BCP instance %d successfully initialized \n", coreNum, bcpInstNum);            
    }
    else
    {
        Bcp_osalLog ("[Core %d]: BCP init failed on instance %d \n", coreNum, bcpInstNum);
        return -1;
    }

#ifdef BCP_LOCAL_DEVICE
    /* Initialize the BCP Device MMRs as part of the system init. */
    if ((pBcpLldObj = Bcp_getLLDHandle (CSL_BCP)) == NULL)
    {
        Bcp_osalLog ("Failed initializing BCP MMRs. No valid LLD handle. \n");
        return -1;
    }
    Bcp_initDevice (pBcpLldObj);
#endif

#ifndef DEVICE_K2L
    /* Device Specific SRIO Initializations: This should always be called before
     * initializing the SRIO Driver. 
     */
    if (SrioDevice_init() < 0)
        return -1;        

    /* Initialize the SRIO Driver */
    if (Srio_init () < 0)
    {
        Bcp_osalLog ("Error: SRIO Driver Initialization Failed\n");
        return -1;
    }
#endif /* DEVICE_K2L */

    /* System Init Done. Return success */
    return 0;
}    

/**
 * ============================================================================
 *  @n@b system_deInit
 *
 *  @b  brief
 *  @n  This API is to be called at system shutdown. This API 
 *      de-initializes and closes the CPPI,QMSS LLDs and BCP driver.
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
	/* Teardown the BCP driver */
	Bcp_deInit (CSL_BCP);
		
    /* Exit CPPI LLD */        
    Cppi_exit ();
}

#ifdef RUN_REMOTE_USECASE_TEST
/**
 * ============================================================================
 *  @n@b setup_run_bcpremote_task
 *
 *  @b  brief
 *  @n  Sets up BCP remote device and runs BCP tests.
 *
 *  @return     
 *  @n  None
 * ============================================================================
 */
Void setup_run_bcpremote_task ()
{
    UInt32                  coreNum;
    Bcp_DrvHandle           hBcp;
    Bcp_RetVal              retVal;                
    Bcp_DrvCfg              bcpDrvCfg;
    Qmss_QueueHnd           hGlblFDQ;
    Cppi_DescCfg            cppiDescCfg;
    UInt32                  numAllocated;

    /* Get the core number on which the test is being run */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Initialize driver configuration before use */
    memset (&bcpDrvCfg, 0, sizeof (Bcp_DrvCfg));

    /* Open BCP driver */
    hBcp = Bcp_open (CSL_BCP, &bcpDrvCfg, &retVal);
    if (retVal == BCP_RETVAL_SUCCESS)
    {
        Bcp_osalLog ("[Core %d]: BCP driver successfully opened \n", coreNum);            
    }
    else
    {
        Bcp_osalLog ("[Core %d]: BCP driver open failed. Error code: %d \n", coreNum, retVal);
        goto cleanup_and_exit;
    }

    /* Initialize and setup free descriptors for use by the application. */
    memset (&cppiDescCfg, 0, sizeof (cppiDescCfg));
    cppiDescCfg.memRegion               =   Qmss_MemRegion_MEMORY_REGION0;
    cppiDescCfg.descNum                 =   BCP_TEST_NUM_HOST_DESC/2;
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
        Bcp_osalLog ("Error setting up Global FDQ, Error: %d \n", hGlblFDQ);
        return;
    }        
    else
    {
        Bcp_osalLog ("Global FDQ %d successfully setup with %d descriptors\n", hGlblFDQ, numAllocated);
    }

    /* Run the BCP client - dispatches BCP requests to be processed at BCP on a remote device */
    Bcp_osalLog ("\n------ Starting BCP client task on BCP remote device -------\n");
    run_bcp_client (hBcp, hGlblFDQ);

    Bcp_osalLog ("\n-------------- BCP Testing Complete --------------\n");
    Bcp_osalLog ("Num Test Failures = %d\n", totalNumTestsFail);

    /* Close the BCP driver and free up any resources allocated for this test. */
    Bcp_close (hBcp);
    
cleanup_and_exit:

    Bcp_osalLog ("**************************************************\n");
    Bcp_osalLog ("****************** BCP Testing End **************\n");
    Bcp_osalLog ("**************************************************\n");

    Bcp_osalLog ("[Core %d]: BCP Test Unloaded successfully \n", coreNum);

    /* Dump memory usage stats */
    dump_memory_usage ();

    Task_exit ();

    return;
}

/**
 * ============================================================================
 *  @n@b setup_run_bcplocal_task
 *
 *  @b  brief
 *  @n  Sets up the BCP local device and runs tests on BCP.
 *
 *  @return     
 *  @n  None
 * ============================================================================
 */
Void setup_run_bcplocal_task ()
{
    UInt32                  coreNum;
    Bcp_DrvHandle           hBcp;
    Bcp_RetVal              retVal;                
    Bcp_DrvCfg              bcpDrvCfg;
    Qmss_QueueHnd           hGlblFDQ;
    Cppi_DescCfg            cppiDescCfg;
    UInt32                  numAllocated;

    /* Get the core number on which the test is being run */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Initialize driver configuration before use */
    memset (&bcpDrvCfg, 0, sizeof (Bcp_DrvCfg));

    /* Open BCP driver */
    hBcp = Bcp_open (CSL_BCP, &bcpDrvCfg, &retVal);
    if (retVal == BCP_RETVAL_SUCCESS)
    {
        Bcp_osalLog ("[Core %d]: BCP driver successfully opened \n", coreNum);            
    }
    else
    {
        Bcp_osalLog ("[Core %d]: BCP driver open failed. Error code: %d \n", coreNum, retVal);
        goto cleanup_and_exit;
    }

    /* Initialize and setup free descriptors for use by the application. */
    memset (&cppiDescCfg, 0, sizeof (cppiDescCfg));
    cppiDescCfg.memRegion               =   Qmss_MemRegion_MEMORY_REGION0;
    cppiDescCfg.descNum                 =   BCP_TEST_NUM_HOST_DESC/2;
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
        Bcp_osalLog ("Error setting up Global FDQ, Error: %d \n", hGlblFDQ);
        return;
    }        
    else
    {
        Bcp_osalLog ("Global FDQ %d successfully setup with %d descriptors\n", hGlblFDQ, numAllocated);
    }

    /* Run BCP server. Accepts remote BCP requests, processes them. */
    Bcp_osalLog ("\n--------- Starting BCP server task on BCP local device ---------\n");
    run_bcp_server (hBcp, hGlblFDQ);

    Bcp_osalLog ("\n-------------- BCP Testing Complete --------------\n");
    Bcp_osalLog ("Num Test Failures = %d\n", totalNumTestsFail);

    /* Close the BCP driver and free up any resources allocated for this test. */
    Bcp_close (hBcp);
    
cleanup_and_exit:

    Bcp_osalLog ("**************************************************\n");
    Bcp_osalLog ("****************** BCP Testing End **************\n");
    Bcp_osalLog ("**************************************************\n");

    Bcp_osalLog ("[Core %d]: BCP Test Unloaded successfully \n", coreNum);

    /* Dump memory usage stats */
    dump_memory_usage ();

    Task_exit ();

    return;
}
#endif

/**
 * ============================================================================
 *  @n@b setup_run_bcp_test
 *
 *  @b  brief
 *  @n  This API sets up the test environment, test parameters, calls the test 
 *      core API to actually run the test and finally de-initializes the system
 *      once done.
 *
 *  @return     
 *  @n  None
 * ============================================================================
 */
Void setup_run_bcp_test ()
{
    UInt32                  coreNum;
    Bcp_DrvHandle           hBcp;
    Bcp_RetVal              retVal;                
    Bcp_DrvCfg              bcpDrvCfg;
    Qmss_QueueHnd           hGlblFDQ;
    Cppi_DescCfg            cppiDescCfg;
    UInt32                  numAllocated;

    /* Get the core number on which the test is being run */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Initialize driver configuration before use */
    memset (&bcpDrvCfg, 0, sizeof (Bcp_DrvCfg));

    /* Open BCP driver */
    hBcp = Bcp_open (0, &bcpDrvCfg, &retVal);
    if (retVal == BCP_RETVAL_SUCCESS)
    {
        Bcp_osalLog ("[Core %d]: BCP driver successfully opened \n", coreNum);            
    }
    else
    {
        Bcp_osalLog ("[Core %d]: BCP driver open failed. Error code: %d \n", coreNum, retVal);
        goto cleanup_and_exit;
    }
   
    /* Initialize and setup free descriptors for use by the application. */
    memset (&cppiDescCfg, 0, sizeof (cppiDescCfg));
    cppiDescCfg.memRegion               =   Qmss_MemRegion_MEMORY_REGION0;
    cppiDescCfg.descNum                 =   BCP_TEST_NUM_HOST_DESC/4;
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
        printf ("Error setting up Global FDQ, Error: %d \n", hGlblFDQ);
        return;
    }        
    else
    {
        printf ("Global FDQ %d successfully setup with %d descriptors\n", hGlblFDQ, numAllocated);
    }

    /* Run the Downlink/Uplink tests */
    Bcp_osalLog ("\n-------------- Rel99 DL Test Start ---------------\n");
    test_wcdma_rel99_dl (hBcp, hGlblFDQ);

    Bcp_osalLog ("\n-------------- Rel99 UL Test Start ---------------\n");
    test_wcdma_rel99_ul (hBcp, hGlblFDQ);

    Bcp_osalLog ("\n-------------- WCDMA DL Test Start --------------\n");
    test_wcdma_dl (hBcp, hGlblFDQ);

    Bcp_osalLog ("\n-------------- WCDMA UL Test Start --------------\n");
	test_wcdma_ul (hBcp, hGlblFDQ);

#ifdef TESTS_NOT_WORKING
	Bcp_osalLog ("\n-------------- WiMax DL Test Start ---------------\n");
    test_wimax_dl  (hBcp, hGlblFDQ);

    Bcp_osalLog ("\n-------------- WiMax UL Test Start ---------------\n");
    test_wimax_ul  (hBcp, hGlblFDQ);    
#endif

    Bcp_osalLog ("\n-------------- LTE UL Test Start --------------\n");
    test_lte_ul (hBcp, hGlblFDQ);

    Bcp_osalLog ("\n-------------- LTE DL Test Start --------------\n");
    test_lte_dl (hBcp, hGlblFDQ);

    Bcp_osalLog ("\n-------------- BCP Testing Complete --------------\n");
    Bcp_osalLog ("Num Test Failures = %d\n", totalNumTestsFail);

    while ((CSL_semAcquireDirect (BCP_APP_SEM)) == 0);
    Bcp_osalBeginMemAccess ((void *)&bIsCoreTestDone, sizeof(UInt32));
    bIsCoreTestDone ++;
    Bcp_osalEndMemAccess ((void *)&bIsCoreTestDone, sizeof(UInt32));
    CSL_semReleaseSemaphore (BCP_APP_SEM);
            	
    Bcp_osalLog ("Waiting for other cores to finish ... \n");
    do 
    {
        Bcp_osalBeginMemAccess ((void *)&bIsCoreTestDone, sizeof(UInt32));
        Bcp_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
    } while (bIsCoreTestDone < bIsSysInitDone);

    /* Close the BCP driver and free up any resources allocated for this test. */
    Bcp_close (hBcp);
    
cleanup_and_exit:

    Bcp_osalLog ("**************************************************\n");
    Bcp_osalLog ("****************** BCP Testing End **************\n");
    Bcp_osalLog ("**************************************************\n");

    Bcp_osalLog ("[Core %d]: BCP Test Unloaded successfully \n", coreNum);

    /* Dump memory usage stats */
    dump_memory_usage ();

    return;
}

/** ============================================================================
 *   @n@b main
 *
 *   @b Description
 *   @n Entry point for the BCP unit test application. This application tests 
 *      BCP processing using the the driver's Higher layer APIs. 
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return
 *   @n None
 *
 * =============================================================================
 */
Void main(Void)
{
    Task_Params             testTaskParams;
    UInt32                  coreNum;

    /* Get the core number on which the test is being run */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);
        	
    Bcp_osalLog ("**************************************************\n");
    Bcp_osalLog ("*************** BCP Testing Start ***************\n");
    Bcp_osalLog ("**************************************************\n");
    
    Bcp_osalLog ("Using BCP Driver version: 0x%x Version Info: %s \n", Bcp_getVersionID (), Bcp_getVersionStr ());

    /* Disable all caches for initial testing */
    CACHE_setL2Size (CACHE_0KCACHE);
    CACHE_setL1DSize (CACHE_L1_0KCACHE);
    Bcp_osalLog ("[Core %d]: L1D cache size %d. L2 cache size %d.\n", coreNum, CACHE_getL1DSize(), CACHE_getL2Size());

    CSL_tscEnable();

	Ipc_start ();

#if defined(RUN_REMOTE_USECASE_TEST) && !defined(SIMULATOR_SUPPORT)
    {
        /* Error. Remote use case supported only on Simulator. */
        Bcp_osalLog ("Error! Remote use case application supported only on Simulator! \n");
        exit (0);
    }
#endif

    if (coreNum == SYSINIT_CORE)
    {
#ifdef BCP_LOCAL_DEVICE
        /* Power on BCP */
        if (enable_bcp () != 0)
        {
            Bcp_osalLog ("[Core %d]: BCP Power enable failed \n", coreNum);
            return;
        }
#endif

#ifndef DEVICE_K2L
        /* Power up SRIO. We'll be using SRIO to talk to BCP on remote device */
        if (enable_srio () != 0)
        {
            Bcp_osalLog ("[Core %d]: SRIO Power enable failed \n", coreNum);
            return;
        }
#endif /* DEVICE_K2L */

        /* Do the system level initialization stuff:
         *  -   Init CPPI and QMSS libraries 
         *  -   Init the BCP instance we are using
         */
        if (system_init (coreNum) != 0)
        {   
            Bcp_osalLog ("[Core %d]: BCP Test system init failed \n", coreNum);
            return;
        }

        while ((CSL_semAcquireDirect (BCP_APP_SEM)) == 0);
        Bcp_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        bIsSysInitDone ++;
        Bcp_osalEndMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        CSL_semReleaseSemaphore (BCP_APP_SEM);
    }
    else
    {
        Bcp_osalLog ("Waiting for Sys Init to be completed ... \n");
        do 
        {
            Bcp_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        } while (!bIsSysInitDone);

        /* Start Queue manager on this core */
        Qmss_start ();
        
        while ((CSL_semAcquireDirect (BCP_APP_SEM)) == 0);
        Bcp_osalBeginMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        bIsSysInitDone ++;
        Bcp_osalEndMemAccess ((void *)&bIsSysInitDone, sizeof(UInt32));
        CSL_semReleaseSemaphore (BCP_APP_SEM);      
    }

#if defined(RUN_REMOTE_USECASE_TEST) && defined(SIMULATOR_SUPPORT)
    {
        /* Build Remote use case application for simulator */
#ifdef BCP_LOCAL_DEVICE
        {
            /* Create another task to simulate device with BCP */
            Task_Params_init (&testTaskParams);
            testTaskParams.stackSize = 4096;
            Task_create((Task_FuncPtr)&setup_run_bcplocal_task, &testTaskParams, NULL);
        }
#else
        {
            /* Create a task to simulate a remote device that has no BCP locally */
            Task_Params_init (&testTaskParams);
            testTaskParams.stackSize = 4096;
            Task_create((Task_FuncPtr)&setup_run_bcpremote_task, &testTaskParams, NULL);
        }
#endif
    }
#else
    {
        /* Create the BCP unit test task */
        Task_Params_init (&testTaskParams);
        testTaskParams.stackSize = 4096;
        Task_create((Task_FuncPtr)&setup_run_bcp_test, &testTaskParams, NULL);
    }
#endif

    /* Start the BIOS Task scheduler */
	BIOS_start ();
  
    /* Test Done. De-init the system */
    system_deInit (coreNum);

    return;        
}
