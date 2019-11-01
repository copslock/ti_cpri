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



#include <stdio.h>
#include <time.h>
#include <string.h>

/* XDC includes */
#include <xdc/runtime/IHeap.h>

/* BIOS includes */
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#include <ti/sysbios/family/c66/tci66xx/CpIntc.h>

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>

/* CSL includes */
#include <ti/csl/soc.h>
#include <ti/csl/csl_chipAux.h>
#include <ti/csl/cslr_tpcc.h>
#include <ti/csl/cslr_tcp3d_cfg.h>
#include <ti/csl/cslr_tcp3d_dma.h>
#include <ti/csl/cslr_tcp3d_dma_offsets.h>
#include <ti/csl/csl_tsc.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/csl_pscAux.h>

#include "sample.h"
#include "tcp3d_drv_sample.h"
#include "tcp3d_main.h"

/**********************************************************************
 ************************** Test Compile Flags ************************
 **********************************************************************/
#define DEBUG_VARS                  0   // add debug variables at different places in the code 

#define DEBUG_PRINT                 0

#define MY_LOG_PRINT0(x) {\
    if(DEBUG_PRINT)\
    {\
        System_printf(#x"\n");\
    }\
}

#define MY_LOG_PRINT1(x, val) {\
    if(DEBUG_PRINT)\
    {\
        System_printf(#x" %d\n", val);\
    }\
}

#define MY_LOG_PRINT2(x, val1, y, val2) {\
    if(DEBUG_PRINT)\
    {\
        System_printf(#x" %d, "#y" %d\n", val1, val2);\
    }\
}

/**
 * This is used for plotting timing diagram using matlab by collecting the data
 * points from running the code.
 * 
 * Requirements:
 *  1) it is required to enable the logging for TCP3D in the Simulator
 *      Configuration file. The decoding and reset times are collected from the
 *      log file (c:\\Tcp3dDebug.log) generated from running the simulator.
 */
#define TEST_PROFILE_LOG             1 

/**********************************************************************
 ************************** Test Definitions **************************
 **********************************************************************/
#define START_CMD_PERIOD            1


#ifdef CSL_PSC_PD_TCP3D_A
/* Convert Keystone CSL definitions ti Keystone2 CSL definitions */
#define CSL_PSC_PD_TCP3D_01     CSL_PSC_PD_TCP3D_A
#define CSL_PSC_LPSC_TCP3D_0    CSL_PSC_LPSC_TCP3D_A
#endif

/**********************************************************************
 ************************** Test Variables ****************************
 **********************************************************************/
/* Code Block Test Variables */
#pragma DATA_SECTION(codeBlockSet, ".main_mem")
cbTestDesc              codeBlockSet;

Char                    *strMode[4] = {"3GPP(0)","LTE(1)","WIMAX(2)","WCDMA Split(3)"};
Char                    *strDBuf[2] = {"Disable(0)","Enable(1)"};
Char                    *strInst[2] = {"TCP3D_0(0)","TCP3D_1(1)"};

UInt32                  keepTestVectMem = 0;
UInt32                  firstTime = 1;
UInt32                  testMaxCodeBlocks;
UInt32                  dspCoreID;

/* File Operation Variables */
UInt32                  testCntr = 0;
UInt32                  testErrCntr = 0;
UInt32                  totErrCnt;
#ifdef  USE_PDK_INSTALL_BASE
Char                    testvectFolderBase[] = "C:\\ti\\csl_lld_keystone2_1_0_0_2\\packages\\ti\\drv\\tcp3d\\test\\gen_test_vectors\\";
#else
Char                    testvectFolderBase[] = "..\\..\\gen_test_vectors\\";
#endif
Char                    folderName[1000] = "";
Char                    *testFolder[] = {
                                            "test0_lte",
                                            "test0_wimax",
                                            "test0_wcdma",
                                            "WIMAX",
                                            "test1_lte",
                                            "test2_lte",
                                            "WCDMA",
                                            "test4_lte",
                                            "test3_lte",
                                            "test1_wimax", // gave error on simulator earlier
                                            "test3_wimax",
                                            "test2_wimax",
                                            "test2_wcdma",
                                            "LTE",
                                            "test3_wcdma",
                                            "test1_wcdma",
                                            "sim_config\\WIMAX",
                                            "sim_config\\LTE",
                                            "sim_config\\WCDMA",
                                            // "LTE_200",
                                            // "WCDMA_200",
                                            // "WIMAX_200",
};
/* Number of test folders computed. Alternately can be set manually as length of *testFolder[] array */
UInt32                  numTests = sizeof(testFolder)/sizeof(*testFolder);

/* Throughput calculation variables */
clock_t                 total_clock_end, total_clock_start;
UInt32                  test_cycles;
UInt32                  TotalBitsDecoded;
Float                   ThroughPut;

/* BIOS variables */
Semaphore_Handle        semRcvDone, semSendBlock, semSendWait, semRcvStart;
IHeap_Handle            dataHeap = NULL;
IHeap_Handle            drvHeap = NULL;

/* Driver configuration variables */
Tcp3d_Result            tcp3dResultSend = TCP3D_DRV_NO_ERR;
Tcp3d_Instance          *tcp3dDrvInst[2] = {NULL, NULL};
Tcp3d_Instance          *inst;
Tcp3d_Ctrl              drvCtrl;
Tcp3d_Sts               drvStatus;

Int32                   sendBlockCnt;
Int32                   rcvBlockCnt;
#if TEST_PREPARE_ONLY_CODEBLOCK_PARAM_DEPENDENT
UInt32                  tempICRegs[15]; /* to store 15 registers */
#endif

/**
 * EDMA3 LLD & TCP3D Driver Init/Deinit related variables
 */
EDMA3_DRV_Handle        hEdma;
UInt32                  tpccNum;
UInt32                  tpccRegionUsed;
EDMA_CONFIG             edmaConfig[2];
UInt8                   instNum;
/* Flags used in ISR functions */
UInt32                  pingComplete, pongComplete;
UInt32                  pauseIntr = 0, l2pIntr = 0;
UInt32                  soldoutCntr = 0;
UInt32                  tcp3dEventCntr = 0;
UInt32                  tpccEvtCntr = 0;
UInt32                  rcvStartFlag = 0;
UInt32                  pauseIntFlag = 0;
UInt32                  afterIntrSoldout = 0, afterIntrPause = 0;
UInt32                  pendPauseCntr = 0;

/**
 * PROFILE LOG related variables
 */
#if TEST_PROFILE_LOG
#include "tcp3d_profile.h"
volatile PROFILE_TAG    profileTag[PROF_TAG_LEN];
volatile UInt32         profileTagInd = 0;
#else // TEST_PROFILE_LOG
/* Dummy macros */
#define PROF_LOG_COMPLETE() {}
#define PROF_LOG_INIT() {}
#define LOG_TIME(TASK, TAG, TIME) {}
#define LOG_TIME_ISR(TASK, TAG, TIME) {}
#endif // TEST_PROFILE_LOG

#if DEBUG_VARS
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_cpIntcAux.h>

/* To track the RCV task posting */
volatile Int            semCnt;

/* Register pointers */
CSL_TpccRegs            *tpcc2Regs = (CSL_TpccRegs *) CSL_EDMA2CC_REGS;
CSL_CPINTC_RegsOvly     cpintc0Regs = (CSL_CPINTC_RegsOvly) CSL_CP_INTC_0_REGS;
CSL_IntcRegsOvly        gemIntcRegs = (CSL_IntcRegsOvly)CSL_CGEM0_5_REG_BASE_ADDRESS_REGS;
#endif

/**********************************************************************
 *********************** Test Local Functions *************************
 **********************************************************************/
/**
 * Task Functions
 */
Void testerTaskFunc(Void);
Void tskHeartBeat(Void);
Void sndBlockTaskFunc(Void);
Void rcvBlockTaskFunc(Void);

/**
 * (De)Init Functions
 */
Void allInit(Void);
Void allDeInit(Void);
Void getMemoryStats(Void);

/**
 * EDMA Channel ISR functions
 */
Void revt0ChCallback(Void);
Void revt1ChCallback(Void);

/**
 * Cache and other IP functions
 */
#ifndef SIMULATOR_SUPPORT
static Int32 enable_ip_instance (UInt32 pwrDmnNum, UInt32 moduleNum);
#endif
static Int32 enable_tcp3d (void);
void tcp3dBeginMemAccess (void *ptr, uint32_t size);
void tcp3dEndMemAccess (void *ptr, uint32_t size);

#if USE_LOCAL_CPINTC_DISPATCH
extern Void CpIntc_dispatchLoc(UInt hostInt);
#endif

Void soldOutAction(Void)
{
    /* clear flag */
    pauseIntFlag = 0;

#if SOLDOUT_USE_L2P_INTERRUPT
    /**
     *  1) enable L2P channel interrupt to get notified to try enqueue again
     *  2) also enable REVT channel interrupt for PAUSE detection
     */
    /* Set interrupt flag on PAUSE channel */
    drvCtrl.cmd = TCP3D_DRV_SET_REVT_INT;
    drvCtrl.intrFlag = TEST_INTR_ENABLE;   // enable
    Tcp3d_control(inst, &drvCtrl);

    /* Call TCP3D driver control to set interrupt on L2P channel */
    drvCtrl.cmd = TCP3D_DRV_SET_L2P_INT;
    drvCtrl.intrFlag = TEST_INTR_ENABLE;   // enable
    Tcp3d_control(inst, &drvCtrl);
#else
    /* keep trying until successful */
    Semaphore_post(semSendBlock);
#endif
}

Void soldOutActionClear (Void)
{
#if SOLDOUT_USE_L2P_INTERRUPT
    if ( pauseIntFlag )
    {
        afterIntrSoldout++;
        
        /**
         *  1) diable L2P channel interrupt to get notified to try enqueue again
         *  2) also disable REVT channel interrupt for PAUSE detection
         */
        /* Call TCP3D driver control to set interrupt on L2P channel */
        drvCtrl.cmd = TCP3D_DRV_SET_L2P_INT;
        drvCtrl.intrFlag = TEST_INTR_DISABLE;   // disable
        Tcp3d_control(inst, &drvCtrl);

        /* Set interrupt flag on PAUSE channel */
        drvCtrl.cmd = TCP3D_DRV_SET_REVT_INT;
        drvCtrl.intrFlag = TEST_INTR_DISABLE;   // disable
        Tcp3d_control(inst, &drvCtrl);
    }
#else
    /* nothing to be done */
#endif
}

/*******************************************************************************
 TESTING METHOD 1: Copy test vector folder to workspace
--------------------------------------------------------------------------------
1.  In this method, ensure that the entire “test\gen_test_vectors” folder is
    placed two levels higher than the unit test project .out file location.
    For example, if the unit test project out file is located under
    “C:\MyPDKWorkspace\tcp3dTestProject\Debug” folder, then please copy
    the folder “test\gen_test_vectors” from the PDK package to “C:\MyPDKWorkspace”.
2.  Next, execute the batch script “gen_test_vectors\genTestVect.bat” from the
    copied location to generate all the necessary test vector files.
3.  Ensure that the compile flag USE_PDK_INSTALL_BASE is undefined in
    the project “tcp3dTestProject” before building.
4.	Build the project.
5.	Launch the debug session to load and run the out file.

 TESTING METHOD 2: Use the test vector folder from the PDK installation
--------------------------------------------------------------------------------
1.  In this method, ensure that the compile flag USE_PDK_INSTALL_BASE is
    defined in the project “tcp3dTestProject”.
2.	Also, ensure the variable “testvectFolderBase” defined in the tcp3d_main.c
    file is set to “<PDK_INSTALL_DIR>\packages\ti\drv\tcp3d\test\gen_test_vectors”
    when the USE_PDK_INSTALL_BASE flag is defined.
3.	Next, execute the batch script “gen_test_vectors\genTestVect.bat” from the
    PDK installation location to generate all the necessary test vector files.
4.	Build the project.
5.	Launch the debug session to load and run the out file.

By default, test project is configured to test using "METHOD 2"
******************************************************************************/
/*
 * main()
 */
Void main(Void)
{
    Task_Params taskParams;

    /* Initialize the heap in shared memory. Using IPC module to do that */ 
    Ipc_start();

    /* Power on TCP3D peripheral before using it */
    if (enable_tcp3d () < 0)
    {
        System_printf ("Error: TCP3D PSC Initialization Failed\n");
        return;
    }
    
    /* Enable time stamp counter */
    CSL_tscEnable();

    /* Enable L1D cache. Disable L2 caching for our tests. */
    CACHE_setL1DSize (CACHE_L1_MAXIM3); /* CACHE_L1_0KCACHE */
    CACHE_setL2Size (CACHE_0KCACHE);

    /* Initialize the default Task parameters */
    Task_Params_init(&taskParams);

    /* Crete the tester Task using default Task parameters */
    Task_create((Task_FuncPtr)testerTaskFunc, &taskParams, NULL);

    BIOS_start();
}

/*******************************************************************************
 ******************************************************************************/
Void testerTaskFunc(Void)
{
    Int                 i;
    Task_Params         taskParams;
    Semaphore_Params    semParams;

    /* Set the one-time global test variables */
    testMaxCodeBlocks   = 8; /* max possible used in init */
    dspCoreID           = CSL_chipReadDNUM();

    /******** Select the TCP3D Instance Number **********/
    instNum = getTcp3dInstNum(dspCoreID);

    /******** Clear TCP3D log file **********/
    PROF_LOG_INIT(instNum);

    /* Initialize the default Task parameters */
    Task_Params_init(&taskParams);

    /* Initialize the default Semaphore parameters */
    Semaphore_Params_init(&semParams);

    /* Crete the Binary Semaphore */
    semParams.mode = Semaphore_Mode_BINARY;
    semRcvDone = Semaphore_create(0, &semParams, NULL);

    /* Get the Heap handles - used when ever memory allocations are needed */
    //dataHeap = HeapMem_Handle_upCast(tcp3dDataHeap);
    dataHeap = (IHeap_Handle) SharedRegion_getHeap(0);
    drvHeap = HeapMem_Handle_upCast(tcp3dDrvHeap);

    while( testCntr < numTests )
    {
        LOG_TIME(0, PROF_START, TSCL); 
        LOG_TIME(0, PROF_STOP, TSCL); 

        /**
         * Create the Binary semaphores each time using the parameters set
         * outside the tester while loop.
         * 
         * It was observed that at times the receive semaphore count was
         * non-zero after the first run and receive task was getting triggered
         * before posting from the ISR callback. So, the semaphores are created
         * for each test to work-around with the problem. 
         */
        semSendBlock = Semaphore_create(0, &semParams, NULL);
        semSendWait = Semaphore_create(0, &semParams, NULL);
        semRcvStart = Semaphore_create(0, &semParams, NULL);
#if DEBUG_VARS
        semCnt = Semaphore_getCount(semRcvStart);
#endif

        /**
         * Create the send and receive tasks for each test using the default
         * tak parameters.
         * 
         * NOTE: No need to do the Task_delete() as these tasks have exits.
         */
        Task_create((Task_FuncPtr)sndBlockTaskFunc, &taskParams, NULL);
        Task_create((Task_FuncPtr)rcvBlockTaskFunc, &taskParams, NULL);

        System_printf("\n******************************************************************\n");
        System_printf("\n----- TEST #%d STARTED ------\n", testCntr);

        /**
         * Prepare data for Code Blocks processing (reading test vector files).
         * Allocates Memory as needed from the tcp3dDataHeap
         */
        if ( firstTime )
        {
            System_printf("\nReading test vector files started (including memory allocation)...\n");
            strcpy(folderName, testvectFolderBase);
            strcat(folderName, testFolder[testCntr]);
            getTestSetCB(dataHeap, &codeBlockSet, folderName);
            System_printf("Reading test vector files complete\n");
#if DEBUG_PRINT
            System_printf("\tPrepared %d code blocks in %s mode\n", codeBlockSet.maxNumCB, strMode[codeBlockSet.mode]);
#endif
            firstTime = 0;
        }
        else
        {
            System_printf("\nUsing the test vectors read before\n");
        }

        System_printf("\n----- TEST INITIALIZATION STARTED -----\n\n");
        allInit();
        getMemoryStats(); /* Heap Stats */
        System_printf("\n----- TEST INITIALIZATION COMPLETE -----\n\n");

#if TEST_PREPARE_ONLY_BETASTATE
        for (i = 0; i < codeBlockSet.maxNumCB ;i++)
        {
            /* Prepare fixed IC registers using the inCfgParams of first block*/
            Tcp3d_prepFixedConfigRegs(codeBlockSet.cbData[i]->inCfgParams, codeBlockSet.cbData[i]->inCfg);

            /* Prepare block size dependent params */
            prepareBlockSizeDepICParams(codeBlockSet.cbData[i]);
        }
#elif TEST_PREPARE_ONLY_CODEBLOCK_PARAM_DEPENDENT
        /* Prepare fixed IC registers using the inCfgParams of first block*/
        Tcp3d_prepFixedConfigRegs(codeBlockSet.cbData[0]->inCfgParams, tempICRegs);
#endif

        /* Start the Send task first */
        Semaphore_post(semSendBlock);

        /* Wait for the Receive task to complete */
        Semaphore_pend(semRcvDone, BIOS_WAIT_FOREVER);

        /**
         * Test Profile Calculations
         * 
         *                              (Total Bits)
         * Throughput (Mbps) = -----------------------------
         *                      (Total Time)*(10^-9)*(10^6)
         * 
         */
        TotalBitsDecoded = 0;
        for (i = 0; i < codeBlockSet.maxNumCB; ++i)
        {
            TotalBitsDecoded += codeBlockSet.cbData[i]->blockSize;
        }

        test_cycles = (total_clock_end - total_clock_start);
        ThroughPut = TotalBitsDecoded*1.0;
        ThroughPut = (ThroughPut/test_cycles)*1000;

        /******** Free code blocks ********/
        if ( keepTestVectMem )
        {
            System_printf("\nNo freeing - Using the test vectors read before\n");
        }
        else
        {
            System_printf("\nTest vectors memory freeing started...\n");
            freeTestSetCB(dataHeap, &codeBlockSet);
            System_printf("Test vectors memory freeing complete\n");
#if DEBUG_PRINT
            System_printf("\tFreed memory allocated for %d code blocks in %s mode\n", codeBlockSet.maxNumCB, strMode[codeBlockSet.mode]);
#endif
            firstTime = 1;
        }

        System_printf("\n----- TEST DE-INITIALIZATION STARTED -----\n\n");
        allDeInit();
        getMemoryStats(); /* Heap Stats */
        System_printf("\n----- TEST DE-INITIALIZATION COMPLETE -----\n");
    
        if ( totErrCnt > 0 )
        {
            System_printf("\n----- TEST #%d FAILED -----\n", testCntr);
            testErrCntr++;
        }
        else
        {
            System_printf("\n----- TEST #%d PASSED -----\n", testCntr);
        }
        System_printf("\n+++++++++++++++++++++++ TEST #%d SUMMARY +++++++++++++++++++++++++\n", testCntr);
        System_printf("TCP3D Peripheral Configuration\n");
        System_printf("    Instance                     : %s\n", strInst[instNum]);
        System_printf("    Mode Tested                  : %s\n", strMode[codeBlockSet.mode]);
        System_printf("    Double Buffer Mode           : %s\n", strDBuf[codeBlockSet.doubleBuffer]);
        System_printf("Max code blocks (Input Capacity) : %d\n", testMaxCodeBlocks);
        System_printf("Code blocks sent for decoding    : %d\n", codeBlockSet.maxNumCB);
        System_printf("Call back counters               : %d - interrupts\n", pauseIntr);
        System_printf("                          (%d-SOLDOUT, %d-PAUSE, %d-PENDPAUSE)\n", afterIntrSoldout, afterIntrPause, pendPauseCntr);
        System_printf("Total Notification Interrupts    : %d\n", tcp3dEventCntr);
        System_printf("Throughput Calculations\n");
        System_printf("    Total Bits Decoded           : %d\n", TotalBitsDecoded);
        System_printf("    Time Taken (in cycles)       : %d\n", test_cycles);
        System_printf("    Effective Throughput         : %f Mbps\n", ThroughPut);
        System_printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

        System_printf("\n******************************************************************\n");

        /* Increment the test counter */
        testCntr++;

        /**
         * Delete the semaphores each time, so that there is no left over count.
         * See the explanation at the beginning of this loop where the create
         * semaphore calls are present. 
         */
#if DEBUG_VARS
        semCnt = Semaphore_getCount(semRcvStart);
#endif
        Semaphore_delete(&semSendWait);
        Semaphore_delete(&semSendBlock);
        Semaphore_delete(&semRcvStart);
    }

    /* All test status print */
    if(testErrCntr)
    {
        System_printf("!!! SOME TESTS FAILED !!!\n");
    }
    else
    {
        System_printf("!!! ALL TESTS PASSED !!!\n");
    }

    /* Save profile data to file */ 
    PROF_LOG_COMPLETE(instNum);

    /* Remove all creations - to make graceful system exit */
    Semaphore_delete(&semRcvDone);

    System_exit(0);
}
   
/*******************************************************************************
 ******************************************************************************/
Void sndBlockTaskFunc(Void)
{
    UInt8               notifyFlag;
    cbDataDesc          *cbPtr;
    static UInt32       cookie = 0;

    sendBlockCnt = 0;

#if DEBUG_VARS
        semCnt = Semaphore_getCount(semSendBlock);
#endif

    total_clock_start = TSCL;
    LOG_TIME(PROF_SEND_TASK, PROF_START, TSCL); 

    while(1)
    {
        /* Pending on Semaphore to run the loop */
        Semaphore_pend(semSendBlock, BIOS_WAIT_FOREVER);

        /* set TCP3D instance to use */
        inst = tcp3dDrvInst[instNum];

        /* Get pointer to the code block data structure */
        cbPtr = codeBlockSet.cbData[sendBlockCnt];

        /* Interrupt flag, used in Tcp3d_enqueueCodeBlock function */
        notifyFlag = 0;
        if ( sendBlockCnt >= (codeBlockSet.maxNumCB-2) )
        {
            notifyFlag = 1; /* Set for the last CB in each path (PING & PONG) */
        }

        /**
         * Prepare input configuration (IC) registers.
         */
        if ( TCP3D_DRV_INPUT_LIST_FULL == tcp3dResultSend )
        {
            /* IC prepare not required. Just clear soldout actions. */
            soldOutActionClear();
        }
        else
        {
            LOG_TIME(PROF_PREPROC, PROF_START, TSCL);

            /* Prepare Input Config Registers */
#if TEST_PREPARE_ONLY_BETASTATE
            /* Prepare only beta state registers */
            prepareBetaStateICParams(cbPtr, cbPtr->mode);
#elif TEST_PREPARE_ONLY_CODEBLOCK_PARAM_DEPENDENT 
            /* Prepare only registers depend on code block configuration
            * (tempICRegs has the fixed registers prepare outside send loop) */
            prepareIC(cbPtr, tempICRegs, 1);
#else
            /* Prepare all registers */
            prepareIC(cbPtr, NULL, NULL);
#endif

            LOG_TIME(PROF_PREPROC, PROF_STOP, TSCL);
        }

        checkBetaValues (cbPtr->inCfg);

        /* Disabling the global interrupts */
        cookie = Hwi_disable();

        tcp3dEndMemAccess(cbPtr->inCfg, cbPtr->sizeCFG);
        tcp3dEndMemAccess(cbPtr->inLLR, cbPtr->sizeLLR);

        tcp3dBeginMemAccess(cbPtr->outHD, cbPtr->sizeHD);
        if (cbPtr->sdFlag)
            tcp3dBeginMemAccess(cbPtr->outSD, cbPtr->sizeSD);
        if (cbPtr->stsFlag)
            tcp3dBeginMemAccess(cbPtr->outSts, cbPtr->sizeSTS);

        /* Restore interrupts */
        Hwi_restore(cookie);

        /**
         * WORKAROUND CODE:
         * This code works in line with the code in the second while loop
         * in the send task where check for completion is done.
         * Here we are setting the last byte in the outHD with some value when
         * the refHD has 0x00. This avoids any false completion of send task.
         */
        if ( sendBlockCnt >= (codeBlockSet.maxNumCB-2) )
        {
            /* Fill the last byte in outHD when refHD last byte is ZERO */
            uint8_t     *bytePtr1, *bytePtr2;
            uint32_t    byteSize;

            bytePtr1 = (UInt8 *) cbPtr->refHD;
            bytePtr2 = (UInt8 *) cbPtr->outHD;
            byteSize = (cbPtr->blockSize>>3);

            if ( bytePtr1[byteSize-1] == 0 )
            {
                bytePtr2[byteSize-1] = 0xde;
            }
        }

        LOG_TIME(PROF_ENQUE_FUNC, PROF_START, TSCL); 

        /* Enqueue the Code block */
        tcp3dResultSend = Tcp3d_enqueueCodeBlock ( inst,
                                                    cbPtr->blockSize,
                                                    (UInt32 *)L2GLBMAP(dspCoreID, cbPtr->inCfg),
                                                    (Int8 *)L2GLBMAP(dspCoreID, cbPtr->inLLR),
                                                    cbPtr->llrOffset,
                                                    (UInt32 *)L2GLBMAP(dspCoreID, cbPtr->outHD),
                                                    (Int8 *)L2GLBMAP(dspCoreID, cbPtr->outSD), 
                                                    cbPtr->sdOffset,
                                                    (UInt32 *)L2GLBMAP(dspCoreID, cbPtr->outSts),
                                                    notifyFlag); // 1 - GEN EVENT, 0 - NO EVENT

        LOG_TIME(PROF_ENQUE_FUNC, PROF_STOP, TSCL); 

        /* Check for soldout case */
        if ( TCP3D_DRV_INPUT_LIST_FULL != tcp3dResultSend )
        {
            /* increment the block count */
            sendBlockCnt++;

            /* goto next block */
            Semaphore_post(semSendBlock);
        }
        else
        {
            /* increment soldout count */
            soldoutCntr++;

            soldOutAction(); /* take action */
        }

        /* Start the driver after START_CMD_PERIOD blocks */
        if ( sendBlockCnt == START_CMD_PERIOD )
        {
            LOG_TIME(PROF_START_FUNC, PROF_START, TSCL);
            if ( TCP3D_DRV_NO_ERR != Tcp3d_start(inst, TCP3D_DRV_START_AUTO) )
            {
                System_printf("Tcp3d_start function returned error (AUTO)\n");
                System_exit(0);
            }
            LOG_TIME(PROF_START_FUNC, PROF_STOP, TSCL);
        }

        /* Check for end of task and exit */
        if ( sendBlockCnt >= codeBlockSet.maxNumCB )
        {
            /* set flags first */
            pauseIntFlag = 0;
            l2pIntr = pauseIntr;
            pendPauseCntr = 0;

#if SOLDOUT_USE_L2P_INTERRUPT
            if ( soldoutCntr )
            {
                /* Call TCP3D driver control to set interrupt on L2P channel */
                drvCtrl.cmd = TCP3D_DRV_SET_L2P_INT;
                drvCtrl.intrFlag = TEST_INTR_DISABLE;   // disable
                Tcp3d_control(inst, &drvCtrl);
            }
#endif

            /* Set interrupt flag PAUSE channel */
            drvCtrl.cmd = TCP3D_DRV_CLR_REVT_INT;
            Tcp3d_control(inst, &drvCtrl);

            /* Check to see if restart needed before exit */
            LOG_TIME(PROF_START_FUNC, PROF_START, TSCL);
            if ( TCP3D_DRV_NO_ERR != Tcp3d_start(inst, TCP3D_DRV_START_AUTO) )
            {
                System_printf("Tcp3d_start function returned error (AUTO)\n");
                System_exit(0);
            }
            LOG_TIME(PROF_START_FUNC, PROF_STOP, TSCL);
            
            /* Set interrupt flag PAUSE channel */
            drvCtrl.cmd = TCP3D_DRV_SET_REVT_INT;
            drvCtrl.intrFlag = TEST_INTR_ENABLE;   // enable
            Tcp3d_control(inst, &drvCtrl);

            /* out of enqueue loop */
            break;
        }
    } /* end of - while(1) */

#if DEBUG_VARS
    semCnt = Semaphore_getCount(semSendWait);
#endif

    /**
     * Check for pending Pauses and waiting for the last block to be decoded 
     */
    while ( 1 )
    {
        /* Pending on Semaphore to run the loop */
        Semaphore_pend(semSendWait, BIOS_WAIT_FOREVER);

        /* Received both the completion events, so exit send task */
        if ( tcp3dEventCntr >= 2 )
        {
            break;
        } else if ( tcp3dEventCntr == 1 )
        {
            /* one code block test case */
            if ( codeBlockSet.maxNumCB == 1 )
            {
                break;
            }
            else if ( codeBlockSet.mode == TEST_MODE_SPLIT )
            { /* missing one notificatin event - possible in split mode */
                /**
                * WORKAROUND CODE:
                * This is possibility in case of SPLIT mode, that one event is
                * lost when both ping and pong channels try to generate system
                * events at close proximity.
                * In this test bench we have enabled notification events for
                * the last two blocks, so checking the outHD & refHD last bytes
                * to confirm the decoding of these blocks are completed.
                */
                
                /* cbPtr for last two code blocks */
                cbDataDesc  *cbPtr1 = codeBlockSet.cbData[codeBlockSet.maxNumCB-2];
                cbDataDesc  *cbPtr2 = codeBlockSet.cbData[codeBlockSet.maxNumCB-1];
                uint8_t     *bytePtr11, *bytePtr12, *bytePtr21, *bytePtr22;
                uint32_t    size1, size2;

                bytePtr11 = (UInt8 *) cbPtr1->refHD;
                bytePtr12 = (UInt8 *) cbPtr1->outHD;
                bytePtr21 = (UInt8 *) cbPtr2->refHD;
                bytePtr22 = (UInt8 *) cbPtr2->outHD;
                size1 = (cbPtr1->blockSize>>3); /* in bytes */
                size2 = (cbPtr2->blockSize>>3); /* in bytes */

                /* check if last HD byte of last two blocks are completed */
                if ((bytePtr11[size1-1] == bytePtr12[size1-1]) &&
                    (bytePtr21[size2-1] == bytePtr22[size2-1]) ) 
                {
                    System_printf("Notification event missed (Race Condition)\n");
                    System_printf("Since the last two block decoding completed, completing send task\n");
                    System_printf("Block : %d\n", codeBlockSet.maxNumCB-2);
                    System_printf("\trefHD[%d] = 0x%x\t outHD[%d] = 0x%x\n", size1-1, bytePtr11[size1-1], size1-1, bytePtr12[size1-1]);
                    System_printf("Block : %d\n", codeBlockSet.maxNumCB-1);
                    System_printf("\trefHD[%d] = 0x%x\t outHD[%d] = 0x%x\n", size2-1, bytePtr21[size2-1], size2-1, bytePtr22[size2-1]);
                    break;
                }
            }
        }

        LOG_TIME(PROF_START_FUNC, PROF_START, TSCL);
        if ( TCP3D_DRV_NO_ERR != Tcp3d_start(inst, TCP3D_DRV_START_AUTO) )
        {
            System_printf("Tcp3d_start function returned error\n");
            System_exit(0);
        }
        LOG_TIME(PROF_START_FUNC, PROF_STOP, TSCL);

        /* keep trying until finding two end events */
        Semaphore_post(semSendWait);

        pendPauseCntr++;
    }
    LOG_TIME(PROF_SEND_TASK, PROF_STOP, TSCL);

    /* Last code block decoded - Start the receive task */
    total_clock_end = TSCL;
    LOG_TIME_ISR(PROF_POST_RECV_PING, PROF_INST, TSCL); 
    Semaphore_post(semRcvStart);
}

/*******************************************************************************
 ******************************************************************************/
Void rcvBlockTaskFunc(Void)
{
    Int32           errCnt;

    cbDataDesc       *cbPtr;
    Int             idx, loopCnt;
    Int             fail = 0;
    UInt8           *ptr1, *ptr2;
   
    rcvBlockCnt = 0;
    totErrCnt = 0;

    while(1)
    {
        Semaphore_pend(semRcvStart, BIOS_WAIT_FOREVER);

        /* prints for send task are done here */
        if ( tcp3dResultSend == TCP3D_DRV_NO_ERR )
        {
#if DEBUG_PRINT
            for ( loopCnt = 0; loopCnt < sendBlockCnt; loopCnt++ )
            {
                cbPtr   = codeBlockSet.cbData[loopCnt];
                System_printf("Send Task: Enqueued Block %d (Size: %d, SW0: %d)\n",
                                    loopCnt, cbPtr->blockSize,
                                    cbPtr->sw0LengthUsed);
            }
#endif
            System_printf("Send Task: Enqueued %d Blocks\n\n", sendBlockCnt);
        }
        else
        {
            System_printf("Send Task: Enqueued Blocks failed (tcp3dResultSend : %d)\n\n", tcp3dResultSend);
            System_exit(0);
        }

        MY_LOG_PRINT0(Rcv Task: SEM RECEIVED);

        LOG_TIME(PROF_RECV_TASK, PROF_START, TSCL); 

        while( rcvBlockCnt < codeBlockSet.maxNumCB )
        {
            /* Get the pointer to the Code Block Set */
            cbPtr   = codeBlockSet.cbData[rcvBlockCnt];
    
            /* Step 2: Verify All the outputs */
            fail = 0;
            /* Step 2.1: Hard Decisions Verification */
            ptr1 = (UInt8 *) cbPtr->refHD;
            ptr2 = (UInt8 *) cbPtr->outHD;

            /* Invalidate out HD */
            CACHE_invL1d (cbPtr->outHD, cbPtr->blockSize>>3, CACHE_WAIT);

            errCnt = 0;    
            for (idx = 0; idx < (cbPtr->blockSize>>3); ++idx)
            {
                if ( ptr1[idx] != ptr2[idx] )
                {
                    errCnt++;
                    System_printf("\tBlock Count %d, HD mismatch byte %d\n", rcvBlockCnt, idx);
                }
            }

            if (errCnt)
            {
                MY_LOG_PRINT2(Rcv task: HD FAILED, rcvBlockCnt, ERRORS:, errCnt);
                fail++;
            }
            else
            {
                MY_LOG_PRINT1(Rcv task: HD PASSED, rcvBlockCnt);
            }
    
            /* Step 2.2: Soft Decisions Verification */
            if (cbPtr->sdFlag)
            {
                if ( codeBlockSet.mode == TEST_MODE_SPLIT ) /* SPLIT MODE */
                    loopCnt = cbPtr->blockSize;
                else
                    loopCnt = (3*cbPtr->blockSize);
   
                /* Invalidate out SD */
                CACHE_invL1d (cbPtr->outSD, loopCnt, CACHE_WAIT);
 
                /* NOTE: Assumed that the Soft Decisions are in a single array */
                errCnt = 0;
                for (idx = 0; idx < loopCnt; ++idx)
                {
                    if ( cbPtr->refSD[idx] != cbPtr->outSD[idx] )
                    {
                        errCnt += 1;
                        System_printf("\tBlock Count %d, SD mismatch byte %d\n", rcvBlockCnt, idx);
                    }
                }
    
                if (errCnt)
                {
                    MY_LOG_PRINT2(Rcv task: SD FAILED, rcvBlockCnt, ERRORS:, errCnt);
                    fail++;
                }
                else
                {
                    MY_LOG_PRINT1(Rcv task: SD PASSED, rcvBlockCnt);
                }
            } /* if (cbPtr->sdFlag) */
    
            /* Step 2.3: errCnt Registers Verification */
            if (cbPtr->stsFlag)
            {
                /* Invalidate out Sts */
                CACHE_invL1d (cbPtr->outSts, 12, CACHE_WAIT);

                errCnt = 0;
                for (idx = 0; idx < 3; ++idx)
                {
                    if ( cbPtr->refSts[idx] != cbPtr->outSts[idx] )
                    {
                        errCnt += 1;
                        System_printf("\tBlock Count %d, STS mismatch word %d\n", rcvBlockCnt, idx);
                    }
                }
                if (errCnt)
                {
                    MY_LOG_PRINT2(Rcv task: STS FAILED, rcvBlockCnt, ERRORS:, errCnt);
                    fail++;
                }
                else
                {
                    MY_LOG_PRINT1(Rcv task: STS PASSED, rcvBlockCnt);
                }
            } /* if (cbPtr->stsFlag) */
            if (fail)
            {
                System_printf("Rcv task: Block %d FAILED\n", rcvBlockCnt);
                totErrCnt++;
            }
#if DEBUG_PRINT
            else
            {
                System_printf("Rcv task: Block %d PASSED\n", rcvBlockCnt);
            }
#endif
            rcvBlockCnt++;
        }
        if(rcvBlockCnt >= codeBlockSet.maxNumCB)
        {
            break;
        }   
    }

    LOG_TIME(PROF_RECV_TASK, PROF_STOP, TSCL); 

    System_printf("Rcv Task: COMPLETE - verified %d blocks\n", rcvBlockCnt);
    
    /* Prepare for next test, set by "tester task" */
    Semaphore_post(semRcvDone);
}

/*******************************************************************************
 ******************************************************************************/
Void revt0ChCallback(Void)
{
    /* Increment the ISR counter */
    pauseIntr++;
    
    pauseIntFlag = 1;

    LOG_TIME_ISR(PROF_RESTART_PING, PROF_INST, TSCL);

    if ( sendBlockCnt >= codeBlockSet.maxNumCB )
        Semaphore_post(semSendWait);
    else
        Semaphore_post(semSendBlock);
}

/*******************************************************************************
 ******************************************************************************/
Void revt1ChCallback(Void)
{
    /* Increment the ISR counter */
    pauseIntr++;
    
    pauseIntFlag = 2;

    LOG_TIME_ISR(PROF_RESTART_PONG, PROF_INST, TSCL);

    if ( sendBlockCnt >= codeBlockSet.maxNumCB )
        Semaphore_post(semSendWait);
    else
        Semaphore_post(semSendBlock);
}

/*******************************************************************************
 ******************************************************************************/
Void tskHeartBeat(Void)
{
    static unsigned int counter = 0u;

    while (counter < 0x1000000u)
    {
        Task_sleep (1000u);
        System_printf("\n!!! EDMA3 LLD HrtBt %x\n", counter);
        counter++;
    }
}

/*******************************************************************************
 ******************************************************************************/
Void getMemoryStats(Void)
{
    Memory_Stats        memStats;

    Memory_getStats(drvHeap, &memStats);
    System_printf("\nHeap Usage/Status\n");
    System_printf("    tcp3dDrvHeap : %d of %d free\n", memStats.totalFreeSize, memStats.totalSize);

    Memory_getStats(dataHeap, &memStats);
    System_printf("    tcp3dDataHeap : %d of %d free\n", memStats.totalFreeSize, memStats.totalSize);
}

/*******************************************************************************
 ******************************************************************************/
Void tcp3dEventISR(UInt32 testEvtNum)
{
    tcp3dEventCntr++;
    tpccEvtCntr++;

    LOG_TIME_ISR(PROF_RESTART_PING, PROF_INST, TSCL);

    if ( sendBlockCnt >= codeBlockSet.maxNumCB )
        Semaphore_post(semSendWait);
    else
        Semaphore_post(semSendBlock);
}

/*******************************************************************************
 ******************************************************************************/
Void registerTcp3dEvent(Void)
{
    static  UInt32 cookie = 0;
    Int     eventId = 0;    /* GEM event id */
    static  UInt32 mapDone = 0;
    UInt32  testEvt = getNotifyEventNum(instNum);
    UInt32  hostIntr = getHostIntrNum(dspCoreID);

    /* Disabling the global interrupts */
    cookie = Hwi_disable();

    /* Completion ISR Registration */
    CpIntc_dispatchPlug(testEvt, tcp3dEventISR, hostIntr, TRUE);
    if (!mapDone)
        CpIntc_mapSysIntToHostInt(0, testEvt, hostIntr);
    CpIntc_enableHostInt(0, hostIntr);
    eventId = CpIntc_getEventId(hostIntr);
    EventCombiner_dispatchPlug (eventId,
#if USE_LOCAL_CPINTC_DISPATCH
                                CpIntc_dispatchLoc,
#else
                                CpIntc_dispatch,
#endif
                                hostIntr,
                                TRUE);
#if DEBUG_PRINT
    System_printf("\t\t testEvt : %d \n", testEvt);
    System_printf("\t\t hostIntr : %d \n", hostIntr);
    System_printf("\t\t eventId : %d \n", eventId);
#endif

    /* enable the 'global' switch */
    CpIntc_enableAllHostInts(0);

    mapDone = 1;

    /* Restore interrupts */
    Hwi_restore(cookie);
}

/*******************************************************************************
 ******************************************************************************/
Void unregisterTcp3dEvent(Void)
{
    static UInt32 cookie = 0;
    Int eventId = 0;    /* GEM event id */
    UInt32  hostIntr = getHostIntrNum(dspCoreID);
    
    /* Disabling the global interrupts */
    cookie = Hwi_disable();

    /* Driver Completion ISR */
    CpIntc_disableHostInt(0, hostIntr);
    eventId = CpIntc_getEventId(hostIntr);
    EventCombiner_disableEvent(eventId);

    /* Restore interrupts */
    Hwi_restore(cookie);
}

/*******************************************************************************
 ******************************************************************************/
Void allInit(Void)
{
    Tcp3d_Result        tcp3dResult = TCP3D_DRV_NO_ERR;
    EDMA3_DRV_Result    edmaResult = EDMA3_DRV_SOK;

#if TEST_PROFILE_LOG
    CSL_Tcp3d_cfgRegs   *tcp3dCfgRegs = (CSL_Tcp3d_cfgRegs *) getTcp3dCfgRegsBase(instNum);
#endif

    /* Initialize EDMA3 first */
    hEdma = NULL;
    tpccNum = 2;
    tpccRegionUsed = 3;
    hEdma = edma3init ( tpccNum,
                        &edmaResult,
                        dspCoreID,
                        tpccRegionUsed);
    if (edmaResult != EDMA3_DRV_SOK)
    {
        System_printf("edma3init() FAILED, error code: %d\n", edmaResult);
    }
    else
    {
        System_printf("EDMA3 LLD Initialization complete (TPCC #%d, Region #%d)\n", tpccNum, tpccRegionUsed);
    }

    /* Allocate all EDMA channels required for TCP3D Driver */
    System_printf("EDMA3 Channels opening started...\n");

    /* Open channels for one instance */
    openEdmaChannels (hEdma, instNum, &edmaConfig[instNum]);

    /* Register call backs */
    EDMA3_DRV_registerTccCb(hEdma, edmaConfig[instNum].pingChRes[0].chNo, (EDMA3_RM_TccCallback)&revt0ChCallback, NULL);
    EDMA3_DRV_registerTccCb(hEdma, edmaConfig[instNum].pongChRes[0].chNo, (EDMA3_RM_TccCallback)&revt1ChCallback, NULL);

#if EDMA_LOCAL_COMP_ISR // flag defined in sample.h file
    /* Fill call back details */
    edmaConfig[instNum].pingChRes[0].cbFunc  = (EDMA3_RM_TccCallback)&revt0ChCallback;
    edmaConfig[instNum].pingChRes[0].cbData  = NULL;
    edmaConfig[instNum].pongChRes[0].cbFunc  = (EDMA3_RM_TccCallback)&revt1ChCallback;
    edmaConfig[instNum].pongChRes[0].cbData  = NULL;

    /**
     * Update the information to use with local EDMA ISR 
     * (NOTE: This function must be called after the channels are opened)
     */
    updateAllocatedTccsLoc(&edmaConfig[instNum]);
#endif

    System_printf("EDMA3 Channels opening complete\n");

    System_printf("TCP3 Decoder Driver Initialization sequence started...\n");

#if TEST_PROFILE_LOG
        /***** Soft Reset the TCP3D for synchronization ******/
        LOG_TIME(PROF_SOFT_RESET, PROF_INST, TSCL); 
        tcp3dCfgRegs->TCP3_SOFT_RESET = 1;
        tcp3dCfgRegs->TCP3_SOFT_RESET = 0;
#endif

    LOG_TIME(PROF_INIT_FUNC, PROF_START, TSCL); 

    /* Initialize the TCP3D first */
    tcp3dDrvInst[instNum] = tcp3dSampleInit (drvHeap,
                                    instNum,
                                    testMaxCodeBlocks,
                                    codeBlockSet.mode,
                                    codeBlockSet.doubleBuffer,
                                    codeBlockSet.lteCrcSel,
                                    dspCoreID,
                                    hEdma,
                                    tpccRegionUsed,
                                    &edmaConfig[instNum],
                                    &tcp3dResult);

    LOG_TIME(PROF_INIT_FUNC, PROF_STOP, TSCL); 

    System_printf("TCP3 Decoder Driver Initialization sequence complete\n");

    /* Register the Notification Event for TCP3D */
    registerTcp3dEvent();

    /* Set the global flags to default values */
    pingComplete = 0;
    pongComplete = 0;
    pauseIntr = 0;
    l2pIntr = 0;
    tcp3dEventCntr = 0;
    pauseIntFlag = 0;
    rcvStartFlag = 0;
    soldoutCntr = 0;
    afterIntrSoldout = 0;
    afterIntrPause = 0;
    pendPauseCntr = 0;
}

/*******************************************************************************
 ******************************************************************************/
Void allDeInit(Void)
{
    EDMA3_DRV_Result    edmaResult = EDMA3_DRV_SOK;

    /* Un-register the Notification Event for TCP3D */
    unregisterTcp3dEvent();

    /* Close all EDMA channels allocated for the test */
    System_printf("EDMA3 Channels freeing started...\n");

    /* Register call backs */
    EDMA3_DRV_unregisterTccCb(hEdma, edmaConfig[instNum].pingChRes[0].chNo);
    EDMA3_DRV_unregisterTccCb(hEdma, edmaConfig[instNum].pingChRes[1].chNo);
    EDMA3_DRV_unregisterTccCb(hEdma, edmaConfig[instNum].pongChRes[0].chNo);
    EDMA3_DRV_unregisterTccCb(hEdma, edmaConfig[instNum].pongChRes[1].chNo);

    /* Close channels */
    closeEdmaChannels(hEdma, instNum, &edmaConfig[instNum]);

    System_printf("EDMA3 Channels freeing complete\n");

    /* Deinit for TCP3D driver */
    System_printf("TCP3 Decoder Driver De-Initialization sequence started...\n");

    tcp3dSampleDeinit(drvHeap, instNum, tcp3dDrvInst[instNum]);

    System_printf("TCP3 Decoder Driver De-Initialization sequence complete\n");

    /* De-init EDMA3 */
    edmaResult = edma3deinit(tpccNum, hEdma);
    if (edmaResult != EDMA3_DRV_SOK)
    {
        System_printf("edma3deinit() FAILED, error code: %d\n", edmaResult);
    }
    else
    {
        System_printf("EDMA3 LLD De-Initialization complete\n");
    }
}

#ifndef SIMULATOR_SUPPORT
/**
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for the 
 *      specified power domain and module. 
 *
 *  @retval
 *      0 for enabling power domain, -1 for failure
 */
static Int32 enable_ip_instance (UInt32 pwrDmnNum, UInt32 moduleNum)
{
    /* Power domains are turned OFF by default for many IP blocks. It
     * needs to be turned on before doing any device
     * register access.
     */
    /* Set IP block Power domain to ON */        
    CSL_PSC_enablePowerDomain (pwrDmnNum);

    /* Enable the clocks too for the IP block */
    CSL_PSC_setModuleNextState (moduleNum, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (pwrDmnNum);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (pwrDmnNum));

    /* Return IP block PSC status */
    if ((CSL_PSC_getPowerDomainState(pwrDmnNum) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (moduleNum) == PSC_MODSTATE_ENABLE))
    {
        /* IP block ON. Ready for use */            
        return 0;
    }
    else
    {
        /* IP block power on failed. Return error */            
        return -1;            
    }
}
#endif

/**
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for TCP3D. 
 *
 *  @retval
 *      Not Applicable.
 */
static Int32 enable_tcp3d (void)
{
#ifndef SIMULATOR_SUPPORT
    Int32  result;
    UInt32 coreID = CSL_chipReadDNUM();
    
    /* TCP3D power domain is turned OFF by default.
     * It needs to be turned on before doing any TCP3D device register access.
     * This is not required for the simulator. 
     */
#if (CSL_TCP3D_PER_CNT > 1)

    if (coreID == 0)
    {
        result = enable_ip_instance (TEST_CSL_PSC_PD_TCP3D_0, 
                                     TEST_CSL_PSC_LPSC_TCP3D_0);
    }
    else 
    {
        result = enable_ip_instance (TEST_CSL_PSC_PD_TCP3D_1, 
                                     TEST_CSL_PSC_LPSC_TCP3D_1);
    }    
#else
    
    result = enable_ip_instance (TEST_CSL_PSC_PD_TCP3D_0, 
                                 TEST_CSL_PSC_LPSC_TCP3D_0);
                                 
#endif /* CSL_TCP3D_PER_CNT > 1 */

    return (result);    
#else
    /* PSC is not supported on simulator. Return success always */
    return 0;
#endif
}

/**
 *  @b Description
 *  @n  
 *      The function is used to indicate that a block of memory is 
 *      about to be accessed. If the memory block is cached then this 
 *      indicates that the application would need to ensure that the 
 *      cache is updated with the data from the actual memory.
 *
 *  @param[in]  ptr
 *       Address of memory block
 *
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void tcp3dBeginMemAccess (void *ptr, uint32_t size)
{
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled */    
    CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);

    /*  Cleanup the prefectch buffer also. */
    CSL_XMC_invalidatePrefetchBuffer();    

    return;
}

/**
 *  @b Description
 *  @n  
 *      The function is used to indicate that the block of memory has 
 *      finished being accessed. If the memory block is cached then the 
 *      application would need to ensure that the contents of the cache 
 *      are updated immediately to the actual memory. 
 *
 *  @param[in]  ptr
 *       Address of memory block
 *  @param[in]  size
 *       Size of memory block
 *
 *  @retval
 *      Not Applicable
 */
void tcp3dEndMemAccess (void *ptr, uint32_t size)
{
    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled */    
    CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);        

    return;
}

/* end of file */
