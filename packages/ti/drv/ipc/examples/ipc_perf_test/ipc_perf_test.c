/*
 *  Copyright (c) Texas Instruments Incorporated 2019
 *  All rights reserved.
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
 */

/**
 *  \file ipc_perf_test.c
 *
 *  \brief IPC  echo sample application performing basic echo communication using
 *  IPC driver
 *  
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>

#include <ti/board/board.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

#include <ti/drv/ipc/ipc.h>
#include <ti/osal/osal.h>
#include <ti/osal/TaskP.h>
#include <ti/drv/ipc/include/ipc_config.h>
#include <ti/drv/ipc/src/ipc_osal.h>

#include "ipc_perf_test.h"
#include "ipc_test_defs.h"
#include "ipc_apputils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENDPT1                  13U
#define IPC_RPMESSAGE_OBJ_SIZE  256U
#define RPMSG_DATA_SIZE         (256U*512U + IPC_RPMESSAGE_OBJ_SIZE)
#define VQ_BUF_SIZE             2048U
#define MSGSIZE                 256U
#define SERVICE                 "ti.ipc4.ping-pong"

#ifdef SOC_AM65XX
#define VRING_BASE_ADDRESS      0xA2000000U
#else
#define VRING_BASE_ADDRESS      0xAA000000U
#endif

#define PRINT_HOST_CORE         (IPC_MPU1_0)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static uint32_t remoteProc[] = {
#if defined (SOC_AM65XX)
#if defined(BUILD_MPU1_0)
    IPC_MCU1_0, IPC_MCU1_1
#elif defined(BUILD_MCU1_0)
    IPC_MPU1_0, IPC_MCU1_1
#elif defined(BUILD_MCU1_1)
    IPC_MPU1_0, IPC_MCU1_0
#endif
#elif defined(SOC_J721E)
#if defined(BUILD_MPU1_0)
    IPC_MCU1_0, IPC_MCU1_1, IPC_MCU2_0, IPC_MCU2_1,
    IPC_MCU3_0, IPC_MCU3_1, IPC_C66X_1, IPC_C66X_2,
    IPC_C7X_1
#elif defined(BUILD_MCU1_0)
    IPC_MPU1_0, IPC_MCU1_1, IPC_MCU2_0, IPC_MCU2_1,
    IPC_MCU3_0, IPC_MCU3_1, IPC_C66X_1, IPC_C66X_2,
    IPC_C7X_1
#elif defined(BUILD_MCU1_1)
    IPC_MPU1_0, IPC_MCU1_0, IPC_MCU2_0, IPC_MCU2_1,
    IPC_MCU3_0, IPC_MCU3_1, IPC_C66X_1, IPC_C66X_2,
    IPC_C7X_1
#elif defined(BUILD_MCU2_0)
    IPC_MPU1_0, IPC_MCU1_0, IPC_MCU1_1, IPC_MCU2_1,
    IPC_MCU3_0, IPC_MCU3_1, IPC_C66X_1, IPC_C66X_2,
    IPC_C7X_1
#elif defined(BUILD_MCU2_1)
    IPC_MPU1_0, IPC_MCU1_0, IPC_MCU1_1, IPC_MCU2_0,
    IPC_MCU3_0, IPC_MCU3_1, IPC_C66X_1, IPC_C66X_2,
    IPC_C7X_1
#elif defined(BUILD_MCU3_0)
    IPC_MPU1_0, IPC_MCU1_0, IPC_MCU1_1, IPC_MCU2_0,
    IPC_MCU2_1, IPC_MCU3_1, IPC_C66X_1, IPC_C66X_2,
    IPC_C7X_1
#elif defined(BUILD_MCU3_1)
    IPC_MPU1_0, IPC_MCU1_0, IPC_MCU1_1, IPC_MCU2_0,
    IPC_MCU2_1, IPC_MCU3_0, IPC_C66X_1, IPC_C66X_2,
    IPC_C7X_1
#elif defined(BUILD_C66X_1)
    IPC_MPU1_0, IPC_MCU1_0, IPC_MCU1_1, IPC_MCU2_0,
    IPC_MCU2_1, IPC_MCU3_0, IPC_MCU3_1, IPC_C66X_2,
    IPC_C7X_1
#elif defined(BUILD_C66X_2)
    IPC_MPU1_0, IPC_MCU1_0, IPC_MCU1_1, IPC_MCU2_0,
    IPC_MCU2_1, IPC_MCU3_0, IPC_MCU3_1, IPC_C66X_1,
    IPC_C7X_1
#elif defined(BUILD_C7X_1)
    IPC_MPU1_0, IPC_MCU1_0, IPC_MCU1_1, IPC_MCU2_0,
    IPC_MCU2_1, IPC_MCU3_0, IPC_MCU3_1, IPC_C66X_1,
    IPC_C66X_2
#endif
#endif
};

static uint32_t curTestIndex = 0U;

uint8_t recvStack[IPC_TASK_STACKSIZE] __attribute__ ((section("ipc_data_buffer"), aligned(8192)));
uint8_t ctrlStack[IPC_TASK_STACKSIZE] __attribute__ ((section("ipc_data_buffer"), aligned(8192)));
uint8_t ctrlBuf[RPMSG_DATA_SIZE]      __attribute__ ((section("ipc_data_buffer"), aligned (8)));
uint8_t sysVqBuf[VQ_BUF_SIZE]         __attribute__ ((section("ipc_data_buffer"), aligned (8)));
uint8_t rpmsgBuf[RPMSG_DATA_SIZE]     __attribute__ ((section("ipc_data_buffer"), aligned (8)));
uint8_t tstBuf[RPMSG_DATA_SIZE]       __attribute__ ((section("ipc_data_buffer"), aligned (8)));
uint32_t testCompleted  = FALSE;
uint32_t gPrintHostCore = FALSE;

SemaphoreP_Handle   semHandle;


int32_t Ipc_perf_test(void)
{
    RPMessage_Handle    handle;
    uint32_t	        myEndPt    = ENDPT1;
    TaskP_Handle        tskHandle  = NULL;
    int32_t		        status     = IPC_SOK;
    uint32_t            selfId     = Ipc_getCoreId();
    uint32_t            curHost    = FALSE;
    Ipc_Testcase*       tstcase    = NULL;

    /* Setup the IPC framework */
    Ipc_perf_test_setup();

    /* Create RPMSG object */
    handle = Ipc_createRpmsg(rpmsgBuf, RPMSG_DATA_SIZE, &myEndPt);
    if(NULL != handle)
    {
        status = RPMessage_announce(RPMESSAGE_ALL, myEndPt, SERVICE);
    }

    /* MCU1_0 needs to kickstart the test */
    if(IPC_MCU1_0 == selfId)
    {
        curTestIndex = 0;
        tstcase = Ipc_getTestcase(curTestIndex);
        Ipc_sendNewTestIndex(handle, myEndPt, curTestIndex, tstcase->hostCore);
    }

    if(IPC_SOK == status)
    {
        tskHandle = Ipc_createRcvThread((void*)handle, &myEndPt);

        while(testCompleted != TRUE)
        {
            tstcase = Ipc_getTestcase(curTestIndex);
            if( NULL != tstcase)
            {
                /* If you are not host core or you are 
                 * A72, wait for the message */
                if( (selfId != tstcase->hostCore) ||
                    ((PRINT_HOST_CORE == selfId) && (curTestIndex == 0)))
                {
                    SemaphoreP_pend(semHandle, BIOS_WAIT_FOREVER);
                    if(testCompleted == TRUE )
                    {
                        continue;
                    }

                    curHost = TRUE;
                    tstcase = Ipc_getTestcase(curTestIndex);

                    if((TRUE == gPrintHostCore) && (curTestIndex == 0))
                    {
                        UART_printf("\n\nPerformance Test : Stared\n");
                    }

                    if((TRUE == gPrintHostCore) && (NULL == tstcase))
                    {
                        /* Test completed */
                        Ipc_printTestReport();

                        //testCompleted = TRUE;
                        continue;
                    }
                    else
                    {
                        SystemP_printf("Starting test now ....\n");
                    }

                }

                /* Perform the test now */
                Ipc_runPerfTest(tstcase->slaveCore, tstcase->param1, tstcase->testId);

                /* Increment the TestId and notify the cores */
                curTestIndex++;

                tstcase = Ipc_getTestcase(curTestIndex);
                if(NULL == tstcase)
                {
                    /* Send own test result to print_host_core */
                    Ipc_sendTestResult(handle, myEndPt);

                    /* Notify other cores that test completed */
                    Ipc_sendTestCompletedMsg(handle, myEndPt);
                }
                else if((selfId != tstcase->hostCore) && (TRUE == curHost))
                {
                    Ipc_sendNewTestIndex(handle, myEndPt, curTestIndex, tstcase->hostCore);
                    curHost = FALSE;
                }
            }
        }

        if(tskHandle != NULL)
        {
            TaskP_delete(tskHandle);
        }

        if(TRUE == gPrintHostCore)
        {
            UART_printf("Performance Test : Completed\n");
        }

    }

    return IPC_SOK;
}

void Ipc_recvTaskFxn(uint32_t *arg0, uint32_t *arg1)
{
    RPMessage_Handle    handle;
    uint8_t             buf[MSGSIZE];
    uint32_t		    remoteEndPt;
    uint32_t		    remoteProcId;
    int32_t		        status = IPC_SOK;
    uint16_t            len    = 0;
    uint32_t            myEndPt = 0;

    handle = (RPMessage_Handle)arg0;
    myEndPt = *arg1;

    while( testCompleted != TRUE)
    {
        len = MSGSIZE;
        status = RPMessage_recv(handle, (Ptr)buf, &len, &remoteEndPt, &remoteProcId,
                IPC_RPMESSAGE_TIMEOUT_FOREVER);
        if(IPC_SOK != status)
        {
            testCompleted = TRUE;
            SystemP_printf("Ipc_recvTaskFxn:  recvd fxn failed...exiting\n");
            break;
        }

        if((buf[0] == IPC_PERF_TEST))
        {
            Ipc_processPerfCmd(handle, remoteEndPt, remoteProcId, 
                    myEndPt, buf, len);
        }
    }
}

TaskP_Handle Ipc_createRcvThread(uint32_t *arg0, uint32_t *arg1)
{
    TaskP_Params       params;
    TaskP_Handle       tskHandle  = NULL;

    TaskP_Params_init(&params);
    params.priority   = 3;
    params.stack      = recvStack;
    params.stacksize  = IPC_TASK_STACKSIZE;
    params.arg0       = arg0;
    params.arg1       = arg1;
    tskHandle = TaskP_create(Ipc_recvTaskFxn, &params);
    return (tskHandle);
}

RPMessage_Handle Ipc_createRpmsg(uint8_t *buf, uint32_t bufSize, uint32_t *myEndPt)
{
    RPMessage_Params    params;
    RPMessage_Handle    handle;

    RPMessageParams_init(&params);
    if(*myEndPt != 0)
    {
        params.requestedEndpt = *myEndPt;
    }
    else
    {
        params.numBufs = 2;
    }

    params.buf            = buf;
    params.bufSize        = bufSize;
    handle = RPMessage_create(&params, myEndPt);
    return (handle);   
}

void Ipc_perf_test_setup(void)
{
    SemaphoreP_Params semPrms;
    Ipc_VirtIoParams  vqParam;
    RPMessage_Params  cntrlParam;
    uint32_t          selfId  = Ipc_getCoreId();
    uint32_t          numProc = sizeof(remoteProc)/sizeof(uint32_t);
   
    Ipc_mpSetConfig(selfId, numProc, remoteProc);

    Ipc_init(NULL);

    vqParam.vqObjBaseAddr = (void*)sysVqBuf;
    vqParam.vqBufSize     = numProc * Ipc_getVqObjMemoryRequiredPerCore();
    vqParam.vringBaseAddr = (void*)VRING_BASE_ADDRESS;
    vqParam.vringBufSize  = IPC_VRING_BUFFER_SIZE;
    vqParam.timeoutCnt    = 100;  /* Wait for counts */
    Ipc_initVirtIO(&vqParam);

    RPMessageParams_init(&cntrlParam);

    /* Set memory for HeapMemory for control task */
    cntrlParam.buf         = ctrlBuf;
    cntrlParam.bufSize     = RPMSG_DATA_SIZE;
    cntrlParam.stackBuffer = ctrlStack;
    cntrlParam.stackSize   = IPC_TASK_STACKSIZE;
    RPMessage_init(&cntrlParam);
    
    SemaphoreP_Params_init(&semPrms);
    semHandle = SemaphoreP_create(0U, &semPrms);

    if(PRINT_HOST_CORE == selfId)
    {
        gPrintHostCore = TRUE;
    }
}

void Ipc_runPerfTest(uint32_t coreId, uint32_t numCount, uint32_t testId)
{
    RPMessage_Handle    handle;

    uint32_t            iCnt;
    volatile uint64_t   roundtrip_start, roundtrip_delta;
    uint64_t            frq;
    uint32_t            myEndPt = 0;
    int32_t             status = IPC_SOK;

    uint8_t             buf[512];
    uint32_t            bufSize = 4;
    uint32_t            remoteEndPt;
    uint32_t            remoteProcId;
    uint16_t            len;

    roundtrip_delta = 0U;
    frq = Ipc_getTimestampFrq();

    handle = Ipc_createRpmsg(tstBuf, RPMSG_DATA_SIZE, &myEndPt);
    if(NULL != handle)
    {
        status = RPMessage_getRemoteEndPt(coreId, SERVICE, &remoteProcId,
                &remoteEndPt, BIOS_WAIT_FOREVER);
        if(coreId != remoteProcId) 
        {
            SystemP_printf("Ipc_runPerfTest (remote %d): RPMessage_getRemoteEndPt() failed %d\n",
                    coreId);
            status = IPC_EFAIL;
        }

        for(iCnt = 0U; iCnt < numCount; iCnt++)
        {
            buf[0] = IPC_PERF_TEST;
            
            roundtrip_start = Ipc_getTimeInUsec(frq);

            status = RPMessage_send(handle, coreId, ENDPT1, myEndPt, (Ptr)buf, bufSize);
            if (status != IPC_SOK) 
            {
                SystemP_printf("Ipc_runPerfTest (remote %d): rpmsg_senderFxn: RPMessage_send "
                        " failed status %d\n", coreId, status);
            }
            /* wait a for a response message: */
            len = bufSize;
            status = RPMessage_recv(handle, (Ptr)buf, &len, &remoteEndPt,
                    &remoteProcId, IPC_RPMESSAGE_TIMEOUT_FOREVER);
            if(status != IPC_SOK) 
            {
                SystemP_printf("Ipc_runPerfTest (remote %d): RPMessage_recv failed with code %d\n",
                        coreId, status);
            }
            roundtrip_delta += (Ipc_getTimeInUsec(frq) - roundtrip_start);
        }

        roundtrip_delta = roundtrip_delta/numCount;
        SystemP_printf("Host: %s, Remote: %s, Size: %d, Roundtrip Time: %d us\n", 
               Ipc_mpGetSelfName(), Ipc_mpGetName(coreId), bufSize,
               (uint32_t)roundtrip_delta);

        /* Store test result data */
        Ipc_addPerfTestResult(testId, (uint32_t)roundtrip_delta);
    }

    RPMessage_delete(&handle);
}

void Ipc_sendTestCompletedMsg(RPMessage_Handle handle, uint32_t srcEndPt)
{
    uint16_t   numCores = Ipc_mpGetNumProcessors();
    uint16_t   iCore;
    uint32_t   dstCoreId;

    for(iCore = 0; iCore < numCores; iCore++)
    {
        dstCoreId = Ipc_mpGetRemoteProcId(iCore);
        Ipc_sendTestCompletedMsgCore(handle, srcEndPt, dstCoreId);
    }

}

void Ipc_sendTestCompletedMsgCore(RPMessage_Handle handle, uint32_t srcEndPt, uint32_t dstCoreId)
{
    uint32_t   bufSize;
    uint8_t    buf[64];
    uint32_t   payloadSize = 5;  /* FuncCode */
    int32_t    status = IPC_SOK;
    uint32_t   testIndex = curTestIndex;
    uint32_t   dstEndPt = ENDPT1;

    /*
     * opcode    - 1byte
     * size      - 4bytes
     * FuncCode 
     *
     * */
    bufSize = 10;

    buf[0] = IPC_PERF_TEST; 
    memcpy(&buf[1], (const void*)&payloadSize, sizeof(uint32_t));
    buf[5] = IPC_FC_PERFTEST_COMPLETED;
    memcpy(&buf[6], (const void*)&testIndex, sizeof(uint32_t));

    status = RPMessage_send(handle, dstCoreId, dstEndPt, srcEndPt, (Ptr)buf, bufSize);
    if (status != IPC_SOK) 
    {
        SystemP_printf("Ipc_sendTestCompletedMsgCore (remote %d): "
                " failed status %d\n", dstCoreId, status);
    }
}


void Ipc_sendNewTestIndex(RPMessage_Handle handle, uint32_t srcEndPt, uint32_t testIndex, 
        uint32_t dstCoreId)
{
    uint32_t   bufSize;
    uint8_t    buf[64];
    uint32_t   payloadSize = 5;  /* FuncCode + TestIndex */
    int32_t    status = IPC_SOK;
    uint32_t   dstEndPt = ENDPT1;

    /*
     * opcode    - 1byte
     * size      - 4bytes
     * FuncCode  - 1byte
     * testIndex - 4bytes
     *
     * */
    bufSize = 10;

    buf[0] = IPC_PERF_TEST; 
    memcpy(&buf[1], (const void*)&payloadSize, sizeof(uint32_t));
    buf[5] = IPC_FC_PERFTEST_TESTINDEX;
    memcpy(&buf[6], (const void*)&testIndex, sizeof(uint32_t));

    status = RPMessage_send(handle, dstCoreId, dstEndPt, srcEndPt, (Ptr)buf, bufSize);
    if (status != IPC_SOK) 
    {
        SystemP_printf("Ipc_sendNewTestIndex (remote %d): RPMessage_send "
                " failed status %d\n", dstCoreId, status);
    }
}

void Ipc_sendTestResult(RPMessage_Handle handle, uint32_t srcEndPt)
{
    uint32_t   bufSize;
    uint8_t    buf[256];
    uint32_t   payloadSize;  /* FuncCode + TestResult size */
    uint32_t   dstEndPt = ENDPT1;
    uint32_t   dstCoreId = PRINT_HOST_CORE;
    uint32_t   szData = 0;
    int32_t    status = IPC_SOK;

    /*
     * opcode    - 1byte
     * size      - 4bytes
     * FuncCode  - TestResult
     * TetResult - 128 byte max
     * */

    buf[0] = IPC_PERF_TEST; 
    buf[5] = IPC_FC_PERFTEST_TESTRESULT;
    szData = Ipc_copyRestResult(&buf[6], 250);

    if(szData > 0U)
    {
        payloadSize = 1 + szData;

        memcpy(&buf[1], (const void*)&payloadSize, sizeof(uint32_t));
        bufSize = 6 + szData;

        status = RPMessage_send(handle, dstCoreId, dstEndPt, srcEndPt, (Ptr)buf, bufSize);
        if (status != IPC_SOK) 
        {
            SystemP_printf("Ipc_sendTestResult: RPMessage_send failed status %d\n", 
                    status);
        }
    }
}

uint32_t Ipc_processPerfCmd(RPMessage_Handle handle, uint32_t dstEndPt, uint32_t dstCoreId, 
                uint32_t srcEndPt, uint8_t *buf, uint32_t bufSize)
{
    uint32_t        retVal = FALSE;
    uint32_t        funcCode;
    uint32_t        dataSize;
    uint32_t        selfId = Ipc_getCoreId();

    if(NULL != buf)
    {
        if(bufSize == 4U)
        {
            /* This is raw data for round-trip measurement */
            if(IPC_SOK != RPMessage_send(handle, dstCoreId, dstEndPt, srcEndPt, 
                        buf, bufSize))
            {
                SystemP_printf("Ipc_processPerfCmd: RPMessage_send failed\n");
            }
        }
        else if(bufSize > 4U)
        {
            memcpy((void*)&dataSize, (const void*)&buf[1], sizeof(uint32_t));
            if(dataSize > 0U)
            {
                funcCode = buf[5];
                switch(funcCode)
                {
                    case IPC_FC_PERFTEST_TESTINDEX:
                        memcpy((void*)&curTestIndex, (const void*)&buf[6], sizeof(uint32_t));
                        SemaphoreP_post(semHandle);
                        retVal = TRUE;
                        break;

                    case IPC_FC_PERFTEST_COMPLETED:
                        memcpy((void*)&curTestIndex, (const void*)&buf[6], sizeof(uint32_t));
                        SemaphoreP_post(semHandle);
                        retVal = TRUE;

                        /* Send Test Result to print host core */
                        if(PRINT_HOST_CORE != selfId)
                        {
                            Ipc_sendTestResult(handle, srcEndPt);
                        }

                        if(PRINT_HOST_CORE != selfId)
                        {
                            testCompleted = TRUE;
                        }

                        break;

                    case IPC_FC_PERFTEST_TESTRESULT:
                        if(PRINT_HOST_CORE == selfId)
                        {
                            Ipc_addPerfTestResults(&buf[6], dataSize-1);
                        }
                        break;
                }

            }
        }
    }

    return retVal;
}

void Ipc_printTestReport()
{
    uint32_t   selfId = Ipc_getCoreId();

    if(PRINT_HOST_CORE == selfId)
    {
        Ipc_printPerfTestReport();
    }    
}

