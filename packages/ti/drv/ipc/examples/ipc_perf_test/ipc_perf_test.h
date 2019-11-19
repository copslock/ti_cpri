/*
 *  Copyright (c) Texas Instruments Incorporated 2019
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
 *  \file ipc_perf_test.h
 *
 *  \brief This file defines the prototype for perfTest
 *
 *
 */
#ifndef IPC_PERF_TEST_H_
#define IPC_PERF_TEST_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drv/ipc/ipc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
 
 /* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */




/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void Ipc_perf_test_setup(void);

RPMessage_Handle Ipc_createRpmsg(uint8_t *buf, uint32_t bufSize, uint32_t *myEndPt);

void Ipc_recvTaskFxn(uint32_t *arg0, uint32_t *arg1);

TaskP_Handle Ipc_createRcvThread(uint32_t *arg0, uint32_t *arg1);

void Ipc_runPerfTest(uint32_t coreId, uint32_t numCount, uint32_t testId);

void Ipc_sendNewTestIndex(RPMessage_Handle handle, uint32_t srcEndPt, uint32_t testIndex, 
        uint32_t dstCoreId);

void Ipc_sendTestCompletedMsgCore(RPMessage_Handle handle, uint32_t srcEndPt, uint32_t dstCoreId);

void Ipc_sendTestCompletedMsg(RPMessage_Handle handle, uint32_t srcEndPt);

uint32_t Ipc_processPerfCmd(RPMessage_Handle handle, uint32_t dstEndPt, uint32_t dstCoreId, 
                uint32_t srcEndPt, uint8_t *buf, uint32_t bufSize);

void Ipc_sendTestResult(RPMessage_Handle handle, uint32_t srcEndPt);

void Ipc_printTestReport();

#ifdef __cplusplus
}
#endif

#endif  /* #define IPC_PERF_TEST_H_ */
