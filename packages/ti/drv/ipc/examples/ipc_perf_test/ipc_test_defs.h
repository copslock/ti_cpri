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
 *  \file ipc_test_defs.h
 *
 *  \brief This file defines the testsetup
 *
 *
 */
#ifndef IPC_TEST_DEFS_H_
#define IPC_TEST_DEFS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
 
 /* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef enum Ipc_TestNames_e
{
    IPC_PERF_TEST = 0x80,
    IPC_TESTTYPE_COUNT
}Ipc_TestNames;

typedef enum Ipc_FC_PerfTest_e
{
    IPC_FC_PERFTEST_TESTINDEX,
    IPC_FC_PERFTEST_COMPLETED,
    IPC_FC_PERFTEST_TESTRESULT,

    IPC_FC_TEST_CNT
}Ipc_FC_PerfTest;

typedef struct Ipc_TestDef_s
{
    uint32_t    testId;
    uint32_t    testType;
    uint32_t    hostCore;
    uint32_t    slaveCore;
    uint32_t    param1;
    uint32_t    param2;
}Ipc_Testcase;

typedef struct Ipc_TestPerfResult_s
{
    uint32_t    testId;
    uint32_t    testTime;
}Ipc_TestPerfResult;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Returns Testcase info
 * */
Ipc_Testcase* Ipc_getTestcase(uint32_t testIndex);

/**
 * Add Performance Test Results to the table
 * */
void Ipc_addPerfTestResult(uint32_t testId, uint32_t testTime);

/**
 * Add Performance Test Results to the table from buffer
 * */
void Ipc_addPerfTestResults(uint8_t *buf, uint32_t bufSize);

/**
 * Copy the test results to the given buffer
 * */
uint32_t Ipc_copyRestResult(uint8_t *buf, uint32_t bufSize);

/**
 * Return TRUE/FALSE if it has test  result to send
 * */
uint32_t Ipc_hasPerfTestResult(void);

/**
 * Print the Performance Test Report
 * */
void Ipc_printPerfTestReport();

/**
 * Return Host core and Slave Core ID for given TestCaseId
 * */
void Ipc_getCoreIdForTestcase(uint32_t testId, uint32_t *hostCore, uint32_t *remCore);

#ifdef __cplusplus
}
#endif

#endif  /* #define IPC_TEST_DEFS_H_ */
