/*
 *  Copyright (C) 2018-2019 Texas Instruments Incorporated
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
 *  \file sciclient_ut_tests.h
 *
 *  \brief This file defines the test cases for SCICLIENT UT.
 */

#ifndef SCICLIENT_TEST_CASES_H_
#define SCICLIENT_TEST_CASES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <ti/csl/soc.h>
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define SCICLIENT_NUM_TESTCASES   (sizeof (gSciclientTestcaseParams) / \
                                   sizeof (App_sciclientTestParams_t))

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Defines the various SCICLIENT test cases. */
App_sciclientTestParams_t gSciclientTestcaseParams[] =
{
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        1U,

        /** *reqId **/
        "PDK-2142::PDK-2140::PDK-2139::PDK-2141",

        /** *testCaseName **/
        "SCICLIENT DMSC Get Firmware Version (Polled)",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient loads firmware and gets ACK after sending get revision\
         message to firmware without any error ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },

    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        2U,

        /** *reqId **/
        "PDK-2142::PDK-2140::PDK-2139::PDK-2141",

        /** *testCaseName **/
        "SCICLIENT DMSC Get Firmware Version (Interrupt)",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient loads firmware and gets ACK after sending get revision\
         message to firmware without any error ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    },
    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        3U,

        /** *reqId **/
        "PDK-2490::PDK-2915::PDK-2907::PDK-2906",

        /** *testCaseName **/
        "SCICLIENT Check for Invalid Req Prm",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient_service should fail when \
         called with invalid parameters ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_NEGATIVE)
    },

    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        4U,

        /** *reqId **/
        "PDK-2490::PDK-2915::PDK-2907::PDK-2906",

        /** *testCaseName **/
        "SCICLIENT Check Timeout",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient_service should fail with timeout when \
         called with invalid parameters ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_NEGATIVE)
    },

    {
        /** enableTest **/
        TEST_ENABLE,

        /** testCaseId **/
        5U,

        /** *reqId **/
        "PDK:4277",

        /** *testCaseName **/
        "SCICLIENT MSMC Query Test",

        /** *userInfo **/
        "None",

        /** *disableReason **/
        "None",

        /** *passFailCriteria **/
        "Sciclient_msmcQuery should return the correct \
         start and end address of msmc region ",

        /** cpuID **/
        APP_SCICLIENT_R5F,

        /** sciclientConfigParams **/
        {},

        /** printEnable **/
        PRINT_ENABLE,

        /** testType **/
        (APP_SCICLIENT_TEST_TYPE_SANITY)
    }
};

#endif /* #ifndef SCICLIENT_TEST_CASES_H_ */
