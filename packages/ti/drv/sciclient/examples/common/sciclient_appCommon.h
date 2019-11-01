/*
 *  Copyright (C) 2017 Texas Instruments Incorporated
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
 *  \file st_sciclient.h
 *
 *  \brief This file contains all the structures, macros, enums
 *  used by the SCICLIENT UT applications.
 */

#ifndef ST_SCICLIENT_H_
#define ST_SCICLIENT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <ti/csl/soc.h>
#include <ti/csl/tistdtypes.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */
/**
 *  \anchor Sciclient_TestEnables
 *  \name Sciclient Test Enable and Disable indexes
 *  @{
 *  Macros to enable or disable tests and printing.
 */
#define TEST_ENABLE                     (TRUE)
#define TEST_DISABLE                    (FALSE)
#define PRINT_ENABLE                    (TRUE)
#define PRINT_DISABLE                   (FALSE)
/* @} */

/**
 *  \anchor Sciclient_ParserIds
 *  \name Sciclient Test case Parser Ids
 *  @{
 *  Parser ID used to identify the different test types.
 */
#define RUN_TEST_ID                 ('1')
#define RUN_TESTS_SANITY            ('2')
#define RUN_TESTS_REGRESSION        ('3')
#define RUN_TESTS_R5F               ('4')
#define RUN_TESTS_A53               ('5')
#define RUN_TESTS_ALL               ('A')
#define PRINT_ALLTESTS              ('d')
#define PRINT_TEST_ID               ('t')
#define PRINT_RESULTS               ('g')
#define PARSER_QUIT                 ('q')
/* @} */
#if defined (__C7100__) || defined (_TMS320C6X)
#define App_sciclientPrintf printf
#define App_sciclientGetChar getchar
#else
#define App_sciclientPrintf UART_printf
#define App_sciclientGetChar UART_getc
#endif

/*Change this to 0 for interactive UT*/
#define APP_SCICLIENT_UT_AUTORUN 1

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 *  \brief CPU Id of the test to be run on.
 */
typedef enum
{
    APP_SCICLIENT_R5F = 0,
    /**< Select R5F for execution */
    APP_SCICLIENT_A53 = 1,
    /**< Select A53 for execution */
}App_sciclientCpuId;

/**
 *  \brief Test types.
 */
typedef enum
{
    APP_SCICLIENT_TEST_TYPE_SANITY      = 0x01,
    /**< Sanity testing */
    APP_SCICLIENT_TEST_TYPE_REGRESSION  = 0x02,
    /**< Regression testing */
    APP_SCICLIENT_TEST_TYPE_FULL        = 0x04,
    /**< Full testing */
    APP_SCICLIENT_TEST_TYPE_FUNCTIONAL  = 0x08,
    /**< Functional testing */
    APP_SCICLIENT_TEST_TYPE_STRESS      = 0x10,
    /**< Stress testing */
    APP_SCICLIENT_TEST_TYPE_NEGATIVE    = 0x20,
    /**< Negative testing */
    APP_SCICLIENT_TEST_TYPE_PERFORMANCE = 0x40,
    /**< Performance Testing */
    APP_SCICLIENT_TEST_TYPE_MISC        = 0x80,
    /**< Miscellaneous Testing */
    APP_SCICLIENT_TEST_TYPE_API         = 0x100
    /**< API testing */
} App_sciclientTestType;

/**
 * \brief Typedef for test case parameters.
 */
typedef struct App_sciclientTestParams App_sciclientTestParams_t;

/**
 * \brief Typedef for test case type function pointer.
 */
typedef void (*sciclientTestCaseFxnPtr)(App_sciclientTestParams_t *testPrms);

/**
 * \brief Typedef for test case configuration parameters.
 */
typedef struct
{

} App_sciclientTestConfigParams_t;

/**
 *  \brief Test case parameter structure.
 */
struct App_sciclientTestParams
{
    Bool                       enableTest;
    /**< Whether test case should be executed or not. */
    uint32_t                   testcaseId;
    /**< Test case ID. */
    char                      *reqId;
    /**< Requirements covered by this test case. */
    char                      *testCaseName;
    /**< Test case name. */
    char                      *userInfo;
    /**< Test case user Info. */
    char                      *disableReason;
    /**< Reason string for disabling a test case. */
    char                      *passFailCriteria;
    /**< Test case pass/fail criteria. */
    App_sciclientCpuId         cpuID;
    /**< Test case CPU ID field */
    App_sciclientTestConfigParams_t sciclientConfigParams;
    /**< Configuration parameters */
    Bool                       printEnable;
    /**< Enable/disable print statements, used for stress testing. */
    uint32_t                   testType;
    /**< Refer #App_sciclientTestType */
    /*
     * Below variables are initialized in code and not in table!!
     */
    int32_t                    isRun;
    /**< Flag to indicate whether the test case is run or not. */
    Int32                      testResult;
    /**< Test result. */
};
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   Fetch testcase data and run the testcase.
 *
 * \param   testParams        structure to details of testcase to run.
 */
void App_sciclientRun(App_sciclientTestParams_t *testParams);

/**
 * \brief   Parse the testcase data
 *
 * \param   None.
 *
 * \return  None
 */
int32_t App_sciclientParser(void);

/**
 * \brief   Fetch testcase data and run the testcase.
 *
 * \param   testParams        structure to details of testcase to run.
 */
int32_t App_sciclientTestMain(App_sciclientTestParams_t *testParams);

uint32_t App_sciclientGetNum();

void App_sciclientConsoleInit();

void App_SciclientC7xPreInit(void);

uint32_t   App_sciclientGetNumTests(void);

uint32_t App_sciclientConsolePuts(char *pTxBuffer,
                         int32_t numBytesToWrite);

#ifdef __cplusplus
}
#endif

#endif /* ST_SCICLIENT_H_ */
