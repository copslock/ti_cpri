/*
 *  Copyright (C) 2017-2018 Texas Instruments Incorporated
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
 *  \file st_App_sciclientParser.c
 *
 *  \brief User interface for SCICLIENTs driver testing.
 *
 *  This file takes the input from the user specified test file and
 *  then initiates the tests on SCICLIENT driver accordingly.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifndef BARE_METAL
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#endif

#include <stdio.h>
#include <string.h>
#include <ti/csl/csl_types.h>
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/board/board.h>
#if defined (__C7100__)
#include <ti/csl/csl_clec.h>
#include <ti/csl/arch/csl_arch.h>
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define DEBUG_FLAG        (0U)
#define APP_PRINT_BUFFER_SIZE (256U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   Parse the testcase data
 *
 * \param   None.
 *
 * \return  None
 */
int32_t App_sciclientParser(void);

/**
 * \brief   Get the index of the testcase to run from st_sciclientTestcases.h.
 *
 * \param   testcaseId        Testcase Id to run.
 *
 * \return  None
 */
int32_t App_sciclientGetIndex(uint32_t testcaseId);

/**
 * \brief   Print main menu.
 *
 * \param   None.
 *
 * \return  None
 */
void App_sciclientShowMainMenu(void);

/**
 * \brief   Print SCICLIENT system setings.
 *
 * \param   tcIdx Testcase index for which setting is to be print.
 *
 * \return  None
 */
void App_sciclientPrintSettings(uint32_t tcIdx);

/**
 * \brief   Print SCICLIENT testcases.
 *
 * \return  None
 */
void App_sciclientPrintTestName(void);

/**
 * \brief   Print SCICLIENT testcase details.
 *
 * \return  None
 */
void App_sciclientPrintTestDetails(uint32_t testId);

/**
 * \brief   Initialize  SCICLIENT testcase details.
 *
 * \return  None
 */
void App_sciclientResultInit(void);

/**
 * \brief   Prints SCICLIENT Test Case Results.
 *
 * \retur  None
 */
void App_sciclientPrintResults(void);

/**
 * \brief   Print messages on UART console.
 *
 * \param   None.
 *
 * \return  None
 */
void App_sciclientUARTConfig(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern App_sciclientTestParams_t gSciclientTestcaseParams[];

uint32_t autoRunEnable = (uint32_t) FALSE;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t App_sciclientParser(void)
{
    uint32_t testcaseId;
    uint32_t numTCPass, numTCFail, totalTCRun, done = 0U;
    int32_t  testcaseIdx, testIp;
    char     option;
    App_sciclientTestParams_t *testParams;

    App_sciclientConsoleInit();
    App_sciclientResultInit();

    while (!done)
    {
        autoRunEnable = (uint32_t) FALSE;
        App_sciclientShowMainMenu();
        #if (APP_SCICLIENT_UT_AUTORUN == 1)
        option = RUN_TESTS_ALL;
        #else
        App_sciclientPrintf( "\nEnter your choice: ");
        option = App_sciclientGetChar();
        App_sciclientPrintf( "%c\n", option);
        #endif

        numTCPass  = 0U;
        numTCFail  = 0U;
        totalTCRun = 0U;
        switch (option)
        {
            case RUN_TEST_ID:
                App_sciclientPrintf(
                                  "\nEnter testcase ID to run: ");
                testIp      = App_sciclientGetNum();
                testcaseIdx = App_sciclientGetIndex(testIp);
                if (testcaseIdx < 0)
                {
                    App_sciclientPrintf(
                                      "Invalid test case Id.");
                }
                else if (TEST_ENABLE ==
                         gSciclientTestcaseParams[testcaseIdx].enableTest)
                {
                    testParams = &gSciclientTestcaseParams[testcaseIdx];
                    if (APP_SCICLIENT_R5F == testParams->cpuID)
                    {
                        totalTCRun++;
                        App_sciclientRun(testParams);
                        if (CSL_PASS == testParams->testResult)
                        {
                            numTCPass++;
                        }
                        else
                        {
                            numTCFail++;
                        }
                    }
                }
                App_sciclientPrintf(
                                  "\nSCICLIENT: Total Testcases run:%u/%u\n",
                                  totalTCRun,
                                  App_sciclientGetNumTests());
                App_sciclientPrintf(
                                  "SCICLIENT: Number of Testcases Passed:%u\n",
                                  numTCPass);
                App_sciclientPrintf(
                                  "SCICLIENT: Number of Testcases Failed:%u\n",
                                  numTCFail);
                break;
            case RUN_TESTS_SANITY:
                for (testcaseId = 0;
                     testcaseId < App_sciclientGetNumTests();
                     testcaseId++)
                {
                    testParams = &gSciclientTestcaseParams[testcaseId];
                    if (TEST_ENABLE ==
                        testParams->enableTest)
                    {
                        if (APP_SCICLIENT_TEST_TYPE_SANITY ==
                            (testParams->testType & APP_SCICLIENT_TEST_TYPE_SANITY))
                        {
                            if (APP_SCICLIENT_R5F == testParams->cpuID)
                            {
                                totalTCRun++;
                                App_sciclientRun(testParams);
                                if (CSL_PASS == testParams->testResult)
                                {
                                    numTCPass++;
                                }
                                else
                                {
                                    numTCFail++;
                                }
                            }
                        }
                    }
                }
                App_sciclientPrintf(
                                  "\nSCICLIENT: Total Testcases run:%u/%u\n",
                                  totalTCRun,
                                  App_sciclientGetNumTests());
                App_sciclientPrintf(
                                  "SCICLIENT: Number of Testcases Passed:%u\n",
                                  numTCPass);
                App_sciclientPrintf(
                                  "SCICLIENT: Number of Testcases Failed:%u\n",
                                  numTCFail);
                App_sciclientPrintResults();
                break;
            case RUN_TESTS_ALL:
                for (testcaseId = 0;
                     testcaseId < App_sciclientGetNumTests();
                     testcaseId++)
                {
                    testParams = &gSciclientTestcaseParams[testcaseId];
                    if (TEST_ENABLE ==
                        testParams->enableTest)
                    {
                        totalTCRun++;
                        App_sciclientRun(testParams);
                        if (CSL_PASS == testParams->testResult)
                        {
                            numTCPass++;
                        }
                        else
                        {
                            numTCFail++;
                        }
                    }
                }
                App_sciclientPrintf(
                                  "\nSCICLIENT: Total Testcases run:%u/%u\n",
                                  totalTCRun,
                                  App_sciclientGetNumTests());
                App_sciclientPrintf(
                                  "SCICLIENT: Number of Testcases Passed:%u\n",
                                  numTCPass);
                App_sciclientPrintf(
                                  "SCICLIENT: Number of Testcases Failed:%u\n",
                                  numTCFail);
                App_sciclientPrintResults();
                done = 1;
                break; 
            case PRINT_ALLTESTS:
                App_sciclientPrintTestName();
                break;
            case PRINT_TEST_ID:
                App_sciclientPrintf(
                                  "\nEnter testcase ID: ");
                testIp      = App_sciclientGetNum();
                testcaseIdx = App_sciclientGetIndex(testIp);
                if (testcaseIdx < 0)
                {
                    App_sciclientPrintf(
                                      "Invalid test case Id.");
                }
                else
                {
                    App_sciclientPrintTestDetails(testcaseIdx);
                }
                break;
            case PRINT_RESULTS:
                App_sciclientPrintResults();
                break;
            case PARSER_QUIT:
                App_sciclientPrintf(
                                  "Exiting Sciclient Test application.\n");
                done = 1;
                break;
            default: App_sciclientPrintf(
                    "Unsupported Option. Please try again.\n");
        }
        if (0U == numTCFail)
        {
            App_sciclientPrintf(
                              "\nAll tests have PASSED.\n");
        }
        else
        {
            App_sciclientPrintf(
                              "\nSome of the Test-cases have FAILED.\n");
        }

    }
    return 0;
}

int32_t App_sciclientGetIndex(uint32_t testcaseId)
{
    int32_t  testcaseIdx = -1;
    uint32_t testCnt;
    const App_sciclientTestParams_t *testParams;

    testParams = &gSciclientTestcaseParams[0u];
    for (testCnt = 0; testCnt < App_sciclientGetNumTests(); testCnt++)
    {
        if (testParams[testCnt].testcaseId == testcaseId)
        {
            testcaseIdx = testCnt;
            break;
        }
    }
    return testcaseIdx;
}

void App_sciclientRun(App_sciclientTestParams_t *testParams)
{
    App_sciclientPrintf( "\n|TEST START|:: %u ::\n",
                      testParams->testcaseId);
    App_sciclientPrintf( "|TEST PARAM|:: %s ::\n",
                      testParams->testCaseName);
    App_sciclientPrintf( "|SR|:: %s ::\n",
                      testParams->reqId);

    App_sciclientPrintf(
        "--------------------------------------------------------------\n");
    App_sciclientPrintf(
        "=========================RUNNING==============================\n");
    App_sciclientTestMain(testParams);
    testParams->isRun = CSL_PASS;
    App_sciclientPrintf(
        "\n======================FINISHED==========================\n");
    if (testParams->testResult == 0U)
    {
        App_sciclientPrintf( "|TEST RESULT|PASS|%u|\n",
                          testParams->testcaseId);
    }
    else
    {
        App_sciclientPrintf( "|TEST RESULT|FAIL|%u|\n",
                          testParams->testcaseId);
    }
    App_sciclientPrintf( "|TEST INFO|:: %s ::\n",
                      testParams->testCaseName);
    App_sciclientPrintf( "|TEST END|:: %u ::\n",
                      testParams->testcaseId);
}

void App_sciclientShowMainMenu(void)
{
    App_sciclientPrintf( "\n=====================\n");
    App_sciclientPrintf( "SCICLIENT UT Select\n");
    App_sciclientPrintf( "=======================\n");
    App_sciclientPrintSettings(0);
    App_sciclientPrintf( "\nSCICLIENT UT main menu:\n");
    App_sciclientPrintf(
                      "1: Manual testing (select specific test case to run)\n");
    App_sciclientPrintf( "2. Sanity testing.\n");
    App_sciclientPrintf( "d: Display test cases.\n");
    App_sciclientPrintf( "t: Display test case Details.\n");
    App_sciclientPrintf( "g: Generate test report.\n");
    App_sciclientPrintf( "q: Quit.\n");
}

void App_sciclientPrintSettings(uint32_t tcIdx)
{
    /* Place holder in case the SCICLIENT has some system settings going
     * forward
     */
    return;
}

void App_sciclientPrintTestName(void)
{
    uint32_t loopCnt;
    const App_sciclientTestParams_t *testParams;

    App_sciclientPrintf( "SCICLIENT Testcase:\n");
    App_sciclientPrintf(
        "--------------------------------------------------------------\n");
    App_sciclientPrintf( "TC Id\tTC name\n");
    App_sciclientPrintf(
        "--------------------------------------------------------------\n");

    for (loopCnt = 0; loopCnt < App_sciclientGetNumTests(); loopCnt++)
    {
        testParams = &gSciclientTestcaseParams[loopCnt];
        App_sciclientPrintf( "%d\t%s\n",
                          testParams->testcaseId,
                          testParams->testCaseName);
    }

    App_sciclientPrintf(
        "--------------------------------------------------------------\n");
}

void App_sciclientPrintTestDetails(uint32_t testId)
{
    const App_sciclientTestParams_t *testParams;

    testParams = &gSciclientTestcaseParams[testId];
    App_sciclientPrintf( "\nTestcase Id : %d\n",
                      testParams->testcaseId);
    App_sciclientPrintf( "Testcase Name : %s",
                      testParams->testCaseName);
    App_sciclientPrintf(
                      "\nTestcase Enabled(0-Disabled/1-Enabled) : %d",
                      testParams->enableTest);
    App_sciclientPrintf( "\nTestcase Req. Id : %s",
                      testParams->reqId);
    App_sciclientPrintf( "\nTestcase User Info. : %s",
                      testParams->userInfo);
    App_sciclientPrintf( "\nTestcase Disable Reason : %s",
                      testParams->disableReason);
    App_sciclientPrintf( "\nTestcase CPU ID : %d",
                      testParams->cpuID);
    App_sciclientPrintf( "\nTestcase Type : %d",
                      testParams->testType);
    App_sciclientPrintf(
                      "\nTestcase Print Enable(0-Disabled/1-Enabled) : %d\n",
                      testParams->printEnable);
}

void App_sciclientResultInit(void)
{
    uint32_t loopCnt;
    App_sciclientTestParams_t *testParams;

    for (loopCnt = 0; loopCnt < App_sciclientGetNumTests(); loopCnt++)
    {
        testParams             = &gSciclientTestcaseParams[loopCnt];
        testParams->isRun      = CSL_EFAIL;
        testParams->testResult = CSL_EFAIL;
    }
}

void App_sciclientPrintResults(void)
{
    uint32_t loopCnt;
    int32_t  testcaseIdx;
    App_sciclientTestParams_t *testParams;
    char     testResult[10];

    App_sciclientPrintf(
        "-----------------------------------------------------------------------------------------------------------\r\n");
    App_sciclientPrintf(
        "ID\t         Description\t\t\t                         Status\r\n");
    App_sciclientPrintf(
        "-----------------------------------------------------------------------------------------------------------\r\n");
    for (loopCnt = 0; loopCnt < App_sciclientGetNumTests(); loopCnt++)
    {
        testParams  = &gSciclientTestcaseParams[loopCnt];
        testcaseIdx = testParams->testcaseId;
        if (CSL_PASS == testParams->testResult)
        {
            strcpy(testResult, "PASS");
        }
        else if (CSL_PASS == testParams->isRun)
        {
            strcpy(testResult, "FAIL");
        }
        else
        {
            strcpy(testResult, "NRY");
        }
        App_sciclientPrintf(
                          "%d\t         %s\t                         ",
                          testcaseIdx, testParams->testCaseName);
        App_sciclientPrintf(
                          "%s\r\n", testResult);
    }

    App_sciclientPrintf(
        "\n-----------------------------------------------------------------------------------------------------------\r\n");
}

uint32_t App_sciclientGetNum()
{
    uint32_t number=0U, numberOfBytes=0U;
    char buffer[APP_PRINT_BUFFER_SIZE];
    numberOfBytes = UART_gets(buffer, APP_PRINT_BUFFER_SIZE);
    if (numberOfBytes != 0U)
    {
        sscanf(buffer, "%u", &number);
    }
    return number;
}

void App_sciclientConsoleInit()
{
    Board_initCfg   boardCfg;
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);
}

uint32_t App_sciclientConsolePuts(char *pTxBuffer,
                                  int32_t numBytesToWrite)
{
    uint32_t retVal = 0U;
    retVal = UART_puts(pTxBuffer, numBytesToWrite);
    return retVal;
}

void App_SciclientC7xPreInit(void)
{
#if defined (__C7100__)
    CSL_ClecEventConfig cfgClec;
    CSL_CLEC_EVTRegs   *clecBaseAddr = (CSL_CLEC_EVTRegs*) CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE;
    uint32_t            i, maxInputs = 2048U;

    /* make secure claim bit to FALSE so that after we switch to non-secure mode
     * we can program the CLEC MMRs
     */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = FALSE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_DISABLE;
    cfgClec.extEvtNum         = 0U;
    cfgClec.c7xEvtNum         = 0U;
    for(i = 0U; i < maxInputs; i++)
    {
        CSL_clecConfigEvent(clecBaseAddr, i, &cfgClec);
    }

    i = CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_TIMER0_INTR_PEND_0 + 992;  /* Used for Timer Interrupt */
    /* Configure CLEC for DMTimer0, SYS/BIOS uses interrupt 14 for DMTimer0 by default */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = TRUE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_CPU_ALL;
    cfgClec.extEvtNum         = 0;
    cfgClec.c7xEvtNum         = 14;
    CSL_clecConfigEvent(clecBaseAddr, i, &cfgClec);

    /* Switch now */
    CSL_c7xSecSupv2NonSecSupv();
#endif

    return;
}


