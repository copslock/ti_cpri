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
 *  \file sciclient_ut_main.c
 *
 *  \brief Implementation of Sciclient Unit Test
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#if defined (__aarch64__)
#include <ti/sysbios/family/arm/v8a/Mmu.h>
#endif

#include <ti/osal/TimerP.h>

#include <stdint.h>
#include <string.h>
#include <ti/csl/tistdtypes.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/hw_types.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>
#include <ti/drv/sciclient/examples/sciclient_unit_testapp/sciclient_ut_tests.h>
#if defined (__C7100__)
#include <ti/sysbios/family/c7x/Mmu.h>
#include <ti/csl/csl_clec.h>
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
void mainTask(UArg arg0, UArg arg1);
static int32_t App_getRevisionTestPol(void);
static int32_t App_getRevisionTestIntr(void);
static int32_t App_timeoutTest(void);
static int32_t App_invalidReqPrmTest(void);
static int32_t App_msmcQueryTest(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static volatile int32_t gTestStatus;
static uint8_t  gAppTskStackMain[16*1024] __attribute__((aligned(8192)));;
/* IMPORTANT NOTE: For C7x,
 * - stack size and stack ptr MUST be 8KB aligned
 * - AND min stack size MUST be 16KB
 * - AND stack assigned for task context is "size - 8KB"
 *       - 8KB chunk for the stack area is used for interrupt handling in this task context
 */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int main(void)
{
    Task_Handle task;
    Task_Params taskParams;
    Error_Block eb;

    App_SciclientC7xPreInit();

    uint32_t retVal = CSL_PASS;
    #if defined (__C7100__)
    /* 256 CLEC interrupt number mapped to interrupt number 14 from BIOS for
     * timer.
     */
    {
        CSL_CLEC_EVTRegs * regs = (CSL_CLEC_EVTRegs *) CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE;
        CSL_ClecEventConfig evtCfg;
        evtCfg.secureClaimEnable = 0;
        evtCfg.evtSendEnable = 1;
        evtCfg.rtMap = 0x3C;
        evtCfg.extEvtNum = 0x0;
        evtCfg.c7xEvtNum = 14;
        /* Clec interrupt number 1024 is connected to GIC interrupt number 32 in J721E.
        * Due to this for CLEC programming one needs to add an offset of 992 (1024 - 32)
        * to the event number which is shared between GIC and CLEC.
        */
        CSL_clecConfigEvent(regs, 256 + 992, &evtCfg);
    }
    #endif

    Task_Params_init(&taskParams);
    taskParams.priority = 14;
    taskParams.stack        = gAppTskStackMain;
    taskParams.stackSize    = sizeof (gAppTskStackMain);

    Error_init(&eb);
    App_sciclientConsoleInit();

    task = Task_create(mainTask, &taskParams, &eb);

    if(NULL==task)
    {
        App_sciclientPrintf("Task_create() mainTask failed! \n");
        BIOS_exit(0);
    }

    BIOS_start();

    return retVal;
}

void mainTask(UArg arg0, UArg arg1)
{
    /*To suppress unused variable warning*/
    (void)arg0;
    (void)arg1;

    volatile uint32_t loopForever = 1U;

    App_sciclientParser();

    while(loopForever);
}

uint32_t App_sciclientGetNumTests(void)
{
    return SCICLIENT_NUM_TESTCASES;
}

int32_t App_sciclientTestMain(App_sciclientTestParams_t *testParams)
{
    switch (testParams->testcaseId)
    {
        case 1:
            testParams->testResult = App_getRevisionTestPol();
            break;
        case 2:
            testParams->testResult = App_getRevisionTestIntr();
            break;
        case 3:
            testParams->testResult = App_invalidReqPrmTest();
            break;
        case 4:
            testParams->testResult = App_timeoutTest();
            break;
        case 5:
            testParams->testResult = App_msmcQueryTest();
            break;
        default:
            break;
    }
    return 0;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

int32_t App_getRevisionTestPol(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL,
        1
    };
    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) &request,
        sizeof(request),
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    status = Sciclient_init(&config);

    if (status == CSL_PASS)
    {
        status = Sciclient_service(&reqPrm, &respPrm);

        if (CSL_PASS == status)
        {
            if (respPrm.flags == TISCI_MSG_FLAG_ACK)
            {
                status = CSL_PASS;
                App_sciclientPrintf(
                                  " DMSC Firmware Version %s\n",
                                  (char *) response.str);
                App_sciclientPrintf(
                                  " Firmware revision 0x%x\n", response.version);
                App_sciclientPrintf(
                                  " ABI revision %d.%d\n", response.abi_major,
                                  response.abi_minor);
            }
            else
            {
                App_sciclientPrintf(
                                  " DMSC Firmware Get Version failed \n");
            }
        }
        else
        {
            App_sciclientPrintf(
                              " DMSC Firmware Get Version failed \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

int32_t App_getRevisionTestIntr(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT,
        NULL
    };

    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) &request,
        sizeof(request),
        SCICLIENT_SERVICE_WAIT_FOREVER
    };
    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_service(&reqPrm, &respPrm);
        if (CSL_PASS == status)
        {
            if (respPrm.flags == TISCI_MSG_FLAG_ACK)
            {
                status = CSL_PASS;
                App_sciclientPrintf(
                                  " DMSC Firmware Version %s\n",
                                  (char *) response.str);
                App_sciclientPrintf(
                                  " Firmware revision 0x%x\n", response.version);
                App_sciclientPrintf(
                                  " ABI revision %d.%d\n", response.abi_major,
                                  response.abi_minor);
            }
            else
            {
                App_sciclientPrintf(
                                  " DMSC Firmware Get Version failed \n");
            }
        }
        else
        {
            App_sciclientPrintf(
                              " DMSC Firmware Get Version failed \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

static int32_t App_invalidReqPrmTest(void)
{
    int32_t status = CSL_EFAIL;

    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT,
        NULL
    };
    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm_badTxSize =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t*) &request,
        100,
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    const Sciclient_ReqPrm_t      reqPrm_good =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t*)&request,
        0,
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm_badRxsize =
    {
        0,
        (uint8_t *) &response,
        100
    };

    Sciclient_RespPrm_t           respPrm_good =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_service(NULL, &respPrm_good);
        if (CSL_EBADARGS == status)
        {
            status = CSL_PASS;
            App_sciclientPrintf(" NULL Arg Test PASSED \n");
        }
        else
        {
            App_sciclientPrintf(" NULL Arg Test FAILED \n");
        }
        status = Sciclient_service(&reqPrm_badTxSize, &respPrm_good);
        if (CSL_EBADARGS == status)
        {
            status = CSL_PASS;
            App_sciclientPrintf(" Tx Payload Check PASSED \n");
        }
        else
        {
            App_sciclientPrintf(" Tx Payload Check FAILED \n");
        }
        status = Sciclient_service(&reqPrm_good, &respPrm_badRxsize);
        if (CSL_EBADARGS == status)
        {
            status = CSL_PASS;
            App_sciclientPrintf(" Rx Payload Check PASSED \n");
        }
        else
        {
            App_sciclientPrintf(" Rx Payload Check FAILED \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

static int32_t App_timeoutTest(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT,
        NULL
    };
    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *)&request,
        sizeof(request),
        (uint32_t)0x0A
    };
    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_service(&reqPrm, &respPrm);
        if (CSL_ETIMEOUT == status)
        {
            status = CSL_PASS;
            App_sciclientPrintf(
                             " Timeout test PASSED \n");
        }
        else
        {
            App_sciclientPrintf(
                             " Timeout Test FAILED \n");
        }
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

static int32_t App_msmcQueryTest(void)
{
    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL
    };

    const struct tisci_query_msmc_req req = {};
    struct tisci_query_msmc_resp resp;

    status = Sciclient_init(&config);
    if (status == CSL_PASS)
    {
        status = Sciclient_msmcQuery(&req, &resp, SCICLIENT_SERVICE_WAIT_FOREVER);
    }
    if (status == CSL_PASS)
    {
        App_sciclientPrintf("MSMC Start Address = 0x%X%X\n", resp.msmc_start_high, resp.msmc_start_low);
        App_sciclientPrintf("MSMC End Address = 0x%X%X\n", resp.msmc_end_high, resp.msmc_end_low);
    }
    else
    {
        App_sciclientPrintf("MSMC Query FAILED !!");
    }
    if (status == CSL_PASS)
    {
        status = Sciclient_deinit();
    }
    return status;
}

#if defined(BUILD_MPU) || defined (__C7100__)
extern void Osal_initMmuDefault(void);
void InitMmu(void)
{
    Osal_initMmuDefault();
}
#endif
