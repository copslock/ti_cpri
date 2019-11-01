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
 *  \file sciclient_firmware_boot_main.c
 *
 *  \brief Implementation of System firmware boot test
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <ti/csl/tistdtypes.h>
#include <ti/csl/soc.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/hw_types.h>
#include <ti/osal/CacheP.h>
#include <ti/drv/sciclient/examples/common/sciclient_appCommon.h>
#include <ti/osal/TimerP.h>

#if defined(BUILD_MCU1_0) && defined(SOC_AM65XX)
#include <ti/drv/mmcsd/MMCSD.h>
#include <ti/drv/mmcsd/soc/MMCSD_soc.h>
#include <ti/drv/mmcsd/src/MMCSD_osal.h>

/* FATFS header file */
#include <ti/fs/fatfs/ff.h>
#include <ti/fs/fatfs/FATFS.h>
#include <ti/board/board.h>
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static volatile int32_t gTestStatus;
static uint64_t time_usecs[5] = { 0 };

#pragma DATA_SECTION(gSciclient_firmware, ".firmware")

#if defined (BUILD_MCU1_0) && defined(SOC_AM65XX)
uint32_t gSciclient_firmware[BINARY_FILE_SIZE_IN_BYTES/4];

/* MMCSD function table for MMCSD implementation */
FATFS_DrvFxnTable FATFS_drvFxnTable = {
    &MMCSD_close,
    &MMCSD_control,
    &MMCSD_init,
    &MMCSD_open,
    &MMCSD_write,
    &MMCSD_read
};

/* FATFS configuration structure */
FATFS_HwAttrs FATFS_initCfg[_VOLUMES] =
{
    { 1U },
    { 0U },
    { 0U },
    { 0U }
};

/* FATFS objects */
FATFS_Object FATFS_objects[_VOLUMES];

/* FATFS Handle */
FATFS_Handle sciclientTest_fatfsHandle = NULL;

/* FATFS configuration structure */
const FATFS_Config FATFS_config[_VOLUMES + 1] = {
    {
        &FATFS_drvFxnTable,
        &FATFS_objects[0],
        &FATFS_initCfg[0]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[1],
         &FATFS_initCfg[1]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[2],
         &FATFS_initCfg[2]
    },

    {NULL, NULL, NULL},

    {NULL, NULL, NULL}
};
#else
uint32_t gSciclient_firmware[BINARY_FILE_SIZE_IN_BYTES/4] = SCICLIENT_FIRMWARE;
#endif
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t App_loadFirmwareTest(void);
#if defined(SOC_AM65XX)
int32_t sciclientTest_ReadSysfwImage(void *sysfw_ptr, uint32_t num_bytes);
#endif
void App_printPerfStats(void);
static int32_t App_boardCfgTest(void);
static int32_t App_getRevisionTestPol(void);
void _resetvectors (void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/**
 *  main
 *  Application main function.
 */
int32_t main(void)
{
#if defined (SOC_J721E)
    /* Relocate CSL Vectors to ATCM*/
    memcpy((void *)CSL_MCU_ARMSS_ATCM_BASE, (void *)_resetvectors, 0x100);
#else
    /* Relocate CSL Vectors to ATCM*/
    memcpy((void *)CSL_MCU_ATCM_BASE, (void *)_resetvectors, 0x100);
    App_sciclientConsoleInit();
#endif
    App_loadFirmwareTest();
    App_getRevisionTestPol();
    App_boardCfgTest();
    #if defined(SOC_AM65XX)
    App_printPerfStats();
    #endif
    return 0;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

int32_t App_loadFirmwareTest(void)
{
    uint64_t start_ticks=0, stop_ticks=0;

    int32_t status = CSL_EFAIL;
    void *sysfw_ptr = gSciclient_firmware;

    #if defined(SOC_AM65XX)
    App_sciclientPrintf(
                      "Make sure sysfw.bin is present in SD Card... \n");
    App_sciclientPrintf(
                      " Reading from SD Card... \n");

    status = sciclientTest_ReadSysfwImage(sysfw_ptr, BINARY_FILE_SIZE_IN_BYTES);
    /*Do a cache writeback*/
    CacheP_wbInv(sysfw_ptr, BINARY_FILE_SIZE_IN_BYTES);

    if (status != CSL_PASS)
    {
        App_sciclientPrintf(
                      " DMSC read from SD Card Failed... \n");
    }
    else
    {
        App_sciclientPrintf(
                       " DMSC Loading the Firmware... \n");
    }
    #else
    sysfw_ptr = (void *)&gSciclient_firmware;

    /*Do a cache writeback*/
    CacheP_wbInv(sysfw_ptr, BINARY_FILE_SIZE_IN_BYTES);
    #endif

    start_ticks = TimerP_getTimeInUsecs();
    status = Sciclient_loadFirmware(sysfw_ptr);
    stop_ticks = TimerP_getTimeInUsecs();

    time_usecs[0U] = (stop_ticks-start_ticks);

    start_ticks=0U;
    stop_ticks=0U;

    if (status == CSL_PASS)
    {
        App_sciclientPrintf(
                          " DMSC Loading the Firmware...SUCCESS \n");
    }
    else
    {
        App_sciclientPrintf(
                          " DMSC Loading the Firmware...FAILED \n");
    }

    return status;
}

#if defined(SOC_AM65XX)
int32_t sciclientTest_ReadSysfwImage(void *sysfw_ptr, uint32_t num_bytes)
{
    int32_t retVal = CSL_PASS;
    const TCHAR *fileName = "0:/sysfw.bin";
    FIL     fp;
    FRESULT  fresult;
    uint32_t bytes_read = 0;
    MMCSD_v2_HwAttrs hwAttrsConfig;

     if(MMCSD_socGetInitCfg(FATFS_initCfg[0].drvInst,&hwAttrsConfig)!=0) {
       App_sciclientPrintf("\nUnable to get config.Exiting. TEST FAILED.\r\n");
       retVal = CSL_EFAIL;
     }

    hwAttrsConfig.enableInterrupt = ((uint32_t)(0U));
    hwAttrsConfig.configSocIntrPath=NULL;

    if(MMCSD_socSetInitCfg(FATFS_initCfg[0].drvInst,&hwAttrsConfig)!=0) {
       App_sciclientPrintf("\nUnable to set config.Exiting. TEST FAILED.\r\n");
       retVal = CSL_EFAIL;
     }

    if (sciclientTest_fatfsHandle)
    {
    }
    else
    {
        /* Initialization of the driver. */
        FATFS_init();

        /* MMCSD FATFS initialization */
        FATFS_open(0U, NULL, &sciclientTest_fatfsHandle);
    }

    fresult = f_open(&fp, fileName, ((BYTE)FA_READ));
    if (fresult != FR_OK)
    {
        App_sciclientPrintf("\n SD Boot - sysfw File open fails \n");
       retVal = CSL_EFAIL;
    }
    fresult  = f_read(&fp, sysfw_ptr, num_bytes, &bytes_read);
    if (fresult != FR_OK)
    {
        App_sciclientPrintf("\n SD Boot - sysfw read fails \n");
       retVal = CSL_EFAIL;
    }

    f_close(&fp);

    return retVal;
}
#endif

int32_t App_getRevisionTestPol(void)
{
    uint64_t start_ticks=0, stop_ticks=0;

    int32_t status = CSL_EFAIL;
    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL
    };

    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        NULL,
        0,
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    start_ticks = TimerP_getTimeInUsecs();
    status = Sciclient_init(&config);
    stop_ticks = TimerP_getTimeInUsecs();

    time_usecs[1] = (stop_ticks-start_ticks);

    start_ticks=0U;
    stop_ticks=0U;

    if (status == CSL_PASS)
    {
        start_ticks = TimerP_getTimeInUsecs();
        status = Sciclient_service(&reqPrm, &respPrm);
        stop_ticks = TimerP_getTimeInUsecs();

        time_usecs[2] = (stop_ticks-start_ticks);

        start_ticks=0U;
        stop_ticks=0U;

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
        start_ticks = TimerP_getTimeInUsecs();
        status = Sciclient_deinit();
        stop_ticks = TimerP_getTimeInUsecs();

        time_usecs[3] = (stop_ticks-start_ticks);

        start_ticks=0U;
        stop_ticks=0U;
    }
    return status;
}

int32_t App_boardCfgTest(void)
{
    int32_t status = CSL_PASS;
    int32_t temp_status = CSL_EFAIL;

    uint64_t start_ticks=0U, stop_ticks=0U;

    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
        NULL
    };

    if (Sciclient_init(&config) == CSL_PASS)
    {
        App_sciclientPrintf(
                          "Sciclient init...PASSED \n");

        start_ticks = TimerP_getTimeInUsecs();
        temp_status = Sciclient_boardCfg(NULL);
        stop_ticks = TimerP_getTimeInUsecs();
        time_usecs[4] = (stop_ticks-start_ticks);

        start_ticks=0U;
        stop_ticks=0U;

        if (temp_status == CSL_PASS)
        {
            App_sciclientPrintf(
                              " Board configuration test...PASSED \n");
        }
        else
        {
            App_sciclientPrintf(
                              " Board configuration test...FAILED \n");
            status = CSL_EFAIL;
        }
        if (Sciclient_boardCfgPm(NULL) == CSL_PASS)
        {
            App_sciclientPrintf(
                              " Board configuration for PM test...PASSED \n");
        }
        else
        {
            App_sciclientPrintf(
                              " Board configuration for PM test...FAILED \n");
            status = CSL_EFAIL;
        }

        if (Sciclient_boardCfgRm(NULL) == CSL_PASS)
        {
            App_sciclientPrintf(
                              " Board configuration for RM test...PASSED \n");
        }
        else
        {
            App_sciclientPrintf(
                              " Board configuration for RM test...FAILED \n");
            status = CSL_EFAIL;
        }

        if (Sciclient_boardCfgSec(NULL) == CSL_PASS)
        {
            App_sciclientPrintf(
                              " Board configuration for SECURITY test......PASSED \n");
        }
        else
        {
            App_sciclientPrintf(
                              " Board configuration for SECURITY test...FAILED \n");
            status = CSL_EFAIL;
        }
    }
    else
    {
        App_sciclientPrintf(
                          "Sciclient init...FAILED \n");
        status = CSL_EFAIL;
    }
    Sciclient_deinit();
    return status;
}

void App_printPerfStats()
{
    App_sciclientPrintf("\n======================================\n");
    App_sciclientPrintf("           PERFORMANCE OF APIS          \n");
    App_sciclientPrintf("======================================\n");

    App_sciclientPrintf("Sciclient_loadFirmware |   %llu us    \n", time_usecs[0]);
    App_sciclientPrintf("Sciclient_init         |   %llu us    \n", time_usecs[1]);
    App_sciclientPrintf("Sciclient_service      |   %llu us    \n", time_usecs[2]);
    App_sciclientPrintf("Sciclient_deinit       |   %llu us    \n", time_usecs[3]);
    App_sciclientPrintf("Sciclient_boardCfg     |   %llu us    \n", time_usecs[4]);
    App_sciclientPrintf("======================================\n\n Note: Here Sciclient_service is done for getRevision(Polling).\n");
}
