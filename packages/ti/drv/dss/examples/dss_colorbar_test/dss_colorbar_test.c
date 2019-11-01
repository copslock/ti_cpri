/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 *  \file dss_colorbar_test.c
 *
 *  \brief DSS colorbar application that display colorbar present in the DSS.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <dss_colorbar_test.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if defined (SOC_AM65XX)
#define TEST_VP_ID                      (CSL_DSS_VP_ID_1)
#define TEST_OVERLAY_ID                 (CSL_DSS_OVERLAY_ID_1)
#else
#define TEST_VP_ID                      (CSL_DSS_VP_ID_2)
#define TEST_OVERLAY_ID                 (CSL_DSS_OVERLAY_ID_2)
#endif

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void DispApp_init(DispApp_Obj *appObj);
static void DispApp_deInit(DispApp_Obj *appObj);
static void DispApp_create(DispApp_Obj *appObj);
static void DispApp_delete(DispApp_Obj *appObj);
static int32_t DispApp_configDctrl(DispApp_Obj *appObj);
static int32_t DispApp_runTest(DispApp_Obj *appObj);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

DispApp_Obj gDispApp_Obj;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * DSS colorbar test
 */
int32_t Dss_colorbarTest(void)
{
    int32_t retVal = FVID2_SOK;

    DispApp_init(&gDispApp_Obj);

    App_print("DSS colorbar application started...\r\n");

    retVal = DispApp_runTest(&gDispApp_Obj);

    DispApp_deInit(&gDispApp_Obj);

    if(FVID2_SOK == retVal)
    {
        App_print("DSS colorbar test Passed!!\r\n");
    }
    else
    {
        App_print("DSS colorbar test Failed!!\r\n");
    }

    return (0);
}

static void DispApp_init(DispApp_Obj *appObj)
{
    int32_t         retVal = FVID2_SOK;
    Fvid2_InitPrms  initPrms;

    Fvid2InitPrms_init(&initPrms);
    initPrms.printFxn = &App_print;
    retVal = Fvid2_init(&initPrms);
    if(retVal != FVID2_SOK)
    {
        App_print("Fvid2 Init Failed!!!\r\n");
    }

    Dss_initParamsInit(&appObj->initParams);
    Dss_init(&appObj->initParams);

    if(FVID2_SOK == retVal)
    {
        /* Create DCTRL handle, used for common driver configuration */
        appObj->dctrlHandle = Fvid2_create(
            DSS_DCTRL_DRV_ID,
            DSS_DCTRL_INST_0,
            NULL,
            NULL,
            NULL);
        if(NULL == appObj->dctrlHandle)
        {
            App_print("DCTRL Create Failed!!!\r\n");
        }
    }

    if(FVID2_SOK == retVal)
    {
         App_print("DispApp_init() - DONE !!!\r\n");
    }

    return;
}

static void DispApp_deInit(DispApp_Obj *appObj)
{
    int32_t  retVal = FVID2_SOK;

    /* Delete DCTRL handle */
    retVal = Fvid2_delete(appObj->dctrlHandle, NULL);
    retVal += Dss_deInit();
    retVal += Fvid2_deInit(NULL);
    if(retVal != FVID2_SOK)
    {
         App_print("DCTRL handle delete failed!!!\r\n");
    }
    else
    {
         App_print("DispApp_deInit() - DONE !!!\r\n");
    }

    return;
}

static int32_t DispApp_runTest(DispApp_Obj *appObj)
{
    int32_t retVal = FVID2_SOK;

    /* Create driver */
    DispApp_create(appObj);

    App_print("Starting display ... !!!\r\n");
    App_print("Display in progress ... DO NOT HALT !!!\r\n");

#if defined (SIMULATOR)
    volatile uint32_t loop = 0xffffffU;
    while (0U != loop--);
#else
    Osal_delay(OSAL_DELAY_COUNT);
#endif

    if(FVID2_SOK == retVal)
    {
        /* Delete driver */
        DispApp_delete(appObj);
    }

    return retVal;
}

static void DispApp_create(DispApp_Obj *appObj)
{
    int32_t  retVal = FVID2_SOK;
    Dss_DctrlVpParams *vpParams;

    vpParams = &appObj->vpParams;
    Dss_dctrlVpParamsInit(vpParams);

    vpParams->vpId = TEST_VP_ID;
#if defined (SOC_AM65XX)
    vpParams->lcdOpTimingCfg.mInfo.standard = FVID2_STD_CUSTOM;
    vpParams->lcdOpTimingCfg.mInfo.width = DISP_APP_LCD_WIDTH;
    vpParams->lcdOpTimingCfg.mInfo.height = DISP_APP_LCD_HEIGHT;
    vpParams->lcdOpTimingCfg.mInfo.hFrontPorch = 48U;
    vpParams->lcdOpTimingCfg.mInfo.hBackPorch = 80U;
    vpParams->lcdOpTimingCfg.mInfo.hSyncLen = 32U;
    vpParams->lcdOpTimingCfg.mInfo.vFrontPorch = 3U;
    vpParams->lcdOpTimingCfg.mInfo.vBackPorch = 14U;
    vpParams->lcdOpTimingCfg.mInfo.vSyncLen = 6U;
#else
    vpParams->lcdOpTimingCfg.mInfo.standard = FVID2_STD_1080P_60;
#endif
    vpParams->lcdOpTimingCfg.dvoFormat = FVID2_DV_GENERIC_DISCSYNC;
    vpParams->lcdOpTimingCfg.videoIfWidth = FVID2_VIFW_24BIT;

    vpParams->lcdPolarityCfg.actVidPolarity = FVID2_POL_HIGH;
    vpParams->lcdPolarityCfg.pixelClkPolarity = FVID2_EDGE_POL_FALLING;
    vpParams->lcdPolarityCfg.hsPolarity = FVID2_POL_HIGH;
    vpParams->lcdPolarityCfg.vsPolarity = FVID2_POL_HIGH;

    DispApp_configDctrl(appObj);

    if(FVID2_SOK == retVal)
    {
        App_print("Display create complete!!\r\n");
    }

    return;
}

static void DispApp_delete(DispApp_Obj *appObj)
{
    int32_t retVal;
    Dss_DctrlVpParams *vpParams;

    vpParams = &appObj->vpParams;

    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_STOP_VP,
        vpParams,
        NULL);
    if(FVID2_SOK != retVal)
    {
        App_print("VP Stop Failed!!!\r\n");
    }

    if(FVID2_SOK == retVal)
    {
         App_print("Display delete complete!!\r\n");
    }

    return;
}

static int32_t DispApp_configDctrl(DispApp_Obj *appObj)
{
    int32_t retVal = FVID2_SOK;
    Dss_DctrlVpParams *vpParams;
    Dss_DctrlOverlayParams *overlayParams;
#if defined (SOC_AM65XX)
    Dss_DctrlOldiParams *oldiParams;
    oldiParams = &appObj->oldiParams;
    Dss_dctrlOldiParamsInit(oldiParams);
#endif

    vpParams = &appObj->vpParams;
    overlayParams = &appObj->overlayParams;
    Dss_dctrlOverlayParamsInit(overlayParams);

    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_SET_VP_PARAMS,
        vpParams,
        NULL);
    if(retVal != FVID2_SOK)
    {
        App_print("Dctrl Set VP Params IOCTL Failed!!!\r\n");
    }

#if defined (SOC_AM65XX)
    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_SET_OLDI_PARAMS,
        oldiParams,
        NULL);
    if(retVal != FVID2_SOK)
    {
        App_print("DCTRL Set OLDI Params IOCTL Failed!!!\r\n");
    }
#endif

#if defined (SOC_J721E)
    uint32_t regVal2;

    /* Select DPI0 connection */
    regVal2 = CSL_REG32_RD(CSL_DSS0_DISPC_0_COMMON_M_BASE +
                           CSL_DSS_COMMON_M_DISPC_CONNECTIONS);
    CSL_FINS(regVal2,
             DSS_COMMON_M_DISPC_CONNECTIONS_DPI_0_CONN,
             CSL_DSS_COMMON_M_DISPC_CONNECTIONS_DPI_0_CONN_VAL_VP2);
    CSL_REG32_WR(CSL_DSS0_DISPC_0_COMMON_M_BASE +
                 CSL_DSS_COMMON_M_DISPC_CONNECTIONS, regVal2);
#endif

    overlayParams->overlayId = TEST_OVERLAY_ID;
    overlayParams->colorbarEnable = TRUE;

    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_SET_OVERLAY_PARAMS,
        overlayParams,
        NULL);
    if(retVal != FVID2_SOK)
    {
        App_print("DCTRL Set Overlay Params IOCTL Failed!!!\r\n");
    }

    return (retVal);
}

void App_print(const char *format, ...)
{
    va_list     vaArgPtr;
    va_start(vaArgPtr, format);

    DSS_log(format, vaArgPtr);
    va_end(vaArgPtr);

    return;
}
