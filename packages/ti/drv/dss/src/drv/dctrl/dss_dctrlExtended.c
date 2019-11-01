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
 *  \file dss_dctrlExtended.c
 *
 *  \brief File containing the DSS Controller APIs for extended peripherals
 *         like eDP, DSI.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/dss/dss.h>
#include <ti/drv/dss/src/csl/dp/csl_dp.h>
#include <ti/drv/dss/src/csl/dp_sd0801/csl_dp_sd0801.h>
#include <ti/drv/dss/src/drv/common/dss_evtMgr.h>
#include <ti/drv/dss/src/drv/dctrl/dss_dctrlDrv.h>
#include <ti/drv/dss/src/drv/dctrl/dss_dctrlIntr.h>
#include <ti/drv/dss/src/drv/dctrl/dss_dctrlPriv.h>
#include <dss_soc_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

typedef enum
{
    DP_VIDEO_STATE_IDLE      = 0U,
    DP_VIDEO_STATE_REQUESTED = 1U,
    DP_VIDEO_STATE_RUNNING   = 2U,
} Dss_DctrlDisplayPortVideoState;

#define DP_PRIVATE_DATA_SIZE          (131072U)
#define DP_PHY_PRIVATE_DATA_SIZE      (1024U)
#define DP_INTR                       (64U)
#define ADDR_AFE                      (0x05050000)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    uint32_t width;
    uint32_t height;
    uint32_t hfp;
    uint32_t hsw;
    uint32_t hbp;
    uint32_t vfp;
    uint32_t vsw;
    uint32_t vbp;
    uint32_t pclk;
    uint32_t scanMode;
    uint32_t hsyncPol;
    uint32_t vsyncPol;
} Dss_DctrlDisplayPortVideoMode;

typedef struct
{
    DP_Config dpCfg;
    DP_SD0801_Config dpPhyCfg;
    DP_FirmwareImage dpFWImage;
    DP_PrivateData *dpPrivData;
    DP_SD0801_PrivateData *dpPhyPrivData;
    DP_Callbacks dpCbInfo;
    DP_UcpuClock dpClkInfo;
    DP_SourceDeviceCapabilities srcCaps;
    HwiP_Handle intrHandle;
    uint32_t isConnected;
    uint32_t hpdInProgress;
    uint32_t hpdPending;
    Dss_DctrlDisplayPortVideoState videoState;
    Dss_DctrlDisplayPortVideoMode videoMode;
    uint32_t isHpdSupported;
    Dss_DctrlDpHpdCbFxn hpdCbFxn;
    void *hpdCbData;
} Dss_DctrlDisplayPortDrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern uint8_t gDctrlDpDram[DCTRL_DP_DMEM_SIZE];
extern uint8_t gDctrlDpIram[DCTRL_DP_IMEM_SIZE];

static uint8_t gDssDctrlDisplayPortDpPrivateData[DP_PRIVATE_DATA_SIZE];
static uint8_t gDssDctrlDisplayPortDpPhyPrivateData[DP_PHY_PRIVATE_DATA_SIZE];

static Dss_DctrlDisplayPortDrvObj gDssDctrlDisplayPortDrvObj;

Fvid2_ModeInfo gDpStdModeInfo[] = {
    {FVID2_STD_1080P_60,       1920, 1080, FVID2_SF_PROGRESSIVE, 148500, 60,
     88, 148, 44, 4, 36, 5}
};

/* ========================================================================== */
/*                  Internal/Private Function Declarations                    */
/* ========================================================================== */

static int32_t Dss_dctrlDrvInitDPTX(uint32_t isHpdSupported);
static void Dss_dctrlDrvDpIntr(uintptr_t arg);
static int32_t Dss_dctrlDrvDpStartVideo(Dss_DctrlDisplayPortDrvObj *pObj);
static int32_t Dss_dctrlDrvDpStopVideo(Dss_DctrlDisplayPortDrvObj *pObj);
static int32_t Dss_dctrlDrvDetectDp(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Dss_dctrlDrvDisableVideoDP(void)
{
    int32_t retVal = FVID2_SOK;
    uint32_t cookie;
    Dss_DctrlDisplayPortDrvObj *pObj = &gDssDctrlDisplayPortDrvObj;

    cookie = HwiP_disable();

    /*
     * We cannot be IDLE when this function is called.
     * Valid states are REQUESTED || RUNNING
     *
     * If state == REQUESTED, HPD never happened, and we
     * never went to RUNNING. So just go back to IDLE
     *
     * If state == RUNNING, we stop video and go to IDLE
     *
     * There is nothing we can do if stopVideo fails, so
     * dont bother
     */
    if((DP_VIDEO_STATE_REQUESTED == pObj->videoState) ||
    (DP_VIDEO_STATE_RUNNING == pObj->videoState))
    {
        if(DP_VIDEO_STATE_RUNNING == pObj->videoState)
        {
            Dss_dctrlDrvDpStopVideo(pObj);
        }
        pObj->videoState = DP_VIDEO_STATE_IDLE;
    }

    HwiP_restore(cookie);

    return retVal;
}

int32_t Dss_dctrlDrvEnableVideoDP(const Fvid2_ModeInfo *mInfo,
                                  uint32_t hsyncPolarity,
                                  uint32_t vsyncPolarity)
{
    int32_t retVal = FVID2_SOK;
    uint32_t cookie;
    Dss_DctrlDisplayPortDrvObj *pObj = &gDssDctrlDisplayPortDrvObj;
    uint32_t numStdModes, modeCnt;

    if(FALSE == pObj->isHpdSupported)
    {
        retVal = Dss_dctrlDrvDetectDp();
    }

    cookie = HwiP_disable();

    if((FVID2_SOK == retVal) &&
    (FALSE == pObj->isHpdSupported))
    {
        pObj->isConnected = TRUE;
    }

    /*
     * Copy the video mode parameters to be
     * used by subsequent videoStart calls
     *
     * XXX bring in EDID support so that we can validate
     * params passed by app, and when all parameters not
     * provided, make a good guess whether to continue or not
     */
    if(FVID2_STD_CUSTOM != mInfo->standard)
    {
        numStdModes = sizeof (gDpStdModeInfo) / sizeof (Fvid2_ModeInfo);
        for (modeCnt = 0U; modeCnt < numStdModes; modeCnt++)
        {
            if (mInfo->standard == gDpStdModeInfo[modeCnt].standard)
            {
                pObj->videoMode.width    = gDpStdModeInfo[modeCnt].width;
                pObj->videoMode.height   = gDpStdModeInfo[modeCnt].height;
                pObj->videoMode.hfp      = gDpStdModeInfo[modeCnt].hFrontPorch;
                pObj->videoMode.hsw      = gDpStdModeInfo[modeCnt].hSyncLen;
                pObj->videoMode.hbp      = gDpStdModeInfo[modeCnt].hBackPorch;
                pObj->videoMode.vfp      = gDpStdModeInfo[modeCnt].vFrontPorch;
                pObj->videoMode.vsw      = gDpStdModeInfo[modeCnt].vSyncLen;
                pObj->videoMode.vbp      = gDpStdModeInfo[modeCnt].vBackPorch;
                pObj->videoMode.pclk     = gDpStdModeInfo[modeCnt].pixelClock;
                pObj->videoMode.scanMode = gDpStdModeInfo[modeCnt].scanFormat;
		break;
            }
        }
        if (modeCnt == numStdModes)
        {
            retVal = FVID2_EFAIL;
        }
    }
    else
    {
        pObj->videoMode.width    = mInfo->width;
        pObj->videoMode.height   = mInfo->height;
        pObj->videoMode.hfp      = mInfo->hFrontPorch;
        pObj->videoMode.hsw      = mInfo->hSyncLen;
        pObj->videoMode.hbp      = mInfo->hBackPorch;
        pObj->videoMode.vfp      = mInfo->vFrontPorch;
        pObj->videoMode.vsw      = mInfo->vSyncLen;
        pObj->videoMode.vbp      = mInfo->vBackPorch;
        pObj->videoMode.pclk     = mInfo->pixelClock;
        pObj->videoMode.scanMode = mInfo->scanFormat;
    }

    pObj->videoMode.hsyncPol = hsyncPolarity;
    pObj->videoMode.vsyncPol = vsyncPolarity;

    /*
     * We are supposed to be IDLE when this function is called
     *
     * If HPD didnt happen yet, we set to REQUESTED, HDP will
     * start video later if this flag is set, and then set us
     * to RUNNING
     *
     * If HPD happend, and LT was successful, we start video
     * and set to RUNNING
     *
     * If startVideo fails, we stay at REQUESTED. reason:
     * startVideo can fail because the connected monitor
     * does not support the timings provided. So we hope
     * that when another monitor is connected, we are at
     * REQUESTED, and HPD takes us to RUNNING
     *
     * XXX check the timings, match with EDID and print a trace
     * accordingly
     */
    if((FVID2_SOK == retVal) &&
    (DP_VIDEO_STATE_IDLE == pObj->videoState))
    {
        pObj->videoState = DP_VIDEO_STATE_REQUESTED;
        if(TRUE == pObj->isConnected)
        {
            retVal = Dss_dctrlDrvDpStartVideo(pObj);
            if(FVID2_SOK == retVal)
            {
                pObj->videoState = DP_VIDEO_STATE_RUNNING;
            }
        }
    }

    HwiP_restore(cookie);

    return retVal;
}

int32_t Dss_dctrlDrvInitDp(uint32_t isHpdSupported)
{
    uint32_t retVal = FVID2_SOK;

    /*
     * XXX push this things to some CSL kind of functions
     */

    /* HPD Pin Mux */
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_PADCONFIG0 + 0x1c4, 0x00040005);

    /* Select EDP PHY CLK source */
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_EDP_PHY0_CLKSEL, 0x0);

    /* Set DPTX_SRC_CFG, 0:vif0_en, 1:vif_1_en, 2:vif_2_en, 3: vif_3_en, 4: vif_0_sel, TBD */
    CSL_REG32_WR(CSL_DSS_EDP0_INTG_CFG_VP_BASE + CSL_DPTX_DPTX_SRC_CFG, 0x1F);

    /* WIZ PHY */
    CSL_REG32_WR(ADDR_AFE + 0x408, 0x30000000);
    CSL_REG32_WR(ADDR_AFE + 0x40c, 0x39000000);
    CSL_REG32_WR(ADDR_AFE + 0x480, 0x70000000);
    CSL_REG32_WR(ADDR_AFE + 0x4c0, 0x80000000);
    CSL_REG32_WR(ADDR_AFE + 0x500, 0x80000000);
    CSL_REG32_WR(ADDR_AFE + 0x540, 0x80000000);
    CSL_REG32_WR(ADDR_AFE + 0x484, 0x10001   );
    CSL_REG32_WR(ADDR_AFE + 0x4c4, 0x10001   );
    CSL_REG32_WR(ADDR_AFE + 0x504, 0x10001   );
    CSL_REG32_WR(ADDR_AFE + 0x544, 0x10001   );

    retVal = Dss_dctrlDrvInitDPTX(isHpdSupported);

    return retVal;
}

int32_t Dss_dctrlDrvRegisterHpdCb(const Dss_DctrlDpHpdCbParams *cbParams)
{
    int32_t retVal = FVID2_SOK;
    uint32_t cookie;
    Dss_DctrlDisplayPortDrvObj *pObj = &gDssDctrlDisplayPortDrvObj;

    if(NULL == cbParams)
    {
        retVal = FVID2_EFAIL;
    }

    cookie = HwiP_disable();

    if(FVID2_SOK == retVal)
    {
        pObj->hpdCbFxn = cbParams->hpdCbFxn;
        pObj->hpdCbData = cbParams->appData;
    }

    if((NULL != pObj->hpdCbFxn) &&
    (TRUE == pObj->hpdPending))
    {
        pObj->hpdPending = FALSE;
        pObj->hpdInProgress = TRUE;
        pObj->hpdCbFxn(TRUE, pObj->hpdCbData);
    }

    HwiP_restore(cookie);

    return retVal;
}

int32_t Dss_dctrlDrvProcessHpdDp(uint32_t hpdState)
{
    uint32_t retVal = FVID2_SOK, dpApiRet;
    uint32_t cookie;
    uint32_t isConnected = FALSE;
    Dss_DctrlDisplayPortDrvObj *pObj = &gDssDctrlDisplayPortDrvObj;
    DP_TrainingStatus trainingResult;

    if(TRUE == hpdState)
    {
        dpApiRet = DP_LinkTraining(pObj->dpPrivData, &trainingResult);
        if ((CDN_EOK == dpApiRet) &&
        (trainingResult == DP_LT_OK))
        {
            isConnected = TRUE;
        }
    }

    /*
     * It is a little bit of unknown ATM whether all these
     * MBOX transactions to enable DP video takes sufficiently
     * insignificant time so that they can be safely called
     * inside a HWI disable/restore block.
     *
     * Keep them as is for the time being. We will see if
     * anyone comes back reporting issues!
     */
    cookie = HwiP_disable();
    pObj->isConnected = isConnected;

    /*
     * If we just connected, check if REQUESTED or RUNNING
     *
     * If RUNNING, then start video
     * If startVideo fails, go back to REQUESTED, so that we
     * do not do anything unexpected when stopVideo is called
     *
     * If REQUESTED, then start video, and if it passes,
     * set to RUNNING
     *
     * If we just disconnected, and we were previously RUNNING,
     * we set to REQUESTED and stop video
     */
    if(TRUE == pObj->isConnected)
    {
        if((DP_VIDEO_STATE_REQUESTED == pObj->videoState) ||
        (DP_VIDEO_STATE_RUNNING == pObj->videoState))
        {
            retVal = Dss_dctrlDrvDpStartVideo(pObj);
            if(FVID2_SOK == retVal)
            {
                if(DP_VIDEO_STATE_REQUESTED == pObj->videoState)
                {
                    pObj->videoState = DP_VIDEO_STATE_RUNNING;
                }
            }
            else
            {
                if(DP_VIDEO_STATE_RUNNING == pObj->videoState)
                {
                    pObj->videoState = DP_VIDEO_STATE_REQUESTED;
                }
            }
        }
    }
    else
    {
        if(DP_VIDEO_STATE_RUNNING == pObj->videoState)
        {
            Dss_dctrlDrvDpStopVideo(pObj);
            pObj->videoState = DP_VIDEO_STATE_REQUESTED;
        }
    }

    pObj->hpdInProgress = FALSE;
    HwiP_restore(cookie);

    return retVal;
}

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static int32_t Dss_dctrlDrvDetectDp()
{
    int32_t retVal = FVID2_SOK, dpApiRet;
    bool hpdState;
    DP_TrainingStatus trainingResult;
    Dss_DctrlDisplayPortDrvObj *pObj = &gDssDctrlDisplayPortDrvObj;

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_GetHpdStatus(pObj->dpPrivData, &hpdState);
        if((CDN_EOK != dpApiRet) ||
        (true != hpdState))
        {
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_LinkTraining(pObj->dpPrivData, &trainingResult);
        if ((CDN_EOK != dpApiRet) ||
        (trainingResult != DP_LT_OK))
        {
            retVal = FVID2_EFAIL;
        }
    }

    return retVal;
}

static void Dss_dctrlDrvDpIntr(uintptr_t arg)
{
    Dss_DctrlDisplayPortDrvObj *pObj = (Dss_DctrlDisplayPortDrvObj *)arg;
    uint32_t evt;
    uint32_t retVal;
    uint8_t hpdEvents = 0u;

    retVal = DP_GetEvent(pObj->dpPrivData, &evt);

    if((CDN_EOK == retVal) &&
    (0 != (evt & DP_TX_HPD_EVENT)))
    {
        retVal = DP_ReadHpdEvent(pObj->dpPrivData, &hpdEvents);
    }

    if((CDN_EOK == retVal) &&
    (FALSE == pObj->hpdInProgress))
    {

        if((0U != (hpdEvents & DP_HPD_STATE)) &&
        (0U != (hpdEvents & DP_HPD_TO_HIGH)))
        {
            if(NULL != pObj->hpdCbFxn)
            {
                pObj->hpdInProgress = TRUE;
                pObj->hpdCbFxn(TRUE, pObj->hpdCbData);
            }
            else
            {
                pObj->hpdPending = TRUE;
            }
        }
        else if((0U == (hpdEvents & DP_HPD_STATE)) &&
        (0U != (hpdEvents & DP_HPD_TO_LOW)))
        {
            if(NULL != pObj->hpdCbFxn)
            {
                pObj->hpdInProgress = TRUE;
                pObj->hpdCbFxn(FALSE, pObj->hpdCbData);
            }
            else
            {
                pObj->hpdPending = FALSE;
            }
        }
    }
}

static int32_t Dss_dctrlDrvDpStopVideo(Dss_DctrlDisplayPortDrvObj *pObj)
{
    /* XXX framer off */
    return FVID2_SOK;
}

static int32_t Dss_dctrlDrvDpStartVideo(Dss_DctrlDisplayPortDrvObj *pObj)
{
    int32_t retVal = FVID2_SOK;
    int32_t dpApiRet = CDN_EOK;
    DP_AudioVideoClkCfg clkCfg;
    DP_VideoParameters videoParams;

    clkCfg.videoClockEnable   = true;
    clkCfg.audioClockEnable   = true;
    clkCfg.pktDataClockEnable = true;

    videoParams.bitsPerSubpixel    = 8;
    videoParams.pxEncFormat        = DP_PXENC_PXL_RGB;
    videoParams.stereoVidAttr      = DP_STEREO_VIDEO_LEFT;
    videoParams.btType             = DP_BT_601;
    videoParams.forceMiscIgnoreBit = false;
    videoParams.alignment          = DP_ALIGN_MSB;
    videoParams.dscEnable          = 0;

    /* fill the ones the driver does not care about */
    videoParams.vicParams.vic   = 0U;
    videoParams.vicParams.hFreq = 0.0;
    videoParams.vicParams.vFreq = 0,0;
    videoParams.vicParams.vicR  = 0;
    videoParams.vicParams.vicPR = 0;

    /* and, now the actually important ones */
    videoParams.vicParams.hActive       = pObj->videoMode.width;
    videoParams.vicParams.hSync         = pObj->videoMode.hsw;
    videoParams.vicParams.hFrontPorch   = pObj->videoMode.hfp;
    videoParams.vicParams.hBackPorch    = pObj->videoMode.hbp;
    videoParams.vicParams.hBlank        = pObj->videoMode.hbp + pObj->videoMode.hfp + pObj->videoMode.hsw;
    videoParams.vicParams.hTotal        = videoParams.vicParams.hBlank + videoParams.vicParams.hActive;
    videoParams.vicParams.vActive       = pObj->videoMode.height;
    videoParams.vicParams.vSync         = pObj->videoMode.vsw;
    videoParams.vicParams.vFrontPorch   = pObj->videoMode.vfp;
    videoParams.vicParams.vBackPorch    = pObj->videoMode.vbp;
    videoParams.vicParams.vBlank        = pObj->videoMode.vbp + pObj->videoMode.vfp + pObj->videoMode.vsw;
    videoParams.vicParams.vTotal        = videoParams.vicParams.vBlank + videoParams.vicParams.vActive;
    videoParams.vicParams.pxlFreq       = pObj->videoMode.pclk / (float64_t)1000;
    videoParams.vicParams.scanMode      = ((FVID2_SF_PROGRESSIVE == pObj->videoMode.scanMode) ? DP_SM_PROGRESSIVE : DP_SM_INTERLACED);
    videoParams.vicParams.hSyncPolarity = ((FVID2_POL_HIGH == pObj->videoMode.hsyncPol) ? DP_SP_ACTIVE_HIGH : DP_SP_ACTIVE_LOW);
    videoParams.vicParams.vSyncPolarity = ((FVID2_POL_HIGH == pObj->videoMode.vsyncPol) ? DP_SP_ACTIVE_HIGH : DP_SP_ACTIVE_LOW);

    if(CDN_EOK == dpApiRet)
    {
        dpApiRet = DP_SetVic(pObj->dpPrivData, 0, &videoParams);
        if(CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SetVic\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(CDN_EOK == dpApiRet)
    {
        dpApiRet = DP_SetAudioVideoClkCfg(pObj->dpPrivData, 0, &clkCfg);
        if(CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SetAudioVideoClkCfg\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(CDN_EOK == dpApiRet)
    {
        dpApiRet = DP_SetFramerEnable(pObj->dpPrivData, true);
        if(CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SetFramerEnable\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(CDN_EOK == dpApiRet)
    {
        dpApiRet = DP_SetVideoSst(pObj->dpPrivData, true);
        if(CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SetVideoSst\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    return retVal;
}

static int32_t Dss_dctrlDrvInitDPTX(uint32_t isHpdSupported)
{
    Dss_DctrlDisplayPortDrvObj *pObj;
    uint32_t memReqDp, memReqDpPhy;
    HwiP_Params hwiParams;
    uint32_t dpApiRet;
    int32_t retVal = FVID2_SOK;

    pObj = &gDssDctrlDisplayPortDrvObj;

    pObj->dpCfg.regBase      = (struct MHDP_ApbRegs_s *)CSL_DSS_EDP0_V2A_CORE_VP_REGS_APB_BASE;
    pObj->dpCfg.regBaseSapb  = (struct MHDP_ApbRegs_s *)CSL_DSS_EDP0_V2A_S_CORE_VP_REGS_SAPB_BASE;

    pObj->dpPhyCfg.regBase   = (uint32_t*)CSL_SERDES_10G0_BASE;
    pObj->dpPhyCfg.regBaseDp = (struct DP_Regs_s *)CSL_DSS_EDP0_V2A_CORE_VP_REGS_APB_BASE;

    pObj->dpFWImage.iMem     = gDctrlDpIram;
    pObj->dpFWImage.iMemSize = DCTRL_DP_IMEM_SIZE;
    pObj->dpFWImage.dMem     = gDctrlDpDram;
    pObj->dpFWImage.dMemSize = DCTRL_DP_DMEM_SIZE;

    pObj->dpCbInfo.event = NULL;
    pObj->dpClkInfo.mhz = 125;

    pObj->srcCaps.maxLinkRate = DP_LINK_RATE_8_10;
    pObj->srcCaps.laneCount = 4;
    pObj->srcCaps.ssc = false;
    pObj->srcCaps.scramblerDisable = false;
    pObj->srcCaps.tps3 = true;
    pObj->srcCaps.tps4 = true;
    pObj->srcCaps.fastLinkTraining = false;
    pObj->srcCaps.maxVoltageSwing = 3;
    pObj->srcCaps.maxPreemphasis = 3;
    pObj->srcCaps.forceVoltageSwing = false;
    pObj->srcCaps.forcePreemphasis = false;
    pObj->srcCaps.laneMapping = DP_LANE_MAPPING_SINGLE_REGULAR;
    pObj->srcCaps.controllersPerPhy = DP_SINGLE_CONTROLLER;

    pObj->isConnected = FALSE;
    pObj->videoState = DP_VIDEO_STATE_IDLE;

    pObj->isHpdSupported = isHpdSupported;

    pObj->hpdPending = FALSE;
    pObj->hpdInProgress = FALSE;

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_Probe(&pObj->dpCfg, &memReqDp);
        if((CDN_EOK != dpApiRet) ||
        (memReqDp > DP_PRIVATE_DATA_SIZE))
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_Probe\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_SD0801_Probe(&pObj->dpPhyCfg, &memReqDpPhy);
        if ((CDN_EOK != dpApiRet) ||
        (memReqDpPhy > DP_PHY_PRIVATE_DATA_SIZE))
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SD0801_Probe\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        pObj->dpPrivData = (DP_PrivateData *)(&gDssDctrlDisplayPortDpPrivateData[0]);
        pObj->dpPhyPrivData = (DP_SD0801_PrivateData *)(&gDssDctrlDisplayPortDpPhyPrivateData[0]);

        dpApiRet = DP_Init(pObj->dpPrivData, &pObj->dpCfg, &pObj->dpCbInfo);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_Init\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_SD0801_Init(pObj->dpPhyPrivData, &pObj->dpPhyCfg);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SD0801_Init\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_SetPhyPd(pObj->dpPrivData, pObj->dpPhyPrivData);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SetPhyPd\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_LoadFirmware(pObj->dpPrivData, &pObj->dpFWImage);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_LoadFirmware\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_SetClock(pObj->dpPrivData, &pObj->dpClkInfo);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SetClock\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
       dpApiRet = DP_StartUcpu(pObj->dpPrivData);
       if(CDN_EOK != dpApiRet)
       {
            GT_0trace(DssTrace, GT_ERR, "error : DP_StartUcpu\r\n");
            retVal = FVID2_EFAIL;
       }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_WaitAlive(pObj->dpPrivData);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_WaitAlive\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        uint8_t mainCtrlResp;

        dpApiRet = DP_MainControl(pObj->dpPrivData, true, &mainCtrlResp);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_MainControl\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_ConfigurePhyAuxCtrl(pObj->dpPrivData);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_ConfigurePhyAuxCtrl\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_ConfigurePhyStartUp(pObj->dpPrivData,
                pObj->srcCaps.laneCount,
                pObj->srcCaps.maxLinkRate);
        if(CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_ConfigurePhyStartUp\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if(FVID2_SOK == retVal)
    {
        dpApiRet = DP_SetSourceCapabilities(pObj->dpPrivData, &pObj->srcCaps);
        if (CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_SetSourceCapabilities\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if((FVID2_SOK == retVal) &&
    (TRUE == pObj->isHpdSupported))
    {
        dpApiRet = DP_Start(pObj->dpPrivData);
        if(CDN_EOK != dpApiRet)
        {
            GT_0trace(DssTrace, GT_ERR, "error : DP_Start\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if((FVID2_SOK == retVal) &&
    (TRUE == pObj->isHpdSupported))
    {
        HwiP_clearInterrupt(DP_INTR);

        HwiP_Params_init(&hwiParams);
        hwiParams.arg = (uintptr_t) pObj;

        pObj->intrHandle = HwiP_create(DP_INTR,
                &Dss_dctrlDrvDpIntr,
                &hwiParams);

        if(NULL == pObj->intrHandle)
        {
            GT_0trace(DssTrace,
                      GT_ERR,
                      "Cannot Register IRQ");
            retVal = FVID2_EALLOC;
        }
    }

    return retVal;
}
