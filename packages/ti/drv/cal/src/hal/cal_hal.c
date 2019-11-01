/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 *  \file cal_hal.c
 *
 *  \brief This file defines all abstractions for CAL module.
 *  This abstraction will support multiple instances of PPI, CSI. Operates 2
 *  modes primarily. memory to memory mode and capture mode.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <ti/csl/soc.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/cslr_cal.h>

#include <ti/drv/cal/cal.h>
#include <ti/drv/cal/src/hal/cal_hal.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define SMA_SW_CSI2_X0_UP_MASK             (0x00000001U)
#define SMA_SW_CSI2_Y0_UP_MASK             (0x00000002U)

#define SMA_SW_CSI2_X1_UP_MASK             (0x00000004U)
#define SMA_SW_CSI2_Y1_UP_MASK             (0x00000008U)

#define SMA_SW_CSI2_X2_UP_MASK             (0x00000010U)
#define SMA_SW_CSI2_Y2_UP_MASK             (0x00000020U)

#define SMA_SW_CSI2_X3_UP_MASK             (0x00000040U)
#define SMA_SW_CSI2_Y3_UP_MASK             (0x00000080U)

#define SMA_SW_CSI2_X4_UP_MASK             (0x00000100U)
#define SMA_SW_CSI2_Y4_UP_MASK             (0x00000200U)

#define SMA_SW_CSI2_X0_DW_MASK             (0x00000400U)
#define SMA_SW_CSI2_Y0_DW_MASK             (0x00000800U)

#define SMA_SW_CSI2_X1_DW_MASK             (0x00001000U)
#define SMA_SW_CSI2_Y1_DW_MASK             (0x00002000U)

#define SMA_SW_CSI2_X2_DW_MASK             (0x00004000U)
#define SMA_SW_CSI2_Y2_DW_MASK             (0x00008000U)

#define SMA_SW_CSI2_X3_DW_MASK             (0x00010000U)
#define SMA_SW_CSI2_Y3_DW_MASK             (0x00020000U)

#define SMA_SW_CSI2_X4_DW_MASK             (0x00040000U)
#define SMA_SW_CSI2_Y4_DW_MASK             (0x00080000U)

/**< Bit 2 set, used to compute register values for lanes enabled. */
#define CAL_BIT_2_SET       ((uint32_t)0x4U)

#if (CAL_CAPT_MAX_CMPLXIO_INST != CSL_CAL_CMPLXIO_CNT)
        #error "Number of complex IO's dont match"
#endif

/**< If PPI is enabled before DMA writes are, there is possibility that 1st
        frame received will be written to address 0x0.
        As Write DMA address is latched with a frame start event.
        If PPI is enabled, before any buffer are primed, the write address is
        not yet updated. 0x0 being reset value of write DMA address, 1st
        frame would be written there. */
#define ENABLE_PPI_WHEN_STARTING

/**< Functional PHY clock is expected to be 96 MHz
        Period of PHY clock in nS is 10.41666
        ((1/<PHY Clock) / 1e-9) */
#define DPHY_FUNCTIONAL_CLK_PERIOD  (10.41666)


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  struct Cal_HalInstObj
 *  \brief Describes a instance of Cal configuration.
 */
typedef struct Cal_HalInstObj
{
    uint32_t           instId;
    /**< Instance ID */
    uint32_t           baseAddr;
    /**< Base address of the CAL module */
    uint32_t           phy0BaseAddress;
    /**< If this CAL instance has an associated PHY, this will hold a non-null
     *      value */
    uint32_t           phy1BaseAddress;
    /**< If this CAL instance has an associated PHY, this will hold a non-null
     *      value */
    uint32_t           isInited;
    /**< Flag to indicate successful initialization */
    uint32_t           openCnt;
    /**< Number of times the hal is opened. */
    Cal_HalInstCfg_t instCfg;
    /**< Instance configuration */
} Cal_HalInstObj_t;

/**
 *  struct Cal_HalObj
 *  \brief Describes a cal object.
 */
typedef struct Cal_HalObj
{
    Cal_HalMode_t     mode;
    /**< Mode */
    Cal_HalInstObj_t *pInstObj;
    /**< Pointer to instance specific config */
} Cal_HalObj_t;

/* ========================================================================== */
/*                           Constants                                        */
/* ========================================================================== */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   Resets the CAL instance
 *
 * \param   baseAddr Base address of the CAL instance to be reset.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalReset(uint32_t baseAddr);

/**
 * \brief   Configure instance LANE config and frame mode of operation. Note
 *              that the low level port is enabled for reception.
 *
 * \param   baseAddr Base address of the CAL.
 * \param   pCfg     Valid lane and frame mode configurations
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalCfgInstLanePpiCfg(uint32_t            baseAddr,
                                    Cal_HalInstCfg_t *pCfg);

/**
 * \brief   Resets CAL / PHY and applies instance specific config for capture
 *              Updated DMA read specifics for other modes.
 *
 * \param   hndl    Handle returned when opened.
 * \param   cfg     Valid configuration for mode specified in open.
 * \param   mode    Mode as specified during open.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalResetApplyInstConfig(const Cal_HalObj_t *pHndl,
                                       Cal_HalInstCfg_t   *pCfg,
                                       Cal_HalMode_t       mode,
                                       uint32_t              enPhy);
/**
 * \brief   Applies supplied configurations, for all mode & all blocks.
 *
 * \param   hndl    Handle returned when opened.
 * \param   cfg     Valid configuration for mode specified in open.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetCfg(const Cal_HalObj_t *hndl, const Cal_HalCfg_t *cfg);

/**
 * \brief   Applies configurations required for m2m mode.
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for m2m.
 * \param   mode    Indicates if in capture / m2m mode.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetCfgForMode(const Cal_HalInstObj_t *pInst,
                                const Cal_HalCfg_t     *cfg,
                                Cal_HalMode_t           mode);

/**
 * \brief   Applies CSI2 Virtual processing configurations.
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for Virtual Channels of CSI2.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalCsi2VcCfg(const Cal_HalInstObj_t   *pInst,
                            const Cal_HalCsi2VcCfg_t *cfg,
                            uint32_t                    cportId);

/**
 * \brief   Applies pixel processing configurations.
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for pixel processing.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetPixProcCfg(const Cal_HalInstObj_t *pInst,
                                const Cal_PixProc_t *cfg,
                                uint32_t                  cportId);

/**
 * \brief   Applies BYS Out port configurations.
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for BYS Out port.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetBysOutCfg(const Cal_HalInstObj_t *pInst,
                               const Cal_BysOut_t  *cfg,
                               uint32_t                  cportId);

/**
 * \brief   Applies VPORT out configurations.
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for VPORT Out.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetVportCfg(const Cal_HalInstObj_t *pInst,
                              const Cal_VPort_t   *cfg,
                              uint32_t                  cportId);

/**
 * \brief   Applies DMA Read configurations.
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for DMA Read.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetRdDmaCfg(const Cal_HalInstObj_t  *pInst,
                              const Cal_HalRdDmaCfg_t *cfg,
                              uint32_t                   cportId);

/**
 * \brief   Applies DMA Read frame size
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for frame config.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetRdDmaFrameCfg(const Cal_HalInstObj_t *pInst,
                                   const Fvid2_Format       *cfg);

/**
 * \brief   Applies DMA Write configurations.
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for DMA Write.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetWrDmaCfg(const Cal_HalInstObj_t  *pInst,
                              const Cal_HalWrDmaCfg_t *cfg,
                              uint32_t                   cportId);

/**
 * \brief   Sets the DMA mode, expected to be used to enable/disable DMA writes
 *              For all the DMA write contexts that have been configured, the
 *              supplied mode is applied
 *
 * \param   pInstObj   Pointer to instance object.
 * \param   pDmaCfg    Pointer to DMA config
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t calSetWrDmaMode(const Cal_HalInstObj_t  *pInstObj,
                               const Cal_HalDmaVcCfg_t *pDmaCfg);

/**
 * \brief   Sets the DMA configurations such as burst size, mode, etc...
 *
 * \param   pInstObj   Pointer to instance object.
 * \param   pDmaCfg    Pointer to DMA config
 *
 * \return  FVID2_SOK
 *
 **/
static int32_t CalCfgInstWrDma(
    uint32_t baseAddr, const Cal_HalInstCfg_t *pCfg);

/**
 * \brief   Configure the interrupt generation on reception of specified line
 *          of video data.
 *
 * \param   pInstObj   Pointer to instance object.
 * \param   plnEvtCfg  Pointer to line event configuration
 *
 * \return  FVID2_SOK
 *
 **/
static int32_t CalSetLineEventCfg(const Cal_HalInstObj_t  *pInstObj,
                                  const Cal_HalLineEventCfg_t *plnEvtCfg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#define CAL_HAL_INST_OBJ_DEFAULTS ( 0x0,                /* instId */ \
                                    0x0,                /* baseAddr */ \
                                    0x0,                /* phy0BaseAddress */ \
                                    0x0,                /* phy1BaseAddress */ \
                                    (uint32_t) FALSE,   /* isInited */ \
                                    0x0,                /* openCnt */ \
                                    )
static Cal_HalInstObj_t gCalInstObj[CSL_CAL_PER_CNT] =
{
    {
        0x0,                /* instId */
        0x0,                /* baseAddr */
        0x0,                /* phy0BaseAddress */
        0x0,                /* phy1BaseAddress */
        (uint32_t) FALSE,   /* isInited */
        0x0                 /* openCnt */
    }
#if (CSL_CAL_PER_CNT > 1U)
    ,
    {
        0x0,
        0x0,
        (uint32_t)FALSE,
        0x0,
        0x0
    }
#endif
/* Currently we do not have SoC with more than 2 CAL instances */
};
/**< Instance specific config */

static Cal_HalObj_t     gCalObj \
    [CSL_CAL_PER_CNT][CAL_HAL_OPEN_NUM];
/**< Handle / open specific config */

static Cal_HalInstCfg_t gCalInstDefaults;
/**< Default instance config */

#if defined (SOC_TDA2EX)
static Cal_HalInstParams_t gCalHalInstParams [CSL_CAL_PER_CNT] = {{0,
                                ISS_CALA_BASE_REGISTER,
                                ISS_CALA_CAMERARX_CORE_0_BASE_REGISTER,
                                ISS_CALA_CAMERARX_CORE_1_BASE_REGISTER, NULL}};
#endif
#if defined (SOC_AM65XX)
static Cal_HalInstParams_t gCalHalInstParams [CSL_CAL_PER_CNT] = {{0,
                                CSL_CAL0_BASE,
                                (CSL_CAL0_BASE + CSL_CAL_SCP_SCP_PHY_A(0)),
                                0U, 0U}};
#endif
/**< Cal Platform data */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Cal_halInit(uint32_t numInst,
                          const Cal_HalInstParams_t *initPrms, void *arg)
{
    uint32_t i, j;
    uint32_t isInited = (uint32_t)FALSE;

    GT_assert(CalTrace, (CSL_CAL_PER_CNT >= numInst));
    GT_assert(CalTrace, (NULL != initPrms));
    GT_assert(CalTrace, (0x0 != initPrms->baseAddress));

    for(i = 0; i < CSL_CAL_PER_CNT; i++)
    {
        if((uint32_t)FALSE != gCalInstObj[i].isInited)
        {
            /* Has been initialized earlier. Exit now */
            isInited = (uint32_t)TRUE;
            /* This is MISRA C violation, for having multiple returns.
             *  This should not cause any issues. Please ignore */
        }
    }

    if((uint32_t)FALSE == isInited)
    {
        /* Do not require to initialize gCalObj, we rely primarily
         * on gCalInstObj
         *  The association between gCalInstObj & gCalObj is done
         * further down */
        /* . Set defaults */
        /* Indicates the urgency of real time traffic using the
         * number of contexts
         *  currently in use by the write DMA.
         *  00: SAFE (n<MFLAGL)
         *  01: VULNERABLE (MFLAGL<=n<MFLAGH)
         *  11: ENDANGERED (MFLAGH<=n)
         *  Assuming 4 write DMA contexts, n = 4 */

        gCalInstDefaults.mFlagH = 10U;
        gCalInstDefaults.mFlagL = 6U;
        /* Assuming CAL receives via a sensor */
        gCalInstDefaults.rdDmaStall   = (uint32_t)TRUE;
        gCalInstDefaults.pwrScpClk    = (uint32_t)FALSE;
        gCalInstDefaults.dmaBurstSize = CSL_CAL_C2VID_CTRL_BURSTSIZE_BURST128;
        gCalInstDefaults.tagCnt       = 15U;
        gCalInstDefaults.postedWrites = (uint32_t)FALSE;

        for(i = 0; i < CSL_CAL_CMPLXIO_CNT; i++)
        {
            gCalInstDefaults.csi2PhyClock[i] = 400U;

            gCalInstDefaults.numCmplxIoInst = CSL_CAL_CMPLXIO_CNT;
            gCalInstDefaults.cmplxIoCfg[i].enable = (uint32_t)TRUE;
            gCalInstDefaults.cmplxIoCfg[i].clockLane.pol      = (uint32_t)FALSE;
            gCalInstDefaults.cmplxIoCfg[i].clockLane.position = 5U;
            gCalInstDefaults.cmplxIoCfg[i].data1Lane.pol      = (uint32_t)FALSE;
            gCalInstDefaults.cmplxIoCfg[i].data1Lane.position = 1U;
            gCalInstDefaults.cmplxIoCfg[i].data2Lane.pol      = (uint32_t)FALSE;
            gCalInstDefaults.cmplxIoCfg[i].data2Lane.position = 2U;
            gCalInstDefaults.cmplxIoCfg[i].data3Lane.pol      = (uint32_t)FALSE;
            gCalInstDefaults.cmplxIoCfg[i].data3Lane.position = 3U;
            gCalInstDefaults.cmplxIoCfg[i].data4Lane.pol      = (uint32_t)FALSE;
            gCalInstDefaults.cmplxIoCfg[i].data4Lane.position = 4U;
            gCalInstDefaults.cmplxIoCfg[i].pwrAuto            = (uint32_t)FALSE;
            /* Always
             * powered
             * up
             */
        }
        for(i = 0; i < CSL_CAL_PPI_CNT; i++)
        {
            gCalInstDefaults.numPpiInst         = CSL_CAL_PPI_CNT;
            gCalInstDefaults.ppiCfg[i].enable   = (uint32_t)FALSE;
            gCalInstDefaults.ppiCfg[i].instance = 0x0U;
            gCalInstDefaults.ppiCfg[i].frame    = (uint32_t)TRUE;
            gCalInstDefaults.ppiCfg[i].ecc      = (uint32_t)FALSE;

            /* Careful while updating these, are used to determine
                the stop state
                detection.
                Please ignore force_rx_mode_0I01 - will be removed */
            gCalInstDefaults.ppiCfg[i].csi2Cfg.force_rx_mode_0I01     = 1U;
            gCalInstDefaults.ppiCfg[i].csi2Cfg.stop_state_x16_I01     = 1U;
            gCalInstDefaults.ppiCfg[i].csi2Cfg.stop_state_x4_I01      = 0U;
            gCalInstDefaults.ppiCfg[i].csi2Cfg.stop_state_counter_I01 = 0x197U;
        }

        for(i = 0; ((i < numInst) && (i < CSL_CAL_PER_CNT)); i++)
        {
            gCalInstObj[i].instId = i;
            GT_assert(CalTrace, (0x0 != initPrms->baseAddress));
            gCalInstObj[i].baseAddr       = initPrms->baseAddress;
            gCalInstObj[i].phy0BaseAddress = initPrms->phy0BaseAddress;
            gCalInstObj[i].phy1BaseAddress = initPrms->phy1BaseAddress;
            gCalInstObj[i].openCnt        = 0U;
            gCalInstObj[i].isInited       = (uint32_t)TRUE;
            memcpy((void *) & gCalInstObj[i].instCfg,
                            (const void *) &gCalInstDefaults,
                            sizeof(Cal_HalInstCfg_t));
            for(j = 0; j < CAL_HAL_OPEN_NUM; j++)
            {
                gCalObj[i][j].pInstObj = &gCalInstObj[i];
            }

            initPrms++;
        }
    }

    return FVID2_SOK;
}

/**************************Function Separator**********************************/

int32_t Cal_halDeInit(void *arg)
{
    uint32_t i;

    for(i = 0; i < CSL_CAL_PER_CNT; i++)
    {
        GT_assert(CalTrace, (0x0 == gCalInstObj[i].openCnt));
        memset(&gCalInstObj[i], 0U, sizeof(Cal_HalInstObj_t));
    }
    /* Do not require any updates to gCalObj, the close will take care */
    return FVID2_SOK;
}

/**************************Function Separator**********************************/

Cal_HalHandle Cal_halOpen(const Cal_HalOpenParams_t *openPrms,
                                void                        *arg)
{
    Cal_HalObj_t *hndl = NULL;
    int32_t         rtnVal;
    uint32_t        openCnt;

    rtnVal = FVID2_SOK;
    if(NULL == openPrms)
    {
        rtnVal = FVID2_EBADARGS;
    }

    if((FVID2_SOK == rtnVal) && (CSL_CAL_PER_CNT <= openPrms->instId))
    {
        rtnVal = FVID2_EBADARGS;
    }
    if((FVID2_SOK == rtnVal) &&
       ((CAL_HAL_MODE_MIN >= openPrms->mode) ||
        (CAL_HAL_MODE_MAX <= openPrms->mode)))
    {
        rtnVal = FVID2_EBADARGS;
    }
    if(FVID2_SOK == rtnVal)
    {
        openCnt = gCalObj[openPrms->instId][0].pInstObj->openCnt;
        if(CAL_HAL_OPEN_NUM <= openCnt)
        {
            rtnVal = FVID2_EALLOC;
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        hndl = &gCalObj[openPrms->instId][openCnt];
        if(NULL == hndl->pInstObj)
        {
            /* Not initialized/ */
            rtnVal = FVID2_EBADARGS;
        }
    }
    if(FVID2_SOK == rtnVal)
    {
        hndl->pInstObj->openCnt++;

        rtnVal = CalResetApplyInstConfig(hndl, &hndl->pInstObj->instCfg,
                                         openPrms->mode, (uint32_t)FALSE);
        if(FVID2_SOK == rtnVal)
        {
            hndl->mode = openPrms->mode;
        }
    }

    return ((Cal_HalHandle) hndl);
}

/**************************Function Separator**********************************/

int32_t Cal_halClose(Cal_HalHandle handle, void *arg)
{
    uint32_t          instance, i;
    int32_t           rtnVal = FVID2_SOK;
    volatile uint32_t reg, timeOut;
    Cal_HalObj_t   *hndl = (Cal_HalObj_t *) handle;

    if(NULL == hndl)
    {
        rtnVal = FVID2_EBADARGS;
    }
    else
    {
        GT_assert(CalTrace, (NULL != hndl->pInstObj));
    }

    if(FVID2_SOK == rtnVal)
    {
        if(0x0 == hndl->pInstObj->openCnt)
        {
            rtnVal = FVID2_EBADARGS;
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        if(CAL_HAL_MODE_CAPTURE == hndl->mode)
        {
            rtnVal = FVID2_SOK;
            for(i = 0; ((FVID2_SOK == rtnVal) &&
                        (i < hndl->pInstObj->instCfg.numPpiInst)); i++)
            {
                if((uint32_t)FALSE != hndl->pInstObj->instCfg.ppiCfg[i].enable)
                {
                    instance = hndl->pInstObj->instCfg.ppiCfg[i].instance;
                    if(CSL_CAL_PPI_CNT < instance)
                    {
                        rtnVal = FVID2_EBADARGS;
                        break;
                    }
#ifndef ENABLE_PPI_WHEN_STARTING
                    if(1U == hndl->pInstObj->openCnt)
                    {
                        /* Last instance of open, disable PPI */
                        reg = HW_RD_REG32(hndl->pInstObj->baseAddr +
                                          CSL_CAL_C2PPI_CSI2_PPI_CTRL(instance));
                        reg &= ~CSL_CAL_C2PPI_CSI2_PPI_CTRL_IF_EN_MASK;
                        HW_WR_REG32(hndl->pInstObj->baseAddr +
                                    CSL_CAL_C2PPI_CSI2_PPI_CTRL(instance), reg);
                        rtnVal = FVID2_SOK;
                    }
#endif              /* ENABLE_PPI_WHEN_STARTING */
                }
            }
            for(i = 0; i < CSL_CAL_CMPLXIO_CNT; i++)
            {
                if(1U == hndl->pInstObj->openCnt)
                {
                    reg = HW_RD_REG32(hndl->pInstObj->baseAddr +
                                      CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(i));
                    reg &= ~(CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_PWR_CMD_MASK);
                    HW_WR_REG32(hndl->pInstObj->baseAddr +
                                CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(i), reg);

                    /* Wait for power down completion */
                    timeOut = 0xFFFFU;
                    while(timeOut)
                    {
                        reg = HW_RD_REG32(hndl->pInstObj->baseAddr +
                                          CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(i));
                        if(0U == (reg & CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_PWR_CMD_MASK))
                        {
                            break;
                        }
                        timeOut--;
                    }
                    if(0U == timeOut)
                    {
                        /* Could not power down the PHY... */
                        rtnVal = FVID2_ETIMEOUT;
                        GT_assert(CalTrace, ((uint32_t)FALSE));
                        break;
                    }
                }
            }
        }
        else if(CAL_HAL_MODE_M2M == hndl->mode)
        {
            rtnVal = FVID2_SOK;
            if(1U == hndl->pInstObj->openCnt)
            {
                /* For the very last clos ensure that there are no pending
                    DMA reads */
                reg = HW_RD_REG32(hndl->pInstObj->baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL);
                if(CSL_CAL_C2VID_RD_DMA_CTRL_GO_MASK == (reg & CSL_CAL_C2VID_RD_DMA_CTRL_GO_MASK))
                {
                    rtnVal = FVID2_EDEVICE_INUSE;
                }
            }
        }
        else
        {
            GT_assert(CalTrace,
                      (CAL_HAL_MODE_CMPLXIO_CTRL != hndl->mode));
            rtnVal = FVID2_SOK;
        }
        hndl->pInstObj->openCnt--;

        hndl->mode = CAL_HAL_MODE_MIN;
    }

    return rtnVal;
}

/**************************Function Separator**********************************/
int32_t Cal_halCaptureStart(Cal_HalHandle     handle,
                                    void           *arg)
{
    Cal_HalInstObj_t *pInstObj;
    volatile uint32_t   reg;
    uint32_t            idx, baseAddr, offset;
    Cal_HalObj_t     *hndl = (Cal_HalObj_t *) handle;
    int32_t rtnVal = FVID2_EBADARGS;

    if (NULL != hndl)
    {
        if (NULL != hndl->pInstObj)
        {
            if ((CAL_HAL_MODE_CAPTURE == hndl->mode) && (NULL != arg))
            {
                pInstObj = hndl->pInstObj;

                rtnVal = calSetWrDmaMode(pInstObj, (Cal_HalDmaVcCfg_t *)arg);
#ifdef ENABLE_PPI_WHEN_STARTING
                for (idx = 0; idx < pInstObj->instCfg.numPpiInst; idx++)
                {
                    if ((uint32_t)FALSE != pInstObj->instCfg.ppiCfg[idx].enable)
                    {
                        baseAddr = pInstObj->baseAddr;
                        offset   = pInstObj->instCfg.ppiCfg[idx].instance;
                        reg      =
                            HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_PPI_CTRL(offset));
                        reg |= CSL_CAL_C2PPI_CSI2_PPI_CTRL_IF_EN_MASK;
                        HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_PPI_CTRL(offset),
                                    reg);
                    }
                }
#endif /* ENABLE_PPI_WHEN_STARTING */
            } /* CAL_HAL_MODE_CAPTURE == hndl->mode */
        } /* NULL != hndl->pInstObj */
    }

    return (rtnVal);
}

/**************************Function Separator**********************************/
int32_t Cal_halCaptureStop(Cal_HalHandle handle, void *arg)
{
    Cal_HalInstObj_t *pInstObj;
    volatile uint32_t   reg;
    uint32_t            idx, baseAddr, offset;
    Cal_HalObj_t     *hndl = (Cal_HalObj_t *) handle;
    int32_t rtnVal = FVID2_EBADARGS;

    if (NULL != hndl)
    {
        if (NULL != hndl->pInstObj)
        {
            if ((CAL_HAL_MODE_CAPTURE == hndl->mode) && (NULL != arg))
            {
                pInstObj = hndl->pInstObj;

                rtnVal = calSetWrDmaMode(pInstObj, (Cal_HalDmaVcCfg_t *)arg);
#ifdef ENABLE_PPI_WHEN_STARTING
                /* TODO - Check if all write DMAs have been stopped.
                            Otherwise multiple handle will not work.
                            Right way would be
                            . Do it in the core
                            . If NOT the last wr context
                            . Setup to receive into 10 X 10 buffer
                            . Updated size, offset
                            . On disable of last context
                            . Setup to receive into 10 x 10 buffer,
                            . Disable PPI
                            . */

                for(idx = 0; idx < pInstObj->instCfg.numPpiInst; idx++)
                {
                    if((uint32_t)FALSE !=
                       pInstObj->instCfg.ppiCfg[idx].enable)
                    {
                        baseAddr = pInstObj->baseAddr;
                        offset   = pInstObj->instCfg.ppiCfg[idx].instance;
                        reg      =
                            HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_PPI_CTRL(offset));
                        reg &= ~((uint32_t) CSL_CAL_C2PPI_CSI2_PPI_CTRL_IF_EN_MASK);
                        HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_PPI_CTRL(offset),
                                    reg);
                    }
                }
#endif          /* ENABLE_PPI_WHEN_STARTING */
            }
        }
    }

    return (rtnVal);
}

/**************************Function Separator**********************************/
int32_t Cal_halUpdateBufAddr(Cal_HalHandle          handle,
                                   const Cal_HalBufferAddr_t *bufPtrs)
{
    volatile uint32_t       reg;
    uint32_t                idx, baseAddr;
    int32_t                 rtnVal = FVID2_EBADARGS;
    Cal_HalObj_t          *hndl = (Cal_HalObj_t *) handle;

    if (NULL != hndl)
    {
        if(NULL != hndl->pInstObj)
        {
            if (NULL != bufPtrs)
            {
                rtnVal = FVID2_SOK;
                for(idx = 0U; idx < bufPtrs->numBuff; idx++)
                {
                    if((uint32_t)0U == (uint32_t)bufPtrs->buffAddr[idx])
                    {
                        rtnVal = FVID2_EBADARGS;
                    }
                    if((CAL_HAL_MODE_M2M == hndl->mode) ||
                       (CAL_HAL_MODE_CAPTURE == hndl->mode))
                    {
                        if(CAL_CAPT_MAX_STREAMS <= bufPtrs->wrDmaCtx[idx])
                        {
                            rtnVal = FVID2_EBADARGS;
                        }
                    }

                    if(FVID2_SOK != rtnVal)
                    {
                        break;
                    }

                    baseAddr = hndl->pInstObj->baseAddr;
                    if(((uint32_t)0x0 == (uint32_t)bufPtrs->cPortId[idx]) &&
                       (CAL_HAL_MODE_M2M == hndl->mode))
                    {
                        reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_PIX_ADDR);
                        reg &= ~CSL_CAL_C2VID_RD_DMA_PIX_ADDR_ADDR_MASK;
                        reg |= CSL_CAL_C2VID_RD_DMA_PIX_ADDR_ADDR_MASK &
                               bufPtrs->buffAddr[idx];
                        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_PIX_ADDR, reg);

                        reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_PIX_OFST);
                        reg &= ~CSL_CAL_C2VID_RD_DMA_PIX_OFST_OFST_MASK;
                        reg |= CSL_CAL_C2VID_RD_DMA_PIX_OFST_OFST_MASK &
                               bufPtrs->pitch[idx];
                        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_PIX_OFST, reg);
                    }

                    if((CAL_HAL_MODE_M2M == hndl->mode) ||
                       (CAL_HAL_MODE_CAPTURE == hndl->mode))
                    {
                        reg = HW_RD_REG32(baseAddr +
                                          CSL_CAL_WR_DMA_WR_DMA_ADDR(bufPtrs->wrDmaCtx
                                                          [idx]));
                        reg &= ~CSL_CAL_WR_DMA_WR_DMA_ADDR_ADDR_MASK;
                        reg |= CSL_CAL_WR_DMA_WR_DMA_ADDR_ADDR_MASK &
                               bufPtrs->buffAddr[idx];
                        HW_WR_REG32(baseAddr +
                                    CSL_CAL_WR_DMA_WR_DMA_ADDR(
                                        bufPtrs->wrDmaCtx[idx]), reg);
                    }
                }
            }
        }
    }

    return (rtnVal);
}

/**************************Function Separator**********************************/

int32_t Cal_halRdDmaStart(Cal_HalHandle     handle,
                                Cal_HalCtrlProcMode_t procMode,
                                void             *arg)
{
    int32_t             rtnVal = FVID2_SOK;
    volatile uint32_t   reg;
    Cal_HalInstObj_t *pInstObj;
    Cal_HalObj_t     *hndl = (Cal_HalObj_t *) handle;

    if(NULL == hndl)
    {
        rtnVal = FVID2_EBADARGS;
    }

    if((FVID2_SOK == rtnVal) && (NULL == hndl->pInstObj))
    {
        rtnVal = FVID2_EBADARGS;
    }

    if(FVID2_SOK == rtnVal)
    {
        pInstObj = hndl->pInstObj;

        rtnVal = FVID2_EAGAIN;

        reg = HW_RD_REG32(pInstObj->baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL);
        if(CSL_CAL_C2VID_RD_DMA_CTRL_GO_MASK != (reg & CSL_CAL_C2VID_RD_DMA_CTRL_GO_MASK))
        {
            reg   |= CSL_CAL_C2VID_RD_DMA_CTRL_GO_MASK;
            rtnVal = FVID2_SOK;
            HW_WR_REG32(pInstObj->baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL, reg);
        }
    }

    return rtnVal;
}

/**************************Function Separator**********************************/

int32_t Cal_halControl(Cal_HalHandle handle,
                             uint32_t        cmd,
                             void         *cmdArgs,
                             void         *arg)
{
    Cal_HalInstObj_t    *pInstObj;
    Cal_HalObj_t        *hndl = (Cal_HalObj_t *) handle;

    int32_t rtnVal = FVID2_SOK;

    if(NULL == hndl)
    {
        rtnVal = FVID2_EBADARGS;
    }

    if((FVID2_SOK == rtnVal) && (NULL != hndl->pInstObj))
    {
        pInstObj = hndl->pInstObj;
    }
    else
    {
        rtnVal = FVID2_EBADARGS;
    }

    if((FVID2_SOK == rtnVal) && (0U == pInstObj->baseAddr))
    {
        rtnVal = FVID2_EBADARGS;
    }

    if((FVID2_SOK == rtnVal) && (0x0 == pInstObj->openCnt))
    {
        rtnVal = FVID2_EFAIL;
    }

    if(FVID2_SOK == rtnVal)
    {
        switch(cmd)
        {
            case IOCTL_CAL_HAL_UPDATE_BUFFERS:
                rtnVal = FVID2_EBADARGS;
                break;

            case IOCTL_CAL_HAL_RD_FMT_UPDATE:
                rtnVal = CalSetRdDmaFrameCfg(pInstObj, (Fvid2_Format *)cmdArgs);
                break;

            case IOCTL_CAL_HAL_START:
                rtnVal = Cal_halCaptureStart (handle, cmdArgs);
                break;

            case IOCTL_CAL_HAL_STOP:
                rtnVal = Cal_halCaptureStop (handle, cmdArgs);
                break;

            case IOCTL_CAL_HAL_SETCFG:
                if(NULL != cmdArgs)
                {
                    rtnVal = CalSetCfg(hndl, (Cal_HalCfg_t *) cmdArgs);
                }
                else
                {
                    rtnVal = FVID2_EBADARGS;
                }
                break;

            case IOCTL_CAL_HAL_GET_INSTANCECFG:
                if(NULL != cmdArgs)
                {
                    memcpy((void *) cmdArgs,
                                    (const void *) &hndl->pInstObj->instCfg,
                                    sizeof(Cal_HalInstCfg_t));
                }
                else
                {
                    rtnVal = FVID2_EBADARGS;
                }
                break;

            case IOCTL_CAL_HAL_SET_INSTANCECFG:
                if(NULL != cmdArgs)
                {
                    rtnVal = CalResetApplyInstConfig(
                        hndl,
                        (Cal_HalInstCfg_t *)
                        cmdArgs,
                        hndl->mode,
                        (uint32_t)TRUE);

                    if(FVID2_SOK == rtnVal)
                    {
                        memcpy((void *) & hndl->pInstObj->instCfg,
                                        (const void *) cmdArgs,
                                        sizeof(Cal_HalInstCfg_t));
                    }
                }
                else
                {
                    rtnVal = FVID2_EBADARGS;
                }
                break;
            case IOCTL_CAL_HAL_SET_VPORT_CFG:
                if(NULL != cmdArgs)
                {
                    rtnVal = CalSetVportCfg(
                        hndl->pInstObj,
                        (Cal_VPort_t *) cmdArgs,
                        *(uint32_t *) arg);
                }
                else
                {
                    rtnVal = FVID2_EBADARGS;
                }
                break;
            case IOCTL_CAL_HAL_SET_BYSOUT_CFG:
                if(NULL != cmdArgs)
                {
                    rtnVal = CalSetBysOutCfg(
                        hndl->pInstObj,
                        (Cal_BysOut_t *) cmdArgs,
                        *(uint32_t *) arg);
                }
                else
                {
                    rtnVal = FVID2_EBADARGS;
                }
                break;

            case IOCTL_CAL_HAL_LINE_EVENT_CFG:
                if(NULL != cmdArgs)
                {
                    rtnVal = CalSetLineEventCfg(hndl->pInstObj,
                                        (Cal_HalLineEventCfg_t *) cmdArgs);
                }
                else
                {
                    rtnVal = FVID2_EBADARGS;
                }
                break;

            default:
                rtnVal = FVID2_EUNSUPPORTED_CMD;
                break;
        }
    }

    return (rtnVal);
}

uint32_t Cal_halIsBysOutEof(Cal_HalHandle handle)
{
    uint32_t rtnVal = 0;
    uint32_t baseAddr;
    Cal_HalObj_t *hndl = (Cal_HalObj_t *) handle;

    if ((NULL == hndl) || (NULL == hndl->pInstObj) ||
        (0U == hndl->pInstObj->baseAddr) || (0x0 == hndl->pInstObj->openCnt))
    {
        rtnVal = 0;
    }
    else
    {
        baseAddr = hndl->pInstObj->baseAddr;
        rtnVal = (HW_RD_REG32(baseAddr + CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW(0U)) &
            CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW_IRQ3_MASK) >> CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW_IRQ3_SHIFT;
    }

    return (rtnVal);
}

void Cal_halClearVportEof(Cal_HalHandle handle)
{
    Cal_HalObj_t *hndl = (Cal_HalObj_t *) handle;
    uint32_t baseAddr;
    uint32_t regVal;

    if ((NULL == hndl) || (NULL == hndl->pInstObj) ||
        (0U == hndl->pInstObj->baseAddr) || (0x0 == hndl->pInstObj->openCnt))
    {
        /* Do Nothing */
    }
    else
    {
        baseAddr = hndl->pInstObj->baseAddr;
        regVal = HW_RD_REG32(baseAddr + CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW(0U));
        regVal |= CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW_IRQ2_MASK;
        HW_WR_REG32(baseAddr + CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW(0U), regVal);
    }
}

uint32_t Cal_halIsVportEof(Cal_HalHandle handle)
{
    uint32_t rtnVal = 0;
    Cal_HalObj_t *hndl = (Cal_HalObj_t *) handle;
    uint32_t baseAddr;

    if ((NULL == hndl) || (NULL == hndl->pInstObj) ||
        (0U == hndl->pInstObj->baseAddr) || (0x0 == hndl->pInstObj->openCnt))
    {
        rtnVal = 0;
    }
    else
    {
        baseAddr = hndl->pInstObj->baseAddr;
        rtnVal = (HW_RD_REG32(baseAddr + CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW(0U)) &
            CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW_IRQ2_MASK) >> CSL_CAL_C2IRQ_HL_IRQSTATUS_RAW_IRQ2_SHIFT;
    }

    return (rtnVal);
}

static Cal_HalPlatformData_t gCalHalPlatformData =
{(CSL_CAL_PER_CNT), NULL};

Cal_HalPlatformData_t *Cal_halGetPlatformData(void)
{
    Cal_HalPlatformData_t *platData = NULL;

    {
        gCalHalPlatformData.calInstPrms     = &gCalHalInstParams[0U];
        platData = &gCalHalPlatformData;
    }
    return (platData);
}

/**************************Function Separator**********************************/

static int32_t CalReset(uint32_t baseAddr)
{
    int32_t           rtnVal = FVID2_SOK;
    volatile uint32_t reg, timeOut;

    GT_assert(CalTrace, (0U != baseAddr));
    timeOut = 0xFFFFU;
    reg     = HW_RD_REG32(baseAddr + CSL_CAL_C2CTL_HL_SYSCONFIG);

    reg &= ~((uint32_t) CSL_CAL_C2CTL_HL_SYSCONFIG_IDLEMODE_MASK);
    reg |= CSL_CAL_C2CTL_HL_SYSCONFIG_IDLEMODE_MASK & ((uint32_t) CSL_CAL_C2CTL_HL_SYSCONFIG_IDLEMODE_NO <<
                                             CSL_CAL_C2CTL_HL_SYSCONFIG_IDLEMODE_SHIFT);

    reg |= CSL_CAL_C2CTL_HL_SYSCONFIG_SOFTRESET_MASK;
    HW_WR_REG32(baseAddr + CSL_CAL_C2CTL_HL_SYSCONFIG, reg);

    while(timeOut)
    {
        reg = HW_RD_REG32(baseAddr + CSL_CAL_C2CTL_HL_SYSCONFIG);
        if(CSL_CAL_C2CTL_HL_SYSCONFIG_SOFTRESET_MASK !=
           (reg & CSL_CAL_C2CTL_HL_SYSCONFIG_SOFTRESET_MASK))
        {
            break;
        }

        timeOut--;
    }

    /* If reset is not done, timeOut will be zero, so only one
       condition check required here
       If reset is done, this condition is anyway false. */
    if((reg & CSL_CAL_C2CTL_HL_SYSCONFIG_SOFTRESET_MASK) ==
       CSL_CAL_C2CTL_HL_SYSCONFIG_SOFTRESET_MASK)
    {
        /* Reset did not complete. We would require an  PRCM reset now! */
        rtnVal = FVID2_ETIMEOUT;
    }
    return rtnVal;
}

/**************************Function Separator**********************************/

static int32_t CalCfgInstWrDma(
    uint32_t baseAddr, const Cal_HalInstCfg_t *pCfg)
{
    volatile uint32_t reg;
    reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_CTRL);
    reg &= ~(CSL_CAL_C2VID_CTRL_POSTED_WRITES_MASK | CSL_CAL_C2VID_CTRL_TAGCNT_MASK |
             CSL_CAL_C2VID_CTRL_BURSTSIZE_MASK | CSL_CAL_C2VID_CTRL_MFLAGL_MASK |
             CSL_CAL_C2VID_CTRL_PWRSCPCLK_MASK | CSL_CAL_C2VID_CTRL_RD_DMA_STALL_MASK |
             CSL_CAL_C2VID_CTRL_MFLAGH_MASK);

    reg |= CSL_CAL_C2VID_CTRL_MFLAGH_MASK & (pCfg->mFlagH << CSL_CAL_C2VID_CTRL_MFLAGH_SHIFT);

    if((uint32_t)TRUE == pCfg->rdDmaStall)
    {
        reg |= CSL_CAL_C2VID_CTRL_RD_DMA_STALL_MASK;
    }

    if((uint32_t)TRUE == pCfg->pwrScpClk)
    {
        reg |= CSL_CAL_C2VID_CTRL_PWRSCPCLK_MASK;
    }

    reg |= CSL_CAL_C2VID_CTRL_MFLAGL_MASK & (pCfg->mFlagL << CSL_CAL_C2VID_CTRL_MFLAGL_SHIFT);

    reg |= CSL_CAL_C2VID_CTRL_BURSTSIZE_MASK &
           (pCfg->dmaBurstSize << CSL_CAL_C2VID_CTRL_BURSTSIZE_SHIFT);

    reg |= CSL_CAL_C2VID_CTRL_TAGCNT_MASK & (pCfg->tagCnt << CSL_CAL_C2VID_CTRL_TAGCNT_SHIFT);

    if((uint32_t)TRUE == pCfg->postedWrites)
    {
        reg |= CSL_CAL_C2VID_CTRL_POSTED_WRITES_MASK;
    }

    HW_WR_REG32(baseAddr + CSL_CAL_C2VID_CTRL, reg);

    return FVID2_SOK;
}

/**************************Function Separator**********************************/

static int32_t CalCfgInstLanePpiCfg(uint32_t baseAddr, Cal_HalInstCfg_t *pCfg)
{
    uint32_t               offset, idx;
    volatile uint32_t      reg;
    Cal_CmplxIoCfg_t *pCmplxIoCfg;
    int32_t rtnVal = FVID2_SOK;

    GT_assert(CalTrace, (NULL != pCfg));
    GT_assert(CalTrace, (CSL_CAL_PPI_CNT >= pCfg->numPpiInst));
    GT_assert(CalTrace, (CSL_CAL_CMPLXIO_CNT >= pCfg->numCmplxIoInst));

    /* Lane Config */
    offset = 0x0;
    for(idx = 0; idx < pCfg->numCmplxIoInst; idx++)
    {
        pCmplxIoCfg = &pCfg->cmplxIoCfg[idx];

        /* Setup the polarity of lanes */
        if((uint32_t)TRUE == pCmplxIoCfg->enable)
        {
            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx));
            reg &= ~(CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_CLOCK_POSITION_MASK |
                     CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA1_POSITION_MASK |
                     CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA2_POSITION_MASK |
                     CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA3_POSITION_MASK |
                     CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA4_POSITION_MASK);
            reg &= ~(CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_CLOCK_POL_MASK |
                     CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA1_POL_MASK |
                     CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA2_POL_MASK |
                     CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA3_POL_MASK |
                     CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA4_POL_MASK);
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_CLOCK_POSITION_MASK &
                   pCmplxIoCfg->clockLane.position;
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA1_POSITION_MASK &
                   (pCmplxIoCfg->data1Lane.position <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA1_POSITION_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA2_POSITION_MASK &
                   (pCmplxIoCfg->data2Lane.position <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA2_POSITION_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA3_POSITION_MASK &
                   (pCmplxIoCfg->data3Lane.position <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA3_POSITION_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA4_POSITION_MASK &
                   (pCmplxIoCfg->data4Lane.position <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA4_POSITION_SHIFT);

            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_CLOCK_POL_MASK &
                   (pCmplxIoCfg->clockLane.pol <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_CLOCK_POL_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA1_POL_MASK &
                   (pCmplxIoCfg->data1Lane.pol <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA1_POL_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA2_POL_MASK &
                   (pCmplxIoCfg->data2Lane.pol <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA2_POL_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA3_POL_MASK &
                   (pCmplxIoCfg->data3Lane.pol <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA3_POL_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA4_POL_MASK &
                   (pCmplxIoCfg->data4Lane.pol <<
                    CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_DATA4_POL_SHIFT);
            HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx), reg);

            /* Dummy read to wait for SCP write to complete */
            reg = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx));
        }
    }
    /* PPI Config */
    offset = 0x0;
    for(idx = 0; idx < pCfg->numPpiInst; idx++)
    {
        if((uint32_t)FALSE != pCfg->ppiCfg[idx].enable)
        {
            offset = pCfg->ppiCfg[idx].instance;
            if(CSL_CAL_PPI_CNT < offset)
            {
                rtnVal = FVID2_EBADARGS;
                break;
            }

            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_PPI_CTRL(offset));
            reg &= ~((uint32_t) (CSL_CAL_C2PPI_CSI2_PPI_CTRL_IF_EN_MASK |
                               CSL_CAL_C2PPI_CSI2_PPI_CTRL_FRAME_MASK |
                               CSL_CAL_C2PPI_CSI2_PPI_CTRL_ECC_EN_MASK));

            if((uint32_t)TRUE == pCfg->ppiCfg[idx].ecc)
            {
                reg |= CSL_CAL_C2PPI_CSI2_PPI_CTRL_ECC_EN_MASK;
            }
            if((uint32_t)TRUE == pCfg->ppiCfg[idx].frame)
            {
                reg |= CSL_CAL_C2PPI_CSI2_PPI_CTRL_FRAME_MASK;
            }
            HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_PPI_CTRL(offset), reg);

#ifndef ENABLE_PPI_WHEN_STARTING
            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_PPI_CTRL(offset));
            reg |= CSL_CAL_C2PPI_CSI2_PPI_CTRL_IF_EN_MASK;
            HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_PPI_CTRL(offset), reg);
#endif      /* ENABLE_PPI_WHEN_STARTING */
        }
    }
    return (rtnVal);
}

/**************************Function Separator**********************************/

static int32_t CalResetApplyInstConfig(const Cal_HalObj_t *pHndl,
                                       Cal_HalInstCfg_t   *pCfg,
                                       Cal_HalMode_t       mode,
                                       uint32_t             enPhy)
{
    int32_t             rtnVal = FVID2_SOK;
    Cal_HalInstObj_t *pInst  = NULL;
    volatile uint32_t   reg, baseAddr;

    GT_assert(CalTrace, (NULL != pHndl));
    GT_assert(CalTrace, (NULL != pCfg));
    GT_assert(CalTrace, (NULL != pHndl->pInstObj));

    pInst = pHndl->pInstObj;
    GT_assert(CalTrace, (0U != pInst->baseAddr));
    baseAddr = pInst->baseAddr;

    /* In Case of M2M mode,
     *      . Reset CAL
     *      . Apply M2M specifics
     * In Case of capture via CSI2 / LVDS (CPI)
     *      . Reset CAL
     *      . Configure generic CAL / DMA write config
     *      . Configure lane and PPI
     *      . Enable Clock to PHY and Reset the same
     */
    /* . Reset CAL */
    rtnVal = CalReset(baseAddr);
    /* . Apply M2M config */
    if((FVID2_SOK == rtnVal) && (CAL_HAL_MODE_M2M == mode))
    {
        reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_CTRL);
        reg &= ~(CSL_CAL_C2VID_CTRL_RD_DMA_STALL_MASK | CSL_CAL_C2VID_CTRL_PWRSCPCLK_MASK |
                 CSL_CAL_C2VID_CTRL_TAGCNT_MASK);

        if((uint32_t)TRUE == pCfg->rdDmaStall)
        {
            reg |= CSL_CAL_C2VID_CTRL_RD_DMA_STALL_MASK;
        }
        if((uint32_t)TRUE == pCfg->pwrScpClk)
        {
            reg |= CSL_CAL_C2VID_CTRL_PWRSCPCLK_MASK;
        }

        reg |= CSL_CAL_C2VID_CTRL_TAGCNT_MASK & (pCfg->tagCnt << CSL_CAL_C2VID_CTRL_TAGCNT_SHIFT);

        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_CTRL, reg);
    }
    else if(CAL_HAL_MODE_CAPTURE == mode)
    {
        GT_assert(CalTrace, (0U != pInst->phy0BaseAddress));
        /* Important to configure RD DMA stall config, as m2m mode will not be
         *  worried about this, but write will definitely be */
        rtnVal = CalCfgInstWrDma(baseAddr, pCfg);

        if(FVID2_SOK == rtnVal)
        {
            rtnVal = CalCfgInstLanePpiCfg(baseAddr, pCfg);
        }

        if((FVID2_SOK == rtnVal) && ((uint32_t)TRUE == enPhy))
        {
            rtnVal = Cal_halPhyEnClockAndReset(baseAddr,
                                                  pInst->phy0BaseAddress,
                                                  pInst->phy1BaseAddress,
                                                  pCfg);
        }
    }
    else if(CAL_HAL_MODE_CMPLXIO_CTRL == mode)
    {
        /* TBD */
        rtnVal = FVID2_EBADARGS;
    }
    else
    {
        rtnVal = FVID2_EBADARGS;
    }

    return (rtnVal);
}

/**************************Function Separator**********************************/

static int32_t CalSetCfg(const Cal_HalObj_t *hndl, const Cal_HalCfg_t *cfg)
{
    /*
     *  .Configure Capture
     *  .Configure M2M
     *  .Configure Complex IO TODO TBD
     */
    int32_t rtnVal = FVID2_SOK;

    GT_assert(CalTrace, (NULL != hndl));
    GT_assert(CalTrace, (NULL != cfg));
    GT_assert(CalTrace, (NULL != hndl->pInstObj));

    if(0x0 == hndl->pInstObj->openCnt)
    {
        rtnVal = FVID2_EBADARGS;
    }

    if(FVID2_SOK == rtnVal)
    {
        if(CAL_HAL_MODE_CAPTURE == hndl->mode)
        {
            rtnVal = CalSetCfgForMode(hndl->pInstObj, cfg, hndl->mode);
        }
        else if(CAL_HAL_MODE_M2M == hndl->mode)
        {
            rtnVal = CalSetCfgForMode(hndl->pInstObj, cfg, hndl->mode);
        }
        else if(CAL_HAL_MODE_CMPLXIO_CTRL == hndl->mode)
        {
            GT_assert(CalTrace, ((uint32_t)FALSE));
            /* TODO TBD implement complex IO config apply */
        }
        else
        {
            /* Wrong mode, but why? */
            rtnVal = FVID2_EBADARGS;
        }
    }

    return FVID2_SOK;
}

/**************************Function Separator**********************************/

static int32_t CalSetCfgForMode(const Cal_HalInstObj_t *pInst,
                                const Cal_HalCfg_t     *cfg,
                                Cal_HalMode_t           mode)
{
    int32_t              rtnVal = FVID2_SOK;
    uint32_t             baseAddr, cId, idx;
    volatile uint32_t    reg;
    Cal_HalDmaVcCfg_t *pDmaCfg = (Cal_HalDmaVcCfg_t *) cfg->pDmaVcCfg;

    GT_assert(CalTrace, (NULL != pInst));
    GT_assert(CalTrace, (NULL != cfg));
    GT_assert(CalTrace, (NULL != pDmaCfg));

    baseAddr = pInst->baseAddr;

    /* Ensure there are no pending reads */
    reg = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL);
    if(((reg & CSL_CAL_C2VID_RD_DMA_CTRL_GO_MASK) == CSL_CAL_C2VID_RD_DMA_CTRL_GO_MASK) &&
       (CAL_HAL_MODE_M2M == mode))
    {
        rtnVal = FVID2_EAGAIN;
    }

    for(idx = 0; ((FVID2_SOK == rtnVal) && (idx < cfg->numCPortId)); idx++)
    {
        cId = cfg->cportId[idx];
        GT_assert(CalTrace, (CAL_CAPT_MAX_STREAMS > cId));

        if(((uint32_t)TRUE == pDmaCfg->isCsi2VcCfgValid[idx]) &&
           (CAL_HAL_MODE_CAPTURE == mode))
        {
            rtnVal = CalCsi2VcCfg(pInst, &pDmaCfg->csi2VcCfg[idx], cId);
        }
        if((uint32_t)TRUE == cfg->isPixProcCfgValid[idx])
        {
            rtnVal = CalSetPixProcCfg(pInst, &cfg->pixProcCfg[idx], cId);
        }
        if((uint32_t)TRUE == pDmaCfg->isWrDmaCfgValid[idx])
        {
            /* CFG WR DMA */
            rtnVal = CalSetWrDmaCfg(pInst, &pDmaCfg->wrDmaCfg[idx], cId);
        }
        if((uint32_t)TRUE == cfg->isBysOutCfgValid[idx])
        {
            /* CFG BYS Out */
            rtnVal = CalSetBysOutCfg(pInst, &cfg->bysOutCfg[idx], cId);
        }
        if((uint32_t)TRUE == cfg->isBysInCfgValid[idx])
        {
            /* CFG BYS In */
            if((uint32_t)TRUE == cfg->bysInEnable[idx])
            {
                reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL2);
                reg &= ~((uint32_t) CSL_CAL_C2VID_BYS_CTRL2_BC2_CPORTIN_MASK);
                reg |= CSL_CAL_C2VID_BYS_CTRL2_BC2_CPORTIN_MASK & cId;
                HW_WR_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL2, reg);

                reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL1);
                reg |= CSL_CAL_C2VID_BYS_CTRL1_BC1_BYSINEN_MASK;
                HW_WR_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL1, reg);
            }
            else
            {
                reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL1);
                reg &= ~CSL_CAL_C2VID_BYS_CTRL1_BC1_BYSINEN_MASK;
                HW_WR_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL1, reg);
            }
        }
        if((uint32_t)TRUE == cfg->isVportCfgValid[idx])
        {
            /* CFG VP in */
            rtnVal = CalSetVportCfg(pInst, &cfg->vportCfg[idx], cId);
        }
        if((uint32_t)TRUE == pDmaCfg->isRdDmaCfgValid[idx])
        {
            /* CFG Read DMA */
            rtnVal = CalSetRdDmaCfg(pInst, &pDmaCfg->rdDmaCfg[idx], cId);
        }
    }

    return rtnVal;
}

/**************************Function Separator**********************************/

static int32_t CalCsi2VcCfg(const Cal_HalInstObj_t   *pInst,
                            const Cal_HalCsi2VcCfg_t *cfg,
                            uint32_t                    cportId)
{
    int32_t           rtnVal = FVID2_SOK;
    uint32_t          baseAddr, instance, context;
    volatile uint32_t reg;

    GT_assert(CalTrace, (NULL != pInst));
    GT_assert(CalTrace, (NULL != cfg));

    baseAddr = pInst->baseAddr;
    instance = cfg->instance;
    context  = cfg->contextToBeUsed;
    GT_assert(CalTrace, (CAL_CAPT_MAX_STREAMS > context));
    GT_assert(CalTrace, ((0x0U == instance) || (0x1U == instance)));

    reg = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_CTX0(instance) + (context * 4U));

    reg &= ~(CSL_CAL_C2PPI_CSI2_CTX0_CTX0_DT_MASK | CSL_CAL_C2PPI_CSI2_CTX0_CTX0_VC_MASK |
             CSL_CAL_C2PPI_CSI2_CTX0_CTX0_CPORT_MASK | CSL_CAL_C2PPI_CSI2_CTX0_CTX0_ATT_MASK |
             CSL_CAL_C2PPI_CSI2_CTX0_CTX0_PACK_MODE_MASK | CSL_CAL_C2PPI_CSI2_CTX0_CTX0_LINES_MASK);

    reg |= CSL_CAL_C2PPI_CSI2_CTX0_CTX0_LINES_MASK & (cfg->lines << CSL_CAL_C2PPI_CSI2_CTX0_CTX0_LINES_SHIFT);

    /* TODO Remove the hard-coding, get appropriate macros */
    if((cfg->dt < 0x17) && (cfg->dt != 0x1))
    {
        /* When receiving non-pixel data would be packed between FS and FE.
         *  no notion of LS & LE.
         * else
         *  Pixel typically line boundaries is specified. Data is packed between
         *      LS & LE.
         */
        reg |= CSL_CAL_C2PPI_CSI2_CTX0_CTX0_PACK_MODE_MASK;
    }

    reg |= CSL_CAL_C2PPI_CSI2_CTX0_CTX0_ATT_MASK & (cfg->att << CSL_CAL_C2PPI_CSI2_CTX0_CTX0_ATT_SHIFT);
    reg |= CSL_CAL_C2PPI_CSI2_CTX0_CTX0_CPORT_MASK & (cportId << CSL_CAL_C2PPI_CSI2_CTX0_CTX0_CPORT_SHIFT);
    reg |= CSL_CAL_C2PPI_CSI2_CTX0_CTX0_VC_MASK & (cfg->virtualChanNum <<
                                    CSL_CAL_C2PPI_CSI2_CTX0_CTX0_VC_SHIFT);
    reg |= CSL_CAL_C2PPI_CSI2_CTX0_CTX0_DT_MASK & (cfg->dt << CSL_CAL_C2PPI_CSI2_CTX0_CTX0_DT_SHIFT);

    HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_CTX0(instance) + (context * 4U), reg);
    return (rtnVal);
}

/**************************Function Separator**********************************/

static int32_t CalSetPixProcCfg(const Cal_HalInstObj_t *pInst,
                                const Cal_PixProc_t *cfg, uint32_t cportId)
{
    int32_t           rtnVal = FVID2_SOK;
    uint32_t          baseAddr, context;
    uint32_t          extract;
    volatile uint32_t reg;

    GT_assert(CalTrace, (NULL != pInst));
    GT_assert(CalTrace, (NULL != cfg));

    baseAddr = pInst->baseAddr;
    context  = cfg->contextToBeUsed;
    extract  = cfg->extract;
    extract--;

    if(context >= CSL_CAL_PIX_PROC_CTX_CNT)
    {
        rtnVal = FVID2_EINVALID_PARAMS;
    }
    else
    {
        reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PIX_PIX_PROC(context));
        reg &= ~((uint32_t) CSL_CAL_C2PIX_PIX_PROC_EN_MASK);
        HW_WR_REG32(baseAddr + CSL_CAL_C2PIX_PIX_PROC(context), reg);

        reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PIX_PIX_PROC(context));
        reg &= ~(CSL_CAL_C2PIX_PIX_PROC_EXTRACT_MASK | CSL_CAL_C2PIX_PIX_PROC_DPCMD_MASK |
                 CSL_CAL_C2PIX_PIX_PROC_DPCME_MASK | CSL_CAL_C2PIX_PIX_PROC_PACK_MASK |
                 CSL_CAL_C2PIX_PIX_PROC_CPORT_MASK);

        GT_assert(CalTrace, (CAL_PIX_EXRCT_MIN != cfg->extract));
        GT_assert(CalTrace, (CAL_PIX_EXRCT_MAX >= cfg->extract));
        reg |= CSL_CAL_C2PIX_PIX_PROC_EXTRACT_MASK &
               ((extract) << CSL_CAL_C2PIX_PIX_PROC_EXTRACT_SHIFT);

        reg |= CSL_CAL_C2PIX_PIX_PROC_DPCMD_MASK &
               (cfg->decCodec << CSL_CAL_C2PIX_PIX_PROC_DPCMD_SHIFT);

        reg |= CSL_CAL_C2PIX_PIX_PROC_DPCME_MASK &
               (cfg->encCodec << CSL_CAL_C2PIX_PIX_PROC_DPCME_SHIFT);

        reg |= CSL_CAL_C2PIX_PIX_PROC_PACK_MASK & (cfg->pack << CSL_CAL_C2PIX_PIX_PROC_PACK_SHIFT);
        reg |= CSL_CAL_C2PIX_PIX_PROC_CPORT_MASK & (cportId << CSL_CAL_C2PIX_PIX_PROC_CPORT_SHIFT);

        HW_WR_REG32(baseAddr + CSL_CAL_C2PIX_PIX_PROC(context), reg);

        reg = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL);
        if((uint32_t)TRUE == cfg->enableDpcmInitContext)
        {
            reg |= CSL_CAL_C2VID_RD_DMA_CTRL_INIT_MASK;
            GT_assert(CalTrace, (0U != cfg->addr));
        }
        else
        {
            reg &= ~((uint32_t) CSL_CAL_C2VID_RD_DMA_CTRL_INIT_MASK);
        }
        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL, reg);

        if((uint32_t)TRUE == cfg->enableDpcmInitContext)
        {
            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_INIT_ADDR);
            reg &= ~CSL_CAL_C2VID_RD_DMA_INIT_ADDR_DIA_ADDR_MASK;
            reg |= CSL_CAL_C2VID_RD_DMA_INIT_ADDR_DIA_ADDR_MASK & cfg->addr;
            HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_INIT_ADDR, reg);

            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_INIT_OFST);
            reg &= ~CSL_CAL_C2VID_RD_DMA_INIT_OFST_DIO_OFST_MASK;
            reg |= CSL_CAL_C2VID_RD_DMA_INIT_OFST_DIO_OFST_MASK & cfg->offSet;
            HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_INIT_OFST, reg);
        }

        /* Enable Pixel Processing, as a last step */
        reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PIX_PIX_PROC(context));
        reg |= CSL_CAL_C2PIX_PIX_PROC_EN_MASK;
        HW_WR_REG32(baseAddr + CSL_CAL_C2PIX_PIX_PROC(context), reg);
    }

    return rtnVal;
}

/**************************Function Separator**********************************/

static int32_t CalSetBysOutCfg(const Cal_HalInstObj_t *pInst,
                               const Cal_BysOut_t *cfg, uint32_t cportId)
{
    uint32_t          baseAddr;
    volatile uint32_t reg;

    GT_assert(CalTrace, (NULL != pInst));
    GT_assert(CalTrace, (NULL != cfg));

    baseAddr = pInst->baseAddr;
    GT_assert(CalTrace, (0U != baseAddr));

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL1);
    reg &= ~CSL_CAL_C2VID_BYS_CTRL1_BC1_PCLK_MASK;
    if((uint32_t)TRUE == cfg->enable)
    {
        reg &= ~(CSL_CAL_C2VID_BYS_CTRL1_BC1_XBLK_MASK | CSL_CAL_C2VID_VPORT_CTRL1_VC1_YBLK_MASK);

        reg |= CSL_CAL_C2VID_BYS_CTRL1_BC1_PCLK_MASK &
               (cfg->pixClock << CSL_CAL_C2VID_BYS_CTRL1_BC1_PCLK_SHIFT);

        reg |= CSL_CAL_C2VID_BYS_CTRL1_BC1_XBLK_MASK &
               (cfg->xBlk << CSL_CAL_C2VID_BYS_CTRL1_BC1_XBLK_SHIFT);

        reg |= CSL_CAL_C2VID_VPORT_CTRL1_VC1_YBLK_MASK &
               (cfg->yBlk << CSL_CAL_C2VID_BYS_CTRL1_BC1_YBLK_SHIFT);
        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL1, reg);

        reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL2);
        reg &= ~((uint32_t) (CSL_CAL_C2VID_BYS_CTRL2_BC2_CPORTOUT_MASK |
                           CSL_CAL_C2VID_BYS_CTRL2_BC2_DUPLICATEDDATA_MASK |
                           CSL_CAL_C2VID_BYS_CTRL2_BC2_FREERUNNING_MASK));
        if((uint32_t)TRUE == cfg->freeRun)
        {
            reg |= CSL_CAL_C2VID_BYS_CTRL2_BC2_FREERUNNING_MASK;
        }

        if((uint32_t)TRUE == cfg->copyStreamToEncode)
        {
            reg |= CSL_CAL_C2VID_BYS_CTRL2_BC2_DUPLICATEDDATA_MASK;
        }

        reg |= CSL_CAL_C2VID_BYS_CTRL2_BC2_CPORTOUT_MASK &
               (cportId << CSL_CAL_C2VID_BYS_CTRL2_BC2_CPORTOUT_SHIFT);

        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL2, reg);
    }
    else
    {
        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_BYS_CTRL1, reg);
    }

    return FVID2_SOK;
}

/**************************Function Separator**********************************/

static int32_t CalSetVportCfg(const Cal_HalInstObj_t *pInst,
                              const Cal_VPort_t   *cfg,
                              uint32_t                  cportId)
{
    uint32_t          baseAddr;
    volatile uint32_t reg;

    GT_assert(CalTrace, (NULL != pInst));
    GT_assert(CalTrace, (NULL != cfg));

    baseAddr = pInst->baseAddr;
    GT_assert(CalTrace, (0U != baseAddr));

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_VPORT_CTRL1);
    reg &= ~CSL_CAL_C2VID_VPORT_CTRL1_VC1_PCLK_MASK;
    if((uint32_t)TRUE == cfg->enable)
    {
        reg &= ~(CSL_CAL_C2VID_VPORT_CTRL1_VC1_XBLK_MASK | CSL_CAL_C2VID_VPORT_CTRL1_VC1_YBLK_MASK |
                 CSL_CAL_C2VID_VPORT_CTRL1_VC1_WIDTH_MASK);

        reg |= CSL_CAL_C2VID_VPORT_CTRL1_VC1_PCLK_MASK & cfg->pixClock;

        reg |= CSL_CAL_C2VID_VPORT_CTRL1_VC1_XBLK_MASK &
               (cfg->xBlk << CSL_CAL_C2VID_VPORT_CTRL1_VC1_XBLK_SHIFT);

        reg |= CSL_CAL_C2VID_VPORT_CTRL1_VC1_YBLK_MASK &
               (cfg->yBlk << CSL_CAL_C2VID_VPORT_CTRL1_VC1_YBLK_SHIFT);

        if(0x0U != cfg->width)
        {
            reg |= CSL_CAL_C2VID_VPORT_CTRL1_VC1_WIDTH_MASK;
        }
        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_VPORT_CTRL1, reg);

        reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_VPORT_CTRL2);
        reg &=
            ~(CSL_CAL_C2VID_VPORT_CTRL2_VC2_RDY_THR_MASK | CSL_CAL_C2VID_VPORT_CTRL2_VC2_FSM_RESET_MASK |
              CSL_CAL_C2VID_VPORT_CTRL2_VC2_FS_RESETS_MASK |
              CSL_CAL_C2VID_VPORT_CTRL2_VC2_FREERUNNING_MASK |
              CSL_CAL_C2VID_VPORT_CTRL2_VC2_CPORT_MASK);

        reg |= CSL_CAL_C2VID_VPORT_CTRL2_VC2_CPORT_MASK & cportId;

        if((uint32_t)TRUE == cfg->freeRun)
        {
            reg |= CSL_CAL_C2VID_VPORT_CTRL2_VC2_FREERUNNING_MASK;
        }

        if((uint32_t)TRUE == cfg->fsReset)
        {
            reg |= CSL_CAL_C2VID_VPORT_CTRL2_VC2_FS_RESETS_MASK;
        }

        reg |= CSL_CAL_C2VID_VPORT_CTRL2_VC2_RDY_THR_MASK &
               (cfg->rdyThr << CSL_CAL_C2VID_VPORT_CTRL2_VC2_RDY_THR_SHIFT);

        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_VPORT_CTRL2, reg);
    }
    else
    {
        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_VPORT_CTRL1, reg);
    }

    return FVID2_SOK;
}

/**************************Function Separator**********************************/

static int32_t CalSetRdDmaCfg(const Cal_HalInstObj_t  *pInst,
                              const Cal_HalRdDmaCfg_t *cfg,
                              uint32_t                   cportId)
{
    uint32_t          baseAddr;
    volatile uint32_t reg;

    GT_assert(CalTrace, (NULL != pInst));
    GT_assert(CalTrace, (NULL != cfg));

    baseAddr = pInst->baseAddr;
    GT_assert(CalTrace, (0U != baseAddr));

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL);
    reg &= ~CSL_CAL_C2VID_RD_DMA_CTRL_PCLK_MASK;

    if((uint32_t)TRUE == cfg->enable)
    {
        reg &= ~((uint32_t) (CSL_CAL_C2VID_RD_DMA_CTRL_GO_MASK |
                           CSL_CAL_C2VID_RD_DMA_CTRL_BW_LIMITER_MASK |
                           CSL_CAL_C2VID_RD_DMA_CTRL_OCP_TAG_CNT_MASK));

        reg |= CSL_CAL_C2VID_RD_DMA_CTRL_BW_LIMITER_MASK &
               (cfg->bwLimit << CSL_CAL_C2VID_RD_DMA_CTRL_BW_LIMITER_SHIFT);

        reg |= CSL_CAL_C2VID_RD_DMA_CTRL_OCP_TAG_CNT_MASK &
               (cfg->ocpTagCnt << CSL_CAL_C2VID_RD_DMA_CTRL_OCP_TAG_CNT_SHIFT);

        reg |= CSL_CAL_C2VID_RD_DMA_CTRL_PCLK_MASK &
               (cfg->pixClock << CSL_CAL_C2VID_RD_DMA_CTRL_PCLK_SHIFT);

        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL, reg);

        reg = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL2);

        reg &= ~(CSL_CAL_C2VID_RD_DMA_CTRL2_CIRC_MODE_MASK |
                 CSL_CAL_C2VID_RD_DMA_CTRL2_CIRC_SIZE_MASK |
                 CSL_CAL_C2VID_RD_DMA_CTRL2_ICM_CSTART_MASK |
                 CSL_CAL_C2VID_RD_DMA_CTRL2_RD_PATTERN_MASK |
                 CSL_CAL_C2VID_RD_DMA_CTRL2_BYSOUT_LE_WAIT_MASK);
        reg |= CSL_CAL_C2VID_RD_DMA_CTRL2_BYSOUT_LE_WAIT_MASK &
               (cfg->bysOutLeWait << CSL_CAL_C2VID_RD_DMA_CTRL2_BYSOUT_LE_WAIT_SHIFT);

        reg |= CSL_CAL_C2VID_RD_DMA_CTRL2_RD_PATTERN_MASK &
               (cfg->ySkipMode << CSL_CAL_C2VID_RD_DMA_CTRL2_RD_PATTERN_SHIFT);

        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL2, reg);

        CalSetRdDmaFrameCfg(pInst, &cfg->format);
    }
    else
    {
        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_CTRL, reg);
    }
    return FVID2_SOK;
}

/**************************Function Separator**********************************/

static int32_t CalSetRdDmaFrameCfg(const Cal_HalInstObj_t *pInst,
                                   const Fvid2_Format       *cfg)
{
    uint32_t          baseAddr;
    uint32_t          hSize;
    volatile uint32_t reg;

    GT_assert(CalTrace, (NULL != pInst));
    GT_assert(CalTrace, (NULL != cfg));

    baseAddr = pInst->baseAddr;
    GT_assert(CalTrace, (0U != baseAddr));


    if ((int32_t) TRUE == Fvid2_isDataFmtRgb32bit(cfg->dataFormat))
    {
        hSize = ((cfg->width * 4U) / 8U);
    }
    else if ((int32_t) TRUE == Fvid2_isDataFmtRgb24bit(cfg->dataFormat))
    {
        hSize = ((cfg->width * 3U) / 8U);
    }
    else if ((int32_t) TRUE == Fvid2_isDataFmtRgb16bit(cfg->dataFormat))
    {
        hSize = ((cfg->width * 2U) / 8U);
    }
    else if((int32_t) TRUE == Fvid2_isDataFmtYuv422I(cfg->dataFormat))
    {
        if(FVID2_CCSF_BITS8_PACKED == cfg->ccsFormat)
        {
            hSize = ((cfg->width * 2U) / 8U);
        }
        else if((FVID2_CCSF_BITS10_UNPACKED16 == cfg->ccsFormat) ||
                (FVID2_CCSF_BITS12_UNPACKED16 == cfg->ccsFormat) ||
                (FVID2_CCSF_BITS16_PACKED     == cfg->ccsFormat))
        {
            hSize = ((cfg->width * 4U) / 8U);
        }
        else
        {
            hSize = cfg->width / 8U;
        }
    }
    else
    {
        hSize = cfg->width / 8U;
    }

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_XSIZE);
    reg &= ~CSL_CAL_WR_DMA_WR_DMA_XSIZE_XSIZE_MASK;
    /* Require to specify the number of 64 bits read, essentially
     *  total bits / 64 or byte count / 8 */
    reg |= CSL_CAL_WR_DMA_WR_DMA_XSIZE_XSIZE_MASK & ((hSize) << CSL_CAL_C2VID_RD_DMA_XSIZE_XSIZE_SHIFT);
    HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_XSIZE, reg);

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_YSIZE);
    reg &= ~CSL_CAL_C2VID_RD_DMA_YSIZE_YSIZE_MASK;
    reg |= CSL_CAL_C2VID_RD_DMA_YSIZE_YSIZE_MASK & (cfg->height << CSL_CAL_C2VID_RD_DMA_YSIZE_YSIZE_SHIFT);
    HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_YSIZE, reg);

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_PIX_OFST);
    reg &= ~CSL_CAL_C2VID_RD_DMA_PIX_OFST_OFST_MASK;
    reg |= (CSL_CAL_C2VID_RD_DMA_PIX_OFST_OFST_MASK & cfg->pitch[0U]);

    HW_WR_REG32(baseAddr + CSL_CAL_C2VID_RD_DMA_PIX_OFST, reg);

    return FVID2_SOK;
}


/**************************Function Separator**********************************/

/**
 * \brief   Applies DMA Write configurations.
 *
 * \param   pInst   Pointer to instance object.
 * \param   cfg     Valid configuration for DMA Write.
 *
 * \return  FVID2_SOK on success else error code.
 *
 **/
static int32_t CalSetWrDmaCfg(const Cal_HalInstObj_t  *pInst,
                              const Cal_HalWrDmaCfg_t *cfg,
                              uint32_t                   cportId)
{
    uint32_t          baseAddr, context;
    volatile uint32_t reg;

    GT_assert(CalTrace, (NULL != pInst));
    GT_assert(CalTrace, (NULL != cfg));

    baseAddr = pInst->baseAddr;
    context  = cfg->contextToBeUsed;
    GT_assert(CalTrace, (0U != baseAddr));
    GT_assert(CalTrace, (CAL_CAPT_MAX_STREAMS > context));

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_WR_DMA_WR_DMA_CTRL(context));
    reg &= ~(CSL_CAL_WR_DMA_WR_DMA_CTRL_YSIZE_MASK | CSL_CAL_WR_DMA_WR_DMA_CTRL_STALL_RD_DMA_MASK |
             CSL_CAL_WR_DMA_WR_DMA_CTRL_CPORT_MASK | CSL_CAL_WR_DMA_WR_DMA_CTRL_DTAG_MASK |
             CSL_CAL_WR_DMA_WR_DMA_CTRL_WR_PATTERN_MASK | CSL_CAL_WR_DMA_WR_DMA_CTRL_MODE_MASK);

    reg |= CSL_CAL_WR_DMA_WR_DMA_CTRL_MODE_MASK & cfg->mode;

    reg |= CSL_CAL_WR_DMA_WR_DMA_CTRL_WR_PATTERN_MASK &
           (cfg->ySkipMode << CSL_CAL_WR_DMA_WR_DMA_CTRL_WR_PATTERN_SHIFT);

    reg |= CSL_CAL_WR_DMA_WR_DMA_CTRL_DTAG_MASK &
           (cfg->stream << CSL_CAL_WR_DMA_WR_DMA_CTRL_DTAG_SHIFT);

    reg |= CSL_CAL_WR_DMA_WR_DMA_CTRL_CPORT_MASK &
           (cportId << CSL_CAL_WR_DMA_WR_DMA_CTRL_CPORT_SHIFT);

    reg |= CSL_CAL_WR_DMA_WR_DMA_CTRL_STALL_RD_DMA_MASK &
           (cfg->stallM2MRd << CSL_CAL_WR_DMA_WR_DMA_CTRL_STALL_RD_DMA_SHIFT);

    reg |= CSL_CAL_WR_DMA_WR_DMA_CTRL_YSIZE_MASK &
           (cfg->format.height << CSL_CAL_WR_DMA_WR_DMA_CTRL_YSIZE_SHIFT);
    HW_WR_REG32(baseAddr + CSL_CAL_WR_DMA_WR_DMA_CTRL(context), reg);

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_WR_DMA_WR_DMA_OFST(context));
    reg &= ~CSL_CAL_WR_DMA_WR_DMA_OFST_OFST_MASK;
    /* No support for circular mode as of now */
    reg &= ~(CSL_CAL_WR_DMA_WR_DMA_OFST_CIRC_MODE_MASK | CSL_CAL_WR_DMA_WR_DMA_OFST_CIRC_SIZE_MASK);
    /* Shifting is not required */
    reg |= CSL_CAL_WR_DMA_WR_DMA_OFST_OFST_MASK & cfg->format.pitch[0];
    HW_WR_REG32(baseAddr + CSL_CAL_WR_DMA_WR_DMA_OFST(context), reg);

    reg  = HW_RD_REG32(baseAddr + CSL_CAL_WR_DMA_WR_DMA_XSIZE(context));
    reg &= ~(CSL_CAL_WR_DMA_WR_DMA_XSIZE_XSIZE_MASK | CSL_CAL_WR_DMA_WR_DMA_XSIZE_XSKIP_MASK);
    reg |= CSL_CAL_WR_DMA_WR_DMA_XSIZE_XSKIP_MASK &
           (cfg->xPixelSkip << CSL_CAL_WR_DMA_WR_DMA_XSIZE_XSKIP_SHIFT);

    reg |= CSL_CAL_WR_DMA_WR_DMA_XSIZE_XSIZE_MASK &
           (cfg->format.width << CSL_CAL_WR_DMA_WR_DMA_XSIZE_XSIZE_SHIFT);
    HW_WR_REG32(baseAddr + CSL_CAL_WR_DMA_WR_DMA_XSIZE(context), reg);

    return FVID2_SOK;
}

/**************************Function Separator**********************************/

static int32_t calSetWrDmaMode(const Cal_HalInstObj_t  *pInstObj,
                               const Cal_HalDmaVcCfg_t *pDmaCfg)
{
    volatile uint32_t reg;
    uint32_t          baseAddr, instance, i;

    baseAddr = pInstObj->baseAddr;
    for(i = 0; i < pDmaCfg->numCPortId; i++)
    {
        if((uint32_t)TRUE == pDmaCfg->isWrDmaCfgValid[i])
        {
            instance = pDmaCfg->wrDmaCfg[i].contextToBeUsed;
            reg      = HW_RD_REG32(baseAddr + CSL_CAL_WR_DMA_WR_DMA_CTRL(instance));
            reg     &= ~((uint32_t) CSL_CAL_WR_DMA_WR_DMA_CTRL_MODE_MASK);

            reg |= CSL_CAL_WR_DMA_WR_DMA_CTRL_MODE_MASK &
                   pDmaCfg->wrDmaCfg[i].mode;

            HW_WR_REG32(baseAddr + CSL_CAL_WR_DMA_WR_DMA_CTRL(instance), reg);
        }
    }

    return (FVID2_SOK);
}

/**************************Function Separator**********************************/

static int32_t CalSetLineEventCfg(const Cal_HalInstObj_t  *pInstObj,
                                  const Cal_HalLineEventCfg_t *plnEvtCfg)
{
    volatile uint32_t reg;
    uint32_t          baseAddr, cportId, i;

    baseAddr = pInstObj->baseAddr;
    for(i = 0; i < plnEvtCfg->numCPortId; i++)
    {
        cportId = plnEvtCfg->cportId[i];
        reg = HW_RD_REG32(baseAddr + CSL_CAL_C2VID_LINE_NUMBER_EVT);

        reg &= ~((uint32_t)CSL_CAL_C2VID_LINE_NUMBER_EVT_CPORT_MASK);
        reg &= ~CSL_CAL_C2VID_LINE_NUMBER_EVT_LINE_MASK;

        reg |= CSL_CAL_C2VID_LINE_NUMBER_EVT_CPORT_MASK &
               (cportId << CSL_CAL_C2VID_LINE_NUMBER_EVT_CPORT_SHIFT);

        reg |= CSL_CAL_C2VID_LINE_NUMBER_EVT_LINE_MASK &
                (plnEvtCfg->lineNumber[cportId] << CSL_CAL_C2VID_LINE_NUMBER_EVT_LINE_SHIFT);

        HW_WR_REG32(baseAddr + CSL_CAL_C2VID_LINE_NUMBER_EVT, reg);
    }
    return (FVID2_SOK);
}

/**************************Function Separator**********************************/
