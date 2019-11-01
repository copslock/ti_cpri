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
 *  \ingroup DRV_DSS_MODULE
 *  \defgroup DRV_DSS_SOC_MODULE DSS SoC Config
 *            This is DSS documentation specific to J7 SoC
 *
 *  @{
 */

/**
 *  \file dss_soc.h
 *
 *  \brief DSS Driver J7 SOC specific file.
 */

#ifndef DSS_SOC_H_
#define DSS_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief DP HPD callback function prototype.
 *
 * \param hpdState [OUT] HPD state TRUE / FALSE.
 * \param appData  [OUT] Application's private data.
 *
 * \return None.
 */
typedef void (*Dss_DctrlDpHpdCbFxn)(uint32_t hpdState, void *appData);

/*
 *  Macros for different driver instance numbers to be passed as instance ID
 *  at the time of driver create.
 *  Note: These are read only macros. Don't modify the value of these macros.
 */

/**
 *  \name DSS DCTRL Instance IDs
 *
 *  @{
 */

/** \brief Display controller instance 0. */
#define DSS_DCTRL_INST_0                (0U)

/** \brief Maximum number of display driver instances */
#define DSS_DCTRL_INST_MAX              (1U)
/* @} */

/**
 *  \name DSS Display Instance IDs
 *
 *  @{
 */

/** \brief Video 1 Pipeline display driver instance number. */
#define DSS_DISP_INST_VID1                       (CSL_DSS_VID_PIPE_ID_VID1)

/** \brief Video Lite 1 Pipeline display driver instance number. */
#define DSS_DISP_INST_VIDL1                      (CSL_DSS_VID_PIPE_ID_VIDL1)

/** \brief Video 2 Pipeline display driver instance number. */
#define DSS_DISP_INST_VID2                       (CSL_DSS_VID_PIPE_ID_VID2)

/** \brief Video Lite 2 Pipeline display driver instance number. */
#define DSS_DISP_INST_VIDL2                      (CSL_DSS_VID_PIPE_ID_VIDL2)

/** \brief Maximum number of display driver instances */
#define DSS_DISP_INST_MAX                        (CSL_DSS_VID_PIPE_ID_MAX)
/* @} */

/**
 *  \anchor Dss_DctrlNodeType
 *  \name DSS DCTRL Node type
 *
 *  Node types that are used by the set path to connect different modules and
 *  create a graph
 *
 *  @{
 */
#define DSS_DCTRL_NODE_TYPE_INVALID    ((uint32_t) 0x0U)
#define DSS_DCTRL_NODE_TYPE_PIPE       ((uint32_t) 0x1U)
#define DSS_DCTRL_NODE_TYPE_OVERLAY    ((uint32_t) 0x2U)
#define DSS_DCTRL_NODE_TYPE_VP         ((uint32_t) 0x3U)
#define DSS_DCTRL_NODE_TYPE_OUTPUT     ((uint32_t) 0x4U)
/* @} */

/**
 *  \anchor Dss_DctrlNodeId
 *  \name DSS DCTRL Node Id
 *
 *  Node ids that are used by the set path to connect different modules and
 *  create a graph
 *
 *  @{
 */
#define DSS_DCTRL_NODE_INVALID                (0x0U)
#define DSS_DCTRL_NODE_VID1                   (0x1U)
#define DSS_DCTRL_NODE_VIDL1                  (0x2U)
#define DSS_DCTRL_NODE_VID2                   (0x3U)
#define DSS_DCTRL_NODE_VIDL2                  (0x4U)
#define DSS_DCTRL_NODE_OVERLAY1               (0x5U)
#define DSS_DCTRL_NODE_OVERLAY2               (0x6U)
#define DSS_DCTRL_NODE_OVERLAY3               (0x7U)
#define DSS_DCTRL_NODE_OVERLAY4               (0x8U)
#define DSS_DCTRL_NODE_VP1                    (0x9U)
#define DSS_DCTRL_NODE_VP2                    (0xAU)
#define DSS_DCTRL_NODE_VP3                    (0xBU)
#define DSS_DCTRL_NODE_VP4                    (0xCU)
#define DSS_DCTRL_NODE_DPI_DPI0               (0xDU)
#define DSS_DCTRL_NODE_DPI_DPI1               (0xEU)
#define DSS_DCTRL_NODE_EDP_DPI0               (0xFU)
#define DSS_DCTRL_NODE_EDP_DPI1               (0x10U)
#define DSS_DCTRL_NODE_EDP_DPI2               (0x11U)
#define DSS_DCTRL_NODE_EDP_DPI3               (0x12U)
#define DSS_DCTRL_NODE_DSI                    (0x13U)
#define DSS_DCTRL_NODE_DISCSYNC0              (0x14U)
#define DSS_DCTRL_NODE_DISCSYNC1              (0x15U)
/* @} */

/** \brief Defines maximum number of nodes for allocation including invalid node
 */
#define DSS_DCTRL_MAX_NODES                   (22U)

/** \brief Defines maximum number of edges for allocation. This is derived by
 *         looking at all possible DSS connections in the SoC.
 */
#define DSS_DCTRL_MAX_EDGES                   (32U)

/**< \brief DSS Functional Interrupt Number for R5 */
#define DSS_FUNC_IRQ_DEFAULT_NUM              (52U)

/**< \brief DSS Safety Interrupt Number for R5 */
#define DSS_SAFE_IRQ_DEFAULT_NUM              (54U)

/**< \brief DSS Functional Interrupt Number for R5 */
#define DSS_SECURE_IRQ_DEFAULT_NUM            (56U)

/**
 *  \anchor Dss_EvtMgrInstId
 *  \name   Instance Ids for DSS Event Manager.
 *
 *  @{
 */
/**< \brief Instance Id for functional interrupts */
#define DSS_EVT_MGR_INST_ID_FUNC              ((uint32_t) 0x00U)
/**< \brief Instance Id for safety interrupts */
#define DSS_EVT_MGR_INST_ID_SAFETY            ((uint32_t) 0x01U)
/**< \brief Instance Id for security interrupts */
#define DSS_EVT_MGR_INST_ID_SECURITY          ((uint32_t) 0x02U)
/**< \brief Max Instance Id */
#define DSS_EVT_MGR_INST_ID_MAX               ((uint32_t) 0x03U)
/**< \brief Invalid Instance Id */
#define DSS_EVT_MGR_INST_ID_INVALID           ((uint32_t) 0xFFU)
/* @} */

/*
 *  SOC specific IOCTLs.
 */

/**
 *  \addtogroup DRV_DSS_DCTRL_IOCTL
 *  @{
 */
/**
 * \brief IOCTL to process Display port HPD.
 *
 *  This IOCTL can be used to process the display port HPD interrupt.
 *  Note: This IOCTL is supported only for J721E SoC.
 *
 * \param cmdArgs       [IN]  Pointer of type uint32_t.
 *                            Supported values: TRUE/FALSE
 * \param cmdArgsStatus [OUT] NULL
 *
 * \return  FVID2_SOK if successful, else suitable error code
 *
 */
#define IOCTL_DSS_DCTRL_PROCESS_DP_HPD        (DSS_DCTRL_SOC_IOCTL_BASE + 0x01U)

/**
 * \brief IOCTL to register Display port HPD callback.
 *
 *  This IOCTL can be used to register the callback for
 *  display port HPD interrupt. Refer #Dss_DctrlDpHpdCbParams
 *  for details.
 *  Note: This IOCTL is supported only for J721E SoC.
 *
 * \param cmdArgs       [IN]  Pointer of type #Dss_DctrlDpHpdCbParams.
 * \param cmdArgsStatus [OUT] NULL
 *
 * \return  FVID2_SOK if successful, else suitable error code
 *
 */
#define IOCTL_DSS_DCTRL_REGISTER_DP_HPD_CB    (DSS_DCTRL_SOC_IOCTL_BASE + 0x02U)
/* @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Structure containing DSS interrupt information. Events should be
 *         enabled only for available Video Ports and Video Pipes
 */
typedef struct
{
    uint32_t dssCommonRegionId;
    /**< DSS Common Region Id. There are multiple common regions in DSS hardware
     *   each having IRQ aggregation registers and IRQ generator. This enables
     *   fully independent monitoring/control of the interrupt events by
     *   different hosts. User can decide which common region to use. By default
     *   region 0 will be used. Refer \ref CSL_DssCommRegId for values
     */
    uint32_t numValidIrq;
    /**< Number of valid DSS interrupts. There can be multiple interrupts in DSS
     *   hardware. This specifies number of valid IRQs in below array.
     */
    uint32_t irqNum[DSS_EVT_MGR_INST_ID_MAX];
    /**< DSS interrupt numbers. This will enable to configure different interrupt
     *   numbers. This will be initialized to default values as specified in the
     *   DssInitParams_init function. User may override these values if allowed.
     *
     *   Note: Driver will only register for the specified interrupt numbers.
     *   If needed the corresponding crossbar mapping for the device interrupt
     *   should be done by the application.
     */
} Dss_IrqParams;

/**
 *  \brief Structure containing resources manager information. This enables
 *         display sharing between two different softwares.
 */
typedef struct
{
    uint32_t isCommRegAvailable[CSL_DSS_COMM_REG_ID_MAX];
    /**< Ids for available common regions */
    uint32_t isPipeAvailable[CSL_DSS_VID_PIPE_ID_MAX];
    /**< Ids for available video pipes */
    uint32_t isOverlayAvailable[CSL_DSS_OVERLAY_ID_MAX];
    /**< Ids for available overlays */
    uint32_t isPortAvailable[CSL_DSS_VP_ID_MAX];
    /**< Ids for available video ports */
} Dss_RmInfo;

/**
 * \brief Structure containing Display Port init parameters
 */
typedef struct
{
    uint32_t isAvailable;
    /**< Flag to indicate whether eDP module is available */
    uint32_t isHpdSupported;
    /**< Flag to indicate whether driver should detect and
     *   handle hot-plug interrupts
     */
} Dss_DpInitParams;

/**
 *  \brief DSS SOC parameters.
 */
typedef struct
{
    Dss_IrqParams irqParams;
    /**< DSS interrupt information */
    Dss_RmInfo rmInfo;
    /**< DSS resource information */
    Dss_DpInitParams dpInitParams;
    /**< DSS DP init information */
} Dss_SocParams;

/**
  *  \brief DSS DP HPD Callback parameters
  */
typedef struct
{
    Dss_DctrlDpHpdCbFxn hpdCbFxn;
    /**< HPD callback function. Refer #Dss_DctrlDpHpdCbFxn */
    void *appData;
    /**< Application data */
} Dss_DctrlDpHpdCbParams;

/* ========================================================================== */
/*                  Internal/Private Function Declarations                    */
/* ========================================================================== */

/**
 *  \brief Check if the display driver instance is of type Video pipeline.
 *
 *  \param  instId  [IN]Driver Instance Id.
 *
 *  \return TRUE if instance is Video pipeline else returns FALSE.
 */
static inline uint32_t Dss_dispIsVidInst(uint32_t instId);

/**
 *  \brief Check if the display driver instance is of type Video lite pipeline.
 *
 *  \param  instId  [IN]Driver Instance Id.
 *
 *  \return TRUE if instance is Video lite pipeline else returns FALSE.
 */
static inline uint32_t Dss_dispIsVidLInst(uint32_t instId);

/**
 *  \brief Dss_IrqParams structure init function.
 *
 *  \param  irqParams      Pointer to #Dss_IrqParams structure.
 *
 *  \return None
 */
static inline void Dss_irqParamsInit(Dss_IrqParams *irqParams);

/**
 *  \brief Dss_RmInfo structure init function.
 *
 *  \param  rmInfo      Pointer to #Dss_RmInfo structure.
 *
 *  \return None
 */
static inline void Dss_rmInfoInit(Dss_RmInfo *rmInfo);

/**
 *  \brief Dss_DpInitParams structure init function.
 *
 *  \param  dpInitParams      Pointer to #Dss_DpInitParams structure.
 *
 *  \return None
 */
static inline void Dss_dpInitParamsInit(Dss_DpInitParams *dpInitParams);

/**
 *  \brief Dss_SocParams structure init function.
 *
 *  \param  socParams   Pointer to #Dss_SocParams structure.
 *
 *  \return None
 */
static inline void Dss_socParamsInit(Dss_SocParams *socParams);

/**
 *  \brief Dss_DctrlDpHpdCbParams structure init function.
 *
 *  \param  cbParams   Pointer to #Dss_DctrlDpHpdCbParams structure.
 *
 *  \return None
 */
static inline void Dss_dctrlDpHpdCbParamsInit(Dss_DctrlDpHpdCbParams *cbParams);

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline uint32_t Dss_dispIsVidInst(uint32_t instId)
{
    uint32_t isVidInst = FALSE;

    if((DSS_DISP_INST_VID1 == instId) ||
       (DSS_DISP_INST_VID2 == instId))
    {
        isVidInst = TRUE;
    }

    return (isVidInst);
}

static inline uint32_t Dss_dispIsVidLInst(uint32_t instId)
{
    uint32_t isVidLInst = FALSE;

    if((DSS_DISP_INST_VIDL1 == instId) ||
       (DSS_DISP_INST_VIDL2 == instId))
    {
        isVidLInst = TRUE;
    }

    return (isVidLInst);
}

static inline void Dss_irqParamsInit(Dss_IrqParams *irqParams)
{
    if(NULL != irqParams)
    {
        irqParams->dssCommonRegionId = CSL_DSS_COMM_REG_ID_0;
        irqParams->numValidIrq = DSS_EVT_MGR_INST_ID_MAX;
        irqParams->irqNum[DSS_EVT_MGR_INST_ID_FUNC] = DSS_FUNC_IRQ_DEFAULT_NUM;
        irqParams->irqNum[DSS_EVT_MGR_INST_ID_SAFETY] =
                                                    DSS_SAFE_IRQ_DEFAULT_NUM;
        irqParams->irqNum[DSS_EVT_MGR_INST_ID_SECURITY] =
                                                    DSS_SECURE_IRQ_DEFAULT_NUM;
    }
}

static inline void Dss_rmInfoInit(Dss_RmInfo *rmInfo)
{
    uint32_t i = 0U;
    if(NULL != rmInfo)
    {
        for(i=CSL_DSS_COMM_REG_ID_0; i<CSL_DSS_COMM_REG_ID_MAX; i++)
        {
            rmInfo->isCommRegAvailable[i] = TRUE;
        }
        for(i=CSL_DSS_VID_PIPE_ID_VID1; i<CSL_DSS_VID_PIPE_ID_MAX; i++)
        {
            rmInfo->isPipeAvailable[i] = TRUE;
        }
        for(i=CSL_DSS_OVERLAY_ID_1; i<CSL_DSS_OVERLAY_ID_MAX; i++)
        {
            rmInfo->isOverlayAvailable[i] = TRUE;
        }
        for(i=CSL_DSS_VP_ID_1; i<CSL_DSS_VP_ID_MAX; i++)
        {
            rmInfo->isPortAvailable[i] = TRUE;
        }
    }
}

static inline void Dss_dpInitParamsInit(Dss_DpInitParams *dpInitParams)
{
    if(NULL != dpInitParams)
    {
        dpInitParams->isAvailable = TRUE;
        dpInitParams->isHpdSupported = TRUE;
    }
}

static inline void Dss_socParamsInit(Dss_SocParams *socParams)
{
    if(NULL != socParams)
    {
        Dss_irqParamsInit(&(socParams->irqParams));
        Dss_rmInfoInit(&(socParams->rmInfo));
        Dss_dpInitParamsInit(&(socParams->dpInitParams));
    }
}

static inline void Dss_dctrlDpHpdCbParamsInit(Dss_DctrlDpHpdCbParams *cbParams)
{
    if(NULL != cbParams)
    {
        cbParams->hpdCbFxn = NULL;
        cbParams->appData = NULL;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef DSS_SOC_V1_H_ */

/* @} */
