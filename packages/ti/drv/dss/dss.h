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
 *  \defgroup DRV_DSS_MODULE DSS Driver
 *
 *  @{
 *
 * The Display Subsystem (DSS) provides the logic to interface display
 * peripherals. This is DSS FVID2 driver documentation.
 */
/* @} */

/**
 *  \ingroup DRV_DSS_MODULE
 *  \defgroup DSS_TOP_LEVEL DSS Driver Header
 *            This is DSS's top level include for applications
 *
 *  @{
 */

/**
 *  \file dss.h
 *
 *  \brief DSS Driver API/interface file.
 */

#ifndef DSS_H_
#define DSS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_dss.h>
#include <ti/drv/fvid2/fvid2.h>
#include <ti/drv/dss/include/dss_cfg.h>
#include <ti/drv/dss/soc/dss_soc.h>
#include <ti/drv/dss/include/dss_disp.h>
#include <ti/drv/dss/include/dss_dctrl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Display driver ID used at the time of FVID2 create. */
#define DSS_DISP_DRV_ID                     (FVID2_DSS_DRV_BASE + 0x00U)

/** \brief Display controller driver ID used at the time of FVID2 create. */
#define DSS_DCTRL_DRV_ID                    (FVID2_DSS_DRV_BASE + 0x01U)

/*
 *  IOCTLs Base address.
 */
/** \brief IOCTL base address for the display driver IOCTLs. */
#define DSS_DISP_IOCTL_BASE                 (FVID2_DSS_DRV_IOCTL_BASE + 0x000U)
/** \brief IOCTL base address for the display controller driver IOCTLs. */
#define DSS_DCTRL_IOCTL_BASE                (FVID2_DSS_DRV_IOCTL_BASE + 0x100U)
/** \brief IOCTL base address for the SOC specific display driver IOCTLs. */
#define DSS_DISP_SOC_IOCTL_BASE             (DSS_DISP_IOCTL_BASE + 0x40U)
/** \brief IOCTL base address for the SOC specific display controller driver
 *   IOCTLs. */
#define DSS_DCTRL_SOC_IOCTL_BASE            (DSS_DCTRL_IOCTL_BASE + 0x40U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief DSS initialization parameters.
 */
typedef struct
{
    Dss_SocParams socParams;
    /**< SoC parameters. Refer #Dss_SocParams for details */
} Dss_InitParams;

/* ========================================================================== */
/*                  Internal/Private Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Dss_InitParams structure init function.
 *
 *  \param  dssParams  Pointer to #Dss_InitParams structure.
 *
 *  \return None
 */
void Dss_initParamsInit(Dss_InitParams *dssParams);

/**
 *  \brief DSS initialization function.
 *   This function initializes the DSS hardware and drivers.
 *   This function should be called before calling any driver APIs and
 *   only once.
 *
 *  \param initParams  Pointer to a #Dss_InitParams structure
 *                     containing the DSS configuration
 *
 *  \return FVID2_SOK if successful, else suitable error code
 */
int32_t Dss_init(const Dss_InitParams *initParams);

/**
 *  \brief DSS de-initialization function.
 *   This function un-initializes the DSS hardware and drivers.
 *   This function should be called during system shutdown if Dss_init()
 *   was called by the application.
 *
 *  \return FVID2_SOK if successful, else suitable error code
 */
int32_t Dss_deInit(void);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef DSS_H_ */

/* @} */
