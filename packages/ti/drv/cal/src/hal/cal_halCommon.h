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
 *  \file cal_halCommon.h
 *
 *  \brief Common Header file for all CAL HALs. The CAL block is sub-divded into
 *  3 distinct functional blocks, namely, CAL - Capture, ISP - Image Processing,
 *  SIMCOP - Simple Image Co-Processor.
 *  This file defines common data-structures, enums, control functions, etc for
 *  all CAL HALs
 *
 */

#ifndef CAL_HALCOMMON_H_
#define CAL_HALCOMMON_H_

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

/** \brief Typedef for HAL handle. */
typedef void *Cal_HalHandle;

#define IOCTL_CAL_HAL_BEGIN             (0U)

/** \brief Common Reset ioctl, all modules that supports reset will
           use this ioctl */
#define IOCTL_CAL_HAL_RESET_ALL         (IOCTL_CAL_HAL_BEGIN + 0U)
/** \brief Common Start ioctl, all modules that supports start will
           use this ioctl */
#define IOCTL_CAL_HAL_START             (IOCTL_CAL_HAL_RESET_ALL + 1U)
/** \brief Common Stop ioctl, all modules that supports start will
           use this ioctl */
#define IOCTL_CAL_HAL_STOP              (IOCTL_CAL_HAL_START + 1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  struct Cal_HalInitParams
 *  \brief CAL HAL module initialization parameters.
 */
typedef struct Cal_HalInitParams
{
    uint32_t prms;
    /**< Init Params, not used */
} Cal_HalInitParams_t;

/**
 *  \brief Enum for selecting the processing mode, most of the CAL modules
 *         support two processing modes, ie free running mode and one
 *         shot mode.
 *         In one shot mode, module processes one frame and then stops, its
 *         enable bit resets to 0. It has to be re-enabled for processing
 *         next frame
 *         In Free Running mode, module continuously processing frame one
 *         after another. It can be stopped by writing 0 to the enable bit.
 */
typedef enum Cal_HalCtrlProcMode
{
    CAL_HAL_PROC_MODE_FREE_RUN = 0x0,
    /**< Free Running Mode */
    CAL_HAL_PROC_MODE_ONE_SHOT = 0x1,
    /**< One Shot processing mode */
    CAL_HAL_PROC_MODE_FORCE_INT = 0x7FFFFFFF
                                     /**< This will ensure enum is not packed,
                                             will always be contained in int */
} Cal_HalCtrlProcMode_t;

/**
 *  struct Cal_HalSubModules
 *  \brief CAL Sub Module identifiers
 */
typedef enum Cal_HalSubModules
{
    CAL_HAL_SUB_MODULE_LVDSRX = 0x01,
    /**< LVDS Rx module identifier */
    CAL_HAL_SUB_MODULE_CAL_A = 0x02,
    /**< CAL module identifier */
    CAL_HAL_SUB_MODULE_BYS_A = 0x04,
    /**< TBD TODO Remove this */
    CAL_HAL_SUB_MODULE_CAL_B = 0x08,
    /**< CAL module identifier */
    CAL_HAL_SUB_MODULE_BYS_B = 0x10,
    /**< TBD TODO Remove this */
    CAL_HAL_SUB_MODULE_ISP = 0x20,
    /**< Image Signal Processing module identifier */
    CAL_HAL_SUB_MODULE_SIMCOP = 0x40,
    /**< Simple Image Co-Processor module identifier */
    CAL_HAL_SUB_MODULE_MAX = 0x41,
    /**< End of enumeration marker */
    CAL_HAL_SUB_MODULE_CAL_FORCE_INT = 0x7FFFFFFF
                               /**< This will ensure enum is not packed, will always be contained in int */
} Cal_HalSubModules_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


#ifdef __cplusplus
}
#endif

#endif  /* #ifndef CAL_HALCOMMON_H_ */
