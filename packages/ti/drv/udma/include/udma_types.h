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
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_TYPE_MODULE UDMA Driver Common Data types
 *            This is UDMA driver common parameters and API
 *
 *  @{
 */

/**
 *  \file udma_types.h
 *
 *  \brief UDMA Low Level Driver API/interface data types file.
 */

#ifndef UDMA_TYPES_H_
#define UDMA_TYPES_H_

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

/** \brief UDMA driver handle */
typedef struct Udma_DrvObj *            Udma_DrvHandle;
/** \brief UDMA channel handle */
typedef struct Udma_ChObj *             Udma_ChHandle;
/** \brief UDMA event handle */
typedef struct Udma_EventObj *          Udma_EventHandle;
/** \brief UDMA ring handle */
typedef struct Udma_RingObj *           Udma_RingHandle;
/** \brief UDMA flow handle */
typedef struct Udma_FlowObj *           Udma_FlowHandle;
/** \brief UDMA proxy handle */
typedef struct Udma_ProxyObj *          Udma_ProxyHandle;
/** \brief UDMA ring monitor handle */
typedef struct Udma_RingMonObj *        Udma_RingMonHandle;

/** \brief Cache line size for alignment of descriptor and buffers */
#define UDMA_CACHELINE_ALIGNMENT        (128U)

/**
 *  \anchor Udma_ErrorCodes
 *  \name UDMA Error Codes
 *
 *  Error codes returned by UDMA APIs
 *
 *  @{
 */
/** \brief API call successful */
#define UDMA_SOK                        (CSL_PASS)
/** \brief API call returned with error as failed. Used for generic error.
 *  It may be some hardware failure and/or software failure. */
#define UDMA_EFAIL                      (CSL_EFAIL)
/** \brief API call returned with error as bad arguments.
 *  Typically, NULL pointer passed to the API where its not expected. */
#define UDMA_EBADARGS                   (CSL_EBADARGS)
/** \brief API call returned with error as invalid parameters. Typically
 *  when parameters passed are not valid or out of range. */
#define UDMA_EINVALID_PARAMS            (CSL_EINVALID_PARAMS)
/** \brief API call returned with error as timed out. Typically API is
 *  waiting for some condition and returned as condition not happened
 *  in the timeout period. */
#define UDMA_ETIMEOUT                   (CSL_ETIMEOUT)
/** \brief API call returned with error as allocation failed. */
#define UDMA_EALLOC                     (CSL_EALLOC)
/* @} */

/** \brief UDMA transaction timeout define - no wait */
#define UDMA_NO_WAIT                    ((uint32_t) 0U)
/** \brief UDMA transaction timeout define - wait for ever */
#define UDMA_WAIT_FOREVER               (~((uint32_t) 0U))

/** \brief Macro used to specify that init is performed for an object. */
#define UDMA_INIT_DONE                  (0xABDCABCDU)
/** \brief Macro used to specify that deinit is performed for an object. */
#define UDMA_DEINIT_DONE                (0x00000000U)

/**
 *  \anchor Udma_UtcId
 *  \name UDMA UTC ID
 *
 *  This represents the various UTC IP in the SOC. The actual UTC present
 *  in the chip is SOC dependent. Refer soc file for the actual instance
 *  present. Kindly use \ref Udma_UtcIdSoc macros for SOC specific name.
 *
 *  @{
 */
#define UDMA_UTC_ID0                    (0U)
#define UDMA_UTC_ID1                    (1U)
#define UDMA_UTC_ID2                    (2U)
#define UDMA_UTC_ID3                    (3U)
/* @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_TYPES_H_ */

/* @} */
