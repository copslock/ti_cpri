/*
 *  Copyright (C) 2018 Texas Instruments Incorporated
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
 *  \file sciclient_priv.h
 *
 *  \brief This file contains the handle structure used internally by sciclient.
 */
#ifndef SCICLIENT_PRIV_H_
#define SCICLIENT_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/csl_sec_proxy.h>
#include <ti/osal/osal.h>
#include <ti/osal/SemaphoreP.h>
#include <ti/osal/RegisterIntr.h>

#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/sciclient/src/sciclient_romMessages.h>
#include <ti/drv/sciclient/soc/sciclient_soc_priv.h>

#if defined (SOC_AM65XX)
#include <ti/drv/sciclient/soc/V0/sciclient_firmware_V0.h>
#include <ti/drv/sciclient/soc/V0/sciclient_defaultBoardcfg.h>
#endif

#if defined (SOC_J721E)
#include <ti/drv/sciclient/soc/V1/sciclient_firmware_V1.h>
#include <ti/drv/sciclient/soc/V1/sciclient_defaultBoardcfg.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *    \brief Maximum number of messages waiting to be read.
 *           Cannot be greater than 256.
 */
#define SCICLIENT_MAX_QUEUE_SIZE            (7U)

/* Current context is SECURE */
#define SCICLIENT_SECURE_CONTEXT            (0U)

/* Current context is NON-SECURE */
#define SCICLIENT_NON_SECURE_CONTEXT        (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Map structure used by #Sciclient_init function.
 */
typedef struct
{
    uint8_t context;
    /**< context(sec/non-sec) **/

    uint32_t hostId;
    /**< CPU ID of the A53/A72/R5F/DSP */

    uint32_t reqHighPrioThreadId;
    /**< Thread ID of the high priority thread(write) allowed for the CPU */

    uint32_t reqLowPrioThreadId;
    /**< Thread ID of the low priority thread(write) allowed for the CPU */

    uint32_t notificationRespThreadId;
    /**< Thread ID of the thread(write) for sending a notification to the
     *   firmware
     */

    uint32_t respThreadId;
    /**< Thread ID of the response thread(read) available for the CPU */

    uint32_t notificationThreadId;
    /**< Thread ID of the notification thread(read) available for the CPU */

    uint32_t respIntrNum;
    /**< Response Interrupt Number. */
} Sciclient_MapStruct_t;

/**
 *  \brief Handle for #Sciclient_service function
 */
typedef struct
{
    SemaphoreP_Handle     semHandles[SCICLIENT_MAX_QUEUE_SIZE];
    /**< Semaphore is posted when there is a interrupt for the response.
     *   Index is the currSeqId
     */
    SemaphoreP_Status     semStatus[SCICLIENT_MAX_QUEUE_SIZE];
    /**< Status returned for the SemaphoreP_pend corresponding to a
     *   particular seqId.
     */
    uint32_t              currSeqId;
    /**< Sequence ID of the current request **/
    HwiP_Handle           notificationIntr;
    /**<  Interrupt for notification **/
    HwiP_Handle           respIntr[2];
    /**<  Interrupt for response message. Have 2 for secure and non-secure context **/
    uint32_t              opModeFlag;
    /**< Operation mode for the Sciclient Service API. Refer to
     * \ref Sciclient_ServiceOperationMode for valid values.
     */
    uint8_t               initCount;
    /**< Count to keep track of the number of inits/de-inits done. Actual
     *   initialization done
     *   only when initCount=0, and de-init done only when initCount=1
     */
    uint32_t              isSecureMode;
    /**< Variable to check whether Core context is secure/non-secure. This has
     * to be given by the user via configParams. Default value is 0.
     */

} Sciclient_ServiceHandle_t;


/**
 *  \anchor Sciclient_proxyMap
 *  \name Sciclient map structure
 *  @{
 *  Map structure for R5F,A53,GPU and ICSSG \n
 *  in different contexts.
 */
extern const Sciclient_MapStruct_t gSciclientMap[SCICLIENT_CONTEXT_MAX_NUM];
/* @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**<
 *  \brief   API to send the board configuration messsage to the firmware .
 *          Valid(not NULL) pointer to Sciclient_BoardCfgPrms_t will use the
 *          provided values for tisci_msg_board_config_req, otherwise
 *          default values are used .
 *
 *  \param pInPrms   [IN] Pointer to #Sciclient_BoardCfgPrms_t .
 *
 *  \return  status    Status of the message.
 */
int32_t Sciclient_boardCfg(const Sciclient_BoardCfgPrms_t * pInPrms);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_PRIV_H_*/
