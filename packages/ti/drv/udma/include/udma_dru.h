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
 *  \defgroup DRV_UDMA_DRU_MODULE UDMA Driver DRU API
 *            This is UDMA driver DRU related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file udma_dru.h
 *
 *  \brief UDMA DRU related parameters and API.
 */

#ifndef UDMA_DRU_H_
#define UDMA_DRU_H_

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
 *  \anchor Udma_UtcType
 *  \name UDMA UTC Type
 *
 *  This represents the various types of UTC present in the SOC.
 *
 *  @{
 */
#define UDMA_UTC_TYPE_DRU               (0U)
#define UDMA_UTC_TYPE_DRU_VHWA          (1U)
/* @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief   This API does a direct TR submission to the specified channel and
 *  core ID.
 *
 *  Note: No error checks are performed by this API to get maximum performance
 *
 *  Requirement: DOX_REQ_TAG(PDK-2593)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *  \param tr           [IN] Pointer to TR to be submitted.
 */
void Udma_chDruSubmitTr(Udma_ChHandle chHandle, const CSL_UdmapTR *tr);

/**
 *  \brief   This API returns the number of queues present in a DRU.
 *
 *  This could be used to configure all the queues in a DRU after UDMA init.
 *
 *  Requirement: TODO
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *  \param utcId        [IN] The UTC instance ID.
 *                           Refer \ref Udma_UtcIdSoc macro for details.
 *
 *  \return \ref Udma_ErrorCodes
 */
uint32_t Udma_druGetNumQueue(Udma_DrvHandle drvHandle, uint32_t utcId);

/**
 *  \brief   This API configures the DRU queue non-real time configurations.
 *
 *  Note: DRU queue configuration should be done before any data transfer
 *  is initiated in a system. So this should be done preferably after
 *  #Udma_init is called.
 *
 *  Requirement: TODO
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *  \param utcId        [IN] The UTC instance ID.
 *                           Refer \ref Udma_UtcIdSoc macro for details.
 *  \param queueId      [IN] Queue ID - 0 to (#CSL_DRU_NUM_QUEUE - 1).
 *                           Refer \ref CSL_DruQueueId.
 *                           All the DRUs only have 5 queues  implemented in
 *                           the current design.
 *                           1 Priority queue and 4 round robin queues.
 *                           The priority queue is queue 0 the round robin
 *                           queues are queues 1 - 4.
 *  \param queueCfg     [IN] Pointer to queue configuration.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_druQueueConfig(Udma_DrvHandle               drvHandle,
                            uint32_t                     utcId,
                            uint32_t                     queueId,
                            const CSL_DruQueueConfig    *queueCfg);

/**
 *  \brief This API returns the software triggers register address for the DRU
 *  channel.
 *
 *  This can be used to directly write to the register to trigger the channel
 *  through software.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2594)
 *
 *  \param chHandle     [IN] UDMA channel handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
volatile uint64_t *Udma_druGetTriggerRegAddr(Udma_ChHandle chHandle);

/*
 * Structure Init functions
 */
/**
 *  \brief CSL_DruQueueConfig structure init function.
 *
 *  \param queueCfg     [IN] Pointer to #CSL_DruQueueConfig structure.
 *
 */
void UdmaDruQueueConfig_init(CSL_DruQueueConfig *queueCfg);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_DRU_H_ */

/* @} */
