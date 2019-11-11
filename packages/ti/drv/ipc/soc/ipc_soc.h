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
 *  \file ipc_soc.h
 *
 *  \brief IPC Low Level Driver SOC specific file.
 */

#ifndef IPC_SOC_TOP_H_
#define IPC_SOC_TOP_H_

#ifdef __cplusplus
extern "C" {
#endif

#define    IPC_INVALID_PROCID (0XFFU)   /**< Invalid Proc ID */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */


/*
 * These functions and structure is for internal use use and
 * are not expected to be called from app
 */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t Ipc_getMailboxInfoTx(uint32_t selfId, uint32_t remoteId, 
                 uint32_t *clusterId, uint32_t *userId, uint32_t *queueId);
int32_t Ipc_getMailboxInfoRx(uint32_t selfId, uint32_t remoteId, 
                 uint32_t *clusterId, uint32_t *userId, uint32_t *queueId);
int32_t Ipc_getMailboxIntrRouterCfg(uint32_t selfId, uint32_t clusterId,
                 uint32_t userId, Ipc_MbConfig* cfg, uint32_t cnt);
uint32_t Ipc_getMailboxBaseAddr(uint32_t custerId);

/**
 * \brief Returns the core name for get core id
 *
 * \param procId [IN] Id of desired core.
 *
 * \return name of the given core id
 * */
const char* Ipc_getCoreName(uint32_t procId);

/**
 * \brief Returns Core ID based on core build flag
 *
 * \return Code ID of the current core
 **/
uint32_t Ipc_getCoreId(void);


/* For Maxwell Device */
#if defined (SOC_AM65XX)
#include <ti/drv/ipc/soc/V0/ipc_soc.h>
#endif

/* For J7ES device */
#if defined (SOC_J721E) || defined (SOC_AM77X)
#include <ti/drv/ipc/soc/V1/ipc_soc.h>
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_SOC_TOP_H_ */
