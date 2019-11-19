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
 *  \file ipc_soc.c
 *
 *  \brief File containing the IPC driver - soc specific implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/drv/ipc/ipc.h>
#include <ti/drv/ipc/soc/ipc_soc.h>
#include <ti/drv/ipc/src/ipc_priv.h>
#include <ti/drv/ipc/src/ipc_mailbox.h>

#include <ti/csl/csl_intr_router.h>
#include <ti/csl/csl_clec.h>
#include <ti/drv/sciclient/sciclient.h>
//#include <ti/drv/sciclient/soc/V1/sciclient_fmwMsgParams.h>

#define NAVSS_INTRTR_INPUT_MAILBOX0_USER0   (436U)
#define NAVSS_INTRTR_INPUT_MAILBOX1_USER0   (432U)
#define NAVSS_INTRTR_INPUT_MAILBOX2_USER0   (428U)
#define NAVSS_INTRTR_INPUT_MAILBOX3_USER0   (424U)
#define NAVSS_INTRTR_INPUT_MAILBOX4_USER0   (420U)
#define NAVSS_INTRTR_INPUT_MAILBOX5_USER0   (416U)
#define NAVSS_INTRTR_INPUT_MAILBOX6_USER0   (412U)
#define NAVSS_INTRTR_INPUT_MAILBOX7_USER0   (408U)
#define NAVSS_INTRTR_INPUT_MAILBOX8_USER0   (404U)
#define NAVSS_INTRTR_INPUT_MAILBOX9_USER0   (400U)
#define NAVSS_INTRTR_INPUT_MAILBOX10_USER0  (396U)
#define NAVSS_INTRTR_INPUT_MAILBOX11_USER0  (392U)

/**
 * \brief Main NavSS512 - Mailbox input line
 */
uint32_t g_Navss512MbInput[IPC_MAILBOX_CLUSTER_CNT] =
{
    NAVSS_INTRTR_INPUT_MAILBOX0_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX1_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX2_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX3_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX4_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX5_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX6_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX7_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX8_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX9_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX10_USER0,
    NAVSS_INTRTR_INPUT_MAILBOX11_USER0
};

/**
 *  \brief Processor IDs to name mapping for all processor in Jacinto7
 */
static Ipc_ProcInfo g_Ipc_mp_procInfo[IPC_MAX_PROCS] =
{
    {IPC_MPU1_0,      "mpu1_0"},      /**< ARM A72 - VM0 */
    {IPC_MCU1_0,      "mcu1_0"},      /**< ARM MCU  R5F - core0 */
    {IPC_MCU1_1,      "mcu1_1"},      /**< ARM MCU  R5F - core1 */
    {IPC_MCU2_0,      "mcu2_0"},      /**< ARM Main R5F - core0 */
    {IPC_MCU2_1,      "mcu2_1"},      /**< ARM Main R5F - core1 */
    {IPC_MCU3_0,      "mcu3_0"},      /**< ARM Main R5F - core2 */
    {IPC_MCU3_1,      "mcu3_1"},      /**< ARM Main R5F - core3 */
    {IPC_C66X_1,      "C66X_1"},      /**< DSP C66x - core0  */
    {IPC_C66X_2,      "C66X_2"},      /**< DSP C66x - core1  */
    {IPC_C7X_1,       "C7X_1"},       /**< DSP C7x - core0 */
    {IPC_MPU1_1,      "mpu1_1"}       /**< ARM A72 - VM1 */
};

/* Mailbox Cluster Base Address */
static uint32_t  g_IPC_Mailbox_BaseAddr[IPC_MAILBOX_CLUSTER_CNT] =
{
    CSL_NAVSS_MAIN_MAILBOX_REGS_0_BASE,     /* Mailbox - cluster0   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_1_BASE,     /* Mailbox - cluster1   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_2_BASE,     /* Mailbox - cluster2   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_3_BASE,     /* Mailbox - cluster3   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_4_BASE,     /* Mailbox - cluster4   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_5_BASE,     /* Mailbox - cluster5   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_6_BASE,     /* Mailbox - cluster6   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_7_BASE,     /* Mailbox - cluster7   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_8_BASE,     /* Mailbox - cluster8   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_9_BASE,     /* Mailbox - cluster9   */
    CSL_NAVSS_MAIN_MAILBOX_REGS_10_BASE,    /* Mailbox - cluster10  */
    CSL_NAVSS_MAIN_MAILBOX_REGS_11_BASE,    /* Mailbox - cluster11  */
};

static Ipc_MailboxInfo   g_IPC_MailboxInfo[IPC_MAX_PROCS][IPC_MAX_PROCS] =
{
    /* Host Processor - A72-vm0	*/
    {
        { { 0xFFU, 0xFFU,  0U}, { 0xFFU, 0xFFU, 0xFFU} },  /* Self - A72-vm0 */
        { {    0U,    0U,  0U}, {    0U,    0U,    1U} },  /* mcu-r5f0 */
        { {    0U,    0U,  2U}, {    0U,    0U,    3U} },  /* mcu-r5f1 */
        { {    1U,    0U,  0U}, {    1U,    0U,    1U} },  /* main-r5f0 */
        { {    1U,    0U,  2U}, {    1U,    0U,    3U} },  /* main-r5f1 */
        { {    2U,    0U,  0U}, {    2U,    0U,    1U} },  /* main-r5f2 */
        { {    2U,    0U,  2U}, {    2U,    0U,    3U} },  /* main-r5f3 */
        { {    3U,    0U,  0U}, {    3U,    0U,    1U} },  /* C66x-0 */
        { {    3U,    0U,  2U}, {    3U,    0U,    3U} },  /* C66x-1 */
        { {    4U,    0U,  0U}, {    4U,    0U,    1U} },  /* C7x-1 */
        { {    0U,    0U, 10U}, {    0U,    0U,   11U} }   /* A72-vm1 */
    },
    /* Host Processor - mcu1_0 	*/
    {
        { {    0U,    1U,  1U }, {    0U,    1U,  0U} },  /* A72-vm0 */
        { { 0xFFU, 0xFFU,  0U }, { 0xFFU, 0xFFU,  0U} },  /* Self - mcu-r5f0 */
        { {    0U,    1U,  4U }, {    0U,    1U,  5U} },  /* mcu-r5f1 */
        { {    7U,    0U,  0U }, {    5U, 0xFFU,  2U} },  /* main-r5f0 */
        { {    7U,    0U,  1U }, {    5U, 0xFFU, 10U} },  /* main-r5f1 */
        { {    7U,    0U,  2U }, {    6U, 0xFFU,  2U} },  /* main-r5f2 */
        { {    7U,    0U,  3U }, {    6U, 0xFFU, 10U} },  /* main-r5f3 */
        { {    7U,    1U,  4U }, {    8U, 0xFFU,  4U} },  /* C66x-0 */
        { {    7U,    1U,  5U }, {    8U, 0xFFU, 12U} },  /* C66x-1 */
        { {    7U,    1U,  6U }, {    9U, 0xFFU,  4U} },  /* C7x-1 */
        { {    0U,    1U,  7U }, {    0U,    1U,  6U} }   /* A72-vm1 */
    },
    /* Host Processor - mcu1_1 */
    {
        { {    0U,    2U,  3U }, {    0U,    2U,  2U} },  /* A72-vm0 */
        { {    0U,    2U,  5U }, {    0U,    2U,  4U} },  /* mcu-r5f0 */
        { { 0xFFU, 0xFFU,  0U }, { 0xFFU, 0xFFU,  0U} },  /* Self - mcu-r5f1 */
        { {    7U,    2U,  8U }, {    5U, 0xFFU,  3U} },  /* main-r5f0 */
        { {    7U,    2U,  9U }, {    5U, 0xFFU, 11U} },  /* main-r5f1 */
        { {    7U,    2U, 10U }, {    6U, 0xFFU,  3U} },  /* main-r5f2 */
        { {    7U,    2U, 11U }, {    6U, 0xFFU, 11U} },  /* main-r5f3 */
        { {    7U,    3U, 12U }, {    8U, 0xFFU,  5U} },  /* C66x-0 */
        { {    7U,    3U, 13U }, {    8U, 0xFFU, 13U} },  /* C66x-1 */
        { {    7U,    3U, 14U }, {    9U, 0xFFU,  5U} },  /* C7x-1 */
        { {    0U,    2U,  9U }, {    0U,    2U,  8U} }   /* A72-vm1 */
    },
    /* Host Processor - mcu2_0  */
    {
        { {    1U,    1U,  1U}, {    1U,    1U, 0U} },  /* A72-vm0 */
        { {    5U,    0U,  2U}, {    7U, 0xFFU, 0U} },  /* mcu-r5f0 */
        { {    5U,    0U,  3U}, {    7U, 0xFFU, 8U} },  /* mcu-r5f1 */
        { { 0xFFU, 0xFFU,  0U}, { 0xFFU, 0xFFU, 0U} },  /* Self - main-r5f0 */
        { {    1U,    1U,  4U}, {    1U,    1U, 5U} },  /* main-r5f1 */
        { {    5U,    0U,  0U}, {    6U, 0xFFU, 0U} },  /* main-r5f2 */
        { {    5U,    0U,  1U}, {    6U, 0xFFU, 8U} },  /* main-r5f3 */
        { {    5U,    1U,  4U}, {    8U, 0xFFU, 0U} },  /* C66x-0 */
        { {    5U,    1U,  5U}, {    8U, 0xFFU, 8U} },  /* C66x-1 */
        { {    5U,    1U,  6U}, {    9U, 0xFFU, 0U} },  /* C7x-1 */
        { {    1U,    1U,  7U}, {    1U,    1U, 6U} }   /* A72-vm1 */
    },
    /* Host Processor - mcu2_1 */
    {
        { {    1U,    2U,  3U }, {    1U,    2U, 2U} },  /* A72-vm0 */
        { {    5U,    2U, 10U }, {    7U, 0xFFU, 1U} },  /* mcu-r5f0 */
        { {    5U,    2U, 11U }, {    7U, 0xFFU, 9U} },  /* mcu-r5f1 */
        { {    1U,    2U,  5U }, {    1U,    2U, 4U} },  /* main-r5f0 */
        { { 0xFFU, 0xFFU,  0U }, { 0xFFU, 0xFFU, 0U} },  /* Self - main-r5f1 */
        { {    5U,    2U,  8U }, {    6U, 0xFFU, 1U} },  /* main-r5f2 */
        { {    5U,    2U,  9U }, {    6U, 0xFFU, 9U} },  /* main-r5f3 */
        { {    5U,    3U, 12U }, {    8U, 0xFFU, 1U} },  /* C66x-0 */
        { {    5U,    3U, 13U }, {    8U, 0xFFU, 9U} },  /* C66x-1 */
        { {    5U,    3U, 14U }, {    9U, 0xFFU, 1U} },  /* C7x-1 */
        { {    0U,    2U,  9U }, {    0U,    2U, 8U} }   /* A72-vm1 */
    },
    /* Host Processor - mcu3_0 */
    {
        { {    2U,    1U,  1U }, {    2U,    1U,  0U} },  /* A72-vm0 */
        { {    6U,    0U,  2U }, {    7U, 0xFFU,  2U} },  /* mcu-r5f0 */
        { {    6U,    0U,  3U }, {    7U, 0xFFU, 10U} },  /* mcu-r5f1 */
        { {    6U,    0U,  0U }, {    5U, 0xFFU,  0U} },  /* main-r5f0 */
        { {    6U,    0U,  1U }, {    5U, 0xFFU,  8U} },  /* main-r5f1 */
        { { 0xFFU, 0xFFU,  0U }, { 0xFFU, 0xFFU,  0U} },  /* Self - main-r5f2 */
        { {    2U,    1U,  4U }, {    2U,    1U,  5U} },  /* main-r5f3 */
        { {    6U,    1U,  4U }, {    8U, 0xFFU,  2U} },  /* C66x-0 */
        { {    6U,    1U,  5U }, {    8U, 0xFFU, 10U} },  /* C66x-1 */
        { {    6U,    1U,  6U }, {    9U, 0xFFU,  2U} },  /* C7x-1 */
        { {    2U,    1U,  7U }, {    2U,    1U,  6U} }   /* A72-vm1 */
    },
    /* Host Processor - mcu3_1	*/
    {
        { {    2U,    2U,  3U }, {    2U,    2U,  2U} },  /* A72-vm0 */
        { {    6U,    2U, 10U }, {    7U, 0xFFU,  3U} },  /* mcu-r5f0 */
        { {    6U,    2U, 11U }, {    7U, 0xFFU, 11U} },  /* mcu-r5f1 */
        { {    6U,    2U,  8U }, {    5U, 0xFFU,  1U} },  /* main-r5f0 */
        { {    6U,    2U,  9U }, {    5U, 0xFFU,  9U} },  /* main-r5f1 */
        { {    2U,    2U,  5U }, {    2U,    2U,  4U} },  /* main-r5f2 */
        { { 0xFFU, 0xFFU,  0U }, { 0xFFU, 0xFFU,  0U} },  /* Self - main-r5f3 */
        { {    6U,    3U, 12U }, {    8U, 0xFFU,  3U} },  /* C66x-0 */
        { {    6U,    3U, 13U }, {    8U, 0xFFU, 11U} },  /* C66x-1 */
        { {    6U,    3U, 14U }, {    9U, 0xFFU,  3U} },  /* C7x-1 */
        { {    2U,    2U,  9U }, {    2U,    2U,  8U} }   /* A72-vm1 */
    },
    /* Host Processor - c66xdsp_1	*/
    {
        { {    3U,    1U,  1U}, {    3U,    1U,  0U} },  /* A72-vm0 */
        { {    8U,    1U,  4U}, {    7U, 0xFFU,  4U} },  /* mcu-r5f0 */
        { {    8U,    1U,  5U}, {    7U, 0xFFU, 12U} },  /* mcu-r5f1 */
        { {    8U,    0U,  0U}, {    5U, 0xFFU,  4U} },  /* main-r5f0 */
        { {    8U,    0U,  1U}, {    5U, 0xFFU, 12U} },  /* main-r5f1 */
        { {    8U,    0U,  2U}, {    6U, 0xFFU,  4U} },  /* main-r5f2 */
        { {    8U,    0U,  3U}, {    6U, 0xFFU, 12U} },  /* main-r5f3 */
        { { 0xFFU, 0xFFU,  0U}, { 0xFFU, 0xFFU,  0U} },  /* Self - C66x-0 */
        { {    3U,    1U,  4U}, {    3U,    1U,  5U} },  /* C66x-1 */
        { {    8U,    1U,  6U}, {    9U, 0xFFU,  6U} },  /* C7x-1 */
        { {    3U,    1U,  7U}, {    3U,    1U,  6U} }   /* A72-vm1 */
    },
    /* Host Processor - c66xdsp_2	*/
    {
        { {    3U,    2U,  3U}, {    3U,    2U,  2U} },  /* A72-vm0 */
        { {    8U,    3U, 12U}, {    7U, 0xFFU,  5U} },  /* mcu-r5f0 */
        { {    8U,    3U, 13U}, {    7U, 0xFFU, 13U} },  /* mcu-r5f1 */
        { {    8U,    2U,  8U}, {    5U, 0xFFU,  5U} },  /* main-r5f0 */
        { {    8U,    2U,  9U}, {    5U, 0xFFU, 13U} },  /* main-r5f1 */
        { {    8U,    2U, 10U}, {    6U, 0xFFU,  5U} },  /* main-r5f2 */
        { {    8U,    2U, 11U}, {    6U, 0xFFU, 13U} },  /* main-r5f3 */
        { {    3U,    2U,  5U}, {    3U,    2U,  4U} },  /* C66x-0 */
        { { 0xFFU, 0xFFU,  0U}, { 0xFFU, 0xFFU,  0U} },  /* Self - C66x-1 */
        { {    8U,    3U, 14U}, {    9U, 0xFFU,  7U} },  /* C7x-1 */
        { {    3U,    2U,  9U}, {    3U,    2U,  8U} }   /* A72-vm1 */
    },
    /* Host Processor - c7x_1	*/
    {
        { {    4U,    1U,  1U}, {    4U,    1U,  0U} },  /* A72-vm0 */
        { {    9U,    1U,  4U}, {    7U, 0xFFU,  6U} },  /* mcu-r5f0 */
        { {    9U,    1U,  5U}, {    7U, 0xFFU, 14U} },  /* mcu-r5f1 */
        { {    9U,    0U,  0U}, {    5U, 0xFFU,  6U} },  /* main-r5f0 */
        { {    9U,    0U,  1U}, {    5U, 0xFFU, 14U} },  /* main-r5f1 */
        { {    9U,    0U,  2U}, {    6U, 0xFFU,  6U} },  /* main-r5f2 */
        { {    9U,    0U,  3U}, {    6U, 0xFFU, 14U} },  /* main-r5f3 */
        { {    9U,    1U,  6U}, {    8U, 0xFFU,  6U} },  /* C66x-0 */
        { {    9U,    1U,  7U}, {    8U, 0xFFU, 14U} },  /* C66x-1 */
        { { 0xFFU, 0xFFU,  0U}, { 0xFFU, 0xFFU,  0U} },  /* Self - C7x-1 */
        { {    4U,    1U,  7U}, {    4U,    1U,  6U} }   /* A72-vm1 */
    },
    /* Host Processor - A72-vm1	*/
    {
        { {    0U,    3U, 11U}, {    0U,    3U, 10U} },  /* A72-vm0 */
        { {    3U,    3U,  6U}, {    3U,    3U,  7U} },  /* C66x-0 */
        { {    3U,    3U,  8U}, {    3U,    3U,  9U} },  /* C66x-1 */
        { {    1U,    3U,  6U}, {    1U,    3U,  7U} },  /* main-r5f0 */
        { {    1U,    3U,  8U}, {    1U,    3U,  9U} },  /* main-r5f1 */
        { {    2U,    3U,  6U}, {    2U,    3U,  7U} },  /* main-r5f2 */
        { {    2U,    3U,  8U}, {    2U,    3U,  9U} },  /* main-r5f3 */
        { {    0U,    3U,  6U}, {    0U,    3U,  7U} },  /* mcu-r5f0 */
        { {    0U,    3U,  8U}, {    0U,    3U,  9U} },  /* mcu-r5f1 */
        { {    4U,    3U,  6U}, {    4U,    3U,  7U} },  /* C7x-1 */
        { { 0xFFU, 0xFFU,  0U}, { 0xFFU, 0xFFU,  0U} }   /* Self - A72-vm1 */
    }
};

uint32_t Ipc_getNavss512MailboxInputIntr(uint32_t clusterId, uint32_t userId);
int32_t Ipc_setCoreEventId(uint32_t selfId, Ipc_MbConfig* cfg, uint32_t intrCnt);


int32_t Ipc_getMailboxInfoTx(uint32_t selfId, uint32_t remoteId,
                 uint32_t *clusterId, uint32_t *userId, uint32_t *queueId)
{
    int32_t retVal = -1;

    if( (selfId < IPC_MAX_PROCS) &&
        (remoteId < IPC_MAX_PROCS))
    {
        Ipc_MailboxInfo   *pMailboxInfo = &g_IPC_MailboxInfo[selfId][remoteId];

        *clusterId = pMailboxInfo->tx.cluster;
        *userId    = pMailboxInfo->tx.user;
        *queueId   = pMailboxInfo->tx.fifo;
        retVal = 0;
    }

    return retVal;
}

int32_t Ipc_getMailboxInfoRx(uint32_t selfId, uint32_t remoteId,
                 uint32_t *clusterId, uint32_t *userId, uint32_t *queueId)
{
    int32_t retVal = -1;

    if( (selfId < IPC_MAX_PROCS) &&
        (remoteId < IPC_MAX_PROCS))
    {
        Ipc_MailboxInfo   *pMailboxInfo = &g_IPC_MailboxInfo[selfId][remoteId];

        *clusterId = pMailboxInfo->rx.cluster;
        *userId    = pMailboxInfo->rx.user;
        *queueId   = pMailboxInfo->rx.fifo;
        retVal = 0;
    }

    return retVal;

}

uint32_t Ipc_getMailboxBaseAddr(uint32_t custerId)
{
    uint32_t baseAddr = 0x00000000U;

    if( custerId < IPC_MAX_PROCS)
    {
        baseAddr = g_IPC_Mailbox_BaseAddr[custerId];
    }

    return baseAddr;
}

uint32_t Ipc_getNavss512MailboxInputIntr(uint32_t clusterId, uint32_t userId)
{
    uint32_t   mailboxIntrNum = 0U;

    if( (clusterId != MAILBOX_CLUSTER_INVALID) &&
        (clusterId < IPC_MAILBOX_CLUSTER_CNT)  &&
        (userId != MAILBOX_USER_INVALID)       &&
        (userId < IPC_MAILBOX_USER_CNT))
    {
        mailboxIntrNum = g_Navss512MbInput[clusterId] + userId;
    }
    return mailboxIntrNum;
}

int32_t Ipc_setCoreEventId(uint32_t selfId, Ipc_MbConfig* cfg, uint32_t intrCnt)
{
    int32_t    retVal          = IPC_SOK;
    uint32_t   outIntrBaseNum  = 0;
    uint32_t   vimEventBaseNum = 0;

    switch(selfId)
    {
        case IPC_MPU1_0:
            outIntrBaseNum = NAVSS512_MPU1_0_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = MPU1_0_MBINTR_OFFSET;
            break;
        case IPC_MCU1_0:
            outIntrBaseNum = NAVSS512_MCU1R5F0_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = MCU1R5F0_MBINTR_OFFSET;
            break;
        case IPC_MCU1_1:
            outIntrBaseNum = NAVSS512_MCU1R5F1_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = MCU1R5F1_MBINTR_OFFSET;
            break;
        case IPC_MCU2_0:
            outIntrBaseNum = NAVSS512_MCU2R5F0_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = MCU2R5F0_MBINTR_OFFSET;
            break;
        case IPC_MCU2_1:
            outIntrBaseNum = NAVSS512_MCU2R5F1_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = MCU2R5F1_MBINTR_OFFSET;
            break;
        case IPC_MCU3_0:
            outIntrBaseNum = NAVSS512_MCU3R5F0_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = MCU3R5F0_MBINTR_OFFSET;
            break;
        case IPC_MCU3_1:
            outIntrBaseNum = NAVSS512_MCU3R5F1_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = MCU3R5F1_MBINTR_OFFSET;
            break;
        case IPC_C66X_1:
            outIntrBaseNum = NAVSS512_C66X1_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = C66X1_MBINTR_OFFSET;
            break;
        case IPC_C66X_2:
            outIntrBaseNum = NAVSS512_C66X2_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = C66X2_MBINTR_OFFSET;
            break;
        case IPC_C7X_1:
            outIntrBaseNum = NAVSS512_C7X1_INPUT_MAILBOX_OFFSET;
            vimEventBaseNum = C7X1_MBINTR_OFFSET;
            break;
        default:
            break;
    }
    cfg->outputIntrNum = outIntrBaseNum + intrCnt;
    cfg->eventId       = vimEventBaseNum + intrCnt;

    return retVal;
}


int32_t Ipc_getMailboxIntrRouterCfg(uint32_t selfId, uint32_t clusterId,
        uint32_t userId, Ipc_MbConfig* cfg, uint32_t cnt)
{
    int32_t    retVal         = IPC_SOK;
    uint32_t   mailboxIntrNum = 0;

    /* Get Navss512 input interrupt number for mailbox */
    mailboxIntrNum = Ipc_getNavss512MailboxInputIntr(clusterId, userId);

    cfg->inputIntrNum  = mailboxIntrNum;
    cfg->priority      = 1U;
    retVal = Ipc_setCoreEventId(selfId, cfg, cnt);

    return retVal;
}

const char* Ipc_getCoreName(uint32_t procId)
{
    char*     p = (char*)0;
    uint32_t id = procId;

    if(id < IPC_MAX_PROCS)
    {
       p = g_Ipc_mp_procInfo[id].name;
    }
    return p;
}

#if  defined(BUILD_C66X_1) || defined(BUILD_C66X_2)
void Ipc_configC66xIntrRouter(uint32_t input)
{
    volatile uint32_t *intr_router = (volatile uint32_t *)IPC_C66X_INTR_VA_BASE;
    volatile uint32_t *RAT = (volatile uint32_t *)IPC_C66X_RAT_BASE;
    uint32_t           inputBase   = 0U;
    uint32_t           outputBase  = 0U;
    uint32_t           mbIntrBase  = 0U;
    uint32_t           outputPin   = 0U;
    uint32_t           inputPin    = 0U;

    /* program virtual address to REGION_BASE */
    RAT[1] = (int)intr_router;

    /* program C66_1/2 INTR_ROUTER physical addr to REGION_TRANS_L */
#ifdef BUILD_C66X_1
    RAT[2] = IPC_C66X_1_INTR_PA_BASE;
#endif
#ifdef BUILD_C66X_2
    RAT[2] = IPC_C66X_2_INTR_PA_BASE;
#endif

    /* enable region and set size to 512 B */
    RAT[0] = 0x80000009U;

#ifdef BUILD_C66X_1
    inputBase  = C66X1_MBINTR_INPUT_BASE;
    outputBase = C66X1_MBINTR_OUTPUT_BASE;
    mbIntrBase = C66X1_MBINTR_OFFSET;
#endif

#ifdef BUILD_C66X_2
    inputBase  = C66X2_MBINTR_INPUT_BASE;
    outputBase = C66X2_MBINTR_OUTPUT_BASE;
    mbIntrBase = C66X2_MBINTR_OFFSET;
#endif

    inputPin  = inputBase  + (input - mbIntrBase);
    outputPin = outputBase + (input - mbIntrBase);

#ifdef SUPPORT_C66X_BIT0
    outputPin++;
#endif

    /* Mailbox Interrupt Router */
    intr_router[inputPin+1] = (0x1 << 16) + (outputPin & 0xFFFF);
}
#endif

#if defined(BUILD_C7X_1)
void Ipc_configClecRouter(uint32_t corePackEvent)
{
    uint32_t              input;
    CSL_ClecEventConfig   cfgClec;
    CSL_CLEC_EVTRegs     *clecBaseAddr = (CSL_CLEC_EVTRegs*)C7X_CLEC_BASE_ADDR;

    input = C7X1_MBINTR_OUTPUT_BASE + (corePackEvent - C7X1_MBINTR_OFFSET);

    /* Configure CLEC */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = TRUE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_CPU_ALL;
    cfgClec.extEvtNum         = 0U;
    cfgClec.c7xEvtNum         = corePackEvent;
    CSL_clecConfigEvent(clecBaseAddr, input, &cfgClec);
}
#endif

#ifdef IPC_SUPPORT_SCICLIENT

/* Indexed list of dst ids */
const uint8_t map_dst_id[] =
{
    TISCI_DEV_COMPUTE_CLUSTER0_GIC500SS,
    TISCI_DEV_MCU_R5FSS0_CORE0,
    TISCI_DEV_MCU_R5FSS0_CORE1,
    TISCI_DEV_R5FSS0_CORE0,
    TISCI_DEV_R5FSS0_CORE1,
    TISCI_DEV_R5FSS1_CORE0,
    TISCI_DEV_R5FSS1_CORE1,
    TISCI_DEV_C66SS0_CORE0,
    TISCI_DEV_C66SS1_CORE0,
    TISCI_DEV_COMPUTE_CLUSTER0_CLEC
};

/* Indexed list of src ids */
const uint16_t map_src_id[] =
{
    TISCI_DEV_NAVSS0_MAILBOX_0,
    TISCI_DEV_NAVSS0_MAILBOX_1,
    TISCI_DEV_NAVSS0_MAILBOX_2,
    TISCI_DEV_NAVSS0_MAILBOX_3,
    TISCI_DEV_NAVSS0_MAILBOX_4,
    TISCI_DEV_NAVSS0_MAILBOX_5,
    TISCI_DEV_NAVSS0_MAILBOX_6,
    TISCI_DEV_NAVSS0_MAILBOX_7,
    TISCI_DEV_NAVSS0_MAILBOX_8,
    TISCI_DEV_NAVSS0_MAILBOX_9,
    TISCI_DEV_NAVSS0_MAILBOX_10,
    TISCI_DEV_NAVSS0_MAILBOX_11,
};

/* Indexed list of host ids */
const uint16_t map_host_id[] =
{
    TISCI_HOST_ID_A72_0,
    TISCI_HOST_ID_R5_0,
    TISCI_HOST_ID_R5_2,
    TISCI_HOST_ID_MAIN_0_R5_0,
    TISCI_HOST_ID_MAIN_0_R5_2,
    TISCI_HOST_ID_MAIN_1_R5_0,
    TISCI_HOST_ID_MAIN_1_R5_2,
    TISCI_HOST_ID_C6X_0_1,
    TISCI_HOST_ID_C6X_1_1,
    TISCI_HOST_ID_C7X_1
};


int32_t Ipc_sciclientIrqRelease(uint16_t coreId, uint32_t clusterId,
        uint32_t userId, uint32_t intNumber)
{
    int32_t                               retVal = IPC_SOK;
    struct tisci_msg_rm_irq_release_req   rmIrqRel;

    rmIrqRel.ia_id                  = 0U;
    rmIrqRel.vint                   = 0U;
    rmIrqRel.global_event           = 0U;
    rmIrqRel.vint_status_bit_index  = 0U;

    rmIrqRel.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                              TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID |
                              TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
    rmIrqRel.src_id         = map_src_id[clusterId];
    rmIrqRel.src_index      = (uint16_t)userId;
    rmIrqRel.dst_id         = (uint16_t)map_dst_id[coreId];
#if defined(BUILD_C7X_1)
    rmIrqRel.dst_host_irq   = (uint16_t)(intNumber +  IPC_C7X_COMPUTE_CLUSTER_OFFSET);
#else
    rmIrqRel.dst_host_irq   = (uint16_t)intNumber;
#endif
    rmIrqRel.secondary_host = (uint8_t)map_host_id[coreId];

    retVal = Sciclient_rmIrqRelease(&rmIrqRel, IPC_SCICLIENT_TIMEOUT);

    return retVal;
}

int32_t Ipc_sciclientIrqSet(uint16_t coreId, uint32_t clusterId,
        uint32_t userId, uint32_t intNumber)
{
    int32_t                           retVal = IPC_SOK;
    struct tisci_msg_rm_irq_set_req   rmIrqReq;
    struct tisci_msg_rm_irq_set_resp  rmIrqResp;

    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;

    rmIrqReq.valid_params   = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                              TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID |
                              TISCI_MSG_VALUE_RM_SECONDARY_HOST_VALID;
    rmIrqReq.src_id         = map_src_id[clusterId];
    rmIrqReq.src_index      = (uint16_t)userId;
    rmIrqReq.dst_id         = (uint16_t)map_dst_id[coreId];
#if defined(BUILD_C7X_1)
    rmIrqReq.dst_host_irq   = (uint16_t)(intNumber +  IPC_C7X_COMPUTE_CLUSTER_OFFSET);
#else
    rmIrqReq.dst_host_irq   = (uint16_t)intNumber;
#endif
    rmIrqReq.secondary_host = (uint8_t)map_host_id[coreId];

    /* Config event */
    retVal = Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, IPC_SCICLIENT_TIMEOUT);

    return retVal;
}

#endif

uint32_t Ipc_getCoreId(void)
{
    uint32_t selfId =  IPC_INVALID_PROCID;

#if defined(BUILD_MPU1_0)
    selfId = IPC_MPU1_0;
#elif defined(BUILD_MCU1_0)
    selfId = IPC_MCU1_0;
#elif defined(BUILD_MCU1_1)
    selfId = IPC_MCU1_1;
#elif defined(BUILD_MCU2_0)
    selfId = IPC_MCU2_0;
#elif defined(BUILD_MCU2_1)
    selfId = IPC_MCU2_1;
#elif defined(BUILD_MCU3_0)
    selfId = IPC_MCU3_0;
#elif defined(BUILD_MCU3_1)
    selfId = IPC_MCU3_1;
#elif defined(BUILD_C66X_1)
    selfId = IPC_C66X_1;
#elif defined(BUILD_C66X_2)
    selfId = IPC_C66X_2;
#elif defined(BUILD_C7X_1)
    selfId = IPC_C7X_1;
#else
#error "Unsupported core Id"
#endif

    return (selfId);
}

uint32_t Ipc_isCacheCoherent(void)
{
    uint32_t isCacheCoherent;

#if defined (BUILD_MPU1_0) || defined (BUILD_C7X_1)
    isCacheCoherent = TRUE;
#else
    isCacheCoherent = FALSE;
#endif

    return (isCacheCoherent);
}
