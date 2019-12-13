/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  \file sciclient.c
 *
 *  \brief File containing the SCICLIENT driver APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/sciclient/src/sciclient_priv.h>
#include <ti/csl/soc.h>
#include <string.h> /*For memcpy*/
#include <ti/csl/csl_clec.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*Header size in words*/
#define SCICLIENT_HEADER_SIZE_IN_WORDS (sizeof (struct tisci_header) \
                                        / sizeof (uint32_t))

/** Indicate that this message is marked secure */
#define TISCI_MSG_FLAG_MASK    (TISCI_BIT(0) | TISCI_BIT(1))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief   Gives the address for status register for a particular thread.
 *
 *  \param   thread    Index of the thread.
 *
 *  \return  address   address of the thread status
 */
static inline uint32_t Sciclient_threadStatusReg(uint32_t thread);

/**
 *  \brief   Read a 32 bit word from the thread.
 *
 *  \param   thread    Index of the thread to be read from.
 *  \param   idx       Index of the word to be read from the thread.
 *
 *  \return  word      Value read back.
 */
static inline uint32_t Sciclient_readThread32(uint32_t thread, uint8_t idx);

/**
 *  \brief   Read the current thread count.
 *
 *  \param   thread    Index of the thread to be read from.
 *
 *  \return  word      Count read back.
 */
static inline uint32_t Sciclient_readThreadCount(uint32_t thread);

/**
 *  \brief   Validate thread has no errors and has space to accept the next
 *           message.
 *
 *  \param   thread    Index of the thread.
 *
 *  \return  status    Status of the message.
 */
static int32_t Sciclient_verifyThread(uint32_t thread);

/**
 *  \brief   Check if there are credits to write to the thread.
 *
 *  \param   thread    Index of the thread.
 *  \param   timeout   Wait for timeout if operation is complete.
 *
 *  \return  status    Status of the message.
 */
static int32_t Sciclient_waitThread(uint32_t thread, uint32_t timeout);

/**
 *  \brief   API to send the message to the thread.
 *
 *  \param   thread         Index of the thread.
 *  \param   pSecHeader     Pointer to the security header extension.
 *  \param   pHeader        Pointer to the header structure.
 *  \param   pPayload       Pointer to the payload structure.
 *  \param   payloadSize    Size of the payload.
 *
 *  \return  status    Status of the message.
 */
static void Sciclient_sendMessage(uint32_t        thread,
                                  const uint8_t  *pSecHeader,
                                  const uint8_t  *pHeader,
                                  const uint8_t  *pPayload,
                                  uint32_t        payloadSize);

/**
 *  \brief   API to identify which mode the CPU is operating in. This utility
 *           function would read CPU related registers to know which mode
 *           (secure or non secure) the CPU is in and then would determine the
 *           context to be used. If more than one context is required for a
 *           a given code, users of SCICLENT would need to modify this function
 *           and recompile.
 *
 *  \param   messageType The Message ID to be checked.
 *
 *  \return  retVal     SCICLENT Context of the CPU
 */
static uint32_t Sciclient_getCurrentContext(uint16_t messageType);

/**
 *  \brief   This utility function would find the proxy map context id for
 *           'gSciclientMap' corresponding to a particular interrupt number.
 *
 *  \param   intrNum    Interrupt number.
 *
 *  \return  retVal     Context Id for the interrupt number.
 */
static int32_t Sciclient_contextIdFromIntrNum(uint32_t intrNum);

/**
 *  \brief   API to flush/remove all outstanding messages on a thread .
 *
 *  \param   thread    Index of the thread.
 *
 *  \return None
 */
static void Sciclient_flush(uint32_t thread);

/**
 *  \brief   ISR called when a response is received from DMSC.
 *
 *  \param   arg    Not used.
 *
 *  \return None
 */
static void Sciclient_ISR(uintptr_t arg);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/**
 *   \brief Handle used by #Sciclient_service function
 */
static Sciclient_ServiceHandle_t gSciclientHandle = (Sciclient_ServiceHandle_t){0};

/**
 *   \brief Size of secure header.This is initialized when the context is
 *          SECURE.
 */
static uint8_t gSecHeaderSizeWords = 0;

/**
 *   \brief Maximum size(bytes) of a sciclient message.
 */
static uint32_t gSciclient_maxMsgSizeBytes;

/** \brief This structure contains configuration parameters for
*       the sec_proxy IP */
#if defined (BUILD_MCU1_0) || defined (BUILD_MCU1_1)
CSL_SecProxyCfg gSciclient_secProxyCfg =
{
    (CSL_sec_proxyRegs *)CSL_MCU_NAVSS0_SEC_PROXY0_CFG_BASE,
    /*< pSecProxyRegs */
    (CSL_sec_proxy_scfgRegs *)CSL_MCU_NAVSS0_SEC_PROXY0_CFG_SCFG_BASE,
    /*< pSecProxyScfgRegs */
    (CSL_sec_proxy_rtRegs *)CSL_MCU_NAVSS0_SEC_PROXY0_CFG_RT_BASE,
    /*< pSecProxyRtRegs */
    (uint64_t)CSL_MCU_NAVSS0_SEC_PROXY0_TARGET_DATA_BASE,
    /*< proxyTargetAddr */
    0                                          // maxMsgSize
};

#else
CSL_SecProxyCfg gSciclient_secProxyCfg =
{
    (CSL_sec_proxyRegs *)CSL_NAVSS0_SEC_PROXY0_CFG_MMRS_BASE,
    /*< pSecProxyRegs */
    (CSL_sec_proxy_scfgRegs *)CSL_NAVSS0_SEC_PROXY0_CFG_SCFG_BASE,
    /*< pSecProxyScfgRegs */
    (CSL_sec_proxy_rtRegs *)CSL_NAVSS0_SEC_PROXY0_CFG_RT_BASE,
    /*< pSecProxyRtRegs */
    (uint64_t)CSL_NAVSS0_SEC_PROXY0_SRC_TARGET_DATA_BASE,
    /*< proxyTargetAddr */
    0                                          // maxMsgSize
};
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Sciclient_loadFirmware(const uint32_t *pSciclient_firmware)
{
    int32_t  status   = CSL_PASS;
    uint32_t txThread = SCICLIENT_ROM_R5_TX_NORMAL_THREAD;
    uint32_t rxThread = SCICLIENT_ROM_R5_RX_NORMAL_THREAD;
    Sciclient_RomFirmwareLoadHdr_t header      = {0};
    Sciclient_RomFirmwareLoadPayload_t payload = {0};
    uint32_t secHeaderSizeWords = sizeof(struct tisci_sec_header)/sizeof(uint32_t);

    volatile Sciclient_RomFirmwareLoadHdr_t *pLocalRespHdr =
        (Sciclient_RomFirmwareLoadHdr_t *)CSL_secProxyGetDataAddr
                                        (&gSciclient_secProxyCfg, rxThread, 0U);
    uint8_t  payloadSize = sizeof (Sciclient_RomFirmwareLoadPayload_t) /
                           sizeof (uint8_t);
    gSciclient_maxMsgSizeBytes = CSL_secProxyGetMaxMsgSize(&gSciclient_secProxyCfg) -
                                CSL_SEC_PROXY_RSVD_MSG_BYTES;

    /* Construct header */
    header.type = SCICLIENT_ROM_MSG_R5_TO_M3_M3FW;
    header.host = TISCI_HOST_ID_R5_1;
    /* ROM expects a sequence number of 0 */
    header.seq  = 0U;
    /* ROM doesn't check for flags */
    header.flags = 0U;

    if (pSciclient_firmware != NULL)
    {
        payload.bufferAddress = (uint32_t)(uintptr_t)pSciclient_firmware;

        /*Size is not needed actually.It is taken from x509 certificate*/
        payload.bufferSizeBytes = 0xffffffffU;

        /* Verify thread status before reading/writing */
        status = Sciclient_verifyThread(txThread);
        if (CSL_PASS == status)
        {
            status = Sciclient_waitThread(txThread, SCICLIENT_SERVICE_WAIT_FOREVER);
        }
        if (CSL_PASS == status)
        {
            /* Writing header and payload */
            Sciclient_sendMessage(txThread,NULL, (uint8_t *) &header,
                                  (uint8_t *)&payload, payloadSize);

            /* CHECKING FOR FIRMWARE LOAD ACK */
            /* Verify thread status before reading/writing */
            status = Sciclient_verifyThread(rxThread);
        }
        if (CSL_PASS == status)
        {
            while ((HW_RD_REG32(Sciclient_threadStatusReg(rxThread)) &
                 CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) == 0U) {;}
            /* Check the message type and flag of the response */
            if ((pLocalRespHdr->type ==
                SCICLIENT_ROM_MSG_M3_TO_R5_M3FW_RESULT)
                && (pLocalRespHdr->flags == SCICLIENT_ROM_MSG_CERT_AUTH_PASS))
            {
                status = CSL_PASS;
            }
            else
            {
                status = CSL_EFAIL;
            }
            /* Reading from the last register of rxThread*/
            (void) Sciclient_readThread32(rxThread,
                            (uint8_t)((gSciclient_maxMsgSizeBytes/4U)-1U));
        }

        /* CHECKING FOR TISCI_MSG_BOOT_NOTIFICATION from DMSC*/
        pLocalRespHdr =
        (Sciclient_RomFirmwareLoadHdr_t *)(CSL_secProxyGetDataAddr(
                                            &gSciclient_secProxyCfg, rxThread, 0U)
                                            + ((uintptr_t) secHeaderSizeWords * (uintptr_t) 4U));
        if (status == CSL_PASS)
        {
            status = Sciclient_verifyThread(rxThread);
        }
        if (status == CSL_PASS)
        {
            while ((HW_RD_REG32(Sciclient_threadStatusReg(rxThread)) &
                 CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) == 0U) {;}
            /* Check the message type and flag of the response */
            if (pLocalRespHdr->type ==
                TISCI_MSG_BOOT_NOTIFICATION)
            {
                status = CSL_PASS;
            }
            else
            {
                status = CSL_EFAIL;
            }
            /* Reading from the last register of rxThread*/
            (void) Sciclient_readThread32(rxThread,
                            (uint8_t)((gSciclient_maxMsgSizeBytes/4U)-1U));
        }
    }
    else
    {
        status = CSL_EFAIL;
    }

    return status;
}

int32_t Sciclient_init(const Sciclient_ConfigPrms_t *pCfgPrms)
{
    int32_t   status = CSL_PASS;
    uintptr_t key;
    uint32_t b_doInit = 0U;
    uint32_t rxThread;

    /* Updating gSciclientHandle.initCount is CRITICAL */
    key = HwiP_disable();
    gSciclientHandle.initCount++;
    if (gSciclientHandle.initCount == 1U)
    {
        b_doInit = 1U;
    }
    HwiP_restore(key);

    if(1U == b_doInit)
    {
        if (pCfgPrms != NULL)
        {
            /* Initialize Config params */
            if((pCfgPrms->opModeFlag ==
                SCICLIENT_SERVICE_OPERATION_MODE_POLLED) ||
                (pCfgPrms->opModeFlag ==
                SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT))
            {
                gSciclientHandle.opModeFlag = pCfgPrms->opModeFlag;
            }
            else
            {
                status = CSL_EBADARGS;
            }

            if( (CSL_PASS==status) && ((pCfgPrms->isSecureMode==0U) ||
                (pCfgPrms->isSecureMode==1U)) )
            {
                gSciclientHandle.isSecureMode = pCfgPrms->isSecureMode;
            }
            else
            {
                status = CSL_EBADARGS;
            }
        }
        else
        {
            gSciclientHandle.opModeFlag =
                    SCICLIENT_SERVICE_OPERATION_MODE_POLLED;
            gSciclientHandle.isSecureMode = 0U;
        }

        if ((gSciclientHandle.opModeFlag ==
             SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT) &&
            (status == CSL_PASS))
        {
            SemaphoreP_Params semParams = {NULL,
                                           SemaphoreP_Mode_BINARY,
                                           1U};
            uint32_t          i = 0U;
            /* Create Sciclient_ServiceHandle_t.semHandles */
            for (i = 0; i < SCICLIENT_MAX_QUEUE_SIZE; i++)
            {
                gSciclientHandle.semStatus[i] = SemaphoreP_OK;
                gSciclientHandle.semHandles[i] = SemaphoreP_create(0U,
                                                        &semParams);
                if (gSciclientHandle.semHandles[i] == NULL)
                {
                    status = CSL_EFAIL;
                    break;
                }
            }
            /* Initialize currSeqId. Make sure currSeqId is never 0 */
            gSciclientHandle.currSeqId = (uint8_t) 1;

            /* Register interrupts for secure and non-secure contexts of the CPU */
            /* Non-Secure */
            uint32_t contextId = Sciclient_getCurrentContext(TISCI_MSG_VERSION);
            if(contextId < SCICLIENT_CONTEXT_MAX_NUM)
            {
                OsalRegisterIntrParams_t    intrPrms;
                rxThread = gSciclientMap[contextId].respThreadId;
                Sciclient_flush(rxThread);
                Osal_RegisterInterrupt_initParams(&intrPrms);
                /* Populate the interrupt parameters */
                intrPrms.corepacConfig.arg              = (uintptr_t) &(gSciclientMap[contextId].respIntrNum);
                intrPrms.corepacConfig.isrRoutine       = &Sciclient_ISR;
                #if defined (_TMS320C6X)
                /* On C66x, we use Event Combiner to map the interrupt to the CPU Intc.  To
                 * do this, OSAL expects that event number holds the interrupt number and we
                 * use the macro for interrupt number to specify we wish to use Event
                 * Combiner.
                 */
                intrPrms.corepacConfig.corepacEventNum  = (int32_t) gSciclientMap[contextId].respIntrNum;
                intrPrms.corepacConfig.intVecNum        = OSAL_REGINT_INTVEC_EVENT_COMBINER;
                #else
                /* Other (non-C66x) CPUs don't use event number and interrupt number is
                 * passed in and programmed to CPU Intc directly.
                 */
                intrPrms.corepacConfig.corepacEventNum  = 0;
                intrPrms.corepacConfig.intVecNum        = (int32_t) gSciclientMap[contextId].respIntrNum;
                #endif
                #if defined (__C7100__)
                {
                    CSL_CLEC_EVTRegs * regs = (CSL_CLEC_EVTRegs *) CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE;
                    CSL_ClecEventConfig evtCfg;
                    evtCfg.secureClaimEnable = 0;
                    evtCfg.evtSendEnable = 1;
                    evtCfg.rtMap = 0x3C;
                    evtCfg.extEvtNum = 0x0;
                    evtCfg.c7xEvtNum = SCICLIENT_C7X_NON_SECURE_INTERRUPT_NUM;
                    /* Clec interrupt number 1024 is connected to GIC interrupt number 32 in J721E.
                     * Due to this for CLEC programming one needs to add an offset of 992 (1024 - 32)
                     * to the event number which is shared between GIC and CLEC.
                     */
                    CSL_clecConfigEvent(regs, CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_NAVSS0_INTR_ROUTER_0_OUTL_INTR_189 + 992, &evtCfg);
                    intrPrms.corepacConfig.priority = 1U;
                }
                #endif
                /* Clear Interrupt */
                Osal_ClearInterrupt(intrPrms.corepacConfig.corepacEventNum, intrPrms.corepacConfig.intVecNum);
                /* Register interrupts */
                status = Osal_RegisterInterrupt(&intrPrms, &gSciclientHandle.respIntr[0]);
                if(OSAL_INT_SUCCESS != status)
                {
                    gSciclientHandle.respIntr[0] = NULL_PTR;
                }
            }
            else
            {
                status = CSL_EFAIL;
            }
            /* Secure Context */
            contextId = Sciclient_getCurrentContext(TISCI_MSG_BOARD_CONFIG);
            if(contextId < SCICLIENT_CONTEXT_MAX_NUM)
            {
                OsalRegisterIntrParams_t    intrPrms;
                rxThread = gSciclientMap[contextId].respThreadId;
                Sciclient_flush(rxThread);
                Osal_RegisterInterrupt_initParams(&intrPrms);
                /* Populate the interrupt parameters */
                intrPrms.corepacConfig.arg              = (uintptr_t) &(gSciclientMap[contextId].respIntrNum);
                intrPrms.corepacConfig.isrRoutine       = &Sciclient_ISR;
                #if defined (_TMS320C6X)
                /* On C66x, we use Event Combiner to map the interrupt to the CPU Intc.  To
                 * do this, OSAL expects that event number holds the interrupt number and we
                 * use the macro for interrupt number to specify we wish to use Event
                 * Combiner.
                 */
                intrPrms.corepacConfig.corepacEventNum  = (int32_t) gSciclientMap[contextId].respIntrNum;
                intrPrms.corepacConfig.intVecNum        = OSAL_REGINT_INTVEC_EVENT_COMBINER;
                #else
                /* Other (non-C66x) CPUs don't use event number and interrupt number is
                 * passed in and programmed to CPU Intc directly.
                 */
                intrPrms.corepacConfig.corepacEventNum  = 0;
                intrPrms.corepacConfig.intVecNum        = (int32_t) gSciclientMap[contextId].respIntrNum;
                #endif
                #if defined (__C7100__)
                {
                    CSL_CLEC_EVTRegs * regs = (CSL_CLEC_EVTRegs *) CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE;
                    CSL_ClecEventConfig evtCfg;
                    evtCfg.secureClaimEnable = 0;
                    evtCfg.evtSendEnable = 1;
                    evtCfg.rtMap = 0x3C;
                    evtCfg.extEvtNum = 0x0;
                    evtCfg.c7xEvtNum = SCICLIENT_C7X_SECURE_INTERRUPT_NUM;
                    /* Clec interrupt number 1024 is connected to GIC interrupt number 32 in J721E.
                     * Due to this for CLEC programming one needs to add an offset of 992 (1024 - 32)
                     * to the event number which is shared between GIC and CLEC.
                     */
                    CSL_clecConfigEvent(regs, CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_NAVSS0_INTR_ROUTER_0_OUTL_INTR_191 + 992, &evtCfg);
                    intrPrms.corepacConfig.priority = 1U;
                }
                #endif
                /* Clear Interrupt */
                Osal_ClearInterrupt(intrPrms.corepacConfig.corepacEventNum, intrPrms.corepacConfig.intVecNum);
                /* Register interrupts */
                status = Osal_RegisterInterrupt(&intrPrms, &gSciclientHandle.respIntr[1]);
                if(OSAL_INT_SUCCESS != status)
                {
                    gSciclientHandle.respIntr[1] = NULL_PTR;
                }
            }
            else
            {
                status = CSL_EFAIL;
            }
        }
    }
    return status;
}

/* HWI_disable instead of semaphores for MCAL polling based. define in separate files*/
int32_t Sciclient_service(const Sciclient_ReqPrm_t *pReqPrm,
                          Sciclient_RespPrm_t      *pRespPrm)
{
    int32_t           status       = CSL_PASS;
    uint32_t          i            = 0U;
    uint32_t          initialCount = 0U;
    /* size of request payload in bytes  */
    uint32_t          txPayloadSize =0U;
    /* size of response payload in bytes */
    uint32_t          rxPayloadSize =0U;
    uint32_t         *pLocalRespPayload = NULL;
    volatile Sciclient_RomFirmwareLoadHdr_t *pLocalRespHdr;
    uint32_t          contextId = SCICLIENT_CONTEXT_MAX_NUM;
    uint32_t          txThread;
    uint32_t          rxThread;
    uint8_t           localSeqId;
    uintptr_t         key = 0U;
    uint32_t          timeToWait;
    struct tisci_header *header;
    uint8_t *pSecHeader = NULL;
    struct tisci_sec_header secHeader;
    uint32_t numWords = 0U;
    uint8_t  trailBytes = 0;

    /* Run all error checks */
    if((pReqPrm == NULL) || (pRespPrm == NULL))
    {
        status = CSL_EBADARGS;
    }
    else
    {
        contextId = Sciclient_getCurrentContext(pReqPrm->messageType);
        if(contextId < SCICLIENT_CONTEXT_MAX_NUM)
        {
            txThread = gSciclientMap[contextId].reqHighPrioThreadId;
            rxThread = gSciclientMap[contextId].respThreadId;
            if(gSciclientMap[contextId].context == SCICLIENT_SECURE_CONTEXT)
            {
                gSecHeaderSizeWords = sizeof(struct tisci_sec_header)/sizeof(uint32_t);
            }
            else
            {
                gSecHeaderSizeWords = 0;
            }
            gSciclient_maxMsgSizeBytes = CSL_secProxyGetMaxMsgSize(&gSciclient_secProxyCfg) -
                                        CSL_SEC_PROXY_RSVD_MSG_BYTES;

            if(gSciclientMap[contextId].context == SCICLIENT_SECURE_CONTEXT)
            {
                secHeader.integ_check = (uint16_t)0;
                secHeader.rsvd = (uint16_t)0;
                pSecHeader = (uint8_t * )(&secHeader);
            }
            if (pReqPrm->reqPayloadSize > 0U)
            {
                txPayloadSize = pReqPrm->reqPayloadSize - sizeof(struct tisci_header);
            }
            else
            {
                txPayloadSize = 0U;
            }
            if (txPayloadSize > gSciclient_maxMsgSizeBytes)
            {
                status = CSL_EBADARGS;
            }
            if ((txPayloadSize > 0U) && (pReqPrm->pReqPayload == NULL))
            {
                status = CSL_EBADARGS;
            }
            if (pRespPrm->respPayloadSize > 0U)
            {
                rxPayloadSize = pRespPrm->respPayloadSize - sizeof(struct tisci_header);
            }
            else
            {
                rxPayloadSize = 0U;
            }
            if (rxPayloadSize > gSciclient_maxMsgSizeBytes)
            {
                status = CSL_EBADARGS;
            }

            if ((rxPayloadSize > 0U) && (pRespPrm->pRespPayload == NULL))
            {
                status = CSL_EBADARGS;
            }
            else
            {
                pLocalRespPayload = (uint32_t *)(pRespPrm->pRespPayload + sizeof(struct tisci_header));
            }
        }
        else
        {
            status = CSL_EBADARGS;
        }

    }

    /* CRITICAL Section */
    key = HwiP_disable();

    if (CSL_PASS == status)
    {
        struct tisci_msg_version_req *dummyHdr = (struct tisci_msg_version_req *)pReqPrm->pReqPayload;
        /* Construct header */
        /*This is done to remove stray messages(due to timeout) in a thread
        * in case of "polling". */
        if (gSciclientHandle.opModeFlag ==
             SCICLIENT_SERVICE_OPERATION_MODE_POLLED)
        {
            Sciclient_flush(rxThread);
        }
        header = &dummyHdr->hdr;
        header->type = pReqPrm->messageType;
        header->host = (uint8_t) gSciclientMap[contextId].hostId;
        localSeqId = (uint8_t) gSciclientHandle.currSeqId;
        header->seq = localSeqId;
        header->flags = pReqPrm->flags;
        gSciclientHandle.currSeqId = (gSciclientHandle.currSeqId + 1U) %
                                    SCICLIENT_MAX_QUEUE_SIZE;
        if (gSciclientHandle.currSeqId == 0U)
        {
            gSciclientHandle.currSeqId++;
        }
    }

    if (status == CSL_PASS)
    {
        /* Verify thread status before reading/writing */
        status = Sciclient_verifyThread(txThread);
    }

    if (CSL_PASS == status)
    {
        status = Sciclient_waitThread(txThread, pReqPrm->timeout);
    }

    if (CSL_PASS == status)
    {
        /* Send Message */
        initialCount = Sciclient_readThreadCount(rxThread);
        Sciclient_sendMessage(txThread, pSecHeader ,(uint8_t *) header,
                              (pReqPrm->pReqPayload + sizeof(struct tisci_header)),
                              txPayloadSize);

        /* Verify thread status before reading/writing */
        status = Sciclient_verifyThread(rxThread);
    }
    if (CSL_PASS == status)
    {
        timeToWait = pReqPrm->timeout;
        pLocalRespHdr =
            (struct tisci_header *)(CSL_secProxyGetDataAddr(
                                            &gSciclient_secProxyCfg, rxThread, 0U)
                                    + ((uintptr_t) gSecHeaderSizeWords * (uintptr_t) 4U));
    }
    /* Wait for response: Polling based waiting */
    if ((gSciclientHandle.opModeFlag ==
         SCICLIENT_SERVICE_OPERATION_MODE_POLLED) &&
        (status == CSL_PASS) &&
        ((pReqPrm->flags & TISCI_MSG_FLAG_MASK) != 0U))
    {
        /* Check if some message is received*/
        while (((HW_RD_REG32(Sciclient_threadStatusReg(rxThread)) &
                CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) - initialCount) <= 0U)
        {
            if (timeToWait > 0U)
            {
                timeToWait--;
            }
            else
            {
                status = CSL_ETIMEOUT;
                break;
            }
        }
        if (status == CSL_PASS)
        {
            /* Check the seqId of response*/
            status = CSL_ETIMEOUT;
            timeToWait =  pReqPrm->timeout;
            while (timeToWait > 0U)
            {
                if ((pLocalRespHdr->seq == (uint32_t) localSeqId))
                {
                    status = CSL_PASS;
                    break;
                }
                timeToWait--;
            }
        }
    }
    HwiP_restore(key);

    /* Wait for response: Interrupt based waiting */
    if ((gSciclientHandle.opModeFlag ==
         SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT) &&
        (status == CSL_PASS))
    {
        status = SemaphoreP_pend(gSciclientHandle.semHandles[localSeqId],timeToWait);
        gSciclientHandle.semStatus[localSeqId] = (SemaphoreP_Status)status;
    }

    if(status == CSL_PASS)
    {
        numWords   = (uint32_t) (rxPayloadSize / 4U);
        trailBytes = (uint8_t) (rxPayloadSize - (numWords * 4U));
        /* Read the full message */
        pRespPrm->flags = Sciclient_readThread32(rxThread, 1U+gSecHeaderSizeWords);

        /*We do not need to read the header*/
        for (i = 0; i < numWords; i++)
        {
            *(pLocalRespPayload + i) = Sciclient_readThread32(
                rxThread,
                ((uint8_t) i +
                 SCICLIENT_HEADER_SIZE_IN_WORDS+gSecHeaderSizeWords));
        }

        if (trailBytes > 0U)
        {
            *(pLocalRespPayload + i) = Sciclient_readThread32(
                rxThread,
                ((uint8_t)i +
                 SCICLIENT_HEADER_SIZE_IN_WORDS+gSecHeaderSizeWords));
        }

        /* Read the last register of the rxThread */
        if ((((uint32_t) gSecHeaderSizeWords*4U) +
            (SCICLIENT_HEADER_SIZE_IN_WORDS*4U) +
            rxPayloadSize) <=
            (gSciclient_maxMsgSizeBytes - 4U))
        {
            (void) Sciclient_readThread32(rxThread,
                            (uint8_t)((gSciclient_maxMsgSizeBytes/4U) - 1U));
        }
    }

    if ((status == CSL_PASS) &&
        (gSciclientHandle.opModeFlag == SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT))
    {
        #if defined (_TMS320C6X)
        Osal_ClearInterrupt((int32_t) gSciclientMap[contextId].respIntrNum, OSAL_REGINT_INTVEC_EVENT_COMBINER);
        Osal_EnableInterrupt((int32_t) gSciclientMap[contextId].respIntrNum, OSAL_REGINT_INTVEC_EVENT_COMBINER);
        #else
        Osal_ClearInterrupt(0, (int32_t) gSciclientMap[contextId].respIntrNum);
        Osal_EnableInterrupt(0, (int32_t) gSciclientMap[contextId].respIntrNum);
        #endif
    }
    return status;
}

int32_t Sciclient_deinit(void)
{
    int32_t   status = CSL_PASS;
    /* gSciclientHandle.initCount is critical */
    uint32_t contextId;
    uint32_t doDeInit = 0;
    uintptr_t key = HwiP_disable();
    if (gSciclientHandle.initCount == 1U)
    {
        gSciclientHandle.initCount--;
        doDeInit = 1U;
    }
    else
    {
        gSciclientHandle.initCount--;
    }
    HwiP_restore(key);
    if (1U == doDeInit)
    {
        uint32_t i = 0U;
        if ((gSciclientHandle.opModeFlag ==
         SCICLIENT_SERVICE_OPERATION_MODE_INTERRUPT) &&
        (status == CSL_PASS))
        {
            /* Delete Sciclient_ServiceHandle_t.semHandles */
            for (i = 0U; i < SCICLIENT_MAX_QUEUE_SIZE; i++)
            {
                (void) SemaphoreP_delete(gSciclientHandle.semHandles[i]);
            }
            /* De-register interrupts */
            if (gSciclientHandle.respIntr[0] != NULL)
            {
                contextId = Sciclient_getCurrentContext(TISCI_MSG_VERSION);
                if(contextId < SCICLIENT_CONTEXT_MAX_NUM)
                {
                    (void) Osal_DeleteInterrupt(gSciclientHandle.respIntr[0], (int32_t) gSciclientMap[contextId].respIntrNum);
                }
            }
            if (gSciclientHandle.respIntr[1] != NULL)
            {
                contextId = Sciclient_getCurrentContext(TISCI_MSG_BOARD_CONFIG);
                if(contextId < SCICLIENT_CONTEXT_MAX_NUM)
                {
                    (void) Osal_DeleteInterrupt(gSciclientHandle.respIntr[1], (int32_t) gSciclientMap[contextId].respIntrNum);
                }
            }
        }
    }
    return status;
}

int32_t Sciclient_abiCheck(void)
{
    int32_t status = CSL_PASS;
    /* Send getRevision message for ABI Revision Check */
    /*THINK: What should be the appropriate timeout value here? */
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        NULL,
        0,
        SCICLIENT_SERVICE_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        (uint32_t) sizeof (response)
    };

    status = Sciclient_service(&reqPrm, &respPrm);
    if ((status != CSL_PASS) ||
        (respPrm.flags != TISCI_MSG_FLAG_ACK) ||
        ((uint32_t)(response.abi_major) != SCICLIENT_FIRMWARE_ABI_MAJOR) )
    {
        status = CSL_EFAIL;
    }

    return status;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

static void Sciclient_ISR(uintptr_t arg)
{
    int32_t contextId = Sciclient_contextIdFromIntrNum(*(uint32_t *)(arg));
    if(contextId  >= 0)
    {
        uint32_t rxThread = gSciclientMap[contextId].respThreadId;
        if(gSciclientMap[contextId].context == SCICLIENT_SECURE_CONTEXT)
        {
            gSecHeaderSizeWords = sizeof(struct tisci_sec_header)/sizeof(uint32_t);
        }
        else
        {
            gSecHeaderSizeWords = 0;
        }
        volatile Sciclient_RomFirmwareLoadHdr_t *pLocalRespHdr =
                (struct tisci_header *)(CSL_secProxyGetDataAddr(
                                                &gSciclient_secProxyCfg,rxThread,0U)
                                        + ((uintptr_t) gSecHeaderSizeWords * (uintptr_t) 4U));
        uint8_t seqId = pLocalRespHdr->seq;
        if ((gSciclientHandle.semStatus[seqId] == SemaphoreP_OK) && (seqId != 0U))
        {
            (void) SemaphoreP_post(gSciclientHandle.semHandles[seqId]);
            #if defined (_TMS320C6X)
            Osal_DisableInterrupt((int32_t) gSciclientMap[contextId].respIntrNum, OSAL_REGINT_INTVEC_EVENT_COMBINER);
            #else
            Osal_DisableInterrupt(0, (int32_t) gSciclientMap[contextId].respIntrNum);
            #endif
        }
        else
        {
            /* This implies that the SemaphoreP_pend for this seqId failed.
            *  So, we need to flush this message.*/
            (void) Sciclient_readThread32(rxThread,
                                (uint8_t)((gSciclient_maxMsgSizeBytes/4U) - 1U));
            gSciclientHandle.semStatus[seqId] = SemaphoreP_OK;
        }
    }
    //return;
}

static inline uint32_t Sciclient_threadStatusReg(uint32_t thread)
{
    return ((uint32_t)(uintptr_t)(gSciclient_secProxyCfg.pSecProxyRtRegs) +
        CSL_SEC_PROXY_RT_THREAD_STATUS(thread));
}

static inline uint32_t Sciclient_readThread32(uint32_t thread, uint8_t idx)
{
    uint32_t ret;
    ret = HW_RD_REG32(CSL_secProxyGetDataAddr(&gSciclient_secProxyCfg,thread,0U) +
        ((uintptr_t) (0x4U) * (uintptr_t) idx));
    return ret;
}

static inline uint32_t Sciclient_readThreadCount(uint32_t thread)
{
    return (HW_RD_REG32(Sciclient_threadStatusReg(thread)) &
        CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK);
}

static int32_t Sciclient_verifyThread(uint32_t thread)
{
    int32_t status = CSL_PASS;
    /* Verify thread status before reading/writing */
    if ((HW_RD_REG32(Sciclient_threadStatusReg(thread)) &
        CSL_SEC_PROXY_RT_THREAD_STATUS_ERROR_MASK) != 0U)
    {
        status = CSL_EFAIL;
    }
    return status;
}

static int32_t Sciclient_waitThread(uint32_t thread, uint32_t timeout)
{
    int32_t  status     = CSL_ETIMEOUT;
    uint32_t timeToWait = timeout;
    /* Checks the thread count is > 0 */
    while (timeToWait > 0U)
    {
        if ((HW_RD_REG32(Sciclient_threadStatusReg(thread)) &
            CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) > 0U)
        {
            status = CSL_PASS;
            break;
        }
        timeToWait--;
    }
    return status;
}

#if defined (__C7100__)
#ifdef __cplusplus
#pragma FUNCTION_OPTIONS("--opt_level=off")
#else
#pragma FUNCTION_OPTIONS(Sciclient_sendMessage, "--opt_level=off")
#endif
#endif
static void Sciclient_sendMessage(uint32_t        thread,
                                  const uint8_t  *pSecHeader,
                                  const uint8_t  *pHeader,
                                  const uint8_t  *pPayload,
                                  uint32_t        payloadSize)
{
    uint32_t        i   = 0U;
    const uint8_t *msg = pSecHeader;
    uint32_t numWords   = 0U;
    uint32_t test = 0U;
    uintptr_t threadAddr = CSL_secProxyGetDataAddr(&gSciclient_secProxyCfg, thread, 0U);

    if(pSecHeader != NULL)
    {
        /* Write secure header */
        for (i = 0U; i < gSecHeaderSizeWords; i++)
        {
            /*Change this when unaligned access is supported*/
            (void) memcpy((void *)&test, (const void *)msg, 4);
            CSL_REG32_WR(threadAddr, test);
            msg += 4;
            threadAddr+=sizeof(uint32_t);
        }
    }
    /* Write header */
    msg = pHeader;
    for (i = 0U; i < SCICLIENT_HEADER_SIZE_IN_WORDS; i++)
    {
        /*Change this when unaligned access is supported*/
        (void) memcpy((void *)&test, (const void *)msg, 4);
        CSL_REG32_WR(threadAddr, test);
        msg += 4;
        threadAddr+=sizeof(uint32_t);
    }
    /* Writing payload */
    if (payloadSize > 0U)
    {
        numWords   = (payloadSize+3U)/4U;
        msg = pPayload;
        for (; i < (SCICLIENT_HEADER_SIZE_IN_WORDS + numWords); i++)
        {
            /*Change this when unaligned access is supported*/
            (void) memcpy((void *)&test, (const void *)msg, 4);
            CSL_REG32_WR(threadAddr, test);
            msg += 4;
            threadAddr+=sizeof(uint32_t);
        }
    }
    /* Write to the last register of the TX thread */
    if ((((uint32_t) gSecHeaderSizeWords*4U)+(SCICLIENT_HEADER_SIZE_IN_WORDS*4U)+payloadSize) <=
        (gSciclient_maxMsgSizeBytes - 4U))
    {
        threadAddr = CSL_secProxyGetDataAddr(&gSciclient_secProxyCfg, thread, 0U) +
        ((uintptr_t) gSciclient_maxMsgSizeBytes  - (uintptr_t) 4U) ;
        CSL_REG32_WR(threadAddr,0U);
    }
}

static uint32_t Sciclient_getCurrentContext(uint16_t messageType)
{
    uint32_t retVal = SCICLIENT_CONTEXT_MAX_NUM;

    if((TISCI_MSG_BOOT_NOTIFICATION == messageType) ||
       (TISCI_MSG_BOARD_CONFIG == messageType) ||
       (TISCI_MSG_BOARD_CONFIG_RM == messageType) ||
       (TISCI_MSG_BOARD_CONFIG_SECURITY == messageType) ||
       (TISCI_MSG_BOARD_CONFIG_PM == messageType))
    {
        retVal = SCICLIENT_CONTEXT_SEC;
    }
    else
    {
        /* For all other message type use non-secure context */
        retVal = SCICLIENT_CONTEXT_NONSEC;
#if defined (BUILD_C7X_1)
        /* C7x is left in secure supervisor mode which causes
         * the non-secure thread access to fail.
         */
        if(gSciclientHandle.isSecureMode == 1U)
        {
            retVal = SCICLIENT_CONTEXT_SEC;
        }
#endif
    }

    return retVal;
}

static int32_t Sciclient_contextIdFromIntrNum(uint32_t intrNum)
{
    int32_t retVal = CSL_EFAIL;
    uint32_t i = 0U;
    while ((i < SCICLIENT_CONTEXT_MAX_NUM) &&
        (gSciclientMap[i].respIntrNum != intrNum))
    {
        i++;
    }
    if (i < SCICLIENT_CONTEXT_MAX_NUM)
    {
        retVal = (int32_t)i;
    }
    return retVal;
}

static void Sciclient_flush(uint32_t thread)
{
    while ((HW_RD_REG32(Sciclient_threadStatusReg(thread)) &
        CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK) > 0U)
    {
        /* Reading from the last register of rxThread*/
        (void) Sciclient_readThread32(thread,
                        (uint8_t)((gSciclient_maxMsgSizeBytes/4U)-1U));
    }

    return ;
}
