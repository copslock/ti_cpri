/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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
 *  \file sciclient_dummy.c
 *
 *  \brief File containing the SCICLIENT driver APIs for platforms not yet
 *  dmsc firmware is supported.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/sciclient/src/sciclient_priv.h>
#include <ti/csl/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern void Sciclient_rmInit(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Sciclient_loadFirmware(const uint32_t *pSciclient_firmware)
{
    int32_t  status   = CSL_PASS;

    return status;
}

int32_t Sciclient_init(const Sciclient_ConfigPrms_t *pCfgPrms)
{
    int32_t   status = CSL_PASS;

    Sciclient_rmInit();

    return status;
}

int32_t Sciclient_service(const Sciclient_ReqPrm_t *pReqPrm,
                          Sciclient_RespPrm_t      *pRespPrm)
{
    int32_t           status       = CSL_PASS;

    return status;
}

int32_t Sciclient_deinit(void)
{
    int32_t   status = CSL_PASS;

    return status;
}

int32_t Sciclient_pmSetModuleClkFreq(uint32_t moduleId,
                                     uint32_t clockId,
                                     uint64_t freqHz,
                                     uint32_t additionalFlag,
                                     uint32_t timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_pmSetModuleState(uint32_t moduleId,
                                   uint32_t state,
                                   uint32_t reqFlag,
                                   uint32_t timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_pmGetModuleState(uint32_t  moduleId,
                                   uint32_t *moduleState,
                                   uint32_t *resetState,
                                   uint32_t *contextLossState,
                                   uint32_t  timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_pmIsModuleValid(uint32_t modId)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_pmSetModuleRst(uint32_t moduleId,
                                 uint32_t resetBit,
                                 uint32_t timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_procBootGetProcessorState(
            uint8_t processorId,
            struct tisci_msg_proc_get_status_resp * procStatus,
            uint32_t  timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_procBootReleaseProcessor(uint8_t  processorId,
                                           uint32_t reqFlag,
                                           uint32_t  timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_procBootRequestProcessor(uint8_t processorId,
                                           uint32_t timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_procBootSetProcessorCfg (
            const struct tisci_msg_proc_set_config_req * configReq,
            uint32_t timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_procBootSetSequenceCtrl(uint8_t  processorId,
                                          uint32_t control_flags_1_set,
                                          uint32_t control_flags_1_clear,
                                          uint32_t reqFlag,
                                          uint32_t timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_procBootWaitProcessorState(
            uint8_t  processorId,
            uint8_t  num_match_iterations,
            uint8_t  delay_per_iteration_us,
            uint32_t status_flags_1_set_all_wait,
            uint32_t status_flags_1_set_any_wait,
            uint32_t status_flags_1_clr_all_wait,
            uint32_t status_flags_1_clr_any_wait,
            uint32_t reqFlag,
            uint32_t timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

int32_t Sciclient_procBootAuthAndStart(
            const struct tisci_msg_proc_auth_boot_req * authBootCfg,
            uint32_t timeout)
{
    int32_t retVal = CSL_PASS;
    return retVal;
}

#if defined (BUILD_MCU1_0)
int32_t Sciclient_boardCfg(Sciclient_BoardCfgPrms_t * pInPrms)
{
    int32_t retVal = CSL_PASS;

    return retVal;
}

int32_t Sciclient_boardCfgPm(Sciclient_BoardCfgPrms_t * pInPrms)
{
    int32_t retVal = CSL_PASS;

    return retVal;
}

int32_t Sciclient_boardCfgRm(Sciclient_BoardCfgPrms_t * pInPrms)
{
    int32_t retVal = CSL_PASS;

    return retVal;
}

int32_t Sciclient_boardCfgSec(Sciclient_BoardCfgPrms_t * pInPrms)
{
    int32_t retVal = CSL_PASS;

   return retVal;
}

#endif

