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
 *  \file sciclient_boardcfg.c
 *
 *  \brief File containing the APIs to send board configuration
 *       messages.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/sciclient/src/sciclient_priv.h>

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#if defined (BUILD_MCU1_0)
int32_t Sciclient_boardCfg(const Sciclient_BoardCfgPrms_t * pInPrms)
{
    int32_t retVal = CSL_PASS;
    struct tisci_msg_board_config_req request = {
        .boardcfgp_low  = (uint32_t) &gBoardConfigLow,
        .boardcfgp_high = (uint32_t) 0x0U,
        .boardcfg_size  = (uint16_t) sizeof(struct tisci_boardcfg)
    };

    /* NULL pInPrms will retain default values */
    if (pInPrms != NULL)
    {
        request.boardcfgp_low = pInPrms->boardConfigLow;
        request.boardcfgp_high = pInPrms->boardConfigHigh;
    }

    Sciclient_ReqPrm_t reqParam = {
        .messageType    = (uint16_t) TISCI_MSG_BOARD_CONFIG,
        .flags          = (uint32_t) TISCI_MSG_FLAG_AOP,
        .pReqPayload    = (const uint8_t *) &request,
        .reqPayloadSize = (uint32_t) sizeof (request),
        .timeout        = (uint32_t) SCICLIENT_SERVICE_WAIT_FOREVER
    };
    Sciclient_RespPrm_t respParam = {
        .flags           = (uint32_t) 0,   /* Populated by the API */
        .pRespPayload    = (uint8_t *) 0,
        .respPayloadSize = (uint32_t) 0
    };

    if((CSL_PASS != Sciclient_service(&reqParam, &respParam))
        || ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = CSL_EFAIL;
    }

    return retVal;
}

int32_t Sciclient_boardCfgPm(const Sciclient_BoardCfgPrms_t * pInPrms)
{
    int32_t retVal = CSL_PASS;
    struct tisci_msg_board_config_pm_req request = {
        .boardcfg_pmp_low  = (uint32_t) NULL, /* PM Board Config structure
                                                 definition removed from TISCI */
        .boardcfg_pmp_high = (uint32_t) 0x0U,
        .boardcfg_pm_size  = (uint16_t) 0x0
    };

    /* NULL pInPrms will retain default values */
    if (pInPrms != NULL)
    {
        request.boardcfg_pmp_low = pInPrms->boardConfigLow;
        request.boardcfg_pmp_high = pInPrms->boardConfigHigh;
        request.boardcfg_pm_size = pInPrms->boardConfigSize;
    }

    Sciclient_ReqPrm_t reqParam = {
        .messageType    = (uint16_t) TISCI_MSG_BOARD_CONFIG_PM,
        .flags          = (uint32_t) TISCI_MSG_FLAG_AOP,
        .pReqPayload    = (const uint8_t *) &request,
        .reqPayloadSize = (uint32_t) sizeof (request),
        .timeout        = (uint32_t) SCICLIENT_SERVICE_WAIT_FOREVER
    };
    Sciclient_RespPrm_t respParam = {
        .flags           = (uint32_t) 0,   /* Populated by the API */
        .pRespPayload    = (uint8_t *) 0,
        .respPayloadSize = (uint32_t) 0
    };

    if((CSL_PASS != Sciclient_service(&reqParam, &respParam))
        || ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = CSL_EFAIL;
    }
    return retVal;
}

int32_t Sciclient_boardCfgRm(const Sciclient_BoardCfgPrms_t * pInPrms)
{
    int32_t retVal = CSL_PASS;

    struct tisci_msg_board_config_rm_req request = {
        .boardcfg_rmp_low  = (uint32_t) &gBoardConfigLow_rm,
        .boardcfg_rmp_high = (uint32_t) 0x0U,
        .boardcfg_rm_size  = (uint16_t) sizeof(struct tisci_local_rm_boardcfg)
    };

    /* NULL pInPrms will retain default values */
    if (pInPrms != NULL)
    {
        request.boardcfg_rmp_low = pInPrms->boardConfigLow;
        request.boardcfg_rmp_high = pInPrms->boardConfigHigh;
        request.boardcfg_rm_size = pInPrms->boardConfigSize;
    }

    Sciclient_ReqPrm_t reqParam = {
        .messageType    = (uint16_t) TISCI_MSG_BOARD_CONFIG_RM,
        .flags          = (uint32_t) TISCI_MSG_FLAG_AOP,
        .pReqPayload    = (const uint8_t *) &request,
        .reqPayloadSize = (uint32_t) sizeof (request),
        .timeout        = (uint32_t) SCICLIENT_SERVICE_WAIT_FOREVER
    };
    Sciclient_RespPrm_t respParam = {
        .flags           = (uint32_t) 0,   /* Populated by the API */
        .pRespPayload    = (uint8_t *) 0,
        .respPayloadSize = (uint32_t) 0
    };

    if((CSL_PASS != Sciclient_service(&reqParam, &respParam))
        || ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = CSL_EFAIL;
    }
    return retVal;
}

int32_t Sciclient_boardCfgSec(const Sciclient_BoardCfgPrms_t * pInPrms)
{
    int32_t retVal = CSL_PASS;

    struct tisci_msg_board_config_security_req request = {
        .boardcfg_securityp_low  = (uint32_t) &gBoardConfigLow_security,
        .boardcfg_securityp_high = (uint32_t) 0x0U,
        .boardcfg_security_size  = (uint16_t) sizeof(struct tisci_boardcfg_sec)
    };

    /* NULL pInPrms will retain default values */
    if (pInPrms != NULL)
    {
        request.boardcfg_securityp_low = pInPrms->boardConfigLow;
        request.boardcfg_securityp_high = pInPrms->boardConfigHigh;
        request.boardcfg_security_size = pInPrms->boardConfigSize;
    }

    Sciclient_ReqPrm_t reqParam = {
        .messageType    = (uint16_t) TISCI_MSG_BOARD_CONFIG_SECURITY,
        .flags          = (uint32_t) TISCI_MSG_FLAG_AOP, 
        .pReqPayload    = (const uint8_t *) &request,
        .reqPayloadSize = (uint32_t) sizeof (request),
        .timeout        = (uint32_t) SCICLIENT_SERVICE_WAIT_FOREVER
    };
    Sciclient_RespPrm_t respParam = {
        .flags           = (uint32_t) 0,   /* Populated by the API */
        .pRespPayload    = (uint8_t *) 0,
        .respPayloadSize = (uint32_t) 0
    };

    if((CSL_PASS != Sciclient_service(&reqParam, &respParam))
        || ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK))
    {
        retVal = CSL_EFAIL;
    }
    return retVal;
}

#endif

