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
 *  \file V0/sciclient_defaultBoardcfg.h
 *
 *  \brief File defining tisci_local_rm_boardcfg for boardCfg RM .
 *
 */

#ifndef SCICLIENT_DEFAULTBOARDCFG_
#define SCICLIENT_DEFAULTBOARDCFG_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/csl/csl_types.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/sciclient/soc/sysfw/include/tisci/tisci_boardcfg.h>
#include <ti/drv/sciclient/soc/sysfw/include/am65x/tisci_resasg_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#if defined (BUILD_MCU1_0)

/* Default board config structure */
extern const struct tisci_boardcfg gBoardConfigLow;

/* Default board config structure for RM*/
extern const struct tisci_local_rm_boardcfg gBoardConfigLow_rm;

/* Default board config structure for SECURITY */
extern const struct tisci_boardcfg_sec gBoardConfigLow_security;

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* \brief Structure to hold the RM board configuration */
struct tisci_local_rm_boardcfg {
    struct tisci_boardcfg_rm      rm_boardcfg;
    /**< Board configuration parameter */
    struct tisci_boardcfg_rm_resasg_entry resasg_entries[TISCI_BOARDCFG_RM_RESASG_ENTRIES_MAX];
    /**< Resource assignment entries */
};
#endif

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SCICLIENT_DEFAULTBOARDCFG_ */
