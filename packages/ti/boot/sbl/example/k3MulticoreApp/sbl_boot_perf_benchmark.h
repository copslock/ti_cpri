/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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

#include <ti/drv/sciclient/src/sciclient_priv.h>
#include <stdio.h>
#include "sbl_profile.h"
#include "sbl_soc_cfg.h"
#include "sbl_log.h"

struct sblTest_boardcfg_rm_resasg {
    struct tisci_boardcfg_substructure_header    subhdr;
    uint16_t                    resasg_entries_size;
    uint16_t                    reserved;
} __attribute__((__packed__));

struct sblTest_boardcfg_rm {
    struct tisci_boardcfg_abi_rev        rev;
    struct tisci_boardcfg_rm_host_cfg    host_cfg;
    struct sblTest_boardcfg_rm_resasg    resasg;
} __attribute__((__packed__));

struct sblTest_local_rm_boardcfg {
    struct sblTest_boardcfg_rm      rm_boardcfg;
    struct tisci_boardcfg_rm_resasg_entry resasg_entries[TISCI_BOARDCFG_RM_RESASG_ENTRIES_MAX];
};

void sbl_puts(char *str);

/* Interface with the SBL */
extern uint32_t syfw_image;
extern sblProfileInfo_t *sblProfileLogAddr;
extern uint32_t *sblProfileLogIndxAddr;
extern uint32_t *sblProfileLogOvrFlwAddr;


