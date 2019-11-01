/*
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include "string.h"
#include "stdio.h"
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_pscAux.h>
#include "sbl_slave_core_boot.h"
#include "sbl_sec.h"
#include "sbl_rprc_parse.h"     /* return code definition */

#include "signed_sec_bm.h"      /* Contains the signed and secure BM */
#include "bootMainGem.h"        /* contains the RAM secure kernel */
#include "secserver_self.h"	/* contains the dsp secure server */

/* temp buffer to store the incoming bootData for secure boot */
SBL_incomingBootData_S sblInBootData __attribute__ ((section (".sblbootbuff")));



/* return the entry point to the code if the code is authenticated 
 * null otherwise
*/
uint32_t SBL_authentication(void *addr)
{
    uint32_t (*fn_auth)(int, char * const []);
    char *argv1 = "0";		/* 0 for DSP auth, 2 for ARM auth */
    char argv2[32];
    char *argv[3] = {0, 0, 0};
    uint32_t argc;

    argv[1] = argv1;
    argv[2] = argv2;

    sprintf(argv2, "0x%08x", (unsigned int)addr);

    fn_auth = (uint32_t (*)(int, char * const []))((uint32_t)signed_sec_bm+8);

    argc = 3; /* 3 arguments */

    return fn_auth(argc, argv);
}


/* Boot Monitor installation wrapper */
uint32_t SBL_mon_init(uint32_t freq)
{
    uint8_t* secbm = 0;   /* this is the start of the secure BM */
  
    void (*mon_install)(uint32_t addr, uint32_t dpsc, 
                        uint32_t freq, uint32_t run_location);

    secbm = (uint8_t*)signed_sec_bm;   /* this is the start of the BM */
    secbm = secbm + 0x4000;            /* update this figure when using newer version of BM */

    /* install boot monitor - 
     * jump to the non-sec-bm with cpuId, dpsc, freq in R0, R1, R2
     * which is a call to non_sec/init.S _start 
     * dpsc is TETRIS_BASE */
    mon_install =  (void(*)(uint32_t, uint32_t, uint32_t, uint32_t))signed_sec_bm;
    mon_install(0, 0x01E80000, freq, (uint32_t)(secbm)); 

    /* NO return code  -  BOTH BM non-secure init and ROM's SL_run don't return error code */
    return 0;
}

/* read of block of data from buffer */
int32_t SBL_MemRead(void       *buff,
                    void       *srcAddr,
                    uint32_t   size)
{
    if (sblInBootData.sbl_boot_buff_idx == 0) 
    {
       sblInBootData.sbl_boot_buff_idx = (uint32_t)srcAddr;
    }

    memcpy(buff, (void*)sblInBootData.sbl_boot_buff_idx, size);

    sblInBootData.sbl_boot_buff_idx += size;

    return E_PASS;
}

/* move the buffer pointer */
void SBL_MemSeek(void *srcAddr, uint32_t location)
{
    sblInBootData.sbl_boot_buff_idx = (uint32_t)srcAddr + location;
}

/* start the RAM Secure Kernel that runs on DSP
 * This needs to be done before DSP can run DSP/BIOS application.
 * Then wait for the SK kernel installation finishes
 */
uint32_t SBL_startSK(void)
{
    uint32_t    i;
    uint32_t    *pgemMagic;
    uint32_t    wait_for_dsps;
    volatile    uint32_t* ipcgr = (uint32_t*)SYS_IPCGR;

    /* Reset the DSP, start clean */
    CSL_PSC_setModuleLocalReset(18, PSC_MDLRST_DEASSERTED);

    /* Copy the gem code from the load address to the run address */
    memcpy((void *)GEM_SK_START_ADDR, bootMain, sizeof(bootMain));

    /* Start the GEMs */
    for (i = 0; i < NGEMS; i++)
    {
        pgemMagic    = (uint32_t *)(GEM_MAGIC(i));
        *pgemMagic   = GEM_SK_START_ADDR;
        ipcgr[i]     = 0x11;  /* Write interrupt bit and SRCS0 */
    }

    /* Now, wait for all GEMs to finish secure kernel installation */
    wait_for_dsps = 1;
    while (wait_for_dsps != 0)
    {
      wait_for_dsps = 0;
      for (i = 0; i < NGEMS; i++)
      {
        /* DSPs will clear the SRCS0 bit when done */
        wait_for_dsps |= (ipcgr[i] & 0x10);  
      }
    }

    return 0;
}

/* Start the Secure Server that runs on DSP
 * This needs to be done before authetication is called.
 */
uint32_t SBL_startDspSecSrv(void)
{
    uint32_t    i;
    uint32_t    *pgemMagic;

    /* Copy the secure server code from the load address to the run address */
    memcpy((void *)GEM_SEC_SRV_START_ADDR, secserver_self, sizeof(secserver_self));

    /* Start the GEMs */
    for (i = 0; i < NGEMS; i++)
    {
        pgemMagic    = (uint32_t *)(GEM_MAGIC(i));
        *pgemMagic   = GEM_SEC_SRV_START_ADDR;
    }

    return 0;
}

