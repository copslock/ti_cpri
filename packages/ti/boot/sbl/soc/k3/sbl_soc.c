/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
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

#include "sbl_soc.h"
#include "sbl_soc_cfg.h"
#include "sbl_log.h"
#include "sbl_profile.h"
#include "sbl_rprc_parse.h"
#include "sbl_sci_client.h"
#include "sbl_err_trap.h"
#include <strings.h>
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include <ti/board/board.h>
#include <ti/board/src/devices/board_devices.h>

sblProfileInfo_t sblProfileLog[MAX_PROFILE_LOG_ENTRIES];
uint32_t sblProfileLogIndx = 0, sblProfileLogOvrFlw = 0;

#if defined (SOC_J721E)
sblRatCfgInfo_t* sblRatCfgList = NULL;
#elif defined (SOC_AM65XX)
sblRatCfgInfo_t* sblRatCfgList = NULL;
#endif

void SBL_init_profile(void)
{
    CSL_armR5PmuEnableAllCntrs(1);    /* Set/clear PMCR E-bit */
    CSL_armR5PmuResetCntrs();         /* Set PMCR P-bit */
    CSL_armR5PmuResetCycleCnt();      /* Set PMCR C-bit */
    CSL_armR5PmuEnableCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM, 1);     /* Set PMCNTENSET for event */
    CSL_armR5PmuClearCntrOverflowStatus(0x80000007);
}

uint32_t SBL_readPmu(void)
{
    return (CSL_armR5PmuReadCntr(0x1F));
}

uint32_t SBL_LogCycleCount(uint8_t *fxn, uint32_t line)
{
    uint32_t indx = sblProfileLogIndx % MAX_PROFILE_LOG_ENTRIES;
    sblProfileInfo_t *sblProfileLogPtr = &(sblProfileLog[indx]);

    if (sblProfileLogIndx == MAX_PROFILE_LOG_ENTRIES)
    {
        sblProfileLogOvrFlw = 1;
    }

    sblProfileLogPtr->fxn = fxn;
    sblProfileLogPtr->line = line;
    sblProfileLogPtr->cycle_cnt = CSL_armR5PmuReadCntr(0x1F);

    sblProfileLogIndx++;

    return sblProfileLogPtr->cycle_cnt;
}

void SBL_printProfileLog(void)
{
#ifdef SBL_DISPLAY_PROFILE_INFO
    uint64_t mcu_clk_freq;
    uint32_t i = 0, prev_cycle_cnt = 0, cycles_per_usec;
    uint32_t lastlogIndx;

    SBL_ADD_PROFILE_POINT;

    Sciclient_pmGetModuleClkFreq(SBL_DEV_ID_MCU1_CPU0, SBL_CLK_ID_MCU1_CPU0, &mcu_clk_freq, SCICLIENT_SERVICE_WAIT_FOREVER);
    cycles_per_usec = (mcu_clk_freq / 1000000);

    SBL_log(SBL_LOG_ERR, "MCU @ %dHz.\n", uint64_to_int32(mcu_clk_freq));
    SBL_log(SBL_LOG_ERR, "cycles per usec  = %d\n", cycles_per_usec);

    lastlogIndx = sblProfileLogIndx;

    if (sblProfileLogOvrFlw)
    {
        i = sblProfileLogIndx;
        prev_cycle_cnt = sblProfileLog[i].cycle_cnt;
        lastlogIndx = MAX_PROFILE_LOG_ENTRIES;
        SBL_log(SBL_LOG_ERR, "Detected overflow, some profile entries might be lost.\n");
        SBL_log(SBL_LOG_ERR, "Rebuild with a larger vlaue of MAX_PROFILE_LOG_ENTRIES ??\n");
    }

    while((i % MAX_PROFILE_LOG_ENTRIES) < lastlogIndx)
    {
        uint32_t cycles_to_us;

        if (sblProfileLog[i].cycle_cnt < prev_cycle_cnt)
        {
            SBL_log(SBL_LOG_MAX,"**");
        }
        else
        {
            SBL_log(SBL_LOG_MAX,"  ");
        }
        cycles_to_us = sblProfileLog[i].cycle_cnt/cycles_per_usec;
        SBL_log(SBL_LOG_ERR,"fxn:%32s\t", sblProfileLog[i].fxn);
        SBL_log(SBL_LOG_ERR,"line:%4d\t", sblProfileLog[i].line);
        SBL_log(SBL_LOG_ERR,"cycle:%10d\t", sblProfileLog[i].cycle_cnt);
        SBL_log(SBL_LOG_ERR,"timestamp:%10dus\n", cycles_to_us);
        prev_cycle_cnt = sblProfileLog[i].cycle_cnt;
        i++;
    }
#else
    if (sblProfileLogOvrFlw)
    {
    }
#endif
}

void SBL_DCacheClean(void *addr, uint32_t size)
{
    uint32_t set = 0, way = 0;
    uint32_t cacheLineSize = CSL_armR5CacheGetDcacheLineSize();
    uintptr_t firstAddr = (uintptr_t ) addr & ~(cacheLineSize -1U);
    uintptr_t lastAddr = (uintptr_t) addr + size;
    uintptr_t num_iterations = (lastAddr - firstAddr) / cacheLineSize;

    if (num_iterations < 64 * 4)
    {
        /* Invalidate by MVA */
        CSL_armR5CacheWbInv((const void *)addr, uint32_to_int32(size));
    }
    else
    {
        /* Invalidating full cache by set and way is more efficient */
	for (set = 0; set < 64; set ++)
        {
            for (way = 0; way < 4; way++)
            {
                CSL_armR5CacheCleanInvalidateDcacheSetWay(set, way);
            }
        }
    }

}

uint32_t SBL_GetCertLen(uint8_t *x509_cert_ptr)
{
    uint32_t cert_len = 0;
    uint8_t *cert_len_ptr = (uint8_t *)&cert_len;

    SBL_ADD_PROFILE_POINT;

    SBL_log(SBL_LOG_MAX,"Searching for X509 certificate ...");
    if ( *x509_cert_ptr != 0x30)
    {
        SBL_log(SBL_LOG_MAX,"not found\r\n");
        SBL_ADD_PROFILE_POINT;
        return 0;
    }

    cert_len = *(x509_cert_ptr + 1);

    /* If you need more than 2 bytes to store the cert length  */
    /* it means that the cert length is greater than 64 Kbytes */
    /* and we do not support it                                */
    if ((cert_len > 0x80) &&
        (cert_len != 0x82))
    {
        SBL_log(SBL_LOG_MAX,"size invalid\r\n");
        SBL_ADD_PROFILE_POINT;
        return 0;
    }

    SBL_log(SBL_LOG_MAX,"found @0x%x, ", x509_cert_ptr);

    if ( cert_len == 0x82)
    {
        *cert_len_ptr = *(x509_cert_ptr + 3);
        *(cert_len_ptr + 1) = *(x509_cert_ptr + 2);

        /* add current offset from start of x509 cert */
        cert_len += 3;
    }
    else
    {
        /* add current offset from start of x509 cert  */
        /* if cert len was obtained from 2nd byte i.e. */
        /* cert size is 127 bytes or less              */
        cert_len += 1;
    }

    /* cert_len now contains the offset of the last byte */
    /* of the cert from the ccert_start. To get the size */
    /* of certificate, add 1                             */
    SBL_log(SBL_LOG_MAX,"size = %d bytes\r\n", cert_len + 1);

    SBL_ADD_PROFILE_POINT;
    return cert_len + 1;
}

uint8_t *SBL_FindSeq(uint8_t *x509_cert_ptr, uint32_t x509_cert_size, uint8_t *seq_oid, uint8_t seq_len)
{
    uint8_t *x509_cert_end = x509_cert_ptr + x509_cert_size - seq_len;

    SBL_ADD_PROFILE_POINT;

    /* searching for the following byte seq in the cert */
    /* seq_id(0x30) seq_len(< 0x80) 0x06 0x09 0x2B...   */
    while (x509_cert_ptr < x509_cert_end)
    {
        if ((*x509_cert_ptr == seq_oid[0]) &&
            (*(x509_cert_ptr + 2) == seq_oid[2]) &&
            (*(x509_cert_ptr - 2) == 0x30))
        {
            if ((bcmp((const void *)x509_cert_ptr, (const void *)seq_oid, seq_len)) == 0)
            {
                SBL_ADD_PROFILE_POINT;
                /* return start boot_seq */
                return (x509_cert_ptr - 2);
            }
        }
        x509_cert_ptr++;
    }

    SBL_ADD_PROFILE_POINT;

    return NULL;
}

uint32_t SBL_GetMsgLen(uint8_t *x509_cert_ptr, uint32_t x509_cert_size)
{
    uint8_t *boot_seq_ptr;
    uint32_t msg_len = 0, boot_seq_len;
    uint8_t *msg_len_ptr = (uint8_t *)&msg_len;
    /* oid encoding of boot_seq extension - 1.3.6.1.4.1.294.1.1 */
    uint8_t boot_seq_oid[] = {0x06, 0x09, 0x2B, 0x06, 0x01, 0x04, 0x01, 0x82, 0x26, 0x01, 0x01};

    SBL_ADD_PROFILE_POINT;

    boot_seq_ptr = SBL_FindSeq(x509_cert_ptr, x509_cert_size, boot_seq_oid, sizeof(boot_seq_oid));
    SBL_log(SBL_LOG_MAX,"Found seq @ 0x%x\r\n", boot_seq_ptr);

    /* length of seq is stored in the byte after the 0x30 seq_id */
    /* length of seq is stored as offset of the last byte of seq */
    /* from current offset. Jump to the end of the boot seq as   */
    /* the length of the message  is the last field of this seq  */
    boot_seq_len = *(++boot_seq_ptr);
    boot_seq_ptr = boot_seq_ptr + boot_seq_len;

    /* The last integer in this sequence is the msg length    */
    /* integers are tagged 0x20, so search backwards for 0x20 */
    /* The msg size can be encoded in 1, 2, 3 or 4 bytes      */
    /* 0x02 0x01 0x##                                         */
    /* 0x02 0x02 0x## 0x##                                    */
    /* 0x02 0x03 0x## 0x## 0x##                               */
    /* 0x02 0x04 0x## 0x## 0x## 0x##                          */
    if ( (*(boot_seq_ptr - 5) == 0x02) &&
         (*(boot_seq_ptr - 4) == 0x04) )
    {
        /* msg length encoded in 4 bytes */
        *msg_len_ptr = *boot_seq_ptr;
        *(msg_len_ptr + 1) = *(boot_seq_ptr - 1);
        *(msg_len_ptr + 2) = *(boot_seq_ptr - 2);
        *(msg_len_ptr + 3) = *(boot_seq_ptr - 3);
    }
    else if ( (*(boot_seq_ptr - 4) == 0x02) &&
         (*(boot_seq_ptr - 3) == 0x03) )
    {
        /* msg length encoded in 3 bytes */
        *msg_len_ptr = *boot_seq_ptr;
        *(msg_len_ptr + 1) = *(boot_seq_ptr - 1);
        *(msg_len_ptr + 2) = *(boot_seq_ptr - 2);
    }
    else if ( (*(boot_seq_ptr - 3) == 0x02) &&
         (*(boot_seq_ptr - 2) == 0x02) )
    {
        /* msg length encoded in 2 bytes */
        *msg_len_ptr = *boot_seq_ptr;
        *(msg_len_ptr + 1) = *(boot_seq_ptr - 1);
    }
    else if ( (*(boot_seq_ptr - 3) == 0x02) &&
         (*(boot_seq_ptr - 1) == 0x01) )
    {
        /* msg length encoded in 1 byte */
        *msg_len_ptr = *boot_seq_ptr;
    }

    SBL_log(SBL_LOG_MAX,"image length = %d bytes\r\n", msg_len);

    SBL_ADD_PROFILE_POINT;

    return msg_len;
}

/* Keep all boot media read offsets and read sizes */
/* word aligned throughout the function in case    */
/* boot media cannot handle byte accesses          */
int32_t SBL_VerifyMulticoreImage(void **img_handle,
                                 uint32_t *ImageOffsetPtr,
                                 uint8_t *scratch_mem,
                                 uint32_t scratch_sz)
{
    uint8_t *scratch_mem_ptr;
    uint32_t cert_len = 0, img_len, cert_load_addr;
    int32_t retVal = E_FAIL, auth_retval = E_FAIL;
    uint32_t SBL_GetCertLen(uint8_t *x509_cert_ptr);
    uint32_t SBL_GetMsgLen(uint8_t *x509_cert_ptr, uint32_t x509_cert_size);

    SBL_ADD_PROFILE_POINT;

    if ((fp_readData != NULL) && (fp_seek != NULL))
    {
        uint8_t x509Header[4];
        /* Read first 4 bytes of image to */
        /* determine if it is a x509 img  */
        fp_readData(x509Header, *img_handle, 4);
        fp_seek(*img_handle, *ImageOffsetPtr);
        cert_len = SBL_GetCertLen(x509Header);
    }

    /* Check if cert size is within valid range */
    if ((cert_len > 0x100) &&
        (cert_len < 0x800) &&
        (scratch_mem !=NULL) &&
        (scratch_sz != 0))
    {
        uint8_t unaligned_bytes = cert_len % 4;
        uint8_t pad_align = 4 - unaligned_bytes;

        SBL_log(SBL_LOG_MAX,"SBL reserved memory Found: Start = @ 0x%x, Size = 0x%x\r\n", scratch_mem, scratch_sz);
        /* adjust cert load addr so that the */
        /* msg is always word aligned        */
        scratch_mem_ptr = scratch_mem + pad_align;
        SBL_log(SBL_LOG_MAX,"Copying %d bytes from app to 0x%x\r\n", cert_len + pad_align, scratch_mem_ptr);
        fp_readData(scratch_mem_ptr, *img_handle, cert_len + pad_align);
        fp_seek(*img_handle, *ImageOffsetPtr);
        cert_load_addr = (uint32_t)scratch_mem_ptr;

        img_len = SBL_GetMsgLen(scratch_mem_ptr, cert_len);
        if ((scratch_mem_ptr + cert_len + img_len)  <
            (scratch_mem + scratch_sz))
        {
            struct tisci_msg_proc_auth_boot_req authReq;

            fp_seek(*img_handle, (*ImageOffsetPtr + cert_len - unaligned_bytes));
            scratch_mem_ptr += (cert_len - unaligned_bytes);

            img_len += unaligned_bytes;
            unaligned_bytes = img_len % 4;
            pad_align = 4 - unaligned_bytes;
            img_len += pad_align;

            SBL_log(SBL_LOG_MAX,"Copying %d bytes from offset 0x%x to 0x%x...", img_len, *ImageOffsetPtr + cert_len - unaligned_bytes, scratch_mem_ptr);
            fp_readData(scratch_mem_ptr, *img_handle, img_len);
            fp_seek(*img_handle, *ImageOffsetPtr);

            /* Request DMSC to authenticate the image */
            authReq.certificate_address_hi = 0;
            authReq.certificate_address_lo = cert_load_addr;
            SBL_log(SBL_LOG_MAX,"Cert @ 0x%x ...", authReq.certificate_address_lo);

            SBL_ADD_PROFILE_POINT;

            retVal = auth_retval = Sciclient_procBootAuthAndStart(&authReq, SCICLIENT_SERVICE_WAIT_FOREVER);
            if (retVal == CSL_PASS)
            {
                SBL_log(SBL_LOG_MAX,"Verify Passed...");
            }
            else
            {
                if (SBL_IsSysfwEnc(NULL) == SBL_SYSFW_CLEAR_TEXT)
                {
                    SBL_log(SBL_LOG_MAX,"Ignored on GP...");
                }
                else
                {
                    SBL_log(SBL_LOG_ERR,"Verify Failed!!\n\r");
                    SBL_log(SBL_LOG_ERR,"Boot Halted!!\n\r");
                    SblErrLoop(__FILE__, __LINE__);
                }
            }

            SBL_ADD_PROFILE_POINT;

            /* Image is loaded. RPRC parsing no longer */
            /* neeeds to access the boot media. Update */
            /* caller with image load address          */
            SBL_SetMulticoreImageImgReadfxn((void *)SBL_ReadMem, (void *)SBL_SeekMem);
            *img_handle = (void *)(cert_load_addr+cert_len);
            *ImageOffsetPtr = 0;
            SBL_log(SBL_LOG_MAX,"done\r\n", img_len, scratch_mem_ptr);
            retVal = E_PASS;
        }
    }

    if ((auth_retval != CSL_PASS) &&
        (SBL_IsSysfwEnc(NULL) != SBL_SYSFW_CLEAR_TEXT))
    {
        SBL_log(SBL_LOG_ERR,"Is app signed correctly??\n\r");
        SBL_log(SBL_LOG_ERR,"App verification fails!! Boot Halted!!\n\r");
        SblErrLoop(__FILE__, __LINE__);
    }
    SBL_ADD_PROFILE_POINT;

    return retVal;
}

void SBL_RAT_Config(sblRatCfgInfo_t* remap_list)
{
    if (remap_list)
    {
    }
    else
    {
        return;
    }
}

void SBL_ConfigureCommonRails(sblCfgPmic_t *pmicVoltCfg, uint8_t powerResource)
{
    uint32_t pmicVoltCfgSz = SBL_MAX_VTM_VDS;
    uint16_t commonRailMaxVoltage = 0;
    sblCfgPmic_t *commonRailPmicCfgPtr = NULL;
    sblCfgPmic_t commonRailPmicCfg;

    /* A ganged rail is a single source that is used  */
    /* to power multiple voltage domains that can run */
    /* at the same voltage, even though each domain   */
    /* is capable of ACS class0                       */
    while (pmicVoltCfgSz--)
    {
        if(pmicVoltCfg->powerResource == powerResource)
        {
            if(commonRailPmicCfgPtr == NULL)
            {
                /* Save the fist instance of a ganged rail */
                commonRailPmicCfgPtr = pmicVoltCfg;
                commonRailPmicCfg = *commonRailPmicCfgPtr;
            }

            if(pmicVoltCfg->millivolts > commonRailMaxVoltage)
            {
                /* Ganged rails must be set to the max value */
                commonRailMaxVoltage = pmicVoltCfg->millivolts;
            }

            /* Zero out any ganged rails here    */
            /* the first ganged rail is stored   */
            /* in commonRailPmicCfg and will     */
            /* be used setup the correct voltage */
            /* for all the ganged rails          */
            pmicVoltCfg->millivolts = 0;
            pmicVoltCfg->slaveAddr = 0;
            pmicVoltCfg->powerResource = 0;

        }
        pmicVoltCfg++;
    }

    commonRailPmicCfg.millivolts = commonRailMaxVoltage;
    *commonRailPmicCfgPtr = commonRailPmicCfg;
    SBL_log(SBL_LOG_MAX, "Common rail: Slave:0x%x, Res:0x%x @ %dmV\r\n", commonRailPmicCfg.slaveAddr, commonRailPmicCfg.powerResource, commonRailPmicCfg.millivolts);
}

void SBL_SetupPmicCfg(sblCfgPmic_t *pmicVoltCfg, uint32_t opp)
{
    uint32_t vd, vtmDevInfo, vtmOppVid;
    uint32_t vtmOppVidMask = (CSL_VTM_CFG1_OPPVID_OPP_LOW_DFLT_MASK << opp);
    sblCfgPmic_t *pmicCfg;

    SBL_ADD_PROFILE_POINT;

    for (vd = 0; vd < SBL_MAX_VTM_VDS; vd++)
    {
	    vtmDevInfo = CSL_REG32_RD(SBL_VTM_CFG_BASE + CSL_VTM_CFG1_VD_DEVINFO(vd));
	    vtmOppVid = CSL_REG32_RD(SBL_VTM_CFG_BASE + CSL_VTM_CFG1_VD_OPPVID(vd));
	    pmicCfg = pmicVoltCfg + vd;
	    if (vtmDevInfo & CSL_VTM_CFG1_DEVINFO_AVS0_SUP_MASK)
	    {
            vtmOppVid = (vtmOppVid & vtmOppVidMask) >> opp;
            pmicCfg->millivolts = sblMapOtpVidToMilliVolts[vtmOppVid];
            SBL_log(SBL_LOG_MAX, "Efuse xlated: VD %d to %d mV (OppVid: 0x%x, Slave:0x%x, Res:0x%x)\r\n", vd, pmicCfg->millivolts, vtmOppVid, pmicCfg->slaveAddr, pmicCfg->powerResource);
        }
    }

    SBL_ADD_PROFILE_POINT;
}

/**********************************************************************
 ******************* SoC Specific Initilization ***********************
 **********************************************************************/
/* Workarounds for AM65xx SoC */
#if defined(SOC_AM65XX)
#include <ti/board/src/evmKeystone3/include/board_internal.h>
static sblCfgPmic_t pmicAvsVoltCfg[] =
{
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {BOARD_I2C_TPS62363_PMIC_ADDR, BOARD_TPS62363_SET3_VOUT, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};

void SBL_SocEarlyInit(void)
{
}

static void AM65XX_ConfigVtm(void)
{
       volatile uint32_t *vtm_tempsens0_ctrl_base = (uint32_t *)0x42050080;
       volatile uint32_t *vtm_tempsens1_ctrl_base = (uint32_t *)0x42050090;
       volatile uint32_t *vtm_tempsens2_ctrl_base = (uint32_t *)0x420500A0;

       *vtm_tempsens0_ctrl_base = 0xE0;
       *vtm_tempsens1_ctrl_base = 0xE0;
       *vtm_tempsens2_ctrl_base = 0xE0;
}

void AM65x_SetPmicVoltages(void *handle, sblCfgPmic_t *pmicVoltCfg)
{
    int32_t retStatus = 0;
    uint32_t pmicVoltCfgSz = SBL_MAX_VTM_VDS;

    SBL_ConfigureCommonRails(pmicVoltCfg, BOARD_TPS62363_SET3_VOUT);
    while (pmicVoltCfgSz--)
    {
        if(pmicVoltCfg->millivolts)
        {
            SBL_ADD_PROFILE_POINT;
            retStatus = Board_tps62363SetVoltage(handle, pmicVoltCfg->slaveAddr, pmicVoltCfg->powerResource, pmicVoltCfg->millivolts);
            if(retStatus != BOARD_SOK)
            {
                SBL_ADD_PROFILE_POINT;
                SBL_log(SBL_LOG_ERR,"Failed to set voltage to %d mV for Slave:0x%x, Res:0x%x\r\n", pmicVoltCfg->millivolts, pmicVoltCfg->slaveAddr, pmicVoltCfg->powerResource);
            }
            else
            {
                SBL_ADD_PROFILE_POINT;
                SBL_log(SBL_LOG_MAX,"Successfully set voltage to %d mV for Slave:0x%x, Res:0x%x\r\n", pmicVoltCfg->millivolts, pmicVoltCfg->slaveAddr, pmicVoltCfg->powerResource)
            }
        }
        pmicVoltCfg++;
    }
}

static void AM65x_SetupPmicAvs(uint32_t opp)
{
    I2C_Handle handle = NULL;

    SBL_log(SBL_LOG_MAX,"Setting up AVS ...\r\n");

    SBL_ADD_PROFILE_POINT;

    handle = Board_getI2CHandle(BOARD_SOC_DOMAIN_WKUP, BOARD_I2C_PMIC_INSTANCE);
    if(handle == NULL)
    {
        SBL_log(SBL_LOG_MAX,"I2C Open Failed\n\r");
        SblErrLoop(__FILE__, __LINE__);
    }

    SBL_SetupPmicCfg(pmicAvsVoltCfg, opp);

    AM65x_SetPmicVoltages(handle, pmicAvsVoltCfg);

    SBL_ADD_PROFILE_POINT;
}

void SBL_SocLateInit(void)
{
    SBL_ADD_PROFILE_POINT;

    AM65XX_ConfigVtm();

    SBL_ADD_PROFILE_POINT;

    AM65x_SetupPmicAvs(SBL_OPP_NOM);

    SBL_ADD_PROFILE_POINT;
}
#endif

/* Workarounds for J721E SoC */
#if defined(SOC_J721E)
#include <ti/board/src/j721e_evm/include/board_internal.h>

static sblCfgPmic_t pmicAvsVoltCfg[] =
{
    {0, 0, 0},
    {0, 0, 0},
    {BOARD_I2C_LEO_PMIC_A_ADDR, BOARD_TPS65941_BUCK1_RESOURCE, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};

void J721E_SetLeoPmicVoltages(void *handle, sblCfgPmic_t *pmicVoltCfg)
{
    int32_t retStatus = 0;
    uint32_t pmicVoltCfgSz = SBL_MAX_VTM_VDS;

    while (pmicVoltCfgSz--)
    {
        if(pmicVoltCfg->millivolts)
        {
            SBL_ADD_PROFILE_POINT;
            retStatus = Board_tps65941SetVoltage(handle, pmicVoltCfg->slaveAddr, pmicVoltCfg->powerResource, pmicVoltCfg->millivolts);
            if(retStatus != BOARD_SOK)
            {
                SBL_ADD_PROFILE_POINT;
                SBL_log(SBL_LOG_MAX,"Failed to set voltage to %d mV for Slave:0x%x, Res:0x%x\r\n", pmicVoltCfg->millivolts, pmicVoltCfg->slaveAddr, pmicVoltCfg->powerResource);
            }
            else
            {
                SBL_ADD_PROFILE_POINT;
                SBL_log(SBL_LOG_MAX,"Successfully set voltage to %d mV for Slave:0x%x, Res:0x%x\r\n", pmicVoltCfg->millivolts, pmicVoltCfg->slaveAddr, pmicVoltCfg->powerResource)
            }
        }
        pmicVoltCfg++;
    }
}

static void J721E_SetupLeoPmicAvs(uint32_t opp)
{
    I2C_Handle handle = NULL;

    SBL_ADD_PROFILE_POINT;

    handle = Board_getI2CHandle(BOARD_SOC_DOMAIN_WKUP, BOARD_I2C_PMIC_INSTANCE);
    if(handle == NULL)
    {
        SBL_log(SBL_LOG_MAX,"I2C Open Failed\n\r");
        SblErrLoop(__FILE__, __LINE__);
    }
    SBL_SetupPmicCfg(pmicAvsVoltCfg, opp);
    J721E_SetLeoPmicVoltages(handle, pmicAvsVoltCfg);

    SBL_ADD_PROFILE_POINT;
}

void SBL_SocLateInit(void)
{
    SBL_ADD_PROFILE_POINT;

	J721E_SetupLeoPmicAvs(SBL_OPP_NOM);

    SBL_ADD_PROFILE_POINT;
}

static void J721E_UART_InitPwrClk(void)
{
    HW_WR_REG32(SBL_UART_PLL_BASE + SBL_UART_PLL_KICK0_OFFSET, SBL_UART_PLL_KICK0_UNLOCK_VAL);
    HW_WR_REG32(SBL_UART_PLL_BASE + SBL_UART_PLL_KICK1_OFFSET, SBL_UART_PLL_KICK1_UNLOCK_VAL);

    HW_WR_REG32(SBL_UART_PLL_BASE + SBL_UART_PLL_DIV_OFFSET, SBL_UART_PLL_DIV_VAL);

    HW_WR_REG32(SBL_UART_PLL_BASE + SBL_UART_PLL_KICK0_OFFSET, SBL_UART_PLL_KICK_LOCK_VAL);
    HW_WR_REG32(SBL_UART_PLL_BASE + SBL_UART_PLL_KICK1_OFFSET, SBL_UART_PLL_KICK_LOCK_VAL);
}

static void J721E_SetupLvCmosDriveStrength(void)
{
    volatile uint32_t *reg1 = (uint32_t *)0x43005008;
    volatile uint32_t *reg2 = (uint32_t *)0x4300500C;
    volatile uint32_t *lvcmos_drv_h_base = (uint32_t *)0x430060c0;
    volatile uint32_t *lvcmos_drv_v_base = (uint32_t *)0x430060d0;
    volatile uint32_t i;

    *reg1 = 0x68EF3490;
    *reg2 = 0xD172BC5A;
    /* Workaround for fixing lvcmos drive strength */
    for (i = 0; i < 4; i++)
    {
       *(lvcmos_drv_h_base + i) = 0xD;
       *(lvcmos_drv_v_base + i ) = 0xD;
    }
}

void SBL_SocEarlyInit(void)
{
    J721E_SetupLvCmosDriveStrength();
    J721E_UART_InitPwrClk();
}

#endif
