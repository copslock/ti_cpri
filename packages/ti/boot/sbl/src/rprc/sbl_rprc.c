/**
 *  \file   sbl_rprc.c
 *
 *  \brief  This file contains functions to parse the multi-core
 *          image file & loads into CPU internal memory & external memory.
 *
 */

/*
 * Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com/
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

#include <string.h>

/* TI-RTOS Header files */
#include "sbl_rprc_parse.h"
#include "sbl_rprc.h"

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */
int32_t (*fp_readData)(void *dstAddr, void *srcAddr,
                        uint32_t length);

/* Function Pointer used while reading data from the storage. */
void  (*fp_seek)(void *srcAddr, uint32_t location);

/* Offset into app image that is being processed */
static uint32_t sblMemOffset = 0;

/* SBL scratch memory defined at compile time */
static uint8_t *sbl_scratch_mem = ((uint8_t *)(SBL_SCRATCH_MEM_START));
static uint32_t sbl_scratch_sz = SBL_SCRATCH_MEM_SIZE;
/******************************************************************************
 ***                     SBL Multicore RPRC parse functions                 ***
*******************************************************************************/
/* read of block of data from buffer */
int32_t SBL_ReadMem(void       *buff,
                    void       *srcAddr,
                    uint32_t   size)
{
    memcpy(buff, (void*)(uintptr_t)((uint32_t)(uintptr_t)srcAddr + sblMemOffset), size);
    sblMemOffset += size;

    return E_PASS;
}

/* move the buffer pointer */
void SBL_SeekMem(void *srcAddr, uint32_t location)
{
    sblMemOffset = location;
}

void SBL_SetRsvdMem(uint8_t *scratch_mem, uint32_t scratch_sz)
{
    sbl_scratch_mem = scratch_mem;
    sbl_scratch_sz = scratch_sz;
}

void SBL_SetMulticoreImageImgReadfxn(void *imge_read_fxn, void *image_seek_fxn)
{
    fp_readData = (int32_t (*)(void *, void *, uint32_t))imge_read_fxn;
    fp_seek = (void (*)(void *, uint32_t))image_seek_fxn;
}

__attribute__((weak)) int32_t SBL_VerifyMulticoreImage(void **img_handle,
                                                       uint32_t *ImageOffsetPtr,
                                                       uint8_t *scratch_mem,
                                                       uint32_t scratch_sz);
int32_t SBL_VerifyMulticoreImage(void **img_handle,
                                 uint32_t *ImageOffsetPtr,
                                 uint8_t *scratch_mem,
                                 uint32_t scratch_sz)
{
    int32_t retVal = E_FAIL;
    return retVal;
}

int32_t SBL_MulticoreImageParse(void *srcAddr,
                                uint32_t ImageOffset,
                                sblEntryPoint_t *pAppEntry)
{
    uint32_t            i;
    uint32_t            entryPoint = 0;
    meta_header_start_t mHdrStr;
    meta_header_core_t  mHdrCore[MAX_INPUT_FILES];
    meta_header_end_t   mHdrEnd;
    int32_t magic_str = META_HDR_MAGIC_STR;
    int32_t retVal    = E_PASS;

    if ((fp_readData == NULL) || (fp_seek == NULL))
    {
        retVal = E_FAIL;
    }
    else
    {
        /* Verify image if possible */
        SBL_VerifyMulticoreImage(&srcAddr, &ImageOffset, sbl_scratch_mem, sbl_scratch_sz);

        /* Read Meta Header Start and get the Number of Input RPRC Files */
        fp_readData(&mHdrStr, srcAddr, sizeof (meta_header_start_t));
        if (mHdrStr.magic_str != (uint32_t) magic_str)
        {
            UART_printf("Invalid magic number in Single image header\r\n");
            retVal = E_FAIL;
        }
        else
        {
            if (mHdrStr.dev_id != GetDeviceId())
            {
                UART_printf("\nWARNING: Device Id Doesnot match\r\n");
            }

            /* Read all the Core offset addresses */
            for (i = (0U); i < mHdrStr.num_files; i++)
            {
                fp_readData(&mHdrCore[i], srcAddr, sizeof (meta_header_core_t));
            }

            /* Add Base Offset address for All core Image start offset */
            for (i = (0U); i < mHdrStr.num_files; i++)
            {
                mHdrCore[i].image_offset += ImageOffset;
            }

            /* Read Meta Header End */
            fp_readData(&mHdrEnd, srcAddr, sizeof (meta_header_end_t));

            /* Now Parse Individual RPRC files */

            for (i = (0U); i < mHdrStr.num_files; i++)
            {
                if (mHdrCore[i].core_id != (0xFFFFFFFFU))
                {
                    fp_seek(srcAddr, mHdrCore[i].image_offset);
                    if (SBL_RprcImageParse(srcAddr, &entryPoint,
                                       (int32_t)(mHdrCore[i].core_id)) != E_PASS)
                    {
                        /* Error occurred parsing the RPRC file continue to
                         * parsing next
                         * image and skip booting the particular core */
                        UART_printf("RPRC parse error\n");
                        retVal = E_FAIL;
                    }
                    else
                    {
                        SBL_BootCore(entryPoint, mHdrCore[i].core_id, pAppEntry);
                    }
                }
            }
        }
    }
    return retVal;
}

void SBL_BootCore(uint32_t entry, uint32_t CoreID, sblEntryPoint_t *pAppEntry)
{
    switch (CoreID)
    {
#if defined(SOC_AM65XX) || defined (SOC_J721E)
        case ONLY_LOAD_ID:
            /* Only loading, ignore entry point*/
            SBL_log(SBL_LOG_MAX, "Only load (not execute) image @0x%x\n", entry);
            pAppEntry->CpuEntryPoint[CoreID] = SBL_INVALID_ENTRY_ADDR;
            break;
        case MPU1_CPU0_ID:
        case MPU1_CPU1_ID:
        case MPU2_CPU0_ID:
        case MPU2_CPU1_ID:
        case DSP1_C66X_ID:
        case DSP2_C66X_ID:
        case DSP1_C7X_ID:
        case DSP2_C7X_ID:
        case MCU1_CPU0_ID:
        case MCU1_CPU1_ID:
        case MCU2_CPU0_ID:
        case MCU2_CPU1_ID:
        case MCU3_CPU0_ID:
        case MCU3_CPU1_ID:
            /* All other cores*/
            SBL_log(SBL_LOG_MAX, "Setting entry point for core %d @0x%x\n", CoreID, entry);
            pAppEntry->CpuEntryPoint[CoreID] = entry;
            break;
        case MPU1_SMP_ID:
            /* Cluster 1 SMP*/
            SBL_log(SBL_LOG_MAX, "Setting SMP entry point for MPU1 @0x%x\n", entry);
            pAppEntry->CpuEntryPoint[MPU1_CPU0_ID] = entry;
            pAppEntry->CpuEntryPoint[MPU1_CPU1_ID] = entry;
            pAppEntry->CpuEntryPoint[CoreID] = SBL_INVALID_ENTRY_ADDR;
            break;
        case MPU2_SMP_ID:
            /* Cluster 2 SMP*/
            SBL_log(SBL_LOG_MAX, "Setting SMP entry point for MPU2 @0x%x\n", entry);
            pAppEntry->CpuEntryPoint[MPU2_CPU0_ID] = entry;
            pAppEntry->CpuEntryPoint[MPU2_CPU1_ID] = entry;
            pAppEntry->CpuEntryPoint[CoreID] = SBL_INVALID_ENTRY_ADDR;
            break;
        case MPU_SMP_ID:
            /* SMP on both clusters*/
            SBL_log(SBL_LOG_MAX, "Setting SMP entry point for all MPUs @0x%x\n", entry);
            pAppEntry->CpuEntryPoint[MPU1_CPU0_ID] = entry;
            pAppEntry->CpuEntryPoint[MPU1_CPU1_ID] = entry;
            pAppEntry->CpuEntryPoint[MPU2_CPU0_ID] = entry;
            pAppEntry->CpuEntryPoint[MPU2_CPU1_ID] = entry;
            pAppEntry->CpuEntryPoint[CoreID] = SBL_INVALID_ENTRY_ADDR;
            break;
        case MCU1_SMP_ID:
            /* Cluster 2 SMP*/
            SBL_log(SBL_LOG_MAX, "Setting Lockstep entry point for MCU1 @0x%x\n", entry);
            pAppEntry->CpuEntryPoint[MCU1_CPU0_ID] = entry;
            pAppEntry->CpuEntryPoint[MCU1_CPU1_ID] = SBL_MCU_LOCKSTEP_ADDR;
            pAppEntry->CpuEntryPoint[CoreID] = SBL_INVALID_ENTRY_ADDR;
            break;
        case MCU2_SMP_ID:
            /* Cluster 2 SMP*/
            SBL_log(SBL_LOG_MAX, "Setting Lockstep entry point for MCU2 @0x%x\n", entry);
            pAppEntry->CpuEntryPoint[MCU2_CPU0_ID] = entry;
            pAppEntry->CpuEntryPoint[MCU2_CPU1_ID] = SBL_MCU_LOCKSTEP_ADDR;
            pAppEntry->CpuEntryPoint[CoreID] = SBL_INVALID_ENTRY_ADDR;
            break;
        case MCU3_SMP_ID:
            /* Cluster 3 SMP*/
            SBL_log(SBL_LOG_MAX, "Setting Lockstep entry point for MCU3 @0x%x\n", entry);
            pAppEntry->CpuEntryPoint[MCU3_CPU0_ID] = entry;
            pAppEntry->CpuEntryPoint[MCU3_CPU1_ID] = SBL_MCU_LOCKSTEP_ADDR;
            pAppEntry->CpuEntryPoint[CoreID] = SBL_INVALID_ENTRY_ADDR;
            break;
#else
        case MPU_CPU0_ID:
            /* CSL_MPU CPU0*/
            pAppEntry->entryPoint_MPU_CPU0 = entry;
            break;
#endif
#if defined(OMAPL137_BUILD) || defined(C6748_BUILD)  || defined(OMAPL138_BUILD)
        case DSP0_ID:
            /* DSP0*/
            pAppEntry->entryPoint_DSP0 = entry;
            break;
#endif    
#if defined(AM572x_BUILD) || defined(AM574x_BUILD) || defined(K2H_BUILD) || defined(K2E_BUILD) || defined(K2L_BUILD) || defined(K2K_BUILD) || defined(K2G_BUILD)
        case MPU_CPU1_ID:
            /* CSL_MPU CPU1 */
            pAppEntry->entryPoint_MPU_CPU1 = entry;
            break;
#endif

#if defined(K2H_BUILD) || defined(K2E_BUILD) || defined(K2K_BUILD) || defined(K2G_BUILD)
        case MPU_CPU2_ID:
            pAppEntry->entryPoint_MPU_CPU2 = entry;
            break;

        case MPU_CPU3_ID:
            pAppEntry->entryPoint_MPU_CPU3 = entry;
            break;

        case MPU_SMP_ID:
            pAppEntry->entryPoint_MPU_CPU0 = entry;
            pAppEntry->entryPoint_MPU_CPU1 = entry;
            pAppEntry->entryPoint_MPU_CPU2 = entry;
            pAppEntry->entryPoint_MPU_CPU3 = entry;
            break;
#endif

#if defined(AM571x_BUILD) || defined(AM572x_BUILD) || defined(AM574x_BUILD)
        case IPU1_CPU0_ID:
            /*IPU1 CPU0*/
            pAppEntry->entryPoint_IPU1_CPU0 = entry;
            break;

        case IPU1_CPU1_ID:
            /*IPU1 CPU1*/
            pAppEntry->entryPoint_IPU1_CPU1 = entry;
            break;

        case IPU1_CPU_SMP_ID:
            /*IPU1 CPU0 & CPU1*/
            pAppEntry->entryPoint_IPU1_CPU0 = entry;
            pAppEntry->entryPoint_IPU1_CPU1 = entry;
            break;
#endif

#if defined (AM572x_BUILD) || defined(AM574x_BUILD)
        case IPU2_CPU0_ID:
            /*IPU2 CPU0*/
            pAppEntry->entryPoint_IPU2_CPU0 = entry;
            break;

        case IPU2_CPU1_ID:
            /*IPU2 CPU1*/
            pAppEntry->entryPoint_IPU2_CPU1 = entry;
            break;

        case IPU2_CPU_SMP_ID:
            /*IPU2 CPU0 & CPU1*/
            pAppEntry->entryPoint_IPU2_CPU0 = entry;
            pAppEntry->entryPoint_IPU2_CPU1 = entry;
            break;
#endif

#if defined(K2H_BUILD) || defined(K2E_BUILD) || defined(K2L_BUILD) || defined(K2K_BUILD) || defined(K2G_BUILD)
        case DSP0_ID:
            pAppEntry->entryPoint_DSP0 = entry;
            break;
#endif

#if defined(AM572x_BUILD) || defined(AM574x_BUILD) || defined(AM571x_BUILD) || defined(K2H_BUILD) || defined(K2L_BUILD) || defined(K2K_BUILD)
        case DSP1_ID:
            /* DSP1*/
            pAppEntry->entryPoint_DSP1 = entry;
            break;
#endif

#if defined (AM572x_BUILD) || defined(AM574x_BUILD) || defined(K2H_BUILD) || defined(K2L_BUILD) || defined(K2K_BUILD)
        case DSP2_ID:
            /* DSP2*/
            pAppEntry->entryPoint_DSP2 = entry;
            break;
#endif

#if defined(K2H_BUILD) || defined(K2L_BUILD) || defined(K2K_BUILD)
        case DSP3_ID:
            pAppEntry->entryPoint_DSP3 = entry;
            break;
#endif

#if defined(K2H_BUILD)
        case DSP4_ID:
            pAppEntry->entryPoint_DSP4 = entry;
            break;

        case DSP5_ID:
            pAppEntry->entryPoint_DSP5 = entry;
            break;

        case DSP6_ID:
            pAppEntry->entryPoint_DSP6 = entry;
            break;

        case DSP7_ID:
            pAppEntry->entryPoint_DSP7 = entry;
            break;
#endif

            default:
            /* Wrong CPU ID */
            break;
    }
}

#if defined(K2G_BUILD) || defined(K2E_BUILD) || defined(K2H_BUILD) || defined(K2K_BUILD) || defined(K2L_BUILD) || defined(AM571x_BUILD) || defined(AM572x_BUILD) || defined(AM574x_BUILD)
static int32_t SBL_RprcImageParse(void *srcAddr,
                                  uint32_t *entryPoint,
                                  int32_t CoreId)
{
    rprcFileHeader_t    header;
    rprcSectionHeader_t section;
    int32_t i;
    int32_t retVal = E_PASS;

    /*read application image header*/
    fp_readData(&header, srcAddr, sizeof (rprcFileHeader_t));

    /*check magic number*/
    if (header.magic != RPRC_MAGIC_NUMBER)
    {
        UART_printf("Invalid magic number in boot image. Expected: %x, received: %x\n", RPRC_MAGIC_NUMBER, header.magic);
        retVal = E_FAIL;
    }
    else
    {
        /* Set the Entry Point */
        *entryPoint = header.entry;

        /*read entrypoint and copy sections to memory*/
        for (i = 0; i < header.SectionCount; i++)
        {
            /*read new section header*/
            fp_readData(&section, srcAddr, sizeof (rprcSectionHeader_t));
#if defined(K2G_BUILD) || defined(K2E_BUILD) || defined(K2H_BUILD) || defined(K2K_BUILD) || defined(K2L_BUILD)
#if 0
            if ((section.addr >= CSL_MSMC_SRAM_REGS) &&
                (section.addr < CSL_MSMC_SRAM_REGS + CSL_MSMC_SRAM_REGS_SIZE))
            {

            }
            else if (section.addr >= CSL_DDR_0_DATA)
            {

            }
            else if ((section.addr >= CSL_C66X_COREPAC_LOCAL_L2_SRAM_REGS) &&
                (section.addr < CSL_C66X_COREPAC_LOCAL_L2_SRAM_REGS + CSL_C66X_COREPAC_LOCAL_L2_SRAM_REGS_SIZE))
            {
                section.addr |= 0x10000000;
            }
            else
            {
                
            }
#else
            if ((section.addr >= CSL_MSMC_SRAM_REGS) &&
                (section.addr < CSL_MSMC_SRAM_REGS + 0x01000000))
            {

            }
            else if (section.addr >= 0x80000000)
            {

            }
            else if ((section.addr >= CSL_C66X_COREPAC_LOCAL_L2_SRAM_REGS) &&
                (section.addr < CSL_C66X_COREPAC_LOCAL_L2_SRAM_REGS + 0x00100000))
            {
                switch (CoreId)
                {
                    case DSP0_ID:
                        section.addr |= 0x10000000; /* CorePac0 L2 SRAM */
                        break;
#if defined(K2H_BUILD) || defined(K2K_BUILD) || defined(K2L_BUILD)
                    case DSP1_ID:
                        section.addr |= 0x11000000; /* CorePac1 L2 SRAM */
                        break;

                    case DSP2_ID:
                        section.addr |= 0x12000000; /* CorePac2 L2 SRAM */
                        break;

                    case DSP3_ID:
                        section.addr |= 0x13000000; /* CorePac3 L2 SRAM */
                        break;
#endif
#if defined(K2H_BUILD) || defined(K2K_BUILD)
                    case DSP4_ID:
                        section.addr |= 0x14000000; /* CorePac4 L2 SRAM */
                        break;

                    case DSP5_ID:
                        section.addr |= 0x15000000; /* CorePac5 L2 SRAM */
                        break;

                    case DSP6_ID:
                        section.addr |= 0x16000000; /* CorePac6 L2 SRAM */
                        break;

                    case DSP7_ID:
                        section.addr |= 0x17000000; /* CorePac7 L2 SRAM */
                        break;
#endif
                    default:
                    /* Wrong CPU ID */
                    break;
                }
            }
            else
            {
                
            }
#endif
#else
#if defined (AM572x_BUILD) || defined(AM574x_BUILD)
            if (((section.addr >= CSL_MPU_OCMC_RAM1_REGS) &&
                 (section.addr < (CSL_MPU_OCMC_RAM1_REGS + SOC_OCMC_RAM1_SIZE))) ||
                ((section.addr >= CSL_MPU_OCMC_RAM2_REGS) &&
                 (section.addr < (CSL_MPU_OCMC_RAM2_REGS + SOC_OCMC_RAM2_SIZE))) ||
                ((section.addr >= CSL_MPU_OCMC_RAM3_REGS) &&
                 (section.addr < (CSL_MPU_OCMC_RAM3_REGS + SOC_OCMC_RAM3_SIZE))))
#elif defined (AM571x_BUILD)
            if ((section.addr >= CSL_MPU_OCMC_RAM1_REGS) &&
                 (section.addr < (CSL_MPU_OCMC_RAM1_REGS + SOC_OCMC_RAM1_SIZE)))
#endif
            {
                /* Internal OCMC RAM Space for all the cores */
            }
            /*copy section to memory*/
            /*check for section mapped into CPU internal memories*/
            else if (section.addr < 0x80000000U)
            {
                switch (CoreId)
                {
                    case MPU_CPU0_ID:
                        /*No action*/
                        break;
#if defined (AM572x_BUILD) || defined(AM574x_BUILD)
                    case MPU_CPU1_ID:
                        /*No action*/
                        break;
#endif
                    case IPU1_CPU0_ID:
                    case IPU1_CPU1_ID:
                    case IPU1_CPU_SMP_ID:

                        if (((section.addr >= CSL_IPU_IPU1_BOOT_SPACE_REGS) &&
                             (section.addr <
                              (CSL_IPU_IPU1_BOOT_SPACE_REGS + 0x10000U))) ||
                            ((section.addr >= CSL_IPU_IPU1_RAM_REGS) &&
                             (section.addr < (CSL_IPU_IPU1_RAM_REGS + 0x10000))))
                        {
                            section.addr = section.addr & 0x000FFFFFU;
                            section.addr = MPU_IPU1_RAM + section.addr;
                        }
                        else
                        {
                            UART_puts("IPU1 - Invalid Memory section", -1);
                        }

                        break;
#if defined (AM572x_BUILD) || defined(AM574x_BUILD)
                    case IPU2_CPU0_ID:
                    case IPU2_CPU1_ID:
                    case IPU2_CPU_SMP_ID:
                        if (((section.addr >= CSL_IPU_IPU1_BOOT_SPACE_REGS) &&
                             (section.addr <
                              (CSL_IPU_IPU1_BOOT_SPACE_REGS + 0x10000U)))
                            ||
                            ((section.addr >= CSL_IPU_IPU1_RAM_REGS) &&
                             (section.addr < (CSL_IPU_IPU1_RAM_REGS + 0x10000))))
                        {
                            section.addr = section.addr & 0x000FFFFFU;
                            section.addr = MPU_IPU2_RAM + section.addr;
                        }
                        else
                        {
                            UART_puts("IPU2  - Invalid Memory section", -1);
                        }

                        break;
#endif
                    case DSP1_ID:

                        if ((section.addr >= SOC_DSP_L2_BASE) &&
                            (section.addr < (SOC_DSP_L2_BASE + 0x48000)))
                        {
                            /* L2 RAM Read the offset */
                            section.addr = section.addr - SOC_DSP_L2_BASE;
                            section.addr = MPU_DSP1_L2_RAM + section.addr;
                        }
                        else if ((section.addr >= SOC_DSP_L1P_BASE) &&
                                 (section.addr < (SOC_DSP_L1P_BASE + 0x8000)))
                        {
                            /* CSL_MPU_DSP1_L1P_CACHE Read the offset */
                            section.addr = section.addr - SOC_DSP_L1P_BASE;
                            section.addr = MPU_DSP1_L1P_CACHE + section.addr;
                        }
                        else if ((section.addr >= SOC_DSP_L1D_BASE) &&
                                 (section.addr < (SOC_DSP_L1D_BASE + 0x8000)))
                        {
                            /* CSL_MPU_DSP1_L1D_CACHE */
                            section.addr = section.addr - SOC_DSP_L1D_BASE;
                            section.addr = MPU_DSP1_L1D_CACHE + section.addr;
                        }
                        else
                        {
                            UART_puts("DSP1 - Invalid Memory section", -1);
                        }

                        break;
#if defined (AM572x_BUILD) || defined(AM574x_BUILD)
                    case DSP2_ID:

                        if ((section.addr >= SOC_DSP_L2_BASE) &&
                            (section.addr < (SOC_DSP_L2_BASE + 0x48000)))
                        {
                            /* DSP2 L2 RAM Read the offset */
                            section.addr = section.addr - SOC_DSP_L2_BASE;
                            section.addr = MPU_DSP2_L2_RAM + section.addr;
                        }
                        else if ((section.addr >= SOC_DSP_L1P_BASE) &&
                                 (section.addr < (SOC_DSP_L1P_BASE + 0x8000)))
                        {
                            /* CSL_MPU_DSP2_L1P_CACHE Read the offset */
                            section.addr = section.addr - SOC_DSP_L1P_BASE;
                            section.addr = MPU_DSP2_L1P_CACHE + section.addr;
                        }
                        else if ((section.addr >= SOC_DSP_L1D_BASE) &&
                                 (section.addr < (SOC_DSP_L1D_BASE + 0x8000)))
                        {
                            /* CSL_MPU_DSP2_L1D_CACHE */
                            section.addr = section.addr - SOC_DSP_L1D_BASE;
                            section.addr = MPU_DSP2_L1D_CACHE + section.addr;
                        }
                        else
                        {
                            UART_puts("DSP2 - Invalid Memory section", -1);
                        }

                        break;
#endif
                    default:
                        break;
                }
            }
            else
            {
                /* To remove MISRA C error */
            }
#endif
            fp_readData((void *) section.addr, srcAddr, section.size);
        }
    }
    return retVal;
}
#endif

#if defined(OMAPL137_BUILD) || defined(OMAPL138_BUILD) || defined(C6748_BUILD)
/* */
static int32_t SBL_RprcImageParse(void *srcAddr,
                                  uint32_t *entryPoint,
                                  int32_t CoreId)
{
    rprcFileHeader_t    header;
    rprcSectionHeader_t section;
    uint32_t i;
    int32_t retVal = E_PASS;

    /*read application image header*/
    fp_readData(&header, srcAddr, sizeof (rprcFileHeader_t));

    /*check magic number*/
    if (header.magic != RPRC_MAGIC_NUMBER)
    {
        UART_printf("Invalid magic number in boot image. Expected: %x, received: %x\n", RPRC_MAGIC_NUMBER, header.magic);
        retVal = E_FAIL;
    }
    else
    {
        /* Set the Entry Point */
        *entryPoint = header.entry;

        /*read entrypoint and copy sections to memory*/
        for (i = 0; i < header.SectionCount; i++)
        {
            fp_readData(&section, srcAddr, sizeof (rprcSectionHeader_t));
            fp_readData((void *) section.addr, srcAddr, section.size);
        }
    }
    return retVal;
}
#endif

#if defined(SOC_AM65XX) || defined (SOC_J721E)
/* */

__attribute__((weak)) void SBL_SetupCoreMem(uint32_t CoreID);
void SBL_SetupCoreMem(uint32_t core_id)
{
    return;
}

static int32_t SBL_RprcImageParse(void *srcAddr,
                                  uint32_t *entryPoint,
                                  int32_t CoreId)
{
    rprcFileHeader_t    header;
    rprcSectionHeader_t section;
    uint32_t sbl_rsvd_mem_start = (uint32_t)(SBL_SCRATCH_MEM_START);
    uint32_t sbl_rsvd_mem_end = (uint32_t)(((uint32_t)(SBL_SCRATCH_MEM_START)) + ((uint32_t)(SBL_SCRATCH_MEM_SIZE)));
    uint32_t i;
    int32_t retVal = E_PASS;

    const uint32_t SocAtcmAddr[] =
    {
    SBL_MCU_ATCM_BASE,
    SBL_MCU1_CPU1_ATCM_BASE_ADDR_SOC,
    SBL_MCU2_CPU0_ATCM_BASE_ADDR_SOC,
    SBL_MCU2_CPU1_ATCM_BASE_ADDR_SOC,
    SBL_MCU3_CPU0_ATCM_BASE_ADDR_SOC,
    SBL_MCU3_CPU1_ATCM_BASE_ADDR_SOC
    };

    const uint32_t SocBtcmAddr[] =
    {
    SBL_MCU_BTCM_BASE,
    SBL_MCU1_CPU1_BTCM_BASE_ADDR_SOC,
    SBL_MCU2_CPU0_BTCM_BASE_ADDR_SOC,
    SBL_MCU2_CPU1_BTCM_BASE_ADDR_SOC,
    SBL_MCU3_CPU0_BTCM_BASE_ADDR_SOC,
    SBL_MCU3_CPU1_BTCM_BASE_ADDR_SOC
    };

    const uint32_t SocC66xL2SramAddr[] =
    {
    SBL_C66X1_L2SRAM_BASE_ADDR_SOC,
    SBL_C66X2_L2SRAM_BASE_ADDR_SOC
    };

    const uint32_t SocC66xL1DmemAddr[] =
    {
    SBL_C66X1_L1DMEM_BASE_ADDR_SOC,
    SBL_C66X2_L1DMEM_BASE_ADDR_SOC
    };

    const uint32_t SocC7xL2SramAddr[] =
    {
    SBL_C7X1_L2SRAM_BASE_ADDR_SOC,
    SBL_C7X2_L2SRAM_BASE_ADDR_SOC
    };

    const uint32_t SocC7xL1DmemAddr[] =
    {
    SBL_C7X1_L1DMEM_BASE_ADDR_SOC,
    SBL_C7X2_L1DMEM_BASE_ADDR_SOC
    };

    /*read application image header*/
    fp_readData(&header, srcAddr, sizeof (rprcFileHeader_t));

    /*check magic number*/
    if (header.magic != RPRC_MAGIC_NUMBER)
    {
        SBL_log(SBL_LOG_ERR, "Invalid magic number in boot image. Expected: %x, received: %x\n", RPRC_MAGIC_NUMBER, header.magic);
        retVal = E_FAIL;
    }
    else
    {
        /* Set the Entry Point */
        *entryPoint = header.entry;

        /* Setup CPUs internal memory before using it */
        SBL_SetupCoreMem(CoreId);

	/*read entrypoint and copy sections to memory*/
        for (i = (0U); i < header.SectionCount; i++)
        {
            fp_readData(&section, srcAddr, sizeof (rprcSectionHeader_t));

            switch (CoreId)
            {
                case MCU1_CPU1_ID:
                case MCU2_CPU0_ID:
                case MCU2_CPU1_ID:
                case MCU3_CPU0_ID:
                case MCU3_CPU1_ID:
                    /*Remap TCM address from R5 local to SoC memory map*/
                    if (section.addr < (SBL_MCU_ATCM_BASE + SBL_MCU_ATCM_SIZE))
                    {
                        /* Get offset into ATCM */
                        SBL_log(SBL_LOG_MAX, "Translating coreid %d local ATCM addr 0x%x to ", CoreId, section.addr);
                        section.addr = section.addr - SBL_MCU_ATCM_BASE;
                        section.addr = SocAtcmAddr[CoreId - MCU1_CPU0_ID] + section.addr;
                        SBL_log(SBL_LOG_MAX, "SoC MCU ATCM addr 0x%x\n", section.addr);
                    }
                    else if ((section.addr >= SBL_MCU_BTCM_BASE) &&
                             (section.addr < (SBL_MCU_BTCM_BASE + SBL_MCU_BTCM_SIZE)))
                    {
                        /* Get offset into BTCM */
                        SBL_log(SBL_LOG_MAX, "Translating coreid %d local BTCM addr 0x%x to ", CoreId, section.addr);
                        section.addr = section.addr - SBL_MCU_BTCM_BASE;
                        section.addr = SocBtcmAddr[CoreId - MCU1_CPU0_ID] + section.addr;
                        SBL_log(SBL_LOG_MAX, "SoC MCU BTCM addr 0x%x\n", section.addr);
                    }
                    else
                    {
                        /* To remove MISRA C error */
                    }
                    break;
                case DSP1_C66X_ID:
                case DSP2_C66X_ID:
                    /*Remap L1 & L2 address from C66x local to SoC memory map*/
                    if ((section.addr >= SBL_C66X_L2SRAM_BASE) &&
                        (section.addr < (SBL_C66X_L2SRAM_BASE + SBL_C66X_L2SRAM_SIZE)))
                    {
                        /* Get offset into L2 SRAM */
                        SBL_log(SBL_LOG_MAX, "Translating C66x local L2SRAM addr 0x%x to ", section.addr);
                        section.addr = section.addr - SBL_C66X_L2SRAM_BASE;
                        section.addr = SocC66xL2SramAddr[CoreId - DSP1_C66X_ID] + section.addr;
                        SBL_log(SBL_LOG_MAX, "SoC C66x L2SRAM addr 0x%x\n", section.addr);
                    }
                    else if ((section.addr >= SBL_C66X_L1DMEM_BASE) &&
                             (section.addr < (SBL_C66X_L1DMEM_BASE + SBL_C66X_L1DMEM_SIZE)))
                    {
                        /* Get offset into L1 DRAM */
                        SBL_log(SBL_LOG_MAX, "Translating C66x local L1DMEM addr 0x%x to ", section.addr);
                        section.addr = section.addr - SBL_C66X_L1DMEM_BASE;
                        section.addr = SocC66xL1DmemAddr[CoreId - DSP1_C66X_ID] + section.addr;
                        SBL_log(SBL_LOG_MAX, "SoC C66x L2DMEM addr 0x%x\n", section.addr);
                    }
                    else
                    {
                        /* To remove MISRA C error */
                    }
                    break;
                case DSP1_C7X_ID:
                case DSP2_C7X_ID:
                    /*Remap L1 & L2 address from C7x local to SoC memory map*/
                    if ((section.addr >= SBL_C7X_L2SRAM_BASE) &&
                        (section.addr < (SBL_C7X_L2SRAM_BASE + SBL_C7X_L2SRAM_SIZE)))
                    {
                        /* Get offset into L2 SRAM */
                        SBL_log(SBL_LOG_MAX, "Translating C7x local L2SRAM addr 0x%x to ", section.addr);
                        section.addr = section.addr - SBL_C7X_L2SRAM_BASE;
                        section.addr = SocC7xL2SramAddr[CoreId - DSP1_C7X_ID] + section.addr;
                        SBL_log(SBL_LOG_MAX, "SoC C7x L2SRAM addr 0x%x\n", section.addr);
                    }
                    else if ((section.addr >= SBL_C7X_L1DMEM_BASE) &&
                             (section.addr < (SBL_C7X_L1DMEM_BASE + SBL_C7X_L1DMEM_SIZE)))
                    {
                        /* Get offset into L1 DRAM */
                        SBL_log(SBL_LOG_MAX, "Translating C7x local L1DMEM addr 0x%x to ", section.addr);
                        section.addr = section.addr - SBL_C7X_L1DMEM_BASE;
                        section.addr = SocC7xL1DmemAddr[CoreId - DSP1_C7X_ID] + section.addr;
                        SBL_log(SBL_LOG_MAX, "SoC C7x L2DMEM addr 0x%x\n", section.addr);
                    }
                    else
                    {
                        /* To remove MISRA C error */
                    }
                    break;
                default:
                    break;
            }

            if ((fp_readData == SBL_ReadMem) &&
               (((section.addr > sbl_rsvd_mem_start) && (section.addr < sbl_rsvd_mem_end)) ||
                (((section.addr + section.size) > sbl_rsvd_mem_start) && ((section.addr + section.size) < sbl_rsvd_mem_end))))
            {
                SBL_log(SBL_LOG_ERR, "Warning!! Section overwrites SBL reserved memory\n");
                retVal = E_FAIL;
            }
            else
            {
                SBL_log(SBL_LOG_MAX, "Copying 0x%x bytes to 0x%x\n", section.size, section.addr);
                fp_readData((void *)(uintptr_t)(section.addr), srcAddr, section.size);
            }
        }
    }
    return retVal;
}
#endif

/**
 * \brief  Function to read the device ID
 *
 *
 * \param   None.
 *
 * \return   Return the device id
 *
 **/
uint32_t GetDeviceId(void)
{
    uint32_t deviceId = 55U;
    return (deviceId);
}
