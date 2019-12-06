/**
 *  \file   sbl_rprc.h
 *
 *  \brief  This file contains function prototypes of RPRC image parse functions.
 *
 */

/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef SBL_RPRC_H_
#define SBL_RPRC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/drv/uart/UART_stdio.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/arch/csl_arch.h>
#include <soc/sbl_soc.h>
#include <stdio.h>

#if defined(SOC_AM65XX) || defined (SOC_J721E)
#include "sbl_log.h"
#include "sbl_soc_cfg.h"
#include <ti/board/board.h>
#include <ti/drv/pm/include/pm_types.h>
#include <ti/drv/pm/include/pmlib_clkrate.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* Magic numbers for gforge and sourceforge */
#define MAGIC_NUM_GF          (0xA1ACED00)
#define MAGIC_NUM_SF          (0x55424CBB)

/* Magic number and tokens for RPRC format */
#define RPRC_MAGIC_NUMBER   0x43525052
#define RPRC_RESOURCE       0
#define RPRC_BOOTADDR       5

#define MAX_INPUT_FILES 10
#define META_HDR_MAGIC_STR 0x5254534D /* MSTR in ascii */
#define META_HDR_MAGIC_END 0x444E454D /* MEND in ascii */

#if !defined(OMAPL137_BUILD)
#define SOC_OCMC_RAM1_SIZE          ((uint32_t) 0x80000)     /*OCMC1 512KB*/
#define SOC_OCMC_RAM2_SIZE          ((uint32_t) 0x100000)    /*OCMC2 1MB   */
#define SOC_OCMC_RAM3_SIZE          ((uint32_t) 0x100000)    /*OCMC3  1MB   */

#define MPU_IPU1_ROM                    (CSL_IPU_IPU1_TARGET_REGS)

#define MPU_IPU1_RAM                    (CSL_IPU_IPU1_TARGET_REGS + \
                                            (uint32_t) 0x20000)

#define MPU_IPU2_ROM                    (CSL_IPU_IPU1_ROM_REGS)

#define MPU_IPU2_RAM                    (CSL_IPU_IPU1_ROM_REGS + \
                                            (uint32_t) 0x20000)

#define MPU_DSP1_L2_RAM                 (0x40800000)
#define MPU_DSP1_L1P_CACHE              (0x40E00000)
#define MPU_DSP1_L1D_CACHE              (0x40F00000)
#define MPU_DSP2_L2_RAM                 (0x41000000)
#define MPU_DSP2_L1P_CACHE              (0x41600000)
#define MPU_DSP2_L1D_CACHE              (0x41700000)

#if !defined(SOC_AM574x) && !defined (SOC_J721E)
#define SOC_DSP_L2_BASE                 (0x800000)
#define SOC_DSP_L1P_BASE                (0xe00000)
#define SOC_DSP_L1D_BASE                (0xf00000)
#endif
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct rprcFileHeader {
    uint32_t magic;
    uint32_t entry;
    uint32_t rsvd_addr;
    uint32_t SectionCount;
    uint32_t version;
} rprcFileHeader_t;

typedef struct rprcSectionHeader {
    uint32_t addr;
    uint32_t rsvd_addr;
    uint32_t size;
    uint32_t rsvdCrc;
    uint32_t rsvd;
} rprcSectionHeader_t;

typedef struct meta_header_start
{
    uint32_t magic_str;
    uint32_t num_files;
    uint32_t dev_id;
    uint32_t rsvd;
}meta_header_start_t;

typedef struct meta_header_core
{
    uint32_t core_id;
    uint32_t image_offset;
}meta_header_core_t;

typedef struct meta_header_end
{
    uint32_t rsvd;
    uint32_t magic_string_end;
}meta_header_end_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief           SBL_RprcImageParse function parse the RPRC executable image.
 *                  Copies individual section into destination location
 *
 * \param[in]    srcAddr - Pointer RPRC image
 * \param[out] entryPoint - CPU entry point address
 * \param[in]    CoreId - CPU ID to identify the CPU core
 *
 *
 * \return uint32_t: Status (success or failure)
 */
static int32_t SBL_RprcImageParse(void *srcAddr, uint32_t *entryPoint,
                              int32_t CoreId);

/**
 * \brief        SBL_BootCore function stores the CPU entry location into
 *               global pointer.
 *
 * \param[in]    entry - CPU Entry location
 * \param[in]    entryPoint - CPU ID
 *
 * \return   none
 */
void SBL_BootCore(uint32_t entry, uint32_t CoreID, sblEntryPoint_t *pAppEntry);

uint32_t GetDeviceId(void);

#ifdef __cplusplus
}
#endif

#endif /*SBL_RPRC_PARSE_H_*/

