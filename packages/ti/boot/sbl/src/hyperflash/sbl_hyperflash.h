/*
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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

/**
 *  \file   sbl_hyperflash.h
 *
 *  \brief  sbl hyperflash header file.
 *
 */

#ifndef _HYPERBUS_TEST_H_
#define _HYPERBUS_TEST_H_

#include <ti/board/board.h>
#include "sbl_log.h"
#include "sbl_typecast.h"
#include "sbl_profile.h"

#include <ti/board/src/flash/include/board_flash.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HyperFlash Command Addresses */
#define HPF_CMD_ADDR1                   (0xAAAU)
#define HPF_CMD_ADDR2                   (0x554U)

#define HPF_RESET_CMD                   (0xF0U)

#define HPF_WRITE_UNLOCK_CMD1           (0xAAU)
#define HPF_WRITE_UNLOCK_CMD2           (0x55U)
#define HPF_ID_ENTRY			(0x90U)
#define HPF_CFI_ENTRY_CMD		(0x98U)
#define HPF_WRITE_BUFFER_CMD            (0x25U)
#define HPF_WRITE_COMMENCE_CMD          (0x29U)

#define HPF_READ_DEV_STS_CMD            (0x70U)
#define HPF_CLEAR_STS_REG_CMD           (0x71U)

#define HPF_CHIP_ERASE_CMD              (0x10U)
#define HPF_SECTOR_ERASE_CMD            (0x30U)

#define IS_HPF_READY                    (0x80U)

/* CFI QRY string  */
#define NUM_READ_FOR_MDLL_LOCK          (20U)
#define CFI_QUERY_STR_Q_OFFSET          (0x0020)
#define CFI_QUERY_STR_R_OFFSET          (0x0022)
#define CFI_QUERY_STR_Y_OFFSET          (0x0024)

typedef enum {
	HYPERFLASH_CS_INDEX = 0,
    HYPERRAM_CS_INDEX
}HyperBus_ChipSel;

/*
 *  \brief    SBL_HYPERFLASHBootImage function copies the application image from
 *            the hyperflash to the DDR memory and gives control to the processor
 *            core.
 *
 *  \param    pointer to the structure holding the entry pointers for different
 *            cores.
 *
 *  \return   error status. If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 *
 */
int32_t SBL_HYPERFLASHBootImage(sblEntryPoint_t *pEntry);

/*
 *  \brief    SBL_hyperflashInit function initialises the hyperflash, it does
 *            the pinmux configuration, the GPIO mux configuration for OSPI vs
 *            hyperflash and then enables the hyperbus.
 *
 *  \params   none
 *
 */
void SBL_hyperflashInit(void);

/*
 *  \brief    Reads the sysfw.bin from the hyperflash memory. Also does the
 *            SBL_hyperflashInit as a part of it.
 *
 *  \params   none
 *
 */
int32_t SBL_ReadSysfwImage(void **pBuffer, uint32_t num_bytes);

/*
 *  \brief    SBL_hyperflashRead reads from the hyperflash memory.
 *
 *  \param    offset            [IN]        Offset to read data from hyperflash
 *  \param    dataBuff          [OUT]       Buffer to store read data
 *  \param    rdCount           [IN]        Number of bytes to read
 *
 */
static int32_t SBL_hyperflashRead(const void *handle, uint32_t offset,
                                  void *dataBuff,
                                  uint32_t rdCount);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _HYPERBUS_TEST_H_ */
