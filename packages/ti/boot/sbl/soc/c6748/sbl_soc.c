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

#include "sbl_soc.h"

#include <ti/board/src/lcdkOMAPL138/include/board_cfg.h>
#include <ti/drv/mmcsd/soc/MMCSD_soc.h>

#define MMCSD_INSTANCE_SDCARD  0

int32_t SBL_socInit(void)
{
    Board_initCfg boardCfg;

    boardCfg = BOARD_INIT_UNLOCK_MMR |
        BOARD_INIT_MODULE_CLOCK      |
        BOARD_INIT_PLL               |
        BOARD_INIT_DDR               |
        BOARD_INIT_PINMUX_CONFIG     |
        BOARD_INIT_UART_STDIO;

    /* Board Library Init. */
    if (Board_init(boardCfg))
    {
        return -1;
    }

    return 0;
}

#if defined (BOOT_MMCSD)
int32_t SBL_mmcsdInit(void *handle)
{
    MMCSD_Error         ret;
    MMCSD_v0_HwAttrs    hwAttrsConfig;

    MMCSD_socGetInitCfg(MMCSD_INSTANCE_SDCARD,&hwAttrsConfig);
    /* Disable the interrupts as poll mode is sufficient for image flashing */
    hwAttrsConfig.enableInterrupt = 0;
    MMCSD_socSetInitCfg(MMCSD_INSTANCE_SDCARD,&hwAttrsConfig);

    MMCSD_init();

    ret = MMCSD_open(MMCSD_INSTANCE_SDCARD, NULL, (MMCSD_Handle)handle);
    if(ret != MMCSD_OK)
    {
        return -1;
    }

    return 0;
}

int32_t SBL_mmcsdClose(void *handle)
{
    MMCSD_Error    ret;

    ret = MMCSD_close((MMCSD_Handle)handle);
    if(ret != MMCSD_OK)
    {
        return -1;
    }

    return 0;
}

int32_t SBL_mmcsdWrite(void *handle, uint8_t *buf, uint32_t blockNum, uint32_t blockCnt)
{
    MMCSD_Error    ret;

    ret = MMCSD_write(handle, buf, blockNum, blockCnt);
    if(ret != MMCSD_OK)
    {
        return -1;
    }

    return 0;
}

int32_t SBL_mmcsdRead(void *handle, uint8_t *buf, uint32_t blockNum, uint32_t blockCnt)
{
    MMCSD_Error    ret;

    ret = MMCSD_read(handle, buf, blockNum, blockCnt);
    if(ret != MMCSD_OK)
    {
        return -1;
    }

    return 0;
}
#endif
