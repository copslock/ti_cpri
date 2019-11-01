/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http:;www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
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
 *              file:    ioLink_TLC59281.c
 *
 *              brief:   PRU IO-Link master TLC59281 driver
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#include <ti/drv/spi/SPI.h>

#include <ti/drv/spi/soc/SPI_soc.h>
#include <ti/drv/spi/test/src/SPI_log.h>
//#include <ti/drv/spi/test/src/SPI_board.h>
#include <ti/board/board.h>
#include <ti/starterware/include/hw/am437x.h>
#include <ti/starterware/include/hw/hw_control_am43xx.h>

/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */

static SPI_Handle      hwHandle;  /* SPI handle */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void TLC59281_initLEDs(void){
    SPI_Params      spiParams;
    SPI_v1_HWAttrs  *hwAttrs;
    SPI_v1_ChnCfg   *chnCfg;

    SPI_Params_init(&spiParams);
    spiParams.frameFormat  = SPI_POL0_PHA0;
    spiParams.dataSize = 16;
    spiParams.bitRate = 1000000;
    spiParams.transferTimeout = 10;
    SPI_init();

    hwHandle = (SPI_Handle)SPI_open(0, &spiParams);
    hwAttrs = (SPI_v1_HWAttrs*)hwHandle->hwAttrs;
    chnCfg  = &(hwAttrs->chnCfg[0]);
    chnCfg->dataLineCommMode = MCSPI_DATA_LINE_COMM_MODE_1; /* configure SPI to use D1 to send, instead of D0 */

    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_SPI0_D1), 0x13070000); /* configure SPI0_D1 pin to SPI mode */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_SPI0_CS0), 0x13070000); /* configure SPI0_CS0 pin to SPI mode */
}

void TLC59281_deInitLEDs(void){
    SPI_close(hwHandle);
}

void TLC59281_updateLEDs(uint32_t leds){
    SPI_Transaction transaction;     /* SPI transaction structure */
    uint32_t        txBuf, rxBuf, terminateXfer = 1;

    txBuf = (leds&0xffff);

    transaction.txBuf = (uint8_t *) &txBuf;
    transaction.rxBuf = (uint8_t *) &rxBuf;
    transaction.count = 1;
    transaction.arg = (void *)&terminateXfer;

    SPI_transfer(hwHandle, &transaction);
}


