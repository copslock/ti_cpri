/********************************************************************
 * Copyright (C) 2013 Texas Instruments Incorporated.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include <ti/drv/dfe/dfe_drv.h>
#include <ti/drv/dfe/dfe_osal.h>
#include <ti/drv/dfe/dfe_internal.h>
#include <ti/csl/cslr_dfe_autocp.h>
#include <ti/csl/cslr_dfe_cdfr.h>

/**
 * @defgroup DFE_LLD_MISC_FUNCTION MISC
 * @ingroup DFE_LLD_FUNCTION
 */

/**
 * @brief Disable All TestBus
 * @ingroup DFE_LLD_MISC_FUNCTION
 *
 * DFE has many test probe points scattered over all sub-modules. CB can be used to capture a train of IQ bus signals at the probe. 
 * All test probes "AND together" shares single CB interface. So software should enable no more than one probe at any time. 
 * The API clears all test probes which was programmed before.
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_disableAllTestbus
(
    DFE_Handle hDfe
)
{
	DfeFl_Status status;
	uint32_t data = 0;
	
    VALID_DFE_HANDLE(hDfe);

    // disable BB testbus
    {
    	DfeFl_BbCapBuffConfig *pCfg = (DfeFl_BbCapBuffConfig *) &data;

    	pCfg->testCbCtrl = DFE_FL_BB_TEST_CB_CTRL_DISABLE;
    	pCfg->testCbAxc = 0;
    
    	CSL_HW_CTRL( dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_CFG_CAPBUFF, pCfg) );
    }
    // disable DDUC testbus
    {
        uint32_t data = 0;
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[0], DFE_FL_DDUC_CMD_CFG_TESTBUS_MUX, &data) );
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[1], DFE_FL_DDUC_CMD_CFG_TESTBUS_MUX, &data) );
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[2], DFE_FL_DDUC_CMD_CFG_TESTBUS_MUX, &data) );
        CSL_HW_CTRL( dfeFl_DducHwControl(hDfe->hDfeDduc[3], DFE_FL_DDUC_CMD_CFG_TESTBUS_MUX, &data) );
    }
    // disable AUTOCP testbus
    {
        CSL_FINS(hDfe->hDfeAutocp[0]->regs->autocp_autocp_1, DFE_AUTOCP_AUTOCP_AUTOCP_1_REG_TEST_BUS_SEL, 0);
    }
    // disable CDFR testbus
    {
        CSL_FINS(hDfe->hDfeCdfr[0]->regs->cdfr_cdfr_1, DFE_CDFR_CDFR_CDFR_1_REG_TB_SEL, 0);
    }
    // SUMMER doesn't have a testbus
    
    // disable CFR testbus
    {
        CSL_FINS(hDfe->hDfeCfr[0]->regs->cfr_test_bus_mux, DFE_CFR_CFR_TEST_BUS_MUX_REG_CFR_TEST_BUS_MUX_SC0, 0);
    }    
    // disable DPD testbus
    {
        uint32_t data = 0;
        if (hDfe->dpdIsDisabled == 0)
        {
            CSL_HW_CTRL( dfeFl_DpdHwControl(hDfe->hDfeDpd[0], DFE_FL_DPD_CMD_TEST_BUS_CTRL, &data) );
        }
    }
    // disable TX testbus
    {
        DfeFl_TxTestBusSel tb_sel;
        
        tb_sel.txPath = DFE_FL_TXA;
        tb_sel.sel = 0;
        CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_TESTBUS_SEL, &tb_sel) );
        
        tb_sel.txPath = DFE_FL_TXB;
        tb_sel.sel = 0;
        CSL_HW_CTRL( dfeFl_TxHwControl(hDfe->hDfeTx[0], DFE_FL_TX_CMD_SET_TESTBUS_SEL, &tb_sel) );        
    }    
    // disable JESDTX testbus
    {
        DfeFl_JesdTxTestBusSel tp = DFE_FL_JESDTX_TESTBUS_SEL_DISABLE;
        
        CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_TEST_BUS, &tp) );
    }
    // disable JESDRX testbus
    {
        DfeFl_JesdRxTestBusSel tp = DFE_FL_JESDRX_TESTBUS_SEL_DISABLE;
    
    	CSL_HW_CTRL( dfeFl_JesdHwControl(hDfe->hDfeJesd[0], DFE_FL_JESDRX_CMD_SET_TESTBUS_SEL, &tp) );
    }    
    // disable RX testbus
    {
        DfeFl_RxTestCtrl RxTestCtrl;
        RxTestCtrl = DFE_FL_RX_TOP_TEST_CTRL_RXN;
        CSL_HW_CTRL( dfeFl_RxHwControl(hDfe->hDfeRx[0], DFE_FL_RX_CMD_SET_TOP_TEST_CTRL, &RxTestCtrl) );
    }        
    // disable FB testbus
    {
        CSL_FINS(hDfe->hDfeFb[0]->regs->testbus, DFE_FB_TESTBUS_REG_TOP_TEST_CTRL, 0);
    }
    
    return DFE_ERR_NONE;
}

/**
 * @brief Program DFE GPIO PinMux
 * @ingroup DFE_LLD_MISC_FUNCTION
 *
 * Select pin function by programming DFE GPIO pinmux. There're total 18 GPIOs, every pin can be configured to be different function.
 *
 *  | Pin Function	    | Value	| Input/Output | Description                              |
 *  | ----------------- | -----	| ------------ | ---------------------------------------- |
 *  | NOTHING	        | 0	    | Input	       | Input only, not configured to a function
 *  | GPIO_SYNC_IN0	1	|       | Input	       | General sync input 0                     |
 *  | GPIO_SYNC_IN1	2	|       | Input	       | General sync input 1                     |
 *  | JESD_SYNC_IN0	3	|       | Input	       | JESD sync inputs for interfacing to data converters which do not support LVDS syncs |
 *  | JESD_SYNC_IN1	4	|       | Input	       |                                          | 
 *  | GPIO_SYNC_OUT0	| 8	    | Output	   | General sync output 0                    |
 *  | GPIO_SYNC_OUT1	| 9	    | Output	   | General sync output 0                    |
 *  | JESD_SYNC_OUT0	| 10    | Output	   | JESD sync outputs for interfacing to data converters which do not support LVDS syncs |
 *  | JESD_SYNC_OUT1	| 11	| Output	   |                                          |
 *  | FB_MUX_CNTL0	    | 12	| Output	   | feedback mux control for Marconi 0       |
 *  | FB_MUX_CNTL1	    | 13	| Output	   |                                          |
 *  | FB_MUX_CNTL2	    | 14	| Output	   |                                          |
 *  | FB_MUX_CNTL3	    | 15	| Output	   | feedback mux control for Marconi 1       |
 *  | FB_MUX_CNTL4	    | 16	| Output	   |                                          |
 *  | FB_MUX_CNTL5	    | 17	| Output	   |                                          |
 *  | DVGA_CNTL0	    | 18	| Output	   | 8 bits DVGA controls                     |
 *  | DVGA_CNTL1	    | 19	| Output	   |                                          |
 *  | DVGA_CNTL2	    | 20	| Output	   |                                          |
 *  | DVGA_CNTL3	    | 21	| Output	   |                                          |
 *  | DVGA_CNTL4	    | 22	| Output	   |                                          |
 *  | DVGA_CNTL5	    | 23	| Output	   |                                          |
 *  | DVGA_CNTL6	    | 24	| Output	   |                                          |
 *  | DVGA_CNTL7	    | 25	| Output	   |                                          |
 *  | SYSREF_REQUEST	| 26	| Output	   | JESD SYSREF request output               |
 *  | MPU_DRIVE	        | 27	| Output	   | Driven by MPU register write             |
 *
 * Note:  ALL pins (including those set to "nothing") also function as mpu gpio read. If the pin is an output pin, the mpu gpio read will return whatever is being output.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param pinId	[in] GPIO pin ID
 *  @param muxSel	[in] select function of the pin
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_progGpioPinMux
(
    DFE_Handle hDfe,
    DfeFl_MiscGpioPin pinId,
    DfeFl_MiscGpioMux muxSel
)
{
    DfeFl_Status status;
    DfeFl_MiscGpioPinMuxConfig dfePinMuxCfg;
    
    VALID_DFE_HANDLE(hDfe);

    dfePinMuxCfg.pin = pinId;
    dfePinMuxCfg.mux = muxSel;

    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_SET_GPIO_PIN_MUX, &dfePinMuxCfg) );
    
    return DFE_ERR_NONE;
}

/**
 * @brief Set DFE GPIO Sync Out Source
 * @ingroup DFE_LLD_MISC_FUNCTION
 *
 * Select sync source for GPIO_SYNC_OUT pin.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param syncoutId	[in] sync out pin, 0/1
 *  @param ssel	[in] sync select
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_setGpioSyncOutSource
(
    DFE_Handle hDfe,
    uint32_t syncoutId,
    DfeFl_MiscSyncGenSig ssel
)
{
    DfeFl_Status status;
	DfeFl_MiscGpioSyncOutSselConfig gpioSyncOutSsel;
    
    VALID_DFE_HANDLE(hDfe);
    
	gpioSyncOutSsel.syncout = syncoutId;
	gpioSyncOutSsel.ssel    = ssel;
	CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_SET_GPIO_SYNC_OUT_SSEL, &gpioSyncOutSsel) );
    
    return DFE_ERR_NONE;
}

/**
 * @brief Set DFE GPIO Bank Output
 * @ingroup DFE_LLD_MISC_FUNCTION
 *
 * Write MPU GPIO Drive register to drive pin output. For pins configured as MPU_DRIVE,
 * DFE drives those pins per bankOutput value. For other pins, it has no effect.
 * 
 * There is a total of 18 GPIOs, one-to-one mapped to the 18 LSBs of bankOutput.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param bankOutput	[in] pin output bitmap value, bit0 driving pin0, bit1 driving pin1, …, bit17 driving pin17.
 *    - -	0 = drive low
 *    - -	1 = drive high
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_setGpioBankOutput
(
    DFE_Handle hDfe,
    uint32_t bankOutput
)
{
    DfeFl_Status status;

    VALID_DFE_HANDLE(hDfe);
 
    CSL_HW_CTRL( dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_SET_GPIO_BANK, &bankOutput) );

    return DFE_ERR_NONE;
}

/**
 * @brief Get DFE GPIO Bank Input
 * @ingroup DFE_LLD_MISC_FUNCTION
 *
 * Read current input status of GPIO pins.
 * There're total 18 GPIOs, one-to-one mapped to 18 LSBs of *bankInput.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param bankInput	[out] pin input bitmap value, bit0 is for pin0, bit1 is for pin1, …, bit17 is for pin17.
 *    - -	0 = input low
 *    - -	1 = input high
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_HW_QUERY, if CSL GetHwStatus() failed
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_getGpioBankInput
(
    DFE_Handle hDfe,
    uint32_t *bankInput
)
{
    DfeFl_Status status;

    VALID_DFE_HANDLE(hDfe);
    if(bankInput == NULL)
    {
        Dfe_osalLog("bankInput pointer NULL !");
        return DFE_ERR_INVALID_PARAMS;        
    }
        
    CSL_HW_QUERY( dfeFl_MiscGetHwStatus(hDfe->hDfeMisc[0], DFE_FL_MISC_QUERY_GET_GPIO_BANK, bankInput) );
    
    return DFE_ERR_NONE;    
}

/**
 * @brief Open Generic IO DMA
 * @ingroup DFE_LLD_MISC_FUNCTION
 *
 * Open and allocate resources for generic IO DMA, which can be used to read/write DFE
 * registers and memories.
 * 
 * Generic IO DMA is using CPP/DMA embedded address mode. The DMA descriptor is embedded
 * at the beginning of the packet.
 *
 * Generic IO DMA doesn't support linked transfers. Each DMA packet can transfer data size
 * from one 32-bits word to 16K 32-bits words. 
 *
 * Upon the API returns DFE_ERR_NONE, generic IO DMA has already been enabled and ready
 * for use. Data available on iqnChnlDl starts CPP/DMA.
 *
 * When generic IO DMA already opened, call this API again will get error #DFE_ERR_ALREADY_OPENED.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param cppDmaId	[in] CPP/DMA channel Id
 *    - 0 ~ 31, open with specified channel
 *    - DFE_FL_CPP_OPEN_ANY, open with any available channel
 *  @param iqnChnlDl	[in] IQN2 CTL Egress channel number, 0 ~ 15
 *  @param iqnChnlUl	[in] IQN2 CTL Ingress channel number, 0 ~ 15
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_AVAILABLE, if CPP/DMA channel not available
 *  - #DFE_ERR_ALREADY_OPENED, if generic IO DMA already opened
 *  - #DFE_ERR_HW_CTRL, if CSL HwControl() failed
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_openGenericDma
(
    DFE_Handle hDfe,
    uint32_t cppDmaId,
    uint32_t iqnChnlDl,
    uint32_t iqnChnlUl
)
{
	DfeFl_Status status;
	DfeFl_CppDmaHandle hDma;
    
	VALID_DFE_HANDLE(hDfe);
	if(iqnChnlDl > 15 || iqnChnlUl > 15)
    {
        Dfe_osalLog("iqnChnlDl/Ul out of range");
        return DFE_ERR_INVALID_PARAMS;        
    }
	
	// check if already opened
	if(hDfe->hDmaGeneric != NULL)
    {
        Dfe_osalLog("CPP/DMA for generic IO already opened");
        return DFE_ERR_ALREADY_OPENED;        
    }
	
    // open CPP/DMA for UL, EMBED mode
    hDma = dfeFl_CppDmaOpen(
        hDfe->hDfeMisc[0],
        cppDmaId, // dmaId
        DFE_FL_CPP_DMA_MODE_EMBED, // mode
        DFE_FL_CPP_OPEN_NONE, // trig
        &hDfe->cppResMgr, // resMgr
        &status);
	if(status != DFE_FL_SOK)
    {
        Dfe_osalLog("dfeFl_CppDmaOpen() failed, CSL error %d", status);
        return DFE_ERR_CPP_DMA_NOT_AVAILABLE;
    }
    
    CSL_HW_CTRL( dfeFl_CppDmaArm(hDma, iqnChnlDl, DFE_FL_CPP_DMA_SSEL_DL_CTL_DATA_AVAIL(iqnChnlDl)) );
    
    // save generic DMA info to context
    hDfe->hDmaGeneric = hDma;
    hDfe->genericDmaIqnChnlDl = iqnChnlDl;
    hDfe->genericDmaIqnChnlUl = iqnChnlUl;
    
    return DFE_ERR_NONE;
}

/**
 * @brief Close Generic IO DMA
 * @ingroup DFE_LLD_MISC_FUNCTION
 *
 * Close generic IO DMA and free resources allocated by Dfe_openGenericDma(). 
 *
 *  @param hDfe	[in] DFE device handle
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_VALID, if CPP/DMA handle not valid
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *  - Dfe_openGenericDma() has called OK.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_closeGenericDma
(
    DFE_Handle hDfe
)
{
    VALID_DFE_HANDLE(hDfe);
    
	// check if valid CPP/DMA opened
	if(hDfe->hDmaGeneric == NULL)
    {
        Dfe_osalLog("CPP/DMA handle (generic) not valid");
        return DFE_ERR_CPP_DMA_NOT_VALID;        
    }

    dfeFl_CppDmaClose(hDfe->hDmaGeneric);
    
    // clean context
    hDfe->hDmaGeneric = NULL;
    hDfe->genericDmaIqnChnlDl = DFE_FL_CPP_OPEN_NONE;
    hDfe->genericDmaIqnChnlUl = DFE_FL_CPP_OPEN_NONE;
    
    return DFE_ERR_NONE;
}

/**
 * @brief Prepare Generic DMA Embedded Header
 * @ingroup DFE_LLD_MISC_FUNCTION
 *
 * The API helps building CPP/DMA embedded header that will be sent together with data payload block. The size of the header is fixed 16 bytes, then followed data payload block.
 *
 * The API supports three read/write modes:
 *  -	DFE_GENERIC_DMA_RW_MODE_WRITE_SINGLE_WORD, write single 32-bits word to DFE. The sizeOrData has the sending value that will be built into the embedded header. There's no other payload to be sent.
 *  -	DFE_GENERIC_DMA_RW_MODE_WRITE_MULTI_WORDS, write multiple 32-bits words to DFE. The sizeOrData has the number of words of data payload, which is immediately following the header. If number of the payload words not multiples of 4, zero shall be padded to fill the remaining words in the final line.
 *  -	DFE_GENERIC_DMA_RW_MODE_READ, read from DFE. The sizeOrData has the number of words of read data, which will be received from IQN2 CTL ingress channel, iqnChnlUl, specified when Dfe_openGenericDma().
 *
 * For writing, offsetAddrInDref is the destination 26-bit address within DFE scope; for reading, it is the source 26-bit address within DFE scope.
 *
 *  @param hDfe	[in] DFE device handle
 *  @param header	[out] pointer to packet header
 *  @param rwMode	[in] generic IO read/write mode
 *  @param offsetAddrInDref	[in] 26-bit address within DFE scope.
 *  @param sizeOrData	[in] size or data. For single word writing, it is the data value; for other modes, it is number of payload words in 32-bits.
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_CPP_DMA_NOT_VALID, if CPP/DMA handle not valid
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *  - DFE PLL and PSCs shall be already up running.
 *  - DFE has loaded target config and completed initialize sequence.
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_prepareGenericDmaHeader
(
    DFE_Handle hDfe,
    uint32_t *header,
    DFE_GenericDmaReadWriteMode rwMode,
    uint32_t offsetAddrInDref,
    uint32_t sizeOrData
)
{
    DfeFl_CppEmbedHeader *embedHdr;
    DfeFl_CppDescripConfig *descripCfg;
        
    VALID_DFE_HANDLE(hDfe);

    if(header == NULL)
    {
        Dfe_osalLog("header pointer is NULL");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    embedHdr = (DfeFl_CppEmbedHeader *)header;
    
    descripCfg = &embedHdr->descripCfg;
    descripCfg->mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE(offsetAddrInDref - (uint32_t)(hDfe->objDfe.regs));
    descripCfg->ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
    descripCfg->mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
    descripCfg->pktSize = DFE_FL_CPP_DMA_PKT_SIZE_64K;
    descripCfg->midImm = 0;
    descripCfg->linkNext = 0; // not used in embedded mode
    // first two words of data
    embedHdr->data0 = 0; 
    embedHdr->data1 = 0; 
    switch(rwMode)
    {
    case DFE_GENERIC_DMA_RW_MODE_WRITE_SINGLE_WORD:
        descripCfg->chanNum = hDfe->genericDmaIqnChnlDl;
        descripCfg->rw = DFE_FL_CPP_DMA_DL;
        descripCfg->midImm = 1;
        embedHdr->data1 = sizeOrData;
        descripCfg->numBytes = 0;
        break;
    case DFE_GENERIC_DMA_RW_MODE_WRITE_MULTI_WORDS:
        descripCfg->chanNum = hDfe->genericDmaIqnChnlDl;
        descripCfg->rw = DFE_FL_CPP_DMA_DL;
        descripCfg->numBytes = sizeOrData;
        break;
    case DFE_GENERIC_DMA_RW_MODE_READ:
        descripCfg->chanNum = hDfe->genericDmaIqnChnlUl;
        descripCfg->rw = DFE_FL_CPP_DMA_UL;
        descripCfg->numBytes = sizeOrData;
        break;
    default:
        Dfe_osalLog("unsupported read/write mode for generic DMA!");
        return DFE_ERR_INVALID_PARAMS;
    }
    
    return DFE_ERR_NONE;
}

