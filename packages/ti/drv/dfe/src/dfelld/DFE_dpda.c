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

#include <math.h>
#include <signal.h>

/**
 * @defgroup DFE_LLD_DPDA_FUNCTION DPDA
 * @ingroup DFE_LLD_FUNCTION
 */
 
/**
 * @brief Start DPDA
 * @ingroup DFE_LLD_DPDA_FUNCTION
 *
 * This function starts DPDA.
 * The API
 * -   clears interrupt flag in the command register
 * -   clears idle status, read status and processed status
 * -   writes the given start address to the command register
 * -   sets the interrupt flag in the command register
 * -   polls the idle status until the DPDA returns to the idle state
 *
 *
 *  @param hDfe [in] DFE device handle
 *  @param startAddress [in] Specifies start address in the instruction RAM, 0 ~ 4095
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_startDpda(
        DFE_Handle hDfe,
        uint16_t startAddress
        )
{
    DfeFl_DpdaHandle    hDpda;
    DfeFl_Status        status;
    uint32_t            dpdaGetHwStatus_arg;
    uint32_t            dpdaHwControl_arg;

    VALID_DFE_HANDLE(hDfe);

    if (startAddress >= 4096) {
        Dfe_osalLog("Start address out of range!");
        return DFE_ERR_INVALID_PARAMS;
    }

    hDpda = hDfe->hDfeDpda[0];

    if (hDfe->dpdaIsDisabled == 0)
    {
        // clear interrupt
        dpdaHwControl_arg = 0;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_SET_NEW_INT_DPD, (void *)&dpdaHwControl_arg) );

        // clear idle status
        dpdaHwControl_arg = 1; // not used
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_CLR_IDLE_INTR_STATUS, (void *)&dpdaHwControl_arg) );

        // clear read status
        dpdaHwControl_arg = 1; // not used
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_CLR_INT_READ_COMPLETE_INTR_STATUS, (void *)&dpdaHwControl_arg) );

        // clear processed status
        dpdaHwControl_arg = 1; // not used
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_CLR_INT_PROCESSED_INTR_STATUS, (void *)&dpdaHwControl_arg) );

        // grant DSP control to interrupt port
        dpdaHwControl_arg = 1;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_SET_DSP_INTR, (void *)&dpdaHwControl_arg) );

        // grant DSP control to antenna mask port
        dpdaHwControl_arg = 1;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_SET_DSP_ANT_EN, (void *)&dpdaHwControl_arg) );

        // write param1
        dpdaHwControl_arg = 1;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_SET_PARAM1_DPD, (void *)&dpdaHwControl_arg) );

        // write param2
        dpdaHwControl_arg = 1;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_SET_PARAM2_DPD, (void *)&dpdaHwControl_arg) );

        // write interrupt address
        dpdaHwControl_arg = startAddress;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_SET_INTR_ADDRESS_DPD, (void *)&dpdaHwControl_arg) );

        // for selected antenna write antenna mask
        dpdaHwControl_arg = 0xF;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_SET_ANT_ENABLED_DPD, (void *)&dpdaHwControl_arg) );

        // set interrupt
        dpdaHwControl_arg = 1;
        CSL_HW_CTRL( dfeFl_DpdaHwControl(hDpda, DFE_FL_DPDA_CMD_SET_NEW_INT_DPD, (void *)&dpdaHwControl_arg) );

        // wait for completion
        dpdaGetHwStatus_arg = 0;
        while (dpdaGetHwStatus_arg != 1)
        {
            CSL_HW_CTRL( dfeFl_DpdaGetHwStatus(hDpda, DFE_FL_DPDA_QUERY_GET_IDLE_INTR_STATUS, &dpdaGetHwStatus_arg) );
        }
    }

    return DFE_ERR_NONE;
} // End of Dfe_startDpda()

/**
 * @brief Loads the DPDA image
 * @ingroup DFE_LLD_DPDA_FUNCTION
 *
 * Load image to DPDA instruction RAM and lookup tables.  Reset and initialize DPDA.
 * The API
 * -   disables the arbiter
 * -   resets DPDA
 * -   forces DPDA into the IDLE state
 * -   clears interrupt mask and status registers
 * -   clears command, test and debug registers
 * -   clears the scalar and IG register files
 * -   clears the stack
 * -   loads the given image
 * -   releases DPDA from reset
 *
 *
 *  @param hDfe [in] DFE device handle
 *  @param igId [in] Identifies IG register, 0 ~ 63
 *  @param igPtr [out] Points to value
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_loadDpda(
        DFE_Handle hDfe,
        uint32_t imageSize,
        DFE_RegPair *imagePtr
        )
{
    DfeFl_MiscHandle    hMisc;
    DfeFl_DpdaHandle    hDpda;
    uint32_t            *regs, regId;
    uint32_t            memMpuAccess;
    int32_t                i;

    VALID_DFE_HANDLE(hDfe);

    if (hDfe->dpdaIsDisabled == 0)
    {
        hMisc = hDfe->hDfeMisc[0];
        // Save memory access rights and grant access to whole DFE
        memMpuAccess = CSL_FEXT(hMisc->regs->cfg2, DFE_MISC_CFG2_REG_MEM_MPU_ACCESS);
        hMisc->regs->cfg2 = CSL_FMK(DFE_MISC_CFG2_REG_MEM_MPU_ACCESS, 0xFFFF);
        // Disable arbiter
        hMisc->regs->arbiter_cfg_arb1 = CSL_FMK(DFE_MISC_ARBITER_CFG_ARB1_REG_ARBITER_CFG, 0);

        hDpda = hDfe->hDfeDpda[0];
        // Init DPDA
        hDpda->regs->inits = CSL_FMK(DFE_DPDA_INITS_REG_INITS_SSEL, DFE_FL_SYNC_GEN_SIG_ALWAYS) \
                           | CSL_FMK(DFE_DPDA_INITS_REG_INIT_CLK_GATE, 1) \
                           | CSL_FMK(DFE_DPDA_INITS_REG_INIT_STATE, 1) \
                           | CSL_FMK(DFE_DPDA_INITS_REG_CLEAR_DATA, 1);
        // Force DPDA to idle
        hDpda->regs->main_control = CSL_FMK(DFE_DPDA_MAIN_CONTROL_REG_CG_DSP_IDLE, 1);
        hDpda->regs->main_control = CSL_FMK(DFE_DPDA_MAIN_CONTROL_REG_CG_DSP_IDLE, 0);
        // Clear DPDA interrupt mask, status and set registers
        hDpda->regs->mask = 0;
        hDpda->regs->status = 0;
        hDpda->regs->force = 0;
        // Clear command registers
        hDpda->regs->interrupt_params = 0;
        hDpda->regs->interrupt_main_and_req = 0;
        // Clear test and debug registers
        hDpda->regs->testbus_control = 0;
        hDpda->regs->debug_breakpoint = 0;
        hDpda->regs->debug_sets = 0;
        // Clear scalar registers
        for (i=0; i<64; i++) {
            hDpda->regs->dpda_scalar[i].ie_register = 0x40000000;
            hDpda->regs->dpda_scalar[i].q_register = 0;
        }
        // Clear IG registers
        for (i=0; i<55; i++) {
            hDpda->regs->dpda_ig_regfile[i] = 0;
        }
        hDpda->regs->dpda_ig_regfile_preg_radd = 0;
        hDpda->regs->dpda_ig_regfile_preg_wadd = 0;
        hDpda->regs->rsvd4[0] = 0;
        hDpda->regs->dpda_ig_regfile_dsp_status1 = 0;
        // Clear LUT scratch RAM
        for (i=0; i<256; i++) {
            hDpda->regs->dpda_into_dpd4_ram0[i] = 0;
            hDpda->regs->dpda_into_dpd4_ram1[i] = 0;
            hDpda->regs->dpda_into_dpd4_ram2[i] = 0;
        }
        // Clear stack
        for (i=0; i<64; i++) {
            hDpda->regs->dpda_stack[i] = 0;
        }

        // Load image
        regs = (uint32_t *)hDpda->regs;
        for (i=0; i<imageSize; i++) {
            regId = (imagePtr[i].addr - DFE_FL_DPDA_0_OFFSET)>>2;
            *(regs + regId) = imagePtr[i].data;
        }

        // Release DPDA from reset
        for (i=0; i<16; i++) {
            hDpda->regs->inits = CSL_FMK(DFE_DPDA_INITS_REG_INITS_SSEL, DFE_FL_SYNC_GEN_SIG_ALWAYS) \
                               | CSL_FMK(DFE_DPDA_INITS_REG_INIT_CLK_GATE, 1) \
                               | CSL_FMK(DFE_DPDA_INITS_REG_INIT_STATE, 1);
        }
        for (i=0; i<16; i++) {
            hDpda->regs->inits = CSL_FMK(DFE_DPDA_INITS_REG_INITS_SSEL, DFE_FL_SYNC_GEN_SIG_ALWAYS) \
                               | CSL_FMK(DFE_DPDA_INITS_REG_INIT_CLK_GATE, 1);
        }
        hDpda->regs->inits = CSL_FMK(DFE_DPDA_INITS_REG_INITS_SSEL, DFE_FL_SYNC_GEN_SIG_ALWAYS);

        // Restore memory access rights
        hMisc->regs->cfg2 = CSL_FMK(DFE_MISC_CFG2_REG_MEM_MPU_ACCESS, memMpuAccess);
    }

    return DFE_ERR_NONE;
} // End of Dfe_loadDpda()

/**
 * @brief Write samples to capture buffer
 * @ingroup DFE_LLD_DPDA_FUNCTION
 *
 * Write samples to capture buffer.  DPDA will read them from capture buffer.
 * For the given CB buffer, the API writes each given sample to the 16MSB of
 * the corresponding 18-bit CB sample and clears the 2 LSB.  It sets the buffer
 * in fine mode and tags it as storing reference or feedback samples.  It also
 * makes sure that the buffer won't skip chunks.
 *
 *  @param hDfe [in] DFE device handle
 *  @param cBufId [in] Identifies CB buffer, 0 ~ 3
 *  @param fbFlag [in] Indicates whether buffer is used for capturing feedback signal
 *                      0 = no feedback signal
 *                      1 = feedback signal
 *  @param cbLength [in] Specifies number of samples, 0 ~ 8192
 *  @param cbTemp [in] Points to the temporary buffer, size 8192*4 bytes
 *  @param cbData [in] Points to the cb data
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_writeDpdaSamples(
                         DFE_Handle hDfe,
                         DfeFl_CbBuf cbBufId,
                         uint8_t fbFlag,
                         uint16_t cbLength,
                         DfeFl_CbComplexInt *cbTemp,
                         DFE_CbData *cbData
                         )
{
    DfeFl_Status      status;
    DfeFl_CbBufMode   cbBufMode;
    DfeFl_CbModeSet   cbBufModeSet;
    DfeFl_CbData      cbRawData;
    DfeFl_CbSkipChunk cbSkipChunk;
    uint32_t          j, srcIdx;

    VALID_DFE_HANDLE(hDfe);

    if (cbBufId > DFE_FL_CB_NUM_BUF) {
        Dfe_osalLog("CB ID out of range!");
        return DFE_ERR_INVALID_PARAMS;
    }

    if ((2*cbLength) >= DFE_FL_MAX_CB_LENGTH) {
        Dfe_osalLog("CB length out of range!");
        return DFE_ERR_INVALID_PARAMS;
    }

    if (hDfe->dpdaIsDisabled == 0)
    {
        cbBufMode.cbBuf = cbBufId;
        cbBufMode.data = (uint32_t)DFE_FL_CB_MPU;
        CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_BUF_MODE, &cbBufMode) );

        // start writing from beginning of buffer
        srcIdx = 0;

        // unpack the input in the cbTemp used to write to DPDPA memory
        for (j = 0; j < cbLength; j++)
        {
            // WARNING: DPDA swaps I and Q fields when reading from the capture buffer
            cbTemp[2*j].real = (uint16_t)cbData[j].Qdata;
            cbTemp[2*j].imag = (uint16_t)cbData[j].Idata;
            cbTemp[2*j + 1].real = (uint16_t)cbData[j].Qdata;
            cbTemp[2*j + 1].imag = (uint16_t)cbData[j].Idata;
        }

        // write from start to end of buffer
        cbRawData.cbBuf = cbBufId;
        cbRawData.startPos = srcIdx;
        cbRawData.size = 2*cbLength;
        cbRawData.data = cbTemp;
        CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_MSB, &cbRawData) );

        // clear LSB
        for (j = 0; j < cbLength; j++)
        {
            // WARNING: DPDA swaps I and Q fields when reading from the capture buffer
            cbTemp[2*j].real = 0;
            cbTemp[2*j].imag = 0;
            cbTemp[2*j + 1].real = 0;
            cbTemp[2*j + 1].imag = 0;
        }
        CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_LSB, &cbRawData) );

        // set buffer in fine mode
        cbBufMode.cbBuf = cbBufId;
        cbBufMode.data = DFE_FL_CB_F;
        CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_BUF_MODE, &cbBufMode) );

        // set buffer as reference or feedback
        cbBufModeSet.cbBuf = cbBufId;
        cbBufModeSet.sel = 0;
        cbBufModeSet.busSel = 0;
        cbBufModeSet.ref_or_fb = fbFlag;
        cbBufModeSet.not_used = 0;
        CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_MODE_SET, &cbBufModeSet) );

        // clear skipchunk
        cbSkipChunk.readref_skipchunk = 0; // don't skip
        cbSkipChunk.readfb_skipchunk  = 0; // don't skip
        CSL_HW_CTRL( dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_SKIP_CHUNK, &cbSkipChunk) );
    }

    return DFE_ERR_NONE;
}

/**
 * @brief Writes DPDA scalar register
 * @ingroup DFE_LLD_DPDA_FUNCTION
 *
 * Write complex value to DPDA scalar register file.
 * The API converts the I and Q parts of the given complex value to the custom
 * floating point format used by DPDA and writes them to the given scalar register.
 *
 *  @param hDfe [in] DFE device handle
 *  @param scalarId [in] Identifies scalar register, 0 ~ 63
 *  @param iScalar [out] I part of scalar value
 *  @param qScalar [out] Q part of scalar value
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_writeDpdaScalar(
                        DFE_Handle hDfe,
                        uint8_t scalarId,
                        float iScalar,
                        float qScalar
                        )
{
    DfeFl_DpdaHandle hDpda = hDfe->hDfeDpda[0];

    double refValue;
    int32_t expValue, realValue, imagValue;
    int32_t ieData, qData;

    volatile CSL_DFE_DPDA_DPDA_SCALAR_REGS *regs;

    VALID_DFE_HANDLE(hDfe);

    if (scalarId >= 64) {
        Dfe_osalLog("Scalar ID out of range!");
        return DFE_ERR_INVALID_PARAMS;
    }

    if (fabs(iScalar) > fabs(qScalar)) {
        refValue = fabs(iScalar);
    } else {
        refValue = fabs(qScalar);
    }
    if (refValue != 0.0) {
        expValue = floor(log2(refValue)) - 21;
        realValue = round((double)iScalar * (double)((long long)(1) << (-expValue)));
        imagValue = round((double)qScalar * (double)((long long)(1) << (-expValue)));
        expValue = expValue + 22;
    } else {
        expValue = -128;
        realValue = 0;
        imagValue = 0;
    }
    ieData = (realValue & 0x07FFFFF) + (expValue & 0x0FF)* (1 << 23);
    qData = (imagValue & 0x07FFFFF);

    if (hDfe->dpdaIsDisabled == 0)
    {
        regs = &hDpda->regs->dpda_scalar[0];
        regs[scalarId].ie_register = CSL_FMK(DFE_DPDA_DPDA_SCALAR_IE_REGISTER_REG_DPDA_SCALAR_IE_REGISTER, ieData);
        regs[scalarId].q_register = CSL_FMK(DFE_DPDA_DPDA_SCALAR_Q_REGISTER_REG_DPDA_SCALAR_Q_REGISTER, qData);
    }

    return DFE_ERR_NONE;
} // End of Dfe_writeDpdaScalar()

/**
 * @brief Write value to DPDA IG register file
 * @ingroup DFE_LLD_DPDA_FUNCTION
 *
 * The API writes the given value to the given IG register.
 *
 *  @param hDfe [in] DFE device handle
 *  @param igId [in] Identifies IG register, 0 ~ 63
 *  @param iq [in] Value
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_writeDpdaIg(
                   DFE_Handle hDfe,
                   uint8_t igId,
                   uint32_t ig
)
{
    DfeFl_DpdaHandle hDpda = hDfe->hDfeDpda[0];

    volatile uint32_t *regs;

    VALID_DFE_HANDLE(hDfe);

    if (igId >= 64) {
        Dfe_osalLog("IG register ID out of range!");
        return DFE_ERR_INVALID_PARAMS;
    }

    if (hDfe->dpdaIsDisabled == 0)
    {
        regs = &hDpda->regs->dpda_ig_regfile[0];
        regs[igId] = ig;
    }

    return DFE_ERR_NONE;
} // End of Dfe_writeDpdaIg()

/**
 * @brief Read DPDA scalar register
 * @ingroup DFE_LLD_DPDA_FUNCTION
 *
 * The API reads complex value from DPDA scalar register file.
 * The API reads a complex value from the given scalar register
 * and it converts the I and Q parts of the complex value from
 * the custom floating point format used by DPDA.
 *
 *  @param hDfe [in] DFE device handle
 *  @param scalarId [in] Identifies scalar register, 0 ~ 63
 *  @param iScalarPtr [in] Points to I part of scalar value
 *  @param qScalarPtr [in] Points to Q part of scalar value
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_readDpdaScalar(
                      DFE_Handle hDfe,
                      uint8_t scalarId,
                      float *iScalarPtr,
                      float *qScalarPtr
                      )
{
    DfeFl_DpdaHandle hDpda = hDfe->hDfeDpda[0];
    uint32_t         ieData;
    uint32_t         qData;
    int32_t          iMantissa;
    int32_t          qMantissa;
    int32_t          exponent;
    float           fiMantissa;
    float           fqMantissa;
    double          val;

    volatile CSL_DFE_DPDA_DPDA_SCALAR_REGS *regs;

    VALID_DFE_HANDLE(hDfe);

    if (scalarId >= 64) {
        Dfe_osalLog("Scalar ID out of range!");
        return DFE_ERR_INVALID_PARAMS;
    }

    if (hDfe->dpdaIsDisabled == 0)
    {
        regs = &hDpda->regs->dpda_scalar[0];
        ieData = CSL_FEXT(regs[scalarId].ie_register, DFE_DPDA_DPDA_SCALAR_IE_REGISTER_REG_DPDA_SCALAR_IE_REGISTER);
        qData = CSL_FEXT(regs[scalarId].q_register, DFE_DPDA_DPDA_SCALAR_Q_REGISTER_REG_DPDA_SCALAR_Q_REGISTER);

        iMantissa = (((int32_t)ieData & 0x7FFFFF) << 9) >> 9;
        qMantissa = (((int32_t)qData & 0x7FFFFF) << 9) >> 9;

        exponent = ((int32_t)(ieData << 1))>>24;
        if ((exponent >= 127) || (exponent < -128))
        {
            return DFE_ERR_INVALID_PARAMS;
        }
        exponent = exponent - 22;

        // convert parameters to floating point
        val = pow(2.0, (double)(exponent));
        fiMantissa = (float)iMantissa*val;
        fqMantissa = (float)qMantissa*val;

        // fill in response
        *iScalarPtr = fiMantissa;
        *qScalarPtr = fqMantissa;
    }

    return DFE_ERR_NONE;
} // End of Dfe_readDpdaScalar()

/**
 * @brief Read complex parameters from DPDA solution RAM
 * @ingroup DFE_LLD_DPDA_FUNCTION
 *
 * Read complex parameters from DPDA solution RAM.
 * The API reads all parameters stored in the given lines from the solution RAM.
 * Note that each line stores 24 complex parameters. It also converts the I and
 *  Q parts of each parameter from the custom floating point format used by the
 *  DPDA and re-orders the parameters as follows:
 *  for (i = 0; i < lineNum; i++) {
 *    for (j = 0; j < 8; j++) {
 *      for (k = 0; k < 3; k++) {
 *        paramTbl[24*2*i+8*2*k+2*j] = tmpTbl[24*2*i+3*2*j+k*2];
 *        paramTbl[24*2*i+8*2*k+2*j+1] = tmpTbl[24*2*i+3*2*j+k*2+1];
 *      }
 *    }
 *  }
 *
 *
 *  @param hDfe [in] DFE device handle
 *  @param lineId [in] Identifies first line, 0 ~ 767
 *  @param lineNum [in] Specifies number of lines, 0 ~ 767
 *  @param paramTbl [out] Points to table of parameters.  Even locations store the I parts
 *   of the corresponding parameters and odd locations the Q parts.
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_readDpdaParams(
                      DFE_Handle hDfe,
                      uint16_t lineId,
                      uint16_t lineNum,
                      float *paramTbl
                      )
{
    DfeFl_Status    status;
    DfeFl_DpdaHandle hDpda = hDfe->hDfeDpda[0];
    uint32_t        i, j, k;
    DfeFl_DpdaPreg  dpdaPreg;
    uint32_t        iedata[24];
    uint32_t        iqdata[24];
    float           params_hw[120*2];
    int32_t         i_mantissa;
    int32_t         q_mantissa;
    int32_t         i_exp;
    float           fi_mantissa;
    float           fq_mantissa;
    float         * local_params_ptr = params_hw;
    float         * params_sw = paramTbl;
    double          val;
    int32_t         exponent;

    VALID_DFE_HANDLE(hDfe);

    if ((lineId + lineNum) >= 768) {
        Dfe_osalLog("Line ID out of range!");
        return DFE_ERR_INVALID_PARAMS;
    }

    // hard-coded 24 columns or 24 entries per line
    dpdaPreg.numEntry = 24;

    // allocate memory for iedata and qdata (1 line)
    dpdaPreg.iedata = (uint32_t *)iedata;
    dpdaPreg.qdata  = (uint32_t *)iqdata;

    if (hDfe->dpdaIsDisabled == 0)
    {
        for (i = lineId; i < (lineId + lineNum); i++)
        {
            dpdaPreg.idx = i;

            // read for idx line 24 parameters from solution RAM
            CSL_HW_CTRL( dfeFl_DpdaGetHwStatus(hDpda, DFE_FL_DPDA_QUERY_GET_PREG, &dpdaPreg) );

            for (j = 0; j < dpdaPreg.numEntry; j++)
            {
                // extract parameters
                i_mantissa = (((int32_t)dpdaPreg.iedata[j] &0x7FFFFF) << 9)>>9;
                i_exp = ((int32_t)(dpdaPreg.iedata[j] << 1))>>24;
                q_mantissa = (((int32_t)dpdaPreg.qdata[j] &0x7FFFFF) << 9)>>9;

                if ((i_exp >= 127) || (i_exp < -128))
                {
                    return DFE_ERR_INVALID_PARAMS;
                }

                // convert parameters to floating point
                exponent = i_exp - 22;
                val = pow(2.0, (double)(exponent));
                fi_mantissa = (float)i_mantissa*val;
                fq_mantissa = (float)q_mantissa*val;

                // fill in response param
                *local_params_ptr++ = fi_mantissa;
                *local_params_ptr++ = fq_mantissa;
            }
        }

        // change order of parameters
        for (i = 0; i < lineNum; i++)
        {
            for (j = 0; j < 8; j++)
            {
                for (k = 0; k < 3; k++)
                {
                    params_sw[24*2*i + 8*2*k + 2*j]     = params_hw[24*2*i + 3*2*j + k*2];
                    params_sw[24*2*i + 8*2*k + 2*j + 1] = params_hw[24*2*i + 3*2*j + k*2 + 1];
                }
            }
        }
    }

    return DFE_ERR_NONE;
} // End of Dfe_readDpdaParams()

/**
 * @brief Read DPDA IG register
 * @ingroup DFE_LLD_DPDA_FUNCTION
 *
 * The API reads a value from the given IG register.
 *
 *  @param hDfe [in] DFE device handle
 *  @param iqId [in] Identifies IG register, 0 ~ 63
 *  @param igPtr [out] Points to value
 *
 * @return
 *  - #DFE_ERR_NONE, if API complete programmed properly
 *  - #DFE_ERR_INVALID_HANDLE, if hDfe is NULL
 *  - #DFE_ERR_INVALID_PARAMS, if invalid parameters
 *
 *
 * @pre
 *  - hDfe should be a valid handle opened by Dfe_open().
 *
 * @post
 * - None.
 *
 * @b Example
 *   @verbatim
         [to be documented]
     @endverbatim
 */
DFE_Err Dfe_readDpdaIg(
                  DFE_Handle hDfe,
                  uint8_t igId,
                  uint32_t *igPtr
                  )
{
    DfeFl_DpdaHandle hDpda = hDfe->hDfeDpda[0];

    volatile uint32_t *regs;

    VALID_DFE_HANDLE(hDfe);

    if (igId >= 64) {
        Dfe_osalLog("IG register ID out of range!");
        return DFE_ERR_INVALID_PARAMS;
    }

    if (hDfe->dpdaIsDisabled == 0)
    {
        regs = &hDpda->regs->dpda_ig_regfile[0];
        *igPtr = regs[igId];
    }

    return DFE_ERR_NONE;
} // End of Dfe_readDpdaIg()

