/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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

/** ===========================================================================
 *   @file  iqn2fl_getHwStatus.c
 *
 *   @brief  IQN2 get hardware status function
 *
 */

#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_getHwStatusAux.h>

/** ============================================================================
 *   @n@b Iqn2Fl_getHwStatus
 *
 *   @b Description
 *   @n This function is used to get the status value of various parameters of the
 *      IQN2 instance. The value returned depends on the query passed.
 *
 *   @b Arguments
 *   @verbatim
            hIqn2         Handle to the iqn2 instance
 
            Query         Query to be performed 
 
            Response      Pointer to buffer to return the data requested by
                          the query passed
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *   @li                    IQN2FL_SOK            - Successful completion of the
 *                                               query
 *
 *   @li                    IQN2FL_BADHANDLE - Invalid handle
 *
 *   @li                    IQN2FL_INVQUERY  - Query command not supported
 *
 *   @li                    IQN2FL_FAIL      - Generic failure
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *       Data requested by query is returned through the variable "response"
 *
 *   @b Writes
 *   @n The input argument "response" is modified
 *
 *   @b Example
 *   @verbatim
        
        // handle for IQN2
        Iqn2Fl_Handle hIqn2;
        // other related declarations
        ...
        // ctrl argument for hw command
        Bool ctrlArg;
        // query response
        uint32_t response;

        // Open handle  - for use 
        hIqn2 = Iqn2Fl_open(&Iqn2Obj, CSL_IQN, &iqn2Param, &status);

        if ((hIqn2 == NULL) || (status != IQN2FL_SOK))
        {
           printf ("\nError opening CSL_IQN");
           exit(1);
        }

        // Do config 
        Config.globalSetup = &gblCfg;
        ...
        //Do setup 
        Iqn2Fl_hwSetup(hIqn2, &Config);

        ctrlArg = 1;
        hIqn2->arg_ail = IQN2FL_AIL_0; // Using AIL_0 instance

        // Send hw control command to global enable EFE of AIL_0 
        Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
        ...
        wait(100); //wait for a sufficient length of time
    
        // Get status of global enable state
        hIqn2->arg_ail = IQN2FL_AIL_0; // for AIL_0 instance
        Iqn2Fl_getHwStatus(hIqn2, IQN2FL_AIL_EFE_GLOBAL_EN_STATUS, (void *)&response);

     @endverbatim
 * =============================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Iqn2Fl_getHwStatus, ".text:iqn2fl");
#endif
Iqn2Fl_Status  Iqn2Fl_getHwStatus(
   Iqn2Fl_Handle                      hIqn2,
   Iqn2Fl_HwStatusQuery               Query,
   void                               *response
)
{
    Iqn2Fl_Status status = IQN2FL_SOK;
    
    if (hIqn2 == NULL)
        return IQN2FL_BADHANDLE;
    
    switch (Query) 
    {
        /** AT2_BCN_PA_TSCOMP_CAPT_STS
         *  CPTS module from NETCP/PA provides a measurement pulse 
         *  called pa_tscomp. When this pulse fires, the BCN Timer 
         *  value is captured to this register. SW may then use this 
         *  captured value to calculate a BCN timer error which is 
         *  corrected by writing to the BCN Offset register.
         *  Response: (uint32_t *) - BCN clock count pa_tscomp capture
         */
        case IQN2FL_QUERY_AT2_BCN_PA_TSCOMP_CAPT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->At2.AT2_BCN.AT2_BCN_PA_TSCOMP_CAPT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_BCN_PA_TSCOMP_CAPT_STS_RD_VAL);
            }
            break;

        /** AT2_BCN_PHYSYNC_CAPT_STS
         *  An external pin called PHYSYNC provides an optional sync 
         *  measurement source. When a rising edge is detected on this 
         *  pin, the BCN Timer value is captured to this register.
         *  Response: (uint32_t *) - BCN clock count physync capture
         */
        case IQN2FL_QUERY_AT2_BCN_PHYSYNC_CAPT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->At2.AT2_BCN.AT2_BCN_PHYSYNC_CAPT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_BCN_PHYSYNC_CAPT_STS_RD_VAL);
            }
            break;

        /** AT2_BCN_RADSYNC_CAPT_STS
         *  An external pin called RADSYNC provides an optional sync 
         *  measurement source. When a rising edge is detected on this 
         *  pin, the BCN Timer value is captured to this register.
         *  Response: (uint32_t *) - BCN clock count radsync capture
         */
        case IQN2FL_QUERY_AT2_BCN_RADSYNC_CAPT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->At2.AT2_BCN.AT2_BCN_RADSYNC_CAPT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_BCN_RADSYNC_CAPT_STS_RD_VAL);
            }
            break;

        /** AT2_BCN_RP1_SYNC_CAPT_STS
         *  OBSAI RP1 optionally may be used as a sync measurement source. 
         *  When a successful RP1 FCB is recieved, optionally a system 
         *  event fires, the FCB message is captured to registers, and 
         *  the BCN timer value is captured to this register.
         *  Response: (uint32_t *) - BCN clock count rp1_sync capture
         */
        case IQN2FL_QUERY_AT2_BCN_RP1_SYNC_CAPT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->At2.AT2_BCN.AT2_BCN_RP1_SYNC_CAPT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_BCN_RP1_SYNC_CAPT_STS_RD_VAL);
            }
            break;

        /** AT2_BCN_SLVSYNC_CAPT_STS
         *  The uAT will provide frame sync pulses back to the AT2. When this 
         *  pulse fires, the BCN Timer value is captured to this register. 
         *  SW may then use this captured value to calculate a BCN timer 
         *  error which is corrected by writing to the BCN Offset register.
         *  Response: (uint32_t *) - BCN clock count selected slave uAT sync capture. 
         *            bcn_slvsel_cfg selects which uAT slave sync is used 
         *            for capturing the BCN value in this register.
         */
        case IQN2FL_QUERY_AT2_BCN_SLVSYNC_CAPT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->At2.AT2_BCN.AT2_BCN_SLVSYNC_CAPT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_BCN_SLVSYNC_CAPT_STS_RD_VAL);
            }
            break;

        /** AT2_BCN_FRM_VALUE_LSB_STS
         *  BCN Frame count value (BTS Frame Number). Increments every time 
         *  10ms BCN timer wraps. This Register only allows reading of the 
         *  value (Part2).
         *  Response: (uint32_t *) - BCN Frame Value LSBs
         */
        case IQN2FL_QUERY_AT2_BCN_FRM_VALUE_LSB_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_BCN_FRM_VALUE_LSB_STS_RD_VAL);
            }
            break;

        /** AT2_BCN_FRM_VALUE_MSB_STS
         *  BCN Frame count value (BTS Frame Number). Increments every time 
         *  10ms BCN timer wraps. This Register only allows reading of the 
         *  value (Part1).
         *  Response: (uint32_t *) - BCN Frame Value MSBs
         */
        case IQN2FL_QUERY_AT2_BCN_FRM_VALUE_MSB_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_MSB_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_BCN_FRM_VALUE_MSB_STS_RD_VAL);
            }
            break;

        /** AT2_GSM_TCOUNT_VALUE_STS
         *  Special GSM T1, T2, T3 count GSM frames in way which is useful to 
         *  GSM only APP SW. This register is for only for reading the 
         *  current state (Users should avoid reading the value near the 
         *  increment to avoid uncertainty)
         *  Argument: a pointer to an array of 3 uint32_t elements
         *  Response: (uint32_t *) - Value of counters
         *            response[0] = value of T1 counter
         *            response[1] = value of T2 counter
         *            response[2] = value of T3 counter
         */
        case IQN2FL_QUERY_AT2_GSM_TCOUNT_VALUE_STS:
            {
                uint32_t tmpReg, *respVal = (uint32_t *)response;
                tmpReg = hIqn2->regs->At2.AT2_GSM.AT2_GSM_TCOUNT_VALUE_STS;
                respVal[0] = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_GSM_TCOUNT_VALUE_STS_T1);
                respVal[1] = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_GSM_TCOUNT_VALUE_STS_T2);
                respVal[2] = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_GSM_TCOUNT_VALUE_STS_T3);
            }
            break;

        /** AT2_RADT_0_STS, AT2_RADT_1_STS, AT2_RADT_2_STS, AT2_RADT_3_STS
         *  radt_samp_symb_value, RADT Frame Value 12 LSBs, radt_fr_value_msbs, 
         *  RADT_CHIP, RADT_SLOT and RADT_FRM.
         *  RADT is specific format which is expected by the RAC and TAC HW 
         *  accelerator modules. EDMA performs a read of these values each 
         *  RAC or TAC iteration cycle giving RAC and TAC a sense of real time.
         *  Argument: a pointer to Iqn2Fl_At2RadtStatus
         *  Response: Iqn2Fl_At2RadtStatus *
         *            For the "radt_num" provided by APP SW
         */
        case IQN2FL_QUERY_AT2_RADT_STS:
            {
                Iqn2Fl_getAt2RadtStatus (hIqn2, (Iqn2Fl_At2RadtStatus *)response);
            }
            break;

        /** AT2 EVENTS - EVT ENABLE CFG - STATUS
         *  AT2 system events control APP SW timing. This MMR Enables each 
         *  of 24 of these system events.
         *  This query reads the value of this register. EVENT Enable when 
         *  the bit is set to a 1 and 0 means EVENT Disable.
         *  Response: (uint32_t *) AT2 EVT ENABLE CFG register value
         */
        case IQN2FL_QUERY_AT2_EVENTS_ENABLE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->At2.AT2_EVENTS.AT2_EVT_ENABLE_CFG;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_EVT_ENABLE_CFG_EN);
            }
            break;

        /** VC_CDMA_TSTAT_L_TDOWN_STS
         *  CDMA TX takedown status lsb Register.
         *  Response: (uint32_t *) CDMA TX takedown status lsb
         */
        case IQN2FL_QUERY_VC_CDMA_TSTAT_L_TDOWN_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_TSTAT_L_TDOWN_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_TSTAT_L_TDOWN_STS_TDOWN_STATUS_L);
            }
            break;

        /** VC_CDMA_TSTAT_H_TDOWN_STS
         *  CDMA TX takedown status msb Register.
         *  Response: (uint32_t *) CDMA TX takedown status msb
         */
        case IQN2FL_QUERY_VC_CDMA_TSTAT_H_TDOWN_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_TSTAT_H_TDOWN_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_TSTAT_H_TDOWN_STS_TDOWN_STATUS_H);
            }
            break;

        /** VC_CDMA_TSTAT_L_ENABLE_STS
         *  CDMA TX enable status lsb Register.
         *  Response: (uint32_t *) CDMA TX enable status lsb
         */
        case IQN2FL_QUERY_VC_CDMA_TSTAT_L_ENABLE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_TSTAT_L_ENABLE_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_TSTAT_L_ENABLE_STS_ENABLE_STATUS_L);
            }
            break;

        /** VC_CDMA_TSTAT_H_ENABLE_STS
         *  CDMA TX enable status msb Register.
         *  Response: (uint32_t *) CDMA TX enable status msb
         */
        case IQN2FL_QUERY_VC_CDMA_TSTAT_H_ENABLE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_TSTAT_H_ENABLE_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_TSTAT_H_ENABLE_STS_ENABLE_STATUS_H);
            }
            break;

        /** VC_CDMA_TSTAT_L_PKT_STS
         *  CDMA TX packet status lsb Register.
         *  Response: (uint32_t *) CDMA TX packet status lsb
         */
        case IQN2FL_QUERY_VC_CDMA_TSTAT_L_PKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_TSTAT_L_PKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_TSTAT_L_PKT_STS_PKT_STATUS_L);
            }
            break;

        /** VC_CDMA_TSTAT_H_PKT_STS
         *  CDMA TX packet status msb Register.
         *  Response: (uint32_t *) CDMA TX packet status msb
         */
        case IQN2FL_QUERY_VC_CDMA_TSTAT_H_PKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_TSTAT_H_PKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_TSTAT_H_PKT_STS_PKT_STATUS_H);
            }
            break;

        /** VC_CDMA_RSTAT_L_TDOWN_STS
         *  CDMA RX takedown status lsb Register.
         *  Response: (uint32_t *) CDMA RX takedown status lsb
         */
        case IQN2FL_QUERY_VC_CDMA_RSTAT_L_TDOWN_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_RSTAT_L_TDOWN_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_RSTAT_L_TDOWN_STS_TDOWN_STATUS_L);
            }
            break;

        /** VC_CDMA_RSTAT_H_TDOWN_STS
         *  CDMA RX takedown status msb Register.
         *  Response: (uint32_t *) CDMA RX takedown status msb
         */
        case IQN2FL_QUERY_VC_CDMA_RSTAT_H_TDOWN_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_RSTAT_H_TDOWN_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_RSTAT_H_TDOWN_STS_TDOWN_STATUS_H);
            }
            break;

        /** VC_CDMA_RSTAT_L_ENABLE_STS
         *  CDMA RX enable status lsb Register.
         *  Response: (uint32_t *) CDMA RX enable status lsb
         */
        case IQN2FL_QUERY_VC_CDMA_RSTAT_L_ENABLE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_RSTAT_L_ENABLE_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_RSTAT_L_ENABLE_STS_ENABLE_STATUS_L);
            }
            break;

        /** VC_CDMA_RSTAT_H_ENABLE_STS
         *  CDMA RX enable status msb Register.
         *  Response: (uint32_t *) CDMA RX enable status msb
         */
        case IQN2FL_QUERY_VC_CDMA_RSTAT_H_ENABLE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_RSTAT_H_ENABLE_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_RSTAT_H_ENABLE_STS_ENABLE_STATUS_H);
            }
            break;

        /** VC_CDMA_RSTAT_L_PKT_STS
         *  CDMA RX packet status lsb Register.
         *  Response: (uint32_t *) CDMA RX packet status lsb
         */
        case IQN2FL_QUERY_VC_CDMA_RSTAT_L_PKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_RSTAT_L_PKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_RSTAT_L_PKT_STS_PKT_STATUS_L);
            }
            break;

        /** VC_CDMA_RSTAT_H_PKT_STS
         *  CDMA RX packet status msb Register.
         *  Response: (uint32_t *) CDMA RX packet status msb
         */
        case IQN2FL_QUERY_VC_CDMA_RSTAT_H_PKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Top.VC_CDMA_STATUS.VC_CDMA_RSTAT_H_PKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_CDMA_RSTAT_H_PKT_STS_PKT_STATUS_H);
            }
            break;

        /** VC_SD_TX_STS
         *  SERDES transmit status Register
         *  Argument: a pointer to an array of 4 uint32_t elements
         *  Response: (uint32_t *) - Link clocks status via
         *                         response[0], response[1], 
         *                         response[2] and response[3]
         */
        case IQN2FL_QUERY_VC_SD_TX_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 4; i++)
                {
                    tmpReg = hIqn2->regs->Top.VC_SD_LK[i].VC_SD_TX_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_VC_SD_TX_STS_LINKSTATUS_OK);
                }
            }
            break;

        /** VC_SD_RX_STS
         *  SERDES receive status Register
         *  Argument: a pointer to an array of 4 uint32_t elements
         *  Response: (uint32_t *) - Status via
         *                         response[0], response[1], 
         *                         response[2] and response[3]
         *  Bit-0: COMMA_DET - STSRX 0 The receiver frame synchronizer must 
         *                     have knowledge that each Ser-des port had 
         *                     completed a requested byte alignment so that 
         *                     the byte alignment control logic can operate.
         *  Bit-1: LOSDTCT - STSRX 1 The receiver frame synchronizer must have 
         *                   knowledge that each Ser-des port had detected a 
         *                   loss of signal condition so that the receiver 
         *                   can suppress events due to a loss of frame 
         *                   synchronization.
         */
        case IQN2FL_QUERY_VC_SD_RX_STS:
            {
                uint32_t i, *respVal = (uint32_t *)response;
                for (i = 0; i < 4; i++)
                {
                    respVal[i] = (uint32_t) hIqn2->regs->Top.VC_SD_LK[i].VC_SD_RX_STS;
                }
            }
            break;

        /** AID2_IQ_EFE_CHAN_ON_STS
         *  Gives current On/Off Status of every available stream. One bit per 
         *  channel. Required because channels only turn on/off on radio frame 
         *  so the chan_en alone does not give channel status. Chan on/off is 
         *  not tracked for packet channels; These bits are 0 for packet 
         *  channels.
         *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
         */
        case IQN2FL_QUERY_AID2_IQ_EFE_CHAN_ON_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_CHAN_ON_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_EFE_CHAN_ON_STS_CHAN_ON);
            }
            break;

        /** AID2_IQ_EFE_IN_PKT_STS
         *  Gives current In/Out packet state of packet channels only. Bits 
         *  are always zero for AxC channels. Bit is activated at SOP, holds 
         *  high mid packet, deactivates at EOP.
         *  Response: (uint32_t *) - 0x1:IN_PKT, 0x0:OUT_PKT
         */
        case IQN2FL_QUERY_AID2_IQ_EFE_IN_PKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_IN_PKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_EFE_IN_PKT_STS_IN_PKT);
            }
            break;

        /** AID2_IQ_EFE_DMA_SYNC_STS
         *  Gives current In/Out packet state of packet channels only. Bits are 
         *  always zero for AxC channels. Bit is activated at SOP, holds high 
         *  mid packet, deactivates at EOP.
         *  Response: (uint32_t *) - 0x1: DMA synchronized to radio timing for 
         *                              this channel 
         *                         0x0: DMA not synchronized to radio timing 
         *                              for this channel. Channel is in
         *                              re-sync mode.
         */
        case IQN2FL_QUERY_AID2_IQ_EFE_DMA_SYNC_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_DMA_SYNC_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_EFE_DMA_SYNC_STS_DMA_SYNC);
            }
            break;

        /** AID2_IQ_IFE_CHAN_ON_STS
         *  Gives current On/Off Status of every available stream. One bit per 
         *  channel. Required because channels only turn on/off on radio frame 
         *  so the chan_en alone does not give channel status. Chan on/off is 
         *  not tracked for packet channels; These bits are 0 for packet 
         *  channels.
         *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
         */
        case IQN2FL_QUERY_AID2_IQ_IFE_CHAN_ON_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_IQ_IFE_CONFIG_GROUP.AID2_IQ_IFE_CHAN_ON_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_IFE_CHAN_ON_STS_CHAN_ON);
            }
            break;

        /** AID2_IQ_IFE_IN_PKT_STS
         *  Gives current In/Out packet state of packet channels only. Bits 
         *  are always zero for AxC channels. Bit is activated at SOP, holds 
         *  high mid packet, deactivates at EOP.
         *  Response: (uint32_t *) - 0x1:IN_PKT, 0x0:OUT_PKT
         */
        case IQN2FL_QUERY_AID2_IQ_IFE_IN_PKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_IQ_IFE_CONFIG_GROUP.AID2_IQ_IFE_IN_PKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_IFE_IN_PKT_STS_IN_PKT);
            }
            break;

        /** AID2_IQ_IDC_STS
         *  IDC Status register.
         *  Empty indicator for both the PSI Staging FIFO and the CDC FIFO
         *     FIFO_NOT_EMPTY (00) = PSI Staging FIFO and CDC FIFO are not both empty
         *     FIFO_EMPTY (11) = PSI Staging FIFO and CDC FIFO are both empty
         *  Response: (uint32_t *) - Empty indicator as mentioned above
         */
        case IQN2FL_QUERY_AID2_IQ_IDC_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_IQ_IDC_GENERAL_STATUS_GROUP.AID2_IQ_IDC_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_IDC_STS_EMPTY);
            }
            break;

        /** AID2_IQ_IDC_INPKT_STS
         *  Per-channel in packet status bits where a 0 indicates that the 
         *  channel is not actively processing a packet and a 1 indicates 
         *  that it is actively processing a packet. The inpkt to channel 
         *  assignment is such that inpkt[0] is associated with channel 0 
         *  and inpkt[15] is associated with channel 15.
         *  Response: (uint32_t *) - In-packet status
         */
        case IQN2FL_QUERY_AID2_IQ_IDC_INPKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_IQ_IDC_GENERAL_STATUS_GROUP.AID2_IQ_IDC_INPKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_IDC_INPKT_STS_INPKT);
            }
            break;

        /** AID2_ECTL_CHAN_ON_STS
         *  Gives current On/Off Status of every available stream. One bit per 
         *  channel. Required because channels only turn on/off on radio frame 
         *  so the chan_en alone does not give channel status. Chan on/off is 
         *  not tracked for packet channels; These bits are 0 for packet channels.
         *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
         */
        case IQN2FL_QUERY_AID2_ECTL_CHAN_ON_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_ECTL_PKT_IF.AID2_ECTL_CHAN_ON_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_ECTL_CHAN_ON_STS_CHAN_ON);
            }
            break;

        /** AID2_ECTL_INPKT_STS
         *  Indicates when a channel is actively receiving a packet from the ECTL.
         *  Per-channel in packet status bits where a 0 indicates that the 
         *  channel is not actively processing a packet and a 1 indicates 
         *  that it is actively processing a packet. The inpkt to channel 
         *  assignment is such that inpkt[0] is associated with channel 0 
         *  and inpkt[15] is associated with channel 15.
         *  Response: (uint32_t *) - In-packet status
         */
        case IQN2FL_QUERY_AID2_ECTL_INPKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_ECTL_PKT_IF.AID2_ECTL_INPKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_ECTL_INPKT_STS_IN_PKT);
            }
            break;

        /** AID2_ICTL_CHAN_ON_STS
         *  Gives current On/Off Status of every available stream. One bit per 
         *  channel. Required because channels only turn on/off on radio frame 
         *  so the chan_en alone does not give channel status. Chan on/off is 
         *  not tracked for packet channels; These bits are 0 for packet channels.
         *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
         */
        case IQN2FL_QUERY_AID2_ICTL_CHAN_ON_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_ICTL_PKT_IF.AID2_ICTL_CHAN_ON_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_ICTL_CHAN_ON_STS_CHAN_ON);
            }
            break;

        /** AID2_ICTL_INPKT_STS
         *  Indicates when a channel is actively receiving a packet from the ICTL.
         *  Per-channel in packet status bits where a 0 indicates that the 
         *  channel is not actively processing a packet and a 1 indicates 
         *  that it is actively processing a packet. The inpkt to channel 
         *  assignment is such that inpkt[0] is associated with channel 0 
         *  and inpkt[15] is associated with channel 15.
         *  Response: (uint32_t *) - In-packet status
         */
        case IQN2FL_QUERY_AID2_ICTL_INPKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_ICTL_IDC_IF.AID2_ICTL_INPKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_ICTL_INPKT_STS_IN_PKT);
            }
            break;

        /** AID2_UAT_SYNC_BCN_CAPTURE_STS
         *  UAT SYNC BCN capture Register.
         *  uAT raw BCN value captured each frame boundary of AT2 master BCN. 
         *  Used to calculate uAT BCN offset value for the purpose of 
         *  aligning uAT to AT2 BCN.
         *  Response: (uint32_t *) - UAT SYNC BCN Capture value
         */
        case IQN2FL_QUERY_AID2_UAT_SYNC_BCN_CAPTURE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_UAT_GEN_CTL.AID2_UAT_SYNC_BCN_CAPTURE_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_UAT_SYNC_BCN_CAPTURE_STS_RD_VAL);
            }
            break;

        /** AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS
         *  UAT SYNC RADT capture Register.
         *  UAT RADT sync capture captures the offset RADT count when a 
         *  uat_mst_slv_sync from the AT occurs. Used by SW to determine 
         *  correct RADT offset to apply.
         *  Argument: a pointer to an array of 8 uint32_t elements
         *  Response: (uint32_t *) - UAT EGR SYNC RADT Capture values
         *                         response[0] to response[7]
         */
        case IQN2FL_QUERY_AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 8; i++)
                {
                    tmpReg = hIqn2->regs->Aid2.AID2_UAT_EGR_RADT[i].AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS_RD_VAL);
                }
            }
            break;

        /** AID2_UAT_ING_SYNC_RADT_CAPTURE_STS
         *  UAT SYNC RADT capture Register.
         *  UAT RADT sync capture captures the offset RADT count when a 
         *  uat_mst_slv_sync from the AT occurs. Used by SW to determine 
         *  correct RADT offset to apply.
         *  Argument: a pointer to an array of 8 uint32_t elements
         *  Response: (uint32_t *) - UAT ING SYNC RADT Capture values
         *                         response[0] to response[7]
         */
        case IQN2FL_QUERY_AID2_UAT_ING_SYNC_RADT_CAPTURE_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 8; i++)
                {
                    tmpReg = hIqn2->regs->Aid2.AID2_UAT_ING_RADT[i].AID2_UAT_ING_SYNC_RADT_CAPTURE_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_UAT_ING_SYNC_RADT_CAPTURE_STS_RD_VAL);
                }
            }
            break;

        /** AID2_IQ_EDC_SOP_CNTR_STS
         *  Counts the number of SOPs seen by the IQ EDC.
         *  Response: (uint32_t *) - SOP count
         */
        case IQN2FL_QUERY_AID2_IQ_EDC_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_IQ_EDC_REGISTER_GROUP.AID2_IQ_EDC_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_EDC_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** AID2_IQ_EDC_EOP_CNTR_STS
         *  Counts the number of EOPs seen by the IQ EDC.
         *  Response: (uint32_t *) - EOP count
         */
        case IQN2FL_QUERY_AID2_IQ_EDC_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_IQ_EDC_REGISTER_GROUP.AID2_IQ_EDC_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_EDC_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** AID2_IQ_IDC_SOP_CNTR_STS
         *  This register provides a count of the Ingress SOPs sent on the PSI 
         *  to the IQN buffer or switch for activity monitoring.
         *  Response: (uint32_t *) - SOP count
         */
        case IQN2FL_QUERY_AID2_IQ_IDC_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_IQ_INGRESS_VBUS_MMR_GROUP.AID2_IQ_IDC_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_IDC_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** AID2_IQ_IDC_EOP_CNTR_STS
         *  This register provides a count of the Ingress EOPs sent on the PSI 
         *  to the IQN buffer or switch for activity monitoring.
         *  Response: (uint32_t *) - EOP count
         */
        case IQN2FL_QUERY_AID2_IQ_IDC_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_IQ_INGRESS_VBUS_MMR_GROUP.AID2_IQ_IDC_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_IQ_IDC_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** AID2_ECTL_SOP_CNTR_STS
         *  Counts the number of SOPs seen by the ECTL.
         *  Response: (uint32_t *) - SOP count
         */
        case IQN2FL_QUERY_AID2_ECTL_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_ECTL_REGISTER_GROUP.AID2_ECTL_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_ECTL_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** AID2_ECTL_EOP_CNTR_STS
         *  Counts the number of EOPs seen by the ECTL.
         *  Response: (uint32_t *) - EOP count
         */
        case IQN2FL_QUERY_AID2_ECTL_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_ECTL_REGISTER_GROUP.AID2_ECTL_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_ECTL_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** AID2_ICTL_SOP_CNTR_STS
         *  This register provides a count of the Ingress SOPs sent on the PSI 
         *  to the IQN buffer or switch for activity monitoring.
         *  Response: (uint32_t *) - Wrapping count of SOPs sent on PSI.
         */
        case IQN2FL_QUERY_AID2_ICTL_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_CTL_INGRESS_VBUS_MMR_GROUP.AID2_ICTL_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_ICTL_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** AID2_ICTL_EOP_CNTR_STS
         *  This register provides a count of the Ingress EOPs sent on the PSI 
         *  to the IQN buffer or switch for activity monitoring.
         *  Response: (uint32_t *) - Wrapping count of EOPs sent on PSI.
         */
        case IQN2FL_QUERY_AID2_ICTL_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Aid2.AID2_CTL_INGRESS_VBUS_MMR_GROUP.AID2_ICTL_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_ICTL_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** DIO2_IQ_EFE_CHAN_ON_STS
         *  Gives current On/Off Status of every available stream. One bit per 
         *  channel. Required because channels only turn on/off on radio frame 
         *  so the chan_en alone does not give channel status. Chan on/off is 
         *  not tracked for packet channels; These bits are 0 for packet 
         *  channels.
         *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
         */
        case IQN2FL_QUERY_DIO2_IQ_EFE_CHAN_ON_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_CHAN_ON_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_EFE_CHAN_ON_STS_CHAN_ON);
            }
            break;

        /** DIO2_IQ_EFE_IN_PKT_STS
         *  Gives current In/Out packet state of packet channels only. Bits 
         *  are always zero for AxC channels. Bit is activated at SOP, holds 
         *  high mid packet, deactivates at EOP.
         *  Response: (uint32_t *) - 0x1:IN_PKT, 0x0:OUT_PKT
         */
        case IQN2FL_QUERY_DIO2_IQ_EFE_IN_PKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_IN_PKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_EFE_IN_PKT_STS_IN_PKT);
            }
            break;

        /** DIO2_IQ_EFE_DMA_SYNC_STS
         *  Gives current In/Out packet state of packet channels only. Bits are 
         *  always zero for AxC channels. Bit is activated at SOP, holds high 
         *  mid packet, deactivates at EOP.
         *  Response: (uint32_t *) - 0x1: DMA synchronized to radio timing for 
         *                              this channel 
         *                         0x0: DMA not synchronized to radio timing 
         *                              for this channel. Channel is in
         *                              re-sync mode.
         */
        case IQN2FL_QUERY_DIO2_IQ_EFE_DMA_SYNC_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_DMA_SYNC_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_EFE_DMA_SYNC_STS_DMA_SYNC);
            }
            break;

        /** DIO2_IQ_IFE_CHAN_ON_STS
         *  Gives current On/Off Status of every available stream. One bit per 
         *  channel. Required because channels only turn on/off on radio frame 
         *  so the chan_en alone does not give channel status. Chan on/off is 
         *  not tracked for packet channels; These bits are 0 for packet 
         *  channels.
         *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
         */
        case IQN2FL_QUERY_DIO2_IQ_IFE_CHAN_ON_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_IQ_IFE_CONFIG_GROUP.DIO2_IQ_IFE_CHAN_ON_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_IFE_CHAN_ON_STS_CHAN_ON);
            }
            break;

        /** DIO2_IQ_IFE_IN_PKT_STS
         *  Gives current In/Out packet state of packet channels only. Bits 
         *  are always zero for AxC channels. Bit is activated at SOP, holds 
         *  high mid packet, deactivates at EOP.
         *  Response: (uint32_t *) - 0x1:IN_PKT, 0x0:OUT_PKT
         */
        case IQN2FL_QUERY_DIO2_IQ_IFE_IN_PKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_IQ_IFE_CONFIG_GROUP.DIO2_IQ_IFE_IN_PKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_IFE_IN_PKT_STS_IN_PKT);
            }
            break;

        /** DIO2_IQ_IDC_STS
         *  IDC Status register.
         *  Empty indicator for both the PSI Staging FIFO and the CDC FIFO
         *     FIFO_NOT_EMPTY (00) = PSI Staging FIFO and CDC FIFO are not both empty
         *     FIFO_EMPTY (11) = PSI Staging FIFO and CDC FIFO are both empty
         *  Response: (uint32_t *) - Empty indicator as mentioned above
         */
        case IQN2FL_QUERY_DIO2_IQ_IDC_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_IQ_IDC_GENERAL_STATUS_GROUP.DIO2_IQ_IDC_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_IDC_STS_EMPTY);
            }
            break;

        /** DIO2_IQ_IDC_INPKT_STS
         *  Per-channel in packet status bits where a 0 indicates that the 
         *  channel is not actively processing a packet and a 1 indicates 
         *  that it is actively processing a packet. The inpkt to channel 
         *  assignment is such that inpkt[0] is associated with channel 0 
         *  and inpkt[15] is associated with channel 15.
         *  Response: (uint32_t *) - In-packet status
         */
        case IQN2FL_QUERY_DIO2_IQ_IDC_INPKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_IQ_IDC_GENERAL_STATUS_GROUP.DIO2_IQ_IDC_INPKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_IDC_INPKT_STS_INPKT);
            }
            break;

        /** DIO2_UAT_SYNC_BCN_CAPTURE_STS
         *  UAT SYNC BCN capture Register.
         *  uAT raw BCN value captured each frame boundary of AT2 master BCN. 
         *  Used to calculate uAT BCN offset value for the purpose of 
         *  aligning uAT to AT2 BCN.
         *  Response: (uint32_t *) - UAT SYNC BCN Capture value
         */
        case IQN2FL_QUERY_DIO2_UAT_SYNC_BCN_CAPTURE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_UAT_GEN_CTL.DIO2_UAT_SYNC_BCN_CAPTURE_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_UAT_SYNC_BCN_CAPTURE_STS_RD_VAL);
            }
            break;

        /** DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS
         *  UAT SYNC RADT capture Register.
         *  UAT RADT sync capture captures the offset RADT count when a 
         *  uat_mst_slv_sync from the AT occurs. Used by SW to determine 
         *  correct RADT offset to apply.
         *  Argument: a pointer to an array of 8 uint32_t elements
         *  Response: (uint32_t *) - UAT EGR SYNC RADT Capture values
         *                         response[0] to response[7]
         */
        case IQN2FL_QUERY_DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 8; i++)
                {
                    tmpReg = hIqn2->regs->Dio2.DIO2_UAT_EGR_RADT[i].DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS_RD_VAL);
                }
            }
            break;

        /** DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS
         *  UAT SYNC RADT capture Register.
         *  UAT RADT sync capture captures the offset RADT count when a 
         *  uat_mst_slv_sync from the AT occurs. Used by SW to determine 
         *  correct RADT offset to apply.
         *  Argument: a pointer to an array of 8 uint32_t elements
         *  Response: (uint32_t *) - UAT ING SYNC RADT Capture values
         *                         response[0] to response[7]
         */
        case IQN2FL_QUERY_DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 8; i++)
                {
                    tmpReg = hIqn2->regs->Dio2.DIO2_UAT_ING_RADT[i].DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS_RD_VAL);
                }
            }
            break;

        /** DIO2_UAT_DIO_EGR_SYNC_RADT_CAPTURE_STS
         *  UAT SYNC DIO egress RADT capture Register. 
         *  RADT timer value is sampled and held in this Register each time 
         *  the AT2 Master BCN wraps. Used for calculating timing correction 
         *  offsetRADT count when a uat_mst_slv_sync from the AT occurs. 
         *  Used by SW to determine correct RADT offset to apply.
         *  Argument: a pointer to an array of 3 uint32_t elements
         *  Response: (uint32_t *) - UAT Sync DIO Egress RADT sync capture.
         *                         response[0] to response[2]
         */
        case IQN2FL_QUERY_DIO2_UAT_DIO_EGR_SYNC_RADT_CAPTURE_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 3; i++)
                {
                    tmpReg = hIqn2->regs->Dio2.DIO2_UAT_DIO_EGR_RADT[i].DIO2_UAT_DIO_EGR_SYNC_RADT_CAPTURE_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_UAT_DIO_EGR_SYNC_RADT_CAPTURE_STS_RD_VAL);
                }
            }
            break;

        /** DIO2_UAT_DIO_ING_SYNC_RADT_CAPTURE_STS
         *  UAT SYNC DIO ingress RADT capture Register. 
         *  RADT timer value is sampled and held in this Register each time 
         *  the AT2 Master BCN wraps. Used for calculating timing correction 
         *  offsetRADT count when a uat_mst_slv_sync from the AT occurs. 
         *  Used by SW to determine correct RADT offset to apply.
         *  Argument: a pointer to an array of 3 uint32_t elements
         *  Response: (uint32_t *) - UAT Sync DIO Ingress RADT sync capture.
         *                         response[0] to response[2]
         */
        case IQN2FL_QUERY_DIO2_UAT_DIO_ING_SYNC_RADT_CAPTURE_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 3; i++)
                {
                    tmpReg = hIqn2->regs->Dio2.DIO2_UAT_DIO_ING_RADT[i].DIO2_UAT_DIO_ING_SYNC_RADT_CAPTURE_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_UAT_DIO_ING_SYNC_RADT_CAPTURE_STS_RD_VAL);
                }
            }
            break;

        /** DIO2_IQ_EDC_SOP_CNTR_STS
         *  Counts the number of SOPs seen by the IQ EDC.
         *  Response: (uint32_t *) - SOP count
         */
        case IQN2FL_QUERY_DIO2_IQ_EDC_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_IQ_EDC_REGISTER_GROUP.DIO2_IQ_EDC_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_EDC_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** DIO2_IQ_EDC_EOP_CNTR_STS
         *  Counts the number of EOPs seen by the IQ EDC.
         *  Response: (uint32_t *) - EOP count
         */
        case IQN2FL_QUERY_DIO2_IQ_EDC_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_IQ_EDC_REGISTER_GROUP.DIO2_IQ_EDC_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_EDC_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** DIO2_IQ_IDC_SOP_CNTR_STS
         *  This register provides a count of the Ingress SOPs sent on the PSI 
         *  to the IQN buffer or switch for activity monitoring.
         *  Response: (uint32_t *) - SOP count
         */
        case IQN2FL_QUERY_DIO2_IQ_IDC_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_IQ_INGRESS_VBUS_MMR_GROUP.DIO2_IQ_IDC_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_IDC_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** DIO2_IQ_IDC_EOP_CNTR_STS
         *  This register provides a count of the Ingress EOPs sent on the PSI 
         *  to the IQN buffer or switch for activity monitoring.
         *  Response: (uint32_t *) - EOP count
         */
        case IQN2FL_QUERY_DIO2_IQ_IDC_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Dio2.DIO2_IQ_INGRESS_VBUS_MMR_GROUP.DIO2_IQ_IDC_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_IQ_IDC_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** AIL_IQ_EFE_CHAN_ON_STS
         *  This register gives current On/Off Status of every available stream. One bit per channel. Required
         *  because channels only turn on/off on radio frame so the chan_en alone does not give
         *  channel status. Chan on/off is not tracked for packet channels; These bits are 0 for
         *  packet channels.
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - 0x1: CHAN_ON 0x0:CHAN_OFF.
         *                         response[0] to response[1]
         */
        case IQN2FL_QUERY_AIL_IQ_EFE_CHAN_ON_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 2; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_CHAN_ON_STS[i];
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_EFE_CHAN_ON_STS_CHAN_ON);
                }
            }
            break;

        /** AIL_IQ_EFE_IN_PKT_STS
         *  Gives current In/Out packet state of packet channels only. Bits are always zero for AxC
         *  channels. Bit is activated at SOP, holds high mid packet, deactivates at EOP.
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - 0x1: IN_PKT 0x0:OUT_PKT.
         *                         response[0] to response[1]
         */
        case IQN2FL_QUERY_AIL_IQ_EFE_IN_PKT_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 2; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_IN_PKT_STS[i];
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_EFE_IN_PKT_STS_IN_PKT);
                }
            }
            break;

        /** AIL_IQ_EFE_DMA_SYNC_STS
         *  Gives current In/Out packet state of packet channels only. Bits are always zero for AxC
         *  channels. Bit is activated at SOP, holds high mid packet, deactivates at EOP
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - 0x1: DMA synchronized to radio timing for this channel 0x0:DMA not synchronized to
         *  radio timing for this channel. Channel is in in re-sync mode.
         *                         response[0] to response[1]
         */
        case IQN2FL_QUERY_AIL_IQ_EFE_DMA_SYNC_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 2; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_DMA_SYNC_STS[i];
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_EFE_DMA_SYNC_STS_DMA_SYNC);
                }
            }
            break;

        /** AIL_IQ_PE_PHY_STS
         *  This register is PE DMA Channel 1 Register.
         *  SI Egress AIL scheduler, active high, indicating that the PE PHY is in ON state. PE PHY will
         *  only turn ON/OFF on PE_FB boundaries.
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_IQ_PE_PHY_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_E_SCH_PHY.AIL_IQ_PE_PHY_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_PE_PHY_STS_PHY_ON);
            }
            break;

        /** AIL_IQ_PE_CPRI_RADSTD_STS
         *  Gives each radio status reflecting enable and satisfying basic frame offset
         *  Argument: a pointer to an array of 8 uint32_t elements
         *  Response: (uint32_t *) - 0x1: RADSTD_ON 0x0:RADSTD_OFF.
         *                         response[0] to response[7]
         */
        case IQN2FL_QUERY_AIL_IQ_PE_CPRI_RADSTD_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 8; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_RADSTD_STS[i];
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_PE_CPRI_RADSTD_STS_ON);
                }
            }
            break;

        /** AIL_IQ_IFE_CHAN_ON_STS
         *  This register gives Gives current On/Off Status of every available stream. One bit per channel. Required
         *  because channels only turn on/off on radio frame so the chan_en alone does not give
         *  channel status. Chan on/off is not tracked for packet channels; These bits are 0 for packet channels.
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - 0x1: CHAN_ON 0x0:CHAN_OFF.
         *                         response[0] to response[1]
         */
        case IQN2FL_QUERY_AIL_IQ_IFE_CHAN_ON_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 2; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_IFE_CONFIG_GROUP.AIL_IQ_IFE_CHAN_ON_STS[i];
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_IFE_CHAN_ON_STS_CHAN_ON);
                }
            }
            break;

        /** AIL_IQ_IFE_IN_PKT_STS
         *  Gives current In/Out packet state of packet channels only. Bits are always zero for AxC
         *  channels. Bit is activated at SOP, holds high mid packet, deactivates at EOP.
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - 0x1: IN_PKT 0x0:OUT_PKT.
         *                         response[0] to response[1]
         */
        case IQN2FL_QUERY_AIL_IQ_IFE_IN_PKT_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 2; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_IFE_CONFIG_GROUP.AIL_IQ_IFE_IN_PKT_STS[i];
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_EFE_IN_PKT_STS_IN_PKT);
                }
            }
            break;

        /** AIL_IQ_IDC_INPKT_STS
         *  Indicates when a channel is activelyreceiving a packet from the IFE.
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - Per-channel in packet status bits where a 0 indicates that the channel is not actively
         *  processing a packet and a 1 indicates that it is actively processing a packet. The inpkt to
         *  channel assignment is such that inpkt[0] is associated with channel 0 and inpkt[15] is associated with channel 15
         *                         response[0] to response[1]
         */
        case IQN2FL_QUERY_AIL_IQ_IDC_INPKT_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 2; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_IDC_GENERAL_STATUS_GROUP.AIL_IQ_IDC_INPKT_STS[i];
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_IDC_INPKT_STS_INPKT);
                }
            }
            break;

        /** AIL_IQ_IDC_STS
         *  This register is IDC Status register.
         *  Empty indicator for both the PSI Staging FIFO and the CDC FIFO
         *  FIFO_NOT_EMPTY (00) = PSI Staging FIFO and CDC FIFO are not both empty
         *  FIFO_EMPTY (11) = PSI Staging FIFO and CDC FIFO are both empty
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_IQ_IDC_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_IDC_GENERAL_STATUS_GROUP.AIL_IQ_IDC_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_IDC_STS_EMPTY);
            }
            break;

        /** AIL_ECTL_CHAN_ON_STS
         *  Gives current On/Off Status of every available stream. One bit per channel. Required
         *  because channels only turn on/off on radio frame so the chan_en alone does not give
         *  channel status. Chan on/off is not tracked for packet channels; These bits are 0 for
         *  packet channels
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_ECTL_CHAN_ON_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ECTL_PKT_IF.AIL_ECTL_CHAN_ON_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_ECTL_CHAN_ON_STS_CHAN_ON);
            }
            break;

        /** AIL_ECTL_INPKT_STS
         *  Indicates when a channel is actively receiving a packet from the ECTL
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_ECTL_INPKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ECTL_PKT_IF.AIL_ECTL_INPKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_ECTL_INPKT_STS_IN_PKT);
            }
            break;

        /** AIL_ICTL_CHAN_ON_STS
         *  Gives current On/Off Status of every available stream. One bit per channel. Required
         *  because channels only turn on/off on radio frame so the chan_en alone does not give
         *  channel status. Chan on/off is not tracked for packet channels; These bits are 0 for
         *  packet channels.
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_ICTL_CHAN_ON_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ICTL_PKT_IF.AIL_ICTL_CHAN_ON_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_ICTL_CHAN_ON_STS_CHAN_ON);
            }
            break;

        /** AIL_ICTL_INPKT_STS
         *  Indicates when a channel is activelyreceiving a packet from the ICTL.
         *  Per-channel in packet status bits where a 0 indicates that the channel is not actively
         *  processing a packet and a 1 indicates that it is actively processing a packet. The inpkt to
         *  channel assignment is such that inpkt[0] is associated with channel 0 and inpkt[15] is
         *  associated with channel 15
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_ICTL_INPKT_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ICTL_IDC_IF.AIL_ICTL_INPKT_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_ICTL_INPKT_STS_IN_PKT);
            }
            break;

        /** AIL_UAT_SYNC_BCN_CAPTURE_STS
         *  This register is UAT SYNC BCN capture Register
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_UAT_SYNC_BCN_CAPTURE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_GEN_CTL.AIL_UAT_SYNC_BCN_CAPTURE_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_UAT_SYNC_BCN_CAPTURE_STS_RD_VAL);
            }
            break;

        /** AIL_UAT_PI_BCN_CAPTURE_STS
         *  This register is UAT pi BCN capture Register
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - Value of counters
         *            response[0] = value of  uAT pi BCN capture. Captures the BCN timer value when the AIL_PHY_RM
         *            detects a CPRI or OBSAI PHY frame boundary (SOF).
         *            response[1] = value of  uAT pi K character position. Captures the k character position when the
         *            AIL_PHY_RM detects a CPRI or OBSAI PHY frame boundary (SOF). 0 means that the kchar
         *            for SOF was detected on the lower byte of the SERDES receive data, 1 means upper byte.
         */
        case IQN2FL_QUERY_AIL_UAT_PI_BCN_CAPTURE_STS:
            {
                uint32_t tmpReg, *respVal = (uint32_t *)response;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_AIL_REGS.AIL_UAT_PI_BCN_CAPTURE_STS;
                respVal[0] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_UAT_PI_BCN_CAPTURE_STS_RD_VAL);
                respVal[1] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_UAT_PI_BCN_CAPTURE_STS_KCHAR_POSITION);
            }
            break;

        /** AIL_UAT_RP301_BCN_CAPTURE_STS
         *  This register is UAT rp301 BCN capture Register
         *  Response: (uint32_t *) - uAT RP3-01 BCN capture. Captures the BCN offset count when the phy
         *  detects an OBSAI RP3-01 strobe from the AIL_PD.
         */
        case IQN2FL_QUERY_AIL_UAT_RP301_BCN_CAPTURE_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_AIL_REGS.AIL_UAT_RP301_BCN_CAPTURE_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_UAT_RP301_BCN_CAPTURE_STS_RD_VAL);
            }
            break;

        /** AIL_UAT_EGR_SYNC_RADT_CAPTURE_STS
         *  This is UAT SYNC RADT capture Register. UAT RADT sync capture captures the offset RADT count when a uat_mst_slv_sync from
         *  the AT occurs. Used by SW to determine correct RADT offset to apply.
         *  Argument: a pointer to an array of 8 uint32_t elements
         *  Response: (uint32_t *) - offset RADT count.
         *                         response[0] to response[7]
         */
        case IQN2FL_QUERY_AIL_UAT_EGR_SYNC_RADT_CAPTURE_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 8; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_EGR_RADT[i].AIL_UAT_EGR_SYNC_RADT_CAPTURE_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_UAT_EGR_SYNC_RADT_CAPTURE_STS_RD_VAL);
                }
            }
            break;

        /** AIL_UAT_ING_SYNC_RADT_CAPTURE_STS
         *  This is UAT SYNC RADT capture Register. UAT RADT sync capture captures the offset RADT count when a uat_mst_slv_sync from
         *  the AT occurs. Used by SW to determine correct RADT offset to apply.
         *  Argument: a pointer to an array of 8 uint32_t elements
         *  Response: (uint32_t *) - offset RADT count.
         *                         response[0] to response[7]
         */
        case IQN2FL_QUERY_AIL_UAT_ING_SYNC_RADT_CAPTURE_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 8; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_ING_RADT[i].AIL_UAT_ING_SYNC_RADT_CAPTURE_STS;
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_UAT_ING_SYNC_RADT_CAPTURE_STS_RD_VAL);
                }
            }
            break;

        /** AIL_IQ_EDC_SOP_CNTR_STS
         *  Counts the number of SOPs seen by the IQ EDC
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_IQ_EDC_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_EDC_REGISTER_GROUP.AIL_IQ_EDC_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_EDC_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** AIL_IQ_EDC_EOP_CNTR_STS
         *  Counts the number of EOPs seen by the IQ EDC
         *  Response: (uint32_t *)
         */
        case IQN2FL_QUERY_AIL_IQ_EDC_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_EDC_REGISTER_GROUP.AIL_IQ_EDC_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_EDC_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** AIL_IQ_IDC_SOP_CNTR_STS
         *  This register provides a count of the Ingress SOPs sent on the PSI to the IQN buffer or
         *  switch for activity monitoring.
         *  Response: (uint32_t *) - Wrapping count of SOPs sent on PSI.
         */
        case IQN2FL_QUERY_AIL_IQ_IDC_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_INGRESS_VBUS_MMR_GROUP.AIL_IQ_IDC_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_IDC_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** AIL_IQ_IDC_EOP_CNTR_STS
         *  This register provides a count of the Ingress EOPs sent on the PSI to the IQN buffer or
         *  switch for activity monitoring.
         *  Response: (uint32_t *) - Wrapping count of EOPs sent on PSI.
         */
        case IQN2FL_QUERY_AIL_IQ_IDC_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_INGRESS_VBUS_MMR_GROUP.AIL_IQ_IDC_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_IQ_IDC_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** AIL_ICTL_SOP_CNTR_STS
         *  This register provides a count of the Ingress SOPs sent on the PSI to the IQN buffer or
         *  switch for activity monitoring.
         *  Response: (uint32_t *) - Wrapping count of SOPs sent on PSI.
         */
        case IQN2FL_QUERY_AIL_ICTL_SOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_CTL_INGRESS_VBUS_MMR_GROUP.AIL_ICTL_SOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_ICTL_SOP_CNTR_STS_SOP_CNT);
            }
            break;

        /** AIL_ICTL_EOP_CNTR_STS
         *  This register provides a count of the Ingress EOPs sent on the PSI to the IQN buffer or
         *  switch for activity monitoring.
         *  Response: (uint32_t *) - Wrapping count of EOPs sent on PSI.
         */
        case IQN2FL_QUERY_AIL_ICTL_EOP_CNTR_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_CTL_INGRESS_VBUS_MMR_GROUP.AIL_ICTL_EOP_CNTR_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_ICTL_EOP_CNTR_STS_EOP_CNT);
            }
            break;

        /** AIL_PE_CPRI_STS
         *  chan-by-chan packet status. When shutting down a link, it is good practice for APP SW
         *  to wait until all channels are out of packet
         *  Response: (uint32_t *) - Each bit indicates if the corresponding channel is active or not. A 1 indicates the channel is
         *  currently in the middle of a packet. A 0 indicatest the channel is out of packet.
         */
        case IQN2FL_QUERY_AIL_PE_CPRI_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PE_CPRI_CW.AIL_PE_CPRI_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_CPRI_STS_IN_PKT);
            }
            break;

        /** AIL_PD_OBSAI_RP3_01_STS
         *  RP3_01 FCB capture from received OBSAI msg. Capture is OBSAI Type triggered,
         *  Type LUT has capture control. Whole OBSAI qwd payload is captured to 4 wd MMRs
         *  Argument: a pointer to an array of 4 uint32_t elements
         *  Response: (uint32_t *) - one of four payload wd.
         *                         response[0] to response[3]
         */
        case IQN2FL_QUERY_AIL_PD_OBSAI_RP3_01_STS:
            {
                uint32_t tmpReg, i, *respVal = (uint32_t *)response;
                for (i = 0; i < 4; i++)
                {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PD_OBSAI_CFG.AIL_PD_OBSAI_RP3_01_STS[i];
                    respVal[i] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_OBSAI_RP3_01_STS_CAPTURE);
                }
            }
            break;

        /** AIL_PHY_RT_DP_STS
         *  The AIL PHY RT Depth Status Register displaysthe real time depth of the Link Buffer
         *  in the RT Block
         *  Response: (uint32_t *) - The RT Depth status displays the operating depth of the RT link buffer.
         */
        case IQN2FL_QUERY_AIL_PHY_RT_DP_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_RT.AIL_PHY_RT_DP_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PHY_RT_DP_STS_VALUE);
            }
            break;

        /** AIL_PHY_TM_STS
         *  The TM Status Register contains status of the TM block.
         *  Indicates that status of the Frame Sync state machine
         *  OFF (11) = FSM in OFF state
         *  IDLE (22) = FSM in IDLE state
         *  RE_SYNC (44) = FSM in RE_SYNC state
         *  FRAME_SYNC (88) = FSM in FRAME_SYNC state
         *  Response: (uint32_t *) - TM status.
         */
        case IQN2FL_QUERY_AIL_PHY_TM_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_TM.AIL_PHY_TM_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PHY_TM_STS_FRM_STATE);
            }
            break;

        /** AIL_PHY_RM_STS
         *  The RM Status Register contains status of the RM block.
         *  ST0 (88) = ST0 State UNSYNC
         *  ST1 (44) = ST1 State WAIT_FOR_K28p7_IDLES
         *  ST2 (22) = ST2 State WAIT_FOR_FRAME_SYNC_T
         *  ST3 (11) = ST3 State FRAME_SYNC
         *  ST4 (1616) = ST4 State WAIT_FOR_SEED
         *  ST5 (3232) = ST5 State WAIT_FOR_ACK
         *  Response: (uint32_t *) - RM status.
         */
        case IQN2FL_QUERY_AIL_PHY_RM_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_RM.AIL_PHY_RM_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PHY_RM_STS_RX_SYNC);
            }
            break;

        /** AIL_PHY_RM_CPRI_HFN_STS
         *  Contains the last received CPRI HFN value.
         *  Response: (uint32_t *) - received CPRI HFN value.
         */
        case IQN2FL_QUERY_AIL_PHY_RM_CPRI_HFN_STS:
            {
                uint32_t tmpReg;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_RM.AIL_PHY_RM_CPRI_HFN_STS;
                *((uint32_t *)response) = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PHY_RM_CPRI_HFN_STS_HFN);
            }
            break;

        /** AIL_PHY_RM_CPRI_BFN_STS
         *  Contains the last received CPRI BFN value, high and low bytes
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - response[0]: Received Node B Frame number low byte, Z.128.0
         *                         response[1]: Received Node B Frame number high byte, Z.130.0
         */
        case IQN2FL_QUERY_AIL_PHY_RM_CPRI_BFN_STS:
            {
                uint32_t tmpReg, *respVal = (uint32_t *)response;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_RM.AIL_PHY_RM_CPRI_BFN_STS;
                respVal[0] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PHY_RM_CPRI_BFN_STS_LOW);
                respVal[1] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PHY_RM_CPRI_BFN_STS_HIGH);
            }
            break;

        /** AIL_PHY_RM_CPRI_STATE_STS
         *  Defines the RM CPRI State Status
         *  Argument: a pointer to an array of 2 uint32_t elements
         *  Response: (uint32_t *) - response[0]: Active high status indicates when the receiber FSM is in the HFSYNC state ST3
         *                         response[1]: Active high status indicates Loss Of Frame when the receiver FSM is in state ST0 or ST1.
         *                         NOTE: The value of this bit will be 0 after reset but will change to a value of 1 if CPRI mode is enabled
         */
        case IQN2FL_QUERY_AIL_PHY_RM_CPRI_STATE_STS:
            {
                uint32_t tmpReg, *respVal = (uint32_t *)response;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_RM.AIL_PHY_RM_CPRI_STATE_STS;
                respVal[0] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PHY_RM_CPRI_STATE_STS_HFSYNC);
                respVal[1] = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PHY_RM_CPRI_STATE_STS_LOF);
            }
            break;

        /** EE_EV0_ORGN_STS
         *  Reports the EE EV0 Origination Status Register into a Iqn2Fl_EeOrigin structure
         *  Argument: a pointer to Iqn2Fl_EeOrigin structure
         *  Response: (Iqn2Fl_EeOrigin *) - response
         */
        case IQN2FL_QUERY_EE_EV0_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_EeOrigin *eeOrigin = (Iqn2Fl_EeOrigin*)response;
                tmpReg = hIqn2->regs->Top.TOP_LEVEL_EE.EE_EV0_ORGN_STS;
                eeOrigin->at_ee_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV0_ORGN_STS_AT_EE_STS);
                eeOrigin->aid_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV0_ORGN_STS_AID_EE_STS);
                eeOrigin->dfe_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV0_ORGN_STS_DFE_EE_STS);
                eeOrigin->dio_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV0_ORGN_STS_DIO_EE_STS);
                eeOrigin->iqs_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV0_ORGN_STS_IQS_EE_STS);
                eeOrigin->ail0_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV0_ORGN_STS_AIL0_EE_STS);
                eeOrigin->ail1_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV0_ORGN_STS_AIL1_EE_STS);
                eeOrigin->psr_ee_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV0_ORGN_STS_PSR_EE_STS);
            }
            break;

        /** EE_EV1_ORGN_STS
         *  Reports the EE EV1 Origination Status Register into a Iqn2Fl_EeOrigin structure
         *  Argument: a pointer to Iqn2Fl_EeOrigin structure
         *  Response: (Iqn2Fl_EeOrigin *) - response
         */
        case IQN2FL_QUERY_EE_EV1_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_EeOrigin *eeOrigin = (Iqn2Fl_EeOrigin*)response;
                tmpReg = hIqn2->regs->Top.TOP_LEVEL_EE.EE_EV1_ORGN_STS;
                eeOrigin->at_ee_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV1_ORGN_STS_AT_EE_STS);
                eeOrigin->aid_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV1_ORGN_STS_AID_EE_STS);
                eeOrigin->dfe_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV1_ORGN_STS_DFE_EE_STS);
                eeOrigin->dio_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV1_ORGN_STS_DIO_EE_STS);
                eeOrigin->iqs_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV1_ORGN_STS_IQS_EE_STS);
                eeOrigin->ail0_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV1_ORGN_STS_AIL0_EE_STS);
                eeOrigin->ail1_ee_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV1_ORGN_STS_AIL1_EE_STS);
                eeOrigin->psr_ee_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EE_EV1_ORGN_STS_PSR_EE_STS);
            }
            break;

        /** PSR_EE_ING_FLUSH_A_STS
         *  Reports Channel flush indication for ingress channels 0 through 31. An ingress flush error indicates
         *  a transfer to this PKTDMA channel was attempted when the channel was full which triggers a flush of
         *  the rest of the packet
         *  Argument: a pointer to Iqn2Fl_PsrEeIngFlushA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_PsrEeIngFlushA *) - response
         */
        case IQN2FL_QUERY_PSR_EE_ING_FLUSH_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_PsrEeIngFlushA *eePsr = (Iqn2Fl_PsrEeIngFlushA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_RAW_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_ING_FLUSH_A_RAW_STS_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_EV0_ENABLED_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_ING_FLUSH_A_EV0_ENABLED_STS_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_EV1_ENABLED_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_ING_FLUSH_A_EV1_ENABLED_STS_ERR);
                }
            }
            break;

        /** PSR_EE_ING_FLUSH_B_STS
         *  Reports Channel flush indication for ingress channels 32 through 47. An ingress flush error indicates
         *  a transfer to this PKTDMA channel was attempted when the channel was full which triggers a flush of
         *  the rest of the packet
         *  Argument: a pointer to Iqn2Fl_PsrEeIngFlushB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_PsrEeIngFlushB *) - response
         */
        case IQN2FL_QUERY_PSR_EE_ING_FLUSH_B_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_PsrEeIngFlushB *eePsr = (Iqn2Fl_PsrEeIngFlushB*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_RAW_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_ING_FLUSH_B_RAW_STS_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_EV0_ENABLED_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_ING_FLUSH_B_EV0_ENABLED_STS_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_EV1_ENABLED_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_ING_FLUSH_B_EV1_ENABLED_STS_ERR);
                }
            }
            break;

        /** EGR_PROTOCOL_ERR_A_STS
         *  Reports Protocol error indication of an unsupported data type or a missing PS_DATA transfer
         *  when one was expected for egress channels 0 through 31
         *  Argument: a pointer to Iqn2Fl_PsrEeEgressProtocolErrA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_PsrEeEgressProtocolErrA *) - response
         */
        case IQN2FL_QUERY_PSR_EE_EGR_PROTOCOL_ERR_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_PsrEeEgressProtocolErrA *eePsr = (Iqn2Fl_PsrEeEgressProtocolErrA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_RAW_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EGR_PROTOCOL_ERR_A_RAW_STS_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_EV0_ENABLED_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EGR_PROTOCOL_ERR_A_EV0_ENABLED_STS_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_EV1_ENABLED_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EGR_PROTOCOL_ERR_A_EV1_ENABLED_STS_ERR);
                }
            }
            break;

        /** EGR_PROTOCOL_ERR_B_STS
         *  Reports Protocol error indication of an unsupported data type or a missing PS_DATA transfer
         *  when one was expected for egress channels 32 through 47
         *  Argument: a pointer to Iqn2Fl_PsrEeEgressProtocolErrB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_PsrEeEgressProtocolErrB *) - response
         */
        case IQN2FL_QUERY_PSR_EE_EGR_PROTOCOL_ERR_B_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_PsrEeEgressProtocolErrB *eePsr = (Iqn2Fl_PsrEeEgressProtocolErrB*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_RAW_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EGR_PROTOCOL_ERR_B_RAW_STS_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_EV0_ENABLED_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EGR_PROTOCOL_ERR_B_EV0_ENABLED_STS_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_EV1_ENABLED_STS;
                    eePsr->err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_EGR_PROTOCOL_ERR_B_EV1_ENABLED_STS_ERR);
                }
            }
            break;

        /** PSR_EE_ORIG_REG
         *  Reports the EE PSR Origination Status Register into a Iqn2Fl_PsrEeOrigin structure
         *  Argument: a pointer to Iqn2Fl_PsrEeOrigin structure
         *  Response: (Iqn2Fl_PsrEeOrigin *) - response
         */
        case IQN2FL_QUERY_PSR_EE_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_PsrEeOrigin *eeOrigin = (Iqn2Fl_PsrEeOrigin*)response;
                tmpReg = hIqn2->regs->Top.PSR_EE.PSR_ORIG_REG;
                eeOrigin->psr_ee_ing_flush_a_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_PSR_ORIG_REG_EE_ING_FLUSH_A);
                eeOrigin->psr_ee_ing_flush_b_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_PSR_ORIG_REG_EE_ING_FLUSH_B);
                eeOrigin->psr_ee_egr_protocol_err_a_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_PSR_ORIG_REG_EE_EGR_PROTOCOL_ERR_A);
                eeOrigin->psr_ee_egr_protocol_err_b_sts = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_PSR_ORIG_REG_EE_EGR_PROTOCOL_ERR_B);
            }
            break;

        /** PKTDMA_DESC_STARVE_STS
         *  Reports PKTDMA SOP and MOP descriptor starvation errors
         *  Argument: a pointer to Iqn2Fl_PktdmaEeDescStarve structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_PktdmaEeDescStarve *) - response
         */
        case IQN2FL_QUERY_PKTDMA_EE_PKTDMA_DESC_STARVE_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_PktdmaEeDescStarve *eePktdma = (Iqn2Fl_PktdmaEeDescStarve*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Top.PKTDMA_EE.PKTDMA_DESC_STARVE_RAW_STS;
                    eePktdma->sop_err = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_PKTDMA_DESC_STARVE_RAW_STS_SOP_ERR);
                    eePktdma->mop_err = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_PKTDMA_DESC_STARVE_RAW_STS_MOP_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Top.PKTDMA_EE.PKTDMA_DESC_STARVE_EV0_ENABLED_STS;
                    eePktdma->sop_err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_PKTDMA_DESC_STARVE_EV0_ENABLED_STS_SOP_ERR);
                    eePktdma->mop_err  = (uint32_t) CSL_FEXT(tmpReg, IQN2_TOP_PKTDMA_DESC_STARVE_EV0_ENABLED_STS_MOP_ERR);
                }
            }
            break;

        /** AT2_AT_EE_0_STS
         *  Reports RP1 Errors and Synchronization input Info
         *  Argument: a pointer to Iqn2Fl_At2EeInfoErr structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_At2EeInfoErr *) - response
         */
        case IQN2FL_QUERY_AT2_AT_EE_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_At2EeInfoErr *eeAt2 = (Iqn2Fl_At2EeInfoErr*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_RAW_STS;
                    eeAt2->rp1_crc_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_RAW_STS_RP1_CRC_ERR);
                    eeAt2->rp1_bit_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_RAW_STS_RP1_BIT_ERR);
                    eeAt2->rp1_sync_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_RAW_STS_RP1_SYNC_INFO);
                    eeAt2->radsync_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_RAW_STS_RADSYNC_INFO);
                    eeAt2->physync_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_RAW_STS_PHYSYNC_INFO);
                    eeAt2->pa_tscomp_info = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_RAW_STS_PA_TSCOMP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_EV0_ENABLED_STS;
                    eeAt2->rp1_crc_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV0_ENABLED_STS_RP1_CRC_ERR);
                    eeAt2->rp1_bit_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV0_ENABLED_STS_RP1_BIT_ERR);
                    eeAt2->rp1_sync_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV0_ENABLED_STS_RP1_SYNC_INFO);
                    eeAt2->radsync_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV0_ENABLED_STS_RADSYNC_INFO);
                    eeAt2->physync_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV0_ENABLED_STS_PHYSYNC_INFO);
                    eeAt2->pa_tscomp_info = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV0_ENABLED_STS_PA_TSCOMP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_EV1_ENABLED_STS;
                    eeAt2->rp1_crc_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV1_ENABLED_STS_RP1_CRC_ERR);
                    eeAt2->rp1_bit_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV1_ENABLED_STS_RP1_BIT_ERR);
                    eeAt2->rp1_sync_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV1_ENABLED_STS_RP1_SYNC_INFO);
                    eeAt2->radsync_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV1_ENABLED_STS_RADSYNC_INFO);
                    eeAt2->physync_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV1_ENABLED_STS_PHYSYNC_INFO);
                    eeAt2->pa_tscomp_info = (uint32_t) CSL_FEXT(tmpReg, IQN_AT2_AT2_AT_EE_0_EV1_ENABLED_STS_PA_TSCOMP_INFO);
                }
            }
            break;

        /** IQS_EE_CHAN_ERR_STS
         *  Reports a transfer occurred for a non-existent destination port or channel.
         *  Argument: a pointer to Iqn2Fl_Iqs2EeChanErr structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Iqs2EeChanErr *) - response
         */
        case IQN2FL_QUERY_IQS_EE_CHAN_ERR_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Iqs2EeChanErr *eeIqs2 = (Iqn2Fl_Iqs2EeChanErr*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_RAW_STS;
                    eeIqs2->ing_pktdma_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_RAW_STS_ING_PKTDMA_ERR);
                    eeIqs2->ing_dio2_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_RAW_STS_ING_DIO2_ERR);
                    eeIqs2->egr_mux_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_RAW_STS_EGR_MUX_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_EV0_ENABLED_STS;
                    eeIqs2->ing_pktdma_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_EV0_ENABLED_STS_ING_PKTDMA_ERR);
                    eeIqs2->ing_dio2_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_EV0_ENABLED_STS_ING_DIO2_ERR);
                    eeIqs2->egr_mux_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_EV0_ENABLED_STS_EGR_MUX_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_EV1_ENABLED_STS;
                    eeIqs2->ing_pktdma_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_EV1_ENABLED_STS_ING_PKTDMA_ERR);
                    eeIqs2->ing_dio2_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_EV1_ENABLED_STS_ING_DIO2_ERR);
                    eeIqs2->egr_mux_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_CHAN_ERR_EV1_ENABLED_STS_EGR_MUX_ERR);
                }
            }
            break;

        /** IQS_EE_ING_FLUSH_ERR_STS
         *  Reports  the DIO detected the need to flush or the transfer was to an
         *  IQS DIO channel that was full.
         *  Argument: a pointer to Iqn2Fl_Iqs2EeIngFlush structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Iqs2EeIngFlush *) - response
         */
        case IQN2FL_QUERY_IQS_EE_ING_FLUSH_ERR_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Iqs2EeIngFlush *eeIqs2 = (Iqn2Fl_Iqs2EeIngFlush*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_RAW_STS;
                    eeIqs2->flush  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_ING_FLUSH_ERR_RAW_STS_FLUSH);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_EV0_ENABLED_STS;
                    eeIqs2->flush  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_ING_FLUSH_ERR_EV0_ENABLED_STS_FLUSH);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_EV1_ENABLED_STS;
                    eeIqs2->flush  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_ING_FLUSH_ERR_EV1_ENABLED_STS_FLUSH);
                }
            }
            break;

        /** IQS_EE_EGR_FLUSH_ERR_STS
         *  Reports  the DIO detected the need to flush or the transfer was to an
         *  IQS DIO channel that was full.
         *  Argument: a pointer to Iqn2Fl_Iqs2EeEgrFlush structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Iqs2EeEgrFlush *) - response
         */
        case IQN2FL_QUERY_IQS_EE_EGR_FLUSH_ERR_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Iqs2EeEgrFlush *eeIqs2 = (Iqn2Fl_Iqs2EeEgrFlush*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_RAW_STS;
                    eeIqs2->flush  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_RAW_STS_FLUSH);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_EV0_ENABLED_STS;
                    eeIqs2->flush  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_EV0_ENABLED_STS_FLUSH);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_EV1_ENABLED_STS;
                    eeIqs2->flush  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_EV1_ENABLED_STS_FLUSH);
                }
            }
            break;

        /** IQS2_EE_ORIG_REG
         *  Reports the EE IQS2 Origination Status Register into a Iqn2Fl_Iqs2EeOrigin structure
         *  Argument: a pointer to Iqn2Fl_Iqs2EeOrigin structure
         *  Response: (Iqn2Fl_Iqs2EeOrigin *) - response
         */
        case IQN2FL_QUERY_IQS2_EE_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Iqs2EeOrigin *eeOrigin = (Iqn2Fl_Iqs2EeOrigin*)response;
                tmpReg = hIqn2->regs->Iqs2.IQS_EE.IQS_ORIG_REG;
                eeOrigin->iqs2_ee_chan_err_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_ORIG_REG_EE_CHAN_ERR);
                eeOrigin->iqs2_ee_ing_flush_err_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_ORIG_REG_EE_ING_FLUSH_ERR);
                eeOrigin->iqs2_ee_egr_flush_err_sts = (uint32_t) CSL_FEXT(tmpReg, IQN_IQS2_IQS_ORIG_REG_EE_EGR_FLUSH_ERR);
            }
            break;

        /** AID2_EE_SII_A_STS
         *  Reports SI si_i IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSiiA *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SII_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSiiA *eeAid2 = (Iqn2Fl_Aid2EeSiiA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_RAW_STS;
                    eeAid2->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_RAW_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeAid2->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_RAW_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeAid2->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeAid2->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeAid2->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_DAT_INFO);
                    eeAid2->si_ing_iq_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_RAW_STS_SI_ING_IQ_FIFO_OVFL_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_EV0_ENABLED_STS;
                    eeAid2->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeAid2->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeAid2->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeAid2->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeAid2->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_DAT_INFO);
                    eeAid2->si_ing_iq_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_FIFO_OVFL_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_EV1_ENABLED_STS;
                    eeAid2->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeAid2->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeAid2->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeAid2->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeAid2->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_DAT_INFO);
                    eeAid2->si_ing_iq_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_FIFO_OVFL_ERR);
                }
            }
            break;

        /** AID2_EE_SII_B_STS
         *  Reports SI si_i CTL errors and info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSiiB *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SII_B_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSiiB *eeAid2 = (Iqn2Fl_Aid2EeSiiB*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_RAW_STS;
                    eeAid2->si_ing_ctl_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_RAW_STS_SI_ING_CTL_PKT_ERR);
                    eeAid2->si_ing_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_RAW_STS_SI_ING_CTL_ICC_EOP_INFO);
                    eeAid2->si_ing_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_RAW_STS_SI_ING_CTL_ICC_DAT_INFO);
                    eeAid2->si_ing_ctl_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_RAW_STS_SI_ING_CTL_FIFO_OVFL_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_EV0_ENABLED_STS;
                    eeAid2->si_ing_ctl_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_EV0_ENABLED_STS_SI_ING_CTL_PKT_ERR);
                    eeAid2->si_ing_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_EV0_ENABLED_STS_SI_ING_CTL_ICC_EOP_INFO);
                    eeAid2->si_ing_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_EV0_ENABLED_STS_SI_ING_CTL_ICC_DAT_INFO);
                    eeAid2->si_ing_ctl_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_EV0_ENABLED_STS_SI_ING_CTL_FIFO_OVFL_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_EV1_ENABLED_STS;
                    eeAid2->si_ing_ctl_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_EV1_ENABLED_STS_SI_ING_CTL_PKT_ERR);
                    eeAid2->si_ing_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_EV1_ENABLED_STS_SI_ING_CTL_ICC_EOP_INFO);
                    eeAid2->si_ing_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_EV1_ENABLED_STS_SI_ING_CTL_ICC_DAT_INFO);
                    eeAid2->si_ing_ctl_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_B_EV1_ENABLED_STS_SI_ING_CTL_FIFO_OVFL_ERR);
                }
            }
            break;

        /** AID2_EE_SII_C_STS
         *  Reports SI si_i IQ per-channel start of frame errors.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiC structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSiiC *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SII_C_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSiiC *eeAid2 = (Iqn2Fl_Aid2EeSiiC*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_RAW_STS;
                    eeAid2->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_C_RAW_STS_SI_ING_IQ_SOF_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_EV0_ENABLED_STS;
                    eeAid2->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_C_EV0_ENABLED_STS_SI_ING_IQ_SOF_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_EV1_ENABLED_STS;
                    eeAid2->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_C_EV1_ENABLED_STS_SI_ING_IQ_SOF_ERR);
                }
            }
            break;

        /** AID2_EE_SII_D_STS
         *  Reports SI si_i CTL per-channel SOP received from ICC info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSiiD *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SII_D_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSiiD *eeAid2 = (Iqn2Fl_Aid2EeSiiD*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_RAW_STS;
                    eeAid2->si_ing_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_D_RAW_STS_SI_ING_CTL_ICC_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_EV0_ENABLED_STS;
                    eeAid2->si_ing_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_D_EV0_ENABLED_STS_SI_ING_CTL_ICC_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_EV1_ENABLED_STS;
                    eeAid2->si_ing_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_D_EV1_ENABLED_STS_SI_ING_CTL_ICC_SOP_INFO);
                }
            }
            break;

        /** AID2_EE_SIE_A_STS
         *  Reports SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSieA *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SIE_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSieA *eeAid2 = (Iqn2Fl_Aid2EeSieA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_RAW_STS;
                    eeAid2->si_egr_iq_efe_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeAid2->si_egr_iq_efe_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeAid2->si_egr_iq_efe_sym_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeAid2->si_egr_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_RAW_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeAid2->si_egr_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_RAW_STS_SI_EGR_IQ_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_EV0_ENABLED_STS;
                    eeAid2->si_egr_iq_efe_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeAid2->si_egr_iq_efe_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeAid2->si_egr_iq_efe_sym_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeAid2->si_egr_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeAid2->si_egr_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_EV1_ENABLED_STS;
                    eeAid2->si_egr_iq_efe_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeAid2->si_egr_iq_efe_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeAid2->si_egr_iq_efe_sym_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeAid2->si_egr_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeAid2->si_egr_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_ICC_DAT_INFO);
                }
            }
            break;

        /** AID2_EE_SIE_B_STS
         *  Reports SI si_e CTL info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSieB *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SIE_B_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSieB *eeAid2 = (Iqn2Fl_Aid2EeSieB*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_RAW_STS;
                    eeAid2->si_egr_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_B_RAW_STS_SI_EGR_CTL_ICC_EOP_INFO);
                    eeAid2->si_egr_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_B_RAW_STS_SI_EGR_CTL_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_EV0_ENABLED_STS;
                    eeAid2->si_egr_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_B_EV0_ENABLED_STS_SI_EGR_CTL_ICC_EOP_INFO);
                    eeAid2->si_egr_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_B_EV0_ENABLED_STS_SI_EGR_CTL_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_EV1_ENABLED_STS;
                    eeAid2->si_egr_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_B_EV1_ENABLED_STS_SI_EGR_CTL_ICC_EOP_INFO);
                    eeAid2->si_egr_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_B_EV1_ENABLED_STS_SI_EGR_CTL_ICC_DAT_INFO);
                }
            }
            break;

        /** AID2_EE_SIE_C_STS
         *  Reports SI si_e CTL per-channel SOP transmitted to ICC.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieC structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSieC *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SIE_C_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSieC *eeAid2 = (Iqn2Fl_Aid2EeSieC*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_RAW_STS;
                    eeAid2->si_egr_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_C_RAW_STS_SI_EGR_CTL_ICC_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_EV0_ENABLED_STS;
                    eeAid2->si_egr_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_C_EV0_ENABLED_STS_SI_EGR_CTL_ICC_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_EV1_ENABLED_STS;
                    eeAid2->si_egr_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_C_EV1_ENABLED_STS_SI_EGR_CTL_ICC_SOP_INFO);
                }
            }
            break;

        /** AID2_SYSCLK_ORIG_REG
         *  Reports the EE AID2 Origination Status Register into a Iqn2Fl_Aid2SysClkEeOrigin structure
         *  Argument: a pointer to Iqn2Fl_Aid2EeOrigin structure
         *  Response: (Iqn2Fl_Aid2SysClkEeOrigin *) - response
         */
        case IQN2FL_QUERY_AID2_SYSCLK_EE_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2SysClkEeOrigin *eeOrigin = (Iqn2Fl_Aid2SysClkEeOrigin*)response;
                tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_SYSCLK_ORIG_REG;
                eeOrigin->aid2_ee_sii_a_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_SYSCLK_ORIG_REG_ORIG_EE_0);
                eeOrigin->aid2_ee_sii_b_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_SYSCLK_ORIG_REG_ORIG_EE_1);
                eeOrigin->aid2_ee_sii_c_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_SYSCLK_ORIG_REG_ORIG_EE_2);
                eeOrigin->aid2_ee_sii_d_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_SYSCLK_ORIG_REG_ORIG_EE_6);
                eeOrigin->aid2_ee_sie_a_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_SYSCLK_ORIG_REG_ORIG_EE_10);
                eeOrigin->aid2_ee_sie_b_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_SYSCLK_ORIG_REG_ORIG_EE_11);
                eeOrigin->aid2_ee_sie_c_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_SYSCLK_ORIG_REG_ORIG_EE_12);
            }
            break;

        /** AID2_EE_DFE_STS
         *  Reports DFE interrupts.
         *  Argument: a pointer to Iqn2Fl_Aid2EeDfe structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeDfe *) - response
         */
        case IQN2FL_QUERY_AID2_EE_DFE_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeDfe *eeAid2 = (Iqn2Fl_Aid2EeDfe*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_RAW_STS;
                    eeAid2->dfe_iqn_aid2_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_DFE_EE_A_RAW_STS_DFE_IQN_AID2_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_EV0_ENABLED_STS;
                    eeAid2->dfe_iqn_aid2_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_DFE_EE_A_EV0_ENABLED_STS_DFE_IQN_AID2_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_EV1_ENABLED_STS;
                    eeAid2->dfe_iqn_aid2_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_DFE_EE_A_EV1_ENABLED_STS_DFE_IQN_AID2_ERR);
                }
            }
            break;

        /** AID2_EE_SII_E_STS
         *  Reports SI si_i IQ info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSiiE *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SII_E_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSiiE *eeAid2 = (Iqn2Fl_Aid2EeSiiE*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_RAW_STS;
                    eeAid2->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_E_RAW_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeAid2->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_E_RAW_STS_SI_ING_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_EV0_ENABLED_STS;
                    eeAid2->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_E_EV0_ENABLED_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeAid2->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_E_EV0_ENABLED_STS_SI_ING_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_EV1_ENABLED_STS;
                    eeAid2->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_E_EV1_ENABLED_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeAid2->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_E_EV1_ENABLED_STS_SI_ING_IQ_PSI_DAT_INFO);
                }
            }
            break;

        /** AID2_EE_SII_F_STS
         *  Reports SI si_i CTL info..
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiF structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSiiF *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SII_F_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSiiF *eeAid2 = (Iqn2Fl_Aid2EeSiiF*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_RAW_STS;
                    eeAid2->si_ing_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_F_RAW_STS_SI_ING_CTL_PSI_EOP_INFO);
                    eeAid2->si_ing_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_F_RAW_STS_SI_ING_CTL_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_EV0_ENABLED_STS;
                    eeAid2->si_ing_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_F_EV0_ENABLED_STS_SI_ING_CTL_PSI_EOP_INFO);
                    eeAid2->si_ing_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_F_EV0_ENABLED_STS_SI_ING_CTL_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_EV1_ENABLED_STS;
                    eeAid2->si_ing_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_F_EV1_ENABLED_STS_SI_ING_CTL_PSI_EOP_INFO);
                    eeAid2->si_ing_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_F_EV1_ENABLED_STS_SI_ING_CTL_PSI_DAT_INFO);
                }
            }
            break;

        /** AID2_EE_SII_G_STS
         *  Reports SI si_i IQ per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiG structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSiiG *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SII_G_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSiiG *eeAid2 = (Iqn2Fl_Aid2EeSiiG*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_RAW_STS;
                    eeAid2->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_G_RAW_STS_SI_ING_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_EV0_ENABLED_STS;
                    eeAid2->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_G_EV0_ENABLED_STS_SI_ING_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_EV1_ENABLED_STS;
                    eeAid2->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_G_EV1_ENABLED_STS_SI_ING_IQ_PSI_SOP_INFO);
                }
            }
            break;

        /** AID2_EE_SII_H_STS
         *  Reports SI si_i CTL per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiH structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSiiH *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SII_H_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSiiH *eeAid2 = (Iqn2Fl_Aid2EeSiiH*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_RAW_STS;
                    eeAid2->si_ing_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_H_RAW_STS_SI_ING_CTL_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_EV0_ENABLED_STS;
                    eeAid2->si_ing_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_H_EV0_ENABLED_STS_SI_ING_CTL_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_EV1_ENABLED_STS;
                    eeAid2->si_ing_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SII_H_EV1_ENABLED_STS_SI_ING_CTL_PSI_SOP_INFO);
                }
            }
            break;

        /** AID2_EE_SIE_D_STS
         *  Reports SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSieD *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SIE_D_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSieD *eeAid2 = (Iqn2Fl_Aid2EeSieD*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_RAW_STS;
                    eeAid2->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeAid2->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeAid2->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_EV0_ENABLED_STS;
                    eeAid2->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeAid2->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeAid2->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_EV1_ENABLED_STS;
                    eeAid2->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeAid2->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeAid2->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_DAT_INFO);
                }
            }
            break;

        /** AID2_EE_SIE_E_STS
         *  Reports SI si_e CTL errors and info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSieE *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SIE_E_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSieE *eeAid2 = (Iqn2Fl_Aid2EeSieE*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_E_RAW_STS;
                    eeAid2->si_egr_ctl_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_RAW_STS_SI_EGR_CTL_PSI_DATA_TYPE_ERR);
                    eeAid2->si_egr_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_RAW_STS_SI_EGR_CTL_PSI_EOP_INFO);
                    eeAid2->si_egr_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_RAW_STS_SI_EGR_CTL_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_E_EV0_ENABLED_STS;
                    eeAid2->si_egr_ctl_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_EV0_ENABLED_STS_SI_EGR_CTL_PSI_DATA_TYPE_ERR);
                    eeAid2->si_egr_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_EV0_ENABLED_STS_SI_EGR_CTL_PSI_EOP_INFO);
                    eeAid2->si_egr_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_EV0_ENABLED_STS_SI_EGR_CTL_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_EV1_ENABLED_STS;
                    eeAid2->si_egr_ctl_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_EV1_ENABLED_STS_SI_EGR_CTL_PSI_DATA_TYPE_ERR);
                    eeAid2->si_egr_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_EV1_ENABLED_STS_SI_EGR_CTL_PSI_EOP_INFO);
                    eeAid2->si_egr_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_E_EV1_ENABLED_STS_SI_EGR_CTL_PSI_DAT_INFO);
                }
            }
            break;

        /** AID2_EE_SIE_F_STS
         *  Reports SI si_e IQ per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieF structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSieF *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SIE_F_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSieF *eeAid2 = (Iqn2Fl_Aid2EeSieF*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_RAW_STS;
                    eeAid2->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_F_RAW_STS_SI_EGR_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_EV0_ENABLED_STS;
                    eeAid2->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_F_EV0_ENABLED_STS_SI_EGR_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_EV1_ENABLED_STS;
                    eeAid2->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_F_EV1_ENABLED_STS_SI_EGR_IQ_PSI_SOP_INFO);
                }
            }
            break;

        /** AID2_EE_SIE_G_STS
         *  Reports SI si_e CTL per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieG structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Aid2EeSieG *) - response
         */
        case IQN2FL_QUERY_AID2_EE_SIE_G_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2EeSieG *eeAid2 = (Iqn2Fl_Aid2EeSieG*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_RAW_STS;
                    eeAid2->si_egr_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_G_RAW_STS_SI_EGR_CTL_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_EV0_ENABLED_STS;
                    eeAid2->si_egr_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_G_EV0_ENABLED_STS_SI_EGR_CTL_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_EV1_ENABLED_STS;
                    eeAid2->si_egr_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_EE_SIE_G_EV1_ENABLED_STS_SI_EGR_CTL_PSI_SOP_INFO);
                }
            }
            break;

        /** AID2_VBUSCLK_ORIG_REG
         *  Reports the EE AID2 VBUSCLK Origination Status Register into a Iqn2Fl_Aid2VbusClkEeOrigin structure
         *  Argument: a pointer to Iqn2Fl_Aid2VbusClkEeOrigin structure
         *  Response: (Iqn2Fl_Aid2VbusClkEeOrigin *) - response
         */
        case IQN2FL_QUERY_AID2_VBUSCLK_EE_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Aid2VbusClkEeOrigin *eeOrigin = (Iqn2Fl_Aid2VbusClkEeOrigin*)response;
                tmpReg = hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_VBUSCLK_ORIG_REG;
                eeOrigin->aid2_ee_sii_e_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_VBUSCLK_ORIG_REG_ORIG_EE_0);
                eeOrigin->aid2_ee_sii_f_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_VBUSCLK_ORIG_REG_ORIG_EE_1);
                eeOrigin->aid2_ee_sii_g_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_VBUSCLK_ORIG_REG_ORIG_EE_2);
                eeOrigin->aid2_ee_sii_h_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_VBUSCLK_ORIG_REG_ORIG_EE_6);
                eeOrigin->aid2_ee_sie_d_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_VBUSCLK_ORIG_REG_ORIG_EE_10);
                eeOrigin->aid2_ee_sie_e_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_VBUSCLK_ORIG_REG_ORIG_EE_11);
                eeOrigin->aid2_ee_sie_f_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_VBUSCLK_ORIG_REG_ORIG_EE_12);
                eeOrigin->aid2_ee_sie_g_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AID2_AID2_VBUSCLK_ORIG_REG_ORIG_EE_16);
            }
            break;

        /** AIL_EE_SII_A_STS
         *  Reports SI si_i IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_Aid2EeSiiA *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiA *eeAil = (Iqn2Fl_AilEeSiiA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_RAW_STS;
                    eeAil->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_RAW_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeAil->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_RAW_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeAil->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeAil->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeAil->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_DAT_INFO);
                    eeAil->si_ing_iq_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_RAW_STS_SI_ING_IQ_FIFO_OVFL_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_EV0_ENABLED_STS;
                    eeAil->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeAil->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeAil->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeAil->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeAil->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_DAT_INFO);
                    eeAil->si_ing_iq_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_FIFO_OVFL_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_EV1_ENABLED_STS;
                    eeAil->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeAil->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeAil->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeAil->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeAil->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_DAT_INFO);
                    eeAil->si_ing_iq_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_FIFO_OVFL_ERR);
                }
            }
            break;

        /** AIL_EE_SII_B_STS
         *  Reports SI si_i CTL errors and info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiB *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_B_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiB *eeAil = (Iqn2Fl_AilEeSiiB*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_RAW_STS;
                    eeAil->si_ing_ctl_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_RAW_STS_SI_ING_CTL_PKT_ERR);
                    eeAil->si_ing_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_RAW_STS_SI_ING_CTL_ICC_EOP_INFO);
                    eeAil->si_ing_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_RAW_STS_SI_ING_CTL_ICC_DAT_INFO);
                    eeAil->si_ing_ctl_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_RAW_STS_SI_ING_CTL_FIFO_OVFL_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_EV0_ENABLED_STS;
                    eeAil->si_ing_ctl_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_EV0_ENABLED_STS_SI_ING_CTL_PKT_ERR);
                    eeAil->si_ing_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_EV0_ENABLED_STS_SI_ING_CTL_ICC_EOP_INFO);
                    eeAil->si_ing_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_EV0_ENABLED_STS_SI_ING_CTL_ICC_DAT_INFO);
                    eeAil->si_ing_ctl_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_EV0_ENABLED_STS_SI_ING_CTL_FIFO_OVFL_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_EV1_ENABLED_STS;
                    eeAil->si_ing_ctl_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_EV1_ENABLED_STS_SI_ING_CTL_PKT_ERR);
                    eeAil->si_ing_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_EV1_ENABLED_STS_SI_ING_CTL_ICC_EOP_INFO);
                    eeAil->si_ing_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_EV1_ENABLED_STS_SI_ING_CTL_ICC_DAT_INFO);
                    eeAil->si_ing_ctl_fifo_ovfl_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_B_EV1_ENABLED_STS_SI_ING_CTL_FIFO_OVFL_ERR);
                }
            }
            break;

        /** AIL_EE_SII_C_0_STS
         *  Reports SI si_i IQ per-channel start of frame errors.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiC0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiC0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_C_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiC0 *eeAil = (Iqn2Fl_AilEeSiiC0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_RAW_STS;
                    eeAil->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_C_0_RAW_STS_SI_ING_IQ_SOF_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_EV0_ENABLED_STS;
                    eeAil->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_C_0_EV0_ENABLED_STS_SI_ING_IQ_SOF_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_EV1_ENABLED_STS;
                    eeAil->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_C_0_EV1_ENABLED_STS_SI_ING_IQ_SOF_ERR);
                }
            }
            break;

        /** AIL_EE_SII_C_1_STS
         *  Reports SI si_i IQ per-channel start of frame errors.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiC1 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiC1 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_C_1_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiC1 *eeAil = (Iqn2Fl_AilEeSiiC1*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_RAW_STS;
                    eeAil->si_ing_iq_sof_err_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_C_1_RAW_STS_SI_ING_IQ_SOF_ERR_64_32);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_EV0_ENABLED_STS;
                    eeAil->si_ing_iq_sof_err_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_C_1_EV0_ENABLED_STS_SI_ING_IQ_SOF_ERR_64_32);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_EV1_ENABLED_STS;
                    eeAil->si_ing_iq_sof_err_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_C_1_EV1_ENABLED_STS_SI_ING_IQ_SOF_ERR_64_32);
                }
            }
            break;

        /** AIL_EE_SII_D_STS
         *  Reports SI si_i CTL per-channel SOP received from ICC info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiD *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_D_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiD *eeAil = (Iqn2Fl_AilEeSiiD*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_RAW_STS;
                    eeAil->si_ing_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_D_RAW_STS_SI_ING_CTL_ICC_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_EV0_ENABLED_STS;
                    eeAil->si_ing_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_D_EV0_ENABLED_STS_SI_ING_CTL_ICC_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_EV1_ENABLED_STS;
                    eeAil->si_ing_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_D_EV1_ENABLED_STS_SI_ING_CTL_ICC_SOP_INFO);
                }
            }
            break;

        /** AIL_EE_SII_E_STS
         *  Reports SI si_i IQ info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiE *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_E_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiE *eeAil = (Iqn2Fl_AilEeSiiE*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_RAW_STS;
                    eeAil->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_E_RAW_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeAil->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_E_RAW_STS_SI_ING_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_EV0_ENABLED_STS;
                    eeAil->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_E_EV0_ENABLED_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeAil->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_E_EV0_ENABLED_STS_SI_ING_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_EV1_ENABLED_STS;
                    eeAil->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_E_EV1_ENABLED_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeAil->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_E_EV1_ENABLED_STS_SI_ING_IQ_PSI_DAT_INFO);
                }
            }
            break;

        /** AIL_EE_SII_F_STS
         *  Reports SI si_i CTL info..
         *  Argument: a pointer to Iqn2Fl_AilEeSiiF structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiF *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_F_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiF *eeAil = (Iqn2Fl_AilEeSiiF*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_RAW_STS;
                    eeAil->si_ing_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_F_RAW_STS_SI_ING_CTL_PSI_EOP_INFO);
                    eeAil->si_ing_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_F_RAW_STS_SI_ING_CTL_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_EV0_ENABLED_STS;
                    eeAil->si_ing_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_F_EV0_ENABLED_STS_SI_ING_CTL_PSI_EOP_INFO);
                    eeAil->si_ing_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_F_EV0_ENABLED_STS_SI_ING_CTL_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_EV1_ENABLED_STS;
                    eeAil->si_ing_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_F_EV1_ENABLED_STS_SI_ING_CTL_PSI_EOP_INFO);
                    eeAil->si_ing_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_F_EV1_ENABLED_STS_SI_ING_CTL_PSI_DAT_INFO);
                }
            }
            break;

        /** AIL_EE_SII_G_0_STS
         *  Reports SI si_i IQ per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiG0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiG0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_G_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiG0 *eeAil = (Iqn2Fl_AilEeSiiG0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_RAW_STS;
                    eeAil->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_G_0_RAW_STS_SI_ING_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_EV0_ENABLED_STS;
                    eeAil->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_G_0_EV0_ENABLED_STS_SI_ING_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_EV1_ENABLED_STS;
                    eeAil->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_G_0_EV1_ENABLED_STS_SI_ING_IQ_PSI_SOP_INFO);
                }
            }
            break;

        /** AIL_EE_SII_G_1_STS
         *  Reports SI si_i IQ per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiG1 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiG1 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_G_1_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiG1 *eeAil = (Iqn2Fl_AilEeSiiG1*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_RAW_STS;
                    eeAil->si_ing_iq_psi_sop_info_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_G_1_RAW_STS_SI_ING_IQ_PSI_SOP_INFO_64_32);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_EV0_ENABLED_STS;
                    eeAil->si_ing_iq_psi_sop_info_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_G_1_EV0_ENABLED_STS_SI_ING_IQ_PSI_SOP_INFO_64_32);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_EV1_ENABLED_STS;
                    eeAil->si_ing_iq_psi_sop_info_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_G_1_EV1_ENABLED_STS_SI_ING_IQ_PSI_SOP_INFO_64_32);
                }
            }
            break;

        /** AIL_EE_SII_H_STS
         *  Reports SI si_i CTL per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiH structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSiiH *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SII_H_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSiiH *eeAil = (Iqn2Fl_AilEeSiiH*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_RAW_STS;
                    eeAil->si_ing_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_H_RAW_STS_SI_ING_CTL_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_EV0_ENABLED_STS;
                    eeAil->si_ing_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_H_EV0_ENABLED_STS_SI_ING_CTL_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_EV1_ENABLED_STS;
                    eeAil->si_ing_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SII_H_EV1_ENABLED_STS_SI_ING_CTL_PSI_SOP_INFO);
                }
            }
            break;

        /** AIL_EE_SIE_A_STS
         *  Reports SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSieA *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SIE_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSieA *eeAil = (Iqn2Fl_AilEeSieA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_RAW_STS;
                    eeAil->si_egr_iq_efe_starve_err = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeAil->si_egr_iq_efe_pkt_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeAil->si_egr_iq_efe_sym_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeAil->si_egr_iq_icc_sof_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_RAW_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeAil->si_egr_iq_icc_dat_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_RAW_STS_SI_EGR_IQ_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_EV0_ENABLED_STS;
                    eeAil->si_egr_iq_efe_starve_err = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeAil->si_egr_iq_efe_pkt_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeAil->si_egr_iq_efe_sym_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeAil->si_egr_iq_icc_sof_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeAil->si_egr_iq_icc_dat_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_EV1_ENABLED_STS;
                    eeAil->si_egr_iq_efe_starve_err = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeAil->si_egr_iq_efe_pkt_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeAil->si_egr_iq_efe_sym_err    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeAil->si_egr_iq_icc_sof_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeAil->si_egr_iq_icc_dat_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_ICC_DAT_INFO);
                }
            }
            break;

        /** AIL_EE_SIE_B_STS
         *  Reports SI si_e CTL info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSieB *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SIE_B_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSieB *eeAil = (Iqn2Fl_AilEeSieB*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_RAW_STS;
                    eeAil->si_egr_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_B_RAW_STS_SI_EGR_CTL_ICC_EOP_INFO);
                    eeAil->si_egr_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_B_RAW_STS_SI_EGR_CTL_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_EV0_ENABLED_STS;
                    eeAil->si_egr_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_B_EV0_ENABLED_STS_SI_EGR_CTL_ICC_EOP_INFO);
                    eeAil->si_egr_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_B_EV0_ENABLED_STS_SI_EGR_CTL_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_EV1_ENABLED_STS;
                    eeAil->si_egr_ctl_icc_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_B_EV1_ENABLED_STS_SI_EGR_CTL_ICC_EOP_INFO);
                    eeAil->si_egr_ctl_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_B_EV1_ENABLED_STS_SI_EGR_CTL_ICC_DAT_INFO);
                }
            }
            break;

        /** AIL_EE_SIE_C_STS
         *  Reports SI si_e CTL per-channel SOP transmitted to ICC.
         *  Argument: a pointer to Iqn2Fl_AilEeSieC structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSieC *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SIE_C_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSieC *eeAil = (Iqn2Fl_AilEeSieC*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_RAW_STS;
                    eeAil->si_egr_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_C_RAW_STS_SI_EGR_CTL_ICC_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_EV0_ENABLED_STS;
                    eeAil->si_egr_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_C_EV0_ENABLED_STS_SI_EGR_CTL_ICC_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_EV1_ENABLED_STS;
                    eeAil->si_egr_ctl_icc_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_C_EV1_ENABLED_STS_SI_EGR_CTL_ICC_SOP_INFO);
                }
            }
            break;

        /** AIL_EE_SIE_D_STS
         *  Reports SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSieD *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SIE_D_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSieD *eeAil = (Iqn2Fl_AilEeSieD*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_RAW_STS;
                    eeAil->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeAil->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeAil->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_EV0_ENABLED_STS;
                    eeAil->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeAil->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeAil->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_EV1_ENABLED_STS;
                    eeAil->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeAil->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeAil->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_DAT_INFO);
                }
            }
            break;

        /** AIL_EE_SIE_E_STS
         *  Reports SI si_e CTL errors and info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSieE *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SIE_E_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSieE *eeAil = (Iqn2Fl_AilEeSieE*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_E_RAW_STS;
                    eeAil->si_egr_ctl_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_RAW_STS_SI_EGR_CTL_PSI_DATA_TYPE_ERR);
                    eeAil->si_egr_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_RAW_STS_SI_EGR_CTL_PSI_EOP_INFO);
                    eeAil->si_egr_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_RAW_STS_SI_EGR_CTL_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_E_EV0_ENABLED_STS;
                    eeAil->si_egr_ctl_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_EV0_ENABLED_STS_SI_EGR_CTL_PSI_DATA_TYPE_ERR);
                    eeAil->si_egr_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_EV0_ENABLED_STS_SI_EGR_CTL_PSI_EOP_INFO);
                    eeAil->si_egr_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_EV0_ENABLED_STS_SI_EGR_CTL_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_EV1_ENABLED_STS;
                    eeAil->si_egr_ctl_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_EV1_ENABLED_STS_SI_EGR_CTL_PSI_DATA_TYPE_ERR);
                    eeAil->si_egr_ctl_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_EV1_ENABLED_STS_SI_EGR_CTL_PSI_EOP_INFO);
                    eeAil->si_egr_ctl_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_E_EV1_ENABLED_STS_SI_EGR_CTL_PSI_DAT_INFO);
                }
            }
            break;

        /** AIL_EE_SIE_F_0_STS
         *  Reports SI si_e IQ per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieF0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSieF0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SIE_F_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSieF0 *eeAil = (Iqn2Fl_AilEeSieF0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_RAW_STS;
                    eeAil->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_F_0_RAW_STS_SI_EGR_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_EV0_ENABLED_STS;
                    eeAil->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_F_0_EV0_ENABLED_STS_SI_EGR_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_EV1_ENABLED_STS;
                    eeAil->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_F_0_EV1_ENABLED_STS_SI_EGR_IQ_PSI_SOP_INFO);
                }
            }
            break;

        /** AIL_EE_SIE_F_1_STS
         *  Reports SI si_e IQ per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieF1 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSieF1 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SIE_F_1_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSieF1 *eeAil = (Iqn2Fl_AilEeSieF1*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_RAW_STS;
                    eeAil->si_egr_iq_psi_sop_info_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_F_1_RAW_STS_SI_EGR_IQ_PSI_SOP_INFO_64_32);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_EV0_ENABLED_STS;
                    eeAil->si_egr_iq_psi_sop_info_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_F_1_EV0_ENABLED_STS_SI_EGR_IQ_PSI_SOP_INFO_64_32);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_EV1_ENABLED_STS;
                    eeAil->si_egr_iq_psi_sop_info_64_32  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_F_1_EV1_ENABLED_STS_SI_EGR_IQ_PSI_SOP_INFO_64_32);
                }
            }
            break;

        /** AIL_EE_SIE_G_STS
         *  Reports SI si_e CTL per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieG structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSieG *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SIE_G_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSieG *eeAil = (Iqn2Fl_AilEeSieG*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_RAW_STS;
                    eeAil->si_egr_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_G_RAW_STS_SI_EGR_CTL_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_EV0_ENABLED_STS;
                    eeAil->si_egr_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_G_EV0_ENABLED_STS_SI_EGR_CTL_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_EV1_ENABLED_STS;
                    eeAil->si_egr_ctl_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_EE_SIE_G_EV1_ENABLED_STS_SI_EGR_CTL_PSI_SOP_INFO);
                }
            }
            break;

        /** AIL_VBUSCLK_ORIG_REG
         *  Reports the EE AIL VBUSCLK Origination Status Register into a Iqn2Fl_AilVbusClkEeOrigin structure
         *  Argument: a pointer to Iqn2Fl_AilVbusClkEeOrigin structure
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilVbusClkEeOrigin *) - response
         */
        case IQN2FL_QUERY_AIL_VBUSCLK_EE_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilVbusClkEeOrigin *eeOrigin = (Iqn2Fl_AilVbusClkEeOrigin*)response;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_VBUSCLK_ORIG_REG;
                eeOrigin->ail_ee_sii_e_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_0);
                eeOrigin->ail_ee_sii_f_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_1);
                eeOrigin->ail_ee_sii_g_0_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_2);
                eeOrigin->ail_ee_sii_g_1_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_3);
                eeOrigin->ail_ee_sii_h_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_6);
                eeOrigin->ail_ee_sie_d_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_10);
                eeOrigin->ail_ee_sie_e_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_11);
                eeOrigin->ail_ee_sie_f_0_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_12);
                eeOrigin->ail_ee_sie_f_1_sts  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_13);
                eeOrigin->ail_ee_sie_g_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_VBUSCLK_ORIG_REG_ORIG_EE_16);
            }
            break;

        /** AIL_EE_RM_0_STS
         *  Reports AIL RM error info.
         *  Argument: a pointer to Iqn2Fl_AilEeRm0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeRm0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_RM_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeRm0 *eeAil = (Iqn2Fl_AilEeRm0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_RAW_STS;
                    eeAil->sync_status_change   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_SYNC_STATUS_CHANGE);
                    eeAil->rm_status_state0     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RM_STATUS_STATE0);
                    eeAil->rm_status_state1     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RM_STATUS_STATE1);
                    eeAil->rm_status_state2     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RM_STATUS_STATE2);
                    eeAil->rm_status_state3     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RM_STATUS_STATE3);
                    eeAil->rm_status_state4     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RM_STATUS_STATE4);
                    eeAil->rm_status_state5     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RM_STATUS_STATE5);
                    eeAil->num_los_det          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_NUM_LOS_DET);
                    eeAil->lcv_det              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_LCV_DET);
                    eeAil->frame_bndry_det      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_FRAME_BNDRY_DET);
                    eeAil->block_bndry_det      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_BLOCK_BNDRY_DET);
                    eeAil->missing_k28p5        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_MISSING_K28P5);
                    eeAil->missing_k28p7        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_MISSING_K28P7);
                    eeAil->k30p7_det            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_K30P7_DET);
                    eeAil->loc_det              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_LOC_DET);
                    eeAil->rx_fifo_ovf          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RX_FIFO_OVF);
                    eeAil->rcvd_los             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RCVD_LOS);
                    eeAil->rcvd_lof             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RCVD_LOF);
                    eeAil->rcvd_rai             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RCVD_RAI);
                    eeAil->rcvd_sdi             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RCVD_SDI);
                    eeAil->rcvd_rst             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RCVD_RST);
                    eeAil->los_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_LOS_ERR);
                    eeAil->lof_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_LOF_ERR);
                    eeAil->hfnsync_state        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_HFNSYNC_STATE);
                    eeAil->lof_state            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_LOF_STATE);
                    eeAil->rm_rst               = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_RAW_STS_RM_RST);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_EV0_ENABLED_STS;
                    eeAil->sync_status_change   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_SYNC_STATUS_CHANGE);
                    eeAil->rm_status_state0     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RM_STATUS_STATE0);
                    eeAil->rm_status_state1     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RM_STATUS_STATE1);
                    eeAil->rm_status_state2     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RM_STATUS_STATE2);
                    eeAil->rm_status_state3     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RM_STATUS_STATE3);
                    eeAil->rm_status_state4     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RM_STATUS_STATE4);
                    eeAil->rm_status_state5     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RM_STATUS_STATE5);
                    eeAil->num_los_det          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_NUM_LOS_DET);
                    eeAil->lcv_det              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_LCV_DET);
                    eeAil->frame_bndry_det      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_FRAME_BNDRY_DET);
                    eeAil->block_bndry_det      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_BLOCK_BNDRY_DET);
                    eeAil->missing_k28p5        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_MISSING_K28P5);
                    eeAil->missing_k28p7        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_MISSING_K28P7);
                    eeAil->k30p7_det            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_K30P7_DET);
                    eeAil->loc_det              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_LOC_DET);
                    eeAil->rx_fifo_ovf          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RX_FIFO_OVF);
                    eeAil->rcvd_los             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RCVD_LOS);
                    eeAil->rcvd_lof             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RCVD_LOF);
                    eeAil->rcvd_rai             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RCVD_RAI);
                    eeAil->rcvd_sdi             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RCVD_SDI);
                    eeAil->rcvd_rst             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RCVD_RST);
                    eeAil->los_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_LOS_ERR);
                    eeAil->lof_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_LOF_ERR);
                    eeAil->hfnsync_state        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_HFNSYNC_STATE);
                    eeAil->lof_state            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_LOF_STATE);
                    eeAil->rm_rst               = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV0_ENABLED_STS_RM_RST);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_EV1_ENABLED_STS;
                    eeAil->sync_status_change   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_SYNC_STATUS_CHANGE);
                    eeAil->rm_status_state0     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RM_STATUS_STATE0);
                    eeAil->rm_status_state1     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RM_STATUS_STATE1);
                    eeAil->rm_status_state2     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RM_STATUS_STATE2);
                    eeAil->rm_status_state3     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RM_STATUS_STATE3);
                    eeAil->rm_status_state4     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RM_STATUS_STATE4);
                    eeAil->rm_status_state5     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RM_STATUS_STATE5);
                    eeAil->num_los_det          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_NUM_LOS_DET);
                    eeAil->lcv_det              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_LCV_DET);
                    eeAil->frame_bndry_det      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_FRAME_BNDRY_DET);
                    eeAil->block_bndry_det      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_BLOCK_BNDRY_DET);
                    eeAil->missing_k28p5        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_MISSING_K28P5);
                    eeAil->missing_k28p7        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_MISSING_K28P7);
                    eeAil->k30p7_det            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_K30P7_DET);
                    eeAil->loc_det              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_LOC_DET);
                    eeAil->rx_fifo_ovf          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RX_FIFO_OVF);
                    eeAil->rcvd_los             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RCVD_LOS);
                    eeAil->rcvd_lof             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RCVD_LOF);
                    eeAil->rcvd_rai             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RCVD_RAI);
                    eeAil->rcvd_sdi             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RCVD_SDI);
                    eeAil->rcvd_rst             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RCVD_RST);
                    eeAil->los_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_LOS_ERR);
                    eeAil->lof_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_LOF_ERR);
                    eeAil->hfnsync_state        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_HFNSYNC_STATE);
                    eeAil->lof_state            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_LOF_STATE);
                    eeAil->rm_rst               = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RM_0_EV1_ENABLED_STS_RM_RST);
                }
            }
            break;

        /** AIL_EE_RT_TM_0_STS
         *  Reports AIL RT TM error info.
         *  Argument: a pointer to Iqn2Fl_AilEeRtTm0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeRtTm0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_RT_TM_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeRtTm0 *eeAil = (Iqn2Fl_AilEeRtTm0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_RAW_STS;
                    eeAil->rt_hdr_error         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_RT_HDR_ERROR);
                    eeAil->rt_em_insert         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_RT_EM_INSERT);
                    eeAil->rt_unfl              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_RT_UNFL);
                    eeAil->rt_ovfl              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_RT_OVFL);
                    eeAil->rt_frm_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_RT_FRM_ERR);
                    eeAil->rt_unalign_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_RT_UNALIGN_ERR);
                    eeAil->rt_aggr_state_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_RT_AGGR_STATE_INFO);
                    eeAil->tm_frame_sync_state  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_SYNC_STATUS_CHANGE);
                    eeAil->delta_inactive       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_DELTA_INACTIVE);
                    eeAil->delta_modified       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_DELTA_MODIFIED);
                    eeAil->frame_misalign       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_FRAME_MISALIGN);
                    eeAil->fifo_undeflow        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_FIFO_UNDEFLOW);
                    eeAil->tm_fail              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_RAW_STS_TM_FAIL);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_EV0_ENABLED_STS;
                    eeAil->rt_hdr_error         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_RT_HDR_ERROR);
                    eeAil->rt_em_insert         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_RT_EM_INSERT);
                    eeAil->rt_unfl              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_RT_UNFL);
                    eeAil->rt_ovfl              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_RT_OVFL);
                    eeAil->rt_frm_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_RT_FRM_ERR);
                    eeAil->rt_unalign_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_RT_UNALIGN_ERR);
                    eeAil->rt_aggr_state_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_RT_AGGR_STATE_INFO);
                    eeAil->tm_frame_sync_state  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_SYNC_STATUS_CHANGE);
                    eeAil->delta_inactive       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_DELTA_INACTIVE);
                    eeAil->delta_modified       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_DELTA_MODIFIED);
                    eeAil->frame_misalign       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_FRAME_MISALIGN);
                    eeAil->fifo_undeflow        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_FIFO_UNDEFLOW);
                    eeAil->tm_fail              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV0_ENABLED_STS_TM_FAIL);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_EV1_ENABLED_STS;
                    eeAil->rt_hdr_error         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_RT_HDR_ERROR);
                    eeAil->rt_em_insert         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_RT_EM_INSERT);
                    eeAil->rt_unfl              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_RT_UNFL);
                    eeAil->rt_ovfl              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_RT_OVFL);
                    eeAil->rt_frm_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_RT_FRM_ERR);
                    eeAil->rt_unalign_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_RT_UNALIGN_ERR);
                    eeAil->rt_aggr_state_info   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_RT_AGGR_STATE_INFO);
                    eeAil->tm_frame_sync_state  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_SYNC_STATUS_CHANGE);
                    eeAil->delta_inactive       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_DELTA_INACTIVE);
                    eeAil->delta_modified       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_DELTA_MODIFIED);
                    eeAil->frame_misalign       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_FRAME_MISALIGN);
                    eeAil->fifo_undeflow        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_FIFO_UNDEFLOW);
                    eeAil->tm_fail              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_RT_TM_0_EV1_ENABLED_STS_TM_FAIL);
                }
            }
            break;

        /** AIL_EE_CI_CO_0_STS
         *  Reports AIL CI CO error info.
         *  Argument: a pointer to Iqn2Fl_AilEeCiCo0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeCiCo0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_CI_CO_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeCiCo0 *eeAil = (Iqn2Fl_AilEeCiCo0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_RAW_STS;
                    eeAil->ci_tbltoolong         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_CI_CO_0_RAW_STS_CI_TBLTOOLONG);
                    eeAil->co_tbltoolong         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_CI_CO_0_RAW_STS_CO_TBLTOOLONG);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_EV0_ENABLED_STS;
                    eeAil->ci_tbltoolong         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_CI_CO_0_EV0_ENABLED_STS_CI_TBLTOOLONG);
                    eeAil->co_tbltoolong         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_CI_CO_0_EV0_ENABLED_STS_CO_TBLTOOLONG);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_EV1_ENABLED_STS;
                    eeAil->ci_tbltoolong         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_CI_CO_0_EV1_ENABLED_STS_CI_TBLTOOLONG);
                    eeAil->co_tbltoolong         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_CI_CO_0_EV1_ENABLED_STS_CO_TBLTOOLONG);
                }
            }
            break;

        /** AIL_SYSCLK_PHY_ORIG_REG
         *  Reports the EE AIL SYSCLK PHY Origination Status Register into a Iqn2Fl_AilSysClkPhyEeOrigin structure
         *  Argument: a pointer to Iqn2Fl_AilSysClkPhyEeOrigin structure
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilSysClkPhyEeOrigin *) - response
         */
        case IQN2FL_QUERY_AIL_SYSCLK_PHY_EE_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilSysClkPhyEeOrigin *eeOrigin = (Iqn2Fl_AilSysClkPhyEeOrigin*)response;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_SYSCLK_PHY_ORIG_REG;
                eeOrigin->ail_ee_rm_0_sts       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_PHY_ORIG_REG_ORIG_EE_0);
                eeOrigin->ail_ee_rt_tm_0_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_PHY_ORIG_REG_ORIG_EE_1);
                eeOrigin->ail_ee_ci_co_0_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_PHY_ORIG_REG_ORIG_EE_2);
            }
            break;

        /** AIL_EE_PD_0_STS
         *  Reports AIL PD 0 error info.
         *  Argument: a pointer to Iqn2Fl_AilEePd0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEePd0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_PD_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEePd0 *eeAil = (Iqn2Fl_AilEePd0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_RAW_STS;
                    eeAil->pd_ee_obsai_frm_win_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_FRM_WIN_ERR);
                    eeAil->pd_ee_sop_err                    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_SOP_ERR);
                    eeAil->pd_ee_obsai_ts_miss_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_TS_MISS_ERR);
                    eeAil->pd_ee_obsai_ts_wdog_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_TS_WDOG_ERR);
                    eeAil->pd_ee_obsai_axc_fail_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_AXC_FAIL_ERR);
                    eeAil->pd_ee_obsai_crc_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_CRC_ERR);
                    eeAil->pd_ee_rp3_01_soc_rst_info        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_RP3_01_SOC_RST_INFO);
                    eeAil->pd_ee_obsai_route_fail_info      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_ROUTE_FAIL_INFO);
                    eeAil->pd_ee_rp3_01_capture_info        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_RP3_01_CAPTURE_INFO);
                    eeAil->pd_ee_rp3_01_crc_fail_err        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_RP3_01_CRC_FAIL_ERR);
                    eeAil->pd_ee_obsai_gsm_off_stb_info     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_GSM_OFF_STB_INFO);
                    eeAil->pd_ee_cpri_cw_crc_err            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_CPRI_CW_CRC_ERR);
                    eeAil->pd_ee_cpri_cw_ovfl_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_CPRI_CW_OVFL_ERR);
                    eeAil->pd_ee_cpri_cw_4b5b_eop_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_CPRI_CW_4B5B_EOP_ERR);
                    eeAil->pd_ee_cpri_cw_4b5b_char_err      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_CPRI_CW_4B5B_CHAR_ERR);
                    eeAil->pd_ee_obsai_sop_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_SOP_INFO);
                    eeAil->pd_ee_obsai_eop_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_EOP_INFO);
                    eeAil->pd_ee_obsai_sof_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_RAW_STS_PD_EE_OBSAI_SOF_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_EV0_ENABLED_STS;
                    eeAil->pd_ee_obsai_frm_win_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_FRM_WIN_ERR);
                    eeAil->pd_ee_sop_err                    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_SOP_ERR);
                    eeAil->pd_ee_obsai_ts_miss_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_TS_MISS_ERR);
                    eeAil->pd_ee_obsai_ts_wdog_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_TS_WDOG_ERR);
                    eeAil->pd_ee_obsai_axc_fail_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_AXC_FAIL_ERR);
                    eeAil->pd_ee_obsai_crc_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_CRC_ERR);
                    eeAil->pd_ee_rp3_01_soc_rst_info        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_RP3_01_SOC_RST_INFO);
                    eeAil->pd_ee_obsai_route_fail_info      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_ROUTE_FAIL_INFO);
                    eeAil->pd_ee_rp3_01_capture_info        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_RP3_01_CAPTURE_INFO);
                    eeAil->pd_ee_rp3_01_crc_fail_err        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_RP3_01_CRC_FAIL_ERR);
                    eeAil->pd_ee_obsai_gsm_off_stb_info     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_GSM_OFF_STB_INFO);
                    eeAil->pd_ee_cpri_cw_crc_err            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_CPRI_CW_CRC_ERR);
                    eeAil->pd_ee_cpri_cw_ovfl_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_CPRI_CW_OVFL_ERR);
                    eeAil->pd_ee_cpri_cw_4b5b_eop_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_CPRI_CW_4B5B_EOP_ERR);
                    eeAil->pd_ee_cpri_cw_4b5b_char_err      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_CPRI_CW_4B5B_CHAR_ERR);
                    eeAil->pd_ee_obsai_sop_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_SOP_INFO);
                    eeAil->pd_ee_obsai_eop_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_EOP_INFO);
                    eeAil->pd_ee_obsai_sof_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV0_ENABLED_STS_PD_EE_OBSAI_SOF_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_EV1_ENABLED_STS;
                    eeAil->pd_ee_obsai_frm_win_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_FRM_WIN_ERR);
                    eeAil->pd_ee_sop_err                    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_SOP_ERR);
                    eeAil->pd_ee_obsai_ts_miss_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_TS_MISS_ERR);
                    eeAil->pd_ee_obsai_ts_wdog_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_TS_WDOG_ERR);
                    eeAil->pd_ee_obsai_axc_fail_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_AXC_FAIL_ERR);
                    eeAil->pd_ee_obsai_crc_err              = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_CRC_ERR);
                    eeAil->pd_ee_rp3_01_soc_rst_info        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_RP3_01_SOC_RST_INFO);
                    eeAil->pd_ee_obsai_route_fail_info      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_ROUTE_FAIL_INFO);
                    eeAil->pd_ee_rp3_01_capture_info        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_RP3_01_CAPTURE_INFO);
                    eeAil->pd_ee_rp3_01_crc_fail_err        = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_RP3_01_CRC_FAIL_ERR);
                    eeAil->pd_ee_obsai_gsm_off_stb_info     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_GSM_OFF_STB_INFO);
                    eeAil->pd_ee_cpri_cw_crc_err            = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_CPRI_CW_CRC_ERR);
                    eeAil->pd_ee_cpri_cw_ovfl_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_CPRI_CW_OVFL_ERR);
                    eeAil->pd_ee_cpri_cw_4b5b_eop_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_CPRI_CW_4B5B_EOP_ERR);
                    eeAil->pd_ee_cpri_cw_4b5b_char_err      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_CPRI_CW_4B5B_CHAR_ERR);
                    eeAil->pd_ee_obsai_sop_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_SOP_INFO);
                    eeAil->pd_ee_obsai_eop_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_EOP_INFO);
                    eeAil->pd_ee_obsai_sof_info             = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_0_EV1_ENABLED_STS_PD_EE_OBSAI_SOF_INFO);
                }
            }
            break;

        /** AIL_EE_PD_1_STS
         *  Reports AIL PD 1 error info.
         *  Argument: a pointer to Iqn2Fl_AilEePd1 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEePd1 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_PD_1_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEePd1 *eeAil = (Iqn2Fl_AilEePd1*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_RAW_STS;
                    eeAil->pd_ee_cpri_bub_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_RAW_STS_PD_EE_CPRI_BUB_FSM_ERR);
                    eeAil->pd_ee_cpri_tdm_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_RAW_STS_PD_EE_CPRI_TDM_FSM_ERR);
                    eeAil->pd_ee_cpri_radstd_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_RAW_STS_PD_EE_CPRI_RADSTD_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_EV0_ENABLED_STS;
                    eeAil->pd_ee_cpri_bub_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_EV0_ENABLED_STS_PD_EE_CPRI_BUB_FSM_ERR);
                    eeAil->pd_ee_cpri_tdm_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_EV0_ENABLED_STS_PD_EE_CPRI_TDM_FSM_ERR);
                    eeAil->pd_ee_cpri_radstd_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_EV0_ENABLED_STS_PD_EE_CPRI_RADSTD_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_EV1_ENABLED_STS;
                    eeAil->pd_ee_cpri_bub_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_EV1_ENABLED_STS_PD_EE_CPRI_BUB_FSM_ERR);
                    eeAil->pd_ee_cpri_tdm_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_EV1_ENABLED_STS_PD_EE_CPRI_TDM_FSM_ERR);
                    eeAil->pd_ee_cpri_radstd_err          = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PD_1_EV1_ENABLED_STS_PD_EE_CPRI_RADSTD_ERR);
                }
            }
            break;

        /** AIL_EE_PE_0_STS
         *  Reports AIL PE 0 error info.
         *  Argument: a pointer to Iqn2Fl_AilEePe0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEePe0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_PE_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEePe0 *eeAil = (Iqn2Fl_AilEePe0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_RAW_STS;
                    eeAil->pe_ee_cpri_cw_null_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_RAW_STS_PE_EE_CPRI_CW_NULL_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_4b5b_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_RAW_STS_PE_EE_CPRI_CW_4B5B_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hypfm_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_RAW_STS_PE_EE_CPRI_CW_HYPFM_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hdlc_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_RAW_STS_PE_EE_CPRI_CW_HDLC_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hypfm_oflow_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_RAW_STS_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR);
                    eeAil->pe_ee_ofifo_oflow_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_RAW_STS_PE_EE_OFIFO_OFLOW_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_EV0_ENABLED_STS;
                    eeAil->pe_ee_cpri_cw_null_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV0_ENABLED_STS_PE_EE_CPRI_CW_NULL_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_4b5b_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV0_ENABLED_STS_PE_EE_CPRI_CW_4B5B_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hypfm_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV0_ENABLED_STS_PE_EE_CPRI_CW_HYPFM_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hdlc_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV0_ENABLED_STS_PE_EE_CPRI_CW_HDLC_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hypfm_oflow_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV0_ENABLED_STS_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR);
                    eeAil->pe_ee_ofifo_oflow_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV0_ENABLED_STS_PE_EE_OFIFO_OFLOW_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_EV1_ENABLED_STS;
                    eeAil->pe_ee_cpri_cw_null_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV1_ENABLED_STS_PE_EE_CPRI_CW_NULL_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_4b5b_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV1_ENABLED_STS_PE_EE_CPRI_CW_4B5B_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hypfm_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV1_ENABLED_STS_PE_EE_CPRI_CW_HYPFM_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hdlc_starve_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV1_ENABLED_STS_PE_EE_CPRI_CW_HDLC_STARVE_ERR);
                    eeAil->pe_ee_cpri_cw_hypfm_oflow_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV1_ENABLED_STS_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR);
                    eeAil->pe_ee_ofifo_oflow_err           = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_PE_0_EV1_ENABLED_STS_PE_EE_OFIFO_OFLOW_ERR);
                }
            }
            break;

        /** AIL_EE_SI_0_STS
         *  Reports AIL SI 0 error info.
         *  Argument: a pointer to Iqn2Fl_AilEeSi0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilEeSi0 *) - response
         */
        case IQN2FL_QUERY_AIL_EE_SI_0_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilEeSi0 *eeAil = (Iqn2Fl_AilEeSi0*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_RAW_STS;
                    eeAil->uat_pi_err               = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_RAW_STS_UAT_PI_ERR);
                    eeAil->cpri_tdm_lut_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_RAW_STS_CPRI_TDM_LUT_ERR);
                    eeAil->cpri_bub_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_RAW_STS_CPRI_BUB_FSM_ERR);
                    eeAil->obsai_phy_sync_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_RAW_STS_OBSAI_PHY_SYNC_ERR);
                    eeAil->obsai_multrulefire_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_RAW_STS_OBSAI_MULTRULEFIRE_ERR);
                    eeAil->obsai_dbm_wrap_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_RAW_STS_OBSAI_DBM_WRAP_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_EV0_ENABLED_STS;
                    eeAil->uat_pi_err               = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV0_ENABLED_STS_UAT_PI_ERR);
                    eeAil->cpri_tdm_lut_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV0_ENABLED_STS_CPRI_TDM_LUT_ERR);
                    eeAil->cpri_bub_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV0_ENABLED_STS_CPRI_BUB_FSM_ERR);
                    eeAil->obsai_phy_sync_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV0_ENABLED_STS_OBSAI_PHY_SYNC_ERR);
                    eeAil->obsai_multrulefire_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV0_ENABLED_STS_OBSAI_MULTRULEFIRE_ERR);
                    eeAil->obsai_dbm_wrap_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV0_ENABLED_STS_OBSAI_DBM_WRAP_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_EV1_ENABLED_STS;
                    eeAil->uat_pi_err               = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV1_ENABLED_STS_UAT_PI_ERR);
                    eeAil->cpri_tdm_lut_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV1_ENABLED_STS_CPRI_TDM_LUT_ERR);
                    eeAil->cpri_bub_fsm_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV1_ENABLED_STS_CPRI_BUB_FSM_ERR);
                    eeAil->obsai_phy_sync_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV1_ENABLED_STS_OBSAI_PHY_SYNC_ERR);
                    eeAil->obsai_multrulefire_err   = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV1_ENABLED_STS_OBSAI_MULTRULEFIRE_ERR);
                    eeAil->obsai_dbm_wrap_err       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_AIL_SI_0_EV1_ENABLED_STS_OBSAI_DBM_WRAP_ERR);
                }
            }
            break;

        /** AIL_SYSCLK_ORIG_REG
         *  Reports the EE AIL SYSCLK Origination Status Register into a Iqn2Fl_AilSysClkEeOrigin structure
         *  Argument: a pointer to Iqn2Fl_AilSysClkEeOrigin structure
         *  use hIqn2->arg_ail to select the AIL instance
         *  Response: (Iqn2Fl_AilSysClkEeOrigin *) - response
         */
        case IQN2FL_QUERY_AIL_SYSCLK_EE_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_AilSysClkEeOrigin *eeOrigin = (Iqn2Fl_AilSysClkEeOrigin*)response;
                tmpReg = hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_SYSCLK_ORIG_REG;
                eeOrigin->ail_ee_pd_0_sts       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_0);
                eeOrigin->ail_ee_pd_1_sts       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_1);
                eeOrigin->ail_ee_pd_2_0_sts     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_2);
                eeOrigin->ail_ee_pd_2_1_sts     = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_3);
                eeOrigin->ail_ee_pe_0_sts       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_6);
                eeOrigin->ail_ee_si_0_sts       = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_7);
                eeOrigin->ail_ee_sii_a_sts      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_8);
                eeOrigin->ail_ee_sii_b_sts      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_9);
                eeOrigin->ail_ee_sii_c_0_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_10);
                eeOrigin->ail_ee_sii_c_1_sts    = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_11);
                eeOrigin->ail_ee_sii_d_sts      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_14);
                eeOrigin->ail_ee_sie_a_sts      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_18);
                eeOrigin->ail_ee_sie_b_sts      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_19);
                eeOrigin->ail_ee_sie_c_sts      = (uint32_t) CSL_FEXT(tmpReg, IQN_AIL_AIL_SYSCLK_ORIG_REG_ORIG_EE_20);
            }
            break;

        /** DIO2_EE_DMA_ING_A_STS
         *  Reports DIO Ingress Interrupt info
         *  Argument: a pointer to Iqn2Fl_Dio2EeDmaIngA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeDmaIngA *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_DMA_ING_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeDmaIngA *eeDio2 = (Iqn2Fl_Dio2EeDmaIngA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_RAW_STS;
                    eeDio2->dma_ing_pend_que_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_STS_DMA_ING_PEND_QUE_OVF_ERR);
                    eeDio2->dma_ing_prog_err                 = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_STS_DMA_ING_PROG_ERR);
                    eeDio2->dma_ing_xfer_done_info           = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_STS_DMA_ING_XFER_DONE_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_EV0_ENABLED_STS;
                    eeDio2->dma_ing_pend_que_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_ENABLED_STS_DMA_ING_PEND_QUE_OVF_ERR);
                    eeDio2->dma_ing_prog_err                 = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_ENABLED_STS_DMA_ING_PROG_ERR);
                    eeDio2->dma_ing_xfer_done_info           = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_ENABLED_STS_DMA_ING_XFER_DONE_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_EV1_ENABLED_STS;
                    eeDio2->dma_ing_pend_que_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_ENABLED_STS_DMA_ING_PEND_QUE_OVF_ERR);
                    eeDio2->dma_ing_prog_err                 = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_ENABLED_STS_DMA_ING_PROG_ERR);
                    eeDio2->dma_ing_xfer_done_info           = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_ENABLED_STS_DMA_ING_XFER_DONE_INFO);
                }
            }
            break;

        /** DIO2_EE_DMA_ING_B_STS
         *  Reports DIO Ingress Interrupt info
         *  Argument: a pointer to Iqn2Fl_Dio2EeDmaIngB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeDmaIngB *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_DMA_ING_B_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeDmaIngB *eeDio2 = (Iqn2Fl_Dio2EeDmaIngB*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_RAW_STS;
                    eeDio2->dma_ing_buf_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_B_RAW_STS_DMA_ING_BUF_OVF_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_EV0_ENABLED_STS;
                    eeDio2->dma_ing_buf_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_B_EV0_ENABLED_STS_DMA_ING_BUF_OVF_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_EV1_ENABLED_STS;
                    eeDio2->dma_ing_buf_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_ING_B_EV1_ENABLED_STS_DMA_ING_BUF_OVF_ERR);
                }
            }
            break;

        /** DIO2_EE_DMA_EGR_A_STS
         *  Reports DIO Egress Interrupt info
         *  Argument: a pointer to Iqn2Fl_Dio2EeDmaEgrA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeDmaEgrA *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_DMA_EGR_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeDmaEgrA *eeDio2 = (Iqn2Fl_Dio2EeDmaEgrA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_RAW_STS;
                    eeDio2->dma_egr_pend_que_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_STS_DMA_EGR_PEND_QUE_OVF_ERR);
                    eeDio2->dma_egr_prog_err                 = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_STS_DMA_EGR_PROG_ERR);
                    eeDio2->dma_egr_xfer_done_info           = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_STS_DMA_EGR_XFER_DONE_INFO);
                    eeDio2->si_ing_iq_fifo_full_info         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_STS_SI_ING_IQ_FIFO_FULL_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_EV0_ENABLED_STS;
                    eeDio2->dma_egr_pend_que_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_ENABLED_STS_DMA_EGR_PEND_QUE_OVF_ERR);
                    eeDio2->dma_egr_prog_err                 = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_ENABLED_STS_DMA_EGR_PROG_ERR);
                    eeDio2->dma_egr_xfer_done_info           = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_ENABLED_STS_DMA_EGR_XFER_DONE_INFO);
                    eeDio2->si_ing_iq_fifo_full_info         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_ENABLED_STS_SI_ING_IQ_FIFO_FULL_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_EV1_ENABLED_STS;
                    eeDio2->dma_egr_pend_que_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_ENABLED_STS_DMA_EGR_PEND_QUE_OVF_ERR);
                    eeDio2->dma_egr_prog_err                 = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_ENABLED_STS_DMA_EGR_PROG_ERR);
                    eeDio2->dma_egr_xfer_done_info           = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_ENABLED_STS_DMA_EGR_XFER_DONE_INFO);
                    eeDio2->si_ing_iq_fifo_full_info         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_ENABLED_STS_SI_ING_IQ_FIFO_FULL_INFO);
                }
            }
            break;

        /** DIO2_EE_DT_A_STS
         *  Reports DIO DIO DataTrace Interrupt info
         *  Argument: a pointer to Iqn2Fl_Dio2EeDtA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeDtA *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_DT_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeDtA *eeDio2 = (Iqn2Fl_Dio2EeDtA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_RAW_STS;
                    eeDio2->dt_buff_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DT_A_RAW_STS_DT_BUFF_OVF_ERR);
                    eeDio2->dt_done_info            = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DT_A_RAW_STS_DT_DONE_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_EV0_ENABLED_STS;
                    eeDio2->dt_buff_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DT_A_EV0_ENABLED_STS_DT_BUFF_OVF_ERR);
                    eeDio2->dt_done_info            = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DT_A_EV0_ENABLED_STS_DT_DONE_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_EV1_ENABLED_STS;
                    eeDio2->dt_buff_ovf_err         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DT_A_EV1_ENABLED_STS_DT_BUFF_OVF_ERR);
                    eeDio2->dt_done_info            = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_DT_A_EV1_ENABLED_STS_DT_DONE_INFO);
                }
            }
            break;

        /** DIO2_EE_SII_A_STS
         *  Reports SI si_i IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSiiA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeSiiA *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_SII_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeSiiA *eeDio2 = (Iqn2Fl_Dio2EeSiiA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_RAW_STS;
                    eeDio2->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_RAW_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeDio2->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_RAW_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeDio2->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeDio2->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeDio2->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_RAW_STS_SI_ING_IQ_IFE_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_EV0_ENABLED_STS;
                    eeDio2->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeDio2->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeDio2->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeDio2->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeDio2->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV0_ENABLED_STS_SI_ING_IQ_IFE_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_EV1_ENABLED_STS;
                    eeDio2->si_ing_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_ICC_SOF_INFO);
                    eeDio2->si_ing_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_ICC_DAT_INFO);
                    eeDio2->si_ing_iq_ife_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_SOP_INFO);
                    eeDio2->si_ing_iq_ife_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_EOP_INFO);
                    eeDio2->si_ing_iq_ife_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_A_EV1_ENABLED_STS_SI_ING_IQ_IFE_DAT_INFO);
                }
            }
            break;

        /** DIO2_EE_SII_C_STS
         *  Reports SI si_i IQ per-channel start of frame errors.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSiiC structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeSiiC *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_SII_C_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeSiiC *eeDio2 = (Iqn2Fl_Dio2EeSiiC*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_RAW_STS;
                    eeDio2->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_C_RAW_STS_SI_ING_IQ_SOF_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_EV0_ENABLED_STS;
                    eeDio2->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_C_EV0_ENABLED_STS_SI_ING_IQ_SOF_ERR);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_EV1_ENABLED_STS;
                    eeDio2->si_ing_iq_sof_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_C_EV1_ENABLED_STS_SI_ING_IQ_SOF_ERR);
                }
            }
            break;

        /** DIO2_EE_SII_E_STS
         *  Reports SI si_i IQ info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSiiE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeSiiE *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_SII_E_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeSiiE *eeDio2 = (Iqn2Fl_Dio2EeSiiE*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_RAW_STS;
                    eeDio2->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_E_RAW_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeDio2->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_E_RAW_STS_SI_ING_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_EV0_ENABLED_STS;
                    eeDio2->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_E_EV0_ENABLED_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeDio2->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_E_EV0_ENABLED_STS_SI_ING_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_EV1_ENABLED_STS;
                    eeDio2->si_ing_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_E_EV1_ENABLED_STS_SI_ING_IQ_PSI_EOP_INFO);
                    eeDio2->si_ing_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_E_EV1_ENABLED_STS_SI_ING_IQ_PSI_DAT_INFO);
                }
            }
            break;

        /** DIO2_EE_SII_G_STS
         *  Reports SI si_i IQ per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSiiG structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeSiiG *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_SII_G_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeSiiG *eeDio2 = (Iqn2Fl_Dio2EeSiiG*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_RAW_STS;
                    eeDio2->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_G_RAW_STS_SI_ING_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_EV0_ENABLED_STS;
                    eeDio2->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_G_EV0_ENABLED_STS_SI_ING_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_EV1_ENABLED_STS;
                    eeDio2->si_ing_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SII_G_EV1_ENABLED_STS_SI_ING_IQ_PSI_SOP_INFO);
                }
            }
            break;


        /** DIO2_EE_SIE_A_STS
         *  Reports SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSieA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeSieA *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_SIE_A_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeSieA *eeDio2 = (Iqn2Fl_Dio2EeSieA*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_RAW_STS;
                    eeDio2->si_egr_iq_efe_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeDio2->si_egr_iq_efe_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeDio2->si_egr_iq_efe_sym_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_RAW_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeDio2->si_egr_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_RAW_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeDio2->si_egr_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_RAW_STS_SI_EGR_IQ_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_EV0_ENABLED_STS;
                    eeDio2->si_egr_iq_efe_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeDio2->si_egr_iq_efe_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeDio2->si_egr_iq_efe_sym_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeDio2->si_egr_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeDio2->si_egr_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV0_ENABLED_STS_SI_EGR_IQ_ICC_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_EV1_ENABLED_STS;
                    eeDio2->si_egr_iq_efe_starve_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_STARVE_ERR);
                    eeDio2->si_egr_iq_efe_pkt_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_PKT_ERR);
                    eeDio2->si_egr_iq_efe_sym_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_EFE_SYM_ERR);
                    eeDio2->si_egr_iq_icc_sof_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_ICC_SOF_INFO);
                    eeDio2->si_egr_iq_icc_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_A_EV1_ENABLED_STS_SI_EGR_IQ_ICC_DAT_INFO);
                }
            }
            break;

        /** DIO2_EE_SIE_D_STS
         *  Reports SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSieD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeSieD *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_SIE_D_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeSieD *eeDio2 = (Iqn2Fl_Dio2EeSieD*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_RAW_STS;
                    eeDio2->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeDio2->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeDio2->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_RAW_STS_SI_EGR_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_EV0_ENABLED_STS;
                    eeDio2->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeDio2->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeDio2->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_EV0_ENABLED_STS_SI_EGR_IQ_PSI_DAT_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_EV1_ENABLED_STS;
                    eeDio2->si_egr_iq_psi_data_type_err  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_DATA_TYPE_ERR);
                    eeDio2->si_egr_iq_psi_eop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_EOP_INFO);
                    eeDio2->si_egr_iq_psi_dat_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_D_EV1_ENABLED_STS_SI_EGR_IQ_PSI_DAT_INFO);
                }
            }
            break;

        /** DIO2_EE_SIE_F_STS
         *  Reports SI si_e IQ per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSieF structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  Response: (Iqn2Fl_Dio2EeSieF *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_SIE_F_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeSieF *eeDio2 = (Iqn2Fl_Dio2EeSieF*)response;

                if(hIqn2->arg_ee == IQN2FL_EE_INT_RAW_STATUS){
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_RAW_STS;
                    eeDio2->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_F_RAW_STS_SI_EGR_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV0) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_EV0_ENABLED_STS;
                    eeDio2->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_F_EV0_ENABLED_STS_SI_EGR_IQ_PSI_SOP_INFO);
                } else if (hIqn2->arg_ee == IQN2FL_EE_INT_EN_STATUS_EV1) {
                    tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_EV1_ENABLED_STS;
                    eeDio2->si_egr_iq_psi_sop_info  = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_EE_SIE_F_EV1_ENABLED_STS_SI_EGR_IQ_PSI_SOP_INFO);
                }
            }
            break;

        /** DIO2_ORIG_REG
         *  Reports the EE DIO2 SYSCLK Origination Status Register into a Iqn2Fl_Dio2EeOrigin structure
         *  Argument: a pointer to Iqn2Fl_Dio2EeOrigin structure
         *  Response: (Iqn2Fl_Dio2EeOrigin *) - response
         */
        case IQN2FL_QUERY_DIO2_EE_ORGN_STS:
            {
                uint32_t tmpReg;
                Iqn2Fl_Dio2EeOrigin *eeOrigin = (Iqn2Fl_Dio2EeOrigin*)response;
                tmpReg = hIqn2->regs->Dio2.DIO2_EE.DIO2_DIO2_ORIG_REG;
                eeOrigin->dio2_ee_dma_ing_a_sts     = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_ING_A);
                eeOrigin->dio2_ee_dma_ing_b_sts     = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_ING_B);
                eeOrigin->dio2_ee_dma_egr_a_sts     = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_EGR_A);
                eeOrigin->dio2_ee_dt_a_sts          = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_DT_A);
                eeOrigin->dio2_ee_sii_a_sts         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_EGR_B);
                eeOrigin->dio2_ee_sii_c_sts         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_EGR_C);
                eeOrigin->dio2_ee_sie_a_sts         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_ING_C);
                eeOrigin->dio2_ee_sii_e_sts         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_EGR_D);
                eeOrigin->dio2_ee_sii_g_sts         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_EGR_E);
                eeOrigin->dio2_ee_sie_d_sts         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_ING_D);
                eeOrigin->dio2_ee_sie_e_sts         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_ING_E);
                eeOrigin->dio2_ee_sie_f_sts         = (uint32_t) CSL_FEXT(tmpReg, IQN_DIO2_DIO2_DIO2_ORIG_REG_ORIG_BITFIELD_ING_F);
            }
            break;

        default:
            status = IQN2FL_INVQUERY;
            break;
    }

    return status;
}
