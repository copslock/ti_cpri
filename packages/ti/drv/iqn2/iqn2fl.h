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


/** @defgroup IQN2FL_API IQN2FL
 *
 * @section Introduction
 *
 * @subsection xxx Purpose and Scope
 * The purpose of this document is to identify a set of functional APIs for
 * the IQN2 HW module.
 *
 * @subsection aaa Terms and Abbreviations
 *   -# CSL:  Chip Support Library
 *   -# API:  Application Programmer Interface
 *   -# IQN2: IQ Net peripheral version 2
 *
 *
 */

/** @file iqn2fl.h
 *
 *  @brief Header file for functional layer of IQN2
 *
 *  Description
 *  - The different symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */

/**
 * @defgroup IQN2FL_DATASTRUCT IQN2FL Data Structures
 * @ingroup IQN2FL_API
 */
/**
 * @defgroup IQN2FL_ENUM  IQN2FL Enumerated Data Types
 * @ingroup IQN2FL_API
 */
/**
 * @defgroup IQN2FL_FUNCTION  IQN2FL Functions
 * @ingroup IQN2FL_API
 */

#ifndef _IQN2FL_H_
#define _IQN2FL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <ti/csl/csl.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr_iqn2.h>

/**************************************************************************\
* IQN2FL global macro declarations
\**************************************************************************/

/** @addtogroup IQN2FL_ENUM
*
* @{ */

/**
 *
 * @brief iqn2 AIL instance indices
 *
 * Use this symbol to specify the iqn2 AIL instance
 *  */
typedef enum 
{
    /** Selects AIL0 */
    IQN2FL_AIL_0  = 0,
    /** Selects AIL1 */
    IQN2FL_AIL_1  = 1,
    /** Max AIL supported */
    IQN2FL_AIL_MAX,
    /** Unsupported AIL */
    IQN2FL_AIL_NONE = 0xF

} Iqn2Fl_AilInstance;


/**
 *
 * @brief iqn2fl return and error codes
 *
 *  */
typedef enum
{
    /** Action not supported by IQN2FL */
    IQN2FL_NOTSUPPORTED = -9,
    /** Invalid query to IQN2FL */
    IQN2FL_INVQUERY = -8,
    /** Invalid command to IQN2FL */
    IQN2FL_INVCMD = -7,
    /** Invalid parameters passed to IQN2FL */
    IQN2FL_INVPARAMS = -6,
    /** Handle passed to IQN2FL was invalid */
    IQN2FL_BADHANDLE = -5,
    /** Encoutered IQN2 system resource overflow */
    IQN2FL_OVFL = -4,
    /** Unused code */
    IQN2FL_UNUSED = -3,
    /** IQN2 Peripheral resource is already in use */
    IQN2FL_INUSE = -2,
    /** IQN2 Generic Failure */
    IQN2FL_FAIL = -1,
    /** IQN2FL successful return code */
    IQN2FL_SOK = 1
} Iqn2Fl_Status;


/**
 *
 * @brief  link rates supported
 *
 * Use this symbol to specify the  link rate
 *  */
typedef enum 
{
   /** Selects 8X link rate */
   IQN2FL_LINK_RATE_8x =0,
   /** Selects 4X link rate */
   IQN2FL_LINK_RATE_4x,
   /** Selects 2X link rate */
   IQN2FL_LINK_RATE_2x,
   /** Selects 5X link rate - only for CPRI */
   IQN2FL_LINK_RATE_5x,
   /** Selects 10X link rate - only for CPRI */
   IQN2FL_LINK_RATE_10x,
   /** Selects 16X link rate - only for CPRI */
   IQN2FL_LINK_RATE_16x

} Iqn2Fl_LinkRate;


/**
 *
 * @brief iqn2 channel radio standard select
 *
 * Use this symbol to specify the iqn2 
 * channel Radio Standard Selection
 *  */
typedef enum 
{
    /** Selects Radio Standard 0 */
    IQN2FL_CHAN_RADIO_SEL_STD_0  = 0,
    /** Selects Radio Standard 1 */
    IQN2FL_CHAN_RADIO_SEL_STD_1,
    /** Selects Radio Standard 2 */
    IQN2FL_CHAN_RADIO_SEL_STD_2,
    /** Selects Radio Standard 3 */
    IQN2FL_CHAN_RADIO_SEL_STD_3,
    /** Selects Radio Standard 4 */
    IQN2FL_CHAN_RADIO_SEL_STD_4,
    /** Selects Radio Standard 5 */
    IQN2FL_CHAN_RADIO_SEL_STD_5,
    /** Selects Radio Standard 6 */
    IQN2FL_CHAN_RADIO_SEL_STD_6,
    /** Selects Radio Standard 7 */
    IQN2FL_CHAN_RADIO_SEL_STD_7,
    /** Selects no link */
    IQN2FL_CHAN_RADIO_SEL_STD_NONE = 0xF

} Iqn2Fl_ChanRadioSel;


/**
 *
 * @brief Force a channel to shut off
 *
 * Use this symbol to force or not force the iqn2 
 * channel shut off
 *  */
typedef enum 
{
    /** No forcing off of symbols */
    IQN2FL_CHAN_NO_FRC_OFF_SYM  = 0,
    /** Force symbols off */
    IQN2FL_CHAN_FRC_SYM_OFF  = 1

} Iqn2Fl_ChanFrcOff;


/**
 *
 * @brief Selects channel ENET or non-ENET mode
 *
 * Use this symbol to specify the iqn2 channel mode
 * to be ENET or non-ENET
 *  */
typedef enum 
{
    /** Selects non-ENET mode when low */
    IQN2FL_CHAN_ENET_CTL_NON_ENET  = 0,
    /** Selects ENET mode when high */
    IQN2FL_CHAN_ENET_CTL_ENET  = 1

} Iqn2Fl_ChanEnetCtlMode;


/**
 *
 * @brief Channel OBSAI control
 *
 * Use this symbol to specify link formats for channel
 *  */
typedef enum 
{
    /** Selects the OBSAI channel as AxC  */
    IQN2FL_CHAN_OBSAI_AXC  = 0,
    /** Selects the OBSAI channel as Control */
    IQN2FL_CHAN_OBSAI_CTL

} Iqn2Fl_ChanObsaiCtl;


/**
 *
 * @brief AIL protocol
 *
 * Use this enum to select Cpri or Obsai for AIL PHY Global Configuration Register, and DFE protocol in case of AID/DFE
 *  */
typedef enum
{
    /** Selects the CPRI protocol */
    IQN2FL_PROTOCOL_CPRI = 0,
    /** Selects the OBSAI protocol */
    IQN2FL_PROTOCOL_OBSAI,
    /** Selects the DFE protocol */
    IQN2FL_PROTOCOL_DFE_245_76,
    /** Selects the DFE protocol */
    IQN2FL_PROTOCOL_DFE_368_64

} Iqn2Fl_Protocol;


/**
 *
 * @brief  data width format supported
 *
 * Use this symbol to specify DL/UL and Generic data formats for IQN2
 *  */
typedef enum 
{
   /** Selects 7bit data width */ 
   IQN2FL_DATA_WIDTH_7_BIT = 0,
   /** Selects 8bit data width */ 
   IQN2FL_DATA_WIDTH_8_BIT,
   /** Selects 15bit data width */ 
   IQN2FL_DATA_WIDTH_15_BIT,
   /** Selects 16bit data width */ 
   IQN2FL_DATA_WIDTH_16_BIT

} Iqn2Fl_DataWidth;

/**
 *
 * @brief  event strove selection
 *
 * Use this symbol to specify rad event strobe type for AT
 *
 *     RADT0_frame (0) = Use RADT Frame to trigger the event
 *     RADT1_frame (1) = Use RADT Frame to trigger the event
 *     RADT2_frame (2) = Use RADT Frame to trigger the event
 *     RADT3_frame (3) = Use RADT Frame to trigger the event
 *     RADT4_frame (4) = Use RADT Frame to trigger the event
 *     RADT5_frame (5) = Use RADT Frame to trigger the event
 *     RADT6_frame (6) = Use RADT Frame to trigger the event
 *     RADT7_frame (7) = Use RADT Frame to trigger the event
 *     RADT0_symb (8) = Use RADT Symbol to trigger the event
 *     RADT1_symb (9) = Use RADT Symbol to trigger the event
 *     RADT2_symb (10) = Use RADT Symbol to trigger the event
 *     RADT3_symb (11) = Use RADT Symbol to trigger the event
 *     RADT4_symb (12) = Use RADT Symbol to trigger the event
 *     RADT5_symb (13) = Use RADT Symbol to trigger the event
 *     RADT6_symb (14) = Use RADT Symbol to trigger the event
 *     RADT7_symb (15) = Use RADT Symbol to trigger the event
 *     BCN_frame (16) = Use BCN Frame to trigger the event
 *     EVT_nrest (17) = Use Next higher event to trigger this event.
 */
typedef enum
{
	IQN2FL_RADT0_FRAME = 0,
	IQN2FL_RADT1_FRAME,
	IQN2FL_RADT2_FRAME,
	IQN2FL_RADT3_FRAME,
	IQN2FL_RADT4_FRAME,
	IQN2FL_RADT5_FRAME,
	IQN2FL_RADT6_FRAME,
	IQN2FL_RADT7_FRAME,
	IQN2FL_RADT0_SYMBOL,
	IQN2FL_RADT1_SYMBOL,
	IQN2FL_RADT2_SYMBOL,
	IQN2FL_RADT3_SYMBOL,
	IQN2FL_RADT4_SYMBOL,
	IQN2FL_RADT5_SYMBOL,
	IQN2FL_RADT6_SYMBOL,
	IQN2FL_RADT7_SYMBOL,
	IQN2FL_BCN_FRAME,
	IQN2_EVT_NREST
} Iqn2Fl_AtEvtStrobe;

/**
 *
 * @brief  channel destination
 *
 * Use this symbol to specify the egress channel destination
*     AID2_AXC (0) = Selects AID2 AXC as the destination
*     AID2_CTl (1) = Selects AID2 CTL as the destination
*     AIL0_AXC (2) = Selects AIL0 AXC as the destination
*     AIL0_CTL (3) = Selects AIL0 CTL as the destination
*     AIL1_AXC (4) = Selects AIL1 AXC as the destination
*     AIL1_CTL (5) = Selects AIL1_CTL as the destination
*     AIL2_AXC (6) = Selects AIL2 AXC as the destination
*     AIL2_CTL (7) = Selects AIL2 CTL as the destination
*     AIL3_AXC (8) = Selects AIL3 AXC as the destination
*     AIL3_CTL (9) = Selects AIL3_CTL as the destination
*/
typedef enum
{
	IQN2FL_AID2_AXC = 0,
	IQN2FL_AID2_CTL,
	IQN2FL_AIL0_AXC,
	IQN2FL_AIL0_CTL,
	IQN2FL_AIL1_AXC,
	IQN2FL_AIL1_CTL,
	IQN2FL_AIL2_AXC,
	IQN2FL_AIL2_CTL,
	IQN2FL_AIL3_AXC,
	IQN2FL_AIL3_CTL
} Iqn2Fl_EgressChanDest;

/**
 *
 * @brief  channel destination
 *
 * Use this symbol to specify the ingress channel destination
 * Selects destination port.
 *     PKTDMA (0) = Selects PKTDMA as the destination
 *     DIO2 (1) = Selects DIO2 as the destination
 *     RESERVED_2 (2) = Selects a reserved destination, do not use.
 *     RESERVED_3 (3) = Selects a reserved destination, do not use.
 */
typedef enum
{
	IQN2FL_PKTDMA = 0,
	IQN2FL_DIO2
} Iqn2Fl_IngressChanDest;

/**
 *
 * @brief  buffer channel number
 *
 * Use this symbol to specify the table selection
*  Selects which buffer channel number table for the DMA engine to use
*     TABLE_A (0) = Selects buffer channel number table A (addressed
*                   from 0 to N/2-1)
*     TABLE_B (1) = Selects buffer channel number table B (addressed
*                   from N/2 to N-1)
*/
typedef enum
{
	IQN2FL_TABLE_A = 0,
	IQN2FL_TABLE_B
} Iqn2Fl_BcnTableSel;

/**
 *
 * @brief  DMA burst lenght
 *
 * Sets the maximum DMA burst length.
 *     1QUAD (0) = 1 quad word transferred per burst
 *     2QUAD (1) = 2 quad words transferred per burst
 *     4QUAD (2) = 4 quad words transferred per burst
 */
typedef enum
{
	IQN2FL_1QW = 0,
	IQN2FL_2QW,
	IQN2FL_4QW
} Iqn2Fl_DmaNumQuadWord;

/**
 *
 * @brief  RT control for insertion/aggregation into PHY
 *
 * OBSAI/CPRI Controls RT to perform appropriate insterion/aggregation
 *  into PHY applied on msg-by-msg basis for OBSAI or sample-by-sample
 *  basis for CPRI.
 *    ADD8 (3) = Aggregate/add both PE and RM contributions, samples
 *               are 8bit I and Q -or- 7bit I and Q
 *    ADD16 (2) = Aggregate/add both PE and RM contributions, samples
 *                are 16bit I and Q -or- 15bit I and Q
 *    INSERTPE (1) = Insert the PE contribution, ignoring any possible
 *                   RM contribution
 *    FWD_RM (0) = Forward the RM contribution ignoring the PE contribution
 */
typedef enum
{
	IQN2FL_FWD_RM = 0,
	IQN2FL_INSERTPE,
	IQN2FL_ADD16,
	IQN2FL_ADD8
} Iqn2Fl_RtControll;

/**
 * @brief This is the set of control commands that are passed to
 * @a Iqn2Fl_HwControl(), with an optional argument type-casted to @a void*
 *
 * The arguments, if any, to be passed with each command are specified
 * next to that command.
 */
typedef enum 
{
    /** AIL EFE Global Enable Set. Use hIqn2->arg_ail to select 
     *  AIL instance (argument type: uint32_t * ) 
     */
    IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_SET = 0,

    /** AIL EFE Global Enable Clear. Use hIqn2->arg_ail to select 
     *  AIL instance (argument type: uint32_t * ) 
     */
    IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_CLEAR,

    /** AIL IQ EFE CFG - LOOPBACK ENABLE
     *  EFE Configuration Register.
     *  (TI use Only) 0x1: Ingress data from ICC is looped back to 
     *  Egress data to ICC. DMA traffic is unused. (i.e. for purpose 
     *  of early DFE only testing)
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *) 
     */
    IQN2FL_CMD_AIL_EFE_CFG_LOOPBACK_EN_REG,

    /** AIL IQ EFE CHAN CFG - CHANNEL ENABLE/DISABLE
     *  EFE DMA Channel Configuration Register.
     *  Use hIqn2->arg_ail to select AIL instance
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG,

    /** AIL IQ IFE CHAN CFG - CHANNEL ENABLE/DISABLE
     *  IFE DMA Channel Configuration Register.
     *  Use hIqn2->arg_ail to select AIL instance
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG,

    /** AIL IQ IDC CH CFG - CHANNEL FORCE OFF
     *  IDC Channel Configuration Register.
     *  Use hIqn2->arg_ail to select AIL instance
     *  (argument type: channel index of type uint8_t *)
     */
    IQN2FL_CMD_AIL_IDC_CH_CFG_CHAN_FRC_OFF_REG,

    /**  AIL IQ EFE CHAN CFG - TDD CHANNEL FORCE OFF
     *  EFE DMA Channel Configuration Register.
     *  Use hIqn2->arg_ail to select AIL instance
     *  (argument type: channel index of type uint8_t *)
     */
    IQN2FL_CMD_AIL_EFE_CHAN_CFG_TDD_CHAN_FRC_OFF_REG,

    /** AIL SI IQ E SCH PHY - PE DMA Channel 1 Register 
     *  SI Egress AIL scheduler, PHY FSM is enabled to turn ON, 
     *  will turn on next PE_FB from uAT
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *) 
     */
    IQN2FL_CMD_AIL_SI_IQ_E_SCH_PHY_EN_REG,

    /** AIL ECTL Global Enable Set. Use hIqn2->arg_ail to select 
     *  AIL instance (argument type: uint32_t * ) 
     */
    IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_SET,

    /** AIL ECTL Global Enable Clear. Use hIqn2->arg_ail to select 
     *  AIL instance (argument type: uint32_t * ) 
     */
    IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_CLEAR,

    /** AIL SI IQ E SCH CPRI - PE CPRI RADSTD CFG. 
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *) 
     *  (argument type: Iqn2Fl_AilSiIqCpriRadstdCfg * )
     */
    IQN2FL_CMD_AIL_PE_CPRI_RADSTD_CFG,

    /** AIL IFE Global Enable Set. Use hIqn2->arg_ail to select 
     *  AIL instance (argument type: uint32_t * ) 
     */
    IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_SET,

    /** AIL IFE Global Enable Clear. Use hIqn2->arg_ail to select 
     *  AIL instance (argument type: uint32_t * ) 
     */
    IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_CLEAR,

    /** AIL ICTL Global Enable Set. Use hIqn2->arg_ail to select 
     *  AIL instance (argument type: uint32_t * ) 
     */
    IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_SET,

    /** AIL ICTL Global Enable Clear. Use hIqn2->arg_ail to select 
     *  AIL instance (argument type: uint32_t * ) 
     */
    IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_CLEAR,

    /** AIL ECTL RATE CTL CFG - ECTL Rate Control Configuration register
     *  Rate Controller will allow the ECTL to create RATE+1 active requests on the PSI bus within
     *  a 16 clock cycle window. As an example, a value of 7 will allow the ICTL to create 8 active
     *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *)
     */
    IQN2FL_CMD_AIL_ECTL_RATE_CTL_CFG_REG,

    /** AIL ICTL RATE CTL CFG - ICTL Rate Control Configuration register
     *  Rate Controller will allow the ICTL to create RATE+1 active requests on the PSI bus within
     *  a 16 clock cycle window. As an example, a value of 7 will allow the ICTL to create 8 active
     *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *)
     */
    IQN2FL_CMD_AIL_ICTL_RATE_CTL_CFG_REG,

    /** AIL IQ IDC RATE CTL CFG - IDC Rate Control Configuration register
     *  Rate Controller will allow the IDC to create RATE+1 active requests on the PSI bus within
     *  a 16 clock cycle window. As an example, a value of 7 will allow the IDC to create 8 active
     *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *)
     */
    IQN2FL_CMD_AIL_IQ_IDC_RATE_CTL_CFG_REG,

    /** AIL UAT GEN CTL -> UAT_RUN and DIAG SYNC
     *  This register simply starts the uAT timers running. It is implied 
     *  that SW is unable to precisely time the start of timers. The 
     *  intent is for the SW to correct the timers by later writting to 
     *  the offset register of each timer.
     *   UAT run starts the BCN and RAD counters free running.
     *   diag_sync = 1 starts the BCN and RAD counters if uat_run is set 
     *     and an AT sync is received. This is only used in simulation and 
     *     for diagnostics.
     *  (argument type: Iqn2Fl_UatCfg* )
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t * )
     */
    IQN2FL_CMD_AIL_UAT_GEN_CTL_UAT_CFG_REG,

    /** AIL UAT GEN CTL -> AIL UAT BCN TERMINAL COUNT REGISTER. 
     *  UAT BCN terminal count. BCN counts from zero to this limit 
     *  and wraps to zero. Program as 2,457,599 for sys_clk=245.76MHz 
     *  and 3,071,999 for sys_clk=307.2MHz.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_TERMINAL_COUNT_REG,

    /** AIL UAT GEN CTL -> AIL UAT BCN OFFSET REGISTER. 
     *  Offset correction to the raw uAT BCN counter. Used to correct 
     *  the alignment of the local uAT BCN to the master AT2 BCN. BCN is 
     *  initially randomly started. SW uses uat_sync_bcn_capture_sts 
     *  rd_val to calculate offset correction factor. This correction 
     *  factor will be Frame size - captured value.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_OFFSET_REG,

    /** AIL UAT AIL ONLY REGS -> AIL UAT PIMAX CFG REGISTER. 
     *  (AIL use only) PI max window. One of two values which indicate 
     *  the legal range for OBSAI or CPRI PHY SOF. When an SOF is recieved 
     *  outside this window, and error is indicated (EE).
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_UAT_PIMAX_CFG_REG,

    /** AIL UAT AIL ONLY REGS -> AIL UAT PIMIN CFG REGISTER. 
     *  (AIL use only) PI min window. One of two values which indicate 
     *  the legal range for OBSAI or CPRI PHY SOF. When an SOF is recieved 
     *  outside this window, and error is indicated (EE).
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_UAT_PIMIN_CFG_REG,

    /** AIL UAT AIL ONLY REGS -> AIL UAT TM FRAME COUNT (BFN) CONFIG REGISTER. 
     *  (AIL CPRI use only) uAT CPRI BFN count value (Write only). SW 
     *  overwrite current value. uAT will increment every TM_FRM_STB.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint16_t *) 
     */
    IQN2FL_CMD_AIL_UAT_TM_BFN_CFG_REG,

    /** AIL UAT AIL ONLY REGS -> AIL UAT RT FRAME BOUNDARY (FB) COMPARE REGISTER. 
     *  (AIL use only) uAT BCN compare value which cause RT_STB to fire. 
     *  Used for Egress PHY timing. RT_STB is the latest moment that RT 
     *  will wait for PE and CI SOF contribution before progressing without 
     *  either input.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_UAT_RT_FB_CFG_REG,

    /** AIL UAT AIL ONLY REGS -> AIL UAT PE FRAME BOUNDARY (FB) COMPARE REGISTER. 
     *  (AIL use only) uAT BCN compare value which cause PE_STB to fire. 
     *  Used for Egress PROTO & PHY timing. PE_STB is the exact time which
     *  PE will start building the PHY protocol. It also represents the 
     *  latest timing for incoming DMA data to contribute to the PHY. Late 
     *  DMA data is rejected by PE.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_UAT_PE_FB_CFG_REG,

    /** AIL UAT AIL ONLY REGS -> AIL UAT TM FRAME BOUNDARY (FB) COMPARE REGISTER. 
     *  (AIL use only) uAT BCN compare value which cause RT_STB to fire. 
     *  Used for Egress PHY timing. TM_STB (OBSAI Delta) is the precise 
     *  time at which TM exports the CPRI or OBSAI PHY SOF.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_UAT_TM_FB_CFG_REG,

    /** AIL UAT EGR RADT -> AIL UAT RADT TERMINAL COUNT REGISTER. 
     *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with 
     *  sys_clk=245.76MHz).
     *  Use hIqn2->arg_ail to select AIL instance 
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AIL_UAT_EGR_RADT_TC_CFG_REG,

    /** AIL UAT EGR RADT -> AIL UAT RADT OFFSET REGISTER. 
     *  UAT RADT offset. Value which is added to the raw RADT 
     *  as a timing correction. RadT is initially randomly started, 
     *  SW uses radt_capture value to calculate offset correction factor.
     *  This correction factor will be Frame size - captured value.
     *  Use hIqn2->arg_ail to select AIL instance 
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AIL_UAT_EGR_RADT_OFFSET_CFG_REG,

    /** AIL UAT ING RADT -> AIL UAT RADT TERMINAL COUNT REGISTER. 
     *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with 
     *  sys_clk=245.76MHz).
     *  Use hIqn2->arg_ail to select AIL instance 
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AIL_UAT_ING_RADT_TC_CFG_REG,

    /** AIL UAT ING RADT -> AIL UAT RADT OFFSET REGISTER. 
     *  UAT RADT offset. Value which is added to the raw RADT 
     *  as a timing correction. RadT is initially randomly started, 
     *  SW uses radt_capture value to calculate offset correction factor.
     *  This correction factor will be Frame size - captured value.
     *  Use hIqn2->arg_ail to select AIL instance 
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AIL_UAT_ING_RADT_OFFSET_CFG_REG,

    /** AIL UAT RADT EVT -> AIL UAT RADT EVENT COMPARE REGISTER. 
     *  UAT RADT event compare per RADT. When compare value equals RADT 
     *  count, frame rate event is generated. Also periodic event (i.e. 4SAMP)
     *  is started. The 0 to 7 are for si egress, 8 to 15 for si ingress, 
     *  16 to 18 for dio egress, 19 to 21 for dio ingress.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AIL_UAT_EVT_RADT_CMP_CFG_REG,

    /** AIL UAT RADT EVT -> AIL UAT RADT EVENT CLOCK COUNT TC REGISTER. 
     *  UAT RADT event clock counter terminal count controls spacing of the 
     *  periodic strobe (i.e. 4SAMP). Once the uat_evt_radt_cmp_cfg equals 
     *  the RADT, the period strobe will fire and re-fire every time a 
     *  clock counter reaches this terminal count. The The 0 to 7 are for si 
     *  egress, 8 to 15 for si ingress, 16 to 18 for dio egress, 19 to 21 
     *  for dio ingress.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AIL_UAT_EVT_CLK_CNT_TC_CFG_REG,

    /** AIL PD CPRI AXC CFG - PD CPRI RADSTD CFG. 
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *) 
     *  (argument type: Iqn2Fl_AilSiIqCpriRadstdCfg * )
     */
    IQN2FL_CMD_AIL_PD_CPRI_AXC_RADSTD_CFG,

    /** AIL PHY CI LUT -> CI LUT CFG 
     *  The AIL PHY CI LUT Select Register selects between the LUT A 
     *  table and the LUT B table for CI CPRI Conversion control. 
     *  Used for dynamic modification of CI effectively giving the 
     *  user a Ping Pong buffer. Select takes effect on next PHY 
     *  frame boundary.
     *  0 Selects Table A, 1 Selects Table B
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_PHY_CI_LUT_CFG_REG,

    /** AIL PHY CO LUT -> CO LUT CFG 
     *  The AIL PHY CO LUT Select Register selects between the LUT A 
     *  table and the LUT B table for CO CPRI Conversion control. 
     *  Used for dynamic modification of CO effectively giving the 
     *  user a Ping Pong buffer. Select takes effect on next PHY 
     *  frame boundary.
     *  0 Selects Table A, 1 Selects Table B
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_PHY_CO_LUT_CFG_REG,

    /** AIL PHY CI LUT A -> LUT A CFG
     *  The AIL PHY CI Look-up Table A Registers control up to eight 
     *  groups of AxC containers for every given CPRI basic frame.
     *  Use hIqn2->arg_ail to select AIL instance 
     *  (argument type: Iqn2Fl_AilPhyLutSetup * )
     */
    IQN2FL_CMD_AIL_PHY_CI_LUTA_CFG_REG,

    /** AIL PHY CI LUT B -> LUT B CFG
     *  The AIL PHY CI Look-up Table B Registers control up to eight 
     *  groups of AxC containers for every given CPRI basic frame.
     *  Use hIqn2->arg_ail to select AIL instance 
     *  (argument type: Iqn2Fl_AilPhyLutSetup * )
     */
    IQN2FL_CMD_AIL_PHY_CI_LUTB_CFG_REG,

    /** AIL PHY CO LUT A -> LUT A CFG
     *  The AIL PHY CO Look-up Table A Registers control up to eight 
     *  groups of AxC containers for every given CPRI basic frame.
     *  Use hIqn2->arg_ail to select AIL instance 
     *  (argument type: Iqn2Fl_AilPhyLutSetup * )
     */
    IQN2FL_CMD_AIL_PHY_CO_LUTA_CFG_REG,

    /** AIL PHY CO LUT B -> LUT B CFG
     *  The AIL PHY CO Look-up Table B Registers control up to eight 
     *  groups of AxC containers for every given CPRI basic frame.
     *  Use hIqn2->arg_ail to select AIL instance 
     *  (argument type: Iqn2Fl_AilPhyLutSetup * )
     */
    IQN2FL_CMD_AIL_PHY_CO_LUTB_CFG_REG,

    /** AIL PHY TM CFG -> ENABLE/DISABLE
     *  The TM Configuration Register is used to program basic 
     *  functionality of the Tx Mac Block. Bit 0 can be updated at any 
     *  time to turn on or off the link.
     *  The Transmit Enable Bit allows the TM block transmit state 
     *  machine to operate
     *     DISABLE (0) = TM Block Disabled
     *     ENABLE (1) = TM Block Enabled
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_PHY_TM_CFG_EN_REG,

    /** AIL PHY RM CFG -> RX ENABLE/DISABLE
     *  RM Configuration Register
     *  Enable RM Link FSM to activate on next recieved PHY frame boundary
     *     DISABLE (0) = RM link disable
     *     ENABLE (1) = RM link enable
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
     */
    IQN2FL_CMD_AIL_PHY_RM_CFG_RX_EN_REG,

    /** AIL IQ EFE FRM SAMP TC CFG update for a symbol index of a radio standard
     *  Command to update the Radio Framing Sample Terminal Count Configuration Register
     *  Use hIqn2->arg_ail to select AIL instance
     *  (argument type: a pointer to an array of 3 uint32_t elements, radioStdId, symbol size and symbol index )
     */
    IQN2FL_CMD_AIL_UPDATE_EGRESS_RADSTD_TC_REG,

    /** TOP VC SYS STS CFG - VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     *  (argument type: Iqn2Fl_TopVCSwResetStbSetup* )
     */
    IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB,

    /** AID2 IQ EFE GLOBAL EN SET STB
     *  Set Global Enable for EFE.
     *  A write of any value to this register which sets (enables) global enable.
     */
    IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_SET,

    /** AID2 IQ EFE GLOBAL EN CLR STB
     *  Clear Global Enable for EFE.
     *  A write of any value to this register which clears (enables) global enable.
     */
    IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_CLEAR,

    /** AID2 IQ EFE CONFIG GROUP - EFE CFG - LOOPBACK EN
     *  EFE Configuration Register.
     *  (TI use Only) 0x1: Ingress data from ICC is looped back to Egress 
     *  data to ICC. DMA traffic is unused. (i.e. for purpose of early 
     *  DFE only testing)
     */
    IQN2FL_CMD_AID2_EFE_CONFIG_LOOPBACK_EN,

    /**
     * AID2 IQ EFE RADIO STANDARD SCHEDULER CONFIGURATION REGISTER LENGHT
     */
    IQN2FL_CMD_AID2_IQ_EFE_RAD_STD_SCH_CFG_LEN,

    /**
	 * AID2 IQ EFE RADIO STANDARD SCHEDULER CONFIGURATION REGISTER LENGHT
	 */
	IQN2FL_CMD_AID2_IQ_EFE_RAD_STD_SCH_CFG_EN,

    /** AID2 IQ IFE GLOBAL EN SET STB
     *  Set Global Enable for IFE.
     *  A write of any value to this register which sets (enables) global enable.
     */
    IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_SET,

    /** AID2 IQ IFE GLOBAL EN CLR STB
     *  Clear Global Enable for IFE.
     *  A write of any value to this register which clears (enables) global enable.
     */
    IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_CLEAR,

    /** AID2 IQ EFE CHAN CFG - CHANNEL ENABLE/DISABLE
     *  EFE DMA Channel Configuration Register.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG,

    /** AID2 IQ ECTL CHAN CFG - CHANNEL ENABLE/DISABLE
     *  ECTL DMA Channel Configuration Register.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AID2_ECTL_CHAN_CFG_CHAN_EN_REG,

    /** AID2 IQ IFE CHAN CFG - CHANNEL ENABLE/DISABLE
     *  IFE DMA Channel Configuration Register.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG,

    /** AID2 IQ ICTL CHAN CFG - CHANNEL ENABLE/DISABLE
     *  ECTL DMA Channel Configuration Register.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AID2_ICTL_CHAN_CFG_CHAN_EN_REG,

    /** AID2 IQ IDC CH CFG - CHANNEL FORCE OFF
     *  IDC Channel Configuration Register.
     *  (argument type: channel index of type uint8_t *)
     */
    IQN2FL_CMD_AID2_IDC_CH_CFG_CHAN_FRC_OFF_REG,

    /**  AID2 IQ EFE CHAN CFG - TDD CHANNEL FORCE OFF
     *  EFE DMA Channel Configuration Register.
     *  (argument type: channel index of type uint8_t *)
     */
    IQN2FL_CMD_AID2_EFE_CHAN_CFG_TDD_CHAN_FRC_OFF_REG,

    /**  AID2 IQ IFE CHAN CFG - TDD CHANNEL FORCE OFF
     *  IFE DMA Channel Configuration Register.
     *  (argument type: channel index of type uint8_t *)
     */
    IQN2FL_CMD_AID2_IFE_CHAN_CFG_TDD_CHAN_FRC_OFF_REG,

    /** AID2 ECTL GLOBAL EN SET STB
     *  Set Global Enable for ECTL.
     *  A write of any value to this register which sets (enables) global enable.
     */
    IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_SET,

    /** AID2 ECTL GLOBAL EN CLR STB
     *  Clear Global Enable for ECTL.
     *  A write of any value to this register which sets (enables) global enable.
     */
    IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_CLEAR,

    /** AID2 ICTL GLOBAL EN SET STB
     *  Set Global Enable for ICTL.
     *  A write of any value to this register which sets (enables) global enable.
     */
    IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_SET,

    /** AID2 ICTL GLOBAL EN CLR STB
     *  Clear Global Enable for ICTL.
     *  A write of any value to this register which sets (enables) global enable.
     */
    IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_CLEAR,

    /** AID2 ICTL RATE CTL CFG - ICTL Rate Control Configuration register
     *  Rate Controller will allow the ICTL to create RATE+1 active requests on the PSI bus within
     *  a 16 clock cycle window. As an example, a value of 7 will allow the ICTL to create 8 active
     *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
     *  (argument type: uint8_t *)
     */
    IQN2FL_CMD_AID2_ICTL_RATE_CTL_CFG_REG,

    /** AID2 ECTL RATE CTL CFG - ECTL Rate Control Configuration register
     *  Rate Controller will allow the ECTL to create RATE+1 active requests on the PSI bus within
     *  a 16 clock cycle window. As an example, a value of 7 will allow the ECTL to create 8 active
     *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
     *  (argument type: uint8_t *)
     */
    IQN2FL_CMD_AID2_ECTL_RATE_CTL_CFG_REG,

    /** AID2 IQ IDC RATE CTL CFG - IDC Rate Control Configuration register
     *  Rate Controller will allow the IDC to create RATE+1 active requests on the PSI bus within
     *  a 16 clock cycle window. As an example, a value of 7 will allow the IDC to create 8 active
     *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
     *  (argument type: uint8_t *)
     */
    IQN2FL_CMD_AID2_IQ_IDC_RATE_CTL_REG,

    /** AID2 UAT CFG - UAT_RUN and DIAG SYNC
     *  This register simply starts the uAT timers running. It is implied 
     *  that SW is unable to precisely time the start of timers. The 
     *  intent is for the SW to correct the timers by later writting to 
     *  the offset register of each timer.
     *   UAT run starts the BCN and RAD counters free running.
     *   diag_sync = 1 starts the BCN and RAD counters if uat_run is set 
     *     and an AT sync is received. This is only used in simulation and 
     *     for diagnostics.
     *  (argument type: Iqn2Fl_UatCfg* )
     */
    IQN2FL_CMD_AID2_UAT_GEN_CTL_UAT_CFG,

    /** AID2 UAT EGR RADT - OFFSET CFG
     *  UAT RADT offset Register.
     *  UAT RADT offset. Value which is added to the raw RADT as a timing 
     *  correction. RadT is initially randomly started, SW uses 
     *  radt_capture value to calculate offset correction factor. This 
     *  correction factor will be Frame size - captured value.
     *  Argument: Pointer to an array of 8 radt offset cfg values.
     */
    IQN2FL_CMD_AID2_UAT_EGR_RADT_OFFSET_CFG,

    /** AID2 UAT ING RADT - OFFSET CFG
     *  UAT RADT offset Register.
     *  UAT RADT offset. Value which is added to the raw RADT as a timing 
     *  correction. RadT is initially randomly started, SW uses 
     *  radt_capture value to calculate offset correction factor. This 
     *  correction factor will be Frame size - captured value.
     *  Argument: Pointer to an array of 8 radt offset cfg values.
     */
    IQN2FL_CMD_AID2_UAT_ING_RADT_OFFSET_CFG,

    /** AID2 UAT RADT EVT
     *  Configures UAT RADT event compare Register per RADT and 
     *  UAT RADT event clock counter terminal count Register per RADT.
     *  (argument type: Iqn2Fl_UatRadtEvtSetup* )
     */
    IQN2FL_CMD_AID2_UAT_RADT_EVT_PER_RADT,

    /** AID2 IQ EFE FRM SAMP TC CFG update for a symbol index of a radio standard
     *  Command to update the Radio Framing Sample Terminal Count Configuration Register
     *  (argument type: a pointer to an array of 3 uint32_t elements, radioStdId, symbol size and symbol index )
     */
    IQN2FL_CMD_AID2_UPDATE_EGRESS_RADSTD_TC_REG,

    /**
     * Sets egress PKTDMA destination channel.
     */
    IQN2FL_CMD_IQS_EGR_PKTDMA_CFG_CHAN,

    /**
	 * Sets egress DIO2 destination channel.
	 */
    IQN2FL_CMD_IQS_EGR_DIO2_CFG_CHAN,

    /**
	 * Sets ingress AID2 destination channel.
	 */
    IQN2FL_CMD_IQS_ING_AID2_AXC_LUT_CFG_CHAN,

    /**
	 * Sets ingress AID2 destination port.
	 */
    IQN2FL_CMD_IQS_ING_AID2_AXC_LUT_CFG_DEST,

    /** DIO2 IQ EFE GLOBAL EN SET STB
     *  Set Global Enable for EFE.
     *  A write of any value to this register which sets (enables) global enable.
     */
    IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_SET,

    /** DIO2 IQ EFE GLOBAL EN CLR STB
     *  Clear Global Enable for EFE.
     *  A write of any value to this register which clears (enables) global enable.
     */
    IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_CLEAR,

    /** DIO2 IQ IFE GLOBAL EN SET STB
     *  Set Global Enable for IFE.
     *  A write of any value to this register which sets (enables) global enable.
     */
    IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_SET,

    /** DIO2 IQ IFE GLOBAL EN CLR STB
     *  Clear Global Enable for IFE.
     *  A write of any value to this register which clears (enables) global enable.
     */
    IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_CLEAR,

    /** DIO2 IQ EFE CHAN CFG - CHANNEL ENABLE/DISABLE
     *  EFE DMA Channel Configuration Register.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_DIO2_EFE_CHAN_CFG_CHAN_EN_REG,

    /** DIO2 IQ IFE CHAN CFG - CHANNEL ENABLE/DISABLE
     *  IFE DMA Channel Configuration Register.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_DIO2_IFE_CHAN_CFG_CHAN_EN_REG,

    /** DIO2 IQ IDC CH CFG - CHANNEL FORCE OFF
     *  IDC Channel Configuration Register.
     *  (argument type: channel index of type uint8_t *)
     */
    IQN2FL_CMD_DIO2_IDC_CH_CFG_CHAN_FRC_OFF_REG,

    /**  DIO2 IQ EFE CHAN CFG - TDD CHANNEL FORCE OFF
     *  EFE DMA Channel Configuration Register.
     *  (argument type: channel index of type uint8_t *)
     */
    IQN2FL_CMD_DIO2_EFE_CHAN_CFG_TDD_CHAN_FRC_OFF_REG,

    /** DIO2 IQ IDC RATE CTL CFG - IDC Rate Control Configuration register
     *  Rate Controller will allow the IDC to create RATE+1 active requests on the PSI bus within
     *  a 16 clock cycle window. As an example, a value of 7 will allow the IDC to create 8 active
     *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
     *  (argument type: uint8_t *)
     */
    IQN2FL_CMD_DIO2_IQ_IDC_RATE_CTL_REG,

    /** DIO2 UAT CFG - UAT_RUN and DIAG SYNC
     *  This register simply starts the uAT timers running. It is implied 
     *  that SW is unable to precisely time the start of timers. The 
     *  intent is for the SW to correct the timers by later writting to 
     *  the offset register of each timer.
     *   UAT run starts the BCN and RAD counters free running.
     *   diag_sync = 1 starts the BCN and RAD counters if uat_run is set 
     *     and an AT sync is received. This is only used in simulation and 
     *     for diagnostics.
     *  (argument type: Iqn2Fl_UatCfg* )
     */
    IQN2FL_CMD_DIO2_UAT_GEN_CTL_UAT_CFG,

    /** DIO2 UAT RADT EVT
     *  Configures UAT RADT event compare Register per RADT and 
     *  UAT RADT event clock counter terminal count Register per RADT.
     *  (argument type: Iqn2Fl_UatRadtEvtSetup* )
     */
    IQN2FL_CMD_DIO2_UAT_RADT_EVT_PER_RADT,

    /** DIO2 UAT EGR RADT - OFFSET CFG
     *  UAT RADT offset Register.
     *  UAT RADT offset. Value which is added to the raw RADT as a timing 
     *  correction. RadT is initially randomly started, SW uses 
     *  radt_capture value to calculate offset correction factor. This 
     *  correction factor will be Frame size - captured value.
     *  (argument type: Iqn2Fl_RadtOffsetCfg* )
     */
    IQN2FL_CMD_DIO2_UAT_EGR_RADT_OFFSET_CFG,

    /** DIO2 UAT ING RADT - OFFSET CFG
     *  UAT RADT offset Register.
     *  UAT RADT offset. Value which is added to the raw RADT as a timing 
     *  correction. RadT is initially randomly started, SW uses 
     *  radt_capture value to calculate offset correction factor. This 
     *  correction factor will be Frame size - captured value.
     *  (argument type: Iqn2Fl_RadtOffsetCfg* )
     */
    IQN2FL_CMD_DIO2_UAT_ING_RADT_OFFSET_CFG,

    /** DIO2 UAT DIO EGR RADT - OFFSET CFG
     *  UAT DIO egress RADT offset Register.
     *  (DIO use only) UAT DIO egress RADT offset. Value which is added 
     *  to the raw RADT as a timing correction. RadT is initially 
     *  randomly started, SW uses radt_capture value to calculate 
     *  offset correction factor. This correction factor will be 
     *  Frame size - captured value.
     *  (argument type: Iqn2Fl_RadtOffsetCfg* )
     */
    IQN2FL_CMD_DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG,

    /** DIO2 UAT DIO ING RADT - OFFSET CFG
     *  UAT DIO ingress RADT offset Register.
     *  (DIO use only) UAT DIO ingress RADT offset. Value which is added 
     *  to the raw RADT as a timing correction. RadT is initially 
     *  randomly started, SW uses radt_capture value to calculate 
     *  offset correction factor. This correction factor will be 
     *  Frame size - captured value.
     *  (argument type: Iqn2Fl_RadtOffsetCfg* )
     */
    IQN2FL_CMD_DIO2_UAT_DIO_ING_RADT_OFFSET_CFG,

    /** DIO2 I GLOBAL EN SET STB
     *  Set Global Enable for Ingress DIO. 
     *  A write of any value to this register sets the global enable
     *  for the Ingress DIO.
     */
    IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_SET,

    /** DIO2 I GLOBAL EN CLR STB
     *  Clear Global Enable for Ingress DIO.
     *  A write of any value to this register which clears the global 
     *  enable for the Ingress DIO.
     */
    IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_CLEAR,

    /** DIO2 E GLOBAL EN SET STB
     *  Set Global Enable for Egress DIO. 
     *  A write of any value to this register sets the global enable
     *  for the Egress DIO.
     */
    IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_SET,

    /** DIO2 E GLOBAL EN CLR STB
     *  Clear Global Enable for Egress DIO.
     *  A write of any value to this register which clears the global 
     *  enable for the Egress DIO.
     */
    IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_CLEAR,

    /** DIO2 DT GLOBAL EN SET STB
     *  Set Global Enable for Data Trace DIO. 
     *  A write of any value to this register sets the global enable
     *  for the Data Trace DIO.
     */
    IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_SET,

    /** DIO2 DT GLOBAL EN CLR STB
     *  Clear Global Enable for Data Trace DIO.
     *  A write of any value to this register which clears the global 
     *  enable for the Data Trace DIO.
     */
    IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_CLEAR,

    /** DIO2 DT START STB
     *  DIO2 DIO Data Trace Start Register.
     *  A write of any value to this register will start Data Trace 
     *  operation if dt_en = 0.
     */
    IQN2FL_CMD_DIO2_DT_START,

    /** DIO2 CORE INGRESS
     *  Configures DIO2 CORE INGRESS registers
     *  (argument type: Iqn2Fl_Dio2CoreIngressSetup* )
     */
    IQN2FL_CMD_DIO2_CORE_INGRESS_CFG,

    /** DIO2 CORE EGRESS
     *  Configures DIO2 CORE EGRESS registers
     *  (argument type: Iqn2Fl_Dio2CoreEgressSetup* )
     */
    IQN2FL_CMD_DIO2_CORE_EGRESS_CFG,

    /** DIO2 I DBCNT0/1/2 RAM MMR
     *  Configures I DBCNT0/1/2 RAM MMR registers per engine basis
     *  (argument type: Iqn2Fl_Dio2DbcntxRamMmrCfg* )
     */
    IQN2FL_CMD_DIO2_I_DBCNT_RAM_MMR_CFG,

    /** DIO2 E DBCNT0/1/2 RAM MMR
     *  Configures E DBCNT0/1/2 RAM MMR registers per engine basis
     *  (argument type: Iqn2Fl_Dio2DbcntxRamMmrCfg* )
     */
    IQN2FL_CMD_DIO2_E_DBCNT_RAM_MMR_CFG,

    /** DIO2 CORE INGRESS DMA ENGINE enable/disable
     *  Command to only enable/disable dio ingress engines
     *  (argument type: a pointer to an array of 3 uint32_t elements, 0 for disable, 1 for enable )
     */
    IQN2FL_CMD_DIO2_CORE_INGRESS_DMA_ENGINE_EN,

    /** DIO2 CORE EGRESS DMA ENGINE enable/disable
     *  Command to only enable/disable dio egress engines
     *  (argument type: a pointer to an array of 3 uint32_t elements, 0 for disable, 1 for enable )
     */
    IQN2FL_CMD_DIO2_CORE_EGRESS_DMA_ENGINE_EN,

    /** DIO2 RECONFIGURE DMA INGRESS-EGRESS ENGINEs
     *  Command to reconfigure dio engines from RAC/TAC default paramteter
     *  (argument type: Iqn2Fl_Dio2ReconfigureEngineSetup* )
     */
    IQN2FL_CMD_DIO2_RECONFIGURE_ENGINE,

    /** DIO2 RECONFIGURE EGRESS ENGINE
     *  Reconfigures DIO2 Egress registers
     *  (argument type: Iqn2Fl_Dio2ReconfigureEgrEngineSetup* )
     */
    IQN2FL_CMD_DIO2_RECONFIGURE_EGRESS_ENGINE,

    /** DIO2 RECONFIGURE INGRESS ENGINE
     *  Reconfigures DIO2 Ingress registers
     *  (argument type: Iqn2Fl_Dio2ReconfigureIngrEngineSetup* )
     */
    IQN2FL_CMD_DIO2_RECONFIGURE_INGRESS_ENGINE,

    /** AT2 START - AT2 TIMER ENABLES CFG - RADT EN
     *  Control for each of the RADT and the BCN timers.
     *  bit 0-7 enable RadT0-to-RadT7. RADT 7 will be used for driving 
     *  the t1-t2-t3 GSM Timer. Once enabled, Timers will start running 
     *  once the compare value equals the BCN timer value (which yeilds 
     *  an exact sync).
     */
    IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN,

    /** AT2 START - AT2 TIMER ENABLES CFG - RUN BCN
     *  Control for each of the RADT and the BCN timers.
     *  SW control which starts the BCN Timer running. SW writes are not 
     *  precise so it is expected the user will correct the timer value 
     *  with an offset.
     */
    IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RUN_BCN,

    /** AT2 BCN - OFFSET CFG - VAL
     *  Offset for the free running raw BCN counter. The offset version 
     *  of BCN is the value used for all measurement and sync purposes. 
     *  (The offset mechanism gives a way to minimize clock domains 
     *  crossing errors when syncing timers). SW uses the desired sync 
     *  input capture status to calculate offset correction factor. 
     *  This correction factor will be Frame size - captured value.
     */
    IQN2FL_CMD_AT2_BCN_OFFSET_CFG_VAL,

    /** AT2 BCN - FRM INIT LSBS CFG - WR VAL
     *  BCN Frame count value (BTS Frame Number). Increments every time 
     *  10ms BCN timer wraps. This Register only allows writing of the 
     *  value (Part 2) User may write to this register while BCN is 
     *  running. Advisable to write mid frame to avoid wrap uncertaninty.
     *  BCN Frame Init LSBs.
     */
    IQN2FL_CMD_AT2_BCN_FRM_INIT_LSBS_CFG_WR_VAL,

    /** AT2 BCN - FRM INIT MSBS CFG - WR VAL
     *  BCN Frame count value (BTS Frame Number). Increments every time 
     *  10ms BCN timer wraps. This Register only allows writing of the 
     *  value (Part 1) User may write to this register while BCN is 
     *  running. 
     *  BCN Frame Init MSBs.
     */
    IQN2FL_CMD_AT2_BCN_FRM_INIT_MSBS_CFG_WR_VAL,

    /** AT2 RADT - INIT 1 CFG - FRM INIT LSB
     *  RADT Frame Init LSBs.
     *  RADT Frame Init LSBs loads a frame count directly to the counter 
     *  LSB bits. This register and the MSB register should only be loaded 
     *  when the RADT is off or if it is known it will not be incrementing 
     *  during the time of the writes.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
     IQN2FL_CMD_AT2_RADT_INIT_1_CFG_FRM_INIT_LSB,

     /** AT2 RADT - INIT 2 CFG - FRM INIT MSB
     *  RADT Frame Init MSBs.
     *  RADT Frame Init MSBs loads a frame count directly to the counter 
     *  MSB bits. This register and the LSB register should only be loaded 
     *  when the RADT is off or if it is known it will not be incrementing 
     *  during the time of the writes.
     *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
     */
    IQN2FL_CMD_AT2_RADT_INIT_2_CFG_FRM_INIT_MSB,

    /** AT2 EVENTS 24ARRAY registers configuration
     *  (argument type: Iqn2Fl_At2Events24Array* )
     */
    IQN2FL_CMD_AT2_EVENTS_24ARRAY_CFG,

    /** AT2 EVENTS - EVT ENABLE CFG - EN
     *  AT2 system events control APP SW timing. This MMR Enables each 
     *  of 24 of these system events.
     *  EVENT Enable when a bit is set to a 1. Must be enabled after 
     *  event configuration. Setting the bit to 0 disables the EVENT.
     */
    IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG,

    /** AT2 EVT FORCE STB
     *  For diagnostic purposes, this register allows the user to 
     *  activate any of the 24 system events with a simple registers 
     *  write. Useful for activating SW thread for testing purposes.
     *  EVENT Force.
     */
    IQN2FL_CMD_AT2_EVT_FORCE,
    /** EE_EOI_0_REG
     *  For exception handling purposes, this register allows the software to acknowledge the servicing of a
     *  handshake interrupt so that another EV0 interrupt can be generated.
     */
    IQN2FL_CMD_EE_EOI_0_REG,
    /** EE_EOI_1_REG
     *  For exception handling purposes, this register allows the software to acknowledge the servicing of a
     *  handshake interrupt so that another EV1 interrupt can be generated.
     */
    IQN2FL_CMD_EE_EOI_1_REG,
    /** EE_EOI_CPPI_REG
     *  For exception handling purposes, this register allows the software to acknowledge the servicing of a
     *  handshake interrupt so that another CPPI interrupt can be generated.
     */
    IQN2FL_CMD_EE_EOI_CPPI_REG,
    /** PSR_EE_ING_FLUSH_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears channel flush indication for ingress channels0 through 31.
     *  Argument: a pointer to Iqn2Fl_PsrEeIngFlushA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_PSR_EE_ING_FLUSH_A_REGSET,
    /** PSR_EE_ING_FLUSH_B_REGSET
     *  Sets, clears, enabled sets, or enabled clears channel flush indication for ingress channels0 through 31.
     *  Argument: a pointer to Iqn2Fl_PsrEeIngFlushA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_PSR_EE_ING_FLUSH_B_REGSET,
    /** PSR_EE_EGR_PROTOCOL_ERR_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears Protocol error indication of an unsupported data type or a missing PS_DATA transfer
     *  when one was expected for egress channels 0 through 31
     *  Argument: a pointer to Iqn2Fl_PsrEeEgressProtocolErrA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_A_REGSET,
    /** PSR_EE_EGR_PROTOCOL_ERR_B_REGSET
     *  Sets, clears, enabled sets, or enabled clears Protocol error indication of an unsupported data type or a missing PS_DATA transfer
     *  when one was expected for egress channels 32 through 47
     *  Argument: a pointer to Iqn2Fl_PsrEeEgressProtocolErrB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_B_REGSET,
    /** PKTDMA_EE_DESC_STARVE_REGSET
     *  Sets, clears, enabled sets, or enabled clears Descriptor starvation errors
     *  Argument: a pointer to Iqn2Fl_PktdmaEeDescStarve structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET,
    /** AT2_AT_EE_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears RP1 Errors and Synchronization input Info
     *  Argument: a pointer to Iqn2Fl_At2EeInfoErr structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AT2_AT_EE_0_REGSET,
    /** IQS_EE_CHAN_ERR_REGSET
     *  Sets, clears, enabled sets, or enabled clears IQS2 Channel error register
     *  Argument: a pointer to Iqn2Fl_Iqs2EeChanErr structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_IQS_EE_CHAN_ERR_REGSET,
    /** IQS_EE_ING_FLUSH_ERR_REGSET
     *  Sets, clears, enabled sets, or enabled clears IQS2 Flush error register
     *  Argument: a pointer to Iqn2Fl_Iqs2EeIngFlush structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_IQS_EE_ING_FLUSH_ERR_REGSET,
    /** IQS_EE_EGR_FLUSH_ERR_REGSET
     *  Sets, clears, enabled sets, or enabled clears IQS2 Flush error register
     *  Argument: a pointer to Iqn2Fl_Iqs2EeEgrFlush structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_IQS_EE_EGR_FLUSH_ERR_REGSET,
    /** AID2_EE_SII_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i IQ errors and info
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SII_A_REGSET,
    /** AID2_EE_SII_B_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i CTL errors and info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SII_B_REGSET,
    /** AID2_EE_SII_C_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i IQ per-channel start of frame errors.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiC structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SII_C_REGSET,
    /** AID2_EE_SII_D_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i CTL per-channel SOP received from ICC info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SII_D_REGSET,
    /** AID2_EE_SIE_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SIE_A_REGSET,
    /** AID2_EE_SIE_B_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e CTL info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SIE_B_REGSET,
    /** AID2_EE_SIE_C_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e CTL per-channel SOP transmitted to ICC.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieC structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SIE_C_REGSET,
    /** AID2_EE_DFE_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 DFE interrupts.
     *  Argument: a pointer to Iqn2Fl_Aid2EeDfe structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_DFE_REGSET,
    /** AID2_EE_SII_E_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i IQ info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SII_E_REGSET,
    /** AID2_EE_SII_F_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i CTL info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiF structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SII_F_REGSET,
    /** AID2_EE_SII_G_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i IQ per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiG structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SII_G_REGSET,
    /** AID2_EE_SII_H_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i CTL per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiH structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SII_H_REGSET,
    /** AID2_EE_SIE_D_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e IQ errors and info..
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SIE_D_REGSET,
    /** AID2_EE_SIE_E_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e CTL errors and info..
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SIE_E_REGSET,
    /** AID2_EE_SIE_F_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e IQ per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieF structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SIE_F_REGSET,
    /** AID2_EE_SIE_G_REGSET
     *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e CLT per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieG structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AID2_EE_SIE_G_REGSET,
    /** AIL_EE_SII_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ errors and info
     *  Argument: a pointer to Iqn2Fl_AilEeSiiA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SII_A_REGSET,
    /** AIL_EE_SII_B_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i CTL errors and info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SII_B_REGSET,
    /** AIL_EE_SII_C_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ per-channel start of frame errors.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiC0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SII_C_0_REGSET,
    /** AIL_EE_SII_C_1_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ per-channel start of frame errors.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiC1 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SII_C_1_REGSET,
    /** AIL_EE_SII_D_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i CTL per-channel SOP received from ICC info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SII_D_REGSET,
    /** AIL_EE_SII_E_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AIL_EE_SII_E_REGSET,
    /** AIL_EE_SII_F_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i CTL info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiF structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AIL_EE_SII_F_REGSET,
    /** AIL_EE_SII_G_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiG0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AIL_EE_SII_G_0_REGSET,
    /** AIL_EE_SII_G_1_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiG1 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_AIL_EE_SII_G_1_REGSET,
    /** AIL_EE_SII_H_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_i CTL per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiH structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SII_H_REGSET,
    /** AIL_EE_SIE_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SIE_A_REGSET,
    /** AIL_EE_SIE_B_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_e CTL info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SIE_B_REGSET,
    /** AIL_EE_SIE_C_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_e CTL per-channel SOP transmitted to ICC.
     *  Argument: a pointer to Iqn2Fl_AilEeSieC structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SIE_C_REGSET,
    /** AIL_EE_SIE_D_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_e IQ errors and info..
     *  Argument: a pointer to Iqn2Fl_AilEeSieD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SIE_D_REGSET,
    /** AIL_EE_SIE_E_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_e CTL errors and info..
     *  Argument: a pointer to Iqn2Fl_AilEeSieE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SIE_E_REGSET,
    /** AIL_EE_SIE_F_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_e IQ per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieF0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SIE_F_0_REGSET,
    /** AIL_EE_SIE_F_1_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_e IQ per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieF1 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SIE_F_1_REGSET,
    /** AIL_EE_SIE_G_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI si_e CLT per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieG structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SIE_G_REGSET,
    /** AIL_EE_RM_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL RM error info.
     *  Argument: a pointer to Iqn2Fl_AilEeRm0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_RM_0_REGSET,
    /** AIL_EE_RT_TM_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL RT and TM error info.
     *  Argument: a pointer to Iqn2Fl_AilEeRtTm0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_RT_TM_0_REGSET,
    /** AIL_EE_CI_CO_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL CI and CO error info.
     *  Argument: a pointer to Iqn2Fl_AilEeRtCiCo0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_CI_CO_0_REGSET,
    /** AIL_EE_PD_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL PD 0 error info.
     *  Argument: a pointer to Iqn2Fl_AilEePd0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_PD_0_REGSET,
    /** AIL_EE_PD_1_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL PD 1 error info.
     *  Argument: a pointer to Iqn2Fl_AilEePd1 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_PD_1_REGSET,
    /** AIL_EE_PE_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL PE 0 error info.
     *  Argument: a pointer to Iqn2Fl_AilEePe0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_PE_0_REGSET,
    /** AIL_EE_SI_0_REGSET
     *  Sets, clears, enabled sets, or enabled clears AIL SI 0 error info.
     *  Argument: a pointer to Iqn2Fl_AilEeSi0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     */
    IQN2FL_CMD_AIL_EE_SI_0_REGSET,
    /** DIO2_EE_DMA_ING_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO Ingress Interrupt info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeDmaIngA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_DMA_ING_A_REGSET,
    /** DIO2_EE_DMA_ING_B_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO Ingress Interrupt info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeDmaIngB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_DMA_ING_B_REGSET,
    /** DIO2_EE_DMA_EGR_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO Egress Interrupt info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeDmaEgrA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_DMA_EGR_A_REGSET,
    /** DIO2_EE_DT_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO DataTrace Interrupt info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeDtA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_DT_A_REGSET,
    /** DIO2_EE_SII_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_i IQ errors and info
     *  Argument: a pointer to Iqn2Fl_Dio2EeSiiA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_SII_A_REGSET,
    /** DIO2_EE_SII_C_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_i IQ per-channel start of frame errors.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSiiC structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_SII_C_REGSET,
    /** DIO2_EE_SII_E_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_i IQ info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSiiE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_SII_E_REGSET,
    /** DIO2_EE_SII_G_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_i IQ per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSiiG structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_SII_G_REGSET,
    /** DIO2_EE_SIE_A_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSieA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_SIE_A_REGSET,
    /** DIO2_EE_SIE_D_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_e IQ errors and info..
     *  Argument: a pointer to Iqn2Fl_Dio2EeSieD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_SIE_D_REGSET,
    /** DIO2_EE_SIE_F_REGSET
     *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_e IQ per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSieF structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     */
    IQN2FL_CMD_DIO2_EE_SIE_F_REGSET

} Iqn2Fl_HwControlCmd;

/**
 * @brief This is the set of query commands to get the status of various
 * operations in IQN2
 *
 * The arguments, if any, to be passed with each command are specified
 * next to that command. */                   
typedef enum 
{

    /** AT2_BCN_PA_TSCOMP_CAPT_STS
     *  CPTS module from NETCP/PA provides a measurement pulse 
     *  called pa_tscomp. When this pulse fires, the BCN Timer 
     *  value is captured to this register. SW may then use this 
     *  captured value to calculate a BCN timer error which is 
     *  corrected by writing to the BCN Offset register.
     *  Response: (uint32_t *) - BCN clock count pa_tscomp capture
     */
    IQN2FL_QUERY_AT2_BCN_PA_TSCOMP_CAPT_STS,

    /** AT2_BCN_PHYSYNC_CAPT_STS
     *  An external pin called PHYSYNC provides an optional sync 
     *  measurement source. When a rising edge is detected on this 
     *  pin, the BCN Timer value is captured to this register.
     *  Response: (uint32_t *) - BCN clock count physync capture
     */
    IQN2FL_QUERY_AT2_BCN_PHYSYNC_CAPT_STS,

    /** AT2_BCN_RADSYNC_CAPT_STS
     *  An external pin called RADSYNC provides an optional sync 
     *  measurement source. When a rising edge is detected on this 
     *  pin, the BCN Timer value is captured to this register.
     *  Response: (uint32_t *) - BCN clock count radsync capture
     */
    IQN2FL_QUERY_AT2_BCN_RADSYNC_CAPT_STS,

    /** AT2_BCN_RP1_SYNC_CAPT_STS
     *  OBSAI RP1 optionally may be used as a sync measurement source. 
     *  When a successful RP1 FCB is recieved, optionally a system 
     *  event fires, the FCB message is captured to registers, and 
     *  the BCN timer value is captured to this register.
     *  Response: (uint32_t *) - BCN clock count rp1_sync capture
     */
    IQN2FL_QUERY_AT2_BCN_RP1_SYNC_CAPT_STS,

    /** AT2_BCN_SLVSYNC_CAPT_STS
     *  The uAT will provide frame sync pulses back to the AT2. When this 
     *  pulse fires, the BCN Timer value is captured to this register. 
     *  SW may then use this captured value to calculate a BCN timer 
     *  error which is corrected by writing to the BCN Offset register.
     *  Response: (uint32_t *) - BCN clock count selected slave uAT sync capture. 
     *            bcn_slvsel_cfg selects which uAT slave sync is used 
     *            for capturing the BCN value in this register.
     */
    IQN2FL_QUERY_AT2_BCN_SLVSYNC_CAPT_STS,

    /** AT2_BCN_FRM_VALUE_LSB_STS
     *  BCN Frame count value (BTS Frame Number). Increments every time 
     *  10ms BCN timer wraps. This Register only allows reading of the 
     *  value (Part2).
     *  Response: (uint32_t *) - BCN Frame Value LSBs
     */
    IQN2FL_QUERY_AT2_BCN_FRM_VALUE_LSB_STS,

    /** AT2_BCN_FRM_VALUE_MSB_STS
     *  BCN Frame count value (BTS Frame Number). Increments every time 
     *  10ms BCN timer wraps. This Register only allows reading of the 
     *  value (Part1).
     *  Response: (uint32_t *) - BCN Frame Value MSBs
     */
    IQN2FL_QUERY_AT2_BCN_FRM_VALUE_MSB_STS,

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
    IQN2FL_QUERY_AT2_GSM_TCOUNT_VALUE_STS,

    /** AT2_RADT_0_STS, AT2_RADT_1_STS, AT2_RADT_2_STS, AT2_RADT_3_STS
     *  radt_samp_symb_value, RADT Frame Value 12 LSBs, radt_fr_value_msbs, 
     *  RADT_CHIP, RADT_SLOT and RADT_FRM.
     *  RADT is specific format which is expected by the RAC and TAC HW 
     *  accelerator modules. EDMA performs a read of these values each 
     *  RAC or TAC iteration cycle giving RAC and TAC a sense of real time.
     *  Argument: a pointer to @param (Iqn2Fl_At2RadtStatus)
     *  Response: @param (Iqn2Fl_At2RadtStatus *)
     *            For the "radt_num" provided by APP SW
     */
    IQN2FL_QUERY_AT2_RADT_STS,

    /** AT2 EVENTS - EVT ENABLE CFG - STATUS
     *  AT2 system events control APP SW timing. This MMR Enables each 
     *  of 24 of these system events.
     *  This query reads the value of this register. EVENT Enable when 
     *  the bit is set to a 1 and 0 means EVENT Disable.
     */
    IQN2FL_QUERY_AT2_EVENTS_ENABLE_STS,

    /** VC_CDMA_TSTAT_L_TDOWN_STS
     *  CDMA TX takedown status lsb Register.
     *  Response: (uint32_t *) CDMA TX takedown status lsb
     */
    IQN2FL_QUERY_VC_CDMA_TSTAT_L_TDOWN_STS,

    /** VC_CDMA_TSTAT_H_TDOWN_STS
     *  CDMA TX takedown status msb Register.
     *  Response: (uint32_t *) CDMA TX takedown status msb
     */
    IQN2FL_QUERY_VC_CDMA_TSTAT_H_TDOWN_STS,

    /** VC_CDMA_TSTAT_L_ENABLE_STS
     *  CDMA TX enable status lsb Register.
     *  Response: (uint32_t *) CDMA TX enable status lsb
     */
    IQN2FL_QUERY_VC_CDMA_TSTAT_L_ENABLE_STS,

    /** VC_CDMA_TSTAT_H_ENABLE_STS
     *  CDMA TX enable status msb Register.
     *  Response: (uint32_t *) CDMA TX enable status msb
     */
    IQN2FL_QUERY_VC_CDMA_TSTAT_H_ENABLE_STS,

    /** VC_CDMA_TSTAT_L_PKT_STS
     *  CDMA TX packet status lsb Register.
     *  Response: (uint32_t *) CDMA TX packet status lsb
     */
    IQN2FL_QUERY_VC_CDMA_TSTAT_L_PKT_STS,

    /** VC_CDMA_TSTAT_H_PKT_STS
     *  CDMA TX packet status msb Register.
     *  Response: (uint32_t *) CDMA TX packet status msb
     */
    IQN2FL_QUERY_VC_CDMA_TSTAT_H_PKT_STS,

    /** VC_CDMA_RSTAT_L_TDOWN_STS
     *  CDMA RX takedown status lsb Register.
     *  Response: (uint32_t *) CDMA RX takedown status lsb
     */
    IQN2FL_QUERY_VC_CDMA_RSTAT_L_TDOWN_STS,

    /** VC_CDMA_RSTAT_H_TDOWN_STS
     *  CDMA RX takedown status msb Register.
     *  Response: (uint32_t *) CDMA RX takedown status msb
     */
    IQN2FL_QUERY_VC_CDMA_RSTAT_H_TDOWN_STS,

    /** VC_CDMA_RSTAT_L_ENABLE_STS
     *  CDMA RX enable status lsb Register.
     *  Response: (uint32_t *) CDMA RX enable status lsb
     */
    IQN2FL_QUERY_VC_CDMA_RSTAT_L_ENABLE_STS,

    /** VC_CDMA_RSTAT_H_ENABLE_STS
     *  CDMA RX enable status msb Register.
     *  Response: (uint32_t *) CDMA RX enable status msb
     */
    IQN2FL_QUERY_VC_CDMA_RSTAT_H_ENABLE_STS,

    /** VC_CDMA_RSTAT_L_PKT_STS
     *  CDMA RX packet status lsb Register.
     *  Response: (uint32_t *) CDMA RX packet status lsb
     */
    IQN2FL_QUERY_VC_CDMA_RSTAT_L_PKT_STS,

    /** VC_CDMA_RSTAT_H_PKT_STS
     *  CDMA RX packet status msb Register.
     *  Response: (uint32_t *) CDMA RX packet status msb
     */
    IQN2FL_QUERY_VC_CDMA_RSTAT_H_PKT_STS,

    /** VC_SD_TX_STS
     *  SERDES transmit status Register
     *  Argument: a pointer to an array of 4 uint32_t elements
     *  Response: (uint32_t *) - Link clocks status via
     *                         response[0], response[1], 
     *                         response[2] and response[3]
     */
    IQN2FL_QUERY_VC_SD_TX_STS,

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
    IQN2FL_QUERY_VC_SD_RX_STS,

    /** AID2_IQ_EFE_CHAN_ON_STS
     *  Gives current On/Off Status of every available stream. One bit per 
     *  channel. Required because channels only turn on/off on radio frame 
     *  so the chan_en alone does not give channel status. Chan on/off is 
     *  not tracked for packet channels; These bits are 0 for packet 
     *  channels.
     *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
     */
    IQN2FL_QUERY_AID2_IQ_EFE_CHAN_ON_STS,

    /** AID2_IQ_EFE_IN_PKT_STS
     *  Gives current In/Out packet state of packet channels only. Bits 
     *  are always zero for AxC channels. Bit is activated at SOP, holds 
     *  high mid packet, deactivates at EOP.
     *  Response: (uint32_t *) - 0x1:IN_PKT, 0x0:OUT_PKT
     */
    IQN2FL_QUERY_AID2_IQ_EFE_IN_PKT_STS,

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
    IQN2FL_QUERY_AID2_IQ_EFE_DMA_SYNC_STS,

    /** AID2_IQ_IFE_CHAN_ON_STS
     *  Gives current On/Off Status of every available stream. One bit per 
     *  channel. Required because channels only turn on/off on radio frame 
     *  so the chan_en alone does not give channel status. Chan on/off is 
     *  not tracked for packet channels; These bits are 0 for packet 
     *  channels.
     *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
     */
    IQN2FL_QUERY_AID2_IQ_IFE_CHAN_ON_STS,

    /** AID2_IQ_IFE_IN_PKT_STS
     *  Gives current In/Out packet state of packet channels only. Bits 
     *  are always zero for AxC channels. Bit is activated at SOP, holds 
     *  high mid packet, deactivates at EOP.
     *  Response: (uint32_t *) - 0x1:IN_PKT, 0x0:OUT_PKT
     */
    IQN2FL_QUERY_AID2_IQ_IFE_IN_PKT_STS,

    /** AID2_IQ_IDC_STS
     *  IDC Status register.
     *  Empty indicator for both the PSI Staging FIFO and the CDC FIFO
     *     FIFO_NOT_EMPTY (00) = PSI Staging FIFO and CDC FIFO are not both empty
     *     FIFO_EMPTY (11) = PSI Staging FIFO and CDC FIFO are both empty
     *  Response: (uint32_t *) - Empty indicator as mentioned above
     */
    IQN2FL_QUERY_AID2_IQ_IDC_STS,

    /** AID2_IQ_IDC_INPKT_STS
     *  Per-channel in packet status bits where a 0 indicates that the 
     *  channel is not actively processing a packet and a 1 indicates 
     *  that it is actively processing a packet. The inpkt to channel 
     *  assignment is such that inpkt[0] is associated with channel 0 
     *  and inpkt[15] is associated with channel 15.
     *  Response: (uint32_t *) - In-packet status
     */
    IQN2FL_QUERY_AID2_IQ_IDC_INPKT_STS,

    /** AID2_ECTL_CHAN_ON_STS
     *  Gives current On/Off Status of every available stream. One bit per 
     *  channel. Required because channels only turn on/off on radio frame 
     *  so the chan_en alone does not give channel status. Chan on/off is 
     *  not tracked for packet channels; These bits are 0 for packet channels.
     *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
     */
    IQN2FL_QUERY_AID2_ECTL_CHAN_ON_STS,

    /** AID2_ECTL_INPKT_STS
     *  Indicates when a channel is actively receiving a packet from the ECTL.
     *  Per-channel in packet status bits where a 0 indicates that the 
     *  channel is not actively processing a packet and a 1 indicates 
     *  that it is actively processing a packet. The inpkt to channel 
     *  assignment is such that inpkt[0] is associated with channel 0 
     *  and inpkt[15] is associated with channel 15.
     *  Response: (uint32_t *) - In-packet status
     */
    IQN2FL_QUERY_AID2_ECTL_INPKT_STS,

    /** AID2_ICTL_CHAN_ON_STS
     *  Gives current On/Off Status of every available stream. One bit per 
     *  channel. Required because channels only turn on/off on radio frame 
     *  so the chan_en alone does not give channel status. Chan on/off is 
     *  not tracked for packet channels; These bits are 0 for packet channels.
     *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
     */
    IQN2FL_QUERY_AID2_ICTL_CHAN_ON_STS,

    /** AID2_ICTL_INPKT_STS
     *  Indicates when a channel is actively receiving a packet from the ICTL.
     *  Per-channel in packet status bits where a 0 indicates that the 
     *  channel is not actively processing a packet and a 1 indicates 
     *  that it is actively processing a packet. The inpkt to channel 
     *  assignment is such that inpkt[0] is associated with channel 0 
     *  and inpkt[15] is associated with channel 15.
     *  Response: (uint32_t *) - In-packet status
     */
    IQN2FL_QUERY_AID2_ICTL_INPKT_STS,

    /** AID2_UAT_SYNC_BCN_CAPTURE_STS
     *  UAT SYNC BCN capture Register.
     *  uAT raw BCN value captured each frame boundary of AT2 master BCN. 
     *  Used to calculate uAT BCN offset value for the purpose of 
     *  aligning uAT to AT2 BCN.
     *  Response: (uint32_t *) - UAT SYNC BCN Capture value
     */
    IQN2FL_QUERY_AID2_UAT_SYNC_BCN_CAPTURE_STS,

    /** AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS
     *  UAT SYNC RADT capture Register.
     *  UAT RADT sync capture captures the offset RADT count when a 
     *  uat_mst_slv_sync from the AT occurs. Used by SW to determine 
     *  correct RADT offset to apply.
     *  Argument: a pointer to an array of 8 uint32_t elements
     *  Response: (uint32_t *) - UAT EGR SYNC RADT Capture values
     *                         response[0] to response[7]
     */
    IQN2FL_QUERY_AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS,

    /** AID2_UAT_ING_SYNC_RADT_CAPTURE_STS
     *  UAT SYNC RADT capture Register.
     *  UAT RADT sync capture captures the offset RADT count when a 
     *  uat_mst_slv_sync from the AT occurs. Used by SW to determine 
     *  correct RADT offset to apply.
     *  Argument: a pointer to an array of 8 uint32_t elements
     *  Response: (uint32_t *) - UAT ING SYNC RADT Capture values
     *                         response[0] to response[7]
     */
    IQN2FL_QUERY_AID2_UAT_ING_SYNC_RADT_CAPTURE_STS,

    /** AID2_IQ_EDC_SOP_CNTR_STS
     *  Counts the number of SOPs seen by the IQ EDC.
     *  Response: (uint32_t *) - SOP count
     */
    IQN2FL_QUERY_AID2_IQ_EDC_SOP_CNTR_STS,

    /** AID2_IQ_EDC_EOP_CNTR_STS
     *  Counts the number of EOPs seen by the IQ EDC.
     *  Response: (uint32_t *) - EOP count
     */
    IQN2FL_QUERY_AID2_IQ_EDC_EOP_CNTR_STS,

    /** AID2_IQ_IDC_SOP_CNTR_STS
     *  This register provides a count of the Ingress SOPs sent on the PSI 
     *  to the IQN buffer or switch for activity monitoring.
     *  Response: (uint32_t *) - SOP count
     */
    IQN2FL_QUERY_AID2_IQ_IDC_SOP_CNTR_STS,

    /** AID2_IQ_IDC_EOP_CNTR_STS
     *  This register provides a count of the Ingress EOPs sent on the PSI 
     *  to the IQN buffer or switch for activity monitoring.
     *  Response: (uint32_t *) - EOP count
     */
    IQN2FL_QUERY_AID2_IQ_IDC_EOP_CNTR_STS,

    /** AID2_ECTL_SOP_CNTR_STS
     *  Counts the number of SOPs seen by the ECTL.
     *  Response: (uint32_t *) - SOP count
     */
    IQN2FL_QUERY_AID2_ECTL_SOP_CNTR_STS,

    /** AID2_ECTL_EOP_CNTR_STS
     *  Counts the number of EOPs seen by the ECTL.
     *  Response: (uint32_t *) - EOP count
     */
    IQN2FL_QUERY_AID2_ECTL_EOP_CNTR_STS,

    /** AID2_ICTL_SOP_CNTR_STS
     *  This register provides a count of the Ingress SOPs sent on the PSI 
     *  to the IQN buffer or switch for activity monitoring.
     *  Response: (uint32_t *) - Wrapping count of SOPs sent on PSI.
     */
    IQN2FL_QUERY_AID2_ICTL_SOP_CNTR_STS,

    /** AID2_ICTL_EOP_CNTR_STS
     *  This register provides a count of the Ingress EOPs sent on the PSI 
     *  to the IQN buffer or switch for activity monitoring.
     *  Response: (uint32_t *) - Wrapping count of EOPs sent on PSI.
     */
    IQN2FL_QUERY_AID2_ICTL_EOP_CNTR_STS,

    /** DIO2_IQ_EFE_CHAN_ON_STS
     *  Gives current On/Off Status of every available stream. One bit per 
     *  channel. Required because channels only turn on/off on radio frame 
     *  so the chan_en alone does not give channel status. Chan on/off is 
     *  not tracked for packet channels; These bits are 0 for packet 
     *  channels.
     *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
     */
    IQN2FL_QUERY_DIO2_IQ_EFE_CHAN_ON_STS,

    /** DIO2_IQ_EFE_IN_PKT_STS
     *  Gives current In/Out packet state of packet channels only. Bits 
     *  are always zero for AxC channels. Bit is activated at SOP, holds 
     *  high mid packet, deactivates at EOP.
     *  Response: (uint32_t *) - 0x1:IN_PKT, 0x0:OUT_PKT
     */
    IQN2FL_QUERY_DIO2_IQ_EFE_IN_PKT_STS,

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
    IQN2FL_QUERY_DIO2_IQ_EFE_DMA_SYNC_STS,

    /** DIO2_IQ_IFE_CHAN_ON_STS
     *  Gives current On/Off Status of every available stream. One bit per 
     *  channel. Required because channels only turn on/off on radio frame 
     *  so the chan_en alone does not give channel status. Chan on/off is 
     *  not tracked for packet channels; These bits are 0 for packet 
     *  channels.
     *  Response: (uint32_t *) - 0x1:CHAN_ON, 0x0:CHAN_OFF
     */
    IQN2FL_QUERY_DIO2_IQ_IFE_CHAN_ON_STS,

    /** DIO2_IQ_IFE_IN_PKT_STS
     *  Gives current In/Out packet state of packet channels only. Bits 
     *  are always zero for AxC channels. Bit is activated at SOP, holds 
     *  high mid packet, deactivates at EOP.
     *  Response: (uint32_t *) - 0x1:IN_PKT, 0x0:OUT_PKT
     */
    IQN2FL_QUERY_DIO2_IQ_IFE_IN_PKT_STS,

    /** DIO2_IQ_IDC_STS
     *  IDC Status register.
     *  Empty indicator for both the PSI Staging FIFO and the CDC FIFO
     *     FIFO_NOT_EMPTY (00) = PSI Staging FIFO and CDC FIFO are not both empty
     *     FIFO_EMPTY (11) = PSI Staging FIFO and CDC FIFO are both empty
     *  Response: (uint32_t *) - Empty indicator as mentioned above
     */
    IQN2FL_QUERY_DIO2_IQ_IDC_STS,

    /** DIO2_IQ_IDC_INPKT_STS
     *  Per-channel in packet status bits where a 0 indicates that the 
     *  channel is not actively processing a packet and a 1 indicates 
     *  that it is actively processing a packet. The inpkt to channel 
     *  assignment is such that inpkt[0] is associated with channel 0 
     *  and inpkt[15] is associated with channel 15.
     *  Response: (uint32_t *) - In-packet status
     */
    IQN2FL_QUERY_DIO2_IQ_IDC_INPKT_STS,

    /** DIO2_UAT_SYNC_BCN_CAPTURE_STS
     *  UAT SYNC BCN capture Register.
     *  uAT raw BCN value captured each frame boundary of AT2 master BCN. 
     *  Used to calculate uAT BCN offset value for the purpose of 
     *  aligning uAT to AT2 BCN.
     *  Response: (uint32_t *) - UAT SYNC BCN Capture value
     */
    IQN2FL_QUERY_DIO2_UAT_SYNC_BCN_CAPTURE_STS,

    /** DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS
     *  UAT SYNC RADT capture Register.
     *  UAT RADT sync capture captures the offset RADT count when a 
     *  uat_mst_slv_sync from the AT occurs. Used by SW to determine 
     *  correct RADT offset to apply.
     *  Argument: a pointer to an array of 8 uint32_t elements
     *  Response: (uint32_t *) - UAT EGR SYNC RADT Capture values
     *                         response[0] to response[7]
     */
    IQN2FL_QUERY_DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS,

    /** DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS
     *  UAT SYNC RADT capture Register.
     *  UAT RADT sync capture captures the offset RADT count when a 
     *  uat_mst_slv_sync from the AT occurs. Used by SW to determine 
     *  correct RADT offset to apply.
     *  Argument: a pointer to an array of 8 uint32_t elements
     *  Response: (uint32_t *) - UAT ING SYNC RADT Capture values
     *                         response[0] to response[7]
     */
    IQN2FL_QUERY_DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS,

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
    IQN2FL_QUERY_DIO2_UAT_DIO_EGR_SYNC_RADT_CAPTURE_STS,

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
    IQN2FL_QUERY_DIO2_UAT_DIO_ING_SYNC_RADT_CAPTURE_STS,

    /** DIO2_IQ_EDC_SOP_CNTR_STS
     *  Counts the number of SOPs seen by the IQ EDC.
     *  Response: (uint32_t *) - SOP count
     */
    IQN2FL_QUERY_DIO2_IQ_EDC_SOP_CNTR_STS,

    /** DIO2_IQ_EDC_EOP_CNTR_STS
     *  Counts the number of EOPs seen by the IQ EDC.
     *  Response: (uint32_t *) - EOP count
     */
    IQN2FL_QUERY_DIO2_IQ_EDC_EOP_CNTR_STS,

    /** DIO2_IQ_IDC_SOP_CNTR_STS
     *  This register provides a count of the Ingress SOPs sent on the PSI 
     *  to the IQN buffer or switch for activity monitoring.
     *  Response: (uint32_t *) - SOP count
     */
    IQN2FL_QUERY_DIO2_IQ_IDC_SOP_CNTR_STS,

    /** DIO2_IQ_IDC_EOP_CNTR_STS
     *  This register provides a count of the Ingress EOPs sent on the PSI 
     *  to the IQN buffer or switch for activity monitoring.
     *  Response: (uint32_t *) - EOP count
     */
    IQN2FL_QUERY_DIO2_IQ_IDC_EOP_CNTR_STS,
    /**  This register gives current On/Off Status of every available stream. One bit per channel. Required
     *  because channels only turn on/off on radio frame so the chan_en alone does not give
     *  channel status. Chan on/off is not tracked for packet channels; These bits are 0 for
     *  packet channels.
     *  Argument: a pointer to an array of 2 uint32_t elements
     *  Response: (uint32_t *) - 0x1: CHAN_ON 0x0:CHAN_OFF.
     *                         response[0] to response[1]
     */
    IQN2FL_QUERY_AIL_IQ_EFE_CHAN_ON_STS,
    /** AIL_IQ_EFE_IN_PKT_STS
     *  Gives current In/Out packet state of packet channels only. Bits are always zero for AxC
     *  channels. Bit is activated at SOP, holds high mid packet, deactivates at EOP.
     *  Argument: a pointer to an array of 2 uint32_t elements
     *  Response: (uint32_t *) - 0x1: IN_PKT 0x0:OUT_PKT.
     *                         response[0] to response[1]
     */
    IQN2FL_QUERY_AIL_IQ_EFE_IN_PKT_STS,
    /** AIL_IQ_EFE_DMA_SYNC_STS
     *  Gives current In/Out packet state of packet channels only. Bits are always zero for AxC
     *  channels. Bit is activated at SOP, holds high mid packet, deactivates at EOP
     *  Argument: a pointer to an array of 2 uint32_t elements
     *  Response: (uint32_t *) - 0x1: DMA synchronized to radio timing for this channel 0x0:DMA not synchronized to
     *  radio timing for this channel. Channel is in in re-sync mode.
     *                         response[0] to response[1]
     */
    IQN2FL_QUERY_AIL_IQ_EFE_DMA_SYNC_STS,
    /** AIL_IQ_PE_PHY_STS
     *  This register is PE DMA Channel 1 Register.
     *  SI Egress AIL scheduler, active high, indicating that the PE PHY is in ON state. PE PHY will
     *  only turn ON/OFF on PE_FB boundaries.
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_IQ_PE_PHY_STS,
    /** AIL_IQ_PE_CPRI_RADSTD_STS
     *  Gives each radio status reflecting enable and satisfying basic frame offset
     *  Argument: a pointer to an array of 8 uint32_t elements
     *  Response: (uint32_t *) - 0x1: RADSTD_ON 0x0:RADSTD_OFF.
     *                         response[0] to response[7]
     */
    IQN2FL_QUERY_AIL_IQ_PE_CPRI_RADSTD_STS,
    /** AIL_IQ_IFE_CHAN_ON_STS
     *  This register gives Gives current On/Off Status of every available stream. One bit per channel. Required
     *  because channels only turn on/off on radio frame so the chan_en alone does not give
     *  channel status. Chan on/off is not tracked for packet channels; These bits are 0 for packet channels.
     *  Argument: a pointer to an array of 2 uint32_t elements
     *  Response: (uint32_t *) - 0x1: CHAN_ON 0x0:CHAN_OFF.
     *                         response[0] to response[1]
     */
    IQN2FL_QUERY_AIL_IQ_IFE_CHAN_ON_STS,
    /** AIL_IQ_IFE_IN_PKT_STS
     *  Gives current In/Out packet state of packet channels only. Bits are always zero for AxC
     *  channels. Bit is activated at SOP, holds high mid packet, deactivates at EOP.
     *  Argument: a pointer to an array of 2 uint32_t elements
     *  Response: (uint32_t *) - 0x1: IN_PKT 0x0:OUT_PKT.
     *                         response[0] to response[1]
     */
    IQN2FL_QUERY_AIL_IQ_IFE_IN_PKT_STS,
    /** AIL_IQ_IDC_INPKT_STS
     *  Indicates when a channel is activelyreceiving a packet from the IFE.
     *  Argument: a pointer to an array of 2 uint32_t elements
     *  Response: (uint32_t *) - Per-channel in packet status bits where a 0 indicates that the channel is not actively
     *  processing a packet and a 1 indicates that it is actively processing a packet. The inpkt to
     *  channel assignment is such that inpkt[0] is associated with channel 0 and inpkt[15] is associated with channel 15
     *                         response[0] to response[1]
     */
    IQN2FL_QUERY_AIL_IQ_IDC_INPKT_STS,
    /** AIL_IQ_IDC_STS
     *  This register is IDC Status register.
     *  Empty indicator for both the PSI Staging FIFO and the CDC FIFO
     *  FIFO_NOT_EMPTY (00) = PSI Staging FIFO and CDC FIFO are not both empty
     *  FIFO_EMPTY (11) = PSI Staging FIFO and CDC FIFO are both empty
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_IQ_IDC_STS,
    /** AIL_ECTL_CHAN_ON_STS
     *  Gives current On/Off Status of every available stream. One bit per channel. Required
     *  because channels only turn on/off on radio frame so the chan_en alone does not give
     *  channel status. Chan on/off is not tracked for packet channels; These bits are 0 for
     *  packet channels
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_ECTL_CHAN_ON_STS,
    /** AIL_ECTL_INPKT_STS
     *  Indicates when a channel is activelyreceiving a packet from the ECTL
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_ECTL_INPKT_STS,
    /** AIL_ICTL_CHAN_ON_STS
     *  Gives current On/Off Status of every available stream. One bit per channel. Required
     *  because channels only turn on/off on radio frame so the chan_en alone does not give
     *  channel status. Chan on/off is not tracked for packet channels; These bits are 0 for
     *  packet channels.
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_ICTL_CHAN_ON_STS,
    /** AIL_ICTL_INPKT_STS
     *  Indicates when a channel is activelyreceiving a packet from the ICTL.
     *  Per-channel in packet status bits where a 0 indicates that the channel is not actively
     *  processing a packet and a 1 indicates that it is actively processing a packet. The inpkt to
     *  channel assignment is such that inpkt[0] is associated with channel 0 and inpkt[15] is
     *  associated with channel 15
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_ICTL_INPKT_STS,
    /** AIL_UAT_SYNC_BCN_CAPTURE_STS
     *  This register is UAT SYNC BCN capture Register
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_UAT_SYNC_BCN_CAPTURE_STS,
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
    IQN2FL_QUERY_AIL_UAT_PI_BCN_CAPTURE_STS,
    /** AIL_UAT_RP301_BCN_CAPTURE_STS
     *  This register is UAT rp301 BCN capture Register
     *  Response: (uint32_t *) - uAT RP3-01 BCN capture. Captures the BCN offset count when the phy
     *  detects an OBSAI RP3-01 strobe from the AIL_PD.
     */
    IQN2FL_QUERY_AIL_UAT_RP301_BCN_CAPTURE_STS,
    /** AIL_UAT_EGR_SYNC_RADT_CAPTURE_STS
     *  This is UAT SYNC RADT capture Register (Egress). UAT RADT sync capture captures the offset RADT count when a uat_mst_slv_sync from
     *  the AT occurs. Used by SW to determine correct RADT offset to apply.
     *  Argument: a pointer to an array of 8 uint32_t elements
     *  Response: (uint32_t *) - offset RADT count.
     *                         response[0] to response[7]
     */
    IQN2FL_QUERY_AIL_UAT_EGR_SYNC_RADT_CAPTURE_STS,
    /** AIL_UAT_ING_SYNC_RADT_CAPTURE_STS
     *  This is UAT SYNC RADT capture Register (Ingress). UAT RADT sync capture captures the offset RADT count when a uat_mst_slv_sync from
     *  the AT occurs. Used by SW to determine correct RADT offset to apply.
     *  Argument: a pointer to an array of 8 uint32_t elements
     *  Response: (uint32_t *) - offset RADT count.
     *                         response[0] to response[7]
     */
    IQN2FL_QUERY_AIL_UAT_ING_SYNC_RADT_CAPTURE_STS,
    /** AIL_IQ_EDC_SOP_CNTR_STS
     *  Counts the number of SOPs seen by the IQ EDC
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_IQ_EDC_SOP_CNTR_STS,
    /** AIL_IQ_EDC_EOP_CNTR_STS
     *  Counts the number of EOPs seen by the IQ EDC
     *  Response: (uint32_t *)
     */
    IQN2FL_QUERY_AIL_IQ_EDC_EOP_CNTR_STS,
    /** AIL_IQ_IDC_SOP_CNTR_STS
     *  This register provides a count of the Ingress SOPs sent on the PSI to the IQN buffer or
     *  switch for activity monitoring.
     *  Response: (uint32_t *) - Wrapping count of SOPs sent on PSI.
     */
    IQN2FL_QUERY_AIL_IQ_IDC_SOP_CNTR_STS,
    /** AIL_IQ_IDC_EOP_CNTR_STS
     *  This register provides a count of the Ingress EOPs sent on the PSI to the IQN buffer or
     *  switch for activity monitoring.
     *  Response: (uint32_t *) - Wrapping count of EOPs sent on PSI.
     */
    IQN2FL_QUERY_AIL_IQ_IDC_EOP_CNTR_STS,
    /** AIL_ICTL_SOP_CNTR_STS
     *  This register provides a count of the Ingress SOPs sent on the PSI to the IQN buffer or
     *  switch for activity monitoring.
     *  Response: (uint32_t *) - Wrapping count of SOPs sent on PSI.
     */
    IQN2FL_QUERY_AIL_ICTL_SOP_CNTR_STS,
    /** AIL_ICTL_EOP_CNTR_STS
     *  This register provides a count of the Ingress EOPs sent on the PSI to the IQN buffer or
     *  switch for activity monitoring.
     *  Response: (uint32_t *) - Wrapping count of EOPs sent on PSI.
     */
    IQN2FL_QUERY_AIL_ICTL_EOP_CNTR_STS,
    /** AIL_PE_CPRI_STS
     *  chan-by-chan packet status. When shutting down a link, it is good practice for APP SW
     *  to wait until all channels are out of packet
     *  Response: (uint32_t *) - Each bit indicates if the corresponding channel is active or not. A 1 indicates the channel is
     *  currently in the middle of a packet. A 0 indicatest the channel is out of packet.
     */
    IQN2FL_QUERY_AIL_PE_CPRI_STS,
    /** AIL_PD_OBSAI_RP3_01_STS
     *  RP3_01 FCB capture from received OBSAI msg. Capture is OBSAI Type triggered,
     *  Type LUT has capture control. Whole OBSAI qwd payload is captured to 4 wd MMRs
     *  Argument: a pointer to an array of 4 uint32_t elements
     *  Response: (uint32_t *) - one of four payload wd.
     *                         response[0] to response[3]
     */
    IQN2FL_QUERY_AIL_PD_OBSAI_RP3_01_STS,
    /** AIL_PHY_RT_DP_STS
     *  The AIL PHY RT Depth Status Register displaysthe real time depth of the Link Buffer
     *  in the RT Block
     *  Response: (uint32_t *) - The RT Depth status displays the operating depth of the RT link buffer.
     */
    IQN2FL_QUERY_AIL_PHY_RT_DP_STS,
    /** AIL_PHY_TM_STS
     *  The TM Status Register contains status of the TM block.
     *  Indicates that status of the Frame Sync state machine
     *  OFF (11) = FSM in OFF state
     *  IDLE (22) = FSM in IDLE state
     *  RE_SYNC (44) = FSM in RE_SYNC state
     *  FRAME_SYNC (88) = FSM in FRAME_SYNC state
     *  Response: (uint32_t *) - TM status.
     */
    IQN2FL_QUERY_AIL_PHY_TM_STS,
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
    IQN2FL_QUERY_AIL_PHY_RM_STS,
    /** AIL_PHY_RM_CPRI_HFN_STS
     *  Contains the last received CPRI HFN value.
     *  Response: (uint32_t *) - received CPRI HFN value.
     */
    IQN2FL_QUERY_AIL_PHY_RM_CPRI_HFN_STS,
    /** AIL_PHY_RM_CPRI_BFN_STS
     *  Contains the last received CPRI BFN value, high and low bytes
     *  Argument: a pointer to an array of 2 uint32_t elements
     *  Response: (uint32_t *) - response[0]: Received Node B Frame number low byte, Z.128.0
     *                         response[1]: Received Node B Frame number high byte, Z.130.0
     */
    IQN2FL_QUERY_AIL_PHY_RM_CPRI_BFN_STS,
    /** AIL_PHY_RM_CPRI_STATE_STS
     *  Defines the RM CPRI State Status
     *  Argument: a pointer to an array of 2 uint32_t elements
     *  Response: (uint32_t *) - response[0]: Active high status indicates when the receiber FSM is in the HFSYNC state ST3
     *                         response[1]: Active high status indicates Loss Of Frame when the receiver FSM is in state ST0 or ST1.
     *                         NOTE: The value of this bit will be 0 after reset but will change to a value of 1 if CPRI mode is enabled
     */
    IQN2FL_QUERY_AIL_PHY_RM_CPRI_STATE_STS,
    /** EE_EV0_ORGN_STS
     *  Reports the EE EV0 Origination Status Register into a Iqn2Fl_EeOrigin structure
     *  Argument: a pointer to Iqn2Fl_EeOrigin structure
     *  Response: (Iqn2Fl_EeOrigin *) - response
     */
    IQN2FL_QUERY_EE_EV0_ORGN_STS,
    /** EE_EV1_ORGN_STS
     *  Reports the EE EV1 Origination Status Register into a Iqn2Fl_EeOrigin structure
     *  Argument: a pointer to Iqn2Fl_EeOrigin structure
     *  Response: (Iqn2Fl_EeOrigin *) - response
     */
    IQN2FL_QUERY_EE_EV1_ORGN_STS,
    /** PSR_EE_ING_FLUSH_A_STS
     *  Reports Channel flush indication for ingress channels0 through 31. An ingress flush error indicates
     *  a transfer to this PKTDMA channel was attempted when the channel was full which triggers a flush of
     *  the rest of the packet
     *  Argument: a pointer to Iqn2Fl_PsrEeIngFlushA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_PsrEeIngFlushA *) - response
     */
    IQN2FL_QUERY_PSR_EE_ING_FLUSH_A_STS,
    /** PSR_EE_ING_FLUSH_B_STS
     *  Reports Channel flush indication for ingress channels 32 through 47. An ingress flush error indicates
     *  a transfer to this PKTDMA channel was attempted when the channel was full which triggers a flush of
     *  the rest of the packet
     *  Argument: a pointer to Iqn2Fl_PsrEeIngFlushB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_PsrEeIngFlushB *) - response
     */
    IQN2FL_QUERY_PSR_EE_ING_FLUSH_B_STS,
    /** EGR_PROTOCOL_ERR_A_STS
     *  Reports Protocol error indication of an unsupported data type or a missing PS_DATA transfer
     *  when one was expected for egress channels 0 through 31
     *  Argument: a pointer to Iqn2Fl_PsrEeEgressProtocolErrA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_PsrEeEgressProtocolErrA *) - response
     */
    IQN2FL_QUERY_PSR_EE_EGR_PROTOCOL_ERR_A_STS,
    /** EGR_PROTOCOL_ERR_B_STS
     *  Reports Protocol error indication of an unsupported data type or a missing PS_DATA transfer
     *  when one was expected for egress channels 32 through 47
     *  Argument: a pointer to Iqn2Fl_PsrEeEgressProtocolErrB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_PsrEeEgressProtocolErrB *) - response
     */
    IQN2FL_QUERY_PSR_EE_EGR_PROTOCOL_ERR_B_STS,
    /** PSR_EE_ORIG_REG
     *  Reports the EE PSR Origination Status Register into a Iqn2Fl_PsrEeOrigin structure
     *  Argument: a pointer to Iqn2Fl_PsrEeOrigin structure
     *  Response: (Iqn2Fl_PsrEeOrigin *) - response
     */
    IQN2FL_QUERY_PSR_EE_ORGN_STS,
    /** PKTDMA_DESC_STARVE_STS
     *  Reports PKTDMA SOP and MOP descriptor starvation errors
     *  Argument: a pointer to Iqn2Fl_PktdmaEeDescStarve structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_PktdmaEeDescStarve *) - response
     */
    IQN2FL_QUERY_PKTDMA_EE_PKTDMA_DESC_STARVE_STS,
    /** AT2_AT_EE_0_STS
     *  Reports RP1 Errors and Synchronization input Info.
     *  Argument: a pointer to Iqn2Fl_At2EeInfoErr structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_At2EeInfoErr *) - response
     */
    IQN2FL_QUERY_AT2_AT_EE_0_STS,
    /** IQS_EE_CHAN_ERR_STS
     *  Reports a transfer occurred for a non-existent destination port or channel.
     *  Argument: a pointer to Iqn2Fl_Iqs2EeChanErr structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Iqs2EeChanErr *) - response
     */
    IQN2FL_QUERY_IQS_EE_CHAN_ERR_STS,
    /** IQS_EE_ING_FLUSH_ERR_STS
     *  Reports  the DIO detected the need to flush or the transfer was to an
     *  IQS DIO channel that was full.
     *  Argument: a pointer to Iqn2Fl_Iqs2EeFlush structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Iqs2EeFlush *) - response
     */
    IQN2FL_QUERY_IQS_EE_ING_FLUSH_ERR_STS,
    /** IQS_EE_EGR_FLUSH_ERR_STS
     *  Reports  the DIO detected the need to flush or the transfer was to an
     *  IQS DIO channel that was full.
     *  Argument: a pointer to Iqn2Fl_Iqs2EeEgrFlush structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Iqs2EeEgrFlush *) - response
     */
    IQN2FL_QUERY_IQS_EE_EGR_FLUSH_ERR_STS,
    /** IQS2_EE_ORIG_REG
     *  Reports the EE IQS2 Origination Status Register into a Iqn2Fl_Iqs2EeOrigin structure
     *  Argument: a pointer to Iqn2Fl_Iqs2EeOrigin structure
     *  Response: (Iqn2Fl_Iqs2EeOrigin *) - response
     */
    IQN2FL_QUERY_IQS2_EE_ORGN_STS,
    /** AID2_EE_SII_A_STS
     *  Reports SI si_i IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSiiA *) - response
     */
    IQN2FL_QUERY_AID2_EE_SII_A_STS,
    /** AID2_EE_SII_B_STS
     *  Reports SI si_i CTL errors and info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSiiB *) - response
     */
    IQN2FL_QUERY_AID2_EE_SII_B_STS,
    /** AID2_EE_SII_C_STS
     *  Reports SI si_i IQ per-channel start of frame errors.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiC structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSiiC *) - response
     */
    IQN2FL_QUERY_AID2_EE_SII_C_STS,
    /** AID2_EE_SII_D_STS
     *  Reports SI si_i CTL per-channel SOP received from ICC info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSiiD *) - response
     */
    IQN2FL_QUERY_AID2_EE_SII_D_STS,
    /** AID2_EE_SIE_A_STS
     *  Reports SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSieA *) - response
     */
    IQN2FL_QUERY_AID2_EE_SIE_A_STS,
    /** AID2_EE_SIE_B_STS
     *  Reports SI si_e CTL info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSieB *) - response
     */
    IQN2FL_QUERY_AID2_EE_SIE_B_STS,
    /** AID2_EE_SIE_C_STS
     *  Reports SI si_e CTL per-channel SOP transmitted to ICC.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieC structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSieC *) - response
     */
    IQN2FL_QUERY_AID2_EE_SIE_C_STS,
    /** AID2_SYSCLK_ORIG_REG
     *  Reports the EE AID2 Origination Status Register into a Iqn2Fl_Aid2SysClkEeOrigin structure
     *  Argument: a pointer to Iqn2Fl_Aid2EeOrigin structure
     *  Response: (Iqn2Fl_Aid2SysClkEeOrigin *) - response
     */
    IQN2FL_QUERY_AID2_SYSCLK_EE_ORGN_STS,
    /** AID2_EE_DFE_STS
     *  Reports DFE interrupts.
     *  Argument: a pointer to Iqn2Fl_Aid2EeDfe structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeDfe *) - response
     */
    IQN2FL_QUERY_AID2_EE_DFE_STS,
    /** AID2_EE_SII_E_STS
     *  Reports SI si_i IQ info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSiiE *) - response
     */
    IQN2FL_QUERY_AID2_EE_SII_E_STS,
    /** AID2_EE_SII_F_STS
     *  Reports SI si_i CTL info..
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiF structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSiiF *) - response
     */
    IQN2FL_QUERY_AID2_EE_SII_F_STS,
    /** AID2_EE_SII_G_STS
     *  Reports SI si_i IQ per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiG structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSiiG *) - response
     */
    IQN2FL_QUERY_AID2_EE_SII_G_STS,
    /** AID2_EE_SII_H_STS
     *  Reports SI si_i CTL per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiH structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSiiH *) - response
     */
    IQN2FL_QUERY_AID2_EE_SII_H_STS,
    /** AID2_EE_SIE_D_STS
     *  Reports SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSieD *) - response
     */
    IQN2FL_QUERY_AID2_EE_SIE_D_STS,
    /** AID2_EE_SIE_E_STS
     *  Reports SI si_e CTL errors and info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSieE *) - response
     */
    IQN2FL_QUERY_AID2_EE_SIE_E_STS,
    /** AID2_EE_SIE_F_STS
     *  Reports SI si_e IQ per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieF structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSieF *) - response
     */
    IQN2FL_QUERY_AID2_EE_SIE_F_STS,
    /** AID2_EE_SIE_G_STS
     *  Reports SI si_e CTL per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSieG structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Aid2EeSieG *) - response
     */
    IQN2FL_QUERY_AID2_EE_SIE_G_STS,
    /** AID2_VBUSCLK_ORIG_REG
     *  Reports the EE AID2 VBUSCLK Origination Status Register into a Iqn2Fl_Aid2VbusClkEeOrigin structure
     *  Argument: a pointer to Iqn2Fl_Aid2VbusClkEeOrigin structure
     *  Response: (Iqn2Fl_Aid2VbusClkEeOrigin *) - response
     */
    IQN2FL_QUERY_AID2_VBUSCLK_EE_ORGN_STS,
    /** AIL_EE_SII_A_STS
     *  Reports SI si_i IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Aid2EeSiiA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_Aid2EeSiiA *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_A_STS,
    /** AIL_EE_SII_B_STS
     *  Reports SI si_i CTL errors and info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSiiB *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_B_STS,
    /** AIL_EE_SII_C_0_STS
     *  Reports SI si_i IQ per-channel start of frame errors.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiC0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSiiC0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_C_0_STS,
    /** AIL_EE_SII_C_1_STS
     *  Reports SI si_i IQ per-channel start of frame errors.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiC1 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSiiC1 *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_C_1_STS,
    /** AIL_EE_SII_D_STS
     *  Reports SI si_i CTL per-channel SOP received from ICC info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSiiD *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_D_STS,
    /** AIL_EE_SII_E_STS
     *  Reports SI si_i IQ info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSiiE *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_E_STS,
    /** AIL_EE_SII_F_STS
     *  Reports SI si_i CTL info..
     *  Argument: a pointer to Iqn2Fl_AilEeSiiF structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_AilEeSiiF *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_F_STS,
    /** AIL_EE_SII_G_0_STS
     *  Reports SI si_i IQ per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiG0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_AilEeSiiG0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_G_0_STS,
    /** AIL_EE_SII_G_1_STS
     *  Reports SI si_i IQ per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiG1 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_AilEeSiiG1 *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_G_1_STS,
    /** AIL_EE_SII_H_STS
     *  Reports SI si_i CTL per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSiiH structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_AilEeSiiH *) - response
     */
    IQN2FL_QUERY_AIL_EE_SII_H_STS,
    /** AIL_EE_SIE_A_STS
     *  Reports SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSieA *) - response
     */
    IQN2FL_QUERY_AIL_EE_SIE_A_STS,
    /** AIL_EE_SIE_B_STS
     *  Reports SI si_e CTL info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSieB *) - response
     */
    IQN2FL_QUERY_AIL_EE_SIE_B_STS,
    /** AIL_EE_SIE_C_STS
     *  Reports SI si_e CTL per-channel SOP transmitted to ICC.
     *  Argument: a pointer to Iqn2Fl_AilEeSieC structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSieC *) - response
     */
    IQN2FL_QUERY_AIL_EE_SIE_C_STS,
    /** AIL_EE_SIE_D_STS
     *  Reports SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_AilEeSieD *) - response
     */
    IQN2FL_QUERY_AIL_EE_SIE_D_STS,
    /** AIL_EE_SIE_E_STS
     *  Reports SI si_e CTL errors and info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSieE *) - response
     */
    IQN2FL_QUERY_AIL_EE_SIE_E_STS,
    /** AIL_EE_SIE_F_0_STS
     *  Reports SI si_e IQ per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieF0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSieF0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_SIE_F_0_STS,
    /** AIL_EE_SIE_F_1_STS
     *  Reports SI si_e IQ per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieF1 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSieF1 *) - response
     */
    IQN2FL_QUERY_AIL_EE_SIE_F_1_STS,
    /** AIL_EE_SIE_G_STS
     *  Reports SI si_e CTL per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_AilEeSieG structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSieG *) - response
     */
    IQN2FL_QUERY_AIL_EE_SIE_G_STS,
    /** AIL_VBUSCLK_ORIG_REG
     *  Reports the EE AIL VBUSCLK Origination Status Register into a Iqn2Fl_AilVbusClkEeOrigin structure
     *  Argument: a pointer to Iqn2Fl_AilVbusClkEeOrigin structure
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilVbusClkEeOrigin *) - response
     */
    IQN2FL_QUERY_AIL_VBUSCLK_EE_ORGN_STS,
    /** AIL_EE_RM_0_STS
     *  Reports AIL RM error info.
     *  Argument: a pointer to Iqn2Fl_AilEeRm0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeRm0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_RM_0_STS,
    /** AIL_EE_RT_TM_0_STS
     *  Reports AIL RT TM error info.
     *  Argument: a pointer to Iqn2Fl_AilEeRtTm0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeRtTm0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_RT_TM_0_STS,
    /** AIL_EE_CI_CO_0_STS
     *  Reports AIL CI CO error info.
     *  Argument: a pointer to Iqn2Fl_AilEeCiCo0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeCiCo0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_CI_CO_0_STS,
    /** AIL_SYSCLK_PHY_ORIG_REG
     *  Reports the EE AIL SYSCLK PHY Origination Status Register into a Iqn2Fl_AilSysClkPhyEeOrigin structure
     *  Argument: a pointer to Iqn2Fl_AilSysClkPhyEeOrigin structure
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilSysClkPhyEeOrigin *) - response
     */
    IQN2FL_QUERY_AIL_SYSCLK_PHY_EE_ORGN_STS,
    /** AIL_EE_PD_0_STS
     *  Reports AIL PD 0 error info.
     *  Argument: a pointer to Iqn2Fl_AilEePd0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEePd0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_PD_0_STS,
    /** AIL_EE_PD_1_STS
     *  Reports AIL PD 1 error info.
     *  Argument: a pointer to Iqn2Fl_AilEePd1 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEePd1 *) - response
     */
    IQN2FL_QUERY_AIL_EE_PD_1_STS,
    /** AIL_EE_PE_0_STS
     *  Reports AIL PE 0 error info.
     *  Argument: a pointer to Iqn2Fl_AilEePe0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEePe0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_PE_0_STS,
    /** AIL_EE_SI_0_STS
     *  Reports AIL SI 0 error info.
     *  Argument: a pointer to Iqn2Fl_AilEeSi0 structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilEeSi0 *) - response
     */
    IQN2FL_QUERY_AIL_EE_SI_0_STS,
    /** AIL_SYSCLK_ORIG_REG
     *  Reports the EE AIL SYSCLK Origination Status Register into a Iqn2Fl_AilSysClkEeOrigin structure
     *  Argument: a pointer to Iqn2Fl_AilSysClkEeOrigin structure
     *  use hIqn2->arg_ail to select the AIL instance
     *  Response: (Iqn2Fl_AilSysClkEeOrigin *) - response
     */
    IQN2FL_QUERY_AIL_SYSCLK_EE_ORGN_STS,
    /** DIO2_EE_DMA_ING_A_STS
     *  Reports DIO Ingress Interrupt info
     *  Argument: a pointer to Iqn2Fl_Dio2EeDmaIngA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeDmaIngA *) - response
     */
    IQN2FL_QUERY_DIO2_EE_DMA_ING_A_STS,
    /** DIO2_EE_DMA_ING_B_STS
     *  Reports DIO Ingress Interrupt info
     *  Argument: a pointer to Iqn2Fl_Dio2EeDmaIngB structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeDmaIngB *) - response
     */
    IQN2FL_QUERY_DIO2_EE_DMA_ING_B_STS,
    /** DIO2_EE_DMA_EGR_A_STS
     *  Reports DIO Egress Interrupt info
     *  Argument: a pointer to Iqn2Fl_Dio2EeDmaEgrA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeDmaEgrA *) - response
     */
    IQN2FL_QUERY_DIO2_EE_DMA_EGR_A_STS,
    /** DIO2_EE_DT_A_STS
     *  Reports DIO DIO DataTrace Interrupt info
     *  Argument: a pointer to Iqn2Fl_Dio2EeDtA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeDtA *) - response
     */
    IQN2FL_QUERY_DIO2_EE_DT_A_STS,
    /** DIO2_EE_SII_A_STS
     *  Reports SI si_i IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSiiA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeSiiA *) - response
     */
    IQN2FL_QUERY_DIO2_EE_SII_A_STS,
    /** DIO2_EE_SII_C_STS
     *  Reports SI si_i IQ per-channel start of frame errors.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSiiC structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeSiiC *) - response
     */
    IQN2FL_QUERY_DIO2_EE_SII_C_STS,
    /** DIO2_EE_SII_E_STS
     *  Reports SI si_i IQ info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSiiE structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeSiiE *) - response
     */
    IQN2FL_QUERY_DIO2_EE_SII_E_STS,
    /** DIO2_EE_SII_G_STS
     *  Reports SI si_i IQ per-channel SOP transmitted to PSI info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSiiG structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeSiiG *) - response
     */
    IQN2FL_QUERY_DIO2_EE_SII_G_STS,
    /** DIO2_EE_SIE_A_STS
     *  Reports SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSieA structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeSieA *) - response
     */
    IQN2FL_QUERY_DIO2_EE_SIE_A_STS,
    /** DIO2_EE_SIE_D_STS
     *  Reports SI si_e IQ errors and info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSieD structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeSieD *) - response
     */
    IQN2FL_QUERY_DIO2_EE_SIE_D_STS,
    /** DIO2_EE_SIE_F_STS
     *  Reports SI si_e IQ per-channel SOP received from PSI info.
     *  Argument: a pointer to Iqn2Fl_Dio2EeSieF structure
     *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
     *  Response: (Iqn2Fl_Dio2EeSieF *) - response
     */
    IQN2FL_QUERY_DIO2_EE_SIE_F_STS,
    /** DIO2_ORIG_REG
     *  Reports the EE DIO2 SYSCLK Origination Status Register into a Iqn2Fl_Dio2EeOrigin structure
     *  Argument: a pointer to Iqn2Fl_Dio2EeOrigin structure
     *  Response: (Iqn2Fl_Dio2EeOrigin *) - response
     */
    IQN2FL_QUERY_DIO2_EE_ORGN_STS
} Iqn2Fl_HwStatusQuery;



/**
 *
 * @brief select EE module working function
 *
 * Use this enumeration to specify one of the register sets of EE
 * It can be RAW, EV0_EN, or EV1_EN
 * */
typedef enum
{
   /** Selects ee interrupt raw status  */
   IQN2FL_EE_INT_RAW_STATUS = 0,
   /** Selects ee interrupt set  */
   IQN2FL_EE_INT_SET,
   /** Selects ee interrupt clear  */
   IQN2FL_EE_INT_CLR,
   /** Selects ee interrupt enabled status ev0  */
   IQN2FL_EE_INT_EN_STATUS_EV0,
   /** Selects ee interrupt enabled status ev1 */
   IQN2FL_EE_INT_EN_STATUS_EV1,
   /** Selects ee interrupt enable ev0  */
   IQN2FL_EE_INT_EN_EV0,
   /** Selects ee interrupt enable ev1*/
   IQN2FL_EE_INT_EN_EV1,
   /** Selects ee interrupt enable set ev0  */
   IQN2FL_EE_INT_EN_SET_EV0,
   /** Selects ee interrupt enable set ev1  */
   IQN2FL_EE_INT_EN_SET_EV1,
   /** Selects ee interrupt enable clear ev0  */
   IQN2FL_EE_INT_EN_CLR_EV0,
   /** Selects ee interrupt enable clear ev1  */
   IQN2FL_EE_INT_EN_CLR_EV1

} Iqn2Fl_EeArgIndex;

/**
@} */



/** @addtogroup IQN2FL_DATASTRUCT
*
* @{ */

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/

/**
 * @brief pointer to the csl iqn2 register global structure
 * */
typedef volatile CSL_Iqn2Regs *Iqn2Fl_RegsOvly;


/**
 * @brief This structure is used to pass multiple arguments for a command
 *        in @a Iqn2Fl_HwControl function. For example, to configure a
 *        specific register among array of registers both the register
 *        index and the value are required. 
 */
typedef struct 
{
    /** Index of register to be configured */
    uint32_t   reg_index;

    /** Value to be set - argument */
    void   *reg_arg;

} Iqn2Fl_HwControlMultiArgs;


/**
 * @brief This is a sub-structure used for configuring the uAT timers.
 * This is used by AIL, AID2 and DIO2.
 */
typedef struct 
{
    /** AID2 UAT CFG - UAT_RUN
     *  This register simply starts the uAT timers running. It is implied 
     *  that SW is unable to precisely time the start of timers. The 
     *  intent is for the SW to correct the timers by later writting to 
     *  the offset register of each timer.
     *  UAT run starts the BCN and RAD counters free running.
     */
    uint8_t  uat_run;

    /** AID2 UAT CFG - DIAG_SYNC
     *  This register simply starts the uAT timers running. It is implied 
     *  that SW is unable to precisely time the start of timers. The 
     *  intent is for the SW to correct the timers by later writting to 
     *  the offset register of each timer.
     *  diag_sync = 1 starts the BCN and RAD counters if uat_run is set 
     *  and an AT sync is received. This is only used in simulation and 
     *  for diagnostics.
     */
    uint8_t  diag_sync;

} Iqn2Fl_UatCfg;


/**
 * @brief This structure is used to configure radio timer offsets by 
 * different IQN2 sub-blocks viz. AID2, DIO2 and AIL2. The configuration
 * is per radio standard.
 */
typedef struct 
{
    /** Radio Standard
     *  Valid values range from 0 to 7. Refer @a Iqn2Fl_ChanRadioSel
     *  APP SW needs to provide this value before calling the control
     *  function.
     */
    uint8_t  radio_std;

    /** RADT OFFSET CFG
     *  UAT RADT offset Register.
     *  UAT RADT offset. Value which is added to the raw RADT as a timing 
     *  correction. RadT is initially randomly started, SW uses 
     *  radt_capture value to calculate offset correction factor. This 
     *  correction factor will be Frame size - captured value.
     */
    uint32_t  offset;

} Iqn2Fl_RadtOffsetCfg;


/**
 * @brief This structure is used to get status of AT2 RADT sample,  
 * symbol, frame LSB, frame MSB, chip, slot and frame values for a 
 * given RADT number.
 */
typedef struct 
{
    /** Radio Timer Number
     *  Valid values range from 0 to 7. APP SW needs to provide this
     *  value before calling the status function. 
     */
    uint8_t  radt_num;

    /** RADT sample count Value. Increments every radt_0_cfg clkcnt_tc +1 
     *  system clock cycles.
     */
    uint32_t  radt_sampcnt_val;

    /** RADT symbol count Value
     */
    uint32_t  radt_symcnt_val;

    /** RADT Frame Value 12 LSBs
     */
    uint32_t  radt_frm_val_lsb;

    /** RADT Frame Value MSBs
     */
    uint32_t  radt_frm_val_msb;

    /** Chip Value
     */
    uint32_t  radt_chip;

    /** Slot Value
     */
    uint32_t  radt_slot;

    /** Frame Value
     */
    uint32_t  radt_frm;

} Iqn2Fl_At2RadtStatus;


/**
 * @brief This is a sub-structure used by @a Iqn2Fl_AilSiIqEfeRadStdGrp,
 * @a Iqn2Fl_AilIqIfeRadStdGrp and @a . This structure is used
 * for configuring the Radio Standard Frame Count Register for 
 * EFE, IFE and PD OBSAI.
 */
typedef struct 
{
    /** Symbol Count. Number of symbols per frame programmed as a Terminal Count */
    uint8_t   sym_tc;

    /** Index Counter Starting Location */
    uint8_t   index_sc;

    /** Index Counter Terminal Count */
    uint8_t   index_tc;

} Iqn2Fl_AilIqFrmTcCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilSiIqEfeCfgGrp. This structure is used
 * for configuring the Egress Framing Engine (EFE) channel configuration */
typedef struct 
{
    /** Enable channel: ENABLED(1), DISABLED(0) */
    uint8_t   chan_en;

    /** Selects OBSAI channel as Control when high, OBSAI channel as AxC when low */
    Iqn2Fl_ChanObsaiCtl   chan_obsai_ctl;

    /** Selects ENET mode when high and non-ENET mode when low */
    Iqn2Fl_ChanEnetCtlMode   chan_enet_ctl;

    /** Selects sample in a QSD that would be the start of the frame */
    uint8_t   axc_fine_offset;

    /** Alternate TDD mode for controlling TDD */
    Iqn2Fl_ChanFrcOff   chan_tdd_frc_off;

    /** Assigns each channel to one of eight radio standard
     *  groups i.e. radio standard 0 may be LTE 20MHz
     */
    Iqn2Fl_ChanRadioSel   chan_radio_sel;

} Iqn2Fl_AilIqEfeChanCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * Egress Framing Engine (EFE) configuration group parameters of an AIL instance */
typedef struct 
{
    /** AIL SI IQ EFE CHANNEL CONFIGURATION */
    Iqn2Fl_AilIqEfeChanCfg   ail_si_iq_efe_chan_config[64];

    /** AIL IQ EFE CONFIG 
     *  Enables loopback when set.
     */
    uint32_t   lpbk_en;

} Iqn2Fl_AilSiIqEfeCfgGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilSiIqEfeRadStdGrp. This structure is used
 * for configuring the Egress Framing Engine (EFE) Radio Standard Config Register */
typedef struct 
{
    /** Selects first symbol to start TDD */
    uint8_t   tdd_first_sym;

    /** Enable TDD. 
     *  1: TDD enabled for this radio standard 
     *  0: TDD disabled for this radio standard 
     */
    uint8_t   tdd_lut_en;

    /** Enables Base-band Hopping mode
     *  1: GSM Baseband hopping enabled for this radio standard
     *  0: GSM Baseband hopping disabled for this radio standard
     */
    uint8_t   gsm_axc_bbhop_mode;

    /** Enables GSM Compresed mode
     *  1: GSM Compression enabled for this radio standard
     *  0: GSM Compression disabled for this radio standard
     */
    uint8_t   gsm_cmp_mode;

} Iqn2Fl_AilIqEfeRadStdCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * Egress Framing Engine (EFE) radio standard group parameters of an AIL instance */
typedef struct 
{
    /** AIL IQ EFE FRM TC CFG */
    Iqn2Fl_AilIqFrmTcCfg   ail_iq_efe_frm_tc_cfg[8];

    /** AIL IQ EFE RAD STD CFG */
    Iqn2Fl_AilIqEfeRadStdCfg   ail_iq_efe_rad_std_cfg[8];

    /** AIL IQ EFE RADIO STANDARD 0 TDD ENABLE LUT */
    uint32_t ail_iq_efe_tdd_en_cfg0[5];

    /** AIL IQ EFE RADIO STANDARD 1 TDD ENABLE LUT */
    uint32_t ail_iq_efe_tdd_en_cfg1[5];

    /** AIL IQ EFE RADIO STANDARD 2 TDD ENABLE LUT */
    uint32_t ail_iq_efe_tdd_en_cfg2[5];

    /** AIL IQ EFE RADIO STANDARD 3 TDD ENABLE LUT */
    uint32_t ail_iq_efe_tdd_en_cfg3[5];

    /** AIL IQ EFE RADIO STANDARD 4 TDD ENABLE LUT */
    uint32_t ail_iq_efe_tdd_en_cfg4[5];

    /** AIL IQ EFE RADIO STANDARD 5 TDD ENABLE LUT */
    uint32_t ail_iq_efe_tdd_en_cfg5[5];

    /** AIL IQ EFE RADIO STANDARD 6 TDD ENABLE LUT */
    uint32_t ail_iq_efe_tdd_en_cfg6[5];

    /** AIL IQ EFE RADIO STANDARD 7 TDD ENABLE LUT */
    uint32_t ail_iq_efe_tdd_en_cfg7[5];

} Iqn2Fl_AilSiIqEfeRadStdGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * configuring Egress AXC TDM Look Up Table RAM of an AIL instance */
typedef struct 
{
    /** List of AxC indexes giving TDM AxC order over CPRI or OBSAI link */
    uint8_t   axc;

    /** Enables each entry. Disabled entries will result in zeroed CPRI containers. */
    uint8_t   en;

} Iqn2Fl_AilSiIqEgrTdmLutRam;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * configuring Egress AXC TDM Look Up Table RAM of an AIL instance */
typedef struct 
{
    /** Modulo Terminal count of the rule counter. Dictates the period 
     *  of the rule. Terminal count is the OBSAI Module minus 1. 
     */
    uint16_t   rule_mod;

    /** Transmission Rule Enable */
    uint8_t   rule_en;

    /** 1: Module Rule oppeations on OBSAI control messages, 0: AxC messages */
    uint8_t   rule_ctl_msg;

    /** Index. Transmission Rule. Index indicates at which count 
     *  the rule will fire. 
     */
    uint16_t   rule_index;

} Iqn2Fl_AilIqPeObsaiModtxruleCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * configuring Egress OBSAI Dual Bit Map RULE RAM of an AIL instance */
typedef struct 
{
    /** DBMF Enable: When disabled and MOD rule fires, first AxC LUT 
     *  entry is used and other DBM programming is disregarded.
     */
    uint8_t   dbm_en;

    /** OBSAI only: Radio Standard 0-7 corresponding to the given rule. 
     *  Controls which RADT to be used for AxC_Offset comparison. Highly 
     *  recommended to use a single rule per radio standard to simplify 
     *  AxC_Offset calculations. 
     */
    uint8_t   dbm_radstd;

    /** DBMF max number of supported channels */
    uint8_t   dbm_x;

    /** DBMF repititions of map1 */
    uint8_t   dbm_1mult;

    /** DBMF number of bits of bit map1 to use. */
    uint8_t   dbm_1size;

    /** DBMF number of bits of bit map2 to use. */
    uint8_t   dbm_2size;

    /** DBMF Starting address for chan TDM LUT pe_rule_chanlut. 
     *  LUT is shared by all 32 DBM rules. Starting address
     *  indicates where pattern for this rule starts.
     */
    uint8_t   dbm_lutstrt;

    /** Normally programmed to 0 corresponding to 1 bubble 
     *  when a bubble is indicated by Map1 or Map2 pattern. 
     *  Burst length indicated by this configuration. 
     */
    uint8_t   dbm_xbubble;

} Iqn2Fl_AilSiIqEgrObsaiDbmRule;


/**
 * @brief This is a sub-structure used in @a Iqn2Fl_AilSiIqEgrSchCpri.
 * This is used for configuring AIL PE CPRI BUB FSM and FSM2 CFG
 * configuration parameters of an AIL instance */
typedef struct 
{
    /** AIL IQ PE CPRI BUB FSM CFG
     *  one-to-one mapping to each radio standard, CPRI Mapping Method 3, 
     *  Bubble insertion state machine control, total of 8 separate FSMs, 
     *  one per 8 groups/radio_standards
     *  (K x Nc) total number of containers per AxC Container Block. 0=one 
     */
    uint32_t   knc[8];

    /** AIL IQ PE CPRI BUB FSM2 CFG
     *  One-to-one mapping to each radio standard, CPRI Mapping Method 3, 
     *  Bubble insertion state machine control, total of 8 separate FSMs, 
     *  one per 8 groups/radio_standards. If there are any bubbles present, 
     *  bub_gap indicates space between bubbles. Program BUB_GAP_INT .GE. 
     *  KNC for no bubbles.
     *  fsm-by-fsm, fractional number of containers between bubbles. 0=zero. 
     *  Value is accumulated every bubble, in this way, a non-zero fraction 
     *  eventually accumulates into the integer field. 
     */
    uint16_t   gap_frac[8];

    /** AIL IQ PE CPRI BUB FSM2 CFG
     *  One-to-one mapping to each radio standard, CPRI Mapping Method 3, 
     *  Bubble insertion state machine control, total of 8 separate FSMs, 
     *  one per 8 groups/radio_standards. If there are any bubbles present, 
     *  bub_gap indicates space between bubbles. Program BUB_GAP_INT .GE. 
     *  KNC for no bubbles.
     *  fsm-by-fsm, integer number of containers between bubbles. 0=one 
     *  which indicates every clock would be a bubble, assuming bub_gap_frac 
     *  is also zero. Algorithm rewinds at the beginning of each AxC Container Block. 
     */
    uint32_t   gap_int[8];

} Iqn2Fl_AilSiIqESchCpriBubFsmCfg;


/**
 * @brief This is a sub-structure used in @a Iqn2Fl_AilSiIqEgrSchCpri.
 * This is used for configuring AIL PE CPRI TDM FSM CFG
 * configuration parameters of an AIL instance */
typedef struct 
{
    /** Number of AxC_LUT entries for this FSM (For each LTE20, 8 sequential 
     *  entries within AxC_LUT). 
     */
    uint8_t   ncont[8];

    /** First AxC_LUT entry for this fsm. Effectively controls how the table 
     *  is shared betweeen up to 8 groups or fsm. 
     */
    uint8_t   lutstrt[8];

} Iqn2Fl_AilSiIqESchCpriTdmFsmCfg;


/**
 * @brief This is a sub-structure used in @a Iqn2Fl_AilSiIqEgrSchCpri and
 * @a Iqn2Fl_AilPdCpriAxcCfg. This is used for configuring AIL PE/PD CPRI
 * RADSTD configuration parameters of an AIL instance */
typedef struct 
{
    /** AIL PE/PD CPRI RADSTD CFG
     *  Enables each individual radio standard for 
     *  the PD for up to 8 radio standards.
     *  Enable bit for the corresponding radio standard
     */
    uint32_t   radstd_cfg_en[8];

    /** AIL PE/PD CPRI RADSTD1 CFG
     *  Defines the Radio Standard Offset per radio standard.
     *  Basic Frame Index of Radio Standard Offset
     */
    uint32_t   radstd1_cfg_bfrm_offset[8];

    /** AIL PE/PD CPRI RADSTD1 CFG
     *  Defines the Radio Standard Offset per radio standard.
     *  Hyper Frame Index of Radio Standard Offset
     */
    uint32_t   radstd1_cfg_hfrm_offset[8];

    /** AIL PE/PD CPRI RADSTD2 CFG
     *  Defines the number of Basic Frames per radio standard.
     *  Number of Basic Frames in a Radio Standard
     */
    uint32_t   radstd2_cfg_bfrm_num[8];

} Iqn2Fl_AilSiIqCpriRadstdCfg;


/**
 * @brief This is a sub-structure used in @a Iqn2Fl_AilSiIqEgrSchCpri.
 * This is used for configuring AIL PE CPRI CONT CFG
 * configuration parameters of an AIL instance */
typedef struct 
{
    /** Group (radio standard) which container belongs to, 0-to-7. When
     *  programming must exactly match the AIL_PHY_CO programmed operation. 
     */
    uint8_t   lut_grp[64];

    /** Enable or Disable of each contain, even if disabled, CONT_LUT_GRP 
     *  must be programmed correctly. 
     *  CONT_DIS (0) = container is unused by PE. Later AxC LUT entry will be 
     *                 unused. PHY will be driven with zero for this container.
     *  CONT_EN (1)  = container is mapped by cont_lut_grp.
     */
    uint8_t   lut_en[64];

} Iqn2Fl_AilSiIqESchCpriContCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * configuring Egress SCH CPRI of an AIL instance */
typedef struct 
{
    /** CPRI, indicating how many CPRI AxC containers are supported within 
     *  each basic frame. Value must be consistently programmed with AIL_PHY_Egress
     *  programmed values or erroneous operation will result. Programmed values 
     *  0-to-63 correspond to length 1-to-64. Circuit using this value, sample and 
     *  hold the value for each PHY frame allowing SW to change the value between frames.
     */
    uint8_t   axc_cont_tc;

    /** AIL IQ PE CPRI BUB FSM/FSM2 CFG
     */
    Iqn2Fl_AilSiIqESchCpriBubFsmCfg   ail_iq_pe_cpri_bub_fsm_cfg;

    /** AIL IQ PE CPRI TDM FSM CFG
     *  One-to-one mapping to each radio standard, CPRI AxC TDM. Intended use 
     *  is one TDM FSM per radio standard (matching AIL_PHY_CI groups)
     */
    Iqn2Fl_AilSiIqESchCpriTdmFsmCfg   ail_iq_pe_cpri_tdm_fsm_cfg;

    /** AIL PE CPRI RADSTD CFG
     *  Enables and configures each individual radio standard for the PE 
     *  for up to 8 radio standards 
     */
    Iqn2Fl_AilSiIqCpriRadstdCfg   ail_iq_pe_cpri_radstd_cfg;

    /** AIL IQ PE CPRI CONT CFG
     *  PD CPRI PHY Container LUT: Maps CPRI containers within a basic frame 
     *  to 1 of 8 groups (radio standard). LUT replays each basic frame. 
     *  Only CPRI16x can use full 64 depth of LUT.
     */
    Iqn2Fl_AilSiIqESchCpriContCfg   ail_iq_pe_cpri_cont_cfg;

} Iqn2Fl_AilSiIqEgrSchCpri;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * configuring ECTL PKT IF of an AIL instance */
typedef struct 
{
    /** Enable channel. 1:Enabled, 0:Disabled */
    uint8_t   chan_en;

    /** Data Buffer Threshold before indicating data available. Per channel. */
    uint8_t   channel;

} Iqn2Fl_AilEctlPktIf;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * configuring ECTL REGISTER GROUP of an AIL instance */
typedef struct 
{
    /** Rate Controller will allow the ECTL to create RATE+1 active requests on 
     *  the PSI bus within a 16 clock cycle window. As an example, a value of 7 
     *  will allow the ICTL to create 8 active requests within a 16-clock cycle 
     *  window which uses 50% of the PSI bus. 
     *  Default value is 15. Setting this value to a non-default value is possible if 0 < rate <= 15
     *  If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AIL_ECTL_RATE_CTL_CFG_REG CMD word
     */
    uint8_t   rate;

    /** Byte swapping control. 0:No swap, 1:Byte swap, 
     *  2:Half word swap (16-bits), 3:Word swap (32 bits)
     */
    uint8_t   dat_swap[4];

    /** IQ swapping control. 0:No swap, 1:No swap, 
     *  2:Byte swap, 3:16-bits swap
     */
    uint8_t   iq_order[4];

} Iqn2Fl_AilEctlRegGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilEgrSetup. This structure is used for
 * configuring EDC (Egress DMA Controller) REGISTER GROUP of an AIL instance */
typedef struct 
{
    /** Controls how the EDC handles packet errors detected by efe 
     *  DROP (0) = Drop the rest of the packet on errors 
     *  NO_DROP (1) = Do not drop packet on errors 
     */
    uint8_t   psi_err_chk_disable;

    /** Byte swapping control. 0:No swap, 1:Byte swap, 
     *  2:Half word swap (16-bits), 3:Word swap (32 bits)
     */
    uint8_t   dat_swap[64];

    /** IQ swapping control. 0:No swap, 1:No swap, 
     *  2:Byte swap, 3:16-bits swap
     */
    uint8_t   iq_order[64];

} Iqn2Fl_AilEdcRegGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilSetup. This structure is used for
 * configuring the Egress parameters of an AIL instance */
typedef struct 
{
    /** AIL SI IQ EFE CONFIG GROUP */
    Iqn2Fl_AilSiIqEfeCfgGrp   ail_si_iq_efe_config_group;

    /** AIL SI IQ EFE RADIO STANDARD GROUP */
    Iqn2Fl_AilSiIqEfeRadStdGrp   ail_si_iq_efe_radio_std_group;

    /** AIL IQ EFE CHAN AXC OFFSET
     *  AxC-by-AxC delay control. Allows different timing alignments for each AxC.
     */
    uint32_t ail_iq_efe_chan_axc_offset[64];

    /** AIL IQ EFE FRM SAMP TC MMR RAM
     *  Radio Framing Counter. Number of samples (4 bytes) per radio symbol 
     *  programmed as a terminal count.
     */
    uint32_t ail_iq_efe_frm_samp_tc_mmr_ram[256];

    /** AIL SI IQ E TDM LUT RAM */
    Iqn2Fl_AilSiIqEgrTdmLutRam   ail_iq_pe_axc_tdm_lut_cfg[256];

    /** AIL SI IQ E SCH PHY - PE DMA Channel 1 Register 
     *  SI Egress AIL scheduler, PHY FSM is enabled to turn ON, 
     *  will turn on next PE_FB from uAT
     */
    uint8_t   phy_en;

    /** AIL SI IQ E OBSAI MODTXRULE */
    Iqn2Fl_AilIqPeObsaiModtxruleCfg   ail_iq_pe_obsai_modtxrule_cfg[32];

    /** AIL SI IQ E OBSAI DBM RULE RAM */
    Iqn2Fl_AilSiIqEgrObsaiDbmRule   ail_si_iq_e_obsai_dbm_rule[32];

    /** AIL SI IQ E OBSAI DBM BITMAP RAM - PE DBMF Bit Map 1 & 2 Register */
    uint32_t ail_si_iq_e_obsai_dbm_bitmap_ram[256];

    /** AIL SI IQ E SCH CPRI */
    Iqn2Fl_AilSiIqEgrSchCpri   ail_si_iq_e_sch_cpri;

    /** AIL ECTL PKT IF REGISTERS */
    Iqn2Fl_AilEctlPktIf   ail_ectl_pkt_if[4];

    /** AIL ECTL REGISTER GROUP */
    Iqn2Fl_AilEctlRegGrp   ail_ectl_reg_grp;

    /** AIL IQ EDC REGISTER GROUP */
    Iqn2Fl_AilEdcRegGrp   ail_iq_edc_reg_grp;

} Iqn2Fl_AilEgrSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilIgrSetup. This structure is used
 * for configuring the Ingress Framing Engine (IFE) channel configuration group */
typedef struct 
{
    /** Enable channel: ENABLED(1), DISABLED(0) */
    uint8_t   chan_en;

    /** AxC Offset
     *    NONE (0) = No offset, ONE (1) = One sample offset
     *    TWO (2) = Two sample offset, THREE (3) = Three sample offset 
     */
    uint8_t   chan_axc_offset;

    /** Radio Standard Select */
    Iqn2Fl_ChanRadioSel   chan_radio_sel;

    /** Forces a channel into the TDD OFF state on the next symbol after 
     *  it is set to a 1 regardless of the TDD configuration of the radio 
     *  standard variant the channel is assigned to. 
     */
    Iqn2Fl_ChanFrcOff   chan_tdd_frc_off;

    /** Selects OBSAI channel as Control when high, OBSAI channel as AxC when low */
    Iqn2Fl_ChanObsaiCtl   chan_obsai_ctl;

    /** Selects between OBSAI ENET when high or OBSAI NON_ENET when low. 
     *  Only valid when CHAN_OBSAI_CTL is set. 
     */
    Iqn2Fl_ChanEnetCtlMode   chan_enet_ctl;

} Iqn2Fl_AilIqIfeChanCfgGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilIqIfeRadStdGrp. This structure is used
 * for configuring the Ingress Framing Engine (IFE) Radio Standard Config Register */
typedef struct 
{
    /** Enable TDD. 
     *  1: TDD enabled for this radio standard 
     *  0: TDD disabled for this radio standard 
     */
    uint8_t   tdd_lut_en;

} Iqn2Fl_AilIqIfeRadStdCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilIgrSetup. This structure is used for
 * Ingress Framing Engine (IFE) radio standard group parameters of an AIL instance */
typedef struct 
{
    /** AIL IQ IFE FRM TC CFG */
    Iqn2Fl_AilIqFrmTcCfg   ail_iq_ife_frm_tc_cfg[8];

    /** AIL IQ IFE RAD STD CFG */
    Iqn2Fl_AilIqIfeRadStdCfg   ail_iq_ife_rad_std_cfg[8];

    /** AIL IQ IFE RADIO STANDARD 0 TDD ENABLE LUT */
    uint32_t ail_iq_ife_tdd_en_cfg0[5];

    /** AIL IQ IFE RADIO STANDARD 1 TDD ENABLE LUT */
    uint32_t ail_iq_ife_tdd_en_cfg1[5];

    /** AIL IQ IFE RADIO STANDARD 2 TDD ENABLE LUT */
    uint32_t ail_iq_ife_tdd_en_cfg2[5];

    /** AIL IQ IFE RADIO STANDARD 3 TDD ENABLE LUT */
    uint32_t ail_iq_ife_tdd_en_cfg3[5];

    /** AIL IQ IFE RADIO STANDARD 4 TDD ENABLE LUT */
    uint32_t ail_iq_ife_tdd_en_cfg4[5];

    /** AIL IQ IFE RADIO STANDARD 5 TDD ENABLE LUT */
    uint32_t ail_iq_ife_tdd_en_cfg5[5];

    /** AIL IQ IFE RADIO STANDARD 6 TDD ENABLE LUT */
    uint32_t ail_iq_ife_tdd_en_cfg6[5];

    /** AIL IQ IFE RADIO STANDARD 7 TDD ENABLE LUT */
    uint32_t ail_iq_ife_tdd_en_cfg7[5];

} Iqn2Fl_AilIqIfeRadStdGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilIgrSetup. This structure is used
 * for configuring the Ingress DMA Controller (IDC) Configuration Group and 
 * ICTL PKT IF registers */
typedef struct 
{
    /** Controls how the IDC handles packet errors detected by IFE. 
     *    DROP (0) = Drop and Mark Packets With Errors 
     *    MARK (1) = Only Mark Packets With Errors
     */
    uint8_t   fail_mark_only;

    /** Forces off all Ingress channels without waiting for an end of 
     *  symbol or time slot. All open packets are automatically closed 
     *  by creating an EOP for each open packet. 
     *    FRC_OFF (1) = Force all channels off and close all open packets 
     *    NOP (0) = No effect
     */
    uint8_t   frc_off_all;

    /** Forces off all Ingress channels without waiting for an end of 
     *  symbol or time slot if there is an RM failure. All open packets 
     *  are automatically closed by creating an EOP for each open packet. 
     *    FRC_OFF (1) = Force off on RM failure. 
     *    NOP (0) = No effect
     *  Note: This field is not supported in ICTL (IDC IF) Config register.
     */
    uint8_t   rm_fail_frc_off_en;

} Iqn2Fl_AilIqIgrCfgGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilIgrSetup. This structure is used for
 * configuring IDC (Ingress DMA Controller) CHANNEL CONFIG GROUP and ICTL PKT IF registers 
 * of an AIL instance */
typedef struct 
{
    /** Byte swapping control. 0:No swap, 1:Byte swap, 
     *  2:Half word swap (16-bits), 3:Word swap (32 bits)
     */
    uint8_t   dat_swap;

    /** IQ swapping control. 0:No swap, 1:No swap, 
     *  2:Byte swap, 3:16-bits swap
     */
    uint8_t   iq_order;

    /** Programmable packet type that is inserted into pkt_type 
     *  field of PKTDMA Info Word 0.
     */
    uint8_t   pkt_type;

    /** Forces off all channel without waiting for an end of symbol 
     *  or time slot. If channel has an open packet it is automatically 
     *  closed by creating an EOP.
     *    FRC_OFF (0) = Force off channel and close an existing open packet
     *    NOP (1) = No effect
     */
    uint8_t   chan_frc_off;

} Iqn2Fl_AilIqIgrChCfgGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilIgrSetup. This structure is used for
 * configuring ICTL IDC IF of an AIL instance */
typedef struct 
{
    /** AIL ICTL CHANNEL CONFIGURATION REGISTERS */
    Iqn2Fl_AilIqIgrChCfgGrp   ictl_chan_cfg[4];

    /** AIL ICTL CONFIGURATION REGISTER */
    Iqn2Fl_AilIqIgrCfgGrp   ictl_cfg;

} Iqn2Fl_AilIctlIdcIf;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilSetup. This structure is used for
 * configuring the Ingress parameters of an AIL instance */
typedef struct 
{
    /** AIL IQ IFE CHANNEL CONFIGURATION GROUP */
    Iqn2Fl_AilIqIfeChanCfgGrp   ail_iq_ife_chan_config_group[64];

    /** AIL IQ IFE RADIO STANDARD GROUP */
    Iqn2Fl_AilIqIfeRadStdGrp   ail_iq_ife_radio_std_group;

    /** AIL IQ IDC CONFIGURATION GROUP */
    Iqn2Fl_AilIqIgrCfgGrp   ail_iq_idc_cfg_grp;

    /** AIL IQ IDC CHANNEL CONFIGURATION GROUP */
    Iqn2Fl_AilIqIgrChCfgGrp   ail_iq_idc_ch_cfg_grp[64];

    /** AIL IFE FRM SAMP TC MMR RAM 
     *  Radio Framing Counter. Number of samples (4 Bytes) per 
     *  radio symbol programmed as a terminal count. 
     */
    uint32_t   samp_tc[256];

    /** AIL ICTL IDC IF REGISTERS */
    Iqn2Fl_AilIctlIdcIf   ail_ictl_idc_if;

    /** AIL ICTL PKT IF - CHAN EN CFG
     *  ICTL Channel Configuration Enable Register
     *  Enable channel
     *     ENABLED (11) = Enable channel
     *     DISABLED (00) = Disable channel
     */
    uint32_t   ail_ictl_pkt_if_chan_en[4];

    /** AIL IQ INGRESS VBUS MMR GROUP 
     *  IDC RATE CONTROL CONFIGURATION REGISTER 
     *  Rate Controller will allow the IDC to create RATE+1 active 
     *  requests on the PSI bus within a 32 clock cycle window. As an 
     *  example, a value of 7 will allow the IDC to create 8 active 
     *  requests within a 32-clock cycle window which uses 25% of the PSI bus.
     *  Default value is 15. Setting this value to a non-default value is possible if 0 < rate_idc <= 15
     *  If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AIL_IQ_IDC_RATE_CTL_CFG_REG CMD word
     */
    uint8_t   rate_idc;

    /** AIL CTL INGRESS VBUS MMR GROUP
     *  ICTL RATE CONTROL CONFIGURATION REGISTER
     *  Rate Controller will allow the ICTL to create RATE+1 active 
     *  requests on the PSI bus within a 16 clock cycle window. As an 
     *  example, a value of 7 will allow the ICTL to create 8 active 
     *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
     *  Default value is 15. Setting this value to a non-default value is possible if 0 < rate_ictl <= 15
     *  If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AIL_ICTL_RATE_CTL_CFG_REG CMD word
     */
    uint8_t   rate_ictl;

} Iqn2Fl_AilIgrSetup;

/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPeCommon. This structure is used for
 * configuring Protocol Encoder (PE) Common Channel registers within an AIL instance.
 * Channel-by-Channel control. OBSAI: 64 generic AxC or control channels CPRI: 64 
 * AxC Channels, 4 CPRI control channels are configured elsewhere. For CPRI, several 
 * of the fields do-not-apply.
 */
typedef struct 
{
    /** CRC: enable CRC generation and insert on AxC by AxC basis */
    uint8_t   crc_en;

    /** OBSAI/CPRI Controls RT to perform appropriate insterion/aggregation
     *  into PHY applied on msg-by-msg basis for OBSAI or sample-by-sample 
     *  basis for CPRI. 
     *    ADD8 (3) = Aggregate/add both PE and RM contributions, samples 
     *               are 8bit I and Q -or- 7bit I and Q
     *    ADD16 (2) = Aggregate/add both PE and RM contributions, samples 
     *                are 16bit I and Q -or- 15bit I and Q
     *    INSERTPE (1) = Insert the PE contribution, ignoring any possible 
     *                   RM contribution
     *    FWD_RM (0) = Forward the RM contribution ignoring the PE contribution 
     */
    Iqn2Fl_RtControll   rt_ctl;

    /** OBSAI CRC: length of CRC. 
     *    32BIT_CRC (0) = 32BIT_CRC
     *    16BIT_CRC (1) = 16BIT_CRC 
     */
    uint8_t   crc_type;

    /** OBSAI Channel is ethernet. This field controls insertion 
     *  of the Ethernet Premble and SOF (and prevents CRC of these bytes)
     *     ENET_ON (1) = channel is ethernet
     *     ENET_OFF (0) = channel is not ethernet 
     */
    uint8_t   ethernet;

    /** OBSAI CRC16 is calucalted over OBSAI header as well as payload 
     *  (only valid for OBSAI TYPES CONTROL (0x0) & MEASUREMENT (0x1)
     *     CRC_HDR_ON (1) = CRC16 is caluculated over 3 byte OBSAI header 
     *                      and 14 bytes of 16 byte payload. Only valid 
     *                      for 19 byte OBSAI messages utilizing CRC16
     *     CRC_HDR_OFF (0) = do not perform CRC over OBSAI Header (program 
     *                       to OFF for CPRI and for most OBSAI types)
     */
    uint8_t   crc_hdr;

} Iqn2Fl_AilPeCommonChanCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPeSetup. This structure is used for
 * configuring Protocol Encoder (PE) Global and Channel registers of an AIL instance */
typedef struct 
{
    /** AIL PE GLOBAL CONFIGURATION REGISTER
     *  Ethernet header selection. When using Ethernet, bit order 
     *  for Ethernet preamble and SOF 0: 0xAAAAAAAB 1: 0x555555D5.
     */
    uint8_t   enet_hdr_sel;

    /** AIL PE CHANNEL CONFIGURATION REGISTER */
    Iqn2Fl_AilPeCommonChanCfg   ail_pe_common_chan_cfg[64];

} Iqn2Fl_AilPeCommon;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPeSetup. This structure is used for
 * configuring Protocol Encoder (PE) OBSAI header LUT within an AIL instance.
 * OBSAI chan-by-chan, control of OBSAI header creation.
 */
typedef struct 
{
    /** OBSAI hdr(5:0) TS : only used if TS is inserted instead of generated */
    uint8_t   ts_adr;

    /** OBSAI hdr(10:6) TYPE: inserted into OBSAI header TYPE field */
    uint8_t   typ;

    /** OBSAI hdr(23:11)ADR : inserted into OBSAI header ADDRESS field */
    uint8_t   adr;

    /** Controls which parts of the OBSAI TS field are inserted vs. calculated
     *    NONE (0) = All TS bits are genrated as actual time stamp by PE 
     *               circuits (normal AxC TS), obsai_ts_adr is not inserted 
     *               and is unused
     *    4LSB (1) = 4 lsb bits of obsai_ts_adr are inserted into the TS 
     *               field as and extention of addressing, 2 msb are 
     *               generated as packet demarkation
     *    ALL (2) = All TS(5:0) bits are inserted from 
     *              pe_obsai_hdr_cfg.ts_adr(5:0) as and extention of addressing
     *    RSVD_MSK (3) = Reserved 
     */
    uint8_t   ts_mask;

    /** TS: OBSAI time stamp generation algorithm
     *    NO_TS (0) = 00_0000, No time stamp insertion -or- OBSAI 
     *                Generic Control Message, one message packet with 
     *                inferred SOP & EOP in same OBSAI message.
     *    NORM_TS (1) = normal time stamp format
     *    GSM_UL (2) = GSM OBSAI Time Stamp, UL time stamp format msb=1 
     *                 first four msg. DL time stamp format msb=1 first msg
     *    GEN_PKT (3) = OBSAI Generic Packet (SOP=10, MOP=00, EOP=11)
     *    ETHERNET (4) = Ethernet Type where last TS indicates number of 
     *                   valid bytes in last xfer
     *    CTR_PKT (5) = 00_0001 OBSAI Control Message for Air Interface 
     *                  Synch Operations, one message packet with inferred 
     *                  SOP & EOP in same OBSAI message.
     *    GSM_DL (6) = GSM OBSAI Time Stamp, UL time stamp format msb=1 
     *                 first four msg. DL time stamp format msb=1 first msg 
     */
    uint8_t   ts_frmt;

    /** Normal header processing is performed except the above MMR 
     *  adr, type, ts_adr fields are unsed and instead this information 
     *  is replaced with the CPPI DMA protocol specific (ps) data. 
     *  Effectively, when used, ps data allows APP SW to control OBSAI 
     *  header on packet by packet basis. Used for Base Band Hopping of 
     *  GSM control packets. PS data is not available for AxC Packets
     *    NO_PS (0) = normal, PS is unused
     *    ADR_PS (1) = MMR pe_obsai_hdr_cfg.adr MMR field above is not 
     *                 used and instead replaced by PS data
     *    ALL_PS (2) = MMR pe_obsai_hdr_cfg.adr, type, ts_adr fields 
     *                 above are not used and instead replaced by PS data
     *    RSVD_PS (3) = Reserved 
     */
    uint8_t   ps_insert;

} Iqn2Fl_AilPeObsaiHdrLut;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPeCpriCw. This structure is used for
 * configuring Protocol Encoder (PE) CPRI CW Channel registers within an AIL instance.
 * PE CPRI, Primary register for configuring CPRI Control words. Contains primary 
 * selection of dilineation type plus many fields which are common. One MMR per each 
 * of four channels.
 */
typedef struct 
{
    /** 0: normal no byte reversal for CRC, 1: reverse crc16 byte order 
     *  (illegal for CRC8 or CRC32)
     *     NO_RVRS (0) = No swapping is done
     *     RVRS_BITS (1) = Swaps the two nibbles in each byte
     *     RVRS_BYTES (2) = Reverses the bits within each nibble
     *     RVRS_BOTH (3) = Reverses the order of the bits in the 
     *                     byte as the bytes first come in to the PE
     */
    uint8_t   crc_rvrs;

    /** Select which form of delineation is to be used for this chan
     *    FAST_ETH (0) = Fast Ethernet, 4B5B encoding
     *    HDLC (1) = HDLC
     *    HYP_FRM (2) = hyper frame delineated, capture all indictated 
     *                  CW for enabled hyperframes, creating CPPI packet
     *    NULL (3) = Null delineated, null character indicates lack of 
     *               traffic, SOP: null-to-non_null EOP: non_null-to-null 
     */
    uint8_t   delin_sel;

    /** Select which form of CRC generation and insertion to perform
     *    NO_CRC (0) = no CRC is used (Users may wish to use processors 
     *                 to generate CRC or use CRC passed by NetCP Ethernet)
     *    CRC32 (1) = CRC32 is used with fixed polynomial
     *    CRC16 (2) = CRC16 is used with fixed polynomial
     *    CRC8 (3) = CRC8 is used with programmable polynomial 
     */
    uint8_t   crc_sel;

    /** 0: CRC operation is initialized with zeros 
     *  1: CRC opperation is initialized with ones 
     */
    uint8_t   crc_init;

    /** Enable use of hyperframe_lut (which limits usable 
     *  hyperframes), intended use is RTWP 
     */
    uint8_t   hf_lut_en;

    /** 0: no swap 1: swap nibbles 2: swap nibble bits 3: swap both
     *    NO_SWAP (0) = No swapping is done
     *    SWAP_NIBBLES (1) = Swaps the two nibbles in each byte
     *    SWAP_NIBBLE_BITS (2) = Reverses the bits within each nibble
     *    SWAP_BOTH (3) = Reverses the order of the bits in the byte 
     *                    as the bytes first come in to the PE 
     */
    uint8_t   dlmt_imux;

    /** 0: no swap 1: swap nibbles 2: swap nibble bits 3: swap both
     *    NO_SWAP (0) = No swapping is done
     *    SWAP_NIBBLES (1) = Swaps the two nibbles in each byte
     *    SWAP_NIBBLE_BITS (2) = Reverses the bits within each nibble
     *    SWAP_BOTH (3) = Reverses the order of the bits in the byte 
     *                    as the bytes first come in to the PE 
     */
    uint8_t   dlmt_omux;

    /** Selects the nibble based swapping for each data byte.
     *    NO_SWAP (0) = No swapping is done
     *    SWAP_NIBBLES (1) = Swaps the two nibbles in each byte
     *    SWAP_NIBBLE_BITS (2) = Reverses the bits within each nibble
     *    SWAP_BOTH (3) = Reverses the order of the bits in the byte 
     *                    as the bytes first come in to the PE 
     */
    uint8_t   imux;

    /** Controls AIL_PHY_RT to perform appropriate insterion/aggregation 
     *  into PHY, applied on sample by sample basis.
     *    ADD8 (3) = Aggregate/add both PE and RM contributions, 
     *               samples are 8bit I and Q (not be appropriate for 
     *               control data)
     *    ADD16 (2) = Aggregate/add both PE and RM contributions, 
     *                samples are 16bit I and Q (not be appropriate 
     *                for control data)
     *    INSERTPE (1) = Insert the PE contribution, ignoring any 
     *                   possible RM contribution
     *    FWD_RM (0) = Forward the RM contribution, discarding any 
     *                 PE contribution. 
     */
    uint8_t   rt_ctl;

    /** CW byte Enable, used to disable CW bytes for some low rate 
     *  HDLC or 5x/10x RTWP. (normal set to 0xffff) 
     */
    uint16_t   byte_en;

} Iqn2Fl_AilPeCpriCwChanCfg;

/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPeCpriCw. This structure is used for
 * configuring Protocol Encoder (PE) CPRI CW Fast Ethernet 4B5B of an AIL instance */
typedef struct 
{
    /** Reverses the bit order of the 4b5b data when high */
    uint8_t   bit_order;

    /** Swaps the order of the SSD and ESD 5 bit values when high */
    uint8_t   ssd_order;

    /** Selects the number of bytes of the preamble added between 
     *  the SSD and ESD and the first byte of data. Selection is 
     *  between no preamble, 7 bytes of preamble and 8 bytes of preamble.
     *    NO_PREAMBLE (0) = No preamble is used
     *    PREAMBLE_7 (1) = Preamble is 7 bytes (correct CPRI preamble length)
     *    PREAMBLE_8 (2) = Preamble is 8 bytes (AIF2 backwards compatability)
    */
    uint8_t   hdr;

    /** Data to send as the first 7 bytes of the preamble */
    uint8_t   hdr_preamble;

    /** Data to send as the last byte of the preamble */
    uint8_t   hdr_sop;

} Iqn2Fl_AilPeCpriCwFastEth4b5b;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPeCpriCw. This structure is used for
 * configuring Protocol Encoder (PE) CPRI CW Look Up Table of an AIL instance */
typedef struct 
{
    /** 0: This CPRI CW is allocated to CPRI CW chan0, 
     *  1: chan1, 2: chan2, 3: chan3 
     */
    uint8_t   cw_chan;

    /** 1: This CPRI CW is used and allocated to cw_chan 
     *  0: unused, disregard programmed cw_chan 
     */
    uint8_t   cw_en;

} Iqn2Fl_AilPeCpriCwLut;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPeSetup. This structure is used for
 * configuring Protocol Encoder (PE) CPRI CW within an AIL instance */
typedef struct 
{
    /** AIL PE CPRI CW CHAN CFG */
    Iqn2Fl_AilPeCpriCwChanCfg   ail_pe_cpri_cw_chan_cfg[4];

    /** AIL PE CPRI CW HYPFRM0 LUT CFG 
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused
     */
    uint32_t   hf_en_part0;

    /** AIL PE CPRI CW HYPFRM1 LUT CFG 
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused
     */
    uint32_t   hf_en_part1;

    /** AIL PE CPRI CW HYPFRM2 LUT CFG 
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused
     */
    uint32_t   hf_en_part2;

    /** AIL PE CPRI CW HYPFRM3 LUT CFG 
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused
     */
    uint32_t   hf_en_part3;

    /** AIL PE CPRI CW HYPFRM4 LUT CFG 
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused
     */
    uint32_t   hf_en_part4;

    /** AIL PE CPRI CW NULL CHARACTER CFG 
     *  Null Delineator Character: bit8 indicates k_char
     */
    uint32_t   null_char;

    /** AIL PE CPRI CW CRC8 CFG 
     *  CRC8 programmable polynomial
     */
    uint32_t   crc8_poly;

    /** AIL PE CPRI CW FAST ETHERNET 4B5B CFG */
    Iqn2Fl_AilPeCpriCwFastEth4b5b   ail_pe_cpri_cw_fast_eth_4b5b[4];

    /** AIL PE CPRI CW LOOK UP TABLE CFG */
    Iqn2Fl_AilPeCpriCwLut   ail_pe_cpri_cw_lut[256];

} Iqn2Fl_AilPeCpriCw;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilSetup. This structure is used for
 * configuring Protocol Encoder (PE) parameters of an AIL instance */
typedef struct 
{
    /** AIL PE COMMON */
    Iqn2Fl_AilPeCommon   ail_pe_common;

    /** AIL PE OBSAI HEADER LUT */
    Iqn2Fl_AilPeObsaiHdrLut   ail_pe_obsai_hdr_lut[64];

    /** AIL PE CPRI CW */
    Iqn2Fl_AilPeCpriCw   ail_pe_cpri_cw;

} Iqn2Fl_AilPeSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdSetup. This structure is used for
 * configuring Protocol Decoder (PD) Channel registers of an AIL instance */
typedef struct 
{
    /** OBSAI: Antenna Carrier offset programmed in 307.2MHz clocks and 
     *  relative to the Frame Boundary of the RadT AT timer. Specifies 
     *  the center of the receive window for an AxC offset (typical WCDMA 
     *  value of (4chip_offset x 320) + 320/2) CPRI: Antenna Carrier 
     *  Offset programmed in (non-oversampled) samples, relative to the 
     *  Radio frame boundary. This concept of an AxC_Offset in addition 
     *  to the Radio Standard Offset seems go beyond the CPRI standard. 
     *  It is expected that most CPRI users will program this field as zero. 
     */
    uint32_t   axc_offset;

    /** Assign each channel to one of 8 radio standards. 
     *  Used for radio standard FSM and RadT selection 
     */
    uint8_t   rad_std;

} Iqn2Fl_AilPdCommon;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdCpriAxcCfg. This structure
 * is used for configuring Protocol Decoder (PD) CPRI AXC0 registers.
 * PD CPRI PHY Container LUT: Maps CPRI transports containers to 1 of 8 
 * groups (radio standard), Only CPRI16x can use full 64 depth of LUT.
 */
typedef struct 
{
    /** Group (radio standard) which container belongs to, 
     *  0-to-7. The PD double buffers these registers on every 
     *  phy frame boundary. 
     */
    uint8_t   cont_lut_grp;

    /** 0: container is unused by PD 1: container is mapped by cont_lut_grp
     *    CONT_DIS (0) = CONT_DIS
     *    CONT_EN (1) = CONT_EN 
     */
    uint8_t   cont_lut_en;

} Iqn2Fl_AilPdCpriAxc0Cfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdCpriAxcCfg. This structure
 * is used for configuring Protocol Decoder (PD) CPRI TDM FSM registers.
 * FSM-by-FSM, CPRI AxC TDM. Intended use is one FSM per radio standard 
 * (matching AIL_PHY_CI groups).
 */
typedef struct 
{
    /** Number of AxC_LUT entries (containers) for this FSM 
     *  (For LTE20, 8 sequential entries within AxC_LUT) 
     */
    uint8_t   ncont;

    /** First AxC_LUT entry for this fsm */
    uint8_t   start_lut;

} Iqn2Fl_AilPdCpriTdmFsmCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdSetup. This structure is used for
 * configuring Protocol Decoder (PD) CPRI AXC registers of an AIL instance */
typedef struct 
{
    /** AIL PD CPRI AXC0 CFG */
    Iqn2Fl_AilPdCpriAxc0Cfg   ail_pd_cpri_axc0_cfg[64];

    /** AIL PD CPRI BUB FSM CFG
     *  CPRI Mapping Method 3, Bubble insertion state machine control, 
     *  total of 8 separate FSMs, one per 8 groups/radio_standards.
     *  (K x Nc) total number of containers per AxC Container Group. 0=one
     */
    uint32_t   bub_fsm_cfg_knc[8];

    /** AIL PD CPRI BUB FSM2 CFG
     *  CPRI Mapping Method 3, Bubble insertion state machine control, 
     *  total of 8 separate FSMs, one per 8 groups/radio_standards.
     *  Bubble Gap Integer portion configuration
     */
    uint32_t   bub_fsm2_cfg_gap_int[8];

    /** AIL PD CPRI BUB FSM2 CFG
     *  CPRI Mapping Method 3, Bubble insertion state machine control, 
     *  total of 8 separate FSMs, one per 8 groups/radio_standards.
     *  Bubble Gap Fractional portion configuration
     */
    uint32_t   bub_fsm2_cfg_gap_frac[8];

    /** AIL PD CPRI TDM FSM CFG
     *  FSM-by-FSM, CPRI AxC TDM. Intended use is one FSM 
     *  per radio standard (matching AIL_PHY_CI groups).
     */
    Iqn2Fl_AilPdCpriTdmFsmCfg   ail_pd_cpri_tdm_fsm_cfg[8];

    /** AIL PD CPRI AXC RADSTD CFG
     *  Enables and configures each individual radio standard for the PE 
     *  for up to 8 radio standards 
     */
    Iqn2Fl_AilSiIqCpriRadstdCfg   ail_iq_pd_cpri_axc_radstd_cfg;

    /** AIL PD CPRI AXC TDM LUT CFG
     *  TDM AxC LUT. Used to map steams of CPRI containers to 
     *  appropriate AxC. AxC are listed in order that they are 
     *  represented on the CPRI link, different portions of LUT 
     *  allocated to different groups (radio standards).
     *  List of AxC indexes giving TDM AxC order over CPRI link
     */
    uint32_t   axc_tdm_lut_cfg_axc[256];

    /** AIL PD CPRI AXC TDM LUT CFG
     *  TDM AxC LUT. Used to map steams of CPRI containers to 
     *  appropriate AxC. AxC are listed in order that they are 
     *  represented on the CPRI link, different portions of LUT 
     *  allocated to different groups (radio standards).
     *  Enables each entry. Disabled entries will result in dropped 
     *  CPRI containers. Main use of this disable, so the SW wis not 
     *  required to maintain unique AxC value for unused containers.
     */
    uint32_t   axc_tdm_lut_cfg_en[256];

} Iqn2Fl_AilPdCpriAxcCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdCpriCwCfg. This structure
 * is used for configuring Protocol Decoder (PD) CPRI CW Channel registers.
 * PD CPRI Codeword Configuration register
 */
typedef struct 
{
    /** All possible CPRI sub-channel per hyperframe are mapped 
     *  to one of four CPRI CW staging areas. This allow CPRI CW 
     *  to be split into 4 differen streams. 
     */
    uint8_t   chan_en;

    /** Enable use of hyperframe_lut (which limits usable 
     *  hyperframes), intended use is RTWP.
     */
    uint8_t   hf_lut_en;

    /** Select which form of delineation is to be used for this chan
     *    NULL (0) = Null delineated, null character indicates lack 
     *               of traffic, SOP: null-to-non_null EOP: non_null-to-null
     *    FAST_ETH (1) = Fast Ethernet, 4B5B encoding
     *    HYP_FRM (2) = hyper frame delineated, capture all indictated CW 
     *                  for enabled hyperframes, creating CPPI packet
     *    HDLC (3) = HDLC 
     */
    uint8_t   delin_sel;

    /** Select which form of CRC check to perform
     *    NO_CRC (0) = no CRC is used 
     *    CRC32 (1) = CRC32 is used with fixed polynomial
     *    CRC16 (2) = CRC16 is used with fixed polynomial
     *    CRC8 (3) = CRC8 is used with programmable polynomial 
     */
    uint8_t   crc_sel;

    /** 0: CRC operation is initialized with zeros 
     *  1: CRC opperation is initialized with ones 
     */
    uint8_t   crc_init;

    /** 0: no swap 1: swap nibbles 2: swap nibble bits 3: swap both */
    uint8_t   dlmt_imux;

    /** 0: no swap 1: swap nibbles 2: swap nibble bits 3: swap both */
    uint8_t   dlmt_omux;

    /** 0: normal no byte reversal for CRC, 1: reverse crc16 byte order 
     *  (illegal for CRC8 or CRC32) 
     */
    uint8_t   qwd_omux;

    /** 0: NO Reverse 1: Reverse bit: 2: Reverse byte 3: Reverse Both */
    uint8_t   hdlc_rvrs_crc;

    /** CW byte Enable, used to disable CW bytes for some low rate 
     *  HDLC or 5x/10x RTWP. (normal use set 0xffff) 
     */
    uint16_t   byte_en;

} Iqn2Fl_AilPdCpriCwChanCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdSetup. This structure is used for
 * configuring Protocol Decoder (PD) CPRI CW registers of an AIL instance */
typedef struct 
{
    /** AIL PD CPRI CW CHAN CFG */
    Iqn2Fl_AilPdCpriCwChanCfg   chan_cfg[4];

    /** AIL PD CPRI CW LUT CFG 
     *  CPRI CW Channel LUT Register.
     *  Selects which of the 4 channels to use for this basic frame if the 
     *  enable bit is set.
     */
    uint8_t   lut_cfg_cw_chan[256];

    /** AIL PD CPRI CW LUT CFG 
     *  CPRI CW Channel LUT Register
     *  All possible CPRI sub-channel. Dicatates whether the control word 
     *  should be captured at all.
     *     DISABLE (00) = When 0 this CW is ignored
     *     ENABLE (11) = When 1 this CW is transferred
     */
    uint8_t   lut_cfg_cw_en[256];

    /** AIL PD CPRI HYPFRM0 LUT CFG 
     *  PD CPRI Hyperframe Enable Part0, used to support RTWP, use enabled 
     *  by cw_chan register, only one hyperframe_lut for all 4 channels.
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused.
     */
    uint32_t   hypfrm0_lut_cfg_hf_en;

    /** AIL PD CPRI HYPFRM1 LUT CFG 
     *  PD CPRI Hyperframe Enable Part1, used to support RTWP, use enabled 
     *  by cw_chan register, only one hyperframe_lut for all 4 channels.
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused.
     */
    uint32_t   hypfrm1_lut_cfg_hf_en;

    /** AIL PD CPRI HYPFRM2 LUT CFG 
     *  PD CPRI Hyperframe Enable Part2, used to support RTWP, use enabled 
     *  by cw_chan register, only one hyperframe_lut for all 4 channels.
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused.
     */
    uint32_t   hypfrm2_lut_cfg_hf_en;

    /** AIL PD CPRI HYPFRM3 LUT CFG 
     *  PD CPRI Hyperframe Enable Part3, used to support RTWP, use enabled 
     *  by cw_chan register, only one hyperframe_lut for all 4 channels.
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused.
     */
    uint32_t   hypfrm3_lut_cfg_hf_en;

    /** AIL PD CPRI HYPFRM4 LUT CFG 
     *  PD CPRI Hyperframe Enable Part4, used to support RTWP, use enabled 
     *  by cw_chan register, only one hyperframe_lut for all 4 channels.
     *  Enable bit per Hyperframe, 1: hyperframe used, 0: hyperframe unused.
     */
    uint32_t   hypfrm4_lut_cfg_hf_en;

    /** AIL PD CPRI NULL CFG 
     *  PD CPRI NULL Delimiter control register.
     *  Null Delineator Character: bit8 indicates this is a k_char when set.
     */
    uint16_t   null_cfg_null_char;

    /** AIL PD CPRI CRC CFG 
     *  PD CPRI CRC8 control register.
     *  CRC8 programmable polynomial.
     */
    uint16_t   crc_cfg_crc8_poly;

    /** AIL PD CPRI CRC CFG 
     *  PD CPRI CRC8 control register.
     *  CRC8 compare value when init is choosen as 1's. Sometimes referred 
     *  as the Magic Number. Depends on chosen polynomial.
     */
    uint16_t   crc_cfg_crc8_comp;

    /** AIL PD CPRI 4B5B CFG 
     *  PD CPRI fast ethernet control register.
     *  Header stripping control
     *    NO_STRIP (0) = No stripping of the header
     *    STRIP_7 (1) = Strip 7 bytes of header
     *    STRIP_8 (2) = Strip 8 bytes of header
     *    NO_CRC (3) = No stripping and ignore the CRC
     */
    uint8_t   cpri_4b5b_cfg_hdr[4];

    /** AIL PD CPRI 4B5B CFG 
     *  PD CPRI fast ethernet control register.
     *  Swaps order of the 4b5b value in the header
     */
    uint8_t   cpri_4b5b_cfg_bit_order[4];

    /** AIL PD CPRI 4B5B CFG 
     *  PD CPRI fast ethernet control register.
     *  Swaps order of the SSD value in the header
     */
    uint8_t   cpri_4b5b_cfg_ssd_order[4];

} Iqn2Fl_AilPdCpriCwCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdObsaiLutCfg. This structure is used for
 * configuring Protocol Decoder (PD) OBSAI CHAN CFG registers of an AIL instance */
typedef struct 
{
    /** Enable WDog timer. Used for missing traffic handling and 
     *  GSM BB_Hop missing EOP. WDog is configured per radstd 
     */
    uint8_t   wdog_en;

    /** Data Format
     *    OTHER (0) = Not GSM UL
     *    GSM_UL (1) = GSM UL, has special OBSAI Time Stamp implications. 
     *                 UL time stamp format msb=1 first four msg  
     */
    uint8_t   gsm_ul;

} Iqn2Fl_AilPdObsaiChanCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdObsaiLutCfg. This structure is used for
 * configuring Protocol Decoder (PD) OBSAI CHAN CFG registers of an AIL instance */
typedef struct 
{
    /** Reception Routing: OBSAI time stamp. Used to extend addressing by 
     *  using the TS field. (Only known use is for OBSAI Generic Packet type 
     *  and Control Packet type) The number of bits of TS to compare is 
     *  controlled via the mask field. 
     */
    uint8_t   chan_ts;

    /** Reception Routing: OBSAI type */
    uint8_t   chan_type;

    /** Reception Routing: OBSAI address */
    uint8_t   chan_adr;

    /** Reception Routing: controls how many OBSAI time stamp bits to use in 
     *  the reception routing
     *    NONE (0) = none. Do not use the TS field for routing
     *    4LSB (1) = 4 lsb bits: Use TS(3:0)
     *    ALL (2) = all Use TS(5:0)
     *    RESERVED (3) = Reserved 
     */
    uint8_t   chan_mask;

    /** 0: this channel is always OFF regardless of compare (assumes other 
     *  fields are junk) 1: channel is enabled 
     */
    uint8_t   chan_en;

} Iqn2Fl_AilPdObsaiRouteCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdObsaiLutCfg. This structure is used for
 * configuring Protocol Decoder (PD) OBSAI TYPE LUT CFG registers of an AIL instance */
typedef struct 
{
    /** TS: OBSAI time stamp check (n/a CPRI)
     *    NO_TS (0) = No time stamp check, might be useful for some future 
     *                radio standard or debug
     *    NORM_TS (1) = normal time stamp format
     *    GSM (2) = GSM OBSAI Time Stamp, UL time stamp format msb=1 first 
     *              four msg. DL time stamp format msb=1 first msg
     *    GEN_PKT (3) = Generic Packet (SOP=10, MOP=00, EOP=11)
     *    ETHERNET (4) = Ethernet Type where last TS indicates number of 
     *                   valid bytes in last xfer
     *    CTR_PKT (5) = TS checked by PD_Route, one message packet with 
     *                  inferred SOP & EOP in same OBSAI message. 
     */
    uint8_t   ts_format;

    /** CRC: length of CRC. 0 = 32BIT_CRC, 1 = 16BIT_CRC */
    uint8_t   crc_type;

    /** CRC: enable CRC check on type-by-type basis */
    uint8_t   crc_en;

    /** CRC16 is calucalted over OBSAI header as well as payload (only valid 
     *  for OBSAI TYPES CONTROL (0x0) & MEASUREMENT (0x1)
     *    CRC_HDR_ON (1) = CRC16 is caluculated over 3 byte OBSAI header and 
     *                     14 bytes of 16 byte payload. Only valid for 19 byte 
     *                     OBSAI messages utilizing CRC16
     *    CRC_HDR_OFF (0) = do not perform CRC over OBSAI Header (program to 
     *                      OFF for CPRI and for most OBSAI types)
     */
    uint8_t   crc_hdr;

    /** Determines if channel is packet type channel or axc type channel. AxC 
     *  types align to Radio Frame Boundary while pkt alignment is a don't 
     *  care. AXC channels are expected to stream where packet channels are 
     *  on-demand. AXC channels use OBSAI Time Stamp and AXC Offset while 
     *  packet traffic does not. Intended to be programmed consistantly with 
     *  other TS format fields
     *    OBSAI_AXC (0) = OBSAI_AXC
     *    OBSAI_PKT (1) = OBSAI_PKT 
     */
    uint8_t   obsai_pkt_en;

    /** 0x0: No Strip 0x1: Strip off the first 8 bytes of each packet. 
     *  Purpose is to strip the ethernet preamble and SOP. PD blindly strips 
     *  without checking content of 8 bytes. CRC is not calculated over 
     *  stripped bytes. 
     */
    uint8_t   enet_strip;

    /** Used for RP3-01 FCB reception (slave timining sync via OBSAI link). 
     *  Capture the OBSAI payload to an MMR, Fire off a strobe to uAT for BCN 
     *  capture, fire off an EE for purposes of alerting processors of FCB 
     *  arival. 
     */
    uint8_t   rp3_01;

    /** Used for RP3-01 Reset reception. If message type is received with 
     *  correct address, the pd_soc_rst signal is set, causing an soc reset, 
     *  if enabled. 
     */
    uint8_t   rp3_01_rst;

} Iqn2Fl_AilPdObsaiTypeLutCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdObsaiCfg. This structure is used for
 * configuring Protocol Decoder (PD) OBSAI LUT CFG registers of an AIL instance */
typedef struct 
{
    /** AIL PD OBSAI CHAN CFG 
     *  x64 chan: PD Channel Control Fields Register 1
     */
    Iqn2Fl_AilPdObsaiChanCfg   chan_cfg[64];

    /** AIL PD OBSAI ROUTE CFG 
     *  x64 chan: Information used to decode OBSAI header 
     *  information for routing into PD channels.
     */
    Iqn2Fl_AilPdObsaiRouteCfg   route_cfg[64];

    /** AIL PD OBSAI TYPE LUT CFG 
     *  Link-by-Link: OBSAI Type Look Up Table. Allows for new OBSAI types to 
     *  be defined (and reconfiguation of exisiting Types). SW should set these 
     *  values via ROM look up.
     */
    Iqn2Fl_AilPdObsaiTypeLutCfg   type_lut_cfg[32];

} Iqn2Fl_AilPdObsaiLutCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPdSetup. This structure is used for
 * configuring Protocol Decoder (PD) OBSAI CFG registers of an AIL instance */
typedef struct 
{
    /** AIL PD OBSAI GSM BBHOP CFG 
     *  Chan-by-chan strobe signal: OBSAI GSM base band hopping control. 
     *  Indicates to PD that next timeslot will not have PHY data. PD 
     *  increments radio FSM by 1 time slot. pd_gsm_bbhop_cfg[1]: 
     *  chan63-to-32 pd_gsm_bbhop_cfg[0]: chan31-to-0. Value is cleared 
     *  once used or is channel is both OFF and not enabled.
     *  1: next GSM time slot is OFF for this chan 0: normal, expect 
     *  data to arrive on PHy (Only used if pd_chan_cfg.gsm_bbhop=1)
     */
    uint32_t   gsm_bbhop_cfg_off_stb[2];

    /** AIL PD OBSAI RADSTD CFG 
     *  OBSAI timing control, RadT usage (used for OBSAI LTE & WiMax). 
     *  Watch dog timer terminal count (sys_clks). Each WDog TC 
     *  increments WDG 8 count by 1.
     */
    uint16_t   radstd_cfg_wdog_tc[8];

    /** AIL PD OBSAI RADSTD CFG 
     *  OBSAI timing control, RadT usage (used for OBSAI LTE & WiMax). 
     *  OBSAI reception timing window width parameter. Indicates the
     *  window size for searching the radio frame boundary identified 
     *  by TS=0. Minimum WCDMA value is (320/2 + 80).
     */
    uint16_t   radstd_cfg_axcoffset_win[8];

    /** AIL PD OBSAI RADT CFG 
     *  OBSAI timing control, RadT usage (used for OBSAI LTE & WiMax). 
     *  Max expected value for the RadT timer value. Used for creating 
     *  an expected timing window (for which TS=0 marks a Radio Frame 
     *  Boundary).
     */
    uint32_t   radt_cfg_tc[8];

    /** AIL PD OBSAI FRM TC CFG 
     *  For framing state machine, supplies terminal counts of some of 
     *  the state counters and some start counts. Eight different versions 
     *  of this register are used to support 8 different radio standard 
     *  variants simultaneously.
     */
    Iqn2Fl_AilIqFrmTcCfg   frm_tc_cfg[8];

    /** AIL PD OBSAI LUT CFG */
    Iqn2Fl_AilPdObsaiLutCfg   lut_cfg;

    /** AIL PD OBSAI FRM MSG TC CFG 
     *  x128 LUT Depth: PD Frame Message Count Register.
     *  Radio AxC framing counter. nx4 sample count (OBSAI equal to 
     *  message count).
     */
    uint16_t   frm_msg_tc_cfg_tc[256];

} Iqn2Fl_AilPdObsaiCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilSetup. This structure is used for
 * configuring Protocol Decoder (PD) parameters of an AIL instance */
typedef struct 
{
    /** AIL PD COMMON CHAN CFG */
    Iqn2Fl_AilPdCommon   ail_pd_common[64];

    /** AIL PD CPRI AXC CFG and AIL PD CPRI AXC TDM LUT CFG */
    Iqn2Fl_AilPdCpriAxcCfg   ail_pd_cpri_axc_cfg;

    /** AIL PD CPRI CW CFG */
    Iqn2Fl_AilPdCpriCwCfg   ail_pd_cpri_cw_cfg;

    /** AIL PD OBSAI CFG and AIL PD OBSAI LUT CFG */
    Iqn2Fl_AilPdObsaiCfg   ail_pd_obsai_cfg;

    /** AIL PD OBSAI FRM MSG TC CFG 
     *  x256 LUT Depth: PD Frame Message Count Register. This table is shared 
     *  between all 8 radio standards. The INDEX values control the mapping 
     *  of different portions of this table to the different radio standards.
     *  Radio AxC framing counter. nx4 sample count (OBSAI equal to message 
     *  count). Note this differs from similar registers in the SI_ING and 
     *  SI_EGR which is programmed in units of samples vs. nx4 samples. For 
     *  nornal TimeStamp prediction, RadStd FSM does not need to predict 
     *  SYM bounaries, only Frame boundaries. For LTE1.4 and LTE3.0 use 
     *  program for only 1 sym per sub-frame.
     */
    uint16_t   ail_pd_obsai_frm_msg_tc_cfg_tc[256];

} Iqn2Fl_AilPdSetup;


/**
 * @brief This is used for configuring AIL PHY CI/CO LUT A/B registers of an 
 * AIL instance. These registers control up to eight groups of AxC containers
 * for every given CPRI basic frame.
 */
typedef struct 
{
    /** Index. Range = 0:7
     */
    uint8_t   index;

    /** SMPL COUNT
     *  Defines the number of consecutive AxC Containers in the AxC 
     *  Container Group
     */
    uint8_t   smpl_count;

    /** SMPL OFFSET
     *  Defines the number of unused bits (gap) preceeding this container 
     *  group. This value must be an even number from 0x0 to 0x1e
     */
    uint8_t   smpl_offset;

    /** SMPL TYPE
     *  Defines the sample type for AxC Container Group
     *     7Bit (00) = 7 Bit Samples
     *     8Bit (11) = 8 Bit Samples
     *    15Bit (22) = 15 Bit Samples
     *    16Bit (33) = 16 Bit Samples
     */
    uint8_t   smpl_type;

    /** SMPL LAST
     *  Indicates this is the last AXC Contgainer Group to be used when set. 
     *  All group entries after this one are ignored.
     */
    uint8_t   smpl_last;

} Iqn2Fl_AilPhyLutSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhySetup. This structure is used for
 * configuring PHY Global registers of an AIL instance. The AIL PHY Global Configuration 
 * Register defines the global operation of the AIL PHY. */
typedef struct 
{
    /** The link rate field defines the rate of the link. The field should 
     *  only be updated if the TM Enable and RM Enable bits are 0. These 
     *  link rates control the protocl layer as well (PD and PE). The field 
     *  can be programmed for the following rates: 
     *     8X (00) = 8x Rate
     *     4X (11) = 4x Rate
     *     2X (22) = 2x Rate
     *     5X (33) = 5x Rate (CPRI Only)
     *    10X (44) = 10x Rate (CPRI Only)
     *    16X (55) = 16x Rate (CPRI Only) 
     */
    uint8_t   link_rate;

    /** The OBSAI/CPRI bit defines the mode of operation for the link. 
     *  This field may only be modified while the AIL is idle and may 
     *  not be updated on-the-fly 
     *     CPRI (00) = CPRI Mode - The AIL Phy is configured for CPRI Operation
     *     OBSAI (11) = OBSAI Mode - The AIL Phy is configured for OBSAI Operation
     */
    uint8_t   obsai_cpri;

    /** Short Frame Enable - Short frame enable mode is only used for 
     *  verification purposes (never set this bit in an actual BTS)
     */
    uint32_t   short_frm_en;

} Iqn2Fl_AilPhyGlbCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhySetup. This structure is used for
 * configuring PHY RT (ReTransmitter) registers of an AIL instance. The AIL PHY RT 
 * Configuration Register defines the basic operation of the RT block. */
typedef struct 
{
    /** The RT Config field defines the operation mode of the RT Block
     *     FWDG_MODE (00) = Forwading Mode - The AIL Phy only transmits 
     *                      data that it receives from an external link
     *     AGGR_MODE (11) = Aggregate Mode - Used to combine local SOC 
     *                      generated traffic with traffic recieved from 
     *                      remote device. In this mode, Protocol Layer 
     *                      (PE) controls the aggregation on a stream-by-
     *                      stream basis. (use PE registers to control the 
     *                      operation)
     *     TRANS_MODE (22) = Transmit Mode - Protocol Layyer (PE) data is 
     *                       transmitted. RT does not aggregate any RM data 
     *                       into the TM path.
     *     CPRI_ULNK_MODE (33) = CPRI Uplink RE Chaining Mode - (CPRI Only), 
     *                           AxC: Aggregation mode where RM data is offset 
     *                           into the egress CPRI link by n basic frames. 
     *                           CW: CPRI control words are delayed such that 
     *                           they are re-transmitted into the next hyperframe
     *                          (i.e ingress hyperframe0 maps to egress hyperframe1).  
     */
    uint8_t   config;

    /** (OBSAI Only) Ingress messages are converted to OBSAI Empty Msg when an 
     *  LCV error is detected in the header. (recommended)
     *     DISABLE (00) = Empty Message Insertion Disabled
     *     ENABLE (11) = Empty Message Insertion Enabled
     */
    uint8_t   em_en;

    /** Selects which Ingress link is used (forwarding or aggregation) into 
     *  egress link.
     *     SEL_LINK0 (00) = Select CI Link 0
     *     SEL_LINK1 (11) = Select CI Link 1
     *     SEL_LINK2 (22) = Select CI Link 2
     *     SEL_LINK3 (33) = Select CI Link 3
     *     SEL_LINK4 (44) = Select CI Link 4
     *     SEL_LINK5 (55) = Select CI Link 5
     *     SEL_LINK6 (66) = Select CI Link 6
     *     SEL_LINK7 (77) = Select CI Link 7
     */
    uint8_t   ci_link;

    /** (CPRI Only) Indicates repositioning of ingress AxC traffic into egress 
     *  link due to RE Uplink Chaining. This value is the basic frame number on 
     *  the egress link corresponding to basic frame 0 on the ingress link.
     */
    uint8_t   bf_delay;

} Iqn2Fl_AilPhyRtCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhyCiCoLutCfg. This structure
 * is used for configuring PHY CI and CO LUT A and B register parameters. */
typedef struct 
{
    /** Defines the number of consecutive AxC Containers in the AxC 
     *  Container Group.
     */
    uint8_t   smpl_count;

    /** Defines the number of unused bits (gap) preceeding this container 
     *  group. This value must be an even number from 0x0 to 0x1e.
     */
    uint8_t   smpl_offset;

    /** Defines the sample type for AxC Container Group
     *  7Bit (00) = 7 Bit Samples
     *  8Bit (11) = 8 Bit Samples
     *  15Bit (22) = 15 Bit Samples
     *  16Bit (33) = 16 Bit Samples
     */
    uint8_t   smpl_type;

    /** Indicates this is the last AXC Container Group to be used when 
     *  set. All group entries after this one are ignored.
     */
    uint8_t   smpl_last;

} Iqn2Fl_AilPhyLutParams;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhySetup. This structure
 * is used for configuring PHY CI and CO LUT registers of an AIL instance. */
typedef struct 
{
    /** AIL PHY CI LUT CFG 
     *  The AIL PHY CI LUT Select Register selects between the LUT A table and 
     *  the LUT B table for CI CPRI Conversion control. Used for dynamic 
     *  modification of CI effectively giving the user a Ping Pong buffer. 
     *  Select takes effect on next PHY frame boundary. 
     *  0 Selects Table A, 1 Selects Table B
     */
    uint8_t   phy_ci_lut_cfg_sel;

    /** AIL PHY CO LUT CFG 
     *  The AIL PHY CO LUT Select Register selects between the LUT A table and 
     *  the LUT B table for CO CPRI Conversion control. Used for dynamic 
     *  modification of CO effectively giving the user a Ping Pong buffer. 
     *  Select takes effect on next PHY frame boundary. 
     *  0 Selects Table A, 1 Selects Table B
     */
    uint8_t   phy_co_lut_cfg_sel;

    /** AIL PHY CI LUTA CFG
     *  The AIL PHY CI Look-up Table A Registers control up to eight groups 
     *  of AxC containers for every given CPRI basic frame.
     */
    Iqn2Fl_AilPhyLutParams   ail_phy_ci_luta_cfg[8];

    /** AIL PHY CI LUTB CFG
     *  The AIL PHY CI Look-up Table B Registers control up to eight groups 
     *  of AxC containers for every given CPRI basic frame.
     */
    Iqn2Fl_AilPhyLutParams   ail_phy_ci_lutb_cfg[8];

    /** AIL PHY CO LUTA CFG
     *  The AIL PHY CO Look-up Table A Registers control up to eight groups 
     *  of AxC containers for every given CPRI basic frame.
     */
    Iqn2Fl_AilPhyLutParams   ail_phy_co_luta_cfg[8];

    /** AIL PHY CO LUTB CFG
     *  The AIL PHY CO Look-up Table B Registers control up to eight groups 
     *  of AxC containers for every given CPRI basic frame.
     */
    Iqn2Fl_AilPhyLutParams   ail_phy_co_lutb_cfg[8];

} Iqn2Fl_AilPhyCiCoLutCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhyTmCfg. This structure
 * is used for configuring PHY TM L1 INBAND CFG register parameters. 
 * (CPRI Only) SW supplied L1 inband alarm signals for for insertion into 
 * Egress CPRI link. 
 */
typedef struct 
{
    /** RST (see CPRI spec for usage) */
    uint8_t   l1_inband_rst;

    /** RAI (see CPRI spec for usage) */
    uint8_t   l1_inband_rai;

    /** SDI (see CPRI spec for usage) */
    uint8_t   l1_inband_sdi;

    /** LOS (see CPRI spec for usage) */
    uint8_t   l1_inband_los;

    /** LOF (see CPRI spec for usage) */
    uint8_t   l1_inband_lof;

} Iqn2Fl_AilPhyTmL1InbCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhyTmCfg. This structure
 * is used for configuring PHY TM L1 INBAND EN CFG register parameters. 
 * (CPRI Only) The L1 Inband Enable Register allows hardware control of the L1 
 * inband alarm signals. A 1 for each bit indicates the hardware control is 
 * enabled. Each enable bit is a gate on an input condition that could affect 
 * an output condition. The nomenclature is defined as TXSIGNAL_RXCOND_EN. The 
 * term ERR indicates the error had been determined by the RM, while the term 
 * RX refers to the actual L1 inband signal received by the RM. 
 */
typedef struct 
{
    /** Enables l1_inband RAI active upon rairx signal from RM */
    uint8_t   rai_rairx_en;

    /** Enables l1_inband RAI active upon lofrx signal from RM */
    uint8_t   rai_lofrx_en;

    /** Enables l1_inband RAI active upon loferr signal from RM */
    uint8_t   rai_loferr_en;

    /** Enables l1_inband RAI active upon losrx signal from RM */
    uint8_t   rai_losrx_en;

    /** Enables l1_inband RAI active upon loserr signal from RM */
    uint8_t   rai_loserr_en;

    /** Enables l1_inband LOF active upon lofrx signal from RM */
    uint8_t   lof_lofrx_en;

    /** Enables l1_inband LOF active upon loferr signal from RM */
    uint8_t   lof_loferr_en;

    /** Enables l1_inband LOS active upon losrx signal from RM */
    uint8_t   los_losrx_en;

    /** Enables l1_inband LOS active upon loserr signal from RM */
    uint8_t   los_loserr_en;

    /** Enables l1_inband SDI active upon rstrx signal from RM */
    uint8_t   sdi_rstrx_en;

} Iqn2Fl_AilPhyTmL1InbEnCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhySetup. This structure
 * is used for configuring PHY TM (Tx Mac) configuration registers of an AIL 
 * instance. */
typedef struct 
{
    /** AIL PHY TM CFG 
     *  The TM Configuration Register is used to program basic functionality 
     *  of the Tx Mac Block. Bit 0 can be updated at any time to turn on or 
     *  off the link.
     *  The Transmit Enable Bit allows the TM block transmit state machine 
     *  to operate
     *     DISABLE (00) = TM Block Disabled
     *     ENABLE (11) = TM Block Enabled
     */
    uint8_t   phy_tm_cfg_en;

    /** AIL PHY TM CTRL CFG - FLUSH
     *  TM State Machine Control Register
     *  Instructs the TM link to flush the FIFO and force a TM Fail condition 
     *  on the link. When set high, the State machine is forced into the 
     *  RE_SYNC state
     *     No_Action (00) = No Action
     *     FLUSH_FIFO (11) = Flush FIFO and force TM Fail condition
     */
    uint8_t   phy_tm_ctrl_cfg_flush;

    /** AIL PHY TM CTRL CFG - IDLE
     *  TM State Machine Control Register
     *  Forces the Frame Sync state machine to transition from either the 
     *  FRAME_SYNC or RESYNC states to the IDLE state and remain in the IDLE 
     *  state until the bit is cleared. If the state machine is in the OFF 
     *  state, this bit is ignored until the state machine reaches the IDLE 
     *  state
     *     No_Action (00) = No Action
     *     Idle (11) = Force to Idle state and remain until cleared
     */
    uint8_t   phy_tm_ctrl_cfg_idle;

    /** AIL PHY TM CTRL CFG - RESYNC
     *  TM State Machine Control Register
     *  Forces the Frame Sync state machine to transition from the FRAME_SYNC 
     *  state to the RESYNC state and remain in the RESYNC state until the bit 
     *  is cleared. If the state machine is in either the OFF state or the 
     *  IDLE state, this bit is ignored until the state machine reaches the 
     *  RESYNC state
     *     No_Action (00) = No Action
     *     RESYNC (11) = Forces the Frame Sync state machine to transition 
     *                   from the FRAME_SYNC state to the RESYNC state and 
     *                   remain in the RESYNC state until the bit is cleared
     */
    uint8_t   phy_tm_ctrl_cfg_resync;

    /** AIL PHY TM CTRL CFG - LOS_EN
     *  TM State Machine Control Register
     *  When set, Loss of Signal from the RM to cause the TM state machine to 
     *  transition to the OFF state.
     *     DISABLE (00) = Loss of Signal Disabled
     *     ENABLE (11) = Loss of Signal Enabled
     */
    uint8_t   phy_tm_ctrl_cfg_los_en;

    /** AIL PHY TM SCR CTRL CFG - SEED_VALUE
     *  (OBSAI Only) The Scrambler Configuration Register contains the seed 
     *  initialization vector for the LFSR scrambler utilized when scrambling 
     *  is enabled in OBSAI 8x mode, and the scrambler enable bit is set. This 
     *  configuration register should only be updated when the Frame Sync state 
     *  machine is disabled.
     *  The seed value is used to initialize the transmit scrambler circuit.
     */
    uint8_t   phy_tm_scr_ctrl_cfg_seed_value;

    /** AIL PHY TM SCR CTRL CFG - SCR_EN
     *  (OBSAI Only) The Scrambler Configuration Register contains the seed 
     *  initialization vector for the LFSR scrambler utilized when scrambling 
     *  is enabled in OBSAI 8x mode, and the scrambler enable bit is set. This 
     *  configuration register should only be updated when the Frame Sync state 
     *  machine is disabled.
     *  Enables the scrambler for operation in the transmit data path.
     *     DISABLE (00) = Scrambler Disabled
     *     ENABLE (11) = Scrambler Enabled
     */
    uint8_t   phy_tm_scr_ctrl_cfg_scr_en;

    /** AIL PHY TM L1 INBAND CFG
     *  (CPRI Only) SW supplied L1 inband alarm signals for for insertion 
     *  into Egress CPRI link.
     */
    Iqn2Fl_AilPhyTmL1InbCfg   ail_phy_tm_l1_inb_cfg;

    /** AIL PHY TM L1 INBAND EN CFG
     *  (CPRI Only) The L1 Inband Enable Register allows hardware control of 
     *  the L1 inband alarm signals. A 1 for each bit indicates the hardware 
     *  control is enabled. Each enable bit is a gate on an input condition 
     *  that could affect an output condition. The nomenclature is defined as 
     *  TXSIGNAL_RXCOND_EN. The term ERR indicates the error had been 
     *  determined by the RM, while the term RX refers to the actual L1 inband 
     *  signal received by the RM.
     */
    Iqn2Fl_AilPhyTmL1InbEnCfg   ail_phy_tm_l1_inb_en_cfg;

    /** AIL PHY TM LOSERR SEL CFG - LINK_LOSERR
     *  Selects which RM link is used to drive the LOSERR condition to 
     *  determine transmit L1 Inband signaling.
     *  RM Link LOSERR Select
     *     LOSERR0 (00) = RM Link LOSERR 0
     *     LOSERR1 (11) = RM Link LOSERR 1
     *     LOSERR2 (22) = RM Link LOSERR 2
     *     LOSERR3 (33) = RM Link LOSERR 3
     *     LOSERR4 (44) = RM Link LOSERR 4
     *     LOSERR5 (55) = RM Link LOSERR 5
     *     LOSERR6 (66) = RM Link LOSERR 6
     *     LOSERR7 (77) = RM Link LOSERR 7
     */
    uint8_t   phy_tm_loserr_sel_cfg_link_loserr;

    /** AIL PHY TM LOFERR SEL CFG - LINK_LOFERR
     *  Selects which RM link is used to drive the LOFERR condition to 
     *  determine transmit L1 Inband signaling.
     *  RM Link LOFERR Select
     *     LOFERR0 (00) = RM Link 0
     *     LOFERR1 (11) = RM Link 1
     *     LOFERR2 (22) = RM Link 2
     *     LOFERR3 (33) = RM Link 3
     *     LOFERR4 (44) = RM Link 4
     *     LOFERR5 (55) = RM Link 5
     *     LOFERR6 (66) = RM Link 6
     *     LOFERR7 (77) = RM Link 7
     */
    uint8_t   phy_tm_loferr_sel_cfg_link_loferr;

    /** AIL PHY TM LOSRX SEL CFG - LINK_LOSRX
     *  Selects which RM link is used to drive the LOSRx condition to 
     *  determine transmit L1 Inband signaling.
     *  RM Link LOSRx Select
     *     LOSRx0 (00) = RM Link LOSRx 0
     *     LOSRx1 (11) = RM Link LOSRx 1
     *     LOSRx2 (22) = RM Link LOSRx 2
     *     LOSRx3 (33) = RM Link LOSRx 3
     *     LOSRx4 (44) = RM Link LOSRx 4
     *     LOSRx5 (55) = RM Link LOSRx 5
     *     LOSRx6 (66) = RM Link LOSRx 6
     *     LOSRx7 (77) = RM Link LOSRx 7
     */
    uint8_t   phy_tm_losrx_sel_cfg_link_losrx;

    /** AIL PHY TM LOFRX SEL CFG - LINK_LOFRX
     *  Selects which RM link is used to drive the LOFRx condition to 
     *  determine transmit L1 Inband signaling.
     *  RM Link LOFRx Select
     *     LOFRx0 (00) = RM Link LOFRx 0
     *     LOFRx1 (11) = RM Link LOFRx 1
     *     LOFRx2 (22) = RM Link LOFRx 2
     *     LOFRx3 (33) = RM Link LOFRx 3
     *     LOFRx4 (44) = RM Link LOFRx 4
     *     LOFRx5 (55) = RM Link LOFRx 5
     *     LOFRx6 (66) = RM Link LOFRx 6
     *     LOFRx7 (77) = RM Link LOFRx 7
     */
    uint8_t   phy_tm_lofrx_sel_cfg_link_lofrx;

    /** AIL PHY TM RAIRX SEL CFG - LINK_RAIRX
     *  Selects which RM link is used to drive the RAIRx condition to 
     *  determine transmit L1 Inband signaling.
     *  RM Link RAIRx Select
     *     RAIRx0 (00) = RM Link RAIRx 0
     *     RAIRx1 (11) = RM Link RAIRx 1
     *     RAIRx2 (22) = RM Link RAIRx 2
     *     RAIRx3 (33) = RM Link RAIRx 3
     *     RAIRx4 (44) = RM Link RAIRx 4
     *     RAIRx5 (55) = RM Link RAIRx 5
     *     RAIRx6 (66) = RM Link RAIRx 6
     *     RAIRx7 (77) = RM Link RAIRx 7
     */
    uint8_t   phy_tm_rairx_sel_cfg_link_rairx;

    /** AIL PHY TM RSTRX SEL CFG - LINK_RSTRX
     *  Selects which RM link is used to drive the RSTRx condition to 
     *  determine transmit L1 Inband signaling.
     *  RM Link RSTRx Select
     *     RSTRx0 (00) = RM Link RSTRx 0
     *     RSTRx1 (11) = RM Link RSTRx 1
     *     RSTRx2 (22) = RM Link RSTRx 2
     *     RSTRx3 (33) = RM Link RSTRx 3
     *     RSTRx4 (44) = RM Link RSTRx 4
     *     RSTRx5 (55) = RM Link RSTRx 5
     *     RSTRx6 (66) = RM Link RSTRx 6
     *     RSTRx7 (77) = RM Link RSTRx 7
     */
    uint8_t   phy_tm_rstrx_sel_cfg_link_rstrx;

    /** AIL PHY TM CPRI PTRP CFG - PTR_P
     *  L1 inband field. Pointer P value which is inserted into the CPRI 
     *  Egress link.
     */
    uint8_t   phy_tm_cpri_ptrp_cfg_ptr_p;

    /** AIL PHY TM CPRI STARTUP CFG - STARTUP
     *  Contains Startup value.
     *  L1 inband field. Startup value which is inserted into the CPRI 
     *  Egress link.
     */
    uint8_t   phy_tm_cpri_startup_cfg_startup;

    /** AIL PHY TM CPRI VERSION CFG - PROT_VERS
     *  L1 inband field. Protocol version value which is inserted into the CPRI 
     *  Egress link.
     */
    uint8_t   phy_tm_cpri_version_cfg_prot_vers;

    /** AIL PHY TM CPRI PORTID A CFG - Z52_0
     *  TM CPRI PORT ID bits 31 to 0 which are inserted into the CPRI Egress 
     *  Link at specific CPRI CW.
     *  Contains the Z.52.0 byte transmitted CPRI Port ID value
     */
    uint8_t   phy_tm_cpri_portid_a_cfg_z52_0;

    /** AIL PHY TM CPRI PORTID A CFG - Z52_1
     *  TM CPRI PORT ID bits 31 to 0 which are inserted into the CPRI Egress 
     *  Link at specific CPRI CW.
     *  Contains the Z.52.1 byte transmitted CPRI Port ID value
     */
    uint8_t   phy_tm_cpri_portid_a_cfg_z52_1;

    /** AIL PHY TM CPRI PORTID A CFG - Z116_0
     *  TM CPRI PORT ID bits 31 to 0 which are inserted into the CPRI Egress 
     *  Link at specific CPRI CW.
     *  Contains the Z.116.0 byte transmitted CPRI Port ID value
     */
    uint8_t   phy_tm_cpri_portid_a_cfg_z116_0;

    /** AIL PHY TM CPRI PORTID A CFG - Z116_1
     *  TM CPRI PORT ID bits 31 to 0 which are inserted into the CPRI Egress 
     *  Link at specific CPRI CW.
     *  Contains the Z.116.1 byte transmitted CPRI Port ID value
     */
    uint8_t   phy_tm_cpri_portid_a_cfg_z116_1;

    /** AIL PHY TM CPRI PORTID B CFG - Z180_0
     *  TM CPRI PORT ID bits 63 to 32 which are inserted into the CPRI Egress 
     *  Link at specific CPRI CW.
     *  Contains the Z.180.0 byte transmitted CPRI Port ID value
     */
    uint8_t   phy_tm_cpri_portid_b_cfg_z180_0;

    /** AIL PHY TM CPRI PORTID B CFG - Z180_1
     *  TM CPRI PORT ID bits 63 to 32 which are inserted into the CPRI Egress 
     *  Link at specific CPRI CW.
     *  Contains the Z.180.1 byte transmitted CPRI Port ID value
     */
    uint8_t   phy_tm_cpri_portid_b_cfg_z180_1;

    /** AIL PHY TM CPRI PORTID B CFG - Z244_0
     *  TM CPRI PORT ID bits 63 to 32 which are inserted into the CPRI Egress 
     *  Link at specific CPRI CW.
     *  Contains the Z.244.0 byte transmitted CPRI Port ID value
     */
    uint8_t   phy_tm_cpri_portid_b_cfg_z244_0;

    /** AIL PHY TM CPRI PORTID B CFG - Z244_1
     *  TM CPRI PORT ID bits 63 to 32 which are inserted into the CPRI Egress 
     *  Link at specific CPRI CW.
     *  Contains the Z.244.1 byte transmitted CPRI Port ID value
     */
    uint8_t   phy_tm_cpri_portid_b_cfg_z244_1;

    /** AIL PHY TM CPRI SCR CTRL CFG - SEED_VALUE
     *  The Scrambler Configuration Register contains the seed initialization 
     *  vector for the CPRI LFSR scrambler utilized in CPRI 8x,10x,16x mode. 
     *  This configuration register should only be updated when the TM Frame 
     *  Sync state machine is disabled.
     *  The seed value is used to initialize the transmit scrambler circuit.
     */
    uint32_t   phy_tm_cpri_scr_ctrl_cfg_seed_value;

} Iqn2Fl_AilPhyTmCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhyRmCfg. This structure
 * is used for configuring PHY RM (Rx Mac) Data Path registers of an AIL 
 * instance. */
typedef struct 
{
    /** Force RM receiver state machine state
     *     ST4 (22) = Force ST4 state
     *     ST5 (33) = Force ST5 state
     *     ST0 (44) = Force ST0 state
     *     ST1 (55) = Force ST1 state
     *     ST2 (66) = Force ST2 state
     *     ST3 (77) = Force ST3 state
     *     OFF (00) = Force state OFF
     */
    uint8_t   force_rx_state;

    /** Suppress error reporting when the receiver state machine is not in 
     *  state ST3
     *     Allow (00) = Allow all RM error reporting when not in ST3
     *     Suppress (11) = Suppress all RM error reporting when not in ST3
     */
    uint8_t   error_suppress;

    /** Enables the RM to automatically disable Serdes symbol alignment when 
     *  the receiver state machine reaches state ST3
     *     Disable (00) = Disable auto alignment
     *     Enable (11) = Enable auto alignment
     */
    uint8_t   sd_auto_align_en;

    /** Enables a state transition from the ST3 to the ST0 state when 
     *  lcv_det_thold is met
     *     No_effect (00) = lcv_det_thold has no effect on the RX FSM
     *     Enable_transition (11) = The RX FSM transitions from ST3 to 
     *                              ST0 when lcv_det_thold is met
     */
    uint8_t   lcv_unsync_en;

} Iqn2Fl_AilPhyRmDpCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhyRmCfg. This structure
 * is used for configuring PHY RM Clock Detection configuration registers 
 * of an AIL instance. */
typedef struct 
{
    /** Enables the clock detect watch dog timer.
     *     Disabled_Cleared (00) = Clock detect watch dog timer Disabled.
     *     Enabled (11) = Clock detect watch dog timer Enabled.
     */
    uint8_t   wd_en;

    /** Enables the clock quality circuit.
     *     Disabled_Cleared (00) = Clock qulity circuit Disabled.
     *     Enabled (11) = Clock qulity circuit Enabled.
     */
    uint8_t   cq_en;

    /** Defines the wrap value of the clock detection watchdog circuit. A 
     *  value of zero disables the clock watchdog timer, Range 0 to 255.
     */
    uint8_t   wd_wrap;

    /** Defines the wrap value of the clock monitor used to define clock 
     *  quality. A value of zero disables the clock monitor, Range 0 to 65,535.
     */
    uint16_t   mon_wrap;

} Iqn2Fl_AilPhyRmClkDetCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilPhySetup. This structure
 * is used for configuring PHY RM (Rx Mac) configuration registers of an AIL 
 * instance. */
typedef struct 
{
    /** AIL PHY RM CFG - RX_EN
     *  RM Configuration Register.
     *  Enable RM Link FSM to activate on next recieved PHY frame boundary
     *     Disable (00) = RM link disable
     *     Enable (11) = RM link enable
     */
    uint8_t   phy_rm_cfg_rx_en;

    /** AIL PHY RM CFG - DATA_TRC_SEL
     *  RM Configuration Register.
     *  Data Trace Select. Selects between the raw data after 8b/10b decoding 
     *  and framed data after the frame sync state machine and de-scrambling 
     *  for Data Trace data
     *     Raw_data_sel (00) = Selects raw data after 8b/10b for data trace
     *     Frm_data_sel (11) = Selects framed data after de-scrambling for 
     *                         data trace, data is not available unless the Rx 
     *                         fsm in in Frame Sync
     */
    uint8_t   phy_rm_cfg_data_trc_sel;

    /** AIL PHY RM CFG - RST_EN
     *  RM Configuration Register.
     *  Enable CPRI SOC Reset Signal. 5 consecutive recieved active 
     *  L1_Inband_rst will fire a signal to SOC core which can be enabled with 
     *  an LPSC to perform HW reset of entire SOC
     *     Disable (00) = CPRI SOC Reset Disable
     *     Enable (11) = CPRI SOC Reset Enable
     */
    uint8_t   phy_rm_cfg_rst_en;

    /** AIL PHY RM DP CFG
     *  Controls the RM Data Path Configuration.
     */
    Iqn2Fl_AilPhyRmDpCfg   phy_rm_dp_cfg;

    /** AIL PHY RM LCV CTRL CFG - EN
     *  Controls the counting of LCV Errors in the RM.
     *  Writing a 1 to the bit will enable the Line Code Violation counter. 
     *  This 16 bit counter will saturate when it reaches a value of 0xffff. 
     *  Writing a 0 to this bit will clear and disable the counter. The 
     *  current counter value is available as status, lcv_cntr_value
     *     Disabled_Cleared (00) = lcv_cntr disabled adn cleared to a value 
     *                             of 0x0000.
     *     Enabled (11) = lcv_cntr enabled, counts each LCV to a max of 0xffff
     */
    uint8_t   phy_rm_lcv_ctrl_cfg_en;

    /** AIL PHY RM LCV CTRL CFG - LOS_DET_THOLD
     *  Controls the counting of LCV Errors in the RM.
     *  Sets 8b10b los detect threshold values in number of Line Code 
     *  Violations received during a master frame, OBSAI, or during a 
     *  Hyperframe, CPRI. Writing to this location will automatically 
     *  clear the num_los counter and num_los_det status bit. Range 0 
     *  to 65,535.
     */
    uint16_t   phy_rm_lcv_ctrl_cfg_los_det_thold;

    /** AIL PHY RM FSM SYNC CFG - SYNC_T
     *  RM FSM Sync Count Configuration Register
     *  Threshold value for consecutive valid blocks of bytes which result 
     *  in state ST1. Range 0 to 65,535.
     */
    uint16_t   phy_rm_fsm_sync_cfg_sync_t;

    /** AIL PHY RM FSM SYNC CFG - FRAME_SYNC_T
     *  RM FSM Sync Count Configuration Register
     *  Threshold value for consecutive valid message groups which result 
     *  in state ST3. Range 0 to 65,535.
     */
    uint16_t   phy_rm_fsm_sync_cfg_frame_sync_t;

    /** AIL PHY RM FSM UNSYNC CFG - UNSYNC_T
     *  RM FSM Unsync Count Configuration Register
     *  Threshold value for consecutive invalid blocks of bytes which result 
     *  in state ST0. Range 0 to 65,535.
     */
    uint16_t   phy_rm_fsm_unsync_cfg_unsync_t;

    /** AIL PHY RM FSM UNSYNC CFG - FRAME_UNSYNC_T
     *  RM FSM Unsync Count Configuration Register
     *  Threshold value for consecutive invalid message groups which result 
     *  in state ST1. Range 0 to 65,535.
     */
    uint16_t   phy_rm_fsm_unsync_cfg_frame_unsync_t;

    /** AIL PHY RM DSCR CTRL CFG - SCR_EN
     *  Controls the descrambling operation of the RM.
     *  Enables the scrambler for operation in the receiver data path
     *     Disable (00) = RM scrambler Disabled
     *     Enable (11) = RM scrambler Enabled
     */
    uint8_t   phy_rm_dscr_ctrl_cfg_scr_en;

    /** AIL PHY RM CLK DET CFG
     *  RM Clock Detection Configuration.
     */
    Iqn2Fl_AilPhyRmClkDetCfg   phy_rm_clk_det_cfg;

} Iqn2Fl_AilPhyRmCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilSetup. This structure is
 * used for configuring UAT params of an AIL instance */
typedef struct 
{
    /** AIL UAT GEN CTL -> AIL UAT BCN TERMINAL COUNT REGISTER. 
     *  UAT BCN terminal count. BCN counts from zero to this limit 
     *  and wraps to zero. Program as 2,457,599 for sys_clk=245.76MHz 
     *  and 3,071,999 for sys_clk=307.2MHz.
     */
    uint32_t   ail_uat_bcn_tc_cfg_val;

    /** AIL UAT GEN CTL -> AIL UAT BCN OFFSET REGISTER. 
     *  Offset correction to the raw uAT BCN counter. Used to correct 
     *  the alignment of the local uAT BCN to the master AT2 BCN. BCN is 
     *  initially randomly started. SW uses uat_sync_bcn_capture_sts 
     *  rd_val to calculate offset correction factor. This correction 
     *  factor will be Frame size - captured value.
     */
    uint32_t   ail_uat_bcn_offset_cfg_val;

    /** AIL UAT AIL REGS -> AIL UAT PIMAX CFG REGISTER. 
     *  (AIL use only) PI max window. One of two values which indicate 
     *  the legal range for OBSAI or CPRI PHY SOF. When an SOF is recieved 
     *  outside this window, and error is indicated (EE).
     */
    uint32_t   ail_uat_pimax_cfg_val;

    /** AIL UAT AIL REGS -> AIL UAT PIMIN CFG REGISTER. 
     *  (AIL use only) PI min window. One of two values which indicate 
     *  the legal range for OBSAI or CPRI PHY SOF. When an SOF is recieved 
     *  outside this window, and error is indicated (EE).
     */
    uint32_t   ail_uat_pimin_cfg_val;

    /** AIL UAT AIL REGS -> AIL UAT TM FRAME COUNT (BFN) CONFIG REGISTER. 
     *  (AIL CPRI use only) uAT CPRI BFN count value (Write only). SW 
     *  overwrite current value. uAT will increment every TM_FRM_STB.
     *  Use hIqn2->arg_ail to select AIL instance (argument type: uint16_t *) 
     */
    uint16_t   ail_uat_tm_bfn_cfg_wr_val;

    /** AIL UAT AIL REGS -> AIL UAT RT FRAME BOUNDARY (FB) COMPARE REGISTER. 
     *  (AIL use only) uAT BCN compare value which cause RT_STB to fire. 
     *  Used for Egress PHY timing. RT_STB is the latest moment that RT 
     *  will wait for PE and CI SOF contribution before progressing without 
     *  either input.
     */
    uint32_t   ail_uat_rt_fb_cfg_val;

    /** AIL UAT AIL REGS -> AIL UAT PE FRAME BOUNDARY (FB) COMPARE REGISTER. 
     *  (AIL use only) uAT BCN compare value which cause PE_STB to fire. 
     *  Used for Egress PROTO & PHY timing. PE_STB is the exact time which
     *  PE will start building the PHY protocol. It also represents the 
     *  latest timing for incoming DMA data to contribute to the PHY. Late 
     *  DMA data is rejected by PE.
     */
    uint32_t   ail_uat_pe_fb_cfg_val;

    /** AIL UAT AIL REGS -> AIL UAT TM FRAME BOUNDARY (FB) COMPARE REGISTER. 
     *  (AIL use only) uAT BCN compare value which cause RT_STB to fire. 
     *  Used for Egress PHY timing. TM_STB (OBSAI Delta) is the precise 
     *  time at which TM exports the CPRI or OBSAI PHY SOF.
     */
    uint32_t   ail_uat_tm_fb_cfg_val;

    /** AIL UAT EGR RADT -> AIL UAT RADT TERMINAL COUNT REGISTER. 
     *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with 
     *  sys_clk=245.76MHz).
     */
    uint32_t   ail_uat_egr_radt_tc_cfg_val[8];

    /** AIL UAT EGR RADT -> AIL UAT RADT OFFSET REGISTER. 
     *  UAT RADT offset. Value which is added to the raw RADT 
     *  as a timing correction. RadT is initially randomly started, 
     *  SW uses radt_capture value to calculate offset correction factor.
     *  This correction factor will be Frame size - captured value.
     */
    uint32_t   ail_uat_egr_radt_offset_cfg_val[8];

    /** AIL UAT ING RADT -> AIL UAT RADT TERMINAL COUNT REGISTER. 
     *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with 
     *  sys_clk=245.76MHz).
     */
    uint32_t   ail_uat_ing_radt_tc_cfg_val[8];

    /** AIL UAT ING RADT -> AIL UAT RADT OFFSET REGISTER. 
     *  UAT RADT offset. Value which is added to the raw RADT 
     *  as a timing correction. RadT is initially randomly started, 
     *  SW uses radt_capture value to calculate offset correction factor.
     *  This correction factor will be Frame size - captured value.
     */
    uint32_t   ail_uat_ing_radt_offset_cfg_val[8];

    /** AIL UAT RADT EVT -> AIL UAT RADT EVENT COMPARE REGISTER. 
     *  UAT RADT event compare per RADT. When compare value equals RADT 
     *  count, frame rate event is generated. Also periodic event (i.e. 4SAMP)
     *  is started. The 0 to 7 are for si egress, 8 to 15 for si ingress, 
     *  16 to 18 for dio egress, 19 to 21 for dio ingress.
     */
    uint32_t   ail_uat_evt_radt_cmp_cfg_val[22];

    /** AIL UAT RADT EVT -> AIL UAT RADT EVENT CLOCK COUNT TC REGISTER. 
     *  UAT RADT event clock counter terminal count controls spacing of the 
     *  periodic strobe (i.e. 4SAMP). Once the uat_evt_radt_cmp_cfg equals 
     *  the RADT, the period strobe will fire and re-fire every time a 
     *  clock counter reaches this terminal count. The The 0 to 7 are for si 
     *  egress, 8 to 15 for si ingress, 16 to 18 for dio egress, 19 to 21 
     *  for dio ingress.
     */
    uint16_t   ail_uat_evt_clk_cnt_tc_cfg_val[22];

} Iqn2Fl_AilUatSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_AilSetup. This structure is used for
 * configuring PHY params like Global, RT, CI/CO LUTs, TM and RM of an AIL instance */
typedef struct 
{
    /** AIL PHY GLB CFG 
     *  The AIL PHY Global Configuration Register defines the global operation 
     *  of the AIL PHY.
     */
    Iqn2Fl_AilPhyGlbCfg   ail_phy_glb_cfg;

    /** AIL PHY RT CFG 
     *  The AIL PHY RT (ReTransmitter) Configuration Register defines the basic 
     *  operation of the RT block.
     */
    Iqn2Fl_AilPhyRtCfg   ail_phy_rt_cfg;

    /** AIL PHY CI and CO LUT CFG */
    Iqn2Fl_AilPhyCiCoLutCfg   ail_phy_ci_co_lut_cfg;

    /** AIL PHY TM (Tx Mac) Registers */
    Iqn2Fl_AilPhyTmCfg   ail_phy_tm_regs;

    /** AIL PHY RM (Rx Mac) Registers */
    Iqn2Fl_AilPhyRmCfg   ail_phy_rm_regs;

} Iqn2Fl_AilPhySetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * configuring the parameters of an AIL instance */
typedef struct 
{
    /** AIL instance number (refer @a Iqn2Fl_AilInstance) */
    Iqn2Fl_AilInstance             ailInstNum;

    /** pointer to AIL Egress setup */
    Iqn2Fl_AilEgrSetup            *pAilEgrSetup;

    /** pointer to AIL Ingress setup */
    Iqn2Fl_AilIgrSetup            *pAilIgrSetup;

    /** pointer to AIL Protocol Encoder (PE) setup */
    Iqn2Fl_AilPeSetup             *pAilPeSetup;

    /** pointer to AIL Protocol Decoder (PD) setup */
    Iqn2Fl_AilPdSetup             *pAilPdSetup;

    /** pointer to AIL Micro Antenna Timer (UAT) setup */
    Iqn2Fl_AilUatSetup            *pAilUatSetup;

    /** pointer to AIL PHY (Global, RT, CI/CO LUT, TM, RM) setup */
    Iqn2Fl_AilPhySetup            *pAilPhySetup;

} Iqn2Fl_AilSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_TopVcSysStsSetup. This structure is used for
 * configuring VC Sys Sts SW Reset Stb parameters of a Top level registers */
typedef struct 
{
    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    uint8_t   sw_reset;

    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    uint8_t   sw_reset_aid;

    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    uint8_t   sw_reset_dio;

    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    uint8_t   sw_reset_pktdma;

    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    uint8_t   sw_reset_ail0;

    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    uint8_t   sw_reset_ail1;

    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    uint8_t   sw_reset_ail2;

    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    uint8_t   sw_reset_ail3;

} Iqn2Fl_TopVCSwResetStbSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_TopSetup. This structure is used for
 * configuring VC Sys Sts parameters of Top level registers */
typedef struct 
{
    /** VC SCRATCH - VC scratch register.
     *  This is the VC Scratch Field used for debug testing basic 
     *  read and write to mmrs.
     */
    uint32_t   scratch;

    /** VC SW RESET STB - VC software reset register.
     *  These are software resets which reset various sections of IQN2.
     */
    Iqn2Fl_TopVCSwResetStbSetup   vc_sys_sts_sw_reset_stb;

    /** VC EMU CFG - IQN2 Emulation Control Register.
     *  FREERUN bit
     *     RESPOND (0) = IQN2 responds to the emulation suspend signal 
     *                   it is monitoring. 
     *     IGNORE (1) = IQN2 ignores emulation suspend signals and runs 
     *                  to completion.
     */
    uint8_t   freerun;

    /** VC EMU CFG - IQN2 Emulation Control Register.
     *  SOFT bit. This bit is ignored by the IQN2. IQN2 always performs 
     *  a graceful SOFT stop. 
     *     HARD_STOP (0) = Value ignored by IQN2
     *     SOFT_STOP (1) = Value ignored by IQN2
     */
    uint8_t   soft;

    /** VC EMU CFG - IQN2 Emulation Control Register.
     *  RT_SEL bit
     *     emu_dbgsusp (0) = IQN2 emulation mode is controlled only by 
     *                       the iqn2_emu_dbgsusp CBA signal. The 
     *                       iqn2_emu_dbgsusp_rt signal is ignored.
     *     emu_dbgsusp_rt (1) = IQN2 emulation mode is controlled only by 
     *                          the iqn2_emu_dbgsusp_rt CBA signal. The 
     *                          iqn2_emu_dbgsusp signal is ignored.
     */
    uint8_t   rt_sel;

    /** VC EMU CFG - IQN2 Emulation Control Register.
     *  Forces the shutdown controller to shut down the entire IQN cleanly.
     *     RUN (0) = IQN2 allowed to run.
     *     SHUTDOWN (1) = IQN2 shuts down.
     */
    uint8_t   frc_shutdown;

    /** VC DTMUX CFG - IQN2 data trace mux select register.
     *  Iqn2 data trace mux select register selects which ail data 
     *  trace to use.
     */
    uint8_t   vc_dtmux;

    /** VC CLKCTL SYSCLK CFG - VC clkctl system clock control register.
     *  SD system clock control Register selects which SERDES txbclk to use 
     *  for sys_clk. Bits 0 and 1 select which serdes lane to use. Bit 2 
     *  selects 1/2 the txbclk speed when set.
     */
    uint8_t   sysclk_sel;

    /** VC CLKCTL SYSCLK CFG - VC clkctl system clock control register.
     *  Selects dfe_pll_clk for the at and uat clock if =1. The selected 
     *  sysclk derived from the SERDES TXBCLK is used when = 0. This should 
     *  only be set when using DFE and not using the AIL.
     */
    uint8_t   at_dfe_clk_sel;

} Iqn2Fl_TopVcSysStsSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_TopSetup. This structure is used for
 * configuring PS Reformater parameters of Top level registers */
typedef struct 
{
    /** PSR EGR CFG
     *  Selects the setting of the amount of bandwidth limiting to use.
     *  Percent bandwidth limit setting for flushing a packet from a channel 
     *  in the PKTDMA. 0 is 1/16 of VBUS bandwidth. Should be set to the 
     *  maximum VBUS bandwidth expected to be used in normal operation 
     *  rounded up to the next faster bandwidth setting.
     */
    uint8_t   bw_limit;

    /** PSR ING CHAN CFG
     *  Controls extraction of PS_DATA from the ingres INFO word. 
     *  Selects if the SI PS_FLAG bit 0 generates a PS_DATA cycle with the 
     *  data coming from the dst_tag field of the INFO word.
     *     enable (00) = PSR Expansion is controlled by SI. Typically AxC 
     *                   channels result in chan_num and sym_index written 
     *                   as PS_DATA and there is no PS_DATA transfer for 
     *                   control channels.(Currently always leave at 0.)
     *     disable (11) = PSR Expansion is disabled, AxC chan_num and sym_index 
     *                    are always written to DST_TAG. This mode will use less 
     *                    BW on PktDMA engine bus. (For TI future use)
     */
    uint8_t   ps_data_ext;

    /** PSR ING CHAN CFG
     *  Selects if a packet with an error generates a flush or a drop.
     *  Normally set to 1. Drops the corrupted packet when FLUSH during 
     *  mid packet. FLUSH likely caused by PktDMA buffer overflow for a 
     *  given channel. (FLUSH also caused by force_flush)
     */
    uint8_t   drop_pkt[48];

    /** PSR ING CHAN CFG
     *  Selects if a packet with an error generates a flush or a drop.
     *  (For TI use only) set to 0. Forces the channel into continuous 
     *  FLUSH. FLUSH operation will EOP any mid-flight packet and discard 
     *  all new traffic for a given channel. Only known use case of this 
     *  field is to support partial resets of i.e. individual links 
     *  (advanced operation which is not officially supported)
     */
    uint8_t   force_flush[48];

    /** PSR EGR CHAN CFG
     *  Enables packing PS data into the dst_tag of the INFO word for 
     *  the channel. Sets the arbitration priority for the channel.
     *  Packs the PS_DATA into the dst_tag field of the INFO word when 
     *  set. When not set PS_DATA is passed as a PS_DATA cycle with no 
     *  modification of the INFO word dst_tag field. Only IQ data uses 
     *  PS_DATA cycles and only OBSAI needing a 24 bit address passes 
     *  PS_DATA without moving it into the dst_tag.
     *     leave_dst_tag (0) = Does not pack PS_DATA into the dst_tag 
     *                         data. Normally used on CTL channels.
     *     replace_dst_tag (1) = Replaces the dst_tag with PS_DATA. 
     *                           Normally used on IQ channels.
     */
    uint8_t   pack_ps_data[48];

    /** PSR EGR CHAN CFG
     *  Sets the arbitration priority for the channel.
     *  Arbitration priority for the channel on the PKTDMA PSI bus. 
     *  0 is highest priority.
     */
    uint8_t   arb_priority[48];

} Iqn2Fl_TopPsrConfigSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * configuring the parameters of Top level registers */
typedef struct 
{
    /** pointer to Top VC Sys Status setup */
    Iqn2Fl_TopVcSysStsSetup   top_vc_sys_sts_cfg;

    /** pointer to Top PSR Config setup */
    Iqn2Fl_TopPsrConfigSetup   top_psr_cfg;

} Iqn2Fl_TopSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Aid2Setup. This structure is
 * used for configuring AID2 EFE Config Group registers */
typedef struct 
{
    /** EFE CHAN CFG - CHAN_EN
     *  Enable channel
     *     ENABLED (1) = Enable channel
     *     DISABLED (0) = Disable channel
     */
    uint8_t   chan_en[32];

    /** EFE CHAN CFG - CHAN_TDD_FRC_OFF
     *  Alternate TDD mode for controlling TDD also used for GSM Base 
     *  Band Hopping. APP SW controls updates this bit each symbol of 
     *  time to control whether the next symbol will be TDD OFF. TDD 
     *  OFF channels generate no Ingress DMA traffic and expect no 
     *  Egress DMA traffic. Zeros are sent over the PHY. In BBHop mode, 
     *  the same applies and in OBSAI, empty_msg is sent over the PHY 
     *  instead of zero'ed traffic 
     *     FRC_SYM_OFF (1) = Force symbols off
     *     NO_FRC_OFF_SYM (0) = No forcing off of symbols
     */
    uint8_t   chan_tdd_frc_off[32];

    /** EFE CHAN CFG - CHAN_RADIO_SEL
     *  Assigns each channel to one of eight radio standard groups. i.e. 
     *  radio standard 0 may be LTE 20MHz
     *     RS0 (0) = Radio Standard 0
     *     RS1 (1) = Radio Standard 1
     *     RS2 (2) = Radio Standard 2
     *     RS3 (3) = Radio Standard 3
     *     RS4 (4) = Radio Standard 4
     *     RS5 (5) = Radio Standard 5
     *     RS6 (6) = Radio Standard 6
     *     RS7 (7) = Radio Standard 7
     */
    Iqn2Fl_ChanRadioSel   chan_radio_sel[32];

    /** EFE CFG - LOOPBACK_EN
     *  (TI use Only) 0x1: Ingress data from ICC is looped back to Egress 
     *  data to ICC. DMA traffic is unused. (i.e. for purpose of early DFE 
     *  only testing)
     */
    uint8_t   loopback_en;

} Iqn2Fl_Aid2IqEfeCfgGrpSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Aid2Setup. This structure is
 * used for configuring AID2 EFE Radio Standard Group registers */
typedef struct 
{
    /** EFE FRM TC CFG - SYM_TC
     *  efe Frame Count Register. There are 8 sets of these values in 
     *  order to support 8 simultaneous radio standard variants.
     *  Radio Framing Counter. Symbol Count. Number of symbols per frame 
     *  programmed as a terminal count.
     */
    uint8_t   sym_tc[8];

    /** EFE FRM TC CFG - INDEX_SC
     *  efe Frame Count Register. There are 8 sets of these values in 
     *  order to support 8 simultaneous radio standard variants.
     *  Radio Framing Counter. Index Counter Starting Location. Starting 
     *  location of the Sample Terminal Count LUT loaded into the Index 
     *  Counter when it first starts and each time it wraps. Depending 
     *  on the radio standard, the index will wrap once per radio frame 
     *  such as WCDMA or muliple times per frame as in LTE. Index is the 
     *  address for XXX_IQ_EFE_FRM_SAMP_TC.
     */
    uint8_t   index_sc[8];

    /** EFE FRM TC CFG - INDEX_TC
     *  efe Frame Count Register. There are 8 sets of these values in 
     *  order to support 8 simultaneous radio standard variants.
     *  Radio Framing Counter. Index Counter Terminal Count. Index counter 
     *  terminal count which is the last value of the Index Counter before 
     *  it wraps. For simple use case, program same as frm_sym_tc.
     */
    uint8_t   index_tc[8];

    /** EFE RAD STD CFG - TDD_FIRST_SYM
     *  efe Radio Standard Configuration Register.
     *  Selects first symbol to start TDD
     */
    uint8_t   tdd_first_sym[8];

    /** EFE RAD STD CFG - TDD_LUT_EN
     *  efe Radio Standard Configuration Register.
     *  Enable TDD
     *     ENABLED (1) = TDD enabled for this radio standard
     *     DISABLED (0) = TDD disabled for this radio standard
     */
    uint8_t   tdd_lut_en[8];

    /** EFE TDD EN CFG0/7 - TDD_EN
     *  Per symbol enables for TDD operation for Radio Standard 0 to 7. 
     *  5 MMRs of 32 bits each can accommodate 160 symbols. Enable 
     *  for symbol 0 in bit 0 of first MMR and enable for symbol 
     *  159 in bit 31 of last MMR.
     *  enables/disables DMA of whole symbols (PktDMA packets). 
     *  Program as 0xffff for most applications.
     *     SYM_ON (1) = symbol dma enabled
     *     SYM_OFF (0) = symbol dma disabled
     */
    uint32_t   tdd_en_cfg[8][5];

} Iqn2Fl_Aid2IqEfeRadioStdGrpSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Aid2Setup. This structure is
 * used for configuring AID2 IFE Channel Configuration Group registers */
typedef struct 
{
    /** IFE CHAN CFG - CHAN_EN
     *  Enable channel
     *     ENABLED (1) = Enable channel
     *     DISABLED (0) = Disable channel
     */
    uint8_t   chan_en;

    /** IFE CHAN CFG - CHAN_AXC_OFFSET
     *  AxC Offset
     *     NONE (0) = No offset
     *     ONE (1) = One sample offset
     *     TWO (2) = Two sample offset
     *     THREE (3) = Three sample offset
     */
    uint8_t   chan_axc_offset;

    /** IFE CHAN CFG - CHAN_RADIO_SEL
     *  Radio Standard Select
     *     RS0 (0) = Radio Standard 0
     *     RS1 (1) = Radio Standard 1
     *     RS2 (2) = Radio Standard 2
     *     RS3 (3) = Radio Standard 3
     *     RS4 (4) = Radio Standard 4
     *     RS5 (5) = Radio Standard 5
     *     RS6 (6) = Radio Standard 6
     *     RS7 (7) = Radio Standard 7
     */
    Iqn2Fl_ChanRadioSel   chan_radio_sel;

    /** IFE CHAN CFG - CHAN_TDD_FRC_OFF
     *  Forces a channel into the TDD OFF state on the next symbol after 
     *  it is set to a 1 regardless of the TDD configuration of the radio 
     *  standard variant the channel is assigned to.
     *     FRC_SYM_OFF (1) = Force symbols off
     *     NO_FRC_OFF_SYM (0) = No forcing off of symbols
     */
    uint8_t   chan_tdd_frc_off;

} Iqn2Fl_Aid2IqIfeChanCfgGrpSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Aid2Setup. This structure is
 * used for configuring AID2 IFE Radio Standard Group registers */
typedef struct 
{
    /** IFE FRM TC CFG - SYM_TC
     *  IFE Frame Count Register. There are 8 sets of these values in 
     *  order to support 8 simultaneous radio standard variants.
     *  Radio Framing Counter. Symbol Count. Number of symbols per frame 
     *  programmed as a terminal count.
     */
    uint8_t   sym_tc[8];

    /** IFE FRM TC CFG - INDEX_SC
     *  IFE Frame Count Register. There are 8 sets of these values in 
     *  order to support 8 simultaneous radio standard variants.
     *  Radio Framing Counter. Index Counter Starting Location. Starting 
     *  location of the Sample Terminal Count LUT loaded into the Index 
     *  Counter when it first starts and each time it wraps. 
     */
    uint8_t   index_sc[8];

    /** IFE FRM TC CFG - INDEX_TC
     *  IFE Frame Count Register. There are 8 sets of these values in 
     *  order to support 8 simultaneous radio standard variants.
     *  Radio Framing Counter. Index Counter Terminal Count. Index counter 
     *  terminal count which is the last value of the Index Counter before 
     *  it wraps. For simple use case, program same as frm_sym_tc.
     */
    uint8_t   index_tc[8];

    /** IFE RAD STD CFG - TDD_LUT_EN
     *  IFE Radio Standard Configuration Register.
     *  Enable TDD
     *     ENABLED (1) = TDD enabled for this radio standard
     *     DISABLED (0) = TDD disabled for this radio standard
     */
    uint8_t   tdd_lut_en[8];

    /** IFE TDD EN CFG0/7 - TDD_EN
     *  Per symbol enables for TDD operation for Radio Standard 0 to 7. 
     *  5 MMRs of 32 bits each can accommodate 160 symbols. Enable 
     *  for symbol 0 in bit 0 of first MMR and enable for symbol 
     *  159 in bit 31 of last MMR.
     *  enables/disables DMA of whole symbols (PktDMA packets). 
     *  Program as 0xffff for most applications.
     *     SYM_ON (1) = symbol dma enabled
     *     SYM_OFF (0) = symbol dma disabled
     */
    uint32_t   tdd_en_cfg[8][5];

} Iqn2Fl_Aid2IqIfeRadioStdGrpSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Aid2Setup. This structure is
 * used for configuring AID2 IDC Channel Config Group and ICTL PKT IF registers */
typedef struct 
{
    /** AID2 IQ IDC CH CFG - DAT_SWAP
     *  Per-channel configuration registers.
     *  Byte swapping control.
     *     NONE (0) = no swap
     *     BYTE (1) = byte swap
     *     HALF (2) = half word swap. 16-bit swap
     *     WORD (3) = word swap. 32-bits
     */
    uint8_t  dat_swap;

    /** AID2 IQ IDC CH CFG - IQ_ORDER
     *  Per-channel configuration registers.
     *  IQ swapping control.
     *     NONE1 (0) = no swap
     *     NONE2 (1) = no swap
     *     BYTE (2) = byte swap
     *     HALF (3) = 16-bit swap
     */
    uint8_t  iq_order;

    /** AID2 IQ IDC CH CFG - PKT_TYPE
     *  Per-channel configuration registers.
     *  Programmable packet type that is inserted into pkt_type field 
     *  of PKTDMA Info Word 0.
     */
    uint8_t  pkt_type;

    /** AID2 IQ IDC CH CFG - CHAN_FRC_OFF
     *  Per-channel configuration registers.
     *  Forces off all channel without waiting for an end of symbol or 
     *  time slot. If channel has an open packet it is automatically 
     *  closed by creating an EOP
     *     FRC_OFF (0) = Force off channel and close an existing open packet
     *     NOP (1) = No effect
     */
    uint8_t  chan_frc_off;

} Iqn2Fl_Aid2IqIngChanCfgGrpSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Aid2Setup. This structure is
 * used for configuring AID2 UAT GEN CTL registers */
typedef struct 
{
    /** AID2 UAT BCN TC CFG - VAL
     *  UAT BCN terminal count Register.
     *  UAT BCN terminal count. BCN counts from zero to this limit and 
     *  wraps to zero. Program as 2,457,599 for sys_clk=245.76MHz and 
     *  3,071,999 for sys_clk=307.2MHz
     */
    uint32_t  bcn_tc_cfg_val; 

    /** AID2 UAT BCN OFFSET CFG - VAL
     *  UAT BCN offset Register.
     *  Offset correction to the raw uAT BCN counter. Used to correct 
     *  the alignment of the local uAT BCN to the master AT2 BCN. BCN 
     *  is initially randomly started. SW uses uat_sync_bcn_capture_sts 
     *  rd_val to calculate offset correction factor. This correction 
     *  factor will be Frame size - captured value.
     */
    uint32_t  bcn_offset_cfg_val; 

} Iqn2Fl_Aid2IqUatGenCtlSetup;


/**
 * @brief This structure is used for configuring UAT RADT EVT registers 
 * on an event by event basis for AID2 and DIO2.
 */
typedef struct 
{
    /** Event number
     *  Value range from 0 t0 21. The 0 to 7 are for si 
     *  egress, 8 to 15 for si ingress, 16 to 18 for dio egress, 
     *  19 to 21 for dio ingress.
     */
    uint8_t  event_num;

    /** AID2 UAT RADT EVT - EVT CMP CFG 
     *  UAT RADT event compare Register per RADT. 
     *  UAT RADT event compare per RADT. When compare value equals RADT 
     *  count, frame rate event is generated. Also periodic event (i.e. 4SAMP)
     *  is started. 
     */
    uint32_t  cmp_cfg_val;

    /** AID2 UAT RADT EVT - EVT CLK CNT TC CFG 
     *  UAT RADT event clock counter terminal count Register per RADT.
     *  UAT RADT event clock counter terminal count controls spacing of 
     *  the periodic strobe (i.e. 4SAMP). Once the uat_evt_radt_cmp_cfg 
     *  equals the RADT, the period strobe will fire and re-fire every 
     *  time a clock counter reaches this terminal count. 
     */
    uint16_t  clk_cnt_tc;

} Iqn2Fl_UatRadtEvtSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * configuring the parameters of AID2 registers */
typedef struct 
{
    /** pointer to AID2 SI IQ EFE CONFIG GROUP setup */
    Iqn2Fl_Aid2IqEfeCfgGrpSetup   aid2_iq_efe_cfg_grp;

    /** pointer to AID2 SI IQ EFE RADIO STANDARD GROUP setup */
    Iqn2Fl_Aid2IqEfeRadioStdGrpSetup   aid2_iq_efe_radio_std_grp;

    /** AID2 IQ EFE CHAN AXC OFFSET 
     *  Sets the AXC offset for each channel.
     *  AxC-by-AxC delay control. Allows different timing alignments for 
     *  each AxC. DIO & AID applicactions this is a 4 sample offset relatvie 
     *  to the group or TDM of AxC. AIL CPRI, this is a sample offset 
     *  relative to the CPRI AxC Group. AIL OBSAI, this is a Radio Timer 
     *  compare value. Other than OBSAI, for most customer applications, 
     *  these fields are programmed as zero.
     */
    uint32_t  efe_axc_offset[32];

    /** AID2 IQ EFE FRM SAMP TC MMR RAM - EFE FRM SAMP TC CFG 
     *  EFE AxC Radio Framing Sample Terminal Count Configuration Register.
     *  Radio Framing Counter. Number of samples (4 Bytes) per radio symbol 
     *  programmed as a terminal count.
     */
    uint32_t  efe_samp_tc[256];

    /** AID2 IQ EFE CHAN TDM LUT - EFE CHAN TDM LUT CFG 
     *  Sets the TDM LUT.
     *  TDM Channel Index.
     */
    uint8_t  efe_chan_index_cfg[256];

    /** AID2 IQ EFE CHAN TDM LUT - EFE CHAN TDM LUT CFG 
     *  Sets the TDM LUT.
     *  TDM Channel Index LUT Enable.
     */
    uint8_t  efe_chan_index_en_cfg[256];

    /** AID2 IQ EFE RADIO STANDARD SCHEDULER GROUP - EFE RAD STD SCH CFG 
     *  EFE Radio Standard Scheduler Configuration Register.
     *  EFE Scheduler TDM Starting address in LUT.
     */
    uint8_t  efe_tdm_start[8];

    /** AID2 IQ EFE RADIO STANDARD SCHEDULER GROUP - EFE RAD STD SCH CFG 
     *  EFE Radio Standard Scheduler Configuration Register.
     *  EFE Scheduler TDM Length.
     */
    uint8_t  efe_tdm_len[8];

    /** AID2 IQ EFE RADIO STANDARD SCHEDULER GROUP - EFE RAD STD SCH CFG 
     *  EFE Radio Standard Scheduler Configuration Register.
     *  EFE Scheculer Enable TDM
     *     ENABLED (1) = TDM enabled for this radio standard
     *     DISABLED (0) = TDM disabled for this radio standard
     */
    uint8_t  efe_tdm_en[8];

    /** pointer to AID2 IQ IFE CHANNEL CONFIGURATION GROUP setup */
    Iqn2Fl_Aid2IqIfeChanCfgGrpSetup   aid2_iq_ife_chan_cfg_grp[32];

    /** pointer to AID2 IQ IFE RADIO STANDARD GROUP setup */
    Iqn2Fl_Aid2IqIfeRadioStdGrpSetup   aid2_iq_ife_radio_std_grp;

    /** AID2 IQ IDC CONFIGURATION GROUP - IDC CFG 
     *  IDC Configuration Register.
     *  Controls how the IDC handles packet errors detected by IFE
     *     DROP (0) = Drop and Mark Packets With Errors
     *     MARK (1) = Only Mark Packets With Errors
     */
    uint8_t  idc_fail_mark_only;

    /** AID2 IQ IDC CONFIGURATION GROUP - IDC CFG 
     *  IDC Configuration Register.
     *  Forces off all Ingress channels without waiting for an end of 
     *  symbol or time slot. All open packets are automatically closed 
     *  by creating an EOP for each open packet 
     *     FRC_OFF (1) = Force all channels off and close all open packets
     *     NOP (0) = No effect
     */
    uint8_t  idc_frc_off_all;

    /** pointer to AID2 IQ IDC CHANNEL CONFIG GROUP setup */
    Iqn2Fl_Aid2IqIngChanCfgGrpSetup   aid2_iq_idc_chan_cfg_grp[32];

    /** AID2 IFE FRM SAMP TC MMR RAM - IFE FRM SAMP TC CFG 
     *  IFE AxC Framing Sample Terminal Count Configuration Register.
     *  Radio Framing Counter. Number of samples (4 Bytes) per radio symbol 
     *  programmed as a terminal count.
     */
    uint32_t  ife_samp_tc[256];

    /** AID2 ECTL PKT IF - ECTL CHAN CFG 
     *  ECTL Channel Configuration Enable Register.
     *  Enable channel
     *     ENABLED (1) = Enable channel
     *     DISABLED (0) = Disable channel
     */
    uint8_t  ectl_chan_en[16];

    /** AID2 ECTL PKT IF - ECTL DB THOLD CFG 
     *  ECTL Database Threshold Register.
     *  Data Buffer Threshold before indicating data available. Per channel.
     */
    uint8_t  ectl_channel[16];

    /** pointer to AID2 ICTL IDC IF setup */
    Iqn2Fl_Aid2IqIngChanCfgGrpSetup   aid2_ictl_idc_if_ch_cfg[16];

    /** AID2 ICTL IDC IF - ICTL CFG - FAIL MARK ONLY 
     *  ICTL Configuration Register
     *  Controls how the ICTL handles packet errors detected by ICTL
     *     DROP (00) = Drop and Mark Packets With Errors
     *     MARK (11) = Only Mark Packets With Errors
     */
    uint8_t  aid2_ictl_idc_if_ictl_cfg_fail_mark_only;

    /** AID2 ICTL IDC IF - ICTL CFG - FORCE OFF ALL
     *  ICTL Configuration Register
     *  Forces off all Ingress channels without waiting for an EOP. All 
     *  open packets are automatically closed by creating an EOP for 
     *  each open packet
     *     FRC_OFF (11) = Force all channels off and close all open packets
     *     NOP (00) = No effect
     */
    uint8_t  aid2_ictl_idc_if_ictl_cfg_force_off_all;

    /** AID2 ICTL PKT IF - ICTL CHAN CFG 
     *  ICTL Channel Configuration Enable Register.
     *  Enable channel
     *     ENABLED (1) = Enable channel
     *     DISABLED (0) = Disable channel
     */
    uint8_t  ictl_pkt_if_chan_en[16];

    /** pointer to AID2 UAT GEN CTL setup */
    Iqn2Fl_Aid2IqUatGenCtlSetup   aid2_iq_uat_gen_ctl;

    /** AID2 UAT EGR RADT - TC CFG 
     *  UAT RADT terminal count Register.
     *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with 
     *  sys_clk=245.76MHz)
     */
    uint32_t  uat_egr_radt_tc_cfg_val[8];

    /** AID2 UAT EGR RADT - OFFSET CFG 
     *  UAT RADT offset Register.
     *  UAT RADT offset. Value which is added to the raw RADT as a timing 
     *  correction. RadT is initially randomly started, SW uses 
     *  radt_capture value to calculate offset correction factor. This 
     *  correction factor will be Frame size - captured value.
     */
    uint32_t  uat_egr_radt_offset_cfg_val[8];

    /** AID2 UAT ING RADT - TC CFG 
     *  UAT RADT terminal count Register.
     *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with 
     *  sys_clk=245.76MHz)
     */
    uint32_t  uat_ing_radt_tc_cfg_val[8];

    /** AID2 UAT ING RADT - OFFSET CFG 
     *  UAT RADT offset Register.
     *  UAT RADT offset. Value which is added to the raw RADT as a timing 
     *  correction. RadT is initially randomly started, SW uses 
     *  radt_capture value to calculate offset correction factor. This 
     *  correction factor will be Frame size - captured value.
     */
    uint32_t  uat_ing_radt_offset_cfg_val[8];

    /** AID2 UAT RADT EVT - EVT CMP CFG 
     *  UAT RADT event compare Register per RADT. The 0 to 7 are for si 
     *  egress, 8 to 15 for si ingress, 16 to 18 for dio egress, 19 to 21 
     *  for dio ingress.
     *  UAT RADT event compare per RADT. When compare value equals RADT 
     *  count, frame rate event is generated. Also periodic event (i.e. 4SAMP)
     *  is started. The 0 to 7 are for si egress, 8 to 15 for si ingress, 
     *  16 to 18 for dio egress, 19 to 21 for dio ingress.
     */
    uint32_t  uat_evt_radt_cmp_cfg_val[22];

    /** AID2 UAT RADT EVT - EVT CLK CNT TC CFG 
     *  UAT RADT event clock counter terminal count Register per RADT.
     *  UAT RADT event clock counter terminal count controls spacing of 
     *  the periodic strobe (i.e. 4SAMP). Once the uat_evt_radt_cmp_cfg 
     *  equals the RADT, the period strobe will fire and re-fire every 
     *  time a clock counter reaches this terminal count. The The 0 to 
     *  7 are for si egress, 8 to 15 for si ingress, 16 to 18 for dio 
     *  egress, 19 to 21 for dio ingress.
     */
    uint16_t  uat_evt_clk_cnt_tc_cfg_val[22];

    /** AID2 IQ EDC REGISTER GROUP - EDC CFG 
     *  EDC Configuration Register.
     *  Controls how the EDC handles packet errors detected by efe
     *     DROP (0) = Drop the rest of the packet on errors
     *     NO_DROP (1) = Do not drop packet on errors
     */
    uint8_t  edc_cfg_psi_err_chk_disable;

    /** AID2 IQ EDC REGISTER GROUP - EDC CH CFG 
     *  Per-channel configuration registers.
     *  Byte swapping control.
     *     NONE (0) = no swap
     *     BYTE (1) = byte swap
     *     HALF (2) = half word swap. 16-bit swap
     *     WORD (3) = word swap. 32-bits
     */
    uint8_t  edc_ch_cfg_dat_swap[32];

    /** AID2 IQ EDC REGISTER GROUP - EDC CH CFG 
     *  Per-channel configuration registers.
     *  IQ swapping control.
     *     NONE1 (0) = no swap
     *     NONE2 (1) = no swap
     *     BYTE (2) = byte swap
     *     HALF (3) = 16-bit swap
     */
    uint8_t  edc_ch_cfg_iq_order[32];

    /** AID2 IQ INGRESS VBUS MMR GROUP
     *  IDC Rate Control Configuration register. Programmable rate control 
     *  for Rate Controller.
     *  Rate Controller will allow the IDC to create RATE+1 active requests 
     *  on the PSI bus within a 32 clock cycle window. As an example, a value 
     *  of 7 will allow the IDC to create 8 active requests within a 32-clock 
     *  cycle window which uses 25% of the PSI bus.
     *  Default value is 15. Setting this value to a non-default value is possible if 0 < idc_rate_ctl_cfg_rate <= 15
     *  If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AID2_IQ_IDC_RATE_CTL_REG CMD word
     */
    uint8_t  idc_rate_ctl_cfg_rate;

    /** AID2 ECTL REGISTER GROUP - ECTL RATE CTL CFG 
     *  ECTL Rate Control Configuration register. Programmable rate control 
     *  for Rate Controller.
     *  Rate Controller will allow the ECTL to create RATE+1 active requests 
     *  on the PSI bus within a 16 clock cycle window. As an example, a 
     *  value of 7 will allow the ICTL to create 8 active requests within a 
     *  16-clock cycle window which uses 50% of the PSI bus.
     *  Default value is 15. Setting this value to a non-default value is possible if 0 < ectl_rate_ctl_cfg <= 15
     *  If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AID2_ECTL_RATE_CTL_CFG_REG CMD word
     */
    uint8_t  ectl_rate_ctl_cfg;

    /** AID2 ECTL REGISTER GROUP - ECTL CH CFG 
     *  Per-channel configuration registers.
     *  Byte swapping control.
     *     NONE (0) = no swap
     *     BYTE (1) = byte swap
     *     HALF (2) = half word swap. 16-bit swap
     *     WORD (3) = word swap. 32-bits
     */
    uint8_t  ectl_ch_cfg_dat_swap[16];

    /** AID2 ECTL REGISTER GROUP - ECTL CH CFG 
     *  Per-channel configuration registers.
     *  IQ swapping control.
     *     NONE1 (0) = no swap
     *     NONE2 (1) = no swap
     *     BYTE (2) = byte swap
     *     HALF (3) = 16-bit swap
     */
    uint8_t  ectl_ch_cfg_iq_order[16];

    /** AID2 CTL INGRESS VBUS MMR GROUP - ICTL RATE CTL CFG 
     *  ICTL Rate Control Configuration register. Programmable 
     *  rate control for Rate Controller.
     *  Rate Controller will allow the IDC to create RATE+1 active requests 
     *  on the PSI bus within a 16 clock cycle window. As an example, a value 
     *  of 7 will allow the ICTL to create 8 active requests within a 16-clock 
     *  cycle window which uses 50% of the PSI bus.
     *  Default value is 15. Setting this value to a non-default value is possible if 0 < ictl_rate_ctl_cfg_rate <= 15
     *  If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AID2_ICTL_RATE_CTL_CFG_REG CMD word
     */
    uint8_t  ictl_rate_ctl_cfg_rate;

} Iqn2Fl_Aid2Setup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Iqs2Setup. This structure is
 * used for configuring IQS2 Ingress Config registers */
typedef struct 
{
    /** IQS ING PKTDMA CFG - PB_SEL
     *  Ingress PKTDMA pushback control.
     *  Sets level where PKTDMA FIFO pushback occurs on ingress ports that 
     *  can be pushed back on. 
     *     pb_at_half_full (0) = Pushes back when the PKTDMA FIFO is over 1/2 
     *                           full. At most 1/2 of the FIFO is available 
     *                           for traffic that can be pushed back on leaving 
     *                           atleast 1/2 of the FIFO for data that cannot 
     *                           be pushed back on.
     *     pb_at_3qtr_full (1) = Pushes back when the PKTDMA FIFO is over 3/4 
     *                           full. At most 3/4 of the FIFO is available 
     *                           for traffic that can be pushed back on 
     *                           leaving atleast 1/4 of the FIFO for data 
     *                           that cannot be pushed back on. 
     */
    uint8_t   pktdma_cfg_pb_sel;

    /** IQS ING AID2 AXC CFG - PRI
     *  Ingress AID2 AXC priority control and channel error status.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   aid2_axc_cfg_pri;

    /** IQS ING AID2 AXC CFG - ALLOW_PUSHBACK
     *  Ingress AID2 AXC priority control and channel error status.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   aid2_axc_cfg_allow_pushback;

    /** IQS ING AID2 CTL CFG - PRI
     *  Ingress AID2 CTL priority and pushback control and channel 
     *  error status.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   aid2_ctl_cfg_pri;

    /** IQS ING AID2 CTL CFG - ALLOW_PUSHBACK
     *  Ingress AID2 CTL priority and pushback control and channel 
     *  error status.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   aid2_ctl_cfg_allow_pushback;

    /** IQS ING AIL0 AXC CFG - PRI
     *  Ingress AIL0 and pushback AXC priority control and channel error 
     *  status.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   ail0_axc_cfg_pri;

    /** IQS ING AIL0 AXC CFG - ALLOW_PUSHBACK
     *  Ingress AIL0 and pushback AXC priority control and channel error 
     *  status.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   ail0_axc_cfg_allow_pushback;

    /** IQS ING AIL0 CTL CFG - PRI
     *  Ingress AIL0 CTL priority and pushback control and channel 
     *  error status.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   ail0_ctl_cfg_pri;

    /** IQS ING AIL0 CTL CFG - ALLOW_PUSHBACK
     *  Ingress AIL0 CTL priority and pushback control and channel 
     *  error status.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   ail0_ctl_cfg_allow_pushback;

    /** IQS ING AIL1 AXC CFG - PRI
     *  Ingress AIL1 AXC priority and pushback control and channel error 
     *  status.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   ail1_axc_cfg_pri;

    /** IQS ING AIL1 AXC CFG - ALLOW_PUSHBACK
     *  Ingress AIL1 AXC priority and pushback control and channel error 
     *  status.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   ail1_axc_cfg_allow_pushback;

    /** IQS ING AIL1 CTL CFG - PRI
     *  Ingress AIL1 CTL priority and pushback control and channel 
     *  error status.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   ail1_ctl_cfg_pri;

    /** IQS ING AIL1 CTL CFG - ALLOW_PUSHBACK
     *  Ingress AIL1 CTL priority and pushback control and channel 
     *  error status.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   ail1_ctl_cfg_allow_pushback;

    /** IQS ING AIL2 AXC CFG - PRI
     *  Ingress AIL2 AXC priority and pushback control and channel error 
     *  status.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   ail2_axc_cfg_pri;

    /** IQS ING AIL2 AXC CFG - ALLOW_PUSHBACK
     *  Ingress AIL2 AXC priority and pushback control and channel error 
     *  status. NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   ail2_axc_cfg_allow_pushback;

    /** IQS ING AIL2 CTL CFG - PRI
     *  Ingress AIL2 CTL priority and pushback control and channel 
     *  error status. NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   ail2_ctl_cfg_pri;

    /** IQS ING AIL2 CTL CFG - ALLOW_PUSHBACK
     *  Ingress AIL2 CTL priority and pushback control and channel 
     *  error status.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   ail2_ctl_cfg_allow_pushback;

    /** IQS ING AIL3 AXC CFG - PRI
     *  Ingress AIL3 AXC priority and pushback control and channel error 
     *  status. NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   ail3_axc_cfg_pri;

    /** IQS ING AIL3 AXC CFG - ALLOW_PUSHBACK
     *  Ingress AIL3 AXC priority and pushback control and channel error 
     *  status.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   ail3_axc_cfg_allow_pushback;

    /** IQS ING AIL3 CTL CFG - PRI
     *  Ingress AIL3 CTL priority and pushback control and channel 
     *  error status. NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     *  Sets ingress port arbitration priority. A 0 is highest priority.
     */
    uint8_t   ail3_ctl_cfg_pri;

    /** IQS ING AIL3 CTL CFG - ALLOW_PUSHBACK
     *  Ingress AIL3 CTL priority and pushback control and channel 
     *  error status. NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     *  When set allows the IQS to push back on the ingress PSI bus of 
     *  this port. This allows reserving atleast 1/2 or 1/4 of the PKTDMA 
     *  FIFO for the non-pushback ports. 
     *     no_pushback (0) = Do not enable pushback. Normally used for IQ 
     *                       data ports going to the PKTDMA and all ports 
     *                       going to the DIO. 
     *     en_pushback (1) = Enable pushback. Normally used only for CTL 
     *                       data ports going to the PKTDMA.
     */
    uint8_t   ail3_ctl_cfg_allow_pushback;

} Iqn2Fl_IqsIngressCfgSetup;


/**
 * @brief This is a sub-structure for @a Iqn2Fl_IqsIngressChanCfgSetup. This
 * structure is used for configuring IQS2 Ingress Channel LUT Config */
typedef struct 
{
    /** Channel to use in the destination port.
     */
    uint8_t   chan;

    /** Selects destination port. 
     *     PKTDMA (0) = Selects PKTDMA as the destination
     *     DIO2 (1) = Selects DIO2 as the destination
     *     RESERVED_2 (2) = Selects a reserved destination, do not use.
     *     RESERVED_3 (3) = Selects a reserved destination, do not use.
     */
    Iqn2Fl_IngressChanDest   dest;

} Iqn2Fl_IqsIngressChanLutCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Iqs2Setup. This structure is
 * used for configuring IQS2 Ingress Channel Config registers */
typedef struct 
{
    /** IQS ING DIO2 PSI CFG - PRI
     *  Enables and sets the priority of the ingress DIO2 PSI channel. 
     *  Sets ingress DIO2 PSI channel priority. Typically higher speed 
     *  standards require higher priorities and control ports can use 
     *  lower priorities. A 0 is highest priority.
     */
    uint8_t   dio2_psi_cfg_pri[16];

    /** IQS ING AID2 AXC LUT CFG
     *  Sets ingress AID2 AxC LUT for destination port and channel.
     */
    Iqn2Fl_IqsIngressChanLutCfg   aid2_axc_lut_cfg[32];

    /** IQS ING AID2 CTL LUT CFG
     *  Sets ingress AID2 CTL LUT for destination port and channel.
     */
    Iqn2Fl_IqsIngressChanLutCfg   aid2_ctl_lut_cfg[16];

    /** IQS ING AIL0 AXC LUT CFG
     *  Sets ingress AIL0 AxC LUT for destination port and channel.
     */
    Iqn2Fl_IqsIngressChanLutCfg   ail0_axc_lut_cfg[64];

    /** IQS ING AIL0 CTL LUT CFG
     *  Sets ingress AIL0 CTL LUT for destination port and channel.
     */
    Iqn2Fl_IqsIngressChanLutCfg   ail0_ctl_lut_cfg[4];

    /** IQS ING AIL1 AXC LUT CFG
     *  Sets ingress AIL1 AxC LUT for destination port and channel.
     */
    Iqn2Fl_IqsIngressChanLutCfg   ail1_axc_lut_cfg[64];

    /** IQS ING AIL1 CTL LUT CFG
     *  Sets ingress AIL1 CTL LUT for destination port and channel.
     */
    Iqn2Fl_IqsIngressChanLutCfg   ail1_ctl_lut_cfg[4];

    /** IQS ING AIL2 AXC LUT CFG
     *  Sets ingress AIL2 AxC LUT for destination port and channel.
     *  NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     */
    Iqn2Fl_IqsIngressChanLutCfg   ail2_axc_lut_cfg[64];

    /** IQS ING AIL2 CTL LUT CFG
     *  Sets ingress AIL2 CTL LUT for destination port and channel.
     *  NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     */
    Iqn2Fl_IqsIngressChanLutCfg   ail2_ctl_lut_cfg[4];

    /** IQS ING AIL3 AXC LUT CFG
     *  Sets ingress AIL3 AxC LUT for destination port and channel.
     *  NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     */
    Iqn2Fl_IqsIngressChanLutCfg   ail3_axc_lut_cfg[64];

    /** IQS ING AIL3 CTL LUT CFG
     *  Sets ingress AIL3 CTL LUT for destination port and channel.
     *  NOTE: AIL 2 and 3 not currently supported. These are to be 
     *  treated as reserved locations.
     */
    Iqn2Fl_IqsIngressChanLutCfg   ail3_ctl_lut_cfg[4];

} Iqn2Fl_IqsIngressChanCfgSetup;


/**
 * @brief This is a sub-structure for @a Iqn2Fl_IqsEgressChanCfgSetup. This
 * structure is used for configuring IQS2 Egress Channel Packet DMA */
typedef struct 
{
    /** Channel to use in the destination port.
     */
    uint8_t   chan;

    /** Selects destination port. 
     *     AID2_AXC (0) = Selects AID2 AXC as the destination
     *     AID2_CTl (1) = Selects AID2 CTL as the destination
     *     AIL0_AXC (2) = Selects AIL0 AXC as the destination
     *     AIL0_CTL (3) = Selects AIL0 CTL as the destination
     *     AIL1_AXC (4) = Selects AIL1 AXC as the destination
     *     AIL1_CTL (5) = Selects AIL1_CTL as the destination
     *     AIL2_AXC (6) = Selects AIL2 AXC as the destination
     *     AIL2_CTL (7) = Selects AIL2 CTL as the destination
     *     AIL3_AXC (8) = Selects AIL3 AXC as the destination
     *     AIL3_CTL (9) = Selects AIL3_CTL as the destination
     */
    Iqn2Fl_EgressChanDest   dest;

    /** Arbitration priority to the output FIFO. A 0 is highest priority.
     */
    uint8_t   arb_pri;

    /** PSI priority to destination module. A 0 is highest priority.
     */
    uint8_t   psi_pri;

} Iqn2Fl_IqsEgrPktdmaCfgSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Iqs2Setup. This structure is
 * used for configuring IQS2 Egress Channel Config registers */
typedef struct 
{
    /** IQS EGR PKTDMA CFG
     *  Sets egress PKTDMA destination port and channel. Also sets internal 
     *  arbitration and port PSI priority.
     */
    Iqn2Fl_IqsEgrPktdmaCfgSetup   egr_pktdma_cfg[48];

    /** IQS EGR DIO2 CFG
     *  Sets egress DIO2 destination port and channel. Also sets internal 
     *  arbitration and port PSI priority. A 0 is highest priority.
     */
    Iqn2Fl_IqsEgrPktdmaCfgSetup   egr_dio2_cfg[16];

} Iqn2Fl_IqsEgressChanCfgSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * configuring the parameters of IQS2 level registers */
typedef struct 
{
    /** pointer to IQS INGRESS CONFIG setup */
    Iqn2Fl_IqsIngressCfgSetup   iqs2_ingress_cfg;

    /** pointer to IQS INGRESS CHAN CONFIG setup */
    Iqn2Fl_IqsIngressChanCfgSetup   iqs2_ingress_chan_cfg;

    /** pointer to IQS EGRESS CHAN CONFIG setup */
    Iqn2Fl_IqsEgressChanCfgSetup   iqs2_egress_chan_cfg;

} Iqn2Fl_Iqs2Setup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 SI IQ EFE Config Group registers */
typedef struct 
{
    /** EFE CHAN CFG - CHAN_EN
     *  Enable channel
     *     ENABLED (1) = Enable channel
     *     DISABLED (0) = Disable channel
     */
    uint8_t   chan_en[16];

    /** EFE CHAN CFG - CHAN_TDD_FRC_OFF
     *  Alternate TDD mode for controlling TDD also used for GSM Base 
     *  Band Hopping. APP SW controls updates this bit each symbol of 
     *  time to control whether the next symbol will be TDD OFF. TDD 
     *  OFF channels generate no Ingress DMA traffic and expect no 
     *  Egress DMA traffic. Zeros are sent over the PHY. In BBHop mode, 
     *  the same applies and in OBSAI, empty_msg is sent over the PHY 
     *  instead of zero'ed traffic 
     *     FRC_SYM_OFF (1) = Force symbols off
     *     NO_FRC_OFF_SYM (0) = No forcing off of symbols
     */
    uint8_t   chan_tdd_frc_off[16];

    /** EFE CHAN CFG - CHAN_RADIO_SEL
     *  Assigns each channel to one of eight radio standard groups. i.e. 
     *  radio standard 0 may be LTE 20MHz
     *     RS0 (0) = Radio Standard 0
     *     RS1 (1) = Radio Standard 1
     *     RS2 (2) = Radio Standard 2
     *     RS3 (3) = Radio Standard 3
     *     RS4 (4) = Radio Standard 4
     *     RS5 (5) = Radio Standard 5
     *     RS6 (6) = Radio Standard 6
     *     RS7 (7) = Radio Standard 7
     */
    Iqn2Fl_ChanRadioSel   chan_radio_sel[16];

    /** EFE CFG - LOOPBACK_EN
     *  (TI use Only) 0x1: Ingress data from ICC is looped back to Egress 
     *  data to ICC. DMA traffic is unused. (i.e. for purpose of early DFE 
     *  only testing)
     */
    uint8_t   loopback_en;

} Iqn2Fl_Dio2SiIqEfeCfgGrpSetup;


/**
 * @brief This is a sub-structure used by @a Iqn2Fl_Dio2Setup.
 * This structure is used for configuring the Radio Standard Frame Count 
 * Register for EFE and IFE.
 */
typedef struct 
{
    /** Symbol Count. Number of symbols per frame programmed as a Terminal Count */
    uint8_t   sym_tc;

    /** Index Counter Starting Location 
     *  Starting location of the Sample Terminal Count LUT loaded into the 
     *  Index Counter when it first starts and each time it wraps. 
     *  Note for EFE: Depending on the radio standard, the index will 
     *                wrap once per radio frame such as WCDMA or muliple 
     *                times per frame as in LTE. Index is the address 
     *                for XXX_IQ_EFE_FRM_SAMP_TC.
     */
    uint8_t   index_sc;

    /** Index Counter Terminal Count 
     *  Index counter terminal count which is the last value of the Index 
     *  Counter before it wraps. For simple use case, program same as 
     *  frm_sym_tc.
     */
    uint8_t   index_tc;

} Iqn2Fl_Dio2FrmTcCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 SI IQ EFE Radio Standard Group registers */
typedef struct 
{
    /** EFE FRM TC CFG
     *  EFE Frame Count Register. There are 8 sets of these values in 
     *  order to support 8 simultaneous radio standard variants.
     */
    Iqn2Fl_Dio2FrmTcCfg   efe_frm_tc_cfg[8];

    /** EFE RAD STD CFG - TDD_FIRST_SYM
     *  EFE Radio Standard Configuration Register.
     *  Selects first symbol to start TDD.
     */
    uint8_t   efe_rad_std_cfg_tdd_first_sym[8];

    /** EFE RAD STD CFG - TDD_LUT_EN
     *  EFE Radio Standard Configuration Register.
     *  Enable TDD
     *     ENABLED (1) = TDD enabled for this radio standard
     *     DISABLED (0) = TDD disabled for this radio standard
     */
    uint8_t   efe_rad_std_cfg_tdd_lut_en[8];

    /** EFE TDD EN CFG0/7 - TDD_EN
     *  Per symbol enables for TDD operation for Radio Standard 0 to 7.  
     *  5 MMRs of 32 bits each can accommodate 160 symbols. Enable for symbol 
     *  0 in bit 0 of first MMR and enable for symbol 159 in bit 31 of last MMR.
     *  Enables/disables DMA of whole symbols (PktDMA packets). Program as 
     *  0xffff for most applications.
     *     SYM_ON (1) = symbol dma enabled
     *     SYM_OFF (0) = symbol dma disabled
     */
    uint32_t   efe_tdd_en_cfg_tdd_en[8][5];

} Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 IQ IFE DMA Channel Config Group registers */
typedef struct 
{
    /** IFE CHAN CFG - CHAN_EN
     *  Enable channel
     *     ENABLED (1) = Enable channel
     *     DISABLED (0) = Disable channel
     */
    uint8_t   chan_en;

    /** IFE CHAN CFG - CHAN_AXC_OFFSET
     *  AxC Offset
     *     NONE (0) = No offset
     *     ONE (1) = One sample offset
     *     TWO (2) = Two sample offset
     *     THREE (3) = Three sample offset
     */
    uint8_t   chan_axc_offset;

    /** IFE CHAN CFG - CHAN_RADIO_SEL
     *  Radio Standard Select
     *     RS0 (0) = Radio Standard 0
     *     RS1 (1) = Radio Standard 1
     *     RS2 (2) = Radio Standard 2
     *     RS3 (3) = Radio Standard 3
     *     RS4 (4) = Radio Standard 4
     *     RS5 (5) = Radio Standard 5
     *     RS6 (6) = Radio Standard 6
     *     RS7 (7) = Radio Standard 7
     */
    Iqn2Fl_ChanRadioSel   chan_radio_sel;

    /** IFE CHAN CFG - CHAN_TDD_FRC_OFF
     *  Forces a channel into the TDD OFF state on the next symbol after it 
     *  is set to a 1 regardless of the TDD configuration of the radio 
     *  standard variant the channel is assigned to.
     *     FRC_SYM_OFF (1) = Force symbols off
     *     NO_FRC_OFF_SYM (0) = No forcing off of symbols
     */
    uint8_t   chan_tdd_frc_off;

} Iqn2Fl_Dio2IqIfeChanCfgGrpSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 SI IQ IFE Radio Standard Group registers */
typedef struct 
{
    /** IFE FRM TC CFG
     *  IFE Frame Count Register. There are 8 sets of these values in 
     *  order to support 8 simultaneous radio standard variants.
     */
    Iqn2Fl_Dio2FrmTcCfg   ife_frm_tc_cfg[8];

    /** IFE RAD STD CFG - TDD_LUT_EN
     *  IFE Radio Standard Configuration Register.
     *  Enable TDD
     *     ENABLED (1) = TDD enabled for this radio standard
     *     DISABLED (0) = TDD disabled for this radio standard
     */
    uint8_t   ife_rad_std_cfg_tdd_lut_en[8];

    /** IFE TDD EN CFG0/7 - TDD_EN
     *  Per symbol enables for TDD operation for Radio Standard 0 to 7.  
     *  5 MMRs of 32 bits each can accommodate 160 symbols. Enable for symbol 
     *  0 in bit 0 of first MMR and enable for symbol 159 in bit 31 of last MMR.
     *  Enables/disables DMA of whole symbols (PktDMA packets). Program as 
     *  0xffff for most applications.
     *     SYM_ON (1) = symbol dma enabled
     *     SYM_OFF (0) = symbol dma disabled
     */
    uint32_t   ife_tdd_en_cfg_tdd_en[8][5];

} Iqn2Fl_Dio2IqIfeRadioStdGrpSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring IDC (Ingress DMA Controller) CHANNEL CONFIG GROUP 
 * registers of DIO2 instance */
typedef struct 
{
    /** Byte swapping control. 0:No swap, 1:Byte swap, 
     *  2:Half word swap (16-bits), 3:Word swap (32 bits)
     */
    uint8_t   dat_swap;

    /** IQ swapping control. 0:No swap, 1:No swap, 
     *  2:Byte swap, 3:16-bits swap
     */
    uint8_t   iq_order;

    /** Programmable packet type that is inserted into pkt_type 
     *  field of PKTDMA Info Word 0.
     */
    uint8_t   pkt_type;

    /** Forces off all channel without waiting for an end of symbol 
     *  or time slot. If channel has an open packet it is automatically 
     *  closed by creating an EOP.
     *    FRC_OFF (0) = Force off channel and close an existing open packet
     *    NOP (1) = No effect
     */
    uint8_t   chan_frc_off;

} Iqn2Fl_Dio2IqIdcChCfgGrp;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 DIO Ingress/Egress DMA Configuration Register 0 
 * of DIO2 instance */
typedef struct 
{
    /** Sets the maximum DMA burst length.
     *     1QUAD (0) = 1 quad word transferred per burst
     *     2QUAD (1) = 2 quad words transferred per burst
     *     4QUAD (2) = 4 quad words transferred per burst
     */
	Iqn2Fl_DmaNumQuadWord   dma_brst_ln;

    /** Sets the number of quad words per AxC.
     *     1QUAD (0) = 1 quad word per AxC
     *     2QUAD (1) = 2 quad words per AxC
     *     4QUAD (2) = 4 quad words per AxC
     */
	Iqn2Fl_DmaNumQuadWord   dma_num_qwd;

    /** Ingress RSA data conversion enable per DMA engine or per channel.
     *     NORSA (0) = RSA Data Format Conversion Disabled
     *     RSAEN (1) = RSA Data Format Conversion Enabled
     */
    uint8_t   rsa_cnvrt_en;

    /** Enable DMA engine.
     *     DMACLR (0) = DMA engine disabled i.e. cleared state
     *     DMAEN (1) = DMA engine enabled
     */
    uint8_t   dma_eng_en;

    /** Set the number of data blocks to transfer before wrapping back 
     *  to dma_base_addr. (1 to 8191 = 2 to 8192). Note: (num_blks + 1) 
     *  must be an even number.
     */
    uint16_t   dma_num_blks;

} Iqn2Fl_Dio2CoreDmaCfg0;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 DIO Ingress/Egress DBCNT0/1/2 Base Address Register 
 * and DIO2 DIO Ingress/Egress DBCNT0/2 Control Register of DIO2 instance */
typedef struct 
{
    /** DIO2 I/E TABLE0/1/2 BASE ADDR CFG - DMA VBUS BASE ADDR AXC
     *  A per AxC register that is a field in the DIO Ingress/Egress 
     *  DBCNT 0/2 LUT that enables the user to program the associated 
     *  AxC's destination VBUS base address. This LUT is comprised of 
     *  two parts. Table A is the lower half of the LUT's address and 
     *  Table B which is the upper half of the LUT's address.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Per AxC VBUS Base address Register.
     */
    uint32_t   dma_vbus_base_addr_axc[32];

    /** DIO2 I/E TABLE0/1/2 CTRL CFG - CH ID
     *  A per AxC register that is a field in the DIO Ingress/Egress DBCNT 
     *  0/1/2 LUT that enables the user to program the associated AxC's 
     *  channel ID and channel enable.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Channel ID Register
     */
    uint8_t   ch_id[32];

    /** DIO2 I/E TABLE0/1/2 CTRL CFG - CH EN
     *  A per AxC register that is a field in the DIO Ingress/Egress DBCNT 
     *  0/1/2 LUT that enables the user to program the associated AxC's 
     *  channel ID and channel enable.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Channel Enable Register
     */
    uint8_t   ch_en[32];

} Iqn2Fl_Dio2DbcntxRamMmr;

/**
 * @brief This is used for configuring DIO2 DIO Ingress/Egress DBCNT Base 
 * Address Register and DIO2 DIO Ingress/Egress DBCNT Control Register 
 * of DIO2 instance on per engine basis */
typedef struct 
{
    /** DIO2 I/E TABLE0/1/2 CTRL CFG
     */
    Iqn2Fl_Dio2DbcntxRamMmr   cfg_dbcn_cnt;

    /** DIO2 Engine Index
     *  Valid values: 0 to 2.
     */
    uint8_t   engine_idx;

} Iqn2Fl_Dio2DbcntxRamMmrCfg;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 Data Trace Register of DIO2 instance */
typedef struct 
{
    /** DIO2 DT CFG0 - DT EN
     *  This register allows the user to program the various mode of 
     *  Data Trace operation.
     *  Enables Data Trace DMA
     *     NODT (0) = Data Trace DMA disabled (cleared state)
     *     DTEN (1) = Data Trace DMA enabled
     */
    uint8_t   dt_cfg0_dt_en;

    /** DIO2 DT CFG0 - DT START MODE
     *  This register allows the user to program the various mode of 
     *  Data Trace operation.
     *  Selects Data Trace start operating mode.
     *     CAPRAW (0) = Raw Data (i.e. unframed) is captured starting 
     *                  when Data Trace is Enable
     *     CAPSOF (1) = Data is captured when Start Of Frame is received
     */
    uint8_t   dt_cfg0_dt_start_mode;

    /** DIO2 DT CFG0 - DT STOP MODE
     *  This register allows the user to program the various mode of 
     *  Data Trace operation.
     *  Selects Data Trace stopping mode.
     *     CIRCULAR (0) = Writes data in a circular fashion and overwriting 
     *                    previously written data until Data Trace is disabled
     *     ONESHOT (1) = Data is captured until the end of the external buffer 
     *                   (dio_dt_cfg2.dt_dma_wrap) is reached, then stops 
     *                   capturing data. The EE event dio_ee_dt_done_info may 
     *                   be used to signal to software when Data Trace has 
     *                   finished capturing data.
     */
    uint8_t   dt_cfg0_dt_stop_mode;

    /** DIO2 DT CFG0 - DT ENDIAN SEL
     *  This register allows the user to program the various mode of 
     *  Data Trace operation.
     *  Selects Data Trace Endian mode.
     *     BIG (0) = Data Trace data in Big Endian format (default)
     *     LITTLE (1) = Data Trace data in Little Endian format.
     */
    uint8_t   dt_cfg0_dt_endian_sel;

    /** DIO2 DT CFG1 - DT DMA BASE ADDR
     *  This register allows the user to program the destination VBUS base 
     *  address for Data Trace.
     *  Sets the destination VBUS base address (upper 28 bits of 32 bit 
     *  data bus) for data trace receive data. The base address must be 
     *  programmed BEFORE dt_en is set.
     */
    uint32_t   dt_cfg1_dt_dma_base_addr;

    /** DIO2 DT CFG2 - DT DMA WRAP
     *  This register allows the user to program the number of burst 
     *  transfers before the destination address wraps back to the base 
     *  address for Data Trace.
     *  Sets the number of burst transfers before the destination address 
     *  wraps back to the base address. Each burst transfer is 4 Quad Words.
     */
    uint32_t   dt_cfg2_dt_dma_wrap;

} Iqn2Fl_Dio2Dt;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 Core Ingress Registers of DIO2 instance */
typedef struct 
{
    /** DIO2 CORE INGRESS - I TABLE SEL CFG - BCN TABLE SEL
     *  This register selects which table to use so that channels may be 
     *  added or deleted for Ingress.
     *  Programmed at the same time is the number of AxCs referenced in 
     *  the table.
     *  The user indicates to the DMA engine, which table is to be active 
     *  during the next frame.
     *  When bcn_table_sel =0, Table A is selected and when bcn_table_sel=1, 
     *  Table B is selected.
     *  The non-selected table may be written any time between frame 
     *  boundary strobes from the uAT.
     *  Switching tables, must be done in the middle, between frame 
     *  boundary strobes in order to assure that the new table takes 
     *  effect in the next frame.
     *  Selects which buffer channel number table for the DMA engine to use
     *     TABLE_A (0) = Selects buffer channel number table A (addressed 
     *                   from 0 to N/2-1)
     *     TABLE_B (1) = Selects buffer channel number table B (addressed 
     *                   from N/2 to N-1)
     */
    Iqn2Fl_BcnTableSel   bcn_table_sel[3];

    /** DIO2 CORE INGRESS - I TABLE SEL CFG - DMA NUM AXC
     *  This register selects which table to use so that channels may be 
     *  added or deleted for Ingress.
     *  Programmed at the same time is the number of AxCs referenced in 
     *  the table.
     *  The user indicates to the DMA engine, which table is to be active 
     *  during the next frame.
     *  When bcn_table_sel =0, Table A is selected and when bcn_table_sel=1, 
     *  Table B is selected.
     *  The non-selected table may be written any time between frame 
     *  boundary strobes from the uAT.
     *  Switching tables, must be done in the middle, between frame 
     *  boundary strobes in order to assure that the new table takes 
     *  effect in the next frame.
     *  Sets the number of AxCs (i.e. 0 to 15 i.e. 1 to 16 AxCs).
     */
    uint8_t   dma_num_axc[3];

    /** DIO2 CORE INGRESS - I DMA CFG0
     *  This register alows the user to program the number of quad words 
     *  in a burst, the number of quad words per AxC and the number of 
     *  data blocks to transfer before wrapping back to dma_base_addr 
     *  for Ingress. This register also allows the user to enable RSA 
     *  mode and the DMA engine itself.
     */
    Iqn2Fl_Dio2CoreDmaCfg0   dma_cfg0[3];

    /** DIO2 CORE INGRESS - I DMA CFG1
     *  This register enables the user to set the DMA block address stride 
     *  between block transfers for Ingress.
     *  Sets the DMA block address stride (in multiples of 0x10).
     */
    uint16_t   dma_cfg1_dma_blk_addr_stride[3];

} Iqn2Fl_Dio2CoreIngressSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Dio2Setup. This structure is
 * used for configuring DIO2 Core Egress Registers of DIO2 instance */
typedef struct 
{
    /** DIO2 CORE EGRESS - E TABLE SEL CFG - BCN TABLE SEL
     *  This register selects which table to use so that channels may be 
     *  added or deleted for Egress.
     *  Programmed at the same time is the number of AxCs referenced in 
     *  the table.
     *  The user indicates to the DMA engine, which table is to be active 
     *  during the next frame.
     *  When bcn_table_sel =0, Table A is selected and when bcn_table_sel=1, 
     *  Table B is selected.
     *  The non-selected table may be written any time between frame 
     *  boundary strobes from the uAT.
     *  Switching tables, must be done in the middle, between frame 
     *  boundary strobes in order to assure that the new table takes 
     *  effect in the next frame.
     *  Selects which buffer channel number table for the DMA engine to use
     *     TABLE_A (0) = Selects buffer channel number table A (addressed 
     *                   from 0 to N/2-1)
     *     TABLE_B (1) = Selects buffer channel number table B (addressed 
     *                   from N/2 to N-1)
     */
    Iqn2Fl_BcnTableSel   bcn_table_sel[3];

    /** DIO2 CORE EGRESS - E TABLE SEL CFG - DMA NUM AXC
     *  This register selects which table to use so that channels may be 
     *  added or deleted for Egress.
     *  Programmed at the same time is the number of AxCs referenced in 
     *  the table.
     *  The user indicates to the DMA engine, which table is to be active 
     *  during the next frame.
     *  When bcn_table_sel =0, Table A is selected and when bcn_table_sel=1, 
     *  Table B is selected.
     *  The non-selected table may be written any time between frame 
     *  boundary strobes from the uAT.
     *  Switching tables, must be done in the middle, between frame 
     *  boundary strobes in order to assure that the new table takes 
     *  effect in the next frame.
     *  Sets the number of AxCs (i.e. 0 to 15 i.e. 1 to 16 AxCs).
     */
    uint8_t   dma_num_axc[3];

    /** DIO2 CORE EGRESS - E DMA CFG0
     *  This register alows the user to program the number of quad words 
     *  in a burst, the number of quad words per AxC and the number of 
     *  data blocks to transfer before wrapping back to dma_base_addr 
     *  for Egress. This register also allows the user to enable RSA 
     *  mode and the DMA engine itself.
     */
    Iqn2Fl_Dio2CoreDmaCfg0   dma_cfg0[3];

    /** DIO2 CORE EGRESS - E DMA CFG1
     *  This register enables the user to set the DMA block address stride 
     *  between block transfers for Egress.
     *  Sets the DMA block address stride (in multiples of 0x10).
     */
    uint16_t   dma_cfg1_dma_blk_addr_stride[3];

} Iqn2Fl_Dio2CoreEgressSetup;

/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * reconfiguring the parameters of DIO2 level registers */
typedef struct
{
    /** DIO2 Engine Index
     *  Valid values: 0 to 2.
     */
    uint32_t engine_idx;
    /** Set the number of data blocks to transfer before wrapping back
         *  to dma_base_addr. (1 to 8191 = 2 to 8192). Note: (num_blks + 1)
         *  must be an even number.*/
    uint32_t egress_num_block;
    /** Set the number of data blocks to transfer before wrapping back
         *  to dma_base_addr. (1 to 8191 = 2 to 8192). Note: (num_blks + 1)
         *  must be an even number.*/
    uint32_t ingress_num_block;
    /**  This register enables the user to set the DMA block address stride
    *  between block transfers for Egress.
    *  Sets the DMA block address stride (in multiples of 0x10).
    */
    uint32_t egress_block_addr_stride;
    /**  This register enables the user to set the DMA block address stride
    *  between block transfers for Ingress.
    *  Sets the DMA block address stride (in multiples of 0x10).
    */
    uint32_t ingress_block_addr_stride;
    /** DIO2 I/E TABLE0/1/2 BASE ADDR CFG - DMA VBUS BASE ADDR AXC
     *  A per AxC register that is a field in the DIO Ingress/Egress
     *  DBCNT 0/2 LUT that enables the user to program the associated
     *  AxC's destination VBUS base address. This LUT is comprised of
     *  two parts. Table A is the lower half of the LUT's address and
     *  Table B which is the upper half of the LUT's address is not supported
     *  in this reconfiguration structure.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Per AxC VBUS Base address Register.*/
    uint32_t egress_axc_buffer_start_addr[16];
    /** DIO2 I/E TABLE0/1/2 BASE ADDR CFG - DMA VBUS BASE ADDR AXC
     *  A per AxC register that is a field in the DIO Ingress/Egress
     *  DBCNT 0/2 LUT that enables the user to program the associated
     *  AxC's destination VBUS base address. This LUT is comprised of
     *  two parts. Table A is the lower half of the LUT's address and
     *  Table B which is the upper half of the LUT's address is not supported
     *  in this reconfiguration structure.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Per AxC VBUS Base address Register.*/
    uint32_t ingress_axc_buffer_start_addr[16];

} Iqn2Fl_Dio2ReconfigureEngineSetup;

/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * reconfiguring the parameters of Egress DIO2 level registers */
typedef struct
{
    /** DIO2 Engine Index
     *  Valid values: 0 to 2.
     */
    uint32_t engine_idx;
    /** Set which table to use so that channels may be
     *  added or deleted for Ingress.
     *  Programmed at the same time is the number of AxCs referenced in
     *  the table.
     *  The user indicates to the DMA engine, which table is to be active
     *  during the next frame.
     *  When bcn_table_sel =0, Table A is selected and when bcn_table_sel=1,
     *  Table B is selected.
     *  The non-selected table may be written any time between frame
     *  boundary strobes from the uAT.
     *  Switching tables, must be done in the middle, between frame
     *  boundary strobes in order to assure that the new table takes
     *  effect in the next frame.
     *  Selects which buffer channel number table for the DMA engine to use
     *     TABLE_A (0) = Selects buffer channel number table A (addressed
     *                   from 0 to N/2-1)
     *     TABLE_B (1) = Selects buffer channel number table B (addressed
     *                   from N/2 to N-1)
     */
    Iqn2Fl_BcnTableSel   bcn_table_sel;
    /** Sets the number of AxCs (i.e. 0 to 15 i.e. 1 to 16 AxCs).
     */
    uint8_t   dma_num_axc;
    /** Programs the number of quad words
     *  in a burst, the number of quad words per AxC and the number of
     *  data blocks to transfer before wrapping back to dma_base_addr
     *  for Egress. This strucuture allows the user to enable RSA
     *  mode and the DMA engine itself.
     */
    Iqn2Fl_Dio2CoreDmaCfg0   dma_cfg0;
    /** This register enables the user to set the DMA block address stride
     *  between block transfers for Egress.
     *  Sets the DMA block address stride (in multiples of 0x10).
     */
    uint32_t dma_cfg1_dma_blk_addr_stride;
    /** DIO2 I/E TABLE0/1/2 BASE ADDR CFG - DMA VBUS BASE ADDR AXC
     *  A per AxC register that is a field in the DIO Ingress/Egress
     *  DBCNT 0/2 LUT that enables the user to program the associated
     *  AxC's destination VBUS base address. This LUT is comprised of
     *  two parts. Table A is the lower half of the LUT's address and
     *  Table B which is the upper half of the LUT's address.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Per AxC VBUS Base address Register.*/
    uint32_t axc_buffer_start_addr[32];
    /** DIO2 I/E TABLE0/1/2 CTRL CFG - CH ID
     *  A per AxC register that is a field in the DIO Ingress/Egress DBCNT
     *  0/1/2 LUT that enables the user to program the associated AxC's
     *  channel ID and channel enable.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Channel ID Register
     */
    uint8_t   ch_id[32];

    /** DIO2 I/E TABLE0/1/2 CTRL CFG - CH EN
     *  A per AxC register that is a field in the DIO Ingress/Egress DBCNT
     *  0/1/2 LUT that enables the user to program the associated AxC's
     *  channel ID and channel enable.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Channel Enable Register
     */
    uint8_t   ch_en[32];

} Iqn2Fl_Dio2ReconfigureEgrEngineSetup;

/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * reconfiguring the parameters of Ingress DIO2 level registers */
typedef struct
{
    /** DIO2 Engine Index
     *  Valid values: 0 to 2.
     */
    uint32_t engine_idx;
    /** Set which table to use so that channels may be
     *  added or deleted for Ingress.
     *  Programmed at the same time is the number of AxCs referenced in
     *  the table.
     *  The user indicates to the DMA engine, which table is to be active
     *  during the next frame.
     *  When bcn_table_sel =0, Table A is selected and when bcn_table_sel=1,
     *  Table B is selected.
     *  The non-selected table may be written any time between frame
     *  boundary strobes from the uAT.
     *  Switching tables, must be done in the middle, between frame
     *  boundary strobes in order to assure that the new table takes
     *  effect in the next frame.
     *  Selects which buffer channel number table for the DMA engine to use
     *     TABLE_A (0) = Selects buffer channel number table A (addressed
     *                   from 0 to N/2-1)
     *     TABLE_B (1) = Selects buffer channel number table B (addressed
     *                   from N/2 to N-1)
     */
    Iqn2Fl_BcnTableSel   bcn_table_sel;
    /** Sets the number of AxCs (i.e. 0 to 15 i.e. 1 to 16 AxCs).
     */
    uint8_t   dma_num_axc;
    /** Programs the number of quad words
     *  in a burst, the number of quad words per AxC and the number of
     *  data blocks to transfer before wrapping back to dma_base_addr
     *  for Egress. This strucuture allows the user to enable RSA
     *  mode and the DMA engine itself.
     */
    Iqn2Fl_Dio2CoreDmaCfg0   dma_cfg0;
    /** This register enables the user to set the DMA block address stride
     *  between block transfers for Egress.
     *  Sets the DMA block address stride (in multiples of 0x10).
     */
    uint32_t dma_cfg1_dma_blk_addr_stride;
    /** DIO2 I/E TABLE0/1/2 BASE ADDR CFG - DMA VBUS BASE ADDR AXC
     *  A per AxC register that is a field in the DIO Ingress/Egress
     *  DBCNT 0/2 LUT that enables the user to program the associated
     *  AxC's destination VBUS base address. This LUT is comprised of
     *  two parts. Table A is the lower half of the LUT's address and
     *  Table B which is the upper half of the LUT's address.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Per AxC VBUS Base address Register.*/
    uint32_t axc_buffer_start_addr[32];
    /** DIO2 I/E TABLE0/1/2 CTRL CFG - CH ID
     *  A per AxC register that is a field in the DIO Ingress/Egress DBCNT
     *  0/1/2 LUT that enables the user to program the associated AxC's
     *  channel ID and channel enable.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Channel ID Register
     */
    uint8_t   ch_id[32];

    /** DIO2 I/E TABLE0/1/2 CTRL CFG - CH EN
     *  A per AxC register that is a field in the DIO Ingress/Egress DBCNT
     *  0/1/2 LUT that enables the user to program the associated AxC's
     *  channel ID and channel enable.
     *  DIO Ingress/Egress DBCNT 0/1/2 - Channel Enable Register
     */
    uint8_t   ch_en[32];

} Iqn2Fl_Dio2ReconfigureIngrEngineSetup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * configuring the parameters of DIO2 level registers */
typedef struct 
{
    /** pointer to DIO2 SI IQ EFE CONFIG GROUP setup */
    Iqn2Fl_Dio2SiIqEfeCfgGrpSetup   dio2_efe_cfg_grp;

    /** pointer to DIO2 SI IQ EFE RADIO STANDARD GROUP setup */
    Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup   dio2_efe_radio_std_grp;

    /** DIO2 IQ EFE CHAN AXC OFFSET CFG - AXC_OFFSET
     *  Sets the AXC offset for each channel.
     *  AxC-by-AxC delay control. Allows different timing alignments for 
     *  each AxC. DIO & AID applicactions this is a 4 sample offset relative 
     *  to the group or TDM of AxC. AIL CPRI, this is a sample offset 
     *  relative to the CPRI AxC Group. AIL OBSAI, this is a Radio Timer 
     *  compare value. Other than OBSAI, for most customer applications, 
     *  these fields are programmed as zero.
     */
    uint32_t   efe_chan_axc_offset_cfg[16];

    /** DIO2 IQ EFE FRM SAMP TC MMR RAM - SAMP_TC
     *  EFE AxC Radio Framing Sample Terminal Count Configuration Register.
     *  Radio Framing Counter. Number of samples (4 Bytes) per radio symbol 
     *  programmed as a terminal count.
     */
    uint32_t   efe_frm_samp_tc_cfg[256];

    /** DIO2 IQ EFE CHAN TDM LUT - CHAN_INDEX_CFG
     *  Sets the TDM LUT.
     *  TDM Channel Index.
     */
    uint8_t   efe_chan_tdm_lut_cfg_chan_idx_cfg[256];

    /** DIO2 IQ EFE CHAN TDM LUT - CHAN_INDEX_EN_CFG
     *  Sets the TDM LUT.
     *  TDM Channel Index LUT Enable.
     */
    uint8_t   efe_chan_tdm_lut_cfg_chan_idx_en_cfg[256];

    /** DIO2 IQ EFE RADIO STANDARD SCHEDULER GROUP - TDM_START
     *  EFE Radio Standard Scheduler Configuration Register.
     *  EFE Scheduler TDM Starting address in LUT.
     */
    uint8_t   efe_rad_std_sch_cfg_tdm_start[8];

    /** DIO2 IQ EFE RADIO STANDARD SCHEDULER GROUP - TDM_LEN
     *  EFE Radio Standard Scheduler Configuration Register.
     *  EFE Scheduler TDM Length.
     */
    uint8_t   efe_rad_std_sch_cfg_tdm_len[8];

    /** DIO2 IQ EFE RADIO STANDARD SCHEDULER GROUP - TDM_EN
     *  EFE Radio Standard Scheduler Configuration Register.
     *  EFE Scheculer Enable TDM
     *     ENABLED (1) = TDM enabled for this radio standard
     *     DISABLED (0) = TDM disabled for this radio standard
     */
    uint8_t   efe_rad_std_sch_cfg_tdm_en[8];

    /** pointer to DIO2 IQ IFE CHANNEL CONFIGURATION GROUP setup */
    Iqn2Fl_Dio2IqIfeChanCfgGrpSetup   dio2_ife_chan_cfg_grp[16];

    /** pointer to DIO2 IQ IFE RADIO STANDARD GROUP setup */
    Iqn2Fl_Dio2IqIfeRadioStdGrpSetup   dio2_ife_radio_std_grp;

    /** DIO2 IQ IDC CONFIGURATION GROUP - FAIL_MARK_ONLY
     *  IDC Configuration Register.
     *  Controls how the IDC handles packet errors detected by IFE
     *     DROP (0) = Drop and Mark Packets With Errors
     *     MARK (1) = Only Mark Packets With Errors
     */
    uint8_t   idc_cfg_grp_fail_mark_only;

    /** DIO2 IQ IDC CONFIGURATION GROUP - FRC_OFF_ALL
     *  IDC Configuration Register.
     *  Forces off all Ingress channels without waiting for an end of 
     *  symbol or time slot. All open packets are automatically closed 
     *  by creating an EOP for each open packet 
     *     FRC_OFF (1) = Force all channels off and close all open packets
     *     NOP (0) = No effect
     */
    uint8_t   idc_cfg_grp_frc_off_all;

    /** pointer to DIO2 IQ IDC CHANNEL CONFIG GROUP setup */
    Iqn2Fl_Dio2IqIdcChCfgGrp   dio2_idc_ch_cfg_grp[16];

    /** DIO2 IFE FRM SAMP TC MMR RAM - SAMP_TC
     *  IFE AxC Framing Sample Terminal Count Configuration Register.
     *  Radio Framing Counter. Number of samples (4 Bytes) per radio symbol 
     *  programmed as a terminal count.
     */
    uint32_t   ife_frm_samp_tc_cfg_samp_tc[256];

    /** DIO2 UAT EGR RADT - TC CFG
     *  UAT RADT terminal count Register.
     *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with sys_clk=245.76MHz)
     */
    uint32_t   uat_egr_radt_tc_cfg_val[8];

    /** DIO2 UAT EGR RADT - OFFSET CFG
     *  UAT RADT offset Register.
     *  UAT RADT offset. Value which is added to the raw RADT as a timing 
     *  correction. RadT is initially randomly started, SW uses radt_capture 
     *  value to calculate offset correction factor. This correction factor 
     *  will be Frame size - captured value.
     */
    uint32_t   uat_egr_radt_offset_cfg_val[8];

    /** DIO2 UAT RADT EVT - EVT RADT CMP CFG
     *  UAT RADT event compare Register per RADT. The 0 to 7 are for si 
     *  egress, 8 to 15 for si ingress, 16 to 18 for dio egress, 19 to 
     *  21 for dio ingress.
     *  UAT RADT event compare per RADT. When compare value equals RADT 
     *  count, frame rate event is generated. Also periodic event 
     *  (i.e. 4SAMP) is started. The 0 to 7 are for si egress, 8 to 15 
     *  for si ingress, 16 to 18 for dio egress, 19 to 21 for dio ingress.
     */
    uint32_t   uat_evt_radt_cmp_cfg_val[22];

    /** DIO2 UAT RADT EVT - EVT CLK CNT TC CFG
     *  UAT RADT event compare Register per RADT. The 0 to 7 are for si 
     *  egress, 8 to 15 for si ingress, 16 to 18 for dio egress, 19 to 
     *  21 for dio ingress.
     *  UAT RADT event clock counter terminal count register per RADT.
     */
    uint16_t   uat_evt_clk_cnt_tc_cfg_val[22];

    /** DIO2 UAT DIO EGR RADT - TC CFG
     *  UAT DIO egress RADT terminal count Register.
     *  (DIO use only) UAT DIO egress RADT terminal count. 
     *  (i.e. 2,457,599 for WCDMA with sys_clk=245.76MHz)
     */
    uint32_t   uat_dio_egr_radt_tc_cfg_val[3];

    /** DIO2 UAT DIO EGR RADT - OFFSET CFG
     *  UAT DIO egress RADT offset Register.
     *  (DIO use only) UAT DIO egress RADT offset. Value which is added 
     *  to the raw RADT as a timing correction. RadT is initially 
     *  randomly started, SW uses radt_capture value to calculate 
     *  offset correction factor. This correction factor will be 
     *  Frame size - captured value.
     */
    uint32_t   uat_dio_egr_radt_offset_cfg_val[3];

    /** DIO2 UAT DIO ING RADT - TC CFG
     *  UAT DIO ingress RADT terminal count Register.
     *  (DIO use only) UAT DIO ingress RADT terminal count. 
     *  (i.e. 2,457,599 for WCDMA with sys_clk=245.76MHz)
     */
    uint32_t   uat_dio_ing_radt_tc_cfg_val[3];

    /** DIO2 UAT DIO ING RADT - OFFSET CFG
     *  UAT RADT offset Register.
     *  (DIO use only) UAT DIO ingress RADT offset. Value which is 
     *  added to the raw RADT as a timing correction. RadT is initially 
     *  randomly started, SW uses radt_capture value to calculate 
     *  offset correction factor. This correction factor will be 
     *  Frame size - captured value.
     */
    uint32_t   uat_dio_ing_radt_offset_cfg_val[3];

    /** DIO2 IQ EDC REGISTER GROUP - EDC CFG
     *  EDC Configuration Register.
     *  Controls how the EDC handles packet errors detected by efe
     *     DROP (0) = Drop the rest of the packet on errors
     *     NO_DROP (1) = Do not drop packet on errors
     */
    uint8_t   dio2_iq_edc_cfg_psi_err_chk_disable;

    /** DIO2 IQ EDC REGISTER GROUP - EDC CH CFG 
     *  Per-channel configuration registers.
     *  Byte swapping control.
     *     NONE (0) = no swap
     *     BYTE (1) = byte swap
     *     HALF (2) = half word swap. 16-bit swap
     *     WORD (3) = word swap. 32-bits
     */
    uint8_t  dio2_iq_edc_ch_cfg_dat_swap[16];

    /** DIO2 IQ EDC REGISTER GROUP - EDC CH CFG 
     *  Per-channel configuration registers.
     *  IQ swapping control.
     *     NONE1 (0) = no swap
     *     NONE2 (1) = no swap
     *     BYTE (2) = byte swap
     *     HALF (3) = 16-bit swap
     */
    uint8_t  dio2_iq_edc_ch_cfg_iq_order[16];

    /** DIO2 IQ INGRESS VBUS MMR GROUP 
     *  IDC Rate Control Configuration register. Programmable rate 
     *  control for Rate Controller.
     *  Rate Controller will allow the IDC to create RATE+1 active 
     *  requests on the PSI bus within a 32 clock cycle window. As an 
     *  example, a value of 7 will allow the IDC to create 8 active 
     *  requests within a 32-clock cycle window which uses 25% of the PSI bus.
     *  Default value is 15. Setting this value to a non-default value is possible if 0 < dio2_iq_idc_rate_ctl_cfg_rate <= 15
     *  If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_DIO2_IQ_IDC_RATE_CTL_REG CMD word
     */
    uint8_t   dio2_iq_idc_rate_ctl_cfg_rate;

    /** DIO2 GLOBAL CFG - VBUSM PRIORITY
     *  The DIO Global Configuration Register allows the user to select 
     *  priority on the VBUSM as well as control the Endeness of RSA 
     *  Data Format Conversian.
     *  Sets vbusm cpriority and cepriority for both reads and writes 
     *  (priority 0 = highest,7 = lowest, reset value = 0 (highest priority))
     */
    uint8_t   dio2_global_cfg_vbusm_priority;

    /** DIO2 GLOBAL CFG - RSA BIG ENDIAN
     *  The DIO Global Configuration Register allows the user to select 
     *  priority on the VBUSM as well as control the Endeness of RSA 
     *  Data Format Conversian.
     *  Sets RSA Data Format Conversian Endeness
     *     RSALTL (0) = Selects little endian
     *     RSABIG (1) = Selects big endian
     */
    uint8_t   dio2_global_cfg_rsa_big_endian;

    /** DIO2 CORE INGRESS
     */
    Iqn2Fl_Dio2CoreIngressSetup   dio2_core_ingress;

    /** DIO2 I AXC OFF MMR - OFFSET CFG
     *  A per-channel, Modulo 8 AxC offset in 4 sample i.e. 1 QW increments 
     *  that determine the starting address within a circular data buffer. 
     *  e.g. a value of 0 would cause zero offset. A value of 1 would cause 
     *  a 4 sample offset. A value of 0 would also represent an offset of 
     *  8, 16, etc.
     *  DIO Ingress AxC offset within a circular buffer.
     */
    uint8_t   dio2_i_axc_off_mmr_cfg_4samp_offset[16];

    /** DIO2 CORE EGRESS
     */
    Iqn2Fl_Dio2CoreEgressSetup   dio2_core_egress;

    /** DIO2 DT
     */
    Iqn2Fl_Dio2Dt   dio2_dt;

    /** DIO2 I DBCNT0/1/2 RAM MMR
     */
    Iqn2Fl_Dio2DbcntxRamMmr   dio2_i_dbcntx_ram_mmr[3];

    /** DIO2 E AOG RAM MMR - E AXC OFF CFG
     *  These registers are per channel and allow the user the ability to 
     *  program the Egress AxC offset from the Frame Boundary in 4 sample 
     *  (1 QW) increments. e.g. a value of 0 would cause zero offset from 
     *  frame boundary. A value of 1 would cause a 4 sample offset from 
     *  frame boundary.
     *  DIO Egress AxC offset from Frame Boundary in 4 sample (1 QW) increments
     */
    uint32_t   dio2_e_aog_ram_mmr_axc_off_cfg_4samp_offset[16];

    /** DIO2 E DBCNT0/1/2 RAM MMR
     */
    Iqn2Fl_Dio2DbcntxRamMmr   dio2_e_dbcntx_ram_mmr[3];

} Iqn2Fl_Dio2Setup;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_At2Setup. This structure is
 * used for configuring AT2 EVENTS 24ARRAY registers of At2 instance */
typedef struct 
{
    /** AT2 EVENTS 24ARRAY - EVENT OFFSET CFG - VAL
     *  event_offset (Step 2) after the start of frame/sym strobes, 
     *  the sys_event generator count out OFFSET clocks then fires once.
     *  Event Offset Index.
     */
    uint32_t   at2_events_24array_offset_cfg_val[24];

    /** AT2 EVENTS 24ARRAY - EVENT OFFSET CFG - EVT STRB SEL
     *  event_offset (Step 2) after the start of frame/sym strobes, 
     *  the sys_event generator count out OFFSET clocks then fires once.
     *  Selects which RADT counter and start-of-symbol vs. start-of-frame. 
     *  (Step 1) Each the selected strobe fires, the given sys_event 
     *  generator resets
     *     RADT0_frame (0) = Use RADT Frame to trigger the event
     *     RADT1_frame (1) = Use RADT Frame to trigger the event
     *     RADT2_frame (2) = Use RADT Frame to trigger the event
     *     RADT3_frame (3) = Use RADT Frame to trigger the event
     *     RADT4_frame (4) = Use RADT Frame to trigger the event
     *     RADT5_frame (5) = Use RADT Frame to trigger the event
     *     RADT6_frame (6) = Use RADT Frame to trigger the event
     *     RADT7_frame (7) = Use RADT Frame to trigger the event
     *     RADT0_symb (8) = Use RADT Symbol to trigger the event
     *     RADT1_symb (9) = Use RADT Symbol to trigger the event
     *     RADT2_symb (10) = Use RADT Symbol to trigger the event
     *     RADT3_symb (11) = Use RADT Symbol to trigger the event
     *     RADT4_symb (12) = Use RADT Symbol to trigger the event
     *     RADT5_symb (13) = Use RADT Symbol to trigger the event
     *     RADT6_symb (14) = Use RADT Symbol to trigger the event
     *     RADT7_symb (15) = Use RADT Symbol to trigger the event
     *     BCN_frame (16) = Use BCN Frame to trigger the event
     *     EVT_nrest (17) = Use Next higher event to trigger this event.
     */
    Iqn2Fl_AtEvtStrobe   at2_events_24array_offset_cfg_evt_strb_sel[24];

    /** AT2 EVENTS 24ARRAY - EVENT MOD TC CFG - VAL
     *  Periodic system event rate. Programmed in sys_clks minus 1. 
     *  (Step 3) after the event has fired once, the event will re-fire 
     *  every MOD clocks.
     *  Event Modulo.
     */
    uint32_t   at2_events_24array_mod_tc_cfg_val[24];

    /** AT2 EVENTS 24ARRAY - EVENT MASK LSBS CFG - VAL
     *  Programmably mask GSM system events. (Step 4) The system event 
     *  generator maintains a count which indexes this mask LUT starting 
     *  with the LSB. When set, the bit suppresses the system event. 
     *  This mechanism supports irregular patterns of system events. 
     *  After the table is exausted, after 64 events, the events are 
     *  always enabled. The LUT is recycled after (Step 1).
     *  Event Mask LSBs. all 0 will disable events, 1 per bit enables 
     *  that event time.
     */
    uint32_t   at2_events_24array_mask_lsbs_cfg_val[24];

    /** AT2 EVENTS 24ARRAY - EVENT MASK MSBS CFG - VAL
     *  Programmably mask GSM system events. 
     *  Event Mask MSBs. all 0 will disable events, 1 per bit enables 
     *  that event time.
     */
    uint32_t   at2_events_24array_mask_msbs_cfg_val[24];

} Iqn2Fl_At2Events24Array;


/**
 * @brief This is a sub-structure in @a Iqn2Fl_Setup. This structure is used for
 * configuring the parameters of AT2 level registers */
typedef struct 
{
    /** AT2 START - AT2 TIMER ENABLES CFG - RADT EN
     *  Control for each of the RADT and the BCN timers.
     *  bit 0-7 enable RadT0-to-RadT7. RADT 7 will be used for driving 
     *  the t1-t2-t3 GSM Timer. Once enabled, Timers will start running 
     *  once the compare value equals the BCN timer value (which yeilds 
     *  an exact sync).
     */
    uint8_t   at2_start_timer_enables_cfg_radt_en;

    /** AT2 START - AT2 TIMER ENABLES CFG - RUN BCN
     *  Control for each of the RADT and the BCN timers.
     *  SW control which starts the BCN Timer running. SW writes are not 
     *  precise so it is expected the user will correct the timer value 
     *  with an offset.
     */
    uint8_t   at2_start_timer_enables_cfg_run_bcn;

    /** AT2 RP1 - CTRL CFG - CRC USE
     *  Configuration for OBSAI RP1 CRC circuit. (Unused for CPRI or 
     *  non-RP1 OBSAI sync options).
     *  When active, a failed CRC check will result in the FCB being dropped.
     *     dont_use = (0)
     *     use = (1)
     */
    uint8_t   at2_rp1_ctrl_cfg_crc_use;

    /** AT2 RP1 - CTRL CFG - CRC FLIP
     *  Configuration for OBSAI RP1 CRC circuit. (Unused for CPRI or 
     *  non-RP1 OBSAI sync options).
     *  bit reversal of calculated CRC prior to comparison
     *     normal (0) = normal order
     *     reverse (1) = reverse order
     */
    uint8_t   at2_rp1_ctrl_cfg_crc_flip;

    /** AT2 RP1 - CTRL CFG - CRC INIT ONES
     *  Configuration for OBSAI RP1 CRC circuit. (Unused for CPRI or 
     *  non-RP1 OBSAI sync options).
     *  Initialization value of the CRC engine
     *     init0 = (0)
     *     init1 = (1)
     */
    uint8_t   at2_rp1_ctrl_cfg_crc_init_ones;

    /** AT2 RP1 - CTRL CFG - CRC INVERT
     *  Configuration for OBSAI RP1 CRC circuit. (Unused for CPRI or 
     *  non-RP1 OBSAI sync options).
     *  bit-by-bit inversion of calculated CRC value prior to comparison
     *     noinvert = (0)
     *     invert = (1)
     */
    uint8_t   at2_rp1_ctrl_cfg_crc_invert;

    /** AT2 BCN - OFFSET CFG - VAL
     *  Offset for the free running raw BCN counter. The offset version 
     *  of BCN is the value used for all measurement and sync purposes. 
     *  (The offset mechanism gives a way to minimize clock domains 
     *  crossing errors when syncing timers). SW uses the desired sync 
     *  input capture status to calculate offset correction factor. 
     *  This correction factor will be Frame size - captured value.
     */
    uint32_t   at2_bcn_offset_cfg_val;

    /** AT2 BCN - SLVSEL CFG - VAL
     *  BCN uat slave select.
     *  BCN uat slave select for capturing the BCN when the uat slave 
     *  sends back it's sync. This is for debug and small time adjustment 
     *  in systems which have different AIL uAT clk vs sys_clk, not in 
     *  TCI6630K2L since ail uAT is on the same sys_clk as AT.
     */
    uint8_t   at2_bcn_slvsel_cfg_val;

    /** AT2 BCN - FRM INIT LSBS CFG - WR VAL
     *  BCN Frame count value (BTS Frame Number). Increments every time 
     *  10ms BCN timer wraps. This Register only allows writing of the 
     *  value (Part 2) User may write to this register while BCN is 
     *  running. Advisable to write mid frame to avoid wrap uncertaninty.
     *  BCN Frame Init LSBs.
     */
    uint16_t   at2_bcn_frm_init_lsbs_cfg_wr_val;

    /** AT2 BCN - FRM INIT MSBS CFG - WR VAL
     *  BCN Frame count value (BTS Frame Number). Increments every time 
     *  10ms BCN timer wraps. This Register only allows writing of the 
     *  value (Part 1) User may write to this register while BCN is 
     *  running. 
     *  BCN Frame Init MSBs.
     */
    uint32_t   at2_bcn_frm_init_msbs_cfg_wr_val;

    /** AT2 BCN - CLKCNT TC CFG - VAL
     *  Terminal count of BCN. User should chose value which yields 10ms.
     *  (245.76MHz) 2,457,599 (307.2MHz) 3,071,999. 
     *  BCN Clock Counter TC.
     */
    uint32_t   at2_bcn_clkcnt_tc_cfg_val;

    /** AT2 BCN - FRAME TC LSB CFG - VAL
     *  BCN Frame count value (BTS Frame Number). Terminal count for 
     *  frame_count field. 
     *  BCN Frame TC LSBs.
     */
    uint32_t   at2_bcn_frame_tc_lsb_cfg_val;

    /** AT2 BCN - FRAME TC MSB CFG - VAL
     *  BCN Frame count value (BTS Frame Number). Terminal count for 
     *  frame_count field. 
     *  BCN Frame TC MSBs.
     */
    uint8_t   at2_bcn_frame_tc_msb_cfg_val;

    /** AT2 GSM - TCOUNT INIT CFG - T1
     *  Special GSM T1, T2, T3 count GSM frames in way which is useful 
     *  to GSM only APP SW. This register is for writing only and can 
     *  be written either before the timer starts counting or on-the-fly 
     *  (Users should avoid modifiying value near wrap position to avoid 
     *  uncertanty).
     *  T1 w=init, affects at_gsm_tcount_value.
     */
    uint16_t   at2_gsm_tcount_init_cfg_t1;

    /** AT2 GSM - TCOUNT INIT CFG - T2
     *  Special GSM T1, T2, T3 count GSM frames in way which is useful 
     *  to GSM only APP SW. This register is for writing only and can 
     *  be written either before the timer starts counting or on-the-fly 
     *  (Users should avoid modifiying value near wrap position to avoid 
     *  uncertanty).
     *  T2 w=init, affects at_gsm_tcount_value.
     */
    uint8_t   at2_gsm_tcount_init_cfg_t2;

    /** AT2 GSM - TCOUNT INIT CFG - T3
     *  Special GSM T1, T2, T3 count GSM frames in way which is useful 
     *  to GSM only APP SW. This register is for writing only and can 
     *  be written either before the timer starts counting or on-the-fly 
     *  (Users should avoid modifiying value near wrap position to avoid 
     *  uncertanty).
     *  T3 w=init, affects at_gsm_tcount_value.
     */
    uint8_t   at2_gsm_tcount_init_cfg_t3;

    /** AT2 RADT - INIT 1 CFG - FRM INIT LSB
     *  RADT Frame Init LSBs.
     *  RADT Frame Init LSBs loads a frame count directly to the counter 
     *  LSB bits. This register and the MSB register should only be loaded 
     *  when the RADT is off or if it is known it will not be incrementing 
     *  during the time of the writes.
     */
    uint16_t   at2_radt_init_1_cfg_frm_init_lsb[8];

    /** AT2 RADT - INIT 2 CFG - FRM INIT MSB
     *  RADT Frame Init MSBs.
     *  RADT Frame Init MSBs loads a frame count directly to the counter 
     *  MSB bits. This register and the LSB register should only be loaded 
     *  when the RADT is off or if it is known it will not be incrementing 
     *  during the time of the writes.
     */
    uint32_t   at2_radt_init_2_cfg_frm_init_msb[8];

    /** AT2 RADT - 0 CFG - CLKCNT TC
     *  AT2 RADT clock and lutindex and symbol counter terminal counts.
     *  Clock divider counter TC 11bit. The clock divider divides down 
     *  the clock to a sample period before driving the sample counter. 
     *  Represents num clock ticks minus 1 per sample.
     */
    uint16_t   at2_radt_0_cfg_clkcnt_tc[8];

    /** AT2 RADT - 0 CFG - LUTINDEX TC
     *  AT2 RADT clock and lutindex and symbol counter terminal counts.
     *  LUT Index TC 8bit.
     */
    uint8_t   at2_radt_0_cfg_lutindex_tc[8];

    /** AT2 RADT - 0 CFG - SYMB TC
     *  AT2 RADT clock and lutindex and symbol counter terminal counts.
     *  Symbol TC 8bit.
     */
    uint8_t   at2_radt_0_cfg_symb_tc[8];

    /** AT2 RADT - 1 CFG - FRM TC LSB
     *  RADT Frame TC LSBs.
     */
    uint32_t   at2_radt_1_cfg_frm_tc_lsb[8];

    /** AT2 RADT - 2 CFG - FRM TC MSB
     *  RADT Frame TC MSBs.
     */
    uint8_t   at2_radt_2_cfg_frm_tc_msb[8];

    /** AT2 RADT - 3 CFG - LUT INDEX STRT
     *  LUT Index start address pointer to the symbol lut memory.
     */
    uint8_t   at2_radt_3_cfg_lut_index_strt[8];

    /** AT2 RADT - 4 CFG - BCN SYNC CMP
     *  RADT BCN sync compare.
     *  BCN compare value for synchronization to start the RADT.
     */
    uint32_t   at2_radt_4_cfg_bcn_sync_cmp[8];

    /** AT2 EVENTS 24ARRAY registers
     */
    Iqn2Fl_At2Events24Array   at2_events_24array;

    /** AT2 EVENTS - EVT ENABLE CFG - EN
     *  AT2 system events control APP SW timing. This MMR Enables each 
     *  of 24 of these system events.
     *  EVENT Enable when a bit is set to a 1. Must be enabled after 
     *  event configuration.  Setting the bit to 0 disables the EVENT.
     */
     uint32_t   at2_evt_enable_cfg;

    /** AT2 RADT SYM LUT RAM - CFG - SYMBCNT TC
     *  RAM based LUT which give a clock count length per sym/time_slot. 
     *  This table approach gives a flexible approach to program framing 
     *  timing for any random radio standard. For GSM, use 104 table 
     *  entries, one each for 104 time_slots per 60 ms. For LTE 
     *  norm_cyc_prefix use 14 table entries, one each for every symbol 
     *  in a sub-frame. For WCDMA use a single entry to be replayed 15 
     *  (time slots) times per frame.
     *  RadT symbol nounter terminal count programmed in sys_clks (usually 
     *  245.76MHz or 307.2MHz)
     */
    uint32_t   at2_radt_sym_lut_ram_cfg_symbcnt_tc[256];

} Iqn2Fl_At2Setup;

/** @brief This object contains the iqn2 EE error and alarm origination information */
typedef struct
{
   /** EE AT origination status value */
    uint32_t    at_ee_sts;
    /** EE AID origination status value */
    uint32_t    aid_ee_sts;
    /** EE DFE origination status value */
    uint32_t    dfe_ee_sts;
    /** EE DIO origination status value */
    uint32_t    dio_ee_sts;
    /** EE IQS origination status value */
    uint32_t    iqs_ee_sts;
    /** EE AIL0 origination status value */
    uint32_t    ail0_ee_sts;
    /** EE AIL1 origination status value */
    uint32_t    ail1_ee_sts;
    /** EE AIL2 origination status value */
    uint32_t    ail2_ee_sts;
    /** EE AIL3 origination status value */
    uint32_t    ail3_ee_sts;
    /** EE PSR origination status value */
    uint32_t    psr_ee_sts;
} Iqn2Fl_EeOrigin;

/** @brief This object contains the iqn2 TOP PSR EE error and alarm origination information */
typedef struct
{
    /** PSR EE_ING_FLUSH_A origination status value */
    uint32_t    psr_ee_ing_flush_a_sts;
    /** PSR EE_ING_FLUSH_B origination status value */
    uint32_t    psr_ee_ing_flush_b_sts;
    /** PSR EE_EGR_PROTOCOL_ERR_A origination status value */
    uint32_t    psr_ee_egr_protocol_err_a_sts;
    /** PSR EE_EGR_PROTOCOL_ERR_B origination status value */
    uint32_t    psr_ee_egr_protocol_err_b_sts;
} Iqn2Fl_PsrEeOrigin;

/** @brief This object contains the iqn2 IQS2 EE error and alarm origination information */
typedef struct
{
    /** IQS2 EE_CHAN_ERR origination status value */
    uint32_t    iqs2_ee_chan_err_sts;
    /** IQS2 EE ING_FLUSH_ERR origination status value */
    uint32_t    iqs2_ee_ing_flush_err_sts;
    /** IQS2 EE EGR_FLUSH_ERR origination status value */
    uint32_t    iqs2_ee_egr_flush_err_sts;
} Iqn2Fl_Iqs2EeOrigin;

/** @brief This object contains the iqn2 AID2 SYSCLK EE error and alarm origination information */
typedef struct
{
    /** AID2 ee_sii_a origination status value */
    uint32_t    aid2_ee_sii_a_sts;
    /** AID2 ee_sii_b origination status value */
    uint32_t    aid2_ee_sii_b_sts;
    /** AID2 ee_sii_c origination status value */
    uint32_t    aid2_ee_sii_c_sts;
    /** AID2 ee_sii_d origination status value */
    uint32_t    aid2_ee_sii_d_sts;
    /** AID2 ee_sie_a origination status value */
    uint32_t    aid2_ee_sie_a_sts;
    /** AID2 ee_sie_b origination status value */
    uint32_t    aid2_ee_sie_b_sts;
    /** AID2 ee_sie_c origination status value */
    uint32_t    aid2_ee_sie_c_sts;
} Iqn2Fl_Aid2SysClkEeOrigin;

/** @brief This object contains the iqn2 AID2 VBUSCLK EE error and alarm origination information */
typedef struct
{
    /** AID2 ee_sii_e origination status value */
    uint32_t    aid2_ee_sii_e_sts;
    /** AID2 ee_sii_f origination status value */
    uint32_t    aid2_ee_sii_f_sts;
    /** AID2 ee_sii_g origination status value */
    uint32_t    aid2_ee_sii_g_sts;
    /** AID2 ee_sii_h origination status value */
    uint32_t    aid2_ee_sii_h_sts;
    /** AID2 ee_sie_d origination status value */
    uint32_t    aid2_ee_sie_d_sts;
    /** AID2 ee_sie_e origination status value */
    uint32_t    aid2_ee_sie_e_sts;
    /** AID2 ee_sie_f origination status value */
    uint32_t    aid2_ee_sie_f_sts;
    /** AID2 ee_sie_g origination status value */
    uint32_t    aid2_ee_sie_g_sts;
} Iqn2Fl_Aid2VbusClkEeOrigin;

/** @brief This object contains the iqn2 AIL VBUSCLK EE error and alarm origination information */
typedef struct
{
    /** AIL ee_sii_e origination status value */
    uint32_t    ail_ee_sii_e_sts;
    /** AIL ee_sii_f origination status value */
    uint32_t    ail_ee_sii_f_sts;
    /** AIL ee_sii_g 0 origination status value */
    uint32_t    ail_ee_sii_g_0_sts;
    /** AIL ee_sii_g 1 origination status value */
    uint32_t    ail_ee_sii_g_1_sts;
    /** AIL ee_sii_h origination status value */
    uint32_t    ail_ee_sii_h_sts;
    /** AIL ee_sie_d origination status value */
    uint32_t    ail_ee_sie_d_sts;
    /** AIL ee_sie_e origination status value */
    uint32_t    ail_ee_sie_e_sts;
    /** AIL ee_sie_f 0 origination status value */
    uint32_t    ail_ee_sie_f_0_sts;
    /** AIL ee_sie_f 1 origination status value */
    uint32_t    ail_ee_sie_f_1_sts;
    /** AIL ee_sie_g origination status value */
    uint32_t    ail_ee_sie_g_sts;
} Iqn2Fl_AilVbusClkEeOrigin;

/** @brief This object contains the iqn2 AIL SYSCLK PHY EE error and alarm origination information */
typedef struct
{
    /** AIL ee_rm_0  origination status value */
    uint32_t    ail_ee_rm_0_sts;
    /** AIL ee_rt_tm_0 origination status value */
    uint32_t    ail_ee_rt_tm_0_sts;
    /** AIL ee_ci_co 0 origination status value */
    uint32_t    ail_ee_ci_co_0_sts;
} Iqn2Fl_AilSysClkPhyEeOrigin;

/** @brief This object contains the iqn2 AIL SYSCLK EE error and alarm origination information */
typedef struct
{
    /** AIL ee_pd_0  origination status value */
    uint32_t    ail_ee_pd_0_sts;
    /** AIL ee_pd_1  origination status value */
    uint32_t    ail_ee_pd_1_sts;
    /** AIL ee_pd_2_0  origination status value */
    uint32_t    ail_ee_pd_2_0_sts;
    /** AIL ee_pd_2_1  origination status value */
    uint32_t    ail_ee_pd_2_1_sts;
    /** AIL ee_pe_0  origination status value */
    uint32_t    ail_ee_pe_0_sts;
    /** AIL ee_si_0  origination status value */
    uint32_t    ail_ee_si_0_sts;
    /** AIL ee_sii_a  origination status value */
    uint32_t    ail_ee_sii_a_sts;
    /** AIL ee_sii_b  origination status value */
    uint32_t    ail_ee_sii_b_sts;
    /** AIL ee_sii_c_0  origination status value */
    uint32_t    ail_ee_sii_c_0_sts;
    /** AIL ee_sii_c_1  origination status value */
    uint32_t    ail_ee_sii_c_1_sts;
    /** AIL ee_sii_d  origination status value */
    uint32_t    ail_ee_sii_d_sts;
    /** AIL ee_sie_a  origination status value */
    uint32_t    ail_ee_sie_a_sts;
    /** AIL ee_sie_b  origination status value */
    uint32_t    ail_ee_sie_b_sts;
    /** AIL ee_sie_c  origination status value */
    uint32_t    ail_ee_sie_c_sts;
} Iqn2Fl_AilSysClkEeOrigin;

/** @brief This object contains the iqn2 DIO2 EE error and alarm origination information */
typedef struct
{
    /** DIO2 ee_dma_ing_a  origination status value */
    uint32_t    dio2_ee_dma_ing_a_sts;
    /** DIO2 ee_dma_ing_b  origination status value */
    uint32_t    dio2_ee_dma_ing_b_sts;
    /** DIO2 ee_dma_egr_a  origination status value */
    uint32_t    dio2_ee_dma_egr_a_sts;
    /** DIO2 ee_dt_a  origination status value */
    uint32_t    dio2_ee_dt_a_sts;
    /** DIO2 ee_sii_a  origination status value */
    uint32_t    dio2_ee_sii_a_sts;
    /** DIO2 ee_sii_c  origination status value */
    uint32_t    dio2_ee_sii_c_sts;
    /** DIO2 ee_sie_a  origination status value */
    uint32_t    dio2_ee_sie_a_sts;
    /** DIO2 ee_sii_e  origination status value */
    uint32_t    dio2_ee_sii_e_sts;
    /** DIO2 ee_sii_g  origination status value */
    uint32_t    dio2_ee_sii_g_sts;
    /** DIO2 ee_sie_d  origination status value */
    uint32_t    dio2_ee_sie_d_sts;
    /** DIO2 ee_sie_e  origination status value */
    uint32_t    dio2_ee_sie_e_sts;
    /** DIO2 ee_sie_f  origination status value */
    uint32_t    dio2_ee_sie_f_sts;
} Iqn2Fl_Dio2EeOrigin;

/** @brief This object contains the TOP PSR EE INGRESS FLUSH A information */
typedef struct
{
   /** Channel flush indication for ingress channels 0 through 31 */
   uint32_t    err;
} Iqn2Fl_PsrEeIngFlushA;

/** @brief This object contains the TOP PSR EE INGRESS FLUSH B information */
typedef struct
{
   /** Channel flush indication for ingress channels 32 through 47 */
   uint32_t    err;
} Iqn2Fl_PsrEeIngFlushB;

/** @brief This object contains the TOP PSR EE EGRESS PROTOCOL ERR A information */
typedef struct
{
   /** Protocol error indication of an unsupported data type or a missing PS_DATA transfer
    * when one was expected for egress channels 0 through 31 */
   uint32_t    err;
} Iqn2Fl_PsrEeEgressProtocolErrA;

/** @brief This object contains the TOP PSR EE EGRESS PROTOCOL ERR B information */
typedef struct
{
   /** Protocol error indication of an unsupported data type or a missing PS_DATA transfer
    * when one was expected for egress channels 32 through 47 */
   uint32_t    err;
} Iqn2Fl_PsrEeEgressProtocolErrB;

/** @brief This object contains the TOP PKTDMA EE DESC STARVE information */
typedef struct
{
   /** PKTDMA SOP Descriptor starvation error. There were not enough descriptors available for
    * the PKTDMA to start the transfer of a packet */
   uint32_t    sop_err;
   /** PKTDMA MOP Descriptor starvation error There were not enough descriptors available for
    * the PKTDMA to complete the transfer of a packet it already started. */
   uint32_t    mop_err;
} Iqn2Fl_PktdmaEeDescStarve;

/** @brief This object contains the AT2 EE AT information */
typedef struct
{
   /** RP1 CRC error (Only valid with using OBSAI RP1 Sync Interface). RP1 FCB was recieved with
    * bad CRC. APP SW should disregard FCB */
   uint32_t   rp1_crc_err;
   /** RP1 bit error (Only valid with using OBSAI RP1 Sync Interface). RP1 FCB recieved pattern is
    * illegal. Data bits were not held for 8 consecutive RP1_clks */
   uint32_t   rp1_bit_err;
   /** An RP1 FCB packet was received (Only valid with using OBSAI RP1 Sync Interface). BCN
    * value was captured for use by APP SWto calc and correct BCN alignment */
   uint32_t   rp1_sync_info;
   /** A strobe was recieved on the radt sync input pin (Only valid when using external sync pin).
    * BCN value was captured for use by APP SWto calc and correct RADT alignment */
   uint32_t   radsync_info;
   /** A strobe was recieved on the phyt (BCN) sync input pin (Only valid when using external
    * sync pin) BCN value was captured for use byAPP SW to calc and correct BCN alignment */
   uint32_t   physync_info;
   /** A pa_tscomp sync input was received from NETCP PA (Valid when using IEEE1588 sync).
    * BCN value was captured for use by APP SW to calc and correct BCN alignment */
   uint32_t   pa_tscomp_info;
} Iqn2Fl_At2EeInfoErr;

/** @brief This object contains the IQS2 EE CHAN ERR information */
typedef struct
{
   /** Ingress PKTDMA detected a channel error. This occurs when the destination port number
    * in the ingress port LUT configuration registeris higher than the number of channels
    * available in the pktdma. This always indicates a programming error. */
   uint32_t   ing_pktdma_err;
   /** Ingress DIO2 detected a channel error. This occurs when the destination port number in
    * the ingress port LUT configuration register is higher than the number of channels
    * available in the DIO. This always indicates a programming error */
   uint32_t   ing_dio2_err;
   /** Egress Mux detected a port or channel error. This occurs when the destination port
    * number in the egress PKTDMA or DIO LUT configuration register is higher than the
    * number of output ports available or higher than the maximum number of channels in the
    * largest port. This always indicates a programming error. */
   uint32_t   egr_mux_err;
} Iqn2Fl_Iqs2EeChanErr;

/** @brief This object contains the IQS2 EE INGRESS FLUSH ERR information */
typedef struct
{
   /** DIO2 Ingress path detected a flush condition. Either the DIO2 detected a flush or there was
    * a write to the DIO2 Ingress FIFO when the FIFO for that channel was full. */
   uint32_t   flush;
} Iqn2Fl_Iqs2EeIngFlush;

/** @brief This object contains the IQS2 EE EGRESS FLUSH ERR information */
typedef struct
{
   /** DIO2 Ingress path detected a flush condition. Either the DIO2 detected a flush or there was
    * a write to the DIO2 Ingress FIFO when the FIFO for that channel was full. */
   uint32_t   flush;
} Iqn2Fl_Iqs2EeEgrFlush;

/** @brief This object contains the AID2 EE SII A ERR information */
typedef struct
{
   /** SI Ingress IQ ICC Start of Frame received */
   uint32_t   si_ing_iq_icc_sof_info;
   /** SI Ingress IQ ICC data transfer received */
   uint32_t   si_ing_iq_icc_dat_info;
   /** SI Ingress IQ IFE transmitted SOP */
   uint32_t   si_ing_iq_ife_sop_info;
   /** SI Ingress IQ IFE transmitted EOP */
   uint32_t   si_ing_iq_ife_eop_info;
   /** SI Ingress IQIFE transmitted valid data phase */
   uint32_t   si_ing_iq_ife_dat_info;
   /** SI Ingress IQ FIFO overflow error */
   uint32_t   si_ing_iq_fifo_ovfl_err;
} Iqn2Fl_Aid2EeSiiA;

/** @brief This object contains the AID2 EE SII B ERR information */
typedef struct
{
   /** SI Ingress CTL Packet error occurred */
   uint32_t   si_ing_ctl_pkt_err;
   /** SI Ingress CTL received EOP from ICC */
   uint32_t   si_ing_ctl_icc_eop_info;
   /** SI Ingress CTL received valid data phase from ICC */
   uint32_t   si_ing_ctl_icc_dat_info;
   /** SI Ingress CTL FIFO overflow error */
   uint32_t   si_ing_ctl_fifo_ovfl_err;
} Iqn2Fl_Aid2EeSiiB;

/** @brief This object contains the AID2 EE SII C ERR information */
typedef struct
{
   /** SI Ingress IQ per channel Start of Frame alignment error */
   uint32_t   si_ing_iq_sof_err;
} Iqn2Fl_Aid2EeSiiC;

/** @brief This object contains the AID2 EE SII D ERR information */
typedef struct
{
   /** SI Ingress CTL per channel SOP received from ICC */
   uint32_t   si_ing_ctl_icc_sop_info;
} Iqn2Fl_Aid2EeSiiD;

/** @brief This object contains the AID2 EE SIE A ERR information */
typedef struct
{
   /** SI Egress IQ EFE data starvation error */
   uint32_t   si_egr_iq_efe_starve_err;
   /** SI Egress IQ EFE packet boundary error */
   uint32_t   si_egr_iq_efe_pkt_err;
   /** SI Egress IQ EFE symbol number error. (Packet boundary errors due to a missing, early, or
    * late SOP disable the reporting of any symbol errors since the symbol number is only valid
    * for SOPs) */
   uint32_t   si_egr_iq_efe_sym_err;
   /** SI Egress IQ transmitted Start of Frame to ICC */
   uint32_t   si_egr_iq_icc_sof_info;
   /** SI Egress IQ transmitted data to ICC */
   uint32_t   si_egr_iq_icc_dat_info;
} Iqn2Fl_Aid2EeSieA;

/** @brief This object contains the AID2 EE SIE B ERR information */
typedef struct
{
   /** SI Egress CTL transmitted EOP to ICC */
   uint32_t   si_egr_ctl_icc_eop_info;
   /** SI Egress CTL transmitted data to ICC */
   uint32_t   si_egr_ctl_icc_dat_info;
} Iqn2Fl_Aid2EeSieB;

/** @brief This object contains the AID2 EE SIE C ERR information */
typedef struct
{
   /** SI Egress CTL per channel SOP transmitted to ICC */
   uint32_t   si_egr_ctl_icc_sop_info;
} Iqn2Fl_Aid2EeSieC;

/** @brief This object contains the AID2 EE DFE ERR information */
typedef struct
{
   /** DFE error interrupts */
   uint32_t   dfe_iqn_aid2_err;
} Iqn2Fl_Aid2EeDfe;

/** @brief This object contains the AID2 EE SII E ERR information */
typedef struct
{
   /** SI Ingress IQ EOP transmitted to PSI */
   uint32_t   si_ing_iq_psi_eop_info;
   /** SI Ingress IQ data transmitted to PSI */
   uint32_t   si_ing_iq_psi_dat_info;
} Iqn2Fl_Aid2EeSiiE;

/** @brief This object contains the AID2 EE SII F ERR information */
typedef struct
{
   /** SI Ingress CTL EOP transmitted to PSI */
   uint32_t   si_ing_ctl_psi_eop_info;
   /** SI Ingress CTL data transmitted to PSI */
   uint32_t   si_ing_ctl_psi_dat_info;
} Iqn2Fl_Aid2EeSiiF;

/** @brief This object contains the AID2 EE SII G ERR information */
typedef struct
{
   /** SI Ingress IQ SOP transmitted to PSI */
   uint32_t   si_ing_iq_psi_sop_info;
} Iqn2Fl_Aid2EeSiiG;

/** @brief This object contains the AID2 EE SII H ERR information */
typedef struct
{
   /** SI Ingress CTL per channel SOP transmitted to PSI */
   uint32_t   si_ing_ctl_psi_sop_info;
} Iqn2Fl_Aid2EeSiiH;

/** @brief This object contains the AID2 EE SIE D ERR information */
typedef struct
{
   /** SI Egress IQ PSI data type error */
   uint32_t   si_egr_iq_psi_data_type_err;
   /** SI Egress IQ EOP received from PSI */
   uint32_t   si_egr_iq_psi_eop_info;
   /** SI Egress IQ valid data received from PSI */
   uint32_t   si_egr_iq_psi_dat_info;
} Iqn2Fl_Aid2EeSieD;

/** @brief This object contains the AID2 EE SIE E ERR information */
typedef struct
{
   /** SI Egress CTL PSI data type error */
   uint32_t   si_egr_ctl_psi_data_type_err;
   /** SI Egress CTL EOP received from PSI */
   uint32_t   si_egr_ctl_psi_eop_info;
   /** SI Egress CTL valid data received from PSI */
   uint32_t   si_egr_ctl_psi_dat_info;
} Iqn2Fl_Aid2EeSieE;

/** @brief This object contains the AID2 EE SIE F ERR information */
typedef struct
{
   /** SI Ingress IQ per channel SOP received from PSI */
   uint32_t   si_egr_iq_psi_sop_info;
} Iqn2Fl_Aid2EeSieF;

/** @brief This object contains the AID2 EE SIE G ERR information */
typedef struct
{
   /** SI Ingress CLT per channel SOP received from PSI */
   uint32_t   si_egr_ctl_psi_sop_info;
} Iqn2Fl_Aid2EeSieG;

/** @brief This object contains the AIL EE SII A ERR information */
typedef struct
{
   /** SI Ingress IQ ICC Start of Frame received */
   uint32_t   si_ing_iq_icc_sof_info;
   /** SI Ingress IQ ICC data transfer received */
   uint32_t   si_ing_iq_icc_dat_info;
   /** SI Ingress IQ IFE transmitted SOP */
   uint32_t   si_ing_iq_ife_sop_info;
   /** SI Ingress IQ IFE transmitted EOP */
   uint32_t   si_ing_iq_ife_eop_info;
   /** SI Ingress IQIFE transmitted valid data phase */
   uint32_t   si_ing_iq_ife_dat_info;
   /** SI Ingress IQ FIFO overflow error */
   uint32_t   si_ing_iq_fifo_ovfl_err;
} Iqn2Fl_AilEeSiiA;

/** @brief This object contains the AIL EE SII B ERR information */
typedef struct
{
   /** SI Ingress CTL Packet error occurred */
   uint32_t   si_ing_ctl_pkt_err;
   /** SI Ingress CTL received EOP from ICC */
   uint32_t   si_ing_ctl_icc_eop_info;
   /** SI Ingress CTL received valid data phase from ICC */
   uint32_t   si_ing_ctl_icc_dat_info;
   /** SI Ingress CTL FIFO overflow error */
   uint32_t   si_ing_ctl_fifo_ovfl_err;
} Iqn2Fl_AilEeSiiB;

/** @brief This object contains the AIL EE SII C 0 ERR information */
typedef struct
{
   /** SI Ingress IQ per channel Start of Frame alignment error */
   uint32_t   si_ing_iq_sof_err;
} Iqn2Fl_AilEeSiiC0;

/** @brief This object contains the AIL EE SII C 1 ERR information */
typedef struct
{
   /** SI Ingress IQ per channel Start of Frame alignment error */
   uint32_t   si_ing_iq_sof_err_64_32;
} Iqn2Fl_AilEeSiiC1;

/** @brief This object contains the AIL EE SII D ERR information */
typedef struct
{
   /** SI Ingress CTL per channel SOP received from ICC */
   uint32_t   si_ing_ctl_icc_sop_info;
} Iqn2Fl_AilEeSiiD;

/** @brief This object contains the AIL EE SII E ERR information */
typedef struct
{
   /** SI Ingress IQ EOP transmitted to PSI */
   uint32_t   si_ing_iq_psi_eop_info;
   /** SI Ingress IQ data transmitted to PSI */
   uint32_t   si_ing_iq_psi_dat_info;
} Iqn2Fl_AilEeSiiE;

/** @brief This object contains the AIL EE SII F ERR information */
typedef struct
{
   /** SI Ingress CTL EOP transmitted to PSI */
   uint32_t   si_ing_ctl_psi_eop_info;
   /** SI Ingress CTL data transmitted to PSI */
   uint32_t   si_ing_ctl_psi_dat_info;
} Iqn2Fl_AilEeSiiF;

/** @brief This object contains the AIL EE SII G 0 ERR information */
typedef struct
{
   /** SI Ingress IQ SOP transmitted to PSI */
   uint32_t   si_ing_iq_psi_sop_info;
} Iqn2Fl_AilEeSiiG0;

/** @brief This object contains the AIL EE SII G 1 ERR information */
typedef struct
{
   /** SI Ingress IQ SOP transmitted to PSI */
   uint32_t   si_ing_iq_psi_sop_info_64_32;
} Iqn2Fl_AilEeSiiG1;

/** @brief This object contains the AIL EE SII H ERR information */
typedef struct
{
   /** SI Ingress CTL per channel SOP transmitted to PSI */
   uint32_t   si_ing_ctl_psi_sop_info;
} Iqn2Fl_AilEeSiiH;

/** @brief This object contains the AIL EE SIE A ERR information */
typedef struct
{
   /** SI Egress IQ EFE data starvation error */
   uint32_t   si_egr_iq_efe_starve_err;
   /** SI Egress IQ EFE packet boundary error */
   uint32_t   si_egr_iq_efe_pkt_err;
   /** SI Egress IQ EFE symbol number error. (Packet boundary errors due to a missing, early, or
    * late SOP disable the reporting of any symbol errors since the symbol number is only valid
    * for SOPs) */
   uint32_t   si_egr_iq_efe_sym_err;
   /** SI Egress IQ transmitted Start of Frame to ICC */
   uint32_t   si_egr_iq_icc_sof_info;
   /** SI Egress IQ transmitted data to ICC */
   uint32_t   si_egr_iq_icc_dat_info;
} Iqn2Fl_AilEeSieA;

/** @brief This object contains the AIL EE SIE B ERR information */
typedef struct
{
   /** SI Egress CTL transmitted EOP to ICC */
   uint32_t   si_egr_ctl_icc_eop_info;
   /** SI Egress CTL transmitted data to ICC */
   uint32_t   si_egr_ctl_icc_dat_info;
} Iqn2Fl_AilEeSieB;

/** @brief This object contains the AIL EE SIE C ERR information */
typedef struct
{
   /** SI Egress CTL per channel SOP transmitted to ICC */
   uint32_t   si_egr_ctl_icc_sop_info;
} Iqn2Fl_AilEeSieC;

/** @brief This object contains the AIL EE SIE D ERR information */
typedef struct
{
   /** SI Egress IQ PSI data type error */
   uint32_t   si_egr_iq_psi_data_type_err;
   /** SI Egress IQ EOP received from PSI */
   uint32_t   si_egr_iq_psi_eop_info;
   /** SI Egress IQ valid data received from PSI */
   uint32_t   si_egr_iq_psi_dat_info;
} Iqn2Fl_AilEeSieD;

/** @brief This object contains the AIL EE SIE E ERR information */
typedef struct
{
   /** SI Egress CTL PSI data type error */
   uint32_t   si_egr_ctl_psi_data_type_err;
   /** SI Egress CTL EOP received from PSI */
   uint32_t   si_egr_ctl_psi_eop_info;
   /** SI Egress CTL valid data received from PSI */
   uint32_t   si_egr_ctl_psi_dat_info;
} Iqn2Fl_AilEeSieE;

/** @brief This object contains the AIL EE SIE F 0 ERR information */
typedef struct
{
   /** SI Ingress IQ per channel SOP received from PSI */
   uint32_t   si_egr_iq_psi_sop_info;
} Iqn2Fl_AilEeSieF0;

/** @brief This object contains the AIL EE SIE F 1 ERR information */
typedef struct
{
   /** SI Ingress IQ per channel SOP received from PSI */
   uint32_t   si_egr_iq_psi_sop_info_64_32;
} Iqn2Fl_AilEeSieF1;

/** @brief This object contains the AIL EE SIE G ERR information */
typedef struct
{
   /** SI Ingress CLT per channel SOP received from PSI */
   uint32_t   si_egr_ctl_psi_sop_info;
} Iqn2Fl_AilEeSieG;

/** @brief This object contains the AIL EE RM 0 ERR information */
typedef struct
{
   /** Sync status has changed in the Rx state machine */
   uint32_t   sync_status_change;
   /** The Rx state machine of the RM is in state0, indicating the link is idle */
   uint32_t   rm_status_state0;
   /** The Rx state machine of the RM is in state1, indicating the link is in K character recieved  state */
   uint32_t   rm_status_state1;
   /** The Rx state machine of the RM is in state2, indicating the link is in pre-sync state */
   uint32_t   rm_status_state2;
   /** The Rx state machine of the RM is in state3, indicating the link is in sync state */
   uint32_t   rm_status_state3;
   /** The Rx state machine of the RM is in state4, indicating the link isin OBSAI scrambler detect state */
   uint32_t   rm_status_state4;
   /** The Rx state machine of the RM is in state5, indicating the link is in OBSAI scramberl received state */
   uint32_t   rm_status_state5;
   /** Number of los detected over los_thold number */
   uint32_t   num_los_det;
   /** lcv error detected */
   uint32_t   lcv_det;
   /** The RM has detected Frame boundary */
   uint32_t   frame_bndry_det;
   /** The RM has detected a valid block boundary */
   uint32_t   block_bndry_det;
   /** An expected k28p5 was missed by the Rx state machine */
   uint32_t   missing_k28p5;
   /** An expected k28p7 was missed by the Rx state machine */
   uint32_t   missing_k28p7;
   /** A k30p7 was detected in the RM received byte data */
   uint32_t   k30p7_det;
   /** Loc was detected in the RM block as determined by the clock monitor circuit */
   uint32_t   loc_det;
   /** RX Fifo has overflowed, indicating the rx clock is faster than the system clock */
   uint32_t   rx_fifo_ovf;
   /** Received an los in the CPRI L1 inband signal while in CPRI mode */
   uint32_t   rcvd_los;
   /** Received an lof in the CPRI L1 inband signal while in CPRI mode */
   uint32_t   rcvd_lof;
   /** Received an rai (Remote Alarm Indicator) in the CPRI L1inband signal while in CPRI mode */
   uint32_t   rcvd_rai;
   /** Recevied an sdi (Signal Defect Indicator) in the CPRI L1inband signal while in CPRI mode */
   uint32_t   rcvd_sdi;
   /** Received a rst set high in the CPRI L1 inband signal while in CPRI mode */
   uint32_t   rcvd_rst;
   /** An los error has occurred in the Rx state machine, indicating that a valid signal is no longer received from the ser-des interface */
   uint32_t   los_err;
   /** An lof error has occurred in the Rx state machine, indicating that a valid frame format is no longer received from the ser-des interface */
   uint32_t   lof_err;
   /** hfnsync state indicates that the Rx state has recognized the hfn value in CPRI mode */
   uint32_t   hfnsync_state;
   /** lof state indicates the Rx state machine lost frame */
   uint32_t   lof_state;
   /** RM reset is an SOC reset generated when the L1 inband rst signal is valid high for 5 consecutive frames */
   uint32_t   rm_rst;
} Iqn2Fl_AilEeRm0;

/** @brief This object contains the AIL EE RT TM 0 ERR information */
typedef struct
{
   /** RT detected an error in the header */
   uint32_t   rt_hdr_error;
   /** RT inserted an empty message */
   uint32_t   rt_em_insert;
   /** RT FIFO Underflow */
   uint32_t   rt_unfl;
   /** RT FIFO Overflow */
   uint32_t   rt_ovfl;
   /** RT detected a framing error fromeither the CI input or the PE input */
   uint32_t   rt_frm_err;
   /** RT detected the alignment window is too large in RE ULNK mode */
   uint32_t   rt_unalign_err;
   /** RT aggregation state info bit 0 = aggr_init, bit 1 = aggr_pe, bit 2 = aggr_rm, bit 3 = aggr_pe_rm */
   uint32_t   rt_aggr_state_info;
   /** TM detects, if AIL link is in frame sync state or not. This field goes high as soon as the TM status has changed to frame sync state */
   uint32_t   tm_frame_sync_state;
   /** TM detected that the frame strobe from the micro-at seems inactive compared to the Tx state machine and link configuration. */
   uint32_t   delta_inactive;
   /** TM detected that the frame strobe from the micro-at was received earlier than expected */
   uint32_t   delta_modified;
   /** TM detected a frame misalignment in the signal it receives from the CO */
   uint32_t   frame_misalign;
   /** TM Fifo underflowed */
   uint32_t   fifo_undeflow;
   /** TM Failure indicates that the tm has forced the upstream logic to flush data buffers and resynchronize to a new frame boundary */
   uint32_t   tm_fail;
} Iqn2Fl_AilEeRtTm0;

/** @brief This object contains the AIL EE CI CO 0 ERR information */
typedef struct
{
   /** CI AXC Group LUT requested more containers transferred in a basic frame than can fit */
   uint32_t   ci_tbltoolong;
   /** CO AXC Group LUT requested more containers transferred in a basic frame than can fit */
   uint32_t   co_tbltoolong;
} Iqn2Fl_AilEeCiCo0;

/** @brief This object contains the AIL EE PD 0 ERR information */
typedef struct
{
   /** OBSAI timestamp failed timing window test. Wrap of the PD_Frame Counters did not
    * predict a Radio Frame Boundary consistent withTS=0 falling within the reception timing
    * window */
   uint32_t   pd_ee_obsai_frm_win_err;
   /** CPRI Received a second SOP without an EOP in between for 4B/5B. OBSAI Was in middle of
    * packet but recieved TS violated protocol. Packet forced to EOP and ERROR (PD_Frame) */
   uint32_t   pd_ee_sop_err;
   /** OBSAI, extra, single missing or double missing timestamp. (If thisis the only failure, AxC
    * will recover. If no recovery, pd_ee_obsai_axc_fail_err will fire next) */
   uint32_t   pd_ee_obsai_ts_miss_err;
   /** OBSAI, watch dog timed out before chan received OBSAI msg */
   uint32_t   pd_ee_obsai_ts_wdog_err;
   /** OBSAI, unrecoverable ts or wdog error, chan restart. Caused by missing or extra OBSAI
    * messages. (OBSAI time stamp prediction failure) */
   uint32_t   pd_ee_obsai_axc_fail_err;
   /** OBSAI, ctrl packet crc failure, pkt marked as bad */
   uint32_t   pd_ee_obsai_crc_err;
   /** OBSAI, received a remote RP3-01 message controlling the entire SOC to perform a HW
    * reset. AIL port will strobe, Must be enabled athigher layers of SOC to perform actual RESET
    * (similar function exists for CPRI, but controlled via PHY_RM) */
   uint32_t   pd_ee_rp3_01_soc_rst_info;
   /** OBSAI, a received message failed to match entries for addr and type in the lut table. */
   uint32_t   pd_ee_obsai_route_fail_info;
   /** OBSAI, a remote rp3-01 message was received as determined by the lut table */
   uint32_t   pd_ee_rp3_01_capture_info;
   /** OBSAI, a remote rp3-01 message was received but the CRC failed for the message */
   uint32_t   pd_ee_rp3_01_crc_fail_err;
   /** OBSAI, a gsm off strobe was received for a channe */
   uint32_t   pd_ee_obsai_gsm_off_stb_info;
   /** When CPRI CW CRC checks are enabled, a CRC was received which did not match the
    * calculated CRC. CRC failure could be caused by SERDES bit error -or- bad programming at
    * either sender or receiver */
   uint32_t   pd_ee_cpri_cw_crc_err;
   /** The CPRI CW Fifo for the channel has overflowed, indicating thatit was not serviced fast
    * enough */
   uint32_t   pd_ee_cpri_cw_ovfl_err;
   /** The CPRI CW 4B5B logic detected an EOP while not currently processing a packet */
   uint32_t   pd_ee_cpri_cw_4b5b_eop_err;
   /** The CPRI CW4B5B logic detected an illegal bit pattern. It could not determine what the
    * correct character was */
   uint32_t   pd_ee_cpri_cw_4b5b_char_err;
   /** OBSAI, on any channel, an sop occurred from the PD */
   uint32_t   pd_ee_obsai_sop_info;
   /** OBSAI, on any channel, an eop occurred from the PD */
   uint32_t   pd_ee_obsai_eop_info;
   /** OBSAI, on any channel, an sof occurred from the PD */
   uint32_t   pd_ee_obsai_sof_info;
} Iqn2Fl_AilEePd0;

/** @brief This object contains the AIL EE PD 1 ERR information */
typedef struct
{
   /** The bubble FSM did not wrap back to zero on a radio frame boundary. This is likely a
    * programming error. In GSM, a radio frame boundary is 60ms */
   uint32_t   pd_ee_cpri_bub_fsm_err;
   /** CPRI Channel TDM did not wrap backto start position prior to end of an iteration window
    * (few GSM cases will legally cause this EE) Mostly likely cause is incorrect programming of
    * TDM and Bubble insertion parameters */
   uint32_t   pd_ee_cpri_tdm_fsm_err;
   /** The radio standard framing FSM did not wrap naturally back to zero on a 10ms boundary
    * (indicated by hyperframe and basic frame offset relative to PHY frame boundary). For GSM
    * or some WiMax radio standards without 10msframing, this error should be ignored */
   uint32_t   pd_ee_cpri_radstd_err;
} Iqn2Fl_AilEePd1;

/** @brief This object contains the AIL EE PE 0 ERR information */
typedef struct
{
   /** Indicates there was no data available for transfer while in an active packet state (cpri_cw_null) */
   uint32_t   pe_ee_cpri_cw_null_starve_err;
   /** Indicates there was no data available for transfer while in an active packet state (cpri_cw_4b5b) */
   uint32_t   pe_ee_cpri_cw_4b5b_starve_err;
   /** Indicates there was no data available for transfer while in an active packet state (cpri_cw_hypfm) */
   uint32_t   pe_ee_cpri_cw_hypfm_starve_err;
   /** Indicates there was no data available for transfer while in an active packet state (cpri_cw_hdlc) */
   uint32_t   pe_ee_cpri_cw_hdlc_starve_err;
   /** Indicates a Hyperframe packet did not complete before the hyperframe boundary */
   uint32_t   pe_ee_cpri_cw_hypfm_oflow_err;
   /** The PE-to-RT FIFO overflowed. Suspected cause is PE_STB from uAT is too early or RT has
    * been programmed to disregard PE traffic */
   uint32_t   pe_ee_ofifo_oflow_err;
} Iqn2Fl_AilEePe0;

/** @brief This object contains the AIL EE SI 0 ERR information */
typedef struct
{
   /** UAT OBSAI PI error. PHY_RM PHY frame boundary detected outside programmed window
    * specified by uat_pimax_cfg_pi_max_cfg (OBSAI defines Pi as reception time of PHY FB) */
   uint32_t   uat_pi_err;
   /** CPRI Channel TDM did not wrap back to start positionprior to end of an iteration window
    * (few GSM cases will legally cause this EE) Mostly likely cause is incorrect programming of
    * TDM and Bubble insertion parameters. */
   uint32_t   cpri_tdm_lut_err;
   /** The bubble FSM did not wrap back to zero on a radio frame boundary. This is likely a
    * programming error. In GSM, a radio frame boundary is 60ms. */
   uint32_t   cpri_bub_fsm_err;
   /** The uAT indicated to the PE_SCH ofan OBSAI PHY frame boundary, but OBSAI FSM did not
    * predict a PHY frame boundary. Likely indicates a programming error of OBSAI PHY FSM.
    * Also will trigger if uAT timing is changed while PE_SCH OBSAI PHY FSM is currently
    * operational */
   uint32_t   obsai_phy_sync_err;
   /** Multiple OBSAI rules fired simultaniously. Indicates a programming error of OBSAI rules */
   uint32_t   obsai_multrulefire_err;
   /** An OBSAI Transmission Rule (MOD orDBM) did not wrap back naturally on a PHY frame
    * boundary. Likely a programming error of OBSAI rules, possibly rules rates which are
    * incompatible with link rate */
   uint32_t   obsai_dbm_wrap_err;
} Iqn2Fl_AilEeSi0;

/** @brief This object contains the DIO2 EE DMA ING A ERR information */
typedef struct
{
   /** Ingress DMA engine(s) error i.e. the trigger pending queue overflowed */
   uint32_t   dma_ing_pend_que_ovf_err;
   /** Ingress DMA Programming error detected per DMA engine */
   uint32_t   dma_ing_prog_err;
   /** Ingress DMA block transfer complete detected per DMA engine */
   uint32_t   dma_ing_xfer_done_info;
} Iqn2Fl_Dio2EeDmaIngA;

/** @brief This object contains the DIO2 EE DMA ING B ERR information */
typedef struct
{
   /** Ingress Data Buffer overflow per channel */
   uint32_t   dma_ing_buf_ovf_err;
} Iqn2Fl_Dio2EeDmaIngB;

/** @brief This object contains the DIO2 EE DMA EGR A ERR information */
typedef struct
{
   /** Egress DMA engine(s) error i.e. the trigger pending queue overflowed */
   uint32_t   dma_egr_pend_que_ovf_err;
   /** Egress DMA Programming error detected per DMA engine */
   uint32_t   dma_egr_prog_err;
   /** Egress DMA block transfer complete detected per DMA engine */
   uint32_t   dma_egr_xfer_done_info;
   /** PSI Staging FIFO Full i.e. back pressuring the egress data path including the VBUSM */
   uint32_t   si_ing_iq_fifo_full_info;
} Iqn2Fl_Dio2EeDmaEgrA;

/** @brief This object contains the DIO2 EE DT A ERR information */
typedef struct
{
   /** Data Trace buffer overflow error */
   uint32_t   dt_buff_ovf_err;
   /** Data Trace one shot capture done info */
   uint32_t   dt_done_info;
} Iqn2Fl_Dio2EeDtA;

/** @brief This object contains the DIO2 EE SII A ERR information */
typedef struct
{
   /** SI Ingress IQ ICC Start of Frame received */
   uint32_t   si_ing_iq_icc_sof_info;
   /** SI Ingress IQ ICC data transfer received */
   uint32_t   si_ing_iq_icc_dat_info;
   /** SI Ingress IQ IFE transmitted SOP */
   uint32_t   si_ing_iq_ife_sop_info;
   /** SI Ingress IQ IFE transmitted EOP */
   uint32_t   si_ing_iq_ife_eop_info;
   /** SI Ingress IQIFE transmitted valid data phase */
   uint32_t   si_ing_iq_ife_dat_info;
} Iqn2Fl_Dio2EeSiiA;

/** @brief This object contains the DIO2 EE SII C ERR information */
typedef struct
{
   /** SI Ingress IQ per channel Start of Frame alignment error */
   uint32_t   si_ing_iq_sof_err;
} Iqn2Fl_Dio2EeSiiC;

/** @brief This object contains the DIO2 EE SII E ERR information */
typedef struct
{
   /** SI Ingress IQ EOP transmitted to PSI */
   uint32_t   si_ing_iq_psi_eop_info;
   /** SI Ingress IQ data transmitted to PSI */
   uint32_t   si_ing_iq_psi_dat_info;
} Iqn2Fl_Dio2EeSiiE;

/** @brief This object contains the DIO2 EE SII G ERR information */
typedef struct
{
   /** SI Ingress IQ SOP transmitted to PSI */
   uint32_t   si_ing_iq_psi_sop_info;
} Iqn2Fl_Dio2EeSiiG;

/** @brief This object contains the DIO2 EE SIE A ERR information */
typedef struct
{
   /** SI Egress IQ EFE data starvation error */
   uint32_t   si_egr_iq_efe_starve_err;
   /** SI Egress IQ EFE packet boundary error */
   uint32_t   si_egr_iq_efe_pkt_err;
   /** SI Egress IQ EFE symbol number error. (Packet boundary errors due to a missing, early, or
    * late SOP disable the reporting of any symbol errors since the symbol number is only valid
    * for SOPs) */
   uint32_t   si_egr_iq_efe_sym_err;
   /** SI Egress IQ transmitted Start of Frame to ICC */
   uint32_t   si_egr_iq_icc_sof_info;
   /** SI Egress IQ transmitted data to ICC */
   uint32_t   si_egr_iq_icc_dat_info;
} Iqn2Fl_Dio2EeSieA;

/** @brief This object contains the DIO EE SIE D ERR information */
typedef struct
{
   /** SI Egress IQ PSI data type error */
   uint32_t   si_egr_iq_psi_data_type_err;
   /** SI Egress IQ EOP received from PSI */
   uint32_t   si_egr_iq_psi_eop_info;
   /** SI Egress IQ valid data received from PSI */
   uint32_t   si_egr_iq_psi_dat_info;
} Iqn2Fl_Dio2EeSieD;

/** @brief This object contains the DIO2 EE SIE F ERR information */
typedef struct
{
   /** SI Ingress IQ per channel SOP received from PSI */
   uint32_t   si_egr_iq_psi_sop_info;
} Iqn2Fl_Dio2EeSieF;


/** @brief This object contains the reference to the instance of IQN2 opened
 *  using the @a Iqn2Fl_open().
 *
 *  The pointer to this, is passed to all IQN2 FL APIs.
 */
typedef struct 
{
    /** This is a pointer to the registers of the IQN2 */
    Iqn2Fl_RegsOvly         regs;
   
    /** This is the instance of IQN2 being referred to by this object  */
    CSL_InstNum             perNum;

    /** This is the dedicated AIL instance number for HW control and get HW status argument*/
    Iqn2Fl_AilInstance      arg_ail;

    /** This is the dedicated EE register set for HW control and get HW status argument*/
    Iqn2Fl_EeArgIndex       arg_ee;

} Iqn2Fl_Obj;


/** @brief handle pointer to iqn2 object
 **/
typedef Iqn2Fl_Obj *Iqn2Fl_Handle;


/** @brief This will have the base-address information for the peripheral
 *  instance
 */
typedef struct 
{
    /** This is a pointer to the registers of the IQN2 */
    Iqn2Fl_RegsOvly   regs;

} Iqn2Fl_BaseAddress;



/** @brief Module specific parameters. 
 */
typedef struct
{
    /** For this module, this is not used as there is only one 
     *  module specific parameter. So user need not worry about this.
     */
    CSL_BitMask8   flags;
    
} Iqn2Fl_Param;



/** @brief This is the Setup structure for configuring IQN2 using @a Iqn2Fl_HwSetup()
 * function 
 */
typedef struct 
{
    /** Pointer to the global IQN2 setup structure */
/*    Iqn2Fl_GlobalSetup   *globalSetup; */

    /** Pointer specifying common setup structure */
/*    Iqn2Fl_CommonSetup   *commonSetup; */

    /** Pointer to the AIL (Antenna Interface Link) setup structure */
    Iqn2Fl_AilSetup   *ailSetup[IQN2FL_AIL_MAX];

    /** Pointer to the Top setup structure */
    Iqn2Fl_TopSetup   *topSetup;

    /** Pointer to the AID2 (Antenna Interface to DFE) setup structure */
    Iqn2Fl_Aid2Setup   *aid2Setup;

    /** Pointer to the IQS2 (IQN Switching Infrastructure) setup structure */
    Iqn2Fl_Iqs2Setup   *iqs2Setup;

    /** Pointer to the DIO2 (Direct I/O) setup structure */
    Iqn2Fl_Dio2Setup   *dio2Setup;

    /** Pointer to the AT2 (Antenna Interface Timer) setup structure */
    Iqn2Fl_At2Setup   *at2Setup;

} Iqn2Fl_Setup;


/** @brief Iqn2 context info is a pointer. 
 */
typedef void *Iqn2Fl_Context;

/**
 * @brief Specification of Iqn2Fl_DeviceCfgBaseAddr
 *
 * The Iqn2Fl_DeviceCfgBaseAddr is used to specify device level configuration
 * to the FL.
 */
typedef struct
{
  void *cfgBase;
} Iqn2Fl_DeviceCfgBaseAddr;

/**
 * @brief Specification of Iqn2Fl_DeviceCfg
 *
 * The Iqn2Fl_DeviceCfg is used to specify device level configuration
 * to the FL.
 */
#define iqn2_MAX_PERIPHS 1 /**< Maximum peripherals (base addresses) supported by FL */
typedef struct
{
  Iqn2Fl_DeviceCfgBaseAddr bases[iqn2_MAX_PERIPHS]; /**< base addreses */
} Iqn2Fl_DeviceCfg;

/**
 * @brief Specification of Iqn2Fl_InitCfg
 *
 * The Iqn2Fl_InitCfg is used to specify per-core
 * configuration to the FL.  It is used with @ref Iqn2Fl_open ()
 * once per core.
 */
typedef struct
{
  Iqn2Fl_DeviceCfg dev; /**< Device Configuration */
} Iqn2Fl_InitCfg;

/**
@} */

/**************************************************************************\
* IQN2FL global function declarations
\**************************************************************************/

/** 
 * @ingroup IQN2FL_FUNCTION
 * @brief Peripheral specific initialization function.
 *
 * This is the peripheral specific intialization function. This function is
 * idempotent in that calling it many times is same as calling it once.
 * This function initializes the FL data structures, and doesn't configure
 * hardware.
 *
 * <b> Usage Constraints: </b>
 * This function should be called before using any of the FL APIs in the IQN2
 * module.
 * 
 * @b Example:
 * @verbatim
 * ...
 * Iqn2Fl_Context Iqn2Context
 * 
    if (IQN2FL_SOK != Iqn2Fl_init(&Iqn2Context) {
        return;
    }
    @endverbatim
 *
 *
 * @return returns the status of the operation
 *
 * @{ */

Iqn2Fl_Status Iqn2Fl_init(
    /** IQN2 specific context information
     */
    Iqn2Fl_Context *pContext
    );


/** 
 *   @ingroup IQN2FL_FUNCTION
 *   @brief Opens the instance of iqn2.
 *
 *   @b Description
 *   @n The open call sets up the data structures for the particular instance of
 *   iqn2 device. The device can be re-opened anytime after it has been normally
 *   closed if so required. The handle returned by this call is input as an
 *   essential argument for rest of the APIs described for this module.
 *
 *   @b Arguments
 *   @verbatim            
           pIqn2LinkObj    Pointer to the object that holds reference to the
                           instance of iqn2 requested after the call
                           
           iqn2Num         Instance of iqn2 to which a handle is requested
           
           pIqn2Param      Module specific parameters
 
           pStatus         pointer for returning status of the function call

     @endverbatim
 *
 *   @return  
 *      Iqn2Fl_Handle
 *        Valid iqn2 instance handle will be returned if status value is
 *        equal to IQN2FL_SOK.
 *
 * @{ */
Iqn2Fl_Handle Iqn2Fl_open (
    /** Pointer to the object that holds reference to the
     *  instance of IQN2 requested after the call
     */
    Iqn2Fl_Obj         *pIqn2Obj,
    /** Instance of IQN2 to which a handle is requested
     */
    CSL_InstNum         iqn2Num,
    /** Device specific configuration used to pass configuration register base address
     */
    Iqn2Fl_InitCfg     *pIqn2Cfg,
    /** This returns the status (success/errors) of the call.
     * Could be 'NULL' if the user does not want status information.
     */
    Iqn2Fl_Status      *pStatus
    );


/**
 * @ingroup IQN2FL_FUNCTION
 * @brief This function initializes the device registers. 
 * This function initializes the device registers with
 * the appropriate values provided through the HwSetup Data structure.
 *
 * <b> Usage Constraints:</b>
 * Both @a Iqn2Fl_init() and @a Iqn2Fl_open() must be called successfully
 * in that order before Iqn2Fl_hwSetup() can be called. The user has to
 * allocate space for & fill in the main setup structure appropriately before
 * calling this function
 *
 * @b Example:
 * @verbatim
    CSL_IqnHandle hIqn2;
    Iqn2Fl_Status status;
    Iqn2Fl_HwSetup  Iqn2Setup;
    Iqn2Fl_HwGlobalStup  globalSetup = {?;
    Iqn2Fl_HwCommonLinkSetup commonSetup = {?;
    Iqn2Fl_HwLinkSetup linkSetup[0~5] = {?;
    
    Iqn2Setup.globalSetup   = &globalSetup;
    Iqn2Setup.commonSetup   = &commonSetup;
    Iqn2Setup.linkSetup  = &linkSetup[0~5] ;
    
    status = Iqn2Fl_hwSetup (hIqn2, &Iqn2Setup);
   @endverbatim
 *
 * @return Returns the status of the setup operation (see @a Iqn2Fl_Status)
 * Status is:
 * IQN2FL_SOK - successful completion of the setup
 * IQN2FL_INVPARAMS - hwSetup structure is not initialized.
 * @{ */

Iqn2Fl_Status  Iqn2Fl_hwSetup(
    /** Pointer to the object that holds reference to the
     *  instance of IQN2 requested after the call
    */
    Iqn2Fl_Handle             hIqn2,
    /** Pointer to setup structure which contains the
     *  information to program IQN2 to a useful state
    */
    Iqn2Fl_Setup             *iqn2Setup
);

/**
  * @ingroup IQN2FL_FUNCTION
  * @brief Controls IQN2 operation based on the control command
  */

Iqn2Fl_Status  Iqn2Fl_hwControl(
    Iqn2Fl_Handle                      hIqn2,
    Iqn2Fl_HwControlCmd                cmd,
    void                               *arg
);

/**
  * @ingroup IQN2FL_FUNCTION
  * @brief Gets IQN2 status values based on the status query command
  */

Iqn2Fl_Status  Iqn2Fl_getHwStatus(
   Iqn2Fl_Handle                      hIqn2,
   Iqn2Fl_HwStatusQuery               query,
   void                               *response
);

/**
  * @ingroup IQN2FL_FUNCTION
  * @brief Closing an instance of iqn2
  */

Iqn2Fl_Status  Iqn2Fl_close(
    /** Pointer to the object that holds reference to the
     *  instance of iqn2 requested after the Iqn2Fl_open(...) call
    */
    Iqn2Fl_Handle        hIqn2
);


/**
@} */


#ifdef __cplusplus
}
#endif

#endif /* _IQN2FL_H_ */
