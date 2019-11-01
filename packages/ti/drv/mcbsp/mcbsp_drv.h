/*
 * mcbsp_drv.h
 *
 * This file contains Application programming interface for the Mcbsp driver and
 * command/macro definitions used by the Mcbsp driver.
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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

/**
 * \file        mcbsp_drv.h
 *
 * \brief       McBSP driver interface definition file
 *
 *              This file contains the interfaces, data types and symbolic
 *              definitions that are needed by the application to utilize the
 *              services of the McBSP device driver.
 *
 *              (C) Copyright 2012, Texas Instruments, Inc
 *
 */

#ifndef _MCBSP_DRV_H_
#define _MCBSP_DRV_H_

/*============================================================================*/
/*                            INCLUDE FILES                                   */
/*============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_mcbsp.h>
#include "mcbspver.h"

/*============================================================================*/
/*                         MACRO TYPES                                        */
/*============================================================================*/

/**
 * \def    MCBSP_LOOPJOB_ENABLE
 *         To enable loop job mode support in the MCBSP driver, uncomment 
 *         following compiler switch and rebuild the MCBSP LLD.
 */
/* #define MCBSP_LOOPJOB_ENABLE */

/**
 * \def    Mcbsp_POLLED_RETRYCOUNT
 *         This macro specifies the default retry count to be used by the Mcbsp
 *         driver when polling for any hardware bit to be set or reset.
 *
 * \note   This default value used by the driver can be changed by an IOCTL
 *         "Mcbsp_Ioctl_SET_TIMEOUT".
 */
#define Mcbsp_POLLED_RETRYCOUNT   (0xFFFFu)

/**
 * \def    MCBSP_CACHE_LENGTH
 *         Defines the Maximum length of cache line that is possible.
 */
#define MCBSP_CACHE_LENGTH        (128u)

/**
 * \def    MCBSP_MAX_CACHE_ALIGN
 *         Defines the Maximum cache line size for alignment.
 */
#define MCBSP_MAX_CACHE_ALIGN        (128u)

/**
 * \def    MCBSP_COMPILE_TIME_SIZE_CHECK
 *         This macro generates compiler error if postulate is false, so 
 *         allows 0 overhead compile time size check.  This "works" when
 *         the expression contains sizeof() which otherwise doesn't work
 *         with preprocessor.
 */
#define MCBSP_COMPILE_TIME_SIZE_CHECK(postulate)                              \
   do {                                                                       \
       typedef struct {                                                       \
         uint8_t McbspNegativeSizeIfPostulateFalse[((int)(postulate))*2 - 1]; \
       } McbspPostulateCheck_t;                                               \
   }                                                                          \
   while (0)

/*============================================================================*/
/*                         ENUMERATED DATA TYPES                              */
/*============================================================================*/

/**
 *  \brief  Mcbsp device operational mode.
 *
 *          This enum is used to define the operational mode of the mcbsp device
 *          like normal mcbsp device.
 */
typedef enum Mcbsp_DevMode_t
{
    Mcbsp_DevMode_McBSP      = (0u)
    /**< Option to operate in normal McBSP mode     */

}Mcbsp_DevMode;
/**< Mcbsp device operational mode.       */

/**
 *  \brief  Mcbsp driver operational mode
 *
 *          Enumeration of the different modes of operation available for the
 *          Mcbsp device driver.(Mcbsp driver supports only EDMA mode).
 */
typedef enum Mcbsp_OpMode_t
{
    Mcbsp_OpMode_POLLED = 0,
    /**< Polled Mode                     */

    Mcbsp_OpMode_INTERRUPT,
    /**< Interrupt Mode                  */

    Mcbsp_OpMode_DMAINTERRUPT
    /**< DMA Mode                        */

}Mcbsp_OpMode;
/**< Mcbsp driver operational mode       */

/**
 * \brief   Mcbsp Emulation mode settings
 *
 *          Use this symbol to set the Emulation Mode to Free mode or any other 
 *          mode.
 *
 * \note    The SOFT mode settings are applicable only is the FREE mode is
 *          disabled.Hence if the soft mode is selected (both enable or disable)
 *          the driver will internally disable the FREE mode.
 */
typedef enum Mcbsp_EmuMode_t
{
    Mcbsp_EmuMode_SOFT_ABORT  = (0u),
    /**< SOFT mode is disabled.          */

    Mcbsp_EmuMode_SOFT_STOP   = (1u),
    /**< SOFT mode is enabled            */

    Mcbsp_EmuMode_FREE        = (2u)
    /**< FREE mode is enabled            */

}Mcbsp_EmuMode;
/**< Mcbsp emulation modes               */

/**
 *  \brief  McBSP digital loopback mode selection
 *
 *          Enum to select the enable/disable of Loopback mode of the mcbsp.
 */
typedef enum Mcbsp_Loopback_t
{
    Mcbsp_Loopback_DISABLE = 0,
    /**< loopback mode off                 */

    Mcbsp_Loopback_ENABLE = 1
    /**< loopback mode on                  */

} Mcbsp_Loopback;
/**< McBSP digital loopback mode selection */

/**
 * \brief   ENUM for receive data justification settings
 *
 *          Use this symbol for setting up RCV sign-extension and justification
 *          mode
 */
typedef enum Mcbsp_Rxjust_t
{
    Mcbsp_RxJust_RZF        = (0u),
    /**< RCV setting - right justify, fill MSBs with zeros                    */

    Mcbsp_RxJust_RSE        = (1u),
    /**< RCV setting - right justify, sign-extend the data into MSBs          */

    Mcbsp_RxJust_RxJUST_LZF = (2u)
    /**< RCV setting - left justify, fill LSBs with zeros                     */

}Mcbsp_Rxjust;
/**< Enumeration for the Receive data justification                           */

/**
 * \brief   Transmit pin mode configuration
 *
 *          DX pin settings used for setting the pin in high impedance mode.
 */
typedef enum Mcbsp_DxEna_t
{
    Mcbsp_DxEna_OFF  = (0),
    /**< DX enabler is OFF                       */

    Mcbsp_DxEna_ON   = (1u)
    /**< DX enabler is ON                        */
}Mcbsp_DxEna;
/**< DX pin High impedance enable/disable option */

/**
 * \brief Polarity of the CLKS pin
 *
 *        Enum to Set the polarity of the CLKS pin used to generate Frame Sync
 *        and bit clock
 */
typedef enum Mcbsp_ClkSPol_t
{
    Mcbsp_ClkSPol_RISING_EDGE   = 0,
    /**< Rising edge of CLKS generates CLKG and FSG.                          */

    Mcbsp_ClkSPol_FALLING_EDGE  = 1
    /**< Falling edge of CLKS generates CLKG and FSG.                         */

}Mcbsp_ClkSPol;
/**< Polarity of the CLKS pin                                                 */

/**
 * \brief   SRG clock source
 *
 *          Use this symbol to select input clock source for Sample Rate
 *          Generator
 */
typedef enum Mcbsp_SrgClk_t
{
    Mcbsp_SrgClk_CLKS   = (0u),
    /**< input clock source for Sample Rate Generator is CLKS pin             */

    Mcbsp_SrgClk_CLKCPU = (1u),
    /**< input clock source for Sample Rate Generator is CPU                  */

    Mcbsp_SrgClk_CLKR   = (2u),
    /**< input clock source for Sample Rate Generator is BCLKR pin            */

    Mcbsp_SrgClk_CLKX   = (3u)
    /**< input clock source for Sample Rate Generator is BCLKX pin            */

}Mcbsp_SrgClk;
/**< SRG clock source selection                                               */

/**
 * \brief Enum to specify the supported buffer formats.
 *
 * Interleaved and non-interleaved is standard format, this enumeration
 * captures the standard and custom data formats.
 */
typedef enum Mcbsp_BufferFormat_t
{
    /* This mode is used for buffer containing the data in continous      *
     * memory locations where each sample is  "wordWidth" long. This      *
     * format is supported for following configurations                   *
     * Note : interleaved and non interleaved does not apply for this     *
     * buffer format                                                      */
    Mcbsp_BufferFormat_1SLOT,

    /* This is used for transfer of data with multiple slots.please note  *
     * that the slot data is not interleaved in this format.              */
    Mcbsp_BufferFormat_MULTISLOT_NON_INTERLEAVED,

    /* This is used for transfer of data for multiple slots.please note   *
     * that the slot data is interleaved in this format.                  */
    Mcbsp_BufferFormat_MULTISLOT_INTERLEAVED
}Mcbsp_BufferFormat;
/**< Mcbsp supported buffer formats                                       */

/**
 * \brief Mcbsp PHASE control enums
 *
 * \note These enums are used to control the Phase settings of the MCBSP.
 */
typedef enum Mcbsp_Phase_t
{
    Mcbsp_Phase_SINGLE = (0u),
    /**< Single phase for frame                */

    Mcbsp_Phase_DUAL   = (1u)
    /**< Dual phase for frame                  */
} Mcbsp_Phase;
/**< Mcbsp frame phase selection ENUMERATION   */

/**
 *  \brief  Mcbsp supported wordlength.
 *
 *          Enumerated constant for wordlength supported by the MCBSP device
 */
typedef enum Mcbsp_WordLength_t
{
    Mcbsp_WordLength_8  = 8u,
    /**< Word length of 8 bits            */

    Mcbsp_WordLength_12 = 12u,
    /**< Word length of 12 bits           */

    Mcbsp_WordLength_16 = 16u,
    /**< Word length of 16 bits           */

    Mcbsp_WordLength_20 = 20u,
    /**< Word length of 20 bits           */

    Mcbsp_WordLength_24 = 24u,
    /**< Word length of 24 bits           */

    Mcbsp_WordLength_32 = 32u
    /**< Word length of 32 bits           */

}Mcbsp_WordLength;
/**< Mcbsp supported wordlength.          */

/**
 * \brief   Frame sync ignore status enum
 *
 * \note    Use this symbol to detect or ignore frame synchronisation
 */
typedef enum Mcbsp_FrmSync_t
{
    Mcbsp_FrmSync_DETECT  = (0u),
    /**< detect frame synchronisation          */

    Mcbsp_FrmSync_IGNORE  = (1u)
    /**< ignore frame synchronisation          */

}Mcbsp_FrmSync;
/**< Frame sync detection options ENUMERATION  */

/**
 * \brief   Enum to select Data delay in bits
 *
 *          Use this Enum to set XMT/RCV Data Delay (in bits)
 */
typedef enum Mcbsp_DataDelay_t
{
    Mcbsp_DataDelay_0_BIT = (0u),
    /**< sets XMT/RCV Data Delay to 0 bits     */

    Mcbsp_DataDelay_1_BIT = (1u),
    /**< sets XMT/RCV Data Delay to 1 bits     */

    Mcbsp_DataDelay_2_BIT = (2u)
    /**< sets XMT/RCV Data Delay to 2 bits     */

} Mcbsp_DataDelay;
/**< Enum to select Data delay in bits         */

/**
 * \brief   Enum to select the companding law
 *
 *          Use this Enum to select the appropriate companding rule applicable.
 */
typedef enum Mcbsp_Compand_t
{
    Mcbsp_Compand_OFF_MSB_FIRST =  (0u),
    /**< No companding, data transfer starts with MSB first.                  */

    Mcbsp_Compand_OFF_LSB_FIRST =  (1u),
    /**< No companding, 8-bit data transfer starts with LSB first.            */

    Mcbsp_Compand_MULAW         =  (2u),
    /**< mu-law comapanding enable for channel                                */

    Mcbsp_Compand_ALAW          =  (3u)
    /**< A-law comapanding enable for channel                                 */

}Mcbsp_Compand;
/**< Enum to select the companding law         */

/**
 * \brief   McBSP 32-bit reversal feature
 *
 *          This ENUM allows the user to select the option of BIT reversal.
 */
typedef enum Mcbsp_BitReversal_t
{
    Mcbsp_BitReversal_DISABLE   = (0u),
    /**< 32-bit reversal disabled                                             */

    Mcbsp_BitReversal_ENABLE    = (1u)
    /**< 32-bit reversal enabled. 32-bit data is received LSB first. Word     *
     * length should be set for 32-bit operation; else operation undefined    */

}Mcbsp_BitReversal;
/**< McBSP 32-bit reversal feature                                            */

/**
 * \brief   Frame sync clock source
 *
 *          Use this ENUM to set the frame sync clock source as internal or
 *          external
 */
typedef enum Mcbsp_FsClkMode_t
{
    Mcbsp_FsClkMode_EXTERNAL = (0u),
    /**< frame sync clock source as internal     */

    Mcbsp_FsClkMode_INTERNAL = (1u),
    /**< frame sync clock source as external     */

    Mcbsp_FsClkMode_DXR_XSR  = (2u)
    /**< frame sync is generated on DXR_XSR copy */
    
} Mcbsp_FsClkMode;
/**< Frame sync clock source                     */

/**
 * \brief   Clock source selection ENUM
 *
 *          Use this symbol to set the clock source as internal or external
 */
typedef enum Mcbsp_TxRxClkMode_t
{
    Mcbsp_TxRxClkMode_EXTERNAL = (0u),
    /**< clock source as internal */

    Mcbsp_TxRxClkMode_INTERNAL = (1u)
    /**< clock source as external */

} Mcbsp_TxRxClkMode;
/**< Clock source selection ENUM  */

/**
 * \brief   Clock polarity
 *
 *          Use this symbol to set XMT or RCV clock polarity as rising or
 *          falling edge
 */
typedef enum Mcbsp_ClkPol_t
{
    Mcbsp_ClkPol_RISING_EDGE  = (0u),
    /**< Data sampled on rising edge of the bit Clock                         */

    Mcbsp_ClkPol_FALLING_EDGE = (1u),
    /**< Data sampled on falling edge of the bit Clock                        */

    Mcbsp_ClkPol_SRG_RISING_EDGE = (0u),
    /**< SRG clock polarity is rising edge      */

    Mcbsp_ClkPol_SRG_FALLING_EDGE = (1u)
   /**< SRG clock polarity Is falling edge      */

} Mcbsp_ClkPol;
/**< Clock polarity                             */

/**
 * \brief   Frame sync polarity
 *
 *          Use this symbol to set frame sync polarity as active-high or
 *          active-low
 */
typedef enum Mcbsp_FsPol_t
{
    Mcbsp_FsPol_ACTIVE_HIGH   = (0u),
    /**< frame sync polarity is active-high */

    Mcbsp_FsPol_ACTIVE_LOW    = (1u)
    /**< frame sync polarity is active-low  */

}Mcbsp_FsPol;
/**< Frame sync polarity                                                      */

/**
 * \brief   MCBSP Interrupt mode
 *
 *          Use this symbol to set Interrupt mode (i.e. source of interrupt
 *          generation).This symbol is used on both RCV and XMT for RINT and
 *          XINT generation mode.
 */
typedef enum Mcbsp_IntMode_t
{
    Mcbsp_IntMode_ON_READY   = (0u),
    /**< Interrupt generated on RRDY of RCV or XRDY of XMT                    */

    Mcbsp_IntMode_ON_EOB     = (1u),
    /**< Interrupt generated on end of 16-channel block transfer in           *
     *   multichannel mode                                                    */

    Mcbsp_IntMode_ON_FSYNC   = (2u),
    /**< Interrupt generated on frame sync                                    */

    Mcbsp_IntMode_ON_SYNCERR = (3u)
    /**< Interrupt generated on synchronisation error                         */

}Mcbsp_IntMode;
/**< MCBSP Interrupt mode                                                     */

/**
 * \brief    Transmit multichannel selection mode bit.
 *            
 *           MCM determines whether all channels or only selected channels are 
 *           enabled and unmasked for transmission/reception.
 *
 */
typedef enum Mcbsp_McmMode_t
{
    Mcbsp_McmMode_ALL_CHAN_ENABLED_UNMASKED  = (0u),
    /**< All the 128 channels are enabled                                     */

    Mcbsp_McmMode_ALL_CHAN_DISABLED_UNMASKED = (1u),
    /**< All channels are disabled unless selected by enable mask             */

    Mcbsp_McmMode_ALL_CHAN_ENABLED_MASKED    = (2u),
    /**< All channels are enabled but masked unless selected by Enable mask   */

    Mcbsp_McmMode_ALL_CHAN_DISABLED_MASKED   = (3u)
    /**< Symmetric transmission and reception                                 */

}Mcbsp_McmMode;
/**< Transmit multichannel selection mode bit.                                */

/**
 * \brief      Partition A/B block channel selection bit.
 */
typedef enum Mcbsp_PartitionSel_t
{
    Mcbsp_PartitionMode_CHAN_0_15    = (0u),
    /**< Select the channel 0-15 in block A     */
    
    Mcbsp_PartitionMode_CHAN_16_31   = (0u),
    /**< Select the channel 16-31 in block B    */

    Mcbsp_PartitionMode_CHAN_32_47   = (1u),
    /**< Select the channel 32-47 in block A    */

    Mcbsp_PartitionMode_CHAN_48_63   = (1u),
    /**< Select the channel 48-63 in block B    */
    
    Mcbsp_PartitionMode_CHAN_64_79   = (2u),
    /**< Select the channel 64-79 in block A    */

    Mcbsp_PartitionMode_CHAN_80_95   = (2u),
    /**< Select the channel 80-95 in block B    */

    Mcbsp_PartitionMode_CHAN_96_111  = (3u),
    /**< Select the channel 96-111 in block A   */

    Mcbsp_PartitionMode_CHAN_112_127 = (3u)
    /**< Select the channel 112-127 in block B  */

}Mcbsp_PartitionSel;
/**< Partition A/B block selection bit. */

/**
 * \brief    Multichannel partition mode selection.
 *
 *           Multichannel mode allows selection of either an 2 partition mode or
 *           8 partition mode.This enum allows the user to select the required
 *           partition mode of choice.
 */
typedef enum Mcbsp_PartitionMode_t
{
    Mcbsp_PartitionMode_2 = (0u),
    /**< 2-partition mode. Only partitions A and B are used.      */
    
    Mcbsp_PartitionMode_8 = (1u)
    /**< 8-partition mode. All partitions (A through H) are used  */

}Mcbsp_PartitionMode;
/**< Multichannel partition mode selction                         */

/*============================================================================*/
/*                              DATA STRUCTURES                               */
/*============================================================================*/

/**
 *  \brief McBSP sample rate generator configuration structure
 *
 *         Configurations for the Sample rate  generator to generate the BCLK 
 *         and Frame Sync signals are specified using this structure.
 */
typedef struct Mcbsp_srgConfig_t
{
    Bool             gSync;
    /**< sample rate generator clock syncronization bit (only if CLKS is used)*/

    Mcbsp_ClkSPol    clksPolarity;
    /**< CLKS polarity used to drive the CLKG and FSG clocks                  */

    Mcbsp_SrgClk     srgInputClkMode;
    /**< Source for the Sample rate generator (CLKS,CPU,CLKX,CLKR)            */

    uint32_t         srgrInputFreq;
    /**< input clock frequency for the SRGR (freq of CLKS or CLKX etc..)      */

    uint32_t         srgFrmPulseWidth;
    /**< Set the Frame Sync Pulse width in terms of FSG clock                 */

} Mcbsp_srgConfig;
/**< McBSP sample rate generator configuration structure                      */

/**
 * \brief   Mcbsp TX/RX section configuration structure.
 *
 *          This strcuture specifies the configuration for the McBSP data stream
 *          including the whether it is single phase or dual phase, number of
 *          frames,the word length in each phase and data delay etc.
 */
typedef struct Mcbsp_DataConfig_t
{
    Mcbsp_Phase               phaseNum;
    /**< Phase of the McBSP data stream                                       */

    Mcbsp_WordLength          wrdLen1;
    /**< Length of the data word in first phase                               */

    Mcbsp_WordLength          wrdLen2;
    /**< Length of the data word in second when dual phase is selected        */

    uint32_t                  frmLen1;
    /**< Length of the data frame in first phase                              */

    uint32_t                  frmLen2;
    /**< Length of the data frame in second phase                             */

    Mcbsp_FrmSync             frmSyncIgn;
    /**< Frame sync ignore bit                                                */

    Mcbsp_DataDelay           dataDelay;
    /**< Data delay to be configured in number of Bits                        */

    Mcbsp_Compand             compandSel;
    /**< companding selection                                                 */

    Mcbsp_BitReversal         bitReversal;
    /**< Transmit 32-bit bit reversal feature enable bit.                     */

    Mcbsp_IntMode             intMode;
    /**< Event for which the interrupt is to be generated                     */

    Mcbsp_Rxjust              rjust;
    /**< Receive sign extension and justification settings (RX only setting)  */
    
    Mcbsp_DxEna               dxState;
    /**< High impedance enable/disbale bit for the DX pin (TX only setting)   */
    
}Mcbsp_DataConfig;
/**< Mcbsp TX/RX section configuration structure.                             */

/**
 * \brief   Mcbsp multi channel control settings
 *
 *          structure to configure the multi channel settings for the mcbsp.used
 *          when the multi channel configuration is to be enabled.
 */
typedef struct Mcbsp_McrSetup_t
{
    Mcbsp_McmMode          multiChanMode;
    /**< Multi channel mode to be selcted                                     */  

    Mcbsp_PartitionSel     partitionSelA;
    /**< Channel selection for partition A                                    */

    Mcbsp_PartitionSel     partitionSelB;
    /**< Channel selection for partition B                                    */

    Mcbsp_PartitionMode    partitionMode;
    /**< Channel partition mode selection                                     */
}Mcbsp_McrSetup;
/**< Mcbsp multi channel control settings                                     */

/**
 * \brief   Mcbsp clock settings setup structure.
 *
 *          This structure contains the information required to configure the 
 *          clocks for the Mcbsp.Both the frame sync settings and the bit clock
 *          settings can be configured in this structure.
 */
typedef struct Mcbsp_ClkSetup_t
{
    Mcbsp_FsClkMode        frmSyncMode;
    /**< Frame sync mode bit (FSXM/FSRM)(Internal/External)                   */

    uint32_t                 samplingRate;
    /**< Frame sync frequency                                                 */

    Mcbsp_TxRxClkMode      clkMode;
    /**< Bit clock mode (internal or external)                                */

    Mcbsp_FsPol            frmSyncPolarity;
    /**< frmSyncTxPolarity                                                    */

    Mcbsp_ClkPol           clkPolarity;
    /**< clkTxPolarity                                                        */
}Mcbsp_ClkSetup;
/**< Mcbsp clock settings setup structure                                     */

/**
 * \brief Mcbsp Hardware specific information Object
 *
 *  This structure maintains the information specific to the hardware instance
 *  of the Mcbsp. information like the base address and the cpu event numbers
 *  and DMA events is specific to the instance. This structure is a collection
 *  of such information.
 */
typedef struct 
{
    uint32_t               instNum;
    /**< Instance of MCBSP being referred by this object                      */

    CSL_McbspRegsOvly    regs;
    /**< Pointer to the register overlay structure of the MCBSP               */

    CSL_BfifoRegsOvly    fifoRegs;
    /**< Fifo address of the mcbsp instance                                   */

    CSL_BdataRegsOvly    dataAddress;
    /**< Mcbsp data registers address                                         */

    uint32_t               edmaTxEventNum;
    /**< edma Transmit event number                                           */

    uint32_t               edmaRxEventNum;
    /**< edma Receive event number                                            */

    uint32_t               cpuTxEventNum;
    /**< Transmit interrupt number                                            */

    uint32_t               cpuRxEventNum;
    /**< Receive interrupt number                                             */

}Mcbsp_HwInfo_Unpadded;

typedef struct
{
    /** Data structure without padding, so sizeof() can compute padding */
    Mcbsp_HwInfo_Unpadded obj;
    /** Pad out to end of MCBSP_MAX_CACHE_ALIGN bytes to prevent something else
     * from being placed on same cache line as Mcbsp_HwInfo. Note that pad[0]
     * is illegal, so must add full MCBSP_MAX_CACHE_ALIGN if structure is
     * already padded by chance. */
    uint8_t                 pad[MCBSP_MAX_CACHE_ALIGN - 
                            (sizeof(Mcbsp_HwInfo_Unpadded) % MCBSP_MAX_CACHE_ALIGN)];
} Mcbsp_HwInfo;
/**< Mcbsp Hardware specific information Object                               */

/**
 *  \brief  Loop job buffer structure.
 *          This structure defines the format of the Loop job buffer.
 *
 *  \note   Loop job buffer is a buffer used by the mcbsp in edma mode. The 
 *          Mcbsp uses this buffer when no iobufs are present and the mcbsp 
 *          is also not stopped. In this condition the Mcbsp utilises the 
 *          loop job buffer to transmit a known pattern of data from the loopjob
 *          buffer or receives the data in to a loop job buffer
 */
typedef struct Mcbsp_TempBuffer_t
{
    Uint8   scratchBuf[(4u) + MCBSP_CACHE_LENGTH];
    /**< This buffer will be aligned and also the same buffer will be used
     * for all slots and also only 4 bytes are required because the
     * max wordwidth is 4 and the cache length is used for buffer alignment   */
    uint32_t *scratchBuffer;
    /**< Pointer to hold the aligned buffer                                   */
}Mcbsp_TempBuffer;
/**< loop job buffer format.                                                  */

/*============================================================================*/
/*                               DATA TYPES                                   */
/*============================================================================*/

/**
 * \brief Global error callback function protype
 *
 *        This is the global error callback function for the McBSP driver.
 *        This function is called directly called from ISR context in case of
 *        error.
 * \note  Since this function is called from an ISR context,care should be taken
 *        that this function conforms to ISR coding guidelines.
 */
typedef void (*Mcbsp_GblErrCallback)(uint32_t Arg1,uint32_t Arg2,uint32_t Arg3);

/**
 *  \brief  McBSP channel setup params
 *
 *          This structure holds configuration to be used for
 *          creating a channel of the Mcbsp. These parameters need to be 
 *          specified during the creation of the channel.
 */
typedef struct Mcbsp_ChanParams_t
{
    uint32_t                  wordWidth;
    /**< This parameter informs the driver what is the width word (not        *
     * slot) and this help driver indirectly to decided no. of bytes to       *
     * be transfered for each slot- This is very                              *
     * important parameter - in case of invalid value default value           *
     * driver will assume is 32                                               */

    void*                     userLoopJobBuffer;
    /**< Buffer to be transferred when the loop job is running.               */

    uint16_t                  userLoopJobLength;
    /**< Number of bytes of the userloopjob buffer for each slot
     * Please note that this is no. of bytes and this should be
     * pre-calcuated properly for word width of slot - Please refer the
     * wordWidth of this structure                                            */

    Mcbsp_GblErrCallback    gblCbk;
    /**< callback required when global error occurs - must be callable        *
     * directly from the ISR context                                          */

    void*                     edmaHandle;
    /**< Handle to the EDMA Driver                                            */

    uint32_t                  edmaEventQue;
    /**< EDMA event queue to be used by the channel                           */

    uint32_t                  hwiNumber;
    /**< Variable to specify the Hwi number to be used by the driver          */

    Mcbsp_BufferFormat      dataFormat;
    /**< Format of the application supplied buffer                            */

    Bool                    enableHwFifo;
    /**< Option to enable/disable the Hardware FIFO                           */

    Mcbsp_DataConfig       *chanConfig;
    /**< settings to configure the TX or RX hardware sections                 */

    Mcbsp_ClkSetup         *clkSetup;
    /**< clock setup for the RX or the TX section                             */

    Mcbsp_McrSetup         *multiChanCtrl;
    /**< multiple channel control settings                                    */

    uint32_t                  chanEnableMask[4];
    /**< Mask of the channels to be enabled or disabled                       */

    uint32_t               numEnabledChannels;
    /**< Number of channels enabled with multichannel mode
     *   Note: This should match the number of bits in chanEnable Mask if in
     *         Mcbsp_McmMode_ALL_CHAN_DISABLED_UNMASKED  mode                 */

}Mcbsp_ChanParams;
/**< Mcbsp Channel setup parameters                                           */

/**
 *  \brief  McBSP device create params
 *
 *          This structure holds the configuration to be used for the
 *          creation of the Mcbsp device instance.This configuration need to be
 *          supplied during the creation of the device instance.
 */
typedef struct Mcbsp_Params_t
{
    Mcbsp_DevMode           mode;
    /**< Mode in which mcbsp instance needs to be created                     */

    Mcbsp_OpMode            opMode;
    /**< Operation mode of driver i.e. interrupt or EDMA.Mcbsp supports only
     *   EDMA mode                                                            */

    Bool                    enablecache;
    /**< whether the cache operations are to be performed on the application
     *   buffers or not                                                       */

    Mcbsp_EmuMode           emulationMode;
    /**< Emulation mode settings for the Mcbsp                                */

    Mcbsp_Loopback          dlbMode;
    /**< digital loop back mode (ENABLE/DISABLE)                              */

    Mcbsp_srgConfig        *srgSetup;
    /**< configuration for the sample rate generator configurations           */

    void*                   txQPendingList;
    /**< Queue to hold the pending buffers received from the application      */
  
    void*                   txQFloatingList;
    /**< Queue to manage floating buffers in DMA                              */

    void*                   rxQPendingList;
    /**< Queue to hold the pending buffers received from the application      */

    void*                   rxQFloatingList;
    /**< Queue to manage floating buffers in DMA                              */

} Mcbsp_Params;
/**< McBSP device setup params                                                */

/**
 *  \brief  McBSP queue element
 *
 *          Doubly linked list for queue elements. 
 *          Required for Mcbsp_IOBuf structure. 
 */
typedef struct Mcbsp_QueueElem_t {
    struct Mcbsp_QueueElem_t* volatile  next;
    struct Mcbsp_QueueElem_t* volatile  prev;
} Mcbsp_QueueElem;

/**
 *  \brief  McBSP frame object params
 *
 *          Mcbsp_IOBuf structures are managed by the driver. MCBSP 
 *          buffers are the basis for all I/O operations. 'cmd' field 
 *          contains the command id for the driver. 'status' is filled 
 *          in by the driver and contains the status of the commmand.
 */
typedef struct Mcbsp_IOBuf_t 
{
    Mcbsp_QueueElem     link;
    /**< queue link                */

    void*               addr;
    /**< buffer address            */

    uint32_t            size;   
    /**< buffer size               */

    uint32_t            arg;    
    /**< arg to be used by end app */

    uint32_t            cmd;    
    /**< command for driver - READ, WRITE, ABORT or FLUSH  */

    int32_t             status; 
    /**< status of command         */

    uint32_t            misc;   
    /**< reserved for driver       */
} Mcbsp_IOBuf;
/**< Mcbsp frame object params     */

/**
 *  \brief MCBSP driver callback function
 *
 *         This is the driver's callback function. The driver will
 *         call a function of this type whenever an I/O operation
 *         is over.
 */
typedef void (*Mcbsp_CallbackFxn)(void* arg, Mcbsp_IOBuf *ioBuf);

/*============================================================================*/
/*                           IOCTL COMMANDS                                   */
/*============================================================================*/

/**
 *  \brief  McBSP Ioctl commands
 *
 *          List of all the ioctl commands supported by the Mcbsp driver.
 *
 */
typedef enum Mcbsp_IOCTL_t
{
    Mcbsp_IOCTL_START = 128,
    /**< Starts the data transfer                                             */

    Mcbsp_IOCTL_SAMPLE_RATE_CHANGE,
    /**< Command to chnage the sample rate                                    */

    Mcbsp_IOCTL_STOP,
    /**< Stops the data transfer                                              */

    Mcbsp_IOCTL_SRGR_START,
    /**< Start the McBSP SRG                                                  */

    Mcbsp_IOCTL_SRGR_STOP,
    /**< Stop the McBSP SRG                                                   */

    Mcbsp_IOCTL_FSGR_START,
    /**< Start the McBSP FSG                                                  */

    Mcbsp_IOCTL_FSGR_STOP,
    /**< Stop the McBSP FSG                                                   */

    Mcbsp_IOCTL_SET_TIMEOUT,
    /**< Modify the timeout value in the driver                               */

    Mcbsp_IOCTL_MUTE_ON,
    /**< Mute ON the transfer                                                 */

    Mcbsp_IOCTL_MUTE_OFF,
    /**< Take out of Muted state                                              */

    Mcbsp_IOCTL_PAUSE,
    /**< Pause the playback operation                                         */

    Mcbsp_IOCTL_RESUME,
    /**< Resume the playback operation                                        */

    Mcbsp_IOCTL_CHAN_RESET,
    /**< Reset an I/O channel                                                 */

    Mcbsp_IOCTL_DEVICE_RESET,
    /**< Reset both input and output channel                                  */

    Mcbsp_IOCTL_SET_CLKMODE,
    /**< Set Bit clock mode for the McBSP                                     */

    Mcbsp_IOCTL_SET_FRMSYNCMODE,
    /**< Set Frame Sync mode for the McBSP                                    */

    Mcbsp_IOCTL_CONFIG_SRGR,
    /**< configure Sample Rate Generator                                      */

    Mcbsp_IOCTL_SET_BCLK_POL,
    /**< Set the CLKR or CLKX clock polarity                                  */

    Mcbsp_IOCTL_SET_FRMSYNC_POL,
    /**< Set the FSR and FSX polarity                                         */

    Mcbsp_IOCTL_MODIFY_LOOPJOB,
    /**< Enable/disable the loopjob                                           */

    Mcbsp_IOCTL_SYNCERR_INT_ENABLE,
    /**< Enable/disable the sync error                                        */

    Mcbsp_IOCTL_LOOPBACK,
    /**< enable/disable the loop back mode                                    */

    Mcbsp_IOCTL_CANCEL_PENDING_IO
    /**< Cancel all the current pending IO in the driver                      */

} Mcbsp_IOCTL;
/**< McBSP Ioctl commands list */

/*============================================================================*/
/*                           MCBSP MODES                                      */
/*============================================================================*/
/**
 *  \brief  McBSP Create Modes
 *
 *          Mcbsp driver create functions take a mode parameter.
 *
 */
/**
 * \def    MCBSP_MODE_INPUT
 *         Defines the mode as input.
 */
#define MCBSP_MODE_INPUT    0x0001
/**
 * \def    MCBSP_MODE_OUTPUT
 *         Defines the mode as output.
 */
#define MCBSP_MODE_OUTPUT   0x0002
/**
 * \def    MCBSP_MODE_INOUT
 *         Defines the mode as input and output.
 */
#define MCBSP_MODE_INOUT    (MCBSP_MODE_INPUT | MCBSP_MODE_OUTPUT)

/*============================================================================*/
/*                           MCBSP STATUS CODES                               */
/*============================================================================*/
/**
 *  \brief  McBSP Driver Status Codes
 *
 *          Mcbsp driver status codes.
 *
 */
/**
 * \def    MCBSP_STATUS_COMPLETED
 *         Defines the status as completed successfully.
 */
#define  MCBSP_STATUS_COMPLETED     0
/**
 * \def    MCBSP_STATUS_PENDING
 *         Defines the status as queued and pending.
 */
#define  MCBSP_STATUS_PENDING       1
/**
 * \def    MCBSP_STATUS_FLUSHED
 *         Defines the status as request flushed. Queued writes 
 *         will be completed w/ MCBSP_STATUS_COMPLETED. Queued 
 *         read requests return w/ MCBSP_STATUS_FLUSHED.
 */
#define  MCBSP_STATUS_FLUSHED      2 

/**
 * \def    MCBSP_STATUS_ABORTED
 *         Defines the status as request aborted. Non-completed
 *         read or write requests return w/ MCBSP_STATUS_ABORTED.
 */
#define  MCBSP_STATUS_ABORTED      3      


/*============================================================================*/
/*                           MCBSP DRIVER ERROR CODES                         */
/*============================================================================*/
/**
 *  \brief  McBSP Driver Error Codes
 *
 *          Mcbsp driver error codes.
 *
 */
/**
 * \def    MCBSP_ERR_BADIO
 *         Generic failure condition
 */
#define  MCBSP_ERR_BADIO        -1
/**
 * \def    MCBSP_ERR_TIMEOUT
 *         Timeout occurred
 */
#define  MCBSP_ERR_TIMEOUT      -2
/**
 * \def    MCBSP_ERR_NOIOBUFFERS
 *         No buffers available for I/O
 */
#define  MCBSP_ERR_NOIOBUFFERS  -3
/**
 * \def    MCBSP_ERR_FREE
 *         Unable to free resources
 */
#define  MCBSP_ERR_FREE         -4
/**
 * \def    MCBSP_ERR_ALLOC
 *         Unable to allocate resource
 */
#define  MCBSP_ERR_ALLOC        -5
/**
 * \def    MCBSP_ERR_ABORT
 *         I/O was aborted before completed
 */
#define  MCBSP_ERR_ABORT        -6
/**
 * \def    MCBSP_ERR_BADMODE
 *         Invalid device mode
 */
#define  MCBSP_ERR_BADMODE      -7
/**
 * \def    MCBSP_ERR_EOF
 *         End-of-File was encountered
 */
#define  MCBSP_ERR_EOF          -8
/**
 * \def    MCBSP_ERR_NOTIMPL
 *         Operation not implemented or supported
 */
#define  MCBSP_ERR_NOTIMPL      -9
/**
 * \def    MCBSP_ERR_BADARGS
 *         Invalid arguments specified
 */
#define  MCBSP_ERR_BADARGS      -10
/**
 * \def    MCBSP_ERR_TIMEOUTUNREC
 *         Unrecoverable timeout occurred
 */
#define  MCBSP_ERR_TIMEOUTUNREC -11
/**
 * \def    MCBSP_ERR_INUSE
 *         Device already in use
 */
#define  MCBSP_ERR_INUSE        -12

/**
 *  \brief  McBSP Driver Buffer Frame Command Codes
 *
 *          Mcbsp driver frame command codes.
 *
 */
/**
 * \def    Mcbsp_IOBuf_Cmd_READ
 *         Read command
 */
#define Mcbsp_IOBuf_Cmd_READ        0
/**
 * \def    Mcbsp_IOBuf_Cmd_WRITE
 *         Write command
 */
#define Mcbsp_IOBuf_Cmd_WRITE       1
/**
 * \def    Mcbsp_IOBuf_Cmd_ABORT
 *         Abort command
 */
#define Mcbsp_IOBuf_Cmd_ABORT       2
/**
 * \def    Mcbsp_IOBuf_Cmd_FLUSH
 *         Flush command
 */
#define Mcbsp_IOBuf_Cmd_FLUSH       3

/**
 *  \brief  McBSP Driver Command Codes Reserved for Control
 *
 *          Mcbsp driver command codes reserved for control.
 *
 */
/**
 * \def    MCBSP_CTRL_CHAN_RESET
 *         Reset channel only
 */
#define MCBSP_CTRL_CHAN_RESET          0
/**
 * \def    MCBSP_CTRL_CHAN_TIMEDOUT
 *         Channel timeout occured
 */
#define MCBSP_CTRL_CHAN_TIMEDOUT       1
/**
 * \def    MCBSP_CTRL_DEVICE_RESET
 *         Reset entire device
 */
#define MCBSP_CTRL_DEVICE_RESET        2

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

int32_t mcbspInit (void);
int32_t mcbspBindDev(void* *devp, int32_t devid, void* devParams);
int32_t mcbspUnBindDev(void* devp);
int32_t mcbspCreateChan(void* *chanp, void* devp, int32_t mode, 
                       void* chanParams, Mcbsp_CallbackFxn cbFxn,
                       void* cbArg);
int32_t mcbspDeleteChan(void* chanp);
int32_t mcbspSubmitChan(void* chanp, Mcbsp_IOBuf *const ioBuf);
int32_t mcbspControlChan(void* chanp, Mcbsp_IOCTL cmd, void* arg);
void mcbspGblXmtIsr(void *hChan);
void mcbspGblRcvIsr(void *hChan);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* _MCBSP_DRV_H_ */
/*============================================================================*/
/*                         END OF FILE                                        */
/*============================================================================*/
