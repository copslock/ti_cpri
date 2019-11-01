/********************************************************************
* Copyright (C) 2012-2013 Texas Instruments Incorporated.
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
 *  @defgroup DFE_FL_MISC_API MISC
 *  @ingroup DFE_FL_API
 */

/** @file dfe_fl_misc.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_MISC CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
/* =============================================================================
 * Revision History
 * ===============
 *
 *
 * =============================================================================
 */

/**
 * @defgroup DFE_FL_MISC_SYMBOL DFE Misc Symbols
 * @ingroup DFE_FL_MISC_API
 */

/**
 * @defgroup DFE_FL_MISC_DATASTRUCT DFE Misc Data Structures
 * @ingroup DFE_FL_MISC_API
 */

/**
 * @defgroup DFE_FL_MISC_ENUM DFE Misc Enumverated Data Types
 * @ingroup DFE_FL_MISC_API
 */

/**
 * @defgroup DFE_FL_MISC_FUNCTION DFE Misc Functions
 * @ingroup DFE_FL_MISC_API
 */

#ifndef _DFE_FL_MISC_H_
#define _DFE_FL_MISC_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl.h>
#include <ti/csl/cslr_dfe_misc.h>

/**
 * @addtogroup DFE_FL_MISC_SYMBOL
 * @{
 */

/** @brief mem_mpu_access
 */
/// flag to enable ARBiter memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_ARB          (0x8000u)
/// flag to enable Capture Buffer memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_CB           (0x4000u)
/// flag to enable FeedBack memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_FB           (0x2000u)
/// flag to enable Rx memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_RX           (0x1000u)
/// flag to enable JESD memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_JESD         (0x0800u)
/// flag to enable Tx memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_TX           (0x0400u)
/// flag to enable DPDA memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_DPDA         (0x0200u)
/// flag to enable DPD memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_DPD          (0x0100u)
/// flag to enable ACL memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_ACL          (0x0080u)
/// flag to enable CFR1 memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_CFR1         (0x0040u)
/// flag to enable CFR0 memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_CFR0         (0x0020u)
/// flag to enable DDUC3 memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_DDUC3        (0x0010u)
/// flag to enable DDUC2 memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_DDUC2        (0x0008u)
/// flag to enable DDUC1 memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_DDUC1        (0x0004u)
/// flag to enable DDCU0 memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_DDUC0        (0x0002u)
/// flag to enable BaseBand memory rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_BB           (0x0001u)
/// flag to enable all memories rw via MPU
#define DFE_FL_MEM_MPU_ACCESS_ALL          (0xffffu)

/** @brief one shot pulse width */
/// one shot but not change raw pulse width
#define DFE_FL_MISC_SYNC_ONE_SHORT_RAW     (0)
/// one shot with pulse width of n clocks
#define DFE_FL_MISC_SYNC_ONE_SHORT_N(n)    (n)

/** @brief  wait count for sync */
/// not wait for sync
#define DFE_FL_MISC_SYNC_NOWAIT            (0)
/// wait until sync has come
#define DFE_FL_MISC_SYNC_WAITFOREVER       (0xffffffffu)
/// wait n loops before give up if sync not come
#define DFE_FL_MISC_SYNC_WAIT(n)           (n)

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_MISC_ENUM
 * @{
 */

/** @brief control commands
 */
typedef enum
{
    ///init clock gate, state, and clear data
    DFE_FL_MISC_CMD_CFG_INITS,
    
    /// Set mem MPU access bitmask
    DFE_FL_MISC_CMD_SET_MEM_MPU_ACCESS,
    /// clear mem MPU access bitmask
    DFE_FL_MISC_CMD_CLR_MEM_MPU_ACCESS,
    
    /**
     * sync gen
     */
    /// issue a sync
    DFE_FL_MISC_CMD_ISSUE_SYNC,
    /// config one-shot pulse width
    DFE_FL_MISC_CMD_CFG_SYNC_ONE_SHOT,
    /// select UL frame strobe source
    DFE_FL_MISC_CMD_SET_UL_SIG_FSTROBE,
    /// select DL frame start source
    DFE_FL_MISC_CMD_SET_DL_SIG_FSTART,
    /// reset sync counter
    DFE_FL_MISC_CMD_RST_SYNC_CNTR,
    /// config sync counter
    DFE_FL_MISC_CMD_CFG_SYNC_CNTR,
    /// select sync counter sync source
    DFE_FL_MISC_CMD_SET_SYNC_CNTR_SSEL,
        
    /**
     * GPIO
     */
    /// Set GPIO pin mux
    DFE_FL_MISC_CMD_SET_GPIO_PIN_MUX,
    /// Select GPIO Sync Out sync source
    DFE_FL_MISC_CMD_SET_GPIO_SYNC_OUT_SSEL,
    /// write GPIO bank outputs
    DFE_FL_MISC_CMD_SET_GPIO_BANK,
    /// write GPIO pin output
    DFE_FL_MISC_CMD_SET_GPIO_PIN,
    
    /**
     * Arbiter
     */
    /// write command to arbiter
    DFE_FL_MISC_CMD_SET_ARBITER_CMD,
    /// write parameter to arbiter
    DFE_FL_MISC_CMD_SET_ARBITER_CMD_PAR,

    /**
     * interrupts
     */
    /// enable a master low priority interrupt
    DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR,
    /// disable a master low priority interrupt
    DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR,
    /// force generating a master low priority interrupt
    DFE_FL_MISC_CMD_SET_FORCE_MASTER_LOWPRI_INTR,    
    /// clear force generating a master low priority interrupt
    DFE_FL_MISC_CMD_CLR_FORCE_MASTER_LOWPRI_INTR,
    /// clear status of a master low priority interrupt
    DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS,    

    /// enable group of master low priority interrupts
    DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTRGRP,
    /// disable group of master low priority interrupts
    DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTRGRP,
    /// force generating group of master low priority interrupts
    DFE_FL_MISC_CMD_SET_FORCE_MASTER_LOWPRI_INTRGRP,    
    /// clear force generating group of master low priority interrupts
    DFE_FL_MISC_CMD_CLR_FORCE_MASTER_LOWPRI_INTRGRP,
    /// clear status of group of master low priority interrupts
    DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTRGRP_STATUS,    
    
    /// enable a master high priority interrupt
    DFE_FL_MISC_CMD_ENB_MASTER_HIPRI_INTR,
    /// disable a master high priority interrupt
    DFE_FL_MISC_CMD_DIS_MASTER_HIPRI_INTR,
    /// force generating a master high priority interrupt
    DFE_FL_MISC_CMD_SET_FORCE_MASTER_HIPRI_INTR,
    /// clear force generating a master high priority interrupt
    DFE_FL_MISC_CMD_CLR_FORCE_MASTER_HIPRI_INTR,
    /// clear status of a master high priority interrupt
    DFE_FL_MISC_CMD_CLR_MASTER_HIPRI_INTR_STATUS,    

    /// enable group of master high priority interrupts
    DFE_FL_MISC_CMD_ENB_MASTER_HIPRI_INTRGRP,
    /// disable group of master high priority interrupts
    DFE_FL_MISC_CMD_DIS_MASTER_HIPRI_INTRGRP,
    /// force generating group of master high priority interrupts
    DFE_FL_MISC_CMD_SET_FORCE_MASTER_HIPRI_INTRGRP,
    /// clear force generating group of master high priority interrupts
    DFE_FL_MISC_CMD_CLR_FORCE_MASTER_HIPRI_INTRGRP,
    /// clear status of group of master high priority interrupts
    DFE_FL_MISC_CMD_CLR_MASTER_HIPRI_INTRGRP_STATUS,    
    
    /// enable a sync interrupt
    DFE_FL_MISC_CMD_ENB_SYNC_INTR,
    /// disable a sync interrupt
    DFE_FL_MISC_CMD_DIS_SYNC_INTR,
    /// force generating a sync interrupt
    DFE_FL_MISC_CMD_SET_FORCE_SYNC_INTR,    
    /// clear force generating a sync interrupt
    DFE_FL_MISC_CMD_CLR_FORCE_SYNC_INTR,    
    /// clear status of a sync interrupt
    DFE_FL_MISC_CMD_CLR_SYNC_INTR_STATUS,

    /// enable group of sync interrupts
    DFE_FL_MISC_CMD_ENB_SYNC_INTRGRP,
    /// disable group of sync interrupts
    DFE_FL_MISC_CMD_DIS_SYNC_INTRGRP,
    /// force generating group of sync interrupts
    DFE_FL_MISC_CMD_SET_FORCE_SYNC_INTRGRP,    
    /// clear force generating group of sync interrupts
    DFE_FL_MISC_CMD_CLR_FORCE_SYNC_INTRGRP,    
    /// clear status of group of sync interrupts
    DFE_FL_MISC_CMD_CLR_SYNC_INTRGRP_STATUS,
    
    /// enable a CPP/DMA done interrupt
    DFE_FL_MISC_CMD_ENB_CPP_DMA_DONE_INTR,
    /// disable a CPP/DMA done interrupt
    DFE_FL_MISC_CMD_DIS_CPP_DMA_DONE_INTR,
    /// force generating a CPP/DMA done interrupt
    DFE_FL_MISC_CMD_SET_FORCE_CPP_DMA_DONE_INTR,    
    /// clear force generating a CPP/DMA done interrupt
    DFE_FL_MISC_CMD_CLR_FORCE_CPP_DMA_DONE_INTR,    
    /// clear status of a CPP/DMA done interrupt
    DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTR_STATUS,    
   
    /// enable group of CPP/DMA done interrupts
    DFE_FL_MISC_CMD_ENB_CPP_DMA_DONE_INTRGRP,
    /// disable group of CPP/DMA done interrupts
    DFE_FL_MISC_CMD_DIS_CPP_DMA_DONE_INTRGRP,
    /// force generating group of CPP/DMA done interrupts
    DFE_FL_MISC_CMD_SET_FORCE_CPP_DMA_DONE_INTRGRP,    
    /// clear force generating group of CPP/DMA done interrupts
    DFE_FL_MISC_CMD_CLR_FORCE_CPP_DMA_DONE_INTRGRP,    
    /// clear status of group of CPP/DMA done interrupts
    DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTRGRP_STATUS,    
    
    /// enable a Misc interrupt
    DFE_FL_MISC_CMD_ENB_MISC_INTR,
    /// disable a Misc interrupt
    DFE_FL_MISC_CMD_DIS_MISC_INTR,
    /// force generating  a Misc interrupt
    DFE_FL_MISC_CMD_SET_FORCE_MISC_INTR,     
    /// clear force generating  a Misc interrupt
    DFE_FL_MISC_CMD_CLR_FORCE_MISC_INTR,     
    /// clear status of a Misc interrupt
    DFE_FL_MISC_CMD_CLR_MISC_INTR_STATUS,     
    
    /// enable group of Misc interrupts
    DFE_FL_MISC_CMD_ENB_MISC_INTRGRP,
    /// disable group of Misc interrupts
    DFE_FL_MISC_CMD_DIS_MISC_INTRGRP,
    /// force generating  group of Misc interrupts
    DFE_FL_MISC_CMD_SET_FORCE_MISC_INTRGRP,     
    /// clear force generating group of Misc interrupts
    DFE_FL_MISC_CMD_CLR_FORCE_MISC_INTRGRP,     
    /// clear status of group of Misc interrupts
    DFE_FL_MISC_CMD_CLR_MISC_INTRGRP_STATUS,     

    /**
     * DVGA
     */
    /// Set DVGA Enable mode
    DFE_FL_MISC_CMD_SET_DVGA_ENABLE,
    /// config DVGA
    DFE_FL_MISC_CMD_CFG_DVGA,
    
    
    DFE_FL_MISC_CMD_MAX_VALUE
} DfeFl_MiscHwControlCmd;

/** @brief query commands
 */
typedef enum
{
    /**
     * interrupt
     */
    /// get status of a master low priority interrupt
    DFE_FL_MISC_QUERY_GET_MASTER_LOWPRI_INTR_STATUS,
    /// get status of group of master low priority interrupts
    DFE_FL_MISC_QUERY_GET_MASTER_LOWPRI_INTRGRP_STATUS,
    /// get status of a master high priority interrupt
    DFE_FL_MISC_QUERY_GET_MASTER_HIPRI_INTR_STATUS,
    /// get status of group of master high priority interrupts
    DFE_FL_MISC_QUERY_GET_MASTER_HIPRI_INTRGRP_STATUS,
    /// get status of a sync interrupt
    DFE_FL_MISC_QUERY_GET_SYNC_INTR_STATUS,
    /// get status of group of sync interrupts
    DFE_FL_MISC_QUERY_GET_SYNC_INTRGRP_STATUS,
    /// get status of a CPP/DMA done interrupt
    DFE_FL_MISC_QUERY_GET_CPP_DMA_DONE_INTR_STATUS,
    /// get status of group of CPP/DMA done interrupts
    DFE_FL_MISC_QUERY_GET_CPP_DMA_DONE_INTRGRP_STATUS,
    /// get status of a Misc interrupt
    DFE_FL_MISC_QUERY_GET_MISC_INTR_STATUS,
    /// get status of group of Misc interrupts
    DFE_FL_MISC_QUERY_GET_MISC_INTRGRP_STATUS,
    
    /**
     * GPIO
     */
    /// get GPIO bank inputs
    DFE_FL_MISC_QUERY_GET_GPIO_BANK,
    /// get GPIO pin input
    DFE_FL_MISC_QUERY_GET_GPIO_PIN,
      
    DFE_FL_MISC_QUERY_MAX_VALUE
} DfeFl_MiscHwStatusQuery;

 
/** @brief gpio pin
 */
typedef enum
{
    /// DFE GPIO pin 0
    DFE_FL_MISC_GPIO_PIN_0 = 0,
    /// DFE GPIO pin 1
    DFE_FL_MISC_GPIO_PIN_1,
    /// DFE GPIO pin 2
    DFE_FL_MISC_GPIO_PIN_2,
    /// DFE GPIO pin 3
    DFE_FL_MISC_GPIO_PIN_3,
    /// DFE GPIO pin 4
    DFE_FL_MISC_GPIO_PIN_4,
    /// DFE GPIO pin 5
    DFE_FL_MISC_GPIO_PIN_5,
    /// DFE GPIO pin 6
    DFE_FL_MISC_GPIO_PIN_6,
    /// DFE GPIO pin 7
    DFE_FL_MISC_GPIO_PIN_7,
    /// DFE GPIO pin 8
    DFE_FL_MISC_GPIO_PIN_8,
    /// DFE GPIO pin 9
    DFE_FL_MISC_GPIO_PIN_9,
    /// DFE GPIO pin 10
    DFE_FL_MISC_GPIO_PIN_10,
    /// DFE GPIO pin 11
    DFE_FL_MISC_GPIO_PIN_11,
    /// DFE GPIO pin 12
    DFE_FL_MISC_GPIO_PIN_12,
    /// DFE GPIO pin 13
    DFE_FL_MISC_GPIO_PIN_13,
    /// DFE GPIO pin 14
    DFE_FL_MISC_GPIO_PIN_14,
    /// DFE GPIO pin 15
    DFE_FL_MISC_GPIO_PIN_15,
    /// DFE GPIO pin 16
    DFE_FL_MISC_GPIO_PIN_16,
    /// DFE GPIO pin 17
    DFE_FL_MISC_GPIO_PIN_17
} DfeFl_MiscGpioPin;

typedef enum
{
    /// DFE SYNC Sig NEVER
    DFE_FL_SYNC_GEN_SIG_NEVER = 0,
    /// DFE SYNC Sig MPU_SYNC
    DFE_FL_SYNC_GEN_SIG_MPU_SYNC = 1,
    /// DFE SYNC Sig UL_IQ0_FSTROBE_SYNC0
    DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0 = 2,
    /// DFE SYNC Sig UL_IQ0_FSTROBE_SYNC1
    DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1 = 3,
    /// DFE SYNC Sig DL_IQ0_FSTART_SYNC0
    DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0 = 4,
    /// DFE SYNC Sig DL_IQ0_FSTART_SYNC1
    DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1 = 5,
    /// DFE SYNC Sig GPIO_SYNC_IN0
    DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0 = 6,
    /// DFE SYNC Sig GPIO_SYNC_IN1
    DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1 = 7,
    /// DFE SYNC Sig JESD_SYNC_IN
    DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN = 8,
    /// DFE SYNC Sig SYSREF
    DFE_FL_SYNC_GEN_SIG_SYSREF = 9,
    /// DFE SYNC Sig MASTER_INTR0
    DFE_FL_SYNC_GEN_SIG_MASTER_INTR0 = 10,
    /// DFE SYNC Sig MASTER_INTR1
    DFE_FL_SYNC_GEN_SIG_MASTER_INTR1 = 11,
    /// DFE SYNC Sig SYNC_GEN_CNTR0
    DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0 = 12,
    /// DFE SYNC Sig SYNC_GEN_CNTR1
    DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1 = 13,
    /// DFE SYNC Sig SYNC_GEN_CNTR2
    DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2 = 14,
    /// DFE SYNC Sig ALWAYS
    DFE_FL_SYNC_GEN_SIG_ALWAYS = 15
} DfeFl_MiscSyncGenSig;

/** @brief master low prority interrupt 
 */
typedef enum
{
    /// DFE low priority interrupt from BB
    DFE_FL_MISC_MASTER_LOWPRI_BB,           
    /// DFE low priority interrupt from BBTX_POWMTR
    DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR,  
    /// DFE low priority interrupt from BBRX_POWMTR
    DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR,  
    /// DFE low priority interrupt from BBTX_GAINUPT
    DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, 
    /// DFE low priority interrupt from BBRX_GAINUPT
    DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, 
    /// DFE low priority interrupt from DDUC0
    DFE_FL_MISC_MASTER_LOWPRI_DDUC0,        
    /// DFE low priority interrupt from DDUC1
    DFE_FL_MISC_MASTER_LOWPRI_DDUC1,        
    /// DFE low priority interrupt from DDUC2
    DFE_FL_MISC_MASTER_LOWPRI_DDUC2,        
    /// DFE low priority interrupt from DDUC3
    DFE_FL_MISC_MASTER_LOWPRI_DDUC3,        
    /// DFE low priority interrupt from CFR0
    DFE_FL_MISC_MASTER_LOWPRI_CFR0,         
    /// DFE low priority interrupt from CFR1
    DFE_FL_MISC_MASTER_LOWPRI_CFR1,         
    /// DFE low priority interrupt from DPD
    DFE_FL_MISC_MASTER_LOWPRI_DPD,          
    /// DFE low priority interrupt from DPDA
    DFE_FL_MISC_MASTER_LOWPRI_DPDA,         
    /// DFE low priority interrupt from TX
    DFE_FL_MISC_MASTER_LOWPRI_TX,           
    /// DFE low priority interrupt from JESD
    DFE_FL_MISC_MASTER_LOWPRI_JESD,         
    /// DFE low priority interrupt from RX_IBPM
    DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM,      
    /// DFE low priority interrupt from CB
    DFE_FL_MISC_MASTER_LOWPRI_CB,           
    /// DFE low priority interrupt from MISC
    DFE_FL_MISC_MASTER_LOWPRI_MISC         

} DfeFl_MiscMasterLowPriIntr;

/** @brief master low prority interrupt 
 */
typedef enum
{
    /// DFE high priority interrupt, TXB_ANT1_PEAK_CLIP 
    DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP,
    /// DFE high priority interrupt, TXB_ANT1_PWR_SAT 
    DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT,
    /// DFE high priority interrupt, TXB_ANT1_TX_ZERO
    DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO,
    /// DFE high priority interrupt, TXB_ANT1_CFR_GAIN 
    DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN,
    /// DFE high priority interrupt, TXB_ANT0_PEAK_CLIP 
    DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP,
    /// DFE high priority interrupt, TXB_ANT0_PWR_SAT
    DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT,
    /// DFE high priority interrupt, TXB_ANT0_TX_ZERO 
    DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO,
    /// DFE high priority interrupt, TXB_ANT0_CFR_GAIN 
    DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN,
    /// DFE high priority interrupt, TXA_ANT1_PEAK_CLIP 
    DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP,
    /// DFE high priority interrupt, TXA_ANT1_PWR_SAT 
    DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT,
    /// DFE high priority interrupt, TXA_ANT1_TX_ZERO 
    DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO,
    /// DFE high priority interrupt, TXA_ANT1_CFR_GAIN 
    DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN,
    /// DFE high priority interrupt, TXA_ANT0_PEAK_CLIP 
    DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP,
    /// DFE high priority interrupt, TXA_ANT0_PWR_SAT 
    DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT,
    /// DFE high priority interrupt, TXA_ANT0_TX_ZERO 
    DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO,
    /// DFE high priority interrupt, TXA_ANT0_CFR_GAIN 
    DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN,
    /// DFE high priority interrupt, CPP_DMA_ERR 
    DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR
    
} DfeFl_MiscMasterHiPriIntr;

/** @brief misc interrupt 
 */
typedef enum
{
    /// DFE MISC interrupt, CPP_RD_NACK
    DFE_FL_MISC_MISC_INTR_CPP_RD_NACK,
    /// DFE MISC interrupt, ARB_FIFO_ERR
    DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR,
    /// DFE MISC interrupt, ARB_P2L_DONE
    DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, 
    /// DFE MISC interrupt, ARB_FB_SWITCH
    DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH,
    /// DFE MISC interrupt, GPIO_0
    DFE_FL_MISC_MISC_INTR_GPIO_0,
    /// DFE MISC interrupt, GPIO_1
    DFE_FL_MISC_MISC_INTR_GPIO_1,
    /// DFE MISC interrupt, GPIO_2
    DFE_FL_MISC_MISC_INTR_GPIO_2,
    /// DFE MISC interrupt, GPIO_3
    DFE_FL_MISC_MISC_INTR_GPIO_3,
    /// DFE MISC interrupt, GPIO_4
    DFE_FL_MISC_MISC_INTR_GPIO_4,
    /// DFE MISC interrupt, GPIO_5
    DFE_FL_MISC_MISC_INTR_GPIO_5,
    /// DFE MISC interrupt, GPIO_6
    DFE_FL_MISC_MISC_INTR_GPIO_6,
    /// DFE MISC interrupt, GPIO_7
    DFE_FL_MISC_MISC_INTR_GPIO_7,
    /// DFE MISC interrupt, GPIO_8
    DFE_FL_MISC_MISC_INTR_GPIO_8,
    /// DFE MISC interrupt, GPIO_9
    DFE_FL_MISC_MISC_INTR_GPIO_9,
    /// DFE MISC interrupt, GPIO_10
    DFE_FL_MISC_MISC_INTR_GPIO_10,
    /// DFE MISC interrupt, GPIO_11
    DFE_FL_MISC_MISC_INTR_GPIO_11,
    /// DFE MISC interrupt, GPIO_12
    DFE_FL_MISC_MISC_INTR_GPIO_12,
    /// DFE MISC interrupt, GPIO_13
    DFE_FL_MISC_MISC_INTR_GPIO_13,
    /// DFE MISC interrupt, GPIO_14
    DFE_FL_MISC_MISC_INTR_GPIO_14,
    /// DFE MISC interrupt, GPIO_15
    DFE_FL_MISC_MISC_INTR_GPIO_15,
    /// DFE MISC interrupt, GPIO_16
    DFE_FL_MISC_MISC_INTR_GPIO_16,
    /// DFE MISC interrupt, GPIO_17
    DFE_FL_MISC_MISC_INTR_GPIO_17
} DfeFl_MiscMiscIntr;

/** @brief sync gen counter
 */
typedef enum
{
    /// DFE SyncGen counter 0
    DFE_FL_SYNC_GEN_CNTR0 = 0,
    /// DFE SyncGen counter 1
    DFE_FL_SYNC_GEN_CNTR1,
    /// DFE SyncGen counter 2
    DFE_FL_SYNC_GEN_CNTR2
} DfeFl_MiscSyncGenCntr;

/** @brief GPIO mux
 */
typedef enum
{
    /// DFE GPIO MUX Select, NOTHING (input)
    DFE_FL_MISC_GPIO_MUX_NOTHING = 0,

    /// DFE GPIO MUX Select, GPIO_SYNC_IN0
    DFE_FL_MISC_GPIO_MUX_GPIO_SYNC_IN0 = 1,
    /// DFE GPIO MUX Select, GPIO_SYNC_IN1
    DFE_FL_MISC_GPIO_MUX_GPIO_SYNC_IN1 = 2,
    /// DFE GPIO MUX Select, JESD_SYNC_IN0
    DFE_FL_MISC_GPIO_MUX_JESD_SYNC_IN0 = 3,
    /// DFE GPIO MUX Select, JESD_SYNC_IN1
    DFE_FL_MISC_GPIO_MUX_JESD_SYNC_IN1 = 4,

    /// DFE GPIO MUX Select, GPIO_SYNC_OUT0
    DFE_FL_MISC_GPIO_MUX_GPIO_SYNC_OUT0 = 8,
    /// DFE GPIO MUX Select, GPIO_SYNC_OUT1
    DFE_FL_MISC_GPIO_MUX_GPIO_SYNC_OUT1 = 9,

    /// DFE GPIO MUX Select, JESD_SYNC_OUT0
    DFE_FL_MISC_GPIO_MUX_JESD_SYNC_OUT0 = 10,
    /// DFE GPIO MUX Select, JESD_SYNC_OUT1
    DFE_FL_MISC_GPIO_MUX_JESD_SYNC_OUT1 = 11,
    /// DFE GPIO MUX Select, FB_MUX_CNTL0
    DFE_FL_MISC_GPIO_MUX_FB_MUX_CNTL0 = 12,
    /// DFE GPIO MUX Select, FB_MUX_CNTL1
    DFE_FL_MISC_GPIO_MUX_FB_MUX_CNTL1 = 13,
    /// DFE GPIO MUX Select, FB_MUX_CNTL2
    DFE_FL_MISC_GPIO_MUX_FB_MUX_CNTL2 = 14,
    /// DFE GPIO MUX Select, FB_MUX_CNTL3
    DFE_FL_MISC_GPIO_MUX_FB_MUX_CNTL3 = 15,
    /// DFE GPIO MUX Select, FB_MUX_CNTL4
    DFE_FL_MISC_GPIO_MUX_FB_MUX_CNTL4 = 16,
    /// DFE GPIO MUX Select, FB_MUX_CNTL5
    DFE_FL_MISC_GPIO_MUX_FB_MUX_CNTL5 = 17,
    /// DFE GPIO MUX Select, DVGA_CNTL0
    DFE_FL_MISC_GPIO_MUX_DVGA_CNTL0 = 18,
    /// DFE GPIO MUX Select, DVGA_CNTL1
    DFE_FL_MISC_GPIO_MUX_DVGA_CNTL1 = 19,
    /// DFE GPIO MUX Select, DVGA_CNTL2
    DFE_FL_MISC_GPIO_MUX_DVGA_CNTL2 = 20,
    /// DFE GPIO MUX Select, DVGA_CNTL3
    DFE_FL_MISC_GPIO_MUX_DVGA_CNTL3 = 21,
    /// DFE GPIO MUX Select, DVGA_CNTL4
    DFE_FL_MISC_GPIO_MUX_DVGA_CNTL4 = 22,
    /// DFE GPIO MUX Select, DVGA_CNTL5
    DFE_FL_MISC_GPIO_MUX_DVGA_CNTL5 = 23,
    /// DFE GPIO MUX Select, DVGA_CNTL6
    DFE_FL_MISC_GPIO_MUX_DVGA_CNTL6 = 24,
    /// DFE GPIO MUX Select, DVGA_CNTL7
    DFE_FL_MISC_GPIO_MUX_DVGA_CNTL7 = 25,
    /// DFE GPIO MUX Select, SYSREF_REQUEST
    DFE_FL_MISC_GPIO_MUX_SYSREF_REQUEST = 26,
    /// DFE GPIO MUX Select, MPU_GPIO_DRIVE (output)
    DFE_FL_MISC_GPIO_MUX_MPU_GPIO_DRIVE = 27

} DfeFl_MiscGpioMux;

/** @brief DVGA mode
 */
typedef enum
{
    /// DFE DVGA mode, TRANSPARENT
    DFE_FL_MISC_DVGA_MODE_TRANSPARENT = 0,
    /// DFE DVGA mode, CLOCKED
    DFE_FL_MISC_DVGA_MODE_CLOCKED

} DfeFl_MiscDvgaMode;

/** @brief DVGA transparent mode
 */
typedef enum
{
    /// DFE DVGA TRANSPARENT mode 0
    DFE_FL_MISC_DVGA_TR_MODE0 = 0,
    /// DFE DVGA TRANSPARENT mode 1
    DFE_FL_MISC_DVGA_TR_MODE1,
    /// DFE DVGA TRANSPARENT mode 2
    DFE_FL_MISC_DVGA_TR_MODE2,
    /// DFE DVGA TRANSPARENT mode 3
    DFE_FL_MISC_DVGA_TR_MODE3
} DfeFl_MiscDvgaTransparentMode;

/** @brief DVGA transparent mode
 */
typedef enum
{
    /// DFE DVGA latch enable polarity, POSITIVE
    DFE_FL_MISC_DVGA_LATCH_ENABLE_POLARITY_POSITIVE = 0,
    /// DFE DVGA latch enable polarity, NEGATIVE
    DFE_FL_MISC_DVGA_LATCH_ENABLE_POLARITY_NEGATIVE
} DfeFl_MiscDvgaLatchEnablePolarity;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_MISC_DATASTRUCT
 * @{
 */

/** @brief argument for runtime control,
 *      DFE_FL_MISC_QUERY_GET_MASTER_LOWPRI_INTR_STATUS,
 *      DFE_FL_MISC_QUERY_GET_MASTER_HIPRI_INTR_STATUS,
 *      DFE_FL_MISC_QUERY_GET_SYNC_INTR_STATUS,
 *      DFE_FL_MISC_QUERY_GET_CPP_DMA_DONE_INTR_STATUS,
 *      DFE_FL_MISC_QUERY_GET_MISC_INTR_STATUS,
 */
typedef struct
{
    /// interrupt selection
    uint32_t intr;
    
    /// data
    uint32_t data;
} DfeFl_MiscIntrStatus;

/** @brief argument for runtime control,
 *      DFE_FL_MISC_CMD_ISSUE_SYNC
 */
typedef struct
{
    /// sync signal
    uint32_t syncSig;
    /// wait count, waiting for new signal coming
    ///  0x00000000 = no wait
    ///  0xffffffff = wait for ever
    ///  others = wait loop count 
    uint32_t waitCnt;
} DfeFl_MiscSyncGenIssueSyncConfig;

/** @brief argument for runtime control,
 *      DFE_FL_MISC_CMD_CFG_SYNC_ONE_SHOT
 *      DFE_FL_MISC_CMD_SET_UL_SIG_FSTROBE
 *      DFE_FL_MISC_CMD_SET_DL_SIG_FSTART
 *      DFE_FL_MISC_QUERY_GET_SYNC_STATUS
 */
typedef struct
{
    /// sync signal
    uint32_t syncSig;
    
    /// data
    uint32_t data;
} DfeFl_MiscSyncGenGeneric;


/** @brief argument for runtime control,
 *      DFE_FL_MISC_CMD_CFG_SYNC_CNTR
 */
typedef struct
{
    /// counter id
    uint32_t cntr;
    
    /**
     * sync counter repeat.
     * If 0, counter counts down delay clocks once, sends a sync, and stops. 
     * If 1, it counts down delay clocks sends a sync, then continuously sends more syncs every period clocks.
     */
    uint32_t repeat;
    /// sync counter invert. Set to 1 to invert the entire bus.
    uint32_t invert;
    /// sync counter period. Number of clocks to wait between syncs when repeat is 1. Does nothing when repeat is 0.
    uint32_t period;
    /**
     * sync counter delay. Number of clocks to wait after sync select source before sending initial sync. 
     * If set to 0, sync counter output will be high if sync select source is high.
     */
    uint32_t delay;
    /// sync counter pulse width, (set to X for pulse width of X clocks; 0 means it will never go high)
    uint32_t pulse;
} DfeFl_MiscSyncCntrConfig;

/** @brief argument for runtime control,
 *      DFE_FL_MISC_CMD_SET_SYNC_CNTR_SSEL
 */
typedef struct
{
    // counter id
    uint32_t cntr;
    
    /** 
     * sync counter sync select; sync counter does NOT start until a sync is sent.  
     * This is written to a shadow register, updated by progSsel.
     */
    uint32_t startSsel;
    /// update startSsel from shadow to working memory
    uint32_t progSsel;
} DfeFl_MiscSyncGenCntrSsel;

/** @brief argument for runtime control,
 *      DFE_FL_MISC_CMD_SET_GPIO_SYNCOUT_SSEL
 */
typedef struct
{
    /// syncout, 0 ~ 1
    uint32_t syncout;
    
    /// sync select
    uint32_t ssel;
} DfeFl_MiscGpioSyncOutSselConfig;

/** @brief argument for runtime control,
 *      DFE_FL_MISC_CMD_SET_GPIO_PIN_MUX
 */
typedef struct
{
    /// gpio pin 0 ~ 17
    uint32_t pin;
    
    /// mux value
    uint32_t mux;
} DfeFl_MiscGpioPinMuxConfig;

/** @brief argument for runtime control,
 *      DFE_FL_MISC_CMD_SET_GPIO_PIN
 *      DFE_FL_MISC_QUERY_GET_GPIO_PIN
 */
typedef struct
{
    /// gpio pin 0 ~ 17
    uint32_t pin;
    
    /// pin value 0/1
    uint32_t value;
} DfeFl_MiscGpioPinIo;

typedef struct
{
    /// per bit stream select (0-3)
    uint32_t streamSel[8];
    /// per bit select
    uint32_t bitSel[8];
} DfeFl_MiscDvgaTransparentModeConfig;

typedef struct
{
    /// number of streams minus one
    uint32_t numStrsM1;
    /// latch enable polarity
    uint32_t lePolarity[4];
    /// latch enable pulse width (clock cycles minus 1)
    uint32_t leWidthM1[4];    
} DfeFl_MiscDvgaClockModeConfig;

/** @brief argument for runtime control,
 *      DFE_FL_MISC_CMD_CFG_DVGA
 */
typedef struct
{
    /// dvga mode, transparent/clock
    uint32_t dvgaMode;
    
    /// dvga config
    union
    {
        /// config for transparent mode
        DfeFl_MiscDvgaTransparentModeConfig trModeCfg;
        
        /// config clock mode
        DfeFl_MiscDvgaClockModeConfig clockModeCfg;        
    } un;
} DfeFl_MiscDvgaConfig;


/** @brief master low prority interrupt group, argument for runtime control,
    DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTRGRP,
    DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTRGRP,
    DFE_FL_MISC_CMD_SET_FORCE_MASTER_LOWPRI_INTRGRP,    
    DFE_FL_MISC_CMD_CLR_FORCE_MASTER_LOWPRI_INTRGRP,
    DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTRGRP_STATUS,    
    DFE_FL_MISC_QUERY_GET_MASTER_LOWPRI_INTRGRP_STATUS
 */
typedef struct
{
    /// DFE low priority interrupt from BB
    uint32_t bb;
    /// DFE low priority interrupt from BBTX_POWMTR
    uint32_t bbtxPowmtr;
    /// DFE low priority interrupt from BBRX_POWMTR
    uint32_t bbrxPowmtr;
    /// DFE low priority interrupt from BBTX_GAINUPT
    uint32_t bbtxGainUpt;
    /// DFE low priority interrupt from BBRX_GAINUPT
    uint32_t bbrxGainUpt;
    /// DFE low priority interrupt from DDUC0..3
    uint32_t dduc[4];
    /// DFE low priority interrupt from CFR0..1
    uint32_t cfr[2];
    /// DFE low priority interrupt from DPD
    uint32_t dpd;
    /// DFE low priority interrupt from DPDA
    uint32_t dpda;
    /// DFE low priority interrupt from TX
    uint32_t tx;
    /// DFE low priority interrupt from JESD
    uint32_t jesd;
    /// DFE low priority interrupt from RX_IBPM
    uint32_t rxIbpm;
    /// DFE low priority interrupt from CB
    uint32_t cb;
    /// DFE low priority interrupt from MISC
    uint32_t misc;
} DfeFl_MiscMasterLowPriIntrGroup;

/** @brief master low prority interrupt group, argument for runtime control,
    DFE_FL_MISC_CMD_ENB_MASTER_HIPRI_INTRGRP,
    DFE_FL_MISC_CMD_DIS_MASTER_HIPRI_INTRGRP,
    DFE_FL_MISC_CMD_SET_FORCE_MASTER_HIPRI_INTRGRP,
    DFE_FL_MISC_CMD_CLR_FORCE_MASTER_HIPRI_INTRGRP,
    DFE_FL_MISC_CMD_CLR_MASTER_HIPRI_INTR_STATUSGRP,  
    DFE_FL_MISC_QUERY_GET_MASTER_HIPRI_INTRGRP_STATUS  
 */
typedef struct
{
    /// DFE high priority interrupt, TXB_ANT1_PEAK_CLIP 
    uint32_t txbAnt1PeakClip;
    /// DFE high priority interrupt, TXB_ANT1_PWR_SAT 
    uint32_t txbAnt1PwrSat;
    /// DFE high priority interrupt, TXB_ANT1_TX_ZERO
    uint32_t txbAnt1TxZero;
    /// DFE high priority interrupt, TXB_ANT1_CFR_GAIN 
    uint32_t txbAnt1CfrGain;
    /// DFE high priority interrupt, TXB_ANT0_PEAK_CLIP 
    uint32_t txbAnt0PeakClip;
    /// DFE high priority interrupt, TXB_ANT0_PWR_SAT
    uint32_t txbAnt0PwrSat;
    /// DFE high priority interrupt, TXB_ANT0_TX_ZER0 
    uint32_t txbAnt0TxZero;
    /// DFE high priority interrupt, TXB_ANT0_CFR_GAIN 
    uint32_t txbAnt0CfrGain;
    /// DFE high priority interrupt, TXA_ANT1_PEAK_CLIP 
    uint32_t txaAnt1PeakClip;
    /// DFE high priority interrupt, TXA_ANT1_PWR_SAT 
    uint32_t txaAnt1PwrSat;
    /// DFE high priority interrupt, TXA_ANT1_TX_ZER0
    uint32_t txaAnt1TxZero;
    /// DFE high priority interrupt, TXA_ANT1_CFR_GAIN 
    uint32_t txaAnt1CfrGain;
    /// DFE high priority interrupt, TXA_ANT0_PEAK_CLIP 
    uint32_t txaAnt0PeakClip;
    /// DFE high priority interrupt, TXA_ANT0_PWR_SAT
    uint32_t txaAnt0PwrSat;
    /// DFE high priority interrupt, TXA_ANT0_TX_ZER0 
    uint32_t txaAnt0TxZero;
    /// DFE high priority interrupt, TXA_ANT0_CFR_GAIN 
    uint32_t txaAnt0CfrGain;
    /// DFE high priority interrupt, CPP_DMA_ERR
    uint32_t cppDmaErr;
    
} DfeFl_MiscMasterHiPriIntrGroup;

/** @brief misc interrupt group, argument for runtime control,
    DFE_FL_MISC_CMD_ENB_MISC_INTRGRP,
    DFE_FL_MISC_CMD_DIS_MISC_INTRGRP,
    DFE_FL_MISC_CMD_SET_FORCE_MISC_INTRGRP,     
    DFE_FL_MISC_CMD_CLR_FORCE_MISC_INTRGRP,     
    DFE_FL_MISC_CMD_CLR_MISC_INTR_STATUSGRP,
    DFE_FL_MISC_QUERY_GET_MISC_INTRGRP_STATUS     
 */
typedef struct
{
    /// DFE MISC interrupt, CPP_RD_NACK
    uint32_t cppRdNack;
    /// DFE MISC interrupt, ARB_FIFO_ERR
    uint32_t arbFifoErr;
    /// DFE MISC interrupt, ARB_P2L_DONE
    uint32_t arbP2lDone;
    /// DFE MISC interrupt, ARB_FB_SWITCH
    uint32_t arbFbSwitch;
    /// DFE MISC interrupt, GPIO_0..17
    uint32_t gpio[18];
} DfeFl_MiscMiscIntrGroup;

/** @brief misc interrupt group, argument for runtime control,
    DFE_FL_MISC_CMD_ENB_SYNC_INTRGRP,
    DFE_FL_MISC_CMD_DIS_SYNC_INTRGRP,
    DFE_FL_MISC_CMD_SET_FORCE_SYNC_INTRGRP,    
    DFE_FL_MISC_CMD_CLR_FORCE_SYNC_INTRGRP,    
    DFE_FL_MISC_CMD_CLR_SYNC_INTRGRP_STATUS,
    DFE_FL_MISC_QUERY_GET_SYNC_INTRGRP_STATUS
 */
typedef struct
{
    /// DFE Sync Intr NEVER
    uint32_t never;
    /// DFE Sync Intr MPU_SYNC
    uint32_t mpuSync;
    /// DFE Sync Intr UL_IQ0_FSTROBE_SYNC0
    uint32_t ulIq0FStrobeSync0;
    /// DFE Sync Intr UL_IQ0_FSTROBE_SYNC1
    uint32_t ulIq0FStrobeSync1;
    /// DFE Sync Intr DL_IQ0_FSTART_SYNC0
    uint32_t dlIq0FStartSync0;
    /// DFE Sync Intr DL_IQ0_FSTART_SYNC1
    uint32_t dlIq0FStartSync1;
    /// DFE Sync Intr GPIO_SYNC_IN0
    uint32_t gpioSyncIn0;
    /// DFE Sync Intr GPIO_SYNC_IN1
    uint32_t gpioSyncIn1;
    /// DFE Sync Intr JESD_SYNC_IN
    uint32_t jesdSyncIn;
    /// DFE Sync Intr SYSREF
    uint32_t sysref;
    /// DFE Sync Intr MASTER_INTR0
    uint32_t masterIntr0;
    /// DFE Sync Intr MASTER_INTR1
    uint32_t masterIntr1;
    /// DFE Sync Intr SYNC_GEN_CNTR0
    uint32_t syncGenCntr0;
    /// DFE Sync Intr SYNC_GEN_CNTR1
    uint32_t syncGenCntr1;
    /// DFE Sync Intr SYNC_GEN_CNTR2
    uint32_t syncGenCntr2;
    /// DFE Sync Intr ALWAYS
    uint32_t always;
} DfeFl_MiscSyncIntrGroup;

/** @brief misc interrupt group, argument for runtime control,
    DFE_FL_MISC_CMD_ENB_CPP_DMA_DONE_INTRGRP,
    DFE_FL_MISC_CMD_DIS_CPP_DMA_DONE_INTRGRP,
    DFE_FL_MISC_CMD_SET_FORCE_CPP_DMA_DONE_INTRGRP,    
    DFE_FL_MISC_CMD_CLR_FORCE_CPP_DMA_DONE_INTRGRP,    
    DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTRGRP_STATUS,    
    DFE_FL_MISC_QUERY_GET_CPP_DMA_DONE_INTRGRP_STATUS
 */
typedef struct
{
    /// DFE CPP/DMA Done0..31
    uint32_t dmaDone[32];
} DfeFl_MiscCppIntrGroup;

/** @brief misc interrupt group, argument for runtime control,
 *
 */
typedef struct
{
	DfeFl_MiscIntrStatus syncSig;
	DfeFl_MiscIntrStatus cppDmaDone;
	DfeFl_MiscMiscIntrGroup arbGpio;
} DfeFl_MiscCppIntrGroupStatus;

/** @brief overlay register pointer to MISC instance
 */
typedef CSL_DFE_MISC_REGS *DfeFl_MiscRegsOvly;

/** @brief Misc Object of Digital radio Front End (DFE) */
typedef struct 
{
    /// handle to DFE global
    DfeFl_Handle           hDfe;
    
    /// pointer to register base address of a MISC instance
    DfeFl_MiscRegsOvly     regs;
   
    /// This is the instance of MISC being referred to by this object
    DfeFl_InstNum             perNum;

} DfeFl_MiscObj;

/** @brief handle pointer to MISC object
 */
typedef DfeFl_MiscObj *DfeFl_MiscHandle;

/**
 * @}
 */

/**
 * @addtogroup DFE_FL_MISC_FUNCTION
 * @{
 */


DfeFl_MiscHandle dfeFl_MiscOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_MiscObj              *pDfeMiscObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
);

DfeFl_Status dfeFl_MiscClose(DfeFl_MiscHandle hDfeMisc);

DfeFl_Status  dfeFl_MiscHwControl
(
    DfeFl_MiscHandle           hDfeMisc,
    DfeFl_MiscHwControlCmd     ctrlCmd,
    void                        *arg
);

DfeFl_Status  dfeFl_MiscGetHwStatus
(
    DfeFl_MiscHandle           hDfeMisc,
    DfeFl_MiscHwStatusQuery    queryId,
    void                        *arg
);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_MISC_H_ */
