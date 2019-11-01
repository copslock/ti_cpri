/**
 *   @file  tcp3d_drv.h
 *
 *   @brief
 *      Header file for the TCP3 Decoder Driver. The file exposes the data
 *      structures and exported API which are available for use by the driver
 *      users.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2011, 2014 Texas Instruments, Inc.
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

/** @defgroup TCP3D_DRV_API TCP3 Decoder Driver
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *  The TCP3 decoder driver provides a well defined standard interface
 *  which allows application developers to send code blocks for decoding and
 *  receive hard decision and status via EDMA3 transfers. 
 */

#ifndef _TCP3D_DRV_H_
#define _TCP3D_DRV_H_

/**
@defgroup TCP3D_DRV_SYMBOL  TCP3D Driver Symbols Defined
@ingroup TCP3D_DRV_API
*/
/**
@defgroup TCP3D_DRV_FUNCTION  TCP3D Driver Functions
@ingroup TCP3D_DRV_API
*/
/**
@defgroup TCP3D_DRV_UTIL_FUNCTION  TCP3D Driver Utility Functions
@ingroup TCP3D_DRV_API
*/
/**
@defgroup TCP3D_DRV_DATASTRUCT  TCP3D Driver Data Structures
@ingroup TCP3D_DRV_API
*/
/**
@defgroup TCP3D_OSAL_API  TCP3D Driver OSAL Functions
@ingroup TCP3D_DRV_API
*/

/**
 *  Driver Includes
 */
/* Types include */
#include <tcp3d_drv_types.h>

/* EDMA3 LLD Driver include */
#include <ti/sdo/edma3/drv/edma3_drv.h>

/* CSL includes */
#include <ti/csl/cslr_tpcc.h>
#include <ti/csl/cslr_tcp3d_cfg.h>
#include <ti/csl/cslr_tcp3d_dma.h>

/* Version include */
#include "tcp3dver.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================= */
/** @addtogroup TCP3D_DRV_SYMBOL
 @{ */

/**
 *  @brief Used for indexing the Status Channel.
 */
#define TCP3D_DRV_MAX_NUM_INSTANCES     (CSL_TCP3D_PER_CNT)

/**
 *  @brief  This is the TCP3D Driver maximum channels required per each PING
 *          and PONG paths. Channel Index values are defined to get references
 *          to the pingCh[] or pongCh[] arrays in the driver instance.
 */
#define TCP3D_DRV_MAX_CH_PER_PATH       (2u)

/**
 *  @brief Used for indexing the Status Channel.
 */
#define TCP3D_DRV_CH_IDX_REVT           (0u)

/**
 *  @brief Used for indexing the L2 to PaRAM Channel.
 */
#define TCP3D_DRV_CH_IDX_L2P            (1u)

/**
 *  @brief  Maximum Link channels required per code block for swapping from
 *          pseudo PaRAM memory during run-time.
 */
#define TCP3D_DRV_LINK_CB               (5u)

/**
 *  @brief  Number of Link channels used for control/reload operations.
 */
#define TCP3D_DRV_LINK_CTRL             (6u)

/**
 *  @brief  Number of Link channels used for notification use.
 */
#define TCP3D_DRV_LINK_NOTIFY           (2u)

/**
 *  @brief  This is the TCP3D Driver maximum link channels required for both
 *          PING and PONG paths.
 */
#define TCP3D_DRV_MAX_LINK_CH           (((TCP3D_DRV_LINK_CB) + \
                                          (TCP3D_DRV_LINK_CTRL) + \
                                          (TCP3D_DRV_LINK_NOTIFY)) << 1) 

/**
 * @brief   This define gives the number of buffers required for TCP3D Driver.
 */
#define TCP3D_DRV_NUM_BUF               (2u)

/**
 * @brief   This is used for getting the instance buffer index.
 */
#define TCP3D_DRV_INST_BUFN             (0u)

/**
 * @brief   This is used for getting the pseudo param buffer index.
 */
#define TCP3D_DRV_PSEUDO_PARAM_BUFN     (1u)

/**
 *  @brief  This is the start flag for PING.
 */
#define TCP3D_DRV_START_PING            (1u)

/**
 *  @brief  This is the start flag for PONG.
 */
#define TCP3D_DRV_START_PONG            (2u)

/**
 *  @brief  This is the start flag for AUTO.
 */
#define TCP3D_DRV_START_AUTO            (3u)

/**
@}
*/

/* ========================================================================= */

/* ========================================================================= */
/** @addtogroup TCP3D_DRV_DATASTRUCT
 @{ */

/**
 *  @brief Memory Buffer class definitions.
 */
typedef enum Tcp3d_BufClass
{
    Tcp3d_BufClass_EXTERNAL = 0,    /**< generic (bulk) external memory */
    Tcp3d_BufClass_INTERNAL,        /**< generic (bulk) internal memory */
    Tcp3d_BufClass_L2RAM,           /**< Local L2 Memory */
    Tcp3d_BufClass_L2SRAM,          /**< Shared L2 Memory (on another core) */
    Tcp3d_BufClass_MSMC             /**< Shared Memory */
} Tcp3d_BufClass;

/**
 *  @brief Driver return values to application.
 */
typedef enum Tcp3d_Result
{
    TCP3D_DRV_NO_ERR,                   /**< No Error, Success */
    TCP3D_DRV_FAIL,                     /**< General Error */
    TCP3D_DRV_INPUT_LIST_FULL,          /**< Enqueue error message */
    TCP3D_DRV_INVALID_INSTANCE_NUMBER,  /**< Unsupported Instance number */
    TCP3D_DRV_INVALID_PARAMETER,        /**< Parameter value is not correct */
    TCP3D_DRV_INVALID_STATE,            /**< API called in invalid state */
    TCP3D_DRV_INVALID_BUFF,             /**< Invalid buffer pointer */
    TCP3D_DRV_INVALID_EDMA_CH,          /**< Invalid EDMA channel */
    TCP3D_DRV_FAIL_EDMA_PARAM_INIT,     /**< EDMA PaRAM initialization failed */
    TCP3D_DRV_FAIL_EDMA_ENABLE_CHANNEL, /**< EDMA enable channel failed */
    TCP3D_DRV_FAIL_EDMA_GET_PARAM_ADDR  /**< EDMA get PaRAM address failed */
} Tcp3d_Result;

/**
 *  @brief Driver state indication
 */
typedef enum Tcp3d_State
{
    /** This is the driver init state which is used to set in the state
     * variables either at the end of initialization or after the reset. */
    TCP3D_DRV_STATE_INIT = 0,
    
    /** This state indicates the driver is running in a steady state. This is
     * used when ever the driver start is done. In this state, the code blocks
     * are DMAed from input list to TCP3D for execution continuously. */
    TCP3D_DRV_STATE_RUNNING,
    
    /** This state indicates the driver is halted from steady state execution.
     * It could happen if chain to the next code block is not available when
     * the time previous block was DMAed for execution or there are no more
     * code blocks for decoding. If there are more blocks for decoding, restart
     * is required for the driver to complete all decoding. */
    TCP3D_DRV_STATE_PAUSE
} Tcp3d_State;

/**
 *  @brief  Driver Control Commands.
 */
typedef enum Tcp3d_CtrlCmd
{
    /** Used for enabling or disbling the interrupt generation by REVT channels.
     * This is done using the EDMA CC IER or IERH register. */
    TCP3D_DRV_SET_REVT_INT,        

    /** Used for clearing the pending interrupts generated by REVT channels
     * in the EDMA CC IPR or IPRH register. */
    TCP3D_DRV_CLR_REVT_INT,

    /** Used for enabling or disbling the interrupt generation by L2P channels.
     * This is done by setting the corresponding PaRAM OPT field bit. 
     * \b Note that this will result in REVT Channel ISR as the TCC value on the
     * L2P channel PaRAM is set with REVT channel number for chaining use. */
    TCP3D_DRV_SET_L2P_INT,
    
    /** Set interrupt for PING Pause channel (depricated) */
    TCP3D_DRV_SET_PING_PAUSE_INT,
    
    /** Set interrupt for PONG Pause channel (depricated) */
    TCP3D_DRV_SET_PONG_PAUSE_INT,
    
    /** Set interrupt for PING L2P channel (depricated) */
    TCP3D_DRV_SET_PING_L2P_INT,
    
    /** Set interrupt for PONG L2P channel (depricated) */
    TCP3D_DRV_SET_PONG_L2P_INT
} Tcp3d_CtrlCmd;

/**
 *  @brief Driver Status query Commands
 */
typedef enum Tcp3d_StsCmd
{
    TCP3D_DRV_GET_STATE,            /**< Get driver state information */
    TCP3D_DRV_GET_PING_OUT_IDX,     /**< To compute the PING output index in
                                            the pseudo PaRAM list */
    TCP3D_DRV_GET_PONG_OUT_IDX,     /**< To compute the PONG output index in
                                            the pseudo PaRAM list */
    TCP3D_DRV_GET_MIN_OUT_IDX       /**< To compute minimum output index in
                                            the pseudo PaRAM list, used in the
                                            wrap-around case */
} Tcp3d_StsCmd;

/**
@}
*/
/* ========================================================================= */

/* ========================================================================= */
/** @addtogroup TCP3D_DRV_DATASTRUCT
 @{ */

/**
 *  @brief Parameters which determine buffer sizes.
 *
 * These are configured and passed with getNumBuf() & getBufDesc() funcitons
 * to get the buffer requirement of the TCP3.
 */
typedef struct Tcp3d_SizeCfg
{
    uint32_t      maxCodeBlocks;  /**< Maximum code blocks for which driver will
                                    be configured */
    uint32_t      mode;           /**< TCP3 Decode mode for which the driver
                                    instance will be used */
} Tcp3d_SizeCfg;

/**
 *  @brief Memory Buffer Structure.
 *
 * This structure is used to request and/or supply the 
 * dynamic memory to the components.
 */
typedef struct Tcp3d_MemBuffer
{

  Tcp3d_BufClass  mclass;    /**< Memory class. It is used to describe kind of 
                       memory that is requested or returned. For 
                       example: external, internal, or similar. One 
                       should use MEM_CLASS_xxx constants. On RETURN, 
                       class may be changed if original memory class 
                       is exhausted. */
  uint16_t log2align; /**< Alignment information (>= 0). If zero, no 
                       alignment is required nor provided. Otherwise, 
                       memory buffer is aligned on appropriate power 
                       of 2 (e.g. if log2align=3, buffer is aligned 
                       modulo-8). */
  uint32_t size;      /**< Number of 8 bit words that are requested or supplied. 
                       Word length depends on the platform, and 
                       corresponds to the shortest element that can be 
                       addressed by the CPU. Word storage length in bits 
                       is defined in types.h as TYP_TWORD_SIZE. The 
                       maximum size depends on the platform. */
  uint16_t  volat;    /**< TRUE: Memory buffer must be restored before and 
                       saved after it is used.
                       FALSE: Save/restore are not necessary.
                       On RETURN, volatile flag may be changed if 
                       original request could not be satisfied. */
  void  *base;     /**< Base address of the requested buffer. */
} Tcp3d_MemBuffer;

/**
 * @brief   The TCP3 decoder initialization parameters structure holds all the
 *          information concerning the user channel. These values are used to
 *          generate the control configuration register values for the TCP3D.
 * 
 *          Valid values for each field are provided in the brackets in the
 *          comments following the field name.
 */ 
typedef struct Tcp3d_CtrlParams
{
    /* Mode Control Register parameters */
    uint8_t       mode;           /**< TCP3D mode (0 - 3) */
    uint16_t        doubleBuf;      /**< Enable/disable the double buffer (0,1) */
    uint16_t        intTable;       /**< Enable/disable the Interleaver Table
                                        Generation (0,1) */
    uint16_t        errIgnore;      /**< Enable/disable error detection to stop
                                        TCP3D from running (0,1) */
    uint16_t        autoTrig;       /**< Enable/disable auto trigger mode (0,1) */
    uint8_t       lteCrcSel;      /**< LTE CRC initial value selection (0,1) */

    /* Endian Control Register parameters */
    uint8_t       endInt;         /**< Interleaver Table Endian mode (0,1) */
    uint8_t       endInData;      /**< Input systematic and parity data Endian
                                        mode (0,1) */

    /* Emulation Control Register parameters */
    uint8_t       emuFreeRun;     /**< Emulation suspend signal (0,1) */   
    uint8_t       emuSoftStop;    /**< Emulation Soft or Hard Stop (0,1) */
    uint8_t       emuRtSel;       /**< Maximum number of iterations (0,1) */

    /* Process 0 Execution Register parameters */
    uint8_t       exeP0cmd;       /**< Process 0 execution command (0,1,4,5,6,7)*/

    /* Process 1 Execution Register parameters */
    uint8_t       exeP1cmd;       /**< Process 1 execution command (0,1,4,5,6,7)*/

} Tcp3d_CtrlParams;

/**
 * @brief   The TCP3D runtime parameters structure holds all the information
 *          that could be changed per code block. These values are used to
 *          generate the appropriate input configuration register values for
 *          the TCP3D.
 * 
 *          Valid values for each field are provided in the brackets in the
 *          comments following the field name.
 */ 
typedef struct Tcp3d_InCfgParams
{
    /* IC0 */
    uint8_t       numsw0;         /**< number of SW0s (0 - 62) */
    uint16_t      blockLen;       /**< code Block Length (39 - 8191) */

    /* IC1 */
    uint8_t       sw0LenSel;      /**< SW0 Length Selection value (0 - 5) */
    uint8_t       sw2LenSel;      /**< SW2 Length Selection value (0,1,2) */
    uint8_t       sw1Len;         /**< SW1 Length (9 - 127) */

    /* IC2 */
    uint8_t       intLoadSel;     /**< Interleaver Table load or generate
                                        selection (0,1) */
    uint16_t        maxStar;        /**< Enable/disable Max Star (0,1) */
    uint16_t        outStsRead;     /**< Enable/disable Output Status registers
                                        read via EDMA3 (0,1) */
    uint8_t       outOrderSel;    /**< Output bit order swapping within 32-bit
                                        word (0,1) */
    uint16_t        extScale;       /**< Enable/disable Extrinsic scaling (0,1) */
    uint16_t        softOutRead;    /**< Enable/disable Soft outputs read
                                        via EDMA3 (0,1) */
    uint8_t       softOutOrderSel;/**< Soft output byte order (0,1)
                                        used only in BIG ENDIAN mode */
    uint8_t       softOutFrmtSel; /**< Soft output bit format (0,1) */
    uint8_t       minIter;        /**< Minumun iterations (0 - 15) */
    uint8_t       maxIter;        /**< Maximum iterations (0 - 15) */
    uint8_t       snrVal;         /**< SNR threshold value in dB used as stopping
                                        criteria (0 - 20) */
    uint16_t        snrReport;      /**< Enable/disable SNR reporting (0,1) */
    uint8_t       stopSel;        /**< Stopping criteria selection (0 - 3) */
    uint8_t       crcIterSel;     /**< LTE CRC consecutive matches for
                                        stopping (0 - 3 ) */
    uint8_t       crcPolySel;     /**< LTE CRC polynomial selection (0,1) */

    /* IC3 */
    uint8_t       maxStarThres;   /**< Max Star Threshold value (0 - 63) */
    uint8_t       maxStarValue;   /**< Max Star Value (0 - 63) */

    /* IC4-IC7 */
    int8_t        betaMap0[8];    /**< Beta state values for MAP0 decoder */
    int8_t        betaMap1[8];    /**< Beta state values for MAP1 decoder */

    /* IC8-IC11 */
    uint8_t       extrScale[16];  /**< Extrinsic scale values */

    /* IC12-IC14 */
    uint16_t      itgParam[5];    /**< Interleaver Table Generation init params */

} Tcp3d_InCfgParams;

/**
 * @brief Structure to store TCP3 decoder specific values to identify its
 *          instance configuration information. This is created to facilitate
 *          the application to choose TCP3 decoder (either TCP3D_0 or TCP3D_1)
 *          for which the driver will be configured.
 * 
 *      This structure could be used for both PING and PONG specific values.
 */
typedef struct Tcp3d_Config
{
    uint32_t      inCfgStart; /**< input configuration registers start address */
    uint32_t      llrStart;   /**< input data (LLR) start address */
    uint32_t      interStart; /**< inter leaver start address */
    uint32_t      hdStart;    /**< output hard decision start address */
    uint32_t      sdStart;    /**< output soft decision start address */
    uint32_t      stsStart;   /**< output status registers start address */
    uint32_t      revtCh;     /**< Channel number associated with REVT */
}Tcp3d_Config;

/**
 * @brief TCP3D Driver instance structure
 */
typedef struct Tcp3d_Instance
{
    /** TCP3D Peripheral instance number */
    uint8_t                 instNum;

    /** Variable to keep the driver state */
    volatile Tcp3d_State    state;

    /** Driver operating Mode for the given instance */
    uint8_t                 mode;
    
    /** Double Buffer mode enable/disable */
    uint16_t                doubleBuffer;

    /** If true, PING path is stopped */
    volatile uint8_t        pingStop;
    
    /** If true, PONG path is stopped */
    volatile uint8_t        pongStop;

    /** TCP3D driver start mode flag. Set to NULL during init to disable the 
     * auto start function call from enqueue funciton until application
     * initiates.
     */
    uint8_t                 startFlag;

    /** CPU/DSP core ID on which this instance of driver is running */
    uint8_t                 coreId;

    /** Gives the number of free entries available in the input ping list for
     * enqueue. This flag is decremented when a code block is enqueued into
     * the ping list. It's value is updated in the start funciton. */
    int32_t                 pingFreeCnt;

    /** Gives the number of free entries available in the input pong list for
     * enqueue. This flag is decremented when a code block is enqueued into
     * the pong list. It's value is updated in the start funciton. */
    int32_t                 pongFreeCnt;

    /** Gives the Maximum number of code blocks that can be enqueued using the
     * driver. This value is set during the init. */
    uint32_t                maxCodeBlocks;

    /** Gives the next code block index for enqueue into the input list. */
    uint32_t                nextCodeBlockIndex;
    
    /** Pointer to the pseudo PaRAM buffer array base. */
    EDMA3_DRV_PaRAMRegs     *pseudoParamBufPtr;

    /** CP_INTC0 input event number used for the output notification. Driver
     * uses this value to write (using EDMA) into the STATUS_SET_INDEX_REG
     * during run-time to cause system event/interrupt. */
    uint32_t                notificationEventNum;

    /**
     *  CP_INTC0 register overlay base address.
     *  This is expected of type CSL_CPINTC_RegsOvly.
     */
    void                    *cpIntc0RegsBase;

    /* EDMA Variables */
    EDMA3_DRV_Handle        edmaHnd;        /**< EDMA3 LLD Driver Handle */
    uint32_t                edmaRegionId;   /**< EDMA shadow region number*/
    uint32_t                pingCh[TCP3D_DRV_MAX_CH_PER_PATH];
                                            /**< Ping channels stored here */
    uint32_t                pongCh[TCP3D_DRV_MAX_CH_PER_PATH];
                                            /**< Pong channels stored here */
    uint32_t                pingChParamAddr[TCP3D_DRV_MAX_CH_PER_PATH];
                                            /**< Physical PaRAM addresses of the 
                                                Ping channels */
    uint32_t                pongChParamAddr[TCP3D_DRV_MAX_CH_PER_PATH];
                                            /**< Physical PaRAM addresses of the 
                                                Pong channels */
    uint32_t                pingLinkCh[TCP3D_DRV_MAX_LINK_CH>>1];
                                            /**< Link channels for Ping path */
    uint32_t                pongLinkCh[TCP3D_DRV_MAX_LINK_CH>>1];
                                            /**< Link channels for Pong path */
    uint32_t                pingLinkChParamAddr[TCP3D_DRV_MAX_LINK_CH>>1];
                                            /**< Link channel PaRAM address for
                                             Ping path */
    uint32_t                pongLinkChParamAddr[TCP3D_DRV_MAX_LINK_CH>>1];
                                            /**< Link channel PaRAM address for
                                                    Pong path */

    /** bit masks used for controlling interrupt generation by EDMA CC */
    uint32_t                l2pChMaskPing;  /**< L2P Channel Mask for PING */
    uint32_t                l2pChMaskPong;  /**< L2P Channel Mask for PONG */
    uint32_t                pauseChMaskPing;  /**< REVT Channel Mask for PING */
    uint32_t                pauseChMaskPong;  /**< REVT Channel Mask for PING */

    /** EDMA shadow registers base address used during run-time */
    CSL_TPCC_ShadowRegs     *tpccShadowRegs;
    /** Register address of TPCC_IECR used for clearing (diable) the IER bits */
    uint32_t                *intEnClrReg[2];
    /** Register address of TPCC_IESR used for setting (enable) the IER bits */
    uint32_t                *intEnSetReg[2];
    /** Register address of TPCC_ICR used for clearing the pending IPR bits */
    uint32_t                *clrIntPendReg[2];
    /** Register address of TPCC_IPR used for checking pending interrupts */
    uint32_t                *intPendReg[2];

    /* Internal Variables (most of the names are self explanatory) */
    uint8_t                 constantOne;/**< variable set to 1 at init time and
                                        used by PAUSE channels */
    Tcp3d_State             pauseState; /**< variable set to TCP3D_DRV_STATE_PAUSE
                                        and used by PAUSE channels */
    uint32_t                resetHdOpt[2];
    uint32_t                resetHdLink[2];
    uint32_t                resetStsOpt[2];
    uint32_t                resetStsLink[2];
    uint32_t                resetSdOpt[2];
    uint32_t                resetSdLink[2];
    EDMA3_DRV_PaRAMRegs     *startPrmPtr;
    EDMA3_DRV_PaRAMRegs     *pingPtrL2p;
    EDMA3_DRV_PaRAMRegs     *pongPtrL2p;
    EDMA3_DRV_PaRAMRegs     revtPrm[2];
    EDMA3_DRV_PaRAMRegs     l2pPrm[2];
    EDMA3_DRV_PaRAMRegs     *lastParam[2];
    EDMA3_DRV_PaRAMRegs     *endListParam[2];
    uint32_t                prevNtfFlag[2];
    uint32_t                maxPingCbIdx;
    uint32_t                maxPongCbIdx;
    uint32_t                maxPingCbCnt;
    uint32_t                maxPongCbCnt;
    uint32_t                nextPingInIdx;
    uint32_t                nextPongInIdx;
    uint32_t                prevPingOutIdx;
    uint32_t                prevPongOutIdx;
    int32_t                 pingLoadCnt;
    int32_t                 pongLoadCnt;
    volatile uint32_t       pingLastOutFlag;
    volatile uint32_t       pongLastOutFlag;
    uint8_t                 pingWrapCheck;    
    uint8_t                 pongWrapCheck;    

    /* Debug Flags */
    volatile uint32_t       pingStartCntr;
    volatile uint32_t       pongStartCntr;
    volatile uint32_t       pingPauseEnCntr;
    volatile uint32_t       pingL2pEnCntr;
    volatile uint32_t       pingIntr;
    volatile uint32_t       pongIntr;

} Tcp3d_Instance;

/**
 * @brief   TCP3D Driver Initialization parameters structure
 */
typedef struct Tcp3d_InitParams
{
    /** 
     * TCP3D Peripheral instance number to setup. Possible values are
     *          CSL_TCP3D_0 or CSL_TCP3D_1.
     */
    uint8_t                   instNum;

    /**
     *  Maximum code blocks for which the driver resources will be 
     *          configured. This value is required to configure some parameters.
     */
    uint32_t                  maxCodeBlocks;

    /**
     *  Core Index value [0,1,2,3]. Used for traslating local L2
     *          addresses into global addresses used in the EDMA transfers.
     */
    uint8_t                   coreID;

    /**
     *  Control parameters for TCP3 decoder. All the elements in this
     *          structure must be filled with the required values with which
     *          the driver is intended to run.
     * 
     *          They are used to set the MODE, ENDIAN and EXECUTE control
     *          registers and also to control the driver code based on the
     *          configuration information like mode & double buffer.
     */
    Tcp3d_CtrlParams        ctrlParams;

    /**
     *  TCP3 decoder configuration registers start address. This is used
     *          only in the init function to start the TCP3 decoder state
     *          machine by writing into the control registers with the correct
     *          configuration values.
     */
    CSL_Tcp3d_cfgRegs       *tcp3dCfgRegs;

    /**
     *  Start addresses of PING (P0) memory area of TCP3 Decoder and
     *          the assiciated REVT channel number will be provided here.  
     *          This structure values must be filled with appropriate addresses 
     *          depending on the decoder instance.
     */
    Tcp3d_Config            pingConfig;

    /**
     *  Start addresses of PONG (P1) memory area of TCP3 Decoderand
     *          the assiciated REVT channel number will be provided here.
     *          This structure values must be filled with appropriate addresses 
     *          depending on the decoder instance.
     */
    Tcp3d_Config            pongConfig;

    /**
     *  EDMA3 LLD Driver handle used in the driver to call EDMA LLD
     *          driver funcitons.
     */
    EDMA3_DRV_Handle        edmaHnd;

    /**
     *  EDMA3 shadow region ID through which all the resources are
     *          allocated.
     */
    uint32_t                  edmaRegionId;

    /**
     *  Ping Channels Array. First channel must be "0" since it is tied 
     *          to the event (REVT0) generated from TCP3D and used for reading
     *          outputs from the decoder memory. Total physical channels 
     *          required is given by the define TCP3D_DRV_MAX_CH_PER_PATH.
     * 
     *          This arrary could be accesssed with the index values provided
     *          as define names starting with TCP3D_DRV_CH_IDX. This is useful
     *          for the application when it needs to setup a callback with
     *          specific Channel (TCC).
     */
    uint32_t                  pingCh[TCP3D_DRV_MAX_CH_PER_PATH];

    /**
     *  Pong Channels Array. First channel must be "1" since it is tied 
     *          to the event (REVT1) generated from TCP3D and used for reading
     *          outputs from the decoder memory.
     *
     *          Total physical channels required is given by the define 
     *          TCP3D_DRV_MAX_CH_PER_PATH.
     *   
     *          This arrary could be accesssed with the index values provided
     *          as define names starting with TCP3D_DRV_CH_IDX. This is useful
     *          for the application when it needs to setup a callback with
     *          specific Channel (TCC).
     */
    uint32_t                  pongCh[TCP3D_DRV_MAX_CH_PER_PATH];

    /**
     *  Link Channels Array.
     * 
     *          Total physical channels required is given by the define
     *          TCP3D_DRV_MAX_LINK_CH.
     * 
     * @note    It is required that all these link channels be consecutive in
     *          their PaRAM memory. 
     */
    uint32_t                  linkCh[TCP3D_DRV_MAX_LINK_CH];

    /**
     *  CP_INTC0 input event used for the output notification.
     */
    uint32_t                  notificationEventNum;

    /**
     *  CP_INTC0 register overlay base address.
     *  This is expected of type CSL_CPINTC_RegsOvly.
     */
    void                    *cpIntc0RegsBase;

    /**
     *  EDMA3 Channel Controller shadow register base address of the region
     *  (edmaRegionId) from where the resources were allocated.
     *  This is expected of type CSL_TPCC_ShadowRegs.
     */
    CSL_TPCC_ShadowRegs     *edma3ShadowRegsBase;

} Tcp3d_InitParams;

/**
 * @brief   TCP3D Driver Status structure.
 */ 
typedef struct Tcp3d_Sts
{
    Tcp3d_StsCmd    cmd;        /**< Command flag for Status query */
    Tcp3d_State     state;      /**< to keep the Driver state value */
    uint32_t          prmOutIdx;  /**< to keep the queried input pseudo PaRAM
                                    index value */
} Tcp3d_Sts;

/**
 * @brief   TCP3D Driver Control structure.
 */ 
typedef struct Tcp3d_Ctrl
{
    Tcp3d_CtrlCmd   cmd;            /**< Command flag for control operation */
    uint32_t          intrFlag;       /**< interrupt flag to enable or disable
                                        1 - for enable
                                        0 - for disable */
} Tcp3d_Ctrl;

/**
@}
*/
/* ========================================================================= */

/* ========================================================================= */
/**
 * Driver Function definitions
 */
/** @addtogroup TCP3D_DRV_FUNCTION
 @{ */

/**
 *  @b  Description
 *  @n  
 *              TCP3D Driver function for providing the number of buffers
 *              required.
 * 
 *  \param[in]      *cfg
 *              Pointer to the structure of type Tcp3d_SizeCfg which has
 *              specific information used for determining the buffer
 *              requirements.
 *
 *              For Example, TCP3D could use one field maxCodeBlocks to
 *              determine number of buffers required.
 * 
 *  \param[out]     *nbufs
 *              Pointer of a variable to which TCP3D Driver proveds the number
 *              of buffers required.
 * 
 *  \pre        Set the cfg->maxCodeBlocks value before calling this API.
 *
 *  \post       
 *
 *  \return     Status reported as either TCP3D_DRV_NO_ERR or TCP3D_DRV_FAIL.
 *
 */
Tcp3d_Result Tcp3d_getNumBuf (IN Tcp3d_SizeCfg  *cfg,
                              OUT int16_t         *nbufs);

/**
 *  @b  Description
 *  @n  
 *              TCP3D Driver function for providing the attributes of all the
 *              number of buffers requested through the structure of type
 *              Tcp3d_MemBuffer provided.
 * 
 *  \param[in]      *cfg
 *              Pointer to the structure of type Tcp3d_SizeCfg which has
 *              specific information used for determining the buffer
 *              requirements.
 *
 *              TCP3D Driver uses the field maxCodeBlocks for determining
 *              the buffer sizes required.
 * 
 *  \param[out]     **bufs
 *              Pointer to the array of Tcp3d_MemBuffer structure of size
 *              provided through the Tcp3d_getNumBuf() API.
 *
 *              TCP3D Driver fills all the fileds of the structure except the
 *              base which application fills after allocating the memory as per
 *              the attributes requested.
 * 
 *  \pre        Set the cfg->maxCodeBlocks value before calling this API.
 *
 *  \post       
 *
 *  \return     Status reported as either TCP3D_DRV_NO_ERR or TCP3D_DRV_FAIL.
 *
 */
Tcp3d_Result Tcp3d_getBufDesc ( IN  Tcp3d_SizeCfg       *cfg,
                                OUT Tcp3d_MemBuffer     bufs[]);

/**
 *  @b  Description
 *  @n  
 *              TCP3D Driver function called to reset the driver at any time
 *              after init and only if both PING & PONG decoders have nothing
 *              in the input list for decoding.
 * 
 *              This API checks if it is called in a correct state and returns
 *              appropriate error message.
 *
 *              This function does the following:
 *              1) Set the instance with the passed values - for example number
 *                  of blocks for decoding in the current subframe which is
 *                  needed for boundary checks and to setup the EDMA channels
 *                  and a new status array pointer where the status register
 *                  values for each code block are to be DMAed.
 *              2) Initialize all the run-time instance variables to default.
 *              3) Initialize the pseudo PaRAM memory with all the defaults
 *                  based on mode.
 *              4) Reset the EDMA channels with default values.
 *              5) Change the state to TCP3D_DRV_STATE_INIT.
 * 
 *  \param[in]      *tcp3dInst
 *              This is the driver instance.
 * 
 *  \param[in]     codeBlocks
 *              Number of code blocks to be decoded for the current sub-frame.
 *              This value should be equal to the maxCodeBlocks.
 *
 *              For saving cycles, you can set to lower value that should be
 *              multiple of 2.
 *
 *  \pre        Allocation of the statusBuf arrary must be big enough to fit
 *              all the register trasfers for the given codeBlocks. If the size
 *              is small, there will be memory step-over.
 *
 *  \post       
 *
 *  \retVal     Success - TCP3D_DRV_NO_ERR 
 *  \retVal     Failure - TCP3D_DRV_INVALID_STATE
 *  \retVal     Failure - TCP3D_DRV_INVALID_PARAMETER
 *
 */
Tcp3d_Result Tcp3d_reset (  IN Tcp3d_Instance  *tcp3dInst,
                            IN uint32_t          codeBlocks);

/**
 *  @b  Description
 *  @n  
 *              TCP3D Driver Initialization function which must be called only
 *              once to initialize the driver instance and other required
 *              resources needed for the driver functionality.
 * 
 *              This funciton does the following:
 *              -# First this function reads the bufs structure to get the base
 *                  addresses to set the instance and other strucures. If any
 *                  base address is NULL, driver exits immediately with error.
 *              -# Initializes all the run-time instance variables to default
 *                  values.
 *              -# All init-time variables are set either from the drvInitParams
 *                  input structure or set directly.
 *              -# Copies all the EDMA resource information into the instance.
 *                  - Does some sanity check on the channel numbers associated
 *                      with REVTs. 
 *                  - Enables all the EVENT triggered channels.
 *                  - Pre-fill most of the PaRAM entries for both the physical
 *                      and linked channels which will be used in run-time in
 *                      the enqueue funciton.
 *              -# Initializes the pseudo PaRAM buffer with fixed values.
 *              -# All required reset funcitons are called from this function
 *                  to eliminate the need to call the reset API immediately.
 *              -# Finally before exit, starts the TCP3 Decoder state machine by
 *                  writing into the control registers from the values provided
 *                  in the drvInitParams->ctrlParams structure.
 * 
 *  \param[in]      **bufs
 *              Pointer to the array of Tcp3d_MemBuffer structure of size
 *              provided through the Tcp3d_getNumBuf() API. This strucure must
 *              have all the fields filled by now. The bufs[0]->base value will
 *              be used for initializing the driver instance.
 *
 *  \param[in]      drvInitParams
 *              Driver initialization parameters structure. This structure is
 *              used for getting all the required resources for the driver to
 *              initialize the instance. Look into the description of the 
 *              Tcp3d_InitParams structure elements for details. 
 * 
 *  \pre          
 *
 *  \post       
 *
 *  \retVal     Success -   TCP3D_DRV_NO_ERR 
 *  \retVal     Failure -   TCP3D_DRV_INVALID_BUFF
 *  \retVal     Failure -   TCP3D_DRV_INVALID_EDMA_CH 
 *  \retVal     Failure -   TCP3D_DRV_FAIL_EDMA_GET_PARAM_ADDR
 *  \retVal     Failure -   TCP3D_DRV_FAIL_EDMA_PARAM_INIT
 *  \retVal     Failure -   TCP3D_DRV_FAIL_EDMA_ENABLE_CHANNEL
 *
 */
Tcp3d_Result Tcp3d_init(IN  Tcp3d_MemBuffer     bufs[],
                        IN  Tcp3d_InitParams    *drvInitParams);

/**
 *  @b  Description
 *  @n  
 *              TCP3D Driver De-Initialization function which must be called
 *              to close the corresponding instance.
 * 
 *              This funciton does the following:
 *              -# First this function clears any EDMA specific registers set.
 * 
 *  \param [in]     *tcp3dInst
 *              This is the driver instance.
 *
 *  \pre          
 *
 *  \post       Application can free the resources and memory allocated for this
 *              instance after this function.
 *
 *  \retVal     Success -   TCP3D_DRV_NO_ERR 
 *  \retVal     Failure -   TCP3D_DRV_FAIL
 *
 */
Tcp3d_Result Tcp3d_deInit(IN  Tcp3d_Instance  *tcp3dInst);

/**
 *  @b  Description
 *  @n  
 *              This is a run-time API for appending the codeblocks to the
 *              pseudo PaRAM list as the blocks arrive. This function updates
 *              the pseudo PaRAM set entries and chains to the previous block
 *              in the corresponding list (PING or PONG).
 * 
 *              Enqueuing is done alternately between the two lists until the
 *              current list is full. The no room case is indicated with return
 *              error TCP3D_DRV_INPUT_LIST_FULL.
 * 
 *              On each succeesful enqueue, the corresponding load counter for
 *              the path is incremented. This funciton updates some run-time
 *              varaibles which keeps track of ping and pong list indexes
 *              and counters.
 * 
 *              This function primarily does the following:
 *              -# Updates the pseudo PaRAM set with the addresses passed as 
 *                  paramters. Also, updates necessary count and index values
 *                  within the pseudo PaRAM set which depend on the block size
 *                  and are not pre-filled during init.
 *              -# Links the optional outputs, if present, to the HD param and
 *                  sets necessary TCC values.
 *              -# If the ntfEventFlag is set for notification, the NTF Param is
 *                  linked with the last param from the set.
 *              -# Chaining to the previous block in the corresponding list is
 *                  done, execept for the first one in the list. This is done by
 *                  - Changing the TCC field of the last param in the previous
 *                      block to trigger the L2P channel.
 *                  - Updating the link field with the dummy or NTFD depending
 *                      on whether the previous block has notification or not.
 *              -# Calls the Tcp3d_start() function with auto mode as needed
 *                  after the application initiated the first start.
 *
 *              This function is executed at the application task thread for
 *              queueing the code blocks to the input pseudo PaRAM list. They
 *              will be copied to the actual PaRAM area using the L2P channel
 *              when the previous block is decoded and outputs are read as
 *              specified.
 * 
 *              Application need to check the return value to take appropriate
 *              action.
 *              -# If no error try enqueuing next block.
 *              -# If LIST FULL error, either wait for some time and try
 *                  enqueuing or enable EDMA completion interrupts for L2P
 *                  channels. Upon receiving the interrupt the enqueuing could
 *                  be tried again.
 * 
 *              Since the TCC for L2P channels always point to REVT, enabling
 *              L2P channel interrupts would showup as REVT channel call backs.
 *
 *  \param [in]     *tcp3dInst
 *              This is the driver instance.
 *
 *  \param [in]     blockLength
 *              Code Block length value which is used for setting some of the
 *              PaRAM counts.
 *
 *  \param [in]     *inputConfigPtr
 *              Pointer to the input config registers prepared for the current
 *              code block.
 *
 *  \param [in]     *llrPtr
 *              Input data pointer where the systematic, parity 1 and parity 2
 *              streams are available.
 *
 *  \param [in]     llrOffset
 *              Input data offset between the three streams. Used for setting
 *              the EDMA trasfer type (A-Sync or AB-Sync) and also to set the
 *              source jump index.
 *
 *  \param [in]     *hdPtr
 *              Pointer to the memory where the decoded hard decision values to
 *              be DMAed.
 *
 *  \param [in]     *statusPtr
 *              Pointer to the memory to DMA the three status register values.
 *              
 *              This is a selective parameter which application uses only if the
 *              corresponding bit is set in the inputConfigRegs for reading the
 *              TCP3D output memory.
 * 
 *              If not used, set to @b NULL to avoid any undesired behaviour
 *              from the TCP3 decoder. This will also save the time for
 *              programming.
 *
 *  \param [in]     *sdPtr
 *              Pointer to the memory where the decoded soft decision values to
 *              be DMAed.
 * 
 *              This is a selective parameter which application uses only if the
 *              corresponding bit is set in the inputConfigRegs for reading the
 *              TCP3D output memory.
 * 
 *              If not used, set to @b NULL to avoid any undesired behaviour
 *              from the TCP3 decoder. This will also save the time for
 *              programming.
 *
 *  \param [in]     sdOffset
 *              Output data offset between the three streams. Not valid for
 *              split more, since there will be only one stream.
 * 
 *              If not used, set to @b NULL to avoid any undesired behaviour
 *              from the TCP3 decoder.
 *
 *  \param [in]     ntfEventFlag
 *              Flag for enabling the notification event to be generated at
 *              the end of output transfer completion of the code block. When
 *              set, the event number provided during the init.
 *  
 *              Useful for application to get notification after a specific
 *              code block is decoded to start the post-processing.
 *
 *  \pre        This driver API MUST be called only after the Tcp3d_init()
 *              function is called.
 *
 *  \post       Driver state value could be changed to TCP3D_DRV_STATE_RUNNING
 *              from TCP3D_DRV_STATE_PAUSE. This happens only if the startFlag
 *              was set by this time.
 *
 *  \retVal     Success -   TCP3D_DRV_NO_ERR
 *  \retVal     Failure -   TCP3D_DRV_INPUT_LIST_FULL
 */
Tcp3d_Result Tcp3d_enqueueCodeBlock(IN  Tcp3d_Instance  *tcp3dInst,
                                    IN  uint32_t          blockLength,
                                    IN  uint32_t          *inputConfigPtr,
                                    IN  int8_t            *llrPtr,
                                    IN  uint32_t          llrOffset,
                                    IN  uint32_t          *hdPtr,
                                    IN  int8_t            *sdPtr,
                                    IN  uint32_t          sdOffset,
                                    IN  uint32_t          *statusPtr,
                                    IN  uint8_t           ntfEventFlag);

/**
 *  @b  Description
 *  @n  
 *              This API could be used for starting the driver to start doing
 *              EDMA transfers to TCP3 decoder for decoding from the pseudo
 *              PaRAM list.
 * 
 *              This function updates the load and free counts for both ping
 *              and pong lists.
 *
 *              This function is executed at the application task thread for
 *              starting either the PING or PONG path execution.
 * 
 *  \param [in]     *inst
 *              This is the driver instance. 
 *
 *  \param [in]     startMode
 *              Flag tells the mode of operation.
 *          -# If set to TCP3D_DRV_START_AUTO, function checks both the paths
 *              (PING & PONG) and start the needed paths.
 *          -# If set with either TCP3D_DRV_START_PING or TCP3D_DRV_START_PONG,
 *              and the corresponding flag (pingStop or pongStop) is set then
 *              that path alone is started.
 *
 *  \pre        Application must have atleast one code block enqueued to the 
 *              pseudo PaRAM list before calling this API so that driver will
 *              have something to start the EDMA trasfers and decoding before
 *              going to PAUSE state.
 * 
 *              This function should be called only after the Tcp3d_init()
 *              function is called.
 *
 *  \post       Driver state variable is updated, if any paths are started.
 *
 *  \retVal     Success -   TCP3D_DRV_NO_ERR
 *  \retVal     Failure -   TCP3D_DRV_FAIL_EDMA_ENABLE_CHANNEL
 *
 */
Tcp3d_Result Tcp3d_start (  IN  Tcp3d_Instance  *inst,
                            IN  uint8_t           startMode);

/**
 *  @b  Description
 *  @n  
 *              This API could be used for querying the TCP3D driver to get
 *              updates or take appropriate actions. The Tcp3d_StsCmd enum
 *              structure gives the supported commads.
 * 
 *  \note       This API is not fully scoped and the possible query commands
 *              could be extended in future.
 * 
 *  \param[in]      *inst
 *              This is the driver instance for which status query is made.
 *
 *  \param[in,out]  *drvStatus
 *              Structure used for keeping the status request command and 
 *              for keeping the status query output.
 *
 *  \pre        Set appropriate command value before calling the function.
 * 
 *  \post       
 *
 *  \retVal     Success -   TCP3D_DRV_NO_ERR (valid command)
 *  \retVal     Failure -   TCP3D_DRV_FAIL (invalid command)
 *
 */
Tcp3d_Result Tcp3d_status ( IN    Tcp3d_Instance    *inst,
                            INOUT Tcp3d_Sts         *drvStatus );

/**
 *  @b  Description
 *  @n  
 *              This API could be used to change or update the TCP3D driver
 *              instance values which are set during the init time. The
 *              Tcp3d_CtrlCmd enum structure gives the supported commads.
 *  
 *  \note       -# This API is not fully scoped and the possible control
 *              commands could be extended in future.
 *              -# We may need to protect the instance value updations, once
 *              they are allowed to change in any state.
 * 
 *  \param[in]      *inst
 *              This is the driver instance.
 *
 *  \param[in]      *drvCtrl
 *              Structure used for keeping the control request command and
 *              also for passing any input values along with as required. 
 *
 *  \pre        Set appropriate command value before calling the function.
 * 
 *  \post       Depending on the command the driver behaviour would be changed. 
 *
 *  \retVal     Success -   TCP3D_DRV_NO_ERR (valid command)
 *  \retVal     Failure -   TCP3D_DRV_FAIL (invalid command)
 *
 */
Tcp3d_Result Tcp3d_control (IN Tcp3d_Instance   *inst,
                            IN Tcp3d_Ctrl       *drvCtrl );

/**
@}
*/
/* ========================================================================= */

/* ========================================================================= */
/**
 * Utility Function definitions
 *  - Register Preparation Function definitions
 *  - Any other functions
 */

/** @addtogroup TCP3D_DRV_UTIL_FUNCTION
 @{ */

/**
 *  @b  Description
 *  @n  
 *              TCP3D Driver function for preparing the common control registers
 *              from the input structure parameters using the CSL_FINS macro.
 * 
 *              The outputs could be used to write into the actual TCP3 decoder
 *              memory registers directly or DMAed to bring the TCP3 decoder
 *              state machine to WAIT for inputs state.
 * 
 *  \param[in]      *ctrl
 *              Pointer to structure of type Tcp3d_CtrlParams for providing
 *              the input parameters for the control variables.
 *
 *  \param[out]     *modeReg
 *              Pointer to the mode register variable to put the prepared value. 
 * 
 *  \param[out]     *endReg
 *              Pointer to the endian register variable for placing the
 *              prepared value. 
 * 
 *  \param[out]     *exeRegP0
 *              Pointer to the process 0 execution register variable for
 *              placing the prepared value. 
 * 
 *  \param[out]     *exeRegP1
 *              Pointer to the process 1 execution register variable for
 *              placing the prepared value.
 * 
 *  \pre        All the parameters in the input ctrl structure must be set
 *              before calling this API. Read the Tcp3d_CtrlParams structure
 *              description to see if some parameters are reserved for future
 *              use in which case they need not be set.  
 *
 *  \post       
 *
 *  \return     
 * 
 */
void Tcp3d_prepControlRegs( IN  Tcp3d_CtrlParams    *ctrl,
                                OUT uint32_t              *modeReg,
                                OUT uint32_t              *endReg,
                                OUT uint32_t              *exeRegP0,
                                OUT uint32_t              *exeRegP1);

/**
 *  @b  Description
 *  @n  
 *              This is a utility function provided as part of TCP3D Driver for
 *              preparing a fixed set of input config registers that would be
 *              fixed for a typical configuration and will not vary from 
 *              code block to code block.
 * 
 *              This function is used for preparing IC2, IC3, IC8-IC11 registers
 *              only of 15 registers (IC0-IC14) using CSL_FINS macro.
 * 
 *              The output outICRegs could be used as template IC registers
 *              array when preparing the input config registers for code blocks.
 * 
 *  \param[in]      *inCfgParams
 *              Pointer to structure of input parameters of type
 *              Tcp3d_InCfgParams for preparing IC0-IC14 (15 registers).
 *
 *  \param[out]     *outICRegs
 *              Pointer to the array for holding the 15 registers memory.
 *              @b Note that only the relavent registers are updated.  
 * 
 *  \pre        Parameters required for preparing fixed registers (IC2, IC3,
 *              IC8-IC11) must be set in the input parameters structure
 *              before calling this API.
 * 
 *              Read the Tcp3d_InCfgParams structure description to see details
 *              on which parameters are needed.
 *
 *  \post       
 *
 *  \return     
 * 
 */
void Tcp3d_prepFixedConfigRegs ( 
                            IN  Tcp3d_InCfgParams * const RESTRICT inCfgParams,
                            OUT uint32_t            * const RESTRICT outICRegs);

/**
 *  @b  Description
 *  @n  
 *              This is a utility function provided as part of TCP3D Driver for
 *              preparing the input config registers that will be used for
 *              sending to TCP3 decoder IP memory before sending the LLR data.
 * 
 *              This function is used for preparing all the 15 input config
 *              registers (IC0-IC14) using CSL_FINS macro.
 * 
 *              This function along with Tcp3d_prepFixedConfigRegs provides
 *              an otimization knowing that some of the registers will not
 *              change for each block in general. The last two paramaters are
 *              provided to use this feature. Once the fixed register fields
 *              are known, Tcp3d_prepFixedConfigRegs funciton could be
 *              called for preparing the fixed registers and get the tempICRegs
 *              array ready in advance sometime during init time. This array
 *              could be supplied with the Tcp3d_prepConfigRegs API along
 *              with a flag to copy the fixed registers from the array instead
 *              of preparing.
 * 
 *  \param[in]      mode
 *              Mode of the TCP3D IP block used for determing what to fill in to
 *              IC12, IC13, IC14 registers (Initial ITG Param values).
 *              These registers must be prepared only for LTE and WIMAX case,
 *              otherwise set to ZERO always.
 * 
 *  \param[in]      *inCfgParams
 *              Pointer to structure of input parameters of type
 *              Tcp3d_InCfgParams for preparing IC0-IC14 (15 registers).
 *
 *  \param[out]     *outICRegs
 *              Pointer to the memory array for holding the fully prepared
 *              registers, ready to be copied to TCP3 decoder IP memory.
 * 
 *  \param[in]      *tempICRegs
 *              Template memory array pointer with fixed registers prepared.
 *              This array should have been prepared one time by calling the
 *              Tcp3d_prepFixedConfigRegs function well in advance.
 *              Part of this array will be used only if the copyFixedReg flag
 *              parameter is set to non-zero value. 
 * 
 *  \param[in]      copyFixedReg
 *              Flag to tell whether to use the tempICRegs array to copy the
 *              fixed input config registers or to prepare all of them in this
 *              function.
 * 
 *  \pre        All the parameters in the input structure param must be set
 *              before calling this API.
 *
 *              You can avoid setting some of the register parameters, if you
 *              use the optimization trick as described in the description. In
 *              that case, the fixed register parameterss need not be set.
 *
 *              Read the Tcp3d_InCfgParams structure description to see details
 *              on which parameters are needed.
 *
 *  \post       
 *
 *  \return     
 * 
 *  \ref        Tcp3d_prepFixedConfigRegs
 *
 */
void Tcp3d_prepConfigRegs (
                            IN  uint8_t                              mode,
                            IN  Tcp3d_InCfgParams * const RESTRICT inCfgParams,
                            OUT uint32_t            * const RESTRICT outICRegs,
                            IN  uint32_t            * const RESTRICT tempICRegs,
                            IN  uint8_t                              copyFixedReg);

/**
 *  @b  Description
 *  @n  
 *              This is a utility function is provided as part of TCP3D Driver
 *              for preparing the specific input config registers which depend
 *              on the block size.
 * 
 *              This function can be used for preparing IC0, IC1, IC12-IC14
 *              registers only out of 15 registers (IC0-IC14) using 
 *              CSL_FINS macro.
 *
 *              This function is called per code block.
 * 
 *  \param[in]      mode
 *              Driver mode of operation.
 *
 *  \param[out]     *outICRegs
 *              Pointer to the array for holding the 15 registers memory.
 *              @b Note that only the relavent registers are updated.  
 * 
 *  \param[in]      numsw0
 *              Number of SW0 used in the decoder.
 *
 *  \param[in]      blockLen
 *              Block length value as required to be populated in the registers.
 *
 *  \param[in]      sw0LenSel
 *              The value of this parameter depends on the actual SW0 length
 *              used and the possible values are described below.
 *                  0 – 16 bits
 *                  1 – 32 bits
 *                  2 – 48 bits
 *                  3 – 64 bits
 *                  4 – 96 bits
 *                  5 – 128 bits
 *
 *  \param[in]      sw1Len
 *              The value of this parameter depends on the actual SW1 length
 *              used and the possible values are described below.
 *                  9 – 10 bits
 *                  10 – 11 bits
 *                  11 – 12 bits
 *                  ...
 *                  127 – 128 bits
 *
 *  \param[in]      sw2LenSel
 *              The value of this parameter depends on the actual SW1 length
 *              used and the possible values are described below.
 *                  0 – SW2 is not present
 *                  1 – SW2 length is same as SW1
 *                  2 – SW2 length is less by 2 bits from SW1
 *
 *  \param[in]      *itgParam
 *              Interleaver Table Generation init params.
 *
 *  \pre        Input Parameters required for this function must be computed
 *              as per the guildelines given in the user guide for preparing
 *              these specific registers (IC0, IC1, IC12-IC14).
 * 
 *  \Note       The following rules must be followed when programming sliding
 *              window values, otherwise expect unpredictable results.
 *          1.  SW0 length >= SW1 length; If num_sw0 > 0
 *          2.  SW1 length >= 10
 *          3.  If (SW0 length - SW1 length  < 4 and SW1 length != SW2 length )
 *                  then SW2 length must = 0 (off)
 *          4.  K <= N * 128 * SW0 Nominal length
 * 
 *          where K = block Length and N = Number of MAP decoders
 *                                      ( 2 - LTE/WiMAX mode )
 *                                      ( 1 - WCDMA mode )
 *
 *  \post       
 *
 *  \return     
 * 
 */
void Tcp3d_prepBlockSizeDepConfigRegs ( IN  uint8_t                   mode,
                                        OUT uint32_t * const RESTRICT outICRegs,
                                        IN  uint8_t                   numsw0,
                                        IN  uint16_t                  blockLen,
                                        IN  uint8_t                   sw0LenSel,
                                        IN  uint8_t                   sw2LenSel,
                                        IN  uint8_t                   sw1Len,
                                        IN  uint16_t * const RESTRICT itgParam);

/**
 *  @b  Description
 *  @n  
 *              This is a utility function is provided as part of TCP3D Driver
 *              for preparing the beta state value dependent input config
 *              registers only.
 * 
 *              This function can be used for preparing IC4-IC7 registers only
 *              out of 15 registers (IC0-IC14) using CSL_FINS macro.
 * 
 *  \param[in]      mode
 *              Driver mode of operation.
 *
 *  \param[out]     *outICRegs
 *              Pointer to the array for holding the 15 registers memory.
 *              @b Note that only the relavent registers are updated.  
 * 
 *  \param[in]      *betaMap0
 *              Beta state values for MAP0 decoder.
 * 
 *  \param[in]      *betaMap1
 *              Beta state values for MAP1 decoder.
 * 
 *  \pre        Input Parameters required for this function must be computed
 *              as per the guildelines given in the user guide for preparing
 *              these specific registers (IC4-IC7).
 * 
 *  \post       
 *
 *  \return     
 * 
 */
void Tcp3d_prepBetaStateConfigRegs( IN  uint8_t                   mode,
                                    OUT uint32_t * const RESTRICT outICRegs,
                                    IN  int8_t   * const RESTRICT betaMap0,
                                    IN  int8_t   * const RESTRICT betaMap1);

/** 
 *  @b  Description
 *  @n  
 *              Calculates initial beta state values using the tail bits that
 *              could be used in preparing the TCP3D input configuration
 *              registers.
 *           
 *  \param[in]  tailBits
 *              Tail Bits buffer of size 12. The tail bits are expected to be
 *              in the order Xt1[0],Pt1[0],Xt1[1],Pt1[1],Xt1[2],Pt1[2],Xt2[0],
 *              Pt2[0],Xt2[1],Pt2[1],Xt2[2],Pt2[2].
 *
 *  \param[in]  signChange
 *              For sign inversion information.
 *                  1 - the sign of the outputs changed
 *                  0 - the output sign is unchanged.
 *  
 *  \param[out] Kt
 *              Number of trellis stages used to calculate initial beta states.
 *              This values is computed using the formula [3 - (Kext-K)],
 *              where K is the code block length.
 *  
 *  \param[out] beta0Ptr
 *              Initial beta state values for the MAP0 decoder computed from
 *              the tail bits. The buffer size is 8.
 *
 *  \param[out] beta1Ptr
 *              Initial beta state values for the MAP1 decoder computed from
 *              the tail bits. The buffer size is 8.
 *
 *  \return
 */
void Tcp3d_betaStates(  IN  int8_t    * const RESTRICT tailBits,
                        IN  int32_t   signChange,
                        IN  int32_t   Kt,
                        OUT int8_t    * const RESTRICT beta0Ptr,
                        OUT int8_t    * const RESTRICT beta1Ptr);

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version information of the TCP3D Driver.
 *
 *  @retval
 *      Version Information.
 */
uint32_t Tcp3d_getVersion (void);

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version string for the TCP3D Driver.
 *
 *  @retval
 *      Version String.
 */
const char* Tcp3d_getVersionStr (void);

/**
@}
*/
/* ========================================================================= */

#ifdef __cplusplus
}
#endif

#endif /* _TCP3D_DRV_H_ */
