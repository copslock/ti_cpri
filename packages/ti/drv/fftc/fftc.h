/** 
 *   @file  fftc.h
 *
 *   @brief  
 *      Header file with data structure and API declarations for FFTC driver.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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
#ifndef _FFTC_H_
#define _FFTC_H_

/* FFTC LLD include */
#include <ti/drv/fftc/fftc_lld.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* CHIP module include */
#include <ti/csl/csl_chip.h>

/* FFTC driver version include */
#include <ti/drv/fftc/fftcver.h>

/** @defgroup FFTC_API FFTC Higher Layer Data Structures & APIs
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *      The FFTC Driver is divided into 2 layers:
 *          -#  Low Level Driver APIs
 *          -#  High Level APIs
 *      
 *      The Low Level Driver APIs provide FFTC MMR access by 
 *      exporting register read/write APIs and also provides
 *      some useful helper APIs in putting together FFTC control
 *      header, DFT size list etc required by the hardware.
 *
 *      The FFTC Higher Layer provide APIs useful in submitting
 *      FFT requests and retrieving their results from the FFTC 
 *      engine without having to know all the details of the 
 *      CPPI/QMSS. 
 *
 *      This section of documentation covers the Higher Layer
 *      APIs and data structures.
 */

/**
@defgroup FFTC_SYMBOL  FFTC Higher Layer Symbols Defined
@ingroup FFTC_API
*/

/**
@defgroup FFTC_DATASTRUCT  FFTC Higher Layer Data Structures
@ingroup FFTC_API
*/

/**
@defgroup FFTC_FUNCTION  FFTC Higher Layer Functions
@ingroup FFTC_API
*/

/**
@defgroup FFTC_OSAL_FUNCTION  FFTC OS Abstraction Layer Functions
@ingroup FFTC_API
*/

#ifdef __cplusplus
extern "C"
{
#endif

/**
@addtogroup FFTC_SYMBOL
@{
*/

/** @brief
 * The number of FFTC instances in the SoC.
 */        
/* CSL does not declare the right number of FFTC instances for 
 * K2H device, so correcting here for the time being.
 */
#ifdef DEVICE_K2H     
#define     FFTC_MAX_NUM_INSTANCES                   4
#else
#define     FFTC_MAX_NUM_INSTANCES                  (CSL_FFTC_PER_CNT)
#endif

/** @brief
 * The number of CPPI flows reserved for each FFTC instance.
 */
#define     FFTC_MAX_NUM_FLOWS                      (8)

/** @brief
 * The maximum number of free queues that can be
 * created by the FFTC driver for use by various flows.
 *
 * Each of the free queues can hold descriptors of a certain
 * size allocated at FFTC open.
 */
#define     FFTC_MAX_NUM_FREEQ                      (8)

/** @brief
 *  The maximum number of Rx objects that will be tracked
 *  by the driver globally in the system across all cores.
 */
#define     FFTC_MAX_NUM_RXOBJECTS                  (32)        

/** @brief  
 *
 *  When returned this, indicates the API succeeded in 
 *  its intent.
 */
#define     FFTC_RETVAL_SUCCESS                     (0)

/** @brief  
 *
 *  When returned this, indicates that the handle passed
 *  to the API was invalid.
 */
#define     FFTC_RETVAL_EBAD_HANDLE                 (-1)

/** @brief  
 *
 *  When returned this, indicates that the input configuration
 *  passed to the API is invalid.
 */
#define     FFTC_RETVAL_EINVALID_PARAMS             (-2)

/** @brief  
 *
 *  When returned this, indicates that the API's attempt to
 *  allocate memory or retrieve descriptors failed and 
 *  is out of memory/descriptors.
 */
#define     FFTC_RETVAL_ENO_MEM                     (-3)

/** @brief  
 *
 *  When returned this, indicates that there is no pending
 *  FFT result available for the application's Rx object.
 */
#define     FFTC_RETVAL_ENO_RESULT                  (-4)

/** @brief  
 *
 *  When returned this, indicates a flow id/Rx object Id mismatch, 
 *  i.e., the destination Id received in the packet doesnt match
 *  the flow/Rx object on which it was actually received.
 */
#define     FFTC_RETVAL_EINV_DESTID_RESULT          (-5)

/** @brief  
 *
 *  When returned this, indicates a general error.
 */
#define     FFTC_RETVAL_EFAILURE                    (-6)

/**
@}
*/

/** @addtogroup FFTC_DATASTRUCT
 @{ */

/** 
 *  @brief  Fftc_DrvHandle
 *          
 *          FFTC driver handle.
 */    
typedef     void*               Fftc_DrvHandle;

/** 
 *  @brief  Fftc_TxHandle
 *          
 *          FFTC Tx object handle.
 */    
typedef     void*               Fftc_TxHandle;

/** 
 *  @brief  Fftc_RxHandle
 *          
 *          FFTC Rx object handle.
 */    
typedef     void*               Fftc_RxHandle;

/** 
 *  @brief  Fftc_RequestHandle
 *          
 *          FFTC Request object handle.
 */    
typedef     void*               Fftc_RequestHandle;

/** 
 *  @brief  Fftc_ResultHandle
 *          
 *          FFTC Result object handle.
 */    
typedef     void*               Fftc_ResultHandle;

/** 
 *  @brief  Fftc_RetVal
 *          
 *          Holds the various error codes reported by 
 *          the FFTC driver.
 *
 *          Please refer to FFTC_RETVAL_* definitions for 
 *          the various error codes reported by this driver.
 */    
typedef     int32_t             Fftc_RetVal;

/** 
 *  @brief  Fftc_DeviceCfg
 *
 *          FFTC device (SoC) specific information.
 *          
 *          Holds all the SoC specific info for any given FFTC
 *          peripheral instance.
 */    
typedef struct  _Fftc_DeviceCfg
{
    /** CPDMA Number corresponding to this FFTC peripheral instance. */
    Cppi_CpDma                  cpdmaNum;

    /** Tx queue number base for this FFTC peripheral instance. */
    uint32_t                    baseQueueNum;

    /** Base address for FFTC configuration registers for this instance */
    void*                       cfgRegs;
} Fftc_DeviceCfg;

/** 
 *  @brief  Fftc_BlockInfo
 *
 *          Structure to specify/hold the DFT block size 
 *          information relevant to an FFTC request/result.
 */     
typedef struct _Fftc_BlockInfo
{
    /** Boolean flag, When set to 1 indicates that all the
     *  DFT blocks of this request/result buffer are of the 
     *  same size and only blockSizes[0] is to be read
     *  to obtain the block size. If not set to 1, indicates
     *  that the buffer contains various DFT blocks of 
     *  mixed sizes.
     */
    uint8_t                     bIsEqualSize;        

    /** An array of DFT block sizes */
    uint16_t*                   blockSizes;

    /** Number of DFT blocks for which the block
     *  size information holds good.
     *
     *  This must never exceed FFTC_MAX_NUM_BLOCKS (128).
     */
    uint8_t                     numBlocks;
} Fftc_BlockInfo;

/** 
 *  @brief  Fftc_BlockResult
 *
 *          Structure to hold the FFT/IFT result on a per
 *          DFT block basis.
 */     
typedef struct _Fftc_BlockResult
{
    /** Result buffer pointer */        
    uint8_t*                    pBuffer;

    /** Result buffer length */
    uint32_t                    bufferLen;

    /** Block Exponent value detected for this FFT block */
    uint16_t                    blockExponentVal;
} Fftc_BlockResult;

/** 
 *  @brief  Fftc_Result
 *
 *          Structure to hold the FFT/IFFT result. 
 */     
typedef struct  _Fftc_Result
{
    /** Flow Id read from the result buffer */                
    uint8_t                     flowId;

    /** Source Id read from the result buffer */                
    uint8_t                     srcId;

    /** Destination Tag read from the result buffer */
    uint16_t                    destnTagInfo;

    /** Boolean flag to indicate if any error was detected while
     *  processing the FFT request.
     */
    uint8_t                     bErrorDetected;

    /** Boolean flag to indicate if a clipping error was detected
     *  while processing any of the blocks corresponding to this
     *  result.
     */
    uint8_t                     bClippingDetected;

    /** Number of FFT blocks in the result buffer */
    uint32_t                    numFFTCBlocks;

    /** FFT/IFT result for each of the blocks submitted */
    Fftc_BlockResult            fftBlockResult [FFTC_MAX_NUM_BLOCKS];                 
} Fftc_Result;

/** 
 *  @brief  Fftc_CppiDescCfg
 *
 *          Structure to specify the CPPI descriptor configuration
 *          for a FFTC free queue used to hold pre-allocated
 *          buffers.
 */    
typedef struct  _Fftc_CppiDescCfg
{
    /** CPPI Memory region to be used for this set of free descriptors. */        
    uint32_t                    descMemRegion;

    /** Number of CPPI free descriptors to allocate */        
    uint32_t                    numDesc;

    /** Size of CPPI free descriptors to allocate */        
    uint32_t                    descSize;    

    /** CPPI Descriptor Type.
     *
     *  Valid values are:
     *      Cppi_DescType_HOST,
     *      Cppi_DescType_MONOLITHIC
     */
    Cppi_DescType               descType;
} Fftc_CppiDescCfg;

/** 
 *  @brief  Fftc_InterruptCfg
 *
 *          Interrupt configuration parameters.
 *
 *          Holds Driver defined QMSS accumulator configuration.
 *          
 *          An QM interrupt is generated when either of the 
 *          following conditions is met:
 *
 *          (1) With Pacing enabled ('bEnablePacing' = 1), when
 *          there is at least one result entry available and the
 *          pacing timer configured in 'pacingFrequency' expires.
 *
 *          (2) With Pacing disabled ('bEnablePacing' = 0), when 
 *          the number of result entries accumulated equals the
 *          threshold configured here in 'intThreshold'.
 */    
typedef struct  _Fftc_InterruptCfg
{
    /** Boolean flag to turn interrupt pacing on/off */
    uint8_t                     bEnablePacing;

    /** Number of 25us ticks to pace the interrupt at.
     *
     *  Valid only if 'bEnablePacing' set to 1.
     */
    uint16_t                    pacingFrequency;

    /** Interrupt pacing mode. Specifies when the interrupt should 
     *  be trigerred.
     *
     *  Valid only if 'bEnablePacing' set to 1.
     */
    Qmss_AccPacingMode          pacingMode;
    
    /** Interrupt threshold value.
     *
     *  Maximum number of result entries (descriptors)
     *  to accumulate before an interrupt is generated.
    */
    uint16_t                    intThreshold;

    /** High priority accumulator channel number to use */
    uint8_t                     accChannelNum;
} Fftc_InterruptCfg;

/** 
 *  @brief  Fftc_DrvCfg
 *
 *          FFTC Driver configuration.
 *          
 *          Holds all the application's descriptor configuration 
 *          required by the driver at Init time.
 */    
typedef struct  _Fftc_DrvCfg
{
    /** Number of free queues to maintain for this application
     *  by the FFTC driver.
     *
     *  Can be used either for Tx or Rx free descriptors.
     */
    uint32_t                    cppiNumFreeDescCfg;

    /** Application's Tx/Rx free descriptor configuration.
     *
     *  Can be used either for Tx or Rx descriptors.
     */
    Fftc_CppiDescCfg            cppiFreeDescCfg [FFTC_MAX_NUM_FREEQ];
} Fftc_DrvCfg;


/** 
 *  @brief  Fftc_TxCfg
 *
 *          FFTC Transmit configuration.
 *          
 *          Used to specify the Tx characteristics
 *          of an application that wishes to submit FFT
 *          requests using the driver.
 */    
typedef struct _Fftc_TxCfg
{
    /** FFTC Tx queue number using which the FFT/IFFT request 
     *  must be submitted.
     */        
    Fftc_QueueId                txQNum;

    /** Boolean flag to indicate whether the Tx queue used
     *  could be shared by other applications too or if it is 
     *  to be exclusively used by this application only.
     *
     *  When this flag is set to 0, the driver ensures that
     *  no other application uses the same FFTC transmit
     *  queue and also puts additional checks to ensure that
     *  the configuration for this queue cannot be overridden
     *  using CPPI packets on the data path. 
     *
     *  When set to 1, the queue is opened in shared mode and its 
     *  configuration can be changed using CPPI packets by 
     *  multiple applications from any core.
     */
    uint8_t                     bSharedMode;

    /** Configuration structure that holds the settings for
     *  various FFTC Queue local configuration registers.
     */
    Fftc_QLocalCfg              fftQCfg;

    /** Boolean flag to indicate whether transmit free descriptors 
     *  and buffers should be allocated by the driver.
     *
     *  Set to 1 to let FFTC allocate a Tx free descriptor queue 0 
     *  otherwise. If set to 1, the FFTC driver pre-allocates
     *  request buffers and descriptors for Tx at Tx open time
     *  and manages them.
     */
    uint8_t                     bManageReqBuffers;

    /** Boolean flag to indicate if the application would like to 
     *  specify the DFT block size list configuration when sending 
     *  data using this Tx object.
     *
     *  Set to:
     *  0   -   if no DFT size list is going to be specified using 
     *          this Tx object.
     *
     *  1   -   if a DFT size list will be specified when using this
     *          Tx object.
     *
     *  This value is only used if 'bManageReqBuffers' is set to 1, i.e.
     *  if the FFTC driver is pre-allocating Tx free descriptors with 
     *  buffers.
     */
    uint8_t                     bEnableDftSizeListCfg;

    /** The maximum number of DFT block sizes that will be
     *  specified using this queue. 
     *
     *  Valid values can range between 0 - 128. 
     *
     *  When set to 0, no additional space is allocated for the
     *  DFT size list configuration when allocating request
     *  buffers in @a Fftc_txOpen (). When a valid value is set, 
     *  the required space for it is allocated in the request buffers.
     *
     *  This value is only used if 'bManageReqBuffers' is set to 1, i.e.
     *  if the FFTC driver is pre-allocating buffers on the free 
     *  queue.
     */
    uint32_t                    maxDftSizeListLen;

    /** CPPI Descriptor Type to use for the Tx Buffer descriptors.
     *
     *  Valid values are:
     *      Cppi_DescType_HOST,
     *      Cppi_DescType_MONOLITHIC
     *
     *  Used only if 'bManageReqBuffers' set to 1.
     */
    Cppi_DescType               descType;

    /** Number of Transmit Free descriptors to allocate for 
     *  this Tx object.
     *
     *  Tx Free descriptors are allocated ONLY if the 
     *  'bManageReqBuffers' flag is set to 1, i.e.
     *  if the FFTC driver is configured to pre-allocate buffers 
     *  on the free queue.
     *
     */        
    uint32_t                    cppiNumDesc;

    /** Maximum size of buffer to allocate for this Tx object.
     *
     *  The buffer size should be chosen such that it can hold all
     *  the FFT blocks that would be submitted using this Tx object.
     *  The driver ensures that additional space for PS info is
     *  allocated if 'bPSInfoPresent' is set to 1.
     *
     *  This value is only used if 'bManageReqBuffers' is set to 1, 
     *  i.e. if the FFTC driver is pre-allocating buffers on 
     *  the free queue.
     */
    uint32_t                    bufferSize;

    /** Boolean flag to indicate if there is going to be any 
     *  Protocol Specific (PS) information that would have to be passed
     *  to the FFT result receiver.
     *
     *  Set to 1 to indicate that there will be PS info, 0 otherwise. 
     *  When set to 0, no space for PS info is allocated.
     *
     *  Valid only if 'bManageReqBuffers' is set to 1
     */
    uint8_t                     bPSInfoPresent;
} Fftc_TxCfg;

typedef struct _Fftc_DrvRxFlowCfg
{
    /** CPPI Descriptor Type to use for the Rx Buffer descriptors.
     *
     *  Valid values are:
     *      Cppi_DescType_HOST,
     *      Cppi_DescType_MONOLITHIC
     */
    Cppi_DescType               descType;

    /** Number of Receive Free descriptors to allocate for 
     *  this Rx object.
     */        
    uint32_t                    cppiNumDesc;

    /** Maximum size of buffer to allocate for Rx object.
     *
     *  The buffer size should be chosen such that it can hold all
     *  the FFT blocks that would be received using this Rx object
     *  and also have enough room to hold any Protocol Specific (PS) 
     *  information if needed.
     */
    uint32_t                    bufferSize;

    /** Boolean flag to indicate if the receiver is willing to accept
     *  Protocol Specific (PS) information from the sender.
     *
     *  Set to 1 to indicate that there will be PS info, 0 otherwise. 
     *
     *  When this flag is set to 0, CPPI Rx flow is configured such 
     *  that no PS info is propagated to the receiver / result
     *  end.
     */
    uint8_t                     bPSInfoPresent;

    /** Indicates where the PS info should be put in the FFT result 
     *  packet.
     *
     *  Set to:
     *  0    -  to inidcate that PS info is in the "Protocol Specific
     *          Word" fields of the CPPI Descriptor.
     *
     *  1    -  to indicate that the PS info is at the start of the 
     *          data buffer itself.
     *
     *  This values is used only if 'bPSInfoPresent' field is set to 
     *  1 and if the descriptor type 'descType' is Cppi_DescType_HOST
     *  only.
     *
     */
    Cppi_PSLoc                  psLocation;
} FFtc_DrvRxFlowCfg;


/** 
 *  @brief  Fftc_RxCfg
 *
 *          FFTC Receive Object configuration.
 *          
 *          Holds Rx specific configuration. Rx Objects
 *          are to be created and used by FFTC driver users
 *          to receive FFT results from the engine.
 */    
typedef struct _Fftc_RxCfg
{
    /** CPPI Destination queue number onto which the FFT/IFFT result from
     *  the FFTC engine should be queued to.
     *
     *  This can be set to -1 to let the driver pick a destination 
     *  queue number.
     */        
    int32_t                     cppiRxQNum;

    /** Flow properties for the Rx object.
     *
     *  Must be set to -1 if a new flow/Rx FDQ must be created 
     *  for this Rx object.
     *
     *  Set to a valid FFTC flow Id to reuse an existing flow
     *  properties for this Rx object. In this case, the Rx FDQ
     *  and the flow properties are inherited from the flow 
     *  specified.
     */
    int8_t                      useFlowId;

    /** Boolean flag to indicate if the driver should manage the
     *  Rx Free Descriptor Queues and flow for the application or not.
     *
     *  When set to 1, driver opens a Rx FDQ and creates a flow for 
     *  the application based on the configuration
     *  specified in 'rxFlowCfg.drvCfg' and manages them. 
     *  Only minimal set of parameters are exposed for configuration
     *  here.
     *
     *  When set to 0, the driver expects all the required flow
     *  configuration and Rx FDQs to be setup and managed by the 
     *  application. The flow configuration specified in 'rxFlowCfg.fullCfg'
     *  is used as is in creating a flow.
     *
     *  Used only if 'useFlowId' set to -1, i.e., when creating a new
     *  flow.
     */
    uint8_t                     bManageRxFlowCfg;

    union
    {
        /** Must be filled if 'bManageRxFlowCfg' set to 1 */            
        FFtc_DrvRxFlowCfg       drvCfg;

        /** Must be filled if 'bManageRxFlowCfg' set to 0 */            
        Cppi_RxFlowCfg          fullCfg;
    }rxFlowCfg;

    /** Boolean flag to indicate whether if interrupt support is 
     *  required for this Rx object.
     *
     *  When set to 1, indicates that high priority accumulation
     *  interrupts must be used for this Rx object.
     */
    uint8_t                     bUseInterrupts;

    /** Boolean flag to indicate if the driver should manage the
     *  High priority accumulator list for the application or not.
     *
     *  When set to 1, driver would allocate the accumulator list
     *  and manage it for the application based on the configuration
     *  specified in 'accumCfg.drvCfg'. Only minimal set of 
     *  parameters are exposed for configuration here.
     *
     *  When set to 0, the accumulator list is expected to be allocated 
     *  by application fully and the configuration specified in 
     *  'accumCfg.fullCfg' is programmed to the QM accumulator as is.
     *
     *  Used only if 'bUseInterrupts' set to 1.
     */
    uint8_t                     bManageAccumList;

    /** Interrupt configuration to use for this Rx object.
     *
     *  Used only if 'bUseInterrupts' set to 1.
     */
    union
    {
        /*  Used when 'bManageAccumList' set to 1. */
        Fftc_InterruptCfg       drvCfg;

        /*  Used when 'bManageAccumList' set to 0. */
        Qmss_AccCmdCfg          fullCfg;
    }accumCfg;

    /** Boolean flag to indicate whether the Rx object must be configured
     *  in "blocking" or "non-blocking" mode. 
     *
     *  Used only if 'bUseInterrupts' set to 1.
     *
     *  In Blocking mode, the driver enables the application
     *  to block or wait on a result using @a Fftc_rxGetResult () API. 
     *  If configured non-blocking, when no result available the 
     *  application is notified of the status and no blocking 
     *  support is provided.
     */
    uint8_t                     bBlockOnResult;
} Fftc_RxCfg;

/**
@}
*/

extern uint32_t Fftc_getVersionID 
( 
    void
);

extern const char* Fftc_getVersionStr 
(
    void
);

extern Fftc_RetVal Fftc_init
(
    uint8_t                     instNum, 
    Fftc_GlobalCfg*             pFFTCGlobalCfg,
    Fftc_DeviceCfg*             pFFTCDevCfg
);

extern Fftc_RetVal Fftc_deInit 
(
    uint8_t                     instNum
);

extern uint8_t Fftc_isInitialized
(
    uint8_t                     instNum
);

extern Fftc_DrvHandle Fftc_open 
(
    uint8_t                     instNum,
    Fftc_DrvCfg*                pFFTCDrvCfg,
    Fftc_RetVal*                pRetVal
);

extern Fftc_RetVal Fftc_close 
(
    Fftc_DrvHandle              hFFTC
);

extern Fftc_LldObj* Fftc_getLLDObject 
(
    Fftc_DrvHandle          hFFTC
);

extern Fftc_TxHandle Fftc_txOpen
(
    Fftc_DrvHandle              hFFTC,
    Fftc_TxCfg*                 pFFTCTxCfg
);

extern Fftc_RetVal Fftc_txClose 
(
    Fftc_TxHandle               hFFTCTxInfo
);

extern Fftc_RxHandle Fftc_rxOpen
(
    Fftc_DrvHandle              hFFTC,
    Fftc_RxCfg*                 pFFTCRxCfg
);

extern Fftc_RetVal Fftc_rxClose 
(
    Fftc_RxHandle               hFFTCRxInfo
);

extern Fftc_RetVal Fftc_rxGetRxQueueNumber
(
    Fftc_RxHandle               hFFTCRxInfo
);

extern Fftc_RetVal Fftc_rxGetFlowId
(
    Fftc_RxHandle               hFFTCRxInfo
);

extern Fftc_RetVal Fftc_findFlowIdByQueueNumber
(
    Fftc_DrvHandle              hFFTC,
    uint32_t                    rxQueueNumber
);

extern Fftc_RetVal Fftc_txGetRequestBuffer 
(
    Fftc_TxHandle               hFFTCTxInfo,
    Fftc_BlockInfo*             pDFTBlockSizeInfo,
    Fftc_QLocalCfg*             pFFTCQConfig, 
    uint32_t                    psInfoLen,
    uint8_t                    	destnFlowId,
    uint16_t                    destnTagInfo,
    Fftc_RequestHandle*         phFFTCRequest,     
    uint8_t**                   ppReqDataBuffer, 
    uint32_t*                   pMaxDataBufferLen
);

extern Fftc_RetVal Fftc_txSubmitRequest 
(
    Fftc_TxHandle               hFFTCTxInfo,
    Fftc_RequestHandle          hRequestInfo,
    uint32_t                    reqBufferLen
);

extern Fftc_RetVal Fftc_txFreeRequestBuffer 
(
    Fftc_TxHandle               hFFTCTxInfo,
    Fftc_RequestHandle          hRequestInfo
);

extern Fftc_RetVal Fftc_rxProcessDesc 
(
    Fftc_RxHandle               hFFTCRxInfo,
    Cppi_Desc*                  pCppiDesc,
    Fftc_ResultHandle*          phResultInfo,
    uint8_t**                   ppResultBuffer,
    uint32_t*                   pResultBufferLen,                   
    uint8_t**                   ppPSInfo,
    uint32_t*                   pPSInfoLen,
    uint8_t*                    pFlowId,
    uint8_t*                    pSrcId,
    uint16_t*                   pDestnTagInfo
);

extern int32_t Fftc_rxGetNumPendingResults 
(
    Fftc_RxHandle               hFFTCRxInfo
);

extern Fftc_RetVal Fftc_rxGetResult
(
    Fftc_RxHandle               hFFTCRxInfo,
    Fftc_ResultHandle*          phResultInfo,
    uint8_t**                   ppResultBuffer,
    uint32_t*                   pResultBufferLen,                   
    uint8_t**                   ppPSInfo,
    uint32_t*                   pPSInfoLen,
    uint8_t*                    pFlowId,
    uint8_t*                    pSrcId,
    uint16_t*                   pDestnTagInfo
);

extern Fftc_RetVal Fftc_rxFreeResult 
(
    Fftc_RxHandle               hFFTCRxInfo,
    Fftc_ResultHandle           hResultInfo
);

extern Fftc_RetVal Fftc_rxParseResult
(    
    Fftc_RxHandle               hFFTCRxInfo,
    Fftc_ResultHandle           hResultInfo,
    uint8_t*                    pResultBuffer,
    uint32_t                    resultBufferLen,
    Fftc_BlockInfo*             pDFTBlockSizeInfo,
    uint8_t                     bSupressSideInfo,
    uint16_t                    cyclicPrefixAddNum, 
    Fftc_Result*                pFFTResult
);

extern void Fftc_rxHiPriorityRxISR 
(
    Fftc_RxHandle               hFFTCRxInfo
);

#ifdef __cplusplus
}
#endif

#endif  /* __FFTC_H__ */
