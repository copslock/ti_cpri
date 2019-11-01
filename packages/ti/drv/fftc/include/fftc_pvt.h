/** 
 *   @file  fftc_pvt.h
 *
 *   @brief  
 *      Private Header file with data structure and API declarations for 
 *      FFTC driver's internal use.
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
#ifndef _FFTC_PVT_H_
#define _FFTC_PVT_H_

/* FFTC types include */
#include <fftc_types.h>

/* FFTC LLD include */
#include <ti/drv/fftc/fftc_lld.h>

/* FFTC listlib include */
#include <ti/drv/fftc/include/listlib.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* CSL Cache include */
#include <ti/csl/csl_cache.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief
 * Size of the mandatory portion of the CPPI Host 
 * descriptor.
 */
#define     FFTC_CPPI_HOST_DESC_SIZE                (32)

/** @brief
 * Size of the mandatory portion of the CPPI Monolithic 
 * descriptor.
 */
#define     FFTC_CPPI_MONOLITHIC_DESC_SIZE          (12)

/** 
 *  @brief  Fftc_TxQInfo
 *
 *          Data structure to hold the current state and configuration
 *          of any given FFTC Transmit queue. 
 *
 *          * Used by FFTC driver internally for book-keeping purposes *
 */    
typedef struct  _Fftc_TxQInfo
{
    /** The FFTC Transmit queue number for this instance. */
    int32_t                     cppiTxQNum;

    /** Boolean flag to indicate if this queue is a "shared"
     *  queue or "dedicated" queue.
     *
     *  A shared queue's configuration can be programmed using 
     *  CPPI packets on the go by any application using it.
     *
     *  A dedicated queue on the other hand is to be used by
     *  one application, its configuration is setup during open
     *  and never overridden using CPPI packets. The only way to
     *  change its configuration would be to close it and re-open
     *  it using a different configuration.
     *
     *  When set to 1 indicates "shared" mode, configuration
     *  programmable using CPPI packets. 0 indicates "dedicated"
     *  mode and configuration setup during queue open.
     */
    uint8_t                     bSharedMode;

    /** Handle to the CPPI Transmit channel corresponding to
     *  the FFTC queue.
     */     
    Cppi_ChHnd                  hCppiTxChan;

    /** CPPI Rx channel handle corresponding to the Tx object. */
    Cppi_ChHnd                  hCppiRxChan;

    /** Handle to the QMSS FFTC queue instance.   */     
    Qmss_QueueHnd               hQmssTxQ;

    /** Boolean flag, set to 1 to supress FFTC side-band info such as
     *  block exponent, clipping detection, error and tag being 
     *  output.
     */
    uint8_t                     bSupressSideInfo;
    
    /** Number of samples to use for cyclic prefix addition.  */
    uint32_t                    cyclicPrefixAddNum;

    /** Reference counter that keeps track of the number of 
     *  applications that are using this FFTC Tx queue.
     *
     *  This counter is incremented whenever '@a FFTC_txQueueOpen ()'
     *  is called and a valid transmit queue handle is returned and
     *  is decremented whenever '@a FFTC_txQueueClose ()' API is called.
     *  When this count reaches zero, the transmit queue configuration
     *  is completely cleaned up and would have to be re-initialized 
     *  using '@a FFTC_txQueueOpen ()' API.
     */
    uint32_t                    refCnt;
} Fftc_TxQInfo;

/** 
 *  @brief  Fftc_TxInfo
 *
 *          Data structure to hold the current state of an FFTC Tx 
 *          object created using the driver APIs.
 *
 *          * Used by FFTC driver internally for book-keeping purposes *
 *
 */    
typedef struct  _Fftc_TxInfo
{
    /** FFTC instance object handle 
     *
     *  Reference to parent driver handle. Indicates
     *  which FFTC instance this object belongs.
     */
    Fftc_DrvHandle              hFFTC;

    /** The Transmit free descriptor queue handle for this object */
    Qmss_QueueHnd               hQmssTxFreeQ;

    /** The transmit free descriptor queue handle to which the
     *  free descriptors must be returned when cleaning up this 
     *  Tx object.
     *
     *  The parent free queue for 'hQmssTxFreeQ' of this Tx object.
     */
    Qmss_QueueHnd               hQmssTxGlblFreeQ;

    /** FFTC Tx Queue Handle for this object. */
    Fftc_TxQInfo*               pFFTCTxQInfo;

    /** Handle to the QMSS FFTC queue instance.   */     
    Qmss_QueueHnd               hQmssTxQ;
   
    /** Tx queue open mode */
    uint8_t                     bSharedMode;

    /** Boolean flag to indicate whether transmit free descriptors 
     *  and buffers were allocated by the driver.
     */
    uint8_t                     bManageReqBuffers;

    /** Number of Transmit Free descriptors allocated for 
     *  this Tx object.
     */        
    uint32_t                    cppiNumDesc;

    /** Size of the descriptor allocated for this Tx object */
    uint32_t                    descSize;

    /** Buffer size allocated for this Tx object. */
    uint32_t                    bufferSize;

    /** Indicates if this Tx object is using DFT size list
     *  configuration.
     */
    uint8_t                     bEnableDftSizeListCfg;

    /** The maximum number of DFT sizes that will ever be configured
     *  using this Tx queue.
     */
    uint32_t                    maxDftSizeListLen;
} Fftc_TxInfo;

/** 
 *  @brief  Fftc_FlowInfo
 *
 *          Data structure to hold the current state and configuration
 *          of a CPPI Rx flow configured using the driver for a given 
 *          FFTC peripheral instance.
 *
 *          * Used by FFTC driver internally for book-keeping purposes *
 */    
typedef struct _Fftc_FlowInfo
{
    /** Reference count to track the number of users of the flow. */
    uint8_t                     refCnt;

    /** Boolean flag that indicates if the flow was created using
     * driver managed Rx flow configuration or not.
     */
    uint8_t                     bUsesDrvRxFlowCfg;

    /** CPPI flow Id corresponding to this flow */
    uint32_t                    cppiFlowId;

    /** CPPI Flow handle for this object */
    Cppi_FlowHnd	            hCppiRxFlow;

    /** Number of Rx Free descriptors allocated for this flow */
    uint32_t                    cppiNumDesc;

    /** Rx free descriptor size allocated for this object */
    uint32_t                    descSize;

    /** Rx buffer size allocated */
    uint32_t                    bufferSize;

    /** The Receive free descriptor queue handle for the object */
    Qmss_QueueHnd               hQmssRxFreeQ;

    /** The Global Receive free descriptor queue handle to which the
     *  free descriptors must be returned when cleaning up this object.
     *
     *  The parent free queue for 'hQmssRxFreeQ' of this object.
     */
    Qmss_QueueHnd               hQmssRxGlblFreeQ;

    /** Due to H/W Bug, the PS Info location is no longer
     *  reliable when read from descriptor. Hence the driver
     *  must track the PS Info location and its presence for
     *  all flows.
     *
     *  Indicates whether the flow was configured to receive 
     *  PS Info.
     */
    uint8_t                     bPSInfoPresent;

    /** Due to H/W Bug, the PS Info location is no longer
     *  reliable when read from descriptor. Hence the driver
     *  must track the PS Info location and its presence for
     *  all flows.
     *
     *  Indicates PS location configured on the flow.
     *
     *  When:
     *  0    -  Inidcates that PS info is in the "Protocol Specific
     *          Word" fields of the CPPI Descriptor.
     *
     *  1    -  Indicates that the PS info is at the start of the 
     *          data buffer itself.
     *
     *  Valid only when 'bPSInfoPresent' set to 1.
     */
    Cppi_PSLoc                  psLocation;
} Fftc_FlowInfo;

/** 
 *  @brief  Fftc_RequestInfo
 *
 *          Data structure to hold the results received from ISR
 *          for an Rx object.
 *
 *          * Used by FFTC driver internally for book-keeping purposes *
 */    
typedef struct  _Fftc_RequestInfo
{
    /** Next node link in this list. */
    Fftc_ListNode               links;

    /** CPPI Descriptor pointer corresponding this FFT request/result */
    Cppi_Desc*                  pCppiDesc;
} Fftc_RequestInfo;

/** 
 *  @brief  Fftc_RxInfo
 *
 *          Data structure to hold the globally accessible 
 *          info related to a Rx object.
 *
 *          Used to hold the pending requests and results info
 *          for all the Rx objects created using the driver.
 *
 *          * Used by FFTC driver internally for book-keeping purposes *
 */    
typedef struct _Fftc_RxInfo
{
    /** FFTC instance object handle 
     *
     *  Reference to parent driver handle. Indicates
     *  which FFTC instance this object belongs.
     */
    Fftc_DrvHandle              hFFTC;

    /** Global Rx object identifier.
     *
     *  Used to identify a Rx object uniquely in the global 
     *  Rx object database.
     */
    uint8_t                     rxGlobalObjId;
    
    /** Rx object configuration */
    Fftc_RxCfg                  cfg;

    /** CPPI flow Id corresponding to this Rx object */
    uint32_t                    cppiFlowId;

    /** Rx free descriptor size allocated for this object. Valid only
     *  if Rx object was created using driver managed Rx flow configuration */
    uint32_t                    descSize;

    /** Due to H/W Bug, the PS Info location is no longer
     *  reliable when read from descriptor. Hence the driver
     *  must track the PS Info location and its presence for
     *  all Rx objects.
     *
     *  Indicates whether the flow used by this object was 
     *  configured to receive PS Info.
     */
    uint8_t                     bPSInfoPresent;

    /** Due to H/W Bug, the PS Info location is no longer
     *  reliable when read from descriptor. Hence the driver
     *  must track the PS Info location and its presence for
     *  all Rx objects.
     *
     *  Indicates PS location configured on the flow used by
     *  this Rx object.
     *
     *  When:
     *  0    -  Inidcates that PS info is in the "Protocol Specific
     *          Word" fields of the CPPI Descriptor.
     *
     *  1    -  Indicates that the PS info is at the start of the 
     *          data buffer itself.
     *
     *  Valid only when 'bPSInfoPresent' set to 1.
     */
    Cppi_PSLoc                  psLocation;

    /** Destination/Rx Queue Number where the FFTC result for
     *  this Rx object will be output to by the hardware.
     */
    uint32_t                    cppiRxQNum;

    /** Handle to the destination queue where results will be stored */
    Qmss_QueueHnd               hQmssRxQ;

    /** Indicates if accumulator list is managed by driver */
    uint8_t                     bUsesDrvAccumList;

    /** Interrupt threshold value configured. */
    uint16_t                    intThreshold;

    /** High priority accumulator channel number used */
    uint8_t                     accChannelNum;

    /** Rx semaphore handle.
     *
     * Used only if the Rx object was configured to use interrupts
     * and in blocking mode.
     */
    void*                       hResultSem;

    /** Boolean flag used by driver internally to track if it
     *  processed the Ping/Pong accumulator list last.
     *
     *  The accumulator writes the results alternatively to the
     *  Ping and Pong lists of the accumulator. The driver needs
     *  to track which of these lists it last processed, so that
     *  it can process the other one next time around.
     */
    uint8_t                     bPingListUsed;

    /** 16-byte aligned high priority accumulator list used
     *  for this core.
     */
    uint32_t*                   pHiPrioAccList;

    /** Original address of the High priority Accumulator list 
     * allocated from the heap (pre-16 byte alignment address).
     *
     * Used to store the original accumulator address, needed while
     * freeing the list.
     */
    uint32_t*                   pOrigAccListAddress;
   
    /** Free result entry list. */
    Fftc_RequestInfo*           freeResultQueue;

    /** Result list. 
     *
     *  Software list maintained by the driver. Holds all
     *  the FFT results arrived for processing by this Rx object.
     */
    Fftc_RequestInfo*           resultQueue;

    /** Number of entries in the result queue yet to be
     *  processed by the DSP application.
     */
    uint8_t                     resultQueueLen;

    /** Indicates the number of times a packet was received by
     * ISR but couldnt be posted to the application, because of
     * lack of any free result entries to save the result, i.e.,
     * result queue is full.
     */
    uint32_t                    rxFull;

    /** Indicates the number of times a result packet was received 
     * with an invalid length, i.e., length of result received
     * doesnt match the expected/calculated length. This is
     * incremented only if @a Fftc_rxParseResult () API is called
     * on a result received.
     */
    uint32_t                    rxBadLength;

    /** Indicates the number of times a result packet was received 
     * with a wrong flow Id, i.e., the flow Id on the received 
     * packet doesnt match the flow Id the Rx object is configured
     * to use. This is incremented only if @a Fftc_rxParseResult () 
     * API is called on a result received.
     */
    uint32_t                    rxBadDestn;
} Fftc_RxInfo;

/** 
 *  @brief  Fftc_RxGlobalInfo
 *
 *          Data structure to hold the globally accessible 
 *          info related to a Rx object.
 *
 *          Used to hold the pending requests and results info
 *          for all the Rx objects created using the driver.
 *
 *          * Used by FFTC driver internally for book-keeping purposes *
 */    
typedef struct _Fftc_RxGlobalInfo
{
    /** Boolean flag to track the validity of this entry */
    uint8_t                     bIsValid;

    /** Rx object identifier.
     *
     *  Used to identify a Rx object uniquely in the global 
     *  Rx object database.
     */
    uint8_t                     Id;

    /** Destination/Rx Queue Number where the FFTC result for
     *  this Rx object will be output to by the hardware.
     */
    uint32_t                    cppiRxQNum;

    /** CPPI flow Id corresponding to this Rx object */
    uint32_t                    cppiFlowId;

} Fftc_RxGlobalInfo;

/** 
 *  @brief  Fftc_UserInfo
 *
 *          Tracks FFTC application specific configuration info
 *          obtained from @a Fftc_open () API.
 *
 *          Data structure to hold the descriptor configuration,
 *          FFTC peripheral instance number and other info
 *          provided by an application that wishes to use the FFTC
 *          driver.
 *
 *          * Used by FFTC driver internally for book-keeping purposes *
 */    
typedef struct  _Fftc_UserInfo
{
    /** FFTC peripheral instance number this object uses */        
    uint8_t                     instNum;

    /** Driver configuration specified by application */
    Fftc_DrvCfg                 drvCfg;

    /** FFTC CPDMA's free descriptor queue handles. */
    Qmss_QueueHnd               hQmssFreeDescQ [FFTC_MAX_NUM_FREEQ];
} Fftc_UserInfo;

/** 
 *  @brief  Fftc_InstanceInfoObj
 *
 *          Data structure to hold the current state and configuration
 *          in driver for each FFTC peripheral instance.
 *
 *          * Used by FFTC driver internally for book-keeping purposes *
 *
 */    
typedef struct  _Fftc_InstanceInfoObj
{
    /** FFTC peripheral instance number */        
    uint8_t                     instNum;

    /** Reference counter that keeps track of the number of 
     *  applications that are using this FFTC instance.
     *
     *  This counter is incremented whenever '@a Fftc_open ()'
     *  is called and a valid FFTC instance handle is returned and
     *  is decremented whenever '@a Fftc_close ()' API is called.
     *  When this count reaches zero, the FFTC CPDMA is closed and
     *  all associated information is completely cleaned up and 
     *  would have to be re-initialized using '@a Fftc_init ()' 
     *  API.
     */
    uint32_t                    refCnt;

    /** CPDMA Number corresponding to this FFTC peripheral instance. */
    Cppi_CpDma                  cpdmaNum;

    /** Tx queue number base for this FFTC peripheral instance. */
    uint32_t                    baseQueueNum;

    /** Handle to the FFTC CPDMA */
    Cppi_Handle                 hCppi;

    /** Handle to the FFTC LLD object corresponding to this instance. */        
    Fftc_LldObj                 fftcLldObj;

    /** Boolean flag to track if DFT size list configuration is being
     *  used by any of the Tx flows configured in the driver.
     *
     *  The DFT size list is a shared resource between all the FFT
     *  queues in the H/W and only one application MUST use it at
     *  a time. The driver ensures this using this flag. This is 
     *  set to 1, when a Tx flow using DFT size list is configured
     *  to indicate DFT size list in use and 0 otherwise.
     */        
    uint8_t                     bIsDftSizeListInUse;

    /** Holds the state information pertinent to all the FFTC Tx queues 
     *  (4) managed by the driver for this instance.
     */    
    Fftc_TxQInfo                Fftc_txQInfo [FFTC_MAX_NUM_TXQUEUES];    

    /** Holds info pertinent to all flows configured using the driver
     *  for this FFTC instance.
     */
    Fftc_FlowInfo               Fftc_flowInfo [FFTC_MAX_NUM_FLOWS];

    /** Holds global info pertinent to all the Rx objects created 
     *  using the driver.
     */
    Fftc_RxGlobalInfo           Fftc_rxObjGlobalInfo [FFTC_MAX_NUM_RXOBJECTS];

    /** Padding to align the data structure on L2 cache line */
    uint8_t                     pad[32];
} Fftc_InstanceInfoObj;
        


/** Tracks info that is maintained by FFTC driver for each
 *  of the FFTC peripheral instances.
 */
#pragma DATA_SECTION (Fftc_instanceInfo, ".fftc");
#pragma DATA_ALIGN (Fftc_instanceInfo, CACHE_L2_LINESIZE)
Fftc_InstanceInfoObj     Fftc_instanceInfo [FFTC_MAX_NUM_INSTANCES];
    
#ifdef __cplusplus
}
#endif

#endif  /* __FFTC_PVT_H__ */
