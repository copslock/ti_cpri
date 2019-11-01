/** 
 *   @file  bcp_pvt.h
 *
 *   @brief  
 *      Private Header file with data structure and API declarations for 
 *      BCP driver's internal use.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
#ifndef _BCP_PVT_H_
#define _BCP_PVT_H_

/* BCP types include */
#include <bcp_types.h>

/* BCP LLD include */
#include <ti/drv/bcp/bcp_lld.h>

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

/** 
 *  @brief  Bcp_TxQInfo
 *
 *          Data structure to hold the current state and configuration
 *          of any given BCP Transmit queue. 
 *
 *          * Used by BCP driver internally for book-keeping purposes *
 */    
typedef struct  _Bcp_TxQInfo
{
    /** The BCP Transmit queue number for this instance. */
    int32_t                     qNum;

    /** CPPI transmit channel handle corresponding to this object. */     
    Cppi_ChHnd                  hCppiTxChan;

    /** Handle to the BCP Tx queue instance.   */     
    Qmss_QueueHnd               hQmssTxQ;

    /** Reference counter that keeps track of the number of 
     *  applications that are using this BCP Tx queue.
     *
     *  This counter is incremented whenever '@a BCP_txQueueOpen ()'
     *  is called and a valid transmit queue handle is returned and
     *  is decremented whenever '@a BCP_txQueueClose ()' API is called.
     *  When this count reaches zero, the transmit queue configuration
     *  is completely cleaned up and would have to be re-initialized 
     *  using '@a BCP_txQueueOpen ()' API.
     */
    uint32_t                    refCnt;
} Bcp_TxQInfo;

/** 
 *  @brief  Bcp_TxInfo
 *
 *          Data structure to hold the current state of an BCP Tx 
 *          object created using the driver APIs.
 *
 *          * Used by BCP driver internally for book-keeping purposes *
 *
 */    
typedef struct  _Bcp_TxInfo
{
    /** BCP instance object handle 
     *
     *  Reference to parent driver handle. Indicates
     *  which BCP instance this object belongs.
     */
    Bcp_DrvHandle               hBcp;

    /** BCP Tx Queue Handle for this object. */
    Bcp_TxQInfo*                pTxQInfo;

    /** BCP tunnel's Tx endpoint info handle */
    void*                       hTxEndpointInfo;
} Bcp_TxInfo;

/** 
 *  @brief  Bcp_FlowInfo
 *
 *          Data structure to hold the current state and configuration
 *          of a CPPI Rx flow configured using the driver for a given 
 *          BCP peripheral instance.
 *
 *          * Used by BCP driver internally for book-keeping purposes *
 */    
typedef struct _Bcp_FlowInfo
{
    /** Reference count to track the number of users of the flow. */
    uint8_t                     refCnt;

    /** CPPI flow Id corresponding to this flow */
    int32_t                     flowId;

    /** CPPI Flow handle for this object */
    Cppi_FlowHnd	            hCppiRxFlow;

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
} Bcp_FlowInfo;

/** 
 *  @brief  Bcp_RxInfo
 *
 *          Data structure to hold the Rx object info.
 *
 *          * Used by BCP driver internally for book-keeping purposes *
 */    
typedef struct _Bcp_RxInfo
{
    /** BCP driver handle 
     *
     *  Reference to parent driver handle. Indicates
     *  which BCP instance this object belongs.
     */
    Bcp_DrvHandle               hBcp;

    /** Global Rx object identifier.
     *
     *  Used to identify a Rx object uniquely in the global 
     *  Rx object database.
     */
    int32_t                     globalObjId;
    
    /** Rx object configuration */
    Bcp_RxCfg                   cfg;

    /** CPPI flow Id corresponding to this Rx object */
    int32_t                     flowId;

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

    /** Destination/Rx Queue Number where the BCP result for
     *  this Rx object will be output to by the hardware.
     */
    uint32_t                    rxQNum;

    /** Handle to the destination queue where results will be stored */
    Qmss_QueueHnd               hQmssRxQ;

    /** CPPI Rx channel handle corresponding to this object. */
    Cppi_ChHnd                  hCppiRxChan;

    void*                       hRxEndpointInfo;
} Bcp_RxInfo;

/** 
 *  @brief  Bcp_RxGlobalInfo
 *
 *          Data structure to hold the globally accessible 
 *          info related to a Rx object.
 *
 *          Used to hold all the Rx Queue Number-Flow Id mappings.
 *
 *          * Used by BCP driver internally for book-keeping purposes *
 */    
typedef struct _Bcp_RxGlobalInfo
{
    /** Boolean flag to track the validity of this entry */
    uint8_t                     bIsValid;

    /** Rx object identifier.
     *
     *  Used to identify a Rx object uniquely in the global 
     *  Rx object database.
     */
    uint8_t                     Id;

    /** Destination/Rx Queue Number where the BCP result for
     *  this Rx object will be output to by the hardware.
     */
    uint32_t                    rxQNum;

    /** CPPI flow Id corresponding to this Rx object */
    int32_t                     flowId;
} Bcp_RxGlobalInfo;

/** 
 *  @brief  Bcp_DrvInfo
 *
 *          Tracks BCP application specific configuration info
 *          obtained from @a Bcp_open () API.
 *
 *          Data structure to hold the BCP peripheral instance 
 *          number and other info provided by an application 
 *          that wishes to use the BCP driver.
 *
 *          * Used by BCP driver internally for book-keeping purposes *
 */    
typedef struct  _Bcp_DrvInfo
{
    /** BCP peripheral instance number this object uses */        
    uint8_t                     instNum;

    /** Driver configuration specified by application */
    Bcp_DrvCfg                  drvCfg;

    /** BCP driver mode */        
    Bcp_DrvMode                 mode;

    /* BCP transport layer callback functions */
    void*                       (*pFxnBcpTunnel_txOpen) (void*);
    int32_t                     (*pFxnBcpTunnel_txClose) (void*);

    void*                       (*pFxnBcpTunnel_rxOpen) (void*);
    int32_t                     (*pFxnBcpTunnel_rxClose) (void*);
    
    int32_t                     (*pFxnBcpTunnel_send) (void*, void*, uint32_t, void *);
    int32_t                     (*pFxnBcpTunnel_recv) (void*, void**);
    int32_t                     (*pFxnBcpTunnel_freeRecvBuffer) (void*, void*, uint32_t);
} Bcp_DrvInfo;        

/** 
 *  @brief  Bcp_instanceInfo
 *
 *          Data structure to hold the current state and configuration
 *          in driver for each BCP peripheral instance.
 *
 *          * Used by BCP driver internally for book-keeping purposes *
 *
 */    
typedef struct  _Bcp_InstanceInfo
{
    /** BCP peripheral instance number */        
    uint8_t                     instNum;

    /** BCP driver mode */        
    Bcp_DrvMode                 mode;

    /* BCP transport layer callback functions */
    void*                       (*pFxnBcpTunnel_txOpen) (void*);
    int32_t                     (*pFxnBcpTunnel_txClose) (void*);

    void*                       (*pFxnBcpTunnel_rxOpen) (void*);
    int32_t                     (*pFxnBcpTunnel_rxClose) (void*);
    
    int32_t                     (*pFxnBcpTunnel_send) (void*, void*, uint32_t, void*);
    int32_t                     (*pFxnBcpTunnel_recv) (void*, void**);
    int32_t                     (*pFxnBcpTunnel_freeRecvBuffer) (void*, void*, uint32_t);

    /** Base Tx queue number for this instance. */
    uint32_t                    baseTxQueueNum;

    /** Counter to track the number of applications using this BCP instance. */
    uint32_t                    refCnt;

    /** Handle to the BCP CDMA */
    Cppi_Handle                 hCppi;

    /** Handle to the BCP LLD object corresponding to this instance. */        
    Bcp_LldObj                  bcpLldObj;

    /** Holds the state information pertinent to all the BCP Tx queues 
     *  managed by the driver for this instance.
     */    
    Bcp_TxQInfo                 Bcp_txQInfo [BCP_MAX_NUM_TXQUEUES];    

    /** Holds info pertinent to all flows configured using the driver
     *  for this BCP instance.
     */
    Bcp_FlowInfo                Bcp_flowInfo [BCP_MAX_NUM_FLOWS];

    /** Holds global info pertinent to all the Rx objects created 
     *  using the driver.
     */
    Bcp_RxGlobalInfo            Bcp_rxObjGlobalInfo [BCP_MAX_NUM_RXOBJECTS];

    /** Padding to align the data structure on L2 cache line */
    uint8_t                     pad[108];
} Bcp_InstanceInfo;
        
/** Tracks info that is maintained by BCP driver for each of its peripheral instances. */
#pragma DATA_SECTION (gBcp_instanceInfo, ".bcp");
#pragma DATA_ALIGN (gBcp_instanceInfo, CACHE_L2_LINESIZE)
Bcp_InstanceInfo                gBcp_instanceInfo [BCP_MAX_NUM_INSTANCES];
    
#ifdef __cplusplus
}
#endif

#endif  /* __BCP_PVT_H__ */

