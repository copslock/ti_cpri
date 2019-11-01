/** 
 *   @file  bcp.h
 *
 *   @brief  
 *      Header file with data structure and API declarations for BCP High
 *      level driver.
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
#ifndef _BCP_H_
#define _BCP_H_

/* BCP LLD and MMR includes */
#include <ti/drv/bcp/bcp_lld.h>
#include <ti/drv/bcp/bcp_mmr.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* CHIP module include */
#include <ti/csl/csl_chip.h>

/* BCP driver version header file */
#include <ti/drv/bcp/bcpver.h>

/** @defgroup BCP_API BCP Higher Layer Data Structures & APIs
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *      The BCP Driver is divided into 2 layers:
 *          -#  Low Level Driver APIs
 *          -#  High Level APIs
 *      
 *      The Low Level Driver APIs provide BCP MMR access by 
 *      exporting register read/write APIs and also provides
 *      some useful helper APIs in putting together BCP global
 *      and submodule headers required by the hardware.
 *
 *      The BCP Higher Layer provide APIs useful in submitting
 *      BCP requests and retrieving their results from the BCP. 
 *
 *      This section of documentation covers the Higher Layer
 *      APIs and data structures.
 */

/**
@defgroup BCP_SYMBOL  BCP Higher Layer Symbols Defined
@ingroup BCP_API
*/

/**
@defgroup BCP_DATASTRUCT  BCP Higher Layer Data Structures
@ingroup BCP_API
*/

/**
@defgroup BCP_FUNCTION  BCP Higher Layer Functions
@ingroup BCP_API
*/

/**
@defgroup BCP_OSAL_FUNCTION  BCP OS Abstraction Layer Functions
@ingroup BCP_API
*/

#ifdef __cplusplus
extern "C"
{
#endif

/**
@addtogroup BCP_SYMBOL
@{
*/

/** @brief
 * The number of BCP instances in the SoC.
 */        
#define     BCP_MAX_NUM_INSTANCES               (CSL_BCP_PER_CNT)

/** @brief
 * The number of CPPI flows reserved for each BCP instance.
 */
#define     BCP_MAX_NUM_FLOWS                   (64)

/** @brief
 *  The maximum number of Rx objects that will be tracked
 *  by the driver globally in the system across all cores.
 */
#define     BCP_MAX_NUM_RXOBJECTS               (64)        

/** @brief  
 *
 *  When returned this, indicates the API succeeded in 
 *  its intent.
 */
#define     BCP_RETVAL_SUCCESS                  (0)

/** @brief  
 *
 *  When returned this, indicates that the handle passed
 *  to the API was invalid.
 */
#define     BCP_RETVAL_EBAD_HANDLE              (-1)

/** @brief  
 *
 *  When returned this, indicates that the input configuration
 *  passed to the API is invalid.
 */
#define     BCP_RETVAL_EINVALID_PARAMS          (-2)

/** @brief  
 *
 *  When returned this, indicates that the API's attempt to
 *  allocate memory or retrieve descriptors failed and 
 *  is out of memory/descriptors.
 */
#define     BCP_RETVAL_ENO_MEM                  (-3)

/** @brief  
 *
 *  When returned this, indicates that there is no pending
 *  BCP processed output available for the application's Rx object.
 */
#define     BCP_RETVAL_ENO_RESULT               (-4)

/** @brief  
 *
 *  When returned this, indicates that the API doesnt support
 *  the feature requested for the inputs passed.
 */
#define     BCP_RETVAL_ENOT_SUPPORTED           (-5)

/** @brief  
 *
 *  When returned this, indicates a general error.
 */
#define     BCP_RETVAL_EFAILURE                 (-6)

/**
@}
*/

/** @addtogroup BCP_DATASTRUCT
 @{ */

/** 
 *  @brief  Bcp_DrvHandle
 *          
 *          BCP driver handle.
 */    
typedef     void*               Bcp_DrvHandle;

/** 
 *  @brief  Bcp_TxHandle
 *          
 *          BCP Tx object handle.
 */    
typedef     void*               Bcp_TxHandle;

/** 
 *  @brief  Bcp_RxHandle
 *          
 *          BCP Rx object handle.
 */    
typedef     void*               Bcp_RxHandle;

/** 
 *  @brief  Bcp_DrvBufferHandle
 *          
 *          BCP driver buffer object handle.
 */    
typedef     void*               Bcp_DrvBufferHandle;

/** 
 *  @brief  Bcp_RetVal
 *          
 *          Holds the various error codes reported by 
 *          the BCP driver.
 *
 *          Please refer to BCP_RETVAL_* definitions for 
 *          the various error codes reported by this driver.
 */    
typedef     int32_t             Bcp_RetVal;

/** 
 *  @brief  Bcp_DrvMode
 *
 *          Enumeration for specifying the BCP driver
 *          operational modes. Indicates if BCP is present
 *          locally on the SoC this driver is being used on
 *          or if it is remote.
 */        
typedef enum   
{
    /** BCP is local to the device.  
     *
     *  BCP is present on the device (SoC) on which driver is being used
     *  currently. 
     */
    Bcp_DrvMode_LOCAL           = 0,

    /** BCP is remotely accessible to the device.  
     *
     *  BCP is not present on the device (SoC) on which driver is being used
     *  currently. It can be accessed via SRIO.
     */
    Bcp_DrvMode_REMOTE          = 1
} Bcp_DrvMode;

/** 
 *  @brief  Bcp_InitCfg
 *
 *          BCP Peripheral Initialization configuration.
 */    
typedef struct _Bcp_InitCfg
{
    /** CPDMA Number corresponding to this BCP instance. */
    Cppi_CpDma                  cpdmaNum;

    /** Tx queue number base for this BCP instance. */
    uint32_t                    baseTxQueueNum;

    /** Base address for BCP configuration registers for this instance. */
    uint32_t                    cfgRegsBaseAddress;

    /** Callback functions to BCP transport layer. */

    /** Called from Bcp_txOpen () API to initialize one endpoint of the Tx tunnel
     *  between device having BCP and remote device. Application developer should 
     *  implement the necessary logic here to perform any setup required to send 
     *  packets to BCP on the same/remote device as this device.
     */
    void*                       (*BcpTunnel_txOpen) (void* txEpCfg);

    /** Called from Bcp_txClose () API to close a BCP tunnel Tx endpoint. */
    int32_t                     (*BcpTunnel_txClose) (void* hTxEpInfo);

    /** Called from Bcp_rxOpen () API to initialize one endpoint of the Rx tunnel
     *  between device having BCP and remote device. Application developer should 
     *  implement the necessary logic here to perform any setup required to receive
     *  packets from BCP on the same/remote device as this device.
     */
    void*                       (*BcpTunnel_rxOpen) (void* rxEpCfg);

    /** Called from Bcp_rxClose () API to close a BCP tunnel Rx endpoint. */
    int32_t                     (*BcpTunnel_rxClose) (void* hRxEpInfo);

    /** Called from Bcp_send () API to send out a packet through the tunnel
     *  to a remote BCP device. Invoked by BCP driver only if it was initialized in 
     *  "remote" mode.
     */
    int32_t                     (*BcpTunnel_send) (void* hTxEpInfo, void* pPkt, uint32_t pktSize, void* pDestnDev);

    /** Called from Bcp_recv () API to receive output from BCP using tunnel from a remote device.
     *  Invoked by BCP driver only if it was initialized in "remote" mode.
     */
    int32_t                     (*BcpTunnel_recv) (void* hRxEpInfo, void** pPkt);

    /** Called from Bcp_rxFreeRecvBuffer () API to free an Rx packet obtained using Bcp_recv ()
     *  API. Invoked by BCP driver only if it was initialized in "remote" mode.
     */
    int32_t                     (*BcpTunnel_freeRecvBuffer) (void* hRxEpInfo, void* pPkt, uint32_t pktSize);
} Bcp_InitCfg;


/** 
 *  @brief  Bcp_DrvCfg
 *
 *          BCP Driver configuration.
 */    
typedef struct _Bcp_DrvCfg
{
    uint32_t                    dummy;        
} Bcp_DrvCfg;

/** 
 *  @brief  Bcp_TxCfg
 *
 *          BCP Tx object configuration.
 */    
typedef struct _Bcp_TxCfg
{
    /** BCP Tx queue number to use for sending data to BCP. 
     *
     *  Valid values are 0 - BCP_MAX_NUM_TXQUEUES
     */        
    Bcp_QueueId                 txQNum;
} Bcp_TxCfg;

/** 
 *  @brief  Bcp_RxCfg
 *
 *          BCP Rx object configuration.
 */    
typedef struct _Bcp_RxCfg
{
    /** Rx queue number on which data from BCP must be
     *  received. 
     *
     *  This can be set to -1 to let the driver pick a Rx
     *  queue number.
     */        
    int32_t                     rxQNum;

    /** CPPI Receive flow configuration. */            
    Cppi_RxFlowCfg              flowCfg;

    /** Corresponding BCP Traffic Manager flow configuration */
    Bcp_TmFlowEntry             tmFlowCfg;

    /** Boolean flag to indicate whether if interrupt support is 
     *  required for this Rx object.
     *
     *  When set to 1, indicates that accumulation interrupts must 
     *  be used for this Rx object.
     */
    uint8_t                     bUseInterrupts;

    /*  Accumulator configuration to use for this Rx object. */
    Qmss_AccCmdCfg              accumCfg;
} Bcp_RxCfg;

/**
@}
*/

extern uint32_t Bcp_getVersionID 
( 
    void
);

extern const char* Bcp_getVersionStr 
(
    void
);

extern Bcp_RetVal Bcp_init
(
    uint8_t                     instNum, 
    Bcp_DrvMode                 mode,
    Bcp_InitCfg*                pBcpInitCfg
);

extern Bcp_RetVal Bcp_deInit 
(
    uint8_t                     instNum
);

extern uint8_t Bcp_isInitialized
(
    uint8_t                     instNum
);

extern Bcp_LldObj* Bcp_getLLDHandle 
(
    uint8_t                     instNum
);

extern Bcp_DrvHandle Bcp_open 
(
    uint8_t                     instNum,
    Bcp_DrvCfg*                 pBcpDrvCfg,
    Bcp_RetVal*                 pRetVal
);

extern Bcp_RetVal Bcp_close 
(
    Bcp_DrvHandle               hBcp
);

extern Bcp_TxHandle Bcp_txOpen
(
    Bcp_DrvHandle               hBcp,
    Bcp_TxCfg*                  pBcpTxCfg,
    void*                       pTxEndpointCfg
);

extern Bcp_RetVal Bcp_txClose 
(
    Bcp_TxHandle                hBcpTxInfo
);

extern Bcp_RxHandle Bcp_rxOpen
(
    Bcp_DrvHandle               hBcp,
    Bcp_RxCfg*                  pBcpRxCfg,
    void*                       pRxEndpointCfg
);

extern Bcp_RetVal Bcp_rxClose 
(
    Bcp_RxHandle                hBcpRxInfo
);

extern Bcp_RetVal Bcp_rxGetRxQueueNumber
(
    Bcp_RxHandle                hBcpRxInfo
);

extern Bcp_RetVal Bcp_rxGetFlowId
(
    Bcp_RxHandle                hBcpRxInfo
);

extern Bcp_RetVal Bcp_findFlowIdByQueueNumber
(
    Bcp_DrvHandle               hBcp,
    uint32_t                    rxQueueNumber
);

extern Bcp_RetVal Bcp_send 
(
    Bcp_TxHandle                hBcpTxInfo,
    Bcp_DrvBufferHandle         hDrvBuffer,
    uint32_t                    drvBufferLen,
    void*                       pDestnAddress
);

extern int32_t Bcp_rxGetNumOutputEntries 
(
    Bcp_RxHandle                hBcpRxInfo
);

extern Bcp_RetVal Bcp_recv
(
    Bcp_RxHandle                hBcpRxInfo,
    Bcp_DrvBufferHandle*        phDrvBuffer,
    uint8_t**                   ppDataBuffer,
    uint32_t*                   pDataBufferLen,                   
    uint8_t**                   ppPSInfo,
    uint32_t*                   pPSInfoLen,
    uint8_t*                    pFlowId,
    uint8_t*                    pSrcId,
    uint16_t*                   pDestnTagInfo
);

extern Bcp_RetVal Bcp_rxProcessDesc 
(
    Bcp_RxHandle                hBcpRxInfo,
    Cppi_Desc*                  pCppiDesc,
    Bcp_DrvBufferHandle*        phDrvBuffer,
    uint8_t**                   ppDataBuffer,
    uint32_t*                   pDataBufferLen,                   
    uint8_t**                   ppPSInfo,
    uint32_t*                   pPSInfoLen,
    uint8_t*                    pFlowId,
    uint8_t*                    pSrcId,
    uint16_t*                   pDestnTagInfo
);

extern Bcp_RetVal Bcp_rxFreeRecvBuffer
(
    Bcp_RxHandle                hBcpRxInfo,
    Bcp_DrvBufferHandle         hDrvBuffer,
    uint32_t                    drvBufferLen
);

#ifdef __cplusplus
}
#endif

#endif  /* __BCP_H__ */

