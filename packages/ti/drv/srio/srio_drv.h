/**
 *   @file  srio_drv.h
 *
 *   @brief
 *      Header file for the SRIO Driver. The file exposes the data structures
 *      and exported API which are available for use by the driver users.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009-2014 Texas Instruments, Inc.
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
 *  \par
*/

/** @defgroup SRIO_LLD_API SRIO LLD
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *  The SRIO Low Level driver provides a well defined standard interface
 *  which allows application developers to send and receive messages via
 *  the Serial RapidIO peripheral. 
 */

#ifndef __SRIO_DRV_H__
#define __SRIO_DRV_H__

/* CPPI Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* CSL SRIO Header File  */
#include <ti/csl/csl_srio.h>

#include <ti/drv/srio/sriover.h>

/**
@defgroup SRIO_LLD_SYMBOL  SRIO LLD Symbols Defined
@ingroup SRIO_LLD_API
*/
/**
@defgroup SRIO_LLD_FUNCTION  SRIO LLD Functions
@ingroup SRIO_LLD_API
*/
/**
@defgroup SRIO_LLD_DATASTRUCT  SRIO LLD Data Structures
@ingroup SRIO_LLD_API
*/
/**
@defgroup SRIO_OSAL_API  SRIO OSAL Functions
@ingroup SRIO_LLD_API
*/
/**
@defgroup SRIO_DEVICE_API  SRIO Device Functions
@ingroup SRIO_LLD_API
*/

/** @addtogroup SRIO_LLD_SYMBOL
 @{ */

/* This macro generates compilier error if postulate is true, so 
 * allows 0 overhead compile time size check.  This "works" when
 * the expression contains sizeof() which otherwise doesn't work
 * with preprocessor */
#define SRIO_COMPILE_TIME_SIZE_CHECK(postulate)                              \
   do {                                                                      \
       typedef struct {                                                      \
         uint8_t SrioNegativeSizeIfPostulateTrue[1 - ((int)(postulate))*2];  \
       } SrioPostulateCheck_t;                                               \
   }                                                                         \
   while (0)

#define SRIO_MAX_CACHE_ALIGN     128 /* Maximum alignment for cache line size */

/**
 * @brief   This defines the maximum depth of the SRIO socket receive queues.
 * This is the MAX number of packets which can be enqueued in the SRIO socket
 * receive queue before packets get dropped.
 */
#define DEFAULT_SRIO_MAX_PENDING_PACKETS       5

/**
 * @brief This is the maximum number of Type9 and Type11 sockets that can 
 * be created. This is limited by the hardware and since Type9 and Type11 
 * share the same QID_MAP register both of them are limited to this 
 */
#define NUM_SRIO_TYPE9_TYPE11_SOCKETS          64

/**
 * @brief This is the maximum number of DIO sockets that can be created. This 
 * limit is specified by the number of LSU blocks. 
 */
#define NUM_DIO_SOCKETS                        8

/**
 * @brief This is a macro provided for the application and should be used if a DOORBELL
 * is to be transmitted.
 *
 * @sa
 *  Srio_sockSend 
 */
#define SRIO_SET_DBELL_INFO(DBELL_REG, DBELL_BIT)  CSL_FMKR(31, 16, DBELL_REG) | \
                                                   CSL_FMKR(15,  0, DBELL_BIT)

/**
 * @brief This is a macro provided for the application and should be used to get
 * the doorbell register information once data is received on the DIO socket.
 *
 * @sa
 *  Srio_sockRecv
 */
#define SRIO_GET_DBELL_REG(DBELL_INFO)  CSL_FEXTR(DBELL_INFO, 31, 16)

/**
 * @brief This is a macro provided for the application and should be used to get
 * the doorbell bit information once data is received on the DIO socket.
 *
 * @sa
 *  Srio_sockRecv
 */
#define SRIO_GET_DBELL_BIT(DBELL_INFO)  CSL_FEXTR(DBELL_INFO, 15,  0)

/**
 * @brief Specifies to use hardware assigned Letter to which the message will be send. 
 * The hardware will check for an unused context starting with letter = 0 (A), 
 * and incrementing to letter = 3 (D). The first unused context with that letter 
 * will be used. If there are no context available with any letters then the 
 * packet is stopped and re-arbitrated in the TXU until one does get available.
 */
#define SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE     4

/**
 * @brief This value can be used to accept all mailbox values on receive side. 
 * Setting a mailbox number to this value will set MBX_MASK to "0x000000" in 
 * RIO_RXU_MAPxx_L register.
 */
#define SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE     0xFFFF

/**
@}
*/

/** @addtogroup SRIO_LLD_DATASTRUCT
 @{ */

/** 
 * @brief SRIO RM Service Handle
 */
typedef void *  Srio_RmServiceHnd;

/** 
 * @brief 
 *  This is the handle which is used for sending and receiving data 
 */
typedef void*   Srio_SockHandle;

/** 
 * @brief 
 *  This is the handle which is used accessing the SRIO driver.
 */
typedef void*   Srio_DrvHandle;

/** 
 * @brief 
 *  This is the handle which encapsulates the SRIO driver buffer information.
 */
typedef void*   Srio_DrvBuffer;

/** 
 * @brief 
 *  The structure describes the SRIO Driver Managed Receive Configuration
 *
 * @details
 *  This specifies the Receive configuration which is a part of the Driver 
 *  Managed configuration. 
 */
typedef struct Srio_DrvManagedRxCfg
{
    /**
     * @brief   This is the memory region to be used for allocating 
     * the receive buffer descriptors. 
     */
    Qmss_MemRegion      rxMemRegion;

    /**
     * @brief   This is the Number of receive buffers and descriptors 
     * which are to be passed to the SRIO receive queues.
     */
    uint32_t            numRxBuffers;

    /**
     * @brief   For Normal sockets this is the Maximum data size which can be
     * received.
     */
    uint32_t            rxMTU;

    /**
     * @brief   This is the receive completion queue in which the received
     * SRIO packets will be placed. If interrupt support is required then the
     * application would need to ensure that they select a correct high priority
     * queue & accumulator channel. If no interrupts are required then this can
     * be any queue. 
     */
    Qmss_QueueHnd       rxCompletionQueue;

    /**
     * @brief   Boolean flag which indicates if the SRIO driver should configure
     * the accumulator with the provided accumulator configuration or not. If this
     * parameter is set to 0 the accumulator configuration below is ignored. This
     * in turn implies that there is no interrupt support and the application would
     * need to poll.
     */
    uint16_t            bIsAccumlatorCfgValid;

    /**
     * @brief   Accumulator Configuration is exposed to the application which 
     * allows the application to determine the parameters for programming 
     * the accumulator. 
     * Note: SRIO driver expects accumulator list (Qmss_AccCmdCfg.listAddress)
     * to be allocated from local memory (un-cached memory) for performance 
     * reasons. Using local memory reduces the overhead of cache invalidates 
     * on every Srio_rxCompletionIsr() call.
     */
    Qmss_AccCmdCfg      accCfg;
}Srio_DrvManagedRxCfg;

/** 
 * @brief 
 *  The structure describes the SRIO Driver Managed Transmit Configuration
 *
 * @details
 *  This specifies the Transmit configuration which is a part of the Driver 
 *  Managed configuration. 
 */
typedef struct Srio_DrvManagedTxCfg
{
    /**
     * @brief  The number of transmit buffers available to the driver instance
     * which can be used to transmit data.
     */
    uint32_t            numTxBuffers;

    /**
     * @brief   This is the memory region to be used for allocating the transmit 
     * buffer descriptors.
     */
    Qmss_MemRegion      txMemRegion;

    /**
     * @brief   For Normal sockets this is the Maximum data size which can be
     * transmitted.
     */
    uint32_t            txMTU;
}Srio_DrvManagedTxCfg;

/** 
 * @brief 
 *  The structure describes the SRIO Driver Managed Configuration
 *
 * @details
 *  The configuration exposes encapsulates majority of the low level configuration
 *  from the application. The configuration works only with NORMAL sockets. 
 */
typedef struct Srio_DrvManagedCfg
{
    /**
     * @brief   Boolean flag which indicates if the SRIO driver instance being
     * configured should support receive or not? If this flag is set to 0 the
     * Receive configuration below is ignored which implies that the the 
     * receive flow is not configured and thus the driver instance and any 
     * associated sockets opened on this instance are no longer capable of 
     * receiving any data.
     */
    uint16_t                bIsRxCfgValid;

    /**
     * @brief   The receive configuration which determines the location of
     * the buffer descriptors, size, receive MTU etc.
     */
    Srio_DrvManagedRxCfg    rxCfg;

    /**
     * @brief   Boolean flag which indicates if the SRIO driver instance being
     * configured should support transmit or not? If this flag is set to 0 the
     * transmit configuration below is ignored which implies that the driver 
     * instance does not have any transmit buffers available and thus any call 
     * to send out data will fail. 
     */
    uint16_t                bIsTxCfgValid;    

    /**
     * @brief   The transmit configuration which determines the location of
     * the buffer descriptors, size, transmit MTU etc.
     */
    Srio_DrvManagedTxCfg    txCfg;
}Srio_DrvManagedCfg;

/**
 * @brief
 *  SRIO priority level set in the TCHAN_SCHED_CFG_REGn registers.
 *
 * @details
 *  This enumeration provides the SRIO PKTDMA TX DMA channels priority.
 *  The value is encoded as follows: 
 *  0 = HIGH PRIORITY,   1 = MEDIUM-HIGH PRIORITY,
 *  2 = MEDIUM-LOW PRIO, 3 = LOW PRIORITY
 *
 *  The priority order from the CDMA is in the reverse order from 
 *  the SRIO. Thus, "0" maps to "3", "1" to "2", "2" to "1" and "3"
 *  to "0". The inversed priority level (3 --> 0, 2 --> 1 etc.) is 
 *  copied to the TX_QUEUE_SCH_INFOx register and used by the SRIO 
 *  IP when forming SRIO headers.
 */
typedef enum
{
    /*
    * @brief 0 = High Priority
    */
    Srio_PktDma_Prio_High = 0,

    /*
    * @brief 1 = Medium-High Priority
    */
    Srio_PktDma_Prio_MediumHigh,

    /*
    * @brief 2 = Medium-Low Priority
    */
    Srio_PktDma_Prio_MediumLow,

    /*
    * @brief 3 = Low Priority
    */
    Srio_PktDma_Prio_Low
}Srio_PktDma_Prio;

/** 
 * @brief
 *  The structure describes the application managed configuration
 *
 * @details
 *  In this configuration the entire low level configuration is exposed to the
 *  application. Applications can specify the CPPI Receive Flows, QMSS Accumulator
 *  configuration. This configuration works only with RAW sockets. 
 */
typedef struct Srio_AppManagedCfg
{
    /**
     * @brief   Boolean flag which indicates if the SRIO driver instance being
     * configured should support receive or not? If this flag is set to 0 the
     * Receive Flow configuration below is ignored which implies that the 
     * the receive flow is not configured and thus the driver instance and any 
     * associated sockets opened on this instance are no longer capable of 
     * receiving any data.
     */
    uint16_t            bIsRxFlowCfgValid;

    /**
     * @brief   The Receive Flow Configuration is exposed to the application. The 
     * Application specifies how flows need to be configured. This allows the 
     * applications complete control over the queues from where the buffer 
     * descriptors are removed when packets are received. 
     */
    Cppi_RxFlowCfg      rxFlowCfg;

    /**
     * @brief   Boolean flag which indicates if the SRIO driver should configure
     * the accumulator with the provided accumulator configuration or not. If this
     * parameter is set to 0 the accumulator configuration below is ignored.
     */
    uint16_t            bIsAccumlatorCfgValid;

    /**
     * @brief   Accumulator Configuration is exposed to the application which 
     * allows the application to determine the parameters for programming 
     * the accumulator. 
     */
    Qmss_AccCmdCfg      accCfg;

    /**
     * @brief   For RAW Sockets this is the receive cleanup API which needs to be 
     * provided by the application. This API is invoked by the driver to cleanup 
     * the buffer descriptor associated with the RAW socket. This parameter can 
     * be set to NULL if the application wishes to only operate on NORMAL sockets
     */
    void                (*rawRxFreeDrvBuffer)(Srio_DrvBuffer hDrvBuffer);

    /**
     * @brief   Same as rawRxFreeDrvBuffer except takes an additional argument
     * of type void *.  The argument is supplied via the rawRxFreeDrvBufferFuncArg
     * parameter.  The SRIO driver will always call the rawRxFreeDrvBuffer function
     * if this parameter is left as NULL.  The additional void * argument allows
     * the application to provide additional control information that may be needed
     * in order to free the driver buffer
     */
    void                (*rawRxFreeDrvBufferArg)(void *arg, Srio_DrvBuffer hDrvBuffer);

    /**
     * @brief   Pointer to an object that will be passed as part of the
     * rawRxFreeDrvBufferArg function.
     */
    void                *rawRxFreeDrvBufferFuncArg;
    
    /**
     * @brief   Indicates the queue number to be used for TX. Using this 
     * parameter same TX queue can be used for multiple SRIO driver instances.
     * This parameter needs to be set to either a valid queue number or 
     * QMSS_PARAM_NOT_SPECIFIED which indicates driver should allocate 
     * the next available queue.
     */
    int16_t             txQueueNum;

    /**
     * @brief   Receive Descriptor Size. This is required to 
     * invalidate cache on the receive side. If this is set to 
     * zero then invalidate will not take place e.g. in case of 
     * buffer descriptors that are allocated from L2 SARAM 
     * which don't need invalidation.
     */
    int32_t             rxDescSize;

    /**
     * @brief
     *  SRIO priority level set in the TCHAN_SCHED_CFG_REGn registers.
     *  This value provides the SRIO PKTDMA TX DMA channels priority.
     *  The value is encoded as follows: 
     *  0 = HIGH PRIORITY,   1 = MEDIUM-HIGH PRIORITY,
     *  2 = MEDIUM-LOW PRIO, 3 = LOW PRIORITY
     *
     *  The priority order from the CDMA is in the reverse order from 
     *  the SRIO. Thus, "0" maps to "3", "1" to "2", "2" to "1" and "3"
     *  to "0". The inversed priority level (3 --> 0, 2 --> 1 etc.) is 
     *  copied to the TX_QUEUE_SCH_INFOx register and used by the SRIO 
     *  IP when forming SRIO headers.
     */
    Srio_PktDma_Prio  srioPktDmaTxPrio;

}Srio_AppManagedCfg;

/**
 * @brief 
 *  Describes driver configuration.
 *
 * @details
 *  There are 2 types of configuration in the driver. Application Managed
 *  and Driver Managed. 
 */
typedef union Srio_DrvConfigType
{
    /**
     * @brief    This is the driver managed configuration.
     */
    Srio_DrvManagedCfg    drvManagedCfg;

    /**
     * @brief    This is the application managed configuration.
     */    
    Srio_AppManagedCfg    appManagedCfg;
} Srio_DrvConfigType;

/**
 * @brief 
 *  The structure describes the SRIO Driver Configuration 
 *
 * @details
 *  SRIO Driver users are expected to populate the driver configuration
 *  block and pass it to the driver during initialization.
 */
typedef struct Srio_DrvConfig
{
    /**
     * @brief   The SRIO driver can be configured to use either of the
     * following configurations:
     *  - Application Managed configuration
     *  - Driver Managed configuration
     *  This flag can be used to select which configuration is specified.
     */
    uint16_t            bAppManagedConfig;

    /**
     * @brief	This value specifies the queue base for the SRIO driver
     * to use. If left unassigned, SRIO driver will default to
     * SRIO_QUEUE_BASE in srio_drv.c. This default value is 672.
     */
    uint16_t			srioQueueBase;

    /**
     * @brief	This value specifies the number of SRIO queue in the
     * device. If left unassigned, SRIO driver will default to
     * SRIO_QUEUE_COUNT in srio_drv.c. This default value is 16.
     */
    uint16_t			srioQueueCount;

    /**
     * @brief    Union structure for the driver configuration.
     */
    Srio_DrvConfigType  u;

    /**
     * @brief    Resource Manager service handle.  Used for everyting 
     *           except Srio_initCfg() and Srio_close().
     *           This is passed through Srio_start()
     *           and stored once per Srio_DrvHandle.  This is typically once
     *           per Linux task, and once per DSP core within the DSP
     *           subystem.
     */    
    Srio_RmServiceHnd     rmServiceHandle;
}Srio_DrvConfig;

/**
 * @brief 
 *  The structure describes the SRIO Driver Configuration 
 *
 * @details
 *  SRIO Driver users are expected to populate the driver configuration
 *  block and pass it to the driver during initialization.
 */
typedef struct Srio_InitConfig
{
    /**
     * @brief    Resource Manager service handle.  Used only 
     *           for Srio_initCfg and Srio_close.  This is passed through
     *           Srio_initCfg() and stored once per gSRIODriverMCB which
     *           is once per Linux task and typically (but not always) once
     *           per multicore DSP subsystem.
     */    
    Srio_RmServiceHnd     rmServiceHandle;
    /**
     * @brief    Virtual address for SRIO configuration registers
     *           This MUST be specified in Linux since physical address
     *           is unknown.
     */    
#ifdef __LINUX_USER_SPACE
    CSL_SrioHandle       srioCfgVAddr;
#endif
} Srio_InitConfig;

/** 
 * @brief 
 *  Enumeration Type which describes the socket.
 *
 * @details
 *  There can be different kinds of SRIO sockets which can be used to 
 *  send and receive data. These enumerations define the supported 
 *  types.
 */
typedef enum Srio_SocketType
{
    /**
     * @brief   Type9 Sockets
     */
    Srio_SocketType_TYPE9       = 0x1,

    /**
     * @brief   Type9 RAW Sockets
     */
    Srio_SocketType_RAW_TYPE9   = 0x2,
    
    /**
     * @brief   Type11 Sockets
     */
    Srio_SocketType_TYPE11      = 0x3,

    /**
     * @brief   Type11 RAW Sockets
     */
    Srio_SocketType_RAW_TYPE11  = 0x4,

    /**
     * @brief   Direct IO Socket.
     */        
    Srio_SocketType_DIO         = 0x5
}Srio_SocketType;

/** 
 * @brief 
 *  SRIO Socket Type11 Binding Information.
 *
 * @details
 *  The structure describes the address information required for binding 
 *  a Type11 socket. This includes information which describes the Type11
 *  endpoint characteristics and is used to describe the local characteristics 
 *  of the endpoint.
 */
typedef struct Srio_Type11BindAddrInfo
{
    /**
     * @brief   Transport Type; 16 bit or 8 bit identifiers. 
     */
    uint16_t      tt;

    /**
     * @brief   This is the 8 bit or 16 bit SRIO identifier 
     */
    uint16_t      id;

    /**
     * @brief   Letter Identifier 
     */
    uint16_t      letter;

    /**
     * @brief   Mailbox number 
     */
    uint16_t      mbox;

    /**
     * @brief   Segmentation Mapping Set to 0 for single segment and 1 for multi segment.
     */
    uint16_t      segMap;
}Srio_Type11BindAddrInfo;

/** 
 * @brief 
 *  SRIO Socket Type11 Address Information.
 *
 * @details
 *  The structure describes the address information required to send & 
 *  receive a Type11 message over a Type11 socket. This is populated to
 *  indicate the remote endpoint where the message has to be sent.
 */
typedef struct Srio_Type11AddrInfo
{
    /**
     * @brief   Transport Type; 16 bit or 8 bit identifiers. 
     */
    uint16_t      tt;

    /**
     * @brief   This is the 8 bit or 16 bit SRIO identifier 
     */
    uint16_t      id;

    /**
     * @brief   Letter Identifier 
     */
    uint16_t      letter;

    /**
     * @brief   Mailbox number 
     */
    uint16_t      mbox;
}Srio_Type11AddrInfo;

/** 
 * @brief 
 *  SRIO Socket Type9 Information.
 *
 * @details
 *  The structure describes the address information required to send & 
 *  receive a Type11 message over a Type11 socket. This is populated to
 *  indicate the remote endpoint where the message has to be sent.
 */
typedef struct Srio_Type9AddrInfo
{
    /**
     * @brief   Transport Type; 16 bit or 8 bit identifiers. 
     */
    uint16_t        tt;
 
    /**
     * @brief   This is the 8 bit or 16 bit SRIO identifier 
     */
    uint16_t        id;

    /**
     * @brief   Class of service
     */
    uint8_t         cos;

    /**
     * @brief   Stream identifier.
     */
    uint16_t        streamId;
}Srio_Type9AddrInfo;

/** 
 * @brief 
 *  SRIO Socket Type9 Binding Information.
 *
 * @details
 *  The structure describes the address information required for binding 
 *  a Type9 socket. This includes information which describes the Type9
 *  endpoint characteristics and is used to describe the local characteristics 
 *  of the endpoint. 
 */
typedef Srio_Type9AddrInfo  Srio_Type9BindAddrInfo;

/** 
 * @brief 
 *  SRIO Socket DIO Binding Information.
 *
 * @details
 *  The structure describes the address information required for binding 
 *  a DIO socket. This includes information which describes the DIO
 *  endpoint characteristics and is used to describe the local characteristics 
 *  of the endpoint. 
 */
typedef struct Srio_DioBindAddrInfo
{
    /**
     * @brief   Indicates if doorbell information needs to be sent out or not.
     */
    uint8_t   doorbellValid;

    /**
     * @brief   CPU controlled request bit used for interrupt generation
     */
    uint8_t   intrRequest;

    /**
     * @brief   Supress good interrupt.
     */
    uint8_t   supInt;

    /**
     * @brief   RapidIO xambs field specifying extended address 
     */
    uint8_t   xambs;

    /**
     * @brief   Packet Priority
     */
    uint8_t   priority;

    /*
     * @brief Indicates the output port number for the packet to be transmitted
     */
    uint8_t   outPortID;

    /*
     * @brief RapidIO tt field specifying 8 or 16bit DeviceIDs
     */    
    uint8_t   idSize;

    /*
     * @brief Defines which sourceID register to be used for this transaction
     */
    uint8_t   srcIDMap;

    /*
     * @brief RapidIO hop_count field specified for Type 8 Maintenance packets
     */    
    uint8_t   hopCount;

    /*
     * @brief RapidIO doorbell info: This is the doorbell register which is to be written 
     * There are 4 registers so this should have a value from 0 - 3.
     */
    uint8_t  doorbellReg;

    /*
     * @brief RapidIO doorbell info: This is the doorbell bit which is to be set. There 
     * are 16 doorbell bits so this should have a value from 0-15.
     */
    uint8_t  doorbellBit;
}Srio_DioBindAddrInfo;

/** 
 * @brief 
 *  SRIO Socket DIO Information
 *
 * @details
 *  The structure describes the DIO request which has to be sent to the remote
 *  endpoint.
 */
typedef struct Srio_DioAddrInfo
{
    /**
     * @brief   32b Ext Address Fields – Packet Types 2,5, and 6
     */
    uint32_t  rapidIOMSB;

    /**
     * @brief   32b Address – Packet Types 2,5, and 6 
     */
    uint32_t  rapidIOLSB;

    /*
     * @brief RapidIO destinationID field specifying target device
     */
    uint16_t  dstID;

    /*
     * @brief Transaction Type
     */
    uint8_t   ttype;

    /*
     * @brief FType for packets
     */
    uint8_t   ftype;
}Srio_DioAddrInfo;

/** 
 * @brief 
 *  SRIO Socket TX Queue Information
 *
 * @details
 *  The structure describes the transmission queue to use and allows
 *  for updating the socket with Srio_setSockOpt.
 */
typedef struct Srio_txQueue
{
    /**
     * @brief   16b number for the queue to open. This number should be based
     * on a device-specific QMSS_SRIO_QUEUE_BASE, with a maximum offset amount
     * of QMSS_MAX_SRIO_QUEUE.
     */
    uint16_t	txQueueNum;

    /**
     * @brief   8b number to describe the queue priority
     */
    uint8_t		txChPriority;
}Srio_txQueueCfg;

/**
 * @brief
 *  SRIO Socket Bind Information
 *
 * @details
 *  There are different types of sockets and this union explains the different 
 *  types of binding information required.
 */
typedef union Srio_SockBindAddrInfo
{
    Srio_Type11BindAddrInfo         type11;
    Srio_Type9BindAddrInfo          type9;
    Srio_DioBindAddrInfo            dio;
}Srio_SockBindAddrInfo;

/** 
 * @brief 
 *  SRIO Socket Address Information
 *
 * @details
 *  The structure describes the various address socket type address characteristics
 *  which are used while sending & receiving data over the specific SRIO socket type.
 */
typedef union Srio_SockAddrInfo
{
    Srio_Type11AddrInfo         type11;
    Srio_Type9AddrInfo          type9;
    Srio_DioAddrInfo            dio;
}Srio_SockAddrInfo;

/** 
 * @brief 
 *  SRIO Driver Option Commands.
 *
 * @details
 *  These option commands are used for the get/set of various configuration
 *  parameters which exist in the driver.
 */
typedef enum Srio_Opt
{
    /**
     * @brief   This is the command which is used to get/set the MAX Pending
     * Packet limit for each socket. This command when used requires a 
     * 2 byte configuration data.
     */
    Srio_Opt_PENDING_PKT_COUNT       = 0x1,

    /**
     * @brief   This command is applicable only for DIO sockets and is used to
     * get the DIO socket last transfer completion code. If there is a pending 
     * transaction on the socket the function returns 0xFF else the function 
     * returns the last recorded completion code. The command uses a 1 byte 
     * configuration data to return the completion code. A value of 0 indicates 
     * transfer was complete with no errors. All other values indicate an error.
     */
    Srio_Opt_DIO_SOCK_COMP_CODE      = 0x2,

    /**
     * @brief   This command is applicable only for DIO sockets and is used 
     * register a DIO socket with a specific Doorbell register and Doorbell
     * bit. The mappings are maintained *only* on a core specific basis. 
     */
    Srio_Opt_REGISTER_DOORBELL       = 0x3,

    /**
     * @brief   This command is applicable only for DIO sockets and is used to
     * get the DIO socket transfer completion code. Note that this just returns 
     * the last recorded completion code in the socket data structure and doesn't 
     * check if transaction is pending or not. A typical use of this option would
     * be in the case where an ISR fills the completion code and application needs  
     * to know the status of completion code after ISR. The command uses a 1 byte 
     * configuration data to return the completion code. A value of 0 indicates 
     * transfer was complete with no errors. All other values indicate an error.
     */
    Srio_Opt_DIO_READ_SOCK_COMP_CODE = 0x4,

    /**
     * @brief	This command is for changing the txQueue and its associated CPPI
     * txChHnd on a given socket instance. The txQueue and txChHnd default when
     * opening the socket comes from the driver instance. This option allows for
     * changing the transmit to a different queue by opening the new queue and
     * CPPI channel base on the txQueueNum. Queue close and channel close will only
     * be called if the updated queue number is different from the txQueue in the
     * driver instance.
     */
    Srio_Opt_UPDATE_TXQUEUE = 0x5

}Srio_Opt;

/** 
 * @brief 
 *  RIO Format Type
 *
 * @details
 *  This enumberation describes the SRIO Packet Ftype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum 
{  
    /*
     * @brief Type 2 Packet Format (Request Class)
     */
    Srio_Ftype_REQUEST        = 2,

    /*
     * @brief Type 5 Packet Format (Write Class)
     */
    Srio_Ftype_WRITE          = 5,

    /*
     * @brief Type 6 Packet Format (Streaming Write Class)
     */
    Srio_Ftype_SWRITE         = 6,

    /*
     * @brief Type 7 Packet Format (Congestion Class)
     */
    Srio_Ftype_CONGESTION     = 7,

    /*
     * @brief Type 8 Packet Format (Maintenance)
     */
    Srio_Ftype_MAINTENANCE   = 8,

    /*
     * @brief Type 9 Packet Format (Data Streaming)
     */
    Srio_Ftype_DATA_STREAMING = 9,
    
    /*
     * @brief Type 10 Packet Format (Doorbell)
     */
    Srio_Ftype_DOORBELL       = 10,

    /*
     * @brief Type 11 Packet Format (Doorbell)
     */
    Srio_Ftype_MESSAGE        = 11,

    /*
     * @brief Type 13 Packet Format Response)
     */
    Srio_Ftype_RESPONSE       = 13
} Srio_Ftype;

/** 
 * @brief 
 *  RIO Transaction Type for Type2 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief NREAD Transaction
     */
    Srio_Ttype_Request_NREAD        = 4,

    /*
     * @brief ATOMIC Increment Transaction
     */
    Srio_Ttype_Request_ATOMIC_INC   = 12,

    /*
     * @brief ATOMIC Decrement Transaction
     */
    Srio_Ttype_Request_ATOMIC_DEC   = 13,

    /*
     * @brief ATOMIC Set Transaction
     */
    Srio_Ttype_Request_ATOMIC_SET   = 14,

    /*
     * @brief ATOMIC Clear Transaction
     */
    Srio_Ttype_Request_ATOMIC_CLR   = 15
}Srio_Ttype_Request;

/** 
 * @brief 
 *  RIO Transaction Type for Type5 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief NWRITE Transaction
     */
    Srio_Ttype_Write_NWRITE             = 4,

    /*
     * @brief NWRITE_R Transaction
     */
    Srio_Ttype_Write_NWRITE_R           = 5,

    /*
     * @brief Atomic Test and Set Transaction
     */
    Srio_Ttype_Write_ATOMIC_TEST_SET    = 14
}Srio_Ttype_Write;

/** 
 * @brief 
 *  RIO Transaction Type for Type6 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Streaming Write Transaction there is no Transaction Type.
     */
    Srio_Ttype_Swrite_DEFAULT            = 0
}Srio_Ttype_Swrite;

/** 
 * @brief 
 *  RIO Transaction Type for Type7 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Congestion Transaction there is no Transaction Type.
     */
    Srio_Ttype_Congestion_DEFAULT            = 0
}Srio_Ttype_Congestion;

/** 
 * @brief 
 *  RIO Transaction Type for Type8 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief Maintenance Read
     */
    Srio_Ttype_Maintenance_READ           = 0,

    /*
     * @brief Maintenance Write
     */
    Srio_Ttype_Maintenance_WRITE          = 1,

    /*
     * @brief Maintenance Read Response
     */
    Srio_Ttype_Maintenance_READR          = 2,

    /*
     * @brief Maintenance Write Response
     */
    Srio_Ttype_Maintenance_WRITER         = 3,

    /*
     * @brief Maintenance Port Write Response
     */
    Srio_Ttype_Maintenance_RORT_WRITE     = 4
}Srio_Ttype_Maintenance;

/** 
 * @brief 
 *  RIO Transaction Type for Type9 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Data Streaming there is no Transaction Type.
     */
    Srio_Ttype_Data_Streaming_DEFAULT      = 0
}Srio_Ttype_Data_Streaming;

/** 
 * @brief 
 *  RIO Transaction Type for Type10 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Doorbell there is no Transaction Type.
     */
    Srio_Ttype_Doorbell_DEFAULT            = 0
}Srio_Ttype_Doorbell;

/** 
 * @brief 
 *  RIO Transaction Type for Type11 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief For Message there is no Transaction Type.
     */
    Srio_Ttype_Message_DEFAULT            = 0
}Srio_Ttype_Message;

/** 
 * @brief 
 *  RIO Transaction Type for Type13 Packet Format
 *
 * @details
 *  This enumberation describes the SRIO Packet Ttype field which is present in the
 *  RIO packet. These are as per the Rapid IO Standard specifications.
 */
typedef enum
{
    /*
     * @brief Response + Doorbell response
     */
    Srio_Ttype_Response_RESPONSE            = 0,

    /*
     * @brief Message Response
     */
    Srio_Ttype_Response_MSG_RESPONSE        = 1,

    /*
     * @brief Response with payload
     */
    Srio_Ttype_Response_RESPONSE_PAYLOAD    = 8
}Srio_Ttype_Response;

/**
@}
*/

/**********************************************************************
 **************************** EXPORTED API ****************************
 **********************************************************************/

extern int32_t Srio_initCfg          (Srio_InitConfig *ptr_cfg);
extern int32_t Srio_init             (void);
extern int32_t Srio_close            (void);
extern Srio_DrvHandle Srio_start     (Srio_DrvConfig* ptr_cfg);
extern int32_t Srio_stop             (Srio_DrvHandle hSrio);

extern Srio_SockHandle Srio_sockOpen (Srio_DrvHandle hSrio, Srio_SocketType type,uint16_t isBlocking);
extern int32_t Srio_sockBind         (Srio_SockHandle srioSock, Srio_SockBindAddrInfo* ptr_addrInfo);
extern int32_t Srio_sockBind_TYPE11  (Srio_SockHandle srioSock, Srio_SockBindAddrInfo* ptr_addrInfo);
extern int32_t Srio_sockBind_TYPE9   (Srio_SockHandle srioSock, Srio_SockBindAddrInfo* ptr_addrInfo);
extern int32_t Srio_sockBind_DIO     (Srio_SockHandle srioSock, Srio_SockBindAddrInfo* ptr_addrInfo);
extern int32_t Srio_sockSend         (Srio_SockHandle srioSock, Srio_DrvBuffer hBuffer, uint32_t size, Srio_SockAddrInfo* to);
extern int32_t Srio_sockSend_TYPE11  (Srio_SockHandle srioSock, Srio_DrvBuffer hBuffer, uint32_t size, Srio_SockAddrInfo* to);
extern int32_t Srio_sockSend_TYPE9   (Srio_SockHandle srioSock, Srio_DrvBuffer hBuffer, uint32_t size, Srio_SockAddrInfo* to);
extern int32_t Srio_sockSend_DIO     (Srio_SockHandle srioSock, Srio_DrvBuffer hBuffer, uint32_t size, Srio_SockAddrInfo* to);
extern int32_t Srio_sockRecv         (Srio_SockHandle srioSock, Srio_DrvBuffer* hDrvBuffer,Srio_SockAddrInfo* from);
extern void Srio_freeRxDrvBuffer     (Srio_SockHandle srioSock, Srio_DrvBuffer hDrvBuffer);
extern int32_t Srio_setSockOpt       (Srio_SockHandle srioSock, Srio_Opt option,void* optval,int32_t optlen);
extern int32_t Srio_getSockOpt       (Srio_SockHandle srioSock, Srio_Opt option,void* optval,int32_t optlen);
extern int32_t Srio_sockClose        (Srio_SockHandle srioSock);
extern int32_t Srio_sockClose_TYPE11 (Srio_SockHandle srioSock);
extern int32_t Srio_sockClose_TYPE9  (Srio_SockHandle srioSock);
extern int32_t Srio_sockClose_DIO    (Srio_SockHandle srioSock);

extern Srio_DrvBuffer Srio_allocTransmitBuffer (Srio_DrvHandle hSrioDrv, uint8_t** ptrData, uint32_t* bufferLen);
extern void Srio_freeTransmitBuffer            (Srio_DrvHandle hSrioDrv, Srio_DrvBuffer hDrvBuffer);

extern void Srio_dioCompletionIsr   (Srio_DrvHandle hSrioDrv, uint8_t intDstDoorbell[]);
extern void Srio_dioTxCompletionIsr (Srio_DrvHandle hSrioDrv, CSL_SrioHandle hSrioCSL);
extern void Srio_rxCompletionIsr    (Srio_DrvHandle hSrioDrv);

extern uint32_t Srio_getVersion (void);
extern const char* Srio_getVersionStr (void);

#endif /* __SRIO_DRV_H__ */

