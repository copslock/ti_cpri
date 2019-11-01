/**
 *   @file  multicoreLoopback.c
 *
 *   @brief   
 *      This is an example application which showcases how the SRIO Driver
 *      API can be used in a multiple core environment. 
 *
 *      The test suite sends/receives data as follows:-
 *
 *      CORE 1 --> CORE 2 --> CORE 3 --> CORE 0 --> CORE 1
 *
 *      At each step the received data is validated (Payload and Received
 *      Identifiers).
 *
 *      The goal of the test suite is to showcase that the SRIO Driver API
 *      are multi-core safe.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2014, Texas Instruments, Inc.
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

#ifndef __LINUX_USER_SPACE
#include <xdc/std.h>
#include <string.h>
#include <stdio.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h> 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h> 

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>
#include <xdc/cfg/global.h>

#else /* __LINUX_USER_SPACE */

#include <sched.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "fw_test.h"
#include "fw_mem_allocator.h"
#include "linuxutil.h"
#include "srio_test.h"
#endif /* __LINUX_USER_SPACE */

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

/* RM include */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>

/* CSL Include Files */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

/* Device specific include */
#include "srioPlatCfg.h"

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

#define NUM_HOST_DESC               32
#define SIZE_HOST_DESC              48
#define SRIO_MAX_MTU                256

/* Number of cores for which the test is being executed. */
#define NUM_CORES                   srio_EXAMPLE_NUM_CORES

#if NUM_CORES > 1
/* Defines the core number responsible for system initialization. */
#define CORE_SYS_INIT               1
#else
#define CORE_SYS_INIT               0
#endif


/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/


Qmss_InitCfg   qmssInitConfig;
Qmss_Result    qmssRegion;

/* Global Varialble which keeps track of the core number executing the
 * application. */
uint32_t        coreNum = 0xFFFF;


/* These are the device identifiers used used in the TEST Application */
const uint32_t DEVICE_ID1_16BIT    = 0xBEEF;
const uint32_t DEVICE_ID1_8BIT     = 0xAB;
const uint32_t DEVICE_ID2_16BIT    = 0x4560;
const uint32_t DEVICE_ID2_8BIT     = 0xCD;
#if (NUM_CORES > 2)
const uint32_t DEVICE_ID3_16BIT    = 0x1234;
const uint32_t DEVICE_ID3_8BIT     = 0x12;
const uint32_t DEVICE_ID4_16BIT    = 0x5678;
const uint32_t DEVICE_ID4_8BIT     = 0x56;
#endif

#ifdef __LINUX_USER_SPACE
/* Memory used for the descriptors. */
uint8_t                *host_region;

Qmss_QueueHnd           qpendRxQueHnd;
int                     qpendFd;
int                     srioFd;
Srio_DrvHandle          isr_hSrioDriver;
#else
/* Memory used for the descriptors. */
#pragma DATA_ALIGN (host_region, 16)
uint8_t        host_region[NUM_HOST_DESC * SIZE_HOST_DESC];

uint32_t       errorCount = 0;

/* Memory used for the accumulator list. */
#pragma DATA_ALIGN (gHiPriAccumList, 16)
uint32_t        gHiPriAccumList[32];

/* RM instance handle */
Rm_Handle                   rmHandle = NULL;
Rm_ServiceHandle           *rmClientServiceHandle = NULL;
#endif

/**********************************************************************
 ************************* Extern Definitions *************************
 **********************************************************************/

extern uint32_t malloc_counter;
extern uint32_t free_counter;

extern int32_t SrioDevice_init (
#ifdef __LINUX_USER_SPACE
CSL_SrioHandle hSrio, 
uint32_t srioSerdesVAddr
#else
void
#endif
);
extern int32_t SrioDevice_deinit (
#ifdef __LINUX_USER_SPACE
CSL_SrioHandle hSrio
#else
void
#endif
);
extern void* Osal_srioDataBufferMalloc(uint32_t numBytes);


/* OSAL Data Buffer Memory Initialization. */
extern int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize);

#ifndef __LINUX_USER_SPACE
/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/* RM test Global Resource List (GRL) */
extern const char rmGlobalResourceList[];
/* RM test Global Policy provided to RM Server */
extern const char rmDspOnlyPolicy[];
/* RM instance transport code */
extern int setupRmTransConfig(uint32_t numTestCores, uint32_t systemInitCore, Task_FuncPtr testTask);
#endif

/**********************************************************************
 ************************ SRIO TEST FUNCTIONS *************************
 **********************************************************************/
/**
 *  @b Description
 *  @n  
 *      Yields the task using something like sleep() or usleep().
 *
 *  @retval
 *      Not Applicable
 */
void yield (void)
{
#ifdef __LINUX_USER_SPACE
//    sleep(0);
    sched_yield();
#else
    Task_sleep(1);
#endif
}

/**
 *  @b Description
 *  @n  
 *      Utility to feed a command to rm
 *
 *  @param[in]  name
 *      Name to query
 *  @param[in]  val
 *      return value or value to set
 *  @param[in]  type
 *      transaction type
 *  @param[in]  name
 *      retry until success
 *
 *  @retval
 *      Value
 */
void ex_rm_cmd (const char *name, uint32_t *val, Rm_ServiceType type, int retry)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    int                 succeed;

    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));
    
    rmServiceReq.type             = type;
    rmServiceReq.resourceName     = name;
    rmServiceReq.resourceNsName   = name;
    rmServiceReq.resourceLength   = 1;
    if (val)
    {
        rmServiceReq.resourceBase = *val;
    }

    /* RM will block until resource is returned since callback is NULL */
    rmServiceReq.callback.serviceCallback = NULL;
    do {
        rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);
        succeed = (rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
                  (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC);
        if (retry && (! succeed))
        {
            yield();
        }
    } while (retry && (! succeed));

    if (succeed)
    {
        if ((type == Rm_service_RESOURCE_GET_BY_NAME) && (val))
        {
            *val = rmServiceResp.resourceBase;
        }
    }
    else
    {
        System_printf ("Core %d: failed rm transaction %d %s %d (%d)\n", coreNum, type, name, val ? *val : 0, rmServiceResp.serviceState);
        errorCount++;
    }
}

/**
 *  @b Description
 *  @n  
 *      Utility to look up a name in RM
 *
 *  @param[in]  name
 *      Name to query
 *
 *  @retval
 *      Value
 */
uint32_t ex_rm_name_lookup (const char *name)
{
    uint32_t val;
    ex_rm_cmd (name, &val, Rm_service_RESOURCE_GET_BY_NAME, 1);

    return val;
}

/**
 *  @b Description
 *  @n  
 *      Utility to set a name in RM
 *
 *  @param[in]  name
 *      Name to query
 *
 *  @param[in]  val
 *      Value
 *
 *  @retval
 *      None
 */
void ex_rm_name_set (const char *name, uint32_t val)
{
    ex_rm_cmd (name, &val, Rm_service_RESOURCE_MAP_TO_NAME, 0);
}

/**
 *  @b Description
 *  @n  
 *      Utility to delete a name from RM
 *
 *  @param[in]  name
 *      Name to delete
 *
 *  @retval
 *      None
 */
void ex_rm_name_del (const char *name)
{
    ex_rm_cmd (name, NULL, Rm_service_RESOURCE_UNMAP_NAME, 0);
}

/**
 *  @b Description
 *  @n  
 *      Utility function which converts a local address to global.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Global Address
 */
static uint32_t l2_global_address (uint32_t addr)
{
#ifdef __LINUX_USER_SPACE
    return addr;
#else
    uint32_t corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM); 

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum*0x1000000)));
#endif
}

/**
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for SRIO. 
 *
 *  @retval
 *      Not Applicable.
 */
static int enable_srio (void)
{
#ifdef __LINUX_USER_SPACE
    srioFd = acquireUioGeneric ("srio");
    if (srioFd < 0)
    {
        System_printf ("core %d: Failed to open uio for srio\n", coreNum);
    }
    return srioFd;
#else
#ifndef SIMULATOR_SUPPORT
    /* SRIO power domain is turned OFF by default. It needs to be turned on before doing any 
     * SRIO device register access. This not required for the simulator. */

    /* Set SRIO Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_SRIO);

    /* Enable the clocks too for SRIO */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_SRIO, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_SRIO);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_SRIO));

    /* Return SRIO PSC status */
    if ((CSL_PSC_getPowerDomainState(CSL_PSC_PD_SRIO) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (CSL_PSC_LPSC_SRIO) == PSC_MODSTATE_ENABLE))
    {
        /* SRIO ON. Ready for use */            
        return 0;
    }
    else
    {
        /* SRIO Power on failed. Return error */            
        return -1;            
    }
#else
    /* PSC is not supported on simulator. Return success always */
    return 0;
#endif
#endif
}

/**
 *  @b Description
 *  @n  
 *      System Initialization Code. This is added here only for illustrative
 *      purposes and needs to be invoked once during initialization at 
 *      system startup.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
static int system_init (void)
{
    Qmss_MemRegInfo     memRegInfo;

    /* Memory Region 0 Configuration */
    memRegInfo.descBase         = (uint32_t *)l2_global_address((uint32_t)host_region);
    memRegInfo.descSize         = SIZE_HOST_DESC;
    memRegInfo.descNum          = NUM_HOST_DESC;
    memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memRegInfo.memRegion        = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;    

    /* Initialize and inset the memory region. */
    qmssRegion = Qmss_insertMemoryRegion (&memRegInfo); 
    if (qmssRegion < QMSS_SOK)
    {
        System_printf ("core %d: Error inserting memory region: %d\n", coreNum, qmssRegion);
        return -1;
    }

    /* Share the region */
    ex_rm_name_set ("srio_mt_region", qmssRegion);

    /* CPPI and Queue Manager are initialized. */
    System_printf ("core %d: Debug: Queue Manager and CPPI are initialized.\n", coreNum);
    System_printf ("core %d: Debug: Host Region %p\n", coreNum, host_region);
    return 0;
}

static int system_deinit (void)
{
    Qmss_Result         qmssResult;
    Cppi_Result         cppiResult;

    /* remove the memory region. */
    qmssResult = Qmss_removeMemoryRegion (qmssRegion, 0);
    if (qmssResult < QMSS_SOK)
    {
        System_printf ("core %d: Error removing memory region: %d\n", coreNum, qmssResult);
        return -1;
    }

    /* De Initialize CPPI CPDMA */
    cppiResult = Cppi_exit ();
    if (cppiResult != CPPI_SOK)
    {
        System_printf ("core %d: Error deinitializing CPPI error code : %d\n", coreNum, cppiResult);
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function tests multicore
 *
 *  @param[in]  hSrioDrv
 *      Handle to the SRIO driver 
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int test_multicore (Srio_DrvHandle hSrioDrv)
{
    Srio_SockHandle         srioSocket;
    Srio_SockBindAddrInfo   bindInfo;
    uint8_t*                txData;
    Srio_SockAddrInfo       to;
    Srio_SockAddrInfo       from;
    int                     idx;
    int32_t                 num_bytes;
    uint8_t*                ptr_rxDataPayload;
    int32_t                 sendToCore;
    int32_t                 recvFromCore;
    Srio_DrvBuffer          hDrvBuffer;
    uint32_t                allocatedLen;    
    uint16_t                coreDeviceID[NUM_CORES];
    char                    waitName[32];

    System_printf ("**********************************************\n");
    System_printf ("******** Multicore Testing (Core %d) *********\n", coreNum);
    System_printf ("**********************************************\n");
    
    /* Open SRIO Socket in Blocking Mode */
    srioSocket =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, TRUE);
    if (srioSocket == NULL)
    {
        System_printf ("core %d: Error: Unable to open socket1\n", coreNum);
        return -1;
    }

    /* Initialize the core Device IDs: Each core has a seperate device ID. */
    coreDeviceID[0] = DEVICE_ID1_16BIT;
    coreDeviceID[1] = DEVICE_ID2_16BIT;
#if (NUM_CORES > 2)
    coreDeviceID[2] = DEVICE_ID3_16BIT;
    coreDeviceID[3] = DEVICE_ID4_16BIT;
#endif

    /* Initialize the core bindings; we use the same mailbox & letter identifiers. */
    bindInfo.type11.tt       = TRUE;
    bindInfo.type11.id       = coreDeviceID[coreNum];
    bindInfo.type11.letter   = 2;
    bindInfo.type11.mbox     = 3;
    bindInfo.type11.segMap   = 0x0;

    /* Bind the SRIO socket */
    if (Srio_sockBind (srioSocket, &bindInfo) < 0)
    {
        System_printf ("core %d: Error: socket1 bind failed\n", coreNum);
        return -1;
    }

    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if ((hDrvBuffer == NULL) || (txData == NULL))
    {
        System_printf ("core %d: Error: Producer Memory Allocation failed.\n", coreNum);
        return -1;
    }

    /* Create the transmit data payload. */
    for (idx = 0; idx < 100; idx++)
        txData[idx] = 0xA0 | coreNum;

    /* We can proceed with the data transmission & reception tests only after all the
     * cores have created and bound their SRIO sockets. Using RM name server as barrier.
     */

    /* Mark self ready */
    snprintf(waitName, sizeof(waitName), "srio_mt_core_%d_started", coreNum);
    ex_rm_name_set (waitName, 1);

    /* Wait for everyone else */
    for (idx = 0; idx < NUM_CORES; idx++)
    {
        if (idx != coreNum)
        {
            snprintf(waitName, sizeof(waitName), "srio_mt_core_%d_started", idx);
            System_printf ("core %d: waiting for %s\n", coreNum, waitName);
            ex_rm_name_lookup (waitName);
        }
    }

    System_printf ("------------------------------------------------------\n");


    /* Is this the starting core? */ 
    if (coreNum == CORE_SYS_INIT)
    {
        /* Starting Core: This is a special case because unlike the other cores; this initiates
         * the whole test cycle by sending the first packet. */
        sendToCore   = coreNum + 1;
        if (sendToCore == NUM_CORES)
            sendToCore = 0;
        recvFromCore = coreNum - 1;
        if (recvFromCore < 0)
            recvFromCore = NUM_CORES-1;

        /* Send the data out. */
        to.type11.tt       = TRUE;
        to.type11.id       = coreDeviceID[sendToCore];
        to.type11.letter   = 2;
        to.type11.mbox     = 3;

        /* Send the data out from the producer core to the consumer core. */
        if (Srio_sockSend (srioSocket, hDrvBuffer, 100, &to) < 0)
        {
            System_printf ("core %d: Error: SRIO Socket send failed\n", coreNum);
            return -1;
        }

        /* Debug Message */
        System_printf ("Debug(Core %d): Successfully sent data to ID:0x%x\n", coreNum, to.type11.id);

        /* CORE0: Wait for data to arrive from NUM_CORES-1 */
        System_printf ("Debug(Core %d): Waiting for data to arrive 0x%x\n", coreNum, bindInfo.type11.id);

        /* Receive the data. */
        num_bytes = Srio_sockRecv (srioSocket, (Srio_DrvBuffer*)&ptr_rxDataPayload, &from);
        if (num_bytes > 0)
        {
            /* Make sure that the data was received from the producer core */
            if (from.type11.id != coreDeviceID[recvFromCore])
            {
                System_printf ("core %d: Error: Invalid source id 0x%x Expected 0x%x\n", 
                               coreNum, from.type11.id, coreDeviceID[recvFromCore]);
                return -1;
            }

            /* Received and Transmitted packet length match. Payload verification */
            for (idx = 0; idx < 100; idx++)
            {
                if (ptr_rxDataPayload[idx] != (0xA0 | recvFromCore))
                {
                    System_printf ("core %d: Error: Receive Data Payload verification failed @ index %d: got %x, expected %x\n", 
                                   coreNum, idx, ptr_rxDataPayload[idx], (0xa0 | recvFromCore));
                    errorCount++;
                }
            }
            System_printf ("Debug(Core %d): Successfully received %d bytes\n", coreNum, num_bytes);

            /* Cleanup the received data payload. */
            Srio_freeRxDrvBuffer(srioSocket, (Srio_DrvBuffer)ptr_rxDataPayload);
        }
        else
        {
            /* Error: In receiving data */ 
            System_printf ("core %d: Error: Unable to receive data %d\n", coreNum, num_bytes);
            return -1;
        }
    }
    else
    {
        /* Any other core besides the starting core will initally wait for data to arrive */
        sendToCore   = (coreNum + 1) % NUM_CORES;
        recvFromCore = coreNum - 1;
        if (recvFromCore < 0)
            recvFromCore = NUM_CORES-1;

        System_printf ("Debug(Core %d): Waiting for data to arrive 0x%x\n", coreNum, bindInfo.type11.id);

        /* Receive the data. */
        num_bytes = Srio_sockRecv (srioSocket, (Srio_DrvBuffer*)&ptr_rxDataPayload, &from);
        if (num_bytes > 0)
        {
            /* Make sure that the data was received from the producer core */
            if (from.type11.id != coreDeviceID[recvFromCore])
            {
                System_printf ("core %d: Error: Invalid source id 0x%x Expected 0x%x\n", 
                               coreNum, from.type11.id, coreDeviceID[recvFromCore]);
                return -1;
            }

            /* Received and Transmitted packet length match. Payload verification */
            for (idx = 0; idx < 100; idx++)
            {
                if (ptr_rxDataPayload[idx] != (0xA0 | recvFromCore))
                {
                    System_printf ("core %d: Error: Receive Data Payload verification failed @ index %d: got %x, expected %x\n", 
                                   coreNum, idx, ptr_rxDataPayload[idx], (0xa0 | recvFromCore));
                    errorCount++;
                }
            }
            System_printf ("Debug(Core %d): Successfully received %d bytes\n", coreNum, num_bytes);

            /* Cleanup the received data payload. */
            Srio_freeRxDrvBuffer(srioSocket, (Srio_DrvBuffer)ptr_rxDataPayload);
        }
        else
        {
            /* Error: In receiving data */ 
            System_printf ("core %d: Error: Unable to receive data %d\n", coreNum, num_bytes);
            return -1;
        }

        /* CoreX: Sends outs the packet to CoreX+1. */
        to.type11.tt       = TRUE;
        to.type11.id       = coreDeviceID[sendToCore];
        to.type11.letter   = 2;
        to.type11.mbox     = 3;

        /* Send the data out from the producer core to the consumer core. */
        if (Srio_sockSend (srioSocket, hDrvBuffer, 100, &to) < 0)
        {
            System_printf ("core %d: Error: SRIO Socket send failed\n", coreNum);
            return -1;
        }
        System_printf ("Debug(Core %d): Successfully sent data to ID:0x%x\n", coreNum, coreDeviceID[sendToCore]);
    }

    /* Close the sockets */
    Srio_sockClose (srioSocket);
 
    /* We are done with the test. */
    if (errorCount == 0)
    {
        System_printf ("Debug(Core %d): Multicore Test Passed\n", coreNum);    
    }
    else
    {
        System_printf ("Core %d: Multicore Test ****FAILED**** with %d errors\n", coreNum, errorCount);
    }
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      This is the Multicore Test task
 *
 *  @retval
 *      Not Applicable.
 */
static void multicoreTest (void)
{
    Srio_DrvConfig  cfg;
    Srio_DrvHandle  hSrioDriver;
    Qmss_Result     qmssResult;
    Srio_InitConfig srioInitCfg;
    char            waitName[32];
    int             i;
#ifndef __LINUX_USER_SPACE
    uint8_t         isAllocated;
    Qmss_QueueHnd   accQueHnd;
#endif
    
    if (coreNum == CORE_SYS_INIT)
    {
        System_printf ("Debug(Core %d): System Initialization for CPPI & QMSS\n", coreNum);

        /* System Initialization */
        if (system_init() < 0)
            return;
        
        /* Power on SRIO peripheral before using it */
        if (enable_srio () < 0)
        {
            errorCount++;
            System_printf ("core %d: Error: SRIO PSC Initialization Failed\n", coreNum);
            return;
        }
        
        /* Device Specific SRIO Initializations: This should always be called before
         * initializing the SRIO Driver. */
        if (SrioDevice_init(
#ifdef __LINUX_USER_SPACE
                            fw_srioCfgVaddr, (uint32_t)fw_srioSerdesCfgVaddr
#endif
                           ) < 0)
        {
            errorCount++;
            System_printf ("core %d: Error: SrioDevice_init() Initialization Failed\n", coreNum);
            return;        
        }
    }
#ifdef __LINUX_USER_SPACE
    else
    {
        System_printf ("Debug(Core %d): Waiting for SRIO to be initialized.\n", coreNum);

        /* Find the shared region */
        qmssRegion = ex_rm_name_lookup ("srio_mt_region");
        /* Open it */
        if ((qmssResult = Qmss_openMemoryRegion (qmssRegion, 0)) != QMSS_SOK)
        {
            System_printf ("core %d: Error: Unable to open region %d the QMSS: %d\n", coreNum, qmssRegion, qmssResult);
            errorCount++;
            return;
        }
    }
#endif

#ifndef __LINUX_USER_SPACE
    if (coreNum == CORE_SYS_INIT)
#endif
    {
        /* Initialize the SRIO Driver for this task/address space */
        srioInitCfg.rmServiceHandle = rmClientServiceHandle;
#ifdef __LINUX_USER_SPACE
        srioInitCfg.srioCfgVAddr = fw_srioCfgVaddr;
#endif
        if (Srio_initCfg (&srioInitCfg) < 0)
        {
            errorCount++;
            System_printf ("core %d: Error: SRIO Driver Initialization Failed\n", coreNum);
            return;
        }

#ifndef __LINUX_USER_SPACE
        /* Mark self ready */
        snprintf(waitName, sizeof(waitName), "srio_mt_core_%d_inited", coreNum);
        ex_rm_name_set (waitName, 1);
#endif
    }

    /* SRIO Driver is operational at this time. */
    System_printf ("Debug(Core %d): SRIO can now be used.\n", coreNum);

    /* Initialize the OSAL Data Buffer */
    if (Osal_dataBufferInitMemory(SRIO_MAX_MTU) < 0)
        return;
 
    /* Initialize the SRIO Driver Configuration. */
    memset ((void *)&cfg, 0, sizeof(Srio_DrvConfig));

    /* Setup the SRIO Driver Managed Configuration. */
    cfg.bAppManagedConfig                   = FALSE;

    /* Driver Managed: Receive Configuration */
    cfg.u.drvManagedCfg.bIsRxCfgValid             = 1;
    cfg.u.drvManagedCfg.rxCfg.rxMemRegion         = (Qmss_MemRegion)qmssRegion;
    cfg.u.drvManagedCfg.rxCfg.numRxBuffers        = 4;
    cfg.u.drvManagedCfg.rxCfg.rxMTU               = SRIO_MAX_MTU;

    /* Receive Configuration. */ 
#ifndef __LINUX_USER_SPACE
    /* Accumulator Configuration. */ 
    {
        int32_t coreToQueueSelector[4];

        /* This is the table which maps the core to a specific receive queue. */
        coreToQueueSelector[0] = 704;
        coreToQueueSelector[1] = 705;
        coreToQueueSelector[2] = 706;
        coreToQueueSelector[3] = 707;

        /* Since we are programming the accumulator we want this queue to be a HIGH PRIORITY Queue */
        accQueHnd = cfg.u.drvManagedCfg.rxCfg.rxCompletionQueue = 
            Qmss_queueOpen (Qmss_QueueType_HIGH_PRIORITY_QUEUE, 
                            coreToQueueSelector[coreNum], 
                            &isAllocated);
        if (cfg.u.drvManagedCfg.rxCfg.rxCompletionQueue < 0)
        {
            System_printf ("Error: Unable to open the SRIO Receive Completion Queue\n");
            return;
        }

        /* Accumulator Configuration is VALID. */
        cfg.u.drvManagedCfg.rxCfg.bIsAccumlatorCfgValid = 1;    

        /* Accumulator Configuration. */      
        cfg.u.drvManagedCfg.rxCfg.accCfg.channel             = coreNum;
        cfg.u.drvManagedCfg.rxCfg.accCfg.command             = Qmss_AccCmd_ENABLE_CHANNEL;
        cfg.u.drvManagedCfg.rxCfg.accCfg.queueEnMask         = 0;
        cfg.u.drvManagedCfg.rxCfg.accCfg.queMgrIndex         = coreToQueueSelector[coreNum];
        cfg.u.drvManagedCfg.rxCfg.accCfg.maxPageEntries      = 2;
        cfg.u.drvManagedCfg.rxCfg.accCfg.timerLoadCount      = 0;
        cfg.u.drvManagedCfg.rxCfg.accCfg.interruptPacingMode = Qmss_AccPacingMode_LAST_INTERRUPT;
        cfg.u.drvManagedCfg.rxCfg.accCfg.listEntrySize       = Qmss_AccEntrySize_REG_D;
        cfg.u.drvManagedCfg.rxCfg.accCfg.listCountMode       = Qmss_AccCountMode_ENTRY_COUNT;
        cfg.u.drvManagedCfg.rxCfg.accCfg.multiQueueMode      = Qmss_AccQueueMode_SINGLE_QUEUE;

        /* Initialize the accumulator list memory */
        memset (&gHiPriAccumList[0], 0, sizeof(gHiPriAccumList));
        cfg.u.drvManagedCfg.rxCfg.accCfg.listAddress = l2_global_address((uint32_t)&gHiPriAccumList[0]);
    }
#else
    /* Opens receive queue */
    if ((qpendFd = acquireUioRxQueue ("qpend", &qpendRxQueHnd)) < 0)
    {
        System_printf ("Error Core %d : cant find qpend queue in uio/proc (%d) \n", coreNum, qpendFd);
        errorCount++;
        return;
    }

    /* Since we are programming the accumulator we want this queue to be a HIGH PRIORITY Queue */
    cfg.u.drvManagedCfg.rxCfg.rxCompletionQueue = qpendRxQueHnd;

    /* Accumulator Configuration is not VALID. */
    cfg.u.drvManagedCfg.rxCfg.bIsAccumlatorCfgValid = 0;    
#endif

    /* Driver Managed: Transmit Configuration */
    cfg.u.drvManagedCfg.bIsTxCfgValid             = 1;
    cfg.u.drvManagedCfg.txCfg.txMemRegion         = (Qmss_MemRegion)qmssRegion;
    cfg.u.drvManagedCfg.txCfg.numTxBuffers        = 4;
    cfg.u.drvManagedCfg.txCfg.txMTU               = SRIO_MAX_MTU;

    /* Connect to RM */
    cfg.rmServiceHandle = rmClientServiceHandle;

    /* Start the SRIO Driver */
    hSrioDriver = Srio_start(&cfg);
    if (hSrioDriver == NULL)
    {
        System_printf ("core %d: Error: SRIO Driver Start Failed\n", coreNum);
        errorCount++;
        return;
    }

    /* SRIO Driver is operational at this time. */
    System_printf ("core %d: Debug: SRIO Driver has been started Instance Handle %p\n", coreNum, hSrioDriver);

    /* Hook up the SRIO interrupts with the core. */
#ifdef __LINUX_USER_SPACE
    isr_hSrioDriver = hSrioDriver;
#else
    EventCombiner_dispatchPlug (48, (EventCombiner_FuncPtr)Srio_rxCompletionIsr, (UArg)hSrioDriver, TRUE);
    EventCombiner_enableEvent(48);

    /* Map the event id to hardware interrupt 8. */
    Hwi_eventMap(8, 48);

    /* Enable interrupt 8. */
    Hwi_enableInterrupt(8);
#endif

    /* SRIO Driver is operational at this time. */
    System_printf ("Debug: SRIO Driver ISR has been registered\n");

    /* Run the Multicore Test */
    if (test_multicore (hSrioDriver) < 0)
    {
        System_printf ("core %d: Error: Multicore Tests failed\n", coreNum);
        errorCount++;
        return;
    }

    /* Stop srio */
    if (Srio_stop(hSrioDriver) < 0)
    {
        System_printf ("core %d: Error: SRIO Driver Stop Failed\n", coreNum);
        errorCount++;
        return;
    }

#ifndef __LINUX_USER_SPACE
    /* Close accumulator queue */
    if (Qmss_queueClose (accQueHnd) < QMSS_SOK)
    {
        System_printf ("core %d: Error: accumulator queue close Failed\n", coreNum);
        errorCount++;
        return;
    }
#else
    /* Clean my receive queue */
    if (releaseUioRxQueue (qpendFd, qpendRxQueHnd) < 0)
    {
        System_printf ("Core %d : failed to release qpend fd/hnd\n", coreNum);
        errorCount++;
    }
#endif

    /* De-initialize the system only if the core was configured to do so. */
    if (coreNum == CORE_SYS_INIT)
    {
        /* Wait for other cores to de-init first */
        for (i = 0; i < NUM_CORES; i++)
        {
            if (i != coreNum)
            {
                snprintf(waitName, sizeof(waitName), "srio_mt_core_%d_finished", i);
                System_printf ("Core %d: waiting for %d to finish\n", coreNum, i);
                ex_rm_name_lookup (waitName);
                ex_rm_name_del (waitName);
            }
            /* Delete the started tags, since we are clearly passed that sync point */
            snprintf(waitName, sizeof(waitName), "srio_mt_core_%d_started", i);
            ex_rm_name_del (waitName);
        }
    }

#ifndef __LINUX_USER_SPACE
    if (coreNum == CORE_SYS_INIT)
#endif /* else do on all cores/tasks for linux */
    { 
        if (Srio_close() < 0)
        {
            errorCount++;
            System_printf ("core %d: Error: Srio_close() Failed\n", coreNum);
            return;        
        }
    }

    if (coreNum == CORE_SYS_INIT)
    {
#ifndef __LINUX_USER_SPACE
        /* Delete the init tag */
        snprintf(waitName, sizeof(waitName), "srio_mt_core_%d_inited", coreNum);
        ex_rm_name_del (waitName);
#endif

        /* Delete the rm server names published */
        ex_rm_name_del ("srio_mt_region");

        system_deinit();

        if (SrioDevice_deinit(
#ifdef __LINUX_USER_SPACE
                              fw_srioCfgVaddr
#endif
                             ) < 0)
        {
            errorCount++;
            System_printf ("core %d: Error: SrioDevice_deinit() Failed\n", coreNum);
            return;        
        }
    } 
#ifdef __LINUX_USER_SPACE
    else
    {
        if ((qmssResult = Qmss_closeMemoryRegion (qmssRegion, 0)) != QMSS_SOK)
        {
            errorCount++;
            System_printf ("core %d: Error: Qmss_closeMemoryRegion failed: %d\n", coreNum, qmssResult);
            return;        
        }
    }
#endif

#ifndef __LINUX_USER_SPACE
    if (coreNum == CORE_SYS_INIT)
#endif /* else do on all linux tasks/cores */
    {
        /* Exit qmss */
        System_printf ("Core %d: exit QMSS\n", coreNum);
        if ((qmssResult = Qmss_exit ()))
        {
            errorCount++;
            System_printf ("Error Core %d : exit error code : %d\n", coreNum, qmssResult);
        }
    }

#ifdef LINUX_USER_SPACE
    /* close/power down srio */
    if (releaseUioGeneric (srioFd) < 0)
    {
        System_printf ("Core %d : failed to release srio fd\n", coreNum);
        errorCount++;
    }
#endif
    
    /* Print out the Malloc & Free Counter */
    System_printf ("Debug(Core %d): Allocation Counter : %d\n", coreNum, malloc_counter);
    System_printf ("Debug(Core %d): Free Counter       : %d\n", coreNum, free_counter);

    /* Check if there is a memory leak? */
    if (free_counter != malloc_counter)
    {
        System_printf ("core %d: cfg.u.drvManagedCfg.rxCfg.numRxBuffers = %d\n", coreNum, cfg.u.drvManagedCfg.rxCfg.numRxBuffers);
        System_printf ("core %d: cfg.u.drvManagedCfg.txCfg.numTxBuffers = %d\n", coreNum, cfg.u.drvManagedCfg.txCfg.numTxBuffers);
        System_printf ("core %d: Error: Memory Leak Detected\n", coreNum);
        errorCount++;
        return;
    }

    if (errorCount == 0)
    {
        System_printf ("Debug(Core %d): Multicore Example successful.\n", coreNum);    
    }

    /* announce done */
    if (coreNum != CORE_SYS_INIT)
    {
        snprintf(waitName, sizeof(waitName), "srio_mt_core_%d_finished", coreNum);
        ex_rm_name_set (waitName, 1);
    }
    return;
}

#ifndef __LINUX_USER_SPACE
/**
 *  @b Description
 *  @n  
 *      Task that runs once RM is setup and ready
 *      This starts qmss/cppi to give same pre-condition as on Linux
 *
 *  @retval
 *      Not Applicable.
 */
static Void postRmSetupTask (UArg arg0, UArg arg1)
{
    Qmss_Result         qmssResult;
    Cppi_Result         cppiResult;
    int32_t             rmResult;
    Qmss_StartCfg       qmssStartCfg;
    Cppi_StartCfg       cppiStartCfg;

    rmClientServiceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    if (rmResult != RM_OK)
    {
        errorCount++;
        System_printf ("Error Core %d : Creating RM service handle error code : %d\n", coreNum, rmResult);
        goto exit;
    } 

    /* Initialize the QMSS Configuration block. */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    if (coreNum == CORE_SYS_INIT)
    {
        /* Mark RM ready */
        ex_rm_name_set ("rm_ready", 1);

        /* Set up the linking RAM. Use the internal Linking RAM. 
         * LLD will configure the internal linking RAM address and maximum internal linking RAM size if 
         * a value of zero is specified. Linking RAM1 is not used */
        qmssInitConfig.linkingRAM0Base = 0;
        qmssInitConfig.linkingRAM0Size = 0;
        qmssInitConfig.linkingRAM1Base = 0;
        qmssInitConfig.maxDescNum      = NUM_HOST_DESC;   

#ifdef xdc_target__bigEndian
        qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
        qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
        qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_be);
#else
        qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
        qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
        qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_le);
#endif    

        qmssGblCfgParams.qmRmServiceHandle = rmClientServiceHandle;

        /* Initialize Queue Manager Sub System */
        qmssResult = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
        if (qmssResult != QMSS_SOK)
        {
            errorCount++;
            System_printf ("Error initializing Queue Manager SubSystem error code : %d\n", qmssResult);
            goto exit;
        }
        /* Initialize CPPI CPDMA */
        cppiResult = Cppi_init (&cppiGblCfgParams);
        if (cppiResult != CPPI_SOK)
        {
            errorCount++;
            System_printf ("core %d: Error initializing Queue Manager SubSystem error code : %d\n", coreNum, cppiResult);
            goto exit;
        }
    }
    else
    {
        /* Wait for RM ready */
        ex_rm_name_lookup ("rm_ready");
    }

    /* Start Queue Manager SubSystem with RM */
    qmssStartCfg.rmServiceHandle = rmClientServiceHandle;
    qmssStartCfg.pQmssGblCfgParams = &qmssGblCfgParams;
    if (Qmss_startCfg(&qmssStartCfg) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("core %d: Error: Unable to start the QMSS\n", coreNum);
        goto exit;
    }

    /* Register RM with CPPI */
    cppiStartCfg.rmServiceHandle = rmClientServiceHandle;
    Cppi_startCfg(&cppiStartCfg);

    multicoreTest();

exit:
    if (coreNum == CORE_SYS_INIT)
    {
        ex_rm_name_del ("rm_ready");

        if ((rmResult = Rm_resourceStatus(rmHandle, FALSE)) != 0)
        {
            errorCount++;
            System_printf ("Error Core %d : Number of unfreed resources : %d\n", coreNum, rmResult);
            /* print them */
            Rm_resourceStatus (rmHandle, TRUE);
        }
        else
        {
            System_printf ("Core %d : All resources freed successfully\n", coreNum);
        }
    }

    if (errorCount == 0)
    {
        System_printf ("Core %d : Master core finished successfully\n", coreNum);
    }
    else
    {
        System_printf ("Core %d : failed (errors = %d)\n", errorCount);
    }
    BIOS_exit(errorCount ? 1 : 0);
}
#endif

/**
 *  @b Description
 *  @n  
 *      Entry point for the example
 *
 *  @retval
 *      Not Applicable.
 */
#ifdef __LINUX_USER_SPACE
void usageTsk(Cppi_Handle cppiHnd)
{
    System_printf ("****************************************************\n");
    System_printf ("******  Multicore Loopback Testing (Core %d) *******\n", coreNum);
    System_printf ("****************************************************\n");

    multicoreTest();
}
#else
Void main(Void)
{
    /* RM configuration */
    Rm_InitCfg      rmInitCfg;
    char            rmInstName[RM_NAME_MAX_CHARS];
    int32_t         rmResult;

    /* Get the core number. */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    System_printf ("****************************************************\n");
    System_printf ("******  Multicore Loopback Testing (Core %d) *******\n", coreNum);
    System_printf ("****************************************************\n");

    /* Initialize the heap in shared memory. Using IPC module to do that */ 
    Ipc_start();

    /* Initialize the system only if the core was configured to do so. */
    if (coreNum == CORE_SYS_INIT)
    {
        /* Create the Server instance */
        memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
        System_sprintf (rmInstName, "RM_Server");
        rmInitCfg.instName = rmInstName;
        rmInitCfg.instType = Rm_instType_SERVER;
        rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
        rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspOnlyPolicy;
        rmHandle = Rm_init(&rmInitCfg, &rmResult);
        if (rmResult != RM_OK)
        {
            errorCount++;
            System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", coreNum, rmResult);
            return;
        }
    }
    else
    {
        /* Create a RM Client instance */
        memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
        System_sprintf (rmInstName, "RM_Client%d", coreNum);
        rmInitCfg.instName = rmInstName;
        rmInitCfg.instType = Rm_instType_CLIENT;
        rmHandle = Rm_init(&rmInitCfg, &rmResult);
        if (rmResult != RM_OK)
        {
            errorCount++;
            System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", coreNum, rmResult);
            return;
        }
    }

    if (setupRmTransConfig(NUM_CORES, CORE_SYS_INIT, postRmSetupTask) < 0)
    {
        errorCount++;
        System_printf ("Error core %d : Transport setup for RM error\n", coreNum);
        return;
    }

    /* Start the BIOS */
    System_printf("Core %d : Starting BIOS...\n", coreNum);
    BIOS_start();
}
#endif

