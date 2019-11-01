/**
 *   @file  test_multicore.c
 *
 *   @brief   
 *      The file contains the multicore test suite for the SRIO Driver and
 *      is used for testing the SRIO driver library to work in a multicore
 *      environment. The test suite sends/receives data as follows:-
 *
 *      CORE 0 --> CORE 1 --> CORE 2 --> CORE 3 --> CORE 0
 *
 *      At each step the received data is validated (Payload and Received
 *      Identifiers).
 *
 *      The goal of the test suite is to showcase that the SRIO Driver API
 *      are multi-core safe.
 *
 *      The test case can only be built by ensuring that the definition
 *      TEST_MULTICORE is defined. 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009-2012 Texas Instruments, Inc.
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
#ifdef TEST_MULTICORE

#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h> 
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>

#include <xdc/cfg/global.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>

/* CSL Include Files. */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_semAux.h>

/* Device specific include */
#include "srioPlatCfg.h"

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

/* Number of cores for which the test is being executed. */
#define NUM_CORES                   srio_EXAMPLE_NUM_CORES

/* SRIO Application Hardware Semaphore. */
#define SRIO_APP_HW_SEM             6

/**********************************************************************
 ************************** EXTERN Definitions ************************
 **********************************************************************/

extern UInt32          coreNum;

extern uint32_t DEVICE_ID1_16BIT;
extern uint32_t DEVICE_ID1_8BIT;
extern uint32_t DEVICE_ID2_16BIT;
extern uint32_t DEVICE_ID2_8BIT;
#if (NUM_CORES > 2)
extern uint32_t DEVICE_ID3_16BIT;
extern uint32_t DEVICE_ID3_8BIT_ID;
extern uint32_t DEVICE_ID4_16BIT;
extern uint32_t DEVICE_ID4_8BIT_ID;
#endif

/* Created an array to pad the cache line with SRIO_MAX_CACHE_ALIGN size */
#pragma DATA_ALIGN   (isSRIOSocketsCreated, 128)
#pragma DATA_SECTION (isSRIOSocketsCreated, ".srioSharedMem");
volatile Uint32     isSRIOSocketsCreated[(SRIO_MAX_CACHE_ALIGN / sizeof(Uint32))] = { 0 };

/**********************************************************************
 ********************** NON Blocking API Functions ********************
 **********************************************************************/

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
Int32 test_multicore (Srio_DrvHandle hSrioDrv)
{
    Srio_SockHandle         srioSocket;
    Srio_SockBindAddrInfo   bindInfo;
    UInt8*                  txData;
    Srio_SockAddrInfo       to;
    Srio_SockAddrInfo       from;
    UInt16                  idx;
    Int32                   num_bytes;
    UInt8*                  ptr_rxDataPayload;
    Int32                   sendToCore;
    Int32                   recvFromCore;
    Srio_DrvBuffer          hDrvBuffer;
    UInt32                  allocatedLen;    
    UInt16                  coreDeviceID[NUM_CORES];

    System_printf ("**********************************************\n");
    System_printf ("******** Multicore Testing (Core %d) *********\n", coreNum);
    System_printf ("**********************************************\n");
    
    /* Open SRIO Socket in Blocking Mode */
    srioSocket =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, TRUE);
    if (srioSocket == NULL)
    {
        System_printf ("Error: Unable to open socket1\n");
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
        System_printf ("Error: socket1 bind failed\n");
        return -1;
    }

    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if (hDrvBuffer == NULL)
    {
        System_printf ("Error: Producer Memory Allocation failed.\n");
        return -1;
    }

    /* Create the transmit data payload. */
    for (idx = 0; idx < 100; idx++)
        txData[idx] = 0xA0 | coreNum;

    /* The global variable is a shared resource which is being accessed from multiple cores. 
     * So here we need to protect it and ensure that there is only 1 core which is accessing 
     * it at a time. We use a Hardware Semaphore to protect this. */
    while ((CSL_semAcquireDirect (SRIO_APP_HW_SEM)) == 0);    

    /* Invalidate the cache and make sure you get the latest from the memory. */
    CACHE_invL1d ((void *) &isSRIOSocketsCreated[0], 128, CACHE_WAIT);

    /* The core has created the sockets:*/
    isSRIOSocketsCreated[0]++;

    /* The SRIO Socket has been created. Writeback the contents to the cache. */
    CACHE_wbL1d ((void *) &isSRIOSocketsCreated[0], 128, CACHE_WAIT);

    /* Release the hardware semaphore. */
    CSL_semReleaseSemaphore (SRIO_APP_HW_SEM);

    /* We can proceed with the data transmission & reception tests only after all the
     * cores have created and bound their SRIO sockets. This is a simple counter running
     * in shared memory which allows us to SYNC up the socket creation. Wait till all the
     * cores have created the sockets */
    while (isSRIOSocketsCreated[0] != NUM_CORES)
        CACHE_invL1d ((void *) &isSRIOSocketsCreated[0], 128, CACHE_WAIT);

    System_printf ("------------------------------------------------------\n");

    /* Is this the starting core? */ 
    if (coreNum == 1)
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
            System_printf ("Error: SRIO Socket send failed\n");
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
                System_printf ("Error: Invalid source id 0x%x Expected 0x%x\n", from.type11.id, coreDeviceID[recvFromCore]);
                return -1;
            }

            /* Received and Transmitted packet length match. Payload verification */
            for (idx = 0; idx < 100; idx++)
            {
                if (ptr_rxDataPayload[idx] != (0xA0 | recvFromCore))
                {
                    System_printf ("Error: Receive Data Payload verification failed @ index %d\n", idx);
                    return -1;
                }
            }
            System_printf ("Debug(Core %d): Successfully received %d bytes\n", coreNum, num_bytes);

            /* Cleanup the received data payload. */
            Srio_freeRxDrvBuffer(srioSocket, (Srio_DrvBuffer)ptr_rxDataPayload);
        }
        else
        {
            /* Error: In receiving data */ 
            System_printf ("Error: Unable to receive data %d\n", num_bytes);
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
                System_printf ("Error: Invalid source id 0x%x Expected 0x%x\n", from.type11.id, coreDeviceID[recvFromCore]);
                return -1;
            }

            /* Received and Transmitted packet length match. Payload verification */
            for (idx = 0; idx < 100; idx++)
            {
                if (ptr_rxDataPayload[idx] != (0xA0 | recvFromCore))
                {
                    System_printf ("Error: Receive Data Payload verification failed @ index %d\n", idx);
                    return -1;
                }
            }
            System_printf ("Debug(Core %d): Successfully received %d bytes\n", coreNum, num_bytes);

            /* Cleanup the received data payload. */
            Srio_freeRxDrvBuffer(srioSocket, (Srio_DrvBuffer)ptr_rxDataPayload);
        }
        else
        {
            /* Error: In receiving data */ 
            System_printf ("Error: Unable to receive data %d\n", num_bytes);
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
            System_printf ("Error: SRIO Socket send failed\n");
            return -1;
        }
        System_printf ("Debug(Core %d): Successfully sent data to ID:0x%x\n", coreNum, coreDeviceID[sendToCore]);
    }

    /* Close the sockets */
    Srio_sockClose (srioSocket);
 
    /* We are done with the test. */
    System_printf ("Debug(Core %d): Multicore Test Passed\n", coreNum);    
    return 0;
}

#endif /* TEST_MULTICORE */


