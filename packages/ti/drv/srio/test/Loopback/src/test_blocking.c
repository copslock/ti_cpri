/**
 *   @file  test_blocking.c
 *
 *   @brief   
 *      Unit Test case which tests the SRIO driver socket blocking 
 *      mode
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2009 Texas Instruments, Inc.
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
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h> 
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>

#include <xdc/cfg/global.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>

#include <ti/csl/csl_chip.h>

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

/* Data Length being transmitted. */
#define SRIO_DATA_LEN       256

/**********************************************************************
 ************************** EXTERN Definitions ************************
 **********************************************************************/

/* Core Number which indicates the core on which the test is executing. */
extern UInt32 coreNum;

extern const uint32_t DEVICE_ID1_16BIT;
extern const uint32_t DEVICE_ID1_8BIT;
extern const uint32_t DEVICE_ID2_16BIT;
extern const uint32_t DEVICE_ID2_8BIT;
extern const uint32_t DEVICE_ID3_16BIT;
extern const uint32_t DEVICE_ID3_8BIT_ID;
extern const uint32_t DEVICE_ID4_16BIT;
extern const uint32_t DEVICE_ID4_8BIT_ID;

/**********************************************************************
 ************************** Global Definitions *************************
 **********************************************************************/

/* Global TEST Status: Value of 0 indicates SUCCESS else FAILURE */
Int32   producerTestStatus = -1;
Int32   consumerTestStatus = -1;

/**********************************************************************
 ************************ Blocking API Functions **********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      This is the Producer Task which will send data using SRIO to the
 *      consumer tasks.
 *
 *  @param[in]  hSrioDriver
 *      Handle to the SRIO driver 
 *
 *  @retval
 *      Not Applicable.
 */
static Void producerTask(UArg arg0, UArg arg1)
{
    Srio_SockHandle         srioSocket;
    Srio_SockBindAddrInfo   bindInfo;
    UInt16                  idx;
    Srio_SockAddrInfo       to;
    UInt8*                  txData;
    Semaphore_Handle        producerSemHandle;
    Srio_DrvHandle          hSrioDrv;
    Srio_DrvBuffer          hDrvBuffer;
    UInt32                  allocatedLen;
    
    System_printf ("Debug(Core %d): Producer Task started.\n", coreNum);

    /* Get the Producer Semaphore Handle. */
    producerSemHandle = (Semaphore_Handle)arg0;

    /* Get the SRIO Driver Handle. */
    hSrioDrv = (Srio_DrvHandle)arg1;

    /* Open SRIO Socket */
    srioSocket = Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, TRUE);
    if (srioSocket == NULL)
    {
        System_printf ("Error: Unable to open socket\n");
        return;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;             /* We are using 16 bit identifiers.                 */
    bindInfo.type11.id       = DEVICE_ID1_16BIT; /* Source Identifier to which the socket is bound   */
    bindInfo.type11.letter   = 0;                /* Letter Identifier                                */
    bindInfo.type11.mbox     = coreNum;          /* Mailbox Number                                   */
    bindInfo.type11.segMap   = 0x0;              /* Single Segment                                   */

    /* Bind the SRIO socket */ 
    if (Srio_sockBind (srioSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Socket bind failed\n");
        return;
    }
    
    System_printf ("Debug(Core %d): Producer sockets have been created and bound.\n", coreNum);

    /*************************************************************************************
     * TEST 1: 
     *  a) Send SRIO_DATA_LEN bytes of data from producer to consumer.
     *  b) Consumer validates if received data was correct.
     *************************************************************************************/

    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if (hDrvBuffer == NULL)
    {
        System_printf ("Error: Producer Memory Allocation failed.\n");
        return;
    }
    
    /* Create the transmit data payload. */
    for (idx = 0; idx < SRIO_DATA_LEN; idx++)
        txData[idx] = idx;

    /* Now we want to send data from the Socket1 to Socket2
     *  So we populate the destination information where the packet is to be sent. */
    to.type11.tt       = TRUE;             /* We are using 16 bit identifiers.            */
    to.type11.id       = DEVICE_ID2_16BIT; /* Identifier where the packet is to be sent   */
    to.type11.letter   = 1;                /* Letter Identifier                           */
    to.type11.mbox     = coreNum;          /* Mailbox Number                              */

    /* Send out some data from socket1 to socket2 */
    if (Srio_sockSend (srioSocket, hDrvBuffer, SRIO_DATA_LEN, &to) < 0)
    {
        System_printf ("Error: Producer was unable to send data. \n");
        return;
    }
    System_printf ("Debug(Core %d): Producer TEST 1 Passed\n", coreNum);

    /* Close the socket */
    Srio_sockClose(srioSocket);

    /* Test has passed and there were no errors. */
    producerTestStatus = 0;

    /* Inform the main unit test task that the producer is done. */
    Semaphore_post (producerSemHandle);
   
    /* Exit the Task. */ 
    Task_exit();
}

/**
 *  @b Description
 *  @n  
 *      This is the consumer task which will receiving data from the producer.
 *
 *  @retval
 *      Not Applicable.
 */
static Void consumerTask(UArg arg0, UArg arg1)
{
    Srio_SockHandle         srioSocket;
    Srio_SockBindAddrInfo   bindInfo;
    UInt8*                  ptr_rxDataPayload;
    Int32                   num_bytes;
    Srio_SockAddrInfo       from;
    UInt16                  idx;
    Semaphore_Handle        consumerSemHandle;
    Srio_DrvHandle          hSrioDrv;

    System_printf ("Debug(Core %d): Consumer Task started.\n", coreNum);

    /* Get the Consumer Semaphore Handle. */
    consumerSemHandle = (Semaphore_Handle)arg0;

    /* Get the SRIO Driver Handle. */
    hSrioDrv = (Srio_DrvHandle)arg1;

    /* Open SRIO Socket */
    srioSocket =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, TRUE);
    if (srioSocket == NULL)
    {
        System_printf ("Error: Unable to open socket\n");
        return;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;            /* We are using 16 bit identifiers.                 */
    bindInfo.type11.id       = DEVICE_ID2_16BIT;/* Source Identifier to which the socket is bound   */
    bindInfo.type11.letter   = 1;               /* Letter Identifier                                */
    bindInfo.type11.mbox     = coreNum;         /* Mailbox Number                                   */
    bindInfo.type11.segMap   = 0x0;             /* Single Segment                                   */

    /* Bind the SRIO socket */ 
    if (Srio_sockBind (srioSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Socket bind failed\n");
        return;
    }

    System_printf ("Debug(Core %d): Consumer is waiting for data...\n", coreNum);

    /*************************************************************************************
     * TEST 1: 
     *  a) Producer sends SRIO_DATA_LEN bytes of data
     *  a) Make sure data is received on consumer.
     *  b) Verify that data was received from the correct source.
     *  c) Validate data payload and length
     *************************************************************************************/

    /* Wait for the data to arrive. */ 
    num_bytes = Srio_sockRecv (srioSocket, (Srio_DrvBuffer*)&ptr_rxDataPayload, &from);
    if (num_bytes > 0)
    {
        /* Valid data was received. Check if the length received matches the length transmitted. */
        if (num_bytes != SRIO_DATA_LEN) 
        {
            /* Error: Data Payload Length mismatch... */
            System_printf ("Error: Invalid data payload received\n");
            return;
        }

        /* Make sure that the data was received from socket1 */
        if (from.type11.id != DEVICE_ID1_16BIT)
        {
            System_printf ("Error: Data received from invalid source id 0x%x expected 0x%x\n", 
                            from.type11.id, DEVICE_ID1_16BIT);
            return;
        }
 
        /* Received and Transmitted packet length match. Payload verification */
        for (idx = 0; idx < SRIO_DATA_LEN; idx++)
        {
            if (ptr_rxDataPayload[idx] != idx)
            {
                System_printf ("Error: Receive Data Payload (0x%p) verification failed @ index %d Expected %d Got %d\n", 
                                ptr_rxDataPayload, idx, idx, ptr_rxDataPayload[idx]);
                return;
            }
        }

        /* Debug Message: */
        System_printf ("Debug(Core %d): Cleaning up receive buffer 0x%p\n", coreNum, ptr_rxDataPayload);

        /* Cleanup the received data payload. */
        Srio_freeRxDrvBuffer(srioSocket, (Srio_DrvBuffer)ptr_rxDataPayload);
    }
    else
    {
        /* This implies that there was either no data or an error. This should not be the case */
        System_printf ("Error: Unable to receive data %d\n", num_bytes);
        return;
    }
    System_printf ("Debug(Core %d): Consumer TEST 1 Passed\n", coreNum);

    /* Close the socket */
    Srio_sockClose(srioSocket);

    /* Test has passed and there were no errors. */
    consumerTestStatus = 0;
    
    /* Inform the main unit test task that the consumer is done. */
    Semaphore_post (consumerSemHandle);

    /* Exit the Task. */
    Task_exit();
}

/**
 *  @b Description
 *  @n  
 *      Entry point for the test case
 *
 *  @param[in]  hSrioDrv
 *      Handle to the SRIO driver 
 *
 *  @retval
 *      Not Applicable.
 */
Int32 test_blocking (Srio_DrvHandle hSrioDrv)
{
    Task_Params         taskParams;
    Semaphore_Handle    producerSemHandle;
    Semaphore_Handle    consumerSemHandle;

    System_printf ("**********************************************\n");
    System_printf ("******** Blocking Testing (Core %d) **********\n", coreNum);
    System_printf ("**********************************************\n");

    /* Create the Producer and Consumer Semaphores */
    producerSemHandle = Semaphore_create (0, NULL, NULL);
    consumerSemHandle = Semaphore_create (0, NULL, NULL);

    /* Create the Consumer Task */
    Task_Params_init(&taskParams);
    taskParams.arg0 = (UArg)consumerSemHandle;
    taskParams.arg1 = (UArg)hSrioDrv;
    taskParams.stackSize = 4096;
    Task_create(consumerTask, &taskParams, NULL);

    /* Create the Producer Task */
    Task_Params_init(&taskParams);
    taskParams.arg0 = (UArg)producerSemHandle;
    taskParams.arg1 = (UArg)hSrioDrv;
    taskParams.stackSize = 4096;
    Task_create(producerTask, &taskParams, NULL);

    /* Wait for the producer and consumer tasks to end. */
    Semaphore_pend (producerSemHandle, BIOS_WAIT_FOREVER);
    Semaphore_pend (consumerSemHandle, BIOS_WAIT_FOREVER);

    /* Were there any errors? */
    if ((consumerTestStatus == 0) && (producerTestStatus == 0))
        return 0;

    /* Producer or Consumer Failed; report this to the Unit Test case. */
    return -1;
}

