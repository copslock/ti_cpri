/**
 *   @file  test_sock9.c
 *
 *   @brief   
 *      Unit Test case which tests the SRIO driver socket Type9 sockets
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
 ************************** Local Definitions *************************
 **********************************************************************/

/* The size of the Type9 payload which is being transmitted and received
 * in this test. */
#define SRIO_TYPE9_MESSAGE_SIZE         16

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
 ************************* Type9 Test Functions ***********************
 **********************************************************************/

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
Int32 test_sock9 (Srio_DrvHandle hSrioDrv)
{
    Srio_SockHandle         srioSocket1;
    Srio_SockHandle         srioSocket2;
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    UInt8*                  txData;
    UInt16                  idx;
    UInt16                  counter;
    UInt32                  allocatedLen;
    Srio_DrvBuffer          hDrvBuffer;
    Int32                   num_bytes;
    Srio_SockAddrInfo  	    from;
    UInt8*                  ptr_rxDataPayload;

    System_printf ("**********************************************\n");
    System_printf ("****** Socket Type9 Testing (Core %d) ********\n", coreNum);
    System_printf ("**********************************************\n");

    /* Open SRIO Socket */
    srioSocket1 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE9, FALSE);
    if (srioSocket1 == NULL)
    {
        System_printf ("Error: Unable to open socket\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type9.tt       = TRUE;             /* We are using 16 bit identifiers.                 */
    bindInfo.type9.id       = DEVICE_ID1_16BIT; /* Source Identifier to which the socket is bound   */
    bindInfo.type9.cos      = 0;                /* Class of Service                                 */
    bindInfo.type9.streamId = 5;                /* Stream Identifier                                */

    /* Bind the SRIO socket */ 
    if (Srio_sockBind (srioSocket1, &bindInfo) < 0)
    {
        System_printf ("Error: Socket bind failed\n");
        return -1;
    }

    /* Open SRIO Socket */
    srioSocket2 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE9, FALSE);
    if (srioSocket2 == NULL)
    {
        System_printf ("Error: Unable to open socket\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type9.tt       = TRUE;            /* We are using 16 bit identifiers.                 */
    bindInfo.type9.id       = DEVICE_ID2_16BIT;/* Source Identifier to which the socket is bound   */
    bindInfo.type9.cos      = 3;               /* Class of Service                                 */
    bindInfo.type9.streamId = 10;              /* Stream Identifier                                */

    /* Bind the SRIO socket */ 
    if (Srio_sockBind (srioSocket2, &bindInfo) < 0)
    {
        System_printf ("Error: Socket bind failed\n");
        return -1;
    }

    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if (hDrvBuffer == NULL)
    {
        System_printf ("Error: Producer Memory Allocation failed.\n");
        return -1;
    }

    /*************************************************************************************
     * TEST:  
     *  a) Send 10 data packets from socket1 to socket2
     *  b) Make sure data is received on socket2
     *  c) Verify that data was received from the correct source.
     *  d) Validate data payload and length
     *************************************************************************************/
    for (counter = 0; counter < 10; counter++)
    {
        /* Create the transmit data payload. */
        for (idx = 0; idx < SRIO_TYPE9_MESSAGE_SIZE; idx++)
            txData[idx] = 0x90 + counter;

        /* Now we want to send data from the Socket1 to Socket2
         *  So we populate the destination information where the packet is to be sent. */ 
        to.type9.tt       = TRUE;            /* We are using 16 bit identifiers.          */
        to.type9.id       = DEVICE_ID2_16BIT;/* Identifier where the packet is to be sent */
        to.type9.cos      = 3;
        to.type9.streamId = 10;

        /* Send out some data from socket1 to socket2 */
        if (Srio_sockSend (srioSocket1, hDrvBuffer, SRIO_TYPE9_MESSAGE_SIZE, &to) < 0)
        {
            System_printf ("Error: Unable to send Type9 message\n");
            return -1;
        }
        System_printf ("Debug(Core %d): Successfully Sent Type9 BD 0x%p Iteration %d \n", 
                        coreNum, hDrvBuffer, counter);

        /* Wait for the data to arrive */
        while (1)
        {
            /* Wait for the data to arrive. */ 
            num_bytes = Srio_sockRecv (srioSocket2, (Srio_DrvBuffer*)&ptr_rxDataPayload, &from);
            if (num_bytes > 0)
            {
                /* Ensure that the packet was received from the correct source. */
                if (from.type9.id != DEVICE_ID1_16BIT)
                {
                    System_printf ("Error: Data received from invalid source id 0x%x Expected 0x%x\n", 
                                   from.type9.id, DEVICE_ID1_16BIT);
                    return -1;
                }

                /* Validate the data payload. */
                for (idx = 0; idx < SRIO_TYPE9_MESSAGE_SIZE; idx++)
                {
                    if (ptr_rxDataPayload[idx] != txData[idx])
                    {
                        System_printf ("Error: Payload (0x%p) verification failed @ index %d, expected 0x%x got 0x%x\n",
                                        ptr_rxDataPayload, idx, txData[idx], ptr_rxDataPayload[idx]);
                        return -1;
                    }
                }

                /* Data has been received; validate it and ensure this is what we expected. */
                System_printf ("Debug(Core %d): Received %d bytes from ID: 0x%x Iteration %d\n",
                                coreNum, num_bytes, from.type9.id, counter);

                /* Cleanup the received data payload. */
                Srio_freeRxDrvBuffer(srioSocket2, (Srio_DrvBuffer)ptr_rxDataPayload);
                break;
            }
        }
    }

    /* Debug Message: Type9 Test has passed. */
    System_printf ("Debug(Core %d): Type9 Send/Receive Test Passed\n", coreNum);

    /* Close the sockets */
    Srio_sockClose (srioSocket1);
    Srio_sockClose (srioSocket2);

    /* Test Passed */
    return 0;
}

