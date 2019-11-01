/**
 *   @file  test_nonblocking.c
 *
 *   @brief   
 *      This is the TEST code for the SRIO Driver. The test code uses 
 *      XDC/BIOS and showcases usage of the SRIO Driver exported API 
 *      for sending/receiving data. 
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
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>

#include <xdc/cfg/global.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>

/* CSL Chip Functional Layer */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_chipAux.h>

/* OSAL Include Files. */
#include <srio_osal.h>

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

/* Data Length being transmitted. */
#define SRIO_DATA_LEN       128

/**********************************************************************
 ************************** EXTERN Definitions ************************
 **********************************************************************/

extern const uint32_t DEVICE_ID1_16BIT;
extern const uint32_t DEVICE_ID1_8BIT;
extern const uint32_t DEVICE_ID2_16BIT;
extern const uint32_t DEVICE_ID2_8BIT;
extern const uint32_t DEVICE_ID3_16BIT;
extern const uint32_t DEVICE_ID3_8BIT_ID;
extern const uint32_t DEVICE_ID4_16BIT;
extern const uint32_t DEVICE_ID4_8BIT_ID;

/* Core Number on which the test is executing. */
extern UInt32 coreNum;

/**********************************************************************
 ********************** NON Blocking API Functions ********************
 **********************************************************************/

/**
 *  @b Description
 *  @n  
 *      The function tests NON Blocking sockets.
 *
 *  @param[in]  hSrioDrv
 *      Handle to the SRIO driver 
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
Int32 test_nonblocking(Srio_DrvHandle hSrioDrv)
{
    Srio_SockHandle         srioSocket1;
    Srio_SockHandle         srioSocket2;
    Srio_SockBindAddrInfo   bindInfo;
    UInt8*                  txData;
    Int32                   num_bytes;
    Srio_SockAddrInfo       to;
    Srio_SockAddrInfo       from;
    UInt16                  idx;
    UInt8*                  ptr_rxDataPayload;
    UInt32                  allocatedLen;
    Srio_DrvBuffer          hDrvBuffer;
    UInt16                  counter;

    System_printf ("**********************************************\n");
    System_printf ("****** Non-Blocking Testing (Core %d) ********\n", coreNum);
    System_printf ("**********************************************\n");

    /* Open SRIO Socket 1 */
    srioSocket1 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, FALSE);
    if (srioSocket1 == NULL)
    {
        System_printf ("Error: Unable to open socket1\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;                 /* We are using 16 bit identifiers.                 */
    bindInfo.type11.id       = DEVICE_ID1_16BIT;     /* Source Identifier to which the socket is bound   */
    bindInfo.type11.letter   = 2;                    /* Letter Identifier                                */
    bindInfo.type11.mbox     = coreNum;              /* Mailbox Number                                   */
    bindInfo.type11.segMap   = 0x0;                  /* Single Segment                                   */

    /* Bind the SRIO socket 1 */ 
    if (Srio_sockBind (srioSocket1, &bindInfo) < 0)
    {
        System_printf ("Error: socket1 bind failed\n");
        return -1;
    }

    /* Open the SRIO Socket2 for Type11 */
    srioSocket2 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, FALSE);
    if (srioSocket2 == NULL)
    {
        System_printf ("Error: Unable to open socket2\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;             /* We are using 16 bit identifiers.                 */
    bindInfo.type11.id       = DEVICE_ID2_16BIT; /* Source Identifier to which the socket is bound   */
    bindInfo.type11.letter   = 3;                /* Letter Identifier                                */
    bindInfo.type11.mbox     = coreNum;          /* Mailbox Number                                   */
    bindInfo.type11.segMap   = 0x0;              /* Single Segment                                   */

    /* Bind the SRIO Socket 2 */ 
    if (Srio_sockBind (srioSocket2, &bindInfo) < 0)
    {
        System_printf ("Error: socket2 bind failed\n");
        return -1;
    }
	
    /*************************************************************************************
     * TEST 1: 
     *  a) Send Data from socket1 to socket2
     *  b) Make sure data is received on socket2
     *  c) Verify that data was received from the correct source.
     *  d) Validate data payload and length
     *************************************************************************************/

    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if (hDrvBuffer == NULL)
        return -1;

    /* Create the transmit data payload. */
    for (idx = 0; idx < SRIO_DATA_LEN; idx++)
        txData[idx] = idx;

    /* Now we want to send data from the Socket1 to Socket2
     *  So we populate the destination information where the packet is to be sent. */ 
    to.type11.tt       = TRUE;             /* We are using 16 bit identifiers.          */
    to.type11.id       = DEVICE_ID2_16BIT; /* Identifier where the packet is to be sent */
    to.type11.letter   = 3;                /* Letter Identifier                         */
    to.type11.mbox     = coreNum;          /* Mailbox Number                            */

    /* Send out some data from socket1 to socket2 */
    if (Srio_sockSend (srioSocket1, hDrvBuffer, SRIO_DATA_LEN, &to) < 0)
    {
        System_printf ("Error: SRIO Socket send failed\n");
        return -1;
    }

    /* Wait for the data to arrive. */
    while (1)
    {
        /* Receive data */
        num_bytes = Srio_sockRecv (srioSocket2, (Srio_DrvBuffer*)&ptr_rxDataPayload, &from);
        if (num_bytes > 0)
        {
            /* Valid data was received. Check if the length received matches the length transmitted. */
            if (num_bytes != SRIO_DATA_LEN) 
            {
                /* Error: Data Payload Length mismatch... */
                System_printf ("Error: Invalid data payload received\n");
                return -1;
            }

            /* Make sure that the data was received from socket1 */
            if (from.type11.id != DEVICE_ID1_16BIT)
            {
                System_printf ("Error: Data received from invalid source id 0x%x Expected 0x%x\n", 
                               from.type11.id, DEVICE_ID1_16BIT);
                return -1;
            }

            /* Received and Transmitted packet length match. Payload verification */
            for (idx = 0; idx < SRIO_DATA_LEN; idx++)
            {
                if (ptr_rxDataPayload[idx] != idx)
                {
                    System_printf ("Error: Receive Data Payload (0x%p) verification failed @ index %d\n", 
                                    ptr_rxDataPayload,idx);
                    return -1;
                }
            }

            /* Cleanup the received data payload. */
            Srio_freeRxDrvBuffer(srioSocket2, (Srio_DrvBuffer)ptr_rxDataPayload);
            break;
        }
    }

    /* Control comes here implies that data was received and validated */
    System_printf ("Debug(Core %d): Test 1 Passed\n", coreNum);

    /*************************************************************************************
     * TEST 2: 
     *  a) Send Data from socket2 to socket1
     *  b) Make sure data is received on socket1
     *  c) Verify that data was received from the correct source.
     *  d) Validate data payload and length
     *************************************************************************************/

    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if (hDrvBuffer == NULL)
    {
        System_printf ("Error: Transmit Buffer Allocation Failed\n");
        return -1;
    }
    
    /* Create the transmit data payload. */
    for (idx = 0; idx < SRIO_DATA_LEN; idx++)
        txData[idx] = 0x12;

    /* Now we want to send data from the Socket2 to Socket1
     *  So we populate the destination information where the packet is to be sent. */ 
    to.type11.tt       = TRUE;                 /* We are using 16 bit identifiers.          */
    to.type11.id       = DEVICE_ID1_16BIT;     /* Identifier where the packet is to be sent */
    to.type11.letter   = 2;                    /* Letter Identifier                         */
    to.type11.mbox     = coreNum;              /* Mailbox Number                            */

    /* Send out some data from socket1 to socket2 */
    if (Srio_sockSend (srioSocket2, hDrvBuffer, SRIO_DATA_LEN, &to) < 0)
    {
        System_printf ("Error: SRIO Socket send failed\n");
        return -1;
    }

    /* Wait for the data to arrive. */
    while (1)
    {
        /* Receive data */
        num_bytes = Srio_sockRecv (srioSocket1, (Srio_DrvBuffer*)&ptr_rxDataPayload, &from);
        if (num_bytes > 0)
        {
            /* Valid data was received. Check if the length received matches the length transmitted. */
            if (num_bytes != SRIO_DATA_LEN) 
            {
                /* Error: Data Payload Length mismatch... */
                System_printf ("Error: Invalid data payload received\n");
                return -1;
            }

            /* Make sure that the data was received from socket1 */
            if (from.type11.id != DEVICE_ID2_16BIT)
            {
                System_printf ("Error: Data received from invalid source id 0x%x\n", from.type11.id);
                return -1;
            }

            /* Received and Transmitted packet length match. Payload verification */
            for (idx = 0; idx < SRIO_DATA_LEN; idx++)
            {
                if (ptr_rxDataPayload[idx] != 0x12)
                {
                    System_printf ("Error: Receive Data Payload (0x%p) verification failed @ index %d\n", 
                                    ptr_rxDataPayload,idx);
                    return -1;
                }
            }

            /* Cleanup the received data payload. */
            Srio_freeRxDrvBuffer(srioSocket1, (Srio_DrvBuffer)ptr_rxDataPayload);
            break;
        }
    }

    /* Control comes here implies that data was received and validated. */
    System_printf ("Debug(Core %d): Test 2 Passed\n", coreNum);

    /*************************************************************************************
     * TEST 3: 
     *  a) Send Data Packet 1 from socket 1 to socket2
     *  b) Send Data Packet 2 from socket 1 to socket2
     *  c) Make sure that there are 2 packets received on socket2
     *  d) Validate data payload and length of both data packets.
     *  e) Verify that data was received from the correct source.
     *************************************************************************************/

    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if (hDrvBuffer == NULL)
        return -1;
    
    /* Create the transmit data payload 1 */
    for (idx = 0; idx < SRIO_DATA_LEN; idx++)
        txData[idx] = 0xAA;

    /* Now we want to send data from the Socket1 to Socket2
     *  So we populate the destination information where the packet is to be sent. */ 
    to.type11.tt       = TRUE;             /* We are using 16 bit identifiers.          */
    to.type11.id       = DEVICE_ID2_16BIT; /* Identifier where the packet is to be sent */
    to.type11.letter   = 3;                /* Letter Identifier                         */
    to.type11.mbox     = coreNum;          /* Mailbox Number                            */

    /* Send out data packet 1 from socket1 to socket2 */
    if (Srio_sockSend (srioSocket1, hDrvBuffer, SRIO_DATA_LEN, &to) < 0)
    {
        System_printf ("Error: SRIO Socket send failed\n");
        return -1;
    }

    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if (hDrvBuffer == NULL)
        return -1;

    /* Create the transmit data payload 2 */
    for (idx = 0; idx < SRIO_DATA_LEN; idx++)
        txData[idx] = 0xBB;

    /* Send out data packet 2 from socket1 to socket2 */
    if (Srio_sockSend (srioSocket1, hDrvBuffer, SRIO_DATA_LEN, &to) < 0)
    {
        System_printf ("Error: SRIO Socket send failed\n");
        return -1;
    }

    /* Initialize the counter field. */
    counter = 0;

    /* There are 2 packets which were sent thus there should be 2 packets received. */
    memset ((void *)&from, 0, sizeof(Srio_SockAddrInfo));

    /* Wait for the data to arrive. */
    while (counter < 2)
    {
        /* Receive data */
        num_bytes = Srio_sockRecv (srioSocket2, (Srio_DrvBuffer*)&ptr_rxDataPayload, &from);
        if (num_bytes > 0)
        {
            /* Valid data was received. Check if the length received matches the length transmitted. */
            if (num_bytes != SRIO_DATA_LEN) 
            {
                /* Error: Data Payload Length mismatch... */
                System_printf ("Error: Invalid data payload received\n");
                return -1;
            }

            /* Make sure that the data was received from socket1 */
            if (from.type11.id != DEVICE_ID1_16BIT)
            {
                System_printf ("Error: Data received from invalid source id 0x%x Expected 0x%x\n", 
                                from.type11.id, DEVICE_ID1_16BIT);
                return -1;
            }            

            /* Received and Transmitted packet length match. Payload verification */
            for (idx = 0; idx < SRIO_DATA_LEN; idx++)
            {
                UInt8  data;

                /* We need to verify the data payload. */
                data = (counter == 0) ? 0xAA : 0xBB;

                if (ptr_rxDataPayload[idx] != data)
                {
                    System_printf ("Error: Receive Data Payload (0x%p) verification failed @ index %d\n", 
                                    ptr_rxDataPayload,idx);
                    return -1;
                }
            }

            /* Cleanup the received data payload. */
            Srio_freeRxDrvBuffer(srioSocket2, (Srio_DrvBuffer)ptr_rxDataPayload);

            /* Increment the counter */
            counter++;
        }
    }
    System_printf ("Debug(Core %d): Test 3 Passed\n", coreNum);

    /*************************************************************************************
     * TEST 4: 
     *  a) Set the Socket option for socket 2 to reduce the MAX limit to 1.
     *  b) Use the GET Socket option for socket2 to get the current MAX limit i.e. 1
     *************************************************************************************/

    /* Now we limit the max number of pending packets to 1 */
    counter = 1;
    if (Srio_setSockOpt (srioSocket2, Srio_Opt_PENDING_PKT_COUNT, (void *)&counter, sizeof(counter)) < 0)
    {
        System_printf ("Error: SRIO Socket Set options failed\n");
        return -1;
    }

    /* Initialize the counter. */
    counter = 0;

    /* Get the current socket option for the max limit of pending packets. */
    if (Srio_getSockOpt (srioSocket2, Srio_Opt_PENDING_PKT_COUNT, (void *)&counter, sizeof(counter)) < 0)
    {
        System_printf ("Error: SRIO Socket Get options failed\n");
        return -1;
    }

    /* Ensure that the value retrieved matches what we configured  */
    if (counter != 1)
    {
        System_printf ("Error: Max Limit %d retrieved does not match configured max limit 1\n", counter);
        return -1;
    }
    System_printf ("Debug(Core %d): Test 4 Passed\n", coreNum);

    /* Close the sockets. */
    Srio_sockClose (srioSocket1);
    Srio_sockClose (srioSocket2);

    /*************************************************************************************
     * TEST 5: 
     *  a) Validate the device id bindings.
     *  b) Binding is only allowed for valid identifiers which are configured in the SRIO IP
     *  c) The test tries to bind an invalid 16 bit id and this should fail.
     *  d) The test tries to bind an invalid 8 bit id and this should fail.
     *  e) The test tries to bind to a valid 8 bit id (configured in the Device ID CSR) and
     *     this should pass.
     *  f) The test tries to bind to a valid 8 bit id (configured in the BRR) and this 
     *     should pass.
     *************************************************************************************/

    /* Open SRIO Socket 1 */
    srioSocket1 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, FALSE);
    if (srioSocket1 == NULL)
    {
        System_printf ("Error: Unable to open socket1\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;   /* We are using 16 bit identifiers.                 */
    bindInfo.type11.id       = 0x0001; /* Source Identifier to which the socket is bound   */
    bindInfo.type11.letter   = 2;      /* Letter Identifier                                */
    bindInfo.type11.mbox     = 1;      /* Mailbox Number                                   */
    bindInfo.type11.segMap   = 0x0;    /* Single Segment                                   */

    /* Bind the SRIO socket: This should fail since we have not configured the id in the SRIO IP. */ 
    if (Srio_sockBind (srioSocket1, &bindInfo) == 0)
    {
        System_printf ("Error(Core %d): Socket Bind test failed for invalid 16bit id\n", coreNum);
        return -1;
    }
    System_printf ("Debug(Core %d): Socket Bind test passed for invalid 16bit id\n", coreNum);

    /* Let us now try and invalid 8 bit binding. */
    bindInfo.type11.tt       = FALSE;  /* We are using 8 bit identifiers.                 */
    bindInfo.type11.id       = 0x11;   /* Source Identifier to which the socket is bound   */
    bindInfo.type11.letter   = 2;      /* Letter Identifier                                */
    bindInfo.type11.mbox     = 1;      /* Mailbox Number                                   */
    bindInfo.type11.segMap   = 0x0;    /* Single Segment                                   */

    /* Bind the SRIO socket: This should fail since we have not configured the id in the SRIO IP. */ 
    if (Srio_sockBind (srioSocket1, &bindInfo) == 0)
    {
        System_printf ("Error(Core %d): Socket Bind test failed for invalid 8bit id\n", coreNum);
        return -1;
    }
    System_printf ("Debug(Core %d): Socket Bind test passed for invalid 8bit id\n", coreNum);

    /* Let us now try with a valid 8 bit binding: The same id we placed in the Device ID CSR. */
    bindInfo.type11.tt       = FALSE;             /* We are using 8 bit identifiers.                  */
    bindInfo.type11.id       = DEVICE_ID1_8BIT;   /* Source Identifier to which the socket is bound   */
    bindInfo.type11.letter   = 2;                 /* Letter Identifier                                */
    bindInfo.type11.mbox     = 1;                 /* Mailbox Number                                   */
    bindInfo.type11.segMap   = 0x0;               /* Single Segment                                   */

    /* Bind the SRIO socket: This should pass since since this id is present in the SRIO Device ID CSR */ 
    if (Srio_sockBind (srioSocket1, &bindInfo) < 0)
    {
        System_printf ("Error(Core %d): Socket Bind test failed for valid 8bit device-id CSR\n", coreNum);
        return -1;
    }
    System_printf ("Debug(Core %d): Socket Bind test passed for valid 8bit device-id CSR\n", coreNum);

    /* Close the socket. */
    Srio_sockClose (srioSocket1);

    /* Open the socket again. */
    srioSocket1 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, FALSE);
    if (srioSocket1 == NULL)
    {
        System_printf ("Error: Unable to open socket1\n");
        return -1;
    }

    /* Let us now try with a valid 8 bit binding: The same id we placed in the BRR. */
    bindInfo.type11.tt       = FALSE;             /* We are using 8 bit identifiers.                  */
    bindInfo.type11.id       = DEVICE_ID2_8BIT;   /* Source Identifier to which the socket is bound   */
    bindInfo.type11.letter   = 2;                 /* Letter Identifier                                */
    bindInfo.type11.mbox     = 1;                 /* Mailbox Number                                   */
    bindInfo.type11.segMap   = 0x0;               /* Single Segment                                   */

    /* Bind the SRIO socket: This should pass since since this id is present in the SRIO BRR Table */ 
    if (Srio_sockBind (srioSocket1, &bindInfo) < 0)
    {
        System_printf ("Error(Core %d): Socket Bind test failed for valid 8bit BRR CSR\n", coreNum);
        return -1;
    }
    System_printf ("Debug(Core %d): Socket Bind test passed for valid 8bit BRR CSR\n", coreNum);
    System_printf ("Debug(Core %d): Test 5 Passed\n", coreNum);

    /* Close the socket. */
    Srio_sockClose (srioSocket1);   

    /*************************************************************************************
     * TEST 6: 
     *  a) Use first available Letter, assigned by the hardware. Hardware will check for 
     *     an unused context starting with letter = 0, and incrementing to letter = 3. 
     *  b) Send Data from socket1 to socket2
     *  c) Make sure data is received on socket2
     *  d) Verify that data was received from the correct source.
     *  e) Validate data payload and length
     *************************************************************************************/
    /* Open SRIO Socket 1 */
    srioSocket1 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, FALSE);
    if (srioSocket1 == NULL)
    {
        System_printf ("Error: Test 6 - Unable to open socket1\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;             /* We are using 16 bit identifiers.                */
    bindInfo.type11.id       = DEVICE_ID1_16BIT; /* Source Identifier to which the socket is bound  */
    bindInfo.type11.letter   = SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE;  /* Letter Identifier          */
    bindInfo.type11.mbox     = coreNum;          /* Mailbox Number                                  */
    bindInfo.type11.segMap   = 0x0;              /* Single Segment                                  */

    /* Bind the SRIO socket 1 */ 
    if (Srio_sockBind (srioSocket1, &bindInfo) < 0)
    {
        System_printf ("Error: Test 6 - socket1 bind failed\n");
        return -1;
    }

    /* Open the SRIO Socket2 for Type11 */
    srioSocket2 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_TYPE11, FALSE);
    if (srioSocket2 == NULL)
    {
        System_printf ("Error: Test 6 - Unable to open socket2\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;             /* We are using 16 bit identifiers.                */
    bindInfo.type11.id       = DEVICE_ID2_16BIT; /* Source Identifier to which the socket is bound  */
    bindInfo.type11.letter   = SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE;   /* Letter Identifier         */
    bindInfo.type11.mbox     = coreNum;          /* Mailbox Number                                  */
    bindInfo.type11.segMap   = 0x0;              /* Single Segment                                  */

    /* Bind the SRIO Socket 2 */ 
    if (Srio_sockBind (srioSocket2, &bindInfo) < 0)
    {
        System_printf ("Error: Test 6 - socket2 bind failed\n");
        return -1;
    }
	
    /* Get a transmit buffer from the SRIO Driver. */
    hDrvBuffer = Srio_allocTransmitBuffer(hSrioDrv, &txData, &allocatedLen);
    if (hDrvBuffer == NULL)
        return -1;

    /* Create the transmit data payload. */
    for (idx = 0; idx < SRIO_DATA_LEN; idx++)
        txData[idx] = idx;

    /* Now we want to send data from the Socket1 to Socket2
     * So we populate the destination information where the packet is to be sent. */ 
    to.type11.tt       = TRUE;             /* We are using 16 bit identifiers.          */
    to.type11.id       = DEVICE_ID2_16BIT; /* Identifier where the packet is to be sent */
    to.type11.letter   = SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE;  /* Letter Identifier    */
    to.type11.mbox     = coreNum;          /* Mailbox Number                            */

    /* Send out some data from socket1 to socket2 */
    if (Srio_sockSend (srioSocket1, hDrvBuffer, SRIO_DATA_LEN, &to) < 0)
    {
        System_printf ("Error: Test 6 - SRIO Socket send failed\n");
        return -1;
    }

    /* Wait for the data to arrive. */
    while (1)
    {
        /* Receive data */
        num_bytes = Srio_sockRecv (srioSocket2, (Srio_DrvBuffer*)&ptr_rxDataPayload, &from);
        if (num_bytes > 0)
        {
            /* Valid data was received. Check if the length received matches the length transmitted. */
            if (num_bytes != SRIO_DATA_LEN) 
            {
                /* Error: Data Payload Length mismatch... */
                System_printf ("Error: Test 6 - Invalid data payload received\n");
                return -1;
            }

            /* Make sure that the data was received from socket1 */
            if (from.type11.id != DEVICE_ID1_16BIT)
            {
                System_printf ("Error: Test 6 - Data received from invalid source id 0x%x Expected 0x%x\n", 
                               from.type11.id, DEVICE_ID1_16BIT);
                return -1;
            }

            /* Received and Transmitted packet length match. Payload verification */
            for (idx = 0; idx < SRIO_DATA_LEN; idx++)
            {
                if (ptr_rxDataPayload[idx] != idx)
                {
                    System_printf ("Error: Test 6 - Receive Data Payload (0x%p) verification failed @ index %d\n", 
                                    ptr_rxDataPayload,idx);
                    return -1;
                }
            }

            /* Cleanup the received data payload. */
            Srio_freeRxDrvBuffer(srioSocket2, (Srio_DrvBuffer)ptr_rxDataPayload);
            break;
        }
    }

    /* Control comes here implies that data was received and validated */
    System_printf ("Debug(Core %d): Test 6 Passed\n", coreNum);

    /* Close the sockets. */
    Srio_sockClose (srioSocket1);
    Srio_sockClose (srioSocket2);

    /* All tests passed. */
    return 0;
}

