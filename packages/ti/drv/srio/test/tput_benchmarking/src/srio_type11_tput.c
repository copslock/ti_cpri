/*
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com/
 *
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

/**
 *   @file  srio_type11_tput.c
 *
 *   @brief   
 *      Type-11 Consumer (Receive) and Producer (Transmit) benchmarking throughput and latency
 *      functions. Please see benchmarking.c brief for more details.
 *
 */

/* Standard library, XDC and sysbios Include Files. */
#include <xdc/runtime/System.h>

/* CSL Chip Functional Layer */
#include <ti/csl/csl_chip.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>

/* Application Include Files */
#include <srio_tput.h>
#include <benchmarking.h>

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

/* Maximum letter and mailbox combinations supported */
#define MAX_LT_AND_MB_COMBINATIONS	16

/* Maximum delay cycles used for throughput pacing search */
#define MAX_PACING_DELAY_CYCLES		2000

/* Number of cycles to delay before starting the producer
 * side functions to ensure that the consumer is ready.
 */
#define PRODUCER_START_DELAY_CYCLES	3000000000

/* Defines to enable debug messages for TX/RX control exchange for debugging purposes only */
//#define DEBUG_CONTROL_MESSAGES_TX
//#define DEBUG_CONTROL_MESSAGES_RX

/**********************************************************************
 *********************** External Definitions *************************
 **********************************************************************/

/* Variables to control test run. */
extern struct_testcontrol testControl;

/* Application variables */
extern uint32_t srio_iterationCount;
extern uint64_t srio_numPackets;
extern uint64_t srio_latencyNumPackets;
extern bool srio_usePolledMode;
extern uint32_t srio_device_ID1;	/* Consumer */
extern uint32_t srio_device_ID2;	/* Producer */

/* Global SRIO Driver Handle. */
extern Srio_DrvHandle      hAppManagedSrioDrv;

/* Memory Heap Queue which has descriptors and buffers allocated and linked together. */
extern Qmss_QueueHnd       memoryHeapQueue;


/**********************************************************************
 ******************** CONSUMER/PRODUCER FUNCTIONS *********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      Read TSCL+TSCH and return as a 64 bit unsigned integer.
 *
 *  @retval
 *      TSCL+TSCH as 64bit unsigned integer
 */
static inline uint64_t tputExampleReadTime ()
{
  uint32_t low = TSCL;
  uint32_t high = TSCH;
  return _itoll(high,low);
}

/**
 *  @b Description
 *  @n  
 *      The function allocates memory.
 *
 *  @param[in]  size
 *      Number of bytes of data to be allocated
 *
 *  @retval
 *      Success -   Allocated Host Descriptor.
 *  @retval
 *      Error   -   NULL
 */
static Cppi_HostDesc* allocMemory (uint32_t size)
{
    Cppi_HostDesc*  ptrHostDesc;

    /* If the allocated size is greater than the MAX allowed */
    if (size > MESSAGE_MAX_DATA_SIZE)
        return NULL;

    /* Pop off a descriptor. */
    ptrHostDesc = Qmss_queuePop (memoryHeapQueue);

    /* Return the allocated descriptor. */
    return ptrHostDesc;
}

/**
 *  @b Description
 *  @n  
 *      The function frees memory.
 *
 *  @param[in]  ptrHostDesc
 *      Descriptor to be cleaned up.
 *
 *  @retval
 *      Not Applicable.
 */
static void freeMemory (Cppi_HostDesc* ptrHostDesc)
{
    /* Pop off a descriptor. */
    Qmss_queuePushDesc (memoryHeapQueue, (uint32_t*)ptrHostDesc);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send the specified message over the control socket.
 *
 *  @param[in]  controlSocket
 *      Control Socket over which the message is to be sent.
 *  @param[in]  message
 *      Message to be sent.
 *  @param[in]  to
 *      Destination information to where the control message is to be sent.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t sendControlMessage (Srio_SockHandle controlSocket, uint32_t message, Srio_SockAddrInfo to, uint16_t segMap, char* infoText)
{
    Cppi_HostDesc*          ptrHostDesc;
    ProdCons_ControlPkt*    ptrControlPacket;
    uint32_t                rxNumBytes;
    uint32_t                packetLength;
    uint64_t				tscTemp = (tputExampleReadTime() + (uint64_t)5000000000);	// set timeout for waiting for descriptor to be free

    /* Wait until timeout period to get a free TX queue as TX queue may be full */
    while (tputExampleReadTime() < tscTemp)
    {
    	/* Allocate a descriptor to send the message. */
    	ptrHostDesc = allocMemory (sizeof(message));
    	if (ptrHostDesc != NULL)
    		break;
    }
    if (ptrHostDesc == NULL)
    {
        System_printf ("Error: %s Unable to get descriptor for sending the control message\n", infoText);
        return -1;
    }

#ifdef DEBUG_CONTROL_MESSAGES_TX
    System_printf ("Debug: %s sending %s\n", infoText, controlDecodeText(message));
#endif

    /* Get the data buffer of the allocated packet. */
    Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(uint8_t**)&ptrControlPacket, &rxNumBytes);

    /* Populate this with the Request. */
	ptrControlPacket->pktType       = ProdCons_ControlPktType_CTRL_MSG;
    ptrControlPacket->id            = message;
	ptrControlPacket->payloadLength = 0;

    /* Total Packet Length includes the header + payload */
	packetLength = ptrControlPacket->payloadLength + SOFTWARE_HEADER_BYTES;

    if (segMap == 1)
    {
		/* Sanity Check: The RAW socket created is a Multi-segment packet. So we cannot send
		 * less than 256 bytes. On the other hand if the socket created was Single Segment we
		 * could not have sent a packet greater than 256 bytes. According to the SRIO
		 * specification depending on the Single Segment or Multiple Segment the Mailbox size
		 * can go from 6 bits to 2 bits. */
		if (packetLength <= 256)
			packetLength = 260;

		/* Ensure that the number of bytes being transmitted is a multiple of double-word.
		 * This is as per the specification. */
		packetLength = ((packetLength + 7) & ~0x7);
    }
    else
		packetLength = testControl.srio_payloadSize + SOFTWARE_HEADER_BYTES;

    /* Set the data length & packet length. */
	Cppi_setDataLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, packetLength);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)ptrHostDesc, packetLength);

    /* Send the control message */
    if (Srio_sockSend (controlSocket, (Srio_DrvBuffer)ptrHostDesc, SIZE_HOST_DESC, &to) < 0)
	{
	    System_printf ("Error: (%s) Sending Control Message Failed.\n", infoText);
		return -1;
    }
    /* Control message has been transmitted. */
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to receive the specified message over the control socket.
 *
 *  @param[in]  controlSocket
 *      Control Socket over which the message is to be received.
 *  @param[out] controlMessage
 *      Control Message populated
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t recvControlMessage (Srio_SockHandle controlSocket, uint32_t* controlMessage, bool isBlocking, char* infoText)
{
    Cppi_HostDesc*          ptrHostDesc;
    Srio_SockAddrInfo       from;
    int32_t                 num_bytes;
    ProdCons_ControlPkt*    ptrControlPacket;
    uint32_t                rxNumBytes;

#ifdef DEBUG_CONTROL_MESSAGES_RX
    if (isBlocking)
    	System_printf ("Debug: %s waiting for a control message.\n", infoText);
#endif
	/* Check if polled mode or not? */
	if (srio_usePolledMode)
	{
		/* Polled Mode: Loop around waiting for the control message to arrive. */
		while (1)
		{
			/* Poll the ISR. */
			Srio_rxCompletionIsr (hAppManagedSrioDrv);

			/* Check if there is data. */
			num_bytes = Srio_sockRecv (controlSocket, (Srio_DrvBuffer*)&ptrHostDesc, &from);
			if (num_bytes < 0)
				return -1;
			if (num_bytes > 0)
				break;
			if (!isBlocking)
			{
				if (num_bytes == 0)
					return -1;
			}
		}
	}
	else
	{
		/* Interrupt Mode: */
		num_bytes = Srio_sockRecv (controlSocket, (Srio_DrvBuffer*)&ptrHostDesc, &from);
		if (num_bytes <= 0)
			return -1;
	}

	/* Get the data buffer of the allocated packet. */
	Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(uint8_t**)&ptrControlPacket, &rxNumBytes);

	/* Make sure this is a control message. */
	if (ptrControlPacket->pktType != ProdCons_ControlPktType_CTRL_MSG)
	{
		System_printf ("Error: (%s)Invalid message received over control socket\n", infoText);
		return -1;
	}

    /* Remember the control message. */
    *controlMessage = ptrControlPacket->id;

    /* Cleanup the received packet. */
	Srio_freeRxDrvBuffer (controlSocket, ptrHostDesc);

#ifdef DEBUG_CONTROL_MESSAGES_RX
	System_printf ("Debug: %s received %s\n", infoText, controlDecodeText(*controlMessage));
#endif
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to wait for the specified message over the control socket.
 *
 *  @param[in]  ptrControlSocket
 *      Pointer to the Control Socket over which the message is to be received.
 *  @param[in]  rcvCtrlMessage
 *      Message to expect from the Control Socket.
 *  @param[in]  dest
 *      Destination information to where the control message is to be sent.
 *  @param[in]  callID
 *      Text to identify from where this was called.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t ctrlWait (Srio_SockHandle* ptrControlSocket, uint32_t rcvCtrlMessage, Srio_SockAddrInfo* dest, char* callID)
{
	uint32_t	receivedCtrlMessage=0;
	int32_t		returnStatus = -1;

	/* Receive control message */
	if (recvControlMessage(ptrControlSocket, &receivedCtrlMessage, CHECK_BLOCKING, callID) < 0)
		System_printf ("%s Error: Failed to receive %s control message within timeout period.\n", callID, controlDecodeText(rcvCtrlMessage));
	else
	{
		/* Verify that correct control message was received */
		if (receivedCtrlMessage != rcvCtrlMessage)
			System_printf ("%s Error: Incorrect message received. Expected: %s, received: %s \n", callID, controlDecodeText(rcvCtrlMessage), controlDecodeText(receivedCtrlMessage));
		else
			returnStatus = 0;
	}
	return returnStatus;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send the specified message and wait for the specified message over
 *      the control socket.
 *
 *  @param[in]  ptrControlSocket
 *      Pointer to the Control Socket over which the message is to be received.
 *  @param[in]  sndCtrlMessage
 *      Message to send to the Control Socket.
 *  @param[in]  rcvCtrlMessage
 *      Message to expect from the Control Socket.
 *  @param[in]  dest
 *      Destination information to where the control message is to be sent.
 *  @param[in]  callID
 *      Text to identify from where this was called.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t ctrlSendWait (Srio_SockHandle* ptrControlSocket, uint32_t sndCtrlMessage, uint32_t rcvCtrlMessage, Srio_SockAddrInfo* dest, uint16_t segMap, char* callID)
{
	uint32_t	receivedCtrlMessage=0;
	int32_t		returnStatus = -1;

	/* Send control message */
	if (sendControlMessage(ptrControlSocket, sndCtrlMessage, *dest, segMap, callID) < 0)
		System_printf ("%s Error: %s control message failed to be sent. CallID: %s\n", callID, controlDecodeText(sndCtrlMessage), callID);
	else
	{
		/* Receive control message */
		if (recvControlMessage(ptrControlSocket, &receivedCtrlMessage, CHECK_BLOCKING, callID) < 0)
			System_printf ("%s Error: Failed to receive %s control message within timeout period.\n", callID, controlDecodeText(rcvCtrlMessage));
		else
		{
			/* Verify that correct control message was received */
			if (receivedCtrlMessage != rcvCtrlMessage)
				System_printf ("%s Error: Incorrect message received. Expected: %s, received: %s \n", callID, controlDecodeText(rcvCtrlMessage), controlDecodeText(receivedCtrlMessage));
			else
				returnStatus = 0;
		}
	}
	return returnStatus;
}

/**
 *  @b Description
 *  @n
 *      The function is used to send the specified message and wait for a message over the control
 *      socket. The received message will be returned.
 *
 *  @param[in]  ptrControlSocket
 *      Pointer to the Control Socket over which the message is to be received.
 *  @param[in]  sndCtrlMessage
 *      Message to send to the Control Socket.
 *  @param[in]  dest
 *      Destination information to where the control message is to be sent.
 *  @param[in]  callID
 *      Text to identify from where this was called.
 *
 *  @retval
 *      Success - message received from the control socket.
 *  @retval
 *      Error   - 0
 */
static uint32_t ctrlSendWaitGetResponse (Srio_SockHandle* ptrControlSocket, uint32_t sndCtrlMessage, Srio_SockAddrInfo* dest, uint16_t segMap, char* callID)
{
	static uint32_t	receivedCtrlMessage=0;

	/* Send control message */
	if (sendControlMessage(ptrControlSocket, sndCtrlMessage, *dest, segMap, callID) < 0)
		System_printf ("%s Error: %s control failed to be sent.\n", callID, controlDecodeText(sndCtrlMessage));
	else
	{
		/* Receive control message */
		if (recvControlMessage(ptrControlSocket, &receivedCtrlMessage, CHECK_BLOCKING, callID) < 0)
			System_printf ("%s Error: Failed to receive a control message within timeout period.\n", callID);
	}
	return receivedCtrlMessage;
}

/**
 *  @b Description
 *  @n
 *      This is the consumer Type11 latency test.
 *
 *  @retval
 *      Success - message received from the control socket.
 *  @retval
 *      Error   - 0
 */
Int32 consumerType11LatencyTest (void)
{
    Srio_SockHandle         dataSocket;
    Srio_SockHandle*        controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Cppi_HostDesc*          ptrHostDesc;
    Cppi_HostDesc*          ptrHostDescSend;
    ProdCons_ControlPkt*    ptrControlPacket;
    Srio_SockAddrInfo       from, controlMsgDest;
    uint32_t                num_bytes, dataSizeBytes, packetLength, heapCounter, rxNumBytes, index;
    uint32_t				iterationCount=0, maxIterationCount=1, packetID=0;
    uint32_t				thisMaxIterationCount=0;
    uint64_t				packetCount=0, iterationPacketCount=0;
    uint16_t                counter = NUM_RX_DESC;
    bool					useBlockingSocket;
    bool					isDisplayInitialInfo = TRUE;

    /* Return immediately if latency test is not enabled */
    if (!testControl.srio_isLatencyTest)
    	return 0;

    /* Clear RX Output Buffer*/
	clearSrioStatsOutputBuffer ();

	/* Set the initial data Size. */
	dataSizeBytes = testControl.srio_payloadSize;

	/* Set the initial iteration count. */
	thisMaxIterationCount = 1;

	/* Packet sizes loop. */
    while (dataSizeBytes <= testControl.srio_payloadEndSize)
    {
		/* Create a Control Type11 NON-BLOCKING Socket if polled mode, and a BLOCKING Socket if not polled mode. */
		useBlockingSocket = (srio_usePolledMode) ? FALSE : TRUE;
		dataSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_RAW_TYPE11, useBlockingSocket);
		if (dataSocket == NULL)
		{
			System_printf ("Error: RX Data Socket Open Failed\n");
			return -1;
		}

		/* Populate the binding information. */
		bindInfo.type11.tt       = TRUE;
		bindInfo.type11.id       = srio_device_ID1;
		bindInfo.type11.letter   = CONSUMER_LTR;
		bindInfo.type11.mbox     = CONSUMER_MBOX;
		bindInfo.type11.segMap    = ((dataSizeBytes + SOFTWARE_HEADER_BYTES > 256) ? 1 : 0);

		/* Bind the socket. */
		if (Srio_sockBind (dataSocket, &bindInfo) < 0)
		{
			System_printf ("Error: Data Socket Binding Failed.\n");
			return -1;
		}

		if (isDisplayInitialInfo)
		{
			if (testControl.srio_isRunProgress)
			{
				System_printf ("------- Latency Measurment -------\n");
				System_printf ("Consumer Type11 Socket Properties:\n");
				System_printf ("  Device Identifier:0x%x\n", bindInfo.type11.id);
				System_printf ("  Mailbox          :0x%x\n", bindInfo.type11.mbox);
				System_printf ("  Letter           :0x%x\n", bindInfo.type11.letter);
				System_printf ("----------------------------------\n");
			}
		}

		/* Ensure that there is sufficient space in the SRIO socket to hold all the packets. */
		if (Srio_setSockOpt (dataSocket, Srio_Opt_PENDING_PKT_COUNT, (void *)&counter, sizeof(counter)) < 0)
		{
			System_printf ("Error: SRIO Socket Set options failed\n");
			return -1;
		}

		/* Set the control socket:  This is used to exchange control messages */
	    controlSocket = (Srio_SockHandle*)dataSocket;

	  /* Populate the destination information to be used for the control messages. */
	  controlMsgDest.type11.tt       = TRUE;
		controlMsgDest.type11.id       = srio_device_ID2;
		controlMsgDest.type11.letter   = ((PRODUCER_LTR == SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE) ? PRODUCER_CTRL_LTR : PRODUCER_LTR);
		controlMsgDest.type11.mbox     = ((PRODUCER_MBOX == SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE) ? PRODUCER_CTRL_MBOX : PRODUCER_MBOX);

		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_latencyNumPackets);

		/* Set maxIterationCount */
		maxIterationCount = thisMaxIterationCount;

		/* Get the initial heap counter */
		heapCounter = Qmss_getQueueEntryCount (memoryHeapQueue);
		
		/* Initialize all the packets. */
		while (heapCounter != 0)
		{
			/* Allocate a transmit packet */
			ptrHostDescSend = allocMemory (sizeof(ProdCons_ControlPkt));
			if (ptrHostDescSend == NULL)
			{
				System_printf ("Error: Memory Allocation Failed\n");
				return -1;
			}

			/* Display message if current packet size is equal to the initial packet size */
			if (dataSizeBytes == testControl.srio_payloadSize)
			{
				debugPrintf_p ("Debug: Setting up descriptor 0x%p\n", ptrHostDescSend);
			}

			/* Get the data buffer pointer of the allocated packet. */
			Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDescSend,(uint8_t**)&ptrControlPacket, &rxNumBytes);

			/* Populate this with the Request. */
			ptrControlPacket->pktType   = ProdCons_ControlPktType_REQUEST;
			ptrControlPacket->id        = 0;
			ptrControlPacket->payloadLength = dataSizeBytes;

			/* Set the packets payload data */
			for (index = 0; index < ptrControlPacket->payloadLength; index++)
				ptrControlPacket->pktPayload[index] = index;

			/* Total Packet Length includes the header + payload */
			packetLength = ptrControlPacket->payloadLength + SOFTWARE_HEADER_BYTES;

			/* Sanity Check: The RAW socket created is a Multi-segment packet. So we cannot send
			 * less than 256 bytes. On the other hand if the socket created was Single Segment we
			 * could not have sent a packet greater than 256 bytes. According to the SRIO
			 * specification depending on the Single Segment or Multiple Segment the Mailbox size
			 * can go from 6 bits to 2 bits. */
			if (testControl.srio_payloadSize + SOFTWARE_HEADER_BYTES > 256)
			{
				if (packetLength <= 256)
					packetLength = 260;

				/* Ensure that the number of bytes being transmitted is a multiple of double-word.
				 * This is as per the specification. */
				packetLength = ((packetLength + 7) & ~0x7);
			}

			/* Set the data length & packet length. */
			Cppi_setDataLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDescSend, packetLength);
			Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)ptrHostDescSend, packetLength);

			/* Clean the packet. */
			freeMemory (ptrHostDescSend);

			/* Goto the next packet. */
			heapCounter = heapCounter - 1;
		}

		/* Loop around the number of iterations until we receive all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Pre-set send packet so that there is no delay in sending it */
			while (1)
			{
				/* Try to get the next TX descriptor. */
				ptrHostDescSend = allocMemory (sizeof(ProdCons_ControlPkt));
				/* Exit this loop if the TX descriptor is available */
				if (ptrHostDescSend != NULL)
					break;
			}

			/* Get the data buffer pointer of the allocated packet. */
			Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDescSend,(uint8_t**)&ptrControlPacket, &rxNumBytes);

			/* Set the identifier in the packet */
			ptrControlPacket->id = packetID;

			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_latencyNumPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Wait for the data to arrive. */
				if (srio_usePolledMode)
					Srio_rxCompletionIsr (hAppManagedSrioDrv);

				/* Receive data from the socket. */
				num_bytes = Srio_sockRecv (dataSocket, (Srio_DrvBuffer*)&ptrHostDesc, &from);
				if (num_bytes <= 0)
					continue;

				/* Send the prefabricated packet. */
				if (Srio_sockSend (controlSocket, (Srio_DrvBuffer)ptrHostDescSend, SIZE_HOST_DESC, &controlMsgDest) < 0)
				{
					System_printf ("Error: Sending Data Failed.\n");
					return -1;
				}

				/* Get the actual descriptor. */
				ptrHostDesc = (Cppi_HostDesc*)QMSS_DESC_PTR(ptrHostDesc);

				/* Free queue buffer. Cleanup the received packet. */
				Srio_freeRxDrvBuffer (dataSocket, ptrHostDesc);
				
				/* Increment the packet counter. */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;

				if (packetCount >= iterationPacketCount)
					break;

				/* Pre-set send packet so that there is no delay in sending it */
				while (1)
				{
					/* Try to get the next TX descriptor. */
					ptrHostDescSend = allocMemory (sizeof(ProdCons_ControlPkt));
					/* Exit this loop if the TX descriptor is available */
					if (ptrHostDescSend != NULL)
						break;
				}

				/* Get the data buffer pointer of the allocated packet. */
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDescSend,(uint8_t**)&ptrControlPacket, &rxNumBytes);

				/* Set the identifier in the packet */
				ptrControlPacket->id = packetID;
			}

			/* Increment the iteration counter. */
			iterationCount += 1;

		}
		/* Calculate the next packet data size to be used */
		dataSizeBytes = ((dataSizeBytes + SOFTWARE_HEADER_BYTES) * 2) - SOFTWARE_HEADER_BYTES;

		/* Wait a little for TX to be ready to receive the control message */
		cycleDelay ((uint64_t)0500000000);

		/* Let the TX side know that all packets were received successfully. */
		sendControlMessage (controlSocket, CTRL_SUCCESS, controlMsgDest, bindInfo.type11.segMap, "RX_1286");

		/* Initial variables for the next run */
		iterationCount = 0;
		packetCount = 0;
		packetID = 0;

		/* Close the data socket */
		Srio_sockClose (dataSocket);

		/* Turn off the initial information headers for the remaining packet sizes. */
		isDisplayInitialInfo = FALSE;
	}

	/* Wait 1 second to stop the intermixing of RX and TX data when
	 * using a common terminal display as with core to core testing. */
	cycleDelay ((uint64_t)1000000000);

	return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the producer Type11 latency test.
 *
 *  @retval
 *      Success - message received from the control socket.
 *  @retval
 *      Error   - 0
 */
Int32 producerType11LatencyTest (void)
{
    Srio_SockHandle         dataSocket;
    Srio_SockHandle*		controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Cppi_HostDesc*          ptrHostDesc;
    Cppi_HostDesc*          ptrHostDescRecv;
    Srio_SockAddrInfo       to[MAX_LT_AND_MB_COMBINATIONS], from, controlMsgDest;
    ProdCons_ControlPkt*    ptrControlPacket;
    uint32_t                num_bytes, rxNumBytes, packetLength, index, heapCounter;
    uint32_t				mailBoxesToCycle, lettersToCycle, combinationsToCycle, mbIndex, ltIndex;
    uint32_t				iterationCount=0, maxIterationCount=0, dataSizeBytes=0, toIndex=0;
    uint32_t				thisMaxIterationCount=0, packetID=0, lldTmrStart, currentCycles=0;
    uint64_t				errEnd, packetCount=0, iterationPacketCount=0;
    uint16_t                counter = NUM_RX_DESC;
    bool					isDisplayInitialInfo = TRUE;
    Tput_Parameters			tparams;

    /* Return immediately if latency test is not enabled */
    if (!testControl.srio_isLatencyTest)
    	return 0;
    
    /* Wait a little for the RX side to be ready before we start the TX side */
    cycleDelay ((uint64_t)PRODUCER_START_DELAY_CYCLES);

    /* Clear TX Output Buffer*/
    clearSrioStatsOutputBuffer ();

    mailBoxesToCycle = ((CONSUMER_MBOX == SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE) ? 4 : 1);
    lettersToCycle = ((CONSUMER_LTR == SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE) ? 4 : 1);
    combinationsToCycle = mailBoxesToCycle * lettersToCycle;

    /* Set initial tparams */
    tparams.numberOfPackets = 0;
    tparams.packetSizeBytes = testControl.srio_payloadSize;
    tparams.overheadBytes = 0;
    tparams.pacingCycles = 0;
    tparams.isShowHeader = TRUE;
    tparams.isTestInfoOnly = TRUE;
    tparams.fType = (uint32_t)Srio_Ftype_MESSAGE;
    tparams.tType = (uint32_t)0;

    /* Display test information */
    displayLatencyStatistics (&tparams, ONE_WAY_TIME);
    tparams.isTestInfoOnly = FALSE;

    /* Set the initial data Size. */
    dataSizeBytes = testControl.srio_payloadSize;

    /* Set the initial max iteration count. */
    thisMaxIterationCount = 1;

    /* Packet sizes loop. */
    while (dataSizeBytes <= testControl.srio_payloadEndSize)
    {
    	/* Clear totals for this packet size */
		tparams.totalLldCycles = 0;
		tparams.totalProcCycles = 0;
		tparams.totalIdleCycles = 0;
		tparams.minLatCycles = 0xFFFFFFFF;
		tparams.maxLatCycles = 0;

		/* Create a Type11 NON-BLOCKING Socket. */
		dataSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_RAW_TYPE11, FALSE);
		if (dataSocket == NULL)
		{
			System_printf ("Error: TX Data Socket Open Failed\n");
			return -1;
		}

		/* Populate the binding information. */
		bindInfo.type11.tt       = TRUE;
		bindInfo.type11.id       = srio_device_ID2;
		bindInfo.type11.letter   = PRODUCER_LTR;
		bindInfo.type11.mbox     = PRODUCER_MBOX;
		bindInfo.type11.segMap = ((dataSizeBytes + SOFTWARE_HEADER_BYTES > 256) ? 1 : 0);

		/* Bind the socket. */
		if (Srio_sockBind (dataSocket, &bindInfo) < 0)
		{
			System_printf ("Error: Data Socket Binding Failed.\n");
			return -1;
		}

		if (isDisplayInitialInfo)
		{
			if (testControl.srio_isRunProgress)
			{
				/* Debug Display Message: */
				System_printf ("------- Latency Measurment -------\n");
				System_printf ("Producer Type11 Socket Properties:\n");
				System_printf ("  Device Identifier:0x%x\n", bindInfo.type11.id);
				System_printf ("  Mailbox          :0x%x\n", bindInfo.type11.mbox);
				System_printf ("  Letter           :0x%x\n", bindInfo.type11.letter);
				System_printf ("----------------------------------\n");
			}
		}

		/* Ensure that there is sufficient space in the SRIO socket to hold all the packets. */
		if (Srio_setSockOpt (dataSocket, Srio_Opt_PENDING_PKT_COUNT, (void *)&counter, sizeof(counter)) < 0)
		{
			System_printf ("Error: SRIO Socket Set options failed\n");
			return -1;
		}

    toIndex = 0;
    for (ltIndex=0; ltIndex < mailBoxesToCycle; ltIndex++)
    {
      for (mbIndex=0; mbIndex < mailBoxesToCycle; mbIndex++)
      {
        /* Populate the Consumer device where the data is destined to. */
        to[toIndex].type11.tt		= TRUE;
        to[toIndex].type11.id		= srio_device_ID1;
        to[toIndex].type11.letter	= ((CONSUMER_LTR == SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE) ? ltIndex : CONSUMER_LTR);
        to[toIndex].type11.mbox		= ((CONSUMER_MBOX == SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE) ? mbIndex : CONSUMER_MBOX);

        if (isDisplayInitialInfo)
        {
          /* Debug Display Message: */
          debugPrintf_ddd  ("Sending to Device Id:0x%x Mailbox:%d Letter:%d\n",
              to[toIndex].type11.id, to[toIndex].type11.mbox, to[toIndex].type11.letter);
        }
        if (toIndex < (MAX_LT_AND_MB_COMBINATIONS - 1))
          toIndex += 1;
      }
    }

    /* Default the index to zero */
    toIndex = 0;

    /* Set the control socket:  This is used to exchange control messages */
    controlSocket = (Srio_SockHandle*)dataSocket;

    /* Populate the destination information for the control messages. */
    controlMsgDest.type11.tt       = TRUE;
		controlMsgDest.type11.id       = srio_device_ID1;
		controlMsgDest.type11.letter   = ((CONSUMER_LTR == SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE) ? CONSUMER_CTRL_LTR : CONSUMER_LTR);
		controlMsgDest.type11.mbox     = ((CONSUMER_MBOX == SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE) ? CONSUMER_CTRL_MBOX : CONSUMER_MBOX);

		/* Get the initial heap counter */
		heapCounter = Qmss_getQueueEntryCount (memoryHeapQueue);

		/* Initialize all the packets. */
		while (heapCounter != 0)
		{
			/* Allocate a transmit packet */
			ptrHostDesc = allocMemory (sizeof(ProdCons_ControlPkt));
			if (ptrHostDesc == NULL)
			{
				System_printf ("Error: Memory Allocation Failed\n");
				return -1;
			}

			/* Display message if current packet size is equal to the initial packet size */
			if (dataSizeBytes == testControl.srio_payloadSize)
			{
				debugPrintf_p ("Debug: Setting up descriptor 0x%p\n", ptrHostDesc);
			}

			/* Get the data buffer pointer of the allocated packet. */
			Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(uint8_t**)&ptrControlPacket, &rxNumBytes);

			/* Populate this with the Request. */
			ptrControlPacket->pktType   = ProdCons_ControlPktType_REQUEST;
			ptrControlPacket->id        = 0;
			ptrControlPacket->payloadLength = dataSizeBytes;

			/* Set the packets payload data */
			for (index = 0; index < ptrControlPacket->payloadLength; index++)
				ptrControlPacket->pktPayload[index] = index;

			/* Total Packet Length includes the header + payload */
			packetLength = ptrControlPacket->payloadLength + SOFTWARE_HEADER_BYTES;

			/* Sanity Check: The RAW socket created is a Multi-segment packet. So we cannot send
			 * less than 256 bytes. On the other hand if the socket created was Single Segment we
			 * could not have sent a packet greater than 256 bytes. According to the SRIO
			 * specification depending on the Single Segment or Multiple Segment the Mailbox size
			 * can go from 6 bits to 2 bits. */
			if (testControl.srio_payloadSize + SOFTWARE_HEADER_BYTES > 256)
			{
				if (packetLength <= 256)
					packetLength = 260;

				/* Ensure that the number of bytes being transmitted is a multiple of double-word.
				 * This is as per the specification. */
				packetLength = ((packetLength + 7) & ~0x7);
			}

			/* Set the data length & packet length. */
			Cppi_setDataLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, packetLength);
			Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)ptrHostDesc, packetLength);

			/* Clean the packet. */
			freeMemory (ptrHostDesc);

			/* Goto the next packet. */
			heapCounter = heapCounter - 1;
		}

		/* Calculate the total number of packets that will be tested */
		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_latencyNumPackets);

		/* Set max iteration count */
		maxIterationCount = thisMaxIterationCount;
		if (isDisplayInitialInfo)
		{
		    if (testControl.srio_isRunProgress)
		    {
				System_printf ("Debug: Measuring latency for %d to %d byte packets. Packet count: %s.\n",
						(dataSizeBytes + SOFTWARE_HEADER_BYTES), (testControl.srio_payloadEndSize + SOFTWARE_HEADER_BYTES),
						uint64ToString(iterationPacketCount));
		    }
		}

		/* Take Test start time-stamp */
		tparams.tsLoopStart = tputExampleReadTime ();

		/* Loop around the number of iterations until we send all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_latencyNumPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Get idle start time-stamp and TX timeout error end time-stamp */
				errEnd = tputExampleReadTime() + (uint64_t)5000000000;
				while (1)
				{
					/* Try to get the next TX descriptor. */
					ptrHostDesc = allocMemory (sizeof(ProdCons_ControlPkt));

					/* Exit this loop if the TX descriptor is available */
					if (ptrHostDesc != NULL)
						break;

					/* Let the user know that the transmitter has stalled while sending packets.
					 * No descriptors became available before the timeout period. */
					if (tputExampleReadTime() > errEnd)
					{
						System_printf ("Error: Transmit is unable to send packets. Packet count: %s\n",
								uint64ToString(packetCount));
						errEnd += (uint64_t)5000000000;
						return -1;
					}
				}
				/* Get the data buffer pointer of the allocated packet. */
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(uint8_t**)&ptrControlPacket, &rxNumBytes);

				/* Set the identifier in the packet */
				ptrControlPacket->id = packetID;

				/* Set index to use different mailboxes and letters based on packet count to eliminate
				 * the concurrent message limitations that happen when always using the same mailbox
				 * and letter */
				toIndex = (uint32_t)((uint32_t)packetCount % (uint32_t)combinationsToCycle);

				/* Take LLD start time-stamp */
				lldTmrStart = TSCL;

				/* Send the prefabricated packet. */
				if (Srio_sockSend (dataSocket, (Srio_DrvBuffer)ptrHostDesc, SIZE_HOST_DESC, &to[toIndex]) < 0)
				{
					System_printf ("Error: Sending Data Failed.\n");
					return -1;
				}

				/* wait for RX message. */
				while (1)
				{
					/* Wait for the data to arrive. */
					if (srio_usePolledMode)
						Srio_rxCompletionIsr (hAppManagedSrioDrv);

					/* Receive data from the socket. */
					num_bytes = Srio_sockRecv (dataSocket, (Srio_DrvBuffer*)&ptrHostDescRecv, &from);
					if (num_bytes > 0)
						break;
				}

				/* Get the number of cycles taken to send and received the packet */
				currentCycles = (TSCL - lldTmrStart);
				
				/* Get the actual descriptor. */
				ptrHostDescRecv = (Cppi_HostDesc*)QMSS_DESC_PTR(ptrHostDescRecv);

				/* Free queue buffer. Cleanup the received packet. */
				Srio_freeRxDrvBuffer (controlSocket, ptrHostDescRecv);

				/* Record LLD cycles */
				tparams.totalLldCycles += currentCycles;

				/* set minimum latency cycles */
				if ((uint32_t)currentCycles < (uint32_t)tparams.minLatCycles)
					tparams.minLatCycles = (uint32_t)currentCycles;

				/* set maximum latency cycles */
				if ((uint32_t)currentCycles > (uint32_t)tparams.maxLatCycles)
					tparams.maxLatCycles = (uint32_t)currentCycles;

				/* Increment the packet counter */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;
			}

			/* Increment the iteration counter */
			iterationCount += 1;

		}
		/* Take test end time-stamp */
		tparams.tsLoopEnd = tputExampleReadTime ();

		/* Wait for SUCCESS message from RX side to indicate no packets were dropped for this packet size */
		ctrlWait (controlSocket, CTRL_SUCCESS, &controlMsgDest, "TX_1620");

		/* Display the stats */
		tparams.numberOfPackets = (uint64_t)(iterationCount * srio_latencyNumPackets);
		tparams.packetSizeBytes = packetLength;
		displayLatencyStatistics (&tparams, ROUND_TRIP_TIME);

		/* Reset control variables and counts for next packet size to be tested */
		isDisplayInitialInfo = FALSE;
		tparams.isShowHeader = FALSE;
		iterationCount = 0;
		packetCount = 0;
		packetID = 0;

		/* Calculate next packet size to be tested */
		dataSizeBytes = ((dataSizeBytes + SOFTWARE_HEADER_BYTES) * 2) - SOFTWARE_HEADER_BYTES;

    	/* Close all open data sockets */
		Srio_sockClose (dataSocket);
    }

	/* Display collected throughput statistics */
	displayStatsOutputBuffer ();

	/* Wait a little for the RX side to be ready before we start the TX side */
	cycleDelay ((uint64_t)1000000000);
	
	return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the consumer Type11 throughput function.
 *
 *  @retval
 *      Not Applicable.
 */
Int32 consumerType11Throughput (void)
{
    Srio_SockHandle         dataSocket;
    Srio_SockHandle*        controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Cppi_HostDesc*          ptrHostDesc;
    ProdCons_ControlPkt*    ptrControlPkt;
    Srio_SockAddrInfo       from, controlMsgDest;
    uint32_t                num_bytes, dataSizeBytes;
    uint32_t				iterationCount=0, maxIterationCount=0, packetID=0;
    uint32_t                controlMessage=0, rxStatus=0, thisMaxIterationCount=0;
    uint64_t				tmrEnd, rxTimeoutLength;
    uint64_t				packetCount=0, iterationPacketCount=0;
    uint64_t				lldTmrStart, lldTmrEnd;
    uint16_t                counter = NUM_RX_DESC;
    bool					useBlockingSocket, isRXfailure=FALSE, resetForNewRun=FALSE;
    bool					enPacingModeTimeoutSend = FALSE, isPacingSearchMode = FALSE, isDisplayInitialInfo = TRUE;
    Tput_Parameters			tparams;

    if (consumerType11LatencyTest() < 0)
    	System_printf("Error: Consumer latency measurement failed to complete.\n");

    /* Clear RX Output Buffer*/
	clearSrioStatsOutputBuffer ();

    /* Set the RX timeout to a shorter time for core to core, but longer time for board to board to allow time to start TX side. */
	rxTimeoutLength = (!testControl.srio_isBoardToBoard ? (uint64_t)2000000000 : (uint64_t)10000000000);

	/* Set the initial data Size. */
	dataSizeBytes = testControl.srio_payloadSize;

	/* Initialize the tparams parameters that will be passed to display the test statistics. */
	tparams.numberOfPackets = 0;
	tparams.packetSizeBytes = testControl.srio_payloadSize;
	tparams.overheadBytes = 0;
	tparams.pacingCycles = 0;
	tparams.isShowHeader = TRUE;
	tparams.isTestInfoOnly = TRUE;
	tparams.fType = (uint32_t)Srio_Ftype_MESSAGE;
	tparams.tType = (uint32_t)0;

	/* Display test information */
	displayTputStatistics (&tparams);
	tparams.isTestInfoOnly = FALSE;

	/* Set the initial iteration count. */
	thisMaxIterationCount = srio_iterationCount;

	/* Packet sizes loop. */
    while (dataSizeBytes <= testControl.srio_payloadEndSize)
    {
    	/* Set control variables for the test start of this packet size */
    	rxStatus = (uint32_t)CTRL_SUCCESS;
    	controlMessage = 0;
    	tparams.totalLldCycles = 0;
    	tparams.totalProcCycles = 0;
    	tparams.totalIdleCycles = 0;

		/* Create a Control Type11 NON-BLOCKING Socket if polled mode, and a BLOCKING Socket if not polled mode. */
		useBlockingSocket = (srio_usePolledMode) ? FALSE : TRUE;
		dataSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_RAW_TYPE11, useBlockingSocket);
		if (dataSocket == NULL)
		{
			System_printf ("Error: RX Data Socket Open Failed\n");
			return -1;
		}

		/* Populate the binding information. */
		bindInfo.type11.tt       = TRUE;
		bindInfo.type11.id       = srio_device_ID1;
		bindInfo.type11.letter   = CONSUMER_LTR;
		bindInfo.type11.mbox     = CONSUMER_MBOX;
		bindInfo.type11.segMap    = ((dataSizeBytes + SOFTWARE_HEADER_BYTES > 256) ? 1 : 0);

		/* Bind the socket. */
		if (Srio_sockBind (dataSocket, &bindInfo) < 0)
		{
			System_printf ("Error: Data Socket Binding Failed.\n");
			return -1;
		}

		if (isDisplayInitialInfo && !isPacingSearchMode)
		{
		    if (testControl.srio_isRunProgress)
		    {
				System_printf ("----------------------------------\n");
				System_printf ("Consumer Type11 Socket Properties:\n");
				System_printf ("  Device Identifier:0x%x\n", bindInfo.type11.id);
				System_printf ("  Mailbox          :0x%x\n", bindInfo.type11.mbox);
				System_printf ("  Letter           :0x%x\n", bindInfo.type11.letter);
				System_printf ("----------------------------------\n");
		    }
		}

		/* Ensure that there is sufficient space in the SRIO socket to hold all the packets. */
		if (Srio_setSockOpt (dataSocket, Srio_Opt_PENDING_PKT_COUNT, (void *)&counter, sizeof(counter)) < 0)
		{
			System_printf ("Error: SRIO Socket Set options failed\n");
			return -1;
		}

		/* Set the control socket:  This is used to exchange control messages */
	    controlSocket = (Srio_SockHandle*)dataSocket;

    /* Populate the destination information to be used for the control messages. */
	  controlMsgDest.type11.tt       = TRUE;
		controlMsgDest.type11.id       = srio_device_ID2;
		controlMsgDest.type11.letter   = ((PRODUCER_LTR == SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE) ? PRODUCER_CTRL_LTR : PRODUCER_LTR);
		controlMsgDest.type11.mbox     = ((PRODUCER_MBOX == SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE) ? PRODUCER_CTRL_MBOX : PRODUCER_MBOX);

		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_numPackets);

		/* Set maxIterationCount based on Pacing Delay Search Mode */
		if (!isPacingSearchMode)
			maxIterationCount = thisMaxIterationCount;
		else
			maxIterationCount = 1;

		/* Send READY control message to let TX know RX is ready to rerun the test for this packet size. */
    	if (resetForNewRun)
    	{
			resetForNewRun = FALSE;
			sendControlMessage (controlSocket, CTRL_READY, controlMsgDest, bindInfo.type11.segMap, "RX_1152");
    	}

		/* Get timeout timer start and end time-stamp */
		tmrEnd = tputExampleReadTime() + rxTimeoutLength;

		/* Loop around the number of iterations until we receive all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_numPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Get LLD start time-stamp */
				lldTmrStart = tputExampleReadTime ();

				/* Wait for the data to arrive. */
				if (srio_usePolledMode)
					Srio_rxCompletionIsr (hAppManagedSrioDrv);

				/* Receive data from the socket. */
				num_bytes = Srio_sockRecv (dataSocket, (Srio_DrvBuffer*)&ptrHostDesc, &from);

				/* Get LLD start time-stamp. Also acts as start time-stamp of other processing cycles. */
				lldTmrEnd = tputExampleReadTime ();

				/* if there is no receive packet available check for timeout */
				if (num_bytes <= 0)
				{
					if (tputExampleReadTime() > tmrEnd)
					{
						if (enPacingModeTimeoutSend)
						{
							/* If here then all queued packets should be cleared from the RX queue. */
							sendControlMessage (controlSocket, CTRL_TIMEOUT, controlMsgDest, bindInfo.type11.segMap, "RX_1210");
							enPacingModeTimeoutSend = FALSE;

							/* Start Pacing Delay Search Mode */
							resetForNewRun = TRUE;
							isPacingSearchMode = TRUE;
							break;
						}
						/* Refresh the rx timeout end time-stamp */
						rxTimeoutLength = (uint64_t)2000000000;
						tmrEnd = tmrEnd + rxTimeoutLength;
					}
					continue;
				}

				/* Record LLD cycles */
				tparams.totalLldCycles += (lldTmrEnd - lldTmrStart);

				/* Refresh timeout timer using lldTmrEnd timestamp capture */
				tmrEnd = lldTmrEnd + rxTimeoutLength;

				/* Take test start time-stamp only after the first packet is received. */
				if (packetCount == (uint64_t)0)
					tparams.tsLoopStart = lldTmrStart;

				/* Get the actual descriptor. */
				ptrHostDesc = (Cppi_HostDesc*)QMSS_DESC_PTR(ptrHostDesc);

				/* Get pointers to the valid data that was received. */
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t**)&ptrControlPkt, &tparams.packetSizeBytes);

				/* Verify the packet type. Determine the type of packet which was received? */
				if (ptrControlPkt->pktType != ProdCons_ControlPktType_REQUEST)
				{
					System_printf ("Error: Invalid request received.\n");
					break;
				}

				/* Verify the packet ID. Check if the identifier in the packet is what we expect it to be? */
				if (ptrControlPkt->id != packetID)
				{
					if (rxStatus != (uint32_t)CTRL_FAILURE)
					{
						/* Initiate Pacing Delay Search Mode */
						rxStatus = (uint32_t)CTRL_FAILURE;
						sendControlMessage (controlSocket, rxStatus, controlMsgDest, bindInfo.type11.segMap, "RX_1252");
						debugPrintf_dd ("Error: Detected Drop! (Expected %lu Got %lu)\n", packetID, ptrControlPkt->id);
						enPacingModeTimeoutSend = TRUE;
					}
					/* Set the packet count to the current packet id to cut down on packet ID validation errors */
					packetID = ptrControlPkt->id;
				}

				/* Free queue buffer. Cleanup the received packet. */
				Srio_freeRxDrvBuffer (dataSocket, ptrHostDesc);

				/* Increment the packet counter. */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;

				/* Record other processing cycles */
				tparams.totalProcCycles += (tputExampleReadTime() - lldTmrEnd);
			}

			/* Exit loop if we have dropped packets */
			if (isRXfailure)
				break;

			/* If testing by seconds then calculate the iterations needed to roughly provide the seconds specified */
			if (iterationCount == 0)
			{
				if (testControl.srio_testTimeInSeconds != 0)
				{
					thisMaxIterationCount = getIterationCountUsingSeconds (tparams.tsLoopStart, (uint64_t)tputExampleReadTime(),
							testControl.srio_testTimeInSeconds);
					maxIterationCount = thisMaxIterationCount;
				}
			}

			/* Increment the iteration counter. */
			iterationCount += 1;

			/* Exit loop here if we are in Pacing Delay Search Mode */
			if (isPacingSearchMode)
				break;
		}
		/* Take test end time-stamp */
		tparams.tsLoopEnd = tputExampleReadTime ();

		/* Pacing Delay Search Mode control */
		if (isPacingSearchMode)
		{
			/* Wait for TX to be ready to receive the control message */
			cycleDelay ((uint64_t)0500000000);

			/* Send READY to let the TX side know that the RX side is ready for the next control command
			 * and wait for the control command from the TX side */
			controlMessage = ctrlSendWaitGetResponse (controlSocket, CTRL_READY, &controlMsgDest, bindInfo.type11.segMap, "RX_1338");

			/* Process the command received from the TX side */
			switch (controlMessage)
			{
				case CTRL_RESET:
					/* Re-run test for this packet size */
					isRXfailure = FALSE;
					resetForNewRun = TRUE;
					break;
				case CTRL_NEXT:
					/* Done with this packet size, go to the next packet size */
					isRXfailure = FALSE;
					isPacingSearchMode = FALSE;
					resetForNewRun = FALSE;
					break;
				case CTRL_CONTINUE:
					/* Stop pacing delay search mode and start the real test for this packet size */
					isRXfailure = FALSE;
					isPacingSearchMode = FALSE;
					resetForNewRun = TRUE;
					break;
			}
			controlMessage = 0;
		}
		if (isRXfailure || resetForNewRun)
		{
			isRXfailure = FALSE;
			packetCount = 0;
			packetID = 0;
			iterationCount = 0;
		}
		else
		{
			if (!isPacingSearchMode)
			{
				/* Set Tput parameters then Display the Tput statistics */
				tparams.numberOfPackets = (uint64_t)(iterationCount * srio_numPackets);
				tparams.overheadBytes = (uint32_t)((((tparams.packetSizeBytes - 1) / 256) + 1) * TYPE_11_MESSAGE_OVERHEAD_BYTES_PER_256B);
				displayTputStatistics (&tparams);
				tparams.isShowHeader = FALSE;

				/* Calculate the next packet data size to be used */
				dataSizeBytes = ((dataSizeBytes + SOFTWARE_HEADER_BYTES) * 2) - SOFTWARE_HEADER_BYTES;
			}
			/* Let the TX side know that all packets were received successfully. */
			sendControlMessage (controlSocket, CTRL_SUCCESS, controlMsgDest, bindInfo.type11.segMap, "RX_1404");
			iterationCount = 0;
			packetCount = 0;
			packetID = 0;
		}
		/* Close the data socket */
		Srio_sockClose (dataSocket);

		/* Turn off the initial information headers for the remaining packet sizes. */
		isDisplayInitialInfo = FALSE;
	}
	/* Wait 1 second before outputting text. This delay is to stop the intermixing of RX and TX
	 * data when using a common terminal display as with core to core testing. */
	cycleDelay ((uint64_t)1000000000);

	/* Display collected throughput statistics */
	displayStatsOutputBuffer ();

    if (testControl.srio_isRunProgress)
    {
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO Type11 RX Throughput Measurement Complete\n");
    }

	return 0;
}

/**
 *  @b Description
 *  @n  
 *      This is the producer Type11 throughput.
 *
 *  @retval
 *      Not Applicable.
 */
Int32 producerType11Throughput (void)
{
    Srio_SockHandle         dataSocket;
    Srio_SockHandle*		controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Cppi_HostDesc*          ptrHostDesc;
    Srio_SockAddrInfo       to[MAX_LT_AND_MB_COMBINATIONS], controlMsgDest;
    ProdCons_ControlPkt*    ptrControlPacket;
    uint32_t                rxNumBytes, controlMessage, packetLength, index, heapCounter;
    uint32_t				mailBoxesToCycle, lettersToCycle, combinationsToCycle, mbIndex, ltIndex;
    uint32_t				iterationCount=0, maxIterationCount=0, dataSizeBytes=0, prevDelay=0, toIndex=0;
    uint32_t				thisMaxIterationCount=0, packetID=0, failedAttempts=0;
    uint32_t				procTmrStart, lldTmrStart;
    uint64_t				errEnd, packetCount=0, iterationPacketCount=0;
    bool					isTXfailure = FALSE, isDisplayInitialInfo = TRUE;
    bool					isPacingSearchMode=FALSE, resetForNewRun=FALSE;
    Tput_BinSearch			tput_pacing;
    Tput_Parameters			tparams;

    if (producerType11LatencyTest() < 0)
    	System_printf("Error: Producer latency measurement failed to complete.\n");

	/* Wait a little for the RX side to be ready before we start the TX side */
	cycleDelay ((uint64_t)PRODUCER_START_DELAY_CYCLES);

    /* Clear TX Output Buffer*/
	clearSrioStatsOutputBuffer ();

    /* Calculate mailbox/letter combinations to use */
    mailBoxesToCycle = ((CONSUMER_MBOX == SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE) ? 4 : 1);
    lettersToCycle = ((CONSUMER_LTR == SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE) ? 4 : 1);
    combinationsToCycle = mailBoxesToCycle * lettersToCycle;

	/* Set initial tparams */
	tparams.numberOfPackets = 0;
	tparams.packetSizeBytes = testControl.srio_payloadSize;
	tparams.overheadBytes = 0;
	tparams.pacingCycles = 0;
	tparams.isShowHeader = ((!testControl.srio_isBoardToBoard && testControl.srio_isRunProgress) ? FALSE : TRUE);
	tparams.isTestInfoOnly = TRUE;
	tparams.fType = (uint32_t)Srio_Ftype_MESSAGE;
	tparams.tType = (uint32_t)0;

	/* Display test information */
	displayTputStatistics (&tparams);
	tparams.isTestInfoOnly = FALSE;

    /* initialize pacing search values */
    initPacingSearchValues (&tput_pacing, MAX_PACING_DELAY_CYCLES);

	/* Set the initial data Size. */
	dataSizeBytes = testControl.srio_payloadSize;

	/* Set the initial max iteration count. */
	thisMaxIterationCount = srio_iterationCount;

	/* Packet sizes loop. */
    while (dataSizeBytes <= testControl.srio_payloadEndSize)
    {
    	/* Clear totals for this packet size */
		tparams.totalLldCycles = 0;
		tparams.totalProcCycles = 0;
		tparams.totalIdleCycles = 0;

		/* Create a Type11 NON-BLOCKING Socket. */
		dataSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_RAW_TYPE11, FALSE);
		if (dataSocket == NULL)
		{
			System_printf ("Error: TX Data Socket Open Failed\n");
			return -1;
		}

		/* Populate the binding information. */
		bindInfo.type11.tt       = TRUE;
		bindInfo.type11.id       = srio_device_ID2;
		bindInfo.type11.letter   = PRODUCER_LTR;
		bindInfo.type11.mbox     = PRODUCER_MBOX;
		bindInfo.type11.segMap = ((dataSizeBytes + SOFTWARE_HEADER_BYTES > 256) ? 1 : 0);

		/* Bind the socket. */
		if (Srio_sockBind (dataSocket, &bindInfo) < 0)
		{
			System_printf ("Error: Data Socket Binding Failed.\n");
			return -1;
		}

		if (isDisplayInitialInfo && !isPacingSearchMode)
		{
		    if (testControl.srio_isRunProgress)
		    {
				/* Debug Display Message: */
				System_printf ("----------------------------------\n");
				System_printf ("Producer Type11 Socket Properties:\n");
				System_printf ("  Device Identifier:0x%x\n", bindInfo.type11.id);
				System_printf ("  Mailbox          :0x%x\n", bindInfo.type11.mbox);
				System_printf ("  Letter           :0x%x\n", bindInfo.type11.letter);
				System_printf ("----------------------------------\n");
		    }
		}

    	toIndex = 0;
    	/* Set all the mailbox/letter combinations to use */
    	for (ltIndex=0; ltIndex < mailBoxesToCycle; ltIndex++)
    	{
        	for (mbIndex=0; mbIndex < mailBoxesToCycle; mbIndex++)
			{
				/* Populate the Consumer device where the data is destined to. */
				to[toIndex].type11.tt		= TRUE;
				to[toIndex].type11.id		= srio_device_ID1;
				to[toIndex].type11.letter	= ((CONSUMER_LTR == SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE) ? ltIndex : CONSUMER_LTR);
				to[toIndex].type11.mbox		= ((CONSUMER_MBOX == SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE) ? mbIndex : CONSUMER_MBOX);

				if (isDisplayInitialInfo && !isPacingSearchMode)
				{
					/* Debug Display Message: */
					debugPrintf_ddd  ("Sending to Device Id:0x%x Mailbox:%d Letter:%d\n",
							to[toIndex].type11.id, to[toIndex].type11.mbox, to[toIndex].type11.letter);
				}
				if (toIndex < (MAX_LT_AND_MB_COMBINATIONS - 1))
					toIndex += 1;
			}
    	}

    	/* Default the index to zero */
    	toIndex = 0;
	
	    /* Set the control socket:  This is used to exchange control messages */
	    controlSocket = (Srio_SockHandle*)dataSocket;

	    /* Populate the destination information for the control messages. */
	    controlMsgDest.type11.tt       = TRUE;
		controlMsgDest.type11.id       = srio_device_ID1;
		controlMsgDest.type11.letter   = ((CONSUMER_LTR == SRIO_TYPE11_HW_ASSIGNED_LETTER_VALUE) ? CONSUMER_CTRL_LTR : CONSUMER_LTR);
		controlMsgDest.type11.mbox     = ((CONSUMER_MBOX == SRIO_TYPE11_RX_ACCEPT_ALL_MBOX_VALUE) ? CONSUMER_CTRL_MBOX : CONSUMER_MBOX);

		/* Get the initial heap counter */
		heapCounter = Qmss_getQueueEntryCount (memoryHeapQueue);
	
		/* Initialize all the packets. */
		while (heapCounter != 0)
		{
			/* Allocate a transmit packet */
			ptrHostDesc = allocMemory (sizeof(ProdCons_ControlPkt));
			if (ptrHostDesc == NULL)
			{
				System_printf ("Error: Memory Allocation Failed\n");
				return -1;
			}

			/* Display message if current packet size is equal to the initial packet size */
			if (dataSizeBytes == testControl.srio_payloadSize)
			{
				debugPrintf_p ("Debug: Setting up descriptor 0x%p\n", ptrHostDesc);
			}

			/* Get the data buffer pointer of the allocated packet. */
			Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(uint8_t**)&ptrControlPacket, &rxNumBytes);

			/* Populate this with the Request. */
			ptrControlPacket->pktType   = ProdCons_ControlPktType_REQUEST;
			ptrControlPacket->id        = 0;
			ptrControlPacket->payloadLength = dataSizeBytes;

			/* Set the packets payload data */
			for (index = 0; index < ptrControlPacket->payloadLength; index++)
				ptrControlPacket->pktPayload[index] = index;

			/* Total Packet Length includes the header + payload */
			packetLength = ptrControlPacket->payloadLength + SOFTWARE_HEADER_BYTES;

			/* Sanity Check: The RAW socket created is a Multi-segment packet. So we cannot send
			 * less than 256 bytes. On the other hand if the socket created was Single Segment we
			 * could not have sent a packet greater than 256 bytes. According to the SRIO
			 * specification depending on the Single Segment or Multiple Segment the Mailbox size
			 * can go from 6 bits to 2 bits. */
			if (testControl.srio_payloadSize + SOFTWARE_HEADER_BYTES > 256)
			{
				if (packetLength <= 256)
					packetLength = 260;

				/* Ensure that the number of bytes being transmitted is a multiple of double-word.
				 * This is as per the specification. */
				packetLength = ((packetLength + 7) & ~0x7);
			}

			/* Set the data length & packet length. */
			Cppi_setDataLen (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, packetLength);
			Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)ptrHostDesc, packetLength);

			/* Clean the packet. */
			freeMemory (ptrHostDesc);

			/* Goto the next packet. */
			heapCounter = heapCounter - 1;
		}

		/* Calculate the total number of packets that will be tested */
		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_numPackets);
		if (isPacingSearchMode)
		{
			/* For pacing delay search mode set iteration count to one to make the search quicker */
			maxIterationCount = 1;
		    if (testControl.srio_isRunProgress)
		    {
				System_printf ("Debug: Determining minimum pacing for %d byte packets. Delay %d cycles.\n",
					packetLength, tput_pacing.curr);
		    }
		}
		else
		{
			/* For normal testing make sure that the iteration count is back to what the user specified */
			maxIterationCount = thisMaxIterationCount;
			if (isDisplayInitialInfo)
			{
			    if (testControl.srio_isRunProgress)
			    {
					System_printf ("Debug: Benchmarking %d to %d byte packets. Delay cycles: %d. Test seconds: %d.\n",
							packetLength, (testControl.srio_payloadEndSize + SOFTWARE_HEADER_BYTES),
							tput_pacing.curr, testControl.srio_testTimeInSeconds);
			    }
			}
		}

		/* Take Test start time-stamp */
		tparams.tsLoopStart = tputExampleReadTime ();

		/* Loop around the number of iterations until we send all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_numPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Calculate TX timeout error end time-stamp */
				errEnd = tputExampleReadTime() + (uint64_t)5000000000;

				/* Check for RX error message.
				 * Check for free TX descriptor.
				 * Notify user if TX has stalled. */
				while (1)
				{
					/* check for RX control message. */
					if (recvControlMessage(controlSocket, &controlMessage, CHECK_NON_BLOCKING, "TX_1760 (FAILURE)") == 0)
					{
						if (controlMessage == CTRL_FAILURE)
						{
							isTXfailure = TRUE;
							isPacingSearchMode = TRUE;
						}
						if (isTXfailure)
							break;
					}

					/* Try to get the next TX descriptor. */
					ptrHostDesc = allocMemory (sizeof(ProdCons_ControlPkt));

					/* Exit this loop if the TX descriptor is available */
					if (ptrHostDesc != NULL)
						break;

					/* Let the user know that the transmitter has stalled while sending packets.
					 * No descriptors became available before the timeout period. */
					if (tputExampleReadTime() > errEnd)
					{
						System_printf ("Error: Transmit is unable to send packets. Packet count: %d\n", packetCount);
						displaySrioStatus ();
						errEnd += (uint64_t)5000000000;
						isTXfailure = TRUE;
						if (isTXfailure)
							break;
					}
				}
				/* If delay not equal zero then delay before sending the next packet. */
				if (tput_pacing.curr != 0)
					cycleDelay ((uint64_t)tput_pacing.curr);

				/* Take other processing calls start time-stamp */
				procTmrStart = TSCL;

				/* Exit loop if TX failure */
				if (isTXfailure)
					break;

				/* Get the data buffer pointer of the allocated packet. */
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc,(uint8_t**)&ptrControlPacket, &rxNumBytes);

				/* Set the identifier in the packet */
				ptrControlPacket->id = packetID;

				/* Record LLD cycles */
				tparams.totalProcCycles += (TSCL - procTmrStart);

				/* Set index to use different mailboxes and letters based on packet count to eliminate
				 * the concurrent message limitations that happen when always using the same mailbox
				 * and letter */
				toIndex = (uint32_t)((uint32_t)packetCount % (uint32_t)combinationsToCycle);

				/* Take LLD start time-stamp */
				lldTmrStart = TSCL;

				/* Send the prefabricated packet. */
				if (Srio_sockSend (dataSocket, (Srio_DrvBuffer)ptrHostDesc, SIZE_HOST_DESC, &to[toIndex]) < 0)
				{
					System_printf ("Error: Sending Data Failed.\n");
					return -1;
				}

				/* Record LLD cycles */
				tparams.totalLldCycles += (TSCL - lldTmrStart);

				/* Increment the packet counter */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;
			}

			/* Exit loop here for a TX failure */
			if(isTXfailure)
				break;

			/* If testing by seconds then calculate the iterations needed to roughly provide the seconds specified */
			if (iterationCount == 0)
			{
				if (testControl.srio_testTimeInSeconds != 0)
				{
					thisMaxIterationCount = getIterationCountUsingSeconds (tparams.tsLoopStart, (uint64_t)tputExampleReadTime(),
							testControl.srio_testTimeInSeconds);
					maxIterationCount = thisMaxIterationCount;
				}
			}

			/* Increment the iteration counter */
			iterationCount += 1;

			/* Exit loop here if we are in Pacing Delay Search Mode */
			if (isPacingSearchMode)
				break;
		}
		/* Take test end time-stamp */
		tparams.tsLoopEnd = tputExampleReadTime ();

		/* If is a TX failure then automatically set pacing delay status to not working */
		tput_pacing.isDelayWorking = (isTXfailure ? FALSE : TRUE);

		/* Save the current pacing value to compare later */
		prevDelay = tput_pacing.curr;

		/* Get the next pacing value */
		getPacingBisect (&tput_pacing);

		/* Control next phase of pacing delay search based on current pacing result */
		if ((tput_pacing.curr != prevDelay) || !tput_pacing.isDelayWorking)
		{
			/* App is here because it is still in pacing delay search mode */
			isTXfailure = TRUE;
			if (isPacingSearchMode && !tput_pacing.isDelayWorking)
			{
				/* Wait for a timeout message from RX. This will indicate that all RX and TX queues are empty */
				ctrlWait (controlSocket, CTRL_TIMEOUT, &controlMsgDest, "TX_1892");
			}
		}
		else
		{
			/* App is here if pacing delay search mode is complete or
			 * if no RX failure messages were received for this packet size */
			if (isPacingSearchMode)
			{
				/* Wait to receive ready message from RX side. RX should be in pacing mode and ready to receive new command. */
				ctrlWait (controlSocket, CTRL_READY, &controlMsgDest, "TX_1909");

				if (thisMaxIterationCount == 1)
					/* Tell the RX side to go to the next packet size. When iteration count is only one */
					sendControlMessage (controlSocket, CTRL_NEXT, controlMsgDest, bindInfo.type11.segMap, "TX_1902");
				else
				{
					/* Tell the RX side to continue on with the number of iterations left */
					sendControlMessage (controlSocket, CTRL_CONTINUE, controlMsgDest, bindInfo.type11.segMap,"TX_1905");
					resetForNewRun = TRUE;
				}

				/* Get out of pacing delay search mode if there were no failures */
				if (!isTXfailure)
					isPacingSearchMode = FALSE;
			}
			/* Turn off the initial information headers for the remaining packet sizes. */
			isDisplayInitialInfo = FALSE;
		}

		if (isTXfailure)
		{
			/* Wait to receive ready message from RX side. RX should be in pacing mode once ready is received. */
			ctrlWait (controlSocket, CTRL_READY, &controlMsgDest, "TX_1927");

			/* Tell RX side to start running test in pacing mode. Wait to receive ready from RX side. */
			/* Once ready is received RX should be waiting for TX to start pacing search test.        */
			ctrlSendWait (controlSocket, CTRL_RESET, CTRL_READY, &controlMsgDest, bindInfo.type11.segMap, "TX_1928");

			/* There were failures so automatically set delay is working flag to false */
			tput_pacing.isDelayWorking = FALSE;
		}
		else
		{
			if (resetForNewRun)
				/* Wait for READY message from RX side to indicate no packets were dropped for this packet size */
				ctrlWait (controlSocket, CTRL_READY, &controlMsgDest, "TX_1961");
			else
				/* Wait for SUCCESS message from RX side to indicate no packets were dropped for this packet size */
				ctrlWait (controlSocket, CTRL_SUCCESS, &controlMsgDest, "TX_1965");

			/* There were no failures so automatically set delay is working flag to true */
			tput_pacing.isDelayWorking = TRUE;
		}

		/* Control based on TX failure */
		if (isTXfailure || resetForNewRun)
		{
			/* Set control variables and counts for pacing delay search mode */
			iterationCount = 0;
		    packetCount = 0;
			packetID = 0;
			if (!resetForNewRun)
				failedAttempts += 1;
			resetForNewRun = FALSE;

			/* Exit test if there were too many failures or pacing delay search attempts */
			if (failedAttempts <= MAX_TX_FAIL_ATTEMPTS)
				isTXfailure = FALSE;
			else
			{
		    	/* Close all open data sockets */
				Srio_sockClose (dataSocket);
				System_printf ("Error: TX failed %d times. Stopping. (cr:%d crmn:%d crmx:%d dkg:%d)\n", MAX_TX_FAIL_ATTEMPTS, tput_pacing.curr, tput_pacing.currMin, tput_pacing.currMax, tput_pacing.knownGood);
				while (1);
			}
		}
		else
		{
			/* Display the stats */
			tparams.numberOfPackets = (uint64_t)(iterationCount * srio_numPackets);
			tparams.packetSizeBytes = packetLength;
			tparams.overheadBytes = (uint32_t)((((rxNumBytes - 1) / 256) + 1) * TYPE_11_MESSAGE_OVERHEAD_BYTES_PER_256B);
			tparams.pacingCycles = tput_pacing.curr;
			displayTputStatistics (&tparams);

			/* Reset control variables and counts for next packet size to be tested */
			tparams.isShowHeader = FALSE;
			if (!isPacingSearchMode)
				failedAttempts = 0;
			iterationCount = 0;
			packetCount = 0;
			packetID = 0;
		    initPacingSearchValues (&tput_pacing, MAX_PACING_DELAY_CYCLES);

			/* Calculate next packet size to be tested */
			dataSizeBytes = ((dataSizeBytes + SOFTWARE_HEADER_BYTES) * 2) - SOFTWARE_HEADER_BYTES;
		}
    	/* Close all open data sockets */
		Srio_sockClose (dataSocket);
    }

	/* Wait 1 second before outputting text. This delay is to stop the intermixing of RX and TX
	 * data when using a common terminal display as with core to core testing. */
	cycleDelay ((uint64_t)1500000000);

	/* Display collected throughput statistics */
	displayStatsOutputBuffer ();

    if (testControl.srio_isRunProgress)
    {
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO Type11 TX Throughput Measurement Complete\n");
    }

	return 0;
}

