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
 *   @file  srio_dio_tput.c
 *
 *   @brief   
 *      Direct I/O Consumer (Receive) and Producer (Transmit) benchmarking throughput and latency
 *      functions. Please see benchmarking.c brief for more details.
 *
 */

/* Standard library, XDC and sysbios Include Files. */
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>

/* CSL Chip Functional Layer */
#include <ti/csl/csl_chip.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>

/* Application Include File */
#include <srio_tput.h>

/* Producer-Consumer Include Files. */
#include <benchmarking.h>

/* Time stamp includes */
#include <ti/csl/csl_tsc.h>

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

/* Maximum delay cycles used for throughput pacing search */
#define MAX_DIO_PACING_DELAY_CYCLES	200000

/* DIO defines
 * These are optimize values to keep buffers in L2 memory.
 * Please do not modify these values as it may cause heap allocation issues. */
#define MAX_DIO_NREAD_TX_BUFFERS	4
#define MAX_DIO_TX_BUFFERS			2
#define MAX_DIO_RX_BUFFERS			2
#define DIO_ERROR_PACKET_SIZE_BYTES	4

/* Number of cycles to delay before starting the producer
 * side functions to ensure that the consumer is ready.
 */
#define PRODUCER_START_DELAY_CYCLES	3000000000

/* Defines to enable debug messages for TX/RX control exchange for debugging purposes only */
//#define DEBUG_CONTROL_MESSAGES_TX
//#define DEBUG_CONTROL_MESSAGES_RX

/**********************************************************************
 ************************ External Definitions ************************
 **********************************************************************/

/* Variables to control test run. */
extern struct_testcontrol testControl;

/* Application variables */
extern uint32_t srio_iterationCount;
extern uint64_t srio_numDioPackets;
extern uint64_t srio_latencyNumPackets;
extern bool srio_usePolledMode;
extern uint32_t srio_device_ID1;	/* Consumer */
extern uint32_t srio_device_ID2;	/* Producer */

/* Consumer Management Socket. */
extern Srio_SockHandle     mgmtSocket;

/* Global SRIO Driver Handle. */
extern Srio_DrvHandle      hAppManagedSrioDrv;


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
 *      Utility function which converts a local address to global for the specified core.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *  @param[in]  coreNum
 *      Core number for which to get global address
 *
 *  @retval
 *      Global Address
 */
static uint32_t l2_global_address_for_core (uint32_t addr, uint32_t coreNum)
{
	/* Compute the global address. */
	return (addr + (0x10000000 + (coreNum*0x1000000)));
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
	uint32_t coreNum;

	/* Get the core number. */
	coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

	/* Compute the global address. */
	return (addr + (0x10000000 + (coreNum*0x1000000)));
}

/**
 *  @b Description
 *  @n  
 *      Application registered DIO ISR.
 *
 *  @retval
 *      Not Applicable.
 */
void myDIOIsr(UArg argument)
{
    uint8_t     intDstDoorbell[4];

    /* The Interrupt Destination Decode registers which need to be looked into.
     * Please refer to the SRIO Device Initialization code. */
    intDstDoorbell[0] = 0x0;
    intDstDoorbell[1] = 0x1;
    intDstDoorbell[2] = 0x2;
    intDstDoorbell[3] = 0x3;

    /* Pass the control to the driver DIO ISR handler. */
    Srio_dioCompletionIsr ((Srio_DrvHandle)argument, intDstDoorbell);
    return;
}

/**
 *  @b Description
 *  @n
 *      The function creates the management socket.
 *
 *  @retval
 *      Not Applicable.
 */
Srio_SockHandle createMgmtSocket(void)
{
    Srio_SockHandle         mgmtSocket;
    Srio_SockBindAddrInfo   bindInfo;
    uint32_t                doorbellInfo;
    uint32_t				        coreNum;
    bool					          useBlockingSocket;
    
    /* Set non-blocking or blocking socket designator based on polled mode */
    useBlockingSocket = (srio_usePolledMode) ? FALSE : TRUE;
    
    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

    /* Create the Management Socket. */
    mgmtSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, useBlockingSocket);
    if (mgmtSocket == NULL)
    {
        System_printf ("Error: Unable to open the DIO Management socket\n");
        return NULL;
    }

    /* DIO Binding Information: Use 16/8 bit identifiers and we are bound to the first source id.
     * and we are using 16/8 bit device identifiers. */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (mgmtSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Management Socket failed.\n");
        return NULL;
    }

    if (coreNum == CONSUMER_CORE)
    {
		/* Register the CONSUMER Doorbell Information */
		doorbellInfo = SRIO_SET_DBELL_INFO(CONSUMER_DOORBELL_REG, CONSUMER_DOORBELL_BIT);
    }
    else
    {
		/* Register the PRODUCER Doorbell Information */
		doorbellInfo = SRIO_SET_DBELL_INFO(PRODUCER_DOORBELL_REG, PRODUCER_DOORBELL_BIT);
    }
    if (Srio_setSockOpt (mgmtSocket, Srio_Opt_REGISTER_DOORBELL, (void *)&doorbellInfo, sizeof(uint32_t)) < 0)
    {
        System_printf ("Error: Unable to register the %s Doorbell\n", ((coreNum == CONSUMER_CORE) ? "CONSUMER" : "PRODUCER"));
        return NULL;
    }

    /* Return the management socket. */
    return mgmtSocket;
}

/**
 *  @b Description
 *  @n
 *      Utility Function which sends a PING reply to the PRODUCER
 *      This is used to synchronize the PRODUCER and CONSUMER
 *      applications
 *
 *  @retval
 *      Not Applicable.
 */
static void sendMgmtPingReply(void)
{
    Srio_SockAddrInfo       to;
    uint32_t                doorbellInfo;

#ifdef DEBUG_CONTROL_MESSAGES_RX
    /* Debug Message: */
    System_printf ("Debug: Mgmt Ping Reply to Device Id:0x%x Doorbell Reg 0x%x Doorbell Bit 0x%x\n",
                    srio_device_ID2, PRODUCER_DOORBELL_REG, PRODUCER_DOORBELL_BIT);
#endif

    /* Program the destination information: */
    to.dio.rapidIOMSB    = 0x0;
    to.dio.rapidIOLSB    = 0x0;
    to.dio.dstID         = srio_device_ID2;
    to.dio.ttype         = 0;
    to.dio.ftype         = Srio_Ftype_DOORBELL;

    /* Use the SRIO Driver Macro to program the doorbell information */
    doorbellInfo = SRIO_SET_DBELL_INFO(PRODUCER_DOORBELL_REG, PRODUCER_DOORBELL_BIT);

    /* Send the Doorbell. */
    if (Srio_sockSend (mgmtSocket, (Srio_DrvBuffer)doorbellInfo, 0, &to) < 0)
    {
        System_printf ("Error: Unable to send doorbell to the producer.\n");
        return;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Utility Function which waits for the reply from the CONSUMER
 *
 *  @retval
 *      Not Applicable.
 */
static void waitMgmtPingRequest(void)
{
    Srio_DrvBuffer          doorbellInfo;
#ifdef DEBUG_CONTROL_MESSAGES_RX
    uint8_t                 doorbellReg;
    uint8_t                 doorbellBit;
#endif
    Srio_SockAddrInfo       from;
    uint32_t                num_bytes;
    uint8_t                 intDstDoorbell[4];

    /* The Interrupt Destination Decode registers which need to be looked into.
     * Please refer to the SRIO Device Initialization code. */
    intDstDoorbell[0] = 0x0;
    intDstDoorbell[1] = 0x1;
    intDstDoorbell[2] = 0x2;
    intDstDoorbell[3] = 0x3;

    if (!srio_usePolledMode)
    {
		/* Receive the doorbell */
		if (Srio_sockRecv(mgmtSocket, (Srio_DrvBuffer*)&doorbellInfo, &from) < 0)
		{
			System_printf ("Error: Unable to receive doorbell from the Management socket\n");
			return;
		}
    }
    else
    {
		/* wait for RX message. */
		while (1)
		{
			/* Wait for the data to arrive. */
			if (srio_usePolledMode)
				/* call the DIO ISR handler. */
				Srio_dioCompletionIsr ((Srio_DrvHandle)hAppManagedSrioDrv, intDstDoorbell);
			/* Receive data from the socket. */
			num_bytes = Srio_sockRecv (mgmtSocket, (Srio_DrvBuffer*)&doorbellInfo, &from);
			if (num_bytes > 0)
				break;
		}
    }

#ifdef DEBUG_CONTROL_MESSAGES_RX
    /* Doorbell has been received. */
    doorbellReg = SRIO_GET_DBELL_REG((uint32_t)doorbellInfo);
    doorbellBit = SRIO_GET_DBELL_BIT((uint32_t)doorbellInfo);
    System_printf ("Debug: Received Doorbell Reg %d and Doorbell bit %d\n", doorbellReg, doorbellBit);
#endif

    return;
}

/**
 *  @b Description
 *  @n
 *      Utility Function which sends a PING request to the CONSUMER
 *      This is used to synchronize the PRODUCER and CONSUMER 
 *      applications
 *
 *  @retval
 *      Not Applicable.
 */
static void sendMgmtPingRequest(void)
{
    Srio_SockAddrInfo       to;
    uint32_t                doorbellInfo; 

    /* Program the destination information: */
    to.dio.rapidIOMSB    = 0x0;
    to.dio.rapidIOLSB    = 0x0;
    to.dio.dstID         = srio_device_ID1;
    to.dio.ttype         = 0;
    to.dio.ftype         = Srio_Ftype_DOORBELL;

#ifdef DEBUG_CONTROL_MESSAGES_TX
    /* Debug Message: */
    System_printf ("Debug: Mgmt Ping Request to Device Id:0x%x Doorbell Reg 0x%x Doorbell Bit 0x%x\n",
    		        srio_device_ID1, CONSUMER_DOORBELL_REG, CONSUMER_DOORBELL_BIT);
#endif

    /* Use the SRIO Driver Macro to program the doorbell information */
    doorbellInfo = SRIO_SET_DBELL_INFO(CONSUMER_DOORBELL_REG, CONSUMER_DOORBELL_BIT);

    /* Send the Doorbell. */
    if (Srio_sockSend (mgmtSocket, (Srio_DrvBuffer)doorbellInfo, 0, &to) < 0)
    {
        System_printf ("Error: Unable to send doorbell to the consumer.\n");
        return;
    }
    return;
}

/**
 *  @b Description
 *  @n  
 *      Utility Function which waits for the reply from the CONSUMER
 *
 *  @retval
 *      Not Applicable.
 */
static void waitMgmtPingResponse(void)
{
    Srio_DrvBuffer          doorbellInfo;
#ifdef DEBUG_CONTROL_MESSAGES_TX
    uint8_t                 doorbellReg;
    uint8_t                 doorbellBit;
#endif
    Srio_SockAddrInfo       from;
    uint32_t                num_bytes;
    uint8_t                 intDstDoorbell[4];

    /* The Interrupt Destination Decode registers which need to be looked into.
     * Please refer to the SRIO Device Initialization code. */
    intDstDoorbell[0] = 0x0;
    intDstDoorbell[1] = 0x1;
    intDstDoorbell[2] = 0x2;
    intDstDoorbell[3] = 0x3;

    if (!srio_usePolledMode)
    {
		/* Receive the doorbell */
		if (Srio_sockRecv(mgmtSocket, (Srio_DrvBuffer*)&doorbellInfo, &from) < 0)
		{
			System_printf ("Error: Unable to receive doorbell from the Management socket\n");
			return;
		}
    }
    else
    {
		/* wait for RX message. */
		while (1)
		{
			/* Wait for the data to arrive. */
			if (srio_usePolledMode)
				/* call the DIO ISR handler. */
				Srio_dioCompletionIsr ((Srio_DrvHandle)hAppManagedSrioDrv, intDstDoorbell);
			/* Receive data from the socket. */
			num_bytes = Srio_sockRecv (mgmtSocket, (Srio_DrvBuffer*)&doorbellInfo, &from);
			if (num_bytes > 0)
				break;
		}
    }
    
#ifdef DEBUG_CONTROL_MESSAGES_TX
    /* Doorbell has been received. */
    doorbellReg = SRIO_GET_DBELL_REG((uint32_t)doorbellInfo);
    doorbellBit = SRIO_GET_DBELL_BIT((uint32_t)doorbellInfo);

    if ( (doorbellReg != CONSUMER_DOORBELL_REG) && (doorbellBit != CONSUMER_DOORBELL_BIT) )
    	System_printf ("Debug: Received Doorbell Reg %d and Doorbell bit %d\n", doorbellReg, doorbellBit);
#endif

    return;    
}

/**
 *  @b Description
 *  @n
 *      This is the consumer DIO control function for NREAD latency.
 *      This just sets up the consumer memory to be read by the producer.
 *
 *  @retval
 *      Not Applicable.
 */
Int32 consumerDIONReadLatencyTest(void)
{
#if 1
	Srio_SockHandle         controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
#endif
    uint8_t*                ptrMemory[MAX_DIO_RX_BUFFERS];
    uint32_t                tempIndex=0;
    uint32_t				thisPayloadEndSize;

    /* Return immediately if latency test is not enabled */
    if (!testControl.srio_isLatencyTest)
    	return 0;

    /* Set max payload size */
	thisPayloadEndSize = testControl.srio_dioPayloadEndSize;

#if 1
    /* Open DIO SRIO Socket */
    controlSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, FALSE);
    if (controlSocket == NULL)
    {
        System_printf ("Error: DIO Control Socket open failed\n");
        return -1;
    }

    /* DIO Binding Information: Use 16 bit identifiers and we are bound to the first source id.
     * and we are using 16 bit device identifiers. */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (controlSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Control Socket failed.\n");
        return -1;
    }
#endif

	/* Set receive memory addresses for the largest DIO Transfer */
	for (tempIndex=0; tempIndex<MAX_DIO_RX_BUFFERS; tempIndex++)
	{
		ptrMemory[tempIndex] = (uint8_t*)l2_global_address((uint32_t)((0x880000 - (thisPayloadEndSize * (tempIndex + 1)))));
        /* Initialize the data payload. */
		memset ((void *)ptrMemory[tempIndex], 0xAA, thisPayloadEndSize);
	}

    if (testControl.srio_isRunProgress)
    {
		/* Debug Message: */
		System_printf ("------------------------------------------------\n");
		System_printf ("Consumer DIO Latency, memory is ready for NREAD:\n");
		for (tempIndex=0; tempIndex<MAX_DIO_RX_BUFFERS; tempIndex++)
			System_printf ("  Memory Address    : 0x%p\n", ptrMemory[tempIndex]);
		System_printf ("------------------------------------------------\n");
    }

	/* Wait for the PING request from PRODUCER to signify the end of the test. */
	waitMgmtPingRequest();

#if 1
	/* Close the socket. */
    Srio_sockClose (controlSocket);
#endif

    if (testControl.srio_isRunProgress)
    {
    	/* Control comes here implies that the test is complete. */
    	System_printf ("Debug: SRIO DIO NREAD RX Latency Measurement Complete\n");
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the producer DIO control function for NREAD latency.
 *
 *  @retval
 *      Not Applicable.
 */
Int32 producerDIONReadLatencyTest(void)
{
    Srio_SockHandle         controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    uint8_t*                ptrMemory;
    Srio_DrvBuffer          hSrcBuffer;
    uint32_t				iterationCount=0, maxIterationCount=0, dataSizeBytes=0, lastByteIndex;
    uint32_t                thisMaxIterationCount=0, thisPayloadEndSize;
    uint32_t				toMemAddr;
    uint32_t				lldTmrStart, currentCycles=0;
    uint64_t				packetCount=0, iterationPacketCount=0;
    Tput_Parameters			tparams;
    bool					isDisplayInitialInfo = TRUE;

    /* Return immediately if latency test is not enabled */
    if (!testControl.srio_isLatencyTest)
    	return 0;

	/* Wait a little for the RX side to be ready before we start the TX side */
	cycleDelay ((uint64_t)PRODUCER_START_DELAY_CYCLES);

    /* Clear TX Output Buffer*/
	clearSrioStatsOutputBuffer ();

    /* initialize tparams values */
	tparams.isTestInfoOnly = TRUE;
	tparams.isShowHeader = TRUE;
	tparams.pacingCycles = 0;
	tparams.totalLldCycles = 0;
	tparams.totalProcCycles = 0;
	tparams.totalIdleCycles = 0;
	tparams.minLatCycles = 0xFFFFFFFF;
	tparams.maxLatCycles = 0;
	tparams.fType = (uint32_t)Srio_Ftype_REQUEST;
	tparams.tType = (uint32_t)Srio_Ttype_Request_NREAD;

	/* Display test information */
	displayLatencyStatistics (&tparams, ONE_WAY_TIME);
	tparams.isTestInfoOnly = FALSE;

	/* Set the initial data size. */
	dataSizeBytes = testControl.srio_dioPayloadSize;

	/* Set the end data size. */
	thisPayloadEndSize = testControl.srio_dioPayloadEndSize;

    /* Open DIO SRIO Socket */
    controlSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, FALSE);
    if (controlSocket == NULL)
    {
        System_printf ("Error: DIO Control Socket open failed\n");
        return -1;
    }

    /* DIO Binding Information: Use 16 bit identifiers and we are bound to the first source id.
     * and we are using 16 bit device identifiers. */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (controlSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Control Socket failed.\n");
        return -1;
    }

	/* Set the initial iteration count. */
	thisMaxIterationCount = 1;

    /* Allocate memory blocks for the largest DIO Transfer */
	ptrMemory = (uint8_t*)Memory_alloc(NULL, thisPayloadEndSize, 0, NULL);

    /* Set the source memory address for the largest DIO Transfer */
	toMemAddr = (uint32_t)l2_global_address_for_core((uint32_t)(0x880000 - thisPayloadEndSize), CONSUMER_CORE);

	/* Packet sizes loop. */
    while (dataSizeBytes <= thisPayloadEndSize)
    {
    	/* Set initial variable values for this data size */
        lastByteIndex = dataSizeBytes - 1;

		/* Initialize the data payload. */
		memset ((void *)ptrMemory, 0x00, dataSizeBytes);

		/* Get the handle to the source buffer. */
		hSrcBuffer = (Srio_DrvBuffer)l2_global_address((uint32_t)ptrMemory);

        /* Program the DIO Destination Information.
         *  - The DIO consumer memory block is located at the end of the Local memory. */
        to.dio.rapidIOMSB    = 0x0;
        to.dio.rapidIOLSB    = toMemAddr;
        to.dio.dstID         = srio_device_ID1;
        to.dio.ttype         = Srio_Ttype_Request_NREAD;
        to.dio.ftype         = Srio_Ftype_REQUEST;

		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_latencyNumPackets);

		maxIterationCount = thisMaxIterationCount;

		if (isDisplayInitialInfo)
		{
		    if (testControl.srio_isRunProgress)
		    {
				/* Debug Messages: */
				System_printf ("Debug: Initiating Data Transfer from 0x%p <- 0x%p ...\n",
								hSrcBuffer, to.dio.rapidIOLSB);
				System_printf ("Debug: Measuring latency for %d to %d byte packets. Packet count: %s.\n",
						dataSizeBytes, thisPayloadEndSize, uint64ToString(iterationPacketCount));
		    }
		}

		/* Get starting time-stamp */
		tparams.tsLoopStart = tputExampleReadTime ();

		/* Loop around the number of iterations until we receive all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_latencyNumPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Set the last byte of the packet to 0x00 for verify */
				ptrMemory[lastByteIndex] = 0x00;

				/* Get LLD start time-stamp */
				lldTmrStart = TSCL;

				/* Initiate the transfer. */
				if (Srio_sockSend (controlSocket, hSrcBuffer, dataSizeBytes, &to) < 0)
				{
					System_printf ("Error: Unable to send payload over DIO socket\n");
					return -1;
				}

				/* Check to see if we have received the packet */
				while (1)
				{
					if ((uint8_t)ptrMemory[lastByteIndex] == 0xAA)
						break;
					cycleDelay ((uint64_t)0);
				}

				/* Get the number of cycles taken to send and received the packet */
				currentCycles = (TSCL - lldTmrStart);

				/* Record LLD cycles */
				tparams.totalLldCycles += currentCycles;

				/* set minimum latency cycles */
				if ((uint32_t)currentCycles < (uint32_t)tparams.minLatCycles)
					tparams.minLatCycles = (uint32_t)currentCycles;

				/* set maximum latency cycles */
				if ((uint32_t)currentCycles > (uint32_t)tparams.maxLatCycles)
					tparams.maxLatCycles = (uint32_t)currentCycles;

				/* Increment the packet counter. */
				packetCount += 1;
			}
			/* Increment the iteration counter. */
			iterationCount += 1;
		}
		/* Get ending time-stamp */
		tparams.tsLoopEnd = tputExampleReadTime ();

		/* Set Tput parameters then Display the Tput statistics */
		tparams.numberOfPackets = (uint64_t)(iterationCount * srio_latencyNumPackets);
		tparams.packetSizeBytes = dataSizeBytes;
		tparams.overheadBytes = (uint32_t)((((tparams.packetSizeBytes - 1) / 256) + 1) * DIO_NREAD_OVERHEAD_BYTES_PER_256B);
		tparams.fType = (uint32_t)to.dio.ftype;
		tparams.tType = (uint32_t)to.dio.ttype;
		displayLatencyStatistics (&tparams, ONE_WAY_TIME);
		tparams.isShowHeader = FALSE;

		/* Calculate the next packet data size to be used */
		dataSizeBytes = dataSizeBytes * 2;

		/* Reset variables for the next run */
		iterationCount = 0;
		packetCount = (uint64_t)0;
		tparams.totalLldCycles = 0;
		tparams.totalProcCycles = 0;
		tparams.totalIdleCycles = 0;
		isDisplayInitialInfo = FALSE;
    }

	/* Display collected throughput statistics */
	displayStatsOutputBuffer ();

    /* Notify the RX side that the test is over */
    sendMgmtPingRequest();

    /* Free the memory allocated for the payload. */
	Memory_free(NULL, ptrMemory, thisPayloadEndSize);

    /* Close the socket. */
    Srio_sockClose (controlSocket);

    if (testControl.srio_isRunProgress)
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO DIO NREAD TX Latency Measurement Complete\n");

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the consumer DIO control function for NREAD.
 *      This just sets up the consumer memory to be read by the producer.
 *  @retval
 *      Not Applicable.
 */
Int32 consumerDIONReadThroughput(void)
{
#if 1
	Srio_SockHandle         controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
#endif
    uint8_t*                ptrMemory[MAX_DIO_RX_BUFFERS];
    uint32_t                tempIndex=0;
    uint32_t				thisPayloadEndSize;

    if (consumerDIONReadLatencyTest() < 0)
    	System_printf("Error: Consumer latency measurement failed to complete.\n");

    /* Set max payload size */
	thisPayloadEndSize = testControl.srio_dioPayloadEndSize;

#if 1
	/* Open DIO SRIO Socket */
    controlSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, TRUE);
    if (controlSocket == NULL)
    {
        System_printf ("Error: DIO Control Socket open failed\n");
        return -1;
    }

    /* DIO Binding Information: Use 16 bit identifiers and we are bound to the first source id.
     * and we are using 16 bit device identifiers. */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (controlSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Control Socket failed.\n");
        return -1;
    }
#endif

	/* Set receive memory addresses for the largest DIO Transfer */
	for (tempIndex=0; tempIndex<MAX_DIO_RX_BUFFERS; tempIndex++)
	{
		ptrMemory[tempIndex] = (uint8_t*)l2_global_address((uint32_t)((0x880000 - (thisPayloadEndSize * (tempIndex + 1)))));
        /* Initialize the data payload. */
		memset ((void *)ptrMemory[tempIndex], 0xAA, thisPayloadEndSize);
	}

    if (testControl.srio_isRunProgress)
    {
		/* Debug Message: */
		System_printf ("----------------------------------------\n");
		System_printf ("Consumer DIO, memory is ready for NREAD:\n");
		for (tempIndex=0; tempIndex<MAX_DIO_RX_BUFFERS; tempIndex++)
			System_printf ("  Memory Address    : 0x%p\n", ptrMemory[tempIndex]);
		System_printf ("----------------------------------------\n");
    }

	/* Wait for the PING request from PRODUCER to signify the end of the test. */
	waitMgmtPingRequest();

#if 1
	/* Close the socket. */
    Srio_sockClose (controlSocket);
#endif

    if (testControl.srio_isRunProgress)
    {
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO DIO NREAD RX Throughput Measurement Complete\n");
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the producer DIO NREAD throughput function
 *
 *  @retval
 *      Not Applicable.
 */
Int32 producerDIONReadThroughput(void)
{
    Srio_SockHandle         controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    uint8_t*                ptrMemory[MAX_DIO_NREAD_TX_BUFFERS];
    Srio_DrvBuffer          hSrcBuffer[MAX_DIO_NREAD_TX_BUFFERS];
    uint32_t				iterationCount=0, maxIterationCount=0, dataSizeBytes=0, lastByteIndex;
    uint32_t                thisMaxIterationCount=0, thisPayloadEndSize;
    uint32_t				rxMemIndex=0, txMemIndex=0, tempIndex=0, toMemAddr[MAX_DIO_RX_BUFFERS];
    uint32_t				lldTmrStart, procTmrStart;
    uint64_t				packetCount=0, iterationPacketCount=0;
    uint8_t					packetID=0;
    Tput_BinSearch			tput_pacing;
    Tput_Parameters			tparams;
    bool					isDisplayInitialInfo = TRUE;

    if (producerDIONReadLatencyTest() < 0)
    	System_printf("Error: Producer latency measurement failed to complete.\n");

	/* Wait a little for the RX side to be ready before we start the TX side */
	cycleDelay ((uint64_t)PRODUCER_START_DELAY_CYCLES);

    /* Clear TX Output Buffer*/
	clearSrioStatsOutputBuffer ();

    /* initialize pacing search values */
    initPacingSearchValues (&tput_pacing, MAX_DIO_PACING_DELAY_CYCLES);

    /* initialize tparams values */
	tparams.isShowHeader = TRUE;
	tparams.pacingCycles = 0;
	tparams.totalLldCycles = 0;
	tparams.totalProcCycles = 0;
	tparams.totalIdleCycles = 0;
	tparams.isTestInfoOnly = TRUE;
	tparams.fType = (uint32_t)Srio_Ftype_REQUEST;
	tparams.tType = (uint32_t)Srio_Ttype_Request_NREAD;

	/* Display test information */
	displayTputStatistics (&tparams);
	tparams.isTestInfoOnly = FALSE;

	/* Set the initial data size. */
	dataSizeBytes = testControl.srio_dioPayloadSize;

	/* Set the end data size. */
	thisPayloadEndSize = testControl.srio_dioPayloadEndSize;

    /* Open DIO SRIO Socket */
    controlSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, FALSE);
    if (controlSocket == NULL)
    {
        System_printf ("Error: DIO Control Socket open failed\n");
        return -1;
    }

    /* DIO Binding Information: Use 16 bit identifiers and we are bound to the first source id.
     * and we are using 16 bit device identifiers. */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (controlSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Control Socket failed.\n");
        return -1;
    }

	/* Set the initial iteration count. */
	thisMaxIterationCount = srio_iterationCount;

    /* Allocate memory blocks for the largest DIO Transfer */
	for (tempIndex=0; tempIndex<MAX_DIO_NREAD_TX_BUFFERS; tempIndex++)
	{
		ptrMemory[tempIndex] = (uint8_t*)Memory_alloc(NULL, thisPayloadEndSize, 0, NULL);
	}

    /* Set to memory addresses for the largest DIO Transfer */
	for (tempIndex=0; tempIndex<MAX_DIO_RX_BUFFERS; tempIndex++)
	{
        toMemAddr[tempIndex] = (uint32_t)l2_global_address_for_core((uint32_t)((0x880000 - (thisPayloadEndSize * (tempIndex + 1)))), CONSUMER_CORE);
	}

	/* Packet sizes loop. */
    while (dataSizeBytes <= thisPayloadEndSize)
    {
    	/* Set initial variable values for this data size */
    	txMemIndex = 0;
    	rxMemIndex = 0;
        lastByteIndex = dataSizeBytes - 1;

        /* Initialize the data payload. */
    	for (tempIndex=0; tempIndex<MAX_DIO_NREAD_TX_BUFFERS; tempIndex++)
    	{
            /* Initialize the data payload. */
    		memset ((void *)ptrMemory[tempIndex], 0x00, dataSizeBytes);

            /* Get the handle to the source buffer. */
            hSrcBuffer[tempIndex] = (Srio_DrvBuffer)l2_global_address((uint32_t)ptrMemory[tempIndex]);
    	}

        /* Program the DIO Destination Information.
         *  - The DIO consumer memory block is located at the end of the Local memory. */
        to.dio.rapidIOMSB    = 0x0;
        to.dio.rapidIOLSB    = toMemAddr[rxMemIndex];
        to.dio.dstID         = srio_device_ID1;
        to.dio.ttype         = Srio_Ttype_Request_NREAD;
        to.dio.ftype         = Srio_Ftype_REQUEST;

		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_numDioPackets);

		maxIterationCount = thisMaxIterationCount;

		if (isDisplayInitialInfo)
		{
		    if (testControl.srio_isRunProgress)
		    {
				/* Debug Messages: */
				System_printf ("Debug: Initiating Data Transfer from 0x%p <- 0x%p ...\n",
								hSrcBuffer[txMemIndex], to.dio.rapidIOLSB);
				System_printf ("Debug: Benchmarking %d to %d byte packets. Delay cycles: %d  Test seconds: %d\n",
						dataSizeBytes, thisPayloadEndSize, tput_pacing.curr, testControl.srio_testTimeInSeconds);
		    }
		}

		/* Get starting time-stamp */
		tparams.tsLoopStart = tputExampleReadTime ();

		/* Loop around the number of iterations until we receive all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_numDioPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Get other processing start time-stamp */
				procTmrStart = TSCL;

				/* Set the next memory area to read and to write. */
				rxMemIndex = packetID % MAX_DIO_RX_BUFFERS;
		        to.dio.rapidIOLSB = toMemAddr[rxMemIndex];
				txMemIndex = packetID % MAX_DIO_NREAD_TX_BUFFERS;

				/* Set the last byte of the packet to 0x00 for verify */
				ptrMemory[txMemIndex][lastByteIndex] = 0x00;

				/* Record other processing cycles */
				tparams.totalProcCycles += (TSCL - procTmrStart);

				/* Get LLD start time-stamp */
				lldTmrStart = TSCL;

				/* Initiate the transfer. */
				if (Srio_sockSend (controlSocket, hSrcBuffer[txMemIndex], dataSizeBytes, &to) < 0)
				{
					System_printf ("Error: Unable to send payload over DIO socket\n");
					return -1;
				}

				/* Record LLD cycles */
				tparams.totalLldCycles += (TSCL - lldTmrStart);

				/* Wait until we receive the entire transfer from the RX memory */
				if (packetCount % MAX_DIO_NREAD_TX_BUFFERS == (MAX_DIO_NREAD_TX_BUFFERS - 1))
				{
					while (1)
					{
						/* Check to see if we have received all return packets */
				    	for (tempIndex=0; tempIndex<MAX_DIO_NREAD_TX_BUFFERS; tempIndex++)
				    	{
				    		if ((uint8_t)ptrMemory[tempIndex][lastByteIndex] != 0xAA)
				    			break;
				    	}
				    	if (tempIndex == MAX_DIO_NREAD_TX_BUFFERS)
							break;
						cycleDelay ((uint64_t)0);
					}
				}
				/* Increment the packet counter. */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;
			}

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
		}
		/* Get ending time-stamp */
		tparams.tsLoopEnd = tputExampleReadTime ();

		/* Set Tput parameters then Display the Tput statistics */
		tparams.numberOfPackets = (uint64_t)(iterationCount * srio_numDioPackets);
		tparams.packetSizeBytes = dataSizeBytes;
		tparams.overheadBytes = (uint32_t)((((tparams.packetSizeBytes - 1) / 256) + 1) * DIO_NREAD_OVERHEAD_BYTES_PER_256B);
		tparams.fType = (uint32_t)to.dio.ftype;
		tparams.tType = (uint32_t)to.dio.ttype;
		tparams.pacingCycles = tput_pacing.curr;
		displayTputStatistics (&tparams);
		tparams.isShowHeader = FALSE;

		/* Calculate the next packet data size to be used */
		dataSizeBytes = dataSizeBytes * 2;

		/* Reset variables for the next run */
		iterationCount = 0;
		packetCount = (uint64_t)0;
		packetID = 0;
		tparams.totalLldCycles = 0;
		tparams.totalProcCycles = 0;
		tparams.totalIdleCycles = 0;
		isDisplayInitialInfo = FALSE;
    }

    /* Notify the RX side that the test is over */
    sendMgmtPingRequest();

    /* Free the memory allocated for the payload. */
	for (tempIndex=0; tempIndex<MAX_DIO_NREAD_TX_BUFFERS; tempIndex++)
	{
		Memory_free(NULL, ptrMemory[tempIndex], thisPayloadEndSize);
	}

    /* Close the socket. */
    Srio_sockClose (controlSocket);

	/* Display collected throughput statistics */
	displayStatsOutputBuffer ();

    if (testControl.srio_isRunProgress)
    {
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO DIO NREAD TX Throughput Measurement Complete\n");
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the consumer DIO latency control function.
 *
 *  @retval
 *      Not Applicable.
 */
Int32 consumerDIOLatencyTest(void)
{
    Srio_SockHandle         controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    Srio_DrvBuffer          hSrcBuffer;
    uint8_t*                ptrTxMemory;
    uint8_t*                ptrMemory;
    uint32_t                dataSizeBytes, toMemAddr;
    uint32_t				iterationCount=0, maxIterationCount=0;
    uint32_t				thisMaxIterationCount=0, thisPayloadEndSize, lastByteIndex;
    uint64_t				packetCount=0, iterationPacketCount=0;
    uint8_t					packetID=0, currID=0;

    /* Return immediately if latency test is not enabled */
    if (!testControl.srio_isLatencyTest)
    	return 0;

	/* Set the initial data Size. */
	dataSizeBytes = testControl.srio_dioPayloadSize;

    /* Set max payload size */
	thisPayloadEndSize = testControl.srio_dioPayloadEndSize;

	/* Set the initial iteration count. */
	thisMaxIterationCount = 1;

	/* Setup memory to prefabricate the return packet */
    ptrTxMemory = (uint8_t*)Memory_alloc(NULL, thisPayloadEndSize, 0, NULL);
	if (ptrTxMemory == NULL)
	{
		System_printf ("Error: Unable to allocate memory for producer payload\n");
		return -1;
	}
    memset ((void *)ptrTxMemory, 0xAA, thisPayloadEndSize);
    hSrcBuffer = (Srio_DrvBuffer)l2_global_address((uint32_t)ptrTxMemory);
    
    /* Setup memory address to write the return packet to on the TX side */
    toMemAddr = (uint32_t)l2_global_address_for_core((uint32_t)(0x880000 - thisPayloadEndSize), PRODUCER_CORE);
    to.dio.rapidIOMSB    = 0x0;
    to.dio.rapidIOLSB    = toMemAddr;
    to.dio.dstID         = srio_device_ID2;
    to.dio.ttype         = Srio_Ttype_Write_NWRITE;
    to.dio.ftype         = Srio_Ftype_WRITE;

    /* Open DIO SRIO Socket for sending the return packet */
    controlSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, TRUE);
    if (controlSocket == NULL)
    {
        System_printf ("Error: DIO Control Socket open failed\n");
        return -1;
    }

    /* DIO Binding Information: */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (controlSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Control Socket failed.\n");
        return -1;
    }

    /* Set receive memory addresses for the largest DIO Transfer */
	ptrMemory = (uint8_t*)l2_global_address((uint32_t)(0x880000 - thisPayloadEndSize));

	/* Packet sizes loop. */
    while (dataSizeBytes <= thisPayloadEndSize)
    {
    	/* Initialize control variables for this data size */
        lastByteIndex = dataSizeBytes - 1;

        /* Initialize RX buffer byte values for this data size. */
		ptrMemory[lastByteIndex] = (uint8_t)(packetID - 1);
		ptrMemory[0] = 0xAA;

		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_latencyNumPackets);

		maxIterationCount = thisMaxIterationCount;

		/* Loop around the number of iterations until we process all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_latencyNumPackets);

			/* Set starting return packet ID */
			ptrTxMemory[lastByteIndex] = packetID;

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				currID = ptrMemory[lastByteIndex];
				if (currID != packetID)
					continue;

				/* Send prefabricated return packet to the TX side */
				if (Srio_sockSend (controlSocket, hSrcBuffer, dataSizeBytes, &to) < 0)
				{
					System_printf ("Error: Unable to send RX dropped packet message over DIO socket\n");
					return -1;
				}

				/* Increment the packet counter. */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;

				/* Set return packet ID */
				ptrTxMemory[lastByteIndex] = packetID;
			}

			/* Increment the iteration counter. */
			iterationCount += 1;
		}

		/* Wait a little bit for the TX side to be ready for the reply message */
		cycleDelay ((uint64_t)0500000000);

		/* Calculate the next packet data size to be used */
		dataSizeBytes = dataSizeBytes * 2;

		/* Initialize variables for the next run */
		iterationCount = 0;
		packetCount = (uint64_t)0;
		packetID = 0;

		/* Send the Reply back to the PRODUCER to move on to the next data size. */
		sendMgmtPingReply();
    }

    /* Close the DIO socket. */
    Srio_sockClose (controlSocket);

	/* Free error message memory */
    Memory_free(NULL, ptrTxMemory, thisPayloadEndSize);
	
	/* Wait 1 second before outputing text. This delay is to stop the intermixing of RX and TX
	 * data when using a common terminal display as with core to core testing. */
	cycleDelay ((uint64_t)1000000000);

    if (testControl.srio_isRunProgress)
    {
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO DIO NWRITE RX Latency Measurement Complete\n");
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the producer DIO latency control function.
 *
 *  @retval
 *      Not Applicable.
 */
Int32 producerDIOLatencyTest(void)
{
    Srio_SockHandle         controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    uint8_t*                ptrMemory;
    uint8_t*                ptrLocMemory;
    Srio_DrvBuffer          hSrcBuffer;
    uint32_t				iterationCount=0, maxIterationCount=0, dataSizeBytes=0, lastByteIndex;
    uint32_t                thisMaxIterationCount=0, thisPayloadEndSize;
    uint32_t				toMemAddr;
    uint32_t				lldTmrStart, currentCycles=0;
    uint64_t				packetCount=0, iterationPacketCount=0;
    uint8_t					packetID=0, currID=0;
    Tput_Parameters			tparams;
    bool					isDisplayInitialInfo = TRUE;

    /* Return immediately if latency test is not enabled */
    if (!testControl.srio_isLatencyTest)
    	return 0;

	/* Wait a little for the RX side to be ready before we start the TX side */
	cycleDelay ((uint64_t)PRODUCER_START_DELAY_CYCLES);

    /* Clear TX Output Buffer*/
	clearSrioStatsOutputBuffer ();

    /* initialize tparams values */
	tparams.isTestInfoOnly = TRUE;
	tparams.isShowHeader = TRUE;
	tparams.pacingCycles = 0;
	tparams.fType = (uint32_t)Srio_Ftype_WRITE;
	tparams.tType = (uint32_t)Srio_Ttype_Write_NWRITE;

	/* Display test information */
	displayLatencyStatistics (&tparams, ONE_WAY_TIME);
	tparams.isTestInfoOnly = FALSE;

	/* Set the initial data size. */
	dataSizeBytes = testControl.srio_dioPayloadSize;

	/* Set the end data size. */
	thisPayloadEndSize = testControl.srio_dioPayloadEndSize;

	/* Set the initial iteration count. */
	thisMaxIterationCount = 1;

	/* Get pointer to return packet address */
	ptrLocMemory = (uint8_t*)l2_global_address((uint32_t)(0x880000 - thisPayloadEndSize));

    /* Initialize the data payload. */
	memset ((void *)ptrLocMemory, 0x0, thisPayloadEndSize);

    /* Open DIO SRIO Socket */
    controlSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, FALSE);
    if (controlSocket == NULL)
    {
        System_printf ("Error: DIO Control Socket open failed\n");
        return -1;
    }

    /* DIO Binding Information: Use 16 bit identifiers and we are bound to the first source id.
     * and we are using 16 bit device identifiers. */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (controlSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Control Socket failed.\n");
        return -1;
    }

    /* Allocate source memory block for the largest DIO Transfer */
	ptrMemory = (uint8_t*)Memory_alloc(NULL, thisPayloadEndSize, 0, NULL);

    /* Set destination memory addresses for the largest DIO Transfer */
    toMemAddr = (uint32_t)l2_global_address_for_core((uint32_t)(0x880000 - thisPayloadEndSize), CONSUMER_CORE);

	/* Packet sizes loop. */
    while (dataSizeBytes <= thisPayloadEndSize)
    {
    	/* Set initial variable values for this data size */
        lastByteIndex = dataSizeBytes - 1;

    	/* Clear totals for this packet size */
		tparams.totalLldCycles = 0;
		tparams.totalProcCycles = 0;
		tparams.totalIdleCycles = 0;
		tparams.minLatCycles = 0xFFFFFFFF;
		tparams.maxLatCycles = 0;

        /* Initialize the source data payload. */
		memset ((void *)ptrMemory, 0xAA, dataSizeBytes);

        /* Get the handle to the source buffer. */
        hSrcBuffer = (Srio_DrvBuffer)l2_global_address((uint32_t)ptrMemory);

        /* Program the DIO Destination Information.
         *  - The DIO consumer memory block is located at the end of the Local memory. */
        to.dio.rapidIOMSB    = 0x0;
        to.dio.rapidIOLSB    = toMemAddr;
        to.dio.dstID         = srio_device_ID1;
        to.dio.ttype         = Srio_Ttype_Write_NWRITE;
        to.dio.ftype         = Srio_Ftype_WRITE;

		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_latencyNumPackets);

		maxIterationCount = thisMaxIterationCount;

		if (isDisplayInitialInfo)
		{
		    if (testControl.srio_isRunProgress)
		    {
				/* Debug Messages: */
				System_printf ("Debug: Initiating Data Transfer from 0x%p -> 0x%p ...\n",
								hSrcBuffer, to.dio.rapidIOLSB);
				System_printf ("Debug: Measuring latency for %d to %d byte packets. Packet count: %s.\n",
						dataSizeBytes, thisPayloadEndSize, uint64ToString(iterationPacketCount));
		    }
		}

		/* Get starting time-stamp */
		tparams.tsLoopStart = tputExampleReadTime ();

		/* Loop around the number of iterations until we receive all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_latencyNumPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Set the last byte of the packet to the packetID for the RX side to verify */
				ptrMemory[lastByteIndex] = packetID;
				ptrLocMemory[lastByteIndex] = packetID - 1;

				/* Get LLD start time-stamp */
				lldTmrStart = TSCL;

				/* Initiate the transfer. */
				if (Srio_sockSend (controlSocket, hSrcBuffer, dataSizeBytes, &to) < 0)
				{
					System_printf ("Error: Unable to send payload over DIO socket\n");
					return -1;
				}

				/* Wait until we receive the return packet from the RX side */
				while (1)
				{
					/* Check to see if we have received the return packet */
					currID = ptrLocMemory[lastByteIndex];
					if (currID == packetID)
						break;
			        cycleDelay ((uint64_t)0);
				}

				/* Get the number of cycles taken to send and received the packet */
				currentCycles = (TSCL - lldTmrStart);

				/* Record LLD cycles */
				tparams.totalLldCycles += currentCycles;

				/* set minimum latency cycles */
				if ((uint32_t)currentCycles < (uint32_t)tparams.minLatCycles)
					tparams.minLatCycles = (uint32_t)currentCycles;

				/* set maximum latency cycles */
				if ((uint32_t)currentCycles > (uint32_t)tparams.maxLatCycles)
					tparams.maxLatCycles = (uint32_t)currentCycles;

				/* Increment the packet counter. */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;
			}

			/* Increment the iteration counter. */
			iterationCount += 1;
		}
		/* Get ending time-stamp */
		tparams.tsLoopEnd = tputExampleReadTime ();

		/* Wait for the PING Reply to come back from the CONSUMER to start next packet size. */
		waitMgmtPingResponse();

		/* Set Tput parameters then Display the Tput statistics */
		tparams.numberOfPackets = (uint64_t)(iterationCount * srio_latencyNumPackets);
		tparams.packetSizeBytes = dataSizeBytes;
		tparams.overheadBytes = (uint32_t)((((tparams.packetSizeBytes - 1) / 256) + 1) * DIO_NWRITE_OVERHEAD_BYTES_PER_256B);
		tparams.fType = (uint32_t)to.dio.ftype;
		tparams.tType = (uint32_t)to.dio.ttype;
		displayLatencyStatistics (&tparams, ROUND_TRIP_TIME);
		tparams.isShowHeader = FALSE;

		/* Calculate the next packet data size to be used */
		dataSizeBytes = dataSizeBytes * 2;

		/* Reset variables for the next run */
		iterationCount = 0;
		packetCount = (uint64_t)0;
		packetID = 0;
		isDisplayInitialInfo = FALSE;
    }

	/* Display collected throughput statistics */
	displayStatsOutputBuffer ();

    /* Free the memory allocated for the payload. */
	Memory_free(NULL, ptrMemory, thisPayloadEndSize);

    /* Close the socket. */
    Srio_sockClose (controlSocket);

    if (testControl.srio_isRunProgress)
    {
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO DIO NWRITE TX Latency Measurement Complete\n");
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the consumer DIO NWRITE throughput function.
 *
 *  @retval
 *      Not Applicable.
 */
Int32 consumerDIONWriteThroughput(void)
{
    Srio_SockHandle         controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    Srio_DrvBuffer          hSrcBuffer;
    uint8_t*                ptrTxMemory;
    uint8_t*                ptrMemory[MAX_DIO_RX_BUFFERS];
    uint32_t                dataSizeBytes, rxMemIndex=0, tempIndex=0, toMemAddr;
    uint32_t				iterationCount=0, maxIterationCount=0;
    uint32_t				thisMaxIterationCount=0, thisPayloadEndSize, lastByteIndex;
    uint64_t				packetCount=0, iterationPacketCount=0;
    uint64_t				lldTmrStart;
    uint8_t					packetID=0, currID=0;
    bool					isRXfailure=FALSE, isPacingSearchMode=FALSE;
    bool					isPacingSearchModeStart=FALSE, isDisablePacingSearchMode=FALSE;
    Tput_Parameters			tparams;

    if (consumerDIOLatencyTest() < 0)
    	System_printf("Error: Consumer latency measurement failed to complete.\n");

    /* Clear RX Output Buffer*/
	clearSrioStatsOutputBuffer ();

    /* Set max payload size */
	thisPayloadEndSize = testControl.srio_dioPayloadEndSize;

	/* Setup memory to send message incase of dropped packets */
    ptrTxMemory = (uint8_t*)Memory_alloc(NULL, DIO_ERROR_PACKET_SIZE_BYTES, 0, NULL);
	if (ptrTxMemory == NULL)
	{
		System_printf ("Error: Unable to allocate memory for producer payload\n");
		return -1;
	}
    memset ((void *)ptrTxMemory, 0xDD, DIO_ERROR_PACKET_SIZE_BYTES);
    hSrcBuffer = (Srio_DrvBuffer)l2_global_address((uint32_t)ptrTxMemory);
    toMemAddr = (uint32_t)l2_global_address_for_core((uint32_t)(0x880000 - DIO_ERROR_PACKET_SIZE_BYTES), PRODUCER_CORE);
    to.dio.rapidIOMSB    = 0x0;
    to.dio.rapidIOLSB    = toMemAddr;
    to.dio.dstID         = srio_device_ID2;
    to.dio.ttype         = Srio_Ttype_Write_NWRITE;
    to.dio.ftype         = Srio_Ftype_WRITE;

	/* Set the initial data Size. */
	dataSizeBytes = testControl.srio_dioPayloadSize;

	/* Set the initial iteration count. */
	thisMaxIterationCount = 1;

    /* Open DIO SRIO Socket */
    controlSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, TRUE);
    if (controlSocket == NULL)
    {
        System_printf ("Error: DIO Control Socket open failed\n");
        return -1;
    }

    /* DIO Binding Information: */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (controlSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Control Socket failed.\n");
        return -1;
    }

	/* Initialize the tparams parameters that will be passed to display the test statistics. */
	tparams.numberOfPackets = 0;
	tparams.packetSizeBytes = testControl.srio_dioPayloadSize;
	tparams.overheadBytes = 0;
	tparams.pacingCycles = 0;
	tparams.isShowHeader = TRUE;
	tparams.totalLldCycles = 0;
	tparams.totalProcCycles = 0;
	tparams.totalIdleCycles = 0;
	tparams.isTestInfoOnly = TRUE;
	tparams.fType = (uint32_t)to.dio.ftype;
	tparams.tType = (uint32_t)to.dio.ttype;

	/* Display test information */
	displayTputStatistics (&tparams);
	tparams.isTestInfoOnly = FALSE;

	/* Set the initial iteration count. */
	thisMaxIterationCount = srio_iterationCount;

    /* Set receive memory addresses for the largest DIO Transfer */
	for (tempIndex=0; tempIndex<MAX_DIO_RX_BUFFERS; tempIndex++)
	{
		ptrMemory[tempIndex] = (uint8_t*)l2_global_address((uint32_t)((0x880000 - (thisPayloadEndSize * (tempIndex + 1)))));
	}

	/* Packet sizes loop. */
    while (dataSizeBytes <= thisPayloadEndSize)
    {
    	/* Initialize control variables for this data size */
    	rxMemIndex = 0;
        lastByteIndex = dataSizeBytes - 1;

        /* Initialize RX buffer byte values for this data size. */
    	for (tempIndex=0; tempIndex<MAX_DIO_RX_BUFFERS; tempIndex++)
    	{
			ptrMemory[tempIndex][lastByteIndex] = (uint8_t)(packetID - (MAX_DIO_RX_BUFFERS - tempIndex));
			ptrMemory[tempIndex][0] = 0xAA;
    	}

		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_numDioPackets);

		/* Set maxIterationCount based on Pacing Delay Search Mode */
		if (!isPacingSearchMode)
			maxIterationCount = thisMaxIterationCount;
		else
			maxIterationCount = 1;

		/* Loop around the number of iterations until we receive all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_numDioPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Set the next memory area to check. */
				rxMemIndex = packetID % MAX_DIO_RX_BUFFERS;

				/* Get LLD start time-stamp */
				lldTmrStart = tputExampleReadTime();

				/* Check to see if we have received the next packet */
				currID = (uint8_t)ptrMemory[rxMemIndex][lastByteIndex];
				if (currID != packetID)
				{
					/* Check to see if we have missed a packet */
					if (currID != (uint8_t)(packetID - MAX_DIO_RX_BUFFERS))
					{
						/* Notify the TX side that we have dropped a packet */
						if (Srio_sockSend (controlSocket, hSrcBuffer, DIO_ERROR_PACKET_SIZE_BYTES, &to) < 0)
						{
							System_printf ("Error: Unable to send RX dropped packet message over DIO socket\n");
							return -1;
						}

						/* Wait a little bit to clear remaining LSU queued packets */
						cycleDelay ((uint64_t)0250000000);

						/* Set failure flag and start pacing search mode */
						isRXfailure = TRUE;
						if (!isPacingSearchMode)
							isPacingSearchModeStart = TRUE;
						isPacingSearchMode = TRUE;
					}

					/* Exit this loop on RX failure */
					if (isRXfailure)
						break;

					continue;
				}
				/* Record LLD cycles */
				tparams.totalLldCycles += (tputExampleReadTime() - lldTmrStart);

				/* Take test start time-stamp only after the first packet is received. */
				if (packetCount == (uint64_t)0)
					tparams.tsLoopStart = lldTmrStart;

				/* Increment the packet counter. */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;
			}

			/* Exit this loop on RX failure */
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

			/* Exit this loop if still in pacing search mode */
			if (isPacingSearchMode)
				break;
		}

		/* Take test end time-stamp */
		tparams.tsLoopEnd = tputExampleReadTime ();

		/* Wait a little bit for the TX side to be ready for the reply message */
		cycleDelay ((uint64_t)0500000000);

		/* Check to see if we should still be in pacing search mode */
		if (isPacingSearchMode)
		{
			if (!isPacingSearchModeStart)
			{
				isDisablePacingSearchMode = ((ptrMemory[0][0] == 0xAA) ? TRUE : FALSE);
			}
			
			isPacingSearchModeStart = FALSE;
		}

		/* If not in pacing search mode then this must be the end of the cycle so display statistics. */
		if (!isPacingSearchMode)
		{
			/* Set Tput parameters then Display the Tput statistics */
			tparams.numberOfPackets = (uint64_t)(iterationCount * srio_numDioPackets);
			tparams.packetSizeBytes = dataSizeBytes;
			tparams.overheadBytes = (uint32_t)((((tparams.packetSizeBytes - 1) / 256) + 1) * DIO_NWRITE_OVERHEAD_BYTES_PER_256B);
			displayTputStatistics (&tparams);
			tparams.isShowHeader = FALSE;

			/* Calculate the next packet data size to be used */
			dataSizeBytes = dataSizeBytes * 2;
		}

		/* Initialize variables for the next run */
		iterationCount = 0;
		packetCount = (uint64_t)0;
		packetID = 0;
		tparams.totalLldCycles = 0;
		tparams.totalProcCycles = 0;
		tparams.totalIdleCycles = 0;
		isRXfailure = FALSE;
		if (isDisablePacingSearchMode)
		{
			isPacingSearchMode = FALSE;
			isDisablePacingSearchMode = FALSE;
		}

		/* Send the Reply back to the PRODUCER to move on to the next data size. */
		sendMgmtPingReply();
    }

    /* Close the DIO socket. */
    Srio_sockClose (controlSocket);

	/* Free error message memory */
    Memory_free(NULL, ptrTxMemory, DIO_ERROR_PACKET_SIZE_BYTES);
	
	/* Wait 1 second before outputting text. This delay is to stop the intermixing of RX and TX
	 * data when using a common terminal display as with core to core testing. */
	cycleDelay ((uint64_t)1000000000);

	/* Display collected throughput statistics */
	displayStatsOutputBuffer ();

    if (testControl.srio_isRunProgress)
    {
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO DIO NWRITE RX Throughput Measurement Complete\n");
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the producer DIO NWRITE throughput function
 *
 *  @retval
 *      Not Applicable.
 */
Int32 producerDIONWriteThroughput(void)
{
    Srio_SockHandle         controlSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    uint8_t*                ptrMemory[MAX_DIO_TX_BUFFERS];
    Srio_DrvBuffer          hSrcBuffer[MAX_DIO_TX_BUFFERS];
    uint32_t				iterationCount=0, maxIterationCount=0, dataSizeBytes=0, lastByteIndex;
    uint32_t                thisMaxIterationCount=0, thisPayloadEndSize;
    uint32_t				rxMemIndex=0, txMemIndex=0, tempIndex=0, toMemAddr[MAX_DIO_RX_BUFFERS];
    uint32_t				errMemAddr, prevDelay=0, pacingSearchAttempts=0;
    uint32_t				lldTmrStart, procTmrStart;
    uint64_t				packetCount=0, iterationPacketCount=0;
    uint8_t					packetID=0;
    Tput_BinSearch			tput_pacing;
    Tput_Parameters			tparams;
    bool					isTXfailure=FALSE, isDisplayInitialInfo = TRUE;
    bool					isPacingSearchMode=FALSE, isDisablePacingSearchMode=FALSE;

    if (producerDIOLatencyTest() < 0)
    	System_printf("Error: Producer latency measurement failed to complete.\n");

	/* Wait a little for the RX side to be ready before we start the TX side */
	cycleDelay ((uint64_t)PRODUCER_START_DELAY_CYCLES);

    /* Clear TX Output Buffer*/
	clearSrioStatsOutputBuffer ();

    /* initialize pacing search values */
    initPacingSearchValues (&tput_pacing, MAX_DIO_PACING_DELAY_CYCLES);

    /* initialize tparams values */
	tparams.isShowHeader = ((!testControl.srio_isBoardToBoard && testControl.srio_isRunProgress) ? FALSE : TRUE);
	tparams.pacingCycles = 0;
	tparams.totalLldCycles = 0;
	tparams.totalProcCycles = 0;
	tparams.totalIdleCycles = 0;
	tparams.isTestInfoOnly = TRUE;
	tparams.fType = (uint32_t)Srio_Ftype_WRITE;
	tparams.tType = (uint32_t)Srio_Ttype_Write_NWRITE;

	/* Set the initial data size. */
	dataSizeBytes = testControl.srio_dioPayloadSize;

	/* Set the end data size. */
	thisPayloadEndSize = testControl.srio_dioPayloadEndSize;

	errMemAddr = l2_global_address_for_core((uint32_t)(0x880000 - DIO_ERROR_PACKET_SIZE_BYTES), PRODUCER_CORE);
	*(uint8_t*)errMemAddr = 0x0;

    /* Open DIO SRIO Socket */
    controlSocket = Srio_sockOpen (hAppManagedSrioDrv, Srio_SocketType_DIO, FALSE);
    if (controlSocket == NULL)
    {
        System_printf ("Error: DIO Control Socket open failed\n");
        return -1;
    }

    /* DIO Binding Information: Use 16 bit identifiers and we are bound to the first source id.
     * and we are using 16 bit device identifiers. */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = SRIO_PORT_NUM;
    bindInfo.dio.idSize         = ((testControl.srio_isDeviceID16Bit) ? 1 : 0);
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the DIO socket. */
    if (Srio_sockBind (controlSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the DIO Control Socket failed.\n");
        return -1;
    }

	/* Display test information */
	displayTputStatistics (&tparams);
	tparams.isTestInfoOnly = FALSE;

	/* Set the initial iteration count. */
	thisMaxIterationCount = srio_iterationCount;

    /* Allocate memory blocks for the largest DIO Transfer */
	for (tempIndex=0; tempIndex<MAX_DIO_TX_BUFFERS; tempIndex++)
	{
		ptrMemory[tempIndex] = (uint8_t*)Memory_alloc(NULL, thisPayloadEndSize, 0, NULL);
	}

    /* Set to memory addresses for the largest DIO Transfer */
	for (tempIndex=0; tempIndex<MAX_DIO_RX_BUFFERS; tempIndex++)
	{
        toMemAddr[tempIndex] = (uint32_t)l2_global_address_for_core((uint32_t)((0x880000 - (thisPayloadEndSize * (tempIndex + 1)))), CONSUMER_CORE);
	}

	/* Packet sizes loop. */
    while (dataSizeBytes <= thisPayloadEndSize)
    {
    	/* Set initial variable values for this data size */
    	txMemIndex = 0;
    	rxMemIndex = 0;
        lastByteIndex = dataSizeBytes - 1;

        /* Initialize the data payload. */
    	for (tempIndex=0; tempIndex<MAX_DIO_TX_BUFFERS; tempIndex++)
    	{
            /* Initialize the data payload. */
    		memset ((void *)ptrMemory[tempIndex], 0xAA, dataSizeBytes);

    		/* If in pacing search mode then set the first byte of payload to indicate to the
    		 *  RX side that we are in pacing search mode. */
    		if (isPacingSearchMode)
    			ptrMemory[tempIndex][0] = 0x0;

            /* Get the handle to the source buffer. */
            hSrcBuffer[tempIndex] = (Srio_DrvBuffer)l2_global_address((uint32_t)ptrMemory[tempIndex]);
    	}

        /* Program the DIO Destination Information.
         *  - The DIO consumer memory block is located at the end of the Local memory. */
        to.dio.rapidIOMSB    = 0x0;
        to.dio.rapidIOLSB    = toMemAddr[rxMemIndex];
        to.dio.dstID         = srio_device_ID1;
        to.dio.ttype         = Srio_Ttype_Write_NWRITE;
        to.dio.ftype         = Srio_Ftype_WRITE;

		iterationPacketCount = (uint64_t)(thisMaxIterationCount * srio_numDioPackets);

		/* Set maxIterationCount based on Pacing Delay Search Mode */
		if (!isPacingSearchMode)
		{
			maxIterationCount = thisMaxIterationCount;
			if (isDisplayInitialInfo)
			{
			    if (testControl.srio_isRunProgress)
			    {
					/* Debug Messages: */
					System_printf ("Debug: Initiating Data Transfer from 0x%p -> 0x%p ...\n",
									hSrcBuffer[txMemIndex], to.dio.rapidIOLSB);
					System_printf ("Debug: Benchmarking %d to %d byte packets. Delay cycles: %d. Test seconds: %d.\n",
							dataSizeBytes, thisPayloadEndSize, tput_pacing.curr, testControl.srio_testTimeInSeconds);
			    }
			}
		}
		else
		{
			maxIterationCount = 1;
		    if (testControl.srio_isRunProgress)
		    {
				/* Debug Message: */
				System_printf ("Debug: Determining minimum pacing for %d byte packets. Delay %d cycles.\n",
						dataSizeBytes, tput_pacing.curr);
		    }
		}

		/* Get starting time-stamp */
		tparams.tsLoopStart = tputExampleReadTime ();

		/* Loop around the number of iterations until we receive all the packets. */
		while (iterationCount < maxIterationCount)
		{
			/* Calculate number of packets to count up to for this iteration */
			iterationPacketCount = (uint64_t)((iterationCount + 1) * srio_numDioPackets);

			/* Packet count loop */
			while (packetCount < iterationPacketCount)
			{
				/* Get other processing start time-stamp */
				procTmrStart = TSCL;

				/* Set the next memory area to read and to write. */
				rxMemIndex = packetID % MAX_DIO_RX_BUFFERS;
		        to.dio.rapidIOLSB = toMemAddr[rxMemIndex];
				txMemIndex = packetID % MAX_DIO_TX_BUFFERS;

				/* Set the last byte of the packet to the packetID for the RX side to verify */
				ptrMemory[txMemIndex][lastByteIndex] = packetID;

				/* Record other processing cycles */
				tparams.totalProcCycles += (TSCL - procTmrStart);

				/* Get LLD start time-stamp */
				lldTmrStart = TSCL;

				/* Initiate the transfer. */
				if (Srio_sockSend (controlSocket, hSrcBuffer[txMemIndex], dataSizeBytes, &to) < 0)
				{
					System_printf ("Error: Unable to send payload over DIO socket\n");
					return -1;
				}

				/* Record LLD cycles */
				tparams.totalLldCycles += (TSCL - lldTmrStart);

				/* Delay before sending the next packet if needed. */
				if (tput_pacing.curr != 0)
					cycleDelay ((uint64_t)tput_pacing.curr);

				/* Check for dropped packet message from the RX side. */
				if (*(uint8_t*)(errMemAddr) != 0x0)
				{
					/* Set for pacing search mode */
					isPacingSearchMode = TRUE;
					isTXfailure = TRUE;

					/* Reset dropped packet message */
					*(uint8_t*)(errMemAddr) = 0x0;
				}

				/* Exit this loop on failure */
				if (isTXfailure)
					break;

				/* Increment the packet counter. */
				packetCount += 1;

				/* Increment the packet ID to verify. */
				packetID += 1;
			}

			/* Exit this loop on failure */
			if (isTXfailure)
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

			/* Exit this loop if still in pacing search mode. */
			if (isPacingSearchMode)
				break;
		}
		/* Get ending time-stamp */
		tparams.tsLoopEnd = tputExampleReadTime ();

		/* Do pacing search mode if true */
		if (isPacingSearchMode)
		{
			/* Increment the pacing search attempt counter */
			pacingSearchAttempts += 1;

			/* If is a TX failure then automatically set pacing delay status to not working */
			tput_pacing.isDelayWorking = (isTXfailure ? FALSE : TRUE);

			/* Save the current pacing value to compare later */
			prevDelay = tput_pacing.curr;

			/* Get the next pacing value */
			getPacingBisect (&tput_pacing);

			/* Control next phase of pacing delay search based on current pacing result */
			if ((tput_pacing.curr != prevDelay) || isTXfailure)
			{
				/* Continue on with pacing search mode */
				isPacingSearchMode = TRUE;
				isDisablePacingSearchMode = FALSE;
			}
			else
			{
				/* Set to disable pacing search mode */
				isDisablePacingSearchMode = TRUE;

				/* Set packet's first byte to notify RX side to get out of pacing search mode */
				to.dio.rapidIOLSB = toMemAddr[0];
				ptrMemory[0][0] = 0xAA;

				/* Notify RX side to get out of pacing search mode */
				if (Srio_sockSend (controlSocket, hSrcBuffer[0], DIO_ERROR_PACKET_SIZE_BYTES, &to) < 0)
				{
					System_printf ("Error: Unable to send payload over DIO socket\n");
					return -1;
				}
			}
		}

		/* Wait for the PING Reply to come back from the CONSUMER to start next packet size. */
		waitMgmtPingResponse();

		/* If not in pacing search mode then this must be the end of the cycle so display statistics. */
		if (!isPacingSearchMode)
		{
			/* Set Tput parameters then Display the Tput statistics */
			tparams.numberOfPackets = (uint64_t)(iterationCount * srio_numDioPackets);
			tparams.packetSizeBytes = dataSizeBytes;
			tparams.overheadBytes = (uint32_t)((((tparams.packetSizeBytes - 1) / 256) + 1) * DIO_NWRITE_OVERHEAD_BYTES_PER_256B);
			tparams.fType = (uint32_t)to.dio.ftype;
			tparams.tType = (uint32_t)to.dio.ttype;
			tparams.pacingCycles = tput_pacing.curr;
			displayTputStatistics (&tparams);
			tparams.isShowHeader = FALSE;

			/* Calculate the next packet data size to be used */
			dataSizeBytes = dataSizeBytes * 2;

		    /* initialize pacing search values */
		    initPacingSearchValues (&tput_pacing, MAX_DIO_PACING_DELAY_CYCLES);
		    pacingSearchAttempts = 0;
		}

		/* Reset variables for the next run */
		iterationCount = 0;
		packetCount = (uint64_t)0;
		packetID = 0;
		tparams.totalLldCycles = 0;
		tparams.totalProcCycles = 0;
		tparams.totalIdleCycles = 0;
		isTXfailure = FALSE;

		/* Disable pacing search mode if set */
		if (isDisablePacingSearchMode)
		{
			isPacingSearchMode = FALSE;
			isDisablePacingSearchMode = FALSE;
			pacingSearchAttempts = 0;
		}
		isDisplayInitialInfo = FALSE;

		/* Quit if too many attempts have been made to get the pacing search delay value */
		if (pacingSearchAttempts >= MAX_TX_FAIL_ATTEMPTS)
		{
			System_printf ("Error: TX failed %d times. Stopping. (cr:%d crmn:%d crmx:%d dkg:%d)\n", MAX_TX_FAIL_ATTEMPTS, tput_pacing.curr, tput_pacing.currMin, tput_pacing.currMax, tput_pacing.knownGood);
			break;
		}
    }

    /* Free the memory allocated for the payload. */
	for (txMemIndex=0; txMemIndex<MAX_DIO_TX_BUFFERS; txMemIndex++)
	{
		Memory_free(NULL, ptrMemory[txMemIndex], thisPayloadEndSize);
	}

    /* Close the socket. */
    Srio_sockClose (controlSocket);

	/* Wait 1 second before outputting text. This delay is to stop the intermixing of RX and TX
	 * data when using a common terminal display as with core to core testing. */
	cycleDelay ((uint64_t)1500000000);

	/* Display collected throughput statistics */
	displayStatsOutputBuffer ();

    if (testControl.srio_isRunProgress)
    {
		/* Control comes here implies that the test is complete. */
		System_printf ("Debug: SRIO DIO NWRITE TX Throughput Measurement Complete\n");
    }

    return 0;
}

