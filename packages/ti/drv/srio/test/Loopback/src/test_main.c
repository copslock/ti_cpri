/**
 *   @file  test_main.c
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
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h> 

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>
#include <ti/csl/csl_qm_queue.h>

#include <xdc/cfg/global.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>

/* CPPI/QMSS Include Files. */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

/* CSL Chip Functional Layer */
#include <ti/csl/csl_chip.h>

/* CSL Cache Functional Layer */
#include <ti/csl/csl_cacheAux.h>

/* PSC CSL Include Files */
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>

/* OSAL Include Files. */
#include <srio_osal.h>

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

/* This is the Number of host descriptors which are available & configured
 * in the memory region for this test. */
#define NUM_HOST_DESC               128

/* This is the size of each descriptor. */
#define SIZE_HOST_DESC              64

/* MTU of the SRIO Driver. We are currently operating @ MTU of 256 bytes. */
#define SRIO_MAX_MTU				256

/* This is the size of the data buffer which is used for testing RAW Sockets. */
#define SIZE_RAW_PACKET				152

/* This is the size of the data buffer which is used for testing DIO Sockets. */
#define SIZE_DIO_PACKET				128

/* Defines the core number responsible for system initialization. */
#define CORE_SYS_INIT               0

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Memory allocated for the descriptors. This is 16 bit aligned. */
#pragma DATA_ALIGN (host_region, 64)
Uint8   host_region[NUM_HOST_DESC * SIZE_HOST_DESC];

/* Memory used for the accumulator list. */
#pragma DATA_ALIGN (gHiPriAccumList, 16)
UInt32              gHiPriAccumList[64];

/* Global SRIO and QMSS Configuration */
Qmss_InitCfg    qmssInitConfig;
UInt32          coreNum = 0xFFFF;

/* Shared Memory Variable to ensure synchronizing SRIO initialization
 * with all the other cores. */
/* Created an array to pad the cache line with SRIO_MAX_CACHE_ALIGN size */
#pragma DATA_ALIGN   (isSRIOInitialized, 128)
#pragma DATA_SECTION (isSRIOInitialized, ".srioSharedMem");
volatile Uint32     isSRIOInitialized[(SRIO_MAX_CACHE_ALIGN / sizeof(Uint32))] = { 0 };

Srio_DrvHandle  hAppManagedSrioDrv;
Srio_DrvHandle  hAppManagedSrioDrv1;
Srio_DrvHandle  hAppManagedSrioDrv2;
Srio_DrvHandle  hDrvManagedSrioDrv;

/* These are the device identifiers used used in the TEST Application */
const uint32_t DEVICE_ID1_16BIT    = 0xBEEF;
const uint32_t DEVICE_ID1_8BIT     = 0xAB;
const uint32_t DEVICE_ID2_16BIT    = 0x4560;
const uint32_t DEVICE_ID2_8BIT     = 0xCD;
const uint32_t DEVICE_ID3_16BIT    = 0x1234;
const uint32_t DEVICE_ID3_8BIT     = 0x12;
const uint32_t DEVICE_ID4_16BIT    = 0x5678;
const uint32_t DEVICE_ID4_8BIT     = 0x56;

/**********************************************************************
 ************************* Extern Definitions *************************
 **********************************************************************/

extern UInt32 malloc_counter;
extern UInt32 free_counter;

extern int32_t SrioDevice_init (void);

extern Int32 test_blocking(Srio_DrvHandle hSrioDrv);
extern Int32 test_nonblocking(Srio_DrvHandle hSrioDrv);
extern Int32 test_multicore (Srio_DrvHandle hSrioDrv);
extern Int32 test_sock9 (Srio_DrvHandle hSrioDrv);

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/* OSAL Data Buffer Initialization. */
extern int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize);

/**********************************************************************
 ************************ SRIO TEST FUNCTIONS *************************
 **********************************************************************/

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
UInt32 l2_global_address (Uint32 addr)
{
	UInt32 corenum;

	/* Get the core number. */
	corenum = CSL_chipReadReg(CSL_CHIP_DNUM); 

	/* Compute the global address. */
	return (addr + (0x10000000 + (corenum*0x1000000)));
}

/**
 *  @b Description
 *  @n  
 *      Utility function that is required by the IPC module to set the proc Id.
 *      The proc Id is set via this function instead of hard coding it in the .cfg file
 *
 *  @retval
 *      Not Applicable.
 */
Void myStartupFxn (Void)
{
	MultiProc_setLocalId (CSL_chipReadReg (CSL_CHIP_DNUM));
}

/**
 *  @b Description
 *  @n  
 *      This function enables the power/clock domains for SRIO. 
 *
 *  @retval
 *      Not Applicable.
 */
static Int32 enable_srio (void)
{
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
}


/**
 *  @b Description
 *  @n  
 *      The function tests the functionality of RAW sockets and it also
 *      illustrates the usage of the driver in polled mode.
 *
 *  @param[in]  hSrioDrv
 *      Handle to the SRIO driver 
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static Int32 test_rawSockets (Srio_DrvHandle hSrioDrv)
{
    Srio_SockHandle         srioSocket1;
    Srio_SockHandle         srioSocket2;
    Srio_SockBindAddrInfo   bindInfo;
    Cppi_DescCfg            descCfg;
    Qmss_QueueHnd           srioTempQueue;
    UInt32                  numAllocated;
    Cppi_HostDesc*          ptrHostDesc;
    Cppi_HostDesc*          ptrRxHostDesc;
    UInt8*                  txData;
    UInt8*                  rxData;
    Qmss_Queue              queueInfo;
    Srio_SockAddrInfo       to;
    UInt16                  idx;
    Srio_SockAddrInfo       from;
    Int32                   num_bytes;
    UInt32                  rxNumBytes;
    UInt32                  count;
 
    System_printf ("**********************************************\n");
    System_printf ("******* RAW Socket Testing (Core %d) *********\n", coreNum);
    System_printf ("**********************************************\n");

    /* Open RAW TYPE11 SRIO Socket */
    srioSocket1 =  Srio_sockOpen (hSrioDrv, Srio_SocketType_RAW_TYPE11, FALSE);
    if (srioSocket1 == NULL)
    {
        System_printf ("Error: Unable to open socket1\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;              /* We are using 16 bit identifiers.        */
    bindInfo.type11.id       = DEVICE_ID2_16BIT;  /* Source Identifier bound to the socket   */
    bindInfo.type11.letter   = 0;                 /* Letter Identifier                       */
    bindInfo.type11.mbox     = coreNum;           /* Mailbox Number                          */
    bindInfo.type11.segMap   = 0x0;               /* Single Segment                          */

    /* Bind the SRIO socket 1 */ 
    if (Srio_sockBind (srioSocket1, &bindInfo) < 0)
    {
        System_printf ("Error: socket1 bind failed\n");
        return -1;
    }

    /* Open RAW TYPE11 SRIO Socket */
    srioSocket2 = Srio_sockOpen (hSrioDrv, Srio_SocketType_RAW_TYPE11, FALSE);
    if (srioSocket2 == NULL)
    {
        System_printf ("Error: Unable to open socket2\n");
        return -1;
    }

    /* Populate the binding information. */
    bindInfo.type11.tt       = TRUE;                  /* We are using 16 bit identifiers.         */
    bindInfo.type11.id       = DEVICE_ID1_16BIT;       /* Source Identifier bound to the socket    */
    bindInfo.type11.letter   = 1;                     /* Letter Identifier                        */
    bindInfo.type11.mbox     = coreNum;               /* Mailbox Number                           */
    bindInfo.type11.segMap   = 0x0;                   /* Single Segment                           */

    /* Bind the SRIO Socket 2 */ 
    if (Srio_sockBind (srioSocket2, &bindInfo) < 0)
    {
        System_printf ("Error: socket2 bind failed\n");
        return -1;
    }

    /* Initialize the return queue to NOT specified; this implies that the descriptors
     * will be placed back into the same queue. */
    queueInfo.qMgr = QMSS_PARAM_NOT_SPECIFIED;
    queueInfo.qNum = QMSS_PARAM_NOT_SPECIFIED;

    /* Populate the CPPI descriptor configuration. */
    memset(&descCfg,0,sizeof(Cppi_DescCfg));
    descCfg.memRegion                 = Qmss_MemRegion_MEMORY_REGION0;
    descCfg.descNum                   = 2;
    descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType                  = Cppi_DescType_HOST;
    descCfg.returnQueue               = queueInfo;
    descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.returnPushPolicy          = Qmss_Location_TAIL;
    descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
    descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;

    /* Initialize the descriptors and place all of them into the general purpose temporary queue */
    srioTempQueue  = Cppi_initDescriptor (&descCfg, &numAllocated);
    if (srioTempQueue < 0)
    {
        System_printf ("Error: Initializing transmit descriptor failed :%d \n", srioTempQueue);
        return NULL;
    }

    /* Debug Message: */
    System_printf ("Debug(Core %d): AppConfig Tx Free Queue 0x%x\n", coreNum, srioTempQueue);

    /*************************************************************************************
     * TEST:  
     *  a) Send 10 data packets from socket1 to socket2
     *  b) Make sure data is received on socket2
     *  c) Verify that data was received from the correct source.
     *  d) Validate data payload and length
     *************************************************************************************/
    for (count = 0; count < 10; count++)
    {
	    /* Pop off a descriptor */
	    ptrHostDesc = (Cppi_HostDesc *)Qmss_queuePop(srioTempQueue);
	    if (ptrHostDesc == NULL)
	        return -1;

	    /* Allocate a buffer for sending data: Buffer should be in global address space */
	    txData = (UInt8*)Osal_DataBufferMalloc (SIZE_RAW_PACKET);
	    if (txData == NULL)
	        return -1;

	    /* Populate the data */
	    for (idx = 0; idx < SIZE_RAW_PACKET; idx++)
	        txData[idx] = 0xC0 | count;

	    /* Now we want to send data from the Socket1 to Socket2
	     *  So we populate the destination information where the packet is to be sent. */ 
	    to.type11.tt       = TRUE;                 /* We are using 16 bit identifiers.          */
	    to.type11.id       = DEVICE_ID1_16BIT;     /* Identifier where the packet is to be sent */
	    to.type11.letter   = 1;                    /* Letter Identifier                         */
	    to.type11.mbox     = coreNum;              /* Mailbox Number                            */

	    /* Populate the Transmit Packet Descriptor data with the payload & length */
	    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (UInt8*)txData, SIZE_RAW_PACKET);
	    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (UInt8*)txData, SIZE_RAW_PACKET);

	    /* Configure the packet length */
	    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)ptrHostDesc, SIZE_RAW_PACKET);

	    /* Send out the descriptor on the RAW socket:
         *  Here the number of bytes we are pushing is the size of the descriptor since RAW sockets
         *  directly operate at descriptor levels. */
	    if (Srio_sockSend (srioSocket1, (Srio_DrvBuffer)ptrHostDesc, SIZE_HOST_DESC, &to) < 0)
	    {
	        System_printf ("Error: SRIO Socket Raw send failed\n");
	        return -1;
	    }

        /* Debug Message: */
        System_printf ("Debug(Core %d): Raw Send 0x%p Data Size: %d Iteration %d passed.\n", 
                        coreNum, ptrHostDesc, SIZE_RAW_PACKET, count);

	    /* Wait for the data to arrive. */
	    while (1)
	    {
            /* POLL the SRIO Driver to check if any packets have been received or not? */ 
            Srio_rxCompletionIsr(hSrioDrv);

	        /* Receive data */
	        num_bytes = Srio_sockRecv (srioSocket2, (Srio_DrvBuffer*)&ptrRxHostDesc, &from);
	        if (num_bytes > 0)
	        {
	            /* Make sure that the data was received from socket1 */
				if (from.type11.id != DEVICE_ID2_16BIT)
	            {
	                System_printf ("Error: Data received from invalid source id 0x%x Expected 0x%x\n", 
	                               from.type11.id, DEVICE_ID2_16BIT);
	                return -1;
	            }

	            /* Valid data was received. Check if the length received matches the length transmitted. */
	            if (num_bytes != SIZE_RAW_PACKET)
	            {
	                /* Error: Data Payload Length mismatch... */
	                System_printf ("Error: Invalid data payload received %d bytes\n", num_bytes);
	                return -1;
	            }

	            /* Get the received data. */
	            Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrRxHostDesc, (UInt8**)&rxData,(UInt32*)&rxNumBytes);

                if (rxNumBytes)
                {
    	            /* Validate the received data */
    	            for (idx = 0; idx < rxNumBytes; idx++)
    	            {
    	                if (txData[idx] != rxData[idx])
    	                {
    	                    System_printf ("Error: Data validation (0x%p) failed @ %d Expected 0x%x Got 0x%x\n", 
    	                                   rxData, idx, txData[idx], rxData[idx]);
    	                    return -1;
    	                }
    	            }
                }
                else
                {
                    /* Error: BD did not contain any data */
                    System_printf ("Error: No data was received from source\n");
                    return -1;
                }

                /* Debug Message: */
                System_printf ("Debug(Core %d): Raw Recv Passed for Iteration %d\n", coreNum, count);

	            /* We are done with the receive buffer descriptor and so we need to move it back
	             * to the SRIO Receive Queue. */
	            Srio_freeRxDrvBuffer(srioSocket2, (Srio_DrvBuffer)ptrRxHostDesc);
	            break;
	        }
	    }

        /* Cleanup the memory for the transmitted packet. */
        Osal_DataBufferFree ((Void*)txData, SIZE_RAW_PACKET);
    }

    /* Control comes here implies that data was received and validated */
    System_printf ("Debug(Core %d): RAW Socket Test Passed\n", coreNum);

    /* Close the temporary queue. */
    Qmss_queueClose (srioTempQueue);

    /* Close the sockets. */
    Srio_sockClose (srioSocket1);
    Srio_sockClose (srioSocket2);

    /* Test completed. */
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      The function tests the functionality of DIO sockets
 *
 *  @param[in]  hSrioDrv
 *      Handle to the SRIO driver 
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static Int32 test_dioSockets (Srio_DrvHandle hSrioDrv)
{
    Srio_SockHandle         srioSocket;
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    uint8_t*                srcDataBuffer;
    uint8_t*                dstDataBuffer;
    uint16_t                idx;
    uint16_t                counter;
    uint8_t                 compCode;
 
    System_printf ("**********************************************\n");
    System_printf ("******* DIO Socket Testing (Core %d) *********\n", coreNum);
    System_printf ("**********************************************\n");

    /* Open DIO SRIO Non-Blocking Socket */
    srioSocket = Srio_sockOpen (hSrioDrv, Srio_SocketType_DIO, FALSE);
    if (srioSocket == NULL)
    {
        System_printf ("Error: Unable to open the DIO socket\n");
        return -1;
    }

    /* DIO Binding Information: Use 16 bit identifiers and we are bound to the first source id.
     * and we are using 16 bit device identifiers. */
    bindInfo.dio.doorbellValid  = 0;
    bindInfo.dio.intrRequest    = 0;
    bindInfo.dio.supInt         = 0;
    bindInfo.dio.xambs          = 0;
    bindInfo.dio.priority       = 0;
    bindInfo.dio.outPortID      = 0;
    bindInfo.dio.idSize         = 1;
    bindInfo.dio.srcIDMap       = 0;
    bindInfo.dio.hopCount       = 0;
    bindInfo.dio.doorbellReg    = 0;
    bindInfo.dio.doorbellBit    = 0;

    /* Bind the SRIO socket: DIO sockets do not need any binding information. */ 
    if (Srio_sockBind (srioSocket, &bindInfo) < 0)
    {
        System_printf ("Error: Binding the SIO socket failed.\n");
        return -1;
    }

    /*************************************************************************************
     * TEST 1: 
     *  a) Test the NWRITE_R command
     *  b) Run 10 iterations of the test to ensure multiple data transfers work.
     *************************************************************************************/

    /* Allocate memory for the Source and Destination Buffers. */
    srcDataBuffer =  (uint8_t*)Osal_DataBufferMalloc(SIZE_DIO_PACKET);
    if (srcDataBuffer == NULL)
    {
        System_printf ("Error: Source Buffer Memory Allocation Failed\n");
        return -1;
    }
    dstDataBuffer =  (uint8_t*)Osal_DataBufferMalloc(SIZE_DIO_PACKET);
    if (dstDataBuffer == NULL)
    {
        System_printf ("Error: Destination Buffer Memory Allocation Failed\n");
        return -1;
    }

    /* Debug Message: */
    System_printf ("Debug(Core %d): Starting the DIO Data Transfer Test Src 0x%p Dst 0x%p\n", 
                    coreNum, srcDataBuffer, dstDataBuffer);

    /* We run through 10 iterations. */    
    for (counter = 0; counter < 10; counter++)
    {
        /* Initialize the data buffers: This is done with the correct payloads. */
        for (idx = 0; idx < SIZE_DIO_PACKET; idx++)
        {
            srcDataBuffer[idx] = 0x30 + counter;
            dstDataBuffer[idx] = 0;
        }

        /* Populate the DIO Address Information where the data is to be sent. */
        to.dio.rapidIOMSB    = 0x0;
        to.dio.rapidIOLSB    = (uint32_t)&dstDataBuffer[0];
        to.dio.dstID         = DEVICE_ID2_16BIT;
        to.dio.ttype         = Srio_Ttype_Write_NWRITE_R;
        to.dio.ftype         = Srio_Ftype_WRITE;

        /* Send the DIO Information. */
        if (Srio_sockSend (srioSocket, srcDataBuffer, SIZE_DIO_PACKET, (Srio_SockAddrInfo*)&to) < 0)
        {
            System_printf ("Debug(Core %d): DIO Socket Test Failed\n", coreNum);
            return -1;
        }

        /* Loop around till the transfer is complete. */
        while (1)
        {
            /* Get the completion code. */
            if (Srio_getSockOpt(srioSocket, Srio_Opt_DIO_SOCK_COMP_CODE, &compCode, sizeof(uint8_t)) < 0)
            {
                System_printf ("Error: Unable to get the completion code\n");
                return -1;
            }

            /* Was the transfer complete? */
            if (compCode != 0xFF)
                break;
        }

        /* Was the transfer good. */
        if (compCode != 0)
        {
            System_printf ("Error: SRIO Transfer was not completed Completion Code %d\n", compCode);
            return -1;
        }

        /* Debug Message: Data Transfer was completed successfully. */
        System_printf ("Debug(Core %d): DIO Socket Send for iteration %d Passed\n", coreNum, counter);

        /* Validate the received buffer. */
        for (idx = 0; idx < SIZE_DIO_PACKET; idx++)
        {
            if (dstDataBuffer[idx] != srcDataBuffer[idx])
            {
                System_printf ("Error(Core %d): Data Validation error expected 0x%x got 0x%x @ index %d\n", 
                               coreNum, srcDataBuffer[idx], dstDataBuffer[idx], idx);
                return -1;
            }
        }

        /* Debug Message: Data was validated */
        System_printf ("Debug(Core %d): DIO Transfer Data Validated for iteration %d\n", coreNum, counter);
    }

    /* Cleanup the source & destination buffers. */
    Osal_DataBufferFree ((Void*)srcDataBuffer, SIZE_DIO_PACKET);
    Osal_DataBufferFree ((Void*)dstDataBuffer, SIZE_DIO_PACKET);
    
    /* Debug Message */
    System_printf ("Debug(Core %d): DIO Data Transfer Test Passed\n", coreNum);

    /* Close the sockets. */
    Srio_sockClose (srioSocket);

    /* Test completed. */
    return 0;
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
static Int32 system_init (Void)
{
    Int32               result;
    Qmss_MemRegInfo     memRegInfo;

    /* Initialize the QMSS Configuration block. */
    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));
    
    /* Initialize the Host Region. */
    memset ((void *)&host_region, 0, sizeof(host_region));

    /* Set up the linking RAM. Use the internal Linking RAM. 
     * LLD will configure the internal linking RAM address and maximum internal linking RAM size if 
     * a value of zero is specified. Linking RAM1 is not used */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_HOST_DESC;

#ifdef xdc_target__bigEndian
    /* PDSP Configuration: Big Endian */
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_be);
#else
    /* PDSP Configuration: Little Endian */
    qmssInitConfig.pdspFirmware[0].pdspId   = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size     = sizeof (acc48_le);
#endif

    /* Initialize Queue Manager Sub System */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem error code : %d\n", result);
        return -1;
    }

    /* Start the QMSS. */
    if (Qmss_start() != QMSS_SOK)
    {
        System_printf ("Error: Unable to start the QMSS\n");
        return NULL;
    }

    /* Memory Region 0 Configuration */
    memRegInfo.descBase         = (UInt32 *)l2_global_address((UInt32)host_region);
    memRegInfo.descSize         = SIZE_HOST_DESC;
    memRegInfo.descNum          = NUM_HOST_DESC;
    memRegInfo.manageDescFlag   = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memRegInfo.memRegion        = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;

    /* Initialize and inset the memory region. */
    result = Qmss_insertMemoryRegion (&memRegInfo); 
    if (result < QMSS_SOK)
    {
        System_printf ("Error inserting memory region: %d\n", result);
        return -1;
    }

    /* Initialize CPPI CPDMA */
    result = Cppi_init (&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        System_printf ("Error initializing Queue Manager SubSystem error code : %d\n", result);
        return -1;
    }

    /* CPPI and Queue Manager are initialized. */
    System_printf ("Debug(Core %d): Queue Manager and CPPI are initialized.\n", coreNum);
    System_printf ("Debug(Core %d): Host Region 0x%x\n", coreNum, host_region);
    return 0;
}

/**
 *  @b Description
 *  @n  
 *      Application Raw Receive Cleanup API.
 *
 *  @retval
 *      Not Applicable.
 */
static void myAppRawRxFree(Srio_DrvBuffer hDrvBuffer)
{
    Qmss_QueueHnd       returnQueueHnd;

    /* Get the return queue. */
    returnQueueHnd = Qmss_getQueueHandle(Cppi_getReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)hDrvBuffer));

    /* Push the descriptor into the return queue. */
    Qmss_queuePushDescSize (returnQueueHnd, (Ptr)hDrvBuffer, sizeof(Cppi_HostDesc));
}

/**
 *  @b Description
 *  @n  
 *      This is the main UNIT Test Task which executes all the test cases. 
 *
 *  @retval
 *      Not Applicable.
 */
static Void unitTestTask(UArg arg0, UArg arg1)
{
    Qmss_QueueHnd   myRxFreeQueueHnd;
    Qmss_QueueHnd   myRxCompletionQueueHnd;
    Qmss_QueueHnd   tmpQueueHnd;
    UInt32          numAllocated;
    UInt8           isAllocated;
    Cppi_DescCfg    descCfg;
    UInt16          index;
    Cppi_HostDesc*  ptrHostDesc;
    UInt8*          ptrRxData;
    UInt32          numRxBuffers;
    Srio_DrvConfig  appCfg;
    Srio_DrvConfig  drvCfg;
    Qmss_Queue      queueInfo;

    /* Initialize the SRIO Driver Configuration. */
    memset ((Void *)&appCfg, 0, sizeof(Srio_DrvConfig));
 
    /* Initialize the SRIO Driver Configuration. */
    memset ((Void *)&drvCfg, 0, sizeof(Srio_DrvConfig));

    /* Initialize the OSAL */
    if (Osal_dataBufferInitMemory(SRIO_MAX_MTU) < 0)
	{
	    System_printf ("Error: Unable to initialize the OSAL. \n");
	    return;
    }

    /********************************************************************************
     * The SRIO Driver Instance is going to be created with the following properties:
     * - Application Managed
     * - Receive Completion Queue is Application specified; which implies that there
     *   is no interrupt support. Applications will hence need to poll the queue to
     *   check if there is data available or not.
     * - The Receive Free Descriptor Queues along with the Size thresholds are 
     *   managed and created by the application.
     ********************************************************************************/
    
    /* Create the application receive free queue. */
    myRxFreeQueueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, 
                                       &isAllocated);
    if (myRxFreeQueueHnd < 0)
	{
	    System_printf ("Error: Unable to create application receive queues.\n");
	    return;
    }

    /* Create the application receive completion queue. */
    myRxCompletionQueueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, 
                                             &isAllocated);
    if (myRxCompletionQueueHnd < 0)
	{
	    System_printf ("Error: Unable to create the application receive completion queue.\n");
	    return;
    }

    /* Debug Message: */
    System_printf ("Debug(Core %d): AppConfig RxFreeQueue: 0x%x RxCompletionQueue: 0x%x\n", coreNum, 
                   myRxFreeQueueHnd, myRxCompletionQueueHnd);

    /* We are going to be using 4 receive buffers in this test. */
    numRxBuffers = 4;

    /* Application created queue which stores all the receive buffers. */
    memset(&descCfg,0,sizeof(Cppi_DescCfg));
    descCfg.memRegion                 = Qmss_MemRegion_MEMORY_REGION0;
	descCfg.descNum                   = numRxBuffers;
	descCfg.destQueueNum              = QMSS_PARAM_NOT_SPECIFIED;
	descCfg.queueType                 = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
	descCfg.initDesc                  = Cppi_InitDesc_INIT_DESCRIPTOR;
	descCfg.descType                  = Cppi_DescType_HOST;
	descCfg.returnQueue               = Qmss_getQueueNumber(myRxFreeQueueHnd);
	descCfg.epibPresent               = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.returnPushPolicy          = Qmss_Location_HEAD;
	descCfg.cfg.host.returnPolicy     = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET;
	descCfg.cfg.host.psLocation       = Cppi_PSLoc_PS_IN_DESC;
	tmpQueueHnd = Cppi_initDescriptor (&descCfg, &numAllocated);
	if (tmpQueueHnd < 0)
	{
	    System_printf ("Error: Unable to create application receive queues.\n");
	    return;
    }

    /* Initialize the application receive buffers. */
    for (index = 0; index < descCfg.descNum; index++)
    {
	    /* Pop off a descriptor */
	    ptrHostDesc = (Cppi_HostDesc *)Qmss_queuePop(tmpQueueHnd);
	    if (ptrHostDesc == NULL)
	        return;
	
	    /* Allocate the receive buffer where the data will be received into by the SRIO CPDMA. */
	    ptrRxData = (UInt8*)Osal_DataBufferMalloc(SRIO_MAX_MTU);
	    if (ptrRxData == NULL)
	        return;

        /* Set the DATA and ORIGNAL DATA in the buffer descriptor. */
        Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (UInt8*)ptrRxData, SRIO_MAX_MTU);
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (UInt8*)ptrRxData, SRIO_MAX_MTU);

        /* Add the packet descriptor to the Application Receive Free Queue. */
	    Qmss_queuePushDescSize (myRxFreeQueueHnd, (UInt32*)ptrHostDesc, SIZE_HOST_DESC);
    }

    /* Close the temporary queue. */
    Qmss_queueClose (tmpQueueHnd);

    /* Setup the SRIO Driver Configuration: This is application managed configuration */
    appCfg.bAppManagedConfig = TRUE;

    /* Get the queue information about the receive completion queue. */
    queueInfo = Qmss_getQueueNumber(myRxCompletionQueueHnd);

    /* The application managed configuration is capable of reception. */
    appCfg.u.appManagedCfg.bIsRxFlowCfgValid = 1;

    /* Configure the Receive Flow */
	appCfg.u.appManagedCfg.rxFlowCfg.flowIdNum          = -1;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_qnum       = queueInfo.qNum;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_qmgr       = queueInfo.qMgr;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_sop_offset      = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_ps_location     = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_desc_type       = 0x1; /* Host Descriptor. */
	appCfg.u.appManagedCfg.rxFlowCfg.rx_error_handling  = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_psinfo_present  = 0x1; /* PS Information */
	appCfg.u.appManagedCfg.rxFlowCfg.rx_einfo_present   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo     = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi     = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo      = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi      = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_lo_sel = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_dest_tag_hi_sel = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_lo_sel  = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_src_tag_hi_sel  = 0x0;

    /* Disable Receive size thresholds. */
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0_en = 0x0;
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1_en = 0x0;
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2_en = 0x0;

	/* Use the Application Receive Free Queue for picking all descriptors. */
	queueInfo = Qmss_getQueueNumber(myRxFreeQueueHnd);
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qnum       = queueInfo.qNum;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq1_qmgr       = queueInfo.qMgr;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qnum       = 0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq2_qmgr       = 0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qnum       = 0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq3_qmgr       = 0;

    /* Use the Receive Queue for picking the SOP packet also. */
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qnum   = queueInfo.qNum;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz0_qmgr   = queueInfo.qMgr;

    /* There are no size thresholds configured. */
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh0    = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh1    = 0x0;
    appCfg.u.appManagedCfg.rxFlowCfg.rx_size_thresh2    = 0x0;

    /* The other threshold queues do not need to be configured */
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qnum   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz1_qmgr   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qnum   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz2_qmgr   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qnum   = 0x0;
	appCfg.u.appManagedCfg.rxFlowCfg.rx_fdq0_sz3_qmgr   = 0x0;

    /* Polling Mode: So dont program the accumulator. */
	appCfg.u.appManagedCfg.bIsAccumlatorCfgValid = 0;

    /* Populate the rest of the configuration. */
    appCfg.u.appManagedCfg.rawRxFreeDrvBuffer = myAppRawRxFree;
    appCfg.u.appManagedCfg.txQueueNum = QMSS_PARAM_NOT_SPECIFIED;
 
    /* Set the PKTDMA TX channel priority to low */
    appCfg.u.appManagedCfg.srioPktDmaTxPrio = Srio_PktDma_Prio_Low;

    /* Start the application Managed SRIO Driver. */
    hAppManagedSrioDrv = Srio_start(&appCfg);
    if (hAppManagedSrioDrv == NULL)
    {
        System_printf ("Error(Core %d): Application Managed SRIO Driver-0 failed to start\n", coreNum);
        return;
    }

    /* Start other Application Managed SRIO Drivers which 
     * demonstrates configuration of driver instance to use
     * same TX queue for every instance. */

    /* Specify a valid SRIO queue number to be used for TX */
    appCfg.u.appManagedCfg.txQueueNum = (QMSS_SRIO_QUEUE_BASE + 2);

    hAppManagedSrioDrv1 = Srio_start(&appCfg);
    if (hAppManagedSrioDrv1 == NULL)
    {
        System_printf ("Error(Core %d): Application Managed SRIO Driver-1 failed to start\n", coreNum);
        return;
    }

    hAppManagedSrioDrv2 = Srio_start(&appCfg);
    if (hAppManagedSrioDrv2 == NULL)
    {
        System_printf ("Error(Core %d): Application Managed SRIO Driver-2 failed to start\n", coreNum);
        return;
    }

    /********************************************************************************
     * The SRIO Driver Instance is going to be created with the following properties:
     * - Driver Managed
     * - Interrupt Support (Pass the Rx Completion Queue as NULL)
     ********************************************************************************/
    
    /* Setup the SRIO Driver Managed Configuration. */
    drvCfg.bAppManagedConfig = FALSE;

    /* Driver Managed: Receive Configuration */
    drvCfg.u.drvManagedCfg.bIsRxCfgValid             = 1;
    drvCfg.u.drvManagedCfg.rxCfg.rxMemRegion         = Qmss_MemRegion_MEMORY_REGION0;
    drvCfg.u.drvManagedCfg.rxCfg.numRxBuffers        = 4;
    drvCfg.u.drvManagedCfg.rxCfg.rxMTU               = SRIO_MAX_MTU;
    
    /* Accumulator Configuration. */ 
    {
	    int32_t coreToQueueSelector[4];

        /* This is the table which maps the core to a specific receive queue. */
	    coreToQueueSelector[0] = 704;
	    coreToQueueSelector[1] = 705;
	    coreToQueueSelector[2] = 706;
	    coreToQueueSelector[3] = 707;

	    /* Since we are programming the accumulator we want this queue to be a HIGH PRIORITY Queue */
	    drvCfg.u.drvManagedCfg.rxCfg.rxCompletionQueue = Qmss_queueOpen (Qmss_QueueType_HIGH_PRIORITY_QUEUE, 
	    															     coreToQueueSelector[coreNum], &isAllocated);
		if (drvCfg.u.drvManagedCfg.rxCfg.rxCompletionQueue < 0)
		{
			System_printf ("Error: Unable to open the SRIO Receive Completion Queue\n");
			return;
		}

		/* Accumulator Configuration is VALID. */
		drvCfg.u.drvManagedCfg.rxCfg.bIsAccumlatorCfgValid = 1;	

		/* Accumulator Configuration. */      
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.channel             = coreNum;
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.command             = Qmss_AccCmd_ENABLE_CHANNEL;
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.queueEnMask         = 0;
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.queMgrIndex         = coreToQueueSelector[coreNum];
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.maxPageEntries      = 2;
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.timerLoadCount      = 0;
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.interruptPacingMode = Qmss_AccPacingMode_LAST_INTERRUPT;
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.listEntrySize       = Qmss_AccEntrySize_REG_D;
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.listCountMode       = Qmss_AccCountMode_ENTRY_COUNT;
	    drvCfg.u.drvManagedCfg.rxCfg.accCfg.multiQueueMode      = Qmss_AccQueueMode_SINGLE_QUEUE;

        /* Initialize the accumulator list memory */
        memset ((Void *)&gHiPriAccumList[0], 0, sizeof(gHiPriAccumList));
        drvCfg.u.drvManagedCfg.rxCfg.accCfg.listAddress = l2_global_address((UInt32)&gHiPriAccumList[0]);
    }

    /* Driver Managed: Transmit Configuration */
    drvCfg.u.drvManagedCfg.bIsTxCfgValid             = 1;
    drvCfg.u.drvManagedCfg.txCfg.txMemRegion         = Qmss_MemRegion_MEMORY_REGION0;
    drvCfg.u.drvManagedCfg.txCfg.numTxBuffers        = 4;
    drvCfg.u.drvManagedCfg.txCfg.txMTU               = SRIO_MAX_MTU;

    /* Start the Driver Managed SRIO Driver. */
    hDrvManagedSrioDrv = Srio_start(&drvCfg);
    if (hDrvManagedSrioDrv == NULL)
    {
        System_printf ("Error(Core %d): SRIO Driver failed to start\n", coreNum);
        return;
    }    

    /* Hook up the SRIO interrupts with the core. */
    EventCombiner_dispatchPlug (48, (EventCombiner_FuncPtr)Srio_rxCompletionIsr, (UArg)hDrvManagedSrioDrv, TRUE);
	EventCombiner_enableEvent(48);

    /* Run the loopback data transfer tests on the system initialization core. */
    if (coreNum == CORE_SYS_INIT)
    {
        System_printf ("********************************************************************************\n");
        System_printf ("******* RAW Socket Testing 0 using driver allocated TX queue (Core %d) *********\n", coreNum);
        System_printf ("********************************************************************************\n");
        /* RAW Socket Test: These are executing on the Application Managed SRIO Driver */
        if (test_rawSockets(hAppManagedSrioDrv) < 0)
        {
            System_printf ("Error: RAW Tests failed on App Managed Driver-0\n");
            Task_exit();
        }

        /* RAW Socket Test: These are executing on the Application Managed SRIO Driver-1 */
        System_printf ("********************************************************************\n");
        System_printf ("******* RAW Socket Testing 1 using same TX queue (Core %d) *********\n", coreNum);
        System_printf ("********************************************************************\n");
        if (test_rawSockets(hAppManagedSrioDrv1) < 0)
        {
            System_printf ("Error: RAW Tests failed on App Managed Driver-1\n");
            Task_exit();
        }

        /* RAW Socket Test: These are executing on the Application Managed SRIO Driver-1 */
        System_printf ("********************************************************************\n");
        System_printf ("******* RAW Socket Testing 2 using same TX queue (Core %d) *********\n", coreNum);
        System_printf ("********************************************************************\n");
        if (test_rawSockets(hAppManagedSrioDrv2) < 0)
        {
            System_printf ("Error: RAW Tests failed on App Managed Driver-2\n");
            Task_exit();
        }

        /* Run the non blocking tests. */
        if (test_nonblocking(hDrvManagedSrioDrv) < 0)
        {
            System_printf ("Error: Non-Blocking Tests failed\n");
            Task_exit();
        }

        /* Run the blocking tests */
        if (test_blocking(hDrvManagedSrioDrv) < 0)
        {
            System_printf ("Error: Blocking Tests failed\n");
            Task_exit();
        }

#ifndef SIMULATOR_SUPPORT
        /* DIO is NOT supported on the simulator. */
        if (test_dioSockets (hDrvManagedSrioDrv) < 0)
        {
            System_printf ("Error: DIO Tests failed\n");
            Task_exit();
        }
        /* Type9 is NOT supported on the simulator */
        if (test_sock9 (hDrvManagedSrioDrv) < 0)
        {
            System_printf ("Error: Sock9 Tests failed\n");
            Task_exit();
        }
#endif
    }

#ifdef TEST_MULTICORE
    /* Run the Multicore Test */
    if (test_multicore (hDrvManagedSrioDrv) < 0)
    {
        System_printf ("Error: Multicore Tests failed\n");
        Task_exit();
    }
#else
    System_printf ("NOTE: Multicore Tests have not been enabled; please define the TEST_MULTICORE\n");
    System_printf ("in the Project Build Settings to run the Multicore Test.\n");
#endif

    /* Print out the Malloc & Free Counter */
    System_printf ("Debug(Core %d): Allocation Counter : %d\n", coreNum, malloc_counter);
    System_printf ("Debug(Core %d): Free Counter       : %d\n", coreNum, free_counter);

    /* Check if there is a memory leak? Since we dont implement a 'deinit' API we need to
     * be careful in these calculations
     *  - For the Application Managed Driver Instance 
     *      There will be 'numRxBuffers' + 3 (Driver Instances) 
     *  - For the Driver Managed Driver Instance 
     *      There will be 'numRxBuffers' + 'numTxBuffers' + 1 (Driver Instance)
     *  Take these into account while checking for memory leaks. */
    if ((numRxBuffers + 3) + free_counter +  
        (drvCfg.u.drvManagedCfg.rxCfg.numRxBuffers + drvCfg.u.drvManagedCfg.txCfg.numTxBuffers + 1) != malloc_counter)
    {
        System_printf ("Error: Memory Leak Detected\n");
        Task_exit();
    }

    /* Control comes here implies that all testing passed. */
    System_printf ("Debug(Core %d): Unit Testing completed successfully.\n", coreNum);
    Task_exit();
}

/**
 *  @b Description
 *  @n  
 *      Entry point for the test code.
 *
 *  @retval
 *      Not Applicable.
 */
Void main(Void)
{
    Task_Params     taskParams;

    /* Get the core number. */
	coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

#ifdef SIMULATOR_SUPPORT
    System_printf ("Executing the SRIO Unit Tests on the SIMULATOR\n");
#else
    System_printf ("Executing the SRIO Unit Tests on the DEVICE\n");
#endif

#ifdef TEST_MULTICORE
    /* Initialize the heap in shared memory. Using IPC module to do that */ 
    Ipc_start();
#endif

    /* Initialize the system only if the core was configured to do so. */
    if (coreNum == CORE_SYS_INIT)
    {
        System_printf ("Debug(Core %d): System Initialization for CPPI & QMSS\n", coreNum);

        /* System Initialization */
        if (system_init() < 0)
            return;

        /* Power on SRIO peripheral before using it */
        if (enable_srio () < 0)
        {
            System_printf ("Error: SRIO PSC Initialization Failed\n");
            return;
        }
        
	    /* Device Specific SRIO Initializations: This should always be called before
         * initializing the SRIO Driver. */
    	if (SrioDevice_init() < 0)
        	return;

        /* Initialize the SRIO Driver */
        if (Srio_init () < 0)
        {
            System_printf ("Error: SRIO Driver Initialization Failed\n");
            return;
        }

    	/* SRIO Driver is operational at this time. */
        System_printf ("Debug(Core %d): SRIO Driver has been initialized\n", coreNum);

        /* Write to the SHARED memory location at this point in time. The other cores cannot execute
         * till the SRIO Driver is up and running. */
        isSRIOInitialized[0] = 1;

        /* The SRIO IP block has been initialized. We need to writeback the cache here because it will
         * ensure that the rest of the cores which are waiting for SRIO to be initialized would now be
         * woken up. */
        CACHE_wbL1d ((void *) &isSRIOInitialized[0], 128, CACHE_WAIT);
    }
    else
    {
        /* All other cores need to wait for the SRIO to be initialized before they proceed with the test. */ 
        System_printf ("Debug(Core %d): Waiting for SRIO to be initialized.\n", coreNum);

        /* All other cores loop around forever till the SRIO is up and running. 
         * We need to invalidate the cache so that we always read this from the memory. */
        while (isSRIOInitialized[0] == 0)
            CACHE_invL1d ((void *) &isSRIOInitialized[0], 128, CACHE_WAIT);

        /* Start the QMSS. */
        if (Qmss_start() != QMSS_SOK)
        {
            System_printf ("Error: Unable to start the QMSS\n");
            return;
        }

        System_printf ("Debug(Core %d): SRIO can now be used.\n", coreNum);
    }

    /* Create the UnitTest Task.*/
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4096;
    Task_create(unitTestTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
}

