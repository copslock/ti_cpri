/**
 *   @file  loopbackDioIsr.c
 *
 *   @brief   
 *      This is an example application which shows how DIO transfer
 *      completion interrupts can be registered and serviced.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2010-2012 Texas Instruments, Inc.
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
#include <string.h>
#include <c6x.h>
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
#include <ti/sysbios/family/c66/tci66xx/CpIntc.h>

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>

#include <xdc/cfg/global.h>

/* SRIO Driver Include File. */
#include <ti/drv/srio/srio_drv.h>
#include <ti/drv/srio/srio_osal.h>

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

/* CSL CPINTC Include Files. */
#include <ti/csl/csl_cpIntc.h>

/* Platform Specific Include Files */
#include "srioPlatCfg.h"

#if !defined(CSL_CIC0_SRIO_INTDST0)
#define CSL_CIC0_SRIO_INTDST0   CSL_INTC0_INTDST0   
#endif

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/

/* This is the Number of host descriptors which are available & configured
 * in the memory region for this example. */
#define NUM_HOST_DESC               128

/* This is the size of each descriptor. */
#define SIZE_HOST_DESC              48

/* MTU of the SRIO Driver. We are currently operating @ MTU of 256 bytes. */
#define SRIO_MAX_MTU				256

/* This is the size of the data buffer which is used for DIO Sockets. */
#define SIZE_DIO_PACKET				128

/* Defines the core number responsible for system initialization. */
#define CORE_SYS_INIT               0

/* Defines number of DIO sockets used in this example */
#define SRIO_DIO_LSU_ISR_NUM_SOCKETS        3

/* Defines number of transfers done by DIO sockets in this example */
#define SRIO_DIO_LSU_ISR_NUM_TRANSFERS      3

/* ISR timeout value (in cycles) used in this example */
#define SRIO_DIO_LSU_ISR_TIMEOUT        100000


/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/

/* Memory allocated for the descriptors. This is 16 bit aligned. */
#pragma DATA_ALIGN (host_region, 16)
Uint8   host_region[NUM_HOST_DESC * SIZE_HOST_DESC];

/* Memory used for the accumulator list. */
#pragma DATA_ALIGN (gHiPriAccumList, 16)
UInt32              gHiPriAccumList[64];

/* Global SRIO and QMSS Configuration */
Qmss_InitCfg   qmssInitConfig;

/* Global Varialble which keeps track of the core number executing the
 * application. */
UInt32          coreNum = 0xFFFF;

/* Shared Memory Variable to ensure synchronizing SRIO initialization
 * with all the other cores. */
/* Created an array to pad the cache line with SRIO_MAX_CACHE_ALIGN size */
#pragma DATA_ALIGN   (isSRIOInitialized, 128)
#pragma DATA_SECTION (isSRIOInitialized, ".srioSharedMem");
volatile Uint32     isSRIOInitialized[(SRIO_MAX_CACHE_ALIGN / sizeof(Uint32))] = { 0 };

Srio_DrvHandle  hDrvManagedSrioDrv;

CSL_SrioHandle  hSrioCSL;

/* These are the device identifiers used in the Example Application */
const uint32_t DEVICE_ID1_16BIT    = 0xBEEF;
const uint32_t DEVICE_ID1_8BIT     = 0xAB;
const uint32_t DEVICE_ID2_16BIT    = 0x4560;
const uint32_t DEVICE_ID2_8BIT     = 0xCD;
const uint32_t DEVICE_ID3_16BIT    = 0x1234;
const uint32_t DEVICE_ID3_8BIT     = 0x12;
const uint32_t DEVICE_ID4_16BIT    = 0x5678;
const uint32_t DEVICE_ID4_8BIT     = 0x56;

/* Array containing SRIO socket handles */
Srio_SockHandle srioSocket[SRIO_DIO_LSU_ISR_NUM_SOCKETS];

/* Source and Destination Data Buffers (payload buffers) */
UInt8* srcDataBuffer[SRIO_DIO_LSU_ISR_NUM_SOCKETS * SRIO_DIO_LSU_ISR_NUM_TRANSFERS];
UInt8* dstDataBuffer[SRIO_DIO_LSU_ISR_NUM_SOCKETS * SRIO_DIO_LSU_ISR_NUM_TRANSFERS];

/* Global debug variable to track number of ISRs raised */
volatile UInt32 srioDbgDioIsrCnt = 0;

/* Global variable to indicate completion of ISR processing */
volatile UInt32 srioLsuIsrServiced = 0;

/* Global variables to count good and bad transfers */
volatile UInt32 srioDioLsuGoodTransfers = 0;
volatile UInt32 srioDioLsuBadTransfers  = 0;

/**********************************************************************
 ************************* Extern Definitions *************************
 **********************************************************************/

extern UInt32 malloc_counter;
extern UInt32 free_counter;

extern int32_t SrioDevice_init (void);

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/* OSAL Data Buffer Memory Initialization. */
extern int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize);

/* Enable Time Stamp Counter */
extern void CSL_tscEnable(void);

/**********************************************************************
 ************************ SRIO EXAMPLE FUNCTIONS **********************
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
static UInt32 l2_global_address (Uint32 addr)
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
 *      This function is application registered SRIO DIO LSU interrupt 
 *      handler (ISR) which is used to process the pending DIO Interrupts. 
 *      SRIO Driver users need to ensure that this ISR is plugged with 
 *      their OS Interrupt Management API. The function expects the 
 *      Interrupt Destination information to be passed along to the 
 *      API because the DIO interrupt destination mapping is configurable 
 *      during SRIO device initialization. 
 *
 *  @param[in]  argument
 *      SRIO Driver Handle
 *
 *  @retval
 *      Not Applicable
 */
static void myDioTxCompletionIsr 
(
    UArg argument
)
{
    /* Pass the control to the driver DIO Tx Completion ISR handler */
    Srio_dioTxCompletionIsr ((Srio_DrvHandle)argument, hSrioCSL);

    /* Wake up the pending task */
    srioLsuIsrServiced = 1;

    /* Debug: Increment the ISR count */
    srioDbgDioIsrCnt++;

    return;
}

/**
 *  @b Description
 *  @n  
 *      The function demonstrates usage of interrupts to indicate 
 *      the end of DIO transfers
 *
 *  @param[in]  hSrioDrv
 *      Handle to the SRIO driver 
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static Int32 dioSocketsWithISR (Srio_DrvHandle hSrioDrv, uint8_t dio_ftype, uint8_t dio_ttype)
{
    Srio_SockBindAddrInfo   bindInfo;
    Srio_SockAddrInfo       to;
    uint16_t                sockIdx, i;
    uint16_t                counter, srcDstBufIdx = 0;
    int32_t                 eventId, startTime;
    UInt8                   **srcDataBufPtr = NULL, **dstDataBufPtr = NULL;
    uint8_t                 compCode;

    System_printf ("*************************************************************\n");
    System_printf ("******* DIO Socket Example with Interrupts (Core %d) ********\n", coreNum);
    System_printf ("*************************************************************\n");

    /* Get the CSL SRIO Handle. */
    hSrioCSL = CSL_SRIO_Open (0);
    if (hSrioCSL == NULL)
        return -1;

     /* SRIO DIO Interrupts need to be routed from the CPINTC0 to GEM Event.
     *  - We have configured DIO Interrupts to get routed to Interrupt Destination 0
     *    (Refer to the CSL_SRIO_RouteLSUInterrupts API configuration in the SRIO Initialization)
     *  - We want this System Interrupt to mapped to Host Interrupt 8 */

    /* Disable Interrupt Pacing for INTDST0 */
    CSL_SRIO_DisableInterruptPacing (hSrioCSL, 0);

    /* Route LSU0 ICR0 to INTDST0 */
    CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 0, 0);

    /* Route LSU0 ICR1 to INTDST0 */
    CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 1, 0);

    /* Route LSU0 ICR2 to INTDST0 */
    CSL_SRIO_RouteLSUInterrupts (hSrioCSL, 2, 0);

    /* Map the System Interrupt i.e. the Interrupt Destination 0 interrupt to the DIO ISR Handler. */
    CpIntc_dispatchPlug(CSL_CIC0_SRIO_INTDST0, (CpIntc_FuncPtr)myDioTxCompletionIsr, (UArg)hSrioDrv, TRUE);

    /* The configuration is for CPINTC0. We map system interrupt 112 to Host Interrupt 8. */
    CpIntc_mapSysIntToHostInt(0, CSL_CIC0_SRIO_INTDST0, 8);

    /* Enable the Host Interrupt. */
    CpIntc_enableHostInt(0, 8);

    /* Enable the System Interrupt */
    CpIntc_enableSysInt(0, CSL_CIC0_SRIO_INTDST0);

    /* Get the event id associated with the host interrupt. */
    eventId = CpIntc_getEventId(8);

    /* Plug the CPINTC Dispatcher. */
    EventCombiner_dispatchPlug (eventId, CpIntc_dispatch, 8, TRUE);

    for (sockIdx = 0; sockIdx < SRIO_DIO_LSU_ISR_NUM_SOCKETS; sockIdx++)
    {
      /* Open DIO SRIO Non-Blocking Socket */
      srioSocket[sockIdx] = Srio_sockOpen (hSrioDrv, Srio_SocketType_DIO, FALSE);
      if (srioSocket[sockIdx] == NULL)
      {
          System_printf ("Error: Unable to open the DIO socket - %d\n", sockIdx);
          return -1;
      }

      /* DIO Binding Information: Use 16 bit identifiers and we are bound to the first source id.
       * and we are using 16 bit device identifiers. */
      bindInfo.dio.doorbellValid  = 0;
      bindInfo.dio.intrRequest    = 1;
      bindInfo.dio.supInt         = 0;
      bindInfo.dio.xambs          = 0;
      bindInfo.dio.priority       = 0;
      bindInfo.dio.outPortID      = 0;
      bindInfo.dio.idSize         = 1;
      bindInfo.dio.srcIDMap       = sockIdx;
      bindInfo.dio.hopCount       = 0;
      bindInfo.dio.doorbellReg    = 0;
      bindInfo.dio.doorbellBit    = 0;

      /* Bind the SRIO socket: DIO sockets do not need any binding information. */ 
      if (Srio_sockBind_DIO (srioSocket[sockIdx], &bindInfo) < 0)
      {
          System_printf ("Error: Binding the SIO socket failed.\n");
          return -1;
      }
    }

    /* Allocate memory for the Source and Destination Buffers */
    for (i = 0; i < (SRIO_DIO_LSU_ISR_NUM_SOCKETS * SRIO_DIO_LSU_ISR_NUM_TRANSFERS); i++)
    {
        srcDataBuffer[i] =  (uint8_t*)Osal_srioDataBufferMalloc(SIZE_DIO_PACKET);
        if (srcDataBuffer[i] == NULL)
        {
            System_printf ("Error: Source Buffer (%d) Memory Allocation Failed\n", i);
            return -1;
        }
        dstDataBuffer[i] =  (uint8_t*)Osal_srioDataBufferMalloc(SIZE_DIO_PACKET);
        if (dstDataBuffer[i] == NULL)
        {
            System_printf ("Error: Destination Buffer (%d) Memory Allocation Failed\n", i);
            return -1;
        }

        /* Initialize the data buffers */
        for (counter = 0; counter < SIZE_DIO_PACKET; counter++)
        {
            srcDataBuffer[i][counter] = 0x30 + (i * 5);
            dstDataBuffer[i][counter] = 0;
        }
        /* Debug Message: */
        System_printf ("Debug(Core %d): Starting the DIO Data Transfer - Src(%d) 0x%p Dst(%d) 0x%p\n", 
                        coreNum, i, srcDataBuffer[i], i, dstDataBuffer[i]);
    }
    
    if ((dio_ftype == Srio_Ftype_WRITE) && (dio_ttype == Srio_Ttype_Write_NWRITE_R))
    {
        /* DIO Write operation */
        srcDataBufPtr = &srcDataBuffer[0];
        dstDataBufPtr = &dstDataBuffer[0];
    }
    else if ((dio_ftype == Srio_Ftype_REQUEST) && (dio_ttype == Srio_Ttype_Request_NREAD))
    {
        /* DIO Read operation */
        srcDataBufPtr = &dstDataBuffer[0];
        dstDataBufPtr = &srcDataBuffer[0];
    }
    else
    {
        /* Debug Message: */
        System_printf ("Debug(Core %d): Unexpected combination of FTYPE and TTYPE. Example couldn't run. Exiting!!!\n", 
                        coreNum);
        return -1;
    }

    /*********************************************************************************
     * Run multiple iterations of the example to ensure multiple data transfers work
     *********************************************************************************/
    for (counter = 0; counter < SRIO_DIO_LSU_ISR_NUM_TRANSFERS; counter++)
    {
        for (sockIdx = 0; sockIdx < SRIO_DIO_LSU_ISR_NUM_SOCKETS; sockIdx++)
        {
            /* Populate the DIO Address Information where the data is to be sent. */
            to.dio.rapidIOMSB    = 0x0;
            to.dio.rapidIOLSB    = (uint32_t)&dstDataBufPtr[srcDstBufIdx][0];
            to.dio.dstID         = DEVICE_ID2_16BIT;
            to.dio.ttype         = dio_ttype;
            to.dio.ftype         = dio_ftype;

            /* Send the DIO Information. */
            if (Srio_sockSend_DIO (srioSocket[sockIdx], srcDataBufPtr[srcDstBufIdx], SIZE_DIO_PACKET, (Srio_SockAddrInfo*)&to) < 0)
            {
                System_printf ("Debug(Core %d): DIO Socket Example Failed\n", coreNum);
                return -1;
            }

            /* Wait for the interrupt to occur without touching the peripheral. */
            /* Other useful work could be done here such as by invoking a scheduler */
            startTime = TSCL;
            while((! srioLsuIsrServiced) && ((TSCL - startTime) < SRIO_DIO_LSU_ISR_TIMEOUT));

            if (! srioLsuIsrServiced) {
              System_printf ("ISR didn't happen within set time - %d cycles. Example failed !!!\n", SRIO_DIO_LSU_ISR_TIMEOUT);
              return -1;
            }

            /* Debug Message: Data Transfer was completed successfully. */
            System_printf ("Debug(Core %d): DIO Socket (%d) Send for iteration %d\n", coreNum, sockIdx, counter);

            /* Read the completion code filled by the ISR */
            compCode = 0xFF;
            if (Srio_getSockOpt(srioSocket[sockIdx], Srio_Opt_DIO_READ_SOCK_COMP_CODE, &compCode, sizeof(uint8_t)) < 0)
            {
                System_printf ("Error: Unable to read the completion code in socket\n");
                return -1;
            }
            /* Was the transfer good. */
            if (compCode == 0)
            {
                srioDioLsuGoodTransfers++;
            }
            else
            {
                srioDioLsuBadTransfers++;
            }

            /* Clear the LSU pending interrupt (ICCx) */
            CSL_SRIO_ClearLSUPendingInterrupt (hSrioCSL, 0xFFFFFFFF, 0xFFFFFFFF);
            srioLsuIsrServiced = 0;
            /* Debug Message: Display ISR count */
            System_printf ("Debug(Core %d): ISR Count: %d\n", coreNum, srioDbgDioIsrCnt);

            /* Load next set of payload buffers */
            srcDstBufIdx++;
            if (srcDstBufIdx > (SRIO_DIO_LSU_ISR_NUM_SOCKETS * SRIO_DIO_LSU_ISR_NUM_TRANSFERS))
            {
                System_printf ("Debug(Core %d): DIO Socket Example --- Out of SRC and DST buffers\n", coreNum);
                return -1;
            }
        }
    }

    /* Validate the received buffer. */
    for (i = 0; i < (SRIO_DIO_LSU_ISR_NUM_SOCKETS * SRIO_DIO_LSU_ISR_NUM_TRANSFERS); i++)
    {
        for (counter = 0; counter < SIZE_DIO_PACKET; counter++)
        {
            if (dstDataBufPtr[i][counter] != srcDataBufPtr[i][counter])
            {
                System_printf ("Error(Core %d): Data Validation error. Buffer Number (%d): Expected 0x%x got 0x%x @ index %d\n", 
                                coreNum, i, srcDataBufPtr[i][counter], dstDataBufPtr[i][counter], counter);
                return -1;
            }
        }
    }

    /* Debug Message: Print error counters */
    System_printf ("Debug(Core %d): Transfer Completion without Errors - %d\n", coreNum, srioDioLsuGoodTransfers);
    System_printf ("Debug(Core %d): Transfer Completion with Errors    - %d\n", coreNum, srioDioLsuBadTransfers); 

    /* Cleanup the source & destination buffers. */
    for (srcDstBufIdx = 0; srcDstBufIdx < (SRIO_DIO_LSU_ISR_NUM_SOCKETS * SRIO_DIO_LSU_ISR_NUM_TRANSFERS); srcDstBufIdx++)
    {
    	Osal_srioDataBufferFree ((Void*)srcDataBufPtr[srcDstBufIdx], SIZE_DIO_PACKET);
    	Osal_srioDataBufferFree ((Void*)dstDataBufPtr[srcDstBufIdx], SIZE_DIO_PACKET);
    }
    
    /* Close the sockets */
    for (sockIdx = 0; sockIdx < SRIO_DIO_LSU_ISR_NUM_SOCKETS; sockIdx++)
    {
        Srio_sockClose_DIO (srioSocket[sockIdx]);
    }

    if (srioDioLsuBadTransfers > 0)
    {
        System_printf ("Debug(Core %d): DIO Transfer Data NOT completed successfully !!! \n", coreNum);
        return -1;
    }

    /* Debug Message: Data was validated */
    System_printf ("Debug(Core %d): DIO Transfer Data Validated for all iterations\n", coreNum);

    /* Debug Message: Example completed */
    if ((dio_ftype == Srio_Ftype_WRITE) && (dio_ttype == Srio_Ttype_Write_NWRITE_R))
    {
        System_printf ("Debug(Core %d): DIO Data Transfer (WRITE) with Interrupts Example Passed\n", coreNum);
    }
    else if ((dio_ftype == Srio_Ftype_REQUEST) && (dio_ttype == Srio_Ttype_Request_NREAD))
    {
        System_printf ("Debug(Core %d): DIO Data Transfer (READ) with Interrupts Example Passed\n", coreNum);
    }

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
        return -1;
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
 *      This is the main DIO Example Task 
 *
 *  @retval
 *      Not Applicable.
 */
static Void dioExampleTask(UArg arg0, UArg arg1)
{
    UInt8           isAllocated;
    Srio_DrvConfig  drvCfg;

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

    /* Enable Time Stamp Counter */
    CSL_tscEnable();

    /* Run the loopback data transfers on the system initialization core. */
    if (coreNum == CORE_SYS_INIT)
    {
        /* DIO Write Operation */
        if (dioSocketsWithISR (hDrvManagedSrioDrv, Srio_Ftype_WRITE, Srio_Ttype_Write_NWRITE_R) < 0)
        {
            System_printf ("Error: Loopback DIO ISR example for Write operation failed\n");
            Task_exit();
        }
        
        /* Reset global counters before next run */
        srioDioLsuGoodTransfers = 0;
        srioDioLsuBadTransfers  = 0;

        /* DIO Read Operation */
        if (dioSocketsWithISR (hDrvManagedSrioDrv, Srio_Ftype_REQUEST, Srio_Ttype_Request_NREAD) < 0)
        {
            System_printf ("Error: Loopback DIO ISR example for Read operation failed\n");
            Task_exit();
        }
    }

    /* Print out the Malloc & Free Counter */
    System_printf ("Debug(Core %d): Allocation Counter : %d\n", coreNum, malloc_counter);
    System_printf ("Debug(Core %d): Free Counter       : %d\n", coreNum, free_counter);

    /* Check if there is a memory leak? Since we dont implement a 'deinit' API we need to
     * be careful in these calculations
     *  - For the Application Managed Driver Instance 
     *      There will be 'numRxBuffers' + 1 (Driver Instance) 
     *  - For the Driver Managed Driver Instance 
     *      There will be 'numRxBuffers' + 'numTxBuffers' + 1 (Driver Instance)
     *  Take these into account while checking for memory leaks. */
    if (free_counter +
        (drvCfg.u.drvManagedCfg.rxCfg.numRxBuffers + drvCfg.u.drvManagedCfg.txCfg.numTxBuffers + 1) != malloc_counter)
    {
        System_printf ("Error: Memory Leak Detected\n");
        Task_exit();
    }

    /* Control comes here implies that example passed. */
    System_printf ("Debug(Core %d): DIO with Interrupts example completed successfully.\n", coreNum);
    Task_exit();
}

/**
 *  @b Description
 *  @n  
 *      Entry point for the example
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
#warn SRIO DIO LSU ISR example is not supported on SIMULATOR !!!
    System_printf ("SRIO DIO LSU ISR example is not supported on SIMULATOR. Exiting!\n");
    return;
#else
    System_printf ("Executing the SRIO DIO example on the DEVICE\n");
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
        /* All other cores need to wait for the SRIO to be initialized before they proceed. */ 
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

    /* Create the DIO Example Task.*/
    Task_Params_init(&taskParams);
    taskParams.stackSize = 4096;
    Task_create(dioExampleTask, &taskParams, NULL);

    /* Start the BIOS */
    BIOS_start();
}
