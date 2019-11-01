/**
 *   @file  device_srio.c
 *
 *   @brief   
 *      The 6670 SRIO Device specific code. The SRIO driver calls out
 *      this code to initialize the SRIO IP block. The file is provided as 
 *      a sample configuration and should be modified by customers for 
 *      their own platforms and configurations.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2011 Texas Instruments, Inc.
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

#include <xdc/runtime/System.h>

/* SRIO Driver Includes. */
#include <ti/drv/srio/srio_types.h>
#include <ti/drv/srio/include/listlib.h>
#include <ti/drv/srio/srio_drv.h>

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>
#include <ti/csl/csl_srioAuxPhyLayer.h>

/* CSL BootCfg Module */
#include <ti/csl/csl_bootcfg.h>
#include <ti/csl/csl_bootcfgAux.h>

/* CSL Chip Functional Layer */
#include <ti/csl/csl_chip.h>

/* QMSS Include */
#include <ti/drv/qmss/qmss_drv.h>

/* PDK module Headers */
#include <ti/platform/platform.h>

/**********************************************************************
 ************************* LOCAL Definitions **************************
 **********************************************************************/

/* These are the GARBAGE queues which are used by the TXU to dump the 
 * descriptor if there is an error instead of recycling the descriptor
 * to the free queue. */
extern const UInt32 GARBAGE_LEN_QUEUE;
extern const UInt32 GARBAGE_TOUT_QUEUE;
extern const UInt32 GARBAGE_RETRY_QUEUE;
extern const UInt32 GARBAGE_TRANS_ERR_QUEUE;
extern const UInt32 GARBAGE_PROG_QUEUE;
extern const UInt32 GARBAGE_SSIZE_QUEUE;

/* SRIO Device Information
 * - 16 bit Device Identifier.
 * - 8 bit Device Identifier.
 * - Vendor Identifier. 
 * - Device Revision. */
#define DEVICE_VENDOR_ID            0x30
#define DEVICE_REVISION             0x0

/* SRIO Assembly Information
 * - Assembly Identifier
 * - Assembly Vendor Identifier. 
 * - Assembly Device Revision. 
 * - Assembly Extension Features */
#define DEVICE_ASSEMBLY_ID          0x0
#define DEVICE_ASSEMBLY_VENDOR_ID   0x30
#define DEVICE_ASSEMBLY_REVISION    0x0
#define DEVICE_ASSEMBLY_INFO        0x0100

/**********************************************************************
 ************************* Extern Definitions *************************
 **********************************************************************/

extern const UInt32 DEVICE_ID1_16BIT;
extern const UInt32 DEVICE_ID1_8BIT;
extern const UInt32 DEVICE_ID2_16BIT;
extern const UInt32 DEVICE_ID2_8BIT;
extern const UInt32 DEVICE_ID3_16BIT;
extern const UInt32 DEVICE_ID3_8BIT;
extern const UInt32 DEVICE_ID4_16BIT;
extern const UInt32 DEVICE_ID4_8BIT;

/**********************************************************************
 *********************** DEVICE SRIO FUNCTIONS ***********************
 **********************************************************************/

/** @addtogroup SRIO_DEVICE_API
 @{ */

/**
 *  @b Description
 *  @n  
 *      The function provides the initialization sequence for the SRIO IP
 *      block. This can be modified by customers for their application and
 *      configuration.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
int32_t SrioDevice_init (void)
{
    CSL_SrioHandle      hSrio;
	int32_t             i, j;
    SRIO_PE_FEATURES    peFeatures;
    SRIO_OP_CAR         opCar;
    Qmss_QueueHnd       queueHnd;
    Qmss_Queue queueInfo;
    uint8_t             isAllocated;
    Bool sameGarbageQ;
    uint32_t            garbageQueue[6];

    garbageQueue[0] = GARBAGE_LEN_QUEUE;
    garbageQueue[1] = GARBAGE_TOUT_QUEUE;
    garbageQueue[2] = GARBAGE_RETRY_QUEUE;
    garbageQueue[3] = GARBAGE_TRANS_ERR_QUEUE;
    garbageQueue[4] = GARBAGE_PROG_QUEUE;
    garbageQueue[5] = GARBAGE_SSIZE_QUEUE;

    /* Get the CSL SRIO Handle. */
    hSrio = CSL_SRIO_Open (0);
    if (hSrio == NULL)
        return -1;
 
    /* Disable the SRIO Global block */
   	CSL_SRIO_GlobalDisable (hSrio);
   	
   	/* Disable each of the individual SRIO blocks. */
   	for(i = 0; i <= 9; i++)
   		CSL_SRIO_DisableBlock(hSrio, i);

    /* Set boot complete to be 0; we are not done with the initialization. */	
	CSL_SRIO_SetBootComplete(hSrio, 0);

    /* Now enable the SRIO block and all the individual blocks also. */
    CSL_SRIO_GlobalEnable (hSrio);
    for(i = 0; i <= 9; i++)
        CSL_SRIO_EnableBlock(hSrio,i);

    /* Configure SRIO to operate in loopback mode. */
//    CSL_SRIO_SetLoopbackMode(hSrio,0);
//    CSL_SRIO_SetLoopbackMode(hSrio,1);
//    CSL_SRIO_SetLoopbackMode(hSrio,2);
//    CSL_SRIO_SetLoopbackMode(hSrio,3);

    /* Configure SRIO to operate in normal mode. */
    CSL_SRIO_SetNormalMode(hSrio,0);
    CSL_SRIO_SetNormalMode(hSrio,1);
    CSL_SRIO_SetNormalMode(hSrio,2);
    CSL_SRIO_SetNormalMode(hSrio,3);

	/* Enable Automatic Priority Promotion of response packets. */
	CSL_SRIO_EnableAutomaticPriorityPromotion(hSrio);

	/* Set the SRIO Prescalar select to operate in the range of 44.7 to 89.5 */
	CSL_SRIO_SetPrescalarSelect (hSrio, 0);

    /* Unlock the Boot Configuration Kicker */
    CSL_BootCfgUnlockKicker ();

    /* Assuming the ref_clock of 250.0 MHz and link rate is 3.125 Gbps; program the PLL accordingly. */
    CSL_BootCfgSetSRIOSERDESConfigPLL (0x233);

    /* Configure the SRIO SERDES Receive Configuration. */
    /* loopback mode */
 //  CSL_BootCfgSetSRIOSERDESRxConfig (0, 0x01C40495);
 //  CSL_BootCfgSetSRIOSERDESRxConfig (1, 0x01C40495);
 //  CSL_BootCfgSetSRIOSERDESRxConfig (2, 0x01C40495);
 //  CSL_BootCfgSetSRIOSERDESRxConfig (3, 0x01C40495);

    /* Non-loopback */
    CSL_BootCfgSetSRIOSERDESRxConfig (0, 0x00440495);
    CSL_BootCfgSetSRIOSERDESRxConfig (1, 0x00440495);
    CSL_BootCfgSetSRIOSERDESRxConfig (2, 0x00440495);
    CSL_BootCfgSetSRIOSERDESRxConfig (3, 0x00440495);

    /* Configure the SRIO SERDES Transmit Configuration. */
    /* loopback mode */
 //  CSL_BootCfgSetSRIOSERDESTxConfig (0, 0x00780795);
 //  CSL_BootCfgSetSRIOSERDESTxConfig (1, 0x00780795);
 //  CSL_BootCfgSetSRIOSERDESTxConfig (2, 0x00780795);
 //  CSL_BootCfgSetSRIOSERDESTxConfig (3, 0x00780795);

    /* Non-loopback */
    CSL_BootCfgSetSRIOSERDESTxConfig (0, 0x00180795);
    CSL_BootCfgSetSRIOSERDESTxConfig (1, 0x00180795);
    CSL_BootCfgSetSRIOSERDESTxConfig (2, 0x00180795);
    CSL_BootCfgSetSRIOSERDESTxConfig (3, 0x00180795);

#ifndef SIMULATOR_SUPPORT
    /* Loop around till the SERDES PLL is not locked. */
    while (1)
    {
        uint32_t    status;

        /* Get the SRIO SERDES Status */
        CSL_BootCfgGetSRIOSERDESStatus(&status);
        if (status & 0x1)
            break;
    }
#endif

    /* Clear the LSU pending interrupts. */
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 1, 0xFF);
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 2, 0xFF);
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 3, 0xFF);
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 4, 0xFF);
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 5, 0xFF);
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 6, 0xFF);
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 7, 0xFF);
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 8, 0xFF);

    /* Set the Device Information */
    CSL_SRIO_SetDeviceInfo (hSrio, DEVICE_ID1_8BIT, DEVICE_VENDOR_ID, DEVICE_REVISION);

    /* Set the Assembly Information */
    CSL_SRIO_SetAssemblyInfo(hSrio, DEVICE_ASSEMBLY_ID, DEVICE_ASSEMBLY_VENDOR_ID, 
                             DEVICE_ASSEMBLY_REVISION, DEVICE_ASSEMBLY_INFO);

    /* TODO: Configure the processing element features
     *  The SRIO RL file is missing the Re-transmit Suppression Support (Bit6) field definition */
    peFeatures.isBridge                          = 0;
    peFeatures.isEndpoint                        = 0;
    peFeatures.isProcessor                       = 1;
    peFeatures.isSwitch                          = 0;
    peFeatures.isMultiport                       = 0;
    peFeatures.isFlowArbiterationSupported       = 0;
    peFeatures.isMulticastSupported              = 0;
    peFeatures.isExtendedRouteConfigSupported    = 0;
    peFeatures.isStandardRouteConfigSupported    = 1;
    peFeatures.isFlowControlSupported            = 1;
    peFeatures.isCRFSupported                    = 0;
    peFeatures.isCTLSSupported                   = 1;
    peFeatures.isExtendedFeaturePtrValid         = 1;
    peFeatures.numAddressBitSupported            = 1;
    CSL_SRIO_SetProcessingElementFeatures (hSrio, &peFeatures);

    /* Configure the source operation CAR */
    memset ((Void *) &opCar, 0, sizeof (opCar));
    opCar.portWriteOperationSupport = 1;
    opCar.atomicClearSupport        = 1;
    opCar.atomicSetSupport          = 1;
    opCar.atomicDecSupport          = 1;
    opCar.atomicIncSupport          = 1;
    opCar.atomicTestSwapSupport     = 1;
    opCar.doorbellSupport           = 1;
    opCar.dataMessageSupport        = 1;
    opCar.writeResponseSupport      = 1;
    opCar.streamWriteSupport        = 1;
    opCar.writeSupport              = 1;
    opCar.readSupport               = 1;
    opCar.dataStreamingSupport      = 1;
    CSL_SRIO_SetSourceOperationCAR (hSrio, &opCar);

    /* Configure the destination operation CAR */
	memset ((Void *) &opCar, 0, sizeof (opCar));
    opCar.portWriteOperationSupport  = 1;
    opCar.doorbellSupport            = 1;
    opCar.dataMessageSupport         = 1;
    opCar.writeResponseSupport       = 1;
    opCar.streamWriteSupport         = 1;
    opCar.writeSupport               = 1;
    opCar.readSupport                = 1;
    CSL_SRIO_SetDestOperationCAR (hSrio, &opCar);

    /* Set the 16 bit and 8 bit identifier for the SRIO Device. */
    CSL_SRIO_SetDeviceIDCSR (hSrio, DEVICE_ID1_8BIT, DEVICE_ID1_16BIT);

    /* Enable TLM Base Routing Information for Maintainance Requests & ensure that
     * the BRR's can be used by all the ports. */
    CSL_SRIO_SetTLMPortBaseRoutingInfo(hSrio, 0, 1, 1, 1, 0);
    CSL_SRIO_SetTLMPortBaseRoutingInfo(hSrio, 0, 2, 1, 1, 0);
    CSL_SRIO_SetTLMPortBaseRoutingInfo(hSrio, 0, 3, 1, 1, 0);
    CSL_SRIO_SetTLMPortBaseRoutingInfo(hSrio, 1, 0, 1, 1, 0);

    /* Configure the Base Routing Register to ensure that all packets matching the 
     * Device Identifier & the Secondary Device Id are admitted. */
    CSL_SRIO_SetTLMPortBaseRoutingPatternMatch(hSrio, 0, 1, DEVICE_ID2_8BIT, 0xFF);
    CSL_SRIO_SetTLMPortBaseRoutingPatternMatch(hSrio, 0, 2, DEVICE_ID3_8BIT, 0xFF);
    CSL_SRIO_SetTLMPortBaseRoutingPatternMatch(hSrio, 0, 3, DEVICE_ID4_8BIT, 0xFF);
    CSL_SRIO_SetTLMPortBaseRoutingPatternMatch(hSrio, 1, 0, DEVICE_ID2_16BIT,  0xFFFF);

    /* We need to open the Garbage collection queues in the QMSS. This is done to ensure that 
     * these queues are not opened by another system entity. */
    for (i = 0; i < 6; i++)
    {
        /* Open the Garabage queues */
        queueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, garbageQueue[i], &isAllocated);
        if (queueHnd < 0)
            return -1;

        /* Make sure the queue has not been opened already; we dont the queues to be shared by some other
         * entity in the system. */
        if (isAllocated > 1)
        {
          /* Check if the already allocated queue is for another garbage Q.  Can group garbage queues
            * for error handling */
          sameGarbageQ = FALSE;
          queueInfo = Qmss_getQueueNumber (queueHnd);

          /* Check all previously allocated garbage Qs */
          for (j = 0; j < i; j++)
          {
            if (queueInfo.qNum == garbageQueue[j])
            {
              sameGarbageQ = TRUE;
            }
          }

          if (!sameGarbageQ)
          {
            return -1;
          }
        }
    }

    /* Set the Transmit Garbage Collection Information. */
    CSL_SRIO_SetTxGarbageCollectionInfo (hSrio, GARBAGE_LEN_QUEUE, GARBAGE_TOUT_QUEUE, 
                                         GARBAGE_RETRY_QUEUE, GARBAGE_TRANS_ERR_QUEUE, 
                                         GARBAGE_PROG_QUEUE, GARBAGE_SSIZE_QUEUE);

    /* Set the Host Device Identifier. */
    CSL_SRIO_SetHostDeviceID (hSrio, DEVICE_ID1_8BIT);

    /* Configure the component tag CSR */
    CSL_SRIO_SetCompTagCSR (hSrio, 0x00000000);

    /* Configure the PLM for all the ports. */
	for (i = 0; i < 4; i++)
	{
	    /* Set the PLM Port Silence Timer. */	
        CSL_SRIO_SetPLMPortSilenceTimer (hSrio, i, 0x2);

        /* TODO: We need to ensure that the Port 0 is configured to support both
         * the 2x and 4x modes. The Port Width field is read only. So here we simply
         * ensure that the Input and Output ports are enabled. */
        CSL_SRIO_EnableInputPort (hSrio, i);
        CSL_SRIO_EnableOutputPort (hSrio, i);

        /* Set the PLM Port Discovery Timer. */
        CSL_SRIO_SetPLMPortDiscoveryTimer (hSrio, i, 0x2);

        /* Reset the Port Write Reception capture. */
        CSL_SRIO_SetPortWriteReceptionCapture(hSrio, i, 0x0);
    }

    /* Set the Port link timeout CSR */
    CSL_SRIO_SetPortLinkTimeoutCSR (hSrio, 0x000FFF);

    /* Set the Port General CSR: Only executing as Master Enable */
    CSL_SRIO_SetPortGeneralCSR (hSrio, 0, 1, 0);

    /* Clear the sticky register bits. */
    CSL_SRIO_SetLLMResetControl (hSrio, 1);

    /* Set the device id to be 0 for the Maintenance Port-Write operation 
     * to report errors to a system host. */
    CSL_SRIO_SetPortWriteDeviceId (hSrio, 0x0, 0x0, 0x0);

    /* Set the Data Streaming MTU */
    CSL_SRIO_SetDataStreamingMTU (hSrio, 64);

    /* Configure the path mode for the ports. */
    for(i = 0; i < 4; i++)
        CSL_SRIO_SetPLMPortPathControlMode (hSrio, i, 0);

    /* Set the LLM Port IP Prescalar. */
    CSL_SRIO_SetLLMPortIPPrescalar (hSrio, 0x21);

    /* Enable the peripheral. */
    CSL_SRIO_EnablePeripheral(hSrio);

    /* Configuration has been completed. */
    CSL_SRIO_SetBootComplete(hSrio, 1);

#ifndef SIMULATOR_SUPPORT
    /* This code checks if the ports are operational or not. The functionality is not supported 
     * on the simulator. */    
	for(i = 0; i < 4; i++)
	{
     UInt waitLength = 0;
     UInt portIsOkay = 0;

     /* Check SRIO ports every millisecond for 2 seconds */
     while (waitLength < 2000)
     {
       if (CSL_SRIO_IsPortOk (hSrio, i) == TRUE)
       {
         portIsOkay = 1;
         break;
       }
       platform_delay(1000);
       waitLength++;
     }

     if (portIsOkay)
     {
       System_printf ("Port %d is okay\n", i);
     }
     else
     {
       System_printf ("Port %d did not initialize\n", i);
     }
	}
#endif

    /* Set all the queues 0 to operate at the same priority level and to send packets onto Port 0 */
    for (i =0 ; i < 16; i++)
        CSL_SRIO_SetTxQueueSchedInfo(hSrio, i, 0, 0);

    /* Set the Doorbell route to determine which routing table is to be used 
     * This configuration implies that the Interrupt Routing Table is configured as 
     * follows:-
     *  Interrupt Destination 0 - INTDST 16 
     *  Interrupt Destination 1 - INTDST 17 
     *  Interrupt Destination 2 - INTDST 18
     *  Interrupt Destination 3 - INTDST 19 
     */
    CSL_SRIO_SetDoorbellRoute(hSrio, 0);

    /* Route the Doorbell interrupts. 
     *  Doorbell Register 0 - All 16 Doorbits are routed to Interrupt Destination 0. 
     *  Doorbell Register 1 - All 16 Doorbits are routed to Interrupt Destination 1. 
     *  Doorbell Register 2 - All 16 Doorbits are routed to Interrupt Destination 2. 
     *  Doorbell Register 3 - All 16 Doorbits are routed to Interrupt Destination 3. */
    for (i = 0; i < 16; i++)
    {
        CSL_SRIO_RouteDoorbellInterrupts(hSrio, 0, i, 0);
        CSL_SRIO_RouteDoorbellInterrupts(hSrio, 1, i, 1);
        CSL_SRIO_RouteDoorbellInterrupts(hSrio, 2, i, 2);
        CSL_SRIO_RouteDoorbellInterrupts(hSrio, 3, i, 3);
    }

    /* Initialization has been completed. */
    return 0;
}

/**
@}
*/

