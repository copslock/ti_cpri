/**
 *   @file  device/c6657/src/device_srio_loopback.c
 *
 *   @brief   
 *      The C6657 SRIO Device specific code. The SRIO driver calls out
 *      this code to initialize the SRIO IP block. The file is provided as 
 *      a sample configuration and should be modified by customers for 
 *      their own platforms and configurations.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2011-2015 Texas Instruments, Inc.
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

/* CSL PSC Module */
#include <ti/csl/csl_pscAux.h>

/* QMSS Include */
#include <ti/drv/qmss/qmss_drv.h>

#include <ti/csl/csl_serdes.h>
#include <ti/csl/csl_serdes_srio.h>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/**********************************************************************
 ************************* LOCAL Definitions **************************
 **********************************************************************/

/* These are the GARBAGE queue handles which are used on TX to 
 * dump the descriptor if there is an error instead of recycling 
 * the descriptor to the free queue. */
static Qmss_QueueHnd garbageQueueHnd[] = {
    QMSS_PARAM_NOT_SPECIFIED,  /* Garbage len queue */
    QMSS_PARAM_NOT_SPECIFIED,  /* Garbage tout queue */
    QMSS_PARAM_NOT_SPECIFIED,  /* Garbage retry queue */
    QMSS_PARAM_NOT_SPECIFIED,  /* Garbage trans err queue */
    QMSS_PARAM_NOT_SPECIFIED,  /* Garbage prog queue */
    QMSS_PARAM_NOT_SPECIFIED   /* Garbage ssize queue */
};

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

extern const uint32_t DEVICE_ID1_16BIT;
extern const uint32_t DEVICE_ID1_8BIT;
extern const uint32_t DEVICE_ID2_16BIT;
extern const uint32_t DEVICE_ID2_8BIT;
/** These are the possible values for SRIO lane speed in Gbps */
#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(SOC_K2K) || defined(SOC_K2H)

/** TODO: Need to move to a common header used across all examples */
typedef enum
{
  srio_lane_rate_1p250Gbps = 0,                 /**< Rate of 1.250Gbps for SRIO lanes */
  srio_lane_rate_2p500Gbps,                     /**< Rate of 2.500Gbps for SRIO lanes */
  srio_lane_rate_3p125Gbps,                     /**< Rate of 3.125Gbps for SRIO lanes */
  srio_lane_rate_5p000Gbps                      /**< Rate of 5.000Gbps for SRIO lanes */
} srioLaneRateGbps_e;

/** These are the possible values for SRIO PLL input reference clock in MHZ */
typedef enum
{
  srio_ref_clock_125p00Mhz = 0,    /**< Reference clock of 125.00Mhz for SRIO PLL */
  srio_ref_clock_156p25Mhz,        /**< Reference clock of 156.25Mhz for SRIO PLL */
  srio_ref_clock_250p00Mhz,        /**< Reference clock of 250.00Mhz for SRIO PLL */
  srio_ref_clock_312p50Mhz         /**< Reference clock of 312.50Mhz for SRIO PLL */
} srioRefClockMhz_e;

/** These are the possible values for SRIO lane mode */
typedef enum
{
  srio_lanes_form_four_1x_ports = 0,             /**< SRIO lanes form four 1x ports */
  srio_lanes_form_one_2x_port_and_two_1x_ports,  /**< SRIO lanes form one 2x port and two 1x ports */
  srio_lanes_form_two_1x_ports_and_one_2x_port,  /**< SRIO lanes form two 1x ports and one 2x port */
  srio_lanes_form_two_2x_ports,                  /**< SRIO lanes form two 2x ports */
  srio_lanes_form_one_4x_port                    /**< SRIO lanes form one 4x port */
} srioLanesMode_e;

/**********************************************************************
 *********************** DEVICE SRIO FUNCTIONS ***********************
 **********************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to set the SRIO lane configuration.
 *      Prerequisite: Run setEnableSrioPllRxTx() first or it will overwrite the MSYNC setting done
 *                    by this function.
 *
 *  @param[in]  hSrio
 *      SRIO Handle for the CSL Functional layer.
 *  @param[in]  laneMode
 *      Mode number to set the lanes to a specific configuration.
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0 (Invalid lane configuration mode specified)
 */
void serdes_setSrioLanes (CSL_SrioHandle hSrio, srioLanesMode_e laneMode)
{
    /* Stub Function. TBD for SERDES configuration */

}
  
int32_t srioDefSerdesSetup (
    CSL_SrioHandle hSrio, 
    uint32_t srioSerdesVAddr,
    srioRefClockMhz_e refClockMhz, 
    srioLaneRateGbps_e linkRateGbps, 
    int isLoopbackMode
)
{
    // Write 0 to Boot complete bit
    CSL_SRIO_SetBootComplete(hSrio,0);

    CSL_SERDES_RESULT status;
    CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
    CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params;
    CSL_SERDES_REF_CLOCK serdes_ref_clk;
    CSL_SERDES_LINK_RATE serdes_link_rate;
    CSL_SERDES_LANE_CTRL_RATE serdes_lane_ctrl_rate;
    CSL_SERDES_LOOPBACK serdes_loopback;
    uint32_t i;

    memset(&serdes_lane_enable_params, 0, sizeof(serdes_lane_enable_params));

    if (refClockMhz == srio_ref_clock_156p25Mhz)
        serdes_ref_clk = CSL_SERDES_REF_CLOCK_156p25M;
    else if (refClockMhz == srio_ref_clock_125p00Mhz)
        serdes_ref_clk = CSL_SERDES_REF_CLOCK_125M;

    if (linkRateGbps == srio_lane_rate_5p000Gbps)
    {
        serdes_link_rate = CSL_SERDES_LINK_RATE_5G;
        serdes_lane_ctrl_rate = CSL_SERDES_LANE_FULL_RATE;
    }
    if (linkRateGbps == srio_lane_rate_3p125Gbps)
    {
        serdes_link_rate = CSL_SERDES_LINK_RATE_6p25G;
        serdes_lane_ctrl_rate = CSL_SERDES_LANE_HALF_RATE;
    }

    if (isLoopbackMode)
        serdes_loopback = CSL_SERDES_LOOPBACK_ENABLED;
    else
        serdes_loopback = CSL_SERDES_LOOPBACK_DISABLED;

    serdes_lane_enable_params.base_addr = srioSerdesVAddr;
    serdes_lane_enable_params.ref_clock = serdes_ref_clk;
    serdes_lane_enable_params.linkrate = serdes_link_rate;
    serdes_lane_enable_params.num_lanes = 4;
    serdes_lane_enable_params.phy_type = SERDES_SRIO;

	/* When this flag is enabled, the RX auto adaptation is disabled */
	/* Att and Boost values are obtained through Serdes Diagnostic PRBS calibration test */
	/* For higher speeds PHY-A >= 5G or when the remote transmitter (PRODUCER) is not up */
	/* at the same time as the receiver (CONSUMER), force the attenuation and boost values */
	/* with the values obtained from Serdes Diagnostic PRBS calibration test  */
	serdes_lane_enable_params.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_ENABLED;
	for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
	{
		serdes_lane_enable_params.loopback_mode[i] = serdes_loopback;
		serdes_lane_enable_params.lane_ctrl_rate[i] = serdes_lane_ctrl_rate;
		
        /* When RX auto adaptation is on, these are the starting values used */
        /* for att, boost adaptation */
        serdes_lane_enable_params.rx_coeff.att_start[i] = 7;
        serdes_lane_enable_params.rx_coeff.boost_start[i] = 5;

        /* For higher speeds PHY-A, force attenuation and boost values  */
	    /* See forceattboost description above for more details  */
        serdes_lane_enable_params.rx_coeff.force_att_val[i] = 1;
        serdes_lane_enable_params.rx_coeff.force_boost_val[i] = 6;

        /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
        serdes_lane_enable_params.tx_coeff.cm_coeff[i] = 0;
        serdes_lane_enable_params.tx_coeff.c1_coeff[i] = 0;
        serdes_lane_enable_params.tx_coeff.c2_coeff[i] = 0;
        serdes_lane_enable_params.tx_coeff.tx_att[i] = 12;
        serdes_lane_enable_params.tx_coeff.tx_vreg[i] = 4;
	}
	serdes_lane_enable_params.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
    serdes_lane_enable_params.lane_mask = 0xF;

    status = CSL_SrioSerdesInit(serdes_lane_enable_params.base_addr,
    								serdes_lane_enable_params.ref_clock,
    								serdes_lane_enable_params.linkrate);

    if (status != 0)
    {
    	printf ("Invalid Serdes Init Params\n");
    }

    /* Common Init Mode */
    /* Iteration Mode needs to be set to Common Init Mode first with a
     * lane_mask value equal to the total number of lanes being configured.
     * For example, if there are a total of 2 lanes being configured, lane
     * mask needs to be set to 0x3 */
    serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
    serdes_lane_enable_params.lane_mask = 0xF;
    lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
    if (lane_retval != 0)
    {
        printf ("Invalid Serdes Common Init\n");
        exit(0);
    }
    printf("SRIO Serdes Common Init Complete\n");

    /* Lane Init Mode */
    /* Once CSL_SerdesLaneEnable is called with
     * iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to
     * be initialized by setting
     * iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask
     * equal to the specific lane being configured.  For example, if lane 0 is
     * being configured, lane mask needs to be set to 0x1. if lane 1 is being
     * configured, lane mask needs to be 0x2 etc */
    serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
    for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
    {
        serdes_lane_enable_params.lane_mask = 1<<i;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
        if (lane_retval != 0)
        {
            printf ("Invalid Serdes Lane Enable Init\n");
            exit(0);
        }
        printf("SRIO Serdes Lane %d Init Complete\n", i);
    }

    printf("SRIO Serdes Init Complete\n");


    return 0;
}
#endif
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
#ifdef _TMS320C6X
#pragma CODE_SECTION(SrioDevice_init, ".text:SrioDevice_init");
#endif
int32_t SrioDevice_init (
#ifndef __LINUX_USER_SPACE
    void
#else
    CSL_SrioHandle hSrio,
    uint32_t       srioSerdesVAddr
#endif
)
{
	int32_t             i;
    SRIO_PE_FEATURES    peFeatures;
    SRIO_OP_CAR         opCar;
    Qmss_QueueHnd       queueHnd;
    uint8_t             isAllocated;
#ifndef __LINUX_USER_SPACE
#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(SOC_K2K) || defined(SOC_K2H)
    uint32_t            srioSerdesVAddr = CSL_SRIO_SERDES_CFG_REGS;
#endif    
    CSL_SrioHandle      hSrio;

    /* Get the CSL SRIO Handle. */
    hSrio = CSL_SRIO_Open (0);
#endif
    if (hSrio == NULL)
        return -1;
 
#ifndef __LINUX_USER_SPACE
    /* Code to disable SRIO reset isolation */
    if (CSL_PSC_isModuleResetIsolationEnabled(CSL_PSC_LPSC_SRIO))
        CSL_PSC_disableModuleResetIsolation(CSL_PSC_LPSC_SRIO);
#endif

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

    /* Configure SRIO ports to operate in loopback mode. */
    CSL_SRIO_SetLoopbackMode(hSrio, 0);
    CSL_SRIO_SetLoopbackMode(hSrio, 1);
    CSL_SRIO_SetLoopbackMode(hSrio, 2);
    CSL_SRIO_SetLoopbackMode(hSrio, 3);

	/* Enable Automatic Priority Promotion of response packets. */
	CSL_SRIO_EnableAutomaticPriorityPromotion(hSrio);

	/* Set the SRIO Prescalar select to operate in the range of 44.7 to 89.5 */
	CSL_SRIO_SetPrescalarSelect (hSrio, 0);

#ifndef __LINUX_USER_SPACE
    /* Unlock the Boot Configuration Kicker */
    CSL_BootCfgUnlockKicker ();
#endif
#if defined(SOC_C6657)
    /* Assuming the link rate is 2500; program the PLL accordingly. */
    CSL_BootCfgSetSRIOSERDESConfigPLL (0x233);

    /* Configure the SRIO SERDES Receive Configuration. */
    CSL_BootCfgSetSRIOSERDESRxConfig (0, 0x004404a5);
    CSL_BootCfgSetSRIOSERDESRxConfig (1, 0x004404a5);
    CSL_BootCfgSetSRIOSERDESRxConfig (2, 0x004404a5);
    CSL_BootCfgSetSRIOSERDESRxConfig (3, 0x004404a5);

    /* Configure the SRIO SERDES Transmit Configuration. */
    CSL_BootCfgSetSRIOSERDESTxConfig (0, 0x001807a5);
    CSL_BootCfgSetSRIOSERDESTxConfig (1, 0x001807a5);
    CSL_BootCfgSetSRIOSERDESTxConfig (2, 0x001807a5);
    CSL_BootCfgSetSRIOSERDESTxConfig (3, 0x001807a5);
#endif    

#ifndef SIMULATOR_SUPPORT
#if defined(DEVICE_K2K) || defined(DEVICE_K2H) || defined(SOC_K2K) || defined(SOC_K2H)
    srioDefSerdesSetup(hSrio,srioSerdesVAddr,srio_ref_clock_156p25Mhz,srio_lane_rate_5p000Gbps,1);
#else
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
#endif

    /* Clear the LSU pending interrupts. */
    CSL_SRIO_ClearLSUPendingInterrupt (hSrio, 0xFFFFFFFF, 0xFFFFFFFF);

    /* Set the Device Information */
    CSL_SRIO_SetDeviceInfo (hSrio, DEVICE_ID1_16BIT, DEVICE_VENDOR_ID, DEVICE_REVISION);

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
    memset ((void *) &opCar, 0, sizeof (opCar));
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
	memset ((void *) &opCar, 0, sizeof (opCar));
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
    CSL_SRIO_SetTLMPortBaseRoutingPatternMatch(hSrio, 0, 1, DEVICE_ID2_16BIT, 0xFFFF);
    CSL_SRIO_SetTLMPortBaseRoutingPatternMatch(hSrio, 1, 0, DEVICE_ID2_8BIT,  0xFF);

    /* We need to open the Garbage collection queues in the QMSS. This is done to ensure that 
     * these queues are not opened by another system entity. */
    for (i = 0; i < 6; i++)
    {
        /* Open the Garabage queues */
        garbageQueueHnd[i] = queueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (queueHnd < 0)
            return -1;

        /* Make sure the queue has not been opened already; we dont the queues to be shared by some other
         * entity in the system. */
        if (isAllocated > 1)
            return -1;
    }

    /* Set the Transmit Garbage Collection Information. */
    CSL_SRIO_SetTxGarbageCollectionInfo (
        hSrio, 
        Qmss_getQIDFromHandle(garbageQueueHnd[0]), 
        Qmss_getQIDFromHandle(garbageQueueHnd[1]),
        Qmss_getQIDFromHandle(garbageQueueHnd[2]), 
        Qmss_getQIDFromHandle(garbageQueueHnd[3]),
        Qmss_getQIDFromHandle(garbageQueueHnd[4]), 
        Qmss_getQIDFromHandle(garbageQueueHnd[5]));

    /* Set the Host Device Identifier. */
    CSL_SRIO_SetHostDeviceID (hSrio, DEVICE_ID1_16BIT);

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
        while (CSL_SRIO_IsPortOk (hSrio, i) != TRUE);
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
 *  @b Description
 *  @n
 *      The function provides the deinitialization sequence for the SRIO IP
 *      block. This can be modified by customers for their application and
 *      configuration.
 *
 *  @retval
 *      Success     -   0
 *  @retval
 *      Error       -   <0
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION(SrioDevice_deinit, ".text:SrioDevice_deinit");
#endif
int32_t SrioDevice_deinit (
#ifndef __LINUX_USER_SPACE
    void
#else
    CSL_SrioHandle hSrio
#endif
)
{
    int                 i;
#ifndef __LINUX_USER_SPACE
    CSL_SrioHandle      hSrio;
    /* Get the CSL SRIO Handle. */
    hSrio = CSL_SRIO_Open (0);
#endif
    if (hSrio == NULL)
        return -1;

    /* Disable the SRIO Global block */
    CSL_SRIO_GlobalDisable (hSrio);

    /* Disable each of the individual SRIO blocks. */
    for(i = 0; i <= 9; i++)
        CSL_SRIO_DisableBlock(hSrio, i);

    /* Set boot complete to be 0; we are not done with the initialization. */
    CSL_SRIO_SetBootComplete(hSrio, 0);

    /* Close the garbage queues */
    for (i = 0; i < 6; i++)
    {
        /* Close the Garabage queues */
        if (Qmss_queueClose (garbageQueueHnd[i]) < QMSS_SOK)
        {
            return -1;
        }

        garbageQueueHnd[i] = QMSS_PARAM_NOT_SPECIFIED;
    }

    return 0;
}
/**
@}
*/





