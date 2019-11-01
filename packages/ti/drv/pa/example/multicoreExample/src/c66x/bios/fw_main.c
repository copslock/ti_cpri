/**
 * @file fw_main.c
 *
 * @brief
 *  Example to illustrate the usage of EMAC CPSW3G switch using CPPI, QMSS
 * 	low level drivers and CSL.
 *
 * 	This example application does the following:
 * 	    (1) Initializes:
 * 	            (a) Queue Manager (QM) Subsystem
 * 	            (b) Packet Accelerator (PA) CPPI DMA
 * 	            (c) Ethernet Subsystem (Ethernet switch + SGMII + MDIO)
 * 	            (d) PA Subsystem + PDSP
 *
 * 	    (2) Sets up the CPPI descriptors and Queues required for sending and
 * 	        receiving data using Ethernet.
 * 	            (a) Uses Host descriptors
 * 	            (b) Uses High Priority Accumulation interrupts
 *
 * 	    (3) Sets up the example application's configuration (MAC address
 * 	        it uses to send/recv data; IP address and port number it's listening
 * 	        on) in PA Subsystem so as to enable the PASS to forward all packets
 * 	        matching this configuration onto the application for processing.
 * 	            (a) Switch MAC address configured   =   0x10:0x11:0x12:0x13:0x14:0x15
 * 	            (b) Example's IP address            =   192.168.1.10
 * 	            (c) Example App's listening port    =   0x5678
 *
 * 	    (4) Sends packets onto wire
 * 	        (constructed manually in code here with following settings):
 * 	            (a) Source MAC      =   0x00:0x01:0x02:0x03:0x04:0x05
 * 	                Destination MAC =   0x10:0x11:0x12:0x13:0x14:0x15
 *              (b) Source IP       =   192.168.1.1
 *                  Destination IP  =   192.168.1.10
 *              (c) Source Port     =   0x1234
 *                  Destination Port=   0x5678
 *              (d) Payload Data (80 bytes)
 *
 *          The packets sent by the application are sent onto wire and
 *          since the destination MAC on the packet is the Ethernet Switch
 *          MAC address, the packets are received by simulator and passed
 *          back up to the example application for processing.
 *
 *      (5) Application receives all packets using QM High priority interrupt
 *          registered; Validates received packet against data sent.
 *
 *  Example application Setup:
 *
 *          PC Running Simulator using CCS connected to a
 *          Switch/Hub. You could put another PC on the Hub to observe packets
 *          being sent onto wire.
 *
 *          Please consult the Readme.txt packaged with the example to
 *          setup the CCS simulator configuration required to run this example
 *          succesfully.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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
#include <multicore_example.h>
#include "ti/csl/csl_bootcfgAux.h"

/* RM Variables */
Rm_Handle rmHandle;
/* RM initialization sync point */
#pragma DATA_SECTION (isRmInitialized, ".rm");
volatile uint32_t           isRmInitialized;

#ifdef  SIMULATOR_SUPPORT
uint32_t autodetectLogic = FALSE;
#else
uint32_t autodetectLogic = TRUE;
#endif


/** ============================================================================
 *   @n@b main
 *
 *   @b Description
 *   @n Entry point for single core example application.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return
 *   @n None
 * =============================================================================
 */
Int32 main (Void)
{
#if RM
    /* RM configuration */
    Rm_InitCfg          rmInitCfg;
    char                rmInstName[RM_NAME_MAX_CHARS];
    int32_t             rmResult, coreNum;
#else
	Task_Params                	cpswTaskParams;
#endif
    uint32_t                    bootMode;
    /* Init internal cycle counter */
    TSCL = 1; 

    if (autodetectLogic == TRUE)
    {
       bootMode = CSL_BootCfgGetBootMode() & 0x7;

       if (bootMode == 0)
    	  no_bootMode = TRUE;
       else
    	  no_bootMode = FALSE;
    }
    else {
    	no_bootMode = TRUE;
    }

    /* Get the core number. */
	coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Power up PASS from Master process/core */
    if ( (coreNum == SYSINIT) && (no_bootMode == TRUE) )
    {
        /* Enable PASS power domain */
        passPowerUp();
    }

#if RM
    /* Reset the variable to indicate to other cores RM init is not yet done */
    isRmInitialized = 0;

    /* Initialize the heap in shared memory used by RM. Using IPC module to do that */
    Ipc_start();

    /* Initialize RM */
    if (coreNum == SYSINIT)
    {
       	/* Create the Server instance */
        memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
        System_sprintf (rmInstName, "RM_Server");
        rmInitCfg.instName = rmInstName;
        rmInitCfg.instType = Rm_instType_SERVER;

        if (no_bootMode == TRUE) {
            rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
        	rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspOnlyPolicy;
        }
        else {
            rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
#ifndef SOC_C6678            
        	rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspPlusArmPolicy;
#else  // There is no ARM processor at C6678
        	rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspOnlyPolicy;
#endif            
        }
        rmHandle = Rm_init(&rmInitCfg, &rmResult);
        if (rmResult != RM_OK)
        {
            errorCount++;
            System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", coreNum, rmResult);
            exit(0);
        }
        isRmInitialized = 1;
        SYS_CACHE_WB((void*)&isRmInitialized, 4, CACHE_WAIT);
    }
    else
    {
    	/* Create a RM Client instance */
        memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
        System_sprintf (rmInstName, "RM_Client%d", coreNum);
        rmInitCfg.instName = rmInstName;
        rmInitCfg.instType = Rm_instType_CLIENT;
        rmHandle = Rm_init(&rmInitCfg, &rmResult);
        if (rmResult != RM_OK)
        {
            errorCount++;
            System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", coreNum, rmResult);
            return 0;
        }

        do{
            SYS_CACHE_INV ((void*)&isRmInitialized, 4, CACHE_WAIT);
        } while (isRmInitialized == 0);

    }

    if (setupRmTransConfig(pa_MC_EXAMPLE_NUM_CORES, SYSINIT, MultiCoreApp) < 0)
    {
        errorCount++;
        System_printf ("Error core %d : Transport setup for RM error\n", coreNum);
        return 0;
    }
#else
    /* Initialize the task params */
    Task_Params_init(&cpswTaskParams);

    /* Create the CPSW single core example task */
    Task_create((Task_FuncPtr)&MultiCoreApp, &cpswTaskParams, NULL);
#endif /* RM */

    /* Start the BIOS Task scheduler */
	BIOS_start ();

	return 0;
}

/* Nothing past this point */
