/**  
 * @file multicore_example.c
 *
 * @brief 
 *  Example to illustrate the usage of EMAC CPSW3G switch using CPPI, QMSS
 * 	low level drivers and CSL.
 *
 * 	This example application does the following:
 * 	    (1) Initializes:
 * 	            (a) Queue Manager (QM) Subsystem 
 * 	            (b) Packet Accelerator (PA) CPPI DMA 
 * 	            (c) Ethernet Subsystem (Ethernet switch + SGMII + MDIO) - (Note: Applicable only for NO_BOOT mode)
 * 	            (d) PA Subsystem + PDSP (Note: PDSP is initialized only during NO_BOOT mode)
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
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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
#ifdef __LINUX_USER_SPACE
#include "armv7/linux/fw_test.h"
#include <unistd.h>
#endif

/* PA LLD include */
#include <ti/drv/pa/pa.h>

/**************************************************************
************************** DEFINITIONS ************************
***************************************************************/
/* Number of packets to be used for testing the example. */
#define                     MAX_NUM_PACKETS                         10u

/**
 *  @b Description
 *  @n  
 *      Yields the task using something like sleep() or usleep().
 *
 *  @retval
 *      Not Applicable
 */
void yield (void)
{
#ifdef __LINUX_USER_SPACE
    sleep(1);
//    sched_yield();
#endif
}

/**************************************************************
**************** EXAMPLE APP FUNCTIONS ************************
***************************************************************/
/** ============================================================================
 *   @n@b MultiCoreApp
 *
 *   @b Description
 *   @n Example application that sets up the application, sends, receives
 *      data.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return
 *   @n None
 *
 * =============================================================================
 */
#ifdef __LINUX_USER_SPACE
void MultiCoreApp(void* args)
#else
void MultiCoreApp(UArg arg0, UArg arg1)
#endif
{
	uint32_t	   i;

#ifdef __LINUX_USER_SPACE
#else
    volatile uint32_t     testComplete=FALSE;
    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM); 
#endif

    System_printf ("************************************************\n");
    System_printf ("*** PA Multi Core Example Started on Core %d ***\n",coreNum);
    System_printf ("************************************************\n");
    System_flush();
    
#ifndef __LINUX_USER_SPACE
    /* Disable L1 and L2 Cache */
    //CACHE_wbAllL1d (CACHE_WAIT);
    //CACHE_setL1DSize(CACHE_L1_0KCACHE);
    //CACHE_setL1PSize(CACHE_L1_0KCACHE);
    #ifndef L2_CACHE
    CACHE_setL2Size(CACHE_0KCACHE);
    #endif
#endif    
    /* Adjust the data packet as a function of the core number */
    ModifyPacket(coreNum);
    
    /* All other cores wait for core 0 to finish the global config
       and setup the QMSS/CPPI/PASS */
    if(coreNum != SYSINIT)
    {
        /* Open the Shared Memory */
    	fw_shmOpen();
        System_printf ("Waiting for global config (core %d) ...\n", coreNum);
        System_flush();
        APP_waitGlobalCfgDone();
    }
    
    /* init RM on All cores */
    initRm();

    /* Core 0 does the global initialization */
    if(coreNum == SYSINIT)
    {
        
        /* Initialize the components required to run the example:
         *  (1) QMSS
         *  (2) CPPI
         *  (3) Ethernet switch subsystem + MDIO + SGMII
         */
        /* Initialize QMSS */
        if (Init_Qmss () != 0)
        {
            System_printf ("QMSS Global init failed \n");
            System_flush();
            APP_exit (-1);
        }
        else
        {
            System_printf ("QMSS successfully initialized \n");            
            System_flush();
        }

        /* Initialize CPPI */
        if (Init_Cppi () != 0)
        {
            System_printf ("CPPI init failed \n");
            System_flush();
            APP_exit (-1);
        }
        else
        {
            System_printf ("CPPI successfully initialized \n");            
            System_flush();
        }

        /* Init PA LLD */
        if (Init_PASS () != 0)
        {
            System_printf ("PASS init failed \n");
            System_flush();
            APP_exit (-1);
        }
        else
        {
            System_printf ("PASS successfully initialized \n");            
            System_flush();
        }
#ifndef __LINUX_USER_SPACE

        if (no_bootMode == TRUE)
        {
            /* Initialize the CPSW switch */
            if (Init_Cpsw () != 0)
            {
                System_printf ("Ethernet subsystem init failed \n");
                System_flush();
                APP_exit (-1);
            }
            else
            {
                System_printf ("Ethernet subsystem successfully initialized \n");
                System_flush();
            }
        }
#endif
        /* Setup Tx */
        if (Setup_Tx () != 0)
        {
            System_printf ("Tx setup failed \n");
            System_flush();
            APP_exit (-1);
        }
        else
        {
            System_printf ("Tx setup successfully done \n");            
            System_flush();
        }
    }
    else
    {
    
        /* Cores other than 0 do local QMSS initialization */
        if (Init_Qmss_Local () != 0)
        {
            System_printf ("QMSS Local init failed \n");
            System_flush();
            APP_exit (-1);
        }
        else
        {
            System_printf ("QMSS Local successfully initialized \n");            
            System_flush();
        }
        
        if (Init_Cppi_Local() != 0)
        {
            System_printf ("CPPI Local init failed \n");
            System_flush();
            APP_exit (-1);
        }
        else
        {
            System_printf ("CPPI Local successfully initialized \n");
            System_flush();
        }
        

        if (Init_Pa_Local() != 0)
        {
            System_printf ("PA Local init failed \n");
            System_flush();
            APP_exit (-1);
        }
        else
        {
            System_printf ("PA Local successfully initialized \n");
            System_flush();
        }

       /* setup the local tx for the child process */
	   Setup_Tx_local();

    }
    

    /* Setup Rx */
    if (Setup_Rx () != 0)
    {
        System_printf ("Rx setup failed \n");
        System_flush();
        APP_exit (-1);
    }
    else
    {
        System_printf ("Rx setup successfully done \n");            
        System_flush();
    }

    /* Setup PA */
    if (Setup_PASS () != 0)
    {
        System_printf ("PASS setup failed \n");
        System_flush();
        APP_exit (-1);
        
    }
    else
    {
        System_printf ("PASS setup successfully done \n");            
        System_flush();
    }

    /* Core 0 finished the global config. Set flag so other
       cores can start their local config. */
    if (coreNum == SYSINIT)
    {
        System_printf ("Publishing global config Done from SYSINIT Core...\n");
        System_flush();
    	APP_publishGlobalCfgDone();
    }  
    
    /* All cores update the counter informing that they finished their setup */
    /* The global variable is a shared resource which is being accessed from multiple cores. 
     * So here we need to protect it and ensure that there is only 1 core which is accessing 
     * it at a time. We use a Hardware Semaphore to protect this. */
    System_printf ("Publishing local config done from core num: %d...\n", coreNum);     
    System_flush();
    APP_publishLocalCfgDone();
    
    /* All cores wait here to sync up and send packets to PA
       at the same time. */
    System_printf ("Waiting for all cores to reach the barrier before transmission starts ... \n");
    System_flush();
    APP_waitAllLocalCfgDone();
    
    /* Send data towards switch */
    System_printf ("\n\nPacket Transmission Start ... core id: %d \n", coreNum);
    System_flush();
    for (i = 0; i < MAX_NUM_PACKETS; i ++)
    {
        if (SendPacket () != 0)
        {
            System_printf ("Packet %d send failed \n", i);
            System_flush();
            APP_exit (-1);
        }
        
    }
    
    /* Wait until all packet reception is done */
    System_printf ("Packet Transmission Done.\nWait for all packets to be Received ... core num: %d\n", coreNum);
    System_flush();
    strcpy(test_stat, "TEST_PASSED");
    while (gRxCounter < gTxCounter)
    {
        if(ReceivePacket() != 0) {
        	strcpy(test_stat, "TEST_FAILED");			
            System_printf ("Test failed on core %d\n",coreNum);			
            System_flush();
        }
    }
    
    System_printf ("Core %d: Packets Sent\t\t=\t%d \nCore %d: Packets Received\t=\t%d \n",coreNum, gTxCounter, coreNum,  gRxCounter);
    System_flush();
    
    if(gRxCounter >= gTxCounter)
        APP_publishTestStatus();
    
    /* Indicate that the test is complete from slave processes */
    APP_publishLocalTestDone();

	/* Delete the port added */
    if (Del_Port() < 0)
    {
	    System_printf ("Failed to delete the L2 entry for the MAC address for procId: %d\n", coreNum);
        System_flush();
    }	

    /* Core 0 collects all the results and declare PASS or Fail */
    if(coreNum == SYSINIT)
    {
   	    System_printf ("Wait for all packets to be Received in all cores... \n");
	    /* The global variable is a shared resource which is being accessed from multiple cores. 
	    * So here we need to protect it and ensure that there is only 1 core which is accessing 
	    * it at a time. We use a Hardware Semaphore to protect this. */
	    System_printf ("Updating the Test Status core: %d, status:%s\n", coreNum, test_stat);		
        System_flush();
        APP_waitAllLocalTestDone();
        
        if (APP_checkTestStatus())
        {
	        System_printf ("All tests have passed!\n");		
            System_flush();
        }
    }
    else
    {

        /* Clean up for Qmss/CPPI from slave cores */
    	clearFramework(coreNum);

    	/* The global variable is a shared resource which is being accessed from multiple cores. 
    	 * So here we need to protect it and ensure that there is only 1 core which is accessing 
    	 * it at a time. We use a Hardware Semaphore to protect this. */
    	System_printf ("Updating the Test Status core: %d, status:%s\n", coreNum, test_stat);	 
        System_flush();
	
    }

    if (coreNum == SYSINIT) 
	{
     	/* Delete the Ip Address added */
     	if (Del_IPAddress() < 0)
     	{
     		System_printf ("Failed to delete the L2 entry for IP address for procId: %d\n", coreNum);
             System_flush();
     	}
     
     	/* Delete the MAC Address added */
     	if (Del_MACAddress() < 0)
     	{
     		System_printf ("Failed to clean up the MAC address for procId: %d\n", coreNum);
             System_flush();
     	}

         /* Clean up for Qmss/CPPI from master core */
     	clearFramework(coreNum);
    }

    if (coreNum == SYSINIT)
    {
    	/* Delete the Shared memory */
    	fw_shmClose();
    	fw_shmDelete();

    #ifdef __LINUX_USER_SPACE
    	/* Delete the Semaphore */
        fw_SemDestroy();
    #endif
    }
	System_printf (" Done...\n");	

    System_printf ("**********************************************\n");
    System_printf ("*** PA Multi Core Example Ended on Core %d ***\n",coreNum);
    System_printf ("**********************************************\n");
    System_flush();

    /* Example application done. Return success */
    APP_exit (0);
}

/* Nothing past this point */

