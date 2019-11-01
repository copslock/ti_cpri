/**  
 * @file pcap_singlecore.c
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
 * 	            (d) PA Subsystem + PDSP - (Note: PDSP is initialized only during NO_BOOT mode)
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
#include <pcap_singlecore.h>
#include <stdio.h>

/**************************************************************
************************** DEFINITIONS ************************
***************************************************************/

/* Counters to track number of packets sent/received by this application */
pktStats_t expected, actual;

/*
 * Default test configuration for the silicon
 *
 * To run test at the CCS simulator
 *    cpswSimTest = 1
 *    cpswLpbkMode = CPSW_LOOPBACK_INTERNAL
 */
#ifdef  SIMULATOR_SUPPORT
int cpswSimTest = 1;
int cpswLpbkMode = CPSW_LOOPBACK_INTERNAL;
#else
int cpswSimTest = 0;
int cpswLpbkMode = CPSW_LOOPBACK_INTERNAL;
#endif

extern void mdebugHaltPdsp (int pdspNum);
volatile int mdebugWait = 1;
uint32_t no_bootMode = TRUE;

/**************************************************************
**************** EXAMPLE APP FUNCTIONS ************************
***************************************************************/

/** ============================================================================
 *   @n@b pCap_SingleCoreApp
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
void pCap_SingleCoreApp (void)
{
	System_printf ("**************************************************\n");
    System_printf ("******* Single Core Packet Capture Unit Test Start *******\n");
    System_printf ("**************************************************\n");
    System_flush();
    int anyTestFailCnt = 0;
    
    if (setupFramework() != 0) {
        System_printf ("Failed to setup test framework \n");
        System_flush();
    }

    System_printf ("-------------------------------------------------------------------------------------------------------------------\n");
    System_printf ("|Test Name\t\t\t|Port ID\t|Pkts Sent\t|Pkts Received\t|Pkts Captured/Mirrored\t|Test Status\n");
    System_printf ("-------------------------------------------------------------------------------------------------------------------\n");
    System_flush();
    /* Run some data through and verify transfer worked */
    for (dest_emac_port_id = 0; dest_emac_port_id < (NUM_MAC_PORTS);  dest_emac_port_id ++)
    {
        /* If the port is not available for test, proceed for next port test */
        if (ports[dest_emac_port_id].pState == NOT_AVAILABLE_FOR_PCAP_TEST)
        	continue;

        /* 1. Ingress Packet Capture test */
        if (pkt_capture_test(TRUE, dest_emac_port_id))
        {
            System_flush();
            anyTestFailCnt ++;
        }
        
        /* 2. Ingress Port Mirror test */
        if (port_mirror_test(TRUE, dest_emac_port_id))
        {
            System_flush();
            anyTestFailCnt ++;
        }

        /* 3. Egress Packet Capture Test */
        if (pkt_capture_test(FALSE, dest_emac_port_id))
        {
            System_flush();
            anyTestFailCnt ++;
        }
        /* 4. Egress port Mirror Test */
        if (port_mirror_test(FALSE, dest_emac_port_id))
        {
            System_flush();
            anyTestFailCnt ++;
        }
    }

    /* 5. Egress Packet Capture Test for CPPI PORT (PORT 0)*/
    dest_emac_port_id = -1;
    if (pkt_capture_test(FALSE, dest_emac_port_id))
    {
        System_printf ("Egress Packet Capture test failed for port # %d", dest_emac_port_id + pa_EMAC_PORT_0);
        System_flush();
        anyTestFailCnt ++;
    }

    System_printf ("-------------------------------------------------------------------------------------------------------------------\n");

    System_printf (" \nExample Done! \nPA Stats After Packet Transmission BEGIN ********* ... \n");
    if (getPaStats ())  {
        System_printf ("Function getPaStats failed\n");
        System_flush();
    }

    System_printf ("**************************************************\n");
    System_printf ("******** Packet Capture Single Core Example End ********\n");
    System_printf ("**************************************************\n");
    System_flush();

    /* Clear framework */
	if (clearFramework() < 0) 
	{
        System_printf ("Failed to Clean the example application \n");
        System_flush();
	}

#if (RM) && !defined(__LINUX_USER_SPACE)
    {
        int32_t rmResult;

        if ((rmResult = Rm_resourceStatus(rmHandle, FALSE)) != 0)
        {
            System_printf ("Error : Number of unfreed resources : %d\n", rmResult);
            System_flush();
        }
        else
        {
            System_printf ("All resources freed successfully\n");
            System_flush();
        }
    }
#endif
    /* Print all tests passed */
    if (anyTestFailCnt == 0)
    	System_printf ("All tests passed \n");
    else
    	System_printf ("Few tests failed \n");
    /* Example application done. Return success */
    APP_exit (0);
    
}

/* Nothing past this point */

