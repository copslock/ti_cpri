/******************************************************************************
 * FILE PURPOSE:  Functions to initialize framework resources for running NWAL
 ******************************************************************************
 * FILE NAME:   fw_soc.c
 *
 * DESCRIPTION: Functions to initialize framework resources for running NWAL
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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
#include "fw_test.h"

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_cpsw.h>
#include <ti/csl/csl_cpsgmiiAux.h>
#include <c6x.h>
#include <stdlib.h>
#include <stdio.h>

Void CycleDelay (int32_t count)
{
    UInt32                  TSCLin;

    if (count <= 0)
        return;

    /* Get the current TSCL  */
    TSCLin = TSCL ;

    while ((TSCL - TSCLin) < (UInt32)count);
}

/***************************************************************************************
 * FUNCTION PURPOSE: Initialize SGMII
 ***************************************************************************************
 * DESCRIPTION: Add switch related initializations
 ***************************************************************************************/
int32_t Init_SGMII (UInt32 macPortNum)
{  
    CSL_SGMII_ADVABILITY    sgmiiCfg;
	CSL_SGMII_STATUS        sgmiiStatus;
    
    /* Configure SGMII Port 1 only since it is connected to RJ45 at all known EVMs */
    if(macPortNum == 1)
    {
        /* Reset the port before configuring it */
        CSL_SGMII_doSoftReset (macPortNum);        
        while (CSL_SGMII_getSoftResetStatus (macPortNum) != 0);   

        /* Hold the port in soft reset and set up
        * the SGMII control register:
        *      (1) Enable Master Mode (default)
        *      (2) Enable Auto-negotiation
        */
        CSL_SGMII_startRxTxSoftReset (macPortNum); 
        CSL_SGMII_disableMasterMode (macPortNum);
       
    
	    /* Setup the Advertised Ability register for this port:
        *      (1) Enable Full duplex mode
        *      (2) Enable Auto Negotiation
        */
        sgmiiCfg.linkSpeed      =   CSL_SGMII_1000_MBPS;
        sgmiiCfg.duplexMode     =   CSL_SGMII_FULL_DUPLEX;
        CSL_SGMII_setAdvAbility (macPortNum, &sgmiiCfg);
    
        CSL_SGMII_enableAutoNegotiation (macPortNum);
        CSL_SGMII_endRxTxSoftReset (macPortNum);   
       
        do
        {
            CSL_SGMII_getStatus(macPortNum, &sgmiiStatus);
        } while (sgmiiStatus.bIsLinkUp != 1);
        
        /* Wait for SGMII Autonegotiation to complete without error */
        do
        {
            CSL_SGMII_getStatus(macPortNum, &sgmiiStatus);
            if (sgmiiStatus.bIsAutoNegError != 0)
                return -1;
        } while (sgmiiStatus.bIsAutoNegComplete != 1);
        
        /* 
         * May need to wait some more time for the external PHY to be ready to transmit packets reliabily.
         * It is possible to access the PHY status register through the MDIO interface to check when 
         * the PHY is ready.
         * To avoid platform-dependent code, we just introduce about 2ms wait here
         */ 
         CycleDelay(2000000);
    }
    

    /* All done with configuration. Return Now. */
    return 0;
}

/***************************************************************************************
 * FUNCTION PURPOSE: Do switch related initializations
 ***************************************************************************************
 * DESCRIPTION: Add switch related initializations
 ***************************************************************************************/
void testNwCPSWInit(nwal_Bool_t enableALE)
#if 1
{  
    CSL_CPSW_ALE_PORTCONTROL        alePortControlCfg;
    
    CSL_CPSW_clearAleTable();
    
    alePortControlCfg.dropUntaggedEnable    =   0;
    alePortControlCfg.vidIngressCheckEnable =   0;
    
    alePortControlCfg.mcastLimit            =   0;
    alePortControlCfg.bcastLimit            =   0;

    /* Disable learning mode for Port 0 */
    alePortControlCfg.noLearnModeEnable     =   1;
    alePortControlCfg.portState     =   ALE_PORTSTATE_FORWARD;
    CSL_CPSW_setAlePortControlReg (0, &alePortControlCfg);  

    /* Enable learning mode for Port 1 */
    alePortControlCfg.noLearnModeEnable     =   0;
    alePortControlCfg.portState     =   ALE_PORTSTATE_FORWARD;
    CSL_CPSW_setAlePortControlReg (1, &alePortControlCfg);

    /* Enable learning mode for Port 2 */
    alePortControlCfg.noLearnModeEnable     =   0;
    alePortControlCfg.portState     =   ALE_PORTSTATE_FORWARD;
    CSL_CPSW_setAlePortControlReg (2, &alePortControlCfg);
}
#else
{  
    CSL_CPSW_enableAleBypass();
    Init_SGMII(1);
#if 0
    CSL_CPSW_ALE_PORTCONTROL        alePortControlCfg;
    
    CSL_CPSW_clearAleTable();
    
    alePortControlCfg.dropUntaggedEnable    =   0;
    alePortControlCfg.vidIngressCheckEnable =   0;
    
    alePortControlCfg.mcastLimit            =   0;
    alePortControlCfg.bcastLimit            =   0;

    /* Disable learning mode for Port 0 */
    alePortControlCfg.noLearnModeEnable     =   1;
    alePortControlCfg.portState     =   ALE_PORTSTATE_FORWARD;
    CSL_CPSW_setAlePortControlReg (0, &alePortControlCfg);  

    if(enableALE)
    {
        alePortControlCfg.noLearnModeEnable     =   0;
    }
    else
    {
        /* Disable learning mode for Port 1 */
        alePortControlCfg.noLearnModeEnable     =   1;
    }
    alePortControlCfg.portState     =   ALE_PORTSTATE_FORWARD;
    CSL_CPSW_setAlePortControlReg (1, &alePortControlCfg);

    /* Disable learning mode for Port 2 */
    if(enableALE)
    {
        alePortControlCfg.noLearnModeEnable     =   0;
    }
    else
    {
        /* Disable learning mode for Port 1 */
        alePortControlCfg.noLearnModeEnable     =   1;
    }
    alePortControlCfg.portState     =   ALE_PORTSTATE_FORWARD;
    CSL_CPSW_setAlePortControlReg (2, &alePortControlCfg);
    #endif
}
#endif
/***************************************************************************************
 * FUNCTION PURPOSE: Add MAC Address to the ALE for the packet being routed from network
 ***************************************************************************************
 * DESCRIPTION: Add MAC Address to the ALE for the packet being routed from network
 ***************************************************************************************/
nwal_Bool_t testNwSwUpdateMacAddr(UInt8 macAddress[6])
{

    Uint32                              i;
    CSL_CPSW_ALE_UNICASTADDR_ENTRY  ucastAddrCfg;
    /* Program the ALE with the MAC address.
    *
    * The ALE entries determine the switch port to which any
    * matching received packet must be forwarded to.
    */
    /* Get the next free ALE entry to program */
    for (i = 0; i < CSL_CPSW_NUMALE_ENTRIES; i++)
    {
        if (CSL_CPSW_getALEEntryType (i) == ALE_ENTRYTYPE_FREE)
        {
            /* Found a free entry */
            break;                    
        }
    }
    if (i == CSL_CPSW_NUMALE_ENTRIES)
    {
        /* No free ALE entry found. return error. */    
        System_printf ("ERROR: No Free Entry in CPSW ALE table \n");
        return 0;            
    }
    else
    {
        /* Found a free ALE entry to program our MAC address */            
        memcpy (ucastAddrCfg.macAddress, macAddress, 6);    /* Set the MAC address */
        ucastAddrCfg.ucastType      =      ALE_UCASTTYPE_UCAST_NOAGE;  /* Add a permanent unicast address entry */
        ucastAddrCfg.secureEnable   =      nwal_FALSE;   
        ucastAddrCfg.blockEnable    =      nwal_FALSE;   
        ucastAddrCfg.portNumber     =      0;   /* DSP CPPI Port */

        /* Setup the ALE entry for this port's MAC address */
        CSL_CPSW_setAleUnicastAddrEntry (i, &ucastAddrCfg);
    }

    /* Done with upading address */
    return 1;

}
