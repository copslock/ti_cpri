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

#define         NUM_MAC_PORTS               2u

void CycleDelay (int32_t count)
{
    uint32_t                  TSCLin;

    if (count <= 0)
        return;

    /* Get the current TSCL  */
    TSCLin = TSCL ;

    while ((TSCL - TSCLin) < (UInt32)count);
}

#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
/**********************************************************************************
 * FUNCTION PURPOSE: Function for updating register value with mask
 **********************************************************************************
 * DESCRIPTION: Function for updating register value with mask
 **********************************************************************************/
static void write_32_mask(volatile uint32_t  base_addr,
                              uint32_t           offset,
                              uint32_t           mask,
                              uint32_t           write_data)
{
    uint32_t read_data, data;
    read_data = (*(volatile uint32_t *)(base_addr + offset));
    data = (write_data & ~mask ) | (read_data & mask);
    (*(uint32_t *) ((base_addr + offset))) = data;
}


/**********************************************************************************
 * FUNCTION PURPOSE: SB SERDES Config. Currently handling only 156.25 Mhz
 **********************************************************************************
 * DESCRIPTION: SB SERDES Config.Currently handling only 156.25 Mhz
 **********************************************************************************/
int16_t sgmiiDefSerdesSetup()
{
    uint32_t            count;
    CSL_SGMII_STATUS    sgmiiStatus;

    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x000,0x0000FFFF,0x00800000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x014,0xFFFF0000,0x00008282);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x060,0xFF000000,0x00142438);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x064,0xFF0000FF,0x00C3C700);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x078,0xFFFF00FF,0x0000C000);   

    /* LANE # 1 */
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x204,0x00FFFF00,0x38000080);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x208,0xFFFFFF00,0x00000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x20C,0x00FFFFFF,0x02000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x210,0x00FFFFFF,0x1B000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x214,0xFFFF0000,0x00006FB8);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x218,0x0000FF00,0x758000E4);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x2AC,0xFFFF00FF,0x00004400);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x22C,0xFF0000FF,0x00300800);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x280,0xFF00FF00,0x00820082);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x284,0x00000000,0x1D0F0385);

    /* LANE # 2 */
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x404,0x00FFFF00,0x38000080);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x408,0xFFFFFF00,0x00000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x40C,0x00FFFFFF,0x02000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x410,0x00FFFFFF,0x1B000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x414,0xFFFF0000,0x00006FB8);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x418,0x0000FF00,0x758000E4);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x4AC,0xFFFF00FF,0x00004400);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x42C,0xFF0000FF,0x00300800);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x480,0xFF00FF00,0x00820082);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x484,0x00000000,0x1D0F0385);

    /* LANE # 3 */
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x604,0x00FFFF00,0x38000080);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x608,0xFFFFFF00,0x00000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x60C,0x00FFFFFF,0x02000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x610,0x00FFFFFF,0x1B000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x614,0xFFFF0000,0x00006FB8);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x618,0x0000FF00,0x758000E4);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x6AC,0xFFFF00FF,0x00004400);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x62C,0xFF0000FF,0x00300800);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x680,0xFF00FF00,0x00820082);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x684,0x00000000,0x1D0F0385);

    /* LANE # 4 */
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x804,0x00FFFF00,0x38000080);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x808,0xFFFFFF00,0x00000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x80C,0x00FFFFFF,0x02000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x810,0x00FFFFFF,0x1B000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x814,0xFFFF0000,0x00006FB8);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x818,0x0000FF00,0x758000E4);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x8AC,0xFFFF00FF,0x00004400);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x82C,0xFF0000FF,0x00300800);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x880,0xFF00FF00,0x00820082);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x884,0x00000000,0x1D0F0385);

    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xa00,0xFFFF00FF,0x00000800);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xa08,0x0000FFFF,0X38A20000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xa30,0xFF0000FF,0x008A8A00);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xa84,0xFFFF00FF,0x00000600);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xa94,0x00FFFFFF,0x10000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xaa0,0x00FFFFFF,0x81000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xabc,0x00FFFFFF,0xFF000000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xac0,0xFFFFFF00,0x0000008B);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xb08,0x0000FFFF,0x583F0000);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xb0c,0xFFFFFF00,0x0000004e);
    
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x000,0xFFFFFF00,0x00000003);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0xa00,0xFFFFFF00,0x0000005F);

    /* Enable TX and RX via the LANExCTL_STS 0x0000 + x*4 */
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x1fe0,0x00000000,0xF800F8C0);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x1fe4,0x00000000,0xF800F8C0);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x1fe8,0x00000000,0xF800F8C0);
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x1fec,0x00000000,0xF800F8C0);

    /*Enable pll via the pll_ctrl 0x0014*/
    write_32_mask(CSL_NETCP_SERDES_CFG_REGS, 0x1ff4,0x00000000,0xe0000000);

    for(count = 0; count < NUM_MAC_PORTS; count++)
    {
        do
        { 
            CSL_SGMII_getStatus(count, &sgmiiStatus);
        } while (sgmiiStatus.bIsLocked != 1);
    }

    for(count = 0; count < 40000; count++);
    CycleDelay(40000);
    return(-1);
}
#endif

/** ============================================================================
 *   @n@b Init_SGMII
 *
 *   @b Description
 *   @n SGMII peripheral initialization code.
 *
 *   @param[in]  
 *   @n macPortNum      MAC port number for which the SGMII port setup must
 *                      be performed.
 * 
 *   @return
 *   @n None
 * =============================================================================
 */
Int32 Init_SGMII (UInt32 macPortNum)
{  
    CSL_SGMII_ADVABILITY    sgmiiCfg;
    CSL_SGMII_STATUS        sgmiiStatus;
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2L) && !defined(DEVICE_K2E) && \
    !defined(SOC_K2K) && !defined(SOC_K2H)  && !defined(SOC_K2L)  && !defined(SOC_K2E)
    /* Configure SGMII Port 1 only since it is connected to RJ45 at all known EVMs */
    if(cpswSimTest || (macPortNum == 1))
    {
#endif
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
       //CSL_SGMII_enableMasterMode (macPortNum);
        
        /* Setup the Advertised Ability register for this port:
        *      (1) Enable Full duplex mode
        *      (2) Enable Auto Negotiation
        */
        sgmiiCfg.linkSpeed      =   CSL_SGMII_1000_MBPS;
        sgmiiCfg.duplexMode     =   CSL_SGMII_FULL_DUPLEX;
        CSL_SGMII_setAdvAbility (macPortNum, &sgmiiCfg);

        CSL_SGMII_enableAutoNegotiation (macPortNum);
        CSL_SGMII_endRxTxSoftReset (macPortNum); 
    
            /* Wait for SGMII Link */
        
        do
        {
            CSL_SGMII_getStatus(macPortNum, &sgmiiStatus);
        } while (sgmiiStatus.bIsLinkUp != 1);

        /* Wait for SGMII Autonegotiation to complete without error */

#ifndef  SIMULATOR_SUPPORT
        do
        {
            CSL_SGMII_getStatus(macPortNum, &sgmiiStatus);
            if (sgmiiStatus.bIsAutoNegError != 0)
                return -1;
        } while (sgmiiStatus.bIsAutoNegComplete != 1);
#endif

        /* 
         * May need to wait some more time for the external PHY to be ready to transmit packets reliabily.
         * It is possible to access the PHY status register through the MDIO interface to check when 
         * the PHY is ready.
         * To avoid platform-dependent code, we just introduce about 2ms wait here
         */ 
         CycleDelay(2000000);
        
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2L) && !defined(DEVICE_K2E) && \
    !defined(SOC_K2K) && !defined(SOC_K2H)  && !defined(SOC_K2L)  && !defined(SOC_K2E)
    }
#endif
    /* All done with configuration. Return Now. */
    return 0;
}

int Init_MAC (UInt32 macPortNum, UInt8 macAddress[6], UInt32 mtu)
{

    /* Reset MAC Sliver 0 */            
    CSL_CPGMAC_SL_resetMac (macPortNum);
    while (CSL_CPGMAC_SL_isMACResetDone (macPortNum) != TRUE);

    /* Setup the MAC Control Register for this port:
     *      (1) Enable Full duplex
     *      (2) Enable GMII
     *      (3) Enable Gigabit 
     *      (4) Enable External Configuration. This enables 
     *          the "Full duplex" and "Gigabit" settings to be
     *          controlled externally from SGMII
     *      (5) Don't enable any control/error/short frames
     */
    CSL_CPGMAC_SL_enableFullDuplex (macPortNum);
    CSL_CPGMAC_SL_enableGMII (macPortNum);
    CSL_CPGMAC_SL_enableGigabit (macPortNum);
    CSL_CPGMAC_SL_enableExtControl (macPortNum);

    /* Configure the MAC address for this port */
    CSL_CPSW_setPortMACAddress (macPortNum, macAddress);

    /* Configure VLAN ID/CFI/Priority.
     *
     * For now, we are not using VLANs so just configure them
     * to all zeros.
     */
    CSL_CPSW_setPortVlanReg (macPortNum, 0, 0, 0);

    /* Configure the Receive Maximum length on this port,
     * i.e., the maximum size the port can receive without
     * any errors.
     *
     * Set the Rx Max length to the MTU configured for the
     * interface.
     */
    CSL_CPGMAC_SL_setRxMaxLen (macPortNum, mtu);

    /* Done setting up the MAC port */
    return 0;
}

Void Init_Switch (UInt32 mtu)
{
    CSL_CPSW_PORTSTAT               portStatCfg;

    /* Enable the CPPI port, i.e., port 0 that does all 
     * the data streaming in/out of EMAC.
     */
    CSL_CPSW_enablePort0 ();
    CSL_CPSW_disableVlanAware ();
    CSL_CPSW_setPort0VlanReg (0, 0, 0);
    CSL_CPSW_setPort0RxMaxLen (mtu);

    /* Enable statistics on both the port groups:
     *
     * MAC Sliver ports -   Port 1, Port 2
     * CPPI Port        -   Port 0
     */
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    portStatCfg.p0AStatEnable   =   1;
    portStatCfg.p0BStatEnable   =   1;
    portStatCfg.p1StatEnable    =   1;
    portStatCfg.p2StatEnable    =   1;
#else
    portStatCfg.p0StatEnable    =   1;
    portStatCfg.p1StatEnable    =   1;
    portStatCfg.p2StatEnable    =   1;
    portStatCfg.p3StatEnable    =   1;
    portStatCfg.p4StatEnable    =   1;
    portStatCfg.p5StatEnable    =   1;
    portStatCfg.p6StatEnable    =   1;
    portStatCfg.p7StatEnable    =   1;
    portStatCfg.p8StatEnable    =   1;
#endif
    CSL_CPSW_setPortStatsEnableReg (&portStatCfg);

    /* Setup the Address Lookup Engine (ALE) Configuration:
     *      (1) Enable ALE.
     *      (2) Clear stale ALE entries.
     *      (3) Disable VLAN Aware lookups in ALE since
     *          we are not using VLANs by default.
     *      (4) No Flow control
     *      (5) Configure the Unknown VLAN processing 
     *          properties for the switch, i.e., which 
     *          ports to send the packets to.
     */
    CSL_CPSW_enableAle ();
    CSL_CPSW_clearAleTable ();
    
    CSL_CPSW_disableAleVlanAware ();
    CSL_CPSW_disableAleTxRateLimit ();
    CSL_CPSW_setAlePrescaleReg (125000000u/1000u);
    CSL_CPSW_setAleUnkownVlanReg (7, 3, 3, 7);
   

     /* Done with switch configuration */
    return;
}
int Switch_update_addr (Uint32 portNum, UInt8 macAddress[6], Uint16 add)
{
    CSL_CPSW_ALE_PORTCONTROL        alePortControlCfg;


    /* Configure the address in "Learning"/"Forward" state */
    alePortControlCfg.portState             =   ALE_PORTSTATE_FORWARD;
    alePortControlCfg.dropUntaggedEnable    =   0;
    alePortControlCfg.vidIngressCheckEnable =   0;
    alePortControlCfg.noLearnModeEnable     =   0;
    alePortControlCfg.mcastLimit            =   0;
    alePortControlCfg.bcastLimit            =   0;

    CSL_CPSW_setAlePortControlReg(portNum, &alePortControlCfg);
    

    /* Done with upading address */
    return 0;
}
/***************************************************************************************
 * FUNCTION PURPOSE: Do switch related initializations
 ***************************************************************************************
 * DESCRIPTION: Add switch related initializations
 ***************************************************************************************/
void testNwCPSWInit(nwal_Bool_t enableALE)
{  
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    uint8_t        macSrcAddress [2][6] =  {{0x20, 0x21, 0x22, 0x23, 0x24, 0x25},
                                            {0x00, 0x01, 0x02, 0x03, 0x04, 0x05}};
    uint32_t       macPortNum, mtu = 1600;
    uint8_t        macAddress0 [6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};  /* MAC address for (CPPI) Port 0 */ 
    uint8_t        macAddress1 [6] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25};  /* MAC address for (EMAC1) Port 1 */ 
    uint8_t        macAddress2 [6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};  /* MAC address for (EMAC2) Port 2 */

    
    sgmiiDefSerdesSetup();
    for (macPortNum = 0; macPortNum < NUM_MAC_PORTS; macPortNum++)
    {
    	if (Init_SGMII (macPortNum))
		  while(1);
          
        Init_MAC (macPortNum, &macSrcAddress[macPortNum][0], mtu);
    }
     /* Setup the Ethernet switch finally. */
    Init_Switch (mtu);
    Switch_update_addr(0, macAddress0, 0);
    Switch_update_addr(1, macAddress1, 0);
    Switch_update_addr(2, macAddress2, 0);
    CSL_CPSW_enableAleBypass();
#endif

}

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
