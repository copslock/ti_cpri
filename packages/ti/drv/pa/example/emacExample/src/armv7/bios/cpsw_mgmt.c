/**
 * @file cpsw_mgmt.c
 *
 * @brief
 *  This file holds all the Ethernet subsystem (CPSW + MDIO + SGMII) components
 *  initialization and setup code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2014, Texas Instruments, Inc.
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
/* C Standard library Include */
#include <string.h>

/* Chip Level definitions include */
#include <ti/csl/csl_chip.h>

/* CSL EMAC include */
#include <ti/csl/csl_cpsw.h>
#include <ti/csl/csl_cpsgmii.h>
#include <ti/csl/csl_cpsgmiiAux.h>
#include <ti/csl/csl_mdio.h>
#include <ti/csl/csl_mdioAux.h>

/* BootCfg module include */
#include <ti/csl/csl_bootcfg.h>
#include <ti/csl/csl_bootcfgAux.h>

#include <cpsw_mgmt.h>
#include <ti/csl/csl_serdes_ethernet.h>

typedef uint32_t csl_serdes_refclk_t;
#define SERDES_REF_CLK_156250_KHZ       156250
#if defined(SOC_K2K) || defined(SOC_K2H)
#define PA_EMAC_EXAMPLE_REF_CLK_KHZ     SERDES_REF_CLK_156250_KHZ
#endif

uint32_t gNum_Mac_Ports;

void cpsw_getStats(CSL_CPSW_STATS*   stats, int clear)
{

   int numBlocks;
   CSL_CPSW_STATS* pStats = stats;

   CSL_CPSW_getStats(stats);
   for (numBlocks = 0; numBlocks < CSL_CPSW_NUMSTATBLOCKS; numBlocks++)
   {
	   System_printf ("Stats for block number: %d \n", numBlocks);
	   System_printf ("********************************************\n");
	   System_printf("	Good Frames Received                      %d\n", stats->RxGoodFrames);

	   System_printf("	Good Broadcast Frames Received            %d\n", stats->RxBCastFrames);

	   System_printf("	Good Multicast Frames Received            %d\n", stats->RxMCastFrames);

	   System_printf("	PauseRx Frames Received                   %d\n", stats->RxPauseFrames);

	   System_printf("	Frames Received with CRC Errors           %d\n", stats->RxCRCErrors);

	   System_printf("	Frames Received with Alignment/Code Errors%d\n", stats->RxAlignCodeErrors);

	   System_printf("	Oversized Frames Received                 %d\n", stats->RxOversized);

	   System_printf("	Jabber Frames Received                    %d\n", stats->RxJabber);

	   System_printf("	Undersized Frames Received                %d\n", stats->RxUndersized);

	   System_printf("	Rx Frame Fragments Received               %d\n", stats->RxFragments);

	   System_printf("	Total Received Bytes in Good Frames       %d\n", stats->RxOctets);

	   System_printf("	Good Frames Sent                          %d\n", stats->TxGoodFrames);

	   System_printf("	Good Broadcast Frames Sent                %d\n", stats->TxBCastFrames);

	   System_printf("	Good Multicast Frames Sent                %d\n", stats->TxMCastFrames);

	   System_printf("	PauseTx Frames Sent                       %d\n", stats->TxPauseFrames);

	   System_printf("	Frames Where Transmission was Deferred    %d\n", stats->TxDeferred);

	   System_printf("	Total Frames Sent With Collision          %d\n", stats->TxCollision);

	   System_printf("	Frames Sent with Exactly One Collision    %d\n", stats->TxSingleColl);

	   System_printf("	Frames Sent with Multiple Colisions       %d\n", stats->TxMultiColl);

	   System_printf("	Tx Frames Lost Due to Excessive Collisions%d\n", stats->TxExcessiveColl);

	   System_printf("	Tx Frames Lost Due to a Late Collision    %d\n", stats->TxLateColl);

	   System_printf("	Tx Frames Lost Due to Carrier Sense Loss  %d\n", stats->TxCarrierSLoss);

	   System_printf("	Total Transmitted Bytes in Good Frames    %d\n", stats->TxOctets);

	   System_printf("	Total Tx&Rx with Octet Size of 64         %d\n", stats->Frame64);

	   System_printf("	Total Tx&Rx with Octet Size of 65 to 127  %d\n", stats->Frame65t127);

	   System_printf("	Total Tx&Rx with Octet Size of 128 to 255 %d\n", stats->Frame128t255);

	   System_printf("	Total Tx&Rx with Octet Size of 256 to 511 %d\n", stats->Frame256t511);

	   System_printf("	Total Tx&Rx with Octet Size of 512 to 1023 %d\n",  stats->Frame512t1023);

	   System_printf("	Total Tx&Rx with Octet Size of >=1024     %d\n", stats->Frame1024tUp);

	   System_printf("	Sum of all Octets Tx or Rx on the Network %d\n", stats->NetOctets);

	   System_printf ("********************************************\n");
       System_flush();
	   stats++;
   }

   if (clear)
   {
        memset(pStats, 0, sizeof(CSL_CPSW_STATS)*CSL_CPSW_NUMSTATBLOCKS);
   }

}
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
#if !defined(SOC_K2K) && !defined(SOC_K2H)  && !defined(SOC_K2L)  && !defined(SOC_K2E)
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
        if (cpswLpbkMode == CPSW_LOOPBACK_NONE)
        {
            CSL_SGMII_disableMasterMode (macPortNum);
        }
        else
        {
            CSL_SGMII_enableMasterMode (macPortNum);

            if (cpswLpbkMode == CPSW_LOOPBACK_INTERNAL)
            {
                CSL_SGMII_enableLoopback (macPortNum);
            }
        }

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
        if (!cpswSimTest)
        {
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
            if((cpswLpbkMode == CPSW_LOOPBACK_EXTERNAL) || (cpswLpbkMode == CPSW_LOOPBACK_NONE))
                CycleDelay(2000000);
        }
#if !defined(SOC_K2K) && !defined(SOC_K2H)  && !defined(SOC_K2L)  && !defined(SOC_K2E)
    }
#endif
    /* All done with configuration. Return Now. */
    return 0;
}

/** ============================================================================
 *   @n@b Init_MAC
 *
 *   @b Description
 *   @n This API initializes the CPGMAC Sliver (MAC Port) port.
 *
 *   @param[in]
 *   @n macPortNum      MAC port number for which the initialization must be done.
 *
 *   @param[in]
 *   @n macAddress      MAC address to configure on this port.
 *
 *   @param[in]
 *   @n mtu             Maximum Frame length to configure on this port.
 *
 *   @return
 *   @n None
 * =============================================================================
 */
int Init_MAC (UInt32 macPortNum, UInt8 macAddress[6], UInt32 mtu)
{

#ifdef NSS_GEN2
    CSL_CPSW_TSCNTL  tsCtrl;

    memset(&tsCtrl, 0, sizeof(CSL_CPSW_TSCNTL));
#endif


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

#ifdef NSS_GEN2
    /*
     * Enable Port Time sync transmit host timestamp
     */
    tsCtrl.tsTxHostEnable = 1;
    tsCtrl.tsMsgTypeEnable = 0xFFFF;

    CSL_CPSW_setPortTimeSyncCntlReg (macPortNum+1, &tsCtrl);
#endif

    /* Done setting up the MAC port */
    return 0;
}

/** ============================================================================
 *   @n@b Init_MDIO
 *
 *   @b Description
 *   @n Not supported at moment. MDIO is not simulated yet.
 *
 *   @param[in]
 *   @n None
 *
 *   @return
 *   @n None
 * =============================================================================
 */
Void Init_MDIO (Void)
{
    /* Return success. */
    return;
}

/** ============================================================================
 *   @n@b Init_CPTS
 *
 *   @b Description
 *   @n Init and configure CPTS module
 *
 *   @param[in]
 *   @n None
 *
 *   @return
 *   @n None
 * =============================================================================
 */
Void Init_CPTS (Void)
{
#ifdef NSS_GEN2
    CSL_CPTS_CONTROL    ctrl;
    uint32_t            refClockSelect = 0;

    memset(&ctrl, 0, sizeof(CSL_CPTS_CONTROL));
    ctrl.cptsEn = 1;
    ctrl.tstampEn = 1;
    ctrl.seqEn = 0;
    ctrl.ts64bMode = 1;

    CSL_CPTS_disableCpts();

    CSL_CPTS_setRFTCLKSelectReg (refClockSelect);

    CSL_CPTS_setCntlReg(&ctrl);

#endif

    /* Return success. */
    return;
}


/** ============================================================================
 *   @n@b Init_Switch
 *
 *   @b Description
 *   @n This API sets up the ethernet switch subsystem and its Address Lookup
 *      Engine (ALE) in "Switch" mode.
 *
 *   @param[in]
 *   @n mtu             Maximum Frame length to configure on the switch.
 *
 *   @return
 *   @n None
 * =============================================================================
 */
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
    #if defined(SOC_K2K) || defined(SOC_K2H)
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

    if(cpswLpbkMode != CPSW_LOOPBACK_NONE)
        CSL_CPSW_enableAleBypass();

    /* Done with switch configuration */
    return;
}


/** ============================================================================
 *   @n@b Switch_update_addr
 *
 *   @b Description
 *   @n This API add/delete entries in the Address Lookup Engine (ALE) in "Switch" mode.
 *
 *   @param[in]
 *   @n portNum         Switch port number.

 *   @param[in]
 *   @n macAddress      MAC address to configure on the switch.
 *
 *   @param[in]
 *   @n add             0:add; 1:delete.
 *
 *   @return
 *   @n None
 *
 *   @Note  It supports "add" operation only now.
 * =============================================================================
 */
int Switch_update_addr (Uint32 portNum, UInt8 macAddress[6], Uint16 add)
{
    Uint32                              i;
    CSL_CPSW_ALE_PORTCONTROL        alePortControlCfg;
    CSL_CPSW_ALE_UNICASTADDR_ENTRY  ucastAddrCfg;


    /* Configure the address in "Learning"/"Forward" state */
    alePortControlCfg.portState             =   ALE_PORTSTATE_FORWARD;
    alePortControlCfg.dropUntaggedEnable    =   0;
    alePortControlCfg.vidIngressCheckEnable =   0;
    alePortControlCfg.noLearnModeEnable     =   (cpswLpbkMode != CPSW_LOOPBACK_NONE)?1:0;
    alePortControlCfg.mcastLimit            =   0;
    alePortControlCfg.bcastLimit            =   0;

    CSL_CPSW_setAlePortControlReg (portNum, &alePortControlCfg);

    /*
     * The following code is required for device simulator only.
     * It is also served as an example of adding MAC address to the ALE table manually
     */

    if (cpswSimTest)
    {
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
            return -1;
        }
        else
        {
            /* Found a free ALE entry to program our MAC address */
            memcpy (ucastAddrCfg.macAddress, macAddress, 6);    // Set the MAC address
            ucastAddrCfg.ucastType      =      ALE_UCASTTYPE_UCAST_NOAGE;   // Add a permanent unicast address entryALE_UCASTTYPE_UCAST_NOAGE.
            ucastAddrCfg.secureEnable   =      FALSE;
            ucastAddrCfg.blockEnable    =      FALSE;
            ucastAddrCfg.portNumber     =      portNum;   // Add the ALE entry for this port

            /* Setup the ALE entry for this port's MAC address */
            CSL_CPSW_setAleUnicastAddrEntry (i, &ucastAddrCfg);
        }
    }

    /* Done with upading address */
    return 0;
}


/** ============================================================================
 *   @n@b Init_SGMII_SERDES
 *
 *   @b Description
 *   @n This API sets up the configuration for the SGMII SERDES. Assumes a 125 MHz
 *       reference clock.
 *
 *   @param[in]
 *   @n None
 *
 *   @return
 *   @n None
 * =============================================================================
 */
Int32 Init_SGMII_SERDES(Void)
{

    CSL_SERDES_LOOPBACK lpbk_mode = (cpswLpbkMode == CPSW_LOOPBACK_SERDES || cpswLpbkMode == CPSW_LOOPBACK_INTERNAL)?CSL_SERDES_LOOPBACK_ENABLED:CSL_SERDES_LOOPBACK_DISABLED;

    if(!cpswSimTest)
    {
#if defined(SOC_K2K) || defined(SOC_K2H)
        CSL_SERDES_RESULT   csl_retval;
        CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
        CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params;
        uint32_t i;

        memset(&serdes_lane_enable_params, 0, sizeof(serdes_lane_enable_params));

        serdes_lane_enable_params.base_addr = CSL_NETCP_SERDES_CFG_REGS;
        serdes_lane_enable_params.ref_clock = CSL_SERDES_REF_CLOCK_156p25M;
        serdes_lane_enable_params.linkrate = CSL_SERDES_LINK_RATE_1p25G;
        serdes_lane_enable_params.num_lanes = gNum_Mac_Ports;
        serdes_lane_enable_params.phy_type = SERDES_SGMII;
        for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
        {
            serdes_lane_enable_params.lane_ctrl_rate[i] = CSL_SERDES_LANE_QUARTER_RATE;
            serdes_lane_enable_params.loopback_mode[i] = lpbk_mode;

            /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
            serdes_lane_enable_params.rx_coeff.att_start[i] = 7;
            serdes_lane_enable_params.rx_coeff.boost_start[i] = 5;

            /* For higher speeds PHY-A, force attenuation and boost values  */
            serdes_lane_enable_params.rx_coeff.force_att_val[i] = 1;
            serdes_lane_enable_params.rx_coeff.force_boost_val[i] = 1;

            /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
            serdes_lane_enable_params.tx_coeff.cm_coeff[i] = 0;
            serdes_lane_enable_params.tx_coeff.c1_coeff[i] = 0;
            serdes_lane_enable_params.tx_coeff.c2_coeff[i] = 0;
            serdes_lane_enable_params.tx_coeff.tx_att[i] = 12;
            serdes_lane_enable_params.tx_coeff.tx_vreg[i] = 4;
        }

        /* if the system has 4 lanes, lane mask = 0xF. If the system has 2 lanes, lane mask = 0x3 etc */
        serdes_lane_enable_params.lane_mask = (1 << serdes_lane_enable_params.num_lanes) - 1;
    	serdes_lane_enable_params.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
        /* Att and Boost values are obtained through Serdes Diagnostic PRBS calibration test */
        /* For higher speeds PHY-A, force attenuation and boost values  */
        serdes_lane_enable_params.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

        /* CMU, COMLANE, and Lane Setup  */
        csl_retval = CSL_EthernetSerdesInit(serdes_lane_enable_params.base_addr,
                                            serdes_lane_enable_params.ref_clock,
                                            serdes_lane_enable_params.linkrate);

        if (csl_retval != 0)
        {
            System_printf ("Invalid Serdes Init Params\n");
        }

        /* Common Init Mode */
        /* Iteration Mode needs to be set to Common Init Mode first */
        serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
        if (lane_retval != 0)
        {
            System_printf ("Invalid Serdes Common Init\n");
            exit(0);
        }
        System_printf("SGMII Serdes Common Init Complete\n");

        /* Lane Init Mode */
        /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be enabled by setting
        iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT */
        serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
        if (lane_retval != 0)
        {
            System_printf ("Invalid Serdes Lane Enable Init\n");
            exit(0);
        }
        System_printf("SGMII Serdes Lanes Init Complete\n");


#elif defined(SOC_K2E)
        uint32_t i;
        CSL_SERDES_RESULT   csl_retval;
        CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
        int numPort1 = (gNum_Mac_Ports > 4)?4:gNum_Mac_Ports;
        int numPort2 = (gNum_Mac_Ports > 4)?gNum_Mac_Ports - 4:0;

        CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params1, serdes_lane_enable_params2;

        memset(&serdes_lane_enable_params1, 0, sizeof(serdes_lane_enable_params1));
        memset(&serdes_lane_enable_params2, 0, sizeof(serdes_lane_enable_params2));

        serdes_lane_enable_params1.base_addr = CSL_NETCP_SERDES_0_CFG_REGS;
        serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_156p25M;
        serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_1p25G;
        serdes_lane_enable_params1.num_lanes = numPort1;
        serdes_lane_enable_params1.phy_type = SERDES_SGMII;
        serdes_lane_enable_params1.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

        for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
        {
            serdes_lane_enable_params1.lane_ctrl_rate[i] = CSL_SERDES_LANE_QUARTER_RATE;
            serdes_lane_enable_params1.loopback_mode[i] = lpbk_mode;

            /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
            serdes_lane_enable_params1.rx_coeff.att_start[i] = 7;
            serdes_lane_enable_params1.rx_coeff.boost_start[i] = 5;

            /* For higher speeds PHY-A, force attenuation and boost values  */
            serdes_lane_enable_params1.rx_coeff.force_att_val[i] = 1;
            serdes_lane_enable_params1.rx_coeff.force_boost_val[i] = 1;

            /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
            serdes_lane_enable_params1.tx_coeff.cm_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.c1_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.c2_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.tx_att[i] = 12;
            serdes_lane_enable_params1.tx_coeff.tx_vreg[i] = 4;
        }
        /* if the system has 4 lanes, lane mask = 0xF. If the system has 2 lanes, lane mask = 0x3 etc */
		serdes_lane_enable_params1.lane_mask = (1 << serdes_lane_enable_params1.num_lanes) - 1;
        serdes_lane_enable_params1.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;

        serdes_lane_enable_params2.base_addr = CSL_NETCP_SERDES_1_CFG_REGS;
        serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_156p25M;
        serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_1p25G;
        serdes_lane_enable_params2.num_lanes = numPort2;
        serdes_lane_enable_params2.phy_type = SERDES_SGMII;
        serdes_lane_enable_params2.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;
        for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
        {
            serdes_lane_enable_params2.lane_ctrl_rate[i] = CSL_SERDES_LANE_QUARTER_RATE;
            serdes_lane_enable_params2.loopback_mode[i] = lpbk_mode;
            /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
            serdes_lane_enable_params2.rx_coeff.att_start[i] = 7;
            serdes_lane_enable_params2.rx_coeff.boost_start[i] = 5;

            /* For higher speeds PHY-A, force attenuation and boost values  */
            serdes_lane_enable_params2.rx_coeff.force_att_val[i] = 1;
            serdes_lane_enable_params2.rx_coeff.force_boost_val[i] = 1;

            /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
            serdes_lane_enable_params2.tx_coeff.cm_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.c1_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.c2_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.tx_att[i] = 12;
            serdes_lane_enable_params2.tx_coeff.tx_vreg[i] = 4;
        }
        /* if the system has 4 lanes, lane mask = 0xF. If the system has 2 lanes, lane mask = 0x3 etc */
        serdes_lane_enable_params2.lane_mask = (1 << serdes_lane_enable_params2.num_lanes) - 1;;
        serdes_lane_enable_params2.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;


        /* SB CMU and COMLANE and Lane Setup */
        csl_retval = CSL_EthernetSerdesInit(serdes_lane_enable_params1.base_addr,
                serdes_lane_enable_params1.ref_clock,
                serdes_lane_enable_params1.linkrate); /* 4 port switch1 */

        if (numPort2)
        {
            csl_retval |= CSL_EthernetSerdesInit(serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock,
                    serdes_lane_enable_params2.linkrate); /* 4 port switch2 */
        }

        if (csl_retval != 0)
        {
            System_printf ("Invalid Serdes Init Params\n");
        }

        /* Common Init Mode */
        /* Iteration Mode needs to be set to Common Init Mode first */
        serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
        if (lane_retval != 0)
        {
            System_printf ("Invalid Serdes Common Init\n");
            exit(0);
        }
        System_printf("SGMII Serdes Common Init Complete\n");

        /* Lane Init Mode */
        /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be enabled by setting
           iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT */
        serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
        lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
        if (lane_retval != 0)
        {
            System_printf ("Invalid Serdes Lane Enable Init\n");
            exit(0);
        }
        System_printf("SGMII Serdes Lanes Init Complete\n");

        if(numPort2)
        {
            /* Common Init Mode */
            /* Iteration Mode needs to be set to Common Init Mode first */
            serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
            lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
            if (lane_retval != 0)
            {
                System_printf ("Invalid Serdes Common Init\n");
                exit(0);
            }
            System_printf("SGMII Serdes Common Init Complete\n");

            /* Lane Init Mode */
            /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be enabled by setting
               iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT */
            serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
            lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
            if (lane_retval != 0)
            {
                System_printf ("Invalid Serdes Lane Enable Init\n");
                exit(0);
            }
            System_printf("SGMII Serdes Lanes Init Complete\n");
         }

#elif defined(SOC_K2L)
        CSL_SERDES_RESULT   csl_retval;
        CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
		uint32_t serdes_mux_ethernet_sel, i;
        int numPort1 = (gNum_Mac_Ports > 2)?2:gNum_Mac_Ports;
        int numPort2 = (gNum_Mac_Ports > 2)?gNum_Mac_Ports - 2:0;

        CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params1, serdes_lane_enable_params2;

        memset(&serdes_lane_enable_params1, 0, sizeof(serdes_lane_enable_params1));
        memset(&serdes_lane_enable_params2, 0, sizeof(serdes_lane_enable_params2));

        serdes_lane_enable_params1.base_addr = CSL_CSISC2_2_SERDES_CFG_REGS;
        serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_156p25M;
        serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_1p25G;
        serdes_lane_enable_params1.num_lanes = numPort1;
        serdes_lane_enable_params1.phy_type = SERDES_SGMII;
        serdes_lane_enable_params1.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

        for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
        {
            serdes_lane_enable_params1.lane_ctrl_rate[i] = CSL_SERDES_LANE_QUARTER_RATE;
            serdes_lane_enable_params1.loopback_mode[i] = lpbk_mode;

            /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
            serdes_lane_enable_params1.rx_coeff.att_start[i] = 7;
            serdes_lane_enable_params1.rx_coeff.boost_start[i] = 5;

            /* For higher speeds PHY-A, force attenuation and boost values  */
            serdes_lane_enable_params1.rx_coeff.force_att_val[i] = 1;
            serdes_lane_enable_params1.rx_coeff.force_boost_val[i] = 1;

            /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
            serdes_lane_enable_params1.tx_coeff.cm_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.c1_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.c2_coeff[i] = 0;
            serdes_lane_enable_params1.tx_coeff.tx_att[i] = 12;
            serdes_lane_enable_params1.tx_coeff.tx_vreg[i] = 4;
        }
        /* if the system has 4 lanes, lane mask = 0xF. If the system has 2 lanes, lane mask = 0x3 etc */
		serdes_lane_enable_params1.lane_mask = (1 << serdes_lane_enable_params1.num_lanes) - 1;;
        serdes_lane_enable_params1.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;

        serdes_lane_enable_params2.base_addr = CSL_CSISC2_3_SERDES_CFG_REGS;
        serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_156p25M;
        serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_1p25G;
        serdes_lane_enable_params2.num_lanes = numPort2;
        serdes_lane_enable_params2.phy_type = SERDES_SGMII;
        serdes_lane_enable_params2.forceattboost = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;

        for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
        {
            serdes_lane_enable_params2.lane_ctrl_rate[i] = CSL_SERDES_LANE_QUARTER_RATE;
            serdes_lane_enable_params2.loopback_mode[i] = lpbk_mode;

            /* When RX auto adaptation is on, these are the starting values used for att, boost adaptation */
            serdes_lane_enable_params2.rx_coeff.att_start[i] = 7;
            serdes_lane_enable_params2.rx_coeff.boost_start[i] = 5;

            /* For higher speeds PHY-A, force attenuation and boost values  */
            serdes_lane_enable_params2.rx_coeff.force_att_val[i] = 1;
            serdes_lane_enable_params2.rx_coeff.force_boost_val[i] = 1;

            /* CM, C1, C2 are obtained through Serdes Diagnostic BER test */
            serdes_lane_enable_params2.tx_coeff.cm_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.c1_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.c2_coeff[i] = 0;
            serdes_lane_enable_params2.tx_coeff.tx_att[i] = 12;
            serdes_lane_enable_params2.tx_coeff.tx_vreg[i] = 4;
        }
        /* if the system has 4 lanes, lane mask = 0xF. If the system has 2 lanes, lane mask = 0x3 etc */
		serdes_lane_enable_params2.lane_mask = (1 << serdes_lane_enable_params2.num_lanes) - 1;;
        serdes_lane_enable_params2.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;


        /* Check CSISC2_3_MUXSEL bit */
        if (CSL_FEXTR(*(volatile uint32_t *)(CSL_BOOT_CFG_REGS + 0x20), 28, 28) == 0)
            serdes_mux_ethernet_sel = 1;

        /* SB CMU and COMLANE and Lane Setup */
        csl_retval = CSL_EthernetSerdesInit(serdes_lane_enable_params1.base_addr,
                serdes_lane_enable_params1.ref_clock,
                serdes_lane_enable_params1.linkrate); /* SGMII Lane 0 and Lane 1 */

        if (serdes_mux_ethernet_sel && numPort2)
        {
            csl_retval |= CSL_EthernetSerdesInit(serdes_lane_enable_params2.base_addr,
                    serdes_lane_enable_params2.ref_clock,
                    serdes_lane_enable_params2.linkrate); /* SGMII Lane 2 and Lane 3 */
        }

        if (csl_retval != 0)
        {
            System_printf ("Invalid Serdes Init Params\n");
        }

        /* Common Init Mode */
         /* Iteration Mode needs to be set to Common Init Mode first */
         serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
         lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
         if (lane_retval != 0)
         {
            System_printf ("Invalid Serdes Common Init\n");
             exit(0);
         }
         System_printf("SGMII Serdes Common Init Complete\n");

         /* Lane Init Mode */
         /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be enabled by setting
            iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT */
         serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
         lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
         if (lane_retval != 0)
         {
             System_printf ("Invalid Serdes Lane Enable Init\n");
             exit(0);
         }
         System_printf("SGMII Serdes Lane %d Init Complete\n", i);

         if(serdes_mux_ethernet_sel && numPort2)
         {
             /* Common Init Mode */
             /* Iteration Mode needs to be set to Common Init Mode first */
             serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
             lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
             if (lane_retval != 0)
             {
                System_printf ("Invalid Serdes Common Init\n");
                 exit(0);
             }
             System_printf("SGMII Serdes Common Init Complete\n");

             /* Lane Init Mode */
             /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be enabled by setting
                iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT */
             serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
                 lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
                 if (lane_retval != 0)
                 {
                    System_printf ("Invalid Serdes Lane Enable Init\n");
                     exit(0);
                 }
                 System_printf("SGMII Serdes Lanes Init Complete\n");
         }

#endif

    }

    /* SGMII SERDES Configuration complete. Return. */
    return 0;
}
/** ============================================================================
 *   @n@b Init_Cpsw
 *
 *   @b Description
 *   @n This API sets up the entire ethernet subsystem and all its associated
 *      components.
 *
 *   @param[in]
 *   @n None
 *
 *   @return
 *   @n None
 * =============================================================================
 */
int dest_emac_port_id = 0;
Int32 Init_Cpsw (Void)
{
    Uint32       macPortNum, mtu = 1518;
    Uint8        macSrcAddress [][6] =  {{0x10, 0x11, 0x12, 0x13, 0x14, 0x15},
                                         {0x20, 0x21, 0x22, 0x23, 0x24, 0x25},
                                         {0x20, 0x21, 0x22, 0x23, 0x24, 0x35},
                                         {0x20, 0x21, 0x22, 0x23, 0x24, 0x45},
                                         {0x20, 0x21, 0x22, 0x23, 0x24, 0x55},
                                         {0x20, 0x21, 0x22, 0x23, 0x24, 0x65},
                                         {0x20, 0x21, 0x22, 0x23, 0x24, 0x75},
                                         {0x20, 0x21, 0x22, 0x23, 0x24, 0x85},
                                         };

    Uint8        macAddress[] [6] = {{0x00, 0x01, 0x02, 0x03, 0x04, 0x05},  /* MAC address for (CPPI) Port 0 */
                                     {0x10, 0x11, 0x12, 0x13, 0x14, 0x15},  /* MAC address for (EMAC) Port 1 */
                                     {0x20, 0x21, 0x22, 0x23, 0x24, 0x25},  /* MAC address for (EMAC) Port 2 */
                                     {0x30, 0x31, 0x32, 0x33, 0x34, 0x35},  /* MAC address for (EMAC) Port 3 */
                                     {0x40, 0x41, 0x42, 0x43, 0x44, 0x45},  /* MAC address for (EMAC) Port 4 */
                                     {0x50, 0x51, 0x52, 0x53, 0x54, 0x55},  /* MAC address for (EMAC) Port 5 */
                                     {0x60, 0x61, 0x62, 0x63, 0x64, 0x65},  /* MAC address for (EMAC) Port 6 */
                                     {0x70, 0x71, 0x72, 0x73, 0x74, 0x75},  /* MAC address for (EMAC) Port 7 */
                                     {0x80, 0x81, 0x82, 0x83, 0x84, 0x85}   /* MAC address for (EMAC) Port 8 */
                                     };
    Uint32       portNum;


    /* Set the global Num Mac Ports to 2 when running in non loopback mode */
    if(cpswLpbkMode == CPSW_LOOPBACK_NONE)
    {
        gNum_Mac_Ports = 2;
    }
    /* Set global Num Mac Ports to total number of ports on the device (NUM_PORTS-1) */
    else
    {
        gNum_Mac_Ports = NUM_PORTS - 1;
    }

    /* Initialize the SERDES modules */
    Init_SGMII_SERDES();

    /* Initialize the SGMII/Sliver submodules for the
     * two corresponding MAC ports.
     */
    for (macPortNum = 0; macPortNum < gNum_Mac_Ports; macPortNum++)
    {
        if (Init_SGMII (macPortNum))
          return -1;
        Init_MAC (macPortNum, &macSrcAddress[macPortNum][0], mtu);
    }

    /* Setup the Phys by initializing the MDIO */
    Init_MDIO ();

    Init_CPTS ();

    /* Setup the Ethernet switch finally. */
    Init_Switch (mtu);

    if(cpswLpbkMode == CPSW_LOOPBACK_NONE)
        Switch_update_addr(0, macAddress[0], 0);
    else
        Switch_update_addr(0, macAddress[1], 0);

    for (portNum = 1; portNum < NUM_PORTS; portNum++)
        Switch_update_addr(portNum, macAddress[portNum], 0);

    /* CPSW subsystem setup done. Return success */
    return 0;
}
