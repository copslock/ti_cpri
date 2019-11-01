/**
 *   @file  k2e/src/nss_device.c
 *
 *   @brief   
 *      This file contains the device specific configuration and initialization routines
 *      for NSS (Network Sub-System). These configurations are not used by PA LLD, SA LLD
 *      or CPSW CSL FL. Instaed, they may be included and used by the application modules
 *      which invoke NSS and those LLDs.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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

/* System includes */
#include <stdint.h>
#include <stdlib.h>

/* NSS includes */
#include <ti/drv/pa/nss_if.h>
#include <ti/drv/pa/nss_cfg.h>

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_qm_queue.h>

/* PDSP images */
#include <ti/drv/pa/fw/v1/pafw.h>
#include <ti/drv/pa/fw/v1/pa2_in0_pdsp0_bin.c>
#include <ti/drv/pa/fw/v1/pa2_in0_pdsp1_bin.c>
#include <ti/drv/pa/fw/v1/pa2_in1_pdsp0_bin.c>
#include <ti/drv/pa/fw/v1/pa2_in1_pdsp1_bin.c>
#include <ti/drv/pa/fw/v1/pa2_in2_pdsp0_bin.c>
#include <ti/drv/pa/fw/v1/pa2_in3_pdsp0_bin.c>
#include <ti/drv/pa/fw/v1/pa2_in4_pdsp0_bin.c>
#include <ti/drv/pa/fw/v1/pa2_in4_pdsp1_bin.c>
#include <ti/drv/pa/fw/v1/pa2_post_pdsp0_bin.c>
#include <ti/drv/pa/fw/v1/pa2_post_pdsp1_bin.c>
#include <ti/drv/pa/fw/v1/pa2_eg0_pdsp0_bin.c>
#include <ti/drv/pa/fw/v1/pa2_eg0_pdsp1_bin.c>
#include <ti/drv/pa/fw/v1/pa2_eg0_pdsp2_bin.c>
#include <ti/drv/pa/fw/v1/pa2_eg1_pdsp0_bin.c>
#include <ti/drv/pa/fw/v1/pa2_eg2_pdsp0_bin.c>

#include <ti/drv/sa/fw/v1/safw.h>
#include <ti/drv/sa/fw/v1/saphp1_bin.c>
#include <ti/drv/sa/fw/v1/saphp2_bin.c>
#include <ti/drv/sa/fw/v1/saphp3_bin.c>


/** @brief NSS initialization parameters */
nssGlobalConfigParams_t nssGblCfgParams =
{
    /** NSS Layout parameters */
    { 
        /** 1: NSS Gen2 device */
        TRUE,
        
        /** Number of PacketDMA Rx channels */
        NSS_NUM_RX_PKTDMA_CHANNELS_GEN2,
        
        /** Number of PacketDMA Tx channels */
        NSS_NUM_TX_PKTDMA_CHANNELS_GEN2,
        
        /** Number of Transmit Queues */
        NSS_NUM_TX_QUEUES_GEN2,
        
        /** Transmit Queue base */
        QMSS_PASS_QUEUE_BASE,
        
        /** Local Transmit Queue base (NSS Gen2 only) */
        QMSS_NETSS_PASS_QUEUE_BASE,
            
        /** Index to the PASS packet input queue */
        NSS_PA_QUEUE_INPUT_INDEX_GEN2,
      
        /** Index to the PASS Mac queue for MAC/SRIO configuration packets */
        NSS_PA_QUEUE_MAC_INDEX_GEN2,
        
        /** Index to the PASS IP queue for (outer) IP configuration packets */
        NSS_PA_QUEUE_OUTER_IP_INDEX_GEN2,
        
        /** Index to the PASS Inner IP queue for inner IP configuration packets and 
             IPSEC Tunnel re-entry packets from SASS */
        NSS_PA_QUEUE_INNER_IP_INDEX_GEN2,
             
        /** Index to the PASS LUT2 queue for LUT2 configuration packets and 
             IPSEC Transport mode re-entry packets from SASS */
        NSS_PA_QUEUE_LUT2_INDEX_GEN2,
             
        /** Index to the PASS IPSEC queue for (outer) IP/IPSEC configuration packets */
        NSS_PA_QUEUE_IPSEC_INDEX_GEN2,
        
        /** Index to the PASS IPSEC2 queue for inner IP/IPSEC configuration packets */
        NSS_PA_QUEUE_IPSEC2_INDEX_GEN2,
        
        /** Index to the PASS Post-Classification queue for Post-Classification
             related configuration packets */
        NSS_PA_QUEUE_POST_INDEX_GEN2,
             
        /** Index to the PASS Tx command queue for Egress Packets */
        NSS_PA_QUEUE_TXCMD_INDEX_GEN2,
        
        /** Index to the PASS (outer) IP firewall queue for (outer) IP firewall configuration
             packets (NSS Gen2 only) */
        NSS_PA_QUEUE_FIREWALL_INDEX_GEN2,
              
        /** Index to the PASS Inner IP firewall queue for inner IP firewall configuration 
             packets (NSS Gen2 only)*/
        NSS_PA_QUEUE_FIREWALL2_INDEX_GEN2,
             
        /** Index to the PASS Egress stage 0 queue (NSS Gen2 only)*/
        NSS_PA_QUEUE_EGRESS0_INDEX_GEN2,
        
        /** Index to the PASS Egress stage 1 queue (NSS Gen2 only)*/
        NSS_PA_QUEUE_EGRESS1_INDEX_GEN2,
        
        /** Index to the PASS Egress stage 2 queue (NSS Gen2 only)*/
        NSS_PA_QUEUE_EGRESS2_INDEX_GEN2,
      
        /** Index to the SASS input 1 queue for IPSEC and SRTP packets*/
        NSS_SA_QUEUE_SASS_INDEX_GEN2,
        
        /** Index to the SASS input 2 queue for Air-Ciphering and data mode packets */
        NSS_SA_QUEUE_SASS2_INDEX_GEN2,
      
        /** Index to the CPSW CPPI port transmit queue */
        NSS_CPSW_QUEUE_ETH_INDEX_GEN2,
        
        /** Index to the CPSW CPPI port transmit queue of priority 0 packets */
        NSS_CPSW_QUEUE_ETH_PRI0_INDEX_GEN2,
        
        /** Index to the CPSW CPPI port transmit queue of priority 1 packets */
        NSS_CPSW_QUEUE_ETH_PRI1_INDEX_GEN2,
        
        /** Index to the CPSW CPPI port transmit queue of priority 2 packets */
        NSS_CPSW_QUEUE_ETH_PRI2_INDEX_GEN2,
        
        /** Index to the CPSW CPPI port transmit queue of priority 3 packets */
        NSS_CPSW_QUEUE_ETH_PRI3_INDEX_GEN2,
        
        /** Index to the CPSW CPPI port transmit queue of priority 4 packets */
        NSS_CPSW_QUEUE_ETH_PRI4_INDEX_GEN2,
        
        /** Index to the CPSW CPPI port transmit queue of priority 5 packets */
        NSS_CPSW_QUEUE_ETH_PRI5_INDEX_GEN2,
        
        /** Index to the CPSW CPPI port transmit queue of priority 6 packets */
        NSS_CPSW_QUEUE_ETH_PRI6_INDEX_GEN2,
        
        /** Index to the CPSW CPPI port transmit queue of priority 7 packets */
        NSS_CPSW_QUEUE_ETH_PRI7_INDEX_GEN2,
        
        /** Number of PASS PDSPs */
        NSS_PA_NUM_PDSPS_GEN2,
        
        /** Number of SASS PDSPs */
        NSS_SA_NUM_PDSPS_GEN2,
        
        /** PA PDSP images */
        {
            /* PDSP Packet Ingress0 PDSP0 image */
            in0_pdsp0,
            
            /* PDSP Packet Ingress0 PDSP1 image */
            in0_pdsp1,
            
            /* PDSP Packet Ingress1 PDSP0 image */
            in1_pdsp0,
            
            /* PDSP Packet Ingress1 PDSP1 image */
            in1_pdsp1,
            
            /* PDSP Packet Ingress2 PDSP0 image */
            in2_pdsp0,
            
            /* PDSP Packet Ingress3 PDSP0 image */
            in3_pdsp0,
            
            /* PDSP Packet Ingress4 PDSP0 image */
            in4_pdsp0,
            
            /* PDSP Packet Ingress4 PDSP1 image */
            in4_pdsp1,
            
            /* PDSP Packet Post PDSP0 image */
            post_pdsp0,
            
            /* PDSP Packet Post PDSP1 image */
            post_pdsp1,
            
            /* PDSP Packet Egress0 PDSP0 image */
            eg0_pdsp0,
            
            /* PDSP Packet Egress0 PDSP1 image */
            eg0_pdsp1,
            
            /* PDSP Packet Egress0 PDSP2 image */
            eg0_pdsp2,
            
            /* PDSP Packet Egress1 PDSP0 image */
            eg1_pdsp0,
            
            /* PDSP Packet Egress2 PDSP0 image */
            eg2_pdsp0,
            
        },
        
        
        /** PA PDSP image sizes  */
        {
            /* PDSP Packet Ingress0 PDSP0 image */
            sizeof(in0_pdsp0),
            
            /* PDSP Packet Ingress0 PDSP1 image */
            sizeof(in0_pdsp1),
            
            /* PDSP Packet Ingress1 PDSP0 image */
            sizeof(in1_pdsp0),
            
            /* PDSP Packet Ingress1 PDSP1 image */
            sizeof(in1_pdsp1),
            
            /* PDSP Packet Ingress2 PDSP0 image */
            sizeof(in2_pdsp0),
            
            /* PDSP Packet Ingress3 PDSP0 image */
            sizeof(in3_pdsp0),
            
            /* PDSP Packet Ingress4 PDSP0 image */
            sizeof(in4_pdsp0),
            
            /* PDSP Packet Ingress4 PDSP1 image */
            sizeof(in4_pdsp1),
            
            /* PDSP Packet Post PDSP0 image */
            sizeof(post_pdsp0),
            
            /* PDSP Packet Post PDSP1 image */
            sizeof(post_pdsp1),
            
            /* PDSP Packet Egress0 PDSP0 image */
            sizeof(eg0_pdsp0),
            
            /* PDSP Packet Egress0 PDSP1 image */
            sizeof(eg0_pdsp1),
            
            /* PDSP Packet Egress0 PDSP2 image */
            sizeof(eg0_pdsp2),
            
            /* PDSP Packet Egress1 PDSP0 image */
            sizeof(eg1_pdsp0),
            
            /* PDSP Packet Egress2 PDSP0 image */
            sizeof(eg2_pdsp0)
            
        },
        
        /** SA PDSP images */
        {
            /* Packet Header Processing 1 image */
            Sa2_php1,
            
            /* Packet Header Processing 2 image */
            Sa2_php2,
            
            /* Packet Header Processing 3 image */
            Sa2_php3,
        },
        
        
        /** SA PDSP image sizes */
        {
            /* Packet Header Processing 1 image */
            sizeof(Sa2_php1),
            
            /* Packet Header Processing 2 image */
            sizeof(Sa2_php2),
            
            /* Packet Header Processing 3 image */
            sizeof(Sa2_php3)
        }
    }
};

/**
@}
*/

