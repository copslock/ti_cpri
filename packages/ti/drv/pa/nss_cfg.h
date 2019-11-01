#ifndef _NSS_CFG_H
#define _NSS_CFG_H
/**
 *   @file  nss_cfg.h
 *
 *   @brief   
 *      This file defines Network Sub-System (NSS) transport layer global configuration constant,
 *      macro definitions and data structures where NSS consists of CPSW, PASS and SASS. 
 *      The definitions here are not used by PA LLD itself. Instead, they are used by the application module 
 *      which invokes NSS including all PA unit tests and examples.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2014 Texas Instruments, Inc.
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

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <stdint.h>

/* NSS level files */
#include <ti/drv/pa/nss_if.h>

/**
 *  @def  NSS_LAYOUT_PARAM_NA
 *        Used if NSS layout parameters are not applicable
 */

#define NSS_LAYOUT_PARAM_NA            -1

/** 
 * @brief NSS Layout configuration structure definition
 */

typedef struct nssLayout_s
{
    int fNssGen2;               /**< 1: NSS Gen2 device */

    int numRxCpdmaChans;        /**< Number of PacketDMA Rx channels */
    int numTxCpdmaChans;        /**< Number of PacketDMA Tx channels */
    
    int numTxQueues;            /**< Number of Transmit Queues */
    
    int txQueueBase;            /**< Transmit Queue base */
    int txLocQueueBase;         /**< Local Transmit Queue base (NSS Gen2 only) */
    
    int qPaInputIndex;          /**< Index to the PASS packet input queue */
    int qPaMacIndex;            /**< Index to the PASS Mac queue for MAC/SRIO configuration packets */
    int qPaIpIndex;             /**< Index to the PASS IP queue for (outer) IP configuration packets */
    int qPaInnerIpIndex;        /**< Index to the PASS Inner IP queue for inner IP configuration packets and 
                                     IPSEC Tunnel re-entry packets from SASS */
    int qPaLut2Index;           /**< Index to the PASS LUT2 queue for LUT2 configuration packets and 
                                     IPSEC Transport mode re-entry packets from SASS*/
    int qPaIpsecIndex;          /**< Index to the PASS IPSEC queue for (outer) IP/IPSEC configuration packets */
    int qPaIpsec2Index;         /**< Index to the PASS IPSEC2 queue for inner IP/IPSEC configuration packets */
    int qPaPostIndex;           /**< Index to the PASS Post-Classification queue for Post-Classification
                                     related configuration packets */
    int qPaTxCmdIndex;          /**< Index to the PASS Tx command queue for Egress Packets */
    
    int qPaFirewallIndex;       /**< Index to the PASS (outer) IP firewall queue for (outer) IP firewall configuration 
                                     packets (NSS Gen2 only) */ 
    int qPaFirewall2Index;      /**< Index to the PASS Inner IP firewall queue for inner IP firewall configuration 
                                     packets (NSS Gen2 only)*/
    int qPaEgress0Index;        /**< Index to the PASS Egress stage 0 queue (NSS Gen2 only)*/
    int qPaEgress1Index;        /**< Index to the PASS Egress stage 1 queue (NSS Gen2 only)*/
    int qPaEgress2Index;        /**< Index to the PASS Egress stage 2 queue (NSS Gen2 only)*/

    
    int qSaIndex;               /**< Index to the SASS input 1 queue for IPSEC and SRTP packets*/
    int qSa2Index;              /**< Index to the SASS input 2 queue for Air-Ciphering and data mode packets */
    
    int qCpswEthIndex;          /**< Index to the CPSW CPPI port transmit queue */
    int qCpswEthPri0Index;      /**< Index to the CPSW CPPI port transmit queue of priority 0 packets */
    int qCpswEthPri1Index;      /**< Index to the CPSW CPPI port transmit queue of priority 1 packets */
    int qCpswEthPri2Index;      /**< Index to the CPSW CPPI port transmit queue of priority 2 packets */
    int qCpswEthPri3Index;      /**< Index to the CPSW CPPI port transmit queue of priority 3 packets */
    int qCpswEthPri4Index;      /**< Index to the CPSW CPPI port transmit queue of priority 4 packets */
    int qCpswEthPri5Index;      /**< Index to the CPSW CPPI port transmit queue of priority 5 packets */
    int qCpswEthPri6Index;      /**< Index to the CPSW CPPI port transmit queue of priority 6 packets */
    int qCpswEthPri7Index;      /**< Index to the CPSW CPPI port transmit queue of priority 7 packets */

    int numPaPdsps;             /**< Number of PASS PDSPs */
    int numSaPdsps;             /**< Number of SASS PDSPs */
    
    const uint32_t *paPdspImage[NSS_PA_MAX_PDSPS];        /**< Array of PA PDSP images */
    const int       paPdspImageSize[NSS_PA_MAX_PDSPS];    /**< Array of PA PDSP image sizes */
    
    const uint32_t *saPdspImage[NSS_SA_MAX_PDSPS];        /**< Array of SA PDSP images */
    const int       saPdspImageSize[NSS_SA_MAX_PDSPS];    /**< Array of SA PDSP image sizes */

} nssLayout_t;

/** 
 * @brief NSS Global configuration structure definition
 */
typedef struct nssGlobalConfigParams_s
{
    /** NSS Layout parameters */
    nssLayout_t                             layout;
    
} nssGlobalConfigParams_t;

#ifdef __cplusplus
}
#endif

#endif /* NSS_CFG_H_ */
