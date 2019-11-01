/******************************************************************************
 * FILE PURPOSE:  Compile time Tunable parameters for NWAL Module
 ******************************************************************************
 * FILE NAME:   nwal_util.h
 *
 * DESCRIPTION: Static inline NWAL utility functions. Can be used in the
 *              case  of Applications using low NWL overhead mode and being
 *              able to get access to low level hardware TX and RX
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2012
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
/* ============================================================= */
/**
 *   @file  nwal_util.h
 *
 *   path  ti/drv/nwal/nwal_util.h
 *
 *   @brief     File contains low level utility function and inline
 *              macros to enable hardware offload for keystone
 *              devices.
 *              Static inline NWAL utility functions. Can be used in the
 *              case of Applications using low NWL overhead mode and being
 *              able  to get access to low level hardware TX and RX
 *
 */

/**
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2012 Texas Instruments, Inc.
 *  \par
 */


#ifndef _NWAL_UTIL_H
#define _NWAL_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif
#include "ti/drv/nwal/nwal.h"
#include "ti/drv/nwal/src/nwal_loc.h"

extern nssGlobalConfigParams_t nssGblCfgParams;
/**************** DEFINES USED BY INLINE MACROS BEGIN ****************/
#if defined(DEVICE_K2E) || defined(K2L) || defined(SOC_K2E) || defined(SOC_K2L)
#define NWAL_PAFRM_ETH_PS_FLAGS_PORT_SHIFT  0
#else
#define NWAL_PAFRM_ETH_PS_FLAGS_PORT_SHIFT  4
#endif
#define NWAL_PA_L4_CHKSUM_ERR               0x4
#define NWAL_PA_IPV4_HDR_CHKSUM_ERR         0x8


/**
 *  @ingroup nwal_api_structures
 *  @brief Cookie updated by NWAL for the packet flow
 *
 *  @details The parameters in this structure will be updated by NWAL during
 *           API call @see nwal_initPsCmdInfo(). Application to pass it back
 *           in @see nwalTxPktInfo_t during @see nwal_send API call for packet
 *           transmission.
 */
#define NWAL_MAX_PS_COMMAND_SIZE       96
typedef struct {
    uint32_t        psCmdLabel[NWAL_MAX_PS_COMMAND_SIZE/4];
                                        /**<  Command Label to be sent to
                                          *   NetCP for the packet flow
                                          *   with list of actions
                                          */
    uint32_t        psCmdLabelLen;      /**< Length of valid command label in
                                          *  bytes
                                          */
    uint8_t         udpChkOff;          /**< Offset for UDP checksum command
                                          *  in the command label
                                          */
    uint8_t         pa2saNextRoute;     /**< Offset for next route information
                                          *  to update SA Context details
                                          */
    uint8_t         saShortInfoOff;     /**< Offset for SA Short Info command
                                          *  in the command label
                                          */
    uint8_t         enetPortNextRoute;  /**< Offset for next route information
                                          *  to update SA Context details
                                          */
    uint8_t         ahPathCmdOff;       /**< Offset for updating ICV bytes in
                                          *  case of IPSec AH crypto offload
                                          */
    Qmss_QueueHnd   txQueue;            /**< Transmit Queue */
}nwalTxPSCmdInfo_t;

#ifdef NWAL_ENABLE_SA
/**
 *  @ingroup nwal_api_structures
 *  @brief Cookie updated by NWAL for the Data Mode SA Channel
 *
 *  @details The parameters in this structure will be updated by NWAL during
 *           API call @see nwal_initDMPSCmdInfo(). Application to pass it back
 *           in  @see nwal_mCmdDMUpdate API call before packet
 *           transmission.
 */
typedef struct {
    Sa_CmdLbUpdateInfo_t*   pUpdateInfo;
    uint32_t*               pCmdLb;/**<  Command Label to be sent to
                                   *   NetCP for the packet flow
                                   *   with list of actions
                                   */
    uint16_t               cmdLbLen;/**< Length of command Label */
    uint32_t*              pSwInfo; /**< Security Context info for the channel */
    Qmss_QueueHnd          rxSbSaQ;  /**< Default  Packet Queue to receive response
                                      * for all data mode SA related
                                      * responses
                                      */
    int16_t                rxPktFlowId; /**< Default Flow Handle for packets
                                           *  from NetCP to host
                                           */
    Qmss_QueueHnd          txQueue; /**< Transmit Queue */
}nwalTxDmPSCmdInfo_t;
#endif
/******************** DEFINES USED BY INLINE MACROS END ********************/

/**************** Low Level API Functions exposed by NWAL BEGIN ***************/
/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_initPsCmdInfo  API for Initializing PS Command Info
 *
 *  @details The API can be called by application to initialize NetCP protocol
 *           specific command for any hardware offload functionality.
 *           The API should be called for similar packet format with protocol
 *           headers at fixed offsets. For transmitting packets out the helper
 *           functions included in nwal_util.h can be used.
 *
 *           The API call is not required for the application using
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  pPktInfo        Transmit packet information @see nwalTxPktInfo_t
 *  @param[in]  pTxPSCmdInfo    Protocol Specific command info initialized with
 *                              static values  during init.Dynamic field update
 *                              will be done during API call nwal_sendxxx
 *
 *  @retval      @see nwal_OK  on success. Error codes @see nwal_RetValue
 *  @pre         @see nwal_start
 */
nwal_RetValue nwal_initPSCmdInfo  (nwal_Inst            nwalInst,
                                   nwalTxPktInfo_t*     pPktInfo,
                                   nwalTxPSCmdInfo_t*   pTxPSCmdInfo);

#ifdef NWAL_ENABLE_SA
/**
 *  @ingroup nwal_api_functions
 *  @brief nwal_getDmCmdLb  Get the Data Mode Command Label created during
 *         channel create time
 *
 *  @details API returns static PS command label infrastructure created
 *           during nwal_setDMSecAssoc().
 *  @param[in]  nwalInst    NWAL Instance  identifier
 *  @param[in]  nwalDmSaHandle        Handle returned after
 *                                    @see nwal_setDMSecAssoc
 *  @param[in]  pTxDmPsCmdInfo       Command label Meta Data for Crypto SB Send
 *  @pre        @see nwal_setDMSecAssoc
 */
nwal_RetValue nwal_initDMPSCmdInfo(nwal_Inst              nwalInst,
                                   nwal_Handle            nwalDmSaHandle,
                                   nwalTxDmPSCmdInfo_t*   pTxDmPsCmdInfo);
#endif

/** @defgroup nwal_fastMacros NWAL Low Level APIs
 *  @{
 */
/** @} */

/** @defgroup nwal_fast_send_macros NWAL Transmit Low Level Helper API's
 *  @ingroup nwal_fastMacros
 */

/** @defgroup nwal_recv_macros NWAL Receive Low Level Helper API's
 *  @ingroup nwal_fastMacros
 */


/*********************** NWAL FAST SEND HELPER MACROS BEGIN ******************/

/********************************************************************
 * FUNCTION PURPOSE: Update the Enet Port in Next Route for PS command
 *                   to NetCP
 ********************************************************************
 * DESCRIPTION: Update the Enet Port in Next Route for PS command
 *              to NetCP
 ********************************************************************/
static inline void nwal_modNREnetPort(uint8_t*             pPsDataBuf,
                                      uint8_t              portNROffset,
                                      nwal_enetPort_t      enetPort)
{
    pasahoNextRoute_t*  nr;
    uint8_t             psFlags =0;

    nr = (pasahoNextRoute_t*)&pPsDataBuf[portNROffset];

    if(enetPort)
    {
        /* Send to explicit port. In the case of loopback mode
         * enetPort is expected to be set to NWAL_ENET_PORT_UNKNOWN
         */
         PASAHO_SET_E (nr, 1);
         psFlags = ((enetPort & pa_EMAC_CTRL_PORT_MASK) <<
                     NWAL_PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
         PASAHO_SET_PKTTYPE  (nr, psFlags);
         PASAHO_SET_DEST  (nr, PASAHO_NR_DEST_ETH);
    }
    /* PS FLAG update not being done. Here assumption is that
     * application is not switching between loopback and
     * non loopback mode between creation of command label and
     * sending packet
     * In the case of non loopback case initial PS command label
     * will already have PS flag set to zero
     */
}

/**
 *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdUpdatePSData  Update NetCP command for any offload
 *         functionilty for TX packets.
 *
 *  @details Inline macro API can be used to update PS data.
 *           API to be called for updating PS data with no additional
 *           per packet modification compared to already available after
 *           @see nwal_initPSCmdInfo API call.
 *           In this case application is expected
 *           to provide full packet including protocol headers and call
 *           Qmss_queuePushXX API to transmit the packet out.
 *           QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *          Application would also need to do required cache flush operation in
 *          the case of non cache coherent architecure. Refer @see nwal_send
 *          for sample implementation
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[in]  pTxPSCmdInfo    Static Command Label @see nwalTxPSCmdInfo_t
 *                              created during API call
 *  @param[in]  ppPktDesc       CPPI packet descriptor which could be used
 *   @param[in] ppPsDataBuf     Buffer containing PS Data
 *                              @see nwal_initPSCmdInfo
 *  @pre        @see nwal_initPSCmdInfo
 */
static inline
void nwal_mCmdUpdatePSData(Ti_Pkt*               pPkt,
                             nwalTxPSCmdInfo_t*    pTxPSCmdInfo,
                             Cppi_HostDesc**       ppPktDesc,
                             uint8_t**             ppPsDataBuf)
{
    *ppPktDesc = Pktlib_getDescFromPacket(pPkt);
    /* Attach the command label to the PS data in descriptor */
    *ppPsDataBuf = Cppi_setPSData (Cppi_DescType_HOST,
                                   (Cppi_Desc *)*ppPktDesc,
                                   (uint8_t *)pTxPSCmdInfo->psCmdLabel,
                                   pTxPSCmdInfo->psCmdLabelLen);
}

/**
 *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdSetPort  Minimal NetCP offload case. EMAC port information
 *         for outgoing packet will be updated.
 *
 *  @details Inline macro API can be used to transmit  packet
 *           out of specific EMAC Port without using any offload feature
 *           at NetCP. In this case application is expected
 *           to provide full packet including protocol headers and call
 *           Qmss_queuePushXX API to transmit the packet out.
 *           QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *          Application would also need to do required cache flush
 *          operation in the case of non cache coherent architecure.
 *          Refer @see nwal_send() for sample implementation
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[in]  pTxPSCmdInfo    Static Command Label @see nwalTxPSCmdInfo_t
 *                              created during API call
 *                              @see nwal_initPSCmdInfo
 *  @param[in]  enetPort        Transmit Enet Port
 *  @pre        @see nwal_initPSCmdInfo
 */
static inline void nwal_mCmdSetPort  (Ti_Pkt*               pPkt,
                                      nwalTxPSCmdInfo_t*    pTxPSCmdInfo,
                                      nwal_enetPort_t       enetPort)
{
    Cppi_HostDesc*      pPktDesc;
    uint8_t*            pPsDataBuf;

    if((pTxPSCmdInfo) && (pTxPSCmdInfo->psCmdLabelLen))
    {
        nwal_mCmdUpdatePSData(pPkt,pTxPSCmdInfo,&pPktDesc,&pPsDataBuf);
        nwal_modNREnetPort(pPsDataBuf,
                           pTxPSCmdInfo->enetPortNextRoute,
                           enetPort);
        return;
    }
    pPktDesc = Pktlib_getDescFromPacket(pPkt);
    /* Redirect packet to port based on Application configuration.
     * Zero value will let switch route the packet to correct outgoing port
     */
    if (!nssGblCfgParams.layout.fNssGen2)
    {
        Cppi_setPSFlags(Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc, enetPort);
    }
    else
    {
        Cppi_DescTag    tag;
        tag.srcTagHi  = 0;
        tag.srcTagLo  = 0;
        tag.destTagHi = 0;
        tag.destTagLo = enetPort;
        Cppi_setTag(Cppi_DescType_HOST,
                   (Cppi_Desc *)pPktDesc,
                   (Cppi_DescTag *)&tag);
    }
    /* Clear PS Data */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc, 0);
}

/**
 *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdSetL4CkSumPort  Update L4 checksum command and outgoing
 *         EMAC port to NetCP command.
 *
 *  @details Inline macro API can be used to do dynamic update for L4
 *           checksum offload command to NetCP before transmitting  packet
 *           out of specific EMAC Port. In this case application is expected
 *           to provide full packet including protocol headers and call
 *           Qmss_queuePushXX API to transmit the packet out.
 *           QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *          Application would also need to do required cache flush operation in
 *          the case of non cache coherent architecure. Refer @see nwal_send()
 *          for sample implementation
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[in]  pTxPSCmdInfo    Static Command Label @see nwalTxPSCmdInfo_t
 *                              created during API call
 *                              @see nwal_initPSCmdInfo
 *  @param[in]  l4OffBytes      Offset to Layer 4 header
 *  @param[in]  l4Len           Layer 4 payload length including header
 *                              + payload
 *  @param[in]  pseudoHdrChecksum   Pseudo Header checksum for L4
 *  @param[in]  enetPort            Transmit Enet Port
 *  @pre        @see nwal_initPSCmdInfo
 */
static inline
void nwal_mCmdSetL4CkSumPort(Ti_Pkt*               pPkt,
                             nwalTxPSCmdInfo_t*    pTxPSCmdInfo,
                             uint16_t              l4OffBytes,
                             uint16_t              l4Len,
                             uint16_t              pseudoHdrChecksum,
                             nwal_enetPort_t       enetPort)
{
    Cppi_HostDesc*      pPktDesc;
    pasahoComChkCrc_t*  ptx;
    uint8_t*            pPsDataBuf;

    nwal_mCmdUpdatePSData(pPkt,pTxPSCmdInfo,&pPktDesc,&pPsDataBuf);

    /* L4 checksum command need to be updated with
     * Offset to the L4 header
     * new pseudo header checksum and
     * L4 length
     */
    ptx = (pasahoComChkCrc_t *)&pPsDataBuf[pTxPSCmdInfo->udpChkOff];

    /* Update Layer 4 Offset bytes */
    PASAHO_CHKCRC_SET_START      (ptx, l4OffBytes);

    /* Update Length */
    PASAHO_CHKCRC_SET_LEN (ptx,l4Len);

    /* Update Pseudo header checksum */
    PASAHO_CHKCRC_SET_INITVAL    (ptx, pseudoHdrChecksum);

    nwal_modNREnetPort(pPsDataBuf,
                       pTxPSCmdInfo->enetPortNextRoute,
                       enetPort);
}


/**
  *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdSetCrypPort  Update Crypto command and outgoing
 *         EMAC port to NetCP command.
 *
 *  @details Inline macro API can be used to do dynamic update of
 *           crypto command parameters to NetCP before transmitting  packet
 *           out of specific EMAC Port. In this case application is expected
 *           to provide full packet including protocol headers and call
 *           Qmss_queuePushXX API to transmit the packet out.
*            QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *           Application would also need to do required cache flush
 *           operation in the case of non cache coherent architecure.
 *           Refer nwal_send() for sample implementation
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[in]  pTxPSCmdInfo    Static Command Label @see nwalTxPSCmdInfo_t
 *                              created during API call
 *                              @see nwal_initPSCmdInfo
 *  @param[in]  saOffBytes      Offset from base of the packet to the header
 *                              of protocol per the following list:
 *                                  - IPSEC ESP with AH: IP header
 *                                  - IPSEC ESP: ESP Header
 *  @param[in]  saPayloadLen    Length of the payload starting from
 *                              saOffBytes to the end of the protocol
 *  @param[in]  swInfo0         Sw Info with SA context related details
 *  @param[in]  swInfo1         Sw Info with SA context related details
 *  @param[in]  enetPort        Transmit Enet Port
 *  @pre        @see nwal_getSecAssoc @see nwal_initPSCmdInfo
 */
static inline void nwal_mCmdSetCrypPort (Ti_Pkt*               pPkt,
                                         nwalTxPSCmdInfo_t*    pTxPSCmdInfo,
                                         uint16_t              saOffBytes,
                                         uint16_t              saPayloadLen,
                                         uint32_t              swInfo0,
                                         uint32_t              swInfo1,
                                         nwal_enetPort_t       enetPort)
{
    Cppi_HostDesc*      pPktDesc;
    uint8_t*            pPsDataBuf;
    pasahoShortInfo_t*  sInfo;
    nwal_mCmdUpdatePSData(pPkt,pTxPSCmdInfo,&pPktDesc,&pPsDataBuf);

    sInfo = (pasahoShortInfo_t *)&pPsDataBuf[pTxPSCmdInfo->saShortInfoOff];
    PASAHO_SINFO_SET_PAYLOAD_OFFSET(sInfo, saOffBytes);
    PASAHO_SINFO_SET_PAYLOAD_LENGTH(sInfo, saPayloadLen);

    nwal_modNREnetPort(pPsDataBuf,
                       pTxPSCmdInfo->enetPortNextRoute,
                       enetPort);

    /* Update the Software Info for Security context in the descriptor
     * itself as packet would be sent directly to SA directly
     */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc,swInfo0);
    Cppi_setSoftwareInfo1(Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc,swInfo1);

}




/**
  *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdSetL3CkSumCrypPort  Update L3 checksum, Crypto command and outgoing
 *         EMAC port to NetCP command.
 *
 *  @details Inline macro API can be used to do dynamic update of
 *           crypto command parameters to NetCP before transmitting  packet
 *           out of specific EMAC Port. In this case application is expected
 *           to provide full packet including protocol headers and call
 *           Qmss_queuePushXX API to transmit the packet out.
*            QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *           Application would also need to do required cache flush
 *           operation in the case of non cache coherent architecure.
 *           Refer nwal_send() for sample implementation
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[in]  pTxPSCmdInfo    Static Command Label @see nwalTxPSCmdInfo_t
 *                              created during API call
 *                              @see nwal_initPSCmdInfo
 *  @param[in]  saOffBytes      Offset from base of the packet to the header
 *                              of protocol per the following list:
 *                                  - IPSEC ESP with AH: IP header
 *                                  - IPSEC ESP: ESP Header
 *  @param[in]  saPayloadLen    Length of the payload starting from
 *                              saOffBytes to the end of the protocol
 *  @param[in]  swInfo0         Sw Info with SA context related details
 *  @param[in]  swInfo1         Sw Info with SA context related details
 *  @param[in]  enetPort        Transmit Enet Port
 *  @pre        @see nwal_getSecAssoc @see nwal_initPSCmdInfo
 */
static inline void nwal_mCmdSetL3CkSumCrypPort (Ti_Pkt*               pPkt,
                                         nwalTxPSCmdInfo_t*    pTxPSCmdInfo,
                                         uint16_t              saOffBytes,
                                         uint16_t              saPayloadLen,
                                         uint32_t              swInfo0,
                                         uint32_t              swInfo1,
                                         nwal_enetPort_t       enetPort)
{
    Cppi_HostDesc*      pPktDesc;
    uint8_t*            pPsDataBuf;
    pasahoShortInfo_t*  sInfo;
    pasahoNextRoute_t*  nr;
    nwal_mCmdUpdatePSData(pPkt,pTxPSCmdInfo,&pPktDesc,&pPsDataBuf);

    sInfo = (pasahoShortInfo_t *)&pPsDataBuf[pTxPSCmdInfo->saShortInfoOff];
    PASAHO_SINFO_SET_PAYLOAD_OFFSET(sInfo, saOffBytes);
    PASAHO_SINFO_SET_PAYLOAD_LENGTH(sInfo, saPayloadLen);


    nr = (pasahoNextRoute_t *)&pPsDataBuf[pTxPSCmdInfo->pa2saNextRoute];
    nr->swInfo0 = swInfo0;
    nr->swInfo1 = swInfo1;


    nwal_modNREnetPort(pPsDataBuf,
                       pTxPSCmdInfo->enetPortNextRoute,
                       enetPort);

    /* Update the Software Info for Security context in the descriptor
     * itself as packet would be sent directly to SA directly
     */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc,swInfo0);
    Cppi_setSoftwareInfo1(Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc,swInfo1);

}

/**
 *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdSetL4CkSumCrypPort  Update L4 checksum,Crypto and
 *         outgoing EMAC port to NetCP command
 *
 *  @details Inline macro API can be used to do dynamic update for Crypto
 *           and L4  checksum offload command to NetCP before transmitting
 *           packet out of specific EMAC Port. In this case application is
 *           expected to provide full packet including protocol headers and
 *           call Qmss_queuePushXX API to transmit the packet out.
 *           QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *          Application would also need to do required cache flush operation in
 *          the case of non cache coherent architecure. Refer nwal_send()
 *          for sample implementation
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[in]  pTxPSCmdInfo    Static Command Label @see nwalTxPSCmdInfo_t
 *                              created during API call
 *                              @see nwal_initPSCmdInfo
 *  @param[in]  l4OffBytes      Offset to L4 Header
 *  @param[in]  l4Len           Layer 4 payload length including header +
 *                              payload
 *  @param[in]  pseudoHdrChecksum   Pseudo Header checksum for L4
 *  @param[in]  saOffBytes      Offset from base of the packet to the header
 *                              of protocol per the following list:
 *                                  - IPSEC ESP with AH: IP header
 *                                  - IPSEC ESP: ESP Header
 *  @param[in]  saPayloadLen    Length of the payload starting from
 *                              saOffBytes to the end of the protocol
 *  @param[in]  swInfo0         Sw Info with SA context related details
 *  @param[in]  swInfo1         Sw Info with SA context related details
 *  @param[in]  enetPort        Transmit Enet Port
 *  @pre        @see nwal_getSecAssoc @see nwal_initPSCmdInfo
 */
static inline void
            nwal_mCmdSetL4CkSumCrypPort (Ti_Pkt*               pPkt,
                                         nwalTxPSCmdInfo_t*    pTxPSCmdInfo,
                                         uint16_t              l4OffBytes,
                                         uint16_t              l4Len,
                                         uint16_t              pseudoHdrChecksum,
                                         uint16_t              saOffBytes,
                                         uint16_t              saPayloadLen,
                                         uint32_t              swInfo0,
                                         uint32_t              swInfo1,
                                         nwal_enetPort_t       enetPort)
{

    pasahoComChkCrc_t   *ptx;
    Cppi_HostDesc*      pPktDesc;
    uint8_t*            pPsDataBuf;
    pasahoShortInfo_t*  sInfo;
    pasahoNextRoute_t*  nr;

    nwal_mCmdUpdatePSData(pPkt,pTxPSCmdInfo,&pPktDesc,&pPsDataBuf);

    /* L4 checksum command need to be updated with
     * Offset to the L4 header
     * new pseudo header checksum and
     * L4 length
     */
    ptx = (pasahoComChkCrc_t *)&pPsDataBuf[pTxPSCmdInfo->udpChkOff];

    /* Update Layer 4 Offset bytes */
    PASAHO_CHKCRC_SET_START      (ptx, l4OffBytes);

    /* Update Length */
    PASAHO_CHKCRC_SET_LEN (ptx,l4Len);

    /* Update Pseudo header checksum */
    PASAHO_CHKCRC_SET_INITVAL    (ptx, pseudoHdrChecksum);

    sInfo = (pasahoShortInfo_t *)&pPsDataBuf[pTxPSCmdInfo->saShortInfoOff];
    PASAHO_SINFO_SET_PAYLOAD_OFFSET(sInfo, saOffBytes);
    PASAHO_SINFO_SET_PAYLOAD_LENGTH(sInfo, saPayloadLen);


    nr = (pasahoNextRoute_t *)&pPsDataBuf[pTxPSCmdInfo->pa2saNextRoute];
    nr->swInfo0 = swInfo0;
    nr->swInfo1 = swInfo1;
    nwal_modNREnetPort(pPsDataBuf,
                       pTxPSCmdInfo->enetPortNextRoute,
                       enetPort);

}

/**
  *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdSetAHCrypPort  Update Crypto command and outgoing
 *         EMAC port to NetCP command for IPSec AH mode.
 *
 *  @details Inline macro API can be used to do dynamic update of
 *           crypto command parameters to NetCP before transmitting  packet
 *           out of specific EMAC Port. In this case application is expected
 *           to provide full packet including protocol headers and call
 *           Qmss_queuePushXX API to transmit the packet out.
*            QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *           Application would also need to do required cache flush
 *           operation in the case of non cache coherent architecure.
 *           Refer nwal_send() for sample implementation
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[in]  pTxPSCmdInfo    Static Command Label @see nwalTxPSCmdInfo_t
 *                              created during API call
 *                              @see nwal_initPSCmdInfo
 *  @param[in]  saOffBytes      Offset from base of the packet to the header
 *                              of protocol per the following list:
 *                                  - IPSEC ESP with AH: IP header
 *                                  - IPSEC ESP: ESP Header
 *  @param[in]  saPayloadLen    Length of the payload starting from
 *                              saOffBytes to the end of the protocol
 *  @param[in]  swInfo0         Sw Info with SA context related details
 *  @param[in]  swInfo1         Sw Info with SA context related details
 *  @param[in]  saAhIcvOffBytes Offset to the ICV field in the case of
 *                              IPSec AH mode for authentication tag insertion
 *                               by NETCP.
 *  @param[in]  saAhMacSize     Size of the authentication tag to be inserted by
 *                              NetCP in the case of IPSec AH mode. Reset to
 *                              zero in the case of ESP mode.
 *                              Application also need to make sure that size is
 *                              not more than max value
 *                              @ref NWAL_IPSEC_AH_MAX_AUTH_TAG_BYTES
 *  @param[in]  enetPort        Transmit Enet Port
 *  @pre        @see nwal_getSecAssoc @see nwal_initPSCmdInfo
 */
static inline void nwal_mCmdSetAHCrypPort (Ti_Pkt*               pPkt,
                                         nwalTxPSCmdInfo_t*    pTxPSCmdInfo,
                                         uint16_t              saOffBytes,
                                         uint16_t              saPayloadLen,
                                         uint32_t              swInfo0,
                                         uint32_t              swInfo1,
                                         uint16_t              saAhIcvOffBytes,
                                         uint16_t              saAhMacSize,
                                         nwal_enetPort_t       enetPort)
{
    Cppi_HostDesc*          pPktDesc;
    uint8_t*                pPsDataBuf;
    pasahoShortInfo_t*      pSInfo;
    pasahoComBlindPatch_t*  pBpCmd;

    nwal_mCmdUpdatePSData(pPkt,pTxPSCmdInfo,&pPktDesc,&pPsDataBuf);

    pSInfo = (pasahoShortInfo_t *)&pPsDataBuf[pTxPSCmdInfo->saShortInfoOff];
    PASAHO_SINFO_SET_PAYLOAD_OFFSET(pSInfo, saOffBytes);
    PASAHO_SINFO_SET_PAYLOAD_LENGTH(pSInfo, saPayloadLen);

    nwal_modNREnetPort(pPsDataBuf,
                       pTxPSCmdInfo->enetPortNextRoute,
                       enetPort);
    pBpCmd = (pasahoComBlindPatch_t *)&pPsDataBuf[pTxPSCmdInfo->ahPathCmdOff];
    PASAHO_BPATCH_SET_PATCH_NBYTES  (pBpCmd, saAhMacSize);
    PASAHO_BPATCH_SET_OFFSET        (pBpCmd, saAhIcvOffBytes);

    /* Update the Software Info for Security context in the descriptor
     * itself as packet would be sent directly to SA directly
     */
    Cppi_setSoftwareInfo0(Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc,swInfo0);
    Cppi_setSoftwareInfo1(Cppi_DescType_HOST, (Cppi_Desc *)pPktDesc,swInfo1);

}


/**
 *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdSetL4CkSumAHCrypPort  Update L4 checksum,Crypto and
 *         outgoing EMAC port for IPSec AH packet to NetCP command.
 *
 *  @details Inline macro API can be used to do dynamic update for Crypto
 *           in IPSec AH mode and L4  checksum offload command to
 *           NetCP before transmitting packet out of specific EMAC Port.
 *           In this case application is  expected to provide full packet
 *           including protocol headers and  call Qmss_queuePushXX API
 *           to transmit the packet out.
 *           QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *          Application would also need to do required cache flush operation in
 *          the case of non cache coherent architecure. Refer nwal_send()
 *          for sample implementation
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[in]  pTxPSCmdInfo    Static Command Label @see nwalTxPSCmdInfo_t
 *                              created during API call
 *                              @see nwal_initPSCmdInfo
 *  @param[in]  l4OffBytes      Offset to L4 Header
 *  @param[in]  l4Len           Layer 4 payload length including header +
 *                              payload
 *  @param[in]  pseudoHdrChecksum   Pseudo Header checksum for L4
 *  @param[in]  saOffBytes      Offset from base of the packet to the header
 *                              of protocol per the following list:
 *                                  - IPSEC ESP with AH: IP header
 *                                  - IPSEC ESP: ESP Header
 *  @param[in]  saPayloadLen    Length of the payload starting from
 *                              saOffBytes to the end of the protocol
 *  @param[in]  swInfo0         Sw Info with SA context related details
 *  @param[in]  swInfo1         Sw Info with SA context related details
 *  @param[in]  saAhIcvOffBytes Offset to the ICV field in the case of
 *                              IPSec AH mode for authentication tag insertion
 *                               by NETCP.
 *  @param[in]  saAhMacSize     Size of the authentication tag to be inserted by
 *                              NetCP in the case of IPSec AH mode. Reset to
 *                              zero in the case of ESP mode.
 *                              Application also need to make sure that size is
 *                              not more than max value
 *                              @ref NWAL_IPSEC_AH_MAX_AUTH_TAG_BYTES
 *  @param[in]  enetPort        Transmit Enet Port
 *  @pre        @see nwal_getSecAssoc @see nwal_initPSCmdInfo
 */
static inline void
            nwal_mCmdSetL4CkSumAHCrypPort(Ti_Pkt*               pPkt,
                                         nwalTxPSCmdInfo_t*    pTxPSCmdInfo,
                                         uint16_t              l4OffBytes,
                                         uint16_t              l4Len,
                                         uint16_t              pseudoHdrChecksum,
                                         uint16_t              saOffBytes,
                                         uint16_t              saPayloadLen,
                                         uint32_t              swInfo0,
                                         uint32_t              swInfo1,
                                         uint16_t              saAhIcvOffBytes,
                                         uint16_t              saAhMacSize,
                                         nwal_enetPort_t       enetPort)
{

    pasahoComChkCrc_t*      ptx;
    Cppi_HostDesc*          pPktDesc;
    uint8_t*                pPsDataBuf;
    pasahoShortInfo_t*      sInfo;
    pasahoNextRoute_t*      nr;
    pasahoComBlindPatch_t*  pBpCmd;

    nwal_mCmdUpdatePSData(pPkt,pTxPSCmdInfo,&pPktDesc,&pPsDataBuf);

    /* L4 checksum command need to be updated with
     * Offset to the L4 header
     * new pseudo header checksum and
     * L4 length
     */
    ptx = (pasahoComChkCrc_t *)&pPsDataBuf[pTxPSCmdInfo->udpChkOff];

    /* Update Layer 4 Offset bytes */
    PASAHO_CHKCRC_SET_START      (ptx, l4OffBytes);

    /* Update Length */
    PASAHO_CHKCRC_SET_LEN (ptx,l4Len);

    /* Update Pseudo header checksum */
    PASAHO_CHKCRC_SET_INITVAL    (ptx, pseudoHdrChecksum);

    sInfo = (pasahoShortInfo_t *)&pPsDataBuf[pTxPSCmdInfo->saShortInfoOff];
    PASAHO_SINFO_SET_PAYLOAD_OFFSET(sInfo, saOffBytes);
    PASAHO_SINFO_SET_PAYLOAD_LENGTH(sInfo, saPayloadLen);


    nr = (pasahoNextRoute_t *)&pPsDataBuf[pTxPSCmdInfo->pa2saNextRoute];
    nr->swInfo0 = swInfo0;
    nr->swInfo1 = swInfo1;

    pBpCmd = (pasahoComBlindPatch_t *)&pPsDataBuf[pTxPSCmdInfo->ahPathCmdOff];
    PASAHO_BPATCH_SET_PATCH_NBYTES  (pBpCmd, saAhMacSize);
    PASAHO_BPATCH_SET_OFFSET        (pBpCmd, saAhIcvOffBytes);

    nwal_modNREnetPort(pPsDataBuf,
                       pTxPSCmdInfo->enetPortNextRoute,
                       enetPort);

}

#ifdef NWAL_ENABLE_SA
/**
 *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mCmdDMUpdate  Update PS Command label for Side Band Data Mode
 *
 *  @details Inline macro API can be used to do dynamic update for Side Band
 *           Data Mode SA.
 *           In this case application is expected to provide full packet
 *           including protocol headers and call Qmss_queuePushXX API to
 *           transmit the packet out.
 *           QMSS push operation is being separated to:
 *              - give flexibility to application for TX time and packet
 *                preparation
 *              - Have control on either doing raw push w/o virtual to
 *                physical address translation
 *          Application would also need to do required cache flush operation in
 *          the case of non cache coherent architecure. Refer @see nwal_sendDM()
 *          for sample implementation
 *
 *  @param[in]  pPkt            Payload to be transmitted to SA
 *  @param[in]  pTxDmPSCmdInfo  Command Label Meta Data returned from
 *                              @see nwal_initDMPSCmdInfo
 *  @param[in]  appCtxId        Application context ID identifying
 *                              the data mode request. The 32 bit ID
 *                              will be received as echo back for the
 *                              response from NetCP through
 *                              @ref nwal_mmGetDmAppCtxId
 *  @param[in]  encOffset       Specify the offset to the encrypted/decrypted
 *                              data in the packet in bytes
 *  @param[in]  encSize         Specify the total number of bytes  to be
 *                              encrypted or decrypted
 *  @param[in]  pEncIV;         Contain the initialization vectors
 *                              used in certain encryption modes.
 *                              @note: IV should be specified here
 *                              in GCM/CCM mode
 *  @param[in]   authOffset;    Specify the offset to the
 *                              authenticated data in the packet
 *                              in bytes
 *  @param[in]  authSize;       Specify the total number of bytes
 *                              to be authenticated
 *  @param[in]   pAuthIV;       Contain the initialization vectors
 *                              used in certain authentication
 *                              modes.
 *                              @note: IV should be specified
 *                              here in GMAC mode
 *  @param[in]   aadSize;       Size of additional authenticated
                                data in bytes
 *  @param[in]   pAad;          Contain the additional
                                authenticated data in
                                GCM/CCM modes
 *  @param[in]
 *  @pre        @see nwal_setDMSecAssoc @see nwal_initDMPSCmdInfo
 */
static inline void  nwal_mCmdDMUpdate (Ti_Pkt*                  pPkt,
                                       nwalTxDmPSCmdInfo_t*     pTxDmPSCmdInfo,
                                       nwal_AppId               appCtxId,
                                       uint8_t                  encOffset,
                                       uint16_t                 encSize,
                                       uint8_t*                 pEncIV,
                                       uint8_t                  authOffset,
                                       uint16_t                 authSize,
                                       uint8_t*                 pAuthIV,
                                       uint16_t                 aadSize,
                                       uint8_t*                 pAad)
{
    uint8_t*                pDataBuf;
    uint32_t                dataLen;
    Cppi_HostDesc*          pPloadDesc;
    uint32_t*               pTmpCmdLb;

    pPloadDesc = Pktlib_getDescFromPacket(pPkt);

    /* Update the Static PS command to the descriptor */
    pTmpCmdLb = (uint32_t *)
                 Cppi_setPSData(Cppi_DescType_HOST,
                                (Cppi_Desc *)pPloadDesc,
                                (uint8_t *)pTxDmPSCmdInfo->pCmdLb,
                                pTxDmPSCmdInfo->cmdLbLen);

    /* Updated Dynamic per packet info in the PS command */
    Pktlib_getDataBuffer(pPkt,&pDataBuf,&dataLen);
    sa_mDmUpdateCmdLb(encOffset,
                      encSize,
                      pEncIV,
                      authOffset,
                      authSize,
                      pAuthIV,
                      aadSize,
                      pAad,
                      pDataBuf,
                      pTxDmPSCmdInfo->pUpdateInfo,
                      pTmpCmdLb);

    /* Configure return Queue and Flow ID for packet returned from SA */
    sa_SWINFO_UPDATE_DEST_INFO(pTxDmPSCmdInfo->pSwInfo,
                               (uint16_t)(pTxDmPSCmdInfo->rxSbSaQ),
                               pTxDmPSCmdInfo->rxPktFlowId);
    Cppi_setSoftwareInfo (Cppi_DescType_HOST,
                          (Cppi_Desc *)pPloadDesc,
                          (uint8_t *)pTxDmPSCmdInfo->pSwInfo);

    /* Store additional Application context
     * Timestamp field in descriptor is being overloaded for storing
     * additional Application context
     */
     Cppi_setTimeStamp(Cppi_DescType_HOST,
                       (Cppi_Desc *)pPloadDesc,
                       (uint32_t)appCtxId);
}
#endif
/*********************** NWAL FAST SEND HELPER MACROS END ******************/

/*********************** NWAL FAST RECEIVE HELPER MACROS BEGIN *************/
/******* NWAL FAST RECEIVE INLINE MACROS ***/
/**
 *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mmGetDmAuthTag  Get Authentication Tag for Data Mode SA packet
 *
 *  @details Inline macro API to retrieve the Authentication tag for Data Mode
 *           SA packet.
 *
 *  @param[in]  pPkt            Packet received after crypto action from NetCP
 *  @param[in]  ppAuthTag       Authentication Tag for the packet
 *  @param[in]  pAuthTagLen     Length of authentication tag
 *  @pre        @see nwal_setDMSecAssoc @see nwal_sendDM
 */
static inline void
            nwal_mmGetDmAuthTag (Ti_Pkt*               pPkt,
                                 uint8_t**             ppAuthTag,
                                 uint32_t*             pAuthTagLen)
{
    Cppi_getPSData (Cppi_DescType_HOST,
                    Cppi_PSLoc_PS_IN_DESC,
                    (Cppi_Desc *)pPkt,
                    ppAuthTag,
                    pAuthTagLen);
}

/**
 *  @ingroup nwal_fast_send_macros
 *  @brief nwal_mmGetDmAppCtxId  Get Application Context for Data Mode SA packet
 *
 *  @details Inline macro API to retrieve the Application Context for Data Mode
 *           SA packet.
 *
 *  @param[in]  pPkt            Packet received after crypto action from NetCP
 *  @param[in]  pAppCtxId       Application Context Id which was passed during
 *                              @see nwal_sendDM
 *  @pre        @see nwal_setDMSecAssoc @see nwal_sendDM
 */
static inline void
            nwal_mmGetDmAppCtxId (Ti_Pkt*               pPkt,
                                  uint32_t*             pAppCtxId)
{
    Cppi_getTimeStamp(Cppi_DescType_HOST,
                      (Cppi_Desc *)pPkt,
                      pAppCtxId);
}

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetAppidFmPkt Get App ID from an incoming packet from NetCP
 *
 *  @details Inline macro API to get AppID from the incoming packet from
 *           NetCP.
 *
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @param[out] pAppId          Application ID returned if found
 *  @retval     @see nwal_TRUE if found / @see nwal_FALSE if not found in packet
 *  @pre        Application should have received the packet through poll
 */
static inline nwal_Bool_t nwal_mGetAppidFmPkt (Ti_Pkt*               pPkt,
                                               nwal_AppId*           pAppId)
{
    Cppi_HostDesc*      pPktDesc;
    uint32_t*           pSwInfo0;

    pPktDesc = Pktlib_getDescFromPacket(pPkt);
    if (Cppi_getSoftwareInfo(Cppi_DescType_HOST,
                              (Cppi_Desc *)pPktDesc,
                              (uint8_t **) &pSwInfo0) == CPPI_EPIB_NOT_PRESENT)
    {
        return nwal_FALSE;
    }

    *pAppId = (nwal_AppId)(*pSwInfo0);
    return nwal_TRUE;
}

static inline uint8_t * nwalCppi_getPSData (Cppi_DescType descType, Cppi_PSLoc location, Cppi_Desc *descAddr, uint32_t *dataLen)
{
    Cppi_HostDesc   *hostDescPtr = (Cppi_HostDesc *) descAddr;
    uint8_t         epibPresent;
    uint8_t         *pPSData;


    if ((*dataLen = CSL_FEXTR (hostDescPtr->packetInfo, 29, 24) * 4) == 0)
    {
        return NULL;
    }

    epibPresent = CSL_FEXTR (hostDescPtr->packetInfo, 31, 31);

    if (descType == Cppi_DescType_HOST)
    {
        if (location == Cppi_PSLoc_PS_IN_SOP)
            pPSData = (uint8_t *) hostDescPtr->buffPtr;
        else
            pPSData = (uint8_t *) (((uint8_t *) &hostDescPtr->psData) - (!epibPresent * CPPI_HOST_DESC_EPIB_SIZE));
    }
    else
    {
        Cppi_MonolithicDesc *monolithicDescPtr = (Cppi_MonolithicDesc *) descAddr;
        pPSData = (uint8_t *) (((uint8_t *) &monolithicDescPtr->psData) - (!epibPresent * CPPI_MONOLITHIC_DESC_EPIB_SIZE));
    }

    return(pPSData);
}


/* find pointer to proto info fields in descriptor */
/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetProtoInfo Get Protocol Info data from an incoming packet
 *         from NetCP
 *
 *  @details Inline macro API to get Protocol Info from the incoming packet
 *           from NetCP.
 *
 *  @param[in]  pPkt     Packet to be transmitted out of NetCP
 *  @retval     Pointer to protocol Info to be passed for other utilities.
 *              Since no error check is being done application would need
 *              to make sure that a valid non Null is being returned before
 *              accessing other RX utilities
 *  @pre        Application should have received the packet through poll
 */
static inline  pasahoLongInfo_t* nwal_mGetProtoInfo(Ti_Pkt *    pPkt)
{
    uint32_t            infoLen = 0;
    Cppi_HostDesc*      pHd = Pktlib_getDescFromPacket(pPkt);
    pasahoLongInfo_t*   pProtoInfo = NULL;

    pProtoInfo = (pasahoLongInfo_t*)nwalCppi_getPSData (Cppi_DescType_HOST,
                                                        Cppi_PSLoc_PS_IN_DESC,
                                                        (Cppi_Desc *)pHd,
                                                        &infoLen);
#if 0
    /* Debug Code for GCC arm-2009q1 Compiler optimization load before store problem */
    if((pProtoInfo) == NULL)
    {
        printf("Null PS Data, Length: %d \n",infoLen);
    }
#endif

    return(pProtoInfo);
}

/** "protoInfo" below is return of NWAL_GET_PROTO_INFO() above**/

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetL3OffBytes Get offset to the L3 header.
 *  This is valid if packet had gone through Layer 2 classification at 
 *  NetCP
 *
 *  @details Inline macro API to get offset to the payload.
 *
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @retval     Offset to the layer 3 header in the packet
 *  @pre        Application should have received the packet through poll
 */
static inline uint16_t nwal_mGetL3OffBytes( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_READ_L3_OFFSET(protoInfo));
}


/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetL4Offset Get Offset to L4 protocol
 *
 *  @details Inline macro API to get offset to L4 protocol
 *
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @retval             L4 Offset Value
 *                       @see nwal_mGetProtoInfo
 *  @pre        Application should have received the packet through poll
 */
static inline uint16_t nwal_mGetL4Offset( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_READ_L4_OFFSET(protoInfo));
}


/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetL4ProtoType Get L4 protocol type
 *
 *  @details Inline macro API to get L4 protocol type.
 *
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @retval              L4 Protocol type
 *                       @see nwal_mGetProtoInfo
 *  @pre  @see nwal_mGetL4Offset Application should make sure that L4 offset is valid
 */
static inline uint16_t nwal_mGetL4ProtoType( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_READ_NXT_HDR_TYPE(protoInfo));
}

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetPloadOffBytes Get offset to the payload.
 *   This is valid if packet had gone through Layer 4 classification.
 *
 *  @details Inline macro API to get offset to the payload.
 *
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @retval     Offset to the payload in the packet
 *  @pre        Application should have received the packet through poll
 */
static inline uint16_t nwal_mGetPloadOffBytes( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_READ_L5_OFFSET(protoInfo));
}

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetPloadLen Get length of the payload
 *   This is valid if packet had gone through Layer 4 classification at NetCP
 *
 *  @details Inline macro API to get offset to the payload
 *
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @retval     Length of the payload in the packet
 *  @pre        Application should have received the packet through poll
 */
static inline uint16_t nwal_mGetPloadLen( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_READ_END_OFFSET(protoInfo) -
           PASAHO_LINFO_READ_L5_OFFSET(protoInfo));
}

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetCryptoStatus Get the status of crypto action by NetCP
 *         for incoming packet for 
 *
 *  @details Inline macro API to get crypto status of the packet 
 *
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @retval     Flag with indication of IPSec Crypto status
 *  @pre        Application should have received the packet through poll
 */
static inline nwal_rxFlag1_t
nwal_mGetCryptoStatus( pasahoLongInfo_t* protoInfo)
{
    nwal_rxFlag1_t rxFlag1=0;
    if((PASAHO_LINFO_IS_IPSEC_ESP(protoInfo)) ||
       (PASAHO_LINFO_IS_IPSEC_AH(protoInfo)))
    {
        rxFlag1 |= (NWAL_RX_IPSEC_CRYPTO_DONE_OK |
                    NWAL_RX_IPSEC_WINDOW_DONE_OK);
    }
    return(rxFlag1);
}

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mIsMacBroadcast Return nwal_TRUE for MAC broadcast packet
 *
 *  @details Inline macro API to check for MAC broadcast packet
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @pre        Application should have received the packet through poll
 */
static inline nwal_Bool_t
nwal_mIsMacBroadcast( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_IS_MAC_BROADCAST(protoInfo));
}

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mIsFragmentPkt Return nwal_TRUE for IP fragment packet received
 *
 *  @details Inline macro API to check for IP Fragment packet
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @pre        Application should have received the packet through poll
 */
static inline nwal_Bool_t
nwal_mIsFragmentPkt( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_READ_FLAG_FRAG(protoInfo));
}


/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mIsMacMulticast Return nwal_TRUE for MAC Multicast packet
 *
 *  @details Inline macro API to check for MAC multicast packet
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @pre        Application should have received the packet through poll
 */
static inline nwal_Bool_t
nwal_mIsMacMulticast( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_IS_MAC_MULTICAST(protoInfo));
}

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mGetRxEmacPort Get the incoming port id for the packet

 *
 *  @details Inline macro API to get incoming port id for the packet
 *
 *  @param[in] protoInfo Non Null Pointer to protocol Info returned from
 *                       @see nwal_mGetProtoInfo
 *  @retval     Emac port for incoming packet @see nwal_enetPort_t
 *  @pre        Application should have received the packet through poll
 */
static inline nwal_macPktType_t
nwal_mGetRxEmacPort( pasahoLongInfo_t* protoInfo)
{
    return(PASAHO_LINFO_READ_INPORT(protoInfo));
}

/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mIsL3CksumStatusPass Layer 3 Checksum result is PASS
 *
 *  @details Inline macro API to check if Layer3 Checksum result is PASS
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @retval     nwal_TRUE for successful checksum result, nwal_FALSE for failure
 *  @pre        Application should have received non zero L3 offset bytes
 *              @see nwal_mGetL3OffBytes
 */
static inline nwal_Bool_t nwal_mIsL3CksumStatusPass(Ti_Pkt*     pPkt)
{
    uint32_t            eflags = 0x0;
    eflags =
        (Cppi_getDescError (Cppi_DescType_HOST,
                            (Cppi_Desc *)Pktlib_getDescFromPacket(pPkt))) &
                            0xf;
    if(eflags & NWAL_PA_IPV4_HDR_CHKSUM_ERR)
    {
        return nwal_FALSE;
    }

    return nwal_TRUE;
}


/**
 *  @ingroup nwal_recv_macros
 *  @brief nwal_mIsL4CksumStatusPass Layer 4 Checksum result is PASS
 *
 *  @details Inline macro API to check if Layer4 Checksum result is PASS
 *  @param[in]  pPkt            Packet to be transmitted out of NetCP
 *  @retval     @see nwal_TRUE for successful checksum result,
 *              @see nwal_FALSE for failure
 *  @pre        Application should have received non zero L4 offset bytes
 *              @see nwal_mGetPloadOffBytes
 */
static inline nwal_Bool_t nwal_mIsL4CksumStatusPass(Ti_Pkt*     pPkt)
{
    uint32_t            eflags = 0x0;
    eflags =
        (Cppi_getDescError (Cppi_DescType_HOST,
                            (Cppi_Desc *)Pktlib_getDescFromPacket(pPkt))) &
                            0xf;
    if(eflags & NWAL_PA_L4_CHKSUM_ERR)
    {
        return nwal_FALSE;
    }
    return nwal_TRUE;
}
#ifdef __cplusplus
}
#endif


#endif  /* _NWAL_UTIL_H */
