/******************************************************************************
 * FILE PURPOSE:  Main Routines for NWAL LLD SA Adaptation functions
 ******************************************************************************
 * FILE NAME:   nwal_sec.c
 *
 * DESCRIPTION: NWAL LLD run time API and internal functions which is SA LLD
 *              dependent
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2015
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

#include <string.h>
#include "nwal_util.h"
#include "nwal_osal.h"
#include "nwal_netcp.h"
#ifdef NWAL_ENABLE_SA
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/sa/salld.h>
#include <ti/drv/sa/sa_osal.h>
#include <ti/csl/csl_qm_queue.h>
#include <stdio.h>

extern nssGlobalConfigParams_t nssGblCfgParams;
extern nwalProcessContext_t     nwalProcessCtx;

/*****************************************************************************
 * FUNCTION PURPOSE: Implements SALLD's debugInfo() API call.
 ******************************************************************************
 * DESCRIPTION: Handles SALLD API calls to debugInfo(). Treated as unexpected
 *              error for any further debugging. Expected only during
 *              configuration
 *
 * RETURN VALUES:
 *   None
 *   queue
 *****************************************************************************/
void nwalSalldDebugInfo (   void*       mID,
                            uint16_t    msgType,
                            uint16_t    messageCode,
                            uint16_t    msgLength,
                            uint16_t*   pSupportingData)
{

  /* Not expected.Based on the SA LLD it will never be called for IPSEC.
   * Need to relook if it has to be filled in
   */
  while(1);
}

/********************************************************************
 * FUNCTION PURPOSE: SA LLD callout for new security key request
 ********************************************************************
 * DESCRIPTION: Returns nwal_TRUE if both arrays match
 ********************************************************************/
void nwalSalldChanKeyRequest (Sa_ChanHandle     handle,
                              Sa_KeyRequest_t*  pKeyReq)
{
    /* Stub function. Not expected to be called for IPSec.
     * Block if called
     */
    while(1);
}

/********************************************************************
 * FUNCTION PURPOSE: SA LLD callout to externally supplied system to
 *          allocate the security context with the specified size.
 *          Cache handling done at the the function invoking SA LLD API
 ********************************************************************
 * DESCRIPTION: Returns nwal_TRUE if both arrays match
 ********************************************************************/
void nwalSalldScAlloc (Sa_ChanHandle    handle,
                       Sa_ScReqInfo_t*  pScReqInfo)
{
    nwalSaComHdrInfo_t *    pSaChanHdr  =
                            (nwalSaComHdrInfo_t *)Sa_chanGetID(handle);
    uint16_t                id          =   pSaChanHdr->index;

    if(pScReqInfo->scSize > NWAL_SEC_CONTEXT_BUF_SIZE)
    {
        /* Unexpected error. Block for debugging */
        while(1);
    }

    pScReqInfo->scID = id;
    if((pSaChanHdr->saChanState & NWAL_CHAN_SA_SC_BUF_ALLOCATED) ==
        NWAL_CHAN_SA_SC_BUF_ALLOCATED)
    {
        /* Unexpected error. Block for debugging TBD:
         * Multiple security context
         */
        while(1);
    }
    unsigned int globAddrTmp = NWAL_locToGlobAddr((unsigned int)pSaChanHdr->pScBuf); 
    pScReqInfo->scBuf =
           (uint8_t*) (NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
           globAddrTmp));
    pSaChanHdr->saChanState |= NWAL_CHAN_SA_SC_BUF_ALLOCATED;
}

/********************************************************************
 *  FUNCTION PURPOSE: Callout to externally supplied system to release
 *  the security context with the specified ID.
 *  Cache handling done at the the function invoking SA LLD API
 ********************************************************************/
void nwalSalldScFree (Sa_ChanHandle handle,
                      uint16_t      scID)
{
    nwalSaComHdrInfo_t *   pSaChanHdr  =
                           (nwalSaComHdrInfo_t *)Sa_chanGetID(handle);
    pSaChanHdr->saChanState =
        (pSaChanHdr->saChanState & ~ NWAL_CHAN_SA_SC_BUF_ALLOCATED);
}

/********************************************************************
 *  FUNCTION PURPOSE: Callout to externally supplied system to register
 *  the security channel with its software routing information.
 *  Cache handling done at the the function invoking SA LLD API
 *
 ********************************************************************
 ********************************************************************
 */
void nwalSalldChanRegister (Sa_ChanHandle   handle,
                            Sa_SWInfo_t*    pSwInfo)
{
    nwalSaComHdrInfo_t *    pSaChanHdr  =
                            (nwalSaComHdrInfo_t *)Sa_chanGetID(handle);

    pSaChanHdr->regSwInfo = *pSwInfo;
    pSaChanHdr->saChanState |= NWAL_CHAN_SA_REGISTERED;
}

/********************************************************************
 *  FUNCTION PURPOSE: Callout to externally supplied system to unregister
 *  the security channel with its software routing information.
 *  Cache handling done at the the function invoking SA LLD API
 *
 ********************************************************************
 ********************************************************************
 */
void nwalSalldChanUnRegister (Sa_ChanHandle handle,
                              Sa_SWInfo_t*  pSwInfo)
{
    nwalSaComHdrInfo_t *    pSaChanHdr  =
                            (nwalSaComHdrInfo_t *)Sa_chanGetID(handle);
    pSaChanHdr->saChanState = NWAL_CHAN_SA_UNREGISTERED;
}

/********************************************************************
 *  FUNCTION PURPOSE: Send an Null packet to the SA sub-system
 *  Cache handling done at the the function invoking SA LLD API
 *
 ********************************************************************
 ********************************************************************
 */
void nwalSalldChanSendNullPkt (Sa_ChanHandle    handle,
                               Sa_PktInfo_t     *pktInfo)
{

    nwalBufPool_t*          pLinkBufTxQ;
    Cppi_HostDesc*          pHd;
    Sa_SWInfo_t*            pSwInfo = &pktInfo->swInfo;
    nwalLocContext_t*       pLocContext;
    Ti_Pkt*                 pPkt = NULL;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        nwal_debug_bk();
        return;
    }

    pLinkBufTxQ = nwal_getLinkedBufQ(pLocContext->cfg.txCtlPool.bufPool,
                                     pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES,
                                     pLocContext->cfg.txCtlPool.numBufPools);
    /* Allocate a packet from heap. The API also does invalidation of packet
     * By default packets will be returned to the return queue for packet lib
     * since no cloning or merge is being done
     */
    while((pPkt = Pktlib_allocPacket((pLinkBufTxQ)->heapHandle,
                            pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES)) == NULL);

    pHd = Pktlib_getDescFromPacket(pPkt);
    Pktlib_setPacketLen(pPkt,0);
    Pktlib_setDataBufferLen(pPkt,0);

    Cppi_setPSLen(Cppi_DescType_HOST, (Cppi_Desc *)pHd,  0);
    Cppi_setSoftwareInfo (Cppi_DescType_HOST,
                          (Cppi_Desc *)pHd,
                          (uint8_t  *)pSwInfo->swInfo);

#ifndef __ARMv7
    /* Update the Cache */
    Pktlib_writebackPkt(pPkt);
#endif

    /* Send the command to PA. This will result in 1 packet in classify1 */
    Qmss_queuePushDescSize (nwalProcessCtx.pSharedMemBase->txQ[NSS_SA_QUEUE_SASS_INDEX],
                            (Ptr)pHd,
                            NWAL_DESC_SIZE);
}

/********************************************************************
 * Global SALLD call-out function table
 ********************************************************************/
static Sa_CallOutFuncs_t nwalSalldCalloutFunc = {
    nwalSalldDebugInfo,         /* Debug function pointer */
    nwalSalldChanKeyRequest,    /* Key Request function Pointer */
    nwalSalldScAlloc,           /* Security Context Allocation
                                 * function pointer
                                 */
    nwalSalldScFree,            /* Security Context Free Function pointer */
    nwalSalldChanRegister,      /* Channel Registration Function pointer */
    nwalSalldChanUnRegister,    /* Channel UnRegister Function pointer */
    nwalSalldChanSendNullPkt    /* Channel Send Null Packet function pointer */
};

/*****************************************************************************
 * FUNCTION PURPOSE: Function to retrieve SA related Buffer requirement
 *****************************************************************************
 * DESCRIPTION: Function to retrieve SA related Buffer requirement
 *****************************************************************************/
nwal_RetValue nwal_getSaBufferReq(nwalSizeInfo_t*       pSizeInfo,
                                  uint16_t              cacheLineSize,
                                  int*                  pSizes,
                                  int*                  pAligns)
{
    Sa_ChanSizeCfg_t    saChanSize;
    Sa_SizeCfg_t        sizeCfg;
    int                 salldSizes[sa_N_BUFS];
    int                 count;
    int                 salldAligns[sa_N_BUFS];
    int                 saSizes[sa_CHAN_N_BUFS];
    int                 saAligns[sa_CHAN_N_BUFS];
    int16_t             retCode;

    /* Memory requirement for SALLD */
    /* Get SALLD buffer requirements and ensure #buffers within limits */
    memset(&sizeCfg,0,sizeof(sizeCfg));
    sizeCfg.nMaxChan =
        pSizeInfo->nMaxIpSecChannels + pSizeInfo->nMaxDmSecChannels;
    if(!sizeCfg.nMaxChan)
    {
        /* No IPSec Channels to be configured */
        return (nwal_OK);
    }
        
    sizeCfg.cacheLineSize = cacheLineSize;
    retCode = Sa_getBufferReq (&sizeCfg, salldSizes, salldAligns);
    if(retCode != sa_ERR_OK)
        return (nwal_ERR_MEM_ALLOC);
    /* Request for one contiuguous block of memory from application */
    pSizes[nwal_BUF_INDEX_SA_LLD_HANDLE] = 0;
    for(count=0;count < sa_N_BUFS;count++)
    {
        pSizes[nwal_BUF_INDEX_SA_LLD_HANDLE] += salldSizes[count];
    }
    if(salldAligns[0] > cacheLineSize)
    {
        pAligns[nwal_BUF_INDEX_SA_LLD_HANDLE] = salldAligns[0];
    }
    else
    {
        pAligns[nwal_BUF_INDEX_SA_LLD_HANDLE] = cacheLineSize;
    }

    /* Memory requirement for SA LLD context per channel */
    pSizes[nwal_BUF_INDEX_SA_CONTEXT]=
       ((pSizeInfo->nMaxIpSecChannels + pSizeInfo->nMaxDmSecChannels) *
         nwal_round_size(cacheLineSize,1,NWAL_SEC_CONTEXT_BUF_SIZE));
    pAligns[nwal_BUF_INDEX_SA_CONTEXT] = cacheLineSize;

    /* Query SA LLD for IPSEC ESP Channel memory requirement */
    memset(&saChanSize,0,sizeof(saChanSize));
    saChanSize.protocolType = sa_PT_IPSEC_ESP;
    saChanSize.cacheLineSize = cacheLineSize;
    retCode = Sa_chanGetBufferReq (&saChanSize, saSizes, saAligns);
    if(saAligns[0] > cacheLineSize)
    {
        pAligns[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE] = saAligns[0];
    }
    else
    {
        pAligns[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE] = cacheLineSize;
    }
    pSizes[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE] =
       ((pSizeInfo->nMaxIpSecChannels + pSizeInfo->nMaxDmSecChannels) *
         nwal_round_size(pAligns[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE],
                         1,
                         saSizes[0]));
    return (nwal_OK);
}
/********************************************************************
 * FUNCTION PURPOSE: Function to do Global Initialization of SA
 *                    resources
 ********************************************************************
 * DESCRIPTION: Function to do Global Initialization of SA
 *              resources
 ********************************************************************/
nwal_RetValue
nwal_GlobSaCreate (  nwalGlobContext_t*     pIhandle,
                     const nwalGlobCfg_t*   pCfg,
                     nwalSizeInfo_t*        pSizeInfo,
                     int                    sizes[nwal_N_BUFS],
                     void*                  bases[nwal_N_BUFS],
                     int                    verifySizes[nwal_N_BUFS],
                     int                    verifyAligns[nwal_N_BUFS])
{
    Sa_ChanSizeCfg_t    saChanSize;
    int                 saSizes[sa_CHAN_N_BUFS];
    int                 saAligns[sa_CHAN_N_BUFS];
    nwalIpSecInfo_t*    pTmpIpSecInfo;
    nwalDmSaInfo_t*     pTmpDmSaInfo;
    uint8_t*            pTmpIpSecSaHandleBuf;
    uint8_t*            pTmpIpSecSaCxtBuf;
    uint8_t*            pTmpBuf;
    int16_t             retCode;
    Sa_SizeCfg_t        sizeCfg;
    int                 salldSizes[sa_N_BUFS];
    int                 salldAligns[sa_CHAN_N_BUFS];
    void*               salldBases[sa_N_BUFS];
    Sa_Config_t         lldCfg;
    int                 tmp_size;
    int                 count;
    uint16_t            cacheLineSize = NWAL_CACHE_LINE_SIZE;
    int                 perInstSize;
    uint8_t i;

    if(!(pSizeInfo->nMaxIpSecChannels + pSizeInfo->nMaxDmSecChannels))
    {
        /* There are no SA Channels to be configured. Skip SA LLD 
         * initialization
         */
        return nwal_OK;
    }
    

    /* NWAL IPSEC ESP Handle as Offset */
    pTmpIpSecInfo = 
        (nwalIpSecInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(pIhandle,
                                      pIhandle->saGlobContext.pIpSecInfo));
     
    pTmpIpSecSaHandleBuf = bases[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE];

    nwalProcessCtx.pSharedMemBase->saGlobContext.salld_handle = 
                            bases[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE];

    pTmpIpSecSaCxtBuf = bases[nwal_BUF_INDEX_SA_CONTEXT];

    /* Querry SA LLD for IPSEC ESP Channel memory requirement */
    memset(&saChanSize,0,sizeof(saChanSize));
    saChanSize.protocolType = sa_PT_IPSEC_ESP;
    saChanSize.cacheLineSize = cacheLineSize;
    Sa_chanGetBufferReq (&saChanSize, saSizes, saAligns);

    if(sizes[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE] <
        verifySizes[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE])
    {
       return (nwal_ERR_MEM_ALLOC);
    }

    if(sizes[nwal_BUF_INDEX_SA_CONTEXT] <
        verifySizes[nwal_BUF_INDEX_SA_CONTEXT])
    {
       return (nwal_ERR_MEM_ALLOC);
    }

    /* Channel Handle initialization for NWAL IPSec channel */
    perInstSize =
        nwal_round_size(verifyAligns[nwal_BUF_INDEX_INT_HANDLES],
                        1,
                        sizeof(nwalIpSecInfo_t));
    for(count =0;count < pSizeInfo->nMaxIpSecChannels;count++)
    {
        memset(pTmpIpSecInfo,0,sizeof(nwalIpSecInfo_t));
        pTmpIpSecInfo->hdr.handleHdr.handleId = NWAL_HANDLE_IPSEC_INST;
        pTmpIpSecInfo->hdr.handleHdr.stateCount =
            NWAL_SET_STATE(pTmpIpSecInfo->handleHdr.stateCount,
                           NWAL_STATE_INACTIVE);
        pTmpIpSecInfo->hdr.pNwalContext = pIhandle;
        pTmpIpSecInfo->hdr.saChanBufLen = nwal_round_size(cacheLineSize,
                                                      1,
                                                      saSizes[0]);

        /* pSaChanBuf and pScBuf needs to be an offset */
        pTmpIpSecInfo->hdr.pSaChanBuf = 
            (uint8_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
            pTmpIpSecSaHandleBuf));
        pTmpIpSecInfo->hdr.pScBuf = 
            (uint8_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
            pTmpIpSecSaCxtBuf));

        pTmpIpSecInfo->hdr.index = count;
        pTmpIpSecInfo->hdr.scBufLen = nwal_round_size(cacheLineSize,
                                                  1,
                                                  NWAL_SEC_CONTEXT_BUF_SIZE);
        /* Update Security context Buff */
        pTmpIpSecSaCxtBuf = pTmpIpSecSaCxtBuf + pTmpIpSecInfo->hdr.scBufLen;

        /* Update for next Security Channel */
        pTmpIpSecSaHandleBuf =
            (pTmpIpSecSaHandleBuf +
             nwal_round_size(cacheLineSize,
                             1,
                             saSizes[0]));

        NWAL_osalWriteBackCache(pTmpIpSecInfo,sizeof(nwalIpSecInfo_t));
        pTmpIpSecInfo = (nwalIpSecInfo_t*)
                        ((uint8_t *)(pTmpIpSecInfo) + perInstSize);

    }

    /* Channel Handle initialization for NWAL Data Mode channel */
    //pTmpDmSaInfo = pIhandle->saGlobContext.pDmSaInfo;
    pTmpDmSaInfo = 
        (nwalDmSaInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(pIhandle,
                                      pIhandle->saGlobContext.pDmSaInfo));
    perInstSize =
        nwal_round_size(verifyAligns[nwal_BUF_INDEX_INT_HANDLES],
                        1,
                        sizeof(nwalDmSaInfo_t));
    for(count =0;count < pSizeInfo->nMaxDmSecChannels;count++)
    {
        memset(pTmpDmSaInfo,0,sizeof(nwalDmSaInfo_t));
        pTmpDmSaInfo->hdr.handleHdr.handleId = NWAL_HANDLE_DM_INST;
        pTmpDmSaInfo->hdr.handleHdr.stateCount =
            NWAL_SET_STATE(pTmpDmSaInfo->handleHdr.stateCount,
                           NWAL_STATE_INACTIVE);
        pTmpDmSaInfo->hdr.pNwalContext = pIhandle;
        pTmpDmSaInfo->hdr.saChanBufLen = nwal_round_size(cacheLineSize,
                                                      1,
                                                      saSizes[0]);
        /* pSachanBuf needs to be an offset */
        pTmpDmSaInfo->hdr.pSaChanBuf = 
            (uint8_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
            pTmpIpSecSaHandleBuf));

        pTmpDmSaInfo->hdr.pScBuf = 
            (uint8_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                        pTmpIpSecSaCxtBuf));
        
        pTmpDmSaInfo->hdr.index = pSizeInfo->nMaxIpSecChannels + count;
        pTmpDmSaInfo->hdr.scBufLen = nwal_round_size(cacheLineSize,
                                                  1,
                                                  NWAL_SEC_CONTEXT_BUF_SIZE);
        /* Update Security context Buff */
        pTmpIpSecSaCxtBuf = pTmpIpSecSaCxtBuf + pTmpDmSaInfo->hdr.scBufLen;

        /* Update for next Security Channel */
        pTmpIpSecSaHandleBuf =
            (pTmpIpSecSaHandleBuf +
             nwal_round_size(cacheLineSize,
                             1,
                             saSizes[0]));

        NWAL_osalWriteBackCache(pTmpDmSaInfo,sizeof(nwalDmSaInfo_t));
        pTmpDmSaInfo =
            (nwalDmSaInfo_t*)((uint8_t *)(pTmpDmSaInfo) + perInstSize);

    }

    /* SA LLD Handle Initialization */
    memset(&sizeCfg,0,sizeof(sizeCfg));
    sizeCfg.nMaxChan =
        pSizeInfo->nMaxIpSecChannels + pSizeInfo->nMaxDmSecChannels;
    sizeCfg.cacheLineSize = cacheLineSize;
    if (nssGblCfgParams.layout.fNssGen2)
    {
        sizeCfg.ctrlBitMap |= sa_SIZE_CONFIG_SASS_GEN2;
    }
    retCode = Sa_getBufferReq (&sizeCfg, salldSizes, salldAligns);
    if(retCode != sa_ERR_OK)
        return (nwal_ERR_SA);

    pTmpBuf = bases[nwal_BUF_INDEX_SA_LLD_HANDLE];
    tmp_size = sizes[nwal_BUF_INDEX_SA_LLD_HANDLE];
    for(count=0;count<sa_N_BUFS;count++)
    {
        if(tmp_size < salldSizes[count])
            return (nwal_ERR_MEM_ALLOC);

        salldBases[count] = pTmpBuf;
        pTmpBuf += salldSizes[count];
        tmp_size = tmp_size - salldSizes[count];
    }

    /* Create SALLD system */
    memset(&lldCfg,0,sizeof(Sa_Config_t));
    lldCfg.ID  = 0xbabebeef;

    /* Update base address to virtual adddress if passed by application */
    if (nwalProcessCtx.baseAddrCfg.pSaVirtBaseAddr)
    {
        lldCfg.baseAddr = (uint32_t)nwalProcessCtx.baseAddrCfg.pSaVirtBaseAddr;
    }
    else
    {
        lldCfg.baseAddr = CSL_NETCP_CFG_SA_CFG_REGS;
    }

    lldCfg.callTable = &nwalSalldCalloutFunc;
    lldCfg.sizeConfig = &sizeCfg;
#ifndef NWAL_DISABLE_LINUX_MULTI_PROC_SA
    lldCfg.instPoolBaseAddr = nwalProcessCtx.baseAddrCfg.pInstPoolSaBaseAddr;
    lldCfg.scPoolBaseAddr = nwalProcessCtx.baseAddrCfg.pScPoolBaseAddr;
#endif
    if(!pIhandle->cfg.saHandle)
    {
        retCode =
            Sa_create (&lldCfg, salldBases, &pIhandle->saGlobContext.salld_handle);
        if(retCode != sa_ERR_OK)
        {
            return (nwal_ERR_SA);
        }
        /* Download the firmware if not aleady downloaded */
        if(pIhandle->cfg.saFwActive == nwal_FALSE )
        {
            /* Download SA PDSP Firmwares */
            if (Sa_resetControl (pIhandle->saGlobContext.salld_handle,
                                 sa_STATE_RESET)!= sa_STATE_RESET)
            {
                return (nwal_ERR_SA);
            }
            for ( i = 0; i < NSS_SA_NUM_PDSPS; i++)
            {
            
               if (Sa_downloadImage (pIhandle->saGlobContext.salld_handle,
                                i,
                                (Ptr)nssGblCfgParams.layout.saPdspImage[i], 
                                nssGblCfgParams.layout.saPdspImageSize[i]) != sa_ERR_OK)
                {
                    return (nwal_ERR_SA);
                }
            }

            if(Sa_resetControl (pIhandle->saGlobContext.salld_handle,
                                sa_STATE_ENABLE) != sa_STATE_ENABLE)
                return (nwal_ERR_SA);
        }
    }
    else
    {
        /* Start previously created SA LLD instance with provided SA handle */
        retCode = Sa_start(pIhandle->cfg.saHandle, &lldCfg);
    }
    return nwal_OK;

}

/********************************************************************
 * FUNCTION PURPOSE: Per Proc initialization for SA LLD
 ********************************************************************
 * DESCRIPTION: Per Proc initialization for SA LLD
 ********************************************************************/
nwal_RetValue nwal_SaStart (nwalGlobContext_t*     pIhandle)
{
    Sa_Config_t     lldCfg;
    int16_t         retCode;
    Sa_SizeCfg_t    sizeCfg;


    memset(&lldCfg,0,sizeof(Sa_Config_t));
    memset(&sizeCfg,0,sizeof(sizeCfg));
    lldCfg.ID  = 0xbabebeef;

    /* Update base address to virtual adddress if passed by application */
    if(nwalProcessCtx.baseAddrCfg.pSaVirtBaseAddr)
    {
        lldCfg.baseAddr = (uint32_t)nwalProcessCtx.baseAddrCfg.pSaVirtBaseAddr;
    }
    else
    {
        lldCfg.baseAddr = CSL_NETCP_CFG_SA_CFG_REGS;
    }
    lldCfg.callTable = &nwalSalldCalloutFunc;

    sizeCfg.nMaxChan =
        pIhandle->memSizeInfo.nMaxIpSecChannels + pIhandle->memSizeInfo.nMaxDmSecChannels;
    sizeCfg.cacheLineSize = NWAL_CACHE_LINE_SIZE;
    lldCfg.sizeConfig = &sizeCfg;
#ifndef NWAL_DISABLE_LINUX_MULTI_PROC_SA
    lldCfg.instPoolBaseAddr = nwalProcessCtx.baseAddrCfg.pInstPoolSaBaseAddr;
    lldCfg.scPoolBaseAddr = nwalProcessCtx.baseAddrCfg.pScPoolBaseAddr;
#endif

    retCode = Sa_start(pIhandle->saGlobContext.salld_handle,&lldCfg);
    if(retCode != sa_ERR_OK)
        return (nwal_ERR_SA);

    return nwal_OK;
}

/********************************************************************
 *  FUNCTION PURPOSE: Get a Free NWAL instance. Generic function to
 *                    get IpSec/Data Mode SA Channel Instance
 *                    Calling function would need to be ensure multicore
 *                    protection
 *                    Function being separated out from nwal_getInst()
 *                    in order to add additional checks for security
 *                    context for the SA channel
 ********************************************************************
 ********************************************************************/
nwal_Handle nwal_getSaInst( nwalGlobContext_t* pNwalContext,
                            nwal_Bool_t        isDataMode)
{
    uint16_t                count;
    nwalHandleHdr_t*        pHandleHdr;
    uint8_t*                pMem;
    NWAL_STATE_COUNT_T      state;
    uint32_t                instAdjSize;
    uint32_t                instSize;
    uint32_t                maxNumInstances;
    nwalSaComHdrInfo_t*     pSaComHdr;
    uint8_t*                pScBufAddr;
    if(isDataMode)
    {
        pMem =
            (uint8_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(pNwalContext,
                                      pNwalContext->saGlobContext.pDmSaInfo));
        maxNumInstances =
            pNwalContext->memSizeInfo.nMaxDmSecChannels;
        instSize = sizeof(nwalDmSaInfo_t);
    }
    else
    {
        pMem =
            (uint8_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(pNwalContext,
                                                   pNwalContext->saGlobContext.pIpSecInfo));
        maxNumInstances =
            pNwalContext->memSizeInfo.nMaxIpSecChannels;
        instSize = sizeof(nwalIpSecInfo_t);
    }
    instAdjSize = nwal_round_size(NWAL_CACHE_LINE_SIZE, 1, instSize);
    /* Retrieve next available IPSec Channel */
    for(count=0;count < maxNumInstances;count++)
    {
        pHandleHdr = (nwalHandleHdr_t*)(pMem);
        NWAL_osalInvalidateCache(pMem,instAdjSize);
        state = NWAL_GET_STATE(pHandleHdr->stateCount);
        if(state == NWAL_STATE_INACTIVE)
        {
            pSaComHdr = (nwalSaComHdrInfo_t*)(pMem);
            pScBufAddr = 
                (uint8_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(pNwalContext,
                pSaComHdr->pScBuf));

            NWAL_osalInvalidateCache(pScBufAddr,pSaComHdr->scBufLen);
            if(Sa_isScBufFree(pScBufAddr) == nwal_TRUE)
            {
                /* Set count to 1 */
                pHandleHdr->stateCount =
                    (NWAL_SET_STATE(pHandleHdr->stateCount,
                                    NWAL_STATE_CFG_IN_PROGRESS) | 1);
                NWAL_osalWriteBackCache(pMem,instAdjSize);
                return((nwal_Handle )pMem);
            }
        }

        pMem = pMem + instAdjSize;
    }
    return NULL;
}


/********************************************************************
 *  FUNCTION PURPOSE: Free an SA Instance preserving the static
 *                    information.
 ********************************************************************
 ********************************************************************/
void nwal_freeSaInst( nwal_Bool_t       isDataMode,
                      void*             saInfo)
{
    nwalSaComHdrInfo_t* pComHdrInfo =(nwalSaComHdrInfo_t*)saInfo;
    pComHdrInfo->appId = 0;
    pComHdrInfo->saChanHandle = 0;
    pComHdrInfo->handleHdr.stateCount =
        NWAL_SET_STATE(pComHdrInfo->handleHdr.stateCount,
                       NWAL_STATE_INACTIVE);
    memset(&pComHdrInfo->regSwInfo,0,sizeof(Sa_SWInfo_t));
    if(isDataMode)
    {
        nwalDmSaInfo_t *    pDmSaInfo = (nwalDmSaInfo_t *)saInfo;
        memset(&pDmSaInfo->dmChnType,
               0,
               (sizeof(nwalDmSaInfo_t)-sizeof(nwalSaComHdrInfo_t)));
    }
    else
    {
        nwalIpSecInfo_t*    pIpSecInfo = (nwalIpSecInfo_t*)saInfo;
        memset(&pIpSecInfo->dir,
               0,
               (sizeof(nwalIpSecInfo_t)-sizeof(nwalSaComHdrInfo_t)));
    }
}

/********************************************************************
 * FUNCTION PURPOSE: Configure PA for Outer IP Address
 ********************************************************************
 * DESCRIPTION: Function to configure outer IP address at PA
 ********************************************************************/
nwal_RetValue nwal_configIPSec
(
   nwalGlobContext_t*       pIhandle,
   nwalIpSecInfo_t*         pIpSecInfo,
   nwalLocRouteInfo_t*         pRouteInfo
)
{
    Cppi_HostDesc       *pHd;
    paReturn_t          paRet;
    paRouteInfo2_t      matchRoute2,failRoute2;
    paCmdReply_t        cmdReply;
    uint16_t            cmdSize = pa_ADD_IP_MIN_CMD_BUF_SIZE_BYTES;
    Int                 cmdDest;
    nwalMacInfo_t*      pMacInfo;
    nwalIpInfo_t*       pIpInfo;
    nwalHandleHdr_t*    pHndlHdr;
    nwal_RetValue       retVal;
    nwalLocContext_t*   pLocContext;
    paParamDesc         paramDesc;
    nwal_Handle         prevHandleAddr;

    pLocContext = nwal_getLocContext(pIhandle);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    memset(&paramDesc,0,sizeof(paParamDesc));


    
    
    if(pIpSecInfo->prevHandle != nwal_HANDLE_INVALID)
    {
        prevHandleAddr = 
            (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(pIhandle,
                                                      pIpSecInfo->prevHandle));
        nwal_InvPreHnd(prevHandleAddr);

        pHndlHdr = (nwalHandleHdr_t*)prevHandleAddr;
        if (pHndlHdr->handleId == NWAL_HANDLE_MAC_INST) {
            pMacInfo = (nwalMacInfo_t *)(prevHandleAddr);
            paramDesc.prevLink = pMacInfo->paMacHandle;
        } else {
            pIpInfo = (nwalIpInfo_t *)(prevHandleAddr);
            paramDesc.prevLink = pIpInfo->paIpHandle;
        }
        paramDesc.validBitMap |= pa_PARAM_VALID_PREVLINK;
    }

    retVal = nwal_prepCmdBuf(pIhandle,
                             &cmdSize,
                             &cmdReply,
                             &pIpSecInfo->transInfo,
                             &pHd);
    if(retVal != nwal_OK)
    {
        return retVal;
    }

    /* Get the Match and Route Info based on Application configuration
     * By default swinfo will indicate IP Sec handle maintained by NWAL.
     * in case if packets are expected to be redirected to host
     */
    retVal = nwal_convRouteInfo(pIhandle,
                                pLocContext,
                                pRouteInfo,
                                pa_DEST_SASS,
                                (uint32_t)pIpSecInfo->transInfo.appId,
                                NULL,
                                NULL,
                                &matchRoute2,
                                &failRoute2);
    if(retVal != nwal_OK)
    {
        return retVal;
    }
    if(pRouteInfo->matchAction == NWAL_MATCH_ACTION_CONTINUE_NEXT_ROUTE)
    {
        /* Default case of offloading SA to NetCP. Override the default
         * config
         */
        matchRoute2.queue = QMSS_PASS_QUEUE_BASE +
                            NSS_SA_QUEUE_SASS_INDEX;
        matchRoute2.swInfo0 = pIpSecInfo->hdr.regSwInfo.swInfo[0];
        matchRoute2.swInfo1 = pIpSecInfo->hdr.regSwInfo.swInfo[1];
        
        /* For PA to SA override to the one created by NWAL
         * Only for fail route application can configure to
         * non default flow ID
         */
        matchRoute2.flowId = pIhandle->rxPaSaFlowId;
    }

    paramDesc.routeInfo = &matchRoute2;
    paramDesc.nextRtFail = &failRoute2;
    paramDesc.index = pa_LUT1_INDEX_NOT_SPECIFIED;
    paramDesc.lutInst = pa_LUT_INST_NOT_SPECIFIED;

    paRet =  Pa_addIp2(pIhandle->memSizeInfo.pahandle,
                       &pIpSecInfo->paIpInfo,
                       &paramDesc,
                       &pIpSecInfo->paIpHandle,
                       (paCmd_t) pHd->buffPtr,
                       &cmdSize,
                       &cmdReply,
                       &cmdDest);

    if(paRet != pa_OK)
    {
        pLocContext->extErr = paRet;
        return nwal_ERR_PA;
    }

    if(cmdDest > NSS_NUM_TX_PKTDMA_CHANNELS)
    {
        return nwal_ERR_INVALID_CMD_DEST;
    }

    retVal =  nwal_txCmdBuf(pIhandle,
                            cmdSize,
                            cmdDest,
                            pHd);
    if(retVal != nwal_OK)
    {
        return retVal;
    }

    nwal_setTransInfo(&pIpSecInfo->transInfo,
                          NWAL_HANDLE_ID_TRANS_ADD_IPSEC,
                          pIpSecInfo);
    /*update global info; need to protect from multicore*/
    pLocContext->numPendPAReq++;

    pIpSecInfo->hdr.appId = pIpSecInfo->transInfo.appId;
    return (nwal_OK);
}

/********************************************************************
 * FUNCTION PURPOSE: Common utility function for SA configuration
 ********************************************************************
 * DESCRIPTION: Common utility function for SA configuration
 ********************************************************************/
nwal_RetValue nwal_utlSecAssoc(nwal_utlSecAssocParam_t* pParam)
{
    Sa_ChanSizeCfg_t        sizeCfg;
    Sa_ChanConfig_t         chanCfg;
    uint16_t                cacheLineSize = NWAL_CACHE_LINE_SIZE;
    int32_t                 saAligns[sa_CHAN_N_BUFS];
    void*                   bases[sa_CHAN_N_BUFS];
    int                     saSizes[sa_CHAN_N_BUFS];
    int16_t                 retCode;
    uint16_t                tmpSize;
    uint16_t                encryptionBlockSize = 0;
    uint16_t                sessionSaltSize = 0;
    uint16_t                ivSize = 0;
    uint16_t                authIvSize = 0;
    uint8_t                 count;

    memset(&sizeCfg, 0 ,sizeof(Sa_ChanSizeCfg_t));
    memset(&chanCfg, 0 ,sizeof(Sa_ChanConfig_t));

    sizeCfg.protocolType = pParam->proto;
    chanCfg.sizeConfig.protocolType = pParam->proto;
    sizeCfg.cacheLineSize = cacheLineSize;

    /* Get SALLD Channel buffer requirements and ensure #buffers
     * within limits
     */
    retCode = Sa_chanGetBufferReq (&sizeCfg,
                                    saSizes,
                                    saAligns);
    tmpSize = pParam->saChanBufLen;
    for (count = 0; count < sa_CHAN_N_BUFS; count++)
    {
        if(tmpSize < saSizes[count])
        {
            return(nwal_ERR_MEM_ALLOC);
        }

        bases[count] = 
                    (void*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                pParam->pSaChanBuf));
        tmpSize = tmpSize - saSizes[count];
    }
    
    /* Create (NEW) SALLD channel */
    chanCfg.ID  = pParam->saID;
    chanCfg.sizeConfig.cacheLineSize = cacheLineSize;
    retCode = Sa_chanCreate ( pParam->pNwalContext->saGlobContext.salld_handle,
                              &chanCfg,
                              bases,
                              pParam->pSaChanHandle);
    if(retCode != sa_ERR_OK)
    {
        return(nwal_ERR_SA);
    }

    /* Assign default encryptionBlockSize, ivSize and sessionSaltSize */
    if(pParam->proto == sa_PT_IPSEC_ESP)
    {
        encryptionBlockSize = 4;
        ivSize = 8;
        if (pParam->cipherMode == NWAL_SA_EALG_AES_CCM)
        {
            sessionSaltSize = 3;
        }
        else
        {
            sessionSaltSize = 4;
        }
    }
    else if(pParam->proto == sa_PT_IPSEC_AH)
    {
        encryptionBlockSize = 0;
        ivSize = 0;
        sessionSaltSize = 0;
    }
    else if(pParam->proto == sa_PT_NULL)
    {
        encryptionBlockSize = 0;
        ivSize = 8;
        sessionSaltSize = 0;
    }

    switch(pParam->cipherMode)
    {
        case NWAL_SA_EALG_NULL:
            pParam->pGenConfig->cipherMode = sa_CipherMode_NULL;
            *(pParam->pEncKeyReqd) = nwal_FALSE;
            encryptionBlockSize = 0;
            ivSize = 0;
            sessionSaltSize = 0;
            break;
        case NWAL_SA_EALG_AES_CTR:
            pParam->pGenConfig->cipherMode = sa_CipherMode_AES_CTR;
            if(pParam->proto == sa_PT_NULL)
            {
                ivSize = 16;
                sessionSaltSize = 0;
            }
            break;
        case NWAL_SA_EALG_AES_CBC:
            pParam->pGenConfig->cipherMode = sa_CipherMode_AES_CBC;
            encryptionBlockSize = 16;
            ivSize = 16;
            sessionSaltSize = 0;
            break;

        case NWAL_SA_EALG_DES_CBC:
        case NWAL_SA_EALG_3DES_CBC:
            pParam->pGenConfig->cipherMode = sa_CipherMode_3DES_CBC;
            encryptionBlockSize = 8;
            sessionSaltSize = 0;
            break;
        case NWAL_SA_EALG_AES_CCM:
            pParam->pGenConfig->cipherMode = sa_CipherMode_CCM;
            *(pParam->pAuthKeyReqd)  = nwal_FALSE;
            if(pParam->proto == sa_PT_NULL)
            {
                ivSize = 8;
                sessionSaltSize = 3;
            }
            break;
        case NWAL_SA_EALG_AES_GCM:
            pParam->pGenConfig->cipherMode = sa_CipherMode_GCM;
            *(pParam->pAuthKeyReqd)  = nwal_FALSE;
            if(pParam->proto == sa_PT_NULL)
            {
                ivSize = 8;
                sessionSaltSize = 4;
            }
            break;
        default:
            return(nwal_ERR_INVALID_PARAM);
    }

    switch(pParam->authMode)
    {
        case NWAL_SA_AALG_NULL:
            pParam->pGenConfig->authMode   = sa_AuthMode_NULL;
            *(pParam->pAuthKeyReqd)  = nwal_FALSE;
            break;
        case NWAL_SA_AALG_GMAC:
            pParam->pGenConfig->authMode   = sa_AuthMode_GMAC;
            encryptionBlockSize   = 4;
            if(pParam->proto == sa_PT_NULL)
            {
                authIvSize = 8;
                ivSize = 8;
            }
            else
            {
                ivSize = 8;
            }
            sessionSaltSize = 4;
            break;
        case NWAL_SA_AALG_HMAC_MD5:
            pParam->pGenConfig->authMode   = sa_AuthMode_HMAC_MD5;
            break;
        case NWAL_SA_AALG_HMAC_SHA1:
            pParam->pGenConfig->authMode   = sa_AuthMode_HMAC_SHA1;
            break;
        case NWAL_SA_AALG_HMAC_SHA2_224:
            pParam->pGenConfig->authMode   = sa_AuthMode_HMAC_SHA2_224;
            break;
        case NWAL_SA_AALG_HMAC_SHA2_256:
        case NWAL_SA_AALG_HMAC_SHA2_256_RFC4868:
            pParam->pGenConfig->authMode   = sa_AuthMode_HMAC_SHA2_256;
            break;
        case NWAL_SA_AALG_AES_XCBC:
            pParam->pGenConfig->authMode   = sa_AuthMode_AES_XCBC;
            break;
        default:
            return(nwal_ERR_INVALID_PARAM);
    }

    if(pParam->pEncryptionBlockSize)
    {
        *pParam->pEncryptionBlockSize = encryptionBlockSize;
    }

    if(pParam->pSessionSaltSize)
    {
        *pParam->pSessionSaltSize = sessionSaltSize;
    }

    if(pParam->pIvSize)
    {
        *pParam->pIvSize = ivSize;
    }

    if(pParam->pAuthIvSize)
    {
        *pParam->pAuthIvSize = authIvSize;
    }

    return (nwal_OK);
}
//#define NWAL_NETCP_TX_PKT_DEBUG
/********************************************************************
 * FUNCTION PURPOSE: IPSec Security Association creation API
 ********************************************************************
 * DESCRIPTION: IPSec Security Association creation API
 ********************************************************************/
nwal_RetValue nwal_setSecAssoc( nwal_Inst               nwalInst,
                                nwal_TransID_t          transId,
                                nwal_AppId              appId,
                                nwalSaIpSecId_t*        pSaId,
                                nwalCreateSAParams_t*   pCreateParam,
                                nwal_Handle*            pNwalSaHandle)
{
    nwalIpSecInfo_t*        pIpSecInfo;
    int16_t                 retCode;
    Sa_ChanCtrlInfo_t       chanCtrlInfo;
    Sa_GenCtrlInfo_t        *pGenCtrlInfo = &chanCtrlInfo.ctrlInfo.gen;
    Sa_KeyCtrlInfo_t        *pKeyCtrlInfo = &chanCtrlInfo.ctrlInfo.key;
    Sa_GenConfigParams_t    genCfgParam;
    Sa_IpsecKeyParams_t     ipSecKeyParam;
    nwalHandleHdr_t*        pHandleHdr;
    NWAL_STATE_COUNT_T      state;
    nwal_RetValue           retValue;
    uint8_t*                pSalt;
    nwal_Bool_t             encKeyReqd=nwal_TRUE;
    nwal_Bool_t             authKeyReqd=nwal_TRUE;
    nwalLocContext_t*       pLocContext;
    uint32_t                key;
    nwal_utlSecAssocParam_t utlSecAssocParam;
    nwalLocRouteInfo_t      routeInfo;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    /* Check if previous handle/MAC is already active */
    if(pCreateParam->h.macHandle != nwal_HANDLE_INVALID)
    {
        pHandleHdr = 
            (nwalHandleHdr_t *)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                        pCreateParam->h.macHandle));
        /* Get the state of the upper layer handle */
        state = NWAL_GET_STATE(pHandleHdr->stateCount);
        if(state != NWAL_STATE_ACTIVE)
        {
            return nwal_ERR_INVALID_PREV_HANDLE_STATE;
        }
    }

    NWAL_osalCsEnter(&key);
    pIpSecInfo = nwal_getSaInst(nwalProcessCtx.pSharedMemBase,nwal_FALSE);
    if(pIpSecInfo == NULL)
    {
        NWAL_osalCsExit(key);
        return nwal_ERR_RES_UNAVAILABLE;
    }
    memset(&pIpSecInfo->paIpInfo,0,sizeof(paIpInfo2_t));
    pIpSecInfo->paIpHandle = 0;


    pIpSecInfo->prevHandle = pCreateParam->h.macHandle;
    pIpSecInfo->hdr.pNwalContext = nwalProcessCtx.pSharedMemBase;

    /* Copy the configuration which will be used later */
     pIpSecInfo->saId.spi = pSaId->spi;
     pIpSecInfo->saId.proto =  pSaId->proto;
     memcpy(&pIpSecInfo->saId.src,&pSaId->src,sizeof(nwalIpAddr_t));
     memcpy(&pIpSecInfo->saId.dst,&pSaId->dst,sizeof(nwalIpAddr_t));

    /* Copy the destination MAC address to be used during header creation */
    memcpy(pIpSecInfo->remMacAddr,
           pCreateParam->saIpSecParam.remMacAddr,
           sizeof(nwalMacAddr_t));

    memset(pGenCtrlInfo,0,sizeof(Sa_GenCtrlInfo_t));

    /* Configure the security channel */
    memset(&genCfgParam,0,sizeof(Sa_GenConfigParams_t));

    memset(&utlSecAssocParam,0,sizeof(nwal_utlSecAssocParam_t));
    utlSecAssocParam.pNwalContext = nwalProcessCtx.pSharedMemBase;

    if(pSaId->proto == nwal_IpSecProtoESP)
    {
        utlSecAssocParam.proto = sa_PT_IPSEC_ESP;
    }
    else if(pSaId->proto == nwal_IpSecProtoAH)
    {
        utlSecAssocParam.proto = sa_PT_IPSEC_AH;
    }
    else if(pSaId->proto == nwal_IpSecProtoESPNATT)
    {
        /* Use ESP proto for NATT */
        utlSecAssocParam.proto = sa_PT_IPSEC_ESP;
    }
    else
    {
        retValue = nwal_ERR_INVALID_PARAM;
        goto ERR_nwal_setSecAssoc;
    }

    /* Register ID as IPSec Channel Handle */
    utlSecAssocParam.saID = (uint32_t)pIpSecInfo;

    utlSecAssocParam.saChanBufLen = pIpSecInfo->hdr.saChanBufLen;
    utlSecAssocParam.pSaChanBuf = pIpSecInfo->hdr.pSaChanBuf;
    utlSecAssocParam.pGenConfig = &genCfgParam;
    utlSecAssocParam.authMode = pCreateParam->saIpSecParam.authMode;
    utlSecAssocParam.cipherMode = pCreateParam->saIpSecParam.cipherMode;
    utlSecAssocParam.pSaChanHandle = &pIpSecInfo->hdr.saChanHandle;
    utlSecAssocParam.pAuthKeyReqd = &authKeyReqd;
    utlSecAssocParam.pEncKeyReqd = &encKeyReqd;
    utlSecAssocParam.pEncryptionBlockSize =
        &genCfgParam.params.ipsec.encryptionBlockSize;
    utlSecAssocParam.pIvSize = &genCfgParam.params.ipsec.ivSize;
    utlSecAssocParam.pSessionSaltSize =
        &genCfgParam.params.ipsec.sessionSaltSize;
    retValue = nwal_utlSecAssoc(&utlSecAssocParam);
    if(retValue != nwal_OK)
    {
        goto ERR_nwal_setSecAssoc;
    }

    pIpSecInfo->macSize = pCreateParam->saIpSecParam.macSize;
    pIpSecInfo->ivSize = genCfgParam.params.ipsec.ivSize;

    /* Check if keys are received */
    if((!(pCreateParam->keyParam.encKeySize) &&
         (*(utlSecAssocParam.pEncKeyReqd)))   ||
       (!(pCreateParam->keyParam.macKeySize)&&
         (*(utlSecAssocParam.pAuthKeyReqd))))
    {
        retValue = nwal_ERR_INVALID_KEY;
        goto ERR_nwal_setSecAssoc;
    }

    switch(pCreateParam->saIpSecParam.saMode)
    {
        case nwal_SA_MODE_TRANSPORT :
            genCfgParam.params.ipsec.transportType =
                                    sa_IPSEC_TRANSPORT_TRANSPORT;
            break;
        case nwal_SA_MODE_TUNNEL :
            genCfgParam.params.ipsec.transportType =
                                    sa_IPSEC_TRANSPORT_TUNNEL;
            break;
        default:
            retValue = nwal_ERR_INVALID_PARAM;
            goto ERR_nwal_setSecAssoc;
    }
    pIpSecInfo->saMode = pCreateParam->saIpSecParam.saMode;

    /* Configure the Flow and Destination Queue from SA PDSP */
    genCfgParam.destInfo.flowID = nwalProcessCtx.pSharedMemBase->rxSaPaFlowId;
    pIpSecInfo->dir = pCreateParam->saIpSecParam.dir;
    if(pIpSecInfo->dir == NWAL_SA_DIR_INBOUND)
    {
        if(nssGblCfgParams.layout.fNssGen2)
        {
            genCfgParam.destInfo.queueID =
                  QMSS_PASS_QUEUE_BASE + NSS_PA_QUEUE_FIREWALL2_INDEX;
        }
        else
        {
            if(pIpSecInfo->saMode == nwal_SA_MODE_TUNNEL)
            {
                genCfgParam.destInfo.queueID =
                  QMSS_PASS_QUEUE_BASE + NSS_PA_QUEUE_INNER_IP_INDEX;
            }
            else
            {
                genCfgParam.destInfo.queueID =
                  QMSS_PASS_QUEUE_BASE + NSS_PA_QUEUE_LUT2_INDEX;
            }
        }
    }
    else
    {
#ifdef NWAL_NETCP_TX_PKT_DEBUG
        genCfgParam.destInfo.queueID =
                nwalProcessCtx.pSharedMemBase->cfg.rxDefPktQ;
#else
        genCfgParam.destInfo.queueID =
                  QMSS_PASS_QUEUE_BASE + NSS_PA_QUEUE_TXCMD_INDEX;
#endif
    }
    /* swInfo is not expected to be used as packet is not expected
     * to reach host
     */
    genCfgParam.destInfo.swInfo0 = (uint32_t)appId;
    genCfgParam.destInfo.swInfo1 = (uint32_t)appId;

    if((pCreateParam->saIpSecParam.validParams & NWAL_SA_INFO_VALID_PARAM_ESN) ==
        NWAL_SA_INFO_VALID_PARAM_ESN)
    {
        genCfgParam.params.ipsec.ctrlBitMap = sa_IPSEC_CONFIG_ESN;
        genCfgParam.params.ipsec.esnLo = pCreateParam->saIpSecParam.esnLo;
        genCfgParam.params.ipsec.esnHi = pCreateParam->saIpSecParam.esnHi;
    }
    else
        genCfgParam.params.ipsec.esnLo = pCreateParam->saIpSecParam.esnLo;

    ipSecKeyParam.sessionSalt = NULL;
    if (pCreateParam->keyParam.encKeySize)
    {
        /* set the EncKeySize, over-ride if necessary */
        genCfgParam.params.ipsec.sessionEncKeySize = pCreateParam->keyParam.encKeySize;
        if (genCfgParam.params.ipsec.sessionSaltSize)
        {
            genCfgParam.params.ipsec.sessionEncKeySize =
            pCreateParam->keyParam.encKeySize -
            genCfgParam.params.ipsec.sessionSaltSize;
            pSalt = pCreateParam->keyParam.pEncKey;
            pSalt =  pSalt + genCfgParam.params.ipsec.sessionEncKeySize;
            ipSecKeyParam.sessionSalt = pSalt;
        }
    }

    if (pCreateParam->keyParam.macKeySize)
    {
        /* set the MacKeySize, over-ride if necessary */
        genCfgParam.params.ipsec.sessionMacKeySize = pCreateParam->keyParam.macKeySize;
        if (genCfgParam.params.ipsec.sessionSaltSize)
        {
            if (pCreateParam->saIpSecParam.authMode == NWAL_SA_AALG_GMAC)
            {
                genCfgParam.params.ipsec.sessionMacKeySize =
                                        pCreateParam->keyParam.macKeySize - genCfgParam.params.ipsec.sessionSaltSize;
                pSalt = pCreateParam->keyParam.pAuthKey;
                pSalt =  pSalt + genCfgParam.params.ipsec.sessionMacKeySize;
                ipSecKeyParam.sessionSalt = pSalt;
            }
        }
    }
    genCfgParam.params.ipsec.macSize = pCreateParam->saIpSecParam.macSize;
    genCfgParam.params.ipsec.nextHdr = NWAL_IP_IN_IP_PROTOCOL;
    genCfgParam.params.ipsec.spi = pSaId->spi;

    /* Initialize the Key */
    ipSecKeyParam.ctrlBitfield = (sa_IPSEC_KEY_CTRL_ENC_KEY |
                                  sa_IPSEC_KEY_CTRL_MAC_KEY |
                                  sa_IPSEC_KEY_CTRL_SALT);
    ipSecKeyParam.sessionEncKey = pCreateParam->keyParam.pEncKey;
    ipSecKeyParam.sessionAuthKey = pCreateParam->keyParam.pAuthKey;



    if(pCreateParam->saIpSecParam.dir == NWAL_SA_DIR_INBOUND )
    {
        /* IPSEC RX Channel */
        pGenCtrlInfo->validBitfield =
            (sa_CONTROLINFO_VALID_RX_CTRL | sa_CONTROLINFO_VALID_REPLAY_WIN);
        pGenCtrlInfo->replayWindowSize =
            pCreateParam->saIpSecParam.replayWindow;
        pGenCtrlInfo->rxCtrl = genCfgParam;
    }else if(pCreateParam->saIpSecParam.dir == NWAL_SA_DIR_OUTBOUND )
    {
        /* IPSEC TX Channel */
        pGenCtrlInfo->validBitfield = sa_CONTROLINFO_VALID_TX_CTRL ;
        pGenCtrlInfo->txCtrl = genCfgParam;
    }else {
        retValue = nwal_ERR_INVALID_PARAM;
        goto ERR_nwal_setSecAssoc;
    }

    chanCtrlInfo.ctrlType =  (uint16_t )sa_CHAN_CTRL_GEN_CONFIG;
    retCode =
        Sa_chanControl ((Sa_ChanHandle)pIpSecInfo->hdr.saChanHandle,
                         &chanCtrlInfo);
    if(retCode != sa_ERR_OK)
    {
        retValue = nwal_ERR_SA;
        goto ERR_nwal_setSecAssoc;
    }

    chanCtrlInfo.ctrlType =  (uint16_t )sa_CHAN_CTRL_KEY_CONFIG;
    memset(pKeyCtrlInfo,0,sizeof(Sa_KeyCtrlInfo_t));
    if(pCreateParam->saIpSecParam.dir == NWAL_SA_DIR_INBOUND)
    {
        /* IPSEC RX Channel */
        pKeyCtrlInfo->ctrlBitfield = sa_KEY_CONTROL_RX_KEY_VALID;
        pKeyCtrlInfo->rxKey.ipsec = ipSecKeyParam;
    }else
    {
        /* IPSEC TX Channel */
        pKeyCtrlInfo->ctrlBitfield = sa_KEY_CONTROL_TX_KEY_VALID;
        pKeyCtrlInfo->txKey.ipsec = ipSecKeyParam;
    }

    retCode =
        Sa_chanControl((Sa_ChanHandle)pIpSecInfo->hdr.saChanHandle,
                        &chanCtrlInfo);
    if(retCode != sa_ERR_OK)
    {
        pLocContext->extErr = retCode;
        retValue = nwal_ERR_SA;
        goto ERR_nwal_setSecAssoc;
    }

    /* Enable the channel */
    chanCtrlInfo.ctrlType =  (uint16_t )sa_CHAN_CTRL_GEN_CONFIG;
    memset(pGenCtrlInfo, 0, sizeof(Sa_GenCtrlInfo_t));
    pGenCtrlInfo->validBitfield = sa_CONTROLINFO_VALID_CTRL_BITMAP;
    if(pCreateParam->saIpSecParam.dir == NWAL_SA_DIR_INBOUND)
    {
        pGenCtrlInfo->ctrlBitfield = sa_CONTROLINFO_CTRL_RX_ON;
    }
    else
    {
        pGenCtrlInfo->ctrlBitfield = sa_CONTROLINFO_CTRL_TX_ON;
    }
    retCode =
        Sa_chanControl ((Sa_ChanHandle)pIpSecInfo->hdr.saChanHandle,
                         &chanCtrlInfo);
    if(retCode != sa_ERR_OK)
    {
        pLocContext->extErr = retCode;
        retValue = nwal_ERR_SA;
        goto ERR_nwal_setSecAssoc;
    }

    /* All SALLD configuration complete. Now add rule at PA for outer
     * IP/IPSec
     */
    static unsigned char zeroIP6[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    memset(&pIpSecInfo->paIpInfo,0,sizeof(paIpInfo2_t));
    pIpSecInfo->paIpInfo.ipType =  pCreateParam->ipType;
    if(pIpSecInfo->paIpInfo.ipType == pa_IPV4)
    {
        if(memcmp(pSaId->src.ipv4, zeroIP6, sizeof(nwalIpv4Addr_t)))
        {
            pIpSecInfo->paIpInfo.validBitMap |= pa_IP_INFO_VALID_SRC;
        }
        if(memcmp(pSaId->dst.ipv4, zeroIP6, sizeof(nwalIpv4Addr_t)))
        {
            pIpSecInfo->paIpInfo.validBitMap |= pa_IP_INFO_VALID_DST;
        }
        memcpy(pIpSecInfo->paIpInfo.src.ipv4,
               pSaId->src.ipv4,
               NWAL_IPV4_ADDR_SIZE);
        memcpy(pIpSecInfo->paIpInfo.dst.ipv4,
               pSaId->dst.ipv4,
               NWAL_IPV4_ADDR_SIZE);
    }
    else if(pIpSecInfo->paIpInfo.ipType == pa_IPV6)
    {
        if(memcmp(pSaId->src.ipv6, zeroIP6, sizeof(nwalIpv6Addr_t)))
        {
            pIpSecInfo->paIpInfo.validBitMap |= pa_IP_INFO_VALID_SRC;
        }
        if(memcmp(pSaId->dst.ipv6, zeroIP6, sizeof(nwalIpv6Addr_t)))
        {
            pIpSecInfo->paIpInfo.validBitMap |= pa_IP_INFO_VALID_DST;
        }
        memcpy(pIpSecInfo->paIpInfo.src.ipv6,
               pSaId->src.ipv6,
               NWAL_IPV6_ADDR_SIZE);
        memcpy(pIpSecInfo->paIpInfo.dst.ipv6,
               pSaId->dst.ipv6,
               NWAL_IPV6_ADDR_SIZE);
    }
    else
    {
        retValue = nwal_ERR_INVALID_PARAM;
        goto ERR_nwal_setSecAssoc;
    }
    pIpSecInfo->paIpInfo.validBitMap |= pa_IP_INFO_VALID_SPI;
    pIpSecInfo->paIpInfo.spi = pSaId->spi;
    if(pSaId->proto == nwal_IpSecProtoESP)
    {
        pIpSecInfo->paIpInfo.validBitMap |= pa_IP_INFO_VALID_PROTO;
        pIpSecInfo->paIpInfo.proto = NWAL_ESP_PROTOCOL;
    }
    else if(pSaId->proto == nwal_IpSecProtoAH)
    {
        pIpSecInfo->paIpInfo.validBitMap |= pa_IP_INFO_VALID_PROTO;
        pIpSecInfo->paIpInfo.proto = NWAL_AH_PROTOCOL;
    }
    else if(pSaId->proto == nwal_IpSecProtoESPNATT)
    {
        /* No valid proto, wildcard entry for NATT */
        pIpSecInfo->paIpInfo.proto = 0;
    }
    else
    {
        retValue = nwal_ERR_INVALID_PARAM;
        goto ERR_nwal_setSecAssoc;
    }

    /* Not used flow,tos,tosCare,sctpPort */
    nwal_saveTransInfo(&pIpSecInfo->transInfo,transId);

    pIpSecInfo->transInfo.appId = appId;
    pIpSecInfo->transInfo.nwalHandle = pIpSecInfo;


    if(pIpSecInfo->dir == NWAL_SA_DIR_INBOUND)
    {
        memset(&routeInfo,0,sizeof(nwalLocRouteInfo_t));

        if((pCreateParam->saIpSecParam.validParams & NWAL_SA_INFO_VALID_PARAM_ROUTE_TYPE) ==
        NWAL_SA_INFO_VALID_PARAM_ROUTE_TYPE)
        {
            routeInfo.routeType = pCreateParam->saIpSecParam.routeType;
        }
        routeInfo.flowId = pCreateParam->saIpSecParam.appRxPktFlowId;
        routeInfo.rxPktQ = pCreateParam->saIpSecParam.appRxPktQueue;
        routeInfo.matchAction = pCreateParam->saIpSecParam.matchAction;
        routeInfo.failAction = pCreateParam->saIpSecParam.failAction;
        /* Add PA rule for outer IP RX classification */
        retValue = nwal_configIPSec(nwalProcessCtx.pSharedMemBase,
                                    pIpSecInfo,
                                    &routeInfo);
        if (retValue != nwal_OK)
            goto ERR_nwal_setSecAssoc;

        if(transId == NWAL_TRANSID_SPIN_WAIT)
        {
            /* Block until response is received from NetCP */
            retValue =
                nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                              NULL,
                              NULL,
                              &pIpSecInfo->transInfo.handleHdr);
        }
    }
    else
    {
        /* Retrieve the swInfo from SA LLD */
        retCode =
            Sa_chanGetSwInfo((Sa_ChanHandle)pIpSecInfo->hdr.saChanHandle,
                              sa_PKT_DIR_TO_NETWORK,
                              &pIpSecInfo->hdr.regSwInfo);
        if(retCode != sa_ERR_OK)
        {
            pLocContext->extErr = retCode;
            retValue = nwal_ERR_SA;
            goto ERR_nwal_setSecAssoc;
        }

        /* For outbound connection update retVal for no callback */
        retValue = nwal_TRANS_COMPLETE;

        /* Set the state of the handle to Active */
        pIpSecInfo->hdr.handleHdr.stateCount =
            NWAL_SET_STATE(pIpSecInfo->hdr.handleHdr.stateCount,
                           NWAL_STATE_ACTIVE);
    }

    *pNwalSaHandle = 
        (nwal_Handle*)NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                 pIpSecInfo);
    NWAL_osalWriteBackCache(pIpSecInfo,sizeof(nwalIpSecInfo_t));
    NWAL_osalCsExit(key);
    return(retValue);

ERR_nwal_setSecAssoc:
    NWAL_osalCsExit(key);
    nwal_freeSaInst(nwal_FALSE,pIpSecInfo);
    return(retValue);

}

/********************************************************************
 * FUNCTION PURPOSE: Free SA Resource for IPSec Channel
 ********************************************************************
 * DESCRIPTION: Free SA Resource for IPSec Channel
 ********************************************************************/
nwal_RetValue nwal_freeSaChan(  nwal_Inst           nwalInst,
                                nwalIpSecInfo_t*    pIpSecInfo)
{
    int16_t                 retCode;
    void*                   bases[sa_CHAN_N_BUFS];
    nwalLocContext_t*       pLocContext;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }


#if (sa_CHAN_N_BUFS > 1)
#error NWAL Incorrect Assumption on number of SA chanel buffers
#endif
    bases[0] = pIpSecInfo->hdr.pSaChanBuf;

    /* Close the SA LLD channel */
    retCode = Sa_chanClose (pIpSecInfo->hdr.saChanHandle, bases);
    if(retCode != sa_ERR_OK)
    {
        pLocContext->extErr = retCode;
        return (nwal_ERR_SA);
    }
    nwal_freeSaInst(nwal_FALSE,pIpSecInfo);
    return(nwal_OK);
}
/********************************************************************
 * FUNCTION PURPOSE: Delete Security Association API
 ********************************************************************
 * DESCRIPTION: Delete Security Association API
 ********************************************************************/
nwal_RetValue nwal_delSecAssoc( nwal_Inst           nwalInst,
                                nwal_TransID_t      transId,
                                nwal_Handle         nwalSecAssocHandle)
{
    nwal_RetValue           retVal;
    nwalIpSecInfo_t*        pIpSecInfo =
          (nwalIpSecInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                nwalSecAssocHandle));

    if(pIpSecInfo->dir == NWAL_SA_DIR_INBOUND)
    {
       /* For Inbound SA free the Outer IP Entry. All SA related cleanup will
        * be done after getting  response from PA
        */
        nwal_saveTransInfo(&pIpSecInfo->transInfo,transId);
        nwal_setTransInfo(&pIpSecInfo->transInfo,
                          NWAL_HANDLE_ID_TRANS_DEL_IPSEC,
                          pIpSecInfo);
        retVal = nwalDelPaHandle(nwalProcessCtx.pSharedMemBase,
                                 &pIpSecInfo->transInfo,
                                 0,
                                 &pIpSecInfo->paIpHandle);
        if (retVal != nwal_OK)
        {
            return retVal;
        }
        if(transId == NWAL_TRANSID_SPIN_WAIT)
        {
            /* Block until response is received from NetCP */
            retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                                   NULL,
                                   NULL,
                                   &pIpSecInfo->transInfo.handleHdr);
        }
    }
    else
    {
        /* For Outbound no action needed */
        retVal = nwal_freeSaChan((nwal_Inst)nwalProcessCtx.pSharedMemBase,pIpSecInfo);
        if(retVal == nwal_OK)
        {
            retVal = nwal_TRANS_COMPLETE;
        }
    }

    NWAL_osalWriteBackCache(pIpSecInfo,sizeof(nwalIpSecInfo_t));
    /* For Outbound SA cleanup the SA resources and exit */
    return (retVal);

}


/********************************************************************
 * FUNCTION PURPOSE: Lookup for existing security Association handle
 ********************************************************************
 * DESCRIPTION: Lookup for existing security Association handle
 ********************************************************************/
nwal_Bool_t nwal_getSecAssoc(  nwal_Inst                nwalInst,
                               nwalSaIpSecId_t*         pSaId,
                               nwal_SaDir               dir,
                               nwal_Handle*             pNwalSecAssocHandle,
                               uint32_t*                pSwInfo0,
                               uint32_t*                pSwInfo1)
{

    uint8_t                 count = 0;
    int16_t                 retCode;
    uint32_t                key;
    NWAL_STATE_COUNT_T      state;
    nwalIpSecInfo_t*        pIpSecInfo =
                            (nwalIpSecInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                             nwalProcessCtx.pSharedMemBase->saGlobContext.pIpSecInfo));

    NWAL_osalCsEnter(&key);
    while(count < nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxIpSecChannels)
    {
        NWAL_osalInvalidateCache(pIpSecInfo,sizeof(nwalIpSecInfo_t));
        state = NWAL_GET_STATE(pIpSecInfo->hdr.handleHdr.stateCount);
        if((state == NWAL_STATE_ACTIVE) &&
           (nwalCompareByteArray((uint8_t *)pSaId,
                                (uint8_t *)&pIpSecInfo->saId,
                                sizeof(nwalSaIpSecId_t))))
        {
            if(dir == pIpSecInfo->dir)
            {
                *pNwalSecAssocHandle =
                    (nwal_Handle*)NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                pIpSecInfo);
                retCode = Sa_chanStart(pIpSecInfo->hdr.saChanHandle);
                if(retCode != sa_ERR_OK)
                {
                     NWAL_osalCsExit(key);
                     return (nwal_FALSE);
                }
                NWAL_osalCsExit(key);
                if((pSwInfo0) && (pSwInfo1))
                {
                    if(nwal_getSaSwInfo(pIpSecInfo,pSwInfo0,pSwInfo1) !=
                        nwal_OK)
                    {
                         NWAL_osalCsExit(key);
                         return (nwal_FALSE);
                    }
                }
                NWAL_osalCsExit(key);
                return(nwal_TRUE);
            }
        }
        pIpSecInfo = nwal_getNextInst(pIpSecInfo,sizeof(nwalIpSecInfo_t));
        count++;
    }

    NWAL_osalCsExit(key);
    return (nwal_FALSE);
}

#define NWAL_DM_TMP_BUF_LEN     50
/********************************************************************
 * FUNCTION PURPOSE: API for creating Data Mode Security Association
 ********************************************************************
 * DESCRIPTION: API for creating Data Mode Security Association
 ********************************************************************/
nwal_RetValue nwal_setDMSecAssoc( nwal_Inst               nwalInst,
                                  nwal_AppId              appId,
                                  nwalCreateDmSAParams_t* pCreateParam,
                                  nwal_Handle*            pNwalDmSaHandle)
{
    int16_t                 retCode;
    Sa_ChanCtrlInfo_t       chanCtrlInfo;
    Sa_GenCtrlInfo_t        *pGenCtrlInfo = &chanCtrlInfo.ctrlInfo.gen;
    Sa_KeyCtrlInfo_t        *pKeyCtrlInfo = &chanCtrlInfo.ctrlInfo.key;
    Sa_GenConfigParams_t    genCfgParam;
    nwal_RetValue           retValue;
    uint8_t*                pSalt;
    nwal_Bool_t             encKeyReqd=nwal_TRUE;
    nwal_Bool_t             authKeyReqd=nwal_TRUE;
    nwalLocContext_t*       pLocContext;
    nwalDmSaInfo_t*         pDmSaInfo;
    uint32_t                key;
    nwal_utlSecAssocParam_t utlSecAssocParam;
    Sa_DataModeKeyParams_t  dmKeyParams;

    Sa_PktInfo_t            salldPktInfo;
    Sa_PktDesc_t*           pSalldPktDesc = &salldPktInfo.pktDesc;
    Sa_CmdLbUpdateInfo_t*   pUpdateInfo;
    Sa_CmdLabelInfo_t*      pCmdLbInfo = &salldPktInfo.cmdlb;
    void*                   segments[2] = {NULL, NULL};
    uint16_t                segUsedSizes[2] = {0,0};
    uint16_t                segAllocSizes[2] = {0, 0};
    int16_t                 saRetVal;
    uint32_t*               pCmdLb ;
    uint8_t                 tmpBuf[NWAL_DM_TMP_BUF_LEN];

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    NWAL_osalCsEnter(&key);
    pDmSaInfo = nwal_getSaInst(nwalProcessCtx.pSharedMemBase,nwal_TRUE);
    if(pDmSaInfo == NULL)
    {
        NWAL_osalCsExit(key);
        return nwal_ERR_RES_UNAVAILABLE;
    }
    memset(&genCfgParam,0,sizeof(Sa_GenConfigParams_t));
    memset(&dmKeyParams,0,sizeof(Sa_DataModeKeyParams_t));
    memset(&utlSecAssocParam,0,sizeof(nwal_utlSecAssocParam_t));


    /* Register ID as IPSec Channel Handle */
    utlSecAssocParam.saID = (uint32_t)pDmSaInfo;

    utlSecAssocParam.pNwalContext = nwalProcessCtx.pSharedMemBase;
    utlSecAssocParam.proto = sa_PT_NULL;
    utlSecAssocParam.saChanBufLen = pDmSaInfo->hdr.saChanBufLen;
    utlSecAssocParam.pSaChanBuf = pDmSaInfo->hdr.pSaChanBuf;
    utlSecAssocParam.pGenConfig = &genCfgParam;
    utlSecAssocParam.authMode = pCreateParam->dmSaParam.authMode;
    utlSecAssocParam.cipherMode = pCreateParam->dmSaParam.cipherMode;
    utlSecAssocParam.pSaChanHandle = &pDmSaInfo->hdr.saChanHandle;
    utlSecAssocParam.pAuthKeyReqd = &authKeyReqd;
    utlSecAssocParam.pEncKeyReqd = &encKeyReqd;
    utlSecAssocParam.pEncryptionBlockSize = NULL;
    utlSecAssocParam.pIvSize = &pDmSaInfo->encIvSize;
    utlSecAssocParam.pAuthIvSize = &pDmSaInfo->authIvSize;
    utlSecAssocParam.pSessionSaltSize =
        &genCfgParam.params.data.sessionSaltSize;
    retValue = nwal_utlSecAssoc(&utlSecAssocParam);
    if(retValue != nwal_OK)
    {
        goto ERR_nwal_setDMSecAssoc;
    }

    /* Check if keys are received */
    if((!(pCreateParam->keyParam.encKeySize) && encKeyReqd )   ||
       (!(pCreateParam->keyParam.macKeySize) && authKeyReqd))
    {
        retValue = nwal_ERR_INVALID_KEY;
        goto ERR_nwal_setDMSecAssoc;
    }
    pDmSaInfo->dmChnType = pCreateParam->dmSaParam.dmChnType;
    /* Flow and Queue Information is left unconfigured as it will be
     * updated while sending to SA
     */
    /* Only using APP ID in this case as poll bit map will indicate that
     * RX queue is a data mode queue
     */
    genCfgParam.destInfo.swInfo0 = (uint32_t)appId;
    genCfgParam.destInfo.swInfo1 = 0;

    if(pCreateParam->keyParam.encKeySize)
    {
        genCfgParam.params.data.sessionEncKeySize =
                    (pCreateParam->keyParam.encKeySize -
                     genCfgParam.params.data.sessionSaltSize);
        dmKeyParams.ctrlBitfield |= sa_DATA_MODE_KEY_CTRL_ENC_KEY;
        dmKeyParams.sessionEncKey = pCreateParam->keyParam.pEncKey;
        if(genCfgParam.params.data.sessionSaltSize)
        {
            dmKeyParams.ctrlBitfield |= sa_DATA_MODE_KEY_CTRL_SALT;
            pSalt = pCreateParam->keyParam.pEncKey;
            pSalt =  pSalt + genCfgParam.params.data.sessionEncKeySize;
            dmKeyParams.sessionSalt = pSalt;
        }
    }


    if(pCreateParam->keyParam.macKeySize)
    {
        dmKeyParams.ctrlBitfield |= sa_DATA_MODE_KEY_CTRL_MAC_KEY;
        dmKeyParams.sessionAuthKey = pCreateParam->keyParam.pAuthKey;
        if (pCreateParam->dmSaParam.authMode == NWAL_SA_AALG_GMAC)
        {
            genCfgParam.params.data.sessionMacKeySize =
                        pCreateParam->keyParam.macKeySize -genCfgParam.params.data.sessionSaltSize ;
        }
        else
        {
            genCfgParam.params.data.sessionMacKeySize =
                        pCreateParam->keyParam.macKeySize;
        }
    if(genCfgParam.params.data.sessionSaltSize)
    {
        dmKeyParams.ctrlBitfield |= sa_DATA_MODE_KEY_CTRL_SALT;
            pSalt = pCreateParam->keyParam.pAuthKey;
            pSalt =  pSalt + genCfgParam.params.data.sessionMacKeySize;
        dmKeyParams.sessionSalt = pSalt;
        }
    }

    genCfgParam.params.data.ivSize = pDmSaInfo->encIvSize;
    genCfgParam.params.data.macSize = pCreateParam->dmSaParam.macSize;

    /* Store aadSize locally. ivSize and aadSize to be used for
     * verification during data transfer
     */
    genCfgParam.params.data.aadSize = pCreateParam->dmSaParam.aadSize;
    pDmSaInfo->aadSize = pCreateParam->dmSaParam.aadSize;

    genCfgParam.params.data.enc =
        (pDmSaInfo->dmChnType == NWAL_DM_CHAN_DECRYPT) ? nwal_FALSE: nwal_TRUE;
    genCfgParam.params.data.enc1st = pCreateParam->dmSaParam.enc1st;

    memset(&chanCtrlInfo,0,sizeof(Sa_ChanCtrlInfo_t));
    pGenCtrlInfo->validBitfield = sa_CONTROLINFO_VALID_TX_CTRL;
    pGenCtrlInfo->txCtrl = genCfgParam;
    chanCtrlInfo.ctrlType =  (uint16_t )sa_CHAN_CTRL_GEN_CONFIG;
    retCode = Sa_chanControl ((Sa_ChanHandle)pDmSaInfo->hdr.saChanHandle,
                               &chanCtrlInfo);
    if(retCode != sa_ERR_OK)
    {
        retValue = nwal_ERR_SA;
        goto ERR_nwal_setDMSecAssoc;
    }

    memset(&chanCtrlInfo,0,sizeof(Sa_ChanCtrlInfo_t));
    chanCtrlInfo.ctrlType =  (uint16_t )sa_CHAN_CTRL_KEY_CONFIG;
    pKeyCtrlInfo->ctrlBitfield = sa_KEY_CONTROL_TX_KEY_VALID;
    pKeyCtrlInfo->txKey.data = dmKeyParams;
    retCode =
        Sa_chanControl ((Sa_ChanHandle)pDmSaInfo->hdr.saChanHandle,
                         &chanCtrlInfo);
    if(retCode != sa_ERR_OK)
    {
        pLocContext->extErr = retCode;
        retValue = nwal_ERR_SA;
        goto ERR_nwal_setDMSecAssoc;
    }

    /* Enable the channel */
    memset(&chanCtrlInfo,0,sizeof(Sa_ChanCtrlInfo_t));
    chanCtrlInfo.ctrlType =  (uint16_t )sa_CHAN_CTRL_GEN_CONFIG;
    pGenCtrlInfo->validBitfield = sa_CONTROLINFO_VALID_CTRL_BITMAP;
    pGenCtrlInfo->ctrlBitfield = sa_CONTROLINFO_CTRL_TX_ON;
    retCode =
        Sa_chanControl ((Sa_ChanHandle)pDmSaInfo->hdr.saChanHandle,
                        &chanCtrlInfo);
    if(retCode != sa_ERR_OK)
    {
        pLocContext->extErr = retCode;
        retValue = nwal_ERR_SA;
        goto ERR_nwal_setDMSecAssoc;
    }

    /* Retrieve the swInfo from SA LLD */
    retCode =
        Sa_chanGetSwInfo((Sa_ChanHandle)pDmSaInfo->hdr.saChanHandle,
                          sa_PKT_DIR_TO_NETWORK,
                          &pDmSaInfo->hdr.regSwInfo);
    if(retCode != sa_ERR_OK)
    {
        pLocContext->extErr = retCode;
        retValue = nwal_ERR_SA;
        goto ERR_nwal_setDMSecAssoc;
    }

    /* Create command label for the dummy packet for optimized send
     */
    memset(&salldPktInfo,0,sizeof(salldPktInfo));
    pCmdLb = (uint32_t*)pDmSaInfo->psCmdLabel;
    pUpdateInfo = &pDmSaInfo->cmdLbUpdate;

    /* Initialize the packet descriptor */
    pSalldPktDesc->nSegments = 1;    /* One Segment */
    pSalldPktDesc->segments = segments;
    pSalldPktDesc->segUsedSizes = segUsedSizes;
    pSalldPktDesc->segAllocSizes = segAllocSizes;
    pSalldPktDesc->segments[1] = NULL;
    pSalldPktDesc->segUsedSizes[1] = 0;
    pSalldPktDesc->segAllocSizes[1] = 0;
    pSalldPktDesc->size = NWAL_DM_TMP_BUF_LEN;

    pSalldPktDesc->segments[0] = &tmpBuf;
    pSalldPktDesc->segUsedSizes[0] = pSalldPktDesc->size;
    pSalldPktDesc->segAllocSizes[0] = NWAL_DM_TMP_BUF_LEN ;

    /* Command Label Info */
    pCmdLbInfo->cmdLbBuf = (uint8_t*)pDmSaInfo->psCmdLabel;
    pCmdLbInfo->cmdLbUpdateInfo = &pDmSaInfo->cmdLbUpdate;
    /* fill the payload Info */
    salldPktInfo.validBitMap = sa_PKT_INFO_VALID_PAYLOAD_INFO |
                               sa_PKT_INFO_VALID_CMDLB_INFO;
    salldPktInfo.payloadInfo.encOffset = 0;
    salldPktInfo.payloadInfo.authOffset = 0;
    salldPktInfo.payloadInfo.encSize  = NWAL_DM_TMP_BUF_LEN;
    salldPktInfo.payloadInfo.authSize = NWAL_DM_TMP_BUF_LEN;

    salldPktInfo.payloadInfo.encIV    = tmpBuf; /* Dummy */
    salldPktInfo.payloadInfo.authIV   = tmpBuf; /* Dummy */
    salldPktInfo.payloadInfo.aad      = tmpBuf; /* Dummy */

    /* IPSEC operation */
    pSalldPktDesc->payloadOffset = 0;
    pSalldPktDesc->payloadLen = pSalldPktDesc->size;

    saRetVal =
        Sa_chanSendData(pDmSaInfo->hdr.saChanHandle,
                        &salldPktInfo, nwal_FALSE);
    if(saRetVal != sa_ERR_OK)
    {
        retValue = nwal_ERR_SA;
        goto ERR_nwal_setDMSecAssoc;
    }

    /* Update the command label information */
    pDmSaInfo->psCmdLabelLen = pCmdLbInfo->cmdLbSize;
    sa_mDmResetCmdLb(pUpdateInfo,pCmdLb);

    /* Set the state of the handle to Active */
    pDmSaInfo->hdr.handleHdr.stateCount =
            NWAL_SET_STATE(pDmSaInfo->hdr.handleHdr.stateCount,
                           NWAL_STATE_ACTIVE);

    *pNwalDmSaHandle = pDmSaInfo;
    NWAL_osalWriteBackCache(pDmSaInfo,sizeof(nwalDmSaInfo_t));
    NWAL_osalCsExit(key);

    return(nwal_OK);

ERR_nwal_setDMSecAssoc:
    NWAL_osalCsExit(key);
    nwal_freeSaInst(nwal_TRUE,pDmSaInfo);
    return(retValue);
}

/********************************************************************
 * FUNCTION PURPOSE: Get the command label and meta data information
 *                  for the Data mode channel
 ********************************************************************
 * DESCRIPTION: Get the command label and meta data information
 *              for the Data mode channel
 ********************************************************************/
static inline nwal_RetValue
              nwal_intGetDmCmdLb(nwal_Inst              nwalInst,
                                 nwal_Handle            nwalDmSaHandle,
                                 nwalTxDmPSCmdInfo_t*   pTxDmPsCmdInfo)
{
    nwalDmSaInfo_t*     pDmSaInfo = (nwalDmSaInfo_t*)nwalDmSaHandle;
    nwalLocContext_t*   pLocContext;

    if(pDmSaInfo->hdr.handleHdr.handleId != NWAL_HANDLE_DM_INST)
    {
        return(nwal_ERR_INVALID_HANDLE);
    }

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    pTxDmPsCmdInfo->pUpdateInfo = &(pDmSaInfo->cmdLbUpdate);
    pTxDmPsCmdInfo->pCmdLb = (uint32_t*)pDmSaInfo->psCmdLabel;
    pTxDmPsCmdInfo->cmdLbLen = pDmSaInfo->psCmdLabelLen;
    pTxDmPsCmdInfo->pSwInfo = pDmSaInfo->hdr.regSwInfo.swInfo;
    pTxDmPsCmdInfo->rxSbSaQ = pLocContext->rxSbSaQ;
    pTxDmPsCmdInfo->rxPktFlowId = pLocContext->rxPktFlowId;
    pTxDmPsCmdInfo->txQueue = QMSS_PASS_QUEUE_BASE +
                              NSS_SA_QUEUE_SASS_INDEX;
    return nwal_OK;
}
/********************************************************************
 * FUNCTION PURPOSE: Get the command label and meta data information
 *                  for the Data mode channel
 ********************************************************************
 * DESCRIPTION: Get the command label and meta data information
 *              for the Data mode channel
 ********************************************************************/
nwal_RetValue nwal_initDMPSCmdInfo(nwal_Inst              nwalInst,
                                   nwal_Handle            nwalDmSaHandle,
                                   nwalTxDmPSCmdInfo_t*   pTxDmPsCmdInfo)
{
    return(nwal_intGetDmCmdLb((nwal_Inst)nwalProcessCtx.pSharedMemBase,
                              nwalDmSaHandle,
                              pTxDmPsCmdInfo));
}

/********************************************************************
 * FUNCTION PURPOSE: Delete Data Mode Security Association API
 ********************************************************************
 * DESCRIPTION: Delete Data Mode Security Association API
 ********************************************************************/
nwal_RetValue nwal_delDMSecAssoc( nwal_Inst           nwalInst,
                                  nwal_Handle         nwalDmSaHandle)
{
    nwalDmSaInfo_t*         pDmSaInfo = (nwalDmSaInfo_t*)nwalDmSaHandle;
    int16_t                 retCode;
    void*                   bases[sa_CHAN_N_BUFS];
    nwalLocContext_t*       pLocContext;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }


#if (sa_CHAN_N_BUFS > 1)
#error NWAL Incorrect Assumption on number of SA chanel buffers
#endif
    bases[0] = pDmSaInfo->hdr.pSaChanBuf;

    /* Close the SA LLD channel */
    retCode = Sa_chanClose (pDmSaInfo->hdr.saChanHandle, bases);
    if(retCode != sa_ERR_OK)
    {
        pLocContext->extErr = retCode;
        return (nwal_ERR_SA);
    }
    nwal_freeSaInst(nwal_TRUE,pDmSaInfo);
    NWAL_osalWriteBackCache(pDmSaInfo,sizeof(nwalDmSaInfo_t));

    return(nwal_OK);
}

/********************************************************************
 * FUNCTION PURPOSE: Create Security Policy Handle
 ********************************************************************
 * DESCRIPTION: Create Security Policy Handle
 ********************************************************************/
nwal_RetValue nwal_setSecPolicy(nwal_Inst               nwalInst,
                                nwal_TransID_t          transId,
                                nwal_AppId              appId,
                                nwalSecPolParams_t*     pPolParam,
                                nwal_Handle*            pNwalSecPolHandle)
{
    nwal_RetValue       nwalRetVal = nwal_OK;
    nwalIpInfo_t*       pNwalIpInfo;
    nwalHandleHdr_t*    pHandleHdr;
    NWAL_STATE_COUNT_T  state;
    uint32_t            key;
    nwalLocRouteInfo_t     routeInfo;
    nwalIpSecInfo_t*    pIpSecInfo; 

    pIpSecInfo = (nwalIpSecInfo_t* )(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                                pPolParam->handle));

    if(pPolParam->dir == NWAL_SA_DIR_OUTBOUND)
    {

        /* For TX no special action is needed return SA Handle for the TX
         * so that application can send at later stage while establishing
         * a connection
         */
        *pNwalSecPolHandle = pPolParam->handle;
        return(nwal_TRANS_COMPLETE);
    }

    pHandleHdr = (nwalHandleHdr_t *)(pIpSecInfo);
    /* Get the state of the upper layer handle */
    state = NWAL_GET_STATE(pHandleHdr->stateCount);
    if(state != NWAL_STATE_ACTIVE)
    {
        return nwal_ERR_INVALID_STATE;
    }

    NWAL_osalCsEnter(&key);
    pNwalIpInfo = nwal_getInst((void*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                             nwalProcessCtx.pSharedMemBase->pIPInfo)),
                                nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxIpAddress,
                                sizeof(nwalIpInfo_t));
    if(pNwalIpInfo == NULL)
    {
        nwalRetVal = nwal_ERR_RES_UNAVAILABLE;
        goto RET_nwal_setSecPolicy;
    }
    memset(pNwalIpInfo,0,sizeof(nwalIpInfo_t));

    pNwalIpInfo->paIpInfo.ipType = pPolParam->ipType;
    /* Store the PA IP configuration */
    nwal_convIpParam(pPolParam->ipType,
                     &pPolParam->dst,
                     &pPolParam->src,
                     &pPolParam->ipOpt,
                     &pNwalIpInfo->paIpInfo);

    nwal_InvPreHnd((nwal_Handle*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                        pPolParam->handle)));
    pNwalIpInfo->prevHandle = pPolParam->handle;

    nwal_saveTransInfo(&pNwalIpInfo->transInfo,transId);
    pNwalIpInfo->transInfo.appId = appId;
    pNwalIpInfo->transInfo.nwalHandle = pNwalIpInfo;
    pNwalIpInfo->dir = pPolParam->dir;

    *pNwalSecPolHandle = (nwal_Handle*)NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                     pNwalIpInfo);
    if(pIpSecInfo->saMode == nwal_SA_MODE_TRANSPORT)
    {
        /* Verify the policy configured already to NetCP
         * through outer IP
         */
        if((pIpSecInfo->paIpInfo.ipType != pNwalIpInfo->paIpInfo.ipType) ||
            (memcmp(&pIpSecInfo->paIpInfo.dst,
                    &pNwalIpInfo->paIpInfo.dst,
                    sizeof(paIpAddr_t))) ||
            (memcmp(&pIpSecInfo->paIpInfo.src,
                    &pNwalIpInfo->paIpInfo.src,
                    sizeof(paIpAddr_t))))
        {
             NWAL_osalCsExit(key);
             nwal_freeInst((nwal_Handle *)pNwalIpInfo,
                            sizeof(nwalIpInfo_t));
             return(nwal_ERR_POLICY_CHECK_FAIL);
        }
        /* PA IP Handle is same as the one created during creation of
         * SA. Save it so that next layer can pick up
         */
        pNwalIpInfo->paIpHandle = pIpSecInfo->paIpHandle;
        pNwalIpInfo->appId = appId;
        pNwalIpInfo->handleHdr.handleId = NWAL_HANDLE_IP_INST;
        nwalRetVal = nwal_TRANS_COMPLETE;
        NWAL_osalWriteBackCache(pNwalIpInfo,sizeof(nwalIpInfo_t));
        goto RET_nwal_setSecPolicy;
    }
    else
    {
        memset(&routeInfo, 0, sizeof(nwalLocRouteInfo_t));
        if((pPolParam->validParams & NWAL_SET_SEC_POLICY_VALID_PARAM_ROUTE_TYPE) ==
            NWAL_SET_SEC_POLICY_VALID_PARAM_ROUTE_TYPE)
        {
            routeInfo.routeType = pPolParam->routeType;
        }
        routeInfo.flowId= pPolParam->appRxPktFlowId;
        routeInfo.rxPktQ= pPolParam->appRxPktQueue;
        routeInfo.matchAction = pPolParam->matchAction;
        routeInfo.failAction = pPolParam->failAction;

        nwalRetVal = nwal_configIP( nwalProcessCtx.pSharedMemBase,
                                    pNwalIpInfo,
                                    &routeInfo);
        if(nwalRetVal != nwal_OK)
        {
        
             NWAL_osalCsExit(key);
             nwal_freeInst((nwal_Handle *)pNwalIpInfo,
                            sizeof(nwalIpInfo_t));
             return(nwalRetVal);
        }
        if(transId == NWAL_TRANSID_SPIN_WAIT)
        {
            /* Block until response is received from NetCP */
            nwalRetVal =
                nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                              NULL,
                              NULL,
                              &pNwalIpInfo->transInfo.handleHdr);
        }
    }

    /* Cache Write back */
    NWAL_osalWriteBackCache(pNwalIpInfo,sizeof(nwalIpInfo_t));
RET_nwal_setSecPolicy:
    NWAL_osalCsExit(key);
    return(nwalRetVal);
}
/********************************************************************
 * FUNCTION PURPOSE: Delete a policy Handle
 ********************************************************************
 * DESCRIPTION: Delete a policy Handle
 ********************************************************************/
nwal_RetValue nwal_delSecPolicy(nwal_Inst           nwalInst,
                                nwal_TransID_t      transId,
                                nwal_Handle         nwalProfHandle)
{
    nwalIpInfo_t*       pIpInfo;
    nwal_RetValue       retVal;
    nwalIpSecInfo_t*    pIpSecInfo;
    nwalHandleHdr_t *   pHndlHdr =
          (nwalHandleHdr_t *)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                         nwalProfHandle));

    if(pHndlHdr->handleId == NWAL_HANDLE_IPSEC_INST)
    {
        /* Case of SA handle being received for SP Outbound direction.
         * Return Success with callbacks. Actual SA delection will be done
         * in nwal_deletSA
         */
        return(nwal_TRANS_COMPLETE);
    }
    else
    {
        if(pHndlHdr->handleId != NWAL_HANDLE_IP_INST)
        {
            return nwal_ERR_INVALID_PARAM;
        }
    }

    pIpInfo =
        (nwalIpInfo_t *)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                         nwalProfHandle));
    pIpSecInfo = 
        (nwalIpSecInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                      pIpInfo->prevHandle));

    if(pIpSecInfo->saMode == nwal_SA_MODE_TRANSPORT)
    {
        /* There is no additional PA IP entry */
        nwal_freeInst((nwal_Handle *)pIpInfo,
                       sizeof(nwalIpInfo_t));
        return(nwal_TRANS_COMPLETE);
    }
    nwal_saveTransInfo(&pIpInfo->transInfo,transId);

    nwal_setTransInfo(&pIpInfo->transInfo,
                      NWAL_HANDLE_ID_TRANS_DEL_IP,
                      pIpInfo);
    retVal = nwalDelPaHandle(nwalProcessCtx.pSharedMemBase,
                             &pIpInfo->transInfo,
                             0,
                             &pIpInfo->paIpHandle);
    if(retVal != nwal_OK)
        return(retVal);

    if(transId == NWAL_TRANSID_SPIN_WAIT)
    {
        /* Block until response is received from NetCP */
        retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                               NULL,
                               NULL,
                               &pIpInfo->transInfo.handleHdr);
    }
    NWAL_osalWriteBackCache(pIpInfo,sizeof(nwalIpInfo_t));
    return(retVal);
}

/********************************************************************
 * FUNCTION PURPOSE: Get handle for security policy if already configured
 ********************************************************************
 * DESCRIPTION: Get handle for security policy
 ********************************************************************/
nwal_Bool_t nwal_getSecPolicy( nwal_Inst           nwalInst,
                               nwalSecPolParams_t* pPolParam,
                               nwal_Handle*        pNwalSecPolHandle)
{
    nwalIpInfo_t*       pNwalIpInfo =
                        (nwalIpInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                            nwalProcessCtx.pSharedMemBase->pIPInfo));
    paIpInfo2_t          paIpInfo;
    uint8_t             count=0;
    nwal_Bool_t         found = nwal_FALSE;
    uint32_t            key;
    NWAL_STATE_COUNT_T  state;


    if(pPolParam->dir == NWAL_SA_DIR_OUTBOUND)
    {
        /* For Outbound Policy handle is same as the SA Handle */
        *pNwalSecPolHandle = pPolParam->handle;
        return nwal_TRUE;
    }
    memset(&paIpInfo,0,sizeof(paIpInfo2_t));

    nwal_convIpParam(pPolParam->ipType,
                     &pPolParam->dst,
                     &pPolParam->src,
                     &pPolParam->ipOpt,
                     &paIpInfo);

   NWAL_osalCsEnter(&key);
    while(count < nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxIpAddress)
    {
        NWAL_osalInvalidateCache(pNwalIpInfo,sizeof(nwalIpInfo_t));
        state = NWAL_GET_STATE(pNwalIpInfo->handleHdr.stateCount);
        if((state == NWAL_STATE_ACTIVE) &&
           (nwalCompareByteArray((uint8_t *)&paIpInfo,
                                (uint8_t *)&pNwalIpInfo->paIpInfo,
                                sizeof(paIpInfo2_t))))
        {
            if((pPolParam->handle == pNwalIpInfo->prevHandle) &&
               (pPolParam->dir == pNwalIpInfo->dir))
            {

                found = nwal_TRUE;
                *pNwalSecPolHandle = 
                    (nwal_Handle*)NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                pNwalIpInfo);
                break;
            }
        }
        pNwalIpInfo = nwal_getNextInst(pNwalIpInfo,sizeof(nwalIpInfo_t));
        count++;
    }
   NWAL_osalCsExit(key);
   return(found);
}


#ifdef NWAL_LIB_ENABLE_PROFILE
unsigned int nwal_sendDM_prof1_sum = 0;
unsigned int nwal_sendDM_prof2_sum = 0;
unsigned int nwal_sendDM_prof3_sum = 0;
unsigned int nwal_sendDM_prof4_sum = 0;
unsigned int nwal_sendDM_prof5_sum = 0;
unsigned int nwal_sendDM_sum = 0;
unsigned int nwal_sendDM_count_sum =0;
extern unsigned int Osal_cache_op_measure(unsigned long long * p_n);
#endif

/********************************************************************
 * FUNCTION PURPOSE: Transmit payload for side band Data Mode Channel
 ********************************************************************
 * DESCRIPTION: Transmit payload for side band Data Mode Channel
 ********************************************************************/
nwal_RetValue nwal_sendDM(nwal_Inst                 nwalInst,
                          nwal_Handle               nwalDmSaHandle,
                          nwalDmTxPayloadInfo_t*    pDmPloadInfo)
{
    Cppi_HostDesc*          pPloadDesc;
    nwal_RetValue           retVal;
    nwalTxDmPSCmdInfo_t     txDmPSCmdInfo;
#ifdef NWAL_LIB_ENABLE_PROFILE
    unsigned int            countBegin,countStart,countEnd,cacheBegin,cacheEnd;
    unsigned long long      numCacheOps=0;
#endif

#ifdef NWAL_LIB_ENABLE_PROFILE
    countBegin = nwal_read_clock();
    countStart = countBegin;
    cacheBegin = Osal_cache_op_measure(&numCacheOps);
#endif

    retVal = nwal_intGetDmCmdLb((nwal_Inst)nwalProcessCtx.pSharedMemBase,
                                nwalDmSaHandle,
                                &txDmPSCmdInfo);
    if(retVal != nwal_OK)
    {
        return retVal;
    }
    pPloadDesc = Pktlib_getDescFromPacket(pDmPloadInfo->pPkt);

#ifdef NWAL_LIB_ENABLE_PROFILE
    countEnd = nwal_read_clock();
    nwal_sendDM_prof1_sum += (countEnd - countStart);
    countStart = countEnd;
#endif
    nwal_mCmdDMUpdate(pDmPloadInfo->pPkt,
                      &txDmPSCmdInfo,
                      pDmPloadInfo->appCtxId,
                      pDmPloadInfo->encOffset,
                      pDmPloadInfo->encSize,
                      pDmPloadInfo->pEncIV,
                      pDmPloadInfo->authOffset,
                      pDmPloadInfo->authSize,
                      pDmPloadInfo->pAuthIV,
                      pDmPloadInfo->aadSize,
                      pDmPloadInfo->pAad);

#ifdef NWAL_LIB_ENABLE_PROFILE
    countEnd = nwal_read_clock();
    nwal_sendDM_prof2_sum += (countEnd - countStart);
    countStart = countEnd;
#endif
#ifndef __ARMv7
     /* Update Cache */
    Pktlib_writebackPkt(pDmPloadInfo->pPkt);
#endif

    /* Send the packet to SA. */
    Qmss_queuePushDescSize (txDmPSCmdInfo.txQueue,
                            (Ptr)pPloadDesc,
                            NWAL_DESC_SIZE);

#ifdef NWAL_LIB_ENABLE_PROFILE
    countEnd = nwal_read_clock();
    cacheEnd = Osal_cache_op_measure(&numCacheOps);
    nwal_sendDM_prof5_sum += (countEnd - countStart)- (cacheEnd - cacheBegin);
    nwal_sendDM_sum +=
                (countEnd - countBegin) - (cacheEnd-cacheBegin);
    nwal_sendDM_count_sum++;
#endif
    return nwal_OK;
}


/********************************************************************
 *  FUNCTION PURPOSE: Poll Data Mode SA channel for Encrypted/Decrypted
 *                    packets
 ********************************************************************
 ********************************************************************/
uint16_t nwal_pollDm(nwal_Inst              nwalInst,
                     nwal_pollDmQCtl        dmQCtl,
                     uint32_t               appCookie,
                     uint16_t               maxPkts,
                     Qmss_QueueHnd          appRxQueue,
                     nwal_rxDmCallback*     pRxDmCallBack)
{
    uint16_t                count= 0;
    uint8_t                 count2=0;
    Qmss_QueueHnd           rxQ;
    nwal_Bool_t             freePkt[NWAL_MAX_RX_PKT_THRESHOLD];
    nwalDmRxPayloadInfo_t   rxPktInfo[NWAL_MAX_RX_PKT_THRESHOLD];
    Cppi_HostDesc*          pHd;
    nwalLocContext_t*       pLocContext;

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(0);
    }

    switch(dmQCtl)
    {
        case nwal_POLL_DM_DEF_SB_SA_Q:
        {
            rxQ = pLocContext->rxSbSaQ;
            break;
        }

        case nwal_POLL_DM_APP_MANAGED_Q:
        {
            rxQ = appRxQueue;
            break;
        }
        default:
        {
            /* Error */
            return(0);
        }
    }

    memset(rxPktInfo,0,sizeof(rxPktInfo));
    memset(freePkt,0,sizeof(freePkt));

    while ((Qmss_getQueueEntryCount (rxQ)) && (count < maxPkts ))
    {
        pHd = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop (rxQ));

        NWAL_osalInvalidateCache(pHd,NWAL_CACHE_LINE_SIZE);
        NWAL_osalInvalidateCache((void *)pHd->buffPtr,pHd->buffLen);

        rxPktInfo[count].pPkt = Pktlib_getPacketFromDesc(pHd);
        if(nwal_mGetAppidFmPkt(rxPktInfo[count].pPkt,&rxPktInfo[count].appId) !=
           nwal_TRUE)
        {
            /* Error case. Should never reach here. Block for futher
             * debugging */
            nwal_debug_bk();
            pLocContext->stats.errNonEpibPkts++;
            break;
        }
        /* Extract PS Info for any authentication tag if exists */
        nwal_mmGetDmAuthTag((Ti_Pkt*)pHd,
                            (uint8_t **)&rxPktInfo[count].pAuthTag,
                            &rxPktInfo[count].authTagLen);


        /* Application context ID for the data mode response */
        nwal_mmGetDmAppCtxId((Ti_Pkt*)pHd,
                             (uint32_t *)&rxPktInfo[count].appCtxId);
        count++;
    }
    if(count)
    {
        if(!pRxDmCallBack)
        {
            pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
            if(pLocContext == NULL)
            {
                nwal_debug_bk();
                return(count);
            }
            pRxDmCallBack = pLocContext->cfg.pRxDmCallBack;
        }
        if(pRxDmCallBack)
        {
            pRxDmCallBack(appCookie,
                          count,
                          rxPktInfo,
                          freePkt);
        }
        for(count2 = 0;count2 < count;count2++)
        {
            if(freePkt[count2] == nwal_TRUE)
            {
                /* Free the packet  */
                Pktlib_freePacket(rxPktInfo[count2].pPkt);
            }
        }
    }

    return(count);
}

#ifndef __ARMv7
extern cregister volatile unsigned int TSCL;
#endif
#define NWAL_SA_STATS_MAX_RETRIES       200
#define NWAL_SA_STATS_QUERRY_INTERVAL   50
/*****************************************************************************
 * Function: Utility function a cycle clock
 ****************************************************************************/
#ifndef __ARMv7 /* Remove this ifdef if you want to use PMU (__asm__ below) */
static uint32_t nwalUtilReadTime32(void)
{
    uint32_t timeVal;

#if defined (_TMS320C6X)
    timeVal = TSCL;
#else
    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(timeVal));
#endif
    return timeVal;
}
#endif

/****************************************************************************
 * FUNCTION PURPOSE: Delay function
 ****************************************************************************
 * DESCRIPTION: The function will create NWAL instance which is prerequiite
 *              for any call to NWAL
 ****************************************************************************/
static inline void nwalUtilCycleDelay (int iCount)
{
  if (iCount <= 0)
    return;

  /* If there is need for ARM-RTOS, adjust #ifdef to use the PMU
   * via the __asm__ in nwalUtilReadTime32 and use same code as
   * c66 */
#ifndef __ARMv7
  {
    uint32_t start = nwalUtilReadTime32();
    uint32_t count = (uint32_t)iCount;

    while ((nwalUtilReadTime32() - start) < count);
  }
#else
  {
    uint32_t sat;
    /* This code is for user-mode where PMU is inaccessible */
    for (sat=0; sat<(uint32_t)iCount; sat++);
  }
#endif
}

/****************************************************************************
 * FUNCTION PURPOSE: Delay function
 ****************************************************************************
 * DESCRIPTION: The function to get SA Data Mode Stats
 ****************************************************************************/

nwal_RetValue nwal_getDataModeStats(nwal_Inst           nwalInst,
                                    nwal_Handle         nwalHandle,
                                    Sa_DataModeStats_t* pSaDataModeStats)
{
    Sa_Stats_t          saChanStats;
    int16_t             ret_code;
    int                 i;

    /* Get the IP Sec Channel Handle */
    nwalDmSaInfo_t*    pDmSaInfo;
    memset(&saChanStats,0,sizeof(saChanStats));

    if((((nwalHandleHdr_t *)(nwalHandle))->handleId &
          NWAL_HANDLE_ID_MASK) !=  NWAL_HANDLE_ID_INST_DM)
    {
        return (nwal_ERR_INVALID_HANDLE);
    }

    pDmSaInfo = (nwalDmSaInfo_t*)nwalHandle;
    ret_code = Sa_chanGetStats (pDmSaInfo->hdr.saChanHandle,
                                sa_STATS_QUERY_FLAG_TRIG,
                                &saChanStats);

    /* Wait for the stats reply */
    for (i = 0; i < NWAL_SA_STATS_MAX_RETRIES; i++)
    {
        ret_code = Sa_chanGetStats (pDmSaInfo->hdr.saChanHandle,
                                    0,
                                    &saChanStats);
        if(ret_code == sa_ERR_OK)
        {
            break;
        }
        nwalUtilCycleDelay (NWAL_SA_STATS_QUERRY_INTERVAL);

    }

    if (i == NWAL_SA_STATS_MAX_RETRIES)
    {
        ret_code = Sa_chanGetStats (pDmSaInfo->hdr.saChanHandle,
                                    sa_STATS_QUERY_FLAG_NOW,
                                    &saChanStats);
    }
    if(ret_code != sa_ERR_OK)
    {
        return(nwal_ERR_SA);
    }
    memcpy(pSaDataModeStats,&saChanStats.data,sizeof(Sa_DataModeStats_t));

    return nwal_OK;
}
/****************************************************************************
 * FUNCTION PURPOSE: Delay function
 ****************************************************************************
 * DESCRIPTION: The function to get SA IPSec Stats
 ****************************************************************************/

nwal_RetValue nwal_getSecAssocStats (nwal_Inst             nwalInst,
                                     nwal_Handle           nwalHandle,
                                     Sa_IpsecStats_t*      pSaIpsecStats)
{
    Sa_Stats_t         saChanStats;
    int16_t ret_code;
    int i;
    nwal_Handle nwalHandleAddr;

    /* Get the IP Sec Channel Handle */
    nwalIpSecInfo_t*    pIpSecInfoOut;
    memset(&saChanStats,0,sizeof(saChanStats));

    nwalHandleAddr = 
                    (nwal_Handle)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                nwalHandle));

    if((((nwalHandleHdr_t *)(nwalHandleAddr))->handleId &
          NWAL_HANDLE_ID_MASK) ==  NWAL_HANDLE_ID_INST_PORT)
    {
        nwalPortInfo_t*     pPortInfo;
        pPortInfo = (nwalPortInfo_t* )nwalHandleAddr;
        nwalHandleAddr = pPortInfo->pL2L3HdrInfo->outHandle;
    }

    if((((nwalHandleHdr_t *)(nwalHandleAddr))->handleId &
          NWAL_HANDLE_ID_MASK) !=  NWAL_HANDLE_ID_INST_IPSEC)
    {
        return (nwal_ERR_INVALID_HANDLE);
    }

    pIpSecInfoOut = (nwalIpSecInfo_t*)nwalHandleAddr;


    ret_code = Sa_chanGetStats (pIpSecInfoOut->hdr.saChanHandle,
                                sa_STATS_QUERY_FLAG_TRIG,
                                &saChanStats);

    /* Wait for the stats reply */
    for (i = 0; i < NWAL_SA_STATS_MAX_RETRIES; i++)
    {
        ret_code = Sa_chanGetStats (pIpSecInfoOut->hdr.saChanHandle,
                                    0,
                                    &saChanStats);
        if(ret_code == sa_ERR_OK)
        {
            break;
        }
        nwalUtilCycleDelay (NWAL_SA_STATS_QUERRY_INTERVAL);

    }

    if (i == NWAL_SA_STATS_MAX_RETRIES)
    {
        ret_code = Sa_chanGetStats (pIpSecInfoOut->hdr.saChanHandle,
                                    sa_STATS_QUERY_FLAG_NOW,
                                    &saChanStats);
    }
    if(ret_code != sa_ERR_OK)
    {
        return(nwal_ERR_SA);
    }
    memcpy(pSaIpsecStats,&saChanStats.ipsec,sizeof(Sa_IpsecStats_t));

    return nwal_OK;
}

/****************************************************************************
 * FUNCTION PURPOSE: Delay function
 ****************************************************************************
 * DESCRIPTION: The function to get SA Global Stats
 ****************************************************************************/

nwal_RetValue nwal_getSASysStats (nwal_Inst             nwalInst,
                                  Sa_SysStats_t*        pSaStats)
{
    Sa_getSysStats (nwalProcessCtx.pSharedMemBase->saGlobContext.salld_handle, pSaStats);
    return nwal_OK;
}

#else


/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 *
 ********************************************************************
 ********************************************************************/
nwal_RetValue nwal_setSecAssoc( nwal_Inst               nwalInst,
                                nwal_TransID_t          transId,
                                nwal_AppId              appId,
                                nwalSaIpSecId_t*        pSaId,
                                nwalCreateSAParams_t*   pCreateParam,
                                nwal_Handle*            pNwalSaHandle)
{
    return (nwal_ERR_SA_NOT_ENABLED);
}

/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 *
 ********************************************************************
 ********************************************************************/
nwal_RetValue nwal_delSecAssoc( nwal_Inst           nwalInst,
                                nwal_TransID_t      transId,
                                nwal_Handle         nwalSecAssocHandle)
{
    return (nwal_ERR_SA_NOT_ENABLED);
}

/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 *
 ********************************************************************
 ********************************************************************/
nwal_Bool_t nwal_getSecAssoc(  nwal_Inst                nwalInst,
                               nwalSaIpSecId_t*         pSaId,
                               nwal_SaDir               dir,
                               nwal_Handle*             pNwalSecAssocHandle,
                               uint32_t*                pSwInfo0,
                               uint32_t*                pSwInfo1)
{
    return (nwal_FALSE);
}

/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 *
 ********************************************************************
 ********************************************************************/
nwal_RetValue nwal_setSecPolicy(nwal_Inst               nwalInst,
                                nwal_TransID_t          transId,
                                nwal_AppId              appId,
                                nwalSecPolParams_t*     pPolParam,
                                nwal_Handle*            pNwalSecPolHandle)
{
    return (nwal_ERR_SA_NOT_ENABLED);
}

/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 *
 ********************************************************************
 ********************************************************************/
nwal_RetValue nwal_delSecPolicy(nwal_Inst           nwalInst,
                                nwal_TransID_t      transId,
                                nwal_Handle         nwalProfHandle)
{
    return (nwal_ERR_SA_NOT_ENABLED);
}

/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 *
 ********************************************************************
 ********************************************************************/
nwal_Bool_t nwal_getSecPolicy( nwal_Inst           nwalInst,
                               nwalSecPolParams_t* pPolParam,
                               nwal_Handle*        pNwalSecPolHandle)
{
    return (nwal_FALSE);
}

/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 ********************************************************************
 ********************************************************************/
nwal_RetValue nwal_setDMSecAssoc( nwal_Inst               nwalInst,
                                  nwal_AppId              appId,
                                  nwalCreateDmSAParams_t* pCreateParam,
                                  nwal_Handle*            pNwalDmSaHandle)
{
    return (nwal_ERR_SA_NOT_ENABLED);
}


/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 ********************************************************************
 ********************************************************************/
nwal_RetValue nwal_sendDM(nwal_Inst                 nwalInst,
                          nwal_Handle               nwalDmSaHandle,
                          nwalDmTxPayloadInfo_t*    pDmPloadInfo)
{
    return (nwal_ERR_SA_NOT_ENABLED);
}


/********************************************************************
 *  FUNCTION PURPOSE: Stub Function if SA is not enabled
 ********************************************************************
 ********************************************************************/
uint16_t nwal_pollDm(nwal_Inst              nwalInst,
                     nwal_pollDmQCtl        dmQCtl,
                     uint32_t               appCookie,
                     uint16_t               maxPkts,
                     Qmss_QueueHnd          appRxQueue,
                     nwal_rxDmCallback*     pRxDmCallBack)
{
    return(0);
}

#endif

