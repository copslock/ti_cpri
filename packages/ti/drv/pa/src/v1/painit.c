/******************************************************************************
 * FILE PURPOSE:  Initialization Routines for PA LLD
 ******************************************************************************
 * FILE NAME:   painit.c  
 *
 * DESCRIPTION: PA initialization routes which are executed only during driver 
 *              initialization.  
 *
 * FUNCTION               DESCRIPTION
 * ========               ===========
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2009-2013
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

#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pa_osal.h>
#include <ti/drv/pa/pasahost.h>
#include "paloc.h"
#include "pafrm.h"

/** 
 * @brief Assert-type macro that checks the size of structures.
 *
 * This assertion controls the compilation process. If the postulate about
 * the structure size is not TRUE the compilation would result in error.
 * Otherwise, it would proceed.
 *
 * @param[in]  postulate A boolean expression that postulates size of a structure
 *
 * @retval `<compilation error>` - if postulate is FALSE 
 * @retval `<compilation proceeds>` - if postulate is TRUE
 *
 * <b>Typical Usage:</b><BR>
 * <code>
 * ... <arbitrary C code>....<BR>
 *
 * utlCheckSize(sizeof(xxxInst_t) <= MAX_BUF_SIZE); <BR>
 *
 * ... <code that may crash when sizeof(xxxInst_t) > MAX_BUF_SIZE> <BR>
 * </code>
 */
#define utlCheckSize(postulate)                                         \
   do {                                                                 \
       typedef struct {                                                 \
         uint16_t NegativeSizeIfPostulateFalse[((int)(postulate))*2 - 1];	\
       } PostulateCheck_t;                                              \
   }                                                                    \
   while (0)

/* Allay of PA memory buffer alignment requirements 
 *                PA_BUFFNUM_INSTANCE - the instance buffer
 *                PA_BUFFNUM_L2_TABLE - the L2 handle table
 *                PA_BUFFNUM_L3_TABLE - the L3 handle table
 *                PA_BUFFNUM_USR_STATS_LNK_TABLE - the User-defined statistics link table
 *                PA_BUFFNUM_VIRTUAL_LINK_TABLE - the virtual link table 
 *                PA_BUFFNUM_ACL_TABLE - the ACL handle table
 *                PA_BUFFNUM_FC_TABLE - the FC handle table
 */
const int paMemAligns[pa_N_BUFS_GEN2] = { 8, 8, 8, 8, 8, 8, 8, 8 };


/* Check that MAX_CMD_BUF_SIZE is correct */
#if    (pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES               > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES               > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_ADD_MAC_MIN_CMD_BUF_SIZE_BYTES                > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_ADD_MAC_MIN_CMD_BUF_SIZE_BYTES                > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES             > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES             > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_DEL_L4_HANDLE_MIN_CMD_BUF_SIZE_BYTES          > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_DEL_L4_HANDLE_MIN_CMD_BUF_SIZE_BYTES          > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_ADD_IP_MIN_CMD_BUF_SIZE_BYTES                 > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_ADD_IP_MIN_CMD_BUF_SIZE_BYTES                 > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES               > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES               > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_ADD_PORT_MIN_CMD_BUF_SIZE_BYTES               > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_ADD_PORT_MIN_CMD_BUF_SIZE_BYTES               > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_CONFIG_EXCEPTION_ROUTE_MIN_CMD_BUF_SIZE_BYTES > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_CONFIG_EXCEPTION_ROUTE_MIN_CMD_BUF_SIZE_BYTES > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_CONFIG_CRC_ENGINE_MIN_CMD_BUF_SIZE_BYTES      > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_CONFIG_CRC_ENGINE_MIN_CMD_BUF_SIZE_BYTES      > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_CONFIG_MULTI_ROUTE_MIN_CMD_BUF_SIZE_BYTES     > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_CONFIG_MULTI_ROUTE_MIN_CMD_BUF_SIZE_BYTES     > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_SET_CUSTOM_LUT1_MIN_CMD_BUF_SIZE_BYTES        > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_SET_CUSTOM_LUT1_MIN_CMD_BUF_SIZE_BYTES        > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_SET_CUSTOM_LUT2_MIN_CMD_BUF_SIZE_BYTES        > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_SET_CUSTOM_LUT2_MIN_CMD_BUF_SIZE_BYTES        > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_802_1ag_DET_MIN_CMD_BUF_SIZE_BYTES            > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_802_1ag_DET_MIN_CMD_BUF_SIZE_BYTES            > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_IPSEC_NAT_T_DET_MIN_CMD_BUF_SIZE_BYTES        > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_IPSEC_NAT_T_DET_MIN_CMD_BUF_SIZE_BYTES        > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_CONFIG_CMD_SET_MIN_CMD_BUF_SIZE_BYTES         > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_CONFIG_CMD_SET_MIN_CMD_BUF_SIZE_BYTES         > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_REQUEST_STATS_MIN_CMD_BUF_SIZE_BYTES          > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_REQUEST_STATS_MIN_CMD_BUF_SIZE_BYTES          > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES       > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES       > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

#if    (pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES          > pa_MAX_CMD_BUF_SIZE_BYTES)
#error (pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES          > pa_MAX_CMD_BUF_SIZE_BYTES)
#endif

/*
 * If the size of pa_MAX_CMD_BUF_SIZE_BYTES changes, it is the time to check
 * whether there are new definitions.
 */
#if   (pa_MAX_CMD_BUF_SIZE_BYTES != 2068)
#error The size of pa_MAX_CMD_BUF_SIZE_BYTES changes, check whether there are other related changes.
#endif  

/* PA Local object */
Pa_LocalObj    paLObj;

/**********************************************************************************
 * FUNCTION PURPOSE: Provide memory buffer allocation requirements
 **********************************************************************************
 * DESCRIPTION: Memory usage by the PA module is returned
 **********************************************************************************/
paReturn_t Pa_getBufferReq (paSizeInfo_t *sizeCfg, int sizes[], int aligns[])
{
  int i;  
  
  if((sizes == NULL) || (aligns == NULL))
    return (pa_INVALID_INPUT_POINTER);

  if((sizeCfg == NULL) || (sizeCfg->nUsrStats > pa_USR_STATS_MAX_COUNTERS))
      return (pa_ERR_CONFIG);
  
  sizes[PA_BUFFNUM_INSTANCE] = sizeof(paInst_t);
  
  /* L2 Table size requirements */
  sizes[PA_BUFFNUM_L2_TABLE] = sizeCfg->nMaxL2 * sizeof(paL2Entry_t);

  /* L3 Table size requirements */
  sizes[PA_BUFFNUM_L3_TABLE] = sizeCfg->nMaxL3 * sizeof(paL3Entry_t);
  
  /* User-defined statistics Link Table size requirements */
  sizes[PA_BUFFNUM_USR_STATS_LNK_TABLE] = sizeCfg->nUsrStats * sizeof(paUsrStatsLnkEntry_t);

  /* Virtual link table size requirements */
  sizes[PA_BUFFNUM_VIRTUAL_LNK_TABLE] = sizeCfg->nMaxVlnk * sizeof(paVirtualLnk_t);
  
  /* ACL Table size requirements */
  if (sizeCfg->nMaxAcl) {
    sizes[PA_BUFFNUM_ACL_TABLE] = sizeCfg->nMaxAcl * sizeof(paAclEntry_t);

    /* ACL table size increases due to Manual Rescore support as below */
    sizes[PA_BUFFNUM_ACL_TABLE] += sizeCfg->nMaxAcl * sizeof (paAclLinkListElem_t);
    sizes[PA_BUFFNUM_ACL_TABLE] += 2 * sizeof(paAclLinkListInfo_t); /* To accomodate both Outer and Inner tables */
  }
  else {
    sizes[PA_BUFFNUM_ACL_TABLE] = 0;
  }
  
  /* FC Table size requirements */
  sizes[PA_BUFFNUM_FC_TABLE] = sizeCfg->nMaxFc * sizeof(paFcEntry_t);

  /* Ethernet OAM table Size requirements */
  sizes[PA_BUFFNUM_EOAM_TABLE] = sizeCfg->nMaxEoam * sizeof(paL2Entry_t);  
  
  for ( i = 0; i < pa_N_BUFS_GEN2; i++)
  {
      aligns[i] = paMemAligns[i];
  }

  return (pa_OK);

} /* Pa_getBufferReq */

/********************************************************************
 * FUNCTION PURPOSE: Initialize global configurations
 ********************************************************************
 * DESCRIPTION: Initialize the global configuations to default
 ********************************************************************/
static void pa_init_global_cfg (paInst_t* paInst) 
{
    paSysInfo_t* pCfg = &paInst->cfg;
    volatile uint32_t *pGlobCfg = (volatile uint32_t *)&paLObj.pSysRegs->SRAM[PAFRM_SYS_GLOB_CFG_OFFSET];
    
    memset(pCfg, 0, sizeof(paSysInfo_t));
    
    if ((paLObj.pPpuRegs[0]->PDSP_CONTROL_STATUS.CONTROL & CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK) == CSL_PA_PDSP_CONTROL_PDSP_ENABLE_MASK)
    {
        /* 
         * PASS has already been configured because PDSP0 is running.
         * Extract the global configurations from PASS
         */ 
        uint32_t cfgData;
        
        /* Max header configuration */
        cfgData = pGlobCfg[PAFRM_SYS_GLOB_CFG_ID_MAX_HDR];
        pCfg->protoLimit.vlanMax         = (cfgData >> 24) & 0xFF;
        pCfg->protoLimit.ipMax           = (cfgData >> 16) & 0xFF;
        pCfg->protoLimit.greMax          = (cfgData >>  8) & 0xFF;
        
        /* Outer IP Configuration */
        cfgData = pGlobCfg[PAFRM_SYS_GLOB_CFG_ID_OUT_IP_REASSM];
        pCfg->outIpReassmConfig.numTrafficFlow  = (cfgData >> 24) & 0xFF;
        pCfg->outIpReassmConfig.destFlowId      = (cfgData >> 16) & 0xFF;
        pCfg->outIpReassmConfig.destQueue       = cfgData & 0xFFFF;
        
        /* Inner IP Configuration */
        cfgData = pGlobCfg[PAFRM_SYS_GLOB_CFG_ID_IN_IP_REASSM];
        pCfg->inIpReassmConfig.numTrafficFlow   = (cfgData >> 24) & 0xFF;
        pCfg->inIpReassmConfig.destFlowId       = (cfgData >> 16) & 0xFF;
        pCfg->inIpReassmConfig.destQueue        = cfgData & 0xFFFF;
        
        /* Command set configuration */
        cfgData = pGlobCfg[PAFRM_SYS_GLOB_CFG_ID_CMDSET];
        pCfg->cmdSetConfig.numCmdSets   = (cfgData >> 24) & 0xFF;
        
        /* User-defined statistics configuration */
        cfgData = pGlobCfg[PAFRM_SYS_GLOB_CFG_ID_USR_STATS];
        pCfg->usrStatsConfig.numCounters    = (cfgData >> 16) & 0xFFFF;
        pCfg->usrStatsConfig.num64bCounters = cfgData & 0xFFFF;
        
        /* Queue Diversion configuration */
        cfgData = pGlobCfg[PAFRM_SYS_GLOB_CFG_ID_QUEUE_DIVERT];
        pCfg->queueDivertConfig.destQueue   = (cfgData >> 16) & 0xFFFF;
        pCfg->queueDivertConfig.destFlowId  = (cfgData >>  8) & 0xFF;
        
        /* ACL Configuration */
        cfgData = pGlobCfg[PAFRM_SYS_GLOB_CFG_ID_OUT_IP_ACL];
        pCfg->outAclConfig.action        = (int) ((cfgData >>  24) & 0xFF);
        pCfg->outAclConfig.destFlowId    = (cfgData >> 16) & 0xFF;
        pCfg->outAclConfig.destQueue     = cfgData & 0xFFFF;
        
        cfgData = pGlobCfg[PAFRM_SYS_GLOB_CFG_ID_IN_IP_ACL];
        pCfg->inAclConfig.action        = (int) ((cfgData >>  24) & 0xFF);
        pCfg->inAclConfig.destFlowId    = (cfgData >> 16) & 0xFF;
        pCfg->inAclConfig.destQueue     = cfgData & 0xFFFF;
    
    }    
    else
    {
        /* 
         * PASS has not been configured because PDSP0 is not running.
         * Use the default lobal configurations
         */  
    
        pCfg->protoLimit.vlanMax         = pa_PROTOCOL_LIMIT_NUM_VLANS_DEF;
        pCfg->protoLimit.ipMax           = pa_PROTOCOL_LIMIT_NUM_IP_DEF;
        pCfg->protoLimit.greMax          = pa_PROTOCOL_LIMIT_NUM_GRE_DEF;
    
        pCfg->cmdSetConfig.numCmdSets    = pa_MAX_CMD_SETS;
    
        pCfg->usrStatsConfig.numCounters = paInst->nUsrStats;
    
        pCfg->outAclConfig.action        = pa_ACL_ACTION_PERMIT;
        pCfg->inAclConfig.action         = pa_ACL_ACTION_PERMIT;
    }
}

/********************************************************************
 * FUNCTION PURPOSE: Initialize system register map
 ********************************************************************
 * DESCRIPTION: Initialize all three-level PASS register sets of base 
 *              addresses
 ********************************************************************/
static void pa_init_regs (uint32_t baseAddr) 
{
  int i;
  
  paLObj.pSysRegs =  (CSL_Pa_ssRegs *)baseAddr;
  
  /* System Statistics initialization */
  paLObj.pSysRegs->STATS_CONTROL.ENABLE_ALLOC = CSL_PA_STATS_ENABLE_ALLOC_ENABLE_MASK;
  
  /* Initialize all clusters */
  for (i = 0; i < PASS_NUM_CLUSTERS; i++)
  {
    paLObj.pClRegs[i] = (CSL_Pa_clRegs *)(baseAddr + PASS_CLUSTER_OFFSET(i));
  }  
  
  paLObj.pPpuRegs[PASS_INGRESS0_PDSP0] = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_INGRESS0, 0));   
  paLObj.pPpuRegs[PASS_INGRESS0_PDSP1] = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_INGRESS0, 1));   
  paLObj.pPpuRegs[PASS_INGRESS1_PDSP0] = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_INGRESS1, 0));   
  paLObj.pPpuRegs[PASS_INGRESS1_PDSP1] = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_INGRESS1, 1));   
  paLObj.pPpuRegs[PASS_INGRESS2_PDSP0] = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_INGRESS2, 0));   
  paLObj.pPpuRegs[PASS_INGRESS3_PDSP0] = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_INGRESS3, 0));   
  paLObj.pPpuRegs[PASS_INGRESS4_PDSP0] = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_INGRESS4, 0));   
  paLObj.pPpuRegs[PASS_INGRESS4_PDSP1] = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_INGRESS4, 1));   
  paLObj.pPpuRegs[PASS_POST_PDSP0]     = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_POST, 0));   
  paLObj.pPpuRegs[PASS_POST_PDSP1]     = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_POST, 1));   
  paLObj.pPpuRegs[PASS_EGRESS0_PDSP0]  = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_EGRESS0, 0));   
  paLObj.pPpuRegs[PASS_EGRESS0_PDSP1]  = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_EGRESS0, 1));   
  paLObj.pPpuRegs[PASS_EGRESS0_PDSP2]  = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_EGRESS0, 2));   
  paLObj.pPpuRegs[PASS_EGRESS1_PDSP0]  = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_EGRESS1, 0));   
  paLObj.pPpuRegs[PASS_EGRESS2_PDSP0]  = (CSL_Pa_ppuRegs *)(baseAddr +  PASS_PPU_OFFSET(PASS_CLUSTER_EGRESS2, 0));  

}

/********************************************************************
 * FUNCTION PURPOSE: RA global configurations
 ********************************************************************
 * DESCRIPTION: RA global configuration
 ********************************************************************/
void pa_ra_global_cfg (paRaConfig_t *pRaCfg, CSL_Pa_ssRegs *pSysRegs) 
{
    uint32_t i;
    volatile uint32_t   *statReg1, *statReg2;
    
    pSysRegs->RA_BRIDGE.CONFIG = 6;  /* RA control data at psInfo byte 24 */
    for (i = 0; i < pa_RA_MAX_HEAP_REGIONS; i++)
    {
        pSysRegs->RA.HEAP_REGION[i].LOW =  pRaCfg->heapBase[i] & 0xFFFFFFFFUL; /* Context region */
        pSysRegs->RA.HEAP_REGION[i].HIGH = pRaCfg->heapBase[i] >> 32;          /* Context region */
    }
    
    pSysRegs->RA.CONFIG = pRaCfg->ipv4MinPktSize << CSL_PA_RA_CONFIG_IPV4_MIN_PKT_SIZE_SHIFT;
    pSysRegs->RA.TOTAL_CONTEXTS = pRaCfg->numCxts;
    pSysRegs->RA.DISCARD_THRESH = pRaCfg->cxtDiscardThresh + (pRaCfg->nodeDiscardThresh << CSL_PA_RA_DISCARD_THRESH_NODES_SHIFT);
    pSysRegs->RA.TICK_VAL = pRaCfg->clockRate >> 2; /* clock rate = 1/4 us) */
    pSysRegs->RA.TIMEOUT_VAL = (pRaCfg->cxtTimeout * 1000) << 2;
    pSysRegs->RA.HEAP_REGION_THRESH = pRaCfg->heapRegionThresh;
  
    statReg1 = (volatile uint32_t *)&pSysRegs->RA.STATS[0];
    statReg2 = (volatile uint32_t *)&pSysRegs->RA.STATS[1];
  
    for (i = 0; i < sizeof(CSL_Pa_raStatsRegs)/sizeof(uint32_t); i++)
    {
        statReg1[i] = CSL_PA_RA_STATS_ENABLE_MASK + PASS_RA_GROUP_STATS_BASE(0) + i;
        statReg2[i] = CSL_PA_RA_STATS_ENABLE_MASK + PASS_RA_GROUP_STATS_BASE(1) + i;
    }
}

/********************************************************************
 * FUNCTION PURPOSE: RA group configurations
 ********************************************************************
 * DESCRIPTION: RA group configuration
 ********************************************************************/
void pa_ra_group_cfg (paRaGroupConfig_t *pRaGroupCfg, CSL_Pa_raFlow_overrideRegs *pRaFlowRegs) 
{

    uint8_t threadId;
    
    threadId = (pRaGroupCfg->timeoutER.dest == pa_DEST_HOST)?PAFRM_DEST_PKTDMA:PAFRM_DEST_RA_DISCARD;
    pRaFlowRegs->TIMEOUT = (threadId << CSL_PA_RA_TIMEOUT_THREAD_ID_SHIFT)                         | 
                           (pRaGroupCfg->timeoutER.flowId << CSL_PA_RA_TIMEOUT_FLOW_INDEX_SHIFT)   |
                           (pRaGroupCfg->timeoutER.queue << CSL_PA_RA_TIMEOUT_DST_QUEUE_SHIFT);
                           
    threadId = (pRaGroupCfg->critErrER.dest == pa_DEST_HOST)?PAFRM_DEST_PKTDMA:PAFRM_DEST_RA_DISCARD;
    pRaFlowRegs->CRITICAL_ERR = (threadId << CSL_PA_RA_CRITICAL_ERR_THREAD_ID_SHIFT)                          | 
                                (pRaGroupCfg->critErrER.flowId << CSL_PA_RA_CRITICAL_ERR_FLOW_INDEX_SHIFT)   |
                                (pRaGroupCfg->critErrER.queue << CSL_PA_RA_CRITICAL_ERR_DST_QUEUE_SHIFT);
                                
    threadId = (pRaGroupCfg->genErrER.dest == pa_DEST_HOST)?PAFRM_DEST_PKTDMA:PAFRM_DEST_RA_DISCARD;
    pRaFlowRegs->NON_CRITICAL_ERR = (threadId << CSL_PA_RA_NON_CRITICAL_ERR_THREAD_ID_SHIFT)                          | 
                                    (pRaGroupCfg->genErrER.flowId << CSL_PA_RA_NON_CRITICAL_ERR_FLOW_INDEX_SHIFT)    |
                                    (pRaGroupCfg->genErrER.queue << CSL_PA_RA_NON_CRITICAL_ERR_DST_QUEUE_SHIFT);
                           
}



/***********************************************************************************
 * FUNCTION PURPOSE: Create a PA instance
 ***********************************************************************************
 * DESCRIPTION: An instance of the PA is created and initialized
 ***********************************************************************************/
paReturn_t Pa_create (paConfig_t *config, void* bases[], Pa_Handle *pHandle)
{
  paInst_t     *paInst = (paInst_t *)bases[PA_BUFFNUM_INSTANCE];
  int          sizes[pa_N_BUFS_GEN2], aligns[pa_N_BUFS_GEN2];
  paSizeInfo_t *sizeCfg;
  paRaConfig_t *raCfg;
  paL2Entry_t  *l2Table;
  paL3Entry_t  *l3Table;
  paAclEntry_t *aclTable;
  paFcEntry_t  *fcTable;
  paL2Entry_t  *eoamTable;  
  paUsrStatsLnkEntry_t *lnkTable;
  paVirtualLnk_t *vlnkTable;
  int          i;
  uint32_t     mtCsKey;

  /* Check for PA Base address null configurations */
  if (config->baseAddr == (uint32_t) NULL)
    return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
  
  if ((paInst == NULL) || (config == NULL)  || (pHandle == NULL) || (config->sizeCfg == NULL) || (config->sizeCfg->nUsrStats > pa_USR_STATS_MAX_COUNTERS) )  {
        return (pa_ERR_CONFIG);
  }
  
  /* For multiprocess support the PA handle does not hold the virtual address
   * instead it holds the offset for the buffer addresses */
  *pHandle = (Pa_Handle)(pa_CONV_BASE_TO_OFFSET(config->instPoolBaseAddr, bases[PA_BUFFNUM_INSTANCE])); 
  sizeCfg = config->sizeCfg; 
  
  if(Pa_getBufferReq (sizeCfg, sizes, aligns) != pa_OK)
    return (pa_ERR_CONFIG);
  
  /* Refresh PA Instance */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));

  /* Verify buffer base addresses */
  for (i = 0; i < pa_N_BUFS_GEN2; i++)  {
    if (bases[i] == NULL && sizes[i])
    {
        Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
        return (pa_ERR_CONFIG);
    }  
    paInst->paBufs[i].base = (void*) pa_CONV_BASE_TO_OFFSET(config->instPoolBaseAddr, bases[i]);
  }

  /* Determine the number of table entries. The result will always be
   * >= 0, so no check is required (0 is a valid number of entries). */
  paInst->nL2 = sizeCfg->nMaxL2;
  paInst->nL3 = sizeCfg->nMaxL3;
  paInst->nUsrStats = sizeCfg->nUsrStats;
  paInst->nMaxVlnk = sizeCfg->nMaxVlnk;
  paInst->nAcl = sizeCfg->nMaxAcl;
  paInst->nFc = sizeCfg->nMaxFc;
  paInst->nEoam = sizeCfg->nMaxEoam;
  paInst->paBufs[PA_BUFFNUM_L2_TABLE].size = sizes[PA_BUFFNUM_L2_TABLE];
  paInst->paBufs[PA_BUFFNUM_L3_TABLE].size = sizes[PA_BUFFNUM_L3_TABLE];
  paInst->paBufs[PA_BUFFNUM_USR_STATS_LNK_TABLE].size = sizes[PA_BUFFNUM_USR_STATS_LNK_TABLE];
  paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size = sizes[PA_BUFFNUM_VIRTUAL_LNK_TABLE];
  paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size = sizes[PA_BUFFNUM_ACL_TABLE];
  paInst->paBufs[PA_BUFFNUM_FC_TABLE].size = sizes[PA_BUFFNUM_FC_TABLE];
  paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size = sizes[PA_BUFFNUM_EOAM_TABLE];  
  
  if (config->initTable)  {

    l2Table = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(config->instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L2_TABLE].base);

    Pa_osalBeginMemAccess ((void*) l2Table, 
                           paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);

    memset (l2Table, 0, paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);

    /* The status value PA_TBL_STAT_INACTIVE should have a value of 0, but take the safe approach */
    for (i = 0; i < paInst->nL2; i++) {
      l2Table[i].hdr.status    = PA_TBL_STAT_INACTIVE;
      l2Table[i].hdr.tableIdx  = i;
      l2Table[i].hdr.type      = PA_TABLE_ENTRY_TYPE_L2;
    }

    Pa_osalEndMemAccess ((void*) l2Table, 
                         paInst->paBufs[PA_BUFFNUM_L2_TABLE].size);
    l3Table = (paL3Entry_t *)pa_CONV_OFFSET_TO_BASE(config->instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_L3_TABLE].base);

    Pa_osalBeginMemAccess ((void*) l3Table,
                           paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);

    memset (l3Table, 0, paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);

    for (i = 0; i < paInst->nL3; i++)  {
      l3Table[i].hdr.status   = PA_TBL_STAT_INACTIVE;
      l3Table[i].hdr.tableIdx = i;
      l3Table[i].hdr.type     = PA_TABLE_ENTRY_TYPE_L3;
    }

    Pa_osalEndMemAccess ((void*) l3Table,
                         paInst->paBufs[PA_BUFFNUM_L3_TABLE].size);

    if (paInst->nEoam) 
    {
      eoamTable = (paL2Entry_t *)pa_CONV_OFFSET_TO_BASE(config->instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].base);

      Pa_osalBeginMemAccess ((void*) eoamTable, 
                             paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);

      memset (eoamTable, 0, paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);

      /* The status value PA_TBL_STAT_INACTIVE should have a value of 0, but take the safe approach */
      for (i = 0; i < paInst->nEoam; i++) {
        eoamTable[i].hdr.status    = PA_TBL_STAT_INACTIVE;
        eoamTable[i].hdr.tableIdx  = i;
        eoamTable[i].hdr.type      = PA_TABLE_ENTRY_TYPE_L2;
      }

      Pa_osalEndMemAccess ((void*) eoamTable, 
                           paInst->paBufs[PA_BUFFNUM_EOAM_TABLE].size);
    }     
                         
    if (paInst->nAcl)
    {
        aclTable    = (paAclEntry_t *)pa_CONV_OFFSET_TO_BASE(config->instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_ACL_TABLE].base);

        Pa_osalBeginMemAccess (aclTable,
                               paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);

        memset (aclTable, 0, paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);

        for (i = 0; i < paInst->nAcl; i++)  {
            aclTable[i].hdr.status   = PA_TBL_STAT_INACTIVE;
            aclTable[i].hdr.tableIdx = i;
            aclTable[i].hdr.type     = PA_TABLE_ENTRY_TYPE_ACL;
        }

        Pa_osalEndMemAccess (aclTable,
                             paInst->paBufs[PA_BUFFNUM_ACL_TABLE].size);
    }       
    
    if (paInst->nFc)
    {
        fcTable = (paFcEntry_t *)pa_CONV_OFFSET_TO_BASE(config->instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_FC_TABLE].base);

        Pa_osalBeginMemAccess (fcTable,
                               paInst->paBufs[PA_BUFFNUM_FC_TABLE].size);

        memset (fcTable, 0, paInst->paBufs[PA_BUFFNUM_FC_TABLE].size);

        for (i = 0; i < paInst->nFc; i++)  {
            fcTable[i].hdr.status   = PA_TBL_STAT_INACTIVE;
            fcTable[i].hdr.tableIdx = i;
            fcTable[i].hdr.type     = PA_TABLE_ENTRY_TYPE_FC;
        }

        Pa_osalEndMemAccess (fcTable,
                             paInst->paBufs[PA_BUFFNUM_FC_TABLE].size);
    }       
    
  }
  
  /* Initialize the user-defined statistics link table */
  if (paInst->nUsrStats)
  {
  
    lnkTable = (paUsrStatsLnkEntry_t *)pa_CONV_OFFSET_TO_BASE(config->instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_USR_STATS_LNK_TABLE].base);

    Pa_osalBeginMemAccess ((void *) lnkTable,
                             paInst->paBufs[PA_BUFFNUM_USR_STATS_LNK_TABLE].size);

      for (i = 0; i < paInst->nUsrStats; i++)  {
          lnkTable[i].lnkIndex = PA_USR_STATS_LNK_END;
      }

    Pa_osalEndMemAccess ((void*) lnkTable,
                           paInst->paBufs[PA_BUFFNUM_USR_STATS_LNK_TABLE].size);
  }
  
  /* Initialize the virtual link table */
  if(paInst->nMaxVlnk)
  {
    vlnkTable = (paVirtualLnk_t *)pa_CONV_OFFSET_TO_BASE(config->instPoolBaseAddr, paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].base);

    Pa_osalBeginMemAccess ((void*) vlnkTable,
                           paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);

    for (i = 0; i < paInst->nMaxVlnk; i++)  {
      vlnkTable[i].hdr.lnkCnt   = 0;
      vlnkTable[i].hdr.status   = PA_TBL_STAT_INACTIVE;
      vlnkTable[i].hdr.tableIdx = i;
      vlnkTable[i].hdr.type     = PA_TABLE_ENTRY_TYPE_VL;
      vlnkTable[i].hdr.pdspNum  = 0;
      vlnkTable[i].hdr.lutIdx   = 0;
    }

    Pa_osalEndMemAccess ((void*) vlnkTable,
                         paInst->paBufs[PA_BUFFNUM_VIRTUAL_LNK_TABLE].size);
  }

  /* Record the instance memory pool address and PASS base address */
  paLObj.cfg.instPoolBaseAddr = config->instPoolBaseAddr;
  paLObj.cfg.baseAddr         = config->baseAddr;
  paLObj.cfg.rmServiceHandle  = config->rmServiceHandle;

  /* Initialize all three-level PASS register sets */
  pa_init_regs(config->baseAddr);
  
  /* Global initialization */
  pa_init_global_cfg(paInst);
  
  /* System Statistics initialization */
  paLObj.pSysRegs->STATS_CONTROL.ENABLE_ALLOC = CSL_PA_STATS_ENABLE_ALLOC_ENABLE_MASK;
  paLObj.pSysRegs->STATS_CONTROL.SOFT_RESET = 1;
  
  
  if (config->initDefaultRoute)
  {
    for(i = 0; i < 16; i++)
      paLObj.pSysRegs->CPSW_THREAD_MAPPER.MAP[i] = (PAFRM_CPSW_DEST_INGRESS0 << CSL_CPSW_THREAD_MAPPER_MAP_TARGET0_SHIFT) |
                                                   (PAFRM_CPSW_DEST_INGRESS0 << CSL_CPSW_THREAD_MAPPER_MAP_TARGET1_SHIFT) |
                                                   (PAFRM_CPSW_DEST_INGRESS0 << CSL_CPSW_THREAD_MAPPER_MAP_TARGET2_SHIFT) |
                                                   (PAFRM_CPSW_DEST_INGRESS0 << CSL_CPSW_THREAD_MAPPER_MAP_TARGET3_SHIFT);
  }
  
  /* Initialize all clusters */
  for (i = 0; i < PASS_NUM_CLUSTERS; i++)
  {
    //paLObj.pClRegs[i]->SPLITTER.EOP_CTRL = 254;
    paLObj.pClRegs[i]->SPLITTER.EOP_CTRL = 128;
    paLObj.pClRegs[i]->SPLITTER.MOP_BUF_SIZE = (i == 0)?0x4000:0x10000;
    paLObj.pClRegs[i]->SPLITTER.MOP_BUF_PTR = 0xFFFC0000;
    //paLObj.pClRegs[i]->SPLITTER.SOP_CTRL = CSL_PA_SPLITTER_SOP_CTRL_ENABLE_MASK | (1518 - 254);
    //To avoid hardware bug: SOP + EOP + 32 + Control size <= 128
    //Restrict control size to 96, the SOP should be 896 - 128
    paLObj.pClRegs[i]->SPLITTER.SOP_CTRL = CSL_PA_SPLITTER_SOP_CTRL_ENABLE_MASK | (896 - 128);
  }  
  
  /* RA module initialization */
  if ((raCfg = config->raCfg))
  {
    pa_ra_global_cfg(raCfg, paLObj.pSysRegs);
  }
  
  PA_SET_STATE_READY(paInst, 1);
  PA_SET_STATE_GTPU_LINK(paInst, 0);
  PA_SET_STATE_EOAM(paInst, 0); /* Default EOAM is not enabled */
  
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  /* Sanity check the defined sizes. This is effectively a compile time check */
  utlCheckSize(pa_ADD_MAC_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandAddLut1_t))); 
  utlCheckSize(pa_ADD_IP_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandAddLut1_t)*2 + sizeof(pafrmCommandCmdHdr_t)*2));
  utlCheckSize(pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandAddLut1_t))); 
  utlCheckSize(pa_DEL_HANDLE_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandDelLut1_t)*2 + sizeof(pafrmCommandCmdHdr_t)*2));
  utlCheckSize(pa_DEL_L4_HANDLE_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof (pafrmCommandDelLut2_t)));
  utlCheckSize(pa_ADD_PORT_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandAddLut2_t)));
  utlCheckSize(pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandAddLut2_t)));
  utlCheckSize(pa_CONFIG_CMD_SET_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandCmdSet_t)));
  utlCheckSize(pa_CONFIG_CRC_ENGINE_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandConfigCRC_t)));
  utlCheckSize(pa_CONFIG_MULTI_ROUTE_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandMultiRoute_t)));
  utlCheckSize(pa_CONFIG_EXCEPTION_ROUTE_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmComEroute_t)));
  utlCheckSize(pa_SET_CUSTOM_LUT1_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmC1Custom_t)));
  utlCheckSize(pa_SET_CUSTOM_LUT2_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmC2Custom_t)));
  utlCheckSize(pa_802_1ag_DET_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrm802p1agDet_t)));
  utlCheckSize(pa_IPSEC_NAT_T_DET_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmIpsecNatTDet_t)));
  utlCheckSize(pa_GTPU_CONFIG_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmGtpuCfg_t)));
  utlCheckSize(pa_EMAC_PORT_PKT_CAPTURE_CONFIG_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmPktCapCfg_t)));
  utlCheckSize(pa_EMAC_PORT_MIRROR_CONFIG_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmPktCapCfg_t) - sizeof(pafrmPktCapIfCfg_t)));
  utlCheckSize(pa_EMAC_PORT_DEFAULT_ROUTE_CONFIG_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmDefRouteCfg_t)));
  utlCheckSize(pa_EMAC_PORT_EQoS_MODE_CONFIG_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmEQosModeConfig_t)));
  utlCheckSize(pa_EMAC_PORT_CONFIG_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + offsetof(pafrmCommandSysConfigPa_t, u) + sizeof(pafrmEQosModeConfig_t)));
  utlCheckSize(pa_REQUEST_STATS_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandReqStats_t)));
  utlCheckSize(pa_GLOBAL_CONFIG_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmCommandConfigPa_t)));
  utlCheckSize(pa_CONFIG_USR_STATS_MIN_CMD_BUF_SIZE_BYTES >= (sizeof(pafrmCommand_t) - sizeof(uint32_t) + sizeof(pafrmUsrStatsCntCfg_t) + sizeof(pafrmUsrStatsEntry_t)*pa_USR_STATS_MAX_COUNTERS));
                 
  return (pa_OK);

} /* Pa_create */

/***********************************************************************************
 * FUNCTION PURPOSE: Adds PA configuration
 ***********************************************************************************
 * DESCRIPTION: This function needs to be called from all cores to initialize PA with 
 *           per core configurations
 ***********************************************************************************/
paReturn_t Pa_startCfg (Pa_Handle pHandle, paStartCfg_t *startCfg)
{

    pHandle = pHandle; /* Eliminate compiler warning of unused parameter */
    if (startCfg)
    {
        paLObj.cfg = *startCfg;
        if (startCfg->baseAddr)
          pa_init_regs(startCfg->baseAddr);
        else
          return (pa_SUB_SYSTEM_BASE_ADDR_NULL);
    }
    else {
      return (pa_INVALID_INPUT_POINTER);
    }

    return (pa_OK);
}

/**********************************************************************************************
 * FUNCTION PURPOSE: Return memory buffer usage for application to free
 **********************************************************************************************
 * DESCRIPTION: Returns the base address, size, alignment, type and memory space
 *              of all buffers previously allocated for the PA drivers instance.
 **********************************************************************************************/
paReturn_t Pa_close (Pa_Handle handle, void* bases[])
{
  paInst_t *paInst = (paInst_t *)pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, handle);
  int i;
  uint32_t mtCsKey;
  
  if (bases == NULL)
    return (pa_INVALID_INPUT_POINTER);
  
  /* Refresh PA Instance for read only */
  Pa_osalMtCsEnter(&mtCsKey);
  Pa_osalBeginMemAccess (paInst, sizeof(paInst_t));
  
  for ( i = 0; i < pa_N_BUFS_GEN2; i++)
  {
    bases[i] = (void *) pa_CONV_OFFSET_TO_BASE(paLObj.cfg.instPoolBaseAddr, paInst->paBufs[i].base);
  }
  
  Pa_osalEndMemAccess (paInst, sizeof(paInst_t));
  Pa_osalMtCsExit(mtCsKey);  

  return (pa_OK);

} /* Pa_close */















