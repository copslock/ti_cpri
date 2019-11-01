/**  
 * @file pa_mgmt.c
 *
 * @brief 
 *  Packet accelerator subsystem management functions.
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
#endif

/* PASS RL file */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_pa_ss.h>

/* Firmware images */
#include <ti/drv/pa/fw/pafw.h>


/* PASS Global configurations */
#define PASS_NUM_64B_USR_STATS            64

static paUsrStatsConfig_t   paUsrStatsCfg =
       {
            pa_USR_STATS_MAX_COUNTERS - PASS_NUM_64B_USR_STATS,   /* Number of user stats (448)*/
            PASS_NUM_64B_USR_STATS                                 /* Number of 64-bit user stats */
       };   
        
static  paSysConfig_t  paGlobalCfg = 
        {
            NULL,               /* pProtoLimit */
            NULL,               /* pOutIpReassmConfig */
            NULL,               /* pInIpReassmConfig */
            NULL,               /* pCmdSetConfig */
            &paUsrStatsCfg,     /* pUsrStatsConfig */
            NULL,               /* pQueueDivertConfig */
            NULL,               /* pPktControl */
            NULL,               /* pOutAclConfig  */
            NULL,               /* pInAclConfig */   
            NULL,               /* pOutIpRaGroupConfig */
            NULL,               /* pInIpRaGroupConfig */
            NULL,               /* pPktControl2 */ 
            NULL                /* pEoamConfig */          
        };


/* User-defined statistics related definitions and data structures */
#define MAX_USR_STATS_TBL_SIZE        64
#define USR_STATS_NO_LINK             -1
typedef struct UsrStats_Entry_s
{
    uint16_t    cntIndex;    /* counter index */
    int         lnkId;       /* link counter offset within the User-status entry table */
    int         fAlloc;      /* Allocate this counter */
    int         f64b;        /* TRUE: 64-byte counter */
    int         fByteCnt;    /* TRUE: Byte counter */
} UsrStats_Entry_t;

#define NUM_USR_STATS                  2
#define USR_STATS_ID_RX_PKTS           0
#define USR_STATS_ID_RX_BYTES          1

static  UsrStats_Entry_t usrStatsTbl[NUM_USR_STATS] =
{
    /* index, lnkId, fAlloc, f64b, fByteCnt */
    {0, USR_STATS_ID_RX_BYTES, TRUE,   FALSE, FALSE},      /* Rx packet count */
    {0, USR_STATS_NO_LINK,     TRUE,   TRUE,  TRUE},       /* Rx byte count */
};

/*
 * User-definded Statitics Map
 *
 * Group 1: counter #0,  packet Counter (Rx Pkt Counter) linked to Rx Byte Counter
 *          counter #1,  packet Counter (Rx Byte Counter) no link
 */
static paUsrStatsCounterEntryConfig_t usrStatsGroup[NUM_USR_STATS];

#ifndef __LINUX_USER_SPACE
extern volatile unsigned int cregister TSCL;
/** ============================================================================
 *   @n@b Download_PAFirmware
 *
 *   @b Description
 *   @n This API downloads the PA firmware required for PDSP operation.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Download_PAFirmware (void)
{

#ifndef NSS_GEN2 
    
    int i;

    /* Hold the PA in reset state during download */
    Pa_resetControl (gPAInstHnd, pa_STATE_RESET);

    /* PDPSs 0-2 use image c1 */
    Pa_downloadImage (gPAInstHnd, 0, (Ptr)c1_0, c1_0Size);
    Pa_downloadImage (gPAInstHnd, 1, (Ptr)c1_1, c1_1Size);
    Pa_downloadImage (gPAInstHnd, 2, (Ptr)c1_2, c1_2Size);

    /* PDSP 3 uses image c2 */
    Pa_downloadImage (gPAInstHnd, 3, (Ptr)c2, c2Size);

    /* PDSPs 4-5 use image m */
    for (i = 4; i < 6; i++)
    {
        Pa_downloadImage (gPAInstHnd, i, (Ptr)m, mSize);
    }
    
#else
    Pa_resetControl (gPAInstHnd, pa_STATE_RESET);
  
    Pa_downloadImage (gPAInstHnd, 0, (Ptr)in0_pdsp0, in0_pdsp0Size);  
    Pa_downloadImage (gPAInstHnd, 1, (Ptr)in0_pdsp1, in0_pdsp1Size);  
    Pa_downloadImage (gPAInstHnd, 2, (Ptr)in1_pdsp0, in1_pdsp0Size);  
    Pa_downloadImage (gPAInstHnd, 3, (Ptr)in1_pdsp1, in1_pdsp1Size);  
    Pa_downloadImage (gPAInstHnd, 4, (Ptr)in2_pdsp0, in2_pdsp0Size);  
    Pa_downloadImage (gPAInstHnd, 5, (Ptr)in3_pdsp0, in3_pdsp0Size);  
    Pa_downloadImage (gPAInstHnd, 6, (Ptr)in4_pdsp0, in4_pdsp0Size);  
    Pa_downloadImage (gPAInstHnd, 7, (Ptr)in4_pdsp1, in4_pdsp1Size);  
    Pa_downloadImage (gPAInstHnd, 8, (Ptr)post_pdsp0, post_pdsp0Size);  
    Pa_downloadImage (gPAInstHnd, 9, (Ptr)post_pdsp1, post_pdsp1Size);  
    Pa_downloadImage (gPAInstHnd, 10, (Ptr)eg0_pdsp0, eg0_pdsp0Size);  
    Pa_downloadImage (gPAInstHnd, 11, (Ptr)eg0_pdsp1, eg0_pdsp1Size);  
    Pa_downloadImage (gPAInstHnd, 12, (Ptr)eg0_pdsp2, eg0_pdsp2Size);  
    Pa_downloadImage (gPAInstHnd, 13, (Ptr)eg1_pdsp0, eg1_pdsp0Size);  
    Pa_downloadImage (gPAInstHnd, 14, (Ptr)eg2_pdsp0, eg2_pdsp0Size);  

#endif    

    /* Enable the PA back */
    Pa_resetControl (gPAInstHnd, pa_STATE_ENABLE);

    return 0;
}        
#endif

/** ============================================================================
 *   @n@b Alloc_UsrStats
 *
 *   @b Description
 *   @n This API allocates a set of user-defined statistics and fills out 
 *      the corresponding user-defined statistics counter configuration
 *      table. 
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Alloc_UsrStats (int numCnts, UsrStats_Entry_t *usrStatsTbl, paUsrStatsCounterEntryConfig_t *usrStatsCfgTbl)
{
    paUsrStatsAlloc_t usrStatsAlloc[MAX_USR_STATS_TBL_SIZE];
	paReturn_t        paret = pa_OK;
    int i, numAlloc;

    if (numCnts > MAX_USR_STATS_TBL_SIZE)
    {
	    System_printf ("%s:%d: Alloc_UsrStats: numCnts (%d) exceeds the limit (%d)\n", 
	  	    			__FILE__, __LINE__, numCnts, MAX_USR_STATS_TBL_SIZE);
    
        return (-1);
    }    
        
     /* Construct the user-defined statistics allocation table */
     memset(usrStatsAlloc, 0, sizeof(paUsrStatsAlloc_t)*numCnts);
     for (i = 0;  i < numCnts; i++)
     {
        if (!usrStatsTbl[i].fAlloc)
        {
            usrStatsAlloc[i].ctrlBitfield = pa_USR_STATS_ALLOC_CNT_PRESENT;
            usrStatsAlloc[i].cntIndex = usrStatsTbl[i].cntIndex;
        }
     
        if(usrStatsTbl[i].f64b)
            usrStatsAlloc[i].ctrlBitfield |= pa_USR_STATS_ALLOC_64B_CNT;
     }   
     
     numAlloc = numCnts;
     
     paret = Pa_allocUsrStats(gPAInstHnd, &numAlloc, usrStatsAlloc);
     
     if ((paret != pa_OK) || (numAlloc < numCnts))
     {
	    System_printf ("%s:%d: Alloc_UsrStats: Pa_allocUsrStats returns %d with %d user-defined stats allocated out of %d\n", 
	  	    			 __FILE__, __LINE__, paret, numAlloc, numCnts);
        return (-1);
     }
     
	 //System_printf ("Alloc_UsrStats(%d): index[0] = %d, index[1] = %d\n", coreNum, usrStatsAlloc[0].cntIndex, usrStatsAlloc[1].cntIndex);
     
     /* Construct the usrStats Configuration table */
     for ( i = 0; i < numCnts; i++)
     {
        usrStatsCfgTbl[i].cntIndex = usrStatsAlloc[i].cntIndex;
        if (usrStatsTbl[i].fAlloc)
            usrStatsTbl[i].cntIndex = usrStatsAlloc[i].cntIndex;
        if(usrStatsTbl[i].lnkId != USR_STATS_NO_LINK)
            usrStatsCfgTbl[i].cntLnk = usrStatsAlloc[usrStatsTbl[i].lnkId].cntIndex;
        else
            usrStatsCfgTbl[i].cntLnk = pa_USR_STATS_LNK_END;
        usrStatsCfgTbl[i].cntType = usrStatsTbl[i].fByteCnt?pa_USR_STATS_TYPE_BYTE:pa_USR_STATS_TYPE_PACKET;              
     }
     
     return (0);   
}

/** ============================================================================
 *   @n@b Free_UsrStats
 *
 *   @b Description
 *   @n This API Frees a set of user-defined statistics.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int Free_UsrStats (int numCnts, UsrStats_Entry_t *usrStatsTbl)
{
    uint16_t        usrStats[MAX_USR_STATS_TBL_SIZE];
	paReturn_t      paret = pa_OK;
    int i, numFree;

    if(numCnts > MAX_USR_STATS_TBL_SIZE)
    {
	    System_printf ("%s:%d: Free_UsrStats: numCnts (%d) exceeds the limit (%d)\n", 
	  	    			__FILE__, __LINE__, numCnts, MAX_USR_STATS_TBL_SIZE);
    
        return (-1);
    }    
        
     /* Construct the user-defined statistics allocation table */
     for (i = 0, numFree = 0;  i < numCnts; i++)
     {
        if (usrStatsTbl[i].fAlloc)
        {
            usrStats[numFree++] = usrStatsTbl[i].cntIndex;
        }
     }   
     
     paret = Pa_freeUsrStats(gPAInstHnd, numFree, usrStats);
     
     if (paret != pa_OK)
     {
	    System_printf ("%s:%d: Free_UsrStats: Pa_freeUsrStats returns %d\n", 
	  	    			__FILE__, __LINE__, paret);
        return (-1);
     }
     
     return (0);   
}

/** ============================================================================
 *   @n@b Config_UsrStats
 *
 *   @b Description
 *   @n This API adds a set of user-defined statistics link at the usrStats Link 
 *      table of LLD and PASS. 
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Config_UsrStats (int numCnts, paUsrStatsCounterEntryConfig_t *usrStatsCfgTbl) 
{
	int 			            j;
	int  			            cmdDest;
	uint16_t		            cmdSize;
    paReturn_t                  retVal;
    paEntryHandle_t             retHandle;
    int32_t                     handleType;
    uint32_t                    psCmd = PASAHO_PACFG_CMD;  
    Cppi_HostDesc*              pHostDesc;
    
    paUsrStatsCounterConfig_t   cntCfg;
    paUsrStatsConfigInfo_t      statsCfgInfo;
    
    paCmdReply_t cmdReplyInfo = {  pa_DEST_HOST,			/* Dest */
					               0,						/* Reply ID (returned in swinfo0) */
					   	           0,						/* Queue */
					               0 };						/* Flow ID */
                               

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }
    
    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
#ifndef __LINUX_USER_SPACE	
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550010;  /* unique for each user stats configuration  command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);
    
    /* Prepare and format the command */                           
    memset(&statsCfgInfo, 0, sizeof(statsCfgInfo));
    memset(&cntCfg, 0, sizeof(cntCfg));                         
    cntCfg.numCnt  = numCnts;
    cntCfg.cntInfo = usrStatsCfgTbl;
    statsCfgInfo.pCntCfg = &cntCfg;
    
	retVal = Pa_configUsrStats (gPAInstHnd,
					            &statsCfgInfo,
					            (paCmd_t) pHostDesc->buffPtr,
                                &cmdSize,
                                &cmdReplyInfo,
                                &cmdDest);
                                
    if (retVal != pa_OK)  
    {
        System_printf ("Config_UsrStats returned error %d\n", retVal);
        return -1;
    }
    
    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;
    
    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
       
    /* Send the command to the PA and wait for the return */
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    Qmss_queuePush (gPaTxQHnd[cmdDest], 
                    pHostDesc, 
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                   );
    
    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * PASS Configuration.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Config_UsrStats: Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
                SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }
            

            retVal  =   Pa_forwardResult (gPAInstHnd, (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)  
            {
                System_printf ("PA sub-system rejected Pa_configUsrStats command\n");
                return -1;
            }
        
            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_configUsrStats command\n");
        return -1;
    }

    return 0;
}

/** ============================================================================
 *   @n@b Reset_UsrStats
 *
 *   @b Description
 *   @n This API clears a set of user-defined statistics link at the usrStats Link 
 *      table of LLD and PASS. 
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Reset_UsrStats (int numCnts, paUsrStatsCounterEntryConfig_t *usrStatsCfgTbl) 
{

    int i;
    
    for(i = 0; i < numCnts; i++)
        usrStatsCfgTbl[i].cntType = pa_USR_STATS_TYPE_DISABLE;
        
    return(Config_UsrStats(numCnts, usrStatsCfgTbl));        
}

/** ============================================================================
 *   @n@b Get_UsrStats
 *
 *   @b Description
 *   @n This API request, dispaly and reset the set of user-stats used by 
 *      application.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
static paUsrStats_t paUsrStats;
static uint8_t paCmdBuf[3000];
 
int32_t Get_UsrStats (int portNum, int numCnts, UsrStats_Entry_t *usrStatsTbl)
{

    int32_t                     j;
    uint16_t                    cmdSize;
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */            
                                                    0,                                          /* User chosen ID to go to swinfo0 */     
                                                    0,                                          /* Destination queue */                   
                                                    0                                           /* Flow ID */  
                                                };
    paReturn_t                  retVal;
    int32_t                     cmdDest;
    uint32_t                    psCmd       =   PASAHO_PACFG_CMD;  
    Cppi_HostDesc               *pHostDesc, *nextHdr;
    uint16_t                    usrCnt[MAX_USR_STATS_TBL_SIZE];
    
    if (numCnts > MAX_USR_STATS_TBL_SIZE)
    {
	    System_printf ("%s:%d: Get_UsrStats: numCnts (%d) exceeds the limit (%d)\n", 
	  	    			__FILE__, __LINE__, numCnts, MAX_USR_STATS_TBL_SIZE);
    
        return (-1);
    }    
    
    for (j = 0; j < numCnts; j++)
        usrCnt[j] = usrStatsTbl[j].cntIndex;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
#ifndef __LINUX_USER_SPACE	
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550020;  /* unique for user stats request command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);
    
	retVal = Pa_requestUsrStatsList (gPAInstHnd, 
                                     TRUE, 
                                     numCnts, 
                                     usrCnt, 
					                 (paCmd_t) pHostDesc->buffPtr,
					                 &cmdSize,
					                 &cmdReplyInfo,
					                 &cmdDest,
                                     NULL);

    if (retVal != pa_OK)  
    {
        System_printf ("Pa_requestUsrStatsList returned error %d\n", retVal);
        return -1;
    }
    
    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;
    
    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
       
    /* Send the command to the PA and wait for the return */
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    Qmss_queuePush (gPaTxQHnd[cmdDest], 
                    pHostDesc, 
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                   );
                   
    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            int offset = 0;
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * MAC address addition.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
            
            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Get_UsrStats: Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                
                return -1;
            }
            
            do                
            {   
                memcpy(paCmdBuf + offset, (uint8_t *)pHostDesc->buffPtr, pHostDesc->buffLen);
                offset += pHostDesc->buffLen;             
                nextHdr = (Cppi_HostDesc *)pHostDesc->nextBDPtr;
                pHostDesc->nextBDPtr = 0;
                pHostDesc->buffLen   = pHostDesc->origBufferLen;
                SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
            } while ((pHostDesc = nextHdr));
            
            retVal  =   Pa_formatUsrStatsReply (gPAInstHnd, (Ptr)paCmdBuf, &paUsrStats);
            if (retVal != pa_OK)  
            {
                System_printf ("PA sub-system rejected Pa_formatUsrStatsReply command\n");
                return -1;
            }
            
            System_printf ("--- PA (%d) User-defined STATS --- \n", portNum);
		    System_printf ("number of Rx bytes(index = %d):    %u\n", usrCnt[USR_STATS_ID_RX_BYTES], (uint32_t)paUsrStats.count64[usrCnt[USR_STATS_ID_RX_BYTES]]);
            System_printf ("number of Rx packets (index = %d): %u\n", usrCnt[USR_STATS_ID_RX_PKTS], paUsrStats.count32[usrCnt[USR_STATS_ID_RX_PKTS]-PASS_NUM_64B_USR_STATS]);
            System_flush();
            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_requestUsrStatsList\n");
        return -1;
    }

    return 0;
                       
}

/** ============================================================================
 *   @n@b Glob_Config
 *
 *   @b Description
 *   @n This API prepares and send a global configuration command packet to
 *      PASS.  
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Glob_Config (paSysConfig_t *pCfg)
{
    int32_t                     j;
    uint16_t                    cmdSize;
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */            
                                                    0,                                          /* User chosen ID to go to swinfo0 */     
                                                    0,                                          /* Destination queue */                   
                                                    0                                           /* Flow ID */  
                                                };
    paReturn_t                  retVal;
    int32_t                     cmdDest;
    uint32_t                    psCmd       =   PASAHO_PACFG_CMD;  
    Cppi_HostDesc*              pHostDesc;
    paCtrlInfo_t                ctrlInfo;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
#ifndef __LINUX_USER_SPACE	
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550011;  /* unique for global config command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);
    
    memset(&ctrlInfo, 0, sizeof(paCtrlInfo_t));
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    ctrlInfo.params.sysCfg = *pCfg;
    
	retVal = Pa_control (gPAInstHnd,
					     &ctrlInfo,
					     (paCmd_t) pHostDesc->buffPtr,
					     &cmdSize,
					     &cmdReplyInfo,
					     &cmdDest);

    if (retVal != pa_OK)  
    {
        System_printf ("Pa_control returned error %d\n", retVal);
        return -1;
    }
    
    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;
    
    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
       
    /* Send the command to the PA and wait for the return */
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    Qmss_queuePush (gPaTxQHnd[cmdDest], 
                    pHostDesc, 
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                   );
    
    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * PASS global configuration.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Glob_Config: Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
                SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }
        
            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_control command\n");
        return -1;
    }

    return 0;
}

/** ============================================================================
 *   @n@b Add_MACAddress
 *
 *   @b Description
 *   @n This API adds the switch MAC address to the PA PDSP Lookup table. This 
 *      ensures that all packets destined for this MAC address get processed
 *      for forwarding to the host.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Add_MACAddress (void)
{
    int32_t                     j;
    uint16_t                    cmdSize;
    paEthInfo_t                 ethInfo     =   {   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },     /* Src mac = dont care */   
                                                    { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15 },     /* Dest mac */
                                                    0,                                          /* vlan = dont care */      
                                                    0x0800,                             		/* ether type = IPv4 */     
                                                    0,                                          /* MPLS tag = dont care */
                                                    0                                           /* Input EMAC port = dont care */  
                                                }; 
    paRouteInfo_t               routeInfo =     {   pa_DEST_CONTINUE_PARSE_LUT1,                /* Continue parsing */             
                                                    0,                                          /* Flow Id = dont care */          
                                                    0,                                          /* queue = dont care */            
                                                    0,                                          /* multi route = dont care */      
                                                    0,                                          /* swinfo0 = dont care */          
                                                    0,                                          /* SwInfo 1 is dont care */
                                                    0,                                          /* customType = pa_CUSTOM_TYPE_NONE */         \
                                                    0,                                          /* customIndex: not used */        \
                                                    0,                                          /* pkyType: for SRIO only */       \
                                                    NULL                                        /* No commands */
                                                };
    paRouteInfo_t               nFailInfo =     {   pa_DEST_DISCARD,                            /* Toss the packet  */           
	                                                0,                                          /* Flow Id = dont care */        
                                                    0,                                          /* queue = dont care */          
                                                    0,                                          /* mutli route = dont care */    
                                                    0,                                          /* swinfo0 = dont care */        
                                                    0,                                          /* SwInfo 1 is dont care */
                                                    0,                                          /* customType = pa_CUSTOM_TYPE_NONE */         \
                                                    0,                                          /* customIndex: not used */        \
                                                    0,                                          /* pkyType: for SRIO only */       \
                                                    NULL                                        /* No commands */
                                                };
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */            
                                                    0,                                          /* User chosen ID to go to swinfo0 */     
                                                    0,                                          /* Destination queue */                   
                                                    0                                           /* Flow ID */  
                                                };
    paReturn_t                  retVal;
    paEntryHandle_t             retHandle;
    int32_t                     handleType, cmdDest;
    uint32_t                    psCmd       =   ((uint32_t)(4 << 5) << 24);  
    uint32_t                    myswinfo[]  =   {0x11112222, 0x33334444};
    Cppi_HostDesc*              pHostDesc;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
#ifndef __LINUX_USER_SPACE	
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550000;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal  =   Pa_addMac  (gPAInstHnd,
                            pa_LUT1_INDEX_NOT_SPECIFIED,
                            &ethInfo,
                            &routeInfo,
                            &nFailInfo,
                            &gPaL2Handles,
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);
    if (retVal != pa_OK)  
    {
        System_printf ("Pa_addMac returned error %d\n", retVal);
        return -1;
    }
    
    /* This sets the extended info for descriptors, and this is required so PS info
     * goes to the right spot 
     */                   
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo);

    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;
    
    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
       
    /* Send the command to the PA and wait for the return */
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    Qmss_queuePush (gPaTxQHnd[cmdDest], 
                    pHostDesc, 
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                   );
    
    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * MAC address addition.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Add_MACAddress: Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
                SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }
            

            retVal  =   Pa_forwardResult (gPAInstHnd, (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)  
            {
                System_printf ("PA sub-system rejected Pa_addMac command\n");
                return -1;
            }
        
            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_addMac command\n");
        return -1;
    }

    return 0;
}

int32_t Del_MACAddress (void)
{
    int32_t           j;
    uint16_t          cmdSize;
    paCmdReply_t      cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */            
                                          0,                                          /* User chosen ID to go to swinfo0 */     
                                          0,                                          /* Destination queue */                   
                                          0                                           /* Flow ID */  
                                      };
    paReturn_t        retVal;
    paEntryHandle_t   retHandle;
    int32_t           handleType, cmdDest;
    uint32_t          psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t          myswinfo[]  =   {0x11112222, 0x33334444};
    Cppi_HostDesc*    pHostDesc;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
#ifndef __LINUX_USER_SPACE	    
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550020;  /* unique for each delete mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal  =   Pa_delHandle (gPAInstHnd,
                            &gPaL2Handles,
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);
    if (retVal != pa_OK)  
    {
        System_printf ("Pa_addMac returned error %d\n", retVal);
        return -1;
    }
    
    /* This sets the extended info for descriptors, and this is required so PS info
     * goes to the right spot 
     */                   
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo);

    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;
    
    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
       
    /* Send the command to the PA and wait for the return */
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);	
    Qmss_queuePush (gPaTxQHnd[cmdDest], 
                    pHostDesc, 
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                   );
    
    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * MAC address addition.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);  			

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
	            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }

            retVal  =   Pa_forwardResult (gPAInstHnd, (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)  
            {
                System_printf ("PA sub-system rejected Pa_delHandle command from Del_MacAddress function \n");
                return -1;
            }
        
            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
      	    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_delHandle command from Del_MacAddress function\n");
        return -1;
    }

    return 0;
}

/** ============================================================================
 *   @n@b Add_IPAddress
 *
 *   @b Description
 *   @n This API adds the IP Address the application's using to the PA PDSP 
 *      Lookup table. This ensures that all packets destined for this 
 *      IP address get forwarded up to the host.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Add_IPAddress (void)
{
    int32_t                       j;
    uint16_t                      cmdSize;
    paIpInfo_t                  ipInfo      =    {  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   /* IP source = dont care */   
                                                    { 0xc0, 0xa8, 0x01, 0xa, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },   /* IP dest */                
                                                    0,         /* SPI = dont care */                                                
                                                    0,         /* flow = dont care */                                                   
                                                    pa_IPV4,   /* IP type */                                                            
                                                    0,         /* GRE protocol */                                                       
                                                    0,         /* Ip protocol = dont care (TCP or UDP or anything else) */              
                                                    0,         /* TOS */                                                                
                                                    FALSE,     /* TOS = dont care (seperate field since TOS=0 is valid */
                                                    0          /* SCTP destination port = dont care */  
                                                };
    //int32_t                       macLink     =   0;  /* Link this with the first MAC address created */
    paRouteInfo_t               routeInfo   =   {   pa_DEST_CONTINUE_PARSE_LUT2,                /* Continue parsing */             
                                                    0,                                          /* Flow Id = dont care */          
                                                    0,                                          /* queue = dont care */            
                                                    0,                                          /* multi route = dont care */      
                                                    0,                                          /* swinfo0 = dont care */          
                                                    0,                                          /* SwInfo 1 is dont care */
                                                    0,                                          /* customType = pa_CUSTOM_TYPE_NONE */         \
                                                    0,                                          /* customIndex: not used */        \
                                                    0,                                          /* pkyType: for SRIO only */       \
                                                    NULL                                        /* No commands */
                                                };
    paRouteInfo_t               nFailInfo   =   {   pa_DEST_DISCARD,                            /* Toss the packet  */           
	                                                0,                                          /* Flow Id = dont care */        
                                                    0,                                          /* queue = dont care */          
                                                    0,                                          /* mutli route = dont care */    
                                                    0,                                          /* swinfo0 = dont care */        
                                                    0,                                          /* SwInfo 1 is dont care */
                                                    0,                                          /* customType = pa_CUSTOM_TYPE_NONE */         \
                                                    0,                                          /* customIndex: not used */        \
                                                    0,                                          /* pkyType: for SRIO only */       \
                                                    NULL                                        /* No commands */
                                                };
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */            
                                                    0,                                          /* User chosen ID to go to swinfo0 */     
                                                    0,                                          /* Destination queue */                   
                                                    0                                           /* Flow ID */  
                                                };
    paReturn_t                  retVal;
    paEntryHandle_t             retHandle;
    int32_t                       handleType, cmdDest;
    uint32_t                      psCmd       =   ((uint32_t)(4 << 5) << 24);  
    uint32_t                      myswinfo[]  =   {0x11112222, 0x33334444};
    Cppi_HostDesc*              pHostDesc;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
#ifndef __LINUX_USER_SPACE		
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550001;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      = Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal  =   Pa_addIp    (gPAInstHnd,
                             pa_LUT_INST_NOT_SPECIFIED,
                             pa_LUT1_INDEX_NOT_SPECIFIED,
                            &ipInfo,
                            gPaL2Handles,
                            &routeInfo,
                            &nFailInfo,
                            &gPaL3Handles,
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);
    if (retVal != pa_OK)  
    {
        System_printf ("Pa_addIp returned error %d\n", retVal);
        return -1;
    }
    
    /* This sets the extended info for descriptors, and this is required so PS info
     * goes to the right spot 
     */                   
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo);

    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;
    
    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
       
    /* Send the command to the PA and wait for the return */
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    Qmss_queuePush (gPaTxQHnd[cmdDest],
                    pHostDesc, 
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                    );

    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * MAC address addition.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Add_IPAddress:Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
                SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }
            

            retVal  =   Pa_forwardResult (gPAInstHnd, (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)  
            {
                System_printf ("PA sub-system rejected Pa_addIp command\n");
                return -1;
            }
        
            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_addIP command\n");
        return -1;
    }

    return 0;
}


/** ============================================================================
 *   @n@b Del_IPAddress
 *
 *   @b Description
 *   @n This API deletes the IP Address the application's using to the PA PDSP 
 *      Lookup table. This ensures that all packets destined for this 
 *      IP address get forwarded up to the host.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Del_IPAddress (void)
{
    int32_t                       j;
    uint16_t                      cmdSize;
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */            
                                                    0,                                          /* User chosen ID to go to swinfo0 */     
                                                    0,                                          /* Destination queue */                   
                                                    0                                           /* Flow ID */  
                                                };
    paReturn_t         retVal;
    paEntryHandle_t    retHandle;
    int32_t              handleType, cmdDest;
    uint32_t             psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t             myswinfo[]  =   {0x11112222, 0x33334444};
    Cppi_HostDesc*     pHostDesc;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);    
#ifndef __LINUX_USER_SPACE		
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550010;  /* unique for each delete ip command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      = Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);
    
    retVal  =   Pa_delHandle (gPAInstHnd,
                            &gPaL3Handles,
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);

    if (retVal != pa_OK)  
    {
        System_printf ("Pa_addIp returned error %d\n", retVal);
        return -1;
    }
    
    /* This sets the extended info for descriptors, and this is required so PS info
     * goes to the right spot 
     */                   
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo);

    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;
    
    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
       
    /* Send the command to the PA and wait for the return */
	SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
	SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);
    Qmss_queuePush (gPaTxQHnd[cmdDest], 
                    pHostDesc,
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                    );

    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * MAC address addition.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);   			

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
                SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);				
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }

            retVal  =   Pa_forwardResult (gPAInstHnd, (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)  
            {
                System_printf ("PA sub-system rejected Pa_delHandle command\n");
                return -1;
            }
        
            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
  		    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_delHandle command\n");
        return -1;
    }

    return 0;
}

/** ============================================================================
 *   @n@b Add_Port
 *
 *   @b Description
 *   @n This API adds the UDP port the application's using to the PA PDSP 
 *      Lookup table. This ensures that all packets destined for this 
 *      UDP port get forwarded up to the host.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Add_Port (void)
{
    int32_t                     j;
    uint16_t                    cmdSize;
    int16_t                     ports       =   {0x5678};

    paRouteInfo_t               routeInfo   =   {   pa_DEST_HOST,           /* Route a match to the host */   
                                                    0,                      /* Flow ID 0 */                   
                                                    0,                      /* Destination queue */           
                                                    -1,                     /* Multi route disabled */        
                                                    0xaaaaaaaa,             /* SwInfo 0 */                    
                                                    0,                      /* SwInfo 1 is dont care */
                                                    0,                      /* customType = pa_CUSTOM_TYPE_NONE */         \
                                                    0,                      /* customIndex: not used */        \
                                                    0,                      /* pkyType: for SRIO only */       \
                                                    NULL                    /* No commands */
                                                };                      
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,           /* Replies go to the host */            
                                                    0,                      /* User chosen ID to go to swinfo0 */     
                                                    0,                      /* Destination queue */                   
                                                    0                       /* Flow ID */  
                                                };
    paReturn_t                  retVal;
    paEntryHandle_t             retHandle;
    int32_t                     handleType, cmdDest;
    uint32_t                    psCmd       =   ((uint32_t)(4 << 5) << 24);  
    uint32_t                    myswinfo[]  =   {0x11112220, 0x33334444};
    Cppi_HostDesc*              pHostDesc;
    paCmdInfo_t                 cmdInfo;

#ifndef __LINUX_USER_SPACE
    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM); 
#endif

    /* update UDP port as a function of the core number*/
    ports += coreNum;

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
#ifndef __LINUX_USER_SPACE		
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550002;  /* unique for each add port command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      = Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    /* Setup the Rx queue as destination for the packets */
    routeInfo.queue         = Qmss_getQIDFromHandle(gRxQHnd);
    routeInfo.flowId        = (uint8_t)Cppi_getFlowId(gRxFlowHnd);
    
    /* Issue command to increment user-defined statistics per-match */
    cmdInfo.cmd = pa_CMD_USR_STATS;
    cmdInfo.params.usrStats.index = usrStatsTbl[USR_STATS_ID_RX_PKTS].cntIndex;
    routeInfo.pCmd = &cmdInfo;

	System_printf ("Destination for the packet Rx queue: %d (Core %d) \n", gRxQHnd, coreNum);

    retVal  =   Pa_addPort  (gPAInstHnd,
                             pa_LUT2_PORT_SIZE_16,
                            ports,
                            gPaL3Handles,
                            FALSE,                      /* New Entry required */
                            pa_PARAMS_NOT_SPECIFIED,    /* No queue diversion */
                            &routeInfo,
                            gPaL4Handles,
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest);
    if (retVal != pa_OK)  
    {
        System_printf ("Pa_addPort returned error %d\n", retVal);
        return -1;
    }

    /* This sets the extended info for descriptors, and this is required so PS info
     * goes to the right spot 
     */                   
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo);

    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;

    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
   
    /* Send the command to the PA and wait for the return */
    SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_WAIT);
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    Qmss_queuePush (gPaTxQHnd[cmdDest],
                    pHostDesc, 
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                   );

    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * MAC address addition.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Add_Port: Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
                SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }

            retVal  =   Pa_forwardResult (gPAInstHnd, (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)  
            {
                System_printf ("PA sub-system rejected Pa_addPort command\n");
                return -1;
            }

    
            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_addPort command\n");
        return -1;
    }

    return 0;
}

/** ============================================================================
 *   @n@b Del_Port
 *
 *   @b Description
 *   @n This API deletes the UDP port the application's using to the PA PDSP
 *      Lookup table.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Del_Port (void)
{
    int32_t                     j;
    uint16_t                    cmdSize;
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,           /* Replies go to the host */            
                                                    0,                      /* User chosen ID to go to swinfo0 */     
                                                    0,                      /* Destination queue */                   
                                                    0                       /* Flow ID */  
                                                };
    paReturn_t       retVal;
    paEntryHandle_t  retHandle;
    int32_t          handleType, cmdDest;
    uint32_t         psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t         myswinfo[]  =   {0x11112222, 0x33334444};
    Cppi_HostDesc*   pHostDesc;
    

    /* Get a Tx free descriptor to send a command to the PA PDSP */
    if ((pHostDesc = Qmss_queuePop (gTxFreeQHnd)) == NULL)
    {
        System_printf ("Error obtaining a Tx free descriptor \n");            
        return -1;
    }

    /* The descriptor address returned from the hardware has the 
     * descriptor size appended to the address in the last 4 bits.
     *
     * To get the true descriptor pointer, always mask off the last 
     * 4 bits of the address.
     */
    pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
#ifndef __LINUX_USER_SPACE	    
    CSL_XMC_invalidatePrefetchBuffer();
#endif
    SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
    SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

    pHostDesc->buffLen      =   pHostDesc->origBufferLen;
    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x55550030;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      = Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);
    
    retVal =  Pa_delL4Handle (gPAInstHnd,
                            gPaL4Handles,
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest );
    if (retVal != pa_OK)  
    {
        System_printf ("Pa_delL4Handle returned error %d\n", retVal);
        return -1;
    }
    
    /* This sets the extended info for descriptors, and this is required so PS info
     * goes to the right spot 
     */                   
    Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)myswinfo);

    /* Set the buffer length to the size used. It will be restored when the descriptor
     * is returned 
     */
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, cmdSize);
    pHostDesc->buffLen  =   cmdSize;
    
    /* Mark the packet as a configuration packet */
    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)&psCmd, 4);
       
    /* Send the command to the PA and wait for the return */
    SYS_CACHE_WB ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_WAIT);
    SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);

    Qmss_queuePush (gPaTxQHnd[cmdDest], 
                    pHostDesc,
                    pHostDesc->buffLen, 
                    SIZE_HOST_DESC, 
                    Qmss_Location_TAIL
                   );

    /* Poll on the PA response queue to see if response from PA has come */
    for (j = 0; j < 100; j++)  
    {
        CycleDelay (1000);

        if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   
        {
            /* We have a response from PA PDSP for the command we submitted earlier for 
             * MAC address addition.
             */
            pHostDesc = Qmss_queuePop (gPaCfgCmdRespQHnd);

            /* Clear the size bytes */
            pHostDesc = (Ptr) ((uint32_t) pHostDesc & 0xFFFFFFF0);
            SYS_CACHE_INV (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            SYS_CACHE_INV ((Ptr)pHostDesc->buffPtr, pHostDesc->buffLen, CACHE_FENCE_WAIT);                  

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)  
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n", 
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
	            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
                Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

                return -1;
            }

            retVal  =   Pa_forwardResult (gPAInstHnd, (Ptr)pHostDesc->buffPtr, &retHandle, &handleType, &cmdDest);
            if (retVal != pa_OK)  
            {
                System_printf ("PA sub-system rejected Pa_addPort command\n");
                return -1;
            }
        
            /* Reset the buffer lenght and put the descriptor back on the Tx free queue */
            pHostDesc->buffLen = pHostDesc->origBufferLen;
            SYS_CACHE_WB (pHostDesc, SIZE_HOST_DESC, CACHE_FENCE_WAIT);
            Qmss_queuePush (gRxFreeQHnd, pHostDesc, pHostDesc->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

            break;
        }
    }

    if (j == 100)  
    {
        System_printf ("Timeout waiting for reply from PA to Pa_addMac command\n");
        return -1;
    }

    return 0;
}
/** ============================================================================
 *   @n@b Init_PASS
 *
 *   @b Description
 *   @n This API initializes the PASS/PDSP and opens a queue that the application
 *      can use to receive command responses from the PASS on the master core
 *      or process.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Init_PASS (void)
{
	uint8_t				 isAllocated;				
    paSizeInfo_t         paSize;
    paConfig_t           paCfg;
    int32_t              retVal, bufSize;    
    int32_t              sizes[pa_N_BUFS];
    int32_t              aligns[pa_N_BUFS];
    void*                bases[pa_N_BUFS];
	uint8_t*             gPAInst;
	uint8_t*             gMemL2Ram;
	uint8_t*             gMemL3Ram;
	uint8_t*             gMemUsrStats;
	pa_Example_shmStr_e  shmStr;
    
    memset(&paSize, 0, sizeof(paSizeInfo_t));
    memset(&paCfg, 0, sizeof(paConfig_t));
    memset(sizes, 0, sizeof(sizes));
    memset(aligns, 0, sizeof(aligns));
    memset(bases, 0, sizeof(bases));
  
    /* Allocate space for the PA LLD buffers. The buffers we need to 
     * allocate space are:
     *      (1) PA LLD Instance Info Handle
     *      (2) PA LLD L2 Handle database
     *      (3) PA LLD L3 Handle database
     *      (4) PA LLD Usr Stats database
     */
    paSize.nMaxL2 = MAX_NUM_L2_HANDLES;
    paSize.nMaxL3 = MAX_NUM_L3_HANDLES;
    paSize.nUsrStats = pa_USR_STATS_MAX_COUNTERS;
    if ((retVal = Pa_getBufferReq(&paSize, sizes, aligns)) != pa_OK)
    {
        System_printf ("Pa_getBufferReq returned error %d\n", retVal);
        return -1;
    }

    /* Get the PA Buffers from Shared Memory */
    shmStr = gPaInstBufAddr;
    gPAInst = (uint8_t*) fw_shmGetEntry(shmStr);

    /* Validate the buffer allocations */
    /* The first buffer is always the instance buffer */
    if ((uint32_t)gPAInst & (aligns[0] - 1))  
    {
        System_printf ("Pa_getBufferReq requires %d alignment for instance buffer, but address is 0x%08x\n", aligns[0], (uint32_t)gPAInst);
        return -1;
    }
    
    bufSize = (int32_t) fw_shmGetEntry(gPaInstBufSize);

    if (bufSize < sizes[0])
    {
        System_printf ("Pa_getBufferReq requires %d bytes for instance buffer, have only %d\n", sizes[0], bufSize);
        return -1;
    }

    bases[0]    =   (void *)gPAInst;

    /* The second buffer is the L2 table */
    shmStr = gMemL2RamBufAddr;
    gMemL2Ram = (uint8_t*) fw_shmGetEntry(shmStr);

    if ((uint32_t)gMemL2Ram & (aligns[1] - 1))
    {
        System_printf ("Pa_getBufferReq requires %d alignment for buffer 1, but address is 0x%08x\n", aligns[1], (uint32_t)gMemL2Ram);
        return (-1);
    }
    
    bufSize = (int32_t) fw_shmGetEntry(gMemL2RamBufSize);
    

    if (bufSize < sizes[1])
    {
        System_printf ("Pa_getBufferReq requires %d bytes for buffer 1, have only %d\n", sizes[1], bufSize);
        return -1;
    }

    bases[1]    =   (void *)gMemL2Ram;

    /* The third buffer is the L3 table */
    shmStr = gMemL3RamBufAddr;
    gMemL3Ram = (uint8_t*) fw_shmGetEntry(shmStr);
    if ((uint32_t)gMemL3Ram & (aligns[2] - 1))  
    {
        System_printf ("Pa_alloc requires %d alignment for buffer 2, but address is 0x%08x\n", aligns[2], (uint32_t)gMemL3Ram);
        return (-1);
    }
    
    bufSize = (int32_t) fw_shmGetEntry(gMemL3RamBufSize);
    

    if (bufSize < sizes[2])
    {
        System_printf ("Pa_alloc requires %d bytes for buffer 2, have only %d\n", sizes[2], bufSize);
        return (-1);
    }

    bases[2]    =   (void *)gMemL3Ram;
    
    /* The fourth buffer is the User stats link table */
    shmStr = gMemUsrStatsBufAddr;
    gMemUsrStats = (uint8_t*) fw_shmGetEntry(shmStr);
    if ((uint32_t)gMemUsrStats & (aligns[3] - 1))  
    {
        System_printf ("Pa_alloc requires %d alignment for buffer 3, but address is 0x%08x\n", aligns[3], (uint32_t)gMemUsrStats);
        return (-1);
    }
    
    bufSize = (int32_t) fw_shmGetEntry(gMemUsrStatsBufSize);

    if (bufSize < sizes[3])
    {
        System_printf ("Pa_alloc requires %d bytes for buffer 3, have only %d\n", sizes[3], bufSize);
        return (-1);
    }

    bases[3]    =   (void *)gMemUsrStats;

    /* Finally initialize the PA LLD */
    paCfg.initTable =   TRUE;
#ifndef __LINUX_USER_SPACE		
    paCfg.baseAddr = CSL_NETCP_CFG_REGS;
    paCfg.initDefaultRoute = TRUE;
#else
    paCfg.baseAddr = (uint32_t) fw_passCfgVaddr;
#endif
    paCfg.sizeCfg   =   &paSize;
    paCfg.instPoolBaseAddr = (void*) shObj;
#if RM
    paCfg.rmServiceHandle = rmClientServiceHandle;
#endif	
   
    if ((retVal = Pa_create (&paCfg, bases, &gPAInstHnd)) != pa_OK)  
    {
        System_printf ("Pa_create returned with error code %d\n", retVal);
        return -1;
    }
    
    SYS_CACHE_WB( (void *)gPAInst, BUFSIZE_PA_INST, CACHE_WAIT);
#ifndef __LINUX_USER_SPACE	
    /* Download the PASS PDSP firmware only if no boot mode is set */
    if (no_bootMode == TRUE)
    {
		if (Download_PAFirmware ())
        {
           return -1;
        }
    }
#endif
    /* Open a PA Command Response Queue.
     *
     * This queue will be used to hold responses from the PA PDSP for all the
     * commands issued by the example application.
     *
     * This queue is used only at configuration time to setup the PA PDSP.
     */
    if ((gPaCfgCmdRespQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error opening a PA Command Response queue \n");
        return -1;
    }          
    
    /* Init done. Return success. */
    return 0;
}


/** ============================================================================
 *   @n@b Init_Pa_Local
 *
 *   @b Description
 *   @n This API starts the PASS/PDSP and opens a queue that the application
 *      can use to receive command responses from the PASS on the local core
 *      or process.
 *
 *   @param[in]
 *   @n None
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Init_Pa_Local (void)
{
	uint8_t						isAllocated;
	paStartCfg_t				paStartCfg;	
	memset(&paStartCfg, 0, sizeof(paStartCfg_t));

	gPAInstHnd   = (Pa_Handle)     fw_shmGetEntry(gPAInstHndAddr);
    gPaL2Handles = (paHandleL2L3_t)fw_shmGetEntry(gPaL2HandlesAddr);
    gPaL3Handles = (paHandleL2L3_t)fw_shmGetEntry(gPaL3HandlesAddr);	

#ifndef __LINUX_USER_SPACE		
      paStartCfg.baseAddr = CSL_NETCP_CFG_REGS;
#else
      paStartCfg.baseAddr = (uint32_t) fw_passCfgVaddr;
#endif	  

#if RM
	  paStartCfg.rmServiceHandle  = rmClientServiceHandle;
#endif
	  paStartCfg.instPoolBaseAddr = shObj;
	  Pa_startCfg(gPAInstHnd, &paStartCfg);

   /* Open a PA Command Response Queue.
	*
	* This queue will be used to hold responses from the PA PDSP for all the
	* commands issued by the example application.
	*
	* This queue is used only at configuration time to setup the PA PDSP.
	*/
   if ((gPaCfgCmdRespQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
   {
	   System_printf ("Error opening a PA Command Response queue \n");
	   return -1;
   } 
   
   return 0;

}

/** ============================================================================
 *   @n@b Clear_PASS
 *
 *   @b Description
 *   @n This API display PA user-defined statistics abd release all PA resources.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Clear_PASS (int portNum)
{
    /*
     * Request, display and reset the user-defined statistics
     *
     */
    if (Get_UsrStats (portNum, NUM_USR_STATS, usrStatsTbl) < 0)
    {
        System_printf ("Get_UsrStats API failed\n");
        return -1;
    }

    /* 
     * Reset User-defined statistics Link Table
     */
    if (Reset_UsrStats(NUM_USR_STATS, usrStatsGroup) < 0)
    {
        System_printf ("Reset_UsrStats API failed\n");
        return -1;
    }          
    
    /* Free User-defined statistics */
    if (Free_UsrStats (NUM_USR_STATS, usrStatsTbl) < 0)
    {
        System_printf ("Free_UsrStats API failed\n");
        return -1;
    }          

   return 0;
}

/** ============================================================================
 *   @n@b Setup_PASS
 *
 *   @b Description
 *   @n This API sets up the PA LLD/PDSP with MAC/IP/UDP configuration used by
 *      the example application.
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int32_t Setup_PASS (void)
{
#ifndef __LINUX_USER_SPACE
    /* Get the core number. */
    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM); 
#endif
    /* All cores use same MAC and IP. 
       Let core 0 setup MAC and IP. All cores set 
       their own UDP port */
    
    if(coreNum == SYSINIT)
    {   
    
        /* 
        * Call Global Configuration for user-defined statistics global configuration 
        */
        if (Glob_Config(&paGlobalCfg) < 0)
        {
            System_printf ("Glob_Config API failed\n");
            return -1;
        }          
     
        /* Setup the PA PDSP to forward packets matching our switch MAC 
         * address up to the host onto the example application.
         */
        if (Add_MACAddress () != 0)
        {
			System_printf ("Add_MAC failed for corenum:%d\n", coreNum);
            return -1;
        }
		else {
			System_printf ("Add_MAC successful for corenum:%d\n", coreNum);
		}

        /* Add the IP address the example uses */
        if (Add_IPAddress () != 0)
        {
			System_printf ("Add_IP failed for corenum:%d\n", coreNum);
            return -1;
        }
		else {
			System_printf ("Add_IP successful for corenum:%d\n", coreNum);
		}
		
    }
    
    /* 
     * Allocate and configure User-defined statistics
     */
    if (Alloc_UsrStats(NUM_USR_STATS, usrStatsTbl, usrStatsGroup) < 0)
    {
        System_printf ("Alloc_UsrStats API failed\n");
        return -1;
    }          
    
    if (Config_UsrStats(NUM_USR_STATS, usrStatsGroup) < 0)
    {
        System_printf ("Alloc_UsrStats API failed\n");
        return -1;
    }          
    
    /* Add the port number on which our application is going to listen on */
    if (Add_Port () != 0)
    {
		System_printf ("Add_Port failed for corenum:%d\n", coreNum);
        return -1;
    }
	else {
		System_printf ("Add_Port successful for corenum:%d\n", coreNum);
	}	

    /* Return success */
    return 0;
}

void mdebugHaltPdsp (Int pdspNum)
{
#ifndef NSS_GEN2

    CSL_Pa_ssRegs *passRegs;
#ifdef __LINUX_USER_SPACE
    passRegs = (CSL_Pa_ssRegs *)fw_passCfgVaddr;
#else
    passRegs = (CSL_Pa_ssRegs *)CSL_NETCP_CFG_REGS;
#endif
	passRegs->PDSP_CTLSTAT[pdspNum].PDSP_CONTROL &= ~(CSL_PA_SS_PDSP_CONTROL_PDSP_ENABLE_MASK);
#endif    

}
