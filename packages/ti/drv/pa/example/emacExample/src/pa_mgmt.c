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
#include <cpsw_singlecore.h>

/* PA LLD include */
#include <ti/drv/pa/pa.h>

/* PASS RL file */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_pa_ss.h>
#include <ti/csl/csl_pscAux.h>

#ifdef __LINUX_USER_SPACE
#include "armv7/linux/fw_test.h"
extern void System_flush(void);
#endif

/* PA command response queue handle */
Qmss_QueueHnd                           gPaCfgCmdRespQHnd;

/* Number of PA internal buffers to allocate */
#define     PA_NUM_BUFFERS              3

/* PA definitions */
#define     MAX_NUM_L2_HANDLES          10
#define     MAX_NUM_L3_HANDLES          20
#define     MAX_NUM_L4_HANDLES          40

#define     BUFSIZE_PA_INST             512
#define     BUFSIZE_L2_TABLE            1000
#define     BUFSIZE_L3_TABLE            4000

/* PA instance */
#ifdef __LINUX_USER_SPACE
uint8_t gPAInst[BUFSIZE_PA_INST]ALIGN(CACHE_LINESZ);
uint8_t gMemL2Ram[BUFSIZE_L2_TABLE]ALIGN(CACHE_LINESZ);
uint8_t gMemL3Ram[BUFSIZE_L3_TABLE]ALIGN(CACHE_LINESZ);
paHandleL2L3_t                          gPaL2Handles[MAX_NUM_L2_HANDLES] ALIGN(CACHE_LINESZ);
paHandleL2L3_t                          gPaL3Handles[MAX_NUM_L3_HANDLES] ALIGN(CACHE_LINESZ);
paHandleL4_t                            gPaL4Handles[MAX_NUM_L4_HANDLES] ALIGN(CACHE_LINESZ);
#else
#ifdef _TMS320C6X
#pragma DATA_ALIGN(gPAInst, CACHE_LINESZ)
uint8_t                                   gPAInst[SYS_ROUND_UP(BUFSIZE_PA_INST, CACHE_LINESZ)];

/* Memory used for PA handles */
#pragma DATA_ALIGN(gMemL2Ram, CACHE_LINESZ)
uint8_t                                   gMemL2Ram[SYS_ROUND_UP(BUFSIZE_L2_TABLE, CACHE_LINESZ)];

#pragma DATA_ALIGN(gMemL3Ram, CACHE_LINESZ)
uint8_t                                   gMemL3Ram[SYS_ROUND_UP(BUFSIZE_L3_TABLE, CACHE_LINESZ)];
#else
uint8_t gPAInst[SYS_ROUND_UP(BUFSIZE_PA_INST, CACHE_LINESZ)] __attribute__ ((aligned (CACHE_LINESZ)));
uint8_t gMemL2Ram[SYS_ROUND_UP(BUFSIZE_L2_TABLE, CACHE_LINESZ)] __attribute__ ((aligned (CACHE_LINESZ)));
uint8_t gMemL3Ram[SYS_ROUND_UP(BUFSIZE_L3_TABLE, CACHE_LINESZ)] __attribute__ ((aligned (CACHE_LINESZ)));
#endif

paHandleL2L3_t                          gPaL2Handles[MAX_NUM_L2_HANDLES];
paHandleL2L3_t                          gPaL3Handles[MAX_NUM_L3_HANDLES];
paHandleL4_t                            gPaL4Handles[MAX_NUM_L4_HANDLES];

#endif /* __LINUX_USER_SPACE */

/* PA Driver Handle */
Pa_Handle                               gPAInstHnd;


/* pa configuration command buffer */
#ifdef __LINUX_USER_SPACE
uint8_t                                *gPaCmdBuf1 = 0;
uint8_t                                *gPaCmdBuf2 = 0;
#else
uint8_t                                 gPaCmdBuf1[pa_EMAC_PORT_CONFIG_MIN_CMD_BUF_SIZE_BYTES];
uint8_t                                 gPaCmdBuf2[pa_EMAC_PORT_CONFIG_MIN_CMD_BUF_SIZE_BYTES];
#endif

extern  Qmss_QueueHnd                   gPaTxQHnd [MAX_PA_TX_QUEUES], gTxFreeQHnd, gRxFreeQHnd, gRxQHnd;

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

static uint8_t srcMac[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

int32_t Add_MACAddress (void)
{
    int32_t                       j;
    uint16_t                      cmdSize;
    paEthInfo_t                 ethInfo     =  { { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },        /* Src mac = dont care */
                                                 { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15 },        /* Dest mac */
                                                    0,                                          /* vlan = dont care */
                                                    0x0800,                             		/* ether type = IPv4 */
                                                    0,                                          /* MPLS tag = don't care */
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
    paReturn_t        retVal;
    paEntryHandle_t   retHandle;
    int32_t             handleType, cmdDest;
    uint32_t            psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t            myswinfo[]  =   {0x11112222, 0x33334444};
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

    /* Populate the Rx free descriptor with the fixed command buffer. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x11111111;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

#ifndef __LINUX_USER_SPACE
    /* Use Source MAC as destination MAC if non-loopback */
    if(cpswLpbkMode == CPSW_LOOPBACK_NONE)
        memcpy(ethInfo.dst, srcMac, sizeof(srcMac));
#endif

    ethInfo.inport  =  pa_EMAC_PORT_NOT_SPECIFIED;                  /* Input EMAC port */
    retVal  =   Pa_addMac  (gPAInstHnd,
                            pa_LUT1_INDEX_NOT_SPECIFIED,
                            &ethInfo,
                            &routeInfo,
                            &nFailInfo,
                            &gPaL2Handles[0],
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

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
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
    int32_t                       j;
    uint16_t                      cmdSize;
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */
                                                    0,                                          /* User chosen ID to go to swinfo0 */
                                                    0,                                          /* Destination queue */
                                                    0                                           /* Flow ID */
                                                };
    paReturn_t        retVal;
    paEntryHandle_t   retHandle;
    int32_t             handleType, cmdDest;
    uint32_t            psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t            myswinfo[]  =   {0x11112222, 0x33334444};
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

    /* Populate the Rx free descriptor with the fixed command buffer. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x11111111;  /* unique for each delete mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal  =   Pa_delHandle (gPAInstHnd,
                            &gPaL2Handles[0],
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

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
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
    int32_t                       macLink     =   {0};  /* Link this with the first MAC address created */
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

    /* Populate the Rx free descriptor with the buffer we just allocated. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x11111111;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      = Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal  =   Pa_addIp    (gPAInstHnd,
                             pa_LUT_INST_NOT_SPECIFIED,
                             pa_LUT1_INDEX_NOT_SPECIFIED,
                            &ipInfo,
                            gPaL2Handles [macLink],
                            &routeInfo,
                            &nFailInfo,
                            &gPaL3Handles[0],
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
    Qmss_queuePush (gPaTxQHnd[cmdDest],
                    (uint32_t *)Convert_CoreLocal2GlobalAddr((uint32_t)pHostDesc),
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

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
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
 *   @n@b Del_IPAddress
 *
 *   @b Description
 *   @n This API deletes the IP Address the application's using to the PA PDSP
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
    int32_t            handleType, cmdDest;
    uint32_t           psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t           myswinfo[]  =   {0x11112222, 0x33334444};
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

    /* Populate the Rx free descriptor with the buffer we just allocated. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x11111111;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      = Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal  =   Pa_delHandle (gPAInstHnd,
                            &gPaL3Handles[0],
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
    Qmss_queuePush (gPaTxQHnd[cmdDest],
                    (uint32_t *)Convert_CoreLocal2GlobalAddr((uint32_t)pHostDesc),
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

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
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
    int32_t                       j;
    uint16_t                      cmdSize;
    uint16_t                      ports       =   {0x5678};

    int32_t                       ipLink      =   {0};  /* Link this with the first IP address created */
    paRouteInfo_t               routeInfo   =   {   pa_DEST_HOST,           /* Route a match to the host */
                                                    0,                      /* Flow ID 0 */
                                                    0,                      /* Destination queue */
                                                    -1,                     /* Multi route disabled */
                                                    0xaaaaaaaa,             /* SwInfo 0 */
                                                    0,                      /* SwInfo 1 is dont care */
                                                    0,                      /* customType = pa_CUSTOM_TYPE_NONE */
                                                    0,                      /* customIndex: not used */
                                                    0,                      /* pkyType: for SRIO only */
                                                    NULL                    /* No commands */
                                                };
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,           /* Replies go to the host */
                                                    0,                      /* User chosen ID to go to swinfo0 */
                                                    0,                      /* Destination queue */
                                                    0                       /* Flow ID */
                                                };
    paReturn_t       retVal;
    paEntryHandle_t  retHandle;
    int32_t            handleType, cmdDest;
    uint32_t           psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t           myswinfo[]  =   {0x11112222, 0x33334444};
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

    /* Populate the Rx free descriptor with the buffer we just allocated. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf2), pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf2), pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES);

    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x11111111;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      = Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    /* Setup the Rx queue as destination for the packets */
    routeInfo.queue         = Qmss_getQIDFromHandle(gRxQHnd);
    routeInfo.flowId        =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal  =   Pa_addPort  (gPAInstHnd,
                            pa_LUT2_PORT_SIZE_16,
                            ports,
                            gPaL3Handles [ipLink],
                            FALSE,                      /* New Entry required */
                            pa_PARAMS_NOT_SPECIFIED,
                            &routeInfo,
                            gPaL4Handles[0],
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
    Qmss_queuePush (gPaTxQHnd[cmdDest],
                    (uint32_t *)Convert_CoreLocal2GlobalAddr((uint32_t)pHostDesc),
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

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
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
 *   @n@b Del_Port
 *
 *   @b Description
 *   @n This API Deletes the UDP port the application's using to the PA PDSP
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
    int32_t                       j;
    uint16_t                      cmdSize;
    paCmdReply_t                cmdReplyInfo =  {   pa_DEST_HOST,           /* Replies go to the host */
                                                    0,                      /* User chosen ID to go to swinfo0 */
                                                    0,                      /* Destination queue */
                                                    0                       /* Flow ID */
                                                };
    paReturn_t       retVal;
    paEntryHandle_t  retHandle;
    int32_t            handleType, cmdDest;
    uint32_t           psCmd       =   ((uint32_t)(4 << 5) << 24);
    uint32_t           myswinfo[]  =   {0x11112222, 0x33334444};
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

    /* Populate the Rx free descriptor with the buffer we just allocated. */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf2), pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES);

    /* Save original buffer information */
    Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)pHostDesc, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf2), pa_ADD_LUT2_MIN_CMD_BUF_SIZE_BYTES);

    cmdSize                 =   pHostDesc->buffLen;
    cmdReplyInfo.replyId    =   0x11111111;  /* unique for each add mac command */

    /* Get the PA response queue number and populate the destination queue number
     * in the PA response configuration.
     */
    cmdReplyInfo.queue      = Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
    cmdReplyInfo.flowId     =   (uint8_t)Cppi_getFlowId(gRxFlowHnd);

    retVal =  Pa_delL4Handle (gPAInstHnd,
                            gPaL4Handles[0],
                            (paCmd_t) pHostDesc->buffPtr,
                            &cmdSize,
                            &cmdReplyInfo,
                            &cmdDest );
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
    Qmss_queuePush (gPaTxQHnd[cmdDest],
                    (uint32_t *)Convert_CoreLocal2GlobalAddr((uint32_t)pHostDesc),
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

            if (pHostDesc->softwareInfo0 != cmdReplyInfo.replyId)
            {
                System_printf ("Found an entry in PA response queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                                pHostDesc->softwareInfo0, cmdReplyInfo.replyId);
                pHostDesc->buffLen  =   pHostDesc->origBufferLen;
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
 *      can use to receive command responses from the PASS.
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
	uint8_t						isAllocated;
    paSizeInfo_t                paSize;
    paConfig_t                  paCfg;
    int32_t                     retVal;
    int32_t                     sizes[pa_N_BUFS], sz;
    int32_t                     aligns[pa_N_BUFS];
    void*                       bases[pa_N_BUFS];
    #ifndef __LINUX_USER_SPACE
    paTimestampConfig_t tsCfg;
    #endif

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
     */
    paSize.nMaxL2 = MAX_NUM_L2_HANDLES;
    paSize.nMaxL3 = MAX_NUM_L3_HANDLES;
    paSize.nUsrStats = 0;
    if ((retVal = Pa_getBufferReq(&paSize, sizes, aligns)) != pa_OK)
    {
        System_printf ("Pa_getBufferReq returned error %d\n", retVal);
        return -1;
    }

    /* Validate the buffer allocations */
    /* The first buffer is always the instance buffer */
    if ((uint32_t)gPAInst & (aligns[0] - 1))
    {
        System_printf ("Pa_getBufferReq requires %d alignment for instance buffer, but address is 0x%08x\n", aligns[0], (uint32_t)gPAInst);
        return -1;
    }

    sz = sizeof(gPAInst);
    if (sz < sizes[0])
    {
        System_printf ("Pa_getBufferReq requires %d bytes for instance buffer, have only %d\n", sizes[0], sizeof(gPAInst));
        return -1;
    }

    bases[0]    =   (void *)gPAInst;

    /* The second buffer is the L2 table */
    if ((uint32_t)gMemL2Ram & (aligns[1] - 1))
    {
        System_printf ("Pa_getBufferReq requires %d alignment for buffer 1, but address is 0x%08x\n", aligns[1], (uint32_t)gMemL2Ram);
        return (-1);
    }

    sz = sizeof(gMemL2Ram);
    if (sz < sizes[1])
    {
        System_printf ("Pa_getBufferReq requires %d bytes for buffer 1, have only %d\n", sizes[1], sizeof(gMemL2Ram));
        return -1;
    }

    bases[1]    =   (void *)gMemL2Ram;

    /* The third buffer is the L3 table */
    if ((uint32_t)gMemL3Ram & (aligns[2] - 1))
    {
        System_printf ("Pa_alloc requires %d alignment for buffer 1, but address is 0x%08x\n", aligns[2], (uint32_t)gMemL3Ram);
        return (-1);
    }

    sz = sizeof(gMemL3Ram);
    if (sz < sizes[2])
    {
        System_printf ("Pa_alloc requires %d bytes for buffer 1, have only %d\n", sizes[2], sizeof(gMemL3Ram));
        return (-1);
    }

    bases[2]    =   (void *)gMemL3Ram;

    bases[3]    =   0;

    /* Finally initialize the PA LLD */
    paCfg.initTable =   TRUE;
    paCfg.initDefaultRoute = TRUE;
#ifdef __LINUX_USER_SPACE
    paCfg.baseAddr = (uint32_t)fw_passCfgVaddr;
#else
    paCfg.baseAddr = CSL_NETCP_CFG_REGS;
#endif

#if RM
#ifdef __LINUX_USER_SPACE
    paCfg.rmServiceHandle = rmClientServiceHandle;
#else
    paCfg.rmServiceHandle = rmServiceHandle;
#endif /* __LINUX_USER_SPACE */
#endif /* RM */

    paCfg.sizeCfg   =   &paSize;
    if ((retVal = Pa_create (&paCfg, bases, &gPAInstHnd)) != pa_OK)
    {
        System_printf ("Pa_create returned with error code %d\n", retVal);
        return -1;
    }

#ifndef __LINUX_USER_SPACE
    /* Download the PASS PDSP firmware only if no boot mode is set*/
    if (no_bootMode == TRUE)
    {
		if (Download_PAFirmware ())
        {
           return -1;
        }
    }

    /* Enable Timer for timestamp */
    memset(&tsCfg, 0, sizeof(paTimestampConfig_t));
    tsCfg.enable = TRUE;
    tsCfg.factor = pa_TIMESTAMP_SCALER_FACTOR_2;

    if(Pa_configTimestamp(gPAInstHnd, &tsCfg) != pa_OK)
        return (-1);

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
    /* Setup the PA PDSP to forward packets matching our switch MAC
     * address up to the host onto the example application.
     */
    if (Add_MACAddress () != 0)
    {
        return -1;
    }

    /* Add the IP address the example uses */
    if (Add_IPAddress () != 0)
    {
        return -1;
    }

    /* Add the port number on which our application is going to listen on */
    if (Add_Port () != 0)
    {
        return -1;
    }

    /* Return success */
    return 0;
}

int32_t getPaStats (void)
{

  #ifndef NSS_GEN2
  Cppi_HostDesc *   hd;
  Qmss_Queue        q;
  paSysStats_t *    stats;
  int32_t    j;
  uint32_t myswinfo[] = { 0x11112222, 0x33334444 };
  uint32_t      psCmd = ((uint32_t)(4 << 5) << 24);  /* Command word - will be moved to common pa/sa file */
  uint16_t          csize;
  paCmdReply_t      cmdReplyInfo =  {   pa_DEST_HOST,                               /* Replies go to the host */
                                                    0,                                          /* User chosen ID to go to swinfo0 */
                                                    0,                                          /* Destination queue */
                                                    0                                           /* Flow ID */
                                                };
  int32_t           cmdDest;
  #else
  paSysStats_t      paStats;
  paSysStats_t      *stats = &paStats;
  #endif
  paReturn_t        paret;

  #ifndef NSS_GEN2
  hd     = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (gTxFreeQHnd)) & ~0xf);
  q = Qmss_getQueueNumber (gTxFreeQHnd);
  Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

  cmdReplyInfo.replyId = 0x11111111;
  cmdReplyInfo.flowId = (uint8_t)Cppi_getFlowId(gRxFlowHnd);

  /* Populate the Rx free descriptor with the fixed command buffer. */
  Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);
  csize = pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES;

  /* Save original buffer information */
  Cppi_setOriginalBufInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)Convert_CoreLocal2GlobalAddr((uint32_t)gPaCmdBuf1), pa_ADD_LUT1_MIN_CMD_BUF_SIZE_BYTES);

  /* Get the PA response queue number and populate the destination queue number
  * in the PA response configuration.
  */
  cmdReplyInfo.queue      =   Qmss_getQIDFromHandle(gPaCfgCmdRespQHnd);
  paret = Pa_requestStats (gPAInstHnd,
  			 			   FALSE,
  						   (paCmd_t) hd->buffPtr,
  						   &csize,
  						   &cmdReplyInfo,
  						   &cmdDest);
  #else

  paret = Pa_querySysStats (gPAInstHnd,
  			 			    TRUE,
                            stats);
  #endif


  if (paret != pa_OK)  {
    System_printf ("function getPaStats: call to Pa_requestStats returned error code %d\n", paret);
    return (-1);
  }

  #ifndef NSS_GEN2
  /* This sets the extended info for descriptors, and this is required so PS info
   * goes to the right spot */
  Cppi_setSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)myswinfo);

  /* Set the buffer length to the size used. It will be restored when the descriptor
   * is returned */
  Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, csize);
  hd->buffLen = csize;

  /* Mark the packet as a configuration packet */
  Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)&psCmd, 4);

  /* Send the request to the PA */
  Qmss_queuePush (gPaTxQHnd[cmdDest],  (uint32_t *)hd, csize, SIZE_HOST_DESC, Qmss_Location_TAIL);

  CycleDelay (100000);

  /* Wait for the PA to return a response */
  for (j = 0; j < 100; j++)  {
     CycleDelay (1000);

     if (Qmss_getQueueEntryCount (gPaCfgCmdRespQHnd) > 0)   {
       hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (gPaCfgCmdRespQHnd)) & ~0xf);

       if (hd->softwareInfo0 != cmdReplyInfo.replyId)  {
         System_printf ("function getPaStats: Found an entry in PA reply queue with swinfo0 = 0x%08x, expected 0x%08x\n",
                  hd->softwareInfo0, cmdReplyInfo.replyId);
         hd->buffLen = hd->origBufferLen;
      	 Qmss_queuePush (gRxFreeQHnd, hd, hd->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

         return (-1);
       }

       stats = (paSysStats_t *)Pa_formatStatsReply (gPAInstHnd, (paCmd_t)hd->buffPtr);
       if (stats == NULL)  {
       	 System_printf ("function getPaStats: Pa_formatStats returned invalid stats\n");
       	 hd->buffLen = hd->origBufferLen;
       	 Qmss_queuePush (gRxFreeQHnd, hd, hd->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);
       	 return (-1);
       }

       hd->buffLen = hd->origBufferLen;
       Qmss_queuePush (gRxFreeQHnd, hd, hd->buffLen, SIZE_HOST_DESC, Qmss_Location_TAIL);

       break;
     }
  }
  #endif

  System_printf ("--- PA STATS --- \n");
  System_printf ("C1 number of packets:           %d\n", stats->classify1.nPackets);
  System_printf ("C1 number IPv4 packets:         %d\n", stats->classify1.nIpv4Packets);
  System_printf ("C1 number IPv6 packets:         %d\n", stats->classify1.nIpv6Packets);
  System_printf ("C1 number custom packets:       %d\n", stats->classify1.nCustomPackets);
  System_printf ("C1 number SRIO packets:         %d\n", stats->classify1.nSrioPackets);
  System_printf ("C1 number llc/snap fail:        %d\n", stats->classify1.nLlcSnapFail);
  System_printf ("C1 number table matched:        %d\n", stats->classify1.nTableMatch);
  System_printf ("C1 number failed table matched: %d\n", stats->classify1.nNoTableMatch);
  System_printf ("C1 number Ingress IP frags:     %d\n", stats->classify1.nIpFrag);
  System_printf ("C1 number IP depth overflow:    %d\n", stats->classify1.nIpDepthOverflow);
  System_printf ("C1 number vlan depth overflow:  %d\n", stats->classify1.nVlanDepthOverflow);
  System_printf ("C1 number gre depth overflow:   %d\n", stats->classify1.nGreDepthOverflow);
  System_printf ("C1 number mpls packets:         %d\n", stats->classify1.nMplsPackets);
  System_printf ("C1 number of parse fail:        %d\n", stats->classify1.nParseFail);
  System_printf ("C1 number invalid IPv6 opts:    %d\n", stats->classify1.nInvalidIPv6Opt);
  System_printf ("C1 number of Egress IP frags:   %d\n", stats->classify1.nTxIpFrag);
  System_printf ("C1 number of silent discard:    %d\n", stats->classify1.nSilentDiscard);
  System_printf ("C1 number of invalid control:   %d\n", stats->classify1.nInvalidControl);
  System_printf ("C1 number of invalid states:    %d\n", stats->classify1.nInvalidState);
  System_printf ("C1 number of system fails:      %d\n\n", stats->classify1.nSystemFail);
  System_printf ("C2 number of packets:           %d\n", stats->classify2.nPackets);
  System_printf ("C2 number of UDP packets:       %d\n", stats->classify2.nUdp);
  System_printf ("C2 number of TCP packets:       %d\n", stats->classify2.nTcp);
  System_printf ("C2 number of custom packets:    %d\n", stats->classify2.nCustom);
  System_printf ("C2 number of silent discard:    %d\n", stats->classify2.nSilentDiscard);
  System_printf ("C2 number of invalid control:   %d\n\n", stats->classify2.nInvalidControl);

  System_printf ("Modify number of command file:  %d\n\n", stats->modify.nCommandFail);
  System_flush();

  return (0);
}

volatile int pdsp_halt = 0;
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
