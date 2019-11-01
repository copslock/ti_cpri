/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 */

/**
 *  \file ipc_testsetup.c
 *
 *  \brief IPC  example code
 *  
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/drv/ipc/ipc.h>
#include <ti/drv/ipc/ipcver.h>
#include <ti/drv/ipc/examples/common/src/ipc_setup.h>

#ifndef BUILD_MPU1_0
#if defined(SOC_AM65XX)
#include <ti/drv/ipc/examples/common/src/ipc_am65xx_rsctable.h>
#else
#include <ti/drv/ipc/examples/common/src/ipc_rsctable.h>
#endif
#endif

#define MSGSIZE  256U
#define SERVICE  "ti.ipc4.ping-pong"
#define ENDPT1   13U
#define NUMMSGS  10000 /* number of message sent per task */
//#define NUMMSGS  1000000   /* number of message sent per task */

extern uint8_t  *pCntrlBuf;
extern uint8_t  *pTaskBuf;
extern uint8_t  *pSendTaskBuf;
extern uint8_t  *pRecvTaskBuf;
extern uint8_t  *pSysVqBuf;

extern uint32_t  selfProcId;
extern uint32_t *pRemoteProcArray;
extern uint32_t  gNumRemoteProc;

uint32_t rpmsgDataSize = RPMSG_DATA_SIZE;

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
#if !defined(BUILD_MPU1_0) && defined(A72_LINUX_OS) && defined(A72_LINUX_OS_IPC_ATTACH)
static uint32_t		RecvEndPt = 0;
#endif

//#define DEBUG_PRINT

bool g_exitRespTsk = 0;

void rpmsg_exit_responseTask()
{
    g_exitRespTsk = 1;
}

/*
 * This "Task" waits for a "ping" message from any processor
 * then replies with a "pong" message.
 */
void rpmsg_responderFxn(UArg arg0, UArg arg1)
{
    RPMessage_Handle    handle;
    RPMessage_Params    params;
    uint32_t		myEndPt = 0;
    uint32_t		remoteEndPt;
    uint32_t		remoteProcId;
    uint16_t		len;
    int32_t		n;
    int32_t		status = 0;
    void		*buf;

    uint32_t            bufSize = rpmsgDataSize;
    char                str[MSGSIZE];

    buf = pRecvTaskBuf; 
    if(buf == NULL) 
    {
        System_printf("RecvTask: buffer allocation failed\n");
        return;
    }

    RPMessageParams_init(&params);
    params.requestedEndpt = ENDPT1;
    params.buf = buf;
    params.bufSize = bufSize;

    handle = RPMessage_create(&params, &myEndPt);
    if(!handle) 
    {
        System_printf("RecvTask: Failed to create endpoint\n");
        return;
    }

#if !defined(BUILD_MPU1_0) && defined(A72_LINUX_OS) && defined(A72_LINUX_OS_IPC_ATTACH)
    RecvEndPt = myEndPt;
#endif

    status = RPMessage_announce(RPMESSAGE_ALL, myEndPt, SERVICE);
    if(status != IPC_SOK) 
    {
        System_printf("RecvTask: RPMessage_announce() failed\n");
        return;
    }

    while(!g_exitRespTsk)
    {
        status = RPMessage_recv(handle, (Ptr)str, &len, &remoteEndPt, &remoteProcId,
                IPC_RPMESSAGE_TIMEOUT_FOREVER);
        if(status != IPC_SOK) 
        {
            System_printf("RecvTask: failed with code %d\n", status);
        }
        else
        {
            /* NULL terminated string */
            str[len] = '\0';
#ifdef DEBUG_PRINT
            System_printf("RecvTask: Revcvd msg \"%s\" len %d from %s\n",
                    str, len, Ipc_mpGetName(remoteProcId));
#endif
        }

        status = sscanf(str, "ping %d", &n);
        if(status == 1) 
        {
            memset(str, 0, MSGSIZE);
            len = snprintf(str, 255, "pong %d", n);
            if(len > 255)
            { 
                System_printf("RecvTask: snprintf failed, len %d\n", len);
                len = 255;
            }
            str[len++] = '\0';
        }
        else
        {
            /* If this is not ping/pong message, just print the message */
            System_printf("%s <--> %s : %s recvd\n",
                    Ipc_mpGetSelfName(),
                    Ipc_mpGetName(remoteProcId),
                    str);
        }
#ifdef DEBUG_PRINT
        System_printf("RecvTask: Sending msg \"%s\" len %d from %s to %s\n",
                str, len, Ipc_mpGetSelfName(),
                Ipc_mpGetName(remoteProcId));
#endif
        status = RPMessage_send(handle, remoteProcId, remoteEndPt, myEndPt, str, len);
        if (status != IPC_SOK) 
        {
            System_printf("RecvTask: Sending msg \"%s\" len %d from %s to %s failed!!!\n",
                str, len, Ipc_mpGetSelfName(),
                Ipc_mpGetName(remoteProcId));
        }
    }

    System_printf("%s responder task exiting ...\n",
                    Ipc_mpGetSelfName());
}

void rpmsg_senderFxn(UArg arg0, UArg arg1)
{
    RPMessage_Handle    handle;
    RPMessage_Params    params;
    uint32_t            myEndPt = 0;
    uint32_t            remoteEndPt;
    uint32_t            remoteProcId;
    uint16_t            dstProc;
    uint16_t            len;
    int32_t             i;
    int32_t             status = 0;
    char                buf[256];   
    uint8_t            *buf1;

    uint32_t            cntPing = 0;
    uint32_t            cntPong = 0;

    buf1 = &pSendTaskBuf[rpmsgDataSize * arg1];
    dstProc = arg0;

    /* Create the endpoint for receiving. */
    RPMessageParams_init(&params);
    params.numBufs = 2;
    params.buf = buf1;
    params.bufSize = rpmsgDataSize;
    handle = RPMessage_create(&params, &myEndPt);
    if(!handle) 
    {
        System_printf("SendTas %d: Failed to create message endpoint\n",
                dstProc);
        return;
    }

    status = RPMessage_getRemoteEndPt(dstProc, SERVICE, &remoteProcId,
            &remoteEndPt, BIOS_WAIT_FOREVER);
    if(dstProc != remoteProcId) 
    {
        System_printf("SendTask%d: RPMessage_getRemoteEndPt() malfunctioned, status %d\n",
                dstProc, status);
        return;
    }

    for (i = 0; i < NUMMSGS; i++) 
    {
        /* Send data to remote endPt: */
        memset(buf, 0, 256);
        len = snprintf(buf, 255, "ping %d", i);
        if(len > 255)
        {
            System_printf("SendTask%d: snprintf failed, len %d\n", dstProc, len);
            len = 255;
        }
        buf[len++] = '\0';

#ifdef DEBUG_PRINT
        System_printf("SendTask%d: Sending \"%s\" from %s to %s...\n", dstProc,
                buf, Ipc_mpGetSelfName(),
                Ipc_mpGetName(dstProc));
#endif
        /* Increase the Ping Counter */
        cntPing++;

        status = RPMessage_send(handle, dstProc, ENDPT1, myEndPt, (Ptr)buf, len);
        if (status != IPC_SOK) 
        {
            System_printf("SendTask%d: RPMessage_send Failed Msg-> \"%s\" from %s to %s...\n", 
                dstProc,
                buf, Ipc_mpGetSelfName(),
                Ipc_mpGetName(dstProc));
            break;
        }
        

        /* wait a for a response message: */
        status = RPMessage_recv(handle, (Ptr)buf, &len, &remoteEndPt,
                &remoteProcId, IPC_RPMESSAGE_TIMEOUT_FOREVER);

        if(status != IPC_SOK) 
        {
            System_printf("SendTask%d: RPMessage_recv failed with code %d\n",
                    dstProc, status);
            break;
        }

        /* Make it NULL terminated string */
        if(len >= MSGSIZE)
        {
            buf[MSGSIZE-1] = '\0';
        }
        else
        {
            buf[len] = '\0';
        }
#ifdef DEBUG_PRINT
        System_printf("SendTask%d: Received \"%s\" len %d from %s endPt %d \n",
                dstProc, buf, len, Ipc_mpGetName(remoteProcId),
                remoteEndPt);
#endif
        cntPong++;
        if((i+1)%50 == 0)
        {
            //System_printf("%s <--> %s, ping/pong iteration %d ...\n",
            //        Ipc_mpGetSelfName(), Ipc_mpGetName(dstProc), i);
        }
    }

    System_printf("%s <--> %s, Ping- %d, pong - %d completed\n",
            Ipc_mpGetSelfName(),
            Ipc_mpGetName(dstProc),
            cntPing, cntPong);

    /* Delete the RPMesg object now */
    RPMessage_delete(&handle);
}

/*
 * This "Task" waits for Linux vdev ready, and late create the vrings
 *
 */
#if !defined(BUILD_MPU1_0) && defined(A72_LINUX_OS) && defined(A72_LINUX_OS_IPC_ATTACH)
void rpmsg_vdevMonitorFxn(UArg arg0, UArg arg1)
{
    int32_t status;

    /* Wait for Linux VDev ready... */
    while(!Ipc_isRemoteReady(IPC_MPU1_0))
    {
        Task_sleep(10);
    }

    /* Create the VRing now ... */
    status = Ipc_lateVirtioCreate(IPC_MPU1_0);
    if(status != IPC_SOK)
    {
        System_printf("%s: Ipc_lateVirtioCreate failed\n", __func__);
        return;
    }

    status = RPMessage_lateInit(IPC_MPU1_0);
    if(status != IPC_SOK)
    {
        System_printf("%s: RPMessage_lateInit failed\n", __func__);
        return;
    }

    status = RPMessage_announce(IPC_MPU1_0, RecvEndPt, SERVICE);
    if(status != IPC_SOK)
    {
        System_printf("rpmsg_vdevMonitorFxn: RPMessage_announce() failed\n");
    }
}
#endif /* !defined(BUILD_MPU1_0) && defined(A72_LINUX_OS) && defined(A72_LINUX_OS_IPC_ATTACH)*/


int32_t Ipc_echo_test(void)
{
    uint32_t          t; 
    Task_Params       params;
    uint32_t          numProc = gNumRemoteProc;
    Ipc_VirtIoParams  vqParam;
    uint32_t          index = 0;

    /* Step1 : Initialize the multiproc */
    Ipc_mpSetConfig(selfProcId, numProc, pRemoteProcArray);

    System_printf("IPC_echo_test (core : %s) .....\r\n%s\r\n",
            Ipc_mpGetSelfName(), IPC_DRV_VERSION_STR);

    Ipc_init(NULL);

    //System_printf("Required Local memory for Virtio_Object = %d\r\n",
    //   numProc * Ipc_getVqObjMemoryRequiredPerCore());

#if !defined(BUILD_MPU1_0) && defined(A72_LINUX_OS)
    /* If A72 remote core is running Linux OS, then
     * load resource table
     */
    Ipc_loadResourceTable((void*)&ti_ipc_remoteproc_ResourceTable);

#if !defined(A72_LINUX_OS_IPC_ATTACH)
    /* Wait for Linux VDev ready... */
    for(t = 0; t < numProc; t++)
    {
        while(!Ipc_isRemoteReady(pRemoteProcArray[t]))
        {
            // Task_sleep(100);
        }
    }
    //System_printf("Linux VDEV ready now .....\n");
#endif
#endif

    /* Step2 : Initialize Virtio */
    vqParam.vqObjBaseAddr = (void*)pSysVqBuf;
    vqParam.vqBufSize     = numProc * Ipc_getVqObjMemoryRequiredPerCore();
    vqParam.vringBaseAddr = (void*)VRING_BASE_ADDRESS;
    vqParam.vringBufSize  = IPC_VRING_BUFFER_SIZE;
    vqParam.timeoutCnt    = 100;  /* Wait for counts */
    Ipc_initVirtIO(&vqParam);

    /* Step 3: Initialize RPMessage */
    RPMessage_Params cntrlParam;

    //System_printf("Required Local memory for RPMessage Object = %d\n",
    //   RPMessage_getObjMemRequired());

    /* Initialize the param */
    RPMessageParams_init(&cntrlParam);

    /* Set memory for HeapMemory for control task */
    cntrlParam.buf         = pCntrlBuf;
    cntrlParam.bufSize     = rpmsgDataSize;
    cntrlParam.stackBuffer = &pTaskBuf[index++ * IPC_TASK_STACKSIZE];
    cntrlParam.stackSize   = IPC_TASK_STACKSIZE;
    RPMessage_init(&cntrlParam);

    /* Respond to messages coming in to endPt ENDPT1 */
    Task_Params_init(&params);
    params.priority   = 3;
    params.stack      = &pTaskBuf[index++ * IPC_TASK_STACKSIZE];
    params.stackSize  = IPC_TASK_STACKSIZE;
    params.arg0       = 0;
    Task_create(rpmsg_responderFxn, &params, NULL);

    for(t = 0; t < numProc; t++, index++)
    {
#if !defined(BUILD_MPU1_0) && defined(A72_LINUX_OS)
        /* Linux does not have a responder func running */
        if(pRemoteProcArray[t] == IPC_MPU1_0)
            continue;
#endif
        /* send messages to peer(s) on ENDPT1 */
        Task_Params_init(&params);
        params.priority  = 3;
        params.stack     = &pTaskBuf[index * IPC_TASK_STACKSIZE];
        params.stackSize = IPC_TASK_STACKSIZE;
        params.arg0      = pRemoteProcArray[t];
        params.arg1      = t;
        Task_create(rpmsg_senderFxn, &params, NULL);

    }

#if !defined(BUILD_MPU1_0) && defined(A72_LINUX_OS) && defined(A72_LINUX_OS_IPC_ATTACH)
    /* Respond to messages coming in to endPt ENDPT1 */
    Task_Params_init(&params);
    params.priority = 3;
    params.stackSize = 0x1000;
    params.arg0 = 0;
    Task_create(rpmsg_vdevMonitorFxn, &params, NULL);
#endif /* !defined(BUILD_MPU1_0) && defined(A72_LINUX_OS) && defined(A72_LINUX_OS_IPC_ATTACH) */

    return 1;
}
