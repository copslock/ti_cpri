/*
 * Copyright (c) 2012-2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== rm_dsp_client_test.c ========
 *
 *  Works with the dspClientTest DSP application over the rpmsg-proto socket.
 */

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>

/* IPC Headers */
#include <ti/ipc/Std.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/transports/TransportRpmsg.h>

/* Socket Includes */
#include "sockutils.h"

/* RM includes */
#include <ti/drv/rm/rm_server_if.h>
#include <ti/drv/rm/rm_transport.h>

/* App defines:  Must match on remote proc side: */
#define HEAPID                0u
#define MAX_MESSAGEQ_NAME_LEN 32u
#define CLIENT_MESSAGEQ_NAME  "RM_CLIENT"
#define SERVER_MESSAGEQ_NAME  "RM_SERVER"

#define PROC_ID_DEFAULT      1     /* Host is zero, remote cores start at 1 */

/* IPC MessageQ RM packet encapsulation structure */
typedef struct {
    /* IPC MessageQ header (must be first element in structure) */
    MessageQ_MsgHeader msgQHeader;
    /* Pointer to RM packet */
    Rm_Packet          rmPkt;
} MsgQ_RmPacket;

Int rmServerExchange_execute(UInt16 procId)
{
    int32_t             status = 0;
    int                 err;
    MessageQ_Msg        msg = NULL;
    MessageQ_Params     msgParams;
    MessageQ_QueueId    queueId = MessageQ_INVALIDMESSAGEQ;
    MessageQ_Handle     msgqHandle;
    char                remoteQueueName[64];
    MessageQ_Msg        rmMsg;
    Rm_Packet          *rmPkt;
    int                 rm_pkt_len;
    sock_h              sock_to_server;
    sock_name_t         local_sock_name;
    sock_name_t         server_sock;
    sock_name_t         server_sock_addr;
    struct sockaddr_un  server_addr;
    char                client_ex_sock_name[] = "/var/run/rm/rm_dsp_client_exchange";
    char                server_sock_name[] = RM_SERVER_SOCKET_NAME;

    printf("Entered rmServerExchange_execute\n");
 
    /* Create the local Message Queue for receiving from DSP Client. */
    MessageQ_Params_init(&msgParams);
    msgqHandle = MessageQ_create(SERVER_MESSAGEQ_NAME, &msgParams);
    if (msgqHandle == NULL) {
        printf("Error in MessageQ_create\n");
        goto exit;
    }
    else {
        printf("Local MessageQId: 0x%x\n", MessageQ_getQueueId(msgqHandle));
    }

    snprintf(remoteQueueName, MAX_MESSAGEQ_NAME_LEN, "%s_%s", CLIENT_MESSAGEQ_NAME,
             MultiProc_getName(procId));

    /* Poll until remote side has it's messageQ created before we send: */
    do {
        status = MessageQ_open(remoteQueueName, &queueId);
        sleep (1);
    } while (status == MessageQ_E_NOTFOUND);

    if (status < 0) {
        printf("Error in MessageQ_open [%d]\n", status);
        goto cleanup;
    }
    else {
        printf("Remote queueId  [0x%x]\n", queueId);
    }

    msg = MessageQ_alloc(HEAPID, sizeof(MessageQ_MsgHeader));
    if (msg == NULL) {
        printf("Error in MessageQ_alloc\n");
        MessageQ_close(&queueId);
        goto cleanup;
    }

    /* handshake with DSP client so that it knows Linux's receive Q */
    MessageQ_setReplyQueue(msgqHandle, msg);
    MessageQ_put(queueId, msg);
    MessageQ_get(msgqHandle, &msg, MessageQ_FOREVER);
    MessageQ_free(msg);

    printf("Setting up socket connection with RM Server\n");

    /* open local sock for communication to RM Server */
    local_sock_name.type = sock_name_e;
    local_sock_name.s.name = client_ex_sock_name;
    sock_to_server = sock_open(&local_sock_name, 0, 0);
    if (!sock_to_server) {
        printf("Local socket to RM Server open failed\n");
        return(-1);
    }
    /* RM Server sock */
    server_sock.type = sock_name_e;
    server_sock.s.name = server_sock_name;

    printf("Waiting for RM messages from DSP Client\n");

    while(1) {
        status = MessageQ_get(msgqHandle, &rmMsg, MessageQ_FOREVER);
        if (status < 0) {
            printf("Error in MessageQ_get [%d]\n", status);
            break;
        }

        rmPkt = &(((MsgQ_RmPacket *)rmMsg)->rmPkt);
        printf("Received RM pkt of size %d from DSP client\n",
               rmPkt->pktLenBytes);

        /* Send received data to RM Server */
        if (sock_send(sock_to_server, (char *)rmPkt, sizeof(*rmPkt),
                      &server_sock)) {
            printf("Failed to send data to RM Server\n");
        }

        /* Wait for response from RM Server */
        rm_pkt_len = 0;
        err = sock_wait(sock_to_server, &rm_pkt_len, NULL, -1);
        if (err == -2) {
            /* Timeout */
            printf("Error socket timeout\n");
            return(-1);
        }
        else if (err < 0) {
            printf("Error in reading from socket, error %d\n", err);
            return(-1);
        }

        server_sock_addr.type = sock_addr_e;
        server_sock_addr.s.addr = &server_addr;
        err = sock_recv(sock_to_server, (char *)rmPkt, rm_pkt_len,
                        &server_sock_addr);
        if (err != rm_pkt_len) {
            printf("recv RM pkt failed from socket, "
                   "received = %d, expected = %d\n",
                   err, rm_pkt_len);
            return(-1);
        }

        /* send back to DSP */
        status = MessageQ_put(queueId, rmMsg);
        if (status < 0) {
             printf("Error in MessageQ_put [%d]\n", status);
             break;
        }
    }

cleanup:
    /* Clean-up */
    status = MessageQ_delete(&msgqHandle);
    if (status < 0) {
        printf("Error in MessageQ_delete [%d]\n", status);
    }

exit:
    printf("Leaving rmServerExchange_execute\n\n");

    return(0);
}

int main(int argc, char ** argv)
{
    Int32 status = 0;
    UInt16 procId = PROC_ID_DEFAULT;

    /* Parse Args: */
    switch (argc) {
        case 1:
            /* use defaults */
            break;
        case 2:
            procId = atoi(argv[2]);
            break;
        default:
            printf("Usage: %s [<ProcId>]\n", argv[0]);
            printf("\tDefaults: ProcId: %d\n", PROC_ID_DEFAULT);
            exit(0);
    }

    Ipc_transportConfig(&TransportRpmsg_Factory);
    status = Ipc_start();
    if (status < 0) {
        printf("Ipc_start failed with error %d\n", status);
        return(0);
    }

    if (procId >= MultiProc_getNumProcessors()) {
        printf("ProcId must be less than %d\n", MultiProc_getNumProcessors());
        Ipc_stop();
        exit(0);
    }
    printf("Using procId : %d\n", procId);

    rmServerExchange_execute(procId);
    Ipc_stop();

    return(0);
}
