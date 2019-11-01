/** 
 *   @file  test_transport_srio.c
 *
 *   @brief  
 *      BCP transport layer implementation over SRIO. Demonstrates the use of
 *      SRIO to send/receive packets to/from BCP from a remote device.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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

#include "bcp_test.h"

/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>
#include <ti/csl/csl_srioAuxPhyLayer.h>

/* SRIO Device IDs for BCP local & remote devices */
extern const UInt32 DEVICE_ID_BCP_16BIT;
extern const UInt32 DEVICE_ID1_16BIT;

/* SRIO Device handle for this device. */
static  Srio_SockHandle     srioSocket1;
static  Srio_DrvHandle      hSrioDriver_Tx;
static  Srio_DrvHandle      hSrioDriver_Rx;

/** ============================================================================
 *   @n@b srio_tx_open
 *
 *   @b Description
 *   @n This API sets up the configuration required to send packets succesfully to
 *      BCP either from a remote device or locally. It uses the Tx configuration
 *      passed by the application to this API.
 *
 *      It sets up a SRIO Type 9 mailbox on the device to enable communication
 *      with another device.
 *
 *   @param[in]  
 *   @n hSrioCfg            SRIO driver Tx configuration (Tx endpoint configuration)
 * 
 *   @return    Void*
 *   @n NULL        -       Error setting up SRIO Tx endpoint
 *   @n >0          -       SRIO Tx endpoint info handle returned.
 * =============================================================================
 */
Void* srio_tx_open (Void*   hSrioCfg)
{
    Srio_SockBindAddrInfo   bindInfo;

    /* Start the SRIO Driver with the configuration passed */
    if ((hSrioDriver_Tx = Srio_start((Srio_DrvConfig *)hSrioCfg)) == NULL)
    {
        Bcp_osalLog ("Error: SRIO Driver Start Failed\n");
        return NULL;
    }

    /* SRIO Driver is operational at this time. */
    Bcp_osalLog ("Debug: SRIO Driver has been started Instance Handle 0x%p\n", hSrioDriver_Tx);
    
    /* Open RAW TYPE9 SRIO Socket. 
     * Setup a unique combination of Letter + Mailbox + Device Id to identify the device.
     *
     * Need only one per device for now. Setup if not already present.
     */
    if (!srioSocket1)
    {
        if ((srioSocket1 =  Srio_sockOpen (hSrioDriver_Tx, Srio_SocketType_RAW_TYPE9, FALSE)) == NULL)
        {
            Bcp_osalLog ("Error: Unable to open socket1\n");
            return NULL;
        }

        /* Populate the binding information. Pick a unique combination for the device. */
        bindInfo.type9.tt       = TRUE;              /* We are using 16 bit identifiers.        */
        bindInfo.type9.id       = DEVICE_ID1_16BIT;  /* Source Identifier bound to the socket   */
#ifdef BCP_LOCAL_DEVICE
        bindInfo.type9.streamId = 1;                 /* Stream Identifier                       */
#else
        bindInfo.type9.streamId = 0;                 /* Stream Identifier                       */
#endif
        bindInfo.type9.cos      = CSL_chipReadReg (CSL_CHIP_DNUM);           /* Class of Service                          */

        /* Bind the SRIO socket 1 */ 
        if (Srio_sockBind (srioSocket1, &bindInfo) < 0)
        {
            Bcp_osalLog ("Error: socket1 bind failed\n");
            return NULL;
        }
    }

    /* Return success */
    return srioSocket1;
}

/** ============================================================================
 *   @n@b srio_tx_close
 *
 *   @b Description
 *   @n This API cleans up and restores the SRIO Tx endpoint configuration.
 *
 *   @param[in]  
 *   @n hSrioSocket         SRIO Tx handle (Tx endpoint info handle)
 * 
 *   @return    Int32
 *   @n 0       -       Always returns 0. Success.
 * =============================================================================
 */
Int32 srio_tx_close (Void* hSrioSocket)
{
    /* Clean up the SRIO binding created earlier
	 * Since we've created only one socket. Check before cleanup */
    if (srioSocket1)
    {
        Srio_sockClose ((Srio_SockHandle) srioSocket1);
        srioSocket1 = NULL;
    }

    /* Return success */
    return 0;        
}

/** ============================================================================
 *   @n@b srio_rx_open
 *
 *   @b Description
 *   @n This API sets up the configuration required to receive packets succesfully 
 *      from BCP either remotely or locally. It uses the Rx configuration
 *      passed to this API by the application.
 *
 *   @param[in]  
 *   @n hSrioCfg            SRIO driver Rx configuration (Rx endpoint configuration)
 * 
 *   @return    Void*
 *   @n NULL        -       Error setting up SRIO Rx endpoint
 *   @n >0          -       SRIO Rx endpoint info handle returned.
 * =============================================================================
 */
Void* srio_rx_open (Void*   hSrioCfg)
{
#ifndef BCP_LOCAL_DEVICE
    Srio_SockBindAddrInfo   bindInfo;
    
    /* Start the SRIO Driver */
    hSrioDriver_Rx = Srio_start((Srio_DrvConfig *) hSrioCfg);
    if (hSrioDriver_Rx == NULL)
    {
        Bcp_osalLog ("Error: SRIO Driver Start Failed\n");
        return NULL;
    }

    /* SRIO Driver is operational at this time. */
    Bcp_osalLog ("Debug: SRIO Driver has been started Instance Handle 0x%p\n", hSrioDriver_Rx);

    /* Open RAW TYPE9 SRIO Socket. 
     * Setup a unique combination Letter + Mailbox + Device Id to identify the device.
     *
     * Need only one per device for now. Setup if not already present.
     */
    if (!srioSocket1)
    {
        srioSocket1 = Srio_sockOpen (hSrioDriver_Rx, Srio_SocketType_RAW_TYPE9, FALSE);
        if (srioSocket1 == NULL)
        {
            System_printf ("Error: Unable to open socket1\n");
            return NULL;
        }

        /* Populate the binding information. */
        bindInfo.type9.tt       = TRUE;                  /* We are using 16 bit identifiers.         */
        bindInfo.type9.id       = DEVICE_ID1_16BIT;       /* Source Identifier bound to the socket    */
#ifdef BCP_LOCAL_DEVICE
        bindInfo.type9.streamId = 1;                 /* Stream Identifier                       */
#else
        bindInfo.type9.streamId = 0;                 /* Stream Identifier                       */
#endif
        bindInfo.type9.cos      = CSL_chipReadReg (CSL_CHIP_DNUM);           /* Class of Service                          */

        /* Bind the SRIO Socket 1 */ 
        if (Srio_sockBind (srioSocket1, &bindInfo) < 0)
        {
            System_printf ("Error: socket1 bind failed\n");
            return NULL;
        }
    }

    return srioSocket1;
#else
    /* BCP output is processed on same device. So no need to setup Rx here on TurboNyquist. */        
    return srioSocket1;
#endif
}

/** ============================================================================
 *   @n@b srio_rx_close
 *
 *   @b Description
 *   @n This API cleans up and restores the SRIO Rx endpoint configuration.
 *
 *   @param[in]  
 *   @n hSrioSocket         SRIO Rx handle (Rx endpoint info handle)
 * 
 *   @return    Int32
 *   @n 0       -       Always returns 0. Success.
 * =============================================================================
 */
Int32 srio_rx_close (Void*  hSrioSocket)
{
	/* Since we've created only one socket. Check before cleanup */
    if (srioSocket1)
    {
        Srio_sockClose ((Srio_SockHandle) srioSocket1);
        srioSocket1 = NULL;
    }

    return 0;        
}

/** ============================================================================
 *   @n@b srio_send
 *
 *   @b Description
 *   @n This API is called from BCP driver to send out a packet to BCP from an
 *      application executing on remote device.
 *
 *      The API sends out the packet using SRIO to BCP device.
 *
 *   @param[in]  
 *   @n hSrioSocket         SRIO Tx handle 
 * 
 *   @param[in]  
 *   @n pPkt                Packet to be sent out to BCP
 * 
 *   @param[in]  
 *   @n pktSize             Size of the packet
 * 
 *   @param[in]  
 *   @n pDestnAddress       SRIO address of the device on which BCP exists.
 * 
 *   @return    Int32
 *   @n 0       -       Succesfully sent out the packet.
 *   @n -1      -       Error sending out the packet.
 * =============================================================================
 */
Int32 srio_send (Void*  hSrioSocket, Void*  pPkt, uint32_t pktSize, Void* pDestnAddress)
{
    /* Send out the descriptor on the RAW socket:
     *  Here the number of bytes we are pushing is the size of the descriptor since RAW sockets
     *  directly operate at descriptor levels. 
     */
    if (Srio_sockSend ((Srio_SockHandle) hSrioSocket, (Srio_DrvBuffer)pPkt, 
                        pktSize, (Srio_SockAddrInfo *)pDestnAddress) < 0)
    {
        Bcp_osalLog ("Error: SRIO Socket Raw send failed\n");
        return -1;
    }

    /* Return success */
    return 0;
}

/** ============================================================================
 *   @n@b srio_recv
 *
 *   @b Description
 *   @n This API is called from BCP driver to receive a BCP output packet received
 *      over SRIO from BCP on remote device.
 *
 *      The API calls the SRIO driver recv API to get the packet from it's flow/Rx Q.
 *
 *   @param[in]  
 *   @n hSrioSocket         SRIO Rx endpoint info handle 
 * 
 *   @param[in]  
 *   @n pPkt                BCP output packet received over SRIO
 * 
 *   @return    Int32
 *   @n >0      -       Number of bytes received. Success.
 *   @n <0      -       No packet received/Error.
 * =============================================================================
 */
Int32 srio_recv (Void*  hSrioSocket, Void** pPkt)
{
    Srio_SockAddrInfo       from;

    /* Poll on any packets received. */
    Srio_rxCompletionIsr (hSrioDriver_Rx);

    /* Process received packets. */
    return Srio_sockRecv ((Srio_SockHandle) hSrioSocket, (Srio_DrvBuffer*)pPkt, &from);
}

/** ============================================================================
 *   @n@b srio_freeRecvBuffer
 *
 *   @b Description
 *   @n SRIO receive buffer cleanup API. Called from BCP driver to free a 
 *      packet received earlier using @a srio_recv () API.
 *
 *   @param[in]  
 *   @n hSrioSocket SRIO socket handle
 *
 *   @param[in]  
 *   @n hDrvBuffer  SRIO driver buffer handle
 *
 *   @param[in]  
 *   @n pktSize     packet size to be freed
 *
 *   @return        Int32
 *   @n 0       -   Success.
 * =============================================================================
 */
Int32 srio_freeRecvBuffer (Void*    hSrioSocket, Void*    hDrvBuffer, uint32_t pktSize)
{
    Qmss_QueueHnd       returnQueueHnd;

    /* Get the return queue. */
    returnQueueHnd = Qmss_getQueueHandle(Cppi_getReturnQueue(Cppi_DescType_HOST, (Cppi_Desc*)hDrvBuffer));

    /* Push the descriptor into the return queue. */
    Qmss_queuePushDescSize (returnQueueHnd, (Ptr)hDrvBuffer, pktSize);

    return 0;
}
