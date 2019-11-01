/** 
 *   @file  bcp_transport_srio_stubs.c
 *
 *   @brief  
 *      BCP transport layer implementation function stubs for SRIO. 
 *
 *      Since the application is not using BCP remotely, the implementation
 *      for all the APIs are currently just stubbed out.
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
*/

#include "bcp_test.h"

#ifndef DEVICE_K2L
/* CSL SRIO Functional Layer */
#include <ti/csl/csl_srio.h>
#include <ti/csl/csl_srioAux.h>
#include <ti/csl/csl_srioAuxPhyLayer.h>

/* SRIO Device IDs for BCP local & remote devices */
extern const UInt32 DEVICE_ID_BCP_16BIT;
extern const UInt32 DEVICE_ID1_16BIT;
#endif /* DEVICE_K2L */ 

/** ============================================================================
 *   @n@b srio_tx_open
 *
 *   @b Description
 *   @n This API sets up the configuration required to send packets succesfully to
 *      BCP either from a remote device or locally. It uses the Tx configuration
 *      passed by the application to this API.
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
    /* Do nothing */
    return NULL;        
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
    /* Do nothing */
    return NULL;        
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
    /* Return success */        
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
    /* Do nothing. Return success */
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
    /* Do nothing */        
    return NULL;        
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
    /* Do nothing */        
    return 0;
}
