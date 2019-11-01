/**
 *  \file   tcp3d_isr.c
 *
 *  \brief  TCP3D Driver ISR functions.
 *
 *  Copyright (C) Texas Instruments Incorporated 2012
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

/**
 *  Include Files
 */
/* TCP3D driver includes */
#include <ti/drv/tcp3d/tcp3d_drv.h>
#include <ti/drv/tcp3d/src/tcp3d_drv_priv.h>

/* TCP3D Types and OSAL defintions: These files can be overriden by customers
 * to point to their copies. Because of this the files have not been explicitly 
 * specified to include the driver path.*/
#include <tcp3d_drv_types.h>
#include <tcp3d_osal.h>

/****************************************************************************
 *              TCP3D Driver Functions                                      *
 ****************************************************************************/

/**
 *  @brief  TCP3D Driver ISR function for the channels associated with REVT0.
 */
static void Tcp3d_revt0ChannelIsr (Tcp3d_Instance *inst)
{
    Tcp3d_Result            tcp3dResult = TCP3D_DRV_NO_ERR;
    uint32_t                utmpIdx;
    uint32_t                pingOutIdx;
    EDMA3_DRV_PaRAMRegs     *currPrmPtr1;

    utmpIdx = inst->maxCodeBlocks;

    /* Increment the ISR counter */
    inst->pingIntr++;

    /* Check to see if restart needed before exit */
    /**
     * Read the source address of L2P Channel PaRAM to get the current
     * pseudo PaRAM pointer for PING path. Then compare with the start
     * pointer for the index.
     */
    currPrmPtr1 = (EDMA3_DRV_PaRAMRegs *) inst->pingPtrL2p->srcAddr;
    pingOutIdx = GET_CB_IDX(currPrmPtr1 - inst->startPrmPtr);

    /* Check if PING path completed decoding */
    if ( pingOutIdx < utmpIdx )
    {
        tcp3dResult = Tcp3d_start(inst, TCP3D_DRV_START_AUTO);
    
        if ( TCP3D_DRV_NO_ERR != tcp3dResult )
        {
            Tcp3d_osalLog("REVT0 ISR: Tcp3d_start function returned error with value : %d\n", tcp3dResult);
        }
    }
}

/**
 *  @brief  TCP3D Driver ISR function for the channels associated with REVT1.
 */
static void Tcp3d_revt1ChannelIsr (Tcp3d_Instance *inst)
{
    Tcp3d_Result            tcp3dResult = TCP3D_DRV_NO_ERR;
    uint32_t                utmpIdx;
    uint32_t                pongOutIdx;
    EDMA3_DRV_PaRAMRegs     *currPrmPtr1;

    utmpIdx = inst->maxCodeBlocks;

    /* Increment the ISR counter */
    inst->pongIntr++;

    /* Check to see if restart needed before exit */
    /**
     * Read the source address of L2P Channel PaRAM to get the current
     * pseudo PaRAM pointer for PONG path. Then compare with the start
     * pointer for the index.
     */
    currPrmPtr1 = (EDMA3_DRV_PaRAMRegs *) inst->pongPtrL2p->srcAddr;
    pongOutIdx = GET_CB_IDX(currPrmPtr1 - inst->startPrmPtr);

    /* Check if PING path completed decoding */
    if ( pongOutIdx < utmpIdx )
    {
        tcp3dResult = Tcp3d_start(inst, TCP3D_DRV_START_AUTO);
    
        if ( TCP3D_DRV_NO_ERR != tcp3dResult )
        {
            Tcp3d_osalLog("REVT1 ISR: Tcp3d_start function returned error with value : %d\n", tcp3dResult);
        }
    }
}

/* end of file */
