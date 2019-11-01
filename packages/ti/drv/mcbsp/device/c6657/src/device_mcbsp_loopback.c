/*
 * mcbsp_device.c
 *
 * This file contains MCBSP IP related EVM (platform) specific routines
 * implementation. This file contains the board specific code for enabling 
 * the use of mcbsp driver, and may contain related device pre-driver 
 * initialization routines. The file is provided as a sample configuration 
 * and should be modified by customers for their own platforms and 
 * configurations.
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
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

/* MCBSP Driver Includes. */
#include <ti/drv/mcbsp/mcbsp_types.h>
#include <ti/drv/mcbsp/mcbsp_drv.h>
#include <ti/drv/mcbsp/mcbsp_osal.h>

/* CSL MCBSP Register Layer */
#include <ti/csl/cslr_mcbsp.h>

/* CSL Chip Functional Layer */
#include <ti/csl/csl_chip.h>


/*============================================================================*/
/*                           IMPORTED VARIABLES                               */
/*============================================================================*/

extern Mcbsp_HwInfo Mcbsp_deviceInstInfo[CSL_MCBSP_PER_CNT];
extern Mcbsp_TempBuffer Mcbsp_muteBuf[CSL_MCBSP_PER_CNT];

#ifdef MCBSP_LOOPJOB_ENABLE
extern Mcbsp_TempBuffer Mcbsp_loopDstBuf[CSL_MCBSP_PER_CNT];
extern Mcbsp_TempBuffer Mcbsp_loopSrcBuf[CSL_MCBSP_PER_CNT];
#endif /* MCBSP_LOOPJOB_ENABLE */

/* ========================================================================== */
/*                           MODULE FUNCTIONS                                 */
/* ========================================================================== */
/**
 * \brief   Initializes McBSP driver's data structures
 *
 *          This function initializes the McBSP driver's data structures
 *          including instance objects and channel objects. This is the 
 *          MCBSP Driver Initialization API which needs to be invoked by 
 *          the users to initialize the MCBSP peripheral. This call is 
 *          *mandatory* and should be called before calling any of the 
 *          other driver API's.  This can be modified by customers for 
 *          their application and configuration.
 *
 * \return  None
 */
void McbspDevice_init(void)
{
    int32_t devId = 0;
    void *key;

    /* Begin Critical Section before accessing shared resources. */
    key = Mcbsp_osalEnterMultipleCoreCriticalSection ();

    /* Invalidate the Cache Contents */
    Mcbsp_osalBeginMemAccess ((void *)Mcbsp_deviceInstInfo, sizeof(Mcbsp_deviceInstInfo));

    /* initialize the loop job buffers and the mute buffers for all instances  */
#ifdef MCBSP_LOOPJOB_ENABLE
    Mcbsp_osalBeginMemAccess ((void *)Mcbsp_loopDstBuf, sizeof(Mcbsp_loopDstBuf));
    Mcbsp_osalBeginMemAccess ((void *)Mcbsp_loopSrcBuf, sizeof(Mcbsp_loopSrcBuf));
    memset((void *)Mcbsp_loopDstBuf,0x0,
        sizeof(Mcbsp_TempBuffer) * CSL_MCBSP_PER_CNT);
    memset((void *)Mcbsp_loopSrcBuf,0x0,
        sizeof(Mcbsp_TempBuffer) * CSL_MCBSP_PER_CNT);
#endif /* MCBSP_LOOPJOB_ENABLE */
    Mcbsp_osalBeginMemAccess ((void *)Mcbsp_muteBuf, sizeof(Mcbsp_muteBuf));
    memset((void *)Mcbsp_muteBuf,0x0,
        sizeof(Mcbsp_TempBuffer) * CSL_MCBSP_PER_CNT);

    /* initialize the information for all the device instances                */
    for (devId = 0; devId < CSL_MCBSP_PER_CNT; devId++)
    {
        if (0 == devId)
        {
            /* instance 0 initialisation                                      */
            Mcbsp_deviceInstInfo[devId].obj.instNum = (uint32_t)devId;
            Mcbsp_deviceInstInfo[devId].obj.regs =
                (CSL_McbspRegsOvly)CSL_Mcbsp0_CFG_DATA_REGS;
            Mcbsp_deviceInstInfo[devId].obj.fifoRegs =
                (CSL_BfifoRegsOvly)CSL_Mcbsp0_FIFO_CFG_REGS;
            Mcbsp_deviceInstInfo[devId].obj.dataAddress =
                (CSL_BdataRegsOvly)CSL_Mcbsp0_FIFO_DATA_REGS;
            Mcbsp_deviceInstInfo[devId].obj.edmaTxEventNum =
                (uint32_t)CSL_EDMA3CC2_XEVT0_MCBSP_A;
            Mcbsp_deviceInstInfo[devId].obj.edmaRxEventNum =
                (uint32_t)CSL_EDMA3CC2_REVT0_MCBSP_A;
            Mcbsp_deviceInstInfo[devId].obj.cpuTxEventNum =
                (uint32_t)CSL_INTC0_XEVT0;
            Mcbsp_deviceInstInfo[devId].obj.cpuRxEventNum =
                (uint32_t)CSL_INTC0_REVT0;
        }
        else if (1 == devId)
        {
            /* instance 1 initialisation                                      */
            Mcbsp_deviceInstInfo[devId].obj.instNum = (uint32_t)devId;
            Mcbsp_deviceInstInfo[devId].obj.regs =
                (CSL_McbspRegsOvly)CSL_Mcbsp1_CFG_DATA_REGS;
            Mcbsp_deviceInstInfo[devId].obj.fifoRegs =
                (CSL_BfifoRegsOvly)CSL_Mcbsp1_FIFO_CFG_REGS;
            Mcbsp_deviceInstInfo[devId].obj.dataAddress =
                (CSL_BdataRegsOvly)CSL_Mcbsp1_FIFO_DATA_REGS;
            Mcbsp_deviceInstInfo[devId].obj.edmaTxEventNum =
                (uint32_t)CSL_EDMA3CC2_XEVT1_MCBSP_B;
            Mcbsp_deviceInstInfo[devId].obj.edmaRxEventNum =
                (uint32_t)CSL_EDMA3CC2_REVT1_MCBSP_B;
            Mcbsp_deviceInstInfo[devId].obj.cpuTxEventNum =
                (uint32_t)CSL_INTC0_XEVT1;
            Mcbsp_deviceInstInfo[devId].obj.cpuRxEventNum =
                (uint32_t)CSL_INTC0_REVT1;
        }
        else
        {
            /* do nothing                                                     */
        }
#ifdef MCBSP_LOOPJOB_ENABLE
        /* align the buffers to the cache line size                           */
        Mcbsp_loopSrcBuf[devId].scratchBuffer = (uint32_t *)
            (((uint32_t)Mcbsp_loopSrcBuf[devId].scratchBuf + 0x7F) & ~0x7F);

        Mcbsp_loopDstBuf[devId].scratchBuffer = (uint32_t *)
            (((uint32_t)Mcbsp_loopDstBuf[devId].scratchBuf + 0x7F) & ~0x7F);

        Mcbsp_osalEndMemAccess ((void *)Mcbsp_loopSrcBuf, sizeof(Mcbsp_loopSrcBuf));
        Mcbsp_osalEndMemAccess ((void *)Mcbsp_loopDstBuf, sizeof(Mcbsp_loopDstBuf));
#endif /* MCBSP_LOOPJOB_ENABLE */
        Mcbsp_muteBuf[devId].scratchBuffer = (uint32_t *)
            (((uint32_t)Mcbsp_muteBuf[devId].scratchBuf + 0x7F) & ~0x7F);
        Mcbsp_osalEndMemAccess ((void *)Mcbsp_muteBuf, sizeof(Mcbsp_muteBuf));
    }

    /* Writeback Global Objects */
    Mcbsp_osalEndMemAccess ((void *)Mcbsp_deviceInstInfo, sizeof(Mcbsp_deviceInstInfo));

    /* End Critical Section */
    Mcbsp_osalExitMultipleCoreCriticalSection (key);

    return;
}

/* ========================================================================== */
/*                              END OF FILE                                   */
/* ========================================================================== */
