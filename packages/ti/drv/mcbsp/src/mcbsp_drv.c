/*
 * mcbsp_drv.c
 *
 * This file contains Driver Layer Interface implementation for the McBSP Driver.
 * McBSP Driver provides Driver Layer Interface to do operations on the McBSP
 * peripheral like device initialization, channel creation, control commands for
 * peripheral specific operations etc. It uses EDMA3 for data transfer.
 *
 * Copyright (C) 2012 - 2018 Texas Instruments Incorporated - http://www.ti.com/
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

/*============================================================================*/
/*                              INCLUDE FILES                                 */
/*============================================================================*/

/* MCBSP Types and OSAL definitions: These files can be override by customers
 * to point to their copies. Because of this the files have not been explicitly 
 * specified to include the driver path.*/
#include <mcbsp_types.h>
#include <mcbsp_osal.h>

/* MCBSP Driver Includes */
#include <ti/drv/mcbsp/mcbsp_drv.h>
#include <ti/drv/mcbsp/include/mcbsp_pvt.h>


/* ========================================================================== */
/*                       GLOBAL MODULE STATE                                  */
/* ========================================================================== */

/**
 * \brief  Array of Mcbsp instance State objects array
 */
#ifdef __cplusplus
#pragma DATA_SECTION (".mcbsp")
#pragma DATA_ALIGN   (MCBSP_MAX_CACHE_ALIGN)
#else
#pragma DATA_SECTION (Mcbsp_Instances, ".mcbsp");
#pragma DATA_ALIGN   (Mcbsp_Instances, MCBSP_MAX_CACHE_ALIGN)
#endif

Mcbsp_Object Mcbsp_Instances[CSL_MCBSP_PER_CNT];

/**
 * \brief  Mcbsp structure containing the information specific to an instance.
 */
/* Shared Memory Variable to ensure synchronizing SRIO initialization
 * with all the other cores. */
/* Created an array to pad the cache line with MCBSP_MAX_CACHE_ALIGN size */
#ifdef __cplusplus
#pragma DATA_SECTION (".mcbsp")
#pragma DATA_ALIGN   (MCBSP_MAX_CACHE_ALIGN)
#else
#pragma DATA_SECTION (Mcbsp_deviceInstInfo, ".mcbsp");
#pragma DATA_ALIGN   (Mcbsp_deviceInstInfo, MCBSP_MAX_CACHE_ALIGN)
#endif
Mcbsp_HwInfo Mcbsp_deviceInstInfo[CSL_MCBSP_PER_CNT];

/**
 *  \brief  Mute buffer per instance
 *
 *  \note   Buffer used when the mcbsp is placed in mute.
 */
#ifdef __cplusplus
#pragma DATA_SECTION (".mcbsp")
#pragma DATA_ALIGN   (MCBSP_MAX_CACHE_ALIGN)
#else
#pragma DATA_SECTION (Mcbsp_muteBuf, ".mcbsp");
#pragma DATA_ALIGN   (Mcbsp_muteBuf, MCBSP_MAX_CACHE_ALIGN)
#endif
Mcbsp_TempBuffer Mcbsp_muteBuf[CSL_MCBSP_PER_CNT];

#ifdef MCBSP_LOOPJOB_ENABLE
/**
 * \brief  Destination loop buffer per instance
 *
 * \note   Buffer used during the loop job mode for the Transmit section
 */
#ifdef __cplusplus
#pragma DATA_SECTION (".mcbsp")
#pragma DATA_ALIGN   (MCBSP_MAX_CACHE_ALIGN)
#else
#pragma DATA_SECTION (Mcbsp_loopDstBuf, ".mcbsp");
#pragma DATA_ALIGN   (Mcbsp_loopDstBuf, MCBSP_MAX_CACHE_ALIGN)
#endif
Mcbsp_TempBuffer Mcbsp_loopDstBuf[CSL_MCBSP_PER_CNT];

/**
 * \brief  Receive loop buffer per instance
 *
 * \note   Buffer used during the loop job mode for the Receive section
 */
#ifdef __cplusplus
#pragma DATA_SECTION (".mcbsp")
#pragma DATA_ALIGN   (MCBSP_MAX_CACHE_ALIGN)
#else
#pragma DATA_SECTION (Mcbsp_loopSrcBuf, ".mcbsp");
#pragma DATA_ALIGN   (Mcbsp_loopSrcBuf, MCBSP_MAX_CACHE_ALIGN)
#endif
Mcbsp_TempBuffer Mcbsp_loopSrcBuf[CSL_MCBSP_PER_CNT];
#endif /* MCBSP_LOOPJOB_ENABLE */

/*============================================================================*/
/*                         LOCAL FUNCTION PROTOTYPES                          */
/*============================================================================*/
int32_t mcbspSubmitReq(Mcbsp_Object_Unpadded *instHandle,
                       Mcbsp_ChannelObj  *chanHandle,
                       Mcbsp_IOBuf       *ioBuf);
void mcbspConfigureFifo(Mcbsp_HwInfo_Unpadded *hMcbsp,
                        Mcbsp_ChannelObj    *chanHandle,
                        Bool                 enableHwFifo);

#ifndef MCBSP_LOOPJOB_ENABLE
extern uint32_t Mcbsp_edmaChanNum;
#endif /* MCBSP_LOOPJOB_ENABLE */


/* ========================================================================== */
/*                    MODULE PUBLIC FUNCTIONS                                 */
/* ========================================================================== */

/**
 *  @b Description
 *  @n  
 *      This is the MCBSP Driver Initialization API which needs to be 
 *      invoked by the users to initialize the MCBSP peripheral. This call
 *      is *mandatory* and should be called before calling any of the 
 *      other driver API's. 
 *
 *      This should only be called *ONCE* for the device.
 *
 *  @retval
 *      MCBSP_STATUS_COMPLETED     if successful 
 *  @retval
 *      MCBSP_ERR_BADARGS          if not successful
 *      MCBSP_ERR_ALLOC
 */
int32_t mcbspInit (void)
{
    int32_t devId = 0;
    void *criticalSectionInfo;

    /* Begin Critical Section before accessing shared resources. */
    criticalSectionInfo = Mcbsp_osalEnterMultipleCoreCriticalSection ();

    /* Invalidate the Cache Contents */
    Mcbsp_osalBeginMemAccess ((void *)Mcbsp_Instances, sizeof(Mcbsp_Instances));

    /* initialize the information for all the device instances                */
    for (devId = 0; devId < CSL_MCBSP_PER_CNT; devId++)
    {
        /* initialise the instance object                                     */
        memset((void *)&Mcbsp_Instances[devId], 0, sizeof(Mcbsp_Object));
        Mcbsp_Instances[devId].obj.inUse = FALSE;
    }

    /* Writeback Global Object */
    Mcbsp_osalEndMemAccess ((void *)Mcbsp_Instances, sizeof(Mcbsp_Instances));

    /* End Critical Section */
    Mcbsp_osalExitMultipleCoreCriticalSection (criticalSectionInfo);

    return MCBSP_STATUS_COMPLETED;
}

/**
 * \brief   Allocates and configures the McBSP instance specified by devid.
 *
 *  Acquires the Handle of the McBSP and configure the McBSP by
 *  default for the following things.
 *      o   Data output for audio playback
 *      o   Data input for audio recording
 *      o   Configure the McBSP in DSP/TDM data format mode of the audio
 *          codec.
 *      o   Configure the McBSP to receive the Frame Sync and bit clock
 *          externally for both receiver and transmitter.
 *      o   McBSP can also be configured to generate Frame Sync and
 *          bit clock internally by enabling sample rate generator and
 *          frame sync generator blocks respectively depending on the
 *          fields set in the device parameter structure which shall
 *          be passed to mcbspBindDev() function as one of the parameter.
 *
 * \param   devp         [OUT]  pointer to hold allocated instance object ptr
 * \param   devid        [IN]   instance number of the mcbsp
 * \param   devParams    [IN]   user supplied data params.
 *
 * \return  MCBSP_STATUS_COMPLETED     if successful
 *
 *          MCBSP_ERR_BADARGS          if not successful
 *          MCBSP_ERR_ALLOC
 *
 * \enter   devp        should be non NULL and valid pointer,
 *          devId       should be < CSL_MCBSP_PER_CNT
 *          devParams   should be non NULL and valid pointer,
 *
 * \leave   Not implemented
 */
int32_t mcbspBindDev(void* *devp, int32_t devid, void* devParams)
{
    Mcbsp_Object_Unpadded *instHandle = NULL;
    Mcbsp_Params *params     = NULL;
    uint32_t      ctrlMask   = 0x00;
    uint32_t      count      = 0x00;
    int32_t       status     = MCBSP_STATUS_COMPLETED;
    void         *criticalSectionInfo;

    /* Begin Critical Section before accessing shared resources. */
    criticalSectionInfo = Mcbsp_osalEnterMultipleCoreCriticalSection ();

    /* Invalidate the Cache Contents */
    Mcbsp_osalBeginMemAccess ((void *)Mcbsp_Instances, sizeof(Mcbsp_Instances));
    Mcbsp_osalBeginMemAccess ((void *)Mcbsp_deviceInstInfo, sizeof(Mcbsp_deviceInstInfo));

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
    if ((NULL == devp) || (NULL == devParams) || (devid >= CSL_MCBSP_PER_CNT))
    {
        status = MCBSP_ERR_BADARGS;
    }

    if (MCBSP_STATUS_COMPLETED == status)
    {
        if (TRUE == Mcbsp_Instances[devid].obj.inUse)
        {
            status = MCBSP_ERR_INUSE;
        }
        else
        {
#endif /* MCBSP_DISABLE_INPUT_PARAM_CHECK */
            /* copy the pointer to the instance object                        */
            instHandle = &(Mcbsp_Instances[devid].obj);

            /* set the module state as in use                                 */
            Mcbsp_Instances[devid].obj.inUse = TRUE;

            params = (Mcbsp_Params *)devParams;

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
            /* Only DMA mode of the operation is supported for Mcbsp mode     */
            if ((Mcbsp_DevMode_McBSP == params->mode) &&
                (Mcbsp_OpMode_DMAINTERRUPT != params->opMode))
            {
                status = MCBSP_ERR_BADMODE;
            }
            else
            {
                if (NULL == params->srgSetup)
                {
                    status = MCBSP_ERR_BADMODE;
                }
            }
        }
    }
#endif /* MCBSP_DISABLE_INPUT_PARAM_CHECK */

    if (MCBSP_STATUS_COMPLETED == status)
    {
        /* update the instance of the device being created                    */
        instHandle->instNum = devid;

        /* update the user supplied params to the instance object             */
        instHandle->mode = params->mode;
        instHandle->opMode = params->opMode;
        instHandle->enablecache = params->enablecache;

        /* copy the SOC related information in to the instance object         */
        instHandle->hwInfo = Mcbsp_deviceInstInfo[devid].obj;

        /* stop the state machine of RX and TX                                */
        instHandle->stopSmFsXmt = TRUE;
        instHandle->stopSmFsRcv = TRUE;

        instHandle->retryCount = Mcbsp_POLLED_RETRYCOUNT;

        /* configure the default values for the transmit channel              */
        instHandle->xmtObj.mode = (uint16_t)MCBSP_MODE_OUTPUT;
        instHandle->xmtObj.devHandle = NULL;
        instHandle->xmtObj.cbFxn   = NULL;
        instHandle->xmtObj.cbArg   = NULL;
        instHandle->xmtObj.edmaHandle = NULL;
        instHandle->xmtObj.edmaEventQue = Mcbsp_TXEVENTQUE;
        instHandle->xmtObj.edmaCallback = NULL;
        instHandle->xmtObj.xferChan = 0x00;
        instHandle->xmtObj.tcc = 0x00;
        instHandle->xmtObj.tempIOBuf = NULL;
        instHandle->xmtObj.submitCount = 0x00;
        instHandle->xmtObj.dataFormat = Mcbsp_BufferFormat_1SLOT;
        instHandle->xmtObj.bMuteON = FALSE;
        instHandle->xmtObj.paused = FALSE;
        instHandle->xmtObj.flush = FALSE;
        instHandle->xmtObj.isTempIOBufValid = FALSE;
        instHandle->xmtObj.enableHwFifo = TRUE;
        instHandle->xmtObj.gblErrCbk = NULL;
        instHandle->xmtObj.userDataBufferSize = 0x00;
        instHandle->xmtObj.loopJobBuffer = NULL;
        instHandle->xmtObj.loopJobLength = 0x00;
        instHandle->xmtObj.nextLinkParamSetToBeUpdated = 0x00;
        instHandle->xmtObj.loopjobUpdatedinParamset = FALSE;
        instHandle->xmtObj.roundedWordWidth = 0x00;

        instHandle->rcvObj.mode = (uint16_t)MCBSP_MODE_INPUT;
        instHandle->rcvObj.devHandle = NULL;
        instHandle->rcvObj.cbFxn   = NULL;
        instHandle->rcvObj.cbArg   = NULL;
        instHandle->rcvObj.edmaHandle = NULL;
        instHandle->rcvObj.edmaEventQue = Mcbsp_RXEVENTQUE;
        instHandle->rcvObj.edmaCallback = NULL;
        instHandle->rcvObj.xferChan = 0x00;
        instHandle->rcvObj.tcc = 0x00;
        instHandle->rcvObj.tempIOBuf = NULL;
        instHandle->rcvObj.submitCount = 0x00;
        instHandle->rcvObj.dataFormat = Mcbsp_BufferFormat_1SLOT;
        instHandle->rcvObj.bMuteON = FALSE;
        instHandle->rcvObj.paused = FALSE;
        instHandle->rcvObj.flush = FALSE;
        instHandle->rcvObj.isTempIOBufValid = FALSE;
        instHandle->rcvObj.enableHwFifo = TRUE;
        instHandle->rcvObj.gblErrCbk = NULL;
        instHandle->rcvObj.userDataBufferSize = 0x00;
        instHandle->rcvObj.loopJobBuffer = NULL;
        instHandle->rcvObj.loopJobLength = 0x00;
        instHandle->rcvObj.nextLinkParamSetToBeUpdated = 0x00;
        instHandle->rcvObj.loopjobUpdatedinParamset = FALSE;
        instHandle->rcvObj.roundedWordWidth = 0x00;

#ifdef MCBSP_LOOPJOB_ENABLE
        /* driver is compiled in loop Job mode                                */
        instHandle->loopJobMode = TRUE;
#else
        instHandle->loopJobMode = FALSE;
#endif

        for (count = 0; count < Mcbsp_MAXLINKCNT; count++)
        {
            instHandle->xmtObj.pramTbl[count] = 0x00;
            instHandle->rcvObj.pramTbl[count] = 0x00;
            instHandle->xmtObj.pramTblAddr[count] = 0x00;
            instHandle->rcvObj.pramTblAddr[count] = 0x00;
        }

        /* set the status of the channel to closed                            */
        instHandle->xmtObj.chanState = Mcbsp_DriverState_CLOSED;
        instHandle->rcvObj.chanState = Mcbsp_DriverState_CLOSED;

        /* Pending and Floating queues for TX and RX channels */
        instHandle->xmtObj.ptrQPendList  = params->txQPendingList;
        instHandle->xmtObj.ptrQFloatList = params->txQFloatingList;
        instHandle->rcvObj.ptrQPendList  = params->rxQPendingList;
        instHandle->rcvObj.ptrQFloatList = params->rxQFloatingList;

        if (MCBSP_STATUS_COMPLETED == status)
        {
            /* Reset the McBSP Transmitter, receiver and SRGR before          *
             * configuration                                                  */
            ctrlMask = ((Mcbsp_SpcrCtrl_TX_DISABLE | 
                         Mcbsp_SpcrCtrl_RX_DISABLE) |
                       (Mcbsp_SpcrCtrl_SRG_DISABLE | 
                        Mcbsp_SpcrCtrl_FSYNC_DISABLE));

            status = Mcbsp_localResetCtrl(instHandle,ctrlMask);
            if (MCBSP_STATUS_COMPLETED != status)
            {
                /* Writeback Global Object */
                Mcbsp_osalEndMemAccess ((void *)Mcbsp_Instances, sizeof(Mcbsp_Instances));
                Mcbsp_osalEndMemAccess ((void *)Mcbsp_deviceInstInfo, sizeof(Mcbsp_deviceInstInfo));

                /* End Critical Section */
                Mcbsp_osalExitMultipleCoreCriticalSection (criticalSectionInfo);

                return status;
            }

            /* copy the configuration for the sample rate generator and config*
             * the emulation mode and DLB mode settings                       */

            /* Configure the McBSP with user supplied parameters              */
            instHandle->srgrConfig = *(params->srgSetup);

            /* reset the Mcbsp                                                */
            instHandle->hwInfo.regs->SPCR = 0x00u;

            /* set the DLB mode settings                                      */
            instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_DLB_MASK);
            instHandle->hwInfo.regs->SPCR |=
                (params->dlbMode << CSL_MCBSP_SPCR_DLB_SHIFT);

            /* set the clock stop mode settings                               */
            instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_CLKSTP_MASK);

            /* set the emulation state                                        */
            instHandle->hwInfo.regs->SPCR &= (~(CSL_MCBSP_SPCR_SOFT_MASK |
                                                CSL_MCBSP_SPCR_FREE_MASK));
            instHandle->hwInfo.regs->SPCR |=
                (params->emulationMode << CSL_MCBSP_SPCR_SOFT_SHIFT);
        }

        if (MCBSP_STATUS_COMPLETED != status)
        {
            *devp = NULL;
        }
        else
        {
            *devp = &(Mcbsp_Instances[devid]);

            /* set the status of the driver to created                        */
            instHandle->devState = Mcbsp_DriverState_CREATED;
        }
    }

    /* Writeback Global Object */
    Mcbsp_osalEndMemAccess ((void *)Mcbsp_Instances, sizeof(Mcbsp_Instances));
    Mcbsp_osalEndMemAccess ((void *)Mcbsp_deviceInstInfo, sizeof(Mcbsp_deviceInstInfo));

    /* End Critical Section */
    Mcbsp_osalExitMultipleCoreCriticalSection (criticalSectionInfo);

    return status;
}

/**
 *  \brief  Creates a communication channel in specified mode to communicate
 *          data between the application and the McBSP device instance. This
 *          function sets the required hardware configurations for the data
 *          transactions. It returns configured channel handle to application
 *          which will be used in all further transactions with the channel.
 *
 *          Pre-requisites:
 *          1.  Valid chanParams structure
 *              This takes much information pertaining to mcbsp channel
 *              configuration such as how many slots are used for this
 *              channel what are their communication parameters, clock settings etc.
 *          2.  Valid device pointer
 *
 * \param   chanp        [IN]     Channel Handler
 * \param   devp         [IN]     Device pointer
 * \param   mode         [IN]     channel  mode -> input or output
 * \param   chanParams   [IN]     channel parameters from user
 * \param   cbFxn        [IN]     callback function pointer
 * \param   cbArg        [IN]     callback function Arguments
 *
 * \return  MCBSP_STATUS_COMPLETED     if successful
 *          MCBSP_ERR_BADIO        if not successful
 *          MCBSP_ERR_ALLOC            "
 *          MCBSP_ERR_BADARGS      if passed invalid chanParams structure
 */
int32_t mcbspCreateChan(void*            *chanp,
                       void*              devp,
                       int32_t            mode,
                       void*              chanParams,
                       Mcbsp_CallbackFxn  cbFxn,
                       void*              cbArg)
{
    Mcbsp_Object *hInst = NULL;
    Mcbsp_Object_Unpadded *instHandle = NULL;
    Mcbsp_ChannelObj    *chanHandle = NULL;
    Mcbsp_ChanParams    *chanparam  = NULL;
    void*                criticalSectionInfo;
    int32_t              status     = MCBSP_STATUS_COMPLETED;

/* Begin parameter checking                                                   */
#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
    if (((NULL == chanParams)
        || (NULL == cbFxn)
        || (NULL == cbArg)
        || (NULL == devp))
        || ((MCBSP_MODE_INPUT != mode) && (MCBSP_MODE_OUTPUT != mode)))
    {
        return MCBSP_ERR_BADARGS;
    }
    else
    {
#endif
        chanparam = (Mcbsp_ChanParams *)chanParams;
#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
        if (NULL == chanparam->edmaHandle)
        {
            return MCBSP_ERR_BADARGS;
        }
    }
#endif  /* MCBSP_DISABLE_INPUT_PARAM_CHECK */
/* End parameter checking                                                     */

    /* Begin Critical Section before accessing shared resources. */
    criticalSectionInfo = Mcbsp_osalEnterMultipleCoreCriticalSection ();

    /* Invalidate the Cache Contents */
    Mcbsp_osalBeginMemAccess ((void *)devp, sizeof(Mcbsp_Object));

    if (MCBSP_STATUS_COMPLETED == status)
    {
        hInst = (Mcbsp_Object *)devp;
        instHandle = (Mcbsp_Object_Unpadded *)&(hInst->obj);

        /* get the pointer to the required channel structure              */
        if (MCBSP_MODE_INPUT == mode)
        {
            chanHandle = &instHandle->rcvObj;
        }
        else
        {
            chanHandle = &instHandle->xmtObj;
        }

        /* we will check if the current requested channel is availablehere*
         * protect the channel status so that multiple threads requesting *
         * same channel do not cause a corruption                         */
        if (Mcbsp_DriverState_CLOSED == chanHandle->chanState)
        {
            chanHandle->chanState = Mcbsp_DriverState_OPENED;
        }
        else
        {
            /* Requested channel is already taken hence we will set status*
             * as invalid                                                 */
            status = MCBSP_ERR_BADMODE;
        }

        if (MCBSP_STATUS_COMPLETED == status)
        {
            chanHandle->mode = (uint16_t)mode;
            chanHandle->devHandle = hInst;

            /* Assign the respective callback function                        */
            chanHandle->cbFxn = cbFxn;
            chanHandle->cbArg = cbArg;

            /* This is used in EDMA mode to populate paramsets in PINGPONG    */
            chanHandle->nextLinkParamSetToBeUpdated = 0;

            /* Initialize Mute parameter                                      */
            chanHandle->bMuteON = FALSE;

            chanHandle->isTempIOBufValid = FALSE;

            /* Counter that counts outstanding requests of this channel       */
            chanHandle->submitCount = 0;

            /* Global error callback registered to channel                    */
            chanHandle->gblErrCbk = chanparam->gblCbk;

            /* copy the edma event queue details                              */
            chanHandle->edmaEventQue = chanparam->edmaEventQue;

            /* store the EDMA3 module handle                                  */
            chanHandle->edmaHandle = chanparam->edmaHandle;

            /* configure the FIFO                                             */
            chanHandle->enableHwFifo = chanparam->enableHwFifo;

            /* copy the user settings in to the channel object                */
            chanHandle->chanConfig = *(chanparam->chanConfig);
            chanHandle->clkSetup   = *(chanparam->clkSetup);
            chanHandle->numEnabledChannels = chanparam->numEnabledChannels;
            chanHandle->dataFormat = chanparam->dataFormat;
        }

        if (MCBSP_STATUS_COMPLETED == status)
        {
            /* configure the actual wordwidth to be used                      */
            switch (chanparam->wordWidth)
            {
                case Mcbsp_WordLength_8:
                    chanHandle->roundedWordWidth = 1u;
                    break;
                case Mcbsp_WordLength_12:
                case Mcbsp_WordLength_16:
                    chanHandle->roundedWordWidth = 2u;
                    break;
                case Mcbsp_WordLength_20:
                case Mcbsp_WordLength_24:
                    chanHandle->roundedWordWidth = 3u;
                    break;
                case Mcbsp_WordLength_32:
                default:
                    chanHandle->roundedWordWidth = 4u;
                    break;
            }

#ifdef  MCBSP_LOOPJOB_ENABLE
            /* Configure the loop job for the user specified buffer if given  */
            if (NULL == chanparam->userLoopJobBuffer)
            {
                if (MCBSP_MODE_INPUT == chanHandle->mode)
                {
                    chanHandle->loopJobBuffer =
                        Mcbsp_loopDstBuf[instHandle->instNum].scratchBuffer;
                }
                else
                {
                    chanHandle->loopJobBuffer =
                        Mcbsp_loopSrcBuf[instHandle->instNum].scratchBuffer;
                }
                chanHandle->loopJobLength = chanHandle->roundedWordWidth;
            }
            else
            {
                /* Apps has preference on the loopjob buffer & lets use it    */
                chanHandle->loopJobBuffer = chanparam->userLoopJobBuffer;
                chanHandle->userLoopJobLength = chanparam->userLoopJobLength;

                /* user loopJob is being used                                 */
                chanHandle->userLoopJob = TRUE;
                if (chanHandle->roundedWordWidth >
                        chanparam->userLoopJobLength)
                {
                    /* not enough loopjob buffer has been provided  we        *
                     * should have at least loopbuffer for 1 sync event       */
                    status = MCBSP_ERR_BADARGS;
                }
            }
#endif
        }

        if ((Mcbsp_DevMode_McBSP == instHandle->mode) && (MCBSP_STATUS_COMPLETED == status))
        {
            if (MCBSP_MODE_INPUT == chanHandle->mode)
            {
                /* Assign the Channel ID and TCC                              */
                chanHandle->xferChan     = instHandle->hwInfo.edmaRxEventNum;
                chanHandle->tcc          = instHandle->hwInfo.edmaRxEventNum;
                chanHandle->edmaCallback =
                    (EDMA3_RM_TccCallback)&Mcbsp_localEdmaCallback;
            }
            else
            {
                /* Assign the Channel ID and TCC                              */
                chanHandle->xferChan     = instHandle->hwInfo.edmaTxEventNum;
                chanHandle->tcc          = instHandle->hwInfo.edmaTxEventNum;
                chanHandle->edmaCallback =
                    (EDMA3_RM_TccCallback)&Mcbsp_localEdmaCallback;
            }

            if (MCBSP_STATUS_COMPLETED == status)
            {
                mcbspConfigureFifo(&(instHandle->hwInfo),
                    chanHandle,
                    chanHandle->enableHwFifo);
            }

            if (MCBSP_MODE_INPUT == chanHandle->mode)
            {
                /* configure the RX section                                   */
                status = Mcbsp_localConfigureRcvChannel(instHandle,chanparam);
            }
            else
            {
                /* configure the TX section                                   */
                status = Mcbsp_localConfigureXmtChannel(instHandle,chanparam);
            }

            if (MCBSP_STATUS_COMPLETED == status)
            {
                /* configure the sample rate generator                        */
                status = Mcbsp_localConfigureSrgr(instHandle,chanHandle);
            }
            
            if (MCBSP_STATUS_COMPLETED == status)
            {
                /* enable the internal sample rate generator if required      */
                if (((MCBSP_MODE_INPUT == chanHandle->mode) &&
                    (TRUE == instHandle->rxSrgEnable)) ||
                    ((MCBSP_MODE_OUTPUT == chanHandle->mode) &&
                    (TRUE == instHandle->txSrgEnable)))
                {
                    /* enable the sample rate generator                       */
                    Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_SRG_ENABLE);
                    instHandle->srgEnabled = TRUE;

                    /* wait for the 2CLKG clock cycles                        */
                    Mcbsp_osalWaitNBitClocks(2u);
                }
            
                /* clear the XSYNCERR (to be done only if TX is used)         */
                if (MCBSP_MODE_OUTPUT == chanHandle->mode)
                {
                    /* Enable the TX section                                  */
                    Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_ENABLE);

                    /* wait for 2 CLKR or CLX cycles                          */
                    Mcbsp_osalWaitNBitClocks(2u);

                    /* Disable the TX section to clear any XYNCERR            */
                    Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_DISABLE);
                }

                /* complete the EDMA setup for the driver                     */
                status =  Mcbsp_localSetupEdmaDuringOpen(chanHandle);
            }

#ifdef MCBSP_LOOPJOB_ENABLE
            if (MCBSP_STATUS_COMPLETED == status)
            {
				        /* configure the FIFO for the specific channel                        */
                if (TRUE == chanHandle->enableHwFifo)
                {
                    /* Disable and enable the FIFO so that the events are             *
                     * generated to the Mcbsp for the first time                      */
                    mcbspConfigureFifo(&instHandle->hwInfo,chanHandle,FALSE);
                    mcbspConfigureFifo(&instHandle->hwInfo,chanHandle,TRUE);
                }

                /* enable the EDMA transfer for the channel so that it is     *
                 * ready to transfer the data as soon as the state machine is *
                 * enabled                                                    */
                EDMA3_DRV_enableTransfer(
                    chanHandle->edmaHandle,
                    chanHandle->xferChan,
                    EDMA3_DRV_TRIG_MODE_EVENT);

                /* Start the McBSP hardware                                   */
                if (MCBSP_MODE_INPUT == chanHandle->mode)
                {
                    Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_RX_ENABLE);
                }
                else
                {
                    Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_ENABLE);
                }

                if (MCBSP_MODE_INPUT == chanHandle->mode)
                {
                    if (TRUE == instHandle->rxFsgEnable)
                    {
                        /* enable the sample rate generator                   */
                        Mcbsp_localResetCtrl(
                            instHandle,
                            Mcbsp_SpcrCtrl_FSYNC_ENABLE);
                    }
                }
                else
                {
                    if (TRUE == instHandle->txFsgEnable)
                    {
                        /* enable the sample rate generator                   */
                        Mcbsp_localResetCtrl(
                            instHandle,
                            Mcbsp_SpcrCtrl_FSYNC_ENABLE);
                    }
                }

                /* State machine stop status                                  */
                if (MCBSP_MODE_INPUT == chanHandle->mode)
                {
                    instHandle->stopSmFsRcv = FALSE;
                }
                else
                {
                    instHandle->stopSmFsXmt = FALSE;
                }
            }
#endif
        }
    }

    if (MCBSP_STATUS_COMPLETED != status )
    {
        if (NULL != chanHandle)
        {
            /* set the status of the channel to closed                        */
            chanHandle->mode = (uint16_t)Mcbsp_DriverState_CLOSED;
        }

        /* channel opening for transaction has failed                         */
        chanHandle = NULL;
    }
    else
    {
        *chanp = chanHandle;
    }

    /* Writeback Global Object */
    Mcbsp_osalEndMemAccess ((void *)devp, sizeof(Mcbsp_Object));

    /* End Critical Section */
    Mcbsp_osalExitMultipleCoreCriticalSection (criticalSectionInfo);

    return status;
}

/**
 * \brief   frees a channel and all it's associated resources.
 *
 *          Frees the EDMA resources including EDMA master channel and link
 *          PaRAM entries held by the channel.it also unregisters all the
 *          interrupt handlers.
 *
 * \param   chanp  [IN]       Handle to the channel
 *
 * \return  MCBSP_STATUS_COMPLETED     if successful
 *          MCBSP_STATUS_ABORTED       if not successful
 *
 * \enter   chanp       should be non NULL and valid pointer,
 *
 * \leave   Not implemented
 */
int32_t mcbspDeleteChan(void* chanp)
{
    Mcbsp_Object_Unpadded *instHandle = NULL;
    Mcbsp_ChannelObj *chanHandle = NULL;
    Uint8    linkCnt = 0;
    uint32_t timeOut = 0x00;
    int32_t  status  = MCBSP_STATUS_COMPLETED;
    void *criticalSectionInfo;

    /* Begin Critical Section before accessing shared resources. */
    criticalSectionInfo = Mcbsp_osalEnterMultipleCoreCriticalSection ();

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
    if (NULL == chanp)
    {
        status = MCBSP_ERR_BADARGS;
    }
    else
    {
#endif /* MCBSP_DISABLE_INPUT_PARAM_CHECK */
        chanHandle = (Mcbsp_ChannelObj *)chanp;

        /* Invalidate the Cache Contents */
        Mcbsp_osalBeginMemAccess ((void *)chanHandle->devHandle, sizeof(Mcbsp_Object));

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
        if (Mcbsp_DriverState_OPENED != chanHandle->chanState)
        {
            status = MCBSP_ERR_BADMODE;
        }
        else
        {
            if (NULL == chanHandle->devHandle)
            {
                status = MCBSP_ERR_BADARGS;
            }
            else
            {
#endif
                instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
                if (NULL == instHandle)
                {
                    status = MCBSP_ERR_BADARGS;
                }
            }
        }
    }
#endif /* MCBSP_DISABLE_INPUT_PARAM_CHECK */

    if (MCBSP_STATUS_COMPLETED == status)
    {
        /* reset the channel              .                               */
        Mcbsp_localAbortReset(chanHandle);

        /* Disable the current transfer to make sure that there is no     *
         * running EDMA transfer taking place                             */
        status = EDMA3_DRV_disableTransfer(
                     chanHandle->edmaHandle,
                     chanHandle->xferChan,
                     EDMA3_DRV_TRIG_MODE_EVENT);

        /* Disable current EDMA transfer                                  */
        if (MCBSP_STATUS_COMPLETED == status)
        {
            if (MCBSP_MODE_OUTPUT == chanHandle->mode)
            {
                timeOut = instHandle->retryCount;

                /* Poll for TXEMPTY                                       */
                while ((CSL_MCBSP_SPCR_XEMPTY_MASK ==
                    (CSL_MCBSP_SPCR_XEMPTY_MASK & instHandle->hwInfo.regs->SPCR))
                    && (0 != timeOut))
                {
                    timeOut--;
                }
            }

            /* Reset McBSP before freeing the edma channels               */
            if (MCBSP_MODE_INPUT == chanHandle->mode)
            {
                Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_RX_DISABLE);
            }
            else
            {
                Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_DISABLE);
            }

            /* Free Edma channels                                         */
            status = EDMA3_DRV_freeChannel(
                         chanHandle->edmaHandle,
                         chanHandle->xferChan);
#ifndef MCBSP_LOOPJOB_ENABLE

            /* free the already allocated dummy PaRAM entries         */
            EDMA3_DRV_freeChannel(chanHandle->edmaHandle,
                          Mcbsp_edmaChanNum);
#endif
                         
            if (MCBSP_STATUS_COMPLETED ==  status)
            {
                /* free the EDMA PaRAM entries used for Ping pong buffer  */
                for (linkCnt = 0; linkCnt < Mcbsp_MAXLINKCNT; linkCnt++)
                {
                    status = EDMA3_DRV_freeChannel(
                                 chanHandle->edmaHandle,
                                 chanHandle->pramTbl[linkCnt]);
                    if (MCBSP_STATUS_COMPLETED != status)
                    {
                        break;
                    }
                }
            }

            if (MCBSP_STATUS_COMPLETED == status)
            {
                /* Set the state of the channel as closed                 */
                chanHandle->chanState = Mcbsp_DriverState_CLOSED;

                /* Set the Muted state to FALSE                           */
                chanHandle->bMuteON   = FALSE;

                /* Set the Paused state to FALSE                          */
                chanHandle->paused    = FALSE;
            }
        }

        /* Writeback Global Object */
        Mcbsp_osalEndMemAccess ((void *)chanHandle->devHandle, sizeof(Mcbsp_Object));
    }

    /* End Critical Section */
    Mcbsp_osalExitMultipleCoreCriticalSection (criticalSectionInfo);

    return (status);
}

/**
 * \brief   Closes the McBSP device instance from use.
 *          This API is called by the application when it no longer requires the
 *          device instance. Note that all the channels have to be closed for
 *          the device instance to be deleted sucessfully.
 *
 * \param   devp    [IN]  Handle to the device created.
 *
 * \return  MCBSP_STATUS_COMPLETED     if successful
 *          MCBSP_STATUS_ABORTED       if not successful
 *
 * \enter   devp       should be non NULL and valid pointer,
 *
 * \leave   Not implemented
 */
int32_t mcbspUnBindDev(void* devp)
{
    Mcbsp_Object *hInst = NULL;
    Mcbsp_Object_Unpadded *instHandle = NULL;
    int32_t status = MCBSP_STATUS_COMPLETED;
    void* criticalSectionInfo;

    /* Begin Critical Section before accessing shared resources. */
    criticalSectionInfo = Mcbsp_osalEnterMultipleCoreCriticalSection ();

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
    if (NULL == devp)
    {
        status = MCBSP_ERR_BADARGS;
    }
    else
    {
#endif /* MCBSP_DISABLE_INPUT_PARAM_CHECK */

        hInst = (Mcbsp_Object *)devp;

        /* Invalidate the Cache Contents */
        Mcbsp_osalBeginMemAccess ((void *)hInst, sizeof(Mcbsp_Object));

        instHandle = (Mcbsp_Object_Unpadded *)&(hInst->obj);

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
        if ((CSL_MCBSP_PER_CNT <= instHandle->instNum)                  ||
            (Mcbsp_DriverState_CLOSED != instHandle->xmtObj.chanState)  ||
            (Mcbsp_DriverState_CLOSED != instHandle->rcvObj.chanState))
        {
            status = MCBSP_ERR_BADARGS;
        }
    }
#endif /* MCBSP_DISABLE_INPUT_PARAM_CHECK */

    if (MCBSP_STATUS_COMPLETED == status)
    {
        /* reset the Mcbsp                                                */
        instHandle->hwInfo.regs->SPCR = 0x00u;

        /* Mark driver state as deleted and module as not in use              */
        instHandle->devState = Mcbsp_DriverState_DELETED;
        instHandle->inUse = FALSE;

        /* update the user params to the instance object                      */
        instHandle->instNum = -1;

        /* Intialize the mcbsp driver to default values                       */
        memset(instHandle, 0, sizeof(Mcbsp_Object_Unpadded));

        /* Writeback Global Object */
        Mcbsp_osalEndMemAccess ((void *)hInst, sizeof(Mcbsp_Object));
    }

    /* End Critical Section */
    Mcbsp_osalExitMultipleCoreCriticalSection (criticalSectionInfo);

    return status;
}

/**
 * \brief   Submit a I/O buffer to a channel for processing
 *
 *  The application calls this function to cause the driver
 *  to process the Mcbsp_IOBuf for read/write/flush/abort operations.
 *
 * \param   chanp         [IN]    Pointer to channel
 * \param   ioBuf         [IN]    Pointer to buffer to be submitted
 *
 * \return  MCBSP_STATUS_COMPLETED, if buffer is fully processed
 *          MCBSP_STATUS_ABORTED,   if buffer is aborted
 *          MCBSP_STATUS_PENDING,   if buffer is not fully processed
 *          MCBSP_ERR_BADIO     in case of an error in processing
 *
 */
int32_t mcbspSubmitChan(void* chanp, Mcbsp_IOBuf *const ioBuf)
{
    Mcbsp_Object_Unpadded *instHandle = NULL;
    Mcbsp_ChannelObj      *chanHandle = NULL;
    int32_t                status     = MCBSP_STATUS_COMPLETED;

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
    if ((NULL == chanp) || ( NULL == ioBuf))
    {
        status = MCBSP_ERR_BADARGS;
    }
    else
    {
#endif
        chanHandle = (Mcbsp_ChannelObj *)chanp;

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK

        if ((NULL == chanHandle) || (NULL == chanHandle->devHandle))
        {
            status = MCBSP_ERR_BADARGS;
        }
        else
        {
#endif
            instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);
#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
            if (NULL == instHandle)
            {
                status = MCBSP_ERR_BADARGS;
            }
            else
            {
                if (NULL == ioBuf->addr)
                {
                    if ((Mcbsp_IOBuf_Cmd_READ == ioBuf->cmd) ||
                        (Mcbsp_IOBuf_Cmd_WRITE == ioBuf->cmd))
                    {
                        status = MCBSP_ERR_BADARGS;
                    }
                }
            }
        }
    }
#endif /* MCBSP_DISABLE_INPUT_PARAM_CHECK */

    if (MCBSP_STATUS_COMPLETED == status)
    {
        /* process the command sent by the application                        */
        switch (ioBuf->cmd)
        {
            case Mcbsp_IOBuf_Cmd_READ:
            case Mcbsp_IOBuf_Cmd_WRITE:
                if (TRUE != chanHandle->flush)
                {
                    status = mcbspSubmitReq(instHandle,chanHandle,ioBuf);
                }
                else
                {
                    status = MCBSP_ERR_BADIO;
                }
                break;
            case Mcbsp_IOBuf_Cmd_FLUSH:
                /* Flush command has been issued need to abort the receive    *
                 * channel buffers and complete the TX buffers normally       */
                chanHandle->flush = TRUE;
                Mcbsp_localAbortReset(chanHandle);
                chanHandle->flush = FALSE;
                break;
            case Mcbsp_IOBuf_Cmd_ABORT:
                Mcbsp_localAbortReset(chanHandle);
                break;
            default:
                status = MCBSP_ERR_BADIO;
                break;
        }
    }

    return status;
}

/**
 * \brief   Implements the IOCTLS for McBSP driver.
 *
 *          ControlChan() implements recieved IOCTL commands from the
 *          application and executes them accordingly.
 *
 * \param   chanp  [IN]    Pointer to channel
 * \param   cmd    [IN]    specific IOCTL command
 * \param   arg    [IN]    arguments required for specific commands
 *
 * \return  MCBSP_STATUS_COMPLETED, if command is executed correctly
 *          MCBSP_STATUS_ABORTED,   if command returns error during execution
 *          MCBSP_ERR_NOTIMPL,  if command is not supported
 *          MCBSP_ERR_BADARGS   if args are not correct
 */
int32_t mcbspControlChan(void* chanp, Mcbsp_IOCTL cmd, void* arg)
{
    Mcbsp_ChannelObj *chanHandle = NULL;
    int32_t status = MCBSP_STATUS_COMPLETED;

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
    if (NULL == chanp)
    {
        status = MCBSP_ERR_BADARGS;
    }
    else
    {
#endif /* MCBSP_DISABLE_INPUT_PARAM_CHECK */

        chanHandle = (Mcbsp_ChannelObj *)chanp;

        /* Invalidate the Cache Contents */
        Mcbsp_osalBeginMemAccess ((void *)chanHandle->devHandle, sizeof(Mcbsp_Object));

#ifndef MCBSP_DISABLE_INPUT_PARAM_CHECK
        if (NULL == chanHandle->devHandle)
        {
            status = MCBSP_ERR_BADARGS;
        }
    }
#endif
        
    if (MCBSP_STATUS_COMPLETED == status)
    {
        /* call the function to process the IOCTL command                 */
        status =  Mcbsp_localSubmitIoctl(
                        chanHandle,
                        cmd,
                        arg,
                        NULL);
    }

    /* Writeback Global Object */
    Mcbsp_osalEndMemAccess ((void *)chanHandle->devHandle, sizeof(Mcbsp_Object));

    return (status);
}

/**
 * \brief   McBSP Tx ISR function
 *
 *          This Function is the interrupt service routine for the Mcbsp TX
 *          event.
 *
 * \param   hChan  [IN]  Handle to the channel
 *
 * \return  None
 */
void mcbspGblXmtIsr(void *hChan)
{
    Mcbsp_Object_Unpadded *instHandle = NULL;
    Mcbsp_ChannelObj *chanHandle = (Mcbsp_ChannelObj *)hChan;

    instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);

    /* Only DMA mode of operation is supported by the Mcbsp driver hence we   *
     * will only check if there is an sync error and notify the application   */
    if (CSL_MCBSP_SPCR_XSYNCERR_MASK ==
        (CSL_MCBSP_SPCR_XSYNCERR_MASK & instHandle->hwInfo.regs->SPCR))
    {
        /* call the application registered global callback function           */
        if (NULL != chanHandle->gblErrCbk)
        {
            (*chanHandle->gblErrCbk)((uint32_t)chanHandle,
                instHandle->hwInfo.regs->SPCR,
                NULL);
        }
    }

    return;
}

/**
 * \brief   McBSP Rx ISR function
 *
 *          This Function is the interrupt service routine for the Mcbsp RX
 *          event.
 *
 * \param   hChan  [IN]  Handle to the channel
 *
 * \return  None
 */
void mcbspGblRcvIsr(void *hChan)
{
    Mcbsp_Object_Unpadded *instHandle = NULL;
    Mcbsp_ChannelObj *chanHandle = (Mcbsp_ChannelObj *)hChan;

    instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);

    /* Only DMA mode of operation is supported by the Mcbsp driver hence we   *
     * will only check if there is an sync error and notify the application   */
    if (CSL_MCBSP_SPCR_RSYNCERR_MASK ==
        (CSL_MCBSP_SPCR_RSYNCERR_MASK & instHandle->hwInfo.regs->SPCR))
    {
        /* call the application registered global callback function           */
        if (NULL != chanHandle->gblErrCbk)
        {
            (*chanHandle->gblErrCbk)((uint32_t)chanHandle,
                instHandle->hwInfo.regs->SPCR,
                NULL);
        }
    }

    return;
}

/*============================================================================*/
/*                          LOCAL FUNCTION DEFINTIONS                         */
/*============================================================================*/

/**
 *  \brief   Submit a I/O buffer to a channel for processing
 *
 *   This function is called with the Mcbsp_IOBuf_Cmd_READ or Mcbsp_IOBuf_Cmd_WRITE  
 *   command to process the IOP. The function handles the MCBSP mode. In case
 *   that the driver is currently idle it sets up buffer for transfer. In case
 *   that the driver is currently processing a buffer then it will queue up the
 *   current buffer and exit.
 *
 * \param   instHandle  [IN]  pointer to the instance object
 * \param   chanHandle  [IN] handle to the channel
 * \param   ioBuf       [IN] pointer to Mcbsp I/O buffer.
 *
 * \return  MCBSP_STATUS_PENDING in case the buffer is sucesfully processed
 *          MCBSP_ERR_BADIO  in case of any error.
 */
int32_t mcbspSubmitReq(Mcbsp_Object_Unpadded *instHandle,
                       Mcbsp_ChannelObj *chanHandle,
                       Mcbsp_IOBuf      *ioBuf)
{
    void*           criticalSectionInfo;
    int32_t         status = MCBSP_STATUS_COMPLETED;

    if ((NULL == chanHandle) || (NULL == ioBuf) || (NULL == instHandle))
    {
        return MCBSP_ERR_BADIO;
    }
    if((0 == ioBuf->size) || (ioBuf->size > Mcbsp_MAX_IOBUF_SIZE))
    {
        return MCBSP_ERR_BADIO;
    }

    /* clean the buffers if the cache operation mode is enabled               */
    if (TRUE == instHandle->enablecache)
    {
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            /* Invalidate the cache */
            Mcbsp_osalBeginMemAccess(ioBuf->addr, ioBuf->size);
        }
        else
        {
            /* Writeback the cache */
            Mcbsp_osalEndMemAccess(ioBuf->addr, ioBuf->size);
        }
    }

    /*==========================MCBSP MODE ===================================*/
    if (Mcbsp_DevMode_McBSP == instHandle->mode)
    {
        /* We are entering the crictical section because the current active   *
         * IOP is being check and it can become NULL at any point. Hence we   *
         * will protect this code from the interrupt handler.                 */
        criticalSectionInfo = Mcbsp_osalEnterSingleCoreCriticalSection();

#ifdef MCBSP_LOOPJOB_ENABLE
        if ((Mcbsp_MAXLINKCNT <= chanHandle->submitCount) ||
#else
        /* We now have 3 buffers loaded in the EDMA                           */
        if (((Mcbsp_MAXLINKCNT + 1) <= chanHandle->submitCount) ||
#endif
            (TRUE == chanHandle->paused))
        {
            /* There are enough buffers programmed in the EDMA or if the MCBSP*
             * is issued a pause command,hence queue buffer in to the pending *
             * queue                                                          */
            chanHandle->submitCount++;

            Mcbsp_osalQueuePut(chanHandle->ptrQPendList, (Mcbsp_QueueElem *)ioBuf);

            /* critical section ends                                          */
            Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);
        }
        else
        {
            /* Either one of the paramset or Both the paramsets are free      */

            /* increment the submit count                                     */
            chanHandle->submitCount++;

            Mcbsp_osalQueuePut(chanHandle->ptrQFloatList, (Mcbsp_QueueElem *)ioBuf);

            Mcbsp_localLoadPktToEdma(chanHandle,ioBuf);

            /* critical section ends                                          */
            Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);

        } /* Pending queue empty or not*/
    }
    if (MCBSP_STATUS_COMPLETED == status)
    {
        status = MCBSP_STATUS_PENDING;
    }
    return (status);
}

/**
 * \brief   McBSP SPCR configuration function
 *
 *  This Function is used to set/reset specific bit of SPCR as specified in
 *  the given mask.
 *
 * \param   instHandle   [IN]  pointer to the mcbsp instance object.
 * \param   selectMask   [IN]  the SPCR control mask
 *
 * \return  MCBSP_STATUS_COMPLETED     if successful
 *
 *          MCBSP_ERR_BADARGS          if not successful
 */
int32_t Mcbsp_localResetCtrl(Mcbsp_Object_Unpadded *instHandle, uint32_t selectMask)
{
    if ((NULL == instHandle) || (NULL == instHandle->hwInfo.regs))
    {
        return MCBSP_ERR_BADARGS;
    }

    /* Configuring SPCR for Frame sync generator enable/disable               */
    if (0u != (selectMask & Mcbsp_SpcrCtrl_FSYNC_DISABLE))
    {
        instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_FRST_MASK );
    }

    if (0 != (selectMask & Mcbsp_SpcrCtrl_RX_DISABLE))
    {
        instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_RRST_MASK );
    }

    /* start the sample rate generator                                        */
    if (0u != (selectMask & Mcbsp_SpcrCtrl_SRG_ENABLE))
    {
        instHandle->hwInfo.regs->SPCR |=  CSL_MCBSP_SPCR_GRST_MASK;
    }

    /* Configuring SPCR for Transmit enable/disable                           */
    if (0 != (selectMask & Mcbsp_SpcrCtrl_TX_DISABLE))
    {
        instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_XRST_MASK);
    }

    /* Configuring SPCR for transmit section enable                           */
    if (0 != (selectMask & Mcbsp_SpcrCtrl_TX_ENABLE))
    {
        instHandle->hwInfo.regs->SPCR |= CSL_MCBSP_SPCR_XRST_MASK;
    }

    /* Configuring SPCR for Receive section enable                            */
    if (0 != (selectMask & Mcbsp_SpcrCtrl_RX_ENABLE))
    {
        instHandle->hwInfo.regs->SPCR |= CSL_MCBSP_SPCR_RRST_MASK ;
    }

    /* Set the FRST bit to 1 to start the internal frame sync generator       */
    if (0u != (selectMask & Mcbsp_SpcrCtrl_FSYNC_ENABLE))
    {
        instHandle->hwInfo.regs->SPCR |=  CSL_MCBSP_SPCR_FRST_MASK;
    }

    /* Configuring SPCR for sample rate generator enable/disable              */
    if (0u != (selectMask & Mcbsp_SpcrCtrl_SRG_DISABLE))
    {
        instHandle->hwInfo.regs->SPCR &=  (~CSL_MCBSP_SPCR_GRST_MASK);
    }

    return MCBSP_STATUS_COMPLETED;
}

/**
 *  \brief   This function completes the current pending request and then
 *           invokes the application registered callback.
 *
 *  \param   chanHandle   [IN]  Handle to the channel
 *
 *  \return  None
 *
 *  \enter   chanHandle  is a valid non null pointer
 *
 *  \leave   Not implemented
 */
void Mcbsp_localCompleteCurrentIo(Mcbsp_ChannelObj *chanHandle)
{
    Mcbsp_IOBuf   *ioBuf = NULL;

    if (TRUE == chanHandle->isTempIOBufValid)
    {
        ioBuf = chanHandle->tempIOBuf;

        chanHandle->tempIOBuf = NULL;
        chanHandle->isTempIOBufValid = FALSE;
        /* call the completion callback function registered with us           *
         * during channel creation                                            */
        if ((NULL != chanHandle->cbFxn) && (NULL != chanHandle->cbArg))
        {
            /*  Application callback                                          */
            (*chanHandle->cbFxn)((void*)chanHandle->cbArg,ioBuf);
        }
    }

    chanHandle->currentError = MCBSP_STATUS_COMPLETED;
    chanHandle->userDataBufferSize = 0;
}

/**
 * \brief     This function configures the transmit section of the mcbsp
 *            sync properties.
 *
 * \param     instHandle  [IN] pointer to the instance object.
 * \param     params      [IN] User supplied channel parameters
 *
 * \return    MCBSP_ERR_BADARGS  if configuration fails.
 *            MCBSP_STATUS_COMPLETED if configuration is sucessful.
 */
int32_t Mcbsp_localConfigureXmtChannel(Mcbsp_Object_Unpadded *instHandle,
                                     Mcbsp_ChanParams *params)
{
    uint32_t  tempVal = 0x00;
    int32_t   status  = MCBSP_STATUS_COMPLETED;

    if ((NULL == instHandle) || (NULL == params))
    {
        return MCBSP_ERR_BADARGS;
    }

    /* configure the transmit section                                         */
    /* configure the transmit interrupt setting                               */
    instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_XINTM_MASK);
    instHandle->hwInfo.regs->SPCR |=
        (params->chanConfig->intMode << CSL_MCBSP_SPCR_XINTM_SHIFT);

	/* configure the DX enabler */
	instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_DXENA_MASK);
	instHandle->hwInfo.regs->SPCR |=
    	(params->chanConfig->dxState << CSL_MCBSP_SPCR_DXENA_SHIFT);

    /* configure the transmit control register settings                       */
    instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XPHASE_MASK);
    instHandle->hwInfo.regs->XCR |=
        (params->chanConfig->phaseNum << CSL_MCBSP_XCR_XPHASE_SHIFT);

    /* configure the frame length for single and dual phase frames            */
    instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XFRLEN1_MASK);
    instHandle->hwInfo.regs->XCR |=
        ((params->chanConfig->frmLen1 - 1u) << CSL_MCBSP_XCR_XFRLEN1_SHIFT);

    /* configure the word length of the single and dual phase frames          */
    switch (params->chanConfig->wrdLen1)
    {
        case Mcbsp_WordLength_8:
            tempVal = 0u;
            break;
        case Mcbsp_WordLength_12:
            tempVal = 1u;
            break;
        case Mcbsp_WordLength_16:
            tempVal = 2u;
            break;
        case Mcbsp_WordLength_20:
            tempVal = 3u;
            break;
        case Mcbsp_WordLength_24:
            tempVal = 4u;
            break;
        case Mcbsp_WordLength_32:
            tempVal = 5u;
            break;
        default:
            /* wordlength is not supported by the driver                      */
            status = MCBSP_ERR_BADARGS;
            break;
    }

    instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XWDLEN1_MASK);
    instHandle->hwInfo.regs->XCR |=
        (tempVal << CSL_MCBSP_XCR_XWDLEN1_SHIFT);

    if (Mcbsp_Phase_DUAL == params->chanConfig->phaseNum)
    {

        instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XFRLEN2_MASK);
        instHandle->hwInfo.regs->XCR |=
            ((params->chanConfig->frmLen2 -1u) << CSL_MCBSP_XCR_XFRLEN2_SHIFT);

        /* configure the word length of the single and dual phase frames      */
        switch (params->chanConfig->wrdLen2)
        {
            case Mcbsp_WordLength_8:
                tempVal = 0u;
                break;
            case Mcbsp_WordLength_12:
                tempVal = 1u;
                break;
            case Mcbsp_WordLength_16:
                tempVal = 2u;
                break;
            case Mcbsp_WordLength_20:
                tempVal = 3u;
                break;
            case Mcbsp_WordLength_24:
                tempVal = 4u;
                break;
            case Mcbsp_WordLength_32:
                tempVal = 5u;
                break;
            default:
                /* wordlength is not supported by the driver                  */
                status = MCBSP_ERR_BADARGS;
                break;
        }

        instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XWDLEN2_MASK);
        instHandle->hwInfo.regs->XCR |=
            (tempVal << CSL_MCBSP_XCR_XWDLEN2_SHIFT);
    }
    /* set the companding selection                                           */
    instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XCOMPAND_MASK);
    instHandle->hwInfo.regs->XCR |=
        (params->chanConfig->compandSel << CSL_MCBSP_XCR_XCOMPAND_SHIFT);

    /* set the bit reverse settings                                           */
    instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XWDREVRS_MASK);
    instHandle->hwInfo.regs->XCR |=
        (params->chanConfig->bitReversal << CSL_MCBSP_XCR_XWDREVRS_SHIFT);

    /* frame ignore settings                                                  */
    instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XFIG_MASK);
    instHandle->hwInfo.regs->XCR |=
        (params->chanConfig->frmSyncIgn << CSL_MCBSP_XCR_XFIG_SHIFT);

    /* configure the data delay                                               */
    instHandle->hwInfo.regs->XCR &= (~CSL_MCBSP_XCR_XDATDLY_MASK);
    instHandle->hwInfo.regs->XCR |=
        (params->chanConfig->dataDelay << CSL_MCBSP_XCR_XDATDLY_SHIFT);

    /* configure the multi channel control register settings                  */
    instHandle->hwInfo.regs->MCR &= (~CSL_MCBSP_MCR_XMCM_MASK);
    instHandle->hwInfo.regs->MCR |= (params->multiChanCtrl->multiChanMode
                                          << CSL_MCBSP_MCR_XMCM_SHIFT);

    /* select the partition mode and the channel selection controls           */
    instHandle->hwInfo.regs->MCR &= (~CSL_MCBSP_MCR_XPABLK_MASK);
    instHandle->hwInfo.regs->MCR |= (params->multiChanCtrl->partitionSelA
                                          << CSL_MCBSP_MCR_XPABLK_SHIFT);

    instHandle->hwInfo.regs->MCR &= (~CSL_MCBSP_MCR_XPBBLK_MASK);
    instHandle->hwInfo.regs->MCR |= (params->multiChanCtrl->partitionSelB
                                          << CSL_MCBSP_MCR_XPBBLK_SHIFT);

    instHandle->hwInfo.regs->MCR &= (~CSL_MCBSP_MCR_XMCME_MASK);
    instHandle->hwInfo.regs->MCR |= (params->multiChanCtrl->partitionMode
                                          << CSL_MCBSP_MCR_XMCME_SHIFT);

    /* Configure the channels to be enabled                                   */
    instHandle->hwInfo.regs->XCERE0 = params->chanEnableMask[0];
    instHandle->hwInfo.regs->XCERE1 = params->chanEnableMask[1];
    instHandle->hwInfo.regs->XCERE2 = params->chanEnableMask[2];
    instHandle->hwInfo.regs->XCERE3 = params->chanEnableMask[3];


    /* configure the clock polarity                                           */
    if (Mcbsp_ClkPol_RISING_EDGE == params->clkSetup->clkPolarity)
    {
        /* clock data sampled on rising edge                                  */
        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_CLKXP_MASK);
    }
    else
    {
        /* clock data sampled on falling edge                                 */
        instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_CLKXP_MASK);
    }

    /* configure the frame sync polarity                                      */
    if (Mcbsp_FsPol_ACTIVE_HIGH == params->clkSetup->frmSyncPolarity)
    {
        /* frame sync polarity is active high                                 */
        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_FSXP_MASK);
    }
    else
    {
        /* frame sync polarity is active low                                  */
        instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_FSXP_MASK);
    }

    /* check if the frame sync generator is to be enabled for this TX section */
    if (Mcbsp_FsClkMode_EXTERNAL == params->clkSetup->frmSyncMode)
    {
        /* External frame sync to be used                                     */
        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_FSXM_MASK);

        /* frame sync generator needs to be disabled                          */
        instHandle->txFsgEnable = FALSE;
    }
    else
    {
        /* internal frame sync to be used                                     */
        instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_FSXM_MASK);
        
        /* The FRST and GRST has to be cleared to configure the FSGM bit. 
         * By clearing the FRST and GRST bit, the sample rate generator and the 
         * frame sync generator will be kept in the reset mode. 
         * Ref IR SDOCM00077966
         */
        tempVal = instHandle->hwInfo.regs->SPCR;
        instHandle->hwInfo.regs->SPCR &= ~(CSL_MCBSP_SPCR_FRST_MASK | 
        											 CSL_MCBSP_SPCR_GRST_MASK );

        /* could be internal or configured for DXR to XSR copy                */
        if (Mcbsp_FsClkMode_INTERNAL == params->clkSetup->frmSyncMode)
        {
            /* set the FSGM bit in the SRGR register                          */
            instHandle->hwInfo.regs->SRGR |= (CSL_MCBSP_SRGR_FSGM_MASK);

            /* frame sync generator needs to be enabled                       */
            instHandle->txFsgEnable = TRUE;
        }
        else
        {   /* DXR to XSR copy generates frame sync                           */
            /* reset the FSGM bit in the SRGR register                        */
            instHandle->hwInfo.regs->SRGR &= (~CSL_MCBSP_SRGR_FSGM_MASK);

            /* frame sync generator needs to be disabled                      */
            instHandle->txFsgEnable = FALSE;
        }
        /* The SPCR register values are reverted back once the FSGM bit is set
         */
        instHandle->hwInfo.regs->SPCR = tempVal;
    }

    /* configure the clock mode (external or internal)                        */
    if (Mcbsp_TxRxClkMode_EXTERNAL == params->clkSetup->clkMode)
    {
        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_CLKXM_MASK);

        if (TRUE == instHandle->txFsgEnable)
        {
            /* frame sync generator is using the internal clock hence need to *
             * enable the sample rate generator                               */
            instHandle->txSrgEnable = TRUE;
        }
        else
        {
            /* dont require to enable the sample rate generator               */
            instHandle->txSrgEnable = FALSE;
        }
    }
    else
    {
        /* external mode clock enabled                                        */
        instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_CLKXM_MASK);

        /* sample rate generator is enabled                                   */
        instHandle->txSrgEnable = TRUE;
    }

    return (status);
}


/**
 * \brief     This function configures the receives section of the mcbsp.
 *
 * \param     instHandle  [IN] pointer to the instance object.
 * \param     params      [IN] User supplied channel parameters
 *
 * \return    MCBSP_ERR_BADARGS  if configuration fails.
 *            MCBSP_STATUS_COMPLETED if confioguration is sucessful.
 */
int32_t Mcbsp_localConfigureRcvChannel(Mcbsp_Object_Unpadded *instHandle,
                                     Mcbsp_ChanParams *params)
{
    uint32_t  tempVal = 0x00;
    int32_t   status  = MCBSP_STATUS_COMPLETED;

    if ((NULL == instHandle) || (NULL == params))
    {
        return MCBSP_ERR_BADARGS;
    }

    /* configure the receive section                                          */
    /* configure the receive interrupt setting                                */
    instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_RINTM_MASK);
    instHandle->hwInfo.regs->SPCR |=
        (params->chanConfig->intMode << CSL_MCBSP_SPCR_RINTM_SHIFT);

    /* configure the Receive sign-extension and justification                 */
    instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_RJUST_MASK);
    instHandle->hwInfo.regs->SPCR |=
        (params->chanConfig->rjust << CSL_MCBSP_SPCR_RJUST_SHIFT);

    /* configure the receive control register settings                        */
    instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RPHASE_MASK);
    instHandle->hwInfo.regs->RCR |=
        (params->chanConfig->phaseNum << CSL_MCBSP_RCR_RPHASE_SHIFT);

    /* configure the frame length for single and dual phase frames            */
    instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RFRLEN1_MASK);
    instHandle->hwInfo.regs->RCR |=
        ((params->chanConfig->frmLen1 - 1u) << CSL_MCBSP_RCR_RFRLEN1_SHIFT);

    /* configure the word length of the single and dual phase frames          */
    switch (params->chanConfig->wrdLen1)
    {
        case Mcbsp_WordLength_8:
            tempVal = 0u;
            break;
        case Mcbsp_WordLength_12:
            tempVal = 1u;
            break;
        case Mcbsp_WordLength_16:
            tempVal = 2u;
            break;
        case Mcbsp_WordLength_20:
            tempVal = 3u;
            break;
        case Mcbsp_WordLength_24:
            tempVal = 4u;
            break;
        case Mcbsp_WordLength_32:
            tempVal = 5u;
            break;
        default:
            /* wordlength is not supported by the driver                      */
            status = MCBSP_ERR_BADARGS;
            break;
    }


    instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RWDLEN1_MASK);
    instHandle->hwInfo.regs->RCR |=
        (tempVal << CSL_MCBSP_RCR_RWDLEN1_SHIFT);

    if (Mcbsp_Phase_DUAL == params->chanConfig->phaseNum)
    {
        instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RFRLEN2_MASK);
        instHandle->hwInfo.regs->RCR |=
        ((params->chanConfig->frmLen2 - 1u) << CSL_MCBSP_RCR_RFRLEN2_SHIFT);
    
        /* configure the word length of the single and dual phase frames      */
        switch (params->chanConfig->wrdLen2)
        {
            case Mcbsp_WordLength_8:
                tempVal = 0u;
                break;
            case Mcbsp_WordLength_12:
                tempVal = 1u;
                break;
            case Mcbsp_WordLength_16:
                tempVal = 2u;
                break;
            case Mcbsp_WordLength_20:
                tempVal = 3u;
                break;
            case Mcbsp_WordLength_24:
                tempVal = 4u;
                break;
            case Mcbsp_WordLength_32:
                tempVal = 5u;
                break;
            default:
                /* wordlength is not supported by the driver                      */
                status = MCBSP_ERR_BADARGS;
                break;
        }

        instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RWDLEN2_MASK);
        instHandle->hwInfo.regs->RCR |=
            (tempVal << CSL_MCBSP_RCR_RWDLEN2_SHIFT);
    }
    /* set the companding selection                                           */
    instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RCOMPAND_MASK);
    instHandle->hwInfo.regs->RCR |=
        (params->chanConfig->compandSel << CSL_MCBSP_RCR_RCOMPAND_SHIFT);

    /* set the bit reverse settings                                           */
    instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RWDREVRS_MASK);
    instHandle->hwInfo.regs->RCR |=
        (params->chanConfig->bitReversal << CSL_MCBSP_RCR_RWDREVRS_SHIFT);

    /* frame ignore settings                                                  */
    instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RFIG_MASK);
    instHandle->hwInfo.regs->RCR |=
        (params->chanConfig->frmSyncIgn << CSL_MCBSP_RCR_RFIG_SHIFT);

    /* configure the data delay                                               */
    instHandle->hwInfo.regs->RCR &= (~CSL_MCBSP_RCR_RDATDLY_MASK);
    instHandle->hwInfo.regs->RCR |=
        (params->chanConfig->dataDelay << CSL_MCBSP_RCR_RDATDLY_SHIFT);

    /* Configure the channels to be enabled                                   */
    instHandle->hwInfo.regs->RCERE0 = params->chanEnableMask[0];
    instHandle->hwInfo.regs->RCERE1 = params->chanEnableMask[1];
    instHandle->hwInfo.regs->RCERE2 = params->chanEnableMask[2];
    instHandle->hwInfo.regs->RCERE3 = params->chanEnableMask[3];

    /* configure the MCR register                                             */
    switch (params->multiChanCtrl->multiChanMode)
    {
        case Mcbsp_McmMode_ALL_CHAN_ENABLED_UNMASKED:
        case Mcbsp_McmMode_ALL_CHAN_DISABLED_UNMASKED:
            break;
        default:
            status = MCBSP_ERR_BADARGS;
            break;
    }

    if (MCBSP_STATUS_COMPLETED == status)
    {
        instHandle->hwInfo.regs->MCR &= (~CSL_MCBSP_MCR_RMCM_MASK);
        instHandle->hwInfo.regs->MCR |= (params->multiChanCtrl->multiChanMode
                                          << CSL_MCBSP_MCR_RMCM_SHIFT);
    }

    /* select the partition mode and the channel selection controls           */
    instHandle->hwInfo.regs->MCR &= (~CSL_MCBSP_MCR_RPABLK_MASK);
    instHandle->hwInfo.regs->MCR |= (params->multiChanCtrl->partitionSelA
                                          << CSL_MCBSP_MCR_RPABLK_SHIFT);

    instHandle->hwInfo.regs->MCR &= (~CSL_MCBSP_MCR_RPBBLK_MASK);
    instHandle->hwInfo.regs->MCR |= (params->multiChanCtrl->partitionSelB
                                          << CSL_MCBSP_MCR_RPBBLK_SHIFT);

    instHandle->hwInfo.regs->MCR &= (~CSL_MCBSP_MCR_RMCME_MASK);
    instHandle->hwInfo.regs->MCR |= (params->multiChanCtrl->partitionMode
                                          << CSL_MCBSP_MCR_RMCME_SHIFT);

    /* configure the clock polarity                                           */
    if (Mcbsp_ClkPol_RISING_EDGE == params->clkSetup->clkPolarity)
    {
        /* clock data sampled on rising edge                                  */
        instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_CLKRP_MASK);
    }
    else
    {
        /* clock data sampled on falling edge                                 */
        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_CLKRP_MASK);
    }

    /* configure the frame sync polarity                                      */
    if (Mcbsp_FsPol_ACTIVE_HIGH == params->clkSetup->frmSyncPolarity)
    {
        /* frame sync polarity is active high                                 */
        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_FSRP_MASK);
    }
    else
    {
        /* frame sync polarity is active low                                  */
        instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_FSRP_MASK);
    }


    /* check if the frame sync generator is to be enabled for this section    */
    if (Mcbsp_FsClkMode_INTERNAL == params->clkSetup->frmSyncMode)
    {
        /* set the frame sync generation mode                                 */
        instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_FSRM_MASK);

        /* frame sync generator needs to be enabled                           */
        instHandle->rxFsgEnable = TRUE;
    }
    else
    {
        /* reset the frame sync generation mode                               */
        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_FSRM_MASK);

        /* frame sync generator needs to be disabled                          */
        instHandle->rxFsgEnable = FALSE;
    }

    /* configure the clock mode (external or internal)                        */
    if (Mcbsp_TxRxClkMode_EXTERNAL == params->clkSetup->clkMode)
    {
        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_CLKRM_MASK);

        if (TRUE == instHandle->rxFsgEnable)
        {
            /* frame sync generator is using the internal clock hence need to *
             * enable the sample rate generator                               */
            instHandle->rxSrgEnable = TRUE;
        }
        else
        {
            /* dont require to enable the sample rate generator               */
            instHandle->rxSrgEnable = FALSE;
        }
    }
    else
    {
        /* external mode clock                                                */
        instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_CLKRM_MASK);

        /* sample rate generator is enabled                                   */
        instHandle->rxSrgEnable = TRUE;
    }
    return (status);
}

/**
 * \brief     This function configures the sample rate generator and frame
 *            sync properties.
 *
 * \param     instHandle  [IN] pointer to the instance object.
 * \param     chanHandle  [IN] Handle to the channel.
 *
 * \return    MCBSP_ERR_BADARGS  if calculation fails.
 *            MCBSP_STATUS_COMPLETED if calculation is sucessful.
 */
int32_t Mcbsp_localConfigureSrgr(Mcbsp_Object_Unpadded *instHandle,
                               Mcbsp_ChannelObj *chanHandle)
{
    Bool             srgrConfig  = FALSE;
    uint32_t         clkgDiv     = 0x00;
    uint32_t         noOfBits    = 0x00;
    uint32_t         framePeriod = 0x00;
    Bool             sclkme      = 0x00;
    Bool             clksm       = 0x00;
    int32_t          status      = MCBSP_STATUS_COMPLETED;
    void*            criticalSectionInfo;

    if ((NULL == instHandle) || (NULL == chanHandle))
    {
        return MCBSP_ERR_BADARGS;
    }

    /* check if the SRGR is not configured already                            */
    /* critical section starts                                                */
    criticalSectionInfo = Mcbsp_osalEnterSingleCoreCriticalSection();

    if (FALSE == instHandle->srgConfigured)
    {
        /* set the status that SRGR is configured                             */
        instHandle->srgConfigured = TRUE;

        /* critical section ends                                              */
        Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);

        /* check if this channel requires the SRGR to be enabled              */
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            if (TRUE == instHandle->rxSrgEnable)
            {
                /* Need to configure the SRGR                                 */
                srgrConfig = TRUE;
            }
            else
            {
	         	instHandle->srgConfigured = FALSE;   
            }
        }
        else
        {
            if (TRUE == instHandle->txSrgEnable)
            {
                /* Need to configure the SRGR                                 */
                srgrConfig = TRUE;
            }
            else
            {
	         	instHandle->srgConfigured = FALSE;   
            }            
        }
    }
    else
    {
        /* critical section ends                                          */
        Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);
    }

    if (TRUE == srgrConfig)
    {
        /* set the input clock selection for Sample rate generator            */
        switch (instHandle->srgrConfig.srgInputClkMode)
        {
            /* Input clock is CLKS                                            */
            case Mcbsp_SrgClk_CLKS:
                sclkme = 0x00;
                clksm = 0x00;

                /* set the CLKS polarity (only if CLKS is used)               */
                instHandle->hwInfo.regs->SRGR &= (~CSL_MCBSP_SRGR_CLKSP_MASK);
                instHandle->hwInfo.regs->SRGR |=
                    (instHandle->srgrConfig.clksPolarity
                        << CSL_MCBSP_SRGR_CLKSP_SHIFT);

                /* set the GSYNC option                                       */
                instHandle->hwInfo.regs->SRGR &= (~CSL_MCBSP_SRGR_GSYNC_MASK);
                instHandle->hwInfo.regs->SRGR |=
                    (instHandle->srgrConfig.gSync
                        << CSL_MCBSP_SRGR_GSYNC_SHIFT);
                break;
            /* Input clock for SRG is from CPU clock                          */
            case Mcbsp_SrgClk_CLKCPU:
                sclkme = 0x00;
                clksm = 0x01u;
                break;
            /* Input clock is from receive pin                                */
            case Mcbsp_SrgClk_CLKR:
                sclkme = 0x01u;
                clksm = 0x00;
                break;
            /* Input clock is from transmit pin                               */
            case Mcbsp_SrgClk_CLKX:
                sclkme = 0x01u;
                clksm = 0x01u;
                break;
            default:
                status = MCBSP_ERR_BADARGS;
                break;
        }

        instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_SCLKME_MASK);
        instHandle->hwInfo.regs->PCR |= (sclkme << CSL_MCBSP_PCR_SCLKME_SHIFT);

        instHandle->hwInfo.regs->SRGR &= (~CSL_MCBSP_SRGR_CLKSM_MASK);
        instHandle->hwInfo.regs->SRGR |= (clksm << CSL_MCBSP_SRGR_CLKSM_SHIFT);

        /* calculate and set the CLKG div values                              */
        if (Mcbsp_Phase_DUAL == chanHandle->chanConfig.phaseNum)
        {
            noOfBits = ((chanHandle->chanConfig.wrdLen1
                            * chanHandle->chanConfig.frmLen1)
                        + (chanHandle->chanConfig.wrdLen2
                            * chanHandle->chanConfig.frmLen2));
        }
        else
        {
            noOfBits  = (chanHandle->chanConfig.wrdLen1
                            * chanHandle->chanConfig.frmLen1)
							+ chanHandle->chanConfig.dataDelay;
        }

        clkgDiv = ((instHandle->srgrConfig.srgrInputFreq/
                    (chanHandle->clkSetup.samplingRate
                    * noOfBits)) - 1u);

        framePeriod = (noOfBits - 1u);

        if ((0xFF < clkgDiv) || (0xFFF < framePeriod) ||
            (MCBSP_STATUS_COMPLETED != status))
        {
            /* The CLKGDIV or frame period value has exceeded the limit       */
            status = MCBSP_ERR_BADARGS;
        }
        else
        {
            /* set the value of the CLKGDIV                                   */
            instHandle->hwInfo.regs->SRGR &= (~CSL_MCBSP_SRGR_CLKGDV_MASK);
            instHandle->hwInfo.regs->SRGR |= clkgDiv;

            /* set the value of the frame period                              */
            instHandle->hwInfo.regs->SRGR &= (~CSL_MCBSP_SRGR_FPER_MASK);
            instHandle->hwInfo.regs->SRGR |=
                (framePeriod << CSL_MCBSP_SRGR_FPER_SHIFT);

            /* set the frame width                                            */
            instHandle->hwInfo.regs->SRGR &= (~CSL_MCBSP_SRGR_FWID_MASK);
            instHandle->hwInfo.regs->SRGR |=
                (instHandle->srgrConfig.srgFrmPulseWidth <<
                    CSL_MCBSP_SRGR_FWID_SHIFT);
        }
    }

    return (status);
}

/**
 * \brief   Abort the queued up requests.
 *
 *          This commands aborts all the pending IO requests and returns them
 *          to the application. The current state of the IO request will be set
 *          to ABORTED.
 *
 * \param   chanHandle   [IN]   Handle to the channel whose requests are to be
 *                              aborted
 *
 * \return  None
 */
void Mcbsp_localAbortReset(Mcbsp_ChannelObj *chanHandle)
{
    Mcbsp_Object_Unpadded *instHandle   = NULL;
    Mcbsp_IOBuf      *ioBuf        = NULL;
    void*             criticalSectionInfo;
    uint32_t          timeoutCount = 0x00;

    instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);

    /* initialize the retry count value                                       */
    timeoutCount = instHandle->retryCount;

    /* critical section starts                                        */
    criticalSectionInfo = Mcbsp_osalEnterSingleCoreCriticalSection();

    if (Mcbsp_DevMode_McBSP == instHandle->mode)
    {
        /* Disable the EDMA transfer for the current transfer channel         */
        EDMA3_DRV_disableTransfer(
            chanHandle->edmaHandle,
            chanHandle->xferChan,
            EDMA3_DRV_TRIG_MODE_EVENT);

        if (MCBSP_MODE_OUTPUT == chanHandle->mode)
        {
            /* Wait for the TX to be empty                                    */
            while ((CSL_MCBSP_SPCR_XEMPTY_MASK ==
                (CSL_MCBSP_SPCR_XEMPTY_MASK & instHandle->hwInfo.regs->SPCR)) &&
                (0 != timeoutCount))
            {
                timeoutCount--;
            }
        }

        /* Stop the McBSP instance                                            */
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_RX_DISABLE);
        }
        else
        {
            Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_DISABLE);
        }

        /* Empty the floating queue. Aborted request is currently enqueued    *
         * in the floating queue. Dequeue the floating request in EDMA        *
         * param table and return the actual transfer element count           */
        while (TRUE !=  Mcbsp_osalQueueEmpty(chanHandle->ptrQFloatList))
        {
            ioBuf = (Mcbsp_IOBuf *) Mcbsp_osalQueueGet(chanHandle->ptrQFloatList);

            if (NULL != ioBuf)
            {
                /* if FLUSH cmd called for INPUT channel then status is equal *
                 * to FLUSHED otherwise status is equal to ABORTED            */
                if ((TRUE == chanHandle->flush) &&
                    (MCBSP_MODE_INPUT == chanHandle->mode))
                {
                    ioBuf->status = MCBSP_STATUS_FLUSHED;
                }
                else
                {
                    ioBuf->status = MCBSP_STATUS_ABORTED;
                }

                if ((NULL !=  chanHandle->cbFxn) && (NULL != chanHandle->cbArg))
                {
                    /*  Application callback                                  */
                    (*chanHandle->cbFxn)((void*)chanHandle->cbArg,ioBuf);
                }
                /* Decrement the submit count                                 */
                chanHandle->submitCount--;
            }
        }

        /* Empty the pending queue                                            */
        while (TRUE != Mcbsp_osalQueueEmpty(chanHandle->ptrQPendList))
        {
            ioBuf = (Mcbsp_IOBuf *) Mcbsp_osalQueueGet(chanHandle->ptrQPendList);

            if (NULL != ioBuf)
            {
                /* if FLUSH cmd called for INPUT channel then status is equal *
                 * to MCBSP_STATUS_FLUSHED otherwise status is equal to MCBSP_STATUS_ABORTED    */
                if ((TRUE == chanHandle->flush) &&
                    (MCBSP_MODE_INPUT == chanHandle->mode))
                {
                    ioBuf->status = MCBSP_STATUS_FLUSHED;
                }
                else
                {
                    ioBuf->status = MCBSP_STATUS_ABORTED;
                }

                if ((NULL != chanHandle->cbFxn) && (NULL != chanHandle->cbArg))
                {
                    /*  Application callback                                  */
                    (*chanHandle->cbFxn)((void*)chanHandle->cbArg,ioBuf);
                }
                
                /* Decrement the submit count                                 */
                chanHandle->submitCount--;
            }
        }

        /* As EDMA is disabled it might have thrown an error and set error bit*
         * Clear the error bit to enable the transfer again                   */
        EDMA3_DRV_clearErrorBits(chanHandle->edmaHandle,chanHandle->xferChan);

        /* Configure the EDMA channel and EDMA param tables with              *
         * intialization configuration as they are configured at the          *
         * create time.                                                       */
        Mcbsp_localEdmaChanPaRAMsetup(chanHandle);

#ifdef MCBSP_LOOPJOB_ENABLE

		/* configure the FIFO for the specific channel                        */
        if (TRUE == chanHandle->enableHwFifo)
        {
            /* Disable and enable the FIFO so that the events are             *
             * generated to the Mcbsp for the first time                      */
            mcbspConfigureFifo(&instHandle->hwInfo,chanHandle,FALSE);
            mcbspConfigureFifo(&instHandle->hwInfo,chanHandle,TRUE);
        }

        /* Enable the EDMA transfer to start the Loop job running             */
        EDMA3_DRV_enableTransfer(
            chanHandle->edmaHandle,
            chanHandle->xferChan,
            EDMA3_DRV_TRIG_MODE_EVENT);

        chanHandle->loopjobUpdatedinParamset = TRUE;
        chanHandle->nextLinkParamSetToBeUpdated = 0;
        
        /* start the Mcbsp channel                                             */
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_RX_ENABLE);
        }
        else
        {
			Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_ENABLE);
		}
#endif
    }
    /* critical section ends                                          */
    Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);
}
/**
 *
 * \brief   This function configures the Hardware FIFO
 *
 * \param   hMcbsp       [IN] pointer to the Mcbsp Hardware information struct
 * \param   chanHandle   [IN] handle to the channel
 * \param   enableHwFifo [IN] option to Enable or disbale the FIFO
 *
 * \return  None
 *
 * \enter   hMcbsp      is a valid non null pointer
 *          chanHandle  is a valid non null pointer
 *
 * \leave   Not implemented
 *
 */
void mcbspConfigureFifo(Mcbsp_HwInfo_Unpadded *hMcbsp,
                        Mcbsp_ChannelObj    *chanHandle,
                        Bool                 enableHwFifo)
{
    /* check if the HW FIFO usage is requested by the user for this channel   */
    if (TRUE == enableHwFifo)
    {
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            /* configure the receive channel                                  */
            /* Step 1 : configure the "WNUMDMA" and "WNUMEVT" bits before     *
             * enabling the FIFO                                              */
            hMcbsp->fifoRegs->RFIFOCTL =
                (1 << CSL_BFIFO_RFIFOCTL_RNUMEVT_SHIFT);

            hMcbsp->fifoRegs->RFIFOCTL |=
                (CSL_BFIFO_RFIFOCTL_RNUMDMA_1WORDS
                    << CSL_BFIFO_RFIFOCTL_RNUMDMA_SHIFT);

            /* enable the FIFO now by setting the "WENA" bit                  */
            hMcbsp->fifoRegs->RFIFOCTL |=
                (CSL_BFIFO_RFIFOCTL_RENA_ENABLED
                    << CSL_BFIFO_RFIFOCTL_RENA_SHIFT);
        }
        else
        {
            /* configure the transmit channel                                 */
            /* Step 1 : configure the "WNUMDMA" and "WNUMEVT" bits before     *
             * enabling the FIFO                                              */
            hMcbsp->fifoRegs->WFIFOCTL =
                (1 << CSL_BFIFO_WFIFOCTL_WNUMEVT_SHIFT);

            hMcbsp->fifoRegs->WFIFOCTL |=
                (CSL_BFIFO_WFIFOCTL_WNUMDMA_1WORDS
                    << CSL_BFIFO_WFIFOCTL_WNUMDMA_SHIFT);

            /* enable the FIFO now by setting the "WENA" bit                  */
            hMcbsp->fifoRegs->WFIFOCTL |=
                (CSL_BFIFO_WFIFOCTL_WENA_ENABLED
                    << CSL_BFIFO_WFIFOCTL_WENA_SHIFT);
        }
    }
    else
    {
        /* FIFO needs to be disabled                                          */
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            /* disable the FIFO now by resetting the "WENA" bit               */
            hMcbsp->fifoRegs->RFIFOCTL =
                (CSL_BFIFO_RFIFOCTL_RENA_DISABLED
                    << CSL_BFIFO_RFIFOCTL_RENA_SHIFT);
        }
        else
        {
            /* disable the FIFO now by resetting the "WENA" bit               */
            hMcbsp->fifoRegs->WFIFOCTL =
                (CSL_BFIFO_WFIFOCTL_WENA_DISABLED
                    << CSL_BFIFO_WFIFOCTL_WENA_SHIFT);
        }
    }
}

/**
 * \brief   This is going to complete the current request and abort
 *          all other reqest.
 *
 * \param   chanHandle   [IN]   Channel handle
 *
 * \return  None
 *
 * \enter   chanHandle  is a valid non null pointer
 *
 * \leave   Not implemented
 */
void Mcbsp_localCancelAndAbortAllIo(Mcbsp_ChannelObj *chanHandle)
{
    chanHandle->submitCount--;
    chanHandle->tempIOBuf->status = MCBSP_STATUS_ABORTED;

    Mcbsp_localCompleteCurrentIo(chanHandle);

    Mcbsp_localAbortReset(chanHandle);
}

#ifndef MCBSP_LOOPJOB_ENABLE
/**
 *  \brief  This function waits for the FIFO to be emptied(if enabled) and the
 *          TX empty bit to be set so that the TX section could be disabled when
 *          there is no data present.
 *
 *  \param  arg0   [IN]  Handle to the TX channel
 *  \param  arg1   [IN]  unused
 *
 *  \return None
 *
 *  \enter  arg0 is a valid non null pointer
 *
 *  \leave  Not implemented
 */
void Mcbsp_TxFifo(int32_t arg0, int32_t arg1)
{
    Mcbsp_ChannelObj  *chanHandle = NULL;
    Mcbsp_Object *hInst = NULL;
    Mcbsp_Object_Unpadded *instHandle = NULL;
    uint32_t           timeOut    = 0x00;
    void*              criticalSectionInfo;

    chanHandle = (Mcbsp_ChannelObj *)arg0;
    hInst = (Mcbsp_Object *)arg1;
    instHandle = (Mcbsp_Object_Unpadded *)&(hInst->obj);

    /* critical section starts                                                */
    criticalSectionInfo = Mcbsp_osalEnterSingleCoreCriticalSection();

    /* update the timeout value from the instance handle                      */
    timeOut = instHandle->retryCount;

    /* we have come here means that the Mcbsp has got an callback but it      *
     * did have any more buffer to load Hence here we will wait for the       *
     * FIFO to become empty (if FIFO is enabled) else wait for the TX to      *
     * become empty.then we will disable the TX section                       */
    if (TRUE == chanHandle->enableHwFifo)
    {
        while ((0 != (instHandle->hwInfo.fifoRegs->WFIFOSTS &
                       CSL_BFIFO_WFIFOSTS_WLVL_MASK))
                && (0 != timeOut))
        {
            /* reduce the timeout count and check if the FIFO is empty    */
            timeOut--;
        }
    }

    /* reinitialize the retry count                                       */
    timeOut = instHandle->retryCount;

    while ((CSL_MCBSP_SPCR_XEMPTY_MASK ==
            (instHandle->hwInfo.regs->SPCR & CSL_MCBSP_SPCR_XEMPTY_MASK))
           && (0 != timeOut))
    {
        /* reduce the retry count and check if the TX has completed           *
         * transmitting all the bytes                                         */
        timeOut--;
    }

    /* we need to stop the frame sycn generator now.But also need to check    *
     * if  1.The frame sycn generator is actually started By TX.              *
     *     2.The RX is  not feeding of the sample rate generator              */
    if ((TRUE == instHandle->txFsgEnable) &&
        ((TRUE != instHandle->rxFsgEnable)
            || (TRUE == instHandle->stopSmFsRcv)))
    {
        /* Now we can disable the frame sync generator                        */
        Mcbsp_localResetCtrl(
            (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj),
            Mcbsp_SpcrCtrl_FSYNC_DISABLE);
        instHandle->fsgEnabled = FALSE;
    }

    /* Stop the TX section                                                    */
    Mcbsp_localResetCtrl(
        (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj),
        Mcbsp_SpcrCtrl_TX_DISABLE);

    /* Transmit state machine is stopped                                      */
    instHandle->stopSmFsXmt = TRUE;

    /* clear the error bits in the EDMA(as this is the last buffer)           */
    EDMA3_DRV_clearErrorBits(
        chanHandle->edmaHandle,
        chanHandle->xferChan);

    /* complete the IOP now and call the callback to the stream               */
    chanHandle->tempIOBuf = (Mcbsp_IOBuf *) Mcbsp_osalQueueGet(chanHandle->ptrQFloatList);

    /* Decrement the submit count for the IO buffers                          */
    chanHandle->submitCount--;

    chanHandle->isTempIOBufValid = TRUE;
    chanHandle->tempIOBuf->status = chanHandle->currentError;
    Mcbsp_localCompleteCurrentIo(chanHandle);

    /* critical section ends                                                  */
    Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);


}
#endif /* #ifndef MCBSP_LOOPJOB_ENABLE */


/**
 * \brief  This function toggles the index of the edma params
 *
 * \param  index [IN]  pointer to current index
 *
 * \return None
 *
 * \enter  index is a valid non null pointer
 *
 * \leave  Not implemented
 */
void Mcbsp_localGetNextIndex(uint32_t *index)
{
    *index = (((*index) + 1u) & 0x01u);
}

/**
 *  \brief  This function loads the buffers to the actual EDMA paramset.
 *
 *  \param  chanHandle [IN]  Handle to channel.
 *  \param  ioBuf      [IN]  pointer to the ioBuf
 *
 *  \return None
 *
 *  \enter  Not implemented
 *
 *  \leave  Not implemented
 */
void Mcbsp_localLoadPktToEdma(Mcbsp_ChannelObj *chanHandle,Mcbsp_IOBuf *ioBuf)
{
#ifndef MCBSP_LOOPJOB_ENABLE
    Mcbsp_Object_Unpadded *instHandle  = NULL;
#endif
    int32_t             status      = MCBSP_STATUS_COMPLETED;

#ifndef MCBSP_LOOPJOB_ENABLE
    instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);
#endif

    chanHandle->currentDataSize = (uint16_t)ioBuf->size;
    chanHandle->userDataBufferSize = (uint32_t)ioBuf->size;
    
#ifdef MCBSP_LOOPJOB_ENABLE
    if (Mcbsp_MAXLINKCNT == chanHandle->submitCount)
    {
#else
    /* The second and the third buffer will go the link paramsets             */
    if (Mcbsp_MAXLINKCNT <= chanHandle->submitCount)
    {
#endif
        /* Though we have to post to param set directly from here,            *
         * there will be differene between first such buffer and              *
         * second buffer. As we have control here we are second buffer        *
         * and first buffer has not yet returned (or corresponding            *
         * edma callback has not been called.For second buffer, we            *
         * will be updating the second param set, which is currently          *
         * hosting loopjob parameter. Hence increment the index to            *
         * point second paramset and since we are moving out loopjob          *
         * from both param sets, the loopjobUpdatedinParamset is reset        */
        chanHandle->loopjobUpdatedinParamset = FALSE;
        Mcbsp_localGetNextIndex(
            &chanHandle->nextLinkParamSetToBeUpdated);
    }
    
    /* Now update the data buffer to the link params. The paramset to         *
     * be updated is decidec by the "nextLinkParamSetToBeUpdated"             */
    if (MCBSP_STATUS_COMPLETED != Mcbsp_localUpdtDtPktToLnkPrms(chanHandle,ioBuf))
    {
        status = MCBSP_ERR_BADIO;
    }
    
    if ((1u == chanHandle->submitCount) && (MCBSP_STATUS_COMPLETED == status))
    {
#ifdef MCBSP_LOOPJOB_ENABLE
        /* if at all this is the very first buffer, then one param set        *
         * has loop job loaded , self linked and active with the main         *
         * xfer channel param. other param set is ready loaded (just          *
         * now)and has link param set as the one having loopjob (this         *
         * is to ensure that if at all we are not getting any more            *
         * buffers loopjob be will taken over). Now we have to link           *
         * the floating / newly loaded param set to xfer channel.             */
        if (MCBSP_STATUS_COMPLETED != EDMA3_DRV_linkChannel(
                                 chanHandle->edmaHandle,
                                 chanHandle->xferChan,
                                 chanHandle->pramTbl[chanHandle->nextLinkParamSetToBeUpdated]))
        {
            status = MCBSP_ERR_BADIO;
        }
#else
        /* configure the FIFO for the specific channel                        */
        if (TRUE == chanHandle->enableHwFifo)
        {
            /* Disable and enable the FIFO so that the events are             *
             * generated to the Mcbsp for the first time                      */
            mcbspConfigureFifo(&instHandle->hwInfo,chanHandle,FALSE);
            mcbspConfigureFifo(&instHandle->hwInfo,chanHandle,TRUE);
        }

        /* enable the EDMA transfer for the channel so that it is             *
         * ready to transfer the data as soon as the state machine is         *
         * enabled                                                            */
        EDMA3_DRV_enableTransfer(
            chanHandle->edmaHandle,
            chanHandle->xferChan,
            EDMA3_DRV_TRIG_MODE_EVENT);

        /* Start the McBSP hardware                                           */
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            instHandle->stopSmFsRcv = FALSE;
            Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_RX_ENABLE);
        }
        else
        {
            instHandle->stopSmFsXmt = FALSE;
            Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_ENABLE);
        }

        if (((MCBSP_MODE_INPUT == chanHandle->mode) && 
            (TRUE == instHandle->rxFsgEnable)) ||
            ((MCBSP_MODE_OUTPUT == chanHandle->mode) &&
             (TRUE == instHandle->txFsgEnable)))
        {
            /* enable the sample rate generator                               */
            Mcbsp_localResetCtrl(
                instHandle,
                Mcbsp_SpcrCtrl_FSYNC_ENABLE);
        }
#endif /* MCBSP_LOOPJOB_ENABLE */
    }
}

/**
 * \brief   Function to modify the sample rate generator configuration
 *
 * \param   chanHandle  [IN]  Handle to the channel
 * \param   arg         [IN]  pointer to the srg config setup
 *
 * \return  None
 */
int32_t Mcbsp_localModifySampleRate(Mcbsp_ChannelObj *chanHandle,void* arg)
{
    Mcbsp_Object_Unpadded *instHandle = NULL;
    Mcbsp_ClkSetup  *clkConfig  = NULL;
    int32_t            status     = MCBSP_STATUS_COMPLETED;

    if ((NULL == arg) || (NULL == chanHandle))
    {
        return MCBSP_ERR_BADARGS;
    }

    instHandle = &(chanHandle->devHandle->obj);
    if (NULL == instHandle)
    {
        return MCBSP_ERR_BADARGS;
    }

    clkConfig = (Mcbsp_ClkSetup *)arg;

    /* check if the Frame sync clock is generated by the module               */
    if (TRUE == instHandle->srgConfigured)
    {
        /* Configure the McBSP with user supplied parameters                  */
        chanHandle->clkSetup = *(clkConfig);
        instHandle->srgConfigured = FALSE;

        /* stop the sample rate generator                             */
        if (TRUE == instHandle->srgEnabled)
        {
            Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_SRG_DISABLE);
        
            status = Mcbsp_localConfigureSrgr(instHandle,chanHandle);
        
            if (MCBSP_STATUS_COMPLETED == status)
            {
                /* enable the sample rate generator                   */
                Mcbsp_localResetCtrl(
                    instHandle,
                    Mcbsp_SpcrCtrl_SRG_ENABLE);
        
                /* wait for the 2CLKG clock cycles                    */
                Mcbsp_osalWaitNBitClocks(2u);
            }
        
            /* clear the XSYNCERR (to be done only if TX is used)     */
            if (MCBSP_MODE_OUTPUT == chanHandle->mode)
            {
                /* Enable the TX section                              */
                Mcbsp_localResetCtrl(
                    instHandle,
                    Mcbsp_SpcrCtrl_TX_ENABLE);
        
                /* wait for 2 CLKR or CLX cycles                      */
                Mcbsp_osalWaitNBitClocks(2u);
        
                /* Disable the TX section to clear any XYNCERR        */
                Mcbsp_localResetCtrl(
                    instHandle,
                    Mcbsp_SpcrCtrl_TX_DISABLE);
            }
        }
    }
    else
    {
        status = MCBSP_ERR_BADMODE;
        
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            if (TRUE != instHandle->rxSrgEnable)
            {
	            /* Since this mode does not use the sample rate generator, so
	             * there is no point in changing the sample rate. So return 
	             * with success*/
				status	= MCBSP_STATUS_COMPLETED;
            }
        }
        else
        {
            if (TRUE != instHandle->txSrgEnable)
            {
	            /* Since this mode does not use the sample rate generator, so
	             * there is no point in changing the sample rate. So return 
	             * with success*/
	            status	= MCBSP_STATUS_COMPLETED;
            }
        }
    }
    return (status);
}

/* ========================================================================== */
/*                             END OF FILE                                    */
/* ========================================================================== */
