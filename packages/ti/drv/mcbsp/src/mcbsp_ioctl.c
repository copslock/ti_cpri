/*
 * mcbsp_ioctl.c
 *
 * This file contains local functions for McBSP driver which contain implemen-
 * tation for control operation specifc for the McBSP peripheral.
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

/* ========================================================================== */
/*                             INCLUDE FILES                                  */
/* ========================================================================== */

/* MCBSP CSL-RL */
#include <ti/csl/cslr_mcbsp.h>

/* MCBSP Types includes */
#include <mcbsp_types.h>

/* MCBSP LLD includes */
#include <ti/drv/mcbsp/mcbsp_drv.h>
#include <ti/drv/mcbsp/include/mcbsp_pvt.h>

/* MCBSP OSAL layer */
#include <mcbsp_osal.h>

#ifdef MCBSP_LOOPJOB_ENABLE
extern far Mcbsp_TempBuffer Mcbsp_loopDstBuf[CSL_MCBSP_PER_CNT];
extern far Mcbsp_TempBuffer Mcbsp_loopSrcBuf[CSL_MCBSP_PER_CNT];
#endif

/* ========================================================================== */
/*                           FUNCTION DEFINTIONS                              */
/* ========================================================================== */

/**
 * \brief   Implements the IOCTLS for McBSP driver.
 *
 *          This function implements the McBSP device specific control operations
 *          including Start/Stop, Pause/Resume, Channel and Device Reset,
 *          Mute-ON/OFF, McBSP sample rate generator configuration etc.
 *
 * \param  chanHandle  [IN]  Handle to channel
 * \param  cmd         [IN]  Control command number
 * \param  arg         [IN]  Arguments for control command
 * \param  param       [IN]  user defined data structure
 *
 * \return  MCBSP_STATUS_COMPLETED in case of sucess.
 *          MCBSP_ERR_NOTIMPL  in case the command is not supported.
 *          MCBSP driver error code in case of any error.
 */
int32_t Mcbsp_localSubmitIoctl(Mcbsp_ChannelObj *chanHandle,
                             Mcbsp_IOCTL        cmd,
                             void*              arg,
                             void*              param)
{
    Mcbsp_Object_Unpadded  *instHandle  = NULL;
    Mcbsp_srgConfig        *srgConfig   = NULL;
    int32_t                 status      = MCBSP_STATUS_COMPLETED;
#ifdef MCBSP_LOOPJOB_ENABLE
    Mcbsp_IOBuf            *ioBuf       = NULL;
    uint16_t                linkCnt     = 0;
    Mcbsp_ChanParams       *params      = NULL;
#endif

    if ((NULL == chanHandle) || (NULL == chanHandle->devHandle))
    {
        return MCBSP_ERR_BADARGS;
    }

    instHandle = &(chanHandle->devHandle->obj);
    if (NULL == instHandle)
    {
        return MCBSP_ERR_BADARGS;
    }

    /* To remove compiler warning                                             */
    param = param;

    /********************command to start the Mcbsp section********************/
    if (Mcbsp_IOCTL_START == cmd)
    {
        /* stop the Mcbsp channel                                             */
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            if (TRUE == instHandle->stopSmFsRcv)
            {
                Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_RX_ENABLE);
                instHandle->stopSmFsRcv = FALSE;
            }
            else
            {
                /* Already stop port command is issued. This is a redundant   *
                 * command.                                                   */
                status = MCBSP_ERR_BADARGS;
            }
        }
        else
        {
            if (TRUE == instHandle->stopSmFsXmt)
            {
                Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_ENABLE);
                instHandle->stopSmFsXmt = FALSE;
            }
            else
            {
                /* Already stop port command is issued. This is a redundant   *
                 * command.                                                   */
                status = MCBSP_ERR_BADARGS;
            }
        }
    }

    /********************command to stop the Mcbsp section*********************/
    else if (Mcbsp_IOCTL_STOP == cmd)
    {
        /* abort all the buffers in the driver                                */
        Mcbsp_localAbortReset(chanHandle);

        /* stop the hardware                                                  */
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_RX_DISABLE);
            instHandle->stopSmFsRcv = TRUE;
        }
        else
        {
            Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_TX_DISABLE);
            instHandle->stopSmFsXmt = TRUE;
        }
    }
    /****************command to start sample rate generator********************/
    else if (Mcbsp_IOCTL_SRGR_START == cmd)
    {
        Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_SRG_ENABLE);
    }
    /*****************command to stop sample rate generator********************/
    else if (Mcbsp_IOCTL_SRGR_STOP == cmd)
    {
        Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_SRG_DISABLE);
    }
    /*****************command to start frame sync generator********************/
    else if (Mcbsp_IOCTL_FSGR_START == cmd)
    {
        Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_FSYNC_ENABLE);
    }
    else if (Mcbsp_IOCTL_FSGR_STOP == cmd)
    {
        Mcbsp_localResetCtrl(instHandle,Mcbsp_SpcrCtrl_FSYNC_DISABLE);
    }
    /******************command to configure the clock polarity*****************/
    else if (Mcbsp_IOCTL_SET_BCLK_POL == cmd)
    {
        if (NULL == arg)
        {
            return MCBSP_ERR_BADARGS;
        }

        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            if (Mcbsp_ClkPol_FALLING_EDGE == (Mcbsp_ClkPol)(*(uint32_t *)arg))
            {
                instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_CLKRP_MASK);
            }
            else
            {
                instHandle->hwInfo.regs->PCR |= CSL_MCBSP_PCR_CLKRP_MASK;
            }
        }
        else
        {
            if (Mcbsp_ClkPol_FALLING_EDGE == (Mcbsp_ClkPol)(*(uint32_t *)arg))
            {
                instHandle->hwInfo.regs->PCR |= CSL_MCBSP_PCR_CLKXP_MASK;
            }
            else
            {
                instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_CLKXP_MASK);
            }
        }
    }
    /*****************command to configure the frame sync polarity*************/
    else if (Mcbsp_IOCTL_SET_FRMSYNC_POL == cmd)
    {
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            if (Mcbsp_FsPol_ACTIVE_LOW == (Mcbsp_FsPol)(*(uint32_t *)arg))
            {
                instHandle->hwInfo.regs->PCR |= CSL_MCBSP_PCR_FSRP_MASK;
            }
            else
            {
                instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_FSRP_MASK);
            }
        }
        else
        {
            if (Mcbsp_FsPol_ACTIVE_LOW  == (Mcbsp_FsPol)(*(uint32_t *)arg))
            {
                instHandle->hwInfo.regs->PCR |= CSL_MCBSP_PCR_FSXP_MASK;
            }
            else
            {
                instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_FSXP_MASK);
            }
        }
    }
    /****************** commmand to enable the sync error interrupt************/
    else if (Mcbsp_IOCTL_SYNCERR_INT_ENABLE == cmd)
    {
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            instHandle->hwInfo.regs->SPCR |= CSL_MCBSP_SPCR_RINTM_MASK;
        }
        else
        {
            instHandle->hwInfo.regs->SPCR |= CSL_MCBSP_SPCR_XINTM_MASK;
        }
    }
    /*****************command to enable disable the LOOP back mode*************/
    else if (Mcbsp_IOCTL_LOOPBACK == cmd)
    {
        if (NULL != arg)
        {
            if (Mcbsp_Loopback_DISABLE == (Mcbsp_Loopback)(*(uint32_t *)arg))
            {
                instHandle->hwInfo.regs->SPCR |= (CSL_MCBSP_SPCR_DLB_MASK);
            }
            else
            {
                instHandle->hwInfo.regs->SPCR &= (~CSL_MCBSP_SPCR_DLB_MASK);
            }
        }
        else
        {
            status = MCBSP_ERR_BADARGS;
        }
    }
    /*********************command to set the bit clock mode *******************/
    else if (Mcbsp_IOCTL_SET_CLKMODE == cmd)
    {
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            /* configure the clock mode (external or internal)                */
            if (Mcbsp_TxRxClkMode_EXTERNAL == (Mcbsp_TxRxClkMode)(*(Uint8 *)arg))
            {
                instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_CLKRM_MASK);

                if (TRUE == instHandle->rxFsgEnable)
                {
                    /* frame sync generator is using the internal clock hence *
                     * need to enable the sample rate generator               */
                    instHandle->rxSrgEnable = TRUE;
                }
                else
                {
                    /* dont require to enable the sample rate generator       */
                    instHandle->rxSrgEnable = FALSE;
                }
            }
            else
            {
                /* external mode clock                                        */
                instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_CLKRM_MASK);

                /* sample rate generator is enabled                           */
                instHandle->rxSrgEnable = TRUE;
            }

        }
        else
        {
            /* configure the clock mode (external or internal)                */
            if (Mcbsp_TxRxClkMode_EXTERNAL == (Mcbsp_TxRxClkMode)(*(Uint8 *)arg))
            {
                instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_CLKXM_MASK);

                if (TRUE == instHandle->txFsgEnable)
                {
                    /* frame sync generator is using the internal clock hence *
                     * need to enable the sample rate generator               */
                    instHandle->txSrgEnable = TRUE;
                }
                else
                {
                    /* dont require to enable the sample rate generator       */
                    instHandle->txSrgEnable = FALSE;
                }
            }
            else
            {
                /* external mode clock enabled                                */
                instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_CLKXM_MASK);

                /* sample rate generator is enabled                           */
                instHandle->txSrgEnable = TRUE;
            }
        }
    }

    /******************** Command to set the frame sycn mode*******************/
    else if (Mcbsp_IOCTL_SET_FRMSYNCMODE == cmd)
    {
        /* Reciever Bit Clock mode                                            */
        if (MCBSP_MODE_INPUT == chanHandle->mode)
        {
            /* check if the frame sync generator is to be enabled for section */
            if (Mcbsp_FsClkMode_INTERNAL == (Mcbsp_FsClkMode)(*(Uint8 *)arg))
            {
                /* set the frame sync generation mode                         */
                instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_FSRM_MASK);

                /* frame sync generator needs to be enabled                   */
                instHandle->rxFsgEnable = TRUE;
            }
            else
            {
                /* reset the frame sync generation mode                       */
                instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_FSRM_MASK);

                /* frame sync generator needs to be disabled                  */
                instHandle->rxFsgEnable = FALSE;
            }
        }
        else
        {
            /* check if the frame sync generator is to be enabled for section */
            if (Mcbsp_FsClkMode_EXTERNAL == (Mcbsp_FsClkMode)(*(Uint8 *)arg))
            {
                /* External frame sync to be used                             */
                instHandle->hwInfo.regs->PCR &= (~CSL_MCBSP_PCR_FSXM_MASK);

                /* frame sync generator needs to be disabled                  */
                instHandle->txFsgEnable = FALSE;
            }
            else
            {
                /* could be internal or configured for DXR to XSR copy        */
                if (Mcbsp_FsClkMode_INTERNAL == (Mcbsp_FsClkMode)(*(Uint8 *)arg))
                {
                    /* internal frame sync to be used                         */
                    instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_FSXM_MASK);

                    /* set the FSGM bit in the SRGR register                  */
                    instHandle->hwInfo.regs->SRGR |= (CSL_MCBSP_SRGR_FSGM_MASK);

                    /* frame sync generator needs to be enabled               */
                    instHandle->txFsgEnable = TRUE;
                }
                else
                {   /* DXR to XSR copy generates frame sync                   */
                    /* internal frame sync to be used                         */
                    instHandle->hwInfo.regs->PCR |= (CSL_MCBSP_PCR_FSXM_MASK);

                    /* reset the FSGM bit in the SRGR register                */
                    instHandle->hwInfo.regs->SRGR &= (~CSL_MCBSP_SRGR_FSGM_MASK);

                    /* frame sync generator needs to be disabled              */
                    instHandle->txFsgEnable = FALSE;
                }
            }
        }
    }
    /****************** command to configure the sample rate generator*********/
    else if (Mcbsp_IOCTL_CONFIG_SRGR == cmd)
    {
        if (NULL == arg)
        {
            return MCBSP_ERR_BADARGS;
        }

        srgConfig = (Mcbsp_srgConfig *)(arg);

        memcpy(&instHandle->srgrConfig,srgConfig,sizeof(Mcbsp_srgConfig));

        Mcbsp_localConfigureSrgr(instHandle,chanHandle);
    }
    /********************command to reset the channel**************************/
    else if ((Mcbsp_IOCTL_CHAN_RESET == cmd) || (MCBSP_CTRL_CHAN_RESET == cmd))
    {
        Mcbsp_localAbortReset(chanHandle);
    }
    /********************command to reset the device***************************/
    else if ((Mcbsp_IOCTL_DEVICE_RESET == cmd) || (MCBSP_CTRL_DEVICE_RESET == cmd))
    {
        Mcbsp_localAbortReset(&instHandle->rcvObj);
        Mcbsp_localAbortReset(&instHandle->xmtObj);
    }
    /********************command to change the sample rate*********************/    
    else if (Mcbsp_IOCTL_SAMPLE_RATE_CHANGE == cmd)
    {
        status = Mcbsp_localModifySampleRate(chanHandle,arg);        
    }
    /********************command to cancel pending IO**************************/
    else if (Mcbsp_IOCTL_CANCEL_PENDING_IO == cmd)
    {
        Mcbsp_localAbortReset(chanHandle);        
    }
#ifdef MCBSP_LOOPJOB_ENABLE
    /****************command to pause the Mcbsp channel************************/
    else if (Mcbsp_IOCTL_PAUSE == cmd)
    {
        if (FALSE == chanHandle->paused)
        {
            chanHandle->paused = TRUE;
        }
        else
        {
            status = MCBSP_ERR_BADMODE;
        }
    }
    /***************** command to resume the Mcbsp channel*********************/
    else if (Mcbsp_IOCTL_RESUME ==  cmd)
    {
        if (TRUE == Mcbsp_osalQueueEmpty(chanHandle->ptrQFloatList))
        {
            for (linkCnt = 0;linkCnt < Mcbsp_MAXLINKCNT;linkCnt++)
            {
                if (FALSE == Mcbsp_osalQueueEmpty(chanHandle->ptrQPendList))
                {
                    ioBuf = (Mcbsp_IOBuf *)Mcbsp_osalQueueGet(chanHandle->ptrQPendList);

                    if (NULL == ioBuf)
                    {
                        return MCBSP_ERR_BADARGS;
                    }

                    /* floating queue not full . Process this request.        */
                    Mcbsp_osalQueuePut(chanHandle->ptrQFloatList, (void*)ioBuf);

                    chanHandle->currentDataSize = (uint16_t)ioBuf->size;
                    chanHandle->userDataBufferSize = (uint32_t)ioBuf->size;

                    /* for DMA mode we will not use chanHandke->dataIOBuf    *
                     * to hold current buffer - we use floatingqueue          */
                    chanHandle->dataIOBuf = NULL;

                    if ((Mcbsp_MAXLINKCNT - 1u) == linkCnt)
                    {
                        /* Though we have to post to param set directly       *
                         * from here,there will be differene between first    *
                         * such buffer and second buffer. As we have control  *
                         * here we are second buffer and first buffer has not *
                         * yet returned (or corresponding edma callback has   *
                         * not been called.For second buffer, we will be      *
                         * updating the second param set, which is currently  *
                         * hosting loopjob parameter. Hence increment the     *
                         * index to point second paramset and since we are    *
                         * moving out loopjob from both param sets, the       *
                         * oopjobUpdatedinParamset is reset                   */
                        chanHandle->loopjobUpdatedinParamset =FALSE;
                        Mcbsp_localGetNextIndex(
                            &chanHandle->nextLinkParamSetToBeUpdated);
                    }

                   /* Now update the data buffer to the link params. The      *
                    * paramset to be updated is decidec by the                *
                    * "nextLinkParamSetToBeUpdated"                           */
                    if (MCBSP_STATUS_COMPLETED !=
                        Mcbsp_localUpdtDtPktToLnkPrms(chanHandle,ioBuf))
                    {
                        status = MCBSP_ERR_BADIO;
                        break;
                    }

                    if (0 == linkCnt)
                    {
                        /* if at all this is the very first buffer, then      *
                         * one param set has loop job loaded , self linked    *
                         * and active with the main xfer channel param.       *
                         * other param set is ready loaded (just now and      *
                         * has link paramater set as the one having loopjob   *
                         * (this is to ensure that if at all we are not       *
                         * getting any more buffers loopjob be will taken     *
                         * over). Now we have to link the floating / newly    *
                         * loaded param set to xfer channel.                  */
                        status = EDMA3_DRV_linkChannel(
                                     chanHandle->edmaHandle,
                                     chanHandle->xferChan,
                                     chanHandle->pramTbl[chanHandle->nextLinkParamSetToBeUpdated]);

                        if (MCBSP_STATUS_COMPLETED != status)
                        {
                            status = MCBSP_ERR_BADIO;
                            break;
                        }
                        else
                        {
                            EDMA3_DRV_enableTransfer(
                                chanHandle->edmaHandle,
                                chanHandle->xferChan,
                                EDMA3_DRV_TRIG_MODE_EVENT);
                        }
                    }
                }
            }
        }
        chanHandle->paused = FALSE;
    }
    /********************command to Mute the Mcbsp section*********************/
    else if (Mcbsp_IOCTL_MUTE_ON == cmd)
    {
        if (MCBSP_MODE_OUTPUT == chanHandle->mode)
        {
            if(TRUE == chanHandle->bMuteON)
            {
                status = MCBSP_ERR_BADMODE;
            }
            else
            {
                chanHandle->bMuteON = (Bool)TRUE;
            }
        }
        else
        {
            status = MCBSP_ERR_BADMODE;
        }
    }
    /********************command to UnMute the Mcbsp section*******************/
    else if (Mcbsp_IOCTL_MUTE_OFF == cmd)
    {
        /* Take out from the the Muted state                                  */
        if (MCBSP_MODE_OUTPUT == chanHandle->mode)
        {
            /* Check if mute is not ON                                        */
            if (FALSE == chanHandle->bMuteON)
            {
                status = MCBSP_ERR_BADARGS;
            }
            else
            {
                chanHandle->bMuteON =(Bool)FALSE;
            }
        }
        else
        {
            status = MCBSP_ERR_NOTIMPL;
        }
    }
    /********************command to Modify the loop job buffer*****************/
    else if (Mcbsp_IOCTL_MODIFY_LOOPJOB == cmd)
    {
        params = (Mcbsp_ChanParams *)arg;
        if (NULL == params)
        {
            return MCBSP_ERR_BADARGS;
        }

        if (NULL == params->userLoopJobBuffer)
        {
            if (MCBSP_MODE_OUTPUT == chanHandle->mode)
            {
                chanHandle->loopJobBuffer = &Mcbsp_loopSrcBuf[instHandle->instNum].scratchBuffer;
            }
            else
            {
                chanHandle->loopJobBuffer = &Mcbsp_loopDstBuf[instHandle->instNum].scratchBuffer;
            }
            chanHandle->userLoopJob = FALSE;
        }
        else
        {
            chanHandle->loopJobBuffer = params->userLoopJobBuffer;

            if (chanHandle->roundedWordWidth > params->userLoopJobLength)
            {
                chanHandle->loopJobLength = chanHandle->roundedWordWidth;
            }
            else
            {
                chanHandle->loopJobLength = params->userLoopJobLength;
                chanHandle->userLoopJobLength = params->userLoopJobLength;
                chanHandle->userLoopJob = TRUE;
            }
        }
    }
#endif
#ifndef MCBSP_LOOPJOB_ENABLE
    /*****************command to modify the Timeout count value****************/
    else if (Mcbsp_IOCTL_SET_TIMEOUT == cmd)
    {
        if (NULL == arg)
        {
            return MCBSP_ERR_BADARGS;
        }

        /* modify the retry count value to be used                            */
        instHandle->retryCount = *(uint32_t *)arg;
    }
#endif
    else
    {
        status = MCBSP_ERR_NOTIMPL;
    }
    return (status);
}

/*============================================================================*/
/*                              END OF FILE                                   */
/*============================================================================*/
