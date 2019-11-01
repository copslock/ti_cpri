/*
 * mcbsp_edma.c
 *
 * This file contains local functions for MCBSP driver which contain implemen-
 * tation for EDMA operation specifc calls like starting an EDMA transfer for
 * for McBSP peripheral, EDMA completion and/or error callbacks etc.
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

/* CSL-RLs required for MCBSP LLD */
#include <ti/csl/cslr_tpcc.h> // Freon only has cslr directory

/* MCBSP Types includes */
#include <mcbsp_types.h>

/* MCBSP LLD includes */
#include <ti/drv/mcbsp/mcbsp_drv.h>
#include <ti/drv/mcbsp/include/mcbsp_pvt.h>

/* MCBSP OSAL layer */
#include <mcbsp_osal.h>



/*============================================================================*/
/*                            IMPORTED VARIABLES                              */
/*============================================================================*/

extern far Mcbsp_TempBuffer Mcbsp_muteBuf[CSL_MCBSP_PER_CNT];

/*============================================================================*/
/*                          LOCAL FUNCTION PROTOTYPES                         */
/*============================================================================*/

int32_t mcbspIsValidIOBuf(Mcbsp_ChannelObj *chanHandle);

/*============================================================================*/
/*                             LOCAL VARIABLES                                */
/*============================================================================*/

#ifndef MCBSP_LOOPJOB_ENABLE
uint32_t Mcbsp_edmaChanNum;
uint32_t Mcbsp_dummyParamSetAddr;

/* Edma dummy function*/
void doNothing(uint32_t tcc, EDMA3_RM_TccStatus edmaStatus, void* appData)
{
/* DO NOTHING (This is a callback for the dummy transfer)                     */
}
#endif
/* ========================================================================== */
/*                          FUNCTION DEFINITIONS                              */
/* ========================================================================== */

/**
 * \brief   Function to intialize and set up the edma for transfer.
 *
 *          This function does the following
 *          1) Requests the EDMA channel from the edma driver.
 *          2) Requests the link channels for linking (exclusive for tx & rx)
 *          3) set up the param set of main xfer channel (for loopjob transfers)
 *          4) set up the param sets of all link channel (for loopjob transfers)
 *          5) link the last link channel to itself (this channel will be linked
 *             to main xfer channel later and to supply the loopjob xfer
 *             information infinely this is done so;
 *          other link channels are setup here just for completion sake
 *
 * \param   chanHandle   [IN] Handle to the channel for which the edma is setup
 *
 * \return  status       MCBSP_STATUS_COMPLETED if is sucess
 *                       error id in case of failure
 *
 */
int32_t Mcbsp_localSetupEdmaDuringOpen(Mcbsp_ChannelObj *chanHandle)
{
    uint32_t              reqTcc      = EDMA3_DRV_TCC_ANY;
    Bool                falsewhile  = TRUE;
    uint32_t              linkCnt     = 0;
    uint32_t              edmaChanNum = 0;
    uint32_t              count       = 0;
    int32_t               status      = MCBSP_STATUS_COMPLETED;
#ifndef MCBSP_LOOPJOB_ENABLE
    EDMA3_RM_EventQueue queueNum    = 0;
#endif    
    

    do
    {
        falsewhile  = FALSE;
        if (NULL == chanHandle)
        {
            return MCBSP_ERR_BADARGS;
        }

        /* request the transfer channel from the EDMA driver                  */
        status = EDMA3_DRV_requestChannel(
                     chanHandle->edmaHandle,
                     &chanHandle->xferChan,
                     &chanHandle->tcc,
                     chanHandle->edmaEventQue,
                     (chanHandle->edmaCallback),
                     chanHandle);

        if (MCBSP_STATUS_COMPLETED != status)
        {
            status = MCBSP_ERR_BADARGS;
            break;
        }

        /* if there are any errors set in the EDMA clear them here            */
        EDMA3_DRV_clearErrorBits(
            chanHandle->edmaHandle,
            chanHandle->xferChan);

        /* Acquire the  PaRAM entries used for EDMA transfers linking         */
        for (count = 0; count < Mcbsp_MAXLINKCNT; count++)
        {
            /* For requesting for a PaRam set                                 */
            edmaChanNum = EDMA3_DRV_LINK_CHANNEL;
            reqTcc = EDMA3_DRV_TCC_ANY;

            status = EDMA3_DRV_requestChannel(
                        chanHandle->edmaHandle,
                        &edmaChanNum,
                        &reqTcc,
                        chanHandle->edmaEventQue,
                        (chanHandle->edmaCallback),
                        NULL);

            if (MCBSP_STATUS_COMPLETED == status)
            {
                chanHandle->pramTbl[count] = edmaChanNum;

                status = EDMA3_DRV_getPaRAMPhyAddr(
                            chanHandle->edmaHandle,
                            chanHandle->pramTbl[count],
                            &(chanHandle->pramTblAddr[count]));
            }

            if (MCBSP_STATUS_COMPLETED != status )
            {
                /* free the Synchonized EDMA channel                          */
                EDMA3_DRV_freeChannel(
                    chanHandle->edmaHandle,
                    chanHandle->xferChan);

                /* free the already allocated PaRAM entries                   */
                for (linkCnt = 0; linkCnt < count; linkCnt++)
                {
                    EDMA3_DRV_freeChannel(
                        chanHandle->edmaHandle,
                        chanHandle->pramTbl[linkCnt]);
                }

                status = MCBSP_ERR_BADARGS;
                break;
            }
        }
        
#ifndef MCBSP_LOOPJOB_ENABLE
		/* Configure for dummy PaRAM set*/
        Mcbsp_edmaChanNum = EDMA3_DRV_LINK_CHANNEL;
        reqTcc 			  = EDMA3_DRV_TCC_ANY;

        status = EDMA3_DRV_requestChannel(
                    chanHandle->edmaHandle,
                    &Mcbsp_edmaChanNum,
                    &reqTcc,
                    queueNum,
                    &doNothing,
                    NULL);

        if (MCBSP_STATUS_COMPLETED == status)
        {
            status = EDMA3_DRV_getPaRAMPhyAddr(
                        chanHandle->edmaHandle,
                        Mcbsp_edmaChanNum,
                        &Mcbsp_dummyParamSetAddr);
        }
#endif        

        /* Configure EDMA channels                                            */
        if (MCBSP_STATUS_COMPLETED == status)
        {
            status = Mcbsp_localEdmaChanPaRAMsetup(chanHandle);
        }
    }while(falsewhile);
    return status;
}
#ifdef MCBSP_LOOPJOB_ENABLE
/**
 *  \brief   This calls the registered application callback for the current
 *           receive request and processes the next request in queue.
 *           This is a mcbsp edma callback function called in edma context
 *
 *  \param   tcc        [IN]  tcc number of EDMA
 *  \param   edmaStatus [IN]  status of EDMA transaction
 *  \param   data       [IN]  paramaters sent to EDMA ISR (chan handler)
 *
 *  \return  Nothing
 *
 *  \entry   tcc         should be a valid value
 *           status      should be a valid value
 *           data        should be a non NULL and valid pointer
 *
 *  \leave   Not implemented
 */
void Mcbsp_localEdmaCallback(uint32_t tcc, EDMA3_RM_TccStatus edmaStatus, void* data)
{
    Mcbsp_ChannelObj      *chanHandle = NULL;
    EDMA3_DRV_PaRAMRegs    pramTbl    = {0};
    int32_t                status     = MCBSP_STATUS_COMPLETED;
    Mcbsp_IOBuf           *ioBuf      = NULL;
    void*                  criticalSectionInfo;

    chanHandle = (Mcbsp_ChannelObj *)data;
    /* to remove the compiler warning                                         */
    tcc= tcc;

    /* critical section starts                                                */
    criticalSectionInfo = Mcbsp_osalEnterMultipleCoreCriticalSection();
    
    if (TRUE != Mcbsp_osalQueueEmpty(chanHandle->ptrQFloatList))
    {
        /* should always have a buffer present because this callback is       *
         * due to that buffer                                                 */

        /* Get the buffer from the top of the queue (atommic operation)       */
        chanHandle->tempIOBuf = Mcbsp_osalQueueGet(chanHandle->ptrQFloatList);

        /* get the param table information of transfer channel                */
        EDMA3_DRV_getPaRAM(chanHandle->edmaHandle,chanHandle->xferChan,&pramTbl);

        /* Handle the I/O buffers appropriately in case of an breakpoint      *
         * in case of an breakpoint.either of the buffers (2 link param Pkts) *
         * could have caused a callback as both of them as linkedto each other*
         * Hence we will handle that condition here                           */
        if (chanHandle->mode == MCBSP_MODE_INPUT)
        {
            /* Check if destination address falls into the range of 1st req   *
             * in the floating queue.                                         */
            if (((pramTbl.destAddr >= (uint32_t)chanHandle->tempIOBuf->addr)
                && (pramTbl.destAddr < (uint32_t)chanHandle->tempIOBuf->addr
                        + chanHandle->tempIOBuf->size)) &&
					(FALSE == Mcbsp_osalQueueEmpty(chanHandle->ptrQFloatList)))                        
            {
                /* Since we have already dequeue the 1st request, dequeue     *
                 * 2nd io request from floating queue                         */
                ioBuf = (Mcbsp_IOBuf *)Mcbsp_osalQueueGet(chanHandle->ptrQFloatList);

                /* Queue the tempIOBuf (i.e. 1st io request) as this pkt     *
                 * should be first in a queue                                 */
                Mcbsp_osalQueuePut(
                    chanHandle->ptrQFloatList,
                    (void*)chanHandle->tempIOBuf);

                /* Queue the ioBuf i.e. 2nd request in floating queue      */
                Mcbsp_osalQueuePut(chanHandle->ptrQFloatList,(void*)ioBuf);
            }
        }
        else
        {
            /* Check if destination address falls into the range of1st request*
             * in the floating queue.                                         */
            if (((pramTbl.srcAddr >= (uint32_t)chanHandle->tempIOBuf->addr)&& 
                (pramTbl.srcAddr < (uint32_t)chanHandle->tempIOBuf->addr + chanHandle->tempIOBuf->size)) && 
                        (FALSE == Mcbsp_osalQueueEmpty(chanHandle->ptrQFloatList)))
            {
                /* Since we have already dequeue the 1st request, dequeue     *
                 * io request from floating queue                             */
                ioBuf = (Mcbsp_IOBuf *)Mcbsp_osalQueueGet(chanHandle->ptrQFloatList);

                /* Queue the tempIOBuf (i.e. 1st io request) as this         *
                 * buffer should be first in a queue                         */
                Mcbsp_osalQueuePut(
                    chanHandle->ptrQFloatList,
                    (void*)chanHandle->tempIOBuf);

                /* Queue the ioBuf i.e. 2nd request in floating queue      */
                Mcbsp_osalQueuePut(chanHandle->ptrQFloatList,(void*)ioBuf);
                status = MCBSP_ERR_NOIOBUFFERS;
            }
        }

        if (MCBSP_STATUS_COMPLETED == status)
        {
            if (EDMA3_RM_XFER_COMPLETE != edmaStatus)
            {
                /* update the status of the IOP as error                      */
                chanHandle->tempIOBuf->status = MCBSP_ERR_ABORT;

                /* There is a data transfer error in EDMA transfer. clear the *
                 * Error bits and return                                      */
                EDMA3_DRV_clearErrorBits(
                    chanHandle->edmaHandle,
                    chanHandle->xferChan);
            }
            else
            {
                chanHandle->tempIOBuf->status  = MCBSP_STATUS_COMPLETED;
            }

            chanHandle->submitCount--;
        }
#ifdef MCBSP_LOOP_PING_PONG
        if(MCBSP_STATUS_COMPLETED == status)
        {
        	/* Put back the buffer in the queue */
            chanHandle->submitCount++;
        	/* Queue the tempIOBuf                         */
        	Mcbsp_osalQueuePut(
        	    chanHandle->ptrQFloatList,
        	    (void*)chanHandle->tempIOBuf);
        }
#endif

        if (MCBSP_STATUS_COMPLETED == status)
        {
#ifndef MCBSP_LOOP_PING_PONG
            /* if the channel is not in a pause mode                          */
            if (FALSE == chanHandle->paused)
            {
                /* Now that we have done with the last data buffer - we check *
                 * if next buffer is available for transmission. Even if we   *
                 * are not having a valid buffer - previous programming of    *
                 * linkchannels (designed in such a way) to send out null     *
                 * buffer Also note that Mcbsp_localIsValidIOBuf function     *
                 * will set the chanHandle->dataIOBuf with next (current now) *
                 * data buffer from queue                                     */
                if ((Mcbsp_STATUS_VALID == (mcbspIsValidIOBuf(chanHandle))))
                {
                    /* This is a valid data request. Call function to process *
                     * it. If DMA cannot be programmed properly with request  *
                     * contents, abort all queued up requests and put state   *
                     * M/C into reset.                                        */
                    status = Mcbsp_localEdmaProcessPkt(
                                 chanHandle,
                                 chanHandle->dataIOBuf);

                    if (MCBSP_STATUS_COMPLETED != status)
                    {
                        Mcbsp_localCancelAndAbortAllIo(chanHandle);
                    }
                }
                else
                {

                    Mcbsp_localGetNextIndex(
                        &chanHandle->nextLinkParamSetToBeUpdated);

                   /* The following function will assign loopjob buffer to    *
                    * free param set Also the control will come here two times*
                    * for last buffer and last but one buffer (obviously for  *
                    * more than one buffer xfer) For last but buffer callback *
                    * itself we will come to know that there is no more buffer *
                    * pending and hence we load loopjob info into next param  *
                    * set and for the last buffer callback will not do it     *
                    * again (for only one buffer scenario the following flag  *
                    * will take care not to update the loopjob second time)   */
                    if (FALSE  == chanHandle->loopjobUpdatedinParamset)
                    {
                        chanHandle->loopjobUpdatedinParamset = TRUE;

                        Mcbsp_localUpdtDtPktToLnkPrms(chanHandle,NULL);
                    }
                }
            }
            else
#else
            if (TRUE == chanHandle->paused)
#endif
            {
                /* Control will come over here when either of receive or      *
                 * transmit state  machine stop command is issued for receive *
                 * or transmit channel. If the nextFlag is already set        *
                 * indicates that its time to reset the state machines and    *
                 * disable the edma transfer                                  */
                if (TRUE == chanHandle->nextFlag)
                {
                    chanHandle->nextFlag = FALSE;
                }
                else
                {
                    /* For the first time when the stop port command is issued*
                     * we will have one more buffer linked with the transfer  *
                     * channel. So we will not stop at this time. We link the *
                     * buffer with NULL loopjob buffer and set a nextFlag to  *
                     * TRUE. We will wait for another callback(indicating the *
                     * io request is complete) to disable EDMA transfer and   *
                     * reset state  machines                                  */
                    chanHandle->nextFlag = TRUE;

                    Mcbsp_localGetNextIndex(&chanHandle->nextLinkParamSetToBeUpdated);

                    Mcbsp_localUpdtDtPktToLnkPrms(chanHandle, NULL);
                }
            }
            if (MCBSP_STATUS_COMPLETED == status)
            {
                chanHandle->isTempIOBufValid = TRUE;
                Mcbsp_localCompleteCurrentIo(chanHandle);
            }
        }
        else if (MCBSP_ERR_NOIOBUFFERS == status)
        {
            /* Handle Underflow condition */
            chanHandle->isTempIOBufValid = TRUE;
            chanHandle->tempIOBuf = NULL;
            Mcbsp_localCompleteCurrentIo(chanHandle);
        }
    }
    /* critical section ends                                          */
    Mcbsp_osalExitMultipleCoreCriticalSection(criticalSectionInfo);

}

/*!
 *  \brief  This function updates the link param set accordingly with data or
 *          loopjobbuffer. This uses properly maintained index to identify to
 *          what param set the info has to be updated.
 *
 *  \param  chanHandle  [IN]  Handle to the mcbsp channel
 *  \param  ioBuf       [IN]  Pointer to I/O buffer
 *
 *  \return MCBSP_STATUS_COMPLETED in case of sucess
 *          else Error code in case of failure
 *
 *  \entry  chanHandle  should be non NULL and valid pointer
 *          ioBuf       should be non NULL and valid pointer
 *
 *  \leave  Not implemented
 */
int32_t Mcbsp_localUpdtDtPktToLnkPrms(Mcbsp_ChannelObj *chanHandle,
                                    Mcbsp_IOBuf *const ioBuf)
{
    Mcbsp_Object_Unpadded *instHandle  = NULL;
    EDMA3_DRV_PaRAMRegs    pramPtr     = {0};
    Bool                   falsewhile  = TRUE;
    int32_t                status      = MCBSP_STATUS_COMPLETED;
    EDMA3_DRV_SyncType     syncType;

    do
    {
        falsewhile = FALSE;
        if ((NULL == chanHandle) || (NULL == chanHandle->devHandle))
        {
            return MCBSP_ERR_BADARGS;
        }

        instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);
        if (NULL == instHandle)
        {
            return MCBSP_ERR_BADARGS;
        }

        /* if at all this is the first buffer we will update the main params  *
         * else we will update the link params                                */
        status = EDMA3_DRV_getPaRAM(
                    chanHandle->edmaHandle,
                    chanHandle->pramTbl[chanHandle->nextLinkParamSetToBeUpdated],
                    &pramPtr);
        if (MCBSP_STATUS_COMPLETED != status )
        {
            status = MCBSP_ERR_BADARGS;
            break;
        }

    /* Following the explanation behind the EDMA configuration                *
     * 1) We will have n arrays of data (now it is data buffer each of length *
     *    databuffersize bytes)                                               *
     * 2) for each sync event we have to fetch the data from each array and   *
     *    write to each slot                                                  *
     * 3) and the no. bytes transferred in this case from each array will be  *
     *    roundedwordwidth/4                                                  *
     * 4) as the loopjob buffer is contigous, we configure the counts as      *
     *    follows                                                             *
     *    a) consider there are arrays of length roundedwordwidth/4 (aCnt)    *
     *    b) these arrays are located apart databuffersize size  (bindex)     *
     *    c) there are 'no.of.slots' no. of such arrays   (bCnt)              *
     *    d) (a) and (b) constitute one frame of EDMA transfer and it happens *
     *       for one sync event in AB sync mode.                              *
     *    e) such frames have to be sent till we have buffer - ie             *
     *       databuffersize / (roundedwordwidth/8 ) and this constitutes  cCnt*
     *    f) address of buffer (rather offset from start of previous frame)   *
     *       is roundedwordwidth/8  and this forms cIndex                     *
     *                                                                        *
     *  Please take some time to visualise the setup mentioned above to check *
     *  with EDMA configuration                                               *
     *  Also we change only specific parameters because others will not have  *
     *  change from what we programed  intially                               */

    /* control will reach this function is two contexts                       *
     * 1) From submitreq function for first time buffers                      *
     * 2) From EDMA callback for further handling of buffers in Queue         *
     *    - we are from EDMA callback becuase we have completed some user     *
     *   data buffer just now                                                 */

        if (NULL != ioBuf)
        {
            /* 1) when control comes to this function from submitreq control  *
             *    will surely comeinto the block                              *
             * 2) When control comes to this function from EDMA callback,     *
             *    control will some only if there is fuirther more buffers to *
             *     handle                                                     */
            if (MCBSP_MODE_INPUT == chanHandle->mode)
            {
                pramPtr.destAddr = (uint32_t)ioBuf->addr;
                Mcbsp_localGetIndicesSyncType(
                    chanHandle,
                    &(pramPtr.destBIdx),
                    &(pramPtr.destCIdx),
                    &(pramPtr.aCnt),
                    &(pramPtr.bCnt),
                    &(pramPtr.cCnt),
                    &syncType,
                    FALSE);
            }
            else
            {
                if (TRUE == chanHandle->bMuteON)
                {
                    /* Mute is set hence we need to switch to the mute buffer */
                    pramPtr.srcAddr =
                        (uint32_t)&Mcbsp_muteBuf[instHandle->instNum].scratchBuffer;
                    Mcbsp_localGetIndicesSyncType(
                        chanHandle,
                        &(pramPtr.srcBIdx),
                        &(pramPtr.srcCIdx),
                        &(pramPtr.aCnt),
                        &(pramPtr.bCnt),
                        &(pramPtr.cCnt),
                        &syncType,
                        TRUE);
                }
                else
                {
                    pramPtr.srcAddr = (uint32_t)ioBuf->addr;
                    Mcbsp_localGetIndicesSyncType(
                        chanHandle,
                        &(pramPtr.srcBIdx),
                        &(pramPtr.srcCIdx),
                        &(pramPtr.aCnt),
                        &(pramPtr.bCnt),
                        &(pramPtr.cCnt),
                        &syncType,
                        FALSE);
                }
            }

            /* Here we are enabling the interrupt for this param set info     *
             * we do it here in this way because using int enable API after   *
             * calling EDMA3_DRV_setPaRAM, may fail our intention. This is    *
             * because immediately after the set param the paramset might be  *
             * copied to linked param and will make the consecutive int enable*
             * (set opt field) api useless for our purpose                    */
            pramPtr.opt |= (0x01u<<20u);

            /* we are loading data request in the param set. As per our design*
             * when we load the data,  we should link this param set with next*
             * one most probably                                              *
             *  a) That may have loopjob loaded                               *
             *  b) it may be the currently loaded set, which will be updated  *
             *     in next edma callback.                                     *
             * hence in both cases to be run next to this data request        */
            pramPtr.linkAddr =
              (uint16_t)(chanHandle->pramTblAddr[ \
              ((chanHandle->nextLinkParamSetToBeUpdated+1) & 0x01)] & 0x0000FFFF);
        }
        else
        {
            /* we have reached here becuase we dont have any more buffer to  *
             * submit for Xfer assign loop job to params- no change in linking*
             * IMP: The following assignment will be skipped if already both  *
             * params are having  loopjob buffer                              */
            if (MCBSP_MODE_INPUT == chanHandle->mode)
            {
                pramPtr.destAddr = (uint32_t)chanHandle->loopJobBuffer;
                Mcbsp_localGetIndicesSyncType(
                    chanHandle,
                    &(pramPtr.destBIdx),
                    &(pramPtr.destCIdx),
                    &(pramPtr.aCnt),
                    &(pramPtr.bCnt),
                    &(pramPtr.cCnt),
                    &syncType,
                    TRUE);
            }
            else
            {
                pramPtr.srcAddr = (uint32_t)chanHandle->loopJobBuffer;
                Mcbsp_localGetIndicesSyncType(
                    chanHandle,
                    &(pramPtr.srcBIdx),
                    &(pramPtr.srcCIdx),
                    &(pramPtr.aCnt),
                    &(pramPtr.bCnt),
                    &(pramPtr.cCnt),
                    &syncType,
                    TRUE);
            }

            /* we are loading loopjob into this param.  That means we have    *
             * reached the end of buffers inpending and floating queues. So   *
             * this loopjob had to be  linked to itself. Also we need to point*
             * the index to other param set in order to load data request     *
             * directly from submit call                                      */
            pramPtr.linkAddr =
                (uint16_t)(chanHandle->pramTblAddr[    \
                chanHandle->nextLinkParamSetToBeUpdated] & 0x0000FFFF);

            pramPtr.opt &= (~(0x01<<20));
        }

        /* Update the changed param set info into the appropriate paramset    */
        status = EDMA3_DRV_setPaRAM(
                    chanHandle->edmaHandle,
                    chanHandle->pramTbl[chanHandle->nextLinkParamSetToBeUpdated],
                    &pramPtr);
        if (MCBSP_STATUS_COMPLETED != status)
        {
            status = MCBSP_ERR_BADARGS;
            break;
        }

        if (NULL == ioBuf)
        {
            /* we are loading loopjob into this param. we need to point the   *
             * index to other param set in order to load data request directly*
             * from submit call                                               */
            Mcbsp_localGetNextIndex(&chanHandle->nextLinkParamSetToBeUpdated);
        }
    }while(falsewhile);

    return status;
}

/**
 * \brief   setup the EDMA channel for the specific channel requested.
 *
 *          This function configures the McBSP synchonized EDMA channel PaRAM
 *          entries it also configures the various other parameters like
 *          source and destination address / modes
 *
 * \param   chanHandle   [IN]   Channel handle
 *
 * \return  MCBSP_STATUS_COMPLETED  if setup successful
 *          Appropriate MCBSP driver error code if configuration has any error
 */
int32_t Mcbsp_localEdmaChanPaRAMsetup(Mcbsp_ChannelObj *chanHandle)
{
    uint32_t               linkCnt    = 0;
    Mcbsp_Object_Unpadded *instHandle = NULL;
    EDMA3_DRV_PaRAMRegs  paramSet   = {0,0,0,0,0,0,0,0,0,0,0,0};
    int32_t              status     = MCBSP_STATUS_COMPLETED;
    EDMA3_DRV_SyncType   tempSyncType;

    if ((NULL == chanHandle) || (NULL == chanHandle->devHandle))
    {
        return MCBSP_ERR_BADARGS;
    }

    instHandle = &(chanHandle->devHandle->obj);
    if (NULL == instHandle)
    {
        return MCBSP_ERR_BADARGS;
    }

    /* Configure the McBSP synchonized EDMA channel PaRAM entries             *
     * Configure source and destination address / modes                       *
     * first - main xfer channel                                              */

    /* following is the explanation behind the EDMA configuration             *
     *
     *  1) We will have n arrays of data (now it is loopjob buffer each of    *
     *     length x bytes)                                                    *
     *  2) for each sync event we have to fetch the data from each array      *
     *     and write to each serilaiser                                       *
     *  3) and the no. bytes transferred in this case from each array will    *
     *     be roundedwordwidth/4                                              *
     *  4) as the loopjob buffer is continous, we configure the counts as     *
     *     follows                                                            *
     *     a) consider there are arrays of length roundedwordwidth/8(aCnt)    *
     *     b) these arrays are located apart lengthofloopjob size (bindex)    *
     *     c) there are 'no.of.slots' no. of such arrays  (bCnt)              *
     *     d) (a) and (b) constitute one frame of EDMA transfer and it        *
     *        happens for one sync event in AB sync mode                      *
     *     e) such frames have to be sent till we have buffer - ie            *
     *        lengthofloopjob / (roundedwordwidth/8 )  and this               *
     *        constitutes  cCnt                                               *
     *     f) address of buffer (rather offset from start of previous         *
     *         frame) is roundedwordwidth/8 and this forms cIndex             *
     *  Please take some time to visualise the setup mentioned above to       *
     *  check with EDMA configuration The same kind of setup will be used     *
     *  for data transfers also and the change will be buffer pointer         *
     *  and length of data buffer.                                            */

    /* Get the PaRAM set for default parameters                               */
    EDMA3_DRV_getPaRAM(chanHandle->edmaHandle,chanHandle->xferChan,&paramSet);

    if (MCBSP_MODE_INPUT == chanHandle->mode)
    {
        /* Configuring Source and Dest addresses                              */
        if (TRUE == chanHandle->enableHwFifo)
        {
            paramSet.srcAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
        }
        else
        {
            paramSet.srcAddr = (uint32_t)&(instHandle->hwInfo.regs->DRR);
        }
#ifdef _BIG_ENDIAN
        paramSet.srcAddr += (4 - chanHandle->roundedWordWidth);
#endif
        paramSet.destAddr = (unsigned int)(chanHandle->loopJobBuffer);

        /* Configuring Src and Dest B&C Indexes                               */
        paramSet.srcBIdx  = 0;
        paramSet.srcCIdx  = 0;

        if (MCBSP_STATUS_COMPLETED != Mcbsp_localGetIndicesSyncType(
                                 chanHandle,
                                 &paramSet.destBIdx,
                                 &paramSet.destCIdx,
                                 &paramSet.aCnt,
                                 &paramSet.bCnt,
                                 &paramSet.cCnt,
                                 &tempSyncType,
                                 TRUE))
        {
            status = MCBSP_ERR_BADARGS;
        }
    }
    else
    {
        /* Configuring Source and Dest addresses                              */
        paramSet.srcAddr  = (unsigned int)(chanHandle->loopJobBuffer);

        if (TRUE == chanHandle->enableHwFifo)
        {
            paramSet.destAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
        }
        else
        {
            paramSet.destAddr = (uint32_t)&(instHandle->hwInfo.regs->DXR);
        }
#ifdef _BIG_ENDIAN
        paramSet.destAddr += (4 - chanHandle->roundedWordWidth);
#endif
        /* Configuring Src and Dest B&C Indexes                               */
        paramSet.destBIdx = 0;
        paramSet.destCIdx = 0;

        if (MCBSP_STATUS_COMPLETED != Mcbsp_localGetIndicesSyncType(
                                 chanHandle,
                                 &paramSet.srcBIdx,
                                 &paramSet.srcCIdx,
                                 &paramSet.aCnt,
                                 &paramSet.bCnt,
                                 &paramSet.cCnt,
                                 &tempSyncType,
                                 TRUE))
        {
            status = MCBSP_ERR_BADARGS;
        }
    }


    /* Configuring bCntReload                                                 */
    paramSet.bCntReload = paramSet.bCnt;

    /* Src & Dest are in INCR modes                                           */
    paramSet.opt &= 0xFFFFFFFCu;

    /* FIFO width is 8 bit                                                    */
    paramSet.opt &= 0xFFFFF8FFu;

    /* Set EDMA3_DRV_OPT_FIELD_TCINTEN to FALSE                               */
    paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_ITCINTEN_MASK));

    paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_TCINTEN_MASK));
    paramSet.opt |= (CSL_TPCC_PARAM_OPT_TCINTEN_DISABLE
                        << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);

    /* Program the TCC                                                        */
    paramSet.opt |= ((chanHandle->tcc << CSL_TPCC_PARAM_OPT_TCC_SHIFT)
                      & CSL_TPCC_PARAM_OPT_TCC_MASK);

    /* EDMA3_DRV_SYNC_AB                                                      */
    paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_SYNCDIM_MASK));
    paramSet.opt |= (tempSyncType << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

    paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_TCCMOD_MASK));
    paramSet.opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);

    /* Now, write the PaRAM Set.                                              */
    status = EDMA3_DRV_setPaRAM(
                  chanHandle->edmaHandle,
                  chanHandle->xferChan,
                  &paramSet);

    if (MCBSP_STATUS_COMPLETED == status)
    {
        /* Configure the link PaRAMs with the appropriate parameters Though   *
         * we configure all the link paramsets, we will be using only one of  *
         * them to link with main xfer channel                                */
        for (linkCnt = 0; linkCnt < Mcbsp_MAXLINKCNT; linkCnt++)
        {
            /* Get the PaRAM set for default parameters                       */
            EDMA3_DRV_getPaRAM (chanHandle->edmaHandle,
                                chanHandle->pramTbl[linkCnt],
                                &paramSet);

            if (MCBSP_MODE_INPUT == chanHandle->mode)
            {
                /* Configuring Source and Dest addresses                      */
                if (TRUE == chanHandle->enableHwFifo)
                {
                    paramSet.srcAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
                }
                else
                {
                    paramSet.srcAddr = (uint32_t)&(instHandle->hwInfo.regs->DRR);
                }
#ifdef _BIG_ENDIAN
                paramSet.srcAddr += (4 - chanHandle->roundedWordWidth);
#endif
                paramSet.destAddr = (uint32_t)(chanHandle->loopJobBuffer);

                /* Configuring Src and Dest B&C Indexes                       */
                paramSet.srcBIdx  = 0;
                paramSet.srcCIdx  = 0;

                if (MCBSP_STATUS_COMPLETED != Mcbsp_localGetIndicesSyncType(
                                         chanHandle,
                                         &paramSet.destBIdx,
                                         &paramSet.destCIdx,
                                         &paramSet.aCnt,
                                         &paramSet.bCnt,
                                         &paramSet.cCnt,
                                         &tempSyncType,
                                         TRUE))
                {
                    status = MCBSP_ERR_BADARGS;
                }
            }
            else
            {
                /* Configuring Source and Dest addresses                      */
                paramSet.srcAddr  = (uint32_t)(chanHandle->loopJobBuffer);

                if (TRUE == chanHandle->enableHwFifo)
                {
                    paramSet.destAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
                }
                else
                {
                    paramSet.destAddr = (uint32_t)&(instHandle->hwInfo.regs->DXR);
                }

#ifdef _BIG_ENDIAN
                paramSet.destAddr += (4 - chanHandle->roundedWordWidth);
#endif

                paramSet.destBIdx  = 0;
                paramSet.destCIdx  = 0;

                if (MCBSP_STATUS_COMPLETED != Mcbsp_localGetIndicesSyncType(
                                         chanHandle,
                                         &paramSet.srcBIdx,
                                         &paramSet.srcCIdx,
                                         &paramSet.aCnt,
                                         &paramSet.bCnt,
                                         &paramSet.cCnt,
                                         &tempSyncType,
                                         TRUE))
                {
                    status = MCBSP_ERR_BADARGS;
                }
            }

            /* Configuring bCntReload                                         */
            paramSet.bCntReload = paramSet.bCnt;

            /* Src & Dest are in INCR modes                                   */
            paramSet.opt &= 0xFFFFFFFCu;

            /* FIFO width is 8 bit                                            */
            paramSet.opt &= 0xFFFFF8FFu;

            /* EDMA3_DRV_SYNC_AB                                              */
            paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_SYNCDIM_MASK));
            paramSet.opt |= (tempSyncType << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

            /* Set EDMA3_DRV_OPT_FIELD_TCINTEN to FALSE                       */
            paramSet.opt |= (CSL_TPCC_PARAM_OPT_TCINTEN_DISABLE
                                << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);

            /* Program the TCC                                                */
            paramSet.opt |= ((chanHandle->tcc << CSL_TPCC_PARAM_OPT_TCC_SHIFT)
                              & CSL_TPCC_PARAM_OPT_TCC_MASK);

            /* early completion interrupt                                     */
            paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_TCCMOD_MASK));
            paramSet.opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);

            paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_TCINTEN_MASK));

            if (MCBSP_STATUS_COMPLETED == status)
            {
                /* Now, write the PaRAM Set.                                  */
                status = EDMA3_DRV_setPaRAM(chanHandle->edmaHandle,
                                            chanHandle->pramTbl[linkCnt],
                                            &paramSet);
                if (MCBSP_STATUS_COMPLETED != status)
                {
                      status = MCBSP_ERR_BADARGS;
                      break;
                }
            }

            if (Mcbsp_MAXLINKCNT - 1 > linkCnt )
            {
                /* link with next paramset both param sets are  linked to each*
                 * other to enable continous xfer - either user audio data or *
                 * loopjob buffer MAXLINKCNT must be 2                        */
                status = EDMA3_DRV_linkChannel(
                             chanHandle->edmaHandle,
                             chanHandle->xferChan,
                             chanHandle->pramTbl[linkCnt]);

                 if (MCBSP_STATUS_COMPLETED != status)
                 {
                     status = MCBSP_ERR_BADARGS;
                     break;
                 }

                 status = EDMA3_DRV_linkChannel(
                             chanHandle->edmaHandle,
                             chanHandle->pramTbl[linkCnt] ,
                             chanHandle->pramTbl[linkCnt]);

                 if (MCBSP_STATUS_COMPLETED != status)
                 {
                     status = MCBSP_ERR_BADARGS;
                     break;
                 }

                 status = EDMA3_DRV_linkChannel(
                             chanHandle->edmaHandle,
                             chanHandle->pramTbl[linkCnt+1] ,
                             chanHandle->pramTbl[linkCnt+1]);

                 if (MCBSP_STATUS_COMPLETED != status)
                 {
                     status = MCBSP_ERR_BADARGS;
                     break;
                 }

                 chanHandle->nextLinkParamSetToBeUpdated = 0;

                 chanHandle->loopjobUpdatedinParamset  = TRUE;
            }
        }/*for loop for linkcount*/
    }
    return (status);
}

#else
/**
 *  \brief   This calls the registered application callback for the current
 *           receive request and processes the next request in queue.
 *           This is a mcbsp edma callback function called in edma context
 *
 *  \param   tcc        [IN]  tcc number of EDMA
 *  \param   edmaStatus [IN]  status of EDMA transaction
 *  \param   data       [IN]  paramaters sent to EDMA ISR (chan handler)
 *
 *  \return  Nothing
 *
 *  \entry   tcc         should be a valid value
 *           status      should be a valid value
 *           data        should be a non NULL and valid pointer
 *
 *  \leave   Not implemented
 */
void Mcbsp_localEdmaCallback(uint32_t tcc, EDMA3_RM_TccStatus edmaStatus, void* data)
{
    Mcbsp_Object_Unpadded *instHandle = NULL;
    Mcbsp_ChannelObj      *chanHandle = NULL;
    int32_t                status     = MCBSP_STATUS_COMPLETED;
    Bool                   falseWhile = TRUE;
    void*                  criticalSectionInfo;

    /* critical section starts                                        */
    criticalSectionInfo = Mcbsp_osalEnterSingleCoreCriticalSection();
    
    do
    {
        falseWhile = FALSE;

        chanHandle = (Mcbsp_ChannelObj *)data;

        if (1u == chanHandle->submitCount)
        {
            /* This is the last buffer available with the driver.Hence we will*
             * stop the EDMA and then proceed to process the buffer           */
            EDMA3_DRV_disableTransfer(
                chanHandle->edmaHandle,
                chanHandle->xferChan,
                EDMA3_DRV_TRIG_MODE_EVENT);

            instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);

            /* close the IOP now                                              */
            if (MCBSP_MODE_OUTPUT == chanHandle->mode)
            {
                /* update the status of the IOP                               */
                if (EDMA3_RM_XFER_COMPLETE != edmaStatus)
                {
                    /* update the status of the IOP as error                  */
                    chanHandle->currentError = MCBSP_ERR_ABORT;
                }
                else
                {
                    chanHandle->currentError = MCBSP_STATUS_COMPLETED;
                }

                Mcbsp_TxFifo((int32_t)&instHandle->xmtObj, (int32_t)instHandle);
                
                /* critical section ends                                      */
                Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);
			    
                break;
            }

            /* Stop the McBSP hardware.                                       *
             * 1. check if the RX has enabled the  FSG and also it is running *
             * 2. Also check that the TX state machine is not running off FSG */
            if (TRUE == instHandle->rxFsgEnable)
            {
                if ((TRUE != instHandle->txFsgEnable) ||
                    (TRUE == instHandle->stopSmFsXmt))
                {
                    Mcbsp_localResetCtrl(
                        (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj),
                            Mcbsp_SpcrCtrl_FSYNC_DISABLE);

                    instHandle->fsgEnabled = FALSE;
                }
            }

            Mcbsp_localResetCtrl(
                (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj),
                 Mcbsp_SpcrCtrl_RX_DISABLE);

            /* receive state machine is stopped                               */
            instHandle->stopSmFsRcv = TRUE;

            /* clear the error bits that have been set                        */
            EDMA3_DRV_clearErrorBits(
                chanHandle->edmaHandle,
                chanHandle->xferChan);            
        }
        else
        {
            /* critical section ends                                          */
            Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);
        }

        /* To remove the compiler warning                                     */
        tcc = tcc;

        if (EDMA3_RM_XFER_COMPLETE != edmaStatus)
        {
            /* clear the error bits that have been set                        */
            EDMA3_DRV_clearErrorBits(
                chanHandle->edmaHandle,
                chanHandle->xferChan);

            /* update the status of the IOP as error                          */
            chanHandle->currentError = MCBSP_ERR_ABORT;
            break;
        }
        else
        {
            chanHandle->currentError = MCBSP_STATUS_COMPLETED;

            /* reduce the submit count as the buffer callback is completed    */
            chanHandle->submitCount--;
        }


        if (TRUE == Mcbsp_osalQueueEmpty(chanHandle->ptrQFloatList))
        {
            /* This cannot happen, if it happens then it is a sprurios one    */
            /* critical section ends                                          */
            Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);
            break;
        }

        /* Get the buffer from the top of the queue (atomic operation)        */
        chanHandle->tempIOBuf = (Mcbsp_IOBuf *) Mcbsp_osalQueueGet(chanHandle->ptrQFloatList);


        instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);

        /* check if the STOP command is issued for the channel. if the stop   *
         * command is issued we will not load the next buffer so that the     *
         * state machine is stopped                                           */
        if (((MCBSP_MODE_INPUT == chanHandle->mode) && (FALSE == instHandle->stopSmFsRcv)) ||
            ((MCBSP_MODE_OUTPUT == chanHandle->mode) && (FALSE == instHandle->stopSmFsXmt)))
        {
            /* now check if any other buffer is available and load that   */
            if ((Mcbsp_STATUS_VALID == (mcbspIsValidIOBuf(chanHandle))))
            {
                /* Valid buffer is found,load the buffer in to link params*/
                status = Mcbsp_localEdmaProcessPkt(
                             chanHandle,
                             chanHandle->dataIOBuf);

                if (MCBSP_STATUS_COMPLETED != status)
                {
                    Mcbsp_localCancelAndAbortAllIo(chanHandle);
                }
            }
        }

        /* complete the IO pkt processing by calling the callback function    */
        chanHandle->isTempIOBufValid = TRUE;
        Mcbsp_localCompleteCurrentIo(chanHandle);

        /* critical section ends                                              */
        Mcbsp_osalExitSingleCoreCriticalSection(criticalSectionInfo);
    }while (falseWhile);
    
}

/**
 *  \brief  This function updates the link param set accordingly with data or
 *          loopjobbuffer. This uses properly maintained index to identify to
 *          what param set the info has to be updated.
 *
 *  \param  chanHandle  [IN]  Handle to the mcbsp channel
 *  \param  ioBuf       [IN]  Pointer to I/O buffer
 *
 *  \return MCBSP_STATUS_COMPLETED in case of sucess
 *          else Error code in case of failure
 *
 *  \entry  chanHandle  should be non NULL and valid pointer
 *          ioBuf       should be non NULL and valid pointer
 *
 *  \leave  Not implemented
 */
int32_t Mcbsp_localUpdtDtPktToLnkPrms(Mcbsp_ChannelObj *chanHandle,
                                    Mcbsp_IOBuf       *const ioBuf)
{
    Mcbsp_Object_Unpadded *instHandle  = NULL;
    EDMA3_DRV_PaRAMRegs   pramPtr     = {0};
    Bool                  falsewhile  = TRUE;
    int32_t                 status      = MCBSP_STATUS_COMPLETED;
    EDMA3_DRV_SyncType    syncType;
    EDMA3_DRV_PaRAMRegs   paramSet    = {0,0,0,0,0,0,0,0,0,0,0,0};    

    do
    {
        falsewhile = FALSE;
        if ((NULL == chanHandle) || (NULL == chanHandle->devHandle))
        {
            return MCBSP_ERR_BADARGS;
        }

        instHandle = (Mcbsp_Object_Unpadded *)&(chanHandle->devHandle->obj);
        if (NULL == instHandle)
        {
            return MCBSP_ERR_BADARGS;
        }

        /* if at all this is the first buffer we will update the main params  *
         * else we will update the link params                                */
        if (1u == chanHandle->submitCount)
        {
            status = EDMA3_DRV_getPaRAM(
                         chanHandle->edmaHandle,
                         chanHandle->xferChan,
                         &pramPtr);
        }
        else
        {
            status = EDMA3_DRV_getPaRAM(
                         chanHandle->edmaHandle,
                         chanHandle->pramTbl[chanHandle->nextLinkParamSetToBeUpdated],
                         &pramPtr);
        }

        if (MCBSP_STATUS_COMPLETED != status )
        {
            status = MCBSP_ERR_BADARGS;
            break;
        }

        if (NULL != ioBuf)
        {
            if (MCBSP_MODE_INPUT == chanHandle->mode)
            {
                if (TRUE == chanHandle->enableHwFifo)
                {
                    pramPtr.srcAddr  = (uint32_t)(instHandle->hwInfo.dataAddress);
                }
                else
                {
                    pramPtr.srcAddr = (uint32_t)&(instHandle->hwInfo.regs->DRR);
                }
#ifdef _BIG_ENDIAN
                pramPtr.srcAddr += (4 - chanHandle->roundedWordWidth);
#endif
                pramPtr.destAddr = (uint32_t)ioBuf->addr;
                Mcbsp_localGetIndicesSyncType(
                    chanHandle,
                    &(pramPtr.destBIdx),
                    &(pramPtr.destCIdx),
                    &(pramPtr.aCnt),
                    &(pramPtr.bCnt),
                    &(pramPtr.cCnt),
                    &syncType,
                    FALSE);
            }
            else
            {
                if (TRUE == chanHandle->enableHwFifo)
                {
                    pramPtr.destAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
                }
                else
                {
                    pramPtr.destAddr = (uint32_t)&(instHandle->hwInfo.regs->DXR);
                }
#ifdef _BIG_ENDIAN
                pramPtr.destAddr += (4 - chanHandle->roundedWordWidth);
#endif
                if (TRUE == chanHandle->bMuteON)
                {
                    /* Mute is set hence we need to switch to the mute buffer */
                    pramPtr.srcAddr =
                        (uint32_t)&Mcbsp_muteBuf[instHandle->instNum].scratchBuffer;
                    Mcbsp_localGetIndicesSyncType(
                        chanHandle,
                        &(pramPtr.srcBIdx),
                        &(pramPtr.srcCIdx),
                        &(pramPtr.aCnt),
                        &(pramPtr.bCnt),
                        &(pramPtr.cCnt),
                        &syncType,
                        TRUE);
                }
                else
                {
                    pramPtr.srcAddr = (uint32_t)ioBuf->addr;
                    Mcbsp_localGetIndicesSyncType(
                        chanHandle,
                        &(pramPtr.srcBIdx),
                        &(pramPtr.srcCIdx),
                        &(pramPtr.aCnt),
                        &(pramPtr.bCnt),
                        &(pramPtr.cCnt),
                        &syncType,
                        FALSE);
                }
            }

            /* Here we are enabling the interrupt for this param set info     *
             * we do it here in this way because using int enable API after   *
             * calling EDMA3_DRV_setPaRAM, may fail our intention. This is    *
             * because immediately after the set param the paramset might be  *
             * copied to linked param and will make the consecutive int enable*
             * (set opt field) api useless for our purpose                    */
            pramPtr.opt |= (0x01u<<20u);

            /* early completion interrupt is enabled                          */
            pramPtr.opt &= (~(CSL_TPCC_PARAM_OPT_TCCMOD_MASK));
            pramPtr.opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);

            /* set the link address as 0xFFFF                                 */
            pramPtr.linkAddr = 0xFFFFu;

             if (MCBSP_MODE_OUTPUT == chanHandle->mode)
			{
				status = EDMA3_DRV_getPaRAM (chanHandle->edmaHandle,
                					         Mcbsp_edmaChanNum,
                        					 &paramSet);

    			paramSet.aCnt     = 1u;
    			paramSet.linkAddr = 0xFFFF;
    			paramSet.opt      &= ~(0x01 << 3);

    			if (MCBSP_STATUS_COMPLETED == status)
    			{
				    /* Now, write the dummy PaRAM Set.                                    */
				    status = EDMA3_DRV_setPaRAM(chanHandle->edmaHandle,
				                                Mcbsp_edmaChanNum,
				                                &paramSet);
				}
			}
        }
        else
        {
            status = MCBSP_ERR_BADARGS;
            break;
        }

        if (1u == chanHandle->submitCount)
        {
            /* Configuring bCntReload                                         */
            pramPtr.bCntReload = pramPtr.bCnt;

            /* Src & Dest are in INCR modes                                   */
            pramPtr.opt &= 0xFFFFFFFCu;

            /* FIFO width is 8 bit                                            */
            pramPtr.opt &= 0xFFFFF8FFu;

            /* Set EDMA3_DRV_OPT_FIELD_TCINTEN to TRUE                        */
            pramPtr.opt |= (CSL_TPCC_PARAM_OPT_TCINTEN_ENABLE
                            << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);

            /* Program the TCC                                                */
            pramPtr.opt |= ((chanHandle->tcc << CSL_TPCC_PARAM_OPT_TCC_SHIFT)
                              & CSL_TPCC_PARAM_OPT_TCC_MASK);

            /* EDMA3_DRV_SYNC_AB                                              */
            pramPtr.opt &= (~(CSL_TPCC_PARAM_OPT_SYNCDIM_MASK));
            pramPtr.opt |= (syncType << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

            /* set the link address as dummy address for Tx and 0xFFFF for Rx */
            if (MCBSP_MODE_OUTPUT == chanHandle->mode)
			{
				pramPtr.linkAddr = 
								 (uint16_t)(Mcbsp_dummyParamSetAddr & 0x0000FFFF);
			}
			else
			{
				pramPtr.linkAddr = 0xFFFFu;
			}

            /* early completion interrupt is enabled                          */
            pramPtr.opt &= (~(CSL_TPCC_PARAM_OPT_TCCMOD_MASK));
            pramPtr.opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);

            status = EDMA3_DRV_setPaRAM(
                        chanHandle->edmaHandle,
                        chanHandle->xferChan,
                        &pramPtr);
                        
        }
        else
        {
            status = EDMA3_DRV_setPaRAM(
                         chanHandle->edmaHandle,
                         chanHandle->pramTbl[chanHandle->
                             nextLinkParamSetToBeUpdated],
                         &pramPtr);
        }

        if (2u == chanHandle->submitCount)
        {
            /* link this paramset with the main channel                       */
            status = EDMA3_DRV_linkChannel(
                        chanHandle->edmaHandle,
                        chanHandle->xferChan,
                        chanHandle->pramTbl \
                        [chanHandle->nextLinkParamSetToBeUpdated]);

            if (MCBSP_STATUS_COMPLETED != status)
            {
                status = MCBSP_ERR_BADIO;
                break;
            }
            
            if (MCBSP_MODE_OUTPUT == chanHandle->mode)
            {         
		        /* link the dummy transfer at the end of the link PaRAM           */
		        status = EDMA3_DRV_linkChannel(
		                    chanHandle->edmaHandle,
		                    chanHandle->pramTbl \
		                    [chanHandle->nextLinkParamSetToBeUpdated],
		                    Mcbsp_edmaChanNum);
		
		        if (MCBSP_STATUS_COMPLETED != status)
		        {
		            status = MCBSP_ERR_BADIO;
		            break;
		        }            
	        }
        }

        /* link this paramset with the other paramset                         */
        if (3u == chanHandle->submitCount)
        {
            status = EDMA3_DRV_linkChannel(
                        chanHandle->edmaHandle,
                        chanHandle->pramTbl[
                            (chanHandle->nextLinkParamSetToBeUpdated + 1u) & 0x01u],
                        chanHandle->pramTbl \
                        [chanHandle->nextLinkParamSetToBeUpdated]);

            if (MCBSP_STATUS_COMPLETED != status)
            {
                status = MCBSP_ERR_BADIO;
                break;
            }
            
            if (MCBSP_MODE_OUTPUT == chanHandle->mode)
            {            
	            /* link the dummy transfer at the end of the link PaRAM           */
	            status = EDMA3_DRV_linkChannel(
	                        chanHandle->edmaHandle,
	                        chanHandle->pramTbl \
	                        [chanHandle->nextLinkParamSetToBeUpdated],
	                        Mcbsp_edmaChanNum);
	
	            if (MCBSP_STATUS_COMPLETED != status)
	            {
	                status = MCBSP_ERR_BADIO;
	                break;
	            }
            }
            
        }

        if (MCBSP_STATUS_COMPLETED != status)
        {
            status = MCBSP_ERR_BADARGS;
            break;
        }
    }while(falsewhile);
    return status;
}

/**
 * \brief   setup the EDMA channel for the specific channel requested.
 *
 *          This function configures the McBSP synchonized EDMA channel PaRAM
 *          entries it also configures the various other parameters like
 *          source and destination address / modes
 *
 * \param   chanHandle   [IN]   Channel handle
 *
 * \return  MCBSP_STATUS_COMPLETED  if setup successful
 *          Appropriate MCBSP driver error code if configuration has any error
 */
int32_t Mcbsp_localEdmaChanPaRAMsetup(Mcbsp_ChannelObj *chanHandle)
{
    uint32_t               linkCnt    = 0;
    Mcbsp_Object_Unpadded *instHandle = NULL;
    EDMA3_DRV_PaRAMRegs  paramSet   = {0,0,0,0,0,0,0,0,0,0,0,0};
    int32_t                status     = MCBSP_STATUS_COMPLETED;
    EDMA3_DRV_SyncType   tempSyncType;

    if ((NULL == chanHandle) || (NULL == chanHandle->devHandle))
    {
        return MCBSP_ERR_BADARGS;
    }

    instHandle = &(chanHandle->devHandle->obj);
    if (NULL == instHandle)
    {
        return MCBSP_ERR_BADARGS;
    }

    /* Configure the McBSP synchonized EDMA channel PaRAM entries             *
     * Configure source and destination address / modes                       *
     * first - main xfer channel                                              */

    /* following is the explanation behind the EDMA configuration             *
     *
     *  1) We will have n arrays of data (now it is loopjob buffer each of    *
     *     length x bytes)                                                    *
     *  2) for each sync event we have to fetch the data from each array      *
     *     and write to each serilaiser                                       *
     *  3) and the no. bytes transferred in this case from each array will    *
     *     be roundedwordwidth/4                                              *
     *  4) as the loopjob buffer is continous, we configure the counts as     *
     *     follows                                                            *
     *     a) consider there are arrays of length roundedwordwidth/8(aCnt)    *
     *     b) these arrays are located apart lengthofloopjob size (bindex)    *
     *     c) there are 'no.of.slots' no. of such arrays  (bCnt)              *
     *     d) (a) and (b) constitute one frame of EDMA transfer and it        *
     *        happens for one sync event in AB sync mode                      *
     *     e) such frames have to be sent till we have buffer - ie            *
     *        lengthofloopjob / (roundedwordwidth/8 )  and this               *
     *        constitutes  cCnt                                               *
     *     f) address of buffer (rather offset from start of previous         *
     *         frame) is roundedwordwidth/8 and this forms cIndex             *
     *  Please take some time to visualise the setup mentioned above to       *
     *  check with EDMA configuration The same kind of setup will be used     *
     *  for data transfers also and the change will be buffer pointer         *
     *  and length of data buffer.                                            */

    /* Get the PaRAM set for default parameters                               */
    EDMA3_DRV_getPaRAM(chanHandle->edmaHandle,chanHandle->xferChan,&paramSet);

    if (MCBSP_MODE_INPUT == chanHandle->mode)
    {
        /* Configuring Source and Dest addresses                              */
        if (TRUE == chanHandle->enableHwFifo)
        {
            paramSet.srcAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
        }
        else
        {
            paramSet.srcAddr = (uint32_t)&(instHandle->hwInfo.regs->DRR);
        }

#ifdef _BIG_ENDIAN
        paramSet.srcAddr += (4 - chanHandle->roundedWordWidth);
#endif
        /* Configuring Src and Dest B&C Indexes                               */
        paramSet.srcBIdx  = 0;
        paramSet.srcCIdx  = 0;

        if (MCBSP_STATUS_COMPLETED != Mcbsp_localGetIndicesSyncType(
                                 chanHandle,
                                 &paramSet.destBIdx,
                                 &paramSet.destCIdx,
                                 &paramSet.aCnt,
                                 &paramSet.bCnt,
                                 &paramSet.cCnt,
                                 &tempSyncType,
                                 TRUE))
        {
            status = MCBSP_ERR_BADARGS;
        }
    }
    else
    {
        /* Configuring Source and Dest addresses                              */
        if (TRUE == chanHandle->enableHwFifo)
        {
            paramSet.destAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
        }
        else
        {
            paramSet.destAddr = (uint32_t)&(instHandle->hwInfo.regs->DXR);
        }
#ifdef _BIG_ENDIAN
        paramSet.destAddr += (4 - chanHandle->roundedWordWidth);
#endif
        /* Configuring Src and Dest B&C Indexes                               */
        paramSet.destBIdx = 0;
        paramSet.destCIdx = 0;

        if (MCBSP_STATUS_COMPLETED != Mcbsp_localGetIndicesSyncType(
                                 chanHandle,
                                 &paramSet.srcBIdx,
                                 &paramSet.srcCIdx,
                                 &paramSet.aCnt,
                                 &paramSet.bCnt,
                                 &paramSet.cCnt,
                                 &tempSyncType,
                                 TRUE))
        {
            status = MCBSP_ERR_BADARGS;
        }
    }

    /* Configuring bCntReload                                                 */
    paramSet.bCntReload = paramSet.bCnt;

    /* Src & Dest are in INCR modes                                           */
    paramSet.opt &= 0xFFFFFFFCu;

    /* FIFO width is 8 bit                                                    */
    paramSet.opt &= 0xFFFFF8FFu;

    /* Set EDMA3_DRV_OPT_FIELD_TCINTEN to FALSE                               */
    paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_ITCINTEN_MASK));

    paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_TCINTEN_MASK));
    paramSet.opt |= (CSL_TPCC_PARAM_OPT_TCINTEN_DISABLE
                        << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);

    /* Program the TCC                                                        */
    paramSet.opt |= ((chanHandle->tcc << CSL_TPCC_PARAM_OPT_TCC_SHIFT)
                      & CSL_TPCC_PARAM_OPT_TCC_MASK);

    /* EDMA3_DRV_SYNC_AB                                                      */
    paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_SYNCDIM_MASK));
    paramSet.opt |= (tempSyncType << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

    /* early completion interrupt is enabled                                  */
    paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_TCCMOD_MASK));
    paramSet.opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);

    /* Now, write the PaRAM Set.                                              */
    status = EDMA3_DRV_setPaRAM(
                  chanHandle->edmaHandle,
                  chanHandle->xferChan,
                  &paramSet);

    if (MCBSP_STATUS_COMPLETED == status)
    {
        /* Configure the link PaRAMs with the appropriate parameters Though   *
         * we configure all the link paramsets, we will be using only one of  *
         * them to link with main xfer channel                                */
        for (linkCnt = 0; linkCnt < Mcbsp_MAXLINKCNT; linkCnt++)
        {
            /* Get the PaRAM set for default parameters                       */
            EDMA3_DRV_getPaRAM (chanHandle->edmaHandle,
                                chanHandle->pramTbl[linkCnt],
                                &paramSet);

            if (MCBSP_MODE_INPUT == chanHandle->mode)
            {
                /* Configuring Source and Dest addresses                      */
                if (TRUE == chanHandle->enableHwFifo)
                {
                    paramSet.srcAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
                }
                else
                {
                    paramSet.srcAddr = (uint32_t)&(instHandle->hwInfo.regs->DRR);
                }
#ifdef _BIG_ENDIAN
                paramSet.srcAddr += (4 - chanHandle->roundedWordWidth);
#endif
                /* Configuring Src and Dest B&C Indexes                       */
                paramSet.srcBIdx  = 0;
                paramSet.srcCIdx  = 0;

                if (MCBSP_STATUS_COMPLETED != Mcbsp_localGetIndicesSyncType(
                                         chanHandle,
                                         &paramSet.destBIdx,
                                         &paramSet.destCIdx,
                                         &paramSet.aCnt,
                                         &paramSet.bCnt,
                                         &paramSet.cCnt,
                                         &tempSyncType,
                                         TRUE))
                {
                    status = MCBSP_ERR_BADARGS;
                }
            }
            else
            {
                /* Configuring Source and Dest addresses                      */
                if (TRUE == chanHandle->enableHwFifo)
                {
                    paramSet.destAddr = (uint32_t)(instHandle->hwInfo.dataAddress);
                }
                else
                {
                    paramSet.destAddr = (uint32_t)&(instHandle->hwInfo.regs->DXR);
                }
#ifdef _BIG_ENDIAN
                paramSet.destAddr += (4 - chanHandle->roundedWordWidth);
#endif
                paramSet.destBIdx  = 0;
                paramSet.destCIdx  = 0;

                if (MCBSP_STATUS_COMPLETED != Mcbsp_localGetIndicesSyncType(
                                         chanHandle,
                                         &paramSet.srcBIdx,
                                         &paramSet.srcCIdx,
                                         &paramSet.aCnt,
                                         &paramSet.bCnt,
                                         &paramSet.cCnt,
                                         &tempSyncType,
                                         TRUE))
                {
                    status = MCBSP_ERR_BADARGS;
                }
            }

            /* Configuring bCntReload                                         */
            paramSet.bCntReload = paramSet.bCnt;

            /* Src & Dest are in INCR modes                                   */
            paramSet.opt &= 0xFFFFFFFCu;

            /* FIFO width is 8 bit                                            */
            paramSet.opt &= 0xFFFFF8FFu;

            /* EDMA3_DRV_SYNC_AB                                              */
            paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_SYNCDIM_MASK));
            paramSet.opt |= (tempSyncType << CSL_TPCC_PARAM_OPT_SYNCDIM_SHIFT);

            /* Set EDMA3_DRV_OPT_FIELD_TCINTEN to FALSE                       */
            paramSet.opt |= (CSL_TPCC_PARAM_OPT_TCINTEN_DISABLE
                                << CSL_TPCC_PARAM_OPT_TCINTEN_SHIFT);

            /* Program the TCC                                                */
            paramSet.opt |= ((chanHandle->tcc << CSL_TPCC_PARAM_OPT_TCC_SHIFT)
                              & CSL_TPCC_PARAM_OPT_TCC_MASK);

            /* early completion interrupt                                     */
            paramSet.opt &= (~(CSL_TPCC_PARAM_OPT_TCCMOD_MASK));
            paramSet.opt |= (1 << CSL_TPCC_PARAM_OPT_TCCMOD_SHIFT);

            if (MCBSP_STATUS_COMPLETED == status)
            {
                /* Now, write the PaRAM Set.                                  */
                status = EDMA3_DRV_setPaRAM(chanHandle->edmaHandle,
                                            chanHandle->pramTbl[linkCnt],
                                            &paramSet);
            }
        }/*for loop for linkcount*/
    }
    return (status);
}
#endif /* #ifdef MCBSP_LOOPJOB_ENABLE */

/**
 * \brief  This checks is the next request in queue is data request. If this is
 *         an abort request, it calls the appropriate function to deal
 *         with it.
 *
 * \param  chanHandle   [IN]  Handle to the Mcbsp Channel
 *
 * \return Mcbsp_STATUS_INVALID,if no outstanding requests
 *         Mcbsp_STATUS_VALID, if pending buffer is there
 *
 */
int32_t mcbspIsValidIOBuf(Mcbsp_ChannelObj *chanHandle)
{
    int32_t          status     = Mcbsp_STATUS_INVALID;

    if ((NULL == chanHandle) || (NULL == chanHandle->devHandle))
    {
        return MCBSP_ERR_BADARGS;
    }

    /* check if the queue has any request to be processed                     */
    if (FALSE == Mcbsp_osalQueueEmpty(chanHandle->ptrQPendList))
    {
        chanHandle->dataIOBuf = (Mcbsp_IOBuf *) Mcbsp_osalQueueGet(chanHandle->ptrQPendList);

        if (NULL != chanHandle->dataIOBuf)
        {
            /* we have a valid buffer to process next                         */
            chanHandle->currentDataSize =
                (uint16_t)(chanHandle->dataIOBuf->size);
            chanHandle->userDataBufferSize =
                (uint32_t)(chanHandle->dataIOBuf->size);

            /* Additional to assigning the important parameters as above      *
             * for DMA mode , we will have max 2 buffers floating and hence   *
             * we use a activequue to manage this                             */
            Mcbsp_osalQueuePut(chanHandle->ptrQFloatList, (Mcbsp_QueueElem *)chanHandle->dataIOBuf);

            /* Set the status as valid                                        */
            status = Mcbsp_STATUS_VALID;
        }
    }
    else
    {
        /* we dont have any buffer in queue stop xmt/rcv SM before giving     *
         * error                                                              */
        chanHandle->dataIOBuf = NULL;

        /* Set the status as invalid to program the loopjob buffer            */
        status = Mcbsp_STATUS_INVALID;
    }
    return status;
}

/**
 * \brief   Process the buffer and update the EDMA paramset
 *
 *          For the transmit operation, the eDMA channel's destination
 *          port is tied to the MCBSP DAT port. In case of receive, the eDMA
 *          channel's source port is tied to the MCBSP DAT port. The source
 *          address for transmit eDMA channel and the destination address for
 *          the receive eDMA channel are set here.
 *
 * \param  chanHandle [IN]    Pointer to channel
 * \param  ioBuf      [IN]    Pointer to request to be processed
 *
 * \return  MCBSP_STATUS_COMPLETED, if the address is set correctly
 *          MCBSP_ERR_BADIO otherwise
 */
int32_t Mcbsp_localEdmaProcessPkt(Mcbsp_ChannelObj *chanHandle,
                                Mcbsp_IOBuf       *ioBuf)
{
    int32_t   status = MCBSP_STATUS_COMPLETED;

    if ((NULL == chanHandle) || (NULL == ioBuf))
    {
        return MCBSP_ERR_BADARGS;
    }

    /* we are going to update the paramset from EDMA callback context. That   *
     * means the one of the param sets are having audio data just completed   *
     * data request and one having currently loaded (active) data request     *
     * Current index is pointing only latest paramset (that is active)  so    *
     * move the index                                                         */
    Mcbsp_localGetNextIndex(&chanHandle->nextLinkParamSetToBeUpdated);

    /* update the edma paramset with the buffer details                       */
    status = Mcbsp_localUpdtDtPktToLnkPrms(chanHandle, ioBuf);

    return (status);
}

/**
 *  \brief  Mcbsp_localGetIndicesSyncType
 *
 *  This function computes the parameters requied to configure EDMA 3 based
 *  on the buffer format seleted while creating the driver.
 *
 *  Entry Criteria : This function expects channel variables such as no of slots
 *                   length of buffers is updated.
 *
 *  \param    chanHandle       [IN]    Pointer to channel handle
 *  \param    bIndex           [IN]    Pointer to stores computed bIndex
 *  \param    cIndex           [IN]    Pointer to stores computed cIndex
 *  \param    aCnt             [IN]    Pointer to stores computed aCnt
 *  \param    bCnt             [IN]    Pointer to stores computed bCnt
 *  \param    cCnt             [IN]    Pointer to stores computed cCnt
 *  \param    syncType         [IN]    Pointer to stores computed mode of EDMA
 *  \param    forLoopJobBuf    [IN]    To calucalate above for loopJob or data
 *
 *  CAUTION:  This function could be called with pointer pointing to EDMA 3
 *            paramset pointer (avoiding couple of variables). Care should be
 *            take to ensure that data types used in EDMA 3 paramset is
 *            consistent with EDMA 3 defined data types.
 *
 *  \return   MCBSP_ERR_BADARGS on invalid buffer format, else MCBSP_STATUS_COMPLETED
 */
int32_t Mcbsp_localGetIndicesSyncType(Mcbsp_ChannelObj   *chanHandle,
                                    volatile int16_t     *bIndex,
                                    volatile int16_t     *cIndex,
                                    volatile uint16_t    *aCnt,
                                    volatile uint16_t    *bCnt,
                                    volatile uint16_t    *cCnt,
                                    EDMA3_DRV_SyncType *syncType,
                                    Bool                forLoopJobBuf)
{
    uint32_t  tempSize  = 0x0;
    int32_t   status    = MCBSP_STATUS_COMPLETED;

    if(((NULL == chanHandle) || (NULL == bIndex) || (NULL == cIndex)
                    || (NULL == aCnt) || (NULL == bCnt) || (NULL == cCnt)
                    || (NULL == syncType)))
    {
        return MCBSP_ERR_BADARGS;
    }

    /* Get the size of the buffer to be used. In case of mute command and     *
     * normal IOP we will use the actual size of the buffer. For the loop job *
     * case 1. if internal loop job then the minimum loop job length will be  *
     *         used.                                                          *
     * case 2. user supplied loop job buffer then the user loop job length    *
     *         will be used.                                                  */
    if ((TRUE == forLoopJobBuf) && (FALSE == chanHandle->bMuteON))
    {
        if (TRUE == chanHandle->userLoopJob)
        {
            /* use the user supplied buffer length for the EDMA params        */
            tempSize = chanHandle->userLoopJobLength;
        }
        else
        {
            tempSize = chanHandle->loopJobLength;
        }
    }
    else
    {
        tempSize = chanHandle->userDataBufferSize;
    }

    /* Get the indices                                                        */
    switch (chanHandle->dataFormat)
    {
        case Mcbsp_BufferFormat_1SLOT:
        case Mcbsp_BufferFormat_MULTISLOT_INTERLEAVED:
            *aCnt = chanHandle->roundedWordWidth;
            *bCnt = 1u;
            *cCnt = (uint16_t)(tempSize/(*aCnt));
            *bIndex = (int16_t)tempSize;
            *cIndex = (int16_t)(*aCnt);
            *syncType = EDMA3_DRV_SYNC_AB;
            break;
        case Mcbsp_BufferFormat_MULTISLOT_NON_INTERLEAVED:
            *aCnt = chanHandle->roundedWordWidth;
            *bCnt = (uint16_t)(chanHandle->numEnabledChannels);

            /* temp Size is always a multiple of the acnt hence the division  *
             * result will always be an integer                               */
            *cCnt = (uint16_t)(tempSize /((*aCnt) * (*bCnt)));
            /* temp Size is always a multiple of the bcnt when using the      *
             * multiple slots hence the result will be always an integer      */
            *bIndex = (int16_t)(tempSize/(*bCnt));
            *cIndex = (int16_t)((*aCnt) - (((*bCnt)- 1u) * (tempSize/(*bCnt))));
            *syncType = EDMA3_DRV_SYNC_A;
            break;
        default:
            status = MCBSP_ERR_BADARGS;
               break;
    }

    /* if the loop job buffer being used is the driver internal loop job      *
     * buffer, dont increment the index for it.Same is the case if mute is ON */
    if (TRUE == forLoopJobBuf)
    {
        if ((TRUE == chanHandle->bMuteON) ||
            (FALSE == chanHandle->userLoopJob))
        {
            *bIndex = 0;
            *cIndex = 0;
        }
    }
    return status;
}

/* ========================================================================== */
/*                              END OF FILE                                   */
/* ========================================================================== */

