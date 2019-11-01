/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010
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
 *  File Name: tsip.c
 *
 *  Source for the TSIP Driver: TSIP control functions
 *
 **/

#include "tsip.h"
#include "tsiploc.h"
#include "tsipcsl.h"
#include "tsip_osal.h"

/* Global Variable which describes the TSIP LLD Version Information */
const char TSIPLLDVersionStr[] = tsip_LLD_VERSION_STR ":" __DATE__  ":" __TIME__;

/**********************************************************************************
 * FUNCTION PURPOSE: Locate the tsip structure associated with a port
 **********************************************************************************
 * DESCRIPTION: Finds the tsip structure with the requested port number. Returns
 *              NULL is that port is not found.
 **********************************************************************************/
tsip_t *tsipFromPort (tsipDrvInst_t *tsipDrvInst, int16_t port)
{
  tsip_t *tsip;
  int16_t i;

  for (i = 0; i < tsipDrvInst->nTsipPorts; i++)  {

    tsip = (tsip_t *)(tsipDrvInst->tsipPort[i]);
    if ((tsip != NULL) && (tsip->port == port))
      return (tsip);

  }

  return (NULL);

} /* tsipFromPort */

/**********************************************************************************
 * FUNCTION PURPOSE: Configure port dma enable info
 **********************************************************************************
 * DESCRIPTION: The dma enable information is computed from the tsip contexts
 *              and made ready for enable.
 **********************************************************************************/
void tsipLoadDmaEnc (tsip_t *tsip, cslTsipChEnable_t *entx, cslTsipChEnable_t *enrx)
{

  entx->port    = tsip->port;
  entx->channel = tsip->tx.channel;
  entx->cnid    = TSIP_INITIAL_TX_CNID;

  entx->baseAddressGlobal = GLOBAL_ADDR(tsip->tx.dmap1->baseAddressLocal);
  entx->frameAlloc        = tsip->tx.dmap1->frameAlloc;
  entx->frameSize         = tsip->tx.dmap1->frameSize;
  entx->frameCount        = tsip->tx.frameCount;

  enrx->port    = tsip->port;
  enrx->channel = tsip->rx.channel;
  enrx->cnid    = TSIP_INITIAL_RX_CNID;

  enrx->baseAddressGlobal = GLOBAL_ADDR(tsip->rx.dmap1->baseAddressLocal);
  enrx->frameAlloc        = tsip->rx.dmap1->frameAlloc;
  enrx->frameSize         = tsip->rx.dmap1->frameSize;
  enrx->frameCount        = tsip->rx.frameCount;

} /* tsipLoadDmaEnc */



/**********************************************************************************
 * FUNCTION PURPOSE: Compute the number of sub-frames until the next desired phase
 **********************************************************************************
 * DESCRIPTION: Calculates the number of sub-frames until the desired phase
 *              value is reached. The value will always lie in the range
 *              0 < offset <= frameSize. The units are sub-Frames
 **********************************************************************************/
 int16_t tsip_compute_phase_offset (tsipTsArray_t *ts, tsipTs_t *tsInfo)
 {
  int16_t phaseOffset;
  int16_t frameSizePhase;
  int16_t phase;

  /* Bound the requested phase between 0 (inclusive) and the max phase (exclusive) */
  phase = tsInfo->phase % tsInfo->maxSubPhase;

  /* Compute the delay required to mee this phase in units of sub-frames */
  if (phase >= tsInfo->subPhase)
    phaseOffset = phase - tsInfo->subPhase;
  else
    phaseOffset = tsInfo->maxSubPhase - (tsInfo->subPhase - phase);

  /* Compute the number of phases in the frame size */
  frameSizePhase = ts->size / tsInfo->subFrame;

  /* Force the phase offset value to be lower then the frame size. This will allow the
   * frame to begin on the nearest phase value that matches the requested phase */
  phaseOffset = phaseOffset % frameSizePhase;

  /* If the result is 0, wrap it to the full frame size */
  if (phaseOffset == 0) 
    phaseOffset = frameSizePhase;

  return (phaseOffset);

} /* tsip_compute_phase_offset */


/*****************************************************************************
 * FUNCTION PURPOSE: Compute the requested sub-phase initial offset
 *****************************************************************************
 * DESCRIPTION: Determines the initial index into the application buffer
 *              so that the frame will complete on the desired sub-phase.
 *****************************************************************************/
tsipReturn_t tsip_sub_phase (tsipTsArray_t *ts, tsipTs_t *tsInfo)
{
  int16_t phaseOffset;

  /* Compute the number of sub-frames before the desired sub-phase occurs. This
   * value will always be strictly greater then 0, and less then or equal to the
   * number of sub-frames in the app buffer */
  phaseOffset = tsip_compute_phase_offset (ts, tsInfo);

  /* phaseOffset now contains the number of sub-phases before the next tx/rx callout
   * should occur. This value is guaranteed to be in the correct range based on
   * the calculations above and (0 < offset <= frame size). Convert the phase
   * value to a sample offset */
  ts->index = ts->size - (phaseOffset * tsInfo->subFrame);

  return (tsip_OK);

} /* tsip_sub_phase */


/*****************************************************************************
 * FUNCTION PURPOSE: Locate a timeslot structure with the specified timeslot
 *****************************************************************************
 * DESCRIPTION: Passes through the timeslot array to find an active or
 *              pending timeslot with the same timeslot/link value
 *****************************************************************************/
tsipTsArray_t *tsip_search_ts (tsipTsArray_t *ts, int16_t timeslot, tsipTsArray_t **prev)
{

  *prev = NULL;

  while (ts != NULL)  {
    
    if (ts->port_Link_Ts == timeslot)
      return (ts);

    *prev = ts;
    ts    = ts->next;

  }

  *prev = NULL;
  return (NULL);

} /* tsip_search_ts */

/******************************************************************************
 * FUNCTION PURPOSE: Search for an available timeslot element
 ******************************************************************************
 * DESCRIPTION: Returns the first element in the free timeslot list.
 *              Returns NULL if no free timeslots are found.
 ******************************************************************************/
tsipTsArray_t *tsip_find_free_ts (tsipShared_t *sTsip)
{
  tsipTsArray_t *ts    = NULL;

  if (sTsip->tsArrayFree != NULL)  {
    ts = sTsip->tsArrayFree;
    sTsip->tsArrayFree = ts->next;
  } 

  return (ts);
  
} /* tsip_find_free_ts */

/******************************************************************************
 * FUNCTION PURPOSE: Remove a timeslot from a linked list
 ******************************************************************************
 * DESCRIPTION: The timeslot is removed from the linked list. Because the
 *              linked list doesn't have a dedicated head element the top
 *              level pointer must be provided in case the first element 
 *              is removed.
 ******************************************************************************/
void tsip_remove_ts_link (tsipTsArray_t *ts, tsipTsArray_t *prev, tsipTsArray_t **head)
{

  if (prev == NULL)  {

    if (ts->next == NULL)
      *head = NULL;
    else
      *head = ts->next;


  }  else  {

    prev->next = ts->next;

  }

  /* Clear the next pointer to indicate the element has been removed */
  ts->next = NULL;

} /* tsip_remove_ts_link */

/****************************************************************************
 * FUNCTION PURPOSE: Add a timeslot to a linked list
 ****************************************************************************
 * DESCRIPTION: The timeslot is placed at the start of the linked list
 ****************************************************************************/
inline void tsip_add_ts_link (tsipTsArray_t *ts, tsipTsArray_t **dest)
{
    ts->next = *dest;
    *dest   = ts;

} /* tsip_add_ts_link */


/******************************************************************************
 * FUNCTION PURPOSE: Return the last element in a chain
 ******************************************************************************
 * DESCRIPTION: Runs through the linked list and returns the last element
 *              in the chain. The element remains on the chain
 ******************************************************************************/
tsipTsArray_t *tsip_get_last_elem (tsipTsArray_t *ts)
{
  tsipTsArray_t *rts = NULL;

  while (ts != NULL)  {

    rts = ts;
    ts  = ts->next;

  }

  return (rts);

} /* tsip_get_last_elem */

/******************************************************************************
 * FUNCTION PURPOSE: Fill in timeslot info and add to the pending enable list
 ******************************************************************************
 * DESCRIPTION: A free timeslot structure is populated and placed onto the
 *              pending enable list.
 ******************************************************************************/
tsipReturn_t tsip_add_pending_timeslot (tsipShared_t *sTsip,
                                            tsipSharedDir_t *sTsipDir, 
                                            tsipTs_t *tsInfo, 
                                            tsipTsArray_t **tsRet)
{
  tsipTsArray_t *ts;

  ts = tsip_find_free_ts (sTsip);
  if (ts == NULL)  
    return (tsip_NO_FREE_TIMESLOT);

  /* Return the context pointer */
  *tsRet = (void *)ts;
  /* Attach the new timeslot to the top of the linked list */
  ts->next = sTsipDir->tsPendingActivate;
  sTsipDir->tsPendingActivate = ts;

  TSIP_SET_TSA_STATE(ts, TSIP_TS_STATE_PENDING_ENABLE);
  TSIP_SET_TSA_COMPAND(ts, tsInfo->compand);

  ts->port_Link_Ts = (uint16_t)(tsInfo->timeslot);
  ts->context      = tsInfo->context;
  ts->base         = tsInfo->base;
  ts->size         = tsInfo->appFrameSize;
  ts->callout      = tsInfo->callout;
  ts->rinDesc      = tsInfo->rinDesc;
  tsip_sub_phase (ts, tsInfo);

  return (tsip_OK);

} /* tsip_add_pending_timeslot */


/******************************************************************************
 * FUNCTION PURPOSE: Move a pending disable timeslot back to the active list
 ******************************************************************************
 * DESCRIPTION: A pending deactivate timeslot is moved back to the active list.
 ******************************************************************************/
tsipReturn_t tsip_restore_pending_deactivate (tsipDir_t *dir, tsipSharedDir_t *sdir, 
                                                  tsipTsArray_t *ts, tsipTsArray_t *prev, 
                                                  tsipTs_t *tsInfo)
{

  /* Remove the timeslot from the pending deactivate list */
  if (prev != NULL)
    prev->next  = ts->next;
  else
    sdir->tsPendingDeactivate = ts->next;


  /* Attach the newly re-activated timeslot to the top of the enabled linked list */
  ts->next = dir->tsArray;
  dir->tsArray = ts;


  /* Setup the timeslot context */
  TSIP_SET_TSA_STATE(ts, TSIP_TS_STATE_ENABLED);
  TSIP_SET_TSA_COMPAND(ts, tsInfo->compand);

  ts->port_Link_Ts = (uint16_t)(tsInfo->timeslot);
  ts->context      = tsInfo->context;
  ts->base         = tsInfo->base;
  ts->size         = tsInfo->appFrameSize;
  ts->callout      = tsInfo->callout;
  ts->rinDesc      = tsInfo->rinDesc;
  tsip_sub_phase (ts, tsInfo);

  return (tsip_OK);

} /* tsip_restore_pending_deactivate */



/******************************************************************************
 * FUNCTION PURPOSE: Configure one direction of a timeslot
 ******************************************************************************
 * DESCRIPTION: A timeslot change is made pending. 
 *          
 *              Whether a timeslot is placed on (or removed) from a pending
 *              list depends on the current elements in the three lists.
 *              The actions are described in the truth tables.
 *              For Timeslot enable:
 *
 *                    ----------
 *  active            |  0  0  |  0  0  1  1  1  1
 *  pending active    |  0  0  |  1  1  0  0  1  1
 *  pending disable   |  0  1  |  0  1  0  1  0  1
 *                    ----------
 *    So a timeslot enable request is valid only if the timeslot is not on
 *    the active list and not on the pending active list. The timeslot
 *    is placed on the pending active list.
 *
 *              For Timeslot disable:
 *
 *                          -------------
 *  active            0  0  |  0  0  1  |  1  1  1
 *  pending active    0  0  |  1  1  0  |  0  1  1
 *  pending disable   0  1  |  0  1  0  |  1  0  1
 *                          -------------
 *
 *   For disable, however, two possible actions can occur. If the timeslot is
 *   already pending active, then the pending active state is simply removed.
 *   The transition for these state are:
 *
 *  active           0      0       0      0      1      0
 *  pending active   1  ->  0       1  ->  0      0 ->   0
 *  pending disable  0      0       1      1      0      1
 *
 *
 *  Based on these transitions the possible states are limited to the
 *  five states outlined in the enable and disable diagrams.
 *      
 *
 ******************************************************************************/
tsipReturn_t tsip_ts_config (tsipShared_t *sTsip, tsipDir_t *dir, 
                                 tsipSharedDir_t *sdir, tsipTs_t *tsInfo)
{
  tsipTsArray_t  *tsActive,  *tsActivePrev;
  tsipTsArray_t  *tsPendEn,  *tsPendEnPrev;
  tsipTsArray_t  *tsPendDe,  *tsPendDePrev;
  tsipReturn_t      retVal;
  tsipTsArray_t  *tsRet = NULL;

  tsActive = tsip_search_ts (dir->tsArray, tsInfo->timeslot, &tsActivePrev);
  tsPendEn = tsip_search_ts (sdir->tsPendingActivate, tsInfo->timeslot, &tsPendEnPrev);
  tsPendDe = tsip_search_ts (sdir->tsPendingDeactivate, tsInfo->timeslot, &tsPendDePrev);

  if (tsInfo->enable == TRUE)  {
    /* If the timeslot was pending de-activate, simply move it back to the active list */
    if (tsPendDe != NULL) {
      tsRet = tsPendDe;
      retVal = tsip_restore_pending_deactivate (dir, sdir, tsPendDe, 
                                                    tsPendDePrev, tsInfo);
      goto end;
    } else if ( (tsActive == NULL) && (tsPendEn == NULL) ) {
      retVal = tsip_add_pending_timeslot (sTsip, sdir, tsInfo, &tsRet);
      goto end;
    } else {
      /* Either active or pending, so select correct context to return */
      if (tsActive) {
        tsRet = tsActive;
      } else {
        tsRet = tsPendEn;
      } 
      retVal = tsip_TIMESLOT_ALREADY_ENABLED;
      goto end;
    }
  }

  /* For disable the code relies on the fact that there are only 5 possible states
   * for the timeslot */

  if (tsPendEn != NULL)  {
    tsip_remove_ts_link (tsPendEn, tsPendEnPrev, &sdir->tsPendingActivate);
    tsPendEn->next     = sTsip->tsArrayFree;
    sTsip->tsArrayFree = tsPendEn;

  }  else if (tsActive != NULL)  {
    tsip_remove_ts_link (tsActive, tsActivePrev, &(dir->tsArray));
    tsActive->next            = sdir->tsPendingDeactivate;
    sdir->tsPendingDeactivate = tsActive;
  }

  /* It is not an error to disable a timeslot that is not enabled */
  retVal = tsip_OK;

end:
  /* Return timeslot's context */
  tsInfo->rinDesc.totdmToFromtdmLink = tsRet;
  return retVal;
} /* tsip_ts_config */


/********************************************************************************
 * FUNCTION PURPOSE: Identify TSIP port from timeslot information
 ********************************************************************************
 * DESCRIPTION: Identify the TSIP port in the driver instance according to the
 *              physical port number recorded in the timeslot variable.
 *              
 ********************************************************************************/
tsip_t *tsip_port_from_timeslot(tsipDrvInst_t *tsipDrvInst, int16_t timeslot, tsipReturn_t *ret)
{
  tsip_t    *tsip;
  int16_t      physPort;
  int16_t      i;
  
  *ret = tsip_OK;
  physPort = TSIP_READ_BITFIELD (timeslot, TSIP_PORT_MSB, TSIP_PORT_LSB);

  for (i = 0; i < tsipDrvInst->nTsipPorts; i++)  {
    tsip = (tsip_t *)(tsipDrvInst->tsipPort[i]);
    if (tsip)  {
      if (physPort == tsip->port)
        break;
    }
  }

  if (i == tsipDrvInst->nTsipPorts)
    *ret = tsip_INVALID_TSIP_PORT_ID;

  return (tsip);
}

/********************************************************************************
 * FUNCTION PURPOSE: Handle timeslot status changes
 ********************************************************************************
 * DESCRIPTION: Handles timeslot status changes. Changes are only made 
 *              pending, the actual enables occur in the super-frame
 *              interrupt.
 ********************************************************************************/
tsipReturn_t Tsip_timeslotConfig(Tsip_DrvHandle handle, tsipTsControl_t *ctl)
{
  tsipReturn_t ret;
  tsipReturn_t retStatus = tsip_OK;

  tsipDrvInst_t *tsipDrvInst = (tsipDrvInst_t *)handle;
  tsipShared_t *sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;
  tsip_t   *tsip;

  tsipTs_t tsInfo;
  tsipTsArray_t  *tsaTx = NULL, *tsaRx = NULL;
  uint32_t  csInfo;

  Tsip_osalEnterCriticalSection(&csInfo);

  tsip = tsip_port_from_timeslot(tsipDrvInst, ctl->rx.timeslot, &ret);
  if (ret != tsip_OK)  {
    Tsip_osalExitCriticalSection (csInfo);
    return (ret);
  }

  /* Configure the tx (to tdm) side */
  tsInfo.enable       = ctl->tx.enable;
  tsInfo.timeslot     = ctl->tx.timeslot;
  tsInfo.base         = ctl->tx.buffer;
  tsInfo.appFrameSize = ctl->tx.frameSize;
  tsInfo.phase        = ctl->phase;
  tsInfo.callout.tx   = ctl->tx.callout;
  tsInfo.context      = (void *)ctl->tx.context;
  tsInfo.subPhase     = tsip->subPhase;
  tsInfo.maxSubPhase  = tsip->maxSubPhase;
  tsInfo.subFrame     = tsip->subFrame;
  tsInfo.compand      = ctl->compand;
  tsInfo.rinDesc.rinBase = NULL; /* No initial delay line buffer */

  ret = tsip_ts_config (sTsip, &(tsip->tx), &(sTsip->sharedTx), &tsInfo);
  tsaTx = ctl->txTsContext = tsInfo.rinDesc.totdmToFromtdmLink;

  /* Don't fail on enabling an already enabled timeslot - but return 
   * the enable status */
  if ( (tsInfo.enable == TRUE) && (ret == tsip_TIMESLOT_ALREADY_ENABLED) )  {
    retStatus = ret;
  } else if (ret != tsip_OK)  {
    Tsip_osalExitCriticalSection (csInfo);
    return (ret);
  }

  /* Configure the rx (from tdm) side */
  tsInfo.enable       = ctl->rx.enable;
  tsInfo.timeslot     = ctl->rx.timeslot;
  tsInfo.base         = ctl->rx.buffer;
  tsInfo.appFrameSize = ctl->rx.frameSize;
  tsInfo.phase        = ctl->phase;
  tsInfo.callout.rx   = ctl->rx.callout;
  tsInfo.context      = (void *)ctl->rx.context;
  tsInfo.compand      = ctl->compand;
  tsInfo.rinDesc.rinBase = NULL; /* No initial delay line buffer */
  ret = tsip_ts_config (sTsip, &(tsip->rx), &(sTsip->sharedRx), &tsInfo);
  tsaRx = tsInfo.rinDesc.totdmToFromtdmLink;

  /* Link tsaTx to tsaRx */
  tsaTx->rinDesc.totdmToFromtdmLink = tsaRx;

  /* Return the warning on re-enable */
  if (ret == tsip_OK)
    ret = retStatus;

  Tsip_osalExitCriticalSection (csInfo);
  return (ret);

} /* Tsip_timeslotConfig */

/*********************************************************************************
 * FUNCTION PURPOSE: Get the TSIP callouts function
 *********************************************************************************
 * DESCRIPTION: Get the Rx Callout and context.
 *********************************************************************************/
tsipReturn_t Tsip_getCallout(Tsip_DrvHandle handle, tsipTsControl_t *ctl)
{
  tsipReturn_t ret;
  tsipTsArray_t *tsa;
  tsipDrvInst_t *tsipDrvInst = (tsipDrvInst_t *)handle;
  tsipShared_t *sTsip;
  tsip_t   *tsip;

  tsipTsArray_t *prev;
  uint32_t  csInfo;

  Tsip_osalEnterCriticalSection(&csInfo);

  tsip = tsip_port_from_timeslot(tsipDrvInst, ctl->rx.timeslot, &ret);
  if (ret != tsip_OK)  {
    Tsip_osalExitCriticalSection (csInfo);
    return (ret);
  }

  tsa = tsip_search_ts (tsip->rx.tsArray, ctl->rx.timeslot, &prev);

  if (tsa == NULL)  {
    sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;
    tsa    = tsip_search_ts (sTsip->sharedRx.tsPendingActivate, ctl->rx.timeslot, &prev);
  }
  
  ctl->rx.context=tsa->context;
  ctl->rx.callout=tsa->callout.rx;
  
  Tsip_osalExitCriticalSection (csInfo);
  return (tsip_OK);

} /* Tsip_getCallout */

/*********************************************************************************
 * FUNCTION PURPOSE: Reset the tdm callouts
 *********************************************************************************
 * DESCRIPTION: Used to change the callout functions and contexts.
 *            . The orignal values of the callout functions are returned
 *              in the control structure.
 *********************************************************************************/
tsipReturn_t Tsip_setCallout(Tsip_DrvHandle handle, tsipTsControl_t *ctl)
{
  tsipReturn_t ret;
  tsipDrvInst_t *tsipDrvInst = (tsipDrvInst_t *)handle;
  tsipShared_t *sTsip;
  tsip_t   *tsip;

  tsipTsArray_t *tsa;
  tsipTsArray_t *prev;

  void *tContext;
  tsipAppCallout_t tCallout;
  uint32_t  csInfo;

  Tsip_osalEnterCriticalSection(&csInfo);

  tsip = tsip_port_from_timeslot(tsipDrvInst, ctl->rx.timeslot, &ret);
  if (ret != tsip_OK)  {
    Tsip_osalExitCriticalSection (csInfo);
    return (ret);
  }

  /* Update the original callouts in the tsa, and return in the control structure
   * the original values */
 
  /* The transmit side */
  tsa = tsip_search_ts (tsip->tx.tsArray, ctl->tx.timeslot, &prev);

  if (tsa == NULL)  {
    sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;
    tsa    = tsip_search_ts (sTsip->sharedTx.tsPendingActivate, ctl->tx.timeslot, &prev);
  }

  if (tsa != NULL)  {
    tContext       = tsa->context;
    tsa->context   = ctl->tx.context;
    ctl->tx.context = tContext;

    tCallout       = tsa->callout;
    tsa->callout.tx = ctl->tx.callout;
    ctl->tx.callout = tCallout.tx;
  }
  
  /* The Receive side */
  tsa = tsip_search_ts (tsip->rx.tsArray, ctl->rx.timeslot, &prev);

  if (tsa == NULL)  {
    sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;
    tsa    = tsip_search_ts (sTsip->sharedRx.tsPendingActivate, ctl->rx.timeslot, &prev);
  }

  if (tsa != NULL)  {
    tContext       = tsa->context;
    tsa->context   = ctl->rx.context;
    ctl->rx.context = tContext;

    tCallout       = tsa->callout;
    tsa->callout.rx = ctl->rx.callout;
    ctl->rx.callout = tCallout.rx;
  }

  Tsip_osalExitCriticalSection (csInfo);
  return (tsip_OK);

} /* Tsip_setCallout */

/*********************************************************************************
 * FUNCTION PURPOSE: perform run time staggering of timeslots
 *********************************************************************************
 * DESCRIPTION: Resets buffer indices to get the desired stagger phase
 *              for a timeslot.
 *********************************************************************************/
tsipReturn_t Tsip_configPhase(Tsip_DrvHandle handle, tsipTsControl_t *ctl)
{
  tsipReturn_t ret;
  tsipDrvInst_t *tsipDrvInst = (tsipDrvInst_t *)handle;
  tsip_t   *tsip;

  tsipTsArray_t *tsa;
  tsipTsArray_t *prev;
  tsipTs_t       tsInfo;

  tsip = tsip_port_from_timeslot(tsipDrvInst, ctl->rx.timeslot, &ret);
  if (ret != tsip_OK)  {
    return (ret);
  }

  /* Transmit direction (to tdm). Don't return an error if an invalid
   * timeslot is provided. An invalid timeslot (-1) is used if only
   * one direction is to be phased */
  tsa = tsip_search_ts (tsip->tx.tsArray, ctl->tx.timeslot, &prev);
  if (tsa != NULL)  {

    /* Adjust the frame lengths */
    tsa->size = ctl->tx.frameSize;

    /* Store required info */
    tsInfo.phase       =  ctl->phase;   /* Desired sub-phase   */
    tsInfo.maxSubPhase =  tsip->maxSubPhase;        /* Max sub-phase value */
    tsInfo.subPhase    =  tsip->subPhase;           /* Current sub-phase   */
    tsInfo.subFrame    =  tsip->subFrame;
    ret = tsip_sub_phase (tsa, &tsInfo);
    if (ret != tsip_OK)
      return (ret);
   }

  /* Receive direction (from tdm)  */
  tsa = tsip_search_ts (tsip->rx.tsArray, ctl->rx.timeslot, &prev);
  if (tsa != NULL)  {

    /* Adjust the frame lengths */
    tsa->size = ctl->rx.frameSize;

    /* Store required info */
    tsInfo.phase       = ctl->phase;
    tsInfo.maxSubPhase = tsip->maxSubPhase;
    tsInfo.subPhase    = tsip->subPhase;
    tsInfo.subFrame    =  tsip->subFrame;    
    ret = tsip_sub_phase (tsa, &tsInfo);
    if (ret != tsip_OK)
      return (ret);

   }

  return (tsip_OK);

} /* Tsip_configPhase */

/******************************************************************************
 * FUNCTION PURPOSE: Returns the index of the last consumed sample in DMA buffer
 ******************************************************************************
 * DESCRIPTION: Returns the index of the last consumed sample in DMA buffer.
 *              The unit is samples, regardless of sample size.
 ******************************************************************************/
int16_t Tsip_getTxDmaPos (void *cxt)
{
  tsipTsArray_t *txTsa = (tsipTsArray_t *)cxt;

  return txTsa->index;

} /* Tsip_getTxDmaPos */


/*********************************************************************************
 * FUNCTION PURPOSE: Create a channel map for the peripheral
 *********************************************************************************
 * DESCRIPTION: Enables any pending timeslots, and performs the enable
 *              by writing the enable bits to the context.
 *********************************************************************************/
void tsip_build_channel_map (tsipSharedDir_t  *sdir, tsipDir_t *dir, 
                                 cslTsipContext_t *cxt, int16_t pendingStateCtl, int16_t mapChangeEnable)
{
  tsipTsArray_t *tsa;

  /* Run through each of the timeslots. Change the enabled status and build maps as required */
  tsa = dir->tsArray;
  while (tsa != NULL)  {

    if (mapChangeEnable == TSIP_ENABLE_MAP_CHANGE)  {

      if (pendingStateCtl == TSIP_TS_ENABLE_PENDING)  {
        if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_PENDING_ENABLE)
          TSIP_SET_TSA_STATE(tsa, TSIP_TS_STATE_ENABLED);
      }
      /* Any enabled timeslots (even if they are already enabled) must be set
          * in the timeslot di-bit map. */
      if (TSIP_GET_TSA_STATE(tsa) != TSIP_TS_STATE_DISABLED)  
          cslTsipEnableTimeslot (cxt, TSIP_GET_TSA_LINK(tsa), TSIP_GET_TSA_TS(tsa),
                                 TSIP_GET_TSA_COMPAND(tsa)); 
      
    } 
    else {
    if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_ENABLED)  
        cslTsipEnableTimeslot (cxt, TSIP_GET_TSA_LINK(tsa), TSIP_GET_TSA_TS(tsa),
                               TSIP_GET_TSA_COMPAND(tsa)); 
    }
    tsa = tsa->next;

  }
  tsa = sdir->tsPendingDeactivate;
  while (tsa != NULL)  {

    if(TSIP_GET_TSA_PORT(tsa) == cxt->port) {
      
      if (mapChangeEnable == TSIP_ENABLE_MAP_CHANGE)  {
        /* Any enabled timeslots (even if they are already enabled) must be set
               * in the timeslot di-bit map. */
        if (TSIP_GET_TSA_STATE(tsa) != TSIP_TS_STATE_DISABLED)  
          cslTsipEnableTimeslot (cxt, TSIP_GET_TSA_LINK(tsa), TSIP_GET_TSA_TS(tsa),
                               TSIP_GET_TSA_COMPAND(tsa)); 
      } else {
        if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_ENABLED)  
          cslTsipEnableTimeslot (cxt, TSIP_GET_TSA_LINK(tsa), TSIP_GET_TSA_TS(tsa),
                               TSIP_GET_TSA_COMPAND(tsa)); 
      }
    }
    tsa = tsa->next;
  }

} /* tsip_build_channel_map */


/************************************************************************************
 * FUNCTION PURPOSE: Perform a binary search
 ************************************************************************************
 * DESCRIPTION: A simple binary search. The index of the match is returned, -1 if
 *              no match is found
 ************************************************************************************/
int16_t tsip_bin_search (uint16_t v, uint16_t *sa, int16_t n)
{
  int16_t  onestep = 0;
  int16_t  p = (n+1) / 2;
  int16_t  step = (p+1)/2;
  uint16_t value;

  if (n == 0)
    return (-1);
  else if (n == 1)
    p = 0;


  do  {

    value = sa[p];
    if (value == v)
      return (p);


    if (value > v)  
      p = p - step;
    else
      p = p + step;

    if (p < 0)
      p = 0;

    if (p >= n)
      p = n-1;

    if (step == 1)
      onestep += 1;

    step = (step + 1) / 2;

  } while (onestep < 4);

  return (-1);

} /* tsip_bin_search */
      

  


/************************************************************************************
 * FUNCTION PURPOSE: Locate the data offset for a timeslot based on the channel maps
 ************************************************************************************
 * DESCRIPTION: Runs through the timeslot array and locates the data offset for
 *              each enabled channel.
 ************************************************************************************/
void tsip_get_data_offsets (tsipSharedDir_t  *sdir, tsipDir_t *dir, cslTsipContext_t *cxt, int16_t pendingStateCtl, int16_t mapChangeEnable)
{
  tsipTsArray_t *tsa;

  uint16_t tsVals[TSIP_MAX_TIMESLOTS];
  uint16_t offsets[TSIP_MAX_TIMESLOTS];
  int16_t  n;
  int16_t  idx;

  /* Get the offset maps for all active timeslots */
  n = cslTsipCreateOffsetMaps (cxt, tsVals, offsets, TSIP_MAX_TIMESLOTS);

  /* Determine the buffer offset for each timeslot. */
  tsa = dir->tsArray;
  while (tsa != NULL)  {

    if ( mapChangeEnable == TSIP_ENABLE_MAP_CHANGE) {
      if (pendingStateCtl == TSIP_TS_ENABLE_PENDING) {
        if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_PENDING_ENABLE)
          TSIP_SET_TSA_STATE(tsa, TSIP_TS_STATE_ENABLED);
      }
      if (TSIP_GET_TSA_STATE(tsa) != TSIP_TS_STATE_DISABLED)   {
        idx = tsip_bin_search (CSL_TSIP_CREATE_OFFSET_MAPVAL(TSIP_GET_TSA_TS(tsa), TSIP_GET_TSA_LINK(tsa)), tsVals, n);
        if (idx == -1)  {
          tsa->dataOffset = 0;
        }  else  {
          tsa->dataOffset = offsets[idx];
        }
      }
    }
    else {
      if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_ENABLED)   {
        idx = tsip_bin_search (CSL_TSIP_CREATE_OFFSET_MAPVAL(TSIP_GET_TSA_TS(tsa), TSIP_GET_TSA_LINK(tsa)), tsVals, n);
        if (idx == -1)  {
          tsa->dataOffset = 0;
        }  else  {
          tsa->dataOffset = offsets[idx];
        }
      }
    }
    tsa = tsa->next;
  }

  /* Determine the buffer offset for each timeslot in pending activate queue */
  tsa = sdir->tsPendingDeactivate;
  while (tsa != NULL)  {
    if ( mapChangeEnable == TSIP_ENABLE_MAP_CHANGE) {
      if (TSIP_GET_TSA_STATE(tsa) != TSIP_TS_STATE_DISABLED)   {
        idx = tsip_bin_search (CSL_TSIP_CREATE_OFFSET_MAPVAL(TSIP_GET_TSA_TS(tsa), TSIP_GET_TSA_LINK(tsa)), tsVals, n);
        if (idx == -1)  {
          tsa->dataOffset = 0;
        }  else  {
          tsa->dataOffset = offsets[idx];
        }
      }
    } else {
        if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_ENABLED)   {
          idx = tsip_bin_search (CSL_TSIP_CREATE_OFFSET_MAPVAL(TSIP_GET_TSA_TS(tsa), TSIP_GET_TSA_LINK(tsa)), tsVals, n);
          if (idx == -1)  {
            tsa->dataOffset = 0;
          }  else  {
            tsa->dataOffset = offsets[idx];
          }
      }
    }
    tsa = tsa->next;

  }

} /* tsip_get_data_offsets */


/*********************************************************************************
 * FUNCTION PURPOSE: increment the channel ID
 *********************************************************************************
 * DESCRIPTION: Adds a constant to the CnID field. Two is added since the 
 *              lsb is used to choose which context will be active. The actual
 *              value is not really used by this driver.
 *********************************************************************************/
uint16_t tsip_inc_cnid (uint16_t cnid)
{

  cnid = (cnid + 2) & TSIP_CNID_INC_MASK;
  return (cnid);

} /* tsip_inc_cnid */

/*********************************************************************************
 * FUNCTION PURPOSE: Creates a new transmit context and provides it to the csl.
 *********************************************************************************
 * DESCRIPTION: The new maps are formed by examining the state of each timeslot
 *              element, and marking the channel enable contexts and dma
 *              size contexts in the peripheral.
 *
 *              growDown effects the mapping of base address.
 *
 *              if growDown is FALSE, then baseRef is the new base address to use.
 *              if growDown is TRUE, then baseRef is the highest address to be used
 *              after the remapping.
 *
 *
 *
 *              ****** Runs from the tsip ISR  *******
 *********************************************************************************/
int16_t tsip_tx_provide_new_context (tsipShared_t *sTsip, tsip_t *tsip, 
                                         unsigned short growDown, uint16_t baseRef, 
                                         int16_t mapChangeEnable)
{
  cslTsipContext_t    cxt;
  uint32_t              newCnid;
  tsipDmaBuffer_t *sDma;
  uint32_t              baseAddress;

#ifdef TSIP_CORE_STAGGER
  uint32_t frfcCurrent;
  uint32_t frfcFrame;
#endif

  /* Request a free context from the csl, and have the csl clear the channel map */
  cslTsipGetFreeTxContext (tsip->port, tsip->tx.channel, &cxt);

  /* Failure to get the context can cause timeslot data misdirection */
  if (cxt.port == -1)  {
    tsip->err.txContextGetFail += 1;
    return (-1);
  }

  /* Determine which of the dma memory structures is not in use, and
   * set pointer p2 to point to it */
  if (tsip->tx.dmap1 == &(tsip->tx.dma1))
    tsip->tx.dmap2 = &(tsip->tx.dma2);
  else
    tsip->tx.dmap2 = &(tsip->tx.dma1);

  /* Enable any pending channels and create the channel map */
  tsip_build_channel_map (&(sTsip->sharedTx), &tsip->tx, &cxt, TSIP_TS_ENABLE_PENDING, mapChangeEnable);

  /* Get the offsets for each channel */
  tsip_get_data_offsets (&(sTsip->sharedTx), &tsip->tx, &cxt, TSIP_TS_REMAIN_PENDING, mapChangeEnable);

  /* Get the frame size and write it into the context */
  tsip->tx.dmap2->frameSize  = (tsipReg) cslTsipGetAndSetFrameSize (&cxt);
  tsip->tx.dmap2->frameAlloc = tsip->tx.dmap2->frameSize;

  sDma = &sTsip->sharedTx.dmaBuffer[tsip->port];
  sDma->frameAlloc = tsip->tx.dmap2->frameAlloc;

  tsip->tx.dmap2->baseAddressLocal = (tsipReg) (sTsip->sharedTx.memBase + baseRef);
  sDma->baseOffset = baseRef;

  if (growDown)  {
    tsip->tx.dmap2->baseAddressLocal -= (tsip->tx.dmap2->frameAlloc * tsip->tx.frameCount);
    sDma->baseOffset                 -= (tsip->tx.dmap2->frameAlloc * tsip->tx.frameCount);
  }

  baseAddress                      = GLOBAL_ADDR (tsip->tx.dmap2->baseAddressLocal);
  cslTsipSetBaseAndAlloc (&cxt, baseAddress, tsip->tx.dmap2->frameAlloc);


  /* Update the Channel ID. This will enable the new map at the next super-frame.
   * The csl will modify the lsb to match the correct context */
  cxt.cnid = tsip_inc_cnid (cxt.cnid);
  newCnid = cslTsipNewTxContext (&cxt);
  tsip->tx.ExptdCnIDFrfc = TSIP_SET_CNID(tsip->tx.ExptdCnIDFrfc, (newCnid >> 8) & 0x0ff);

#ifdef TSIP_CORE_STAGGER


  /* If the DMA has proceeded accross the sub-frame boundary provide the 
   * new context now. Otherwise store it for the next ISR. 
   * Note that the tsip instance frfc expected will always be the value at the
   * base of the buffer, and the dma will be in the second half of the buffer.
   * The check is to see if the frfc has crossed the 3/4 mark */
  frfcCurrent = cslTsipReadTxFrfc (tsip->port);
  frfcFrame   = TSIP_GET_FRFC(tsip->tx.ExptdCnIDFrfc + ((tsip->subFrame >> 1) * 3));

  if (frfcCurrent >= frfcFrame)  {
    cslTsipSendNewTxContext (tsip->port, newCnid, tsip->tx.channel);

  }  else  {

    tsip->stagger.txChEn = newCnid;
    tsip->stagger.mapChangeState |= TSIP_MAP_STATE_CHANGE_TX_CHANGE;

  }

#else

  cslTsipSendNewTxContext (tsip->port, newCnid, tsip->tx.channel);  

#endif

  return (0);


} /* tsip_tx_provide_new_context */

/**********************************************************************************
 * FUNCTION PURPOSE: Creates a new received context and provides it to the csl
 **********************************************************************************
 * DESCRIPTION: Creates the new enable maps, provides them to the peripheral.
 *              Unlike the tx direction the maps are not recomputed. Returns
 *              the new frame allocation size.
 *
 *              growDown effects the mapping of base address.
 *
 *              if growDown is FALSE, then baseRef is the new base address to use.
 *              if growDown is TRUE, then baseRef is the highest address to be used
 *              after the remapping.
 *
 *              ******** Runs from the tsip ISR ********
 **********************************************************************************/
int16_t tsip_rx_provide_new_context (tsipShared_t *sTsip, tsip_t *tsip, 
                                         unsigned short growDown, uint16_t baseRef, 
                                         int16_t mapChangeEnable)
{
  cslTsipContext_t    cxt;
  tsipDmaBuffer_t *sDma;
  uint32_t              baseAddress;
  uint32_t              chEn;

#ifdef TSIP_CORE_STAGGER
  uint32_t frfcCurrent;
  uint32_t frfcFrame;
#endif

  /* Request a free context from the csl and have the csl clear the channel map */
  cslTsipGetFreeRxContext (tsip->port, tsip->rx.channel, &cxt);

  /* Record a failure */
  if (cxt.port == -1)  {
    tsip->err.rxContextGetFail += 1;
    return (-1);
  }

  /* Determine which of the dma memory structures is not in use, and
   * set pointer p2 to point to it */
  if (tsip->rx.dmap1 == &(tsip->rx.dma1))
    tsip->rx.dmap2 = &(tsip->rx.dma2);
  else
    tsip->rx.dmap2 = &(tsip->rx.dma1);

  /* Create the map, send to the peripheral, but leave timeslots in the
   * pending state until the cnid changes.  */
  tsip_build_channel_map (&(sTsip->sharedRx), &tsip->rx, &cxt, TSIP_TS_REMAIN_PENDING, mapChangeEnable);

  /* Get the frame size and write it to the context. This is not really required
   * for the rx direction, but it could aid in any initial debugging */
  tsip->rx.dmap2->frameSize        = (tsipReg) cslTsipGetAndSetFrameSize (&cxt);
  tsip->rx.dmap2->frameAlloc       = tsip->rx.dmap2->frameSize;

  sDma                             = &sTsip->sharedRx.dmaBuffer[tsip->port];
  sDma->frameAlloc                 = tsip->rx.dmap2->frameAlloc;
  sDma->baseOffset                 = baseRef;

  tsip->rx.dmap2->baseAddressLocal = (tsipReg) (sTsip->sharedRx.memBase + baseRef);

  if (growDown)  {
    tsip->rx.dmap2->baseAddressLocal -= (tsip->rx.dmap2->frameAlloc * tsip->rx.frameCount);
    sDma->baseOffset                 -= (tsip->rx.dmap2->frameAlloc * tsip->rx.frameCount);
  }

  baseAddress                      = GLOBAL_ADDR (tsip->rx.dmap2->baseAddressLocal);
  cslTsipSetBaseAndAlloc (&cxt, baseAddress, tsip->rx.dmap2->frameAlloc);


  /* Update the channel ID. This will enable the new map at the next super-frame.
   * the csl will modify the lsb to match the correct context. The expted cnid
   * is NOT updated for rx. The driver will react to changes in cnid when they
   * are seen in the dma buffers. */
  cxt.cnid = tsip_inc_cnid (cxt.cnid);
  chEn = cslTsipNewRxContext (&cxt);

#ifdef TSIP_CORE_STAGGER

  /* If the DMA has proceeded accross the sub-frame boundary provide the 
   * new context now. Otherwise store it for the next ISR. 
   * Note that the tsip instance frfc expected will always be the value at the
   * base of the buffer, and the dma will be in the second half of the buffer.
   * The check is to see if the frfc has crossed the 3/4 mark */
  frfcCurrent = cslTsipReadRxFrfc (tsip->port);
  frfcFrame   = TSIP_GET_FRFC(tsip->rx.ExptdCnIDFrfc + ((tsip->subFrame >> 1) * 3));

  if (frfcCurrent >= frfcFrame)  {
    cslTsipSendNewRxContext (tsip->port, chEn, tsip->rx.channel);

  }  else  {

    tsip->stagger.rxChEn = chEn;
    tsip->stagger.mapChangeState |= TSIP_MAP_STATE_CHANGE_RX_CHANGE;

  }


#else
  
  cslTsipSendNewRxContext (tsip->port, chEn, tsip->rx.channel);

#endif


  return (0);

} /* tsip_rx_provide_new_context */


/***********************************************************************************
 * FUNCTION PURPOSE: Recompute the rx maps.
 ***********************************************************************************
 * DESCRIPTION: Unlike in the tx direction, in the rx direction the isr code
 *              reacts to a map change. This function is called when the CnID
 *              shows a change in the map ID. This function will find recompute
 *              the local maps based on the tsip channel enable map.
 *
 *              ******** Runs from the tsip ISR ********
 ***********************************************************************************/
void tsip_update_rx_maps (tsipShared_t *sTsip, tsip_t *tsip)
{
  cslTsipContext_t cxt;

  /* Request the active context from the csl */
  cslTsipGetActiveRxContext (tsip->port, tsip->rx.channel, &cxt);

  /* Failure to get the context can cause timeslot data misdirection */
  if (cxt.port == -1)  {
    tsip->err.rxActiveContextGetFail += 1;
    return;
  }

  /* Get the offsets for each channel */
  tsip_get_data_offsets (&(sTsip->sharedRx), &tsip->rx, &cxt, TSIP_TS_ENABLE_PENDING, TSIP_ENABLE_MAP_CHANGE);

} /* tsip_update_rx_maps */


/***********************************************************************************
 * FUNCTION PURPOSE: Determine which dma half to use.
 ***********************************************************************************
 * DESCRIPTION: Looks at the free running frame count at the base and mid point
 *              of the DMA. The DMA should be active in the half with the latest
 *              value. The peripheral frfc will contain the latest value and is used
 *              as a reference to prevent any wrapping problems.
 ***********************************************************************************/
#define TSIP_DELTA(x,y)   ((x)>(y) ? (x)-(y) : (y)-(x))
void tsip_dma_sync (tsipDir_t *dir)
{

  int16_t    sFrameSize;
  uint32_t  v1;
  uint32_t  v2;
  uint32_t  pu;
  uint32_t  nFrfc;
  uint32_t *p;
  int16_t i;

  /* Determine the number of frames in a super-frame */
  sFrameSize = (int16_t) (dir->frameCount >> 1);

  /* Look for the discontinuity in the frfc stream. This will point to the
   * active dma half. The reference value is the base value */
  pu = (uint32_t)(dir->dmap1->baseAddressLocal+dir->dmap1->frameSize - sizeof (uint32_t));
  p  = (uint32_t *)pu;
  v1 = (*p & 0x00ffffff);
  pu += dir->dmap1->frameAlloc;

  /* On exit from the loop, and possible the following correction, the
   * value i will contain the frame number where the next sample will be
   * placed or read from the DMA */
  for (i = 1; i < dir->frameCount; i++)  {
    p = (uint32_t *)pu;
    v2 = (*p & 0x00ffffff);
    pu += dir->dmap1->frameAlloc;

    if (v2 != ((v1 + 1) & 0x00ffffff))
      break;

    v1 = v2;
  }

  if (i == dir->frameCount)
    i = 0;

  if (i < sFrameSize)
    dir->offset = 1;
  else
    dir->offset = 0;

  nFrfc = *((uint32_t *)(dir->dmap1->baseAddressLocal + (dir->offset * dir->dmap1->frameAlloc * sFrameSize) + dir->dmap1->frameAlloc - sizeof(uint32_t)));
  nFrfc = nFrfc & 0x00ffffff;
  dir->ExptdCnIDFrfc = TSIP_SET_FRFC(dir->ExptdCnIDFrfc, nFrfc);


} /* tsip_dma_sync */

#undef TSIP_DELTA


/***********************************************************************************
 * FUNCTION PURPOSE: resync the foreground operation to match the tsip peripheral
 ***********************************************************************************
 * DESCRIPTION: The free running frame counter from the peripheral is checked and
 *              aligned with the frfc marker in the dma memory. The currently active
 *              DMA half if found, and the isr operation is directed to the half
 *              not currently active.
 *
 *              ******* Runs from the tsip ISR ********
 ***********************************************************************************/
void tsip_resync_rx (tsip_t *tsip)
{
  /* Sync to the dma */
  tsip_dma_sync (&(tsip->rx));

} /* tsip_resync_rx */


/*************************************************************************************
 * FUNCTION PURPOSE: resync the foreground operation to match the tsip peripheral
 *************************************************************************************
 * DESCRIPTION: Analogous to the rx version, the free running frame count is used
 *              to find the DMA buffer half which is not currently being used
 *              by the DMA.
 *
 *              ******** Runs from the tsip ISR ********
 *************************************************************************************/
void tsip_resync_tx (tsip_t *tsip)
{
  /* Sync to the dma */
  tsip_dma_sync (&(tsip->tx));

} /* tsip_resync_tx */

/************************************************************************************
 * FUNCTION PURPOSE: Start the DMA for a port
 ************************************************************************************
 * DESCRIPTION: The transmit buffer is initialized with the frame counters
 *              and the DMA channel is enabled.
 ************************************************************************************/
void tsipStartDma (tsipShared_t *sTsip, tsip_t *tsip)
{
  cslTsipChEnable_t entx;
  cslTsipChEnable_t enrx;
  uint32_t wAddr;
  uint32_t *tframe;
  int16_t i;

  /* Initialize the tx dma buffer */
  for (i = 0; i < tsip->tx.frameCount; i++)  {
     wAddr  = tsip->tx.dmap1->baseAddressLocal + (i * tsip->tx.dmap1->frameAlloc);
     tframe = (uint32_t *)(wAddr);
    *tframe = tsip->tx.ExptdCnIDFrfc + i;
     wAddr  = wAddr + tsip->tx.dmap1->frameSize - TSIP_FRAME_FOOTER_SIZE;
     tframe = (uint32_t *)(wAddr);
    *tframe = tsip->tx.ExptdCnIDFrfc + i;
  }

  tsipLoadDmaEnc (tsip, &entx, &enrx);
        
  tsip->err.txChEnabled = cslTsipEnableTxChannel (&entx);
  tsip->err.rxChEnabled = cslTsipEnableRxChannel (&enrx);
  sTsip->enableMap |= (1 << tsip->port);

} /* tsipStartDma */

/*********************************************************************************
 * FUNCTION PURPOSE: Determine the size requirements for a dma buffer
 *********************************************************************************
 * DESCRIPTION: The size requirements for single frame are determined.
 *              This function assumes that all timeslot data is 8 bits. If
 *              16 bit timeslots need to be supported the timeslots will have
 *              to be arranged in sequential order to allow for gapping
 *              that may be required when 8 and 16 bit timeslots are interleaved.
 *
 *              The portability of this is questionable. Adding one to the frame
 *              size for every element is also assuming the unit of frame size is
 *              bytes.
 **********************************************************************************/
uint16_t tsip_get_size_req (tsipTsArray_t *ts)
{
  uint16_t framesize = TSIP_FRAME_MIN_SIZE;

  while (ts != NULL)  {
    framesize += 1;
    ts = ts->next;
  }

  framesize = TSIP_FRAME_ROUND_SIZE(framesize);

  return (framesize);

} /* tsip_get_size_req */

/*************************************************************************************
 * FUNCTION PURPOSE: Move a sequence of timeslot structures from one list to another
 *************************************************************************************
 * DESCRIPTION: All elements from the source chain which have the matching port number
 *              are removed and attached to the dest chain, with new status status.
 *************************************************************************************/
void tsip_ts_chain_move (tsipTsArray_t **source, tsipTsArray_t **dest, int16_t port, uint16_t state)
{
  tsipTsArray_t *ts;
  tsipTsArray_t *prev;
  tsipTsArray_t *next;
  int16_t  iport;

  ts   = *source;
  prev = NULL;

  while (ts != NULL)  {

    next = ts->next;

    iport = TSIP_GET_TSA_PORT(ts);

    if (iport == port)  {

      /* Remove the element from the current chain */
      tsip_remove_ts_link (ts, prev, source);
      TSIP_SET_TSA_STATE(ts, state);

      /* Attach the element to the destination chain */
      tsip_add_ts_link (ts, dest);

    }  else  {
      prev = ts;
    }

    ts   = next;
  }

} /* tsip_ts_chain_move */

/*********************************************************************************
 * FUNCTION PURPOSE: Move a sequence of timeslots from one list to another
 *********************************************************************************
 * DESCRIPTION: All elements in the source chain in the matching state are
 *              moved to the dest chain, with the status changed
 *********************************************************************************/
void tsip_ts_chain_move_by_state (tsipTsArray_t **source, tsipTsArray_t **dest, 
                                      uint16_t state, uint16_t newState)
{
  tsipTsArray_t *ts;
  tsipTsArray_t *prev;
  tsipTsArray_t *next;

  ts   = *source;
  prev = NULL;

  while (ts != NULL)  {

    next = ts->next;

    if (TSIP_GET_TSA_STATE(ts) == state)  {

      /* remove the element from th current chain */
      tsip_remove_ts_link (ts, prev, source);
      TSIP_SET_TSA_STATE(ts, newState);

      /* Attach the element to the destination chain */
      tsip_add_ts_link (ts, dest);

    }  else  {
      prev = ts;
    }

    ts = next;

  }
      
} /* tsip_ts_chain_move_by_state */

/*********************************************************************************
 * FUNCTION PURPOSE: Create the command sequence for timeslot disables.
 *********************************************************************************
 * DESCRIPTION: When timeslots are disabled the base address of the dma buffer
 *              is left unchanged, and the frame allocation (and frame size)
 *              are reduced if required. Because of this it is never necessary
 *              to change the base address for other ports.
 *
 *              The procedure for disable is to determine which port
 *              the most recent disable is for, then get the count
 *              of active timeslots. This determines the frame allocation,
 *              which is saved in the shared context. The command queue
 *              is filled with one command, which is resize frame smaller. This
 *              command is issued even if the frame allocation doesn't change,
 *              since it requires a context change (timeslots disabled).
 *
 *              All of the pending disables in the queue are done for the
 *              one port which has priority.
 *              
 **********************************************************************************/
tsipReturn_t tsip_com_seq_deactivate (tsipDrvInst_t *tsipDrvInst, tsipSharedDir_t *sdir)
{
  tsipTsArray_t   *tsLast;
  tsip_t          *tsip;
  tsipDmaBuffer_t *dmaBuffer;
  tsipDir_t       *dir;
  int16_t            port;
  tsipShared_t    *sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;

  /* The element at the end of the chain has priority */
  tsLast = tsip_get_last_elem (sdir->tsPendingDeactivate);

  /* If there was nothing in the list there is nothing to do */
  if (tsLast == NULL)
    return (tsip_OK);

  /* Determine which port this disable applies to. Get the tsip structure
   * and the dma buffer information for this port. */
  port      = TSIP_GET_TSA_PORT(tsLast);
  tsip      = tsipFromPort (tsipDrvInst, port);
  dmaBuffer = (&sdir->dmaBuffer[port]);
  if (sdir->dirId == TSIP_SHARED_DIR_TX)
    dir = &(tsip->tx);
  else
    dir = &(tsip->rx);

  /* Get the new required size for the dma buffer based on all enabled timeslots
   * for this port.  */
  dmaBuffer->frameAlloc = tsip_get_size_req (dir->tsArray);

  /* Remove the timeslot info from the deallocate list, along with any other timeslot
   * deactivations for the same port */
  tsip_ts_chain_move (&(sdir->tsPendingDeactivate), &(sTsip->tsArrayFree), port, TSIP_TS_STATE_DISABLED);

  /* Create the command queue */
  TSIP_FORM_COMMAND(sdir->commandStack[0],0,port,TSIP_COMMAND_DELAY,4);
  TSIP_FORM_COMMAND(sdir->commandStack[1],0,port,TSIP_COMMAND_RESIZE_FRAME_SMALLER,0);
  sdir->stackp = 2;

  return (tsip_OK);

} /* tsip_com_seq_deactivate */



/*******************************************************************************************
 * FUNCTION PURPOSE: Create a command sequence for timeslot enables
 *******************************************************************************************
 * DESCRIPTION: The command sequence to enable timeslots is determined. 
 *
 *              !!! The delay after resize is required !!! Otherwise a second
 *                  set of pending enables can get folded into the current enable request.
 *******************************************************************************************/
tsipReturn_t tsip_com_seq_activate (tsipDrvInst_t *tsipDrvInst, tsipSharedDir_t *sdir)
{
  tsipTsArray_t   *tsLast;
  tsipDir_t       *dir;
  tsip_t          *tsip;
  tsipDmaBuffer_t *dmaBuffer;
  int16_t                port;
  int16_t                newFrameSize;
  int16_t                req;
  int16_t                avail;
  tsip_t          *tsipn;
  tsipDir_t       *dirn;
  uint16_t               required[TSIP_MAX_N_PORTS];
  uint16_t               newFree[TSIP_MAX_N_PORTS];
  uint16_t               newBase[TSIP_MAX_N_PORTS];
  tsipCommand_t    commands[TSIP_MAX_N_PORTS+3];
  uint16_t               oldPortAlloc;
  uint16_t               totalFree;
  uint16_t               idiv;
  uint16_t               div;
  int16_t                i,j;
  uint16_t               freeSpace;
  uint16_t               totalReq;
  tsipShared_t        *sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;

  /* The element at the end of the chain has priority */
  tsLast = tsip_get_last_elem (sdir->tsPendingActivate);

  /* If there was nothing in the list there is nothing to do */
  if (tsLast == NULL)
    return (tsip_OK);

  /* Determine which port this disable applies to. Get the tsip structure
   * and the dma buffer information for this port. */
  port      = TSIP_GET_TSA_PORT(tsLast);
  tsip      = tsipFromPort (tsipDrvInst, port);
  dmaBuffer = (&sdir->dmaBuffer[port]);
  if (sdir->dirId == TSIP_SHARED_DIR_TX)
    dir = &(tsip->tx);
  else
    dir = &(tsip->rx);

  /* Remove the timeslot info from the allocate list, along with any other timeslot
   * activations for the same port, and put them in the timeslot list for the port,
   * in pending enable state. */
  tsip_ts_chain_move (&(sdir->tsPendingActivate), &(dir->tsArray), port, TSIP_TS_STATE_PENDING_ENABLE);

  /* Get the new required size for the dma buffer based on all enabled timeslots
   * for this port.  */
  newFrameSize = tsip_get_size_req (dir->tsArray);

  /* If the newFrameSize is the same (or smaller, which should never happen) then
   * the current frame size, keep the base address the same and add the new timeslots */
  if (newFrameSize <= dmaBuffer->frameAlloc)  {

    /* Create the command queue */
    TSIP_FORM_COMMAND(sdir->commandStack[0],0,port,TSIP_COMMAND_DELAY,4);
    TSIP_FORM_COMMAND(sdir->commandStack[1],0,port,TSIP_COMMAND_RESIZE_FRAME_SMALLER,0);
    sdir->stackp = 2;

    return (tsip_OK);

  }

  /* The new frame is larger then the old frame. Check if there is enough free room
   * available below the current base address to resize down */
  if (port == 0)
    freeSpace = dmaBuffer->baseOffset;
  else
    freeSpace = dmaBuffer->baseOffset - (sdir->dmaBuffer[port-1].baseOffset + 
                (sdir->dmaBuffer[port-1].frameAlloc * dir->frameCount));

  avail = (dmaBuffer->frameAlloc * dir->frameCount) + freeSpace;
  req   = newFrameSize * dir->frameCount;

  if (avail >= req)  {

    /* The current port can grow downward in memory. No other ports need to
     * be changed */
    TSIP_FORM_COMMAND(sdir->commandStack[0],0,port,TSIP_COMMAND_DELAY,4);
    TSIP_FORM_COMMAND(sdir->commandStack[1],0,port,TSIP_COMMAND_RESIZE_FRAME_LARGER,0);
    sdir->stackp = 2;

    return (tsip_OK);

  }


  /* If there is not enough space to grow down, remap all the memories */
  totalReq = 0;
  for (i = 0; i < tsipDrvInst->nTsipPorts; i++)  {
    tsipn = tsipFromPort (tsipDrvInst, i);

    if (tsipn != NULL)  {
      if (sdir->dirId == TSIP_SHARED_DIR_TX)
        dirn = &(tsipn->tx);
      else
        dirn = &(tsipn->rx);

      if (i == port)  {
        required[i] = newFrameSize * dirn->frameCount;
        oldPortAlloc = sdir->dmaBuffer[i].frameAlloc * dirn->frameCount;
      }  else  {
        required[i] = sdir->dmaBuffer[i].frameAlloc * dirn->frameCount;
      }

    }  else  {

      required[i] = sdir->dmaBuffer[i].frameAlloc * TSIP_UNOPENED_PORT_FRAME_ALLOC;

    }

    totalReq = totalReq + required[i];
  }

  /* Make sure there is really enough space for everything */
  if (totalReq > sdir->lengthBytes)  {
    tsip_ts_chain_move_by_state (&(dir->tsArray), &(sTsip->tsArrayFree), TSIP_TS_STATE_PENDING_ENABLE, TSIP_TS_STATE_DISABLED);
    return (tsip_FAIL);
  }


  /* Reformat the map. Divide the remaining space by the number of ports,
   * and allocate this between the ports. Maintain the required alignments. */
  totalFree = sdir->lengthBytes - totalReq; 
  idiv = totalFree / tsipDrvInst->nTsipPorts;
  div = TSIP_FRAME_ROUND_SIZE(idiv);
  if (TSIP_FRAME_SIZE_REMAIN(idiv))
    div = div - TSIP_FRAME_ROUND_SIZE(1);  /* This prevents the total size exceeding the max */

  for (i = 0; i < tsipDrvInst->nTsipPorts-1; i++)  {
    newFree[i] = div;
    totalFree -= div;
  }
  newFree[tsipDrvInst->nTsipPorts-1] = totalFree;

  /* The requirements for each port and the free sizes are known. Convert
   * this to the new base addresses for each port */
  newBase[0] = newFree[0];
  for (i = 1; i < tsipDrvInst->nTsipPorts; i++) 
    newBase[i] = newBase[i-1] + required[i-1] + newFree[i];

  /* The calculations made above are what will be done after all
   * chages are made. It may be necessary to shift the current port
   * base up before it is resized. Because of that, compute what
   * the base address of the current ports memory will be given
   * the current allocation */
  newBase[port] = newBase[port] + required[port] - oldPortAlloc;

  /* Determine the sequence of event required to change from the
   * current bases to the new bases. Ports which increase in
   * base address must be done in decreasing port order, ports
   * which decrease must be done in increasing port order. */
  for (i = tsipDrvInst->nTsipPorts-1, j = 0; i >= 0; i--)  {
    if (newBase[i] > sdir->dmaBuffer[i].baseOffset)  {
      TSIP_FORM_COMMAND(commands[j],0,i,TSIP_COMMAND_SHIFT_BASE_HIGHER,newBase[i] - sdir->dmaBuffer[i].baseOffset);
      j += 1;
    }
  }

  for (i = 0; i < tsipDrvInst->nTsipPorts; i++)  {
    if ((port != i) && (newBase[i] < sdir->dmaBuffer[i].baseOffset))  {
      TSIP_FORM_COMMAND(commands[j],0,i,TSIP_COMMAND_SHIFT_BASE_LOWER,sdir->dmaBuffer[i].baseOffset - newBase[i]);
      j += 1;
    }
  }

  /* Add the command to resize the port */
  TSIP_FORM_COMMAND(commands[j],0,port,TSIP_COMMAND_RESIZE_FRAME_LARGER,0);
  j += 1;

  /* Add a delay after resize */
  TSIP_FORM_COMMAND(commands[j],0,port,TSIP_COMMAND_DELAY,4);
  j += 1;

  /* Put the commands on the stack in the order in which they execute */
  for (i = 0; i < j; i++)
    sdir->commandStack[i] = commands[j-1-i];

  sdir->stackp = j;

  return (tsip_OK);



} /* tsip_com_seq_activate */
    


/*************************************************************************************
 * FUNCTION PURPOSE: Execute the resize smaller command
 *************************************************************************************
 * DESCRIPTION: This command simply recalculates the frame allocation required
 *              base on active and pending timeslots, reforms the enable maps,
 *              then provides the context to the port. This command executes
 *              when timeslots are closed, or when a timeslot enable does not
 *              change the required allocation.
 *************************************************************************************/
tsipCommand_t tsip_com_smaller (tsipShared_t *sTsip, tsipCommand_t command, 
                                       tsip_t *tsip, tsipSharedDir_t *sdir)
{
  uint16_t base;
  int16_t  ret;

  /* For this command the base address is not changed from the current value */
  base = sdir->dmaBuffer[tsip->port].baseOffset;

  if (sdir->dirId == TSIP_SHARED_DIR_RX)  {
    ret = tsip_rx_provide_new_context (sTsip, tsip, FALSE, base, TSIP_ENABLE_MAP_CHANGE);
    tsip->rxRemap = TRUE;

  }  else  {
    ret = tsip_tx_provide_new_context (sTsip, tsip, FALSE, base, TSIP_ENABLE_MAP_CHANGE);

  }

  /* If the command failed, return the command so it will try again */
  if (ret != 0)
    return (command);

  /* Otherwise the command executed completely and can be closed */
  return (TSIP_COMMAND_NULL);

}

/**************************************************************************************
 * FUNCTION PURPOSE: Increase the frame allocation for the port
 **************************************************************************************
 * DESCRIPTION: This command recalculates the frame allocation to a larger size,
 *              and drops the base address of the port lower so that the same upper
 *              limit is kept.
 **************************************************************************************/
tsipCommand_t tsip_com_larger (tsipShared_t *sTsip, tsipCommand_t command, 
                                      tsip_t *tsip, tsipSharedDir_t *sdir)
{
  uint16_t top;
  uint16_t frameCount;
  int16_t  ret;

  if (sdir->dirId == TSIP_SHARED_DIR_RX)    {
    frameCount = tsip->rx.frameCount;
    tsip->rxRemap = TRUE;
  } else  {
    frameCount = tsip->tx.frameCount;
  }

  /* For this command the base address if relative to the current top */
  top = sdir->dmaBuffer[tsip->port].baseOffset + (sdir->dmaBuffer[tsip->port].frameAlloc * frameCount);


  if (sdir->dirId == TSIP_SHARED_DIR_RX)  {
    ret = tsip_rx_provide_new_context (sTsip, tsip, TRUE, top, TSIP_ENABLE_MAP_CHANGE);

  }  else  {
    ret = tsip_tx_provide_new_context (sTsip, tsip, TRUE, top, TSIP_ENABLE_MAP_CHANGE);

  }

  /* If the command failed, return the command so it will try again */
  if (ret != 0)
    return (command);

  /* Otherwise the command executed completely and can be closed */
  return (TSIP_COMMAND_NULL);

} /* tsip_com_larger */

/*************************************************************************************
 * FUNCTION PURPOSE: Reduce the base address for a port
 *************************************************************************************
 * DESCRIPTION: The base address of the port is lowered by the amount specified in
 *              the command.
 *************************************************************************************/
tsipCommand_t tsip_com_shift_down (tsipShared_t *sTsip, tsipCommand_t command, 
                                          tsip_t *tsip, tsipSharedDir_t *sdir)
{
  uint16_t base;
  int16_t  ret;

  /* For this command the base address is not changed from the current value */
  base = sdir->dmaBuffer[tsip->port].baseOffset;
  base = base - TSIP_READ_PARAM(command);
  
  if (sdir->dirId == TSIP_SHARED_DIR_RX)  {
    ret = tsip_rx_provide_new_context (sTsip, tsip, FALSE, base, TSIP_NO_MAP_CHANGE);
    tsip->rxRemap = FALSE;

  }  else  {
    ret = tsip_tx_provide_new_context (sTsip, tsip, FALSE, base, TSIP_NO_MAP_CHANGE);

  }

  /* If the command failed, return the command so it will try again */
  if (ret != 0)
    return (command);

  /* Otherwise the command executed completely and can be closed */
  return (TSIP_COMMAND_NULL);

} /* tsip_com_shift_down */


/*************************************************************************************
 * FUNCTION PURPOSE: Increase the base address of a port
 *************************************************************************************
 * DESCRIPTION: The base address of a port is increased. The most it can be increased
 *              is the size of the current frame allocation. This size is then
 *              subtracted from the command parameter, and if more increases
 *              are required the command is returned.
 *************************************************************************************/
tsipCommand_t tsip_com_shift_up (tsipShared_t *sTsip, tsipCommand_t command,
                                        tsip_t *tsip, tsipSharedDir_t *sdir)
{
  uint16_t  base;
  uint16_t  frameAlloc;
  uint16_t  adjust;
  int16_t   ret;
  uint32_t param;

  param      = TSIP_READ_PARAM(command);
  frameAlloc = sdir->dmaBuffer[tsip->port].frameAlloc;

  if (param > frameAlloc)
    adjust = frameAlloc;
  else
    adjust = param;

  base = sdir->dmaBuffer[tsip->port].baseOffset + adjust;

  if (sdir->dirId == TSIP_SHARED_DIR_RX)  {
    ret = tsip_rx_provide_new_context (sTsip, tsip, FALSE, base, TSIP_NO_MAP_CHANGE);
    tsip->rxRemap = FALSE;

  }  else  {
    ret = tsip_tx_provide_new_context (sTsip, tsip, FALSE, base, TSIP_NO_MAP_CHANGE);

  }

  /* If the command failed, return the command so it will try again */
  if (ret != 0)
    return (command);


  /* Subtract the increase from paramter. If it becomes 0 terminate the command, 
   * otherwise return the command with the reduced count */
  param = param - adjust;
  if (param == 0)
    return (TSIP_COMMAND_NULL);

  command = TSIP_SET_PARAM(command, param);
  return (command);

} /* tsip_com_shift_up */
  

/**************************************************************************************
 * FUNCTION PURPOSE: Process a command
 **************************************************************************************
 * DESCRIPTION: The specified command is processed. The port is enabled.
 **************************************************************************************/
tsipCommand_t tsip_process_command (tsipShared_t *sTsip, tsipCommand_t command, 
                                           tsip_t *tsip, tsipSharedDir_t *sdir)
{
  int16_t action;
  uint32_t param;
  tsipCommand_t com;


  action = TSIP_READ_COMMAND(command);
  param  = TSIP_READ_PARAM(command);


  switch (action)  {

    case TSIP_COMMAND_RESIZE_FRAME_SMALLER:
      com = tsip_com_smaller (sTsip, command, tsip, sdir);
      break;

    case TSIP_COMMAND_RESIZE_FRAME_LARGER:
      com = tsip_com_larger (sTsip, command, tsip, sdir);
      break;

    case TSIP_COMMAND_SHIFT_BASE_LOWER:
      com = tsip_com_shift_down (sTsip, command, tsip, sdir);
      break;

    case TSIP_COMMAND_SHIFT_BASE_HIGHER:
      com = tsip_com_shift_up (sTsip, command, tsip, sdir);
      break;

    case TSIP_COMMAND_DELAY:
      if (param > 1)  
        com = TSIP_SET_PARAM(command, param-1);
      else
        com = TSIP_COMMAND_NULL;
      break;

    default: 
        com = TSIP_COMMAND_NULL;
        break;

  }

  return (com);

} /* tsip_process_command */
  


/*******************************************************************************
 * FUNCTION PURPOSE: Change configuration for a disabled port
 *******************************************************************************
 * DESCRIPTION: A request was made to change the shared allocation for a
 *              disabled port. The change can be made immediately for all
 *              commands, with no changes in any peripheral settings. The
 *              only two commands that should ever be sen are shift base lower
 *              or shift base higher.
 *******************************************************************************/
tsipCommand_t tsip_process_command_dis_port (tsipCommand_t command, int16_t port, tsipSharedDir_t *sdir)
{
  tsipDmaBuffer_t *dmaBuffer;
  int16_t   action;
  uint32_t param;


  dmaBuffer = &(sdir->dmaBuffer[port]);
  action    = TSIP_READ_COMMAND(command);
  param     = TSIP_READ_PARAM(command);

  switch (action)  {

    case TSIP_COMMAND_SHIFT_BASE_LOWER:
      dmaBuffer->baseOffset = dmaBuffer->baseOffset - param;
      break;

    case TSIP_COMMAND_SHIFT_BASE_HIGHER:
      dmaBuffer->baseOffset = dmaBuffer->baseOffset + param;
      break;

    case TSIP_COMMAND_DELAY:
      if (param > 1)  {
        command = TSIP_SET_PARAM(command, param-1);
        return (command);
      }
      break;
      

  }

  return (TSIP_COMMAND_NULL);

} /* tsip_process_command_dis_port */

/*********************************************************************
 * FUNCTION PURPOSE: Returns version number
 ********************************************************************/
uint32_t Tsip_getVersion 
(
  void
)
{
  return tsip_LLD_VERSION_ID;
} /* TSIP_getVersion */

/*********************************************************************
 * FUNCTION PURPOSE: Returns version string
 ********************************************************************/
const char* Tsip_getVersionStr
(
  void
)
{
  return TSIPLLDVersionStr;
} /* TSIP_getVersionStr */
