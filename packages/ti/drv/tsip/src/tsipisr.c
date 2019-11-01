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
 *  File Name: tsipisr.c
 *
 *  Source for the TSIP Driver: TSIP ISR functions
 *
 **/

#include "tsip.h"
#include "tsiploc.h"
#include "tsipcsl.h"
#include "tsip_osal.h"

/*********************************************************************************
 * FUNCTION PURPOSE: Create a command sequence to change TSIP configuration
 *********************************************************************************
 * DESCRIPTION: The sequence of events required to reconfigure TSIP are
 *              computed and placed into the command queue.
 *********************************************************************************/
tsipReturn_t tsipCreateCommandSequence (tsipDrvInst_t *tsipDrvInst, tsipSharedDir_t *sdir)
{

  if (sdir->tsPendingDeactivate != NULL)
    return (tsip_com_seq_deactivate (tsipDrvInst, sdir));

  else if (sdir->tsPendingActivate != NULL)
    return (tsip_com_seq_activate (tsipDrvInst, sdir));

  return (tsip_OK);

} /* tsipCreateCommandSequence */

/*********************************************************************************
 * FUNCTION PURPOSE: Verifies the expected time in the frfc
 *********************************************************************************
 * DESCRIPTION: Checks the frfc value found in the dma buffer to that expected
 *              by the tsip context. Each frame is headed and followed by the
 *              CnID and FRFC, but only the second frame FRFC and the
 *              last FRAME in memory are checked. The element at the end
 *              of each frame is checked only. On transmit the FRFC at the
 *              start of the frame is not updated.
 *
 *              The first frame FRFC is not checked because it can be 
 *              over-written during a base address up shift in the tx
 *              direction.
 *********************************************************************************/
int16_t tsip_verify_frfc (tsipDir_t *tdir, uint32_t base, uint16_t sFrameSize)
{
  uint32_t *marka;    /* Where to find the base CnID/frfc */
  uint32_t *markb;    /* Where to find the end (but head) CnID/frfc */

  tsipReg frfcExpected;  /* The expected frfc       */
  tsipReg frfcActual;    /* The frfc actually found */

  /* The first mark is at the base of the buffer */
  /* Check FRFC at the end of the frame as Xmt side TSIP updates only the end one */
  marka = (uint32_t *)(base + tdir->dmap1->frameAlloc +  tdir->dmap1->frameSize - sizeof (uint32_t));

  /* The second mark is at the head of the last frame */
  markb = (uint32_t *) (base + (tdir->dmap1->frameAlloc * (sFrameSize - 1))+ tdir->dmap1->frameSize - sizeof (uint32_t));


  /* The first frfc should match the input expected value *. Note that the
   * expected value is in the first frame, and here the second frame is 
   * checked, which is why 1 is added. */
  frfcExpected = TSIP_GET_FRFC(tdir->ExptdCnIDFrfc) + 1;
  TSIP_FRFC_WRAP(frfcExpected);
  frfcActual   = TSIP_GET_FRFC(*marka);

  if (frfcExpected != frfcActual)  {
    return (TSIP_FRFC_FAIL);
  }

  /* The mark at the last sample should follow in sequence. */
  frfcExpected = frfcExpected + sFrameSize - 2;
  TSIP_FRFC_WRAP(frfcExpected);
  frfcActual   = TSIP_GET_FRFC(*markb);

  if (frfcExpected != frfcActual)  {
    return (TSIP_FRFC_FAIL);
  }

  /* Otherwise the frfc matches */
  return (TSIP_SUCCESS);

} /* tsip_verify_frfc */

/**********************************************************************************
 * FUNCTION PURPOSE: Read one or two bytes from an array, return as a uint16_t
 **********************************************************************************
 * DESCRIPTION: Reads one or two bytes from the specified address. As coded this
 *              requires the word to be equivalent of bytes. If that's not the
 *              case (55x porting) this will need to be called based on 
 *              core type.
 **********************************************************************************/
static inline tsipData_t tsip_read_bytes (uint8_t *wbase, int16_t sizeBytes)
{
  if (sizeBytes == 1)
    return ((tsipData_t)*wbase);

  else
    return (  ((tsipData_t)wbase[0] << 8)  | ((tsipData_t)wbase[1]) );
} /* tsip_read_bytes */

/**********************************************************************************
 * FUNCTION PURPOSE: Write one or two bytes to an array from a uint16_t
 **********************************************************************************
 * DESCRIPTION: Writes one or two bytes to the specified array. This is
 *              also assuming words are bytes. Must update to work if this is
 *              not the case.
 **********************************************************************************/
static inline void tsip_write_bytes (uint8_t *wbase, int16_t sizeBytes, tsipData_t value)
{
  wbase[0] = (value & 0x00ff);

  if (sizeBytes == 2)
    wbase[1] = (value >> 8);

}


#ifdef TSIP_CORE_STAGGER
/**********************************************************************************
 * FUNCTION PURPOSE: Check buffer frfc phasing
 **********************************************************************************
 * DESCRIPTION: A buffer is declared to be improperly phased only if the sequence
 *              of frfcs is continuous and wrong. 
 *
 *              Returns 0 if the phasing is correct or can not be proven wrong
 *                      1 if the phasing is proven wrong.
 **********************************************************************************/
uint32_t tsip_debug_wait_pos = 0;

int16_t tsip_phase_check (int16_t sFrameSize, uint32_t *base, uint32_t val, int16_t size32)
{
  int16_t i;
  uint32_t v1, v2;

  /* Do not declare a mis-phase unless there is continuity in the frfc */
  for (i = 1, v1 = base[0] & 0xf; i < sFrameSize; i++)  {
    v2 = base[i*size32] & 0xf;
    v1 = (v1 + 1) & 0xf;
    if (v1 != v2) {
      return (0);         /* Inconclusive - there is an frfc discontinuity */
    }
  }

  /* The frfcs have continuity. Verify the initial value. Only check the 3 lsb.
     to prevent detections on post correction while the system has
     the worng offset location. */
  v1  = base[0] & 0x7;
  val = val & 0x7;

  if (v1 != val)  {
    return (1);
  }

  return (0);
} /* tsip_phase_check */

/**********************************************************************************
 * FUNCTION PURPOSE: Verify the cross core staggering phase
 **********************************************************************************
 * DESCRIPTION: On every frfc failure the phasing of the DMA is verified. If it
 *              is not as expected then the DMA channel for this port and core
 *              are disabled and the channel is restarted in phase.
 *
 *              Note that false detections are possible on missed ISRs during
 *              a channel map transition when examining the frfc alone. To
 *              prevent this the pattern over an entire buffer quarter must
 *              be verified.
 *
 *              The recovery procedure:
 *
 *              1 - disable the DMA channel
 *              2 - clear the DMA channel maps
 *              3 - move all active timeslots to the pending activate list
 *              4 - reset the context to show no timeslots are enabled
 *              5 - remap the ISR to go to tsipStaggerStartIsr
 *              6 - re-enable the DMA
 *
 *              Returns 0 if phasing is correct on both tx and rx
 *                      1 if the phasing on either direction is invalid
 *
 **********************************************************************************/
void tsip_flush_commands (
  int16_t port, 
  int16_t *stackp_p, 
  tsipCommand_t *stack
)
{
  int16_t stackp = *stackp_p;
  int i,j;
  tsipCommand_t  command;
  int16_t cmd_port;

  for (i = 0, j = 0; i < stackp; i++) {
    command = stack[i];
    cmd_port = TSIP_READ_PORT(command);
    if (port != cmd_port) {
      stack[j++] = command;
    }
  }
  *stackp_p = j;
}

void tsip_restart (tsipShared_t *sTsip, tsip_t *tsip)
{
  uint32_t origTxFrfc, origRxFrfc;
  uint32_t minWait = tsip->subFrame >> 1;

  /* Restart both TX and RX, then re-run the same sync process as bootup */
  /* Stop both DMAs */
  cslTsipStopRxDma (tsip->port, tsip->rx.channel);
  cslTsipStopTxDma (tsip->port, tsip->tx.channel);
  origRxFrfc = cslTsipReadRxFrfc(tsip->port);
  origTxFrfc = cslTsipReadTxFrfc(tsip->port);
  /* Move all active timeslots to the pending activate list */
  tsip_ts_chain_move (&tsip->rx.tsArray, 
                          &sTsip->sharedRx.tsPendingActivate, 
                          tsip->port, 
                          TSIP_TS_STATE_PENDING_ENABLE);
  /* Move all pending deactivate to free, since they are already killed */
  tsip_ts_chain_move (&sTsip->sharedRx.tsPendingDeactivate, 
                          &sTsip->tsArrayFree, 
                          tsip->port, 
                          TSIP_TS_STATE_DISABLED);



  tsip->rx.dmap1 = tsip->rx.dmap2 = &tsip->rx.dma1;
  tsip->rx.dma2.frameAlloc   = 
    tsip->rx.dma1.frameAlloc = 
    tsip->rx.dma2.frameSize  = 
    tsip->rx.dma1.frameSize  = TSIP_FRAME_MIN_SIZE;
  tsip->rx.ExptdCnIDFrfc = 0;
  tsip->rx.ExptdCnIDFrfc = TSIP_SET_CNID(tsip->rx.ExptdCnIDFrfc, TSIP_INITIAL_RX_CNID);
  tsip->rx.offset = 0;

  /* Initialise the frame alloc in shared buffer */
  sTsip->sharedRx.dmaBuffer[tsip->port].frameAlloc = TSIP_FRAME_MIN_SIZE;
  
  /* Move all active  timeslots to the pending activate list */
  tsip_ts_chain_move (&tsip->tx.tsArray, 
                          &sTsip->sharedTx.tsPendingActivate, 
                          tsip->port, 
                          TSIP_TS_STATE_PENDING_ENABLE);
  /* Move all pending deactivate to free, since they are already killed */
  tsip_ts_chain_move (&sTsip->sharedTx.tsPendingDeactivate, 
                          &sTsip->tsArrayFree, 
                          tsip->port, 
                          TSIP_TS_STATE_DISABLED);

  tsip->tx.dmap1 = tsip->tx.dmap2 = &tsip->tx.dma1;
  tsip->tx.dma2.frameAlloc   = 
    tsip->tx.dma1.frameAlloc = 
    tsip->tx.dma2.frameSize  = 
    tsip->tx.dma1.frameSize  = TSIP_FRAME_MIN_SIZE;
  tsip->tx.ExptdCnIDFrfc     = 1;  /* TX requires prefetch, so data xmted on 2nd FS */
  tsip->tx.ExptdCnIDFrfc     = TSIP_SET_CNID(tsip->tx.ExptdCnIDFrfc, TSIP_INITIAL_TX_CNID);
  tsip->tx.offset = 0;

  /* Initialise the frame alloc in shared buffer */
  sTsip->sharedTx.dmaBuffer[tsip->port].frameAlloc = TSIP_FRAME_MIN_SIZE;

  /* Since there are long busy waits below that could trigger watchdog failure
   * or other "crash", tsip_debug_wait_pos indicates which busy wait is active
   * while tsip_debug_wait_start and tsip_debug_wait_now give an idea of how long
   * the loop waited for the "crash".
   * When removing the busy waits, this should be removed (since neither
   * watchdog or other "crash" will not occur.
   */
  tsip_debug_wait_pos = 1;
  while (((cslTsipReadRxFrfc(tsip->port) - origRxFrfc) & 0xffffff) < minWait);

  tsip_debug_wait_pos = 2;
  while (((cslTsipReadTxFrfc(tsip->port) - origTxFrfc) & 0xffffff) < minWait);
  
  tsip_debug_wait_pos = 0;
  /* Clear channel bitmaps */
  cslTsipClearChannel (tsip->port, tsip->tx.channel, tsip->rx.channel);
  tsipStartDma (sTsip, tsip);

  /* Reset stagger state machine then run it */
  tsip->stagger.syncState = TSIP_STAGGER_STATE_VERIFY_SYNC;
  tsip->stagger.mapChangeState = 0;
  tsip->stagger.isrChoice = TSIP_STAGGER_STAGGER_START_ISR;
  /* Dump command stacks */
  tsip_flush_commands (tsip->port, 
                           &sTsip->sharedRx.stackp,
                           sTsip->sharedRx.commandStack);
  tsip_flush_commands (tsip->port, 
                           &sTsip->sharedTx.stackp,
                           sTsip->sharedTx.commandStack);

}

int16_t tsip_verify_stagger_phase(tsipShared_t *sTsip, tsip_t *tsip)
{

  int16_t  rxsFrameSize;
  int16_t  txsFrameSize;

  uint32_t rxbase;
  uint32_t txbase;

  uint32_t rxVal;
  uint32_t txVal;

  int16_t rxsize32;
  int16_t txsize32;

  unsigned short rxFail = FALSE;
  unsigned short txFail = FALSE;



  rxsFrameSize = tsip->rx.frameCount >> 1;
  rxbase       = tsip->rx.dmap1->baseAddressLocal + (tsip->rx.offset * tsip->rx.dmap1->frameAlloc * rxsFrameSize) +
                 tsip->rx.dmap1->frameSize - sizeof(uint32_t);
  rxVal        = ((tsip->rx.channel & 1) << 2) + (tsip->rx.offset << 3);
  rxsize32     = tsip->rx.dmap1->frameAlloc / sizeof(uint32_t);

  txsFrameSize = tsip->tx.frameCount >> 1;
  txbase       = tsip->tx.dmap1->baseAddressLocal + (tsip->tx.offset * tsip->tx.dmap1->frameAlloc * txsFrameSize) +
                 tsip->tx.dmap1->frameSize - sizeof(uint32_t);
  txVal        = ((tsip->tx.channel & 1) << 2) + (tsip->tx.offset << 3) + 1;
  txsize32     = tsip->tx.dmap1->frameAlloc / sizeof(uint32_t);

  if (tsip_phase_check (rxsFrameSize, (uint32_t *)rxbase, rxVal, rxsize32) != 0) {
    rxFail = TRUE;
  }

  if (tsip_phase_check (txsFrameSize, (uint32_t *)txbase, txVal, txsize32) != 0) {
    txFail = TRUE;
  }

  if (rxFail || txFail) {
    tsip_restart (sTsip, tsip);
    return (1);
  }

  return (0);
} /* tsip_verify_stagger_phase */
#endif /* TSIP_CORE_STAGGER */

/**********************************************************************************
 * FUNCTION PURPOSE: Process the receive side of the tsip
 **********************************************************************************
 * DESCRIPTION: Verifies the free running frame counts, detects implemented
 *              map changes, copies data to the application and makes application
 *              callbacks, and initiates map changes
 **********************************************************************************/
void tsip_process_rx (tsipDrvInst_t *tsipDrvInst, tsip_t *tsip)
{
  uint16_t    CnID;          /* Channel n ID */
  uint32_t    base;          /* Base address of current buffer half (local address)  */
  uint8_t     *wbase;         /* Pointer to data in smallest element size             */
  int16_t     sizeBytes;     /* Number of bytes to read                              */
  uint16_t    sFrameSize;    /* Number of frames in a super-frame                    */
  int16_t     j;             /*                                                      */
  int16_t     port;          /*                                                      */
  uint32_t    timestamp;     /* The Timestamp (frfc) of the 1st sample in the buffer */
  tsipData_t  *restrict appData; /* Application buffer for data                          */
  unsigned short   resync;        /* If true a resync has occurred                        */
  tsipTsArray_t *tsa;  /* Timeslot info                                        */
  tsipShared_t *sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;
  tsipCommand_t  command;
  tsipReg localFrameAlloc;
  uint16_t      localIndex;

  /* The super-frame is always setup to be half the allocated frame size */
  sFrameSize = tsip->rx.frameCount >> 1;

  /* Locate the base address of the current buffer. Extract the CnID and frfc */
  base      =   tsip->rx.dmap1->baseAddressLocal + (tsip->rx.offset * tsip->rx.dmap1->frameAlloc * sFrameSize);
  CnID      =   TSIP_GET_CNID (*(uint32_t *)base);
  timestamp =   TSIP_GET_FRFC (tsip->rx.ExptdCnIDFrfc);
  resync    =   FALSE;

  /* Verify that the free running frame count matches the expected value at the
   * start of the buffer and at the end of the buffer. The values in between
   * are not checked. */
  if (tsip_verify_frfc (&(tsip->rx), base, sFrameSize) == TSIP_FRFC_FAIL)   {
    tsip->err.rxFrfcError += 1;
    tsip_resync_rx (tsip);
    resync = TRUE;
  }

  /* On resync, re-locate the base address of the current buffer. 
   * Extract the CnID and frfc */
  if (resync)  {
    base      =   tsip->rx.dmap1->baseAddressLocal + (tsip->rx.offset * tsip->rx.dmap1->frameAlloc * sFrameSize);
    CnID      =   TSIP_GET_CNID (*(uint32_t *)base);
    timestamp =   TSIP_GET_FRFC (*(uint32_t *)base);
    tsip->err.rxResync += 1;
  }


  /* Check for CnID change. Do not update the maps unless the command stack
   * is empty. The CnID change can be caused by a command that does not
   * update the maps, but only shifts the buffer */
  if (CnID  != (TSIP_GET_CNID(tsip->rx.ExptdCnIDFrfc)))  {
    if (tsip->rxRemap)  {
        tsip_update_rx_maps (sTsip, tsip);
        tsip->rxRemap = FALSE;
    }
    tsip->rx.ExptdCnIDFrfc = TSIP_SET_CNID (tsip->rx.ExptdCnIDFrfc, CnID);
  }

  /* Sort the data from the tsip */
  tsa = tsip->rx.tsArray;
  while (tsa != NULL)  {

    if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_ENABLED)  {
      appData = tsa->base;

      /* Setup the mask for 16 or 8 bit data. */
      sizeBytes = 1;

      /* This little dance between words and uint16_ts is to allow the C code
       * to do computations using word addresses, but reads using uint16_ts */
      localFrameAlloc = tsip->rx.dmap1->frameAlloc;
      localIndex      = tsa->index;
      wbase           = (uint8_t *)(base + tsa->dataOffset);

#pragma MUST_ITERATE(4,8,4)
      for (j = 0; j < sFrameSize; j++)  {
        appData[localIndex++] = tsip_read_bytes (wbase, sizeBytes);
        wbase += localFrameAlloc;
      }   
      tsa->index      = localIndex;

      /* Make the callout. The timestamp corresponds to the first sample in the
       * application buffer */
      if (tsa->index >= tsa->size)  {
        tsa->callout.rx (tsa->context, &tsa->base, &tsa->rinDesc.rinBase,
                         (timestamp + sFrameSize - tsa->size) & 0xffff, 
                         tsa->size);
        tsa->index = 0;
      }

     /* End enabled timeslot */
    }  else if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_PENDING_ENABLE)  {

      tsa->index += sFrameSize;
      if (tsa->index >= tsa->size)
        tsa->index = 0;
    }
      

    tsa = tsa->next;

  } /* End timeslot loop */


  /* Update the base and alloc pointers. This must be done before any possible
   * map changes are considered */
  tsip->rx.dmap1 = tsip->rx.dmap2;


  /* Update the dma buffer offset for the next entry into this function. */
  tsip->rx.offset = (tsip->rx.offset + 1) & 1;

  /* Update the application context pointers for any timeslots pending enable. This
   * maintains sub-staggering. Do this before processing the command stack. */
  tsa = sTsip->sharedRx.tsPendingActivate;
#pragma PROB_ITERATE(0,0)
  while (tsa != NULL)  {

    if (TSIP_GET_TSA_PORT (tsa) == tsip->port)  {

      tsa->index += sFrameSize;
      if (tsa->index >= tsa->size)
        tsa->index = 0;
    }

    tsa = tsa->next;

  }
  


  /* Check for a requested timeslot change. The enable maps must be provided
   * to the peripheral only if the change will occur when the next superframe marks 
   * the end of the dma buffer. This is required since 
   * at a map change the new data always arrives at the 
   * base address of the context. Since this driver is using a double buffered setup, 
   * the map change can occur only if this function has just processed data at the
   * base address of the dma memory buffer. *
   *
   * Because of shared DMA memory between ports, a timeslot change will not be
   * made until the corresponding port change is complete.
   *
   * The changes to the memory allocation are stored in tsip->rx.dmap2.
   * On the next entry to this isr the values from tsip->rx.dmap1 will be used,
   * since that transfer is currently in progress and uses the previous mapping */

  if (sTsip->sharedRx.stackp)  {

    if ( (tsip->token & TSIP_CHANGE_TOKEN_ENABLE)          && 
         (base == (uint32_t) tsip->rx.dmap1->baseAddressLocal)   && 
         ((tsip->token & TSIP_CHANGE_TOKEN_TX) == 0) )         {

      command = sTsip->sharedRx.commandStack[sTsip->sharedRx.stackp-1];
      port = TSIP_READ_PORT(command);

      if (port == tsip->port)  
        command = tsip_process_command (sTsip, command, tsip, &(sTsip->sharedRx));

      else if ( ((sTsip->enableMap & (1 << port))) == 0)  
        command = tsip_process_command_dis_port (command, port, &(sTsip->sharedRx));
    

      if (command == TSIP_COMMAND_NULL)
        sTsip->sharedRx.stackp -= 1;
      else
        sTsip->sharedRx.commandStack[sTsip->sharedRx.stackp-1] = command;
    }
  }  else  {

    tsipCreateCommandSequence (tsipDrvInst, &(sTsip->sharedRx));

  }



  /* Update the expected timestamp for the next entry. CnId is not updated.  */
  timestamp = timestamp + sFrameSize;
  tsip->rx.ExptdCnIDFrfc = TSIP_SET_FRFC(tsip->rx.ExptdCnIDFrfc, timestamp);
        
} /* tsip_process_rx */

/**********************************************************************************
 * FUNCTION PURPOSE: Processes the transmit side of the TSIP
 **********************************************************************************
 * DESCRIPTION: Verifies the free running frame counts, implements requested
 *              map changes, copies data to the application and makes application
 *              callouts.
 **********************************************************************************/
void tsip_process_tx (tsipDrvInst_t *tsipDrvInst, tsip_t *tsip)
{
  volatile uint32_t   base;          /* Base address of current buffer half (local address)  */
  uint32_t   phdr;          /* Pointer to frame header/footer location              */
  uint8_t    *wbase;         /* Pointer using smallest size element                  */
  tsipData_t *restrict routBuffer;    /* Delay line sample buffer                  */
  tsipData_t *restrict appData;       /* Application buffer for data               */
  int16_t    sizeBytes;     /* Number of bytes to write                             */
  uint16_t   sFrameSize;    /* The number of frames in a super-frame                */
  uint32_t   timestamp;     /* The Timestamp (frfc) of the 1st sample in the buffer */
  uint32_t   writeTs;       /* The new timestamp (frfc) to write to all buffers     */
  uint32_t   writeHdr;      /* The new CnID/frfc to be written to all buffers       */
  uint32_t   cnid;          /* The CnID value to use to write all buffers           */
  int16_t    i, j;          /*                                                      */
  int16_t    port;          /*                                                      */
  unsigned short   resync;        /* If true a resync has occurred                        */
  tsipTsArray_t *tsa;  /* Timeslot info                                        */
  tsipTsArray_t *rxTsa;/* Associated RX timeslot                               */
  tsipShared_t  *sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;
  tsipCommand_t  command;
  tsipReg localFrameAlloc;
  tsipReg localFrameSize;
  uint16_t      localIndex;
  uint16_t      routLocalIndex;
  tsipData_t throwawayBuf[8];

  /* The super-frame is always setup to be half the allocated frame size */
  sFrameSize = tsip->tx.frameCount >> 1;

  /* Locate the base address of the current buffer. Extract the frfc */
  base      =   tsip->tx.dmap1->baseAddressLocal + (tsip->tx.offset * tsip->tx.dmap1->frameAlloc * sFrameSize);
  resync = FALSE;

  /* Verify that the free running frame count in the first and last buffer locations
   * match what is expected. In the transmit case what we're looking for is actually
   * the frfc left by the peripheral on the previous pass through the buffer - it is
   * actually one context frame count in the past. */
  if (tsip_verify_frfc (&(tsip->tx), base, sFrameSize) == TSIP_FRFC_FAIL)   {
    tsip->err.txFrfcError += 1;
    tsip_resync_tx (tsip);
    resync = TRUE;
  }

  /* If a resync occurred recompute the base address and timestamp */
  if (resync)  {
    base      =   tsip->tx.dmap1->baseAddressLocal + (tsip->tx.offset * tsip->tx.dmap1->frameAlloc * sFrameSize);
    timestamp =   TSIP_GET_FRFC (*(uint32_t *)(base + tsip->tx.dmap1->frameSize - sizeof (uint32_t)));
    tsip->err.txResync += 1;

  }  else  {

    timestamp = TSIP_GET_FRFC (tsip->tx.ExptdCnIDFrfc);
  }


  /* Shift the DMA states. This sets the maps to use on the next entry.
   * In the tx direction the map to verify the frfc can be different then
   * the map used to write new data into the buffer. */
  tsip->tx.dmap1 = tsip->tx.dmap2;

  /* Handle any pending map changes. All changes will be placed into dma pointer 2. */
  if (sTsip->sharedTx.stackp)  {

    if ( (tsip->token & TSIP_CHANGE_TOKEN_ENABLE)         && 
         (base == (uint32_t) tsip->tx.dmap1->baseAddressLocal)  && 
         ((tsip->token & TSIP_CHANGE_TOKEN_TX) == TSIP_CHANGE_TOKEN_TX)  ) {

      command = sTsip->sharedTx.commandStack[sTsip->sharedTx.stackp-1];
      port = TSIP_READ_PORT(command);

      if (port == tsip->port)  
        command = tsip_process_command (sTsip, command, tsip, &(sTsip->sharedTx));

      else if ( ((sTsip->enableMap & (1 << port))) == 0)  
        command = tsip_process_command_dis_port (command, port, &(sTsip->sharedTx));
    

      if (command == TSIP_COMMAND_NULL)
        sTsip->sharedTx.stackp -= 1;
      else
        sTsip->sharedTx.commandStack[sTsip->sharedTx.stackp-1] = command;
    }

  }  else  {  /* sTsip->sharedTx.stackp == 0 */

    tsipCreateCommandSequence (tsipDrvInst, &(sTsip->sharedTx));
  }



  /* Recompute the base since it may have been changed doing map updates. */
  base      = tsip->tx.dmap2->baseAddressLocal + (tsip->tx.offset * tsip->tx.dmap2->frameAlloc * sFrameSize);


  /* Write the CnID frfc to each location in the dma buffer. This is the 
   * timestamp we will be expecting to see written by the peripheral
   * two entries later */
  writeTs  = timestamp + (2 * sFrameSize);
  writeHdr = 0;
  cnid     = TSIP_GET_CNID(tsip->tx.ExptdCnIDFrfc);
  writeHdr = TSIP_SET_CNID(writeHdr, cnid);
  phdr     = base;

  localFrameAlloc = tsip->tx.dmap2->frameAlloc;
  localFrameSize  = tsip->tx.dmap2->frameSize;
#pragma MUST_ITERATE(4,8,4)
  for (i = 0; i < sFrameSize; i++, writeTs++)  {
    
    writeHdr = TSIP_SET_FRFC(writeHdr, writeTs);
    *(uint32_t *)(phdr + (i * localFrameAlloc)) = writeHdr;
    *(uint32_t *)(phdr + (i * localFrameAlloc) + localFrameSize - sizeof (uint32_t)) = writeHdr;

  }
    

  /* Now write each timeslot data into the buffer */
  tsa = tsip->tx.tsArray;
  while (tsa != NULL)  {

    if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_ENABLED)  {

      /* Setup the mask for 16 or 8 bit data. */
      sizeBytes = 1;

      /* Get data from the application if required. The timestamp corresponds
       * to the first sample to be provided to the TSIP */
      if ((tsa->index >= tsa->size))  {
        tsa->callout.tx (tsa->context, &tsa->base, NULL,
                         (timestamp + (2 * sFrameSize)) & 0xffff, 
                         tsa->size);
        tsa->index = 0;
      }

      appData          = tsa->base;

      /* Do a read/modify/write to the DMA buffer.  */
      localFrameAlloc  = tsip->tx.dmap2->frameAlloc;
      localIndex       = tsa->index;
      wbase            = (uint8_t *)(base + tsa->dataOffset);

      /* Use of throwawayBuf prevents compiler from creating redundant loops */
      routBuffer       = throwawayBuf;
      routLocalIndex   = 0;
      rxTsa            = tsa->rinDesc.totdmToFromtdmLink;

      if (rxTsa->rinDesc.rinBase) {
        routBuffer     = rxTsa->rinDesc.rinBase;
        routLocalIndex = rxTsa->index;
      }
#pragma MUST_ITERATE(4,8,4)
      for (j = 0; j < sFrameSize; j++)  {
        tsipData_t oneAppData = appData[localIndex++];
        /* If possible, read the value to put into the delay line
         * from what just went out.  When dynamic timeslot buffer
         * allocation is performed, the old data is lost when
         * timeslots are added/deleted.
         */
#ifdef TSIP_STATIC_DMA_BUF
        /* read back what just went out (no y2x_delay) */
        routBuffer[routLocalIndex++] = tsip_read_bytes (wbase, sizeBytes);
#else
        /* put what will go out (which costs 2*sFrameSize y2x_delay) */
        routBuffer[routLocalIndex++] = oneAppData;
#endif
        /* Write new data into DMA buffer */
        tsip_write_bytes (wbase, sizeBytes, oneAppData);
        wbase += localFrameAlloc;
      }
      tsa->index       = localIndex;
      /* Do NOT write back routLocalIndex.  This will be updated by RX */

     /* end ts enabled */

    }  else if (TSIP_GET_TSA_STATE(tsa) == TSIP_TS_STATE_PENDING_ENABLE)  {

      if (tsa->index >= tsa->size)
        tsa->index = 0;
      tsa->index += sFrameSize;
      
    }

    tsa = tsa->next;

  } /* end ts loop */


  /* Update the application context pointers for any timeslots pending enable. This
   * maintains sub-staggering. Do this after processing the command stack. */
  tsa = sTsip->sharedTx.tsPendingActivate;
#pragma PROB_ITERATE(0,0)
  while (tsa != NULL)  {
    if (TSIP_GET_TSA_PORT (tsa) == tsip->port)  {
      if (tsa->index >= tsa->size)
        tsa->index = 0;
      tsa->index += sFrameSize;      
    }

    tsa = tsa->next;
  }
  

  /* Update the expected timestamp for the next entry. CnId is not updated.  */
  timestamp = timestamp + sFrameSize;
  tsip->tx.ExptdCnIDFrfc = TSIP_SET_FRFC(tsip->tx.ExptdCnIDFrfc, timestamp);

  /* Update the dma buffer offset for the next entry into this function.
   * The next entry to the function will use dmap1. */
  tsip->tx.offset = (tsip->tx.offset + 1) & 1;

} /* tsip_process_tx */
        


/**********************************************************************************
 * FUNCTION PURPOSE: Super-frame interrupt handler
 **********************************************************************************
 * DESCRIPTION: Moves data from the application to the TSIP DMA buffers. Handles
 *              timeslot enable/disable requests.
 **********************************************************************************/
void tsipIsr (void *vtsip)
{
  tsip_t  *tsip = (tsip_t *)vtsip;
  tsipPortInst_t *tsipPortInst = (tsipPortInst_t *)vtsip;
  tsipDrvInst_t *tsipDrvInst = tsipPortInst->tsipDrvInst;
  unsigned int mda, mdo;

#ifdef TSIP_CORE_STAGGER
  tsipShared_t *sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;

  if (tsip_verify_stagger_phase(sTsip, tsip)) {    /* DMA disabled on phase error detection*/
    return;
  }
#endif

  mda = cslTsipReadTxFrfc(tsip->port);  /* The port FRFC */
  mdo = mda - (tsip->tx.ExptdCnIDFrfc & 0xffffff) - tsip->subFrame;


  /* Allow for a change in maps only if this interrupt occurs within a reasonable 
   * amount of time, here in the first 25% of the frame. This value is valid
   * even for core sub-staggering */
  if (mdo <= (tsip->tx.frameCount >> 3))  {
    tsip->token |= TSIP_CHANGE_TOKEN_ENABLE;
  } else  {
    tsip->token &= ~TSIP_CHANGE_TOKEN_ENABLE;
    tsip->err.changeMiss += 1;
  }

  /* Swap the token. Only one direction (tx or rx) is allowed to make a map
   * change during an interrupt */
  tsip->token = (tsip->token + 1) & TSIP_CHANGE_TOKEN_ARITH_MASK;


  /* Sub-phase staggering */
  tsip->subPhase += 1;
  if (tsip->subPhase >= tsip->maxSubPhase)
    tsip->subPhase = 0;

  /* Process data to be sent to the TSIP */
  tsip_process_tx (tsipDrvInst, tsip);

  /* Process data received from the TSIP */
  tsip_process_rx (tsipDrvInst, tsip);


  /* Make the sub-frame callout */
  if (tsip->subFrameCallout != NULL)
  {
    (*tsip->subFrameCallout) (tsip->cxt, tsip->rx.ExptdCnIDFrfc & 0xffff);
  }
} /* tsipIsr */


/* ------------------------------------------------------------------------------------------------ 
 *    The following functions are only required if core staggering is enabled in gghwcfg.h 
 */
#ifdef TSIP_CORE_STAGGER


/***************************************************************************************************
 * FUNCTION PURPOSE: TSIP ISR
 ***************************************************************************************************
 * DESCRIPTION: Handles the staggered ISR. 
 *
 *      This function must decide which of three actions to take:
 *
 *          1) Do nothing
 *          2) Call the full ISR processing
 *          3) Activate pending changes
 *
 *
 *      Pending changes are activated if mapChangeState value is set. After a map change
 *      the function exits.
 *
 *      The decision to call the full ISR processing is more difficult. It is not possible
 *      to know if an interrupt has been missed (although the C64X+ interrupt controller
 *      does track one miss). The solution is to look at where the tsip context indicates
 *      the next buffer half to process is. This code then checks the first and last FRFC
 *      value in this buffer. If they are equal the full processing is called.
 *
 *      If the software is not synced to the hardware here then this will result in data loss.
 *      
 *          
 ***************************************************************************************************/
void tsipMidIsr (void *vtsip)
{
  tsip_t *tsip = (tsip_t *)vtsip;


  uint32_t  frfc1;
  uint32_t  frfc2;
  uint32_t  delta;
  uint32_t  sFrameSize;

  /* Tx pending map changes */
  if (tsip->stagger.mapChangeState & TSIP_MAP_STATE_CHANGE_TX_CHANGE)  {
    cslTsipSendNewTxContext (tsip->port, tsip->stagger.txChEn, tsip->tx.channel);
    tsip->stagger.mapChangeState &= ~TSIP_MAP_STATE_CHANGE_TX_CHANGE;
  }

  if (tsip->stagger.mapChangeState & TSIP_MAP_STATE_CHANGE_RX_CHANGE)  {
    cslTsipSendNewRxContext (tsip->port, tsip->stagger.rxChEn, tsip->rx.channel);
    tsip->stagger.mapChangeState &= ~TSIP_MAP_STATE_CHANGE_RX_CHANGE;
  }

  sFrameSize = tsip->rx.frameCount >> 1;
  frfc1 = cslTsipReadRxFrfc(tsip->port);                             /* The actual FRFC */
  frfc2 = TSIP_GET_FRFC(tsip->rx.ExptdCnIDFrfc);                 /* The port predicted FRFC */
  

  /* In normal operation the port frfc (frfc1) will always be
   * greater then the expected frfc (frfc2). The exception
   * is when the frfc wraps. In this case the else case
   * does the correct operation. But if the wrap
   * occurs and there is no sync halTsipIsr can be entered in
   * error. This is not a critical problem since the system
   * is already out of sync. */

  if (frfc1 >= frfc2)
    delta = frfc1 - frfc2;
  else
    delta = (1<<24) - (frfc2 - frfc1);
 
  if (delta >= sFrameSize)
    tsipIsr (vtsip);

} /* tsipMidIsr */


/***************************************************************************************************
 * FUNCTION PURPOSE: Synchronization tsip super-frame interrupt
 ***************************************************************************************************
 * DESCRIPTION: When the TSIP port is first started the phase of the receive frame is unkown.
 *              This function reads the phase and performs a tsip restart to get the desired
 *              phase. Once the phase is verified the interrupt callout is re-assigned to
 *              the staggered sub frame interrupt.
 *
 *              The staggering is done on an even-odd core basis. The only supported
 *              size for staggering is 4 samples per frame, 16 samples per frame in the
 *              dma buffer. 
 *
 *                             Rx DMA buffers
 *                Even cores                Odd cores
 *                 FRFC                        FRFC
 *             /--------------\            /--------------\
 *             |  0x.......0  |            |  0x.......4  |
 *             |  0x.......1  |            |  0x.......5  |
 *             |  0x.......2  |            |  0x.......6  |
 *             |  0x.......3  |            |  0x.......7  |
 *             +--------------+            +--------------+
 *             |  0x.......4  |            |  0x.......8  |
 *             |  0x.......5  |            |  0x.......9  |
 *             |  0x.......6  |            |  0x.......A  |
 *             |  0x.......7  |            |  0x.......B  |
 *             +--------------+            +--------------+
 *             |  0x.......8  |            |  0x.......C  |
 *             |  0x.......9  |            |  0x.......D  |
 *             |  0x.......A  |            |  0x.......E  |
 *             |  0x.......B  |            |  0x.......F  |
 *             +--------------+            +--------------+
 *             |  0x.......C  |            |  0x.......0  |
 *             |  0x.......D  |            |  0x.......1  |
 *             |  0x.......E  |            |  0x.......2  |
 *             |  0x.......F  |            |  0x.......3  |
 *             \--------------/            \--------------/
 *
 *              The Tx dma buffers are similiar, but they are offset by 1 by the port
 *
 *                             Tx DMA buffers
 *                Even cores                Odd cores
 *                 FRFC                        FRFC
 *             /--------------\            /--------------\
 *             |  0x.......1  |            |  0x.......5  |
 *             |  0x.......2  |            |  0x.......6  |
 *             |  0x.......3  |            |  0x.......7  |
 *             |  0x.......4  |            |  0x.......8  |
 *             +--------------+            +--------------+
 *             |  0x.......5  |            |  0x.......9  |
 *             |  0x.......5  |            |  0x.......A  |
 *             |  0x.......6  |            |  0x.......B  |
 *             |  0x.......8  |            |  0x.......C  |
 *             +--------------+            +--------------+
 *             |  0x.......9  |            |  0x.......D  |
 *             |  0x.......A  |            |  0x.......E  |
 *             |  0x.......B  |            |  0x.......F  |
 *             |  0x.......C  |            |  0x.......0  |
 *             +--------------+            +--------------+
 *             |  0x.......D  |            |  0x.......1  |
 *             |  0x.......E  |            |  0x.......2  |
 *             |  0x.......F  |            |  0x.......3  |
 *             |  0x.......0  |            |  0x.......4  |
 *             \--------------/            \--------------/
 *
 *
 *
 *******************************************************************************************************/
static void tsip_restart_tx_dma (int16_t port, int16_t channel)
{
  volatile int x;
  cslTsipStopTxDma  (port, channel);
  for (x = 0; x < TSIP_FOR_LOOP_STAGGER_DELAY; x++);
  cslTsipStartTxDma (port, channel);
}
static void tsip_restart_rx_dma (int16_t port, int16_t channel)
{
  volatile int x;
  cslTsipStopRxDma  (port, channel);
  for (x = 0; x < TSIP_FOR_LOOP_STAGGER_DELAY; x++);
  cslTsipStartRxDma (port, channel);
}
void tsipStaggerStartIsr (void *vtsip)
{
  tsip_t *tsip = (tsip_t *)vtsip;
  uint32_t frfc;
  int16_t   core;

  core = deviceWhoAmI();

  if (tsip->stagger.syncState == TSIP_STAGGER_STATE_FIND_RX_SYNC)  {

    /* Read the current frfc. If it indicates that the value is the value
     * desired in the last quarter of the buffer restart the dma. The new
     * start should take effect at the end of the current sub-frame and
     * place new samples at the start of the buffer. */

    frfc = cslTsipReadRxFrfc (tsip->port) & 0x0000000f;

    if ((core & 1) == 0)  {   /* Even cores */

      if (frfc >= 0xc)  {
        tsip_restart_rx_dma (tsip->port, tsip->rx.channel);
        tsip->stagger.syncState = TSIP_STAGGER_STATE_VERIFY_SYNC;
      }

    }  else  {   /* Odd cores */

      if (frfc < 4)   {
        tsip_restart_rx_dma (tsip->port, tsip->rx.channel);
        tsip->stagger.syncState = TSIP_STAGGER_STATE_VERIFY_SYNC;
      }
    }


  }  else if (tsip->stagger.syncState == TSIP_STAGGER_STATE_FIND_TX_SYNC)  {

    /* Repeat for the tx direction. Subtract one from the value read from the
     * port to make the if conditions a single check instead of a double */

    frfc = (cslTsipReadTxFrfc (tsip->port) - 1) & 0x0000000f;


    if ((core & 1) == 0)  {   /* Even cores */

      if (frfc >= 0xc)  {
        tsip_restart_tx_dma (tsip->port, tsip->tx.channel);
        tsip->stagger.syncState = TSIP_STAGGER_STATE_VERIFY_SYNC;
      }

    }  else  {   /* Odd cores */

      if (frfc < 4)   {
        tsip_restart_tx_dma (tsip->port, tsip->tx.channel);
        tsip->stagger.syncState = TSIP_STAGGER_STATE_VERIFY_SYNC;
      }
    }



  }  else if (tsip->stagger.syncState == TSIP_STAGGER_STATE_VERIFY_SYNC)  {

    /* Read the rx frfc from the base address of the buffer. If it matches the
     * desired value the sync is established. Otherwise the sync is incorrect */
    frfc = *((uint32_t *)(tsip->rx.dmap1->baseAddressLocal));
    frfc = frfc & 0x0000000f;


    if ((core & 1) == 0)  {  /* Even cores */

      if (frfc != 0)  {
        tsip->stagger.syncState = TSIP_STAGGER_STATE_FIND_RX_SYNC;
        return;
      }

    }  else {  /* Odd cores  */

      if (frfc != 4)  {
        tsip->stagger.syncState = TSIP_STAGGER_STATE_FIND_RX_SYNC;
        return;
      }

    }



    /* Read the tx frfc from the base address of the buffer and compare. The tx frfc
     * is written by the port only at the end of the frame. */
    frfc = *((uint32_t *)(tsip->tx.dmap1->baseAddressLocal + sizeof(uint32_t)));
    frfc = frfc & 0x0000000f;

    if ((core & 1) == 0)  {  /* Even cores */

      if (frfc != 1)   {
        tsip->stagger.syncState = TSIP_STAGGER_STATE_FIND_TX_SYNC;
        return;
      }

    }  else  {  /* Odd cores  */

      if (frfc != 5)  {
        tsip->stagger.syncState = TSIP_STAGGER_STATE_FIND_TX_SYNC;
        return;
      }

    }

    /* Both tx and rx have passed sync check. Redirect this interrupt to the run time processing */
    tsip->stagger.isrChoice = TSIP_STAGGER_MID_ISR;

  }  else  {

    /* The state value is garbage. Change to look for the sync state */
    tsip->stagger.syncState =  TSIP_STAGGER_STATE_FIND_RX_SYNC;

  }

} /* tsipStaggerStartIsr  */
    

#endif  /* TSIP_CORE_STAGGER */


/*********************************************************************************
 * FUNCTION PURPOSE: super-frame ISR function
 *********************************************************************************
 * DESCRIPTION: Wrapper of super-frame ISR functions. For the case of cross-core
 *              staggering, trigger tsipStaggerStartIsr and tsipMidIsr
 *              depending on stagger status
 *********************************************************************************/
void Tsip_superFrameIsr (Tsip_PortHandle portHandle)
{

#ifdef TSIP_CORE_STAGGER
  tsip_t *tsip = (tsip_t *)portHandle;
  if (tsip->stagger.isrChoice == TSIP_STAGGER_MID_ISR) {
    tsipMidIsr(portHandle);
  } else {
    tsipStaggerStartIsr(portHandle);
  }
#else
  tsipIsr(portHandle);
#endif /* TSIP_CORE_STAGGER */

}

