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
 *  File Name: tsipinit.c
 *
 *  Source for TSIP driver: TSIP init functions
 *  This code is executed only during driver initialization.
 *
 **/

#include "tsip.h"
#include "tsiploc.h"
#include "tsipcsl.h"
#include "tsip_osal.h"
#include <string.h>

/* Array of TSIP memory buffer alignment requirements common to all the TSIP ports
 *                TSIP_BUFFNUM_INSTANCE_DRIVER - the driver instance buffer
 *                TSIP_BUFFNUM_HEAP_SHARED - heap for the TSIP shared memory
 *                TSIP_BUFFNUM_HEAP_TIMESLOT - heap for TSIP timeslots
 *                TSIP_BUFFNUM_TX_DMA_BUFFER - DMA buffer for transmit
 *                TSIP_BUFFNUM_RX_DMA_BUFFER - DMA buffer for receive
 */
const int tsipMemAlignsShared[tsip_N_BUFS_SHARED] = { 2, 2, 8, 2, 2};
/* Array of TSIP memory buffer alignment requirements specific to individual TSIP ports
 *                TSIP_BUFFNUM_INSTANCE_PORT - the port instance buffer
 */
const int tsipMemAlignsPort[tsip_N_BUFS_PORT] = { 8 };


/*********************************************************************************
 * FUNCTION PURPOSE: Return TSIP driver instance size
 *********************************************************************************
 * DESCRIPTION: Returns memory requirements for the TSIP driver instance
 *********************************************************************************/
uint16_t tsipGetDriverInstanceSize ()
{
  uint16_t size;
  size = sizeof(tsipDrvInst_t);
  TSIP_ALIGN_STRUCT(size);

  return (size);

} /* tsipGetDriverInstanceSize */

/*********************************************************************************
 * FUNCTION PURPOSE: Return heap requirements for the TSIP shared memory
 *********************************************************************************
 * DESCRIPTION: Returns requirements for all but the timeslot arrays. The
 *              timeslot arrays are allocated when the first port is 
 *              initialized.
 *********************************************************************************/
uint16_t tsipGetSharedHeapSize (uint16_t numPorts)
{
  uint16_t size;

  /* The shared structure */
  size = sizeof (tsipShared_t);
  TSIP_ALIGN_STRUCT(size);

  /* The array of dma buffers (offsets, lengths). The number of elements 
   * in the array is equal to the max number of ports */
  size += (numPorts * sizeof (tsipDmaBuffer_t));  /* dmaBuffer, Tx */
  TSIP_ALIGN_STRUCT(size);
  size += (numPorts * sizeof (tsipDmaBuffer_t));  /* dmaBuffer, Rx */
  TSIP_ALIGN_STRUCT(size);


  /* The command stack */
  size += ((numPorts+8) * sizeof (tsipCommand_t));  /* Tx */
  TSIP_ALIGN_STRUCT(size);
  size += ((numPorts+8) * sizeof (tsipCommand_t));  /* Rx */
  TSIP_ALIGN_STRUCT(size);

  return (size);

} /* tsipGetSharedHeapSize */


/************************************************************************************
 * FUNCTION PURPOSE: Determine the memory usage of a frame
 ************************************************************************************
 * DESCRIPTION: Computes the DMA requirements for the maximum number of possible
 *              ports. This function computes the memory usage for a single frame
 *              on all possible ports. 
 *
 *        !!!!!! ALL SIZING CODE ASSUMES THAT EVERY PORT USES THE SAME
 *               SUB-FRAME SIZE. IF THAT IS NOT TRUE THIS CODE WILL BREAK  !!!!!!
 *
 ************************************************************************************/
uint16_t tsip_get_dma_mem_usage (tsipSizeInfo_t *sizeCfg)
{
  uint16_t size;
  uint16_t nChannels;
  uint16_t  numPorts;
  
  if (sizeCfg->validParams & TSIP_SIZE_INFO_VALIDPARAMS_NUM_PORTS)
    numPorts = sizeCfg->numPorts;
  else
    numPorts = 2; /* Default number of ports : 2 */

  nChannels = sizeCfg->maxChannels;

  size  = (numPorts) * (TSIP_FRAME_HEADER_SIZE + TSIP_FRAME_FOOTER_SIZE);
  size += TSIP_FRAME_ROUND_SIZE((nChannels * TSIP_ELEMENT_BUF_SIZE));
  size += ((numPorts - 1) * TSIP_FRAME_ROUND_SIZE(TSIP_ELEMENT_BUF_SIZE));
  size  = TSIP_FRAME_ROUND_SIZE(size);

  return (size);

} /* tsip_get_dma_mem_usage */

/************************************************************************************
 * FUNCTION PURPOSE: Calculate DMA buffer requirements
 ************************************************************************************
 * DESCRIPTION: Returns the size requirements for TSIP ports. A common buffer is 
 *              divided by the maximum possible number of ports. The memory size
 *              returned is what is required if the maximum number of ports are
 *              operational with the worst possible alignment.
 ************************************************************************************/
uint16_t tsipGetDmaBufInfo (tsipSizeInfo_t *sizeCfg)
{
    uint16_t size;

    /* Determine the size for each frame, padded out to an alignment length */
    size = tsip_get_dma_mem_usage (sizeCfg);

    /* There are two sub-frames to be buffered */
    size = size * sizeCfg->subFrameSize * 2 * TSIP_ELEMENT_BUF_SIZE;

    return (size);

} /* tsipGetDmaBufInfo */


/*********************************************************************************
 * FUNCTION PURPOSE: Initialize the TSIP shared memory context
 *********************************************************************************
 * DESCRIPTION: Called from Tsip_createShared. The TSIP shared memory context
 *              is initialized, and all allocation is done later in 
 *              Tsip_createShared
 *********************************************************************************/
int16_t tsipSharedInit (void *vcxt, uint16_t numPorts)
{
  tsipShared_t *sTsip = (tsipShared_t *)vcxt;
  uint32_t base  = (uint32_t)vcxt;

  /* A value of 0 indicates that no ports have been configured */
  sTsip->allocated  = 0;
  sTsip->enableMap  = 0;

  /* Allocate the dma buffer pointers */
  base += sizeof (tsipShared_t);
  TSIP_ALIGN_STRUCT(base);  

  sTsip->sharedTx.dmaBuffer = (tsipDmaBuffer_t *)base;
  base += (numPorts * sizeof (tsipDmaBuffer_t));
  TSIP_ALIGN_STRUCT(base);  

  sTsip->sharedRx.dmaBuffer = (tsipDmaBuffer_t *)base;
  base += (numPorts * sizeof (tsipDmaBuffer_t));
  TSIP_ALIGN_STRUCT(base);  

  /* The command stack */
  sTsip->sharedTx.commandStack = (tsipCommand_t *)base;
  base += ((numPorts+8) * sizeof (tsipCommand_t));
  TSIP_ALIGN_STRUCT(base);  

  sTsip->sharedRx.commandStack = (tsipCommand_t *)base;
  base += ((numPorts+8) * sizeof (tsipCommand_t));
  TSIP_ALIGN_STRUCT(base);  

  sTsip->sharedTx.stackp = 0;
  sTsip->sharedRx.stackp = 0;

  return (tsip_OK);

} /* tsipSharedInit */

/*********************************************************************************
 * FUNCTION PURPOSE: Perform initial buffer allocation 
 *********************************************************************************
 * DESCRIPTION: Allocates the initial buffer among all possible ports. Port 0
 *              will always be at a lower address then port 1, which will always
 *              be lower then port 2, etc.
 *
 *              The memory buffer is divided by the number of ports. Each
 *              port is assigned the minimal size and the base address 
 *              for each port so that the port will use the upper address
 *              range of the memory.
 *********************************************************************************/
int16_t tsip_shared_buf_alloc (tsipSharedDir_t *sdir, uint16_t *dmaBase, uint16_t dmaSize, 
                            int16_t minSizeWords, uint16_t numPorts)
{
  int16_t  i;
  uint16_t buffOffset;
  uint16_t size;
  int16_t frameSizes[] = { TSIP_SUB_FRAME_RATES };
  int16_t frameMax;
  int16_t totalAllocSize;
  
  for (i = 1, frameMax = frameSizes[0]; i < numPorts; i++)  {
    if (frameSizes[i] > frameMax) {
      frameMax = frameSizes[i];
    }  
  }    
  totalAllocSize = minSizeWords * frameMax * 2;

  /* Adjust the base address and size if aligment requirements are not met */
  sdir->memBase      = TSIP_FRAME_ROUND_SIZE((uint32_t)dmaBase);
  sdir->lengthBytes  = dmaSize - TSIP_FRAME_SIZE_REMAIN((uint32_t)dmaBase);

  /* No check is made here that the rounding of the size will lead to 
   * an overflow of the buffer. That should be impossible based on 
   * the getSizes function. */
  size = TSIP_FRAME_ROUND_SIZE (sdir->lengthBytes / numPorts);
  buffOffset = sdir->lengthBytes;

  /* Divide up the memory */
  for (i = numPorts-1; i >= 0; i--)  {
    sdir->dmaBuffer[i].baseOffset  = buffOffset - totalAllocSize;
    sdir->dmaBuffer[i].frameAlloc  = minSizeWords;
    buffOffset -= size;
  }

  return (0);

} /* tsip_shared_buf_alloc */


/**************************************************************************************
 * FUNCTION PURPOSE: Provide memory buffer allocation requirements common to TSIP ports
 **************************************************************************************
 * DESCRIPTION: Memory usage by the TSIP module which is common to all the TSIP 
 *              ports is returned
 **************************************************************************************/
tsipReturn_t Tsip_getBufferReqShared (tsipSizeInfo_t *sizeCfg, int sizes[], int aligns[])
{
  int i;  

  if(sizeCfg == NULL)
      return (tsip_ERR_CONFIG);

  sizes[TSIP_BUFFNUM_INSTANCE_DRIVER]= tsipGetDriverInstanceSize();
  if ( sizeCfg->validParams & TSIP_SIZE_INFO_VALIDPARAMS_NUM_PORTS) 
    sizes[TSIP_BUFFNUM_HEAP_SHARED] = tsipGetSharedHeapSize(sizeCfg->numPorts);
  else
    sizes[TSIP_BUFFNUM_HEAP_SHARED] = tsipGetSharedHeapSize(2); /* Assume default num ports to 2 */
	
  sizes[TSIP_BUFFNUM_HEAP_TIMESLOT] = sizeCfg->maxChannels * 2 * sizeof (tsipTsArray_t);
  sizes[TSIP_BUFFNUM_TX_DMA_BUFFER]= tsipGetDmaBufInfo(sizeCfg);
  sizes[TSIP_BUFFNUM_RX_DMA_BUFFER]= tsipGetDmaBufInfo(sizeCfg);

  for ( i = 0; i < tsip_N_BUFS_SHARED; i++)
  {
      aligns[i] = tsipMemAlignsShared[i];
  }

  return (tsip_OK);

} /* Tsip_getBufferReqShared */


/**************************************************************************************
 * FUNCTION PURPOSE: Provide memory buffer allocation requirements sepcific to
 *                   individual TSIP ports
 **************************************************************************************
 * DESCRIPTION: Memory usage by the TSIP module sepcific to individual TSIP ports
 *              is returned
 **************************************************************************************/
tsipReturn_t Tsip_getBufferReqPort (tsipSizeInfo_t *sizeCfg, int sizes[], int aligns[])
{
  int i;  

  sizes[TSIP_BUFFNUM_INSTANCE_PORT] = sizeof(tsipPortInst_t);

  for ( i = 0; i < tsip_N_BUFS_PORT; i++)
  {
      aligns[i] = tsipMemAlignsPort[i];
  }

  return (tsip_OK);

} /* Tsip_getBufferReqPort */


/***********************************************************************************
 * FUNCTION PURPOSE: Create a TSIP driver instance and perform initialization 
 *                   common to all the TSIP ports
 ***********************************************************************************
 * DESCRIPTION: An instance of the TSIP driver is created and initialization common
 *              to all the TSIP ports is performed
 ***********************************************************************************/
tsipReturn_t Tsip_createShared (tsipConfig_t *config, void* bases[], Tsip_DrvHandle *pHandle)
{
  tsipDrvInst_t *tsipDrvInst = (tsipDrvInst_t *)bases[0];
  tsipSizeInfo_t *sizeCfg;
  tsipShared_t *sTsip;
  int16_t    i;

  if ((tsipDrvInst == NULL) || (config == NULL) || (pHandle == NULL)|| (config->sizeCfg == NULL))
    return (tsip_ERR_CONFIG);

  *pHandle = (Tsip_DrvHandle *)tsipDrvInst;
  sizeCfg = config->sizeCfg; 

  /* If core staggering is implemented only a subFrameSize of 8 is currently allowed */
#ifdef TSIP_CORE_STAGGER
  if (sizeCfg->subFrameSize != 8)  {
    return (tsip_ERR_CONFIG);
  }
#endif

  /* Verify buffer base addresses */
  for (i = 0; i < tsip_N_BUFS_SHARED; i++)  {

    if (bases[i] == NULL)
      return (tsip_ERR_CONFIG);

    tsipDrvInst->tsipBufsShared[i].base = bases[i];
  }

  /* Initialize TSIP driver instance */
  if(sizeCfg->validParams & TSIP_SIZE_INFO_VALIDPARAMS_NUM_PORTS)
  	tsipDrvInst->nTsipPorts = sizeCfg->numPorts;
  else
  	tsipDrvInst->nTsipPorts = 2;   /* Default for C6678 */

  for (i = 0; i < tsipDrvInst->nTsipPorts; i++)
    tsipDrvInst->tsipPort[i] = NULL;

  tsipDrvInst->tsipShared = (Tsip_HandleShared)tsipDrvInst->tsipBufsShared[TSIP_BUFFNUM_HEAP_SHARED].base;

  /* Initialize TSIP shared memory context */
  tsipSharedInit(tsipDrvInst->tsipShared, tsipDrvInst->nTsipPorts);
  sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;

  /* Initialize TSIP timeslot array */
  sTsip->tsArrayFree = (tsipTsArray_t *)tsipDrvInst->tsipBufsShared[TSIP_BUFFNUM_HEAP_TIMESLOT].base;

  /* Form the singly linked list of free timeslot structures */
  for (i = 0; i < 2*(sizeCfg->maxChannels); i++)  {
    TSIP_SET_TSA_STATE(&(sTsip->tsArrayFree[i]), TSIP_TS_STATE_DISABLED);
    sTsip->tsArrayFree[i].next = (tsipTsArray_t *)&(sTsip->tsArrayFree[i+1]);
  }
  sTsip->tsArrayFree[(2 * sizeCfg->maxChannels)-1].next = NULL;

  /* Allocate the DMA buffer memory */
  tsip_shared_buf_alloc (&sTsip->sharedTx, 
                         tsipDrvInst->tsipBufsShared[TSIP_BUFFNUM_TX_DMA_BUFFER].base, 
                         tsipGetDmaBufInfo(sizeCfg), 
                         TSIP_FRAME_MIN_SIZE, tsipDrvInst->nTsipPorts);
  tsip_shared_buf_alloc (&sTsip->sharedRx, 
                         tsipDrvInst->tsipBufsShared[TSIP_BUFFNUM_RX_DMA_BUFFER].base, 
                         tsipGetDmaBufInfo(sizeCfg), 
                         TSIP_FRAME_MIN_SIZE, tsipDrvInst->nTsipPorts);

  memset (tsipDrvInst->tsipBufsShared[TSIP_BUFFNUM_TX_DMA_BUFFER].base, 0x7F7F, 
          tsipGetDmaBufInfo(sizeCfg));

  sTsip->sharedTx.dirId = TSIP_SHARED_DIR_TX;
  sTsip->sharedRx.dirId = TSIP_SHARED_DIR_RX;

  /* Initialize pending timeslot arrays to NULL */
  sTsip->sharedTx.tsPendingActivate   = NULL;
  sTsip->sharedTx.tsPendingDeactivate = NULL;

  sTsip->sharedRx.tsPendingActivate   = NULL;
  sTsip->sharedRx.tsPendingDeactivate = NULL;

  /* Indicate that common allocation is done */
  sTsip->allocated = 1;

  return (tsip_OK);

} /* Tsip_createShared */

/***********************************************************************************
 * FUNCTION PURPOSE: Create a TSIP port instance and perform port initialization
 ***********************************************************************************
 * DESCRIPTION: An instance of the TSIP port is created and initialized
 ***********************************************************************************/
tsipReturn_t Tsip_createPort (Tsip_DrvHandle handle, tsipConfig_t *config, 
                              void* bases[], Tsip_PortHandle *portHandle)
{
  tsipPortInst_t* tsipPortInst = (tsipPortInst_t*)bases[0];
  tsipDrvInst_t *tsipDrvInst = (tsipDrvInst_t *)handle;
  tsipShared_t *sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;
  tsip_t *tsip;
  tsipSizeInfo_t *sizeCfg;
  tsipReturn_t ret;
  void *tsipPort;
  int i, j;

  if ((tsipDrvInst == NULL) || (portHandle == NULL) ||
      (config == NULL) || (config->sizeCfg == NULL) )
    return (tsip_ERR_CONFIG);

  tsipPortInst->tsipDrvInst = (tsipDrvInst_t *)handle;
  *portHandle = (Tsip_PortHandle *)tsipPortInst;
  sizeCfg = config->sizeCfg;

  /* Check if there is resource for creating the TSIP port */
  for (i = 0; i < tsipDrvInst->nTsipPorts; i++)  {
    tsipPort = tsipDrvInst->tsipPort[i];
    if (tsipPort == NULL)
      break;
  }

  if (i == tsipDrvInst->nTsipPorts)
    return (tsip_NO_PORTS_AVAILABLE);

  /* Verify buffer base addresses */
  for (j = 0; j < tsip_N_BUFS_PORT; j++)  {

    if (bases[j] == NULL)
      return (tsip_ERR_CONFIG);

    tsipPortInst->tsipBufsPort[j].base = bases[j];
  }
  
   /* Memory allocation */
  tsipDrvInst->tsipPort[i] = (Tsip_HandlePort) tsipPortInst->tsipBufsPort[TSIP_BUFFNUM_INSTANCE_PORT].base;
  tsip = (tsip_t *)tsipDrvInst->tsipPort[i];

  /* Initialize the structure */
  tsip->port         = config->physPort;
  tsip->subFrame     = sizeCfg->subFrameSize;
  tsip->subPhase     = 0;
  tsip->maxSubPhase  = config->maxPhase;
  tsip->rxRemap      = FALSE;

  tsip->tx.channel       = config->tx.channel;
  tsip->tx.tsArray       = NULL;
  tsip->tx.offset        = 0;

  /* TX requires prefetch, so data is transmited 2nd Frame Sync */
  tsip->tx.ExptdCnIDFrfc        = 1;  
  tsip->tx.ExptdCnIDFrfc = TSIP_SET_CNID(tsip->tx.ExptdCnIDFrfc, TSIP_INITIAL_TX_CNID);

  tsip->rx.channel       = config->rx.channel;
  tsip->rx.tsArray       = NULL;
  tsip->rx.offset        = 0;

  tsip->rx.ExptdCnIDFrfc        = 0;
  tsip->rx.ExptdCnIDFrfc = TSIP_SET_CNID(tsip->rx.ExptdCnIDFrfc, TSIP_INITIAL_RX_CNID);

  tsip->subFrameCallout = config->subFrameCallout;
  tsip->cxt             = config->cxt;

  /* The port is not yet enabled. But setup the dma context to the
   * current shared mapping */
  tsip->tx.dma1.baseAddressLocal = sTsip->sharedTx.memBase + sTsip->sharedTx.dmaBuffer[tsip->port].baseOffset;
  tsip->tx.dma1.frameAlloc       = sTsip->sharedTx.dmaBuffer[tsip->port].frameAlloc;
  tsip->tx.dma1.frameSize        = tsip->tx.dma1.frameAlloc;
  tsip->tx.frameCount            = 2 * tsip->subFrame;

  tsip->tx.dma2  = tsip->tx.dma1;
  tsip->tx.dmap1 = &(tsip->tx.dma1);
  tsip->tx.dmap2 = tsip->tx.dmap1;

  tsip->rx.dma1.baseAddressLocal = sTsip->sharedRx.memBase + sTsip->sharedRx.dmaBuffer[tsip->port].baseOffset;
  tsip->rx.dma1.frameAlloc       = sTsip->sharedRx.dmaBuffer[tsip->port].frameAlloc;
  tsip->rx.dma1.frameSize        = tsip->rx.dma1.frameAlloc;
  tsip->rx.frameCount            = 2 * tsip->subFrame;

  tsip->rx.dma2 = tsip->rx.dma1;
  tsip->rx.dmap1 = &(tsip->rx.dma1);
  tsip->rx.dmap2 = tsip->rx.dmap1;

  tsip->token = TSIP_CHANGE_TOKEN_TX;

  memset (&(tsip->err), 0, sizeof (tsipErr_t));

#ifdef TSIP_CORE_STAGGER
  tsip->stagger.syncState = TSIP_STAGGER_STATE_VERIFY_SYNC;
  tsip->stagger.mapChangeState = 0;
  tsip->stagger.isrChoice = TSIP_STAGGER_STAGGER_START_ISR;
#endif

  /* Convert the number of links into a data rate. This mapping must be
   * changed for devices with a different link/active map */
  if (config->tx.dataRate == CSL_TSIP_DATARATE_8M)  {
    tsip->tsPerLink    = 128;
  } else if (config->tx.dataRate == CSL_TSIP_DATARATE_16M)  {
    tsip->tsPerLink    = 256;
  } else if (config->tx.dataRate == CSL_TSIP_DATARATE_32M)  {
    tsip->tsPerLink    = 512;
  } else
    return (tsip_ERR_CONFIG);

  /*Only Master core configures the TSIP port registers*/
  if(config->masterCore)
  {
    /* Pass the configuration through the csl */
    ret = cslTsipConfigPort(config);
    if (ret != tsip_OK)
     return(ret);

    /* Enable the port. It doesn't matter if another core already enabled
     * the port. clsTipEnablePort returns non-zero only if the port was
     * already on. */
    cslTsipEnablePort (tsip->port);
  }

  return (tsip_OK);

} /* Tsip_createPort */



/***********************************************************************************
 * FUNCTION PURPOSE: Enables Transmit and Receive channels on a TSIP port
 ***********************************************************************************
 * DESCRIPTION: TSIP transmit and receive channel is enabled for a TSIP port
 ***********************************************************************************/
tsipReturn_t Tsip_enablePortChannel (Tsip_DrvHandle handle, Tsip_PortHandle portHandle,
                                     int16_t txCh, int16_t rxCh)

{
  tsipDrvInst_t *tsipDrvInst = (tsipDrvInst_t *)handle;
  tsipShared_t *sTsip = (tsipShared_t *)tsipDrvInst->tsipShared;
  tsip_t *tsip = (tsip_t *)portHandle;

  /* Clear the DMA channels */
  cslTsipClearChannel (tsip->port, txCh, rxCh);

  /* Flag the shared memory to indicate the port DMA channel is enabled */
  sTsip->enableMap |= (1 << tsip->port);

  /* Enable the DMA */
  tsipStartDma (sTsip, tsip);

  return (tsip_OK);
}



/**********************************************************************************************
 * FUNCTION PURPOSE: Return memory buffer usage for application to free
 **********************************************************************************************
 * DESCRIPTION: Returns the base address of all buffers previously allocated for the TSIP driver
 *              instance and those shared across TSIP ports
 **********************************************************************************************/
tsipReturn_t Tsip_closeShared (Tsip_DrvHandle handle, void* bases[])
{
  tsipDrvInst_t *tsipDrvInst = (tsipDrvInst_t *)handle;
  int i;
  
  for ( i = 0; i < tsip_N_BUFS_SHARED; i++)
  {
    bases[i] = tsipDrvInst->tsipBufsShared[i].base;
  }

  return (tsip_OK);

} /* Tsip_closeShared */

/**********************************************************************************************
 * FUNCTION PURPOSE: Return memory buffer usage for application to free
 **********************************************************************************************
 * DESCRIPTION: Returns the base address of all buffers previously allocated for the TSIP port
 *              instance.
 **********************************************************************************************/
tsipReturn_t Tsip_closePort (Tsip_PortHandle portHandle, void* bases[])
{
  tsipPortInst_t *tsipPortInst = (tsipPortInst_t *)portHandle;
  int i;

  for ( i = 0; i < tsip_N_BUFS_PORT; i++)
  {
    bases[i] = tsipPortInst->tsipBufsPort[i].base;
  }

  return (tsip_OK);

} /* Tsip_closePort */


