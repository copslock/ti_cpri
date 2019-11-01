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
 *  File Name: tsipcsl.c
 *
 *  Source for TSIP driver: CSL interface functions
 *
 **/
#include "tsip_cfg.h"
#include "tsipcsl.h"
#include <ti/csl/csl_tsip.h>
#include <ti/csl/csl_tsipAux.h>
#include <string.h>

/******************************************************************************
 * FUNCTION PURPOSE: Clear the DMA structure for a channel
 ******************************************************************************
 * DESCRIPTION: Clears all the channel bitmaps for the indicated channels
 ******************************************************************************/
int16_t cslTsipClearChannel (int16_t port, int16_t txChan, int16_t rxChan)
{
  CSL_TsipHandle  hTsip;
  uint16_t i;

  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);
  
  /* Clear all the channel bitmaps */
  for (i = 0; i < CSL_TSIP_BITMAP_SIZE; i++)  {
    CSL_FINST ((hTsip->XBM[txChan].XBMA)[i], TSIP_XCH0BMA_XCH0BMA, RESETVAL);
    CSL_FINST ((hTsip->XBM[txChan].XBMB)[i], TSIP_XCH0BMB_XCH0BMB, RESETVAL);
    CSL_FINST ((hTsip->RBM[rxChan].RBMA)[i], TSIP_RCH0BMA_RCH0BMA, RESETVAL);
    CSL_FINST ((hTsip->RBM[rxChan].RBMB)[i], TSIP_RCH0BMB_RCH0BMB, RESETVAL);
  }

  return (CSL_SOK);
} /* cslTsipClearChannel */

/**********************************************************************************
 * FUNCTION PURPOSE: Configure a tsip port
 **********************************************************************************
 * DESCRIPTION: The specified TSIP port is setup, but not enabled. 
 **********************************************************************************/
int16_t cslTsipConfigPort (tsipConfig_t *cfg)
{
  CSL_TsipHandle  hTsip;
  tsipSizeInfo_t *sizeCfg;
  sizeCfg = cfg->sizeCfg;

  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(cfg->physPort);

  /* It's an error if the port is enabled. Check the SIU global control register */
  if (CSL_tsipSIUIsReceiveEnabled(hTsip) || 
      CSL_tsipSIUIsTransmitEnabled(hTsip))
  {
    return (tsip_TX_RX_ALREADY_ENABLED);
  }

  /* Also verify that the tdmu is not enabled */
  if (CSL_tsipTDMUIsEnabled(hTsip)) {
    return (tsip_TDMU_DMATCU_ALREADY_ENABLED);
  }

  /* Global configuration */
  /* Set the test mode */
  CSL_tsipEmutstSetTestMode(hTsip, cfg->testMode);
  /* Select the test mode if it is enabled */
  CSL_tsipEmutstSelectTestMode(hTsip, cfg->testModeSelect);
  /* Set the endianess */
  CSL_tsipTDMUSetEndianess(hTsip, cfg->endian);
  /* Set the SIU Clock Redundancy Mode */
  CSL_tsipSIUSetClockRedundancy(hTsip, cfg->clkRedund);
  /* Set the Max Transfer Priority */
  CSL_tsipDMATCUSetMaxTransferPriority(hTsip, cfg->maxPriority);
  /* Set the Transfer Priority */
  CSL_tsipDMATCUSetTransferPriority (hTsip, cfg->priority);


  /* Frame count and frame size configuration */
#ifdef TSIP_CORE_STAGGER
  CSL_tsipSIUSetTxFrameCount(hTsip, (sizeCfg->subFrameSize >> 1)-1);
  CSL_tsipSIUSetRxFrameCount(hTsip, (sizeCfg->subFrameSize >> 1)-1);
#else
  CSL_tsipSIUSetTxFrameCount(hTsip, sizeCfg->subFrameSize-1);
  CSL_tsipSIUSetRxFrameCount(hTsip, sizeCfg->subFrameSize-1);
#endif
  CSL_tsipSIUSetTxFrameSize(hTsip, cfg->tx.frameSize);
  CSL_tsipSIUSetRxFrameSize(hTsip, cfg->rx.frameSize);

  /* Transmit and receive clock source configuration */
  CSL_tsipSIUSetTxClockSource(hTsip, cfg->tx.clkSrc);
  CSL_tsipSIUSetRxClockSource(hTsip, cfg->rx.clkSrc);


  /* Transmit and receive control configuration */
  /* The data delay must be modified to handle the positive 2 offset. If the
   * desired data delay is 1 or 0, then the delay must be changed to do this
   * using a wrap. The clock rate (single or double) must be taken into
   * account as well */
  if (cfg->tx.dataDelay < 2)  {
    cfg->tx.dataDelay = (cfg->tx.tsPerFrame * 8) + cfg->tx.dataDelay;
  if (cfg->tx.clkMode == CSL_TSIP_CLKM_DBL)
    cfg->tx.dataDelay = cfg->tx.dataDelay * 2;
  }
  cfg->tx.dataDelay    = cfg->tx.dataDelay - 2;

  CSL_tsipSIUSetTxDataDelay(hTsip, cfg->tx.dataDelay);
  CSL_tsipSIUSetTxOutputDelay(hTsip, cfg->tx.bdxDelay);
  CSL_tsipSIUSetTxDisabledState(hTsip, cfg->tx.idleDrive);
  CSL_tsipSIUSetTxFrameSyncPolarity(hTsip, cfg->tx.fsyncPol);
  CSL_tsipSIUSetTxFrameSyncClockPolarity(hTsip, cfg->tx.fsyncClkPol);
  CSL_tsipSIUSetTxDataClockPolarity(hTsip, cfg->tx.clkPol);
  CSL_tsipSIUSetTxDataRate(hTsip, cfg->tx.dataRate);
  CSL_tsipSIUSetTxClockMode(hTsip, cfg->tx.clkMode);


  /* The data delay must be modified as it was for tx to account for
   * data delay 0 and 1, and the double clock rate */
  if (cfg->rx.dataDelay < 2)  {
    cfg->rx.dataDelay = (cfg->rx.tsPerFrame * 8) + cfg->rx.dataDelay;
  if (cfg->rx.clkMode == CSL_TSIP_CLKM_DBL)
    cfg->rx.dataDelay = cfg->rx.dataDelay * 2;
  }
  cfg->rx.dataDelay = cfg->rx.dataDelay - 2;

  CSL_tsipSIUSetRxDataDelay(hTsip, cfg->rx.dataDelay);
  CSL_tsipSIUSetTxOutputDelay(hTsip, cfg->rx.bdxDelay);
  CSL_tsipSIUSetRxFrameSyncPolarity(hTsip, cfg->rx.fsyncPol);
  CSL_tsipSIUSetRxFrameSyncClockPolarity(hTsip, cfg->rx.fsyncClkPol);
  CSL_tsipSIUSetRxDataClockPolarity(hTsip, cfg->rx.clkPol);
  CSL_tsipSIUSetRxDataRate(hTsip, cfg->rx.dataRate);
  CSL_tsipSIUSetRxClockMode(hTsip, cfg->rx.clkMode);

  /* Transmit and receive timeslot interrupt configuration */
  CSL_tsipDMATCUSetTxSuperFrameIntSelection(hTsip, cfg->tx.superFrameInt);
  CSL_tsipDMATCUSetTxFrameIntSelection(hTsip, cfg->tx.frameInt);
  CSL_tsipDMATCUSetTxFrameIntDelay(hTsip, cfg->tx.frameIntDelay);

  CSL_tsipDMATCUSetRxSuperFrameIntSelection(hTsip, cfg->rx.superFrameInt);
  CSL_tsipDMATCUSetRxFrameIntSelection(hTsip, cfg->rx.frameInt);
  CSL_tsipDMATCUSetRxFrameIntDelay(hTsip, cfg->rx.frameIntDelay);

  /* Return the hardware ID for the port */
  cfg->hwId = CSL_tsipGetPid(hTsip);

  return (tsip_OK);
} /* cslTsipConfigPort */


/*********************************************************************************
 * FUNCTION PURPOSE: Globally enable the tsip port
 *********************************************************************************
 * DESCRIPTION: The TDMU and SIU Enables are set. If either of them
 *              were already set then an error is returned.
 *********************************************************************************/
int16_t cslTsipEnablePort (int16_t port)
{
  int16_t   ret = 0;
  CSL_TsipHandle  hTsip;

  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  if (CSL_tsipTDMUIsEnabled(hTsip))
    ret |= (1 << 0);
  else  
    CSL_tsipTDMUEnable(hTsip);

  if (CSL_tsipSIUIsReceiveEnabled(hTsip))
    ret = ret | (1 << 1);
  else  
    CSL_tsipSIUEnableReceive(hTsip);

  if (CSL_tsipSIUIsTransmitEnabled(hTsip))
    ret = ret | (1 << 2);
  else  
    CSL_tsipSIUEnableTransmit(hTsip);

  return (ret);

} /* cslTsipEnablePort */


/******************************************************************************
 * FUNCTION PURPOSE: Enable a transmit tsip channel
 ******************************************************************************
 * DESCRIPTION: Loads the input CnID and enables a channel. Returns non-zero
 *              if the channel was already enabled.
 ******************************************************************************/
int16_t cslTsipEnableTxChannel (cslTsipChEnable_t *en)
{
  uint32_t        channel = en->channel;
  CSL_TsipHandle  hTsip;

  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(en->port);
  
  /* Read the enable status. Return if the channel is already enabled */
  if (CSL_tsipTDMUIsTxChannelEnabled(hTsip, channel))
    return (-1);

  /* Set configuration for Tx Channel A and B */
  CSL_tsipTDMUSetTxChannelAConfig(hTsip, channel, en->baseAddressGlobal, en->frameAlloc,
                                  en->frameSize, en->frameCount);
  CSL_tsipTDMUSetTxChannelBConfig(hTsip, channel, en->baseAddressGlobal, en->frameAlloc,
                                  en->frameSize, en->frameCount);


  /* Write the initial cnid and enable the channel */
  CSL_tsipTDMUTxChannelSetConfiguration(hTsip, channel, en->cnid);
  CSL_tsipTDMUTxChannelEnable(hTsip, channel);

  return (0);

} /* cslTsipEnableTxChannel */
     

/********************************************************************************
 * FUNCTION PURPOSE: Enable a receive tsip channel
 ********************************************************************************
 * DESCRIPTION: Loads the input cnid and enables a channel. Returns non-zero
 *              if the channel was already enabled.
 ********************************************************************************/
int16_t cslTsipEnableRxChannel (cslTsipChEnable_t *en)
{
  uint32_t        channel = en->channel;
  CSL_TsipHandle  hTsip;

  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(en->port);
  
  /* Read the enable status. Return if the channel is already enabled */
  if (CSL_tsipTDMUIsRxChannelEnabled(hTsip, channel))
    return (-1);

  /* Set configuration for Tx Channel A and B */
  CSL_tsipTDMUSetRxChannelAConfig(hTsip, channel, en->baseAddressGlobal, en->frameAlloc,
                                  en->frameSize, en->frameCount);
  CSL_tsipTDMUSetRxChannelBConfig(hTsip, channel, en->baseAddressGlobal, en->frameAlloc,
                                  en->frameSize, en->frameCount);


  /* Write the initial cnid and enable the channel */
  CSL_tsipTDMURxChannelSetConfiguration(hTsip, channel, en->cnid);
  CSL_tsipTDMURxChannelEnable(hTsip, channel);

  return (0);

} /* cslTsipEnableRxChannel */

/****************************************************************************************
 * FUNCTION PURPOSE: Enables a new Tx context
 ****************************************************************************************
 * DESCRIPTION: Writes the channel enable register with the new id, enabling the
 *              new context. Returns the current cnid;
 ****************************************************************************************/
uint32_t cslTsipNewTxContext (cslTsipContext_t *cxt)
{
  uint32_t chEn;
  /* Create the new register contents. The channel will be enabled */
  chEn = ((uint32_t)(cxt->cnid & 0xFE)) | cxt->cxnum;
  chEn = (chEn << 8) | 1;
  return (chEn);
} /* cslTsipNewTxContext */

/****************************************************************************************
 * FUNCTION PURPOSE: Enables a new Rx context
 ****************************************************************************************
 * DESCRIPTION: Writes the channel enable register with the new id, enabling the
 *              new context. 
 ****************************************************************************************/
uint32_t cslTsipNewRxContext (cslTsipContext_t *cxt)
{
  uint32_t chEn;
  /* Create the new register contents. The channel will be enabled */
  chEn = ((uint32_t)(cxt->cnid & 0xFE)) | cxt->cxnum;
  chEn = (chEn << 8) | 1;
  return (chEn);
} /* cslTsipNewRxContext */

/**************************************************************************************
 * FUNCTION PURPOSE: Return the free transmit context
 **************************************************************************************
 * DESCRIPTION: Finds which tx context is currently free, and stores values into the
 *              supplied context structure. 
 **************************************************************************************/
void cslTsipGetFreeTxContext (int16_t port, int16_t txChannel, cslTsipContext_t *cxt)
{
  CSL_TsipHandle  hTsip;
  uint16_t status;

  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  /* Return the port. On failure this will be replaced with -1 */
  cxt->port = port;
  
  /* Determine which context is free by reading the Transmit Channel Active 
   * Status Register and extracting the bits relative to the specified channel */
  status = CSL_tsipTDMUGetTxChannelStatus(hTsip, txChannel);

  /* Store pointers to the Timeslot Data Management Unit channel bitmaps and the
   * DMA Transfer Control Unit */

  if ((status == CSL_TSIP_CHST_INACTIVE) || (status == CSL_TSIP_CHST_BACTIVE))  {
    cxt->cxnum       = 0;
    cxt->channelMaps = (void *)&((hTsip->XBM[txChannel]).XBMA);
    /* store the handle of the tsip being used in the context */
    cxt->context     = (uint32_t)&(hTsip->DXCH[txChannel].ABASE);
 } else if (status == CSL_TSIP_CHST_AACTIVE)  {
    cxt->cxnum       = 1;
    cxt->channelMaps = (void *)&((hTsip->XBM[txChannel]).XBMB);
    /* store the handle of the tsip being used in the context */
    cxt->context     = (uint32_t)&(hTsip->DXCH[txChannel].BBASE);
  }  else  {
    cxt->port = -1;
    return;
  }

  /* Store related values for future context operations. The data rate sets the 
   * number of links, and is stored in the transmit control register */
  status = CSL_tsipSIUGetTxDataRate(hTsip);

  if (status == CSL_TSIP_DATARATE_8M)
    cxt->nLinks = 8;
  else if (status == CSL_TSIP_DATARATE_16M)
    cxt->nLinks = 4;
  else if (status == CSL_TSIP_DATARATE_32M)
    cxt->nLinks = 2;
  else {
    cxt->port = -1;
    return;
  }
  
  /* Read the currently active cnid */
  cxt->cnid = CSL_tsipTDMUTxChannelGetConfiguration(hTsip, txChannel);
  
  /* The number of timeslots per link is stored in the transmit size register.
   * The value stored is actually the number of logical timeslots per frame-1, which
   * is converted based on the number of links to timeslots per link. There are
   * always 8 physical timeslots in a logical timeslot.  */
  cxt->nTs = CSL_tsipSIUGetTxFrameSize(hTsip)+1;
  cxt->nTs = (cxt->nTs * 8) / cxt->nLinks;

  /* Clear the context */
  memset (cxt->channelMaps, 0, CSL_TSIP_BITMAP_SIZE * sizeof(uint32_t));

} /* cslTsipGetFreeTxContext */

/*************************************************************************************
 * FUNCTION PURPOSE: Return the free receive context
 *************************************************************************************
 * DESCRIPTION: Finds which rx context is currently free, and stores values into the
 *              supplied context structure. If no free context is found, the port
 *              value returned in the context is -1.
 *************************************************************************************/
void cslTsipGetFreeRxContext (int16_t port, int16_t rxChannel, cslTsipContext_t *cxt)
{
  CSL_TsipHandle  hTsip;
  uint16_t status;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);
  
  /* Return the port. On failure this will be replaced with -1 */
  cxt->port = port;

  /* Determine which context is free by reading the Transmit Channel Active 
   * Status Register and extracting the bits relative to the specified channel */
  status = CSL_tsipTDMUGetRxChannelStatus(hTsip, rxChannel);

  /* Store pointers to the Timeslot Data Management Unit channel bitmaps and the
   * DMA Transfer Control Unit */
  if ((status == CSL_TSIP_CHST_INACTIVE) || (status == CSL_TSIP_CHST_BACTIVE))  {
    cxt->cxnum       = 0;
    cxt->channelMaps = (void *)&((hTsip->RBM[rxChannel]).RBMA);
    /* store the handle of the tsip being used in the context */
    cxt->context     = (uint32_t)&(hTsip->DRCH[rxChannel].ABASE);
  } else if (status == CSL_TSIP_CHST_AACTIVE)  {
    cxt->cxnum       = 1;
    cxt->channelMaps = (void *)&((hTsip->RBM[rxChannel]).RBMB);
    /* store the handle of the tsip being used in the context */
    cxt->context     = (uint32_t)&(hTsip->DRCH[rxChannel].BBASE);
  }  else  {
    cxt->port = -1;
    return;
  }

  /* Store related values for future context operations. The data rate sets the 
   * number of links, and is stored in the transmit control register */
  status = CSL_tsipSIUGetRxDataRate(hTsip);

  if (status == CSL_TSIP_DATARATE_8M)
    cxt->nLinks = 8;
  else if (status == CSL_TSIP_DATARATE_16M)
    cxt->nLinks = 4;
  else if (status == CSL_TSIP_DATARATE_32M)
    cxt->nLinks = 2;
  else {
    cxt->port = -1;
    return;
  }
  
  /* Read the currently active cnid */
  cxt->cnid = CSL_tsipTDMURxChannelGetConfiguration(hTsip, rxChannel);

  /* The number of timeslots per link is stored in the transmit size register.
   * The value stored is actually the number of logical timeslots per frame-1, which
   * is converted based on the number of links to timeslots per link. There are
   * always 8 physical timeslots in a logical timeslot.  */
  cxt->nTs = CSL_tsipSIUGetRxFrameSize(hTsip)+1;
  cxt->nTs = (cxt->nTs * 8) / cxt->nLinks;

  /* Clear the context */
  memset (cxt->channelMaps, 0, CSL_TSIP_BITMAP_SIZE * sizeof(uint32_t));
} /* cslTsipGetFreeRxContext */

/**************************************************************************************
 * FUNCTION PURPOSE: Return the active rx context
 **************************************************************************************
 * DESCRIPTION: Finds which rx context is currently running and stores values into the
 *              supplied context structure.
 **************************************************************************************/
void cslTsipGetActiveRxContext (int16_t port, int16_t rxChannel, cslTsipContext_t *cxt)
{
  CSL_TsipHandle  hTsip;
  uint16_t status;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);
  
  /* Return the port. On failure this will be replaced with -1 */
  cxt->port = port;

  /* Determine which context is free by reading the Transmit Channel Active 
   * Status Register and extracting the bits relative to the specified channel */
  status = CSL_tsipTDMUGetRxChannelStatus(hTsip, rxChannel);

  /* Store pointers to the Timeslot Data Management Unit channel bitmaps and the
   * DMA Transfer Control Unit */

  if (status == CSL_TSIP_CHST_AACTIVE)  {
    cxt->cxnum       = 0;
    cxt->channelMaps = (void *)&((hTsip->RBM[rxChannel]).RBMA);
    /* store the handle of the tsip being used in the context */
    cxt->context     = (uint32_t)&(hTsip->DRCH[rxChannel].ABASE);
  } else if (status == CSL_TSIP_CHST_BACTIVE)  {
    cxt->cxnum       = 1;
    cxt->channelMaps = (void *)&((hTsip->RBM[rxChannel]).RBMB);
    /* store the handle of the tsip being used in the context */
    cxt->context     = (uint32_t)&(hTsip->DRCH[rxChannel].BBASE);
  }  else  {
    cxt->port = -1;
    return;
  }

  /* Store related values for future context operations. The data rate sets the 
   * number of links, and is stored in the transmit control register */
  status = CSL_tsipSIUGetRxDataRate(hTsip);

  if (status == CSL_TSIP_DATARATE_8M)
    cxt->nLinks = 8;
  else if (status == CSL_TSIP_DATARATE_16M)
    cxt->nLinks = 4;
  else if (status == CSL_TSIP_DATARATE_32M)
    cxt->nLinks = 2;
  else {
    cxt->port = -1;
    return;
  }
  
  /* Read the currently active cnid */
  cxt->cnid = CSL_tsipTDMURxChannelGetConfiguration(hTsip, rxChannel);
  
  /* The number of timeslots per link is stored in the transmit size register.
   * The value stored is actually the number of logical timeslots per frame-1, which
   * is converted based on the number of links to timeslots per link. There are
   * always 8 physical timeslots in a logical timeslot.  */
  cxt->nTs = CSL_tsipSIUGetRxFrameSize(hTsip)+1;
  cxt->nTs = (cxt->nTs * 8) / cxt->nLinks;

} /* cslTsipGetActiveRxContext */


/***************************************************************************************
 * DESCRIPTION: Return the global enable status
 ***************************************************************************************
 * DESCRIPTION: Returns non-zero if the tx and rx are enabled in the global control
 ***************************************************************************************/
int16_t cslTsipGetGlobalStatus (int16_t port)
{
  CSL_TsipHandle  hTsip;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  if (CSL_tsipSIUIsReceiveEnabled(hTsip) && 
      CSL_tsipSIUIsTransmitEnabled(hTsip))  
    return (1);
  else
    return (0);

} /* cslTsipGetGlobalStatus */

/************************************************************************************
 * FUNCTION PURPOSE: Provide a new transmit context to the TSIP
 ************************************************************************************
 * DESCRIPTION: Sets a new context to the pending state. It will take effect
 *              on the next super-frame boundary
 ************************************************************************************/
void cslTsipSendNewTxContext (int16_t port, uint32_t cnId, int16_t txChannel)
{
  uint32_t chEn;
  CSL_TsipHandle  hTsip;

  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);
  chEn = cnId>>8;

  CSL_tsipTDMUTxChannelSetConfiguration(hTsip, txChannel, chEn);
  CSL_tsipTDMUTxChannelEnable(hTsip, txChannel);

} /* cslTsipSendNewTxContext */


/**************************************************************************************
 * FUNCTION PURPOSE: Provide a new receive context to the TSIP
 **************************************************************************************
 * DESCRIPTION: Sets a new context to the pending state. It will take effect on
 *              the new super-frame boundary
 **************************************************************************************/
void cslTsipSendNewRxContext (int16_t port, uint32_t cnId, int16_t rxChannel)
{  
  uint32_t chEn;
  CSL_TsipHandle  hTsip;

  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);
  chEn = cnId>>8;

  CSL_tsipTDMURxChannelSetConfiguration(hTsip, rxChannel, chEn);
  CSL_tsipTDMURxChannelEnable(hTsip, rxChannel);

} /* cslTsipSendNewRxContext */


/**************************************************************************************
 * FUNCTION PURPOSE: Returns the current tx free running frame count
 **************************************************************************************
 * DESCRIPTION: Reads the tx frfc and returns it.
 **************************************************************************************/
uint32_t cslTsipReadTxFrfc (int16_t port)
{
  uint32_t        frfc;
  CSL_TsipHandle  hTsip;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  frfc = CSL_tsipTDMUGetTxFreeRunningCounter(hTsip);
  return (frfc);

} /* cslTsipReadTxFrfc */

/***************************************************************************************
 * FUNCTION PURPOSE: Returns the current rx free running frame count
 ***************************************************************************************
 * DESCRIPTION: Reads teh rx frfc and returns it.
 ***************************************************************************************/
uint32_t cslTsipReadRxFrfc (int16_t port)
{
  uint32_t        frfc;
  CSL_TsipHandle  hTsip;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  frfc = CSL_tsipTDMUGetRxFreeRunningCounter(hTsip);
  return (frfc);

} /* cslTsipReadRxFrfc */


/*************************************************************************************** 
 * FUNCTION PURPOSE: Disable the tx and rx dma for a channel
 ***************************************************************************************
 * DESCRIPTION: The channel enable bits for the tx and rx channel are cleared
 ***************************************************************************************/
void cslTsipDisableTxRxDma (int16_t port, int16_t chnum)
{
  cslTsipStopRxDma(port, chnum);
  cslTsipStopTxDma(port, chnum);

} /* cslTsipDisableTxRxDma */


/*************************************************************************************** 
 * FUNCTION PURPOSE: Enable the tx and rx dma for a channel (translator)
 ***************************************************************************************
 * DESCRIPTION: The channel enable bits for the tx and rx channel are set
 ***************************************************************************************/
void cslTsipEnableTxRxDma (int16_t port, int16_t chnum)
{
  cslTsipStartRxDma(port, chnum);
  cslTsipStartTxDma(port, chnum);

} /* cslTsipDisableTxRxDma */


/*******************************************************************************
 * FUNCTION PURPOSE: start an RX DMA channel
 *******************************************************************************
 * DESCRIPTION: This function is used to enable an RX DMA channel. No
 *              dma channel configuration is performed.
 *******************************************************************************/
void cslTsipStartRxDma (int16_t port, int16_t chan)
{
  CSL_TsipHandle  hTsip;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  CSL_tsipTDMURxChannelEnable(hTsip, chan);

} /* cslTsipStartRxDma */


/*******************************************************************************
 * FUNCTION PURPOSE: start a TX DMA channel
 *******************************************************************************
 * DESCRIPTION: This function is used to enable a TX DMA channel. No
 *              dma channel configuration is performed.
 *******************************************************************************/
void cslTsipStartTxDma (int16_t port, int16_t chan)
{
  CSL_TsipHandle  hTsip;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  CSL_tsipTDMUTxChannelEnable(hTsip, chan);

} /* cslTsipStartTxDma */


/********************************************************************************
 * FUNCTION PURPOSE: Stop an RX DMA channel
 ********************************************************************************
 * DESCRIPTION: This function is used to disable an RX DMA channel. No
 *              channel configuration information is changed
 ********************************************************************************/
void cslTsipStopRxDma (int16_t port, int16_t chan)
{
  CSL_TsipHandle  hTsip;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  CSL_tsipTDMUTxChannelDisable(hTsip, chan);

} /* cslTsipStopRxDma */


/********************************************************************************
 * FUNCTION PURPOSE: Stop a TX DMA channel
 ********************************************************************************
 * DESCRIPTION: This function is used to disable a TX DMA channel. No
 *              channel configuration information is changed
 ********************************************************************************/
void cslTsipStopTxDma (int16_t port, int16_t chan)
{
  CSL_TsipHandle  hTsip;
  
  /* Open the TSIP module */
  hTsip  = CSL_tsipOpen(port);

  CSL_tsipTDMUTxChannelDisable(hTsip, chan);

} /* cslTsipStopTxDma */

/***************************************************************************************
 * FUNCTION PURPOSE: Set the base address and allocation for a context
 ***************************************************************************************
 * DESCRIPTION: The base address and allocation for a context are changed. 
 *              No check is made to see if the context is actually active
 ***************************************************************************************/
void cslTsipSetBaseAndAlloc (cslTsipContext_t *cxt, uint32_t base, uint32_t alloc)
{ 
  uint32_t *reg;

  /* the context base address is stored in the cslTsipContext_t*/
  reg  = (uint32_t*)CSL_TSIP_CONTEXT_ADDR_BASE(cxt->context);
  *reg = base;

  /* move to the alloc */
  reg  = (uint32_t*)CSL_TSIP_CONTEXT_ADDR_ALLOC(cxt->context);
  *reg = alloc;

}


/***************************************************************************************
 * FUNCTION PURPOSE: Compute and set the frame size for a context
 ***************************************************************************************
 * DESCRIPTION: The frame size is used only for tx, but can also be set for rx. 
 *              The frame size specifies the amount of data transferred to the peripheral
 *              during transmit per frame, including the leading and trailing cnid and 
 *              frfc. It is dependent on the number and alignment of channels, and is
 *              rounded up to a 32 bit word size.
 ****************************************************************************************/
uint32_t cslTsipGetAndSetFrameSize (cslTsipContext_t *cxt)
{
  uint16_t offset;
  uint32_t *reg;
  
  /* Read the offset to the end of all active channels */
  offset = cslTsipGetDataOffset (cxt, cxt->nLinks+1, cxt->nTs+1);

  /* Round the value up to a multiple of 4 bytes */
  offset = (offset + 3) & 0xfffc;

  /* Add the 4 bytes of trailer CnID/frfc */
  offset = offset + 4;

  /* Write this value into the provided context */
  reg  = (uint32_t*)CSL_TSIP_CONTEXT_ADDR_FRAMESIZE(cxt->context);
 *reg  = offset;

  return (offset);

} /* cslTsipGetAndSetFrameSize */


/************************************************************************************
 * FUNCTION PURPOSE: Enable a timeslot.
 ************************************************************************************
 * DESCRIPTION: A timeslot is enabled by writing the di-bit in the channnel map
 *              pointed to in the context.
 ************************************************************************************/
void cslTsipEnableTimeslot (cslTsipContext_t *cxt, uint16_t link, uint16_t ts, int16_t compand)
{
  uint16_t bitNum;
  uint16_t wordNum;
  int16_t  diBitIndex;
  uint32_t *mapBase;
  uint32_t  tsipWord;


  /* Convert the link and timeslot into a di-bit index */
  diBitIndex = (cxt->nLinks * ts) + link;

  /* Convert the di-bit index into a word number and a base bit. The bits are
   * packed in reverse order (see the spec) */
  wordNum = (diBitIndex * 2) / 32;
  bitNum  = 30 - ((diBitIndex * 2) - (wordNum * 32));

  /* Read the current word at that context and insert the desired di-bit */
  mapBase = (uint32_t *)(cxt->channelMaps);
  tsipWord = mapBase[wordNum];
  tsipWord = TSIP_SETBITFIELD(tsipWord, compand, bitNum+1, bitNum);
  mapBase[wordNum] = tsipWord;

} /* cslTsipEnableTimeslot */

/***************************************************************************************
 * FUNCTION PURPOSE: Scans the context to find the index for the specified timeslot
 ***************************************************************************************
 * DESCRIPTION: The location of timeslot data in the DMA buffer is a function of
 *              which timeslots that preceed this one are enabled. This function 
 *              counts these ancestors, along with alignment requirements, to find
 *              the offset for the desired timeslot. The data is viewed as a byte
 *              stream, so the returned offset is in bytes.
 *
 *              There is a 4 byte header for the CnID/FRFC which is included here.
 ****************************************************************************************/
uint16_t cslTsipGetDataOffset (cslTsipContext_t *cxt, uint16_t link, uint16_t ts)
{
  uint32_t tsipWord, *mapBase;
  int sizeBytes, bitNum;
  unsigned int diBitIdx, nDiBits, offset, diBit;
  unsigned short adjustBySizeBytes;

  /* Pre-compute the exact number of iterations needed (diBits to traverse).
   * If either ts or link is out of range, then this is a memory size request.
   * For memory size request, we don't remove sizeBytes from the end result;
   * otherwise we do to maintain bit-exactness with original code.
   */
  if ((ts >= cxt->nTs) || (link >= cxt->nLinks)) {
    /* Size check would stop inner break in orig code */
    nDiBits = cxt->nLinks * cxt->nTs;
    adjustBySizeBytes = FALSE;
  } else {
    nDiBits = ts * cxt->nLinks + link;
    adjustBySizeBytes = TRUE;
  }
  mapBase = (uint32_t *)(cxt->channelMaps);
  offset = 0;
  for (diBitIdx = 0; diBitIdx < nDiBits; diBitIdx++) {
    /* read 16 sets of diBits from IOspace, only when needed; process
     * remaining 15 sets without touchign IOSpace
     */
    if ((diBitIdx & 0xf) == 0) {
      bitNum = 30;
      tsipWord = *mapBase++;
    } else {
      bitNum -= 2;
    }
    diBit = (tsipWord >> bitNum) & 0x3;

    /* Return the size of the element */
    if (diBit == CSL_TSIP_TIMESLOT_DISABLED) {
      sizeBytes = 0;
    } else if (diBit == CSL_TSIP_TIMESLOT_LINEAR) {
      sizeBytes = 1;
    } else {
      sizeBytes = 2;
      if (offset & 0x1) {
        offset += 1;
      }
    }
    offset += sizeBytes;
  }

  if (! adjustBySizeBytes) {
    sizeBytes = 0;
  }
  /* 4-sizeBytes comes from the fact that the "return"
   * in the original code above is before offset += sizeBytes
   */
  return (uint16_t)(offset + 4 - sizeBytes);

} /* cslTsipGetDataOffset */


/*******************************************************************************************
 * FUNCTION PURPOSE: Returns a timeslot offst list
 *******************************************************************************************
 * DESCRIPTION: The di-bit index is converted into a list of timeslots and offsets. 
 *              To make the list searchable by a binary search, each timeslot value
 *              has the link value incorporated into the lowest three bits;
 *
 *              /---------------------------------------------------------------\
 *              | 15                                     3 |  2             0   |
 *              |        Timeslot number                   |    Link number     |
 *              \---------------------------------------------------------------/
 *
 *              The offset array has the offset to the timeslot, in bytes. So if element
 *              4 in the tsmap array has timeslot 10, link 1, the element 4 in the
 *              offset array has the offset to that link/timeslot in the dma buffers.
 *
 *              The number of elements in the array is returned.
 ***************************************************************************************************/
int16_t cslTsipCreateOffsetMaps (cslTsipContext_t *cxt, uint16_t *tsmap, uint16_t *offsets, int16_t arraySize)
{
  uint32_t tsipWord, *mapBase;
  int sizeBytes, bitNum, p;
  unsigned int diBitIdx, nDiBits, offset, diBit, ts, link, nLink;

  mapBase = (uint32_t *)(cxt->channelMaps);
  offset = 0;
  nDiBits = cxt->nLinks * cxt->nTs;
  p = 0;
  ts = 0;
  link = 0;
  nLink = cxt->nLinks;
  /* Unconditionally traverse/analyze all diBits */
  for (diBitIdx = 0; diBitIdx < nDiBits; diBitIdx++) {
    /* read 16 sets of diBits from IOspace, only when needed; process
     * remaining 15 sets without touchign IOSpace
     */
    if ((diBitIdx & 0xf) == 0) {
      bitNum = 30;
      tsipWord = *mapBase++;
    } else {
      bitNum -= 2;
    }
    diBit = (tsipWord >> bitNum) & 0x3;

    /* Return the size of the element */
    if (diBit == CSL_TSIP_TIMESLOT_DISABLED) {
      sizeBytes = 0;
    } else if (diBit == CSL_TSIP_TIMESLOT_LINEAR) { 
      sizeBytes = 1;
    } else {
      sizeBytes = 2;
      if (offset & 1) {
        offset += 1;
      }
    }
    /* If the timeslot is enabled, record its propreties */
    if (sizeBytes) {
      tsmap[p]     = CSL_TSIP_CREATE_OFFSET_MAPVAL(ts, link);
      offsets[p++] = offset + 4;
    }
    offset += sizeBytes;

    if (p >= arraySize) {
      break;
    }
    /* Maintain link and ts (like i and j in orig loop) */
    link++;
    if (link >= nLink) {
      link = 0;
      ts++;
    }
  }

  return (int16_t)p;

} /* cslTsipCreateOffsetMaps */
