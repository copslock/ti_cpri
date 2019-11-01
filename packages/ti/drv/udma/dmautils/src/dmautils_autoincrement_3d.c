/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 */


/*
 *******************************************************************************
 *
 *  DMA utility functions
 *
 *******************************************************************************
*/

/**
 *****************************************************************************
 * \file  dmautils_autoincrement_3d.c
 *
 * \brief*  This file contains function implementation for dma util functions for
 *  configuring DMA on J7 device for autoincrement 3D usecase.
 *
 *
 *****************************************************************************
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <math.h>
#include <stdint.h>

#include "ti/drv/udma/udma.h"
#include "ti/drv/udma/dmautils/src/dmautils_autoincrement_3d_priv.h"
#include "ti/drv/udma/dmautils/include/dmautils_autoincrement_3d.h"

#if defined (HOST_EMULATION)
  #if defined (__C7100__)
    #include <c7x_host_emulation.h>
  #endif
#else
  #if defined (__C7100__)
    #include <c7x.h>
  #endif
#endif


/* If its a Loki Build then force the mode to be in hostemulation as Loki doesnt support DRU */
#if defined (LOKI_BUILD)
#define HOST_EMULATION (1U)
#endif

#define DMAUTILS_ALIGN_CEIL(VAL, ALIGN) ((((VAL) + (ALIGN) - 1)/(ALIGN)) * (ALIGN) )


#ifdef HOST_EMULATION
#include "stdlib.h"

void hostEmulation_updateTriggerCount(struct Udma_DrvObj * udmaDrvHandle,
                                                                              volatile uint64_t *pSwTrigReg)
{
  uint32_t i;
  CSL_DRU_t *druRegs = udmaDrvHandle->utcInfo[0].druRegs;
  uintptr_t origPtr = (uintptr_t)pSwTrigReg;
  uintptr_t currPtr;

  for ( i = 0; i < udmaDrvHandle->utcInfo[0].numCh; i++)
  {
    currPtr = (uintptr_t)&druRegs->CHRT[i].CHRT_SWTRIG;
    if ( currPtr == origPtr)
    {
      /* Use reserved space for tracking number of triggers submitted for a given channel */
      druRegs->CHRT[i].Resv_256[0]++;
      break;
    }
  }
}

void hostEmulation_druChSubmitAtomicTr(CSL_DRU_t *pRegs,
                                           uint32_t chId,
                                           void *  vdata)
{
  CSL_UdmapTR * tr = (CSL_UdmapTR *)vdata;
  CSL_UdmapTR * origTransferRecord   = ( CSL_UdmapTR *)&pRegs->CHATOMIC[chId];
  CSL_UdmapTR * nextTransferRecord  = (CSL_UdmapTR *)&pRegs->CHATOMIC[chId].DEBUG[0];
  CSL_UdmapTR * nextTransferRecord1 = (CSL_UdmapTR *)&pRegs->CHATOMIC[chId].DEBUG[1];
  CSL_UdmapTR * nextTransferRecord2 = (CSL_UdmapTR *)&pRegs->CHATOMIC[chId].DEBUG[2];

  /* Use reserved space for tracking number of triggers submitted for a given channel */
  pRegs->CHRT[chId].Resv_256[0] = 0;

  *origTransferRecord  = *tr;
  *nextTransferRecord = *tr;
  *nextTransferRecord1 = *tr;
  *nextTransferRecord2 = *tr;
}


uint64_t hostEmulation_addressUpdate( uint64_t base, int32_t offset, uint64_t addrMask )
{
  uint64_t newAddr;

  newAddr = base + offset;

  if ( addrMask != 0 )
  {
    newAddr = ( base & addrMask ) | ( newAddr & ~addrMask );
  }

  return newAddr;
}

void hostEmulation_circMask( uint32_t cbk0, uint32_t cbk1, uint64_t * circMask0, uint64_t * circMask1  )
{
  uint32_t blockSize0 = cbk0 + 9U; /* power-of-2 of block size in bytes */
  uint32_t blockSize1 = cbk0 + cbk1 + 10U; /* power-of-2 of block size in bytes */

  if ( blockSize1 > 31U )
  {
    blockSize1 = 32U; /* clamp to 4GB maximum size */
  }
  *circMask0 = (~0ULL) << blockSize0;
  *circMask1 = (~0ULL) << blockSize1;
}

static void hostEmulation_triggerDMA(struct Udma_DrvObj * udmaDrvHandle)
{
  uint32_t chId;
  CSL_DRU_t * druRegs;
  CSL_UdmapCfg  * udmapRegs;
  CSL_ringacc_cfgRegs * ringAccCfgRegs;


  druRegs             = udmaDrvHandle->utcInfo[0].druRegs;
  ringAccCfgRegs  = udmaDrvHandle->raRegs.pCfgRegs;
  udmapRegs        = &udmaDrvHandle->udmapRegs;


  for ( chId = 0; chId < 32; chId++)//:TODO: Remove hard coded value of 32
  {
    if ( (druRegs->CHRT[chId].CHRT_SWTRIG & CSL_DRU_CHRT_SWTRIG_GLOBAL_TRIGGER0_MASK) == 1U)
    {
      uint8_t *srcPtr;
      uint8_t *dstPtr;
      uint32_t triggerType;
      uint32_t circDir;
      uint32_t icnt0;
      uint32_t CBK0, CBK1;
      uint32_t AM0, AM1, AM2, AM3;
      uint64_t srcAM0, srcAM1, srcAM2, srcAM3;
      uint64_t dstAM0, dstAM1, dstAM2, dstAM3;
      uint64_t circMask0;
      uint64_t circMask1;
      uint32_t AMODE;
      uint32_t loopCnt1Reset, loopCnt2Reset;
      uint8_t * interimBuffer = NULL;
      uint32_t srcLoopExitCondition = 0;
      uint32_t dstLoopExitCondition = 0;
      uint32_t totalSrcCnt, totalDstCnt;
      /* Clear the sw trigger so that next trigger can happen */


      druRegs->CHRT[chId].Resv_256[0]--;
      /* Use reserved space for tracking number of triggers submitted for a given channel */
      if ( druRegs->CHRT[chId].Resv_256[0] == 0 )
      {
        druRegs->CHRT[chId].CHRT_SWTRIG  = druRegs->CHRT[chId].CHRT_SWTRIG & (uint64_t)(~CSL_DRU_CHRT_SWTRIG_GLOBAL_TRIGGER0_MASK);
      }

      CSL_UdmapTR * origTransferRecord  = (CSL_UdmapTR *)(void *) &druRegs->CHATOMIC[chId];
      CSL_UdmapTR * nextTransferRecord = (CSL_UdmapTR *)(void *) &druRegs->CHATOMIC[chId].DEBUG[0];
      CSL_UdmapTR * nextTransferRecord1 = (CSL_UdmapTR *)(void *) &druRegs->CHATOMIC[chId].DEBUG[1];
      CSL_UdmapTR * nextTransferRecord2 = (CSL_UdmapTR *)(void *) &druRegs->CHATOMIC[chId].DEBUG[2];

      /* Do the actual transfer */

      triggerType = CSL_FEXT(origTransferRecord->flags, UDMAP_TR_FLAGS_TRIGGER0_TYPE);
      AMODE = CSL_FEXT(origTransferRecord->fmtflags, UDMAP_TR_FMTFLAGS_AMODE);

      if ( AMODE == CSL_UDMAP_TR_FMTFLAGS_AMODE_CIRCULAR)
      {
        circDir        = CSL_FEXT(origTransferRecord->fmtflags, UDMAP_TR_FMTFLAGS_DIR);
        CBK0         = CSL_FEXT(origTransferRecord->fmtflags, UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK0);
        CBK1         = CSL_FEXT(origTransferRecord->fmtflags, UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK1);
        AM0           = CSL_FEXT(origTransferRecord->fmtflags, UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM0);
        AM1           = CSL_FEXT(origTransferRecord->fmtflags, UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM1);
        AM2           = CSL_FEXT(origTransferRecord->fmtflags, UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM2);
        AM3           = CSL_FEXT(origTransferRecord->fmtflags, UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM3);
        hostEmulation_circMask(CBK0, CBK1, &circMask0, &circMask1);
      }
      else
      {
        /* Linear Addressing */
        circMask0 = (~0ULL);
        circMask1 = (~0ULL);
        AM0 = 0;
        AM1 = 0;
        AM2 = 0;
        AM3 = 0;
        circDir = 0;
      }

      if ( circDir == CSL_UDMAP_TR_FMTFLAGS_DIR_DST_USES_AMODE)
      {
        dstAM0 = (AM0 == 0 ) ? 0 : ( (AM0 == 1) ? circMask0 : circMask1  );/* Circular update */
        srcAM0 = 0;/* Linear Update */

        dstAM1 = (AM1 == 0 ) ? 0 : ( (AM1 == 1) ? circMask0 : circMask1  );/* Circular update */
        srcAM1 = 0;/* Linear Update */

        dstAM2 = (AM2 == 0 ) ? 0 : ( (AM2 == 1) ? circMask0 : circMask1  );/* Circular update */
        srcAM2 = 0;/* Linear Update */

        dstAM3 = (AM3 == 0 ) ? 0 : ( (AM3 == 1) ? circMask0 : circMask1  );/* Circular update */
        srcAM3 = 0;/* Linear Update */
      }
      else
      {
        dstAM0 = 0;/* Linear Update */
        srcAM0 = (AM0 == 0 ) ? 0 : ( (AM0 == 1) ? circMask0 : circMask1  );

        dstAM1 = 0;/* Linear Update */
        srcAM1 = (AM1 == 0 ) ? 0 : ( (AM1 == 1) ? circMask0 : circMask1  );

        dstAM2 = 0;/* Linear Update */
        srcAM2 = (AM2 == 0 ) ? 0 : ( (AM2 == 1) ? circMask0 : circMask1  );

        dstAM3 = 0;/* Linear Update */
        srcAM3 = (AM3 == 0 ) ? 0 : ( (AM3 == 1) ? circMask0 : circMask1  );

      }

      loopCnt1Reset = 0;
      loopCnt2Reset = 0;

      /* allocate worst case, actual buffer used will depend on trugerType */
      interimBuffer = (uint8_t *)malloc(origTransferRecord->icnt0 * origTransferRecord->icnt1 * origTransferRecord->icnt2 *
                                        origTransferRecord->icnt3);
      dstPtr = interimBuffer;

      totalSrcCnt = nextTransferRecord->icnt0 * nextTransferRecord->icnt1 * nextTransferRecord->icnt2 * nextTransferRecord->icnt3;
      totalDstCnt = nextTransferRecord->dicnt0 * nextTransferRecord->dicnt1 * nextTransferRecord->dicnt2 * nextTransferRecord->dicnt3;

      srcLoopExitCondition  = ( totalSrcCnt < totalDstCnt )? totalSrcCnt : totalDstCnt;
      dstLoopExitCondition  = srcLoopExitCondition;
      /* Set the minimum value of icnt3 for both src and dst as TR completes whenever anyone of them  is exauhsted */
      /* Transfer source data to a intermediate linear buffer */

      while (1)
      {
        srcPtr = (uint8_t *)nextTransferRecord->addr;
/*        dstPtr = interimBuffer +
                    (origTransferRecord->icnt1 - nextTransferRecord->icnt1) * origTransferRecord->icnt0 +
                    (origTransferRecord->icnt2 - nextTransferRecord->icnt2) * origTransferRecord->icnt0 * origTransferRecord->icnt1;*/

        for (icnt0 = 0; icnt0 < nextTransferRecord->icnt0; icnt0++)
        {
          *dstPtr = *srcPtr;
            srcPtr = (uint8_t *)hostEmulation_addressUpdate((uint64_t)srcPtr, 1, srcAM0);
            dstPtr++;
        }
        nextTransferRecord->icnt1--;
        nextTransferRecord->addr   = hostEmulation_addressUpdate(nextTransferRecord->addr ,  nextTransferRecord->dim1,   srcAM1);

        if ( nextTransferRecord->icnt1 == 0)
        {
          loopCnt1Reset = 1;
          nextTransferRecord->icnt2--;
          nextTransferRecord->icnt1 = origTransferRecord->icnt1;

          nextTransferRecord->addr   = hostEmulation_addressUpdate(nextTransferRecord1->addr ,  nextTransferRecord->dim2,   srcAM2);

          nextTransferRecord1->addr   = nextTransferRecord->addr;
        }

        if ( nextTransferRecord->icnt2 == 0)
        {
          loopCnt2Reset= 1;
          nextTransferRecord->icnt3--;
          nextTransferRecord->icnt2 = origTransferRecord->icnt2;

          nextTransferRecord->addr   = hostEmulation_addressUpdate(nextTransferRecord2->addr,  nextTransferRecord->dim3,   srcAM3);

          nextTransferRecord1->addr   = nextTransferRecord->addr;
          nextTransferRecord2->addr   = nextTransferRecord->addr;
        }

        if ( nextTransferRecord->icnt3 == 0)
        {
          CSL_REG64_FINS(&druRegs->CHRT[chId].CHRT_CTL, DRU_CHRT_CTL_ENABLE, 0);
          break;
        }


        if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC )
        {
          break;
        }
        else if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC )
        {
          if ( loopCnt1Reset == 1)
          {
            nextTransferRecord1->addr   = nextTransferRecord->addr;
            break;
          }
        }
        else if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT3_DEC )
        {
          if ( loopCnt2Reset == 1)
          {
            nextTransferRecord2->addr   = nextTransferRecord->addr;
            break;
          }
        }
        else
        {
          /*Indicates 4D sync,  Just continue */
        }
        srcLoopExitCondition--;
        if ( srcLoopExitCondition == 0)
        {
          nextTransferRecord->icnt3 = 0;
          CSL_REG64_FINS(&druRegs->CHRT[chId].CHRT_CTL, DRU_CHRT_CTL_ENABLE, 0);
          break;
        }
      }

      loopCnt1Reset = 0;
      loopCnt2Reset = 0;

     srcPtr = interimBuffer;
      /* Now copy the intermediate data to the destination buffer */
     while (1)
      {
        dstPtr = (uint8_t *)nextTransferRecord->daddr;

        for (icnt0 = 0; icnt0 < nextTransferRecord->dicnt0; icnt0++)
        {
          *dstPtr = *srcPtr;
           *srcPtr++;
            dstPtr = (uint8_t *)hostEmulation_addressUpdate((uint64_t)dstPtr, 1, dstAM0);
        }

        nextTransferRecord->dicnt1--;

        nextTransferRecord->daddr = hostEmulation_addressUpdate(nextTransferRecord->daddr, nextTransferRecord->ddim1, dstAM1);

        if ( nextTransferRecord->dicnt1 == 0)
        {
          loopCnt1Reset = 1;
          nextTransferRecord->dicnt2--;
          nextTransferRecord->dicnt1 = origTransferRecord->dicnt1;

          nextTransferRecord->daddr = hostEmulation_addressUpdate(nextTransferRecord1->daddr, nextTransferRecord->ddim2, dstAM2);
          nextTransferRecord1->daddr = nextTransferRecord->daddr;
        }

        if ( nextTransferRecord->dicnt2 == 0)
        {
          loopCnt2Reset= 1;
          nextTransferRecord->dicnt3--;
          nextTransferRecord->dicnt2 = origTransferRecord->dicnt2;

          nextTransferRecord->daddr = hostEmulation_addressUpdate(nextTransferRecord2->daddr, nextTransferRecord->ddim3, dstAM3);

          nextTransferRecord1->daddr = nextTransferRecord->daddr;
          nextTransferRecord2->daddr = nextTransferRecord->daddr;
        }

        if ( nextTransferRecord->dicnt3 == 0)
        {
          CSL_REG64_FINS(&druRegs->CHRT[chId].CHRT_CTL, DRU_CHRT_CTL_ENABLE, 0);
          break;
        }

        if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC )
        {
          break;
        }
        else if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC )
        {
          if ( loopCnt1Reset == 1)
          {
            break;
          }
        }
        else if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT3_DEC )
        {
          if ( loopCnt2Reset == 1)
          {
            break;
          }
        }

        dstLoopExitCondition--;
        if ( dstLoopExitCondition == 0)
        {
          nextTransferRecord->dicnt3 = 0;
          CSL_REG64_FINS(&druRegs->CHRT[chId].CHRT_CTL, DRU_CHRT_CTL_ENABLE, 0);
          break;
        }
      }

      if (( nextTransferRecord->icnt3 == 0 ) || ( nextTransferRecord->dicnt3 == 0 ))
      {
        /* this indicates that TR is received from PSIL */
        if ( (druRegs->CHNRT[chId].CFG & CSL_DRU_CHNRT_CFG_CHAN_TYPE_OWNER_MASK) != 0 )
        {
           CSL_UdmapCppi5TRPD * trDescriptor;
           uint64_t * ringPtr;
           uint64_t currTr;
           uint64_t numTr;
           CSL_UdmapTR           *pTr;
           /* Get Ring Memory Pointer */
          ringPtr = (uint64_t *) ((uint64_t)((uint64_t)ringAccCfgRegs->RING[udmapRegs->txChanCnt + chId].BA_HI << 32) |
                                            ringAccCfgRegs->RING[udmapRegs->txChanCnt + chId].BA_LO);

           trDescriptor = (CSL_UdmapCppi5TRPD *) (*ringPtr);

           numTr = CSL_udmapCppi5GetPktLen(trDescriptor);
           /* Use this field to track the TR, For the target build this would be handled by hardware */
           /* In real hardware this will not be like this it is done just for host emulation*/
           currTr =  druRegs->CHATOMIC[chId].DEBUG[1].NEXT_TR_WORD0_1;

            if ( currTr < numTr)
            {
               currTr++;
               pTr = (CSL_UdmapTR *)( (uint8_t *)trDescriptor + sizeof(CSL_UdmapTR) * currTr);

               /* Update both original and next transfer record by reading the TR from the TR descriptor */
              hostEmulation_druChSubmitAtomicTr(druRegs, chId, (void *) pTr);

               druRegs->CHATOMIC[chId].DEBUG[1].NEXT_TR_WORD0_1 = currTr;

            }
        }
      }
      if ( interimBuffer != NULL)
      {
        free(interimBuffer);
      }
    }
  }
}
#endif


static int32_t DmaUtilsAutoInc3d_getTotalBlockCount(uint8_t * trMem, uint32_t numTr)
{
    uint32_t i;
    CSL_UdmapTR * pTr;
    uint32_t isRingBasedFlowReq = 0;
    uint32_t numTotBlks = 0;
    uint32_t triggerType;
    uint32_t srcCounts;
    uint32_t dstCounts;

    pTr = ( CSL_UdmapTR * )trMem;

    if ( numTr > DMAUTILS_MAX_NUM_TR_DIRECT_TR_MODE)
    {
      isRingBasedFlowReq = 1U;
    }

    if (  isRingBasedFlowReq == 1U )
    {
      /* Setup TR descriptor */
      pTr = (CSL_UdmapTR *)(trMem + sizeof(CSL_UdmapTR));
    }

    for ( i = 0; i < numTr; i++)
    {
        triggerType = CSL_FEXT(pTr[i].flags, UDMAP_TR_FLAGS_TRIGGER0_TYPE );
        if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC )
        {
            srcCounts = pTr[i].icnt1 * pTr[i].icnt2 * pTr[i].icnt3;
            dstCounts = pTr[i].dicnt1 * pTr[i].dicnt2 * pTr[i].dicnt3;
        }
        else if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC )
        {
            srcCounts = pTr[i].icnt2 * pTr[i].icnt3;
            dstCounts = pTr[i].dicnt2 * pTr[i].dicnt3;
        }
        else if ( triggerType == CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT3_DEC )
        {
            srcCounts = pTr[i].icnt3;
            dstCounts = pTr[i].dicnt3;
        }
        else
        {
            srcCounts = 1U;
            dstCounts = 1U;
        }

        /* Always pick the minmum of the src and dst count as once any one is exhausted TR is compete */
        numTotBlks += ( srcCounts > dstCounts) ? dstCounts : srcCounts;
    }

    return numTotBlks;

}



static void DmaUtilsAutoInc3d_setupTr(CSL_UdmapTR * tr,
                                                                        DmaUtilsAutoInc3d_TransferProp * transferProp);

static void DmaUtilsAutoInc3d_setupTr(CSL_UdmapTR * tr,
                                                                        DmaUtilsAutoInc3d_TransferProp * transferProp)
{

    uint32_t triggerBoundary;
    uint32_t waitBoundary;
    uint32_t fmtflags = 0;

    switch (transferProp->syncType)
    {
        case DMAUTILSAUTOINC3D_SYNC_1D :
        {
          triggerBoundary = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT1_DEC;
          waitBoundary     = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT1_DEC;
          break;
        }
        case DMAUTILSAUTOINC3D_SYNC_2D :
        {
          triggerBoundary = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC;
          waitBoundary     = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT2_DEC;
          break;
        }
        case DMAUTILSAUTOINC3D_SYNC_3D:
        {
          triggerBoundary = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT3_DEC;
          waitBoundary     = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT3_DEC;
          break;
        }
        case DMAUTILSAUTOINC3D_SYNC_4D:
        {
          triggerBoundary = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL;
          waitBoundary     = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION;
          break;
        }
        default :
        {
          triggerBoundary = CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ICNT2_DEC;
          waitBoundary     = CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT2_DEC;
          break;
        }
    }

    /* Configure circularity parameters if required */
    if ( ( transferProp->circProp.circSize1 != 0 ) ||
            ( transferProp->circProp.circSize2 != 0 ) )
    {
        int32_t CBK0;
        int32_t CBK1;
        uint32_t circSize1 = transferProp->circProp.circSize1;
        uint32_t circSize2 = transferProp->circProp.circSize2;
        uint32_t circDir;

        if ( transferProp->circProp.circDir == DMAUTILSAUTOINC3D_CIRCDIR_SRC )
        {
          circDir = CSL_UDMAP_TR_FMTFLAGS_DIR_SRC_USES_AMODE;
        }
        else
        {
          circDir = CSL_UDMAP_TR_FMTFLAGS_DIR_DST_USES_AMODE;
        }

        CBK0 = log2((double) circSize1) - 9;
        CBK1 = log2((double)circSize2)  - 1U -CBK0;

        if ( CBK1 < 0 )
        {
           CBK1 = 0;
        }

        fmtflags = CSL_FMK(UDMAP_TR_FMTFLAGS_AMODE, CSL_UDMAP_TR_FMTFLAGS_AMODE_CIRCULAR) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_DIR, circDir) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_ELYPE, CSL_UDMAP_TR_FMTFLAGS_ELYPE_1) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_DFMT, CSL_UDMAP_TR_FMTFLAGS_DFMT_NO_CHANGE ) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_SECTR, CSL_UDMAP_TR_FMTFLAGS_SECTR_NONE ) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK0, CBK0 ) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_CBK1, CBK1 ) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM0, transferProp->circProp.addrModeIcnt0) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM1, transferProp->circProp.addrModeIcnt1) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM2, transferProp->circProp.addrModeIcnt2) |
                               CSL_FMK(UDMAP_TR_FMTFLAGS_AMODE_SPECIFIC_AM3, transferProp->circProp.addrModeIcnt3);

    }
     /* Setup TR */
    tr->flags     = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING)             |
               CSL_FMK(UDMAP_TR_FLAGS_STATIC, FALSE)                                           |
               CSL_FMK(UDMAP_TR_FLAGS_EOL, FALSE)                                              |   /* NA */
               CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, waitBoundary)    |
               CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0)          |/*Set the trigger to local trigger*/
               CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, triggerBoundary)      |/* This is to transfer a 2D block for each trigger*/
               CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE)               |
               CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL)      |
               CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0)                                           |   /* This will come back in TR response */
               CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U)                                         |
               CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U)                                         |
               CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);

    tr->addr        = (uintptr_t)transferProp->ioPointers.srcPtr;
    tr->icnt0        = transferProp->transferDim.sicnt0;
    tr->icnt1        = transferProp->transferDim.sicnt1;
    tr->icnt2        = transferProp->transferDim.sicnt2;
    tr->icnt3        = transferProp->transferDim.sicnt3;
    tr->dim1        = transferProp->transferDim.sdim1;
    tr->dim2        = transferProp->transferDim.sdim2;
    tr->dim3        = transferProp->transferDim.sdim3;
    tr->fmtflags    = fmtflags;
    tr->daddr       = (uintptr_t) transferProp->ioPointers.dstPtr;
    tr->dicnt0       = transferProp->transferDim.dicnt0;
    tr->dicnt1       = transferProp->transferDim.dicnt1;
    tr->dicnt2       = transferProp->transferDim.dicnt2;
    tr->dicnt3       = transferProp->transferDim.dicnt3;
    tr->ddim1      =  transferProp->transferDim.ddim1;
    tr->ddim2      =  transferProp->transferDim.ddim2;
    tr->ddim3      =  transferProp->transferDim.ddim3;
}

static void DmaUtilsAutoInc3d_printf(void * autoIncrementContext, int traceLevel, const char *format, ...);
static void DmaUtilsAutoInc3d_printf(void * autoIncrementContext, int traceLevel, const char *format, ...)
{
  DmaUtilsAutoInc3d_Context * dmautilsContext = (DmaUtilsAutoInc3d_Context *)autoIncrementContext;
  va_list args;

  if ( dmautilsContext->initParams.DmaUtilsVprintf != NULL )
  {
      if ( traceLevel < dmautilsContext->initParams.traceLogLevel)
      {
          va_start(args, format);
          dmautilsContext->initParams.DmaUtilsVprintf(format, args);
          va_end(args);
      }
  }
}

static void  DmaUtilsAutoInc3d_initializeContext(void * autoIncrementContext);
static void  DmaUtilsAutoInc3d_initializeContext(void * autoIncrementContext)
{
    DmaUtilsAutoInc3d_Context * autoIncHandle = (DmaUtilsAutoInc3d_Context *)autoIncrementContext;

    memset(autoIncHandle, 0, sizeof(DmaUtilsAutoInc3d_Context));

//:TODO: This needs to be done at appropriate place
#ifdef HOST_EMULATION

#endif
    return ;
}


int32_t DmaUtilsAutoInc3d_getContextSize(int32_t numChannels)
{
    int32_t contextSize = 0;

    contextSize += DMAUTILS_ALIGN_CEIL(sizeof(DmaUtilsAutoInc3d_ChannelContext), 128) * numChannels;

    contextSize += DMAUTILS_ALIGN_CEIL(sizeof(DmaUtilsAutoInc3d_Context), 128);

    return contextSize;
}

static int32_t DmaUtilsAutoInc3d_setupContext(void * autoIncrementContext, DmaUtilsAutoInc3d_InitParam * initParams);
static int32_t DmaUtilsAutoInc3d_setupContext(void * autoIncrementContext, DmaUtilsAutoInc3d_InitParam * initParams)
{
    int32_t     retVal = UDMA_SOK;
    int32_t memLocation = 0;
    uint8_t * memPointer;
    int32_t i;
    DmaUtilsAutoInc3d_Context * dmaUtilsContext;

    memPointer  = (uint8_t *)autoIncrementContext;
    dmaUtilsContext = (DmaUtilsAutoInc3d_Context *)memPointer;
    memLocation += DMAUTILS_ALIGN_CEIL(sizeof(DmaUtilsAutoInc3d_Context), 128);

    dmaUtilsContext->initParams = *initParams;

    for ( i = 0; i < initParams->numChannels; i++)
    {
      dmaUtilsContext->channelContext[i] = (DmaUtilsAutoInc3d_ChannelContext *) (memPointer + memLocation);
      memLocation += DMAUTILS_ALIGN_CEIL(sizeof (DmaUtilsAutoInc3d_ChannelContext), 128);
    }

    if ( memLocation > initParams->contextSize)
    {
        DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, " DmaUtilsAutoInc3d_setupContext : Failed \n");
        retVal = UDMA_EINVALID_PARAMS;
    }

    return retVal;
}

int32_t DmaUtilsAutoInc3d_init(void * autoIncrementContext , DmaUtilsAutoInc3d_InitParam * initParams, DmaUtilsAutoInc3d_ChannelInitParam chInitParams[])
{
  uint32_t size;
  int32_t     retVal = UDMA_SOK;
  int32_t i;
  uint32_t                chType;
  uint32_t                eventId;
  Udma_ChPrms       chPrms;
  Udma_ChUtcPrms  utcPrms;
  DmaUtilsAutoInc3d_Context              * dmautilsContext;
  DmaUtilsAutoInc3d_ChannelContext * channelContext;
  Udma_ChHandle channelHandle;

  if ( initParams == NULL)
  {
    retVal = UDMA_EBADARGS;
    goto Exit;
  }

  if ( autoIncrementContext == NULL)
  {
    retVal = UDMA_EBADARGS;
    goto Exit;
  }

  size = DmaUtilsAutoInc3d_getContextSize(initParams->numChannels);

  if ( size != initParams->contextSize )
  {
    retVal = UDMA_EINVALID_PARAMS;
    goto Exit;
  }
  /* Reset internal variables of autoincrement context */
  DmaUtilsAutoInc3d_initializeContext(autoIncrementContext);

  dmautilsContext = (DmaUtilsAutoInc3d_Context *)autoIncrementContext;

  retVal = DmaUtilsAutoInc3d_setupContext(autoIncrementContext, initParams);
  if   (UDMA_SOK != retVal)
  {
    goto Exit;
  }

  /* Initialize the channel params to default */
   chType = UDMA_CH_TYPE_UTC;
   UdmaChPrms_init(&chPrms, chType);
   chPrms.utcId = UDMA_UTC_ID_MSMC_DRU0;

  UdmaChUtcPrms_init(&utcPrms);

  for ( i = 0; i < initParams->numChannels; i++)
  {
      channelContext = dmautilsContext->channelContext[i];

      if ( chInitParams[i].druOwner == DMAUTILSAUTOINC3D_DRUOWNER_DIRECT_TR )
      {
          chPrms.fqRingPrms.ringMem      = NULL;
          chPrms.cqRingPrms.ringMem     = NULL;
          chPrms.tdCqRingPrms.ringMem = NULL;
          chPrms.fqRingPrms.elemCnt     = 0U;
          chPrms.cqRingPrms.elemCnt     = 0U;
          chPrms.tdCqRingPrms.elemCnt  = 0U;

          utcPrms.druOwner = CSL_DRU_OWNER_DIRECT_TR;
      }
      else
      {
          chPrms.fqRingPrms.ringMem     = &channelContext->ringMem;
          chPrms.cqRingPrms.ringMem    = &channelContext->responseRingMem;
          chPrms.tdCqRingPrms.ringMem = NULL;
          chPrms.fqRingPrms.ringMemSize = sizeof(uint64_t);
          chPrms.cqRingPrms.ringMemSize = sizeof(uint64_t);
          chPrms.tdCqRingPrms.ringMemSize = 0;
          chPrms.fqRingPrms.elemCnt      = 1U;/* We have only one element per ring */
          chPrms.cqRingPrms.elemCnt     = 1U;/* We have only one element per ring */
          chPrms.tdCqRingPrms.elemCnt  = 0U;/* We have only one element per ring */

          utcPrms.druOwner = CSL_DRU_OWNER_UDMAC_TR;
      }

      channelHandle = &(channelContext->chHandle);

      retVal = Udma_chOpen(initParams->udmaDrvHandle, channelHandle, chType, &chPrms);
      if(UDMA_SOK != retVal)
      {
          DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "Udma_chOpen : Failed \n");
         goto Exit;
      }

      /* Config UTC channel */
      utcPrms.druQueueId  =chInitParams[i].dmaQueNo;

      retVal = Udma_chConfigUtc(channelHandle, &utcPrms);
      if(UDMA_SOK != retVal)
      {
          DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "Udma_chConfigUtc : Failed \n");
          goto Exit;
      }

      /* Enable The channel */
      retVal = Udma_chEnable(channelHandle);
      if(UDMA_SOK != retVal)
      {
          DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "Udma_chEnable : Failed \n");
          goto Exit;
      }

      eventId = Udma_chGetNum(channelHandle);

      channelContext->swTriggerPointer = Udma_druGetTriggerRegAddr(channelHandle);
      //:TODO: Currently it is assumed that DRU local events are routed to 32 event of c7x. This needs to be done cleanly
      channelContext->waitWord =  ((uint64_t)1U << (32 + eventId) );
  }

Exit:
    return retVal;
}

int32_t DmaUtilsAutoInc3d_getTrMemReq(int32_t numTRs)
{
    int32_t isRingBasedFlowReq = 0;
    int32_t trMemReq = 0;

    if ( numTRs > DMAUTILS_MAX_NUM_TR_DIRECT_TR_MODE)
    {
      isRingBasedFlowReq = 1;
    }

    if ( isRingBasedFlowReq )
    {
         //:TODO: Check how to make sure align required
         /* This indicates ring accelerator mode and hence will need memory for TR descriptor */
        trMemReq = sizeof(CSL_UdmapCppi5TRPD) + /* Number of Bytes for TR descriptor */
                               sizeof(CSL_UdmapTR) + /* Padding required to bring start of TR request to natural alignment for memory fetch efficiency */
                               numTRs * sizeof(CSL_UdmapTR) + /* Memory for Transfer Records */
                               numTRs * sizeof(uint32_t);/* Memory for Transfer Response Records */
    }
    else
    {
      trMemReq = numTRs * sizeof(CSL_UdmapTR);
    }

    return trMemReq;
}


int32_t DmaUtilsAutoInc3d_prepareTr(DmaUtilsAutoInc3d_TrPrepareParam * trPrepParam ,  DmaUtilsAutoInc3d_TransferProp transferProp[])
{
    int32_t size;
    int32_t     retVal = UDMA_SOK;
    int32_t isRingBasedFlowReq = 0;
    CSL_UdmapTR * pTrArray;
    int32_t i;

    if ( trPrepParam == NULL )
    {
      retVal = UDMA_EBADARGS;
      goto Exit;
    }

    size = DmaUtilsAutoInc3d_getTrMemReq(trPrepParam->numTRs);

    if ( trPrepParam->trMemSize < size )
    {
      retVal = UDMA_EINVALID_PARAMS;
      goto Exit;
    }

    if ( trPrepParam->trMem == NULL )
    {
      retVal = UDMA_EBADARGS;
      goto Exit;
    }

    if ( trPrepParam->numTRs > DMAUTILS_MAX_NUM_TR_DIRECT_TR_MODE)
    {
        isRingBasedFlowReq = 1;
    }

    pTrArray = (CSL_UdmapTR *)trPrepParam->trMem;

    if ( isRingBasedFlowReq == 1 )
    {
      /* This needs to be updated with correct value during configure */
      uint32_t cqRingNum = 0;
      /* Setup TR descriptor */

      CSL_UdmapCppi5TRPD * pTrpd = (CSL_UdmapCppi5TRPD *)trPrepParam->trMem;
      CSL_UdmapTR           *pTr = (CSL_UdmapTR *)(trPrepParam->trMem + sizeof(CSL_UdmapTR));

      UdmaUtils_makeTrpd(pTrpd, UDMA_TR_TYPE_9, trPrepParam->numTRs, cqRingNum);
      pTrArray = pTr;

    }

    for ( i = 0; i < trPrepParam->numTRs ; i++)
    {
          DmaUtilsAutoInc3d_setupTr(&pTrArray[i], &transferProp[i]);
    }


Exit:

    return retVal;

}


int32_t DmaUtilsAutoInc3d_configure(void * autoIncrementContext, int32_t channelId, uint8_t * trMem, int32_t numTr)
{
    int32_t     retVal = UDMA_SOK;
    DmaUtilsAutoInc3d_Context              * dmautilsContext;
    DmaUtilsAutoInc3d_ChannelContext * channelContext;
    uint32_t isRingBasedFlowReq =0;
    Udma_ChHandle channelHandle;

    uint32_t i;

#ifdef HOST_EMULATION
    uint32_t druChannelNum;
#endif
    if ( autoIncrementContext == NULL)
    {
      retVal = UDMA_EBADARGS;
      DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "DmaUtilsAutoInc3d_configure : Failed :autoIncrementContext == NULL \n");
      goto Exit;
    }

    if ( trMem == NULL )
    {
      retVal = UDMA_EBADARGS;
      DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "DmaUtilsAutoInc3d_configure : Failed : trMem == NULL \n");
      goto Exit;
    }

    dmautilsContext = (DmaUtilsAutoInc3d_Context *)autoIncrementContext;

    channelContext = dmautilsContext->channelContext[channelId];
    channelHandle = &channelContext->chHandle;

    if ( numTr > DMAUTILS_MAX_NUM_TR_DIRECT_TR_MODE)
    {
        isRingBasedFlowReq = 1U;
    }

    if ( isRingBasedFlowReq  == 0U)
    {

        CSL_UdmapTR * tr;

        tr = ( CSL_UdmapTR *)trMem;

        /* Submit the TR using atomic write */
        //:TODO: currently host emulation doesn't handle multiple TR in direct TR mode
        for ( i = 0; i < numTr; i++)
        {
#ifndef HOST_EMULATION
              Udma_chDruSubmitTr(channelHandle, tr + i);
#else
              druChannelNum = Udma_chGetNum(channelHandle);
              hostEmulation_druChSubmitAtomicTr(dmautilsContext->initParams.udmaDrvHandle->utcInfo[UDMA_UTC_ID_MSMC_DRU0].druRegs,
                                                                                druChannelNum , (void *)tr);
#endif
        }
    }
    else
    {
      uint32_t cqRingNum = Udma_chGetCqRingNum(channelHandle);

      CSL_UdmapCppi5TRPD * pTrpd = (CSL_UdmapCppi5TRPD *)trMem;

      /* Update the cq ring number as it was not set correctly during descriptor preperation */
      CSL_udmapCppi5SetReturnPolicy(
              pTrpd,
              CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR,
              CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_ENTIRE_PKT,
              CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_NO,
              CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_TAIL,
              cqRingNum);

      Udma_ringQueueRaw(Udma_chGetFqRingHandle(channelHandle),
              (uint64_t)trMem);

#ifdef HOST_EMULATION
      CSL_UdmapTR           *pTr = (CSL_UdmapTR *)(trMem + sizeof(CSL_UdmapTR));

      druChannelNum = (channelHandle->extChNum - channelHandle->utcInfo->startCh);
      hostEmulation_druChSubmitAtomicTr(dmautilsContext->initParams.udmaDrvHandle->utcInfo[UDMA_UTC_ID_MSMC_DRU0].druRegs,
                                                                        druChannelNum,
                                                                        (void *)pTr);

      /* Use this field to track the TR, For the target build this would be handled by hardware */
      /* In real hardware this will not be like this it is done just for host emulation*/
      dmautilsContext->initParams.udmaDrvHandle->utcInfo[UDMA_UTC_ID_MSMC_DRU0].druRegs->CHATOMIC[druChannelNum].DEBUG[1].NEXT_TR_WORD0_1 = 1;

#endif
    }

    dmautilsContext->blkIdx[channelId] = DmaUtilsAutoInc3d_getTotalBlockCount(trMem, numTr);
Exit :

    return retVal;

}


int32_t DmaUtilsAutoInc3d_trigger(void * autoIncrementContext, int32_t channelId)
{
    DmaUtilsAutoInc3d_Context              * dmautilsContext;

    dmautilsContext = (DmaUtilsAutoInc3d_Context *)autoIncrementContext;

    CSL_druChSetGlobalTrigger0Raw(dmautilsContext->channelContext[channelId]->swTriggerPointer);//:TODO: This should be replaced by something else as we are not suppose to directly use these registers
#ifdef HOST_EMULATION
    hostEmulation_updateTriggerCount(dmautilsContext->initParams.udmaDrvHandle,
                                                     dmautilsContext->channelContext[channelId]->swTriggerPointer);
#endif

    dmautilsContext->blkIdx[channelId]--;

    return dmautilsContext->blkIdx[channelId];
}



void  DmaUtilsAutoInc3d_wait(void * autoIncrementContext, int32_t channelId)
{
    DmaUtilsAutoInc3d_Context              * dmautilsContext;

    dmautilsContext = (DmaUtilsAutoInc3d_Context *)autoIncrementContext;

#ifndef HOST_EMULATION
    volatile uint64_t eflRegisterVal;
    uint64_t waitWord;

    waitWord = dmautilsContext->channelContext[channelId]->waitWord;
    eflRegisterVal = __get_indexed(__EFR,0);
    while((eflRegisterVal & waitWord ) != waitWord )
    {
      eflRegisterVal = __get_indexed(__EFR,0);
    }
    __set_indexed(__EFCLR,0, waitWord);
#else
    /* Do the actual Transfer for host emulation*/
    hostEmulation_triggerDMA(dmautilsContext->initParams.udmaDrvHandle);
#endif

    return;
}


int32_t DmaUtilsAutoInc3d_deconfigure(void * autoIncrementContext, int32_t channelId, uint8_t * trMem, int32_t numTr)
{
    int32_t     retVal = UDMA_SOK;
    DmaUtilsAutoInc3d_Context              * dmautilsContext;
    DmaUtilsAutoInc3d_ChannelContext * channelContext;
    uint32_t isRingBasedFlowReq =0;
    Udma_ChHandle channelHandle;
#ifdef HOST_EMULATION
    uint32_t druChannelNum;
#endif
    if ( autoIncrementContext == NULL)
    {
      retVal = UDMA_EBADARGS;
      DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "DmaUtilsAutoInc3d_configure : Failed :autoIncrementContext == NULL \n");
      goto Exit;
    }

    if ( trMem == NULL )
    {
      retVal = UDMA_EBADARGS;
      DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "DmaUtilsAutoInc3d_configure : Failed : trMem == NULL \n");
      goto Exit;
    }

    dmautilsContext = ( DmaUtilsAutoInc3d_Context *)autoIncrementContext;

    channelContext = dmautilsContext->channelContext[channelId];
    channelHandle = &channelContext->chHandle;


    /* disable  The channel */
    if ( numTr > DMAUTILS_MAX_NUM_TR_DIRECT_TR_MODE)
    {
        isRingBasedFlowReq = 1U;
    }

    if ( isRingBasedFlowReq  == 1 )
    {
       uint64_t    pDesc = 0;
      retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(channelHandle), &pDesc);
      if(UDMA_SOK != retVal)
      {
          DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "DmaUtilsAutoInc3d_deconfigure : Failed : Udma_ringDequeueRaw\n");
          retVal = UDMA_EFAIL;
          goto Exit;
      }
    }

Exit:
    return retVal;

}


int32_t DmaUtilsAutoInc3d_deinit(void * autoIncrementContext)
{
    int32_t     retVal = UDMA_SOK;
    DmaUtilsAutoInc3d_Context              * dmautilsContext;
    DmaUtilsAutoInc3d_ChannelContext * channelContext;
    Udma_ChHandle channelHandle;
    int32_t i;
    if ( autoIncrementContext == NULL)
    {
        retVal = UDMA_EBADARGS;
        DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "DmaUtilsAutoInc3d_configure : Failed :autoIncrementContext == NULL \n");
        goto Exit;
    }

    dmautilsContext = ( DmaUtilsAutoInc3d_Context *)autoIncrementContext;
    for ( i = 0; i < dmautilsContext->initParams.numChannels; i++)
     {
         channelContext = dmautilsContext->channelContext[i];
         channelHandle = &(channelContext->chHandle);

#if !HOST_EMULATION
         /* Avoid calling chDisable API for host emulation as it depends on some of the hardware sequence
         which are not emulated in host emulation */
         retVal = Udma_chDisable(channelHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
#endif
         if(UDMA_SOK != retVal)
         {
             DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "DmaUtilsAutoInc3d_deconfigure : Failed : Udma_chDisable\n");
             retVal = UDMA_EFAIL;
             goto Exit;
         }

         retVal = Udma_chClose(channelHandle);
         if(UDMA_SOK != retVal)
         {
             DmaUtilsAutoInc3d_printf(autoIncrementContext, 0, "DmaUtilsAutoInc3d_deinit : Udma_chClose : Failed \n");
            goto Exit;
         }
    }
Exit:

    return retVal;

}

