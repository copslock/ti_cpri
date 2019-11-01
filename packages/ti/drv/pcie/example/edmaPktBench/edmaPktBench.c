
/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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

#include "pcie_sample.h"
#include "edmaPktMsiBench.h"
#include <ti/csl/csl_edma.h>
#include <ti/drv/pcie/src/pcieloc.h> /* for pcie_setbits */
#include <ti/osal/CacheP.h>
#include <ti/osal/HwiP.h>
#include <ti/csl/csl_timer.h>
#include <stdio.h> /* snprintf */

/* Which EDMA to use */
#define EDMAPKT_BASE (CSL_MPU_EDMA_TPCC_REGS)

/* Starting param/channel for each port model */
#define EDMAPKT_FIRST_REAL_PARAM_TX_P1 (16u)
#define EDMAPKT_FIRST_REAL_CH_TX_P1    (0u)
#define EDMAPKT_INTERRUPT_CH_TX_P1     (2u)

#define EDMAPKT_FIRST_REAL_PARAM_TX_P2 (21u)
#define EDMAPKT_FIRST_REAL_CH_TX_P2    (4u)
#define EDMAPKT_INTERRUPT_CH_TX_P2     (6u)

#define EDMAPKT_FIRST_REAL_PARAM_RX_P1 (26u)
#define EDMAPKT_FIRST_REAL_CH_RX_P1    (8u)
#define EDMAPKT_INTERRUPT_CH_RX_P1     (10u)

#define EDMAPKT_FIRST_REAL_PARAM_RX_P2 (31u)
#define EDMAPKT_FIRST_REAL_CH_RX_P2    (12u)
#define EDMAPKT_INTERRUPT_CH_RX_P2     (14u)

/* Number of times to repeat TX cycle */
#define EDMAPKT_NUM_CYCLES          (100u)
#define EDMAPKT_NUM_TX_TXNS (EDMAPKT_NUM_CYCLES)        /* space to record timing of TX cycles */
#define EDMAPKT_NUM_RX_TXNS (EDMAPKT_NUM_CYCLES * 32)   /* space to record timing of RX cycles */

/* Interupt vector numbers */
#define EDMAPKT_EDMA_VECTOR         (102u)
#define EDMAPKT_TIMER_VECTOR        (101u)

/* Set up timer to tick every 64 bytes at 100M.
 * Assuming 8 byte preamble, 12 byte gap, and 64 byte frame = 84 bytes/672 bits.
 * This is 672/100M = 6.72us.
 * With 20M reference clock this is every 134 clocks.
 */
#define EDMAPKT_TIMER_TICK_CLOCKS       (134u)
#define EDMAPKT_TIMER_INITIAL_COUNT     (0xffffffffu - EDMAPKT_TIMER_TICK_CLOCKS)
#define EDMAPKT_TIMER_RLD_COUNT         (0xffffffffu - EDMAPKT_TIMER_TICK_CLOCKS)

/* Virtual params for EDMA for TX pkt stored in RAM */
typedef volatile EDMA3CCPaRAMEntry txRAMParams_t[EDMAPKT_NUM_TX_PARAMS + 1]; /* +1 pads to 64 byte line boundary */
/* Local copies of packets pulled over PCIE */
typedef volatile uint8_t           txLocPktRAM_t[EDMAPKT_TX_PKT_RAM_SIZE];
/* Local copies of packets pushed over PCIE */
typedef volatile uint8_t           rxLocPktRAM_t[EDMAPKT_RX_PKT_RAM_SIZE];
/* Virtual params for EDMA for RX pkt stored in RAM */
typedef volatile EDMA3CCPaRAMEntry rxRAMParams_t[EDMAPKT_NUM_RX_PARAMS + 1]; /* +1 pads to 64 byte line boundary */
/* Local Descriptors containing addresses in txLocPktRAM for tx pkts */
typedef volatile void             *txDestDesc_t[EDMAPKT_NUM_TX_PKTS + 15]; /* +15 pads to 64 byte line boundary */
/* Local Descriptors containing remote destination address in PCIE for rx pkts */
typedef volatile void             *rxDestDesc_t[EDMAPKT_NUM_RX_PKTS + 15]; /* +15 pads to 64 byte line boundary */
/* Local Descriptors containing local source address in rxLocPktRAM for rx pkts */
typedef volatile void             *rxSrcDesc_t[EDMAPKT_NUM_RX_PKTS + 15]; /* +15 pads to 64 byte line boundary */

/* All local buffers aligned and padded for cache and put in sections for placement */
rxRAMParams_t rxRAMParamsP1 __attribute__ ((aligned(64), section(".bss:paramram")));
rxRAMParams_t rxRAMParamsP2 __attribute__ ((aligned(64), section(".bss:paramram")));
txRAMParams_t txRAMParamsP1 __attribute__ ((aligned(64), section(".bss:paramram")));
txRAMParams_t txRAMParamsP2 __attribute__ ((aligned(64), section(".bss:paramram")));
txLocPktRAM_t txPktLocRAMP1 __attribute__ ((aligned(64), section(".bss:pktram")));
txLocPktRAM_t txPktLocRAMP2 __attribute__ ((aligned(64), section(".bss:pktram")));
rxLocPktRAM_t rxPktLocRAMP1 __attribute__ ((aligned(64), section(".bss:pktram")));
rxLocPktRAM_t rxPktLocRAMP2 __attribute__ ((aligned(64), section(".bss:pktram")));
txDestDesc_t  txDestDescP1  __attribute__ ((aligned(64), section(".bss:pktdesc")));
txDestDesc_t  txDestDescP2  __attribute__ ((aligned(64), section(".bss:pktdesc")));
rxDestDesc_t  rxDestDescP1  __attribute__ ((aligned(64), section(".bss:pktdesc")));
rxDestDesc_t  rxDestDescP2  __attribute__ ((aligned(64), section(".bss:pktdesc")));
rxSrcDesc_t   rxSrcDescP1   __attribute__ ((aligned(64), section(".bss:pktdesc")));
rxSrcDesc_t   rxSrcDescP2   __attribute__ ((aligned(64), section(".bss:pktdesc")));

/* Parameters to configure TX chain */
typedef struct txEdmaSetup_s {
  txRAMParams_t *RAMParams;  /* Virtual params for edma stored in RAM */
  txDestDesc_t  *destDesc;   /* local destination descriptors holding destination pkt addresses */
  txLocPktRAM_t *pktLocRAM;  /* Local (dst) packet memory */
  int32_t        realCh;     /* First channel (of 3) for this port */
  int32_t        realParam;  /* First param number (of EDMAPKT_NUM_TX_PARAMS_PER_PKT + EDMAPKT_NUM_TX_PARAMS_LINKAGE) + 1 */
} txEdmaSetup_t;

/* Parameters to configure RX chain (only 1 packet at a time implemented) */
typedef struct rxEdmaSetup_s {
  rxRAMParams_t *RAMParams;  /* Virtual params for edma stored in RAM */
  rxSrcDesc_t   *srcDesc;    /* local source descriptors holding source pkt address */
  rxDestDesc_t  *destDesc;   /* local destination descriptiors holding remote pkt address */
  rxLocPktRAM_t *pktLocRAM;  /* Local (src) packet memory */
  int32_t        realCh;     /* First channel (of 3) for this port */
  int32_t        realParam;  /* First param number (of EDMAPKT_NUM_TX_PARAMS_PER_PKT + EDMAPKT_NUM_TX_PARAMS_LINKAGE) + 1 */
} rxEdmaSetup_t;


/* Memory for timestamp logs */
uint32_t txTime20MHzP1[EDMAPKT_NUM_TX_TXNS];
int32_t txLogIdxP1 = 0;
uint32_t txTime20MHzP2[EDMAPKT_NUM_TX_TXNS];
int32_t txLogIdxP2 = 0;
uint32_t rxTime20MHzP1[EDMAPKT_NUM_RX_TXNS];
int32_t rxLogIdxP1 = 0;
uint32_t rxTime20MHzP2[EDMAPKT_NUM_RX_TXNS];
int32_t rxLogIdxP2 = 0;

/* State variables used internally */
volatile uint32_t ticks = 0;
volatile uint32_t sendphase = 0;
volatile uint32_t triggered = 0;
#define EDMAPKT_ENABLEESR_TX_P1 (1u)
#define EDMAPKT_ENABLEESR_TX_P2 (2u)
#define EDMAPKT_ENABLEESR_RX_P1 (4u)
#define EDMAPKT_ENABLEESR_RX_P2 (8u)
volatile uint32_t enableESR = 0;
volatile uint32_t finishedESR = 0;
volatile uint32_t lateTX = 0;
volatile uint32_t lateRX = 0;
uint32_t txCyclesP1 = 0;  /* Number of TX cycles done on port 1 */
uint32_t txCyclesP2 = 0;  /* Number of TX cycles done on port 2 */
uint32_t rxPktsP1 = 0;  /* Number of RX packets done on port 1 */
uint32_t rxPktsP2 = 0;  /* Number of RX packets done on port 2 */
volatile uint32_t triggerTxTickP1 = 0;
volatile uint32_t triggerTxTickP2 = 0;
volatile uint32_t triggerRxTickP1 = 0;
volatile uint32_t triggerRxTickP2 = 0;

HwiP_Handle timerIsrHnd;
HwiP_Handle edmaIsrHnd;

/* Configures self-priming chain of multiple TX packets with virtual
 * parameters in RAM */
void setupTxDMA (edmaPktBenchTxBuf_t *remBuf, txEdmaSetup_t *tx)
{
  int32_t pkt, paIndx;
  uint32_t opt_interrupt, opt_static_chained_ch1, opt_chained_ch1, opt_static_chained_ch2, opt_static_chained_ch3;
  volatile EDMA3CCPaRAMEntry *pParam;

  /* Set up virtual params - set all 0 */
  memset ((void *)tx->RAMParams, 0, sizeof(*tx->RAMParams));

  /* fix up defaults that aren't 0 outside opt */
  for (paIndx = 0; paIndx < EDMAPKT_NUM_TX_PARAMS; paIndx++)
  {
    pParam = &(*tx->RAMParams)[paIndx];
    pParam->bCnt     = 1;
    pParam->cCnt     = 1;
  }

  /* Setup "descriptor" with real src/dst */
  for (pkt = 0; pkt < EDMAPKT_NUM_TX_PKTS; pkt++)
  {
    /* Source will be something on other side of PCIE */
    /* Normally RC will fill this in, but no real packet traffic so fill it in here */
    remBuf->txSrcDesc[pkt] = &remBuf->buf[pkt*EDMAPKT_TX_PKT_SIZE];
    /* Destination is local memory.  Keeping separate buffers for debug purposes */
    /* This enables double/triple buffer management */
    (*tx->destDesc)[pkt] = &(*tx->pktLocRAM)[pkt*EDMAPKT_TX_PKT_SIZE];
    /* Mark each packet to prove dma completed */
    remBuf->buf[pkt*EDMAPKT_TX_PKT_SIZE] = (uint8_t)pkt + 1u;
  }
  opt_interrupt   = 0;
  opt_chained_ch1 = 0;
  opt_static_chained_ch2 = 0;
  opt_static_chained_ch3 = 0;

  pcie_setbits (opt_interrupt, EDMA_TPCC_OPT_TCINTEN, 1); /* Interrupt when whole transfer is done */
  pcie_setbits (opt_interrupt, EDMA_TPCC_OPT_TCC, tx->realCh + 2); /* interrupt channel 2 */
  pcie_setbits (opt_interrupt, EDMA_TPCC_OPT_STATIC, 1); /* don't wipe this param so it can be reused */

  pcie_setbits (opt_chained_ch1, EDMA_TPCC_OPT_TCCHEN, 1); /* transfer complete chaining */
  pcie_setbits (opt_chained_ch1, EDMA_TPCC_OPT_TCC, tx->realCh + 1); /* chain channel #1 */

  opt_static_chained_ch1 = opt_chained_ch1;
  pcie_setbits (opt_static_chained_ch1, EDMA_TPCC_OPT_STATIC, 1); /* don't wipe this param so it can be reused */

  pcie_setbits (opt_static_chained_ch2, EDMA_TPCC_OPT_TCCHEN, 1); /* transfer complete chaining */
  pcie_setbits (opt_static_chained_ch2, EDMA_TPCC_OPT_TCC, tx->realCh + 2); /* chain channel #2 */
  pcie_setbits (opt_static_chained_ch2, EDMA_TPCC_OPT_STATIC, 1); /* don't wipe this param so it can be reused */

  pcie_setbits (opt_static_chained_ch3, EDMA_TPCC_OPT_TCCHEN, 1); /* transfer complete chaining */
  pcie_setbits (opt_static_chained_ch3, EDMA_TPCC_OPT_TCC, tx->realCh + 3); /* chain channel #3 */
  pcie_setbits (opt_static_chained_ch3, EDMA_TPCC_OPT_STATIC, 1); /* don't wipe this param so it can be reused */

  paIndx = 0;
  pParam = &(*tx->RAMParams)[paIndx];
  pParam->opt      = opt_static_chained_ch1;
  /* Step 0 : Fetch first block of params from RAM to PaRAM */
  pParam->srcAddr  = (uint32_t)&(*tx->RAMParams)[1];
  pParam->destAddr = EDMAPKT_BASE + EDMA_TPCC_OPT(tx->realParam + 1);
  pParam->aCnt     = EDMAPKT_NUM_TX_PARAMS_PER_LINKAGE * sizeof(*pParam);

  /* This transfer links to next param */
  pParam->linkAddr = 0xffff; /* stop and cross trigger channel */
  paIndx++;
  pParam = &(*tx->RAMParams)[paIndx];

  for (pkt = 0; pkt < EDMAPKT_NUM_TX_PKTS; pkt++)
  {
    pParam->opt      = opt_chained_ch1; /* chain to self triggers linked param */
    /* Step 1 : load source addr from desc over pcie to param */
    pParam->srcAddr  = (uint32_t)&remBuf->txSrcDesc[pkt];
    pParam->destAddr = EDMAPKT_BASE + EDMA_TPCC_SRC(tx->realParam + 3);
    pParam->aCnt     = sizeof(pParam->srcAddr);
    /* This transfer links to next param */
    pParam->linkAddr = EDMA_TPCC_OPT(tx->realParam + 2);

    paIndx++;
    pParam = &(*tx->RAMParams)[paIndx];
    pParam->opt      = opt_static_chained_ch2;
    /* Step 2 : load dest addr from local desc to param */
    pParam->srcAddr  = (uint32_t)&((*tx->destDesc)[pkt]);
    pParam->destAddr = EDMAPKT_BASE + EDMA_TPCC_DST(tx->realParam + 3);
    pParam->aCnt     = sizeof(pParam->destAddr);
    pParam->linkAddr = 0xffff; /* Stop and cross trigger channel */

    paIndx++;
    pParam = &(*tx->RAMParams)[paIndx];
    pParam->opt      = opt_static_chained_ch3; /* chain to self triggers linked param */
    /* Step 3 : Copy the buffer */
    pParam->srcAddr  = 0xfee1deadu; /* replaced by step 1 above */
    pParam->destAddr = 0xbabefaceu; /* replaced by step 2 above */
    pParam->aCnt     = EDMAPKT_TX_PKT_SIZE;
    pParam->linkAddr = 0xffff; /* Stop and cross trigger channel */

    if (pkt != (EDMAPKT_NUM_TX_PKTS - 1))
    {
      paIndx++;
      pParam = &(*tx->RAMParams)[paIndx];
      /* Static to avoid clobbering new incoming params */
      pParam->opt      = opt_static_chained_ch1;
      /* Step 4 : Copy the next params from RAM to PaRAM */
      pParam->srcAddr  = (uint32_t)&(*tx->RAMParams)[paIndx + 1];
      pParam->destAddr = EDMAPKT_BASE + EDMA_TPCC_OPT(tx->realParam + 1);
      pParam->aCnt     = EDMAPKT_NUM_TX_PARAMS_PER_LINKAGE * sizeof(*pParam);
      pParam->linkAddr = 0xffff; /* Stop and cross trigger channel */
    }
    else
    {
      /* After buffer copy done, no more packets */
      pParam->opt      = opt_interrupt; /* no chain causes pump to unprime */
    }
    
    paIndx++;
    pParam = &(*tx->RAMParams)[paIndx];
  }
  /* Write back all the above so EDMA can see it */
  CacheP_wb ((const void*)tx->RAMParams, sizeof(*tx->RAMParams));
  CacheP_wb (tx->destDesc, sizeof(*tx->destDesc));
  CacheP_wb (&remBuf->txSrcDesc[0], sizeof(remBuf->txSrcDesc));
}

/* Configures self-priming chain of multiple RX packets with virtual
 * parameters in RAM.  Chain of 1 supported */
void setupRxDMA (edmaPktBenchRxBuf_t *remBuf, rxEdmaSetup_t *rx)
{
  int32_t paIndx;
  uint32_t opt_interrupt, opt_static_chained_ch1, opt_chained_ch1, opt_chained_ch2;
  volatile EDMA3CCPaRAMEntry *pParam;

  /* Set up virtual params - set all 0 */
  memset ((void *)rx->RAMParams, 0, sizeof(*rx->RAMParams));

  /* fix up defaults that aren't 0 outside opt */
  for (paIndx = 0; paIndx < EDMAPKT_NUM_RX_PARAMS; paIndx++)
  {
    pParam = &(*rx->RAMParams)[paIndx];
    pParam->bCnt     = 1;
    pParam->cCnt     = 1;
  }

  /* destination will be something on other side of PCIE - but descriptor
   * (allocation) is done locally */
  (*rx->destDesc)[0] = &remBuf->buf[0];
  /* source is local memory. This enables double/triple buffer management */
  (*rx->srcDesc)[0] = &(*rx->pktLocRAM)[0];
  /* Mark each packet to prove dma completed */
  (*rx->pktLocRAM)[0] = (uint8_t)1u;

  opt_interrupt   = 0;
  opt_chained_ch1 = 0;
  opt_chained_ch2 = 0;

  pcie_setbits (opt_interrupt, EDMA_TPCC_OPT_TCINTEN, 1); /* Interrupt when whole transfer is done */
  pcie_setbits (opt_interrupt, EDMA_TPCC_OPT_TCC, rx->realCh + 2); /* interrupt channel 2 */
  pcie_setbits (opt_chained_ch1, EDMA_TPCC_OPT_TCCHEN, 1); /* transfer complete chaining */
  pcie_setbits (opt_chained_ch1, EDMA_TPCC_OPT_TCC, rx->realCh + 1); /* chain channel #1 */
  opt_static_chained_ch1 = opt_chained_ch1;
  pcie_setbits (opt_static_chained_ch1, EDMA_TPCC_OPT_STATIC, 1); /* don't wipe this param so it can be reused */

  pcie_setbits (opt_chained_ch2, EDMA_TPCC_OPT_TCCHEN, 1); /* transfer complete chaining */
  pcie_setbits (opt_chained_ch2, EDMA_TPCC_OPT_TCC, rx->realCh + 2); /* chain channel #2 */

  paIndx = 0;
  pParam = &(*rx->RAMParams)[paIndx];
  pParam->opt      = opt_static_chained_ch1;
  /* Step 0 : Fetch first block of params from RAM to PaRAM */
  pParam->srcAddr  = (uint32_t)&(*rx->RAMParams)[1];
  pParam->destAddr = EDMAPKT_BASE + EDMA_TPCC_OPT(rx->realParam + 1);
  pParam->aCnt     = 3 * sizeof(*pParam);

  /* This transfer links to next param */
  pParam->linkAddr = 0xffff; /* stop and cross trigger channel */
  paIndx++;
  pParam = &(*rx->RAMParams)[paIndx];

  pParam->opt      = opt_chained_ch1; /* chain to self triggers linked param */
  /* Step 1 : load source addr from desc locally */
  pParam->srcAddr  = (uint32_t)&((*rx->srcDesc)[0]);
  pParam->destAddr = EDMAPKT_BASE + EDMA_TPCC_SRC(rx->realParam + 3);
  pParam->aCnt     = sizeof(pParam->srcAddr);
  /* This transfer links to next param */
  pParam->linkAddr = EDMA_TPCC_OPT(rx->realParam + 2);

  paIndx++;
  pParam = &(*rx->RAMParams)[paIndx];
  pParam->opt      = opt_chained_ch2;
  /* Step 2 : load dest addr from local desc to param */
  pParam->srcAddr  = (uint32_t)&((*rx->destDesc)[0]);
  pParam->destAddr = EDMAPKT_BASE + EDMA_TPCC_DST(rx->realParam + 3);
  pParam->aCnt     = sizeof(pParam->destAddr);
  pParam->linkAddr = 0xffff; /* Stop and cross trigger channel */

  paIndx++;
  pParam = &(*rx->RAMParams)[paIndx];
  pParam->opt      = opt_interrupt; /* no chain causes pump to unprime */
  /* Step 3 : Copy the buffer */
  pParam->srcAddr  = 0xfee1deadu; /* replaced by step 1 above */
  pParam->destAddr = 0xbabefaceu; /* replaced by step 2 above */
  pParam->aCnt     = EDMAPKT_RX_PKT_SIZE;
  pParam->linkAddr = 0xffff;     /* Stop */

    
  /* Write back all the above so EDMA can see it */
  CacheP_wb ((const void*)rx->RAMParams, sizeof(*rx->RAMParams));
  CacheP_wb (rx->destDesc, sizeof(*rx->destDesc));
  CacheP_wb (rx->srcDesc, sizeof(*rx->srcDesc));
  CacheP_wb ((const void*)rx->pktLocRAM, EDMAPKT_RX_PKT_SIZE);
}

/* Timer interupt - triggers all DMAs.  If this moves to devices where
 * EDMA can be triggered by a timer directly it would replace a lot of this */
void timerIsr()
{
  /* Disable the Timer interrupts */
  TIMERIntDisable(SOC_TIMER9_BASE, TIMER_INT_OVF_EN_FLAG);

  /* Clear the status of the interrupt flags */
  TIMERIntStatusClear(SOC_TIMER9_BASE, TIMER_INT_OVF_IT_FLAG);

  ticks++;
  sendphase++;

  if (sendphase < 32) {
    /* TX active, RX active */
    /* fire RX */
    if (triggered)
    {
      if (enableESR & EDMAPKT_ENABLEESR_RX_P1) {
        /* Fire edma */
        EDMA3SetEvt (EDMAPKT_BASE, EDMAPKT_FIRST_REAL_CH_RX_P1 + 0);
        enableESR       &= ~EDMAPKT_ENABLEESR_RX_P1;
        triggerRxTickP1  = ticks;
      }
      else
      {
        if ((finishedESR & EDMAPKT_ENABLEESR_RX_P1) == 0) {
          lateRX++;
        }
      }
      if (enableESR & EDMAPKT_ENABLEESR_RX_P2) {
        /* Fire edma */
        EDMA3SetEvt (EDMAPKT_BASE, EDMAPKT_FIRST_REAL_CH_RX_P2 + 0);
        enableESR       &= ~EDMAPKT_ENABLEESR_RX_P2;
        triggerRxTickP2  = ticks;
      }
      else
      {
        if ((finishedESR & EDMAPKT_ENABLEESR_RX_P2) == 0) {
          lateRX++;
        }
      }
    }
  } else if (sendphase < 37) {
    /* Idle */
    if (triggered)
    {
      triggered = 0;
      if (((finishedESR | enableESR) & (EDMAPKT_ENABLEESR_TX_P1 | EDMAPKT_ENABLEESR_TX_P2)) == 0)
      {
        lateTX++;
      }
    }
  } else {
    sendphase = 0;
    /* Fire TX and RX */
    if (enableESR & EDMAPKT_ENABLEESR_TX_P1) {
      /* Fire virtual channel 0 */
      EDMA3SetEvt (EDMAPKT_BASE, EDMAPKT_FIRST_REAL_CH_TX_P1 + 0);
      enableESR       &= ~EDMAPKT_ENABLEESR_TX_P1;
      triggerTxTickP1  = ticks;
      triggered        = 1;
    }
    if (enableESR & EDMAPKT_ENABLEESR_TX_P2) {
      /* Fire virtual channel 0 */
      EDMA3SetEvt (EDMAPKT_BASE, EDMAPKT_FIRST_REAL_CH_TX_P2 + 0);
      enableESR       &= ~EDMAPKT_ENABLEESR_TX_P2;
      triggerTxTickP2  = ticks;
      triggered        = 1;
    }
    if (enableESR & EDMAPKT_ENABLEESR_RX_P1) {
      /* Fire virtual channel 0 */
      EDMA3SetEvt (EDMAPKT_BASE, EDMAPKT_FIRST_REAL_CH_RX_P1 + 0);
      enableESR       &= ~EDMAPKT_ENABLEESR_RX_P1;
      triggerRxTickP1  = ticks;
      triggered        = 1;
    }
    if (enableESR & EDMAPKT_ENABLEESR_RX_P2) {
      /* Fire virtual channel 0 */
      EDMA3SetEvt (EDMAPKT_BASE, EDMAPKT_FIRST_REAL_CH_RX_P2 + 0);
      enableESR       &= ~EDMAPKT_ENABLEESR_RX_P2;
      triggerRxTickP2  = ticks;
      triggered        = 1;
    }
  }

  /* Enable the Timer interrupts */
  TIMERIntEnable(SOC_TIMER9_BASE, TIMER_INT_OVF_EN_FLAG);
}

/* Setup/enable timer interrupt */
void setupTimer()
{
  HwiP_Params             hwiInputParams;
  HwiP_Fxn                hwiFxn;
  uint32_t                vector = EDMAPKT_TIMER_VECTOR;
  uint32_t                cpuEvent = vector - 37;
  uint32_t                xbarIndex = vector - 37;

  /* Enable timer 9 power/clock */
  HW_WR_REG32(SOC_L4PER_CM_CORE_BASE + CM_L4PER_TIMER9_CLKCTRL, 0x2);

  while ((HW_RD_REG32(SOC_L4PER_CM_CORE_BASE +
                      CM_L4PER_TIMER9_CLKCTRL) & (0x00030000)) != 0x0) ;
 
  /*Reset the timer module */
  TIMERReset(SOC_TIMER9_BASE);

  /* Disable free run in emulation mode */
  TIMEREmuModeConfigure(SOC_TIMER9_BASE, TIMER_FROZEN);

  /* Load the counter with the initial count value */
  TIMERCounterSet(SOC_TIMER9_BASE, EDMAPKT_TIMER_INITIAL_COUNT);

  /* Load the load register with the reload count value */
  TIMERReloadSet(SOC_TIMER9_BASE, EDMAPKT_TIMER_RLD_COUNT);

  /* Configure the Timer for Auto-reload and compare mode */
  TIMERModeConfigure(SOC_TIMER9_BASE, TIMER_AUTORLD_NOCMP_ENABLE);

  /* Configure the posted mode of TIMER */
  TIMERPostedModeConfig(SOC_TIMER9_BASE, TIMER_NONPOSTED);

  /* Configure the read mode of TIMER */
  TIMERReadModeConfig(SOC_TIMER9_BASE, TIMER_READ_MODE_NONPOSTED);

  /* setup interrupt */
  CSL_xbarIrqConfigure(CSL_XBAR_IRQ_CPU_ID_MPU, xbarIndex, CSL_XBAR_TIMER9_IRQ);

  hwiFxn                  = (HwiP_Fxn)timerIsr;
  hwiInputParams.arg      = (uintptr_t)NULL;

  /* Setup Hardware Interrupt Controller */
  hwiInputParams.name = NULL;
  hwiInputParams.priority = 0;
  hwiInputParams.evtId = cpuEvent; /* Event ID not used in GIC */
  hwiInputParams.triggerSensitivity = 0x3; /* interrupt edge triggered */
  timerIsrHnd = HwiP_create(vector, hwiFxn, &hwiInputParams);

  if (timerIsrHnd == NULL)
  {
    PCIE_logPrintf ("FAIL: Failed to install timer isr\n");
  }
  else 
  {
    /* Enable the Timer9 interrupts */
    TIMERIntEnable(SOC_TIMER9_BASE, TIMER_INT_OVF_EN_FLAG);

    /* Start the Timer */
    TIMEREnable(SOC_TIMER9_BASE);
  }
}

/* Cleanup/remove timer interrupt */
void removeTimerIsr (void)
{
  /* Disable the Timer9 interrupts */
  TIMERIntDisable(SOC_TIMER9_BASE, TIMER_INT_OVF_EN_FLAG);

  /* Stop the Timer */
  TIMERDisable(SOC_TIMER9_BASE);

  HwiP_delete (timerIsrHnd);
}

/* Helper to discover if channel is pending interrupt */
uint32_t edmaChIPRBit (uint32_t chnum, uint32_t IPR, uint32_t IPRH)
{
  uint32_t result;
  if (chnum < 32)
  {
    result = (IPR >> chnum) & 1U;
  }
  else 
  {
    result = (IPRH >> (32U - chnum)) & 1U;
  }

  return result;
}

/* Generate log entry */
void edmaPktLogTime (uint32_t timeInTicks, int32_t *logIdxP1, uint32_t *timeBuffer, int32_t maxIdx)
{
  if (*logIdxP1 < maxIdx) {
    timeBuffer[*logIdxP1] = timeInTicks;
    (*logIdxP1)++;
  }  
}

/* EDMA interrupt occurs when a chain of packets completes.  Discovers if it was
 * late, records log buffer of timestamps, and requests timer to retrigger.
 * Timer doesn't self retrigger to avoid problems when DMA is late (missed
 * dma events)
 */
void edmaIsr (void)
{
  uint32_t now = TIMERCounterGet (SOC_TIMER9_BASE);
  uint32_t IPR;
  uint32_t IPRH;
  uint32_t timeInTicks;
  uint32_t deltaTicks;

  timeInTicks = (now - EDMAPKT_TIMER_RLD_COUNT);

  IPR = EDMA3GetIntrStatus(EDMAPKT_BASE);
  IPRH = EDMA3IntrStatusHighGet(EDMAPKT_BASE);

  /* Generate log for each finished tx/rx port model, and retrigger if required */
  if (edmaChIPRBit (EDMAPKT_INTERRUPT_CH_TX_P1, IPR, IPRH))
  {
    /* TX for Port 1 model finished */
    deltaTicks = timeInTicks + (ticks - triggerTxTickP1) * EDMAPKT_TIMER_TICK_CLOCKS;
    edmaPktLogTime (deltaTicks, &txLogIdxP1, &txTime20MHzP1[0], sizeof(txTime20MHzP1)/sizeof(txTime20MHzP1[0]));
    txCyclesP1++;
    if (txCyclesP1 < EDMAPKT_NUM_CYCLES) {
      enableESR   |= EDMAPKT_ENABLEESR_TX_P1;
    } else {
      finishedESR |= EDMAPKT_ENABLEESR_TX_P1;
    }

    EDMA3ClrIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_TX_P1);
  }

  if (edmaChIPRBit (EDMAPKT_INTERRUPT_CH_TX_P2, IPR, IPRH))
  {
    /* TX for Port 2 model finished */
    deltaTicks = timeInTicks + (ticks - triggerTxTickP2) * EDMAPKT_TIMER_TICK_CLOCKS;
    edmaPktLogTime (deltaTicks, &txLogIdxP2, &txTime20MHzP2[0], sizeof(txTime20MHzP2)/sizeof(txTime20MHzP2[0]));
    txCyclesP2++;
    if (txCyclesP2 < EDMAPKT_NUM_CYCLES) {
      enableESR   |= EDMAPKT_ENABLEESR_TX_P2;
    } else {
      finishedESR |= EDMAPKT_ENABLEESR_TX_P2;
    }
    EDMA3ClrIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_TX_P2);
  }

  if (edmaChIPRBit (EDMAPKT_INTERRUPT_CH_RX_P1, IPR, IPRH))
  {
    /* RX for Port 1 model finished */
    deltaTicks = timeInTicks + (ticks - triggerRxTickP1) * EDMAPKT_TIMER_TICK_CLOCKS;
    edmaPktLogTime (deltaTicks, &rxLogIdxP1, &rxTime20MHzP1[0], sizeof(rxTime20MHzP1)/sizeof(rxTime20MHzP1[0]));
    rxPktsP1++;
    if (rxPktsP1 < EDMAPKT_NUM_RX_TXNS) {
      enableESR   |= EDMAPKT_ENABLEESR_RX_P1;
    } else {
      finishedESR |= EDMAPKT_ENABLEESR_RX_P1;
    }
    EDMA3ClrIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_RX_P1);
  }

  if (edmaChIPRBit (EDMAPKT_INTERRUPT_CH_RX_P2, IPR, IPRH))
  {
    /* RX for Port 2 model finished */
    deltaTicks = timeInTicks + (ticks - triggerRxTickP2) * EDMAPKT_TIMER_TICK_CLOCKS;
    edmaPktLogTime (deltaTicks, &rxLogIdxP2, &rxTime20MHzP2[0], sizeof(rxTime20MHzP2)/sizeof(rxTime20MHzP2[0]));
    rxPktsP2++;
    if (rxPktsP2 < EDMAPKT_NUM_RX_TXNS) {
      enableESR   |= EDMAPKT_ENABLEESR_RX_P2;
    } else {
      finishedESR |= EDMAPKT_ENABLEESR_RX_P2;
    }
    EDMA3ClrIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_RX_P2);
  }
}

/* Install DMA done ISR */
void setupDmaIsr (void)
{
  HwiP_Params             hwiInputParams;
  HwiP_Fxn                hwiFxn;
  uint32_t                vector = EDMAPKT_EDMA_VECTOR;
  uint32_t                cpuEvent = vector - 37;
  uint32_t                xbarIndex = vector - 37;

  /* setup interrupt */
  CSL_xbarIrqConfigure(CSL_XBAR_IRQ_CPU_ID_MPU, xbarIndex, CSL_XBAR_EDMA_TPCC_IRQ_REGION0);

  hwiFxn                  = (HwiP_Fxn)edmaIsr;
  hwiInputParams.arg      = (uintptr_t)NULL;

  /* Setup Hardware Interrupt Controller */
  hwiInputParams.name = NULL;
  hwiInputParams.priority = 0;
  hwiInputParams.evtId = cpuEvent; /* Event ID not used in GIC */
  hwiInputParams.triggerSensitivity = 0x3; /* interrupt edge triggered */
  edmaIsrHnd = HwiP_create(vector, hwiFxn, &hwiInputParams);

  if (edmaIsrHnd == NULL)
  {
    PCIE_logPrintf ("FAIL: Failed to install edma isr\n");
  }

  /* Enable interrupts */
  EDMA3EnableEvtIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_TX_P1);
  EDMA3EnableEvtIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_TX_P2);
  EDMA3EnableEvtIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_RX_P1);
  EDMA3EnableEvtIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_RX_P2);
}

/* Cleanup/remove DMA done ISR */
void removeDmaIsr (void)
{
  EDMA3DisableEvtIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_TX_P1);
  EDMA3DisableEvtIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_TX_P2);
  EDMA3DisableEvtIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_RX_P1);
  EDMA3DisableEvtIntr (EDMAPKT_BASE, EDMAPKT_INTERRUPT_CH_RX_P2);

  HwiP_delete (edmaIsrHnd);
}


/* Set up DMA channels to map to tcc/evtq/starting param */
void initChParms (uint32_t realParamNum, EDMA3CCPaRAMEntry *firstRamParam,
                  uint32_t realChNum, uint32_t tccNum, uint32_t evtqNum)
{
  uint32_t chnum;

  /* Prime pump so param used by channel 0 is really in PaRAM */
  EDMA3SetPaRAM (EDMAPKT_BASE, realParamNum, firstRamParam);
  /* Use channel 0 as main trigger to load RAM to PaRAM */
  EDMA3ChannelToParamMap (EDMAPKT_BASE, realChNum + 0, realParamNum + 0);
  /* Channel 1 loads the addresses into PaRAM */
  EDMA3ChannelToParamMap (EDMAPKT_BASE, realChNum + 1, realParamNum + 1);
  /* Channel 2 does the copy then transfers next RAM to PaRAM */
  EDMA3ChannelToParamMap (EDMAPKT_BASE, realChNum + 2, realParamNum + 3);
  /* Channel 3 does the RAM to PaRAM for next packet */
  EDMA3ChannelToParamMap (EDMAPKT_BASE, realChNum + 3, realParamNum + 4);

  for (chnum = realChNum; chnum < (realChNum + 4); chnum++)
  {
    /* Clear old events */
    EDMA3ClrEvt (EDMAPKT_BASE, chnum);

    /* Disconnect system event so hw doesn't cause emda to misfire */
    /* 1 is unused DMA source */
    CSL_xbarDmaConfigure (CSL_XBAR_DMA_CPU_ID_EDMA, 1, chnum + CSL_XBAR_INST_DMA_EDMA_DREQ_0);
           
    /* Setup channel */
    EDMA3RequestChannel(EDMAPKT_BASE, EDMA3_CHANNEL_TYPE_DMA, chnum, tccNum,
                        evtqNum);
  }

  /* Clear the pending bit */
  EDMA3ClrIntr (EDMAPKT_BASE, (realChNum + 2));
}

/* Printf results in microseconds (us) */
void reportStats (uint32_t *buf, int32_t nElem, const char *msg)
{
  uint32_t minTime, maxTime;
  uint64_t sumTime;
  int32_t i;
  float minTimeUS, maxTimeUS, avgTimeUS;
  char printline[80];

  sumTime = minTime = maxTime = buf[0];

  for (i = 1; i < nElem; i++)
  {
    sumTime += buf[i];
    if (minTime > buf[i])
    {
      minTime = buf[i];
    }
    if (maxTime < buf[i])
    {
      maxTime = buf[i];
    }
  }
  minTimeUS = (1.0 * minTime) / 20; /* 20mhz */
  maxTimeUS = (1.0 * maxTime) / 20; /* 20mhz */
  avgTimeUS = (1.0 * sumTime) / nElem / 20; /* 20mhz */

  snprintf (printline, sizeof(printline), 
            "For %s: min/max/avg us = %5.2f/%5.2f/%5.2f\n", 
            msg, minTimeUS, maxTimeUS, avgTimeUS);
  PCIE_logPrintf (printline);
}

/* Burn one interrupt before starting test, to make sure other side is ready */
void msiSync (volatile uint32_t *localMsi)
{
  edmaPktBenchWaitMsi (&ticks, 0xffffffffu);

  /* Send interrupt to RC using CSL to minimize latency */
  edmaPktBenchSendMsi ();

  do {
    /* Cache invalidate the receive buffer */
    CacheP_Inv ((void *)localMsi, sizeof(*localMsi));
  } while (*localMsi != 1);
}

/* Transmits MSI while timer drives EDMA */
uint32_t msiTxBench (volatile uint32_t *localMsi, volatile uint32_t *remoteMsi, uint32_t requestedESR)
{
  uint32_t msiTimeout = 0, waitMsiTimeout = 0;
  uint32_t msiMin = 0, msiMax = 0;
  uint64_t msiSum = 0;
  int32_t msiNum = 0;
  uint32_t expectedMsiTrackVal = 2; /* 1 used by sync */
  char printline[80];

  while (finishedESR != requestedESR)
  {
    uint32_t startMsiTicks = ticks;
    uint32_t startMsiTime;

    if (edmaPktBenchWaitMsi (&ticks, 10u))
    {
      waitMsiTimeout++; /* Previous MSI timed out */
    }

    /* Align with tick to maximize delay */
    while (startMsiTicks == ticks);
    startMsiTicks = ticks;
    startMsiTime = TIMERCounterGet (SOC_TIMER9_BASE) - EDMAPKT_TIMER_RLD_COUNT;

    /* Send interrupt to RC using CSL to minimize latency */
    edmaPktBenchSendMsi ();

    do {
      /* Cache invalidate the receive buffer */
      CacheP_Inv ((void *)localMsi, sizeof(*localMsi));
    } while ((*localMsi != expectedMsiTrackVal) && ((ticks - startMsiTicks) < 1000));

    if ((ticks - startMsiTicks) >= 1000)
    {
      msiTimeout++;
    }
    else
    {
      uint32_t stopMsiTicks = ticks;
      uint32_t stopMsiTime = TIMERCounterGet (SOC_TIMER9_BASE) - EDMAPKT_TIMER_RLD_COUNT;
      uint32_t time20Mhz;

      expectedMsiTrackVal++;

      if (stopMsiTicks != ticks)
      {
        stopMsiTicks = ticks;
        stopMsiTime = TIMERCounterGet (SOC_TIMER9_BASE);
      }

      msiNum++;

      time20Mhz = ((EDMAPKT_TIMER_TICK_CLOCKS * stopMsiTicks) + stopMsiTime) - 
                  ((EDMAPKT_TIMER_TICK_CLOCKS * startMsiTicks) + startMsiTime);

      if (msiNum == 1)
      {
        msiSum = msiMin = msiMax = time20Mhz;
      }
      else
      {
        if (msiMin > time20Mhz) {
          msiMin = time20Mhz;
        }
        if (msiMax < time20Mhz) {
          msiMax = time20Mhz;
        }
        msiSum += time20Mhz;
      }
    }
  }

  /* Tell other side test is done */
  *remoteMsi = 1;

  if (! msiTimeout)
  {
    float msiMinUS = (1.0 * msiMin) / 20;
    float msiMaxUS = (1.0 * msiMax) / 20;
    float msiAvgUS = (1.0 * msiSum / msiNum) / 20;
    snprintf (printline, sizeof(printline), 
              "For %d MSIs: ROUND TRIP min/max/avg us = %5.2f/%5.2f/%5.2f\n", 
              (int)msiNum, msiMinUS, msiMaxUS, msiAvgUS);
    PCIE_logPrintf (printline);
  } 
  else
  {
    PCIE_logPrintf ("FAIL: %d MSI timed out (%d sent)\n", (int)msiTimeout, (int)msiNum);
  }
  
  return msiTimeout;
}

/* Receives MSI */
void msiRxBench (volatile uint32_t *localMsi, volatile uint32_t *remoteMsi, SemaphoreP_Handle *sem)
{
  uint32_t msiNum = 1;
  do {
    /* Wait for an interrupt */
    if (SemaphoreP_TIMEOUT != SemaphoreP_pend (sem, 1000))
    {
      /* Not a timeout - real interrupt */
      /* Tell other side got the interrupt */
      *remoteMsi = msiNum;
      msiNum++;
    }
 
    /* Cache invalidate the receive buffer to check if we should exit */
    CacheP_Inv ((void *)localMsi, sizeof(*localMsi));
  } while (*localMsi == 0);
}

/*
 * This benchmark emulates an ICSS generating packet traffic.  
 *
 * Benchmark runs from EP side.  This test case uses second TI device
 * as RC where DDR or SRAM can be mapped into EP's address space.
 *
 * All packet buffers are in RC memory, as if there is another
 * host using TI device as ICSS enabled NIC card.
 *
 * Data model assumes there is "realtime" interval every other 
 * 125us, and "nonrealtime" interval the alternate 125us.  
 *
 * For realtime, 32 packets are transmited from RC's memory over
 * ethernet (for this example, DMAed into SRAM on EP).  This assumes
 * real NIC would precompute this list before each interval, we
 * just use same 32 packets over and over.  This is done twice,
 * once for each port.
 *
 * For receive, 32 packets are received one-by-one.  This is modeled
 * as one packet recieved every 6.72us and pushed into RC memory (modeled
 * as DMA from SRAM of this device to RC memory) for each port
 * at same time.
 *
 * For NRT traffic model, 100mbit wire rate is assumed.  Thus for each
 * port, send and receive 32 bytes every 3.5us during the nonrealtime
 * window.  (Not implemented)
 *
 * Continuously send MSIs and measure latency to facilite finding
 * worst case.
 */

int32_t PcieEdmaPktBench (volatile uint32_t *localMsi, edmaPktBenchBuf_t *buf, 
                          pcieMode_e mode, SemaphoreP_Handle *msiSem)
{
  txEdmaSetup_t txSetup;
  rxEdmaSetup_t rxSetup;
  uint32_t region = 0;
  uint32_t requestedESR;
  int32_t p1Errors, p2Errors;
  int32_t pkt;
  int32_t retVal = 0;

  /* Let previous printf() settle */
  Task_sleep(1000);

  if (mode == pcie_EP_MODE)
  {
    txSetup.RAMParams = &txRAMParamsP1;
    txSetup.destDesc  = &txDestDescP1;
    txSetup.pktLocRAM = &txPktLocRAMP1;
    txSetup.realCh    = EDMAPKT_FIRST_REAL_CH_TX_P1;
    txSetup.realParam = EDMAPKT_FIRST_REAL_PARAM_TX_P1;
    setupTxDMA(&buf->tx[0], &txSetup);

    txSetup.RAMParams = &txRAMParamsP2;
    txSetup.destDesc  = &txDestDescP2;
    txSetup.pktLocRAM = &txPktLocRAMP2;
    txSetup.realCh    = EDMAPKT_FIRST_REAL_CH_TX_P2;
    txSetup.realParam = EDMAPKT_FIRST_REAL_PARAM_TX_P2;
    setupTxDMA(&buf->tx[1], &txSetup);

    rxSetup.RAMParams = &rxRAMParamsP1;
    rxSetup.destDesc  = &rxDestDescP1;
    rxSetup.srcDesc   = &rxSrcDescP1;
    rxSetup.pktLocRAM = &rxPktLocRAMP1;
    rxSetup.realCh    = EDMAPKT_FIRST_REAL_CH_RX_P1;
    rxSetup.realParam = EDMAPKT_FIRST_REAL_PARAM_RX_P1;
    setupRxDMA(&buf->rx[0], &rxSetup);
  
    rxSetup.RAMParams = &rxRAMParamsP2;
    rxSetup.destDesc  = &rxDestDescP2;
    rxSetup.srcDesc   = &rxSrcDescP2;
    rxSetup.pktLocRAM = &rxPktLocRAMP2;
    rxSetup.realCh    = EDMAPKT_FIRST_REAL_CH_RX_P2;
    rxSetup.realParam = EDMAPKT_FIRST_REAL_PARAM_RX_P2;
    setupRxDMA(&buf->rx[1], &rxSetup);

    /* Setup DMA */
    EDMAsetRegion(region);
    EDMA3Init (EDMAPKT_BASE, 0);

    initChParms (EDMAPKT_FIRST_REAL_PARAM_TX_P1,  (EDMA3CCPaRAMEntry *)&txRAMParamsP1[0], EDMAPKT_FIRST_REAL_CH_TX_P1, 0, 0);
    initChParms (EDMAPKT_FIRST_REAL_PARAM_TX_P2,  (EDMA3CCPaRAMEntry *)&txRAMParamsP2[0], EDMAPKT_FIRST_REAL_CH_TX_P2, 0, 0);
    initChParms (EDMAPKT_FIRST_REAL_PARAM_RX_P1,  (EDMA3CCPaRAMEntry *)&rxRAMParamsP1[0], EDMAPKT_FIRST_REAL_CH_RX_P1, 1, 1);
    initChParms (EDMAPKT_FIRST_REAL_PARAM_RX_P2,  (EDMA3CCPaRAMEntry *)&rxRAMParamsP2[0], EDMAPKT_FIRST_REAL_CH_RX_P2, 1, 1);

    /* Set up interupt for dma done */
    setupDmaIsr();
    /* Timer actually fires the DMA!! */
    setupTimer();

    /* Exchange 1 MSI to sync other side */
    msiSync (localMsi);

    /* Start cycle for both ports */
    requestedESR = (EDMAPKT_ENABLEESR_TX_P1 | EDMAPKT_ENABLEESR_TX_P2 |
                    EDMAPKT_ENABLEESR_RX_P1 | EDMAPKT_ENABLEESR_RX_P2);
    enableESR    = requestedESR;

    /* Wait for EDMA to complete, while running MSI latency benchmark */
    if (msiTxBench(localMsi, &buf->msiTracker[0], requestedESR) > 0)
    {
      retVal = 1; /* msi test failed */
    }

    /* Cache invalidate the receive buffer */
    CacheP_Inv ((void *)&txPktLocRAMP1[0], sizeof(txPktLocRAMP1));
    CacheP_Inv ((void *)&txPktLocRAMP2[0], sizeof(txPktLocRAMP2));

    /* Verify that all the packets were dma'd */
    p1Errors = 0;
    p2Errors = 0;

    for (pkt = 0; pkt < EDMAPKT_NUM_TX_PKTS; pkt++)
    {
      if (txPktLocRAMP1[pkt*EDMAPKT_TX_PKT_SIZE] != ((uint8_t)pkt + 1u))
      {
        p1Errors++;
      }
      if (txPktLocRAMP2[pkt*EDMAPKT_TX_PKT_SIZE] != ((uint8_t)pkt + 1u))
      {
        p2Errors++;
      }
    }

    if (buf->rx[0].buf[0] != ((uint8_t)1u))
    {
      p1Errors++;
    }

    if (buf->rx[1].buf[0] != ((uint8_t)1u))
    {
      p2Errors++;
    }
    
    if ((p1Errors != 0) || (p2Errors != 0))
    {
      PCIE_logPrintf ("FAIL: Discovered %d port1 and %d port2 errors\n", p1Errors, p2Errors);
      retVal = 1;
    }

    if ((lateTX != 0) || (lateRX != 0))
    {
      PCIE_logPrintf ("WARN: %d late transmits and %d late receives\n", lateTX, lateRX);
    }

    reportStats (txTime20MHzP1, txLogIdxP1, "TX 32 pkts port 1");
    reportStats (txTime20MHzP2, txLogIdxP1, "TX 32 pkts port 2");
    reportStats (rxTime20MHzP1, txLogIdxP1, "RX 1 pkt port 1");
    reportStats (rxTime20MHzP2, txLogIdxP1, "RX 1 pkt port 2");

    /* Cleanup */
    removeTimerIsr();
    removeDmaIsr();
    EDMA3Deinit(EDMAPKT_BASE, 0);
  }
  else
  {
    if (msiSem != NULL)
    {
      /* Process MSIs */
      msiRxBench(localMsi, &buf->msiTracker[0], msiSem);
    }
    else
    {
      PCIE_logPrintf ("FAIL: Missing semaphore on RC\n");
      retVal = 1;
    }
  }

  return retVal;
} /* PcieEdmaPktBench */

/* Nothing past this point */
