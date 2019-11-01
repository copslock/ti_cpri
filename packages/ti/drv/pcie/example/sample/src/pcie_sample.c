/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2010-2019
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
 *   @file  pcie_sample.c
 *
 *   @brief   
 *      This is the PCIe example code.   
 *
 */

/** 
 * In the PCIe sample example two EVMs are used to test the PCIe driver. 
 * As described in the following figure, EVM RC is configured as a Root Complex
 * and EVM EP is configured as End Point.
 *
 *         EVM RC                                          EVM EP
 *   ------------------                             -------------------
 *   |                |                             |                 |
 *   |   Root         |          PCIe Link          |  End Point      |
 *   |   Complex      | <-------------------------->|                 |
 *   |                |                             |                 |
 *   ------------------                             -------------------
 *  
 * Once the PCIe link is established, the following sequence of actions will happen:
 *  - EVM RC sends data to EVM EP
 *  - EVM EP waits to receive all the data
 *  - EVM EP sends the data back to EVM RC
 *  - EVM RC waits to receive all the data
 *  - EVM RC verifies if the received data matches the sent data and declares test pass or fail.
 *  - EVM EP sends 10 MSI and 10 INTA's to EVM RC (on certain device and core combinations).
 *
 */


#include "pcie_sample.h"
#include <ti/drv/pcie/soc/pcie_soc.h>

#include <stdint.h>

#ifdef __TI_ARM_V7R4__
#include <ti/sysbios/hal/Cache.h>
#endif
#ifdef __ARM_ARCH_7A__
#include <ti/sysbios/family/arm/a15/Cache.h>
#include <ti/sysbios/family/arm/a15/Mmu.h>
#if defined(SOC_K2G)
#include <ti/csl/cslr_msmc.h>
#define COHERENT  /* Cache ops unnecessary */
#endif
#endif
#ifdef __aarch64__
#define COHERENT  /* Cache ops unnecessary */
#endif
#if defined(SOC_AM574x) || defined(SOC_AM572x) || defined(SOC_AM571x) || \
    defined(SOC_K2G) || defined(SOC_AM65XX) || defined(__ARM_ARCH_7A__)
#include "ti/board/board.h"
#endif

#if !defined(SOC_AM572x) && !defined(SOC_AM571x) && !defined(SOC_AM574x) && !defined(SOC_AM65XX)
#include <ti/csl/csl_bootcfgAux.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/csl_serdes_pcie.h>
#include <ti/csl/csl_pscAux.h>
#define PCIE_REV0_HW
#else
#include "pcie_sample_board.h"
#ifdef SOC_AM65XX
#include <ti/csl/cslr_gic500.h>
#define PCIE_REV2_HW
#else
#define PCIE_REV1_HW
#endif
#endif
#ifdef _TMS320C6X 
#include <ti/csl/csl_cacheAux.h>
#endif
#include <ti/csl/csl_chip.h>
#ifdef SOC_AM65XX
#include <ti/csl/arch/csl_arch.h>
#endif
#ifdef UDMA
#include "pcie_udma.h"
#endif

#if (defined(_TMS320C6X) || defined (__TI_ARM_V7M4__)) || defined (__TI_ARM_V7R4__)
#pragma DATA_SECTION(dstBuf, ".dstBufSec")
/* Cache coherence: Align must be a multiple of cache line size (L2=128 bytes, L1=64 bytes) to operate with cache enabled. */
/* Aligning to 256 bytes because the PCIe inbound offset register masks the last 8bits of the buffer address  */
#pragma DATA_ALIGN(dstBuf, 256) // TI way of aligning
#endif

/* last element in the buffer is a marker that indicates the buffer status: full/empty */
#define PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE 128
#define PCIE_EXAMPLE_UINT32_SIZE           4 /* preprocessor #if requires a real constant, not a sizeof() */

#define PCIE_EXAMPLE_DSTBUF_BYTES ((PCIE_BUFSIZE_APP + 1) * PCIE_EXAMPLE_UINT32_SIZE)
#define PCIE_EXAMPLE_DSTBUF_REM (PCIE_EXAMPLE_DSTBUF_BYTES % PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE)
#define PCIE_EXAMPLE_DSTBUF_PAD (PCIE_EXAMPLE_DSTBUF_REM ? (PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE - PCIE_EXAMPLE_DSTBUF_REM) : 0)
#define PCIE_EDMA_EXAMPLE_DSTBUF_BYTES (PCIE_EXAMPLE_LINE_SIZE * PCIE_EXAMPLE_UINT32_SIZE)
typedef struct dstBuf_s {
  volatile uint32_t buf[PCIE_BUFSIZE_APP + 1];
  /* Cache coherence: Must pad to cache line size in order to enable cacheability */
#if PCIE_EXAMPLE_DSTBUF_PAD
  uint8_t padding[PCIE_EXAMPLE_DSTBUF_PAD];
#endif
#if defined(EDMA) || defined(UDMA)
  volatile uint32_t edma_buf[PCIE_EXAMPLE_LINE_SIZE];
#endif
#ifdef EDMAPKTBENCH
  edmaPktBenchBuf_t edmaPktBenchBuf;
#endif
} dstBuf_t;
dstBuf_t dstBuf
#ifdef __ARM_ARCH_7A__
__attribute__((aligned(256), section(".bss:dstBufSec"))) // GCC way of aligning
#endif
; // for dstBuf

#define PCIE_EXAMPLE_BUF_EMPTY 0
#define PCIE_EXAMPLE_BUF_FULL  1

/* Does not need to be aligned (even for cache) since it is only accessed locally */
uint32_t srcBuf[PCIE_BUFSIZE_APP];

/* Global variable timers for throughput */
uint64_t totalDMATime = 0;

#ifdef EDMA
/* This is the data that will be used as a temporary space holder
 * for the data being transfered using DMA.
 *
 * This is done since EDMA cannot send a specific value or token
 * but instead it can send blocks of data.
 * */
 #ifdef _TMS320C6X 
#pragma DATA_SECTION(dataContainer, ".testData")
#pragma DATA_ALIGN(dataContainer, PCIE_EXAMPLE_LINE_SIZE)
#endif
UInt32 dataContainer[PCIE_EXAMPLE_LINE_SIZE]
#ifdef __ARM_ARCH_7A__
__attribute__((aligned(256))) // GCC way of aligning
#endif
; // for dstBuf
#endif

#ifdef _TMS320C6X 
extern volatile unsigned int cregister TSCL;
#endif

/* Global config variable that controls
   the PCIe mode. It is global so it can be poked
   from CCS. It should be set either to EP or RC. */
pcieMode_e PcieModeGbl = pcie_EP_MODE;      
#if defined(SOC_AM571x)
Board_IDInfo boardId;
Board_STATUS ret;
#endif

#ifndef CSL_PSC_PD_PCIEX
#ifndef CSL_PSC_PD_PCIE
#define CSL_PSC_PD_PCIE CSL_PSC_PD_PCIE_0
#endif
#else
#define CSL_PSC_PD_PCIE CSL_PSC_PD_PCIEX
#endif

#ifndef CSL_PSC_LPSC_PCIEX 
#ifndef CSL_PSC_LPSC_PCIE
#define CSL_PSC_LPSC_PCIE CSL_PSC_LPSC_PCIE_0
#endif
#else
#define CSL_PSC_LPSC_PCIE CSL_PSC_LPSC_PCIEX
#endif
void cache_invalidate (void *ptr, int size)
{
#ifdef _TMS320C6X 
  uint32_t key;
  /* Disable Interrupts */
  key = _disable_interrupts();

  /*  Cleanup the prefetch buffer also. */
  CSL_XMC_invalidatePrefetchBuffer();

  CACHE_invL1d (ptr, size, CACHE_FENCE_WAIT);
  CACHE_invL2  (ptr, size, CACHE_FENCE_WAIT);

  /* Reenable Interrupts. */
  _restore_interrupts(key);
#elif defined(__ARM_ARCH_7A__) || defined(__TI_ARM_V7R4__)
#ifndef COHERENT
  /*  while bios could have been used on c66 that device chose csl */
  Cache_inv (ptr, size, Cache_Type_ALLD, TRUE);
#endif
#else
/* #error dont know how to invalidate the cache */
#endif
}

void cache_writeback (void *ptr, int size)
{
#ifdef _TMS320C6X 
  uint32_t key;
  /* Disable Interrupts */
  key = _disable_interrupts();

  CACHE_wbL1d (ptr, size, CACHE_FENCE_WAIT);
  CACHE_wbL2  (ptr, size, CACHE_FENCE_WAIT);

  /* Reenable Interrupts. */
  _restore_interrupts(key);
#elif defined(__ARM_ARCH_7A__)
#ifndef COHERENT
  /*  while bios could have been used on c66 that device chose csl */
  Cache_wb (ptr, size, Cache_Type_ALLD, TRUE);
#endif
#elif defined(__arch64__) || defined(__TI_ARM_V7R4__)
#ifndef COHERENT
  /*  while bios could have been used on c66 that device chose csl */
  Cache_wb (ptr, size, Cache_Type_ALLD, TRUE);
#endif
  CSL_archMemoryFence();
#else
/* #error dont know how to writeback the cache */
#endif
}

/*****************************************************************************
 * Function: Converts a core local L2 address to a global L2 address 
 *   Input addr:  L2 address to be converted to global.
 *   return:  uint32_t   Global L2 address
 *****************************************************************************/
uint32_t pcieConvert_CoreLocal2GlobalAddr (uint32_t  addr)
{
#ifdef _TMS320C6X

  uint32_t coreNum;

  /* Get the core number. */
  coreNum = CSL_chipReadReg(CSL_CHIP_DNUM); 
  
#if defined(SOC_AM572x) || defined(SOC_AM571x) || defined(SOC_AM574x)
  /* Compute the global address. */
  return ((1 << 30) | (coreNum << 24) | (addr & 0x00ffffff));

#else
  /* Compute the global address. */
  return ((1 << 28) | (coreNum << 24) | (addr & 0x00ffffff));
#endif
#else
  return addr;
#endif
}    

#ifdef PCIE_EXAMPLE_DMA_RC
/*****************************************************************************
 * Pass one "token" by writing to the "write" address then attempting
 * to read the same value back via the "read" address.  
 *
 * The EDMA generates the transactions
 *
 * 0: pass
 * 1: fail
 ****************************************************************************/
int PcieExampleEdmaRC(dstBuf_t *remote_address, 
                      uint32_t value,
                      uint64_t timeLimit,
                      EDMA3_DRV_Handle hEdma) 
{
  int ACount, BCount, CCount;
  int i;
  int fail = 0;
  volatile uint32_t *remoteBuf;
  uint32_t *source;
  unsigned long totalTime=0;
  unsigned long*totalTimePointer;

  for (i = 0; i < PCIE_EXAMPLE_LINE_SIZE - 1U; i++) {
    dataContainer[i] = value;
  }
  dataContainer[PCIE_EXAMPLE_LINE_SIZE - 1U] = PCIE_EXAMPLE_BUF_FULL; 
  /*
   * Setting up EDMA parameters
   *
   * The following expressions take care of overflow
   *
   */
  if (PCIE_EXAMPLE_LINE_SIZE >= 16384) {
    ACount = 16384;
    BCount = PCIE_EXAMPLE_LINE_SIZE*PCIE_EXAMPLE_UINT32_SIZE / 16384;
  } else {
    ACount = PCIE_EDMA_EXAMPLE_DSTBUF_BYTES;
    BCount = 1;
  }
  CCount = 1;
  
  remoteBuf = (remote_address->edma_buf);
  source = (uint32_t *)pcieConvert_CoreLocal2GlobalAddr((uint32_t)dataContainer);
  totalTimePointer=&totalTime;
  *totalTimePointer=0;

  edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) source, (unsigned int*) remoteBuf,
               ACount, BCount, CCount, EDMA3_DRV_SYNC_A,totalTimePointer);

    totalDMATime += *totalTimePointer;
	PCIE_logPrintf("EDMA write %d bytes with %d cycles\n", (PCIE_EXAMPLE_LINE_SIZE*PCIE_EXAMPLE_UINT32_SIZE), (unsigned int)totalDMATime);
  do {
      cache_invalidate ((void *)dstBuf.edma_buf, PCIE_EDMA_EXAMPLE_DSTBUF_BYTES);
  } while(dstBuf.edma_buf[PCIE_EXAMPLE_LINE_SIZE-1U] != PCIE_EXAMPLE_BUF_FULL);    

  /* check all the data */
  for (i=0; i<PCIE_EXAMPLE_LINE_SIZE; i++)
  {
    if(dstBuf.edma_buf[i] != dataContainer[i])
    {
      fail++;
    }
  }

  *totalTimePointer=0;
  totalDMATime = 0;

  edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) remoteBuf, (unsigned int*) source,
               ACount, BCount, CCount, EDMA3_DRV_SYNC_A,totalTimePointer);

  totalDMATime += *totalTimePointer;
  PCIE_logPrintf("EDMA read %d bytes with %d cycles\n", (PCIE_EXAMPLE_LINE_SIZE*PCIE_EXAMPLE_UINT32_SIZE), (unsigned int)totalDMATime);
  
  return fail;
}
#endif /* PCIE_EXAMPLE_DMA_RC */

#ifdef PCIE_EXAMPLE_DMA_EP
/*****************************************************************************
 * Write a block of data to remote side then verify it
 *
 * The EDMA generates the transactions
 *
 * 0: pass
 * 1: fail
 ****************************************************************************/
int PcieExampleEdmaEP(dstBuf_t *remote_address, 
                      uint32_t value,
                      uint64_t timeLimit,
                      EDMA3_DRV_Handle hEdma) 
{
  int ACount, BCount, CCount;
  int fail = 0;
  volatile uint32_t *remoteBuf;
  volatile uint32_t *source;
  unsigned long totalTime=0;
  unsigned long*totalTimePointer;
  
  do {
      cache_invalidate ((void *)dstBuf.edma_buf, PCIE_EDMA_EXAMPLE_DSTBUF_BYTES);
  } while(dstBuf.edma_buf[PCIE_EXAMPLE_LINE_SIZE-1U] != PCIE_EXAMPLE_BUF_FULL);    

  /*
   * Setting up EDMA parameters
   * The following expressions take care of overflow
   */
  if (PCIE_EXAMPLE_LINE_SIZE >= 16384) {
    ACount = 16384;
    BCount = PCIE_EXAMPLE_LINE_SIZE*PCIE_EXAMPLE_UINT32_SIZE / 16384;
  } else {
    ACount = PCIE_EDMA_EXAMPLE_DSTBUF_BYTES;
    BCount = 1;
  }
  CCount = 1;
  
  remoteBuf = remote_address->edma_buf;
  source = (volatile uint32_t *)pcieConvert_CoreLocal2GlobalAddr((uint32_t)dstBuf.edma_buf);
  totalTimePointer=&totalTime;
  *totalTimePointer=0;

  edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) source, (unsigned int*) remoteBuf,
               ACount, BCount, CCount, EDMA3_DRV_SYNC_A,totalTimePointer);

  totalDMATime += *totalTimePointer;
  return fail;
}
#endif /* PCIE_EXAMPLE_DMA_EP */

/*****************************************************************************
 * Function: Enable/Disable DBI writes
 ****************************************************************************/
pcieRet_e pcieCfgDbi(Pcie_Handle handle, uint8_t enable)
{
  pcieRegisters_t        regs;
  pcieRet_e              retVal;
#if defined(PCIE_REV0_HW) || defined(PCIE_REV2_HW)
  pcieCmdStatusReg_t     cmdStatus;

  memset (&cmdStatus, 0, sizeof(cmdStatus));
  memset (&regs, 0, sizeof(regs));

  regs.cmdStatus = &cmdStatus;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Read CMD STATUS register failed!\n");
    return retVal;
  }
  cmdStatus.dbi = enable;
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET CMD STATUS register failed!\n");
    return retVal;
  }
#else
  pciePlconfDbiRoWrEnReg_t dbiRo;

  memset (&dbiRo, 0, sizeof(dbiRo));
  memset (&regs, 0, sizeof(regs));

  regs.plconfDbiRoWrEn = &dbiRo;

  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET CMD STATUS register failed!\n");
    return retVal;
  }
#endif
  return pcie_RET_OK;
} /* pcieCfgDbi */

/*****************************************************************************
 * Function: Power domain configuration
 ****************************************************************************/
pcieRet_e pciePowerCfg(void)
{
#if !defined(SOC_AM572x) && !defined(SOC_AM571x) && !defined(SOC_AM574x) && !defined(SOC_AM65XX)
  /* Turn on the PCIe power domain */
  if (CSL_PSC_getPowerDomainState (CSL_PSC_PD_PCIE) != PSC_PDSTATE_ON) {
    /* Enable the domain */
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_PCIE);
    /* Enable MDCTL */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_PCIE, PSC_MODSTATE_ENABLE);
    /* Apply the domain */
    CSL_PSC_startStateTransition (CSL_PSC_PD_PCIE);
    /* Wait for it to finish */
    while (! CSL_PSC_isStateTransitionDone (CSL_PSC_PD_PCIE));
  } else {
    PCIE_logPrintf ("Power domain is already enabled.  You probably re-ran without device reset (which is OK)\n");
  }
#endif

  return pcie_RET_OK;
}

/*****************************************************************************
 * Function: Utility function a cycle clock
 ****************************************************************************/
static uint32_t readTime32(void)
{
  uint32_t timeVal;

#if defined (_TMS320C6X)
  timeVal = TSCL;
#elif __ARM_ARCH_7A__
  __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(timeVal));
#else
/* M4 specific implementation*/
  static uint32_t simuTimer = 0;
  simuTimer++;
  timeVal = simuTimer;
#endif
  return timeVal;
}

/*****************************************************************************
 * Function: Utility function to introduce delay 
 ****************************************************************************/
void cycleDelay (uint32_t count)
{
  uint32_t start = (uint32_t)readTime32();

  while (((uint32_t)readTime32() - start) < count);
}

/*****************************************************************************
 * Function: Serdes configuration 
 ****************************************************************************/
pcieRet_e pcieSerdesCfg(void)
{
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(DEVICE_K2L) && \
    !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2L) && !defined(SOC_K2E) && !defined(SOC_K2G) && \
    !defined(SOC_AM572x) && !defined(SOC_AM571x) && !defined(SOC_AM574x) && \
    !defined(SOC_AM65XX)
  uint16_t cfg;

  /* Provide PLL reference clock to SERDES inside PCIESS
     Program PLL settings and enable PLL from PCIe SERDES.*/
  cfg = 0x01C9; /* value based on PCIe userguide */

  CSL_BootCfgSetPCIEConfigPLL(cfg);
#else /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !DEVICE_K2L */
#if defined(SOC_AM572x) || defined(SOC_AM571x) || defined(SOC_AM574x)

  /*Set PCIE_PERSTn to out of reset state*/
  PlatformPCIE_GPIO_Init();
  PlatformPCIE_PERSTn_Reset(0);

  PlatformPCIESS1ClockEnable();
  PlatformPCIESS2ClockEnable();
  PlatformPCIESS1PllConfig();
  PlatformPCIESSSetPhyMode();

  PlatformPCIESS1SetDll();
  PlatformPCIESS2SetDll();

  PlatformPCIESS1CtrlConfig();
  PlatformPCIESS2CtrlConfig();
  PlatformPCIESS1Reset();
  PlatformPCIESS2Reset();
  PlatformPCIESS1PhyConfig();
  PlatformPCIESS2PhyConfig();
#elif defined(SOC_AM65XX)
#ifdef am65xx_idk
  PlatformPCIESSSerdesConfig(0, 0);
  PlatformPCIESSSerdesConfig(1, 0);
#else
  PlatformPCIESSSerdesConfig(1, 1);
#endif
#else
#ifndef  SIMULATOR_SUPPORT

  uint32_t i;

#if defined(DEVICE_K2E) || defined(SOC_K2E)
  /* Configure 2 lanes of serdes with different config */

  CSL_SERDES_RESULT status1, status2;
  CSL_SERDES_LANE_ENABLE_STATUS lane_retval1 = CSL_SERDES_LANE_ENABLE_NO_ERR;
  CSL_SERDES_LANE_ENABLE_STATUS lane_retval2 = CSL_SERDES_LANE_ENABLE_NO_ERR;
  CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params1, serdes_lane_enable_params2;

  memset(&serdes_lane_enable_params1, 0, sizeof(serdes_lane_enable_params1));
  memset(&serdes_lane_enable_params2, 0, sizeof(serdes_lane_enable_params2));

  serdes_lane_enable_params1.base_addr = CSL_PCIE_0_SERDES_CFG_REGS;
  serdes_lane_enable_params1.peripheral_base_addr = CSL_PCIE_0_SLV_CFG_REGS;
  serdes_lane_enable_params1.ref_clock = CSL_SERDES_REF_CLOCK_100M;
  serdes_lane_enable_params1.linkrate = CSL_SERDES_LINK_RATE_5G;
  serdes_lane_enable_params1.num_lanes = 1;
  serdes_lane_enable_params1.phy_type = SERDES_PCIe;
  serdes_lane_enable_params1.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
  serdes_lane_enable_params1.lane_mask = 0x1;
  for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
  {
      serdes_lane_enable_params1.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
      serdes_lane_enable_params1.lane_ctrl_rate[i] = CSL_SERDES_LANE_FULL_RATE; /* GEN2 */
  }

  serdes_lane_enable_params2.base_addr = CSL_PCIE_1_SERDES_CFG_REGS;
  serdes_lane_enable_params2.peripheral_base_addr = CSL_PCIE_1_SLV_CFG_REGS;
  serdes_lane_enable_params2.ref_clock = CSL_SERDES_REF_CLOCK_100M;
  serdes_lane_enable_params2.linkrate = CSL_SERDES_LINK_RATE_5G;
  serdes_lane_enable_params2.num_lanes = 1;
  serdes_lane_enable_params2.phy_type = SERDES_PCIe;
  serdes_lane_enable_params2.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
  serdes_lane_enable_params2.lane_mask = 0x1;
  for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
  {
      serdes_lane_enable_params2.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
      serdes_lane_enable_params2.lane_ctrl_rate[i] = CSL_SERDES_LANE_FULL_RATE; /* GEN2 */
  }

  //SB CMU and COMLANE Setup
  status1 = CSL_PCIeSerdesInit(serdes_lane_enable_params1.base_addr, serdes_lane_enable_params1.ref_clock, serdes_lane_enable_params1.linkrate);

  if (status1 != 0)
    PCIE_logPrintf ("Debug: Invalid PCIE 0 Serdes Init Params\n");


  status2 = CSL_PCIeSerdesInit(serdes_lane_enable_params2.base_addr, serdes_lane_enable_params2.ref_clock, serdes_lane_enable_params2.linkrate);

  if (status2 != 0)
    PCIE_logPrintf ("Debug: Invalid PCIE 1 Serdes Init Params\n");

  /* Common Init Mode */
  /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
  /* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
  serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
  serdes_lane_enable_params1.lane_mask = 0x1;
  lane_retval1 = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);

  /* Lane Init Mode */
  /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
     iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
  /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
  serdes_lane_enable_params1.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
  for(i=0; i< serdes_lane_enable_params1.num_lanes; i++)
  {
      serdes_lane_enable_params1.lane_mask = 1<<i;
      lane_retval1 = CSL_SerdesLaneEnable(&serdes_lane_enable_params1);
  }

  if (lane_retval1 != 0)
  {
      PCIE_logPrintf ("Invalid Serdes Lane Enable Init\n");
      exit(0);
  }

  /* Common Init Mode */
  /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
  /* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
  serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
  serdes_lane_enable_params2.lane_mask = 0x1;
  lane_retval2 = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);

  /* Lane Init Mode */
  /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
     iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
  /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
  serdes_lane_enable_params2.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
  for(i=0; i< serdes_lane_enable_params2.num_lanes; i++)
  {
      serdes_lane_enable_params2.lane_mask = 1<<i;
      lane_retval2 = CSL_SerdesLaneEnable(&serdes_lane_enable_params2);
  }

  if (lane_retval2 != 0)
  {
      PCIE_logPrintf ("Invalid Serdes Lane Enable Init\n");
      exit(0);
  }

  PCIE_logPrintf ("Debug: Serdes Setup Successfully\n");
#else
  /* Configure all lane of serdes with common config */
  CSL_SERDES_RESULT status;
  CSL_SERDES_LANE_ENABLE_STATUS lane_retval = CSL_SERDES_LANE_ENABLE_NO_ERR;
  CSL_SERDES_LANE_ENABLE_PARAMS_T serdes_lane_enable_params;

  memset(&serdes_lane_enable_params, 0, sizeof(serdes_lane_enable_params));

#if defined(DEVICE_K2L) || defined(SOC_K2L)
  /* Check CSISC2_3_MUXSEL bit */
  if (CSL_FEXTR(*(volatile uint32_t *)(CSL_BOOT_CFG_REGS + 0x20), 28, 28) != 1)
  {
      PCIE_logPrintf ("PCIe Serdes Mux Not Selected!\n");
      exit(1);
  }

  serdes_lane_enable_params.base_addr = CSL_CSISC2_3_SERDES_CFG_REGS;
  serdes_lane_enable_params.peripheral_base_addr = CSL_PCIE_0_SLV_CFG_REGS;
#elif defined(SOC_K2G)
   serdes_lane_enable_params.base_addr = CSL_PCIE_0_SERDES_CFG_REGS;
   serdes_lane_enable_params.peripheral_base_addr = CSL_PCIE_0_SLV_CFG_REGS;
#else
   serdes_lane_enable_params.base_addr = CSL_PCIE_SERDES_CFG_REGS;
   serdes_lane_enable_params.peripheral_base_addr = CSL_PCIE_SLV_CFG_REGS;
#endif

   serdes_lane_enable_params.ref_clock = CSL_SERDES_REF_CLOCK_100M;
   serdes_lane_enable_params.linkrate = CSL_SERDES_LINK_RATE_5G;
   serdes_lane_enable_params.num_lanes = 1;
   serdes_lane_enable_params.phy_type = SERDES_PCIe;
   serdes_lane_enable_params.operating_mode = CSL_SERDES_FUNCTIONAL_MODE;
   serdes_lane_enable_params.lane_mask = 0x1;
   for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
   {
       serdes_lane_enable_params.loopback_mode[i] = CSL_SERDES_LOOPBACK_DISABLED;
       serdes_lane_enable_params.lane_ctrl_rate[i] = CSL_SERDES_LANE_FULL_RATE; /* GEN2 */
   }

   status = CSL_PCIeSerdesInit(serdes_lane_enable_params.base_addr,
                               serdes_lane_enable_params.ref_clock,
                               serdes_lane_enable_params.linkrate);

   if (status != 0)
   {
       PCIE_logPrintf ("Invalid Serdes Init Params\n");
   }

   /* Common Init Mode */
   /* Iteration Mode needs to be set to Common Init Mode first with a lane_mask value equal to the total number of lanes being configured */
   /* For example, if there are a total of 2 lanes being configured, lane mask needs to be set to 0x3 */
   serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT;
   serdes_lane_enable_params.lane_mask = 0x1;
   lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);

   /* Lane Init Mode */
   /* Once CSL_SerdesLaneEnable is called with iteration_mode = CSL_SERDES_LANE_ENABLE_COMMON_INIT, the lanes needs to be initialized by setting
      iteration_mode =  CSL_SERDES_LANE_ENABLE_LANE_INIT with the lane_mask equal to the specific lane being configured */
   /* For example, if lane 0 is being configured, lane mask needs to be set to 0x1. if lane 1 is being configured, lane mask needs to be 0x2 etc */
   serdes_lane_enable_params.iteration_mode = CSL_SERDES_LANE_ENABLE_LANE_INIT;
   for(i=0; i< serdes_lane_enable_params.num_lanes; i++)
   {
       serdes_lane_enable_params.lane_mask = 1<<i;
       lane_retval = CSL_SerdesLaneEnable(&serdes_lane_enable_params);
   }

   if (lane_retval != 0)
   {
       PCIE_logPrintf ("Invalid Serdes Lane Enable Init\n");
       exit(0);
   }

   PCIE_logPrintf ("Debug: Serdes Setup Successfully\n");
  #endif
#endif
#endif
#endif
  /*Wait for PLL to lock (3000 CLKIN1 cycles) */
  cycleDelay(10000);
  
  return pcie_RET_OK;
}

/*****************************************************************************
 * Function: Enable/Disable LTSSM (Link Training)
 * This function demonstrates how one can write one binary to use either
 * rev of PCIE
 ****************************************************************************/
pcieRet_e pcieLtssmCtrl(Pcie_Handle handle, uint8_t enable)
{
  pcieCmdStatusReg_t       cmdStatus;
  pcieTiConfDeviceCmdReg_t deviceCmd;
  pcieRegisters_t          regs;
  pcieRet_e retVal;

  memset (&cmdStatus,    0, sizeof(cmdStatus));
  memset (&deviceCmd,    0, sizeof(deviceCmd));
  memset (&regs,         0, sizeof(regs));

  regs.cmdStatus = &cmdStatus;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
  {
    if (retVal == pcie_RET_INV_REG)
    {
      /* The cmdStatus register doesn't exist; try the deviceCmd instead */
      regs.cmdStatus       = NULL;
      regs.tiConfDeviceCmd = &deviceCmd;
      if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
      {
        PCIE_logPrintf ("Read CMD STATUS and DEVICE CMD registers failed!\n");
        return retVal;
      }
    }
    else
    {
      PCIE_logPrintf ("Read CMD STATUS register failed!\n");
      return retVal;
    }
  }

  if(enable)
    deviceCmd.ltssmEn = cmdStatus.ltssmEn = 1;
  else
    deviceCmd.ltssmEn = cmdStatus.ltssmEn = 0;

  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("SET CMD STATUS register failed!\n");
    return retVal;
  }

  return pcie_RET_OK;
}

/*****************************************************************************
 * Function: Configure PCIe in Gen1 vs Gen2 mode
 ****************************************************************************/
pcieRet_e pcieSetGen2(Pcie_Handle handle)
{
  pcieRet_e              retVal;

  pcieRegisters_t        regs;
  pcieLinkCapReg_t       linkCap;
  pcieGen2Reg_t          gen2;

  uint8_t                targetGen, dirSpd;

#if defined(GEN3)
  targetGen = 3;
  dirSpd    = 1;
#elif defined(GEN2)
  targetGen = 2;
  dirSpd    = 1;
#else
  targetGen = 1;
  dirSpd    = 0;
#endif

  memset (&gen2,             0, sizeof(gen2));
  memset (&linkCap,          0, sizeof(linkCap));
  memset (&regs,             0, sizeof(regs));

  /* Set gen1/gen2 in link cap */
  regs.linkCap = &linkCap;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("GET linkCap register failed!\n");
    return retVal;
  }

  if (linkCap.maxLinkSpeed != targetGen)
  {
    PCIE_logPrintf ("PowerUP linkCap gen=%d change to %d\n", linkCap.maxLinkSpeed, targetGen);
    linkCap.maxLinkSpeed = targetGen;
  }
  else
  {
    regs.linkCap = NULL; /* Nothing to write back */
  }

  /* Setting PL_GEN2 */
  gen2.numFts = 0xF;
  gen2.dirSpd = dirSpd;
  gen2.lnEn   = 1;
#ifdef PCIESS1_X2
  gen2.lnEn = 2;
#endif
  regs.gen2 = &gen2;

  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("SET GEN2/link cap register failed!\n");
    return retVal;
  }

  return retVal;
}

/*****************************************************************************
 * Function: Configure PCIe in Root Complex Mode
 ****************************************************************************/
pcieRet_e pcieCfgRC(Pcie_Handle handle)
{
  pcieRet_e retVal;

  pcieObSizeReg_t        obSize;
  pcieType1Bar32bitIdx_t type1Bar32bitIdx;
  pcieStatusCmdReg_t     statusCmd;
  pcieDevStatCtrlReg_t   devStatCtrl;
  pcieAccrReg_t          accr;
                 
  pcieRegisters_t        setRegs;
  pcieRegisters_t        getRegs;

  memset (&obSize,           0, sizeof(obSize));
  memset (&type1Bar32bitIdx, 0, sizeof(type1Bar32bitIdx));
  memset (&statusCmd,        0, sizeof(statusCmd));
  memset (&devStatCtrl,      0, sizeof(devStatCtrl));
  memset (&accr,             0, sizeof(accr));

  /*Disable link training*/
  if ((retVal = pcieLtssmCtrl(handle, FALSE)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Failed to disable Link Training!\n");
    return retVal;
  }
  
  /* Configure the size of the translation regions */   
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));
  
#ifdef PCIE_REV0_HW
  /* Only required for v0 hw */
  obSize.size = pcie_OB_SIZE_8MB;
  setRegs.obSize = &obSize;
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET OB_SIZE register failed!\n");
    return retVal;
  }
#endif

  /* Set gen2/link cap */
  if ((retVal = pcieSetGen2(handle)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("pcieSetGen2 failed!\n");
    return retVal;
  }
  
#if !defined(PCIE_REV2_HW) /* RC does not support BAR */
  /* Configure BAR Masks */   
  /* First need to enable writing on BAR mask registers */
  if ((retVal = pcieCfgDbi (handle, 1)) != pcie_RET_OK)
  {
    return retVal;
  }
  
  /* Configure Masks*/
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));

  type1Bar32bitIdx.reg.reg32 = PCIE_BAR_MASK;
  setRegs.type1BarMask32bitIdx = &type1Bar32bitIdx;

  /* BAR 0 */
  type1Bar32bitIdx.idx = 0; /* configure BAR 0*/
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET BAR MASK register failed!\n");
    return retVal;
  }
  
  /* BAR 1 */
  type1Bar32bitIdx.idx = 1; /* configure BAR 1*/
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET BAR MASK register failed!\n");
    return retVal;
  }
  
  /* Disable writing on BAR Masks */
  if ((retVal = pcieCfgDbi (handle, 0)) != pcie_RET_OK)
  {
    return retVal;
  }
#endif

  /* Enable memory access and mastership of the bus */
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));

  getRegs.statusCmd = &statusCmd;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Read Status Comand register failed!\n");
    return retVal;
  }
  statusCmd.memSp  = 1;
  statusCmd.busMs  = 1;
  statusCmd.resp   = 1;
  statusCmd.serrEn = 1;
  setRegs.statusCmd = &statusCmd;   
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET Status Command register failed!\n");
    return retVal;
  }

  /* Enable Error Reporting */
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));

  getRegs.devStatCtrl = &devStatCtrl;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Regad Device Status Control register failed!\n");
    return retVal;
  }
 
  devStatCtrl.reqRp = 1;
  devStatCtrl.fatalErRp = 1;
  devStatCtrl.nFatalErRp = 1;
  devStatCtrl.corErRp = 1;
  setRegs.devStatCtrl = &devStatCtrl;   
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET Device Status Control register failed!\n");
    return retVal;
  }

#if defined(PCIE_REV0_HW) || defined(PCIE_REV2_HW)
  /* Enable ECRC */
  memset (&setRegs, 0, sizeof(setRegs));
  
  accr.chkEn=1;
  accr.chkCap=1;
  accr.genEn=1;
  accr.genCap=1;
  setRegs.accr = &accr;

  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET ACCR register failed!\n");
    return retVal;
  }
#endif

  return pcie_RET_OK;
}

/*****************************************************************************
 * Function: Configure PCIe in End Point Mode
 ****************************************************************************/
pcieRet_e pcieCfgEP(Pcie_Handle handle)
{
  pcieRet_e retVal;

  pcieObSizeReg_t        obSize;
  pcieGen2Reg_t          gen2;
  pcieType0Bar32bitIdx_t type0Bar32bitIdx;
  pcieStatusCmdReg_t     statusCmd;
  pcieDevStatCtrlReg_t   devStatCtrl;
  pcieAccrReg_t          accr;
                 
  pcieRegisters_t        setRegs;
  pcieRegisters_t        getRegs;

  memset (&obSize,           0, sizeof(obSize));
  memset (&gen2,             0, sizeof(gen2));
  memset (&type0Bar32bitIdx, 0, sizeof(type0Bar32bitIdx));
  memset (&statusCmd,        0, sizeof(statusCmd));
  memset (&devStatCtrl,      0, sizeof(devStatCtrl));
  memset (&accr,             0, sizeof(accr));

  /*Disable link training*/
  if ((retVal = pcieLtssmCtrl(handle, FALSE)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Failed to disable Link Training!\n");
    return retVal;
  }
  
  /* Configure the size of the translation regions */   
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));
  
#ifdef PCIE_REV0_HW
  /* Only required for rev 0 */
  obSize.size = pcie_OB_SIZE_8MB;
  setRegs.obSize = &obSize;
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET OB_SIZE register failed!\n");
    return retVal;
  }
#endif

  /* Set gen2/link cap */
  if ((retVal = pcieSetGen2(handle)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("pcieSetGen2 failed!\n");
    return retVal;
  }
  
  /* Configure BAR Masks */   
  /* First need to enable writing on BAR mask registers */
  if ((retVal = pcieCfgDbi (handle, 1)) != pcie_RET_OK)
  {
    return retVal;
  }

  /* Configure Masks*/
  memset (&getRegs, 0, sizeof(getRegs));
  memset (&setRegs, 0, sizeof(setRegs));
  type0Bar32bitIdx.reg.reg32 = PCIE_BAR_MASK;
  setRegs.type0BarMask32bitIdx = &type0Bar32bitIdx;

  /* BAR 0 */
  type0Bar32bitIdx.idx = 0; /* configure BAR 0*/
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET BAR MASK register failed!\n");
    return retVal;
  }
  
  /* BAR 1 */
  type0Bar32bitIdx.idx = 1; /* configure BAR 1*/
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET BAR MASK register failed!\n");
    return retVal;
  }
  
  /* Disable DBI writes */
  if ((retVal = pcieCfgDbi (handle, 0)) != pcie_RET_OK)
  {
    return retVal;
  }
  
  /* Enable memory access and mastership of the bus */
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));

  getRegs.statusCmd = &statusCmd;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Read Status Comand register failed!\n");
    return retVal;
  }
  statusCmd.memSp  = 1;
  statusCmd.busMs  = 1;
  statusCmd.resp   = 1;
  statusCmd.serrEn = 1;
  setRegs.statusCmd = &statusCmd;   
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET Status Command register failed!\n");
    return retVal;
  }

  /* Enable Error Reporting */
  memset (&setRegs, 0, sizeof(setRegs));
  memset (&getRegs, 0, sizeof(getRegs));

  getRegs.devStatCtrl = &devStatCtrl;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Regad Device Status Control register failed!\n");
    return retVal;
  }
 
  devStatCtrl.reqRp = 1;
  devStatCtrl.fatalErRp = 1;
  devStatCtrl.nFatalErRp = 1;
  devStatCtrl.corErRp = 1;
  setRegs.devStatCtrl = &devStatCtrl;   
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET Device Status Control register failed!\n");
    return retVal;
  }
 
#if defined(PCIE_REV0_HW) || defined(PCIE_REV2_HW)
  /* Enable ECRC */
  memset (&setRegs, 0, sizeof(setRegs));
  
  accr.chkEn=1;
  accr.chkCap=1;
  accr.genEn=1;
  accr.genCap=1;
  setRegs.accr = &accr;

  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET ACCR register failed!\n");
    return retVal;
  }
#endif

  return pcie_RET_OK;
}

#ifdef PCIE_REV0_HW
/*****************************************************************************
 * Function: Configure and enable Outbound Address Translation for rev 0
 ****************************************************************************/
pcieRet_e pcieObTransCfg(Pcie_Handle handle, uint32_t obAddrLo, uint32_t obAddrHi, uint8_t region)
{

  pcieRet_e retVal;

  pcieRegisters_t      setRegs;
  pcieRegisters_t      getRegs;

  pcieCmdStatusReg_t   cmdStatus;

  memset (&setRegs,   0, sizeof(setRegs));
  memset (&getRegs,   0, sizeof(getRegs));
  memset (&cmdStatus, 0, sizeof(cmdStatus));

  /* Set outbound offset registers */
  if ((retVal = Pcie_cfgObOffset(handle, obAddrLo, obAddrHi, region)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Failed to configure ObOffset registers!\n");
    return retVal;
  }

  /*enable Outbound address translation*/
  memset (&setRegs,    0, sizeof(setRegs));
  memset (&getRegs,    0, sizeof(getRegs));

  getRegs.cmdStatus = &cmdStatus;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Read CMD STATUS register failed!\n");
    return retVal;
  }
  cmdStatus.obXltEn = 1;
  setRegs.cmdStatus = &cmdStatus;   
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET CMD STATUS register failed!\n");
    return retVal;
  }

  return pcie_RET_OK;
}


/*****************************************************************************
 * Function: Configure and enable Inbound Address Translation for rev 0
 ****************************************************************************/
pcieRet_e pcieIbTransCfg(Pcie_Handle handle, pcieIbTransCfg_t *ibCfg)
{
  pcieRet_e retVal;

  pcieRegisters_t      setRegs;
  pcieRegisters_t      getRegs;

  pcieCmdStatusReg_t   cmdStatus;

  memset (&setRegs,   0, sizeof(setRegs));
  memset (&getRegs,   0, sizeof(getRegs));
  memset (&cmdStatus, 0, sizeof(cmdStatus));

  /* Set inbound offset registers */
  if ((retVal = Pcie_cfgIbTrans(handle, ibCfg)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Failed to configure Inbound Translation registers!\n");
    return retVal;
  }

  /*enable Inbound address translation*/
  memset (&setRegs,    0, sizeof(setRegs));
  memset (&getRegs,    0, sizeof(getRegs));

  getRegs.cmdStatus = &cmdStatus;
  if ((retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Read CMD STATUS register failed!\n");
    return retVal;
  }
  cmdStatus.ibXltEn = 1;
  setRegs.cmdStatus = &cmdStatus;   
  
  if ((retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &setRegs)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("SET CMD STATUS register failed!\n");
    return retVal;
  }

  return pcie_RET_OK;
}
#else

/*****************************************************************************
 * Function: Configure and enable Outbound Address Translation for rev 1/2
 ****************************************************************************/
pcieRet_e pcieObTransCfg(Pcie_Handle handle, uint32_t obAddrLo, uint32_t obAddrHi, uint8_t region)
{
  pcieAtuRegionParams_t regionParams;
  pcieRet_e             retVal;
  uint32_t              resSize;

  memset (&regionParams, 0, sizeof(regionParams));

  if ((retVal = Pcie_getMemSpaceReserved (handle, &resSize)) != pcie_RET_OK) {
    PCIE_logPrintf ("getMemSpaceReserved failed (%d)\n", (int)retVal);
    return retVal;
  }

  if(PcieModeGbl == pcie_RC_MODE)
  {
    /*Configure OB region for remote configuration access space*/
    regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
    regionParams.tlpType      = PCIE_TLP_TYPE_CFG;
    regionParams.enableRegion = 1;

    regionParams.lowerBaseAddr    = PCIE_WINDOW_CFG_BASE + resSize;
    regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
    regionParams.regionWindowSize = PCIE_WINDOW_CFG_MASK;

    regionParams.lowerTargetAddr = 0U;
    regionParams.upperTargetAddr = 0U;

    if ( (retVal = Pcie_atuRegionConfig(
                     handle,
                     pcie_LOCATION_LOCAL,
                     (uint32_t) 0U,
                     &regionParams)) != pcie_RET_OK) 
    {
      return retVal;
    }
  }
#ifdef PCIE_REV2_HW
  else
  {
    /* Set up OB region for MSI generation on EP */
    /* Configure OB region for MSI generation access space */
    regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
    regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
    regionParams.enableRegion = 1;

    regionParams.lowerBaseAddr    = PCIE_WINDOW_MSI_ADDR + resSize;
    regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
    regionParams.regionWindowSize = PCIE_WINDOW_MSI_MASK;

    regionParams.lowerTargetAddr = PCIE_PCIE_MSI_BASE;
    regionParams.upperTargetAddr = 0U;

    if ( (retVal = Pcie_atuRegionConfig(
                     handle,
                     pcie_LOCATION_LOCAL,
                     (uint32_t) 0U,
                     &regionParams)) != pcie_RET_OK) 
    {
      return retVal;
    }
  }
#endif

  /*Configure OB region for memory transfer*/
  regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
  regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
  regionParams.enableRegion = 1;

  regionParams.lowerBaseAddr    = PCIE_WINDOW_MEM_BASE + resSize;
  regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
  regionParams.regionWindowSize = PCIE_WINDOW_MEM_MASK;

  regionParams.lowerTargetAddr = obAddrLo;
  regionParams.upperTargetAddr = obAddrHi;

  return Pcie_atuRegionConfig(
           handle,
           pcie_LOCATION_LOCAL,
           (uint32_t) 1U,
           &regionParams);
}

/*****************************************************************************
 * Function: Configure and enable Inbound Address Translation for rev 1/2
 ****************************************************************************/
pcieRet_e pcieIbTransCfg(Pcie_Handle handle, pcieIbTransCfg_t *ibCfg)
{
  pcieAtuRegionParams_t regionParams;
  pcieRet_e             retVal = pcie_RET_OK;

  memset (&regionParams, 0, sizeof(regionParams));

  /*Configure IB region for memory transfer*/
  regionParams.regionDir    = PCIE_ATU_REGION_DIR_INBOUND;
  regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
  regionParams.enableRegion = 1;
  regionParams.matchMode    = PCIE_ATU_REGION_MATCH_MODE_ADDR;

  regionParams.lowerBaseAddr    = ibCfg->ibStartAddrLo;
  regionParams.upperBaseAddr    = ibCfg->ibStartAddrHi;
  regionParams.regionWindowSize = PCIE_INBOUND_MASK;

  /* This aligns the buffer to 4K, which needs to be compensated by the application */
  regionParams.lowerTargetAddr = (ibCfg->ibOffsetAddr & ~0xfffU) ;
  regionParams.upperTargetAddr = 0;

  if ( (retVal = Pcie_atuRegionConfig(
                   handle,
                   pcie_LOCATION_LOCAL,
                   (uint32_t) 0U,
                   &regionParams)) != pcie_RET_OK)
  {
    return retVal;
  }

#ifdef PCIE_REV2_HW
  if (PcieModeGbl == pcie_RC_MODE)
  {
    /*Configure IB region for MSI receive */
    regionParams.regionDir    = PCIE_ATU_REGION_DIR_INBOUND;
    regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
    regionParams.enableRegion = 1;
    regionParams.matchMode    = PCIE_ATU_REGION_MATCH_MODE_ADDR;

    regionParams.lowerBaseAddr    = PCIE_PCIE_MSI_BASE;
    regionParams.upperBaseAddr    = 0U;
    regionParams.regionWindowSize = PCIE_WINDOW_MSI_MASK;

    /* Point at GICD_SETSPI_NSR in ARM GIC to directly trigger a SPI */
    regionParams.lowerTargetAddr = (CSL_GIC0_DISTRIBUTOR_BASE + CSL_GIC500_MSG_SETSPI_NSR) - PCIE_PCIE_MSI_OFF;
    regionParams.upperTargetAddr = 0;

    retVal = Pcie_atuRegionConfig(
               handle,
               pcie_LOCATION_LOCAL,
               (uint32_t) 1U,
               &regionParams);
  }
#endif

  return retVal;
}
#endif


/*****************************************************************************
 * Function: Initialize application buffers
 ****************************************************************************/
void pcieInitAppBuf(void)
{
  uint32_t i;
  
  for (i=0; i<PCIE_BUFSIZE_APP; i++)
  {
    dstBuf.buf[i] = 0;
    srcBuf[i] = i;
  }
  
  dstBuf.buf[PCIE_BUFSIZE_APP] = PCIE_EXAMPLE_BUF_EMPTY;
  cache_writeback ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
  
#ifdef EDMA
  for (i = 0; i < PCIE_EXAMPLE_LINE_SIZE - 1U; i++) 
  {
    dstBuf.edma_buf[i] = 0;
  }
  dstBuf.edma_buf[PCIE_EXAMPLE_LINE_SIZE - 1U] = PCIE_EXAMPLE_BUF_EMPTY;
  cache_writeback ((void *)dstBuf.edma_buf, PCIE_EDMA_EXAMPLE_DSTBUF_BYTES);
#endif
}

/*****************************************************************************
 * Function: Check LTSSM status and wait for the link to be up
 ****************************************************************************/
void pcieWaitLinkUp(Pcie_Handle handle)
{
  pcieRegisters_t  getRegs;

  memset (&getRegs, 0, sizeof(getRegs));

#if defined(PCIE_REV0_HW) || defined(PCIE_REV2_HW)
  pcieDebug0Reg_t            ltssmStateReg;
  getRegs.debug0 =          &ltssmStateReg;
#else
  pcieTiConfDeviceCmdReg_t   ltssmStateReg;
  getRegs.tiConfDeviceCmd = &ltssmStateReg;
#endif
  
  memset (&ltssmStateReg,  0, sizeof(ltssmStateReg));
  
  uint8_t ltssmState = 0;
 
  while(ltssmState != pcie_LTSSM_L0)
  {
    cycleDelay(100);
    if (Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &getRegs) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Read LTSSM state failed!\n");
      return;
    }
    ltssmState = ltssmStateReg.ltssmState;
  }
}

pcieRet_e pcieCheckLinkParams(Pcie_Handle handle)
{
#if defined(SOC_K2G) || defined(SOC_AM572x) || defined(SOC_AM571x) || defined(SOC_AM574x)
  pcieRet_e retVal = pcie_RET_OK;
  pcieRegisters_t regs;
  pcieLinkStatCtrlReg_t linkStatCtrl;
  int32_t expLanes = 1, expSpeed = 1;
  const char *pass = "PASS", *fail = "FAIL";
  const char *result = pass;

#ifdef GEN2
  expSpeed = 2;
#endif
#ifdef PCIESS1_X2
  expLanes = 2;
#endif
/* the following code distinguishes between AM571x & AM570x, 
   AM570x is addressed name DRA71x */
#if defined(SOC_AM571x)
  if ((boardId.boardName[0] == 'D') &&
      (boardId.boardName[1] == 'R') &&
      (boardId.boardName[2] == 'A'))
  {
    expLanes = 1;
  }
#endif

  /* Get link status */
  memset (&regs, 0, sizeof(regs));
  regs.linkStatCtrl = &linkStatCtrl;

  PCIE_logPrintf ("Checking link speed and # of lanes\n");
  retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
  if (retVal != pcie_RET_OK) {
     PCIE_logPrintf ("Failed to read linkStatCtrl: %d\n", retVal);
  } else {
    /* Check number of lanes */
    if (expLanes != linkStatCtrl.negotiatedLinkWd) {
       result = fail;
       retVal = pcie_RET_UNSUPPORTED;
    } else {
       result = pass;
    }
    PCIE_logPrintf ("Expect %d lanes, found %d lanes (%s)\n",
                    (int)expLanes, (int)linkStatCtrl.negotiatedLinkWd, result);

    /* Check speed */
    if (expSpeed != linkStatCtrl.linkSpeed) {
       result = fail;
       retVal = pcie_RET_UNSUPPORTED;
    } else {
       result = pass;
    }
    PCIE_logPrintf ("Expect gen %d speed, found gen %d speed (%s)\n",
                    (int)expSpeed, (int)linkStatCtrl.linkSpeed, result);
  }

  return retVal;
#else
  return pcie_RET_OK;
#endif
}


#if defined(PCIE_REV1_HW)
void pcieEpSendInts (Pcie_Handle handle)
{
  pcieRegisters_t             regs;
  pcieTiConfMsiXmtReg_t       msiXmt;
  pcieTiConfIntxAssertReg_t   intxAssert;
  pcieTiConfIntxDeassertReg_t intxDeassert;
  pcieRet_e                   retVal;
  int32_t                     i, intNum = 0;

  PCIE_logPrintf ("EP sending interrupts to RC\n");

  memset (&regs, 0, sizeof(regs));
  memset (&msiXmt, 0, sizeof(msiXmt));

  regs.tiConfMsiXmt = &msiXmt;

  /* Mark that RC received no ISRs */
  dstBuf.buf[PCIE_BUFSIZE_APP] = 0;
  cache_writeback ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);

  for (i = 0; i < PCIE_NUM_INTS; i++)
  {
    /* Read the current value */
    retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't read msi regs in ticonf\n");
      exit(1);
    }
    if (msiXmt.msiReqGrant == 1U)
    {
      PCIE_logPrintf("Can't send MSI: another transmission already pending\n");
      exit(1);
    }
    msiXmt.msiReqGrant = 1U;
    msiXmt.msiFuncNum  = 0U;
    msiXmt.msiVector   = 0U;
    msiXmt.msiTc       = 0;
    /* Send it */
    retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't write msi regs in ticonf\n");
      exit(1);
    }
    /* Wait for it to go away */
    do {
      retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
      if (retVal != pcie_RET_OK)
      {
        PCIE_logPrintf("Can't read msi regs in ticonf\n");
        exit(1);
      }
    } while (msiXmt.msiReqGrant == 1);
    /* Wait for RC to process it */
    do {
      cache_invalidate ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(dstBuf.buf[PCIE_BUFSIZE_APP] == intNum);
    /* Count it */
    intNum++;
  }

  /* Now send legacy interrupts */
  memset (&regs, 0, sizeof(regs));
  memset (&intxAssert, 0, sizeof(intxAssert));
  memset (&intxDeassert, 0, sizeof(intxDeassert));
  for (i = 0; i < PCIE_NUM_INTS; i++)
  {
    /* Read the current value */
    regs.tiConfIntxDeassert = NULL;
    regs.tiConfIntxAssert   = &intxAssert;
    retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't read legacy assert regs in ticonf\n");
      exit(1);
    }
    if (intxAssert.assertF0)
    {
      PCIE_logPrintf("Legacy interrupt already asserted\n");
      exit(1);
    }
    /* Send it */
    intxAssert.assertF0 = 1U;
    retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't write legacy assert regs in ticonf\n");
      exit(1);
    }
    /* read it back */
    retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't read legacy assert regs in ticonf\n");
      exit(1);
    }
    if (! intxAssert.assertF0) {
      PCIE_logPrintf("Legacy interrupt failed to assert\n");
      exit(1);
    }
    /* Deassert it */
    regs.tiConfIntxDeassert = &intxDeassert;
    regs.tiConfIntxAssert   = NULL;
    intxDeassert.deassertF0 = 1U;
    /* Remove it */
    retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't write legacy assert regs in ticonf\n");
      exit(1);
    }
    /* Next iteration will verify it deasserted */
    /* Wait for RC to process it */
    do {
      cache_invalidate ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(dstBuf.buf[PCIE_BUFSIZE_APP] == intNum);
    /* Count it */
    intNum++;
  }
}

#elif defined(PCIE_REV2_HW) && defined(BUILD_MPU)
void pcieEpSendInts (Pcie_Handle handle)
{
  pcieRegisters_t             regs;
  pcieEpIrqSetReg_t           intxAssert;
  pcieEpIrqClrReg_t           intxDeassert;
  pcieRet_e                   retVal;
  int32_t                     i, intNum = 0;

  PCIE_logPrintf ("EP sending interrupts to RC\n");

  /* Mark that RC received no ISRs */
  dstBuf.buf[PCIE_BUFSIZE_APP] = 0;
  cache_writeback ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);

  for (i = 0; i < PCIE_NUM_INTS; i++)
  {
    /* Send it - MSI is just a mem write */
    *(volatile uint32_t *)(PCIE_WINDOW_MSI_ADDR + PCIE_PCIE_MSI_OFF) = PCIE_WINDOW_MSI_DATA;
    /* Wait for RC to process it */
    do {
      cache_invalidate ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(dstBuf.buf[PCIE_BUFSIZE_APP] == intNum);
    /* Count it */
    intNum++;
  }

  /* Now send legacy interrupts */
  memset (&regs, 0, sizeof(regs));
  memset (&intxAssert, 0, sizeof(intxAssert));
  memset (&intxDeassert, 0, sizeof(intxDeassert));
  regs.legacyIrqEnableSet[0] = NULL;

  for (i = 0; i < PCIE_NUM_INTS; i++)
  {
    /* Read the current value */
    regs.epIrqClr = NULL;
    regs.epIrqSet = &intxAssert;
    retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't read legacy assert regs\n");
      exit(1);
    }
    if (intxAssert.epIrqSet)
    {
      PCIE_logPrintf("Legacy interrupt already asserted\n");
      exit(1);
    }
    /* Send it */
    intxAssert.epIrqSet = 1U;
    retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't write legacy assert regs\n");
      exit(1);
    }
    /* read it back */
    retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't read legacy assert regs\n");
      exit(1);
    }
    if (! intxAssert.epIrqSet) {
      PCIE_logPrintf("Legacy interrupt failed to assert\n");
      exit(1);
    }
    /* Deassert it */
    regs.epIrqClr = &intxDeassert;
    regs.epIrqSet = NULL;
    intxDeassert.epIrqClr = 1U;
    /* Remove it */
    retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
      PCIE_logPrintf("Can't write legacy assert regs in ticonf\n");
      exit(1);
    }
    /* Next iteration will verify it deasserted */
    /* Wait for RC to process it */
    do {
      cache_invalidate ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(dstBuf.buf[PCIE_BUFSIZE_APP] == intNum);
    /* Count it */
    intNum++;
  }
}
#endif

#if defined(PCIE_REV1_HW) || (defined(PCIE_REV2_HW) && defined(BUILD_MPU))
void pcieRcWaitInts (Pcie_Handle handle, SemaphoreP_Handle sem, void *pcieBase)
{
  int32_t numInts = 0;

  PCIE_logPrintf ("RC waiting for %d of each of 2 types of interrupts\n", PCIE_NUM_INTS);
  while (numInts < 2*PCIE_NUM_INTS)
  {
    /* Wait on the semaphore */
    SemaphoreP_pend (sem, SemaphoreP_WAIT_FOREVER);
    /* Count it */
    numInts ++;
    /* Tell EP got it */
    *((volatile uint32_t *)pcieBase + PCIE_BUFSIZE_APP) = numInts;
  }
  PCIE_logPrintf ("RC got all %d interrupts\n", (int)numInts);
}
#endif

void pcieSetLanes (Pcie_Handle handle)
{
#ifdef PCIE_REV2_HW
  pcieLnkCtrlReg_t lnkCtrlReg;
  pcieRegisters_t  regs;
  uint8_t          origLanes;

  memset (&regs, 0, sizeof(regs));
  regs.lnkCtrl = &lnkCtrlReg;
  if (Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs) != pcie_RET_OK)
  {
    PCIE_logPrintf ("Read LnkCtrl register failed!\n");
    exit(1);
  }
  origLanes = lnkCtrlReg.lnkMode;
#ifdef am65xx_idk
  lnkCtrlReg.lnkMode = 3; /* bitfield enabling both lanes */
#else
  lnkCtrlReg.lnkMode = 1;
#endif
  if (Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs) != pcie_RET_OK)
  {
    PCIE_logPrintf ("Write LnkCtrl register failed!\n");
    exit(1);
  }
  PCIE_logPrintf ("Set lanes from %d to %d\n", (int)origLanes, (int)lnkCtrlReg.lnkMode);
#endif
}

/*****************************************************************************
 * Function: pcie main task 
 ****************************************************************************/
uint32_t OFFSET = 0;
uint32_t base_address = 0x5500000;
int i;
void pcie (void)
{
#ifdef _TMS320C6X 
  TSCL = 1;
#endif
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(DEVICE_K2L) && \
    !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2L) && !defined(SOC_K2E) && !defined(SOC_K2G) && \
    !defined(SOC_AM572x) && !defined(SOC_AM571x) && !defined(SOC_AM574x) && \
    !defined(SOC_AM65XX)
  uint16_t lock=0;
#endif
  int32_t          deviceNum = 0;
  pcieRet_e        retVal;
  pcieIbTransCfg_t ibCfg;
  pcieBarCfg_t     barCfg;
  Pcie_Handle      handle = NULL;
  void            *pcieBase;
  dstBuf_t        *pciedstBufBase;
  uint32_t    i;
  char             pcieModeResponse;
#if defined(PCIE_REV1_HW) || (defined(PCIE_REV2_HW) && defined(BUILD_MPU))
  SemaphoreP_Handle sem = NULL;
#endif

#ifdef am65xx_evm
  deviceNum = 1; /* The second interface is hooked up on GP EVM */
#endif

  /* Get remote buffer out of cache */
  cache_writeback ((void *)&dstBuf, sizeof(dstBuf));
  /* Unlock kicker once, and don't relock, because its not multicore safe */
#if !defined(SOC_AM572x) && !defined(SOC_AM571x) && !defined(SOC_AM574x) && !defined(SOC_AM65XX)
  CSL_BootCfgUnlockKicker();
#endif

#ifdef EDMA
  EDMA3_DRV_Handle hEdma = NULL;
  hEdma = edmaInit(hEdma);
  if (hEdma==NULL) PCIE_logPrintf("ERROR: EDMA handle not initialized!\n");
#endif

#ifndef IO_CONSOLE
  Console_printf ("IO_CONSOLE not defined.  Most output will go to UART\n");
#endif

  PCIE_logPrintf ("**********************************************\n");
  PCIE_logPrintf ("*             PCIe Test Start                *\n");

#if defined(SOC_K2G) || defined(SOC_AM572x) || defined(SOC_AM571x) || defined(SOC_AM574x) || defined(SOC_AM65XX)
  PCIE_logPrintf ("Enter: E for Endpoint or R for Root Complex   \n");
  PCIE_logScanf  ("%c", &pcieModeResponse);
  if ((pcieModeResponse == 'E') || (pcieModeResponse == 'e'))
  {
    PcieModeGbl = pcie_EP_MODE;
    PCIE_logPrintf ("*                EP mode                     *\n");
  }
  else if ((pcieModeResponse == 'R') || (pcieModeResponse == 'r'))
  {
    PcieModeGbl = pcie_RC_MODE;
    PCIE_logPrintf ("*                RC mode                     *\n");
  }
  else
  {
    PCIE_logPrintf ("Wrong Mode Enter. Please enter E or R \n");
    exit(1);
  }
#endif

  if(PcieModeGbl == pcie_RC_MODE)
    PCIE_logPrintf ("*                RC mode                     *\n");
  else
    PCIE_logPrintf ("*                EP mode                     *\n");
  
  PCIE_logPrintf ("**********************************************\n\n");
  
  PCIE_logPrintf ("Version #: 0x%08x; string %s\n\n", (unsigned)Pcie_getVersion(), Pcie_getVersionStr());
  
  /* Pass device config to LLD */
  if ((retVal = Pcie_init (&pcieInitCfg)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("LLD device configuration failed\n");
    exit(1);
  }

  /* Initialize application buffers */
  pcieInitAppBuf();

  /* Power up PCIe Module */
  if ((retVal = pciePowerCfg()) != pcie_RET_OK) {
    PCIE_logPrintf ("PCIe Power Up failed (%d)\n", (int)retVal);
    exit(1);
  }

  PCIE_logPrintf ("PCIe Power Up.\n");

  if ((retVal = Pcie_open(deviceNum, &handle)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("Open failed (%d)\n", (int)retVal);
    exit(1);
  }

  /* Configure SERDES*/
  if ((retVal = pcieSerdesCfg()) != pcie_RET_OK) {
    PCIE_logPrintf ("PCIe Serdes config failed (%d)\n", (int)retVal);
    exit(1);
  }

  /* Set the PCIe mode*/
  if ((retVal = Pcie_setInterfaceMode(handle, PcieModeGbl)) != pcie_RET_OK) {
    PCIE_logPrintf ("Set PCIe Mode failed (%d)\n", (int)retVal);
    exit(1);
  }
  
  /* Wait until the PCIe SERDES PLL locks */
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H) && !defined(DEVICE_K2E) && !defined(DEVICE_K2L) && \
    !defined(SOC_K2K) && !defined(SOC_K2H) && !defined(SOC_K2L) && !defined(SOC_K2E) && !defined(SOC_K2G) && \
    !defined(SOC_AM572x) && !defined(SOC_AM571x) && !defined(SOC_AM574x) && \
    !defined(SOC_AM65XX)
  while (!lock)
  {
    CSL_BootCfgGetPCIEPLLLock(&lock);
  }                    
#endif /* !DEVICE_K2K && !DEVICE_K2H && !DEVICE_K2E && !DEVICE_K2L && !SOC_K2G && !SOC_AM572x && !SOC_AM571x && !SOC_AM574x */


  PCIE_logPrintf ("PLL configured.\n");

  if(PcieModeGbl == pcie_RC_MODE)
  {
    /* Configure application registers for Root Complex*/
    if ((retVal = pcieCfgRC(handle)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure PCIe in RC mode (%d)\n", (int)retVal);
      exit(1);
    }

#if !defined(PCIE_REV2_HW) /* RC does not support BAR */
    /* Configure Address Translation */
    
    barCfg.location = pcie_LOCATION_LOCAL;
    barCfg.mode     = pcie_RC_MODE;
    barCfg.base     = PCIE_IB_LO_ADDR_RC;
    barCfg.prefetch = pcie_BAR_NON_PREF;
    barCfg.type     = pcie_BAR_TYPE32;
    barCfg.memSpace = pcie_BAR_MEM_MEM;
    barCfg.idx      = PCIE_BAR_IDX_RC;
    
    if ((retVal = Pcie_cfgBar(handle, &barCfg)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure BAR (%d)\n", (int)retVal);
      exit(1);
    }
#endif

    ibCfg.ibBar         = PCIE_BAR_IDX_RC; /* Match BAR that was configured above*/
    ibCfg.ibStartAddrLo = PCIE_IB_LO_ADDR_RC;
    ibCfg.ibStartAddrHi = PCIE_IB_HI_ADDR_RC;
    ibCfg.ibOffsetAddr  = (uint32_t)pcieConvert_CoreLocal2GlobalAddr ((uint32_t)dstBuf.buf);
    ibCfg.region        = PCIE_IB_REGION_RC;       

    if ((retVal = pcieIbTransCfg(handle, &ibCfg)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure Inbound Translation (%d)\n", (int)retVal);
      exit(1);
    }
    else
    {
      PCIE_logPrintf ("Successfully configured Inbound Translation!\n");
    }

    if ((retVal = pcieObTransCfg (handle, PCIE_OB_LO_ADDR_RC, PCIE_OB_HI_ADDR_RC, PCIE_OB_REGION_RC)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure Outbound Address Translation (%d)\n", (int)retVal);
      exit(1);
    }
    else
    {
      PCIE_logPrintf ("Successfully configured Outbound Translation!\n");
    }
  }
  else
  {
    /* Configure application registers for End Point*/
    if ((retVal = pcieCfgEP(handle)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure PCIe in EP mode (%d)\n", (int)retVal);
      exit(1);
    }

    /* Configure Address Translation */
    
    barCfg.location = pcie_LOCATION_LOCAL;
    barCfg.mode     = pcie_EP_MODE;
    barCfg.base     = PCIE_IB_LO_ADDR_EP;
    barCfg.prefetch = pcie_BAR_NON_PREF;
    barCfg.type     = pcie_BAR_TYPE32;
    barCfg.memSpace = pcie_BAR_MEM_MEM;
    barCfg.idx      = PCIE_BAR_IDX_EP;
    
    if ((retVal = Pcie_cfgBar(handle, &barCfg)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure BAR!\n");
      exit(1);
    }

    ibCfg.ibBar         = PCIE_BAR_IDX_EP; /* Match BAR that was configured above*/
    ibCfg.ibStartAddrLo = PCIE_IB_LO_ADDR_EP;
    ibCfg.ibStartAddrHi = PCIE_IB_HI_ADDR_EP;
    ibCfg.ibOffsetAddr  = (uint32_t)pcieConvert_CoreLocal2GlobalAddr ((uint32_t)dstBuf.buf);
    ibCfg.region        = PCIE_IB_REGION_EP;       

    if ((retVal = pcieIbTransCfg(handle, &ibCfg)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure Inbound Translation (%d)!\n", (int)retVal);
      exit(1);
    }
    else
    {
      PCIE_logPrintf ("Successfully configured Inbound Translation!\n");
    }

    if ((retVal = pcieObTransCfg (handle, PCIE_OB_LO_ADDR_EP, PCIE_OB_HI_ADDR_EP, PCIE_OB_REGION_EP)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure Outbound Address Translation(%d)!\n", (int)retVal);
      exit(1);
    }
    else
    {
      PCIE_logPrintf ("Successfully configured Outbound Translation!\n");
    }
  }

  /* Configure/limit number of lanes */
  pcieSetLanes (handle);

  PCIE_logPrintf ("Starting link training...\n");

  /*Enable link training*/
  if ((retVal = pcieLtssmCtrl(handle, TRUE)) != pcie_RET_OK) 
  {
    PCIE_logPrintf ("Failed to Enable Link Training! (%d)\n", (int)retVal);
    exit(1);
  }

  /* Wait for link to be up */
  pcieWaitLinkUp(handle);

  PCIE_logPrintf ("Link is up.\n");
  if ((retVal = pcieCheckLinkParams(handle)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("Link width/speed verification FAILed: %d\n", retVal);
    /* This exit() can be removed if this example is being used as
     * template with non TI card that supports slower or narrower connections
     */
    exit(1);
  }

  if ((retVal = Pcie_getMemSpaceRange (handle, &pcieBase, NULL)) != pcie_RET_OK) {
    PCIE_logPrintf ("getMemSpaceRange failed (%d)\n", (int)retVal);
    exit(1);
  }
  
#if defined(PCIE_REV1_HW)
  /* Adjust PCIE base to point at remote target buffer */
  pcieBase = (char *)pcieBase + 
                     PCIE_WINDOW_MEM_BASE +  /* data area doesn't start at low address */
                     (((uint32_t)&dstBuf) & 0xfff); /* dstBuf needs to be 4K aligned in addr tran */
#elif defined(PCIE_REV2_HW)
  /* Adjust PCIE base to point at remote target buffer */
  pcieBase = (char *) PCIE_WINDOW_MEM_BASE +  /* data area doesn't start at low address */
                     (((uint32_t)&dstBuf) & 0xffff); /* dstBuf needs to be 64K aligned in addr tran */
#else
  /* No adjustment needed */
#endif
  pciedstBufBase = (dstBuf_t *)pcieBase; 

  if(PcieModeGbl == pcie_RC_MODE)
  {
#if defined(PCIE_REV1_HW) || (defined(PCIE_REV2_HW) && defined(BUILD_MPU))
    sem = PlatformSetupMSIAndINTX (handle);
#endif
    /**********************************************************************/
    /* Push a single message to the EP then verify that it is echoed back */
    /**********************************************************************/

    /* Write from RC to EP                                                */
    for (i=0; i<PCIE_BUFSIZE_APP; i++)
    {
      pciedstBufBase->buf[i] = srcBuf[i];
    }
    
    /* Mark that the buffer is full, so EP can process it */
    pciedstBufBase->buf[PCIE_BUFSIZE_APP] = PCIE_EXAMPLE_BUF_FULL;

    /* Note on cache coherence: Write back is not necessary because pcieBase is in
       peripheral address space instead of physical memory*/
    
    /* Data sent to EP.
       RC waits for the loopback to be completed and
       receive data back from EP */

    do {
      cache_invalidate ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(dstBuf.buf[PCIE_BUFSIZE_APP] != PCIE_EXAMPLE_BUF_FULL);


    /* check all the data */
    for (i=0; i<PCIE_BUFSIZE_APP; i++)
    {
      if(dstBuf.buf[i] != srcBuf[i])
      {
        PCIE_logPrintf ("Received data = %u\nTransmited data = %u\nIndex = %u.\n\nTest failed.\n",
                        (unsigned)dstBuf.buf[i], (unsigned)srcBuf[i], (unsigned)i);
        exit(1);
      }
    }
    
    PCIE_logPrintf ("Root Complex received data.\n");
#if defined(PCIE_REV1_HW) || (defined(PCIE_REV2_HW) && defined(BUILD_MPU))
    pcieRcWaitInts (handle, sem, pcieBase);
#endif


  
#ifdef PCIE_EXAMPLE_DMA_RC
    if (PcieExampleEdmaRC(pciedstBufBase, 0xbabeface, 100000,
                          (EDMA3_DRV_Handle)hEdma
                         )) {
PCIE_logPrintf ("Failed to pass token \n");
      exit(1);
    }

    PCIE_logPrintf ("Root Complex DMA received data.\n");
    PCIE_logPrintf ("EDMA Test passed.\n");
    PCIE_logPrintf ("=== this is not an optimized example ===\n");
#endif
  }
  else
  {
    /**********************************************************************/
    /* Wait for a single message from the RC then echo it back            */
    /**********************************************************************/

    /* EP waits for the data received from RC */
    do {
      cache_invalidate ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(dstBuf.buf[PCIE_BUFSIZE_APP] != PCIE_EXAMPLE_BUF_FULL);

    PCIE_logPrintf ("End Point received data.\n");

    /* Loopback to RC what was written in the DST buffer.
       Write from EP to RC */
    for (i=0; i<PCIE_BUFSIZE_APP; i++)
    {
      pciedstBufBase->buf[i] = dstBuf.buf[i];
    }
    
    /* Mark that the buffer is full, so RC can process it */
    pciedstBufBase->buf[PCIE_BUFSIZE_APP] = PCIE_EXAMPLE_BUF_FULL;

    /* Note on cache coherence: Write back is not necessary because pcieBase is in
       peripheral address space instead of physical memory*/

    PCIE_logPrintf ("End Point sent data to Root Complex, completing the loopback.\n");
#if defined(PCIE_REV1_HW) || (defined(PCIE_REV2_HW) && defined(BUILD_MPU))
    /* Send msi to EP */
    pcieEpSendInts (handle);
#endif
    PCIE_logPrintf ("End of Test.\n");

#ifdef PCIE_EXAMPLE_DMA_EP
    if (PcieExampleEdmaEP(pciedstBufBase, 0xbabeface, 100000,
                          (EDMA3_DRV_Handle)hEdma)) 
    {
      PCIE_logPrintf ("Failed to pass token \n");
      exit(1);
    }
    PCIE_logPrintf ("End Point sent data to Root Complex, DMA completing the loopback.\n");
    PCIE_logPrintf ("End of DMA Test.\n");
#endif


  }
#ifdef UDMA
  if (pcieUdmaTest((void *)&pciedstBufBase->edma_buf[0], sizeof(pciedstBufBase->edma_buf)))
  {
    PCIE_logPrintf("UDMA test failed\n");
    exit(1);
  }
#endif

#ifdef EDMA
  edmaDeinit(hEdma);
#endif
#ifdef EDMAPKTBENCH
  if (PcieEdmaPktBench(&dstBuf.edmaPktBenchBuf.msiTracker[0], 
                       &pciedstBufBase->edmaPktBenchBuf, 
                       PcieModeGbl, sem))
  {
    PCIE_logPrintf ("EDMA packet IO benchmark failed to execute correctly\n");
    exit(1);
  }
#endif
  PCIE_logPrintf ("Test passed.\n");
#ifndef IO_CONSOLE
  Console_printf ("Test passed.\n");
#endif

  BIOS_exit(0);

}

int main() {
  Task_Params params;
  Task_Params_init (&params);
  params.stackSize = 36864; //32768;
  Task_create((Task_FuncPtr) pcie, &params, NULL);

#ifdef __ARM_ARCH_7A__
  {
    /* Add MMU entries for MMR's required for PCIE example */
    Mmu_DescriptorAttrs attrs;
    extern char ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_0__A;
    extern char ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_1__A;
    uint32_t addr0 = (uint32_t)&ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_0__A;
    uint32_t addr1 = (uint32_t)&ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_1__A;

    Mmu_initDescAttrs(&attrs);

    attrs.type = Mmu_DescriptorType_TABLE;
    attrs.shareable = 0;            // non-shareable
    attrs.accPerm = 1;              // read/write at any privelege level
    attrs.attrIndx = 0;             // Use MAIR0 Register Byte 3 for
                                    // determining the memory attributes
                                    // for each MMU entry


    // Update the first level table's MMU entry for 0x00000000 with the
    // new attributes.
    Mmu_setFirstLevelDesc((Ptr)0x00000000, (UInt64)addr0, &attrs);
    // Update the first level table's MMU entry for 0x40000000 with the
    // new attributes.
    Mmu_setFirstLevelDesc((Ptr)0x40000000, (UInt64)addr1, &attrs);

#if defined(SOC_K2G)
    {
      CSL_MsmcRegs *msmc = (CSL_MsmcRegs *)CSL_MSMC_CFG_REGS;
      int32_t index, privid;
      // Set up SES & SMS to make all masters coherent
      for (privid = 0; privid < 16; privid++)
      {
        for (index = 0; index < 8; index++)
        {
          uint32_t ses_mpaxh = msmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH;
          uint32_t sms_mpaxh = msmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH;
          if (CSL_FEXT (ses_mpaxh, MSMC_SES_MPAXH_0_SEGSZ) != 0)
          {
            // Clear the "US" bit to make coherent.  This is at 0x80.
            ses_mpaxh &= ~0x80;
            msmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH = ses_mpaxh;
          }
          if (CSL_FEXT (sms_mpaxh, MSMC_SMS_MPAXH_0_SEGSZ) != 0)
          {
            // Clear the "US" bit to make coherent.  This is at 0x80.
            sms_mpaxh &= ~0x80;
            msmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH = sms_mpaxh;
          }
        }
      }
    }
#endif
  }
#endif

#if defined(SOC_AM574x) || defined(SOC_AM572x) || defined(SOC_AM571x) || \
    defined(SOC_K2G) || defined(SOC_AM65XX) || defined(__ARM_ARCH_7A__)
  Board_initCfg boardCfg;
  boardCfg = BOARD_INIT_UNLOCK_MMR 
#ifndef IO_CONSOLE
               | BOARD_INIT_UART_STDIO
               | BOARD_INIT_MODULE_CLOCK
#endif
#if !defined(DEVICE_K2E) && !defined(SOC_K2E)
               | BOARD_INIT_PINMUX_CONFIG
#endif
    ;
  Board_init(boardCfg);
#if defined(SOC_AM571x)
  ret = Board_getIDInfo(&boardId);
  if (ret != BOARD_SOK)
  {
    return 0;
  }
#endif
#endif
  BIOS_start();
  return 0;
}

