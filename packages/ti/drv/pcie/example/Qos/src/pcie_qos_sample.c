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


#include "pcie_qos_sample.h"
#include <ti/drv/pcie/soc/pcie_soc.h>

#include <stdint.h>

#ifdef __TI_ARM_V7R4__
#include <ti/sysbios/hal/Cache.h>
#endif
#ifdef __aarch64__
#define COHERENT  /* Cache ops unnecessary */
#endif

#include "ti/board/board.h"
#include "pcie_sample_board.h"
#include <ti/csl/cslr_gic500.h>
#define PCIE_REV2_HW

#include <ti/csl/csl_chip.h>
#include <ti/csl/arch/csl_arch.h>

#ifdef UDMA
#include "pcie_udma.h"
#endif

#if defined (__TI_ARM_V7R4__)
#pragma DATA_SECTION(dstBuf, ".dstBufSec")
/* Cache coherence: Align must be a multiple of cache line size (32 bytes) to operate with cache enabled. */
/* Aligning to 256 bytes because the PCIe inbound offset register masks the last 8bits of the buffer address  */
#pragma DATA_ALIGN(dstBuf, 256) // TI way of aligning
#endif

/* last element in the buffer is a marker that indicates the buffer status: full/empty */
#define PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE 128
#define PCIE_EXAMPLE_UINT32_SIZE           4 /* preprocessor #if requires a real constant, not a sizeof() */

#define PCIE_EXAMPLE_DSTBUF_BYTES ((PCIE_BUFSIZE_APP + 1) * PCIE_EXAMPLE_UINT32_SIZE)
#define PCIE_EXAMPLE_DSTBUF_REM (PCIE_EXAMPLE_DSTBUF_BYTES % PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE)
#define PCIE_EXAMPLE_DSTBUF_PAD (PCIE_EXAMPLE_DSTBUF_REM ? (PCIE_EXAMPLE_MAX_CACHE_LINE_SIZE - PCIE_EXAMPLE_DSTBUF_REM) : 0)

typedef struct dstBuf_s {
  volatile uint32_t buf[PCIE_BUFSIZE_APP + 1];
  /* Cache coherence: Must pad to cache line size in order to enable cacheability */
#if PCIE_EXAMPLE_DSTBUF_PAD
  uint8_t padding[PCIE_EXAMPLE_DSTBUF_PAD];
#endif
} dstBuf_t;
dstBuf_t dstBuf; // for dstBuf

#define PCIE_EXAMPLE_BUF_EMPTY 0
#define PCIE_EXAMPLE_BUF_FULL  1

/* Does not need to be aligned (even for cache) since it is only accessed locally */
uint32_t srcBuf[PCIE_BUFSIZE_APP];

/* Address used for low priority traffic buffer starting address, currently placed in DDR, make sure it is not used */
const uint32_t lowPriAddr[3] = {0x80000000, 0x81000000, 0x82000000}; 

/* PCIE RC IATU table */ 
const uint32_t obRCcfg[] = {
	PCIE_WINDOW_MEM_BASE_VC0, PCIE_WINDOW_MEM_MASK_VC0, PCIE_OB_LO_ADDR_RC_VC0, PCIE_OB_HI_ADDR_RC_VC0,
	PCIE_WINDOW_MEM_BASE_VC1, PCIE_WINDOW_MEM_MASK_VC1, PCIE_OB_LO_ADDR_RC_VC1, PCIE_OB_HI_ADDR_RC_VC1,
	PCIE_WINDOW_MEM_BASE_VC2, PCIE_WINDOW_MEM_MASK_VC2, PCIE_OB_LO_ADDR_RC_VC2, PCIE_OB_HI_ADDR_RC_VC2, 
	PCIE_WINDOW_MEM_BASE_VC3, PCIE_WINDOW_MEM_MASK_VC3, PCIE_OB_LO_ADDR_RC_VC3, PCIE_OB_HI_ADDR_RC_VC3
};

const uint32_t ibRCcfg[] = {
	PCIE_IB_LO_ADDR_RC_VC0, PCIE_IB_HI_ADDR_RC_VC0, PCIE_WINDOW_MEM_MASK_VC0,
	PCIE_IB_LO_ADDR_RC_VC1, PCIE_IB_HI_ADDR_RC_VC1, PCIE_WINDOW_MEM_MASK_VC1, 
	PCIE_IB_LO_ADDR_RC_VC2, PCIE_IB_HI_ADDR_RC_VC2, PCIE_WINDOW_MEM_MASK_VC2, 
	PCIE_IB_LO_ADDR_RC_VC3, PCIE_IB_HI_ADDR_RC_VC3, PCIE_WINDOW_MEM_MASK_VC3
};

/* PCIE EP IATU table */ 
const uint32_t obEPcfg[] = {
	PCIE_WINDOW_MEM_BASE_VC0, PCIE_WINDOW_MEM_MASK_VC0, PCIE_OB_LO_ADDR_EP_VC0, PCIE_OB_HI_ADDR_EP_VC0,
	PCIE_WINDOW_MEM_BASE_VC1, PCIE_WINDOW_MEM_MASK_VC1, PCIE_OB_LO_ADDR_EP_VC1, PCIE_OB_HI_ADDR_EP_VC1,
	PCIE_WINDOW_MEM_BASE_VC2, PCIE_WINDOW_MEM_MASK_VC2, PCIE_OB_LO_ADDR_EP_VC2, PCIE_OB_HI_ADDR_EP_VC2, 
	PCIE_WINDOW_MEM_BASE_VC3, PCIE_WINDOW_MEM_MASK_VC3, PCIE_OB_LO_ADDR_EP_VC3, PCIE_OB_HI_ADDR_EP_VC3
};

const uint32_t ibEPcfg[] = {
	PCIE_IB_LO_ADDR_EP_VC0, PCIE_IB_HI_ADDR_EP_VC0, PCIE_WINDOW_MEM_MASK_VC0, 
	PCIE_IB_LO_ADDR_EP_VC1, PCIE_IB_HI_ADDR_EP_VC1, PCIE_WINDOW_MEM_MASK_VC1,
	PCIE_IB_LO_ADDR_EP_VC2, PCIE_IB_HI_ADDR_EP_VC2, PCIE_WINDOW_MEM_MASK_VC2,
	PCIE_IB_LO_ADDR_EP_VC3, PCIE_IB_HI_ADDR_EP_VC3, PCIE_WINDOW_MEM_MASK_VC3
};

/* Global config variable that controls
   the PCIe mode. It is global so it can be poked
   from CCS. It should be set either to EP or RC. */
pcieMode_e PcieModeGbl = pcie_EP_MODE;      

void cache_invalidate (void *ptr, int size)
{
#if defined(__TI_ARM_V7R4__)
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
#if defined(__arch64__) || defined(__TI_ARM_V7R4__)
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
 * Function: Enable/Disable DBI writes
 ****************************************************************************/
pcieRet_e pcieCfgDbi(Pcie_Handle handle, uint8_t enable)
{
  pcieRegisters_t        regs;
  pcieRet_e              retVal;

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

  return pcie_RET_OK;
} /* pcieCfgDbi */

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
#ifdef am65xx_idk
  PlatformPCIESSSerdesConfig(0, 0);
  PlatformPCIESSSerdesConfig(1, 0);
#else
  PlatformPCIESSSerdesConfig(1, 1);
#endif
 
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

  pcieStatusCmdReg_t     statusCmd;
  pcieDevStatCtrlReg_t   devStatCtrl;
  pcieAccrReg_t          accr;
                 
  pcieRegisters_t        setRegs;
  pcieRegisters_t        getRegs;

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
  
  /* Set gen2/link cap */
  if ((retVal = pcieSetGen2(handle)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("pcieSetGen2 failed!\n");
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

/*****************************************************************************
 * Function: Configure PCIe in End Point Mode
 ****************************************************************************/
pcieRet_e pcieCfgEP(Pcie_Handle handle)
{
  pcieRet_e retVal;

  pcieGen2Reg_t          gen2;
  pcieStatusCmdReg_t     statusCmd;
  pcieDevStatCtrlReg_t   devStatCtrl;
  pcieAccrReg_t          accr;
                 
  pcieRegisters_t        setRegs;
  pcieRegisters_t        getRegs;

  memset (&gen2,             0, sizeof(gen2));
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
  
  /* Set gen2/link cap */
  if ((retVal = pcieSetGen2(handle)) != pcie_RET_OK)
  {
    PCIE_logPrintf ("pcieSetGen2 failed!\n");
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

/*****************************************************************************
 * Function: Configure and enable Outbound Address Translation for rev 1/2
 ****************************************************************************/
pcieRet_e pcieObTransCfg(Pcie_Handle handle)
{
  pcieAtuRegionParams_t regionParams;
  pcieRet_e             retVal;
  uint32_t              resSize, i;

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
                     (uint32_t) 4U,
                     &regionParams)) != pcie_RET_OK) 
    {
      return retVal;
    }
  
    for (i=0; i<4; i++) {
	   memset (&regionParams, 0, sizeof(regionParams));
    
	   /*Configure OB region for memory transfer*/
       regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
       regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
       regionParams.enableRegion = 1;

       regionParams.lowerBaseAddr    = obRCcfg[0+4*i] + resSize;
       regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
       regionParams.regionWindowSize = obRCcfg[1+4*i];

       regionParams.lowerTargetAddr = obRCcfg[2+4*i];
       regionParams.upperTargetAddr = obRCcfg[3+4*i];

       if ( (retVal = Pcie_atuRegionConfig(
                     handle,
                     pcie_LOCATION_LOCAL,
                     (uint32_t) i,
                     &regionParams)) != pcie_RET_OK) 
       {
            return retVal;
       }
    } 
  } 
  else 
  {
	for (i=0; i<4; i++) {
	   memset (&regionParams, 0, sizeof(regionParams));
    
	   /*Configure OB region for memory transfer*/
       regionParams.regionDir    = PCIE_ATU_REGION_DIR_OUTBOUND;
       regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
       regionParams.enableRegion = 1;

       regionParams.lowerBaseAddr    = obEPcfg[0+4*i] + resSize;
       regionParams.upperBaseAddr    = 0; /* only 32 bits needed given data area size */
       regionParams.regionWindowSize = obEPcfg[1+4*i];

       regionParams.lowerTargetAddr = obEPcfg[2+4*i];
       regionParams.upperTargetAddr = obEPcfg[3+4*i];

       if ( (retVal = Pcie_atuRegionConfig(
                     handle,
                     pcie_LOCATION_LOCAL,
                     (uint32_t) i,
                     &regionParams)) != pcie_RET_OK) 
       {
            return retVal;
       } 
    }	   
  }	  
  return retVal;
}

/* Change function compilation optimization from -O2 to -O0 to avoid ARM GCC compiler bug
   for() loop failed to exit if using -O2*/
#ifdef __aarch64__
#pragma GCC optimize ("O0")
#endif
/*****************************************************************************
 * Function: Configure and enable Inbound Address Translation for rev 1/2
 ****************************************************************************/
pcieRet_e pcieIbTransCfg(Pcie_Handle handle)
{
  pcieAtuRegionParams_t regionParams;
  pcieRet_e             retVal = pcie_RET_OK;
  uint32_t              i;

  for (i=0; i<4; i++) {
      /*Configure IB region for memory transfer*/
      memset (&regionParams, 0, sizeof(regionParams));
  
      regionParams.regionDir    = PCIE_ATU_REGION_DIR_INBOUND;
      regionParams.tlpType      = PCIE_TLP_TYPE_MEM;
      regionParams.enableRegion = 1;
      regionParams.matchMode    = PCIE_ATU_REGION_MATCH_MODE_ADDR;

      if(PcieModeGbl == pcie_RC_MODE) {
         regionParams.lowerBaseAddr    = ibRCcfg[0+i*3];
         regionParams.upperBaseAddr    = ibRCcfg[1+i*3];
         regionParams.regionWindowSize = ibRCcfg[2+i*3];

         /* This aligns the buffer to 4K, which needs to be compensated by the application */
         regionParams.lowerTargetAddr = ((uint32_t)lowPriAddr[i] & ~0xfffU) ;
		 if (i == 3) {
			regionParams.lowerTargetAddr = ((uint32_t)dstBuf.buf & ~0xfffU) ; 
		 } 	 
         regionParams.upperTargetAddr = 0;

         if ( (retVal = Pcie_atuRegionConfig(
                   handle,
                   pcie_LOCATION_LOCAL,
                   i,
                   &regionParams)) != pcie_RET_OK)
         {
            return retVal;
         }
      }
	  else {
	     regionParams.lowerBaseAddr    = ibEPcfg[0+i*3];
         regionParams.upperBaseAddr    = ibEPcfg[1+i*3];
         regionParams.regionWindowSize = ibEPcfg[2+i*3];

         /* This aligns the buffer to 4K, which needs to be compensated by the application */
         regionParams.lowerTargetAddr = ((uint32_t)lowPriAddr[i] & ~0xfffU) ;
		 if (i == 3) {
			regionParams.lowerTargetAddr = ((uint32_t)dstBuf.buf & ~0xfffU) ; 
		 } 	 
         regionParams.upperTargetAddr = 0;

         if ( (retVal = Pcie_atuRegionConfig(
                   handle,
                   pcie_LOCATION_LOCAL,
                   i,
                   &regionParams)) != pcie_RET_OK)
         {
            return retVal;
         }
	  }
  }
  return retVal;
}
#ifdef __aarch64__
#pragma GCC optimize ("O2")
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
	*(unsigned int*)(uintptr_t)(lowPriAddr[0] + 4*i) = 0;
  	*(unsigned int*)(uintptr_t)(lowPriAddr[1] + 4*i) = 0;
 	*(unsigned int*)(uintptr_t)(lowPriAddr[2] + 4*i) = 0;
  }
  
  dstBuf.buf[PCIE_BUFSIZE_APP] = PCIE_EXAMPLE_BUF_EMPTY;
  *(unsigned int*)(uintptr_t)(lowPriAddr[0] + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_EMPTY;
  *(unsigned int*)(uintptr_t)(lowPriAddr[1] + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_EMPTY;  
  *(unsigned int*)(uintptr_t)(lowPriAddr[2] + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_EMPTY;
  
  cache_writeback ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
  cache_writeback ((void *)lowPriAddr[0], PCIE_EXAMPLE_DSTBUF_BYTES);
  cache_writeback ((void *)lowPriAddr[1], PCIE_EXAMPLE_DSTBUF_BYTES);
  cache_writeback ((void *)lowPriAddr[2], PCIE_EXAMPLE_DSTBUF_BYTES);
}

/*****************************************************************************
 * Function: Check LTSSM status and wait for the link to be up
 ****************************************************************************/
void pcieWaitLinkUp(Pcie_Handle handle)
{
  pcieRegisters_t  getRegs;

  memset (&getRegs, 0, sizeof(getRegs));

  pcieDebug0Reg_t            ltssmStateReg;
  getRegs.debug0 =          &ltssmStateReg;
  
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
  /* Get link status */
  pcieRet_e retVal = pcie_RET_OK;
  pcieRegisters_t regs;
  pcieLinkStatCtrlReg_t linkStatCtrl;
    
  memset (&regs, 0, sizeof(regs));
  regs.linkStatCtrl = &linkStatCtrl;

  PCIE_logPrintf ("Checking link speed and # of lanes\n");
  retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
  if (retVal != pcie_RET_OK) {
     PCIE_logPrintf ("Failed to read linkStatCtrl: %d\n", retVal);
  } else {
    /* Check number of lanes */
    PCIE_logPrintf ("GEN %d speed with x%d lane\n", (int)linkStatCtrl.linkSpeed, linkStatCtrl.negotiatedLinkWd);
  }

  return pcie_RET_OK;
}

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
  lnkCtrlReg.lnkMode = 3;
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

void pcieSetupVC(uint32_t csl_pcie_dat_base) {

    /* AM65x supports 4 virtual channels (VC) and 4 traffic classes (TC)
     * One or multiple TCs can be mapped to a VC.
     * One TC must not be mapped to multiple VCs.
     * TC/VC mapping must be identical for ports on both sides of a link.
     * For simplicity, a 1:1 TC to VC mapping is used here
     */
	 
	PCIE_logPrintf("PCIE VC setup ....\n");
		
    /* VC_TC_MAP_VC0_BIT1: Bit locations within this field correspond to TC values */
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x115c)) = 0x00000001 ; //TC0 ----> VC0
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x115c)) = 0x80000001 ;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x1168)) = 0x01000002 ; //TC1 ----> VC1
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x1168)) = 0x81000002 ;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x1174)) = 0x02000004 ; //TC2 ----> VC2
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x1174)) = 0x82000004 ;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x1180)) = 0x03000008 ; //TC3 ----> VC3
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x1180)) = 0x83000008 ;

    /* change the IATU OB and IB 1 to use TC1 */
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6204)) = 0x00000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6304)) = 0x00000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6200)) = 0x00000020;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6300)) = 0x00000020;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6204)) = 0x80000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6304)) = 0x80000000;

    /* change the IATU OB and IB 2 to use TC2 */
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6404)) = 0x00000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6504)) = 0x00000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6400)) = 0x00000040;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6500)) = 0x00000040;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6404)) = 0x80000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6504)) = 0x80000000;

    /* change the IATU OB and IB 3 to use TC3 */
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6604)) = 0x00000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6704)) = 0x00000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6600)) = 0x00000060;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6700)) = 0x00000060;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6604)) = 0x80000000;
    *((uint32_t *)(uintptr_t) (csl_pcie_dat_base + 0x6704)) = 0x80000000;

    /* Check VC negotiation pending */
    while ((*((uint32_t volatile *)(uintptr_t) (csl_pcie_dat_base + 0x1160)) >> 17) & 0x1);
    while ((*((uint32_t volatile *)(uintptr_t) (csl_pcie_dat_base + 0x116C)) >> 17) & 0x1);
    while ((*((uint32_t volatile *)(uintptr_t) (csl_pcie_dat_base + 0x1178)) >> 17) & 0x1);
    while ((*((uint32_t volatile *)(uintptr_t) (csl_pcie_dat_base + 0x1184)) >> 17) & 0x1);
	
	PCIE_logPrintf("PCIE VC setup passed\n");
}

void pcieConfigMasterPortOrderId(void){

    uint32_t cba_qos_base = CSL_CBASS0_QOS_BASE;
    // configure hp master reads on pcie0 to use orderid 8 for TC3
    *((uint32_t *)(uintptr_t) (cba_qos_base + 0x18000 + 0x00) )   = 0x00007000; 
    *((uint32_t *)(uintptr_t) (cba_qos_base + 0x18000 + 0x0c) )   = 0x00007080;
    // configure hp master writes on pcie0 to use orderid 8 for TC3
    *((uint32_t *)(uintptr_t) (cba_qos_base + 0x18400 + 0x00) )   = 0x00007000; 
    *((uint32_t *)(uintptr_t) (cba_qos_base + 0x18400 + 0x0c) )   = 0x00007080;
	
    // configure hp master reads on pcie1 to use orderid 8
    *((uint32_t *)(uintptr_t) (cba_qos_base + 0x19000 + 0x0C) )   = 0x00007080;
    *((uint32_t *)(uintptr_t) (cba_qos_base + 0x19000 + 0x14) )   = 0x00000008;
    // configure hp master writes on pcie1 to use orderid 8
    *((uint32_t *)(uintptr_t) (cba_qos_base + 0x19400 + 0x0C) )   = 0x00007080;
    *((uint32_t *)(uintptr_t) (cba_qos_base + 0x19400 + 0x14) )   = 0x00000008;
}

void configureNBThreadmap(void) {
	
	/* Maxwell has 2 ports into the NB that are based on orderIDs (0-7 and 8-15), so in effect the bits are for specific orderIDs. NAVSS_THREADMAP register has 2 MMR bits, where bit 0 is for orderIDs 0-7, and bit 1 is for orderIDs 8-15. And the value of each bit determines which thread in MSMC to map, 0 = thread 0 (NRT), 1 = thread 2 (RT).*/
	/* NB0 for MSMC, NB1 for DDR */
	*((uint32_t *)(uintptr_t) (0x03802010)) = 0x00000003;
    *((uint32_t *)(uintptr_t) (0x03803010)) = 0x00000000;
}

void configureNB1DdrAttr(void) {

    uint32_t i;
    for (i = 0; i < 16; i++) {
        //256MB DDR
        *((uint32_t *)(uintptr_t) (0x03840000 + 4*i)) = 0x00000000;
    }
}

void ddrRefreshCtrlCheck(void) {
    uint32_t temp, refreshBurst, perBankRefresh;

    temp = *((uint32_t *)(uintptr_t)(CSL_DDRSS0_CTL_CFG_BASE + 0x50));
    refreshBurst = (temp>>4)&0x1F;
    perBankRefresh = (temp>>2)&0x1;

    if ((refreshBurst != 0) || (perBankRefresh != 1)) {
        PCIE_logPrintf("Suggest setting REFRESH_BURST to 0 and PER_BANK_REFRESH to 1 for best results\n");
    } else {
        PCIE_logPrintf("DDR CFG REFRESH_BURST: %d, PER_BANK_REFRESH: %d as expected\n", refreshBurst, perBankRefresh);
    }
}

void printQosResults(uint32_t iteration, uint32_t dataArray[], uint32_t size) {
    uint32_t i;
    uint32_t min = 0xFFFFFFFF, max = 0, sum = 0, average;
    uint32_t outliner = 0;

    for (i = 0; i < size; i++) {
        sum += dataArray[i];
        if (dataArray[i] < min) {
            min = dataArray[i];
        }
        if (dataArray[i] > max) {
            max = dataArray[i];
        }
    }

    average = (float)sum/size;
    for (i = 0; i < size; i++) {
        if ((float)dataArray[i] >= 1.2*average) {
            outliner++;
        }
    }
    PCIE_logPrintf("================================================================================\n");
    PCIE_logPrintf("Test iteration %d: total %d latency data collected.\n", iteration+1, size);
    PCIE_logPrintf("Minimum latency %d cycles, Maximum latency %d cycles, average latency %d cycles\n",
            min, max, (uint32_t)average);
    PCIE_logPrintf("Total outliners(20 percent higher than average): %d\n", outliner);
}
/*****************************************************************************
 * Function: pcie main task 
 ****************************************************************************/

int i;
void pcie (void)
{
  int32_t          deviceNum = 0;
  pcieRet_e        retVal;
  Pcie_Handle      handle = NULL;
  void            *pcieBase;
  dstBuf_t        *pciedstBufBase;
  uint32_t    i;
  char             pcieModeResponse;

#ifdef am65xx_evm
  deviceNum = 1; /* The second interface is hooked up on GP EVM */
#endif

#ifndef UDMA
      if (pcieUdmaTest((void *)0x81000000, PCIE_EXAMPLE_LINE_SIZE))
      {
         PCIE_logPrintf("UDMA test failed\n");
         exit(1);
      }
#endif

  /* Get remote buffer out of cache */
  cache_writeback ((void *)&dstBuf, sizeof(dstBuf));

  PCIE_logPrintf ("**********************************************\n");
  PCIE_logPrintf ("*             PCIe Test Start                *\n");

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
  
  if(PcieModeGbl == pcie_RC_MODE)
  {
    /* Configure application registers for Root Complex*/
    if ((retVal = pcieCfgRC(handle)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure PCIe in RC mode (%d)\n", (int)retVal);
      exit(1);
    }
    
    if ((retVal = pcieIbTransCfg(handle)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure Inbound Translation (%d)\n", (int)retVal);
      exit(1);
    }
    else
    {
        PCIE_logPrintf ("Successfully configured Inbound Translation!\n");
    }

    if ((retVal = pcieObTransCfg (handle)) != pcie_RET_OK) 
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

    if ((retVal = pcieIbTransCfg(handle)) != pcie_RET_OK) 
    {
      PCIE_logPrintf ("Failed to configure Inbound Translation (%d)!\n", (int)retVal);
      exit(1);
    }
    else
    {
      PCIE_logPrintf ("Successfully configured Inbound Translation!\n");
    }
	
    if ((retVal = pcieObTransCfg (handle)) != pcie_RET_OK) 
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

  pcieSetupVC(PCIE_REG_BASE);

  pcieConfigMasterPortOrderId();
  
  configureNBThreadmap();
  
  if ((retVal = Pcie_getMemSpaceRange (handle, &pcieBase, NULL)) != pcie_RET_OK) {
    PCIE_logPrintf ("getMemSpaceRange failed (%d)\n", (int)retVal);
    exit(1);
  }
  
  /* Adjust PCIE base to point at remote target buffer */
  pcieBase = (char *) PCIE_WINDOW_MEM_BASE_VC3 +  /* data area doesn't start at low address */
                     (((uint32_t)&dstBuf) & 0xffff); /* dstBuf needs to be 64K aligned in addr tran */

  pciedstBufBase = (dstBuf_t *)pcieBase; 

  if(PcieModeGbl == pcie_RC_MODE)
  {
    /**********************************************************************/
    /* Push a single message to the EP then verify that it is echoed back */
    /**********************************************************************/

    /* Write from RC to EP                                                */
    for (i=0; i<PCIE_BUFSIZE_APP; i++)
    {
      pciedstBufBase->buf[i] = srcBuf[i] << 3;
	  *(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC0 + 4*i) = srcBuf[i] << 0;
  	  *(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC1 + 4*i) = srcBuf[i] << 1;
 	  *(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC2 + 4*i) = srcBuf[i] << 2;
    }
    
    /* Mark that the buffer is full, so EP can process it */
    pciedstBufBase->buf[PCIE_BUFSIZE_APP] = PCIE_EXAMPLE_BUF_FULL;
    *(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC0 + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_FULL;
  	*(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC1 + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_FULL;
 	*(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC2 + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_FULL;

    /* Note on cache coherence: Write back is not necessary because pcieBase is in
       peripheral address space instead of physical memory*/
    
    /* Data sent to EP.
       RC waits for the loopback to be completed and
       receive data back from EP */

	do {
      cache_invalidate ((void *)lowPriAddr[0], PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[0] + PCIE_BUFSIZE_APP * 4) != PCIE_EXAMPLE_BUF_FULL);
	PCIE_logPrintf ("Root Complex received VC0 data.\n");

	do {
      cache_invalidate ((void *)lowPriAddr[1], PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[1] + PCIE_BUFSIZE_APP * 4) != PCIE_EXAMPLE_BUF_FULL);
	PCIE_logPrintf ("Root Complex received VC1 data.\n");

	do {
      cache_invalidate ((void *)lowPriAddr[2], PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[2] + PCIE_BUFSIZE_APP * 4) != PCIE_EXAMPLE_BUF_FULL);
	PCIE_logPrintf ("Root Complex received VC2 data.\n");
	
	do {
      cache_invalidate ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(dstBuf.buf[PCIE_BUFSIZE_APP] != PCIE_EXAMPLE_BUF_FULL);
    PCIE_logPrintf ("Root Complex received VC3 data.\n");

    /* check all the data */
    for (i=0; i<PCIE_BUFSIZE_APP; i++)
    {
      if(dstBuf.buf[i] != (srcBuf[i] << 3))
      {
        PCIE_logPrintf ("Received data = %u\nTransmited data = %u\nIndex = %u.\n\nTest failed.\n",
                        (unsigned)dstBuf.buf[i], (unsigned)srcBuf[i], (unsigned)i);
        exit(1);
      }
	  if(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[0] + i*4) != (srcBuf[i] << 0))
      {
        PCIE_logPrintf ("Received data = %u\nTransmited data = %u\nIndex = %u.\n\nTest failed.\n",
                        *(uint32_t volatile *)(lowPriAddr[0] + i*4), (unsigned)srcBuf[i]<<0, (unsigned)i);
        exit(1);
      }
	  if(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[1] + i*4) != (srcBuf[i] << 1))
      {
        PCIE_logPrintf ("Received data = %u\nTransmited data = %u\nIndex = %u.\n\nTest failed.\n",
                        *(uint32_t volatile *)(lowPriAddr[1] + i*4), (unsigned)srcBuf[i]<<1, (unsigned)i);
        exit(1);
      }	 
	  if(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[2] + i*4) != (srcBuf[i] << 2))
      {
        PCIE_logPrintf ("Received data = %u\nTransmited data = %u\nIndex = %u.\n\nTest failed.\n",
                        *(uint32_t volatile *)(lowPriAddr[2] + i*4), (unsigned)srcBuf[i]<<2, (unsigned)i);
        exit(1);
      }	  
    }
    
    PCIE_logPrintf ("Root Complex received all correct data.\n");
  
  }
  else
  {
    /**********************************************************************/
    /* Wait for a single message from the RC then echo it back            */
    /**********************************************************************/

    /* EP waits for the data received from RC */
    do {
      cache_invalidate ((void *)lowPriAddr[0], PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[0] + PCIE_BUFSIZE_APP * 4) != PCIE_EXAMPLE_BUF_FULL);
	PCIE_logPrintf ("End Point received VC0 data.\n");     
	
	do {
      cache_invalidate ((void *)lowPriAddr[1], PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[1] + PCIE_BUFSIZE_APP * 4) != PCIE_EXAMPLE_BUF_FULL);
	PCIE_logPrintf ("End Point received VC1 data.\n");     
	
	do {
      cache_invalidate ((void *)lowPriAddr[2], PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(*(uint32_t volatile *)(uintptr_t)(lowPriAddr[2] + PCIE_BUFSIZE_APP * 4) != PCIE_EXAMPLE_BUF_FULL);
	PCIE_logPrintf ("End Point received VC2 data.\n");     
	
	do {
      cache_invalidate ((void *)dstBuf.buf, PCIE_EXAMPLE_DSTBUF_BYTES);
    } while(dstBuf.buf[PCIE_BUFSIZE_APP] != PCIE_EXAMPLE_BUF_FULL);
	PCIE_logPrintf ("End Point received VC3 data.\n");

    /* Loopback to RC what was written in the DST buffer.
       Write from EP to RC */
    for (i=0; i<PCIE_BUFSIZE_APP; i++)
    {
      pciedstBufBase->buf[i] = dstBuf.buf[i];
	  *(uint32_t*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC0 + 4*i) = *(uint32_t*)(uintptr_t)(lowPriAddr[0] + i*4);
      *(uint32_t*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC1 + 4*i) = *(uint32_t*)(uintptr_t)(lowPriAddr[1] + i*4);
      *(uint32_t*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC2 + 4*i) = *(uint32_t*)(uintptr_t)(lowPriAddr[2] + i*4);
    }
    
    /* Mark that the buffer is full, so RC can process it */
    pciedstBufBase->buf[PCIE_BUFSIZE_APP] = PCIE_EXAMPLE_BUF_FULL;
    *(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC0 + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_FULL;
  	*(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC1 + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_FULL;
 	*(unsigned int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC2 + 4*PCIE_BUFSIZE_APP) = PCIE_EXAMPLE_BUF_FULL;

    /* Note on cache coherence: Write back is not necessary because pcieBase is in
       peripheral address space instead of physical memory*/

    PCIE_logPrintf ("End Point sent data to Root Complex, completing the loopback.\n");
  }
 
  /* PCIE QOS test only runs from RC side
     RC reads from EP's DDR into its own DDR through VC0, as the background traffic.
	 Meanwhile, RC reads from EP's MSMC through VC3, the read latency is recorded.
     The record buffer is placed in OCMC to avoid interference with code running in MSMC
  */ 
  if(PcieModeGbl == pcie_RC_MODE) {
#ifdef UDMA
      PCIE_logPrintf("PCIE QOS test started ....\n");
	  
      ddrRefreshCtrlCheck();
	  
	  configureNB1DdrAttr();

      if (pcieUdmaTest((void *)PCIE_WINDOW_MEM_BASE_VC0, PCIE_EXAMPLE_LINE_SIZE))
	  {
         PCIE_logPrintf("UDMA test failed\n");
         exit(1);
      }
#endif
  }

  PCIE_logPrintf ("Test passed.\n");

  BIOS_exit(0);

}

int main() {
  Task_Params params;
  Task_Params_init (&params);
  params.stackSize = 36864; //32768;
  Task_create((Task_FuncPtr) pcie, &params, NULL);

  Board_initCfg boardCfg;
  boardCfg = BOARD_INIT_UNLOCK_MMR 
#ifndef IO_CONSOLE
               | BOARD_INIT_UART_STDIO
               | BOARD_INIT_MODULE_CLOCK
#endif
               | BOARD_INIT_PINMUX_CONFIG
    ;
  Board_init(boardCfg);

  BIOS_start();
  return 0;
}

