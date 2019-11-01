/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2018-2019
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
 * @file pcie_example_board.h
 *
 */
#if defined (QOS)
#include "pcie_qos_sample.h"
#else
#include "pcie_sample.h"
#endif
#include "pcie_sample_board.h"
#include <ti/csl/soc.h>
#include <ti/csl/csl_serdes.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_serdes_pcie.h>
#include <ti/csl/cslr_pcie.h>

#include <ti/osal/osal.h>
#include <ti/drv/pcie/pcie.h>
#include <ti/drv/sciclient/sciclient.h>

#include <string.h>

#include <stdint.h>
#include <stdbool.h>

/* UART Header files */
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

#if defined (__aarch64__)
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/arm/v8a/Mmu.h>

struct MmuCfg_t {
  uint64_t    vaddr;
  uint64_t    paddr;
  size_t      size;
  int         attrInd;
} MmuCfgTbl[] = {
  { 0x00100000, 0x00100000, 0x00900000, 0 }, /* Main MMR0     */
  { 0x01800000, 0x01800000, 0x00100000, 0 }, /* gicv3         */
  { 0x02000000, 0x02000000, 0x00100000, 0 }, /* I2C           */
  { 0x02100000, 0x02100000, 0x00080000, 0 }, /* McSPI         */
  { 0x02400000, 0x02400000, 0x000c0000, 0 }, /* dmtimer       */
  { 0x02800000, 0x02800000, 0x00001000, 0 }, /* uart          */
  { 0x02980000, 0x02980000, 0x00010000, 0 }, /* DDRSS0        */
  { 0x02C40000, 0x02C40000, 0x00100000, 0 }, /* pinmux ctrl   */
  { 0x03800000, 0x03800000, 0x00060000, 0 }, /* NAVSS0_NBSS   */
  { 0x05500000, 0x05500000, 0x00200000, 0 }, /* PCIE ctrl     */
  { 0x10000000, 0x10000000, 0x10000000, 0 }, /* PCIE data     */
  { 0x28380000, 0x28380000, 0x03880000, 0 }, /* mcu navss     */
  { 0x30000000, 0x30000000, 0x0F000000, 0 }, /* ctrcontrol0   */
  { 0x40D00000, 0x40D00000, 0x00002000, 0 }, /* PLL0          */
  { 0x40F00000, 0x40F00000, 0x00020000, 0 }, /* MCU MMR0      */
  { 0x41C00000, 0x41C00000, 0x00080000, 7 }, /* ocmc          */
  { 0x42000000, 0x42000000, 0x00001000, 0 }, /* PSC WKUP      */
  { 0x43000000, 0x43000000, 0x00020000, 0 }, /* WKUP MMR0     */
  { 0x45D80000, 0x45D80000, 0x00020000, 0 }, /* CBASS0_QOS    */
  { 0x70000000, 0x70000000, 0x04000000, 7 }, /* msmc          */
  { 0x80000000, 0x80000000, 0x80000000, 7 }, /* DDR           */
  { 0, 0, 0, 8 } /* attrInd = 8 -> end of table */
};

void InitMmu(void)
{
  bool        retVal = FALSE;
  uint32_t    j = 0;
  Mmu_MapAttrs attrs;
  Mmu_initMapAttrs(&attrs);
  while (MmuCfgTbl[j].attrInd < 8) {
    attrs.attrIndx = MmuCfgTbl[j].attrInd;
    retVal = Mmu_map(MmuCfgTbl[j].vaddr, MmuCfgTbl[j].paddr, MmuCfgTbl[j].size, &attrs);
    if(retVal == FALSE)
      break;
    j++;
  }

  if(retVal == FALSE) {
    UART_printf("Mmu_map idx %d returned error %d", j, retVal);
    while(1);
  }
}
#endif

#define KICK0 0x68EF3490ull
#define KICK1 0xD172BC5Aull

static void unlock_mmrs(void)
{

  /* Unlock Lock1 Kick Registers */
  *(volatile uint32_t *) (CSL_CTRL_MMR0_CFG0_BASE
          + CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0) = KICK0;
  *(volatile uint32_t *) (CSL_CTRL_MMR0_CFG0_BASE
          + CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1) = KICK1;
}

#define PCIE_REF_CLOCK            CSL_SERDES_REF_CLOCK_100M /* Ref clock of serdes */
#define PCIE_LINK_RATE            CSL_SERDES_LINK_RATE_8G /* Link rate of serdes */
#define PCIE_NUM_LANES            1 /* Number of lanes to be tested */
#define PCIE_LANE_MASK            0x1 /* All lanes are set */
#define PCIE_PHY_TYPE             CSL_SERDES_PHY_TYPE_PCIe /* For running PCIe tests */
#define PCIE_LANE_RATE            CSL_SERDES_LANE_FULL_RATE /* Set to run at full rate of the link rate */
#define PCIE_LOOPBACK_MODE        CSL_SERDES_LOOPBACK_DISABLED /* For internal near end serial loopback tests */
#define PCIE_OPERATING_MODE       CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM /* Should always be set to Diagnostic Mode for BER, EYE and PRBS tests */
#define PCIE_FORCEATTBOOST        CSL_SERDES_FORCE_ATT_BOOST_DISABLED

void CSL_serdesWaitForCMUOK(uint32_t baseAddr);

void PlatformPCIESSSerdesConfig(int32_t serdes, int32_t iface)
{
  uint32_t serdesBase;
  uint32_t i;
  static int32_t sciInit = 0;
  int32_t sciStatus = CSL_EFAIL;
  CSL_SerdesResult status;
  CSL_SerdesLaneEnableParams serdesLaneEnableParams;
  CSL_SerdesLaneEnableStatus laneRetVal = CSL_SERDES_LANE_ENABLE_NO_ERR;
  Sciclient_ConfigPrms_t        sciConfig =
  {
    SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
  };
  const Sciclient_ReqPrm_t      sciReqPrm =
  {
    TISCI_MSG_VERSION,
    TISCI_MSG_FLAG_AOP,
    (uint8_t *)NULL,
    0,
    SCICLIENT_SERVICE_WAIT_FOREVER
  };

  struct tisci_msg_version_resp sciResponse;
  Sciclient_RespPrm_t           sciRespPrm =
  {
    0,
    (uint8_t *) &sciResponse,
    (uint32_t)sizeof (sciResponse)
  };

  if (! sciInit)
  {
    sciStatus = Sciclient_init(&sciConfig);
    if (sciStatus != CSL_PASS)
    {
      UART_printf("SYSFW init ...FAILED\n");
      exit(1);
    }
    /* Check that sciclient is working (tap the mic) */
    sciStatus = Sciclient_service(&sciReqPrm, &sciRespPrm);
    if (CSL_PASS == sciStatus)
    {
      if (sciRespPrm.flags == (uint32_t)TISCI_MSG_FLAG_ACK)
      {
        UART_printf("SYSFW  ver %s running\n", (char *) sciResponse.str);
      }
      else
      {
        UART_printf("SYSFW Get Version failed \n");
        exit(1);
      }
    }
    sciInit = 1;
  }
  memset (&serdesLaneEnableParams, 0, sizeof(serdesLaneEnableParams));

  unlock_mmrs();

  if (serdes == 0)
  {
    serdesBase = CSL_SERDES0_BASE;
  }
  else
  {
    serdesBase = CSL_SERDES1_BASE;
  }

  /* CTRLMMR_SERDES0_CTRL Register 0x0010_4080
   [1:0] LANE_FUNC_SEL: Input that selects which IP input is muxed into serdes output lane
   00 : USB
   01 : PCIE0 L0
   10 : ICSS2 L0
   11 : Reserved
   [7:4] CLK_SEL
   Bit0 - Select Right CML clock
   Bit1 - Enable Chaining
   Bit2 - Select Left CML clock
   Bit3 - Right side chaining enable is inverted from Left side (Bit1)

   CTRLMMR_SERDES1_CTRL Register 0x0010_4090
   [1:0] LANE_FUNC_SEL: Input that selects which IP input is muxed into serdes output lane
   00 : PCIE1 L0
   01 : PCIE0 L1
   10 : ICSS2 L1
   11 : Reserved
   [7:4] CLK_SEL
   Bit0 - Select Right CML clock
   Bit1 - Enable Chaining
   Bit2 - Select Left CML clock
   Bit3 - Right side chaining enable is inverted from Left side (Bit1)
   */

  if (serdes == 0)
  {
    CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL), 1, 0, 0x1);
    CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL), 7, 4, 0x4); //Left CML
    sciStatus = Sciclient_pmSetModuleClkParent (
                  TISCI_DEV_SERDES0,
                  TISCI_DEV_SERDES0_BUS_LI_REFCLK,
                  TISCI_DEV_SERDES0_BUS_LI_REFCLK_PARENT_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_HSDIV_CLKOUT4_CLK,
                  SCICLIENT_SERVICE_WAIT_FOREVER);
  }
  else
  {
    if (iface == 0)
    {
      CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CTRL), 1, 0, 0x1);
    }
    else
    {
      CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CTRL), 1, 0, 0x0);
    }
    CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CTRL), 7, 4, 0x1); //Right CML

    sciStatus = Sciclient_pmSetModuleClkParent (
                  TISCI_DEV_SERDES1,
                  TISCI_DEV_SERDES1_BUS_RI_REFCLK,
                  TISCI_DEV_SERDES1_BUS_RI_REFCLK_PARENT_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_HSDIV_CLKOUT4_CLK,
                  SCICLIENT_SERVICE_WAIT_FOREVER);
  }

  if (CSL_PASS != sciStatus)
  {
    UART_printf("SYSFW setModuleClkParent failed\n");
    exit(1);
  }

  //PUT SERDES PHY OUT OF RESET
  CSL_serdesPorReset(serdesBase);

  //Setup Serdes
  serdesLaneEnableParams.baseAddr = serdesBase;
  serdesLaneEnableParams.refClock = PCIE_REF_CLOCK;
  serdesLaneEnableParams.linkRate = PCIE_LINK_RATE;
  serdesLaneEnableParams.numLanes = PCIE_NUM_LANES;
  serdesLaneEnableParams.laneMask = PCIE_LANE_MASK;
  serdesLaneEnableParams.phyType = PCIE_PHY_TYPE;
  serdesLaneEnableParams.operatingMode = PCIE_OPERATING_MODE;

  for (i = 0; i < serdesLaneEnableParams.numLanes; i++)
  {
    serdesLaneEnableParams.laneCtrlRate[i] = PCIE_LANE_RATE;
    serdesLaneEnableParams.loopbackMode[i] = PCIE_LOOPBACK_MODE;
  }

  status = CSL_serdesPCIeInit(serdesLaneEnableParams.baseAddr,
                              serdesLaneEnableParams.numLanes,
                              serdesLaneEnableParams.refClock,
                              serdesLaneEnableParams.linkRate); //Use this for PCIe serdes config load

  /* Return error if input params are invalid */
  if (status != CSL_SERDES_NO_ERR)
  {
    PCIE_logPrintf ("Invalid SERDES Init Params \n");
    exit (1);
  }

  /* Common Lane Enable API for lane enable, pll enable etc */
  laneRetVal = CSL_serdesLaneEnable(&serdesLaneEnableParams);

  if (laneRetVal != 0)
  {
    PCIE_logPrintf ("Invalid Serdes Lane Enable\n");
    exit (1);
  }

  PCIE_logPrintf ("Serdes Init Complete (%d)\n", serdes);

  /* Check CMU_OK */
  CSL_serdesWaitForCMUOK(serdesLaneEnableParams.baseAddr);
}

/* On the AM65XX devices, no interrupts can be sent to R5
   because the legacy interrupts are not connected to the NVIC,
   and MSI/MSIX use direct writes to the GIC-500
 */
#ifdef BUILD_MPU
static uint32_t msi_ints = 0, intx_ints = 0, unknown_ints = 0;
SemaphoreP_Handle semaphoreHandle;
static void PlatformMsiIsr (uintptr_t vhandle)
{
    /* Interrupt is already demuxed as it is specific SPI */
    msi_ints++;

    /* Tell user task ISR happend */
    SemaphoreP_postFromISR (semaphoreHandle);
}

static void PlatformIntxIsr (uintptr_t vhandle)
{
    /* Interrupt is already demuxed as it is specific SPI */
    intx_ints++;

    /* Tell user task ISR happend */
    SemaphoreP_postFromISR (semaphoreHandle);
}

void *pcieMsiHwi;
void *pcieIntxHwi;

SemaphoreP_Handle PlatformSetupMSIAndINTX (Pcie_Handle handle)
{
    OsalRegisterIntrParams_t           interruptRegParams;
    pcieRet_e                          retVal;
    pcieRegisters_t                    regs;
    pcieRegisters_t                    epRegs;
    pcieMsiCapReg_t                    epMsiCap;
    pcieMsiLo32Reg_t                   epMsiLowAddress;
    pcieMsiUp32Reg_t                   epMsiUpAddress;
    pcieMsiDataReg_t                   epMsiDataVal;
    pcieLegacyIrqEnableSetReg_t        rcLegacyEnable;

    /* Create a semaphore for user task to wait for interrupt */
    semaphoreHandle = SemaphoreP_create (0, NULL);
    if (!semaphoreHandle)
    {
        PCIE_logPrintf("Failed to create semaphore\n");
        exit(1);
    }
    memset (&regs, 0, sizeof(regs));
    memset (&epRegs, 0, sizeof(epRegs));
    memset (&epMsiCap, 0, sizeof(epMsiCap));
    memset (&epMsiLowAddress, 0, sizeof(epMsiLowAddress));
    memset (&epMsiUpAddress, 0, sizeof(epMsiUpAddress));
    memset (&epMsiDataVal, 0, sizeof(epMsiDataVal));

    /* Read existing EP registers */
    epRegs.msiCap = &epMsiCap;
    epRegs.msiLo32 = &epMsiLowAddress;
    epRegs.msiUp32 = &epMsiUpAddress;
    epRegs.msiData = &epMsiDataVal;
    retVal = Pcie_readRegs (handle, pcie_LOCATION_REMOTE, &epRegs);
    if (retVal != pcie_RET_OK)
    {
        PCIE_logPrintf ("read of EP interrupt regs failed (%d)\n", retVal);
        exit(1);
    }

    /* Enable MSI on EP */
    epMsiCap.msiEn = 1;
    epMsiDataVal.data = PCIE_WINDOW_MSI_DATA;
    epMsiUpAddress.addr = 0;
    epMsiLowAddress.addr = (PCIE_PCIE_MSI_BASE+PCIE_PCIE_MSI_OFF)>>2; /* because lld wants upper 30 bits only */
    retVal = Pcie_writeRegs (handle, pcie_LOCATION_REMOTE, &epRegs);
    if (retVal != pcie_RET_OK)
    {
        PCIE_logPrintf ("write of EP interrupt regs failed (%d)\n", retVal);
        exit(1);
    }

    /* Enable legacy on RC */
    memset (&rcLegacyEnable, 0, sizeof(rcLegacyEnable));
    regs.legacyIrqEnableSet[0] = &rcLegacyEnable;

    /* read current */
    retVal = Pcie_readRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
        PCIE_logPrintf("Can't read legacy enable\n");
        exit(1);
    }
    rcLegacyEnable.legacyIrqEnSet = 1;

    /* write back */
    retVal = Pcie_writeRegs (handle, pcie_LOCATION_LOCAL, &regs);
    if (retVal != pcie_RET_OK)
    {
        PCIE_logPrintf("Can't write legacy enable\n");
        exit(1);
    }

    /*   Construct Hwi object for this peripheral. */
    /*   Initialize with defaults. */
    Osal_RegisterInterrupt_initParams(&interruptRegParams);

    /* Populate the interrupt parameters */
    interruptRegParams.corepacConfig.arg = (uintptr_t)handle;
    interruptRegParams.corepacConfig.name = "PCIE_MSI";
    interruptRegParams.corepacConfig.isrRoutine = PlatformMsiIsr;
    interruptRegParams.corepacConfig.priority = 0x20U;
    interruptRegParams.corepacConfig.corepacEventNum = 0;               /* Only needed for c66 */
    interruptRegParams.corepacConfig.intVecNum = PCIE_SPI_BASE;
    interruptRegParams.corepacConfig.triggerSensitivity = OSAL_ARM_GIC_TRIG_TYPE_EDGE;

    Osal_RegisterInterrupt(&interruptRegParams, &pcieMsiHwi);

    if (! pcieMsiHwi)
    {
        PCIE_logPrintf("Hwi create (MSI) failed\n");
        exit(1);
    }

    /* Populate the interrupt parameters */
    interruptRegParams.corepacConfig.arg = (uintptr_t)handle;
    interruptRegParams.corepacConfig.name = "PCIE_INTX";
    interruptRegParams.corepacConfig.isrRoutine = PlatformIntxIsr;
    interruptRegParams.corepacConfig.priority = 0x20U;
    interruptRegParams.corepacConfig.corepacEventNum = 0;               /* Only needed for c66 */
#ifdef am65xx_idk
    interruptRegParams.corepacConfig.intVecNum = CSL_GIC0_INTR_PCIE0_BUS_PCIE0_PEND;
#else
    interruptRegParams.corepacConfig.intVecNum = CSL_GIC0_INTR_PCIE1_BUS_PCIE0_PEND;
#endif
    interruptRegParams.corepacConfig.triggerSensitivity = OSAL_ARM_GIC_TRIG_TYPE_EDGE;

    Osal_RegisterInterrupt(&interruptRegParams, &pcieIntxHwi);

    if (! pcieIntxHwi)
    {
        PCIE_logPrintf("Hwi create (INTX) failed\n");
        exit(1);
    }

    return semaphoreHandle;
}

void PlatformGetInts (uint32_t *msis, uint32_t *intx, uint32_t *unknowns)
{
    if (msis)
    {
        *msis = msi_ints;
    }
    if (intx)
    {
        *intx = intx_ints;
    }
    if (unknowns)
    {
        *unknowns = unknown_ints;
    }
}
#endif

/* Nothing past this point */

