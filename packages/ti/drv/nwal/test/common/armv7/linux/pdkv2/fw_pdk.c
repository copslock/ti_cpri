/******************************************************************************
 * FILE PURPOSE:  Functions to initialize framework resources for running NWAL
 ******************************************************************************
 * FILE NAME:   fw_soc.c
 *
 * DESCRIPTION: Functions to initialize framework resources for running NWAL
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2013
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
#include "fw_test.h"
/* CSL RL includes */
#include <stdlib.h>
#include <stdio.h>
#include <ti/drv/qmss/qmss_qm.h>
//#include <ti/drv/qmss/device/qmss_device.c>
//#include <ti/drv/cppi/device/cppi_device.c>

#if defined(DEVICE_K2H) || defined(SOC_K2H)
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (DEVICE_K2K) || defined(SOC_K2K)
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (DEVICE_K2L) || defined(SOC_K2L)
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#elif defined (DEVICE_K2E) || defined(SOC_K2E)
#include <ti/drv/qmss/device/k2e/src/qmss_device.c>
#include <ti/drv/cppi/device/k2e/src/cppi_device.c>
#else /*Default */
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#endif /* Device */


/* Global variables to hold virtual address of various subsystems */
extern void *fw_qmssCfgVaddr;
extern void *fw_qmssDataVaddr;
extern void *fw_srioCfgVaddr;
extern void *fw_netcpCfgVaddr;
extern Rm_ServiceHandle   *rmClientServiceHandle;


/******************************************************************************
* Macro to convert to IP Register Virtual Address from a mapped base Virtual Address
* Input: virtBaseAddr: Virtual base address mapped using mmap for IP
*        phyBaseAddr: Physical base address for the IP
*        phyRegAddr:  Physical register address
******************************************************************************/
static inline void* FW_GET_REG_VADDR (void * virtBaseAddr, 
                                      uint32_t phyBaseAddr, 
                                      uint32_t phyRegAddr)
{
    return((void *)((uint8_t *)virtBaseAddr + (phyRegAddr - phyBaseAddr)));
}
/*****************************************************************************
 * FUNCTION PURPOSE: Global Initialization of CPPI. Once Per System
 *****************************************************************************
 * DESCRIPTION: The function will initialize the CPPI
 *****************************************************************************/
nwal_Bool_t testNwGlobCppiInit (void)
{
  int32_t result, i;
  Cppi_GlobalConfigParams     fw_cppiGblCfgParams[CPPI_MAX_CPDMA];
  Cppi_StartCfg               fw_cppiStartCfg;

  for (i=0; i<CPPI_MAX_CPDMA; i++)
    fw_cppiGblCfgParams[i] = cppiGblCfgParams[i];

  /* PASS CPDMA regs */
  fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].gblCfgRegs =
        FW_GET_REG_VADDR(fw_netcpCfgVaddr,
                         CSL_NETCP_CFG_REGS,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].gblCfgRegs);

  fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].txChRegs =
        FW_GET_REG_VADDR(fw_netcpCfgVaddr,
                         CSL_NETCP_CFG_REGS,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].txChRegs);

  fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].rxChRegs =
        FW_GET_REG_VADDR(fw_netcpCfgVaddr,
                         CSL_NETCP_CFG_REGS,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].rxChRegs);

  fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].txSchedRegs =
        FW_GET_REG_VADDR(fw_netcpCfgVaddr,
                         CSL_NETCP_CFG_REGS,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].txSchedRegs);

  fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].rxFlowRegs =
        FW_GET_REG_VADDR(fw_netcpCfgVaddr,
                         CSL_NETCP_CFG_REGS,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_PASS_CPDMA].rxFlowRegs);

 /* QMSS CPDMA regs */
  fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].gblCfgRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].gblCfgRegs);

  fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].txChRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].txChRegs);

  fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].rxChRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].rxChRegs);

  fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].txSchedRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].txSchedRegs);

  fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].rxFlowRegs =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_cppiGblCfgParams[Cppi_CpDma_QMSS_CPDMA].rxFlowRegs);

  result = Cppi_init (fw_cppiGblCfgParams);
  if (result != CPPI_SOK)
  {
    printf (">function cppi_init: Cppi_init failed with error code %d\n", result);
    return (-1);
  }
  if (rmClientServiceHandle)
  {
    fw_cppiStartCfg.rmServiceHandle = rmClientServiceHandle;
    Cppi_startCfg(&fw_cppiStartCfg);
  }
   return 1;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Global Initialization of Queue Manager. Once Per System
 *****************************************************************************
 * DESCRIPTION: The function will initialize the Queue Manager
 *****************************************************************************/
nwal_Bool_t testNwGlobQmInit (void)
{
    Qmss_InitCfg              qmssInitConfig;
    int32_t                   result;
    Qmss_GlobalConfigParams   fw_qmssGblCfgParams;


    memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Use Internal Linking RAM for optimal performance */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = TEST_CONFIG_MAX_DESC_NUM;
    qmssInitConfig.qmssHwStatus = QMSS_HW_INIT_COMPLETE;

  fw_qmssGblCfgParams = qmssGblCfgParams[0];

  fw_qmssGblCfgParams.qmConfigReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmConfigReg);

  fw_qmssGblCfgParams.qmDescReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmDescReg);

  fw_qmssGblCfgParams.qmQueMgmtReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmQueMgmtReg);

  fw_qmssGblCfgParams.qmQueMgmtProxyReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmQueMgmtProxyReg);

  fw_qmssGblCfgParams.qmQueStatReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmQueStatReg);

  fw_qmssGblCfgParams.qmQueIntdReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmQueIntdReg);

  fw_qmssGblCfgParams.qmPdspCmdReg[0] =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmPdspCmdReg[0]);

  fw_qmssGblCfgParams.qmPdspCmdReg[1] =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmPdspCmdReg[1]);

  fw_qmssGblCfgParams.qmPdspCtrlReg[0] =
    FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmPdspCtrlReg[0]);

  fw_qmssGblCfgParams.qmPdspCtrlReg[1] =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmPdspCtrlReg[1]);

  fw_qmssGblCfgParams.qmPdspIRamReg[0] =
    FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmPdspIRamReg[0]);

  fw_qmssGblCfgParams.qmPdspIRamReg[1] =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmPdspIRamReg[1]);

  fw_qmssGblCfgParams.qmStatusRAM =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmStatusRAM);
  fw_qmssGblCfgParams.qmLinkingRAMReg =
    FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmLinkingRAMReg);
  fw_qmssGblCfgParams.qmMcDMAReg =
            FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmMcDMAReg);

  fw_qmssGblCfgParams.qmTimer16Reg[0] =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmTimer16Reg[0]);

  fw_qmssGblCfgParams.qmTimer16Reg[1] =
        FW_GET_REG_VADDR(fw_qmssCfgVaddr,
                         QMSS_CFG_BASE_ADDR,
                         (uint32_t)fw_qmssGblCfgParams.qmTimer16Reg[1]);

  fw_qmssGblCfgParams.qmQueMgmtDataReg = (void *)((uint32_t)fw_qmssDataVaddr); 
  fw_qmssGblCfgParams.qmQueMgmtProxyDataReg =NULL;
  if (rmClientServiceHandle)
  {
      fw_qmssGblCfgParams.qmRmServiceHandle = rmClientServiceHandle;
  }
  result = Qmss_init (&qmssInitConfig, &fw_qmssGblCfgParams);
    if (result != QMSS_SOK)  
    {
        System_printf ("function testNwGlobQmInit: qmss_Init failed with error code %d\n", result);
        return (nwal_FALSE);
    }
    return (nwal_TRUE);
}

/***************************************************************************************
 * FUNCTION PURPOSE: Do switch related initializations
 ***************************************************************************************
 * DESCRIPTION: Add switch related initializations
 ***************************************************************************************/
void testNwCPSWInit(nwal_Bool_t enableALE)
{  
    /* STUB Function */
}

/***************************************************************************************
 * FUNCTION PURPOSE: Add MAC Address to the ALE for the packet being routed from network
 ***************************************************************************************
 * DESCRIPTION: Add MAC Address to the ALE for the packet being routed from network
 ***************************************************************************************/
nwal_Bool_t testNwSwUpdateMacAddr(uint8_t macAddress[6])
{  
    /* STUB Function */

    /* Done with upading address */
    return 1;
}
