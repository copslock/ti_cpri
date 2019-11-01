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

/**
 *  \file udma_apputils.c
 *
 *  \brief Common UDMA application utility used in all UDMA example.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/udma/udma.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/udma/examples/udma_apputils/udma_apputils.h>
#if defined (__C7100__)
#include <ti/csl/csl_clec.h>
#include <ti/csl/arch/csl_arch.h>
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Udma_appUtilsCacheWb(const void *addr, int32_t size)
{
    uint32_t    isCacheCoherent = Udma_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        CacheP_wb(addr, size);
    }

    return;
}

void Udma_appUtilsCacheInv(const void * addr, int32_t size)
{
    uint32_t    isCacheCoherent = Udma_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        CacheP_Inv(addr, size);
    }

    return;
}

void Udma_appUtilsCacheWbInv(const void * addr, int32_t size)
{
    uint32_t    isCacheCoherent = Udma_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        CacheP_wbInv(addr, size);
    }

    return;
}

uint64_t Udma_appVirtToPhyFxn(const void *virtAddr, uint32_t chNum, void *appData)
{
    uint64_t    phyAddr;

    phyAddr = (uint64_t) virtAddr;
#if defined (BUILD_C66X_1) || defined (BUILD_C66X_2)
    /* Convert local L2RAM address to global space */
    if((phyAddr >= CSL_C66_COREPAC_L2_BASE) &&
       (phyAddr < (CSL_C66_COREPAC_L2_BASE + CSL_C66_COREPAC_L2_SIZE)))
    {
#if defined (BUILD_C66X_1)
        phyAddr -= CSL_C66_COREPAC_L2_BASE;
        phyAddr += CSL_C66SS0_C66_SDMA_L2SRAM_0_BASE;
#endif
#if defined (BUILD_C66X_2)
        phyAddr -= CSL_C66_COREPAC_L2_BASE;
        phyAddr += CSL_C66SS1_C66_SDMA_L2SRAM_0_BASE;
#endif
    }
#endif

    return (phyAddr);
}

void *Udma_appPhyToVirtFxn(uint64_t phyAddr, uint32_t chNum, void *appData)
{
    void       *virtAddr;

#if defined (__aarch64__) || defined (__C7100__)
    virtAddr = (void *) phyAddr;
#else
    uint32_t temp;

    /* Convert global L2RAM address to local space */
#if defined (BUILD_C66X_1)
    if((phyAddr >= CSL_C66SS0_C66_SDMA_L2SRAM_0_BASE) &&
       (phyAddr < (CSL_C66SS0_C66_SDMA_L2SRAM_0_BASE + CSL_C66_COREPAC_L2_SIZE)))
    {
        phyAddr -= CSL_C66SS0_C66_SDMA_L2SRAM_0_BASE;
        phyAddr += CSL_C66_COREPAC_L2_BASE;
    }
#endif
#if defined (BUILD_C66X_2)
    if((phyAddr >= CSL_C66SS1_C66_SDMA_L2SRAM_0_BASE) &&
       (phyAddr < (CSL_C66SS1_C66_SDMA_L2SRAM_0_BASE + CSL_C66_COREPAC_L2_SIZE)))
    {
        phyAddr -= CSL_C66SS1_C66_SDMA_L2SRAM_0_BASE;
        phyAddr += CSL_C66_COREPAC_L2_BASE;
    }
#endif

    /* R5/C66x is 32-bit; need to truncate to avoid void * typecast error */
    temp = (uint32_t) phyAddr;
    virtAddr = (void *) temp;
#endif

    return (virtAddr);
}

uint32_t Udma_appIsPrintSupported(void)
{
    uint32_t retVal = TRUE;

    /* Semi hosting not supported for MPU on Simulator */
#if (defined (BUILD_MPU) && defined (SIMULATOR))
    retVal = FALSE;
#endif

    /* Printf doesn't work for MPU when run from SBL with no CCS connection
     * There is no flag to detect SBL or CCS mode. Hence disable the print
     * for MPU unconditionally */
#if defined (BUILD_MPU)
    retVal = FALSE;
#endif

    return (retVal);
}

void Udma_appC66xIntrConfig(void)
{
#if defined (_TMS320C6X)
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    /* On C66x builds we define OS timer tick in the configuration file to
     * trigger event #21 for C66x_1 and #20 for C66x_2. Map
     * DMTimer 0 interrupt to these events through DMSC RM API.
     */
    rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_DST_ID_VALID |
                                      TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
    rmIrqReq.src_id                 = TISCI_DEV_TIMER0;
    rmIrqReq.src_index              = 0U;
#if defined (BUILD_C66X_1)
    rmIrqReq.dst_id                 = TISCI_DEV_C66SS0_CORE0;
    rmIrqReq.dst_host_irq           = 21U;
#endif
#if defined (BUILD_C66X_2)
    rmIrqReq.dst_id                 = TISCI_DEV_C66SS1_CORE0;
    rmIrqReq.dst_host_irq           = 20U;
#endif
    /* Unused params */
    rmIrqReq.global_event           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    Sciclient_rmIrqSet(&rmIrqReq, &rmIrqResp, UDMA_SCICLIENT_TIMEOUT);
#endif

    return;
}

void Udma_appC7xPreInit(void)
{
#if defined (__C7100__)
    CSL_ClecEventConfig cfgClec;
    CSL_CLEC_EVTRegs   *clecBaseAddr = (CSL_CLEC_EVTRegs*) CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE;
    uint32_t            i, maxInputs = 2048U;

    /* make secure claim bit to FALSE so that after we switch to non-secure mode
     * we can program the CLEC MMRs
     */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = FALSE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_DISABLE;
    cfgClec.extEvtNum         = 0U;
    cfgClec.c7xEvtNum         = 0U;
    for(i = 0U; i < maxInputs; i++)
    {
        CSL_clecConfigEvent(clecBaseAddr, i, &cfgClec);
    }

    i = CSLR_COMPUTE_CLUSTER0_GIC500SS_SPI_TIMER0_INTR_PEND_0 + 992;  /* Used for Timer Interrupt */
    /* Configure CLEC for DMTimer0, SYS/BIOS uses interrupt 14 for DMTimer0 by default */
    cfgClec.secureClaimEnable = FALSE;
    cfgClec.evtSendEnable     = TRUE;
    cfgClec.rtMap             = CSL_CLEC_RTMAP_CPU_ALL;
    cfgClec.extEvtNum         = 0;
    cfgClec.c7xEvtNum         = 14;
    CSL_clecConfigEvent(clecBaseAddr, i, &cfgClec);

    /* Switch now */
    CSL_c7xSecSupv2NonSecSupv();
#endif

    return;
}

uint32_t Udma_appIsUdmapStatsSupported(void)
{
    uint32_t retVal = TRUE;

#if defined (SOC_AM65XX)
    uint32_t jtagIdVal, variatn;

    jtagIdVal = CSL_REG32_RD(CSL_WKUP_CTRL_MMR0_CFG0_BASE + CSL_WKUP_CTRL_MMR_CFG0_JTAGID);
    variatn   = CSL_FEXT(jtagIdVal, WKUP_CTRL_MMR_CFG0_JTAGID_VARIATN);
    if (variatn == 0U)
    {
        retVal = FALSE;
    }
#endif

    return (retVal);
}
