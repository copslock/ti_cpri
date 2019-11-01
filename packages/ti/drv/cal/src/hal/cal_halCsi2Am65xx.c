/* =============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2018
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
 *  \file cal_halCsi2Am65xx.c
 *
 *  \brief This file the function to initialize CSI2 PHY for TDA2ex platform
 *
 *  TODO / TBD
 *  TDA2EX_CAL_TODO i913 errata for CSI2 / CAL.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stddef.h>

#include <ti/csl/soc.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/cslr_cal.h>

#include <ti/drv/cal/cal.h>
#include <ti/drv/cal/src/hal/cal_hal.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**< Bit feilds definition for CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL reg */
#define CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL_CSI0_MODE_MASK                (0x00010000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL_CSI0_LANEENABLE_MASK          (0x0000001FU)
#define CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL_CSI0_CAMMODE_MASK             (0x03000000U)
#define CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL_CSI0_CTRLCLKEN_MASK           (0x00008000U)

/**< Bit 1 set, used to compute register values for lanes enabled. */
#define CSI0_X_LANE_ENABLED                ((uint32_t)0x1U)


/**< Functional PHY clock is expected to be 96 MHz
        Period of PHY clock in nS is 10.41666
        ((1/<PHY Clock) / 1e-9) */
#define DPHY_FUNCTIONAL_CLK_PERIOD  (10.41666)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                           Constants                                        */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Prototype                                */
/* ========================================================================== */

static void ConfigureCamRxCtrl(const Cal_CmplxIoCfg_t *pCmplxIoCfg,
                                uint32_t inst);
static void ConfigureStopStateTimeouts(uint32_t camRxCoreBaseAddr,
                                uint32_t csi2Clk, uint32_t inst);
/* ========================================================================== */
/*                          Function Implementations                          */
/* ========================================================================== */

int32_t Cal_halPhyEnClockAndReset(uint32_t            baseAddr,
                                     uint32_t            cam0RxCoreBaseAddr,
                                     uint32_t            cam1RxCoreBaseAddr,
                                     const Cal_HalInstCfg_t *pCfg)
{
    int32_t                rtnVal;
    uint32_t               idx, rxCoreBaseAddr;
    volatile uint32_t      reg, timeOut;

    GT_assert(CalTrace, (0U != baseAddr));
    GT_assert(CalTrace, (0U != cam0RxCoreBaseAddr));
    GT_assert(CalTrace, (NULL != pCfg));

    rtnVal = FVID2_SOK;
    /* Steps
     *  1. Configure D-PHY mode and enable required lanes
     *  2. Reset complex IO - Wait for completion of reset
     *          Note if the external sensor is not sending byte clock, the
     *          reset will timeout
     *  3 Program Stop States
     *      A. Program THS_TERM, THS_SETTLE, etc... Timings parameters in terms
     *              of DDR clock periods
     *      B. Enable stop state transition timeouts
     *  4.Force FORCERXMODE
     *      D. Enable pull down using pad control
     *      E. Power up PHY
     *      F. Wait for power up completion
     *      G. Wait for all enabled lane to reach stop state
     *      H. Disable pull down using pad control
     */
    for(idx = 0U; ((idx < pCfg->numCmplxIoInst) &&
                    (TRUE == pCfg->cmplxIoCfg[idx].enable)); idx++)
    {
        /* 1. Configure D-PHY mode and enable required lanes */
        ConfigureCamRxCtrl((const Cal_CmplxIoCfg_t *)
                            &pCfg->cmplxIoCfg[idx], idx);
        rxCoreBaseAddr = cam0RxCoreBaseAddr;
        if (idx > 0U)
        {
            rxCoreBaseAddr = cam1RxCoreBaseAddr;
            if (idx >= CSL_CAL_CMPLXIO_CNT)
            {
                GT_assert(CalTrace, ((uint32_t)FALSE));
            }
        }

/* C & C++ INVARIANT_CONDITION.GEN
 * Expression 'FVID2_SOK == rtnVal' used in the condition always yields the
 * same result.
 * KW State: Ignore -> Waiver -> Case by case
 * MISRAC_WAIVER:
 * In cases where value in the if condition  is dependent on the return of a
 * function and currently the function is hardcoded to return a value. Code is
 * currently unreachable but as the implementation of the function changes, it
 * will not be unreachable.
 */
        if((FVID2_SOK == rtnVal) && (0U != rxCoreBaseAddr))
        {
            /* 2. Reset complex IO - Do not wait for reset completion */
            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx));
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_RESET_CTRL_MASK;
            HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx), reg);
            /* Dummy read to allow SCP to complete */
            reg     = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx));
            timeOut = 0xFFFFFU;

            /* Timeouts */
            ConfigureStopStateTimeouts(rxCoreBaseAddr, pCfg->csi2PhyClock[idx],
                                        idx);

            /* B. Enable stop state transition timeouts */
            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_TIMING(idx));

            reg &= ~((uint32_t) (CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_X4_IO1_MASK |
                               CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_X16_IO1_MASK |
                               CSL_CAL_C2PPI_CSI2_TIMING_FORCE_RX_MODE_IO1_MASK |
                               CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_COUNTER_IO1_MASK));
            reg |= CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_X16_IO1_MASK &
                   (pCfg->ppiCfg[idx].csi2Cfg.stop_state_x16_I01 <<
                    CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_X16_IO1_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_X4_IO1_MASK &
                   (pCfg->ppiCfg[idx].csi2Cfg.stop_state_x4_I01 <<
                    CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_X4_IO1_SHIFT);
            reg |= CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_COUNTER_IO1_MASK &
                   (pCfg->ppiCfg[idx].csi2Cfg.stop_state_counter_I01 <<
                    CSL_CAL_C2PPI_CSI2_TIMING_STOP_STATE_COUNTER_IO1_SHIFT);
            HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_TIMING(idx), reg);

            /* 4 Force FORCERXMODE */
            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_TIMING(idx));
            reg |= CSL_CAL_C2PPI_CSI2_TIMING_FORCE_RX_MODE_IO1_MASK;
            HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_TIMING(idx), reg);

            /* E. Power up the PHY using the complex IO */
            reg  = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx));
            reg |= CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_PWR_CMD_MASK &
                   ((uint32_t) CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_PWR_CMD_STATE_ON <<
                             CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_PWR_CMD_SHIFT);
            HW_WR_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx), reg);

            /* F. Wait for power up completion */
            timeOut = 0xFFFFFFU;
            while(timeOut)
            {
                reg = HW_RD_REG32(baseAddr + CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG(idx));
                reg = (reg & CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_PWR_STATUS_MASK);
                reg = (reg >> CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_PWR_STATUS_SHIFT);
                if(CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_PWR_STATUS_STATE_ON == reg)
                {
                    break;
                }
                timeOut--;
            }
            if(0U == timeOut)
            {
                /* Could not power up the PHY... */
                rtnVal = FVID2_ETIMEOUT;
                GT_assert(CalTrace, ((uint32_t)FALSE));
                break;
            }
        }
    }

    return rtnVal;
}


/**************************Function Separator**********************************/

static void ConfigureCamRxCtrl(const Cal_CmplxIoCfg_t *pCmplxIoCfg,
                                uint32_t inst)
{
    volatile uint32_t reg, enabledLanes;
    uint32_t modeMask, camModeMask, ctrlClkEnMask, laneEnMask;
    uint32_t laneEn;

    modeMask =  CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL_CSI0_MODE_MASK;
    laneEnMask = CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL_CSI0_LANEENABLE_MASK;
    camModeMask = CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL_CSI0_CAMMODE_MASK;
    ctrlClkEnMask = CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL_CSI0_CTRLCLKEN_MASK;
    laneEn = CSI0_X_LANE_ENABLED;

    /* Unlock SOC main control mmr partition 1 for write access */
    HW_WR_REG32(CSL_CTRL_MMR0_CFG0_BASE +
                      CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK0, 0x68EF3490U);
    HW_WR_REG32(CSL_CTRL_MMR0_CFG0_BASE +
                      CSL_MAIN_CTRL_MMR_CFG0_LOCK1_KICK1, 0xD172BC5AU);

    reg = HW_RD_REG32(CSL_CTRL_MMR0_CFG0_BASE +
                      CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL);
    reg &= ~((uint32_t) (modeMask | laneEnMask | camModeMask | ctrlClkEnMask));

    /* Disable Off mode */
    reg &= ~modeMask;

    /* Enable Required lanes */
    enabledLanes = 0x0;
    if((uint32_t)TRUE == pCmplxIoCfg->enable)
    {
        /* All lane modules requires to be enabled, when operting with 1, 2
            3 lane mode */
        enabledLanes = (enabledLanes | (laneEn << 0U));
        enabledLanes = (enabledLanes | (laneEn << 1U));
        enabledLanes = (enabledLanes | (laneEn << 2U));
        enabledLanes = (enabledLanes | (laneEn << 3U));
        enabledLanes = (enabledLanes | (laneEn << 4U));
        reg |= (laneEnMask & enabledLanes);
    }

    /* Enable DPHY Mode */
    reg &= ~camModeMask;

    /* Enable CTRL CLK */
    reg |= ctrlClkEnMask;

    HW_WR_REG32(CSL_CTRL_MMR0_CFG0_BASE +
                CSL_MAIN_CTRL_MMR_CFG0_CAL0_CTRL, reg);
}

/**************************Function Separator**********************************/

static void ConfigureStopStateTimeouts(uint32_t camRxCoreBaseAddr,
                                        uint32_t csi2Clk, uint32_t inst)
{
    volatile uint32_t reg;
    uint32_t thsTerm, thsSettle, tclkTerm, tclkSettle;
    Float32  ddrClkPeriod, temp;

    /* A. Setup the timings parameters in terms of DDR &
            PHY functional clock */
    reg  = HW_RD_REG32(camRxCoreBaseAddr + CAL_CSI2_PHY_REG0);
    reg &= ~((uint32_t) (CAL_CSI2_PHY_REG0_THS_TERM_MASK |
                       CAL_CSI2_PHY_REG0_THS_SETTLE_MASK));

    /* Effective time for enabling of termination = synchronizer delay
        + timer delay + LPRX delay + combinational routing delay
        THS_TERM = Floor (20/DDR Clock) */

    /* Get DDR clock period in nS */
    ddrClkPeriod = (Float32)((Float32)csi2Clk * (Float32)1000000U);
    ddrClkPeriod = (Float32)((Float32)1U / ddrClkPeriod);
    ddrClkPeriod = (Float32)(ddrClkPeriod * (Float32)1000000000U);
    GT_assert(CalTrace, ((uint32_t)ddrClkPeriod > 0U));
    temp    = ((Float32)20U / ddrClkPeriod);
    thsTerm = (uint32_t)temp;

    /* Effective Ths-settle seen on line
        (starting to look for sync pattern) THS_SETTLE =
        synchronizer delay + timer delay + LPRX delay + combinational
        routing delay - pipeline delay in HS data path.
        (105 / DDR Clock) + 4
        */
    temp = (Float32)(((Float32)105 / ddrClkPeriod) + (Float32)4U);
    thsSettle = (uint32_t)temp;

    reg |= thsTerm << CAL_CSI2_PHY_REG0_THS_TERM_SHIFT;

    reg |= thsSettle << CAL_CSI2_PHY_REG0_THS_SETTLE_SHIFT;
    reg |= CAL_CSI2_PHY_REG0_HSCLOCKCONFIG_MASK;
    HW_WR_REG32(camRxCoreBaseAddr + CAL_CSI2_PHY_REG0, reg);

    reg  = HW_RD_REG32(camRxCoreBaseAddr + CAL_CSI2_PHY_REG1);
    reg &= ~(CAL_CSI2_PHY_REG1_TCLK_TERM_MASK |
             CAL_CSI2_PHY_REG1_TCLK_SETTLE_MASK);
    /* Requirement from D-PHY spec= (Dn Voltage < 450 mV) - 38 ns
        Effective time for enabling of termination (TCLK_TERM) =
            synchronizer delay + timer delay + LPRX delay +
                combinational routing delay
        TCLK_TERM = 23 / Functional clock period - 2
    */
    temp = (((Float32)23U / (Float32)DPHY_FUNCTIONAL_CLK_PERIOD) - (Float32)2U);
    tclkTerm = (uint32_t)temp;

    /* Derived requirement from D-PHY spec = 100 ns to 300ns.
        Effective Ths-settle = synchronizer delay + timer delay +
            LPRX delay + combinational routing delay

        Average of minimum delay + maximum delay / 2

        TCLK_SETTLE = floor(260/<Functional PHY Clock in nS>) - 2
    */
    temp = (((Float32)260U /
             (Float32)DPHY_FUNCTIONAL_CLK_PERIOD) - (Float32)2U);
    tclkSettle = (uint32_t)temp;

    reg |= CAL_CSI2_PHY_REG1_TCLK_TERM_MASK &
           (tclkTerm << CAL_CSI2_PHY_REG1_TCLK_TERM_SHIFT);

    reg |= CAL_CSI2_PHY_REG1_TCLK_SETTLE_MASK &
           (tclkSettle << CAL_CSI2_PHY_REG1_TCLK_SETTLE_SHIFT);

    HW_WR_REG32(camRxCoreBaseAddr + CAL_CSI2_PHY_REG1, reg);
}

