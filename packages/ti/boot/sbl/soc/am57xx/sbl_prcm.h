/**
 *  \file  sbl_prcm.h
 *
 * \brief  This file contains the function prototypes of SBL prcm APIS.
 *
 */

/*
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SBL_PRCM_H_
#define SBL_PRCM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PRCM_MODULEMODE_DISABLE          (0U)
#define PRCM_MODULEMODE_ENABLE           (2U)
#define PRCM_MODULEMODE_AUTO             (1U)
#define PRCM_MODULEMODE_MASK             (3U)
#define PRCM_IDLE_ST_MASK                (0x00030000U)
#define PRCM_IDLE_ST_SHIFT               (16U)

typedef enum prcmCdClkTrnModes
{
    PRCM_CD_CLKTRNMODES_MIN = 0U,
    /**< Min value of the enum. Can be used for validation. */
    PRCM_CD_CLKTRNMODES_NO_SLEEP = PRCM_CD_CLKTRNMODES_MIN,
    /**< No sleep in the clock domain */
    PRCM_CD_CLKTRNMODES_SW_SLEEP = 1U,
    /**< Start a software forced sleep transition on the clock domain. */
    PRCM_CD_CLKTRNMODES_SW_WAKEUP = 2U,
    /**< Start a software forced wake-up transition on the clock domain. */
    PRCM_CD_CLKTRNMODES_HW_AUTO = 3U,
    /**< If there is no activity in the clock domain HW automatically puts it
     *   into sleep mode. On detecting any activity it wakes up. */
    PRCM_CD_CLKTRNMODES_MAX = PRCM_CD_CLKTRNMODES_HW_AUTO
	/**< Max value of the enum. Can be used for
	 *   validation. */
} prcmCdClkTrnModes_t;

/**
 * \brief This API Enables the clock for a module.
 *
 *  \param  domainOffset     Clock domain offset
 *  \param  clkCtrlReg       Clock control register
 *  \param  clkStCtrlReg     Clock ST control register
 *  \param  clkActMask       Clock activity mask value
 */
void SBL_PRCMModuleEnable(uint32_t domainOffset,
                          uint32_t clkCtrlReg,
                          uint32_t clkStCtrlReg,
                          uint32_t clkActMask);

/**
 * \brief This API Disables the clock for a module.
 *
 *  \param  domainOffset     Clock domain offset
 *  \param  clkCtrlReg       Clock control register
 *  \param  clkStCtrlReg     Clock ST control register
 */
void SBL_PRCMModuleDisable(uint32_t domainOffset,
                           uint32_t clkCtrlReg,
                           uint32_t clkStCtrlReg);

/**
 * \brief This API sets the CM clock operating modes.
 *
 * \param   domainOffset   Clock domain offset
 * \param   clkStrCtrlReg  Clock ST control register
 * \param   clkMode        Mode at which the CD is desired to operate.
 *                         Refer enum #prcmCdClkTrnModes_t for valid modes.
 */
void SBL_PRCMSetClkOperMode(uint32_t domainOffset,
                            uint32_t clkStCtrlReg,
                            uint32_t clkMode);

/**
 * \brief This API releases reset for the given reset domain.
 *
 * \param   ctrlRegAddr     Address of the clock ctrl register
 * \param   ctrlRegShift    Shift value for reset for control register
 * \param   stsRegAddr      Address of the status register
 * \param   stsRegShift     Shift value for reset for Status register
 */
void SBL_ResetRelease(uint32_t ctrlRegAddr,
                      uint32_t ctrlregShift,
                      uint32_t stsRegAddr,
                      uint32_t stsRegShift);

/**
 * \brief    This API asserts reset for the given reset domain.
 *
 * \param    stsRegAddr      Address of the status register
 * \param    stsRegShift     Shift value for reset in Status register
 */
void SBL_ResetAssert(uint32_t ctrlRegAddr, uint32_t ctrlregShift);

/**
 * \brief    This API Gets the Reset Status.
 *
 * \param    stsRegAddr     Address of the status register
 * \param    stsRegShift    Shift value for reset for Status register
 * \param    rStsMask       Pointer to a variable to return the status value.
 */
void SBL_ResetGetStatus(uint32_t stsRegAddr,
                        uint32_t stsRegShift,
                        uint32_t *rStsMask);

/**
 * \brief    This API Clears the Reset Status.
 *
 * \param    stsRegAddr     Address of the status register
 * \param    stsRegShift    Shift value for reset in Status register
 *
 */
void SBL_ResetClearStatus(uint32_t stsRegAddr,
                          uint32_t stsRegShift);

#ifdef __cplusplus
}
#endif

#endif
