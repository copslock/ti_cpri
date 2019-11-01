/**
 *  \file   sbl_avs_config.c
 *
 *  \brief  This file contains functions and data related to AVS and ABB
 *          configuration for the voltage rails.
 *
 */

/*
 * Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com/
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

#include "sbl_avs_config.h"
#include "sbl_prcm.h"

#include <ti/csl/cslr_device.h>
#include <ti/csl/hw_types.h>
#include <ti/board/board.h>
#include <ti/drv/pm/pmhal.h>
#if defined(idkAM572x)
#include <ti/board/src/idkAM572x/device/pmic_device.h>
#elif defined(idkAM571x)
#include <ti/board/src/idkAM571x/device/pmic_device.h>
#elif defined(evmAM572x)
#include <ti/board/src/evmAM572x/device/pmic_device.h>
#elif defined(idkAM574x)
#include <ti/board/src/idkAM574x/device/pmic_device.h>
#endif

/**********************************************************************
************************** Internal functions ************************
**********************************************************************/

/**
 * \brief    SBL_Read_Efuse_Reg function to read the voltage value from efuse
 *
 * \param    railVoltage pointer to the rail voltage.
 *
 * \retval   regVal  Value from the Efuse registers.
 *
 **/
static uint32_t SBL_Read_Efuse_Reg(const voltage_rail_t *railVoltage);

/**
 * \brief     SBL_GetOppConfig function to get the 
 *
 * \param     oppVal   value representing the OPP mode.
 * \param     boardId  Board ID value
 *
 * \retval    voltCoreData   pointer to the AVS data for the particular OPP.
 *
 **/
static const vcores_data_t* SBL_GetOppConfig(uint32_t oppVal, uint32_t boardId);

/**
 * \brief     Initialize the I2C controller by configuring the pinmux and
 *            module clocks.
 *
 * \param     oppVal   value representing the OPP mode.
 *
 **/
static void SBL_I2CInit();

/**
 * \brief     Function to resolve the Board Name string to BoardId.
 * 
 * \param     pBoardName  Pointer to the Board Name String.
 *
 **/
static uint32_t SBL_GetBoardid(char *pBoardName);

 /**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/

/* TPS659039 */
#define TPS659039_I2C_SLAVE_ADDR            0x58
#define TPS659039_REG_ADDR_SMPS12           0x23
#define TPS659039_REG_ADDR_SMPS45           0x2B
#define TPS659039_REG_ADDR_SMPS6            0x2F
#define TPS659039_REG_ADDR_SMPS7            0x33
#define TPS659039_REG_ADDR_SMPS8            0x37

#define TPS659039_PMIC_DEV_CTRL             0xA0

/* TPS659039 Voltage settings in mv for OPP_NOMINAL */
#define VDD_MPU_AM57XX      1100
#define VDD_DSP_AM57XX      1060
#define VDD_GPU_AM57XX      1060
#define VDD_CORE_AM57XX     1030
#define VDD_IVA_AM57XX      1060

/* Efuse register offsets for DRA7xx platform */
#define AM57XX_EFUSE_BASE                   0x4A002000U
#define AM57XX_EFUSE_REGBITS                16

/* STD_FUSE_OPP_VMIN_IVA_2 */
#define STD_FUSE_OPP_VMIN_IVA_NOM           (AM57XX_EFUSE_BASE + 0x05CC)
/* STD_FUSE_OPP_VMIN_IVA_3 */
#define STD_FUSE_OPP_VMIN_IVA_OD            (AM57XX_EFUSE_BASE + 0x05D0)
/* STD_FUSE_OPP_VMIN_IVA_4 */
#define STD_FUSE_OPP_VMIN_IVA_HIGH          (AM57XX_EFUSE_BASE + 0x05D4)
/* STD_FUSE_OPP_VMIN_DSPEVE_2 */
#define STD_FUSE_OPP_VMIN_DSPEVE_NOM        (AM57XX_EFUSE_BASE + 0x05E0)
/* STD_FUSE_OPP_VMIN_DSPEVE_3 */
#define STD_FUSE_OPP_VMIN_DSPEVE_OD         (AM57XX_EFUSE_BASE + 0x05E4)
/* STD_FUSE_OPP_VMIN_DSPEVE_4 */
#define STD_FUSE_OPP_VMIN_DSPEVE_HIGH       (AM57XX_EFUSE_BASE + 0x05E8)
/* STD_FUSE_OPP_VMIN_CORE_2 */
#define STD_FUSE_OPP_VMIN_CORE_NOM          (AM57XX_EFUSE_BASE + 0x05F4)
/* STD_FUSE_OPP_VMIN_GPU_2 */
#define STD_FUSE_OPP_VMIN_GPU_NOM           (AM57XX_EFUSE_BASE + 0x1B08)
/* STD_FUSE_OPP_VMIN_GPU_3 */
#define STD_FUSE_OPP_VMIN_GPU_OD            (AM57XX_EFUSE_BASE + 0x1B0C)
/* STD_FUSE_OPP_VMIN_GPU_4 */
#define STD_FUSE_OPP_VMIN_GPU_HIGH          (AM57XX_EFUSE_BASE + 0x1B10)
/* STD_FUSE_OPP_VMIN_MPU_2 */
#define STD_FUSE_OPP_VMIN_MPU_NOM           (AM57XX_EFUSE_BASE + 0x1B20)
/* STD_FUSE_OPP_VMIN_MPU_3 */
#define STD_FUSE_OPP_VMIN_MPU_OD            (AM57XX_EFUSE_BASE + 0x1B24)
/* STD_FUSE_OPP_VMIN_MPU_4 */
#define STD_FUSE_OPP_VMIN_MPU_HIGH          (AM57XX_EFUSE_BASE + 0x1B28)

#define CTRL_CORE_PAD_I2C1_SDA                      (0x400U)
#define CTRL_CORE_PAD_I2C1_SCL                      (0x404U)

#define CTRL_CORE_PAD_I2C1_SCL_PIN_PULLUP_EN        (0x00050000U)
#define CTRL_CORE_PAD_I2C1_SDA_PIN_PULLUP_EN        (0x00050000U)

#define BOARD_NAME_LENGTH                           (8)
#define OPP_TABLE_SIZE                              (3)

#if defined(idkAM572x) || defined(idkAM574x)
/* Structure to hold the AVS values of all voltage rails for OPP NOM */
const vcores_data_t idkAM572x_opp_nom_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    TPS659039_REG_ADDR_SMPS7,
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_DSP_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_DSPEVE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_GPU_AM57XX,
    TPS659039_REG_ADDR_SMPS6,
    {
        STD_FUSE_OPP_VMIN_GPU_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_IVA_AM57XX,
    TPS659039_REG_ADDR_SMPS8,
    {
        STD_FUSE_OPP_VMIN_IVA_NOM,
        AM57XX_EFUSE_REGBITS
    },
    }
};

/* Structure to hold the AVS values of all voltage rails for OPP OD */
const vcores_data_t idkAM572x_opp_od_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_OD,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    /* core does not support opp OD; using opp NOM */
    TPS659039_REG_ADDR_SMPS7,
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_DSP_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_DSPEVE_OD,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_GPU_AM57XX,
    TPS659039_REG_ADDR_SMPS6,
    {
        STD_FUSE_OPP_VMIN_GPU_OD,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_IVA_AM57XX,
    TPS659039_REG_ADDR_SMPS8,
    {
        STD_FUSE_OPP_VMIN_IVA_OD,
        AM57XX_EFUSE_REGBITS
    },
    }
};

/* Structure to hold the AVS values of all voltage rails for OPP HIGH */
const vcores_data_t idkAM572x_opp_high_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    TPS659039_REG_ADDR_SMPS7,
    /* core does not support opp high; using opp NOM */
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_DSP_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_DSPEVE_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_GPU_AM57XX,
    TPS659039_REG_ADDR_SMPS6,
    {
        STD_FUSE_OPP_VMIN_GPU_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_IVA_AM57XX,
    TPS659039_REG_ADDR_SMPS8,
    {
        STD_FUSE_OPP_VMIN_IVA_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    }
};
#endif

/* Structure defining the OPP table for AM572xIDK. */
boardOppData_t gAm572xIdkOppTable[] =
{
    #if defined(idkAM572x) || defined(idkAM574x)
    {OPP_MODE_NOM, &idkAM572x_opp_nom_volts},
    {OPP_MODE_OD, &idkAM572x_opp_od_volts},
    {OPP_MODE_HIGH, &idkAM572x_opp_high_volts}
    #endif
};

#if defined(idkAM571x)
/* Structure to hold the AVS values of all voltage rails for OPP NOM */
const vcores_data_t idkAM571x_opp_nom_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    TPS659039_REG_ADDR_SMPS7,
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    /* DSP rail not connected for idkAM571x */
    {
    0U,
    0U,
    {
        0U,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_GPU_AM57XX,
    TPS659039_REG_ADDR_SMPS6,
    {
        STD_FUSE_OPP_VMIN_GPU_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_IVA_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_IVA_NOM,
        AM57XX_EFUSE_REGBITS
    },
    }
};

/* Structure to hold the AVS values of all voltage rails for OPP OD */
const vcores_data_t idkAM571x_opp_od_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_OD,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    /* core does not support opp OD; using opp NOM */
    TPS659039_REG_ADDR_SMPS7,
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    /* DSP rail not connected for idkAM571x */
    {
    0U,
    0U,
    {
        0U,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_GPU_AM57XX,
    TPS659039_REG_ADDR_SMPS6,
    {
        STD_FUSE_OPP_VMIN_GPU_OD,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_IVA_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_IVA_OD,
        AM57XX_EFUSE_REGBITS
    },
    }
};

/* Structure to hold the AVS values of all voltage rails for OPP HIGH */
const vcores_data_t idkAM571x_opp_high_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    TPS659039_REG_ADDR_SMPS7,
    /* core does not support opp high; using opp NOM */
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    /* DSP rail not connected for idkAM571x */
    {
    0U,
    0U,
    {
        0U,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_GPU_AM57XX,
    TPS659039_REG_ADDR_SMPS6,
    {
        STD_FUSE_OPP_VMIN_GPU_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_IVA_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_IVA_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    }
};
#endif

/* Structure defining the OPP table for AM571xIDK. */
boardOppData_t gAm571xIdkOppTable[] =
{
    #if defined(idkAM571x)
    {OPP_MODE_NOM, &idkAM571x_opp_nom_volts},
    {OPP_MODE_OD, &idkAM571x_opp_od_volts},
    {OPP_MODE_HIGH, &idkAM571x_opp_high_volts}
    #endif
};

#if defined(evmAM572x)
/* Structure to hold the AVS values of all voltage rails for OPP NOM */
const vcores_data_t evmAM572x_opp_nom_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    TPS659039_REG_ADDR_SMPS6,
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_DSP_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_DSPEVE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    }
};

/* Structure to hold the AVS values of all voltage rails for OPP OD */
const vcores_data_t evmAM572x_opp_od_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_OD,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    /* core does not support opp OD; using opp NOM */
    TPS659039_REG_ADDR_SMPS6,
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_DSP_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_DSPEVE_OD,
        AM57XX_EFUSE_REGBITS
    },
    },

    /* GPU rail not connected for evmAM572x */
    {
    0U,
    0U,
    {
        0U,
        AM57XX_EFUSE_REGBITS
    },
    },

    /* IVA rail not connected for evmAM572x */
    {
    0U,
    0U,
    {
        0U,
        AM57XX_EFUSE_REGBITS
    },
    }
};

/* Structure to hold the AVS values of all voltage rails for OPP HIGH */
const vcores_data_t evmAM572x_opp_high_volts = {
    {
    VDD_MPU_AM57XX,
    TPS659039_REG_ADDR_SMPS12,
    {
        STD_FUSE_OPP_VMIN_MPU_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_CORE_AM57XX,
    TPS659039_REG_ADDR_SMPS6,
    /* core does not support opp high; using opp NOM */
    {
        STD_FUSE_OPP_VMIN_CORE_NOM,
        AM57XX_EFUSE_REGBITS
    },
    },

    {
    VDD_DSP_AM57XX,
    TPS659039_REG_ADDR_SMPS45,
    {
        STD_FUSE_OPP_VMIN_DSPEVE_HIGH,
        AM57XX_EFUSE_REGBITS
    },
    }
};

#endif
/* Structure defining the OPP table for GPEVM. */
boardOppData_t gAm572xGpevmOppTable[] =
{
    #if defined(evmAM572x)
    {OPP_MODE_NOM, &evmAM572x_opp_nom_volts},
    {OPP_MODE_OD, &evmAM572x_opp_od_volts},
    {OPP_MODE_HIGH, &evmAM572x_opp_high_volts}
    #endif
};

/** \brief Contains pointers to the OPP Table data for different boards. */
static boardOppData_t *gBoardOppTable [BOARD_MAX + 1U]= {
    NULL, /* BOARD_UNKNOWN */
    gAm572xGpevmOppTable, /* BOARD_GPEVMAM572x */
    gAm572xIdkOppTable, /* BOARD_IDKAM572x */
    gAm571xIdkOppTable, /* BOARD_IDKAM571x */
	gAm572xIdkOppTable, /* BOARD_IDKAM574x */
    NULL  /* BOARD Custom */
};

void SBL_Configure_AVS(uint32_t oppMode)
{
    uint32_t val;
    volatile uint32_t delayVar = 0U;
    uint32_t offset_code = 0U;
    const vcores_data_t *vcores = NULL;
    pmic_data_t *pPmicData;
    pmhalVmOppId_t pmOpp;
    Board_IDInfo boardInfo;
    uint32_t boardId;

    /*
    ** Initialize the I2C controller to communicate with PMIC device.
    ** This initialization includes setting up the clock and pinmux
    ** for I2C Bus.
    ** This is required as the AVS setting needs to be completed before
    ** IO calibration sequence is initiated.
    */
    SBL_I2CInit();

    /* Read the EEPROM to identify the board. */
    Board_getIDInfo(&boardInfo);

    /* Resolve the Board Name from EEPROM to a boardID. */
    boardId = SBL_GetBoardid(boardInfo.boardName);

    /* Get the OPP configuration data for the specific board. */
    vcores = SBL_GetOppConfig(oppMode, boardId);

    /* Get equivalent OPP macro in PM LLD */
    if (oppMode == OPP_MODE_NOM) pmOpp = PMHAL_VM_OPP_NOM;
    else if (oppMode == OPP_MODE_OD) pmOpp = PMHAL_VM_OPP_OD;
    else if (oppMode == OPP_MODE_HIGH) pmOpp = PMHAL_VM_OPP_HIGH;
    else pmOpp = PMHAL_VM_OPP_UNDEF;

    /* Get the Pointer to the PMIC data structure. */
    pPmicData = Board_GetPmicData();

    pPmicData->pmic_device_open(pPmicData->dev_instance - 1);

    if(BOARD_GPEVMAM572x == boardId)
    {
        /* Set the DEV_CTRL.DEV_ON to 1 to avoid turning off the PMIC for GPEVM */
        pPmicData->pmic_write(pPmicData->slave_addr, TPS659039_PMIC_DEV_CTRL, 0x01);
    }

    /* Configure AVS-0 voltage on CORE_VD */
    val = SBL_Read_Efuse_Reg(&vcores->core);
    offset_code = pPmicData->pmic_get_offset(val, pPmicData);
    /*PMIC I2C write */
    pPmicData->pmic_write(pPmicData->slave_addr, vcores->core.addr, offset_code);

    /* Configure the AVS voltage to MPU rail */
    val = SBL_Read_Efuse_Reg(&vcores->mpu);
    offset_code = pPmicData->pmic_get_offset(val, pPmicData);
    /*PMIC I2C write */
    pPmicData->pmic_write(pPmicData->slave_addr, vcores->mpu.addr, offset_code);
    PMHALVMEnableABB(PMHAL_PRCM_VD_MPU, pmOpp);

    /*
    ** Delay required after configuring 2 voltage rails
    */
    for (delayVar = 0; delayVar < 0x1000U; delayVar++) ;

    /*
    ** Check if the Voltage rail is Configured If yes then set the AVS value for
    ** the rail.
    */
    if(vcores->dsp.value != 0U)
    {
        /* Configure the AVS voltage to DSP rail */
        val = SBL_Read_Efuse_Reg(&vcores->dsp);
        offset_code = pPmicData->pmic_get_offset(val, pPmicData);
        /*PMIC I2C write */
        pPmicData->pmic_write(pPmicData->slave_addr, vcores->dsp.addr, offset_code);
        PMHALVMEnableABB(PMHAL_PRCM_VD_DSPEVE, pmOpp);
    }

    if(vcores->gpu.value != 0U)
    {
        /* Configure the AVS voltage to GPU rail */
        val = SBL_Read_Efuse_Reg(&vcores->gpu);
        offset_code = pPmicData->pmic_get_offset(val, pPmicData);
        /*PMIC I2C write */
        pPmicData->pmic_write(pPmicData->slave_addr, vcores->gpu.addr, offset_code);
        PMHALVMEnableABB(PMHAL_PRCM_VD_GPU, pmOpp);
    }

    if(vcores->iva.value != 0U)
    {
        /* Configure the AVS voltage to IVA rail */
        val = SBL_Read_Efuse_Reg(&vcores->iva);
        offset_code = pPmicData->pmic_get_offset(val, pPmicData);
        /*PMIC I2C write */
        pPmicData->pmic_write(pPmicData->slave_addr, vcores->iva.addr, offset_code);
        PMHALVMEnableABB(PMHAL_PRCM_VD_IVAHD, pmOpp);
    }

    /* Close the PMIC device. */
    pPmicData->pmic_device_close();
}

static const vcores_data_t* SBL_GetOppConfig(uint32_t oppVal, uint32_t boardId)
{
    uint32_t size = OPP_TABLE_SIZE;
    uint32_t count = 0U;
    const struct vcores_data *pOppCfg = NULL;
    boardOppData_t *pOppTable = NULL;

    /* Assign the pointer to the board specific OPP Table. */
    pOppTable = gBoardOppTable[boardId];

    while(count < size)
    {
        if(oppVal == (pOppTable + count)->oppVal)
        {
            pOppCfg = (pOppTable + count)->pboardOppData;
            break;
        }
        count++;
    }

    return pOppCfg;
}

static void SBL_I2CInit()
{
    /* Set the Clock operational mode for the clock domain. */
    SBL_PRCMSetClkOperMode(CSL_MPU_L4PER_CM_CORE_REGS, CM_L4PER_CLKSTCTRL,
        PRCM_CD_CLKTRNMODES_SW_WAKEUP);

    HW_WR_REG32(CSL_MPU_L4PER_CM_CORE_REGS + CM_L4PER_I2C1_CLKCTRL, 0x2U);

    /* Check for module enable status */
    while(2U !=
        (HW_RD_REG32(CSL_MPU_L4PER_CM_CORE_REGS + CM_L4PER_I2C1_CLKCTRL) & 3U));

    /* Check clock activity - ungated */
    while(CM_L4PER_CLKSTCTRL_CLKACTIVITY_L4PER_L3_GICLK_MASK !=
    (HW_RD_REG32(CSL_MPU_L4PER_CM_CORE_REGS + CM_L4PER_CLKSTCTRL) &
    CM_L4PER_CLKSTCTRL_CLKACTIVITY_L4PER_L3_GICLK_MASK));

    /* SDA */
    HW_WR_REG32((SOC_CORE_PAD_IO_REGISTERS_BASE + CTRL_CORE_PAD_I2C1_SDA),
            (CTRL_CORE_PAD_I2C1_SDA_PIN_PULLUP_EN));

    /* SCL */
    HW_WR_REG32((SOC_CORE_PAD_IO_REGISTERS_BASE + CTRL_CORE_PAD_I2C1_SCL),
            (CTRL_CORE_PAD_I2C1_SCL_PIN_PULLUP_EN));
}

static uint32_t SBL_Read_Efuse_Reg(const voltage_rail_t *railVoltage)
{
    uint32_t val;

    if (!railVoltage->value)
        return 0;
    if (!railVoltage->efuse.reg)
        return railVoltage->value;

    switch (railVoltage->efuse.reg_bits)
    {
        case 16:
            val = HW_RD_REG16(railVoltage->efuse.reg);
            break;
        case 32:
            val = HW_RD_REG32(railVoltage->efuse.reg);
            break;
        default:
            return railVoltage->value;
    }

    if (!val)
    {
        return railVoltage->value;
    }

    return val;
}

static uint32_t SBL_GetBoardid(char *pBoardName)
{
    uint32_t boardId;

    /* Check if the board is GPEVM by comparing the string read from EEPROM. */
    if (strncmp("AM572PM_", pBoardName, BOARD_NAME_LENGTH) == 0U)
    {
        boardId = BOARD_GPEVMAM572x;
    }
    /* Check if the board is AM572xIDK by comparing the string read from EEPROM. */
    else if (strncmp("AM572IDK", pBoardName, BOARD_NAME_LENGTH) == 0U)
    {
        boardId = BOARD_IDKAM572x;
    }
	/* Check if the board is AM574xIDK by comparing the string read from EEPROM. */
    else if (strncmp("AM574IDK", pBoardName, BOARD_NAME_LENGTH) == 0U)
    {
        boardId = BOARD_IDKAM574x;
    }
    /* Check if the board is AM571xIDK by comparing the string read from EEPROM. */
    else if (strncmp("AM571IDK", pBoardName, BOARD_NAME_LENGTH) == 0U)
    {
        boardId = BOARD_IDKAM571x;
    }
    else
    {
        /* If the board is not one of these, then the board
        ** ID is returned as BOARD_UNKNOWN.
        */
        boardId = BOARD_UNKNOWN;
    }

    return boardId;
}
