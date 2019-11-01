/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <sbl_lib.h>
#include <ti/csl/cslr_device.h>
#include <sbl_eveloader.h>
#include "mailbox.h"


#define MAX_NUM_EVES               (4)

extern unsigned char gAM57XX_EVE_FIRMWARE;

/*
 * Variable for version 1 of Meta Header structure
 */
sbllibMetaHeaderV1_t    gUtilsAppMetaHeaderV1;

/*
 * Variable for RPRC Header structure
 */
sbllibRPRCImageHeader_t gUtilsAppRPRCHeader;

/*
 * Variable for Entry points
 */
sbllibEntryPoints_t gUtilsEntryPoints =
{{0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}};

static int  GetNumEVEDevices();

/**
 * \brief   This function copies data from DDR
 *
 * \param   dstAddr       Destination Address
 * \param   srcOffsetAddr NOR Source Offset Address
 * \param   length        The length of data block to be copied.
 *
 * \return  status        Whether copy is done successfully.
 */
int32_t ddr3ReadLocal(void    *dstAddr,
                          uint32_t srcOffsetAddr,
                          uint32_t length)
{
    memcpy((uint32_t *)dstAddr, (uint32_t *)srcOffsetAddr, length);

    return 0;
}

/**
 * \brief   This function moves the read head by n bytes.
 *
 * \param   srcAddr     Read head pointer.
 * \param   numBytes    Number of bytes of data by which read head is moved.
 *
 * \return  None
 */
void ddr3Seek(uint32_t *srcAddr, uint32_t numBytes)
{
    *(srcAddr) = numBytes;
}

/**
 * \brief   This is a dummy DDR copy function. TDA2xx SOC family doesn't have
 *          CRC feature and hence data is directly read from boot media.
 *
 * \param   dstAddr   Destination Address
 * \param   srcAddr   DDR Source Address
 * \param   length    The length of data block to be copied.
 *
 * \return  status    Whether data is copied correctly
 */
int32_t dummyDDRRead(void   *dstAddr,
                         uint32_t  srcAddr,
                         uint32_t  length)
{
    int32_t retVal = 0;

    /* This is dummy function */
    return retVal;
}


int main()
{
    uint32_t msg = 'D';
    int32_t retval = 2;
	uint32_t sblBuildMode = SBLLIB_SBL_BUILD_MODE_PROD;

    /* Wait for message from MPU */
    while (MESSAGE_INVALID ==
           MailboxGetMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_0, &msg)) ;

    retval = eveBoot(GetNumEVEDevices());

    if (SYSTEM_LINK_STATUS_SOK == retval)
    {
        /* Send ack message to MPU */
        MailboxSendMessage(CSL_MPU_MAILBOX3_REGS, MAILBOX_QUEUE_0, 0xAA55FF00);
    }
	
#ifdef PROC_EVE1_INCLUDE
    if (GetNumEVEDevices() >= 1)
        SBLLibEVE1BringUp(gUtilsEntryPoints.entryPoint[SBLLIB_CORE_ID_EVE1],
                          sblBuildMode);
#endif
#ifdef PROC_EVE2_INCLUDE
    if (GetNumEVEDevices() >= 2)
        SBLLibEVE2BringUp(gUtilsEntryPoints.entryPoint[SBLLIB_CORE_ID_EVE2],
                          sblBuildMode);
#endif

}

int32_t eveBoot( int32_t num_eve_devices)
{
    int32_t retVal = SYSTEM_LINK_STATUS_SOK;
    uint32_t oppId = SBLLIB_PRCM_DPLL_OPP_NOM;
    
    pmhalPrcmDpllConfig_t      *dpllParams;
    pmhalPrcmSysClkVal_t sysClkFreq = PMHALCMGetSysClockFreqEnum();
    sbllibAppImageParseParams_t appImgParams;


	sbllibInitParams_t sblInitPrms;
	
    SBLLibInitParamsInit(&sblInitPrms);

    SBLLibInit(&sblInitPrms);
	
    /* Configure DPLL EVE */
    retVal = SBLLibGetDpllStructure(PMHAL_PRCM_DPLL_EVE,
                                     sysClkFreq,
                                     oppId,
                                     &dpllParams);

    retVal += PMHALCMDpllConfigure(PMHAL_PRCM_DPLL_EVE,
                                   dpllParams,
                                   PM_TIMEOUT_INFINITE);
	
    resetAllEVECores(num_eve_devices);
	
     /* Initialize App Image Params */
    SBLLibAppImageParamsInit(&appImgParams);
    appImgParams.appImgMetaHeaderV1 = &gUtilsAppMetaHeaderV1;
    appImgParams.appImgRPRCHeader   = &gUtilsAppRPRCHeader;
    appImgParams.entryPoints        = &gUtilsEntryPoints;
    appImgParams.skipDDRCopy        = 1U;

    loadAppImage(&appImgParams);

    return retVal;	
}

#undef  CTRL_WKUP_STD_FUSE_DIE_ID_2
#define CTRL_WKUP_STD_FUSE_DIE_ID_2  0x4AE0C20C
static int  GetNumEVEDevices()
{
    uint32_t board_type = (  *((uint32_t *) CTRL_WKUP_STD_FUSE_DIE_ID_2)
                           & 0xFF000000) >> 24;
    int      num_eves = 0;
    if (     board_type == 0x3E ||  /* AM5729-E (EtherCat)*/
             board_type == 0x4E)    /* AM5729 */
        num_eves = 4;
    else if (board_type == 0x5F ||  /* AM5749-E (EtherCat) */
             board_type == 0xA6 ||  /*  AM5749 */
             board_type == 0x69)    /*  AM5749IDK (shown on package sticker) */
                                    /* (data sheet: 0x69 is Jacinto 6 Plus) */
        num_eves = 2;

    if (num_eves > MAX_NUM_EVES)
    {
        num_eves = MAX_NUM_EVES;
    }
    return num_eves;
}

int32_t loadAppImage(sbllibAppImageParseParams_t *imageParams)
{
    imageParams->appImageOffset = (uint32_t) &gAM57XX_EVE_FIRMWARE;

    SBLLibRegisterImageCopyCallback(&ddr3ReadLocal,
                                    &dummyDDRRead,
                                    &ddr3Seek);

    return (SBLLibMultiCoreImageParseV1(imageParams));
}

void resetAllEVECores(int32_t num_eve_devices)
{
    int32_t retVal = SYSTEM_LINK_STATUS_SOK;

    /* Enable EVE clock domains */
    if (num_eve_devices >= 1)
        retVal = PMHALCMSetCdClockMode(
            (pmhalPrcmCdId_t) PMHAL_PRCM_CD_EVE1,
            (pmhalPrcmCdClkTrnModes_t) PMHAL_PRCM_CD_CLKTRNMODES_SW_WAKEUP,
            PM_TIMEOUT_NOWAIT);
    if (num_eve_devices >= 2)
        retVal += PMHALCMSetCdClockMode(
            (pmhalPrcmCdId_t) PMHAL_PRCM_CD_EVE2,
            (pmhalPrcmCdClkTrnModes_t) PMHAL_PRCM_CD_CLKTRNMODES_SW_WAKEUP,
            PM_TIMEOUT_NOWAIT);
#ifdef SOC_AM572x
    if (num_eve_devices >= 3)
        retVal += PMHALCMSetCdClockMode(
            (pmhalPrcmCdId_t) PMHAL_PRCM_CD_EVE3,
            (pmhalPrcmCdClkTrnModes_t) PMHAL_PRCM_CD_CLKTRNMODES_SW_WAKEUP,
            PM_TIMEOUT_NOWAIT);
    if (num_eve_devices >= 4)
        retVal += PMHALCMSetCdClockMode(
            (pmhalPrcmCdId_t) PMHAL_PRCM_CD_EVE4,
            (pmhalPrcmCdClkTrnModes_t) PMHAL_PRCM_CD_CLKTRNMODES_SW_WAKEUP,
            PM_TIMEOUT_NOWAIT);
#endif

    /* Enable EVE modules */
    if (num_eve_devices >= 1)
        retVal += PMHALModuleModeSet(
            (pmhalPrcmModuleId_t) PMHAL_PRCM_MOD_EVE1,
            (pmhalPrcmModuleSModuleMode_t) PMHAL_PRCM_MODULE_MODE_AUTO,
            PM_TIMEOUT_INFINITE);
    if (num_eve_devices >= 2)
        retVal += PMHALModuleModeSet(
            (pmhalPrcmModuleId_t) PMHAL_PRCM_MOD_EVE2,
            (pmhalPrcmModuleSModuleMode_t) PMHAL_PRCM_MODULE_MODE_AUTO,
            PM_TIMEOUT_INFINITE);
#ifdef SOC_AM572x
    if (num_eve_devices >= 3)
        retVal += PMHALModuleModeSet(
            (pmhalPrcmModuleId_t) PMHAL_PRCM_MOD_EVE3,
            (pmhalPrcmModuleSModuleMode_t) PMHAL_PRCM_MODULE_MODE_AUTO,
            PM_TIMEOUT_INFINITE);
    if (num_eve_devices >= 4)
        retVal += PMHALModuleModeSet(
            (pmhalPrcmModuleId_t) PMHAL_PRCM_MOD_EVE4,
            (pmhalPrcmModuleSModuleMode_t) PMHAL_PRCM_MODULE_MODE_AUTO,
            PM_TIMEOUT_INFINITE);
#endif

    if (SYSTEM_LINK_STATUS_SOK != retVal)
    {
        /* ToDo Add Failure print Statements */
    }

    /* Reset EVE1 */
    if (num_eve_devices >= 1)
        SBLLibCPUReset(SBLLIB_CORE_ID_EVE1);

    /* Reset EVE2 */
    if (num_eve_devices >= 2)
        SBLLibCPUReset(SBLLIB_CORE_ID_EVE2);

    /* Reset EVE3 */
    if (num_eve_devices >= 3)
        SBLLibCPUReset(SBLLIB_CORE_ID_EVE3);

    /* Reset EVE4 */
    if (num_eve_devices >= 4)
        SBLLibCPUReset(SBLLIB_CORE_ID_EVE4);
}
