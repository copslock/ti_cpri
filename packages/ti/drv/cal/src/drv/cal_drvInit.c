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
 *  \file cal_drvInit.c
 *
 *  \brief File containing the CAL capture driver Init APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/cal/src/drv/cal_drvPriv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 *  \brief Platform specific data containing base address information of
 *  various modules.
 */
typedef struct
{
    CalDrv_CaptInitParams     calDrvInitPrms[CAL_CAPT_INST_ID_MAX];
    /**< Cal driver init parameters. */

    uint32_t                    reserved;
    /**< Needed in case both VIP and DSS are not defined!! */
} CalDrv_PlatformData;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  Below ifdef __cplusplus is added so that C++ build passes without
 *  typecasting. This is because the prototype is build as C type
 *  whereas this file is build as CPP file. Hence we get C++ build error.
 *  Also if tyecasting is used, then we get MisraC error Rule 11.1.
 */
#ifdef __cplusplus
extern "C" {
#endif

const CalDrv_PlatformData *CalDrv_getPlatformData(void);
void  CalDrv_initPlatData(void);
const CalDrv_PlatformData *CalDrv_getPlatformData(void);
static int32_t Cal_captCoreInit(void);
#ifdef __cplusplus
}
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
static CalDrv_PlatformData gCalDrvPlatDataAm6xx;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t Cal_init(void)
{
    int32_t retVal = FVID2_SOK;
    const CalDrv_PlatformData *drvPlatData = NULL;
    uint32_t coreInst = 0;
    uint32_t instCnt, CalInstCnt = 0;
    CalDrv_CaptInitParams calDrvInitPrms[CAL_CAPT_INST_ID_MAX];

#if defined (SOC_AM65XX)
    CalDrv_initPlatData();
#endif

    drvPlatData = CalDrv_getPlatformData();
    GT_assert(CalTrace, (NULL != drvPlatData));

    for (instCnt = 0U; instCnt < CAL_CAPT_INST_ID_MAX; instCnt++)
    {
        calDrvInitPrms[CalInstCnt].drvInstId =
            drvPlatData->calDrvInitPrms[instCnt].drvInstId;
        /* Get core instance objects for a given instance */
        calDrvInitPrms[CalInstCnt].coreInstObj =
            Cal_coreCaptGetCoreInstObj((Cal_CaptInstId_t) coreInst);
    
        GT_assert(CalTrace,
                  (NULL != calDrvInitPrms[CalInstCnt].coreInstObj));
    
        calDrvInitPrms[CalInstCnt].coreOps = Cal_coreCaptGetCoreOps();
        CalInstCnt++;
        coreInst++;
    }

    /* Call Cal init only if required */
    if (CalInstCnt > 0U)
    {
        retVal =  Cal_captCoreInit();
        retVal += CalDrv_captInit(CalInstCnt, &calDrvInitPrms[0U]);
    }
    return retVal;
}

int32_t Cal_deInit(void)
{
	int32_t retVal;

	retVal = Cal_coreCaptDeInit(NULL);
	retVal += Cal_emDeInit();
	retVal += CalDrv_captDeInit();

	return retVal;
}


#if defined (SOC_TDA2EX) || defined (SOC_AM65XX)
void  CalDrv_initPlatData(void)
{
    uint32_t idx = 0;

    gCalDrvPlatDataAm6xx.calDrvInitPrms[idx].drvInstId = CAL_CAPT_INST_ID_A_ID;
    gCalDrvPlatDataAm6xx.calDrvInitPrms[idx].coreInstObj = NULL;
    gCalDrvPlatDataAm6xx.calDrvInitPrms[idx].coreOps     = NULL;
    idx++;
    gCalDrvPlatDataAm6xx.calDrvInitPrms[idx].drvInstId = CAL_CAPT_INST_ID_A_ID;
    gCalDrvPlatDataAm6xx.calDrvInitPrms[idx].coreInstObj = NULL;
    gCalDrvPlatDataAm6xx.calDrvInitPrms[idx].coreOps     = NULL;
    idx++;
    GT_assert(CalTrace, (CAL_CAPT_INST_ID_MAX >= idx));
}
#endif

const CalDrv_PlatformData *CalDrv_getPlatformData(void)
{
    CalDrv_PlatformData *platData = NULL;
#if defined (SOC_AM65XX)
    platData = &gCalDrvPlatDataAm6xx;
#else
    GT_0trace(CalTrace, GT_ERR, "VPS: Unsupported SOC!!\r\n");
#endif
    return (platData);
}

static int32_t Cal_captCoreInit(void)
{
    uint32_t instCnt = 0U;
    int32_t retVal = FVID2_SOK;
    Cal_EmPlatformData_t *calEmPlatData;
    Cal_HalPlatformData_t *calHalPlatData;
    Cal_CoreCaptInitParams_t   coreInitParams[CAL_CAPT_INST_ID_MAX];

    calHalPlatData = Cal_halGetPlatformData();
    if (NULL != calHalPlatData)
    {
        coreInitParams[instCnt].instId          = CAL_CAPT_INST_ID_A_ID;
        coreInitParams[instCnt].halPlatformData = calHalPlatData;
        coreInitParams[instCnt].subModules      = CAL_HAL_SUB_MODULE_CAL_A;
        coreInitParams[instCnt].irqNum = CAL_EM_INST_ID_CAL0;
        instCnt++;
        GT_assert(CalTrace, instCnt < CAL_CAPT_INST_ID_MAX);

        /** TODO - The number of core & IP block could be different.
         *      Right now, we have 1 capture block with multiple modes
         *      Each mode represented as a different instance of capture
         *      core.
         *      Require to pass number of IP and number of core instances
         *      desired.
         *      Right now passing the core instances only */
        retVal = Cal_coreCaptInit(instCnt,
                                     &coreInitParams[0], NULL);
    }
    else
    {
        retVal = FVID2_EFAIL;
        GT_assert(CalTrace, FALSE);
/* MISRA.UNREACH.GEN
 * Un Reachable code
 * Name 'status = FVID2_EFAIL;'
 * KW State: Not A Problem -> Waiver -> Case by case
 * MISRAC_WAIVER:
 * In cases where value in the if condition  is dependent on the return of a
 * function and currently the function is hardcoded to return a value. Code is
 * currently unreachable but as the implementation of the function changes, it
 * will not be unreachable
 */
    }

    //Cal_initEmPlatformData(initPrms->irqParams.calIrqNum);
    calEmPlatData = Cal_getEmPlatformData();
    retVal = Cal_emInit(calEmPlatData->numCalEmInst,
                       (const Cal_EmInitParams_t *)(&calEmPlatData->calEmInitParam[0]));
    if (retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR, "VPS: CAL EM Init Failed\r\n");
    }
    return retVal;
}

