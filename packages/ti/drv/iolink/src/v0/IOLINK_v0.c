/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http:;www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
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

/**
 *  \file   IOLINK_v0.c
 *
 *  \brief  PRU IO-Link FW specific IO-Link Master Driver.
 *
 *  This file contains the driver APIs for IO-Link FW controller.
 *
 */


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/*#include <ti/sysbios/hal/Hwi.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>*/ /* TBD: call osal.h */

#include <ti/drv/iolink/src/v0/IOLINK_v0.h>
#include <ti/drv/iolink/src/v0/IOLINK_memoryMap.h>
#include <ti/drv/iolink/src/v0/IOLINK_fw_pru0.h>

#define IOLINK_v0 IO_LINK_FIRMWARE_PRU0

/* ========================================================================== */
/*                          External Variables                                */
/* ========================================================================== */

extern PRUICSS_Config pruss_config[2 + 1];


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void IOLINK_init_v0(IOLINK_Handle handle);
static IOLINK_STATUS IOLINK_close_v0(IOLINK_Handle handle);
static IOLINK_Handle IOLINK_open_v0(IOLINK_Handle handle, const IOLINK_Params *params);
static IOLINK_STATUS IOLINK_control_v0(IOLINK_Handle handle, uint32_t cmd, void *arg);

static IOLINK_STATUS IOLINK_pruInit(IOLINK_Handle handle);
static void IOLINK_setCallbacks(IOLINK_Handle handle, IOLINK_v0_Callbacks *callbacks);
static void IOLINK_enableChannels(IOLINK_Handle handle, bool enable);
static void IOLINK_sendCommand(IOLINK_Handle handle, uint32_t channel, uint32_t cmd, uint32_t cmdArg);
static void IOLINK_setBuffer(IOLINK_Handle handle, uint32_t channel, uint8_t txBufLen, uint8_t rxBufLen, uint8_t *txBuf, uint8_t *rxBuf);
static void IOLINK_sendBuffer(IOLINK_Handle handle, uint32_t channel);
static void IOLINK_start10msTimer(IOLINK_Handle handle, uint32_t channel);
static void IOLINK_stop10msTimer(IOLINK_Handle handle, uint32_t channel);
static void IOLINK_setCycleTimer(IOLINK_Handle handle, uint32_t channel, uint32_t delay);
static void IOLINK_startCycleTimer(IOLINK_Handle handle, uint32_t channel);
static void IOLINK_stopCycleTimer(IOLINK_Handle handle, uint32_t channel);
static void IOLINK_startAdjustableTimer(IOLINK_Handle handle, uint32_t channel, uint8_t type, uint32_t compare);
static void IOLINK_stopAdjustableTimer(IOLINK_Handle handle);

static void IOLINK_cycleTickHwi(void *arg);
static void IOLINK_adjustableTimerHwi(void *arg);
static void IOLINK_pruCompleteHwi(void *arg);

static void IOLINK_softwareTimerSwiFxn(void *arg0, void *arg1);
static void IOLINK_cycleTimerSwiFxn(void *arg0, void *arg1);
static void IOLINK_tRenTimerSwiFxn(void *arg0, void *arg1);
static void IOLINK_tDmtTimerSwiFxn(void *arg0, void *arg1);
static void IOLINK_pruCompleteSwiFxn(void *arg0, void *arg1);

/* IOLINK function table for IOLINK master driver implementation */
const IOLINK_FxnTable IOLINK_v0_FxnTable =
{
    &IOLINK_init_v0,
    &IOLINK_open_v0,
    &IOLINK_close_v0,
    &IOLINK_control_v0
};

/*
 *  ======== IOLINK_init_v0 ========
 */
static void IOLINK_init_v0(IOLINK_Handle handle)
{
    if(handle != NULL)
    {
        /* Mark the object as available */
        ((IOLINK_v0_Object *)(handle->object))->isOpen = (bool)false;
    }
    return;
}

/*
 *  ======== IOLINK_open_v0 ========
 */
static IOLINK_Handle IOLINK_open_v0(IOLINK_Handle handle, const IOLINK_Params *params)
{
    IOLINK_v0_SwAttrs const *swAttrs;
    IOLINK_v0_Object        *object;
    PRUICSS_Handle           pruIcssHandle;
    OsalRegisterIntrParams_t interruptRegParams;
    SwiP_Params              swiParams;
    IOLINK_PruIcssHwiAttrs  *hwiAttrs;
    uint32_t                 key;
    IOLINK_STATUS            status = IOLINK_STATUS_SUCCESS;
    uint32_t                 i;

    /* Input parameter validation */
    if(handle != NULL)
    {
        /* Get the pointer to the object and swAttrs */
        swAttrs  = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;
        object   = (IOLINK_v0_Object *)handle->object;

        /* Determine if the device index was already opened */
        key = IOLINK_osalHardwareIntDisable();
        if(object->isOpen == true)
        {
            IOLINK_osalHardwareIntRestore(key);
            handle = NULL;
        }
        else
        {
            /* Mark the handle as being used */
            object->isOpen = (bool)true;
            IOLINK_osalHardwareIntRestore(key);

            /* IOLink Timer initialization */
            for (i = 0; i < IOLINK_MAX_NUM_CHN; i++)
            {
                object->timerConfig.cycleTimer.enable[i] = false;
                object->timerConfig.cycleTimer.timer[i] = 0;
                object->timerConfig.swTimer.enable[i] = false;
            }

            object->timerConfig.swTimer.timer = 0;
            object->timerConfig.swTimer.timerCnt = 0;
            object->timerConfig.swTimer.timerDiv = 100U;


            /* Store the IOLINK parameters */
            if (params == NULL)
            {
                /* No params passed in, so use the defaults */
                IOLINK_Params_init(&(object->iolinkParams));
            }
            else {
                /* Copy the params contents */
                object->iolinkParams = *params;
            }

            /* Initialize ICSS PRU */
            pruIcssHandle = PRUICSS_create(pruss_config, swAttrs->pruIcssConfig.icssNum);
            if (pruIcssHandle != NULL)
            {
                object->pruIcssHandle = pruIcssHandle;
                PRUICSS_enableOCPMasterAccess(pruIcssHandle);
            }
            else
            {
                status = IOLINK_STATUS_ERROR;
            }

            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Initialize timers */
                IOLINK_cTimerInit();
                IOLINK_adjustableTimerInit();
            }

            /* Register H/W interrupts */
            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Register pruCompleteHwi */
                Osal_RegisterInterrupt_initParams(&interruptRegParams);

                /* Populate the interrupt parameters */
                hwiAttrs = (IOLINK_PruIcssHwiAttrs *)&swAttrs->pruCompleteIntConfig;
                interruptRegParams.corepacConfig.arg = (uintptr_t)handle;
                interruptRegParams.corepacConfig.name = NULL;
                interruptRegParams.corepacConfig.priority = hwiAttrs->intPriority;
                interruptRegParams.corepacConfig.isrRoutine = (HwiP_Fxn)&IOLINK_pruCompleteHwi;
                interruptRegParams.corepacConfig.corepacEventNum = hwiAttrs->socEvId;
                interruptRegParams.corepacConfig.intVecNum = hwiAttrs->coreIntNum;
                IOLINK_osalRegisterInterrupt(&interruptRegParams, &(object->pruCompleteHwi));
                if(object->pruCompleteHwi == NULL)
                {
                    status = IOLINK_STATUS_ERROR;
                }
            }

            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Register cycleTickHwi */
                hwiAttrs = (IOLINK_PruIcssHwiAttrs *)&swAttrs->cycleCounterIntConfig;
                interruptRegParams.corepacConfig.priority = hwiAttrs->intPriority;
                interruptRegParams.corepacConfig.isrRoutine = (HwiP_Fxn)&IOLINK_cycleTickHwi;
                interruptRegParams.corepacConfig.corepacEventNum = hwiAttrs->socEvId;
                interruptRegParams.corepacConfig.intVecNum = hwiAttrs->coreIntNum;
                IOLINK_osalRegisterInterrupt(&interruptRegParams, &(object->cycleCounterHwi));
                if(object->cycleCounterHwi == NULL)
                {
                    status = IOLINK_STATUS_ERROR;
                }
            }

            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Register adjustableTimerHwi */
                hwiAttrs = (IOLINK_PruIcssHwiAttrs *)&swAttrs->adjustableTimerIntConfig;
                interruptRegParams.corepacConfig.priority = hwiAttrs->intPriority;
                interruptRegParams.corepacConfig.isrRoutine = (HwiP_Fxn)&IOLINK_adjustableTimerHwi;
                interruptRegParams.corepacConfig.corepacEventNum = hwiAttrs->socEvId;
                interruptRegParams.corepacConfig.intVecNum = hwiAttrs->coreIntNum;
                IOLINK_osalRegisterInterrupt(&interruptRegParams, &(object->adjustableTimerHwi));
                if(object->adjustableTimerHwi == NULL)
                {
                    status = IOLINK_STATUS_ERROR;
                }
            }

            /* Register S/W interrupts */
            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Register softwareTimerSwi */
                SwiP_Params_init(&swiParams);
                swiParams.arg0 = (uintptr_t)handle;
                swiParams.arg1 = (uintptr_t)0;
                object->softwareTimerSwi = IOLINK_osalSwiCreate((SwiP_Fxn)&IOLINK_softwareTimerSwiFxn,
                                                                &swiParams);
                if(object->softwareTimerSwi == NULL)
                {
                    status = IOLINK_STATUS_ERROR;
                }
            }

            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Register cycleTimerElapsedSwi */
                for (i = 0; i < IOLINK_MAX_NUM_CHN; i++)
                {
                    swiParams.arg1 = (uintptr_t)i;
                    object->cycleTimerElapsedSwi[i] = IOLINK_osalSwiCreate((SwiP_Fxn)&IOLINK_cycleTimerSwiFxn,
                                                                           &swiParams);
                    if(object->cycleTimerElapsedSwi[i] == NULL)
                    {
                        status = IOLINK_STATUS_ERROR;
                        break;
                    }
                }
            }

            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Register tDMTTimerSwi */
                swiParams.arg1 = (uintptr_t)0;
                object->tDMTTimerSwi = IOLINK_osalSwiCreate((SwiP_Fxn)&IOLINK_tDmtTimerSwiFxn,
                                                            &swiParams);
                if(object->tDMTTimerSwi == NULL)
                {
                    status = IOLINK_STATUS_ERROR;
                }
            }

            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Register tRenTimerSwi */
                object->tRenTimerSwi = IOLINK_osalSwiCreate((SwiP_Fxn)&IOLINK_tRenTimerSwiFxn,
                                                            &swiParams);
                if(object->tRenTimerSwi == NULL)
                {
                    status = IOLINK_STATUS_ERROR;
                }
            }

            if (status == IOLINK_STATUS_SUCCESS)
            {
                /* Register pruCompleteSwi */
                for (i = 0; i < IOLINK_MAX_NUM_CHN; i++)
                {
                    swiParams.arg1 = (uintptr_t)i;
                    object->pruCompleteSwi[i] = IOLINK_osalSwiCreate((SwiP_Fxn)&IOLINK_pruCompleteSwiFxn,
                                                                     &swiParams);
                    if(object->pruCompleteSwi[i] == NULL)
                    {
                        status = IOLINK_STATUS_ERROR;
                        break;
                    }
                }
            }

            if (status == IOLINK_STATUS_SUCCESS)
            {
                status = IOLINK_pruInit(handle);
                if (status == IOLINK_STATUS_SUCCESS)
                {
                    IOLINK_enableChannels(handle, true);
                }
            }

            if (status == IOLINK_STATUS_ERROR)
            {
                handle = NULL;
            }
        }
    }

    return (handle);
}

/*
 *  ======== IOLINK_close_v0 ========
 */
static IOLINK_STATUS IOLINK_close_v0(IOLINK_Handle handle)
{
    IOLINK_v0_SwAttrs const *swAttrs;
    IOLINK_v0_Object        *object;
    IOLINK_STATUS            status = IOLINK_STATUS_SUCCESS;
    uint32_t                 i;

    if(handle != NULL)
    {
        /* Get the pointer to the object and swAttrs */
        swAttrs = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;
        object  = (IOLINK_v0_Object *)handle->object;

        /* De-register H/W interrupts */
        if (object->adjustableTimerHwi != NULL)
        {
            IOLINK_osalHardwareIntDestruct(object->adjustableTimerHwi, swAttrs->adjustableTimerIntConfig.socEvId);
            object->adjustableTimerHwi = NULL;
        }

        if (object->cycleCounterHwi != NULL)
        {
            IOLINK_osalHardwareIntDestruct(object->cycleCounterHwi, swAttrs->cycleCounterIntConfig.socEvId);
            object->cycleCounterHwi = NULL;
        }

        if (object->pruCompleteHwi != NULL)
        {
            IOLINK_osalHardwareIntDestruct(object->pruCompleteHwi, swAttrs->pruCompleteIntConfig.socEvId);
            object->pruCompleteHwi = NULL;
        }

        /* De-register S/W interrupts */
        /* TBD: use osal swi APIs */
        if (object->softwareTimerSwi != NULL)
        {
            IOLINK_osalSoftwareIntDestruct(&(object->softwareTimerSwi));
            object->softwareTimerSwi = NULL;
        }

        if (object->tRenTimerSwi != NULL)
        {
            IOLINK_osalSoftwareIntDestruct(&(object->tRenTimerSwi));
            object->tRenTimerSwi = NULL;
        }

        if (object->tDMTTimerSwi != NULL)
        {
            IOLINK_osalSoftwareIntDestruct(&(object->tDMTTimerSwi));
            object->tDMTTimerSwi = NULL;
        }

        for (i = 0; i < IOLINK_MAX_NUM_CHN; i++)
        {
            if (object->cycleTimerElapsedSwi[i] != NULL)
            {
                IOLINK_osalSoftwareIntDestruct(&(object->cycleTimerElapsedSwi[i]));
                object->cycleTimerElapsedSwi[i] = NULL;
            }
            if (object->pruCompleteSwi[i] != NULL)
            {
                IOLINK_osalSoftwareIntDestruct(&(object->pruCompleteSwi[i]));
                object->pruCompleteSwi[i] = NULL;
            }
        }

        IOLINK_enableChannels(handle, false);

        /* Disable the PRU */
        if (PRUICSS_pruDisable(object->pruIcssHandle, swAttrs->pruIcssConfig.pruNum) != 0)
        {
            status = IOLINK_STATUS_ERROR;
        }

        object->pruIcssHandle = NULL;
        object->isOpen = (bool)false;
    }

    return (status);
}

/*
 *  ======== IOLINK_control_v0 ========
 */
static IOLINK_STATUS IOLINK_control_v0(IOLINK_Handle handle, uint32_t cmd, void *arg)
{
    uint32_t                *data = (uint32_t *)arg;
    IOLINK_STATUS            status = IOLINK_STATUS_SUCCESS;

    /* Input parameter validation */
    if(handle != NULL)
    {
        switch (cmd)
        {
            case IOLINK_CTRL_SEND_CMD:
            {
                IOLINK_sendCommand(handle, *data, *(data + 1U), *(data + 2U));
                break;
            }

            case IOLINK_CTRL_SET_XFER_BUFFER:
            {
                IOLINK_setBuffer(handle, *data, (uint8_t)(*(data + 1U)), (uint8_t)(*(data + 2U)), (uint8_t *)(*(data + 3U)), (uint8_t *)(*(data + 4U)));
                break;
            }

            case IOLINK_CTRL_START_XFER:
            {
                IOLINK_sendBuffer(handle, *data);
                break;
            }

            case IOLINK_CTRL_START_TIMER:
            {
                if (*(data + 1U) == IOLINK_TIMER_TYPE_10MS)
                {
                    IOLINK_start10msTimer(handle, *data);
                }
                else if (*(data + 1U) == IOLINK_TIMER_TYPE_CYCLE)
                {
                    IOLINK_startCycleTimer(handle, *data);
                }
                else
                {
                    IOLINK_startAdjustableTimer(handle, *data, *(data + 2U), *(data + 3U));
                }
                break;
            }

            case IOLINK_CTRL_STOP_TIMER:
            {
                if (*(data + 1U) == IOLINK_TIMER_TYPE_10MS)
                {
                    IOLINK_stop10msTimer(handle, *data);
                }
                else if (*(data+1) == IOLINK_TIMER_TYPE_CYCLE)
                {
                    IOLINK_stopCycleTimer(handle, *data);
                }
                else
                {
                    IOLINK_stopAdjustableTimer(handle);
                }
                break;
            }

            case IOLINK_CTRL_SET_CYCLE_TIMER:
            {
                IOLINK_setCycleTimer(handle, *data, *(data + 1U));
                break;
            }

            case IOLINK_CTRL_SET_CALLBACKS:
            {
                IOLINK_setCallbacks(handle, (IOLINK_v0_Callbacks *)data);
                break;
            }


            default:
            {
                status = IOLINK_STATUS_ERROR;
                break;
            }
        }
    }
    else
    {
        status = IOLINK_STATUS_ERROR;
    }

    return (status);
}

/*
 *  ======== IOLINK_pruInit ========
 */
static IOLINK_STATUS IOLINK_pruInit(IOLINK_Handle handle)
{
    IOLINK_v0_SwAttrs const *swAttrs;
    IOLINK_v0_Object        *object;
    uint32_t                 i;
    uint32_t                 baseAddr;
    IOLINK_STATUS            status = IOLINK_STATUS_SUCCESS;
    //uint8_t                  global_cfg[4U] = {0, 0, 25U, 0}; /* globalSts, globalCtrl, maxResponseTime (Tbit), firmwareRev */
    uint32_t                 global_cfg;      /* firmwareRev */

    /* Get the pointer to the object and swAttrs */
    swAttrs = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;
    object  = (IOLINK_v0_Object *)handle->object;

    /* Disable the PRU */
    if (PRUICSS_pruDisable(object->pruIcssHandle, swAttrs->pruIcssConfig.pruNum) != 0)
    {
        status = IOLINK_STATUS_ERROR;
    }

    if (status == IOLINK_STATUS_SUCCESS)
    {
        /* Reset the PRU */
        if (PRUICSS_pruReset(object->pruIcssHandle, swAttrs->pruIcssConfig.pruNum) != 0)
        {
            status = IOLINK_STATUS_ERROR;
        }
    }

    if (status == IOLINK_STATUS_SUCCESS)
    {
        baseAddr = swAttrs->pruIcssConfig.dataMemBaseAddr;

        /* PRU DRAM cleanup */
        for(i = 0; i < 1024U; i++)
        {
            *((uint32_t*) (baseAddr + i * 4U)) = 0U;
        }

        /* Initialize the PRU's DRAM */
        global_cfg = (0 << 0)                         |   /* globalSts */
                     (0 << 8U)                        |   /* globalCtrl */
                     (object->iolinkParams.tA << 16U) |   /* maxResponseTime (Tbit) */
                     (swAttrs->version << 24U);           /* firmwareRev */
        *((uint32_t*) (baseAddr + globalSts)) = global_cfg;

        /* Set the PRU's TX pins (GPO's) */
        *((uint32_t*) (baseAddr + channel0mem + 4U)) = (channel0TxPin << 24U);
        *((uint32_t*) (baseAddr + channel1mem + 4U)) = (channel1TxPin << 24U);
        *((uint32_t*) (baseAddr + channel2mem + 4U)) = (channel2TxPin << 24U);
        *((uint32_t*) (baseAddr + channel3mem + 4U)) = (channel3TxPin << 24U);
        *((uint32_t*) (baseAddr + channel4mem + 4U)) = (channel4TxPin << 24U);
        *((uint32_t*) (baseAddr + channel5mem + 4U)) = (channel5TxPin << 24U);
        *((uint32_t*) (baseAddr + channel6mem + 4U)) = (channel6TxPin << 24U);
        *((uint32_t*) (baseAddr + channel7mem + 4U)) = (channel7TxPin << 24U);

        /* Set the PRU's TX Enable ports (GPIO's) */
        *((uint32_t*) (baseAddr + channel0mem + 8U)) = channel0TxEnGpioAdr;
        *((uint32_t*) (baseAddr + channel1mem + 8U)) = channel1TxEnGpioAdr;
        *((uint32_t*) (baseAddr + channel2mem + 8U)) = channel2TxEnGpioAdr;
        *((uint32_t*) (baseAddr + channel3mem + 8U)) = channel3TxEnGpioAdr;
        *((uint32_t*) (baseAddr + channel4mem + 8U)) = channel4TxEnGpioAdr;
        *((uint32_t*) (baseAddr + channel5mem + 8U)) = channel5TxEnGpioAdr;
        *((uint32_t*) (baseAddr + channel6mem + 8U)) = channel6TxEnGpioAdr;
        *((uint32_t*) (baseAddr + channel7mem + 8U)) = channel7TxEnGpioAdr;

        /* Set the PRU's TX Enable pins (GPIO's) */
        *((uint32_t*) (baseAddr + channel0mem + 12U)) = channel0TxEnGpioPin;
        *((uint32_t*) (baseAddr + channel1mem + 12U)) = channel1TxEnGpioPin;
        *((uint32_t*) (baseAddr + channel2mem + 12U)) = channel2TxEnGpioPin;
        *((uint32_t*) (baseAddr + channel3mem + 12U)) = channel3TxEnGpioPin;
        *((uint32_t*) (baseAddr + channel4mem + 12U)) = channel4TxEnGpioPin;
        *((uint32_t*) (baseAddr + channel5mem + 12U)) = channel5TxEnGpioPin;
        *((uint32_t*) (baseAddr + channel6mem + 12U)) = channel6TxEnGpioPin;
        *((uint32_t*) (baseAddr + channel7mem + 12U)) = channel7TxEnGpioPin;

        /* Initialize the PRU's pinmux */
        IOLINK_pruIcssPinMuxCfg();

        /* ICSS0 interrupt INTC configuration */
        baseAddr = swAttrs->pruIcssConfig.intcBaseAddr;

        /* Set the interrupt polarity of system event 16 to active high */
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_SIPR0)) |= (1<<16);
        /* Set the type of system event 16 to level or pulse interrupt */
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_SITR0)) &= ~(1<<16);
        /* map system event with index 16 to channel 3 */
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_CMR4)) &= ~(CSL_ICSSM_INTC_CMR4_CH_MAP_16_MASK);
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_CMR4)) |= (0x3<<CSL_ICSSM_INTC_CMR4_CH_MAP_16_SHIFT);
        /* map channel 3 to interrupt with index 3 */
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_HMR0)) &= ~(CSL_ICSSM_INTC_HMR0_HINT_MAP_3_MASK);
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_HMR0)) |= (0x3<<CSL_ICSSM_INTC_HMR0_HINT_MAP_3_SHIFT);
        /* clear the system event with index 16  (pr0_iep_tim_cap_cmp_pend) */
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_SICR)) = 16;
        /* enable the system event with index 16  (pr0_iep_tim_cap_cmp_pend) */
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_EISR)) = 16;
        /* enable host interrupt output with index 3 */
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_HIEISR)) = 0x3;
        /* globally enable all interrupts */
        *((uint32_t*) (baseAddr + CSL_ICSSM_INTC_GER)) = 0x1;
    }

    if (status == IOLINK_STATUS_SUCCESS)
    {
        /* Write the DRAM and IRAM of PRU 0 */
        if (PRUICSS_pruWriteMemory(object->pruIcssHandle,
                                   swAttrs->pruIcssConfig.instMem,
                                   0,
                                   (uint32_t *)IO_LINK_FIRMWARE_PRU0,
                                   sizeof(IO_LINK_FIRMWARE_PRU0)) == 0)
        {
            status = IOLINK_STATUS_ERROR;
        }

        if (status == IOLINK_STATUS_SUCCESS)
        {
            /* load the LUT's into the PRU data memory */
            if (PRUICSS_pruWriteMemory(object->pruIcssHandle,
                                       swAttrs->pruIcssConfig.dataMem0,
                                       pru0LutAdr,
                                       memInitDataPRU0,
                                       1024U) == 0)
            {
                status = IOLINK_STATUS_ERROR;
            }
        }

        if (status == IOLINK_STATUS_SUCCESS)
        {
            /*
             * load the LUT of the other PRU, initialization and
             * loading of the other PRU's firmware could be done here
             */
            if (PRUICSS_pruWriteMemory(object->pruIcssHandle,
                                       swAttrs->pruIcssConfig.dataMem1,
                                       pru1LutAdr,
                                       memInitDataPRU1,
                                       1024U) == 0)

            {
                status = IOLINK_STATUS_ERROR;
            }
        }

        if (status == IOLINK_STATUS_SUCCESS)
        {
            if (PRUICSS_pruEnable(object->pruIcssHandle, swAttrs->pruIcssConfig.pruNum) != 0)
            {
                status = IOLINK_STATUS_ERROR;
            }
        }
    }

    return (status);
}

/*
 *  ======== IOLINK_setCallbacks ========
 */
static void IOLINK_setCallbacks(IOLINK_Handle handle, IOLINK_v0_Callbacks *callbacks)
{
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->callbacks = *callbacks;
}

/*
 *  ======== IOLINK_enableChannels ========
 */
static void IOLINK_enableChannels(IOLINK_Handle handle, bool enable)
{
    IOLINK_v0_SwAttrs const *swAttrs;
    uint32_t                 baseAddr;

    /* Get the pointer to the swAttrs */
    swAttrs = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;

    baseAddr = swAttrs->pruIcssConfig.dataMemBaseAddr;

    *((uint32_t*) (baseAddr)) &= ~(0xFFU << 8U);
    *((uint32_t*) (baseAddr)) |= (((uint32_t)enable) << 8U);
}

/*
 *  ======== IOLINK_sendCommand ========
 */
static void IOLINK_sendCommand(IOLINK_Handle handle, uint32_t channel, uint32_t cmd, uint32_t cmdArg)
{
    IOLINK_v0_SwAttrs const *swAttrs;
    uint32_t                 pruCtrlRegAddr;

    /* Get the pointer to the swAttrs */
    swAttrs = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;

    pruCtrlRegAddr = swAttrs->pruIcssConfig.dataMemBaseAddr + PRU_CTRL_ADDR(channel);

    switch (cmd)
    {
        case IOLINK_COMMAND_STARTPULSE:
            *((uint32_t *)(pruCtrlRegAddr)) |= (1U << 2U);
            *((uint32_t *)(pruCtrlRegAddr)) |= 0x01U;
            break;

        case IOLINK_COMMAND_SETCOM:
            *((uint32_t *)(pruCtrlRegAddr)) &= ~(0xFFU << 8U);
            *((uint32_t *)(pruCtrlRegAddr)) |= (cmdArg << 8U);
            break;

        default:
            break;
    }

    return;
}

/*
 *  ======== IOLINK_setBuffer ========
 */
static void IOLINK_setBuffer(IOLINK_Handle handle, uint32_t channel, uint8_t txBufLen, uint8_t rxBufLen, uint8_t *txBuf, uint8_t *rxBuf)
{
    IOLINK_v0_SwAttrs const *swAttrs;
    IOLINK_v0_Object        *object;
    uint32_t                 pruTxBufAddr;
    uint8_t                  newlen;
    uint8_t                  tmp[128];
    uint8_t                  i;
    IOLINK_dataConverter     convert;

    /* Get the pointer to the object and swAttrs */
    swAttrs = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;
    object  = (IOLINK_v0_Object *)handle->object;

    pruTxBufAddr = swAttrs->pruIcssConfig.dataMemBaseAddr + PRU_TXBUF_ADDR(channel);

    object->rxBufConfig.rxBufAddr[channel] = rxBuf;
    object->rxBufConfig.rxBufLen[channel] = (uint32_t)rxBufLen;

    /*
     * add the buffer length information bytes at the beginning and
     * copy the new array of data to the PRU's TX buffer
     */
    if ((txBufLen + 2U) % 4U != 0)
    {
        newlen = ((txBufLen + 2U)/4U) * 4U + 4U;
    }
    else
    {
        newlen = (txBufLen + 2U);
    }

    tmp[0] = txBufLen;
    tmp[1] = rxBufLen;

    for(i = 0; i < txBufLen; i++)
    {
        tmp[i + 2U] = txBuf[i];
    }

    for (i = txBufLen; i < newlen; i++)
    {
        tmp[i + 2U] = 0;
    }

    {
        convert.data8bit = tmp;

        newlen = newlen/4U;

        for(int i = 0; i < newlen; i ++)
        {
            *((uint32_t *)(pruTxBufAddr + (i * 4U))) = convert.data32bit[i];
        }
    }

    return;
}

/*
 *  ======== IOLINK_sendBuffer ========
 */
static void IOLINK_sendBuffer(IOLINK_Handle handle, uint32_t channel)
{
    IOLINK_v0_SwAttrs const *swAttrs;
    uint32_t                 pruCtrlRegAddr;

    /* Get the pointer to the swAttrs */
    swAttrs = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;

    pruCtrlRegAddr = swAttrs->pruIcssConfig.dataMemBaseAddr + PRU_CTRL_ADDR(channel);
    *((uint32_t *)pruCtrlRegAddr) |= 0x1U;
}

/*
 *  ======== IOLINK_start10msTimer ========
 */
static void IOLINK_start10msTimer(IOLINK_Handle handle, uint32_t channel)
{
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->timerConfig.swTimer.enable[channel] = true;
    if (object->timerConfig.swTimer.timerCnt == 0)
    {
        object->timerConfig.swTimer.timer = 0;
    }
    object->timerConfig.swTimer.timerCnt++;
}

/*
 *  ======== IOLINK_stop10msTimer ========
 */
static void IOLINK_stop10msTimer(IOLINK_Handle handle, uint32_t channel)
{
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->timerConfig.swTimer.enable[channel] = false;
    object->timerConfig.swTimer.timerCnt--;
}

/*
 *  ======== IOLINK_setCycleTimer ========
 */
static void IOLINK_setCycleTimer(IOLINK_Handle handle, uint32_t channel, uint32_t delay)
{
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->timerConfig.cycleTimer.delay[channel] = delay;
}

/*
 *  ======== IOLINK_startCycleTime ========
 */
static void IOLINK_startCycleTimer(IOLINK_Handle handle, uint32_t channel)
{
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->timerConfig.cycleTimer.timer[channel] = 0;
    object->timerConfig.cycleTimer.enable[channel] = true;
}

/*
 *  ======== IOLINK_stopCycleTimer ========
 */
void IOLINK_stopCycleTimer(IOLINK_Handle handle, uint32_t channel)
{
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->timerConfig.cycleTimer.enable[channel] = false;
}

/*
 *  ======== IOLINK_startAdjustableTimer ========
 */
static void IOLINK_startAdjustableTimer(IOLINK_Handle handle, uint32_t channel, uint8_t type, uint32_t compare)
{
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->timerConfig.adjTimer.timerType = type;
    object->timerConfig.adjTimer.activeChannel = channel;
    IOLINK_adjustableTimerStart(compare);
}

/*
 *  ======== IOLINK_stopAdjustableTimer ========
 */
static void IOLINK_stopAdjustableTimer(IOLINK_Handle handle)
{
    IOLINK_adjustableTimerStop();
}

/*
 * ======== IOLINK_cycleTickHwi ========
 *
 * This hardware interrupt runs at 100 us per tick
 * It is used to generate the clock for 8 independent software timers (cycle timer) and
 * one 10 ms tick (slower software timers which are supported by the IO-Link master stack)
 */
static void IOLINK_cycleTickHwi(void *arg)
{
    IOLINK_Handle            handle = (IOLINK_Handle)arg;
    IOLINK_v0_Object        *object;
    uint32_t                 i;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    IOLINK_clearCycleTimerInt();

    /* 100 us software timer for cycle time measurement */
    for (i = 0; i < IOLINK_MAX_NUM_CHN; i++)
    {
        if (object->timerConfig.cycleTimer.enable[i] == true)
        {
            object->timerConfig.cycleTimer.timer[i]++;
            if (object->timerConfig.cycleTimer.timer[i] >= object->timerConfig.cycleTimer.delay[i])
            {
                object->timerConfig.cycleTimer.enable[i] = false;
                object->timerConfig.cycleTimer.timer[i] = 0;
                IOLINK_osalSoftwareIntPost(object->cycleTimerElapsedSwi[i]);
            }
        }
    }

    if (object->timerConfig.swTimer.timerCnt > 0)
    {
        object->timerConfig.swTimer.timer++;

        /* generate 10 ms tick for additional timers */
        if (object->timerConfig.swTimer.timer >= object->timerConfig.swTimer.timerDiv)
        {
            object->timerConfig.swTimer.timer = 0;
            IOLINK_osalSoftwareIntPost(object->softwareTimerSwi);
        }
    }
}

/*
 * ======== IOLINK_adjustableTimerHwi ========
 *
 * This hardware interrupt is called by the adjustable timer after it has reached its compare threshold.
 */
static void IOLINK_adjustableTimerHwi(void *arg)
{
    IOLINK_Handle            handle = (IOLINK_Handle)arg;
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    /* clear adjustable timer pending compare match events */
    IOLINK_clearAdjTimerInt();

    switch(object->timerConfig.adjTimer.timerType)
    {
        case IOLINK_TIMER_ADJ_TDMT:
            IOLINK_osalSoftwareIntPost(object->tDMTTimerSwi);
            break;

        case IOLINK_TIMER_ADJ_TREN:
            IOLINK_osalSoftwareIntPost(object->tRenTimerSwi);
            break;

        default:
            break;
    }
}

/*
 * ======== IOLINK_pruCompleteHwi ========
 *
 * This hardware interrupt is called by the PRU after it has finished receiving data from the device
 */
static void IOLINK_pruCompleteHwi(void *arg)
{
    IOLINK_Handle            handle = (IOLINK_Handle)arg;
    IOLINK_v0_SwAttrs const *swAttrs;
    IOLINK_v0_Object        *object;
    uint32_t                 baseAddr;
    uint32_t                 i;

    /* Get the pointer to the object and swAttrs */
    swAttrs = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;
    object  = (IOLINK_v0_Object *)handle->object;

    IOLINK_clearPruCompInt();

    /* check every channel for completion (status byte) */
    baseAddr = swAttrs->pruIcssConfig.dataMemBaseAddr;
    for (i = 0; i < IOLINK_MAX_NUM_CHN; i++)
    {
        if ((*((uint32_t*) (baseAddr + PRU_STATUS_ADDR(i))) & 0xFFFFU) == IOLINK_PRU_STATUS_COMPLETE)
        {
            /* reset the status byte and post a software interrupt */
            *((uint32_t*) (baseAddr + PRU_STATUS_ADDR(i))) &= 0xFFFF0000U;
            IOLINK_osalSoftwareIntPost(object->pruCompleteSwi[i]);
        }
    }
}

/*
 * ======== IOLINK_softwareTimerSwiFxn ========
 *
 * This software interrupt can be used as a clock for more software timers,
 * which should be provided by the stack (10 ms tick).
 */
static void IOLINK_softwareTimerSwiFxn(void *arg0, void *arg1)
{
    IOLINK_Handle            handle = (IOLINK_Handle)arg0;
    IOLINK_v0_Object        *object;
    uint32_t                 i;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    for(i = 0; i < IOLINK_MAX_NUM_CHN; i++)
    {
        if(object->timerConfig.swTimer.enable[i] == true)
        {
            object->callbacks.swTimerCallback(handle, i);
        }
    }
}

/*
 * ======== IOLINK_cycleTimerSwiFxn ========
 *
 * This software interrupt is used to call a stack supplied callback function.
 * arg1 indicates the dedicated channel number
 */
static void IOLINK_cycleTimerSwiFxn(void *arg0, void *arg1)
{
    IOLINK_Handle            handle = (IOLINK_Handle)arg0;
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->callbacks.cycleTimerCallback(handle, (uint32_t)arg1);
}

/*
 * ======== IOLINK_tRenTimerSwiFxn ========
 *
 * This software interrupt is used to call a stack supplied callback function.
 * arg1 indicates the dedicated channel number
 */
static void IOLINK_tRenTimerSwiFxn(void *arg0, void *arg1)
{
    IOLINK_Handle            handle = (IOLINK_Handle)arg0;
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->callbacks.adjTimerCallback(handle,
                                       object->timerConfig.adjTimer.activeChannel,
                                       IOLINK_TIMER_ADJ_TREN);
}

/*
 * ======== IOLINK_tDmtTimerSwiFxn ========
 *
 * This software interrupt is used to call a stack supplied callback function.
 * arg1 indicates the dedicated channel number
 */
static void IOLINK_tDmtTimerSwiFxn(void *arg0, void *arg1)
{
    IOLINK_Handle            handle = (IOLINK_Handle)arg0;
    IOLINK_v0_Object        *object;

    /* Get the pointer to the object */
    object  = (IOLINK_v0_Object *)handle->object;

    object->callbacks.adjTimerCallback(handle,
                                       object->timerConfig.adjTimer.activeChannel,
                                       IOLINK_TIMER_ADJ_TDMT);
}

/*
 * ======== IOLINK_pruCompleteSwiFxn ========
 *
 * This software interrupt is used to call a stack supplied callback function.
 * arg1 indicates the dedicated channel number
 *
 * The PRU indicates the completion of a communication cycle by generating a hardware interrupt,
 * which will then post the software interrupt with the dedicated channel number.
 *
 * This software interrupt will then read the PRU's buffer and copy it to the stack maintained
 * memory address. It will also check for errors and call the stack supplied callback function.
 */
static void IOLINK_pruCompleteSwiFxn(void *arg0, void *arg1)
{
    IOLINK_Handle            handle = (IOLINK_Handle)arg0;
    IOLINK_v0_SwAttrs const *swAttrs;
    IOLINK_v0_Object        *object;
    uint32_t                 baseAddr;
    uint32_t                 chRxBuf;
    uint32_t                 channel;
    uint8_t                 *rxBuf;
    uint8_t                  rxBuflen;
    uint32_t                 tmp[64];
    uint32_t                 i;
    uint8_t                  receivedLen;
    IOLINK_dataConverter     convert;


    /* Get the pointer to the object and swAttrs */
    swAttrs = (IOLINK_v0_SwAttrs const *)handle->ipAttrs;
    object  = (IOLINK_v0_Object *)handle->object;

    baseAddr = swAttrs->pruIcssConfig.dataMemBaseAddr;
    channel = (uint32_t)arg1;
    chRxBuf = baseAddr + PRU_RXBUF_ADDR(channel);

    /* load received bytes into buffer */
    rxBuf = object->rxBufConfig.rxBufAddr[channel];
    rxBuflen = object->rxBufConfig.rxBufLen[channel];


    for(i = 0; i < 64U; i++)
    {
        tmp[i] = *((uint32_t*) (chRxBuf + 4U * i));
    }

    convert.data32bit = tmp;

    receivedLen = convert.data8bit[0];

    for (i = 0; i < rxBuflen; i++)
    {
        rxBuf[i] = convert.data8bit[i+1];
    }

    /* check the PRU's error register and the received message length */
    if((receivedLen != rxBuflen) || ((*((uint32_t*) (baseAddr + PRU_STATUS_ADDR(channel))) & 0xFFFF0000) != 0))
    {
        object->callbacks.xferErrRspCallback(handle, channel);
    }
    else
    {
        object->callbacks.xferRspCallback(handle, channel);
    }
}


