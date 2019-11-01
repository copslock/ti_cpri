/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
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

/**
 *  \file   IOLINK_drv.c
 *
 *  \brief  IOLINK Driver high level APIs implementation.
 *
 *   This file contains the driver APIs for IOLINK controller.
 */


#include <ti/drv/iolink/IOLINK.h>
#include <ti/drv/iolink/src/IOLINK_osal.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* Externs */
extern const IOLINK_config_list IOLINK_config;

/* Used to check status and initialization */
static int32_t IOLINK_count = (-((int32_t)1));

/* Default IOLINK parameters structure */
const IOLINK_Params IOLINK_defaultParams =
{
    25U,      /* tA */
};

/*
 *  ======== IOLINK_init ========
 */
void IOLINK_init(void)
{
    if (IOLINK_count == (-((int32_t)1)))
    {
        /* Call each driver's init function */
        for (IOLINK_count = 0; IOLINK_config[IOLINK_count].fxnTablePtr != NULL; IOLINK_count++)
        {
            IOLINK_config[IOLINK_count].fxnTablePtr->initFxn((IOLINK_Handle)&(IOLINK_config[IOLINK_count]));
        }
    }
}

/*
 *  ======== IOLINK_open ========
 */
IOLINK_Handle IOLINK_open(uint32_t index, IOLINK_Params *params)
{
    IOLINK_Handle handle;

    /* Get handle for this driver instance */
    handle = (IOLINK_Handle)&(IOLINK_config[index]);

    if (handle != NULL)
    {
        handle = handle->fxnTablePtr->openFxn(handle, params);
    }

    return (handle);
}
/*
 *  ======== IOLINK_close ========
 */
IOLINK_STATUS IOLINK_close(IOLINK_Handle handle)
{
    IOLINK_STATUS status;

    if (handle == NULL)
    {
        status = IOLINK_STATUS_ERROR;
    }
    else
    {
        status = handle->fxnTablePtr->closeFxn(handle);
    }

    return (status);
}

/*
 *  ======== IOLINK_control ========
 */
IOLINK_STATUS IOLINK_control(IOLINK_Handle handle, uint32_t cmd, void *arg)
{
    IOLINK_STATUS status;

    if (handle == NULL)
    {
        status = IOLINK_STATUS_ERROR;
    }
    else
    {
        status = handle->fxnTablePtr->controlFxn(handle, cmd, arg);
    }

    return (status);
}

/*
 *  ======== IOLINK_Params_init =======
 */
void IOLINK_Params_init(IOLINK_Params *params)
{
    *params = IOLINK_defaultParams;
}
