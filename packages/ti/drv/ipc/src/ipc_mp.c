/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 *  \file ipc_utils.c
 *
 *  \brief File containing the IPC driver utilities for MultiProc handling.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <ti/drv/ipc/ipc.h>
#include <ti/drv/ipc/include/ipc_mp.h>
#include <ti/drv/ipc/include/ipc_types.h>
#include "ipc_priv.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

typedef struct Ipc_MpConfig_s
{
    uint32_t	selfProcId;
    /**< own processor id */

    char        name[IPC_MAX_PROC_NAMELEN];
    /**< Name of self processor */

    uint16_t    numProcessors;
    /**< Number of Processors */

    Ipc_ProcInfo procInfo[IPC_MAX_PROCS];
    /**< Array of processors Info */
}Ipc_MpConfig;

static Ipc_MpConfig   g_ipcMpConfig;

int32_t Ipc_mpSetConfig(uint32_t selfId, uint16_t numProc, uint32_t procArry[IPC_MAX_PROCS])
{
    int32_t        retVal = IPC_SOK;
    Ipc_MpConfig  *pMpCfg = &g_ipcMpConfig;
    uint32_t       i = 0;
    uint32_t       id;

    if( (selfId >= IPC_MAX_PROCS) || (numProc > IPC_MAX_PROCS) )
    {
        retVal = IPC_EINVALID_PARAMS;
    }
    else
    {
        pMpCfg->selfProcId    = selfId;
        pMpCfg->numProcessors = numProc;
        strncpy(pMpCfg->name, Ipc_getCoreName(selfId), IPC_MAX_PROC_NAMELEN-1);
        pMpCfg->name[IPC_MAX_PROC_NAMELEN-1] = '\0';
        for(i = 0; i < numProc; i++)
        {
            id = procArry[i];
            pMpCfg->procInfo[i].procId = id;
            if(id < IPC_MAX_PROCS)
            {
                strncpy(pMpCfg->procInfo[i].name, Ipc_getCoreName(id), IPC_MAX_PROC_NAMELEN-1);
                pMpCfg->procInfo[i].name[IPC_MAX_PROC_NAMELEN-1] = '\0';
            }
        }
    }

    return retVal;
}

uint32_t Ipc_mpGetId(const char* name)
{
    uint32_t       procId = IPC_MP_INVALID_ID;
    uint16_t       i      = 0;
    Ipc_MpConfig  *pMpCfg = &g_ipcMpConfig;

    if( (NULL == name) || (strlen(name) == 0U) )
    {
        /* Invalid parameter */
   
    }
    else
    {
        for(i = 0; i < pMpCfg->numProcessors; i++)
        {
            if ((pMpCfg->procInfo[i].name != NULL) &&
                (strncmp(name, pMpCfg->procInfo[i].name, IPC_MAX_PROC_NAMELEN) == 0U))
            {
                procId = pMpCfg->procInfo[i].procId;
            }
        }
    }

    return procId;
}

const char* Ipc_mpGetName(uint32_t id)
{
    char          *name = NULL;
    Ipc_MpConfig  *pMpCfg = &g_ipcMpConfig;
    uint16_t       i;

    if(id >= IPC_MAX_PROCS)
    {
        /* TBD : add failure log */
    }    
    else
    {
        if(id ==  pMpCfg->selfProcId)
        {
            name = pMpCfg->name;
        }
        else
        {
            for(i = 0; i < pMpCfg->numProcessors; i++)
            {
                if (pMpCfg->procInfo[i].procId == id)
                {
                    name = (char *)(pMpCfg->procInfo[i].name);
                }
            }
        }
    }

    return (const char*)name;
}

const char* Ipc_mpGetSelfName(void)
{
    Ipc_MpConfig  *pMpCfg = &g_ipcMpConfig;
    return (const char*)pMpCfg->name;
}

uint16_t Ipc_mpGetNumProcessors(void)
{
    return g_ipcMpConfig.numProcessors;
}

uint32_t Ipc_mpGetSelfId(void)
{
    return g_ipcMpConfig.selfProcId; 
}

uint32_t Ipc_mpGetRemoteProcId(uint32_t coreIndex)
{
    uint32_t       remoteId = 0xFFU;
    Ipc_MpConfig  *pMpCfg = &g_ipcMpConfig;

    if(coreIndex < g_ipcMpConfig.numProcessors)
    {
        remoteId = pMpCfg->procInfo[coreIndex].procId;
    }

    return remoteId; 
}
