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
 *  \ingroup DRV_IPC_MODULE
 *  \defgroup DRV_IPC_MP_MODULE IPC Driver MultiProc configuration
 *            This is documentation for MultiProc configuration used by IPC
 *
 *  @{
 */


/**
 *  \file ipc_mp.h
 *
 *  \brief IPC - Multiproc interface
 */

#ifndef IPC_MP_H_
#define IPC_MP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/drv/ipc/soc/ipc_soc.h>
#include <ti/drv/ipc/include/ipc_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
 *  \brief Invalid processor id.
 */
#define IPC_MP_INVALID_ID            (0xFFFFFFFFU)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Sets the multiproc configuration.
 *
 *  sets the current current processor, number of processor, 
 *  and array of processors participating
 *
 *  \param selfId    [IN] Self ProcId
 *  \param numProc   [IN] Number of core in application
 *  \param procArry  [IN] Array of processorId in application
 *
 *  \return #IPC_SOK for success
            #IPC_MP_INVALID_ID for invalid parameter
 *
 */
int32_t Ipc_mpSetConfig(uint32_t selfId, uint16_t numProc, uint32_t procArry[IPC_MAX_PROCS]);

/**
 *  \brief Gets the MultiProc id.
 *
 *  Retrieves the MultiProc id for the processor with corresponding MultiProc
 *  name. #IPC_MP_INVALID_ID is returned if the name was not found.
 *
 *  \param name    [IN] Name of the processor.
 *
 *  \return ProcId if valid core name, else returns -1
 *
 */
uint32_t Ipc_mpGetId(const char* name);

/**
 *  \brief Gets the name of a processor.
 *
 *  \param id    [IN] MultiProc id.
 *
 *  \return Name of the processor
 *          The returned string should never be modified.
 *
 */
const char* Ipc_mpGetName(uint32_t id);

/**
 *  \brief      Gets the number of processors.
 *
 *  \return     Number of processors configured with MultiProc
 */
uint16_t Ipc_mpGetNumProcessors(void);


/**
 *  \brief Gets executing processor's MultiProc id.
 *
 *  \return Executing processor's id
 *
 */
uint32_t Ipc_mpGetSelfId(void);

/**
 *  \brief    Get the processor name for current processor
 *
 */
const char* Ipc_mpGetSelfName(void);

/**
 *  \brief  Returns procId of the given index
 *
 */
uint32_t Ipc_mpGetRemoteProcId(uint32_t coreIndex);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_MP_H_ */

/* @} */
