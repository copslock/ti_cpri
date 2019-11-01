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
 *  \file ipc_priv.h
 *
 *  \brief IPC - private definitions.
 */

#ifndef IPC_PRIV_H_
#define IPC_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief Maximum length of ProcessorName
 */
#define IPC_MAX_PROC_NAMELEN         (32)

/**
 * \brief Maximum EndPoints Supported
 */
#define MAXENDPOINTS           256


/**
 *  \brief Default RPMessage buffer count
 */
#define RPMessage_Buffer_Count_Default		(16)

/**
 *  \brief Maximum value of System Reserved Endpoints
 */
#define RPMessage_MAX_RESERVED_ENDPOINT  100

/**
 *  \brief This structure is for proc info
 */
typedef struct Ipc_ProcInfo_s
{
    uint32_t procId;
    /**< unique processor id */

    char     name[IPC_MAX_PROC_NAMELEN];
    /**< processor name */
}Ipc_ProcInfo;

/**
 *  \brief Instance specific parameter, only 1 instance supported
 */
typedef struct Ipc_Object_s
{
    uint32_t        instId;
    /**< Instance ID */
    Ipc_InitPrms    initPrms;
    /**< Initialization parameters */
    void            *interruptHandle;
    /**< Interrupt handler handle */
} Ipc_Object;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

Ipc_Object *getIpcObjInst(uint32_t instId);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_UTILS_H_ */
