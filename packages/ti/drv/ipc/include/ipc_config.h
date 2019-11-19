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
 *  \defgroup DRV_IPC_CFG_MODULE IPC Driver Configurations
 *            This is documentation for IPC driver configuration
 *
 *  @{
 */

/**
 *  \file ipc_config.h
 *
 *  \brief configurations for ipc module.
 */

#ifndef IPC_CONFIG_H_
#define IPC_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \brief Control End Point
 *          End Point used to communicate control messages.
 *          Primarily used to communicate available end point on a processor ID
 */
#define IPC_CTRL_ENDPOINT_ID            (53U)

/**
 * \brief Size of VRing. There are 256 buffer of 512 bytes 
 * Allocate same amount for book keeping.
 */
#define  IPC_VRING_SIZE                 (0x20000U)

#ifdef BUILD_C7X_1
#define  IPC_TASK_STACKSIZE  0x4000U
#else
#define  IPC_TASK_STACKSIZE  0x2000U
#endif

/** \brief SCICLIENT API timeout */
#define IPC_SCICLIENT_TIMEOUT              (SCICLIENT_SERVICE_WAIT_FOREVER)


#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_CONFIG_H_ */

/* @} */
