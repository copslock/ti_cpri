/*
 *  Copyright (C) 2019 Texas Instruments Incorporated
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
 *
 */
/**
 *  \file V1/sciclient_soc_priv.h
 *
 *  \brief Private J721E specific SOC file
 */

#ifndef SCICLIENT_SOC_PRIV_H_
#define SCICLIENT_SOC_PRIV_H_

#if defined (BUILD_MCU1_0)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_R5_NONSEC_0)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_R5_SEC_0)
#endif
#if defined (BUILD_MCU1_1)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_R5_NONSEC_1)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_R5_SEC_1)
#endif
#if defined (BUILD_MPU1_0)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_A72_NONSEC_0)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_A72_SEC_0)
#endif
#if defined (BUILD_MCU2_0)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_MAIN_0_R5_NONSEC_0)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_MAIN_0_R5_SEC_0)
#endif
#if defined (BUILD_MCU2_1)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_MAIN_0_R5_NONSEC_1)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_MAIN_0_R5_SEC_1)
#endif
#if defined (BUILD_MCU3_0)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_MAIN_1_R5_NONSEC_0)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_MAIN_1_R5_SEC_0)
#endif
#if defined (BUILD_MCU3_1)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_MAIN_1_R5_NONSEC_1)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_MAIN_1_R5_SEC_1)
#endif
#if defined (BUILD_C7X_1)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_C7X_NONSEC_0)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_C7X_SEC_0)
#endif
#if defined (BUILD_C66X_1)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_C6X_0_NONSEC_0)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_C6X_0_SEC_0)
#endif
#if defined (BUILD_C66X_2)
#define SCICLIENT_CONTEXT_NONSEC    (SCICLIENT_CONTEXT_C6X_1_NONSEC_0)
#define SCICLIENT_CONTEXT_SEC       (SCICLIENT_CONTEXT_C6X_1_SEC_0)
#endif

#endif /* SCICLIENT_SOC_PRIV_H_ */
