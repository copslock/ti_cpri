/**********************************************************************
* Copyright (C) 2012-2019 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 12.03.9e11b77(origin/DRV-3827_extract_sanity_to_c_file)
*          Do not edit it manually.
**********************************************************************
* Cadence Core Driver for the Cadence DP_SD0801 PHY for DisplayPort (DP)
* core. This header file lists the API providing a HAL (hardware
* abstraction layer) interface for the DP_SD0801 "Torrent" core.
**********************************************************************/

/* parasoft-begin-suppress METRICS-18-3 "Follow the Cyclomatic Complexity limit of 10" */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions" */
/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4" */
/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement" */
/* parasoft-begin-suppress MISRA2012-RULE-8_7 "Functions and objects should not be defined with external linkage if they are referenced in only one translation unit" */

/**
 * This file contains sanity API functions. The purpose of sanity functions
 * is to check input parameters validity. They take the same parameters as
 * original API functions and return 0 on success or CDN_EINVAL on wrong parameter
 * value(s).
 */

#ifndef DP_SD0801_SANITY_H
#define DP_SD0801_SANITY_H

#include "cdn_errno.h"
#include "cdn_stdtypes.h"
#include "dp_sd0801_if.h"

uint32_t DP_SD0801_ConfigSF(const DP_SD0801_Config *obj);
uint32_t DP_SD0801_LinkStateSF(const DP_SD0801_LinkState *obj);
uint32_t DP_SD0801_PrivateDataSF(const DP_SD0801_PrivateData *obj);
uint32_t DP_SD0801_VoltageCoefficientSF(const DP_SD0801_VoltageCoefficients *obj);

uint32_t DP_SD0801_SanityFunction1(const DP_SD0801_Config* config, const uint32_t* memReq);
uint32_t DP_SD0801_SanityFunction2(const DP_SD0801_PrivateData* pD, const DP_SD0801_Config* config);
uint32_t DP_SD0801_SanityFunction3(const DP_SD0801_PrivateData* pD);
uint32_t DP_SD0801_SanityFunction4(const DP_SD0801_PrivateData* pD, const DP_SD0801_LinkRate linkRate);
uint32_t DP_SD0801_SanityFunction9(const DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState);
uint32_t DP_SD0801_SanityFunction12(const DP_SD0801_PrivateData* pD, const uint8_t voltageSwing, const uint8_t preEmphasis, const DP_SD0801_VoltageCoefficients* coefficients);
uint32_t DP_SD0801_SanityFunction14(const DP_SD0801_PrivateData* pD, const uint8_t voltageSwing, const uint8_t preEmphasis, const DP_SD0801_VoltageCoefficients* coefficients);
uint32_t DP_SD0801_SanityFunction15(const DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState);

#define DP_SD0801_ProbeSF DP_SD0801_SanityFunction1
#define DP_SD0801_InitSF DP_SD0801_SanityFunction2
#define DP_SD0801_ConfigurePhyAuxCtrSF DP_SD0801_SanityFunction3
#define DP_SD0801_PhyStartUpSF DP_SD0801_SanityFunction4
#define DP_SD0801_PhyInitSF DP_SD0801_SanityFunction3
#define DP_SD0801_PhySetResetSF DP_SD0801_SanityFunction3
#define DP_SD0801_WaitPmaCmnReadySF DP_SD0801_SanityFunction3
#define DP_SD0801_PhyRunSF DP_SD0801_SanityFunction3
#define DP_SD0801_ConfigLaneSF DP_SD0801_SanityFunction9
#define DP_SD0801_SetLinkRateSF DP_SD0801_SanityFunction9
#define DP_SD0801_EnableLanesSF DP_SD0801_SanityFunction9
#define DP_SD0801_GetDefaultCoeffsSF DP_SD0801_SanityFunction12
#define DP_SD0801_GetCoefficientsSF DP_SD0801_SanityFunction12
#define DP_SD0801_SetCoefficientsSF DP_SD0801_SanityFunction14
#define DP_SD0801_ReadLinkStatSF DP_SD0801_SanityFunction15

#endif  /* DP_SD0801_SANITY_H */

/* parasoft-end-suppress MISRA2012-RULE-8_7 */
/* parasoft-end-suppress METRICS-41-3 */
/* parasoft-end-suppress METRICS-39-3 */
/* parasoft-end-suppress METRICS-36-3 */
/* parasoft-end-suppress METRICS-18-3 */
