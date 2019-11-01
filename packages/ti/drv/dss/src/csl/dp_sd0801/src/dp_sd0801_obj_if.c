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

#include "dp_sd0801_obj_if.h"

/* parasoft suppress item METRICS-41-3 "Number of blocks of comments per statement" */

DP_SD0801_OBJ *DP_SD0801_GetInstance(void)
{
    static DP_SD0801_OBJ driver =
    {
        .probe = DP_SD0801_Probe,
        .init = DP_SD0801_Init,
        .configurePhyAuxCtrl = DP_SD0801_ConfigurePhyAuxCtrl,
        .phyStartUp = DP_SD0801_PhyStartUp,
        .PhyInit = DP_SD0801_PhyInit,
        .phySetReset = DP_SD0801_PhySetReset,
        .waitPmaCmnReady = DP_SD0801_WaitPmaCmnReady,
        .phyRun = DP_SD0801_PhyRun,
        .configLane = DP_SD0801_ConfigLane,
        .setLinkRate = DP_SD0801_SetLinkRate,
        .enableLanes = DP_SD0801_EnableLanes,
        .getDefaultCoeffs = DP_SD0801_GetDefaultCoeffs,
        .getCoefficients = DP_SD0801_GetCoefficients,
        .setCoefficients = DP_SD0801_SetCoefficients,
        .readLinkStat = DP_SD0801_ReadLinkStat,
    };

    return &driver;
}
