/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2015-2019
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
 * @file pcie_example_board.h
 *
 */

#ifndef _PCIE_SAMPLE_BOARD_H_
#define _PCIE_SAMPLE_BOARD_H_

#include <ti/osal/osal.h>

void PlatformUnlockMMR(void);
void PlatformPCIESS1ClockEnable(void);
void PlatformPCIESS1PllConfig(void);
void PlatformPCIESSSetPhyMode(void);
void PlatformPCIESS1Reset(void);
void PlatformPCIESS1CtrlConfig(void);
void PlatformPCIESS1PhyConfig(void);
void PlatformPCIESS2ClockEnable(void);
void PlatformPCIESS2CtrlConfig(void);
void PlatformPCIESS2Reset(void);
void PlatformPCIESS2PhyConfig(void);
void PlatformPCIESS1SetDll(void);
void PlatformPCIESS2SetDll(void);
SemaphoreP_Handle PlatformSetupMSIAndINTX (void *handle);
void PlatformPCIE_PERSTn_Reset(uint32_t);
void PlatformPCIE_GPIO_Init(void);

#endif

