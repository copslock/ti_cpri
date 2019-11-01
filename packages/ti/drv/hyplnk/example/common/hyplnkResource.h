/*
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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

/*
 * This file contains headers and prototypes that differ between compilation
 * for ARM versus compilation for DSP
*/

#ifdef __ARMv7
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#else
#include <c6x.h>
#include "hyplnkIsr.h"
#include <ti/csl/csl_cacheAux.h>
#endif
#include <string.h>
#include <stdio.h>
#include <ti/drv/hyplnk/device/hyplnk_device.h>
#include "hyplnkLLDCfg.h"
#define System_printf printf

#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_pscAux.h>

#ifdef _VIRTUAL_ADDR_SUPPORT
uint32_t hyplnk_mmap(uint32_t addr, uint32_t size);
#endif

uint32_t hyplnk_memAllocInit();
uint32_t hyplnk_memRelease();
uint32_t hyplnk_mmap(uint32_t addr, uint32_t size);
void hyplnkPSCSetup(uint32_t pwrDmnNum, uint32_t lpscNum);
void hyplnkPSCDisable(uint32_t pwrDmnNum, uint32_t lpscNum);
