/******************************************************************************
 * Copyright (c) 2017 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/
/*
 *  Linker command file
 *
 */

-c
-heap  0x8000
-stack 0x8000

/* Memory Map 1 - the default */
MEMORY
{
    L1PSRAM (RWX)  : org = 0x0E00000, len = 0x7FFF
    L1DSRAM (RWX)  : org = 0x0F00000, len = 0x7FFF 

    L2SRAM (RWX)   : org = 0x0800000, len = 0x40000
}

SECTIONS
{
    .csl_vect   >       L2SRAM
    .cppi       >       L2SRAM
    .linkram    >       L2SRAM
    .mac_buffer >       L2SRAM
    .text       >       L2SRAM
    GROUP (NEAR_DP)
    {
    .neardata
    .rodata 
    .bss
    } load > L2SRAM
    .stack      >       L2SRAM
    .cinit      >       L2SRAM
    .cio        >       L2SRAM
    .const      >       L2SRAM
    .data       >       L2SRAM
    .switch     >       L2SRAM
    .sysmem     >       L2SRAM
    .far        >       L2SRAM
    .testMem    >       L2SRAM
    .fardata    >       L2SRAM
}
