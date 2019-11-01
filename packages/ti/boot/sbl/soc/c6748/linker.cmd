/*
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

-c
-heap  0x4000
-stack 0x4000

MEMORY
{
    SBL_MEM (RWX)     : o = 0x80000000, l = 0x20000 /* Onchip RAM */
}

SECTIONS
{
    .text          >  SBL_MEM
    .stack         >  SBL_MEM
    .bss           >  SBL_MEM
    .cio           >  SBL_MEM
    .const         >  SBL_MEM
    .data          >  SBL_MEM
    .switch        >  SBL_MEM
    .sysmem        >  SBL_MEM
    .far           >  SBL_MEM
    .args          >  SBL_MEM
    .ppinfo        >  SBL_MEM
    .ppdata        >  SBL_MEM
    .pinit         >  SBL_MEM
    .cinit         >  SBL_MEM
    .binit         >  SBL_MEM
    .init_array    >  SBL_MEM
    .neardata      >  SBL_MEM
    .fardata       >  SBL_MEM
    .rodata        >  SBL_MEM
    .c6xabi.exidx  >  SBL_MEM
    .c6xabi.extab  >  SBL_MEM
}
