/**
 *  \file linker.cmd
 *
 *  \brief This file is the linker script for am5x required for building
 *         SBL with the gcc toolchain.
 *
 */

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

MEMORY
{
    DDR0 :   o = 0x80000400,  l = (0x40000000 - 0x400) /* external DDR Bank 0 */
    OCMC_RAM1 (RWIX): org = 0x40300000 ,  len = 0x0001FFFF  /* OCMCRAM1 region meant for IO delay relocation */
    MMU_TABLE (RW): org = 0x40370000, len = 0x10000 /* OCMCRAM1 region meant to hold MMU table */
    DSP1TRAMPOLINE (RWIX): org = 0x40330000, len = 0x800 /* Memory area reserved for DSP1 Trampoline */
    DSP2TRAMPOLINE (RWIX): org = 0x40330800, len = 0x800 /* Memory area reserved for DSP2 Trampoline */
    SBL_MEM  (RWIX): org = 0x40331000, len = 0x0004EFFF  /* OCMCRAM1 Region meant for SBL */
    OCMC_RAM2 (RWIX): org = 0x40400000  , len = 0x00100000  /* OCMC RAM        */
    OCMC_RAM3 (RWIX): org = 0x40500000 ,  len = 0x00100000  /* OCMC RAM        */
}

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

SECTIONS
{
        .startcode :
        {
            . = ALIGN(4);
            *(.public.*)
            *sbl_init.ao (.text)
        } >SBL_MEM

        .text :
        {
            . = ALIGN(4);
            *(.text*)
            *(.rodata*)
        } >SBL_MEM

        .data :
        {
            . = ALIGN(4);
            *(.data*)
        } >SBL_MEM

        .bss :
        {
            . = ALIGN(4);
            _bss_start = .;
            *(.bss*)
            *(COMMON)
            _bss_end = .;
        } >SBL_MEM

        .heap :
        {
            . = ALIGN(4);
            __end__ = .;
            end = __end__;
            __HeapBase = __end__;
            *(.heap*)
            . = . + 0x400;
            __HeapLimit = .;
        } >SBL_MEM

        .stack :
        {
            . = ALIGN(8);
            __StackLimit = . ;
            *(.stack*)
            . = . + 0x10000;
            __StackTop = .;
        } >SBL_MEM
        _stack = __StackTop;
        
        SBL_MMU_TABLE :
        {
            *(SBL_MMU_TABLE*)
        } > MMU_TABLE
}

