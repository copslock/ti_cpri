/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* This file is a copied and modified version of the J7ES.cmd provided from 
 * bios_6_73_00_12/packages/ti/sysbios/platforms/c6x/include/
 *
 * Default linker command file places all sections into L2SRAM which causes
 * linker failures in the event of an entire program being larger than 288KB.
 *
 * Eventually this file should be deleted when J7ES platform is fully supported
 * by the BIOS package.
 */

#define DDR0_ALLOCATED_START  0xA0000000
#define C66x1_ALLOCATED_START DDR0_ALLOCATED_START + 0x06000000
#define C66x1_IPC_ALLOC_START DDR0_ALLOCATED_START + 0x07000000 /* IPC region separate for proper caching */

#define C66x1_EXT_DATA_BASE   (C66x1_ALLOCATED_START + 0x00100000)
#define C66x1_MEM_TEXT_BASE   (C66x1_ALLOCATED_START + 0x00200000)
#define C66x1_MEM_DATA_BASE   (C66x1_ALLOCATED_START + 0x00300000)
#define C66x1_DDR_SPACE_BASE  (C66x1_ALLOCATED_START + 0x00400000)

MEMORY
{
    L2SRAM:         o = 0x00800000            l = 0x00048000   /* 288KB LOCAL L2/SRAM */
    L1PSRAM:        o = 0x00E00000            l = 0x00008000   /* 32KB LOCAL L1P/SRAM */
    L1DSRAM:        o = 0x00F00000            l = 0x00008000   /* 32KB LOCAL L1D/SRAM */
    DDR0_RESERVED   o = 0x80000000            l = 0x20000000   /* 512MB Reserved for A72 OS */
    C66X1_IPC_D     o = C66x1_IPC_ALLOC_START l = 0x00100000   /*  1MB DDR */
    C66X1_EXT_D     o = C66x1_EXT_DATA_BASE   l = 0x00100000   /*  1MB DDR */
    C66X1_TEXT      o = C66x1_MEM_TEXT_BASE   l = 0x00100000   /*  1MB DDR */
    C66X1_DATA      o = C66x1_MEM_DATA_BASE   l = 0x00100000   /*  1MB DDR */
    C66X1_DDR_SPACE o = C66x1_DDR_SPACE_BASE  l = 0x00C00000   /* 12MB DDR */
}

--entry_point=ti_sysbios_family_c64p_Hwi0
--diag_suppress=10063
--retain="*(xdc.meta)"

-stack  0x4000                              /* SOFTWARE STACK SIZE           */
-heap   0x8000                              /* HEAP AREA SIZE                */

/* Set L1D, L1P and L2 Cache Sizes */
ti_sysbios_family_c66_Cache_l1dSize = 32768;
ti_sysbios_family_c66_Cache_l1pSize = 32768;
ti_sysbios_family_c66_Cache_l2Size = 32768;

SECTIONS
{
    xdc.meta (COPY):    { } > L2SRAM
    .vecs:               load >  C66X1_TEXT ALIGN(0x400)
    .text:_c_int00       load >  C66X1_TEXT ALIGN(0x400)
    .text:                >  C66X1_DDR_SPACE
    .stack:               >  C66X1_DDR_SPACE
    GROUP:                >  C66X1_DDR_SPACE
    {
        .bss:
        .neardata:
        .rodata:
    }
    .cio:                >  C66X1_DDR_SPACE
    .const:              >  C66X1_DDR_SPACE
    .data:               >  C66X1_DDR_SPACE
    .switch:             >  C66X1_DDR_SPACE
    .sysmem:             >  C66X1_DDR_SPACE
    .far:                >  C66X1_DDR_SPACE
    .args:               >  C66X1_DDR_SPACE
    .ppinfo:             >  C66X1_DDR_SPACE
    .ppdata:             >  C66X1_DDR_SPACE
    .ti.decompress:      >  C66X1_DDR_SPACE
    .ti.handler_table:   >  C66X1_DDR_SPACE

    /* COFF sections */
    .pinit:              >  C66X1_DDR_SPACE
    .cinit:              >  C66X1_DDR_SPACE

    /* EABI sections */
    .binit:              >  C66X1_DDR_SPACE
    .init_array:         >  C66X1_DDR_SPACE
    .fardata:            >  C66X1_DDR_SPACE
    .c6xabi.exidx:       >  C66X1_DDR_SPACE
    .c6xabi.extab:       >  C66X1_DDR_SPACE

    .csl_vect:           > C66X1_DDR_SPACE
  
    ipc_data_buffer:     > C66X1_DDR_SPACE type=NOLOAD
    .resource_table: { __RESOURCE_TABLE = .;} > C66X1_EXT_D
}
