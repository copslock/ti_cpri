/*
 *
 * Copyright (c) 2018 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

-c
-heap  0x20000
-stack 0x20000
--args 0x1000
--diag_suppress=10068 /* "no matching section" */
--cinit_compression=off
--retain="*(xdc.meta)"

MEMORY
{
    L2SRAM (RWX):  org = 0x64800000,          len = 0x00070000
	DDR0   (RWIX)  : ORIGIN = 0xA8000000, LENGTH = 0x08000000  /* 128 MB per core */
    MSMC3       (RWIX)  : ORIGIN = 0x70400000, LENGTH = 0x00080000  /* 512 KB per core */
}

SECTIONS
{
    xdc.meta (COPY):    { } > DDR0
    boot:
    {
      boot.*<boot.oe71>(.text)
    } load > DDR0 ALIGN(0x200000)
    .vecs       >       DDR0 ALIGN(0x400000)
    .text       >       DDR0 ALIGN(0x200000)

    .bss        >       DDR0  /* Zero-initialized data */
    .data       >       DDR0  /* Initialized data */

    .cinit      >       DDR0  /* could be part of const */
    .init_array >       DDR0  /* C++ initializations */
    .stack      >       DDR0 ALIGN(0x2000)
    .args       >       DDR0
    .cio        >       DDR0
    .const      >       DDR0
    .switch     >       DDR0 /* For exception handling. */
    .sysmem     >       DDR0 /* heap */

    GROUP:              >  DDR0
    {
        .data.ti_sysbios_family_c7x_Mmu_tableArray     : type=NOINIT
        .data.ti_sysbios_family_c7x_Mmu_tableArraySlot : type=NOINIT
        .data.ti_sysbios_family_c7x_Mmu_level1Table    : type=NOINIT
    }

    ipc_data_buffer:       > DDR0
    .resource_table: { __RESOURCE_TABLE = .;} > DDR0

    .udma_buffer_msmc       >       MSMC3
    .udma_buffer_ddr        >       DDR0
    .udma_buffer_internal   >       L2SRAM
    .udma_buffer_ospi       >       DDR0

}
