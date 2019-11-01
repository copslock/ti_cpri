/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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

/*
 *  \file ipc_am65xx_rsctable.h
 *
 *  \brief Define the resource table entries for all R5F cores. This will be
 *  incorporated into corresponding base images, and used by the remoteproc
 *  on the host-side to allocated/reserve resources.
 *
 */

#ifndef IPC_AM65XX_RSCTABLE_H_
#define IPC_AM65XX_RSCTABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drv/ipc/include/ipc_rsctypes.h>

#define IPC_VRING_BASEADDR             0xA0000000
#define SZ_1M                          0x00100000

#define R5F_MEM_RPMSG_VRING0           (IPC_VRING_BASEADDR)
#define R5F_MEM_RPMSG_VRING1           (R5F_MEM_RPMSG_VRING0+0x10000)

#define MCU1_0_R5F_MEM_RPMSG_VRING0    (IPC_VRING_BASEADDR)
#define MCU1_0_R5F_MEM_RPMSG_VRING1    (MCU1_0_R5F_MEM_RPMSG_VRING0+0x10000)

#define MCU1_1_R5F_MEM_RPMSG_VRING0    (IPC_VRING_BASEADDR+0x01000000)
#define MCU1_1_R5F_MEM_RPMSG_VRING1    (MCU1_1_R5F_MEM_RPMSG_VRING0+0x10000)

#define R5F_MEM_IPC_VRING_SIZE         SZ_1M
#define R5F_NUM_ENTRIES                2

/*
 * Assign fixed RAM addresses to facilitate a fixed MMU table.
 * PHYS_MEM_IPC_VRING & PHYS_MEM_IPC_DATA MUST be together.
 */
/* See CMA BASE addresses in Linux side: arch/arm/mach-omap2/remoteproc.c */
#define PHYS_MEM_IPC_VRING             (IPC_VRING_BASEADDR)
#define MCU1_0_PHYS_MEM_IPC_VRING      (IPC_VRING_BASEADDR)
#define MCU1_1_PHYS_MEM_IPC_VRING      (MCU1_0_PHYS_MEM_IPC_VRING+0x01000000)

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define R5F_RPMSG_VQ0_SIZE             256
#define R5F_RPMSG_VQ1_SIZE             256

/* flip up bits whose indices represent features we support */
#define RPMSG_R5F_C0_FEATURES          1

extern char xdc_runtime_SysMin_Module_State_0_outbuf__A;
#define TRACEBUFADDR ((uintptr_t)&xdc_runtime_SysMin_Module_State_0_outbuf__A)

const Ipc_ResourceTable ti_ipc_remoteproc_ResourceTable __attribute__ ((section (".resource_table"), aligned (4096))) = 
{
    1,                   /* we're the first version that implements this */
    NUM_ENTRIES,         /* number of entries in the table */
    0, 0,                /* reserved, must be zero */

    /* offsets to entries */
    {
        offsetof(Ipc_ResourceTable, rpmsg_vdev),
        offsetof(Ipc_ResourceTable, trace),
    },

    /* rpmsg vdev entry */
    {
        TYPE_VDEV, VIRTIO_ID_RPMSG, 0,
        RPMSG_R5F_C0_FEATURES, 0, 0, 0, 2, { 0, 0 },
        /* no config data */
    },
    /* the two vrings */
#if defined (BUILD_MCU1_0)
    { MCU1_0_R5F_MEM_RPMSG_VRING0, 4096, R5F_RPMSG_VQ0_SIZE, 1, 0 },
    { MCU1_0_R5F_MEM_RPMSG_VRING1, 4096, R5F_RPMSG_VQ1_SIZE, 2, 0 },
#elif defined (BUILD_MCU1_1)
    { MCU1_1_R5F_MEM_RPMSG_VRING0, 4096, R5F_RPMSG_VQ0_SIZE, 1, 0 },
    { MCU1_1_R5F_MEM_RPMSG_VRING1, 4096, R5F_RPMSG_VQ1_SIZE, 2, 0 },
#else
    { R5F_MEM_RPMSG_VRING0, 4096, R5F_RPMSG_VQ0_SIZE, 1, 0 },
    { R5F_MEM_RPMSG_VRING1, 4096, R5F_RPMSG_VQ1_SIZE, 2, 0 },
#endif

    {
        (TRACE_INTS_VER0 | TYPE_TRACE), TRACEBUFADDR, 0x80000, 0, "trace:r5f0",
    },
};


#ifdef __cplusplus
}
#endif

#endif /* IPC_AM65XX_RSCTABLE_H_ */
