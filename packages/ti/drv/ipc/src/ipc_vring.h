/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 */

/* An interface for efficient virtio implementation, currently for use by KVM
 * and lguest, but hopefully others soon.  Do NOT change this since it will
 * break existing servers and clients.
 *
 * This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of IBM nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL IBM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Copyright Rusty Russell IBM Corporation 2007. */

/**
 *  \file ipc_vring.h
 *
 *  \brief virtio ring definitions.
 */

#ifndef IPC_VRING_H_
#define IPC_VRING_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/**
 *  \brief This marks a buffer as continuing via the next field. 
 */
#define VRING_DESC_F_NEXT   1U

/**
 *  \brief This marks a buffer as write-only (otherwise read-only). 
 */
#define VRING_DESC_F_WRITE  2U

/** 
 *  \brief The Host uses this in used->flags to advise the Guest: don't kick 
 *         me when you add a buffer.  It's unreliable, so it's simply an 
 *         optimization.  Guest will still kick if it's out of buffers. 
 */
#define VRING_USED_F_NO_NOTIFY  1U

/**
 *  \brief The Guest uses this in avail->flags to advise the Host: don't 
 *         interrupt me when you consume a buffer.  It's unreliable, so it's 
 *         simply an optimization.  
 */
#define VRING_AVAIL_F_NO_INTERRUPT  1U


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Virtio ring descriptors: 16 bytes.  These can chain together via 
 *         "next". 
 */
struct vring_desc
{
    uint32_t addr;      
    /**< Address (guest-physical). */

    uint32_t padding; 
    /**< Because 64 bits is originally used for addr */

    uint32_t len;
    /**< Length. */

    uint16_t flags;
    /**< The flags as indicated above. Optional for now!*/

    uint16_t next;
    /**< We chain unused descriptors via this, too */
};


struct vring_avail
{
    uint16_t flags;
    uint16_t idx;
    uint16_t ring[256];
};

/**
 *  \brief u32 is used here for ids for padding reasons. 
 */
struct vring_used_elem
{
    uint32_t id;
    /**< Index of start of used descriptor chain. */

    uint32_t len;
    /**< Total length of the descriptor chain which was used (written to) */
};

struct vring_used
{
    uint16_t flags;
    uint16_t idx;
    struct vring_used_elem ring[256];
};

struct VRing
{
    uint32_t num;
    struct vring_desc *desc;
    struct vring_avail *avail;
    struct vring_used *used;
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void vring_init(struct VRing *vr, uint32_t num, void *p,
                              uint32_t pagesize)
{
    vr->num = num;
    vr->desc = (struct vring_desc *) p;
    vr->avail = (struct vring_avail *)
                    ((uintptr_t)p + (num * sizeof(struct vring_desc)));
    vr->used = (struct vring_used *)(((uintptr_t)&vr->avail->ring[num] + pagesize-1)
                & ~(pagesize - 1));
}

static inline uint32_t vring_size(uint32_t num, uint32_t pagesize)
{
    return ((sizeof(struct vring_desc) * num + sizeof(uint16_t) * (2 + num)
                + pagesize - 1) & ~(pagesize - 1))
                + sizeof(uint16_t) * 2 + sizeof(struct vring_used_elem) * num;
}


#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_VRING_H_ */

