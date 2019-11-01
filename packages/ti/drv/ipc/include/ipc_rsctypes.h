/*
 * Copyright (c) 2011-2016, Texas Instruments Incorporated
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

/**
 *  \ingroup DRV_IPC_MODULE
 *  \defgroup DRV_IPC_RSCTBLE_MODULE IPC Driver Resource Table configuration
 *            This is documentation for Resource Table configuratin for
 *            remote core as A72 running Linux.
 *
 *  @{
 */

/*
 *  \file ipc_rsctypes.h
 *
 *  \brief Include common definitions for sizes and type of resources
 *  used by the the resource table in each base image, which is
 *  read from remoteproc on host side.
 *
 */

#ifndef IPC_RSCTYPES_H_
#define IPC_RSCTYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drv/ipc/include/ipc_types.h>

/* Virtio Ids: keep in sync with the linux "include/linux/virtio_ids.h" */
/** \brief Virtio type console used for Linux remoteproc */
#define VIRTIO_ID_CONSOLE       3

/** \brief Virtio type as Remote Proc Messaging */
#define VIRTIO_ID_RPMSG         7

/* Indices of rpmsg virtio features we support */
/** \brief RP supports name service notifications */
#define VIRTIO_RPMSG_F_NS       0
/** \brief Support symmetric vring */
#define VIRTIO_RING_F_SYMMETRIC 30

/* Resource info: Must match include/linux/remoteproc.h: */
/** \brief carvout type */
#define TYPE_CARVEOUT    0
/** \brief Devmem type */
#define TYPE_DEVMEM      1
/** \brief trace type */
#define TYPE_TRACE       2
/** \brief VDEV type */
#define TYPE_VDEV        3

/** \brief Number of e ntries */
#define NUM_ENTRIES      2

/** \brief Name Length */
#define NAME_LEN         32


#define TRACE_INTS_VER0        (0 << 16)
#define TRACE_INTS_VER1        (1 << 16)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief IPC Resource Table Header
 */
typedef struct
{
    uint32_t ver;
    /**< Version Number */

    uint32_t num;
    /**< Number of Device */

    uint32_t reserved[2];
    /**< Reserved for future use */
}Ipc_Hdr;

/**
 * \brief Structure used for remoteproc trace
 */
typedef struct
{
    uint32_t  type;
    /**< Type of trace  */

#ifdef BUILD_C7X_1
    uint64_t  da;
#else
    uint32_t  da;
#endif
    /**< Device Address  */

    uint32_t  len;
    /**< Length of buffer  */

    uint32_t  reserved;
    /**< Reserved for future use  */

    uint8_t   name[NAME_LEN];
    /**<  Name of the trace */
}Ipc_Trace;

/**
 * \brief Resource Table Device VRing Structure
 */
typedef struct
{
    uint32_t  da;
    /**< device address */

    uint32_t  align;
    /**< Alignment of the buffer. */

    uint32_t  num;
    /**< Number of buffers */

    uint32_t  notifyid;
    /**< NotifyId for receive channel */

    uint32_t  reserved;
    /**< Reserved for future use */

}Ipc_VDevVRing;

/**
 *  \brief VDEV structure. Must match with Linux
 */
typedef struct
{
    uint32_t  type;
    /**< type of VDEV */

    uint32_t  id;
    /**< ID of VDEV */

    uint32_t  notifyid;
    /**< NotifiedId  */

    uint32_t  dfeatures;
    /**< Not used */

    uint32_t  gfeatures;
    /**< not used */

    uint32_t  config_len;
    /**< Length of configuration */

    uint8_t   status;
    /**< Status of VDev. It is updated by remote proc during loading */

    uint8_t   num_of_vrings;
    /**< number of vrings */

    uint8_t   reserved[2];
    /**< Reserved for future use */

}Ipc_VDev;

/**
 *  \brief IPC Resource Table used by IPC app
 */
typedef struct
{
    Ipc_Hdr base;
    /**< Header Information */

    uint32_t offset[NUM_ENTRIES];  
    /**< offset, Should match 'num' in actual definition */

    Ipc_VDev       rpmsg_vdev;
    /**< RPMessage vDev Entry */

    Ipc_VDevVRing  rpmsg_vring0;
    /**< 1st - VRing  */

    Ipc_VDevVRing  rpmsg_vring1;
    /**< 2nd VRing */

    /* trace entry */
    Ipc_Trace     trace;
    /**< Trace used by remote proc  */

}Ipc_ResourceTable;


#ifdef __cplusplus
}
#endif

#endif /* IPC_RSCTYPES_H_ */

/* @} */
