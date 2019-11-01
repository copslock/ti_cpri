/**
 *  \file   sbl_rprc_parse.h
 *
 *  \brief  This file contains function prototypes of RPRC image parse functions.
 *
 */

/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef SBL_RPRC_PARSE_H_
#define SBL_RPRC_PARSE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include "sbl_slave_core_boot.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#ifndef E_PASS
#define E_PASS    ((int32_t)(0))
#endif
#ifndef E_FAIL
#define E_FAIL    (-((int32_t)1))
#endif

#ifndef SBL_SCRATCH_MEM_START
#define SBL_SCRATCH_MEM_START ((uint32_t)0)
#endif

#ifndef SBL_SCRATCH_MEM_SIZE
#define SBL_SCRATCH_MEM_SIZE ((uint32_t)0)
#endif

#define SBL_TCM_ENABLED (0x01)

/* Function Pointer used while reading data from the storage. */
extern int32_t (*fp_readData)(void *dstAddr, void *srcAddr,
                        uint32_t length);

/* Function Pointer used while reading data from the storage. */
extern void  (*fp_seek)(void *srcAddr, uint32_t location);
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief     SBL_SeekMem jump of an offset from the start of the app
 *            image, so that the next read will return dataa from the offset.
 *
 * \param     srcAddr  handle to access the AppImage
 * \param     location offset within the AppImage from where the next
 *            read should reurn data
 *
 * \retval    error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 */
void SBL_SeekMem(void *srcAddr, uint32_t location);

/**
 * \brief     SBL_ReadMem reads a specified number of bytes of data from the
 *            current offset within the AppImage to a user speified memory
 *            buffer. The offset that SBL_ReadMem uses must be set apriori
 *            using SBL_SeekMem
 *
 * \param     buff     dest of the read operation. Must point to a valid
 *                     memory buffer
 * \param     srcAddr  handle to access the AppImage
 * \param     size     number of bytes to read.
 *
 * \retval    error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 */
int32_t SBL_ReadMem(void *buff,void *srcAddr,uint32_t size);

/**
 * \brief     SBL_SetRsvdMem uptes the reserved memory used by the SBL when
 *            parsing signed images - this must be called as a prelude
 *            to the *ImageParse APIs.This memory cannot be used by apps
 *            during load time.
 *
 * \param     scratch_mem    start of memory that can be reserved by SBL
 * \param     scratch_sz     size of memory chunk reserved for SBL use
 *
 * \retval    error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 */
void SBL_SetRsvdMem(uint8_t *scratch_mem, uint32_t scratch_sz);

/**
 * \brief     SBL_SetMulticoreImageImgReadfxn uptes the functions used by
 *            the image parsing routines - this must be called as a prelude
 *            to the *ImageParse APIs so that the APIs can read the images
 *            from teh appropriate boot media.
 *
 * \param     imge_read_fxn  pointer to func that reads an app from boot media
 * \param     image_seek_fxn pointer to func that updates the offset so that
 *                           the next read operation will happen from the new
 *                           from start of the app.
 *
 * \retval    error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 */
void SBL_SetMulticoreImageImgReadfxn(void *imge_read_fxn, void *image_seek_fxn);

/**
 * \brief     SBL_VerifyMulticoreImage function parses the multi-core
 *            app image looking for SoC specifc headers than can be
 *            used to optinally verify the multicore image before processing
 *            it. If such headers are found, they are processed and the handle
 *            is updated so that it now points to the start of the multicoreimage
 *            rather than to the start of the app image.
 *
 * \param     img_handle       reference used to read the app from boot media
 * \param     ImageOffsetPtr   offset to start of image from img_handle
 * \param     scratch_mem      temporary memory reserved for SBL use that's
 *                             defined as compile time
 * \param     scratch_sz       size of buffer reserved for use by SBL
 *
 * \retval    error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 */
int32_t SBL_VerifyMulticoreImage(void **img_handle,
                                 uint32_t *ImageOffsetPtr,
                                 uint8_t *scratch_mem,
                                 uint32_t scratch_sz);

/**
 * \brief     SBL_MulticoreImageParse function parses the multi-core
 *            app image. Read the image header & check for AppImage.
 *            Device ID field confirms the boot device ID. Parses the
 *            meta header & finds the number executables Parses & load
 *            each section into CPU internal memory & external memory
 *
 * \param     srcAddr  Start address of AppImage ImageOffset - Dummy
 *
 * \retval    error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 */
int32_t SBL_MulticoreImageParse(void *srcAddr,
                                uint32_t ImageOffset,
                                sblEntryPoint_t *pAppEntry);

#ifdef __cplusplus
}
#endif

#endif /*SBL_RPRC_PARSE_H_*/

