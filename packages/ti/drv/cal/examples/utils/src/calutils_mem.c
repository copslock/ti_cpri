/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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

/**
 *  \file calutils_mem.c
 *
 *  \brief Frame buffer memory allocator API.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#if defined (BARE_METAL)
#include <stdlib.h>
#else
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/IHeap.h>
#include <ti/sysbios/heaps/HeapMem.h>
#endif
#include <ti/osal/osal.h>
#include <calutils.h>

/* See calutils_mem.h for function documentation  */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#if defined(BARE_METAL)
/** \brief Typedef for Heap memory handle */
typedef void* HeapMem_Handle;
#endif

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief Maximum frame buffer memory pool size */
#define CALUTILS_MEM_FRAME_HEAP_SIZE    (20U * MB)

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Memory pool handle */
HeapMem_Handle gCalUtils_heapMemFrameHandle = NULL;
#if !defined(BARE_METAL)
HeapMem_Struct gCalUtils_heapMemFrameStruct;
#endif

/* Memory pool */
/* Place the frame heap in frame buffer section. */
uint32_t gCalUtils_heapMemFrame[(uint32_t) CALUTILS_MEM_FRAME_HEAP_SIZE /
                              (uint32_t) sizeof (uint32_t)]
                              __attribute__ ((section (".bss:frameBuffer")))
                              __attribute__((aligned(CAL_BUFFER_ALIGNMENT_RECOMMENDED)));

Bool           gCalUtils_memClearBuf;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t CalUtils_memInit(void)
{
    uint32_t memSize;

    memSize = sizeof (gCalUtils_heapMemFrame);

    CalUtils_memInit_internal(gCalUtils_heapMemFrame, memSize);
    return 0;
}

int32_t CalUtils_memInit_internal(uint32_t *pMemAddr, uint32_t memSize)
{
#if !defined (BARE_METAL)
    HeapMem_Params heapMemPrm;

    /* create memory pool heap  */
    HeapMem_Params_init(&heapMemPrm);

    heapMemPrm.buf  = pMemAddr;
    heapMemPrm.size = memSize;

    HeapMem_construct(&gCalUtils_heapMemFrameStruct, &heapMemPrm);
    gCalUtils_heapMemFrameHandle = HeapMem_handle(&gCalUtils_heapMemFrameStruct);
    GT_assert(CalUtilsTrace, gCalUtils_heapMemFrameHandle != NULL);
#endif
    gCalUtils_memClearBuf = (Bool) FALSE;

    return (FVID2_SOK);
}

int32_t CalUtils_memDeInit(void)
{
    /* delete memory pool heap  */
#if !defined (BARE_METAL)
    HeapMem_destruct(&gCalUtils_heapMemFrameStruct);
    gCalUtils_heapMemFrameHandle = NULL;
#endif
    return (FVID2_SOK);
}

int32_t CalUtils_memFrameGetSize(const Fvid2_Format *pFormat,
                               uint32_t             *size,
                               uint32_t             *cOffset)
{
    int32_t  status = FVID2_SOK;
    uint32_t bufferHeight;

    bufferHeight = pFormat->height;
    switch (pFormat->dataFormat)
    {
        case FVID2_DF_RAW08:
        case FVID2_DF_RAW16:
        case FVID2_DF_RAW24:
        case FVID2_DF_YUV422I_YUYV:
        case FVID2_DF_YUV422I_YVYU:
        case FVID2_DF_YUV422I_UYVY:
        case FVID2_DF_YUV422I_VYUY:
        case FVID2_DF_YUV444I:
        case FVID2_DF_RGB24_888:
        case FVID2_DF_BGR24_888:
        case FVID2_DF_RAW_VBI:
        case FVID2_DF_BGRX_4444:
        case FVID2_DF_XBGR_4444:
        case FVID2_DF_AGBR16_4444:
        case FVID2_DF_RGBA16_4444:
        case FVID2_DF_XGBR16_1555:
        case FVID2_DF_AGBR16_1555:
        case FVID2_DF_BGR16_565:
        case FVID2_DF_XBGR24_8888:
        case FVID2_DF_RGBX24_8888:
        case FVID2_DF_ABGR32_8888:
        case FVID2_DF_RGBA32_8888:
        case FVID2_DF_BGRA32_8888:
        case FVID2_DF_ARGB32_8888:
        case FVID2_DF_BAYER_RAW:
        case FVID2_DF_BAYER_GRBG:
        case FVID2_DF_BAYER_RGGB:
        case FVID2_DF_BAYER_BGGR:
        case FVID2_DF_BAYER_GBRG:
            /* for single plane data format's */
            *size = pFormat->pitch[0] * bufferHeight;
            break;

        case FVID2_DF_YUV422SP_UV:
        case FVID2_DF_YUV420SP_UV:
            /* for Y plane  */
            *size = pFormat->pitch[0] * bufferHeight;

            /* cOffset is at end of Y plane  */
            if (NULL != cOffset)
            {
                *cOffset = *size;
            }
            if (pFormat->dataFormat == FVID2_DF_YUV420SP_UV)
            {
                /* C plane height is 1/2 of Y plane */
                bufferHeight = bufferHeight / 2U;
            }

            /* for C plane  */
            *size += (pFormat->pitch[1] * bufferHeight);
            break;

        default:
            /* illegal data format  */
            status = FVID2_EFAIL;
            break;
    }

    /* align size to minimum required frame buffer alignment  */
    *size = CalUtils_align(*size, (uint32_t) CAL_BUFFER_ALIGNMENT_RECOMMENDED);

    return (status);
}

int32_t CalUtils_memFrameAlloc(Fvid2_Format *pFormat,
                             Fvid2_Frame  *pFrame,
                             uint32_t        numFrames)
{
    int32_t  status;
    uint32_t size, cOffset, frameId;
    uint8_t *pBaseAddr;

    /* align height to multiple of 2  */
    pFormat->height = CalUtils_align(pFormat->height, (uint32_t) 2U);

    /* get frame size for given pFormat */
    status = CalUtils_memFrameGetSize(pFormat, &size, &cOffset);
    if (status == FVID2_SOK)
    {
        /* allocate the memory for 'numFrames' */

        /* for all 'numFrames' memory is contigously allocated  */
        pBaseAddr = (uint8_t *) CalUtils_memAlloc(
            (size * numFrames),
            (uint32_t) CAL_BUFFER_ALIGNMENT_RECOMMENDED);
        if (pBaseAddr == NULL)
        {
            status = FVID2_EALLOC;  /* Error in allocation, exit with error */
        }
    }

    if (status == FVID2_SOK)
    {
        /* init memory pointer for 'numFrames'  */
        for (frameId = 0; frameId < numFrames; frameId++)
        {
            /* init Fvid2_Frame to 0's  */
            Fvid2Frame_init(pFrame);

            /* copy chNum to Fvid2_Frame from Fvid2_Format */
            pFrame->chNum      = pFormat->chNum;
            pFrame->addr[0] = (uint64_t) pBaseAddr;

            switch (pFormat->dataFormat)
            {
                case FVID2_DF_RAW08:
                case FVID2_DF_RAW16:
                case FVID2_DF_RAW24:
                case FVID2_DF_RAW_VBI:
                case FVID2_DF_YUV422I_UYVY:
                case FVID2_DF_YUV422I_VYUY:
                case FVID2_DF_YUV422I_YUYV:
                case FVID2_DF_YUV422I_YVYU:
                case FVID2_DF_YUV444I:
                case FVID2_DF_RGB24_888:
                case FVID2_DF_BGR24_888:
                case FVID2_DF_BGRX_4444:
                case FVID2_DF_XBGR_4444:
                case FVID2_DF_AGBR16_4444:
                case FVID2_DF_RGBA16_4444:
                case FVID2_DF_XGBR16_1555:
                case FVID2_DF_AGBR16_1555:
                case FVID2_DF_BGR16_565:
                case FVID2_DF_XBGR24_8888:
                case FVID2_DF_RGBX24_8888:
                case FVID2_DF_ARGB32_8888:
                case FVID2_DF_ABGR32_8888:
                case FVID2_DF_RGBA32_8888:
                case FVID2_DF_BGRA32_8888:
                case FVID2_DF_BAYER_RAW:
                case FVID2_DF_BAYER_GRBG:
                case FVID2_DF_BAYER_RGGB:
                case FVID2_DF_BAYER_BGGR:
                case FVID2_DF_BAYER_GBRG:
                    break;
                case FVID2_DF_YUV422SP_UV:
                case FVID2_DF_YUV420SP_UV:
                    /* assign pointer for C plane */
                    pFrame->addr[3] = (uint64_t) pFrame->addr[0] + cOffset;
                    break;
                default:
                    /* illegal data format  */
                    status = FVID2_EFAIL;
                    break;
            }

            /* go to next frame */
            pFrame++;
            /* increment base address */
            pBaseAddr += size;
        }
    }

    GT_assert(CalUtilsTrace, status == FVID2_SOK);

    return (status);
}

int32_t CalUtils_memFrameFree(const Fvid2_Format *pFormat,
                            Fvid2_Frame        *pFrame,
                            uint32_t              numFrames)
{
    int32_t  status;
    uint32_t size, cOffset;

    /* get frame size for given 'pFormat' */
    status = CalUtils_memFrameGetSize(pFormat, &size, &cOffset);
    if (status == FVID2_SOK)
    {
        /* free the frame buffer memory */

        /* for all 'numFrames' memory is allocated contiguously during alloc,
         *  so first frame memory pointer points to the complete
         *  memory block for all frames */
        CalUtils_memFree((void *)pFrame->addr[0], (size * numFrames));
    }

    return (FVID2_SOK);
}

#if defined (BARE_METAL)
uint8_t *fw_mem_start_phy = (uint8_t*)&gCalUtils_heapMemFrame[0];
uint8_t *fw_mem_start = (uint8_t*)gCalUtils_heapMemFrame;
static uint8_t *fw_mem_end = (uint8_t*)(&gCalUtils_heapMemFrame[((uint32_t) CALUTILS_MEM_FRAME_HEAP_SIZE /
                                                                (uint32_t) sizeof (uint32_t)) - 1U]);
static uint8_t *fw_mem_alloc_ptr = (uint8_t*)gCalUtils_heapMemFrame;
/* Macro to align x to y */
#define align(x,y)   ((x + y) & (~y))

void * CalUtils_alignedMalloc(uint32_t    size,    uint32_t    alignment)
{
  uint8_t *alloc_ptr;
  void *     p_block = (void *) NULL;

  alloc_ptr = (uint8_t*)align((uint32_t)fw_mem_alloc_ptr, alignment);

  if ((alloc_ptr + size) < fw_mem_end)    {
    p_block =(void *)alloc_ptr;
    fw_mem_alloc_ptr = alloc_ptr + size;
  }

    return p_block;
}

void CalUtils_alignedFree(void *p, uint32_t size )
{
  /* Nothing to be done here */
}
#endif

void * CalUtils_memAlloc(uint32_t size, uint32_t align)
{
    void * addr;

    /* allocate memory  */
#if defined (BARE_METAL)
    addr = CalUtils_alignedMalloc(size, align);
#else
    addr = HeapMem_alloc(gCalUtils_heapMemFrameHandle, size, align, NULL);
#endif
    if (((Bool) (addr != NULL)) && gCalUtils_memClearBuf)
    {
        memset(addr, 0x80U, size);
        /* Flush and invalidate the CPU write */
        CacheP_wbInv(addr, size);
    }

    return (addr);
}

int32_t CalUtils_memFree(void *addr, uint32_t size)
{
    /* free previously allocated memory  */
#if defined (BARE_METAL)
   CalUtils_alignedFree(addr, size);
#else
    HeapMem_free(gCalUtils_heapMemFrameHandle, addr, size);
#endif
    return (FVID2_SOK);
}

int32_t CalUtils_memClearOnAlloc(Bool enable)
{
    gCalUtils_memClearBuf = enable;

    return (FVID2_SOK);
}

/**
 *  CalUtils_memGetHeapStat
 *  \brief Returns the current status (free size) of the various heaps used.
 *
 *  \param heapStat [OUT]   Status filled by the function.
 *
 */
void CalUtils_memGetHeapStat(CalUtils_MemHeapStatus *heapStat)
{
    /* NULL pointer check */
    GT_assert(CalUtilsTrace, NULL != heapStat);

    heapStat->freeSysHeapSize  = CalUtils_memGetSystemHeapFreeSpace();
    heapStat->freeBufHeapSize  = CalUtils_memGetBufferHeapFreeSpace();

    return;
}

/**
 *  CalUtils_memCheckHeapStat
 *  \brief Checks the current status of each heap with the value passed.
 *  This returns an error if the values of each of the heap doesn't match.
 *
 *  \param heapStat [IN]    Older status to be compared with the current status.
 *
 *  \return                 Returns 0 if the heap sizes match else return
 *                          FVID2_EFAIL.
 */
int32_t CalUtils_memCheckHeapStat(const CalUtils_MemHeapStatus *heapStat)
{
    int32_t retVal = FVID2_SOK;
    CalUtils_MemHeapStatus curStat;

    /* NULL pointer check */
    GT_assert(CalUtilsTrace, NULL != heapStat);

    CalUtils_memGetHeapStat(&curStat);

    if (heapStat->freeSysHeapSize != curStat.freeSysHeapSize)
    {
        GT_1trace(CalUtilsTrace, GT_CRIT,
                  "Warning: Memory leak (%d bytes) in System Heap!!\r\n",
                  (heapStat->freeSysHeapSize - curStat.freeSysHeapSize));
        retVal = FVID2_EFAIL;
    }
    if (heapStat->freeBufHeapSize != curStat.freeBufHeapSize)
    {
        GT_1trace(CalUtilsTrace, GT_CRIT,
                  "Warning: Memory leak (%d bytes) in Buffer Heap!!\r\n",
                  (heapStat->freeBufHeapSize - curStat.freeBufHeapSize));
        retVal = FVID2_EFAIL;
    }

    return (retVal);
}

uint32_t CalUtils_memGetSystemHeapFreeSpace(void)
{
#if   !defined (BARE_METAL)
    Memory_Stats stats;
    extern const IHeap_Handle Memory_defaultHeapInstance;

    Memory_getStats(Memory_defaultHeapInstance, &stats);

    return ((uint32_t) (stats.totalFreeSize));
#else
    return ((uint32_t) 1U);
#endif
}

uint32_t CalUtils_memGetBufferHeapFreeSpace(void)
{
    uint32_t       totalFreeSize = 0U;
#if   !defined (BARE_METAL)
    Memory_Stats stats;

    if (NULL != gCalUtils_heapMemFrameHandle)
    {
        HeapMem_getStats(gCalUtils_heapMemFrameHandle, &stats);
        totalFreeSize = (uint32_t) stats.totalFreeSize;
    }
#endif
    return (totalFreeSize);
}

