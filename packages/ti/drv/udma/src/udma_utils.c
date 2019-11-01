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
 *  \file udma_utils.c
 *
 *  \brief File containing the UDMA driver utility functions.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#if defined (UDMA_CFG_PRINT_ENABLE)
/* This is needed for vsnprintf */
#include <stdio.h>
#include <stdarg.h>
#endif
#include <ti/drv/udma/src/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    uint32_t trType;
    uint32_t trSize;
    uint32_t trSizeEncoded;
} UdmaUtilsTrSizeTable;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* \brief TR size table */
static UdmaUtilsTrSizeTable gUdmaUtilsTrSizeTable[] =
{
    {UDMA_TR_TYPE_0,  16U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_16B},
    {UDMA_TR_TYPE_1,  32U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_32B},
    {UDMA_TR_TYPE_2,  32U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_32B},
    {UDMA_TR_TYPE_3,  32U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_32B},
    {UDMA_TR_TYPE_4,  64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_5,  64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_8,  64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_9,  64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_10, 64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_11, 64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B},
    {UDMA_TR_TYPE_15, 64U, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B}
};

#define UDMA_UTILS_NUM_TR_TYPE          ((sizeof (gUdmaUtilsTrSizeTable)) / \
                                         (sizeof (UdmaUtilsTrSizeTable)))

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

uint32_t UdmaUtils_getRingMemSize(uint8_t mode,
                                  uint32_t elemCnt,
                                  uint8_t elemSize)
{
    uint32_t    ringMemSize;

    ringMemSize = ((uint32_t) 1U << (elemSize + 2U));   /* Element size in bytes */
    ringMemSize *= elemCnt;
    /* In the case of a credentials mode or qm mode, each ring write
     * results in the ring occupancy increasing by 2 elements (one entry for
     * the credentials, one entry for the data) */
    if((TISCI_MSG_VALUE_RM_RING_MODE_CREDENTIALS == mode) ||
       (TISCI_MSG_VALUE_RM_RING_MODE_QM == mode))
    {
        ringMemSize <<= 1U;
    }

    return (ringMemSize);
}

uint32_t UdmaUtils_getTrSizeEncoded(uint32_t trType)
{
    uint32_t i, trSizeEncoded = CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B;

    for(i=0; i<UDMA_UTILS_NUM_TR_TYPE; i++)
    {
        if(gUdmaUtilsTrSizeTable[i].trType == trType)
        {
            trSizeEncoded = gUdmaUtilsTrSizeTable[i].trSizeEncoded;
            break;
        }
    }

    return (trSizeEncoded);
}

uint32_t UdmaUtils_getTrSizeBytes(uint32_t trType)
{
    uint32_t i, trSize = 64U;

    for(i=0; i<UDMA_UTILS_NUM_TR_TYPE; i++)
    {
        if(gUdmaUtilsTrSizeTable[i].trType == trType)
        {
            trSize = gUdmaUtilsTrSizeTable[i].trSize;
            break;
        }
    }

    return (trSize);
}

uint64_t Udma_virtToPhyFxn(const void *virtAddr,
                           Udma_DrvHandle drvHandle,
                           Udma_ChHandle chHandle)
{
    uint32_t    chNum = UDMA_DMA_CH_INVALID;
    void *      appData = NULL_PTR;
    uint64_t    phyAddr;

    if(NULL_PTR != chHandle)
    {
        chNum   = chHandle->chPrms.chNum;
        appData = chHandle->chPrms.appData;
    }

    if((Udma_VirtToPhyFxn) NULL_PTR != drvHandle->initPrms.virtToPhyFxn)
    {
        phyAddr = drvHandle->initPrms.virtToPhyFxn(virtAddr, chNum, appData);
    }
    else
    {
        phyAddr = Udma_defaultVirtToPhyFxn(virtAddr, chNum, appData);
    }

    return (phyAddr);
}

void *Udma_phyToVirtFxn(uint64_t phyAddr,
                        Udma_DrvHandle drvHandle,
                        Udma_ChHandle chHandle)
{
    uint32_t    chNum = UDMA_DMA_CH_INVALID;
    void *      appData = NULL_PTR;
    void *      virtAddr;

    if(NULL_PTR != chHandle)
    {
        chNum   = chHandle->chPrms.chNum;
        appData = chHandle->chPrms.appData;
    }

    if((Udma_VirtToPhyFxn) NULL_PTR != drvHandle->initPrms.virtToPhyFxn)
    {
        virtAddr = drvHandle->initPrms.phyToVirtFxn(phyAddr, chNum, appData);
    }
    else
    {
        virtAddr = Udma_defaultPhyToVirtFxn(phyAddr, chNum, appData);
    }

    return (virtAddr);
}

void Udma_printf(Udma_DrvHandle drvHandle, const char *format, ...)
{
#if defined (UDMA_CFG_PRINT_ENABLE)
    va_list     vaArgPtr;
    char       *buf;

    if((drvHandle != NULL_PTR) && (drvHandle->initPrms.printFxn != (Udma_PrintFxn) NULL_PTR))
    {
        if((Udma_OsalMutexLockFxn) NULL_PTR != drvHandle->initPrms.osalPrms.lockMutex)
        {
            drvHandle->initPrms.osalPrms.lockMutex(drvHandle->printLock);
        }

        buf = &drvHandle->printBuf[0];
        (void) va_start(vaArgPtr, format);
        (void) vsnprintf(
            buf, UDMA_CFG_PRINT_BUF_LEN, (const char *) format, vaArgPtr);
        va_end(vaArgPtr);

        drvHandle->initPrms.printFxn("[UDMA] ");
        drvHandle->initPrms.printFxn(buf);

        /* This assumes that both lock/unlock will be both provided or not
         * provided. Any other combo will result in invalid lock operation */
        if((Udma_OsalMutexUnlockFxn) NULL_PTR != drvHandle->initPrms.osalPrms.unlockMutex)
        {
            drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->printLock);
        }
    }
#endif

    return;
}
