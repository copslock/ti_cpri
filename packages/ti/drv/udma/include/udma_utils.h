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
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_UTILS_MODULE UDMA Driver UTILS API
 *            This is UDMA driver utilty parameters and API
 *
 *  @{
 */

/**
 *  \file udma_utils.h
 *
 *  \brief UDMA utility API to make TR, get TR descriptor memory requirement.
 */

#ifndef UDMA_UTILS_H_
#define UDMA_UTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor Udma_TrType
 *  \name UDMA TR type
 *
 *  Various UDMA TR type supported
 *
 *  @{
 */
#define UDMA_TR_TYPE_0                  (CSL_UDMAP_TR_FLAGS_TYPE_1D_DATA_MOVE)
#define UDMA_TR_TYPE_1                  (CSL_UDMAP_TR_FLAGS_TYPE_2D_DATA_MOVE)
#define UDMA_TR_TYPE_2                  (CSL_UDMAP_TR_FLAGS_TYPE_3D_DATA_MOVE)
#define UDMA_TR_TYPE_3                  (CSL_UDMAP_TR_FLAGS_TYPE_4D_DATA_MOVE)
#define UDMA_TR_TYPE_4                  (CSL_UDMAP_TR_FLAGS_TYPE_4D_DATA_MOVE_FORMATTING)
#define UDMA_TR_TYPE_5                  (CSL_UDMAP_TR_FLAGS_TYPE_4D_CACHE_WARM)
#define UDMA_TR_TYPE_8                  (CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE)
#define UDMA_TR_TYPE_9                  (CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING)
#define UDMA_TR_TYPE_10                 (CSL_UDMAP_TR_FLAGS_TYPE_2D_BLOCK_MOVE)
#define UDMA_TR_TYPE_11                 (CSL_UDMAP_TR_FLAGS_TYPE_2D_BLOCK_MOVE_REPACKING)
#define UDMA_TR_TYPE_15                 (CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION)
/* @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA get ring size utility API.
 *
 *  \param mode         [IN] Ring mode. Refer \ref tisci_msg_value_rm_ring_mode
 *  \param elemCnt      [IN] Ring element count.
 *  \param elemSize     [IN] Ring element size. Refer \ref Udma_RingElemSize
 *
 *  \return Size of the ring memory required in bytes
 */
uint32_t UdmaUtils_getRingMemSize(uint8_t mode,
                                  uint32_t elemCnt,
                                  uint8_t elemSize);

/**
 *  \brief UDMA utility API to make TRPD. Other TRPD make APIs should be
 *  called only after this API.
 *
 *  Note: This sets the most commonly used  value for all the fields.
 *  User can override the fields after calling this API if required.
 *
 *  \param pTrpd        [IN] Pointer to TRPD memory
 *  \param trType       [IN] TR type. Refer \ref Udma_TrType
 *  \param trCnt        [IN] Number of TR entries present in the TRPD
 *  \param cqRingNum    [IN] Completion ring number where the TRPD is returned
 *                           after processing
 */
static inline void UdmaUtils_makeTrpd(void *pTrpd,
                                      uint32_t trType,
                                      uint32_t trCnt,
                                      uint32_t cqRingNum);

/**
 *  \brief Returns the size of TR (encoded) based on the type
 *
 *  \param trType       [IN] TR type. Refer \ref Udma_TrType
 *
 *  \return Size of the TR record in encoded value to be programmed in TRPD.
 */
uint32_t UdmaUtils_getTrSizeEncoded(uint32_t trType);

/**
 *  \brief Returns the size of TR in bytes based on the type
 *
 *  \param trType       [IN] TR type. Refer \ref Udma_TrType
 *
 *  \return Size of the TR record in bytes.
 */
uint32_t UdmaUtils_getTrSizeBytes(uint32_t trType);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void UdmaUtils_makeTrpd(void *pTrpd,
                                      uint32_t trType,
                                      uint32_t trCnt,
                                      uint32_t cqRingNum)
{
    uint32_t descType = CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR;
    uint32_t trSizeEncoded = UdmaUtils_getTrSizeEncoded(trType);

    /* Setup descriptor */
    CSL_udmapCppi5SetDescType(pTrpd, descType);
    CSL_udmapCppi5TrSetReload((CSL_UdmapCppi5TRPD*)pTrpd, 0U, 0U);
    CSL_udmapCppi5SetPktLen(pTrpd, descType, trCnt);
    CSL_udmapCppi5SetIds(pTrpd, descType, 0U, UDMA_DEFAULT_FLOW_ID); /* Flow ID and Packet ID */
    CSL_udmapCppi5SetSrcTag(pTrpd, 0x0000);
    CSL_udmapCppi5SetDstTag(pTrpd, 0x0000);
    CSL_udmapCppi5TrSetEntryStride((CSL_UdmapCppi5TRPD*)pTrpd, trSizeEncoded);
    CSL_udmapCppi5SetReturnPolicy(
        pTrpd,
        descType,
        CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_ENTIRE_PKT,
        CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_NO,
        CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_TAIL,
        cqRingNum);

    return;
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_UTILS_H_ */

/* @} */
