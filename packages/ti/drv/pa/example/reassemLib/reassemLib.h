/**  
 * @file reassemLib.h
 *
 * @brief 
 *  Holds all the constants, structures and API definitions required by the 
 *  IP Reassembly sample code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2011-2013, Texas Instruments, Inc.
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
 *
*/
#ifndef _reassemLib_H_
#define _reassemLib_H_

#ifdef __cplusplus
extern "C" {
#endif

/* C Standard library Include */
#include <stdint.h>
#include <string.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* PA LLD include */
#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pasahost.h>

/* SA LLD Include Files. */
#ifdef _INCLUDE_SA
#include <ti/drv/sa/salld.h>
#endif

/******************************************************************************
 * FUNCTION PURPOSE: Read 8 bit value from 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Returns 8 bit value from 16 bit word.  Assumes nothing.
 * 
 * uint16_t pktRead8bits (
 *    uint8_t *base,       - Base of byte array
 *    uint16_t byteOffset); - Byte offset to read
 * 
 *****************************************************************************/
static inline uint16_t pktRead8bits (uint8_t *base, uint16_t byteOffset) 
{
  char *src = (char *)base;
  char wvalue = *(src + byteOffset);
  uint16_t readByte = (uint16_t)(wvalue & 0xFF);
  return readByte;
} /* pktRead8bits */

/******************************************************************************
 * FUNCTION PURPOSE: Write 8 bit value into 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Writes 8 bit value into 16 bit word; nothing assumed.
 * 
 * void pktWrite8bits (
 *    uint8_t *base,      - Base of byte array
 *    uint16_t byteOffset, - byte offset to write
 *    uint16_t val)        - Byte in low 8 bits of val
 * 
 *****************************************************************************/
static inline void pktWrite8bits (uint8_t *base, uint16_t byteOffset, uint16_t val)
{
  char *wptr = ((char *)base + byteOffset);
  *wptr = (char)(val & 0xFF);
} /* pktWrite8bits */

/******************************************************************************
 * FUNCTION PURPOSE: Read 16 bit value from 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Returns 16 bit value from 16 bit word.  No assumptions.
 * 
 * uint16_t pktRead16bits (
 *    uint8_t *base,       - Base of byte array
 *    uint16_t byteOffset); - Byte offset to read, assumed to be even
 * 
 *****************************************************************************/
static inline uint16_t pktRead16bits (uint8_t *base, uint16_t byteOffset) 
{
  char *wptr = ((char *)base + byteOffset);
  uint16_t ret;

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  ret = (((uint16_t)wptr[0]) << 8) | (wptr[1] & 0xFF);

  return ret;
} /* pktRead16bits */

/******************************************************************************
 * FUNCTION PURPOSE: Write 16 bit value into 16 bit word (macro version)
 ******************************************************************************
 * DESCRIPTION: Writes 16 bit value into 16 bit word.  No assumptions
 * 
 * void pktWrite16bits (
 *    uint8_t *base,      - Base of byte array
 *    uint16_t byteOffset, - byte offset to write; assumed to be even
 *    uint16_t val)        - 16 bit val
 * 
 *****************************************************************************/
static inline void pktWrite16bits (uint8_t *base, uint16_t byteOffset, uint16_t val) 
{
  char *wptr = ((char *)base + byteOffset);

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  wptr[0] = (char)(val>>8);
  wptr[1] = (char)(val & 0xff);

} /* pktWrite16bits */



#if defined(_LITTLE_ENDIAN)
  #define SWIZ(x)  (sizeof((x)) == 1 ? (x) : (sizeof((x)) == 2 ? swiz16((x)) : (sizeof((x)) == 4 ? swiz32((x)) : 0)))
#else
  #define SWIZ(x)  (x)
#endif

/* IPv4 definitions */
#define IPV4_VER_MASK                   0xF0
#define IPV4_VER_VALUE                  0x40
#define IPV4_HLEN_MASK                  0x0F
#define IPV4_HLEN_SHIFT                 0
#define IPV4_FRAGO_MASK                 0x1FFF
#define IPV4_FLAGS_MF_MASK              0x2000
#define IPV4_FLAGS_DF_MASK              0x4000
#define IPV4_FRAG_DET_MASK              (IPV4_FLAGS_MF_MASK | \
                                         IPV4_FRAGO_MASK)

/* IPv6 definitions */
#define IPV6_VER_VALUE                  0x60
#define IPV6_FRAGO_MASK                 0xFFF8 
#define IPV6_FLAGS_MF_MASK              0x0001
#define FRAG_FIRST                      0x1
#define FRAG_MIDDLE                     0x2
#define FRAG_LAST                       0x4

#ifndef PA_FIELDOFFSET
    #define PA_FIELDOFFSET(type,member) ((size_t) &(((type*) 0)->member))
#endif

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as a UDP packet.
 */
#define IPPROTO_UDP             17

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as a IP-IP Tunnel packet.
 */
#define IPPROTO_IP              4

/**
 * @brief   This is the protocol identification field in the IPv6 header
 * which identifies the packet as an ESP packet.
 */
#define IPPROTO6_ESP            41

/**
 * @brief   This is the protocol identification field in the IPv6 header
 * which identifies the packet as a fragmentated packet.
 */
#define IPPROTO6_FRAG           44

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as an ESP packet.
 */
#define IPPROTO_ESP             50

/**
 * @brief   This is the protocol identification field in the IPv4 header
 * which identifies the packet as an AH packet.
 */
#define IPPROTO_AH              51


#define paEX_TRUE		((Bool) 1)
#define paEX_FALSE		((Bool) 0)

/** 
 * @brief 
 *  IPv4 Address
 *
 * @details
 *  IPv4 Address represented in network order.
 */
typedef struct paEx_IPN
{
    union
    {
        uint8_t    a8[4];
        uint32_t   a32;
    }u;
}paEx_IPN;

/** 
 * @brief 
 *  IPv6 Address
 *
 * @details
 *  IPv6 Address represented in network order.
 */
typedef struct paEx_IP6N
{
    union
    {
        uint8_t    a8[16];
        uint16_t   a16[8];
        uint32_t   a32[4];
    }u;
}paEx_IP6N;

/** 
 * @brief
 *  Enumeration for IP Version
 *
 * @details
 *  Enumeration which describes the IP version being used.
 */
typedef enum paEx_IPVersion
{
    /**
     * @brief   IPv4
     */
    paEx_IPVersion_IPV4    = 0x1,

    /**
     * @brief   IPv6
     */
    paEx_IPVersion_IPV6    = 0x2
}paEx_IPVersion;

/** 
 * @brief 
 *  IP Address Format used by the NETFP Library
 *
 * @details
 *  Generic data structure which encapsulates both IPv4
 *  and IPv6 addresses.
 */
typedef struct paEx_IPAddr
{
    paEx_IPVersion     ver;
    union
    {
        paEx_IPN   ipv4;
        paEx_IP6N  ipv6;
    }addr;
}paEx_IPAddr;

#define IPHDR_SIZE      20
typedef struct paEx_IPHeader
{
    uint8_t    VerLen;
    uint8_t    Tos;
    uint16_t   TotalLen;
    uint16_t   Id;
    uint16_t   FlagOff;
    uint8_t    Ttl;
    uint8_t    Protocol;
    uint16_t   Checksum;
    uint8_t    IPSrc[4];
    uint8_t    IPDst[4];
    uint8_t    Options[1];
}paEx_IPHeader;

#define IPv6HDR_SIZE      40
typedef struct paEx_IPv6Header
{
    uint8_t        VerTC;
    uint8_t        FlowLabel[3];
    uint16_t       PayloadLength;
    uint8_t        NextHeader;
    uint8_t        HopLimit;
    paEx_IP6N     SrcAddr;
    paEx_IP6N     DstAddr;
}paEx_IPv6Header;

#define IPV6_FRAGHDR_SIZE      8
typedef struct paEx_IPv6FragHeader
{
    uint8_t  NextHeader;
    uint8_t  Rsvd;
    uint16_t FragOffset;
    uint8_t  FragId[4];
}paEx_IPv6FragHeader;

/** 
 * @brief IP Reassembly configuration structure
 */
typedef struct paIPReassemblyConfig_s
{
    uint16_t     numReassemblyContexts;      /**< maxmium number of active reassembly contexts supported */
    uint16_t     descSize;                   /**< CPPI descriptor size */
    uint32_t     timeout;                    /**< Reassembly context timeout in milliseconds */
    
} paIPReassemblyConfig_t;


/** 
 * @brief IP Reassembly Statistics
 */
typedef struct paIPReassemblyStats_s
{
    uint32_t    fragmentsRxed;          /**< number of fragments received */
    uint32_t    reassembledPkts;        /**< number of reassembled packet generated */
    uint32_t    reassemblyTimeout;      /**< number of the timeout event occurs */
    uint32_t    fragmentsDecrypted;     /**< number of decrypted (inner IP) fragments received */
    uint32_t    decryptionFailures;     /**< number of decrypted fragments with errors */
    
} paIPReassemblyStats_t;


/**********************************************************************
 ************************** Exported Functions ************************
 **********************************************************************/
/* IP Reassembly Exported Functions. */
extern int paEx_reassemLibInit(paIPReassemblyConfig_t *pConfig);
extern int paEx_reassemLibProc(Cppi_HostDesc *pHostDesc, Qmss_QueueHnd destQ);
extern int paEx_reassemLibDelete(void);
extern void paEx_reassemLibTimerTick(uint32_t timeElapsed);
extern void paEx_reassemLibQueryStats(paIPReassemblyStats_t *pStats, int doClear);

#ifdef __cplusplus
}
#endif

#endif
