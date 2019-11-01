#ifndef _PALOC_H
#define _PALOC_H

/******************************************************************************
 * FILE PURPOSE: Local Defines for the PA LLD 
 ******************************************************************************
 * FILE NAME:   paloc.h
 *
 * DESCRIPTION: Internal module data structures and definitions  
 *
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2009-2013
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

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */
#include <string.h>
#include <stddef.h>
#include <ti/csl/cslr_pa_ss.h>
#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pa_fc.h>

/* 
 * Shut off: remark #880-D: parameter "descType" was never referenced
*
* This is better than removing the argument since removal would break
* backwards compatibility
*/
#ifdef _TMS320C6X
#pragma diag_suppress 880
#pragma diag_suppress 681
#elif defined(__GNUC__)
/* Same for GCC:
* warning: unused parameter descType [-Wunused-parameter]
*/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif


/* Macro definitions */
/* Main macros for accessing configuration bit fields       *
 * Input parameter a is tuint containing bit field.         *
 * b is bit offset withing bit field. c is number of bits   *
 * used by that parameter. x is new value of parameter that *
 * is packed in this bit field.                             */
#define UTL_GET_BITFIELD(a,b,c)    (((a)>>(b)) & ((1U<<(c))-1))

/* following one enforces strict setting to prevent overflow into other bits, would
   cost program space for additional protection */
#define UTL_SET_BITFIELD(a,x,b,c)  (a) &= ~(((1U<<(c))-1)<<(b)), \
                                   (a) |= (((x) & ((1U<<(c))-1))<<(b))

#define pa_CONV_BASE_TO_OFFSET(poolbase, base)  ((uint32_t)base-(uint32_t)poolbase)
#define pa_CONV_OFFSET_TO_BASE(poolbase, offset)  ((uint32_t)poolbase + (uint32_t) offset)

/* PA memory buffer requirements for an instance of PA */
#define PA_BUFFNUM_INSTANCE                 pa_BUF_INST
#define PA_BUFFNUM_L2_TABLE                 pa_BUF_L2_TABLE
#define PA_BUFFNUM_L3_TABLE                 pa_BUF_L3_TABLE
#define PA_BUFFNUM_USR_STATS_LNK_TABLE      pa_BUF_USR_STATS_TABLE
#define PA_BUFFNUM_VIRTUAL_LNK_TABLE        pa_BUF_VLINK_TABLE
#define PA_BUFFNUM_ACL_TABLE                pa_BUF_ACL_TABLE
#define PA_BUFFNUM_FC_TABLE                 pa_BUF_FC_TABLE
#define PA_BUFFNUM_EOAM_TABLE               pa_BUF_EOAM_TABLE 

/* PA EOAM Multiply and shift values considered for 1588 time convertion */
#define pa_1588_TIME_CONV_MUL_FOR_INPUT_FREQ_1000MHZ    49152
#define pa_1588_TIME_CONV_MUL_FOR_INPUT_FREQ_1050MHZ    46811
#define pa_1588_TIME_CONV_MUL_FOR_INPUT_FREQ_1049p6MHZ  46829
#define pa_1588_TIME_CONV_SHIFT                         13

/* 
 * Table Entry status
 */
enum  {

  PA_TBL_STAT_INACTIVE,
  PA_TBL_STAT_PENDING_SUBS_REPLY,
  PA_TBL_STAT_ACTIVE
};

/*
 * L2 input MAC configuration 
 *    Broken out from paL2Entry for easy comparison
 */
typedef struct paL2MacCfg_s  {

  paMacAddr_t  dstMac;   /* Destination Mac Address */
  paMacAddr_t  srcMac;   /* Source Mac Address */
  
  uint16_t  vlan;        /* VLAN */
  uint16_t  ethertype;   /* Ethernet Type */
  
  uint32_t  mplsTag;     /* MPLS tag */
  uint8_t   inport;      /* Input EMAC port */
  uint16_t  vlanPri;     /* VLAN priority */
  
}  paL2MacCfg_t;

/*
 * L2 input SRIO configuration 
 *    Broken out from paL2Entry for easy comparison
 */

typedef struct paL2SrioCfg_s  {

  uint16_t    validBitMap; /* valid parameters bit map: refer tp pa.h */ 
  uint16_t    srcId;       /* Source Device ID */
  uint16_t    destId;      /* Destination device ID */
  uint8_t     tt;          /* Transport type: 8-bit or 16-bit device */
  uint8_t     cc;          /* Completion code */
  uint8_t     pri;         /*3-bit priority */
  uint8_t     msgType;     /* Message Type: Type 9 or 11 */
  uint16_t    typeParam1;  /* Type9: streamId; Type11: mbox */
  uint8_t     typeParam2;  /* Type9: cos; Type11: letter */
  
}  paL2SrioCfg_t;

/*
 * L2 input configuration 
 */
typedef union paL2InCfg_s  {
    paL2MacCfg_t    mac;    /* MAC specific parameters */
    paL2SrioCfg_t   srio;   /* SRIO specific parameters */
}  paL2InCfg_t;

/*
 * Common L2/L3/VL/ACL Entry header
 */
typedef struct paL2L3Header_t  {

  int8_t    type;           /* L2, L3 or ACL entry        */
  int8_t    status;         /* PA_TBL_STAT_...            */
  uint16_t  tableIdx;       /* Entry number in this table */
  
  int8_t    subType;        /* MAC, SRIO, IP , Custom     */
  int8_t    pdspNum;        /* PDSP identifier            */
  int16_t   lutIdx;         /* PDSP LUT entry             */
  
  uint16_t  virLink;        /* TBD: virtual link number   */
  int16_t   lnkCnt;         /* Number of channels linking to this entry */
  
} paL2L3Header_t;

/*
 *  An L2 table element
 */
typedef struct paL2Entry_s  {

  paL2L3Header_t hdr;  /* L2 header, Must be first entry */
  paL2InCfg_t  cfg;    /* L2 configuration parameters as defined above */

} paL2Entry_t;


/*
 * L3 custom entry
 */
typedef struct paL3CustomCfg_s
{
    uint8_t customIdx;                        /* Index of Custom LUT1 type */
    uint8_t match[pa_NUM_BYTES_CUSTOM_LUT1];  /* Custom LUT1 byte masks */
} paL3CustomCfg_t;
  
/*
 * An L3 table element
 */
typedef struct paL3Entry_s  {

  paL2L3Header_t  hdr;         /* L3 header, Must be first entry */
  paLnkHandle_t  pHandle;     /* Previous link handle, which could be hard link or virtual link, NULL for no link */
  paLnkHandle_t  nHandle;     /* Next link handle, can only be NULL or Virtual Link*/
  
  union  {
    paIpInfo2_t      ipInfo;      /* IP specific parameters */
    paL3CustomCfg_t customInfo;  /* Custom LUT1 specific parameters */
  } u;
    
} paL3Entry_t;

/*
 * A Virtual Link table element (expansion of L3 table) 
 */
typedef struct paVlinkEntry_s  {

  paL2L3Header_t  hdr;         /* Common Header, Must be first entry */
    
} paVlinkEntry_t;

typedef paVlinkEntry_t  paVirtualLnk_t;

/*
 * An EOAM table element (same as L2 Table)
 */
typedef paL2Entry_t paEoamEntry_t;

/*
 * An ACL table element (expansion of L3 table) 
 */
typedef struct paAclEntry_s  {

  paL2L3Header_t  hdr;         /* ACL header, Must be first entry */
  paHandleL2L3_t  pHandle;     /* Previous L2/L3 handle, NULL for no link */
  paAclInfo_t     aclInfo;     /* ACL specific parameters */
  
} paAclEntry_t;

#define PA_ACL_LINK_ELEM_ACTIVE_ENTRY           (1 << 0)
#define PA_ACL_LINK_ELEM_RESCORE_COMPLETE       (1 << 1)

/*
 * An ACL link list element (used for re-score operations)
 */
typedef struct paAclLinkListElem_s {

  uint16_t            ctrlBitMap;    /* Control flags as defined below
                                      *  B0: active entry
                                      *  B1: rescoring schedule is completed 
                                      */
  uint16_t            elemIdx;       /* ACL Element Index */
  paHandleAcl_t       aclHandle;     /* ACL handle */
  uint16_t            score;         /* ACL entry score */
  uint16_t            scorePending;  /* ACL entry score after rescoring is completed */
  struct paAclLinkListElem_s *prev;  /* Point to the previous element in the list */  
  struct paAclLinkListElem_s *next;  /* Point to the next element in the list */ 
}paAclLinkListElem_t; 

#define PA_ACL_LINK_INFO_RESCORING_ACTIVE       (1 << 0)

/*
 * An ACL Link link list information 
 */
typedef struct paAclLinkListInfo_s  {

  uint8_t             ctrlBitMap;    /* Control flags as defined below
                                      * B0: Rescoring is active
                                      */
  uint8_t             lastIdx;       /* Latest ACL entry table index, Pending LUT1 index */     
  uint16_t            numEntries;    /* Number of active ACL entries */
  
  paAclLinkListElem_t *head;  /* Point to the first element in the list */
  paAclLinkListElem_t *tail;  /* Point to the last element in the list */
} paAclLinkListInfo_t;

/* ACL Rescore header */
typedef struct paAclRescoreHdr_s {
  uint32_t            final_cur;        /* final Offset of the rescore operations (hi 16 bits), current offset of the rescore operation (lo 16 bit) */
} paAclRescoreHdr_t;

/* ACL rescore data list */
typedef struct paAclRescoreList_s  {
  uint32_t  idx_score[pa_MAX_NUM_LUT1_ENTRIES];  /* Array of rescoring elements */
} paAclRescoreList_t;

/* Linked List Info OFFSET from the aclTable */
#define  PA_ACL_LINK_LIST_INFO_TABLE(isOuter, n, base)    ( (uint32_t) base + (uint32_t)(isOuter * sizeof(paAclLinkListInfo_t)) + (uint32_t) (n * sizeof(paAclEntry_t)) )

/* Linked List Element */
#define  PA_ACL_LINK_LIST_ELEM_TABLE(n, base)             (  PA_ACL_LINK_LIST_INFO_TABLE(1, n, base) + sizeof(paAclLinkListInfo_t) )

/* ACL Score values */
#define PA_ACL_HIGH_SCORE_LIST     0xFF00
#define PA_ACL_MID_SCORE_LIST      0x8000
#define PA_ACL_LOW_SCORE_LIST      0x0100

/* ACL Rescore Trigger */
#define PA_ACL_RESCORE_NO_TRIGGER  0
#define PA_ACL_RESCORE_TRIGGER     1

/* Minimum Rescore Distance */
#define PA_ACL_MIN_RESCORE_DISTANCE         2
#define PA_ACL_IDEAL_SCORE_DISTANCE     0x100

/* communication to firmware bit map */
#define PA_ACL_FW_RESCORE_LIST_GENERATED    (1 << 15)
#define PA_ACL_FW_RESCORE_PEND_LUT1_IDX     (1 << 14)

/*
 * An FC table element (expansion of L3 table) 
 */
typedef struct paFcEntry_s  {

  paL2L3Header_t  hdr;         /* FC header, Must be first entry */
  paFcInfo_t      fcInfo;      /* FC specific parameters */
  
} paFcEntry_t;

/* PA Entry Types */
#define  PA_TABLE_ENTRY_TYPE_L2    2
#define  PA_TABLE_ENTRY_TYPE_L3    3 
#define  PA_TABLE_ENTRY_TYPE_L4    4
#define  PA_TABLE_ENTRY_TYPE_VL    5
#define  PA_TABLE_ENTRY_TYPE_ACL   10
#define  PA_TABLE_ENTRY_TYPE_FC    11
#define  PA_TABLE_ENTRY_TYPE_EOAM  12
#define  PA_TABLE_ENTRY_TYPE_NONE  0xFF /* The L4 entry does not connect to a upper layer (no longer used) */

/* PA Enter sub-types */
#define  PA_TABLE_ENTRY_SUBTYPE_MAC         0
#define  PA_TABLE_ENTRY_SUBYTPE_SRIO        1
#define  PA_TABLE_ENTRY_SUBYTPE_IP          2 
#define  PA_TABLE_ENTRY_SUBYTPE_IPSEC       3
#define  PA_TABLE_ENTRY_SUBYTPE_CUSTOM      4
#define  PA_TABLE_ENTRY_SUBYTPE_PORT16      5
#define  PA_TABLE_ENTRY_SUBYTPE_PORT32      6
#define  PA_TABLE_ENTRY_SUBYTPE_L3L4        7
#define  PA_TABLE_ENTRY_SUBYTPE_IP_IPSEC    10     /* Internal IP entry which linking to external IP/IPSEC entries */ 
#define  PA_TABLE_ENTRY_SUBTYPE_VLINK_BASE      20
#define  PA_TABLE_ENTRY_SUBTYPE_VLINK_MAC       PA_TABLE_ENTRY_SUBTYPE_VLINK_BASE + pa_VIRTUAL_LNK_TYPE_MAC
#define  PA_TABLE_ENTRY_SUBTYPE_VLINK_OUTER_IP  PA_TABLE_ENTRY_SUBTYPE_VLINK_BASE + pa_VIRTUAL_LNK_TYPE_OUTER_IP
#define  PA_TABLE_ENTRY_SUBTYPE_VLINK_INNER_IP  PA_TABLE_ENTRY_SUBTYPE_VLINK_BASE + pa_VIRTUAL_LNK_TYPE_INNER_IP

typedef struct  paL4Entry_s {

  uint8_t  lnkType;      /* Identifies the handle type which linking to the L4 entry */
                         /* PA_TABLE_ENTRY_TYPE_NONE if there is no link */
  uint8_t  customIndex;  /* custom Index, valid if subType =  PA_TABLE_ENTRY_SUBYTPE_CUSTOM */
  uint8_t  subType;      /* PA entry sub-type: PA_TABLE_ENTRY_SUBYTPE_CUSTOM, PA_TABLE_ENTRY_SUBYTPE_PORT16 or
                                               PA_TABLE_ENTRY_SUBYTPE_PORT32 */
  uint8_t  rsvd1;        /* alignment */                                             
  uint16_t lnkTableIdx;  /* Entry number in this table of the upper link*/
  uint16_t lnk;          /* 16-bit Link to previous entry */
  
  union  {
    uint16_t   port16;         /* 16-bit port with link */
    uint32_t   port32;         /* 32-bit port */
    uint8_t    customInfo[4];  /* custom port */
  } u;
  
} paL4Entry_t;

#define PA_L4_CUSTOM_NO_LINK  0xff

typedef struct  paUsrStatsLnkEntry_s
{
    uint16_t    lnkIndex;    /* next layer counter index */
} paUsrStatsLnkEntry_t;

#define PA_USR_STATS_LNK_END     0x8000

typedef struct paMemBuf_s {
    int     size;
    void*   base;
} paMemBuf_t;

typedef struct {
  paProtocolLimit_t         protoLimit;           /* the protocol limit configuration structure */
  paIpReassmConfig_t        outIpReassmConfig;    /* the outer IP Reassembly  configuration structure */
  paIpReassmConfig_t        inIpReassmConfig;     /* the inner IP Reassembly  configuration structure */
  paCmdSetConfig_t          cmdSetConfig;         /* the command set configuration structure */
  paUsrStatsConfig_t        usrStatsConfig;       /* the user-defined statistics configuration structure */
  paQueueDivertConfig_t     queueDivertConfig;    /* the queue-diversion configuration structure */
  paAclConfig_t             outAclConfig;         /* the outer ACL configuration structure */
  paAclConfig_t             inAclConfig;          /* the inner ACL configuration structure */
  paRaGroupConfig_t         outIpRaGroupConfig;   /* the outer IP Reassembly group configuration structures */
  paRaGroupConfig_t         inIpRaGroupConfig;    /* the inner IP Reassembly group configuration structures */
  paEoamGlobalConfig_t      eOamConfig;           /* the ethernet OAM configuration structure */
  paQueueBounceConfig_t     queueBounceConfig;    /* the queue-bounce configuration structure */
} paSysInfo_t;

/*
 * PASS command ID formatting
 *  Bit 12-15 is used to identify the type of table in the command comId field
 */
#define PA_COMID_L2        (0 << 12)
#define PA_COMID_L3        (1 << 12)
#define PA_COMID_ACL       (2 << 12)
#define PA_COMID_FC        (3 << 12)
#define PA_COMID_EOAM      (4 << 12)

#define PA_COMID_L_MASK    (15 << 12)
#define PA_COMID_IDX_MASK  (~(15 << 12))

/*
 * PDSPs and PDSP Layout
 * (TBD: Move to pa.h)
 */
enum  {

  PASS_PDSP0 = 0,
  PASS_PDSP1,
  PASS_PDSP2,
  PASS_PDSP3,
  PASS_PDSP4,
  PASS_PDSP5,
  PASS_PDSP6,
  PASS_PDSP7,
  PASS_PDSP8,
  PASS_PDSP9,
  PASS_PDSP10,
  PASS_PDSP11,
  PASS_PDSP12,
  PASS_PDSP13,
  PASS_PDSP14,
  PASS_NUM_PDSPS
};

#define PASS_INGRESS0_PDSP0      PASS_PDSP0
#define PASS_INGRESS0_PDSP1      PASS_PDSP1
#define PASS_INGRESS1_PDSP0      PASS_PDSP2
#define PASS_INGRESS1_PDSP1      PASS_PDSP3
#define PASS_INGRESS2_PDSP0      PASS_PDSP4
#define PASS_INGRESS3_PDSP0      PASS_PDSP5
#define PASS_INGRESS4_PDSP0      PASS_PDSP6
#define PASS_INGRESS4_PDSP1      PASS_PDSP7
#define PASS_POST_PDSP0          PASS_PDSP8
#define PASS_POST_PDSP1          PASS_PDSP9
#define PASS_EGRESS0_PDSP0       PASS_PDSP10
#define PASS_EGRESS0_PDSP1       PASS_PDSP11
#define PASS_EGRESS0_PDSP2       PASS_PDSP12
#define PASS_EGRESS1_PDSP0       PASS_PDSP13
#define PASS_EGRESS2_PDSP0       PASS_PDSP14


enum  {

  PASS_CLUSTER0 = 0,
  PASS_CLUSTER1,
  PASS_CLUSTER2,
  PASS_CLUSTER3,
  PASS_CLUSTER4,
  PASS_CLUSTER5,
  PASS_CLUSTER6,
  PASS_CLUSTER7,
  PASS_CLUSTER8,
  PASS_NUM_CLUSTERS
};

#define PASS_CLUSTER_INGRESS0     PASS_CLUSTER0       
#define PASS_CLUSTER_INGRESS1     PASS_CLUSTER1       
#define PASS_CLUSTER_INGRESS2     PASS_CLUSTER2       
#define PASS_CLUSTER_INGRESS3     PASS_CLUSTER3       
#define PASS_CLUSTER_INGRESS4     PASS_CLUSTER4       
#define PASS_CLUSTER_POST         PASS_CLUSTER5       
#define PASS_CLUSTER_EGRESS0      PASS_CLUSTER6       
#define PASS_CLUSTER_EGRESS1      PASS_CLUSTER7       
#define PASS_CLUSTER_EGRESS2      PASS_CLUSTER8   

#define PASS_CLUSTER_EF_REC1      PASS_CLUSTER_EGRESS0
#define PASS_CLUSTER_EF_REC2      PASS_CLUSTER_EGRESS0
#define PASS_CLUSTER_EF_REC3      PASS_CLUSTER_EGRESS1
#define PASS_CLUSTER_EF_REC4      PASS_CLUSTER_EGRESS2

/* MACRO to calculate the address offset of PASS clusters and PPU (packet processing unit) respectively */
#define PASS_CLUSTER_OFFSET(clNum)          (0x400000U + (clNum)*0x100000U)  
#define PASS_PPU_OFFSET(clNum, pdspNum)     (PASS_CLUSTER_OFFSET((clNum)) + (pdspNum)*0x10000U + 0x8000U)
  
/*
 * One instance of the driver
 */
#define PA_USR_STATS_BITMAP_SIZE    ((pa_USR_STATS_MAX_COUNTERS + 31) >> 5)
 
typedef struct paInst_s  {

  paMemBuf_t paBufs[pa_N_BUFS];
  int  nL2;             /* Number of entries in L2 table */
  int  nL3;             /* Number of entries in L3 table */
  int  nUsrStats;       /* Number of user-defined statistics in the user-statistics linking table */
  int  nAcl;            /* Number of entries in ACL table */
  int  nFc;             /* Number of entries in FC table */
  int  nEoam;           /* Number of entries in EOAM table */
  int16_t  nMaxVlnk;    /* Number of virtual links in the VLNK table */
  int16_t  n2152Entries;/* Number of active GTP-U (UDP port = 2158) ports */
                        /* 0 to 1 transition: Disable GTP-U processing */
                        /* 1 to 0 transition: Enable GTP-U processing */
  
  uint32_t stateBitfield;  /* TRUE: Instance State */
/* Bit 0: PASS Ready
   Bit 1: GTPU Classification with link
   Bit 2: EOAM Classification 
   Bit 3-31: Reserved
*/
  paSysInfo_t cfg;      /* record the global system configurations */
  uint32_t    usrStatsAllocBitmap[PA_USR_STATS_BITMAP_SIZE];/* User-defined statistics allocation bitmap */
  
} paInst_t;

#define PA_SET_STATE_READY(a,b)        UTL_SET_BITFIELD((a)->stateBitfield, b, 0, 1)
#define PA_TEST_STATE_READY(a)         UTL_GET_BITFIELD((a)->stateBitfield, 0, 1)
#define PA_SET_STATE_GTPU_LINK(a,b)    UTL_SET_BITFIELD((a)->stateBitfield, b, 1, 1)
#define PA_TEST_STATE_GTPU_LINK(a)     UTL_GET_BITFIELD((a)->stateBitfield, 1, 1)
#define PA_SET_STATE_EOAM(a,b)         UTL_SET_BITFIELD((a)->stateBitfield, b, 2, 1)
#define PA_GET_STATE_EOAM(a)           UTL_GET_BITFIELD((a)->stateBitfield, 2, 1)

/*
 * PDSP mapping structure
 */
typedef struct paPdspMap_s {
    uint8_t clNum;      /* Cluster number */
    uint8_t pdspNum;    /* PDSP number */
    uint16_t verBase;   /* version Base address */ 
} paPdspMap_t;

/* 
 * PASS Statistics allocation:
 * Define PASS-related Statistics map of the 4K 32-bit counters available at the statistics module
 * 
 */
#define PASS_SYS_STATS_BASE     0
#define PASS_SYS_STATS_SIZE     128 

#define PASS_RA_STATS_BASE      128
#define PASS_RA_STATS_SIZE      32
#define PASS_RA_GROUP_STATS_BASE(group) (PASS_RA_STATS_BASE+(group)*PASS_RA_STATS_SIZE)  

#define CSL_PA_RA_STATS_ENABLE_MASK    CSL_PA_RA_TOTAL_FRAGS_ENABLE_MASK  

#define PASS_FC_STATS_BASE     0x300
#define PASS_FC_STATS_SIZE     256
#define PASS_FC_STATS_BASE_ENTRY(index) (PASS_FC_STATS_BASE+(index))

#define PASS_ACL_STATS_BASE     0x400
#define PASS_ACL_STATS_SIZE     1024
#define PASS_ACL_STATS_BASE_ENTRY(index) (PASS_ACL_STATS_BASE+(index)*2)

#define PASS_EOAM_STATS_BASE     0x400
#define PASS_EOAM_STATS_SIZE     1024
#define PASS_EOAM_STATS_BASE_ENTRY(index) (PASS_EOAM_STATS_BASE+(index)*2)

/*
 *  MPLS labels are restriced to 20 bits
 */
#define PA_MPLS_LABEL_MASK  0x000fffff

/*
 * Flow labels are restricted to 20 bits
 */
#define PA_IP_FLOW_MASK 0x000fffff

/*
 * Maximum loop count for restart ack from PDSP.
 */
#define PA_MAX_PDSP_ENABLE_LOOP_COUNT   50000

/*
 * Define TRUE and FALSE
 */
#undef TRUE
#undef FALSE
#define TRUE     (uint16_t)1
#define FALSE    (uint16_t)0

/* PA Local Object */
typedef struct
{
    /* PA start config */
    paStartCfg_t cfg;
    CSL_Pa_ssRegs *pSysRegs;  /* Pointer to system-level register */
    CSL_Pa_clRegs *pClRegs[PASS_NUM_CLUSTERS]; /* Pointers to cluster-level registers */
    CSL_Pa_ppuRegs *pPpuRegs[PASS_NUM_PDSPS];  /* Pointers to ppu-level registers where PPU stands for packet processing unit including a PDSP */
}Pa_LocalObj;

extern Pa_LocalObj    paLObj;

/*
 * Define the IP reassembly traffic flow structure
 */

/* IP Reassem TF Structure */
typedef struct
{
    uint32_t word0;
    uint32_t word1;
    uint32_t word2;
    uint32_t word3;
}Pa_IpTrafficFlow_t;

extern const paPdspMap_t paPdspMap[PASS_NUM_PDSPS];

/* local global functions */
void pa_ra_global_cfg (paRaConfig_t *pRaCfg, CSL_Pa_ssRegs *pSysRegs);
void pa_ra_group_cfg (paRaGroupConfig_t *pRaGroupCfg, CSL_Pa_raFlow_overrideRegs *pRaFlowRegs);

/******************************************************************************
 * FUNCTION PURPOSE: Write 16 bit value into Big-Endian Byte Array (macro version)
 ******************************************************************************
 * DESCRIPTION: Writes 16 bit value into Big-Endian byte array. No alignment assumed 
 * 
 * void pktWrite16bits_m (
 *    uint8_t *base,       - Base of byte array
 *    int      byteOffset, - byte offset to write; 
 *    uint16_t val)        - 16 bit val
 * 
 *****************************************************************************/
static inline void pktWrite16bits_m (uint8_t *base, int byteOffset, uint16_t val) 
{
  uint8_t *wptr = (base + byteOffset);

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  wptr[0] = (uint8_t)(val>>8);
  wptr[1] = (uint8_t)(val & 0xff);

} /* pktWrite16bits_m */

/******************************************************************************
 * FUNCTION PURPOSE: Read 16 bit value from Big-Endian Byte Array (macro version)
 ******************************************************************************
 * DESCRIPTION: Returns 16 bit value from Big-Endian byte array.  
 * 
 * tuint pktRead16bits_m (
 *    uint8_t *base,       - Base of byte array
 *    int     byteOffset); - Byte offset to read
 * 
 *****************************************************************************/
static inline uint16_t pktRead16bits_m (uint8_t *base, int byteOffset) 
{
  uint8_t *wptr = (base + byteOffset);
  uint16_t ret;

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  ret = (((uint16_t)wptr[0]) << 8) | (wptr[1] & 0xFF);

  return ret;
} /* pktRead16bits_m */

/******************************************************************************
 * FUNCTION PURPOSE: Write 32 bit value into Big-Endian Byte Array (macro version)
 ******************************************************************************
 * DESCRIPTION: Writes 32 bit value into Big-Endian byte array; No alignment assumed
 * 
 * void pktWrite32bits_m (
 *    uint8_t *base,       - Base of byte array
 *    int      byteOffset, - byte offset to write; assumed to be even.
 *    uint32_t val)        - 32 bit val
 * 
 *****************************************************************************/
static inline void pktWrite32bits_m (uint8_t *base, int byteOffset, uint32_t val) 
{
  /* Shift/mask is endian-portable, but look out for stupid compilers */
  pktWrite16bits_m (base, byteOffset, (uint16_t)(val>>16));
  pktWrite16bits_m (base, byteOffset+2, (uint16_t)(val&0xffff));

} /* pktWrite32bits_m */

/******************************************************************************
 * FUNCTION PURPOSE: Read 32 bit value from Big-Endian Byte Array (macro version)
 ******************************************************************************
 * DESCRIPTION: Read 32 bit value from Big-Endian byte array; No alignment assumed
 * 
 * uint32_t pktRead32bits_m (
 *    uint8_t *base,      - Base of byte array
 *    int     byteOffset) - byte offset to write; assumed to be even.
 * 
 *****************************************************************************/
static inline uint32_t pktRead32bits_m (uint8_t *base, int byteOffset) 
{
  uint32_t ret;

  /* Shift/mask is endian-portable, but look out for stupid compilers */
  ret = (((uint32_t)pktRead16bits_m (base, byteOffset)) << 16);
  ret |= (uint32_t)pktRead16bits_m (base, byteOffset + 2);

  return ret;
} /* pktRead32bits_m */

/* Data format and extraction Macros */

#define PALLD_MK_UINT16(high8,low8)                               \
    ((uint16_t)( ((uint16_t)(high8) << 8) | (uint16_t)(low8) ))

#define PALLD_UINT16_LOW8(a)	                                    \
    ((uint8_t)((a) & 0x00FF))

#define PALLD_UINT16_HIGH8(a)	                                    \
    ((uint8_t)(((a) >> 8) & 0x00FF))

#define PALLD_MK_UINT32(high16,low16)                             \
    ((uint32_t)( ((uint32_t)(high16) << 16) | (uint32_t)(low16) ))

#define PALLD_MK_UINT32_FROM8S(high8,med_high8,med_low8,low8)     \
    PALLD_MK_UINT32(PALLD_MK_UINT16(high8,med_high8), PALLD_MK_UINT16(med_low8, low8))

#define PALLD_MK_UINT32_FROM_8ARRAY(ptr) \
	((uint32_t)(ptr[0]<<24) | (ptr[1]<<16) | (ptr[2]<<8) | (ptr[3]))
	
#define PALLD_UINT32_LOW16(u32)                                   \
    ((uint16_t)((u32) & 0xFFFF))

#define PALLD_UINT32_HIGH16(u32)                                  \
    ((uint16_t)(((u32) >> 16) & 0xFFFF))

#define PALLD_UINT32_LOW8(u32)                                    \
    ((uint8_t)((u32) & 0x00FF))

#define PALLD_UINT32_MED_LOW8(u32)                                \
    ((uint8_t)(((u32) >> 8) & 0xFF))

#define PALLD_UINT32_MED_HIGH8(u32)                               \
    ((uint8_t)(((u32) >> 16) & 0xFF))

#define PALLD_UINT32_HIGH8(u32)                                   \
    ((uint8_t)(((u32) >> 24) & 0xFF))






#ifdef __cplusplus
}
#endif

#endif  /* _PALOC_H */
