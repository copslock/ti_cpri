/*
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#include "../pautest.h"

#ifdef __LINUX_USER_SPACE
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif

/* Ethernet OAM Packet Forwading test
 * This test tests the LLD and firmware the ability to perform Forwarding of the Ethernet OAM control packets to
 * host operation  and also regular packet forward operations when Ethernet OAM feature is enabled.
 *
 * Add/Delete EOAM flow test
 * This test test the LLD Pa_addEoamFlow, Pa_delHandle as well as the
 * PDSP firmware for classification of EOAM packets and respective routing. This test has the following sub-tests
 *  - Test the LLD and firmware for the ability to insert EOAM entries
 *  - Test the LLD to detect duplicate entry
 *  - Test the LLD when entering more entries than configured for
 *  - Test for global configuration
 *  - Test the firmware to test take a burst of Pa_AddEoamFlow commands
 *  - Test the firmware for the ability to detect EOAM header packets and routing
*/
#include "reassemLib.h"

static char *tfName = "paTestEOAMFlow";

#ifdef NSS_GEN2
#define PA_USE_HW_RA               /* Use Hradware Reassembly Engine */
#endif

#ifdef PA_USE_HW_RA
static paRaStats_t paTestRaStats;
#endif


#define T14_NUM_PACKET_ITERATIONS    1          /* Number of times the packet stream is passed through */
#define T14_NUM_PACKET_GROUPS     10      /* Number of packet groups where each group contains packets with specified payload size */

#define T14_EXPTPKT_FIFO_SIZE       20          /* Must hold the number of packets in transit in PA at one time */

/* General purpose queue usage */
#define Q_CMD_RECYCLE          0        /* Command descriptors/buffers recycled here after sent to PA */
#define Q_CMD_REPLY            1        /* Replies from PA routed here */
#define Q_MATCH_EOAM_PKT     2    /* Packets which matched EOAM target flow classification and belong to 1DM/LMM/LMR/DMM/DMR types */
#define Q_ESP_FIX            3    /* Cipher Packets */
#define Q_MATCH                4        /* Packets from PA which match a lookup criteria */
#define Q_NFAIL                5        /* Packets from PA which matches a mac lookup, but failed an L3 lookup */
#define Q_PARSE_ERR            6        /* Packets which resulted in a parse error */
#define Q_PATCH_COUNT        7    /* EOAM insert Count queue */
#define Q_PATCH_TIME         8    /* EOAM insert time queue */
#define Q_DPKT_RECYCLE       9    /* Data packet recycle queue */
#define Q_IP_REASSM1         10   /* IP Reassembly Input Queue 1 (Outer IP) */
#define Q_IP_REASSM2         11   /* IP Reassembly Input Queue 2 (Inner IP) */
#define Q_IP_FRAG            12   /* IP Fragmentation */
#define Q_L2_CAPTURE         13   /* L2 Capture queue */


/* Global Configration IDs */
#define T14_EOAM_TIME_OFFSET_CONFIG               0
#define T14_GLOBAL_SYS_CONFIG_EOAM_ENABLE         1
#define T14_GLOBAL_SYS_CONFIG_EOAM_DISABLE        2
#define T14_GLOBAL_SYS_CONFIG_USR_STATS           3
#define T14_GLOBAL_SYS_CONFIG_PKTCTRL             4


/* LLDP Packet index */
#define  T14_LLDP_PKT_INDEX                          0x2200

/* The number of PA L2 and L3 handles maintained by this test */
#define T14_NUM_L2_ENTRIES                  (sizeof(t14EthSetup)/sizeof(t14EthSetup_t))
#define T14_L2_CAP_HANDLE_INDEX             6
#define T14_NUM_L3_ENTRIES                  (sizeof(t14IpSetup)/sizeof(t14IpSetup_t))
#define T14_NUM_L4_ENTRIES                  (sizeof(t14UdpSetup)/sizeof(t14UdpSetup_t))
#define T14_NUM_EOAM_ENTRIES                (sizeof(t14EoamInfo)/sizeof(t14EoamPaSetup_t))
#define T14_NUM_ACL_ENTRIES                 (sizeof(t14AclSetup)/sizeof(t14AclPaSetup_t))
#define T14_NUM_EGRESS0_CMD_ENTRIES         (sizeof (t14EoamEgressPktTestInfo)/ sizeof (pktTestInfo3_t))

#define T14_NUM_LOCAL_L2_HANDLES            T14_NUM_L2_ENTRIES
#define T14_NUM_LOCAL_L3_HANDLES              T14_NUM_L3_ENTRIES
#define T14_NUM_LOCAL_L4_HANDLES            T14_NUM_L4_ENTRIES
#define T14_NUM_LOCAL_EOAM_HANDLES      T14_NUM_EOAM_ENTRIES
#define T14_NUM_LOCAL_ACL_HANDLES       T14_NUM_ACL_ENTRIES
#define T14_NUM_GEN_CONFIGS             10     /* Maxmium number of general configuration commands */

#include "test14pkts.h"

 /* L3 handles are managed. This structure is used to track the handle and
  * the activation state state of the handle */
enum  {
    T14_HANDLE_UNCONFIGURED = 0,
    T14_HANDLE_PENDING_ACK,
    T14_HANDLE_ACTIVE,
    T14_HANDLE_DISABLED
};

typedef struct t14Handles_s  {

    paHandleL2L3_t  paHandle;     /* The L2/L3 handle returned by the PA LLD */

    unsigned int    state;        /* T14_HANDLE_UNCONFIGURED = handle not configured
                                   * T14_HANDLE_PENDING_ACK = handle configured and sent to pa
                                   * T14_HANDLE_ACTIVE = handle creation acknowledged by pa
                                   * T14_HANDLE_DISABLED = handle was created then released */

  unsigned int  linkCnt;
} t14Handles_t;

typedef struct t14HandlesL4_s  {
  paHandleL4_t   paHandle;
  unsigned int   state;
}t14HandlesL4_t;

typedef struct t14HandlesEoam_s  {
  paHandleEoam_t   paHandle;     /* The EOAM handle returned by the PA LLD */
  unsigned int  state;
 }t14HandlesEoam_t;

typedef struct t14HandlesAcl_s  {
  paHandleAcl_t   paHandle;     /* The ACL handle returned by the PA LLD */
  unsigned int  state;
}t14HandlesAcl_t;


/* A grouping of run time created grouped together to make cleanup easier on
 * error exit */
typedef struct t14TestEncap_s  {
  tFramework_t  *tf;
  paTest_t      *pat;
  /* There is one base packet for each L3 table entry */
  Cppi_HostDesc  *hd[T14_NUM_LOCAL_L3_HANDLES];
  /* The command to the modify PDSP to add the Insert Time/Count and route to PA receive */
  uint32_t        cmdStack[T14_NUM_EGRESS0_CMD_ENTRIES][2 * (sizeof(pasahoNextRoute_t)   + \
                                                             sizeof(pasahoInsMsgCount_t) + \
                                                             sizeof(pasahoInsMsgTime_t)) / sizeof (uint32_t)];
  /* The command to the modify PDSP to add the TCP/UDP checksum and route to PA receive */
  uint32_t        cmdStackReasm[T14_NUM_LOCAL_L4_HANDLES][(2 * sizeof(pasahoNextRoute_t) +     \
                                                                 sizeof(pasahoComChkCrc_t) +     \
                                                                 sizeof(pasahoIpFrag_t)    +     \
                                                                 sizeof(pasahoComBlindPatch_t) + \
                                                                 (2 * sizeof(pasahoPatchMsgLen_t))) / sizeof (uint32_t)];
    t14Handles_t    l2Handles[T14_NUM_LOCAL_L2_HANDLES+1];      /* MAC handles */
    t14Handles_t    l3Handles[T14_NUM_LOCAL_L3_HANDLES+1];      /* IP handles  */
    t14HandlesL4_t  l4Handles[T14_NUM_LOCAL_L4_HANDLES+1];      /* UDP/TCP handles */
  t14HandlesEoam_t eoamHandles[T14_NUM_LOCAL_EOAM_HANDLES+1];   /* EOAM handles */
  t14HandlesAcl_t  aclHandles[T14_NUM_LOCAL_ACL_HANDLES+1]; /* ACL handles */
  unsigned int     genCmdAck[T14_NUM_GEN_CONFIGS];             /* General configurations */
 } t14TestEncap_t;

static paSysStats_t paTestL2ExpectedStats;    /* Expected stats results */
static paUsrStats_t paTestExpectedUsrStats;

static paCmdReply_t cmdReply = {  pa_DEST_HOST,             /* Dest */
                                  0,                        /* Reply ID (returned in swinfo0) */
                                  0,                        /* Queue */
                                  0 };                      /* Flow ID */


static paRouteInfo2_t nfailRoute_1 =                                          {
                                            0,                          /* validBitMap */
                                            pa_DEST_HOST,               /* Dest */
                                            0,                          /* Flow ID */
                                            0,                          /* queue */
                                            0,                          /* Multi route */
                                            T14_CMD_SWINFO0_PKT_ID,                             /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                        /* Egress Flow Info */
                                            0                           /* Ctrl Bit map */
                                         };


static paRouteInfo2_t nfailRoute_2 =                                          {
                                            0,                          /* validBitMap */
                                            pa_DEST_HOST,               /* Dest */
                                            0,                          /* Flow ID */
                                            0,                          /* queue */
                                            0,                          /* Multi route */
                                            T14_CMD_SWINFO0_PKT_ID,                             /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                        /* Egress Flow Info */
                                            0                           /* Ctrl Bit map */
                                         };

static paRouteInfo2_t matchRoute2[] = {
                                         {
                                            0,                          /* validBitMap */
                                            pa_DEST_HOST,               /* Dest */
                                            0,                          /* Flow ID */
                                            0,                          /* queue */
                                            0,                          /* Multi route */
                                            0,                          /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                        /* Egress Flow Info */
                                            0                           /* Ctrl Bit map */
                                         },

                                         {
                                            0,                          /* validBitMap */
                                            pa_DEST_CONTINUE_PARSE_LUT1,                /* Dest */
                                            0,                          /* Flow ID */
                                            0,                          /* queue */
                                            0,                          /* Multi route */
                                            0,                          /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                        /* Egress Flow Info */
                                            0                           /* Ctrl Bit map */
                                         },

                                         {
                                            0,                          /* validBitMap */
                                            pa_DEST_CONTINUE_PARSE_LUT2,                /* Dest */
                                            0,                          /* Flow ID */
                                            0,                          /* queue */
                                            0,                          /* Multi route */
                                            0,                          /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                       /* Egress Flow Info */
                                            0                           /* Ctrl Bit map */
                                         },

                                      {
                                            0,                          /* validBitMap */
                                            pa_DEST_HOST,               /* Dest */
                                            0,                          /* Flow ID */
                                            0,                          /* queue */
                                            0,                          /* Multi route */
                                            0,                          /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                        /* Egress Flow Info */
                                            0                           /* Ctrl Bit map */
                                         },

                                         {
                                            0,                          /* validBitMap */
                                            pa_DEST_HOST,               /* Dest */
                                            0,                          /* Flow ID */
                                            TF_PA_QUEUE_IPSEC2,                         /* queue */
                                            0,                          /* Multi route */
                                            0,                          /* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : not used  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                        /* Egress Flow Info */
                                            0                           /* Ctrl Bit map */
                                         },
 								  	                     {
                                            pa_ROUTE_INFO_VALID_CTRLBITMAP,  /* validBitMap            */
                                            pa_DEST_CONTINUE_PARSE_LUT1,     /* Dest */
                     								  			0,							/* Flow ID */
 								  	                    		0,							/* queue */
                  								  		   -1,							/* Multi route */
 								                    		    T14_CMD_SWINFO0_CAP_PKT_ID,	/* sw Info 0 */
                                            0,                          /* sw Info 1 */
                                            0,                          /* customType : pa_CUSTOM_TYPE_NONE  */
                                            0,                          /* customIndex: not used  */
                                            0,                          /* pkyType: for SRIO only */
                                            NULL,                       /* No commands            */
                                            0,                          /* Priority Type          */
                                            NULL,                       /* Egress Flow Info */
                                            pa_ROUTE_INFO_L2_PKT_CAPTURE/* ctrlBitMap */
 								       	                 },
                                      };



/* Global configurations */
#define T14_NUM_64B_USR_STATS            64

static paIpReassmConfig_t   t14OutIpReassmCfg =
        {
            2,      /* Number of traffic Flow */
            0,      /* CPPI Flow */
            TF_FIRST_GEN_QUEUE + Q_IP_REASSM1   /* destination queue */
        };

static paIpReassmConfig_t   t14InIpReassmCfg =
        {
            2,      /* Number of traffic Flow */
            0,      /* CPPI Flow */
            TF_FIRST_GEN_QUEUE + Q_IP_REASSM2   /* destination queue */
        };


static paUsrStatsConfig_t   t14UsrStatsCfg =
       {
            pa_USR_STATS_MAX_COUNTERS - T14_NUM_64B_USR_STATS,   /* Number of user stats (448)*/
            T14_NUM_64B_USR_STATS                                /* Number of 64-bit user stats */
       };

static  paIpsecNatTConfig_t  t14NatTCfg =
        {
            pa_IPSEC_NAT_T_CTRL_ENABLE |     /* ctrlBitMap */
            pa_IPSEC_NAT_T_CTRL_LOC_LUT1,
            4500                             /* UDP port number */
        };

/* Each EOAM target flow classification needs an user defined counter associated with it */
#define T14_NUM_EOAM_USR_STATS              ( sizeof (t14EoamInfo)/sizeof (t14EoamPaSetup_t))
#define T14_NUM_OTHER_USR_STATS             4

#define T14_USR_STATS_ID_TX_PKTS            ( T14_NUM_EOAM_USR_STATS + 0 )

#define T14_USR_STATS_ID_TX_BYTES           ( T14_NUM_EOAM_USR_STATS + 1 )
#define T14_USR_STATS_ID_L2_PADDING_ERR     ( T14_NUM_EOAM_USR_STATS + 2 )
#define T14_USR_STATS_ID_L2_TX_PADDING      ( T14_NUM_EOAM_USR_STATS + 3 )
#define T14_NUM_USR_STATS                   ( T14_NUM_EOAM_USR_STATS + T14_NUM_OTHER_USR_STATS )

static  pauUsrStatsEntry_t t14OtherUsrStatsTbl[T14_NUM_OTHER_USR_STATS] =
{
    /* index, lnkId, fAlloc, f64b, fByteCnt */
    {0, T14_USR_STATS_ID_TX_BYTES, TRUE,   FALSE, FALSE},      /* Tx packet count */
    {0, PAU_USR_STATS_NO_LINK,     TRUE,   TRUE,  TRUE},       /* Tx byte count */
    {0, PAU_USR_STATS_NO_LINK,     TRUE,   FALSE, FALSE},      /* L2 padding error count */
    {0, PAU_USR_STATS_NO_LINK,     TRUE,   FALSE, FALSE},      /* L2 padding count */
};


static  pauUsrStatsEntry_t t14UsrStatsTbl[T14_NUM_USR_STATS];

/*
 * User-definded Statitics Map
 *
 * Group 1: counter 0,  packet Counter (Rx Padding Error) no link
 *          counter 1,  packet Counter (Tx Padding) no link
 */

#ifdef _TMS320C6X
#pragma DATA_SECTION (t14UsrStatsGroup1, ".testPkts")
#endif

static paUsrStatsCounterEntryConfig_t t14UsrStatsGroup1[T14_NUM_USR_STATS];

#ifdef _TMS320C6X
#pragma DATA_SECTION(t14UsrStatsSetup, ".testPkts")
#endif
static pauUsrStatsSetup_t  t14UsrStatsSetup[] = {
    /* entry 0 */
    {
        sizeof(t14UsrStatsGroup1)/sizeof(paUsrStatsCounterEntryConfig_t),    /* number of entries */
        t14UsrStatsGroup1,                                                   /* counter Info table */
        pa_OK                                                               /* Expected return value */
    }
};

static  paEoamGlobalConfig_t t14EoamGlobalCfg =
        {
            pa_EOAM_VALID_STATS_CTRL,
            0,
            pa_INPUT_FREQ_1050MHZ,
            {
                1,                              /* number of protocols to exclude the statistics */
                0x88cc,                         /* Protocol */
            },
        };


static paPacketControlConfig_t  t14PktCtrlCfg =
        {
            0,                  /* ctrlBitMap */
            0,                  /* rxPaddingErrStatsIndex */
            0                   /* txPaddingStatsIndex */
        };


static  paRaGroupConfig_t t14OutRaGroupCfg =
        {
#ifdef PA_USE_HW_RA
            pa_RA_CTRL_ENABLE
            #ifdef NETSS_INTERNAL_PKTDMA
            | pa_RA_CTRL_USE_LOCAL_DMA
            #endif
            ,
            //pa_RA_CTRL_TO_QUEUE,              /* ctrlBitMap */
#else
            0,
#endif
            0,                                  /* Flow Id */
            {                                   /* TimeoutER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            },

            {                                   /* CritErrER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            },

            {                                   /* genErrER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            }

        };

static  paRaGroupConfig_t t14InRaGroupCfg =
        {
#ifdef PA_USE_HW_RA
            pa_RA_CTRL_ENABLE
            #ifdef NETSS_INTERNAL_PKTDMA
            | pa_RA_CTRL_USE_LOCAL_DMA
            #endif
            ,
            //pa_RA_CTRL_TO_QUEUE,                /* ctrlBitMap */
#else
            0,
#endif
            0,                                  /* Flow Id (5) */
            {                                   /* TimeoutER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            },

            {                                   /* CritErrER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            },

            {                                   /* genErrER */
                pa_DEST_DISCARD,                /* dest */
                0,                              /* flowId */
                0                               /* queue */
            }

        };

static  paSysConfig_t  t14GlobalCfg =
        {
            NULL,               /* pProtoLimit */
            &t14OutIpReassmCfg, /* pOutIpReassmConfig */
            &t14InIpReassmCfg,  /* pInIpReassmConfig */
            NULL,               /* pCmdSetConfig */
            NULL,               /* pUsrStatsConfig */
            NULL,               /* pQueueDivertConfig */
            NULL,               /* pPktControl */
            NULL,               /* pQueueBounceConfig */
            NULL,               /* pOutAclConfig */
            NULL,               /* pInAclConfig */
           &t14OutRaGroupCfg,   /* pOutIpRaGroupConfig */
           &t14InRaGroupCfg,    /* pInIpRaGroupConfig */
            NULL,               /* pPktControl2 */
           &t14EoamGlobalCfg    /* pEoamConfig */
        };

static  paSysConfig_t  t14GlobalCfg2 =
        {
            NULL,               /* pProtoLimit */
            NULL,               /* pOutIpReassmConfig */
            NULL,               /* pInIpReassmConfig */
            NULL,               /* pCmdSetConfig */
            NULL,               /* pUsrStatsConfig */
            NULL,               /* pQueueDivertConfig */
            &t14PktCtrlCfg,     /* pPktControl */
            NULL,               /* pQueueBounceConfig */
            NULL,               /* pOutAclConfig */
            NULL,               /* pInAclConfig */
            NULL,               /* pOutIpRaGroupConfig */
            NULL,               /* pInIpRaGroupConfig */
            NULL,               /* pPktControl2 */
            NULL    /* pEoamConfig */
        };

static  paSetTimeOffset_t  t14TimeOffsetCfg =
  {
    0,  /* Offset for seconds */
    0   /* Offset for nano seconds */
  };

static paTxChksum_t t14pktChksum = {  /* The UDP checksum command */

        0,      /* Start offset of UDP header */
        0,      /* Checksum length (UDP header + payload */
        6,      /* Offset to checksum location RELATIVE TO THE START OF THE TCP/UDP HEADER */
        0,      /* Initial value is IPv4 pseudo header checksum value */
        1       /* computed value of 0 written as -0 */

    };

static  paCmdNextRoute_t t14pktRoute1 = { pa_NEXT_ROUTE_PARAM_PRESENT |
                                          pa_NEXT_ROUTE_PROC_USR_STATS,  /* ctrlBitfield */
                                          pa_DEST_HOST,        /* Dest */
                                          0,                   /* pkyType: for SRIO only */
                                          0,                   /* Flow ID */
                                          TF_PA_QUEUE_TXCMD,   /* queue */
                                          0,                   /* sw Info 0 */
                                          0,                   /* sw Info 1 */
                                          0,                    /* Multi route */
                                          0};

static  paCmdNextRoute_t t14pktRoute2 = { pa_NEXT_ROUTE_PARAM_PRESENT |   /* ctrlBitfield */
                                          pa_NEXT_ROUTE_PROC_NEXT_CMD,
                                          pa_DEST_HOST,       /* Dest */
                                          0,                  /* pkyType: for SRIO only */
                                          0,                  /* Flow ID */
                                          TF_PA_QUEUE_INPUT,  /* queue */
                                          0x12345678,         /* sw Info 0 */
                                          0x87654321,         /* sw Info 1 */
                                          0,                  /* Multi route */
                                          0};

static paPatchInfo_t t14pktPatch = {  /* The Blind patch command */

                                            0,      /* ctrlBitfield (Overwrite) */
                                            8,      /* The number of bytes to be patched */
                                            8,      /* The number of patch bytes in the patch command */
                                            0,      /* offset */
                                            0       /* Pointer to the patch data */

                                   };
static uint16_t t14MtuSize[T14_NUM_LOCAL_L4_HANDLES] = {
                                                        80,
                                                        200,
                                                        300,
                                                        512};

static uint16_t t14pktPayloadSize[T14_NUM_LOCAL_L4_HANDLES][10] =
{
    {32,   40,  60, 80,  100, 120,  80, 60,  40,   20},    /* MTU size = 80   */
    {60,  300, 500, 120, 400, 250, 100, 500, 80,   300},   /* MTU size = 200 */
    {300,  60, 500, 600, 400, 100, 100, 700, 350,  100},   /* MTU size = 300 */
    {100, 500, 550, 60,  900, 600, 400, 850, 1500, 1000},  /* MTU size = 512 */
};

#define T14_NUM_REASM_PKTS    sizeof(t14ReasmPktInfo)/sizeof(pktTestInfo2_t)
#ifdef _TMS320C6X
#pragma DATA_SECTION(t14pktBuf, ".testPkts")
#endif
static uint8_t t14pktBuf[T14_NUM_REASM_PKTS][2000];

#ifdef _TMS320C6X
#pragma DATA_SECTION(t14Encap, ".testPkts")
#endif
t14TestEncap_t  t14Encap;


/* Simple User-Statistics routine: It does not process link and counter type */
static void t14UpdateUsrStats(paUsrStats_t* pStats, uint16_t cntIndex, uint32_t size)
{
    if (cntIndex < T14_NUM_64B_USR_STATS)
    {
        pStats->count64[cntIndex] += (uint64_t)size ;
    }
    else
    {
        pStats->count32[cntIndex - T14_NUM_64B_USR_STATS] += size;
    }
}

void t14Cleanup (t14TestEncap_t *tencap, paTestStatus_t testStatus)
{
    int            i;
    int            cmdDest;
    uint16_t       cmdSize;
    paReturn_t     paret;
    Cppi_HostDesc *hd;
    paTestStatus_t newStatus = PA_TEST_PASSED;

    /* Delete active L4 handles */
    for (i = 0; i < T14_NUM_LOCAL_L4_HANDLES; i++)  {

        cmdReply.replyId = T14_CMD_SWINFO0_DEL_PORT_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
        cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

        if ((tencap->l4Handles[i].state == T14_HANDLE_PENDING_ACK) || (tencap->l4Handles[i].state == T14_HANDLE_ACTIVE))  {
            hd = testCommonDelL4Handles (tencap->tf, tencap->l4Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
                    &cmdReply, &cmdDest, &cmdSize, &paret);

            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
                continue;
            }

            Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
            paTestL2ExpectedStats.classify2.nPackets += 1;
            #ifdef NSS_GEN2
            paTestL2ExpectedStats.classify1.nPackets += 1;
            #endif

            /* Wait for the response */
            if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
                System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
                testStatus = PA_TEST_FAILED;
            }

            /* Recycle the command packet as well */
            hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
                continue;
            }
            testCommonRecycleLBDesc (tencap->tf, hd);
        }
    }

    /* Delete active L3 Handles */
    for (i = 0; i < T14_NUM_LOCAL_L3_HANDLES; i++)  {
        cmdReply.replyId = T14_CMD_SWINFO0_DEL_IP_ID + i;  /* T14_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
        cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

        if ((tencap->l3Handles[i].state == T14_HANDLE_PENDING_ACK) || (tencap->l3Handles[i].state == T14_HANDLE_ACTIVE))  {
            hd = testCommonDelHandle (tencap->tf, &tencap->l3Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
                    &cmdReply, &cmdDest, &cmdSize, &paret);

            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
                continue;
            }

            Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
            paTestL2ExpectedStats.classify1.nPackets += 1;

            /* Wait for the response */
            if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
                System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
                testStatus = PA_TEST_FAILED;
            }
            /* Recycle the command packet as well */
            hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
                continue;
            }
            testCommonRecycleLBDesc (tencap->tf, hd);


        }
    }


    /* Delete active L2 Handles */
    for (i = 0; i < T14_NUM_LOCAL_L2_HANDLES; i++)  {
        cmdReply.replyId = T14_CMD_SWINFO0_DEL_MAC_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
        cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

        if ((tencap->l2Handles[i].state == T14_HANDLE_PENDING_ACK) || (tencap->l2Handles[i].state == T14_HANDLE_ACTIVE))  {
            hd = testCommonDelHandle (tencap->tf, &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
                    &cmdReply, &cmdDest, &cmdSize, &paret);

            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
                continue;
            }

            Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
            paTestL2ExpectedStats.classify1.nPackets += 1;

            /* Wait for the response */
            if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
                System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
                testStatus = PA_TEST_FAILED;
            }

            /* Recycle the command packet as well */
            hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
                continue;
            }
            testCommonRecycleLBDesc (tencap->tf, hd);


        }
    }

    /* Delete active EOAM Handles */
    for (i = 0; i < T14_NUM_LOCAL_EOAM_HANDLES; i++)  {
        cmdReply.replyId = T14_CMD_SWINFO0_DEL_EOAM_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
        cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

        if ((tencap->eoamHandles[i].state == T14_HANDLE_PENDING_ACK) || (tencap->eoamHandles[i].state == T14_HANDLE_ACTIVE))  {
            hd = testCommonDelEoamHandle (tencap->tf, &tencap->eoamHandles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
                    &cmdReply, &cmdDest, &cmdSize, &paret);

            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
                System_flush();
                continue;
            }

            Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
            paTestL2ExpectedStats.classify1.nPackets += 1;

            /* Wait for the response */
            if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
                System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
                testStatus = PA_TEST_FAILED;
            }
            /* Recycle the command packet as well */
            hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
                System_flush();
                continue;
            }
            testCommonRecycleLBDesc (tencap->tf, hd);
        }
    }

   	/* Delete active ACL Handles */
   	for (i = 0; i < T14_NUM_LOCAL_ACL_HANDLES; i++)  {
   		cmdReply.replyId = T14_CMD_SWINFO0_DEL_ACL_ID + i;  /* T2_CMD_SWINFO0_ADD_ID identifies command, 16 LS bits identify the local handle number */
   		cmdReply.queue   = tencap->tf->QGen[Q_CMD_REPLY];

  		if ((tencap->aclHandles[i].state == T14_HANDLE_PENDING_ACK) || (tencap->aclHandles[i].state == T14_HANDLE_ACTIVE))  {
  			hd = testCommonDelAclHandle (tencap->tf, &tencap->aclHandles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
  					&cmdReply, &cmdDest, &cmdSize, &paret);

  			if (paret != pa_OK)  {
  				System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, __LINE__, paret);
  				continue;
  			}

  			Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
  			paTestL2ExpectedStats.classify1.nPackets += 1;

  			/* Wait for the response */
  			if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, cmdReply.replyId, __LINE__)) {
  				System_printf ("%s (%s:%d): testCommonWaitCmdReply failed\n", tfName, __FILE__, __LINE__);
  				testStatus = PA_TEST_FAILED;
   			}
   			/* Recycle the command packet as well */
   			hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
   			if (hd == NULL)  {
   				System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
   				continue;
   			}
   			testCommonRecycleLBDesc (tencap->tf, hd);
   		}
   	}

    /* Pop any descriptors off of the return queues and restore them to the linked buffer queues or the free Q */
    while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_CMD_RECYCLE]) > 0)  {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
        testCommonRecycleLBDesc (tencap->tf, hd);
    }

    while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_CMD_REPLY]) > 0)  {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_REPLY])) & ~15);
        testCommonRecycleLBDesc (tencap->tf, hd);
    }

     while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_MATCH]) > 0)  {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
        Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
    }

     while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_NFAIL]) > 0)  {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_NFAIL])) & ~15);
        Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
    }

    while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_PARSE_ERR]) > 0)  {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_PARSE_ERR])) & ~15);
        Qmss_queuePush (tencap->tf->QfreeDesc, (Ptr)hd, hd->buffLen, TF_SIZE_DESC, Qmss_Location_TAIL);
    }

#ifdef TO_BE_COMPLETED
    newStatus = testCommonCheckStats (tencap->tf, tencap->pat, tfName, &paTestL2ExpectedStats, tencap->tf->QLinkedBuf1,
                           tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QGen[Q_CMD_REPLY], TRUE);
    if (newStatus == PA_TEST_FAILED)
        testStatus = PA_TEST_FAILED;
#else
  newStatus = newStatus;
#endif

    /* Return result */
    tencap->pat->testStatus = testStatus;

}

static void t14Ipv6UpdatesStats(paSysStats_t *stats, int pktIndex, uint16_t pktLen, uint16_t mtuSize, uint16_t ipOffset, uint16_t l4Offset, int ah, int first)
{

    int32_t numFrags;
    uint16_t ipHdrLen;
    uint16_t fragSize;
    uint16_t payloadSize;

    static int32_t fFrags[T14_NUM_LOCAL_L4_HANDLES] = {FALSE, FALSE, FALSE, FALSE};


    /* Reset the fragement flag when the first packet in this packet stream is generated */
    if(first)fFrags[pktIndex] = FALSE;

    mtuSize &= 0xFFF8;

    payloadSize = (pktLen - l4Offset);

    ipHdrLen = (l4Offset - ipOffset) + 8;
    fragSize = mtuSize - ipHdrLen;

    numFrags = 1;

    if ((fragSize + 8) < payloadSize)
    {
        /* note: the condition for non-fragmented packet should not include the fragment header */
        while (payloadSize > fragSize)
        {
            numFrags++;
            payloadSize -= fragSize;
        }
    }

    if (numFrags > 1)
    {
        fFrags[pktIndex] = TRUE;
        stats->classify1.nTxIpFrag += numFrags;
        stats->classify1.nIpFrag   += numFrags;
    }

    /* MAC update due to fragments */
    stats->classify1.nPackets += (numFrags - 1);
    stats->classify1.nTableMatch += (numFrags - 1);

    /* IP Reassembly updates */
    if (fFrags[pktIndex])
    {
        stats->classify1.nPackets += numFrags;
    }
}

static void t14Ipv4UpdatesStats(paSysStats_t *stats, int pktIndex, uint16_t pktLen, uint16_t mtuSize, uint16_t ipOffset, int ah, int first)
{

    int      numFrags;
    uint16_t fragSize, lastFragSize;
    uint16_t payloadSize;

    static int fFrags[T14_NUM_LOCAL_L4_HANDLES] = {FALSE, FALSE, FALSE, FALSE};


    /* Reset the fragement flag when the first packet in this packet stream is generated */
    if(first)fFrags[pktIndex] = FALSE;

    payloadSize = (pktLen - ipOffset) - 20;
    lastFragSize = mtuSize - 20;
    fragSize = lastFragSize & 0xFFF8;

    numFrags = 1;
    while (payloadSize >= lastFragSize)
    {
        numFrags += 1;
        payloadSize -= fragSize;
    }

    if (numFrags > 1)
    {
        fFrags[pktIndex] = TRUE;
        stats->classify1.nTxIpFrag += numFrags;
        stats->classify1.nIpFrag   += numFrags;
    }

    /* MAC update due to fragments */
    stats->classify1.nPackets += (numFrags - 1);
    stats->classify1.nTableMatch += (numFrags - 1);

    /* IP Reassembly updates */
    if (fFrags[pktIndex])
    {
        stats->classify1.nPackets += numFrags;
    }
}
int t14NumTxPadding = 0;

/* Padding check for IP fragments only */
static int t14TxPaddingCheck(Cppi_HostDesc *hd, int* cnt)
{
    pasahoLongInfo_t    *pInfo;
    uint32_t             infoLen;
    uint16_t             ipOffset;
    uint16_t             fragOffset;
    uint16_t             pktLen, ipLen;
    uint8_t*             ipHdr;

    /* Get the parse information */
    if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pInfo, &infoLen) != CPPI_SOK)  {
        System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
        return (-1);
    }

    /* Extract IP related information. */
    ipOffset = PASAHO_LINFO_READ_START_OFFSET(pInfo);
    pktLen = Cppi_getPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd);

    ipHdr = (uint8_t *)(hd->buffPtr + ipOffset);

    fragOffset = (ipHdr[6] << 8) + ipHdr[7];
    fragOffset &= 0x3fff;

    if (!fragOffset)
    {
        /* It is a non-fragmented packet, padding check should be done later */
        return 0;
    }

    ipLen = (ipHdr[2] << 8) + ipHdr[3];

    if ((ipLen + ipOffset) < 60)
    {
        /* padding is required */
        t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[T14_USR_STATS_ID_L2_TX_PADDING].cntIndex, 1);
        *cnt += 1;
        if(pktLen != 60)
            return -1;
    }

    return 0;

}

/* Padding check for fragments packet at the final matched location */
static int t14TxPaddingCheck2(Cppi_HostDesc *hd, int* cnt)
{
    pasahoLongInfo_t    *pInfo;
    uint32_t             infoLen;
    uint16_t             endOffset;
    uint16_t             pktLen;

    /* Get the parse information */
    if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pInfo, &infoLen) != CPPI_SOK)  {
        System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
        return (-1);
    }

    /* Extract IP related information. */
    endOffset = PASAHO_LINFO_READ_END_OFFSET(pInfo);
    pktLen = Cppi_getPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd);

    if (endOffset < 60)
    {
        /* padding is required */
        t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[T14_USR_STATS_ID_L2_TX_PADDING].cntIndex, 1);
        *cnt += 1;
        if(pktLen != 60)
            return -1;
    }

    return 0;

}

#ifndef SIMULATOR_SUPPORT
static void t14IpRreassemTimeoutUpdatesStats(paSysStats_t *stats, uint32_t count)
{
    stats->classify1.nIpv4Packets -= count;
    stats->classify1.nTableMatch -= count;
    stats->classify2.nUdp -= count;
    stats->classify2.nPackets -= count;

}
#endif


#define MAX_RX_FRAGS_PER_CALL       20

static int t14RxFrags(t14TestEncap_t *tencap, Qmss_QueueHnd inQ, Qmss_QueueHnd outQ, int* pExpCnt)
{
    Cppi_HostDesc*       hd[MAX_RX_FRAGS_PER_CALL];
    int                  fragCnt = 0;
    int                  i, j, mid;
    Qmss_Queue           q;

    q.qMgr = 0;
    q.qNum = TF_DEF_RET_Q;
    /* Process the received fragments */
    while ((Qmss_getQueueEntryCount(inQ) > 0) && (fragCnt < MAX_RX_FRAGS_PER_CALL))  {

        hd[fragCnt]  = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (inQ)) & ~15);
        if (hd[fragCnt] == NULL)  {
            System_printf ("%s (%s:%d): Failed to pop a received packet from queue %d\n", tfName, __FILE__, __LINE__, inQ);
            return (-1);
        }

        if(t14TxPaddingCheck(hd[fragCnt], &t14NumTxPadding)) {
            System_printf ("%s (%s:%d): Tx padding check failed for received packet from queue %d\n", tfName, __FILE__, __LINE__, inQ);
            return (-1);
        }

        /* All the fragments should be cleaned up after reassembled */
        Cppi_setReturnPolicy (Cppi_DescType_HOST, (Cppi_Desc *) hd[fragCnt], Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET);
        Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd[fragCnt], q);

        fragCnt++;

    }

    /*
     * Send fragments to the reassembly function in alternative order to verify the the IP reassembly route support
     * multiple reassembly context simultaneouely.
     */

    mid = (fragCnt + 1)/2;

    for ( i = 0, j = mid; i < mid; i++, j++)
    {
        if (paEx_reassemLibProc(hd[i], outQ))
        {
            System_printf ("%s (%s:%d): sample_procPkts(%d) return error!\n", tfName, i, __FILE__, __LINE__);
            testCommonRecycleLBDesc (tencap->tf, hd[i]);
            return (-1);
        }

        #ifndef  SIMULATOR_SUPPORT
        /* There is a pending simulator timing prroblem. To be enabled after the simulator bug is fixed */
        if ((j == (fragCnt - 1)) && (i == (mid -1)))
        {
            uint32_t timeoutCnt;
            /*
             * Test IP Reassembly timeout:
             * This fragment (packet) should be the last one within its group
             * If it is an IP fragments, the paEx_reassemLibTimerTick() will flush out the remaining fragments
             * If it is the case, update the corresponding statistics and *pExpCnt
             */
            paIPReassemblyStats_t stats;

            paEx_reassemLibQueryStats(&stats, FALSE);
            timeoutCnt = stats.reassemblyTimeout;

            /* Verify the Reassembly timeout operation */
            paEx_reassemLibTimerTick(5001);

            /* adjust the expected statistics */
            paEx_reassemLibQueryStats(&stats, FALSE);
            timeoutCnt = stats.reassemblyTimeout - timeoutCnt;

            if (timeoutCnt)
            {
                t14IpRreassemTimeoutUpdatesStats(&paTestL2ExpectedStats, timeoutCnt);
                *pExpCnt -= timeoutCnt;
            }

            if (paEx_reassemLibProc(hd[j], outQ))
            {
                System_printf ("%s (%s:%d): sample_procPkts(%d) return error!\n", tfName, j, __FILE__, __LINE__);
                testCommonRecycleLBDesc (tencap->tf, hd[j]);
                return (-1);
            }

            if (timeoutCnt)
            {
                paEx_reassemLibTimerTick(5001);  /* kick out the last fragments */
                paTestL2ExpectedStats.classify1.nPackets++; /* There is an extra null packet entering PDSP1/PDSP2 */
            }

        }
        else
        #endif
        if (j < fragCnt)
        {
            if (paEx_reassemLibProc(hd[j], outQ))
            {
                System_printf ("%s (%s:%d): sample_procPkts(%d) return error!\n", tfName, j, __FILE__, __LINE__);
                testCommonRecycleLBDesc (tencap->tf, hd[j]);
                return (-1);
            }
        }
    }

    /* Since the packets went to the modify PDSP and then back to the QM, a descriptor
     * and linked buffer was required while the packet was in the QM. This will
     * be recycled to the default recycle queue */
    while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
        hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
        if (hd[0] == NULL)  {
            System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
            return (-1);
        }

        testCommonRecycleLBDesc (tencap->tf, hd[0]);

    }

    return (0);
}


/* Search the receive data packet queue for received data packets. Remain in
 * this function until all buffers are restored to their respective queues */
static int t14ReceiveDataPkts (t14TestEncap_t *tencap, int expCount)
{
    Cppi_HostDesc    *hd;
    uint32_t             *swInfo;
    pasahoLongInfo_t *pinfo;
    uint32_t              infoLen;
    int               i;
    int               pktId = 0;
    uint32_t              flags;
    unsigned int              eflags;
    int               count = 0;


    for (i = 0; i < 100; i++)  {

        utilCycleDelay (5000);

        /* Process OutIp fragments */
        if (t14RxFrags(tencap, tencap->tf->QGen[Q_IP_REASSM1], tencap->tf->QPaTx[TF_PA_Q_OUTER_IP], &expCount))
        {
            return (-1);

        }

        /* Process Inner fragments */
        if (t14RxFrags(tencap, tencap->tf->QGen[Q_IP_REASSM2], tencap->tf->QPaTx[TF_PA_Q_INNER_IP], &expCount))
        {
            return (-1);
        }

        /* Look for ESP packets: Update the startOffset and endOffset,
         * Push to Ingress3: for nextHdr is IP
         *         Ingress4: Otherwise
         *
         * Note: Fixed ivSize (8) and icvSize (12) are used since it is just a proof-of-concept test.
         *       Those two variables may be added to the packet packet information in enhanced test cases
         */
        while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_ESP_FIX]) > 0)  {
            uint8_t  startOffset;
            uint16_t endOffset;

            hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_ESP_FIX])) & ~15);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): Failed to pop a received ESP packet\n", tfName, __FILE__, __LINE__);
                return (-1);
            }

            /* Verify the parse information is correct */
            if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
                System_printf ("%s (%s:%d): Error getting control info from received ESP packet\n", tfName, __FILE__, __LINE__);
                testCommonRecycleLBDesc (tencap->tf, hd);
                return (-1);
            }

            startOffset = PASAHO_LINFO_READ_START_OFFSET(pinfo) + 8;
            PASAHO_LINFO_SET_START_OFFSET(pinfo, startOffset);

            endOffset = PASAHO_LINFO_READ_END_OFFSET(pinfo) - 12;
            PASAHO_LINFO_SET_END_OFFSET(pinfo, endOffset);

            PASAHO_LINFO_SET_NXT_HDR_TYPE(pinfo, PASAHO_HDR_ESP_DECODED);

            while (endOffset > hd->buffLen)
            {
                endOffset -= hd->buffLen;
                hd = (Cppi_HostDesc *)hd->nextBDPtr;
            }

            Qmss_queuePushDescSize (tencap->tf->QPaTx[TF_PA_Q_FIREWALL2], (Ptr)hd, TF_SIZE_DESC);

           utilCycleDelay (10000);

        }

        utilCycleDelay (10000);

        /* Look for packets in the rx packet queue, verify that the UDP port found matches the next value in
        * the fifo, then verify the receive packet information */
        while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_MATCH]) > 0)  {

            utilCycleDelay (5000);

            hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): Failed to pop a received packet\n", tfName, __FILE__, __LINE__);
                return (-1);
            }

            if(t14TxPaddingCheck2(hd, &t14NumTxPadding)) {
                System_printf ("%s (%s:%d): Tx padding check failed for reassembled packet\n", tfName, __FILE__, __LINE__);
                return (-1);
            }

            /* Verify swInfo0 for packet match and packet ID number */
            Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

            if ((*swInfo & T14_CMD_SWINFO0_TYPE_MASK) != T14_CMD_SWINFO0_PKT_ID)  {
                System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
                               tfName, __FILE__, __LINE__, *swInfo);
                testCommonRecycleLBDesc (tencap->tf, hd);
                return (-1);
            }

            pktId = *swInfo & T14_CMD_SWINFO0_ID_MASK;

            if (pktId >= T14_NUM_LOCAL_L4_HANDLES)  {
                System_printf ("%s (%s:%d): Found a packet with unexpected packet id %d (max = %d)\n",
                    tfName, __FILE__, __LINE__, pktId, T14_NUM_LOCAL_L4_HANDLES - 1);
                testCommonRecycleLBDesc (tencap->tf, hd);
                return (-1);
            }

            /* Verify the parse information is correct */
            if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
                System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
                testCommonRecycleLBDesc (tencap->tf, hd);
                return (-1);
            }

            flags = PASAHO_LINFO_READ_HDR_BITMASK(pinfo);

            if ((flags & (PASAHO_HDR_BITMASK_UDP | PASAHO_HDR_BITMASK_TCP)) == 0)  {
                System_printf ("%s (%s:%d): Found a packet without any L4 info\n", tfName, __FILE__, __LINE__);
                testCommonRecycleLBDesc (tencap->tf, hd);
                return (-1);
            }

            eflags = Cppi_getDescError (Cppi_DescType_HOST, (Cppi_Desc *)hd) & 0xf;


            if (eflags != 0)  {
                System_printf ("%s (%s:%d): Packet with index %d returned with error flags = 0x%02x\n", tfName, __FILE__, __LINE__, pktId, eflags);
            }


            /* Return the descriptor/buffer */
            testCommonRecycleLBDesc (tencap->tf, hd);

            count++;

        }

        if(count >= expCount)
            break;

    }

    if (i == 100)  {
        System_printf ("%s (%s:%d): Error - unable to recover all packets\n", tfName, __FILE__, __LINE__);
        System_flush();
        return (-1);
    }

    /* Since the packets went to the modify PDSP and then back to the QM, a descriptor
     * and linked buffer was required while the packet was in the QM. This will
     * be recycled to the default recycle queue */
    while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
        utilCycleDelay (5000);
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
        if (hd == NULL)  {
            System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
            return (-1);
        }

        testCommonRecycleLBDesc (tencap->tf, hd);

    }

    return (0);

}

static void t14SendDataPkts (t14TestEncap_t *tencap, int nPkts, int group, int fFirst)
{
  Cppi_HostDesc      *hd;
  Qmss_Queue         q;
  int                i, ipv6Pkt, flag;
  uint16_t           cmdStackSize, ipPayloadLen;
  paReturn_t         paret;
  uint8_t            *buf;
  uint16_t           payloadLen;
  paCmdInfo_t        cmdInfo[7];
  paTxChksum_t       *pTxChksum = &cmdInfo[0].params.chksum;
  paPatchInfo_t      *pTxPatch = &cmdInfo[6].params.patch;
  int                ipOffset, ipOffset2, udpOffset, ahEspOffset;
  volatile int       mdebugWait = 1;

  /* Group 1: UDP checksum */
  cmdInfo[0].cmd = pa_CMD_TX_CHECKSUM;
  cmdInfo[0].params.chksum = t14pktChksum;
  cmdInfo[1].cmd = pa_CMD_NEXT_ROUTE;
  cmdInfo[1].params.route = t14pktRoute1;
  /* Group 2: IP fragmentation plus Blind Patch (Simulating AH haeder patch) */
  /* cmdInfo[2].cmd = pa_CMD_PATCH_MSG_LEN or pa_CMD_NONE */
  cmdInfo[2].params.patchMsgLen.msgLenSize = 2;
  cmdInfo[2].params.patchMsgLen.offset = 14 + 4;
  cmdInfo[2].params.patchMsgLen.msgLen = 2;
  cmdInfo[3].params.patchMsgLen.msgLenSize = 2;
  cmdInfo[3].params.patchMsgLen.offset = 14 + 4;
  cmdInfo[3].params.patchMsgLen.msgLen = 2;
  cmdInfo[4].cmd = pa_CMD_IP_FRAGMENT;
  /* cmdInfo[4].params.ipFrag= t14pktIpFrag; */
  cmdInfo[5].cmd = pa_CMD_NEXT_ROUTE;
  cmdInfo[5].params.route = t14pktRoute2;
  cmdInfo[6].cmd = pa_CMD_PATCH_DATA;
  cmdInfo[6].params.patch = t14pktPatch;

  /* Attach one free descriptor to each of the packets */
  for (i = 0; i < nPkts; i++)  {
    if (t14ReasmPktInfo[i].ctrlBitMap & T14_PKT_TYPE_IPV6 ) {
      ipv6Pkt = 1;
      cmdInfo[4].params.ipFrag.ipOffset = 14;
      cmdInfo[4].params.ipFrag.mtuSize  = 256;
    }
    else {
      ipv6Pkt  = 0;
      cmdInfo[4].params.ipFrag.ipOffset = 0;
      cmdInfo[4].params.ipFrag.mtuSize  = 256;
    }

    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);
    if (tencap->hd[i] == NULL)  {
      System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
      t14Cleanup (tencap, PA_TEST_FAILED);
    }
    /* Setup the return for the descriptor */
    q.qMgr = 0;
    q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);

    buf        = t14pktBuf[i];
    payloadLen = t14pktPayloadSize[i % nPkts][group];
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)buf), (uint32_t)(t14ReasmPktInfo[i].testInfo.pktLen + payloadLen));
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t14ReasmPktInfo[i].testInfo.pktLen + payloadLen);

    memcpy(buf, t14ReasmPktInfo[i].testInfo.pkt, t14ReasmPktInfo[i].testInfo.pktLen);
    testGenPayload(PAU_PAYLOAD_INC8, 0, payloadLen, &buf[t14ReasmPktInfo[i].testInfo.pktLen]);

    t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[T14_USR_STATS_ID_TX_PKTS].cntIndex, 1);
    t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[T14_USR_STATS_ID_TX_BYTES].cntIndex,
                       (uint32_t)t14ReasmPktInfo[i].testInfo.pktLen + payloadLen);

    /* Update the packet header and command based on the payload size */
    udpOffset = PASAHO_LINFO_READ_L4_OFFSET(t14ReasmPktInfo[i].testInfo.info);
    ipOffset =  PASAHO_LINFO_READ_L3_OFFSET(t14ReasmPktInfo[i].testInfo.info);
    ipOffset2 = udpOffset - 20;

    if (ipv6Pkt) {
      ipOffset2 -= 20;
      ipPayloadLen = (udpOffset - ipOffset) - 40 + payloadLen + 8;
    }
    flag = 0;
    if (ipv6Pkt) {
      /*
       * For our test case, the following assumptions are made
       * 1. The total szie of extension headers of outer IP is less than 40 bytes
       * 2. There is no extension header for outer IP
       */
      if ((ipOffset2 - ipOffset) >= 40) {
        uint32_t hdrbitmask = PASAHO_LINFO_READ_HDR_BITMASK(t14ReasmPktInfo[i].testInfo.info);
        if (hdrbitmask & PASAHO_HDR_BITMASK_AH) {
            /* Support IPSec AH packets */
            buf[ipOffset + 4] = (ipPayloadLen ) >> 8;
            buf[ipOffset + 5] = (ipPayloadLen ) & 0xFF;
            buf[ipOffset + 3] = (uint8_t)group;
            buf[udpOffset + 4] =  (payloadLen + 8) >> 8;
            buf[udpOffset + 5] =  (payloadLen + 8) & 0xFF;

            ahEspOffset = PASAHO_LINFO_READ_ESP_AH_OFFSET(t14ReasmPktInfo[i].testInfo.info);
            flag = 1;
        }
        else {
        /* It is IP over IP packet
         * Not supported */
        return;
        }
      }
      else {
        buf[ipOffset + 4] = (ipPayloadLen) >> 8;
        buf[ipOffset + 5] = (ipPayloadLen) & 0xFF;
        buf[ipOffset + 3] = (uint8_t)group;
        buf[udpOffset + 4] =  (payloadLen + 8) >> 8;
        buf[udpOffset + 5] =  (payloadLen + 8) & 0xFF;
      }

      pTxChksum->startOffset = udpOffset;
      pTxChksum->lengthBytes = payloadLen + 8;
      pTxChksum->initialSum  = utilGetIpPsudoChkSum(&buf[ipOffset], payloadLen + 8, 0x11);
    }
    else {
      if (ipOffset != ipOffset2) {
        uint32_t hdrbitmask = PASAHO_LINFO_READ_HDR_BITMASK(t14ReasmPktInfo[i].testInfo.info);
        if (hdrbitmask & PASAHO_HDR_BITMASK_AH) {
            int ahLen;

            ahEspOffset = PASAHO_LINFO_READ_ESP_AH_OFFSET(t14ReasmPktInfo[i].testInfo.info);
            ahLen = (buf[ahEspOffset + 1] + 2 )* 4;

            /* Support IPSec AH packets */
            buf[ipOffset + 2] = (payloadLen + ahLen + 28 ) >> 8;
            buf[ipOffset + 3] = (payloadLen + ahLen + 28) & 0xFF;
            buf[ipOffset + 5] = (uint8_t)group;
            buf[udpOffset + 4] =  (payloadLen + 8) >> 8;
            buf[udpOffset + 5] =  (payloadLen + 8) & 0xFF;
            flag = 1;
        }
        else {
        /* It is IP over IP packet
         * Not supported */
        return;
        }
      }
      else {
        buf[ipOffset2 + 2] = (payloadLen + 28) >> 8;
        buf[ipOffset2 + 3] = (payloadLen + 28) & 0xFF;
        buf[ipOffset2 + 5] = (uint8_t)group;
        buf[udpOffset + 4] =  (payloadLen + 8) >> 8;
        buf[udpOffset + 5] =  (payloadLen + 8) & 0xFF;
      }
      /* Update IP checksum */
      // utilUpdateIpChksums(&buf[ipOffset2]);
      pTxChksum->startOffset = udpOffset;
      pTxChksum->lengthBytes = payloadLen + 8;
      pTxChksum->initialSum  = utilGetIpPsudoChkSum(&buf[ipOffset], payloadLen + 8, 0x11);
    }

    /* Add Message Length Patch command if PPPoE header is included */
    cmdInfo[2].cmd = cmdInfo[3].cmd =  (ipOffset > 14)?pa_CMD_PATCH_MSG_LEN:pa_CMD_NONE;
    cmdInfo[4].params.ipFrag.ipOffset = ipOffset;

    cmdInfo[4].params.ipFrag.mtuSize = t14MtuSize[i % nPkts];
    pTxPatch->offset = udpOffset + 8;
    pTxPatch->patchData = &buf[udpOffset + 8];  /* patch with the same data */

    cmdStackSize = sizeof(tencap->cmdStackReasm[i]);
    paret = Pa_formatTxCmd ( 7,
                             cmdInfo,
                             0,
                             (Ptr)&tencap->cmdStackReasm[i],    /* Command buffer       */
                             &cmdStackSize);               /* Command size         */

    if (paret != pa_OK)  {
      System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
      t14Cleanup (tencap, PA_TEST_FAILED);
    }

    Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)&tencap->cmdStackReasm[i], cmdStackSize);

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X

    /*
     * Write back the entire cache to make sure that the test packets are updated.
     * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
     *       the portion of packet which is updated.
     *
     */
    CACHE_wbAllL1d(CACHE_WAIT);
    //CACHE_wbAllL2(CACHE_WAIT);
#endif
#endif

    /* Send the data to the modify PDSP */
    //mdebugHaltPdsp(5);
    Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)tencap->hd[i], (uint32_t)t14ReasmPktInfo[i].testInfo.pktLen + payloadLen, TF_SIZE_DESC, Qmss_Location_TAIL);
    //while (mdebugWait);

    testCommonIncStats (t14ReasmPktInfo[i].statsMap, &paTestL2ExpectedStats);
    testCommonIncStats (t14ReasmPktInfo[i].testInfo.statsMap, &paTestL2ExpectedStats);
    if (ipv6Pkt) {
      if (flag)
        t14Ipv6UpdatesStats(&paTestL2ExpectedStats, i, t14ReasmPktInfo[i].testInfo.pktLen + payloadLen,  t14MtuSize[i % nPkts], ipOffset, ahEspOffset, TRUE, fFirst);
      else
        t14Ipv6UpdatesStats(&paTestL2ExpectedStats, i, t14ReasmPktInfo[i].testInfo.pktLen + payloadLen,  t14MtuSize[i % nPkts], ipOffset, udpOffset, FALSE, fFirst);
    }
    else {
      if (flag)
        t14Ipv4UpdatesStats(&paTestL2ExpectedStats, i, t14ReasmPktInfo[i].testInfo.pktLen + payloadLen,  t14MtuSize[i % nPkts], ipOffset, TRUE, fFirst);
      else
        t14Ipv4UpdatesStats(&paTestL2ExpectedStats, i, t14ReasmPktInfo[i].testInfo.pktLen + payloadLen,  t14MtuSize[i % nPkts], ipOffset, FALSE, fFirst);
    }
    utilCycleDelay (20000);
  }

  /* Wait for descriptors to return. It is assumed that they are returning in order. */
  for (i = 0; i < 100; i++)  {
    if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= nPkts)
      break;
    utilCycleDelay (500);
  }

  if (i == 100)  {
       System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
       t14Cleanup (tencap, PA_TEST_FAILED);
  }

  /* Recycle the descriptors */
  for (i = 0; i < nPkts; i++) {
    tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE])) & ~15);
    if (tencap->hd[i] == NULL)  {
      System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
      t14Cleanup (tencap, PA_TEST_FAILED);
    }
    Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
    tencap->hd[i] = NULL;
  }

  /* Since the packets went to the modify PDSP and then back to the QM, a descriptor
   * and linked buffer was required while the packet was in the QM. This will
   * be recycled to the default recycle queue */
  while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
    Cppi_HostDesc *hdNext;
    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
    if (hd == NULL)  {
      System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
      break;
    }
    do {
      /* Goto the next descriptor. */
      hdNext = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc*)hd);
      testCommonRecycleLBDesc (tencap->tf, hd);
    } while((hd = hdNext) != NULL);
  }
}

static void t14DispAclStats(t14TestEncap_t *tencap)
{
    int i;
    paReturn_t paret;
    paAclStats_t aclStats;

    for (i = 0; i < T14_NUM_LOCAL_ACL_HANDLES; i++)  {

        if ( tencap->aclHandles[i].state == T14_HANDLE_ACTIVE)  {

        paret = Pa_queryAclStats(tencap->tf->passHandle, tencap->aclHandles[i].paHandle, 0,  &aclStats);

            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): Pa_queryAclStats returned error code %d\n", tfName, __FILE__, __LINE__, paret);
                continue;
            }

        System_printf ("ACL Stats (%d): nMatchPackets = %d,  nMatchBytes = %d\n", i, aclStats.nMatchPackets, aclStats.nMatchBytes);

        }
    }

    System_flush();
}

/* Check for pa lld return errors. Exit the test on any error condition */
static void t14HandleError (t14TestEncap_t *tencap, paReturn_t paret, Cppi_HostDesc *hd, int line)
{

    if (paret == pa_OK)
        return;

    System_printf ("%s (%s:%d): PA LLD returned error code %d\n", tfName, __FILE__, line, paret);

    if ((hd != NULL) && testCommonRecycleLBDesc (tencap->tf, hd))
        System_printf ("%s: (%s:%d): testCommonRecycleLBDesc failed to return a buffer/descriptor\n", tfName, __FILE__, __LINE__);

    t14Cleanup (tencap, PA_TEST_FAILED);  /* No return */

    /* Return */
    Task_exit();

}

static void t14CmdRep (t14TestEncap_t *tencap)
{
    Cppi_HostDesc    *hd;
    uint32_t         *swinfo;
    paReturn_t        paret;
    uint32_t                swinfoType;
    uint32_t                swinfoIdx;
    paEntryHandle_t   reth;
  int                     htype;
  int               cmdDest;
  char               *s;
  unsigned int       *stateP;
  unsigned int          stateV;
  int                       max;

   /* Wait for the command response */
    utilCycleDelay (20000);
    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_CMD_REPLY]) > 0)  {
        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_REPLY])) & ~15);

            Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swinfo);

            swinfoType = swinfo[0] & T14_CMD_SWINFO0_TYPE_MASK;
            swinfoIdx  = swinfo[0] & T14_CMD_SWINFO0_ID_MASK;

            paret = Pa_forwardResult (tencap->tf->passHandle, (void *)hd->buffPtr, &reth, &htype, &cmdDest);


            switch (swinfoType)  {

                case T14_CMD_SWINFO0_ADD_MAC_ID:
                    stateP = &tencap->l2Handles[swinfoIdx].state;
                    stateV = T14_HANDLE_ACTIVE;
                    max = T14_NUM_LOCAL_L2_HANDLES;
                    s = "pa_addMac";
                    break;

                case T14_CMD_SWINFO0_ADD_EOAM_ID:
                    stateP = &tencap->eoamHandles[swinfoIdx].state;
                    stateV = T14_HANDLE_ACTIVE;
                    max = T14_NUM_LOCAL_EOAM_HANDLES;
                    s = "pa_addEoamFlow";
                    break;

                case T14_CMD_SWINFO0_ADD_ACL_ID:
                    stateP = &tencap->aclHandles[swinfoIdx].state;
                    stateV = T14_HANDLE_ACTIVE;
                    max = T14_NUM_LOCAL_ACL_HANDLES;
                    s = "pa_addAcl";
                    break;

                case T14_CMD_SWINFO0_ADD_IP_ID:
                    stateP = &tencap->l3Handles[swinfoIdx].state;
                    stateV = T14_HANDLE_ACTIVE;
                    max = T14_NUM_LOCAL_L3_HANDLES;
                    s = "pa_addIp";
                    break;

                case T14_CMD_SWINFO0_ADD_PORT_ID:
                    stateP = &tencap->l4Handles[swinfoIdx].state;
                    stateV = T14_HANDLE_ACTIVE;
                    max = T14_NUM_LOCAL_L4_HANDLES;
                    s = "pa_addPort";
                    break;

                case T14_CMD_SWINFO0_DEL_MAC_ID:
                    stateP = &tencap->l2Handles[swinfoIdx].state;
                    stateV = T14_HANDLE_DISABLED;
                    max = T14_NUM_LOCAL_L2_HANDLES;
                    s = "pa_delMac";
                    break;

                case T14_CMD_SWINFO0_DEL_EOAM_ID:
                    stateP = &tencap->eoamHandles[swinfoIdx].state;
                    stateV = T14_HANDLE_DISABLED;
                    max = T14_NUM_LOCAL_EOAM_HANDLES;
                    s = "Pa_delEoamHandle";
                    break;

                case T14_CMD_SWINFO0_DEL_ACL_ID:
                    stateP = &tencap->aclHandles[swinfoIdx].state;
                    stateV = T14_HANDLE_ACTIVE;
                    max = T14_NUM_LOCAL_ACL_HANDLES;
                    s = "pa_delAcl";
                    break;

                case T14_CMD_SWINFO0_DEL_IP_ID:
                    stateP = &tencap->l3Handles[swinfoIdx].state;
                    stateV = T14_HANDLE_DISABLED;
                    max = T14_NUM_LOCAL_L3_HANDLES;
                    s = "pa_delIp";
                    break;

                case T14_CMD_SWINFO0_DEL_PORT_ID:
                    stateP = &tencap->l4Handles[swinfoIdx].state;
                    stateV = T14_HANDLE_DISABLED;
                    max = T14_NUM_LOCAL_L4_HANDLES;
                    s = "pa_delPort";
                    break;

          case T14_CMD_SWINFO0_GLOBAL_CFG_ID:
            stateP = &tencap->genCmdAck[swinfoIdx];
            stateV = TRUE;
            max    = T14_NUM_GEN_CONFIGS;
            s      = "Pa_control";
            break;

          case T14_CMD_SWINFO0_TIMEOFFSET_ID:
            stateP = &tencap->genCmdAck[swinfoIdx];
            stateV = TRUE;
            max    = T14_NUM_GEN_CONFIGS;
            s      = "Pa_control";
            break;

                default:
                    System_printf ("%s (%s:%d): Unknown command ID found in swinfo0 (0x%08x)\n", tfName, __FILE__, __LINE__, swinfo[0]);
                    t14Cleanup (tencap, PA_TEST_FAILED);
            /* Return */
            Task_exit();
            break;
            }

            /* In this test only valid responses are exected from PA */
            if (paret != pa_OK)  {
                System_printf ("%s (%s:%d): PA command %s returned error code %d, Index = %d\n", tfName, __FILE__, __LINE__, s, paret, swinfoIdx);
                t14Cleanup (tencap, PA_TEST_FAILED);
          /* Return */
          Task_exit();
            }

            if (swinfoIdx >= max)  {
                System_printf ("%s (%s:%d): Received command ack (%s) for out of range handle (%d, max = %d)\n",
                                tfName, __FILE__, __LINE__, s, swinfoIdx, max);
                t14Cleanup (tencap, PA_TEST_FAILED);
          /* Return */
          Task_exit();
            }

            /* Response looks valid. Change the internal handle state */
            *stateP = stateV;

        /* Return the descriptor used to received the reply from PA */
        testCommonRecycleLBDesc (tencap->tf, hd);
    }

    /* Recycle the command packet */
    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_CMD_RECYCLE]) > 0)  {
            hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
            if (hd == NULL)  {
                System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
                continue;
            }
            testCommonRecycleLBDesc (tencap->tf, hd);
    }
}

static paTestStatus_t t14NatTConfiguration (t14TestEncap_t *tencap, int enable)
{
	Cppi_HostDesc  *hd;
	paReturn_t      paret;
	int  			cmdDest;
	uint16_t			cmdSize;
  paCtrlInfo_t    ctrlInfo;

  /* Issue the command set command */
  ctrlInfo.code = pa_CONTROL_IPSEC_NAT_T_CONFIG;
  ctrlInfo.params.ipsecNatTDetCfg = t14NatTCfg;
  if (!enable) {
    ctrlInfo.params.ipsecNatTDetCfg.ctrlBitMap  &= ~pa_IPSEC_NAT_T_CTRL_ENABLE;
  }

  cmdReply.replyId  = T14_CMD_SWINFO0_NAT_T_CFG_ID;
	cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
 	hd = testCommonGlobalConfig (tencap->tf, &ctrlInfo,
 	                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
 	                             &cmdReply, &cmdDest, &cmdSize, &paret);

  if (hd == NULL)  {
    System_printf ("%s: (%s:%d): Failure in NAT-T Config command\n", tfName, __FILE__, __LINE__);
   	return (PA_TEST_FAILED);
  }

  /* Send command */
 	Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
  paTestL2ExpectedStats.classify1.nPackets += 1;

	if (testCommonWaitCmdReply (tencap->tf, tencap->pat, tfName, cmdReply.queue, T14_CMD_SWINFO0_NAT_T_CFG_ID, __LINE__)) {
		System_printf ("%s (%s:%d): testCommonGlobalConfig failed\n", tfName, __FILE__, __LINE__);
		return (PA_TEST_FAILED);
 	}

 	/* Recycle the command packet as well */
 	hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_CMD_RECYCLE])) & ~15);
 	if (hd == NULL)  {
 		System_printf ("%s (%s:%d): Did not find an expected command in the recycle queue\n", tfName, __FILE__, __LINE__);
 		return(PA_TEST_FAILED);
 	}
 	testCommonRecycleLBDesc (tencap->tf, hd);

	return (PA_TEST_PASSED);
}

static paTestStatus_t t14GlobalConfiguration (t14TestEncap_t *tencap, uint32_t confCode)
{
    int             i, validCode = 1;
    Cppi_HostDesc  *hd;
    paReturn_t      paret;
    int             cmdDest;
    uint16_t            cmdSize;
  paCtrlInfo_t    ctrlInfo;


  switch (confCode)
  {
    case T14_GLOBAL_SYS_CONFIG_USR_STATS:
      t14GlobalCfg.pUsrStatsConfig = &t14UsrStatsCfg;
      ctrlInfo.code              = pa_CONTROL_SYS_CONFIG;
      ctrlInfo.params.sysCfg     = t14GlobalCfg;
      cmdReply.replyId           = T14_CMD_SWINFO0_GLOBAL_CFG_ID;
      break;
    case T14_GLOBAL_SYS_CONFIG_EOAM_ENABLE:
      ctrlInfo.code              = pa_CONTROL_SYS_CONFIG;
      t14GlobalCfg.pUsrStatsConfig = NULL;
      ctrlInfo.params.sysCfg     = t14GlobalCfg;
      t14EoamGlobalCfg.enable    = TRUE;
      cmdReply.replyId           = T14_CMD_SWINFO0_GLOBAL_CFG_ID;
      break;

    case T14_EOAM_TIME_OFFSET_CONFIG:
      ctrlInfo.code               = pa_CONTROL_TIME_OFFSET_CONFIG;
      ctrlInfo.params.tOffsetCfg  = t14TimeOffsetCfg;
      cmdReply.replyId            = T14_CMD_SWINFO0_TIMEOFFSET_ID;
      break;

    case T14_GLOBAL_SYS_CONFIG_EOAM_DISABLE:
      ctrlInfo.code              = pa_CONTROL_SYS_CONFIG;
      t14GlobalCfg.pUsrStatsConfig = NULL;
      t14EoamGlobalCfg.enable    = FALSE;
      ctrlInfo.params.sysCfg     = t14GlobalCfg;
      cmdReply.replyId           = T14_CMD_SWINFO0_GLOBAL_CFG_ID;
      break;

    case T14_GLOBAL_SYS_CONFIG_PKTCTRL:
      ctrlInfo.code              = pa_CONTROL_SYS_CONFIG;
      ctrlInfo.params.sysCfg     = t14GlobalCfg2;
      cmdReply.replyId           = T14_CMD_SWINFO0_GLOBAL_CFG_ID;
      break;

    default:
      validCode = 0;
      break;
  }

   if (validCode == 0)
      return (PA_TEST_FAILED);

    cmdReply.queue             = tencap->tf->QGen[Q_CMD_REPLY];

    hd = testCommonGlobalConfig (tencap->tf, &ctrlInfo,
                                 tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf3,
                                 &cmdReply, &cmdDest, &cmdSize, &paret);

  if (hd == NULL)  {
     System_printf ("%s: (%s:%d): Failure in GlobalConfig command (PA Return code %d \n", tfName, __FILE__, __LINE__, paret);
     return (PA_TEST_FAILED);
  }

  /* Send command */
    Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

  tencap->genCmdAck[0] = FALSE;

    /* All the packets should have been acked */
    for (i = 0; i < 100; i++)  {
        t14CmdRep (tencap);

        if (tencap->genCmdAck[0])
            break;
        else
            utilCycleDelay (500);
    }

    if (i == 100)  {
        System_printf ("%s: (%s:%d): Pa_control commands was not acked\n", tfName, __FILE__, __LINE__);
        return (PA_TEST_FAILED);
    }
    return (PA_TEST_PASSED);
}

static paTestStatus_t t14OpenL2 (t14TestEncap_t *tencap, t14EthSetup_t *ethSetup, int nL2Handles)
{
    int             i, j;
    Cppi_HostDesc  *hd;
    paReturn_t      paret;
    int             cmdDest;
    uint16_t        cmdSize;
    uint32_t        ctrlBitMap = 0;

    for (i = 0; i < nL2Handles; i++)  {
      cmdReply.replyId = T14_CMD_SWINFO0_ADD_MAC_ID + i;
      cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

      if (i == T14_L2_CAP_HANDLE_INDEX) {
        nfailRoute_2.swInfo0 = T14_CMD_SWINFO0_PKT_ID + i;
        hd = testCommonAddMac2 (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&ethSetup[i].ethInfo,  &matchRoute2[T14_MAC_NEXT_ROUTE_L2CAP], &nfailRoute_2,
                              NULL, &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
                             &cmdReply, &cmdDest, &cmdSize, ctrlBitMap, &paret);
      }
      else {
        nfailRoute_2.swInfo0 = T14_CMD_SWINFO0_PKT_ID + i;
        hd = testCommonAddMac2 (tencap->tf, pa_LUT1_INDEX_NOT_SPECIFIED, (paEthInfo_t *)&ethSetup[i].ethInfo,  &matchRoute2[T14_MAC_NEXT_ROUTE_LUT1], &nfailRoute_2,
                                NULL, &tencap->l2Handles[i].paHandle, tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1,
                                &cmdReply, &cmdDest, &cmdSize, ctrlBitMap, &paret);
      }

      if (hd == NULL)  {
        System_printf ("%s: (%s:%d): Failure in common addMac command, entry number %d\n", tfName, __FILE__, __LINE__, i);
        t14Cleanup (tencap, PA_TEST_FAILED);
        /* Return */
        Task_exit();
      }

      Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
      tencap->l2Handles[i].state = T14_HANDLE_PENDING_ACK;
      paTestL2ExpectedStats.classify1.nPackets += 1;

      for (j = 0; j < 100; j++)  {
        utilCycleDelay (1000);
        t14CmdRep (tencap);
        if (tencap->l2Handles[i].state == T14_HANDLE_ACTIVE) {
          ethSetup[i].acked = TRUE;
          break;
        }
      }

      if (j == 100)  {
        System_printf ("%s: (%s:%d): pa_addMac command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
        return (PA_TEST_FAILED);
      }
    }
    return (PA_TEST_PASSED);
}

static paTestStatus_t t14OpenAcl (t14TestEncap_t  *tencap, t14AclPaSetup_t *aclSetup, int nACLHandles)
{
    int             i, j, m, n = nACLHandles;
    Cppi_HostDesc  *hd;
    paReturn_t      paret;
    int             cmdDest;
    uint16_t            cmdSize;
  t14Handles_t *linkedHandles   = tencap->l3Handles;
  t14HandlesAcl_t *nextHandles  = tencap->aclHandles;

    for (i = 0; i < n; i++)  {
    cmdReply.replyId = T14_CMD_SWINFO0_ADD_ACL_ID + i;
        cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

        hd = testCommonAddAcl (tencap->tf, aclSetup[i].aclInst, aclSetup[i].action, &aclSetup[i].aclInfo,
                               &tencap->aclHandles[aclSetup[i].handleIdx].paHandle,
                               (aclSetup[i].lHandleIdx != -1)?linkedHandles[aclSetup[i].lHandleIdx].paHandle:NULL,
                               (aclSetup[i].nHandleIdx != -1)?nextHandles[aclSetup[i].nHandleIdx].paHandle:NULL,
                               tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf2,
                               &cmdReply, &cmdDest, &cmdSize, &paret);

        if (hd == NULL)  {

            /* It's not a failure if the return code from pa was expected */
            if (paret == aclSetup[i].ret)  {
                aclSetup[i].acked = TRUE;
                continue;
            }

            System_printf ("%s: (%s:%d): Failure in common addAcl command, entry number %d\n", tfName, __FILE__, __LINE__, i);
            t14HandleError (tencap, paret, hd, __LINE__);
        }
        else if (paret == pa_DUP_ENTRY) {
            /* The ack will be handle for the first entry */
            aclSetup[i].acked = TRUE;
        }

        Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
        tencap->aclHandles[aclSetup[i].handleIdx].state = T14_HANDLE_PENDING_ACK;
        paTestL2ExpectedStats.classify1.nPackets += 1;

   {
      for (j = 0; j < 100; j++) {
        /* wait for previous entry to be completed */
        utilCycleDelay (1000);
        t14CmdRep (tencap);
        if (tencap->aclHandles[i].state == T14_HANDLE_ACTIVE) {
          aclSetup[i].acked = TRUE;
          break;
        }

        if (j == 100)  {
         System_printf ("%s: (%s:%d): addAcl commands for entry %d was acked \n", tfName, __FILE__, __LINE__, aclSetup[i].nHandleIdx);
         return (PA_TEST_FAILED);
        }
      }
      }
  }

    /* All the packets should have been acked */
    for (i = 0; i < 100; i++)  {
        t14CmdRep (tencap);

        for (j = m = 0; j < n; j++)  {
            if (aclSetup[j].acked == TRUE)
                m += 1;
        }

        if (m == n)
            break;
        else
            utilCycleDelay (1000);
    }

    if (i == 100)  {
        System_printf ("%s: (%s:%d): Command %d (out of %d) addAcl commands were acked \n", tfName, __FILE__, __LINE__, m, n);
        return (PA_TEST_FAILED);
    }

    return (PA_TEST_PASSED);

}

static paTestStatus_t t14OpenL3 (t14TestEncap_t *tencap, t14IpSetup_t *ipSetup, int nL3Handles)
{
    int             i, j, k = 0, l = 0;
    Cppi_HostDesc  *hd;
    paReturn_t      paret;
    int             cmdDest;
    uint16_t        cmdSize;
    uint32_t        ctrlBitMap = 0;
    paRouteInfo2_t  *nRoute;

    for (i = 0; i < nL3Handles; i++)  {
      cmdReply.replyId = T14_CMD_SWINFO0_ADD_IP_ID + i;
      cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
      if (ipSetup[i].lutInst == pa_LUT1_INST_1) {
        matchRoute2[ipSetup[i].nextRoute].swInfo0 = T14_CMD_SWINFO0_PKT_ID + k;
        nfailRoute_1.swInfo0 = T14_CMD_SWINFO0_PKT_ID1 + k;
        nRoute               = &nfailRoute_1;
        k++;
      }
      else {
        matchRoute2[ipSetup[i].nextRoute].swInfo0 = T14_CMD_SWINFO0_PKT_ID + l;
        nfailRoute_2.swInfo0 = T14_CMD_SWINFO0_PKT_ID2 + l;
        l++;
        nRoute = &nfailRoute_2;
      }

      hd = testCommonAddIp3 (tencap->tf,
                                ipSetup[i].lutInst, pa_LUT1_INDEX_NOT_SPECIFIED,
                                &ipSetup[i].ipInfo,
                                &matchRoute2[ipSetup[i].nextRoute],
                                nRoute,
                                &tencap->l3Handles[i].paHandle,
                                ipSetup[i].isPrevLinkL3?tencap->l3Handles[ipSetup[i].lHandleIdx].paHandle:tencap->l2Handles[ipSetup[i].lHandleIdx].paHandle,
                                NULL,
                                tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf2,
                                &cmdReply, &cmdDest, &cmdSize,ctrlBitMap, &paret);


      if (hd == NULL)  {
        System_printf ("%s: (%s:%d): Failure in common addIp command, entry number %d\n", tfName, __FILE__, __LINE__, i);
        t14Cleanup (tencap, PA_TEST_FAILED);
        /* Return */
        Task_exit();
      }

      Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
      tencap->l3Handles[i].state = T14_HANDLE_PENDING_ACK;
      paTestL2ExpectedStats.classify1.nPackets += 1;

      for (j = 0; j < 100; j++)  {
        utilCycleDelay (1000);
        t14CmdRep (tencap);
        if (tencap->l3Handles[i].state == T14_HANDLE_ACTIVE) {
          ipSetup[i].acked = TRUE;
          break;
        }
      }

      if (j == 100)  {
        System_printf ("%s: (%s:%d): pa_addIp command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
        return (PA_TEST_FAILED);
      }
    }

    return (PA_TEST_PASSED);
}

static paTestStatus_t t14OpenL4 (t14TestEncap_t *tencap, t14UdpSetup_t *udpSetup, int nL4Handles)
{
    int             i, j;
    Cppi_HostDesc  *hd;
    paReturn_t      paret;
    int             cmdDest;
    uint16_t            cmdSize;

    for (i = 0; i < nL4Handles; i++)  {
      cmdReply.replyId  = T14_CMD_SWINFO0_ADD_PORT_ID + i;
      cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];
      matchRoute2[T14_MAC_NEXT_ROUTE_HOST].swInfo0 = T14_CMD_SWINFO0_PKT_ID + i;
      nfailRoute_2.swInfo0 = T14_CMD_SWINFO0_PKT_ID + i;


      hd = testCommonAddPort3 (tencap->tf, pa_LUT2_PORT_SIZE_16, udpSetup[i].port, FALSE, pa_PARAMS_NOT_SPECIFIED, &matchRoute2[T14_UDP_NEXT_ROUTE_HOST],
                             &tencap->l4Handles[i].paHandle, &tencap->l3Handles[udpSetup[i].lHandleIdx].paHandle,
                             tencap->tf->QGen[Q_CMD_RECYCLE], tencap->tf->QLinkedBuf1, &cmdReply, &cmdDest, &cmdSize, &paret);


      if (hd == NULL)  {
        System_printf ("%s: (%s:%d): Failure in common addPort command, entry number %d\n", tfName, __FILE__, __LINE__, i);
        t14Cleanup (tencap, PA_TEST_FAILED);
        /* Return */
        Task_exit();
      }

      Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);
      paTestL2ExpectedStats.classify2.nPackets += 1;
      paTestL2ExpectedStats.classify1.nPackets += 1;
      tencap->l4Handles[i].state = T14_HANDLE_PENDING_ACK;

      for (j = 0; j < 100; j++)  {
        utilCycleDelay (1000);
        t14CmdRep (tencap);
        if (tencap->l4Handles[i].state == T14_HANDLE_ACTIVE) {
          udpSetup[i].acked = TRUE;
          break;
        }
      }

      if (j == 100)  {
        System_printf ("%s: (%s:%d): pa_addPort command (%d) were not acked\n", tfName, __FILE__, __LINE__, i);
        return (PA_TEST_FAILED);
      }
    }

    return (PA_TEST_PASSED);
}

static Cppi_HostDesc  *t14FormEoamPktDataDescr (t14TestEncap_t *tencap, eoamPktTestInfo_t *t14EoamPktInfo)
{
    Cppi_HostDesc  *hd;
    Qmss_Queue      q;
    pktTestInfo_t  *t14PktInfo = &t14EoamPktInfo->testInfo;

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);
    if (hd == NULL)  {
        System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
        return (NULL);
    }

    /* Recycle the free descriptors right back to the free descriptor queue */
    q.qMgr = 0;
    q.qNum = tencap->tf->QfreeDesc;

    /* Setup the return for the descriptor */
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

    /* Make sure there is no control info.  */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);

    /* Attach the data and set the length */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)t14PktInfo->pkt), t14PktInfo->pktLen);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t14PktInfo->pktLen);

    return (hd);
}

static Cppi_HostDesc  *t14FormPktDataDescr (t14TestEncap_t *tencap, pktTestInfo2_t *t14PktInfo2)
{
    Cppi_HostDesc  *hd;
    Qmss_Queue      q;
    pktTestInfo_t  *t14PktInfo = &t14PktInfo2->testInfo;

    hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);
    if (hd == NULL)  {
        System_printf ("%s (%s:%d): Failed to pop a descriptor off the free descriptor Q (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
        return (NULL);
    }

    /* Recycle the free descriptors right back to the free descriptor queue */
    q.qMgr = 0;
    q.qNum = tencap->tf->QfreeDesc;

    /* Setup the return for the descriptor */
    Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)hd, q);

    /* Make sure there is no control info.  */
    Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, 0);

    /* Attach the data and set the length */
    Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t *)utilgAddr((uint32_t)t14PktInfo->pkt), t14PktInfo->pktLen);
    Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)hd, t14PktInfo->pktLen);

    return (hd);
}

static paPatchCount_t t14pktPatchCount = {  /* The patch count command */

        0,      /* Start offset of count to be patched in L2 Header */
      0,        /* Counter index to be read */
    };

static paPatchTime_t t14pktPatchTime = {  /* The UDP checksum command */

        0,      /* Start offset of time to be patched in L2 Header */
    };

#define T14_CONVERT_SHIFT_VAL         20

static void test14ConvertTicksToTime( paInputFreq_e freq,
                                      paSetTimeOffset_t *tOffset,
                                      uint32_t msw,
                                      uint32_t lsw,
                                      uint32_t *seconds,
                                      uint32_t *nseconds )
{
  uint32_t mul = 0;
  uint32_t sec, nsec, tick32;
  uint64_t tick64;

  tick64  = (uint64_t) msw << 32;
  tick64 += (uint64_t) lsw;

  switch (freq)
  {
    case pa_INPUT_FREQ_1000MHZ:
        {
          mul = 6291456;
          sec = tick64/166666666;
          /* Get remaining ticks, which should fit in 32 bits */
          tick64 -= sec * 166666666;
          break;
        }

    case pa_INPUT_FREQ_1050MHZ:
      {
        mul = 5991863;
        sec = tick64/175000000;
        /* Get remaining ticks, which should fit in 32 bits */
        tick64 -= sec * 175000000;
        break;
      }

    case pa_INPUT_FREQ_1049p6MHZ:
      {
        mul = 5994146;
        sec = tick64/174933333;
        /* Get remaining ticks, which should fit in 32 bits */
        tick64 -= sec * 174933333;
        break;
      }

    default:
      break;
  }

  tick32  = tick64 & 0xFFFFFFFFL;

  nsec    = ( ( (uint64_t)tick32 * mul ) >> T14_CONVERT_SHIFT_VAL );

  /* Add the time offsets */
  *seconds  = sec   + tOffset->offset_sec;
  *nseconds = nsec  + tOffset->offset_ns;

  return;
}

static void t14SendRxEgressDataPkts (t14TestEncap_t *tencap, int n, int group, int fFirst)
{
    Cppi_HostDesc        *hd[2];
    Qmss_Queue            q;
    int                         i;

#ifdef T14_DEBUG_TXCMD
  int flag;
#endif
    uint16_t                cmdStackSize;
    paReturn_t            paret;
  paCmdInfo_t         cmdInfo[10];
  Ptr                 cmdBuf;
    pasahoLongInfo_t   *pinfo;
    uint32_t                infoLen;

    /* Attach one free descriptor to each of the packets */
    for (i = 0; i < n; i++)  {
            tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QfreeDesc)) & ~15);

            if (tencap->hd[i] == NULL)  {
                System_printf ("%s (%s:%d): Failed to pop a descriptor from the free queue (%d)\n", tfName, __FILE__, __LINE__, tencap->tf->QfreeDesc);
                t14Cleanup (tencap, PA_TEST_FAILED);
            }

        t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[T14_USR_STATS_ID_TX_PKTS].cntIndex, 1);
        t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[T14_USR_STATS_ID_TX_BYTES].cntIndex,
                        (uint32_t)t14EoamEgressPktTestInfo[i].pktTestInfo.pktLen);

        /* Update expected User Stats */
        if (t14EoamEgressPktTestInfo[i].ctrlBitMap & T14_EOAM_NEED_RECORDS)
          t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[t14EoamEgressPktTestInfo[i].eoamFlowNum].cntIndex, 1);


          /* Setup the return for the descriptor */
            q.qMgr = 0;
        q.qNum = tencap->tf->QGen[Q_DPKT_RECYCLE];
            Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), q);


          Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)utilgAddr((uint32_t)t14EoamEgressPktTestInfo[i].pktTestInfo.pkt), (uint32_t)(t14EoamEgressPktTestInfo[i].pktTestInfo.pktLen));
          Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint32_t)t14EoamEgressPktTestInfo[i].pktTestInfo.pktLen);

        if (t14EoamEgressPktTestInfo[i].isPatchCount)
        {
          /* Update Patch Count offset  */
              t14pktPatchCount.startOffset = t14EoamEgressPktTestInfo[i].startOffset;
          t14pktPatchCount.countIndex  = t14UsrStatsTbl[t14EoamEgressPktTestInfo[i].eoamFlowNum].cntIndex;
          cmdBuf = (Ptr)&tencap->cmdStack[i];
          cmdStackSize = sizeof(tencap->cmdStack[i]);
          /* Patch Count */
          cmdInfo[0].cmd = pa_CMD_PATCH_COUNT;
          cmdInfo[0].params.patchCount = t14pktPatchCount;
          cmdInfo[1].cmd = pa_CMD_NEXT_ROUTE;
          t14pktRoute1.queue = t14Encap.tf->QPaTx[TF_PA_Q_INPUT]; // tencap->tf->QGen[Q_PATCH_COUNT];
          cmdInfo[1].params.route = t14pktRoute1;
        }
        else
        {
          /* Update Patch Count offset  */
              t14pktPatchTime.startOffset = t14EoamEgressPktTestInfo[i].startOffset;
          cmdBuf = (Ptr)&tencap->cmdStack[i];
          cmdStackSize = sizeof(tencap->cmdStack[i]);
          /* Patch Time */
          cmdInfo[0].cmd = pa_CMD_PATCH_TIME;
          cmdInfo[0].params.patchTime = t14pktPatchTime;
          cmdInfo[1].cmd = pa_CMD_NEXT_ROUTE;
          t14pktRoute1.queue = t14Encap.tf->QPaTx[TF_PA_Q_INPUT]; // tencap->tf->QGen[Q_PATCH_TIME];
          cmdInfo[1].params.route = t14pktRoute1;
        }

          paret = Pa_formatTxCmd (  2,
                                    cmdInfo,
                                    0,
                                    cmdBuf,    /* Command buffer       */
                                    &cmdStackSize);               /* Command size         */

        if (paret != pa_OK)  {
            System_printf ("%s (%s:%d): Pa_formatTxCmd returned error code %d\n", tfName, __FILE__, __LINE__, paret);
            t14Cleanup (tencap, PA_TEST_FAILED);
        }

          Cppi_setPSData (Cppi_DescType_HOST, (Cppi_Desc *)(tencap->hd[i]), (uint8_t *)cmdBuf, cmdStackSize);

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X

        /*
        * Write back the entire cache to make sure that the test packets are updated.
        * Note: It may be more efficient to call CACHE_wbL1d(blockPtr, byteCnt, wait) only for
        *       the portion of packet which is updated.
        *
        */
        CACHE_wbAllL1d(CACHE_WAIT);
        //CACHE_wbAllL2(CACHE_WAIT);
#endif
#endif

            /* Send the data to the modify PDSP */
            //mdebugHaltPdsp(5);
            Qmss_queuePush (tencap->tf->QPaTx[TF_PA_Q_TXCMD], (Ptr)tencap->hd[i], (uint32_t)t14EoamEgressPktTestInfo[i].pktTestInfo.pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);

            //while (mdebugWait);
#ifdef T14_DEBUG_TXCMD
            /* Wait for the packet to arrive */
            flag = 0;
            do {
              if (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_PATCH_COUNT]))
                 flag = 1;
              else if(Qmss_getQueueEntryCount(tencap->tf->QGen[Q_PATCH_TIME]))
                 flag = 1;
            }  while (!flag);

#endif
         testCommonIncStats (t14EoamEgressPktTestInfo[i].pktTestInfo.statsMap, &paTestL2ExpectedStats);
#ifdef T14_TEST_HUGE_TIME_COUNTS
       utilCycleDelay (20000 * 100000);
#else
       utilCycleDelay (20000);
#endif

        /* This should result in patched packet in either Count Queue or Patch Time Queue */
        if (t14EoamEgressPktTestInfo[i].isPatchCount)
        {
          /* Look out for patched packet under Patch Count Queue */
            while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_MATCH_EOAM_PKT]) > 0)  {

                  uint32_t count_r;
                  int      index;
                  uint8_t  *pkt;

                  hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH_EOAM_PKT])) & ~15);
                  if (hd[0] == NULL)  {
                    System_printf ("%s (%s:%d): Failed to pop a received Patch Count packet\n", tfName, __FILE__, __LINE__);
                    System_flush();
                    return;
                  }

                  /* Read the Count Information from the packet and print it */
                  index = t14pktPatchCount.startOffset;
                  pkt   = (uint8_t *) hd[0]->buffPtr;
                  count_r = ( pkt[index + 0] << 24 ) |
                            ( pkt[index + 1] << 16 ) |
                            ( pkt[index + 2] <<  8 ) |
                            ( pkt[index + 3] <<  0 );

                  System_printf("The count patched in the packet is %d \n", count_r);
                  /* Print the Count information read from the packet offset specified */
                  testCommonRecycleLBDesc (tencap->tf, hd[0]);
                  utilCycleDelay (10000);
            }
        }
        else /* otherwise in patch time queue */
        {
            uint32_t paTimeStamp_lsw, paTimeStamp_msw;
            uint32_t sec, nsec;

          /* Look out for patched packet under Patch Time Queue */
            while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_MATCH_EOAM_PKT]) > 0)  {
                  uint32_t time_sec,  time_nsec;
                  int      index_sec, index_nsec;
                  uint8_t  *pkt;

                  hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH_EOAM_PKT])) & ~15);
                  if (hd[0] == NULL)  {
                    System_printf ("%s (%s:%d): Failed to pop a received Patch Time packet\n", tfName, __FILE__, __LINE__);
                    System_flush();
                    return;
                  }

                  pkt   = (uint8_t *) hd[0]->buffPtr;

                  index_sec   =  t14pktPatchTime.startOffset + 0;
                  index_nsec  =  t14pktPatchTime.startOffset + 4;

                  time_sec = ( pkt[index_sec + 0] << 24 ) |
                             ( pkt[index_sec + 1] << 16 ) |
                             ( pkt[index_sec + 2] <<  8 ) |
                             ( pkt[index_sec + 3] <<  0 );

                  time_nsec  = ( pkt[index_nsec + 0] << 24 ) |
                               ( pkt[index_nsec + 1] << 16 ) |
                               ( pkt[index_nsec + 2] <<  8 ) |
                               ( pkt[index_nsec + 3] <<  0 );

#ifdef T14_DEBUG_TXCMD
                  /* Now Send the same packet and capture the received PA time stamp to display */
                  Qmss_queuePush (t14Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd[0], t14EoamEgressPktTestInfo[i].pktTestInfo.pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
                  testCommonIncStats (t14EoamEgressPktTestInfo[i].pktTestInfo.statsMap, &paTestL2ExpectedStats);

                  /* Print the Seconds and Nano seconds time that is patched in the packet information */
                  System_printf("The time patched in the packet is sec = 0x%08x, nano seconds = 0x%08x \n", time_sec, time_nsec);

                  while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_MATCH_EOAM_PKT]) > 0)  {
                    hd[1] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH_EOAM_PKT])) & ~15);
                    if (hd[1] == NULL)  {
                      System_printf ("%s (%s:%d): Failed to pop a descriptor off the receive packet queue\n", tfName, __FILE__, __LINE__);
                      testCommonRecycleLBDesc (tencap->tf, hd[1]);
                      return;
                    }

                    /* Verify the parse information is correct */
                    if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd[1], (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
                      System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
                      testCommonRecycleLBDesc (tencap->tf, hd[1]);
                      return;
                    }

                    /* Now read the PA Time stamp from the descriptor and print it */
                    paTimeStamp_lsw = PASAHO_LINFO_READ_PATSTAMP_LSW_GEN2(pinfo);
                    paTimeStamp_msw = PASAHO_LINFO_READ_PATSTAMP_MSW_GEN2(pinfo);

                    /* Converted to Seconds and nano seconds from Rx PA Time Stamp) */
                    test14ConvertTicksToTime(t14EoamGlobalCfg.freq, &t14TimeOffsetCfg, paTimeStamp_msw, paTimeStamp_lsw, &sec, &nsec);
                    System_printf ("From PA TimeStamp: Ticks = (MSW) = 0x%08x, (LSW) = 0x%08x, Sec = 0x%08x, Nano Seconds  = 0x%08x \n", paTimeStamp_msw, paTimeStamp_lsw, sec, nsec);
                    System_flush();

                    testCommonRecycleLBDesc (tencap->tf, hd[1]);
                    utilCycleDelay (10000);
                  }
#endif

                  /* Verify the parse information is correct */
                  if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd[0], (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
                    System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
                    testCommonRecycleLBDesc (tencap->tf, hd[1]);
                    return;
                  }

                  /* Print the Seconds and Nano seconds time that is patched in the packet information */
                  System_printf("The time patched in the packet is sec = 0x%08x, nano seconds = 0x%08x \n", time_sec, time_nsec);

                  /* Now read the PA Time stamp from the descriptor and print it */
                  paTimeStamp_lsw = PASAHO_LINFO_READ_PATSTAMP_LSW_GEN2(pinfo);
                  paTimeStamp_msw = PASAHO_LINFO_READ_PATSTAMP_MSW_GEN2(pinfo);

                  /* Converted to Seconds and nano seconds from Rx PA Time Stamp) */
                  test14ConvertTicksToTime(t14EoamGlobalCfg.freq, &t14TimeOffsetCfg, paTimeStamp_msw, paTimeStamp_lsw, &sec, &nsec);
                  System_printf ("From PA TimeStamp: Ticks = (MSW) = 0x%08x, (LSW) = 0x%08x, Sec = 0x%08x, Nano Seconds  = 0x%08x \n", paTimeStamp_msw, paTimeStamp_lsw, sec, nsec);
                  System_flush();

                  /* Print the Time information read from the packet offset specified */
                  testCommonRecycleLBDesc (tencap->tf, hd[0]);
                  utilCycleDelay (10000);

            }
        }
    }

   /* Wait for descriptors to return. It is assumed that they are returning in order. */
   for (i = 0; i < 100; i++)  {

    if ((Qmss_getQueueEntryCount (tencap->tf->QGen[Q_DPKT_RECYCLE])) >= n)
        break;

     utilCycleDelay (500);
   }

   if (i == 100)  {
        System_printf ("%s (%s:%d): Timeout waiting for descriptors to modify PDSP to be recycled\n", tfName, __FILE__, __LINE__);
        t14Cleanup (tencap, PA_TEST_FAILED);
   }

   /* Recycle the descriptors */
   for (i = 0; i < n; i++) {
        tencap->hd[i] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_DPKT_RECYCLE]) & ~15));
        if (tencap->hd[i] == NULL)  {
            System_printf ("%s (%s:%d): Failed to pop recycled data packet descriptor\n", tfName, __FILE__, __LINE__);
            t14Cleanup (tencap, PA_TEST_FAILED);
        }

        Qmss_queuePushDesc (tencap->tf->QfreeDesc, (Ptr) tencap->hd[i]);
        tencap->hd[i] = NULL;

   }

    /* Since the packets went to the modify PDSP and then back to the QM, a descriptor
     * and linked buffer was required while the packet was in the QM. This will
     * be recycled to the default recycle queue */
    while (Qmss_getQueueEntryCount(tencap->tf->QDefRet) > 0)  {
        Cppi_HostDesc *hdNext;
        hd[0] = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QDefRet)) & ~15);
        if (hd[0] == NULL)  {
            System_printf ("%s (%s:%d): Could not pop descriptor from default recycle queue\n", tfName, __FILE__, __LINE__);
            break;
        }

        do
        {
            /* Goto the next descriptor. */
            hdNext = (Cppi_HostDesc*)Cppi_getNextBD(Cppi_DescType_HOST, (Cppi_Desc*)hd[0]);
                testCommonRecycleLBDesc (tencap->tf, hd[0]);
        } while((hd[0] = hdNext) != NULL);
    }

}

int  t14SendEoamPkts (eoamPktTestInfo_t *eoam, char *tfName)
{
    int  idx, expCount = 0;
    Cppi_HostDesc  *hd;

    /* Send packets for each EOAM group */
    while (eoam->testInfo.pkt != NULL) {
      hd = t14FormEoamPktDataDescr (&t14Encap, eoam);
      if (hd == NULL)  {
        System_printf ("%s (%s:%d): Failed to get free descriptor\n", tfName, __FILE__, __LINE__);
        t14Cleanup (&t14Encap, PA_TEST_FAILED);
        /* Return */
        Task_exit();
      }
      /* Push the EOAM test packet */
      Qmss_queuePush (t14Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd, eoam->testInfo.pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);

      /* Update expectations */
      testCommonIncStats (eoam->testInfo.statsMap, &paTestL2ExpectedStats);
      if (!(eoam->ctrlBitMap & T14_PKT_DISCARD ) ) {
         /* It is expected to arrive at either eoam match queue or next fail queue */
          expCount++;
      }

      if (eoam->ctrlBitMap & T14_EOAM_NEED_RECORDS) {
        idx = eoam->eoamFlowNum;
        t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[idx].cntIndex, 1);
      }

      /* Dealy to allow packets go through the PASS */
      utilCycleDelay (1000);
      eoam->ctrlBitMap |= T14_PKT_SENT;
      /* Next EOAM group */
      eoam++;
    }

    return (expCount);
}

int t14GetExpectedPktInfo( eoamPktTestInfo_t* eoam, int pktId)
{
   int index = 0;

   while (eoam->testInfo.pkt != NULL) {

      if (eoam->testInfo.idx == pktId)
        break;

      /* point to next element */
      eoam ++;

      /* Record the index */
      index ++;
   }

   return (index);
}

int t14GetExpectedIpPktInfo( pktTestInfo2_t* eoam, int pktId)
{
   int index = 0;

   while (eoam->testInfo.pkt != NULL) {

      if (eoam->testInfo.idx == pktId)
        break;

      /* point to next element */
      eoam ++;

      /* Record the index */
      index ++;
   }

   return (index);
}

/* Look for Eoam packets in the receive queue, verify the match */
int t14RxEoamPkts (t14TestEncap_t *tencap, eoamPktTestInfo_t* eoam, int expCount)
{
    int               index;
    uint8_t          *pkt;
    Cppi_HostDesc    *hd;
    pasahoLongInfo_t *pinfo;
    uint32_t          infoLen;
    uint32_t         *swInfo;
    int               pktId = 0;
    int               count = 0;
    unsigned int      idx, i;

    for ( i = 0; i < 100; i++) {
      utilCycleDelay(1000);

      /* Look out for EOAM known packet Queues */
      while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_MATCH_EOAM_PKT]) > 0)  {

        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH_EOAM_PKT])) & ~15);
        if (hd == NULL)  {
          System_printf ("%s (%s:%d): Failed to pop a descriptor off the receive packet queue\n", tfName, __FILE__, __LINE__);
          testCommonRecycleLBDesc (tencap->tf, hd);
          return (-1);
        }

        /* Verify the parse information is correct */
        if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
          System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
          testCommonRecycleLBDesc (tencap->tf, hd);
          return (-1);
        }

        /* Verify swInfo0 for packet match and packet ID number */
        Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

        pktId = *swInfo & T14_CMD_SWINFO0_TYPE_MASK;

        /* Allow EOAM control or data packets */
        if (pktId != T14_CMD_SWINFO0_EOAM_PKT_ID)
        {
          System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
                   tfName, __FILE__, __LINE__, *swInfo);
          testCommonRecycleLBDesc (tencap->tf, hd);
          return (-1);
        }

        idx = *swInfo & T14_CMD_SWINFO0_ID_MASK;
        pkt = (uint8_t* ) hd->buffPtr;
        pktId = (idx * T14_MAX_PKTS_PER_ID) + T14_EOAM_MATCH_BASE;
        idx   = pktId + pkt[T14_EOAM_PACKET_INDEX_OFFSET];
        /* search the packet based on ID */
        index = t14GetExpectedPktInfo(eoam,  idx );
        if (testCommonComparePktInfoEoamMode (tfName, eoam[index].testInfo.info, pinfo, TRUE))  {
          testCommonRecycleLBDesc (tencap->tf, hd);
          return (-1);
        }

        testCommonRecycleLBDesc (tencap->tf, hd);
        eoam[index].ctrlBitMap |= T14_PKT_RECEIVED;
        count ++;
      }

      /* Look out for EOAM known packet Queues */
      while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_NFAIL]) > 0)  {

        hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_NFAIL])) & ~15);
        if (hd == NULL)  {
          System_printf ("%s (%s:%d): Failed to pop a descriptor off the receive packet queue\n", tfName, __FILE__, __LINE__);
          testCommonRecycleLBDesc (tencap->tf, hd);
          return (-1);
        }

        /* Verify the parse information is correct */
        if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
          System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
          testCommonRecycleLBDesc (tencap->tf, hd);
          return (-1);
        }

        /* Verify swInfo0 for packet match and packet ID number */
        Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

        pktId = *swInfo & T14_CMD_SWINFO0_TYPE_MASK;

        /* Allow EOAM control or data packets */
        if (pktId != T14_CMD_SWINFO0_PKT_ID)
        {
          System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
                   tfName, __FILE__, __LINE__, *swInfo);
          testCommonRecycleLBDesc (tencap->tf, hd);
          return (-1);
        }

        idx = *swInfo & T14_CMD_SWINFO0_ID_MASK;
        pkt = (uint8_t* ) hd->buffPtr;
        pktId = (idx * T14_MAX_PKTS_PER_ID) + T14_L2_NFAIL_BASE;
        idx   = pktId + pkt[T14_EOAM_PACKET_INDEX_OFFSET];
        index = t14GetExpectedPktInfo(eoam, idx );
        if (testCommonComparePktInfoEoamMode (tfName, eoam[index].testInfo.info, pinfo, FALSE))  {
          testCommonRecycleLBDesc (tencap->tf, hd);
          return (-1);
        }

        testCommonRecycleLBDesc (tencap->tf, hd);
        eoam[index].ctrlBitMap |= T14_PKT_RECEIVED;
        count ++;
      }

      if (count >= expCount)
        break;
    }

    if (i == 100)  {
      System_printf ("%s (%s:%d): Error - unable to recover all packets\n", tfName, __FILE__, __LINE__);
      System_flush();
      return (-1);
    }
    return(0);
}

int t14SendPkts (pktTestInfo2_t *testPkt, char *tfName)
{
    int  idx, expCount = 0;
    Cppi_HostDesc  *hd;

    /* Send packets for IP end */
    while (testPkt->testInfo.pkt != NULL) {
      hd = t14FormPktDataDescr (&t14Encap, testPkt);
      if (hd == NULL)  {
        System_printf ("%s (%s:%d): Failed to get free descriptor\n", tfName, __FILE__, __LINE__);
        t14Cleanup (&t14Encap, PA_TEST_FAILED);
        /* Return */
        Task_exit();
      }
      /* Push the test packet */
      Qmss_queuePush (t14Encap.tf->QPaTx[TF_PA_Q_INPUT], (Ptr)hd, testPkt->testInfo.pktLen, TF_SIZE_DESC, Qmss_Location_TAIL);
      testPkt->ctrlBitMap |= T14_PKT_SENT;

      /* Update expectations */
      testCommonIncStats (testPkt->testInfo.statsMap, &paTestL2ExpectedStats);
      testCommonIncStats (testPkt->statsMap, &paTestL2ExpectedStats);
      if (!(testPkt->ctrlBitMap & T14_PKT_DISCARD )) {
         /* It is expected to arrive at either eoam match queue or next fail queue */
          expCount++;
      }

      if (testPkt->ctrlBitMap & T14_EOAM_NEED_RECORDS) {
        idx = testPkt->eoamFlowNum;
        t14UpdateUsrStats(&paTestExpectedUsrStats, t14UsrStatsTbl[idx].cntIndex, 1);
      }

      /* Dealy to allow packets go through the PASS */
      utilCycleDelay (2000);
      /* Next test packet */
      testPkt++;
    }

    return (expCount);
}

int t14RxPkts (t14TestEncap_t *tencap, pktTestInfo2_t* testInfo, int expCount)
{
  int               index;
  uint8_t          *pkt;
  Cppi_HostDesc    *hd;
  pasahoLongInfo_t *pinfo;
  uint32_t          infoLen;
  uint32_t         *swInfo;
  int               pktId = 0;
  int               i, count = 0;
  unsigned int      idx;

  for ( i = 0; i < 100; i++) {
    utilCycleDelay(1000);
    /* Look for ESP packets: Update the startOffset and endOffset,
     * Push to Ingress3: for nextHdr is IP
     *         Ingress4: Otherwise
     *
     * Note: Fixed ivSize (8) and icvSize (12) are used since it is just a proof-of-concept test.
     *       Those two variables may be added to the packet packet information in enhanced test cases
     */
    while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_ESP_FIX]) > 0)  {
      uint8_t  startOffset;
      uint16_t endOffset;

      hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_ESP_FIX])) & ~15);
      if (hd == NULL)  {
        System_printf ("%s (%s:%d): Failed to pop a received ESP packet\n", tfName, __FILE__, __LINE__);
        return (-1);
      }

      /* Verify the parse information is correct */
      if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
        System_printf ("%s (%s:%d): Error getting control info from received ESP packet\n", tfName, __FILE__, __LINE__);
        testCommonRecycleLBDesc (tencap->tf, hd);
        return (-1);
      }

      startOffset = PASAHO_LINFO_READ_START_OFFSET(pinfo) + 8;
      PASAHO_LINFO_SET_START_OFFSET(pinfo, startOffset);

      endOffset = PASAHO_LINFO_READ_END_OFFSET(pinfo) - 12;
      PASAHO_LINFO_SET_END_OFFSET(pinfo, endOffset);

      PASAHO_LINFO_SET_NXT_HDR_TYPE(pinfo, PASAHO_HDR_ESP_DECODED);

      while (endOffset > hd->buffLen)  {
        endOffset -= hd->buffLen;
        hd = (Cppi_HostDesc *)hd->nextBDPtr;
      }

      Qmss_queuePushDescSize (tencap->tf->QPaTx[TF_PA_Q_FIREWALL2], (Ptr)hd, TF_SIZE_DESC);
      utilCycleDelay (10000);
    }

    /*
     * Look for L2 Capture packets: Examine and recycle
     */
	  while (Qmss_getQueueEntryCount(tencap->tf->QGen[Q_L2_CAPTURE]) > 0)  {
      hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_L2_CAPTURE])) & ~15);
		  if (hd == NULL)  {
		    System_printf ("%s (%s:%d): Failed to pop a received L2 Capture packet\n", tfName, __FILE__, __LINE__);
		 	  return (-1);
		  }
  	  testCommonRecycleLBDesc (tencap->tf, hd);
      // Since this is a copy packet, no need to increment the count, the count gets incremented in later receive types
      // count ++;
	  }

    /* Look out for match IP packet Queues */
    while (Qmss_getQueueEntryCount (tencap->tf->QGen[Q_MATCH]) > 0)  {

      hd = (Cppi_HostDesc *)(((uint32_t)Qmss_queuePop (tencap->tf->QGen[Q_MATCH])) & ~15);
      if (hd == NULL)  {
        System_printf ("%s (%s:%d): Failed to pop a descriptor off the receive packet queue\n", tfName, __FILE__, __LINE__);
        testCommonRecycleLBDesc (tencap->tf, hd);
        return (-1);
      }

      /* Verify the parse information is correct */
      if (Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc *)hd, (uint8_t **)&pinfo, &infoLen) != CPPI_SOK)  {
        System_printf ("%s (%s:%d): Error getting control info from received data packet\n", tfName, __FILE__, __LINE__);
        testCommonRecycleLBDesc (tencap->tf, hd);
        return (-1);
      }

      /* Verify swInfo0 for packet match and packet ID number */
      Cppi_getSoftwareInfo (Cppi_DescType_HOST, (Cppi_Desc *)hd, (uint8_t **)&swInfo);

      pktId = *swInfo & T14_CMD_SWINFO0_TYPE_MASK;

      /* Allow EOAM control or data packets */
      if ( pktId != T14_CMD_SWINFO0_PKT_ID )
      {
        System_printf ("%s (%s:%d): Found a packet in the receive packet queue but with incorrect swinfo0 Id: 0x%08x\n",
                 tfName, __FILE__, __LINE__, *swInfo);
        testCommonRecycleLBDesc (tencap->tf, hd);
        return (-1);
      }

      idx = *swInfo & T14_CMD_SWINFO0_ID_MASK;
      if (idx > T14_NUM_L4_ENTRIES)
        return (-1);

      pkt = (uint8_t* ) hd->buffPtr;
      pktId = (idx * T14_MAX_PKTS_PER_ID) + T14_L4_MATCH_BASE;

      idx   = pktId + pkt[T14_IP_PACKET_INDEX_OFFSET];
      /* search the packet based on ID */
      index = t14GetExpectedIpPktInfo(testInfo,  idx );
      if (testCommonComparePktInfo (tfName, testInfo[index].testInfo.info, pinfo))  {
        testCommonRecycleLBDesc (tencap->tf, hd);
        return (-1);
      }

      testCommonRecycleLBDesc (tencap->tf, hd);
      testInfo[index].ctrlBitMap |= T14_PKT_RECEIVED;
      count ++;
    }
    /* Check if we received everything */
    if(count >= expCount)
      break;
  }

	if (i == 100)  {
		System_printf ("%s (%s:%d): Error - unable to recover all packets\n", tfName, __FILE__, __LINE__);
    System_flush();
		return (-1);
	}

  return(0);
}

static paTestStatus_t t14OpenEoam (t14TestEncap_t  *tencap, t14EoamPaSetup_t *eoamSetup, int n, char *id)
{
    int             i, j, m;
    Cppi_HostDesc  *hd;
    paReturn_t      paret;
    int             cmdDest;
    uint16_t            cmdSize;

    for (i = 0; i < n; i++)  {
    cmdReply.replyId = T14_CMD_SWINFO0_ADD_EOAM_ID + i;
    cmdReply.queue    = tencap->tf->QGen[Q_CMD_REPLY];

    eoamSetup[i].eoamInfo.destQueue  = tencap->tf->QGen[Q_MATCH_EOAM_PKT];
    eoamSetup[i].eoamInfo.flowId     = tencap->tf->tfFlowNum[0];
    eoamSetup[i].eoamInfo.statsIndex = t14UsrStatsTbl[i].cntIndex;

    hd = testCommonAddEoam ( tencap->tf,
                            &eoamSetup[i].ethInfo,
                            &eoamSetup[i].eoamInfo,
                            &tencap->eoamHandles[i].paHandle,
                            tencap->tf->QGen[Q_CMD_RECYCLE],
                            tencap->tf->QLinkedBuf2,
                            &cmdReply,
                            &cmdDest,
                            &cmdSize,
                            &paret);

     if (hd == NULL)  {
      /* It's not a failure if the return code from pa was expected */
      if (paret == eoamSetup[i].ret)  {
        eoamSetup[i].acked = TRUE;
        continue;
      }

      System_printf ("%s: (%s:%d): Failure in common addEoamFlow command, %s entry number %d\n", tfName, __FILE__, __LINE__, id, i);
      t14HandleError (tencap, paret, hd, __LINE__);
    }
    else if (paret == pa_DUP_ENTRY) {
      /* The ack will be handle for the first entry */
      eoamSetup[i].acked = TRUE;
    }

    Qmss_queuePush (tencap->tf->QPaTx[cmdDest - pa_CMD_TX_DEST_0 + TF_PA_Q_CONFIG_BASE], (Ptr)hd, cmdSize, TF_SIZE_DESC, Qmss_Location_TAIL);

    tencap->eoamHandles[i].state = T14_HANDLE_PENDING_ACK;
    paTestL2ExpectedStats.classify1.nPackets += 1;

    t14CmdRep (tencap);
    if (tencap->eoamHandles[i].state == T14_HANDLE_ACTIVE)
      eoamSetup[i].acked = TRUE;
    }

    /* All the packets should have been acked */
    for (i = 0; i < 100; i++)  {
      for (j = m = 0; j < n; j++)  {
        if (eoamSetup[j].acked == TRUE)
          m += 1;
      }
      if (m == n) {
        break;
      }
      else {
        utilCycleDelay (1000);
      }
    }

    if (i == 100)  {
        System_printf ("%s: (%s:%d): Command %d (out of %d) addEoamFlow commands were acked (%s)\n", tfName, __FILE__, __LINE__, m, n, id);
        System_flush();
        return (PA_TEST_FAILED);
    }

    return (PA_TEST_PASSED);

}

#define T14_DEBUG_CODE 0

#if     T14_DEBUG_CODE
#define T14_MAX_TIME_ACC   1000
volatile uint32_t t14TimeAccNSecErr[T14_MAX_TIME_ACC];
volatile uint32_t t14TimeAccNSec[T14_MAX_TIME_ACC];
volatile uint32_t t14RoLSW[T14_MAX_TIME_ACC];
volatile uint32_t t14RoMSW[T14_MAX_TIME_ACC];
volatile uint32_t *accMem = (uint32_t *) 0x260200D0;
volatile uint32_t *RoMem  = (uint32_t *) 0x260200F0;
#endif

#ifdef __LINUX_USER_SPACE
void* paTestEOAMFlow (void *args)
{
    void  *a0 = (void *)((paTestArgs_t *)args)->tf;
    void  *a1 = (void *)((paTestArgs_t *)args)->pat;
#else
void paTestEOAMFlow (UArg a0, UArg a1)
{
#endif

  int             i, n, k, expCount;
  paTestStatus_t  newStatus;
  volatile int mdebugWait = 1;
  paIPReassemblyConfig_t ipReassemCfg;

  t14Encap.tf  = (tFramework_t *)a0;
  t14Encap.pat = (paTest_t *)a1;

  for (i = 0; i < T14_NUM_LOCAL_L4_HANDLES; i++)
    t14Encap.l4Handles[i].state = T14_HANDLE_UNCONFIGURED;
  for (i = 0; i < T14_NUM_LOCAL_L3_HANDLES; i++)
    t14Encap.l3Handles[i].state = T14_HANDLE_UNCONFIGURED;
  for (i = 0; i < T14_NUM_LOCAL_L2_HANDLES; i++)
    t14Encap.l2Handles[i].state = T14_HANDLE_UNCONFIGURED;
  for (i = 0; i < T14_NUM_LOCAL_EOAM_HANDLES; i++)
    t14Encap.eoamHandles[i].state = T14_HANDLE_UNCONFIGURED;

  /* Runtime initial values for queue */
  matchRoute2[T14_MAC_NEXT_ROUTE_L2CAP].queue = (uint16_t)t14Encap.tf->QGen[Q_L2_CAPTURE];

  matchRoute2[T14_IP_NEXT_ROUTE_HOST].queue = (uint16_t) t14Encap.tf->QGen[Q_MATCH];
  nfailRoute_1.queue                        = (uint16_t) t14Encap.tf->QGen[Q_MATCH];
  matchRoute2[T14_IP_NEXT_ROUTE_HOST_ESP].queue = (uint16_t) t14Encap.tf->QGen[Q_ESP_FIX];

  nfailRoute_2.queue    = (uint16_t) t14Encap.tf->QGen[Q_NFAIL];
  cmdReply.queue      = (uint16_t) t14Encap.tf->QGen[Q_CMD_REPLY];

  /* Runtime initial values for flow */
  nfailRoute_1.flowId                            =
  nfailRoute_2.flowId                            =
  t14pktRoute1.flowId                            =
  t14pktRoute2.flowId                            =
  cmdReply.flowId                                =  t14Encap.tf->tfFlowNum[0];


  matchRoute2[T14_IP_NEXT_ROUTE_HOST].flowId     =
  matchRoute2[T14_IP_NEXT_ROUTE_LUT1].flowId     =
  matchRoute2[T14_IP_NEXT_ROUTE_LUT2].flowId     =
  matchRoute2[T14_IP_NEXT_ROUTE_IPSEC2].flowId   =
  matchRoute2[T14_MAC_NEXT_ROUTE_L2CAP].flowId   =
  matchRoute2[T14_IP_NEXT_ROUTE_HOST_ESP].flowId = t14Encap.tf->tfFlowNum[0];


  t14OutIpReassmCfg.destFlowId = t14Encap.tf->tfFlowNum[0];
  t14InIpReassmCfg.destFlowId = t14Encap.tf->tfFlowNum[0];

#ifdef NSS_GEN2
    #ifndef NETSS_INTERNAL_PKTDMA
      t14OutRaGroupCfg.flowId = t14Encap.tf->tfFlowNum1;
      t14InRaGroupCfg.flowId = t14Encap.tf->tfFlowNum2;
    #else
      t14OutRaGroupCfg.flowId = t14Encap.tf->tfLocFlowNum;
      t14InRaGroupCfg.flowId = t14Encap.tf->tfLocFlowNum;
    #endif
#endif

  /* Zero out the expected stats. The stats will be updated as packets are sent into PA */
  memset (&paTestL2ExpectedStats, 0, sizeof(paTestL2ExpectedStats));
  memset (&paTestExpectedUsrStats, 0, sizeof(paTestExpectedUsrStats));

  /* Initialize the reassembly control blocks */
  memset(&ipReassemCfg, 0, sizeof(paIPReassemblyConfig_t));
  ipReassemCfg.timeout = 5000;
  ipReassemCfg.descSize = TF_SIZE_DESC;
  ipReassemCfg.numReassemblyContexts = 10;

  if (paEx_reassemLibInit(&ipReassemCfg))
  {
    System_printf ("%s (%s:%d):  sample_initIPReassembly fail\n", tfName, __FILE__, __LINE__);
    t14Cleanup (&t14Encap, PA_TEST_FAILED);
  }

  /* Global Configurations for User Stats */
  newStatus = t14GlobalConfiguration (&t14Encap, T14_GLOBAL_SYS_CONFIG_USR_STATS);
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }

  /* Allocate User-defined statistics */
  /* Initialize the user statsTbl for EOAM */
  for (i = 0; i < T14_NUM_EOAM_USR_STATS; i++) {
    memset(&t14UsrStatsTbl[i], 0, sizeof (pauUsrStatsEntry_t));
    t14UsrStatsTbl[i].cntIndex = 0;
    t14UsrStatsTbl[i].lnkId    = PAU_USR_STATS_NO_LINK;
    t14UsrStatsTbl[i].fAlloc   = TRUE;
    t14UsrStatsTbl[i].f64b     = FALSE;
    t14UsrStatsTbl[i].fByteCnt = FALSE;
  }
  /* Initialize oter user stats */
  for (i = T14_NUM_EOAM_USR_STATS, n = 0; i < T14_NUM_USR_STATS; i++) {
    t14UsrStatsTbl[i] = t14OtherUsrStatsTbl[n++];
  }
  if(testCommonAllocUsrStats(t14Encap.tf, tfName, T14_NUM_USR_STATS, t14UsrStatsTbl, t14UsrStatsGroup1))
  {
      newStatus = PA_TEST_FAILED;
      t14Cleanup (&t14Encap, newStatus);  /* No return */
  }

  /* Global Configuration 2 */
  t14PktCtrlCfg.rxPaddingErrStatsIndex = t14UsrStatsTbl[T14_USR_STATS_ID_L2_PADDING_ERR].cntIndex;
  t14PktCtrlCfg.txPaddingStatsIndex    = t14UsrStatsTbl[T14_USR_STATS_ID_L2_TX_PADDING].cntIndex;
  newStatus                            = t14GlobalConfiguration (&t14Encap, T14_GLOBAL_SYS_CONFIG_PKTCTRL);

  if (newStatus == PA_TEST_FAILED)
    t14Cleanup (&t14Encap, newStatus);  /* No return */

  /* Usr Stats configuration */
  newStatus = testCommonUsrStatsSetup (t14Encap.tf, t14Encap.pat, tfName, sizeof(t14UsrStatsSetup)/sizeof(pauUsrStatsSetup_t), t14UsrStatsSetup,
                                       T14_CMD_SWINFO0_USR_STATS_CFG_ID, t14Encap.tf->QLinkedBuf3, t14Encap.tf->QGen[Q_CMD_RECYCLE],
                                       t14Encap.tf->QGen[Q_CMD_REPLY], FALSE);

  /* Global configurations for Eoam and others (Enables EOAM feature also) */
  newStatus = t14GlobalConfiguration (&t14Encap, T14_GLOBAL_SYS_CONFIG_EOAM_ENABLE);
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }

#if T14_DEBUG_CODE
  /* Check if the time accumulation is going on correct */
  for ( i = 0; i < T14_MAX_TIME_ACC; i++) {
    t14TimeAccNSecErr[i]  = accMem[0];
    t14TimeAccNSec[i]     = accMem[1];
    t14RoLSW[i]           = RoMem[0];
    t14RoMSW[i]           = RoMem[1];
    /* Wait enough time to trigger rollovers for next read */
    utilCycleDelay(73830*4);
  }
#endif

  /* Burst in the L2 configuraton */
  newStatus = t14OpenL2 (&t14Encap, t14EthSetup, T14_NUM_L2_ENTRIES);
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }

  /* Burst in the L3 configuration */
  newStatus = t14OpenL3 (&t14Encap, t14IpSetup, T14_NUM_L3_ENTRIES);
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }

  /* Burst in the ACL configuraton */
  newStatus = t14OpenAcl (&t14Encap, t14AclSetup, T14_NUM_ACL_ENTRIES);
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }

  /* Burst in the L4 configuration */
  newStatus = t14OpenL4 (&t14Encap, t14UdpSetup, T14_NUM_L4_ENTRIES);
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }

  /* Set the time offset for EOAM */
  newStatus = t14GlobalConfiguration (&t14Encap, T14_EOAM_TIME_OFFSET_CONFIG);
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }
    paTestL2ExpectedStats.classify2.nPackets += 1;
    paTestL2ExpectedStats.classify1.nPackets += 1;


  /* Add Eoam target classification entries */
    newStatus = t14OpenEoam (&t14Encap, t14EoamInfo, T14_NUM_EOAM_ENTRIES, "Ethernet OAM setup");
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }

#if 1
  /* Clear the Send and receive states */
  for (i = 0; i < n; i++) {
    t14EoamPktTestInfo[i].ctrlBitMap &= ~(T14_PKT_SENT | T14_PKT_RECEIVED);
  }
  /* Fire in each of the EOAM packets multiple times */
  expCount = t14SendEoamPkts(t14EoamPktTestInfo, tfName);

  /* Receive the EOAM Packets */
  if (t14RxEoamPkts (&t14Encap, t14EoamPktTestInfo, expCount)) {
    t14Cleanup (&t14Encap, PA_TEST_FAILED);
    /* Return */
    Task_exit();
  }

  /* NAT-T Configuration */
	newStatus = t14NatTConfiguration (&t14Encap, 1);
	if (newStatus == PA_TEST_FAILED)
		t14Cleanup (&t14Encap, newStatus);  /* No return */

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t14Encap.tf, t14Encap.pat, tfName, &paTestL2ExpectedStats, t14Encap.tf->QLinkedBuf1,
	                       t14Encap.tf->QGen[Q_CMD_RECYCLE], t14Encap.tf->QGen[Q_CMD_REPLY], TRUE);

	if (newStatus == PA_TEST_FAILED) {
 		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	  t14Cleanup (&t14Encap, newStatus);
  }

  /* Fire in each of the IP packets multiple times */
  n = sizeof (t14IpPktTestInfo) / sizeof (pktTestInfo2_t);
  /* Clear the Send and receive states */
  for (i = 0; i < n; i++) {
    t14IpPktTestInfo[i].ctrlBitMap &= ~(T14_PKT_SENT | T14_PKT_RECEIVED);
  }

  expCount = t14SendPkts(t14IpPktTestInfo, tfName);
  /* Receive the IPSec/IP Packets */
  if (t14RxPkts (&t14Encap, t14IpPktTestInfo, expCount)) {
    t14Cleanup (&t14Encap, PA_TEST_FAILED);
    /* Return */
    Task_exit();
  }

    /* NAT-T Configuration */
	newStatus = t14NatTConfiguration (&t14Encap, 0);
	if (newStatus == PA_TEST_FAILED)
		t14Cleanup (&t14Encap, newStatus);  /* No return */

  /* Fire in each of the ACL packets multiple times */
  /* Display ACL statistics */
  t14DispAclStats(&t14Encap);

	/* Verify and clear the stats */
	newStatus =  testCommonCheckStats (t14Encap.tf, t14Encap.pat, tfName, &paTestL2ExpectedStats, t14Encap.tf->QLinkedBuf1,
	                       t14Encap.tf->QGen[Q_CMD_RECYCLE], t14Encap.tf->QGen[Q_CMD_REPLY], TRUE);

	if (newStatus == PA_TEST_FAILED) {
 		System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
	  t14Cleanup (&t14Encap, newStatus);
  }
#endif

#if 1
  /* Send the packets. Use the modify PDSP to generate the TCP/UDP checksums.
   * Use the modify PDSP to genertae IP fragments if necessary */
  t14pktRoute1.statsIndex = t14UsrStatsTbl[T14_USR_STATS_ID_TX_PKTS].cntIndex;
  t14pktRoute1.queue      = TF_PA_QUEUE_TXCMD;
  for (k = 0; k < T14_NUM_PACKET_GROUPS; k+=1)  {
    /*
     * There are 4 streams of variable-size packets
     * Send multiple test groups at a time to verify the followings:
     *   - PASS can maintain multiple traffic flows
     *   - PASS will forward both fragments and non-fragmented packets to the host
     *     when the traffic flow is active
     */
    for (i = 0; i < 1; i++) {
      t14SendDataPkts (&t14Encap, T14_NUM_REASM_PKTS, k + i, i == 0);
    }

    if (t14ReceiveDataPkts (&t14Encap, T14_NUM_REASM_PKTS * i)) {
      /* Display RA statistics */
      Pa_queryRaStats (t14Encap.tf->passHandle, TRUE, &paTestRaStats);
      testCommonDispRaStats (tfName, &paTestRaStats);
      System_printf ("%s (%s:%d):  Receive packets fail\n", tfName, __FILE__, __LINE__);
      System_flush();
      t14Cleanup (&t14Encap, PA_TEST_FAILED);
    }
  }
#endif

#ifdef PA_USE_HW_RA
    /* Display RA statistics */
    Pa_queryRaStats (t14Encap.tf->passHandle, TRUE, &paTestRaStats);
    testCommonDispRaStats (tfName, &paTestRaStats);
#endif

#if 1
  /* Now check the egress Packets (TxCmd) */
  t14SendRxEgressDataPkts(&t14Encap, T14_NUM_EGRESS0_CMD_ENTRIES, 0, 0);
#endif

  /* Verify and clear the Usr stats */
  newStatus =  testCommonCheckUsrStatsList (t14Encap.tf, t14Encap.pat, tfName, &paTestExpectedUsrStats, T14_NUM_64B_USR_STATS, T14_NUM_USR_STATS, t14UsrStatsTbl,
                                            t14Encap.tf->QLinkedBuf1, t14Encap.tf->QGen[Q_CMD_RECYCLE], t14Encap.tf->QGen[Q_CMD_REPLY], TRUE);

#ifndef PA_USE_HW_RA
  /* Note: Expected tx padding count can not be verified with Hardware RA module */
  if (newStatus == PA_TEST_FAILED) {
    System_printf ("%s (%s:%d): testCommonCheckUsrStats Failed\n", tfName, __FILE__, __LINE__);
    t14Cleanup (&t14Encap, newStatus);
  }
#endif

  /* Verify and clear the stats */
  newStatus =  testCommonCheckStats (t14Encap.tf, t14Encap.pat, tfName, &paTestL2ExpectedStats, t14Encap.tf->QLinkedBuf1,
                                     t14Encap.tf->QGen[Q_CMD_RECYCLE], t14Encap.tf->QGen[Q_CMD_REPLY], TRUE);
  if (newStatus == PA_TEST_FAILED) {
    System_printf ("%s (%s:%d): testCommonCheckStats Failed\n", tfName, __FILE__, __LINE__);
    t14Cleanup (&t14Encap, newStatus);
  }

  /* Clear all Usr Stats Link */
  newStatus = testCommonUsrStatsConfigReset (t14Encap.tf, t14Encap.pat, tfName, sizeof(t14UsrStatsSetup)/sizeof(pauUsrStatsSetup_t), t14UsrStatsSetup,
                                             T14_CMD_SWINFO0_USR_STATS_CFG_ID, t14Encap.tf->QLinkedBuf3, t14Encap.tf->QGen[Q_CMD_RECYCLE],
                                             t14Encap.tf->QGen[Q_CMD_REPLY]);

  if (newStatus == PA_TEST_FAILED) {
    System_printf ("%s (%s:%d): testCommonUsrStatsConfigReset Failed\n", tfName, __FILE__, __LINE__);
    t14Cleanup (&t14Encap, newStatus);
  }

  /* Free User-defined statistics */
  if(testCommonFreeUsrStats(t14Encap.tf, tfName, T14_NUM_USR_STATS, t14UsrStatsTbl)) {
    System_printf ("%s (%s:%d): testCommonFreeUsrStats Failed\n", tfName, __FILE__, __LINE__);
    t14Cleanup (&t14Encap, PA_TEST_FAILED);
  }

  /* No return from cleanup */
  t14Cleanup (&t14Encap, newStatus);

  /* Disable the EOAM feature for other tests */
  newStatus = t14GlobalConfiguration (&t14Encap, T14_GLOBAL_SYS_CONFIG_EOAM_DISABLE);
  if (newStatus == PA_TEST_FAILED) {
    t14Cleanup (&t14Encap, newStatus);  /* No return */
    /* Return */
    Task_exit();
  }

#ifdef __LINUX_USER_SPACE
  return (void *)0;
#endif
}


