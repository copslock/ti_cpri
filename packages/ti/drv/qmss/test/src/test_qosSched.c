/**
 *   @file  test_qosSched.c
 *
 *   @brief   
 *      This is the QMSS unit test code for QoS scheduler.  This
 *      corresponds to the firmware in qos_sched_be[] and 
 *      qos_sched_le[] and the LLD in qmss_qosSched.c
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2015, Texas Instruments, Inc.
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
 *  \par
*/

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <string.h>
#include <stddef.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>
#include <ti/drv/qmss/qmss_qosSched.h>

/* CPPI LLD includes */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* RM include */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>

/* CSL RL includes */
#include <ti/csl/csl_chip.h>

/* OSAL includes */
#include <qmss_osal.h>

/* Test includes */
#include <test_qosSchedScen.h>
#include <stdlib.h>
#include <test_qosSched.h>
#ifdef TEST_DROP_SCHED
#include <test_qosSchedDropSched.h>
#endif

#define RM 1

#ifdef USE_QOS_MODEL
  #define SUBSYS (Qmss_SubSysHnd) 0
  #define PDSPID Qmss_PdspId_CMODEL_DSP1
#else
  #define SUBSYS (Qmss_SubSysHnd) 0
  #define PDSPID Qmss_PdspId_PDSP2
#endif

#ifdef TEST_DROP_SCHED
#define DESC_AREA_SIZE ((SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC) + \
                        (SIZE_STATS_DESC * NUM_STATS_DESC))
#define NUM_DESC (NUM_MONOLITHIC_DESC + NUM_STATS_DESC)
#else
#define DESC_AREA_SIZE (SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC)
#define NUM_DESC (NUM_MONOLITHIC_DESC)
#endif
/************************ GLOBAL VARIABLES ********************/
/* Descriptor pool.  2 areas allocated in 1 array to ensure ordering */
#pragma DATA_ALIGN (descArea, 16)
UInt8 descArea[DESC_AREA_SIZE];

/* Timestamp when model moved each descriptor */
#ifdef _TMS320C6X
#pragma DATA_SECTION  (timestamps, ".far:timestamps");
#endif
struct 
{
    uint32_t *descPtr;
    uint32_t  timestamp;
} timestamps[NUM_MONOLITHIC_DESC];

/* This allocates memory as if all ports are used */
#define FULL_PORT_MAX_DESC (TEST_MAX_FULL_LOG_GROUPS * QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP * PROFILE_DESCS)
#define LITE_PORT_MAX_DESC (QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS * QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP * PROFILE_DESCS)
#if TEST_FULL_MAX_PHYS_PORTS > 0
uint32_t fullPortSeqns[TEST_FULL_MAX_PHYS_PORTS][FULL_PORT_MAX_DESC];
#endif
#if TEST_LITE_MAX_PHYS_PORTS > 0
uint32_t litePortSeqns[TEST_LITE_MAX_PHYS_PORTS][LITE_PORT_MAX_DESC];
#endif

uint32_t *expectedSeqn[TEST_PORTS] =
{
#if TEST_FULL_MAX_PHYS_PORTS >= 1
   fullPortSeqns[0],
#endif
#if TEST_FULL_MAX_PHYS_PORTS >= 2
   fullPortSeqns[1],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 1
   litePortSeqns[0],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 2
   litePortSeqns[1],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 3
   litePortSeqns[2],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 4
   litePortSeqns[3],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 5
   litePortSeqns[4],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 6
   litePortSeqns[5],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 7
   litePortSeqns[6],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 8
   litePortSeqns[7],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 9
   litePortSeqns[8],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 10
   litePortSeqns[9],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 11
   litePortSeqns[10],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 12
   litePortSeqns[11],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 13
   litePortSeqns[12],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 14
   litePortSeqns[13],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 15
   litePortSeqns[14],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 16
   litePortSeqns[15],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 17
   litePortSeqns[16],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 18
   litePortSeqns[17],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 19
   litePortSeqns[18],
#endif
#if TEST_LITE_MAX_PHYS_PORTS >= 20
   litePortSeqns[19],
#endif
};



pktGenEngList_t pktGenEngList;
pktGenEngState_t pktGenEngStates[MAX_PKT_GEN_QUEUES];

/* Global variable common to all test cases */

/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;
/* QM descriptor configuration */
Qmss_DescCfg            descCfg;
/* Store the queue handle for destination queues on which allocated descriptors are stored */
Qmss_QueueHnd           QueHnd[QMSS_MAX_MEM_REGIONS];

#if RM    
Rm_Handle               rmHandle = NULL;
#endif


/************************ EXTERN VARIABLES ********************/

#if RM
/* RM test Global Resource List (GRL) */
extern const char rmGlobalResourceList[];
/* RM test Global Policy provided to RM Server */
extern const char rmDspOnlyPolicy[];
#endif

/************************** FUNCTIONS *************************/
/**
 *  @b Description
 *  @n  
 *      Utility function which converts a local GEM L2 memory address 
 *      to global memory address.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Computed L2 global Address
 */
static UInt32 l2_global_address (UInt32 addr)
{
#ifdef _TMS320C6X
	    UInt32 corenum;

        /* Get the core number. */
        corenum = CSL_chipReadReg (CSL_CHIP_DNUM); 

        /* Compute the global address. */
        return (addr + (0x10000000 + (corenum * 0x1000000)));
#else
        return (addr);
#endif
}

/*****************************************************************************
 * Function concatenates port, group, queue, and descNum into a unique ID
 * that can be written into the descriptor
 *****************************************************************************/
static uint32_t inline make_descID (uint32_t port, uint32_t group, uint32_t queue, uint32_t descNum)
{
    uint32_t result = 
#ifdef WIDE
            (port  << 29) |
#else
            (port  << 27) |
#endif
            (group << 24) | (queue << 20) | (descNum & 0xfffff);
    return result;
}

/*****************************************************************************
 * Function splits port, group, queue, and descNum from unique ID
 *****************************************************************************/
#ifdef DEBUG_FCN
static break_descID (uint32_t *port, uint32_t *group, uint32_t *queue, uint32_t *descNum, uint32_t descID)
{
#ifdef WIDE
    *port    = (descID >> 29) & 0x0007;
    *group   = (descID >> 24) & 0x001f;
#else
    *port    = (descID >> 27) & 0x001f;
    *group   = (descID >> 24) & 0x0007;
#endif
    *queue   = (descID >> 20) & 0x000f;
    *descNum = (descID >>  0) & 0xffff;
}
#endif
/*****************************************************************************
 * Function splits just port
 *****************************************************************************/
static inline uint32_t break_descID_port (uint32_t descID)
{
#ifdef WIDE
    return ((descID >> 29) & 0x0007);
#else
    return ((descID >> 27) & 0x001f);
#endif
}
/*****************************************************************************
 * Divert with error check
 *****************************************************************************/
void queue_divert_and_check (Qmss_QueueHnd src, Qmss_QueueHnd dst)
{
    Qmss_Result result;

    if ((result = Qmss_queueDivert (src, dst, Qmss_Location_TAIL)) != QMSS_SOK)
    {
#ifdef _TMS320C6X
    	UInt32 corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    	uint32_t corenum = 0;
#endif
        errorCount++;
        System_printf("core %d: queue divert from %d to %d failed: %d\n", corenum, src, dst, result);
    }
}

#if defined (__ARM_ARCH_7A__)
/*****************************************************************************
 * Function: Utility function a cycle clock
 ****************************************************************************/
static uint32_t utilReadTime32(void)
{
    uint32_t timeVal;
    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(timeVal));
    return timeVal;
}

/*****************************************************************************
 * Function: Utility function to introduce delay
 ****************************************************************************/
void utilCycleDelay (int32_t iCount)
{
  uint32_t start = utilReadTime32();
  uint32_t count;

  if (iCount <= 100)
    iCount = 100;

  count = (uint32_t)iCount;

  while ((utilReadTime32() - start) < count);
}

#endif

/**
 *  @b Description
 *  @n  
 *      Utility function to sleep N cycles.  This is immune to timer rollover.
 *
 *  @param[in]  n
 *      Number of cycles to sleep
 *
 */
void delay (uint32_t cycles)
{
#ifdef _TMS320C6X
	uint32_t start = TSCL;

    while ( (TSCL - start) < cycles);
#elif defined (__ARM_ARCH_7A__)
#else
    utilCycleDelay((int32_t) cycles);
#endif
}

/**
 *  @b Description
 *  @n  
 *      Utility function to get 64 bit timer
 *
 *  @param[in]  n
 *      timer value
 *
 */
static inline uint64_t read_timer64 ()
{
#ifdef _TMS320C6X
	uint32_t low = TSCL;
    uint32_t high = TSCH;
    return _itoll (high, low);
#elif defined (__ARM_ARCH_7A__)
    uint32_t low;
    static uint32_t high = 0;
    static uint32_t last = 0;

    low  = utilReadTime32();
    if (last > low) {
      high++;
    }
    last = low;
    return (((uint64_t)high << 32) | low);
#else
#endif

}

/**
 *  @b Description
 *  @n  
 *      Utility function to convert Qmss_QosSchedAcctType to a string for print
 *
 *  @param[in]  type
 *      Value to convert
 *
 */

const char *type_packets = "Qmss_QosSchedAcctType_PACKETS";
const char *type_bytes   = "Qmss_QosSchedAcctType_BYTES";
const char *type_invalid = "Invalid Type";
const char *string_type (Qmss_QosSchedAcctType type)
{
    const char *ret = type_invalid;
    switch (type) {
        case Qmss_QosSchedAcctType_PACKETS:
            ret = type_packets;
            break;
        case Qmss_QosSchedAcctType_BYTES:
            ret = type_bytes;
            break;
    }
    return ret;
}

/**
 *  @b Description
 *  @n  
 *      Utility function to print configuration
 *
 *  @param[in]  cfg
 *      Configuration to print
 *
 */
void print_config (Qmss_QosSchedPortCfg *cfg)
{
    int group, queue;

    System_printf (        "cfg->wrrType          = %s\n", string_type(cfg->wrrType));
    System_printf (        "cfg->cirType          = %s\n", string_type(cfg->cirType));
    System_printf (        "cfg->congestionType   = %s\n", string_type(cfg->congestionType));
    System_printf (        "cfg->outThrotType     = %s\n", string_type(cfg->outThrotType));
    System_printf (        "cfg->cirIteration     = %d\n", cfg->cirIteration);
    System_printf (        "cfg->cirMax           = %d\n", cfg->cirMax);
    System_printf (        "cfg->groupCount       = %d\n", cfg->groupCount);
    System_printf (        "cfg->outputQueue.qMgr = %d\n", cfg->outputQueue.qMgr);
    System_printf (        "cfg->outputQueue.qNum = %d\n", cfg->outputQueue.qNum);

    for (group = 0; group < cfg->groupCount; group++)
    {
        Qmss_QosSchedGroupCfg *pGroup = &cfg->group[group];
        System_printf (    "cfg->group[%d].cirIteration     = %d\n", group, pGroup->cirIteration);
        System_printf (    "cfg->group[%d].pirIteration     = %d\n", group, pGroup->pirIteration);
        System_printf (    "cfg->group[%d].cirMax           = %d\n", group, pGroup->cirMax);
        System_printf (    "cfg->group[%d].pirMax           = %d\n", group, pGroup->pirMax);
        System_printf (    "cfg->group[%d].wrrInitialCredit = %d\n", group, pGroup->wrrInitialCredit);
        System_printf (    "cfg->group[%d].totQueueCount    = %d\n", group, pGroup->totQueueCount);
        System_printf (    "cfg->group[%d].spQueueCount     = %d\n", group, pGroup->spQueueCount);
        System_printf (    "cfg->group[%d].wrrQueueCount    = %d\n", group, pGroup->wrrQueueCount);
        for (queue = 0; queue < pGroup->totQueueCount; queue++)
        {
            Qmss_QosSchedQueueCfg *pQueue = &pGroup->Queue[queue];
            System_printf (    "cfg->group[%d].Queue[%d].wrrInitialCredit = %d\n", group, queue, pQueue->wrrInitialCredit);
            System_printf (    "cfg->group[%d].Queue[%d].congestionThresh = %d\n", group, queue, pQueue->congestionThresh);
        }
    }
}

/**
 *  @b Description
 *  @n  
 *      Utility function to convert bitrate or packet rate to firmware parameters
 *
 *  @param[out]  res
 *      result
 *  @param[in] ticks_second
 *      current qos clock rate
 *  @param[in] rate
 *      rate (bits per second or packets per second) to convert
 *  @param[in] type
 *      specifies whether rate is in packets per second or bits per second
 *
 */
void scenario_rate_conversion_wrapper (int32_t *res, uint32_t ticks_second, uint32_t rate, Qmss_QosSchedAcctType type)
{
    uint32_t corenum;
    Qmss_Result retVal;

    if (type == Qmss_QosSchedAcctType_BYTES)
    {
         retVal = Qmss_convertQosSchedBitRate (res, ticks_second, rate);
    }
    else
    {
         retVal = Qmss_convertQosSchedPacketRate (res, ticks_second, rate);
    }
    if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
#ifdef _TMS320C6X
    	corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    	corenum = 0;
#endif
        System_printf ("\nCore %d: Rate conversion failed: %d\n", corenum, retVal);
        errorCount++;
    }
}

/**
 *  @b Description
 *  @n  
 *      Utility function to convert bitrate or packet rate to firmware parameters
 *
 *  @param[out]  res
 *      result
 *  @param[in] rate
 *      rate (bits per second or packets per second) to convert
 *  @param[in] type
 *      specifies whether rate is in packets per second or bits per second
 *
 */
void scenario_wrr_conversion_wrapper (int32_t *res, uint32_t wrrCredit, Qmss_QosSchedAcctType type)
{
    uint32_t corenum;
    Qmss_Result retVal;

    if (type == Qmss_QosSchedAcctType_BYTES)
    {
         retVal = Qmss_convertQosSchedWrrBits (res, wrrCredit);
    }
    else
    {
         retVal = Qmss_convertQosSchedWrrPackets (res, wrrCredit);
    }
    if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
#ifdef _TMS320C6X
    	corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    	corenum = 0;
#endif
        System_printf ("\nCore %d: Rate conversion failed: %d\n", corenum, retVal);
        errorCount++;
    }
}

/* helper function to configure a joint WRR port used by multiple scenarios */
int scenario_config_common_joint (
    uint32_t *timer, Qmss_QosSchedPortCfg *portCfg, int *portMap, 
    packetGenCfg *pktGen, int *nPorts, int nJointWrr)
{
    uint32_t                       ports           = 3;
    uint32_t                       pktGens         = 8 + nJointWrr;
    uint32_t                       ticks_second    = 1000000 / 50;
    uint32_t                       port;
    uint32_t                       gen;
    uint32_t                       queue;
    uint32_t                       sum;
    Qmss_QosSchedAcctType          bytes           = Qmss_QosSchedAcctType_BYTES;

    /* Make sure enough memory is in portCfg */
    if (*nPorts < ports)
    {
        System_printf ("Build supports too few ports (%d, %d)\n", *nPorts, ports);
        return 1;
    }

    *nPorts = ports;
    /* Make sure enough memory is in pktGen */
    if (pktGen->numQCfgs < pktGens)
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /* Map indicies 0,1,2 to ports 5,6,8 */
    portMap[0] = 5;
    portMap[1] = 8;
    portMap[2] = 6;

    /* set port 1,4 to be 4 wrr */
    memset (portCfg, 0, ports * sizeof(Qmss_QosSchedPortCfg));

    for (port = 0; port < ports; port++)
    {
        portCfg[port].isJoint = Qmss_QosSchedIsJoint_SPLIT;
        portCfg[port].wrrType = bytes;
        portCfg[port].cirType = bytes;
        portCfg[port].outThrotType = bytes;
        portCfg[port].congestionType = Qmss_QosSchedAcctType_PACKETS;
        scenario_rate_conversion_wrapper 
            (&portCfg[port].cirMax, 1, 7000000, bytes);
        portCfg[port].overheadBytes = 0;
        portCfg[port].groupCount = 1;
        portCfg[port].outThrotThresh = 0;
        portCfg[port].group[0].totQueueCount = 4;
        portCfg[port].group[0].spQueueCount = 0;
        portCfg[port].group[0].wrrQueueCount = 4;
        for (queue = 0; queue < 8; queue++)
        {    
            scenario_wrr_conversion_wrapper 
                (&portCfg[port].group[0].Queue[queue].wrrInitialCredit,
                 8000 * (queue + 1), bytes);
            portCfg[port].group[0].Queue[queue].congestionThresh = 50;
        }
    }
    scenario_rate_conversion_wrapper (&portCfg[0].cirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper (&portCfg[1].cirIteration, ticks_second, 20000000, bytes);
    scenario_rate_conversion_wrapper (&portCfg[2].cirIteration, ticks_second, 15000000, bytes);
#ifdef TEST_MULTI_GROUP
    /* this doesn't hurt functionality on drop sched, but port readback would fail since
     * firmware drops this part of config */
    for (port = 0; port < ports; port++)
    {
        portCfg[port].group[0].cirIteration     = portCfg[port].cirIteration;
        portCfg[port].group[0].cirMax           = portCfg[port].cirMax;
        portCfg[port].group[0].wrrInitialCredit = portCfg[port].group[0].Queue[0].wrrInitialCredit;
    }
#endif
    /* turn middle port into joint port */
    portCfg[2].isJoint = Qmss_QosSchedIsJoint_JOINT;
    portCfg[2].group[0].totQueueCount = nJointWrr;
    portCfg[2].group[0].wrrQueueCount = nJointWrr;

    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));
    for (gen = 0; gen < pktGens; gen++)
    {
        pktGen->Qs[gen].group      = 0;
        pktGen->Qs[gen].nSizes     = 1;
        pktGen->Qs[gen].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[gen].pktSize[0] = 100;
    }

    /* 4:3:2:1 */
    for (gen = 0; gen < 4; gen++)
    {
        pktGen->Qs[gen].pps        = (12500000 * (gen + 1) / 10) / (pktGen->Qs[gen].pktSize[0] * 8);
        pktGen->Qs[gen].ppsExp     = (10000000 * (gen + 1) / 10) / (pktGen->Qs[gen].pktSize[0] * 8);
        pktGen->Qs[gen + 4].pps    = (25000000 * (gen + 1) / 10) / (pktGen->Qs[gen + 4].pktSize[0] * 8);
        pktGen->Qs[gen + 4].ppsExp = (20000000 * (gen + 1) / 10) / (pktGen->Qs[gen + 4].pktSize[0] * 8);

        pktGen->Qs[gen].queue      = 0 + gen;
        pktGen->Qs[gen].port       = 5;
        pktGen->Qs[gen + 4].queue  = 0 + gen;
        pktGen->Qs[gen + 4].port   = 8;
    }

    /* 8:7:6:5:4:3:2:1 */
    sum = (nJointWrr * (nJointWrr + 1)) / 2;
    for (gen = 0; gen < nJointWrr; gen++)
    {
        pktGen->Qs[gen + 8].pps    = (18750000 * (gen + 1) / sum) / (pktGen->Qs[gen + 8].pktSize[0] * 8);
        pktGen->Qs[gen + 8].ppsExp = (15000000 * (gen + 1) / sum) / (pktGen->Qs[gen + 8].pktSize[0] * 8);

        pktGen->Qs[gen + 8].queue  = 0 + gen;
        pktGen->Qs[gen + 8].port   = 6;
    }

    pktGen->numQCfgs = pktGens;
    /* run long enough for 1% accuracy */
    pktGen->genTime  = 10000;
    pktGen->readTime = 10500;
#if defined(QOS_SCHED_FAST_SCENARIO) && !defined(TEST_DROP_SCHED)
    /* this one really needs to run 10 seconds for 1% accuracy */
    pktGen->genTime *= 100;
    pktGen->readTime *= 100;
#endif

    for (gen = 0; gen < pktGens; gen++)
    {
        pktGen->Qs[gen].ppsError   = pktGen->Qs[gen].ppsExp / (100); /* 1.0% */
    }

    return 0;
}

/*****************************************************************************
 * The following data structures are a C model of the firmware's foreground
 * scheduling.  It doesn't consider congestion dropping since that is a 
 * background task that uses all available fw cycles to drop.
 * **************************************************************************/
typedef struct {
    int32_t       WrrInitialCredit;     // Initial Queue WRR credit on a "new" schedule
    int32_t       WrrCurrentCredit;     // Current Queue WRR credit
    uint32_t      CongestionThresh;     // The max amount of congestion before drop
    uint32_t      PacketsForwarded;     // Number of packets forwarded
    uint64_t      BytesForwarded;       // Number of bytes forwarded
    Qmss_QueueHnd QueueNumber;          // Queue Number/Queue Handle
} modelQueue;

typedef struct {
#ifdef TEST_MULTI_GROUP
    int32_t     CirIteration;           // CIR credit per iteration
    int32_t     PirIteration;           // PIR credit per iteration
    int32_t     CirCurrent;             // Current CIR credit
    int32_t     PirCurrent;             // Current PIR credit
    int32_t     CirMax;                 // Max total CIR credit
    int32_t     PirMax;                 // Max total PIR credit
    int32_t     WrrInitialCredit;       // Initial Group WRR credit on a "new" schedule
    int32_t     WrrCurrentCredit;       // Current Group WRR credit
#endif
    uint8_t     QueueCount;             // Total number of active QOS queues (up to 8)
    uint8_t     SPCount;                // The number of SP queues (usually 2 or 3)
    uint8_t     RRCount;                // The number of RR queues (usually QueueCount-SPCount)
    uint8_t     NextQueue;              // The next RR queue to examine in the group
    uint8_t     WrrCreditMask;          // Flag mask of WRR queues that have WRR credit remaining
    modelQueue  Queue[8];               // Up to eight queues per logical group
} modelGroup;

typedef struct {
    int           fByteWrrCredits;        // When set, WRR credits are in bytes, else packets
    int           fByteCirCredits;        // When set, CIR/PIR credits are in bytes, else packets
    int           fByteCongest;           // When set, congestion is in bytes, else packets
    int           fByteDestThrottle;    // When set, destination throttle is in bytes, else packets
    int32_t       CirIteration;           // CIR credit per iteration (always in bytes)
    int32_t       CirCurrent;             // Current CIR credit (always in bytes)
    int32_t       CirMax;                 // Max total CIR credit (always in bytes)
    uint32_t      WrrCreditMask;          // Flag mask of WRR groups that have WRR credit remaining
    uint8_t       GroupCount;             // The number of logical groups
    uint8_t       NextGroup;              // The next RR group to examine
    uint8_t       LastTimerTicks;         // Used to schedule missed interrupts.  Initialized
                                          // to TimerTicks when port is turned on.
    uint8_t       OverheadBytes;          // Number of bytes of wire overhead to account, beyond packet size in QM.
                                          // This is often set to 24.  This only affects credits deducted,
                                          // not statistics.  It also only has effect on credits configured
    uint8_t       RemoveBytes;            // Number of bytes to remove from accounting for each packet.
                                          // This only affects credits deducted,
                                          // not statistics.  It also only has effect on credits configured
                                          // as bytes, not packets.
    modelGroup    Group[TEST_MAX_FULL_LOG_GROUPS];  // Up to 1(drop)/5(normal)/17(wide) logical groups
    Qmss_QueueHnd DestQueueNumber;        // Output queue
    int32_t       DestThrottleThresh;   // If output queue has more than this many bytes/packets nothing will be forwarded
} modelPort;

modelPort qos_model_ports[TEST_MAX_PHYS_PORTS];
/* Using uint32_t to measure time model takes to run, internal operation only needs 8 bits */
uint32_t qos_model_timer_ticks = 0;

#ifdef QOS_MODEL_DEBUG_TRIGGER_ENABLE
modelPort qos_model_ports_dbg_snapshot[TEST_MAX_PHYS_PORTS];
#endif

/*****************************************************************************
 * model function to transfer a packet from input queue to output queue
 * **************************************************************************/
uint32_t qos_model_queue_scheduler (modelPort *pPort, modelQueue *pQueue)
{
    uint32_t ByteSize;
    void *desc;

    Qmss_queuePopDescSize (pQueue->QueueNumber, &desc, &ByteSize);

#ifdef QOS_MODEL_DEBUG_TRIGGER_ENABLE
    if ((*((uint32_t *)(QMSS_DESC_PTR(desc))) + DESC_ID_OFFSET) == QOS_MODEL_DEBUG_TRIGGER_VAL)
    {
        /* Make snapshot of state at this descriptor for debug purposes */
        memcpy (qos_model_ports_dbg_snapshot, qos_model_ports, sizeof(qos_model_ports));
    }
#endif
    Qmss_queuePush (pPort->DestQueueNumber, desc, ByteSize, QMSS_DESC_SIZE(desc), Qmss_Location_TAIL);

    pQueue->PacketsForwarded += 1;
    pQueue->BytesForwarded   += ByteSize;

    return(ByteSize);
}

/*****************************************************************************
 * model function to schedule a group
 * **************************************************************************/
int qos_model_group_scheduler (modelPort *pPort, modelGroup *pGroup)
{
    int32_t BytesUsed;
    int32_t packetSent = 0;
    uint8_t PacketPendingMask;
    int     i, j;

    PacketPendingMask = 0;
    for (i = 0; i < pGroup->QueueCount; i++)
    {
        if (Qmss_getQueueEntryCount (pGroup->Queue[i].QueueNumber)) 
        {
            PacketPendingMask |= 1 << i;
        }
    }

    // If no packets, nothing to do 
    if(!PacketPendingMask)
        return 0;

    //
    // Try to take a high priority queue first
    //
    for( i=0; i<pGroup->SPCount; i++ )
    {
        if( PacketPendingMask & (1<<i) )
            return( qos_model_queue_scheduler (pPort, &pGroup->Queue[i]) );
    }

    //
    // Next try to pick a round robin queue
    //
    if (PacketPendingMask & (((1 << pGroup->RRCount) - 1) << pGroup->SPCount))
    {
        // There are RR packets pending
        for( i=0; i<pGroup->RRCount; i++ )
        {
            // If all queues with WRR credit remaining are empty, reset the credit
            while ( !(pGroup->WrrCreditMask & PacketPendingMask) )
            {
                // Reset credits
                for(j=pGroup->SPCount; j<(pGroup->SPCount+pGroup->RRCount); j++)
                {
                    pGroup->Queue[j].WrrCurrentCredit += pGroup->Queue[j].WrrInitialCredit;

                    if (pGroup->Queue[j].WrrCurrentCredit > (pGroup->Queue[j].WrrInitialCredit << 1))
                        pGroup->Queue[j].WrrCurrentCredit = (pGroup->Queue[j].WrrInitialCredit << 1);
                      
                    if (pGroup->Queue[j].WrrCurrentCredit > 0 || (! pGroup->Queue[j].WrrInitialCredit))
                        pGroup->WrrCreditMask |= (1<<j);
                }

                // While loop must terminate given 
                // (PacketPendingMask & (((1 << pGroup->RRCount) - 1) << pGroup->SPCount)
            }

            // If the next queue has WRR credit and packets, then schedule a packet
            if( (pGroup->WrrCreditMask & PacketPendingMask) & (1<<pGroup->NextQueue) )
            {
                // Attempt to schedule a packet
                BytesUsed = qos_model_queue_scheduler( pPort, &pGroup->Queue[pGroup->NextQueue] );

                // Deduct the WRR credit
                if( pPort->fByteWrrCredits )
                    pGroup->Queue[pGroup->NextQueue].WrrCurrentCredit -= (BytesUsed + pPort->OverheadBytes - pPort->RemoveBytes) << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT;
                else
                    pGroup->Queue[pGroup->NextQueue].WrrCurrentCredit -= 1 << QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT;

                // Clear the queues's WWR credit mask if we depleted the WRR credit
                if( pGroup->Queue[pGroup->NextQueue].WrrCurrentCredit <= 0 )
                    pGroup->WrrCreditMask &= ~(1<<pGroup->NextQueue);

                packetSent = 1;
            }

            // Move on to the next group
            pGroup->NextQueue++;
            if( pGroup->NextQueue == pGroup->SPCount+pGroup->RRCount )
                pGroup->NextQueue = pGroup->SPCount;

            // Quit now if we moved a packet
            if(packetSent)
                return(BytesUsed);
        }
    }

    //
    // Finally, try to get a packet from the OPTIONAL best effort queues
    //
    for( i=pGroup->SPCount+pGroup->RRCount; i<pGroup->QueueCount; i++ )
    {
        if( PacketPendingMask & (1<<i) )
            return( qos_model_queue_scheduler (pPort, &pGroup->Queue[i]) );
    }

    // No packet was transferred
    return(0);
}
/*****************************************************************************
 * utility used by port model
 * Returns 0 if no space left, else 1
 ****************************************************************************/
int32_t model_phys_port_update_output_space (modelPort *pPort, int32_t *OutputSpaceAvail, int32_t BytesUsed)
{
    if (*OutputSpaceAvail)
    {
        if (pPort->fByteDestThrottle)
        {
            *OutputSpaceAvail -= BytesUsed + pPort->OverheadBytes - pPort->RemoveBytes;
        }
        else
        {
            (*OutputSpaceAvail)--;
        }

        if (*OutputSpaceAvail <= 0)
        {
            return 0;
        }
    }

    return 1;
}

/*****************************************************************************
 * model function to schedule a port
 ****************************************************************************/
int qos_model_port_scheduler (modelPort *pPort)
{
    int32_t  BytesUsed;              // Bytes used is returned from the Logical Scheduler
    int32_t  CirCreditUsed;          // Cir/Pir Credit used (in packets or bytes as configured)
#ifdef TEST_MULTI_GROUP
    int32_t  WrrCreditUsed;          // Wrr Credit used (in packets or bytes as configured)
    uint32_t PirCreditMask = 0;     // Flag set when more PIR credit remains
#endif
    uint32_t PacketPendingMask;     // Flag mask of RR groups that are not empty
    int      fPacketsSent;
    int      i;
    int      PacketScheduled = 0;
    int      OutputSpaceAvail = 0;

    /* Add credits for all TimerTicks that occurred since last time this port ran */
    while ((uint8_t)(qos_model_timer_ticks - pPort->LastTimerTicks) > 0)
    {
        pPort->LastTimerTicks++;
        //
        // Add credits for all time based credit counters
        //

        // Credit for the main port
        pPort->CirCurrent +=  pPort->CirIteration;
        if( pPort->CirCurrent > pPort->CirMax )
            pPort->CirCurrent = pPort->CirMax;

        // Credit for the port's logical groups
#ifdef TEST_MULTI_GROUP
        for( i=0; i<pPort->GroupCount; i++ )
        {
            pPort->Group[i].CirCurrent +=  pPort->Group[i].CirIteration;
            // Cap CIR credit at its max level
            if( pPort->Group[i].CirCurrent > pPort->Group[i].CirMax )
                pPort->Group[i].CirCurrent = pPort->Group[i].CirMax;
            pPort->Group[i].PirCurrent +=  pPort->Group[i].PirIteration;
            if( pPort->Group[i].PirCurrent > 0 )
            {
                // Track every group with PIR credit for later
                PirCreditMask |= (1<<i);
                // Cap PIR credit at its max level
                if( pPort->Group[i].PirCurrent > pPort->Group[i].PirMax )
                    pPort->Group[i].PirCurrent = pPort->Group[i].PirMax;
            }
        }
#endif
    }

    /* Find out how much room is left in output queue */
    if (pPort->DestThrottleThresh)
    {
        OutputSpaceAvail = pPort->DestThrottleThresh;
        if (pPort->fByteDestThrottle)
        {
            OutputSpaceAvail -= Qmss_getQueueByteCount (pPort->DestQueueNumber);
        }
        else
        {
            OutputSpaceAvail -= Qmss_getQueueEntryCount (pPort->DestQueueNumber);
        }
        // No room in output queue */
        if (OutputSpaceAvail <= 0)
        {
            return PacketScheduled;
        }
    }

    // Assume all groups have packets pending until we find out otherwise
    PacketPendingMask = 0xffffffff;

    //
    // Schedule each logic group's CIR, while also ensuring that the
    // physical port's CIR is not violated.
    // If the physical port has no credit quit out of the scheduler entirely
    if( pPort->CirCurrent <= 0 )
        return PacketScheduled;

    // Foreground task can exit once all packets are sent either because
    // the input queues are empty, or we ran out of group CIR, or we run 
    // out of port CIR.
    do
    {
        fPacketsSent = 0;
        
        for( i=0; i<pPort->GroupCount; i++ )
        {
#ifdef TEST_MULTI_GROUP
            if(pPort->Group[i].CirCurrent > 0)
            {
#endif
                // Attempt to schedule a packet
                BytesUsed = qos_model_group_scheduler ( pPort, &pPort->Group[i] );

                // If no packet scheduled, clear the pending mask
                if( !BytesUsed )
                {
                    PacketPendingMask &= ~(1<<i);
                }
                else
                {
                    // Use packet or byte count, depending on configuration
                    if( pPort->fByteCirCredits )
                        CirCreditUsed = (BytesUsed + pPort->OverheadBytes - pPort->RemoveBytes) << QMSS_QOS_SCHED_BYTES_SCALE_SHIFT;
                    else
                        CirCreditUsed = 1 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;

                    // Here we have a packet, so deduct the credit
                    pPort->CirCurrent          -= CirCreditUsed;
#ifdef TEST_MULTI_GROUP
                    pPort->Group[i].CirCurrent -= CirCreditUsed;
                    pPort->Group[i].PirCurrent -= CirCreditUsed;
#endif

                    fPacketsSent = 1;
                    PacketScheduled++;

                    // If the physical port has no credit quit out of the scheduler entirely
                    if( pPort->CirCurrent <= 0 )
                        return PacketScheduled;

                    // See if we used up output space
                    if (model_phys_port_update_output_space (pPort, &OutputSpaceAvail, BytesUsed) == 0)
                    {
                        return PacketScheduled;
                    }
                }
#ifdef TEST_MULTI_GROUP
            }
#endif
        }
    } while (fPacketsSent);
    //
    // Schedule each logic group's PIR in a WRR fashion while the
    // physical port's CIR is not violated.
    //
#ifdef TEST_MULTI_GROUP
    while(pPort->CirCurrent > 0)
    {
        // If there are no groups left with PIR group credit and packets, then we're done
        if( !(PirCreditMask & PacketPendingMask) )
            return PacketScheduled;

        // If all groups with WRR credit remaining are empty, add WRR credit
        while( ! (PirCreditMask & pPort->WrrCreditMask & PacketPendingMask))
        {
            // Reset credits
            for(i=0; i<pPort->GroupCount; i++)
            {
                pPort->Group[i].WrrCurrentCredit += pPort->Group[i].WrrInitialCredit;

                if (pPort->Group[i].WrrCurrentCredit > (pPort->Group[i].WrrInitialCredit << 1))
                    pPort->Group[i].WrrCurrentCredit = (pPort->Group[i].WrrInitialCredit << 1);
                      
                if ((pPort->Group[i].WrrCurrentCredit > 0) || (! pPort->Group[i].WrrInitialCredit))
                    pPort->WrrCreditMask |= (1<<i);
            }
            // while loop will always terminate because PirCreditMask & PacketPendingMask check 
        }

        // If this group has PIR credit, WRR credit, and packets pending, then schedule a packet
        if( (PirCreditMask & pPort->WrrCreditMask & PacketPendingMask) & (1<<pPort->NextGroup) )
        {
            // Attempt to schedule a packet
            BytesUsed = qos_model_group_scheduler ( pPort, &pPort->Group[pPort->NextGroup] );

            // If no packet scheduled, clear the pending mask
            if( !BytesUsed )
                PacketPendingMask &= ~(1<<pPort->NextGroup);
            else
            {
                // Use packet or byte count, depending on configuration
                if( pPort->fByteCirCredits )
                    CirCreditUsed = (BytesUsed + pPort->OverheadBytes - pPort->RemoveBytes) << QMSS_QOS_SCHED_BYTES_SCALE_SHIFT;
                else
                    CirCreditUsed = 1 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;

                // Use packet or byte count, depending on configuration
                if( pPort->fByteWrrCredits )
                    WrrCreditUsed = (BytesUsed + pPort->OverheadBytes - pPort->RemoveBytes) << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT;
                else
                    WrrCreditUsed = 1 << QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT;

                // Deduct the PIR/CIR credit
                pPort->CirCurrent -= CirCreditUsed;
                pPort->Group[pPort->NextGroup].PirCurrent -= CirCreditUsed;

                // We also deduct the WRR credit
                pPort->Group[pPort->NextGroup].WrrCurrentCredit -= WrrCreditUsed;

                // Clear the group's PIR credit mask if we depleted the PIR credit
                if( pPort->Group[pPort->NextGroup].PirCurrent <= 0 )
                    PirCreditMask &= ~(1<<pPort->NextGroup);

                // Clear the group's WWR credit mask if we depleted the WRR credit
                if( pPort->Group[pPort->NextGroup].WrrCurrentCredit <= 0 )
                    pPort->WrrCreditMask &= ~(1<<pPort->NextGroup);

                PacketScheduled++;

                // See if we used up output space
                if (model_phys_port_update_output_space (pPort, &OutputSpaceAvail, BytesUsed) == 0)
                {
                    return PacketScheduled;
                }
            }
        }

        // Move on to the next group
        pPort->NextGroup++;
        if( pPort->NextGroup == pPort->GroupCount )
            pPort->NextGroup = 0;
    }
#endif // TEST_MULTI_GROUP
    return PacketScheduled;
}

/*****************************************************************************
 * Model of the qos foreground scheduler which is used to check that the
 * firmware returns correct results.  This runs until packetsRemaining are
 * scheduled.  Returns timer ticks used to schedule all the packets
 * **************************************************************************/
uint32_t qos_model (Qmss_QosSchedPortCfg *cfg, testState_t *state, int packetsRemaining)
{
    int i;
    int port, group, queue;
    int queueIdx = 0;
    int loopLimit;
    uint32_t start_ticks;

    /* Assign queues */
    loopLimit = TEST_FULL_MAX_PHYS_PORTS;
    if (loopLimit > TEST_PORTS)
    {
        loopLimit = TEST_PORTS;
    }
    for (port = 0; port < loopLimit; port++)
    {
        for (group = 0; group < TEST_MAX_FULL_LOG_GROUPS; group++)
        {
            for (queue = 0; queue < QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP; queue++)
            {
                qos_model_ports[port].Group[group].Queue[queue].QueueNumber = state->fwQHnds[queueIdx++];
            }
        }
    }
    loopLimit = TEST_MAX_PHYS_PORTS;
    if (loopLimit > TEST_PORTS)
    {
        loopLimit = TEST_PORTS;
    }

    for (port = TEST_FULL_MAX_PHYS_PORTS; port < loopLimit; port++)
    {
        for (group = 0; group < QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS; group++)
        {
            for (queue = 0; queue < QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP; queue++)
            {
                qos_model_ports[port].Group[group].Queue[queue].QueueNumber = state->fwQHnds[queueIdx++];
            }
        }
    }

    /* Copy config */
    for (port = 0; port < TEST_MAX_PHYS_PORTS; port++)
    {
        modelPort *pPort               = &qos_model_ports[port];
        Qmss_QosSchedPortCfg *pPortCfg = &cfg[port];
        pPort->fByteWrrCredits         = (pPortCfg->wrrType == Qmss_QosSchedAcctType_BYTES) ? 1 : 0;
        pPort->fByteCirCredits         = (pPortCfg->cirType == Qmss_QosSchedAcctType_BYTES) ? 1 : 0;
        pPort->fByteCongest            = (pPortCfg->congestionType == Qmss_QosSchedAcctType_BYTES) ? 1 : 0;
        pPort->fByteDestThrottle       = (pPortCfg->outThrotType == Qmss_QosSchedAcctType_BYTES) ? 1 : 0;
        pPort->CirIteration            = pPortCfg->cirIteration;
        pPort->CirCurrent              = 0;
        pPort->CirMax                  = pPortCfg->cirMax;
        pPort->GroupCount              = pPortCfg->groupCount;
        pPort->WrrCreditMask           = 0;
        pPort->NextGroup               = 0;
        pPort->DestQueueNumber         = Qmss_getQueueHandle (pPortCfg->outputQueue);
        pPort->OverheadBytes           = pPortCfg->overheadBytes;
        pPort->RemoveBytes             = pPortCfg->removeBytes;
        pPort->DestThrottleThresh      = pPortCfg->outThrotThresh;
        pPort->LastTimerTicks          = (uint8_t)qos_model_timer_ticks;
        for (group = 0; group < TEST_MAX_FULL_LOG_GROUPS; group++)
        {
            modelGroup *pGroup               = &pPort->Group[group];
            Qmss_QosSchedGroupCfg *pGroupCfg = &pPortCfg->group[group];
#ifdef TEST_MULTI_GROUP
            pGroup->CirIteration             = pGroupCfg->cirIteration;
            pGroup->PirIteration             = pGroupCfg->pirIteration;
            pGroup->CirCurrent               = 0;
            pGroup->PirCurrent               = 0;
            pGroup->CirMax                   = pGroupCfg->cirMax;
            pGroup->PirMax                   = pGroupCfg->pirMax;
            pGroup->WrrInitialCredit         = pGroupCfg->wrrInitialCredit;
            pGroup->WrrCurrentCredit         = 0;
#endif
            pGroup->QueueCount               = pGroupCfg->totQueueCount;
            pGroup->SPCount                  = pGroupCfg->spQueueCount;
            pGroup->RRCount                  = pGroupCfg->wrrQueueCount;
            pGroup->NextQueue                = pGroupCfg->spQueueCount;
            pGroup->WrrCreditMask            = 0;
            for (queue = 0; queue < QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP; queue++)
            {
                modelQueue *pQueue               = &pGroup->Queue[queue];
                Qmss_QosSchedQueueCfg *pQueueCfg = &pGroupCfg->Queue[queue];
                pQueue->WrrInitialCredit         = pQueueCfg->wrrInitialCredit;
                pQueue->WrrCurrentCredit         = 0;
                pQueue->CongestionThresh         = pQueueCfg->congestionThresh;
                pQueue->PacketsForwarded         = 0;
                pQueue->BytesForwarded           = 0;
            }
        }
    }

    start_ticks = qos_model_timer_ticks;
    while (packetsRemaining)
    {
        // Schedule packets from all active physical ports
        qos_model_timer_ticks++;

        for(i=0; i<TEST_PORTS; i++)
        {
            packetsRemaining -= qos_model_port_scheduler (&qos_model_ports[i]);
        }
    }

    return qos_model_timer_ticks - start_ticks;
}


/*****************************************************************************
 * This function reads back configuration and compares it to portCfg
 *****************************************************************************/
void port_readback (int port, Qmss_QosSchedPortCfg *portCfg)
{
#ifdef _TMS320C6X
	uint32_t               corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	uint32_t               corenum = 0;
#endif
    Qmss_QosSchedPortCfg portCfgRB;
    Qmss_Result          result;
    int                  error = 0;
    int                  group, size;
    char                *orig_p, *new_p;

    /* Load back the configuration it should read back the same */
    memset (&portCfgRB, 0, sizeof(portCfgRB));
    if ((result = Qmss_getCfgQosSchedPortSubSys (SUBSYS, PDSPID, port, &portCfgRB)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        errorCount++;
        System_printf("Core %d : Failed to query port %d's config: %d\n", corenum, port, result);
    } 
    orig_p = (char *)portCfg;
    new_p = (char *)&portCfgRB;
    size = offsetof(Qmss_QosSchedPortCfg, group);
    /* Compare the port config prior to groups */
    error = error || memcmp (orig_p, new_p, size);
    /* Compare the port config after the groups */
    orig_p += offsetof(Qmss_QosSchedPortCfg, group) + sizeof(portCfg->group);
    new_p += offsetof(Qmss_QosSchedPortCfg, group) + sizeof(portCfg->group);
    size = sizeof (*portCfg) - offsetof(Qmss_QosSchedPortCfg, group) - sizeof(portCfg->group);
    error = error || memcmp (orig_p, new_p, size);
    /* Compare the configured groups except for the queues */
    for (group = 0; group < portCfg->groupCount; group++)
    {
        orig_p = (char *)&portCfg->group[group];
        new_p = (char *)&portCfgRB.group[group];
        size = offsetof(Qmss_QosSchedGroupCfg, Queue);
        error = error || memcmp (orig_p, new_p, size);
        /* Compare the configured queues */
        orig_p = (char *)portCfg->group[group].Queue;
        new_p = (char *)portCfgRB.group[group].Queue;
        size = sizeof(Qmss_QosSchedQueueCfg) * portCfg->group[group].totQueueCount;
        error = error || memcmp (orig_p, new_p, size);
    }

    if (error)
    {
        errorCount++;
        System_printf("Core %d : Didn't read back port %d's configuration correctly\n", corenum, port);
    }
}

/*****************************************************************************
 * This function sets the port configuration then reads it back to
 * ensure same result is returned.
 *****************************************************************************/
void port_config (int port, Qmss_QosSchedPortCfg *portCfg)
{
#ifdef _TMS320C6X
	uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	uint32_t             corenum = 0;
#endif
    Qmss_Result result;

    /* Set the configuration */
    if ((result = Qmss_putCfgQosSchedPortSubSys (SUBSYS, PDSPID, port, portCfg)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        errorCount++;
        System_printf("Core %d : Failed to set port %d's config: %d\n", corenum, port, result);
    } 

    port_readback (port, portCfg);
}

/*****************************************************************************
 * This functin puts TEST_DESCS on each of TEST_QUEUES pointed to by 
 * state->fwQHnds.
 *****************************************************************************/
int distribute_packets (testState_t *state, Qmss_QosSchedPortCfg *portCfg, 
                        int numDescsPerQueue, uint32_t *portTimestamps)
{
#ifdef _TMS320C6X
	uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	uint32_t             corenum = 0;
#endif
    Qmss_Queue dropQNum = Qmss_getQueueNumber (state->dropQHnd);
    uint32_t   port, group, queue, descNum;
    uint32_t   groupCount, queueCount;
    int        absQueueNum;
    int        totalDescs = 0;

    for (descNum = 0; descNum < numDescsPerQueue; descNum++)
    {
        absQueueNum = 0;
        for (port = 0; port < TEST_PORTS; port++)
        {
#if TEST_FULL_MAX_PHYS_PORTS > 0
            if (port >= TEST_FULL_MAX_PHYS_PORTS) 
            {
                groupCount = QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS;
            }
            else
            {
                groupCount = TEST_MAX_FULL_LOG_GROUPS;
            }
#else
            groupCount = QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS;
#endif
            for (group = 0; group < groupCount; group++)
            {
                queueCount = portCfg[port].group[group].totQueueCount;
                for (queue = 0; queue < queueCount; queue++)
                {
                    uint32_t *desc;
                    desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (state->freeQ));
                    if (! desc) 
                    {
                        System_printf ("Core %d : failed to pop a free desc\n", corenum);
                        errorCount++;
                        break;
                    }
                    /* Write unique ID onto the descriptor for tracking it */
                    /* fw doesn't use the first word of the descriptor */
                    desc[DESC_ID_OFFSET] = make_descID (port, group, queue, descNum);
                    /* Only set the return queue as if the descriptor is CPPI and
                     * set the descriptor type.  Otherwise
                     * QoS doesn't use any of the fields, so don't set them.
                     * If packets are sent to a CPPI destination such as ethernet
                     * then set the CPPI as if it were going directly to ethernet.
                     */
                    Cppi_setReturnQueue (Cppi_DescType_MONOLITHIC, (Cppi_Desc *)desc, dropQNum);
                    Cppi_setDescType ((Cppi_Desc *)desc, (Cppi_DescType_MONOLITHIC));
                    Qmss_queuePush (state->fwQHnds[absQueueNum], desc, QOS_DATA_PACKET_SIZE, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
                    absQueueNum++;
                    totalDescs++;
                    if (portTimestamps && (descNum == 0)) 
                    {
#ifdef _TMS320C6X
                    	portTimestamps[port] = TSCL;
#else
                    	portTimestamps[port] = utilReadTime32();
#endif
                    }
                }
#if TEST_FULL_MAX_PHYS_PORTS > 0
                if (port >= TEST_FULL_MAX_PHYS_PORTS)
                {
                    absQueueNum += QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP - queueCount;
                }
                else
                {
                    absQueueNum += QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP - queueCount;
                }
#else
                absQueueNum += QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP - queueCount;
#endif
            }
        }
    }
    return totalDescs;
}

/*****************************************************************************
 * This function sets up the QoS such that no credits are given, such that
 * no packets will drain out the output queue.  Instead, only the congestion
 * thresholds are set, such that correct congestion drop behavior can be
 * tested
 *****************************************************************************/
void congestion_test (testState_t *state, Qmss_QosSchedAcctType type)
{
    Qmss_Result          result;
    int                  port, group, queue;
    int                  descNum;
#ifdef _TMS320C6X
    uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    uint32_t             corenum = 0;
#endif
    int                  absQueue;
    uint32_t             startTime, endTime;
    int                  expectedDisableDrops, actualDisableDrops;
    int                  expectedDiscards;
    Qmss_QosSchedPortCfg portCfg[TEST_PORTS];

    System_printf("Core %d: starting congestion (dropping) test using %s units\n", 
                  corenum, string_type (type));
    expectedDiscards = 0;
    memset (portCfg, 0, sizeof(portCfg));
    for (port = 0; port < TEST_PORTS; port++)
    {
        if ((result = Qmss_disableQosSchedPortSubSys (SUBSYS, PDSPID, port)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
        {
            System_printf ("Core %d : failed to disable QoS port %d: %d\n", corenum, port, result);
            errorCount++;
        }
        portCfg[port].wrrType = type;
        portCfg[port].cirType = type;
        portCfg[port].congestionType = type;
        portCfg[port].outThrotType = type;
        portCfg[port].cirIteration = 0; // so we can test drop feature
        portCfg[port].cirMax = 10 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
        portCfg[port].overheadBytes = QOS_DATA_PACKET_OVHD;
        portCfg[port].removeBytes = 0;
        portCfg[port].outThrotThresh = 0;
        if (port >= TEST_FULL_MAX_PHYS_PORTS) 
        {
            portCfg[port].groupCount = QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS;
        }
        else
        {
            portCfg[port].groupCount = TEST_MAX_FULL_LOG_GROUPS;
        }
        portCfg[port].outputQueue = Qmss_getQueueNumber (state->qosOutQHnd);
        for (group = 0; group < portCfg[port].groupCount; group++)
        {
            portCfg[port].group[group].cirIteration = 0; // so we can test drop feature
            portCfg[port].group[group].pirIteration = 0; // so we can test drop feature
#ifndef TEST_DROP_SCHED
            portCfg[port].group[group].cirMax = 10 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
            portCfg[port].group[group].pirMax = 10 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
#else
            portCfg[port].group[group].cirMax = 0 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
            portCfg[port].group[group].pirMax = 0 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
#endif
            portCfg[port].group[group].wrrInitialCredit = 0; // so we can test drop feature
            if (port >= TEST_FULL_MAX_PHYS_PORTS) 
            {
                portCfg[port].group[group].totQueueCount = 4;
                portCfg[port].group[group].spQueueCount = 4;
                portCfg[port].group[group].wrrQueueCount = 0;
            } 
            else 
            {
                portCfg[port].group[group].totQueueCount = 8;
                portCfg[port].group[group].spQueueCount = 2;
                portCfg[port].group[group].wrrQueueCount = 5;
            }
            for (queue = 0; queue < portCfg[port].group[group].totQueueCount; queue++)
            {
                portCfg[port].group[group].Queue[queue].wrrInitialCredit = 0; // so we can test drop feature
                portCfg[port].group[group].Queue[queue].congestionThresh = port + 1;
                if (type == Qmss_QosSchedAcctType_BYTES)
                {
                    portCfg[port].group[group].Queue[queue].congestionThresh *= QOS_DATA_PACKET_SIZE;
                }
                expectedDiscards += PROFILE_DESCS - (port + 1);
            }
        }

        /* set and check the configuration */
        port_config (port, &portCfg[port]);
    }

    /* Put some packets in each input queue */
    distribute_packets (state, portCfg, PROFILE_DESCS, NULL);

    /* Enable the port */
    for (port = 0; port < TEST_PORTS; port++)
    {
        if ((result = Qmss_enableQosSchedPortSubSys (SUBSYS, PDSPID, port)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
        {
            System_printf ("Core %d : failed to enable QoS port %d: %d\n", corenum, port, result);
            errorCount++;
        }
    }

    /* Start the clock */
#ifdef _TMS320C6X
    startTime = TSCL;
#else
    startTime  = utilReadTime32();
#endif

    /* Timestamp arrival of each descriptor */
    for (descNum = 0; descNum < expectedDiscards; )
    {
        uint32_t *desc;
        desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePopRaw(state->dropQHnd));

        if (desc) 
        {
#ifdef _TMS320C6X
        	timestamps[descNum].timestamp = TSCL;
#else
        	timestamps[descNum].timestamp = utilReadTime32();
#endif
            timestamps[descNum].descPtr   = desc;
            descNum ++;
            Qmss_queuePushDesc (state->freeQ, desc);
        }
    }
#ifdef _TMS320C6X
    endTime = TSCL;
#else
    endTime = utilReadTime32();
#endif
    System_printf ("Core %d: discarded %d descriptors across %d queues "
                   "in %d cycles (%d cycles per descriptor)\n",
                   corenum, 
                   expectedDiscards, 
                   TEST_QUEUES,
                   endTime - startTime, 
                   (endTime - startTime) / (expectedDiscards));

    /* Check that the correct number of descriptors were pulled from each queue */
    absQueue = 0;
    expectedDisableDrops = 0;
    for (port = 0; port < TEST_PORTS; port++)
    {
        for (group = 0; group < portCfg[port].groupCount; group++)
        {
            for (queue = 0; queue < portCfg[port].group[group].totQueueCount; queue++)
            {

               uint32_t count = Qmss_getQueueEntryCount (state->fwQHnds[absQueue]);
               uint32_t expectCount = portCfg[port].group[group].Queue[queue].congestionThresh;
               uint32_t expectDrop;
               Qmss_QosSchedStats      stats;
               if (type == Qmss_QosSchedAcctType_BYTES)
               {
                   expectCount /= QOS_DATA_PACKET_SIZE;
               }
               expectDrop = PROFILE_DESCS - expectCount;
               if (count != expectCount) 
               {
                   System_printf ("Core %d : expect %d descs but found %d : %d\n", 
                                  corenum, expectCount, count);
                   errorCount++;
               }
               /* Move remaining descriptors back to free queue */
               if (port != 1) 
               {
                   /* Port 1 is used to test discard on disable */
                   queue_divert_and_check (state->fwQHnds[absQueue], state->freeQ);
               }
               else 
               {
                   expectedDisableDrops += expectCount;
               }
               absQueue++;
               /* Check the stats */
               Qmss_getQosSchedStatsSubSys (SUBSYS, PDSPID, &stats, port, group, queue,
                                      QMSS_QOS_SCHED_STATS_DISCARDED_BYTES |
                                      QMSS_QOS_SCHED_STATS_DISCARDED_PACKETS);
               if (stats.bytesForwarded || stats.packetsForwarded)
               {
                   System_printf ("Core %d: found forwarded stats when not expected: %d %d\n", 
                                  corenum, (uint32_t)stats.bytesForwarded, stats.packetsForwarded);
                   errorCount++;
               }
               if (stats.packetsDiscarded != expectDrop)
               {
                   System_printf ("Core %d: found wrong pkt discard stats: %d %d\n", 
                                  corenum, stats.packetsDiscarded, expectDrop);
                   errorCount++;
               }
               if (stats.bytesDiscarded != (expectDrop * QOS_DATA_PACKET_SIZE))
               {
                   System_printf ("Core %d: found wrong pkt discard stats: %d %d\n", 
                                  corenum, (uint32_t)stats.bytesDiscarded, 
                                  expectDrop * QOS_DATA_PACKET_SIZE);
                   errorCount++;
               }
            }
        }
    }    
    
    /* Disable port 1 */
    if (TEST_PORTS > 1)
    {
        if ((result = Qmss_disableQosSchedPortSubSys (SUBSYS, PDSPID, 1)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
        {
            System_printf ("Core %d : failed to disable QoS port %d: %d\n", corenum, port, result);
            errorCount++;
        }
        /* Re-enable port 1 */
        if ((result = Qmss_enableQosSchedPortSubSys (SUBSYS, PDSPID, 1)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
        {
            System_printf ("Core %d : failed to enable QoS port %d: %d\n", corenum, port, result);
            errorCount++;
        }
        /* Now the drop Q should have the port 1 packets */
        actualDisableDrops = Qmss_getQueueEntryCount (state->dropQHnd);
        queue_divert_and_check (state->dropQHnd, state->freeQ);
        if (actualDisableDrops != expectedDisableDrops)
        {
            System_printf ("Core %d : On port disable, found %d drops expected %d\n", 
                           corenum, actualDisableDrops, expectedDisableDrops);
            errorCount++;
        }
    }
    
    for (port = 0; port < TEST_PORTS; port++)
    {
        /* Configuration shouldn't have changed during test (check memory corruption) */
        port_readback (port, &portCfg[port]);
    }
}

/*****************************************************************************
 * This function sends portCfg to each of TEST_PORTS, then puts descriptors
 * on each queue, and records the results and determines the data rate on
 * each port.  If model is set, the resulting sequence is compared to the
 * C model.
 *
 * While the port is enabled inside this function, it is possible to use
 * this function to test reconfiguring running ports since it never
 * disables the port.
 *****************************************************************************/
void transfer_test (const char *description, testState_t *state, Qmss_QosSchedPortCfg *portCfg, 
                    int numDescsPerQueue, int model, int checkRate)
{
    Qmss_Result          result;
    int                  port, group, queue;
    int                  descNum;
#ifdef _TMS320C6X
	uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	uint32_t             corenum = 0;
#endif
    uint32_t             startTime, endTime;
    uint32_t             portDescNum[TEST_PORTS];
    uint32_t             portStartTimes[TEST_PORTS];
    uint32_t             portStopTimes[TEST_PORTS];
    int                  totalDescs;
    uint32_t             model_ticks = 0;

    memset (portDescNum, 0, sizeof(portDescNum));
    System_printf("Core %d: starting transfer test: %s\n", corenum, description);

    if (model) 
    {
        /* Comparing to model requires port to be disabled to get good timing */
        for (port = 0; port < TEST_PORTS; port++)
        {
            if ((result = Qmss_disableQosSchedPortSubSys (SUBSYS, PDSPID, port)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
            {
                System_printf ("Core %d : failed to disable QoS port %d: %d\n", corenum, port, result);
                errorCount++;
            }
        }
        /* Distribute the packets for the model */
        totalDescs = distribute_packets (state, portCfg, numDescsPerQueue, NULL);
        /* Run the model */
        model_ticks = qos_model (portCfg, state, totalDescs);
        /* Drain the output queue and record the sequence */
        /* The sequence numbers are sorted by port, since the fw isn't guaranteed
         * to start all ports at the same time */
        for (descNum = 0; descNum < totalDescs; descNum++)
        {
            uint32_t *desc;
            desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (state->qosOutQHnd));
            if (desc) 
            {
                uint32_t descID = desc[DESC_ID_OFFSET];
                uint32_t port = break_descID_port (descID);
                expectedSeqn[port][portDescNum[port]++] = descID;
                Qmss_queuePushDesc (state->freeQ, desc);
            }
            else
            {
                System_printf ("Core %d : Model failed (didn't move enough descs %d out of %d)\n", 
                               corenum, descNum, totalDescs);
                errorCount++;
                break;
            }
        }
    }

    /* Send the configuration */
    for (port = 0; port < TEST_PORTS; port++)
    {
        /* set and check the configuration */
        port_config (port, &portCfg[port]);
    }

    /* Put some packets in each input queue */
    if (! model)
    {
        /* The port is running now, so start timing while distributing */
#ifdef _TMS320C6X
        startTime = TSCL;
#else
        startTime = utilReadTime32();
#endif
    }

    totalDescs = distribute_packets (state, portCfg, numDescsPerQueue, portStartTimes);

    if (model)
    {
        /* The port wasn't running during distribution, so start timer just
         * before the port is enabled */
#ifdef _TMS320C6X
        startTime = TSCL;
#else
        startTime = utilReadTime32();
#endif
    }
    /* Enable the port */
    if (model) 
    {
        /* Comparing to model requires port to be enabled to get good timing */
        for (port = 0; port < TEST_PORTS; port++)
        {
            if ((result = Qmss_enableQosSchedPortSubSys (SUBSYS, PDSPID, port)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
            {
                System_printf ("Core %d : failed to enable QoS port %d: %d\n", corenum, port, result);
                errorCount++;
            }
#ifdef _TMS320C6X
            portStartTimes[port] = TSCL;
#else
            portStartTimes[port] = utilReadTime32();
#endif
        }
    }

    /* Timestamp arrival of each descriptor */
    for (descNum = 0; descNum < totalDescs; )
    {
        uint32_t *desc;
        desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (state->qosOutQHnd));
        if (desc) 
        {
#ifdef _TMS320C6X
            timestamps[descNum].timestamp = TSCL;
#else
            timestamps[descNum].timestamp = utilReadTime32();
#endif
            timestamps[descNum].descPtr   = desc;
            descNum ++;
            Qmss_queuePushDesc (state->freeQ, desc);
        }
    }

#ifdef _TMS320C6X
        endTime  = TSCL;
#else
        endTime  = utilReadTime32();
#endif

    System_printf ("Core %d: moved %d descriptors across %d queues "
                   "in %d cycles (%d cycles per descriptor)\n",
                   corenum, 
                   totalDescs, 
                   TEST_QUEUES,
                   endTime - startTime, 
                   (endTime - startTime) / (totalDescs));

    if (model)
    {
        int fail = 0;
        memset (portDescNum, 0, sizeof(portDescNum));
        /* Check that the descriptors came in the right order */
        for (descNum = 0; descNum < totalDescs; descNum++)
        {
            uint32_t descID = timestamps[descNum].descPtr[DESC_ID_OFFSET];
            uint32_t port = break_descID_port (descID);
            uint32_t expectedDescID = expectedSeqn[port][portDescNum[port]++];
            if (expectedDescID != descID)
            {
                System_printf ("Core %d: port %d idx %d, portDescNum %d, found ID 0x%08x expected ID 0x%08x\n", 
                               corenum, port, descNum, portDescNum[port] - 1, descID, expectedDescID);

                fail = 1;
            }
        }
        if (fail)
        {
            System_printf("Core %d: model order doesn't match firmware\n", corenum);
            errorCount++;
        }
    }

    /* Find the first and last timestamp for each port and number of descs */
    memset (portDescNum, 0, sizeof(portDescNum));
    for (descNum = 0; descNum < totalDescs; descNum++)
    {
        uint32_t descID = timestamps[descNum].descPtr[DESC_ID_OFFSET];
        uint32_t port = break_descID_port (descID);
        portStopTimes[port] = timestamps[descNum].timestamp;
        portDescNum[port]++;
    }

    /* Now we know how long it took between first and last desc */
    if (checkRate)
    {
        uint32_t totalActualCycles = 0;
        for (port = 0; port < TEST_PORTS; port++)
        {
            uint32_t cyclesPerTick = QOS_TIMER_CONFIG * 2 * 3;
            uint32_t expectedTicks, cirMaxTicks, expectedCycles, actualCycles;
            uint32_t allowedError, actualError;
            if (portCfg[port].cirType == Qmss_QosSchedAcctType_BYTES) 
            {
               /* Number of packets - 1, since one packet can go out without any available credit
                * since credit goes negative */
               expectedTicks = (portDescNum[port] - 1) * 
                               ((QOS_DATA_PACKET_SIZE + QOS_DATA_PACKET_OVHD) << QMSS_QOS_SCHED_BYTES_SCALE_SHIFT) /
                               portCfg[port].cirIteration;
            }
            else
            {
               /* Number of packets - 1, since one packet can go out without any available credit
                * since credit goes negative */
               expectedTicks = (portDescNum[port] - 1) *
                               (1 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT) /
                               portCfg[port].cirIteration;
            }
            cirMaxTicks = portCfg[port].cirMax / portCfg[port].cirIteration;
            actualCycles = portStopTimes[port] - portStartTimes[port];
            expectedCycles = expectedTicks * cyclesPerTick;
            /* Two ticks of uncertainty */
            allowedError = 2 * cyclesPerTick;
            if (! model)
            {
                /* Assume cirMax accumulated before starting */
                allowedError += cirMaxTicks * cyclesPerTick;
            } 
            actualError = expectedCycles - actualCycles;
            System_printf ("Core %d: port %d took %d cycles to move %d descs ",
                           corenum, port, actualCycles, 
                           portDescNum[port]);
            System_printf ("expected %d, error=%d, allowed=%d) ", 
                           expectedCycles, actualError, allowedError);
#ifndef SIMULATOR_SUPPORT
            if (abs(actualError) > allowedError) 
            {
                System_printf("(*** FAIL ***)");
                errorCount++;
            }
            else
            {
                System_printf("(*** PASS ***)");
            }
            System_printf("\n");
#else
            System_printf("(*** CHECK SKIPPED ON SIMULATOR ***)\n");
#endif
            if (actualCycles > totalActualCycles)
            {
                totalActualCycles = actualCycles;
            }
        }
        if (model)
        {
            uint32_t modelCycles = model_ticks * QOS_TIMER_CONFIG * 2 * 3;
            uint32_t error = abs(modelCycles - totalActualCycles);
            uint32_t allowedError = 2 * QOS_TIMER_CONFIG * 2 * 3;
            System_printf ("Core %d: firmware took %d cycles; model took %d; error %d; allowed %d",
                           corenum, totalActualCycles, modelCycles, error, allowedError);
#ifndef SIMULATOR_SUPPORT
            if (error > allowedError)
            {
                System_printf("(*** FAIL ***)");
                errorCount++;
            }
            else
            {
                System_printf("(*** PASS ***)");
            }
#else
            System_printf("(*** Timing check skipped on simulator ***)");
#endif
            System_printf("\n");
        }
    }

    /* Check the stats */
    for (port = 0; port < TEST_PORTS; port++)
    {
        for (group = 0; group < portCfg[port].groupCount; group++)
        {
#ifndef TEST_DROP_SCHED
            /* Test group stats */
            Qmss_QosSchedStats groupStats[QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP];
            int nStats = QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP;

            if ((result = Qmss_getQosSchedGroupStatsSubSys 
                            (SUBSYS, PDSPID, groupStats, &nStats, port, group, 0)) != 
                QMSS_QOS_SCHED_RETCODE_SUCCESS)
            {
               System_printf ("Core %d: group stat query failed: %d\n", 
                              corenum, result);
               errorCount++;
            }
            if (nStats !=  portCfg[port].group[group].totQueueCount)
            {
               System_printf ("Core %d: port %d group %d got back wrong # of group stats %d %d\n",
                              corenum, port, group, nStats, (int)portCfg[port].group[group].totQueueCount);
               errorCount++;
            }
#endif
                                                       
            for (queue = 0; queue < portCfg[port].group[group].totQueueCount; queue++)
            {
               Qmss_QosSchedStats      stats;
               /* Check the stats */
               Qmss_getQosSchedStatsSubSys (SUBSYS, PDSPID, &stats, port, group, queue,
                                            QMSS_QOS_SCHED_STATS_FORWARDED_BYTES |
                                            QMSS_QOS_SCHED_STATS_FORWARDED_PACKETS);
#ifndef TEST_DROP_SCHED
               /* Test group stats */
               if (memcmp (&stats, &groupStats[queue], sizeof(stats)))
               {
                   System_printf ("Core %d: group stat port %d group %d queue %d mismatch single queue stat\n", 
                                  corenum, port, group, queue);
                   errorCount++;
               }
#endif
               if (stats.bytesDiscarded || stats.packetsDiscarded)
               {
                   System_printf ("Core %d: found dropped stats when not expected: %d %d\n", 
                                  corenum, (uint32_t)stats.bytesDiscarded, stats.packetsDiscarded);
                   errorCount++;
               }
               if (stats.packetsForwarded != numDescsPerQueue)
               {
                   System_printf ("Core %d: port %d, group %d, queue %d, found wrong pkt forward stats: %d %d\n", 
                                  corenum, port, group, queue, stats.packetsForwarded, numDescsPerQueue);
                   errorCount++;
               }
               if (stats.bytesForwarded != (numDescsPerQueue * QOS_DATA_PACKET_SIZE))
               {
                   System_printf ("Core %d: port %d, group %d, queue %d, found wrong byte forward stats: %d %d\n", 
                                  corenum, port, group, queue, (uint32_t)stats.bytesForwarded, 
                                  numDescsPerQueue * QOS_DATA_PACKET_SIZE);
                   errorCount++;
               }
            }
        }
    }
    for (port = 0; port < TEST_PORTS; port++)
    {
        /* Configuration shouldn't have changed during test (check memory corruption) */
        port_readback (port, &portCfg[port]);
    }
}

/*****************************************************************************
 * Insert engine #idx into list based on updated head engine's nextTime
 *****************************************************************************/
void pkt_gen_reinsert_engine (int which)
{
    uint32_t *list = pktGenEngList.idx;
    /* Split table into 4 identical smaller tables for faster insert */
    int      tableBase = ((MAX_PKT_GEN_QUEUES + 3)/4) * which;
    int      tableEnd = pktGenEngList.numPresent;
    uint32_t headIdx = list[tableBase];
    uint64_t val = pktGenEngStates[headIdx].nextTime;
    uint32_t idx;

    if (tableEnd > (tableBase + ((MAX_PKT_GEN_QUEUES + 3)/4)))
    {
        tableEnd = tableBase + ((MAX_PKT_GEN_QUEUES + 3)/4);
    }

    /* Shift list and insert head back into correct location based on nextTime */
    for (idx = tableBase + 1; idx < tableEnd; idx++)
    {
        if (val > pktGenEngStates[list[idx]].nextTime)
        {
            list[idx - 1] = list[idx];
        }
        else 
        {
            break;
        }
    }
    list[idx - 1] = headIdx;
}

/*****************************************************************************
 * Drain output queue and update stats.  This requires checking if any 
 * descriptors are in queue before using queuePopDescSize, to avoid popping 
 * a bogus 0 from the size.  This would occur if the fw pushes a descriptor
 * in the queue, after the sw reads the size but before it reads the pointer.
 *
 * Note: this function gets optimized (-o3) regardless of project settings via
 * ti/drv/qmss/test/k2x/c66/bios/optOnlySendScenModel.txt.  This is required
 * to generate packets fast enough for some scenarios.
 *****************************************************************************/
int drain_scen_output (testState_t *state, int record_time)
{
    uint32_t *desc;
    uint32_t pktSize;
    uint32_t numDescs = Qmss_getQueueEntryCount (state->qosOutQHnd); 
    uint32_t outIdx;

    /* Can only pop with size if something is there, else can
     * get 0 for size and a pointer for the next desc! */
    if (numDescs)
    {
        Qmss_queuePopDescSize (state->qosOutQHnd, (void **)&desc, &pktSize);
        desc = (uint32_t *)QMSS_DESC_PTR(desc);
        if (! desc)
        {
#ifdef _TMS320C6X
	        uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	        uint32_t             corenum = 0;
#endif
            System_printf ("Core %d : failed to pop result, even though queue depth found\n", corenum);
            errorCount++;
            return 0;
        }
        outIdx = desc[DESC_ID_OFFSET] >> 20;
        pktGenEngStates[outIdx].packetsRx++;
        pktGenEngStates[outIdx].bytesRx += pktSize;
        if (record_time)
        {
            pktGenEngStates[outIdx].lastRxTime = read_timer64();
        }
        Qmss_queuePush (state->freeQ, desc, pktSize, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
        return 1;
    }
    return 0;
}

/*****************************************************************************
 * Execute use case scenarios which are defined in test_qosSchedScen.h.
 *
 * Each scenario provides a qos configuration, the packet stream 
 * parameters, and expected results.
 *
 * Note: this function gets optimized (-o3) regardless of project settings via
 * ti/drv/qmss/test/k2x/c66/bios/optOnlySendScenModel.txt.  This is required
 * to generate packets fast enough for some scenarios.
 *****************************************************************************/
void sendScenarioModel (testState_t *state, packetGenCfg *pktGen, uint64_t *startTime_p,
                        uint32_t proxyFlag)
{
#ifdef _TMS320C6X
	uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	uint32_t             corenum = 0;
#endif
    int gen, idx, genTbl;
    uint32_t         *desc;
    uint32_t          descNum = 0;
    pktGenEngState_t *genState;
    packetGenQCfg    *genCfg;
    uint64_t          startTime, genEndTime, sampTime, thisTime, nextPlayTime, backlogTime;
    uint32_t          pktSize;

    /* Generate the packets while simutaneosly checking the results */
    *startTime_p = startTime = read_timer64();

    pktGenEngList.numPresent = pktGen->numQCfgs;
#if defined(QOS_SCHED_FAST_SCENARIO) && !defined(TEST_DROP_SCHED)
    pktGen->genTime /= 100;
    pktGen->readTime /= 100;
#endif
    for (gen = 0; gen < pktGen->numQCfgs; gen++)
    {
        genState = &pktGenEngStates[gen];
        genCfg   = &pktGen->Qs[gen];
        if (genCfg->port != GENPORT_DROPSCHED) 
        {
            uint32_t queueIdx;
            int skipFullPorts, skipLitePorts;
            /* Find absolute queue index */
            skipFullPorts = genCfg->port;
            skipLitePorts = 0;
            if (genCfg->port > TEST_FULL_MAX_PHYS_PORTS)
            {
                skipLitePorts = skipFullPorts - TEST_FULL_MAX_PHYS_PORTS;
                skipFullPorts = TEST_FULL_MAX_PHYS_PORTS;
            }
            queueIdx  = skipFullPorts * TEST_MAX_FULL_LOG_GROUPS * QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP;
            queueIdx += skipLitePorts * QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS * QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
#if TEST_FULL_MAX_PHYS_PORTS > 0
            if (genCfg->port >= TEST_FULL_MAX_PHYS_PORTS)
            {
                queueIdx += QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP * genCfg->group;
            }
            else
            {
                queueIdx += QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP * genCfg->group;
            }
#else
            queueIdx += QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP * genCfg->group;
#endif
            queueIdx += genCfg->queue;
            genState->outputQ   = state->fwQHnds[queueIdx];
        }
        else
        {
#ifdef TEST_DROP_SCHED
            genState->outputQ = state->dropFwQHnds[genCfg->queue];
#else
            System_printf ("core %d: invalid config w/o drop sched\n", corenum);
            return;
#endif
        }
        genState->nextTime     = startTime;
        genState->deltaTime    = CORE_SPEED / genCfg->pps;
        genState->sizeIdx      = 0;
        genState->packetsRx    = 0;
        genState->bytesRx      = 0;
        genState->bytesTx      = 0;
        genState->packetsTx    = 0;
        genState->packetYanked = 0;
        pktGenEngList.idx[gen] = gen;

        /* Check that valid sizes are given */
        if (genCfg->sizeMode == packet_GEN_SIZE_MODE_RANDOM)
        {
            if ((genCfg->nSizes != 2) || 
                (genCfg->pktSize[0] > genCfg->pktSize[1]))
            {
                System_printf("core %d: found invalid size mode configuration for gen %d\n", corenum, gen);
            }
        }
    }

    /* Generate the packets while simutaneosly checking the results */
    genEndTime   = startTime + (uint64_t)(CORE_SPEED / 1000) * pktGen->genTime;
    sampTime     = startTime + (uint64_t)(CORE_SPEED / 1000) * pktGen->readTime;
    nextPlayTime = startTime;
    genState     = &pktGenEngStates[pktGenEngList.idx[0]];
    genCfg       = &pktGen->Qs[pktGenEngList.idx[0]];
    genTbl       = 0;
                
    while ( ((thisTime = read_timer64()) < genEndTime) && (thisTime < sampTime))
    {
        /* Should we send packet on first (soonest) engine? */
        if (thisTime > nextPlayTime)
        {
            backlogTime = thisTime - nextPlayTime;
            /* Get a free descriptor */
            desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (state->freeQ));
            if (! desc) 
            {
                System_printf ("Core %d : failed to pop a free desc\n", corenum);
                errorCount++;
                break;
            }
            /* Decide the "size" of the packet (no real data is DMAd) */
            if (genCfg->sizeMode == packet_GEN_SIZE_MODE_ROUND_ROBIN)
            {
                pktSize = genCfg->pktSize[genState->sizeIdx++];
                if (genState->sizeIdx >= genCfg->nSizes)
                {
                    genState->sizeIdx = 0;
                }
            }
            else
            {
                pktSize = (rand() % (genCfg->pktSize[1] - genCfg->pktSize[0])) + 
                              genCfg->pktSize[0];
            }
            /* Identify the packet */
            desc[DESC_ID_OFFSET] = (pktGenEngList.idx[genTbl * ((MAX_PKT_GEN_QUEUES + 3)/4)] << 20) | (descNum & 0xfffff);
            descNum++;
            /* Send the packet */
#ifdef TEST_DROP_SCHED
            if (proxyFlag)
            {
                Qmss_qosSchedDropSchedPushProxy (genState->outputQ, desc, pktSize, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
            }
            else
#endif
            {
                Qmss_queuePush (genState->outputQ, desc, pktSize, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
            }
            /* stats */
            genState->packetsTx ++;
            genState->bytesTx += pktSize;

            /* Reschedule the engine */
            genState->nextTime += genState->deltaTime;
            pkt_gen_reinsert_engine (genTbl);
            /* Determine which of 4 tables has earliest generator */
            for (idx = 0; idx < 4; idx++)
            {
                int tblBase = idx * ((MAX_PKT_GEN_QUEUES + 3)/4);
                int genIdx;
                if (tblBase > pktGenEngList.numPresent)
                {
                    break; /* sub table not use for this test */
                }
                genIdx = pktGenEngList.idx[tblBase];
                pktGenEngState_t *tempGenState = &pktGenEngStates[genIdx];
                if (idx == 0 || (tempGenState->nextTime < nextPlayTime))
                {
                    genState     = tempGenState;
                    genCfg       = &pktGen->Qs[genIdx];
                    genTbl       = idx;
                    nextPlayTime = genState->nextTime;
                }
            }
        }
        /* Try to drain the output */
        drain_scen_output (state, 1);
    }
    
    /* Wait for sample time */
    while ( ((thisTime = read_timer64()) < sampTime))
    {
        drain_scen_output (state, 1);
    }

    /* Remove any remaining input */
    for (gen = 0; gen < pktGen->numQCfgs; gen++)
    {
        uint32_t bytes, entries;
        queue_divert_and_check (pktGenEngStates[gen].outputQ, state->divQ);
        bytes = Qmss_getQueueByteCount (state->divQ);
        entries = Qmss_getQueueEntryCount (state->divQ);
        pktGenEngStates[gen].bytesTx -= bytes;
        pktGenEngStates[gen].packetsTx -= entries;
        pktGenEngStates[gen].packetYanked = entries;
        queue_divert_and_check (state->divQ, state->freeQ);
    }

    /* Drain rest of output */
    while (drain_scen_output(state, 0));

    if (backlogTime > 250000)
    {
        System_printf ("NOTE: packet generator got behind by 0x%x%08x cycles\n",
                       (uint32_t)(backlogTime >> 32), (uint32_t)backlogTime);
    }
}  /* sendScenarioModel */

/* Checks bytes stats */
void check_bytes (int gen, char *name, uint64_t fwd, uint64_t drop, uint64_t gentx, uint64_t genrx)
{
#ifdef _TMS320C6X
	uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	uint32_t             corenum = 0;
#endif
    uint64_t          actDrop = gentx - genrx;

    if (fwd != genrx)
    {
        System_printf ("Core %d : Stats mismatch for %s %d bytes fwd (found 0x%08x%08x, expect 0x%08x%08x)\n", 
                       corenum, name, gen, (uint32_t)(fwd >> 32), (uint32_t)fwd, 
                       (uint32_t)(genrx >> 32), (uint32_t)genrx);
        errorCount++;
    }
    if (drop != actDrop)
    {
        System_printf ("Core %d : Stats mismatch for %s %d bytes drop (found 0x%08x%08x, expect 0x%08x%08x)\n", 
                       corenum, name, gen, (uint32_t)(drop >> 32), (uint32_t)drop, 
                       (uint32_t)(actDrop >> 32), (uint32_t)actDrop);
        errorCount++;
    }
} /* check_bytes */

/* Checks packet stats */
void check_packets (int gen, char *name, uint32_t fwd, uint32_t drop, uint32_t gentx, uint32_t genrx, uint32_t yanked)
{
#ifdef _TMS320C6X
	uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	uint32_t             corenum = 0;
#endif
    uint32_t          actDrop = gentx - genrx;

    if (fwd != genrx)
    {
        System_printf ("Core %d : Stats mismatch for %s %d packets fwd (found %d, expect %d)\n", 
                       corenum, name, gen, fwd, genrx);
        errorCount++;
    }
    if (drop != actDrop)
    {
        /* If packets were yanked from under QoS during cleardown, allow
         * one error.  This is because the packet could have been
         * yanked in race with drop.  This packet will appear as 
         * 0 bytes, so byte drop stats don't need this special case (and
         * would therefore catch a real fencepost error).
         */
        if ((! yanked) || ((drop - 1) != actDrop)) {
            System_printf ("Core %d : Stats mismatch for %s %d packets drop (found %d, expect %d)\n", 
                           corenum, name, gen, drop, actDrop);
            errorCount++;
        }
    }
} /* check_packets */

/* Runs programmed real world scenarios at certain rates for both
 * qos and drop scheduler */
void scenario_test (testState_t *state)
{
    Qmss_Result       result;
    packetGenCfg      pktGen;
    int               nPorts;
    uint32_t          timer;
#ifdef _TMS320C6X
	uint32_t             corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	uint32_t             corenum = 0;
#endif
    int               i, gen, portNum, portIdx;
    uint32_t         *desc;
    uint32_t          numDescs = Qmss_getQueueEntryCount (state->freeQ); 
    uint32_t          endDescs;
    packetGenQCfg    *genCfg;
    pktGenEngState_t *genState;
    Qmss_Queue        freeQNum = Qmss_getQueueNumber (state->freeQ);
    Qmss_QosSchedPortCfg portCfg[TEST_PORTS];
    int                  portMap[TEST_PORTS];
    uint64_t          startTime;
    uint32_t          actRxTimeMs;
    uint32_t          statsReset =
                 QMSS_QOS_SCHED_STATS_FORWARDED_BYTES   |
                 QMSS_QOS_SCHED_STATS_FORWARDED_PACKETS |
                 QMSS_QOS_SCHED_STATS_DISCARDED_BYTES   |
                 QMSS_QOS_SCHED_STATS_DISCARDED_PACKETS;
    uint32_t          proxyFlag = 0;

#ifdef TEST_DROP_SCHED
    uint16_t                      qosSchedOutQIdx[TEST_PORTS];
    Qmss_QosSchedDropSchedOutProf outProfs[QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES];
    uint16_t                      outQIdx[QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES];
    Qmss_QosSchedDropSchedCfgProf cfgProfs[QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES];
    Qmss_QosSchedDropSchedQueCfg  queProfs[QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS];
    uint32_t                      expNumPushStats;
    Qmss_QosSchedDropSchedCfg     dropCfg;
    dropSchedScenCfg              cfg;
    dropSchedScenCfg             *cfg_p = &cfg;
    int                         **portMap_p = &cfg.qosSchedPortMap;

    cfg.outProfs        = outProfs;
    cfg.outQIdx         = outQIdx;
    cfg.cfgProfs        = cfgProfs;
    cfg.queProfs        = queProfs;
    cfg.dropCfg         = &dropCfg;
    cfg.qosSchedCfgs    = portCfg;
    cfg.qosSchedOutQIdx = qosSchedOutQIdx;
    cfg.expNumPushStats = &expNumPushStats;
#else
    qosSchedScenCfg       cfg;
    qosSchedScenCfg      *cfg_p = &cfg;
    int                 **portMap_p = &cfg.portMap;

    cfg.portCfg         = portCfg;
#endif

    /* Set the return queue of all the descriptors to the free queue */
    for (i = 0; i < numDescs; i++)
    {
        desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (state->freeQ));
        if (! desc) 
        {
            System_printf ("Core %d : failed to pop a free desc\n", corenum);
            errorCount++;
            break;
        }

        Cppi_setReturnQueue (Cppi_DescType_MONOLITHIC, (Cppi_Desc *)desc, freeQNum);
        Cppi_setDescType ((Cppi_Desc *)desc, (Cppi_DescType_MONOLITHIC));
        Qmss_queuePush (state->freeQ, desc, QOS_DATA_PACKET_SIZE, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
    }

    for (i = 0; qosSchedTestScenCfgs[i].fcn; i++)
    {
        /* Run the test case */
        System_printf ("Core %d: Running scenario %d: %s\n", corenum, i + 1, qosSchedTestScenCfgs[i].desc);
        /* Indicate amount of memory available for configs */
        pktGen.numQCfgs = MAX_PKT_GEN_QUEUES;
        nPorts          = TEST_PORTS;
        /* put back portmap */
        *portMap_p      = portMap;

        /* Clear the config */
        memset (portCfg, 0, sizeof(portCfg));
#ifdef TEST_DROP_SCHED
        cfg.fUsePushProxy = 0;
        memset (outProfs, 0, sizeof(outProfs));
        memset (outQIdx, 0, sizeof(outQIdx));
        memset (cfgProfs, 0, sizeof(cfgProfs));
        memset (queProfs, 0, sizeof(queProfs));
        memset (&dropCfg, 0, sizeof(dropCfg));
        memset (qosSchedOutQIdx, 0, sizeof(qosSchedOutQIdx));
        memset (&expNumPushStats, 0, sizeof(expNumPushStats));
#endif

        /* Test test scenario configuration */
        if (qosSchedTestScenCfgs[i].fcn (&timer, cfg_p, &pktGen, &nPorts))
        {
            errorCount++;
            continue;
        }

        
        /* Configure the timer */
        if ((result = Qmss_configureQosSchedTimerSubSys (SUBSYS, PDSPID, timer) != QMSS_QOS_SCHED_RETCODE_SUCCESS))
        {
            errorCount++;
            System_printf("Core %d : Failed to configure QoS timer: %d\n", corenum, result);
        }
        /* Configure QoS */
        for (portIdx = 0; portIdx < nPorts; portIdx++)
        {
            if (*portMap_p)
            {
                portNum = portMap[portIdx];
            }
            else
            {
                portNum = portIdx;
            }

#ifdef TEST_DROP_SCHED
            if (qosSchedOutQIdx[portIdx] != 0xffff)
            {
                portCfg[portIdx].outputQueue = Qmss_getQueueNumber (state->fwQHnds[qosSchedOutQIdx[portIdx]]);
            }
            else
#endif
            {
                /* Insert the output queue handle */
                portCfg[portIdx].outputQueue = Qmss_getQueueNumber (state->qosOutQHnd);
            }
            /* Configure the port */
            port_config (portNum, &portCfg[portIdx]);

            /* Enable the port */
            if ((result = Qmss_enableQosSchedPortSubSys (SUBSYS, PDSPID, portNum)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
            {
                System_printf ("Core %d : failed to enable QoS port %d: %d\n", corenum, portNum, result);
                errorCount++;
            }
        }
#ifdef TEST_DROP_SCHED
        /* Set the output queues for drop scheduler */
        for (portNum = 0; portNum < QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES; portNum++)
        {
            if (outQIdx[portNum] != 0xffff)
            {
                outProfs[portNum].outputQueue = Qmss_getQueueNumber (state->fwQHnds[outQIdx[portNum]]);
            }
        }
        /* Set the stats queues */
        dropCfg.statsQueues[1].src = Qmss_getQueueNumber(state->statsQs[0]);
        dropCfg.statsQueues[1].dst = Qmss_getQueueNumber(state->statsQs[1]);

        /* config drop scheduler */
        testDropSchedSendConfigs (cfg_p);

        proxyFlag = cfg.fUsePushProxy;
#endif

        sendScenarioModel (state, &pktGen, &startTime, proxyFlag);

        /* Check results */
        for (gen = 0; gen < pktGen.numQCfgs; gen++)
        {
            Qmss_QosSchedStats      stats;
            uint32_t                actPPS;
            uint32_t                actError;

            genState = &pktGenEngStates[gen];
            genCfg   = &pktGen.Qs[gen];

            /* Check stats */
#ifdef TEST_DROP_SCHED
            if (genCfg->port != GENPORT_DROPSCHED)
#endif
            {
                Qmss_getQosSchedStatsSubSys (SUBSYS, PDSPID, &stats, genCfg->port, genCfg->group, genCfg->queue, statsReset);
                check_bytes (gen, "stream", stats.bytesForwarded, stats.bytesDiscarded, 
                             genState->bytesTx, genState->bytesRx);
                check_packets (gen, "stream", stats.packetsForwarded, stats.packetsDiscarded, 
                               genState->packetsTx, genState->packetsRx, genState->packetYanked);
            }

            /* Check PPS */
            actRxTimeMs = (genState->lastRxTime - startTime) / (CORE_SPEED/1000);

            /* This reduces rounding error when packets go at low rate */
            if (actRxTimeMs < pktGen.genTime)
            {
                actRxTimeMs = pktGen.genTime;
            }
            actPPS = ((uint64_t)genState->packetsRx * 1000) / actRxTimeMs;
            actError = abs(genCfg->ppsExp - actPPS);
            System_printf ("Core %d : stream %d : found %d pps, expected %d pps, error %d, allowed %d ", 
                           corenum, gen, actPPS, genCfg->ppsExp, actError, genCfg->ppsError);
#ifndef SIMULATOR_SUPPORT
            if (actError > genCfg->ppsError)
            {
                System_printf ("(*** FAIL ***)\n");
                errorCount++;
            }
            else
            {
                System_printf ("(PASS)\n");
            }
#else
            System_printf ("(*** Timing check skipped on simulator ***)\n");
#endif

        }
#ifdef TEST_DROP_SCHED
        {
            uint32_t numPushStats;
            int block;

            /* Check drop sched stats given that one stats blocks can map to two or more 
             * input queues */
            for (block = 0; block < QMSS_QOS_SCHED_DROP_SCHED_STATS_BLOCKS; block++)
            {
                Qmss_QosSchedStats      stats;
                uint64_t                bytesTx = 0, bytesRx = 0;
                uint32_t                pktsTx = 0, pktsRx = 0;
                
                /* Query the block including push stats */
                testQosSched_getPushStats (state, block, &stats, statsReset);
                /* Count up all generators that point at this block */
                for (gen = 0; gen < pktGen.numQCfgs; gen++)
                {
                    genState = &pktGenEngStates[gen];
                    genCfg   = &pktGen.Qs[gen];
                    if ((genCfg->port == GENPORT_DROPSCHED) && 
                        (queProfs[genCfg->queue].valid == Qmss_QosSchedDropSchedProf_VALID) &&
                        (queProfs[genCfg->queue].statBlockIdx == block))
                    {
                        pktsTx += genState->packetsTx;
                        pktsRx += genState->packetsRx;
                        bytesTx += genState->bytesTx;
                        bytesRx += genState->bytesRx;
                    }
                }
                check_bytes (block, "stats block", stats.bytesForwarded, stats.bytesDiscarded, bytesTx, bytesRx);
                check_packets (block, "stats block", stats.packetsForwarded, stats.packetsDiscarded, pktsTx, pktsRx, 0);
            }
            /* Check number of push stats */
            numPushStats = testGetNumPushStats();
            if (numPushStats != expNumPushStats)
            {
                 System_printf ("Core %d : expected %d push stats, got %d\n", corenum, expNumPushStats, numPushStats);
                 errorCount++;
            }

        }
#endif

#ifdef TEST_DROP_SCHED
        /* Readback and check drop scheduler */
        testDropSchedRBConfigs(cfg_p);
#endif

        /* Disable the ports */
        for (portIdx = 0; portIdx < nPorts; portIdx++)
        {
            if (*portMap_p)
            {
                portNum = portMap[portIdx];
            }
            else
            {
                portNum = portIdx;
            }

            if ((result = Qmss_disableQosSchedPortSubSys(SUBSYS, PDSPID, portNum)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
            {
                System_printf ("Core %d : failed to disable QoS port %d: %d\n", corenum, portNum, result);
                errorCount++;
            }
        }

        /* check descriptors */
        endDescs = Qmss_getQueueEntryCount (state->freeQ); 

        if (endDescs != numDescs)
        {
            System_printf ("Core %d : leaked descriptors (%d, %d)\n", corenum, endDescs, numDescs);
            errorCount++;
        }
    }
} /* scenario_test */

/* Insert a descriptor region */
Qmss_QueueHnd insert_memory_block (void *descArea, uint32_t descSize, 
                                   uint32_t nDescs, uint32_t startIndex,
                                   Qmss_MemRegion reg)
{
    Qmss_Result   result;
    Qmss_QueueHnd retVal;
    uint32_t      numAllocated, corenum;

#ifdef _TMS320C6X
	corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
	corenum = 0;
#endif

    memInfo.descBase = (UInt32 *) l2_global_address ((UInt32) descArea);
    memInfo.descSize = descSize;
    memInfo.descNum = nDescs;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = reg;
    memInfo.startIndex = startIndex;

    result = Qmss_insertMemoryRegion (&memInfo);
    if (result < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", corenum, memInfo.memRegion, result);
        errorCount++;
    }

    descCfg.memRegion = reg;
    descCfg.descNum = nDescs;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;

    /* Initialize the descriptors and push to free Queue */
    if ((retVal = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing descriptor error code: %d \n", corenum, retVal);
        errorCount++;
    }
    else
    {
        if (descCfg.descNum != numAllocated)
        {
            errorCount++;
        }
            
        System_printf ("Core %d : Number of descriptors requested : %d. Number of descriptors allocated : %d \n",
            corenum, descCfg.descNum, numAllocated);
    }

    return retVal;
} /* insert_memory_block */

/* Empty then close a queue, and check for error */
void test_empty_and_close_queue (UInt32 corenum, Qmss_QueueHnd hnd)
{
    Qmss_Result retVal;
    Qmss_queueEmpty (hnd);
    if (QMSS_SOK != (retVal = Qmss_queueClose (hnd)))
    {
        System_printf ("Core %d: error closing %d (%d)\n", corenum, hnd, retVal);
        errorCount++;
    }
}

/* Cleans up for exit */
void testCleanup (testState_t *state)
{
    UInt32               corenum;
    Qmss_Result          retVal;
    int                  i;
#ifdef _TMS320C6X
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
    System_printf ("Core %d: closing queues\n", corenum);

    test_empty_and_close_queue (corenum, state->freeQ);
    test_empty_and_close_queue (corenum, state->divQ);
    test_empty_and_close_queue (corenum, state->qosOutQHnd);
    test_empty_and_close_queue (corenum, state->dropQHnd);
    for (i = 0; i < QOS_TX_QUEUES; i++)
    {
        test_empty_and_close_queue (corenum, state->fwQHnds[i]);
    }
#ifdef TEST_DROP_SCHED
    test_empty_and_close_queue (corenum, state->statsQs[0]);
    test_empty_and_close_queue (corenum, state->statsQs[1]);
    for (i = 0; i < QOS_TX_DROP_QUEUES; i++)
    {
        test_empty_and_close_queue (corenum, state->dropFwQHnds[i]);
    }
#endif
    System_printf ("Core %d: remove region(s)\n", corenum);
    if ( (retVal = Qmss_removeMemoryRegion (Qmss_MemRegion_MEMORY_REGION0, 0)) != QMSS_SOK)
    {
        System_printf ("Core %d: error removing region 0 (%d)\n", corenum, retVal);
        errorCount++;
    }
#ifdef TEST_DROP_SCHED
    if ( (retVal = Qmss_removeMemoryRegion (Qmss_MemRegion_MEMORY_REGION1, 0)) != QMSS_SOK)
    {
        System_printf ("Core %d: error removing region 1 (%d)\n", corenum, retVal);
        errorCount++;
    }
#endif
    System_printf ("Core %d: exit qmss\n", corenum);
    if ( (retVal = Qmss_exit ()) != QMSS_SOK)
    {
        System_printf ("Core %d: error exiting qm (%d)\n", corenum, retVal);
        errorCount++;
    }
} /* testCleanup */

/* Runs QoS scheduler (not drop scheduler) tests */
void testQosSched (testState_t *state)
{
#if RM    
    /* RM configuration */
    Rm_InitCfg           rmInitCfg;
    char                 rmServerName[RM_NAME_MAX_CHARS] = "RM_Server";
    Rm_ServiceHandle     *rmServiceHandle; 
    int32_t              rmResult;
#endif      
    Qmss_Result          result;
    UInt32               corenum;
    Qmss_QueueHnd        baseQueue;
    int                  queueNum;
    uint8_t              isAllocated;
    Qmss_QosSchedPortCfg portCfg[TEST_PORTS];
    int                  port, group, queue;
#ifdef FULL_TEST_SUITE
    int                  beQueues, rrQueues, spQueues;
#endif
    uint32_t             fwQGroup;

    /* Reset timer */
#ifdef _TMS320C6X
    TSCL = 0;
#endif

    /* Get the core number. */
#ifdef _TMS320C6X
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
    System_printf ("**********Core %d TESTING QoS scheduler ************\n", corenum);
#ifdef SIMULATOR_SUPPORT
    System_printf ("Simulator can successfully execute the firmware.\n"
                   "However, since its not cycle accurate, the actual.\n"
                   "throughput will be off.  All tput checks disabled\n");
#ifndef SIMULATOR_SUPPORT_FULL
    System_printf ("Some test cases disabled to speed up test case on simulator\n");
#endif
#endif
#if RM    
    /* Create the Server instance */
    memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
    rmInitCfg.instName = rmServerName;
    rmInitCfg.instType = Rm_instType_SERVER;
    rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
    rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspOnlyPolicy;
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    if (rmResult != RM_OK)
    {
        System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", corenum, rmResult);
        errorCount++;
        return;
    }

    rmServiceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    if (rmResult != RM_OK)
    {
        System_printf ("Error Core %d : Creating RM service handle error code : %d\n", corenum, rmResult);
        errorCount++;
        return;
    } 
#endif 

    memset ((Void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up the linking RAM. Use internal Linking RAM.  */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0x0;
    qmssInitConfig.maxDescNum      = NUM_DESC;

#ifndef USE_QOS_MODEL
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP2;
#ifdef TEST_MULTI_GROUP
#ifdef WIDE
#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].firmware = &qos_sched_wide_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_sched_wide_be);
#else
    qmssInitConfig.pdspFirmware[0].firmware = &qos_sched_wide_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_sched_wide_le);
#endif
#else
#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].firmware = &qos_sched_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_sched_be);
#else
    qmssInitConfig.pdspFirmware[0].firmware = &qos_sched_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_sched_le);
#endif
#endif
#else
#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].firmware = &qos_sched_drop_sched_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_sched_drop_sched_be);
#else
    qmssInitConfig.pdspFirmware[0].firmware = &qos_sched_drop_sched_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_sched_drop_sched_le);
#endif
#endif
#endif

#if RM
    qmssGblCfgParams.qmRmServiceHandle = rmServiceHandle;
#endif

    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", corenum, result);
        errorCount++;
        return;
    }

    /* Start Queue Manager SubSystem */
    result = Qmss_start ();
    if (result != QMSS_SOK)
    {
        System_printf ("Core %d : Error starting Queue Manager error code : %d\n", corenum, result);
    }

#ifdef USE_QOS_MODEL
    /* Busy wait until the model is started */
    while (Qmss_getQosSchedFwVersionSubSys(SUBSYS, PDSPID) != 0xf0f0f0f0);
#endif

    /* Now that the FW is downloaded, can query its version */
    System_printf ("Core %d : QoS Sched Firmware Rev 0x%08x\n", corenum, Qmss_getQosSchedFwVersionSubSys(SUBSYS, PDSPID));
    System_printf ("Core %d : QoS Sched Magic 0x%08x\n", corenum, Qmss_getQosSchedFwMagic());

    /* Setup memory region for monolithic descriptors */
    memset (descArea, 0, sizeof(descArea));
    state->freeQ = insert_memory_block (descArea, SIZE_MONOLITHIC_DESC, NUM_MONOLITHIC_DESC, 0,
                                        Qmss_MemRegion_MEMORY_REGION0);
    /* freeQ, fwQHnds, dropQHnd, divQ must all be on same QM so we can divert */
    fwQGroup = Qmss_getQueueGroup (state->freeQ);
    
#ifdef TEST_DROP_SCHED
    /* Insert stats queue descriptor area */
    state->statsQs[0] = insert_memory_block (descArea + SIZE_MONOLITHIC_DESC*NUM_MONOLITHIC_DESC,
                                             SIZE_STATS_DESC, NUM_STATS_DESC, NUM_MONOLITHIC_DESC,
                                             Qmss_MemRegion_MEMORY_REGION1);
    /* Open stats return queue */
    state->statsQs[1] = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 
                                        QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (state->statsQs[1] < 0) 
    {
        errorCount++;
        System_printf("stats return Queue open failed: %d\n", state->statsQs[1]);
    }
#endif

    /* Open scratch queue */
    state->divQ = Qmss_queueOpenInGroup (fwQGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 
                                         QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (state->divQ < 0) 
    {
        errorCount++;
        System_printf("scratch Queue open failed: %d\n", state->divQ);
    }

    /* Allocate block of queues to be used by firmware */
    baseQueue = Qmss_queueBlockOpenInGroup (state->fwQHnds, fwQGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QOS_TX_QUEUES, 32);
    if (baseQueue < 0) 
    {
        System_printf ("Core %d : Failed to open %d contiguous queues\n", corenum, QOS_TX_QUEUES);
        errorCount++;
    }

    /* Set the FW's base queue */
    if ((result = Qmss_setQosSchedQueueBaseSubSys (SUBSYS, PDSPID, baseQueue)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Setting QoS queue base address error code: %d \n", corenum, result);
        errorCount++;
    }

    /* Configure the queue thresholds as required by the FW */
    for (queueNum = 0; queueNum < QOS_TX_QUEUES; queueNum++) 
    {
        Qmss_setQueueThreshold (state->fwQHnds[queueNum], 1, 1);
    }

    /* Open output queue */
    state->qosOutQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 
                                       QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (state->qosOutQHnd < 0) 
    {
        errorCount++;
        System_printf("out Queue open failed: %d\n", state->qosOutQHnd);
    }

    /* Open drop/congestion queue in same group (for divert) */
    state->dropQHnd = Qmss_queueOpenInGroup (fwQGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                            QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (state->dropQHnd < 0) 
    {
        errorCount++;
        System_printf("drop Queue open failed: %d\n", state->dropQHnd);
    }

    /* Set up QoS's timer */
    if ((result = Qmss_configureQosSchedTimerSubSys (SUBSYS, PDSPID, QOS_TIMER_CONFIG) != QMSS_QOS_SCHED_RETCODE_SUCCESS))
    {
        errorCount++;
        System_printf("Core %d : Failed to configure QoS timer: %d\n", corenum, result);
    }

    /* Run congestion test */
    congestion_test(state, Qmss_QosSchedAcctType_PACKETS);
    congestion_test(state, Qmss_QosSchedAcctType_BYTES);

    /* Set up for scheduling using packet count units */
    memset (portCfg, 0, sizeof(portCfg));
    for (port = 0; port < TEST_PORTS; port++)
    {
        portCfg[port].wrrType = Qmss_QosSchedAcctType_PACKETS;
        portCfg[port].cirType = Qmss_QosSchedAcctType_PACKETS;
        portCfg[port].congestionType = Qmss_QosSchedAcctType_PACKETS;
        portCfg[port].outThrotType = Qmss_QosSchedAcctType_PACKETS;
        portCfg[port].cirIteration = 1 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
        portCfg[port].cirMax = 10 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
        portCfg[port].overheadBytes = QOS_DATA_PACKET_OVHD;
        portCfg[port].removeBytes = 0;
        portCfg[port].outThrotThresh = 0;
        if (port >= TEST_FULL_MAX_PHYS_PORTS) 
        {
            portCfg[port].groupCount = QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS;
        }
        else
        {
            portCfg[port].groupCount = TEST_MAX_FULL_LOG_GROUPS;
        }
        portCfg[port].outputQueue = Qmss_getQueueNumber (state->qosOutQHnd);
        for (group = 0; group < portCfg[port].groupCount; group++)
        {
#ifdef TEST_MULTI_GROUP
            portCfg[port].group[group].cirIteration = 1 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
            portCfg[port].group[group].pirIteration = 1 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
            portCfg[port].group[group].wrrInitialCredit = 1 << QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT;
            portCfg[port].group[group].cirMax = 10 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
            portCfg[port].group[group].pirMax = 10 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
#endif
            if (port >= TEST_FULL_MAX_PHYS_PORTS) 
            {
                portCfg[port].group[group].totQueueCount = 4;
                portCfg[port].group[group].spQueueCount = 4;
                portCfg[port].group[group].wrrQueueCount = 0;
            } 
            else 
            {
                portCfg[port].group[group].totQueueCount = 8;
                portCfg[port].group[group].spQueueCount = 2;
                portCfg[port].group[group].wrrQueueCount = 5;
            }
            for (queue = 0; queue < portCfg[port].group[group].totQueueCount; queue++)
            {
                portCfg[port].group[group].Queue[queue].wrrInitialCredit = 1 << QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT;
                /* Disable congestion dropping so we can test by pre-loading queues without dropping
                 * the data */
                portCfg[port].group[group].Queue[queue].congestionThresh = 0;
            }
        }

        /* set and check the configuration */
        port_config (port, &portCfg[port]);
    }

    /* This runs the transfer test while reconfiguring the ports on fly after the drop test */
#ifdef FULL_TEST_SUITE
    transfer_test ("packet units changed on the fly", state, portCfg, PROFILE_DESCS, 0, 1);
#endif

    /* This re runs the test against the model, which resets the ports */
#ifdef FULL_TEST_SUITE
    transfer_test ("packet units after reset compared to model", state, portCfg, PROFILE_DESCS, 1, 1);
#endif

    /* Re run but configure for 1 mbit from each port */
#define TEST_RATE 1000000 /* bits per second */
#define TEST_CLOCK 10000 /* ticks per second */
    for (port = 0; port < TEST_PORTS; port++)
    {
        portCfg[port].wrrType = Qmss_QosSchedAcctType_BYTES;
        portCfg[port].cirType = Qmss_QosSchedAcctType_BYTES;
        portCfg[port].congestionType = Qmss_QosSchedAcctType_BYTES;
        portCfg[port].outThrotType = Qmss_QosSchedAcctType_BYTES;
        portCfg[port].cirIteration = (((TEST_RATE / TEST_CLOCK) << QMSS_QOS_SCHED_BYTES_SCALE_SHIFT) / 8);
        if (portCfg[port].cirIteration == 0)
        {
            portCfg[port].cirIteration = 1;
        }
        portCfg[port].cirMax = 10 * portCfg[port].cirIteration;
        for (group = 0; group < portCfg[port].groupCount; group++)
        {
#ifdef TEST_MULTI_GROUP
            portCfg[port].group[group].cirIteration = portCfg[port].cirIteration / portCfg[port].groupCount;
            if (portCfg[port].group[group].cirIteration == 0)
            {
                portCfg[port].group[group].cirIteration = 1;
            }
            portCfg[port].group[group].pirIteration = portCfg[port].group[group].cirIteration;
            portCfg[port].group[group].wrrInitialCredit = 
                50 << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT;
            portCfg[port].group[group].cirMax = 10 * portCfg[port].group[group].cirIteration;
            portCfg[port].group[group].pirMax = 10 * portCfg[port].group[group].pirIteration;
#endif
            for (queue = 0; queue < portCfg[port].group[group].totQueueCount; queue++)
            {
                portCfg[port].group[group].Queue[queue].wrrInitialCredit = 
                    50 << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT;
            }
        }
    }

    /* This runs the transfer test while reconfiguring the ports on fly after the packet test */
#ifdef FULL_TEST_SUITE
    transfer_test ("byte units reconfiguring on fly", state, portCfg, PROFILE_DESCS, 0, 1);
#endif

    /* This re runs the test against the model, which resets the ports */
#ifdef FULL_TEST_SUITE
    transfer_test ("byte units with reset and comparing to model", state, portCfg, PROFILE_DESCS, 1, 1);
#endif

    /* reconfigure for a more compex case where all the rates are different and
     * PIR is available.  This does NOT match any realistic use case.
     * It only makes sense to compare this result to the model.
     */
    for (port = 0; port < TEST_PORTS; port++)
    {
        portCfg[port].cirMax = 10 * portCfg[port].cirIteration;
        for (group = 0; group < portCfg[port].groupCount; group++)
        {
#ifdef TEST_MULTI_GROUP
            portCfg[port].group[group].cirIteration = 
                portCfg[port].cirIteration / portCfg[port].groupCount;

            /* Make the rates different */
            portCfg[port].group[group].cirIteration += 
                group * portCfg[port].group[group].cirIteration / portCfg[port].groupCount;

            if (portCfg[port].group[group].cirIteration == 0)
            {
                portCfg[port].group[group].cirIteration = 1;
            }

            portCfg[port].group[group].pirIteration = portCfg[port].group[group].cirIteration * 11 / 10;
            portCfg[port].group[group].wrrInitialCredit = 
                (50 * (group + 1)) << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT;
            portCfg[port].group[group].cirMax = 10 * portCfg[port].group[group].cirIteration;
            portCfg[port].group[group].pirMax = 10 * portCfg[port].group[group].pirIteration;
#endif
            for (queue = 0; queue < portCfg[port].group[group].totQueueCount; queue++)
            {
                portCfg[port].group[group].Queue[queue].wrrInitialCredit = 
                    (50 * (queue + 1)) << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT;
            }
        }
    }

    /* This re runs the test against the model, which resets the ports */
    transfer_test ("complex configuration with reset vs model\n", state, portCfg, PROFILE_DESCS, 1, 0);

    /* Now try combinations of 0, 1, 2 of each type of queue */
#ifdef FULL_TEST_SUITE
    for (spQueues = 0; spQueues <= 2; spQueues++)
    {
        for (rrQueues = 0; rrQueues <= 2; rrQueues++)
        {
            for (beQueues = 0; beQueues <= 2; beQueues++)
            {
                int portLimit = TEST_PORTS;
                if (!spQueues && !rrQueues & !beQueues)
                {
                    continue;
                }
                if (portLimit > TEST_FULL_MAX_PHYS_PORTS)
                {
                    portLimit = TEST_FULL_MAX_PHYS_PORTS;
                }
                for (port = 0; port < portLimit ; port++)
                {
                    for (group = 0; group < portCfg[port].groupCount; group++)
                    {
                        if (port < TEST_FULL_MAX_PHYS_PORTS)
                        {
                            portCfg[port].group[group].spQueueCount = spQueues;
                            portCfg[port].group[group].wrrQueueCount = rrQueues;
                            portCfg[port].group[group].totQueueCount = spQueues + rrQueues + beQueues;
                        }
                        for (queue = 0; queue < portCfg[port].group[group].totQueueCount; queue++)
                        {
                            portCfg[port].group[group].Queue[queue].wrrInitialCredit = 
                                (50 * (queue + 1)) << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT;
                        }
                    }
                }
                System_printf ("Core %d: trying %d sp queues, %d rr queues, and %d be queues\n",
                               corenum, spQueues, rrQueues, beQueues);
                /* This re runs the test against the model, which resets the ports */
                transfer_test ("queue loop corner case test", state, portCfg, PROFILE_DESCS > 5 ? 5 : PROFILE_DESCS, 1, 0);
            }
        }
    }
#endif

    /* Disable all the ports from previous test */
    for (port = 0; port < TEST_PORTS; port++)
    {
        if ((result = Qmss_disableQosSchedPortSubSys (SUBSYS, PDSPID, port)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
        {
            System_printf ("Core %d : failed to disable QoS port %d: %d\n", corenum, port, result);
            errorCount++;
        }
    }

} /* testQosSched */

Void run_test (Void)
{
    testState_t state;
#ifdef _TMS320C6X
    uint32_t      corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    uint32_t      corenum = 0;
#endif
    testQosSched (&state);
#ifdef TEST_DROP_SCHED
    testDropSched (&state);
#endif
#if defined(FULL_TEST_SUITE) || defined(TEST_DROP_SCHED)
    System_printf ("Core %d: Running deployment scenarios\n", corenum);
    scenario_test (&state);
#endif

    /* Clean up */
    testCleanup (&state);

#if RM
    {
        int32_t              rmResult;
        
        if ((rmResult = Rm_resourceStatus(rmHandle, FALSE)) != 0)
        {
            System_printf ("Error Core %d : Number of unfreed resources : %d\n", corenum, rmResult);
            errorCount++;
        }
        else
            System_printf ("Core %d : All resources freed successfully\n", corenum);
    }
#endif

    if (errorCount == 0)
        System_printf ("\nCore %d : QoS scheduler tests Passed\n", corenum);
    else 
        System_printf ("\nCore %d : ***********FAIL***********\n", corenum);
} /* run_test */


