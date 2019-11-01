/**
 *   @file  test_srioRtr.c
 *
 *   @brief
 *      This is the QMSS unit test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <xdc/cfg/global.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <string.h>
#include <stdlib.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>
#include <ti/drv/qmss/qmss_srioRtr.h>

/* CPPI LLD includes */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* CSL RL includes */
#include <ti/csl/csl_chip.h>

/* OSAL includes */
#include <qmss_osal.h>

/************************ USER DEFINES ********************/
#define NUM_HOST_DESC               1024
#define SIZE_HOST_DESC              48
#define SIZE_BUF                    4
#define PROFILE_DESCS               400
#define NUM_CREDIT_PACKETS          4
#define TEST_PDSP                   Qmss_PdspId_PDSP2
#define TEST_PORTS                  4
#define SRIO_ROUTER  // vs switch
//#define VERBOSE

/************************ GLOBAL VARIABLES ********************/
/* Descriptor pool [Size of descriptor * Number of descriptors] */
#ifdef _TMS320C6X
#pragma DATA_ALIGN (hostDesc, 64)
uint8_t hostDesc[SIZE_HOST_DESC * NUM_HOST_DESC];
#pragma DATA_ALIGN (buffers, 64)
uint8_t buffers[SIZE_BUF * NUM_HOST_DESC];
#else
uint8_t hostDesc[SIZE_HOST_DESC * NUM_HOST_DESC] __attribute__ ((aligned (64)));;
uint8_t buffers[SIZE_BUF * NUM_HOST_DESC] __attribute__ ((aligned (64)));;
#endif

/* Timestamp when SRIO router moved each descriptor */
struct
{
    void    *descPtr;
    int      queueNum;
    uint32_t timestamp;
} timestamps[NUM_HOST_DESC];

/* Global variable common to all test cases */

/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;
/* QM descriptor configuration */
Cppi_DescCfg            descCfg;
/* Handle to CPPI heap */
IHeap_Handle            cppiHeap;

/************************ EXTERN VARIABLES ********************/
/* Error counter */
extern uint32_t                 errorCount;
/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/*************************** FUNCTIONS ************************/

#ifdef _TMS320C6X
/**
 *  @b Description
 *  @n
 *      The function is used to get the handle to the CPPI memory heap.
 *      If the application is run on a multiple cores then place the CPPI global
 *      variables in shared memory.
 *
 *  @retval
 *      None
 */
static Void cppiHeapInit ()
{
    cppiHeap = HeapMem_Handle_upCast (cppiLocalHeap);
}
#endif

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
static uint32_t l2_global_address (uint32_t addr)
{
#ifdef _TMS320C6X
    uint32_t corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum * 0x1000000)));
#else
    return addr;
#endif
}

/* Check if valid queue handle */
void check_result (const char *desc, Qmss_Result hnd)
{
    if (hnd < QMSS_SOK)
    {
        System_printf("Failed to allocate queue (%d): %s\n", hnd, desc);
        exit(1);
    }
}

/* check if two memories of uint32_t are the same */
void check_memory (const char *desc, const void *buf1, const void *buf2, uint32_t size)
{
    if (memcmp (buf1, buf2, size) != 0)
    {
        System_printf("Buffer compare failed: %s\n", desc);
        exit(1);
    }
}

/* Check for NULL pointer */
void check_null (const char *desc, const void *ptr)
{
    if (! ptr)
    {
        System_printf("NULL check failed: %s\n", desc);
        exit(1);
    }
}

void check_stats (int port, Qmss_SrioRtrStats *check)
{
    Qmss_SrioRtrStats stats;
    check_result ("query stats", Qmss_getSrioRtrStats (TEST_PDSP, port, 0, 0, &stats));
    if (stats.hostPkts != check->hostPkts) {
        System_printf ("port %d stats: expect %d host packets, found %d\n", port, check->hostPkts, stats.hostPkts);
        errorCount++;
    }
    if (stats.packetsRx != check->packetsRx) {
        System_printf ("port %d stats: expect %d rx packets, found %d\n", port, check->packetsRx, stats.packetsRx);
        errorCount++;
    }
    if (stats.packetsTx != check->packetsTx) {
        System_printf ("port %d stats: expect %d tx packets, found %d\n", port, check->packetsTx, stats.packetsTx);
        errorCount++;
    }
    if (stats.creditsTx != check->creditsTx) {
        System_printf ("port %d stats: expect %d tx credits packets, found %d\n", port, check->creditsTx, stats.creditsTx);
        errorCount++;
    }
    if (stats.creditsRx != check->creditsRx) {
        System_printf ("port %d stats: expect %d rx credits packets, found %d\n", port, check->creditsRx, stats.creditsRx);
        errorCount++;
    }
    if (stats.fwdRets != check->fwdRets) {
        System_printf ("port %d stats: expect %d forward return packets, found %d\n", port, check->fwdRets, stats.fwdRets);
        errorCount++;
    }
}
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
#else
    uint32_t start, end;

    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(start));
    if ((0x100000000 - start) < cycles) {
        do {
            __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(end));
        } while (end > start);
        cycles -= 0x100000000 - start;
        start = 0;
    }
    do {
            __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(end));
    } while ((end - start) < cycles);
#endif
}


void spoof_srio_fwd (Qmss_QueueHnd srcQ, Qmss_QueueHnd dstQ)
{
    void *descPtr;

    if (descPtr = Qmss_queuePop (srcQ))
    {
       void *cleanDescPtr = (void *)QMSS_DESC_PTR (descPtr);
       volatile uint32_t *patchRoutingData;
       volatile uint32_t *checkPSInfo;
       uint32_t routeLen, dest, bits;

       Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, cleanDescPtr, (uint8_t **)&checkPSInfo, &routeLen);
#ifndef SRIO_ROUTER
       patchRoutingData = checkPSInfo;
#else
       /* patch destination in first word of buffer */
       Cppi_getData (Cppi_DescType_HOST, cleanDescPtr, (uint8_t **)&patchRoutingData, &routeLen);
#endif
       bits = *patchRoutingData;
       dest = bits & 0xffff;
       dest++;
       bits &= 0xffff0000;
       bits |= dest;
       *patchRoutingData = bits;
       if (checkPSInfo[1] & 0x1)
       {
           System_printf("error: intermediate packet has LSB of mailbox id (t11) or cos (t9) set\n");
           errorCount++;
       }
       Qmss_queuePushDesc (dstQ, descPtr);
    }
}

void drain_1_desc (const char *desc, Qmss_QueueHnd q)
{
    void *descPtr;
    uint32_t descSize;

    check_null (desc, descPtr = Qmss_queuePop (q));
    descSize = QMSS_DESC_SIZE (descPtr);
    if (descSize != SIZE_HOST_DESC)
    {
        System_printf ("error: %s desc size hint lost (exp %d, got %d)\n",
                       desc, SIZE_HOST_DESC, descSize);
        errorCount++;
    }
    /* intentionally drop descPtr for cleanup */
}

#ifdef VERBOSE
void printSrioRtrCfg (void)
{
    Qmss_SrioRtrGblCfg      srioRtrGblCfgRB;
    Qmss_SrioRtrRouteTbl    routeTblRB;
    Qmss_SrioRtrPortCfg     srioRtrPortCfgRB;
    int                     i, j;

    /* Print global config */
    memset (&srioRtrGblCfgRB, 0, sizeof(srioRtrGblCfgRB));
    check_result ("printf global config", Qmss_getSrioRtrGblCfg (TEST_PDSP, 0, &srioRtrGblCfgRB));
    System_printf ("gblCfg.queueBase  = %d\n", srioRtrGblCfgRB.queueBase);
    System_printf ("gblCfg.creditSize = %d\n", srioRtrGblCfgRB.creditSize);

    /* Print each port */
    for (i = 0; i < TEST_PORTS; i++)
    {
        memset (&srioRtrPortCfgRB, 0, sizeof(srioRtrPortCfgRB));
        check_result ("printf port", Qmss_getSrioRtrPortCfg (TEST_PDSP, i, 0, &srioRtrPortCfgRB));
        System_printf ("portCfg[%d].hostSrioTxQueue       = %d\n",     i, srioRtrPortCfgRB.hostSrioTxQueue);
        System_printf ("portCfg[%d].creditSrioTxQueue     = %d\n",     i, srioRtrPortCfgRB.creditSrioTxQueue);
        System_printf ("portCfg[%d].creditReturnQueue     = %d\n",     i, srioRtrPortCfgRB.creditReturnQueue);
        System_printf ("portCfg[%d].creditSourceQueue     = %d\n",     i, srioRtrPortCfgRB.creditSourceQueue);
        System_printf ("portCfg[%d].destID                = 0x%08x\n", i, srioRtrPortCfgRB.destID);
        System_printf ("portCfg[%d].fwdTxFinCmpQueue      = %d\n",     i, srioRtrPortCfgRB.fwdTxFinCmpQueue);
        System_printf ("portCfg[%d].hostTxFinCmpQueue     = %d\n",     i, srioRtrPortCfgRB.hostTxFinCmpQueue);
        System_printf ("portCfg[%d].sourcePortSrioTxQueue = { ", i);
        for (j = 0; j < TEST_PORTS; j++)
        {
            System_printf ("%d ", srioRtrPortCfgRB.sourcePortSrioTxQueue[j]);
        }
        System_printf ("}\n\n");
    }

    /* print routing table */
    check_result ("printf routing table", Qmss_getSrioRtrGblRouteTbl (TEST_PDSP, 0, &routeTblRB));
    for (i = 0; i < 256; i+= 8)
    {
        System_printf ("Routing[%03d-%03d] = ", i, i+7);
        for (j = 0; j < 8; j++)
        {
            System_printf ("%d %d, ", routeTblRB.isFinalDest[i+j], routeTblRB.forwardToPort[i+j]);
        }
        System_printf("\n");
    }
    System_printf("\n");
}

void printSrioRtrStats ()
{
    Qmss_SrioRtrStats stats;
    int i;

    for (i = 0; i < TEST_PORTS; i++)
    {
        check_result ("printf stats", Qmss_getSrioRtrStats (TEST_PDSP, i, 0, 0, &stats));
        System_printf ("stats[%d].hostPkts  = %d\n", i, stats.hostPkts);
        System_printf ("stats[%d].packetsRx = %d\n", i, stats.packetsRx);
        System_printf ("stats[%d].packetsTx = %d\n", i, stats.packetsTx);
        System_printf ("stats[%d].creditsTx = %d\n", i, stats.creditsTx);
        System_printf ("stats[%d].creditsRx = %d\n", i, stats.creditsRx);
        System_printf ("stats[%d].fwdRets   = %d\n", i, stats.fwdRets);
        System_printf ("stats[%d].hostRets  = %d\n", i, stats.hostRets);
    }
}
#endif

void testSrioRtr (void)
{
    Qmss_Result             result;
    uint32_t                numAllocated, corenum, startTime, dTime;
    Qmss_QueueHnd           freeQ;
    Qmss_QueueHnd           baseQueue;
    Qmss_QueueHnd           hostRxQueue;
    Qmss_QueueHnd           errorQueue;
    Qmss_QueueHnd           emptyQueue;
    Qmss_QueueHnd           creditPool01;
    Qmss_QueueHnd           creditPool12;
    Qmss_QueueHnd           creditPool23;
    Qmss_QueueHnd           creditPool3host;
    Qmss_QueueHnd           fwQHnds[QMSS_SRIORTR_FW_QUEUES];
    Qmss_QueueHnd           fakeSrioTxQueue12;
    Qmss_QueueHnd           fakeSrioTxQueue23;
    uint8_t                 isAllocated;
    Qmss_SrioRtrGblCfg      srioRtrGblCfg, srioRtrGblCfgRB;
    Qmss_SrioRtrPortCfg     srioRtrPortCfg[TEST_PORTS], srioRtrPortCfgRB;
    Qmss_SrioRtrRouteTbl    routeTbl, routeTblRB;
    Qmss_SrioRtrStats       srioRtrStatsCheck;
    int                     port, i, nextCredit;

#ifdef _TMS320C6X
    /* Reset timer */
    TSCL = 0;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
    System_printf ("**********Core %d TESTING SRIO ROUTER ************\n", corenum);

#ifdef _TMS320C6X
    /* Initialize the heap in shared memory for CPPI data structures */
    cppiHeapInit ();
#endif

    memset ((Void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up the linking RAM. Use internal Linking RAM.  */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0x0;
    qmssInitConfig.maxDescNum      = NUM_HOST_DESC;

#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId = TEST_PDSP;
    qmssInitConfig.pdspFirmware[0].firmware = &sriortr_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (sriortr_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId = TEST_PDSP;
    qmssInitConfig.pdspFirmware[0].firmware = &sriortr_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (sriortr_le);
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

    /* Now that the FW is downloaded, can query its version */
    System_printf ("Core %d : SrioRtr Firmware Rev 0x%08x\n", corenum, Qmss_getSrioRtrFwVersion(TEST_PDSP));
    System_printf ("Core %d : SrioRtr Magic 0x%08x\n", corenum, Qmss_getSrioRtrFwMagic(TEST_PDSP));

    /* Setup memory region for host descriptors */
    memset ((Void *) &hostDesc, 0, SIZE_HOST_DESC * NUM_HOST_DESC);
    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) hostDesc);
    memInfo.descSize = SIZE_HOST_DESC;
    memInfo.descNum = NUM_HOST_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    result = Qmss_insertMemoryRegion (&memInfo);
    if (result < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", corenum, memInfo.memRegion, result);
        errorCount++;
    }

    memset (&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion             = Qmss_MemRegion_MEMORY_REGION0;
    descCfg.descNum               = NUM_HOST_DESC;
    descCfg.destQueueNum          = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType             = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    descCfg.initDesc              = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType              = Cppi_DescType_HOST;
    descCfg.returnQueue.qNum      = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qMgr      = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnPushPolicy      = Qmss_Location_TAIL;
    descCfg.epibPresent           = Cppi_EPIB_NO_EPIB_PRESENT; // dont change
    descCfg.cfg.host.returnPolicy = Cppi_ReturnPolicy_RETURN_ENTIRE_PACKET; // dont change
    descCfg.cfg.host.psLocation   = Cppi_PSLoc_PS_IN_DESC; // dont change


    /* Initialize the descriptors and push to free Queue */
    if ((freeQ = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing descriptor error code: %d \n", corenum, freeQ);
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

    /* Allocate block of queues to be used by firmware */
    baseQueue = Qmss_queueBlockOpen (fwQHnds, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_SRIORTR_FW_QUEUES, 32);
    if (baseQueue < 0)
    {
        System_printf ("Core %d : Failed to open %d contiguous queues\n", corenum, QMSS_SRIORTR_FW_QUEUES);
        errorCount++;
    }


    srioRtrGblCfg.queueBase = baseQueue;
    srioRtrGblCfg.creditSize = 20;
    check_result ("set global config", Qmss_setSrioRtrGblCfg (TEST_PDSP, 0, &srioRtrGblCfg));
    memset (&srioRtrGblCfgRB, 0, sizeof(srioRtrGblCfgRB));
    check_result ("get global config", Qmss_getSrioRtrGblCfg (TEST_PDSP, 0, &srioRtrGblCfgRB));
    check_memory ("check global readback", &srioRtrGblCfg, &srioRtrGblCfgRB, sizeof(srioRtrGblCfg));

    /* Open queue to receive all output after snaking through fw */
    check_result ("open hostRxQueue", hostRxQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));
    check_result ("open errorQueue", errorQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));
    check_result ("open empty queue", emptyQueue = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));

    /* Part 1: configure everything as empty/errors */
    for (port = 0; port < TEST_PORTS; port++)
    {
        srioRtrPortCfg[port].hostSrioTxQueue   = errorQueue;
        srioRtrPortCfg[port].creditSrioTxQueue = errorQueue;
        srioRtrPortCfg[port].fwdTxFinCmpQueue  = errorQueue;
        for (i = 0; i < TEST_PORTS; i++)
        {
            srioRtrPortCfg[port].sourcePortSrioTxQueue[i] = errorQueue;
        }
        srioRtrPortCfg[port].creditReturnQueue = errorQueue;
        srioRtrPortCfg[port].creditSourceQueue = emptyQueue;
        srioRtrPortCfg[port].destID = 0;

    }

    /* In order to do daisy chain on single pdsp on single device need to patch psinfo between port 1 and 2 */
    check_result ("open fakeSrioTxQueue12", fakeSrioTxQueue12 = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));
    /* In order to do daisy chain on single pdsp on single device need to patch psinfo between port 2 and 3 */
    check_result ("open fakeSrioTxQueue23", fakeSrioTxQueue23 = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));

    /* Part 2: daisy chain the ports */
    /* Data flow is host->host queue on port 0
     * port 0 forwards host traffic to port 1 directly
     * port 1 forwards to port 2 via port 1's fwd return queue
     * port 2 forwards to port 3 via port 2's fwd return queue
     * port 3 forwards to host (by spoofing port 0) via port 3's fwd return queue
     *
     * *******************************************************************************************
     * note this is NOT how it it will really be connected with srio but dataflow hops are similar
     * *******************************************************************************************
     */
    srioRtrPortCfg[0].hostSrioTxQueue          = fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_FWD_QUEUE(1)];
    srioRtrPortCfg[1].sourcePortSrioTxQueue[2] = fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_FWD_RET_QUEUE(1)];
    srioRtrPortCfg[1].fwdTxFinCmpQueue         = fakeSrioTxQueue12;
    srioRtrPortCfg[2].sourcePortSrioTxQueue[3] = fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_FWD_RET_QUEUE(2)];
    srioRtrPortCfg[2].fwdTxFinCmpQueue         = fakeSrioTxQueue23;
    srioRtrPortCfg[3].sourcePortSrioTxQueue[0] = fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_FWD_RET_QUEUE(3)];
    srioRtrPortCfg[3].fwdTxFinCmpQueue         = hostRxQueue;
    srioRtrPortCfg[0].destID                   = 0xfee10000; // actually goes to host queue */
    srioRtrPortCfg[2].destID                   = 0xfee1000a;
    srioRtrPortCfg[3].destID                   = 0xfee1000b;
    srioRtrPortCfg[1].creditSrioTxQueue        = fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_CREDIT_QUEUE(0)];
    srioRtrPortCfg[2].creditSrioTxQueue        = fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_CREDIT_QUEUE(1)];
    srioRtrPortCfg[3].creditSrioTxQueue        = fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_CREDIT_QUEUE(2)];

    /* Part 3: setup credit pools between ports 0,1; 1,2; 2,3; 3 and host */
    check_result ("open credit pool 0:1", creditPool01 = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));
    check_result ("open credit pool 1:2", creditPool12 = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));
    check_result ("open credit pool 2:3", creditPool23 = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));
    check_result ("open credit pool 3:host", creditPool3host = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated));
    srioRtrPortCfg[0].creditReturnQueue = creditPool01;
    srioRtrPortCfg[1].creditSourceQueue = creditPool01;

    srioRtrPortCfg[1].creditReturnQueue = creditPool12;
    srioRtrPortCfg[2].creditSourceQueue = creditPool12;

    srioRtrPortCfg[2].creditReturnQueue = creditPool23;
    srioRtrPortCfg[3].creditSourceQueue = creditPool23;

    srioRtrPortCfg[3].creditReturnQueue = creditPool3host;

    /* Configure the ports */
    for (port = 0; port < TEST_PORTS; port++)
    {
        check_result ("cfg port", Qmss_setSrioRtrPortCfg (TEST_PDSP, port, 0, &srioRtrPortCfg[port]));
        memset (&srioRtrPortCfgRB, 0, sizeof(srioRtrPortCfgRB));
        check_result ("readback port", Qmss_getSrioRtrPortCfg (TEST_PDSP, port, 0, &srioRtrPortCfgRB));
        check_memory ("check port", &srioRtrPortCfg[port], &srioRtrPortCfgRB, sizeof(srioRtrPortCfgRB));
    }
    /* Put packets in the credit pools */
    /* Note: test case ignores psinfo and buffer.  Real SRIO would need these set to some payload and routing info */
    for (i = 0; i < NUM_CREDIT_PACKETS; i++)
    {
        void *descPtr;
        check_null ("pool01 fill pop", descPtr = Qmss_queuePop (freeQ));
        Qmss_queuePushDescSize (creditPool01, descPtr, SIZE_HOST_DESC);
        check_null ("pool12 fill pop", descPtr = Qmss_queuePop (freeQ));
        Qmss_queuePushDescSize (creditPool12, descPtr, SIZE_HOST_DESC);
        check_null ("pool23 fill pop", descPtr = Qmss_queuePop (freeQ));
        Qmss_queuePushDescSize (creditPool23, descPtr, SIZE_HOST_DESC);
        check_null ("pool3host fill pop", descPtr = Qmss_queuePop (freeQ));
        Qmss_queuePushDescSize (creditPool3host, descPtr, SIZE_HOST_DESC);
    }
    /* Put packets in the input queue. */
    for (i = 0; i < PROFILE_DESCS; i++)
    {
        void *descPtr, *cleanDescPtr;
        uint8_t *bufAddr;
        /* Switching would really put final dest (8) here, but since its all
         * using 1 table, need to have fake srio port increment this
         */
        uint32_t PSINFO[2] = { 9, 0x87654321 };
        check_null ("pool01 fill pop", descPtr = Qmss_queuePop (freeQ));
        cleanDescPtr = (void *)QMSS_DESC_PTR (descPtr);
        bufAddr = (uint8_t *)l2_global_address ((uint32_t)&buffers[i*SIZE_BUF]);
        *(volatile uint32_t *)bufAddr = 9;
        Cppi_setData (Cppi_DescType_HOST, cleanDescPtr, bufAddr, SIZE_BUF);
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, cleanDescPtr, bufAddr, SIZE_BUF);
        Cppi_setPacketLen (Cppi_DescType_HOST, cleanDescPtr, SIZE_BUF);
        Cppi_setPSData (Cppi_DescType_HOST, cleanDescPtr, (uint8_t *)PSINFO, 8);
        Qmss_queuePushDescSize (fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_HOST_QUEUE(0)], descPtr, SIZE_HOST_DESC);
    }
    /* Set routing table */
    memset (&routeTbl, 0, sizeof(routeTbl));
    /* daisy chain it */
    routeTbl.forwardToPort[9] = 2;
    routeTbl.forwardToPort[10] = 3;
    routeTbl.forwardToPort[11] = 0;
    routeTbl.isFinalDest[11] = 1;
    /* Set it */
    check_result ("set routing table", Qmss_setSrioRtrGblRouteTbl (TEST_PDSP, 0, &routeTbl));
    check_result ("read routing table", Qmss_getSrioRtrGblRouteTbl (TEST_PDSP, 0, &routeTblRB));
    check_memory ("compare routing table", &routeTbl, &routeTblRB, sizeof(routeTbl));
#ifdef VERBOSE
    printSrioRtrCfg();
#endif
    /* Enable the ports */
    check_result ("enable ports", Qmss_enableSrioRtrPorts (TEST_PDSP, 0xf));
#ifdef _TMS320C6X
    startTime = TSCL;
#else
    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(startTime));
#endif

    /* Drain SW queues and benchmark */
    for (i = 0, nextCredit = 0; i < PROFILE_DESCS; )
    {
        void *descPtr;

        /* Feed port 3 credit -- not needed with real srio */
        if (i == nextCredit)
        {
            if (descPtr = Qmss_queuePop (creditPool3host))
            {

                Qmss_queuePushDesc (fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_CREDIT_QUEUE(3)], descPtr);
                nextCredit += srioRtrGblCfg.creditSize;
            }
        }

        if ( timestamps[i].descPtr = Qmss_queuePop (hostRxQueue))
        {
           void *cleanDescPtr = (void *)QMSS_DESC_PTR (timestamps[i].descPtr);
           uint32_t descSize = QMSS_DESC_SIZE(timestamps[i].descPtr);
           volatile uint32_t *checkPSInfo;
           uint32_t routeLen;
#ifdef _TMS320C6X
           timestamps[i].timestamp = TSCL;
#else
           __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(timestamps[i].timestamp));
#endif
           Cppi_getPSData (Cppi_DescType_HOST, Cppi_PSLoc_PS_IN_DESC, cleanDescPtr, (uint8_t **)&checkPSInfo, &routeLen);
           if (! (checkPSInfo[1] & 0x1))
           {
               System_printf("error: final packet has LSB of mailbox id (t11) or cos (t9) clear\n");
               errorCount++;
           }
           if (descSize != SIZE_HOST_DESC)
           {
               System_printf("error: lost descSize (got %d, expect %d)\n", descSize, SIZE_HOST_DESC);
               errorCount++;
           }
           i++;
           if (i == PROFILE_DESCS)
           {
               System_printf ("packet %d: PS[0] = 0x%08x, PS[1] = 0x%08x\n", i, checkPSInfo[0], checkPSInfo[1]);
           }
        }

        /* forward betwen 1 and 2 */
        spoof_srio_fwd (fakeSrioTxQueue12, fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_FWD_QUEUE(2)]);
        /* forward betwen 2 and 3 */
        spoof_srio_fwd (fakeSrioTxQueue23, fwQHnds[QMSS_SRIORTR_INPUT_QUEUEOFFSET_FWD_QUEUE(3)]);
    }

    /* Check credit packets */
    for (i = 0; i < NUM_CREDIT_PACKETS; i++)
    {
        drain_1_desc ("pool01 drain pop", (creditPool01));
        drain_1_desc ("pool12 drain pop", (creditPool12));
        drain_1_desc ("pool23 drain pop", (creditPool23));
        drain_1_desc ("pool3host drain pop", (creditPool3host));
    }

    /* Check stats */
    srioRtrStatsCheck.packetsRx = 0;
    srioRtrStatsCheck.packetsTx = PROFILE_DESCS; /* b/c port 3 "sends" to port 0 to send to host */
    srioRtrStatsCheck.creditsTx = 0;
    srioRtrStatsCheck.creditsRx = PROFILE_DESCS/srioRtrGblCfg.creditSize + 1;
    srioRtrStatsCheck.hostPkts  = PROFILE_DESCS;
    srioRtrStatsCheck.fwdRets   = 0;
    check_stats (0, &srioRtrStatsCheck);

    srioRtrStatsCheck.packetsRx = PROFILE_DESCS;
    srioRtrStatsCheck.packetsTx = 0;
    srioRtrStatsCheck.creditsTx = PROFILE_DESCS/srioRtrGblCfg.creditSize + 1;
    srioRtrStatsCheck.creditsRx = PROFILE_DESCS/srioRtrGblCfg.creditSize + 1;
    srioRtrStatsCheck.hostPkts  = 0;
    srioRtrStatsCheck.fwdRets   = PROFILE_DESCS;
    check_stats (1, &srioRtrStatsCheck);

    srioRtrStatsCheck.packetsRx = PROFILE_DESCS;
    srioRtrStatsCheck.packetsTx = PROFILE_DESCS;
    srioRtrStatsCheck.creditsTx = PROFILE_DESCS/srioRtrGblCfg.creditSize + 1;
    srioRtrStatsCheck.creditsRx = PROFILE_DESCS/srioRtrGblCfg.creditSize + 1;
    srioRtrStatsCheck.hostPkts  = 0;
    srioRtrStatsCheck.fwdRets   = PROFILE_DESCS;
    check_stats (2, &srioRtrStatsCheck);

    srioRtrStatsCheck.packetsRx = PROFILE_DESCS;
    srioRtrStatsCheck.packetsTx = PROFILE_DESCS;
    srioRtrStatsCheck.creditsTx = PROFILE_DESCS/srioRtrGblCfg.creditSize + 1;
    srioRtrStatsCheck.creditsRx = PROFILE_DESCS/srioRtrGblCfg.creditSize;
    srioRtrStatsCheck.hostPkts  = 0;
    srioRtrStatsCheck.fwdRets   = PROFILE_DESCS;
    check_stats (3, &srioRtrStatsCheck);

    dTime = timestamps[PROFILE_DESCS-1].timestamp - startTime;
    System_printf ("Received %d descriptors in %d cycles (%d pps at 1.2G)\n",
                   PROFILE_DESCS, dTime,
                   1200000000 / dTime * PROFILE_DESCS * TEST_PORTS);

#ifdef VERBOSE
    printSrioRtrStats();
#endif

    if (errorCount == 0)
    {
        System_printf ("\nCore %d : SRIO rtr tests Passed\n", corenum);
    }
    else
    {
        System_printf ("\nCore %d : SRIO rtr tests *failed* (%d errors)\n", corenum, errorCount);
    }
}

Void run_test (Void)
{
    testSrioRtr ();
}


