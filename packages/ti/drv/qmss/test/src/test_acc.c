/**
 *   @file  test_acc.c
 *
 *   @brief
 *      Accumulator test file (limited to new features)
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014-2017, Texas Instruments, Inc.
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

/* XDC includes */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <xdc/cfg/global.h>

#include <string.h>

/* sysbios includes */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

/* CSL RL includes */
#include <ti/csl/csl_chip.h>
#ifdef _TMS320C6X
#include <ti/csl/csl_cacheAux.h>
#endif
/* Queue Reclamation test and firmware is verfied
 * is done only on K2H SoC as the firmware is
 * same for all the devices and no need to test
 * on all possible SoCs
 */
#if defined(DEVICE_K2H) || defined(SOC_K2H)

#ifdef _TMS320C6X
/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/cppi/test/src/cppi_osal.h>

#include <ti/osal/CacheP.h>
IHeap_Handle            cppiHeap;

//The following definitions are for the barrier usage test
#define ACC_Q              704
#define ACC_FDQ            750
#define TX_Q               800
#define TX_FREE_Q          1000
#define RX_FREE_Q          2000
#define BARRIER_Q          2100
#define DEST_Q             2200
#define BAD_Q              2300
#define BARRIER_PKTS       100000
#define DESC_ADDR         (uint32_t *)0x60000000 /* DDR3B */
#define DESC_BUF_ADDR     (uint8_t  *)0x70000000 /* DDR3B */
#define DESC_BUF_SIZE      2048 /* bytes */
#define BENCH_BUF_SIZE     16

#define CLOCKS_PER_PERIOD  98304 /* assuming 983Mhz CPU, this is a 0.1ms period */
#define PERIODS_PER_SEC    10000

/* CPDMA configuration */
Cppi_CpDmaInitCfg       cpdmaCfg;
/* Tx channel configuration */
Cppi_TxChInitCfg        txChCfg;
Cppi_ChHnd              txChHnd;
/* Rx channel configuration */
Cppi_RxChInitCfg        rxChCfg;
Cppi_ChHnd              rxChHnd;
/* Rx flow configuration */
Cppi_RxFlowCfg          rxFlowCfg;
/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

#endif /* C66x */
#else
/************************ USER DEFINES ********************/

#define NUM_DATA_BUFFER             32
#define SIZE_DATA_BUFFER            64

#define MAPPED_VIRTUAL_ADDRESS      0x81000000

/* MPAX segment 2 registers */
#define XMPAXL2                     0x08000010
#define XMPAXH2                     0x08000014

#endif //If a K2 device

/*********** USER DEFINES for all SoCs *********************/
#define NUM_HOST_DESC (256)
#define SIZE_HOST_DESC (64)
#define NUM_MONOLITHIC_DESC         256
#define SIZE_MONOLITHIC_DESC        160



/************************ GLOBAL VARIABLES ********************/
/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;

/* Queue handles for close */
Qmss_QueueHnd           handles[QMSS_MAX_QUEUES_SS_0];
uint32_t                handle_idx = 0;

/* Accumulator list */
#define MAX_LIST_SIZE 80
#ifdef _TMS320C6X
#pragma DATA_ALIGN (hiPrioList, 16)
Uint32                  hiPrioList[MAX_LIST_SIZE];
#else
Uint32                  hiPrioList[MAX_LIST_SIZE] __attribute__ ((aligned (16)));
#endif

/************************ EXTERN VARIABLES ********************/

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/*************************** FUNCTIONS ************************/

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

    if ( (addr >= 0x800000) && (addr <= 0x1000000))
    {
        /* Get the core number. */
        corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
        /* Compute the global address. */
        return (addr + (0x10000000 + (corenum * 0x1000000)));
    }
    else
    {
        return addr;
    }
#else
    return addr;
#endif
}

/* These only apply to K2H devices */
#if defined(DEVICE_K2H) || defined(SOC_K2H)

#ifdef _TMS320C6X
#pragma DATA_ALIGN (hostDescBuf, 128);
#pragma DATA_SECTION (hostDescBuf, ".far:hostDescBuf");
uint8_t hostDescBuf[SIZE_HOST_DESC * NUM_HOST_DESC];


/* This function reads a QMSS INTD Int Count Register.
 * Reading has no effect on the register.
 * "intnum" is:  0..31 for High Pri interrupts
 *              32..47 for Low Pri interrupts
 *              48..49 for PKTDMA Starvation interrupts
 */
uint32_t intd_read_intcount(uint32_t intd, uint32_t intnum)
{
  uint32_t *reg;
  uint32_t  value;
  uint32_t  offset = (intd - 1) * 0x100 + 0x300 + (intnum * 4);

  reg = (uint32_t *)(CSL_QMSS_CFG_INTD_1_REGS + offset);

  value = *reg;

  return(value);
}


void benchmark_barrier (Qmss_QueueHnd freeQ, Qmss_QueueHnd barrierQ, const char *name)
{
    uint32_t start, stop;
    int32_t cnt;
    void *desc;
    TSCL = 0;

    start = TSCL;
    for (cnt = 0; cnt < 10000; )
    {
       if (desc = Qmss_queuePop (freeQ))
       {
           Qmss_queuePushDesc (barrierQ, desc);
           cnt++;
       }
    }
    while (Qmss_getQueueEntryCount (freeQ) != NUM_HOST_DESC);
    stop = TSCL;

    System_printf ("Took %d cycles to move 10000 descs through %s barrier\n", (int)(stop - start), name);
    System_printf ("Assuming 983Mhz clock, this is %d desc/sec\n", 983000000 / ((stop - start) / 10000));
}


extern uint32_t errorCount;

/* This function is written to produce a single benchmark, depending on these
 * macros:
 *   NO_ACCUMULATOR - If defined, the Accumulator is bypassed, allowing timing of
 *                    the barrier processing in isolation.
 *   NO_BARRIER -     If defined, the barrier queue is not programmed.  Allows timing of
 *                    Accumulator and pktDMA.
 *   ARM_CORE -       Includes a check for descriptors arriving in the wrong destination
 *                    queue. This does not happen on C66x, so it should not be defined.
 */
void benchmark_acc_with_barrier(void)
{
    Qmss_Result     reg0, reg1, retVal;
    Qmss_MemRegInfo memInfo;
    Qmss_DescCfg    descCfg;
    Qmss_AccCmdCfg  cfg;
    Qmss_QueueHnd   rxfdq, txfdq, accfdq;
    Qmss_QueueHnd   accQ, ddrQ, destQ, badQ;
    uint32_t        start, next, stop;
    uint32_t        cur_time, desc_time;
    uint32_t        numAllocated;
    uint32_t        pingpong;
    uint32_t        lat_time;
    uint32_t        lat_min, bar_min;
    uint32_t        lat_max, bar_max;
    uint32_t        lat_avg, bar_avg;
    uint32_t        lat_sum, bar_sum;
    uint32_t        acc_pkts, bar_pkts;
    int32_t         idx, iter;
    int32_t         err1_cnt, err2_cnt;
    int32_t         cnt;
    uint8_t         isAllocated;
    uint8_t        *buffAddr;
    uint32_t       *desc;
    uint32_t       *list;
    Cppi_Handle     cppiHnd;
    Cppi_FlowHnd    rxFlowHnd;
    Cppi_DescTag    tag;
    Qmss_Queue      que;
    Cppi_Result     result;

    /* Initialize CPPI LLD */
    result = Cppi_init (&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        System_printf ("Error : Initializing CPPI LLD error code : %d\n", result);
        errorCount++;
    }

    /* Set up QMSS CPDMA configuration */
    memset ((void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
    cpdmaCfg.dmaNum = Cppi_CpDma_QMSS_CPDMA;

    /* Open QMSS CPDMA */
    cppiHnd = (Cppi_Handle) Cppi_open (&cpdmaCfg);
    if (cppiHnd == NULL)
    {
        errorCount++;
        System_printf ("Error : Initializing QMSS CPPI CPDMA %d\n", cpdmaCfg.dmaNum);
        return;
    }

    rxChCfg.channelNum = 0;
    rxChCfg.rxEnable = Cppi_ChState_CHANNEL_ENABLE;
    rxChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
    if (rxChHnd == NULL)
    {
        System_printf ("Error : Opening Rx channel : %d\n", rxChCfg.channelNum);
        errorCount++;
    }

    txChCfg.channelNum = 0;
    txChCfg.txEnable = Cppi_ChState_CHANNEL_ENABLE;
    txChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
    if (txChHnd == NULL)
    {
        System_printf ("Error : Opening Tx channel : %d\n", txChCfg.channelNum);
        errorCount++;
    }

    /* Configure Rx flow 0 */
    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));
    rxFlowCfg.rx_dest_qmgr = 0;
    rxFlowCfg.rx_dest_qnum = BARRIER_Q;
    rxFlowCfg.rx_dest_tag_hi = DEST_Q >> 8; //This will be read by the Acc firmware
    rxFlowCfg.rx_dest_tag_lo = DEST_Q & 0xff;
    rxFlowCfg.rx_dest_tag_lo_sel = 1;
    rxFlowCfg.rx_dest_tag_hi_sel = 1;
    rxFlowCfg.rx_fdq0_sz0_qmgr = 0;
    rxFlowCfg.rx_fdq0_sz0_qnum = RX_FREE_Q;
    rxFlowCfg.rx_fdq1_qmgr = 0;
    rxFlowCfg.rx_fdq1_qnum = RX_FREE_Q;
    rxFlowCfg.rx_psinfo_present = 1;
    rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
    if (rxFlowHnd == NULL)
    {
        System_printf ("Error : Opening Rx flow : %d\n", rxFlowCfg.flowIdNum);
        errorCount++;
    }

    /* Setup memory region for barrier host descriptors */
    memInfo.descBase = DESC_ADDR;
    memset ((void *) memInfo.descBase, 0, (SIZE_HOST_DESC * NUM_HOST_DESC));
    memInfo.descSize = SIZE_HOST_DESC;
    memInfo.descNum  = NUM_HOST_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;
    memInfo.queueGroup = 0;

    reg0 = Qmss_insertMemoryRegion (&memInfo);
    if (reg0 < QMSS_SOK)
    {
        System_printf ("Error : Inserting memory region %d error code : %d\n", memInfo.memRegion, reg0);
        errorCount++;
    }

    /* Setup memory region for accumulator host descriptors */
    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) hostDescBuf);
    memset ((void *) memInfo.descBase, 0, (SIZE_HOST_DESC * NUM_HOST_DESC));
    memInfo.descSize = SIZE_HOST_DESC;
    memInfo.descNum  = NUM_HOST_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = NUM_HOST_DESC;
    memInfo.queueGroup = 0;

    reg1 = Qmss_insertMemoryRegion (&memInfo);
    if (reg1 < QMSS_SOK)
    {
        System_printf ("Error : Inserting memory region %d error code : %d\n", memInfo.memRegion, reg1);
        errorCount++;
    }

    memset (&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = reg0;
    descCfg.descNum = NUM_HOST_DESC;
    descCfg.destQueueNum = TX_FREE_Q;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;

    /* Initialize the descriptors and push to free Queue */
    if ((txfdq = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error: Initializing descriptor error code: %d \n", txfdq);
        errorCount++;
        return;
    }
    if (numAllocated != NUM_HOST_DESC)
    {
        System_printf ("Error: Missing descriptors (found %d of %d)\n", numAllocated, NUM_HOST_DESC);
        errorCount++;
        return;
    }

    //Move half the descriptors to RX_FREE_Q
    if ((rxfdq = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                RX_FREE_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open Rx Free queue: %d\n", rxfdq);
        errorCount++;
        return;
    }

    tag.srcTagHi = 0;
    tag.srcTagLo = 0;
    tag.destTagHi = BAD_Q >> 8;
    tag.destTagLo = BAD_Q & 0xff;
    que.qMgr = 0;
    que.qNum = DEST_Q;
    buffAddr = DESC_BUF_ADDR + (NUM_HOST_DESC/2 * BENCH_BUF_SIZE);

    for (cnt = 0; cnt < NUM_HOST_DESC/2; cnt++)
    {
        desc = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (TX_FREE_Q));
        if (desc)
        {
            Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc *)desc,
                                    buffAddr, BENCH_BUF_SIZE);
            Cppi_setTag (Cppi_DescType_HOST, (Cppi_Desc *)desc, &tag);
            Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)desc, que);
            Qmss_queuePushDesc (RX_FREE_Q, desc);
        }
        buffAddr += BENCH_BUF_SIZE;
    }

    //Update the descriptors in TX_FREE_Q
    que.qMgr = 0;
    que.qNum = TX_FREE_Q;
    buffAddr = DESC_BUF_ADDR;

    for (cnt = 0; cnt < NUM_HOST_DESC; cnt++)
    {
        /* Put freeQ in dest tag */
        desc = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (TX_FREE_Q));
        if (desc)
        {
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)desc,
                          buffAddr, BENCH_BUF_SIZE);
            Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)desc, BENCH_BUF_SIZE);
            Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)desc, que);
            Cppi_setPSLen (Cppi_DescType_HOST, (Cppi_Desc *)desc, 4);
            Qmss_queuePushDesc (TX_FREE_Q, desc);
        }
        else
        {
            System_printf ("Error: got NULL desc %d \n", cnt);
            errorCount++;
            return;
        }
        buffAddr += BENCH_BUF_SIZE;
    }

    memset (&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = reg1;
    descCfg.descNum = NUM_HOST_DESC;
    descCfg.destQueueNum = ACC_FDQ;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;

    /* Initialize the descriptors in the accumulator FDQ */
    if ((accfdq = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error: Initializing Acc desc error code: %d \n", accfdq);
        errorCount++;
        return;
    }
    if (numAllocated != NUM_HOST_DESC)
    {
        System_printf ("Error: Missing Acc desc (found %d of %d)\n", numAllocated, NUM_HOST_DESC);
        errorCount++;
        return;
    }

    //Open the Accumulator's queue to monitor
    if ((accQ = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                ACC_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open Acc queue: %d\n", accQ);
        errorCount++;
        return;
    }

    memset ((void *) &hiPrioList, 0, sizeof (hiPrioList));
    cfg.channel = 0;
    cfg.command = Qmss_AccCmd_ENABLE_CHANNEL;
    cfg.queueEnMask = 0;
    cfg.listAddress = l2_global_address ((uint32_t) hiPrioList);
    /* Get queue manager and queue number from handle */
    cfg.queMgrIndex = Qmss_getQIDFromHandle (accQ);
    cfg.maxPageEntries = 16 + 1;
    cfg.timerLoadCount = 0;
    cfg.interruptPacingMode = Qmss_AccPacingMode_NONE; //interrupt at full list
    cfg.listEntrySize = Qmss_AccEntrySize_REG_D;
    cfg.listCountMode = Qmss_AccCountMode_ENTRY_COUNT;
    cfg.multiQueueMode = Qmss_AccQueueMode_SINGLE_QUEUE;

    if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg)) != QMSS_ACC_SOK)
    {
        System_printf ("Error programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                       cfg.channel, cfg.queMgrIndex, result);
        errorCount++;
    }

    /* Write back the DDR */
//    CacheP_wb (hostDescBuf, sizeof(hostDescBuf));

    if ((destQ = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                DEST_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open destination queue: %d\n", destQ);
        errorCount++;
        return;
    }

    if ((badQ = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                BAD_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open illegal-dest queue: %d\n", badQ);
        errorCount++;
        return;
    }

    if ((ddrQ = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                               BARRIER_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open fw queue ddr barrier: %d\n", ddrQ);
        errorCount++;
        return;
    }

#ifndef NO_BARRIER
    retVal = Qmss_programDDRBarrierQueue (Qmss_PdspId_PDSP1, ddrQ, 0);
    if (retVal != QMSS_ACC_SOK)
    {
        System_printf ("Error: failed to program DDR barrier queue: %d\n", retVal);
        errorCount++;
        return;
    }
#endif

    iter = 0;
    err1_cnt = 0;
    err2_cnt = 0;
    pingpong = 0;
    acc_pkts = bar_pkts= 0;
    lat_sum  = bar_sum = 0;
    lat_min  = bar_min = 0x7fffffff;
    lat_max  = bar_max = 0;
    TSCL = 0; //if this isn't the first set, it won't actually zero it.
    TSCH = 0;

    start = TSCL;
    next  = start + CLOCKS_PER_PERIOD;

    while (iter < PERIODS_PER_SEC) //we will ignore the first two
    {
      //Since the main loop will iterate once each 0.1ms, pushing x descriptors
      //here will create a push rate of x0,000 descriptors per second.  Set
      //this loop limit to the maximum of desired Acc or Barrier packet rate.
      for (idx = 0; idx < 20; idx++)
      {
#ifndef NO_BARRIER
        //Push descriptors to the Tx pktDMA barrier, channel 0
        if (idx < 20)
        {
          desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop(TX_FREE_Q));
          if (desc)
          {
            desc[8] = TSCL;
            Qmss_queuePushDescSizeRaw(TX_Q, desc, 48);
          }
          else //count starvations
            err2_cnt ++;
        }
#endif

#ifndef NO_ACCUMULATOR
        //Push descriptors to the Accumulator.
        if (idx < 5)
        {
          desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop(ACC_FDQ));
          //And tag descriptor PS word with the CPU time of the push.
          if (desc)
          {
            desc[8] = TSCL;
            Qmss_queuePushDesc (ACC_Q, desc);
          }
          else //count starvations
            err1_cnt ++;
        }
#endif
      }

      //While waiting for this time period to elapse, check all the
      //output queues and recycle appropriately.

      cur_time = TSCL;
      while (cur_time < next)
      {
#ifndef NO_ACCUMULATOR
        uint32_t value;

        //Check for an accumulator interrupt.
        value = intd_read_intcount(1, 0);
        if (value == 1) //an interrupt occurred
        {
          if (pingpong == 0) //ping
          {
            list = &hiPrioList[0];
            pingpong = 1;
          }
          else
          {
            list = &hiPrioList[17];
            pingpong = 0;
          }

          cnt = list[0];
          for (idx = 0; idx < cnt; idx++)
          {
            desc = (uint32_t *)list[idx+1];
            if (desc)
            {
              cur_time  = TSCL;
              desc_time = desc[8];

              lat_time = (uint32_t)(cur_time - desc_time);
              lat_sum += lat_time;
              if (lat_time < lat_min)
                lat_min = lat_time;
              if (lat_time > lat_max)
                lat_max = lat_time;

              acc_pkts ++;
              Qmss_queuePushDesc (ACC_FDQ, desc);
            }
          }

          //reset the interrupt.
          result = Qmss_ackInterrupt (0, 1);
          result = Qmss_setEoiVectorByIntd (0, Qmss_IntdInterruptType_HIGH, 0);
        }
#endif

#ifndef NO_BARRIER
        //Process descriptors correctly arriving at the destination queue.
        cnt = Qmss_getQueueEntryCount(destQ);
        for (idx = 0; idx < cnt; idx++)
        {
          desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop(DEST_Q));
          if (desc)
          {
            //Write the illegal tag (bad queue number) back to the descriptor
            tag.destTagHi = BAD_Q >> 8;
            tag.destTagLo = BAD_Q & 0xff;
            Cppi_setTag (Cppi_DescType_HOST, (Cppi_Desc *)desc, &tag);

            cur_time  = TSCL;
            desc_time = desc[8];

            lat_time = (uint32_t)(cur_time - desc_time);
            bar_sum += lat_time;
            if (lat_time < bar_min)
              bar_min = lat_time;
            if (lat_time > bar_max)
              bar_max = lat_time;

            bar_pkts ++;
            Qmss_queuePushDesc (RX_FREE_Q, desc);
          }
        }

#ifdef ARM_CORE
        //A descriptor arriving in this queue (badQ) indicates that the firmware
        //read the tag before the write from the Rx pktDMA had landed.  This is
        //an error.  This failure should not occur on a DSP core, only an ARM.
        cnt = Qmss_getQueueEntryCount(badQ);
        for (idx = 0; idx < cnt; idx++)
        {
          desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop(BAD_Q));
          if (desc)
          {
            err2_cnt ++;

            //Make sure the bad queue number is back in the descriptor
            tag.destTagHi = BAD_Q >> 8;
            tag.destTagLo = BAD_Q & 0xff;
            Cppi_setTag (Cppi_DescType_HOST, (Cppi_Desc *)desc, &tag);
            Qmss_queuePushDesc (RX_FREE_Q, desc);
          }
        }
#endif
#endif
        cur_time = TSCL;
      }

      next += CLOCKS_PER_PERIOD;
      iter ++;
    }

    stop    = TSCL;
    lat_avg = lat_sum / acc_pkts;
    bar_avg = bar_sum / bar_pkts;

    if (err1_cnt > 0)
      System_printf ("Acc FDQ starved %d times\n", err1_cnt);

    if (err2_cnt > 0)
      System_printf ("Barrier FDQ starved %d times\n", err2_cnt);

    System_printf ("Ran %d cycles, moved %d descs through Accumulator and %d through barrier\n",
                  (int)(stop - start), acc_pkts, bar_pkts);
    System_printf ("Minimum Acc latency: %d \n", lat_min);
    System_printf ("Average Acc latency: %d \n", lat_avg);
    System_printf ("Maximum Acc latency: %d \n", lat_max);
    System_printf ("Minimum Bar latency: %d \n", bar_min);
    System_printf ("Average Bar latency: %d \n", bar_avg);
    System_printf ("Maximum Bar latency: %d \n", bar_max);

    cnt = Qmss_getQueueEntryCount(rxfdq);
    if (cnt != NUM_HOST_DESC/2)
      System_printf ("Error : Rx FDQ ended with %d descs, should be %d\n", cnt, NUM_HOST_DESC/2);

    Qmss_queueEmpty(rxfdq);
    Qmss_queueEmpty(txfdq);
    Qmss_queueEmpty(accfdq);


    /* unprogram accumulator */
    if ( (result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, cfg.channel)) != QMSS_ACC_SOK)
    {
        errorCount++;
        System_printf ("Error disabling accumulator: %d \n", result);
    }

    /* Disable DDR Q */
    retVal = Qmss_programDDRBarrierQueue (Qmss_PdspId_PDSP1, 0, 0);

    if (retVal != QMSS_ACC_SOK)

    {
        System_printf ("Error : failed to unprogram DDR barrier queue: %d\n", retVal);
        errorCount++;
        return;
    }

    if ( (retVal = Qmss_queueClose (txfdq)) != QMSS_SOK)
    {
        System_printf ("Error : closing txfdq %d: %d\n", txfdq, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (rxfdq)) != QMSS_SOK)
    {
        System_printf ("Error : closing rxfdq %d: %d\n", rxfdq, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (ddrQ)) != QMSS_SOK)
    {
        System_printf ("Error : closing ddrQ %d: %d\n", ddrQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (destQ)) != QMSS_SOK)
    {
        System_printf ("Error : closing destQ %d: %d\n", destQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (badQ)) != QMSS_SOK)
    {
        System_printf ("Error : closing badQ %d: %d\n", badQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (accQ)) != QMSS_SOK)
    {
        System_printf ("Error : closing accQ %d: %d\n", accQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (accfdq)) != QMSS_SOK)
    {
        System_printf ("Error : closing accfdq %d: %d\n", accfdq, retVal);
        errorCount++;
    }

    if ( (retVal = Qmss_removeMemoryRegion (reg0, 0)) != QMSS_SOK)
    {
        System_printf ("Error : removing memory region 0: %d\n", retVal);
        errorCount++;
    }

    if ( (retVal = Qmss_removeMemoryRegion (reg1, 0)) != QMSS_SOK)
    {
        System_printf ("Error : removing memory region 1: %d\n", retVal);
        errorCount++;
    }

    /* Close Tx channel */
    Cppi_channelDisable (txChHnd);
    if ((result = Cppi_channelClose (txChHnd)) != CPPI_SOK)
    {
        System_printf ("Error : Closing Tx channel error code : %d\n", result);
        errorCount++;
    }

    /* Close Rx channel */
    Cppi_channelDisable (rxChHnd);
    if ((result = Cppi_channelClose (rxChHnd)) != CPPI_SOK)
    {
        System_printf ("Error : Closing Rx channel error code : %d\n", result);
        errorCount++;
    }
}


void test_barrier (void)
{
    Qmss_Result     reg0, retVal;
    Qmss_MemRegInfo memInfo;
    Qmss_DescCfg    descCfg;
    Qmss_QueueHnd   freeQ, ddrInfQ, msmcInfQ;
    Qmss_QueueHnd   ddrNetCPQ, msmcNetCPQ;
    extern uint32_t errorCount;
    uint32_t        numAllocated;
    int32_t         cnt;
    uint8_t         isAllocated;


    /* Setup memory region for monolithic descriptors */
    memset ((void *) hostDescBuf, 0, (SIZE_HOST_DESC * NUM_HOST_DESC));
    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) hostDescBuf);
    memInfo.descSize = SIZE_HOST_DESC;
    memInfo.descNum = NUM_HOST_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    reg0 = Qmss_insertMemoryRegion (&memInfo);
    if (reg0 < QMSS_SOK)
    {
        System_printf ("Error : Inserting memory region %d error code : %d\n", memInfo.memRegion, reg0);
        errorCount++;
    }

    memset (&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = Qmss_MemRegion_MEMORY_REGION0;
    descCfg.descNum = NUM_HOST_DESC;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;

    /* Initialize the descriptors and push to free Queue */
    if ((freeQ = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error: Initializing descriptor error code: %d \n", freeQ);
        errorCount++;
        return;
    }
    if (numAllocated != NUM_HOST_DESC)
    {
        System_printf ("Error: Missing descriptors (found %d of %d)\n", numAllocated, NUM_HOST_DESC);
        errorCount++;
        return;
    }

    for (cnt = 0; cnt < NUM_HOST_DESC; cnt++)
    {
        /* Put freeQ in dest tag */
        uint32_t *desc = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (freeQ));
        if (desc)
        {
            desc[1] = freeQ;
            Qmss_queuePushDesc (freeQ, desc);
        }
        else
        {
            System_printf ("Error: got NULL desc %d \n", cnt);
            errorCount++;
            return;
        }
    }
    /* Write back the DDR */
    CacheP_wb (hostDescBuf, sizeof(hostDescBuf));

    if ((ddrInfQ = Qmss_queueOpen
                      (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open fw queue ddr infra barrier: %d\n", ddrInfQ);
        errorCount++;
        return;
    }
    if ((msmcInfQ = Qmss_queueOpen
                      (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open fw queue msmc infra barrier: %d\n", msmcInfQ);
        errorCount++;
        return;
    }

    if ((ddrNetCPQ = Qmss_queueOpen
                      (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open fw queue ddr netcp barrier: %d\n", ddrNetCPQ);
        errorCount++;
        return;
    }
    if ((msmcNetCPQ = Qmss_queueOpen
                      (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open fw queue msmc netcp barrier: %d\n", msmcNetCPQ);
        errorCount++;
        return;
    }

    if ((retVal = Qmss_programDDRBarrierQueue (Qmss_PdspId_PDSP1, ddrInfQ, ddrNetCPQ)) != QMSS_ACC_SOK)
    {
        System_printf ("Error: failed to program DDR barrier queue: %d\n", retVal);
        errorCount++;
        return;
    }

    if ((retVal = Qmss_programMSMCBarrierQueue (Qmss_PdspId_PDSP1, msmcInfQ, msmcNetCPQ)) != QMSS_ACC_SOK)
    {
        System_printf ("Error: failed to program MSMC barrier queue: %d\n", retVal);
        errorCount++;
        return;
    }

    /* Measure time to send 10000 packets through ddrQ */
    benchmark_barrier (freeQ, ddrInfQ, "ddr infra");

    /* Measure time to send 10000 packets through msmcQ */
    benchmark_barrier (freeQ, msmcInfQ, "msmc infra");

    /* Move return queue from dest tag to swinfo[1] */
    for (cnt = 0; cnt < NUM_HOST_DESC; cnt++)
    {
        /* Remove freeQ from dest tag; put it in swinfo[1] */
        uint32_t *desc = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (freeQ));
        if (desc)
        {
            desc[1] = 0; /* dest tag */
            desc[10] = freeQ; /* swinfo[1] */
            Qmss_queuePushDesc (freeQ, desc);
        }
        else
        {
            System_printf ("Error: got NULL desc %d \n", cnt);
            errorCount++;
            return;
        }
    }

    /* Write back the DDR */
    CacheP_wb (hostDescBuf, sizeof(hostDescBuf));

    /* Measure time to send 10000 packets through ddrNetCPQ */
    benchmark_barrier (freeQ, ddrNetCPQ, "ddr netcp");

    /* Measure time to send 10000 packets through msmcNetCPQ */
    benchmark_barrier (freeQ, msmcNetCPQ, "msmc netcp");

    /* Disable DDR Q */
    if ((retVal = Qmss_programDDRBarrierQueue (Qmss_PdspId_PDSP1, 0, 0)) != QMSS_ACC_SOK)
    {
        System_printf ("Error: failed to unprogram DDR barrier queue: %d\n", retVal);
        errorCount++;
        return;
    }

    /* Disable MSMC Q */
    if ((retVal = Qmss_programMSMCBarrierQueue (Qmss_PdspId_PDSP1, 0, 0)) != QMSS_ACC_SOK)
    {
        System_printf ("Error: failed to program MSMC barrier queue: %d\n", retVal);
        errorCount++;
        return;
    }

    Qmss_queueEmpty(freeQ);

    if ( (retVal = Qmss_queueClose (freeQ)) != QMSS_SOK)
    {
        System_printf ("Error closing freeQ %d: %d\n", freeQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (ddrInfQ)) != QMSS_SOK)
    {
        System_printf ("Error closing ddrInfQ %d: %d\n", ddrInfQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (msmcInfQ)) != QMSS_SOK)
    {
        System_printf ("Error closing msmcInfQ %d: %d\n", msmcInfQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (ddrNetCPQ)) != QMSS_SOK)
    {
        System_printf ("Error closing ddrNetCPQ %d: %d\n", ddrNetCPQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (msmcNetCPQ)) != QMSS_SOK)
    {
        System_printf ("Error closing msmcNetCPQ %d: %d\n", msmcNetCPQ, retVal);
        errorCount++;
    }

    if ( (retVal = Qmss_removeMemoryRegion (reg0, 0)) != QMSS_SOK)
    {
        System_printf ("Error removing memory region 0: %d\n", retVal);
        errorCount++;
    }

}


/* The barrier usage test works like this:

   1) barrierQ is programmed into Acc as the DDR barrier queue.
   2) pktDMA rxFreeQ is initialized, with dest_tag initialized to an incorrect queue.
   3) pktDMA txFreeQ is initialized.
   4) an rx flow is programmed with:
      a) rxFreeQ as the descriptor source,
      b) barrierQ as the destination Q,
      c) dest_tag is to be hardcoded from Rx flow with final destination Rx Q.
   5) Loop for X output descriptors:
      a) Pop N descriptors from txFreeQ, and push to pktDMA Tx Q.
      b) Check for output on final Rx Q:
         1) check dest_tag for final Rx Q, increment an error count if not equal.
         2) write incorrect queue back to dest_tag.
         3) push to rxFreeQ.
      c) Check for output on the incorrect queue (catches missed barrier events).
         1) increment an error count.
         2) push to rxFreeQ.
*/
void test_barrier_usage(void)
{
    Qmss_Result     reg0, retVal;
    Qmss_MemRegInfo memInfo;
    Qmss_DescCfg    descCfg;
    Qmss_QueueHnd   rxfdq, txfdq, ddrQ, destQ, badQ;
    extern uint32_t errorCount;
    uint32_t        numAllocated;
    uint32_t        start, stop;
    int32_t         idx;
    int32_t         err1_cnt, err2_cnt;
    int32_t         cnt, txcnt, rxcnt;
    uint8_t         isAllocated;
    uint8_t        *buffAddr;
    Cppi_Handle     cppiHnd;
    Cppi_FlowHnd    rxFlowHnd;
    Cppi_DescTag    tag;
    Qmss_Queue      que;
    Cppi_Result     result;
    Cppi_HostDesc  *host;

    /* Initialize CPPI LLD */
    result = Cppi_init (&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        System_printf ("Error : Initializing CPPI LLD error code : %d\n", result);
        errorCount++;
    }

    /* Set up QMSS CPDMA configuration */
    memset ((void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
    cpdmaCfg.dmaNum = Cppi_CpDma_QMSS_CPDMA;

    /* Open QMSS CPDMA */
    cppiHnd = (Cppi_Handle) Cppi_open (&cpdmaCfg);
    if (cppiHnd == NULL)
    {
        errorCount++;
        System_printf ("Error : Initializing QMSS CPPI CPDMA %d\n", cpdmaCfg.dmaNum);
        return;
    }

    rxChCfg.channelNum = 0;
    rxChCfg.rxEnable = Cppi_ChState_CHANNEL_ENABLE;
    rxChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
    if (rxChHnd == NULL)
    {
        System_printf ("Error : Opening Rx channel : %d\n", rxChCfg.channelNum);
        errorCount++;
    }

    txChCfg.channelNum = 0;
    txChCfg.txEnable = Cppi_ChState_CHANNEL_ENABLE;
    txChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
    if (txChHnd == NULL)
    {
        System_printf ("Error : Opening Tx channel : %d\n", txChCfg.channelNum);
        errorCount++;
    }

    /* Configure Rx flow 0 */
    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));
    rxFlowCfg.rx_dest_qmgr = 0;
    rxFlowCfg.rx_dest_qnum = BARRIER_Q;
    rxFlowCfg.rx_dest_tag_hi = DEST_Q >> 8; //This will be read by the Acc firmware
    rxFlowCfg.rx_dest_tag_lo = DEST_Q & 0xff;
    rxFlowCfg.rx_dest_tag_lo_sel = 1;
    rxFlowCfg.rx_dest_tag_hi_sel = 1;
    rxFlowCfg.rx_fdq0_sz0_qmgr = 0;
    rxFlowCfg.rx_fdq0_sz0_qnum = RX_FREE_Q;
    rxFlowCfg.rx_fdq1_qmgr = 0;
    rxFlowCfg.rx_fdq1_qnum = RX_FREE_Q;
    rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
    if (rxFlowHnd == NULL)
    {
        System_printf ("Error : Opening Rx flow : %d\n", rxFlowCfg.flowIdNum);
        errorCount++;
    }

    /* Setup memory region for host descriptors */
    memset ((void *) hostDescBuf, 0, (SIZE_HOST_DESC * NUM_HOST_DESC));
    memInfo.descBase = DESC_ADDR;
//    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) hostDescBuf);
    memInfo.descSize = SIZE_HOST_DESC;
    memInfo.descNum  = NUM_HOST_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    reg0 = Qmss_insertMemoryRegion (&memInfo);
    if (reg0 < QMSS_SOK)
    {
        System_printf ("Error : Inserting memory region %d error code : %d\n", memInfo.memRegion, reg0);
        errorCount++;
    }

    memset (&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = reg0;
    descCfg.descNum = NUM_HOST_DESC;
    descCfg.destQueueNum = TX_FREE_Q;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;

    /* Initialize the descriptors and push to free Queue */
    if ((txfdq = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error: Initializing descriptor error code: %d \n", txfdq);
        errorCount++;
        return;
    }
    if (numAllocated != NUM_HOST_DESC)
    {
        System_printf ("Error: Missing descriptors (found %d of %d)\n", numAllocated, NUM_HOST_DESC);
        errorCount++;
        return;
    }

    //Move half the descriptors to RX_FREE_Q
    if ((rxfdq = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                RX_FREE_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open Rx Free queue: %d\n", rxfdq);
        errorCount++;
        return;
    }

    tag.srcTagHi = 0;
    tag.srcTagLo = 0;
    tag.destTagHi = BAD_Q >> 8;
    tag.destTagLo = BAD_Q & 0xff;
    que.qMgr = 0;
    que.qNum = DEST_Q;
    buffAddr = DESC_BUF_ADDR + (NUM_HOST_DESC/2 * DESC_BUF_SIZE);

    for (cnt = 0; cnt < NUM_HOST_DESC/2; cnt++)
    {
        uint32_t *desc = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (TX_FREE_Q));
        if (desc)
        {
            host = (Cppi_HostDesc *)desc;
            Cppi_setOriginalBufInfo(Cppi_DescType_HOST, (Cppi_Desc *)desc,
                                    buffAddr, DESC_BUF_SIZE);
            Cppi_setTag (Cppi_DescType_HOST, (Cppi_Desc *)desc, &tag);
            Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)desc, que);
            Qmss_queuePushDesc (RX_FREE_Q, desc);
        }
        buffAddr += DESC_BUF_SIZE;
    }

    //Update the descriptors in TX_FREE_Q
    que.qMgr = 0;
    que.qNum = TX_FREE_Q;
    buffAddr = DESC_BUF_ADDR;

    for (cnt = 0; cnt < NUM_HOST_DESC; cnt++)
    {
        /* Put freeQ in dest tag */
        uint32_t *desc = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (TX_FREE_Q));
        if (desc)
        {
            host = (Cppi_HostDesc *)desc;
            Cppi_setData (Cppi_DescType_HOST, (Cppi_Desc *)desc,
                          buffAddr, DESC_BUF_SIZE);
            Cppi_setPacketLen (Cppi_DescType_HOST, (Cppi_Desc *)desc, DESC_BUF_SIZE);
            Cppi_setReturnQueue (Cppi_DescType_HOST, (Cppi_Desc *)desc, que);
            Qmss_queuePushDesc (TX_FREE_Q, desc);
        }
        else
        {
            System_printf ("Error: got NULL desc %d \n", cnt);
            errorCount++;
            return;
        }
        buffAddr += DESC_BUF_SIZE;
    }

    /* Write back the DDR */
//    CacheP_wb (hostDescBuf, sizeof(hostDescBuf));

    if ((destQ = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                DEST_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open destination queue: %d\n", destQ);
        errorCount++;
        return;
    }

    if ((badQ = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                BAD_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open illegal-dest queue: %d\n", badQ);
        errorCount++;
        return;
    }

    if ((ddrQ = Qmss_queueOpen(Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                               BARRIER_Q, &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open fw queue ddr barrier: %d\n", ddrQ);
        errorCount++;
        return;
    }

    retVal = Qmss_programDDRBarrierQueue (Qmss_PdspId_PDSP1, ddrQ, 0);
    if (retVal != QMSS_ACC_SOK)
    {
        System_printf ("Error: failed to program DDR barrier queue: %d\n", retVal);
        errorCount++;
        return;
    }

    txcnt = 0;
    rxcnt = 0;
    err1_cnt = 0;
    err2_cnt = 0;
    TSCL = 0; //since this isn't the first set, it doesn't actually zero it.
    start = TSCL;

    while (rxcnt < BARRIER_PKTS)
    {
      //Push descriptors to the Tx pktDMA, channel 0
      for (idx = 0; idx < 16; idx++)
      {
        if (txcnt < BARRIER_PKTS)
        {
          uint32_t *desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop(TX_FREE_Q));
          if (desc)
          {
            //No MFENCE needed, since descriptors are not modified
            Qmss_queuePushDescSizeRaw(TX_Q, desc, 32);
            txcnt++;
          }
        }
      }

      //Process descriptors correctly arriving at the destination queue.
      cnt = Qmss_getQueueEntryCount(destQ);
      for (idx = 0; idx < cnt; idx++)
      {
        uint32_t *desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop(DEST_Q));
        if (desc)
        {
          host = (Cppi_HostDesc *)desc;

          //Examine the dest tag. This test shouldn't fail because the descriptor
          //is in the correct queue, but we'll check for completeness.
          tag = Cppi_getTag(Cppi_DescType_HOST, (Cppi_Desc *)host);
          if ((tag.destTagHi != (DEST_Q >> 8)) ||
              (tag.destTagLo != (DEST_Q & 0xff)))
            err1_cnt ++;

          //Write the illegal tag (bad queue number) back to the descriptor
          tag.destTagHi = BAD_Q >> 8;
          tag.destTagLo = BAD_Q & 0xff;
          Cppi_setTag (Cppi_DescType_HOST, (Cppi_Desc *)desc, &tag);
          Qmss_queuePushDesc (RX_FREE_Q, desc);

          rxcnt++;
        }
      }

      //A descriptor arriving in this queue (badQ) indicates that the firmware
      //read the tag before the write from the Rx pktDMA had landed.  This is
      //an error.  This failure should not occur on a DSP core, only an ARM.
      cnt = Qmss_getQueueEntryCount(badQ);
      for (idx = 0; idx < cnt; idx++)
      {
        uint32_t *desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop(BAD_Q));
        if (desc)
        {
          host = (Cppi_HostDesc *)desc;
          err2_cnt ++;

          //Make sure the bad queue number is back in the descriptor
          tag.destTagHi = BAD_Q >> 8;
          tag.destTagLo = BAD_Q & 0xff;
          Cppi_setTag (Cppi_DescType_HOST, (Cppi_Desc *)desc, &tag);
          Qmss_queuePushDesc (RX_FREE_Q, desc);
        }
      }
    }

    stop = TSCL;
    System_printf ("Took %d cycles to move %d descs through pktDMA and barrier\n", (int)(stop - start), rxcnt);
    System_printf ("Assuming 983Mhz clock, this is %d desc/sec\n", 983000000 / ((stop - start) / 10000));

    if (err1_cnt > 0)
      System_printf ("Error : %d descs received with invalid dest tags\n", err1_cnt);

    if (err2_cnt > 0)
      System_printf ("Error : %d descs routed to erroneous dest queue\n", err2_cnt);

    cnt = Qmss_getQueueEntryCount(rxfdq);
    if (cnt != NUM_HOST_DESC/2)
      System_printf ("Error : Rx FDQ ended with %d descs, should be %d\n", cnt, NUM_HOST_DESC/2);

    Qmss_queueEmpty(rxfdq);
    Qmss_queueEmpty(txfdq);

    /* Disable DDR Q */
    retVal = Qmss_programDDRBarrierQueue (Qmss_PdspId_PDSP1, 0, 0);

    if (retVal != QMSS_ACC_SOK)
    {
        System_printf ("Error : failed to unprogram DDR barrier queue: %d\n", retVal);
        errorCount++;
        return;
    }

    Qmss_queueEmpty(destQ);
    Qmss_queueEmpty(ddrQ);
    Qmss_queueEmpty(badQ);

    if ( (retVal = Qmss_queueClose (txfdq)) != QMSS_SOK)
    {
        System_printf ("Error : closing txfdq %d: %d\n", txfdq, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (rxfdq)) != QMSS_SOK)
    {
        System_printf ("Error : closing rxfdq %d: %d\n", rxfdq, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (ddrQ)) != QMSS_SOK)
    {
        System_printf ("Error : closing ddrQ %d: %d\n", ddrQ, retVal);
        errorCount++;
    }

    if ( (retVal = Qmss_queueClose (destQ)) != QMSS_SOK)
    {
        System_printf ("Error : closing destQ %d: %d\n", destQ, retVal);
        errorCount++;
    }

    if ( (retVal = Qmss_queueClose (badQ)) != QMSS_SOK)
    {
        System_printf ("Error : closing badQ %d: %d\n", badQ, retVal);
        errorCount++;
    }

    if ( (retVal = Qmss_removeMemoryRegion (reg0, 0)) != QMSS_SOK)
    {
        System_printf ("Error : removing memory region 0: %d\n", retVal);
        errorCount++;
    }

    /* Close Tx channel */
    Cppi_channelDisable (txChHnd);
    if ((result = Cppi_channelClose (txChHnd)) != CPPI_SOK)
    {
        System_printf ("Error : Closing Tx channel error code : %d\n", result);
        errorCount++;
    }

    /* Close Rx channel */
    Cppi_channelDisable (rxChHnd);
    if ((result = Cppi_channelClose (rxChHnd)) != CPPI_SOK)
    {
        System_printf ("Error : Closing Rx channel error code : %d\n", result);
        errorCount++;
    }
}


//This function tests a push to head reclamation scenario.
void test_reclamation_to_head(void)
{
    Qmss_Result     reg0, retVal;
    Qmss_MemRegInfo memInfo;
    Qmss_DescCfg    descCfg;
    Qmss_QueueHnd   freeQ, destQ, recQ;
    extern uint32_t errorCount;
    uint32_t        numAllocated;
    int32_t         cnt;
    uint8_t         isAllocated;
    uint32_t       *desc1;
    uint32_t       *desc2;
    uint32_t       *desc3;
    uint32_t       *desc4;

    /* Setup memory region for monolithic descriptors */
    memset ((void *) hostDescBuf, 0, (SIZE_HOST_DESC * NUM_HOST_DESC));
    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) hostDescBuf);
    memInfo.descSize = SIZE_HOST_DESC;
    memInfo.descNum = NUM_HOST_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    reg0 = Qmss_insertMemoryRegion (&memInfo);
    if (reg0 < QMSS_SOK)
    {
        System_printf ("Error : Inserting memory region %d error code : %d\n", memInfo.memRegion, reg0);
        errorCount++;
    }

    memset (&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = Qmss_MemRegion_MEMORY_REGION0;
    descCfg.descNum = NUM_HOST_DESC;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;

    /* Initialize the descriptors and push to free Queue */
    if ((freeQ = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error: Initializing descriptor error code: %d \n", freeQ);
        errorCount++;
        return;
    }
    if (numAllocated != NUM_HOST_DESC)
    {
        System_printf ("Error: Missing descriptors (found %d of %d)\n", numAllocated, NUM_HOST_DESC);
        errorCount++;
        return;
    }

    if ((destQ = Qmss_queueOpen
                      (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open dest queue: %d\n", destQ);
        errorCount++;
        return;
    }
    if ((recQ = Qmss_queueOpen
                      (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated)) < QMSS_SOK)
    {
        System_printf ("Error: Failed to open reclamation queue: %d\n", recQ);
        errorCount++;
        return;
    }

    //Pop two descriptors and initialize for the reclamation queue
    /* Put freeQ in dest tag */
    desc1 = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (freeQ));
    desc2 = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (freeQ));
    if ((desc1 == NULL) || (desc2 == NULL) || (desc1 == desc2))
    {
        System_printf ("Error: reclaim input descriptors bad 0x%x 0x%x\n",
                       desc1, desc2);
        errorCount++;
        return;
    }

    //initialize the words that the firmware will use
    desc1[2] = 0x0000c000 + destQ;
    desc1[5] = (uint32_t) desc2;
    desc2[2] = 0x0000c000 + destQ;
    desc2[5] = 0;

    if ((retVal = Qmss_programReclaimQueue (Qmss_PdspId_PDSP1, recQ)) != QMSS_ACC_SOK)
    {
        System_printf ("Error: failed to program reclamation queue: %d\n", retVal);
        errorCount++;
        return;
    }

    //The two descriptors are linked. Push the head descriptor to the reclamation queue.
    Qmss_queuePushDesc(recQ, desc1);

    for (cnt = 0; cnt < 1000; cnt++)
      asm("   nop 5  ");

    //Now, examine the results:
    cnt = Qmss_getQueueEntryCount(destQ);
    if (cnt != 2)
    {
      System_printf ("Error : Dest Q ended with %d descs, should be %d\n", cnt, 2);
      errorCount++;
    }

    desc3 = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (destQ));
    desc4 = (uint32_t *)QMSS_DESC_PTR (Qmss_queuePop (destQ));

    /* Push head causes descriptors to get reversed */
    if (desc3 != desc2)
    {
      System_printf ("Error : Desc2 did not recycle correctly, 0x%x != 0x%x\n",
                     desc2, desc3);
      errorCount++;
    }

    if (desc4 != desc1)
    {
      System_printf ("Error : Desc1 did not recycle correctly, 0x%x != 0x%x\n",
                     desc1, desc4);
      errorCount++;
    }

    Qmss_queueEmpty(destQ);
    Qmss_queueEmpty(freeQ);

    if ((retVal = Qmss_programReclaimQueue (Qmss_PdspId_PDSP1, 0)) != QMSS_ACC_SOK)
    {
        System_printf ("Error: failed to unprogram reclamation queue: %d\n", retVal);
        errorCount++;
        return;
    }

    if ( (retVal = Qmss_queueClose (freeQ)) != QMSS_SOK)
    {
        System_printf ("Error closing freeQ %d: %d\n", freeQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (destQ)) != QMSS_SOK)
    {
        System_printf ("Error closing destQ %d: %d\n", destQ, retVal);
        errorCount++;
    }
    if ( (retVal = Qmss_queueClose (recQ)) != QMSS_SOK)
    {
        System_printf ("Error closing recQ %d: %d\n", recQ, retVal);
        errorCount++;
    }

    if ( (retVal = Qmss_removeMemoryRegion (reg0, 0)) != QMSS_SOK)
    {
        System_printf ("Error removing memory region 0: %d\n", retVal);
        errorCount++;
    }

}
#endif
#endif

/**
 *  @b Description
 *  @n
 *      Entry point for the test code.
 *      This is an testcase for new features added to the accumulator
 *      after 2014.
 *
 *      It performs the following
 *          - Initialize QM
 *          - Downloads accumulator
 *          - Verifies bounds check for MaxEntries
 *          - Deinitialize QM
 *
 *      For an example that uses the accumulator, see
 *      example/InfraDmaSC.
 *
 *  @retval
 *      Not Applicable.
 */
void accTest (void)
{
    int32_t                 result;
    extern uint32_t         errorCount;
    Qmss_QueueHnd           rxQueHnd;
    Qmss_AccCmdCfg          cfg;
    uint8_t                 isAllocated;

    System_printf ("**************************************************\n");
    System_printf ("************* QMSS accumulator testing ***********\n");
    System_printf ("**************************************************\n");

#ifdef _TMS320C6X
    /* Reset the variable to indicate to other cores system init is not yet done */
    System_printf ("L1D cache size %d. L2 cache size %d.\n", CACHE_getL1DSize(), CACHE_getL2Size());
#endif

    memset ((void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up the linking RAM. Use the internal Linking RAM.
     * LLD will configure the internal linking RAM address and default size if a value of zero is specified.
     * Linking RAM1 is not used */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_HOST_DESC + NUM_MONOLITHIC_DESC;

    /* Select endian dependent firmware version */
#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_le);
#endif

    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error: Initializing Queue Manager SubSystem error code : %d\n", result);
        return;
    }

    /* Start Queue Manager SubSystem */
    result = Qmss_start ();
    if (result != QMSS_SOK)
    {
        System_printf ("Error starting Queue Manager error code : %d\n", result);
    }
    else
    {
        System_printf ("\nQMSS initialization done\n");
    }

    /* Opens receive queue in first QM */
    if ((rxQueHnd = Qmss_queueOpenInGroup (0, Qmss_QueueType_HIGH_PRIORITY_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Opening Receive Queue: %d \n", rxQueHnd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Receive Queue Number : %d opened\n", rxQueHnd);
    }

    /* program the high priority accumulator */
    memset ((void *) &hiPrioList, 0, sizeof (hiPrioList));
    cfg.channel = 0;
    cfg.command = Qmss_AccCmd_ENABLE_CHANNEL;
    cfg.queueEnMask = 0;
    cfg.listAddress = l2_global_address ((uint32_t) hiPrioList); /* Should be global if reading on another core */
    /* Get queue manager and queue number from handle */
    cfg.queMgrIndex = Qmss_getQIDFromHandle (rxQueHnd);
    cfg.maxPageEntries = MAX_LIST_SIZE + 1; /* should fail */
    cfg.timerLoadCount = 0;
    cfg.interruptPacingMode = Qmss_AccPacingMode_NONE;
    cfg.listEntrySize = Qmss_AccEntrySize_REG_D;
    cfg.listCountMode = Qmss_AccCountMode_ENTRY_COUNT;
    cfg.multiQueueMode = Qmss_AccQueueMode_SINGLE_QUEUE;

    /* Negative test case */
    if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg)) != QMSS_ACC_INVALID_MAXENTRIES)
    {
        System_printf ("Programming high priority accumulator for channel : %d queue : %d "
                       "should have failed %d but got error code : %d\n",
                       cfg.channel, cfg.queMgrIndex, QMSS_ACC_INVALID_MAXENTRIES, result);
        errorCount++;
    }
    else
    {
        System_printf ("high priority accumulator negative test passed for channel : %d queue : %d\n",
                       cfg.channel, cfg.queMgrIndex);
    }

    /* Postive test case */
    cfg.maxPageEntries = MAX_LIST_SIZE; /* should fail */
    if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg)) != QMSS_ACC_SOK)
    {
        System_printf ("Error programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                       cfg.channel, cfg.queMgrIndex, result);
        errorCount++;
    }
    else
    {
        System_printf ("high priority accumulator positive test passed for channel : %d queue : %d\n",
                       cfg.channel, cfg.queMgrIndex);
    }

    /* unprogram accumulator */
    if ( (result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, cfg.channel)) != QMSS_ACC_SOK)
    {
        errorCount++;
        System_printf ("Error disabling accumulator: %d \n", result);
    }
    /* close queues */
    if ( (result = Qmss_queueClose(rxQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error closing queue %d QM: %d \n", rxQueHnd, result);
    }

/* These only apply to K2 devices */
#if defined(DEVICE_K2H) || defined(SOC_K2H) 

#ifdef _TMS320C6X
    test_reclamation_to_head();

    test_barrier ();
    test_barrier_usage();

    //See preamble for usage notes.
    benchmark_acc_with_barrier();
#endif
#endif

    System_printf ("exit QM\n");
    if ( (result = Qmss_exit()) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error exiting QM: %d \n", result);
    }

    if (errorCount)
    {
        System_printf ("test failed with %d errors\n", errorCount);
    }
    else
    {
        System_printf ("*******************************************************\n");
        System_printf ("********* QMSS Accumulator testing Done (PASS) ********\n");
        System_printf ("*******************************************************\n");
    }
} /* accTest */

void run_test (void)
{
    accTest ();
}

