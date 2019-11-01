/**
 *   @file  test_qosSchedDropSched.c
 *
 *   @brief   
 *      This is the QMSS unit test code for QoS scheduler.  This
 *      corresponds to the firmware in qos_sched_be[] and 
 *      qos_sched_le[] and the LLD in qmss_qosSched.c
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2013, Texas Instruments, Inc.
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

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_qosSched.h>


/* CSL RL includes */
#include <ti/csl/csl_chip.h>

/* Test includes */
#include <test_qosSchedScen.h>
#include <test_qosSchedDropSched.h>
#include <stdlib.h>
#include <test_qosSched.h>

/* Collect push stats */
uint32_t numPushStats = 0;
Qmss_QosSchedStats pushStats[QMSS_QOS_SCHED_DROP_SCHED_STATS_BLOCKS];

/* Compare two output profiles, ignoring average depth */
int test_drop_sched_compare_outprof (Qmss_QosSchedDropSchedOutProf *cfg1, Qmss_QosSchedDropSchedOutProf *cfg2)
{
    size_t nonavg_size = offsetof (Qmss_QosSchedDropSchedOutProf, avgQueDepth);
    return memcmp (cfg1, cfg2, nonavg_size);
}

/* Compare two dropcfgs, ignoring 0 seeds */
int test_drop_sched_compare_dropcfg (Qmss_QosSchedDropSchedCfg *cfg1, Qmss_QosSchedDropSchedCfg *cfg2)
{
    size_t nonseed_size = offsetof (Qmss_QosSchedDropSchedCfg, seed1);
    int retVal;
    if ((retVal = memcmp (cfg1, cfg2, nonseed_size)))
    {
        return retVal;
    }

    return 0;
}

void testDropSchedSendConfigs (dropSchedScenCfg *cfg)
{      
    Qmss_Result result;
#ifdef _TMS320C6X
    uint32_t      corenum = CSL_chipReadReg(CSL_CHIP_DNUM); 
#else
    uint32_t      corenum = 0;
#endif
    /* Send the configurations */
    if ( (result = Qmss_putCfgQosSchedDropSchedOutProfs 
                     (0, cfg->outProfs, QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES)) != 
                      QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : failed to set output profiles: %d \n", corenum, result);
        errorCount++;
    }
    if ( (result = Qmss_putCfgQosSchedDropSchedCfgProfs 
                     (0, cfg->cfgProfs, QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES)) !=
                      QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : failed to set config profiles: %d \n", corenum, result);
        errorCount++;
    }
    if ( (result = Qmss_putCfgQosSchedDropSchedQueCfgs 
                     (0, cfg->queProfs, QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS)) !=
                      QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : failed to set queue configs: %d \n", corenum, result);
        errorCount++;
    }
    if ( (result = Qmss_putCfgQosSchedDropSchedCfg (0, cfg->dropCfg)) !=
                      QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : failed to set top level config: %d \n", corenum, result);
        errorCount++;
    }
    /* Enable the drop scheduler */
    if ((result = Qmss_enableQosSchedDropSched (0)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Core %d : failed to enable drop scheduler: %d\n", corenum, result);
        errorCount++;
    }
}

typedef union
{
    Qmss_QosSchedDropSchedOutProf outProfs[QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES];
    Qmss_QosSchedDropSchedCfgProf cfgProfs[QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES];
    Qmss_QosSchedDropSchedQueCfg  queProfs[QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS];
    Qmss_QosSchedDropSchedCfg     dropCfg;
} dropSchedConfigsU;

void testDropSchedRBConfigs (dropSchedScenCfg *expected)
{
    Qmss_Result       result;
#ifdef _TMS320C6X
    uint32_t      corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    uint32_t      corenum = 0;
#endif
    dropSchedConfigsU actual;
    int               i;

    memset (actual.outProfs, 0, sizeof(actual.outProfs));
    if ( (result = Qmss_getCfgQosSchedDropSchedOutProfs 
                     (0, actual.outProfs, sizeof(actual.outProfs)/sizeof(actual.outProfs[0]))) != 
                      QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : failed to get output profiles: %d \n", corenum, result);
        errorCount++;
    }

    if (test_drop_sched_compare_outprof (actual.outProfs, expected->outProfs) )
    {
        System_printf ("Error Core %d : failed to compare output profiles\n", corenum);
        errorCount++;
    }

    memset (actual.cfgProfs, 0, sizeof(actual.cfgProfs));
    if ( (result = Qmss_getCfgQosSchedDropSchedCfgProfs 
                     (0, actual.cfgProfs, sizeof(actual.cfgProfs)/sizeof(actual.cfgProfs[0]))) !=
                      QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : failed to get config profiles: %d \n", corenum, result);
        errorCount++;
    }

    for (i = 0; i < sizeof(actual.cfgProfs)/sizeof(actual.cfgProfs[0]); i++)
    {
        /* Fix "INHERITED" vs "PACKETS" */
        if ((actual.cfgProfs[i].tailDropType == Qmss_QosSchedAcctType_PACKETS) &&
            (expected->cfgProfs[i].tailDropType == Qmss_QosSchedAcctType_INHERITED))
        {
            actual.cfgProfs[i].tailDropType = Qmss_QosSchedAcctType_INHERITED;
        }
    }

    if (memcmp (actual.cfgProfs, expected->cfgProfs, sizeof(actual.cfgProfs)) )
    {
        System_printf ("Error Core %d : failed to compare config profiles\n", corenum);
        errorCount++;
    }

    memset (actual.queProfs, 0, sizeof(actual.queProfs));
    if ( (result = Qmss_getCfgQosSchedDropSchedQueCfgs 
                     (0, actual.queProfs, sizeof(actual.queProfs)/sizeof(actual.queProfs[0]))) !=
                      QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : failed to get queue configs: %d \n", corenum, result);
        errorCount++;
    }
    if (memcmp (actual.queProfs, expected->queProfs, sizeof(actual.queProfs)) )
    {
        System_printf ("Error Core %d : failed to compare queue configs\n", corenum);
        errorCount++;
    }

    memset (&actual.dropCfg, 0, sizeof(actual.dropCfg));
    if ( (result = Qmss_getCfgQosSchedDropSchedCfg (0, &actual.dropCfg)) !=
                      QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : failed to get top level config: %d \n", corenum, result);
        errorCount++;
    }
    if (test_drop_sched_compare_dropcfg (&actual.dropCfg, expected->dropCfg) )
    {
        System_printf ("Error Core %d : failed to compare top level config\n", corenum);
        errorCount++;
    }
}

void testDropSched (testState_t *state)
{
    Qmss_Result          result;
#ifdef _TMS320C6X
    uint32_t      corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    uint32_t      corenum = 0;
#endif
    Qmss_QueueHnd        baseQueue;
    int                  queueNum;

    System_printf ("Core %d : testing drop scheduler\n", corenum);

    /* Allocate block of queues for drop scheduler */
    baseQueue = Qmss_queueBlockOpen (state->dropFwQHnds, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QOS_TX_DROP_QUEUES, 32);
    if (baseQueue < 0) 
    {
        System_printf ("Core %d : Failed to open %d contiguous queues\n", corenum, QOS_TX_DROP_QUEUES);
        errorCount++;
    }

    /* Set the FW's base queue */
    if ((result = Qmss_setQosSchedDropSchedQueueBase (baseQueue)) != QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Setting QoS queue base address error code: %d \n", corenum, result);
        errorCount++;
    }

    /* Configure the queue thresholds as required by the FW */
    for (queueNum = 0; queueNum < QOS_TX_DROP_QUEUES; queueNum++) 
    {
        Qmss_setQueueThreshold (state->dropFwQHnds[queueNum], 1, 1);
    }
}


void testQosSched_getPushStats (testState_t *state, int blockNum, 
                                Qmss_QosSchedStats *stats, uint32_t which_reset)
{
    Qmss_QosSchedDropSchedStats curStats;
    void                       *desc;
    Qmss_Result                 retVal;
#ifdef _TMS320C6X
    uint32_t      corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
#else
    uint32_t      corenum = 0;
#endif

    /* Drain the push stats */
    while (desc = Qmss_queuePop (state->statsQs[1]))
    {
        Qmss_QosSchedDropSchedPushStats thisPushStat;
        int pushBlockNum;
        retVal = Qmss_convertQosSchedDropSchedPushStats (&thisPushStat, desc);
        Qmss_queuePushDesc (state->statsQs[0], desc);
 
        if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS)
        {
            System_printf ("Error core %d: Push stat conversion returned error: %d\n", corenum, retVal);
            errorCount++;
        }
        else
        {
            pushBlockNum = thisPushStat.statBlockIdx;
            pushStats[pushBlockNum].bytesForwarded += thisPushStat.block.bytesForwarded;
            pushStats[pushBlockNum].bytesDiscarded += thisPushStat.block.bytesDiscarded;
            pushStats[pushBlockNum].packetsForwarded += thisPushStat.block.packetsForwarded;
            pushStats[pushBlockNum].packetsDiscarded += thisPushStat.block.packetsDiscarded;
            numPushStats++;
        }
    }

    /* Query the non-push stats */
    Qmss_getQosSchedDropSchedStats (&curStats, 0, blockNum, which_reset);

    /* Add and return the push stats */
    stats->bytesForwarded = curStats.bytesForwarded + pushStats[blockNum].bytesForwarded;
    stats->bytesDiscarded = curStats.bytesDiscarded + pushStats[blockNum].bytesDiscarded;
    stats->packetsForwarded = curStats.packetsForwarded + pushStats[blockNum].packetsForwarded;
    stats->packetsDiscarded = curStats.packetsDiscarded + pushStats[blockNum].packetsDiscarded;

    /* clear push stats */
    if (which_reset & QMSS_QOS_SCHED_STATS_FORWARDED_BYTES)
    {
        pushStats[blockNum].bytesForwarded = 0;
    }
    if (which_reset & QMSS_QOS_SCHED_STATS_FORWARDED_PACKETS)
    {
        pushStats[blockNum].packetsForwarded = 0;
    }
    if (which_reset & QMSS_QOS_SCHED_STATS_DISCARDED_BYTES)
    {
        pushStats[blockNum].bytesDiscarded = 0;
    }
    if (which_reset & QMSS_QOS_SCHED_STATS_DISCARDED_PACKETS)
    {
        pushStats[blockNum].packetsDiscarded = 0;
    }
}

uint32_t testGetNumPushStats (void)
{
    uint32_t retVal = numPushStats;

    numPushStats = 0;
    return retVal;
}


