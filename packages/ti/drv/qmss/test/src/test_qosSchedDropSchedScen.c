/**
 *   @file  test_qosSchedScen.c
 *
 *   @brief   
 *      This file contains system scenarios for testing the QoS scheduler.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2014, Texas Instruments, Inc.
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

#include "test_qosSchedScen.h"
extern UInt32 errorCount;

/*****************************************************************************
 * Base QoS configuration used by scenarios 1,2,4
 *****************************************************************************/
int qos_sched_base_config_1 (int32_t *nPorts, Qmss_QosSchedPortCfg *portCfg, 
                             uint32_t ticks_second)
{
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t ports = 5;
    uint32_t port, queue;

    /*************************************************************************
     * Configure 4 ports facing the drop scheduler each which a fixed 
     * CIR of 50Mbps.
     *
     * Configure 1 port combining the 4 ports with fixed CIR of 100.
     *************************************************************************/

    /* Make sure enough memory is in portCfg */
    if (*nPorts < ports) 
    {
        System_printf ("Build supports too few ports (%d, %d)\n", *nPorts, ports);
        return 1;
    }
    *nPorts = ports;
    memset (portCfg, 0, ports * sizeof(Qmss_QosSchedPortCfg));

    for (port = 0; port < ports; port++)
    {
        portCfg[port].wrrType = bytes;
        portCfg[port].cirType = bytes;
        portCfg[port].outThrotType = bytes;
        portCfg[port].congestionType = Qmss_QosSchedAcctType_PACKETS;
        scenario_rate_conversion_wrapper 
            (&portCfg[port].cirIteration, ticks_second, 50000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].cirMax, 1, 7000000, bytes);
        portCfg[port].overheadBytes = 0;
        portCfg[port].groupCount = 1;
        portCfg[port].outThrotThresh = (100000000/8/ticks_second);
        portCfg[port].group[0].totQueueCount = 2;
        portCfg[port].group[0].spQueueCount = 2;
        portCfg[port].group[0].wrrQueueCount = 0;
    }

    /* port 4 is a real output port so no threshold needed */
    portCfg[4].outThrotThresh = 0;
    scenario_rate_conversion_wrapper 
        (&portCfg[4].cirIteration, ticks_second, 100000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[4].cirMax, 1, 7000000, bytes);
    portCfg[4].group[0].totQueueCount = 4;
    portCfg[4].group[0].spQueueCount = 0;
    portCfg[4].group[0].wrrQueueCount = 4;
    portCfg[4].outThrotThresh = 0;
    for (queue = 0; queue < 4; queue++)
    {    
        scenario_wrr_conversion_wrapper 
            (&portCfg[4].group[0].Queue[queue].wrrInitialCredit,
             (1000 * (queue + 1)), bytes);
    }

    return 0;
}

/*****************************************************************************
 * Base QoS configuration used by scenarios 1,2,4
 *****************************************************************************/
int qos_base_config_1 (int32_t *nPorts, dropSchedScenCfg *cfg, 
                       uint32_t ticks_second, uint32_t nGroups)
{
    Qmss_QosSchedDropSchedOutProf *outProfs        = cfg->outProfs;
    Qmss_QosSchedDropSchedCfgProf *cfgProfs        = cfg->cfgProfs;
    Qmss_QosSchedDropSchedQueCfg  *queProfs        = cfg->queProfs;
    Qmss_QosSchedDropSchedCfg     *dropCfg         = cfg->dropCfg;
    Qmss_QosSchedPortCfg          *portCfg         = cfg->qosSchedCfgs;
    uint16_t                      *outQIdx         = cfg->outQIdx;
    uint16_t                      *qosSchedOutQIdx = cfg->qosSchedOutQIdx;

    /* don't use portmap */
    cfg->qosSchedPortMap = NULL;

    qos_sched_base_config_1 (nPorts, portCfg, ticks_second);

    /* Connect ports 0-3 to port 4, and port 4 to output */
    qosSchedOutQIdx[0] = 16;
    qosSchedOutQIdx[1] = 17;
    qosSchedOutQIdx[2] = 18;
    qosSchedOutQIdx[3] = 19;
    qosSchedOutQIdx[4] = 0xffff;

    /* Set up 8 input queues in irregular fashion */
    memset (queProfs, 0, sizeof (*queProfs) * QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS);
    queProfs[1].valid        = Qmss_QosSchedDropSchedProf_VALID;
    queProfs[1].statBlockIdx = 7;
    queProfs[1].outProfIdx   = 15;
    queProfs[3].valid        = Qmss_QosSchedDropSchedProf_VALID;
    queProfs[3].statBlockIdx = 6;
    queProfs[3].outProfIdx   = 14;
    queProfs[5].valid        = Qmss_QosSchedDropSchedProf_VALID;
    queProfs[5].statBlockIdx = 5;
    queProfs[5].outProfIdx   = 13;
    queProfs[7].valid        = Qmss_QosSchedDropSchedProf_VALID;
    queProfs[7].statBlockIdx = 4;
    queProfs[7].outProfIdx   = 12;
    queProfs[9].valid        = Qmss_QosSchedDropSchedProf_VALID;
    queProfs[9].statBlockIdx = 3;
    queProfs[9].outProfIdx   = 11;
    queProfs[11].valid        = Qmss_QosSchedDropSchedProf_VALID;
    queProfs[11].statBlockIdx = 2;
    queProfs[11].outProfIdx   = 10;
    queProfs[13].valid        = Qmss_QosSchedDropSchedProf_VALID;
    queProfs[13].statBlockIdx = 1;
    queProfs[13].outProfIdx   = 9;
    queProfs[15].valid        = Qmss_QosSchedDropSchedProf_VALID;
    queProfs[15].statBlockIdx = 0;
    queProfs[15].outProfIdx   = 8;

    /* Set up 8 output profiles linking to 4 qos ports */
    memset (outProfs, 0, sizeof(*outProfs) * QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES);
    memset (outQIdx, -1, sizeof(uint16_t) * QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES);
    outProfs[8].valid          = Qmss_QosSchedDropSchedProf_VALID;
    outProfs[8].REDDropProb    = 0;
    outQIdx[8]                 = 0;
    outProfs[8].cfgProfIdx     = 0;
    outProfs[9].valid          = Qmss_QosSchedDropSchedProf_VALID;
    outProfs[9].REDDropProb    = 0;
    outQIdx[9]                 = 1;
    outProfs[9].cfgProfIdx     = 0;
    outProfs[10].valid         = Qmss_QosSchedDropSchedProf_VALID;
    outProfs[10].REDDropProb   = 0;
    outQIdx[10]                = 4;
    outProfs[10].cfgProfIdx    = 1;
    outProfs[11].valid         = Qmss_QosSchedDropSchedProf_VALID;
    outProfs[11].REDDropProb   = 0;
    outQIdx[11]                = 5;
    outProfs[11].cfgProfIdx    = 1;
    outProfs[12].valid         = Qmss_QosSchedDropSchedProf_VALID;
    outProfs[12].REDDropProb   = (uint16_t)(65536 * 0.01);
    outQIdx[12]                = 8;
    outProfs[12].cfgProfIdx    = 2;
    outProfs[13].valid         = Qmss_QosSchedDropSchedProf_VALID;
    outProfs[13].REDDropProb   = (uint16_t)(65536 * 0.015);
    outQIdx[13]                = 9;
    outProfs[13].cfgProfIdx    = 2;
    outProfs[14].valid         = Qmss_QosSchedDropSchedProf_VALID;
    outProfs[14].REDDropProb   = (uint16_t)(65536 * 0.02);
    outQIdx[14]                = 12;
    outProfs[14].cfgProfIdx    = 3;
    outProfs[15].valid         = Qmss_QosSchedDropSchedProf_VALID;
    outProfs[15].REDDropProb   = (uint16_t)(65536 * 0.025);
    outQIdx[15]                = 13;
    outProfs[15].cfgProfIdx    = 3;

    /* Set up 4 config profiles */
    memset (cfgProfs, 0, sizeof(*cfgProfs) * QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES);
    /* tail drop with bytes */
    cfgProfs[0].mode           = Qmss_QosSchedDropSchedMode_TAILDROP;
    cfgProfs[0].tailDropType   = Qmss_QosSchedAcctType_BYTES;
    cfgProfs[0].tailDropThresh = 20000;

    /* tail drop with packets */
    cfgProfs[1].mode           = Qmss_QosSchedDropSchedMode_TAILDROP;
    cfgProfs[1].tailDropType   = Qmss_QosSchedAcctType_PACKETS;
    cfgProfs[1].tailDropThresh = 100;

    /* red drop alone */
    cfgProfs[2].mode           = Qmss_QosSchedDropSchedMode_RED;
    cfgProfs[2].tailDropType   = Qmss_QosSchedAcctType_BYTES;
    cfgProfs[2].tailDropThresh = 0;
    cfgProfs[2].REDLowThresh   = 5000 << 8;
    cfgProfs[2].REDHighThresh  = 10000 << 8;
    cfgProfs[2].timeConstantP2 = 8;

    /* red drop with tail drop */
    cfgProfs[3].mode           = Qmss_QosSchedDropSchedMode_RED;
    cfgProfs[3].tailDropType   = Qmss_QosSchedAcctType_BYTES;
    cfgProfs[3].tailDropThresh = 20000;
    cfgProfs[3].REDLowThresh   = 5000 << 8;
    cfgProfs[3].REDHighThresh  = 10000 << 8;
    cfgProfs[3].timeConstantP2 = 8;

    /* Set up top level config */
    memset (dropCfg, 0, sizeof(*dropCfg));

    dropCfg->seed1           = 0xf00;
    dropCfg->seed2           = 0;
    dropCfg->seed3           = 0;

    /* Expect no push stats */
    *cfg->expNumPushStats = 0;
    return 0;
}

int qos_base_gen_config_1 (packetGenCfg *pktGen, int pktGens)
{
    int gen;
    /*************************************************************************
     * Configure the packet generator as follows
     * Total (3) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (3) with (20M)bps and (1000)bytes/packet
     *
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds.
     * 
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *    Stream No.(1) output with (?)pps or (5M)bps
     *Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 1000;
    pktGen->readTime = 1100;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (gen = 0; gen < pktGens; gen++)
    {
        pktGen->Qs[gen].port       = GENPORT_DROPSCHED; // drop sched port
        pktGen->Qs[gen].group      = 0;
        pktGen->Qs[gen].queue      = 15;
        pktGen->Qs[gen].nSizes     = 1;
        pktGen->Qs[gen].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[gen].pktSize[0] = 100;
        pktGen->Qs[gen].pps        = 200000000 / (pktGen->Qs[gen].pktSize[0] * 8);
        pktGen->Qs[gen].ppsExp     = 50000000 / (pktGen->Qs[gen].pktSize[0] * 8);
        pktGen->Qs[gen].ppsError   = pktGen->Qs[gen].ppsExp / (100); /* 1.0% */
    }

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 1
 * tail bytes
 *****************************************************************************/
int scenario_config_01 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    return qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 1);
}

/*****************************************************************************
 * Configuration and expected results for scenario 2
 * tail packets
 *****************************************************************************/
int scenario_config_02 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal;
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    retVal = qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 1);

    if (retVal)
    {
        return retVal;
    }

    pktGen->Qs[0].queue      = 9;

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 3
 * red only
 *****************************************************************************/
int scenario_config_03 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal;
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    retVal = qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 1);

    if (retVal)
    {
        return retVal;
    }

    pktGen->Qs[0].queue      = 5;

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 4
 * red only but use push proxy 
 *****************************************************************************/
int scenario_config_04 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal = scenario_config_03 (timer, cfg, pktGen, nPorts);

    if (retVal)
    {
        return retVal;
    }

    cfg->fUsePushProxy = 1;
    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 5
 * red + tail drop bytes
 *****************************************************************************/
int scenario_config_05 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal;
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    retVal = qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 1);

    if (retVal)
    {
        return retVal;
    }

    pktGen->Qs[0].queue      = 1;

    return 0;
}

/*****************************************************************************
 * Test WRR port alone
 *****************************************************************************/
int scenario_config_06 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal;
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    retVal = qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 4);

    if (retVal)
    {
        return retVal;
    }

    pktGen->Qs[0].queue      = 0;
    pktGen->Qs[0].port       = 4;
    pktGen->Qs[1].queue      = 1;
    pktGen->Qs[1].port       = 4;
    pktGen->Qs[2].queue      = 2;
    pktGen->Qs[2].port       = 4;
    pktGen->Qs[3].queue      = 3;
    pktGen->Qs[3].port       = 4;
    cfg->qosSchedCfgs[4].group[0].Queue[0].congestionThresh = 50;
    cfg->qosSchedCfgs[4].group[0].Queue[1].congestionThresh = 50;
    cfg->qosSchedCfgs[4].group[0].Queue[2].congestionThresh = 50;
    cfg->qosSchedCfgs[4].group[0].Queue[3].congestionThresh = 50;
    /* 4:3:2:1 */
    pktGen->Qs[0].ppsExp     = (100000000 * 1 / 10) / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp     = (100000000 * 2 / 10) / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp     = (100000000 * 3 / 10) / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[3].ppsExp     = (100000000 * 4 / 10) / (pktGen->Qs[3].pktSize[0] * 8);

    return 0;
}
/*****************************************************************************
 * Test WRR port alone with push proxy
 *****************************************************************************/
int scenario_config_07 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal = scenario_config_06 (timer, cfg, pktGen, nPorts);

    if (retVal)
    {
        return retVal;
    }

    cfg->fUsePushProxy = 1;
    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 8
 * all 4 scenarios at once
 *****************************************************************************/
int scenario_config_08 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal;
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    retVal = qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 4);

    if (retVal)
    {
        return retVal;
    }

    pktGen->Qs[0].queue      = 1;
    pktGen->Qs[1].queue      = 5;
    pktGen->Qs[2].queue      = 9;
    pktGen->Qs[3].queue      = 15;

    /* 4:3:2:1 */
    pktGen->Qs[0].ppsExp     = (100000000 * 4 / 10) / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp     = (100000000 * 3 / 10) / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp     = (100000000 * 2 / 10) / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[3].ppsExp     = (100000000 * 1 / 10) / (pktGen->Qs[3].pktSize[0] * 8);

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 9
 * all 4 scenarios at once using push proxy
 *****************************************************************************/
int scenario_config_09 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal = scenario_config_08 (timer, cfg, pktGen, nPorts);

    if (retVal)
    {
        return retVal;
    }

    cfg->fUsePushProxy = 1;
    return 0;
}

/*****************************************************************************
 * Test joint port plus endcaps
 *****************************************************************************/
int scenario_config_joint (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts, int nJointWrr)
{
    Qmss_QosSchedDropSchedOutProf *outProfs        = cfg->outProfs;
    Qmss_QosSchedDropSchedCfgProf *cfgProfs        = cfg->cfgProfs;
    Qmss_QosSchedDropSchedQueCfg  *queProfs        = cfg->queProfs;
    Qmss_QosSchedDropSchedCfg     *dropCfg         = cfg->dropCfg;
    Qmss_QosSchedPortCfg          *portCfg         = cfg->qosSchedCfgs;
    int                           *portMapCfg      = cfg->qosSchedPortMap;
    uint16_t                      *outQIdx         = cfg->outQIdx;
    uint16_t                      *qosSchedOutQIdx = cfg->qosSchedOutQIdx;

    /* No drop sched needed */
    memset (queProfs, 0, sizeof (*queProfs) * QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS);
    memset (outProfs, 0, sizeof(*outProfs) * QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES);
    memset (outQIdx, -1, sizeof(uint16_t) * QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES);
    memset (cfgProfs, 0, sizeof(*cfgProfs) * QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES);
    memset (dropCfg, 0, sizeof(*dropCfg));

    /* Expect no push stats */
    *cfg->expNumPushStats = 0;

    /* map ports directly to output queue without hierarchy */
    qosSchedOutQIdx[0] = qosSchedOutQIdx[1] = qosSchedOutQIdx[2] = 0xffff;

    return scenario_config_common_joint (timer, portCfg, portMapCfg, pktGen,
                                         nPorts, nJointWrr);
}

/*****************************************************************************
 * Test 8WRR joint port plus endcaps
 *****************************************************************************/
int scenario_config_10 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    return scenario_config_joint (timer, cfg, pktGen, nPorts, 8);
}

/*****************************************************************************
 * Test 5WRR joint port plus endcaps
 *****************************************************************************/
int scenario_config_11 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    return scenario_config_joint (timer, cfg, pktGen, nPorts, 5);
}

/*****************************************************************************
 * Test 4WRR joint port plus endcaps
 *****************************************************************************/
int scenario_config_12 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    return scenario_config_joint (timer, cfg, pktGen, nPorts, 4);
}

/*****************************************************************************
 * Test WRR port alone with 1000:100:10:1 ratio
 *****************************************************************************/
int scenario_config_13 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal, idx;
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    retVal = qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 4);

    if (retVal)
    {
        return retVal;
    }

    pktGen->Qs[0].queue      = 0;
    pktGen->Qs[0].port       = 4;
    pktGen->Qs[1].queue      = 1;
    pktGen->Qs[1].port       = 4;
    pktGen->Qs[2].queue      = 2;
    pktGen->Qs[2].port       = 4;
    pktGen->Qs[3].queue      = 3;
    pktGen->Qs[3].port       = 4;
    cfg->qosSchedCfgs[4].group[0].Queue[0].congestionThresh = 30;
    cfg->qosSchedCfgs[4].group[0].Queue[0].wrrInitialCredit = 102400000;
    cfg->qosSchedCfgs[4].group[0].Queue[1].congestionThresh = 30;
    cfg->qosSchedCfgs[4].group[0].Queue[1].wrrInitialCredit = 10240000;
    cfg->qosSchedCfgs[4].group[0].Queue[2].congestionThresh = 30;
    cfg->qosSchedCfgs[4].group[0].Queue[2].wrrInitialCredit = 1024000;
    cfg->qosSchedCfgs[4].group[0].Queue[3].congestionThresh = 30;
    cfg->qosSchedCfgs[4].group[0].Queue[3].wrrInitialCredit = 102400;

    /* 1000:100:10:1 */
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].pps        = 110000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[0].ppsExp     = (uint32_t)((100000000ull * 1000 / 1111) / (pktGen->Qs[0].pktSize[0] * 8));
    pktGen->Qs[1].pktSize[0] = 1000;
    pktGen->Qs[1].pps        = 11000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp     = (uint32_t)((100000000ull * 100  / 1111) / (pktGen->Qs[1].pktSize[0] * 8));
    pktGen->Qs[2].pktSize[0] = 1000;
    pktGen->Qs[2].pps        = 1100000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp     = (uint32_t)((100000000ull * 10   / 1111) / (pktGen->Qs[2].pktSize[0] * 8));
    pktGen->Qs[3].pktSize[0] = 1000;
    pktGen->Qs[3].pps        = 110000 / (pktGen->Qs[3].pktSize[0] * 8);
    pktGen->Qs[3].ppsExp     = (uint32_t)((100000000ull * 1    / 1111) / (pktGen->Qs[3].pktSize[0] * 8));
    for (idx = 0; idx < 4; idx++)
    {
       pktGen->Qs[idx].ppsError = pktGen->Qs[idx].ppsExp / (20); /* 5.0% */
    }
    pktGen->Qs[3].ppsError++;  /* extra +1 because +/- 1 in result is 10% error */
    pktGen->genTime  = 20000;
    pktGen->readTime = 21000;

    return 0;
}

/*****************************************************************************
 * Test WRR port alone with 10000:100:10:1 ratio
 *****************************************************************************/
int scenario_config_14 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal, idx;
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    retVal = qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 4);

    if (retVal)
    {
        return retVal;
    }

    pktGen->Qs[0].queue      = 0;
    pktGen->Qs[0].port       = 4;
    pktGen->Qs[1].queue      = 1;
    pktGen->Qs[1].port       = 4;
    pktGen->Qs[2].queue      = 2;
    pktGen->Qs[2].port       = 4;
    pktGen->Qs[3].queue      = 3;
    pktGen->Qs[3].port       = 4;
    cfg->qosSchedCfgs[4].group[0].Queue[0].congestionThresh = 30;
    cfg->qosSchedCfgs[4].group[0].Queue[0].wrrInitialCredit = 1024000000;
    cfg->qosSchedCfgs[4].group[0].Queue[1].congestionThresh = 30;
    cfg->qosSchedCfgs[4].group[0].Queue[1].wrrInitialCredit = 10240000;
    cfg->qosSchedCfgs[4].group[0].Queue[2].congestionThresh = 30;
    cfg->qosSchedCfgs[4].group[0].Queue[2].wrrInitialCredit = 1024000;
    cfg->qosSchedCfgs[4].group[0].Queue[3].congestionThresh = 30;
    cfg->qosSchedCfgs[4].group[0].Queue[3].wrrInitialCredit = 102400;

    /* 10000:100:10:1 */
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].pps        = 1100000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[0].ppsExp     = (uint32_t)((100000000ull * 10000 / 10111) / (pktGen->Qs[0].pktSize[0] * 8));
    pktGen->Qs[1].pktSize[0] = 1000;
    pktGen->Qs[1].pps        = 11000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp     = (uint32_t)((100000000ull * 100  / 10111) / (pktGen->Qs[1].pktSize[0] * 8));
    pktGen->Qs[2].pktSize[0] = 1000;
    pktGen->Qs[2].pps        = 1100000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp     = (uint32_t)((100000000ull * 10   / 10111) / (pktGen->Qs[2].pktSize[0] * 8));
    pktGen->Qs[3].pktSize[0] = 1000;
    pktGen->Qs[3].pps        = 110000 / (pktGen->Qs[3].pktSize[0] * 8);
    pktGen->Qs[3].ppsExp     = (uint32_t)((100000000ull * 1    / 10111) / (pktGen->Qs[3].pktSize[0] * 8));
    for (idx = 0; idx < 4; idx++)
    {
       pktGen->Qs[idx].ppsError = pktGen->Qs[idx].ppsExp / (20); /* 5.0% */
    }
    pktGen->Qs[2].ppsError++;  /* extra +1 because +/- 1 in result is 10% error */
    pktGen->Qs[3].ppsError++;  /* extra +1 because +/- 1 in result is 100% error */
    pktGen->genTime  = 20000;
    pktGen->readTime = 21000;

    return 0;
}

/*****************************************************************************
 * Run long enough to fire two push stat
 *****************************************************************************/
int scenario_config_15 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    Qmss_QosSchedDropSchedQueCfg  *queProfs = cfg->queProfs;
    int retVal = scenario_config_08 (timer, cfg, pktGen, nPorts);
    int prof;

    if (retVal)
    {
        return retVal;
    }

    for (prof = 0; prof < QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS; prof++)
    {
        if (queProfs[prof].valid)
        {
            queProfs[prof].pushStatsIdx = 2;
        }
    }

    pktGen->genTime  = 100000;
    pktGen->readTime = 100100;

    /* Expect 2 push stats blocks */
    *cfg->expNumPushStats = 2;

    return 0;
}

/*****************************************************************************
 * Run all ports and queues
 *****************************************************************************/
int scenario_config_16 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    Qmss_QosSchedDropSchedOutProf *outProfs        = cfg->outProfs;
    Qmss_QosSchedDropSchedCfgProf *cfgProfs        = cfg->cfgProfs;
    Qmss_QosSchedDropSchedQueCfg  *queProfs        = cfg->queProfs;
    Qmss_QosSchedDropSchedCfg     *dropCfg         = cfg->dropCfg;
    Qmss_QosSchedPortCfg          *portCfg         = cfg->qosSchedCfgs;
    uint16_t                      *outQIdx         = cfg->outQIdx;
    uint16_t                      *qosSchedOutQIdx = cfg->qosSchedOutQIdx;

    uint32_t                       ticks_second = 1000000 / 50;
    int                            idx;
    Qmss_QosSchedAcctType          bytes = Qmss_QosSchedAcctType_BYTES;

    /* don't use portmap */
    cfg->qosSchedPortMap = NULL;

    memset (outProfs, 0, sizeof(*outProfs) * QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES);
    for (idx = 0; idx <  QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES; idx++)
    {
        outProfs[idx].cfgProfIdx    = idx % QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES;
        outProfs[idx].valid         = Qmss_QosSchedDropSchedProf_VALID;
        outProfs[idx].REDDropProb   = (uint16_t)(65536 * 0.025);
        outQIdx[idx]                = idx;
    }
    memset (cfgProfs, 0, sizeof(*cfgProfs) * QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES);
    for (idx = 0; idx < QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES; idx++)
    {
        cfgProfs[idx].mode           = Qmss_QosSchedDropSchedMode_RED;
        cfgProfs[idx].tailDropType   = Qmss_QosSchedAcctType_BYTES;
        cfgProfs[idx].tailDropThresh = 0;
        cfgProfs[idx].REDLowThresh   = 5000 << 8;
        cfgProfs[idx].REDHighThresh  = 10000 << 8;
        cfgProfs[idx].timeConstantP2 = 8;
    }
    memset (queProfs, 0, sizeof(*queProfs) * QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS);
    for (idx = 0; idx < QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS; idx++)
    {
        queProfs[idx].valid        = Qmss_QosSchedDropSchedProf_VALID;
        queProfs[idx].statBlockIdx = idx % QMSS_QOS_SCHED_DROP_SCHED_STATS_BLOCKS;
        queProfs[idx].outProfIdx   = idx % QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES;
        queProfs[idx].pushStatsIdx = 0;
    }

    dropCfg->seed1           = 0xf00;
    dropCfg->seed2           = 0;
    dropCfg->seed3           = 0;
    memset (portCfg, 0, sizeof(*portCfg) * QMSS_QOS_SCHED_DROP_SCHED_LITE_MAX_PHYS_PORTS);
    for (idx = 0; idx < QMSS_QOS_SCHED_DROP_SCHED_LITE_MAX_PHYS_PORTS; idx++)
    {
        int queue;

        portCfg[idx].wrrType = bytes;
        portCfg[idx].cirType = bytes;
        portCfg[idx].outThrotType = bytes;
        portCfg[idx].congestionType = Qmss_QosSchedAcctType_PACKETS;
        scenario_rate_conversion_wrapper 
            (&portCfg[idx].cirIteration, ticks_second, 50000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[idx].cirMax, 1, 7000000, bytes);
        portCfg[idx].overheadBytes = 0;
        portCfg[idx].groupCount = 1;
        portCfg[idx].outThrotThresh = 0;
        portCfg[idx].group[0].totQueueCount = 4;
        portCfg[idx].group[0].spQueueCount = 0;
        portCfg[idx].group[0].wrrQueueCount = 4;
        for (queue = 0; queue < 4; queue++)
        {    
            scenario_wrr_conversion_wrapper 
                (&portCfg[idx].group[0].Queue[queue].wrrInitialCredit,
                 1000 , bytes);
        }
        /* No hierachy - just send packets to output */
        qosSchedOutQIdx[idx] = 0xffff;
    }

    /* Make sure enough memory is in pktGen */
    if (pktGen->numQCfgs < QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS);
        return 1;
    }
    
    pktGen->numQCfgs = QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS;
    pktGen->genTime  = 1000;
    pktGen->readTime = 1100;
    memset (pktGen->Qs, 0, QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS* sizeof(pktGen->Qs[0]));

    for (idx = 0; idx < QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS; idx++)
    {
        pktGen->Qs[idx].port       = GENPORT_DROPSCHED; // drop sched port
        pktGen->Qs[idx].group      = 0;
        pktGen->Qs[idx].queue      = idx;
        pktGen->Qs[idx].nSizes     = 1;
        pktGen->Qs[idx].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[idx].pktSize[0] = 100;
        pktGen->Qs[idx].pps        = 3125; // * 80 = 250K pps
        pktGen->Qs[idx].ppsExp     = 3125;
        pktGen->Qs[idx].ppsError   = pktGen->Qs[idx].ppsExp / (100); /* 1.0% */
    }

    /* Expect no push stats blocks */
    *cfg->expNumPushStats = 0;

    return 0;
}

/*****************************************************************************
 * Show that drop disabled works
 *****************************************************************************/
int scenario_config_17 (uint32_t *timer, dropSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    int retVal;
    uint32_t ticks_second = 1000000 / 50;

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    retVal = qos_base_config_1 (nPorts, cfg, ticks_second, 1) || qos_base_gen_config_1 (pktGen, 2);

    if (retVal)
    {
        return retVal;
    }

    pktGen->Qs[1].queue  = 0; /* feed packets on disabled queue */
    pktGen->Qs[1].ppsExp = 0; /* none should come back, and we shouldn't leak descs */
    pktGen->Qs[1].ppsError = 0; /* no error */

    /* Expect no push stats blocks */
    *cfg->expNumPushStats = 0;

    return 0;
}



const qosSchedTestScenCfg qosSchedTestScenCfgs[] =
{
#if (! defined(SIMULATOR_SUPPORT)) || defined(SIMULATOR_SUPPORT_FULL)
    scenario_config_01, "check drop scheduler with tail drop (bytes)",
    scenario_config_02, "check drop scheduler with tail drop (packets)",
    scenario_config_03, "check drop scheduler with RED only",
    scenario_config_04, "check drop scheduler with RED only and push proxy",
    scenario_config_05, "check drop scheduler with RED + tail (bytes)",
    scenario_config_06, "wrr inputs only",
    scenario_config_07, "wrr inputs only with push proxy",
    scenario_config_08, "all 4 scenarios simutaneously",
#endif
    scenario_config_09, "all 4 scenarios simutaneously with push proxy",
    scenario_config_10, "8wrr joint port plus preceding and following port",
    scenario_config_11, "5wrr joint port plus preceding and following port",
    scenario_config_12, "4wrr joint port plus preceding and following port",
    scenario_config_13, "1000:100:10:1 wrr port",
    scenario_config_14, "10000:100:10:1 wrr port",
#if (! defined(SIMULATOR_SUPPORT)) || defined(SIMULATOR_SUPPORT_FULL)
    scenario_config_15, "run scen 7 long enough (2 minutes) to test push stat",
    scenario_config_16, "enable all ports and profiles",
    scenario_config_17, "drop disable test",
#endif
    NULL, NULL
};


