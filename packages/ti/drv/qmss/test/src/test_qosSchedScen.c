/**
 *   @file  test_qosSchedScen.c
 *
 *   @brief   
 *      This file contains system scenarios for testing the QoS scheduler.
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
int qos_base_config_1 (int32_t *nPorts, Qmss_QosSchedPortCfg *portCfg, 
                        uint32_t ticks_second, uint32_t nGroups)
{
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t ports = 1;
    uint32_t port, group;

    /*************************************************************************
     * Configure the 1 port with nGroups group as follows:
     *
     * Total configured (1) Port
     * Port No.(0) configuration:
     *     Port CIR (50Mbps) CIR_MAX/CBS (7M)
     *     Total configured (1) Group
     *     Group CIR (0M) CIR_MAX/CBS (0M)
     *     Group PIR (30Mbps) PIR_MAX/PBS (7M)
     *     Group No.(n) configuration: wrrInitialCredit (0)
     *           Total configured (8) Queues: (3) Priority queues (5) WRR queues
     *            Priority queue No.(0) congestionThresh (1000)
     *            Priority queue No.(1) congestionThresh (1000)
     *            Priority queue No.(2) congestionThresh (1000)
     *            WRR queue No.(3) wrrInitialCredit (8) 
     *            WRR queue No.(3) congestionThresh (1000)
     *            WRR queue No.(4) wrrInitialCredit (16) 
     *            WRR queue No.(4) congestionThresh (1000)
     *            WRR queue No.(5) wrrInitialCredit (24) 
     *            WRR queue No.(5) congestionThresh (1000)
     *            WRR queue No.(6) wrrInitialCredit (0) 
     *            WRR queue No.(6) congestionThresh (1000)
     *            WRR queue No.(7) wrrInitialCredit (0) 
     *            WRR queue No.(7) congestionThresh (1000)
     *************************************************************************/

    /* Make sure enough memory is in portCfg */
    if (*nPorts < 1) 
    {
        System_printf ("Build supports too few ports (%d, %d)\n", *nPorts, ports);
        return 1;
    }
    *nPorts = ports;
    memset (portCfg, 0, ports * sizeof(Qmss_QosSchedPortCfg));

    port = 0;
    portCfg[port].wrrType = bytes;
    portCfg[port].cirType = bytes;
    portCfg[port].congestionType = Qmss_QosSchedAcctType_PACKETS;
    portCfg[port].outThrotType = Qmss_QosSchedAcctType_PACKETS;
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirIteration, ticks_second, 50000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirMax, 1, 7000000, bytes);
    portCfg[port].overheadBytes = 0;
    portCfg[port].groupCount = nGroups;
    for (group = 0; group < nGroups; group++) 
    {
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirIteration, ticks_second, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pirIteration, ticks_second, 30000000, bytes);
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].wrrInitialCredit, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirMax, 1, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pirMax, 1, 7000000, bytes);
        portCfg[port].group[group].totQueueCount = 8;
        portCfg[port].group[group].spQueueCount = 3;
        portCfg[port].group[group].wrrQueueCount = 5;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[0].wrrInitialCredit, 0, bytes);
        portCfg[port].group[group].Queue[0].congestionThresh = 1000;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[1].wrrInitialCredit, 0, bytes);
        portCfg[port].group[group].Queue[1].congestionThresh = 1000;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[2].wrrInitialCredit, 0, bytes);
        portCfg[port].group[group].Queue[2].congestionThresh = 1000;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[3].wrrInitialCredit, 8<<15, bytes);
        portCfg[port].group[group].Queue[3].congestionThresh = 1000;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[4].wrrInitialCredit, 16<<15, bytes);
        portCfg[port].group[group].Queue[4].congestionThresh = 1000;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[5].wrrInitialCredit, 24<<15, bytes);
        portCfg[port].group[group].Queue[5].congestionThresh = 1000;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[6].wrrInitialCredit, 0, bytes);
        portCfg[port].group[group].Queue[6].congestionThresh = 1000;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[7].wrrInitialCredit, 0, bytes);
        portCfg[port].group[group].Queue[7].congestionThresh = 1000;
    }

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 1
 *****************************************************************************/
int scenario_config_01 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 3;
    uint32_t queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 1))
    {
        return 1;
    }
    /*************************************************************************
     * Configure the packet generator as follows
     * Total (3) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (3) with (20M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (0) Queue (4) with (20M)bps and (1000)bytes/packet
     * Stream No.(3) input to Port (0) Group (0) Queue (5) with (20M)bps and (1000)bytes/packet
     *
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds.
     * 
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *    Stream No.(1) output with (?)pps or (5M)bps
     *    Stream No.(2) output with (?)pps or (10M)bps
     *    Stream No.(3) output with (?)pps or (15M)bps
     *Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 3;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < 3; queue++)
    {
         pktGen->Qs[queue].port       = 0;
         pktGen->Qs[queue].group      = 0;
         pktGen->Qs[queue].queue      = 3+queue;
         pktGen->Qs[queue].nSizes     = 1;
         pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
         pktGen->Qs[queue].pktSize[0] = 1000;
         pktGen->Qs[queue].pps        = 20000000 / (pktGen->Qs[queue].pktSize[0] * 8);
    }
    pktGen->Qs[0].ppsExp =  5000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 10000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp = 15000000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */
    pktGen->Qs[2].ppsError = pktGen->Qs[2].ppsExp / (100/5); /* 5% */


    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 2
 *****************************************************************************/
int scenario_config_02 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 3;
    uint32_t queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /* Scenario 2 uses same QoS config as Scenario 1 */
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 1))
    {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (3) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (3) with (20M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (0) Queue (4) with (20M)bps and (512)bytes/packet
     * Stream No.(3) input to Port (0) Group (0) Queue (5) with (20M)bps and (123)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * If N, results are read after wait_seconds.
     *
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(1) output with (?)pps or (5M)bps
     *     Stream No.(2) output with (?)pps or (10M)bps
     *     Stream No.(3) output with (?)pps or (15M)bps
     *Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 3;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[2].pktSize[0] = 123;
    for (queue = 0; queue < 3; queue++)
    {
         pktGen->Qs[queue].port     = 0;
         pktGen->Qs[queue].group    = 0;
         pktGen->Qs[queue].queue    = 3+queue;
         pktGen->Qs[queue].pps      = 20000000 / (pktGen->Qs[queue].pktSize[0] * 8);
         pktGen->Qs[queue].nSizes   = 1;
         pktGen->Qs[queue].sizeMode = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    }
    pktGen->Qs[0].ppsExp =  5000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 10000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp = 15000000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */
    pktGen->Qs[2].ppsError = pktGen->Qs[2].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 3
 *****************************************************************************/
int scenario_config_03 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t ports = 1;
    uint32_t pktGens = 3;
    uint32_t queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;
    /*************************************************************************
     * Total configured (1) Port
     * Port No.(0) configuration:
     *     Port CIR (50Mbps) CIR_MAX/CBS (7M)
     *     Total configured (1) Group
     *     Group CIR (0M) CIR_MAX/CBS (0M)
     *     Group PIR (30Mbps) PIR_MAX/PBS (7M)
     *     Group No.(0) configuration: wrrInitialCredit (0)
     *           Total configured (8) Queues: (8) Priority queues (0) WRR queues
     *            Priority queue No.(0) congestionThresh (1000)
     *            Priority queue No.(1) congestionThresh (1000)
     *            Priority queue No.(2) congestionThresh (1000)
     *            Priority queue No.(3) congestionThresh (1000)
     *            Priority queue No.(4) congestionThresh (1000)
     *            Priority queue No.(5) congestionThresh (1000)
     *            Priority queue No.(6) congestionThresh (1000)
     *            Priority queue No.(7) congestionThresh (1000)
     *************************************************************************/

    /* Make sure enough memory is in portCfg */
    if (*nPorts < 1) 
    {
        System_printf ("Build supports too few ports (%d, %d)\n", *nPorts, ports);
        return 1;
    }
    *nPorts = ports;
    memset (portCfg, 0, ports * sizeof(Qmss_QosSchedPortCfg));

    portCfg[0].wrrType = bytes;
    portCfg[0].cirType = bytes;
    portCfg[0].congestionType = Qmss_QosSchedAcctType_PACKETS;
    portCfg[0].outThrotType = Qmss_QosSchedAcctType_PACKETS;
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 50000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirMax, 1, 7000000, bytes);
    portCfg[0].overheadBytes = 0;
    portCfg[0].groupCount = 1;
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirIteration, ticks_second, 0, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].pirIteration, ticks_second, 30000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[0].wrrInitialCredit, 0, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirMax, 1, 0, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].pirMax, 1, 7000000, bytes);
    portCfg[0].group[0].totQueueCount = 8;
    portCfg[0].group[0].spQueueCount = 8;
    portCfg[0].group[0].wrrQueueCount = 0;
    for (queue = 0; queue < 8; queue++)
    {
        scenario_wrr_conversion_wrapper 
            (&portCfg[0].group[0].Queue[queue].wrrInitialCredit, 0, bytes);
        portCfg[0].group[0].Queue[queue].congestionThresh = 1000;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     *
     * Total (3) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (3) with (20M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (0) Queue (4) with (20M)bps and (512)bytes/packet
     * Stream No.(3) input to Port (0) Group (0) Queue (5) with (20M)bps and (123)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."
     *
     * Configure the expected results
     * "Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(0) output with (?)pps or (20M)bps
     *     Stream No.(1) output with (?)pps or (10M)bps
     *     Stream No.(2) output with (?)pps or (0M)bps
     * Tolerances for above each number that is checked (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 3;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[2].pktSize[0] = 123;
    for (queue = 0; queue < 3; queue++)
    {
         pktGen->Qs[queue].port     = 0;
         pktGen->Qs[queue].group    = 0;
         pktGen->Qs[queue].queue    = 3+queue;
         pktGen->Qs[queue].pps      = 20000000 / (pktGen->Qs[queue].pktSize[0] * 8);
         pktGen->Qs[queue].nSizes   = 1;
         pktGen->Qs[queue].sizeMode = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    }
    pktGen->Qs[0].ppsExp = 20000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 10000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp = 0;
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */
    pktGen->Qs[2].ppsError = pktGen->Qs[2].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 4
 *****************************************************************************/
int scenario_config_04 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t pktGens = 3;
    uint32_t queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /* Scenario 4 uses same QoS config as Scenario 1, except different port and group CIR */
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 1))
    {
        return 1;
    }

    /* Change CIR to 60Mbps */
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 60000000, bytes);
    /* Change group PIR to 60Mbps */
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].pirIteration, ticks_second, 60000000, bytes);

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (6) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (1) with (10M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (0) Queue (2) with (10M)bps and (512)bytes/packet
     * Stream No.(3) input to Port (0) Group (0) Queue (3) with (10M)bps and (123)bytes/packet
     * Stream No.(4) input to Port (0) Group (0) Queue (4) with (20M)bps and (1000)bytes/packet
     * Stream No.(5) input to Port (0) Group (0) Queue (5) with (20M)bps and (512)bytes/packet
     * Stream No.(6) input to Port (0) Group (0) Queue (6) with (20M)bps and (123)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."
     *
     * Configure the expected results
     * "Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(1) output with (?)pps or (10M)bps
     *     Stream No.(2) output with (?)pps or (10M)bps
     *     Stream No.(3) output with (?)pps or (10M)bps
     *     Stream No.(4) output with (?)pps or (5M)bps
     *     Stream No.(5) output with (?)pps or (10M)bps
     *     Stream No.(6) output with (?)pps or (15M)bps
     * Tolerances for above each number that is checked.(5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 6;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[2].pktSize[0] = 123;
    pktGen->Qs[3].pktSize[0] = 1000;
    pktGen->Qs[4].pktSize[0] = 512;
    pktGen->Qs[5].pktSize[0] = 123;
    for (queue = 0; queue < pktGens; queue++)
    {
         pktGen->Qs[queue].port     = 0;
         pktGen->Qs[queue].group    = 0;
         pktGen->Qs[queue].queue    = queue;
         pktGen->Qs[queue].nSizes   = 1;
         pktGen->Qs[queue].sizeMode = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    }
    pktGen->Qs[0].pps = 10000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].pps = 10000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].pps = 10000000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[3].pps = 20000000 / (pktGen->Qs[3].pktSize[0] * 8);
    pktGen->Qs[4].pps = 20000000 / (pktGen->Qs[4].pktSize[0] * 8);
    pktGen->Qs[5].pps = 20000000 / (pktGen->Qs[5].pktSize[0] * 8);
    pktGen->Qs[0].ppsExp = 10000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 10000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp = 10000000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[3].ppsExp =  5000000 / (pktGen->Qs[3].pktSize[0] * 8);
    pktGen->Qs[4].ppsExp = 10000000 / (pktGen->Qs[4].pktSize[0] * 8);
    pktGen->Qs[5].ppsExp = 15000000 / (pktGen->Qs[5].pktSize[0] * 8);
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */
    pktGen->Qs[2].ppsError = pktGen->Qs[2].ppsExp / (100/5); /* 5% */
    pktGen->Qs[3].ppsError = pktGen->Qs[3].ppsExp / (100/5); /* 5% */
    pktGen->Qs[4].ppsError = pktGen->Qs[4].ppsExp / (100/5); /* 5% */
    pktGen->Qs[5].ppsError = pktGen->Qs[5].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 5
 *****************************************************************************/
int scenario_config_05 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t group;
    uint32_t pktGens = 3;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /* Start with base config, but then modify CIR/PIR to:
     *    Port CIR (75Mbps) CIR_MAX/CBS (7M)
     *    Group CIR (15M) CIR_MAX/CBS (7M)
     *    Group PIR (50Mbps) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *    Group CIR (15M) CIR_MAX/CBS (7M)
     *    Group PIR (50Mbps) PIR_MAX/PBS (7M)
     *      Group No.(1) configuration: wrrInitialCredit (2)
     *    Group CIR (15M) CIR_MAX/CBS (7M)
     *    Group PIR (50Mbps) PIR_MAX/PBS (7M)
     *         Group No.(2) configuration: wrrInitialCredit (3)
     *************************************************************************/
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 3))
    {
        return 1;
    }
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 75000000, bytes);
    for (group = 0; group < 3; group++)
    {
        scenario_rate_conversion_wrapper 
            (&portCfg[0].group[group].cirIteration, ticks_second, 15000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[0].group[group].pirIteration, ticks_second, 50000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[0].group[group].cirMax, 1, 7000000, bytes);
        scenario_wrr_conversion_wrapper 
            (&portCfg[0].group[group].wrrInitialCredit, (group+1) << 15, bytes);
    }
    /*************************************************************************
     * Configure the packet generator as follows
     * "Total (3) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (1) with (50M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (1) Queue (3) with (50M)bps and (512)bytes/packet
     * Stream No.(3) input to Port (0) Group (2) Queue (5) with (50M)bps and (123)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."
     * 
     * Configure the expected results
     * "Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(1) output with (?)pps or (20M)bps
     *     Stream No.(2) output with (?)pps or (25M)bps
     *     Stream No.(3) output with (?)pps or (30M)bps
     * Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 3;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 1;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pps        = 50000000 / (pktGen->Qs[0].pktSize[0] * 8);

    pktGen->Qs[1].port       = 0;
    pktGen->Qs[1].group      = 1;
    pktGen->Qs[1].queue      = 3;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[1].nSizes     = 1;
    pktGen->Qs[1].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[1].pps        = 50000000 / (pktGen->Qs[1].pktSize[0] * 8);

    pktGen->Qs[2].port       = 0;
    pktGen->Qs[2].group      = 2;
    pktGen->Qs[2].queue      = 5;
    pktGen->Qs[2].pktSize[0] = 123;
    pktGen->Qs[2].nSizes     = 1;
    pktGen->Qs[2].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[2].pps        = 50000000 / (pktGen->Qs[2].pktSize[0] * 8);

    pktGen->Qs[0].ppsExp = 20000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 25000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp = 30000000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */
    pktGen->Qs[2].ppsError = pktGen->Qs[2].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 6
 *****************************************************************************/
int scenario_config_06 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t group;
    uint32_t pktGens = 3;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /* Start with base config, but then modify CIR/PIR to:
     *    Port CIR (75Mbps) CIR_MAX/CBS (7M)
     *    Group CIR (5M) CIR_MAX/CBS (7M)
     *    Group PIR (40Mbps) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *    Group CIR (5M) CIR_MAX/CBS (7M)
     *    Group PIR (40Mbps) PIR_MAX/PBS (7M)
     *      Group No.(1) configuration: wrrInitialCredit (2)
     *    Group CIR (5M) CIR_MAX/CBS (7M)
     *    Group PIR (10Mbps) PIR_MAX/PBS (7M)
     *         Group No.(2) configuration: wrrInitialCredit (3)
     *************************************************************************/
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 3))
    {
        return 1;
    }
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 75000000, bytes);
    for (group = 0; group < 3; group++)
    {
        scenario_rate_conversion_wrapper 
            (&portCfg[0].group[group].cirIteration, ticks_second, 5000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[0].group[group].pirIteration, ticks_second, 45000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[0].group[group].cirMax, 1, 7000000, bytes);
        scenario_wrr_conversion_wrapper 
            (&portCfg[0].group[group].wrrInitialCredit, (group+1) << 15, bytes);
    }
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[2].pirIteration, ticks_second, 15000000, bytes);

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (3) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (1) with (50M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (1) Queue (3) with (50M)bps and (512)bytes/packet
     * Stream No.(3) input to Port (0) Group (2) Queue (5) with (50M)bps and (123)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."
     * 
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *    Stream No.(1) output with (?)pps or (21.6M)bps
     *    Stream No.(2) output with (?)pps or (38.4M)bps
     *    Stream No.(3) output with (?)pps or (15M)bps
     * Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 3;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 1;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pps        = 50000000 / (pktGen->Qs[0].pktSize[0] * 8);

    pktGen->Qs[1].port       = 0;
    pktGen->Qs[1].group      = 1;
    pktGen->Qs[1].queue      = 3;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[1].nSizes     = 1;
    pktGen->Qs[1].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[1].pps        = 50000000 / (pktGen->Qs[1].pktSize[0] * 8);

    pktGen->Qs[2].port       = 0;
    pktGen->Qs[2].group      = 2;
    pktGen->Qs[2].queue      = 5;
    pktGen->Qs[2].pktSize[0] = 123;
    pktGen->Qs[2].nSizes     = 1;
    pktGen->Qs[2].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[2].pps        = 50000000 / (pktGen->Qs[2].pktSize[0] * 8);

    pktGen->Qs[0].ppsExp = 21600000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 38400000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp = 15000000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */
    pktGen->Qs[2].ppsError = pktGen->Qs[2].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 7
 *****************************************************************************/
int scenario_config_07 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t pktGens = 3;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /* Start with base config, but then modify CIR/PIR to:
     *    Port CIR (100Mbps) CIR_MAX/CBS (7M)
     *    Group CIR (10M) CIR_MAX/CBS (7M)
     *    Group PIR (100Mbps) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *    Group CIR (10M) CIR_MAX/CBS (7M)
     *    Group PIR (100Mbps) PIR_MAX/PBS (7M)
     *      Group No.(1) configuration: wrrInitialCredit (99)
     *************************************************************************/
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 2))
    {
        return 1;
    }
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 100000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].pirIteration, ticks_second, 100000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[0].wrrInitialCredit, 1 << 15, bytes);

    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].pirIteration, ticks_second, 100000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[1].wrrInitialCredit, 99 << 15, bytes);

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (2) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (1) with (100M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (1) Queue (3) with (100M)bps and (512)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."
     * 
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(1) output with (?)pps or (10.8M)bps
     *     Stream No.(2) output with (?)pps or (89.2M)bps
     * (?)% packets dropped from each input stream
     * (?)% packets forwarded from each input stream.
     * Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 2;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 1;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pps        = 100000000 / (pktGen->Qs[0].pktSize[0] * 8);

    pktGen->Qs[1].port       = 0;
    pktGen->Qs[1].group      = 1;
    pktGen->Qs[1].queue      = 3;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[1].nSizes     = 1;
    pktGen->Qs[1].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[1].pps        = 100000000 / (pktGen->Qs[1].pktSize[0] * 8);

    pktGen->Qs[0].ppsExp = 10800000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 89200000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 8
 *****************************************************************************/
int scenario_config_08 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t pktGens = 3;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /* Start with base config, but then modify CIR/PIR to:
     *    Port CIR (15Mbps) CIR_MAX/CBS (7M)
     *    Group CIR (5M) CIR_MAX/CBS (7M)
     *    Group PIR (10Mbps) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *    Group CIR (10M) CIR_MAX/CBS (7M)
     *    Group PIR (15Mbps) PIR_MAX/PBS (7M)
     *      Group No.(1) configuration: wrrInitialCredit (2)
     *************************************************************************/
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 2))
    {
        return 1;
    }
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 15000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirIteration, ticks_second, 5000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].pirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[0].wrrInitialCredit, 1 << 15, bytes);

    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].pirIteration, ticks_second, 15000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[1].wrrInitialCredit, 2 << 15, bytes);

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (2) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (1) with (20M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (1) Queue (3) with (20M)bps and (512)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."

     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(1) output with (?)pps or (5M)bps
     *     Stream No.(2) output with (?)pps or (10M)bps
     * Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 2;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 1;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pps        = 20000000 / (pktGen->Qs[0].pktSize[0] * 8);

    pktGen->Qs[1].port       = 0;
    pktGen->Qs[1].group      = 1;
    pktGen->Qs[1].queue      = 3;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[1].nSizes     = 1;
    pktGen->Qs[1].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[1].pps        = 20000000 / (pktGen->Qs[1].pktSize[0] * 8);

    pktGen->Qs[0].ppsExp = 5000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 10000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 9
 *****************************************************************************/
int scenario_config_09 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    uint32_t pktGens = 3;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /* Start with base config, but then modify CIR/PIR to:
     *    Port CIR (50Mbps) CIR_MAX/CBS (7M)
     *    Group CIR (5M) CIR_MAX/CBS (7M)
     *    Group PIR (10Mbps) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *    Group CIR (10M) CIR_MAX/CBS (7M)
     *    Group PIR (15Mbps) PIR_MAX/PBS (7M)
     *      Group No.(1) configuration: wrrInitialCredit (2)
     *************************************************************************/
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 2))
    {
        return 1;
    }
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 50000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirIteration, ticks_second, 5000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].pirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[0].wrrInitialCredit, 1 << 15, bytes);

    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].pirIteration, ticks_second, 15000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[1].wrrInitialCredit, 2 << 15, bytes);

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (2) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (1) with (20M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (1) Queue (3) with (20M)bps and (512)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."
     *
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(1) output with (?)pps or (10M)bps
     *     Stream No.(2) output with (?)pps or (15M)bps
     * Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 2;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 1;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pps        = 20000000 / (pktGen->Qs[0].pktSize[0] * 8);

    pktGen->Qs[1].port       = 0;
    pktGen->Qs[1].group      = 1;
    pktGen->Qs[1].queue      = 3;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[1].nSizes     = 1;
    pktGen->Qs[1].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[1].pps        = 20000000 / (pktGen->Qs[1].pktSize[0] * 8);

    pktGen->Qs[0].ppsExp = 10000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 15000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */

    return 0;
}

#ifndef WIDE  /* Following test cases require more than one port */

/*****************************************************************************
 * Configuration and expected results for scenario 10
 *****************************************************************************/
int scenario_config_10 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    Qmss_QosSchedAcctType packets = Qmss_QosSchedAcctType_PACKETS;
    uint32_t pktGens = 3;
    int nPorts2 = *nPorts - 1;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */
    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /*************************************************************************
     * Start with base config, but then modify CIR/PIR to:
     *    Port CIR (50Mbps) CIR_MAX/CBS (7M)
     *    Group CIR (5M) CIR_MAX/CBS (7M)
     *    Group PIR (10Mbps) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *    Group CIR (10M) CIR_MAX/CBS (7M)
     *    Group PIR (15Mbps) PIR_MAX/PBS (7M)
     *      Group No.(1) configuration: wrrInitialCredit (2)
     *************************************************************************/
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 2))
    {
        return 1;
    }
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 50000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirIteration, ticks_second, 5000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].pirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[0].wrrInitialCredit, 1 << 15, bytes);

    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].pirIteration, ticks_second, 15000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[1].wrrInitialCredit, 2 << 15, bytes);

    /*************************************************************************
     * For second port, start with base config, but then modify CIR/PIR to:
     *    set cir/pir to packets intead of bytes
     *    Port CIR (4K) CIR_MAX/CBS (7M)
     *    Group CIR (0) CIR_MAX/CBS (7M)
     *    Group PIR (1K) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *************************************************************************/
    if (qos_base_config_1 (&nPorts2, &portCfg[1], ticks_second, 1))
    {
        return 1;
    }
    (*nPorts)++;
    portCfg[1].cirType = packets;
    scenario_rate_conversion_wrapper 
        (&portCfg[1].cirIteration, ticks_second, 4000, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[1].group[0].cirIteration, ticks_second, 0, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[1].group[0].pirIteration, ticks_second, 1000, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[1].group[0].cirMax, 1, 2000, packets);
    scenario_wrr_conversion_wrapper 
        (&portCfg[1].group[0].wrrInitialCredit, 1 << 15, bytes);

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (3) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (1) with (20M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (1) Queue (3) with (20M)bps and (512)bytes/packet
     * Stream No.(3) input to Port (1) Group (0) Queue (3) with (10k)pps and (512)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."
     *
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(1) output with (?)pps or (10M)bps
     *     Stream No.(2) output with (?)pps or (15M)bps
     *     Stream No.(2) output with (1k)pps or ()bps
     * Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 3;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 1;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pps        = 20000000 / (pktGen->Qs[0].pktSize[0] * 8);

    pktGen->Qs[1].port       = 0;
    pktGen->Qs[1].group      = 1;
    pktGen->Qs[1].queue      = 3;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[1].nSizes     = 1;
    pktGen->Qs[1].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[1].pps        = 20000000 / (pktGen->Qs[1].pktSize[0] * 8);

    pktGen->Qs[2].port       = 1;
    pktGen->Qs[2].group      = 0;
    pktGen->Qs[2].queue      = 3;
    pktGen->Qs[2].pktSize[0] = 512;
    pktGen->Qs[2].nSizes     = 1;
    pktGen->Qs[2].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[2].pps        = 10000;

    pktGen->Qs[0].ppsExp = 10000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 15000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp = 1000;
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */
    pktGen->Qs[2].ppsError = pktGen->Qs[2].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 11 and 12
 *****************************************************************************/
int scenario_config_11_12 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts, uint8_t removeBytes)
{
    uint32_t ticks_second = 1000000 / 50;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    Qmss_QosSchedAcctType packets = Qmss_QosSchedAcctType_PACKETS;
    uint32_t pktGens = 3;
    int nPorts2 = *nPorts - 1;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    /*************************************************************************
     * Start with base config, but then modify CIR/PIR to:
     *    Port CIR (50Mbps) CIR_MAX/CBS (7M)
     *    Group CIR (5M) CIR_MAX/CBS (7M)
     *    Group PIR (10Mbps) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *    Group CIR (10M) CIR_MAX/CBS (7M)
     *    Group PIR (15Mbps) PIR_MAX/PBS (7M)
     *      Group No.(1) configuration: wrrInitialCredit (2)
     *************************************************************************/
    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 2))
    {
        return 1;
    }
    scenario_rate_conversion_wrapper 
        (&portCfg[0].cirIteration, ticks_second, 50000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirIteration, ticks_second, 5000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].pirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[0].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[0].wrrInitialCredit, 1 << 15, bytes);

    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirIteration, ticks_second, 10000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].pirIteration, ticks_second, 15000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[0].group[1].cirMax, 1, 7000000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[0].group[1].wrrInitialCredit, 2 << 15, bytes);

    portCfg[0].removeBytes = removeBytes;

    /*************************************************************************
     * For second port, start with base config, but then modify CIR/PIR to:
     *    set cir/pir to packets intead of bytes
     *    Port CIR (4K) CIR_MAX/CBS (7M)
     *    Group CIR (0) CIR_MAX/CBS (7M)
     *    Group PIR (64) PIR_MAX/PBS (7M)
     *    Group No.(0) configuration: wrrInitialCredit (1)
     *************************************************************************/
    if (qos_base_config_1 (&nPorts2, &portCfg[1], ticks_second, 1))
    {
        return 1;
    }
    (*nPorts)++;
    portCfg[1].cirType = packets;
    scenario_rate_conversion_wrapper 
        (&portCfg[1].cirIteration, ticks_second, 4000, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[1].group[0].cirIteration, ticks_second, 0, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[1].group[0].pirIteration, ticks_second, 64, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[1].group[0].cirMax, 1, 2000, packets);
    scenario_wrr_conversion_wrapper 
        (&portCfg[1].group[0].wrrInitialCredit, 1 << 15, bytes);

    portCfg[1].removeBytes = removeBytes;
    /*************************************************************************
     * Configure the packet generator as follows
     * Total (3) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (1) with (20M)bps and (1000)bytes/packet
     * Stream No.(2) input to Port (0) Group (1) Queue (3) with (20M)bps and (512)bytes/packet
     * Stream No.(3) input to Port (1) Group (0) Queue (3) with (10k)pps and (512)bytes/packet
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds."
     *
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *     Stream No.(1) output with (?)pps or (10M)bps
     *     Stream No.(2) output with (?)pps or (15M)bps
     *     Stream No.(3) output with (64)pps or ()bps
     * Tolerances for above each number that is checked. (5%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 3;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 1;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pps        = 20000000 / (pktGen->Qs[0].pktSize[0] * 8);

    pktGen->Qs[1].port       = 0;
    pktGen->Qs[1].group      = 1;
    pktGen->Qs[1].queue      = 3;
    pktGen->Qs[1].pktSize[0] = 512;
    pktGen->Qs[1].nSizes     = 1;
    pktGen->Qs[1].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[1].pps        = 20000000 / (pktGen->Qs[1].pktSize[0] * 8);

    pktGen->Qs[2].port       = 1;
    pktGen->Qs[2].group      = 0;
    pktGen->Qs[2].queue      = 3;
    pktGen->Qs[2].pktSize[0] = 512;
    pktGen->Qs[2].nSizes     = 1;
    pktGen->Qs[2].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[2].pps        = 10000;

    pktGen->Qs[0].ppsExp = 10000000 / ((pktGen->Qs[0].pktSize[0] - removeBytes) * 8);
    pktGen->Qs[1].ppsExp = 15000000 / ((pktGen->Qs[1].pktSize[0] - removeBytes) * 8);
    pktGen->Qs[2].ppsExp = 64;
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/5); /* 5% */
    pktGen->Qs[1].ppsError = pktGen->Qs[1].ppsExp / (100/5); /* 5% */
    pktGen->Qs[2].ppsError = pktGen->Qs[2].ppsExp / (100/5); /* 5% */

    return 0;
}

/*****************************************************************************
 * Configuration and expected results for scenario 11
 *****************************************************************************/
int scenario_config_11 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    cfg->portMap = NULL; /* no map required */
    return scenario_config_11_12 (timer, cfg, pktGen, nPorts, 0);
}

/*****************************************************************************
 * Configuration and expected results for scenario 12
 *****************************************************************************/
int scenario_config_12 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    cfg->portMap = NULL; /* no map required */
    return scenario_config_11_12 (timer, cfg, pktGen, nPorts, 50);
}

/*****************************************************************************
 * Configuration and expected results for scenario 13
 *****************************************************************************/
int scenario_config_13 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
   return scenario_config_common_joint (timer, cfg->portCfg, cfg->portMap, pktGen, nPorts, 8);
}

/*****************************************************************************
 * Configuration and expected results for scenario 14
 *****************************************************************************/
int scenario_config_14 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
   return scenario_config_common_joint (timer, cfg->portCfg, cfg->portMap, pktGen, nPorts, 5);
}

/*****************************************************************************
 * Configuration and expected results for scenario 15
 *****************************************************************************/
int scenario_config_15 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
   return scenario_config_common_joint (timer, cfg->portCfg, cfg->portMap, pktGen, nPorts, 4);
}

#else
int scenario_config_wide_01 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    Qmss_QosSchedAcctType bytes   = Qmss_QosSchedAcctType_BYTES;
    uint32_t ports                = 1;
    uint32_t nGroups              = 17;
    uint32_t nQueuesInGroup       = 8;
    uint32_t rateMbps             = 200000000; // this is limited by test program, not qos
    uint32_t pktSize              = 100;
    uint32_t ticks_second         = 1000000 / 50;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;
    uint32_t port, group, queue, pktGens;
    uint32_t aggPpsSent           = 0;
    uint32_t aggPpsExp            = 0;

    cfg->portMap = NULL; /* no map required */

    /* Make sure enough memory is in portCfg */
    if (*nPorts < 1) 
    {
        System_printf ("Build supports too few ports (%d, %d)\n", *nPorts, ports);
        return 1;
    }
    *nPorts = ports;
    memset (portCfg, 0, ports * sizeof(Qmss_QosSchedPortCfg));

    port = 0;
    portCfg[port].wrrType = bytes;
    portCfg[port].cirType = bytes;
    portCfg[port].congestionType = Qmss_QosSchedAcctType_PACKETS;
    portCfg[port].outThrotType = Qmss_QosSchedAcctType_PACKETS;
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirIteration, ticks_second, rateMbps, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirMax, 1, 7000000, bytes);
    portCfg[port].overheadBytes = 0;
    portCfg[port].groupCount = nGroups;
    for (group = 0; group < nGroups; group++) 
    {
        /* use same config as port to make wrr weights kick in */
        portCfg[port].group[group].pirIteration = portCfg[port].cirIteration;
        portCfg[port].group[group].pirMax       = portCfg[port].cirMax;
        /* Set 1:2..16:17 weights within group */
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].wrrInitialCredit, (group + 1) << 10, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirIteration, ticks_second, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirMax, 1, 0, bytes);
        portCfg[port].group[group].totQueueCount = nQueuesInGroup;
        portCfg[port].group[group].spQueueCount = 0;
        portCfg[port].group[group].wrrQueueCount = nQueuesInGroup;
        for (queue = 0; queue < nQueuesInGroup; queue++)
        {
            scenario_wrr_conversion_wrapper 
                (&portCfg[port].group[group].Queue[queue].wrrInitialCredit, (8 - queue) << 10, bytes);
            portCfg[port].group[group].Queue[queue].congestionThresh = 10;
        }
    }

    /* Make sure enough memory is in pktGen */
    pktGens = nGroups * nQueuesInGroup;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < pktGens; queue++)
    {
         uint32_t groupSharePps, queueSharePps;
         uint32_t runTime;

         pktGen->Qs[queue].port       = 0;
         pktGen->Qs[queue].group      = queue >> 3;
         pktGen->Qs[queue].queue      = queue & 0x7;
         pktGen->Qs[queue].nSizes     = 1;
         pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
         pktGen->Qs[queue].pktSize[0] = pktSize;
         groupSharePps                = (rateMbps / 8 / pktGen->Qs[queue].pktSize[0]) * (1 + pktGen->Qs[queue].group) / (nGroups * (nGroups + 1) / 2);
         queueSharePps                = groupSharePps * (8 - pktGen->Qs[queue].queue) / (nQueuesInGroup * (nQueuesInGroup + 1) / 2);
         pktGen->Qs[queue].pps        = queueSharePps * 11 / 10;
         pktGen->Qs[queue].ppsExp     = queueSharePps;
         /* Error is relative to runtime.  If test runs for 0.5 second, its 10pps; 1 second 5pps, 5 or more seconds, 1pps */
         runTime = pktGen->readTime;
         runTime /= 100;
         if (runTime > 5000) 
         {
             pktGen->Qs[queue].ppsError   = 1;
         }
         else
         {
             pktGen->Qs[queue].ppsError   = 5000 / runTime;
         }
         aggPpsSent                  += pktGen->Qs[queue].pps;
         aggPpsExp                   += pktGen->Qs[queue].ppsExp;
    }

    System_printf ("Aggregate PPS to send: %d; Aggregate PPS expected: %d\n", aggPpsSent, aggPpsExp);

    return 0;
}

#endif

/*****************************************************************************
 * Configuration and expected results for scenario 16
 *****************************************************************************/
int scenario_config_16 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts)
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1;
    uint32_t queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;
    Qmss_QosSchedAcctType packets = Qmss_QosSchedAcctType_PACKETS;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_base_config_1 (nPorts, portCfg, ticks_second, 1))
    {
        return 1;
    }

    /* Set rate to 25000 pps */
    portCfg[0].cirType = packets;
    scenario_rate_conversion_wrapper
        (&portCfg[0].cirIteration, ticks_second, 25000, packets);
    scenario_rate_conversion_wrapper
        (&portCfg[0].cirMax, 1, 2000, packets);
    scenario_rate_conversion_wrapper
        (&portCfg[0].group[0].cirIteration, ticks_second, 25000, packets);
    scenario_rate_conversion_wrapper
        (&portCfg[0].group[0].pirIteration, ticks_second, 0, packets);
    scenario_rate_conversion_wrapper
        (&portCfg[0].group[0].cirMax, 1, 2000, packets);

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (1) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (0) with (25000)pps and (0)bytes/packet
     *
     * How long to generate traffic before reading results (60) (wait_seconds)
     * Stop traffic generation before examining results (Y/N):N
     * o   If Y, traffic is stopped after wait_seconds, and result is read at instant last packet received from QoS.
     * o   If N, results are read after wait_seconds.
     *
     * Configure the expected results
     * Total (?)pps/(?)bps present to each output port.  (Its valid to test when the input traffic is LESS than port CIR)
     *    Stream No.(1) output with (25000)pps
     *Tolerances for above each number that is checked. (1%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 1;
    if (pktGen->numQCfgs < pktGens)
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }

    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < 1; queue++)
    {
         pktGen->Qs[queue].port       = 0;
         pktGen->Qs[queue].group      = 0;
         pktGen->Qs[queue].queue      = queue;
         pktGen->Qs[queue].nSizes     = 1;
         pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
         pktGen->Qs[queue].pktSize[0] = 0;
         pktGen->Qs[queue].pps        = 25000;
    }
    pktGen->Qs[0].ppsExp =  25000;
    pktGen->Qs[0].ppsError = pktGen->Qs[0].ppsExp / (100/1); /* 1% */

    return 0;
}


#ifdef TEST_SIMUBYTEPKT
int qos_simu_base_config_1 (int32_t *nPorts, Qmss_QosSchedPortCfg *portCfg, 
                            uint32_t ticks_second)
{
    Qmss_QosSchedAcctType packets = Qmss_QosSchedAcctType_PACKETS;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    Qmss_QosSchedAcctType both = Qmss_QosSchedAcctType_BOTH;
    uint32_t ports = 1;
    uint32_t nGroups = 1;
    uint32_t port, group;

    /*************************************************************************
     * Configure the 1 port with 1 group as follows:
     *
     * Total configured (1) Port
     * Port No.(0) configuration:
     *     Port CIR (4Mbps and 4Kpps)
     *     Group CIR (4Mbps and 4Kpps)
     *     Group PIR (0)
     *     Group No.(0) configuration: wrrInitialCredit (0)
     *           Total configured (1) Queues: (1) Priority queues
     *************************************************************************/

    /* Make sure enough memory is in portCfg */
    if (*nPorts < 1) 
    {
        System_printf ("Build supports too few ports (%d, %d)\n", *nPorts, ports);
        return 1;
    }
    *nPorts = ports;
    memset (portCfg, 0, ports * sizeof(Qmss_QosSchedPortCfg));

    port = 0;
    portCfg[port].wrrType = bytes;
    portCfg[port].cirType = both;
    portCfg[port].congestionType = packets;
    portCfg[port].outThrotType = packets;
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirIteration, ticks_second, 4000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].pktCirIteration, ticks_second, 4000, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirMax, 1, 7000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].pktCirMax, 1, 10, packets);
    portCfg[port].overheadBytes = 0;
    portCfg[port].groupCount = 1;
    for (group = 0; group < nGroups; group++) 
    {
        portCfg[port].group[group].cirType = both;
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirIteration, ticks_second, 4000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktCirIteration, ticks_second, 4000, packets);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pirIteration, ticks_second, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktPirIteration, ticks_second, 0, packets);
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].wrrInitialCredit, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirMax, 1, 7000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktCirMax, 1, 10, packets);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pirMax, 1, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktPirMax, 1, 0, packets);
        portCfg[port].group[group].totQueueCount = 1;
        portCfg[port].group[group].spQueueCount = 1;
        portCfg[port].group[group].wrrQueueCount = 0;
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[0].wrrInitialCredit, 0, bytes);
        portCfg[port].group[group].Queue[0].congestionThresh = 1000;
    }

    return 0;
} /* qos_simu_base_config_1 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 01
 *****************************************************************************/
int scenario_config_simu_01 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_1 (nPorts, portCfg, ticks_second))
    {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (1) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (0) with 6Kpps of (64)bytes/packet
     *
     * Configure the expected results
     *    Stream No.(1) output with (4000)pps
     *Tolerances for above each number that is checked. (1%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 1;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 0;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pktSize[0] = 64;
    pktGen->Qs[0].pps        = 6000;
    pktGen->Qs[0].ppsExp     = 4000;
    pktGen->Qs[0].ppsError   = pktGen->Qs[0].ppsExp / (100/1); /* 1% */

    return 0;
} /* scenario_config_simu_01 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 02
 *****************************************************************************/
int scenario_config_simu_02 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_1 (nPorts, portCfg, ticks_second))
    {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (1) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (0) with 2Kpps of (1000)bytes/packet
     *
     * Configure the expected results
     *    Stream No.(1) output with 4Mbps
     *Tolerances for above each number that is checked. (1%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 1;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 0;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].pps        = 2000;
    pktGen->Qs[0].ppsExp     = 4000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[0].ppsError   = pktGen->Qs[0].ppsExp / (100/1); /* 1% */

    return 0;
} /* scenario_config_simu_02 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 03
 *****************************************************************************/
int scenario_config_simu_03 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_1 (nPorts, portCfg, ticks_second))
    {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (1) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (0) with 6Kpps of (1000)bytes/packet
     *
     * Configure the expected results
     *    Stream No.(1) output with 4Mbps
     *Tolerances for above each number that is checked. (1%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 1;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 0;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pktSize[0] = 1000;
    pktGen->Qs[0].pps        = 6000;
    pktGen->Qs[0].ppsExp     = 4000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[0].ppsError   = pktGen->Qs[0].ppsExp / (100/1); /* 1% */

    return 0;	
} /* scenario_config_simu_03 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 04
 *****************************************************************************/
int scenario_config_simu_04 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_1 (nPorts, portCfg, ticks_second)) {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (1) Input streams
     * Stream No.(1) input to Port (0) Group (0) Queue (0) with 20Kpps of (64)bytes/packet
     *
     * Configure the expected results
     *    Stream No.(1) output with 4Kpps
     *Tolerances for above each number that is checked. (1%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 1;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 60000;
    pktGen->readTime = 60000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    pktGen->Qs[0].port       = 0;
    pktGen->Qs[0].group      = 0;
    pktGen->Qs[0].queue      = 0;
    pktGen->Qs[0].nSizes     = 1;
    pktGen->Qs[0].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
    pktGen->Qs[0].pktSize[0] = 64;
    pktGen->Qs[0].pps        = 20000;
    pktGen->Qs[0].ppsExp     = 4000;
    pktGen->Qs[0].ppsError   = pktGen->Qs[0].ppsExp / (100/1); /* 1% */

    return 0;
} /* scenario_config_simu_04 */

int qos_simu_base_config_2 (int32_t *nPorts, Qmss_QosSchedPortCfg *portCfg, 
                            uint32_t ticks_second)
{
    Qmss_QosSchedAcctType packets = Qmss_QosSchedAcctType_PACKETS;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    Qmss_QosSchedAcctType both = Qmss_QosSchedAcctType_BOTH;
    uint32_t ports = 1;
    uint32_t nGroups = 1;
    uint32_t port, group, queue;

    /*************************************************************************
     * Configure the 1 port with 1 group as follows:
     *
     * Total configured (1) Port
     * Port No.(0) configuration:
     *     Port CIR (100Kpps and 100Mbps)
     *     Group CIR (100Kpps and 100Mbps)
     *     Group PIR (0)
     *     Group No.(0) configuration: wrrInitialCredit (0)
     *           Total configured (8) Queues: (3) SP (5) WRR queues
     *           Weights: 10:10:10:5:5
     *************************************************************************/

    /* Make sure enough memory is in portCfg */
    if (*nPorts < 1) 
    {
        System_printf ("Build supports too few ports (%d, %d)\n", *nPorts, ports);
        return 1;
    }
    *nPorts = ports;
    memset (portCfg, 0, ports * sizeof(Qmss_QosSchedPortCfg));

    port = 0;
    portCfg[port].wrrType = bytes;
    portCfg[port].cirType = both;
    portCfg[port].congestionType = packets;
    portCfg[port].outThrotType = packets;
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirIteration, ticks_second, 100000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].pktCirIteration, ticks_second, 100000, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirMax, 1, 7000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].pktCirMax, 1, 100, packets);
    portCfg[port].overheadBytes = 0;
    portCfg[port].groupCount = 1;
    for (group = 0; group < nGroups; group++) 
    {
        portCfg[port].group[group].cirType = both;
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirIteration, ticks_second, 100000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktCirIteration, ticks_second, 100000, packets);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pirIteration, ticks_second, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktPirIteration, ticks_second, 0, packets);
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].wrrInitialCredit, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirMax, 1, 7000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktCirMax, 1, 100, packets);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pirMax, 1, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktPirMax, 1, 0, packets);
        portCfg[port].group[group].totQueueCount = 8;
        portCfg[port].group[group].spQueueCount = 3;
        portCfg[port].group[group].wrrQueueCount = 5;
	for (queue = 0; queue < 8; queue++)
	{
            scenario_wrr_conversion_wrapper 
                (&portCfg[port].group[group].Queue[queue].wrrInitialCredit, 0, bytes);
            portCfg[port].group[group].Queue[queue].congestionThresh = 100;
	}
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[3].wrrInitialCredit, 8000, bytes);
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[4].wrrInitialCredit, 8000, bytes);
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[5].wrrInitialCredit, 8000, bytes);
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[6].wrrInitialCredit, 4000, bytes);
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].Queue[7].wrrInitialCredit, 4000, bytes);
    }

    return 0;
} /* qos_simu_base_config_2 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 05
 *****************************************************************************/
int scenario_config_simu_05 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1, queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_2 (nPorts, portCfg, ticks_second)) {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (8) Input streams
     * Stream No.(N) input to Port (0) Group (0) Queue (N) with 20Mbps of (256)bytes/packet
     *
     * Configure the expected results
     *    Stream No.(0) output with 20Mbps
     *    Stream No.(1) output with 20Mbps
     *    Stream No.(2) output with 20Mbps
     *    Stream No.(3) output with 10Mbps
     *    Stream No.(4) output with 10Mbps
     *    Stream No.(5) output with 10Mbps
     *    Stream No.(6) output with 5Mbps
     *    Stream No.(7) output with 5Mbps
     *Tolerances for above each number that is checked. (2%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 8;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 100000;
    pktGen->readTime = 100000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].port       = 0;
        pktGen->Qs[queue].group      = 0;
        pktGen->Qs[queue].queue      = queue;
        pktGen->Qs[queue].nSizes     = 1;
        pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[queue].pktSize[0] = 256;
        pktGen->Qs[queue].pps        = 20000000 / (pktGen->Qs[queue].pktSize[0] * 8);
    }
    pktGen->Qs[0].ppsExp = 20000000 / (pktGen->Qs[0].pktSize[0] * 8);
    pktGen->Qs[1].ppsExp = 20000000 / (pktGen->Qs[1].pktSize[0] * 8);
    pktGen->Qs[2].ppsExp = 20000000 / (pktGen->Qs[2].pktSize[0] * 8);
    pktGen->Qs[3].ppsExp = 10000000 / (pktGen->Qs[3].pktSize[0] * 8);
    pktGen->Qs[4].ppsExp = 10000000 / (pktGen->Qs[4].pktSize[0] * 8);
    pktGen->Qs[5].ppsExp = 10000000 / (pktGen->Qs[5].pktSize[0] * 8);
    pktGen->Qs[6].ppsExp =  5000000 / (pktGen->Qs[6].pktSize[0] * 8);
    pktGen->Qs[7].ppsExp =  5000000 / (pktGen->Qs[7].pktSize[0] * 8);
    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].ppsError   = pktGen->Qs[queue].ppsExp / (100/2); /* 2% */
    }

    return 0;
} /* scenario_config_simu_05 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 06
 *****************************************************************************/
int scenario_config_simu_06 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1, queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_2 (nPorts, portCfg, ticks_second)) {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (8) Input streams
     * Stream No.(N) input to Port (0) Group (0) Queue (N) with 15Kpps of 64b
     * (120000pps exceeds 100K limit)
     *
     * Configure the expected results
     *    Stream No.(0) output with (15000pps)
     *    Stream No.(1) output with (15000pps)
     *    Stream No.(2) output with (15000pps)
     *    Stream No.(3) output with (13750pps)
     *    Stream No.(4) output with (13750pps)
     *    Stream No.(5) output with (13750pps)
     *    Stream No.(6) output with (6875pps)
     *    Stream No.(7) output with (6875pps)
     *Tolerances for above each number that is checked. (2%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 8;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 100000;
    pktGen->readTime = 100000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].port       = 0;
        pktGen->Qs[queue].group      = 0;
        pktGen->Qs[queue].queue      = queue;
        pktGen->Qs[queue].nSizes     = 1;
        pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[queue].pktSize[0] = 64;
        pktGen->Qs[queue].pps        = 15000;
    }
    pktGen->Qs[0].ppsExp = 15000;
    pktGen->Qs[1].ppsExp = 15000;
    pktGen->Qs[2].ppsExp = 15000;
    pktGen->Qs[3].ppsExp = 13750;
    pktGen->Qs[4].ppsExp = 13750;
    pktGen->Qs[5].ppsExp = 13750;
    pktGen->Qs[6].ppsExp =  6875;
    pktGen->Qs[7].ppsExp =  6875;
    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].ppsError   = pktGen->Qs[queue].ppsExp / (100/2); /* 2% */
    }

    return 0;
} /* scenario_config_simu_06 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 07
 *****************************************************************************/
int scenario_config_simu_07 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1, queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_2 (nPorts, portCfg, ticks_second)) {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (8) Input streams
     * Stream No.(N) input to Port (0) Group (0) Queue (N) with 15Kpps of 64b
     * except queue 6 gets 7.5Kpps of 128b packets.  This shows WRR is still
     * byte-based not packet based
     *
     * 45K is "lost" by SP, so 55K needs to come out the WRR with shares
     * 4/4/4/1/2 or 3666.67/sh.  If it WRR was by packet it would have been
     * 2/2/2/1/1 as before.
     *
     * Configure the expected results
     *    Stream No.(0) output with (15000pps)
     *    Stream No.(1) output with (15000pps)
     *    Stream No.(2) output with (15000pps)
     *    Stream No.(3) output with (14667pps)
     *    Stream No.(4) output with (14667pps)
     *    Stream No.(5) output with (14667pps)
     *    Stream No.(6) output with (3667pps)
     *    Stream No.(7) output with (7333pps)
     *Tolerances for above each number that is checked. (2%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 8;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 100000;
    pktGen->readTime = 100000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].port       = 0;
        pktGen->Qs[queue].group      = 0;
        pktGen->Qs[queue].queue      = queue;
        pktGen->Qs[queue].nSizes     = 1;
        pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[queue].pktSize[0] = 64;
        pktGen->Qs[queue].pps        = 15000;
    }
    pktGen->Qs[6].pktSize[0] = 128;
    pktGen->Qs[6].pps        = 7500;
    pktGen->Qs[0].ppsExp = 15000;
    pktGen->Qs[1].ppsExp = 15000;
    pktGen->Qs[2].ppsExp = 15000;
    pktGen->Qs[3].ppsExp = 14667;
    pktGen->Qs[4].ppsExp = 14667;
    pktGen->Qs[5].ppsExp = 14667;
    pktGen->Qs[6].ppsExp =  3667;
    pktGen->Qs[7].ppsExp =  7333;
    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].ppsError   = pktGen->Qs[queue].ppsExp / (100/2); /* 2% */
    }

    return 0;
} /* scenario_config_simu_07 */

int qos_simu_base_config_3 (int32_t *nPorts, Qmss_QosSchedPortCfg *portCfg, 
                            uint32_t ticks_second)
{
    Qmss_QosSchedAcctType packets = Qmss_QosSchedAcctType_PACKETS;
    Qmss_QosSchedAcctType bytes = Qmss_QosSchedAcctType_BYTES;
    Qmss_QosSchedAcctType both = Qmss_QosSchedAcctType_BOTH;
    uint32_t ports = 1;
    uint32_t nGroups = 5;
    uint32_t port, group, queue;

    /*************************************************************************
     * Configure the 1 port with 1 group as follows:
     *
     * Total configured (1) Port
     * Port No.(0) configuration:
     *     Port CIR (100Kpps and 100Mbps)
     *     Group CIR (0)
     *     Group PIR (100Kpps and 100Mbps)
     *     Group No.(0..4) configuration: wrrInitialCredit (10:10:10:5:5)
     *           Total configured (1) SP
     *           Weights: N/A
     *************************************************************************/

    /* Make sure enough memory is in portCfg */
    if (*nPorts < 1) 
    {
        System_printf ("Build supports too few ports (%d, %d)\n", *nPorts, ports);
        return 1;
    }
    *nPorts = ports;
    memset (portCfg, 0, ports * sizeof(Qmss_QosSchedPortCfg));

    port = 0;
    portCfg[port].wrrType = bytes;
    portCfg[port].cirType = both;
    portCfg[port].congestionType = packets;
    portCfg[port].outThrotType = packets;
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirIteration, ticks_second, 100000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].pktCirIteration, ticks_second, 100000, packets);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].cirMax, 1, 7000000, bytes);
    scenario_rate_conversion_wrapper 
        (&portCfg[port].pktCirMax, 1, 100, packets);
    portCfg[port].overheadBytes = 0;
    portCfg[port].groupCount = 5;
    for (group = 0; group < nGroups; group++) 
    {
        portCfg[port].group[group].cirType = both;
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirIteration, ticks_second, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktCirIteration, ticks_second, 0, packets);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pirIteration, ticks_second, 100000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktPirIteration, ticks_second, 100000, packets);
        scenario_wrr_conversion_wrapper 
            (&portCfg[port].group[group].wrrInitialCredit, 10<<15, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].cirMax, 1, 0, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktCirMax, 1, 0, packets);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pirMax, 1, 7000000, bytes);
        scenario_rate_conversion_wrapper 
            (&portCfg[port].group[group].pktPirMax, 1, 100, packets);
        portCfg[port].group[group].totQueueCount = 1;
        portCfg[port].group[group].spQueueCount = 1;
        portCfg[port].group[group].wrrQueueCount = 0;
	for (queue = 0; queue < 1; queue++)
	{
            scenario_wrr_conversion_wrapper 
                (&portCfg[port].group[group].Queue[queue].wrrInitialCredit, 0, bytes);
            portCfg[port].group[group].Queue[queue].congestionThresh = 100;
	}
    }
    scenario_wrr_conversion_wrapper 
        (&portCfg[port].group[0].wrrInitialCredit, 8000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[port].group[1].wrrInitialCredit, 8000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[port].group[2].wrrInitialCredit, 8000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[port].group[3].wrrInitialCredit, 4000, bytes);
    scenario_wrr_conversion_wrapper 
        (&portCfg[port].group[4].wrrInitialCredit, 4000, bytes);

    return 0;
} /* qos_simu_base_config_3 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 07
 *****************************************************************************/
int scenario_config_simu_08 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1, queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_3 (nPorts, portCfg, ticks_second)) {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (5) Input streams
     * Stream No.(N) input to Port (0) Group (0) Queue (N) with 25Mbps of 256b
     * (125MBps exceeds 100Mbps limit)
     *
     * 100Mbps of 256b packets is 48828pps.  Thus 10/10/10/5/5 ratio is 
     * 12207:12207:12207:6103:6103
     *    Stream No.(0) output with (12207pps)
     *    Stream No.(1) output with (12207pps)
     *    Stream No.(2) output with (12207pps)
     *    Stream No.(3) output with (6103pps)
     *    Stream No.(4) output with (6103pps)
     *Tolerances for above each number that is checked. (2%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 5;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 100000;
    pktGen->readTime = 100000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].port       = 0;
        pktGen->Qs[queue].group      = queue;
        pktGen->Qs[queue].queue      = 0;
        pktGen->Qs[queue].nSizes     = 1;
        pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[queue].pktSize[0] = 256;
        pktGen->Qs[queue].pps        = 25000000 / (pktGen->Qs[queue].pktSize[0] * 8);
    }
    pktGen->Qs[0].ppsExp = 12207;
    pktGen->Qs[1].ppsExp = 12207;
    pktGen->Qs[2].ppsExp = 12207;
    pktGen->Qs[3].ppsExp = 6103;
    pktGen->Qs[4].ppsExp = 6103;
    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].ppsError   = pktGen->Qs[queue].ppsExp / (100/2); /* 2% */
    }

    return 0;
} /* scenario_config_simu_08 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 08
 *****************************************************************************/
int scenario_config_simu_09 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1, queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_3 (nPorts, portCfg, ticks_second)) {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (5) Input streams
     * Stream No.(N) input to Port (0) Group (0) Queue (N) with 25Kpps of 64b
     * (125000pps exceeds 100K limit)
     *
     *
     * Configure the expected results
     *    Stream No.(0) output with (25000pps)
     *    Stream No.(1) output with (25000pps)
     *    Stream No.(2) output with (25000pps)
     *    Stream No.(3) output with (12500pps)
     *    Stream No.(4) output with (12500pps)
     *Tolerances for above each number that is checked. (2%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 5;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 100000;
    pktGen->readTime = 100000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].port       = 0;
        pktGen->Qs[queue].group      = queue;
        pktGen->Qs[queue].queue      = 0;
        pktGen->Qs[queue].nSizes     = 1;
        pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[queue].pktSize[0] = 64;
        pktGen->Qs[queue].pps        = 30000;
    }
    pktGen->Qs[0].ppsExp = 25000;
    pktGen->Qs[1].ppsExp = 25000;
    pktGen->Qs[2].ppsExp = 25000;
    pktGen->Qs[3].ppsExp = 12500;
    pktGen->Qs[4].ppsExp = 12500;
    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].ppsError   = pktGen->Qs[queue].ppsExp / (100/2); /* 2% */
    }

    return 0;
} /* scenario_config_simu_09 */

/*****************************************************************************
 * Configuration and expected results for simutaneous byte/packet scenario 08
 *****************************************************************************/
int scenario_config_simu_10 (uint32_t *timer, qosSchedScenCfg *cfg, packetGenCfg *pktGen, int *nPorts) 
{
    uint32_t ticks_second = 1000000 / 50;
    uint32_t pktGens = 1, queue;
    Qmss_QosSchedPortCfg *portCfg = cfg->portCfg;

    cfg->portMap = NULL; /* no map required */

    /*************************************************************************
     * Configure the timer to 50us
     *************************************************************************/
    *timer = CORE_SPEED / ticks_second / 3 / 2;

    if (qos_simu_base_config_3 (nPorts, portCfg, ticks_second)) {
        return 1;
    }

    /*************************************************************************
     * Configure the packet generator as follows
     * Total (5) Input streams
     * Stream No.(N) input to Port (0) Group (0) Queue (N) with 25Kpps of 64b
     * except queue 3 which has 25Kppps of 128b packets.  
     * The 125000pps violates the 100Kpps, but show the WRR still
     * does bytes
     *
     * 100Kpps will be allowed with 26665/26665/26665/6668/13336pps.
     *
     * The shares will be 100000/(4+4+4+1+2) or 6666.67/share
     * This is 13.627/13.627/13.627/6.881/6.881Mbps, which 
     * Configure the expected results
     *    Stream No.(0) output with (26667pps)
     *    Stream No.(1) output with (26667pps)
     *    Stream No.(2) output with (26667pps)
     *    Stream No.(3) output with ( 6667pps)
     *    Stream No.(4) output with (13333pps)
     *Tolerances for above each number that is checked. (2%)
     *************************************************************************/

    /* Make sure enough memory is in pktGen */
    pktGens = 5;
    if (pktGen->numQCfgs < pktGens) 
    {
        System_printf ("Build supports too few packet generators (%d, %d)\n", pktGen->numQCfgs, pktGens);
        return 1;
    }
    
    pktGen->numQCfgs = pktGens;
    pktGen->genTime  = 100000;
    pktGen->readTime = 100000;
    memset (pktGen->Qs, 0, pktGens * sizeof(pktGen->Qs[0]));

    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].port       = 0;
        pktGen->Qs[queue].group      = queue;
        pktGen->Qs[queue].queue      = 0;
        pktGen->Qs[queue].nSizes     = 1;
        pktGen->Qs[queue].sizeMode   = packet_GEN_SIZE_MODE_ROUND_ROBIN;
        pktGen->Qs[queue].pktSize[0] = 64;
        pktGen->Qs[queue].pps        = 30000;
    }
    pktGen->Qs[3].pktSize[0] = 128;
    pktGen->Qs[0].ppsExp = 26667;
    pktGen->Qs[1].ppsExp = 26667;
    pktGen->Qs[2].ppsExp = 26667;
    pktGen->Qs[3].ppsExp = 6667;
    pktGen->Qs[4].ppsExp = 13333;
    for (queue = 0; queue < pktGens; queue++)
    {
        pktGen->Qs[queue].ppsError   = pktGen->Qs[queue].ppsExp / (100/2); /* 2% */
    }

    return 0;
} /* scenario_config_simu_10 */
#endif

const qosSchedTestScenCfg qosSchedTestScenCfgs[] =
{
    scenario_config_01, "Check the scheduler of queue's WRR (with the same length of  bytes for 3 input streams)",
    scenario_config_02, "Check the scheduler of queue's WRR (with the different length of  bytes for 3 input streams)",
    scenario_config_03, "Check the scheduler of queue's SP",
    scenario_config_04, "Check the scheduler of queue's SP and WRR",
    scenario_config_05, "Check the scheduler of group's WRR",
    scenario_config_06, "Check the scheduler of group's WRR (test the second scheduler case, which still exist surplus streams after the first wrr scheduler)",
    scenario_config_07, "Check the scheduler of group's WRR ( one queue is configured with the minimum credit  ,and the other queue is configured with the maximal credit",
    scenario_config_08, "Check the shapping of QOS (basic shapping test case)",
    scenario_config_09, "Check the shapping of QOS",
#ifndef WIDE
    scenario_config_10, "Check the shapping of QOS (including bps and pps, and the pps is configued with kpps)",
    scenario_config_11, "Check the shapping of QOS (including bps and pps, and the precision for the pps)",
    scenario_config_12, "repeat previous test case, with removeBytes = 50",
    scenario_config_13, "8wrr joint port plus preceding and following port",
    scenario_config_14, "5wrr joint port plus preceding and following port",
    scenario_config_15, "4wrr joint port plus preceding and following port",
#else
    scenario_config_wide_01, "Test 1:2:..:16:17 WRR ratio on wide port",
#endif
    scenario_config_16, "25000 pps packet rate with sizes set to 0",
#ifdef TEST_SIMUBYTEPKT
    scenario_config_simu_01, "Simu BPS/PPS check w/choke PPS",
    scenario_config_simu_02, "Simu BPS/PPS check w/choke BPS",
    scenario_config_simu_03, "Simu BPS/PPS check w/choke both, BPS wins",
    scenario_config_simu_04, "Simu BPS/PPS check w/choke both, PPS wins",
    scenario_config_simu_05, "Simu BPS/PPS check WRR with choke BPS",
    scenario_config_simu_06, "Simu BPS/PPS check WRR with choke PPS",
    scenario_config_simu_07, "Simu BPS/PPS check WRR with choke PPS show WRR is bytes",
    scenario_config_simu_08, "Simu BPS/PPS check group WRR with choke BPS",
    scenario_config_simu_09, "Simu BPS/PPS check group WRR with choke PPS",
    scenario_config_simu_10, "Simu BPS/PPS check group WRR with choke PPS, show WRR is bytes",
#endif
    NULL, NULL
};


