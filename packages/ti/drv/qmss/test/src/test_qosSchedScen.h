/**
 *   @file  test_qosSchedScen.h
 *
 *   @brief   
 *      Structure definitions for QoS scheduler scenarios
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

#ifndef __TEST_QOS_SCHED_SCEN_H__
#define __TEST_QOS_SCHED_SCEN_H__

#include <stdint.h>

#define CORE_SPEED                  983000000

/* Configuration functions for qos and packet generator */
typedef enum
{
   packet_GEN_SIZE_MODE_ROUND_ROBIN, /* Generate packets with each size in pktSize[] in round robin fashion */
   packet_GEN_SIZE_MODE_RANDOM       /* Generate packets with each size uniform between pktSize[0] and pktSizse[1] */
} packetGenSizeMode;

typedef struct 
{
#define GENPORT_DROPSCHED 255
   uint32_t          port;     /* Associated input queue port */
   uint32_t          group;    /* Associated input queue group */
   uint32_t          queue;    /* Associated input queue in group */
   uint32_t          pps;      /* Generate packets per second */
   uint32_t          nSizes;   /* Number of sizes in pktSize */
   uint32_t          pktSize[16];  /* generated packet size (just value of Qm C register) */
   packetGenSizeMode sizeMode;
   uint32_t          ppsExp;   /* Expected result packets per second */
   uint32_t          ppsError; /* Allowed error */
} packetGenQCfg;

typedef struct 
{
   int           numQCfgs;
   uint32_t      genTime;  /* milliseconds to generate traffic */
   uint32_t      readTime; /* milliseconds from start of test to check results */
#define MAX_PKT_GEN_QUEUES 136
   packetGenQCfg Qs[MAX_PKT_GEN_QUEUES];
} packetGenCfg;


#ifdef TEST_DROP_SCHED
typedef struct
{
    Qmss_QosSchedDropSchedOutProf *outProfs;
    uint16_t                      *outQIdx; /* output queue index per outProf */
    Qmss_QosSchedDropSchedCfgProf *cfgProfs;
    Qmss_QosSchedDropSchedQueCfg  *queProfs;
    Qmss_QosSchedDropSchedCfg     *dropCfg;
    Qmss_QosSchedPortCfg          *qosSchedCfgs;
    int                           *qosSchedPortMap;
    uint16_t                      *qosSchedOutQIdx; /* output queue index per port */
    uint32_t                      *expNumPushStats; /* Expected push stats blocks */
    uint32_t                       fUsePushProxy;
} dropSchedScenCfg;

typedef int (*qosSchedTestConfigFcn_t) (uint32_t *, dropSchedScenCfg *cfgs, packetGenCfg *pktGen, int *nPorts);
#else
typedef struct
{
    Qmss_QosSchedPortCfg *portCfg;
    int                  *portMap;
} qosSchedScenCfg;
typedef int (*qosSchedTestConfigFcn_t) (uint32_t *, qosSchedScenCfg *cfgs, packetGenCfg *pktGen, int *nPorts);
#endif

typedef struct
{
    qosSchedTestConfigFcn_t  fcn;
    char                    *desc;
} qosSchedTestScenCfg;
extern const qosSchedTestScenCfg qosSchedTestScenCfgs[];
#endif

/* Prototype for common functions */
void scenario_rate_conversion_wrapper (int32_t *res, uint32_t ticks_second, uint32_t rate, Qmss_QosSchedAcctType type);
void scenario_wrr_conversion_wrapper (int32_t *res, uint32_t wrrCredit, Qmss_QosSchedAcctType type);
int scenario_config_common_joint (uint32_t *timer, Qmss_QosSchedPortCfg *protCfg, int *portMap, packetGenCfg *pktGen, int *nPorts, int nJointWrr);


