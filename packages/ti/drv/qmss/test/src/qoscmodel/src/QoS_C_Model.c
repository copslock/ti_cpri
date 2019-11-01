/**
 *   @file QoS_C_Model.c
 *
 *   @brief   
 *      This is the QMSS unit test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2015, Texas Instruments, Inc.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>
#include <ti/drv/qmss/qmss_qosSched.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/csl/csl_chip.h>

#include <ti/drv/qmss/qmss_qosSched.h>
#include <ti/drv/qmss/include/qmss_qos_sched_regs.h>
#include "QoS_C_Model.h"

#ifdef QOS_DROP_SCHEDULER
#define   MAX_PORTS       20
#else
#define   MAX_PORTS       12
#endif

//Global variables: These should all be initialized in main()
uint32_t      coreId;
uint32_t      queueBase;
uint32_t      tInterval;
uint32_t      PortCount;
uint8_t       TimerTicks;
uint64_t      gTimer;
Qmss_QosSchedRegs *qos_cmd_overlay;
PhysicalPortCfg  PortsCfg[MAX_PORTS];
PhysicalPortProc PortsProc[MAX_PORTS];

/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;
/* QM descriptor configuration */
Qmss_DescCfg            descCfg;

#pragma DATA_SECTION (local_overlay, ".qos_data")
Qmss_QosSchedRegs     local_overlay;

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;


static inline uint64_t read_timer64(void)
{
  uint32_t low  = TSCL;
  uint32_t high = TSCH;
  return _itoll (high, low);
}


int timerExpired(void)
{
  int retVal = 0;
  uint64_t tVal = read_timer64();

  if (tVal > gTimer)
  {
     retVal = 1;
     gTimer += tInterval;
  }

  return (retVal);
}


void clearTimer(void)
{
//  TSCH = 0;
//  TSCL = 0;
//  gTimer = 0;
}


//This function copies a port configuration from qos_cmd_overlay to Ports[port].
void copy_to_model(uint32_t port_num)
{
  uint32_t idx, idx2, tmp;
  PhysicalPortCfg *port;
  LogicalGroupCfg *group;
  QosQueueCfg     *que;
  uint32_t         isJoint = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
                                      QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_IS_JOINT);

  if (isJoint && (port_num & 1))
  {
    uint32_t copyQueues;

    port = &PortsCfg[port_num - 1];
    group = &port->Group[0];

    copyQueues = group->QueueCount;
    if (copyQueues > QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP)
    {
      copyQueues -= QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
    }
    else
    {
      copyQueues = 0;
    }

    // Joint and odd port: copy queues to previous (Lite) port
    for (idx2 = 0; idx2 < copyQueues; idx2++)
    {
      que = &group->Queue[idx2 + QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP];

      que->WrrInitialCredit = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[0].QUEUE[idx2].WRR_INITIAL_CREDIT;
      que->CongestionThresh = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[0].QUEUE[idx2].CONGESTION_THRESH;
    }
  } 

  //Copy the Port configuration:
  port = &PortsCfg[port_num];

  port->GroupCount = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
                              QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_NUM_GROUPS);
#ifndef QOS_DROP_SCHEDULER
  if (port_num < 2)
  {
    if (port->GroupCount > 5)
      port->GroupCount = 5;
  }
  else
  {
    if (port->GroupCount > 1)
      port->GroupCount = 1;
  }
#else
  if (port->GroupCount > 1)
    port->GroupCount = 1;
#endif

  port->fByteWrrCredits = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
                                   QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_WRR_IS_BYTES);
  port->fIsSupportByteShaping = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
                                         QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_BYTES);
  port->fIsSupportPacketShaping = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
                                          QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_PKTS);
  port->fByteCongest = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
                                QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CONG_THRESH_IS_BYTES);
  port->fByteDestThrottle = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
                                     QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_THROT_IS_BYTES);
  port->fIsJoint = isJoint;
  tmp = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
                 QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_QNUM_QMGR);
  port->DestQueueNumber.qMgr = tmp >> 12;
  port->DestQueueNumber.qNum = tmp  & 0xfff;

  port->RemoveBytes = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS_2,
                               QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_REMOVE_BYTES);
  port->OverheadBytes = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS_2,
                                 QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OVERHEAD_BYTES);
  port->DestThrottleThresh = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS_2,
                                      QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OUT_THROT_THRESH);

  port->CirIterationByByte = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.byteRate.CIR_ITERATION_CREDIT;
  port->CirMaxByByte = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.byteRate.CIR_MAX;
  port->CirIterationByPkt = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.packetRate.CIR_ITERATION_CREDIT;
  port->CirMaxByPkt = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.packetRate.CIR_MAX;

  //Copy the Group configurations:
  for (idx = 0; idx < port->GroupCount; idx++)
  {
    group = &port->Group[idx];

    group->WrrInitialCredit = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].wrrCfg.WRR_INITIAL_CREDIT;

    group->QueueCount = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS,
                                 QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_TOT);
    group->SPCount = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS,
                              QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_SP);
    group->RRCount = CSL_FEXT(qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS,
                              QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_WRR);

    group->fIsSupportByteShaping   = CSL_FEXT (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS, 
                                               QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_BYTE_SHAPING);
    group->fIsSupportPacketShaping = CSL_FEXT (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS, 
                                               QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_PKT_SHAPING);
    group->fIsInherited            = CSL_FEXT (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS, 
                                               QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_INHERITED);

    group->CirIterationByByte = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].byteRate.CIR_ITERATION;
    group->PirIterationByByte = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].byteRate.PIR_ITERATION;
    group->CirMaxByByte = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].byteRate.CIR_MAX;
    group->PirMaxByByte = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].byteRate.PIR_MAX;
    group->CirIterationByPkt = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].packetRate.CIR_ITERATION;
    group->PirIterationByPkt = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].packetRate.PIR_ITERATION;
    group->CirMaxByPkt = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].packetRate.CIR_MAX;
    group->PirMaxByPkt = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].packetRate.PIR_MAX;

    //Copy the Queue configurations:
    //(field .QueueNumber is set when the QoS Base Queue command arrives.)
    for (idx2 = 0; idx2 < group->QueueCount; idx2++)
    {
      que = &group->Queue[idx2];

      que->WrrInitialCredit = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].QUEUE[idx2].WRR_INITIAL_CREDIT;
      que->CongestionThresh = qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].QUEUE[idx2].CONGESTION_THRESH;
    }
  }
}


//This function copies a port configuration from Ports[port] to qos_cmd_overlay.
void copy_from_model(uint32_t port_num)
{
  uint32_t idx, idx2, tmp;
  PhysicalPortCfg *port;
  LogicalGroupCfg *group;
  QosQueueCfg     *que;

  //Copy the Port configuration:
  port = &PortsCfg[port_num];

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_NUM_GROUPS, port->GroupCount);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_WRR_IS_BYTES, port->fByteWrrCredits);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_BYTES, port->fIsSupportByteShaping);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_PKTS, port->fIsSupportPacketShaping);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CONG_THRESH_IS_BYTES, port->fByteCongest);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_THROT_IS_BYTES, port->fByteDestThrottle);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_IS_JOINT, port->fIsJoint);

  tmp  = port->DestQueueNumber.qMgr << 12;
  tmp |= port->DestQueueNumber.qNum & 0xfff;
  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_QNUM_QMGR, tmp);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS_2,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OVERHEAD_BYTES, port->OverheadBytes);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS_2,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_REMOVE_BYTES, port->RemoveBytes);

  CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.header.PORT_FLAGS_2,
            QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OUT_THROT_THRESH, port->DestThrottleThresh);

  qos_cmd_overlay->u.narrow.SHADOW_U.PORT.byteRate.CIR_ITERATION_CREDIT = port->CirIterationByByte;
  qos_cmd_overlay->u.narrow.SHADOW_U.PORT.byteRate.CIR_MAX = port->CirMaxByByte;
  qos_cmd_overlay->u.narrow.SHADOW_U.PORT.packetRate.CIR_ITERATION_CREDIT = port->CirIterationByPkt;
  qos_cmd_overlay->u.narrow.SHADOW_U.PORT.packetRate.CIR_MAX = port->CirMaxByPkt;

  //Copy the Group configurations:
  for (idx = 0; idx < port->GroupCount; idx++)
  {
    group = &port->Group[idx];

    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].wrrCfg.WRR_INITIAL_CREDIT = group->WrrInitialCredit;

    CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS,
              QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_TOT, group->QueueCount);

    CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS,
              QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_SP, group->SPCount);

    CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS,
              QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_WRR, group->RRCount);

    CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS, 
              QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_BYTE_SHAPING,
              group->fIsSupportByteShaping);
    CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS, 
              QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_PKT_SHAPING,
              group->fIsSupportPacketShaping);
    CSL_FINS (qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].FLAGS_QUEUE_COUNTS, 
              QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_INHERITED,
              group->fIsInherited);

    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].byteRate.CIR_ITERATION = group->CirIterationByByte;
    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].byteRate.PIR_ITERATION = group->PirIterationByByte;
    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].byteRate.CIR_MAX = group->CirMaxByByte;
    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].byteRate.PIR_MAX = group->PirMaxByByte;
    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].packetRate.CIR_ITERATION = group->CirIterationByPkt;
    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].packetRate.PIR_ITERATION = group->PirIterationByPkt;
    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].packetRate.CIR_MAX = group->CirMaxByPkt;
    qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].packetRate.PIR_MAX = group->PirMaxByPkt;

    //Copy the Queue configurations:
    //(field .QueueNumber is set when the SET_QUEUE_BASE command arrives.)
    for (idx2 = 0; idx2 < group->QueueCount; idx2++)
    {
      que = &group->Queue[idx2];

      qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].QUEUE[idx2].WRR_INITIAL_CREDIT = que->WrrInitialCredit;
      qos_cmd_overlay->u.narrow.SHADOW_U.PORT.GROUPS[idx].QUEUE[idx2].CONGESTION_THRESH  = que->CongestionThresh;
    }
  }
}


//This function sets all input queue numbers based on queueBase for the non-
//drop scheduler. It is called when the SET_QUEUE_BASE command arrives.
void set_queue_numbers(void)
{
  uint32_t port, grp, que;
  uint32_t curr_queue = 0;
  Qmss_Queue bQue;

  bQue.qMgr = queueBase >> 12;
  bQue.qNum = queueBase & 0xfff;

#ifndef QOS_DROP_SCHEDULER
  for (port = 0; port < 2; port++) //full ports (5 groups with 8 queues)
  {
    for (grp = 0; grp < 5; grp++)
    {
      for (que = 0; que < 8; que++)
      {
        PortsProc[port].Group[grp].Queue[que].QueueNumber.qMgr = bQue.qMgr;
        PortsProc[port].Group[grp].Queue[que].QueueNumber.qNum = bQue.qNum + curr_queue;
        curr_queue ++;
      }
    }
  }

  for (port = 2; port < MAX_PORTS; port++) //lite ports (1 group with 4 queues)
  {
    for (que = 0; que < 4; que++)
    {
      PortsProc[port].Group[0].Queue[que].QueueNumber.qMgr = bQue.qMgr;
      PortsProc[port].Group[0].Queue[que].QueueNumber.qNum = bQue.qNum + curr_queue;
      curr_queue ++;
    }
    /* Fill the other 4 queues for "joint" ports -- this overlaps following lite port */
    for (que = 0; que < 4; que++)
    {
      PortsProc[port].Group[0].Queue[4+que].QueueNumber.qMgr = bQue.qMgr;
      PortsProc[port].Group[0].Queue[4+que].QueueNumber.qNum = bQue.qNum + curr_queue + que;
      /* dont update curr queue, to allow intentional overlap with next port */
    }
  }
#else
  for (port = 0; port < MAX_PORTS; port++) //lite ports (1 group with 4 queues)
  {
    for (que = 0; que < 4; que++)
    {
      PortsProc[port].Group[0].Queue[que].QueueNumber.qMgr = bQue.qMgr;
      PortsProc[port].Group[0].Queue[que].QueueNumber.qNum = bQue.qNum + curr_queue;
      curr_queue ++;
    }
  }
#endif
}


uint32_t DropPacket(Qmss_Queue queueNumber)
{
  uint32_t  bytes;
  void   *desc;
  Qmss_Queue retQ;
  Qmss_QueueHnd qhnd;
  Cppi_DescType dTyp;

  qhnd = Qmss_getQueueHandle(queueNumber);
  Qmss_queuePopDescSize(qhnd, (void **)&desc, &bytes);
  desc = (void *)QMSS_DESC_PTR(desc);

  // get the return queue's handle from the descriptor header
  dTyp = Cppi_getDescType(desc);
  retQ = Cppi_getReturnQueue(dTyp, desc);
  qhnd = Qmss_getQueueHandle(retQ);

  if (desc != NULL)
    Qmss_queuePush(qhnd, desc, bytes, 32, Qmss_Location_TAIL);

  return(bytes);
}


uint32_t QueueByteLength(Qmss_Queue queueNumber)
{
  uint32_t bytes;
  Qmss_QueueHnd qhnd = Qmss_getQueueHandle(queueNumber);

  bytes = Qmss_getQueueByteCount(qhnd);

  return(bytes);
}


uint32_t QueuePacketCount(Qmss_Queue queueNumber)
{
  uint32_t pkts = 0;
  Qmss_QueueHnd qhnd = Qmss_getQueueHandle(queueNumber);

  pkts = Qmss_getQueueEntryCount(qhnd);

  return(pkts);
}


uint32_t ReadQosQueuePendingBits(LogicalGroupCfg *groupCfg, LogicalGroupProc *groupProc)
{
  uint32_t idx;
  uint32_t mask = 0;
  Qmss_QueueHnd qhnd;

  for (idx = 0; idx < groupCfg->QueueCount; idx++)
  {
    qhnd = Qmss_getQueueHandle(groupProc->Queue[idx].QueueNumber);
    mask |= Qmss_getQueueThresholdStatus(qhnd) << idx;
  }

  return(mask);
}


volatile uint32_t lookfor_id = 0xffffffff;
volatile uint32_t wait = 1;
int32_t TransferPacket(Qmss_Queue egressQueue, Qmss_Queue qosQueue)
{
  uint32_t  bytes;
  void   *desc;
  Qmss_QueueHnd qhnd = Qmss_getQueueHandle(qosQueue);
  Qmss_QueueHnd dhnd = Qmss_getQueueHandle(egressQueue);

  Qmss_queuePopDescSize(qhnd, (void **)&desc, &bytes);
  desc = (void *)QMSS_DESC_PTR(desc);

  if (desc != NULL)
  {
    if (*(((uint32_t *)desc)+1) == lookfor_id) {
      while (wait);
    }
    Qmss_queuePush(dhnd, desc, bytes, 32, Qmss_Location_TAIL);
    return(bytes);
  }
  return(-1);
}


void PortReset(uint32_t port_num, uint8_t enable)
{
  uint32_t grp, que, cnt, idx;
  Qmss_Queue q;
  PhysicalPortCfg  *portCfg  = &PortsCfg[port_num];
  PhysicalPortProc *portProc = &PortsProc[port_num]; 
  LogicalGroupProc *groupProc;
  LogicalGroupCfg  *groupCfg;

  portProc->fEnabled         = enable;
  portProc->NextGroup        = 0;
  portProc->CirCurrentByPkt  = 0;
  portProc->CirCurrentByByte = 0;
  portProc->WrrCreditMask    = 0;
  portProc->LastTimerTicks   = TimerTicks;

  for (grp = 0; grp < portCfg->GroupCount; grp++)
  {
    groupProc = &portProc->Group[grp];
    groupCfg  = &portCfg->Group[grp];
    groupProc->WrrCurrentCredit = 0;
    groupProc->WrrCreditMask = 0;
    groupProc->CirCurrentByByte = 0;
    groupProc->CirCurrentByPkt = 0;
    groupProc->PirCurrentByByte = 0;
    groupProc->PirCurrentByPkt = 0;
    groupProc->NextQueue = groupCfg->SPCount;

    if (enable == 0)
    {
      for (que = 0; que < groupCfg->QueueCount; que++)
      {
        groupProc->Queue[que].WrrCurrentCredit = 0;

        q = groupProc->Queue[que].QueueNumber;
        cnt = QueuePacketCount(q);

        for (idx = 0; idx < cnt; idx ++)
        {
          DropPacket(q);
        }
      }
    }
  }
}


void initialize_overlay(Qmss_QosSchedRegs *overlay)
{
  memset(overlay, 0, sizeof(Qmss_QosSchedRegs));

  overlay->VERSION = 0xf0f0f0f0;

#ifndef QOS_DROP_SCHEDULER
  CSL_FINS(overlay->MAGIC, QMSS_QOSSCHED_MAGIC_MAGIC, (uint32_t)QMSS_QOS_SCHED_MAGIC_MULTIGROUP);
#else
  CSL_FINS(overlay->MAGIC, QMSS_QOSSCHED_MAGIC_MAGIC, (uint32_t)QMSS_QOS_SCHED_MAGIC_DROPSCHED);
#endif
}

#if !defined(CSL_QMSS_CFG_PDSP1_REGS)
#define CSL_QMSS_CFG_PDSP1_REGS CSL_QM_SS_CFG_ADSP1_REGS 
#endif

#if !defined(CSL_QMSS_CFG_PDSP1_SRAM)
#define CSL_QMSS_CFG_PDSP1_SRAM CSL_QM_SS_CFG_SCRACH_RAM1_REGS 
#endif

void switch_cmd_buffers(uint32_t pdsp)
{
  uint32_t *ctl_reg = (uint32_t *)(CSL_QMSS_CFG_PDSP1_REGS + pdsp*0x100);

  // Reset the PDSP
  CSL_FINS(*ctl_reg, PDSP_PDSP_CONTROL_REG_PDSP_ENABLE, 0);

  // Set the overlay to the PDSP's Scratch RAM
  qos_cmd_overlay = (Qmss_QosSchedRegs *)(CSL_QMSS_CFG_PDSP1_SRAM + pdsp*0x4000);
  initialize_overlay(qos_cmd_overlay);
}


//This function makes the C model behave as if it is running on a PDSP. It
//examines Command Word 0 of the QoS Command Overlay that is placed at the
//top of LL2, not the normal one in the PDSP Scratch RAM.
void check_for_cmd(void)
{
  uint32_t cmd, opt, idx;
  uint32_t port, grp, que;
  uint32_t result = 0;

  cmd = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                  QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND);

  if (cmd != 0)
  {
    switch (cmd)
    {
      case QMSS_QOSSCHED_COMMAND_GET_QUEUE_BASE: //0x80
        CSL_FINS (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                  QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX, queueBase);
        result = 1;
        break;

      case QMSS_QOSSCHED_COMMAND_SET_QUEUE_BASE: //0x81
        queueBase = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                              QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX);
        set_queue_numbers();
        result = 1;
        break;

      case QMSS_QOSSCHED_COMMAND_TIMER_CONFIG: //0x82
        tInterval = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                              QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX);
        tInterval *= 6;
        result = 1;
        break;

      case QMSS_QOSSCHED_COMMAND_ENABLE_PORT: //0x90
        //idx: msb(yte) > 0 is for drop scheduler (not currently supported)
        //idx: lsb(yte) = port number 0..19
        idx = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                        QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX);

        //opt: 1=enable, 0=disable
        opt = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                        QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION);

        if ((idx & 0xff00) == 0)
        {
          idx &= 0x00ff;
          if (idx < MAX_PORTS)
          {
            if (opt)
              PortReset(idx, 1);
            else
              PortReset(idx, 0);
          }
          result = 1;
        }
        else
          result = 3;
        break;

      case QMSS_QOSSCHED_COMMAND_SHADOW_PORT: //0x91
        //idx: msb(yte) > 0 is for drop scheduler (not currently supported)
        //idx: lsb(yte) = port number 0..19
        idx = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                        QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX);

        //opt: 1=copy from cmd buff to model array, 0=copy from model array to cmd buff
        opt = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                        QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION);

        if ((idx & 0xff00) == 0)
        {
          idx &= 0x00ff;
          if (idx < MAX_PORTS)
          {
            if (opt)
              copy_to_model(idx);
            else
              copy_from_model(idx);
          }
          result = 1;
        }
        else
          result = 3;
        break;

      case QMSS_QOSSCHED_COMMAND_STATS_REQUEST: //0x92
	{
          Qmss_QosSchedStatsRegs *statsReg_p = &qos_cmd_overlay->u.narrow.STATS;
	  uint32_t startQ, endQ;
          //idx: bits 0..4 - port num
          //     bits 5-7  - group num
          //     bits 8-15 - queue within group
          idx = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                          QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX);
          port = (idx & 0x1f);
          grp  = (idx & 0xe0) >> 5;
          startQ = endQ = (idx & 0xff00) >> 8;

	  if (startQ == 0xff)
	  {
            startQ = 0;
	    endQ = PortsCfg[port].Group[grp].QueueCount - 1;
	    statsReg_p = qos_cmd_overlay->u.narrow.SHADOW_U.STATS.stats;
	    qos_cmd_overlay->u.narrow.SHADOW_U.STATS.flags = endQ + 1;
	  }
          //opt: 0x01: reset forwarded bytes stat
          //     0x02: reset forwarded pkts stat
          //     0x04: reset discarded bytes stat
          //     0x08: reset discarded pkts stat
          //     0x80: request drop sched stats instead of sched
          opt = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                          QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION);

	  for (que = startQ; que <= endQ; que++, statsReg_p++)
	  {
            statsReg_p->BYTES_FORWARDED_LSW = PortsProc[port].Group[grp].Queue[que].BytesForwarded & 0xffffffff;
            statsReg_p->BYTES_FORWARDED_MSW = PortsProc[port].Group[grp].Queue[que].BytesForwarded >> 32;
            statsReg_p->BYTES_DISCARDED_LSW = PortsProc[port].Group[grp].Queue[que].BytesDropped & 0xffffffff;
            statsReg_p->BYTES_DISCARDED_MSW = PortsProc[port].Group[grp].Queue[que].BytesDropped >> 32;
            statsReg_p->PACKETS_FORWARDED = PortsProc[port].Group[grp].Queue[que].PacketsForwarded;
            statsReg_p->PACKETS_DISCARDED = PortsProc[port].Group[grp].Queue[que].PacketsDropped;

            if (opt & 0x01)
              PortsProc[port].Group[grp].Queue[que].BytesForwarded = 0;
            if (opt & 0x02)
              PortsProc[port].Group[grp].Queue[que].PacketsForwarded = 0;
            if (opt & 0x04)
              PortsProc[port].Group[grp].Queue[que].BytesDropped = 0;
            if (opt & 0x08)
              PortsProc[port].Group[grp].Queue[que].PacketsDropped = 0;
	  }

          result = 1;
          break;
	}
      case 0xff: //Hijack a PDSP
        //opt: 0..7 for PDSP0..7
        opt = CSL_FEXT (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
                        QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION);
        switch_cmd_buffers(opt);
        break;

      default:
        printf("Illegal command code 0x%x encountered.\n", cmd);
        result = 2;
        break;
    }
  }

  if (result > 0)
  {
    //Set the return value
    CSL_FINS (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD1,
              QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE, result);

    //Clear the command byte
    CSL_FINS (qos_cmd_overlay->COMMAND_BUFFER.COMMAND_BUFFER_WORD0,
              QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, 0);
  }
}


//
// This is the main loop that drives everything. I'm not going to show initialization
// since it can be inferred from the comments on the structure definitions above.
//
void ForegroundTask(void)
{
    uint32_t i;

    // Process one control message
    check_for_cmd();

    // Schedule packets from all active physical ports
    for(i=0; i<PortCount; i++)
        if (PortsProc[i].fEnabled)
            PhysPortScheduler(&PortsCfg[i], &PortsProc[i]);
}

uint32_t getQueueLength (Qmss_Queue qNumber, uint8_t fBytes)
{
    uint32_t retVal;

    if (fBytes)
    {
        retVal = QueueByteLength(qNumber);
    }
    else
    {
        retVal = QueuePacketCount(qNumber);
    }
    return retVal;
}

//
// This is the background task that checks for queue congestion. Note that it is not
// run to completion, but constantly yields to the ForegroundTask() function.
//
void BackgroundTask(void)
{
    int32_t   ByteSize;
    uint32_t  i,j,k;

    // Do this forever
    while(1)
    {
        // Look at enabled Physical Ports
        for(i=0; i<PortCount; i++)
        {
            if (PortsProc[i].fEnabled)
            {
                // Look at all groups within a port
                for(j=0; j<PortsCfg[i].GroupCount; j++)
                {
                    // Look at all queues without a group
                    for( k=0; k<PortsCfg[i].Group[j].QueueCount; k++ )
                    {
                        if( PortsCfg[i].Group[j].Queue[k].CongestionThresh > 0 )
                        {
                            while( getQueueLength (PortsProc[i].Group[j].Queue[k].QueueNumber, PortsCfg[i].fByteCongest) >
                                                   PortsCfg[i].Group[j].Queue[k].CongestionThresh )
                            {
                                ByteSize = DropPacket( PortsProc[i].Group[j].Queue[k].QueueNumber);
                                PortsProc[i].Group[j].Queue[k].PacketsDropped += 1;
                                PortsProc[i].Group[j].Queue[k].BytesDropped += ByteSize;

                                if (timerExpired())  // this costs 1 PDSP cycle if timer didn't expire
                                {
                                    clearTimer();
                                    TimerTicks++;
                                    ForegroundTask();
                                }
                            }
                        }
                    }
                }
            }
        }

        if (timerExpired())  // this costs 1 PDSP cycle if timer didn't expire
        {
            clearTimer();
            TimerTicks++;
            ForegroundTask();
        }
    }
}


// Returns 0 if no space left, else 1
int32_t PhysPortUpdateOutputSpace (PhysicalPortCfg *pPort, int32_t *OutputSpaceAvail, int32_t BytesUsed)
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


int isCreditAvail (int32_t credit1, int32_t credit2, uint8_t flag1, uint8_t flag2)
{
    int creditAvail = 1;   /* flag1 || flag2 if do care above invalid config */
    if((credit1 <= 0) && (flag1)) {
       creditAvail = 0;
    }
    if((credit2 <= 0) && (flag2)) {
       creditAvail = 0;
    }
    // don't care if neither fIsSupportPacketShaping nor fIsSupportByteShaping is set; invalid config
    return creditAvail;
}


//
// This is the function that schedules packets on a physical port
//
void PhysPortScheduler(PhysicalPortCfg *pPortCfg, PhysicalPortProc *pPortProc)
{
    int32_t BytesUsed;              // Bytes used is returned from the Logical Scheduler
    int32_t WrrCreditUsed;          // Wrr Credit used (in packets or bytes as configured)
    uint8_t PacketPendingMask;      // Flag mask of RR groups that are not empty
    uint8_t PirCreditMask = 0;      // Flag set when more PIR credit remains
    int     fPacketsSent;
    int     i;
    int     OutputSpaceAvail = 0;


    /* Add credits for all TimerTicks that occurred since last time this port ran */
    while ((uint8_t)(TimerTicks - pPortProc->LastTimerTicks) > 0)
    {
        pPortProc->LastTimerTicks++;
        //
        // Add credits for all time based credit counters
        //

        // Credit for the main port
        if (pPortCfg->fIsSupportPacketShaping)
        {
            pPortProc->CirCurrentByPkt +=  pPortCfg->CirIterationByPkt;
            if( pPortProc->CirCurrentByPkt > pPortCfg->CirMaxByPkt )
                pPortProc->CirCurrentByPkt = pPortCfg->CirMaxByPkt;
        }
        if (pPortCfg->fIsSupportByteShaping)
        {
            pPortProc->CirCurrentByByte +=  pPortCfg->CirIterationByByte;
            if( pPortProc->CirCurrentByByte > pPortCfg->CirMaxByByte)
                pPortProc->CirCurrentByByte = pPortCfg->CirMaxByByte;
        }

        // Credit for the port's logical groups
        for( i=0; i<pPortCfg->GroupCount; i++ )
        {
            if (pPortCfg->Group[i].fIsSupportPacketShaping)
            {
                pPortProc->Group[i].CirCurrentByPkt +=  pPortCfg->Group[i].CirIterationByPkt;
                // Cap CIR credit at its max level
                if( pPortProc->Group[i].CirCurrentByPkt > pPortCfg->Group[i].CirMaxByPkt )
                    pPortProc->Group[i].CirCurrentByPkt = pPortCfg->Group[i].CirMaxByPkt;
                pPortProc->Group[i].PirCurrentByPkt +=  pPortCfg->Group[i].PirIterationByPkt;
                if( pPortProc->Group[i].PirCurrentByPkt> 0 )
                {
                    // Track every group with PIR credit for later
                    PirCreditMask |= (1<<i);
                    // Cap PIR credit at its max level
                    if( pPortProc->Group[i].PirCurrentByPkt > pPortCfg->Group[i].PirMaxByPkt )
                        pPortProc->Group[i].PirCurrentByPkt = pPortCfg->Group[i].PirMaxByPkt;
                }
            }
            
            if (pPortCfg->Group[i].fIsSupportByteShaping)
            {
                pPortProc->Group[i].CirCurrentByByte +=  pPortCfg->Group[i].CirIterationByByte;
                // Cap CIR credit at its max level
                if( pPortProc->Group[i].CirCurrentByByte > pPortCfg->Group[i].CirMaxByByte )
                    pPortProc->Group[i].CirCurrentByByte = pPortCfg->Group[i].CirMaxByByte;
                pPortProc->Group[i].PirCurrentByByte +=  pPortCfg->Group[i].PirIterationByByte;

                PirCreditMask &= ~(1<<i);

                if( pPortProc->Group[i].PirCurrentByByte > 0 )
                {
                    // Track every group with PIR credit for later
                    PirCreditMask |= (1<<i);
                    // Cap PIR credit at its max level
                    if( pPortProc->Group[i].PirCurrentByByte > pPortCfg->Group[i].PirMaxByByte )
                        pPortProc->Group[i].PirCurrentByByte = pPortCfg->Group[i].PirMaxByByte;
                }
            }
        }
    }

    /* Find out how much room is left in output queue */
    if (pPortCfg->DestThrottleThresh)
    {
        OutputSpaceAvail = pPortCfg->DestThrottleThresh;
        OutputSpaceAvail -= getQueueLength (pPortCfg->DestQueueNumber, pPortCfg->fByteDestThrottle);
        // No room in output queue */
        if (OutputSpaceAvail <= 0)
        {
            return;
        }
    }

    // Assume all groups have packets pending until we find out otherwise
    PacketPendingMask = 0x1F;

    //
    // Schedule each logic group's CIR, while also ensuring that the
    // physical port's CIR is not violated.
    // If the physical port has no credit quit out of the scheduler entirely
    if (!isCreditAvail (pPortProc->CirCurrentByPkt, pPortProc->CirCurrentByByte, 
                        pPortCfg->fIsSupportPacketShaping, pPortCfg->fIsSupportByteShaping))
        return;

    // Foreground task can exit once all packets are sent either because
    // the input queues are empty, or we ran out of group CIR, or we run
    // out of port CIR.
    do
    {
        fPacketsSent = 0;

        for( i=0; i<pPortCfg->GroupCount; i++ )
        {
            if (isCreditAvail (pPortProc->Group[i].CirCurrentByPkt, pPortProc->Group[i].CirCurrentByByte,
                               pPortCfg->Group[i].fIsSupportPacketShaping, pPortCfg->Group[i].fIsSupportByteShaping))
            {
                // Attempt to schedule a packet
                BytesUsed = LogicalGroupScheduler( pPortCfg, &pPortCfg->Group[i], &pPortProc->Group[i] );

                // If no packet scheduled, clear the pending mask
                if( !BytesUsed )
                {
                    PacketPendingMask &= ~(1<<i);
                }
                else
                {
                    uint32_t bytes = (uint32_t)BytesUsed & ~0x40000000;
                    uint32_t bytesAdjusted = (bytes + pPortCfg->OverheadBytes - pPortCfg->RemoveBytes) << QMSS_QOS_SCHED_BYTES_SCALE_SHIFT;
                    uint32_t packetsAdjusted = 1 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;

                    if (pPortCfg->fIsSupportByteShaping)
                    {
                        pPortProc->CirCurrentByByte          -= bytesAdjusted;
                    }

                    if (pPortCfg->Group[i].fIsSupportByteShaping)
                    {
                        pPortProc->Group[i].CirCurrentByByte -= bytesAdjusted;
                        pPortProc->Group[i].PirCurrentByByte -= bytesAdjusted;
                    }

                    if (pPortCfg->fIsSupportPacketShaping)
                    {
                        pPortProc->CirCurrentByPkt          -= packetsAdjusted;
                    }

                    if (pPortCfg->Group[i].fIsSupportPacketShaping)
                    {
                        pPortProc->Group[i].CirCurrentByPkt -= packetsAdjusted;
                        pPortProc->Group[i].PirCurrentByPkt -= packetsAdjusted;
                    }

                    fPacketsSent = 1;

                    // If the physical port has no credit quit out of the scheduler entirely
                    if (!isCreditAvail (pPortProc->CirCurrentByPkt, pPortProc->CirCurrentByByte,
                                        pPortCfg->fIsSupportPacketShaping, pPortCfg->fIsSupportByteShaping))
                        return;

                    // See if we used up output space
                    if (PhysPortUpdateOutputSpace (pPortCfg, &OutputSpaceAvail, bytes) == 0)
                    {
                        return;
                    }
                }
            }
        }
    } while (fPacketsSent);
    //
    // Schedule each logic group's PIR in a WRR fashion while the
    // physical port's CIR is not violated.
    //

    do
    {
        // If there are no groups left with PIR group credit and packets, then we're done
        if( !(PirCreditMask & PacketPendingMask) )
            return;

        // If all groups with WRR credit remaining are empty, add WRR credit
        while( ! (PirCreditMask & pPortProc->WrrCreditMask & PacketPendingMask))
        {
            // Reset credits
            for(i=0; i<pPortCfg->GroupCount; i++)
            {
                pPortProc->Group[i].WrrCurrentCredit += pPortCfg->Group[i].WrrInitialCredit;

                if (pPortProc->Group[i].WrrCurrentCredit > (pPortCfg->Group[i].WrrInitialCredit << 1))
                    pPortProc->Group[i].WrrCurrentCredit = (pPortCfg->Group[i].WrrInitialCredit << 1);
                      
                if (pPortProc->Group[i].WrrCurrentCredit > 0 || (! pPortCfg->Group[i].WrrInitialCredit))
                    pPortProc->WrrCreditMask |= (1<<i);
            }
            // while loop will always terminate because PirCreditMask & PacketPendingMask check 
        }

        // If this group has PIR credit, WRR credit, and packets pending, then schedule a packet
        if( (PirCreditMask & pPortProc->WrrCreditMask & PacketPendingMask) & (1<<pPortProc->NextGroup) )
        {
            // Attempt to schedule a packet
            BytesUsed = LogicalGroupScheduler( pPortCfg, &pPortCfg->Group[pPortProc->NextGroup], &pPortProc->Group[pPortProc->NextGroup]);

            // If no packet scheduled, clear the pending mask
            if( !BytesUsed )
                PacketPendingMask &= ~(1<<pPortProc->NextGroup);
            else
            {
                uint32_t bytes = (uint32_t)BytesUsed & ~0x40000000;
                uint32_t bytesAdjusted = (bytes + pPortCfg->OverheadBytes - pPortCfg->RemoveBytes);
                uint32_t packetsAdjusted = 1 << QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;

                // Use packet or byte count, depending on configuration
                if( pPortCfg->fByteWrrCredits )
                    WrrCreditUsed = bytesAdjusted << QMSS_QOS_WRR_BYTES_SCALE_SHIFT;
                else
                    WrrCreditUsed = 1 << QMSS_QOS_WRR_PACKETS_SCALE_SHIFT;

                // We also deduct the WRR credit
                pPortProc->Group[pPortProc->NextGroup].WrrCurrentCredit -= WrrCreditUsed;
                bytesAdjusted <<= QMSS_QOS_SCHED_BYTES_SCALE_SHIFT;

                // Deduct the PIR/CIR credit
                if (pPortCfg->fIsSupportPacketShaping)
                {
                    pPortProc->CirCurrentByPkt -= packetsAdjusted;
                }

                if (pPortCfg->Group[pPortProc->NextGroup].fIsSupportPacketShaping)
                {
                    pPortProc->Group[pPortProc->NextGroup].PirCurrentByPkt -= packetsAdjusted;
                }

                // Deduct the PIR/CIR credit
                if (pPortCfg->fIsSupportByteShaping)
                {
                    pPortProc->CirCurrentByByte-= bytesAdjusted;
                }

                if (pPortCfg->Group[pPortProc->NextGroup].fIsSupportByteShaping)
                {
                    pPortProc->Group[pPortProc->NextGroup].PirCurrentByByte -= bytesAdjusted;
                }
                
                // Clear the group's PIR credit mask if we depleted the PIR credit
                if (!isCreditAvail (pPortProc->Group[pPortProc->NextGroup].PirCurrentByPkt, pPortProc->Group[pPortProc->NextGroup].PirCurrentByByte,
                                    pPortCfg->Group[pPortProc->NextGroup].fIsSupportPacketShaping, pPortCfg->Group[pPortProc->NextGroup].fIsSupportByteShaping))
                    PirCreditMask &= ~(1<<pPortProc->NextGroup);

                // Clear the group's WRR credit mask if we depleted the WRR credit
                if( pPortProc->Group[pPortProc->NextGroup].WrrCurrentCredit <= 0 )
                    pPortProc->WrrCreditMask &= ~(1<<pPortProc->NextGroup);

                // See if we used up output space
                if (PhysPortUpdateOutputSpace (pPortCfg, &OutputSpaceAvail, bytes) == 0)
                {
                    return;
                }
            }
        }

        // Move on to the next group
        pPortProc->NextGroup++;
        if( pPortProc->NextGroup == pPortCfg->GroupCount )
            pPortProc->NextGroup = 0;

    } while (isCreditAvail (pPortProc->CirCurrentByPkt, pPortProc->CirCurrentByByte,
                            pPortCfg->fIsSupportPacketShaping, pPortCfg->fIsSupportByteShaping));
}


//
// This is the function that schedules a single packet from queues on a logical group
// The function returns the packet size of the packet selected
//
int32_t LogicalGroupScheduler(PhysicalPortCfg *pPortCfg, LogicalGroupCfg *pGroupCfg, LogicalGroupProc *pGroupProc)
{
    int32_t   BytesUsed;
    int32_t   packetSent = 0;
    uint8_t   PacketPendingMask;
    int     i, j;

#if 0
    if (timerExpired())  // this costs 1 PDSP cycle if timer didn't expire
    {
        clearTimer();
        TimerTicks++;
    }
#endif
    // With queues, we can directly read the pending status
    PacketPendingMask = ReadQosQueuePendingBits(pGroupCfg, pGroupProc);

    // If no packets, nothing to do
    if(!PacketPendingMask)
        return 0;

    //
    // Try to take a high priority queue first
    //
    for( i=0; i<pGroupCfg->SPCount; i++ )
    {
        if( PacketPendingMask & (1<<i) )
            return( QosQueueScheduler(pPortCfg, &pGroupProc->Queue[i]));
    }

    //
    // Next try to pick a round robin queue
    //
    if (PacketPendingMask & (((1 << pGroupCfg->RRCount) - 1) << pGroupCfg->SPCount))
    {
        // There are RR packets pending
        for( i=0; i<pGroupCfg->RRCount; i++ )
        {
            // If all queues with WRR credit remaining are empty, reset the credit
            while ( !(pGroupProc->WrrCreditMask & PacketPendingMask) )
            {
                // Reset credits
                for(j=pGroupCfg->SPCount; j<(pGroupCfg->SPCount+pGroupCfg->RRCount); j++)
                {
                    pGroupProc->Queue[j].WrrCurrentCredit += pGroupCfg->Queue[j].WrrInitialCredit;

                    if (pGroupProc->Queue[j].WrrCurrentCredit > (pGroupCfg->Queue[j].WrrInitialCredit << 1))
                        pGroupProc->Queue[j].WrrCurrentCredit = (pGroupCfg->Queue[j].WrrInitialCredit << 1);
                      
                    if (pGroupProc->Queue[j].WrrCurrentCredit > 0 || (! pGroupCfg->Queue[j].WrrInitialCredit))
                        pGroupProc->WrrCreditMask |= (1<<j);
                }

                // While loop must terminate given 
                // (PacketPendingMask & (((1 << pGroup->RRCount) - 1) << pGroup->SPCount)
            }

            // If the next queue has WRR credit and packets, then schedule a packet
            if( (pGroupProc->WrrCreditMask & PacketPendingMask) & (1<<pGroupProc->NextQueue) )
            {
                // Attempt to schedule a packet
                BytesUsed = QosQueueScheduler( pPortCfg, &pGroupProc->Queue[pGroupProc->NextQueue] );

                // If 0x40000000, will 'fall off" in shift below
                // Deduct the WRR credit
                if( pPortCfg->fByteWrrCredits )
                    pGroupProc->Queue[pGroupProc->NextQueue].WrrCurrentCredit -= (BytesUsed + pPortCfg->OverheadBytes - pPortCfg->RemoveBytes) << QMSS_QOS_WRR_BYTES_SCALE_SHIFT;
                else
                    pGroupProc->Queue[pGroupProc->NextQueue].WrrCurrentCredit -= 1 << QMSS_QOS_WRR_PACKETS_SCALE_SHIFT;

                // Clear the queues's WWR credit mask if we depleted the WRR credit
                if( pGroupProc->Queue[pGroupProc->NextQueue].WrrCurrentCredit <= 0 )
                    pGroupProc->WrrCreditMask &= ~(1<<pGroupProc->NextQueue);

                packetSent = 1;
            }

            // Move on to the next group
            pGroupProc->NextQueue++;
            if( pGroupProc->NextQueue == pGroupCfg->SPCount+pGroupCfg->RRCount )
                pGroupProc->NextQueue = pGroupCfg->SPCount;

            // Quit now if we moved a packet
            if(packetSent)
                return(BytesUsed);
        }
    }

    //
    // Finally, try to get a packet from the OPTIONAL best effort queues
    //
    for( i=pGroupCfg->SPCount+pGroupCfg->RRCount; i<pGroupCfg->QueueCount; i++ )
    {
        if( PacketPendingMask & (1<<i) )
            return( QosQueueScheduler(pPortCfg, &pGroupProc->Queue[i] ));
    }

    // No packet was transferred
    return(0);
}


//
// This is the function that moves a packet from the QOS queue to the egress
//
int32_t QosQueueScheduler(PhysicalPortCfg *pPortCfg, QosQueueProc *pQueueProc)
{
    int32_t   ByteSize;

    ByteSize = TransferPacket( pPortCfg->DestQueueNumber, pQueueProc->QueueNumber );

    if (ByteSize != -1)
    {
      pQueueProc->PacketsForwarded += 1;
      pQueueProc->BytesForwarded   += ByteSize;
      return(ByteSize | 0x40000000);
    }
    return(0);
}


void main(void)
{
  Qmss_Result qmresult;

  printf("sizes: %d %d %d\n", 
         sizeof(Qmss_QosSchedNarrowRegs), 
	 sizeof(Qmss_QosSchedWideRegs),
         sizeof(Qmss_QosSchedDropPlusNarrowRegs));
  
  coreId = CSL_chipReadReg (CSL_CHIP_DNUM);

  //Because these memsets will clear the model's structs and the the pseudo cmd
  //buffer, this means that this program (which should run on DSP 1) should be
  //started BEFORE any program(s) that will send commands to it.

  qos_cmd_overlay = &local_overlay;
  memset(PortsCfg, 0, sizeof(PortsCfg));
  memset(PortsProc, 0, sizeof(PortsProc));
  initialize_overlay(&local_overlay);

  memset((void *) &qmssInitConfig, 0, sizeof(Qmss_InitCfg));

  /* Set up the linking RAM. Use internal Linking RAM.  */
  qmssInitConfig.linkingRAM0Base = 0;
  qmssInitConfig.linkingRAM0Size = 0;
  qmssInitConfig.linkingRAM1Base = 0x0;
  qmssInitConfig.maxDescNum      = 0x3fff;

  qmresult = Qmss_init(&qmssInitConfig, &qmssGblCfgParams);
  if (qmresult != QMSS_SOK)
    printf("Error: Could not initialize QMSS on core %d (%d).\n", coreId, qmresult);

  qmresult = Qmss_start();
  if (qmresult != QMSS_SOK)
    printf("Error: Could not start QMSS on core %d (%d).\n", coreId, qmresult);

  if (qmresult == QMSS_SOK)
  {
    // timer message can change this, put message value *6 to account for pdsp
    // clock properties.
    tInterval = ((350000000/100000)/2) * 6;
    TSCL = 0;
    gTimer = 0;
    queueBase = 0;
    TimerTicks = 0;
    PortCount = MAX_PORTS;

    // run forever:
    BackgroundTask();
  }
}
