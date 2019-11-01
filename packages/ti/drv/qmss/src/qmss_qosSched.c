/**
 *   @file  qmss_qosSched.c
 *
 *   @brief   
 *      This file has API that configure the Queue Manager QoS packet 
 *      scheduler firmware.
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

/* QMSS Types includes */
#include <stdint.h>
#include <stdlib.h>

/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_qosSched.h>
#include <ti/drv/qmss/include/qmss_pvt.h>
#include <ti/drv/qmss/include/qmss_qos_sched_regs.h>

/* QMSS OSAL layer */
#include <qmss_osal.h>

/* Standard includes */
#include <string.h>

/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/**********************************************************************
 ************************** Functions *********************************
 **********************************************************************/

/* Private function to get register pointer */
static Qmss_QosSchedRegs *qmss_qos_sched_get_regptr (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId)
{
    uint32_t subSys = (uint32_t)subSysHnd;
    Qmss_QosSchedRegs *retVal;

    if (pdspId >= Qmss_PdspId_CMODEL_DSP1)
    {
        /* Use C model on C66x #1 */
        retVal = (Qmss_QosSchedRegs *)
            (0x10800000 | ((pdspId - Qmss_PdspId_CMODEL_DSP1 + 1) << 24));
    } 
    else if ((pdspId >= QMSS_MAX_PDSP) || (subSys >= QMSS_MAX_SUBSYS))
    {
        retVal = NULL;
    } 
    else
    {
        retVal = (Qmss_QosSchedRegs *) qmssLObj[subSys].p.regs.qmPdspCmdReg[pdspId];
    }
    return retVal;
} /* qmss_qos_sched_get_regptr */

static uint32_t qmss_qos_sched_get_magic (Qmss_QosSchedRegs *regPtr)
{
    if (regPtr)
    {
        return regPtr->MAGIC;
    }

    return 0xffffffff;
} /* qmss_qos_sched_get_magic */

/* Private function to create hw queue ID for QoS Firmware */
static int qmss_qos_sched_act_types_is_bytes (
    uint32_t *isBytes, 
    uint32_t *isPackets, 
    Qmss_QosSchedAcctType type,
    Qmss_QosSchedAcctType prevType,
    int       isSimu
)
{
    if (type == Qmss_QosSchedAcctType_INHERITED)
    {
        type = prevType;
    }
    switch (type)
    {
        case Qmss_QosSchedAcctType_INHERITED:
            /* Treat inherit like packets for bw compatibility */
        case Qmss_QosSchedAcctType_PACKETS:
            *isBytes = 0;
            if (isPackets)
            {
                *isPackets = 1;
            }
            break;
        case Qmss_QosSchedAcctType_BYTES:
            *isBytes = 1;
            if (isPackets)
            {
                *isPackets = 0;
            }
            break;
        case Qmss_QosSchedAcctType_BOTH:
            if (isSimu && isPackets)
            {
                *isBytes = 1;
                *isPackets = 1;
            }
            else
            {
                return 1; /* fail */
            }
            break;
        default:
            return 1; /* fail */
    }

    return 0;
} /* qmss_qos_sched_act_types_is_bytes */

/* Private function to create enum from bytes flag */
static Qmss_QosSchedAcctType qmss_qos_sched_is_bytes_to_act_types(uint32_t isBytes, uint32_t isPackets, int isSimu)
{
    Qmss_QosSchedAcctType retVal;
    if (isSimu)
    {
        if (isBytes) 
        {
           if (isPackets) 
           {
               retVal = Qmss_QosSchedAcctType_BOTH;
           }
           else
           {
               retVal = Qmss_QosSchedAcctType_BYTES;
           }
        }
        else
        {
           retVal = Qmss_QosSchedAcctType_PACKETS;
        }
    }
    else
    {
        retVal = isBytes ? Qmss_QosSchedAcctType_BYTES 
                   : Qmss_QosSchedAcctType_PACKETS;
    }

    return retVal;
} /* qmss_qos_sched_is_bytes_to_act_types */

/* Private function to create valid flag from enum */
static int qmss_qos_sched_prof_valid_is_valid (uint32_t *isValid, Qmss_QosSchedDropSchedProfValid valid)
{
    switch (valid)
    {
        case Qmss_QosSchedDropSchedProf_INVALID:
            *isValid = 0;
            break;
        case Qmss_QosSchedDropSchedProf_VALID:
            *isValid = 1;
            break;
        default:
            return 1;
    }

    return 0;
} /* qmss_qos_sched_prof_valid_is_valid */

/* Private function to create enum from valid flag */
static Qmss_QosSchedDropSchedProfValid qmss_qos_sched_is_valid_to_prof_valid (uint32_t isValid)
{
    return isValid ? Qmss_QosSchedDropSchedProf_VALID
                   : Qmss_QosSchedDropSchedProf_INVALID;
} /* qmss_qos_sched_is_valid_to_prof_valid */

/* private function to convert queue number to bitfield */
static inline uint32_t qmss_convert_qmss_queue_to_bitfield (Qmss_Queue queue)
{
    return Qmss_getQIDFromHandle(Qmss_getQueueHandle (queue));
} /* qmss_convert_qmss_queue_to_bitfield */

/* private function to convert a bitfield into queue number */
static inline Qmss_Queue qmss_convert_bitfield_to_qmss_queue (Qmss_SubSysHnd subSysHnd, uint32_t bitfield)
{
    /* This is inverse of qmss_convert_qmss_queue_to_bitfield */
    return Qmss_getQueueNumber (Qmss_getHandleFromQIDSubSys(subSysHnd, bitfield));
} /* qmss_convert_bitfield_to_qmss_queue */

/* private function to convert drop sched mode bitfield to enum */
static inline Qmss_QosSchedDropSchedMode qmss_drop_sched_mode_to_enum (uint32_t bitfield)
{
    return (bitfield == QMSS_QOSSCHED_DROPSCHED_MODE_REDDROP) ? 
                Qmss_QosSchedDropSchedMode_RED : 
                Qmss_QosSchedDropSchedMode_TAILDROP;
} /* qmss_drop_sched_mode_to_enum */

/* private function to convert drop sched enum to mode bitfield */
static inline int qmss_drop_sched_enum_to_mode (uint32_t *bitfield, Qmss_QosSchedDropSchedMode mode)
{
    switch (mode)
    {
        case Qmss_QosSchedDropSchedMode_TAILDROP:
            *bitfield = QMSS_QOSSCHED_DROPSCHED_MODE_TAILDROP;
            break;
        case Qmss_QosSchedDropSchedMode_RED:
            *bitfield = QMSS_QOSSCHED_DROPSCHED_MODE_REDDROP;
            break;
        case Qmss_QosSchedDropSchedMode_REM:
            /* Not supported */
        default:
            return 1;
    }

    return 0;
} /* qmss_drop_sched_enum_to_mode */

/* Private function to wait until command buf is ready */
static void qmss_wait_qos_sched_command_buf(Qmss_QosSchedRegs *regPtr) 
{
    uint32_t           result;

    do
    {
        result = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0, 
                           QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND);  
    } while (result != 0);
} /* qmss_wait_qos_sched_command_buf */

/* Private function to determine number of ports */
static uint32_t qmss_qos_sched_get_max_ports (uint32_t magic)
{
    uint32_t maxPorts;

    if (magic == QMSS_QOS_SCHED_MAGIC_MULTIGROUP)
    {
        maxPorts = QMSS_QOS_SCHED_MAX_PHYS_PORTS;
    }
    else if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
    {
        maxPorts = QMSS_QOS_SCHED_WIDE_MAX_PHYS_PORTS;
    } 
    else
    {
        maxPorts = QMSS_QOS_SCHED_DROP_SCHED_LITE_MAX_PHYS_PORTS;
    }

    return maxPorts;
} /* qmss_qos_sched_get_max_ports */

/* Private function to error check port, group, and queue */
static Qmss_Result qmss_qos_sched_check_args (int zeroIsBad, uint32_t port, uint32_t group, 
                                              uint32_t queue, Qmss_QosSchedRegs *regPtr, uint32_t isJoint)
{
    uint32_t maxPorts, maxGroups, maxQueues;
    uint32_t magic;

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);

    maxPorts = qmss_qos_sched_get_max_ports (magic);

    if (magic == QMSS_QOS_SCHED_MAGIC_MULTIGROUP)
    {
        if (port >= QMSS_QOS_SCHED_FULL_MAX_PHYS_PORTS)
        {
            maxGroups = QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS;
            maxQueues = QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
            if (isJoint)
            {
                maxQueues *= 2;
            }
        }
        else
        {
            maxGroups = QMSS_QOS_SCHED_FULL_MAX_LOG_GROUPS;
            maxQueues = QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP;
            if (zeroIsBad && isJoint)
            {
                /* not an error for stats check -- zeroIsBad == 0 */
                return QMSS_QOS_SCHED_ISJOINT_LITE_ONLY;
            }
        }
    }
    else if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
    {
        maxQueues = QMSS_QOS_SCHED_WIDE_MAX_QUEUES_PER_GROUP;
        maxGroups = QMSS_QOS_SCHED_WIDE_MAX_LOG_GROUPS;
    }
    else if (magic == QMSS_QOS_SCHED_MAGIC_DROPSCHED)
    {
        maxQueues = QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
        maxGroups = QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS;
        if (isJoint)
        {
            maxQueues *= 2;
        }
    }
    else
    {
        return QMSS_QOS_SCHED_INVALID_MAGIC;
    }

    if (port >= maxPorts)
    {
        return QMSS_QOS_SCHED_INVALID_PORT;
    }

    if (zeroIsBad)  /* one-based counting */
    {
        if (group == 0)
        {
            return QMSS_QOS_SCHED_INVALID_GROUP;
        }
        if (queue == 0)
        {
            return QMSS_QOS_SCHED_INVALID_QUEUE;
        }
        maxGroups++;
        maxQueues++;
    }

    if (group >= maxGroups) 
    {
        return QMSS_QOS_SCHED_INVALID_GROUP;
    }

    if (queue >= maxQueues) 
    {
        return QMSS_QOS_SCHED_INVALID_QUEUE;
    }

    return QMSS_QOS_SCHED_RETCODE_SUCCESS;
} /* qmss_qos_sched_check_args */

/*
 * Creates a set_queue_base message for either QoS scheduler or
 * drop scheduler.
 */
static Qmss_Result qmss_internal_set_qos_sched_queue_base (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueNum, uint32_t type)
{
    uint32_t           cmdBuf = 0;
    Qmss_Result        retVal;
    void              *key;
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    key    = Qmss_osalCsEnter();

    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, 
              QMSS_QOSSCHED_COMMAND_SET_QUEUE_BASE);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION,
              type);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX, 
              queueNum);
    regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0 = cmdBuf;

    /* Wait till command is executed */
    qmss_wait_qos_sched_command_buf (regPtr);

    retVal = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD1, 
                       QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE);

    Qmss_osalCsExit(key);

    return retVal;
} /* qmss_internal_set_qos_sched_queue_base */

/*
 * Creates a get_queue_base message for either QoS scheduler or
 * drop scheduler.
 */
static Qmss_Result qmss_internal_get_qos_sched_queue_base (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t *queueNum, uint32_t type)
{
    uint32_t           cmdBuf = 0;
    Qmss_Result        retVal;
    void              *key;
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    key = Qmss_osalCsEnter();

    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, 
              QMSS_QOSSCHED_COMMAND_GET_QUEUE_BASE);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION,
              type);
    regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0 = cmdBuf;

    /* Wait till command is executed */
    qmss_wait_qos_sched_command_buf (regPtr);

    *queueNum = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0, 
                          QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX);

    retVal = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD1, 
                       QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE);

    Qmss_osalCsExit(key);

    return retVal;
} /* qmss_internal_get_qos_sched_queue_base */

/* private function to enable/disable drop scheduler */
static Qmss_Result qmss_internal_enable_drop_sched (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, uint32_t option)
{   
    uint32_t           cmdBuf = 0;
    Qmss_Result        retVal;
    void              *key;
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    uint32_t           magic;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);
        
    if (magic != QMSS_QOS_SCHED_MAGIC_DROPSCHED)
    {
        return QMSS_QOS_SCHED_REQ_DROP_SCHED;
    }

    if (port != 0)
    {
        return QMSS_QOS_SCHED_INVALID_PORT;
    }

    key = Qmss_osalCsEnter();
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, QMSS_QOSSCHED_COMMAND_ENABLE_PORT);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION, option);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX, 0x100 + port);
    regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0 = cmdBuf;

    /* Wait till command is executed */
    qmss_wait_qos_sched_command_buf (regPtr);

    retVal = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD1, 
                       QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE);

    Qmss_osalCsExit(key);

    return retVal;
} /* qmss_internal_enable_drop_sched */

/* Private function to error check cir or pir values */
static int qmss_qos_sched_check_rate (int32_t iterCredit, int32_t maxCredit)
{
    if ((0x7fffffff - maxCredit) < iterCredit)
    {
        /* cfg->cirMax + cfg->cirIteration is an overflow */
        return QMSS_QOS_SCHED_INVALID_RATEMAX;
    }

    if (maxCredit < 0)
    {
        return QMSS_QOS_SCHED_INVALID_RATEMAX;
    }

    if (iterCredit < 0)
    {
        return QMSS_QOS_SCHED_INVALID_RATEITER;
    }

    return -1; /* OK */
}

/** @addtogroup QMSS_LLD_FUNCTION
@{ 
*/

/**
 *  @b Description
 *  @n  
 *      This function is used to set the QoS ingress base queue number. The QoS ingress queue 
 *      are restricted to a set of 80 or 120 starting at a fixed base which must be a multiple of 32.
 *
 *      If the drop scheduler is present, then there are 80 queues arranged 
 *      into 20 ports each of which has 1 group of 4 queues.
 *
 *      Otherwise, 120 queues are mapped into ports and groups.  Assuming there are 8 queues
 *      per group and 5 groups per port the assignment is:
 *
 *      <TABLE>
 *      <TR><TH>Port</TH><TH>Group</TH><TH>Assigned Queues</TH></TR>
 *      <TR><TD>0</TD><TD>0</TD><TD>base + 0 - 7</TD></TR>
 *      <TR><TD>0</TD><TD>1</TD><TD>base + 8 -15</TD></TR>
 *      <TR><TD>0</TD><TD>2</TD><TD>base + 16-23</TD></TR>
 *      <TR><TD>0</TD><TD>3</TD><TD>base + 24-31</TD></TR>
 *      <TR><TD>0</TD><TD>4</TD><TD>base + 32-39</TD></TR>
 *      <TR><TD>1</TD><TD>0</TD><TD>base + 40-47</TD></TR>
 *      <TR><TD>1</TD><TD>1</TD><TD>base + 48-55</TD></TR>
 *      <TR><TD>1</TD><TD>2</TD><TD>base + 56-63</TD></TR>
 *      <TR><TD>1</TD><TD>3</TD><TD>base + 64-71</TD></TR>
 *      <TR><TD>1</TD><TD>4</TD><TD>base + 72-79</TD></TR>
 *      <TR><TD>2</TD><TD>0</TD><TD>base + 80-83</TD></TR>
 *      <TR><TD>3</TD><TD>0</TD><TD>base + 84-87</TD></TR>
 *      <TR><TD>4</TD><TD>0</TD><TD>base + 88-91</TD></TR>
 *      <TR><TD>5</TD><TD>0</TD><TD>base + 92-95</TD></TR>
 *      <TR><TD>6</TD><TD>0</TD><TD>base + 96-99</TD></TR>
 *      <TR><TD>7</TD><TD>0</TD><TD>base + 100-103</TD></TR>
 *      <TR><TD>8</TD><TD>0</TD><TD>base + 104-107</TD></TR>
 *      <TR><TD>9</TD><TD>0</TD><TD>base + 108-111</TD></TR>
 *      <TR><TD>10</TD><TD>0</TD><TD>base + 112-115</TD></TR>
 *      <TR><TD>11</TD><TD>0</TD><TD>base + 116-119</TD></TR>
 *      </TABLE>
 *
 *      Each device has a block of queues reserved for this purpose.  It is defined in CSL
 *      as QMSS_TRAFFIC_SHAPING_QUEUE_BASE and QMSS_MAX_TRAFFIC_SHAPING_QUEUE.  
 *
 *      On some devices, the QMSS_MAX_TRAFFIC_SHAPING_QUEUE may be limited to something less
 *      than 80.  If more queues are needed, then the traffic shaping/QoS subsystem allows 
 *      any range of general purpose queues used.  In other words, 
 *      QMSS_TRAFFIC_SHAPING_QUEUE_BASE is just a suggestion.
 *
 *      If general queues are used, they should be reserved through @ref Qmss_queueOpenSubSys,
 *      similar to example code in test_descAlloc.c inside QOS_TEST.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  queueNum
 *      Base queue number 
 *
 *  @retval
 *      Success - QCMD_RETCODE_SUCCESS
 *  @retval
 *      Failure - Command buffer return status - QMSS_QOS_SCHED_INVALID_COMMAND, QMSS_QOS_SCHED_INVALID_INDEX, QMSS_QOS_SCHED_INVALID_OPTION
 */
Qmss_Result Qmss_setQosSchedQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueNum)
{
    return qmss_internal_set_qos_sched_queue_base (subSysHnd, pdspId, queueNum, 0);
} /* Qmss_setQosSchedQueueBaseSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_setQosSchedQueueBaseSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_setQosSchedQueueBase (uint32_t queueNum)
{
    return Qmss_setQosSchedQueueBaseSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, queueNum);
} /* Qmss_setQosSchedQueueBase */

/**
 *  @b Description
 *  @n  
 *      This function is used to get the QoS ingress base queue number. The QoS ingress queue 
 *      are restricted to a set of 32 starting at a fixed base which must be a multiple of 32.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[out]  queueNum
 *      Base queue number 
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - Command buffer return status - QMSS_QOS_SCHED_INVALID_COMMAND, QMSS_QOS_SCHED_INVALID_INDEX, QMSS_QOS_SCHED_INVALID_OPTION
 */
Qmss_Result Qmss_getQosSchedQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t *queueNum)
{
    return qmss_internal_get_qos_sched_queue_base (subSysHnd, pdspId, queueNum, 0);
} /* Qmss_getQosSchedQueueBaseSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getQosSchedQueueBaseSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getQosSchedQueueBase (uint32_t *queueNum)
{
    return Qmss_getQosSchedQueueBaseSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, queueNum);
} /* Qmss_getQosSchedQueueBase */

/**
 *  @b Description
 *  @n  
 *      This function is used to configure the QoS packet scheduler's credit interval timer
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  timerConstant
 *      The PDSP timer determines when credit is passed out. The timer is 
 *      configured by supplying a timer constant. The constant is computed as follows:
 *      
 *      timerConstant = (QMSS_Clock_Frequency * Desired_Interval) / 2
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_COMMAND
 *                QMSS_QOS_SCHED_INVALID_INDEX
 *                QMSS_QOS_SCHED_INVALID_OPTION\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 */
Qmss_Result Qmss_configureQosSchedTimerSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t timerConstant)
{
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    uint32_t           cmdBuf = 0;
    Qmss_Result        retVal;
    void              *key;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    key = Qmss_osalCsEnter();
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, 
              QMSS_QOSSCHED_COMMAND_TIMER_CONFIG);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX, 
              timerConstant);
    regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0 = cmdBuf;

    /* Wait till command is executed */
    qmss_wait_qos_sched_command_buf (regPtr);

    retVal = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD1, 
                       QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE);

    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_configureQosSchedTimerSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_configureQosSchedTimerSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_configureQosSchedTimer (uint32_t timerConstant)
{
    return Qmss_configureQosSchedTimerSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, timerConstant);
} /* Qmss_configureQosSchedTimer */

/* Internal function to generate shadow accesses */
static Qmss_Result qmss_qos_sched_shadow (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, uint32_t id, uint32_t option)
{
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    uint32_t           magic;
    uint32_t           cmdBuf = 0;
    uint32_t           maxPorts;
    Qmss_Result        retVal;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);

    maxPorts = qmss_qos_sched_get_max_ports (magic);

    /* Check parameters */
    if (id != QMSS_QOSCHED_PORT_SHADOW_CFG_ID_QOS_SCHED_PORT)
    {
        if (magic != QMSS_QOS_SCHED_MAGIC_DROPSCHED)
        {
            return QMSS_QOS_SCHED_REQ_DROP_SCHED;
        }
        /* Only allowed to use port 0 with drop scheduler */
        maxPorts = 1;
    }

    if (port >= maxPorts) 
    {
        return QMSS_QOS_SCHED_INVALID_PORT;
    }

    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, QMSS_QOSSCHED_COMMAND_SHADOW_PORT);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION, option);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_INDEX_CFG_ID, id);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_INDEX_CFG_IDX, port);
    regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0 = cmdBuf;

    /* Wait till command is executed */
    qmss_wait_qos_sched_command_buf (regPtr);

    retVal = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD1, 
                       QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE);

    return retVal;
} /* qmss_qos_sched_shadow */

/* Internal function to send a port specific command */
static Qmss_Result qmss_qos_sched_port_control (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, uint32_t cmd, uint32_t option, int32_t critSec)
{
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    uint32_t           cmdBuf = 0;
    Qmss_Result        retVal;
    void              *key;
    uint32_t           magic;
    uint32_t           maxPorts;
    QMSS_COMPILE_TIME_SIZE_CHECK (sizeof(Qmss_QosSchedNarrowRegs) == QOS_SCHED_OVERLAY_SIZE);
    QMSS_COMPILE_TIME_SIZE_CHECK (sizeof(Qmss_QosSchedWideRegs) == QOS_SCHED_OVERLAY_SIZE);
    QMSS_COMPILE_TIME_SIZE_CHECK (sizeof(Qmss_QosSchedDropPlusNarrowRegs) == QOS_SCHED_OVERLAY_SIZE);

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);
        
    maxPorts = qmss_qos_sched_get_max_ports (magic);

    if (port >= maxPorts) 
    {
        return QMSS_QOS_SCHED_INVALID_PORT;
    }

    if (critSec) 
    {
        key = Qmss_osalCsEnter();
    }
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, cmd);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION, option);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX, port);
    regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0 = cmdBuf;

    /* Wait till command is executed */
    qmss_wait_qos_sched_command_buf (regPtr);

    retVal = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD1, 
                       QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE);

    if (critSec)
    {
        Qmss_osalCsExit(key);
    }

    return retVal;
} /* qmss_qos_sched_port_control */

/**
 *  @b Description
 *  @n  
 *      This function is used to enable a QoS port. 
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Port to enable - Valid range is 0 to @ref QMSS_QOS_SCHED_MAX_PHYS_PORTS
 *      (if build is qos scheduler) or 0 to @ref 
 *      QMSS_QOS_SCHED_DROP_SCHED_MAX_PHYS_PORTS (if build is with drop 
 *      scheduler)
 *      
 *  @pre  
 *      The port's configuration must be performed by the host before calling this function.
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_COMMAND
 *                QMSS_QOS_SCHED_INVALID_INDEX
 *                QMSS_QOS_SCHED_INVALID_OPTION\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 */
Qmss_Result Qmss_enableQosSchedPortSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port)
{
    return qmss_qos_sched_port_control (subSysHnd, pdspId, port, QMSS_QOSSCHED_COMMAND_ENABLE_PORT, 1, 1);
} /* Qmss_enableQosSchedPortSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_enableQosSchedPortSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_enableQosSchedPort (uint32_t port)
{
    return Qmss_enableQosSchedPortSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port);
} /* Qmss_enableQosSchedPort */

/**
 *  @b Description
 *  @n  
 *      This function is used to disable a QoS scheduler port. Only disabled ports can be modified by the host.
 *      When a port is disabled all packet in the QoS queue contained in that port are discarded.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Port to disable - Valid range is 0 to @ref QMSS_QOS_SCHED_MAX_PHYS_PORTS
 *      (if build is qos scheduler) or 0 to @ref 
 *      QMSS_QOS_SCHED_DROP_SCHED_MAX_PHYS_PORTS (if build is with drop 
 *      scheduler)
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_COMMAND
 *                QMSS_QOS_SCHED_INVALID_INDEX
 *                QMSS_QOS_SCHED_INVALID_OPTION\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 */
Qmss_Result Qmss_disableQosSchedPortSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port)
{
    return qmss_qos_sched_port_control (subSysHnd, pdspId, port, QMSS_QOSSCHED_COMMAND_ENABLE_PORT, 0, 1);
} /* Qmss_disableQosSchedPortSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_disableQosSchedPortSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_disableQosSchedPort (uint32_t port)
{
    return Qmss_disableQosSchedPortSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port);
} /* Qmss_disableQosSchedPort */

/**
 *  @b Description
 *  @n  
 *      This function is used to configure a QoS scheduler port.  The configuration
 *      operates on a shadow configuration such that the configuration can occur
 *      whether or not the port is enabled.  
 *
 *      The entire configuration must be specified.  If unknown the previous 
 *      config can be retrieved with @ref Qmss_getCfgQosSchedPortSubSys then modified.
 *
 *      This function copies the new configuration to the shadow area, then
 *      sends a command to the firmware to accept the change.  This minimizes
 *      network downtime during reconfiguration.
 *
 *      If the queues or groups are renumbered by changing counts, the descriptors
 *      are left in the original queues.  Thus if renumbering is needed,
 *      it is suggested to disable the port before reconfiguring.  If
 *      only the credits are changed, the reconfiguration should be
 *      done with the port enabled.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Port to configure - Valid range is 0 to @ref QMSS_QOS_SCHED_MAX_PHYS_PORTS
 *      (if build is qos scheduler) or 0 to @ref 
 *      QMSS_QOS_SCHED_DROP_SCHED_MAX_PHYS_PORTS (if build is with drop 
 *      scheduler)
 *
 *  @param[in]  cfg
 *      Port configuration parameters
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_GROUP
 *                QMSS_QOS_SCHED_INVALID_QUEUE
 *                QMSS_QOS_SCHED_INVALID_WRRTYPE
 *                QMSS_QOS_SCHED_INVALID_CIRTYPE
 *                QMSS_QOS_SCHED_INVALID_CONGTYPE
 *                QMSS_QOS_SCHED_INVALID_QUEUE_NUM
 *                QMSS_QOS_SCHED_INVALID_PARAM
 *                QMSS_QOS_SCHED_ISJOINT_EVEN_ONLY
 *                QMSS_QOS_SCHED_ISJOINT_LITE_ONLY
 */
Qmss_Result Qmss_putCfgQosSchedPortSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedPortCfg *cfg)
{
    Qmss_QosSchedRegs           *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedPortHeaderRegs *pShadowReg;
    Qmss_QosSchedPortRate_t     *commonPortRate, *packetPortRate;
    Qmss_QosSchedGroupCfg       *pGroupCfg;
    Qmss_QosSchedGroupBOPRegs   *pGroupBopReg;
    Qmss_QosSchedGroupBAPRegs   *pGroupBapReg;
    Qmss_QosSchedQueueCfg       *pQueueCfg;
    Qmss_QosSchedQueueRegs      *pQueueReg;
    uint32_t                     wrrIsBytes, cirIsBytes, cirIsPackets, congIsBytes, throtIsBytes, isJoint;
    uint32_t                     groupCirIsBytes, groupCirIsPackets;
    uint32_t                     temp;
    int                          group, queue, count;
    Qmss_Result                  retVal;
    uint32_t                     magic;
    void                        *key;
    int                          simuPktByte = 0;
    int                          rateRet; /* return value from check_rate */
    
    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);

    simuPktByte = (magic == QMSS_QOS_SCHED_MAGIC_MULTIGROUP);

    /* Check input arguments */
    if (! cfg)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    isJoint = 0;
    if (cfg->isJoint == Qmss_QosSchedIsJoint_JOINT)
    { 
        if (port % 2)
        {
            return QMSS_QOS_SCHED_ISJOINT_EVEN_ONLY;
        }
        isJoint = 1;
    }

    if ((retVal = qmss_qos_sched_check_args (1, port, cfg->groupCount, 1, regPtr, isJoint)) != 
        QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        return retVal;
    }
    
    if (qmss_qos_sched_act_types_is_bytes (
           &wrrIsBytes, NULL, cfg->wrrType, Qmss_QosSchedAcctType_INHERITED, 0))
    {
        return QMSS_QOS_SCHED_INVALID_WRRTYPE;
    }

    if (qmss_qos_sched_act_types_is_bytes (
            &cirIsBytes, &cirIsPackets, cfg->cirType, 
            Qmss_QosSchedAcctType_INHERITED, simuPktByte))
    {
        return QMSS_QOS_SCHED_INVALID_CIRTYPE;
    }
    
    if (qmss_qos_sched_act_types_is_bytes (
            &congIsBytes, NULL, cfg->congestionType, 
            Qmss_QosSchedAcctType_INHERITED, 0))
    {
        return QMSS_QOS_SCHED_INVALID_CONGTYPE;
    }

    if (qmss_qos_sched_act_types_is_bytes (
            &throtIsBytes, NULL, cfg->outThrotType, 
            Qmss_QosSchedAcctType_INHERITED, 0))
    {
        return QMSS_QOS_SCHED_INVALID_THROTTYPE;
    }

    if ((rateRet = qmss_qos_sched_check_rate (
                       cfg->cirIteration, cfg->cirMax)) 
        >= 0)
    {
        return rateRet + QMSS_QOS_SCHED_INVALID_PORTCIR;
    }

    if (simuPktByte)
    {
        /* Check port params for simutaneous byte+packet load */
        if ((rateRet = qmss_qos_sched_check_rate (
                           cfg->pktCirIteration, cfg->pktCirMax))
            >= 0)
        {
            return rateRet + QMSS_QOS_SCHED_INVALID_PORTPKTCIR;
        }
    }
    else
    {
        /* Check port params for byte or packet loads */
        if (cfg->pktCirIteration)
        {
            return QMSS_QOS_SCHED_INVALID_PORTPKTCIR;
        }

        if (cfg->pktCirMax)
        {
            return QMSS_QOS_SCHED_INVALID_PORTPKTCIR_MAX;
        }
        
    }

    /* Check group configuration */
    for (group = 0; group < cfg->groupCount; group++) 
    {
        pGroupCfg = &cfg->group[group];

        if ((retVal = qmss_qos_sched_check_args (1, port, cfg->groupCount, pGroupCfg->totQueueCount, regPtr, isJoint)) != 
            QMSS_QOS_SCHED_RETCODE_SUCCESS)
        {
            return retVal;
        }

        if ((pGroupCfg->spQueueCount + pGroupCfg->wrrQueueCount) > pGroupCfg->totQueueCount)
        {
            return QMSS_QOS_SCHED_INVALID_QUEUE;
        }

        /* Check rate type */
        if (qmss_qos_sched_act_types_is_bytes (
                &groupCirIsBytes, &groupCirIsPackets, 
                pGroupCfg->cirType, 
                cfg->cirType, simuPktByte))
        {
            return QMSS_QOS_SCHED_INVALID_GROUP_CIRTYPE;
        }

        if ((magic == QMSS_QOS_SCHED_MAGIC_MULTIGROUP) || 
            (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP))
        {
            if ((rateRet = qmss_qos_sched_check_rate (
                               pGroupCfg->cirIteration, pGroupCfg->cirMax)) 
                >= 0)
            {
                return rateRet + QMSS_QOS_SCHED_INVALID_GROUPCIR;
            }

            if ((rateRet = qmss_qos_sched_check_rate (
                               pGroupCfg->pirIteration, pGroupCfg->pirMax)) 
                >= 0)
            {
                return rateRet + QMSS_QOS_SCHED_INVALID_GROUPPIR;
            }

            if (simuPktByte && groupCirIsBytes && groupCirIsPackets)
            {
                /* Check group params for simutaneous packets & bytes */
                if ((rateRet = qmss_qos_sched_check_rate (
                                   pGroupCfg->pktCirIteration, 
                                   pGroupCfg->pktCirMax)) 
                    >= 0)
                {
                    return rateRet + QMSS_QOS_SCHED_INVALID_GROUPPKTCIR;
                }

                if ((rateRet = qmss_qos_sched_check_rate (
                                   pGroupCfg->pktPirIteration, 
                                   pGroupCfg->pktPirMax)) 
                    >= 0)
                {
                    return rateRet + QMSS_QOS_SCHED_INVALID_GROUPPKTPIR;
                }
            }
            else
            {
                /* Check group params for byte or packet configs */
                if (pGroupCfg->pktCirIteration)
                {
                    return QMSS_QOS_SCHED_INVALID_GROUPPKTCIR;
                }

                if (pGroupCfg->pktCirMax)
                {
                    return QMSS_QOS_SCHED_INVALID_GROUPPKTCIR_MAX;
                }

                if (pGroupCfg->pktPirIteration)
                {
                    return QMSS_QOS_SCHED_INVALID_PKTPIR_ITERATION;
                }

                if (pGroupCfg->pktPirMax)
                {
                    return QMSS_QOS_SCHED_INVALID_PKTPIR_MAX;
                }
            }

            if (pGroupCfg->wrrInitialCredit < 0)
            {
                return QMSS_QOS_SCHED_INVALID_WRR_CREDIT;
            }

            if (pGroupCfg->wrrInitialCredit > 0x3FFFFFFF)
            {
                /* Addition of credit would overflow */
                return QMSS_QOS_SCHED_INVALID_WRR_CREDIT;
            }

            if (pGroupCfg->wrrInitialCredit > 0)
            {
                /* Adding too few credits per iteration will make firmware take too
                 * much cycles, so restricting to reasonable values (>= 50 bytes, >= 1 packet)
                 */
                if (wrrIsBytes)
                {
                    if (pGroupCfg->wrrInitialCredit < 
                        (50 << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT))
                    {
                        return QMSS_QOS_SCHED_SMALL_WRR_CREDIT;
                    }
                }
                else
                {
                    if (pGroupCfg->wrrInitialCredit < 
                        (1 << QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT))
                    {
                        return QMSS_QOS_SCHED_SMALL_WRR_CREDIT;
                    }
                }
            }
        }
        else
        {
            if ((pGroupCfg->cirMax) ||
                (pGroupCfg->cirIteration) ||
                (pGroupCfg->pirMax) ||
                (pGroupCfg->pirIteration) ||
                (pGroupCfg->wrrInitialCredit))
            {
                return QMSS_QOS_SCHED_UNUSED_GROUP_ARGS;
            }
        }

        /* Check queue configuration */
        for (queue = 0; queue < pGroupCfg->totQueueCount; queue++) 
        {
            pQueueCfg = &pGroupCfg->Queue[queue];
            if (pQueueCfg->wrrInitialCredit < 0)
            {
                return QMSS_QOS_SCHED_INVALID_WRR_CREDIT;
            }

            if (pQueueCfg->wrrInitialCredit > 0x3FFFFFFF)
            {
                /* Addition of credit could overflow */
                return QMSS_QOS_SCHED_INVALID_WRR_CREDIT;
            }

            if (pQueueCfg->wrrInitialCredit > 0)
            {
                /* Adding too few credits per iteration will make firmware take too
                 * much cycles, so restricting to reasonable values (>= 50 bytes, >= 1 packet)
                 */
                if (wrrIsBytes)
                {
                    if (pQueueCfg->wrrInitialCredit < 
                        (50 << QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT))
                    {
                        return QMSS_QOS_SCHED_SMALL_WRR_CREDIT;
                    }
                }
                else
                {
                    if (pQueueCfg->wrrInitialCredit < 
                        (1 << QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT))
                    {
                        return QMSS_QOS_SCHED_SMALL_WRR_CREDIT;
                    }
                }
            }
        }
    }

    key = Qmss_osalCsEnter();

    /* Program the port */
    if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
    {
        pShadowReg     = &regPtr->u.wide.SHADOW_U.PORT.header;
        commonPortRate = &regPtr->u.wide.SHADOW_U.PORT.rate;
        packetPortRate = NULL;
        pGroupBopReg   = regPtr->u.wide.SHADOW_U.PORT.GROUPS;
        pGroupBapReg   = NULL;
    }
    else if (magic == QMSS_QOS_SCHED_MAGIC_DROPSCHED)
    {
        pShadowReg     = &regPtr->u.dropPlusNarrow.SHADOW_U.PORT.header;
        commonPortRate = &regPtr->u.dropPlusNarrow.SHADOW_U.PORT.rate;
        packetPortRate = NULL;
        pGroupBopReg   = regPtr->u.dropPlusNarrow.SHADOW_U.PORT.GROUPS;
        pGroupBapReg   = NULL;
    }
    else 
    {
        pShadowReg     = &regPtr->u.narrow.SHADOW_U.PORT.header;
        commonPortRate = &regPtr->u.narrow.SHADOW_U.PORT.byteRate;
        packetPortRate = &regPtr->u.narrow.SHADOW_U.PORT.packetRate;
        pGroupBopReg   = NULL;
        pGroupBapReg   = regPtr->u.narrow.SHADOW_U.PORT.GROUPS;
    }
    temp = 0;
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_WRR_IS_BYTES,
              wrrIsBytes);
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CONG_THRESH_IS_BYTES,
              congIsBytes);
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_THROT_IS_BYTES,
              throtIsBytes);
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_IS_JOINT,
              isJoint);
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_NUM_GROUPS,
              cfg->groupCount);
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_QNUM_QMGR,
              qmss_convert_qmss_queue_to_bitfield (cfg->outputQueue));
    if (simuPktByte)
    {
        CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_PKTS,
                  cirIsPackets);
        CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_BYTES,
                  cirIsBytes);
    }
    else
    {
       CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_IS_BYTES,
                 cirIsBytes);
    }

    pShadowReg->PORT_FLAGS           = temp;

    temp = 0;
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_REMOVE_BYTES,
              cfg->removeBytes);
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OVERHEAD_BYTES,
              cfg->overheadBytes);
    CSL_FINS (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OUT_THROT_THRESH,
              cfg->outThrotThresh);
    pShadowReg->PORT_FLAGS_2         = temp;

    commonPortRate->CIR_ITERATION_CREDIT = (uint32_t)cfg->cirIteration;
    commonPortRate->CIR_MAX              = (uint32_t)cfg->cirMax;

    if (packetPortRate)
    {
        if (!cirIsBytes && cirIsPackets)
        {
            /* packets only puts main rate in packet rate */
            packetPortRate->CIR_ITERATION_CREDIT = (uint32_t)cfg->cirIteration;
            packetPortRate->CIR_MAX              = (uint32_t)cfg->cirMax;
            /* enables full parameter readback */
            commonPortRate->CIR_ITERATION_CREDIT = (uint32_t)cfg->pktCirIteration;
            commonPortRate->CIR_MAX              = (uint32_t)cfg->pktCirMax;
        }
        else
        {
            /* copy the packet specific rate for bytes or both */
            packetPortRate->CIR_ITERATION_CREDIT = (uint32_t)cfg->pktCirIteration;
            packetPortRate->CIR_MAX              = (uint32_t)cfg->pktCirMax;
        }
    }

    pGroupCfg = cfg->group;

    /* Program the groups */
    for (group = 0; group < cfg->groupCount; group++, pGroupCfg++) 
    {
        temp = 0;
        CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_TOT,
                  pGroupCfg->totQueueCount);
        CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_SP,
                  pGroupCfg->spQueueCount);
        CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_WRR,
                  pGroupCfg->wrrQueueCount);

        if (pGroupBapReg)
        {
            /* Guaranteed to pass since same call done above */
            qmss_qos_sched_act_types_is_bytes (
                    &groupCirIsBytes, &groupCirIsPackets, 
                    pGroupCfg->cirType, 
                    cfg->cirType, simuPktByte);

            CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_BYTE_SHAPING,
                      groupCirIsBytes);
            CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_PKT_SHAPING,
                      groupCirIsPackets);
            CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_INHERITED,
                      (pGroupCfg->cirType == Qmss_QosSchedAcctType_INHERITED) ? 1 : 0);
            pGroupBapReg->FLAGS_QUEUE_COUNTS = temp;

            pGroupBapReg->byteRate.CIR_ITERATION       = (uint32_t)pGroupCfg->cirIteration;
            pGroupBapReg->byteRate.PIR_ITERATION       = (uint32_t)pGroupCfg->pirIteration;
            pGroupBapReg->byteRate.CIR_MAX             = (uint32_t)pGroupCfg->cirMax;
            pGroupBapReg->byteRate.PIR_MAX             = (uint32_t)pGroupCfg->pirMax;
            if (groupCirIsBytes && groupCirIsPackets)
            {
                /* the pktPir/pktCir is only used for "both" */
                pGroupBapReg->packetRate.CIR_ITERATION = (uint32_t)pGroupCfg->pktCirIteration;
                pGroupBapReg->packetRate.PIR_ITERATION = (uint32_t)pGroupCfg->pktPirIteration;
                pGroupBapReg->packetRate.CIR_MAX       = (uint32_t)pGroupCfg->pktCirMax;
                pGroupBapReg->packetRate.PIR_MAX       = (uint32_t)pGroupCfg->pktPirMax;
            }                
            else
            {
                pGroupBapReg->packetRate.CIR_ITERATION = (uint32_t)pGroupCfg->cirIteration;
                pGroupBapReg->packetRate.PIR_ITERATION = (uint32_t)pGroupCfg->pirIteration;
                pGroupBapReg->packetRate.CIR_MAX       = (uint32_t)pGroupCfg->cirMax;
                pGroupBapReg->packetRate.PIR_MAX       = (uint32_t)pGroupCfg->pirMax;
            }
            pGroupBapReg->wrrCfg.WRR_INITIAL_CREDIT    = (uint32_t)pGroupCfg->wrrInitialCredit;
        }
        else
        {
            pGroupBopReg->FLAGS_QUEUE_COUNTS = temp;
            pGroupBopReg->rate.CIR_ITERATION            = (uint32_t)pGroupCfg->cirIteration;
            pGroupBopReg->rate.PIR_ITERATION            = (uint32_t)pGroupCfg->pirIteration;
            pGroupBopReg->rate.CIR_MAX                  = (uint32_t)pGroupCfg->cirMax;
            pGroupBopReg->rate.PIR_MAX                  = (uint32_t)pGroupCfg->pirMax;
            pGroupBopReg->wrrCfg.WRR_INITIAL_CREDIT     = (uint32_t)pGroupCfg->wrrInitialCredit;
        }

        /* Program the queues */
        count = pGroupCfg->totQueueCount;
        if (isJoint) /* also guarantees lite port */
        {
            if (count > QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP)
            {
                count = QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
            }
        }
        for (queue = 0; queue < count; queue++) 
        {
            pQueueCfg = &pGroupCfg->Queue[queue];
            if (pGroupBopReg)
            {
                pQueueReg = &pGroupBopReg->QUEUE[queue];
            }
            else
            {
                pQueueReg = &pGroupBapReg->QUEUE[queue];
            }

            pQueueReg->WRR_INITIAL_CREDIT = (uint32_t)pQueueCfg->wrrInitialCredit;;
            pQueueReg->CONGESTION_THRESH  = pQueueCfg->congestionThresh;
        }
        if (pGroupBapReg) 
        {
            pGroupBapReg++;
        }
        else
        {
            pGroupBopReg++;
        }
    }

    /* Now that the configuration is written, tell the FW to commit it */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSCHED_PORT_SHADOW_CFG_ID_QOS_SCHED_PORT, 1);

    /* send the config to odd port for joint */
    if (retVal == QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        if (isJoint)
        {
            /* patch the config for odd port - isJoint set guarantees narrow/lite port */
            pGroupCfg = &cfg->group[0];
            if (pGroupBapReg)
            {
                pGroupBapReg --; /* narrow/lite port only has one group */
            }
            if (pGroupBopReg)
            {
                pGroupBopReg --;
            }
            if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
            {
                pGroupBopReg   = regPtr->u.wide.SHADOW_U.PORT.GROUPS;
                pGroupBapReg   = NULL;
            }
            else if (magic == QMSS_QOS_SCHED_MAGIC_DROPSCHED)
            {
                pGroupBopReg   = regPtr->u.dropPlusNarrow.SHADOW_U.PORT.GROUPS;
                pGroupBapReg   = NULL;
            }
            else 
            {
                pGroupBopReg   = NULL;
                pGroupBapReg   = regPtr->u.narrow.SHADOW_U.PORT.GROUPS;
            }

            temp = 0;
            count = pGroupCfg->spQueueCount - QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
            if (count < 0)
            {
                count = 0;
            }
            CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_SP,
                      ((uint32_t)count));

            count = pGroupCfg->spQueueCount + pGroupCfg->wrrQueueCount - 
                    QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
            if (count < 0)
            {
                count = 0;
            }
            CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_WRR,
                      ((uint32_t)count));
            count = pGroupCfg->totQueueCount - QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
            if (count < 0)
            {
                count = 0;
            }
            CSL_FINS (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_TOT,
                      ((uint32_t)count));

            if (pGroupBapReg)
            {
                pGroupBapReg->FLAGS_QUEUE_COUNTS = temp;
            }
            else
            {
                pGroupBopReg->FLAGS_QUEUE_COUNTS = temp;
            }


            /* Program the queues */
            for (queue = 0; queue < count; queue++) 
            {
                pQueueCfg = &pGroupCfg->Queue[queue + QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP];
                if (pGroupBapReg)
                {
                    pQueueReg = &pGroupBapReg->QUEUE[queue];
                }
                else
                {
                    pQueueReg = &pGroupBopReg->QUEUE[queue];
                }
    
                pQueueReg->WRR_INITIAL_CREDIT = (uint32_t)pQueueCfg->wrrInitialCredit;;
                pQueueReg->CONGESTION_THRESH  = pQueueCfg->congestionThresh;
            }

            /* Disable the odd port */
            retVal = qmss_qos_sched_port_control (subSysHnd, pdspId, port + 1, QMSS_QOSSCHED_COMMAND_ENABLE_PORT, 0, 0);

            /* configure odd port */
            if (retVal == QMSS_QOS_SCHED_RETCODE_SUCCESS)
            {
                retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port + 1, QMSS_QOSCHED_PORT_SHADOW_CFG_ID_QOS_SCHED_PORT, 1);
            }
        }
    }

    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_putCfgQosSchedPortSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_putCfgQosSchedPortSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_putCfgQosSchedPort (uint32_t port, Qmss_QosSchedPortCfg *cfg)
{
    return Qmss_putCfgQosSchedPortSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, cfg);
} /* Qmss_putCfgQosSchedPort */

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the current configuration of a QoS
 *      scheduler port.  It sends a command to the firmware which
 *      copies the configuration from the active area to the shadow area
 *      (the active area continues to run!)
 *
 *      This function can be used either for diagnostic purposes to verify
 *      the configuration, or to query the current configuration, modify
 *      it, and put it back using @ref Qmss_putCfgQosSchedPortSubSys.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Port to query - Valid range is 0 to @ref QMSS_QOS_SCHED_MAX_PHYS_PORTS
 *      (if build is qos scheduler) or 0 to @ref 
 *      QMSS_QOS_SCHED_DROP_SCHED_MAX_PHYS_PORTS (if build is with drop 
 *      scheduler)
 *
 *  @param[out]  cfg
 *      Port configuration parameters
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_PARAM
 */
Qmss_Result Qmss_getCfgQosSchedPortSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedPortCfg *cfg)
{
    Qmss_QosSchedRegs           *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedPortHeaderRegs *pShadowReg;
    Qmss_QosSchedPortRate_t     *commonPortRate, *packetPortRate;
    Qmss_QosSchedGroupCfg       *pGroupCfg;
    Qmss_QosSchedGroupBOPRegs   *pGroupBopReg;
    Qmss_QosSchedGroupBAPRegs   *pGroupBapReg;
    Qmss_QosSchedQueueCfg       *pQueueCfg;
    Qmss_QosSchedQueueRegs      *pQueueReg;
    uint32_t                     temp, isJoint;
    int                          group, queue, count;
    Qmss_Result                  retVal;
    void                        *key;
    uint32_t                     magic;
    int                          simuPktByte = 0;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);

    if (! cfg)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    key = Qmss_osalCsEnter();

    /* Request firmware to copy port's config to shadow */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSCHED_PORT_SHADOW_CFG_ID_QOS_SCHED_PORT, 0);
    if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS) 
    {
        goto end;
    }

    /* Convert from registers to *cfg */
    memset (cfg, 0, sizeof(*cfg));

    if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
    {
        pShadowReg     = &regPtr->u.wide.SHADOW_U.PORT.header;
        commonPortRate = &regPtr->u.wide.SHADOW_U.PORT.rate;
        packetPortRate = NULL;
        pGroupBopReg   = regPtr->u.wide.SHADOW_U.PORT.GROUPS;
        pGroupBapReg   = NULL;
        simuPktByte    = 0;
    }
    else if (magic == QMSS_QOS_SCHED_MAGIC_DROPSCHED)
    {
        pShadowReg     = &regPtr->u.dropPlusNarrow.SHADOW_U.PORT.header;
        commonPortRate = &regPtr->u.dropPlusNarrow.SHADOW_U.PORT.rate;
        packetPortRate = NULL;
        pGroupBopReg   = regPtr->u.dropPlusNarrow.SHADOW_U.PORT.GROUPS;
        pGroupBapReg   = NULL;
        simuPktByte    = 0;
    }
    else 
    {
        pShadowReg     = &regPtr->u.narrow.SHADOW_U.PORT.header;
        commonPortRate = &regPtr->u.narrow.SHADOW_U.PORT.byteRate;
        packetPortRate = &regPtr->u.narrow.SHADOW_U.PORT.packetRate;
        pGroupBopReg   = NULL;
        pGroupBapReg   = regPtr->u.narrow.SHADOW_U.PORT.GROUPS;
        simuPktByte    = 1;
    }

    temp                = pShadowReg->PORT_FLAGS;
    cfg->wrrType        = qmss_qos_sched_is_bytes_to_act_types(
                              CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_WRR_IS_BYTES), 0, 0);
    cfg->congestionType = qmss_qos_sched_is_bytes_to_act_types(
                              CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CONG_THRESH_IS_BYTES), 0, 0);
    cfg->outThrotType   = qmss_qos_sched_is_bytes_to_act_types(
                              CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_THROT_IS_BYTES), 0, 0);
    isJoint             = CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_IS_JOINT);
    cfg->isJoint        = isJoint ? Qmss_QosSchedIsJoint_JOINT : Qmss_QosSchedIsJoint_SPLIT;
    cfg->groupCount     = CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_NUM_GROUPS);
    cfg->outputQueue    = qmss_convert_bitfield_to_qmss_queue (
                              subSysHnd,
                              CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_QNUM_QMGR));
    if (simuPktByte)
    {
       cfg->cirType = qmss_qos_sched_is_bytes_to_act_types(
                          CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_BYTES), 
                          CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_PKTS), 1);
    }
    else
    {
       cfg->cirType = qmss_qos_sched_is_bytes_to_act_types(
                          CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_IS_BYTES), 
                          0, 0);
    }

    temp                = pShadowReg->PORT_FLAGS_2;
    cfg->removeBytes    = CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_REMOVE_BYTES);
    cfg->overheadBytes  = CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OVERHEAD_BYTES);
    cfg->outThrotThresh = CSL_FEXT (temp, QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OUT_THROT_THRESH);

    cfg->cirIteration   = (int32_t)commonPortRate->CIR_ITERATION_CREDIT;
    cfg->cirMax         = (int32_t)commonPortRate->CIR_MAX;

    if (packetPortRate)
    {
        if (cfg->cirType == Qmss_QosSchedAcctType_PACKETS)
        {
            /* packets only puts main rate in packet rate */
            cfg->cirIteration    = (int32_t)packetPortRate->CIR_ITERATION_CREDIT;
            cfg->cirMax          = (int32_t)packetPortRate->CIR_MAX;
            /* enables full parameter readback */
            cfg->pktCirIteration = (int32_t)commonPortRate->CIR_ITERATION_CREDIT;
            cfg->pktCirMax       = (int32_t)commonPortRate->CIR_MAX;
        }
        else
        {
            /* copy the packet specific rate for bytes or both */
            cfg->pktCirIteration = (int32_t)packetPortRate->CIR_ITERATION_CREDIT;
            cfg->pktCirMax       = (int32_t)packetPortRate->CIR_MAX;
        }

    }

    pGroupCfg = cfg->group;

    for (group = 0; group < cfg->groupCount; group++, pGroupCfg++) 
    {
        if (pGroupBapReg)
        {
            temp                        = pGroupBapReg->FLAGS_QUEUE_COUNTS;

            pGroupCfg->cirType = qmss_qos_sched_is_bytes_to_act_types(
                CSL_FEXT (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_BYTE_SHAPING), 
                CSL_FEXT (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_PKT_SHAPING), 1);

            switch (pGroupCfg->cirType)
            {
                default: /* cant happen */
                case Qmss_QosSchedAcctType_INHERITED:
                case Qmss_QosSchedAcctType_PACKETS:
                    pGroupCfg->cirIteration    = (int32_t)pGroupBapReg->packetRate.CIR_ITERATION;
                    pGroupCfg->pirIteration    = (int32_t)pGroupBapReg->packetRate.PIR_ITERATION;
                    pGroupCfg->cirMax          = (int32_t)pGroupBapReg->packetRate.CIR_MAX;
                    pGroupCfg->pirMax          = (int32_t)pGroupBapReg->packetRate.PIR_MAX;
                    /* pGroupCfg->pkt* is 0 from memset above */
                    break;
                case Qmss_QosSchedAcctType_BYTES:
                    pGroupCfg->cirIteration    = (int32_t)pGroupBapReg->byteRate.CIR_ITERATION;
                    pGroupCfg->pirIteration    = (int32_t)pGroupBapReg->byteRate.PIR_ITERATION;
                    pGroupCfg->cirMax          = (int32_t)pGroupBapReg->byteRate.CIR_MAX;
                    pGroupCfg->pirMax          = (int32_t)pGroupBapReg->byteRate.PIR_MAX;
                    /* pGroupCfg->pkt* is 0 from memset above */
                    break;
                case Qmss_QosSchedAcctType_BOTH:
                    pGroupCfg->cirIteration    = (int32_t)pGroupBapReg->byteRate.CIR_ITERATION;
                    pGroupCfg->pirIteration    = (int32_t)pGroupBapReg->byteRate.PIR_ITERATION;
                    pGroupCfg->cirMax          = (int32_t)pGroupBapReg->byteRate.CIR_MAX;
                    pGroupCfg->pirMax          = (int32_t)pGroupBapReg->byteRate.PIR_MAX;
                    pGroupCfg->pktCirIteration = (int32_t)pGroupBapReg->packetRate.CIR_ITERATION;
                    pGroupCfg->pktPirIteration = (int32_t)pGroupBapReg->packetRate.PIR_ITERATION;
                    pGroupCfg->pktCirMax       = (int32_t)pGroupBapReg->packetRate.CIR_MAX;
                    pGroupCfg->pktPirMax       = (int32_t)pGroupBapReg->packetRate.PIR_MAX;
                    break;
            }

            pGroupCfg->wrrInitialCredit = (int32_t)pGroupBapReg->wrrCfg.WRR_INITIAL_CREDIT;

            if (CSL_FEXT (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_INHERITED))
            {
                /* User gave "inherited" instead of actual type */
                pGroupCfg->cirType              = Qmss_QosSchedAcctType_INHERITED;
            }
        }
        else
        {
            pGroupCfg->cirType              = Qmss_QosSchedAcctType_INHERITED;
            if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
            {
                pGroupCfg->cirIteration     = (int32_t)pGroupBopReg->rate.CIR_ITERATION;
                pGroupCfg->pirIteration     = (int32_t)pGroupBopReg->rate.PIR_ITERATION;
                pGroupCfg->cirMax           = (int32_t)pGroupBopReg->rate.CIR_MAX;
                pGroupCfg->pirMax           = (int32_t)pGroupBopReg->rate.PIR_MAX;
                pGroupCfg->wrrInitialCredit = (int32_t)pGroupBopReg->wrrCfg.WRR_INITIAL_CREDIT;
            }
            else
            {
                /* group params are set to 0 by memset() above */
            }
            temp                 = pGroupBopReg->FLAGS_QUEUE_COUNTS;
        }

        pGroupCfg->totQueueCount = CSL_FEXT (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_TOT);
        pGroupCfg->spQueueCount  = CSL_FEXT (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_SP);
        pGroupCfg->wrrQueueCount = CSL_FEXT (temp, QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_WRR);


        /* Convert the queues */
        count = pGroupCfg->totQueueCount;
        if (isJoint) /* also guarantees lite port */
        {
            if (count > QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP)
            {
                count = QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
            }
        }

        for (queue = 0; queue < count; queue++) 
        {
            pQueueCfg = &pGroupCfg->Queue[queue];
            if (pGroupBapReg)
            {
                pQueueReg = &pGroupBapReg->QUEUE[queue];
            }
            else
            {
                pQueueReg = &pGroupBopReg->QUEUE[queue];
            }

            pQueueCfg->wrrInitialCredit = (int32_t)pQueueReg->WRR_INITIAL_CREDIT;
            pQueueCfg->congestionThresh = pQueueReg->CONGESTION_THRESH;
        }

        if (pGroupBapReg) 
        {
            pGroupBapReg++;
        }
        else
        {
            pGroupBopReg++;
        }
    }

    /* Request firmware to copy adjacent port's config to shadow */
    if (isJoint)
    {
        /* patch the config for odd port - isJoint set guarantees narrow/lite port */
        pGroupCfg = &cfg->group[0];
        if (pGroupBapReg)
        {
            pGroupBapReg --; /* narrow/lite port only has one group */
        }
        if (pGroupBopReg)
        {
            pGroupBopReg --;
        }

        retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port + 1, QMSS_QOSCHED_PORT_SHADOW_CFG_ID_QOS_SCHED_PORT, 0);
        if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS) 
        {
            goto end;
        }
        /* Convert the queues */
        count = pGroupCfg->totQueueCount - QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP;
        for (queue = 0; queue < count; queue++) 
        {
            pQueueCfg = &pGroupCfg->Queue[queue + QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP];
            if (pGroupBapReg)
            {
                pQueueReg = &pGroupBapReg->QUEUE[queue];
            }
            else
            {
                pQueueReg = &pGroupBopReg->QUEUE[queue];
            }

            pQueueCfg->wrrInitialCredit = (int32_t)pQueueReg->WRR_INITIAL_CREDIT;
            pQueueCfg->congestionThresh = pQueueReg->CONGESTION_THRESH;
        }
    }

    retVal = QMSS_QOS_SCHED_RETCODE_SUCCESS;
end:
    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_getCfgQosSchedPortSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getCfgQosSchedPortSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getCfgQosSchedPort (uint32_t port, Qmss_QosSchedPortCfg *cfg)
{
    return Qmss_getCfgQosSchedPortSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, cfg);
} /* Qmss_getCfgQosSchedPort */

/**
 *  @b Description
 *  @n Internal function that implements the various queue stats query APIs
 */
#define QMSS_QOS_SCHED_STATS_ALL_QUEUES 0xff
static Qmss_Result Qmss_internalGetQosSchedStatsSubSys (
    Qmss_SubSysHnd subSysHnd, 
    Qmss_PdspId pdspId, 
    Qmss_QosSchedStats *stats, 
    int *nStats, 
    uint32_t port, 
    uint32_t group, 
    uint32_t queue, 
    uint32_t which_reset)
{
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    uint32_t           cmdBuf = 0;
    Qmss_Result        retVal;
    void              *key;
    uint32_t           magic;
    uint32_t           checkQueue = queue;

    /* Clear stats in case of error */
    memset (stats, 0, sizeof(*stats));

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    /* Its OK not to give array size if no array */
    if ((! nStats) && (stats))
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);

    if ((checkQueue == QMSS_QOS_SCHED_STATS_ALL_QUEUES) &&
        (magic != QMSS_QOS_SCHED_MAGIC_DROPSCHED))
    {
        checkQueue = 0;
    }
    /* if DROPSCHED, QMSS_QOS_SCHED_STATS_ALL_QUEUES is unsupported and check args returns error */ 

    /* Note this cannot detect if queue is out of range since LLD doesn't track whether
     * a port is joint */
    if ((retVal = qmss_qos_sched_check_args (0, port, group, checkQueue, regPtr, 1)) != 
        QMSS_QOS_SCHED_RETCODE_SUCCESS)
    {
        return retVal;
    }

    key = Qmss_osalCsEnter();

    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, 
              QMSS_QOSSCHED_COMMAND_STATS_REQUEST);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION, 
              which_reset);
    if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
    {
        CSL_FINS (cmdBuf, QMSS_QOSSCHED_WIDE_COMMAND_INDEX_PORT, port);
        CSL_FINS (cmdBuf, QMSS_QOSSCHED_WIDE_COMMAND_INDEX_GROUP, group);
        CSL_FINS (cmdBuf, QMSS_QOSSCHED_WIDE_COMMAND_INDEX_QUEUE, queue);
    }
    else
    {
        CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_INDEX_PORT, port);
        CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_INDEX_GROUP, group);
        CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_INDEX_QUEUE, queue);
    }

    regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0 = cmdBuf;

    /* Wait till command is executed */
    qmss_wait_qos_sched_command_buf (regPtr);

    retVal = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD1, 
                       QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE);

    if (stats && (retVal == QMSS_QOS_SCHED_RETCODE_SUCCESS))
    {
       Qmss_QosSchedStatsRegs *statsReg_p;
       int queues = 1;

       if (queue == QMSS_QOS_SCHED_STATS_ALL_QUEUES)
       {
           uint32_t flags;
           if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
           {
               statsReg_p = regPtr->u.wide.SHADOW_U.STATS.stats;
               flags      = regPtr->u.wide.SHADOW_U.STATS.flags;
           }
           else
           {
               statsReg_p = regPtr->u.narrow.SHADOW_U.STATS.stats;
               flags      = regPtr->u.narrow.SHADOW_U.STATS.flags;
           }
           queues = CSL_FEXT (flags, QMSS_QOSSCHED_GROUP_SHADOW_STATS_FLAGS_QUEUES);
       }
       else
       {
           if (magic == QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP)
           {
               statsReg_p = &regPtr->u.wide.STATS;
           } 
           else if (magic == QMSS_QOS_SCHED_MAGIC_MULTIGROUP)
           {
               statsReg_p = &regPtr->u.narrow.STATS;
           } 
           else 
           {
               statsReg_p = &regPtr->u.dropPlusNarrow.STATS;
           }
       }

       if (queues <= *nStats)
       {
           *nStats = queues;
           for (queue = 0; queue < queues; queue++, stats++, statsReg_p++)
           {
               /* Must read the results inside a critical section since more than one 
                * core could ask for stats at the same time
                */
               stats->bytesForwarded   = (((uint64_t)statsReg_p->BYTES_FORWARDED_MSW) << 32) |
                                         (((uint64_t)statsReg_p->BYTES_FORWARDED_LSW)      );
               stats->bytesDiscarded   = (((uint64_t)statsReg_p->BYTES_DISCARDED_MSW) << 32) |
                                         (((uint64_t)statsReg_p->BYTES_DISCARDED_LSW)      );
               stats->packetsForwarded = statsReg_p->PACKETS_FORWARDED;
               stats->packetsDiscarded = statsReg_p->PACKETS_DISCARDED;
           }
       }
    }

    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_internalGetQosSchedStatsSubSys */

/**
 *  @b Description
 *  @n  
 *      This function returns the bytes and packets forwarded and discarded by the
 *      port.  The statistics are synchronously and atomically collected and reset.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[out]  stats
 *      Statistics structure to contain returned stats.  NULL is valid in order to
 *      reset and discard the stats.
 *
 *  @param[in]  port
 *      Port to request stats - Valid range is 0 to @ref QMSS_QOS_SCHED_MAX_PHYS_PORTS
 *      (if build is qos scheduler) or 0 to @ref 
 *      QMSS_QOS_SCHED_DROP_SCHED_MAX_PHYS_PORTS (if build is with drop 
 *      scheduler)
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  group
 *      Group to request stats - Valid range is 0 to 4
 *
 *  @param[in]  queue
 *      Queue in group to request stats - Valid range is 0 to 7
 *
 *  @param[in]  which_reset
 *      Bitfield listing the stats to reset.  Can be one or more of
 *         QMSS_QOS_SCHED_STATS_FORWARDED_BYTES   |
 *         QMSS_QOS_SCHED_STATS_FORWARDED_PACKETS |
 *         QMSS_QOS_SCHED_STATS_DISCARDED_BYTES   |
 *         QMSS_QOS_SCHED_STATS_DISCARDED_PACKETS
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_GROUP
 *                QMSS_QOS_SCHED_INVALID_QUEUE
 */
Qmss_Result Qmss_getQosSchedStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QosSchedStats *stats, uint32_t port, uint32_t group, uint32_t queue, uint32_t which_reset)
{
    int nStats = 1;
    return Qmss_internalGetQosSchedStatsSubSys 
              (QMSS_SUBSYS_HND_GLOBAL, pdspId, stats, 
               &nStats, port, group, queue, which_reset);
} /* Qmss_getQosSchedStatsSubSys */


/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getQosSchedStatsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getQosSchedStats (Qmss_QosSchedStats *stats, uint32_t port, uint32_t group, uint32_t queue, uint32_t which_reset)
{
    int nStats = 1;
    return Qmss_internalGetQosSchedStatsSubSys 
              (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, stats, 
               &nStats, port, group, queue, which_reset);
} /* Qmss_getQosSchedStats */

/**
 *  @b Description
 *  @n  
 *      This function returns the bytes and packets forwarded and discarded by the
 *      port.  The statistics are synchronously and atomically collected and reset.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[out]  stats
 *      Array of statistics structure to contain returned stats.  NULL is valid in order to
 *      reset and discard the stats.
 *
 *  @param[in,out] nStats
 *      Input: Elements allocated in stats array
 *      Output: Elements filled in stats array (number of queues in group for this port/group)
 *
 *  @param[in]  port
 *      Port to request stats - Valid range is 0 to @ref QMSS_QOS_SCHED_MAX_PHYS_PORTS
 *      (if build is qos scheduler) or 0 to @ref 
 *      QMSS_QOS_SCHED_DROP_SCHED_MAX_PHYS_PORTS (if build is with drop 
 *      scheduler)
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  group
 *      Group to request stats - Valid range is 0 to 4
 *
 *  @param[in]  reset
 *      0: dont reset; nonzero: reset all stats in this group
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_GROUP
 *                QMSS_QOS_SCHED_INVALID_QUEUE
 */
Qmss_Result Qmss_getQosSchedGroupStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QosSchedStats *stats, int *nStats, uint32_t port, uint32_t group, uint32_t reset)
{
    uint32_t which_reset = 0;
    if (reset)
    {
        which_reset = QMSS_QOS_SCHED_STATS_FORWARDED_BYTES   |
                      QMSS_QOS_SCHED_STATS_FORWARDED_PACKETS |
                      QMSS_QOS_SCHED_STATS_DISCARDED_BYTES   |
                      QMSS_QOS_SCHED_STATS_DISCARDED_PACKETS;
    }

    return Qmss_internalGetQosSchedStatsSubSys 
              (subSysHnd, pdspId, stats, 
               nStats, port, group, QMSS_QOS_SCHED_STATS_ALL_QUEUES, which_reset);
} /* Qmss_getQosSchedGroupStatsSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getQosSchedGroupStatsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getQosSchedGroupStats (Qmss_QosSchedStats *stats, int *nStats, uint32_t port, uint32_t group, uint32_t reset)
{
    return Qmss_getQosSchedGroupStatsSubSys 
              (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, stats, 
               nStats, port, group, reset);
} /* Qmss_getQosSchedGroupStats */

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version information of the current
 *      QoS scheduler firmware running on the PDSP.
 *
 *  @retval
 *      Version Information, or 0xffffffff if error
 */
uint32_t Qmss_getQosSchedFwVersionSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId)
{
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);

    if (! regPtr)
    {
        return 0xffffffff;
    }

    return regPtr->VERSION;
} /* Qmss_getQosSchedFwVersionSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getQosSchedFwVersionSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
uint32_t Qmss_getQosSchedFwVersion (void)
{
    return Qmss_getQosSchedFwVersionSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP);
} /* Qmss_getQosSchedFwVersion */

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version information of the current
 *      QoS scheduler firmware running on the PDSP.
 *
 *  @retval
 *      Magic number, or 0xffffffff if error
 */
uint32_t Qmss_getQosSchedFwMagicSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId)
{
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);

    return qmss_qos_sched_get_magic (regPtr);
} /* Qmss_getQosSchedFwMagicSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getQosSchedFwMagicSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
uint32_t Qmss_getQosSchedFwMagic (void)
{
    return Qmss_getQosSchedFwMagicSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP);
} /* Qmss_getQosSchedFwMagic */

/**
 *  @b Description
 *  @n
 *      This function performs unit conversion from bits per second rate
 *      to a cir/pir rate given the configured ticks per second.
 *      The result is rounded to nearest.
 *
 *  @param[out] result
 *      Calculated result or 0 on under/over flow
 *
 *  @param[in] ticksPerSecond
 *      QoS timer rate.  This is not checked against actual configuration.
 *      For example, 20000 means 50us.
 *
 *  @param[in] bitRatePerSecond
 *      Requested bits per second.  For example, 50000000 means 50 megabits
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_OVERFLOW  -- result overflows
 *                QMSS_QOS_SCHED_UNDERFLOW -- result is 0, but
 *                                            bitRatePerSecond was not 0
 *                QMSS_QOS_SCHED_INVALID_PARAM -- result was NULL
 */
Qmss_Result Qmss_convertQosSchedBitRate (int32_t *result, uint32_t ticksPerSecond,
                                         uint32_t bitRatePerSecond)
{
    uint64_t res;

    if (! result)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Return 0 if under/over flow detected */
    *result = 0;

    /* Add 1/2 ticksPerSecond to round to nearest */
    res =   bitRatePerSecond;
    /* -3 to account for bits vs bytes */
    res <<= (QMSS_QOS_SCHED_BYTES_SCALE_SHIFT - 3);
    res +=  (ticksPerSecond >> 1);
    res /=  ticksPerSecond;

    if (bitRatePerSecond && (! res))
    {
        return QMSS_QOS_SCHED_UNDERFLOW;
    }

    if (res > (uint64_t)((uint32_t)res))
    {
        return QMSS_QOS_SCHED_OVERFLOW;
    }

    *result = res;

    return QMSS_QOS_SCHED_RETCODE_SUCCESS;
} /* Qmss_convertQosSchedBitRate */

/**
 *  @b Description
 *  @n
 *      This function performs unit conversion from packets per second rate
 *      to a cir/pir rate given the configured ticks per second.
 *      The result is rounded to nearest.
 *
 *  @param[out] result
 *      Calculated result or 0 on under/over flow
 *
 *  @param[in] ticksPerSecond
 *      QoS timer rate.  This is not checked against actual configuration.
 *      For example, 20000 means 50us.
 *
 *  @param[in] packetRatePerSecond
 *      Requested packets per second.  For example, 1000 means 1000 packets
 *      per second.
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_OVERFLOW  -- result overflows
 *                QMSS_QOS_SCHED_UNDERFLOW -- result is 0, but
 *                                            packetRatePerSecond was not 0
 *                QMSS_QOS_SCHED_INVALID_PARAM -- result was NULL
 */
Qmss_Result Qmss_convertQosSchedPacketRate (int32_t *result, uint32_t ticksPerSecond,
                                            uint32_t packetRatePerSecond)
{
    uint64_t res;

    if (! result)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Return 0 if under/over flow detected */
    *result = 0;

    /* Add 1/2 ticksPerSecond to round to nearest */
    res =   packetRatePerSecond;
    res <<= QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT;
    res +=  (ticksPerSecond >> 1);
    res /=  ticksPerSecond;
                    
    if (packetRatePerSecond && (! res))
    {
        return QMSS_QOS_SCHED_UNDERFLOW;
    }

    if (res > (uint64_t)((uint32_t)res))
    {
        return QMSS_QOS_SCHED_OVERFLOW;
    }
    
    *result = res;

    return QMSS_QOS_SCHED_RETCODE_SUCCESS;
} /* Qmss_convertQosSchedPacketRate */

/**
 *  @b Description
 *  @n
 *      This function performs unit conversion from bits per credit
 *      to a wrr credit.
 *      The result is rounded to nearest.
 *
 *  @param[out] result
 *      Calculated result or 0 on under/over flow
 *
 *  @param[in] wrrCredit
 *      Requested credit in bits.  For example, 80000 means 10000 bytes.
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_OVERFLOW  -- result overflows
 *                QMSS_QOS_SCHED_UNDERFLOW -- result is 0, but
 *                                            wrrCredit was not 0
 *                QMSS_QOS_SCHED_INVALID_PARAM -- result was NULL
 */
Qmss_Result Qmss_convertQosSchedWrrBits (int32_t *result, uint32_t wrrCredit)
{
    uint64_t res;

    if (! result)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Return 0 if under/over flow detected */
    *result = 0;

    /* Add 1/2 ticksPerSecond to round to nearest */
    res =   wrrCredit;
    /* -3 to account for bits vs bytes */
    res <<= (QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT - 3);

    if (wrrCredit && (! res))
    {
        return QMSS_QOS_SCHED_UNDERFLOW;
    }

    if (res > (uint64_t)((uint32_t)res))
    {
        return QMSS_QOS_SCHED_OVERFLOW;
    }

    *result = res;

    return QMSS_QOS_SCHED_RETCODE_SUCCESS;
} /* Qmss_convertQosSchedWrrBits */

/**
 *  @b Description
 *  @n
 *      This function performs unit conversion from bits per credit
 *      to a wrr credit.
 *      The result is rounded to nearest.
 *
 *  @param[out] result
 *      Calculated result or 0 on under/over flow
 *
 *  @param[in] wrrCredit
 *      Requested credits in packets.  For example, 1000 means 1000 packets.
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_OVERFLOW  -- result overflows
 *                QMSS_QOS_SCHED_UNDERFLOW -- result is 0, but
 *                                            wrrCredit was not 0
 *                QMSS_QOS_SCHED_INVALID_PARAM -- result was NULL
 */
Qmss_Result Qmss_convertQosSchedWrrPackets (int32_t *result, uint32_t wrrCredit)
{
    uint64_t res;

    if (! result)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Return 0 if under/over flow detected */
    *result = 0;

    /* Add 1/2 ticksPerSecond to round to nearest */
    res =   wrrCredit;
    res <<= QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT;
                    
    if (wrrCredit && (! res))
    {
        return QMSS_QOS_SCHED_UNDERFLOW;
    }

    if (res > (uint64_t)((uint32_t)res))
    {
        return QMSS_QOS_SCHED_OVERFLOW;
    }
    
    *result = res;

    return QMSS_QOS_SCHED_RETCODE_SUCCESS;
} /* Qmss_convertQosSchedWrrPackets */

/**
 *  @b Description
 *  @n  
 *      This function is used to set the drop scheduler ingress base queue 
 *      number.  The drop scheduler ingress queue are restricted to a set of 
 *      80 starting at a fixed base which must be a multiple of 32.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  queueNum
 *      Base queue number 
 *
 *  @retval
 *      Success - QCMD_RETCODE_SUCCESS
 *  @retval
 *      Failure - Command buffer return status - QMSS_QOS_SCHED_INVALID_COMMAND, QMSS_QOS_SCHED_INVALID_INDEX, QMSS_QOS_SCHED_INVALID_OPTION
 */
Qmss_Result Qmss_setQosSchedDropSchedQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueNum)
{
    return qmss_internal_set_qos_sched_queue_base (subSysHnd, pdspId, queueNum, 1);
} /* Qmss_setQosSchedDropSchedQueueBaseSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_setQosSchedDropSchedQueueBaseSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_setQosSchedDropSchedQueueBase (uint32_t queueNum)
{
    return Qmss_setQosSchedDropSchedQueueBaseSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, queueNum);
} /* Qmss_setQosSchedDropSchedQueueBase */

/**
 *  @b Description
 *  @n  
 *      This function is used to get the drop scheduler ingress base queue 
 *      number. The drop scheduler ingress queue are restricted to a set of 
 *      32 starting at a fixed base which must be a multiple of 32.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[out]  queueNum
 *      Base queue number 
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - Command buffer return status - QMSS_QOS_SCHED_INVALID_COMMAND, QMSS_QOS_SCHED_INVALID_INDEX, QMSS_QOS_SCHED_INVALID_OPTION
 */
Qmss_Result Qmss_getQosSchedDropSchedQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t *queueNum)
{
    return qmss_internal_get_qos_sched_queue_base (subSysHnd, pdspId, queueNum, 1);
} /* Qmss_getQosSchedDropSchedQueueBaseSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getQosSchedDropSchedQueueBaseSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getQosSchedDropSchedQueueBase (uint32_t *queueNum)
{
    return Qmss_getQosSchedDropSchedQueueBaseSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, queueNum);
} /* Qmss_getQosSchedDropSchedQueueBase */

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the current configuration the drop
 *      scheduler output profiles. 
 *      It sends a command to the firmware which
 *      copies the configuration from the active area to the shadow area
 *      (the active area continues to run!)
 *
 *      This function can be used either for diagnostic purposes to verify
 *      the configuration, or to query the current configuration, modify
 *      it, and put it back using @ref Qmss_putCfgQosSchedDropSchedOutProfsSubSys.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Drop Scheduler instance (must be 0)
 *
 *  @param[out]  profs
 *      output profiles
 *      
 *  @param[in]  nProfs
 *      Number of profiles in profs[] (for overrun check)
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_PARAM
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED
 */
Qmss_Result Qmss_getCfgQosSchedDropSchedOutProfsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedOutProf *profs, int32_t nProfs)
{
    Qmss_QosSchedRegs                 *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedDropSchedOutProfRegs *pShadowReg;
    Qmss_QosSchedDropSchedOutProf     *pCfg;
    uint32_t                           temp;
    int                                prof;
    Qmss_Result                        retVal;
    void                              *key;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (! profs)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    if (nProfs != QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Convert from registers to profs */
    memset (profs, 0, sizeof(*profs) * nProfs);

    key = Qmss_osalCsEnter();

    /* Request firmware to copy port's config to shadow */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_OUT_PROF, 0);
    if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS) 
    {
        goto end;
    }

    for (prof = 0; prof < nProfs; prof++)
    {
        pShadowReg        = regPtr->u.dropPlusNarrow.SHADOW_U.OUT_PROFILES + prof;
        pCfg              = profs + prof;
        temp              = pShadowReg->CONFIG1;
        pCfg->REDDropProb = CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_DROPPROB);
        pCfg->outputQueue = qmss_convert_bitfield_to_qmss_queue (
                                subSysHnd,
                                CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_OUTQUEUE));
        temp              = pShadowReg->CONFIG2;
        pCfg->valid       = qmss_qos_sched_is_valid_to_prof_valid (
                                CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_ENABLED));
        pCfg->cfgProfIdx  = CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_CFGIDX);
        pCfg->avgQueDepth = pShadowReg->AVG_Q_DEPTH;
    }

    retVal = QMSS_QOS_SCHED_RETCODE_SUCCESS;
end:
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_getCfgQosSchedDropSchedOutProfsSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getCfgQosSchedDropSchedOutProfsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getCfgQosSchedDropSchedOutProfs (uint32_t port, Qmss_QosSchedDropSchedOutProf *profs, int32_t nProfs)
{
    return Qmss_getCfgQosSchedDropSchedOutProfsSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, profs, nProfs);
} /* Qmss_getCfgQosSchedDropSchedOutProfs */

/**
 *  @b Description
 *  @n  
 *      This function is used to configure the drop scheduler output 
 *      profiles.  The configuration
 *      operates on a shadow configuration such that the configuration can occur
 *      whether or not the drop scheduler is enabled.  
 *
 *      The entire configuration must be specified.  If unknown the previous 
 *      config can be retrieved with @ref Qmss_getCfgQosSchedDropSchedOutProfsSubSys then modified.
 *
 *      This function copies the new configuration to the shadow area, then
 *      sends a command to the firmware to accept the change.  This minimizes
 *      network downtime during reconfiguration.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Drop Scheduler instance (must be 0)
 *
 *  @param[in]  profs
 *      output profiles
 *
 *  @param[in]  nProfs
 *      Number of profiles in profs[] (for overrun check)
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_PARAM
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED
 *                QMSS_QOS_SCHED_INVALID_VALID_FLAG
 *                QMSS_QOS_SCHED_INVALID_CFG_PROF_IDX
 */
Qmss_Result Qmss_putCfgQosSchedDropSchedOutProfsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedOutProf *profs, int32_t nProfs)
{
    Qmss_QosSchedRegs                 *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedDropSchedOutProfRegs *pShadowReg;
    Qmss_QosSchedDropSchedOutProf     *pCfg;
    uint32_t                           temp;
    int                                prof;
    uint32_t                           profIsValid[QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES];
    void                              *key;
    Qmss_Result                        retVal;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (! profs)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    if (nProfs != QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Check arguments before touching shadow */
    for (prof = 0; prof < nProfs; prof++)
    {
        pCfg = profs + prof;
        if (qmss_qos_sched_prof_valid_is_valid (&profIsValid[prof], pCfg->valid))
        {
            return QMSS_QOS_SCHED_INVALID_VALID_FLAG;
        }
        /* Only require rest of arguments to be valid if prof is valid */
        if (pCfg->valid == Qmss_QosSchedDropSchedProf_VALID)
        {
            if (pCfg->cfgProfIdx >= QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES)
            {
                return QMSS_QOS_SCHED_INVALID_CFG_PROF_IDX;
            }
        }
    }

    key = Qmss_osalCsEnter();

    for (prof = 0; prof < nProfs; prof++)
    {
        pShadowReg            = regPtr->u.dropPlusNarrow.SHADOW_U.OUT_PROFILES + prof;
        pCfg                  = profs + prof;
        temp                  = 0;
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_DROPPROB, pCfg->REDDropProb);
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_OUTQUEUE,
                  qmss_convert_qmss_queue_to_bitfield (pCfg->outputQueue));
        pShadowReg->CONFIG1 = temp;
        temp                    = 0;
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_ENABLED,
                  profIsValid[prof]);
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_CFGIDX,
                  pCfg->cfgProfIdx);
        pShadowReg->CONFIG2     = temp;
        pShadowReg->AVG_Q_DEPTH = 0;
    }

    /* Request firmware to copy shadow to port's config */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_OUT_PROF, 1);

    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_putCfgQosSchedDropSchedOutProfsSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_putCfgQosSchedDropSchedOutProfsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_putCfgQosSchedDropSchedOutProfs (uint32_t port, Qmss_QosSchedDropSchedOutProf *profs, int32_t nProfs)
{
    return Qmss_putCfgQosSchedDropSchedOutProfsSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, profs, nProfs);
} /* Qmss_putCfgQosSchedDropSchedOutProfs */

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the current configuration the drop
 *      scheduler config profiles. 
 *      It sends a command to the firmware which
 *      copies the configuration from the active area to the shadow area
 *      (the active area continues to run!)
 *
 *      This function can be used either for diagnostic purposes to verify
 *      the configuration, or to query the current configuration, modify
 *      it, and put it back using @ref Qmss_putCfgQosSchedDropSchedCfgProfsSubSys.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Drop Scheduler instance (must be 0)
 *
 *  @param[out]  profs
 *      config profiles
 *      
 *  @param[in]  nProfs
 *      Number of profiles in profs[] (for overrun check)
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_PARAM
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED
 */
Qmss_Result Qmss_getCfgQosSchedDropSchedCfgProfsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedCfgProf *profs, int32_t nProfs)
{
    Qmss_QosSchedRegs                 *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedDropSchedCfgProfRegs *pShadowReg;
    Qmss_QosSchedDropSchedCfgProf     *pCfg;
    uint32_t                           temp;
    int                                prof;
    Qmss_Result                        retVal;
    void                              *key;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (! profs)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    if (nProfs != QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Convert from registers to profs */
    memset (profs, 0, sizeof(*profs) * nProfs);

    key = Qmss_osalCsEnter ();

    /* Request firmware to copy port's config to shadow */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_CFG_PROF, 0);
    if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS) 
    {
        goto end;
    }

    for (prof = 0; prof < nProfs; prof++)
    {
        pShadowReg           = regPtr->u.dropPlusNarrow.SHADOW_U.CFG_PROFILES + prof;
        pCfg                 = profs + prof;
        temp                 = pShadowReg->CONFIG;
        pCfg->mode           = qmss_drop_sched_mode_to_enum (
                                   CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_MODE));
        pCfg->tailDropType   = qmss_qos_sched_is_bytes_to_act_types (
                                   CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TD_IS_BYTES), 0, 0);
        pCfg->timeConstantP2 = CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TCSHIFT);
        pCfg->tailDropThresh = pShadowReg->TAIL_THRESH;
        pCfg->REDLowThresh   = pShadowReg->RED_LOW;
        pCfg->REDHighThresh  = pShadowReg->RED_HIGH;
    }

    retVal = QMSS_QOS_SCHED_RETCODE_SUCCESS;
end:
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_getCfgQosSchedDropSchedCfgProfsSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getCfgQosSchedDropSchedCfgProfsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getCfgQosSchedDropSchedCfgProfs (uint32_t port, Qmss_QosSchedDropSchedCfgProf *profs, int32_t nProfs)
{
    return Qmss_getCfgQosSchedDropSchedCfgProfsSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, profs, nProfs);
} /* Qmss_getCfgQosSchedDropSchedCfgProfs */

/**
 *  @b Description
 *  @n  
 *      This function is used to configure the drop scheduler config 
 *      profiles.  The configuration
 *      operates on a shadow configuration such that the configuration can occur
 *      whether or not the drop scheduler is enabled.  
 *
 *      The entire configuration must be specified.  If unknown the previous 
 *      config can be retrieved with @ref Qmss_getCfgQosSchedDropSchedCfgProfsSubSys then modified.
 *
 *      This function copies the new configuration to the shadow area, then
 *      sends a command to the firmware to accept the change.  This minimizes
 *      network downtime during reconfiguration.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Drop Scheduler instance (must be 0)
 *
 *  @param[in]  profs
 *      config profiles
 *
 *  @param[in]  nProfs
 *      Number of profiles in profs[] (for overrun check)
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_PARAM
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED
 *                QMSS_QOS_SCHED_INVALID_DROP_MODE
 *                QMSS_QOS_SCHED_INVALID_RED_TC
 *                QMSS_QOS_SCHED_INVALID_RED_THRESH
 *                QMSS_QOS_SCHED_INVALID_RED_THRESH_RECIP
 */
Qmss_Result Qmss_putCfgQosSchedDropSchedCfgProfsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedCfgProf *profs, int32_t nProfs)
{
    Qmss_QosSchedRegs                 *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedDropSchedCfgProfRegs *pShadowReg;
    Qmss_QosSchedDropSchedCfgProf     *pCfg;
    uint32_t                           temp;
    int                                prof;
    uint32_t                           modes[QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES];
    uint32_t                           types[QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES];
    uint32_t                           recips[QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES];
    void                              *key;
    Qmss_Result                        retVal;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (! profs)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    if (nProfs != QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Check arguments before touching shadow */
    for (prof = 0; prof < nProfs; prof++)
    {
        double recip;
        uint32_t diff;

        pCfg = profs + prof;
        if (qmss_drop_sched_enum_to_mode (&modes[prof], pCfg->mode))
        {
            return QMSS_QOS_SCHED_INVALID_DROP_MODE;
        }
        if (qmss_qos_sched_act_types_is_bytes (
                &types[prof], NULL, pCfg->tailDropType, Qmss_QosSchedAcctType_INHERITED, 0))
        {
            return QMSS_QOS_SCHED_INVALID_TAILDROP_THRESH_TYPE;
        }
        /* Only require RED params valid if they are enabled */
        if (pCfg->mode != Qmss_QosSchedDropSchedMode_TAILDROP)
        {
            if (pCfg->timeConstantP2 > QMSS_QOS_SCHED_DROP_SCHED_MAX_TC)
            {
                return QMSS_QOS_SCHED_INVALID_RED_TC;
            }
            if (pCfg->REDLowThresh >= pCfg->REDHighThresh)
            { 
                return QMSS_QOS_SCHED_INVALID_RED_THRESH;
            }
            /* Compute thresh recip */
            diff = (pCfg->REDHighThresh - pCfg->REDLowThresh) >> pCfg->timeConstantP2;
            if (diff == 0)
            {
                /* can't compute a useful reciprocol */
                return QMSS_QOS_SCHED_INVALID_RED_THRESH_RECIP;
            }
            recip = 1.0;
            recip /= diff;
            /* Convert to Q32 */
            recips[prof] = (uint32_t)(recip * 65536 * 65536);
            if (recips[prof] < 256)
            {
                /* can't compute a useful reciprocol */
                return QMSS_QOS_SCHED_INVALID_RED_THRESH_RECIP;
            }
        }
        else
        {
            recips[prof] = 0;
        }
        if (pCfg->tailDropThresh)
        {
            if (types[prof]) 
            {
                if (pCfg->tailDropThresh >= 0x1fffe0)
                {
                    return QMSS_QOS_SCHED_INVALID_TAIL_DROP_THRESH;
                }
            } 
            else
            {
                if (pCfg->tailDropThresh >= 0xff)
                {
                    return QMSS_QOS_SCHED_INVALID_TAIL_DROP_THRESH;
                }
            }
        }
    }

    key = Qmss_osalCsEnter();

    for (prof = 0; prof < nProfs; prof++)
    {
        pShadowReg               = regPtr->u.dropPlusNarrow.SHADOW_U.CFG_PROFILES + prof;
        pCfg                     = profs + prof;
        temp                     = 0;
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_MODE, modes[prof]);
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TD_IS_BYTES, types[prof]);
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TCSHIFT, pCfg->timeConstantP2);
        pShadowReg->CONFIG       = temp;

        pShadowReg->TAIL_THRESH  = pCfg->tailDropThresh;
        pShadowReg->RED_LOW      = pCfg->REDLowThresh;
        pShadowReg->RED_HIGH     = pCfg->REDHighThresh;
        pShadowReg->THRESH_RECIP = recips[prof];
    }

    /* Request firmware to copy shadow to port's config */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_CFG_PROF, 1);

    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_putCfgQosSchedDropSchedCfgProfsSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_putCfgQosSchedDropSchedCfgProfsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_putCfgQosSchedDropSchedCfgProfs (uint32_t port, Qmss_QosSchedDropSchedCfgProf *profs, int32_t nProfs)
{
    return Qmss_putCfgQosSchedDropSchedCfgProfsSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, profs, nProfs);
} /* Qmss_putCfgQosSchedDropSchedCfgProfs */

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the current configuration the drop
 *      scheduler input queue configs. 
 *      It sends a command to the firmware which
 *      copies the configuration from the active area to the shadow area
 *      (the active area continues to run!)
 *
 *      This function can be used either for diagnostic purposes to verify
 *      the configuration, or to query the current configuration, modify
 *      it, and put it back using @ref Qmss_putCfgQosSchedDropSchedQueCfgsSubSys.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Drop Scheduler instance (must be 0)
 *
 *  @param[out]  cfgs
 *      input queue configurations
 *      
 *  @param[in]  nCfgs
 *      Number of configurations in cfgs[] (for overrun check)
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_PARAM
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED
 */
Qmss_Result Qmss_getCfgQosSchedDropSchedQueCfgsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedQueCfg *cfgs, int32_t nCfgs)
{
    Qmss_QosSchedRegs                *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedDropSchedQueCfgRegs *pShadowReg;
    Qmss_QosSchedDropSchedQueCfg     *pCfg;
    uint32_t                          temp;
    int                               cfg;
    Qmss_Result                       retVal;
    void                             *key;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (! cfgs)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    if (nCfgs != QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Convert from registers to profs */
    memset (cfgs, 0, sizeof(*cfgs) * nCfgs);

    key = Qmss_osalCsEnter ();

    /* Request firmware to copy port's config to shadow */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_QUE_CFG, 0);
    if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS) 
    {
        goto end;
    }

    for (cfg = 0; cfg < nCfgs; cfg++)
    {
        pShadowReg         = regPtr->u.dropPlusNarrow.SHADOW_U.QUE_CONFIGS + cfg;
        pCfg               = cfgs + cfg;
        temp               = pShadowReg->CONFIG;
        pCfg->valid        = qmss_qos_sched_is_valid_to_prof_valid (
                                 CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_VALID));
        pCfg->pushStatsIdx = CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_PUSHSTATSIDX);
        pCfg->statBlockIdx = CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_STATSIDX);
        pCfg->outProfIdx   = CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_OUTIDX);
    }

    retVal = QMSS_QOS_SCHED_RETCODE_SUCCESS;
end:
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_getCfgQosSchedDropSchedQueCfgsSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getCfgQosSchedDropSchedQueCfgsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getCfgQosSchedDropSchedQueCfgs (uint32_t port, Qmss_QosSchedDropSchedQueCfg *cfgs, int32_t nCfgs)
{
    return Qmss_getCfgQosSchedDropSchedQueCfgsSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, cfgs, nCfgs);
} /* Qmss_getCfgQosSchedDropSchedQueCfgs */

/**
 *  @b Description
 *  @n  
 *      This function is used to configure the drop scheduler input 
 *      queue configs.  The configuration
 *      operates on a shadow configuration such that the configuration can occur
 *      whether or not the drop scheduler is enabled.  
 *
 *      The entire configuration must be specified.  If unknown the previous 
 *      config can be retrieved with @ref Qmss_getCfgQosSchedDropSchedQueCfgsSubSys then modified.
 *
 *      This function copies the new configuration to the shadow area, then
 *      sends a command to the firmware to accept the change.  This minimizes
 *      network downtime during reconfiguration.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Drop Scheduler instance (must be 0)
 *
 *  @param[in]  cfgs
 *      input queue configurations
 *
 *  @param[in]  nCfgs
 *      Number of configurations in cfgs[] (for overrun check)
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_INVALID_PARAM
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED
 *                QMSS_QOS_SCHED_INVALID_VALID_FLAG
 *                QMSS_QOS_SCHED_INVALID_STATS_BLOCK_IDX
 *                QMSS_QOS_SCHED_INVALID_OUT_PROF_IDX
 */
Qmss_Result Qmss_putCfgQosSchedDropSchedQueCfgsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedQueCfg *cfgs, int32_t nCfgs)
{
    Qmss_QosSchedRegs                *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedDropSchedQueCfgRegs *pShadowReg;
    Qmss_QosSchedDropSchedQueCfg     *pCfg;
    uint32_t                          temp;
    int                               cfg;
    uint32_t                          profIsValid[QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS];
    void                             *key;
    Qmss_Result                       retVal;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (! cfgs)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    if (nCfgs != QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Check arguments before touching shadow */
    for (cfg = 0; cfg < nCfgs; cfg++)
    {
        pCfg = cfgs + cfg;
        if (qmss_qos_sched_prof_valid_is_valid (&profIsValid[cfg], pCfg->valid))
        {
            return QMSS_QOS_SCHED_INVALID_VALID_FLAG;
        }
        /* Only require rest of arguments to be valid if prof is valid */
        if (pCfg->valid == Qmss_QosSchedDropSchedProf_VALID)
        {
            /* > is correct, because statBlockIdx is 1 based */
            if (pCfg->statBlockIdx > QMSS_QOS_SCHED_DROP_SCHED_STATS_BLOCKS)
            {
                return QMSS_QOS_SCHED_INVALID_STATS_BLOCK_IDX;
            }
            if (pCfg->outProfIdx >= QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES)
            {
                return QMSS_QOS_SCHED_INVALID_OUT_PROF_IDX;
            }
        }
    }

    key = Qmss_osalCsEnter();

    for (cfg = 0; cfg < nCfgs; cfg++)
    {
        pShadowReg              = regPtr->u.dropPlusNarrow.SHADOW_U.QUE_CONFIGS + cfg;
        pCfg                    = cfgs + cfg;
        temp                    = 0;
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_VALID, profIsValid[cfg]);
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_PUSHSTATSIDX, pCfg->pushStatsIdx);
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_STATSIDX, pCfg->statBlockIdx);
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_OUTIDX, pCfg->outProfIdx);
        pShadowReg->CONFIG      = temp;
    }

    /* Request firmware to copy shadow to port's config */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_QUE_CFG, 1);

    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_putCfgQosSchedDropSchedQueCfgsSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_putCfgQosSchedDropSchedQueCfgsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_putCfgQosSchedDropSchedQueCfgs (uint32_t port, Qmss_QosSchedDropSchedQueCfg *cfgs, int32_t nCfgs)
{
    return Qmss_putCfgQosSchedDropSchedQueCfgsSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, cfgs, nCfgs);
} /* Qmss_putCfgQosSchedDropSchedQueCfgs */

/**
 *  @b Description
 *  @n  
 *      This function is used to retrieve the current configuration the drop
 *      scheduler top level config.
 *      It sends a command to the firmware which
 *      copies the configuration from the active area to the shadow area
 *      (the active area continues to run!)
 *
 *      This function can be used either for diagnostic purposes to verify
 *      the configuration, or to query the current configuration, modify
 *      it, and put it back using @ref Qmss_putCfgQosSchedDropSchedCfgSubSys.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Drop Scheduler instance (must be 0)
 *
 *  @param[out]  pCfg
 *      Drop scheduler top level config
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED
 *                QMSS_QOS_SCHED_INVALID_PARAM
 */
Qmss_Result Qmss_getCfgQosSchedDropSchedCfgSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedCfg *pCfg)
{
    Qmss_QosSchedRegs             *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedDropSchedCfgRegs *pShadowReg;
    uint32_t                       temp;
    Qmss_Result                    retVal;
    void                          *key;
    int                            i;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (! pCfg)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Convert from registers to profs */
    memset (pCfg, 0, sizeof(*pCfg));

    key = Qmss_osalCsEnter ();

    /* Request firmware to copy port's config to shadow */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_CFG, 0);
    if (retVal != QMSS_QOS_SCHED_RETCODE_SUCCESS) 
    {
        goto end;
    }

    pShadowReg      = &regPtr->u.dropPlusNarrow.SHADOW_U.DROP_CFG;

    pCfg->seed1     = pShadowReg->SEED1;
    pCfg->seed2     = pShadowReg->SEED2;
    pCfg->seed3     = pShadowReg->SEED3;

    for (i = 0; i < QMSS_QOS_SCHED_DROP_SCHED_STATS_QUEUES; i++)
    {
        temp = pShadowReg->STATS_QUEUES[i];
        pCfg->statsQueues[i].src = qmss_convert_bitfield_to_qmss_queue (
            subSysHnd,
            CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_SRC));
        pCfg->statsQueues[i].dst = qmss_convert_bitfield_to_qmss_queue (
            subSysHnd,
            CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_DST));
    }

    retVal = QMSS_QOS_SCHED_RETCODE_SUCCESS;
end:
    Qmss_osalCsExit (key);

    return retVal;
} /* Qmss_getCfgQosSchedDropSchedCfgSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getCfgQosSchedDropSchedCfgSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getCfgQosSchedDropSchedCfg (uint32_t port, Qmss_QosSchedDropSchedCfg *pCfg)
{
    return Qmss_getCfgQosSchedDropSchedCfgSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, pCfg);
} /* Qmss_getCfgQosSchedDropSchedCfg */

/**
 *  @b Description
 *  @n  
 *      This function is used to configure the drop scheduler top level 
 *      config.  The configuration
 *      operates on a shadow configuration such that the configuration can occur
 *      whether or not the drop scheduler is enabled.  
 *
 *      The entire configuration must be specified.  If unknown the previous 
 *      config can be retrieved with @ref Qmss_getCfgQosSchedDropSchedCfgSubSys then modified.
 *
 *      This function copies the new configuration to the shadow area, then
 *      sends a command to the firmware to accept the change.  This minimizes
 *      network downtime during reconfiguration.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Drop Scheduler instance (must be 0)
 *
 *  @param[in]  pCfg
 *      Drop scheduler top level config
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED
 *                QMSS_QOS_SCHED_INVALID_VALID_FLAG
 *                QMSS_QOS_SCHED_INVALID_QOS_TICKS
 *                QMSS_QOS_SCHED_INVALID_DROP_TICKS
 *                QMSS_QOS_SCHED_INVALID_INTNUM
 *                QMSS_QOS_SCHED_INVALID_PARAM
 */
Qmss_Result Qmss_putCfgQosSchedDropSchedCfgSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedCfg *pCfg)
{
    Qmss_QosSchedRegs             *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    Qmss_QosSchedDropSchedCfgRegs *pShadowReg;
    uint32_t                       temp;
    void                          *key;
    Qmss_Result                    retVal;
    int                            i;

    /* Check arguments before touching shadow */
    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (! pCfg)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    key = Qmss_osalCsEnter();

    pShadowReg         = &regPtr->u.dropPlusNarrow.SHADOW_U.DROP_CFG;

    pShadowReg->SEED1  = pCfg->seed1;
    pShadowReg->SEED2  = pCfg->seed2;
    pShadowReg->SEED3  = pCfg->seed3;

    for (i = 0; i < QMSS_QOS_SCHED_DROP_SCHED_STATS_QUEUES; i++)
    {
        temp = 0;
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_SRC, 
                  qmss_convert_qmss_queue_to_bitfield (pCfg->statsQueues[i].src));
        CSL_FINS (temp, QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_DST, 
                  qmss_convert_qmss_queue_to_bitfield (pCfg->statsQueues[i].dst));
        pShadowReg->STATS_QUEUES[i] = temp;
    }

    /* Request firmware to copy shadow to port's config */
    retVal = qmss_qos_sched_shadow (subSysHnd, pdspId, port, QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_CFG, 1);

    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_putCfgQosSchedDropSchedCfgSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_putCfgQosSchedDropSchedCfgSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_putCfgQosSchedDropSchedCfg (uint32_t port, Qmss_QosSchedDropSchedCfg *pCfg)
{
    return Qmss_putCfgQosSchedDropSchedCfgSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port, pCfg);
} /* Qmss_putCfgQosSchedDropSchedCfg */

/**
 *  @b Description
 *  @n  
 *      This function is used to enable the drop scheduler. 
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Port to disable - must be 0
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_COMMAND
 *                QMSS_QOS_SCHED_INVALID_INDEX
 *                QMSS_QOS_SCHED_INVALID_OPTION\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 */
Qmss_Result Qmss_enableQosSchedDropSchedSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port)
{
    return qmss_internal_enable_drop_sched (subSysHnd, pdspId, port, 1);
} /* Qmss_enableQosSchedDropSchedSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_enableQosSchedDropSchedSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_enableQosSchedDropSched (uint32_t port)
{
    return Qmss_enableQosSchedDropSchedSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port);
} /* Qmss_enableQosSchedDropSched */

/**
 *  @b Description
 *  @n  
 *      This function is used to disable the drop scheduler. 
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  port
 *      Port to disable - must be 0
 *      
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *  @retval
 *      Failure - 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_COMMAND
 *                QMSS_QOS_SCHED_INVALID_INDEX
 *                QMSS_QOS_SCHED_INVALID_OPTION\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_INVALID_PORT
 */
Qmss_Result Qmss_disableQosSchedDropSchedSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port)
{
    return qmss_internal_enable_drop_sched (subSysHnd, pdspId, port, 1);
} /* Qmss_disableQosSchedDropSchedSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_disableQosSchedDropSchedSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_disableQosSchedDropSched (uint32_t port)
{
    return Qmss_disableQosSchedDropSchedSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, port);
} /* Qmss_disableQosSchedDropSched */

/**
 *  @b Description
 *  @n  
 *      This function returns the bytes and packets forwarded and 
 *      discarded by the drop scheduler.  The statistics are synchronously 
 *      and atomically collected and reset.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[out]  stats
 *      Statistics structure to contain returned stats.  NULL is valid in order to
 *      reset and discard the stats.
 *
 *  @param[in]  port
 *      Port to request stats - Must be 0.
 *
 *  @param[in]  block
 *      Which stats block to query/reset
 *
 *  @param[in]  which_reset
 *      Bitfield listing the stats to reset.  Can be one or more of
 *         QMSS_QOS_SCHED_STATS_FORWARDED_BYTES   |
 *         QMSS_QOS_SCHED_STATS_FORWARDED_PACKETS |
 *         QMSS_QOS_SCHED_STATS_DISCARDED_BYTES   |
 *         QMSS_QOS_SCHED_STATS_DISCARDED_PACKETS
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *
 *  @retval
 *      Failure 
 *             The following errors are detected by the firmware:
 *                QMSS_QOS_SCHED_INVALID_INDEX\n
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED,
 *                QMSS_QOS_SCHED_INVALID_PORT,
 *                QMSS_QOS_SCHED_INVALID_PARAM
 */
Qmss_Result Qmss_getQosSchedDropSchedStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QosSchedDropSchedStats *stats, uint32_t port, uint32_t block, uint32_t which_reset)
{
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    uint32_t           cmdBuf = 0;
    Qmss_Result        retVal;
    void              *key;
    uint32_t           magic;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);
        
    if (magic != QMSS_QOS_SCHED_MAGIC_DROPSCHED)
    {
        return QMSS_QOS_SCHED_REQ_DROP_SCHED;
    }

    if (port != 0)
    {
        return QMSS_QOS_SCHED_INVALID_PORT;
    }

    if (block >= QMSS_QOS_SCHED_DROP_SCHED_STATS_BLOCKS)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Clear stats in case of error */
    memset (stats, 0, sizeof(*stats));

    key = Qmss_osalCsEnter();

    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND, 
              QMSS_QOSSCHED_COMMAND_STATS_REQUEST);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION, 
              which_reset | 0x80);
    CSL_FINS (cmdBuf, QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX, 
              block);

    regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD0 = cmdBuf;

    /* Wait till command is executed */
    qmss_wait_qos_sched_command_buf (regPtr);

    retVal = CSL_FEXT (regPtr->COMMAND_BUFFER.COMMAND_BUFFER_WORD1, 
                       QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE);

    if (stats && (retVal == QMSS_QOS_SCHED_RETCODE_SUCCESS))
    {
       /* Must read the results inside a critical section since more than one 
        * core could ask for stats at the same time
        */
       stats->bytesForwarded   = regPtr->u.narrow.STATS.BYTES_FORWARDED_LSW;
       stats->bytesDiscarded   = regPtr->u.narrow.STATS.BYTES_DISCARDED_LSW;
       stats->packetsForwarded = regPtr->u.narrow.STATS.PACKETS_FORWARDED;
       stats->packetsDiscarded = regPtr->u.narrow.STATS.PACKETS_DISCARDED;
    }

    Qmss_osalCsExit(key);

    return retVal;
} /* Qmss_getQosSchedDropSchedStatsSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_getQosSchedDropSchedStatsSubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_getQosSchedDropSchedStats (Qmss_QosSchedDropSchedStats *stats, uint32_t port, uint32_t block, uint32_t which_reset)
{
    return Qmss_getQosSchedDropSchedStatsSubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, stats, port, block, which_reset);
} /* Qmss_getQosSchedDropSchedStats */

/**
 *  @b Description
 *  @n  
 *      This function converts a push stats descriptor handling the
 *      cache and endian as necessary.  As soon as this function
 *      returns, the descriptor can be returned.
 *
 *      The drop scheduler statistics are 32 bit internally.  If
 *      one of the stats' MSBs gets set, and the queue's 
 *      @ref Qmss_QosSchedDropSchedQueCfg.statBlockIdx is nonzero,
 *      then a descriptor is popped from 
 *      Qmss_QosSchedDropSchedCfg.statsQueues[statBlockIdx-1].src,
 *      stats are copied in, then it is returned to sw via 
 *      Qmss_QosSchedDropSchedCfg.statsQueues[statBlockIdx-1].dst.
 *      This function converts those descriptors.
 *
 *  @param[out]  stats
 *      Statistics structure to contain returned stats. 
 *
 *  @param[in]  desc
 *      Pointer to descriptor containing stats.  It is assumed that any
 *      required cache or fence operations are done on the desc
 *
 *  @retval
 *      Success - QMSS_QOS_SCHED_RETCODE_SUCCESS
 *
 *  @retval
 *      Failure 
 *             The following errors are detected by the LLD:
 *                QMSS_QOS_SCHED_REQ_DROP_SCHED,
 *                QMSS_QOS_SCHED_INVALID_PORT,
 *                QMSS_QOS_SCHED_INVALID_PARAM
 */
Qmss_Result Qmss_convertQosSchedDropSchedPushStats (Qmss_QosSchedDropSchedPushStats *stats, void *desc)
{
    Qmss_QosSchedDropSchedPushStatsFmt *statsDesc = (Qmss_QosSchedDropSchedPushStatsFmt *)QMSS_DESC_PTR(desc);
    uint32_t                            temp;
    uint8_t                             size;

    if (!stats)
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    /* Invalidate the cache */
    Qmss_osalBeginMemAccess (statsDesc, sizeof(Qmss_QosSchedDropSchedPushStatsFmt));
    temp                          = statsDesc->STATSINFO;
    size                          = CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_STATSINFO_SIZE);
    if (size != sizeof(Qmss_QosSchedDropSchedPushStatsFmt))
    {
        return QMSS_QOS_SCHED_INVALID_PARAM;
    }

    stats->statBlockIdx           = CSL_FEXT (temp, QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_STATSINFO_BLOCK);
    stats->block.bytesForwarded   = statsDesc->BYTES_FORWARDED;
    stats->block.bytesDiscarded   = statsDesc->BYTES_DISCARDED;
    stats->block.packetsForwarded = statsDesc->PACKETS_FORWARDED;
    stats->block.packetsDiscarded = statsDesc->PACKETS_DISCARDED;

    return QMSS_QOS_SCHED_RETCODE_SUCCESS;
} /* Qmss_convertQosSchedDropSchedPushStats */

/**
 *  @b Description
 *  @n  
 *
 *      This function pushes a descriptor onto a queue specified by the queue handle using
 *      a proxy function that is part of qos scheduler with drop scheduler.  This can be 
 *      used on architectures which cannot do atomic 64 bit writes AND whose hardware 
 *      queue proxy isn't working.  Most code need not use this functions such as those 
 *      running on c66x which can do atomic 64 bit write (and instead shouls use
 *      Qmss_queuePush()).
 *
 *      The "descSize" is used to specify the size of the descriptor being pushed.
 *      The optional parameter "packetSize" is used specify the size of packet during pop 
 *      operation. 
 *      The optional parameter "location" is used to override the default(tail) and push the packet 
 *      to the head of the queue.
 *
 *      **No validation is done on the input parameters**.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      QoS PDSP to program.
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *          Qmss_PdspId_PDSP4 selects firmware downloaded to PDSP 4
 *          Qmss_PdspId_PDSP6 selects firmware downloaded to PDSP 6
 *          Qmss_PdspId_PDSP8 selects firmware downloaded to PDSP 8
 *
 *  @param[in]  hnd
 *      Queue handle.
 *
 *  @param[in]  descAddr
 *      Memory address of the descriptor. Should be a global address.
 * 
 *  @param[in]  packetSize
 *      Size of packet pointed to by the descriptor.
 * 
 *  @param[in]  descSize
 *      Size of the descriptor. Minimum size is 16 bytes. Maximum size is 256 bytes
 * 
 *  @param[in]  location
 *      0 - Tail.
 *      1 - Head
 *
 *  @pre  
 *      Qmss_queueOpenSubSys function should be called before calling this function.
 *
 *  @retval
 *      None
 */
Qmss_Result Qmss_qosSchedDropSchedPushProxySubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QueueHnd hnd, void *descAddr, uint32_t packetSize, uint32_t descSize, Qmss_Location location)
{
    Qmss_QosSchedRegs *regPtr = qmss_qos_sched_get_regptr (subSysHnd, pdspId);
    uint32_t           subSys = (uint32_t)subSysHnd;
    uint32_t           qSubSys = QMSS_QUEUE_SUBSYS(hnd);
    uint32_t           regc = 0, regd = 0;
    void              *key;
    uint32_t           magic;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (subSys != qSubSys)
    {
        /* must be on same subsys */
        return QMSS_INVALID_PARAM;
    }

    magic = qmss_qos_sched_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_QOSSCHED_MAGIC_MAGIC);
        
    if (magic != QMSS_QOS_SCHED_MAGIC_DROPSCHED)
    {
        return QMSS_QOS_SCHED_REQ_DROP_SCHED;
    }

    if (location != Qmss_Location_TAIL)
    {
        return QMSS_QOS_SCHED_TAIL_ONLY;
    }

    descAddr = Qmss_internalVirtToPhyDesc (hnd, descAddr);

    CSL_FINS (regc, QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_QNUMSIZE_QNUM, 
              Qmss_getQIDFromHandle(hnd));
    CSL_FINS (regc, QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_QNUMSIZE_SIZE,
              packetSize);

    regd = ((uint32_t) descAddr | ((descSize >> 4) - 1));

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalMtCsEnter ();

    while (regPtr->u.narrow.PROXY.DESCPTR);
    regPtr->u.narrow.PROXY.QUEUENUM_SIZE = regc;
    regPtr->u.narrow.PROXY.DESCPTR       = regd;
    
    /* End Critical Section */   
    Qmss_osalMtCsExit (key);

    return QMSS_QOS_SCHED_RETCODE_SUCCESS;
} /* Qmss_qosSchedDropSchedPushProxySubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_qosSchedDropSchedPushProxySubSys
 *      which operates on the second PDSP of the global subsystem.
 */
Qmss_Result Qmss_qosSchedDropSchedPushProxy (Qmss_QueueHnd hnd, void *descAddr, uint32_t packetSize, uint32_t descSize, Qmss_Location location)
{
    return Qmss_qosSchedDropSchedPushProxySubSys (QMSS_SUBSYS_HND_GLOBAL, QMSS_QOS_DEFAULT_PDSP, hnd, descAddr, packetSize, descSize, location);
} /* Qmss_qosSchedDropSchedPushProxy */

/**
@}
*/

