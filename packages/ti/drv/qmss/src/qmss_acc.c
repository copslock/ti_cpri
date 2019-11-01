/**
 *   @file  qmss_acc.c
 *
 *   @brief   
 *      This is the Queue Manager accumulator and interrupt distributor implementation.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2016, Texas Instruments, Inc.
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
#include <string.h>

/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/include/qmss_pvt.h>

/* QMSS OSAL layer */
#include <qmss_osal.h>

/* RM LLD includes */
#include <ti/drv/rm/rm_services.h>

/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/* QMSS Local Object is valid flag */
extern uint8_t                  qmssLObjIsValid[];

/* QMSS Local object */
extern Qmss_LocalObj            qmssLObj[];

/**********************************************************************
 ************************** Functions *********************************
 **********************************************************************/

/** @addtogroup QMSS_LLD_FUNCTION
@{ 
*/

/**
 *  @b Description
 *  @n  
 *      This function programs the accumulator with values passed in the cfg structure
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      Accumulator to open
 *          Qmss_PdspId_PDSP1..8 selects firmware downloaded to PDSP 1 to 8.
 *
 *  @param[in]  channel
 *      Channel number within accumulator to open
 *      
 *  @post  
 *      Handle is returned, and channel is allocated to current context if a resource
 *      manager is in use.
 *
 *  @retval
 *      Handle
 *  @retval
 *      Success >= 0 (handle is returned) 
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure = QMSS_RESOURCE_ALLOCATE_INIT_DENIED
 *  @retval 
 *      Failure < 0
 */
Qmss_AccChHandle Qmss_openAccumulatorChSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint8_t channel)
{
    uint32_t            subSys   = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;

    if (qmssLObjIsValid[subSys] == 0)
        return QMSS_NOT_INITIALIZED;

    if (! lObjPtr->regs.qmPdspCmdReg[pdspId])
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    if (lObjPtr->qmRmServiceHandle)
    {
        int32_t accumCh = (int32_t)channel;
        int32_t intd = lObjPtr->pdspIntdMap[pdspId];
        if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_ALLOCATE_INIT, 
                            lObjPtr->rmAccumCh[intd], &accumCh, 1, 0, NULL))
        {
            return QMSS_RESOURCE_ALLOCATE_INIT_DENIED;
        }
    }

    return (Qmss_AccChHandle) Qmss_accMakeHandle (subSys, pdspId, channel);
} /* Qmss_openAccumulatorChSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_openAccumulatorCh
 *      which operates on the global subsystem.
 */
Qmss_AccChHandle Qmss_openAccumulatorCh (Qmss_PdspId pdspId, uint8_t channel)
{
    return Qmss_openAccumulatorChSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, channel);
} /* Qmss_openAccumulatorCh */

/**
 *  @b Description
 *  @n  
 *      This function programs the accumulator channel with values passed in the cfg structure
 *
 *  @param[in]  handle
 *      Accumulator channel handle to program
 *
 *  @param[in]  cfg
 *      Initialization structure that contains accumulator configurable fields
 *
 *  @post  
 *      Accumulator channel is programmed.
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_INVALID_PARAM
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND, QMSS_ACC_INVALID_CHANNEL, 
 *                  QMSS_ACC_CHANNEL_NOT_ACTIVE, QMSS_ACC_CHANNEL_ALREADY_ACTIVE, QMSS_ACC_INVALID_QUEUE_NUMBER,
 *                  QMSS_SUBSYS_UNSUPPORTED
 */
Qmss_Result Qmss_cfgAccumulatorCh (Qmss_AccChHandle handle, Qmss_AccCmdCfg *cfg)
{
    Qmss_AccCmd          cmd;
    volatile uint32_t    *cmdPtr, *reg;
    uint32_t             index;
    uint8_t              result;
    void                 *key;      
    uint32_t             subSys;
    Qmss_PdspId          pdspId;
    uint8_t              channel;
    Qmss_LocalObjParams *lObjPtr;
    Qmss_Result          retVal;

    if (cfg == NULL)
    {
        return QMSS_INVALID_PARAM;
    }

    if (handle == 0)
    {
        return QMSS_INVALID_PARAM;
    }

    Qmss_accBreakHandle ((int32_t)handle, &subSys, &pdspId, &channel);
    lObjPtr = &qmssLObj[subSys].p;

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalAccCsEnter ();
    
    memset ((void *) &cmd, 0, sizeof (Qmss_AccCmd));
    CSL_FINSR (cmd.word0, 7, 0, channel);
    CSL_FINSR (cmd.word0, 15, 8, cfg->command);
    cmd.word1 = cfg->queueEnMask;
    cmd.word2 = (uint32_t)Qmss_internalVirtToPhy (subSys, (void *)cfg->listAddress);
    CSL_FINSR (cmd.word3, 15, 0, cfg->queMgrIndex);
    CSL_FINSR (cmd.word3, 31, 16, cfg->maxPageEntries);
    CSL_FINSR (cmd.word4, 15, 0, cfg->timerLoadCount);
    CSL_FINSR (cmd.word4, 17, 16, cfg->interruptPacingMode);
    CSL_FINSR (cmd.word4, 19, 18, cfg->listEntrySize);
    CSL_FINSR (cmd.word4, 20, 20, cfg->listCountMode);
    CSL_FINSR (cmd.word4, 21, 21, cfg->multiQueueMode);
    
    /* Point to the accumulator command register's last word */
    reg = (uint32_t *) ((uint8_t *) lObjPtr->regs.qmPdspCmdReg[pdspId] + 4 * 4);

    /* Write command word last */
    cmdPtr = ((uint32_t *) &cmd) + 4;

    for (index = 0; index < 5; index++)
        *reg-- = *cmdPtr--;

    /* wait for the command to clear */
    reg++;
    do
    {
        result = CSL_FEXTR (*reg, 15, 8);  
    } while (result != 0);

    retVal = (Qmss_Result) (CSL_FEXTR (*reg, 31, 24));
    
    /* End Critical Section */
    Qmss_osalAccCsExit (key);

    return retVal;
} /* Qmss_cfgAccumulatorCh */

/**
 *  @b Description
 *  @n  
 *      This function programs the accumulator with values passed in the cfg structure
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      Accumulator to program
 *          Qmss_PdspId_PDSP1 selects firmware downloaded to PDSP 1
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *
 *  @param[in]  cfg
 *      Initialization structure that contains accumulator configurable fields
 *
 *  @post  
 *      Accumulator channel is programmed and channel is alocated from
 *      resource managere (if present)
 *
 *      This combines the operation of allocating the channel from the 
 *      resource manager (if any), and configuring it.  These features 
 *      can be accessed separately by using @ref Qmss_openAccumulatorCh 
 *      and @ref Qmss_cfgAccumulatorCh.  In applications where
 *      the accumulator is reconfigured "on the fly", separateing 
 *      open/close (and calling from outside realtime code)
 *      will reduce the latency of config/stop (when called in realtime code).
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure - QMSS_INVALID_PARAM
 *  @retval
 *      Failure = QMSS_RESOURCE_ALLOCATE_INIT_DENIED
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND, QMSS_ACC_INVALID_CHANNEL, 
 *                  QMSS_ACC_CHANNEL_NOT_ACTIVE, QMSS_ACC_CHANNEL_ALREADY_ACTIVE, QMSS_ACC_INVALID_QUEUE_NUMBER
 */

Qmss_Result Qmss_programAccumulatorSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_AccCmdCfg *cfg)
{
    Qmss_AccChHandle handle = Qmss_openAccumulatorChSubSys (subSysHnd, pdspId, cfg->channel);
    if (handle < 0)
    {
        return (Qmss_Result)handle;
    }

    return Qmss_cfgAccumulatorCh (handle, cfg);
    /* Note: handle is intentionally discarded because it has no backing memory. */
} /* Qmss_programAccumulatorSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_programAccumulatorSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_programAccumulator (Qmss_PdspId pdspId, Qmss_AccCmdCfg *cfg)
{
    return Qmss_programAccumulatorSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, cfg);
} /* Qmss_programAccumulator */

/**
 *  @b Description
 *  @n  
 *      This function releases the channel to the resource manager, 
 *      if present.
 *
 *  @param[in]  handle
 *      Channel handle of channel to close
 *
 *  @post  
 *      Channel is released from resource manager (if present)
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND, QMSS_ACC_INVALID_CHANNEL, 
 *                  QMSS_ACC_CHANNEL_NOT_ACTIVE, QMSS_ACC_CHANNEL_ALREADY_ACTIVE, QMSS_ACC_INVALID_QUEUE_NUMBER
 */
Qmss_Result Qmss_closeAccumulatorCh (Qmss_AccChHandle handle)
{
    uint32_t             subSys;
    Qmss_PdspId          pdspId;
    uint8_t              channel;
    Qmss_LocalObjParams *lObjPtr;

    if (handle == 0)
    {
        return QMSS_INVALID_PARAM;
    }

    Qmss_accBreakHandle ((int32_t)handle, &subSys, &pdspId, &channel);
    lObjPtr = &qmssLObj[subSys].p;

    if (lObjPtr->qmRmServiceHandle)

    {
        int32_t accumCh = channel;
        int32_t intd = lObjPtr->pdspIntdMap[pdspId];
        if (!Qmss_rmService((Rm_ServiceHandle *)lObjPtr->qmRmServiceHandle, Rm_service_RESOURCE_FREE, 
                            lObjPtr->rmAccumCh[intd], &accumCh, 1, 0, NULL))
        {
            return QMSS_RESOURCE_FREE_DENIED;
        }
    }

    return QMSS_ACC_SOK;
} /* Qmss_closeAccumulatorCh */

/**
 *  @b Description
 *  @n  
 *      This function disables the accumulator functionality for the specified channel
 *
 *      This function combines the behavior of @ref Qmss_stopAccumulatorCh() and 
 *      @ref Qmss_closeAccumulatorCh ()
 *
 *  @param[in]  handle
 *      Channel handle of channel to disable
 *
 *  @post  
 *      Channel is disabled
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_INVALID_PARAM
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND, QMSS_ACC_INVALID_CHANNEL, 
 *                  QMSS_ACC_CHANNEL_NOT_ACTIVE, QMSS_ACC_CHANNEL_ALREADY_ACTIVE, QMSS_ACC_INVALID_QUEUE_NUMBER
 */
Qmss_Result Qmss_stopAccumulatorCh (Qmss_AccChHandle handle)
{
    Qmss_AccCmd          cmd;
    volatile uint32_t    *cmdPtr, *reg;
    uint32_t             index;
    uint8_t              result;
    void                 *key;          
    uint32_t             subSys;
    Qmss_PdspId          pdspId;
    uint8_t              channel;
    Qmss_LocalObjParams *lObjPtr;
    Qmss_Result          retVal;

    if (handle == 0)
    {
        return QMSS_INVALID_PARAM;
    }

    Qmss_accBreakHandle ((int32_t)handle, &subSys, &pdspId, &channel);
    lObjPtr = &qmssLObj[subSys].p;

    if (! lObjPtr->regs.qmPdspCmdReg[pdspId])
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalAccCsEnter ();

    memset ((void *) &cmd, 0, sizeof (Qmss_AccCmd));
    CSL_FINSR (cmd.word0, 7, 0, channel);
    CSL_FINSR (cmd.word0, 15, 8, Qmss_AccCmd_DISABLE_CHANNEL);

    /* Point to the accumulator command register's last word */
    reg = (uint32_t *) ((uint8_t *) lObjPtr->regs.qmPdspCmdReg[pdspId] + 4 * 4);

    /* Write command word last */
    cmdPtr = ((uint32_t *) &cmd) + 4;

    for (index = 0; index < 5; index++)
        *reg-- = *cmdPtr--;

    /* Wait for the command to clear */
    reg++;
    do
    {
        result = CSL_FEXTR (*reg, 15, 8);  
    } while (result != 0);

    retVal = (Qmss_Result) (CSL_FEXTR (*reg, 31, 24));

    /* End Critical Section */
    Qmss_osalAccCsExit (key);
    
    return retVal;
} /* Qmss_stopAccumulatorCh */

/**
 *  @b Description
 *  @n  
 *      This function disables the accumulator functionality for the specified channel
 *
 *      This function combines the behavior of @ref Qmss_stopAccumulatorCh() and 
 *      @ref Qmss_closeAccumulatorCh ()
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      Accumulator to program. 
 *          Qmss_PdspId_PDSP1..8 selects firmware downloaded to PDSP 1 to 8
 *
 *  @param[in]  channel
 *      Channel to disable
 *
 *  @post  
 *      Channel is disabled and released from resource manager (if present)
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure - QMSS_INVALID_PARAM
 *  @retval
 *      Failure - QMSS_RESOURCE_FREE_DENIED
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND, QMSS_ACC_INVALID_CHANNEL, 
 *                  QMSS_ACC_CHANNEL_NOT_ACTIVE, QMSS_ACC_CHANNEL_ALREADY_ACTIVE, QMSS_ACC_INVALID_QUEUE_NUMBER
 */
Qmss_Result Qmss_disableAccumulatorSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint8_t channel)
{
    Qmss_Result         qResult;
    uint32_t            subSys   = (uint32_t)subSysHnd;
    Qmss_AccChHandle    chHandle = (Qmss_AccChHandle)Qmss_accMakeHandle (subSysHnd, pdspId, channel);

    /* For backwards compatibility */
    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    /* close is done first even though not that logical, since it
     * is used to guard the accumulator from unintended resource
     * conflicts.  
     *
     * When used in user-code, the Qmss_closeAccumulatorCh should be called
     * after the Qmss_stopAccumulatorCh, since stop will be guarded in 
     * resource manager by the Qmss_openAccumulatorCh.
     *
     */
    if ( (qResult = Qmss_closeAccumulatorCh (chHandle)) != QMSS_ACC_SOK)
    {
        return qResult;
    }

    return Qmss_stopAccumulatorCh (chHandle);
} /* Qmss_disableAccumulator */

/**
 *  @b Description
 *  @n
 *      Backwards compatibility wrapper of @ref Qmss_disableAccumulatorSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_disableAccumulator (Qmss_PdspId pdspId, uint8_t channel)
{
    return Qmss_disableAccumulatorSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, channel);
} /* Qmss_disableAccumulator */

/**
 *  @b Description
 *  @n  
 *      This function programs the timer constant used by the firmware to generate the timer tick.
 *
 *      The Accumulator time "tick" is controlled by a local timer connected to the PDSP core. 
 *      This timer has a programmable count based on the sub-system clock. When this count expires, 
 *      a local "tick" is registered in the firmware. The tick is used when timing channel interrupts 
 *      based on the "Timer Load Count" value supplied in the timer configuration.
 *
 *      The value of "Timer Constant" is the number of QM sub-system clocks divided by 2 that 
 *      comprise a single "tick" in the accumulator firmware.
 *
 *      For example, if the QM sub-system clock is 350MHz, and the desired firmware "tick" is 20us, 
 *      the proper Timer Constant for this command is computed as follows:
 *      
 *      Timer Constant = (350,000,000 cycles/sec) * (0.000020 sec) / (2 cycles)
 *      Timer Constant = 3,500
 *      
 *      The firmware initializes with a default constant value of 4375. This corresponds to firmware tick of 25us.
 * 
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      Accumulator to program
 *          Qmss_PdspId_PDSP1 selects firmware downloaded to PDSP 1
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *
 *  @param[in]  timerConstant
 *      16 bit wide value calculated as explained above
 *
 *  @post  
 *      Timer constant is programmed and in use.
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND   
 */

Qmss_Result Qmss_configureAccTimerSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint16_t timerConstant)
{

    volatile uint32_t   *reg;
    uint8_t             result;
    void                *key;
    uint32_t            subSys   = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;
    Qmss_Result         retVal;

    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    if (! lObjPtr->regs.qmPdspCmdReg[pdspId])
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalAccCsEnter ();

    /* Point to the accumulator command register's last word */
    reg = (uint32_t *) ((uint8_t *) lObjPtr->regs.qmPdspCmdReg[pdspId] + 4);
    *reg-- = timerConstant;
   
    *reg = Qmss_AccCmd_CONFIG_TIMER_CONSTANT << 8; 

    /* wait for the command to clear */
    do
    {
        result = CSL_FEXTR (*reg, 15, 8);  
    } while (result != 0);

    retVal = (Qmss_Result) (CSL_FEXTR (*reg, 31, 24));

    /* End Critical Section */
    Qmss_osalAccCsExit (key);

    return retVal;
} /* Qmss_configureAccTimerSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_configureAccTimerSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_configureAccTimer (Qmss_PdspId pdspId, uint16_t timerConstant)
{
    return Qmss_configureAccTimerSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, timerConstant);
} /* Qmss_configureAccTimer */

/**
 *  @b Description
 *  @n  
 *      
 *      The Accumulator firmware also supports an optional reclamation queue which can be used for 
 *      packet discards. Any descriptor placed on the reclamation queue will be recycled in the 
 *      same manner as if it had been submitted to CDMA. 
 *
 *      The descriptor packet information field is used to determine the return queue and the 
 *      return handling, including options to unlink host descriptors and push to either the 
 *      front or the back of the return queue.
 *      Setting queue to zero disables the reclamation feature
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      Accumulator to program
 *          Qmss_PdspId_PDSP1 selects firmware downloaded to PDSP 1
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *
 *  @param[in]  hnd
 *      Reclamation queue number
 *
 *  @post  
 *      Reclamation queue is configured.
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND
 */

Qmss_Result Qmss_programReclaimQueueSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QueueHnd hnd)
{
    volatile uint32_t   *reg;
    uint8_t             result;
    void                *key;
    uint32_t            subSys   = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr = &qmssLObj[subSys].p;
    uint32_t            qSubSys  = QMSS_QUEUE_SUBSYS(hnd);
    Qmss_Result         retVal;

    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    if (! lObjPtr->regs.qmPdspCmdReg[pdspId])
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    /* Ensure queue is in same subsystem as pdsp */
    if (qSubSys != subSys)
    {
        return QMSS_INVALID_PARAM;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalAccCsEnter ();

    /* Point to the accumulator command register's last word */
    reg = (uint32_t *) ((uint8_t *) lObjPtr->regs.qmPdspCmdReg[pdspId] + 4);
    *reg-- = hnd;
    *reg = Qmss_AccCmd_CONFIG_RECLAIM_QUEUE << 8;

    /* wait for the command to clear */
    do
    {
        result = CSL_FEXTR (*reg, 15, 8);  
    } while (result != 0);

    retVal = (Qmss_Result) (CSL_FEXTR (*reg, 31, 24));

    /* End Critical Section */
    Qmss_osalAccCsExit (key);

    return retVal;
} /* Qmss_programReclaimQueueSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_programReclaimQueueSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_programReclaimQueue (Qmss_PdspId pdspId, Qmss_QueueHnd hnd)
{
    return Qmss_programReclaimQueueSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, hnd);
} /* Qmss_programReclaimQueue */

/**
 *  @b Description
 *  @n  
 *      The Accumulator firmware supports an optional diversion queue which 
 *      can be used remotely cause a queue diversion. When enabled, any 
 *      descriptor placed on the diversion queue will be loaded, and its 
 *      Timestamp field will be written to the QM diversion register. 
 *      The descriptor pointer will then be pushed onto the diversion 
 *      completion queue.
 *
 *      Setting the diversion queue to 0 disables diversion queue 
 *      monitoring. The firmware initializes with a default diversion 
 *      queue of 0 (disabled).
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      Accumulator to program
 *          Qmss_PdspId_PDSP1 selects firmware downloaded to PDSP 1
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *
 *  @param[in]  divQ
 *      Diversion queue handle.  This is the queue which is monitored for 
 *      diversion requests
 *
 *  @param[in]  divCompletionQ
 *      Return queue handle.  After diversion request is completed, the
 *      descriptor is returned in this queue.
 *
 *  @post  
 *      Diversion queue is configured.
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure - QMSS_INVALID_PARAM
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND
 */

Qmss_Result Qmss_programDiversionQueueSubSys (Qmss_SubSysHnd subSysHnd,
                                              Qmss_PdspId pdspId, Qmss_QueueHnd divQ,
                                              Qmss_QueueHnd divCompletionQ)
{
    volatile uint32_t   *reg;
    uint8_t             result;
    void                *key;
    uint32_t            subSys         = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr       = &qmssLObj[subSys].p;
    uint32_t            divQSubSys     = QMSS_QUEUE_SUBSYS(divQ);
    uint32_t            divCompQSubSys = QMSS_QUEUE_SUBSYS(divQ);
    Qmss_Result         retVal;

    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    if (! lObjPtr->regs.qmPdspCmdReg[pdspId])
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    /* Make sure queue numbers fit in bitfields.  This doesn't check if the
     * queue number exists in the system */
    if ((divQ > 0xffff) || (divCompletionQ > 0xffff)) {
        return QMSS_INVALID_PARAM;
    }
    
    /* Make sure all components are in same subsystem */
    if ((divQSubSys != subSys) || (divCompQSubSys != subSys))
    {
        return QMSS_INVALID_PARAM;
    }

    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalAccCsEnter ();

    /* Point to the accumulator command register's last word */
    reg = (uint32_t *) ((uint8_t *) lObjPtr->regs.qmPdspCmdReg[pdspId] + 4);
    *reg-- = (divCompletionQ  << 16) | divQ;
    *reg = Qmss_AccCmd_CONFIG_DIVERSION_QUEUE << 8;

    /* wait for the command to clear */
    do
    {
        result = CSL_FEXTR (*reg, 15, 8);  
    } while (result != 0);

    retVal = (Qmss_Result) (CSL_FEXTR (*reg, 31, 24));

    /* End Critical Section */
    Qmss_osalAccCsExit (key);

    return retVal;
} /* Qmss_programDiversionQueueSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_programDiversionQueueSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_programDiversionQueue (Qmss_PdspId pdspId, Qmss_QueueHnd divQ,
                                        Qmss_QueueHnd divCompletionQ)
{
    return Qmss_programDiversionQueueSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, divQ, divCompletionQ);
} /* Qmss_programDiversionQueue */

/* These only apply to K2 devices */
#if defined(DEVICE_K2H) || defined(SOC_K2H) || \
    defined(DEVICE_K2K) || defined(SOC_K2K) || \
    defined(DEVICE_K2L) || defined(SOC_K2L) || \
    defined(DEVICE_K2E) || defined(SOC_K2E)
/**
 *  @b Description
 *  @n  
 *      The Accumulator firmware supports an optional DDR barrier queue which 
 *      can be used on K2 for a coherence barrier.
 *      When enabled, any descriptor placed on the DDR barrier queue will be 
 *      loaded, and it will be placed in queue specified in its destination
 *      tag field.
 *      This causes a coherence barrier assuming all descriptors and buffers
 *      are in shareable DDR by reading forward queue from the source tag.
 *
 *      Setting the DDR barrier queue to 0 disables DDR barrier queue 
 *      monitoring. The firmware initializes with a default DDR barrier 
 *      queue of 0 (disabled).
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      Accumulator to program
 *          Qmss_PdspId_PDSP1 selects firmware downloaded to PDSP 1
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *
 *  @param[in]  ddrInfBarrierQ
 *      DDR barrier queue handle.  This is the queue which is monitored for 
 *      barrier requests with return queue in destTag.  Logically this is
 *      for "Inf"rastructure DMA.
 *
 *  @param[in]  ddrNetCPBarrierQ
 *      DDR barrier queue handle.  This is the queue which is monitored for 
 *      barrier requests with return queue in LSBs of swInfo[1].  Logically
 *      this is for traffic from NETCP
 *
 *  @post  
 *      DDR Barrier queue is configured.
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure - QMSS_INVALID_PARAM
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND
 */

Qmss_Result Qmss_programDDRBarrierQueueSubSys (Qmss_SubSysHnd subSysHnd,
                                               Qmss_PdspId pdspId, 
                                               Qmss_QueueHnd ddrInfBarrierQ,
                                               Qmss_QueueHnd ddrNetCPBarrierQ)
{
    volatile uint32_t   *reg;
    uint8_t             result;
    void                *key;
    uint32_t            subSys         = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr       = &qmssLObj[subSys].p;
    Qmss_Result         retVal;

    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    if (! lObjPtr->regs.qmPdspCmdReg[pdspId])
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    /* Make sure queue numbers fit in bitfields.  This doesn't check if the
     * queue number exists in the system */
    if ((ddrInfBarrierQ > 0xffffu) || (ddrNetCPBarrierQ > 0xffffu)) {
        return QMSS_INVALID_PARAM;
    }
    
    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalAccCsEnter ();

    /* Point to the accumulator command register's last word */
    reg = (uint32_t *) ((uint8_t *) lObjPtr->regs.qmPdspCmdReg[pdspId] + 4);
    *reg-- = (ddrNetCPBarrierQ << 16) | ddrInfBarrierQ;
    *reg = Qmss_AccCmd_CONFIG_DDR_BARRIER_QUEUE << 8;

    /* wait for the command to clear */
    do
    {
        result = CSL_FEXTR (*reg, 15, 8);  
    } while (result != 0);

    retVal = (Qmss_Result) (CSL_FEXTR (*reg, 31, 24));

    /* End Critical Section */
    Qmss_osalAccCsExit (key);

    return retVal;
} /* Qmss_programDDRBarrierQueueSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_programDDRBarrierQueueSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_programDDRBarrierQueue (Qmss_PdspId pdspId, 
                                         Qmss_QueueHnd ddrInfBarrierQ,
					 Qmss_QueueHnd ddrNetCPBarrierQ)
{
    return Qmss_programDDRBarrierQueueSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, ddrInfBarrierQ, ddrNetCPBarrierQ);
} /* Qmss_programDDRBarrierQueue */

/**
 *  @b Description
 *  @n  
 *      The Accumulator firmware supports an optional MSMC barrier queue which 
 *      can be used on K2 for a coherence barrier.
 *      When enabled, any descriptor placed on the MSMC barrier queue will be 
 *      loaded, and it will be placed in queue specified in its destination
 *      tag field.
 *
 *      This causes a coherence barrier assuming all descriptors and buffers
 *      are in shareable DDR or MSMC by generating extra read of all 8 
 *      MSMC banks and shareable DDR.
 *
 *      Setting the MSMC barrier queue to 0 disables MSMC barrier queue 
 *      monitoring. The firmware initializes with a default MSMC barrier 
 *      queue of 0 (disabled).
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      Accumulator to program
 *          Qmss_PdspId_PDSP1 selects firmware downloaded to PDSP 1
 *          Qmss_PdspId_PDSP2 selects firmware downloaded to PDSP 2
 *
 *  @param[in]  msmcInfBarrierQ
 *      MSMC barrier queue handle.  This is the queue which is monitored for 
 *      barrier requests with return queue in destTag.  Logically this is
 *      for "Inf"rastructure DMA.
 *
 *  @param[in]  msmcNetCPBarrierQ
 *      DDR barrier queue handle.  This is the queue which is monitored for 
 *      barrier requests with return queue in LSBs of swInfo[1].  Logically
 *      this is for traffic from NETCP
 *
 *  @post  
 *      MSMC Barrier queue is configured.
 *
 *  @retval
 *      Value reported by accumulator
 *  @retval
 *      Success - QMSS_ACC_SOK
 *  @retval
 *      Failure - QMSS_NOT_INITIALIZED
 *  @retval
 *      Failure - QMSS_INVALID_PARAM
 *  @retval
 *      Failure - Accumulator status - QMSS_ACC_IDLE, QMSS_ACC_INVALID_COMMAND
 */
Qmss_Result Qmss_programMSMCBarrierQueueSubSys (Qmss_SubSysHnd subSysHnd,
                                                Qmss_PdspId pdspId, 
                                                Qmss_QueueHnd msmcInfBarrierQ,
						Qmss_QueueHnd msmcNetCPBarrierQ)
{
    volatile uint32_t   *reg;
    uint8_t             result;
    void                *key;
    uint32_t            subSys         = (uint32_t)subSysHnd;
    Qmss_LocalObjParams *lObjPtr       = &qmssLObj[subSys].p;
    Qmss_Result         retVal;

    if (qmssLObjIsValid[subSys] == 0)
    {
        return QMSS_NOT_INITIALIZED;
    }

    if (! lObjPtr->regs.qmPdspCmdReg[pdspId])
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    /* Make sure queue numbers fit in bitfields.  This doesn't check if the
     * queue number exists in the system */
    if ((msmcInfBarrierQ > 0xffffu) || (msmcNetCPBarrierQ > 0xffffu)) {
        return QMSS_INVALID_PARAM;
    }
    
    /* Begin Critical Section before accessing shared resources. */
    key = Qmss_osalAccCsEnter ();

    /* Point to the accumulator command register's last word */
    reg = (uint32_t *) ((uint8_t *) lObjPtr->regs.qmPdspCmdReg[pdspId] + 8);
    /* Base address of DDR after MPAX is 0x80000000, and base address of MSMC
     * after MPAX is = 0x0c000000 */
    *reg-- = (0x8000u << 16) | (0x0c00u);
    *reg-- = (msmcNetCPBarrierQ << 16) | msmcInfBarrierQ;
    *reg = Qmss_AccCmd_CONFIG_MSMC_BARRIER_QUEUE << 8;

    /* wait for the command to clear */
    do
    {
        result = CSL_FEXTR (*reg, 15, 8);  
    } while (result != 0);

    retVal = (Qmss_Result) (CSL_FEXTR (*reg, 31, 24));

    /* End Critical Section */
    Qmss_osalAccCsExit (key);

    return retVal;
} /* Qmss_programMSMCBarrierQueueSubSys */

/**
 *  @b Description
 *  @n  
 *      Backwards compatibility wrapper of @ref Qmss_programMSMCBarrierQueueSubSys
 *      which operates on the global subsystem.
 */
Qmss_Result Qmss_programMSMCBarrierQueue (Qmss_PdspId pdspId, 
                                          Qmss_QueueHnd msmcInfBarrierQ,
                                          Qmss_QueueHnd msmcNetCPBarrierQ)
{
    return Qmss_programMSMCBarrierQueueSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, msmcInfBarrierQ, msmcNetCPBarrierQ);
} /* Qmss_programMSMCBarrierQueue */
#endif /* K2 devices only */

/**
@}
*/
