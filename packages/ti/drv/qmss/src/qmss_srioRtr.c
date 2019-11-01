/**
 *   @file  qmss_srioRtr.c
 *
 *   @brief   
 *      This file has API that configure the Queue Manager 
 *      SRIO router PDSP firmware.
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

/* QMSS Types includes */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_srioRtr.h>
#include <ti/drv/qmss/include/qmss_pvt.h>

/* QMSS OSAL layer */
#include <qmss_osal.h>

/* CSL RL includes */
#include <ti/drv/qmss/include/qmss_srioRtr_regs.h>
      
/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/**********************************************************************
 ************************** Functions *********************************
 **********************************************************************/
/* Private function to create hw queue ID for srio router Firmware */
static inline void qmss_sriortr_queuehnd_to_bitfield (volatile uint32_t *reg, Qmss_QueueHnd queue)
{
  CSL_FINS (*reg, QMSS_SRIORTR_QUEUECFG_OUTPUT_QUEUES_QUEUE, QMSS_QUEUE_QID(queue));
} /* qmss_convert_qmss_queue_to_bitfield */

/* Private function to create handle from hw queue ID */
static inline Qmss_QueueHnd qmss_sriortr_bitfield_to_queuehnd (uint32_t subSys, volatile uint32_t *reg)
{
  return QMSS_QUEUE_HNDL(subSys, CSL_FEXT(*reg, QMSS_SRIORTR_QUEUECFG_OUTPUT_QUEUES_QUEUE));
} /* qmss_convert_qmss_queue_to_bitfield */

/* Private function to get register pointer */
static Qmss_SrioRtrRegs *qmss_sriortr_get_regptr (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId)
{
    uint32_t subSys = (uint32_t)subSysHnd;

    if (pdspId >= QMSS_MAX_PDSP)
    {
        return NULL;
    }
    if (subSys >= QMSS_MAX_SUBSYS)
    {
        return NULL;
    }
    return (Qmss_SrioRtrRegs *) qmssLObj[subSys].p.regs.qmPdspCmdReg[pdspId];
} /* qmss_srioRtr_get_regptr */

static uint32_t qmss_sriortr_get_magic (Qmss_SrioRtrRegs *regPtr)
{
    if (regPtr)
    {
        return CSL_FEXT(regPtr->MAGIC, QMSS_SRIORTR_MAGIC);
    }

    return 0xffffffff;
} /* qmss_sriortr_get_magic */

static Qmss_Result qmss_sriortr_check_inputs (Qmss_SrioRtrRegs *regPtr)
{
    uint32_t magic;

    if (! regPtr)
    {
        return QMSS_SUBSYS_UNSUPPORTED;
    }

    magic = qmss_sriortr_get_magic (regPtr);
    magic = CSL_FEXT (magic, QMSS_SRIORTR_MAGIC_MAGIC);

    if (magic != QMSS_SRIORTR_MAGIC)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_MAGIC;
    }

    return QMSS_SRIORTR_RETCODE_SUCCESS;
} /* qmss_sriortr_check_inputs */

/** @addtogroup QMSS_LLD_FUNCTION
@{ 
*/

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version information of the current
 *      sriortr firmware running on the PDSP.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to query
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @retval
 *      Version Information, or 0xffffffff if error
 */
uint32_t Qmss_getSrioRtrFwVersionSubSys      (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);

    if (! regPtr)
    {
        return 0xffffffff;
    }

    return CSL_FEXT(regPtr->VERSION, QMSS_SRIORTR_VERSION);
} /* Qmss_getSrioRtrFwVersionSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_getSrioRtrFwVersionSubSys on global subsystem.
 */
uint32_t Qmss_getSrioRtrFwVersion            (Qmss_PdspId pdspId)
{
    return Qmss_getSrioRtrFwVersionSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId);
} /* Qmss_getSrioRtrFwVersion */

/**
 *  @b Description
 *  @n  
 *      The function is used to get the version information of the current
 *      sriortr firmware running on the PDSP.
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to query
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @retval
 *      Magic number, or 0xffffffff if error
 */
uint32_t Qmss_getSrioRtrFwMagicSubSys        (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);

    return qmss_sriortr_get_magic (regPtr);
} /* Qmss_getSrioRtrFwMagicSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_getSrioRtrFwMagicSubSys on global subsystem.
 */
uint32_t Qmss_getSrioRtrFwMagic              (Qmss_PdspId pdspId)
{
    return Qmss_getSrioRtrFwMagicSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId);
} /* Qmss_getSrioRtrFwMagic */

/* Internal function to disable ports and return previous state */
static uint32_t qmss_sriortr_disable_ports (Qmss_SrioRtrRegs *regPtr, uint32_t ports)
{
    uint32_t retVal = regPtr->ENABLES.portEnabled & ports;

    /* Disable requested ports */
    CSL_FINS(regPtr->ENABLES.portEnables, QMSS_SRIORTR_ENABLES_PORTENABLES_SET,
             CSL_FEXT(regPtr->ENABLES.portEnables, QMSS_SRIORTR_ENABLES_PORTENABLES_SET) & ~ports);
    /* Wait for it to take effect */
    while (CSL_FEXT(regPtr->ENABLES.portEnabled, QMSS_SRIORTR_ENABLES_PORTENABLED_STAT) & ports);

    return retVal;
} /* qmss_sriortr_enable_ports */

/**
 *  @b Description
 *  @n  
 *      This disables processing of port(s).  This enables reconfiguraton of those
 *      port(s).
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] ports
 *      bitfield indicating which port(s) to disable.  A value of 1 means
 *      disable the port; a value of 0 means ignore the port.
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORTS : more than 4 ports specified
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_disableSrioRtrPortsSubSys   (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t ports)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    Qmss_Result       retVal;
    void             *key;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if (ports & ~0xf)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_PORTS;
    }

    key = Qmss_osalCsEnter();

    /* Intentional discard return (ports previously enabled) for this use case */
    (void)qmss_sriortr_disable_ports (regPtr, ports);

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;    
} /* Qmss_disableSrioRtrPortsSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_disableSrioRtrPortsSubSys on global subsystem.
 */
Qmss_Result Qmss_disableSrioRtrPorts         (Qmss_PdspId pdspId, uint32_t ports)
{
    return Qmss_disableSrioRtrPortsSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, ports);
} /* Qmss_disableSrioRtrPorts */

/* Internal function to enable ports */
static void qmss_sriortr_enable_ports (Qmss_SrioRtrRegs *regPtr, uint32_t ports)
{
    /* Enable requested ports */
    CSL_FINS(regPtr->ENABLES.portEnables, QMSS_SRIORTR_ENABLES_PORTENABLES_SET,
             CSL_FEXT(regPtr->ENABLES.portEnables, QMSS_SRIORTR_ENABLES_PORTENABLES_SET) | ports);
    /* Wait for it to take effect */
    while (CSL_FEXT(regPtr->ENABLES.portEnabled, QMSS_SRIORTR_ENABLES_PORTENABLED_STAT) != ports);
} /* qmss_sriortr_enable_ports */

/**
 *  @b Description
 *  @n  
 *      This enables processing of port(s).  
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] ports
 *      bitfield indicating which port(s) to enable.  A value of 1 means
 *      enable the port; a value of 0 means ignore the port.
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORTS : more than 4 ports specified
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_enableSrioRtrPortsSubSys    (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t ports)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    Qmss_Result       retVal;
    void             *key;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if (ports & ~0xf)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_PORTS;
    }

    key = Qmss_osalCsEnter();

    qmss_sriortr_enable_ports (regPtr, ports);

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;    
} /* Qmss_enableSrioRtrPortsSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_enableSrioRtrPortsSubSys on global subsystem.
 */
Qmss_Result Qmss_enableSrioRtrPorts          (Qmss_PdspId pdspId, uint32_t ports)
{
    return Qmss_enableSrioRtrPortsSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, ports);
} /* Qmss_enableSrioRtrPorts */

/**
 *  @b Description
 *  @n  
 *      This reads back the global configuration
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] stop
 *      if true, stop the ports automatically before reading
 *
 *  @param[out] cfg
 *      returned configuration
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORT  : invalid port #
 *      QMSS_SRIORTR_RETCODE_INVALID_CFG   : cfg is NULL
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_getSrioRtrGblCfgSubSys      (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int stop, Qmss_SrioRtrGblCfg *cfg)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    uint32_t          subSys = (uint32_t)subSysHnd;
    Qmss_Result       retVal;
    void             *key;
    uint32_t          status;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if (! cfg)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_CFG;
    }

    key = Qmss_osalCsEnter();

    if (stop)
    {
        status = qmss_sriortr_disable_ports (regPtr, 0xf);
    }

    cfg->creditSize = CSL_FEXT (regPtr->QUEUE_CFG.CREDITS, QMSS_SRIORTR_QUEUECFG_CREDITS);
    cfg->queueBase = qmss_sriortr_bitfield_to_queuehnd (subSys, &regPtr->QUEUE_CFG.SRIORTR_INPUT_QUEUEBASE);

    if (stop && status)
    {
        qmss_sriortr_enable_ports (regPtr, status);
    }

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;
} /* Qmss_getSrioRtrGblCfgSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_getSrioRtrGblCfgSubSys on global subsystem.
 */
Qmss_Result Qmss_getSrioRtrGblCfg            (Qmss_PdspId pdspId, int stop, Qmss_SrioRtrGblCfg *cfg)
{
    return Qmss_getSrioRtrGblCfgSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, stop, cfg);
} /* Qmss_getSrioRtrGblCfg */

/**
 *  @b Description
 *  @n  
 *      This sets the global configuration
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] stop
 *      if true, stop the ports automatically before configuring
 *
 *  @param[in] cfg
 *      configuration
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORT  : invalid port #
 *      QMSS_SRIORTR_RETCODE_INVALID_CFG   : cfg is NULL
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_setSrioRtrGblCfgSubSys      (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int stop, Qmss_SrioRtrGblCfg *cfg)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    Qmss_Result       retVal;
    void             *key;
    uint32_t          status;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if (! cfg)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_CFG;
    }

    if ( (QMSS_QUEUE_QID(cfg->queueBase) / 32) != (QMSS_QUEUE_QID(cfg->queueBase) + 11) / 32)
    {
        /* The queueBase straddles a 32 queue boundary */
        return QMSS_SRIORTR_RETCODE_INVALID_CFG;
    }

    key = Qmss_osalCsEnter();

    if (stop)
    {
        status = qmss_sriortr_disable_ports (regPtr, 0xf);
    }

    CSL_FINS (regPtr->QUEUE_CFG.CREDITS, QMSS_SRIORTR_QUEUECFG_CREDITS, cfg->creditSize);
    qmss_sriortr_queuehnd_to_bitfield (&regPtr->QUEUE_CFG.SRIORTR_INPUT_QUEUEBASE, cfg->queueBase);

    if (stop && status)
    {
        qmss_sriortr_enable_ports (regPtr, status);
    }

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;
} /* Qmss_setSrioRtrGblCfgSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_setSrioRtrGblCfgSubSys on global subsystem.
 */
Qmss_Result Qmss_setSrioRtrGblCfg            (Qmss_PdspId pdspId, int stop, Qmss_SrioRtrGblCfg *cfg)
{
    return Qmss_setSrioRtrGblCfgSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, stop, cfg);
} /* Qmss_setSrioRtrGblCfg */

/**
 *  @b Description
 *  @n  
 *      This reads back a ports configuration
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] portNum
 *      port to read
 *
 *  @param[in] stop
 *      if true, stop the ports automatically before reading
 *
 *  @param[out] cfg
 *      returned configuration
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORT  : invalid port #
 *      QMSS_SRIORTR_RETCODE_INVALID_CFG   : cfg is NULL
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_getSrioRtrPortCfgSubSys     (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int portNum, int stop, Qmss_SrioRtrPortCfg *cfg)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    uint32_t          subSys = (uint32_t)subSysHnd;
    Qmss_Result       retVal;
    void             *key;
    int               i;
    uint32_t          status;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if (! cfg)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_CFG;
    }

    if ((portNum < 0) || (portNum >= 4))
    {
        return QMSS_SRIORTR_RETCODE_INVALID_PORT;
    }

    key = Qmss_osalCsEnter();

    if (stop)
    {
        status = qmss_sriortr_disable_ports (regPtr, 1 << portNum);
    }

    cfg->hostSrioTxQueue     = qmss_sriortr_bitfield_to_queuehnd (subSys, &regPtr->QUEUE_CFG.SRIORTR_OUTPUT_QUEUES[4][portNum]);
    cfg->creditSrioTxQueue   = qmss_sriortr_bitfield_to_queuehnd (subSys, &regPtr->QUEUE_CFG.SRIORTR_OUTPUT_QUEUES[5][portNum]);
    cfg->creditReturnQueue   = qmss_sriortr_bitfield_to_queuehnd (subSys, &regPtr->QUEUE_CFG.SRIORTR_CREDIT_RETURN_QUEUES[portNum]);
    cfg->creditSourceQueue   = qmss_sriortr_bitfield_to_queuehnd (subSys, &regPtr->QUEUE_CFG.SRIORTR_CREDIT_SOURCE_QUEUES[portNum]);
    cfg->fwdTxFinCmpQueue    = qmss_sriortr_bitfield_to_queuehnd (subSys, &regPtr->QUEUE_CFG.SRIORTR_FWD_RX_FREE_QUEUES[portNum]);
    cfg->destID              = CSL_FEXT (regPtr->DESTID.DEST_ID[portNum], QMSS_SRIORTR_DEST_ID_DEST);
    for (i = 0; i < 4; i++)
    {
        cfg->sourcePortSrioTxQueue[i] = qmss_sriortr_bitfield_to_queuehnd (subSys, &regPtr->QUEUE_CFG.SRIORTR_OUTPUT_QUEUES[portNum][i]);
    }

    if (stop && status)
    {
        qmss_sriortr_enable_ports (regPtr, 1 << portNum);
    }

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;
} /* Qmss_getSrioRtrPortCfgSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_getSrioRtrPortCfgSubSys on global subsystem.
 */
Qmss_Result Qmss_getSrioRtrPortCfg           (Qmss_PdspId pdspId, int portNum, int stop, Qmss_SrioRtrPortCfg *cfg)
{
    return Qmss_getSrioRtrPortCfgSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, portNum, stop, cfg);
} /* Qmss_getSrioRtrPortCfg */

/**
 *  @b Description
 *  @n  
 *      This sets a ports configuration
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] portNum
 *      port to configure
 *
 *  @param[in] stop
 *      if true, stop the port automatically before configuring
 *
 *  @param[in] cfg
 *      configuration
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORT  : invalid port #
 *      QMSS_SRIORTR_RETCODE_INVALID_CFG   : cfg is NULL
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_setSrioRtrPortCfgSubSys     (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int portNum, int stop, Qmss_SrioRtrPortCfg *cfg)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    Qmss_Result       retVal;
    void             *key;
    int               i;
    uint32_t          status;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if (! cfg)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_CFG;
    }

    if ((portNum < 0) || (portNum >= 4))
    {
        return QMSS_SRIORTR_RETCODE_INVALID_PORT;
    }

    key = Qmss_osalCsEnter();

    if (stop)
    {
        status = qmss_sriortr_disable_ports (regPtr, 1 << portNum);
    }

    qmss_sriortr_queuehnd_to_bitfield (&regPtr->QUEUE_CFG.SRIORTR_OUTPUT_QUEUES[4][portNum]    , cfg->hostSrioTxQueue);
    qmss_sriortr_queuehnd_to_bitfield (&regPtr->QUEUE_CFG.SRIORTR_OUTPUT_QUEUES[5][portNum]    , cfg->creditSrioTxQueue);
    qmss_sriortr_queuehnd_to_bitfield (&regPtr->QUEUE_CFG.SRIORTR_CREDIT_RETURN_QUEUES[portNum], cfg->creditReturnQueue);
    qmss_sriortr_queuehnd_to_bitfield (&regPtr->QUEUE_CFG.SRIORTR_CREDIT_SOURCE_QUEUES[portNum], cfg->creditSourceQueue);
    qmss_sriortr_queuehnd_to_bitfield (&regPtr->QUEUE_CFG.SRIORTR_FWD_RX_FREE_QUEUES[portNum]  , cfg->fwdTxFinCmpQueue);
    CSL_FINS (regPtr->DESTID.DEST_ID[portNum], QMSS_SRIORTR_DEST_ID_DEST, cfg->destID);
    for (i = 0; i < 4; i++)
    {
        qmss_sriortr_queuehnd_to_bitfield (&regPtr->QUEUE_CFG.SRIORTR_OUTPUT_QUEUES[portNum][i], cfg->sourcePortSrioTxQueue[i]);
    }

    if (stop && status)
    {
        qmss_sriortr_enable_ports (regPtr, 1 << portNum);
    }

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;
} /* Qmss_setSrioRtrPortCfgSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_setSrioRtrPortCfgSubSys global subsystem.
 */
Qmss_Result Qmss_setSrioRtrPortCfg           (Qmss_PdspId pdspId, int portNum, int stop, Qmss_SrioRtrPortCfg *cfg)
{
    return Qmss_setSrioRtrPortCfgSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, portNum, stop, cfg);
} /* Qmss_setSrioRtrPortCfg */

/**
 *  @b Description
 *  @n  
 *      This returns a ports configuration
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] portNum
 *      port to read
 *
 *  @param[in] reset
 *      if true, reset the stats after reading.  Recommended to use together with "stop"
 *
 *  @param[in] stop
 *      if true, stop the port automatically before configuring (but this is expensive)
 *
 *  @param[out] stats
 *      stats results.  Can be NULL to do reset without returning values.
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORT  : invalid port #
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_getSrioRtrStatsSubSys       (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int portNum, int reset, int stop, Qmss_SrioRtrStats *stats)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    Qmss_Result       retVal;
    void             *key;
    uint32_t          status;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if ((portNum < 0) || (portNum >= 4))
    {
        return QMSS_SRIORTR_RETCODE_INVALID_PORT;
    }

    key = Qmss_osalCsEnter();

    if (stop)
    {
        status = qmss_sriortr_disable_ports (regPtr, 1 << portNum);
    }

    if (stats) 
    {
        stats->packetsRx = CSL_FEXT (regPtr->STATS.PACKETS_RX[portNum], QMSS_SRIORTR_STATS_PACKETS_RX);
        stats->packetsTx = CSL_FEXT (regPtr->STATS.PACKETS_TX[portNum], QMSS_SRIORTR_STATS_PACKETS_TX);
        stats->creditsRx = CSL_FEXT (regPtr->STATS.CREDITS_RX[portNum], QMSS_SRIORTR_STATS_CREDITS_RX);
        stats->creditsTx = CSL_FEXT (regPtr->STATS.CREDITS_TX[portNum], QMSS_SRIORTR_STATS_CREDITS_TX);
        stats->fwdRets   = CSL_FEXT (regPtr->STATS.FWD_RETS  [portNum], QMSS_SRIORTR_STATS_FWD_RETS);
        stats->hostPkts  = CSL_FEXT (regPtr->STATS.HOST_RX   [portNum], QMSS_SRIORTR_STATS_HOST_RX);
    }

    if (reset)
    {
        CSL_FINS (regPtr->STATS.PACKETS_RX[portNum], QMSS_SRIORTR_STATS_PACKETS_RX, 0);
        CSL_FINS (regPtr->STATS.PACKETS_TX[portNum], QMSS_SRIORTR_STATS_PACKETS_TX, 0);
        CSL_FINS (regPtr->STATS.CREDITS_RX[portNum], QMSS_SRIORTR_STATS_CREDITS_RX, 0);
        CSL_FINS (regPtr->STATS.CREDITS_TX[portNum], QMSS_SRIORTR_STATS_CREDITS_TX, 0);
        CSL_FINS (regPtr->STATS.FWD_RETS  [portNum], QMSS_SRIORTR_STATS_FWD_RETS  , 0);
        CSL_FINS (regPtr->STATS.HOST_RX   [portNum], QMSS_SRIORTR_STATS_HOST_RX   , 0);
    }

    if (stop && status)
    {
        qmss_sriortr_enable_ports (regPtr, 1 << portNum);
    }

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;
} /* Qmss_getSrioRtrStatsSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_getSrioRtrStatsSubSys on global subsystem.
 */
Qmss_Result Qmss_getSrioRtrStats             (Qmss_PdspId pdspId, int portNum, int reset, int stop, Qmss_SrioRtrStats *stats)
{
    return Qmss_getSrioRtrStatsSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, portNum, reset, stop, stats);
} /* Qmss_getSrioRtrStats */

/**
 *  @b Description
 *  @n  
 *      This returns the global routing table
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] stop
 *      if true, stop the all ports automatically before configuring
 *
 *  @param[out] tbl
 *      table results
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORT  : invalid port #
 *      QMSS_SRIORTR_RETCODE_INVALID_CFG   : tbl is NULL
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_getSrioRtrGblRouteTblSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int stop, Qmss_SrioRtrRouteTbl *tbl)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    Qmss_Result       retVal;
    void             *key;
    int               i, j, ent;
    uint32_t          status;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if (! tbl)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_CFG;
    }

    key = Qmss_osalCsEnter();

    if (stop)
    {
        status = qmss_sriortr_disable_ports (regPtr, 0xf);
    }

    for (i = 0, ent = 0; i < 32; i++)
    {
        uint32_t word = CSL_FEXT (regPtr->ROUTETBL.ROUTE_TBL[i], QMSS_SRIORTR_ROUTE_TBL_8_DESTS);
        for (j = 0; j < 8; j++, ent++)
        {
            uint32_t thisEnt = word >> (j*4);
            tbl->forwardToPort[ent] = CSL_FEXT (thisEnt, QMSS_SRIORTR_ROUTE_TBL_1_DEST_PORT);
            tbl->isFinalDest[ent]   = CSL_FEXT (thisEnt, QMSS_SRIORTR_ROUTE_TBL_1_DEST_FINDEST);
        }
    }

    if (stop && status)
    {
        qmss_sriortr_enable_ports (regPtr, status);
    }

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;
} /* Qmss_getSrioRtrGblRouteTblSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_setSrioRtrGblRouteTblSubSys on global subsystem.
 */
Qmss_Result Qmss_getSrioRtrGblRouteTbl       (Qmss_PdspId pdspId, int stop, Qmss_SrioRtrRouteTbl *tbl)
{
    return Qmss_getSrioRtrGblRouteTblSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, stop, tbl);
} /* Qmss_getSrioRtrGblRouteTbl */

/**
 *  @b Description
 *  @n  
 *      This configures the global routing table
 *
 *  @param[in]  subSysHnd
 *      Handle to specific subsystem returned by @ref Qmss_initSubSys or
 *      @ref Qmss_startSubSysCfg.
 *
 *  @param[in]  pdspId
 *      sriortr PDSP to control
 *          Qmss_PdspId_PDSP# selects firmware downloaded to PDSP #
 *
 *  @param[in] stop
 *      if true, stop the all ports automatically before configuring
 *
 *  @param[in] tbl
 *      table to set
 *
 *  @retval
 *      QMSS_SRIORTR_RETCODE_SUCCESS       : worked
 *      QMSS_SRIORTR_RETCODE_INVALID_MAGIC : wrong/no firmware found on PDSP
 *      QMSS_SRIORTR_RETCODE_INVALID_PORT  : invalid port #
 *      QMSS_SRIORTR_RETCODE_INVALID_CFG   : tbl is NULL
 *      QMSS_SUBSYS_UNSUPPORTED            : subsystem doesn't have pdsps
 */
Qmss_Result Qmss_setSrioRtrGblRouteTblSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int stop, Qmss_SrioRtrRouteTbl *tbl)
{
    Qmss_SrioRtrRegs *regPtr = qmss_sriortr_get_regptr (subSysHnd, pdspId);
    Qmss_Result       retVal;
    void             *key;
    int               i, j, ent;
    uint32_t          status;

    retVal = qmss_sriortr_check_inputs (regPtr);

    if (retVal != QMSS_SRIORTR_RETCODE_SUCCESS)
    {
        return retVal;
    }

    if (! tbl)
    {
        return QMSS_SRIORTR_RETCODE_INVALID_CFG;
    }

    key = Qmss_osalCsEnter();

    if (stop)
    {
        status = qmss_sriortr_disable_ports (regPtr, 0xf);
    }

    for (i = 0, ent = 0; i < 32; i++)
    {
        uint32_t word = 0;
        for (j = 0; j < 8; j++, ent++)
        {
            uint32_t thisEnt = 0;
            CSL_FINS (thisEnt, QMSS_SRIORTR_ROUTE_TBL_1_DEST_PORT, tbl->forwardToPort[ent] & 0x3);
            CSL_FINS (thisEnt, QMSS_SRIORTR_ROUTE_TBL_1_DEST_FINDEST, tbl->isFinalDest[ent] & 0x1);
            word |= thisEnt << (j*4);
        }
        CSL_FINS (regPtr->ROUTETBL.ROUTE_TBL[i], QMSS_SRIORTR_ROUTE_TBL_8_DESTS, word);
    }

    if (stop && status)
    {
        qmss_sriortr_enable_ports (regPtr, status);
    }

    Qmss_osalCsExit(key);

    return QMSS_SRIORTR_RETCODE_SUCCESS;
} /* Qmss_setSrioRtrGblRouteTblSubSys */

/**
 *  @b Description
 *  @n  
 *      Runs @ref Qmss_setSrioRtrGblRouteTblSubSys on global subsystem.
 */
Qmss_Result Qmss_setSrioRtrGblRouteTbl       (Qmss_PdspId pdspId, int stop, Qmss_SrioRtrRouteTbl *tbl)
{
    return Qmss_setSrioRtrGblRouteTblSubSys (QMSS_SUBSYS_HND_GLOBAL, pdspId, stop, tbl);
} /* Qmss_setSrioRtrGblRouteTbl */


/**
@}
*/

