/******************************************************************************
 * FILE PURPOSE: Defines and Include files for all NetCP related defines and
 *               includes for NWAL module
 ******************************************************************************
 * FILE NAME:   nwal_netcp.h
 *
 * DESCRIPTION: Defines and include files for all NetCP related dependent
 *              components and defines
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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
 */
/* ============================================================= */
/**
 *   @file  nwal_netcp.h
 *
 *   path  ti/drv/nwal/nwal_netcp.h
 *
 *   @brief  Defines and Include files for all NetCP related defines and
 *          includes for NWAL module
 *
 *
 */

/**
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2010-2012 Texas Instruments, Inc.
 *  \par
 */


#ifndef _NWAL_NETCP_H
#define _NWAL_NETCP_H
#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pasahost.h>
#include <ti/drv/pa/nss_if.h>
#include <ti/drv/pa/nss_cfg.h>
#include <ti/runtime/pktlib/pktlib.h>
#ifdef NWAL_ENABLE_SA
#include <ti/drv/sa/salld.h>
#endif


/**
 *  @ingroup nwal_api_constants
 *  @{
 *
 *  @name   Threshold for Maximum RX packet to be received in one call back
 *  @brief  Threshold for Maximum RX packet to be received in one call back
 *
 *  @details Threshold for Maximum RX packet to be received in one call back
 */
/* @{ */
/**
 *  @def  NWAL_queueHnd
 *        QMSS Queue Handle for NetCP devices
 */
#define NWAL_queueHnd                   Qmss_QueueHnd
/*  @}  */

/* @{ */
/**
 *  @def  NWAL_hwChanHandle
 *        CPPI Handle for NetCP devices
 */
#define NWAL_hwChanHandle   Cppi_Handle
/*  @}  */


/* @{ */
/**
 *  @def  NWAL_FLOW_NOT_SPECIFIED
 *        Flow not valid from Application
 */
#define NWAL_FLOW_NOT_SPECIFIED     CPPI_PARAM_NOT_SPECIFIED
/*  @}  */


/* @{ */
/**
 *  @def  NWAL_QUEUE_NOT_SPECIFIED
 *        Hardware Queue Parameter not valid from Application
 */
#define NWAL_QUEUE_NOT_SPECIFIED     QMSS_PARAM_NOT_SPECIFIED
/*  @}  */
/*  @}  */

#ifdef __cplusplus
}
#endif

#endif  /* _NWAL_NETCP_H */
