/******************************************************************************
 * FILE PURPOSE:  Compile time Tunable parameters for NWAL Module
 ******************************************************************************
 * FILE NAME:   nwal_tune.h
 *
 * DESCRIPTION: NWAL Tunable parameter definitions
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
 *   @file  nwal_tune.h
 *
 *   path  ti/drv/nwal/nwal_tune.h
 *
 *   @brief  Compile time Tunable parameters for NWAL Module
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


#ifndef _NWAL_TUNE_H
#define _NWAL_TUNE_H

#ifdef __cplusplus
extern "C" {
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
 *  @def  NWAL_MAX_RX_PKT_THRESHOLD
 *        Threshold for maximum packets which can be received in one call back
 */
#define NWAL_MAX_RX_PKT_THRESHOLD                   32
/*  @}  */

/**
 *  
 * @{ */
/**
 *  @def  NWAL_CACHE_LINE_SIZE
 *        Default cache line size used while factoring in memory requirements at NWAL.
 *        Default value is factoring in C66x devices
 *        For processors such as ARMv7 this could be modified to 64 to reduce memory
 *        requirement at NWAL
 */
#define NWAL_CACHE_LINE_SIZE                        128
/** @} */


/**
 *  
 * @{ */
/**
 *  @def  NWAL_DESC_SIZE
 *        Default value is factoring in additional PS command for the packets to NetCP
 */
#define NWAL_DESC_SIZE                              128
/** @} */

/**
 *  
 * @{ */
/**
 *  @def  NWAL_ENABLE_RX_NETCP_IP_INTENSIVE_CHECK
 *        Enable more intensive check at NetCP for IPv4 header for
 *        received packets. Disable by commenting out below define
 *        only for specific application need
 */
#define NWAL_ENABLE_RX_NETCP_IP_INTENSIVE_CHECK     1
/** @} */

/**
 *
 * @{ */
/**
 *  @def  NWAL_MAX_PENDING_IPSEC_STATS
 *        Maximum number of IPSec Stats requests pending
 *        at a point of time at each core/process
 *        Parameter
 */
#define NWAL_MAX_PENDING_IPSEC_STATS                4
/** @} */

/*@}*/ /* @name COMMON APIs */  

#ifdef __cplusplus
}
#endif
  

#endif  /* _NWAL_TUNE_H */
