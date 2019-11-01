/**
 *   @file  qmss_firmware.h
 *
 *   @brief   
 *      This is the PDSP firmware include file.
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


#ifndef QMSS_FW_H_
#define QMSS_FW_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(DEVICE_K2H) || \
    defined(DEVICE_K2K) || \
    defined(DEVICE_K2E) || \
    defined(DEVICE_K2L) || \
    defined(DEVICE_K2G) || \
    defined(SOC_K2G)    || \
    defined(SOC_K2H)    || \
    defined(SOC_K2K)    || \
    defined(SOC_K2E)    || \
    defined(SOC_K2L)

/* Firmware for "Keystone 2" */
#include <ti/drv/qmss/firmware/v1/acc16_le_bin.h>
#include <ti/drv/qmss/firmware/v1/acc16_be_bin.h>
#include <ti/drv/qmss/firmware/v1/acc32_le_bin.h>
#include <ti/drv/qmss/firmware/v1/acc32_be_bin.h>
#include <ti/drv/qmss/firmware/v1/acc48_le_bin.h>
#include <ti/drv/qmss/firmware/v1/acc48_be_bin.h>
#include <ti/drv/qmss/firmware/v1/qos_le_bin.h>
#include <ti/drv/qmss/firmware/v1/qos_be_bin.h>
#include <ti/drv/qmss/firmware/v1/qos_sched_le_bin.h>
#include <ti/drv/qmss/firmware/v1/qos_sched_be_bin.h>
#include <ti/drv/qmss/firmware/v1/qos_sched_wide_le_bin.h>
#include <ti/drv/qmss/firmware/v1/qos_sched_wide_be_bin.h>
#include <ti/drv/qmss/firmware/v1/qos_sched_drop_sched_le_bin.h>
#include <ti/drv/qmss/firmware/v1/qos_sched_drop_sched_be_bin.h>
#include <ti/drv/qmss/firmware/v1/sriortr_le_bin.h>
#include <ti/drv/qmss/firmware/v1/sriortr_be_bin.h>

#elif defined(DEVICE_C6678) || \
      defined(DEVICE_C6657) || \
      defined(SOC_C6678)    || \
      defined(SOC_C6657)

/* Firmware for "Keystone 1" */
#include <ti/drv/qmss/firmware/v0/acc16_le_bin.h>
#include <ti/drv/qmss/firmware/v0/acc16_be_bin.h>
#include <ti/drv/qmss/firmware/v0/acc32_le_bin.h>
#include <ti/drv/qmss/firmware/v0/acc32_be_bin.h>
#include <ti/drv/qmss/firmware/v0/acc48_le_bin.h>
#include <ti/drv/qmss/firmware/v0/acc48_be_bin.h>
#include <ti/drv/qmss/firmware/v0/qos_le_bin.h>
#include <ti/drv/qmss/firmware/v0/qos_be_bin.h>
#include <ti/drv/qmss/firmware/v0/qos_sched_le_bin.h>
#include <ti/drv/qmss/firmware/v0/qos_sched_be_bin.h>
#include <ti/drv/qmss/firmware/v0/qos_sched_wide_le_bin.h>
#include <ti/drv/qmss/firmware/v0/qos_sched_wide_be_bin.h>
#include <ti/drv/qmss/firmware/v0/qos_sched_drop_sched_le_bin.h>
#include <ti/drv/qmss/firmware/v0/qos_sched_drop_sched_be_bin.h>
#include <ti/drv/qmss/firmware/v0/sriortr_le_bin.h>
#include <ti/drv/qmss/firmware/v0/sriortr_be_bin.h>

#else
#error Device does not have supported firmware
#endif

/** @addtogroup QMSS_LLD_DATASTRUCT
@{ 
*/        

/** 
 * @brief PDSP Firmware symbols.
 */
/** @brief 32 channel high priority accumulation little endian firmware */
extern const unsigned int acc32_le[];
/** @brief 32 channel high priority accumulation big endian firmware */
extern const unsigned int acc32_be[];
/** @brief 16 channel low priority accumulation little endian firmware */
extern const unsigned int acc16_le[];
/** @brief 16 channel low priority accumulation big endian firmware */
extern const unsigned int acc16_be[];
/** @brief 48 channel combined high and low priority accumulation little endian firmware */
extern const unsigned int acc48_le[];
/** @brief 48 channel combined high and low priority accumulation big endian firmware */
extern const unsigned int acc48_be[];
/** @brief QoS little endian firmware */
extern const unsigned int qos_le[];
/** @brief QoS big endian firmware */
extern const unsigned int qos_be[];
/** @brief QoS scheduler little endian firmware */
extern const unsigned int qos_sched_le[];
/** @brief QoS scheduler big endian firmware */
extern const unsigned int qos_sched_be[];
/** @brief QoS scheduler with drop scheduler little endian firmware */
extern const unsigned int qos_sched_drop_sched_le[];
/** @brief QoS scheduler with drop scheduler big endian firmware */
extern const unsigned int qos_sched_drop_sched_be[];
/** @brief wide QoS scheduler little endian firmware */
extern const unsigned int qos_sched_wide_le[];
/** @brief wide QoS scheduler big endian firmware */
extern const unsigned int qos_sched_wide_be[];
/** @brief SRIO router/switch - little endian */
extern const unsigned int sriortr_le[];
/** @brief SRIO router/switch - big endian */
extern const unsigned int sriortr_be[];

/** 
@} 
*/

#ifdef __cplusplus
}
#endif

#endif /* QMSS_FW_H_ */

