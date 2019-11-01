/**
 *   @file  qmss_drv.h
 *
 *   @brief   
 *      This is the Queue Manager Sub System Low Level Driver include file.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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


/** @defgroup QMSS_LLD_API QMSS
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 *
 *
 * @subsection References
 *   -# QMSS Functional Specification 
 *
 * @subsection Assumptions
 *    
 */
#ifndef QMSS_DRV_H_
#define QMSS_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * Shut off: remark #880-D: parameter "descType" was never referenced
 *
 * This is better than removing the argument since removal would break
 * backwards compatibility
 */
#ifdef _TMS320C6X
#pragma diag_suppress 880
#pragma diag_suppress 681
#elif defined(__GNUC__)
/* Same for GCC:
 * warning: unused parameter descType [-Wunused-parameter]
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif

/* QMSS includes */
#include <ti/drv/qmss/qmssver.h>
#include <ti/drv/qmss/qmss_qm.h>
#include <ti/drv/qmss/qmss_acc.h>
#include <ti/drv/qmss/qmss_qos.h>
#include <ti/drv/qmss/qmss_mgmt.h>

#ifdef __cplusplus
}
#endif

#endif /* QMSS_DRV_H_ */

