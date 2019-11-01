/**
 *  @file   trace.h
 *
 *  @brief  Common trace utility for PDK drivers, required registration
 *          of trace callback function by application with each PDK driver
 *          to receive traces.
 *          Currently only supported by EMAC driver
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _TRACE_H
#define _TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Trace level indicating fatal error
 */
#define                UTIL_TRACE_LEVEL_FATAL_ERR            1

/**
 * @brief Trace level indicating error
 */
#define                UTIL_TRACE_LEVEL_ERR                  2

/**
 * @brief Trace level indicating warning
 */
#define                UTIL_TRACE_LEVEL_WARN                 3

/**
 * @brief Trace level indicating unexpected condition
 */
#define                UTIL_TRACE_LEVEL_UNEXPECTED           4
/**
 * @brief Trace level for info
 */
#define                UTIL_TRACE_LEVEL_INFO                 5

/**
*@brief  This callback function is registered by an application  to receive
*        driver level traces.
*/
typedef void UTIL_TRACE_CB_T(uint8_t traceLevel, const char* fmt, ...);


/* to disable tracing, comment out the followng macro */
#define UTILS_TRACE_ENABLE

/* Common macro which can be used across drivers to provide driver level traces to 
 * applications which have registered a driver trace callback function */
#ifdef UTILS_TRACE_ENABLE
#define UTILS_trace(dbg_level, trace_fxn, fmt, ...)  if(trace_fxn) { trace_fxn(dbg_level, "func: %s line:%d: " fmt,__FUNCTION__, __LINE__, __VA_ARGS__); } 
#else
#define UTILS_trace(dbg_level, trace_fxn, fmt, ...)
#endif

#ifdef __cplusplus
}
#endif
  

#endif  /* _TRACE_H */
