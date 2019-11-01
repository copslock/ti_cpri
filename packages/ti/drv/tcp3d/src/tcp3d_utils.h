/*
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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



#ifndef _TCP3D_UTILS_H_
#define _TCP3D_UTILS_H_

/* c99 types includes */
#include <stdint.h>
#include <stdlib.h>

/* ========================================================================= */
/**
 * @brief  Macro for providing the address with the alignment requested
 */
#define ALIGN(Addr, Algnmt) ((Addr+(Algnmt)-1)&(0xFFFF-(Algnmt)+1))

/**
 * @brief  Macro for computing minimum of the two values
 */
#define MIN(x,y)            ((x) < (y) ? (x):(y))

/**
 * @brief  Macro for computing maximum of the two values
 */
#define MAX(x,y)            ((x) > (y) ? (x):(y))

/**
 * @brief   Macro for computing hard decisions size in words 
 */
#define COMPUTE_HD_WORD_SIZE(x) (((x)+31)>>5)

/**
 * @brief   Macro for computing hard decisions size in bytes
 */
#define COMPUTE_HD_BYTE_SIZE(x) (COMPUTE_HD_WORD_SIZE(x)*4)

/**
 * @brief   Macro for computing Kext value for WCDMA using the formula
 *              Kext=4*Ceil(K/4)
 */
#define COMPUTE_KEXT(a)         (4*((a+3)>>2))

/**
 * @brief   Macro for computing Kout value for WCDMA using the formula
 *              Kout = (2*KEXT-K).
 * @ref     COMPUTE_KEXT macro
 */
#define COMPUTE_KOUT(x)         ((2*COMPUTE_KEXT(x))-(x))

/**
 * @brief   Macro for computing Kt value for WCDMA using the formula
 *              Kt = 3 - (Kext-K)
 * @ref     COMPUTE_KOUT macro
 */
#define COMPUTE_KT(x)           (3 -(COMPUTE_KEXT(x)-(x)))

/* ========================================================================= */

/* ========================================================================= */
/**
 *  \fn         uint32_t Tcp3d_glbMap (uint8_t coreID, uint32_t addr)
 *
 *  \brief      This is a utility function provided as part of TCP3D Driver for
 *              converting the local address to the global address using the 
 *              coreID.
 *              The address mapping will be done only if the address falls in
 *              the defined local L2 memory area. Otherwise, the address is
 *              returned as is.
 * 
 *  \param[in]      coreID
 *              Core ID value corresponding to the local core. If the coreID
 *              value is invalid, the address translation is not done.
 * 
 *  \param[in]      addr
 *              Local address for which the global mapped address is required.
 *
 *  \pre        Pass the coreID value [0-3] depending on where test application
 *              is running.
 *
 *  \post       None
 *
 *  \return     Returns the global address value of the passed local address.
 * 
 */
uint32_t Tcp3d_glbMap (uint8_t coreID, uint32_t addr);

/**
 *  \fn         uint32_t Tcp3d_div32by16(uint32_t num, uint16_t den)
 *
 *  \brief      This is a utility function provided as part of TCP3D Driver for
 *              calculating the division of a 32-bit value by a 16-bit value.
 * 
 *  \param[in]      num
 *              Numerator value.
 * 
 *  \param[in]      den
 *              Denominator value.
 *
 *  \pre        Pass the coreID value [0-3] depending on where test application
 *              is running.
 *
 *  \post       None
 *
 *  \return     Returns the global address value of the passed local address.
 * 
 */
uint32_t Tcp3d_div32by16(uint32_t num, uint16_t den);

/* ========================================================================= */

#endif /* _TCP3D_UTILS_H_ */
