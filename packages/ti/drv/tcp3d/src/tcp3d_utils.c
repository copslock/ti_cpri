/**
 *  \file   tcp3d_utils.c
 *
 *  \brief  TCP3D Driver utility functions.
 *
 *  Copyright (C) Texas Instruments Incorporated 2009
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

/**
 *  Include Files
 */
/* c99 types includes */
#include <stdint.h>
#include <stdlib.h>

/**
 *  @brief      This is a utility function provided as part of TCP3D Driver for
 *              converting the local address to the global address using the 
 *              coreID.
 *              The address mapping will be done only if the address falls in
 *              the defined local L2 memory area. Otherwise, the address is
 *              returned as is.
 */
uint32_t Tcp3d_glbMap (uint8_t coreID, uint32_t addr)
{
    uint32_t  upcastAddr = 0;

    /*
     * Address mapping is done based on the mapping shown below.
     * 
     * L2SRAM      : org = 0x00800000, len = 0x100000 (local)
     * GEM0_L2_MEM : org = 0x10800000, len = 0x100000 (global)
     * GEM1_L2_MEM : org = 0x11800000, len = 0x100000 (global)
     * GEM2_L2_MEM : org = 0x12800000, len = 0x100000 (global)
     * GEM3_L2_MEM : org = 0x13800000, len = 0x100000 (global)
     *
     * MSMCSRAM    : org = 0x0c000000, len = 0x200000 (global)
     */

    /* Check if the address is in L2SRAM & a valid coreID */
    if ( ( addr >= 0x00800000 ) && ( addr < 0x00900000 ) )
    {
        upcastAddr = (uint32_t)( (0x10 | ( coreID & 0x3 )) << 24 );
    }

    return ( addr | upcastAddr );
}

/* Division of (a/b) */
uint32_t Tcp3d_div32by16(uint32_t num, uint16_t den)
{
    int32_t   expn;
    uint32_t  normal;
    uint32_t  a, b;
    int32_t   i;
    int32_t   ret;

    normal =  _norm( den );
    a = ( den << normal ) & 0x7fff0000;
    b = 0x80000000;                     /* dividend = 1 */

    #ifdef _TMS320C6X
    #pragma MUST_ITERATE( 15,15 );
    #endif
    for(i = 15; i > 0; i--)
    {
        b = _subc( b, a );                 /* divide */
    }
    b = b & 0x7fff;
    expn = 30 - (int32_t) normal;
    ret = _sshvr( _mpylir( b,num), expn );

    return (ret);
}

/* end of file */
