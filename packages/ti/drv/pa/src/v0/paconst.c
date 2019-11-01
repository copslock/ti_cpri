/******************************************************************************
 * FILE PURPOSE:  Constant Register values for PASS
 ******************************************************************************
 * FILE NAME:   paconst.c  
 *
 * DESCRIPTION: Constant Registers for PA 
 *
 * FUNCTION               DESCRIPTION
 * ========               ===========
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2016
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

#include "pa.h"
/****************************************************************************
 * DATA PURPOSE: PDSP Constant Registers
 ****************************************************************************
 * DESCRIPTION: Specify the user-defined PDSP constant registers (c24-c31)
 ****************************************************************************/
 const uint32_t pap_pdsp_const_reg_map[6][4] = 
 {
    /* PDSP0: C24-C31 */
    {
        0x0000007F,         /* C25-C24 */    
        0x0000006E,         /* C27-C26 */
        0x00000000,         /* C29-C28 */
        0x00000473          /* C31-C30 */
    },
    
    /* PDSP1: C24-C31 */
    {
        0x0001007F,         /* C25-C24 */    
        0x00480040,         /* C27-C26 */
        0x00000000,         /* C29-C28 */
        0x00000473          /* C31-C30 */
    },
    
    /* PDSP2: C24-C31 */
    {
        0x0002007F,         /* C25-C24 */    
        0x00490044,         /* C27-C26 */
        0x00000000,         /* C29-C28 */
        0x00000473          /* C31-C30 */
    },
    
    /* PDSP3: C24-C31 */
    {
        0x0003007F,         /* C25-C24 */    
        0x0000006E,         /* C27-C26 */
        0x00000000,         /* C29-C28 */
        0x00000473          /* C31-C30 */
    },
 
    /* PDSP4: C24-C31 */
    {
        0x0070007F,         /* C25-C24 */    
        0x00000003,         /* C27-C26 */
        0x04080404,         /* C29-C28 */
        0x00000473          /* C31-C30 */
    },
 
    /* PDSP5: C24-C31 */
    {
        0x0071007F,         /* C25-C24 */    
        0x00000003,         /* C27-C26 */
        0x04080404,         /* C29-C28 */
        0x00000473          /* C31-C30 */
    }
};

