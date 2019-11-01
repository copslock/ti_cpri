/********************************************************************
 * Copyright (C) 2013 Texas Instruments Incorporated.
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
#ifndef __DFE_INTERNAL_H__
#define __DFE_INTERNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#define VALID_DFE_HANDLE(h) \
do {\
    if(h == NULL) {Dfe_osalLog("DFE Handle is NULL"); return DFE_ERR_INVALID_HANDLE; }\
} while(0);

#define CSL_HW_CTRL(stmt) \
do { \
    status = (stmt); \
	if(status != DFE_FL_SOK) { Dfe_osalLog(#stmt " error = %d", status); return DFE_ERR_HW_CTRL; } \
} while(0);

#define CSL_HW_QUERY(stmt) \
do { \
    status = (stmt); \
	if(status != DFE_FL_SOK) { Dfe_osalLog(#stmt " error = %d", status); return DFE_ERR_HW_QUERY; } \
} while(0);

#define FREQLOW(x) (x & 0xFFFF)
#define FREQMID(x) ((x>>16) & 0xFFFF)
#define FREQHIGH(x) ((x>>32) & 0xFFFF)

#define min(a, b)  (((a) < (b)) ? (a) : (b))

#ifdef __cplusplus
}
#endif

#endif /* __DFE_INTERNAL_H__ */
