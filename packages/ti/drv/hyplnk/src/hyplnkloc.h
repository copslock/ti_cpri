/*
 *
 * Copyright (C) 2010-2016 Texas Instruments Incorporated - http://www.ti.com/ 
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

/* ================================================================= */
/*  file  hyplnkloc.h
 *
 *  Internal module data structures and definitions
 *
 */
#ifndef _HYPLNKLOC_H
#define _HYPLNKLOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* System level header files */

/* Common utility macros */
int hyplnk_check_handle_fcn (Hyplnk_Handle handle);
/* Handle must point to one of the valid peripheral base addresses */
#define hyplnk_check_handle(x) \
  if (hyplnk_check_handle_fcn(x) == 0) {  \
    return hyplnk_RET_INV_HANDLE;         \
  }

/* Save last warning if debug is enabled */
#ifdef hyplnk_DEBUG
#define hyplnk_check_result(wv,x)         \
  {                                       \
    hyplnkRet_e thisRet = x;              \
    if (thisRet < hyplnk_RET_OK) {        \
      /* latch warning */                 \
      wv = thisRet;                       \
    } else if (thisRet > hyplnk_RET_OK) { \
      return thisRet;                     \
    }                                     \
  }
#else
#define hyplnk_check_result(wv,x)    \
  {                                  \
    hyplnkRet_e thisRet = x;         \
    if (thisRet > hyplnk_RET_OK) {   \
      return thisRet;                \
    }                                \
  }
#endif

#ifdef hyplnk_DEBUG
#define hyplnk_USELESS_WRITE hyplnk_RET_DBG_USELESS_WRITE
#else
#define hyplnk_USELESS_WRITE hyplnk_RET_OK
#endif
    
#ifdef hyplnk_DEBUG
/* Declare range_check */
#define hyplnk_range_check_begin uint32_t range_check = 0
#define hyplnk_range_check_return                          \
                         (range_check ?                    \
                          hyplnk_RET_DBG_WRITE_OVERFLOW :  \
                          hyplnk_RET_OK)
/* Make sure "val" fits in the specified field */
#define hyplnk_range_check(flag,val,mask)                  \
   flag = flag || (val & ~mask)
#else
/* Optimize out range check */
#define hyplnk_range_check_begin
#define hyplnk_range_check_return hyplnk_RET_OK
#define hyplnk_range_check(flag,val,mask)
#endif

/* Sets a bitfield, while optionally checking range.  
 */
#define hyplnk_setbits(newval,field,val)                      \
  {                                                           \
    /* Eval "val" only once */                                \
    uint32_t working_val = val;                               \
    uint32_t working_mask = (field##_MASK >> field##_SHIFT);  \
    /* warning if the value is outside the range */           \
    /* This generates runtime overhead if hyplnk_DEBUG set */ \
    hyplnk_range_check(range_check,working_val,working_mask); \
    working_val &= working_mask;                              \
    working_val <<= field##_SHIFT;                            \
    newval &= ~field##_MASK;                                  \
    newval |= working_val;                                    \
  } 


/* Extracts a bitfield */
#define hyplnk_getbits(val,field,final_result) \
  final_result = (val & field##_MASK) >> field##_SHIFT;


/*****************************************************************************
 * Local Object - per core device configuration
 *
 * This will store any device addresses as accessible from this core.  Thus,
 * if virt2phys is required, store virtual addresses
 *****************************************************************************/
typedef struct
{
  Hyplnk_InitCfg cfg;
} Hyplnk_LocalObj;

extern Hyplnk_LocalObj hyplnkLObj;
extern uint8_t hyplnkLObjIsValid;

#ifdef __cplusplus
}
#endif

#endif  /* _HYPLNKLOC_H */

/* Nothing past this point */

