/*
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/ 
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
/*  file  pcieloc.h
 *
 *  Internal module data structures and definitions
 *
 */
#ifndef PCIELOC__H
#define PCIELOC__H

#ifdef __cplusplus
extern "C" {
#endif


/* System level header files */
#include <ti/drv/pcie/pcie.h>

/* Common utility macros */

/* Handle must point to one of the valid peripheral base addresses */
int32_t pcie_check_handle_fcn (Pcie_Handle handle);
Pcie_DeviceCfgBaseAddr *pcie_handle_to_cfg (Pcie_Handle handle);

#define pcie_check_handle(x) {\
    if (pcie_check_handle_fcn(x) == 0) {  \
      return pcie_RET_INV_HANDLE;         \
    }										\
  }

/* Save last warning if debug is enabled */
#ifdef pcie_DEBUG
#define pcie_check_result(wv,x)         \
  {                                     \
    pcieRet_e thisRet = (x);              \
    if (thisRet < pcie_RET_OK) {        \
      /* latch warning */               \
      (wv) = thisRet;                     \
    } else if (thisRet > pcie_RET_OK) { \
      return thisRet;                   \
    }                                   \
  }
#else
#define pcie_check_result(wv,x)    \
  {                                  \
    pcieRet_e thisRet = (x);         \
    if (thisRet > pcie_RET_OK) {   \
      return thisRet;                \
    }                                \
  }
#endif

#ifdef pcie_DEBUG
/* Declare range_check */
#define pcie_range_check_begin uint32_t range_check = 0
#define pcie_range_check_return                          \
                         (range_check ?                  \
                          pcie_RET_DBG_WRITE_OVERFLOW :  \
                          pcie_RET_OK)
/* Make sure "val" fits in the specified field */
#define pcie_range_check(flag,val,mask)                  \
   flag = flag || (val & ~mask)
#else
/* Optimize out range check */
#define pcie_range_check_begin
#define pcie_range_check_return (pcie_RET_OK)
#define pcie_range_check(flag,val,mask)
#endif

/* Sets a bitfield, while optionally checking range.  
 */
#define pcie_setbits(newval,field,val)                        \
  {                                                           \
    /* Eval "val" only once */                                \
    uint32_t working_val = (val);                               \
    uint32_t working_mask = (field##_MASK >> field##_SHIFT);  \
    /* warning if the value is outside the range */           \
    /* This generates runtime overhead if pcie_DEBUG set */   \
    pcie_range_check(range_check,working_val,working_mask);   \
    working_val &= working_mask;                              \
    working_val <<= field##_SHIFT;                            \
    (newval) &= ~((uint32_t)(field##_MASK));                    \
    (newval) |= working_val;                                    \
  } 


/* Extracts a bitfield */
#define pcie_getbits(val,field,final_result) \
  ((final_result) = ((val) & (field##_MASK)) >> (field##_SHIFT))

/*****************************************************************************
 * pcie handle - internal definition
 *
 * handle/instance pointer, complete type (external is void *)
 *****************************************************************************/
typedef struct Pcie_IntHandle_s
{
  /*! table of soc specific PCIE functions */
  Pcie_FxnTable          fxnTable;
  /*! sructure containing base addresses for this PCIE */
  Pcie_DeviceCfgBaseAddr bases;
  /*! index of this interface (0 or 1) */
  uint32_t               pcie_index;
} *Pcie_IntHandle;

/*****************************************************************************
 * Local Object - per core device configuration
 *
 * This will store any device addresses as accessible from this core.  Thus,
 * if virt2phys is required, store virtual addresses
 *****************************************************************************/
typedef struct 
{
    /*! one instance for each peripheral on device, which is used as handle */
    struct Pcie_IntHandle_s insts[pcie_MAX_PERIPHS];
} Pcie_LocalObj;

extern Pcie_LocalObj pcieLObj;
extern uint8_t pcieLObjIsValid;


#ifdef __cplusplus
}
#endif

#endif  /* _PCIELOC_H */

/* Nothing past this point */

