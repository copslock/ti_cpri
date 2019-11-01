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



/* 
 * Copyright (c) 2009
 * Texas Instruments
 *
 *  All rights reserved.  Property of Texas Instruments
 *  Restricted rights to use, duplicate or disclose this code are
 *  granted through contract.
 * 
 * */
/*
 * ======== CpIntc.c ========
 */

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Startup.h>
#include <xdc/runtime/System.h>

#include "ti/sysbios/family/c66/tci66xx/package/internal/CpIntc.xdc.h"

/*
 *  ======== CpIntc_dispatch ========
 */
//UInt32  cpintcCntr = 0;
//UInt32  cpintcCntr1 = 0;
//UInt32  cpintcCntr2 = 0;
//UInt32  sysIntTrack[200];
Void CpIntc_dispatchLoc(UInt hostInt)
{
    Int32 i;
    UInt32 index;
    UInt32 offset;
    UInt32 srsrVal;
    Int32  sysInt;
    UInt32 id = 0;
    extern volatile cregister UInt32 DNUM;

//    cpintcCntr++;

    /* for core# 4-7 use INTC1 otherwise use INTC0 */
    if (DNUM > 3) {
        id = 1;
    }
    
//    if ( (*((UInt32*)0x0180000C)) == 0x1 )
//        System_printf("MISS detected\n");
 
    sysInt = CpIntc_module->hostIntToSysInt[hostInt];
    
//    sysIntTrack[cpintcCntr]=((hostInt<<16)|sysInt);
//    sysIntTrack[0]=((hostInt<<16)|sysInt);

    /* 
     *  If only one system interrupt is mapped to a host interrupt
     *  we don't need to read the Sys Status Raw Registers. We
     *  know exactly which system interrupt triggered the interrupt.
     */     
    if (sysInt != 0xff && sysInt != 0xfe) {
//        cpintcCntr1++;
        /* clear system interrupt associated with host interrupt */
        CpIntc_clearSysInt(id, sysInt);

        /* call function with arg */
        CpIntc_module->dispatchTab[sysInt].fxn(
            CpIntc_module->dispatchTab[sysInt].arg);
    }
    else {
//        cpintcCntr2++;
        /*
         *  Loop through System Interrupt Status Enabled/Clear Registers for
         *  pending enabled interrupts. The highest numbered system interrupt
         *  will be processed first from left to right.
         */
        for (i = CpIntc_numStatusRegs - 1; i >= 0; i--) {
            offset = i << 5;
            
            /*
             *  SDOCM00062100 - Nyquist CpIntc_dispatch needs to read the
             *  correct status pending and enabled register once the
             *  Simulator is fixed.
             *  Fix is:
             *      srsrVal = CpIntc_module->controller[id]->SECR[j - i];
             */
            srsrVal = CpIntc_module->controller[id]->SRSR[i] &
                      CpIntc_module->controller[id]->ESR[i];
            
            /* Find pending interrupts from left to right */
            while (srsrVal) {
                index = 31 - _lmbd(1, srsrVal);
                srsrVal &= ~(1 << index);
                    
                /* Make sure pending interrupt is mapped to host interrupt */
                if (CpIntc_module->controller[id]->CMR[offset + index]
                    == hostInt) {
                    /* clear system interrupt first */
                    CpIntc_clearSysInt(id, offset + index);
                       
                    /* call function with arg */
                    CpIntc_module->dispatchTab[offset + index].fxn(
                        CpIntc_module->dispatchTab[offset + index].arg);
                } 
            }
        }
    }
}


