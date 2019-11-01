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

#ifndef _TCP3D_SINGLE_INST_H_
#define _TCP3D_SINGLE_INST_H_

#ifdef CSL_TCP3D_A
/* Convert Keystone CSL definitions ti Keystone2 CSL definitions */
#define CSL_EDMACC_2_REGS       CSL_EDMA2CC_REGS
#define CSL_CIC_0_REGS          CSL_CP_INTC_0_REGS
#define CSL_TCP3D_0             CSL_TCP3D_A
#define CSL_TCP3D_0_CFG_REGS    CSL_TCP3D_A_CFG_REGS
#define CSL_TCP3D_0_DATA_REGS   CSL_TCP3D_A_DATA_REGS
#endif

/**
 * Test TCP3D Notification Events
 */
#ifdef SOC_C6657
#define TCP3D_0_TEST_EVENT              23
#else
#define TCP3D_0_TEST_EVENT              7
#endif

/**
 * This gives the channel numbers to which the TCP3 decoder REVTs are mapped.
 * ((per Data sheet SPRS835C, SPRS893 and internal spec 
 * intc_1.3.4.12.xlsx,TPCC2)
 */
#ifdef SOC_C6657
#define TCP3D_0_REVT0_CH_NUMBER         0
#define TCP3D_0_REVT1_CH_NUMBER         1
#else
#define TCP3D_0_REVT0_CH_NUMBER         34
#define TCP3D_0_REVT1_CH_NUMBER         35
#endif
INLINE UInt32 getHostIntrNum(UInt32 dspCoreID)
{
    /* Host Interrupts for CPINTC0 (per spec - 0.0.1) */
#ifdef SOC_C6657	
    UInt32  hostIntr[] = {6u, 26u, 255u, 255u};
#else
    UInt32  hostIntr[] = {13u, 29u, 45u, 61u};
#endif    

    return hostIntr[dspCoreID];
}

INLINE UInt32 getNotifyEventNum(UInt8 instNum)
{
    return TCP3D_0_TEST_EVENT;
}

INLINE CSL_TPCC_ShadowRegs * getEdma3ShadowRegsBase(UInt32 regionNum)
{
    CSL_TpccRegs *tpcc2Regs = (CSL_TpccRegs *) CSL_EDMACC_2_REGS;

    return (CSL_TPCC_ShadowRegs *) &tpcc2Regs->SHADOW[regionNum];
}

INLINE UInt32 getCpIntc0RegsBase()
{
    return CSL_CIC_0_REGS;
}

INLINE UInt32 getRevt0ChannelNum(UInt8 instNum)
{
    return TCP3D_0_REVT0_CH_NUMBER;
}

INLINE UInt32 getRevt1ChannelNum(UInt8 instNum)
{
    return TCP3D_0_REVT1_CH_NUMBER;
}

INLINE UInt8 getTcp3dInstNum(UInt32 dspCoreID)
{
    return CSL_TCP3D_0;
}

INLINE UInt32 getTcp3dCfgRegsBase(UInt8 instNum)
{
    return CSL_TCP3D_0_CFG_REGS;
}

INLINE UInt32 getTcp3dDataRegsBase(UInt8 instNum)
{
    return CSL_TCP3D_0_DATA_REGS;
}

#endif  /* _TCP3D_SINGLE_INST_H_ */

