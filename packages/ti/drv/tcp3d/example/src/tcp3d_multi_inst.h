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

#ifndef _TCP3D_MULTI_INST_H_
#define _TCP3D_MULTI_INST_H_

/**
 * Test TCP3D Notification Events
 */
#define TCP3D_0_TEST_EVENT              7
#define TCP3D_1_TEST_EVENT              23

/**
 * This gives the channel numbers to which the TCP3 decoder REVTs are mapped.
 * ((per Data sheet SPRS835C, SPRS893 and internal spec 
 * intc_1.3.4.12.xlsx,TPCC2)
 */
#define TCP3D_0_REVT0_CH_NUMBER         34
#define TCP3D_0_REVT1_CH_NUMBER         35
#define TCP3D_1_REVT0_CH_NUMBER         36
#define TCP3D_1_REVT1_CH_NUMBER         37

INLINE UInt32 getHostIntrNum(UInt32 dspCoreID)
{
    /* Host Interrupts for CPINTC0 (per spec intc_1.3.4.12.xls) */
    UInt32  hostIntr[] = {13u, 29u, 45u, 61u};

    return hostIntr[dspCoreID];
}

INLINE UInt32 getNotifyEventNum(UInt8 instNum)
{
    UInt32 testEvt;

    if ( instNum == CSL_TCP3D_0 )
        testEvt = TCP3D_0_TEST_EVENT; //  First instance
    else
        testEvt = TCP3D_1_TEST_EVENT; //  Second instance

    return testEvt;
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
    UInt32 chNum;

    if ( instNum == CSL_TCP3D_0 )
        chNum = TCP3D_0_REVT0_CH_NUMBER; //  First instance
    else
        chNum = TCP3D_1_REVT0_CH_NUMBER; //  Second instance

    return chNum;
}

INLINE UInt32 getRevt1ChannelNum(UInt8 instNum)
{
    UInt32 chNum;

    if ( instNum == CSL_TCP3D_0 )
        chNum = TCP3D_0_REVT1_CH_NUMBER; //  First instance
    else
        chNum = TCP3D_1_REVT1_CH_NUMBER; //  Second instance

    return chNum;
}

INLINE UInt8 getTcp3dInstNum(UInt32 dspCoreID)
{
    UInt8 instNum;

    if ( dspCoreID == 0 )
        instNum = CSL_TCP3D_0; //  First instance
    else
        instNum = CSL_TCP3D_1; //  Second instance

    return instNum;
}

INLINE UInt32 getTcp3dCfgRegsBase(UInt8 instNum)
{
    UInt32  regBase;

    if ( instNum == CSL_TCP3D_0 )
        regBase = CSL_TCP3D_0_CFG_REGS; // First instance
    else
        regBase = CSL_TCP3D_1_CFG_REGS; //  Second instance

    return regBase;
}

INLINE UInt32 getTcp3dDataRegsBase(UInt8 instNum)
{
    UInt32  regBase;

    if ( instNum == CSL_TCP3D_0 )
        regBase = CSL_TCP3D_0_DATA_REGS; // First instance
    else
        regBase = CSL_TCP3D_1_DATA_REGS; //  Second instance

    return regBase;
}

#endif  /* _TCP3D_MULTI_INST_H_ */

