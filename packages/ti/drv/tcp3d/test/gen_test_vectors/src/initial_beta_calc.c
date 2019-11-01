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



#include <math.h>
#include "sim_param.h"

//****************************************************************************
//Calculates initial beta states for 3gpp and stores them into the register
//structure. The 12 input tail bit LLRs are stored in array of 12 elements, first
//six elements correspond to tail bits of encoder 1, next 6 elements correspond 
// to encoder 2.
//****************************************************************************
void Initial_beta_calc(TCP3_REGS *reg, int8_t tail_llrs[])
{
	int32_t xt0[6];
	int32_t xt1[6];
    int32_t i, Kt;
    int32_t beta0[8];
    int32_t beta1[8];
    int32_t betaMax;
	
	
	//Initial beta states (only for LTE and 3GPP)
	if(reg->mode_sel != 2)
	{
	    //In LTE or 3gpp handle tail bits:
	    Kt = 3 - (reg->ExtndNumInfoBits - reg->NumInfoBits); //number of tail stages
        if(Kt==3)
        {
            for(i=0; i<6; i++)
            {
                xt0[i] = tail_llrs[i];    //MAP 0 tail bits
                xt1[i] = tail_llrs[i+6];  //MAP 1 tail bits
            }
            beta0[0] = 0;
            beta0[1] = xt0[0] + xt0[1];
            beta0[2] = xt0[0]          + xt0[2] + xt0[3];
            beta0[3] =          xt0[1] + xt0[2] + xt0[3];
            beta0[4] =          xt0[1] + xt0[2]          + xt0[4] + xt0[5];
            beta0[5] = xt0[0]          + xt0[2]          + xt0[4] + xt0[5];
            beta0[6] = xt0[0] + xt0[1]          + xt0[3] + xt0[4] + xt0[5];
            beta0[7] =                            xt0[3] + xt0[4] + xt0[5];

            betaMax = beta0[0];
            for(i=1; i<8; i++)
            {
                if(beta0[i]>betaMax) 
                    betaMax = beta0[i];
            }
            for(i=0; i<8; i++)
            {
                beta0[i] += 127 - betaMax;
            }

            beta1[0] = 0;
            beta1[1] = xt1[0] + xt1[1];
            beta1[2] = xt1[0]          + xt1[2] + xt1[3];
            beta1[3] =          xt1[1] + xt1[2] + xt1[3];
            beta1[4] =          xt1[1] + xt1[2]          + xt1[4] + xt1[5];
            beta1[5] = xt1[0]          + xt1[2]          + xt1[4] + xt1[5];
            beta1[6] = xt1[0] + xt1[1]          + xt1[3] + xt1[4] + xt1[5];
            beta1[7] =                            xt1[3] + xt1[4] + xt1[5];

            betaMax = beta1[0];
            for(i=1; i<8; i++)
            {
                if(beta1[i]>betaMax) 
                    betaMax = beta1[i];
            }
            for(i=0; i<8; i++)
            {
                beta1[i] += 127 - betaMax;
            }
        }
        else if(Kt==2)
        {
            for(i=0; i<4; i++)
            {
                xt0[i] = tail_llrs[i+2];  //MAP 0
                xt1[i] = tail_llrs[i+2+6];  //MAP 1
            }
            beta0[0] = 0;
            beta0[1] = xt0[0] + xt0[1];
            beta0[2] = xt0[0]          + xt0[2] + xt0[3];
            beta0[3] =          xt0[1] + xt0[2] + xt0[3];

            betaMax = beta0[0];
            for(i=1; i<4; i++)
            {
                if(beta0[i]>betaMax) 
                    betaMax = beta0[i];
            }
            for(i=0; i<4; i++)
            {
                beta0[i] += 127 - betaMax;
            }
            for(i=4; i<8; i++)
            {
                beta0[i] = -127;
            }

            beta1[0] = 0;
            beta1[1] = xt1[0] + xt1[1];
            beta1[2] = xt1[0]          + xt1[2] + xt1[3];
            beta1[3] =          xt1[1] + xt1[2] + xt1[3];

            betaMax = beta1[0];
            for(i=1; i<4; i++)
            {
                if(beta1[i]>betaMax) 
                    betaMax = beta1[i];
            }
            for(i=0; i<4; i++)
            {
                beta1[i] += 127 - betaMax;
            }
            for(i=4; i<8; i++)
            {
                beta1[i] = -127;
            }
        }
        else if(Kt==1)
        {
            for(i=0; i<2; i++)
            {
                xt0[i] = tail_llrs[i+4];  //MAP 0
                xt1[i] = tail_llrs[i+4+6];  //MAP 1
            }
            beta0[0] = 0;
            beta0[1] = xt0[0] + xt0[1];

            betaMax = beta0[0];
            for(i=1; i<2; i++)
            {
                if(beta0[i]>betaMax) 
                    betaMax = beta0[i];
            }
            for(i=0; i<2; i++)
            {
                beta0[i] += 127 - betaMax;
            }
            for(i=2; i<8; i++)
            {
                beta0[i] = -127;
            }

            beta1[0] = 0;
            beta1[1] = xt1[0] + xt1[1];

            betaMax = beta1[0];
            for(i=1; i<2; i++)
            {
                if(beta1[i]>betaMax) 
                    betaMax = beta1[i];
            }
            for(i=0; i<2; i++)
            {
                beta1[i] += 127 - betaMax;
            }
            for(i=2; i<8; i++)
            {
                beta1[i] = -127;
            }
        }
        else if(Kt==0)
        {
            beta0[0] = +127;
            for(i=1; i<8; i++)
            {
                beta0[i] = -127;
            }

            beta1[0] = +127;
            for(i=1; i<8; i++)
            {
                beta1[i] = -127;
            }
        }
	
	    //CFG4
	    reg->beta_st0_map0 = beta0[0];
	    reg->beta_st1_map0 = beta0[1];
	    reg->beta_st2_map0 = beta0[2];
	    reg->beta_st3_map0 = beta0[3];
	                               
	    //CFG5                      
	    reg->beta_st4_map0 = beta0[4];
	    reg->beta_st5_map0 = beta0[5];
	    reg->beta_st6_map0 = beta0[6];
	    reg->beta_st7_map0 = beta0[7];
	                               
	    //CFG6                      
	    reg->beta_st0_map1 = beta1[0];
	    reg->beta_st1_map1 = beta1[1];
	    reg->beta_st2_map1 = beta1[2];
	    reg->beta_st3_map1 = beta1[3];
	                               
	    //CFG7                      
	    reg->beta_st4_map1 = beta1[4];
	    reg->beta_st5_map1 = beta1[5];
	    reg->beta_st6_map1 = beta1[6];
	    reg->beta_st7_map1 = beta1[7];
    }                              
	else
	{
	    //WiMAX mode:
	    //CFG4                 
	    reg->beta_st0_map0 = 0;
	    reg->beta_st1_map0 = 0;
	    reg->beta_st2_map0 = 0;
	    reg->beta_st3_map0 = 0;
	                          
	    //CFG5                 
	    reg->beta_st4_map0 = 0;
	    reg->beta_st5_map0 = 0;
	    reg->beta_st6_map0 = 0;
	    reg->beta_st7_map0 = 0;
	                          
	    //CFG6                 
	    reg->beta_st0_map1 = 0;
	    reg->beta_st1_map1 = 0;
	    reg->beta_st2_map1 = 0;
	    reg->beta_st3_map1 = 0;
	                          
	    //CFG7                 
	    reg->beta_st4_map1 = 0;
	    reg->beta_st5_map1 = 0;
	    reg->beta_st6_map1 = 0;
	    reg->beta_st7_map1 = 0;    
	    
	}
}