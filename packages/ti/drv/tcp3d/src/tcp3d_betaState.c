/** 
 *  \file   tcp3d_betaState.c
 *
 *  \brief  Beta State calculations functions from the tail bits for TCP3D pre-process.
 *
 *  Copyright (c) Texas Instruments Incorporated 2008
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
#include <tcp3d_drv_types.h>

/** 
 *  \brief  Calculates initial beta states using the tail bits for use with the
 *          TCP3D input configuration registers.
 */
void Tcp3d_betaStates(
                    IN  int8_t    * const RESTRICT tailBits,
                    IN  int32_t   signChange,
                    IN  int32_t   Kt,
                    OUT int8_t    * const RESTRICT beta0Ptr,
                    OUT int8_t    * const RESTRICT beta1Ptr)
{
    int32_t Tmp0,Tmp1,Tmp2;
    int64_t llRes, llFin;
    int32_t bit10, bit32, bit54, bit76, bit98, bitba;

    int32_t scale;
    int32_t beta0,beta1,beta2,beta3,beta4,beta5,beta6,beta7,betaMax;

    if(signChange)
    {
        scale = 0x00800080;   //+128
    }
    else
    {
        scale = 0xff80ff80;  //-128
    }

    //Load 12 tail samples
    Tmp0   = _mem4(&tailBits[0]);
    Tmp1   = _mem4(&tailBits[4]);
    Tmp2   = _mem4(&tailBits[8]);

#ifdef _BIG_ENDIAN
    Tmp0 = _rotl(_swap4(Tmp0), 16);
    Tmp1 = _rotl(_swap4(Tmp1), 16);
    Tmp2 = _rotl(_swap4(Tmp2), 16);
#endif

    //***************************************************
    //Initial Beta States Calculation for MAP 0 and MAP1
    //***************************************************
    //Process 4 samples
    llRes = _mpysu4ll(Tmp0, 0x01010101);
    llFin = _smpy2ll(_loll(llRes), scale);
    bit10 = _shr2(_spack2(_hill(llFin),_loll(llFin)), 8);
    llFin = _smpy2ll(_hill(llRes), scale);
    bit32 = _shr2(_spack2(_hill(llFin),_loll(llFin)), 8);

    //Process 4 samples
    //llRes = _mpysu4ll(_hill(llTmp), 0x01010101);
    llRes = _mpysu4ll(Tmp1, 0x01010101);
    llFin = _smpy2ll(_loll(llRes), scale);
    bit54 = _shr2(_spack2(_hill(llFin),_loll(llFin)), 8);
    llFin = _smpy2ll(_hill(llRes), scale);
    bit76 = _shr2(_spack2(_hill(llFin),_loll(llFin)), 8);

    //Process 4 samples
    llRes = _mpysu4ll(Tmp2, 0x01010101);
    llFin = _smpy2ll(_loll(llRes), scale);
    bit98 = _shr2(_spack2(_hill(llFin),_loll(llFin)), 8);
    llFin = _smpy2ll(_hill(llRes), scale);
    bitba = _shr2(_spack2(_hill(llFin),_loll(llFin)), 8);

    //Compute beta state values based on the Kt (# of trellis stages)
    if(Kt==3)
    {
        //Beta MAP0
        beta0 = 0;
        beta1 = _ext(bit10,16,16) + _ext(bit10,0,16); //xt0[0] + xt0[1];
        beta2 = _ext(bit10,16,16) + _ext(bit32,16,16) + _ext(bit32,0,16); //xt0[0] + xt0[2] + xt0[3];
        beta3 = _ext(bit10,0,16) + _ext(bit32,16,16) + _ext(bit32,0,16);  //xt0[1] + xt0[2] + xt0[3];
        beta4 = _ext(bit10,0,16) + _ext(bit32,16,16) + _ext(bit54,16,16) + _ext(bit54,0,16); //xt0[1] + xt0[2]+ xt0[4] + xt0[5];
        beta5 = _ext(bit10,16,16) + _ext(bit32,16,16) + _ext(bit54,16,16) + _ext(bit54,0,16);//xt0[0]+ xt0[2]+ xt0[4] + xt0[5];
        beta6 = _ext(bit10,16,16) + _ext(bit10,0,16) + _ext(bit32,0,16) + _ext(bit54,16,16) + _ext(bit54,0,16); //xt0[0] + xt0[1]+ xt0[3] + xt0[4] + xt0[5];
        beta7 = _ext(bit32,0,16) + _ext(bit54,16,16) + _ext(bit54,0,16);// xt0[3] + xt0[4] + xt0[5];

        betaMax = _max2(beta0,beta1);
        betaMax = _max2(betaMax,beta2);
        betaMax = _max2(betaMax,beta3);
        betaMax = _max2(betaMax,beta4);
        betaMax = _max2(betaMax,beta5);
        betaMax = _max2(betaMax,beta6);
        betaMax = _max2(betaMax,beta7);
        betaMax = _ext(betaMax,16,16);
        betaMax = 127 - betaMax;

        beta0 = beta0 + betaMax;
        beta1 = beta1 + betaMax;
        beta2 = beta2 + betaMax;
        beta3 = beta3 + betaMax;
        beta4 = beta4 + betaMax;
        beta5 = beta5 + betaMax;
        beta6 = beta6 + betaMax;
        beta7 = beta7 + betaMax;

        beta0 = _max2(0x8000ff81, beta0);
        beta1 = _max2(0x8000ff81, beta1);
        beta2 = _max2(0x8000ff81, beta2);
        beta3 = _max2(0x8000ff81, beta3);
        beta4 = _max2(0x8000ff81, beta4);
        beta5 = _max2(0x8000ff81, beta5);
        beta6 = _max2(0x8000ff81, beta6);
        beta7 = _max2(0x8000ff81, beta7);

#ifdef _BIG_ENDIAN
        _mem4(&beta0Ptr[0]) = _packl4(_pack2(beta0,beta1),_pack2(beta2,beta3));
        _mem4(&beta0Ptr[4]) = _packl4(_pack2(beta4,beta5),_pack2(beta6,beta7));
#else
        _mem4(&beta0Ptr[0]) = _packl4(_pack2(beta3,beta2),_pack2(beta1,beta0));
        _mem4(&beta0Ptr[4]) = _packl4(_pack2(beta7,beta6),_pack2(beta5,beta4));
#endif

        //Beta MAP1
        beta0 = 0;
        beta1 = _ext(bit76,16,16) + _ext(bit76,0,16); //xt1[0] + xt1[1];
        beta2 = _ext(bit76,16,16) + _ext(bit98,16,16) + _ext(bit98,0,16); //xt1[0] + xt1[2] + xt1[3];
        beta3 = _ext(bit76,0,16) + _ext(bit98,16,16) + _ext(bit98,0,16);  //xt1[1] + xt1[2] + xt1[3];
        beta4 = _ext(bit76,0,16) + _ext(bit98,16,16) + _ext(bitba,16,16) + _ext(bitba,0,16); //xt1[1] + xt1[2]+ xt1[4] + xt1[5];
        beta5 = _ext(bit76,16,16) + _ext(bit98,16,16) + _ext(bitba,16,16) + _ext(bitba,0,16);//xt1[0]+ xt1[2]+ xt1[4] + xt1[5];
        beta6 = _ext(bit76,16,16) + _ext(bit76,0,16) + _ext(bit98,0,16) + _ext(bitba,16,16) + _ext(bitba,0,16); //xt1[0] + xt1[1]+ xt1[3] + xt1[4] + xt1[5];
        beta7 = _ext(bit98,0,16) + _ext(bitba,16,16) + _ext(bitba,0,16);// xt1[3] + xt1[4] + xt1[5];

        betaMax = _max2(beta0,beta1);
        betaMax = _max2(betaMax,beta2);
        betaMax = _max2(betaMax,beta3);
        betaMax = _max2(betaMax,beta4);
        betaMax = _max2(betaMax,beta5);
        betaMax = _max2(betaMax,beta6);
        betaMax = _max2(betaMax,beta7);
        betaMax = _ext(betaMax,16,16);
        betaMax = 127 - betaMax;

        beta0 = beta0 + betaMax;
        beta1 = beta1 + betaMax;
        beta2 = beta2 + betaMax;
        beta3 = beta3 + betaMax;
        beta4 = beta4 + betaMax;
        beta5 = beta5 + betaMax;
        beta6 = beta6 + betaMax;
        beta7 = beta7 + betaMax;

        beta0 = _max2(0x8000ff81, beta0);
        beta1 = _max2(0x8000ff81, beta1);
        beta2 = _max2(0x8000ff81, beta2);
        beta3 = _max2(0x8000ff81, beta3);
        beta4 = _max2(0x8000ff81, beta4);
        beta5 = _max2(0x8000ff81, beta5);
        beta6 = _max2(0x8000ff81, beta6);
        beta7 = _max2(0x8000ff81, beta7);

#ifdef _BIG_ENDIAN
        _mem4(&beta1Ptr[0]) = _packl4(_pack2(beta0,beta1),_pack2(beta2,beta3));
        _mem4(&beta1Ptr[4]) = _packl4(_pack2(beta4,beta5),_pack2(beta6,beta7));
#else
        _mem4(&beta1Ptr[0]) = _packl4(_pack2(beta3,beta2),_pack2(beta1,beta0));
        _mem4(&beta1Ptr[4]) = _packl4(_pack2(beta7,beta6),_pack2(beta5,beta4));
#endif
    }
    else if(Kt==2)
    {
        //Beta MAP0
        beta0 = 0;
        beta1 = _ext(bit32,16,16) + _ext(bit32,0,16); //xt0[0] + xt0[1];
        beta2 = _ext(bit32,16,16) + _ext(bit54,16,16) + _ext(bit54,0,16); //xt0[0] + xt0[2] + xt0[3];
        beta3 = _ext(bit32,0,16) + _ext(bit54,16,16) + _ext(bit54,0,16);  //xt0[1] + xt0[2] + xt0[3];

        betaMax = _max2(beta0,beta1);
        betaMax = _max2(betaMax,beta2);
        betaMax = _max2(betaMax,beta3);
        betaMax = _ext(betaMax,16,16);
        betaMax = 127 - betaMax;

        beta0 = beta0 + betaMax;
        beta1 = beta1 + betaMax;
        beta2 = beta2 + betaMax;
        beta3 = beta3 + betaMax;

        beta0 = _max2(0x8000ff81, beta0);
        beta1 = _max2(0x8000ff81, beta1);
        beta2 = _max2(0x8000ff81, beta2);
        beta3 = _max2(0x8000ff81, beta3);

#ifdef _BIG_ENDIAN
        _mem4(&beta0Ptr[0]) = _packl4(_pack2(beta0,beta1),_pack2(beta2,beta3));
#else
        _mem4(&beta0Ptr[0]) = _packl4(_pack2(beta3,beta2),_pack2(beta1,beta0));
#endif
        _mem4(&beta0Ptr[4]) = 0x81818181;

        //Beta MAP1
        //temp
        beta0 = _ext(bit98,16,16);
        beta0 = _ext(bit98,0,16);
        beta0 = _ext(bitba,16,16);
        beta0 = _ext(bitba,0,16);

        //temp
        beta0 = 0;
        beta1 = _ext(bit98,16,16) + _ext(bit98,0,16); //xt1[0] + xt1[1];
        beta2 = _ext(bit98,16,16) + _ext(bitba,16,16) + _ext(bitba,0,16); //xt1[0] + xt1[2] + xt1[3];
        beta3 = _ext(bit98,0,16) + _ext(bitba,16,16) + _ext(bitba,0,16);  //xt1[1] + xt1[2] + xt1[3];

        betaMax = _max2(beta0,beta1);
        betaMax = _max2(betaMax,beta2);
        betaMax = _max2(betaMax,beta3);
        betaMax = _ext(betaMax,16,16);
        betaMax = 127 - betaMax;

        beta0 = beta0 + betaMax;
        beta1 = beta1 + betaMax;
        beta2 = beta2 + betaMax;
        beta3 = beta3 + betaMax;

        beta0 = _max2(0x8000ff81, beta0);
        beta1 = _max2(0x8000ff81, beta1);
        beta2 = _max2(0x8000ff81, beta2);
        beta3 = _max2(0x8000ff81, beta3);

#ifdef _BIG_ENDIAN
        _mem4(&beta1Ptr[0]) = _packl4(_pack2(beta0,beta1),_pack2(beta2,beta3));
#else
        _mem4(&beta1Ptr[0]) = _packl4(_pack2(beta3,beta2),_pack2(beta1,beta0));
#endif
        _mem4(&beta1Ptr[4]) = 0x81818181;
    }
    else if(Kt==1)
    {
        //Beta MAP0
        beta0 = 0;
        beta1 = _ext(bit54,16,16) + _ext(bit54,0,16); //xt0[0] + xt0[1];

        betaMax = _max2(beta0,beta1);
        betaMax = _ext(betaMax,16,16);
        betaMax = 127 - betaMax;

        beta0 = beta0 + betaMax;
        beta1 = beta1 + betaMax;

        beta0 = _max2(0x8000ff81, beta0);
        beta1 = _max2(0x8000ff81, beta1);

#ifdef _BIG_ENDIAN
        _mem4(&beta0Ptr[0]) = _packl4(_pack2(beta0,beta1),0x81818181);
#else
        _mem4(&beta0Ptr[0]) = _packl4(0x81818181,_pack2(beta1,beta0));
#endif
        _mem4(&beta0Ptr[4]) = 0x81818181;

        //Beta MAP1
        beta0 = 0;
        beta1 = _ext(bitba,16,16) + _ext(bitba,0,16); //xt1[0] + xt1[1];

        betaMax = _max2(beta0,beta1);
        betaMax = _ext(betaMax,16,16);
        betaMax = 127 - betaMax;

        beta0 = beta0 + betaMax;
        beta1 = beta1 + betaMax;

        beta0 = _max2(0x8000ff81, beta0);
        beta1 = _max2(0x8000ff81, beta1);

#ifdef _BIG_ENDIAN
        _mem4(&beta1Ptr[0]) = _packl4(_pack2(beta0,beta1),0x81818181);
#else
        _mem4(&beta1Ptr[0]) = _packl4(0x81818181,_pack2(beta1,beta0));
#endif
        _mem4(&beta1Ptr[4]) = 0x81818181;

    }
    else if(Kt==0)
    {
#ifdef _BIG_ENDIAN
        _mem4(&beta0Ptr[0]) = 0x7f818181;
        _mem4(&beta1Ptr[0]) = 0x7f818181;
#else
        _mem4(&beta0Ptr[0]) = 0x8181817f;
        _mem4(&beta1Ptr[0]) = 0x8181817f;
#endif
        _mem4(&beta0Ptr[4]) = 0x81818181;
        _mem4(&beta1Ptr[4]) = 0x81818181;
    }
}

/* nothing past this line */
