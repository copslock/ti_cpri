/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <math.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

#include "cslUtils.h"

#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif

#include "checkRfUtils.h"


//////////////////////////////////////////////////////////////////////////////
// References to external variables
//#pragma DATA_SECTION(tx0Tab,".iqn2txAxC0");
//Complex16 tx0Tab[];
//#pragma DATA_SECTION(tx1Tab,".iqn2txAxC1");
//Complex16 tx1Tab[];
//#pragma DATA_SECTION(tx2Tab,".iqn2txAxC2");
//Complex16 tx2Tab[];
//#pragma DATA_SECTION(tx3Tab,".iqn2txAxC3");
//Complex16 tx3Tab[];
//#pragma DATA_SECTION(tx4Tab,".iqn2txAxC4");
//Complex16 tx4Tab[];
//#pragma DATA_SECTION(tx5Tab,".iqn2txAxC5");
//Complex16 tx5Tab[];
//#pragma DATA_SECTION(tx6Tab,".iqn2txAxC6");
//Complex16 tx6Tab[];
//#pragma DATA_SECTION(tx7Tab,".iqn2txAxC7");
//Complex16 tx7Tab[];
//#pragma DATA_SECTION(tx8Tab,".iqn2txAxC8");
//Complex16 tx8Tab[];
//#pragma DATA_SECTION(tx9Tab,".iqn2txAxC9");
//Complex16 tx9Tab[];
//#pragma DATA_SECTION(tx10Tab,".iqn2txAxC10");
//Complex16 tx10Tab[];
//#pragma DATA_SECTION(tx11Tab,".iqn2txAxC11");
//Complex16 tx11Tab[];
//#pragma DATA_SECTION(tx12Tab,".iqn2txAxC12");
//Complex16 tx12Tab[];
//#pragma DATA_SECTION(tx13Tab,".iqn2txAxC13");
//Complex16 tx13Tab[];
//#pragma DATA_SECTION(tx14Tab,".iqn2txAxC14");
//Complex16 tx14Tab[];
//#pragma DATA_SECTION(tx15Tab,".iqn2txAxC15");
//Complex16 tx15Tab[];
//
//#pragma DATA_SECTION(rx0Tab,".iqn2rxAxC0");
//Complex16 rx0Tab[];
//#pragma DATA_SECTION(rx1Tab,".iqn2rxAxC1");
//Complex16 rx1Tab[];
//#pragma DATA_SECTION(rx2Tab,".iqn2rxAxC2");
//Complex16 rx2Tab[];
//#pragma DATA_SECTION(rx3Tab,".iqn2rxAxC3");
//Complex16 rx3Tab[];
//#pragma DATA_SECTION(rx4Tab,".iqn2rxAxC4");
//Complex16 rx4Tab[];
//#pragma DATA_SECTION(rx5Tab,".iqn2rxAxC5");
//Complex16 rx5Tab[];
//#pragma DATA_SECTION(rx6Tab,".iqn2rxAxC6");
//Complex16 rx6Tab[];
//#pragma DATA_SECTION(rx7Tab,".iqn2rxAxC7");
//Complex16 rx7Tab[];
//#pragma DATA_SECTION(rx8Tab,".iqn2rxAxC8");
//Complex16 rx8Tab[];
//#pragma DATA_SECTION(rx9Tab,".iqn2rxAxC9");
//Complex16 rx9Tab[];
//#pragma DATA_SECTION(rx10Tab,".iqn2rxAxC10");
//Complex16 rx10Tab[];
//#pragma DATA_SECTION(rx11Tab,".iqn2rxAxC11");
//Complex16 rx11Tab[];
//#pragma DATA_SECTION(rx12Tab,".iqn2rxAxC12");
//Complex16 rx12Tab[];
//#pragma DATA_SECTION(rx13Tab,".iqn2rxAxC13");
//Complex16 rx13Tab[];
//#pragma DATA_SECTION(rx14Tab,".iqn2rxAxC14");
//Complex16 rx14Tab[];
//#pragma DATA_SECTION(rx15Tab,".iqn2rxAxC15");
//Complex16 rx15Tab[];
//
Complex16 *txPtr[16]; // = &tx0Tab[0];
Complex16 *rxPtr[16]; // = &tx0Tab[0];


//void resetAxCPtr(){
//    txPtr[0] = &tx0Tab[0];
//    txPtr[1] = &tx1Tab[0];
//    txPtr[2] = &tx2Tab[0];
//    txPtr[3] = &tx3Tab[0];
//    txPtr[4] = &tx4Tab[0];
//    txPtr[5] = &tx5Tab[0];
//    txPtr[6] = &tx6Tab[0];
//    txPtr[7] = &tx7Tab[0];
//    txPtr[8] = &tx8Tab[0];
//    txPtr[9] = &tx9Tab[0];
//    txPtr[10] = &tx10Tab[0];
//    txPtr[11] = &tx11Tab[0];
//    txPtr[12] = &tx12Tab[0];
//    txPtr[13] = &tx13Tab[0];
//    txPtr[14] = &tx14Tab[0];
//    txPtr[15] = &tx15Tab[0];
//
//    rxPtr[0] = &rx0Tab[0];
//    rxPtr[1] = &rx1Tab[0];
//    rxPtr[2] = &rx2Tab[0];
//    rxPtr[3] = &rx3Tab[0];
//    rxPtr[4] = &rx4Tab[0];
//    rxPtr[5] = &rx5Tab[0];
//    rxPtr[6] = &rx6Tab[0];
//    rxPtr[7] = &rx7Tab[0];
//    rxPtr[8] = &rx8Tab[0];
//    rxPtr[9] = &rx9Tab[0];
//    rxPtr[10] = &rx10Tab[0];
//    rxPtr[11] = &rx11Tab[0];
//    rxPtr[12] = &rx12Tab[0];
//    rxPtr[13] = &rx13Tab[0];
//    rxPtr[14] = &rx14Tab[0];
//    rxPtr[15] = &rx15Tab[0];
//}
//
//void resetRxCaptBuf(){
//    memset((void*)&rx0Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx1Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx2Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx3Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx4Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx5Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx6Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx7Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx8Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx9Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx10Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx11Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx12Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx13Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx14Tab[0],0,20*(2048+160+(6*(2048+144))));
//    memset((void*)&rx15Tab[0],0,20*(2048+160+(6*(2048+144))));
//}

void getcomplex(Complex16* payload,uint32_t index,uint32_t tx, float sampFreq)
{
	float freq;
	Complex16 x1;
	Complex16 y;

#if (DFE_JESD_LOOPBACK == 1) || (DFE_NORMAL_OPERATION == 1)
    if (tx==0)
#else
    if ((tx==0)||(tx==2)||(tx==4)||(tx==6))
#endif
    {
                // 1 MHz signal
                freq = FREQ;
                x1.re = (int16_t) ( (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x1.im = (int16_t) ( (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                y = x1;
#if (DFE_JESD_LOOPBACK == 1) || (DFE_NORMAL_OPERATION == 1)
    } else if (tx==2)
#else
    } else if ((tx==1)|| (tx==3)||(tx==5)|| (tx==7))
#endif
    {
                Complex16         x2;
                // 1 MHz signal
                freq = FREQ;
                x1.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // 2 MHz signal
                freq = 2*FREQ;
                x2.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // dual tone signal
                y.re = x1.re + x2.re;
                y.im = x1.im + x2.im;
    } else {
                y.re = 0;
                y.im = 0;
    }
#if DUAL_TONE_FIVE_NINE == 1
                Complex16         x2;
                // 5 MHz signal
                freq = 5;
                x1.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // 9 MHz signal
                freq = 9;
                x2.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // dual tone signal
                y.re = x1.re + x2.re;
                y.im = x1.im + x2.im;

#elif QUAD_TONE_ONE_TWO == 1
                Complex16         x2,x3,x4;
                // 1 MHz signal
                freq = 1;
                x1.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.25 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // 2 MHz signal
                freq = 2;
                x2.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.25 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // -1 MHz signal
                freq = 1;
                x3.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x3.im = (int16_t) (-0.25 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // -2 MHz signal
                freq = 2;
                x4.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x4.im = (int16_t) (-0.25 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // quad tone signal
                y.re = x1.re + x2.re + x3.re + x4.re;
                y.im = x1.im + x2.im + x3.im + x4.im;

#elif QUAD_TONE_FIVE_NINE == 1
                Complex16         x2,x3,x4;
                // 5 MHz signal
                freq = 5;
                x1.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.25 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // 9 MHz signal
                freq = 9;
                x2.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.25 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // -5 MHz signal
                freq = 5;
                x3.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x3.im = (int16_t) (-0.25 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // -9 MHz signal
                freq = 9;
                x4.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                x4.im = (int16_t) (-0.25 * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                // quad tone signal
                y.re = x1.re + x2.re + x3.re + x4.re;
                y.im = x1.im + x2.im + x3.im + x4.im;

#elif EIGHTEEN_TONES == 1
                Complex16         x2;
                uint32_t            ii;
                float             scale;

                scale = 1.0f/18.0f;
                y.re = 0;
                y.im = 0;
                for (ii = 0; ii < 9; ii ++)
                {
                    // Plus MHz signal
                    freq = ii + 1;
                    x1.re = (int16_t) (scale * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    x1.im = (int16_t) (scale * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    // Minus MHz signal
                    x2.re = (int16_t) (scale * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    x2.im = (int16_t) (-scale * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    y.re = y.re + x1.re + x2.re;
                    y.im = y.im + x1.im + x2.im;
                }

#elif TWENTY_FOUR_TONES == 1
                Complex16         x2;
                uint32_t            ii;
                float             scale;

                scale = 1.0f/24.0f;
                y.re = 0;
                y.im = 0;
                for (ii = 0; ii < 12; ii ++)
                {
                    // Plus MHz signal
                    freq = ii + 1;
                    x1.re = (int16_t) (scale * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    x1.im = (int16_t) (scale * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    // Minus MHz signal
                    x2.re = (int16_t) (scale * (cos(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    x2.im = (int16_t) (-scale * (sin(2*PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    y.re = y.re + x1.re + x2.re;
                    y.im = y.im + x1.im + x2.im;
                }

#elif FORTY_EIGHT_TONES == 1
                Complex16         x2;
                uint32_t            ii;
                float             scale;

                scale = 1.0f/48.0f;
                y.re = 0;
                y.im = 0;
                for (ii = 0; ii < 24; ii ++)
                {
                    // Plus MHz signal
                    freq = ii + 1;
                    x1.re = (int16_t) (scale * (cos(PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    x1.im = (int16_t) (scale * (sin(PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    // Minus MHz signal
                    x2.re = (int16_t) (scale * (cos(PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    x2.im = (int16_t) (-scale * (sin(PI*freq*(1/sampFreq)*index)) * Q_FACTOR );
                    y.re = y.re + x1.re + x2.re;
                    y.im = y.im + x1.im + x2.im;
                }

#elif NINETY_SIX_TONES == 1
                Complex16         x2;
                uint32_t            ii;
                float             scale,freq1;

                scale = 1.0f/96.0f;
                y.re = 0;
                y.im = 0;
                for (ii = 0; ii < 48; ii ++)
                {
                    // Plus MHz signal
                    freq = ii + 1;
                    freq1 = (float)freq/2;
                    x1.re = (int16_t) (scale * (cos(PI*freq1*(1/sampFreq)*index)) * Q_FACTOR );
                    x1.im = (int16_t) (scale * (sin(PI*freq1*(1/sampFreq)*index)) * Q_FACTOR );
                    // Minus MHz signal
                    x2.re = (int16_t) (scale * (cos(PI*freq1*(1/sampFreq)*index)) * Q_FACTOR );
                    x2.im = (int16_t) (-scale * (sin(PI*freq1*(1/sampFreq)*index)) * Q_FACTOR );
                    y.re = y.re + x1.re + x2.re;
                    y.im = y.im + x1.im + x2.im;
                }
#endif
                payload->re = y.re;
                payload->im = y.im;
}


void getTMdata(Complex16* payload,uint32_t tx)
{
	Complex16 y;
    y.re = txPtr[tx]->re;
    y.im = txPtr[tx]->im;
    txPtr[tx]++;

    payload->re = y.re;
    payload->im = y.im;
}

void getRMdata(Complex16* payload,uint32_t rx)
{
    Complex16 y;
    y.re = rxPtr[rx]->re;
    y.im = rxPtr[rx]->im;
    rxPtr[rx]++;

    payload->re = y.re;
    payload->im = y.im;
}

////////////


